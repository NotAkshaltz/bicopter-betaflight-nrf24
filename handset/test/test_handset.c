// Host test: handset_app.c with a fake HAL whose SPI is decoded by an nRF24 command model.
// Every transmitted payload is decoded with the receiver's protocol core.
#include <stdio.h>
#include <string.h>
#include "../config/link_config.h"
#include "../shared/nrf24_bicopter_link.h"
#include "../src/handset_app.h"

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static uint32_t now;
static bool csnLow, ce;
static uint8_t frame[40]; static int flen;
static uint8_t regs[32], txAddr[5];
static int txCount; static uint8_t lastPkt[32]; static uint8_t lastCh[2000];
static bool newSample = true, frozen;
static uint16_t adcVal = 2048;
static uint16_t throttleAdc = 100;
static uint8_t armSwitch;
static bool txBusy, txWillAck = true;
static uint32_t txCompletesAt, txStartedAt, txDurationUs = 1500;
static bool haveTxStart;

static void completeTx(void) {
    if (txBusy && (int32_t)(now - txCompletesAt) >= 0) {
        txBusy = false;
        regs[7] |= txWillAck ? 0x20 : 0x10; // TX_DS or MAX_RT, not instant completion
    }
}

uint32_t hal_micros(void) { completeTx(); return now; }
void hal_delay_us(uint32_t us) { now += us; completeTx(); }
void hal_led(bool on) { (void)on; }
uint8_t hal_boot_random_nonzero(void) { return 0x77; }
bool hal_take_new_inputs(hsRawInputs_t *o) {
    if (!newSample || frozen) return false;
    newSample = false;
    for (int i = 0; i < 4; i++) o->analog[i] = adcVal;
    o->analog[2] = throttleAdc;
    for (int i = 0; i < 4; i++) o->sw[i] = (uint8_t)(i % 3);
    o->sw[0] = armSwitch;
    return true;
}
static void endFrame(void) {
    if (flen == 0) return;
    uint8_t c = frame[0];
    if (c == 0xE1 || c == 0xA0 || c == 0x25) CHECK(!txBusy); // no flush/load/retune while busy
    if ((c & 0xE0) == 0x20 && flen >= 2) { uint8_t r = c & 31; if (r == 0x10) memcpy(txAddr, &frame[1], 5); else if (r == 7) regs[7] &= (uint8_t)~(frame[1] & 0x70); else regs[r] = frame[1]; }
    if (c == 0xA0) { memcpy(lastPkt, &frame[1], flen - 1); }
    flen = 0;
}
void hal_nrf_csn(bool high) { if (high && csnLow) endFrame(); csnLow = !high; }
uint8_t hal_spi_xfer(uint8_t b) {
    hal_delay_us(1); // finite 8 MHz SPI bus, one microsecond per byte
    uint8_t ret = 0;
    if (flen == 1 && (frame[0] & 0xE0) == 0x00) ret = regs[frame[0] & 31];
    if (flen < 40) frame[flen++] = b;
    return ret;
}
void hal_nrf_ce(bool high) {
    completeTx();
    if (!ce && high) {
        CHECK(!txBusy);
        if (haveTxStart) CHECK((uint32_t)(now - txStartedAt) >= BL_FRAME_PERIOD_US);
        txStartedAt = now; haveTxStart = true;
        txBusy = true; txCompletesAt = now + txDurationUs;
        if (txCount < 2000) lastCh[txCount] = regs[5];
        txCount++;
        blControl_t d;
        CHECK(blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &d) == BL_DECODE_OK);
    }
    if (ce && !high) CHECK((uint32_t)(now - txStartedAt) >= 10);
    ce = high;
}

static void runSamplingCase(uint32_t samplePeriod)
{
    now = 0; csnLow = ce = false; flen = txCount = 0;
    txBusy = haveTxStart = false; txDurationUs = 1500; txWillAck = true;
    memset(regs, 0, sizeof(regs)); memset(lastCh, 0, sizeof(lastCh));
    newSample = true; frozen = false; adcVal = 2048; throttleAdc = 100; armSwitch = 0;
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) { cal[i].min = 100; cal[i].center = 2048; cal[i].max = 4000; cal[i].reverse = false; }
    CHECK(hs_init(cal));
    uint8_t a[5]; blDeriveAddress(HANDSET_LINK_ID, a);
    CHECK(memcmp(a, txAddr, 5) == 0);
    CHECK(regs[6] == (0x20 | HANDSET_RF_PWR_BITS));
    CHECK(regs[0] == 0x0E && regs[0x1C] == 1 && regs[0x1D] == 4 && regs[4] == 0x20);

    // Feed the RX freshness logic with what the handset sends
    blFreshness_t f; blFreshnessInit(&f);
    int accepted = 0, faultFrames = 0;
    uint8_t ch[4]; blDeriveRfChannels(HANDSET_LINK_ID, ch);
    int lastTx = 0;
    const uint32_t started = now;
    uint32_t nextSample = now;
    while ((uint32_t)(now - started) < 3000000) {
        hal_delay_us(100);
        if ((int32_t)(now - nextSample) >= 0) {
            newSample = true;
            do { nextSample += samplePeriod; } while ((int32_t)(now - nextSample) >= 0);
        }
        const uint32_t t = now - started;
        if (t >= 2000000) frozen = true; // first second announces boot, second carries controls
        hs_poll();
        if (txCount != lastTx) {
            lastTx = txCount;
            blControl_t d; blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &d);
            if (d.flags) faultFrames++;
            if (blFreshnessCheck(&f, d.session, d.sampleSeq, d.flags) == BL_FRESH_ACCEPT) accepted++;
            if (t < HANDSET_RESTART_ANNOUNCE_US) CHECK(d.flags & BL_FLAG_TX_RESTART);
            if (t > 2000000 + HANDSET_SAMPLE_TIMEOUT_US + 6000) CHECK(d.flags & BL_FLAG_TX_INPUT_FAULT);
            CHECK(d.channelUs[0] == 1500 && d.channelUs[4] == 1000 && d.channelUs[5] == 1500 && d.channelUs[6] == 2000);
        }
    }
    for (int i = 0; i < 400 && i < txCount; i++) CHECK(lastCh[i] == ch[i % 4]);
    printf("  acquisition period %u us: sent %d in 3 s, accepted %d, flagged %d\n", samplePeriod, txCount, accepted, faultFrames);
    CHECK(txCount >= 590 && txCount <= 601); // 5 ms minimum + foreground/SPI latency
    const int expected = samplePeriod > 5000 ? (int)(1000000U / samplePeriod) : 200;
    CHECK(accepted >= expected - 10 && accepted <= expected + 2);
    // Rail detection
    hsRawInputs_t in = { .analog = {2048, 2048, 2, 2048}, .sw = {0, 0, 0, 0} };
    uint16_t us[8];
    CHECK(!hs_inputs_to_channels(&in, cal, us));
    in.analog[2] = 2048;
    CHECK(hs_inputs_to_channels(&in, cal, us));
}

static void pollFor(uint32_t duration)
{
    const uint32_t started = now;
    uint32_t nextSample = now;
    while ((uint32_t)(now - started) < duration) {
        hal_delay_us(100);
        if ((int32_t)(now - nextSample) >= 0) {
            newSample = true;
            do { nextSample += 1000; } while ((int32_t)(now - nextSample) >= 0);
        }
        hs_poll();
    }
}

static void testBootWithArmHigh(void)
{
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) cal[i] = (hsStickCal_t){100, 2048, 4000, false};
    frozen = false; newSample = true; armSwitch = 2; throttleAdc = 3000;
    CHECK(hs_init(cal));
    pollFor(1500000);
    blControl_t c; CHECK(blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c) == BL_DECODE_OK);
    CHECK(c.flags & BL_FLAG_TX_RESTART); CHECK(c.channelUs[4] == 1000 && c.channelUs[2] == 1000);
    armSwitch = 0; pollFor(50000); // throttle high: remain locked
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c); CHECK(c.flags & BL_FLAG_TX_RESTART);
    throttleAdc = 100; pollFor(50000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c); CHECK(c.flags == 0 && c.channelUs[4] == 1000);
    armSwitch = 2; pollFor(50000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c); CHECK(c.flags == 0 && c.channelUs[4] == 2000);
    // The HAL returns the same boot session: the explicit restart flag still exists.
    CHECK(hs_init(cal)); pollFor(10000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c); CHECK(c.flags & BL_FLAG_TX_RESTART);
    CHECK(c.channelUs[4] == 1000 && c.channelUs[2] == 1000);
    cal[0].center = cal[0].min;
    CHECK(!hs_init(cal));
}

// In flight, one bad ADC sample or a late acquisition must only flag TX_INPUT_FAULT on the
// affected packets. It must never enter the restart lock (which makes the receiver disarm).
static void testInputFaultNoRestart(void)
{
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) cal[i] = (hsStickCal_t){100, 2048, 4000, false};
    frozen = false; newSample = true; armSwitch = 0; throttleAdc = 100; adcVal = 2048;
    CHECK(hs_init(cal)); pollFor(1500000);
    armSwitch = 2; throttleAdc = 2048; pollFor(50000);
    blControl_t c; blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c);
    CHECK(c.flags == 0 && c.channelUs[4] == 2000);
    const uint16_t flyingThrottle = c.channelUs[2];
    adcVal = 0; pollFor(1000); adcVal = 2048;                   // one bad sample
    pollFor(10000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c);
    CHECK(!(c.flags & BL_FLAG_TX_RESTART)); CHECK(c.flags == 0);
    CHECK(c.channelUs[4] == 2000 && c.channelUs[2] == flyingThrottle);
    frozen = true; pollFor(100000);                              // acquisition stalls 100 ms
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c);
    CHECK(c.flags == BL_FLAG_TX_INPUT_FAULT);
    frozen = false; pollFor(10000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c);
    CHECK(c.flags == 0 && c.channelUs[4] == 2000 && c.channelUs[2] == flyingThrottle);
}

static void testCalibratedEndpoints(void)
{
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) cal[i] = (hsStickCal_t){0, 2048, 4095, false};
    hsRawInputs_t in = { .analog = {2048, 2048, 2048, 2048} };
    uint16_t us[8];
    for (int axis = 0; axis < 4; axis++) {
        in.analog[axis] = 0;
        CHECK(hs_inputs_to_channels(&in, cal, us)); CHECK(us[axis] == 1000);
        in.analog[axis] = 4095;
        CHECK(hs_inputs_to_channels(&in, cal, us)); CHECK(us[axis] == 2000);
        cal[axis].reverse = true;
        CHECK(hs_inputs_to_channels(&in, cal, us)); CHECK(us[axis] == 1000);
        cal[axis].reverse = false; in.analog[axis] = 2048;
    }
    in.analog[0] = 4096; CHECK(!hs_inputs_to_channels(&in, cal, us));
    in.analog[0] = 2048; cal[0].min = 5; cal[0].max = 4090;
    in.analog[0] = 5; CHECK(hs_inputs_to_channels(&in, cal, us)); CHECK(us[0] == 1000);
    in.analog[0] = 4090; CHECK(hs_inputs_to_channels(&in, cal, us)); CHECK(us[0] == 2000);
    in.analog[0] = 0; CHECK(!hs_inputs_to_channels(&in, cal, us));
    in.analog[0] = 4095; CHECK(!hs_inputs_to_channels(&in, cal, us));

    // Actual application must unlock and transmit valid controls at a calibrated rail.
    cal[0].min = 0; cal[0].max = 4095;
    adcVal = 2048; throttleAdc = 0; armSwitch = 0; frozen = false; newSample = true;
    CHECK(hs_init(cal)); pollFor(1500000);
    blControl_t c; CHECK(blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c) == BL_DECODE_OK);
    CHECK(c.flags == 0 && c.channelUs[BL_THROTTLE_CHANNEL] == 1000);
    armSwitch = 2; throttleAdc = 4095; pollFor(10000);
    blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &c);
    CHECK(c.flags == 0 && c.channelUs[BL_ARM_CHANNEL] == 2000 && c.channelUs[BL_THROTTLE_CHANNEL] == 2000);
}

static void scheduleCase(uint32_t initialTime)
{
    now = initialTime; csnLow = ce = txBusy = haveTxStart = false;
    flen = txCount = 0; memset(regs, 0, sizeof(regs));
    txDurationUs = 1500; txWillAck = true;
    frozen = false; newSample = true; adcVal = 2048; throttleAdc = 100; armSwitch = 0;
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) cal[i] = (hsStickCal_t){100, 2048, 4000, false};
    CHECK(hs_init(cal)); pollFor(1500000);
    armSwitch = 2; throttleAdc = 2048; pollFor(10000);
    const uint32_t stalls[] = {9000, 20000, 100000};
    for (unsigned i = 0; i < sizeof(stalls) / sizeof(stalls[0]); i++) {
        hal_delay_us(stalls[i]); newSample = true;
        int before = txCount; hs_poll(); CHECK(txCount == before + 1);
        const uint32_t launch = txStartedAt;
        hal_delay_us(100); hs_poll(); CHECK(txCount == before + 1);
        pollFor(6000); CHECK((uint32_t)(txStartedAt - launch) >= BL_FRAME_PERIOD_US);
    }
    // A completion delayed past the normal slot must not be flushed or counted
    // as a failure while still pending. MAX_RT is counted once when it completes.
    hal_delay_us(6000); txDurationUs = 20000; txWillAck = false;
    hs_poll(); CHECK(txBusy);
    const int before = txCount;
    const uint32_t noAckBefore = hs_stats()->noAck;
    const uint8_t channel = regs[5];
    pollFor(10000);
    CHECK(txCount == before && txBusy && regs[5] == channel);
    CHECK(hs_stats()->noAck == noAckBefore);
    txDurationUs = 1500;
    hal_delay_us(10000); hs_poll();
    CHECK(txCount == before + 1 && hs_stats()->noAck == noAckBefore + 1);
    uint8_t channels[4]; blDeriveRfChannels(HANDSET_LINK_ID, channels);
    CHECK(lastCh[before] == channels[before % 4]);
    txWillAck = true;
}

static void testLateLoopAndBusyRadio(void)
{
    scheduleCase(0);
    scheduleCase(UINT32_MAX - 1630000); // normal operation and stalls cross wrap
}

int main(void)
{
    runSamplingCase(10000); // 100 Hz
    runSamplingCase(5000);  // 200 Hz
    runSamplingCase(1000);  // 1 kHz
    runSamplingCase(500);   // 2 kHz
    testBootWithArmHigh();
    testInputFaultNoRestart();
    testCalibratedEndpoints();
    testLateLoopAndBusyRadio();
    printf("%s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
