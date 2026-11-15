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

uint32_t hal_micros(void) { return now; }
void hal_delay_us(uint32_t us) { now += us; }
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
    if ((c & 0xE0) == 0x20 && flen >= 2) { uint8_t r = c & 31; if (r == 0x10) memcpy(txAddr, &frame[1], 5); else if (r == 7) regs[7] &= (uint8_t)~(frame[1] & 0x70); else regs[r] = frame[1]; }
    if (c == 0xA0) { memcpy(lastPkt, &frame[1], flen - 1); }
    flen = 0;
}
void hal_nrf_csn(bool high) { if (high && csnLow) endFrame(); csnLow = !high; }
uint8_t hal_spi_xfer(uint8_t b) {
    uint8_t ret = 0;
    if (flen == 1 && (frame[0] & 0xE0) == 0x00) ret = regs[frame[0] & 31];
    if (flen < 40) frame[flen++] = b;
    return ret;
}
void hal_nrf_ce(bool high) {
    if (ce && !high) { // falling edge of a CE pulse after W_TX_PAYLOAD -> one transmission
        if (txCount < 2000) lastCh[txCount] = regs[5];
        txCount++; regs[7] |= 0x20;
        blControl_t d;
        CHECK(blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &d) == BL_DECODE_OK);
    }
    ce = high;
}

static void runSamplingCase(uint32_t samplePeriod)
{
    now = 0; csnLow = ce = false; flen = txCount = 0;
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
    for (uint32_t t = 0; t < 3000000; t += 100) {
        now += 100;
        if (t % samplePeriod == 0) newSample = true;
        if (t == 2000000) frozen = true; // first second announces boot, second carries controls
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
    CHECK(txCount >= 598 && txCount <= 603);
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
    for (uint32_t t = 0; t < duration; t += 100) {
        now += 100;
        if (t % 1000 == 0) newSample = true;
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

int main(void)
{
    runSamplingCase(10000); // 100 Hz
    runSamplingCase(5000);  // 200 Hz
    runSamplingCase(1000);  // 1 kHz
    runSamplingCase(500);   // 2 kHz
    testBootWithArmHigh();
    printf("%s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
