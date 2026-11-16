// Runs the REAL src/main/rx/nrf24_bicopter.c against a register-level nRF24L01+ model
// (RX FIFO of 3, RX_DR flag, active-low IRQ line with edge capture) and a simulated handset.
#include <string.h>

#include "platform.h"
#include "build/debug.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/rx/rx_spi.h"
#include "rx/nrf24_bicopter.h"
#include "rx/nrf24_bicopter_link.h"
#include "test_common.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = DEBUG_RX_NRF24_BICOPTER;

// ---------------- radio model ----------------
static struct {
    bool present, irqWired;
    uint8_t regs[32], rxAddr[5];
    bool rxDr, ce;
    uint32_t tunedAt;
    struct { uint8_t len; uint8_t d[32]; } fifo[3];
    int count;
    bool extiFlag; uint32_t extiTs;
    int ops;          // SPI transactions
} R;
static uint32_t nowUs;
static int receiverResetCalls;
void failsafeOnReceiverReset(void) { receiverResetCalls++; }

timeUs_t micros(void) { return nowUs; }
static void op(void) { R.ops++; }

void NRF24L01_Initialize(uint8_t base) { op(); R.regs[0] = base | 0x02; R.ce = false; }
void NRF24L01_WriteReg(uint8_t reg, uint8_t v) {
    op(); if (!R.present) return;
    if (reg == NRF24L01_07_STATUS) { if (v & 0x40) R.rxDr = false; return; }
    R.regs[reg & 31] = v;
}
void NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *d, uint8_t n) { op(); if (R.present && reg == NRF24L01_0A_RX_ADDR_P0) memcpy(R.rxAddr, d, n); }
uint8_t NRF24L01_ReadReg(uint8_t reg) {
    op(); if (!R.present) return 0xFF;
    if (reg == NRF24L01_07_STATUS) return (uint8_t)((R.rxDr ? 0x40 : 0) | ((R.count ? 0 : 7) << 1));
    if (reg == NRF24L01_17_FIFO_STATUS) return (uint8_t)((R.count == 0) | ((R.count == 3) << 1) | 0x10);
    return R.regs[reg & 31];
}
void NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *d, uint8_t n) { op(); if (reg == NRF24L01_0A_RX_ADDR_P0) memcpy(d, R.present ? R.rxAddr : (uint8_t[5]){0xFF,0xFF,0xFF,0xFF,0xFF}, n); }
uint8_t NRF24L01_ReadPayloadWidth(void) { op(); return R.count ? R.fifo[0].len : 0; }
void NRF24L01_ReadPayload(uint8_t *d, uint8_t n) { op(); memcpy(d, R.fifo[0].d, n); memmove(&R.fifo[0], &R.fifo[1], sizeof(R.fifo[0]) * 2); R.count--; }
void NRF24L01_FlushRx(void) { op(); R.count = 0; }
void NRF24L01_FlushTx(void) { op(); }
void NRF24L01_Activate(uint8_t c) { op(); (void)c; }
void NRF24L01_SetRxMode(void) { op(); R.regs[0] |= 1; R.rxDr = false; R.ce = true; R.tunedAt = nowUs; }
void NRF24L01_SetChannelRxMode(uint8_t ch) { op(); R.regs[NRF24L01_05_RF_CH] = ch; R.ce = true; R.tunedAt = nowUs; }

bool rxSpiExtiConfigured(void) { return R.irqWired; }
bool rxSpiGetExtiState(void) { return !R.rxDr; }      // line high unless RX_DR asserted
bool rxSpiPollExti(void) { return R.extiFlag; }
void rxSpiResetExti(void) { R.extiFlag = false; }
timeUs_t rxSpiGetLastExtiTimeUs(void) { return R.extiTs; }

static void radioDeliver(const uint8_t *d, uint8_t len, uint8_t rfch, uint32_t t)
{
    CHECK((int32_t)(nowUs - t) >= 0); // t is packet completion, never a future event
    // Stay on the channel for RX settling plus the entire modeled 1 ms packet.
    if (!R.ce || R.regs[NRF24L01_05_RF_CH] != rfch || (int32_t)(t - R.tunedAt) < 1130) return;
    if (R.count == 3) return;                 // FIFO full: radio drops the packet
    R.fifo[R.count].len = len; memcpy(R.fifo[R.count].d, d, len > 32 ? 32 : len); R.count++;
    if (!R.rxDr) { R.rxDr = true; if (R.irqWired) { R.extiFlag = true; R.extiTs = t; } }
}

// ---------------- harness ----------------
static uint8_t payload[RX_SPI_MAX_PAYLOAD_SIZE];
static rxRuntimeState_t rs;
static rxSpiExtiConfig_t exti;

static bool initRx(bool irq, bool present)
{
    memset(&R, 0, sizeof(R)); R.present = present; R.irqWired = irq;
    receiverResetCalls = 0;
    rxSpiConfig_t cfg = {0};
    return nrf24BicopterInit(&cfg, &rs, &exti);
}

typedef struct { uint8_t session; uint16_t seq; bool freezeSeq; uint8_t flags; uint32_t linkId; int lossPct; } txSim_t;

// Run for durationUs; RX called every loopUs. Returns number of DATA results; tracks max RX gap.
static int run(txSim_t *tx, uint32_t durationUs, uint32_t loopUs, uint32_t *k, double *nextTx, uint32_t *maxGapUs, uint32_t *lastData, int *maxOps, bool txOn)
{
    uint8_t ch[4]; blDeriveRfChannels(BICOPTER_NRF24_LINK_ID, ch);
    int data = 0;
    uint32_t end = nowUs + durationUs;
    for (; (int32_t)(end - nowUs) > 0; nowUs += loopUs) {
        while (*nextTx + 1000 <= nowUs) {
            if (txOn && (int)(rng32() % 100) >= tx->lossPct) {
                blControl_t c; memset(&c, 0, sizeof(c));
                c.session = tx->session; c.sampleSeq = tx->seq; c.flags = tx->flags;
                for (int i = 0; i < 8; i++) c.channelUs[i] = (uint16_t)(1000 + (tx->seq + i * 100) % 1001);
                c.channelUs[BL_THROTTLE_CHANNEL] = 1000;
                c.channelUs[BL_ARM_CHANNEL] = 1000;
                uint8_t buf[20]; blEncode(tx->linkId, &c, buf);
                radioDeliver(buf, 20, ch[*k % 4], (uint32_t)*nextTx + 1000);
            }
            if (!tx->freezeSeq) tx->seq++;
            (*k)++; *nextTx += BL_FRAME_PERIOD_US;
        }
        R.ops = 0;
        if (nrf24BicopterDataReceived(payload) == RX_SPI_RECEIVED_DATA) {
            CHECK((int32_t)(nowUs - nrf24BicopterFrameTimeUs()) >= 0);
            data++;
            uint32_t gap = nowUs - *lastData;
            if (*lastData && gap > *maxGapUs) *maxGapUs = gap;
            *lastData = nowUs;
            uint16_t rc[18] = {0}; nrf24BicopterSetRcDataFromPayload(rc, payload);
            CHECK(rc[0] >= 1000 && rc[0] <= 2000);
        }
        if (R.ops > *maxOps) *maxOps = R.ops;
    }
    return data;
}

static void testInit(void)
{
    CHECK(initRx(true, true));
    CHECK_EQ(rs.channelCount, 8);
    CHECK_EQ(exti.trigger, BETAFLIGHT_EXTI_TRIGGER_FALLING);
    CHECK_EQ(R.regs[NRF24L01_06_RF_SETUP], 0x22);     // 250 kbps, -12 dBm (ACK only)
    CHECK_EQ(R.regs[NRF24L01_01_EN_AA], 0x01);
    CHECK_EQ(R.regs[NRF24L01_1C_DYNPD], 0x01);
    CHECK_EQ(R.regs[NRF24L01_1D_FEATURE], 0x04);
    CHECK_EQ(R.regs[NRF24L01_03_SETUP_AW], 0x03);
    uint8_t a[5]; blDeriveAddress(BICOPTER_NRF24_LINK_ID, a);
    CHECK(memcmp(a, R.rxAddr, 5) == 0);
    CHECK(!initRx(true, false));                       // radio absent / miswired -> RX disabled
    CHECK(debug[7] != 0);
}

static void scenario(bool irq, uint32_t loopUs, const char *name)
{
    nowUs = 1000;
    CHECK(initRx(irq, true));
    txSim_t tx = { .session = 0x42, .seq = 100, .linkId = BICOPTER_NRF24_LINK_ID };
    uint32_t k = 0, gap = 0, last = 0; double next = nowUs + 3333; int maxOps = 0;
    int d = run(&tx, 3000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    printf("  [%s] 3 s clean: %d fresh frames of %u sent, max gap %.1f ms, max SPI ops/call %d\n", name, d, k, gap / 1000.0, maxOps);
    CHECK(d > (int)k - 15);
    CHECK(gap < 20000);
    CHECK(maxOps <= 12);

    // Frozen acquisition: transport continues, sequence frozen -> no fresh frames at all
    tx.freezeSeq = true;
    int frozen = run(&tx, 500000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    CHECK(frozen <= 1);   // only the sample taken just before the freeze
    tx.freezeSeq = false;
    // Handset input fault flag -> no fresh frames
    tx.flags = BL_FLAG_TX_INPUT_FAULT;
    CHECK_EQ(run(&tx, 300000, loopUs, &k, &next, &gap, &last, &maxOps, true), 0);
    tx.flags = 0;
    // Foreign handset (different link ID) on the same channels -> nothing accepted
    tx.linkId = 0x11223344U;
    CHECK_EQ(run(&tx, 300000, loopUs, &k, &next, &gap, &last, &maxOps, true), 0);
    tx.linkId = BICOPTER_NRF24_LINK_ID;
    // Recovery after these
    last = 0; gap = 0;
    uint32_t t0 = nowUs;
    d = run(&tx, 1000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    CHECK(d > 150);
    // Link loss 2 s then handset restart (new session, seq from 0)
    run(&tx, 2000000, loopUs, &k, &next, &gap, &last, &maxOps, false);
    tx.session = 0x43; tx.seq = 0;
    last = 0; gap = 0; t0 = nowUs;
    uint32_t firstAfter = 0;
    for (int i = 0; i < 400 && !firstAfter; i++) {
        if (run(&tx, 1000, loopUs, &k, &next, &gap, &last, &maxOps, true)) firstAfter = nowUs - t0;
    }
    printf("  [%s] handset restart after 2 s loss: first fresh frame after %.1f ms\n", name, firstAfter / 1000.0);
    CHECK(firstAfter != 0 && firstAfter < 120000);
    // 25% loss: stream continues, no gap reaching the 150 ms RXLOSS threshold
    tx.lossPct = 25; gap = 0;
    d = run(&tx, 5000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    printf("  [%s] 25%% loss: %d fresh frames in 5 s, max gap %.1f ms\n", name, d, gap / 1000.0);
    CHECK(gap < 150000);
    const nrf24BicopterStats_t *st = nrf24BicopterGetStats();
    printf("  [%s] stats: acc %u malformed %u dup %u stale %u pend %u fault %u spurious %u flush %u\n", name,
        st->accepted, st->malformed, st->duplicate, st->stale, st->resyncPending, st->txFault, st->spurious, st->fifoFlush);
}

static void testCorruptWidthAndBacklog(void)
{
    nowUs = 5000;
    CHECK(initRx(true, true));
    // Corrupt width (>32) -> FIFO flushed, nothing accepted
    R.ce = true; R.count = 1; R.fifo[0].len = 40; R.rxDr = true; R.extiFlag = true; R.extiTs = nowUs;
    CHECK_EQ(nrf24BicopterDataReceived(payload), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(R.count, 0);
    CHECK_EQ(nrf24BicopterGetStats()->fifoFlush, 1);
    // Three packets queued behind one edge: read one per call, all consumed, two accepted
    uint8_t ch[4]; blDeriveRfChannels(BICOPTER_NRF24_LINK_ID, ch);
    for (int i = 0; i < 3; i++) {
        blControl_t c; memset(&c, 0, sizeof(c)); c.session = 9; c.sampleSeq = (uint16_t)(10 + i);
        for (int j = 0; j < 8; j++) c.channelUs[j] = 1500;
        uint8_t b[20]; blEncode(BICOPTER_NRF24_LINK_ID, &c, b);
        R.tunedAt = 0; radioDeliver(b, 20, R.regs[NRF24L01_05_RF_CH], nowUs);
    }
    CHECK_EQ(R.count, 3);
    int got = 0;
    for (int i = 0; i < 3; i++) { nowUs += 100; got += nrf24BicopterDataReceived(payload) == RX_SPI_RECEIVED_DATA; CHECK_EQ(R.count, 2 - i); }
    CHECK_EQ(got, 2);    // first packet of a new session is the re-anchor candidate
    // Spurious: IRQ line low with empty FIFO -> one check, then back-off
    R.rxDr = true; int ops = 0;
    for (int i = 0; i < 5; i++) { R.ops = 0; nowUs += 100; nrf24BicopterDataReceived(payload); ops += R.ops; }
    CHECK(ops <= 3);
}

static int deliverControl(uint8_t session, uint16_t seq, uint8_t flags, uint16_t arm, uint16_t throttle, uint32_t at)
{
    nowUs = at;
    blControl_t c = {0};
    c.session = session; c.sampleSeq = seq; c.flags = flags;
    for (int i = 0; i < 8; i++) c.channelUs[i] = 1500;
    c.channelUs[BL_ARM_CHANNEL] = arm; c.channelUs[BL_THROTTLE_CHANNEL] = throttle;
    uint8_t packet[BL_PAYLOAD_LEN];
    CHECK(blEncode(BICOPTER_NRF24_LINK_ID, &c, packet));
    radioDeliver(packet, sizeof(packet), R.regs[NRF24L01_05_RF_CH], nowUs);
    return nrf24BicopterDataReceived(payload);
}

static void testRestartInterlock(void)
{
    nowUs = 1000; CHECK(initRx(true, true));
    CHECK_EQ(deliverControl(1, 95, 0, 1000, 1000, 10000), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(deliverControl(1, 100, 0, 1000, 1000, 15000), RX_SPI_RECEIVED_DATA);
    CHECK_EQ(deliverControl(1, 105, 0, 2000, 1700, 20000), RX_SPI_RECEIVED_DATA);
    CHECK_EQ(receiverResetCalls, 0);
    // New session after just 5ms must request immediate Betaflight failsafe/disarm.
    CHECK_EQ(deliverControl(2, 0, 0, 2000, 1700, 25000), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(receiverResetCalls, 1);
    CHECK_EQ(deliverControl(2, 5, 0, 2000, 1700, 30000), RX_SPI_RECEIVED_DATA);
    uint16_t rc[18] = {0}; nrf24BicopterSetRcDataFromPayload(rc, payload);
    CHECK_EQ(rc[BL_ARM_CHANNEL], 1000); CHECK_EQ(rc[BL_THROTTLE_CHANNEL], 1000);
    CHECK_EQ(deliverControl(2, 10, 0, 2000, 1700, 35000), RX_SPI_RECEIVED_DATA);
    nrf24BicopterSetRcDataFromPayload(rc, payload); CHECK_EQ(rc[BL_ARM_CHANNEL], 1000);
    // ARM off alone is insufficient while throttle remains high.
    CHECK_EQ(deliverControl(2, 15, 0, 1000, 1700, 40000), RX_SPI_RECEIVED_DATA);
    CHECK_EQ(deliverControl(2, 20, 0, 2000, 1700, 45000), RX_SPI_RECEIVED_DATA);
    nrf24BicopterSetRcDataFromPayload(rc, payload); CHECK_EQ(rc[BL_ARM_CHANNEL], 1000);
    CHECK_EQ(deliverControl(2, 25, 0, 1000, 1000, 50000), RX_SPI_RECEIVED_DATA);
    CHECK_EQ(deliverControl(2, 30, 0, 2000, 1000, 55000), RX_SPI_RECEIVED_DATA);
    nrf24BicopterSetRcDataFromPayload(rc, payload); CHECK_EQ(rc[BL_ARM_CHANNEL], 2000);
    // Explicit restart works even when the random session byte is reused.
    CHECK_EQ(deliverControl(2, 0, BL_FLAG_TX_RESTART, 1000, 1000, 60000), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(receiverResetCalls, 2);
    CHECK_EQ(deliverControl(2, 5, BL_FLAG_TX_RESTART, 1000, 1000, 65000), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(receiverResetCalls, 2);
    CHECK_EQ(deliverControl(2, 10, 0, 2000, 1700, 70000), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(deliverControl(2, 15, 0, 2000, 1700, 75000), RX_SPI_RECEIVED_DATA);
    nrf24BicopterSetRcDataFromPayload(rc, payload); CHECK_EQ(rc[BL_ARM_CHANNEL], 1000);
}

// Armed and flying, then an RF gap (RX task still called, no packets), then the same
// session continues. RF loss belongs to Betaflight's failsafe: the receiver must pass the
// pilot's ARM/throttle straight through when the link returns, whatever the gap length.
static void dropoutCase(uint32_t gapUs)
{
    nowUs = 1000; CHECK(initRx(true, true));
    uint16_t seq = 100; uint32_t t = 10000;
    deliverControl(1, seq, 0, 1000, 1000, t);
    for (int i = 0; i < 3; i++) { seq++; t += 5000; deliverControl(1, seq, 0, 1000, 1000, t); }
    for (int i = 0; i < 40; i++) { seq++; t += 5000; deliverControl(1, seq, 0, 2000, 1500, t); }
    uint16_t rc[18] = {0}; nrf24BicopterSetRcDataFromPayload(rc, payload);
    CHECK_EQ(rc[BL_ARM_CHANNEL], 2000);
    for (uint32_t g = 1000; g < gapUs; g += 1000) { nowUs = t + g; nrf24BicopterDataReceived(payload); }
    seq += (uint16_t)(gapUs / 5000); t += gapUs;
    int got = 0;
    for (int i = 0; i < 20; i++) { seq++; t += 5000; if (deliverControl(1, seq, 0, 2000, 1500, t) == RX_SPI_RECEIVED_DATA) got++; }
    CHECK(got >= 18);
    nrf24BicopterSetRcDataFromPayload(rc, payload);
    CHECK_EQ(rc[BL_ARM_CHANNEL], 2000); CHECK_EQ(rc[BL_THROTTLE_CHANNEL], 1500);
    CHECK_EQ(receiverResetCalls, 0);
}

static void testDropoutKeepsControl(void)
{
    dropoutCase(100000); dropoutCase(160000); dropoutCase(300000); dropoutCase(1000000); dropoutCase(3000000);
}

// A single (or a short burst of) TX_INPUT_FAULT packet(s) in flight is dropped; the next
// good sample is accepted with ARM/throttle untouched and no failsafe reset.
static void testInputFaultKeepsControl(void)
{
    nowUs = 1000; CHECK(initRx(true, true));
    uint16_t seq = 10; uint32_t t = 10000;
    deliverControl(3, seq, 0, 1000, 1000, t);
    for (int i = 0; i < 3; i++) { seq++; t += 5000; deliverControl(3, seq, 0, 1000, 1000, t); }
    for (int i = 0; i < 10; i++) { seq++; t += 5000; CHECK_EQ(deliverControl(3, seq, 0, 2000, 1500, t), RX_SPI_RECEIVED_DATA); }
    for (int i = 0; i < 3; i++) { seq++; t += 5000; CHECK_EQ(deliverControl(3, seq, BL_FLAG_TX_INPUT_FAULT, 2000, 1500, t), RX_SPI_RECEIVED_NONE); }
    seq++; t += 5000; CHECK_EQ(deliverControl(3, seq, 0, 2000, 1500, t), RX_SPI_RECEIVED_DATA);
    uint16_t rc[18] = {0}; nrf24BicopterSetRcDataFromPayload(rc, payload);
    CHECK_EQ(rc[BL_ARM_CHANNEL], 2000); CHECK_EQ(rc[BL_THROTTLE_CHANNEL], 1500);
    CHECK_EQ(receiverResetCalls, 0);
}

static void queuedTimestampCase(uint32_t base, bool irq)
{
    nowUs = base + 1000; CHECK(initRx(irq, true));
    // The receiver stalls on one channel. The handset returns every 20 ms;
    // all three packets share the first falling IRQ edge, not the last arrival.
    for (int i = 0; i < 3; i++) {
        blControl_t c = { .session = 1, .sampleSeq = (uint16_t)(100 + i) };
        for (int ch = 0; ch < 8; ch++) c.channelUs[ch] = 1500;
        c.channelUs[BL_ARM_CHANNEL] = c.channelUs[BL_THROTTLE_CHANNEL] = 1000;
        uint8_t p[BL_PAYLOAD_LEN]; CHECK(blEncode(BICOPTER_NRF24_LINK_ID, &c, p));
        nowUs = base + 10000 + 20000 * i;
        radioDeliver(p, sizeof(p), R.regs[NRF24L01_05_RF_CH], nowUs);
    }
    CHECK_EQ(R.count, 3);
    if (irq) CHECK_EQ(R.extiTs, base + 10000);
    uint32_t previous = 0; int accepted = 0;
    for (int i = 0; i < 3; i++) {
        nowUs = base + 60000 + 250 * i;
        if (nrf24BicopterDataReceived(payload) == RX_SPI_RECEIVED_DATA) {
            const uint32_t frame = nrf24BicopterFrameTimeUs();
            CHECK_EQ(frame, nowUs); // ambiguous backlog uses service time
            CHECK((int32_t)(nowUs - frame) >= 0);
            if (accepted) CHECK((int32_t)(frame - previous) >= 0);
            previous = frame; accepted++;
        }
        CHECK_EQ((uint16_t)debug[4] >> 8, BL_HOP_SYNCED);
    }
    CHECK_EQ(accepted, 2); CHECK_EQ(R.count, 0);
}

static void testQueuedPacketTimestamps(void)
{
    queuedTimestampCase(0, true);
    queuedTimestampCase(UINT32_MAX - 60000, true); // backlog drains across timer wrap
    queuedTimestampCase(0, false);
    queuedTimestampCase(UINT32_MAX - 60000, false);
}

int main(void)
{
    RUN(testInit);
    RUN(testCorruptWidthAndBacklog);
    RUN(testRestartInterlock);
    RUN(testDropoutKeepsControl);
    RUN(testInputFaultKeepsControl);
    RUN(testQueuedPacketTimestamps);
    scenario(true, 250, "IRQ, 4 kHz loop");
    scenario(true, 1000, "IRQ, 1 kHz loop");
    scenario(false, 250, "polled, 4 kHz loop");
    SUMMARY();
}
