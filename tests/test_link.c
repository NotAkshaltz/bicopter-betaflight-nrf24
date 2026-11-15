// Unit tests for the hardware independent NRF24_BICOPTER protocol core.
#include <string.h>

#include "rx/nrf24_bicopter_link.h"
#include "test_common.h"

#define LINK 0xBE5336DFU

static blControl_t makeControl(uint8_t session, uint16_t seq)
{
    blControl_t c;
    memset(&c, 0, sizeof(c));
    c.session = session;
    c.sampleSeq = seq;
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) {
        c.channelUs[i] = (uint16_t)(1000 + (i * 137) % 1001);
    }
    return c;
}

static void testRoundTrip(void)
{
    uint8_t buf[BL_PAYLOAD_LEN];
    for (int iter = 0; iter < 20000; iter++) {
        blControl_t c = makeControl((uint8_t)(1 + rng32() % 255), (uint16_t)rng32());
        for (int i = 0; i < BL_CHANNEL_COUNT; i++) {
            c.channelUs[i] = (uint16_t)(1000 + rng32() % 1001);
        }
        c.flags = (uint8_t)(rng32() & BL_FLAGS_DEFINED_MASK);
        CHECK(blEncode(LINK, &c, buf));
        blControl_t d;
        CHECK_EQ(blDecode(LINK, buf, BL_PAYLOAD_LEN, &d), BL_DECODE_OK);
        CHECK(memcmp(&c.channelUs, &d.channelUs, sizeof(c.channelUs)) == 0);
        CHECK_EQ(d.session, c.session);
        CHECK_EQ(d.sampleSeq, c.sampleSeq);
        CHECK_EQ(d.flags, c.flags);
        if (g_failures) return;
    }
    // extremes
    blControl_t c = makeControl(1, 0);
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) c.channelUs[i] = (i & 1) ? 2000 : 1000;
    CHECK(blEncode(LINK, &c, buf));
    blControl_t d;
    CHECK_EQ(blDecode(LINK, buf, BL_PAYLOAD_LEN, &d), BL_DECODE_OK);
    CHECK(memcmp(&c.channelUs, &d.channelUs, sizeof(c.channelUs)) == 0);
}

static void testKnownVector(void)
{
    // Fixed vector so the handset implementation can be checked byte-for-byte.
    blControl_t c = makeControl(0x5A, 0x1234);
    uint8_t buf[BL_PAYLOAD_LEN];
    CHECK(blEncode(LINK, &c, buf));
    printf("  vector(link=0x%08X): ", LINK);
    for (int i = 0; i < BL_PAYLOAD_LEN; i++) printf("%02X", buf[i]);
    printf("\n");
    CHECK_EQ(buf[0], 0xB1);
    CHECK_EQ(buf[1], 2);
    CHECK_EQ(buf[4], 0x5A);
    CHECK_EQ(buf[5], 0x34);
    CHECK_EQ(buf[6], 0x12);
    const uint8_t expected[BL_PAYLOAD_LEN] = {
        0xB1,0x02,0x34,0x09,0x5A,0x34,0x12,0x00,0x24,0x22,
        0xD1,0x66,0x24,0xB6,0x6A,0xF3,0xEF,0x00,0x2A,0x89
    };
    CHECK(memcmp(buf, expected, sizeof(expected)) == 0);
    // CRC-16/CCITT-FALSE check value for "123456789" with seed 0xFFFF is 0x29B1
    CHECK_EQ(blCrc16(0xFFFF, (const uint8_t *)"123456789", 9), 0x29B1);
}

static void testEncodeRejects(void)
{
    uint8_t buf[BL_PAYLOAD_LEN];
    blControl_t c = makeControl(1, 1);
    c.channelUs[3] = 999;
    CHECK(!blEncode(LINK, &c, buf));
    c.channelUs[3] = 2001;
    CHECK(!blEncode(LINK, &c, buf));
    c = makeControl(0, 1);
    CHECK(!blEncode(LINK, &c, buf));
    c = makeControl(1, 1);
    c.flags = 0x04;
    CHECK(!blEncode(LINK, &c, buf));
}

static void testDecodeRejects(void)
{
    uint8_t buf[BL_PAYLOAD_LEN], bad[32];
    blControl_t c = makeControl(7, 99);
    CHECK(blEncode(LINK, &c, buf));

    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[20] = 0;
    CHECK_EQ(blDecode(LINK, bad, 19, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 21, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 32, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 0, NULL), BL_DECODE_BAD_LENGTH);

    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[0] = 0xAE;   // e.g. an INAV bind packet
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_TYPE);
    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[1] = 3;
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_VERSION);

    // A packet from a handset built with a different link ID is rejected
    CHECK(blDecode(LINK ^ 1, buf, BL_PAYLOAD_LEN, NULL) != BL_DECODE_OK);
    CHECK(blDecode(0x12345678U, buf, BL_PAYLOAD_LEN, NULL) != BL_DECODE_OK);

    // Every single bit error is detected
    int undetected = 0;
    for (int bit = 0; bit < BL_PAYLOAD_LEN * 8; bit++) {
        memcpy(bad, buf, BL_PAYLOAD_LEN);
        bad[bit / 8] ^= (uint8_t)(1 << (bit % 8));
        if (blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL) == BL_DECODE_OK) undetected++;
    }
    CHECK_EQ(undetected, 0);
    // Random corruption of several bytes
    undetected = 0;
    for (int iter = 0; iter < 200000; iter++) {
        memcpy(bad, buf, BL_PAYLOAD_LEN);
        int n = 2 + rng32() % 4;
        for (int k = 0; k < n; k++) bad[rng32() % BL_PAYLOAD_LEN] ^= (uint8_t)(1 + rng32() % 255);
        if (memcmp(bad, buf, BL_PAYLOAD_LEN) != 0 && blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL) == BL_DECODE_OK) undetected++;
    }
    printf("  random multi-byte corruptions accepted: %d / 200000\n", undetected);
    CHECK(undetected <= 10);

    // Out-of-range channel with a valid CRC (crafted) is rejected
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[7] |= 0xFF; bad[8] |= 0x03;   // channel 0 = 1023
    uint16_t crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_RANGE);

    // Reserved flag bit with valid CRC
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[17] = 0x80;
    crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_FLAGS);

    // Session 0 with valid CRC
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[4] = 0;
    crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_SESSION);
}

static void testDerivation(void)
{
    for (int iter = 0; iter < 100000; iter++) {
        uint32_t id = iter < 3 ? (uint32_t[]){0, 0xFFFFFFFFU, LINK}[iter] : (uint32_t)rng32() ^ ((uint32_t)rng32() << 16);
        uint8_t a[BL_ADDR_LEN], a2[BL_ADDR_LEN], ch[BL_RF_CHANNEL_COUNT];
        blDeriveAddress(id, a);
        blDeriveAddress(id, a2);
        CHECK(memcmp(a, a2, BL_ADDR_LEN) == 0);
        for (int i = 0; i < BL_ADDR_LEN; i++) {
            CHECK(a[i] != 0x00 && a[i] != 0xFF && a[i] != 0x55 && a[i] != 0xAA);
        }
        blDeriveRfChannels(id, ch);
        for (int i = 0; i < BL_RF_CHANNEL_COUNT; i++) {
            CHECK(ch[i] >= 4 && ch[i] <= 80);
            if (i) CHECK_EQ(ch[i] - ch[i - 1], BL_RF_CHANNEL_SPACING);
        }
        CHECK(blCrcSeed(id) != 0);
        if (g_failures) return;
    }
    uint8_t a[5], ch[4];
    blDeriveAddress(LINK, a);
    blDeriveRfChannels(LINK, ch);
    printf("  link 0x%08X: address %02X %02X %02X %02X %02X, RF_CH %d %d %d %d, tag 0x%04X, crcSeed 0x%04X\n",
           LINK, a[0], a[1], a[2], a[3], a[4], ch[0], ch[1], ch[2], ch[3], blLinkTag(LINK), blCrcSeed(LINK));
}

static void testFreshness(void)
{
    blFreshness_t f;
    blFreshnessInit(&f);
    // ADC sampling can be faster than transmission: skipped sample numbers are valid.
    CHECK_EQ(blFreshnessCheck(&f, 10, 500, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 10, 500, 0), BL_FRESH_RESYNC_PENDING);   // duplicate of candidate
    CHECK_EQ(blFreshnessCheck(&f, 10, 505, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 10, 510, 0), BL_FRESH_ACCEPT);
    // Duplicate (frozen acquisition / repeated sample)
    for (int i = 0; i < 1000; i++) CHECK_EQ(blFreshnessCheck(&f, 10, 510, 0), BL_FRESH_DUPLICATE);
    // Lost packets within the window are fine
    CHECK_EQ(blFreshnessCheck(&f, 10, 520, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 10, 1520, 0), BL_FRESH_ACCEPT);
    uint16_t s = 1520;
    // Stale
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s - 1), 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s - 100), 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s - 99), 0), BL_FRESH_STALE);
    CHECK_EQ(f.lastSeq, s); // two consecutive stale packets must never rewind the reference
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 1), 0), BL_FRESH_ACCEPT);  // stream continues
    s++;
    // TX fault flag never accepted, does not move the reference
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 1), BL_FLAG_TX_INPUT_FAULT), BL_FRESH_TX_FAULT);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 2), BL_FLAG_TX_INPUT_FAULT), BL_FRESH_TX_FAULT);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 3), 0), BL_FRESH_ACCEPT);
    s += 3;
    // Wrap-around 65535 -> 0
    blFreshnessInit(&f);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65534, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65535, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 0, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 1, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65535, 0), BL_FRESH_STALE);
    // Lost frames within the forward half recover even at fast input acquisition rates.
    CHECK_EQ(blFreshnessCheck(&f, 3, 2001, 0), BL_FRESH_ACCEPT);
    // Handset restart: new session, sequence restarts
    CHECK_EQ(blFreshnessCheck(&f, 4, 0, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 4, 5, 0), BL_FRESH_ACCEPT);
    // Old session packet after the switch is not accepted
    CHECK_EQ(blFreshnessCheck(&f, 3, 300, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 4, 10, 0), BL_FRESH_ACCEPT);
    // Same-session counter rollback cannot masquerade as recovery.
    CHECK_EQ(blFreshnessCheck(&f, 4, 0, 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 4, 1, 0), BL_FRESH_STALE);
    CHECK_EQ(f.lastSeq, 10);
    CHECK_EQ(blFreshnessCheck(&f, 4, 11, BL_FLAG_TX_RESTART), BL_FRESH_TX_FAULT);
    CHECK_EQ(f.lastSeq, 10);
    // Repeated frozen candidates never anchor, while advancing samples do.
    blFreshnessInit(&f);
    for (int i = 0; i < 1000; i++) CHECK(blFreshnessCheck(&f, 9, 42, 0) != BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 9, 47, 0), BL_FRESH_ACCEPT);
}

// ---- Hop scheduler simulation (pure core, ideal radio) ----
typedef struct {
    double ppm;          // handset clock error
    int jitterUs;        // +/- jitter on each transmission
    int lossPct;         // random packet loss
    uint32_t phaseUs;    // handset start offset
    uint32_t stepUs;     // receiver processing period
    uint32_t outageFromUs, outageToUs;
} hopSim_t;

static void runHopSim(const hopSim_t *s, uint32_t durationUs, uint32_t *firstSyncUs, double *rxRatioAfterSync, uint32_t *reacqUs)
{
    uint8_t ch[BL_RF_CHANNEL_COUNT];
    blDeriveRfChannels(LINK, ch);
    blHop_t h;
    uint32_t now = 1000000;   // arbitrary start time
    blHopInit(&h, now);
    uint32_t start = now;
    uint32_t k = 0;
    double nextTx = start + s->phaseUs;
    uint32_t sent = 0, heard = 0;
    *firstSyncUs = 0;
    *reacqUs = 0;
    uint32_t tunedAt = now;
    uint8_t tunedIdx = h.idx;
    uint32_t lastHeard = 0;
    for (; now - start < durationUs; now += s->stepUs) {
        while (nextTx <= now) {
            uint32_t t = (uint32_t)nextTx + (s->jitterUs ? (uint32_t)((int)(rng32() % (2 * s->jitterUs + 1)) - s->jitterUs) : 0);
            uint8_t txIdx = k % BL_RF_CHANNEL_COUNT;
            bool inOutage = (t - start) >= s->outageFromUs && (t - start) < s->outageToUs;
            bool lost = (int)(rng32() % 100) < s->lossPct;
            if (!inOutage) {
                if (*firstSyncUs) sent++;
                // received if tuned to this channel for >=130us before the packet ends (1ms airtime)
                if (!lost && txIdx == tunedIdx && (int32_t)(t - tunedAt) >= 130) {
                    blHopOnPacket(&h, t + 1000, true);
                    if (!*firstSyncUs) *firstSyncUs = t - start; else heard++;
                    if (s->outageToUs && (t - start) >= s->outageToUs && !*reacqUs) *reacqUs = (t - start) - s->outageToUs;
                    lastHeard = t;
                }
            }
            k++;
            nextTx += BL_FRAME_PERIOD_US * (1.0 + s->ppm * 1e-6);
        }
        if (blHopTick(&h, now)) {
            tunedIdx = h.idx;
            tunedAt = now;
        }
    }
    (void)lastHeard;
    *rxRatioAfterSync = sent ? (double)heard / sent : 0;
}

static void testHopping(void)
{
    uint32_t first, reacq;
    double ratio;
    uint32_t worstFirst = 0;
    for (int trial = 0; trial < 200; trial++) {
        hopSim_t s = { .ppm = 0, .jitterUs = 50, .lossPct = 0, .phaseUs = rng32() % 20000, .stepUs = 125 };
        runHopSim(&s, 3000000, &first, &ratio, &reacq);
        CHECK(first != 0);
        const int32_t acq = (int32_t)(first - s.phaseUs);   // jitter can make this slightly negative
        if (acq > (int32_t)worstFirst) worstFirst = (uint32_t)acq;
        CHECK(ratio > 0.999);
        if (g_failures) { printf("  trial %d phase %u first %u ratio %f\n", trial, s.phaseUs, first, ratio); return; }
    }
    printf("  worst initial acquisition: %.1f ms\n", worstFirst / 1000.0);
    CHECK(worstFirst < 4 * BL_SEARCH_DWELL_US);

    // Clock error and slow processing loop
    const double ppms[] = { -500, -200, 200, 500 };
    for (int i = 0; i < 4; i++) {
        hopSim_t s = { .ppm = ppms[i], .jitterUs = 100, .lossPct = 0, .phaseUs = 7000, .stepUs = 1000 };
        runHopSim(&s, 10000000, &first, &ratio, &reacq);
        printf("  ppm %+5.0f, 1 kHz loop: received %.4f\n", ppms[i], ratio);
        CHECK(ratio > 0.999);
    }
    // 30% random loss: stays synchronised (ratio ~0.70)
    hopSim_t s = { .ppm = 100, .jitterUs = 50, .lossPct = 30, .phaseUs = 3000, .stepUs = 250 };
    runHopSim(&s, 10000000, &first, &ratio, &reacq);
    printf("  30%% loss: received %.4f\n", ratio);
    CHECK(ratio > 0.66);
    // Outage then recovery
    hopSim_t o = { .ppm = 150, .jitterUs = 50, .lossPct = 0, .phaseUs = 1234, .stepUs = 250,
                   .outageFromUs = 2000000, .outageToUs = 4000000 };
    runHopSim(&o, 8000000, &first, &ratio, &reacq);
    printf("  2 s outage: reacquired %.1f ms after the handset is audible again\n", reacq / 1000.0);
    CHECK(reacq != 0 && reacq < 4 * BL_SEARCH_DWELL_US);
}

static void testHopLateFifoPacket(void)
{
    // A packet read after the retune (left in FIFO) with a reliable timestamp must not skip a channel.
    blHop_t h;
    blHopInit(&h, 0);
    blHopOnPacket(&h, 10000, true);          // anchored: next hop at 12500
    CHECK_EQ(h.nextHopUs, 12500);
    CHECK(blHopTick(&h, 12500));
    uint8_t idx = h.idx;
    // The packet at 15000 arrives on idx, but is only processed at 17600 (after the 17500 retune)
    CHECK(blHopTick(&h, 17600));
    CHECK_EQ(h.idx, (idx + 1) % BL_RF_CHANNEL_COUNT);
    blHopOnPacket(&h, 15000, true);
    CHECK_EQ(h.nextHopUs, 15000 + 2500 + 5000);   // current channel's slot is kept
    CHECK(!blHopTick(&h, 22000));
    CHECK(blHopTick(&h, 22500));
    // Unreliable timestamp while synced does not move timing
    uint32_t before = h.nextHopUs;
    blHopOnPacket(&h, 24000 + 1700, false);
    CHECK_EQ(h.nextHopUs, before);
    // Missed slots lead to search after BL_SYNC_MAX_MISSED_SLOTS
    blHopInit(&h, 0);
    blHopOnPacket(&h, 1000, true);
    uint32_t t = 3500;
    int ticks = 0;
    while (h.state == BL_HOP_SYNCED && ticks < 100) { blHopTick(&h, t); t += BL_FRAME_PERIOD_US; ticks++; }
    CHECK_EQ(h.state, BL_HOP_SEARCH);
    CHECK_EQ(ticks, BL_SYNC_MAX_MISSED_SLOTS + 2);
    // A delayed call advances several slots at once (no loop)
    blHopInit(&h, 0);
    blHopOnPacket(&h, 1000, true);
    uint8_t i0 = h.idx;
    CHECK(blHopTick(&h, 3500 + 3 * BL_FRAME_PERIOD_US + 10));
    CHECK_EQ(h.idx, (i0 + 4) % BL_RF_CHANNEL_COUNT);
    CHECK_EQ(h.missedSlots, 3);
    // Timer wrap-around of the 32-bit microsecond counter
    blHopInit(&h, 0xFFFFF000U);
    blHopOnPacket(&h, 0xFFFFFF00U, true);
    CHECK(!blHopTick(&h, 0xFFFFFF00U + 2000));
    CHECK(blHopTick(&h, 0xFFFFFF00U + 2500));
}

int main(void)
{
    RUN(testRoundTrip);
    RUN(testKnownVector);
    RUN(testEncodeRejects);
    RUN(testDecodeRejects);
    RUN(testDerivation);
    RUN(testFreshness);
    RUN(testHopLateFifoPacket);
    RUN(testHopping);
    SUMMARY();
}
