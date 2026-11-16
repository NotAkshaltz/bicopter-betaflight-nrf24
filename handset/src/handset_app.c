// Portable handset application: input sampling -> NRF24_BICOPTER v2 packets -> nRF24L01+ TX.
#include <string.h>

#include "../config/link_config.h"
#include "../shared/nrf24_bicopter_link.h"
#include "handset_app.h"

// nRF24L01+ commands / registers
#define W_REGISTER 0x20
#define R_REGISTER 0x00
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define ACTIVATE 0x50
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_TX_ADDR 0x10
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D
#define ST_TX_DS 0x20
#define ST_MAX_RT 0x10

// CONFIG: EN_CRC | CRCO (2-byte CRC) | PWR_UP, PRIM_RX = 0 (transmitter)
#define TX_CONFIG 0x0E

static uint8_t rfCh[BL_RF_CHANNEL_COUNT];
static uint8_t addr[BL_ADDR_LEN];
static hsStickCal_t cal[HS_ANALOG_COUNT];
static blControl_t ctl;
static uint32_t nextTxUs, lastSampleUs, hop;
static bool haveSample, awaitingResult;
static bool startupLocked;
static uint32_t restartStartedUs;
static hsStats_t stats;

static void wreg(uint8_t r, uint8_t v) { hal_nrf_csn(false); hal_spi_xfer(W_REGISTER | r); hal_spi_xfer(v); hal_nrf_csn(true); }
static uint8_t rreg(uint8_t r) { hal_nrf_csn(false); hal_spi_xfer(R_REGISTER | r); uint8_t v = hal_spi_xfer(0xFF); hal_nrf_csn(true); return v; }
static void wmulti(uint8_t cmd, const uint8_t *d, uint8_t n) { hal_nrf_csn(false); hal_spi_xfer(cmd); for (uint8_t i = 0; i < n; i++) hal_spi_xfer(d[i]); hal_nrf_csn(true); }
static void cmd(uint8_t c) { hal_nrf_csn(false); hal_spi_xfer(c); hal_nrf_csn(true); }

static uint16_t mapStick(uint16_t raw, const hsStickCal_t *c)
{
    int32_t us;
    if (raw >= c->center) {
        const int32_t span = (int32_t)c->max - c->center;
        us = 1500 + (span > 0 ? ((int32_t)(raw - c->center) * 500) / span : 0);
    } else {
        const int32_t span = (int32_t)c->center - c->min;
        us = 1500 - (span > 0 ? ((int32_t)(c->center - raw) * 500) / span : 0);
    }
    if (us < BL_CHANNEL_US_MIN) us = BL_CHANNEL_US_MIN;
    if (us > BL_CHANNEL_US_MAX) us = BL_CHANNEL_US_MAX;
    if (c->reverse) us = 3000 - us;
    return (uint16_t)us;
}

bool hs_inputs_to_channels(const hsRawInputs_t *in, const hsStickCal_t c[HS_ANALOG_COUNT], uint16_t outUs[8])
{
    bool ok = true;
    for (int i = 0; i < HS_ANALOG_COUNT; i++) {
        const uint16_t raw = in->analog[i];
        const bool atRail = raw <= HANDSET_ADC_RAIL_MARGIN || raw >= HANDSET_ADC_MAX - HANDSET_ADC_RAIL_MARGIN;
        // A calibrated endpoint can legitimately reach a rail. Only reject a rail
        // outside that stick's measured range; ADC values above 12 bits are invalid.
        if (raw > HANDSET_ADC_MAX || (atRail && (raw < c[i].min || raw > c[i].max))) ok = false;
        outUs[i] = mapStick(in->analog[i], &c[i]);
    }
    for (int i = 0; i < HS_SWITCH_COUNT; i++) {
        outUs[HS_ANALOG_COUNT + i] = in->sw[i] == 0 ? 1000 : (in->sw[i] == 1 ? 1500 : 2000);
        if (in->sw[i] > 2) ok = false;
    }
    return ok;
}

bool hs_init(const hsStickCal_t c[HS_ANALOG_COUNT])
{
    for (int i = 0; i < HS_ANALOG_COUNT; i++) {
        if (!(c[i].min < c[i].center && c[i].center < c[i].max) || c[i].max > HANDSET_ADC_MAX) return false;
    }
    memcpy(cal, c, sizeof(cal));
    blDeriveAddress(HANDSET_LINK_ID, addr);
    blDeriveRfChannels(HANDSET_LINK_ID, rfCh);
    memset(&ctl, 0, sizeof(ctl));
    memset(&stats, 0, sizeof(stats));
    ctl.session = hal_boot_random_nonzero();
    if (!ctl.session) return false;
    ctl.sampleSeq = 0;
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) ctl.channelUs[i] = 1500;
    ctl.channelUs[2] = 1000; ctl.channelUs[4] = 1000;   // throttle low, arm off until sampled
    ctl.flags = BL_FLAG_TX_INPUT_FAULT;                 // nothing valid until the first sample
    haveSample = false;

    hal_nrf_ce(false);
    hal_delay_us(100000);                 // power-on reset
    wreg(REG_CONFIG, TX_CONFIG);
    hal_delay_us(5000);                   // power down -> standby
    wreg(REG_SETUP_AW, 0x03);
    if (rreg(REG_SETUP_AW) != 0x03) return false;
    wreg(REG_FEATURE, 0x04);              // EN_DPL
    if (rreg(REG_FEATURE) != 0x04) {
        const uint8_t key = 0x73;         // ACTIVATE is a command, not a register write
        wmulti(ACTIVATE, &key, 1);
        wreg(REG_FEATURE, 0x04);
        if (rreg(REG_FEATURE) != 0x04) return false;
    }
    wreg(REG_EN_AA, 0x01);
    wreg(REG_EN_RXADDR, 0x01);
    wreg(REG_SETUP_RETR, 0x20);           // ARD 750 us, ARC 0 (no retransmit: next frame is newer data)
    wreg(REG_RF_SETUP, 0x20 | HANDSET_RF_PWR_BITS);   // 250 kbps
    wreg(REG_DYNPD, 0x01);
    wmulti(W_REGISTER | REG_TX_ADDR, addr, BL_ADDR_LEN);
    wmulti(W_REGISTER | REG_RX_ADDR_P0, addr, BL_ADDR_LEN);   // to receive the auto-ACK
    cmd(FLUSH_TX); cmd(FLUSH_RX);
    wreg(REG_STATUS, 0x70);
    nextTxUs = hal_micros();
    lastSampleUs = hal_micros();
    hop = 0;
    awaitingResult = false;
    startupLocked = true;
    restartStartedUs = hal_micros();
    return rreg(REG_CONFIG) == TX_CONFIG;
}

void hs_poll(void)
{
    hsRawInputs_t in;
    const uint32_t now = hal_micros();
    if (hal_take_new_inputs(&in)) {
        uint16_t us[8];
        const bool ok = hs_inputs_to_channels(&in, cal, us);
        memcpy(ctl.channelUs, us, sizeof(us));
        ctl.sampleSeq++;                  // ONLY here: one increment per acquisition
        ctl.flags = ok ? 0 : BL_FLAG_TX_INPUT_FAULT;
        lastSampleUs = now;
        haveSample = true;
        // The restart lock is only entered by hs_init() (boot or re-init). An input fault
        // or a late sample flags the packet as TX_INPUT_FAULT instead: the receiver drops
        // it and Betaflight's own failsafe handles a fault that persists. Entering the
        // restart lock here would make the receiver disarm at once on one bad reading.
        if (startupLocked && ok && (uint32_t)(now - restartStartedUs) >= HANDSET_RESTART_ANNOUNCE_US &&
            us[BL_ARM_CHANNEL] <= BL_ARM_LOW_US && us[BL_THROTTLE_CHANNEL] <= BL_THROTTLE_LOW_US) {
            startupLocked = false;
        }
    }
    if (!haveSample || (int32_t)(now - lastSampleUs) > HANDSET_SAMPLE_TIMEOUT_US) {
        ctl.flags |= BL_FLAG_TX_INPUT_FAULT;
    }
    if (startupLocked) {
        ctl.flags |= BL_FLAG_TX_RESTART;
        ctl.channelUs[BL_ARM_CHANNEL] = BL_CHANNEL_US_MIN;
        ctl.channelUs[BL_THROTTLE_CHANNEL] = BL_CHANNEL_US_MIN;
    }

    if ((int32_t)(now - nextTxUs) < 0) return;
    if (awaitingResult) {
        const uint8_t st = rreg(REG_STATUS);
        // Never retune or flush an outstanding transmission. A radio that stops
        // completing transactions must cause link loss, not overlapping TX attempts.
        if (!(st & (ST_TX_DS | ST_MAX_RT))) return;
        if (st & ST_TX_DS) stats.acked++; else stats.noAck++;
        hal_led((st & ST_TX_DS) != 0);
        awaitingResult = false;
    }

    uint8_t pkt[BL_PAYLOAD_LEN];
    if (!blEncode(HANDSET_LINK_ID, &ctl, pkt)) return;
    if (ctl.flags & BL_FLAG_TX_INPUT_FAULT) stats.inputFaultFrames++;

    hal_nrf_ce(false);
    wreg(REG_RF_CH, rfCh[hop % BL_RF_CHANNEL_COUNT]);
    wreg(REG_STATUS, 0x70);
    cmd(FLUSH_TX);
    wmulti(W_TX_PAYLOAD, pkt, BL_PAYLOAD_LEN);
    // Schedule from this actual launch, skipping missed slots after a late loop.
    // No catch-up burst is allowed, including after a 32-bit timer wrap.
    nextTxUs = hal_micros() + BL_FRAME_PERIOD_US;
    hal_nrf_ce(true);
    hal_delay_us(15);                     // >10 us CE pulse starts one transmission
    hal_nrf_ce(false);
    hop++;
    stats.sent++;
    awaitingResult = true;
}

const hsStats_t *hs_stats(void) { return &stats; }
