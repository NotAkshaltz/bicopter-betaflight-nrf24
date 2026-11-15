/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * NRF24_BICOPTER receiver: Betaflight RX_SPI protocol adapter for an nRF24L01+.
 *
 * Derived in spirit from rx/nrf24_inav.c (same radio settings: 250 kbps, 5-byte
 * address, 2-byte CRC, dynamic payload, auto-ACK) but with a project specific
 * packet format and link state machine; see rx/nrf24_bicopter_link.h.
 *
 * Runtime behaviour (called from rxFrameCheck() once per scheduler loop):
 *  - Idle cost: one GPIO read (IRQ wired) or one STATUS read every
 *    BICOPTER_NRF24_POLL_INTERVAL_US (IRQ not wired), plus a time comparison.
 *  - At most ONE payload is read per call (bounded SPI traffic); if more are
 *    waiting in the RX FIFO they are read on following calls.
 *  - At most one retune per call.
 *  - Only fresh, valid control samples return RX_SPI_RECEIVED_DATA, which is the
 *    only thing that refreshes Betaflight's RX signal watchdog.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_NRF24_BICOPTER

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"
#include "flight/failsafe.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/nrf24_bicopter_link.h"
#include "rx/nrf24_bicopter.h"

#ifndef BICOPTER_NRF24_LINK_ID
#error "BICOPTER_NRF24_LINK_ID must be defined in the board config (same value as the handset firmware)"
#endif

#ifndef BICOPTER_NRF24_RX_RF_PWR
// Only used for the auto-ACK transmitted by the receiver.
#define BICOPTER_NRF24_RX_RF_PWR NRF24L01_06_RF_SETUP_RF_PWR_n12dbm
#endif

#ifndef BICOPTER_NRF24_POLL_INTERVAL_US
#define BICOPTER_NRF24_POLL_INTERVAL_US 500     // STATUS poll period when no IRQ pin is configured
#endif

#define BICOPTER_NRF24_SPURIOUS_BACKOFF_US 1000 // IRQ low but FIFO empty: re-check the level after this

STATIC_ASSERT(BL_PAYLOAD_LEN <= RX_SPI_MAX_PAYLOAD_SIZE, bicopter_payload_too_large);
STATIC_ASSERT(BL_CHANNEL_COUNT <= MAX_SUPPORTED_RC_CHANNEL_COUNT, bicopter_too_many_channels);

#define NRF24_STATUS_RX_P_NO_SHIFT  1
#define NRF24_STATUS_RX_P_NO_MASK   0x07
#define NRF24_RX_P_NO_FIFO_EMPTY    7
#define NRF24_ACTIVATE_FEATURES     0x73

typedef enum {
    INIT_OK = 0,
    INIT_FAIL_SETUP_AW = 1,
    INIT_FAIL_FEATURE = 2,
    INIT_FAIL_REGISTERS = 3,
    INIT_FAIL_ADDRESS = 4,
    INIT_FAIL_CONFIG = 5,
} initStatus_e;

static uint32_t linkId;
static uint8_t rfChannels[BL_RF_CHANNEL_COUNT];
static uint8_t rxAddr[BL_ADDR_LEN];

static blFreshness_t freshness;
static blHop_t hop;
static nrf24BicopterStats_t stats;

static bool irqConfigured;
static bool fifoPending;
static bool havePendingExtiTime;
static timeUs_t pendingExtiTimeUs;
static timeUs_t nextPollUs;
static timeUs_t nextLevelCheckUs;
static timeUs_t lastFrameTimeUs;
static initStatus_e initStatus;
static bool haveSession;
static uint8_t observedSession;
static bool restartAnnounced;
static bool armInterlockReleased;

static void tuneToHopChannel(void)
{
    NRF24L01_SetChannelRxMode(rfChannels[hop.idx]);
}

static bool writeRegVerify(uint8_t reg, uint8_t value)
{
    NRF24L01_WriteReg(reg, value);
    return NRF24L01_ReadReg(reg) == value;
}

static void updateDebug(void)
{
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 0, (int16_t)stats.accepted);
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 1, (int16_t)(stats.malformed + stats.fifoFlush));
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 2, (int16_t)(stats.duplicate + stats.stale));
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 3, (int16_t)(stats.resyncPending + stats.txFault));
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 4, (int16_t)((hop.state << 8) | rfChannels[hop.idx]));
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 5, (int16_t)hop.missedSlots);
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 6, (int16_t)stats.spurious);
    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 7, (int16_t)initStatus);
}

static rx_spi_received_e serviceRadio(uint8_t *payload, timeUs_t nowUs)
{
    const uint8_t status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    const uint8_t pipe = (status >> NRF24_STATUS_RX_P_NO_SHIFT) & NRF24_STATUS_RX_P_NO_MASK;

    if (pipe == NRF24_RX_P_NO_FIFO_EMPTY) {
        if (status & BIT(NRF24L01_07_STATUS_RX_DR)) {
            NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR));
        }
        fifoPending = false;
        havePendingExtiTime = false;
        nextLevelCheckUs = nowUs + BICOPTER_NRF24_SPURIOUS_BACKOFF_US;
        stats.spurious++;
        return RX_SPI_RECEIVED_NONE;
    }

    uint8_t buf[NRF24L01_MAX_PAYLOAD_SIZE];
    const uint8_t width = NRF24L01_ReadPayloadWidth();
    const bool widthOk = (width >= 1) && (width <= NRF24L01_MAX_PAYLOAD_SIZE);
    if (widthOk) {
        NRF24L01_ReadPayload(buf, width);
    } else {
        // nRF24L01+ product specification: a width above 32 means a corrupt packet; flush RX FIFO
        NRF24L01_FlushRx();
        stats.fifoFlush++;
    }
    // Clear RX_DR only after the payload has been read, then look at what is left
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR));
    fifoPending = widthOk && !(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BIT(NRF24L01_17_FIFO_STATUS_RX_EMPTY));

    if (!widthOk) {
        havePendingExtiTime = false;
        return RX_SPI_RECEIVED_NONE;
    }

    // The EXTI timestamp belongs to the most recently received packet, so it is only
    // attributed to the packet that empties the FIFO.
    bool timeReliable = false;
    timeUs_t packetTimeUs = nowUs;
    if (!fifoPending) {
        if (irqConfigured) {
            timeReliable = havePendingExtiTime;
            if (havePendingExtiTime) {
                packetTimeUs = pendingExtiTimeUs;
            }
        } else {
            timeReliable = true;    // polled: error bounded by the poll interval
        }
        havePendingExtiTime = false;
    }

    blControl_t control;
    if (pipe != NRF24L01_PIPE0 || blDecode(linkId, buf, width, &control) != BL_DECODE_OK) {
        stats.malformed++;
        return RX_SPI_RECEIVED_NONE;
    }

    // Authentic packet from our handset: it keeps the hop timing aligned even if the
    // sample itself is not fresh.
    blHopOnPacket(&hop, packetTimeUs, timeReliable);

    const bool newSession = haveSession && observedSession != control.session;
    const bool restart = (control.flags & BL_FLAG_TX_RESTART) != 0;
    if (newSession || (restart && !restartAnnounced)) {
        if (stats.accepted) {
            failsafeOnReceiverReset();
        }
        armInterlockReleased = false;
        blFreshnessInit(&freshness);
    }
    haveSession = true;
    observedSession = control.session;
    restartAnnounced = restart;
    if (restart) {
        // Repeated announcements must not refresh the watchdog or repeatedly disarm.
        stats.txFault++;
        return RX_SPI_RECEIVED_NONE;
    }
    // An input-fault packet is simply not accepted (BL_FRESH_TX_FAULT below), so a
    // persistent fault ends in Betaflight's normal RXLOSS/failsafe handling. It does not
    // re-engage the arm interlock: doing so would force ARM off, i.e. disarm in flight,
    // on the first good sample after a single bad stick reading.

    switch (blFreshnessCheck(&freshness, control.session, control.sampleSeq, control.flags)) {
    case BL_FRESH_ACCEPT:
        break;
    case BL_FRESH_DUPLICATE:
        stats.duplicate++;
        return RX_SPI_RECEIVED_NONE;
    case BL_FRESH_STALE:
        stats.stale++;
        return RX_SPI_RECEIVED_NONE;
    case BL_FRESH_TX_FAULT:
        stats.txFault++;
        return RX_SPI_RECEIVED_NONE;
    case BL_FRESH_RESYNC_PENDING:
    default:
        stats.resyncPending++;
        return RX_SPI_RECEIVED_NONE;
    }

    // Copy into the shared RX payload buffer only once the packet is accepted, so a
    // later rejected packet can never overwrite data that is waiting to be decoded.
    if (!armInterlockReleased) {
        if (control.channelUs[BL_ARM_CHANNEL] <= BL_ARM_LOW_US &&
            control.channelUs[BL_THROTTLE_CHANNEL] <= BL_THROTTLE_LOW_US) {
            armInterlockReleased = true;
        } else {
            control.channelUs[BL_ARM_CHANNEL] = BL_CHANNEL_US_MIN;
            control.channelUs[BL_THROTTLE_CHANNEL] = BL_CHANNEL_US_MIN;
        }
    }
    // Re-encode after safety overrides, so the deferred decoder sees a valid CRC.
    if (!blEncode(linkId, &control, payload)) {
        return RX_SPI_RECEIVED_NONE;
    }
    lastFrameTimeUs = packetTimeUs;
    stats.accepted++;
    return RX_SPI_RECEIVED_DATA;
}

rx_spi_received_e nrf24BicopterDataReceived(uint8_t *payload)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    const timeUs_t nowUs = micros();
    // RF loss is deliberately left to Betaflight's failsafe (failsafe_delay stage 1, then
    // failsafe_procedure). The arm interlock is only engaged at init, on a new handset
    // session and on an explicit restart announcement, never on a radio gap: forcing ARM
    // off when the link returns would turn every dropout longer than RXLOSS into a disarm.
    bool service = fifoPending;

    if (irqConfigured) {
        if (rxSpiPollExti()) {
            // Falling edge on IRQ (RX_DR asserted)
            rxSpiResetExti();
            pendingExtiTimeUs = rxSpiGetLastExtiTimeUs();
            havePendingExtiTime = true;
            service = true;
        } else if (!rxSpiGetExtiState() && cmpTimeUs(nowUs, nextLevelCheckUs) >= 0) {
            // IRQ is active-low and still asserted without a new edge (missed edge)
            service = true;
        }
    } else if (cmpTimeUs(nowUs, nextPollUs) >= 0) {
        nextPollUs = nowUs + BICOPTER_NRF24_POLL_INTERVAL_US;
        service = true;
    }

    if (service) {
        ret = serviceRadio(payload, nowUs);
    }

    if (blHopTick(&hop, micros())) {
        tuneToHopChannel();
    }

    updateDebug();
    return ret;
}

void nrf24BicopterSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    blControl_t control;
    // payload was validated before it was accepted; decode again to unpack it
    if (blDecode(linkId, payload, BL_PAYLOAD_LEN, &control) != BL_DECODE_OK) {
        return;
    }
    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
        rcData[ch] = control.channelUs[ch];
    }
}

timeUs_t nrf24BicopterFrameTimeUs(void)
{
    return lastFrameTimeUs;
}

const nrf24BicopterStats_t *nrf24BicopterGetStats(void)
{
    return &stats;
}

static initStatus_e nrf24BicopterSetup(void)
{
    // PWR_UP, 2-byte CRC; IRQ pin reports RX_DR only
    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO) |
                        BIT(NRF24L01_00_CONFIG_MASK_MAX_RT) | BIT(NRF24L01_00_CONFIG_MASK_TX_DS));

    // A wrong value here usually means SPI wiring, clock or radio power is wrong
    if (!writeRegVerify(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES)) {
        return INIT_FAIL_SETUP_AW;
    }

    // nRF24L01+ has FEATURE always accessible; original nRF24L01 and some clones need ACTIVATE
    if (!writeRegVerify(NRF24L01_1D_FEATURE, BIT(NRF24L01_1D_FEATURE_EN_DPL))) {
        NRF24L01_Activate(NRF24_ACTIVATE_FEATURES);
        if (!writeRegVerify(NRF24L01_1D_FEATURE, BIT(NRF24L01_1D_FEATURE_EN_DPL))) {
            return INIT_FAIL_FEATURE;
        }
    }

    bool ok = true;
    ok &= writeRegVerify(NRF24L01_01_EN_AA, BIT(NRF24L01_01_EN_AA_ENAA_P0));
    ok &= writeRegVerify(NRF24L01_02_EN_RXADDR, BIT(NRF24L01_02_EN_RXADDR_ERX_P0));
    ok &= writeRegVerify(NRF24L01_04_SETUP_RETR, 0);
    ok &= writeRegVerify(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | BICOPTER_NRF24_RX_RF_PWR);
    ok &= writeRegVerify(NRF24L01_1C_DYNPD, BIT(NRF24L01_1C_DYNPD_DPL_P0));
    ok &= writeRegVerify(NRF24L01_11_RX_PW_P0, NRF24L01_MAX_PAYLOAD_SIZE);  // ignored with DPL
    ok &= writeRegVerify(NRF24L01_05_RF_CH, rfChannels[hop.idx]);
    if (!ok) {
        return INIT_FAIL_REGISTERS;
    }

    uint8_t readBack[BL_ADDR_LEN];
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, BL_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rxAddr, BL_ADDR_LEN);  // ACKs use pipe 0 address
    NRF24L01_ReadRegisterMulti(NRF24L01_0A_RX_ADDR_P0, readBack, BL_ADDR_LEN);
    if (memcmp(readBack, rxAddr, BL_ADDR_LEN) != 0) {
        return INIT_FAIL_ADDRESS;
    }

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_SetRxMode();   // clears interrupts, CE high

    const uint8_t expectedConfig = BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO) |
        BIT(NRF24L01_00_CONFIG_MASK_MAX_RT) | BIT(NRF24L01_00_CONFIG_MASK_TX_DS) |
        BIT(NRF24L01_00_CONFIG_PWR_UP) | BIT(NRF24L01_00_CONFIG_PRIM_RX);
    if (NRF24L01_ReadReg(NRF24L01_00_CONFIG) != expectedConfig) {
        return INIT_FAIL_CONFIG;
    }
    return INIT_OK;
}

bool nrf24BicopterInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(rxSpiConfig);    // the link ID is fixed at build time, rx_spi_id is not used

    linkId = BICOPTER_NRF24_LINK_ID;
    blDeriveAddress(linkId, rxAddr);
    blDeriveRfChannels(linkId, rfChannels);
    blFreshnessInit(&freshness);
    memset(&stats, 0, sizeof(stats));
    fifoPending = false;
    havePendingExtiTime = false;
    lastFrameTimeUs = 0;
    haveSession = false;
    restartAnnounced = false;
    armInterlockReleased = false;

    rxRuntimeState->channelCount = BL_CHANNEL_COUNT;

    // nRF24L01+ IRQ is active low. rxSpiExtiInit() is called by rxSpiInit() after this.
    extiConfig->ioConfig = IOCFG_IPU;
    extiConfig->trigger = BETAFLIGHT_EXTI_TRIGGER_FALLING;
    irqConfigured = rxSpiExtiConfigured();

    blHopInit(&hop, micros());
    initStatus = nrf24BicopterSetup();
    // The search dwell starts once the radio is actually listening
    blHopInit(&hop, micros());

    const timeUs_t nowUs = micros();
    nextPollUs = nowUs;
    nextLevelCheckUs = nowUs;

    updateDebug();
    return initStatus == INIT_OK;
}

#endif // USE_RX_NRF24_BICOPTER
