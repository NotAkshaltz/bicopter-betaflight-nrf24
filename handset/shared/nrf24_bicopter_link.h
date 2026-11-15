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
 * NRF24_BICOPTER link protocol, version 2 - hardware independent core.
 *
 * This file (and nrf24_bicopter_link.c) is shared VERBATIM between the
 * Betaflight receiver and the STM32F103 handset firmware. It has no
 * dependency on Betaflight or on any MCU so it can also be unit tested on a PC.
 *
 * Radio layer (both ends):
 *   nRF24L01+, 250 kbps, 2-byte radio CRC, 5-byte address, auto-acknowledge
 *   on pipe 0 with dynamic payload length, no ACK payload, no retransmits.
 *   The 5 address bytes are written to RX_ADDR_P0 / TX_ADDR in array order
 *   (addr[0] first, i.e. addr[0] is the least significant address byte).
 *
 * Pairing: there is no over-the-air bind. Both ends are built with the same
 * 32-bit link ID; it determines the radio address, the hop channels, the
 * packet identity tag and the CRC seed.
 *
 * Control packet (BL_PAYLOAD_LEN = 20 bytes, multi-byte fields little-endian):
 *   [0]      type      = BL_PKT_TYPE_CONTROL (0xB1)
 *   [1]      version   = BL_PROTO_VERSION (2)
 *   [2..3]   link tag  = blLinkTag(linkId)
 *   [4]      session   = random non-zero value chosen at handset boot
 *   [5..6]   sampleSeq = incremented ONLY when a new joystick/switch sample set
 *                        has been acquired (not per transmission)
 *   [7..16]  8 channels x 10 bits, packed LSB-first; value v in 0..1000 means
 *            (1000 + v) microseconds. Values 1001..1023 are invalid.
 *   [17]     flags: bit0 = input fault, bit1 = restart/interlock, bits 2..7 zero
 *   [18..19] CRC-16/CCITT (poly 0x1021, MSB first) over bytes 0..17,
 *            initial value blCrcSeed(linkId)
 *
 * Timing: the handset sends one packet every BL_FRAME_PERIOD_US, packet k on
 * hop channel (k mod BL_RF_CHANNEL_COUNT). See blHop* below for the receiver.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BL_PROTO_VERSION            2
#define BL_PKT_TYPE_CONTROL         0xB1
#define BL_PAYLOAD_LEN              20
#define BL_ADDR_LEN                 5
#define BL_CHANNEL_COUNT            8
#define BL_CHANNEL_VALUE_MAX        1000
#define BL_CHANNEL_US_MIN           1000
#define BL_CHANNEL_US_MAX           (BL_CHANNEL_US_MIN + BL_CHANNEL_VALUE_MAX)

#define BL_RF_CHANNEL_COUNT         4
#define BL_RF_CHANNEL_FIRST         4      // lowest nRF24 RF_CH used (2404 MHz)
#define BL_RF_CHANNEL_SPACING       19     // MHz between hop channels
#define BL_RF_CHANNEL_OFFSET_SPAN   20     // per-link offset 0..19 -> highest RF_CH = 80

#define BL_FRAME_PERIOD_US          5000   // 200 packets per second
#define BL_SEQ_ACCEPT_WINDOW        32767  // forward half of the 16-bit sample counter
#define BL_SYNC_MAX_MISSED_SLOTS    8      // consecutive empty slots before searching again (40 ms)
#define BL_SEARCH_DWELL_US          ((BL_RF_CHANNEL_COUNT + 1) * BL_FRAME_PERIOD_US)

#define BL_FLAG_TX_INPUT_FAULT      0x01
#define BL_FLAG_TX_RESTART          0x02   // startup/input recovery interlock; never a valid control sample
#define BL_FLAGS_DEFINED_MASK       (BL_FLAG_TX_INPUT_FAULT | BL_FLAG_TX_RESTART)
#define BL_ARM_CHANNEL             4
#define BL_THROTTLE_CHANNEL        2
#define BL_ARM_LOW_US              1250
#define BL_THROTTLE_LOW_US         1050

typedef struct blControl_s {
    uint8_t session;
    uint16_t sampleSeq;
    uint8_t flags;
    uint16_t channelUs[BL_CHANNEL_COUNT];   // 1000..2000
} blControl_t;

typedef enum {
    BL_DECODE_OK = 0,
    BL_DECODE_BAD_LENGTH,
    BL_DECODE_BAD_TYPE,
    BL_DECODE_BAD_VERSION,
    BL_DECODE_BAD_TAG,
    BL_DECODE_BAD_CRC,
    BL_DECODE_BAD_FLAGS,
    BL_DECODE_BAD_RANGE,
    BL_DECODE_BAD_SESSION,
} blDecodeResult_e;

typedef enum {
    BL_FRESH_ACCEPT = 0,        // new control sample: may refresh the RX watchdog
    BL_FRESH_DUPLICATE,         // same sample as the last accepted one
    BL_FRESH_STALE,             // older than the last accepted sample (same session)
    BL_FRESH_RESYNC_PENDING,    // waiting for a forward-moving confirmation, not necessarily seq+1
    BL_FRESH_TX_FAULT,          // handset reported its own input acquisition fault
} blFreshResult_e;

typedef struct blFreshness_s {
    bool haveRef;
    uint8_t session;
    uint16_t lastSeq;
    bool haveCandidate;
    uint8_t candSession;
    uint16_t candSeq;
} blFreshness_t;

typedef enum {
    BL_HOP_SEARCH = 0,
    BL_HOP_SYNCED = 1,
} blHopState_e;

typedef struct blHop_s {
    blHopState_e state;
    uint8_t idx;                // index into the hop channel table currently tuned
    uint32_t nextHopUs;         // time of the next retune
    uint32_t lastHopUs;         // time of the last retune
    uint16_t missedSlots;       // consecutive slots without an authentic packet
    bool heardThisSlot;
} blHop_t;

uint32_t blMix32(uint32_t x);
uint16_t blLinkTag(uint32_t linkId);
uint16_t blCrcSeed(uint32_t linkId);
uint16_t blCrc16(uint16_t crc, const uint8_t *data, size_t len);
void blDeriveAddress(uint32_t linkId, uint8_t addr[BL_ADDR_LEN]);
void blDeriveRfChannels(uint32_t linkId, uint8_t channels[BL_RF_CHANNEL_COUNT]);

// Returns false (and leaves out unspecified) if any channel is outside 1000..2000us,
// session is zero, or undefined flag bits are set.
bool blEncode(uint32_t linkId, const blControl_t *control, uint8_t out[BL_PAYLOAD_LEN]);
blDecodeResult_e blDecode(uint32_t linkId, const uint8_t *buf, uint8_t len, blControl_t *out);

void blFreshnessInit(blFreshness_t *f);
blFreshResult_e blFreshnessCheck(blFreshness_t *f, uint8_t session, uint16_t seq, uint8_t flags);

void blHopInit(blHop_t *h, uint32_t nowUs);
// An authentic (decoded OK) packet was read. If timeReliable is false the
// timestamp is only approximate (e.g. a packet left over in the RX FIFO) and the
// slot timing is not re-anchored while synchronised.
void blHopOnPacket(blHop_t *h, uint32_t packetTimeUs, bool timeReliable);
// Returns true if h->idx changed and the radio must be retuned. Bounded: never loops.
bool blHopTick(blHop_t *h, uint32_t nowUs);
