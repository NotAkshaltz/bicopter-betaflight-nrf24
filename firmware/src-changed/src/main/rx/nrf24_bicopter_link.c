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
 * NRF24_BICOPTER link protocol v2 - hardware independent core.
 * Shared verbatim with the handset firmware; see nrf24_bicopter_link.h.
 */

#include <string.h>

#include "nrf24_bicopter_link.h"

#define BL_OFFSET_TAG       2
#define BL_OFFSET_SESSION   4
#define BL_OFFSET_SEQ       5
#define BL_OFFSET_CHANNELS  7
#define BL_OFFSET_FLAGS     17
#define BL_OFFSET_CRC       18
#define BL_CHANNEL_BITS     10

uint32_t blMix32(uint32_t x)
{
    // "lowbias32" integer hash: good avalanche, identical on every 32-bit C target
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

uint16_t blLinkTag(uint32_t linkId)
{
    return (uint16_t)(blMix32(linkId) >> 16);
}

uint16_t blCrcSeed(uint32_t linkId)
{
    uint16_t seed = (uint16_t)(blMix32(linkId ^ 0xC0FFEE11U) & 0xFFFFU);
    if (seed == 0) {
        seed = 0x1D0F;
    }
    return seed;
}

uint16_t blCrc16(uint16_t crc, const uint8_t *data, size_t len)
{
    // CRC-16/CCITT, polynomial 0x1021, MSB first, no reflection, no final xor
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint8_t blFixAddressByte(uint8_t b)
{
    // Avoid address bytes with no or only regular bit transitions; 0x55/0xAA also
    // resemble the radio preamble.
    if (b == 0x00 || b == 0xFF || b == 0x55 || b == 0xAA) {
        b ^= 0x3C;
    }
    return b;
}

void blDeriveAddress(uint32_t linkId, uint8_t addr[BL_ADDR_LEN])
{
    const uint32_t a = blMix32(linkId ^ 0xA5C3B1E7U);
    const uint32_t b = blMix32(a ^ linkId);
    addr[0] = blFixAddressByte((uint8_t)(a & 0xFF));
    addr[1] = blFixAddressByte((uint8_t)((a >> 8) & 0xFF));
    addr[2] = blFixAddressByte((uint8_t)((a >> 16) & 0xFF));
    addr[3] = blFixAddressByte((uint8_t)((a >> 24) & 0xFF));
    addr[4] = blFixAddressByte((uint8_t)(b & 0xFF));
}

void blDeriveRfChannels(uint32_t linkId, uint8_t channels[BL_RF_CHANNEL_COUNT])
{
    const uint8_t offset = (uint8_t)(blMix32(linkId ^ 0x5EED5EEDU) % BL_RF_CHANNEL_OFFSET_SPAN);
    for (int i = 0; i < BL_RF_CHANNEL_COUNT; i++) {
        channels[i] = (uint8_t)(BL_RF_CHANNEL_FIRST + offset + i * BL_RF_CHANNEL_SPACING);
    }
}

bool blEncode(uint32_t linkId, const blControl_t *control, uint8_t out[BL_PAYLOAD_LEN])
{
    if (control->session == 0 || (control->flags & (uint8_t)~BL_FLAGS_DEFINED_MASK)) {
        return false;
    }
    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
        if (control->channelUs[ch] < BL_CHANNEL_US_MIN || control->channelUs[ch] > BL_CHANNEL_US_MAX) {
            return false;
        }
    }

    memset(out, 0, BL_PAYLOAD_LEN);
    const uint16_t tag = blLinkTag(linkId);
    out[0] = BL_PKT_TYPE_CONTROL;
    out[1] = BL_PROTO_VERSION;
    out[BL_OFFSET_TAG] = (uint8_t)(tag & 0xFF);
    out[BL_OFFSET_TAG + 1] = (uint8_t)(tag >> 8);
    out[BL_OFFSET_SESSION] = control->session;
    out[BL_OFFSET_SEQ] = (uint8_t)(control->sampleSeq & 0xFF);
    out[BL_OFFSET_SEQ + 1] = (uint8_t)(control->sampleSeq >> 8);

    unsigned bitPos = 0;
    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
        const uint16_t value = (uint16_t)(control->channelUs[ch] - BL_CHANNEL_US_MIN);
        for (int bit = 0; bit < BL_CHANNEL_BITS; bit++, bitPos++) {
            if (value & (1U << bit)) {
                out[BL_OFFSET_CHANNELS + (bitPos >> 3)] |= (uint8_t)(1U << (bitPos & 7));
            }
        }
    }

    out[BL_OFFSET_FLAGS] = control->flags;
    const uint16_t crc = blCrc16(blCrcSeed(linkId), out, BL_OFFSET_CRC);
    out[BL_OFFSET_CRC] = (uint8_t)(crc & 0xFF);
    out[BL_OFFSET_CRC + 1] = (uint8_t)(crc >> 8);
    return true;
}

blDecodeResult_e blDecode(uint32_t linkId, const uint8_t *buf, uint8_t len, blControl_t *out)
{
    if (len != BL_PAYLOAD_LEN) {
        return BL_DECODE_BAD_LENGTH;
    }
    if (buf[0] != BL_PKT_TYPE_CONTROL) {
        return BL_DECODE_BAD_TYPE;
    }
    if (buf[1] != BL_PROTO_VERSION) {
        return BL_DECODE_BAD_VERSION;
    }
    const uint16_t tag = (uint16_t)(buf[BL_OFFSET_TAG] | (buf[BL_OFFSET_TAG + 1] << 8));
    if (tag != blLinkTag(linkId)) {
        return BL_DECODE_BAD_TAG;
    }
    const uint16_t crc = (uint16_t)(buf[BL_OFFSET_CRC] | (buf[BL_OFFSET_CRC + 1] << 8));
    if (crc != blCrc16(blCrcSeed(linkId), buf, BL_OFFSET_CRC)) {
        return BL_DECODE_BAD_CRC;
    }
    const uint8_t flags = buf[BL_OFFSET_FLAGS];
    if (flags & (uint8_t)~BL_FLAGS_DEFINED_MASK) {
        return BL_DECODE_BAD_FLAGS;
    }
    const uint8_t session = buf[BL_OFFSET_SESSION];
    if (session == 0) {
        return BL_DECODE_BAD_SESSION;
    }

    blControl_t decoded;
    unsigned bitPos = 0;
    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
        uint16_t value = 0;
        for (int bit = 0; bit < BL_CHANNEL_BITS; bit++, bitPos++) {
            if (buf[BL_OFFSET_CHANNELS + (bitPos >> 3)] & (1U << (bitPos & 7))) {
                value |= (uint16_t)(1U << bit);
            }
        }
        if (value > BL_CHANNEL_VALUE_MAX) {
            return BL_DECODE_BAD_RANGE;
        }
        decoded.channelUs[ch] = (uint16_t)(BL_CHANNEL_US_MIN + value);
    }
    decoded.session = session;
    decoded.sampleSeq = (uint16_t)(buf[BL_OFFSET_SEQ] | (buf[BL_OFFSET_SEQ + 1] << 8));
    decoded.flags = flags;

    if (out) {
        *out = decoded;
    }
    return BL_DECODE_OK;
}

void blFreshnessInit(blFreshness_t *f)
{
    memset(f, 0, sizeof(*f));
}

/*
 * Freshness rules:
 *  - A packet flagged BL_FLAG_TX_INPUT_FAULT is never accepted.
 *  - Same session as the reference and sequence advanced by 1..BL_SEQ_ACCEPT_WINDOW: accept.
 *    (The handset only advances the sequence when a new input sample exists, so a frozen
 *    acquisition produces duplicates, never acceptances.)
 *  - Sequence unchanged: duplicate.
 *  - Backward movement in an established session is always stale. It cannot re-anchor.
 *  - A new session needs two forward-moving samples. ADC acquisition and radio rates
 *    are independent, so confirmation permits any positive step in the forward half.
 *  - Restart announcements and input faults never count as valid control samples.
 *  - Sequence numbers are 16-bit and compared modulo 2^16, so wrap-around is seamless.
 */
blFreshResult_e blFreshnessCheck(blFreshness_t *f, uint8_t session, uint16_t seq, uint8_t flags)
{
    if (flags & (BL_FLAG_TX_INPUT_FAULT | BL_FLAG_TX_RESTART)) {
        return BL_FRESH_TX_FAULT;
    }

    const bool sameSession = f->haveRef && session == f->session;
    if (sameSession) {
        const uint16_t step = (uint16_t)(seq - f->lastSeq);
        if (step == 0) {
            return BL_FRESH_DUPLICATE;
        }
        if (step <= BL_SEQ_ACCEPT_WINDOW) {
            f->lastSeq = seq;
            f->haveCandidate = false;
            return BL_FRESH_ACCEPT;
        }
        return BL_FRESH_STALE;
    }

    const uint16_t candidateStep = (uint16_t)(seq - f->candSeq);
    if (f->haveCandidate && f->candSession == session && candidateStep > 0 && candidateStep <= BL_SEQ_ACCEPT_WINDOW) {
        f->haveRef = true;
        f->session = session;
        f->lastSeq = seq;
        f->haveCandidate = false;
        return BL_FRESH_ACCEPT;
    }

    f->haveCandidate = true;
    f->candSession = session;
    f->candSeq = seq;
    return BL_FRESH_RESYNC_PENDING;
}

void blHopInit(blHop_t *h, uint32_t nowUs)
{
    memset(h, 0, sizeof(*h));
    h->state = BL_HOP_SEARCH;
    h->idx = 0;
    h->lastHopUs = nowUs;
    h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
}

/*
 * Slot timing. The handset transmits packet k at T0 + k*P on channel k mod N.
 * The receiver retunes half way between expected packets, so every packet arrives
 * in the middle of its listening window with +/- P/2 tolerance, and the retune never
 * interrupts the auto-ACK that follows a packet.
 */
void blHopOnPacket(blHop_t *h, uint32_t packetTimeUs, bool timeReliable)
{
    h->heardThisSlot = true;
    h->missedSlots = 0;

    if (h->state == BL_HOP_SYNCED && !timeReliable) {
        return;  // keep the existing slot timing
    }

    uint32_t next = packetTimeUs + BL_FRAME_PERIOD_US / 2;
    if ((int32_t)(packetTimeUs - h->lastHopUs) < 0) {
        // The packet arrived before the last retune (it waited in the FIFO), so it belongs
        // to the previous channel; the channel tuned now carries the following packet.
        next += BL_FRAME_PERIOD_US;
    }
    h->nextHopUs = next;
    h->state = BL_HOP_SYNCED;
}

bool blHopTick(blHop_t *h, uint32_t nowUs)
{
    const int32_t late = (int32_t)(nowUs - h->nextHopUs);
    if (late < 0) {
        return false;
    }

    if (h->state == BL_HOP_SEARCH) {
        // Dwell on each channel for longer than one full hop cycle of the handset.
        h->idx = (uint8_t)((h->idx + 1) % BL_RF_CHANNEL_COUNT);
        h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
        h->lastHopUs = nowUs;
        h->heardThisSlot = false;
        return true;
    }

    // Number of slot boundaries passed (normally 1; more if this call was delayed).
    const uint32_t slots = 1U + (uint32_t)late / BL_FRAME_PERIOD_US;
    const uint32_t missedNow = slots - (h->heardThisSlot ? 1U : 0U);
    const uint32_t missed = (uint32_t)h->missedSlots + missedNow;
    h->missedSlots = (uint16_t)(missed > 0xFFFFU ? 0xFFFFU : missed);
    h->heardThisSlot = false;
    h->idx = (uint8_t)((h->idx + slots) % BL_RF_CHANNEL_COUNT);
    h->nextHopUs += slots * BL_FRAME_PERIOD_US;
    h->lastHopUs = nowUs;

    if (h->missedSlots > BL_SYNC_MAX_MISSED_SLOTS) {
        h->state = BL_HOP_SEARCH;
        h->missedSlots = 0;
        h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
    }
    return true;
}
