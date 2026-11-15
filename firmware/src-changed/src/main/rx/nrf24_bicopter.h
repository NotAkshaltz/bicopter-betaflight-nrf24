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

#pragma once

#include <stdint.h>

#include "common/time.h"

#include "rx/rx_spi.h"

typedef struct nrf24BicopterStats_s {
    uint32_t accepted;      // fresh control samples handed to the RX layer
    uint32_t malformed;     // bad width/length/type/version/tag/CRC/flags/range/session
    uint32_t duplicate;
    uint32_t stale;
    uint32_t resyncPending;
    uint32_t txFault;
    uint32_t spurious;      // IRQ/poll found an empty RX FIFO
    uint32_t fifoFlush;     // invalid payload width -> RX FIFO flushed
} nrf24BicopterStats_t;

bool nrf24BicopterInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig);
rx_spi_received_e nrf24BicopterDataReceived(uint8_t *payload);
void nrf24BicopterSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
timeUs_t nrf24BicopterFrameTimeUs(void);
const nrf24BicopterStats_t *nrf24BicopterGetStats(void);
