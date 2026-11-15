// Host-test stand-in for Betaflight's platform.h: only what rx/nrf24_bicopter.c needs.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define USE_RX_NRF24_BICOPTER
#ifndef BICOPTER_NRF24_LINK_ID
#define BICOPTER_NRF24_LINK_ID 0xBE5336DFU
#endif
