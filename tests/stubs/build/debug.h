#pragma once
#include <stdint.h>
#define DEBUG16_VALUE_COUNT 8
enum { DEBUG_NONE = 0, DEBUG_RX_NRF24_BICOPTER = 1 };
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;
#define DEBUG_SET(mode, index, value) do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)
