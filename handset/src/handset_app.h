#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "handset_hal.h"

typedef struct {
    uint16_t min, center, max;   // raw ADC calibration per stick
    bool reverse;
} hsStickCal_t;

typedef struct {
    uint32_t sent, acked, noAck, inputFaultFrames;
} hsStats_t;

// Returns false if the radio did not respond (check wiring/power).
bool hs_init(const hsStickCal_t cal[HS_ANALOG_COUNT]);
// Call often from the main loop. TX launches are at least BL_FRAME_PERIOD_US apart
// (nominal 200 Hz); foreground/SPI latency can lower the actual rate. Never catches up in bursts.
void hs_poll(void);
const hsStats_t *hs_stats(void);
// Exposed for tests: convert raw inputs to channel microseconds.
bool hs_inputs_to_channels(const hsRawInputs_t *in, const hsStickCal_t cal[HS_ANALOG_COUNT], uint16_t outUs[8]);
