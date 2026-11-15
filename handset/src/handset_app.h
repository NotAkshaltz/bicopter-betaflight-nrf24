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
// Call as often as possible from the main loop; transmits one packet every BL_FRAME_PERIOD_US.
void hs_poll(void);
const hsStats_t *hs_stats(void);
// Exposed for tests: convert raw inputs to channel microseconds.
bool hs_inputs_to_channels(const hsRawInputs_t *in, const hsStickCal_t cal[HS_ANALOG_COUNT], uint16_t outUs[8]);
