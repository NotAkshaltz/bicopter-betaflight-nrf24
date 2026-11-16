// Handset link configuration. HANDSET_LINK_ID MUST equal BICOPTER_NRF24_LINK_ID in
// betaflight src/config/configs/BICOPTER_PICO2/config.h.
#pragma once

#define HANDSET_LINK_ID         0xBE5336DFU

// nRF24 RF_SETUP power bits for the handset transmitter (0x06 = 0 dBm before the PA).
// Use 0x00 (-18 dBm) for close-range bench tests: a PA/LNA module at full power can
// saturate the receiver's LNA when the two are less than ~1-2 m apart.
#define HANDSET_RF_PWR_BITS     0x00

// Channel order sent on air (AETR + 4 aux). Betaflight default rcmap is AETR1234.
enum { HS_CH_ROLL = 0, HS_CH_PITCH, HS_CH_THROTTLE, HS_CH_YAW, HS_CH_AUX1_ARM, HS_CH_AUX2, HS_CH_AUX3, HS_CH_AUX4 };

// A new input sample must arrive at least this often or packets carry TX_INPUT_FAULT.
#define HANDSET_SAMPLE_TIMEOUT_US   50000
// Announce a reset for at least one second and until a new safe input sample.
#define HANDSET_RESTART_ANNOUNCE_US 1000000U
// Rail readings outside each stick's calibrated range indicate an input fault.
// A calibrated rail endpoint is valid; a broken wire at that endpoint cannot be
// distinguished by ADC value alone and needs separate board-level detection.
#define HANDSET_ADC_RAIL_MARGIN     8
#define HANDSET_ADC_MAX             4095
