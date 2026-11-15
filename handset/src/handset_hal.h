// Hardware abstraction the STM32F103 board port must implement.
// Nothing here assumes a pin map, clock tree, HAL library or toolchain.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define HS_ANALOG_COUNT 4      // roll, pitch, throttle, yaw sticks
#define HS_SWITCH_COUNT 4      // aux1 (arm) .. aux4, value 0..2 (2- or 3-position)

typedef struct {
    uint16_t analog[HS_ANALOG_COUNT];   // raw 12-bit ADC counts
    uint8_t sw[HS_SWITCH_COUNT];        // 0 = low, 1 = mid, 2 = high
} hsRawInputs_t;

// Monotonic microsecond counter (32-bit wrap allowed).
uint32_t hal_micros(void);
// Returns true exactly once per COMPLETED new acquisition (e.g. from the ADC DMA
// transfer-complete interrupt plus a switch read) and copies it out. Returns false
// if no new acquisition finished since the previous call. Must not return true for a
// repeated copy of old data: this is what makes frozen acquisition detectable.
bool hal_take_new_inputs(hsRawInputs_t *out);
// Non-zero random byte that differs on each boot (e.g. hash of unique ID + ADC noise + timer).
uint8_t hal_boot_random_nonzero(void);

// nRF24 SPI: mode 0, MSB first, <= 8 MHz. CSN is handled by hal_nrf_csn().
uint8_t hal_spi_xfer(uint8_t byte);
void hal_nrf_csn(bool high);
void hal_nrf_ce(bool high);
void hal_delay_us(uint32_t us);
void hal_led(bool on);
