// Host-test stand-in for rx/rx_spi.h (types copied from Betaflight 2026.6.2).
#pragma once
#include <stdint.h>
#include "drivers/io.h"
#include "rx/rx.h"

typedef enum { BETAFLIGHT_EXTI_TRIGGER_RISING = 0, BETAFLIGHT_EXTI_TRIGGER_FALLING = 1, BETAFLIGHT_EXTI_TRIGGER_BOTH = 2 } extiTrigger_t;

typedef struct rxSpiConfig_s {
    uint8_t rx_spi_protocol;
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;
} rxSpiConfig_t;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_PROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;

typedef struct {
    ioConfig_t ioConfig;
    extiTrigger_t trigger;
} rxSpiExtiConfig_t;
