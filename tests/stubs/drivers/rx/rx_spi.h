#pragma once
#include <stdbool.h>
#include "common/time.h"
#define RX_SPI_MAX_PAYLOAD_SIZE 35
bool rxSpiExtiConfigured(void);
bool rxSpiGetExtiState(void);
bool rxSpiPollExti(void);
void rxSpiResetExti(void);
timeUs_t rxSpiGetLastExtiTimeUs(void);
