#pragma once
#include <stdint.h>
#define MAX_SUPPORTED_RC_CHANNEL_COUNT 18
typedef struct rxRuntimeState_s {
    uint8_t channelCount;
} rxRuntimeState_t;
