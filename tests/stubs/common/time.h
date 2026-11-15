#pragma once
#include <stdint.h>
typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
