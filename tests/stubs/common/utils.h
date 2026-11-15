#pragma once
#define BIT(x) (1 << (x))
#define UNUSED(x) (void)(x)
#define STATIC_ASSERT(condition, name) _Static_assert((condition), #name)
