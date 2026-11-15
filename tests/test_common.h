#pragma once
#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { g_checks++; long long _a = (long long)(a), _b = (long long)(b); if (_a != _b) { g_failures++; \
    printf("  FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)
#define RUN(test) do { int _f = g_failures; test(); printf("%s %s\n", g_failures == _f ? "PASS" : "FAIL", #test); } while (0)
#define SUMMARY() do { printf("\n%d checks, %d failures\n", g_checks, g_failures); return g_failures ? 1 : 0; } while (0)

// Deterministic PRNG for tests
static unsigned long long g_rng = 0x1234567887654321ULL;
static unsigned rng32(void) { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17; return (unsigned)(g_rng >> 16); }
