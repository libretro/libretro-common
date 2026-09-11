#ifndef MUDLORD_LIBRETRO_DSP_COMMON_H
#define MUDLORD_LIBRETRO_DSP_COMMON_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float dsp_clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline int dsp_clampi(int x, int lo, int hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float dsp_db_to_gain(float db)
{
    return powf(10.0f, db * 0.05f);
}

static inline unsigned dsp_next_pow2(unsigned v)
{
    unsigned p = 1u;
    while (p < v && p < (1u << 31))
        p <<= 1;
    return p;
}

static inline float dsp_lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

#endif
#ifndef DSPFILTER_EXPORT
#ifdef HAVE_FILTERS_BUILTIN
#define DSPFILTER_EXPORT
#elif defined(_WIN32)
#define DSPFILTER_EXPORT __declspec(dllexport)
#else
#define DSPFILTER_EXPORT __attribute__((visibility("default")))
#endif
#endif
