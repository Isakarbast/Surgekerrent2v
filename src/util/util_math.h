/*
 * util_math.h - small numeric helpers shared by all modules.
 */
#ifndef UTIL_MATH_H
#define UTIL_MATH_H

#include <stdbool.h>
#include <stdint.h>

#define M_PI_F        3.14159265358979f
#define M_2PI_F       6.28318530717959f
#define ONE_BY_SQRT3  0.57735026918963f
#define TWO_BY_SQRT3  1.15470053837925f
#define SQRT3_BY_2    0.86602540378444f
#define SQRT3         1.73205080756888f

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float absf(float x) { return (x < 0.0f) ? -x : x; }
static inline float minf(float a, float b) { return (a < b) ? a : b; }
static inline float maxf(float a, float b) { return (a > b) ? a : b; }
static inline float signf(float x) { return (x < 0.0f) ? -1.0f : 1.0f; }
static inline float sqf(float x) { return x * x; }

float wrap_far(float x);

/* Wrap angle into [0, 2pi). Fast path for the normal case (within a few
 * turns); anything larger or non-finite goes through wrap_far(), so these
 * can never loop forever - they run inside the 20 kHz ISR. */
static inline float wrap_2pi(float x)
{
    if (!(x > -4.0f * M_2PI_F && x < 4.0f * M_2PI_F)) x = wrap_far(x); /* also catches NaN */
    while (x >= M_2PI_F) x -= M_2PI_F;
    while (x < 0.0f) x += M_2PI_F;
    return x;
}

/* Wrap angle into [-pi, pi). */
static inline float wrap_pi(float x)
{
    if (!(x > -4.0f * M_2PI_F && x < 4.0f * M_2PI_F)) x = wrap_far(x);
    while (x >= M_PI_F) x -= M_2PI_F;
    while (x < -M_PI_F) x += M_2PI_F;
    return x;
}

/* Linear map with clamping, in_lo may be > in_hi. */
static inline float mapf_clamped(float x, float in_lo, float in_hi, float out_lo, float out_hi)
{
    float t = (x - in_lo) / (in_hi - in_lo);
    t = clampf(t, 0.0f, 1.0f);
    return out_lo + t * (out_hi - out_lo);
}

/* First-order IIR low-pass: y += alpha * (x - y). */
static inline void lpf_update(float *y, float x, float alpha)
{
    *y += alpha * (x - *y);
}

/* alpha for a first-order LPF of cutoff fc sampled at dt. */
static inline float lpf_alpha(float fc_hz, float dt)
{
    float rc = 1.0f / (M_2PI_F * fc_hz);
    return dt / (rc + dt);
}

/* Move x toward target by at most step. */
static inline float ramp_towards(float x, float target, float step)
{
    if (x < target) {
        x += step;
        if (x > target) x = target;
    } else if (x > target) {
        x -= step;
        if (x < target) x = target;
    }
    return x;
}

float fast_atan2f(float y, float x);
void fast_sincosf(float theta, float *s, float *c);
float fast_sqrtf(float x);

#endif
