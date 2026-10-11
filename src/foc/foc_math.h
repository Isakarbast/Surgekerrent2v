/*
 * foc_math.h - coordinate transforms and space-vector modulation.
 *
 * Conventions (used everywhere in this firmware):
 *  - Amplitude-invariant Clarke: |i_ab| equals phase current peak.
 *    Electrical power P = 1.5 * (vd*id + vq*iq).
 *  - Positive phase current flows from the inverter INTO the motor.
 *  - theta is the electrical rotor angle, d-axis aligned with rotor flux.
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdbool.h>

typedef struct { float a, b, c; } abc_t;
typedef struct { float alpha, beta; } ab_t;
typedef struct { float d, q; } dq_t;

/* Clarke using all three phases (rejects common-mode offset). */
static inline ab_t foc_clarke3(abc_t i)
{
    ab_t r;
    r.alpha = (2.0f * i.a - i.b - i.c) * (1.0f / 3.0f);
    r.beta = (i.b - i.c) * 0.57735026918963f;
    return r;
}

static inline dq_t foc_park(ab_t x, float s, float c)
{
    dq_t r;
    r.d = c * x.alpha + s * x.beta;
    r.q = -s * x.alpha + c * x.beta;
    return r;
}

static inline ab_t foc_inv_park(dq_t x, float s, float c)
{
    ab_t r;
    r.alpha = c * x.d - s * x.q;
    r.beta = s * x.d + c * x.q;
    return r;
}

static inline abc_t foc_inv_clarke(ab_t x)
{
    abc_t r;
    r.a = x.alpha;
    r.b = -0.5f * x.alpha + 0.86602540378444f * x.beta;
    r.c = -0.5f * x.alpha - 0.86602540378444f * x.beta;
    return r;
}

/*
 * Space-vector PWM via min/max zero-sequence injection (mathematically
 * identical to 7-segment SVPWM). v in volts, vbus in volts.
 * Writes duties in [0,1]. Returns true if the vector had to be
 * scaled down to fit the hexagon (overmodulation request).
 */
bool foc_svpwm(ab_t v, float vbus, float duty[3]);

/* Magnitude-limit a dq voltage vector to vmax, giving d-axis priority. */
bool foc_limit_dq(dq_t *v, float vmax);

#endif
