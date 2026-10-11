#include "util_math.h"
#include <math.h>

/*
 * atan2 approximation, max error ~0.0015 rad. Good enough for observer
 * angle extraction; the PLL downstream smooths residual error.
 */
float fast_atan2f(float y, float x)
{
    float ax = absf(x), ay = absf(y);
    if (ax < 1e-12f && ay < 1e-12f) return 0.0f;

    float mx = maxf(ax, ay);
    float mn = minf(ax, ay);
    float a = mn / mx;
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (ay > ax) r = 1.57079637f - r;
    if (x < 0.0f) r = M_PI_F - r;
    if (y < 0.0f) r = -r;
    return r;
}

/*
 * Cortex-M4F has a single-precision FPU; sinf/cosf from newlib are
 * slow (~1us each). Use a 5th-order polynomial after range reduction.
 * Max abs error ~1e-5.
 */
static float sin_poly(float x) /* x in [-pi/2, pi/2] */
{
    float x2 = x * x;
    return x * (1.0f + x2 * (-0.16666667f + x2 * (0.0083333310f + x2 * (-0.00019840874f + x2 * 2.7525562e-6f))));
}

void fast_sincosf(float theta, float *s, float *c)
{
    float t = wrap_pi(theta);
    float ts = t;
    if (ts > 1.57079637f) ts = M_PI_F - ts;
    else if (ts < -1.57079637f) ts = -M_PI_F - ts;
    *s = sin_poly(ts);

    float tc = wrap_pi(t + 1.57079637f);
    if (tc > 1.57079637f) tc = M_PI_F - tc;
    else if (tc < -1.57079637f) tc = -M_PI_F - tc;
    *c = sin_poly(tc);
}

float wrap_far(float x)
{
    if (!isfinite(x)) return 0.0f;
    return fmodf(x, M_2PI_F); /* result in (-2pi, 2pi): the fast loops finish it */
}

float fast_sqrtf(float x)
{
    /* Compiles to VSQRT.F32 on Cortex-M4F with -ffast-math/-fno-math-errno. */
    return (x <= 0.0f) ? 0.0f : sqrtf(x);
}
