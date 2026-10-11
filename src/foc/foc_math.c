#include "foc_math.h"

#include "util_math.h"

bool foc_svpwm(ab_t v, float vbus, float duty[3])
{
    bool saturated = false;
    if (vbus < 1.0f) {
        duty[0] = duty[1] = duty[2] = 0.5f;
        return true;
    }

    abc_t p = foc_inv_clarke(v);
    float vmax = maxf(p.a, maxf(p.b, p.c));
    float vmin = minf(p.a, minf(p.b, p.c));

    /* Line-line span must fit into vbus; otherwise scale uniformly. */
    float span = vmax - vmin;
    float scale = 1.0f;
    if (span > vbus) {
        scale = vbus / span;
        saturated = true;
    }
    float offset = -0.5f * (vmax + vmin) * scale;
    float inv = 1.0f / vbus;

    duty[0] = clampf(0.5f + (p.a * scale + offset) * inv, 0.0f, 1.0f);
    duty[1] = clampf(0.5f + (p.b * scale + offset) * inv, 0.0f, 1.0f);
    duty[2] = clampf(0.5f + (p.c * scale + offset) * inv, 0.0f, 1.0f);
    return saturated;
}

bool foc_limit_dq(dq_t *v, float vmax)
{
    float mag2 = v->d * v->d + v->q * v->q;
    if (mag2 <= vmax * vmax) return false;

    v->d = clampf(v->d, -vmax, vmax);
    float q_max = fast_sqrtf(vmax * vmax - v->d * v->d);
    v->q = clampf(v->q, -q_max, q_max);
    return true;
}
