#include "pll.h"

#include "util_math.h"

void pll_init(pll_t *p, float kp, float ki)
{
    p->kp = kp;
    p->ki = ki;
    pll_reset(p, 0.0f, 0.0f);
}

void pll_reset(pll_t *p, float theta, float omega)
{
    p->theta = wrap_2pi(theta);
    p->omega = omega;
}

void pll_update(pll_t *p, float theta_meas, float dt)
{
    /* Predict to the measurement instant, then correct. theta is thus the
     * estimate AT this sample, not one period ahead. */
    float pred = wrap_2pi(p->theta + p->omega * dt);
    float err = wrap_pi(theta_meas - pred);
    p->omega += p->ki * err * dt;
    p->theta = wrap_2pi(pred + p->kp * err * dt);
}
