#include "flux_observer.h"

#include "util_math.h"

void flux_obs_init(flux_observer_t *o, float rs, float ld, float lq, float flux,
                   float gain, float pll_kp, float pll_ki)
{
    o->rs = rs;
    o->ld = ld;
    o->lq = lq;
    o->flux = flux;
    o->gamma = gain / maxf(flux * flux, 1e-8f);
    pll_init(&o->pll, pll_kp, pll_ki);
    flux_obs_reset(o, 0.0f, 0.0f);
}

void flux_obs_reset(flux_observer_t *o, float theta, float omega)
{
    float s, c;
    fast_sincosf(theta, &s, &c);
    o->x_alpha = o->flux * c;
    o->x_beta = o->flux * s;
    o->theta_raw = theta;
    o->flux_err = 0.0f;
    pll_reset(&o->pll, theta, omega);
}

void flux_obs_update(flux_observer_t *o, ab_t v_ab, ab_t i_ab, float id, float dt)
{
    float l_ia = o->lq * i_ab.alpha;
    float l_ib = o->lq * i_ab.beta;
    float psi_a = maxf(o->flux + (o->ld - o->lq) * id, 0.2f * o->flux);

    float eta_a = o->x_alpha - l_ia;
    float eta_b = o->x_beta - l_ib;
    float err = psi_a * psi_a - (eta_a * eta_a + eta_b * eta_b);
    /* Clamp correction so a large transient cannot make the integrator explode. */
    err = clampf(err, -psi_a * psi_a, psi_a * psi_a);

    float k = 0.5f * o->gamma * err;
    o->x_alpha += (v_ab.alpha - o->rs * i_ab.alpha + k * eta_a) * dt;
    o->x_beta += (v_ab.beta - o->rs * i_ab.beta + k * eta_b) * dt;

    eta_a = o->x_alpha - l_ia;
    eta_b = o->x_beta - l_ib;
    o->theta_raw = fast_atan2f(eta_b, eta_a);
    o->flux_err = fast_sqrtf(eta_a * eta_a + eta_b * eta_b) / psi_a - 1.0f;

    pll_update(&o->pll, o->theta_raw, dt);
}
