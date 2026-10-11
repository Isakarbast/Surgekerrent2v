/*
 * Nonlinear flux observer (Ortega, Praly, Astolfi, Lee, Nam - IEEE TCST 2011),
 * the same family used by VESC and ODrive for sensorless PMSM control.
 *
 * State x = stator flux in alpha/beta. We use the "active flux"
 * eta = x - Lq*i, which for ANY saliency lies exactly on the d-axis with
 * magnitude psi_a = psi + (Ld - Lq)*id. The correction term pulls |eta|
 * back to psi_a:
 *     dx/dt = v - R*i + (gamma/2) * eta * (psi_a^2 - |eta|^2)
 * Angle = atan2(eta_beta, eta_alpha). For surface-mount hub motors
 * (Ld ~= Lq) this reduces to the classic Ortega observer.
 *
 * Works down to roughly 5-10% of rated speed with good parameters. Below
 * that, back-EMF is buried in noise and inverter nonlinearity - use halls.
 */
#ifndef FLUX_OBSERVER_H
#define FLUX_OBSERVER_H

#include "foc_math.h"
#include "pll.h"

typedef struct {
    float rs, ld, lq, flux;
    float gamma;          /* = gain / flux^2 */
    float x_alpha, x_beta;
    float theta_raw;
    float flux_err;       /* normalized |eta|/psi - 1, health indicator */
    pll_t pll;
} flux_observer_t;

void flux_obs_init(flux_observer_t *o, float rs, float ld, float lq, float flux,
                   float gain, float pll_kp, float pll_ki);
void flux_obs_reset(flux_observer_t *o, float theta, float omega);
/* v_ab: mean voltage the motor actually saw since the previous call.
 * id: latest measured d-axis current (only matters for salient motors). */
void flux_obs_update(flux_observer_t *o, ab_t v_ab, ab_t i_ab, float id, float dt);

static inline float flux_obs_angle(const flux_observer_t *o) { return o->pll.theta; }
static inline float flux_obs_omega(const flux_observer_t *o) { return o->pll.omega; }

#endif
