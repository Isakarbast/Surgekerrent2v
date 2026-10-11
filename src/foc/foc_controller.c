#include "foc_controller.h"

#include "board_config.h"
#include "util_math.h"

void foc_ctrl_init(foc_ctrl_t *f, float rs, float ld, float lq, float flux,
                   float kp, float ki, float max_mod)
{
    f->rs = rs;
    f->ld = ld;
    f->lq = lq;
    f->flux = flux;
    f->max_modulation = max_mod;
    f->decoupling_enabled = true;
    pi_init(&f->pi_d, kp, ki);
    pi_init(&f->pi_q, kp, ki);
    foc_ctrl_reset(f);
}

void foc_ctrl_reset(foc_ctrl_t *f)
{
    pi_reset(&f->pi_d, 0.0f);
    pi_reset(&f->pi_q, 0.0f);
    f->id_ref = 0.0f;
    f->iq_ref = 0.0f;
    f->i_dq.d = f->i_dq.q = 0.0f;
    f->v_dq.d = f->v_dq.q = 0.0f;
    f->v_ab.alpha = f->v_ab.beta = 0.0f;
    f->duty[0] = f->duty[1] = f->duty[2] = 0.5f;
    f->modulation = 0.0f;
    f->saturated = false;
}

static void output_voltage(foc_ctrl_t *f, float theta_out)
{
    float s, c;
    fast_sincosf(theta_out, &s, &c);
    f->v_ab = foc_inv_park(f->v_dq, s, c);
    f->saturated |= foc_svpwm(f->v_ab, f->vbus, f->duty);
}

void foc_ctrl_step(foc_ctrl_t *f, abc_t i_abc, float theta_e, float omega_e,
                   float vbus, float dt)
{
    float s, c;
    fast_sincosf(theta_e, &s, &c);

    f->vbus = vbus;
    f->i_ab = foc_clarke3(i_abc);
    f->i_dq = foc_park(f->i_ab, s, c);

    float vmax = f->max_modulation * vbus * ONE_BY_SQRT3;

    /* Cross-coupling / back-EMF feed-forward. */
    float ff_d = 0.0f, ff_q = 0.0f;
    if (f->decoupling_enabled) {
        ff_d = -omega_e * f->lq * f->i_dq.q;
        ff_q = omega_e * (f->ld * f->i_dq.d + f->flux);
    }

    /* d axis gets voltage priority: it controls flux / field weakening. */
    float vd = ff_d + pi_run(&f->pi_d, f->id_ref - f->i_dq.d, dt,
                             -vmax - ff_d, vmax - ff_d);
    vd = clampf(vd, -vmax, vmax);
    float vq_max = fast_sqrtf(vmax * vmax - vd * vd);
    float vq = ff_q + pi_run(&f->pi_q, f->iq_ref - f->i_dq.q, dt,
                             -vq_max - ff_q, vq_max - ff_q);

    f->v_dq.d = vd;
    f->v_dq.q = vq;
    f->saturated = foc_limit_dq(&f->v_dq, vmax);
    f->modulation = fast_sqrtf(sqf(f->v_dq.d) + sqf(f->v_dq.q)) /
                    maxf(vbus * ONE_BY_SQRT3, 1e-3f);

    output_voltage(f, theta_e + BOARD_PWM_OUTPUT_DELAY_PERIODS * dt * omega_e);
}

void foc_ctrl_apply_voltage(foc_ctrl_t *f, dq_t v, abc_t i_abc, float theta_e, float vbus)
{
    float s, c;
    fast_sincosf(theta_e, &s, &c);
    f->i_ab = foc_clarke3(i_abc);
    f->i_dq = foc_park(f->i_ab, s, c);
    f->vbus = vbus;
    f->v_dq = v;
    f->saturated = foc_limit_dq(&f->v_dq, f->max_modulation * vbus * ONE_BY_SQRT3);
    f->modulation = fast_sqrtf(sqf(f->v_dq.d) + sqf(f->v_dq.q)) /
                    maxf(vbus * ONE_BY_SQRT3, 1e-3f);
    output_voltage(f, theta_e);
}
