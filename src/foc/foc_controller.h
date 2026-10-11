/*
 * foc_controller.h - synchronous-frame current controller.
 *
 * Runs every PWM period inside the ADC ISR. Given measured phase currents,
 * rotor angle and speed, it drives id/iq to their references and produces
 * three duty cycles.
 */
#ifndef FOC_CONTROLLER_H
#define FOC_CONTROLLER_H

#include <stdbool.h>

#include "foc_math.h"
#include "pi_controller.h"

typedef struct {
    /* parameters */
    float rs, ld, lq, flux;
    float max_modulation;         /* fraction of linear SVPWM limit */
    bool decoupling_enabled;

    /* controllers */
    pi_ctrl_t pi_d, pi_q;

    /* references (written by medium loop, read by fast loop) */
    volatile float id_ref, iq_ref;

    /* state / outputs - read-only for other modules */
    dq_t i_dq;                    /* measured */
    dq_t v_dq;                    /* commanded after limiting */
    ab_t i_ab, v_ab;              /* stationary frame, v_ab is what observer uses */
    float duty[3];
    float modulation;             /* |v| / (vbus/sqrt3), 0..1 */
    float vbus;
    bool saturated;
} foc_ctrl_t;

void foc_ctrl_init(foc_ctrl_t *f, float rs, float ld, float lq, float flux,
                   float kp, float ki, float max_mod);
void foc_ctrl_reset(foc_ctrl_t *f);

/*
 * One control step. dt = PWM period. omega_e in rad/s electrical.
 * The output angle is advanced by BOARD_PWM_OUTPUT_DELAY_PERIODS*dt*omega
 * to compensate for the computation + PWM update delay.
 */
void foc_ctrl_step(foc_ctrl_t *f, abc_t i_abc, float theta_e, float omega_e,
                   float vbus, float dt);

/* Apply a fixed voltage vector (open loop / detection). Currents are still
 * measured and transformed so i_dq / i_ab stay valid. */
void foc_ctrl_apply_voltage(foc_ctrl_t *f, dq_t v, abc_t i_abc, float theta_e, float vbus);

#endif
