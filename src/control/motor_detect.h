/*
 * motor_detect.h - automatic motor parameter identification.
 *
 * Sequence (wheel MUST be off the ground - the motor spins):
 *   1. ALIGN      lock rotor to theta = 0 with DC current on d
 *   2. RESIST     two-point DC measurement R = dV/dI (cancels dead-time drop)
 *   3. IND_D/Q    high-frequency +-V square wave, L = V*dt/|di|
 *   4. ENC_OFFSET (encoder only) read mech angle while aligned
 *   5. HALL       slow open-loop rotation forward + backward, map each hall
 *                 code to its mean angle (averaging both directions cancels
 *                 the load-angle lag)
 *   6. FLUX       open-loop spin-up, psi = |v - R*i - jwL*i| / w
 *
 * Runs inside the fast loop. It only issues commands; motor_control
 * executes them and keeps every protection active. Any drive-blocking
 * fault, the brake lever or the throttle aborts it immediately.
 */
#ifndef MOTOR_DETECT_H
#define MOTOR_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_math.h"

typedef enum {
    DET_IDLE = 0,
    DET_ALIGN,
    DET_RESIST_1,
    DET_RESIST_2,
    DET_IND_D,
    DET_IND_Q,
    DET_ENC_OFFSET,
    DET_HALL_FWD,
    DET_HALL_REV,
    DET_FLUX_RAMP,
    DET_FLUX_MEASURE,
    DET_FLUX_RAMP_DOWN,
    DET_DONE,
    DET_ERROR,
} detect_state_t;

typedef enum {
    DET_ERR_NONE = 0,
    DET_ERR_R_RANGE,
    DET_ERR_L_RANGE,
    DET_ERR_HALL_CODES,
    DET_ERR_FLUX_RANGE,
    DET_ERR_ABORTED,
} detect_error_t;

typedef struct {
    /* inputs, filled by motor_control each period */
    dq_t i_dq;          /* measured in the frame of the last command */
    dq_t v_dq;          /* voltage actually commanded last period */
    float vbus;
    uint8_t hall_code;
    float enc_mech;     /* rad, or < 0 if no encoder */
} detect_in_t;

typedef struct {
    bool current_mode;  /* true: track i_ref; false: apply v directly */
    float theta, omega;
    dq_t i_ref;
    dq_t v;
} detect_cmd_t;

typedef struct {
    detect_state_t state;
    detect_error_t error;
    float dt;
    float i_test;       /* DC test current [A] */
    float omega_flux;   /* target speed for flux measurement [rad/s el] */
    bool do_hall, do_encoder;
    uint8_t pole_pairs;

    uint32_t t;         /* samples in current state */
    double acc_v, acc_i, acc_n;
    float v1, i1;
    float v_inj;
    float di_acc;
    float prev_i;
    float theta, omega;
    float hall_s[8], hall_c[8];
    uint32_t hall_n[8];
    double acc_vd, acc_vq, acc_id, acc_iq;

    /* results */
    float rs, ld, lq, flux;
    float hall_angle[8];
    float enc_offset;
} motor_detect_t;

void detect_start(motor_detect_t *d, float i_test, float dt, uint8_t pole_pairs,
                  bool do_hall, bool do_encoder);
void detect_abort(motor_detect_t *d);
/* One fast-loop step. Returns false once finished (DONE or ERROR). */
bool detect_step(motor_detect_t *d, const detect_in_t *in, detect_cmd_t *cmd);
const char *detect_state_name(detect_state_t s);

#endif
