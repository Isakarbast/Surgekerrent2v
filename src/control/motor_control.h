/*
 * motor_control.h - the fast (PWM-rate) control loop and inverter state.
 *
 * Data flow each PWM period (ADC ISR):
 *   ADC -> current_sense / voltage_sense -> fast protections
 *       -> hall / encoder / flux observer -> angle selection
 *       -> [RUN] field weakening + FOC current loop
 *          [DETECT] motor_detect command
 *       -> dead-time compensation -> PWM
 *
 * The medium loop (vehicle_control) only writes mc_set_current_command()
 * and mc_request_*(); everything that touches the inverter happens here.
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "as5047p.h"
#include "config.h"
#include "current_sense.h"
#include "field_weakening.h"
#include "flux_observer.h"
#include "foc_controller.h"
#include "hall_sensor.h"
#include "motor_detect.h"
#include "voltage_sense.h"

typedef enum {
    MC_STATE_CALIBRATING = 0,
    MC_STATE_IDLE,        /* outputs Hi-Z, estimators tracking */
    MC_STATE_RUN,         /* closed-loop current control */
    MC_STATE_DETECT,
    MC_STATE_FAULT,       /* outputs Hi-Z until faults cleared */
} mc_state_t;

typedef enum {
    MC_ANGLE_NONE = 0,
    MC_ANGLE_HALL,
    MC_ANGLE_OBSERVER,
    MC_ANGLE_ENCODER,
} mc_angle_src_t;

typedef struct {
    volatile mc_state_t state;
    const app_config_t *cfg;
    float dt;

    foc_ctrl_t foc;
    flux_observer_t obs;
    hall_sensor_t hall;
    as5047p_t enc;
    current_sense_t cs;
    voltage_sense_t vs;
    field_weakening_t fw;
    motor_detect_t det;
    float det_theta;           /* frame of the last detection command */

    abc_t i_abc;
    float duty_applied[3];     /* duties active during the latest sample */
    ab_t v_ab_prev;            /* voltage commanded one ISR earlier (still active for half a period) */
    float theta, omega;        /* selected electrical angle / speed */
    mc_angle_src_t angle_src;
    float deadtime_comp;       /* duty fraction */
    uint16_t obs_lock_count;
    bool obs_locked;           /* observer converged (flux magnitude plausible at speed) */

    /* commands from the medium loop */
    volatile float iq_cmd;
    volatile bool run_req, detect_req, detect_abort_req;

    /* filtered telemetry (updated in fast loop, read anywhere) */
    float id_filt, iq_filt, vq_abs_filt, ibatt_filt, erpm_filt, i_phase_peak;
    float fast_alpha;

    /* timing */
    volatile uint32_t loop_count;
    uint32_t loop_cycles, loop_cycles_max;
} motor_control_t;

extern motor_control_t g_mc;

void mc_init(const app_config_t *cfg);
/* Re-derive controller params after a config change (motor must be idle). */
void mc_apply_config(const app_config_t *cfg);
void mc_fast_loop(void);

void mc_set_current_command(float iq);
void mc_request_run(bool run);
bool mc_request_detection(void);
void mc_abort_detection(void);  /* e.g. rider pulled the brake */
void mc_emergency_stop(void);  /* Hi-Z immediately, from any context */

static inline float mc_erpm(void) { return g_mc.erpm_filt; }
static inline bool mc_is_stopped(void) { return g_mc.erpm_filt < 100.0f && g_mc.erpm_filt > -100.0f; }

#endif
