#include "motor_control.h"

#include <math.h>

#include "board_config.h"
#include "fault_manager.h"
#include "hal.h"
#include "util_math.h"

motor_control_t g_mc;

#define RAD_S_TO_ERPM  (60.0f / M_2PI_F)
#define CAL_SAMPLES    4096u
#define OBS_FAULT_FLUX_ERR 0.5f
#define OBS_LOCK_FLUX_ERR  0.2f
#define OBS_LOCK_SAMPLES   1000u   /* 50 ms at 20 kHz */
#define HANDOVER_MAX_ERR_RAD (20.0f * M_PI_F / 180.0f)

static void outputs_off(motor_control_t *m)
{
    hal_pwm_outputs_disable();
    foc_ctrl_reset(&m->foc);
    m->fw.id_fw = 0.0f;
}

void mc_emergency_stop(void)
{
    hal_pwm_outputs_disable();
    g_mc.state = MC_STATE_FAULT;
}

void mc_apply_config(const app_config_t *cfg)
{
    motor_control_t *m = &g_mc;
    const motor_config_t *mo = &cfg->motor;
    const foc_config_t *fc = &cfg->foc;
    m->cfg = cfg;

    foc_ctrl_init(&m->foc, mo->rs_ohm, mo->ld_h, mo->lq_h, mo->flux_wb,
                  fc->current_kp, fc->current_ki, fc->max_modulation);
    flux_obs_init(&m->obs, mo->rs_ohm, mo->ld_h, mo->lq_h, mo->flux_wb,
                  fc->observer_gain, fc->pll_kp, fc->pll_ki);
    hall_init(&m->hall, mo->hall_angle_rad, (float)hal_cycles_per_us() * 1e6f);
    fw_init(&m->fw, mo->ld_h, mo->lq_h, mo->flux_wb, fc->fw_max_current, fc->fw_start_duty);
#if BOARD_HAS_SPI_ENCODER
    if (fc->sensor_mode == SENSOR_MODE_ENCODER)
        as5047p_init(&m->enc, mo->pole_pairs, mo->direction, mo->encoder_offset_rad);
#endif
}

void mc_init(const app_config_t *cfg)
{
    motor_control_t *m = &g_mc;
    m->dt = 1.0f / hal_pwm_frequency_hz();
    m->state = MC_STATE_CALIBRATING;
    m->iq_cmd = 0.0f;
    m->run_req = false;
    m->detect_req = false;
    m->fast_alpha = lpf_alpha(50.0f, m->dt);
    m->deadtime_comp = BOARD_DEADTIME_NS * 1e-9f * hal_pwm_frequency_hz();
    m->duty_applied[0] = m->duty_applied[1] = m->duty_applied[2] = 0.5f;

    float max_duty_sample = 1.0f - 2.0f * BOARD_MIN_LOWSIDE_SAMPLE_NS * 1e-9f * hal_pwm_frequency_hz();
    cs_init(&m->cs, hal_adc_vref() / BOARD_ADC_MAX_COUNTS * BOARD_AMPS_PER_VOLT, max_duty_sample);
    vs_init(&m->vs, hal_adc_vref(), BOARD_VBUS_SCALE, m->dt);
    mc_apply_config(cfg);

    hal_pwm_outputs_disable();
    cs_calibrate_start(&m->cs, CAL_SAMPLES);
}

void mc_set_current_command(float iq) { g_mc.iq_cmd = iq; }
void mc_abort_detection(void) { g_mc.detect_abort_req = true; }
void mc_request_run(bool run) { g_mc.run_req = run; }

bool mc_request_detection(void)
{
    if (g_mc.state != MC_STATE_IDLE || !mc_is_stopped()) return false;
    g_mc.detect_req = true;
    return true;
}

/* ---------------------------------------------------------------------- */

static bool fast_protections(motor_control_t *m)
{
    const limits_config_t *l = &m->cfg->limits;
    float trip = l->abs_current_max;

    if (absf(m->i_abc.a) > trip || absf(m->i_abc.b) > trip || absf(m->i_abc.c) > trip) {
        fault_raise(FAULT_OVERCURRENT_SW);
        return false;
    }
    if (hal_pwm_hw_break_tripped()) {
        fault_raise(FAULT_OVERCURRENT_HW);
        return false;
    }
    if (m->vs.vbus > l->overvoltage_trip) {
        fault_raise(FAULT_OVERVOLTAGE);
        return false;
    }
    if (m->cs.status == CS_ERR_SUM || m->cs.status == CS_ERR_OFFSET_RANGE) {
        fault_raise(FAULT_CURRENT_SENSE);
        return false;
    }
    return true;
}

static void update_angle(motor_control_t *m)
{
    const foc_config_t *fc = &m->cfg->foc;

#if BOARD_HAS_HALL
    hall_update(&m->hall, hal_hall_read(), hal_cycles());
#endif
#if BOARD_HAS_SPI_ENCODER
    if (fc->sensor_mode == SENSOR_MODE_ENCODER) as5047p_update(&m->enc, m->dt);
#endif

    /* Observer only has valid voltage information while we're switching. */
    if (hal_pwm_outputs_enabled()) {
        /* Since the last ISR the bridge applied the older command for the first
         * half-period and the newer one for the second half. */
        ab_t v_mean = {0.5f * (m->foc.v_ab.alpha + m->v_ab_prev.alpha),
                       0.5f * (m->foc.v_ab.beta + m->v_ab_prev.beta)};
        flux_obs_update(&m->obs, v_mean, m->foc.i_ab, m->foc.i_dq.d, m->dt);
        float erpm_obs = absf(flux_obs_omega(&m->obs)) * RAD_S_TO_ERPM;
        bool plausible = absf(m->obs.flux_err) < OBS_LOCK_FLUX_ERR &&
                         erpm_obs > 0.5f * fc->hall_observer_crossover_erpm;
        if (!plausible) m->obs_lock_count = 0;
        else if (m->obs_lock_count < OBS_LOCK_SAMPLES) m->obs_lock_count++;
        m->obs_locked = m->obs_lock_count >= OBS_LOCK_SAMPLES;
    } else {
        m->obs_lock_count = 0;
        m->obs_locked = false;
        if (m->hall.valid) flux_obs_reset(&m->obs, m->hall.theta, m->hall.omega);
    }

    switch (fc->sensor_mode) {
    case SENSOR_MODE_HALL:
        m->angle_src = MC_ANGLE_HALL;
        break;
    case SENSOR_MODE_SENSORLESS:
        m->angle_src = MC_ANGLE_OBSERVER;
        break;
    case SENSOR_MODE_ENCODER:
        m->angle_src = MC_ANGLE_ENCODER;
        break;
    case SENSOR_MODE_HALL_OBSERVER:
    default: {
        float erpm = absf(flux_obs_omega(&m->obs)) * RAD_S_TO_ERPM;
        float x = fc->hall_observer_crossover_erpm;
        float h = 0.5f * fc->hall_observer_hysteresis_erpm;
        /* Hand over only when the observer is locked AND agrees with the
         * halls (hall interpolation is good to ~+-10 deg at this speed). A
         * mismatch means wrong motor params: stay on halls. */
        float agree = absf(wrap_pi(flux_obs_angle(&m->obs) - m->hall.theta));
        if (m->angle_src == MC_ANGLE_OBSERVER) {
            if (erpm < x - h || !m->obs_locked) m->angle_src = MC_ANGLE_HALL;
        } else if (erpm > x + h && m->obs_locked && agree < HANDOVER_MAX_ERR_RAD) {
            m->angle_src = MC_ANGLE_OBSERVER;
        } else {
            m->angle_src = MC_ANGLE_HALL;
        }
        /* Hall failure at speed: stay on the observer if it is healthy. */
        if (m->hall.fault && erpm > x) m->angle_src = MC_ANGLE_OBSERVER;
        break;
    }
    }

    switch (m->angle_src) {
    case MC_ANGLE_OBSERVER:
        m->theta = flux_obs_angle(&m->obs);
        m->omega = flux_obs_omega(&m->obs);
        break;
    case MC_ANGLE_ENCODER:
        m->theta = m->enc.elec_angle;
        m->omega = m->enc.pll.omega;
        break;
    case MC_ANGLE_HALL:
    default:
        m->theta = m->hall.theta;
        m->omega = m->hall.omega;
        break;
    }
}

static void deadtime_compensate(motor_control_t *m, float duty[3])
{
    /* Dead time steals (sign(i) * t_dead * f_pwm) of duty. Smooth sign
     * around zero current avoids chatter. */
    const float band = 0.02f * m->cfg->limits.motor_current_max + 0.5f;
    const float *i = &m->i_abc.a;
    for (int k = 0; k < 3; k++) {
        duty[k] = clampf(duty[k] + m->deadtime_comp * clampf(i[k] / band, -1.0f, 1.0f), 0.0f, 1.0f);
    }
}

static void run_detection(motor_control_t *m, float vbus)
{
    /* Fresh sample, in the frame of the command that produced it. */
    float sn, cs;
    fast_sincosf(m->det_theta, &sn, &cs);
    detect_in_t in = {
        .i_dq = foc_park(m->foc.i_ab, sn, cs),
        .v_dq = m->foc.v_dq,
        .vbus = vbus,
        .hall_code = hal_hall_read(),
        .enc_mech = (m->cfg->foc.sensor_mode == SENSOR_MODE_ENCODER) ? m->enc.mech_angle : -1.0f,
    };
    detect_cmd_t cmd = {0};
    bool running = detect_step(&m->det, &in, &cmd);
    m->det_theta = cmd.theta;

    if (!running) {
        outputs_off(m);
        if (m->det.state == DET_ERROR) fault_raise(FAULT_DETECTION);
        m->state = (m->det.state == DET_DONE) ? MC_STATE_IDLE : MC_STATE_FAULT;
        return;
    }
    if (cmd.current_mode) {
        m->foc.decoupling_enabled = false;
        m->foc.id_ref = cmd.i_ref.d;
        m->foc.iq_ref = cmd.i_ref.q;
        foc_ctrl_step(&m->foc, m->i_abc, cmd.theta, 0.0f, vbus, m->dt);
    } else {
        foc_ctrl_apply_voltage(&m->foc, cmd.v, m->i_abc, cmd.theta, vbus);
    }
}

void mc_fast_loop(void)
{
    motor_control_t *m = &g_mc;
    uint32_t t0 = hal_cycles();
    hal_adc_fast_t raw;
    hal_adc_read_fast(&raw);
    float vbus = vs_update(&m->vs, raw.vbus);

    if (m->state == MC_STATE_CALIBRATING) {
        if (cs_calibrate_sample(&m->cs, &raw)) {
            if (m->cs.status != CS_OK) {
                fault_raise(FAULT_CURRENT_SENSE);
                m->state = MC_STATE_FAULT;
            } else {
                m->state = MC_STATE_IDLE;
            }
        }
        m->loop_count++;
        return;
    }

    m->i_abc = cs_convert(&m->cs, &raw, m->duty_applied);
    if (m->cfg->motor.direction < 0) { /* reverse = swap B and C */
        float t = m->i_abc.b;
        m->i_abc.b = m->i_abc.c;
        m->i_abc.c = t;
    }

    if (!fast_protections(m) && m->state != MC_STATE_FAULT) {
        outputs_off(m);
        if (m->state == MC_STATE_DETECT) detect_abort(&m->det);
        m->state = MC_STATE_FAULT;
    }

    /* Keep i_ab current for the observer even when not in RUN. */
    m->foc.i_ab = foc_clarke3(m->i_abc);
    ab_t v_ab_last = m->foc.v_ab;
    update_angle(m);

    switch (m->state) {
    case MC_STATE_IDLE:
        if (m->detect_req) {
            m->detect_req = false;
            /* From the FACTORY limits, not the ride profile: detection results
             * must not depend on whether the rider is in ECO or RACE. */
            float i_test = minf(0.3f * g_config.limits.motor_current_max, 15.0f);
            m->det_theta = 0.0f;
            m->detect_abort_req = false;
            detect_start(&m->det, i_test, m->dt, m->cfg->motor.pole_pairs,
                         BOARD_HAS_HALL != 0,
                         m->cfg->foc.sensor_mode == SENSOR_MODE_ENCODER);
            foc_ctrl_reset(&m->foc);
            hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
            hal_pwm_outputs_enable();
            m->state = MC_STATE_DETECT;
        } else if (m->run_req && !fault_blocks_drive()) {
            /* Back-EMF is supplied by the decoupling feed-forward, so
             * enabling while the wheel is already spinning is bumpless. */
            foc_ctrl_reset(&m->foc);
            m->foc.decoupling_enabled = true;
            hal_pwm_set_duty(0.5f, 0.5f, 0.5f);
            hal_pwm_outputs_enable();
            m->state = MC_STATE_RUN;
        }
        break;

    case MC_STATE_RUN: {
        if (!m->run_req || fault_blocks_drive()) {
            outputs_off(m);
            m->state = fault_blocks_drive() ? MC_STATE_FAULT : MC_STATE_IDLE;
            break;
        }
        if (m->angle_src == MC_ANGLE_HALL && !m->hall.valid) {
            fault_raise(FAULT_HALL);
            outputs_off(m);
            m->state = MC_STATE_FAULT;
            break;
        }
        if (m->angle_src == MC_ANGLE_OBSERVER && m->obs_locked &&
            absf(m->obs.flux_err) > OBS_FAULT_FLUX_ERR) {
            fault_raise(FAULT_OBSERVER);
            outputs_off(m);
            m->state = MC_STATE_FAULT;
            break;
        }
        /* Pure sensorless has no standstill start: the rider pushes off,
         * we regulate zero current until the observer locks, then apply torque. */
        float iq = m->iq_cmd;
        if (m->cfg->foc.sensor_mode == SENSOR_MODE_SENSORLESS && !m->obs_locked) iq = 0.0f;
        m->foc.iq_ref = iq;
        m->foc.id_ref = fw_update(&m->fw, iq, m->foc.modulation, m->dt);
        foc_ctrl_step(&m->foc, m->i_abc, m->theta, m->omega, vbus, m->dt);
        break;
    }

    case MC_STATE_DETECT:
        /* Any blocking fault from the slower loops, or a rider abort
         * (brake / throttle), stops a spinning detection immediately. */
        if (m->detect_abort_req || fault_blocks_drive()) {
            m->detect_abort_req = false;
            detect_abort(&m->det);
        }
        run_detection(m, vbus);
        break;

    case MC_STATE_FAULT:
    default:
        hal_pwm_outputs_disable();
        if (!fault_blocks_drive() && !m->run_req) m->state = MC_STATE_IDLE;
        break;
    }

    /* Last line of defence: a non-finite value must never reach the timer.
     * (Config validation should make this unreachable.) */
    if (hal_pwm_outputs_enabled() &&
        !(isfinite(m->foc.duty[0]) && isfinite(m->foc.duty[1]) && isfinite(m->foc.duty[2]))) {
        fault_raise(FAULT_NUMERIC);
        outputs_off(m);
        m->state = MC_STATE_FAULT;
    }

    if (hal_pwm_outputs_enabled() && (m->state == MC_STATE_RUN || m->state == MC_STATE_DETECT)) {
        float d[3] = {m->foc.duty[0], m->foc.duty[1], m->foc.duty[2]};
        deadtime_compensate(m, d);
        if (m->cfg->motor.direction < 0) {
            float t = d[1];
            d[1] = d[2];
            d[2] = t;
        }
        hal_pwm_set_duty(d[0], d[1], d[2]);
        m->duty_applied[0] = d[0];
        m->duty_applied[1] = d[1];
        m->duty_applied[2] = d[2];
    } else {
        m->duty_applied[0] = m->duty_applied[1] = m->duty_applied[2] = 0.5f;
        m->foc.v_ab.alpha = m->foc.v_ab.beta = 0.0f;
    }

    m->v_ab_prev = v_ab_last;

    /* Telemetry filters */
    float a = m->fast_alpha;
    lpf_update(&m->id_filt, m->foc.i_dq.d, a);
    lpf_update(&m->iq_filt, m->foc.i_dq.q, a);
    lpf_update(&m->vq_abs_filt, absf(m->foc.v_dq.q), a);
    float p = 1.5f * (m->foc.v_dq.d * m->foc.i_dq.d + m->foc.v_dq.q * m->foc.i_dq.q);
    lpf_update(&m->ibatt_filt, (vbus > 1.0f && hal_pwm_outputs_enabled()) ? p / vbus : 0.0f, a);
    lpf_update(&m->erpm_filt, m->omega * RAD_S_TO_ERPM, a);
    float ipk = maxf(absf(m->i_abc.a), maxf(absf(m->i_abc.b), absf(m->i_abc.c)));
    m->i_phase_peak = maxf(ipk, m->i_phase_peak * 0.9995f);

    m->loop_count++;
    m->loop_cycles = hal_cycles() - t0;
    if (m->loop_cycles > m->loop_cycles_max) m->loop_cycles_max = m->loop_cycles;
}
