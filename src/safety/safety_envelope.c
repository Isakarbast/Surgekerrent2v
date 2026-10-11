#include "safety_envelope.h"

#include <math.h>

#include "board_config.h"
#include "util_math.h"

/* Compile-time proof that the envelope fits the hardware: the trip level
 * must be measurable (below ADC clipping, with margin) and within the
 * board's rating, otherwise an overcurrent could go unseen. */
_Static_assert((int)ENV_ABS_TRIP_MAX < (int)(0.98f * BOARD_CURRENT_SENSE_RANGE_A),
               "ENV_ABS_TRIP_MAX must be below the current-sense ADC range");
_Static_assert((int)ENV_ABS_TRIP_MAX <= (int)BOARD_MAX_PHASE_CURRENT,
               "ENV_ABS_TRIP_MAX exceeds BOARD_MAX_PHASE_CURRENT");
_Static_assert((int)ENV_PHASE_CURRENT_MAX < (int)ENV_ABS_TRIP_MAX,
               "operating current ceiling must be below the trip ceiling");

static bool ok_range(float x, float lo, float hi) { return isfinite(x) && x >= lo && x <= hi; }

env_violation_t envelope_check_profile(const profile_params_t *p)
{
    const float *all = &p->motor_current_max;
    for (unsigned i = 0; i < sizeof(*p) / sizeof(float); i++)
        if (!isfinite(all[i]) || all[i] < 0.0f) return ENV_VIOL_NOT_FINITE;

    if (p->motor_current_max > ENV_PHASE_CURRENT_MAX) return ENV_VIOL_MOTOR_CURRENT;
    if (p->motor_current_regen_max > ENV_PHASE_CURRENT_MAX) return ENV_VIOL_REGEN_CURRENT;
    if (p->battery_current_max > ENV_BATTERY_CURRENT_MAX ||
        p->battery_current_max > p->motor_current_max) return ENV_VIOL_BATTERY_CURRENT;
    if (p->battery_current_regen_max > ENV_BATTERY_REGEN_MAX ||
        p->battery_current_regen_max > p->motor_current_regen_max) return ENV_VIOL_BATTERY_REGEN;
    if (p->abs_current_max > ENV_ABS_TRIP_MAX) return ENV_VIOL_ABS_TRIP;
    if (p->abs_current_max < 1.05f * maxf(p->motor_current_max, p->motor_current_regen_max))
        return ENV_VIOL_ABS_TRIP_MARGIN;
    if (!ok_range(p->erpm_max, 100.0f, ENV_ERPM_MAX)) return ENV_VIOL_ERPM;
    if (!ok_range(p->current_ramp_a_per_s, 10.0f, ENV_RAMP_MAX_A_PER_S)) return ENV_VIOL_RAMP;
    if (p->fw_max_current > ENV_FW_CURRENT_MAX || p->fw_max_current > p->motor_current_max)
        return ENV_VIOL_FIELD_WEAKENING;
    if (p->speed_limit_kmh > ENV_SPEED_LIMIT_MAX_KMH) return ENV_VIOL_SPEED_LIMIT;
    if (p->throttle_expo > 1.0f) return ENV_VIOL_THROTTLE_EXPO;
    return ENV_OK;
}

/* Every float in the config, so a NaN/Inf can't hide in a field that has
 * no dedicated range check. Keep in sync with config.h. */
static bool all_floats_finite(const app_config_t *c)
{
    const motor_config_t *m = &c->motor;
    const foc_config_t *f = &c->foc;
    const limits_config_t *l = &c->limits;
    const vehicle_config_t *v = &c->vehicle;
    const float scalars[] = {
        m->rs_ohm, m->ld_h, m->lq_h, m->flux_wb, m->encoder_offset_rad, m->ntc_r25, m->ntc_beta,
        m->temp_motor_start_c, m->temp_motor_max_c,
        f->current_kp, f->current_ki, f->current_bandwidth_hz, f->speed_kp, f->speed_ki,
        f->observer_gain, f->pll_kp, f->pll_ki, f->hall_observer_crossover_erpm,
        f->hall_observer_hysteresis_erpm, f->fw_max_current, f->fw_start_duty, f->max_modulation,
        l->motor_current_max, l->motor_current_regen_max, l->battery_current_max,
        l->battery_current_regen_max, l->abs_current_max, l->battery_voltage_max,
        l->battery_voltage_min, l->battery_voltage_cutback_start, l->overvoltage_trip,
        l->temp_fet_start_c, l->temp_fet_max_c, l->erpm_max, l->current_ramp_a_per_s,
        v->wheel_diameter_m, v->gear_ratio, v->speed_limit_kmh, v->speed_limit_margin_kmh,
        v->throttle_v_min, v->throttle_v_max, v->throttle_v_fault_low, v->throttle_v_fault_high,
        v->throttle_expo, v->brake_v_min, v->brake_v_max, v->regen_on_brake_fraction,
        v->pas_start_cadence_rpm,
    };
    for (unsigned i = 0; i < sizeof(scalars) / sizeof(scalars[0]); i++)
        if (!isfinite(scalars[i])) return false;
    for (int i = 0; i < 8; i++)
        if (!isfinite(m->hall_angle_rad[i])) return false;
    for (int i = 0; i < 6; i++)
        if (!isfinite(v->pas_assist_current[i])) return false;
    return true;
}

static bool voltage_window_ok(float fault_lo, float lo, float hi, float fault_hi)
{
    return fault_lo >= 0.0f && fault_lo < lo && lo + 0.5f <= hi && hi < fault_hi && fault_hi <= 5.5f;
}

env_violation_t envelope_check_config(const app_config_t *cfg)
{
    if (!all_floats_finite(cfg)) return ENV_VIOL_NOT_FINITE;
    env_violation_t v = envelope_check_profile(&cfg->ride.custom);
    if (v != ENV_OK) return v;

    const foc_config_t *f = &cfg->foc;
    if (!ok_range(f->max_modulation, 0.5f, 0.95f) || !ok_range(f->fw_start_duty, 0.5f, f->max_modulation))
        return ENV_VIOL_MODULATION;
    if (!ok_range(f->current_bandwidth_hz, 100.0f, ENV_CURRENT_BW_MAX_HZ)) return ENV_VIOL_CURRENT_BANDWIDTH;
    if (!ok_range(f->fw_max_current, 0.0f, ENV_FW_CURRENT_MAX)) return ENV_VIOL_FIELD_WEAKENING;
    if (!ok_range(f->current_kp, 1e-5f, 10.0f) || !ok_range(f->current_ki, 1e-3f, 1e5f) ||
        !ok_range(f->speed_kp, 0.0f, 1.0f) || !ok_range(f->speed_ki, 0.0f, 10.0f) ||
        !ok_range(f->observer_gain, 100.0f, 10000.0f) ||
        !ok_range(f->pll_kp, 10.0f, 20000.0f) || !ok_range(f->pll_ki, 10.0f, 1e7f) ||
        !ok_range(f->hall_observer_crossover_erpm, 300.0f, ENV_ERPM_MAX) ||
        !ok_range(f->hall_observer_hysteresis_erpm, 0.0f, f->hall_observer_crossover_erpm))
        return ENV_VIOL_CONTROLLER_GAINS;

    const motor_config_t *m = &cfg->motor;
    if (!ok_range(m->rs_ohm, 1e-4f, 10.0f) || !ok_range(m->ld_h, 1e-6f, 0.1f) ||
        !ok_range(m->lq_h, 1e-6f, 0.1f) || !ok_range(m->flux_wb, 1e-4f, 1.0f) ||
        !ok_range(m->temp_motor_max_c, 40.0f, 150.0f) ||
        !ok_range(m->temp_motor_start_c, 30.0f, m->temp_motor_max_c))
        return ENV_VIOL_MOTOR_PARAMS;

    for (int i = 0; i < 6; i++)
        if (!ok_range(cfg->vehicle.pas_assist_current[i], 0.0f, ENV_PHASE_CURRENT_MAX)) return ENV_VIOL_PAS_CURRENT;

    for (int i = 0; i < 8; i++) {
        float a = m->hall_angle_rad[i];
        if (a >= 0.0f ? a >= M_2PI_F : a != -1.0f) return ENV_VIOL_SENSOR_CALIBRATION; /* -1 = unused */
    }
    if (!ok_range(m->encoder_offset_rad, 0.0f, M_2PI_F)) return ENV_VIOL_SENSOR_CALIBRATION;

    const vehicle_config_t *vc = &cfg->vehicle;
    if (!voltage_window_ok(vc->throttle_v_fault_low, vc->throttle_v_min, vc->throttle_v_max,
                           vc->throttle_v_fault_high) ||
        !ok_range(vc->brake_v_min, 0.0f, 5.0f) || !ok_range(vc->brake_v_max, vc->brake_v_min + 0.5f, 5.5f) ||
        !ok_range(vc->throttle_expo, 0.0f, 1.0f))
        return ENV_VIOL_INPUT_CALIBRATION;
    if (!ok_range(vc->regen_on_brake_fraction, 0.0f, 1.0f) || !ok_range(vc->pas_start_cadence_rpm, 0.0f, 200.0f) ||
        vc->pas_magnets == 0u || vc->pas_levels > 5u ||
        (m->ntc_r25 != 0.0f && (!ok_range(m->ntc_r25, 100.0f, 1e6f) || !ok_range(m->ntc_beta, 1000.0f, 10000.0f))))
        return ENV_VIOL_VEHICLE_PARAMS;

    if (vc->throttle_enabled > 1u || vc->brake_switch_enabled > 1u || vc->brake_analog_enabled > 1u ||
        (unsigned)f->sensor_mode > SENSOR_MODE_ENCODER || (unsigned)f->control_mode > CONTROL_MODE_DUTY ||
        (unsigned)vc->type > VEHICLE_EBIKE_THROTTLE || (m->direction != 1 && m->direction != -1) ||
        cfg->ride.boot_profile >= PROFILE_CUSTOM)
        return ENV_VIOL_INVALID_ENUM;
    return ENV_OK;
}

static bool clamp(float *x, float lo, float hi)
{
    float c = isfinite(*x) ? clampf(*x, lo, hi) : lo;
    bool same = (c == *x);
    *x = c;
    return same;
}

bool envelope_clamp_profile(profile_params_t *p)
{
    bool ok = true;
    ok &= clamp(&p->motor_current_max, 0.0f, ENV_PHASE_CURRENT_MAX);
    ok &= clamp(&p->motor_current_regen_max, 0.0f, ENV_PHASE_CURRENT_MAX);
    ok &= clamp(&p->battery_current_max, 0.0f, minf(ENV_BATTERY_CURRENT_MAX, p->motor_current_max));
    ok &= clamp(&p->battery_current_regen_max, 0.0f, minf(ENV_BATTERY_REGEN_MAX, p->motor_current_regen_max));
    ok &= clamp(&p->abs_current_max, 1.05f * maxf(p->motor_current_max, p->motor_current_regen_max),
                ENV_ABS_TRIP_MAX);
    /* If the trip can't sit 5 % above the operating current, pull the operating current down. */
    float op_max = p->abs_current_max / 1.05f;
    ok &= clamp(&p->motor_current_max, 0.0f, op_max);
    ok &= clamp(&p->motor_current_regen_max, 0.0f, op_max);
    ok &= clamp(&p->erpm_max, 100.0f, ENV_ERPM_MAX);
    ok &= clamp(&p->current_ramp_a_per_s, 10.0f, ENV_RAMP_MAX_A_PER_S);
    ok &= clamp(&p->fw_max_current, 0.0f, minf(ENV_FW_CURRENT_MAX, p->motor_current_max));
    ok &= clamp(&p->speed_limit_kmh, 0.0f, ENV_SPEED_LIMIT_MAX_KMH);
    ok &= clamp(&p->throttle_expo, 0.0f, 1.0f);
    return ok;
}

bool envelope_clamp_limits(limits_config_t *l)
{
    bool ok = true;
    ok &= clamp(&l->motor_current_max, 0.0f, ENV_PHASE_CURRENT_MAX);
    ok &= clamp(&l->motor_current_regen_max, 0.0f, ENV_PHASE_CURRENT_MAX);
    ok &= clamp(&l->battery_current_max, 0.0f, ENV_BATTERY_CURRENT_MAX);
    ok &= clamp(&l->battery_current_regen_max, 0.0f, ENV_BATTERY_REGEN_MAX);
    ok &= clamp(&l->abs_current_max, 1.0f, ENV_ABS_TRIP_MAX);
    ok &= clamp(&l->erpm_max, 100.0f, ENV_ERPM_MAX);
    ok &= clamp(&l->current_ramp_a_per_s, 10.0f, ENV_RAMP_MAX_A_PER_S);
    return ok;
}

const char *envelope_violation_name(env_violation_t v)
{
    static const char *names[ENV_VIOL_COUNT] = {
        "ok", "not_finite", "motor_current", "regen_current", "battery_current",
        "battery_regen", "abs_trip", "abs_trip_margin", "erpm", "ramp", "field_weakening",
        "speed_limit", "throttle_expo", "modulation", "current_bandwidth", "controller_gains",
        "motor_params", "pas_current", "invalid_enum", "sensor_calibration",
        "input_calibration", "vehicle_params",
    };
    return (v < ENV_VIOL_COUNT) ? names[v] : "?";
}
