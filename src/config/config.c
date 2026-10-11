#include "config.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "safety_envelope.h"
#include "util_math.h"

app_config_t g_config;

void config_load_defaults(app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->size = sizeof(*cfg);

    /* Conservative placeholder motor: typical 10" 48 V scooter hub motor.
     * Run motor detection before riding - these are NOT your motor. */
    motor_config_t *m = &cfg->motor;
    m->rs_ohm = 0.080f;
    m->ld_h = 180e-6f;
    m->lq_h = 200e-6f;
    m->flux_wb = 0.0120f;
    m->pole_pairs = 15;
    m->direction = 1;
    for (int i = 0; i < 8; i++) m->hall_angle_rad[i] = -1.0f; /* run hall detection */
    m->ntc_r25 = 10000.0f;
    m->ntc_beta = 3950.0f;
    m->temp_motor_start_c = 100.0f;
    m->temp_motor_max_c = 120.0f;

    foc_config_t *f = &cfg->foc;
    f->current_bandwidth_hz = 1000.0f;
    f->current_kp = m->lq_h * M_2PI_F * f->current_bandwidth_hz;
    f->current_ki = m->rs_ohm * M_2PI_F * f->current_bandwidth_hz;
    f->speed_kp = 0.004f;   /* A per eRPM */
    f->speed_ki = 0.02f;
    f->observer_gain = 1500.0f;
    f->pll_kp = 2000.0f;
    f->pll_ki = 30000.0f;
    f->hall_observer_crossover_erpm = 3000.0f;
    f->hall_observer_hysteresis_erpm = 500.0f;
    f->fw_max_current = 0.0f;     /* opt-in: FW raises back-EMF risk on fault, see SAFETY.md */
    f->fw_start_duty = 0.90f;
    f->max_modulation = 0.95f;
    f->sensor_mode = SENSOR_MODE_HALL_OBSERVER;
    f->control_mode = CONTROL_MODE_TORQUE;

    limits_config_t *l = &cfg->limits;
    l->motor_current_max = 40.0f;
    l->motor_current_regen_max = 20.0f;
    l->battery_current_max = 25.0f;
    l->battery_current_regen_max = 8.0f;
    l->abs_current_max = 80.0f;
    l->battery_voltage_max = 54.6f + 1.0f;   /* 13S full + margin */
    l->battery_voltage_min = 39.0f;          /* 13S * 3.0 V */
    l->battery_voltage_cutback_start = 42.0f;
    l->overvoltage_trip = 60.0f;
    l->temp_fet_start_c = 85.0f;
    l->temp_fet_max_c = 100.0f;
    l->erpm_max = 20000.0f;
    l->current_ramp_a_per_s = 200.0f;

    vehicle_config_t *v = &cfg->vehicle;
    v->type = VEHICLE_ESCOOTER;
    v->wheel_diameter_m = 0.254f;            /* 10" */
    v->gear_ratio = 1.0f;
    v->speed_limit_kmh = 25.0f;              /* EU default; check your local law */
    v->speed_limit_margin_kmh = 2.0f;
    v->battery_cells_series = 13;
    v->throttle_v_min = 0.9f;
    v->throttle_v_max = 4.1f;
    v->throttle_v_fault_low = 0.4f;
    v->throttle_v_fault_high = 4.6f;
    v->throttle_expo = 0.3f;
    v->throttle_enabled = true;
    v->brake_switch_enabled = true;
    v->brake_analog_enabled = false;
    v->brake_v_min = 0.9f;
    v->brake_v_max = 4.1f;
    v->regen_on_brake_fraction = 0.5f;
    v->pas_magnets = 12;
    v->pas_levels = 5;
    const float pas_default[6] = {0.0f, 6.0f, 10.0f, 15.0f, 20.0f, 25.0f};
    for (int i = 0; i < 6; i++) v->pas_assist_current[i] = pas_default[i];
    v->pas_start_cadence_rpm = 10.0f;
    v->pas_stop_timeout_ms = 400;
    v->can_node_id = 1;
    v->can_bitrate = 500000;

    ride_config_t *r = &cfg->ride;
    r->boot_profile = PROFILE_SPORT;
    r->advanced_unlocked = 0;
    r->terms_version = 0;
    /* CUSTOM starts as a copy of the factory RACE values. */
    r->custom.motor_current_max = l->motor_current_max;
    r->custom.motor_current_regen_max = l->motor_current_regen_max;
    r->custom.battery_current_max = l->battery_current_max;
    r->custom.battery_current_regen_max = l->battery_current_regen_max;
    r->custom.abs_current_max = l->abs_current_max;
    r->custom.erpm_max = l->erpm_max;
    r->custom.current_ramp_a_per_s = l->current_ramp_a_per_s;
    r->custom.fw_max_current = f->fw_max_current;
    r->custom.speed_limit_kmh = v->speed_limit_kmh;
    r->custom.throttle_expo = v->throttle_expo;
}

/* Non-finite -> lo. Callers pass lo as the SAFE side (0 A, slowest ramp,
 * lowest trip voltage...). Fields where the low end is unsafe are handled
 * explicitly before this is used. */
static bool clampf_chk(float *x, float lo, float hi)
{
    if (!isfinite(*x)) {
        *x = lo;
        return false;
    }
    float c = clampf(*x, lo, hi);
    bool ok = (c == *x);
    *x = c;
    return ok;
}

bool config_validate(app_config_t *cfg)
{
    bool ok = true;
    limits_config_t *l = &cfg->limits;
    motor_config_t *m = &cfg->motor;
    foc_config_t *f = &cfg->foc;
    vehicle_config_t *v = &cfg->vehicle;

    ok &= clampf_chk(&l->abs_current_max, 1.0f, minf(BOARD_MAX_PHASE_CURRENT, ENV_ABS_TRIP_MAX));
    ok &= clampf_chk(&l->motor_current_max, 0.0f, l->abs_current_max / 1.05f);
    ok &= clampf_chk(&l->motor_current_regen_max, 0.0f, l->abs_current_max / 1.05f);
    ok &= clampf_chk(&l->battery_current_max, 0.0f, l->motor_current_max);
    ok &= clampf_chk(&l->battery_current_regen_max, 0.0f, l->motor_current_regen_max);
    ok &= clampf_chk(&l->overvoltage_trip, BOARD_MIN_BUS_VOLTAGE, BOARD_MAX_BUS_VOLTAGE);
    ok &= clampf_chk(&l->battery_voltage_max, BOARD_MIN_BUS_VOLTAGE, l->overvoltage_trip);
    ok &= clampf_chk(&l->battery_voltage_min, BOARD_MIN_BUS_VOLTAGE, l->battery_voltage_max);
    ok &= clampf_chk(&l->battery_voltage_cutback_start, l->battery_voltage_min, l->battery_voltage_max);
    ok &= clampf_chk(&l->temp_fet_max_c, 40.0f, 110.0f);
    ok &= clampf_chk(&l->temp_fet_start_c, 30.0f, l->temp_fet_max_c - 5.0f);
    ok &= clampf_chk(&l->erpm_max, 100.0f, 150000.0f);
    ok &= clampf_chk(&l->current_ramp_a_per_s, 10.0f, 5000.0f);

    ok &= clampf_chk(&m->rs_ohm, 1e-4f, 10.0f);
    ok &= clampf_chk(&m->ld_h, 1e-6f, 0.1f);
    ok &= clampf_chk(&m->lq_h, 1e-6f, 0.1f);
    ok &= clampf_chk(&m->flux_wb, 1e-4f, 1.0f);
    if (m->pole_pairs == 0 || m->pole_pairs > 60) { m->pole_pairs = 1; ok = false; }
    if (m->direction != 1 && m->direction != -1) { m->direction = 1; ok = false; }

    ok &= clampf_chk(&f->max_modulation, 0.5f, 0.95f);
    ok &= clampf_chk(&f->fw_start_duty, 0.5f, f->max_modulation);
    ok &= clampf_chk(&f->fw_max_current, 0.0f, l->motor_current_max);

    /* Low values here are UNSAFE (speed limit 0 = off; small wheel = the
     * speedometer under-reads and the limit fires late): NaN -> safe side. */
    if (!isfinite(v->speed_limit_kmh)) { v->speed_limit_kmh = 25.0f; ok = false; }
    if (!isfinite(v->wheel_diameter_m)) { v->wheel_diameter_m = 1.0f; ok = false; }
    ok &= clampf_chk(&v->wheel_diameter_m, 0.05f, 1.0f);
    ok &= clampf_chk(&v->gear_ratio, 0.1f, 50.0f);
    ok &= clampf_chk(&v->speed_limit_kmh, 0.0f, 120.0f);
    ok &= clampf_chk(&v->regen_on_brake_fraction, 0.0f, 1.0f);
    ok &= clampf_chk(&v->throttle_expo, 0.0f, 1.0f);
    /* Calibration values from flash: anything implausible is reset to a safe
     * default (hall angles -> "not detected", which blocks driving). */
    for (int i = 0; i < 8; i++) {
        float a = m->hall_angle_rad[i];
        if (!isfinite(a) || (a >= 0.0f ? a >= M_2PI_F : a != -1.0f)) {
            for (int k = 0; k < 8; k++) m->hall_angle_rad[k] = -1.0f;
            ok = false;
            break;
        }
    }
    if (!isfinite(m->encoder_offset_rad) || m->encoder_offset_rad < 0.0f || m->encoder_offset_rad >= M_2PI_F) {
        m->encoder_offset_rad = 0.0f;
        ok = false;
    }
    if (!(isfinite(v->throttle_v_fault_low) && isfinite(v->throttle_v_min) && isfinite(v->throttle_v_max) &&
          isfinite(v->throttle_v_fault_high) && v->throttle_v_fault_low >= 0.0f &&
          v->throttle_v_fault_low < v->throttle_v_min && v->throttle_v_min + 0.5f <= v->throttle_v_max &&
          v->throttle_v_max < v->throttle_v_fault_high && v->throttle_v_fault_high <= 5.5f)) {
        v->throttle_v_fault_low = 0.4f;
        v->throttle_v_min = 0.9f;
        v->throttle_v_max = 4.1f;
        v->throttle_v_fault_high = 4.6f;
        ok = false;
    }
    if (!(isfinite(v->brake_v_min) && isfinite(v->brake_v_max) && v->brake_v_min >= 0.0f &&
          v->brake_v_min + 0.5f <= v->brake_v_max && v->brake_v_max <= 5.5f)) {
        v->brake_v_min = 0.9f;
        v->brake_v_max = 4.1f;
        ok = false;
    }
    ok &= clampf_chk(&v->pas_start_cadence_rpm, 0.0f, 200.0f);
    if (v->throttle_enabled > 1) { v->throttle_enabled = 1; ok = false; }
    if (v->brake_switch_enabled > 1) { v->brake_switch_enabled = 1; ok = false; }  /* fail towards braking */
    if (v->brake_analog_enabled > 1) { v->brake_analog_enabled = 0; ok = false; }
    if ((unsigned)f->sensor_mode > SENSOR_MODE_ENCODER) { f->sensor_mode = SENSOR_MODE_HALL_OBSERVER; ok = false; }
    if ((unsigned)f->control_mode > CONTROL_MODE_DUTY) { f->control_mode = CONTROL_MODE_TORQUE; ok = false; }
    if ((unsigned)v->type > VEHICLE_EBIKE_THROTTLE) { v->type = VEHICLE_ESCOOTER; ok = false; }
    if (v->pas_levels > 5) { v->pas_levels = 5; ok = false; }
    if (v->pas_magnets == 0) { v->pas_magnets = 12; ok = false; }

    /* Factory limits and the CUSTOM profile must both sit inside the
     * envelope. A corrupted or tampered flash image gets clamped here. */
    ok &= envelope_clamp_limits(l);
    ok &= envelope_clamp_profile(&cfg->ride.custom);
    if (cfg->ride.boot_profile >= PROFILE_CUSTOM) { cfg->ride.boot_profile = PROFILE_SPORT; ok = false; }
    if (cfg->ride.advanced_unlocked > 1) { cfg->ride.advanced_unlocked = 0; ok = false; }

    return ok;
}
