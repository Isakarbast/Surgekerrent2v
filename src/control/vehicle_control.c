#include "vehicle_control.h"

#include <math.h>

#include "app.h"
#include "board_config.h"
#include "can_bus.h"
#include "fault_manager.h"
#include "gate_driver.h"
#include "hal.h"
#include "motor_control.h"
#include "util_math.h"

vehicle_t g_vehicle;

#define DT                 BOARD_MEDIUM_LOOP_DT
#define REGEN_FADE_ERPM    500.0f   /* regen fades to 0 below this: never drive backwards */
#define COAST_DELAY_MS     500u     /* keep PWM on briefly after zero command */
#define STALL_TIME_MS      3000u
#define STALL_ERPM         60.0f

float vehicle_erpm_to_kmh(const app_config_t *cfg, float erpm)
{
    float mech_rpm = erpm / (float)cfg->motor.pole_pairs / cfg->vehicle.gear_ratio;
    return mech_rpm * M_PI_F * cfg->vehicle.wheel_diameter_m * 60.0f / 1000.0f;
}

void vehicle_init(const app_config_t *cfg)
{
    vehicle_t *v = &g_vehicle;
    const vehicle_config_t *vc = &cfg->vehicle;
    v->cfg = cfg;
    throttle_init(&v->throttle, vc->throttle_v_min, vc->throttle_v_max,
                  vc->throttle_v_fault_low, vc->throttle_v_fault_high, vc->throttle_expo);
    brake_init(&v->brake, vc->brake_switch_enabled, vc->brake_analog_enabled,
               vc->brake_v_min, vc->brake_v_max);
    pas_init(&v->pas, vc->pas_magnets, vc->pas_start_cadence_rpm, vc->pas_stop_timeout_ms);
    ntc_init(&v->ntc_fet, BOARD_NTC_FET_PULLUP, BOARD_NTC_FET_R25, BOARD_NTC_FET_BETA);
    ntc_init(&v->ntc_motor, BOARD_NTC_MOTOR_PULLUP, cfg->motor.ntc_r25, cfg->motor.ntc_beta);
    pi_init(&v->speed_pi, cfg->foc.speed_kp, cfg->foc.speed_ki);
    v->pas_level = (vc->type == VEHICLE_EBIKE_PAS) ? 1 : 0;
    v->iq_target = v->iq_cmd = 0.0f;
    v->idle_ms = COAST_DELAY_MS;
    v->last_fast_count = g_mc.loop_count;
}

static float adc_volts(hal_adc_slow_ch_t ch)
{
    return (float)hal_adc_read_slow(ch) / BOARD_ADC_MAX_COUNTS * hal_adc_vref();
}

/* ---------------- slow protections ---------------- */

static void check_supervision(vehicle_t *v)
{
    /* Fast loop alive? If the ADC ISR stops, nothing else protects the FETs. */
    if (g_mc.loop_count == v->last_fast_count) {
        if (++v->fast_dead_ms >= 2 && g_mc.state != MC_STATE_CALIBRATING) {
            mc_emergency_stop();
            fault_raise(FAULT_LOOP_OVERRUN);
        }
    } else {
        v->fast_dead_ms = 0;
    }
    v->last_fast_count = g_mc.loop_count;

    /* Fast loop must finish within 80 % of the PWM period. */
    float budget = 0.8f * (float)hal_cycles_per_us() * 1e6f / hal_pwm_frequency_hz();
    if ((float)g_mc.loop_cycles_max > budget) fault_raise(FAULT_LOOP_OVERRUN);

    /* Gate driver: react to nFAULT immediately, full SPI check at 10 Hz. */
    const gate_driver_t *gd = gate_driver_get();
    bool poll = hal_gate_fault_pin();
    if (++v->gd_poll_ms >= 100) {
        v->gd_poll_ms = 0;
        poll = true;
    }
    if (poll) {
        uint32_t f = gd->read_faults();
        if (f & GD_FAULT_FATAL_MASK) {
            mc_emergency_stop();
            fault_raise(FAULT_GATE_DRIVER);
        }
    }
}

static void check_voltage_temp(vehicle_t *v)
{
    const limits_config_t *l = &v->cfg->limits;
    float vbus = g_mc.vs.vbus_filt;

    if (vbus < l->battery_voltage_min) fault_raise(FAULT_UNDERVOLTAGE);
    else if (vbus > l->battery_voltage_min + 1.0f) fault_clear_auto(FAULT_UNDERVOLTAGE);

    if (v->ntc_fet.temp_c > l->temp_fet_max_c + 5.0f) fault_raise(FAULT_FET_OVERTEMP);
    else if (v->ntc_fet.temp_c < l->temp_fet_start_c) fault_clear_auto(FAULT_FET_OVERTEMP);

    const motor_config_t *m = &v->cfg->motor;
    if (m->ntc_r25 > 0.0f) {
        if (v->ntc_motor.temp_c > m->temp_motor_max_c + 5.0f) fault_raise(FAULT_MOTOR_OVERTEMP);
        else if (v->ntc_motor.temp_c < m->temp_motor_start_c) fault_clear_auto(FAULT_MOTOR_OVERTEMP);
    }

    /* A failed FET sensor is serious: we can no longer protect the power stage.
     * Force heavy derating by pretending we're at the start threshold. */
    if (v->ntc_fet.fault) {
        fault_raise(FAULT_TEMP_SENSOR);
        v->ntc_fet.temp_c = maxf(v->ntc_fet.temp_c, 0.5f * (l->temp_fet_start_c + l->temp_fet_max_c));
    } else if (!v->ntc_motor.fault) {
        fault_clear_auto(FAULT_TEMP_SENSOR);
    }
}

static void check_sensors(vehicle_t *v)
{
    sensor_mode_t sm = v->cfg->foc.sensor_mode;
    hall_diag_tick(&g_mc.hall);
    bool hall_needed = (sm == SENSOR_MODE_HALL) ||
                       (sm == SENSOR_MODE_HALL_OBSERVER && g_mc.angle_src == MC_ANGLE_HALL);
    /* Before detection the hall table is empty and every code is "invalid". */
    if (hall_needed && g_mc.hall.fault && app_motor_ready(v->cfg)) fault_raise(FAULT_HALL);
    if (sm == SENSOR_MODE_ENCODER && g_mc.enc.fault) fault_raise(FAULT_ENCODER);

    if (v->throttle.fault) fault_raise(FAULT_THROTTLE);
    else fault_clear_auto(FAULT_THROTTLE);

    /* Stall: high torque, no rotation -> winding/FET hot spot. */
    float max_i = v->cfg->limits.motor_current_max;
    if (absf(g_mc.iq_filt) > 0.7f * max_i && absf(mc_erpm()) < STALL_ERPM) {
        if (++v->stall_ms > STALL_TIME_MS) fault_raise(FAULT_STALL);
    } else {
        v->stall_ms = 0;
    }
}

/* ---------------- torque request ---------------- */

static float drive_request(vehicle_t *v, float throttle)
{
    const app_config_t *cfg = v->cfg;
    float iq = 0.0f;

    switch (cfg->vehicle.type) {
    case VEHICLE_EBIKE_PAS:
        if (v->pas.pedalling && v->pas_level > 0) {
            uint8_t lvl = v->pas_level > 5 ? 5 : v->pas_level;
            iq = cfg->vehicle.pas_assist_current[lvl];
        }
        /* Throttle on a pedelec: EU allows only walk-assist (<= 6 km/h). */
        if (cfg->vehicle.throttle_enabled && v->speed_kmh < 6.0f)
            iq = maxf(iq, throttle * cfg->vehicle.pas_assist_current[1]);
        break;

    case VEHICLE_ESCOOTER:
    case VEHICLE_EBIKE_THROTTLE:
    default:
        switch (cfg->foc.control_mode) {
        case CONTROL_MODE_SPEED: {
            float target = throttle * cfg->limits.erpm_max;
            if (throttle <= 0.0f) {
                pi_reset(&v->speed_pi, 0.0f);
                iq = 0.0f;
            } else {
                /* Speed mode only drives forward; braking is the brake's job. */
                iq = pi_run(&v->speed_pi, target - mc_erpm(), DT, 0.0f, v->lim.iq_drive_max);
            }
            break;
        }
        case CONTROL_MODE_DUTY: {
            /* Voltage-like feel: current limited by the gap to the target duty. */
            float mod_err = throttle - g_mc.foc.modulation;
            iq = clampf(mod_err * 4.0f, 0.0f, 1.0f) * v->lim.iq_drive_max;
            break;
        }
        case CONTROL_MODE_TORQUE:
        default:
            iq = throttle * cfg->limits.motor_current_max;
            break;
        }
        break;
    }
    return iq;
}

void vehicle_medium_loop(void)
{
    vehicle_t *v = &g_vehicle;
    const app_config_t *cfg = v->cfg;
    v->count++;

    /* Inputs */
    float thr_v = adc_volts(HAL_ADC_SLOW_THROTTLE) / BOARD_THROTTLE_DIVIDER;
    float brk_v = adc_volts(HAL_ADC_SLOW_BRAKE) / BOARD_THROTTLE_DIVIDER;
    float throttle = cfg->vehicle.throttle_enabled ? throttle_update(&v->throttle, thr_v) : 0.0f;
    brake_update(&v->brake, hal_brake_switch_active(), brk_v);
    pas_update(&v->pas, hal_pas_read());

    if (++v->slow_div >= 10) { /* 100 Hz */
        v->slow_div = 0;
        ntc_update(&v->ntc_fet, hal_adc_read_slow(HAL_ADC_SLOW_TEMP_FET));
        ntc_update(&v->ntc_motor, hal_adc_read_slow(HAL_ADC_SLOW_TEMP_MOTOR));
        check_voltage_temp(v);
    }
    check_supervision(v);
    check_sensors(v);

    float erpm = mc_erpm();
    v->speed_kmh = vehicle_erpm_to_kmh(cfg, erpm);
    v->odometer_m += absf(v->speed_kmh) / 3.6f * DT;
    v->trip_wh += g_mc.vs.vbus_filt * g_mc.ibatt_filt * DT / 3600.0f;

    limits_input_t li = {
        .vbus = g_mc.vs.vbus_filt,
        .temp_fet = v->ntc_fet.temp_c,
        .temp_motor = v->ntc_motor.temp_c,
        .temp_motor_valid = cfg->motor.ntc_r25 > 0.0f && !v->ntc_motor.fault,
        .erpm = erpm,
        .speed_kmh = v->speed_kmh,
        .vq_abs = g_mc.vq_abs_filt,
        .batt_drive_cap = -1.0f,
        .batt_regen_cap = -1.0f,
    };
    if (g_can.bms_seen) {
        if (g_can.bms_timeout) {
            /* BMS went silent: it may have opened its contactor. No regen
             * (nowhere for the energy to go -> overvoltage), limp drive. */
            li.batt_drive_cap = 0.5f * cfg->limits.battery_current_max;
            li.batt_regen_cap = 0.0f;
        } else {
            li.batt_drive_cap = g_can.bms_stop ? 0.0f : g_can.bms_discharge_a;
            li.batt_regen_cap = g_can.bms_charge_a;
        }
    }
    limits_compute(cfg, &li, &v->lim);

    /* Torque request with brake priority */
    float target;
    bool braking = brake_cut_drive(&v->brake);
    if (g_mc.state == MC_STATE_DETECT && (braking || throttle > 0.0f)) {
        /* Rider intervention always wins over a running detection. */
        mc_abort_detection();
    }
    if (fault_blocks_drive() || g_mc.state == MC_STATE_DETECT) {
        target = 0.0f;
        if (braking) throttle_disarm(&v->throttle);
    } else if (braking) {
        float amount = v->brake.analog_enabled ? v->brake.analog_value
                                               : cfg->vehicle.regen_on_brake_fraction;
        if (v->brake.switch_active) amount = maxf(amount, cfg->vehicle.regen_on_brake_fraction);
        float fade = clampf(absf(erpm) / REGEN_FADE_ERPM, 0.0f, 1.0f);
        target = -signf(erpm) * amount * fade * v->lim.iq_regen_max;
        /* After braking the throttle must be released before it works again. */
        throttle_disarm(&v->throttle);
    } else {
        target = drive_request(v, throttle);
    }
    if (!isfinite(target) || !isfinite(v->lim.iq_drive_max) || !isfinite(v->lim.iq_regen_max)) {
        fault_raise(FAULT_NUMERIC);
        target = 0.0f;
        v->iq_cmd = 0.0f;
    }
    v->iq_target = clampf(target, -v->lim.iq_regen_max, v->lim.iq_drive_max);

    /* Brake cuts drive torque instantly - no ramp - before regen ramps in. */
    if (braking && v->iq_cmd * signf(erpm) > 0.0f) v->iq_cmd = 0.0f;

    /* Slew limit. Releasing torque is 5x faster than applying it. */
    float rate = cfg->limits.current_ramp_a_per_s * DT;
    bool releasing = absf(v->iq_target) < absf(v->iq_cmd) || signf(v->iq_target) != signf(v->iq_cmd);
    v->iq_cmd = ramp_towards(v->iq_cmd, v->iq_target, releasing ? 5.0f * rate : rate);
    mc_set_current_command(v->iq_cmd);

    /* Run / coast decision */
    bool want_torque = absf(v->iq_cmd) > 0.05f;
    if (want_torque) v->idle_ms = 0;
    else if (v->idle_ms < 0xFFFFFFFFu) v->idle_ms++;
    bool keep_sync = (cfg->foc.sensor_mode == SENSOR_MODE_SENSORLESS) &&
                     absf(erpm) > 0.5f * cfg->foc.hall_observer_crossover_erpm;
    mc_request_run(!fault_blocks_drive() && app_motor_ready(cfg) && (want_torque || v->idle_ms < COAST_DELAY_MS || keep_sync));
}
