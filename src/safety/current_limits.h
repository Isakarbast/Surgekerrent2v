/*
 * current_limits.h - current derating ("foldback") so the controller degrades
 * gracefully instead of tripping hard mid-ride. Hard trips only happen at
 * the absolute limits (see protections in motor_control / vehicle_control).
 *
 * A sudden cut at speed is itself a hazard (rider thrown forward), so the
 * soft limits are deliberately placed well inside the hard ones.
 */
#ifndef CURRENT_LIMITS_H
#define CURRENT_LIMITS_H

#include "config.h"

typedef struct {
    float vbus;
    float temp_fet, temp_motor;
    bool temp_motor_valid;
    float erpm;
    float speed_kmh;
    float vq_abs;          /* filtered |vq| for battery-current estimation */
    float batt_drive_cap;  /* external (BMS) battery current caps [A]; < 0 = none */
    float batt_regen_cap;
} limits_input_t;

typedef struct {
    float iq_drive_max;    /* >= 0 */
    float iq_regen_max;    /* >= 0 (applied as negative iq) */
    /* individual factors, for telemetry / debugging */
    float k_temp_fet, k_temp_motor, k_uv, k_ov, k_speed, k_erpm;
} limits_output_t;

void limits_compute(const app_config_t *cfg, const limits_input_t *in, limits_output_t *out);

#endif
