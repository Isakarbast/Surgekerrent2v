#include "current_limits.h"

#include "util_math.h"

/* 1 below start, linear to 0 at end. Works for rising (start<end) or falling. */
static float fold(float x, float start, float end)
{
    return mapf_clamped(x, start, end, 1.0f, 0.0f);
}

void limits_compute(const app_config_t *cfg, const limits_input_t *in, limits_output_t *out)
{
    const limits_config_t *l = &cfg->limits;
    const vehicle_config_t *v = &cfg->vehicle;

    out->k_temp_fet = fold(in->temp_fet, l->temp_fet_start_c, l->temp_fet_max_c);
    out->k_temp_motor = in->temp_motor_valid
        ? fold(in->temp_motor, cfg->motor.temp_motor_start_c, cfg->motor.temp_motor_max_c)
        : 1.0f;
    float k_temp = minf(out->k_temp_fet, out->k_temp_motor);

    /* Undervoltage: reduce drive so pack sag doesn't hit BMS cutoff. */
    out->k_uv = fold(in->vbus, l->battery_voltage_cutback_start, l->battery_voltage_min);
    /* Overvoltage: reduce regen over the last 2 V below the max. */
    out->k_ov = fold(in->vbus, l->battery_voltage_max - 2.0f, l->battery_voltage_max);

    /* Legal road speed limit - drive only, braking is always allowed. */
    out->k_speed = 1.0f;
    if (v->speed_limit_kmh > 0.0f) {
        out->k_speed = fold(absf(in->speed_kmh),
                            v->speed_limit_kmh - v->speed_limit_margin_kmh,
                            v->speed_limit_kmh);
    }
    out->k_erpm = fold(absf(in->erpm), l->erpm_max * 0.95f, l->erpm_max);

    float drive = l->motor_current_max * minf(k_temp, minf(out->k_uv, minf(out->k_speed, out->k_erpm)));
    float regen = l->motor_current_regen_max * minf(k_temp, out->k_ov);

    /* Battery current: Ibat ~= 1.5 * vq * iq / vbus (id*vd term small outside FW). */
    float ib_drive = l->battery_current_max;
    float ib_regen = l->battery_current_regen_max;
    if (in->batt_drive_cap >= 0.0f) ib_drive = minf(ib_drive, in->batt_drive_cap);
    if (in->batt_regen_cap >= 0.0f) ib_regen = minf(ib_regen, in->batt_regen_cap);
    float vq = maxf(in->vq_abs, 0.5f);
    float iq_batt_drive = ib_drive * in->vbus / (1.5f * vq);
    float iq_batt_regen = ib_regen * in->vbus / (1.5f * vq);

    out->iq_drive_max = maxf(0.0f, minf(drive, iq_batt_drive));
    out->iq_regen_max = maxf(0.0f, minf(regen, iq_batt_regen));
}
