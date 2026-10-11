#include "ride_profiles.h"

#include <string.h>

#include "hal.h"
#include "motor_control.h"
#include "safety_envelope.h"
#include "util_math.h"
#include "vehicle_control.h"

app_config_t g_active_config;
static ride_profile_t s_active = PROFILE_SPORT;

/* Preset definitions: fractions of the factory limits. */
typedef struct {
    const char *name;
    float k_motor, k_battery, k_regen;
    float k_ramp;
    float speed_factor;      /* x factory speed limit, never > 1 */
    float throttle_expo;
    bool allow_fw;
} preset_t;

static const preset_t k_presets[PROFILE_CUSTOM] = {
    [PROFILE_ECO]   = {"eco",   0.50f, 0.50f, 1.00f, 0.50f, 0.80f, 0.50f, false},
    [PROFILE_SPORT] = {"sport", 0.80f, 0.80f, 1.00f, 1.00f, 1.00f, 0.30f, false},
    [PROFILE_RACE]  = {"race",  1.00f, 1.00f, 1.00f, 1.50f, 1.00f, 0.15f, true},
};

const char *profile_name(ride_profile_t p)
{
    if (p < PROFILE_CUSTOM) return k_presets[p].name;
    return (p == PROFILE_CUSTOM) ? "custom" : "?";
}

void profile_params_for(const app_config_t *base, ride_profile_t p, profile_params_t *out)
{
    if (p == PROFILE_CUSTOM) {
        *out = base->ride.custom;
    } else {
        const preset_t *pr = &k_presets[(p < PROFILE_CUSTOM) ? p : PROFILE_SPORT];
        const limits_config_t *l = &base->limits;
        out->motor_current_max = l->motor_current_max * pr->k_motor;
        out->motor_current_regen_max = l->motor_current_regen_max * pr->k_regen;
        out->battery_current_max = l->battery_current_max * pr->k_battery;
        out->battery_current_regen_max = l->battery_current_regen_max * pr->k_regen;
        out->abs_current_max = l->abs_current_max;
        out->erpm_max = l->erpm_max;
        out->current_ramp_a_per_s = l->current_ramp_a_per_s * pr->k_ramp;
        out->fw_max_current = pr->allow_fw ? base->foc.fw_max_current : 0.0f;
        out->speed_limit_kmh = base->vehicle.speed_limit_kmh * pr->speed_factor;
        out->throttle_expo = pr->throttle_expo;
    }
    envelope_clamp_profile(out); /* defence in depth */
}

static void build(const app_config_t *base, ride_profile_t p, app_config_t *out)
{
    profile_params_t pp;
    profile_params_for(base, p, &pp);
    memcpy(out, base, sizeof(*out));
    out->limits.motor_current_max = pp.motor_current_max;
    out->limits.motor_current_regen_max = pp.motor_current_regen_max;
    out->limits.battery_current_max = pp.battery_current_max;
    out->limits.battery_current_regen_max = pp.battery_current_regen_max;
    out->limits.abs_current_max = pp.abs_current_max;
    out->limits.erpm_max = pp.erpm_max;
    out->limits.current_ramp_a_per_s = pp.current_ramp_a_per_s;
    out->foc.fw_max_current = pp.fw_max_current;
    out->vehicle.speed_limit_kmh = pp.speed_limit_kmh;
    out->vehicle.throttle_expo = pp.throttle_expo;
    config_validate(out);
}

static void apply(void)
{
    static app_config_t next;
    build(&g_config, s_active, &next);

    /* Swap atomically w.r.t. the control ISRs (~1 us with IRQs off). */
    uint32_t irq = hal_irq_save();
    memcpy(&g_active_config, &next, sizeof(next));
    /* Fields cached inside modules at init time: */
    g_mc.fw.fw_max = next.foc.fw_max_current;
    g_mc.fw.ki = next.foc.fw_max_current * 20.0f;
    g_vehicle.throttle.expo = next.vehicle.throttle_expo;
    hal_irq_restore(irq);
}

void profiles_init(void)
{
    s_active = (ride_profile_t)g_config.ride.boot_profile;
    if (s_active >= PROFILE_CUSTOM) s_active = PROFILE_SPORT;
    build(&g_config, s_active, &g_active_config);
}

void profiles_refresh(void)
{
    if (s_active == PROFILE_CUSTOM && !g_config.ride.advanced_unlocked) s_active = PROFILE_SPORT;
    apply();
}

ride_profile_t profile_active(void) { return s_active; }

profile_sel_result_t profile_select(ride_profile_t p)
{
    if (p >= PROFILE_COUNT) return PROFILE_SEL_INVALID;
    if (p == PROFILE_CUSTOM) {
        if (!g_config.ride.advanced_unlocked) return PROFILE_SEL_LOCKED;
        if (s_active != PROFILE_CUSTOM && !mc_is_stopped()) return PROFILE_SEL_NOT_STOPPED;
    }
    /* Switching between presets is allowed while riding: a higher limit is
     * reached through the normal torque ramp, a lower one applies at once. */
    s_active = p;
    apply();
    return PROFILE_SEL_OK;
}

void profiles_on_lock(void)
{
    if (s_active == PROFILE_CUSTOM) s_active = (ride_profile_t)g_config.ride.boot_profile;
    apply();
}
