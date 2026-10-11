/*
 * safety_envelope.h - hard ceilings for every user-reachable setting.
 *
 * Two entry points:
 *   envelope_check_*  - used by the protocol: returns the FIRST violated rule
 *                       so the request can be refused with a reason.
 *   envelope_clamp_*  - used on boot / when building the active config:
 *                       defence in depth against corrupted or tampered flash.
 * Every check treats NaN/Inf as a violation (NaN compares false against
 * any limit and would otherwise sail through).
 */
#ifndef SAFETY_ENVELOPE_H
#define SAFETY_ENVELOPE_H

#include <stdbool.h>

#include "config.h"

typedef enum {
    ENV_OK = 0,
    ENV_VIOL_NOT_FINITE,          /* NaN / Inf / negative where not allowed */
    ENV_VIOL_MOTOR_CURRENT,       /* > ENV_PHASE_CURRENT_MAX */
    ENV_VIOL_REGEN_CURRENT,       /* > ENV_PHASE_CURRENT_MAX */
    ENV_VIOL_BATTERY_CURRENT,     /* > ENV_BATTERY_CURRENT_MAX or > motor current */
    ENV_VIOL_BATTERY_REGEN,       /* > ENV_BATTERY_REGEN_MAX */
    ENV_VIOL_ABS_TRIP,            /* > ENV_ABS_TRIP_MAX */
    ENV_VIOL_ABS_TRIP_MARGIN,     /* trip < 1.05x the larger operating current */
    ENV_VIOL_ERPM,
    ENV_VIOL_RAMP,
    ENV_VIOL_FIELD_WEAKENING,     /* > ENV_FW_CURRENT_MAX or > motor current */
    ENV_VIOL_SPEED_LIMIT,
    ENV_VIOL_THROTTLE_EXPO,
    ENV_VIOL_MODULATION,          /* max_modulation > 0.95 */
    ENV_VIOL_CURRENT_BANDWIDTH,
    ENV_VIOL_CONTROLLER_GAINS,    /* current/speed/observer gains out of sane range */
    ENV_VIOL_MOTOR_PARAMS,        /* R/L/flux not physical */
    ENV_VIOL_PAS_CURRENT,
    ENV_VIOL_INVALID_ENUM,        /* flag not 0/1, or unknown mode number */
    ENV_VIOL_SENSOR_CALIBRATION,  /* hall angles / encoder offset out of [0, 2pi) */
    ENV_VIOL_INPUT_CALIBRATION,   /* throttle/brake voltage window not ordered or out of 0..5.5 V */
    ENV_VIOL_VEHICLE_PARAMS,      /* regen fraction, PAS cadence, NTC, ... */
    ENV_VIOL_COUNT
} env_violation_t;

env_violation_t envelope_check_profile(const profile_params_t *p);
/* Checks the fields an unlocked user may edit (foc, motor, PAS, custom profile). */
env_violation_t envelope_check_config(const app_config_t *cfg);
bool envelope_clamp_profile(profile_params_t *p);   /* false if something was changed */
bool envelope_clamp_limits(limits_config_t *l);
const char *envelope_violation_name(env_violation_t v);

#endif
