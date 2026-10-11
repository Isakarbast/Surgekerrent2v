/*
 * ride_profiles.h - ECO / SPORT / RACE presets and the unlockable CUSTOM mode.
 *
 * g_config        persistent configuration (factory limits + CUSTOM values)
 * g_active_config what the control loops actually use: g_config with the
 *                 active profile's parameters overlaid, re-clamped to the
 *                 safety envelope.
 *
 * Presets are fixed in firmware and scale the FACTORY limits (g_config.limits),
 * which the protocol can never modify. Presets never raise the factory
 * speed limit. CUSTOM uses g_config.ride.custom, is only selectable after an
 * authenticated unlock, and only while the vehicle is stopped.
 */
#ifndef RIDE_PROFILES_H
#define RIDE_PROFILES_H

#include <stdbool.h>

#include "config.h"

typedef enum {
    PROFILE_SEL_OK = 0,
    PROFILE_SEL_INVALID,
    PROFILE_SEL_LOCKED,        /* CUSTOM requested but not unlocked */
    PROFILE_SEL_NOT_STOPPED,   /* CUSTOM can only be entered at standstill */
} profile_sel_result_t;

extern app_config_t g_active_config;

void profiles_init(void);                       /* after g_config is loaded */
profile_sel_result_t profile_select(ride_profile_t p);
ride_profile_t profile_active(void);
/* Rebuild g_active_config from g_config (after any config change). */
void profiles_refresh(void);
/* Parameters a profile resolves to for a given factory config. */
void profile_params_for(const app_config_t *base, ride_profile_t p, profile_params_t *out);
const char *profile_name(ride_profile_t p);
/* Re-lock advanced mode (drops back to the boot profile if CUSTOM is active). */
void profiles_on_lock(void);

#endif
