/*
 * vehicle_control.h - the 1 kHz medium loop: rider inputs -> torque request.
 *
 * Priority order (highest first):
 *   1. any drive-blocking fault        -> no torque, outputs off
 *   2. brake lever                     -> drive cut, regen (never below ~0 speed)
 *   3. throttle / pedal assist         -> drive torque
 * followed by derating (current_limits.c), slew-rate limiting and the run/coast
 * decision for the inverter.
 */
#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "brake.h"
#include "config.h"
#include "current_limits.h"
#include "ntc.h"
#include "pas.h"
#include "pi_controller.h"
#include "throttle.h"

typedef struct {
    const app_config_t *cfg;
    throttle_t throttle;
    brake_t brake;
    pas_t pas;
    ntc_t ntc_fet, ntc_motor;
    limits_output_t lim;
    pi_ctrl_t speed_pi;

    volatile uint8_t pas_level;
    float iq_target, iq_cmd;
    float speed_kmh;
    float odometer_m;
    float trip_wh;

    uint32_t idle_ms, stall_ms, gd_poll_ms, slow_div;
    uint32_t last_fast_count;
    uint8_t fast_dead_ms;
    volatile uint32_t count;
} vehicle_t;

extern vehicle_t g_vehicle;

void vehicle_init(const app_config_t *cfg);
void vehicle_medium_loop(void);
float vehicle_erpm_to_kmh(const app_config_t *cfg, float erpm);

#endif
