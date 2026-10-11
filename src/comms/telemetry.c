#include "telemetry.h"

#include "fault_manager.h"
#include "hal.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "unlock.h"
#include "vehicle_control.h"

void telemetry_fill(telemetry_t *t)
{
    const vehicle_t *v = &g_vehicle;
    t->time_ms = hal_millis();
    t->mc_state = (uint8_t)g_mc.state;
    t->angle_src = (uint8_t)g_mc.angle_src;
    t->pas_level = v->pas_level;
    t->flags = (uint8_t)((v->brake.switch_active ? 1u : 0u) | (v->throttle.armed ? 2u : 0u) |
                         (v->pas.pedalling ? 4u : 0u) | (hal_pwm_outputs_enabled() ? 8u : 0u) | (unlock_is_active() ? 16u : 0u));
    t->faults = fault_active_mask();
    t->vbus = g_mc.vs.vbus_filt;
    t->ibatt = g_mc.ibatt_filt;
    t->id = g_mc.id_filt;
    t->iq = g_mc.iq_filt;
    t->iq_target = v->iq_target;
    t->erpm = g_mc.erpm_filt;
    t->speed_kmh = v->speed_kmh;
    t->modulation = g_mc.foc.modulation;
    t->temp_fet = v->ntc_fet.temp_c;
    t->temp_motor = v->ntc_motor.temp_c;
    t->throttle = v->throttle.value;
    t->odometer_km = v->odometer_m / 1000.0f;
    t->trip_wh = v->trip_wh;
    t->iq_limit_drive = v->lim.iq_drive_max;
    t->iq_limit_regen = v->lim.iq_regen_max;
    t->loop_us_max = (uint16_t)(g_mc.loop_cycles_max / hal_cycles_per_us());
    t->hall_code = hal_hall_read();
    t->profile = (uint8_t)profile_active();
}
