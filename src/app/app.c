/*
 * app.c - boot sequence, scheduler hooks, main loop and watchdog policy.
 *
 * Execution contexts (highest priority first):
 *   ADC ISR   20 kHz  app_fast_loop_isr   -> mc_fast_loop
 *   timer ISR  1 kHz  app_medium_loop_isr -> vehicle_medium_loop
 *   main loop  free   comms, LED, watchdog
 *
 * The watchdog is only kicked when BOTH ISR loops have made progress since
 * the last kick, so a hung ISR, a hung main loop, or a stuck interrupt
 * storm all end in a reset. On reset the timer outputs go Hi-Z and the
 * gate driver's input pull-downs hold all FETs off.
 */
#include "app.h"

#include "board_config.h"
#include "can_bus.h"
#include "config_storage.h"
#include "fault_manager.h"
#include "gate_driver.h"
#include "hal.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "uart_protocol.h"
#include "unlock.h"
#include "vehicle_control.h"

/* Must exceed the longest main-loop stall: a 2 KB flash page erase is up to
 * 40 ms on STM32G4 (the HAL kicks right before it). */
#define WATCHDOG_TIMEOUT_MS 100u

static uint32_t s_wd_fast, s_wd_medium;

void app_fast_loop_isr(void) { mc_fast_loop(); }

void app_medium_loop_isr(void)
{
    if (g_mc.state == MC_STATE_CALIBRATING) return;
    vehicle_medium_loop();
}

static void snapshot(fault_log_entry_t *e)
{
    e->vbus = g_mc.vs.vbus;
    e->current = g_mc.i_phase_peak;
    e->erpm = g_mc.erpm_filt;
    e->temp_fet = g_vehicle.ntc_fet.temp_c;
}

bool app_motor_ready(const app_config_t *cfg)
{
    sensor_mode_t sm = cfg->foc.sensor_mode;
    if (sm == SENSOR_MODE_HALL || sm == SENSOR_MODE_HALL_OBSERVER) {
        int valid = 0;
        for (int i = 0; i < 8; i++)
            if (cfg->motor.hall_angle_rad[i] >= 0.0f) valid++;
        if (valid != 6) return false;
    }
    return true;
}

void app_clear_hw_faults(void)
{
    hal_pwm_hw_break_clear();
    gate_driver_get()->clear_faults();
    throttle_disarm(&g_vehicle.throttle);
}

void app_init(void)
{
    fault_init();
    fault_set_snapshot_provider(snapshot);

    if (hal_reset_cause() == HAL_RESET_WATCHDOG) fault_raise(FAULT_WATCHDOG_RESET);

    storage_result_t sr = config_storage_load(&g_config);
    if (!config_validate(&g_config) || sr != STORAGE_LOADED) fault_raise(FAULT_CONFIG);
    config_update_crc(&g_config);
    if (!app_motor_ready(&g_config)) fault_raise(FAULT_CONFIG);

    const gate_driver_t *gd = gate_driver_get();
    if (!gd->init()) fault_raise(FAULT_GATE_DRIVER);

    unlock_init();
    profiles_init();             /* builds g_active_config from g_config + boot profile */
    mc_init(&g_active_config);
    vehicle_init(&g_active_config);
    proto_init();
    hal_can_init(g_config.vehicle.can_bitrate);
    can_bus_init(g_config.vehicle.can_node_id);

    hal_start_scheduler();       /* PWM timer + ADC trigger; current offsets calibrate first */
    hal_watchdog_start(WATCHDOG_TIMEOUT_MS);
}

static void led_update(void)
{
    /* Solid = ok, 1 Hz blink = idle w/o motor params, N fast blinks = fault N. */
    uint32_t t = hal_millis();
    fault_id_t f = fault_first_active();
    if (f != FAULT_NONE && fault_blocks_drive()) {
        uint32_t period = ((uint32_t)f + 4u) * 250u;   /* N blinks + 1 s pause */
        uint32_t phase = t % period;
        uint32_t n = phase / 250u;
        hal_led_set(n < (uint32_t)f && (phase % 250u) < 125u);
    } else if (!app_motor_ready(&g_config)) {
        hal_led_set((t / 500u) & 1u);
    } else {
        hal_led_set(true);
    }
}

void app_main_loop_iteration(void)
{
    proto_poll();
    can_bus_poll();
    led_update();

    uint32_t fast = g_mc.loop_count;
    uint32_t medium = g_vehicle.count;
    bool calibrating = (g_mc.state == MC_STATE_CALIBRATING);
    if (fast != s_wd_fast && (medium != s_wd_medium || calibrating)) {
        s_wd_fast = fast;
        s_wd_medium = medium;
        hal_watchdog_kick();
    }
}
