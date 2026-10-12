/* Fault injection at the control-loop boundary, using the real controller. */
#include <string.h>

#include "app.h"
#include "can_bus.h"
#include "config_storage.h"
#include "fault_manager.h"
#include "hal.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "safety_envelope.h"
#include "sim_hal.h"
#include "test_framework.h"
#include "util_math.h"
#include "vehicle_control.h"

static int s_motor_ntc_raw = -1;

uint16_t __real_hal_adc_read_slow(hal_adc_slow_ch_t ch);
uint16_t __wrap_hal_adc_read_slow(hal_adc_slow_ch_t ch)
{
    if (ch == HAL_ADC_SLOW_TEMP_MOTOR && s_motor_ntc_raw >= 0) return (uint16_t)s_motor_ntc_raw;
    return __real_hal_adc_read_slow(ch);
}

static void run(float seconds)
{
    for (unsigned i = 0; i < (unsigned)(seconds * hal_pwm_frequency_hz()); i++) {
        sim_step();
        if (i % 20u == 0) app_main_loop_iteration();
    }
}

static void boot_ready(void)
{
    s_motor_ntc_raw = -1;
    memset(&g_mc, 0, sizeof(g_mc));
    memset(&g_vehicle, 0, sizeof(g_vehicle));
    sim_reset_defaults();
    sim_flash_erase_all();
    config_load_defaults(&g_config);
    const uint8_t seq[] = {1, 3, 2, 6, 4, 5};
    for (unsigned i = 0; i < 6; i++) g_config.motor.hall_angle_rad[seq[i]] = (float)i * M_PI_F / 3.0f;
    g_config.motor.rs_ohm = g_sim_p.rs;
    g_config.motor.ld_h = g_sim_p.ld;
    g_config.motor.lq_h = g_sim_p.lq;
    g_config.motor.flux_wb = g_sim_p.flux;
    CHECK(config_storage_save(&g_config));
    hal_init();
    app_init();
    run(0.6f);
    CHECK(!fault_blocks_drive());
}

static void prepare_observer(float flux_scale)
{
    g_mc.state = MC_STATE_RUN;
    g_mc.run_req = true;
    g_mc.iq_cmd = 10.0f;
    g_mc.obs.x_alpha = flux_scale * g_mc.obs.flux;
    g_mc.obs.x_beta = 0.0f;
    g_mc.obs.pll.omega = 500.0f;
    g_mc.obs_lock_count = 1000;
    g_mc.obs_locked = true;
    hal_pwm_outputs_enable();
}

static void test_hall_failure_requires_healthy_observer(void)
{
    boot_ready();
    prepare_observer(5.0f);
    g_mc.hall.fault = true;
    mc_fast_loop();
    CHECK(!g_mc.obs_locked);
    CHECK(g_mc.state == MC_STATE_FAULT);
    CHECK(!hal_pwm_outputs_enabled());
    CHECK(fault_blocks_drive());
}

static void test_hall_failure_keeps_healthy_observer(void)
{
    boot_ready();
    prepare_observer(1.0f);
    g_mc.hall.fault = true;
    mc_fast_loop();
    CHECK(g_mc.obs_locked);
    CHECK(g_mc.angle_src == MC_ANGLE_OBSERVER);
    CHECK(hal_pwm_outputs_enabled());
    CHECK_NEAR(g_mc.foc.iq_ref, 10.0f, 1e-5f);
    CHECK(!fault_blocks_drive());
}

static void test_sensorless_flux_failure_stops_pwm(void)
{
    boot_ready();
    g_active_config.foc.sensor_mode = SENSOR_MODE_SENSORLESS;
    prepare_observer(5.0f);
    mc_fast_loop();
    CHECK(!g_mc.obs_locked);
    CHECK(fault_is_active(FAULT_OBSERVER));
    CHECK(!hal_pwm_outputs_enabled());
}

static void test_bms_timeout_cuts_regen_immediately(void)
{
    boot_ready();
    g_mc.erpm_filt = 3000.0f;
    g_mc.vq_abs_filt = 5.0f;
    g_vehicle.iq_cmd = -10.0f;
    g_sim_p.brake_switch = true;
    g_can.bms_seen = true;
    g_can.bms_timeout = true;
    vehicle_medium_loop();
    CHECK(g_vehicle.lim.iq_regen_max == 0.0f);
    CHECK(g_mc.iq_cmd == 0.0f);
}

static void test_bms_stop_cuts_drive_immediately(void)
{
    boot_ready();
    g_vehicle.iq_cmd = 32.0f;
    g_can.bms_seen = true;
    g_can.bms_stop = true;
    vehicle_medium_loop();
    CHECK(g_vehicle.lim.iq_drive_max == 0.0f);
    CHECK(g_mc.iq_cmd == 0.0f);
}

static void test_lower_profile_caps_existing_command(void)
{
    boot_ready();
    g_vehicle.iq_cmd = 32.0f;
    CHECK(profile_select(PROFILE_ECO) == PROFILE_SEL_OK);
    vehicle_medium_loop();
    CHECK(g_mc.iq_cmd <= g_vehicle.lim.iq_drive_max);
    CHECK(g_mc.iq_cmd > 0.0f);
}

static void test_motor_ntc_failure_removes_current_allowance(void)
{
    const int failed_readings[] = {0, 4095}; /* short and open circuit */
    for (unsigned i = 0; i < 2; i++) {
        boot_ready();
        g_sim_p.temp_motor_c = 110.0f;
        run(2.0f);
        CHECK(g_vehicle.lim.k_temp_motor < 0.6f);
        s_motor_ntc_raw = failed_readings[i];
        run(0.02f);
        CHECK(g_vehicle.ntc_motor.fault);
        CHECK(fault_is_active(FAULT_TEMP_SENSOR));
        CHECK(g_vehicle.lim.iq_drive_max == 0.0f);
        CHECK(g_vehicle.lim.iq_regen_max == 0.0f);

        s_motor_ntc_raw = -1;
        run(0.02f);
        CHECK(!g_vehicle.ntc_motor.fault);
        CHECK(!fault_is_active(FAULT_TEMP_SENSOR));
        CHECK(g_vehicle.lim.k_temp_motor > 0.0f && g_vehicle.lim.k_temp_motor < 0.6f);
    }
}

static void test_disabled_motor_ntc_does_not_derate(void)
{
    boot_ready();
    g_active_config.motor.ntc_r25 = 0.0f;
    vehicle_init(&g_active_config);
    s_motor_ntc_raw = 4095;
    run(0.02f);
    CHECK(!fault_is_active(FAULT_TEMP_SENSOR));
    CHECK(g_vehicle.lim.k_temp_motor == 1.0f);
    CHECK(g_vehicle.lim.iq_drive_max > 0.0f);
}

static void test_motor_ntc_failure_stops_field_weakening(void)
{
    boot_ready();
    g_mc.state = MC_STATE_RUN;
    g_mc.run_req = true;
    g_mc.fw.fw_max = 10.0f;
    g_mc.fw.id_fw = 10.0f;
    g_vehicle.idle_ms = 0; /* the controller was driving on the previous tick */
    g_mc.foc.modulation = g_mc.fw.start_mod;
    hal_pwm_outputs_enable();
    s_motor_ntc_raw = 4095;
    g_vehicle.slow_div = 9; /* sample the thermistors on this medium tick */
    vehicle_medium_loop();
    mc_fast_loop();
    CHECK(!hal_pwm_outputs_enabled());
    CHECK(g_mc.fw.id_fw == 0.0f);
}

static void test_motor_ntc_failure_aborts_detection(void)
{
    boot_ready();
    CHECK(mc_request_detection());
    mc_fast_loop();
    CHECK(g_mc.state == MC_STATE_DETECT);
    s_motor_ntc_raw = 0;
    g_vehicle.slow_div = 9;
    vehicle_medium_loop();
    mc_fast_loop();
    CHECK(g_mc.state != MC_STATE_DETECT);
    CHECK(!hal_pwm_outputs_enabled());
}

static void test_mtpa_respects_phase_current_budget(void)
{
    const float commands[] = {150.0f, -20.0f};
    for (unsigned i = 0; i < 2; i++) {
        boot_ready();
        g_config.motor.ld_h = 150e-6f;
        g_config.motor.lq_h = 1e-3f;
        g_config.ride.advanced_unlocked = 1;
        g_config.ride.custom.motor_current_max = 150.0f;
        g_config.ride.custom.abs_current_max = 160.0f;
        CHECK(envelope_check_config(&g_config) == ENV_OK);
        CHECK(profile_select(PROFILE_CUSTOM) == PROFILE_SEL_OK);
        mc_apply_config(&g_active_config);
        g_mc.state = MC_STATE_RUN;
        g_mc.run_req = true;
        g_mc.iq_cmd = commands[i];
        hal_pwm_outputs_enable();
        mc_fast_loop();
        float limit = commands[i] < 0.0f ? g_active_config.limits.motor_current_regen_max
                                         : g_active_config.limits.motor_current_max;
        float amplitude = hypotf(g_mc.foc.id_ref, g_mc.foc.iq_ref);
        CHECK(amplitude <= limit + 1e-4f);
        CHECK(g_mc.foc.id_ref < 0.0f);
        CHECK(g_mc.foc.iq_ref * commands[i] > 0.0f);
    }
}

static void test_field_weakening_respects_phase_current_budget(void)
{
    boot_ready();
    g_mc.state = MC_STATE_RUN;
    g_mc.run_req = true;
    g_mc.iq_cmd = g_active_config.limits.motor_current_max;
    g_mc.fw.fw_max = g_active_config.limits.motor_current_max;
    g_mc.fw.id_fw = 0.5f * g_mc.fw.fw_max;
    g_mc.foc.modulation = g_mc.fw.start_mod;
    hal_pwm_outputs_enable();
    mc_fast_loop();
    CHECK_NEAR(g_mc.foc.id_ref, -0.5f * g_mc.fw.fw_max, 1e-5f);
    CHECK(hypotf(g_mc.foc.id_ref, g_mc.foc.iq_ref) <= g_active_config.limits.motor_current_max + 1e-4f);
}

int main(void)
{
    RUN_TEST(test_hall_failure_requires_healthy_observer);
    RUN_TEST(test_hall_failure_keeps_healthy_observer);
    RUN_TEST(test_sensorless_flux_failure_stops_pwm);
    RUN_TEST(test_bms_timeout_cuts_regen_immediately);
    RUN_TEST(test_bms_stop_cuts_drive_immediately);
    RUN_TEST(test_lower_profile_caps_existing_command);
    RUN_TEST(test_motor_ntc_failure_removes_current_allowance);
    RUN_TEST(test_disabled_motor_ntc_does_not_derate);
    RUN_TEST(test_motor_ntc_failure_stops_field_weakening);
    RUN_TEST(test_motor_ntc_failure_aborts_detection);
    RUN_TEST(test_mtpa_respects_phase_current_budget);
    RUN_TEST(test_field_weakening_respects_phase_current_budget);
    return TEST_SUMMARY();
}
