/*
 * System test: the complete firmware (app + fast/medium loops + protocol)
 * running against the simulated PMSM in sim_hal.c.
 */
#include <string.h>

#include "app.h"
#include "board_config.h"
#include "config.h"
#include "fault_manager.h"
#include "hal.h"
#include "config_storage.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "safety_envelope.h"
#include "unlock.h"
#include "sim_hal.h"
#include "test_framework.h"
#include "uart_protocol.h"
#include "util_math.h"
#include "vehicle_control.h"

static void run(float sec)
{
    uint64_t n = (uint64_t)(sec * 20000.0f);
    for (uint64_t i = 0; i < n; i++) {
        sim_step();
        if ((i % 20u) == 0u) app_main_loop_iteration();
    }
}

/* Send a command over the simulated UART; returns reply cmd byte (0 if none). */
static uint8_t transact(uint8_t cmd, const uint8_t *payload, size_t len, uint8_t *reply, size_t *reply_len)
{
    uint8_t frame[260];
    size_t n = proto_build_frame(cmd, payload, len, frame);
    uint8_t junk[512];
    while (sim_uart_take(junk, sizeof(junk))) {}
    sim_uart_inject(frame, (uint32_t)n);
    run(0.01f);
    uint8_t buf[512];
    uint32_t got = sim_uart_take(buf, sizeof(buf));
    if (got < 5 || buf[0] != PROTO_SOF) return 0;
    if (reply_len) *reply_len = buf[1] - 1u;
    if (reply) memcpy(reply, &buf[3], buf[1] - 1u);
    return buf[2];
}

static float true_elec_angle_error(void)
{
    return absf(wrap_pi(g_mc.theta - g_sim.theta_e)) * 180.0f / M_PI_F;
}

static void boot(void)
{
    sim_reset_defaults();
    sim_flash_erase_all();
    hal_init();
    app_init();
    run(0.3f);
}

static void test_boot_and_calibration(void)
{
    boot();
    CHECK(g_mc.state == MC_STATE_IDLE);
    CHECK(g_mc.cs.status == CS_OK);
    CHECK(fault_is_active(FAULT_CONFIG));   /* fresh board: no motor params */
    CHECK(!app_motor_ready(&g_config));
    CHECK(!hal_pwm_outputs_enabled());

    /* Throttle must do nothing without detected parameters. */
    g_sim_p.throttle_v = 4.0f;
    run(0.5f);
    CHECK(!hal_pwm_outputs_enabled());
    CHECK(absf(g_sim.omega_m) < 0.01f);
    g_sim_p.throttle_v = 0.85f;
    run(0.3f);
}

static void test_ping(void)
{
    uint8_t r[16];
    size_t n = 0;
    CHECK(transact(CMD_PING, NULL, 0, r, &n) == (CMD_PING | 0x80));
    CHECK(n == 8 && memcmp(r, "SK2V", 4) == 0);
}

static void test_detection(void)
{
    CHECK(transact(CMD_START_DETECTION, NULL, 0, NULL, NULL) == (CMD_START_DETECTION | 0x80));
    for (int i = 0; i < 600 && g_mc.state == MC_STATE_DETECT; i++) run(0.05f);

    const motor_detect_t *d = &g_mc.det;
    printf("  detect: state=%s R=%.4f (%.4f) Ld=%.1fuH (%.1f) Lq=%.1fuH (%.1f) flux=%.5f (%.5f)\n",
           detect_state_name(d->state), d->rs, g_sim_p.rs, d->ld * 1e6, g_sim_p.ld * 1e6,
           d->lq * 1e6, g_sim_p.lq * 1e6, d->flux, g_sim_p.flux);
    CHECK(d->state == DET_DONE);
    CHECK_NEAR(d->rs, g_sim_p.rs, 0.1 * g_sim_p.rs);
    CHECK_NEAR(d->ld, g_sim_p.ld, 0.15 * g_sim_p.ld);
    CHECK_NEAR(d->lq, g_sim_p.lq, 0.15 * g_sim_p.lq);
    CHECK_NEAR(d->flux, g_sim_p.flux, 0.1 * g_sim_p.flux);

    /* Hall centres: code seq {1,3,2,6,4,5} centred on k*60 deg. */
    const uint8_t seq[6] = {1, 3, 2, 6, 4, 5};
    for (int k = 0; k < 6; k++) {
        float expect = (float)k * M_PI_F / 3.0f;
        float err = absf(wrap_pi(d->hall_angle[seq[k]] - expect)) * 180.0f / M_PI_F;
        CHECK(err < 8.0f);
    }

    CHECK(transact(CMD_APPLY_DETECTION, NULL, 0, NULL, NULL) == (CMD_APPLY_DETECTION | 0x80));
    CHECK(app_motor_ready(&g_config));
    CHECK(transact(CMD_SAVE_CONFIG, NULL, 0, NULL, NULL) == (CMD_SAVE_CONFIG | 0x80));
    run(0.5f);
}

static void test_ride_speed_limit_and_observer(void)
{
    /* Rider + scooter (100 kg) reflected to a 10" hub: J = m r^2. */
    g_sim_p.inertia = 100.0f * 0.127f * 0.127f;
    g_sim_p.friction = 0.05f;
    CHECK(!fault_blocks_drive());

    g_sim_p.throttle_v = 4.1f;
    float max_obs_err = 0.0f, max_kmh = 0.0f;
    bool saw_observer = false;
    for (int i = 0; i < 150; i++) {
        run(0.1f);
        if (g_mc.angle_src == MC_ANGLE_OBSERVER) {
            saw_observer = true;
            max_obs_err = maxf(max_obs_err, true_elec_angle_error());
        }
        max_kmh = maxf(max_kmh, g_vehicle.speed_kmh);
    }
    float true_kmh = g_sim.omega_m * 0.127f * 3.6f;
    printf("  ride: fw speed %.2f km/h, true %.2f km/h, max %.2f, observer max err %.2f deg, ibatt %.1f A\n",
           g_vehicle.speed_kmh, true_kmh, max_kmh, max_obs_err, g_mc.ibatt_filt);
    CHECK(fault_active_mask() == (1u << FAULT_CONFIG) || fault_active_mask() == 0u);
    CHECK(saw_observer);
    CHECK(max_obs_err < 5.0f);
    CHECK_NEAR(g_vehicle.speed_kmh, true_kmh, 0.5);
    CHECK(max_kmh <= g_config.vehicle.speed_limit_kmh + 0.5f);
    CHECK(g_vehicle.speed_kmh > g_config.vehicle.speed_limit_kmh - 3.0f);
    CHECK(g_mc.ibatt_filt <= g_config.limits.battery_current_max + 1.0f);
}

static void test_regen_braking(void)
{
    float v0 = g_vehicle.speed_kmh;
    /* Brake while throttle still held: drive must be gone within 2 ms. */
    g_sim_p.throttle_v = 4.1f;
    run(0.3f);
    g_sim_p.brake_switch = true;
    run(0.002f);
    CHECK(g_vehicle.iq_cmd <= 0.0f);
    g_sim_p.throttle_v = 0.85f;
    float min_ibatt = 0.0f, max_vbus = 0.0f;
    for (int i = 0; i < 30; i++) {
        run(0.1f);
        min_ibatt = minf(min_ibatt, g_mc.ibatt_filt);
        max_vbus = maxf(max_vbus, g_mc.vs.vbus);
    }
    printf("  brake: %.1f -> %.1f km/h, peak regen %.1f A, peak vbus %.1f V\n",
           v0, g_vehicle.speed_kmh, min_ibatt, max_vbus);
    /* 10 A phase regen on 100 kg: ~2.7 km/h in 3 s plus friction. */
    CHECK(g_vehicle.speed_kmh < v0 - 2.0f);
    CHECK(min_ibatt < -1.0f);
    CHECK(min_ibatt >= -g_config.limits.battery_current_regen_max - 1.0f);
    CHECK(!fault_blocks_drive());

    /* Throttle right after releasing brake must be ignored until released. */
    g_sim_p.brake_switch = false;
    g_sim_p.throttle_v = 4.1f;
    run(0.3f);
    CHECK(g_vehicle.iq_cmd <= 0.01f);
    g_sim_p.throttle_v = 0.85f;
    run(0.5f);
}

static void test_hw_overcurrent_fault(void)
{
    g_sim_p.throttle_v = 3.0f;
    run(0.5f);
    CHECK(hal_pwm_outputs_enabled());
    g_sim_p.hw_break = true;
    run(0.01f);
    CHECK(fault_is_active(FAULT_OVERCURRENT_HW));
    CHECK(!hal_pwm_outputs_enabled());

    /* Clear refused while throttle held */
    CHECK(transact(CMD_CLEAR_FAULTS, NULL, 0, NULL, NULL) == CMD_NACK);
    g_sim_p.throttle_v = 0.85f;
    run(0.1f);
    CHECK(transact(CMD_CLEAR_FAULTS, NULL, 0, NULL, NULL) == (CMD_CLEAR_FAULTS | 0x80));
    CHECK(!fault_is_active(FAULT_OVERCURRENT_HW));
}

static void test_undervoltage_foldback(void)
{
    g_sim_p.vbus = g_config.limits.battery_voltage_cutback_start - 1.0f;
    run(0.5f);
    float expect = (g_sim_p.vbus - g_config.limits.battery_voltage_min) /
                   (g_config.limits.battery_voltage_cutback_start - g_config.limits.battery_voltage_min);
    CHECK(g_vehicle.lim.k_uv < 0.99f);
    CHECK_NEAR(g_vehicle.lim.k_uv, expect, 0.05);
    g_sim_p.vbus = g_config.limits.battery_voltage_min - 2.0f;
    run(0.5f);
    CHECK(fault_is_active(FAULT_UNDERVOLTAGE));
    g_sim_p.vbus = 48.0f;
    run(0.5f);
    CHECK(!fault_is_active(FAULT_UNDERVOLTAGE));
}

static void test_config_survives_reboot(void)
{
    /* Simulate a power cycle: flash persists, RAM does not. */
    float rs = g_config.motor.rs_ohm;
    memset(&g_config, 0, sizeof(g_config));
    sim_reset_defaults();
    hal_init();
    app_init();
    run(0.3f);
    CHECK_NEAR(g_config.motor.rs_ohm, rs, 1e-6);
    CHECK(app_motor_ready(&g_config));
    CHECK(!fault_is_active(FAULT_CONFIG));
}

/* ---------------- ride profiles & unlock ---------------- */

static uint8_t s_nack_reason, s_nack_detail;

static uint8_t tx(uint8_t cmd, const void *payload, size_t len, uint8_t *reply, size_t *rlen)
{
    uint8_t r[260];
    size_t n = 0;
    uint8_t c = transact(cmd, (const uint8_t *)payload, len, r, &n);
    if (c == CMD_NACK) {
        s_nack_reason = r[1];
        s_nack_detail = (n >= 3) ? r[2] : 0;
    }
    if (reply) memcpy(reply, r, n);
    if (rlen) *rlen = n;
    return c;
}

static bool acked(uint8_t c, uint8_t cmd) { return c == (cmd | 0x80); }

/* 100 kg rider+scooter on a 10" wheel: inertia m*r^2, rolling resistance
 * Crr 0.01 -> ~1.2 N m at the wheel. */
static void vehicle_plant(void)
{
    g_sim_p.inertia = 100.0f * 0.127f * 0.127f;
    g_sim_p.friction = 0.05f;
    g_sim_p.load_torque = 1.2f;
}

static void stop_vehicle(void)
{
    g_sim_p.throttle_v = 0.85f;
    g_sim_p.brake_switch = true;
    for (int i = 0; i < 200 && !mc_is_stopped(); i++) run(0.1f);
    g_sim_p.brake_switch = false;
    run(1.0f); /* coast delay, outputs off */
}

/* Upload a whole config through SET_CONFIG chunks, then COMMIT. */
static uint8_t upload_config(app_config_t *c)
{
    uint8_t off0[2] = {0, 0};
    tx(CMD_GET_CONFIG, off0, 2, NULL, NULL);          /* start edit session */
    config_update_crc(c);
    const uint8_t *b = (const uint8_t *)c;
    for (size_t off = 0; off < sizeof(*c); off += 100) {
        uint8_t p[102];
        size_t len = sizeof(*c) - off < 100 ? sizeof(*c) - off : 100;
        p[0] = (uint8_t)off;
        p[1] = (uint8_t)(off >> 8);
        memcpy(&p[2], &b[off], len);
        if (!acked(tx(CMD_SET_CONFIG, p, len + 2, NULL, NULL), CMD_SET_CONFIG)) return 0;
    }
    return tx(CMD_COMMIT_CONFIG, NULL, 0, NULL, NULL);
}

static uint8_t do_unlock(uint16_t terms, bool corrupt)
{
    uint8_t ch[40];
    size_t n = 0;
    uint8_t c = tx(CMD_UNLOCK_CHALLENGE, NULL, 0, ch, &n);
    if (!acked(c, CMD_UNLOCK_CHALLENGE)) return c;
    uint8_t resp[34];
    resp[0] = (uint8_t)terms;
    resp[1] = (uint8_t)(terms >> 8);
    /* What the rider's app does after "I agree": */
    unlock_compute_mac(g_unlock_key, &ch[0], &ch[16], terms, &resp[2]);
    if (corrupt) resp[10] ^= 0x01;
    return tx(CMD_UNLOCK_RESPONSE, resp, sizeof(resp), NULL, NULL);
}

static void test_profiles_locked(void)
{
    vehicle_plant();
    stop_vehicle();
    CHECK(profile_active() == PROFILE_SPORT);             /* factory boot profile */
    uint8_t p = PROFILE_CUSTOM;
    CHECK(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL) == CMD_NACK && s_nack_reason == NACK_LOCKED);

    app_config_t c = g_config;
    c.ride.custom.motor_current_max = 60.0f;
    CHECK(upload_config(&c) == CMD_NACK && s_nack_reason == NACK_LOCKED);
    CHECK(g_config.ride.custom.motor_current_max != 60.0f);

    /* ECO: half current, 80 % of the legal speed limit. */
    p = PROFILE_ECO;
    CHECK(acked(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL), CMD_SET_PROFILE));
    CHECK_NEAR(g_active_config.limits.motor_current_max, 0.5 * g_config.limits.motor_current_max, 1e-3);
    g_sim_p.throttle_v = 4.1f;
    float max_kmh = 0.0f, max_iq = 0.0f;
    for (int i = 0; i < 200; i++) {
        run(0.1f);
        max_kmh = maxf(max_kmh, g_vehicle.speed_kmh);
        max_iq = maxf(max_iq, g_mc.iq_filt);
    }
    printf("  eco: max %.1f km/h, max iq %.1f A\n", max_kmh, max_iq);
    CHECK(max_kmh <= 0.8f * g_config.vehicle.speed_limit_kmh + 0.5f);
    CHECK(max_iq <= 0.5f * g_config.limits.motor_current_max + 1.0f);

    /* Presets can be switched while riding; CUSTOM stays locked. */
    p = PROFILE_RACE;
    CHECK(acked(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL), CMD_SET_PROFILE));
    CHECK_NEAR(g_active_config.limits.motor_current_max, g_config.limits.motor_current_max, 1e-3);
    CHECK(fault_active_mask() == 0u);
    stop_vehicle();
}

static void test_unlock_auth(void)
{
    CHECK(!unlock_is_active());
    /* Wrong MAC */
    CHECK(do_unlock(1, true) == CMD_NACK && s_nack_reason == NACK_AUTH && s_nack_detail == UNLOCK_ERR_BAD_MAC);
    /* Response without a fresh challenge (replay) */
    uint8_t resp[34] = {1, 0};
    CHECK(tx(CMD_UNLOCK_RESPONSE, resp, sizeof(resp), NULL, NULL) == CMD_NACK &&
          s_nack_detail == UNLOCK_ERR_NO_CHALLENGE);
    /* Valid signature but outdated terms */
    CHECK(do_unlock(0, false) == CMD_NACK && s_nack_detail == UNLOCK_ERR_TERMS_VERSION);
    CHECK(!unlock_is_active());
    /* Correct */
    CHECK(acked(do_unlock(1, false), CMD_UNLOCK_RESPONSE));
    CHECK(unlock_is_active());
    app_config_t stored;
    CHECK(config_storage_load(&stored) == STORAGE_LOADED && stored.ride.advanced_unlocked == 1);
}

static void test_custom_envelope(void)
{
    app_config_t c = g_config;
    c.ride.custom.motor_current_max = 200.0f;
    CHECK(upload_config(&c) == CMD_NACK && s_nack_reason == NACK_ENVELOPE &&
          s_nack_detail == ENV_VIOL_MOTOR_CURRENT);

    c = g_config;
    c.ride.custom.battery_current_max = 250.0f;
    CHECK(upload_config(&c) == CMD_NACK && s_nack_detail == ENV_VIOL_BATTERY_CURRENT);

    c = g_config;
    c.limits.motor_current_max = 150.0f;                  /* factory field */
    CHECK(upload_config(&c) == CMD_NACK && s_nack_reason == NACK_PROTECTED_FIELD);

    c = g_config;
    c.vehicle.wheel_diameter_m = 0.15f;                   /* classic speedometer hack */
    CHECK(upload_config(&c) == CMD_NACK && s_nack_reason == NACK_PROTECTED_FIELD);

    /* At the ceiling: accepted. Speed limit off (private land). */
    c = g_config;
    c.ride.custom = (profile_params_t){
        .motor_current_max = 150.0f, .motor_current_regen_max = 40.0f,
        .battery_current_max = 80.0f, .battery_current_regen_max = 15.0f,
        .abs_current_max = 160.0f, .erpm_max = 30000.0f, .current_ramp_a_per_s = 600.0f,
        .fw_max_current = 0.0f, .speed_limit_kmh = 0.0f, .throttle_expo = 0.1f,
    };
    CHECK(acked(upload_config(&c), CMD_COMMIT_CONFIG));
    CHECK(g_config.ride.custom.motor_current_max == 150.0f);

    /* CUSTOM refused while moving, accepted at standstill. A config commit
     * disarms the throttle, so release it first. */
    g_sim_p.throttle_v = 0.85f;
    run(0.5f);
    g_sim_p.throttle_v = 4.1f;
    run(1.0f);
    CHECK(!mc_is_stopped());
    uint8_t p = PROFILE_CUSTOM;
    CHECK(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL) == CMD_NACK && s_nack_reason == NACK_NOT_STOPPED);
    stop_vehicle();
    CHECK(acked(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL), CMD_SET_PROFILE));
    CHECK(g_active_config.limits.motor_current_max == 150.0f);

    g_sim_p.throttle_v = 4.1f;
    float max_kmh = 0.0f, max_ipk = 0.0f;
    for (int i = 0; i < 60; i++) {
        run(0.1f);
        max_kmh = maxf(max_kmh, g_vehicle.speed_kmh);
        max_ipk = maxf(max_ipk, g_mc.i_phase_peak);
    }
    printf("  custom: max %.1f km/h, peak phase %.1f A, faults 0x%x\n", max_kmh, max_ipk, fault_active_mask());
    CHECK(max_kmh > g_config.vehicle.speed_limit_kmh + 5.0f);  /* limit really off */
    CHECK(max_ipk < ENV_ABS_TRIP_MAX);                         /* no trip at the ceiling */
    CHECK(fault_active_mask() == 0u);
    stop_vehicle();
}

static void test_unlock_survives_reboot_and_lock(void)
{
    sim_reset_defaults();
    hal_init();
    app_init();
    run(0.3f);
    vehicle_plant();
    CHECK(unlock_is_active());
    CHECK(profile_active() == PROFILE_SPORT);              /* never boots into CUSTOM */

    CHECK(acked(tx(CMD_LOCK_ADVANCED, NULL, 0, NULL, NULL), CMD_LOCK_ADVANCED));
    CHECK(!unlock_is_active());
    uint8_t p = PROFILE_CUSTOM;
    CHECK(tx(CMD_SET_PROFILE, &p, 1, NULL, NULL) == CMD_NACK && s_nack_reason == NACK_LOCKED);
}

static void test_unlock_bruteforce_lockout(void)
{
    for (int i = 0; i < (int)UNLOCK_MAX_FAILS; i++) do_unlock(1, true);
    CHECK(do_unlock(1, false) == CMD_NACK && s_nack_detail == UNLOCK_ERR_LOCKED_OUT);
    CHECK(!unlock_is_active());
    run(61.0f);
    CHECK(acked(do_unlock(1, false), CMD_UNLOCK_RESPONSE));
}

static void test_detection_brake_abort(void)
{
    stop_vehicle();
    g_sim_p.inertia = 0.02f;   /* wheel lifted */
    g_sim_p.load_torque = 0.0f;
    CHECK(acked(tx(CMD_START_DETECTION, NULL, 0, NULL, NULL), CMD_START_DETECTION));
    run(8.0f);                                   /* into the hall-mapping spin */
    CHECK(g_mc.state == MC_STATE_DETECT);
    g_sim_p.brake_switch = true;
    run(0.003f);                                 /* 1 kHz loop + next fast period */
    CHECK(g_mc.state != MC_STATE_DETECT);
    CHECK(!hal_pwm_outputs_enabled());
    CHECK(g_mc.det.state == DET_ERROR && g_mc.det.error == DET_ERR_ABORTED);
    g_sim_p.brake_switch = false;
    /* Old motor parameters are untouched by an aborted detection. */
    CHECK(app_motor_ready(&g_config));
    run(0.5f);
    CHECK(acked(tx(CMD_CLEAR_FAULTS, NULL, 0, NULL, NULL), CMD_CLEAR_FAULTS));
}

int main(void)
{
    RUN_TEST(test_boot_and_calibration);
    RUN_TEST(test_ping);
    RUN_TEST(test_detection);
    RUN_TEST(test_ride_speed_limit_and_observer);
    RUN_TEST(test_regen_braking);
    RUN_TEST(test_hw_overcurrent_fault);
    RUN_TEST(test_undervoltage_foldback);
    RUN_TEST(test_config_survives_reboot);
    RUN_TEST(test_profiles_locked);
    RUN_TEST(test_unlock_auth);
    RUN_TEST(test_custom_envelope);
    RUN_TEST(test_unlock_survives_reboot_and_lock);
    RUN_TEST(test_unlock_bruteforce_lockout);
    RUN_TEST(test_detection_brake_abort);
    return TEST_SUMMARY();
}
