/* Unit tests for the MCU-independent building blocks. */
#include <string.h>

#include "board_config.h"
#include "config.h"
#include "config_storage.h"
#include "crc.h"
#include "current_sense.h"
#include "fault_manager.h"
#include "field_weakening.h"
#include "foc_math.h"
#include "hall_sensor.h"
#include "current_limits.h"
#include "pi_controller.h"
#include "pll.h"
#include "ride_profiles.h"
#include "safety_envelope.h"
#include "sha256.h"
#include "sim_hal.h"
#include "test_framework.h"
#include "throttle.h"
#include "util_math.h"

static void test_crc_vectors(void)
{
    const uint8_t s[] = "123456789";
    CHECK(crc16_ccitt(s, 9, 0xFFFF) == 0x29B1);
    CHECK(crc32(s, 9) == 0xCBF43926u);
}

static void test_sincos_atan2(void)
{
    double max_err = 0, max_atan = 0;
    for (int i = -2000; i <= 2000; i++) {
        float th = (float)i * 0.005f;
        float s, c;
        fast_sincosf(th, &s, &c);
        max_err = fmax(max_err, fabs(s - sinf(th)));
        max_err = fmax(max_err, fabs(c - cosf(th)));
        float a = fast_atan2f(sinf(th), cosf(th));
        max_atan = fmax(max_atan, fabs(wrap_pi(a - atan2f(sinf(th), cosf(th)))));
    }
    CHECK(max_err < 2e-5);
    CHECK(max_atan < 2e-3);
}

static void test_clarke_park_roundtrip(void)
{
    for (int k = 0; k < 36; k++) {
        float th = (float)k * 0.1745f;
        float s, c;
        fast_sincosf(th, &s, &c);
        dq_t in = {3.0f, -7.0f};
        ab_t ab = foc_inv_park(in, s, c);
        abc_t abc = foc_inv_clarke(ab);
        CHECK_NEAR(abc.a + abc.b + abc.c, 0.0, 1e-4);
        dq_t out = foc_park(foc_clarke3(abc), s, c);
        CHECK_NEAR(out.d, in.d, 1e-4);
        CHECK_NEAR(out.q, in.q, 1e-4);
    }
}

static void test_svpwm(void)
{
    float vbus = 48.0f;
    float vmax = vbus * ONE_BY_SQRT3;
    for (int k = 0; k < 360; k += 5) {
        float th = (float)k * M_PI_F / 180.0f;
        ab_t v = {0.99f * vmax * cosf(th), 0.99f * vmax * sinf(th)};
        float d[3];
        bool sat = foc_svpwm(v, vbus, d);
        CHECK(!sat);
        for (int i = 0; i < 3; i++) CHECK(d[i] >= 0.0f && d[i] <= 1.0f);
        /* Line-line voltages must be reproduced exactly. */
        abc_t p = foc_inv_clarke(v);
        CHECK_NEAR((d[0] - d[1]) * vbus, p.a - p.b, 1e-3);
        CHECK_NEAR((d[1] - d[2]) * vbus, p.b - p.c, 1e-3);
    }
    ab_t big = {100.0f, 0.0f};
    float d[3];
    CHECK(foc_svpwm(big, vbus, d));
    for (int i = 0; i < 3; i++) CHECK(d[i] >= 0.0f && d[i] <= 1.0f);
}

static void test_pi_antiwindup(void)
{
    pi_ctrl_t pi;
    pi_init(&pi, 1.0f, 100.0f);
    for (int i = 0; i < 1000; i++) pi_run(&pi, 10.0f, 1e-3f, -5.0f, 5.0f);
    CHECK(pi.integral <= 5.0f);
    /* After error reverses, output must leave saturation immediately-ish. */
    float u = pi_run(&pi, -10.0f, 1e-3f, -5.0f, 5.0f);
    CHECK(u < 5.0f);
}

static void test_pll_tracks(void)
{
    pll_t p;
    pll_init(&p, 2000.0f, 30000.0f);
    float th = 0, w = 500.0f, dt = 5e-5f;
    for (int i = 0; i < 20000; i++) {
        th = wrap_2pi(th + w * dt);
        pll_update(&p, th, dt);
    }
    CHECK_NEAR(p.omega, w, 1.0);
    CHECK_NEAR(wrap_pi(p.theta - th), 0.0, 0.01);
}

static void test_current_sense(void)
{
    current_sense_t cs;
    float apc = BOARD_ADC_VREF / BOARD_ADC_MAX_COUNTS * BOARD_AMPS_PER_VOLT;
    cs_init(&cs, apc, 0.94f);
    cs_calibrate_start(&cs, 100);
    hal_adc_fast_t raw = {2050, 2040, 2060, 1000};
    bool done = false;
    for (int i = 0; i < 100; i++) done = cs_calibrate_sample(&cs, &raw);
    CHECK(done && cs.status == CS_OK);

    /* +10 A into phase A, -4 B, -6 C (inverted amplifier: counts go down). */
    float per_a = 1.0f / apc;
    hal_adc_fast_t s = {(uint16_t)(2050 - 10 * per_a + 0.5f), (uint16_t)(2040 + 4 * per_a + 0.5f),
                        (uint16_t)(2060 + 6 * per_a + 0.5f), 1000};
    float duty[3] = {0.5f, 0.5f, 0.5f};
    abc_t i = cs_convert(&cs, &s, duty);
    CHECK_NEAR(i.a, 10.0, 0.1);
    CHECK_NEAR(i.b, -4.0, 0.1);
    CHECK_NEAR(i.c, -6.0, 0.1);

    /* Phase A at 98 % duty: garbage in A must be replaced by -(B+C). */
    s.ia = 0;
    duty[0] = 0.98f;
    i = cs_convert(&cs, &s, duty);
    CHECK(cs.reconstructed_phase == 0);
    CHECK_NEAR(i.a, 10.0, 0.1);

    /* Broken amplifier stuck at 0 V fails calibration. */
    cs_calibrate_start(&cs, 10);
    hal_adc_fast_t dead = {0, 2048, 2048, 0};
    for (int k = 0; k < 10; k++) cs_calibrate_sample(&cs, &dead);
    CHECK(cs.status == CS_ERR_OFFSET_RANGE);
}

static void test_throttle_safety(void)
{
    throttle_t t;
    throttle_init(&t, 0.9f, 4.1f, 0.4f, 4.6f, 0.0f);
    /* Held open at power-on: must stay 0. */
    for (int i = 0; i < 1000; i++) throttle_update(&t, 4.0f);
    CHECK(t.value == 0.0f);
    for (int i = 0; i < 300; i++) throttle_update(&t, 0.85f);
    CHECK(t.armed);
    for (int i = 0; i < 300; i++) throttle_update(&t, 4.1f);
    CHECK(t.value > 0.95f);
    /* Signal shorted to 5 V: fault, zero output. */
    for (int i = 0; i < 50; i++) throttle_update(&t, 5.0f);
    CHECK(t.fault && t.value == 0.0f);
    /* Recovery requires release. */
    for (int i = 0; i < 300; i++) throttle_update(&t, 3.0f);
    CHECK(t.value == 0.0f);
}

static void test_hall_speed(void)
{
    float table[8] = {-1, 0, 2.0944f, 1.0472f, 4.18879f, 5.23599f, 3.14159f, -1};
    hall_sensor_t h;
    float cps = 170e6f;
    hall_init(&h, table, cps);
    const uint8_t seq[6] = {1, 3, 2, 6, 4, 5};
    float omega = 300.0f; /* rad/s electrical */
    float t_edge = (M_PI_F / 3.0f) / omega;
    double t = 0;
    for (int e = 0; e < 24; e++) {
        for (int k = 0; k < 20; k++) {
            hall_update(&h, seq[e % 6], (uint32_t)(t * cps));
            t += t_edge / 20.0;
        }
    }
    CHECK_NEAR(h.omega, omega, 3.0);
    CHECK(!h.fault);
}

static void test_limits_derating(void)
{
    app_config_t cfg;
    config_load_defaults(&cfg);
    limits_input_t in = {.vbus = 50.0f, .temp_fet = 30.0f, .temp_motor = 30.0f,
                         .temp_motor_enabled = true, .erpm = 0, .speed_kmh = 0, .vq_abs = 1.0f,
                         .batt_drive_cap = -1.0f, .batt_regen_cap = -1.0f};
    limits_output_t o;
    limits_compute(&cfg, &in, &o);
    CHECK_NEAR(o.iq_drive_max, cfg.limits.motor_current_max, 1e-3);

    in.temp_fet = 0.5f * (cfg.limits.temp_fet_start_c + cfg.limits.temp_fet_max_c);
    limits_compute(&cfg, &in, &o);
    CHECK_NEAR(o.iq_drive_max, 0.5f * cfg.limits.motor_current_max, 0.1);

    in.temp_fet = 30.0f;
    in.speed_kmh = cfg.vehicle.speed_limit_kmh + 1.0f;
    limits_compute(&cfg, &in, &o);
    CHECK(o.iq_drive_max == 0.0f);
    CHECK(o.iq_regen_max > 0.0f); /* braking still allowed above the limit */

    /* Battery current: at high vq, iq limited so that Ibat <= max. */
    in.speed_kmh = 10.0f;
    in.vq_abs = 25.0f;
    limits_compute(&cfg, &in, &o);
    CHECK(1.5f * in.vq_abs * o.iq_drive_max / in.vbus <= cfg.limits.battery_current_max + 0.01f);
}

static void test_mtpa(void)
{
    CHECK(mtpa_id(100e-6f, 100e-6f, 0.01f, 30.0f) == 0.0f);   /* SPM: none */
    CHECK(mtpa_id(100e-6f, 200e-6f, 0.01f, 30.0f) < 0.0f);    /* IPM: negative */
}

static void test_config_storage(void)
{
    sim_reset_defaults();
    sim_flash_erase_all();
    app_config_t a, b;
    CHECK(config_storage_load(&a) == STORAGE_DEFAULTS_NO_DATA);

    a.limits.motor_current_max = 33.0f;
    CHECK(config_storage_save(&a));
    CHECK(config_storage_load(&b) == STORAGE_LOADED);
    CHECK(b.limits.motor_current_max == 33.0f);

    a.limits.motor_current_max = 44.0f;
    CHECK(config_storage_save(&a)); /* goes to the other slot */
    CHECK(config_storage_load(&b) == STORAGE_LOADED);
    CHECK(b.limits.motor_current_max == 44.0f);

    /* Corrupt the newest slot -> fall back to the previous good one. */
    sim_flash_corrupt(1, 100);
    CHECK(config_storage_load(&b) == STORAGE_LOADED);
    CHECK(b.limits.motor_current_max == 33.0f);
}

static void test_config_validate_clamps(void)
{
    app_config_t c;
    config_load_defaults(&c);
    CHECK(config_validate(&c));
    c.limits.abs_current_max = 10000.0f;
    c.limits.motor_current_max = 9000.0f;
    CHECK(!config_validate(&c));
    CHECK(c.limits.abs_current_max <= BOARD_MAX_PHASE_CURRENT);
    CHECK(c.limits.motor_current_max < c.limits.abs_current_max);
}

static void test_fault_manager(void)
{
    fault_init();
    fault_raise(FAULT_UNDERVOLTAGE);
    CHECK(fault_blocks_drive());
    fault_clear_auto(FAULT_UNDERVOLTAGE);
    CHECK(!fault_blocks_drive());

    fault_raise(FAULT_OVERCURRENT_SW);
    fault_clear_auto(FAULT_OVERCURRENT_SW); /* latched: must not clear */
    CHECK(fault_is_active(FAULT_OVERCURRENT_SW));
    CHECK(!fault_clear_latched(false));      /* throttle held */
    CHECK(fault_is_active(FAULT_OVERCURRENT_SW));
    CHECK(fault_clear_latched(true));
    CHECK(!fault_blocks_drive());
}

static void hex(const char *h, uint8_t *out)
{
    for (int i = 0; h[2 * i]; i++) sscanf(&h[2 * i], "%2hhx", &out[i]);
}

static void test_sha256_hmac_vectors(void)
{
    uint8_t d[32], e[32];
    sha256("abc", 3, d);
    hex("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", e);
    CHECK(memcmp(d, e, 32) == 0);
    /* 2-block message */
    const char *m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    sha256(m, strlen(m), d);
    hex("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", e);
    CHECK(memcmp(d, e, 32) == 0);
    /* RFC 4231 test case 2 */
    const char *msg = "what do ya want for nothing?";
    hmac_sha256((const uint8_t *)"Jefe", 4, msg, strlen(msg), d);
    hex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", e);
    CHECK(memcmp(d, e, 32) == 0);
    CHECK(ct_equal(d, e, 32));
    e[31] ^= 1;
    CHECK(!ct_equal(d, e, 32));
}

static profile_params_t max_profile(void)
{
    profile_params_t p = {
        .motor_current_max = 150.0f, .motor_current_regen_max = 60.0f,
        .battery_current_max = 100.0f, .battery_current_regen_max = 30.0f,
        .abs_current_max = 160.0f, .erpm_max = 30000.0f, .current_ramp_a_per_s = 500.0f,
        .fw_max_current = 20.0f, .speed_limit_kmh = 0.0f, .throttle_expo = 0.2f,
    };
    return p;
}

static void test_envelope_refuses(void)
{
    profile_params_t p = max_profile();
    CHECK(envelope_check_profile(&p) == ENV_OK);          /* 150 A is allowed */

    p = max_profile(); p.motor_current_max = 151.0f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_MOTOR_CURRENT);
    p = max_profile(); p.abs_current_max = 200.0f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_ABS_TRIP);
    p = max_profile(); p.abs_current_max = 150.0f;        /* no margin above 150 A */
    CHECK(envelope_check_profile(&p) == ENV_VIOL_ABS_TRIP_MARGIN);
    p = max_profile(); p.battery_current_max = 120.0f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_BATTERY_CURRENT);
    p = max_profile(); p.battery_current_regen_max = 80.0f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_BATTERY_REGEN);
    p = max_profile(); p.motor_current_max = NAN;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_NOT_FINITE);
    p = max_profile(); p.fw_max_current = -5.0f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_NOT_FINITE);
    p = max_profile(); p.erpm_max = 1e9f;
    CHECK(envelope_check_profile(&p) == ENV_VIOL_ERPM);

    /* Clamp path (tampered flash): everything ends up legal. */
    p = max_profile();
    p.motor_current_max = 999.0f; p.abs_current_max = INFINITY; p.battery_current_max = NAN;
    CHECK(!envelope_clamp_profile(&p));
    CHECK(envelope_check_profile(&p) == ENV_OK);
    CHECK(p.motor_current_max <= ENV_PHASE_CURRENT_MAX);

    app_config_t c;
    config_load_defaults(&c);
    CHECK(envelope_check_config(&c) == ENV_OK);
    c.foc.max_modulation = 1.2f;
    CHECK(envelope_check_config(&c) == ENV_VIOL_MODULATION);
}

static void test_preset_scaling(void)
{
    app_config_t c;
    config_load_defaults(&c);
    profile_params_t eco, sport, race;
    profile_params_for(&c, PROFILE_ECO, &eco);
    profile_params_for(&c, PROFILE_SPORT, &sport);
    profile_params_for(&c, PROFILE_RACE, &race);
    CHECK_NEAR(eco.motor_current_max, 0.5 * c.limits.motor_current_max, 1e-3);
    CHECK(eco.motor_current_max < sport.motor_current_max && sport.motor_current_max < race.motor_current_max);
    CHECK_NEAR(race.motor_current_max, c.limits.motor_current_max, 1e-3);
    /* Presets never exceed the factory speed limit. */
    CHECK(eco.speed_limit_kmh < c.vehicle.speed_limit_kmh);
    CHECK(race.speed_limit_kmh <= c.vehicle.speed_limit_kmh);
    CHECK(envelope_check_profile(&race) == ENV_OK);
}

static void test_wrap_never_hangs(void)
{
    /* These used to loop forever on Inf / huge values inside the 20 kHz ISR. */
    CHECK(wrap_2pi(INFINITY) == 0.0f);
    CHECK(wrap_pi(-INFINITY) == 0.0f);
    CHECK(wrap_2pi(NAN) == 0.0f);
    float w = wrap_2pi(1e30f);
    CHECK(w >= 0.0f && w < M_2PI_F);
    CHECK_NEAR(wrap_pi(7.0f * M_2PI_F + 1.0f), 1.0, 1e-4);
    CHECK_NEAR(wrap_2pi(-0.5f), M_2PI_F - 0.5f, 1e-6);
}

static void test_envelope_rejects_bad_calibration(void)
{
    app_config_t c;
    config_load_defaults(&c);
    c.motor.hall_angle_rad[3] = INFINITY;
    CHECK(envelope_check_config(&c) == ENV_VIOL_NOT_FINITE);
    config_load_defaults(&c);
    c.motor.hall_angle_rad[3] = 100.0f;
    CHECK(envelope_check_config(&c) == ENV_VIOL_SENSOR_CALIBRATION);
    config_load_defaults(&c);
    c.vehicle.throttle_v_max = c.vehicle.throttle_v_min;          /* divide by zero */
    CHECK(envelope_check_config(&c) == ENV_VIOL_INPUT_CALIBRATION);
    config_load_defaults(&c);
    c.vehicle.throttle_v_fault_high = 3.0f;                        /* full throttle = "fault" */
    CHECK(envelope_check_config(&c) == ENV_VIOL_INPUT_CALIBRATION);
    config_load_defaults(&c);
    c.vehicle.brake_switch_enabled = 0x2C;                         /* not a valid bool */
    CHECK(envelope_check_config(&c) == ENV_VIOL_INVALID_ENUM);
    config_load_defaults(&c);
    c.vehicle.regen_on_brake_fraction = NAN;
    CHECK(envelope_check_config(&c) == ENV_VIOL_NOT_FINITE);

    /* Flash path: validate repairs to safe values instead of refusing. */
    config_load_defaults(&c);
    c.vehicle.speed_limit_kmh = NAN;
    c.vehicle.wheel_diameter_m = NAN;
    c.limits.motor_current_max = NAN;
    c.motor.hall_angle_rad[1] = -INFINITY;
    c.vehicle.throttle_v_min = 5.0f;
    CHECK(!config_validate(&c));
    CHECK(c.vehicle.speed_limit_kmh == 25.0f);          /* not 0 = unlimited */
    CHECK(c.vehicle.wheel_diameter_m == 1.0f);          /* speedometer over-reads: safe */
    CHECK(c.limits.motor_current_max == 0.0f);
    CHECK(c.motor.hall_angle_rad[1] == -1.0f);          /* forces re-detection */
    CHECK(c.vehicle.throttle_v_min < c.vehicle.throttle_v_max);
    CHECK(envelope_check_config(&c) == ENV_OK);
}

static void test_hall_recovers_without_sector_change(void)
{
    const float angles[8] = {-1.0f, 0.0f, 2.0f * M_PI_F / 3.0f, M_PI_F / 3.0f,
                            4.0f * M_PI_F / 3.0f, 5.0f * M_PI_F / 3.0f, M_PI_F, -1.0f};
    const uint8_t invalid_codes[] = {0, 7};
    for (unsigned i = 0; i < 2; i++) {
        hall_sensor_t h;
        hall_init(&h, angles, 170000000.0f);
        hall_update(&h, 1, 1000);
        CHECK(h.valid);
        hall_update(&h, invalid_codes[i], 2000);
        CHECK(!h.valid);
        hall_update(&h, 1, 3000);
        CHECK(h.valid);
        CHECK_NEAR(h.theta, angles[1], 1e-5f);
        CHECK(h.omega == 0.0f);
    }
}

int main(void)
{
    RUN_TEST(test_sha256_hmac_vectors);
    RUN_TEST(test_envelope_refuses);
    RUN_TEST(test_preset_scaling);
    RUN_TEST(test_wrap_never_hangs);
    RUN_TEST(test_envelope_rejects_bad_calibration);
    RUN_TEST(test_crc_vectors);
    RUN_TEST(test_sincos_atan2);
    RUN_TEST(test_clarke_park_roundtrip);
    RUN_TEST(test_svpwm);
    RUN_TEST(test_pi_antiwindup);
    RUN_TEST(test_pll_tracks);
    RUN_TEST(test_current_sense);
    RUN_TEST(test_throttle_safety);
    RUN_TEST(test_hall_speed);
    RUN_TEST(test_hall_recovers_without_sector_change);
    RUN_TEST(test_limits_derating);
    RUN_TEST(test_mtpa);
    RUN_TEST(test_config_storage);
    RUN_TEST(test_config_validate_clamps);
    RUN_TEST(test_fault_manager);
    return TEST_SUMMARY();
}
