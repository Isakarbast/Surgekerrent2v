#include "uart_protocol.h"

#include <string.h>

#include "app.h"
#include "board_config.h"
#include "config_storage.h"
#include "crc.h"
#include "fault_manager.h"
#include "hal.h"
#include "motor_control.h"
#include "ride_profiles.h"
#include "safety_envelope.h"
#include "unlock.h"
#include "telemetry.h"
#include "util_math.h"
#include "vehicle_control.h"

typedef enum { RX_SOF, RX_LEN, RX_BODY, RX_CRC1, RX_CRC2 } rx_state_t;

static rx_state_t s_rx;
static uint8_t s_len, s_pos;
static uint8_t s_body[PROTO_MAX_LEN];
static uint16_t s_crc_rx;
static app_config_t s_staging;
static proto_tx_fn s_tx;

static void tx_uart(const uint8_t *d, size_t n) { (void)hal_uart_write(d, n); }

void proto_set_tx(proto_tx_fn fn) { s_tx = fn; }

void proto_init(void)
{
    s_rx = RX_SOF;
    s_tx = tx_uart;
    memcpy(&s_staging, &g_config, sizeof(s_staging));
}

size_t proto_build_frame(uint8_t cmd, const uint8_t *payload, size_t len, uint8_t *out)
{
    if (len > PROTO_MAX_LEN - 1u) return 0;
    out[0] = PROTO_SOF;
    out[1] = (uint8_t)(len + 1u);
    out[2] = cmd;
    if (len) memcpy(&out[3], payload, len);
    uint16_t crc = crc16_ccitt(&out[1], len + 2u, 0xFFFFu);
    out[3 + len] = (uint8_t)(crc >> 8);
    out[4 + len] = (uint8_t)crc;
    return len + 5u;
}

static void reply(uint8_t cmd, const void *payload, size_t len)
{
    uint8_t frame[PROTO_MAX_LEN + 5];
    size_t n = proto_build_frame(cmd | 0x80u, (const uint8_t *)payload, len, frame);
    if (n && s_tx) s_tx(frame, n);
}

static void nack_detail(uint8_t cmd, proto_nack_t why, uint8_t detail)
{
    uint8_t p[3] = {cmd, (uint8_t)why, detail};
    uint8_t frame[8];
    size_t n = proto_build_frame(CMD_NACK, p, 3, frame);
    if (s_tx) s_tx(frame, n);
}

static void nack(uint8_t cmd, proto_nack_t why) { nack_detail(cmd, why, 0); }

/* Fields only the factory firmware image may set: they define the power
 * limits of ECO/SPORT/RACE and the speedometer (and so the legal limit). */
static bool protected_fields_unchanged(const app_config_t *a, const app_config_t *b)
{
    return memcmp(&a->limits, &b->limits, sizeof(a->limits)) == 0 &&
           a->vehicle.type == b->vehicle.type &&
           memcmp(&a->vehicle.speed_limit_kmh, &b->vehicle.speed_limit_kmh, sizeof(float)) == 0 &&
           memcmp(&a->vehicle.speed_limit_margin_kmh, &b->vehicle.speed_limit_margin_kmh, sizeof(float)) == 0 &&
           memcmp(&a->vehicle.wheel_diameter_m, &b->vehicle.wheel_diameter_m, sizeof(float)) == 0 &&
           memcmp(&a->vehicle.gear_ratio, &b->vehicle.gear_ratio, sizeof(float)) == 0 &&
           a->vehicle.battery_cells_series == b->vehicle.battery_cells_series &&
           a->motor.pole_pairs == b->motor.pole_pairs &&
           a->ride.advanced_unlocked == b->ride.advanced_unlocked &&
           a->ride.terms_version == b->ride.terms_version;
}

static bool motor_idle(void)
{
    return (g_mc.state == MC_STATE_IDLE || g_mc.state == MC_STATE_FAULT) && mc_is_stopped();
}

static void apply_new_config(void)
{
    profiles_refresh();                 /* rebuild g_active_config */
    /* Both loops keep running while the motor is idle: swap their state
     * with IRQs off so neither sees a half-initialised struct (~10 us). */
    uint32_t irq = hal_irq_save();
    mc_apply_config(&g_active_config);
    vehicle_init(&g_active_config);
    hal_irq_restore(irq);
}

static void handle(uint8_t cmd, const uint8_t *p, uint8_t n)
{
    switch (cmd) {
    case CMD_PING: {
        uint8_t r[8] = {'S', 'K', '2', 'V', FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH,
                        (uint8_t)((unlock_using_dev_key() ? 1u : 0u) | (unlock_is_active() ? 2u : 0u))};
        reply(cmd, r, sizeof(r));
        break;
    }
    case CMD_GET_TELEMETRY: {
        telemetry_t t;
        telemetry_fill(&t);
        reply(cmd, &t, sizeof(t));
        break;
    }
    case CMD_GET_CONFIG: {
        if (n != 2) { nack(cmd, NACK_BAD_LENGTH); break; }
        uint16_t off = (uint16_t)(p[0] | (p[1] << 8));
        if (off >= sizeof(app_config_t)) { nack(cmd, NACK_INVALID); break; }
        if (off == 0) memcpy(&s_staging, &g_config, sizeof(s_staging)); /* new edit session */
        uint8_t r[2 + 128];
        size_t len = sizeof(app_config_t) - off;
        if (len > 128) len = 128;
        r[0] = p[0];
        r[1] = p[1];
        memcpy(&r[2], (const uint8_t *)&g_config + off, len);
        reply(cmd, r, 2 + len);
        break;
    }
    case CMD_SET_CONFIG: {
        if (n < 3) { nack(cmd, NACK_BAD_LENGTH); break; }
        uint16_t off = (uint16_t)(p[0] | (p[1] << 8));
        size_t len = n - 2u;
        if ((size_t)off + len > sizeof(app_config_t)) { nack(cmd, NACK_INVALID); break; }
        memcpy((uint8_t *)&s_staging + off, &p[2], len);
        reply(cmd, NULL, 0);
        break;
    }
    case CMD_COMMIT_CONFIG: {
        if (!unlock_is_active()) { nack(cmd, NACK_LOCKED); break; }
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        if (s_staging.magic != CONFIG_MAGIC || s_staging.version != CONFIG_VERSION ||
            !config_crc_ok(&s_staging)) { nack(cmd, NACK_BAD_CRC); break; }
        if (!protected_fields_unchanged(&s_staging, &g_config)) { nack(cmd, NACK_PROTECTED_FIELD); break; }
        env_violation_t env = envelope_check_config(&s_staging);
        if (env != ENV_OK) { nack_detail(cmd, NACK_ENVELOPE, (uint8_t)env); break; }
        config_validate(&s_staging);
        memcpy(&g_config, &s_staging, sizeof(g_config));
        apply_new_config();
        if (!config_storage_save(&g_config)) { nack(cmd, NACK_FLASH); break; }
        reply(cmd, NULL, 0);
        break;
    }
    case CMD_SAVE_CONFIG:
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        if (!config_storage_save(&g_config)) { nack(cmd, NACK_FLASH); break; }
        reply(cmd, NULL, 0);
        break;
    case CMD_LOAD_DEFAULTS:
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        config_load_defaults(&g_config);   /* factory reset: also re-locks advanced mode */
        config_update_crc(&g_config);
        apply_new_config();
        memcpy(&s_staging, &g_config, sizeof(s_staging));
        reply(cmd, NULL, 0);
        break;
    case CMD_CLEAR_FAULTS:
        if (!fault_clear_latched(g_vehicle.throttle.value == 0.0f)) {
            nack(cmd, NACK_THROTTLE_HELD);
            break;
        }
        app_clear_hw_faults();
        reply(cmd, NULL, 0);
        break;
    case CMD_START_DETECTION:
        if (!motor_idle() || fault_blocks_drive() || !mc_request_detection()) {
            nack(cmd, NACK_NOT_IDLE);
            break;
        }
        reply(cmd, NULL, 0);
        break;
    case CMD_DETECT_STATUS: {
        const motor_detect_t *d = &g_mc.det;
        struct __attribute__((packed)) {
            uint8_t state, error;
            float rs, ld, lq, flux, enc_offset;
            float hall[8];
        } r = {(uint8_t)d->state, (uint8_t)d->error, d->rs, d->ld, d->lq, d->flux, d->enc_offset, {0}};
        for (int i = 0; i < 8; i++) r.hall[i] = d->hall_angle[i];
        reply(cmd, &r, sizeof(r));
        break;
    }
    case CMD_APPLY_DETECTION: {
        const motor_detect_t *d = &g_mc.det;
        if (d->state != DET_DONE || !motor_idle()) { nack(cmd, NACK_INVALID); break; }
        motor_config_t *m = &g_config.motor;
        foc_config_t *f = &g_config.foc;
        m->rs_ohm = d->rs;
        m->ld_h = d->ld;
        m->lq_h = d->lq;
        m->flux_wb = d->flux;
        if (d->do_hall)
            for (int i = 0; i < 8; i++) m->hall_angle_rad[i] = d->hall_angle[i];
        if (d->do_encoder) m->encoder_offset_rad = d->enc_offset;
        /* Pole-zero cancellation tuning: kp = L*wc, ki = R*wc. */
        float wc = M_2PI_F * f->current_bandwidth_hz;
        f->current_kp = 0.5f * (d->ld + d->lq) * wc;
        f->current_ki = d->rs * wc;
        config_validate(&g_config);
        config_update_crc(&g_config);
        apply_new_config();
        reply(cmd, NULL, 0);
        break;
    }
    case CMD_GET_FAULT_LOG: {
        uint8_t count;
        const fault_log_entry_t *log = fault_log(&count);
        const uint8_t max_entries = (uint8_t)((PROTO_MAX_LEN - 2u) / sizeof(fault_log_entry_t));
        uint8_t r[1 + FAULT_LOG_LEN * sizeof(fault_log_entry_t)];
        if (count > max_entries) count = max_entries;
        r[0] = count;
        memcpy(&r[1], log, count * sizeof(fault_log_entry_t));
        reply(cmd, r, 1u + count * sizeof(fault_log_entry_t));
        break;
    }
    case CMD_SET_PAS_LEVEL:
        if (n != 1 || p[0] > g_config.vehicle.pas_levels) { nack(cmd, NACK_INVALID); break; }
        g_vehicle.pas_level = p[0];
        reply(cmd, NULL, 0);
        break;
    case CMD_PROFILE_INFO: {
        struct __attribute__((packed)) {
            uint8_t active, boot, unlocked, dev_key;
            uint16_t terms_accepted, terms_required;
            float env[9];
            profile_params_t active_params;
        } r;
        r.active = (uint8_t)profile_active();
        r.boot = g_config.ride.boot_profile;
        r.unlocked = unlock_is_active() ? 1u : 0u;
        r.dev_key = unlock_using_dev_key() ? 1u : 0u;
        r.terms_accepted = g_config.ride.terms_version;
        r.terms_required = UNLOCK_TERMS_VERSION_REQUIRED;
        const float env[9] = {ENV_PHASE_CURRENT_MAX, ENV_ABS_TRIP_MAX, ENV_BATTERY_CURRENT_MAX,
                              ENV_BATTERY_REGEN_MAX, ENV_FW_CURRENT_MAX, ENV_ERPM_MAX,
                              ENV_RAMP_MAX_A_PER_S, ENV_SPEED_LIMIT_MAX_KMH, ENV_CURRENT_BW_MAX_HZ};
        memcpy(r.env, env, sizeof(env));
        profile_params_t ap;
        profile_params_for(&g_config, profile_active(), &ap);
        memcpy(&r.active_params, &ap, sizeof(ap));
        reply(cmd, &r, sizeof(r));
        break;
    }
    case CMD_SET_PROFILE: {
        if (n != 1) { nack(cmd, NACK_BAD_LENGTH); break; }
        profile_sel_result_t res = profile_select((ride_profile_t)p[0]);
        if (res == PROFILE_SEL_LOCKED) nack(cmd, NACK_LOCKED);
        else if (res == PROFILE_SEL_NOT_STOPPED) nack(cmd, NACK_NOT_STOPPED);
        else if (res != PROFILE_SEL_OK) nack(cmd, NACK_INVALID);
        else reply(cmd, NULL, 0);
        break;
    }
    case CMD_SET_BOOT_PROFILE:
        if (n != 1 || p[0] >= PROFILE_CUSTOM) { nack(cmd, NACK_INVALID); break; }
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        g_config.ride.boot_profile = p[0];
        config_update_crc(&g_config);
        if (!config_storage_save(&g_config)) { nack(cmd, NACK_FLASH); break; }
        reply(cmd, NULL, 0);
        break;
    case CMD_UNLOCK_CHALLENGE: {
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        uint8_t r[UNLOCK_NONCE_LEN + UNLOCK_UID_LEN + 2];
        if (!unlock_issue_challenge(&r[0], &r[UNLOCK_NONCE_LEN])) {
            nack_detail(cmd, NACK_AUTH, UNLOCK_ERR_LOCKED_OUT);
            break;
        }
        r[UNLOCK_NONCE_LEN + UNLOCK_UID_LEN] = (uint8_t)UNLOCK_TERMS_VERSION_REQUIRED;
        r[UNLOCK_NONCE_LEN + UNLOCK_UID_LEN + 1] = (uint8_t)(UNLOCK_TERMS_VERSION_REQUIRED >> 8);
        reply(cmd, r, sizeof(r));
        break;
    }
    case CMD_UNLOCK_RESPONSE: {
        if (n != 2 + UNLOCK_MAC_LEN) { nack(cmd, NACK_BAD_LENGTH); break; }
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        uint16_t terms = (uint16_t)(p[0] | (p[1] << 8));
        unlock_result_t res = unlock_verify(terms, &p[2]);
        if (res != UNLOCK_OK) { nack_detail(cmd, NACK_AUTH, (uint8_t)res); break; }
        memcpy(&s_staging, &g_config, sizeof(s_staging));
        reply(cmd, NULL, 0);
        break;
    }
    case CMD_LOCK_ADVANCED:
        if (profile_active() == PROFILE_CUSTOM && !mc_is_stopped()) { nack(cmd, NACK_NOT_STOPPED); break; }
        if (!unlock_revoke()) { nack(cmd, NACK_FLASH); break; }
        profiles_on_lock();
        memcpy(&s_staging, &g_config, sizeof(s_staging));
        reply(cmd, NULL, 0);
        break;
    case CMD_REBOOT:
        if (!motor_idle()) { nack(cmd, NACK_NOT_IDLE); break; }
        reply(cmd, NULL, 0);
        hal_delay_us(2000);
        hal_system_reset();
        break;
    default:
        nack(cmd, NACK_UNKNOWN_CMD);
        break;
    }
}

void proto_rx_byte(uint8_t b)
{
    switch (s_rx) {
    case RX_SOF:
        if (b == PROTO_SOF) s_rx = RX_LEN;
        break;
    case RX_LEN:
        if (b == 0 || b > PROTO_MAX_LEN) { s_rx = RX_SOF; break; }
        s_len = b;
        s_pos = 0;
        s_rx = RX_BODY;
        break;
    case RX_BODY:
        s_body[s_pos++] = b;
        if (s_pos >= s_len) s_rx = RX_CRC1;
        break;
    case RX_CRC1:
        s_crc_rx = (uint16_t)(b << 8);
        s_rx = RX_CRC2;
        break;
    case RX_CRC2: {
        s_crc_rx |= b;
        uint8_t hdr = s_len;
        uint16_t crc = crc16_ccitt(&hdr, 1, 0xFFFFu);
        crc = crc16_ccitt(s_body, s_len, crc);
        s_rx = RX_SOF;
        if (crc == s_crc_rx) handle(s_body[0], &s_body[1], (uint8_t)(s_len - 1u));
        break;
    }
    }
}

void proto_poll(void)
{
    uint8_t b;
    int budget = 256; /* bound main-loop time */
    while (budget-- > 0 && hal_uart_read_byte(&b)) proto_rx_byte(b);
}
