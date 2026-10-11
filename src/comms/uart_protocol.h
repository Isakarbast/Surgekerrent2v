/*
 * uart_protocol.h - binary config/telemetry protocol (USB-UART / BLE bridge).
 *
 * Frame:  0xAA | len | cmd | payload[len-1] | crc16_hi | crc16_lo
 *   len  = 1 + payload length (max 250)
 *   crc  = CRC-16/CCITT-FALSE over len, cmd, payload
 * Replies use the same framing with cmd | 0x80. NACK = 0x7F + [cmd, reason].
 *
 * Commands that move the motor or write flash are only accepted while the
 * motor is stopped and idle. NACK = 0x7F + [cmd, reason, detail].
 *
 * Lock policy:
 *   locked   : ride profiles ECO/SPORT/RACE, PAS, detection, telemetry only.
 *              COMMIT_CONFIG is refused.
 *   unlocked : COMMIT_CONFIG allowed, but factory fields stay protected and
 *              every value is checked against the safety envelope - a
 *              violation is refused, never silently clamped.
 * Config edit session: GET_CONFIG at offset 0 re-syncs the staging buffer.
 */
#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTO_SOF        0xAAu
#define PROTO_MAX_LEN    250u

typedef enum {
    CMD_PING            = 0x01,
    CMD_GET_TELEMETRY   = 0x02,
    CMD_GET_CONFIG      = 0x03, /* [u16 offset] -> [u16 offset][data...] (<=128 B) */
    CMD_SET_CONFIG      = 0x04, /* [u16 offset][data...] into staging buffer */
    CMD_COMMIT_CONFIG   = 0x05, /* validate staging (CRC), apply and save */
    CMD_SAVE_CONFIG     = 0x06, /* save current running config */
    CMD_LOAD_DEFAULTS   = 0x07,
    CMD_CLEAR_FAULTS    = 0x08,
    CMD_START_DETECTION = 0x09,
    CMD_DETECT_STATUS   = 0x0A,
    CMD_APPLY_DETECTION = 0x0B, /* copy detection results into config (not saved) */
    CMD_GET_FAULT_LOG   = 0x0C,
    CMD_SET_PAS_LEVEL   = 0x0D, /* [u8 level] */
    CMD_REBOOT          = 0x0E,
    CMD_PROFILE_INFO    = 0x10, /* -> active/boot profile, lock state, envelope, active params */
    CMD_SET_PROFILE     = 0x11, /* [u8 profile] */
    CMD_UNLOCK_CHALLENGE= 0x12, /* -> nonce[16] uid[12] u16 terms_required */
    CMD_UNLOCK_RESPONSE = 0x13, /* [u16 terms_version][mac 32] */
    CMD_LOCK_ADVANCED   = 0x14, /* revoke unlock */
    CMD_SET_BOOT_PROFILE= 0x15, /* [u8 profile] ECO/SPORT/RACE, saved */
    CMD_NACK            = 0x7F,
} proto_cmd_t;

typedef enum {
    NACK_UNKNOWN_CMD = 1,
    NACK_BAD_LENGTH,
    NACK_NOT_IDLE,
    NACK_BAD_CRC,
    NACK_FLASH,
    NACK_INVALID,
    NACK_THROTTLE_HELD,
    NACK_LOCKED,          /* advanced mode not unlocked */
    NACK_ENVELOPE,        /* detail byte = env_violation_t */
    NACK_PROTECTED_FIELD, /* tried to change factory limits / speed limit / wheel size */
    NACK_AUTH,            /* detail byte = unlock_result_t */
    NACK_NOT_STOPPED,
} proto_nack_t;

void proto_init(void);
void proto_poll(void);   /* call from main loop */

/* Exposed for tests: feed bytes, get a reply through a sink. */
typedef void (*proto_tx_fn)(const uint8_t *data, size_t len);
void proto_set_tx(proto_tx_fn fn);
void proto_rx_byte(uint8_t b);
size_t proto_build_frame(uint8_t cmd, const uint8_t *payload, size_t len, uint8_t *out);

#endif
