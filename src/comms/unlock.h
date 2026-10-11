/*
 * unlock.h - authenticated unlock of the CUSTOM ("VESC") profile.
 *
 * Only an app holding the secret key can unlock advanced mode, and only for
 * THIS controller and a current version of the terms:
 *
 *   1. app  -> UNLOCK_CHALLENGE
 *      ctrl -> nonce[16] (hardware RNG, single use, 30 s TTL), uid[12], terms_required
 *   2. rider reads the terms in the app and taps "I agree"
 *   3. app  -> UNLOCK_RESPONSE: terms_version (u16), mac[32]
 *      mac = HMAC-SHA256(key, "SK2V-UNLOCK-v1" | nonce | uid | terms_version_le16)
 *   4. ctrl verifies, stores unlocked + terms_version in flash
 *
 * 5 wrong responses -> 60 s lockout. Raising UNLOCK_TERMS_VERSION_REQUIRED
 * in a firmware update revokes existing unlocks (rider must agree again).
 * See docs/PROFILES_AND_UNLOCK.md for the app-side spec.
 */
#ifndef UNLOCK_H
#define UNLOCK_H

#include <stdbool.h>
#include <stdint.h>

#define UNLOCK_TERMS_VERSION_REQUIRED 1u
#define UNLOCK_NONCE_LEN              16u
#define UNLOCK_UID_LEN                12u
#define UNLOCK_MAC_LEN                32u
#define UNLOCK_KEY_LEN                32u
#define UNLOCK_MAX_FAILS              5u
#define UNLOCK_LOCKOUT_MS             60000u
#define UNLOCK_CHALLENGE_TTL_MS       30000u

typedef enum {
    UNLOCK_OK = 0,
    UNLOCK_ERR_NO_CHALLENGE,     /* no challenge issued, or it was already used */
    UNLOCK_ERR_EXPIRED,
    UNLOCK_ERR_BAD_MAC,
    UNLOCK_ERR_TERMS_VERSION,    /* agreed to an outdated terms version */
    UNLOCK_ERR_LOCKED_OUT,
    UNLOCK_ERR_FLASH,
} unlock_result_t;

void unlock_init(void);
/* Returns false while locked out. */
bool unlock_issue_challenge(uint8_t nonce[UNLOCK_NONCE_LEN], uint8_t uid[UNLOCK_UID_LEN]);
/* Verifies and, on success, persists the unlock. */
unlock_result_t unlock_verify(uint16_t terms_version, const uint8_t mac[UNLOCK_MAC_LEN]);
/* Revoke (always allowed). Persists. */
bool unlock_revoke(void);
bool unlock_is_active(void);
bool unlock_using_dev_key(void);

/* The MAC computation, shared with tests and documented for the app team. */
void unlock_compute_mac(const uint8_t key[UNLOCK_KEY_LEN], const uint8_t nonce[UNLOCK_NONCE_LEN],
                        const uint8_t uid[UNLOCK_UID_LEN], uint16_t terms_version,
                        uint8_t mac[UNLOCK_MAC_LEN]);
extern const uint8_t g_unlock_key[UNLOCK_KEY_LEN];

#endif
