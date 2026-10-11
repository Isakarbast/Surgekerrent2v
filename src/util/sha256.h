/*
 * sha256.h - SHA-256 and HMAC-SHA256 (FIPS 180-4, RFC 2104).
 * Used for the advanced-mode unlock challenge/response.
 */
#ifndef SHA256_H
#define SHA256_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t state[8];
    uint64_t bit_len;
    uint8_t buf[64];
    uint32_t buf_len;
} sha256_ctx_t;

void sha256_init(sha256_ctx_t *c);
void sha256_update(sha256_ctx_t *c, const void *data, size_t len);
void sha256_final(sha256_ctx_t *c, uint8_t out[32]);
void sha256(const void *data, size_t len, uint8_t out[32]);

void hmac_sha256(const uint8_t *key, size_t key_len, const void *msg, size_t msg_len, uint8_t out[32]);

/* Constant-time comparison: timing doesn't reveal how many bytes matched. */
bool ct_equal(const uint8_t *a, const uint8_t *b, size_t len);

#endif
