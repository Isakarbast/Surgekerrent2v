/*
 * config_storage.h - power-loss-safe config persistence.
 *
 * Two flash slots (A/B). Each save goes to the slot NOT holding the newest
 * valid copy, with an incremented sequence number; the old copy is only
 * superseded once the new one is fully written and verified. A brown-out
 * mid-write therefore leaves the previous config intact.
 */
#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <stdbool.h>

#include "config.h"

typedef enum {
    STORAGE_LOADED = 0,
    STORAGE_DEFAULTS_NO_DATA,
    STORAGE_DEFAULTS_CORRUPT,
    STORAGE_DEFAULTS_VERSION,
} storage_result_t;

storage_result_t config_storage_load(app_config_t *cfg);
/* Only call with the motor stopped (flash erase stalls the bus on single-bank parts). */
bool config_storage_save(app_config_t *cfg);
void config_update_crc(app_config_t *cfg);
bool config_crc_ok(const app_config_t *cfg);

#endif
