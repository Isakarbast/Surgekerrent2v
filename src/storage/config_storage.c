#include "config_storage.h"

#include <stddef.h>
#include <string.h>

#include "crc.h"
#include "hal.h"

typedef struct {
    uint32_t seq;
    uint32_t seq_inv;   /* ~seq: detects partially written headers */
    app_config_t cfg;
} slot_image_t;

void config_update_crc(app_config_t *cfg)
{
    cfg->crc = crc32((const uint8_t *)cfg, offsetof(app_config_t, crc));
}

bool config_crc_ok(const app_config_t *cfg)
{
    return cfg->crc == crc32((const uint8_t *)cfg, offsetof(app_config_t, crc));
}

static const slot_image_t *slot(int i) { return (const slot_image_t *)hal_flash_slot_ptr(i); }

static bool slot_valid(const slot_image_t *s)
{
    return s->seq == ~s->seq_inv && s->cfg.magic == CONFIG_MAGIC &&
           s->cfg.size == sizeof(app_config_t) && config_crc_ok(&s->cfg);
}

static int newest_slot(void)
{
    int best = -1;
    uint32_t best_seq = 0;
    for (int i = 0; i < HAL_FLASH_CONFIG_SLOTS; i++) {
        const slot_image_t *s = slot(i);
        if (slot_valid(s) && s->cfg.version == CONFIG_VERSION &&
            (best < 0 || (int32_t)(s->seq - best_seq) > 0)) {
            best = i;
            best_seq = s->seq;
        }
    }
    return best;
}

storage_result_t config_storage_load(app_config_t *cfg)
{
    int n = newest_slot();
    if (n >= 0) {
        memcpy(cfg, &slot(n)->cfg, sizeof(*cfg));
        return STORAGE_LOADED;
    }

    storage_result_t why = STORAGE_DEFAULTS_NO_DATA;
    for (int i = 0; i < HAL_FLASH_CONFIG_SLOTS; i++) {
        const slot_image_t *s = slot(i);
        if (s->cfg.magic == CONFIG_MAGIC) {
            why = (slot_valid(s) && s->cfg.version != CONFIG_VERSION) ? STORAGE_DEFAULTS_VERSION
                                                                       : STORAGE_DEFAULTS_CORRUPT;
        }
    }
    config_load_defaults(cfg);
    return why;
}

bool config_storage_save(app_config_t *cfg)
{
    if (sizeof(slot_image_t) > hal_flash_slot_size()) return false;

    cfg->magic = CONFIG_MAGIC;
    cfg->version = CONFIG_VERSION;
    cfg->size = sizeof(app_config_t);
    config_update_crc(cfg);

    int cur = newest_slot();
    int target = (cur == 0) ? 1 : 0;
    uint32_t seq = (cur >= 0) ? slot(cur)->seq + 1u : 1u;

    static slot_image_t img; /* static: too big for a small stack */
    memset(&img, 0xFF, sizeof(img));
    img.seq = seq;
    img.seq_inv = ~seq;
    memcpy(&img.cfg, cfg, sizeof(*cfg));

    if (!hal_flash_slot_erase(target)) return false;
    /* Write body first, header last: a torn write leaves the header invalid. */
    const uint32_t hdr = offsetof(slot_image_t, cfg);
    if (!hal_flash_slot_write(target, hdr, (const uint8_t *)&img + hdr, sizeof(img) - hdr)) return false;
    if (!hal_flash_slot_write(target, 0, &img, hdr)) return false;

    return memcmp(slot(target), &img, sizeof(img)) == 0;
}
