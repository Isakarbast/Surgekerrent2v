/*
 * AS5047P 14-bit magnetic absolute encoder over SPI (mode 1, 16-bit).
 * Reads ANGLECOM (dynamic-angle-error-compensated angle) every call.
 * The SPI protocol is pipelined: each response belongs to the previous
 * command, which is fine because we issue the same read every period.
 */
#ifndef AS5047P_H
#define AS5047P_H

#include <stdbool.h>
#include <stdint.h>

#include "pll.h"

typedef struct {
    uint8_t pole_pairs;
    int8_t direction;
    float offset;          /* electrical offset [rad] from detection */
    float mech_angle;      /* [rad] */
    float elec_angle;      /* [rad] */
    pll_t pll;             /* speed estimate */
    uint16_t error_count;
    bool fault;
} as5047p_t;

void as5047p_init(as5047p_t *e, uint8_t pole_pairs, int8_t direction, float offset);
/* Returns false on parity / error-flag failure (last good angle retained). */
bool as5047p_update(as5047p_t *e, float dt);

#endif
