/*
 * pas.h - pedal assist sensor (cadence type, N magnets, single output).
 * Assist is only allowed while pedals turn forward above a minimum
 * cadence, and stops within pas_stop_timeout_ms after pedalling stops
 * (EN 15194 requires stop within 2 s / 5 m; default here is 0.4 s).
 */
#ifndef PAS_H
#define PAS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t magnets;
    float start_rpm;
    uint16_t timeout_ms;
    bool last_level;
    uint32_t ms_since_edge;
    uint32_t last_period_ms;
    uint8_t valid_edges;
    float cadence_rpm;
    bool pedalling;
} pas_t;

void pas_init(pas_t *p, uint8_t magnets, float start_rpm, uint16_t timeout_ms);
void pas_update(pas_t *p, bool level); /* 1 kHz */

#endif
