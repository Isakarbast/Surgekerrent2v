#include "pas.h"

void pas_init(pas_t *p, uint8_t magnets, float start_rpm, uint16_t timeout_ms)
{
    p->magnets = magnets ? magnets : 1;
    p->start_rpm = start_rpm;
    p->timeout_ms = timeout_ms;
    p->last_level = false;
    p->ms_since_edge = 0;
    p->last_period_ms = 0;
    p->valid_edges = 0;
    p->cadence_rpm = 0.0f;
    p->pedalling = false;
}

void pas_update(pas_t *p, bool level)
{
    if (p->ms_since_edge < 0xFFFFFFFFu) p->ms_since_edge++;

    if (level && !p->last_level) { /* rising edge */
        p->last_period_ms = p->ms_since_edge;
        p->ms_since_edge = 0;
        if (p->valid_edges < 255) p->valid_edges++;
        if (p->last_period_ms > 0)
            p->cadence_rpm = 60000.0f / ((float)p->last_period_ms * (float)p->magnets);
    }
    p->last_level = level;

    if (p->ms_since_edge > p->timeout_ms) {
        p->cadence_rpm = 0.0f;
        p->valid_edges = 0;
    }
    /* Require 2 consecutive edges so a single wobble doesn't start the motor. */
    p->pedalling = (p->valid_edges >= 2) && (p->cadence_rpm >= p->start_rpm);
}
