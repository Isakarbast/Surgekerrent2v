/*
 * brake.h - brake lever switch (digital, debounced) and optional analog
 * (hall) brake lever for proportional regen. Brake ALWAYS overrides throttle.
 * A faulted analog brake reports 0 regen but the switch still cuts drive.
 */
#ifndef BRAKE_H
#define BRAKE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool switch_enabled, analog_enabled;
    float v_min, v_max;
    uint8_t debounce;
    bool switch_active;
    float analog_value;    /* 0..1 */
    bool analog_fault;
} brake_t;

void brake_init(brake_t *b, bool switch_en, bool analog_en, float v_min, float v_max);
void brake_update(brake_t *b, bool switch_raw, float analog_volts); /* 1 kHz */
/* True if drive torque must be cut. */
static inline bool brake_cut_drive(const brake_t *b)
{
    return b->switch_active || (b->analog_enabled && b->analog_value > 0.0f);
}

#endif
