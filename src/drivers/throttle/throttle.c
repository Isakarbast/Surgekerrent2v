#include "throttle.h"

#include "util_math.h"

#define THROTTLE_FAULT_MS  20u
#define THROTTLE_ARM_MS    200u
#define THROTTLE_DEADBAND  0.03f

void throttle_init(throttle_t *t, float v_min, float v_max, float v_fault_low,
                   float v_fault_high, float expo)
{
    t->v_min = v_min;
    t->v_max = v_max;
    t->v_fault_low = v_fault_low;
    t->v_fault_high = v_fault_high;
    t->expo = expo;
    t->volts_filt = v_min;
    t->value = 0.0f;
    t->fault_ms = 0;
    t->closed_ms = 0;
    t->fault = false;
    t->armed = false;
}

void throttle_disarm(throttle_t *t)
{
    t->armed = false;
    t->closed_ms = 0;
}

float throttle_update(throttle_t *t, float volts)
{
    /* ~30 Hz LPF at 1 kHz: kills PWM/EMI noise without feeling laggy. */
    lpf_update(&t->volts_filt, volts, 0.17f);

    if (volts < t->v_fault_low || volts > t->v_fault_high) {
        if (t->fault_ms < 0xFFFF) t->fault_ms++;
        if (t->fault_ms >= THROTTLE_FAULT_MS) {
            t->fault = true;
            throttle_disarm(t);
        }
    } else {
        t->fault_ms = 0;
        t->fault = false;
    }

    float x = mapf_clamped(t->volts_filt, t->v_min, t->v_max, 0.0f, 1.0f);
    x = (x < THROTTLE_DEADBAND) ? 0.0f : (x - THROTTLE_DEADBAND) / (1.0f - THROTTLE_DEADBAND);
    x = (1.0f - t->expo) * x + t->expo * x * x * x;

    if (!t->armed) {
        if (x == 0.0f && !t->fault) {
            if (++t->closed_ms >= THROTTLE_ARM_MS) t->armed = true;
        } else {
            t->closed_ms = 0;
        }
    }

    t->value = (t->armed && !t->fault) ? x : 0.0f;
    return t->value;
}
