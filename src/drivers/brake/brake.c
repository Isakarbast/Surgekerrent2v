#include "brake.h"

#include "util_math.h"

#define BRAKE_DEBOUNCE_MS 5u

void brake_init(brake_t *b, bool switch_en, bool analog_en, float v_min, float v_max)
{
    b->switch_enabled = switch_en;
    b->analog_enabled = analog_en;
    b->v_min = v_min;
    b->v_max = v_max;
    b->debounce = 0;
    b->switch_active = false;
    b->analog_value = 0.0f;
    b->analog_fault = false;
}

void brake_update(brake_t *b, bool switch_raw, float analog_volts)
{
    if (b->switch_enabled) {
        /* Asymmetric: engage fast (1 ms), release only after debounce. */
        if (switch_raw) {
            b->switch_active = true;
            b->debounce = BRAKE_DEBOUNCE_MS;
        } else if (b->debounce > 0) {
            b->debounce--;
        } else {
            b->switch_active = false;
        }
    } else {
        b->switch_active = false;
    }

    if (b->analog_enabled) {
        b->analog_fault = (analog_volts < 0.4f) || (analog_volts > 4.6f);
        b->analog_value = b->analog_fault ? 0.0f
                        : mapf_clamped(analog_volts, b->v_min, b->v_max, 0.0f, 1.0f);
        if (b->analog_value < 0.05f) b->analog_value = 0.0f;
    } else {
        b->analog_value = 0.0f;
    }
}
