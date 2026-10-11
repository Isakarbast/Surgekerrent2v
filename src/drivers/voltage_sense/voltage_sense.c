#include "voltage_sense.h"

#include "board_config.h"
#include "util_math.h"

void vs_init(voltage_sense_t *vs, float adc_vref, float divider_ratio, float fast_dt)
{
    vs->scale = adc_vref / BOARD_ADC_MAX_COUNTS * divider_ratio;
    vs->vbus = 0.0f;
    vs->vbus_filt = 0.0f;
    vs->alpha = lpf_alpha(100.0f, fast_dt);
}

float vs_update(voltage_sense_t *vs, uint16_t raw)
{
    vs->vbus = (float)raw * vs->scale;
    if (vs->vbus_filt == 0.0f) vs->vbus_filt = vs->vbus;
    lpf_update(&vs->vbus_filt, vs->vbus, vs->alpha);
    return vs->vbus;
}
