#include "ntc.h"

#include <math.h>

#include "board_config.h"
#include "util_math.h"

void ntc_init(ntc_t *n, float r_pullup, float r25, float beta)
{
    n->r_pullup = r_pullup;
    n->r25 = r25;
    n->beta = beta;
    n->temp_c = 25.0f;
    n->fault = false;
}

float ntc_counts_to_celsius(float counts, float r_pullup, float r25, float beta)
{
    float ratio = counts / BOARD_ADC_MAX_COUNTS;
    ratio = clampf(ratio, 1e-4f, 0.9999f);
    float r = r_pullup * ratio / (1.0f - ratio);
    float inv_t = 1.0f / 298.15f + logf(r / r25) / beta;
    return 1.0f / inv_t - 273.15f;
}

float ntc_update(ntc_t *n, uint16_t raw)
{
    if (n->r25 <= 0.0f) return n->temp_c; /* disabled */
    /* <1 % or >99 % of range: open circuit / short. Retain the last
     * temperature; vehicle_control applies derating from the fault flag. */
    if (raw < BOARD_ADC_MAX_COUNTS * 0.01f || raw > BOARD_ADC_MAX_COUNTS * 0.99f) {
        n->fault = true;
        return n->temp_c;
    }
    n->fault = false;
    float t = ntc_counts_to_celsius((float)raw, n->r_pullup, n->r25, n->beta);
    lpf_update(&n->temp_c, t, 0.1f);
    return n->temp_c;
}
