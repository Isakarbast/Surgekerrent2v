#ifndef NTC_H
#define NTC_H

#include <stdbool.h>
#include <stdint.h>

/* NTC to GND with pull-up to ADC reference. */
typedef struct {
    float r_pullup, r25, beta;
    float temp_c;          /* filtered */
    bool fault;            /* open or shorted sensor */
} ntc_t;

void ntc_init(ntc_t *n, float r_pullup, float r25, float beta);
/* Call at ~100 Hz. Returns filtered temperature. */
float ntc_update(ntc_t *n, uint16_t raw);
float ntc_counts_to_celsius(float counts, float r_pullup, float r25, float beta);

#endif
