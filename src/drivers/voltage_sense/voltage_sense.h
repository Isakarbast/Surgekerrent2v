#ifndef VOLTAGE_SENSE_H
#define VOLTAGE_SENSE_H

#include <stdint.h>

/* Bus voltage from the divider. Filtered copy for the slow loops. */
typedef struct {
    float scale;       /* volts per count */
    float vbus;        /* instantaneous (fast loop) */
    float vbus_filt;   /* ~100 Hz LPF */
    float alpha;
} voltage_sense_t;

void vs_init(voltage_sense_t *vs, float adc_vref, float divider_ratio, float fast_dt);
float vs_update(voltage_sense_t *vs, uint16_t raw);

#endif
