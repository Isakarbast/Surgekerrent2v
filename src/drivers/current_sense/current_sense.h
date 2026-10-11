/*
 * current_sense.h - three low-side shunt phase current measurement.
 *
 * Responsibilities:
 *  - zero-current offset calibration (at boot, outputs off)
 *  - ADC counts -> amps, including amplifier polarity
 *  - reconstruction of the phase whose low-side on-time is too short to
 *    sample (high duty), using ia + ib + ic = 0
 *  - plausibility checks (offset drift, Kirchhoff sum) -> sensor fault
 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_math.h"
#include "hal.h"

typedef enum {
    CS_OK = 0,
    CS_CALIBRATING,
    CS_ERR_OFFSET_RANGE,   /* amplifier output far from mid-rail: broken CSA / shunt */
    CS_ERR_SUM,            /* ia+ib+ic persistently != 0: one channel wrong */
} cs_status_t;

typedef struct {
    float amps_per_count;
    float offset[3];        /* in counts */
    float max_duty_sample;  /* above this, that phase is reconstructed */
    /* calibration accumulators */
    uint32_t cal_count, cal_target;
    float cal_acc[3];
    /* diagnostics */
    float sum_err_filt;
    uint16_t sum_err_trips;
    uint8_t reconstructed_phase; /* 0..2, or 0xFF */
    cs_status_t status;
} current_sense_t;

void cs_init(current_sense_t *cs, float amps_per_count, float max_duty_sample);
/* Start averaging; feed samples with cs_calibrate_sample until done. */
void cs_calibrate_start(current_sense_t *cs, uint32_t samples);
/* Returns true when calibration is finished (check cs->status). */
bool cs_calibrate_sample(current_sense_t *cs, const hal_adc_fast_t *raw);

/*
 * Convert a sample. prev_duty = duties that were active WHEN the sample was
 * taken (i.e. last period's output). Returns phase currents in amps.
 */
abc_t cs_convert(current_sense_t *cs, const hal_adc_fast_t *raw, const float prev_duty[3]);

#endif
