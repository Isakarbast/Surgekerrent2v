#include "current_sense.h"

#include "board_config.h"
#include "util_math.h"

#define CS_OFFSET_TOLERANCE 0.10f   /* +-10 % of full scale around mid-rail */
#define CS_SUM_ERR_LIMIT_A  (0.08f * BOARD_MAX_PHASE_CURRENT)

void cs_init(current_sense_t *cs, float amps_per_count, float max_duty_sample)
{
    cs->amps_per_count = amps_per_count;
    for (int i = 0; i < 3; i++) cs->offset[i] = BOARD_ADC_MAX_COUNTS * 0.5f;
    cs->max_duty_sample = max_duty_sample;
    cs->cal_count = cs->cal_target = 0;
    cs->sum_err_filt = 0.0f;
    cs->sum_err_trips = 0;
    cs->reconstructed_phase = 0xFF;
    cs->status = CS_CALIBRATING;
}

void cs_calibrate_start(current_sense_t *cs, uint32_t samples)
{
    cs->cal_count = 0;
    cs->cal_target = samples;
    cs->cal_acc[0] = cs->cal_acc[1] = cs->cal_acc[2] = 0.0f;
    cs->status = CS_CALIBRATING;
}

bool cs_calibrate_sample(current_sense_t *cs, const hal_adc_fast_t *raw)
{
    if (cs->status != CS_CALIBRATING) return true;

    cs->cal_acc[0] += raw->ia;
    cs->cal_acc[1] += raw->ib;
    cs->cal_acc[2] += raw->ic;
    if (++cs->cal_count < cs->cal_target) return false;

    const float mid = BOARD_ADC_MAX_COUNTS * 0.5f;
    const float tol = BOARD_ADC_MAX_COUNTS * CS_OFFSET_TOLERANCE;
    cs->status = CS_OK;
    for (int i = 0; i < 3; i++) {
        cs->offset[i] = cs->cal_acc[i] / (float)cs->cal_count;
        if (absf(cs->offset[i] - mid) > tol) cs->status = CS_ERR_OFFSET_RANGE;
    }
    return true;
}

abc_t cs_convert(current_sense_t *cs, const hal_adc_fast_t *raw, const float prev_duty[3])
{
#if BOARD_CSA_INVERTED
    const float k = -cs->amps_per_count;
#else
    const float k = cs->amps_per_count;
#endif
    float i[3];
    i[0] = ((float)raw->ia - cs->offset[0]) * k;
    i[1] = ((float)raw->ib - cs->offset[1]) * k;
    i[2] = ((float)raw->ic - cs->offset[2]) * k;

    /* Find the phase with the highest duty: shortest low-side window. */
    int hi = 0;
    if (prev_duty[1] > prev_duty[hi]) hi = 1;
    if (prev_duty[2] > prev_duty[hi]) hi = 2;

    if (prev_duty[hi] > cs->max_duty_sample) {
        /* Only one phase can be above the limit with SVPWM at < 100 %
         * modulation, so the other two are always valid. */
        i[hi] = -(i[(hi + 1) % 3] + i[(hi + 2) % 3]);
        cs->reconstructed_phase = (uint8_t)hi;
    } else {
        cs->reconstructed_phase = 0xFF;
        /* Kirchhoff check only meaningful when all three were measured. */
        lpf_update(&cs->sum_err_filt, absf(i[0] + i[1] + i[2]), 0.01f);
        if (cs->sum_err_filt > CS_SUM_ERR_LIMIT_A) {
            if (cs->sum_err_trips < 0xFFFF) cs->sum_err_trips++;
            if (cs->sum_err_trips > 200) cs->status = CS_ERR_SUM;
        } else if (cs->sum_err_trips > 0) {
            cs->sum_err_trips--;
        }
    }

    abc_t out = {i[0], i[1], i[2]};
    return out;
}
