/*
 * throttle.h - analog hall throttle with wire-fault detection.
 *
 * Safety behaviour:
 *  - Signal outside [fault_low, fault_high] for > 20 ms -> fault, output 0.
 *    (A 5 V hall throttle idles ~0.8 V; 0 V = broken wire/ground,
 *     5 V = signal shorted to supply -> would be full throttle!)
 *  - Power-on / fault-recovery interlock: output stays 0 until the throttle
 *    has read closed for 200 ms. Prevents a vehicle launching at boot if the
 *    throttle is stuck or held.
 */
#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float v_min, v_max, v_fault_low, v_fault_high, expo;
    float volts_filt;
    float value;              /* 0..1 after curve */
    uint16_t fault_ms, closed_ms;
    bool fault;
    bool armed;
} throttle_t;

void throttle_init(throttle_t *t, float v_min, float v_max, float v_fault_low,
                   float v_fault_high, float expo);
/* volts at the connector. Call at 1 kHz. Returns 0..1. */
float throttle_update(throttle_t *t, float volts);
void throttle_disarm(throttle_t *t); /* force re-arming, e.g. after a fault */

#endif
