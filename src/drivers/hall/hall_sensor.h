/*
 * hall_sensor.h - 3x digital hall sensors with angle interpolation.
 *
 * Polled from the fast loop (every PWM period) with a cycle-counter
 * timestamp, so no extra IRQ and no ISR race. Between edges the angle is
 * extrapolated with the measured speed but never beyond the next sector
 * boundary, so a sudden stop can't make the angle run away.
 *
 * Sector centre angles come from motor detection (config.hall_angle_rad),
 * which makes any wiring order / sensor spacing work.
 */
#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define HALL_SPEED_EDGES 6

typedef struct {
    float angle[8];            /* centre angle per code, < 0 invalid */
    float cycles_per_s;
    uint8_t state;
    bool valid;                /* current code has a known angle */
    int8_t dir;
    float edge_angle;
    uint32_t last_edge_cycles;
    /* speed estimation over the last full electrical revolution */
    float interval_s[HALL_SPEED_EDGES];
    float interval_angle[HALL_SPEED_EDGES];
    uint8_t n_intervals, idx;
    float omega;               /* rad/s electrical, signed */
    float theta;               /* interpolated electrical angle */
    /* diagnostics */
    uint16_t invalid_code_count;
    uint16_t skip_count;       /* sector skipped: noise or failed sensor */
    uint8_t diag_div;
    bool stale;                /* timed out: next edge interval is not a speed sample */
    bool fault;
} hall_sensor_t;

void hall_init(hall_sensor_t *h, const float angle_table[8], float cycles_per_s);
void hall_update(hall_sensor_t *h, uint8_t state, uint32_t now_cycles);
/* Decrement fault counters; call at 1 kHz. */
void hall_diag_tick(hall_sensor_t *h);

#endif
