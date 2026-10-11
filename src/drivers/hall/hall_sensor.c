#include "hall_sensor.h"

#include "util_math.h"

#define SECTOR_RAD      (M_PI_F / 3.0f)
#define HALL_TIMEOUT_S  0.1f      /* no edge for this long -> speed 0 */
#define HALL_FAULT_LIM  50u

void hall_init(hall_sensor_t *h, const float angle_table[8], float cycles_per_s)
{
    for (int i = 0; i < 8; i++) h->angle[i] = angle_table[i];
    h->cycles_per_s = cycles_per_s;
    h->state = 0xFF;
    h->valid = false;
    h->dir = 0;
    h->edge_angle = 0.0f;
    h->last_edge_cycles = 0;
    h->n_intervals = 0;
    h->idx = 0;
    h->omega = 0.0f;
    h->theta = 0.0f;
    h->invalid_code_count = 0;
    h->skip_count = 0;
    h->diag_div = 0;
    h->stale = true;
    h->fault = false;
}

static void on_edge(hall_sensor_t *h, uint8_t new_state, uint32_t now)
{
    float a_prev = h->angle[h->state & 7u];
    float a_new = h->angle[new_state];
    float diff = wrap_pi(a_new - a_prev);
    float dt = (float)(uint32_t)(now - h->last_edge_cycles) / h->cycles_per_s;

    if (h->stale) {
        /* First edge after standstill: the time since the previous edge is
         * meaningless (and the 32-bit cycle counter may have wrapped). */
        h->stale = false;
        h->n_intervals = 0;
        h->dir = (diff > 0.0f) ? 1 : -1;
        h->edge_angle = wrap_2pi(a_prev + 0.5f * diff);
        h->omega = 0.0f;
    } else if (absf(diff) > 1.5f * SECTOR_RAD || !h->valid) {
        /* Skipped a sector or came from invalid code: resync, drop history. */
        if (h->valid && h->skip_count < 0xFFFF) h->skip_count++;
        h->n_intervals = 0;
        h->omega = 0.0f;
        h->dir = 0;
        h->edge_angle = a_new;
    } else {
        int8_t dir = (diff > 0.0f) ? 1 : -1;
        if (dir != h->dir) h->n_intervals = 0; /* reversal: stale history */
        h->dir = dir;
        h->edge_angle = wrap_2pi(a_prev + 0.5f * diff);

        h->interval_s[h->idx] = dt;
        h->interval_angle[h->idx] = diff;
        h->idx = (uint8_t)((h->idx + 1u) % HALL_SPEED_EDGES);
        if (h->n_intervals < HALL_SPEED_EDGES) h->n_intervals++;

        float sum_t = 0.0f, sum_a = 0.0f;
        for (uint8_t i = 0; i < h->n_intervals; i++) {
            sum_t += h->interval_s[i];
            sum_a += h->interval_angle[i];
        }
        /* Need 2+ intervals: the first after standstill is not a speed. */
        h->omega = (h->n_intervals >= 2 && sum_t > 0.0f) ? sum_a / sum_t : 0.0f;
    }
    h->last_edge_cycles = now;
}

void hall_update(hall_sensor_t *h, uint8_t state, uint32_t now)
{
    state &= 7u;
    if (h->angle[state] < 0.0f) {
        if (h->invalid_code_count < 0xFFFF) h->invalid_code_count++;
        h->valid = false;
        h->omega = 0.0f;
        return; /* keep last theta */
    }

    if (state != h->state) {
        if (h->state != 0xFF) on_edge(h, state, now);
        else {
            h->edge_angle = h->angle[state];
            h->last_edge_cycles = now;
        }
        h->state = state;
        h->valid = true;
    }

    float since = (float)(uint32_t)(now - h->last_edge_cycles) / h->cycles_per_s;
    float expected = (h->omega != 0.0f) ? SECTOR_RAD / absf(h->omega) : 0.0f;
    if (h->omega == 0.0f || since > HALL_TIMEOUT_S || since > 2.0f * expected) {
        /* Stopped or slowing hard: sit at sector centre (max +-30 deg error). */
        h->omega = (since > HALL_TIMEOUT_S) ? 0.0f : h->omega;
        if (since > HALL_TIMEOUT_S) {
            h->n_intervals = 0;
            h->stale = true;
        }
        h->theta = h->angle[state];
        return;
    }

    float adv = h->omega * since;
    adv = clampf(adv, -SECTOR_RAD, SECTOR_RAD);
    h->theta = wrap_2pi(h->edge_angle + adv);
}

void hall_diag_tick(hall_sensor_t *h)
{
    h->fault = (h->invalid_code_count > HALL_FAULT_LIM) || (h->skip_count > HALL_FAULT_LIM);
    /* Leaky counters: sporadic glitches are tolerated, sustained ones are not. */
    if (++h->diag_div >= 100) {
        h->diag_div = 0;
        if (h->invalid_code_count) h->invalid_code_count--;
        if (h->skip_count) h->skip_count--;
    }
}
