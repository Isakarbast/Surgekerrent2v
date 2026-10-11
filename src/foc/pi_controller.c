#include "pi_controller.h"

#include "util_math.h"

void pi_init(pi_ctrl_t *pi, float kp, float ki)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
}

void pi_reset(pi_ctrl_t *pi, float integral) { pi->integral = integral; }

float pi_run(pi_ctrl_t *pi, float error, float dt, float lo, float hi)
{
    float p = pi->kp * error;
    float u_unclamped = p + pi->integral;

    bool sat_hi = (u_unclamped >= hi) && (error > 0.0f);
    bool sat_lo = (u_unclamped <= lo) && (error < 0.0f);
    if (!sat_hi && !sat_lo) {
        pi->integral += pi->ki * error * dt;
    }
    pi->integral = clampf(pi->integral, lo, hi);
    return clampf(p + pi->integral, lo, hi);
}
