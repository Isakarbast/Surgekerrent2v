/*
 * Parallel-form PI with clamping anti-windup:
 *     u = kp*e + integral,   integral += ki*e*dt
 * The integrator is frozen while the output is saturated in the direction
 * of the error, and is always kept inside [lo, hi].
 */
#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float integral;
} pi_ctrl_t;

void pi_init(pi_ctrl_t *pi, float kp, float ki);
void pi_reset(pi_ctrl_t *pi, float integral);
float pi_run(pi_ctrl_t *pi, float error, float dt, float lo, float hi);

#endif
