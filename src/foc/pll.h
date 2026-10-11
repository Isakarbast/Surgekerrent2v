/*
 * Angle-tracking PLL: turns a noisy angle measurement into a smooth angle
 * and a speed estimate. Type-2 loop (PI on phase error).
 */
#ifndef PLL_H
#define PLL_H

typedef struct {
    float kp, ki;
    float theta;   /* [0, 2pi) */
    float omega;   /* rad/s */
} pll_t;

void pll_init(pll_t *p, float kp, float ki);
void pll_reset(pll_t *p, float theta, float omega);
void pll_update(pll_t *p, float theta_meas, float dt);

#endif
