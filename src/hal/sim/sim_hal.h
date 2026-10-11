/*
 * sim_hal.h - host-side HAL with a PMSM + inverter model.
 *
 * Lets the complete firmware (fast loop, medium loop, detection, protocol)
 * run on a PC for unit/system tests and for tuning experiments without
 * risking hardware. The model is ideal-ish: no dead time, no ADC noise
 * unless enabled, diode conduction approximated as zero current when the
 * bridge is off (valid while back-EMF < vbus).
 */
#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* motor */
    float rs, ld, lq, flux;
    int pole_pairs;
    float inertia;        /* kg m^2 (rider+vehicle reflected to the wheel for vehicle tests) */
    float friction;       /* N m s/rad */
    float load_torque;    /* N m, constant */
    float hall_offset;    /* rad, physical sensor misalignment */
    /* supply */
    float vbus;
    float r_batt;         /* pack internal resistance */
    /* sensors */
    float adc_noise_counts;
    float throttle_v, brake_v;
    bool brake_switch;
    bool pas_level;
    float temp_fet_c, temp_motor_c;
    bool hw_break;
} sim_params_t;

typedef struct {
    float theta_e, omega_m;   /* true rotor state */
    float id, iq;             /* true currents */
    float ia, ib, ic;
    float duty[3];            /* active */
    float duty_pending[3];    /* written by firmware, loaded at mid-period (underflow) */
    bool outputs_on;
    bool low_sides_on;
    double time_s;
    uint64_t pwm_ticks;
    float vbus_actual;
    float ibatt;
} sim_state_t;

extern sim_params_t g_sim_p;
extern sim_state_t g_sim;

void sim_reset_defaults(void);
/* Advance one PWM period and run the ISRs (fast every period, medium at 1 kHz). */
void sim_step(void);
void sim_run_seconds(float s);
/* UART loopback helpers */
void sim_uart_inject(const uint8_t *data, uint32_t len);
uint32_t sim_uart_take(uint8_t *out, uint32_t max);
void sim_flash_corrupt(int slot, uint32_t offset);
void sim_flash_erase_all(void);

#endif
