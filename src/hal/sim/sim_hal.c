#include "sim_hal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "hal.h"
#include "ring_buffer.h"

sim_params_t g_sim_p;
sim_state_t g_sim;

#define SIM_CPU_HZ   170000000.0
#define SIM_SUBSTEPS 20
#define FLASH_SLOT_SIZE 2048u

static uint8_t s_flash[HAL_FLASH_CONFIG_SLOTS][FLASH_SLOT_SIZE];
static uint8_t s_uart_rx_mem[1024], s_uart_tx_mem[4096];
static ring_buffer_t s_uart_rx, s_uart_tx;
static uint16_t s_drv_regs[16];
static bool s_wd_started;
static uint32_t s_medium_div;

void sim_reset_defaults(void)
{
    memset(&g_sim, 0, sizeof(g_sim));
    sim_params_t *p = &g_sim_p;
    p->rs = 0.10f;
    p->ld = 150e-6f;
    p->lq = 180e-6f;
    p->flux = 0.014f;
    p->pole_pairs = 15;
    p->inertia = 0.02f;
    p->friction = 0.002f;
    p->load_torque = 0.0f;
    p->hall_offset = 0.0f;
    p->vbus = 48.0f;
    p->r_batt = 0.05f;
    p->adc_noise_counts = 0.0f;
    p->throttle_v = 0.85f;
    p->brake_v = 0.85f;
    p->brake_switch = false;
    p->pas_level = false;
    p->temp_fet_c = 30.0f;
    p->temp_motor_c = 30.0f;
    p->hw_break = false;
    g_sim.duty[0] = g_sim.duty[1] = g_sim.duty[2] = 0.5f;
    memcpy(g_sim.duty_pending, g_sim.duty, sizeof(g_sim.duty));
    g_sim.vbus_actual = p->vbus;
    rb_init(&s_uart_rx, s_uart_rx_mem, sizeof(s_uart_rx_mem));
    rb_init(&s_uart_tx, s_uart_tx_mem, sizeof(s_uart_tx_mem));
    memset(s_drv_regs, 0, sizeof(s_drv_regs));
    s_medium_div = 0;
}

void sim_flash_erase_all(void) { memset(s_flash, 0xFF, sizeof(s_flash)); }
void sim_flash_corrupt(int slot, uint32_t off) { s_flash[slot][off] ^= 0x5A; }

/* ---------------- plant model ---------------- */

static void plant_step(float dt)
{
    sim_params_t *p = &g_sim_p;
    sim_state_t *s = &g_sim;
    float th = s->theta_e;
    float c = cosf(th), sn = sinf(th);
    float we = s->omega_m * (float)p->pole_pairs;
    float vbus = s->vbus_actual;

    float vd, vq;
    if (s->outputs_on || s->low_sides_on) {
        /* Dead time: during the blanking interval the diode that conducts is
         * set by current direction, so each phase loses sign(i)*t_dead*f_pwm
         * of duty - exactly what the firmware compensates. */
        const float dt_frac = BOARD_DEADTIME_NS * 1e-9f * BOARD_PWM_FREQ_HZ;
        const float iph[3] = {s->ia, s->ib, s->ic};
        float d[3];
        for (int k = 0; k < 3; k++) {
            float sgn = (iph[k] > 0.05f) ? 1.0f : ((iph[k] < -0.05f) ? -1.0f : iph[k] / 0.05f);
            d[k] = s->low_sides_on ? 0.0f : s->duty[k] - sgn * dt_frac;
        }
        float avg = (d[0] + d[1] + d[2]) / 3.0f;
        float va = (d[0] - avg) * vbus, vb = (d[1] - avg) * vbus, vc = (d[2] - avg) * vbus;
        float valpha = (2.0f * va - vb - vc) / 3.0f;
        float vbeta = (vb - vc) / sqrtf(3.0f);
        vd = c * valpha + sn * vbeta;
        vq = -sn * valpha + c * vbeta;

        float did = (vd - p->rs * s->id + we * p->lq * s->iq) / p->ld;
        float diq = (vq - p->rs * s->iq - we * p->ld * s->id - we * p->flux) / p->lq;
        s->id += did * dt;
        s->iq += diq * dt;
        s->ibatt = 1.5f * (vd * s->id + vq * s->iq) / vbus;
    } else {
        /* Bridge off: approximate diodes blocking (bemf < vbus). */
        s->id = 0.0f;
        s->iq = 0.0f;
        s->ibatt = 0.0f;
    }

    float torque = 1.5f * (float)p->pole_pairs * (p->flux * s->iq + (p->ld - p->lq) * s->id * s->iq);
    float domega = (torque - p->friction * s->omega_m - p->load_torque * (s->omega_m > 0 ? 1.0f : (s->omega_m < 0 ? -1.0f : 0.0f))) / p->inertia;
    s->omega_m += domega * dt;
    s->theta_e = fmodf(s->theta_e + we * dt, 6.2831853f);
    if (s->theta_e < 0) s->theta_e += 6.2831853f;

    float ialpha = c * s->id - sn * s->iq;
    float ibeta = sn * s->id + c * s->iq;
    s->ia = ialpha;
    s->ib = -0.5f * ialpha + 0.8660254f * ibeta;
    s->ic = -0.5f * ialpha - 0.8660254f * ibeta;

    s->vbus_actual = p->vbus - p->r_batt * s->ibatt;
}

void sim_step(void)
{
    float dt = 1.0f / BOARD_PWM_FREQ_HZ;
    for (int i = 0; i < SIM_SUBSTEPS; i++) {
        /* Sample/ISR happens at the counter peak; preload transfers at the
         * following underflow, half a period later - same as TIM1 on the MCU. */
        if (i == SIM_SUBSTEPS / 2) memcpy(g_sim.duty, g_sim.duty_pending, sizeof(g_sim.duty));
        plant_step(dt / SIM_SUBSTEPS);
    }
    g_sim.time_s += dt;
    g_sim.pwm_ticks++;
    app_fast_loop_isr();
    if (++s_medium_div >= (uint32_t)(BOARD_PWM_FREQ_HZ / BOARD_MEDIUM_LOOP_HZ)) {
        s_medium_div = 0;
        app_medium_loop_isr();
    }
}

void sim_run_seconds(float sec)
{
    uint64_t n = (uint64_t)(sec * BOARD_PWM_FREQ_HZ);
    for (uint64_t i = 0; i < n; i++) sim_step();
}

/* ---------------- HAL implementation ---------------- */

void hal_init(void)
{
    s_wd_started = false;
}
void hal_start_scheduler(void) {}
uint32_t hal_millis(void) { return (uint32_t)(g_sim.time_s * 1000.0); }
uint32_t hal_cycles(void) { return (uint32_t)(uint64_t)(g_sim.time_s * SIM_CPU_HZ); }
uint32_t hal_cycles_per_us(void) { return (uint32_t)(SIM_CPU_HZ / 1e6); }
void hal_delay_us(uint32_t us) { (void)us; }
void hal_system_reset(void) {}
hal_reset_cause_t hal_reset_cause(void) { return HAL_RESET_POWER_ON; }
uint32_t hal_irq_save(void) { return 0; }
uint32_t hal_random_u32(void) { return ((uint32_t)rand() << 16) ^ (uint32_t)rand(); }
void hal_device_uid(uint8_t uid[12])
{
    static const uint8_t k_uid[12] = {0x31, 0x00, 0x42, 0x00, 0x0D, 0x51, 0x33, 0x30, 0x35, 0x38, 0x31, 0x20};
    memcpy(uid, k_uid, 12);
}
void hal_irq_restore(uint32_t s) { (void)s; }

float hal_pwm_frequency_hz(void) { return BOARD_PWM_FREQ_HZ; }
void hal_pwm_set_duty(float a, float b, float c)
{
    g_sim.duty_pending[0] = a;
    g_sim.duty_pending[1] = b;
    g_sim.duty_pending[2] = c;
}
void hal_pwm_outputs_enable(void)
{
    if (g_sim_p.hw_break) return;
    g_sim.outputs_on = true;
    g_sim.low_sides_on = false;
}
void hal_pwm_outputs_disable(void)
{
    g_sim.outputs_on = false;
    g_sim.low_sides_on = false;
}
void hal_pwm_low_sides_on(void)
{
    g_sim.outputs_on = false;
    g_sim.low_sides_on = true;
}
bool hal_pwm_outputs_enabled(void) { return g_sim.outputs_on; }
bool hal_pwm_hw_break_tripped(void)
{
    if (g_sim_p.hw_break) g_sim.outputs_on = false;
    return g_sim_p.hw_break;
}
void hal_pwm_hw_break_clear(void) { g_sim_p.hw_break = false; }

static uint16_t to_counts(float v)
{
    float n = g_sim_p.adc_noise_counts;
    float c = v / BOARD_ADC_VREF * BOARD_ADC_MAX_COUNTS;
    if (n > 0.0f) c += n * ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f);
    if (c < 0.0f) c = 0.0f;
    if (c > BOARD_ADC_MAX_COUNTS) c = BOARD_ADC_MAX_COUNTS;
    return (uint16_t)(c + 0.5f);
}

static float csa_volts(float i)
{
    float v = i * BOARD_SHUNT_OHM * BOARD_CSA_GAIN;
#if BOARD_CSA_INVERTED
    v = -v;
#endif
    return BOARD_ADC_VREF * 0.5f + v;
}

void hal_adc_read_fast(hal_adc_fast_t *o)
{
    o->ia = to_counts(csa_volts(g_sim.ia));
    o->ib = to_counts(csa_volts(g_sim.ib));
    o->ic = to_counts(csa_volts(g_sim.ic));
    o->vbus = to_counts(g_sim.vbus_actual / BOARD_VBUS_SCALE);
}

static float ntc_volts(float t_c, float r25, float beta, float pullup)
{
    float r = r25 * expf(beta * (1.0f / (t_c + 273.15f) - 1.0f / 298.15f));
    return BOARD_ADC_VREF * r / (r + pullup);
}

uint16_t hal_adc_read_slow(hal_adc_slow_ch_t ch)
{
    switch (ch) {
    case HAL_ADC_SLOW_THROTTLE: return to_counts(g_sim_p.throttle_v * BOARD_THROTTLE_DIVIDER);
    case HAL_ADC_SLOW_BRAKE: return to_counts(g_sim_p.brake_v * BOARD_THROTTLE_DIVIDER);
    case HAL_ADC_SLOW_TEMP_FET:
        return to_counts(ntc_volts(g_sim_p.temp_fet_c, BOARD_NTC_FET_R25, BOARD_NTC_FET_BETA, BOARD_NTC_FET_PULLUP));
    case HAL_ADC_SLOW_TEMP_MOTOR:
        return to_counts(ntc_volts(g_sim_p.temp_motor_c, 10000.0f, 3950.0f, BOARD_NTC_MOTOR_PULLUP));
    case HAL_ADC_SLOW_VREF:
    default: return to_counts(1.212f);
    }
}
float hal_adc_vref(void) { return BOARD_ADC_VREF; }

uint8_t hal_hall_read(void)
{
    /* Standard 120-degree hall sequence; sector 0 centred on theta = 0. */
    static const uint8_t seq[6] = {1, 3, 2, 6, 4, 5};
    float th = g_sim.theta_e + g_sim_p.hall_offset + 3.14159265f / 6.0f;
    th = fmodf(th, 6.2831853f);
    if (th < 0) th += 6.2831853f;
    int sector = (int)(th / (6.2831853f / 6.0f)) % 6;
    return seq[sector];
}
bool hal_brake_switch_active(void) { return g_sim_p.brake_switch; }
bool hal_pas_read(void) { return g_sim_p.pas_level; }
void hal_gate_enable(bool on) { (void)on; }
bool hal_gate_fault_pin(void) { return false; }
void hal_led_set(bool on) { (void)on; }
void hal_aux_output_set(bool on) { (void)on; }

uint16_t hal_spi_transfer16(hal_spi_dev_t dev, uint16_t tx)
{
    if (dev != HAL_SPI_DEV_GATE_DRIVER) return 0;
    uint8_t addr = (uint8_t)((tx >> 11) & 0xFu);
    if (tx & 0x8000u) return s_drv_regs[addr];
    if (addr >= 2) s_drv_regs[addr] = tx & 0x7FFu & (addr == 2 ? ~1u : 0x7FFu);
    return 0;
}

size_t hal_uart_write(const uint8_t *d, size_t n)
{
    size_t i = 0;
    while (i < n && rb_put(&s_uart_tx, d[i])) i++;
    return i;
}
bool hal_uart_read_byte(uint8_t *b) { return rb_get(&s_uart_rx, b); }
void sim_uart_inject(const uint8_t *d, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) rb_put(&s_uart_rx, d[i]);
}
uint32_t sim_uart_take(uint8_t *out, uint32_t max)
{
    uint32_t n = 0;
    while (n < max && rb_get(&s_uart_tx, &out[n])) n++;
    return n;
}

void hal_can_init(uint32_t bitrate) { (void)bitrate; }
bool hal_can_send(const hal_can_frame_t *f) { (void)f; return true; }
bool hal_can_receive(hal_can_frame_t *f) { (void)f; return false; }

uint32_t hal_flash_slot_size(void) { return FLASH_SLOT_SIZE; }
const void *hal_flash_slot_ptr(int slot) { return s_flash[slot]; }
bool hal_flash_slot_erase(int slot)
{
    memset(s_flash[slot], 0xFF, FLASH_SLOT_SIZE);
    return true;
}
bool hal_flash_slot_write(int slot, uint32_t off, const void *data, uint32_t len)
{
    if (off + len > FLASH_SLOT_SIZE) return false;
    /* NOR semantics: can only clear bits. */
    const uint8_t *src = (const uint8_t *)data;
    for (uint32_t i = 0; i < len; i++) s_flash[slot][off + i] &= src[i];
    return true;
}

void hal_watchdog_start(uint32_t ms) { (void)ms; s_wd_started = true; }
void hal_watchdog_kick(void) {}
