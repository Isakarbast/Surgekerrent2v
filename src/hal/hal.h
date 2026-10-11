/*
 * hal.h - hardware abstraction boundary.
 *
 * Everything above this layer is MCU-independent and builds on the host
 * for unit tests (see src/hal/sim). A port must implement every function
 * below and must call the two scheduler hooks:
 *
 *   app_fast_loop_isr()   - from the ADC end-of-injected-conversion IRQ,
 *                           once per PWM period (highest priority)
 *   app_medium_loop_isr() - from a 1 kHz timer IRQ (lower priority)
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------- raw ADC samples (12-bit, right aligned) ---------- */
typedef struct {
    uint16_t ia, ib, ic; /* phase current sense amp outputs */
    uint16_t vbus;
} hal_adc_fast_t;

typedef enum {
    HAL_ADC_SLOW_THROTTLE = 0,
    HAL_ADC_SLOW_BRAKE,
    HAL_ADC_SLOW_TEMP_FET,
    HAL_ADC_SLOW_TEMP_MOTOR,
    HAL_ADC_SLOW_VREF,
    HAL_ADC_SLOW_COUNT
} hal_adc_slow_ch_t;

typedef enum {
    HAL_RESET_POWER_ON = 0,
    HAL_RESET_WATCHDOG,
    HAL_RESET_SOFTWARE,
    HAL_RESET_BROWNOUT,
    HAL_RESET_PIN,
    HAL_RESET_OTHER
} hal_reset_cause_t;

/* ---------- system ---------- */
void hal_init(void);
void hal_start_scheduler(void);          /* enable PWM timer + ADC IRQs */
uint32_t hal_millis(void);
uint32_t hal_cycles(void);               /* free-running CPU cycle counter */
uint32_t hal_cycles_per_us(void);
void hal_delay_us(uint32_t us);
void hal_system_reset(void);
hal_reset_cause_t hal_reset_cause(void);
uint32_t hal_irq_save(void);
uint32_t hal_random_u32(void);           /* hardware TRNG */
void hal_device_uid(uint8_t uid[12]);    /* factory-unique chip ID */
void hal_irq_restore(uint32_t state);

/* ---------- PWM / inverter ---------- */
float hal_pwm_frequency_hz(void);
/* duty 0..1 per phase (high-side on-time fraction). */
void hal_pwm_set_duty(float da, float db, float dc);
void hal_pwm_outputs_enable(void);
/* Hi-Z all six switches immediately (coast). */
void hal_pwm_outputs_disable(void);
/* All three low sides on (active short: used for bootstrap charge / braking). */
void hal_pwm_low_sides_on(void);
bool hal_pwm_outputs_enabled(void);
/* True if the hardware break input (gate driver fault / comparator) tripped. */
bool hal_pwm_hw_break_tripped(void);
void hal_pwm_hw_break_clear(void);

/* ---------- ADC ---------- */
void hal_adc_read_fast(hal_adc_fast_t *out);   /* call from fast loop */
uint16_t hal_adc_read_slow(hal_adc_slow_ch_t ch);
float hal_adc_vref(void);                      /* ADC reference voltage [V] */

/* ---------- GPIO ---------- */
uint8_t hal_hall_read(void);                   /* bits: C<<2 | B<<1 | A */
bool hal_brake_switch_active(void);
bool hal_pas_read(void);
void hal_gate_enable(bool on);                 /* gate driver EN pin */
bool hal_gate_fault_pin(void);                 /* true if nFAULT asserted */
void hal_led_set(bool on);
void hal_aux_output_set(bool on);              /* e.g. headlight / precharge relay */

/* ---------- SPI (gate driver, magnetic encoder) ---------- */
typedef enum { HAL_SPI_DEV_GATE_DRIVER = 0, HAL_SPI_DEV_ENCODER } hal_spi_dev_t;
uint16_t hal_spi_transfer16(hal_spi_dev_t dev, uint16_t tx);

/* ---------- UART ---------- */
size_t hal_uart_write(const uint8_t *data, size_t len);
bool hal_uart_read_byte(uint8_t *byte);

/* ---------- CAN (classic 2.0B) ---------- */
typedef struct {
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} hal_can_frame_t;
void hal_can_init(uint32_t bitrate);    /* called once the config is loaded */
bool hal_can_send(const hal_can_frame_t *f);
bool hal_can_receive(hal_can_frame_t *f);

/* ---------- Flash (non-volatile config) ---------- */
#define HAL_FLASH_CONFIG_SLOTS 2
uint32_t hal_flash_slot_size(void);
const void *hal_flash_slot_ptr(int slot);
bool hal_flash_slot_erase(int slot);
bool hal_flash_slot_write(int slot, uint32_t offset, const void *data, uint32_t len);

/* ---------- watchdog ---------- */
void hal_watchdog_start(uint32_t timeout_ms);
void hal_watchdog_kick(void);

/* ---------- scheduler hooks implemented by the application ---------- */
void app_fast_loop_isr(void);
void app_medium_loop_isr(void);

#endif
