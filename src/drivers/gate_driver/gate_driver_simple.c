/*
 * Non-SPI gate drivers (FD6288, IR2101/2110, EG2133 ...): enable pin and an
 * optional fault line. Overcurrent protection then relies entirely on the
 * external comparator wired to the timer break input - make sure it exists.
 */
#include "board_config.h"
#include "gate_driver.h"
#include "hal.h"

static bool simple_init(void)
{
    hal_gate_enable(true);
    return true;
}

static uint32_t simple_read_faults(void)
{
    return hal_gate_fault_pin() ? GD_FAULT_PIN : GD_FAULT_NONE;
}

static void simple_clear_faults(void)
{
    /* Toggle enable to reset latched drivers. */
    hal_gate_enable(false);
    hal_delay_us(50);
    hal_gate_enable(true);
}

const gate_driver_t gate_driver_simple = {
    .init = simple_init,
    .read_faults = simple_read_faults,
    .clear_faults = simple_clear_faults,
    .enable = hal_gate_enable,
    .name = "simple",
};

const gate_driver_t *gate_driver_get(void)
{
#if BOARD_GATE_DRIVER_DRV8353
    return &gate_driver_drv8353;
#else
    return &gate_driver_simple;
#endif
}
