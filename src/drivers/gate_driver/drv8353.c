#include "drv8353.h"

#include "board_config.h"
#include "gate_driver.h"
#include "hal.h"

static uint16_t drv_read(uint8_t addr)
{
    uint16_t tx = (uint16_t)(0x8000u | ((addr & 0xFu) << 11));
    return hal_spi_transfer16(HAL_SPI_DEV_GATE_DRIVER, tx) & 0x7FFu;
}

static void drv_write(uint8_t addr, uint16_t data)
{
    uint16_t tx = (uint16_t)(((addr & 0xFu) << 11) | (data & 0x7FFu));
    (void)hal_spi_transfer16(HAL_SPI_DEV_GATE_DRIVER, tx);
}

static bool drv_write_verify(uint8_t addr, uint16_t data)
{
    drv_write(addr, data);
    return drv_read(addr) == (data & 0x7FFu);
}

static uint16_t csa_gain_bits(void)
{
    const float g = BOARD_CSA_GAIN;
    if (g <= 5.0f) return DRV8353_CSA_GAIN_5;
    if (g <= 10.0f) return DRV8353_CSA_GAIN_10;
    if (g <= 20.0f) return DRV8353_CSA_GAIN_20;
    return DRV8353_CSA_GAIN_40;
}

static void drv8353_enable(bool on)
{
    hal_gate_enable(on);
    if (on) hal_delay_us(1000); /* tWAKE ~1 ms before SPI is accepted */
}

static bool drv8353_init(void)
{
    drv8353_enable(true);

    bool ok = true;
    ok &= drv_write_verify(DRV8353_REG_GATE_HS, DRV8353_HS_LOCK_UNLOCK | 0x0FFu);
    /* 6x PWM, all bridges shut down on OCP, report OTW on nFAULT. */
    ok &= drv_write_verify(DRV8353_REG_DRV_CTRL,
                           DRV8353_DC_PWM_MODE_6X | DRV8353_DC_OCP_ACT | DRV8353_DC_OTW_REP);
    /* Gate currents: max here; reduce IDRIVE if you see ringing / EMI on the switch node. */
    ok &= drv_write_verify(DRV8353_REG_GATE_LS, 0x3FFu);
    /* VDS OCP latched, 4 us deglitch, VDS level 0x9 (~0.7 V: tune to FET Rds_on * I_trip). */
    ok &= drv_write_verify(DRV8353_REG_OCP_CTRL,
                           DRV8353_OCP_MODE_LATCH | DRV8353_OCP_DEG_4US |
                           DRV8353_OCP_DT_100NS | 0x9u);
    ok &= drv_write_verify(DRV8353_REG_CSA_CTRL,
                           DRV8353_CSA_VREF_DIV | csa_gain_bits() | DRV8353_CSA_SEN_LVL_1V);
    /* Lock registers against corruption by EMI-induced SPI glitches. */
    drv_write(DRV8353_REG_GATE_HS, DRV8353_HS_LOCK_LOCK | 0x0FFu);

    drv_write(DRV8353_REG_DRV_CTRL,
              DRV8353_DC_PWM_MODE_6X | DRV8353_DC_OCP_ACT | DRV8353_DC_OTW_REP | DRV8353_DC_CLR_FLT);
    return ok;
}

static uint32_t drv8353_read_faults(void)
{
    uint32_t f = GD_FAULT_NONE;
    uint16_t f1 = drv_read(DRV8353_REG_FAULT1);
    uint16_t f2 = drv_read(DRV8353_REG_FAULT2);

    if (f1 & DRV8353_F1_VDS_OCP) f |= GD_FAULT_VDS_OCP;
    if (f1 & DRV8353_F1_GDF) f |= GD_FAULT_GDF;
    if (f1 & DRV8353_F1_UVLO) f |= GD_FAULT_UVLO;
    if (f1 & DRV8353_F1_OTSD) f |= GD_FAULT_OTSD;
    if (f2 & (DRV8353_F2_SA_OC | DRV8353_F2_SB_OC | DRV8353_F2_SC_OC)) f |= GD_FAULT_SENSE_OC;
    if (f2 & DRV8353_F2_OTW) f |= GD_FAULT_OTW;
    if (f2 & DRV8353_F2_GDUV) f |= GD_FAULT_CP_UV;

    /* A dead SPI bus reads all-ones or all-zeros; detect via a config reg. */
    uint16_t csa = drv_read(DRV8353_REG_CSA_CTRL);
    if ((csa & DRV8353_CSA_VREF_DIV) == 0u || csa == 0x7FFu) f |= GD_FAULT_COMMS;

    if (f == GD_FAULT_NONE && hal_gate_fault_pin()) f |= GD_FAULT_PIN;
    return f;
}

static void drv8353_clear_faults(void)
{
    drv_write(DRV8353_REG_DRV_CTRL,
              DRV8353_DC_PWM_MODE_6X | DRV8353_DC_OCP_ACT | DRV8353_DC_OTW_REP | DRV8353_DC_CLR_FLT);
}

const gate_driver_t gate_driver_drv8353 = {
    .init = drv8353_init,
    .read_faults = drv8353_read_faults,
    .clear_faults = drv8353_clear_faults,
    .enable = drv8353_enable,
    .name = "DRV8353",
};
