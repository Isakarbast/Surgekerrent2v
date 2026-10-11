/*
 * DRV8353RS / DRV8353RH register map (SPI variant).
 * Register layout verified against TI SLVSEX6; check your exact part
 * (DRV8350/8353, R/non-R) - some bits differ between variants.
 *
 * SPI frame: [15] R/W (1 = read) [14:11] address [10:0] data. Mode 1, 16-bit.
 */
#ifndef DRV8353_H
#define DRV8353_H

#define DRV8353_REG_FAULT1      0x00u
#define DRV8353_REG_FAULT2      0x01u
#define DRV8353_REG_DRV_CTRL    0x02u
#define DRV8353_REG_GATE_HS     0x03u
#define DRV8353_REG_GATE_LS     0x04u
#define DRV8353_REG_OCP_CTRL    0x05u
#define DRV8353_REG_CSA_CTRL    0x06u
#define DRV8353_REG_DRV_CFG     0x07u

/* FAULT1 (0x00) */
#define DRV8353_F1_FAULT        (1u << 10)
#define DRV8353_F1_VDS_OCP      (1u << 9)
#define DRV8353_F1_GDF          (1u << 8)
#define DRV8353_F1_UVLO         (1u << 7)
#define DRV8353_F1_OTSD         (1u << 6)

/* FAULT2 / VGS status (0x01) */
#define DRV8353_F2_SA_OC        (1u << 10)
#define DRV8353_F2_SB_OC        (1u << 9)
#define DRV8353_F2_SC_OC        (1u << 8)
#define DRV8353_F2_OTW          (1u << 7)
#define DRV8353_F2_GDUV         (1u << 6)

/* DRIVER CONTROL (0x02) */
#define DRV8353_DC_OCP_ACT      (1u << 10)  /* 1 = all half-bridges shut down on OCP */
#define DRV8353_DC_DIS_GDUV     (1u << 9)
#define DRV8353_DC_DIS_GDF      (1u << 8)
#define DRV8353_DC_OTW_REP      (1u << 7)
#define DRV8353_DC_PWM_MODE_6X  (0u << 5)
#define DRV8353_DC_CLR_FLT      (1u << 0)

/* GATE DRIVE HS (0x03) */
#define DRV8353_HS_LOCK_UNLOCK  (3u << 8)
#define DRV8353_HS_LOCK_LOCK    (6u << 8)

/* OCP CONTROL (0x05) */
#define DRV8353_OCP_TRETRY_50US (1u << 10)
#define DRV8353_OCP_DT_100NS    (1u << 8)
#define DRV8353_OCP_MODE_LATCH  (0u << 6)
#define DRV8353_OCP_DEG_4US     (2u << 4)

/* CSA CONTROL (0x06) */
#define DRV8353_CSA_VREF_DIV    (1u << 9)   /* bidirectional: output centred at VREF/2 */
#define DRV8353_CSA_GAIN_5      (0u << 6)
#define DRV8353_CSA_GAIN_10     (1u << 6)
#define DRV8353_CSA_GAIN_20     (2u << 6)
#define DRV8353_CSA_GAIN_40     (3u << 6)
#define DRV8353_CSA_SEN_LVL_1V  (3u << 0)

#endif
