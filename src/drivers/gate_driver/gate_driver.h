/*
 * gate_driver.h - generic gate driver interface.
 *
 * Smart drivers (DRV835x) are configured over SPI and report detailed
 * faults; simple drivers (FD6288, IR2101, EG2133...) only have an enable
 * pin and maybe a fault pin. Both implement this interface.
 */
#ifndef GATE_DRIVER_H
#define GATE_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GD_FAULT_NONE      = 0,
    GD_FAULT_VDS_OCP   = 1u << 0,   /* FET desaturation / shoot-through */
    GD_FAULT_GDF       = 1u << 1,   /* gate drive fault */
    GD_FAULT_UVLO      = 1u << 2,   /* supply undervoltage */
    GD_FAULT_OTSD      = 1u << 3,   /* driver overtemperature shutdown */
    GD_FAULT_OTW       = 1u << 4,   /* overtemperature warning (not fatal) */
    GD_FAULT_SENSE_OC  = 1u << 5,   /* sense amplifier overcurrent */
    GD_FAULT_CP_UV     = 1u << 6,   /* charge pump undervoltage */
    GD_FAULT_COMMS     = 1u << 7,   /* SPI readback mismatch */
    GD_FAULT_PIN       = 1u << 8,   /* nFAULT asserted, cause unknown */
} gd_fault_t;

typedef struct {
    bool (*init)(void);
    uint32_t (*read_faults)(void);    /* gd_fault_t bitmask */
    void (*clear_faults)(void);
    void (*enable)(bool on);
    const char *name;
} gate_driver_t;

extern const gate_driver_t gate_driver_drv8353;
extern const gate_driver_t gate_driver_simple;

const gate_driver_t *gate_driver_get(void);

/* Faults that must stop the inverter (OTW is only a warning). */
#define GD_FAULT_FATAL_MASK (~(uint32_t)GD_FAULT_OTW)

#endif
