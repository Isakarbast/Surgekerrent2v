/*
 * fault_manager.h - central fault bookkeeping.
 *
 * Every protection raises a fault here; the motor controller consults
 * fault_blocks_drive() before enabling PWM. Faults are either
 *   LATCHED   - stay until explicitly cleared (UART/CAN/power-cycle) AND
 *               the throttle has been released, or
 *   AUTO      - clear themselves when the condition goes away
 *               (e.g. undervoltage recovers with hysteresis).
 * fault_raise() is ISR-safe and may be called from any context.
 */
#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT_SW,      /* phase current above abs_current_max */
    FAULT_OVERCURRENT_HW,      /* timer break input (comparator / driver OCP) */
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_FET_OVERTEMP,
    FAULT_MOTOR_OVERTEMP,
    FAULT_GATE_DRIVER,
    FAULT_CURRENT_SENSE,
    FAULT_HALL,
    FAULT_ENCODER,
    FAULT_THROTTLE,
    FAULT_OBSERVER,            /* estimated flux implausible (wrong params / lost sync) */
    FAULT_STALL,               /* high current, no rotation, too long */
    FAULT_WATCHDOG_RESET,      /* previous boot ended in a watchdog reset */
    FAULT_LOOP_OVERRUN,        /* fast loop missed its deadline */
    FAULT_CONFIG,              /* config invalid / not detected */
    FAULT_TEMP_SENSOR,         /* NTC open/short */
    FAULT_DETECTION,           /* motor detection failed */
    FAULT_NUMERIC,             /* NaN/Inf reached the control path (should be impossible) */
    FAULT_COUNT
} fault_id_t;

#define FAULT_LOG_LEN 16

typedef struct {
    uint32_t time_ms;
    uint8_t id;
    float vbus;
    float current;
    float erpm;
    float temp_fet;
} fault_log_entry_t;

void fault_init(void);
void fault_raise(fault_id_t id);
/* Clear an AUTO fault whose condition is gone. No effect on latched ones. */
void fault_clear_auto(fault_id_t id);
/* Request to clear all latched faults; honoured only if throttle released. */
bool fault_clear_latched(bool throttle_released);
uint32_t fault_active_mask(void);
bool fault_is_active(fault_id_t id);
bool fault_blocks_drive(void);
fault_id_t fault_first_active(void);
const char *fault_name(fault_id_t id);
const fault_log_entry_t *fault_log(uint8_t *count);

/* Snapshot provider set by the application for the fault log. */
typedef void (*fault_snapshot_fn)(fault_log_entry_t *e);
void fault_set_snapshot_provider(fault_snapshot_fn fn);

#endif
