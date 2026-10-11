#include "fault_manager.h"

#include "hal.h"

typedef struct {
    const char *name;
    bool latched;
    bool blocks_drive;
} fault_info_t;

static const fault_info_t k_fault_info[FAULT_COUNT] = {
    [FAULT_NONE]            = {"none", false, false},
    [FAULT_OVERCURRENT_SW]  = {"overcurrent_sw", true, true},
    [FAULT_OVERCURRENT_HW]  = {"overcurrent_hw", true, true},
    [FAULT_OVERVOLTAGE]     = {"overvoltage", true, true},
    [FAULT_UNDERVOLTAGE]    = {"undervoltage", false, true},
    [FAULT_FET_OVERTEMP]    = {"fet_overtemp", false, true},
    [FAULT_MOTOR_OVERTEMP]  = {"motor_overtemp", false, true},
    [FAULT_GATE_DRIVER]     = {"gate_driver", true, true},
    [FAULT_CURRENT_SENSE]   = {"current_sense", true, true},
    [FAULT_HALL]            = {"hall", true, true},
    [FAULT_ENCODER]         = {"encoder", true, true},
    [FAULT_THROTTLE]        = {"throttle", false, true},
    [FAULT_OBSERVER]        = {"observer", true, true},
    [FAULT_STALL]           = {"stall", true, true},
    [FAULT_WATCHDOG_RESET]  = {"watchdog_reset", true, false}, /* informational */
    [FAULT_LOOP_OVERRUN]    = {"loop_overrun", true, true},
    [FAULT_CONFIG]          = {"config", false, false},  /* informational: defaults / not detected */
    [FAULT_TEMP_SENSOR]     = {"temp_sensor", false, false},  /* derating handles it */
    [FAULT_DETECTION]       = {"detection", true, true},
    [FAULT_NUMERIC]         = {"numeric", true, true},
};

static volatile uint32_t s_active;
static fault_log_entry_t s_log[FAULT_LOG_LEN];
static volatile uint8_t s_log_head, s_log_count;
static fault_snapshot_fn s_snapshot;

void fault_init(void)
{
    s_active = 0;
    s_log_head = 0;
    s_log_count = 0;
}

void fault_set_snapshot_provider(fault_snapshot_fn fn) { s_snapshot = fn; }

void fault_raise(fault_id_t id)
{
    if (id <= FAULT_NONE || id >= FAULT_COUNT) return;
    uint32_t bit = 1u << id;

    uint32_t irq = hal_irq_save();
    bool is_new = (s_active & bit) == 0u;
    s_active |= bit;
    if (is_new) {
        fault_log_entry_t *e = &s_log[s_log_head];
        e->time_ms = hal_millis();
        e->id = (uint8_t)id;
        e->vbus = e->current = e->erpm = e->temp_fet = 0.0f;
        if (s_snapshot) s_snapshot(e);
        s_log_head = (uint8_t)((s_log_head + 1u) % FAULT_LOG_LEN);
        if (s_log_count < FAULT_LOG_LEN) s_log_count++;
    }
    hal_irq_restore(irq);
}

void fault_clear_auto(fault_id_t id)
{
    if (id <= FAULT_NONE || id >= FAULT_COUNT || k_fault_info[id].latched) return;
    uint32_t irq = hal_irq_save();
    s_active &= ~(1u << id);
    hal_irq_restore(irq);
}

bool fault_clear_latched(bool throttle_released)
{
    if (!throttle_released) return false;
    uint32_t mask = 0;
    for (int i = 1; i < FAULT_COUNT; i++)
        if (k_fault_info[i].latched) mask |= 1u << i;
    uint32_t irq = hal_irq_save();
    s_active &= ~mask;
    hal_irq_restore(irq);
    return true;
}

uint32_t fault_active_mask(void) { return s_active; }

bool fault_is_active(fault_id_t id) { return (s_active & (1u << id)) != 0u; }

bool fault_blocks_drive(void)
{
    uint32_t a = s_active;
    for (int i = 1; i < FAULT_COUNT; i++)
        if ((a & (1u << i)) && k_fault_info[i].blocks_drive) return true;
    return false;
}

fault_id_t fault_first_active(void)
{
    uint32_t a = s_active;
    for (int i = 1; i < FAULT_COUNT; i++)
        if (a & (1u << i)) return (fault_id_t)i;
    return FAULT_NONE;
}

const char *fault_name(fault_id_t id)
{
    return (id < FAULT_COUNT) ? k_fault_info[id].name : "?";
}

const fault_log_entry_t *fault_log(uint8_t *count)
{
    *count = s_log_count;
    return s_log; /* circular; newest at (head-1) */
}
