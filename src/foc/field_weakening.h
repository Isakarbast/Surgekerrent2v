/*
 * d-axis current reference generation:
 *  - MTPA for motors with saliency (Lq > Ld): negative Id adds reluctance torque.
 *  - Field weakening: when the voltage vector approaches the modulation
 *    limit, an integrator pushes Id negative to reduce back-EMF, letting the
 *    motor go faster than its base speed.
 *
 * WARNING: in deep field weakening the motor's open-circuit back-EMF exceeds
 * the battery voltage. If the controller trips at that moment, the body
 * diodes rectify that back-EMF into the bus -> large regen current and
 * overvoltage. Keep fw_max_current modest and size bus caps / TVS
 * accordingly. See docs/SAFETY.md.
 */
#ifndef FIELD_WEAKENING_H
#define FIELD_WEAKENING_H

typedef struct {
    float ld, lq, flux;
    float fw_max;         /* max |Id| from FW [A] */
    float start_mod;      /* modulation index where FW starts */
    float ki;             /* A per (modulation * s) */
    float id_fw;          /* current FW contribution (>= 0, applied negative) */
} field_weakening_t;

void fw_init(field_weakening_t *fw, float ld, float lq, float flux,
             float fw_max, float start_mod);
/* Returns id_ref (<= 0). modulation = foc_ctrl_t.modulation. */
float fw_update(field_weakening_t *fw, float iq_ref, float modulation, float dt);
float mtpa_id(float ld, float lq, float flux, float iq);

#endif
