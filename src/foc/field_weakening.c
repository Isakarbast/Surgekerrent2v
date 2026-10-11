#include "field_weakening.h"

#include "util_math.h"

void fw_init(field_weakening_t *fw, float ld, float lq, float flux,
             float fw_max, float start_mod)
{
    fw->ld = ld;
    fw->lq = lq;
    fw->flux = flux;
    fw->fw_max = fw_max;
    fw->start_mod = start_mod;
    fw->ki = fw_max * 20.0f; /* reach full FW in ~50 ms at 0.1 modulation excess */
    fw->id_fw = 0.0f;
}

float mtpa_id(float ld, float lq, float flux, float iq)
{
    float dl = lq - ld;
    /* Surface-mount (hub) motors: dl ~ 0 -> no benefit, avoid divide by ~0. */
    if (dl < 0.05f * ld) return 0.0f;
    return (flux - fast_sqrtf(flux * flux + 8.0f * dl * dl * iq * iq)) / (4.0f * dl);
}

float fw_update(field_weakening_t *fw, float iq_ref, float modulation, float dt)
{
    if (fw->fw_max > 0.0f) {
        float excess = modulation - fw->start_mod;
        fw->id_fw = clampf(fw->id_fw + fw->ki * excess * dt, 0.0f, fw->fw_max);
    } else {
        fw->id_fw = 0.0f;
    }
    float id_mtpa = mtpa_id(fw->ld, fw->lq, fw->flux, iq_ref);
    return minf(id_mtpa, -fw->id_fw);
}
