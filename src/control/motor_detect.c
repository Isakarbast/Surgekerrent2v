#include "motor_detect.h"

#include <string.h>

#include "util_math.h"

#define SEC(d, s) ((uint32_t)((s) / (d)->dt))

static void enter(motor_detect_t *d, detect_state_t s)
{
    d->state = s;
    d->t = 0;
    d->acc_v = d->acc_i = d->acc_n = 0.0;
    d->acc_vd = d->acc_vq = d->acc_id = d->acc_iq = 0.0;
}

static void fail(motor_detect_t *d, detect_error_t e)
{
    d->error = e;
    enter(d, DET_ERROR);
}

void detect_start(motor_detect_t *d, float i_test, float dt, uint8_t pole_pairs,
                  bool do_hall, bool do_encoder)
{
    memset(d, 0, sizeof(*d));
    d->dt = dt;
    d->i_test = i_test;
    d->pole_pairs = pole_pairs;
    d->do_hall = do_hall;
    d->do_encoder = do_encoder;
    d->omega_flux = 2000.0f / 60.0f * M_2PI_F;   /* 2000 eRPM */
    for (int i = 0; i < 8; i++) d->hall_angle[i] = -1.0f;
    enter(d, DET_ALIGN);
}

void detect_abort(motor_detect_t *d)
{
    if (d->state != DET_DONE && d->state != DET_IDLE) fail(d, DET_ERR_ABORTED);
}

static void cmd_current(detect_cmd_t *c, float theta, float omega, float id)
{
    c->current_mode = true;
    c->theta = theta;
    c->omega = omega;
    c->i_ref.d = id;
    c->i_ref.q = 0.0f;
}

bool detect_step(motor_detect_t *d, const detect_in_t *in, detect_cmd_t *c)
{
    d->t++;
    switch (d->state) {
    case DET_ALIGN:
        /* Ramp up so the rotor snaps gently rather than slamming. */
        cmd_current(c, 0.0f, 0.0f, d->i_test * minf(1.0f, (float)d->t / (float)SEC(d, 0.5f)));
        if (d->t > SEC(d, 1.0f)) enter(d, DET_RESIST_1);
        break;

    case DET_RESIST_1:
    case DET_RESIST_2: {
        float i_set = (d->state == DET_RESIST_1) ? 0.5f * d->i_test : d->i_test;
        cmd_current(c, 0.0f, 0.0f, i_set);
        if (d->t > SEC(d, 0.3f)) { /* settled: accumulate */
            d->acc_v += in->v_dq.d;
            d->acc_i += in->i_dq.d;
            d->acc_n += 1.0;
        }
        if (d->t > SEC(d, 0.8f)) {
            float v = (float)(d->acc_v / d->acc_n);
            float i = (float)(d->acc_i / d->acc_n);
            if (d->state == DET_RESIST_1) {
                d->v1 = v;
                d->i1 = i;
                enter(d, DET_RESIST_2);
            } else {
                float di = i - d->i1;
                if (absf(di) < 0.1f * d->i_test) { fail(d, DET_ERR_R_RANGE); break; }
                d->rs = (v - d->v1) / di;
                if (d->rs < 0.001f || d->rs > 5.0f) { fail(d, DET_ERR_R_RANGE); break; }
                d->v1 = v; /* holding voltage for inductance step */
                d->v_inj = 0.2f;
                enter(d, DET_IND_D);
            }
        }
        break;
    }

    case DET_IND_D:
    case DET_IND_Q: {
        /* Hold DC on d (keeps rotor locked) and add a +-v_inj square wave on
         * d or q that toggles every 2 periods. The PWM hardware applies a new
         * duty with some delay (half a period on STM32 TIM1), so an interval
         * between two samples only sees a clean +-v_inj if the last two
         * commands were equal - only those intervals are used, which makes
         * the result independent of the PWM update timing. */
        bool on_d = (d->state == DET_IND_D);
        float sign = ((d->t >> 1) & 1u) ? 1.0f : -1.0f;
        c->current_mode = false;
        c->theta = 0.0f;
        c->omega = 0.0f;
        c->v.d = d->v1 + (on_d ? sign * d->v_inj : 0.0f);
        c->v.q = on_d ? 0.0f : sign * d->v_inj;

        float i_now = on_d ? in->i_dq.d : in->i_dq.q;
        float di = absf(i_now - d->prev_i);
        d->prev_i = i_now;
        /* Commands of steps t-1 and t-2 produced this interval. */
        bool clean = (((d->t - 1u) >> 1) & 1u) == (((d->t - 2u) >> 1) & 1u);

        if (d->t < 4u || !clean) {
            /* skip */
        } else if (d->t < SEC(d, 0.2f)) {
            /* Auto-scale injection until ripple ~15 % of test current. */
            lpf_update(&d->di_acc, di, 0.05f);
            if (d->di_acc < 0.15f * d->i_test && d->v_inj < 0.25f * in->vbus)
                d->v_inj *= 1.004f;
        } else {
            d->acc_i += di;
            d->acc_n += 1.0;
        }
        if (d->t > SEC(d, 0.6f)) {
            float di_avg = (float)(d->acc_i / d->acc_n);
            if (di_avg < 1e-3f) { fail(d, DET_ERR_L_RANGE); break; }
            float l = d->v_inj * d->dt / di_avg;
            if (l < 1e-6f || l > 0.05f) { fail(d, DET_ERR_L_RANGE); break; }
            if (on_d) {
                d->ld = l;
                enter(d, DET_IND_Q);
            } else {
                d->lq = l;
                enter(d, d->do_encoder ? DET_ENC_OFFSET : (d->do_hall ? DET_HALL_FWD : DET_FLUX_RAMP));
                d->theta = 0.0f;
                d->omega = 0.0f;
            }
            d->prev_i = 0.0f;
            d->di_acc = 0.0f;
        }
        break;
    }

    case DET_ENC_OFFSET:
        cmd_current(c, 0.0f, 0.0f, d->i_test);
        if (d->t > SEC(d, 0.3f) && in->enc_mech >= 0.0f) {
            d->enc_offset = wrap_2pi(-in->enc_mech * (float)d->pole_pairs);
            enter(d, d->do_hall ? DET_HALL_FWD : DET_FLUX_RAMP);
        }
        break;

    case DET_HALL_FWD:
    case DET_HALL_REV: {
        /* 1 Hz electrical, 3 turns each direction. */
        float w = M_2PI_F * ((d->state == DET_HALL_FWD) ? 1.0f : -1.0f);
        d->theta = wrap_2pi(d->theta + w * d->dt);
        cmd_current(c, d->theta, 0.0f, d->i_test);
        if (d->t > SEC(d, 1.0f)) { /* skip first turn: rotor catching up */
            uint8_t h = in->hall_code & 7u;
            float s, co;
            fast_sincosf(d->theta, &s, &co);
            d->hall_s[h] += s;
            d->hall_c[h] += co;
            d->hall_n[h]++;
        }
        if (d->t > SEC(d, 3.0f)) {
            if (d->state == DET_HALL_FWD) {
                enter(d, DET_HALL_REV);
            } else {
                int valid = 0;
                for (int h = 0; h < 8; h++) {
                    if (d->hall_n[h] > SEC(d, 0.05f)) {
                        d->hall_angle[h] = wrap_2pi(fast_atan2f(d->hall_s[h], d->hall_c[h]));
                        valid++;
                    }
                }
                /* Exactly six codes, and never 0b000 / 0b111. */
                if (valid != 6 || d->hall_angle[0] >= 0.0f || d->hall_angle[7] >= 0.0f) {
                    fail(d, DET_ERR_HALL_CODES);
                    break;
                }
                enter(d, DET_FLUX_RAMP);
            }
        }
        break;
    }

    case DET_FLUX_RAMP:
        d->omega = minf(d->omega + d->omega_flux / 3.0f * d->dt, d->omega_flux); /* 3 s ramp */
        d->theta = wrap_2pi(d->theta + d->omega * d->dt);
        cmd_current(c, d->theta, d->omega, d->i_test);
        if (d->omega >= d->omega_flux) enter(d, DET_FLUX_MEASURE);
        break;

    case DET_FLUX_MEASURE:
        d->theta = wrap_2pi(d->theta + d->omega * d->dt);
        cmd_current(c, d->theta, d->omega, d->i_test);
        if (d->t > SEC(d, 0.5f)) {
            d->acc_vd += in->v_dq.d;
            d->acc_vq += in->v_dq.q;
            d->acc_id += in->i_dq.d;
            d->acc_iq += in->i_dq.q;
            d->acc_n += 1.0;
        }
        if (d->t > SEC(d, 1.5f)) {
            float n = (float)d->acc_n;
            float vd = (float)d->acc_vd / n, vq = (float)d->acc_vq / n;
            float id = (float)d->acc_id / n, iq = (float)d->acc_iq / n;
            float l = 0.5f * (d->ld + d->lq);
            float w = d->omega;
            float ed = vd - d->rs * id + w * l * iq;
            float eq = vq - d->rs * iq - w * l * id;
            d->flux = fast_sqrtf(ed * ed + eq * eq) / w;
            if (d->flux < 1e-4f || d->flux > 0.5f) { fail(d, DET_ERR_FLUX_RANGE); break; }
            enter(d, DET_FLUX_RAMP_DOWN);
        }
        break;

    case DET_FLUX_RAMP_DOWN:
        d->omega = maxf(d->omega - d->omega_flux / 2.0f * d->dt, 0.0f);
        d->theta = wrap_2pi(d->theta + d->omega * d->dt);
        cmd_current(c, d->theta, d->omega, d->i_test * 0.5f);
        if (d->omega <= 0.0f) enter(d, DET_DONE);
        break;

    case DET_DONE:
    case DET_ERROR:
    case DET_IDLE:
    default:
        c->current_mode = false;
        c->v.d = c->v.q = 0.0f;
        return false;
    }
    return true;
}

const char *detect_state_name(detect_state_t s)
{
    static const char *names[] = {
        "idle", "align", "resist_1", "resist_2", "ind_d", "ind_q", "enc_offset",
        "hall_fwd", "hall_rev", "flux_ramp", "flux_measure", "flux_ramp_down",
        "done", "error",
    };
    return (s <= DET_ERROR) ? names[s] : "?";
}
