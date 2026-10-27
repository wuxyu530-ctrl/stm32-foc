/**
 * @file    foc_current.c
 * @brief   dq 电流环一个控制周期的实现，说明见 foc_current.h
 *
 * 【Claude 代写，待复核】
 */
#include "foc_current.h"
#include "transform.h"
#include "svpwm.h"
#include <math.h>

/* v_max = (2·d_max − 1)·Vbus/√3，推导见 foc_current.h §3 */
static float calc_vmax(float vbus, float d_max)
{
    return (2.0f * d_max - 1.0f) * vbus * FOC_1_SQRT3;
}

void foc_cur_init(foc_cur_t *c, float kp, float ki, float ts, float vbus, float d_max)
{
    c->vbus  = vbus;
    c->d_max = d_max;
    c->v_max = calc_vmax(vbus, d_max);
    foc_pi_init(&c->pi_d, kp, ki, ts, c->v_max);
    foc_pi_init(&c->pi_q, kp, ki, ts, c->v_max);
}

void foc_cur_reset(foc_cur_t *c)
{
    foc_pi_reset(&c->pi_d);
    foc_pi_reset(&c->pi_q);
}

void foc_cur_set_vbus(foc_cur_t *c, float vbus)
{
    c->vbus  = vbus;
    c->v_max = calc_vmax(vbus, c->d_max);
}

void foc_cur_step(foc_cur_t *c, const foc_abc_t *i_abc, const foc_sincos_t *sc,
                  float id_ref, float iq_ref, foc_abc_t *duty, foc_cur_out_t *out)
{
    /* ① 三相电流 → dq 电流 */
    foc_ab_t i_ab;
    foc_dq_t i_dq;
    foc_clarke(i_abc, &i_ab);
    foc_park(&i_ab, sc, &i_dq);

    /* ② d 轴先算，限幅取整个 v_max */
    c->pi_d.out_max = c->v_max;
    float vd = foc_pi_update(&c->pi_d, id_ref - i_dq.d);

    /* ③ q 轴用剩下的余量：√(v_max² − vd²)。|vd| ≤ v_max 已由 PI_d 保证，
     *    这里再防一手浮点误差导致根号里出现极小的负数 */
    float rem = c->v_max * c->v_max - vd * vd;
    c->pi_q.out_max = (rem > 0.0f) ? sqrtf(rem) : 0.0f;
    float vq = foc_pi_update(&c->pi_q, iq_ref - i_dq.q);

    /* ④ dq 电压 → αβ → 三相占空比。|v| ≤ v_max 保证 SVPWM 内部不会再缩放 */
    foc_dq_t v_dq = { vd, vq };
    foc_ab_t v_ab;
    foc_inv_park(&v_dq, sc, &v_ab);
    foc_svpwm(&v_ab, c->vbus, c->d_max, duty);

    if (out) {
        out->id = i_dq.d;
        out->iq = i_dq.q;
        out->vd = vd;
        out->vq = vq;
    }
}
