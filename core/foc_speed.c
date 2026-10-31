/**
 * @file    foc_speed.c
 * @brief   速度环实现，说明见 foc_speed.h
 *
 * 【Claude 代写，待复核】
 */
#include "foc_speed.h"

void foc_spd_init(foc_spd_t *s, float kp, float ki, float ts, float iq_max, int dir, int32_t cpr)
{
    foc_pi_init(&s->pi, kp, ki, ts, iq_max);
    s->cpr      = cpr;
    s->k_cnt    = (float)dir * FOC_2PI / ((float)cpr * ts);
    s->cnt_last = 0;
    s->w        = 0.0f;
}

void foc_spd_reset(foc_spd_t *s, uint32_t cnt, float iq_init)
{
    s->cnt_last = cnt;
    s->w        = 0.0f;
    s->pi.integ = iq_init;
}

float foc_spd_step(foc_spd_t *s, uint32_t cnt, float w_ref)
{
    /* ① 测速：计数差，处理回绕 */
    int32_t d = (int32_t)cnt - (int32_t)s->cnt_last;
    if (d >  s->cpr / 2) d -= s->cpr;
    if (d < -s->cpr / 2) d += s->cpr;
    s->cnt_last = cnt;
    s->w = s->k_cnt * (float)d;

    /* ② 速度 PI → iq_ref（限幅 + 钳位在 foc_pi_update 内部） */
    return foc_pi_update(&s->pi, w_ref - s->w);
}
