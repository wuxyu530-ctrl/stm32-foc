/**
 * @file    foc_position.c
 * @brief   位置环实现，说明见 foc_position.h
 */
#include "foc_position.h"

void foc_pos_init(foc_pos_t *p, float kp, float w_max, int dir, int32_t cpr)
{
    foc_pi_init(&p->pi, kp, 0.0f, 1.0f, w_max);     /* ki = 0：纯 P，限幅仍由 foc_pi 负责 */
    p->cpr      = cpr;
    p->k_cnt    = (float)dir * FOC_2PI / (float)cpr;
    p->cnt_last = 0;
    p->pos_cnt  = 0;
    p->theta    = 0.0f;
}

void foc_pos_reset(foc_pos_t *p, uint32_t cnt)
{
    p->cnt_last = cnt;
    p->pos_cnt  = 0;
    p->theta    = 0.0f;
    foc_pi_reset(&p->pi);
}

float foc_pos_step(foc_pos_t *p, uint32_t cnt, float theta_ref)
{
    /* ① 累计位置：计数差，处理回绕 */
    int32_t d = (int32_t)cnt - (int32_t)p->cnt_last;
    if (d >  p->cpr / 2) d -= p->cpr;
    if (d < -p->cpr / 2) d += p->cpr;
    p->cnt_last = cnt;
    p->pos_cnt += d;
    p->theta    = p->k_cnt * (float)p->pos_cnt;   /* 用整数累计再换算，长时间运行不累积浮点误差 */

    /* ② 位置 P → ω_ref（限幅在 foc_pi_update 内部） */
    return foc_pi_update(&p->pi, theta_ref - p->theta);
}
