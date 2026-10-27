/**
 * @file    pi.c
 * @brief   离散 PI 控制器的实现
 *
 * 公式、钳位规则、验收标准见 pi.h，实现前请先通读。
 */
#include "pi.h"

/* ------------------------------------------------------------------------ */
void foc_pi_init(foc_pi_t *pi, float kp, float ki, float ts, float out_max)
{
    pi->kp      = kp;
    pi->ki_ts   = ki * ts;
    pi->out_max = out_max;
    pi->integ   = 0.0f;
}

/* ------------------------------------------------------------------------ */
void foc_pi_reset(foc_pi_t *pi)
{
    pi->integ = 0.0f;
}

/* ------------------------------------------------------------------------ */
/* 【Claude 代写，待复核】见 docs/理解清单.md "PI 代码复核"一条 */
float foc_pi_update(foc_pi_t *pi, float err)
{
    /* 1. 未限幅输出：比例项 + 【上一周期为止】累加的积分项（前向欧拉） */
    float u_raw = pi->kp * err + pi->integ;

    /* 2. 输出限幅到 [−out_max, +out_max] */
    float u = u_raw;
    if (u > pi->out_max) {
        u = pi->out_max;
    } else if (u < -pi->out_max) {
        u = -pi->out_max;
    }

    /* 3. 钳位抗饱和：输出已饱和，且误差还在往更饱和的方向推 → 本周期不积分。
     *    饱和但误差已反向时照常积分，积分才能往回退、帮助退出饱和。 */
    bool push_further = (u_raw >  pi->out_max && err > 0.0f) ||
                        (u_raw < -pi->out_max && err < 0.0f);
    if (!push_further) {
        pi->integ += pi->ki_ts * err;
    }

    /* 4. 返回限幅后的输出 */
    return u;
}
