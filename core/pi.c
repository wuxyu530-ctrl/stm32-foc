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
float foc_pi_update(foc_pi_t *pi, float err)
{
    /* TODO: 按 pi.h §2、§3 实现，四步：
     *
     *   1. u_raw = 比例项 + 【当前】积分项（积分此时还不含本周期的 err）
     *   2. u = u_raw 限幅到 [−out_max, +out_max]
     *   3. 判断是否需要钳位：输出饱和 且 err 把它往更饱和的方向推
     *      不需要钳位时：integ += ki_ts * err
     *   4. 返回 u
     *
     * 写完运行  make test ，test_pi 一组全部通过即可。
     */
    (void)pi;
    (void)err;
    return 0.0f;
}
