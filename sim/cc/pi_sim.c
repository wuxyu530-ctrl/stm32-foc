/**
 * @file    pi_sim.c
 * @brief   Simulink C Caller 封装：double ↔ float，持有一个 PI 实例
 */
#include "pi_sim.h"
#include "pi.h"

static foc_pi_t s_pi;
static int      s_inited = 0;

void pi_sim_reset(void)
{
    s_inited = 0;
}

double pi_sim_step(double err, double kp, double ki, double ts, double out_max)
{
    if (!s_inited) {            /* 第一次调用时用模型参数初始化 */
        foc_pi_init(&s_pi, (float)kp, (float)ki, (float)ts, (float)out_max);
        s_inited = 1;
    }
    return (double)foc_pi_update(&s_pi, (float)err);
}
