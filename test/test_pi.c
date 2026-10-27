/**
 * @file    test_pi.c
 * @brief   离散 PI（前向欧拉 + 限幅 + 钳位抗饱和）的单元测试
 *
 * 前三组检查单步行为；最后两组把 PI 接上本项目电机的离散模型做闭环，
 * 结果必须与 Simulink sim/current_loop_dt.slx 一致。
 */
#include <math.h>
#include "test_framework.h"
#include "pi.h"

/* 本项目参数（与 sim/current_loop_design.m 一致） */
#define R_OHM   3.21f
#define L_H     0.88e-3f
#define TS      50e-6f
#define WBW     (FOC_2PI * 1000.0f)          /* 带宽 1 kHz */
#define KP      (L_H * WBW)                  /* ≈ 5.529 V/A */
#define KI      (R_OHM * WBW)                /* ≈ 20169 V/(A·s) */
#define VMAX    (12.0f * FOC_1_SQRT3)        /* ≈ 6.928 V */

/* ① 只有比例项 */
static void test_pi_p_only(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 2.0f, 0.0f, TS, 5.0f);
    CHECK_NEAR(foc_pi_update(&pi, 1.0f),  2.0f, 1e-6f, "P: 2·1");
    CHECK_NEAR(foc_pi_update(&pi, -2.0f), -4.0f, 1e-6f, "P: 2·(−2)");
    CHECK_NEAR(foc_pi_update(&pi, 10.0f), 5.0f, 1e-6f, "P: 正向限幅");
    CHECK_NEAR(foc_pi_update(&pi, -10.0f), -5.0f, 1e-6f, "P: 负向限幅");
    CHECK_NEAR(pi.integ, 0.0f, 1e-9f, "ki = 0 时积分始终为 0");
}

/* ② 只有积分项：前向欧拉，本周期输出不含本周期误差 */
static void test_pi_forward_euler(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 0.0f, 1.0f, 1.0f, 100.0f);      /* ki_ts = 1 */
    CHECK_NEAR(foc_pi_update(&pi, 1.0f), 0.0f, 1e-6f, "第 0 拍输出 0（积分还没加）");
    CHECK_NEAR(foc_pi_update(&pi, 2.0f), 1.0f, 1e-6f, "第 1 拍输出 = e0");
    CHECK_NEAR(foc_pi_update(&pi, 3.0f), 3.0f, 1e-6f, "第 2 拍输出 = e0+e1");
    CHECK_NEAR(pi.integ, 6.0f, 1e-6f, "积分 = e0+e1+e2");
    foc_pi_reset(&pi);
    CHECK_NEAR(pi.integ, 0.0f, 1e-9f, "reset 后积分清零");
    CHECK_NEAR(pi.ki_ts, 1.0f, 1e-9f, "reset 不改参数");
}

/* ③ 钳位：饱和且误差同向 → 不积分；饱和但误差反向 → 照常积分 */
static void test_pi_clamping(void)
{
    foc_pi_t pi;
    foc_pi_init(&pi, 1.0f, 1.0f, 1.0f, 5.0f);       /* kp = 1, ki_ts = 1 */

    pi.integ = 4.0f;
    CHECK_NEAR(foc_pi_update(&pi, 3.0f), 5.0f, 1e-6f, "u_raw = 3+4 = 7，饱和输出 5");
    CHECK_NEAR(pi.integ, 4.0f, 1e-6f, "正饱和且 e>0：积分冻结");

    pi.integ = 8.0f;                                  /* u_raw = −1+8 = 7 > 5，仍饱和 */
    CHECK_NEAR(foc_pi_update(&pi, -1.0f), 5.0f, 1e-6f, "正饱和输出 5");
    CHECK_NEAR(pi.integ, 7.0f, 1e-6f, "正饱和但 e<0：积分照常往回退");

    pi.integ = -4.0f;
    CHECK_NEAR(foc_pi_update(&pi, -3.0f), -5.0f, 1e-6f, "负饱和输出 −5");
    CHECK_NEAR(pi.integ, -4.0f, 1e-6f, "负饱和且 e<0：积分冻结");

    pi.integ = 1.0f;                                  /* 不饱和：正常积分 */
    CHECK_NEAR(foc_pi_update(&pi, 2.0f), 3.0f, 1e-6f, "不饱和输出 2+1");
    CHECK_NEAR(pi.integ, 3.0f, 1e-6f, "不饱和：积分照常");
}

/* 闭环：PI → 下一周期生效（单位延迟）→ 电机 1/(Ls+R) 精确离散 → 每周期采样
 * 返回周期内细分采样到的电流最大值，并输出前 n_out 拍的 PI 输出 */
static float closed_loop(float iref, int n, float *out, int n_out, float *i_final, float *u_absmax)
{
    foc_pi_t pi;
    foc_pi_init(&pi, KP, KI, TS, VMAX);
    const int SUB = 20;
    float i = 0.0f, v_applied = 0.0f, peak = 0.0f, umax = 0.0f;
    for (int k = 0; k < n; k++) {
        float u = foc_pi_update(&pi, iref - i);
        if (k < n_out) out[k] = u;
        if (fabsf(u) > umax) umax = fabsf(u);
        for (int m = 1; m <= SUB; m++) {              /* 本周期电压为上一拍的 u */
            float a = expf(-TS * (float)m / SUB * R_OHM / L_H);
            float im = i * a + (1.0f - a) * v_applied / R_OHM;
            if (im > peak) peak = im;
        }
        float a = expf(-TS * R_OHM / L_H);
        i = i * a + (1.0f - a) * v_applied / R_OHM;
        v_applied = u;
    }
    *i_final = i;
    *u_absmax = umax;
    return peak;
}

/* ④a 1 A 阶跃：不饱和 */
static void test_pi_closed_loop_1a(void)
{
    float out[3], fin, umax;
    float peak = closed_loop(1.0f, 200, out, 3, &fin, &umax);
    CHECK_NEAR(out[0], 5.529f, 2e-3f, "第 0 拍 = Kp·1");
    CHECK_NEAR(out[1], 6.538f, 2e-3f, "第 1 拍 = Kp·1 + Ki·Ts·1");
    CHECK_NEAR(out[2], 5.958f, 2e-3f, "第 2 拍");
    CHECK(peak > 1.03f && peak < 1.05f, "1 A 超调应在 3%%~5%%，实际峰值 %.4f A", peak);
    CHECK_NEAR(fin, 1.0f, 1e-3f, "无稳态误差");
}

/* ④b 2 A 阶跃：开始会饱和，钳位后不应超调 */
static void test_pi_closed_loop_2a(void)
{
    float out[6], fin, umax;
    float peak = closed_loop(2.0f, 400, out, 6, &fin, &umax);
    CHECK_NEAR(out[0], VMAX, 1e-4f, "第 0 拍饱和在 Vmax");
    CHECK(umax <= VMAX + 1e-5f, "输出从不超过 Vmax，实际最大 %.4f", umax);
    CHECK(peak < 2.002f, "钳位后无超调，实际峰值 %.4f A", peak);
    CHECK_NEAR(fin, 2.0f, 1e-3f, "无稳态误差");
}

void test_pi_all(void)
{
    RUN_TEST(test_pi_p_only);
    RUN_TEST(test_pi_forward_euler);
    RUN_TEST(test_pi_clamping);
    RUN_TEST(test_pi_closed_loop_1a);
    RUN_TEST(test_pi_closed_loop_2a);
}
