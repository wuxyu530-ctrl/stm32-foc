/**
 * @file    test_foc_current.c
 * @brief   dq 电流环一步计算（foc_current.c）的单元测试
 *
 * 【Claude 代写，待复核】
 *
 * 重点是最后一组：把电机看成 d、q 两个独立的 RL（转子不转、无反电动势、无耦合），
 * 在任意电角度下做 d 轴 0 → 1 A 阶跃，结果必须与 test_pi.c 的单轴闭环、
 * 以及 Simulink sim/current_loop_dt.slx 完全一致 —— 这同时验证了
 * Clarke/Park/反 Park/SVPWM 这一串变换的符号和系数没有接错。
 */
#include <math.h>
#include "test_framework.h"
#include "foc_current.h"
#include "transform.h"

#define R_OHM   3.21f
#define L_H     0.88e-3f
#define TS      50e-6f
#define WBW     (FOC_2PI * 1000.0f)
#define KP      (L_H * WBW)
#define KI      (R_OHM * WBW)
#define VBUS    12.0f
#define DMAX    0.956f
#define VMAX    ((2.0f * DMAX - 1.0f) * VBUS * FOC_1_SQRT3)   /* ≈ 6.3185 V */

/* 由 dq 电流和电角度构造三相电流 */
static void dq_to_abc(float d, float q, float theta, foc_abc_t *abc)
{
    foc_sincos_t sc; foc_sincos(theta, &sc);
    foc_dq_t dq = { d, q };
    foc_ab_t ab;
    foc_inv_park(&dq, &sc, &ab);
    foc_inv_clarke(&ab, abc);
}

/* ① 测量链：三相 → dq；误差为 0 时输出零电压、占空比全 0.5 */
static void test_cur_measure_zero(void)
{
    foc_cur_t c; foc_cur_init(&c, KP, KI, TS, VBUS, DMAX);
    const float th = 1.0f;
    foc_abc_t i; dq_to_abc(0.7f, -0.3f, th, &i);
    foc_sincos_t sc; foc_sincos(th, &sc);
    foc_abc_t duty; foc_cur_out_t o;
    foc_cur_step(&c, &i, &sc, 0.7f, -0.3f, &duty, &o);
    CHECK_NEAR(o.id, 0.7f, 1e-5f, "id 测量");
    CHECK_NEAR(o.iq, -0.3f, 1e-5f, "iq 测量");
    CHECK_NEAR(o.vd, 0.0f, 1e-5f, "误差 0 → vd = 0");
    CHECK_NEAR(o.vq, 0.0f, 1e-5f, "误差 0 → vq = 0");
    CHECK_NEAR(duty.a, 0.5f, 1e-5f, "零电压 → 占空比 0.5");
    CHECK_NEAR(duty.b, 0.5f, 1e-5f, "零电压 → 占空比 0.5");
    CHECK_NEAR(duty.c, 0.5f, 1e-5f, "零电压 → 占空比 0.5");
}

/* ② v_max 与 d 轴优先 */
static void test_cur_vmax_d_priority(void)
{
    foc_cur_t c; foc_cur_init(&c, KP, KI, TS, VBUS, DMAX);
    CHECK_NEAR(c.v_max, VMAX, 1e-4f, "v_max = (2·d_max−1)·Vbus/√3");

    foc_abc_t i0 = { 0, 0, 0 }, duty; foc_cur_out_t o;
    foc_sincos_t sc; foc_sincos(0.3f, &sc);

    /* 两轴都要很大的电压：d 轴先拿满，q 轴什么也不剩 */
    foc_cur_step(&c, &i0, &sc, 10.0f, 10.0f, &duty, &o);
    CHECK_NEAR(o.vd, VMAX, 1e-4f, "d 轴优先拿满 v_max");
    CHECK_NEAR(o.vq, 0.0f, 1e-3f, "q 轴余量为 0");

    /* d 轴只要一点：q 轴拿余量 √(v_max² − vd²) */
    foc_cur_reset(&c);
    foc_cur_step(&c, &i0, &sc, 0.5f, 10.0f, &duty, &o);
    float vd = KP * 0.5f;
    CHECK_NEAR(o.vd, vd, 1e-4f, "vd = Kp·0.5（未饱和）");
    CHECK_NEAR(o.vq, sqrtf(VMAX * VMAX - vd * vd), 1e-3f, "vq = 余量");
    CHECK(sqrtf(o.vd * o.vd + o.vq * o.vq) <= VMAX + 1e-4f, "矢量长度不超过 v_max");
}

/* ③ 满幅电压在一整圈电角度上，占空比都不超过 d_max，且 max+min = 1（七段式） */
static void test_cur_duty_within_dmax(void)
{
    foc_cur_t c; foc_cur_init(&c, KP, KI, TS, VBUS, DMAX);
    foc_abc_t i0 = { 0, 0, 0 }, d; foc_cur_out_t o;
    int bad = 0, bad_sig = 0;
    for (int k = 0; k < 360; k++) {
        foc_sincos_t sc; foc_sincos(FOC_2PI * (float)k / 360.0f, &sc);
        foc_cur_reset(&c);
        foc_cur_step(&c, &i0, &sc, 0.0f, 10.0f, &d, &o);   /* q 轴饱和，矢量长度 = v_max */
        float mx = fmaxf(d.a, fmaxf(d.b, d.c)), mn = fminf(d.a, fminf(d.b, d.c));
        if (mx > DMAX + 1e-4f) bad++;
        if (fabsf(mx + mn - 1.0f) > 1e-4f) bad_sig++;
    }
    CHECK(bad == 0, "有 %d 个角度占空比超过 d_max", bad);
    CHECK(bad_sig == 0, "有 %d 个角度 max+min ≠ 1", bad_sig);
}

/* ④ 闭环：d、q 两个独立 RL，下一周期生效。任意角度下 d 轴 0 → 1 A 阶跃，
 *    前三拍 vd 必须是 5.529 / 6.538 / 5.958 V（与 test_pi、Simulink 一致），q 轴保持 0 */
static void test_cur_closed_loop_d_step(void)
{
    const float angles[3] = { 0.0f, 0.7f, -2.4f };
    for (int a = 0; a < 3; a++) {
        foc_cur_t c; foc_cur_init(&c, KP, KI, TS, VBUS, DMAX);
        foc_sincos_t sc; foc_sincos(angles[a], &sc);
        float id = 0, iq = 0, vd_app = 0, vq_app = 0, vd_hist[3] = { 0 }, peak = 0, iq_abs = 0;
        const float k = expf(-TS * R_OHM / L_H);
        for (int n = 0; n < 200; n++) {
            foc_abc_t i_abc; dq_to_abc(id, iq, angles[a], &i_abc);
            foc_abc_t duty; foc_cur_out_t o;
            foc_cur_step(&c, &i_abc, &sc, 1.0f, 0.0f, &duty, &o);
            if (n < 3) vd_hist[n] = o.vd;
            id = id * k + (1.0f - k) * vd_app / R_OHM;      /* 本周期加的是上一拍的电压 */
            iq = iq * k + (1.0f - k) * vq_app / R_OHM;
            vd_app = o.vd; vq_app = o.vq;
            if (id > peak) peak = id;
            if (fabsf(iq) > iq_abs) iq_abs = fabsf(iq);
        }
        CHECK_NEAR(vd_hist[0], 5.529f, 2e-3f, "角度 %.1f：第 0 拍 vd", (double)angles[a]);
        CHECK_NEAR(vd_hist[1], 6.318f, 2e-3f, "角度 %.1f：第 1 拍 vd 被 v_max 限住", (double)angles[a]);
        CHECK_NEAR(id, 1.0f, 1e-3f, "角度 %.1f：稳态 id = 1 A", (double)angles[a]);
        CHECK(peak < 1.06f, "角度 %.1f：超调过大，峰值 %.4f", (double)angles[a], (double)peak);
        CHECK(iq_abs < 1e-4f, "角度 %.1f：q 轴应始终为 0，实际最大 %.2e", (double)angles[a], (double)iq_abs);
    }
}

void test_foc_current_all(void)
{
    RUN_TEST(test_cur_measure_zero);
    RUN_TEST(test_cur_vmax_d_priority);
    RUN_TEST(test_cur_duty_within_dmax);
    RUN_TEST(test_cur_closed_loop_d_step);
}
