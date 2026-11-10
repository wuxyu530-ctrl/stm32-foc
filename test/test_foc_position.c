/**
 * @file    test_foc_position.c
 * @brief   位置环（foc_position.c）的单元测试
 *
 * 闭环测试：位置 P → 速度 PI（foc_speed）→ 电流环（一阶滞后）→ 机械模型（M8a 实测参数）→ 编码器量化，
 * 位置环和速度环都是 1 kHz，与 main.c 的 APP_M9_POSITION 一致。
 */
#include <math.h>
#include "test_framework.h"
#include "foc_position.h"
#include "foc_speed.h"

#define TW      1e-3f
#define CPR     4096
#define DIR     (-1)
#define KT_J    1.1e4f
#define B_J     0.97f
#define TC_J    183.0f
#define WC_S    (FOC_2PI * 50.0f)
#define KP_S    (WC_S / KT_J)
#define KI_S    (KP_S * WC_S / 4.0f)
#define IQMAX   0.4f
#define WC_P    (FOC_2PI * 10.0f)          /* 位置环穿越频率 10 Hz */
#define WMAX    (300.0f * FOC_2PI / 60.0f)  /* 转速限幅 300 rpm */
#define DEG     (FOC_PI / 180.0f)

/* ① 累计位置：方向、回绕、多圈 */
static void test_pos_measure(void)
{
    foc_pos_t p; foc_pos_init(&p, 0.0f, WMAX, DIR, CPR);
    const float k = FOC_2PI / CPR;
    foc_pos_reset(&p, 10);
    foc_pos_step(&p, (uint32_t)(10 - 100 + CPR), 0.0f);           /* 计数减小 100，跨过 0 */
    CHECK_NEAR(p.theta, 100.0f * k, 1e-5f, "计数减小 100 → θ = +100 个计数（方向 −1）");

    uint32_t c = (uint32_t)(10 - 100 + CPR);
    for (int i = 0; i < 20; i++) {                                 /* 再减小 20 × 1000 个计数 ≈ 4.9 圈 */
        c = (uint32_t)(((int32_t)c - 1000 + CPR * 10) % CPR);
        foc_pos_step(&p, c, 0.0f);
    }
    CHECK_NEAR(p.theta, 20100.0f * k, 1e-3f, "多圈累计不丢圈");
}

/* ② 纯 P 与限幅 */
static void test_pos_p_limit(void)
{
    foc_pos_t p; foc_pos_init(&p, WC_P, WMAX, DIR, CPR);
    foc_pos_reset(&p, 0);
    CHECK_NEAR(foc_pos_step(&p, 0, 0.1f), WC_P * 0.1f, 1e-4f, "小误差：ω_ref = Kp·误差");
    CHECK_NEAR(foc_pos_step(&p, 0, 2.0f), WMAX, 1e-4f, "大误差：限幅在 ω_max");
    CHECK_NEAR(foc_pos_step(&p, 0, -2.0f), -WMAX, 1e-4f, "反向大误差：限幅在 −ω_max");
    CHECK_NEAR(p.pi.integ, 0.0f, 1e-9f, "ki = 0，积分始终为 0");
}

/* 三环闭环仿真：位置目标 theta_ref，t_load 之后施加负载转矩（以等效 iq 表示） */
typedef struct { float th_end, th_peak, th_min_after, iq_abs_max, w_abs_max, t_settle; } pos_result_t;

static pos_result_t closed_loop(float theta_ref, int n_ms, int t_load, float load_iq)
{
    foc_pos_t p; foc_pos_init(&p, WC_P, WMAX, DIR, CPR);
    foc_spd_t s; foc_spd_init(&s, KP_S, KI_S, TW, IQMAX, DIR, CPR);
    const int   sub = 100;
    const float dt = TW / (float)sub, tau_i = 1.0f / (FOC_2PI * 1000.0f);
    double w = 0.0, iq = 0.0, pos = 0.37;          /* pos：沿 θe 正方向走过的计数（连续量） */
    pos_result_t r = { 0 };
    r.th_min_after = 1e9f; r.t_settle = -1.0f;

    #define CNT_NOW() ((uint32_t)(((-(long)floor(pos)) % CPR + CPR) % CPR))
    foc_pos_reset(&p, CNT_NOW());
    foc_spd_reset(&s, CNT_NOW(), 0.0f);
    for (int k = 0; k < n_ms; k++) {
        float w_ref  = foc_pos_step(&p, CNT_NOW(), theta_ref);
        float iq_ref = foc_spd_step(&s, CNT_NOW(), w_ref);
        if (fabsf(iq_ref) > r.iq_abs_max) r.iq_abs_max = fabsf(iq_ref);
        double load = (k >= t_load) ? load_iq * KT_J : 0.0;
        for (int j = 0; j < sub; j++) {
            iq += dt * (iq_ref - iq) / tau_i;
            double drive = KT_J * iq - B_J * w - load;
            double acc;
            if (w > 1e-9)       acc = drive - TC_J;
            else if (w < -1e-9) acc = drive + TC_J;
            else                acc = (fabs(drive) <= TC_J) ? 0.0 : drive - (drive > 0 ? TC_J : -TC_J);
            w += acc * dt;
            pos += w * dt / FOC_2PI * CPR;
            if (fabs(w) > r.w_abs_max) r.w_abs_max = (float)fabs(w);
        }
        float th = (float)(pos - 0.37) * FOC_2PI / CPR;            /* 真实位置 [rad] */
        if (th > r.th_peak) r.th_peak = th;
        if (k >= t_load && th < r.th_min_after) r.th_min_after = th;
        if (r.t_settle < 0.0f && fabsf(th - theta_ref) < 0.5f * DEG && k > 5) r.t_settle = (float)k;
        if (fabsf(th - theta_ref) >= 0.5f * DEG) r.t_settle = -1.0f;   /* 离开 ±0.5° 就重新计 */
        r.th_end = th;
    }
    #undef CNT_NOW
    return r;
}

/* ③ 阶跃 90°：限速 300 rpm 走过去，无明显超调，停在目标 */
static void test_pos_step_90(void)
{
    pos_result_t r = closed_loop(90.0f * DEG, 1000, 100000, 0.0f);
    CHECK_NEAR(r.th_end, 90.0f * DEG, 0.2f * DEG, "1 s 后停在 90°（误差 < 0.2°）");
    CHECK(r.th_peak < 91.5f * DEG, "超调 < 1.5°，实际峰值 %.2f°", (double)(r.th_peak / DEG));
    CHECK(r.w_abs_max < WMAX * 1.25f, "转速不应明显超过限幅，实际 %.1f rpm", (double)(r.w_abs_max * 60.0f / FOC_2PI));
    CHECK(r.iq_abs_max <= IQMAX + 1e-6f, "iq_ref 不超过 iq_max");
    CHECK(r.t_settle > 0.0f && r.t_settle < 300.0f, "300 ms 内进入 ±0.5°，实际 %.0f ms", (double)r.t_settle);
}

/* ④ 保持在 0° 时突加负载（相当于 0.05 A 的转矩）：被推开一点，再被拉回来 */
static void test_pos_hold_load(void)
{
    pos_result_t r = closed_loop(0.0f, 1500, 500, 0.05f);
    CHECK(r.th_min_after > -5.0f * DEG, "负载推开不超过 5°，实际 %.2f°", (double)(r.th_min_after / DEG));
    CHECK_NEAR(r.th_end, 0.0f, 0.2f * DEG, "1 s 后回到 0°（速度环积分抵消负载）");
}

void test_foc_position_all(void)
{
    RUN_TEST(test_pos_measure);
    RUN_TEST(test_pos_p_limit);
    RUN_TEST(test_pos_step_90);
    RUN_TEST(test_pos_hold_load);
}
