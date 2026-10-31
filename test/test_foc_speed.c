/**
 * @file    test_foc_speed.c
 * @brief   速度环（foc_speed.c）的单元测试
 *
 * 【Claude 代写，待复核】
 *
 * 闭环测试里的电机模型用 M8a 实测参数（docs/M8a_q轴电流测试.md）：
 *   J·dω/dt = Kt·iq − B·ω − Tc·sign(ω)，Kt/J = 1.1e4，B/J = 0.97 /s，Tc/J = 183 rad/s²
 *   电流环看成一阶滞后，时间常数 1/(2π·1000 Hz)
 *   编码器：位置向下取整成计数，方向 −1（转速为正时计数减小），0~4095 回绕
 * 与 docs/M8b_速度环.md 里的 Python 仿真同一套模型，数值应一致。
 */
#include <math.h>
#include "test_framework.h"
#include "foc_speed.h"

#define TW      1e-3f
#define CPR     4096
#define DIR     (-1)
#define KT_J    1.1e4f
#define B_J     0.97f
#define TC_J    183.0f
#define WC      (FOC_2PI * 50.0f)
#define KP      (WC / KT_J)              /* ≈ 0.0286 A/(rad/s) */
#define KI      (KP * WC / 4.0f)         /* ≈ 2.24 A/rad */
#define IQMAX   0.4f
#define RPM     (FOC_2PI / 60.0f)        /* 1 rpm = 0.1047 rad/s */

/* ① 测速：计数差 → 角速度，方向和回绕 */
static void test_spd_measure(void)
{
    foc_spd_t s; foc_spd_init(&s, 0.0f, 0.0f, TW, IQMAX, DIR, CPR);
    const float k = FOC_2PI / (CPR * TW);                 /* 1 个计数 ≈ 1.534 rad/s */

    foc_spd_reset(&s, 10, 0.0f);
    foc_spd_step(&s, (uint32_t)(10 - 48 + CPR), 0.0f);    /* 计数减小 48，跨过 0 回绕到 4058 */
    CHECK_NEAR(s.w, 48.0f * k, 1e-3f, "计数减小 48 → ω = +48 个计数（方向 −1）");

    foc_spd_reset(&s, 4090, 0.0f);
    foc_spd_step(&s, 5, 0.0f);                            /* 计数增加 11，跨过 4095 回绕 */
    CHECK_NEAR(s.w, -11.0f * k, 1e-3f, "计数增加 11 → ω = −11 个计数");

    foc_spd_step(&s, 5, 0.0f);
    CHECK_NEAR(s.w, 0.0f, 1e-6f, "计数不变 → ω = 0");
}

/* ② 只有比例项：iq = Kp·误差，超出时限在 ±iq_max；reset 给积分初值 */
static void test_spd_p_and_reset(void)
{
    foc_spd_t s; foc_spd_init(&s, KP, 0.0f, TW, IQMAX, DIR, CPR);
    foc_spd_reset(&s, 100, 0.0f);
    CHECK_NEAR(foc_spd_step(&s, 100, 10.0f), KP * 10.0f, 1e-6f, "静止，误差 10 rad/s → iq = Kp·10");
    CHECK_NEAR(foc_spd_step(&s, 100, 100.0f), IQMAX, 1e-6f, "误差大 → 限在 +iq_max");
    CHECK_NEAR(foc_spd_step(&s, 100, -100.0f), -IQMAX, 1e-6f, "反向 → 限在 −iq_max");

    foc_spd_reset(&s, 100, 0.05f);
    CHECK_NEAR(foc_spd_step(&s, 100, 0.0f), 0.05f, 1e-6f, "reset 的积分初值直接出现在输出里");
}

/* 闭环仿真：返回真实转速峰值 [rpm]；out_iq0 = 阶跃后第一拍 iq_ref */
typedef struct { float w_peak, w_mean_end, iq_mean_end, iq_abs_max, iq0; } spd_result_t;

static spd_result_t closed_loop(float w0_rpm, float w1_rpm, float iq_init)
{
    foc_spd_t s; foc_spd_init(&s, KP, KI, TW, IQMAX, DIR, CPR);
    const int   sub = 100;                     /* 每个速度环周期内 100 个积分小步（10 µs） */
    const float dt  = TW / (float)sub, tau_i = 1.0f / (FOC_2PI * 1000.0f);
    double w = w0_rpm * RPM, iq = iq_init, pos = 0.37;   /* pos：沿 θe 正方向走过的计数（连续量） */
    spd_result_t r = { 0 };
    float  iq_ref = iq_init, w_ref = w0_rpm * RPM;
    int    n_end = 0;

    #define CNT_NOW() ((uint32_t)(((-(long)floor(pos)) % CPR + CPR) % CPR))   /* 方向 −1 */
    foc_spd_reset(&s, CNT_NOW(), iq_init);
    for (int k = -200; k < 300; k++) {
        if (k == 0) w_ref = w1_rpm * RPM;
        iq_ref = foc_spd_step(&s, CNT_NOW(), w_ref);
        if (k == 0) r.iq0 = iq_ref;
        if (k >= 0) {
            if (fabsf(iq_ref) > r.iq_abs_max) r.iq_abs_max = fabsf(iq_ref);
            if (k >= 150) { r.iq_mean_end += iq_ref; r.w_mean_end += (float)(w / RPM); n_end++; }
        }
        for (int j = 0; j < sub; j++) {
            iq += dt * (iq_ref - iq) / tau_i;
            double acc = KT_J * iq - B_J * w;
            if (w > 0.0) acc -= TC_J;
            else if (w < 0.0) acc += TC_J;
            else if (fabs(acc) <= TC_J) acc = 0.0;       /* 静止且转矩不够：不动 */
            else acc -= (acc > 0 ? TC_J : -TC_J);
            w += acc * dt;
            pos += w * dt / FOC_2PI * CPR;
            if (k >= 0 && w / RPM > r.w_peak) r.w_peak = (float)(w / RPM);
        }
    }
    #undef CNT_NOW
    r.iq_mean_end /= (float)n_end; r.w_mean_end /= (float)n_end;
    return r;
}

/* ③ 小阶跃 600 → 700 rpm（从稳态开始，积分初值 = 抵消摩擦所需的 iq）：
 *    线性区，iq 不饱和。设计值：第一拍 iq ≈ 0.32 A，峰值约 720 rpm（超调约 20%），稳态无误差 */
static void test_spd_step_600_700(void)
{
    const float iq_ss = (B_J * 600.0f * RPM + TC_J) / KT_J;   /* 600 rpm 稳态需要的 iq ≈ 0.022 A */
    spd_result_t r = closed_loop(600.0f, 700.0f, iq_ss);
    CHECK_NEAR(r.iq0, KP * 100.0f * RPM + iq_ss, 0.05f, "第一拍 iq ≈ Kp·10.47 + 摩擦（±1 个计数的量化）");
    CHECK(r.iq_abs_max <= IQMAX + 1e-6f, "iq_ref 不超过 iq_max，实际 %.3f", (double)r.iq_abs_max);
    CHECK(r.w_peak > 712.0f && r.w_peak < 730.0f, "峰值应约 720 rpm，实际 %.1f", (double)r.w_peak);
    CHECK_NEAR(r.w_mean_end, 700.0f, 1.0f, "150 ms 后平均转速 = 700 rpm（无稳态误差）");
    CHECK_NEAR(r.iq_mean_end, (B_J * 700.0f * RPM + TC_J) / KT_J, 0.004f, "稳态 iq = 抵消摩擦所需的值");
}

/* ④ 从静止启动 0 → 600 rpm（积分从 0 开始）：一开始饱和在 iq_max，钳位后超调有限 */
static void test_spd_startup(void)
{
    spd_result_t r = closed_loop(0.0f, 600.0f, 0.0f);
    CHECK_NEAR(r.iq0, IQMAX, 1e-6f, "第一拍饱和在 iq_max");
    CHECK(r.iq_abs_max <= IQMAX + 1e-6f, "iq_ref 不超过 iq_max，实际 %.3f", (double)r.iq_abs_max);
    CHECK(r.w_peak < 640.0f, "钳位后超调应小于 40 rpm，实际峰值 %.1f", (double)r.w_peak);
    CHECK_NEAR(r.w_mean_end, 600.0f, 1.0f, "最终 600 rpm");
}

void test_foc_speed_all(void)
{
    RUN_TEST(test_spd_measure);
    RUN_TEST(test_spd_p_and_reset);
    RUN_TEST(test_spd_step_600_700);
    RUN_TEST(test_spd_startup);
}
