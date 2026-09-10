/**
 * @file    test_svpwm.c
 * @brief   七段式 SVPWM 的单元测试
 *
 * 最强的一条是【反算验证】：从占空比推回逆变器实际输出的电压矢量，
 * 必须等于输入的期望矢量。它不依赖任何"标准答案"，
 * 扇区判断错、作用时间错、零矢量分配错 —— 全都逃不掉。
 */
#include <math.h>
#include "test_framework.h"
#include "svpwm.h"
#include "transform.h"          /* 反算时复用【已验证过】的 Clarke */

#define NPTS    720             /* 一整圈，每 0.5° 一个点 */
#define TOL     1e-4f
#define VDC     12.0f
#define ULIM    (VDC * FOC_1_SQRT3)     /* 线性区上限 |U|max = Vdc/√3 */

/* 从占空比反算逆变器实际输出的 αβ 电压矢量。
 * 三相负载中性点浮空 → 相电压 = Vdc·(d_x − 三相占空比平均值) */
static void duty_to_uab(const foc_abc_t *d, float vdc, foc_ab_t *out)
{
    float avg = (d->a + d->b + d->c) * FOC_1_3;
    foc_abc_t uph = { vdc * (d->a - avg),
                      vdc * (d->b - avg),
                      vdc * (d->c - avg) };
    foc_clarke(&uph, out);
}

/* ======================================================================== */
/* ① 反算验证 —— 最强的判据                                                 */
/* ======================================================================== */
static void test_svpwm_reconstruct(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;

        for (int mi = 1; mi <= 4; mi++) {          /* 几个调制比 */
            float mag = ULIM * (float)mi / 5.0f;   /* 0.2~0.8 倍线性区上限 */
            foc_ab_t  u = { mag * cosf(th), mag * sinf(th) };
            foc_abc_t d;
            foc_ab_t  back;

            bool sat = foc_svpwm(&u, VDC, 1.0f, &d);

            CHECK(sat == false,
                  "θ=%.4f mag=%.3f: 线性区内不该限幅", th, mag);

            duty_to_uab(&d, VDC, &back);
            CHECK_NEAR(back.alpha, u.alpha, TOL,
                       "θ=%.4f mag=%.3f: α 反算不上", th, mag);
            CHECK_NEAR(back.beta,  u.beta,  TOL,
                       "θ=%.4f mag=%.3f: β 反算不上", th, mag);
        }
    }
}

/* ======================================================================== */
/* ② 七段式签名：max + min ≡ 1                                              */
/* ======================================================================== */
static void test_svpwm_max_plus_min_is_one(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th  = (float)k * FOC_2PI / (float)NPTS;
        float mag = ULIM * 0.6f;
        foc_ab_t  u = { mag * cosf(th), mag * sinf(th) };
        foc_abc_t d;

        foc_svpwm(&u, VDC, 1.0f, &d);

        float mx = fmaxf(d.a, fmaxf(d.b, d.c));
        float mn = fminf(d.a, fminf(d.b, d.c));

        CHECK_NEAR(mx + mn, 1.0f, TOL,
                   "θ=%.4f: 零矢量未均分  max=%.4f min=%.4f", th, mx, mn);
    }
}

/* ======================================================================== */
/* ③ 占空比必须在 [0, 1] 内                                                 */
/* ======================================================================== */
static void test_svpwm_duty_in_range(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th  = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t  u = { ULIM * cosf(th), ULIM * sinf(th) };   /* 顶到线性区边界 */
        foc_abc_t d;

        foc_svpwm(&u, VDC, 1.0f, &d);

        CHECK(d.a >= -TOL && d.a <= 1.0f + TOL, "θ=%.4f da 越界: %f", th, d.a);
        CHECK(d.b >= -TOL && d.b <= 1.0f + TOL, "θ=%.4f db 越界: %f", th, d.b);
        CHECK(d.c >= -TOL && d.c <= 1.0f + TOL, "θ=%.4f dc 越界: %f", th, d.c);
    }
}

/* ======================================================================== */
/* ④ 零输入 → 三相都是 0.5（纯零矢量）                                       */
/* ======================================================================== */
static void test_svpwm_zero_input(void)
{
    foc_ab_t  u = { 0.0f, 0.0f };
    foc_abc_t d;

    bool sat = foc_svpwm(&u, VDC, 1.0f, &d);

    CHECK(sat == false, "零输入不该限幅");
    CHECK_NEAR(d.a, 0.5f, TOL, "零输入时 da 应为 0.5");
    CHECK_NEAR(d.b, 0.5f, TOL, "零输入时 db 应为 0.5");
    CHECK_NEAR(d.c, 0.5f, TOL, "零输入时 dc 应为 0.5");
}

/* ======================================================================== */
/* ⑤ 线性区边界：|U| = Vdc/√3 时，max(d) 恰好摸到 1.0                        */
/* ======================================================================== */
static void test_svpwm_linear_boundary(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t  u = { ULIM * cosf(th), ULIM * sinf(th) };
        foc_abc_t d;

        foc_svpwm(&u, VDC, 1.0f, &d);

        float mx = fmaxf(d.a, fmaxf(d.b, d.c));
        CHECK(mx <= 1.0f + TOL,
              "θ=%.4f: 线性区边界上 max(d) 不该超过 1.0，实得 %f", th, mx);
    }
}

/* ======================================================================== */
/* ⑥ 限幅行为：超出 d_max 时必须缩放，且缩放后 max(d) 恰好等于 d_max         */
/* ======================================================================== */
static void test_svpwm_dmax_clamp(void)
{
    /* DMAX 必须选在 0.93301 以下，否则本测试会误判。
     *
     * 原因：内切圆 |U| = Vdc/√3 上，max(d) 并非常数，而是随扇区内角度 θ 摆动
     *       max(d) = 0.5 + (sin(60°-θ) + sin θ) / 2
     *         θ = 30°（扇区正中，内切圆切点）  → max(d) = 1.00000
     *         θ =  0°/60°（扇区边界方向）      → max(d) = 0.93301
     *       即 max(d) ∈ [0.93301, 1.0]。
     *
     * 换句话说 max(d) <= d_max 划出的是一个"缩小的六边形"而不是圆：
     * 若取 DMAX = 0.94，靠近扇区边界的那几段圆弧本来就落在六边形内部，
     * 不该限幅，foc_svpwm 会如实返回 false —— 那是正确行为，不是 bug。
     * 取 0.90 才能保证整圈每个角度都必然越界，从而真正验到限幅逻辑。 */
    const float DMAX = 0.90f;

    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t  u = { ULIM * cosf(th), ULIM * sinf(th) };   /* 必然超 0.90 */
        foc_abc_t d;

        bool sat = foc_svpwm(&u, VDC, DMAX, &d);

        CHECK(sat == true, "θ=%.4f: 顶到线性区边界时应报告限幅", th);

        float mx = fmaxf(d.a, fmaxf(d.b, d.c));
        float mn = fminf(d.a, fminf(d.b, d.c));

        CHECK_NEAR(mx, DMAX, TOL,
                   "θ=%.4f: 限幅后 max(d) 应恰好等于 d_max", th);
        CHECK_NEAR(mx + mn, 1.0f, TOL,
                   "θ=%.4f: 限幅后 max+min 仍应为 1", th);
    }
}

/* ======================================================================== */
/* ⑦ ★ 限幅只能减小幅值，绝不能改变方向（否则引入相位失真）                  */
/* ======================================================================== */
static void test_svpwm_clamp_preserves_direction(void)
{
    const float DMAX = 0.90f;

    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t  u = { ULIM * cosf(th), ULIM * sinf(th) };
        foc_abc_t d;
        foc_ab_t  back;

        foc_svpwm(&u, VDC, DMAX, &d);
        duty_to_uab(&d, VDC, &back);

        /* 叉积为 0 ⟺ 两矢量共线 */
        float cross = u.alpha * back.beta - u.beta * back.alpha;
        CHECK_NEAR(cross, 0.0f, 1e-3f,
                   "θ=%.4f: 限幅改变了矢量方向（叉积应为 0）", th);

        /* 点积为正 ⟺ 同向而非反向 */
        float dot = u.alpha * back.alpha + u.beta * back.beta;
        CHECK(dot > 0.0f, "θ=%.4f: 限幅后矢量反向了", th);

        /* 幅值应变小 */
        float m_in  = sqrtf(u.alpha * u.alpha + u.beta * u.beta);
        float m_out = sqrtf(back.alpha * back.alpha + back.beta * back.beta);
        CHECK(m_out < m_in + TOL, "θ=%.4f: 限幅后幅值反而变大了", th);
    }
}

/* ======================================================================== */
/* ⑧ 扇区判断：每 60° 一个扇区，顺序 1→2→3→4→5→6                            */
/* ======================================================================== */
static void test_svpwm_sector(void)
{
    for (int k = 0; k < NPTS; k++) {
        /* 取每个 60° 区间的中点附近，避开边界的浮点歧义 */
        float th = ((float)k + 0.5f) * FOC_2PI / (float)NPTS;
        foc_ab_t u = { cosf(th), sinf(th) };

        uint8_t expect = (uint8_t)(th / FOC_PI_3) + 1u;
        if (expect > 6u) expect = 6u;

        uint8_t got = foc_svpwm_sector(&u);
        CHECK(got == expect,
              "θ=%.2f° 应在扇区 %u，实得 %u",
              th * 180.0f / FOC_PI, expect, got);
    }
}


/* ======================================================================== */
void test_svpwm_all(void)
{
    RUN_TEST(test_svpwm_reconstruct);
    RUN_TEST(test_svpwm_max_plus_min_is_one);
    RUN_TEST(test_svpwm_duty_in_range);
    RUN_TEST(test_svpwm_zero_input);
    RUN_TEST(test_svpwm_linear_boundary);
    RUN_TEST(test_svpwm_dmax_clamp);
    RUN_TEST(test_svpwm_clamp_preserves_direction);
    RUN_TEST(test_svpwm_sector);
}
