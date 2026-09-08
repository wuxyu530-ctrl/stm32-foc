/**
 * @file    test_transform.c
 * @brief   Clarke / Park 变换的单元测试
 *
 * 判据分三类，从弱到强：
 *   ① 已知点     —— 手算特殊输入的输出
 *   ② 数学性质   —— 不依赖具体数值，覆盖无穷多输入（主力）
 *   ③ 往返一致   —— 正变换接逆变换必须回到原点
 */
#include <math.h>
#include "test_framework.h"
#include "transform.h"

#define NPTS   720          /* 遍历一整圈，每 0.5° 一个点 */
#define TOL    1e-5f

/* ======================================================================== */
/* ① Clarke：幅值不变约定的定义性质                                          */
/*    输入三相平衡正弦 → 输出 α=M·cosθ, β=M·sinθ                            */
/* ======================================================================== */
static void test_clarke_amplitude_invariant(void)
{
    const float M = 3.7f;                    /* 任取一个幅值 */

    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;

        foc_abc_t in = {
            .a = M * cosf(th),
            .b = M * cosf(th - FOC_2PI / 3.0f),
            .c = M * cosf(th + FOC_2PI / 3.0f),
        };
        foc_ab_t out;
        foc_clarke(&in, &out);

        CHECK_NEAR(out.alpha, M * cosf(th), TOL,
                   "θ=%.4f rad: α 应为 M·cosθ", th);
        CHECK_NEAR(out.beta,  M * sinf(th), TOL,
                   "θ=%.4f rad: β 应为 M·sinθ", th);
    }
}

/* ======================================================================== */
/* ② Clarke ↔ 逆Clarke 往返                                                 */
/*    注意：只有零序分量为 0 的输入才可能往返成功，                          */
/*    所以这里从 αβ 出发（逆→正），而不是从 abc 出发                        */
/* ======================================================================== */
static void test_clarke_roundtrip(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t in = { .alpha = 2.1f * cosf(th), .beta = 1.3f * sinf(th) };

        foc_abc_t abc;
        foc_ab_t  back;
        foc_inv_clarke(&in, &abc);
        foc_clarke(&abc, &back);

        CHECK_NEAR(back.alpha, in.alpha, TOL, "θ=%.4f: 往返后 α 变了", th);
        CHECK_NEAR(back.beta,  in.beta,  TOL, "θ=%.4f: 往返后 β 变了", th);
    }
}

/* ======================================================================== */
/* ③ 逆Clarke 的输出三相之和必须恒为 0（无中线，零序无处可去）              */
/* ======================================================================== */
static void test_inv_clarke_sum_zero(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;
        foc_ab_t in = { .alpha = 5.0f * cosf(th), .beta = 5.0f * sinf(th) };

        foc_abc_t abc;
        foc_inv_clarke(&in, &abc);

        CHECK_NEAR(abc.a + abc.b + abc.c, 0.0f, TOL,
                   "θ=%.4f: a+b+c 应为 0，实得 a=%.4f b=%.4f c=%.4f",
                   th, abc.a, abc.b, abc.c);
    }
}

/* ======================================================================== */
/* ④ Park 的定义性质 ★ 最重要的一条                                         */
/*    旋转矢量用同速旋转的坐标系去看，必须是静止的                          */
/* ======================================================================== */
static void test_park_rotating_vector_is_static(void)
{
    const float M = 2.5f;

    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;

        foc_ab_t in = { .alpha = M * cosf(th), .beta = M * sinf(th) };
        foc_sincos_t sc;
        foc_dq_t dq;

        foc_sincos(th, &sc);
        foc_park(&in, &sc, &dq);

        CHECK_NEAR(dq.d, M,    TOL, "θ=%.4f: d 应恒为 %.2f", th, M);
        CHECK_NEAR(dq.q, 0.0f, TOL, "θ=%.4f: q 应恒为 0",     th);
    }
}

/* ======================================================================== */
/* ⑤ Park 保持矢量模长（旋转不改变长度）                                    */
/* ======================================================================== */
static void test_park_preserves_magnitude(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;

        foc_ab_t in = { .alpha = 0.37f, .beta = -0.82f };   /* 固定的任意矢量 */
        foc_sincos_t sc;
        foc_dq_t dq;

        foc_sincos(th, &sc);
        foc_park(&in, &sc, &dq);

        float mag_in = in.alpha * in.alpha + in.beta * in.beta;
        float mag_dq = dq.d * dq.d + dq.q * dq.q;

        CHECK_NEAR(mag_dq, mag_in, TOL,
                   "θ=%.4f: Park 改变了矢量长度", th);
    }
}

/* ======================================================================== */
/* ⑥ Park ↔ 逆Park 往返（抓符号写反，最经典的 bug）                         */
/* ======================================================================== */
static void test_park_roundtrip(void)
{
    for (int k = 0; k < NPTS; k++) {
        float th = (float)k * FOC_2PI / (float)NPTS;

        foc_ab_t in = { .alpha = 0.37f, .beta = -0.82f };
        foc_sincos_t sc;
        foc_dq_t dq;
        foc_ab_t back;

        foc_sincos(th, &sc);
        foc_park(&in, &sc, &dq);
        foc_inv_park(&dq, &sc, &back);

        CHECK_NEAR(back.alpha, in.alpha, TOL, "θ=%.4f: 往返后 α 变了", th);
        CHECK_NEAR(back.beta,  in.beta,  TOL, "θ=%.4f: 往返后 β 变了", th);
    }
}

/* ======================================================================== */
void test_transform_all(void)
{
    RUN_TEST(test_clarke_amplitude_invariant);
    RUN_TEST(test_clarke_roundtrip);
    RUN_TEST(test_inv_clarke_sum_zero);
    RUN_TEST(test_park_rotating_vector_is_static);
    RUN_TEST(test_park_preserves_magnitude);
    RUN_TEST(test_park_roundtrip);
}
