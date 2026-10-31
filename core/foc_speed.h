/**
 * @file    foc_speed.h
 * @brief   速度环：编码器测速 + 速度 PI（平台无关）
 *
 * 【Claude 代写，待复核】设计推导见 docs/M8b_速度环.md
 *
 * ============================================================================
 * 1. 在三环里的位置
 * ============================================================================
 *
 *   ω_ref ─►(−)─► 速度 PI ─► iq_ref ─► 电流环（foc_current.c）─► iq ─► 电机机械部分 ─► ω
 *            ▲                                                                       │
 *            └──────────── 测速：编码器计数差 / Ts ◄──────────────────────────────────┘
 *
 *   控制器（本文件）：输入 速度误差 [rad/s]，输出 iq_ref [A]
 *   被控对象（M8a 实测）：iq → ω = (Kt/J) / (s + B/J)，Kt/J ≈ 1.1e4 rad/(s²·A)，B/J ≈ 1 /s
 *
 * ============================================================================
 * 2. 测速
 * ============================================================================
 *
 *   每 Ts 调用一次，读编码器计数 cnt（0 ~ cpr−1 回绕），与上一次相减：
 *
 *     d = cnt − cnt_last，超过 ±cpr/2 视为跨过回绕点，补一圈
 *     ω = dir · d · 2π / (cpr · Ts)            [机械角速度 rad/s]
 *
 *   dir：θe 增大时计数减小取 −1（本项目），使 ω > 0 与 iq > 0 产生的转向一致。
 *   分辨率：1 个计数 = 2π/(4096 × 1 ms) ≈ 1.53 rad/s ≈ 14.6 rpm —— 这是速度环噪声的主要来源，
 *   也是不用 D 项的原因（微分会放大它）。
 *   前提：两次调用之间转过不到半圈（Ts = 1 ms 时 < 30000 rpm）。
 *
 * ============================================================================
 * 3. PI：直接复用 core/pi.c（前向欧拉 + 限幅 + 钳位抗饱和）
 * ============================================================================
 *
 *   Kp = ωc / (Kt/J)          被控对象在 ωc 附近 ≈ (Kt/J)/s，令开环增益在 ωc 处为 1
 *   Ki = Kp · ωc / 4          PI 零点放在 ωc/4：积分够快，才能及时消除恒定摩擦
 *   限幅 iq_max                钳位规则与电流环相同
 *
 *   ωc = 2π × 50 Hz 时：Kp = 0.0286 A/(rad/s)，Ki = 2.24 A/rad，相位裕度约 55°。
 */
#ifndef FOC_SPEED_H
#define FOC_SPEED_H

#include <stdint.h>
#include "foc_types.h"
#include "pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    foc_pi_t pi;        /**< 速度 PI：误差 [rad/s] → iq_ref [A] */
    float    k_cnt;     /**< 一个计数差对应的角速度 dir·2π/(cpr·Ts) [rad/s] */
    int32_t  cpr;       /**< 编码器每圈计数 */
    uint32_t cnt_last;  /**< 上一次的计数 */
    float    w;         /**< 最近一次测得的机械角速度 [rad/s] */
} foc_spd_t;

/**
 * @brief 初始化
 * @param kp, ki   速度 PI 参数 [A/(rad/s)]、[A/rad]
 * @param ts       速度环周期 [s]
 * @param iq_max   iq_ref 限幅 [A]
 * @param dir      编码器方向（+1 / −1），见 §2
 * @param cpr      编码器每圈计数
 */
void foc_spd_init(foc_spd_t *s, float kp, float ki, float ts, float iq_max, int dir, int32_t cpr);

/**
 * @brief 使能速度环前调用：记下当前计数作为测速起点，积分设为 iq_init
 * @param iq_init  积分初值 [A]。从静止启动给 0；若想无扰切换，给切换前的 iq
 */
void foc_spd_reset(foc_spd_t *s, uint32_t cnt, float iq_init);

/**
 * @brief 执行一个速度环周期（每 Ts 一次）
 * @param cnt    当前编码器计数（0 ~ cpr−1）
 * @param w_ref  目标机械角速度 [rad/s]
 * @return       iq_ref [A]，|iq_ref| ≤ iq_max
 */
float foc_spd_step(foc_spd_t *s, uint32_t cnt, float w_ref);

#ifdef __cplusplus
}
#endif
#endif /* FOC_SPEED_H */
