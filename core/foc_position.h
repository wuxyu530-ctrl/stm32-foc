/* SPDX-License-Identifier: MIT */
/**
 * @file    foc_position.h
 * @brief   位置环：编码器累计位置 + 位置 P 控制（平台无关）
 *
 * 设计见 docs/M9_位置环.md
 *
 * ============================================================================
 * 1. 在三环里的位置
 * ============================================================================
 *
 *   θ_ref ─►(−)─► 位置 P ─► ω_ref ─► 速度环（foc_speed.c）─► iq_ref ─► 电流环 ─► 电机
 *            ▲                                                                   │
 *            └──────────── 编码器累计位置 θ ◄──────────────────────────────────────┘
 *
 *   控制器（本文件）：输入 位置误差 [rad，机械角]，输出 ω_ref [rad/s，机械角速度]
 *   被控对象：速度环闭环（带宽约 100 Hz）+ 积分（转速积分成位置）
 *
 * ============================================================================
 * 2. 为什么只用 P
 * ============================================================================
 *
 *   被控对象里已经有一个积分（转速 → 位置），P 控制下阶跃无稳态误差；
 *   恒定负载转矩由内层速度环的积分抵消（静止时速度误差必须为 0 → ω_ref = 0 → 位置误差 = 0）。
 *   Kp = ωc_pos：开环 ≈ Kp/s，令 ωc_pos 处增益为 1；ωc_pos 取速度环穿越频率的约 1/5。
 *   输出限幅 ω_max：大阶跃时以恒定转速走，避免速度环和电流环饱和。
 *
 * ============================================================================
 * 3. 位置测量
 * ============================================================================
 *
 *   每次调用读计数（0 ~ cpr−1），与上次相减、处理回绕后累加成连续位置：
 *     θ += dir · d · 2π / cpr        [rad，机械角，reset 时刻为 0]
 *   分辨率 2π/4096 ≈ 0.088°。前提：两次调用之间转过不到半圈。
 */
#ifndef FOC_POSITION_H
#define FOC_POSITION_H

#include <stdint.h>
#include "foc_types.h"
#include "pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    foc_pi_t pi;        /**< 位置 P（ki = 0）：误差 [rad] → ω_ref [rad/s]，限幅 ω_max */
    float    k_cnt;     /**< 一个计数对应的角度 dir·2π/cpr [rad] */
    int32_t  cpr;       /**< 编码器每圈计数 */
    uint32_t cnt_last;  /**< 上一次的计数 */
    int32_t  pos_cnt;   /**< 累计位置 [count]（带方向），reset 时为 0 */
    float    theta;     /**< 累计位置 [rad，机械角] */
} foc_pos_t;

/**
 * @brief 初始化
 * @param kp     位置环比例系数 [(rad/s)/rad] = 期望穿越频率 ωc_pos [rad/s]
 * @param w_max  输出转速限幅 [rad/s]
 * @param dir    编码器方向（+1 / −1），与 foc_speed 相同
 * @param cpr    编码器每圈计数
 */
void foc_pos_init(foc_pos_t *p, float kp, float w_max, int dir, int32_t cpr);

/** @brief 以当前计数为位置零点 */
void foc_pos_reset(foc_pos_t *p, uint32_t cnt);

/**
 * @brief 执行一个位置环周期
 * @param cnt        当前编码器计数（0 ~ cpr−1）
 * @param theta_ref  目标位置 [rad，机械角，相对 reset 时刻]
 * @return           ω_ref [rad/s]，|ω_ref| ≤ ω_max
 */
float foc_pos_step(foc_pos_t *p, uint32_t cnt, float theta_ref);

#ifdef __cplusplus
}
#endif
#endif /* FOC_POSITION_H */
