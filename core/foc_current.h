/**
 * @file    foc_current.h
 * @brief   dq 电流环：一个控制周期的完整计算（平台无关）
 *
 * 【Claude 代写，待复核】见 docs/理解清单.md「M7 代码复核」与 docs/M7_电流闭环.md
 *
 * ============================================================================
 * 1. 一次 foc_cur_step() 做的事（每个 PWM 周期在 ADC 中断里调用一次）
 * ============================================================================
 *
 *   三相电流 i_abc ──Clarke──► i_αβ ──Park(θe)──► id, iq
 *                                                   │
 *        id_ref − id ──► PI_d ──► vd ┐              │
 *        iq_ref − iq ──► PI_q ──► vq ┤ d 轴优先限幅 │
 *                                    ▼              │
 *                     (vd, vq) ──反 Park(θe)──► v_αβ ──SVPWM──► 三相占空比
 *
 *   占空比由调用者写进 CCR，下一个更新事件生效（本板约半个周期后，见 HARDWARE.md）。
 *
 * ============================================================================
 * 2. 电压矢量限幅：d 轴优先
 * ============================================================================
 *
 * 真实 FOC 限的是矢量长度 √(vd² + vq²) ≤ v_max，而不是两轴各自 ±v_max。
 * 本实现：
 *   ① PI_d 的限幅 = v_max，先算出 vd
 *   ② PI_q 的限幅 = √(v_max² − vd²)，再算 vq
 * 两个 PI 各自按自己的限幅做钳位抗饱和（core/pi.c）。
 * d 轴优先的理由：d 轴电流决定磁场（id = 0 控制时需要它牢牢守住 0），
 * 剩下的电压余量才给产生转矩的 q 轴。
 *
 * ============================================================================
 * 3. v_max 怎么定：让 SVPWM 永远不需要再截断
 * ============================================================================
 *
 * 理想上限是 Vbus/√3（六边形内切圆）。但本板低边采样要求每相下管在周期中间
 * 导通足够久，占空比最多到 d_max（M2_DMAX = 0.956）。七段式 SVPWM 满足
 * max(d) + min(d) = 1，矢量长度为 U 时 max(d) = 0.5 + U·√3/(2·Vbus)，
 * 令 max(d) = d_max 得：
 *
 *        v_max = (2·d_max − 1) · Vbus / √3          （12 V、0.956 → 6.32 V）
 *
 * 用它给 PI 限幅，SVPWM 算出的占空比就天然不超过 d_max，不会被硬截断变形。
 */
#ifndef FOC_CURRENT_H
#define FOC_CURRENT_H

#include "foc_types.h"
#include "pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    foc_pi_t pi_d;      /**< d 轴电流 PI */
    foc_pi_t pi_q;      /**< q 轴电流 PI */
    float    vbus;      /**< 母线电压 [V] */
    float    d_max;     /**< 占空比上限 */
    float    v_max;     /**< 电压矢量长度上限 [V]，由 vbus、d_max 算出（§3） */
} foc_cur_t;

/** 每周期的中间量，给调试/记录用 */
typedef struct {
    float id, iq;       /**< 实测 dq 电流 [A] */
    float vd, vq;       /**< 限幅后的 dq 电压指令 [V] */
} foc_cur_out_t;

/**
 * @brief 初始化。两个轴用同一组 Kp、Ki（表贴式电机 Ld ≈ Lq）
 * @param kp, ki  按 Kp = L·ω_bw、Ki = R·ω_bw 设计
 */
void foc_cur_init(foc_cur_t *c, float kp, float ki, float ts, float vbus, float d_max);

/** @brief 清零两个 PI 的积分（每次使能电流环前调用） */
void foc_cur_reset(foc_cur_t *c);

/** @brief 母线电压变化时更新 vbus 和 v_max（以后接母线电压 ADC 用） */
void foc_cur_set_vbus(foc_cur_t *c, float vbus);

/**
 * @brief 执行一个控制周期
 * @param i_abc   三相电流 [A]
 * @param sc      电角度 θe 的 sin、cos
 * @param id_ref  d 轴电流目标 [A]
 * @param iq_ref  q 轴电流目标 [A]
 * @param duty    输出三相占空比（0~1，已保证不超过 d_max）
 * @param out     可为 NULL；非空时写入本周期的 id、iq、vd、vq
 */
void foc_cur_step(foc_cur_t *c, const foc_abc_t *i_abc, const foc_sincos_t *sc,
                  float id_ref, float iq_ref, foc_abc_t *duty, foc_cur_out_t *out);

#ifdef __cplusplus
}
#endif
#endif /* FOC_CURRENT_H */
