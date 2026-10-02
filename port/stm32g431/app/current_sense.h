/**
 * @file    current_sense.h
 * @brief   三电阻低边电流采样（B-G431B-ESC1 板级）
 *
 * 信号链（见 HARDWARE.md §1.1 与原理图 SHUNT SENSING CIRCUIT）：
 *   相电流 → R54/55/56 (3 mΩ) → 1.5k/22k/2.2k 偏置衰减网络 → OPAMP PGA ×16（外部偏置）
 *         → ADC 注入组（TIM1_TRGO2 在 000 窗口触发）→ 12 位码值
 *
 *   i = -(code - code0) × CS_AMP_PER_LSB
 *
 *   code0          零电流时的码值，理论 ≈ 2553，每相实测标定
 *   CS_AMP_PER_LSB 每个码值对应的电流，由四个电阻 + PGA 增益算出
 *   负号           低边采样：电流流出逆变器（进电机）为正时，000 期间电流经采样电阻
 *                  自下而上，采样电阻上端电压为负 → 码值下降
 *
 * 调用关系：
 *   cs_init()  →  （TIM1 计数器运行）→  cs_calibrate_offset()  →  每个 PWM 周期
 *   ADC2 注入序列完成中断 → 本模块换算成安培 → 回调 cs_on_sample()（由应用层实现）
 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>
#include "foc_types.h"

#define CS_AMP_PER_LSB   0.029372f   /* A / LSB，HARDWARE.md §1.1 */
#define CS_CODE0_NOMINAL 2553u       /* 理论零点，用于判断标定结果是否离谱 */

/* 启动三个运放、校准 ADC、开启注入组（等 TIM1_TRGO2 触发）。TIM1 计数器需另行启动。 */
void cs_init(void);

/* 采 n 个周期取平均，得到三相零点。必须在 MOS 全关（无电流）、TIM1 计数器运行时调用。阻塞。
 * 返回 false 表示超时（没有触发）或零点偏离理论值太多。 */
bool cs_calibrate_offset(uint32_t n);

/* 读标定得到的零点 */
void cs_get_offset(float code0[3]);

/* 最近一次采样的原始码值（调试用） */
void cs_get_raw(uint16_t code[3]);

/* 应用层实现：每个 PWM 周期（20 kHz）在 ADC 中断里被调用一次，传入三相电流 [A]。
 * 标定期间不会调用。 */
void cs_on_sample(const foc_abc_t *i_abc);

#endif /* CURRENT_SENSE_H */
