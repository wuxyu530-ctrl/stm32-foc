/**
 * @file    pi_sim.h
 * @brief   给 Simulink C Caller 调用的 PI 封装（只在仿真里用，不编进固件）
 *
 * Simulink 的信号是 double，固件里的 PI 是 float：这里做类型转换，
 * 并持有一个 foc_pi_t 实例作为状态。被测的是 core/pi.c 里那份真正的固件代码。
 *
 * 使用要求：
 *   - C Caller 的采样时间必须设为 Ts（每个控制周期只调用一次，否则状态会被多算）
 *   - 模型设置 → 仿真目标 → 初始化函数填 pi_sim_reset();（每次仿真开始清零）
 */
#ifndef PI_SIM_H
#define PI_SIM_H

void   pi_sim_reset(void);
double pi_sim_step(double err, double kp, double ki, double ts, double out_max);

#endif
