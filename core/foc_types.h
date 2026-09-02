/**
 * @file    foc_types.h
 * @brief   FOC 算法库的基础类型与常量
 *
 * 本目录（core/）下的所有代码必须保持"平台无关"：
 *   - 只允许 include <stdint.h> <stdbool.h> <math.h>
 *   - 禁止 include 任何 HAL / CMSIS / 寄存器头文件
 *   - 禁止直接读写任何硬件寄存器
 * 这样同一份 .c 才能既编进 STM32，又编成 .so 给 Simulink / Python 调用。
 */
#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 数学常量（float 精度，避免隐式转 double） ---------- */
#define FOC_PI          3.14159265358979323846f
#define FOC_2PI         6.28318530717958647692f
#define FOC_PI_3        1.04719755119659774615f   /* π/3  = 60°  */
#define FOC_SQRT3       1.73205080756887729353f
#define FOC_1_SQRT3     0.57735026918962576451f   /* 1/√3        */
#define FOC_SQRT3_2     0.86602540378443864676f   /* √3/2        */
#define FOC_2_3         0.66666666666666666667f   /* 2/3         */
#define FOC_1_3         0.33333333333333333333f   /* 1/3         */

/* ---------- 坐标系数据类型 ---------- */

/** 三相量（abc 静止坐标系）。电流单位 A，电压单位 V，占空比无量纲 0~1 */
typedef struct { float a, b, c; } foc_abc_t;

/** 两相静止坐标系（αβ），Clarke 变换的输出 */
typedef struct { float alpha, beta; } foc_ab_t;

/** 两相旋转坐标系（dq），Park 变换的输出 */
typedef struct { float d, q; } foc_dq_t;

/** 电角度的正弦余弦对。一次三角运算复用于 Park 和反 Park，省一半开销 */
typedef struct { float sin, cos; } foc_sincos_t;

#ifdef __cplusplus
}
#endif
#endif /* FOC_TYPES_H */
