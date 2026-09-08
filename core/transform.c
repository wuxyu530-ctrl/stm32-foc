/**
 * @file    transform.c
 * @brief   FOC 坐标变换的实现
 *
 * 数学约定、验收标准、以及各接口的详细说明见 transform.h。
 * 实现前请先通读该头文件。
 * 前提是a b c三相定子线圈，下面说的α轴和a相重合，并且对于d q旋转坐标系而言，这个d轴超前α轴θ角，q轴超前d轴90°，超前的方向这里定位逆时针
 */
#include "transform.h"
#include <math.h>

/* ------------------------------------------------------------------------ */
void foc_sincos(float theta, foc_sincos_t *out)
{
    /* 这个已经给你了 —— 后面若要换 CORDIC 或查表，只改这一个函数 */
    out->sin = sinf(theta);
    out->cos = cosf(theta);
}

/* ------------------------------------------------------------------------ */
void foc_clarke(const foc_abc_t *in, foc_ab_t *out)
{
    /* 把a b c三相定子线圈的三相电压，投影到α β坐标系上(静止->静止)，也就是把Ua Ub Uc换算成Uα Uβ，前面需要乘一个系数2/3，这个叫等幅值 */
    out->alpha = FOC_2_3*(in->a -(in->b+in->c)*0.5f);
    out->beta  = FOC_1_SQRT3*(in->b - in->c);
}

/* ------------------------------------------------------------------------ */
void foc_inv_clarke(const foc_ab_t *in, foc_abc_t *out)
{
    /* 可以用clarke变换的结果，加上Ua+Ub+Uc = 0（三相电压之和恒为0）解方程求出Ua Ub Uc；也可以直接Uα Uβ直接往 a b c做投影即可得出结果（但不要乘2/3系数，为什么？）*/
    out->a = in->alpha;
    out->b = FOC_SQRT3_2*in->beta - in->alpha*0.5f;
    out->c = -FOC_SQRT3_2*in->beta - in->alpha*0.5f;
}

/* ------------------------------------------------------------------------ */
void foc_park(const foc_ab_t *in, const foc_sincos_t *sc, foc_dq_t *out)
{
    /*已知Uα Uβ，求Ud Uq（静止坐标系->旋转坐标系），实际上是同一个矢量，需要把它从水平 竖直坐标系上的量度，变换到θ角的d q坐标系上的量度*/
    /*标准的做法是，先求出 d q这个新的旋转坐标系，d q这两个轴的单位矢量，在α β这个水平竖直坐标系下的向量表示，然后再用已知矢量 Uα，Uβ往d q这个单位矢量上做投影，方法是直接点积，即可得到结果*/
    out->d = in->alpha * sc->cos + in->beta * sc->sin;
    out->q = -in->alpha*sc->sin + in->beta*sc->cos;
}

/* ------------------------------------------------------------------------ */
void foc_inv_park(const foc_dq_t *in, const foc_sincos_t *sc, foc_ab_t *out)
{
    /*已知Ud Uq，然后反向投影到α β上，即可得到结果*/
    out->alpha = in->d * sc->cos - in->q*sc->sin;
    out->beta  = in->q*sc->cos + in->d*sc->sin;
}
