/**
 * @file    svpwm.c
 * @brief   七段式 SVPWM 的实现
 *
 * 数学约定、限幅规则、验收标准见 svpwm.h —— 动手前请通读。
 */
#include "svpwm.h"
#include <math.h>

/* ------------------------------------------------------------------------ */
/*此为扇区判断函数，注意按照我这样的写法，对于和扇区边界重合的情况，比如θ=0°，是判断到扇区6的，θ=60°是判断到扇区2的，零矢量也是判断到扇区6，但这其实没影响*/
/*因为扇区的判断是为了后续计算所在扇区，相邻矢量的作用时间而服务的，比如说θ=0°，实际上不管是判断到扇区1还是6，最后都是算出V2或V6的作用时间是0的，只有V1在作用*/
uint8_t foc_svpwm_sector(const foc_ab_t *u_ab)
{
    if(u_ab->beta > 0.0f)/*若在扇区的上半部分*/
    {
        if(u_ab->beta < FOC_SQRT3 * u_ab->alpha)/*在扇区上半部分且在60°线下方*/
        {
            return 1;
        }
        else/*在扇区上半部分且在60°线上方*/
        {
            if(u_ab->beta > -FOC_SQRT3 * u_ab->alpha)/*在扇区上半部分且在60°线上方和120°线上方*/
            {
                return 2;
            }
            else/*在扇区上半部分且在60°线上方和120°线下方*/
            {
                return 3;
            }
        }

    }
    else/*若在扇区的下半部分*/
    {
        if(u_ab->beta > FOC_SQRT3 * u_ab->alpha)/*在扇区下半部分且在60°线上方*/
        {
            return 4;
        }
        else/*在扇区下半部分且在60°线下方*/
        {
            if(u_ab->beta < -FOC_SQRT3 * u_ab->alpha)/*在扇区下半部分且在60°线下方,和120°线下方*/
            {
                return 5;
            }
            else/*在扇区下半部分且在60°线下方,和120°线上方*/
            {
                return 6;
            }
        }
    }
}

/* ------------------------------------------------------------------------ */
bool foc_svpwm(const foc_ab_t *u_ab, float vdc, float d_max, foc_abc_t *duty)
{
    /* TODO: 七段式 SVPWM
     *
     * 分两步走，建议先让第 1 步的测试全绿，再加第 2 步：
     *
     *   第 1 步：算出未限幅的 da/db/dc
     *            必须满足 max+min = 1（七段式签名）
     *            必须满足"反算回去等于输入矢量"
     *
     *   第 2 步：按 svpwm.h §4 的规则做限幅
     *            k = (d_max - 0.5) / (dmx - 0.5)
     *            d_x ← 0.5 + k·(d_x - 0.5)
     *            返回是否发生了限幅
     *
     * 实现方法 A（扇区+作用时间）或 B（min-max 注入）都可以，见 svpwm.h §5。
     */
    (void)u_ab; (void)vdc; (void)d_max;
    duty->a = 0.5f;
    duty->b = 0.5f;
    duty->c = 0.5f;
    return false;
}
