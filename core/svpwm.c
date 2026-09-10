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
    float t1,t2,dutymax,k;

    /* 六个基本矢量的 (α, β) 分量，推导 switch 里各扇区系数时的依据。
     * 系数已展开写死在下面各 case 中，此表仅作注释保留：
     *     V1 = ( 2/3·vdc,          0        )   mos 100
     *     V2 = ( 1/3·vdc,  1/√3·vdc)          mos 110
     *     V3 = (-1/3·vdc,  1/√3·vdc)          mos 010
     *     V4 = (-2/3·vdc,          0        )   mos 011
     *     V5 = (-1/3·vdc, -1/√3·vdc)          mos 001
     *     V6 = ( 1/3·vdc, -1/√3·vdc)          mos 101
     * 六个矢量模长均为 2/3·vdc，互隔 60°。 */

    uint8_t Sectors = foc_svpwm_sector(u_ab);
    
    switch(Sectors){
        case 1:
            t2 = FOC_SQRT3*u_ab->beta/vdc;
            t1 = 0.5f*(3*u_ab->alpha - FOC_SQRT3*u_ab->beta)/vdc;
            duty->a = 0.5f*(1+t1+t2);
            duty->b = 0.5f*(1-t1+t2);
            duty->c = 0.5f*(1-t1-t2);
            break;

        case 2:
            t1 = FOC_SQRT3_2*(FOC_SQRT3*u_ab->alpha+u_ab->beta)/vdc;
            t2 = FOC_SQRT3_2*(u_ab->beta-FOC_SQRT3*u_ab->alpha)/vdc;
            duty->a = 0.5f*(1+t1-t2);
            duty->b = 0.5f*(1+t1+t2);
            duty->c = 0.5f*(1-t1-t2);
            break;

        case 3:
            t1 = FOC_SQRT3*u_ab->beta/vdc;
            t2 = -0.5f*(FOC_SQRT3*u_ab->beta+3*u_ab->alpha)/vdc;
            duty->a = 0.5f*(1-t1-t2);
            duty->b = 0.5f*(1+t1+t2);
            duty->c = 0.5f*(1-t1+t2);
            break;

        case 4:
            t2 = -FOC_SQRT3*u_ab->beta/vdc;
            t1 = 0.5f*(FOC_SQRT3*u_ab->beta-3*u_ab->alpha)/vdc;
            duty->a = 0.5f*(1-t1-t2);
            duty->b = 0.5f*(1+t1-t2);
            duty->c = 0.5f*(1+t1+t2);

            break;

        case 5:
            t1 = -FOC_SQRT3_2*(FOC_SQRT3*u_ab->alpha+u_ab->beta)/vdc;
            t2 = FOC_SQRT3_2*(FOC_SQRT3*u_ab->alpha-u_ab->beta)/vdc;
            duty->a = 0.5f*(1+t2-t1);
            duty->b = 0.5f*(1-t1-t2);
            duty->c = 0.5f*(1+t1+t2);
            break;

        case 6:
            t1 = -FOC_SQRT3*u_ab->beta/vdc;
            t2 = 0.5f*(FOC_SQRT3*u_ab->beta+3*u_ab->alpha)/vdc;
            duty->a = 0.5f*(1+t1+t2);
            duty->b = 0.5f*(1-t1-t2);
            duty->c = 0.5f*(1+t1-t2);
            break;
        default:
            /* foc_svpwm_sector() 只会返回 1~6，正常走不到这里。
             * 但一旦走到，duty 会是未初始化的栈垃圾并被直接写进 CCR，
             * 因此兜底成三相同电位（等效零矢量，电机不受激励），
             * 并返回 true 通知电流环停止积分。 */
            duty->a = 0.5f;
            duty->b = 0.5f;
            duty->c = 0.5f;
            return true;
    }

    /*限幅占空比，确保当目标矢量超出边界（六边形边界），具体表现是t1+t2>1，或者说某一相的占空比大于1或某一相的占空比小于0时，可以正常的保证方向的同时缩放回边界内，确保在输出能力内，避免钳位*/
    dutymax = fmaxf(duty->a, fmaxf(duty->b, duty->c));/*C 标准库没有三参数的 max，math.h 提供的是两参数的 fmaxf（float 版），嵌套取三相最大*/
    /*下面只考虑dutymax>1的情况，不用再考虑dutymin<0的情况，原因是因为我们的mos状态是七段式变化的，满足零矢量均分，天然的存在dutymax+dutymin=1的等式，于是dutymax>1等价于dutymin<0*/
    if(dutymax > d_max)
    {
        k = (d_max-0.5f)/(dutymax-0.5f);/*系数这么算，不直接用d_max/dmax，是因为我们占空比的本质是两种不同mos状态维持的时间，我们三相的占空比都是0.5+系数*维持时间来的*/
        /*我们要生成某个方向的矢量，实际上是根据V1 V2的维持时间来生成的，实际上本来的系数kt应该等于 kt = t_max/(t1+t2),然后t1new=kt*t1 t2new=kt*t2*/
        /*代回da db dc，会发现kt = (da新-0.5)/(da-0.5) ,所以系数是这么来的*/
        /*一个更强力的解释是，在扇区内t1 t2都是有关U的线性函数，所以实际上这里svpwm限幅后的输出，就等于拿kU去跑一遍svpwm，实际上把目标矢量缩短到k倍就完全等价于时间缩短k倍，于是也就能得出占空比算出的比例系数*/
        duty->a = 0.5f+k*(duty->a-0.5f);
        duty->b = 0.5f+k*(duty->b-0.5f);
        duty->c = 0.5f+k*(duty->c-0.5f);

        return true;
    }
    else
    {
        return false;
    }

}
