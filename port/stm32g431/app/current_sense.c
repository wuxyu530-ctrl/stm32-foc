/**
 * @file    current_sense.c
 * @brief   三电阻低边电流采样实现，见 current_sense.h
 */
#include "current_sense.h"
#include "main.h"
#include "adc.h"
#include "opamp.h"

static volatile uint16_t s_code[3];             /* 最近一次原始码值 */
static float             s_code0[3] = { CS_CODE0_NOMINAL, CS_CODE0_NOMINAL, CS_CODE0_NOMINAL };

/* 零点标定期间：ISR 只累加，不回调应用层 */
static volatile bool     s_calibrating = false;
static volatile uint32_t s_cal_count   = 0;
static volatile uint32_t s_cal_sum[3];

void cs_init(void)
{
    /* 运放上电。PGA 模式、×16、外部偏置，配置都在 CubeMX 生成的 MX_OPAMPx_Init 里 */
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /* ADC 自校准：消除 ADC 自身的偏移误差。必须在 ADC 使能之前做 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 开启注入组，等待 TIM1_TRGO2 硬件触发。
     * ADC1 只转 1 个通道（相 1），ADC2 顺序转 2 个通道（相 2、相 3），ADC2 最后完成，
     * 所以只开 ADC2 的注入序列完成中断（JEOS），在那里一次读全三相。 */
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
}

bool cs_calibrate_offset(uint32_t n)
{
    s_cal_sum[0] = s_cal_sum[1] = s_cal_sum[2] = 0;
    s_cal_count  = 0;
    s_calibrating = true;

    uint32_t t0 = HAL_GetTick();
    while (s_cal_count < n) {
        if (HAL_GetTick() - t0 > 1000u + n / 10u) {   /* 20 kHz 下 n 个样本约 n/20 ms */
            s_calibrating = false;
            return false;                              /* 没有触发：TIM1 没在跑？ */
        }
    }
    s_calibrating = false;

    bool ok = true;
    for (int k = 0; k < 3; k++) {
        s_code0[k] = (float)s_cal_sum[k] / (float)n;
        float dev = s_code0[k] - (float)CS_CODE0_NOMINAL;
        if (dev > 100.0f || dev < -100.0f) ok = false;  /* 偏离理论值 100 码以上，大概率是接错/没上电 */
    }
    return ok;
}

void cs_get_offset(float code0[3])
{
    code0[0] = s_code0[0]; code0[1] = s_code0[1]; code0[2] = s_code0[2];
}

void cs_get_raw(uint16_t code[3])
{
    code[0] = s_code[0]; code[1] = s_code[1]; code[2] = s_code[2];
}

/* ADC 注入序列完成中断回调（HAL 弱函数的强定义）。ADC1/ADC2 共用 ADC1_2_IRQn，
 * HAL 会对两个句柄都检查一遍，所以这里要判断是不是 ADC2。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;

    uint16_t c0 = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);  /* 相 1：OPAMP1 */
    uint16_t c1 = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);  /* 相 2：OPAMP2 */
    uint16_t c2 = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);  /* 相 3：OPAMP3 */
    s_code[0] = c0; s_code[1] = c1; s_code[2] = c2;

    if (s_calibrating) {
        s_cal_sum[0] += c0; s_cal_sum[1] += c1; s_cal_sum[2] += c2;
        s_cal_count++;
        return;
    }

    foc_abc_t i = {
        .a = -((float)c0 - s_code0[0]) * CS_AMP_PER_LSB,
        .b = -((float)c1 - s_code0[1]) * CS_AMP_PER_LSB,
        .c = -((float)c2 - s_code0[2]) * CS_AMP_PER_LSB,
    };
    cs_on_sample(&i);
}
