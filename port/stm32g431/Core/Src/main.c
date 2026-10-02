/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include "svpwm.h"
#include "transform.h"
#include "current_sense.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 当前跑哪个实验。改这一行再编译烧录即可切换。 */
#define APP_M2_OPENLOOP    1   /* 开环 SVPWM 拖电机转 */
#define APP_M4A_ENCODER    2   /* 不开 PWM，手转电机，串口看编码器读数 */
#define APP_M4A2_CALIB     3   /* d 轴对齐 + 慢速拖转，测极对数、电角度零点、方向 */
#define APP_M3_CURRENT     4   /* 电流采样：零点标定 → 直流矢量验符号 → 开环转动看三相波形 */
#define APP_MODE           APP_M3_CURRENT

/* M3 电流采样参数 */
#define M3_U_DC            1.5f   /* 直流矢量测试电压 [V] */
#define M3_U_ROT           2.0f   /* 开环转动电压 [V] */
#define M3_F_ROT           5.0f   /* 开环转动电频率 [Hz] */
#define M3_I_TRIP          2.0f   /* 软件过流保护阈值 [A]：任一相超过即关 PWM */
#define PWM_HZ             20000.0f

/* M4a-2 标定参数 */
#define CAL_U_ALIGN        1.0f   /* d 轴电压 [V]。GM2804 相电阻几欧，1 V 约零点几安 */
#define CAL_F_ELEC         1.0f   /* 拖转电频率 [Hz]：每秒一整圈电角度，足够慢，转子精确跟随 */
#define CAL_N_ELEC_REVS    14     /* 拖多少圈电角度。7 对极 → 两圈机械 */
#define ENC_CPR            4096   /* 编码器每圈计数：1024 线 × 4 */

#define TIM1_ARR   4250u

/* M2：开环 SVPWM。每 1 ms 电角度前进一步，U_dq = (0, U_AMP) 经逆 Park → SVPWM → CCR。
 * 电机像步进电机一样跟随旋转磁场，不需要编码器和电流采样。
 *   M2_U_AMP   相电压幅值 [V]（幅值不变约定）。GM2804 相电阻约几欧，1 V 对应零点几安
 *   M2_F_ELEC  电角频率 [Hz]。7 对极 → 机械转速 = F_ELEC / 7 圈/秒
 *   M2_VDC     暂用固定 12 V，M3 之后改为 ADC 实测 */
#define M2_VDC     12.0f
#define M2_U_AMP   2.5f
#define M2_F_ELEC  30.0f
#define M2_DMAX    0.956f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* M4a：Z 相中断和主循环共享的变量。
 * 中断里写、主循环里读，必须加 volatile：告诉编译器"这个变量随时可能被别处改掉"，
 * 每次都老老实实去内存里读，不许把它缓存在寄存器里或者优化掉。 */
static volatile uint32_t g_enc_z_count  = 0;   /* Z 脉冲累计次数（每转一圈 +1） */
static volatile uint32_t g_enc_cnt_at_z = 0;   /* 最近一次 Z 脉冲到来时 TIM4 的计数值 */
static int32_t           g_enc_pos      = 0;   /* 上电以来的累计位置 [count]，处理了回绕，可正可负 */

/* ---- 20 kHz 控制中断（cs_on_sample）与主循环共享的状态 ---- */
typedef enum { CTL_IDLE = 0, CTL_HOLD, CTL_ROTATE } ctl_mode_t;
static volatile ctl_mode_t g_ctl_mode  = CTL_IDLE;  /* IDLE：中断里不写 CCR */
static volatile float      g_ctl_ud    = 0.0f;
static volatile float      g_ctl_uq    = 0.0f;
static volatile float      g_ctl_theta = 0.0f;      /* HOLD：固定角度；ROTATE：每周期递增 */
static volatile float      g_ctl_f     = 0.0f;      /* ROTATE 的电频率 [Hz] */
static volatile foc_abc_t  g_i_abc;                 /* 最近一次三相电流 [A] */
static volatile int        g_fault     = 0;         /* 过流跳闸后置 1 */

/* 求平均：主循环设 g_avg_left = n，中断每周期累加一次并减一，减到 0 表示完成 */
static volatile uint32_t   g_avg_left  = 0;
static volatile float      g_avg_sum[3];
static volatile float      g_avg_min, g_avg_max;    /* a 相的最小/最大值，看噪声 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- printf 重定向到 USART2（经板载 ST-LINK 虚拟串口到电脑） ----
 * AC6 标准库默认用"半主机"（借调试器输出）实现 printf，脱离调试器运行会卡死。
 * 下面声明不用半主机，补上库需要的几个底层函数，并让 fputc 走 USART2。 */
__asm(".global __use_no_semihosting");
void  _sys_exit(int ret)                       { (void)ret; while (1) {} }
void  _ttywrch(int ch)                         { (void)ch; }
char *_sys_command_string(char *cmd, int len)  { (void)cmd; (void)len; return NULL; }
FILE  __stdout;                                /* 自己提供 stdout，库就不用半主机去打开它 */
FILE  __stdin;

int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
    return ch;
}

/* 读 TIM4，把 0~4095 的回绕计数展开成连续的累计位置 g_enc_pos。
 * 前提：两次调用之间转过的角度小于半圈（2048 count）。1 ms 调一次时，对应 < 30000 rpm，绰绰有余。 */
static void enc_update(void)
{
    static uint32_t last = 0;
    static int      init = 0;
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);
    if (!init) { last = cnt; init = 1; }
    int32_t d = (int32_t)cnt - (int32_t)last;
    if (d >  ENC_CPR / 2) d -= ENC_CPR;
    if (d < -ENC_CPR / 2) d += ENC_CPR;
    g_enc_pos += d;
    last = cnt;
}

/* 在电角度 theta 方向上施加电压矢量 (Ud, Uq)，写 CCR */
static void apply_udq(float ud, float uq, float theta);

/* 把三相占空比写进 TIM1 CCR1~3（预装载使能，实际在下一个更新事件生效） */
static void pwm_set_duty(const foc_abc_t *d)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(d->a * TIM1_ARR + 0.5f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(d->b * TIM1_ARR + 0.5f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(d->c * TIM1_ARR + 0.5f));
}

static void apply_udq(float ud, float uq, float theta)
{
    foc_sincos_t sc  = { sinf(theta), cosf(theta) };
    foc_dq_t     udq = { ud, uq };
    foc_ab_t     uab;
    foc_abc_t    duty;
    foc_inv_park(&udq, &sc, &uab);
    foc_svpwm(&uab, M2_VDC, M2_DMAX, &duty);
    pwm_set_duty(&duty);
}

/* 开 PWM（先零矢量），供需要驱动电机的模式共用 */
static void pwm_start_zero(void)
{
    const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
    pwm_set_duty(&zero);
    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/* ---- 20 kHz 控制中断：每个 PWM 周期 ADC 采完三相后由 current_sense.c 调用 ---- */
void cs_on_sample(const foc_abc_t *i)
{
  DBG_PIN_GPIO_Port->BSRR = DBG_PIN_Pin;           /* PC11 拉高：示波器看中断时刻和耗时 */

  g_i_abc.a = i->a; g_i_abc.b = i->b; g_i_abc.c = i->c;

  /* 软件过流保护：任一相超阈值，立即关掉 TIM1 主输出（六管全关），不等主循环 */
  if (fabsf(i->a) > M3_I_TRIP || fabsf(i->b) > M3_I_TRIP || fabsf(i->c) > M3_I_TRIP) {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    g_ctl_mode = CTL_IDLE;
    g_fault = 1;
  }

  if (g_avg_left) {
    g_avg_sum[0] += i->a; g_avg_sum[1] += i->b; g_avg_sum[2] += i->c;
    if (i->a < g_avg_min) g_avg_min = i->a;
    if (i->a > g_avg_max) g_avg_max = i->a;
    g_avg_left--;
  }

  switch (g_ctl_mode) {
    case CTL_HOLD:
      apply_udq(g_ctl_ud, g_ctl_uq, g_ctl_theta);
      break;
    case CTL_ROTATE: {
      float th = g_ctl_theta + FOC_2PI * g_ctl_f / PWM_HZ;   /* 每周期前进 2π·f/20000 */
      if (th >= FOC_2PI) th -= FOC_2PI;
      g_ctl_theta = th;
      apply_udq(g_ctl_ud, g_ctl_uq, th);
      break;
    }
    default:
      break;
  }

  DBG_PIN_GPIO_Port->BRR = DBG_PIN_Pin;
}

/* 阻塞求 n 个周期的三相电流平均值，同时给出 a 相的峰峰值 */
static void avg_currents(uint32_t n, float out[3], float *pp_a)
{
  g_avg_sum[0] = g_avg_sum[1] = g_avg_sum[2] = 0.0f;
  g_avg_min = 1e9f; g_avg_max = -1e9f;
  g_avg_left = n;
  while (g_avg_left) { }
  out[0] = g_avg_sum[0] / (float)n;
  out[1] = g_avg_sum[1] / (float)n;
  out[2] = g_avg_sum[2] / (float)n;
  if (pp_a) *pp_a = g_avg_max - g_avg_min;
}

static void print_avg(const char *tag, uint32_t n)
{
  float a[3], pp;
  avg_currents(n, a, &pp);
  printf("%-14s ia=%+7.1f  ib=%+7.1f  ic=%+7.1f  sum=%+6.1f mA   (ia p-p %.1f mA)\r\n",
         tag, (double)(a[0]*1000.0f), (double)(a[1]*1000.0f), (double)(a[2]*1000.0f),
         (double)((a[0]+a[1]+a[2])*1000.0f), (double)(pp*1000.0f));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n==== 3-phase-foc, APP_MODE=%d, SYSCLK=%lu Hz ====\r\n",
         APP_MODE, (unsigned long)SystemCoreClock);

#if APP_MODE == APP_M2_OPENLOOP
  {
    /* 先输出零矢量（三相 0.5），再开 PWM；此时相间电压为零，电机不动 */
    const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
    pwm_set_duty(&zero);

    HAL_Delay(3000);                              /* 上电 3 s 后再开，留时间看静态电流 */

    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_1);   /* CHx  = 上管 */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  /* CHxN = 下管（互补 + 死区） */
    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start  (&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* TIM1 是高级定时器，HAL_TIM_PWM_Start 内部会置 MOE，不必手动开 */
  }
#elif APP_MODE == APP_M4A_ENCODER
  /* PWM 不启动，六个管子全关，电机可以用手自由转动。 */

  /* 启动 TIM4 编码器接口。CubeMX 已配成 TI1+TI2 四倍频、ARR = 4095（1024 线 × 4 = 4096 计数/圈）。
   * 必须用 TIM_CHANNEL_ALL：A 相进 CH1、B 相进 CH2，两路都要开才能判方向、四倍频计数。 */
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
    printf("TIM4 encoder start FAILED\r\n");
  }
  printf("M4a: turn the motor by hand. 1 rev should be 4096 counts and 1 Z pulse.\r\n");

#elif APP_MODE == APP_M4A2_CALIB
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  printf("M4a-2: align + slow sweep. U=%.2f V, f=%.1f Hz, %d elec revs\r\n",
         (double)CAL_U_ALIGN, (double)CAL_F_ELEC, CAL_N_ELEC_REVS);
  printf("PWM starts in 3 s ...\r\n");
  HAL_Delay(3000);
  pwm_start_zero();

  /* ① 对齐。先拉到 90°，再拉到 0°：
   *    如果转子恰好停在和 0° 相差 180° 的位置，直接拉 0° 时两边力矩抵消，可能卡住不动；
   *    先去 90° 再回 0°，就不会落在那个不稳定点上。 */
  apply_udq(CAL_U_ALIGN, 0.0f, FOC_PI / 2.0f);
  HAL_Delay(500);
  apply_udq(CAL_U_ALIGN, 0.0f, 0.0f);
  HAL_Delay(1000);                               /* 等转子停稳 */

  enc_update();
  {
    uint32_t cnt0 = __HAL_TIM_GET_COUNTER(&htim4);
    printf("ALIGN theta_e=0  ->  enc cnt=%lu  pos=%ld\r\n",
           (unsigned long)cnt0, (long)g_enc_pos);
  }

#elif APP_MODE == APP_M3_CURRENT
  printf("M3: current sensing. U_dc=%.1f V, rotate %.1f V @ %.1f Hz, trip %.1f A\r\n",
         (double)M3_U_DC, (double)M3_U_ROT, (double)M3_F_ROT, (double)M3_I_TRIP);

  /* ① 运放、ADC 准备好；只启动 TIM1 计数器（不开输出），让 TRGO2 开始每周期触发 ADC */
  cs_init();
  __HAL_TIM_ENABLE(&htim1);
  HAL_Delay(100);

  /* ② 零点标定：MOS 全关，没有电流，此时的码值就是零点 */
  {
    bool ok = cs_calibrate_offset(4000);
    float c0[3]; cs_get_offset(c0);
    printf("[1] offset  code0 = %.1f / %.1f / %.1f   (nominal %u)  %s\r\n",
           (double)c0[0], (double)c0[1], (double)c0[2], CS_CODE0_NOMINAL, ok ? "OK" : "!! CHECK");
  }

  /* ③ 零点标定之后、仍然 MOS 全关：电流应该 ≈ 0，峰峰值就是采样噪声 */
  print_avg("[2] MOS off", 4000);

  /* ④ 开 PWM，零矢量：六管开关，但相间电压为零，电流仍应 ≈ 0 */
  pwm_start_zero();
  HAL_Delay(500);
  print_avg("[3] zero vec", 4000);

  /* ⑤ 直流矢量验符号：电压矢量依次指向 A、B、C 相轴（0°、120°、240°）。
   *    指向哪一相，电流就从那一相流进电机、从另外两相流出：
   *    该相电流应为正，另外两相各约为它的 -1/2，三相之和 ≈ 0 */
  {
    const float   th[3]  = { 0.0f, FOC_2PI / 3.0f, 2.0f * FOC_2PI / 3.0f };
    const char   *tag[3] = { "[4] DC -> A", "[5] DC -> B", "[6] DC -> C" };
    for (int k = 0; k < 3 && !g_fault; k++) {
      g_ctl_ud = M3_U_DC; g_ctl_uq = 0.0f; g_ctl_theta = th[k];
      g_ctl_mode = CTL_HOLD;
      HAL_Delay(400);                               /* 等转子转过去对齐、电流稳定 */
      print_avg(tag[k], 4000);
    }
    g_ctl_mode = CTL_IDLE;
    const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
    pwm_set_duty(&zero);
  }

  if (g_fault) {
    printf("!! OVERCURRENT TRIP, PWM off\r\n");
  } else {
    /* ⑥ 开环转动，主循环以 1 kHz 把三相电流按 VOFA+ FireWater 格式输出 */
    printf("[7] rotating. Switch VOFA+ engine to FireWater in 5 s ...\r\n");
    HAL_Delay(5000);
    g_ctl_ud = 0.0f; g_ctl_uq = M3_U_ROT; g_ctl_f = M3_F_ROT; g_ctl_theta = 0.0f;
    g_ctl_mode = CTL_ROTATE;
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if APP_MODE == APP_M2_OPENLOOP
    {
      static uint32_t last_ms = 0;
      static float    theta   = 0.0f;              /* 电角度 [rad] */
      uint32_t now = HAL_GetTick();

      if (now != last_ms) {                        /* 1 kHz 更新 */
        last_ms = now;

        theta += FOC_2PI * M2_F_ELEC * 0.001f;
        if (theta >= FOC_2PI) theta -= FOC_2PI;

        foc_sincos_t sc  = { sinf(theta), cosf(theta) };
        foc_dq_t     udq = { 0.0f, M2_U_AMP };     /* 纯 q 轴电压 → 旋转矢量 */
        foc_ab_t     uab;
        foc_abc_t    duty;

        foc_inv_park(&udq, &sc, &uab);
        foc_svpwm(&uab, M2_VDC, M2_DMAX, &duty);
        pwm_set_duty(&duty);
      }
    }
#elif APP_MODE == APP_M4A_ENCODER
    {
      static uint32_t last_print = 0;

      if (HAL_GetTick() - last_print >= 100) {     /* 每 100 ms 打印一行 */
        last_print += 100;

        static uint32_t last_cnt = 0;
        static int32_t  pos      = 0;              /* 上电以来的累计位置 [count]，可正可负 */

        uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim4);   /* 0 ~ 4095 */

        /* 计数器到 4095 会回到 0（或反过来），直接相减会差一整圈。
         * 100 ms 内手转不可能超过半圈，所以差值超过 ±2048 就说明跨过了回绕点，补回一圈。 */
        int32_t d = (int32_t)cnt - (int32_t)last_cnt;
        if (d >  2048) d -= 4096;
        if (d < -2048) d += 4096;
        pos     += d;
        last_cnt = cnt;

        printf("cnt=%4lu  pos=%+7ld  rev=%+.3f  z=%lu  cnt@z=%4lu\r\n",
               (unsigned long)cnt,
               (long)pos,
               (double)pos / 4096.0,
               (unsigned long)g_enc_z_count,
               (unsigned long)g_enc_cnt_at_z);
      }
    }
#elif APP_MODE == APP_M4A2_CALIB
    {
      static uint32_t last_ms   = 0;
      static float    theta     = 0.0f;
      static int      rev       = 0;               /* 已走完的电角度整圈数 */
      static int32_t  pos_start = 0;
      static int32_t  pos_prev  = 0;
      static int      done      = 0;
      uint32_t now = HAL_GetTick();

      if (!done && now != last_ms) {               /* 1 kHz */
        last_ms = now;
        enc_update();
        if (rev == 0 && theta == 0.0f) { pos_start = g_enc_pos; pos_prev = g_enc_pos; }

        theta += FOC_2PI * CAL_F_ELEC * 0.001f;
        if (theta >= FOC_2PI) {                    /* 走完一整圈电角度 */
          theta -= FOC_2PI;
          rev++;
          int32_t step = g_enc_pos - pos_prev;     /* 这一圈电角度对应的编码器计数 */
          pos_prev = g_enc_pos;
          printf("elec rev %2d: pos=%7ld  step=%+6ld  cnt=%4lu\r\n",
                 rev, (long)g_enc_pos, (long)step,
                 (unsigned long)__HAL_TIM_GET_COUNTER(&htim4));
        }
        apply_udq(CAL_U_ALIGN, 0.0f, theta);       /* d 轴电压：转子磁极正对 theta */

        if (rev >= CAL_N_ELEC_REVS) {
          done = 1;
          const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
          pwm_set_duty(&zero);                     /* 结束：回零矢量，电机不再受力 */

          float per_rev = (float)(g_enc_pos - pos_start) / (float)CAL_N_ELEC_REVS;
          printf("---- RESULT ----\r\n");
          printf("counts per elec rev = %.1f\r\n", (double)per_rev);
          printf("pole pairs          = %.3f  (4096 / |counts per elec rev|)\r\n",
                 (double)((float)ENC_CPR / fabsf(per_rev)));
          printf("direction           = %s\r\n",
                 per_rev > 0 ? "+1 (theta_e up -> count up)" : "-1 (theta_e up -> count DOWN)");
        }
      }
    }
#elif APP_MODE == APP_M3_CURRENT
    {
      static uint32_t last_ms = 0;
      static int      tripped_reported = 0;
      uint32_t now = HAL_GetTick();
      if (g_fault && !tripped_reported) { printf("!! OVERCURRENT TRIP, PWM off\r\n"); tripped_reported = 1; }
      if (g_ctl_mode == CTL_ROTATE && now != last_ms) {   /* 1 kHz 抽样输出，VOFA+ FireWater: "名字:v1,v2,...\n" */
        last_ms = now;
        float a = g_i_abc.a, b = g_i_abc.b, c = g_i_abc.c;
        printf("i:%.3f,%.3f,%.3f,%.3f\n", (double)a, (double)b, (double)c, (double)(a + b + c));
      }
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
/* PB8 (ENC_Z) 上升沿中断回调。编码器每转一圈，Z 相输出一个脉冲。
 * HAL 的 EXTI9_5_IRQHandler → HAL_GPIO_EXTI_IRQHandler → 这里。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ENC_Z_Pin) {
    g_enc_z_count++;
    g_enc_cnt_at_z = __HAL_TIM_GET_COUNTER(&htim4);   /* 记下 Z 到来时的位置：每圈应该几乎一样 */
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
