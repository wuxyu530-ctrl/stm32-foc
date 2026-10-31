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
#include "foc_current.h"
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
#define APP_M4B_RES        5   /* 测相电阻：锁转子，多个直流电压点线性拟合 U = R·I + U0 */
#define APP_M4C_IND        6   /* 测电感：d 轴电压阶跃，重复叠加平均，输出电流波形供拟合 τ = L/R */
#define APP_M7_CURRENT     7   /* 电流闭环：对齐 → 角度自检 → d 轴电流阶跃叠加平均，和 Simulink 对比 */
#define APP_M8A_IQ_SPIN    8   /* q 轴电流测试：M7 的 ①～④ 之后给恒定 iq，电机加速再滑行，记录转速（为速度环测 ψf、J） */
#define APP_MODE           APP_M8A_IQ_SPIN

/* ---- M7 电流闭环参数【Claude 代写，待复核】见 docs/M7_电流闭环.md ---- */
#define MOTOR_R            3.21f      /* 相电阻 [Ω]，M4b */
#define MOTOR_L            0.88e-3f   /* 相电感 [H]，M4c */
#define MOTOR_POLE_PAIRS   7          /* 极对数，M4a-2 实测 7.004 */
#define ENC_DIR            (-1)       /* M4a-2：θe 增大时编码器计数减小 */
#define M7_FBW             1000.0f    /* 电流环带宽 [Hz]：Kp = L·2π·FBW，Ki = R·2π·FBW */
#define M7_U_ALIGN         0.8f       /* 对齐与角度自检用的 d 轴电压 [V]（开环） */
#define M7_ID_LO           0.2f       /* 阶跃前 d 轴电流目标 [A]：保持转子对齐 */
#define M7_ID_HI           1.0f       /* 阶跃后 [A]。需要电压 R·1 A ≈ 3.2 V，远低于 v_max ≈ 6.3 V */
#define M7_PERIOD          200        /* 一个高低周期的 PWM 周期数：200 × 50 µs = 10 ms */
#define M7_STEP_AT         100        /* 第 100 个周期目标从 LO 跳到 HI */
#define M7_PRE             10         /* 记录阶跃前 10 个点作基线 */
#define M7_LEN             60         /* 共记录 60 个点（3 ms） */
#define M7_REPEAT          64         /* 叠加次数：噪声降为 1/8 */

/* ---- M8a q 轴电流测试参数【Claude 代写，待复核】 ---- */
#define M8_IQ              0.10f      /* q 轴电流目标 [A]：产生转矩，id = 0 */
#define M8_T_RUN_MS        1500       /* 给 iq 的时间 [ms]，之后 iq = 0 惯性滑行 */
#define M8_T_COAST_MS      2000       /* 滑行记录时间 [ms] */
#define M8_LOG_MS          2          /* 每 2 ms 打印一行 */
#define M8_RPM_WIN         5          /* 转速 = 最近 5 次记录（约 10 ms）的位置差 / 时间差 */
#define M8_RPM_MAX         1500.0f    /* 超过这个转速把 iq 置 0。run1 在 2350 rpm（vq 接近 v_max）时过流跳闸，先避开高调制区 */

/* M3 电流采样参数 */
#define M3_U_DC            1.5f   /* 直流矢量测试电压 [V] */
#define M3_U_ROT           2.0f   /* 开环转动电压 [V] */
#define M3_F_ROT           5.0f   /* 开环转动电频率 [Hz] */
#define M3_I_TRIP          2.0f   /* 软件过流保护阈值 [A]：任一相超过即关 PWM */
#define PWM_HZ             20000.0f

/* M4b 测电阻参数 */
#define M4B_N_PTS          4
static const float M4B_U[M4B_N_PTS] = { 0.6f, 1.0f, 1.4f, 1.8f };   /* d 轴电压 [V] */

/* M4c 测电感参数 */
#define M4C_U_LO           0.4f   /* 阶跃前 d 轴电压 [V]：保持转子对齐 */
#define M4C_U_HI           1.8f   /* 阶跃后 [V] */
#define M4C_PERIOD         400    /* 一个高低周期的 PWM 周期数：400 × 50 µs = 20 ms */
#define M4C_STEP_AT        200    /* 第 200 个周期电压从 LO 跳到 HI */
#define M4C_PRE            10     /* 记录阶跃前 10 个点作基线 */
#define M4C_LEN            190    /* 共记录 190 个点（9.5 ms） */
#define M4C_REPEAT         128    /* 重复叠加次数：噪声降为 1/sqrt(128) ≈ 1/11 */

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
typedef enum { CTL_IDLE = 0, CTL_HOLD, CTL_ROTATE, CTL_CURRENT } ctl_mode_t;
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

/* M4c 阶跃叠加 */
static volatile int        g_step_run  = 0;         /* 1：中断里执行阶跃并累加 */
static volatile uint32_t   g_step_n    = 0;         /* 当前在周期内的第几个 PWM 周期 */
static volatile uint32_t   g_step_rep  = 0;         /* 已完成的重复次数 */
static float               g_step_sum[M4C_LEN];     /* 每个采样点 id 的累加和 */

/* ---- M7 电流闭环【Claude 代写，待复核】 ---- */
static foc_cur_t           g_cur;                   /* dq 电流环（两个 PI + 限幅），只在中断里调用 step */
static volatile uint32_t   g_enc_cnt0  = 0;         /* 对齐到 θe = 0 时的编码器计数，每次上电重新测 */
static volatile float      g_id_ref    = 0.0f;      /* CTL_CURRENT 的 dq 电流目标 [A] */
static volatile float      g_iq_ref    = 0.0f;
static volatile float      g_theta_e   = 0.0f;      /* 中断里最近一次由编码器算出的电角度 [rad] */
static volatile foc_cur_out_t g_cur_out;            /* 中断里最近一次的 id、iq、vd、vq */
static volatile int        g_m7_run    = 0;         /* 1：中断里执行 d 轴电流阶跃并叠加记录 */
static volatile uint32_t   g_m7_n      = 0;         /* 当前在高低周期内的第几个 PWM 周期 */
static volatile uint32_t   g_m7_rep    = 0;         /* 已完成的重复次数 */
static float               g_m7_id[M7_LEN];         /* 每个采样点 id 的累加和 */
static float               g_m7_iq[M7_LEN];         /* iq 的累加和（应始终 ≈ 0） */
static float               g_m7_vd[M7_LEN];         /* vd 指令的累加和（对照 Simulink 的电压台阶） */
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

/* 由编码器计数算电角度 θe ∈ [0, 2π)【Claude 代写，待复核】
 *   θe = 2π · frac( ENC_DIR · 极对数 · (cnt − cnt0) / 4096 )
 * cnt0 是对齐到 θe = 0 时的计数；frac() 取小数部分，把多圈电角度折回一圈。 */
static float theta_e_from_encoder(void)
{
  int32_t d = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - (int32_t)g_enc_cnt0;   /* −4095 ~ +4095 */
  float   e = (float)(ENC_DIR * MOTOR_POLE_PAIRS * d) / (float)ENC_CPR;        /* 电角度圈数，可为负 */
  e -= floorf(e);                                                              /* 取小数部分 → [0, 1) */
  return FOC_2PI * e;
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

  if (g_step_run) {
    /* θ = 0 时 d 轴就是 α 轴：id = iα = (2/3)·(ia − (ib + ic)/2) */
    float id = FOC_2_3 * (i->a - 0.5f * (i->b + i->c));
    uint32_t n = g_step_n;
    if (n >= M4C_STEP_AT - M4C_PRE && n < M4C_STEP_AT - M4C_PRE + M4C_LEN)
      g_step_sum[n - (M4C_STEP_AT - M4C_PRE)] += id;
    /* 这一周期算出的占空比要等下一个更新事件才生效，所以第 n 个样本看到的是第 n-1 次写入的电压 */
    g_ctl_ud = (n < M4C_STEP_AT) ? M4C_U_LO : M4C_U_HI;
    if (++n >= M4C_PERIOD) {
      n = 0;
      if (++g_step_rep >= M4C_REPEAT) { g_step_run = 0; g_ctl_ud = M4C_U_LO; }
    }
    g_step_n = n;
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
    case CTL_CURRENT: {                              /* M7 电流闭环【Claude 代写，待复核】 */
      float th = theta_e_from_encoder();
      g_theta_e = th;
      foc_sincos_t sc;
      foc_sincos(th, &sc);

      /* d 轴电流阶跃：目标在 LO/HI 之间切换，并按"相对阶跃时刻的位置"叠加记录 */
      float id_ref = g_id_ref;
      uint32_t n = g_m7_n;
      if (g_m7_run) id_ref = (n < M7_STEP_AT) ? M7_ID_LO : M7_ID_HI;

      foc_abc_t     duty;
      foc_cur_out_t o;
      foc_cur_step(&g_cur, i, &sc, id_ref, g_iq_ref, &duty, &o);
      pwm_set_duty(&duty);                           /* 下一个更新事件生效（约半个周期后） */
      g_cur_out.id = o.id; g_cur_out.iq = o.iq; g_cur_out.vd = o.vd; g_cur_out.vq = o.vq;

      if (g_m7_run) {
        /* 第 n 个样本里：o.id 是本次采到的电流，o.vd 是本次算出、下个周期才生效的电压 */
        if (n >= M7_STEP_AT - M7_PRE && n < M7_STEP_AT - M7_PRE + M7_LEN) {
          uint32_t k = n - (M7_STEP_AT - M7_PRE);
          g_m7_id[k] += o.id;
          g_m7_iq[k] += o.iq;
          g_m7_vd[k] += o.vd;
        }
        if (++n >= M7_PERIOD) {
          n = 0;
          if (++g_m7_rep >= M7_REPEAT) g_m7_run = 0;   /* 结束后目标回到 g_id_ref */
        }
        g_m7_n = n;
      }
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

/* 把三相电流投影到电角度 theta 方向（即 Park 变换的 d 分量）*/
static float i_along(const float abc[3], float theta)
{
  foc_abc_t    i  = { abc[0], abc[1], abc[2] };
  foc_ab_t     ab;
  foc_sincos_t sc = { sinf(theta), cosf(theta) };
  foc_dq_t     dq;
  foc_clarke(&i, &ab);
  foc_park(&ab, &sc, &dq);
  return dq.d;
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

#elif APP_MODE == APP_M4B_RES
  printf("M4b: phase resistance by multi-point fit  U = R*I + U0\r\n");
  cs_init();
  __HAL_TIM_ENABLE(&htim1);
  HAL_Delay(100);
  if (!cs_calibrate_offset(4000)) printf("!! offset calibration suspicious\r\n");
  pwm_start_zero();
  HAL_Delay(300);

  {
    const float  th[3]  = { 0.0f, FOC_2PI / 3.0f, 2.0f * FOC_2PI / 3.0f };
    const char   axn[3] = { 'A', 'B', 'C' };
    float        r_axis[3] = { 0 };

    for (int k = 0; k < 3 && !g_fault; k++) {
      float sx = 0, sy = 0, sxx = 0, sxy = 0;      /* 最小二乘：x = 电流 I，y = 电压 U */
      printf("-- axis %c --\r\n", axn[k]);

      /* 先用最小电压把转子拉到这个方向，停稳 */
      g_ctl_ud = M4B_U[0]; g_ctl_uq = 0.0f; g_ctl_theta = th[k];
      g_ctl_mode = CTL_HOLD;
      HAL_Delay(600);

      for (int n = 0; n < M4B_N_PTS && !g_fault; n++) {
        g_ctl_ud = M4B_U[n];
        HAL_Delay(300);                            /* 电流在 L/R 的几倍时间内稳定，300 ms 足够 */
        float abc[3];
        avg_currents(8000, abc, NULL);             /* 0.4 s 平均，压掉 ±4 LSB 的单点噪声 */
        float id = i_along(abc, th[k]);
        printf("   U=%.2f V   I=%.4f A   (ia %+.3f ib %+.3f ic %+.3f)\r\n",
               (double)M4B_U[n], (double)id, (double)abc[0], (double)abc[1], (double)abc[2]);
        sx += id; sy += M4B_U[n]; sxx += id * id; sxy += id * M4B_U[n];
      }
      float N  = (float)M4B_N_PTS;
      float R  = (N * sxy - sx * sy) / (N * sxx - sx * sx);   /* 斜率 */
      float U0 = (sy - R * sx) / N;                           /* 截距 */
      r_axis[k] = R;
      printf("   => R = %.3f ohm,  U0 = %.3f V (dead-time etc.)\r\n", (double)R, (double)U0);
    }
    g_ctl_mode = CTL_IDLE;
    const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
    pwm_set_duty(&zero);

    if (g_fault) printf("!! OVERCURRENT TRIP\r\n");
    else printf("---- RESULT ----\r\nR_phase = %.3f ohm  (A %.3f / B %.3f / C %.3f)\r\n",
                (double)((r_axis[0] + r_axis[1] + r_axis[2]) / 3.0f),
                (double)r_axis[0], (double)r_axis[1], (double)r_axis[2]);
  }

#elif APP_MODE == APP_M4C_IND
  printf("M4c: inductance by d-axis voltage step %.2f -> %.2f V, %d repeats\r\n",
         (double)M4C_U_LO, (double)M4C_U_HI, M4C_REPEAT);
  cs_init();
  __HAL_TIM_ENABLE(&htim1);
  HAL_Delay(100);
  {
    uint32_t c0 = cs_isr_count();                  /* 自检：采样回调应正好每秒 20000 次 */
    HAL_Delay(1000);
    printf("current-sense ISR rate = %lu /s (expect 20000)\r\n", (unsigned long)(cs_isr_count() - c0));
  }
  if (!cs_calibrate_offset(4000)) printf("!! offset calibration suspicious\r\n");
  pwm_start_zero();
  HAL_Delay(300);

  /* 转子对齐到 θ = 0：先 90° 再 0°，避开 180° 不稳定点 */
  g_ctl_ud = M4C_U_LO; g_ctl_uq = 0.0f; g_ctl_theta = FOC_PI / 2.0f; g_ctl_mode = CTL_HOLD;
  HAL_Delay(400);
  g_ctl_theta = 0.0f;
  HAL_Delay(800);

  for (int k = 0; k < M4C_LEN; k++) g_step_sum[k] = 0.0f;
  g_step_n = 0; g_step_rep = 0; g_step_run = 1;
  while (g_step_run && !g_fault) { }               /* 128 × 20 ms ≈ 2.6 s */

  g_ctl_mode = CTL_IDLE;
  {
    const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
    pwm_set_duty(&zero);
  }
  if (g_fault) {
    printf("!! OVERCURRENT TRIP\r\n");
  } else {
    printf("---- STEP (k, t_us, id_A) ; step written at k=%d ----\r\n", M4C_PRE);
    for (int k = 0; k < M4C_LEN; k++)
      printf("%d,%d,%.5f\r\n", k, (k - M4C_PRE) * 50, (double)(g_step_sum[k] / (float)M4C_REPEAT));
    printf("---- END ----\r\n");
  }

#elif APP_MODE == APP_M7_CURRENT || APP_MODE == APP_M8A_IQ_SPIN
  /* 【Claude 代写，待复核】流程与预期结果见 docs/M7_电流闭环.md。M8a 共用 ①～④，从 ⑤ 开始不同 */
  {
    const float wbw = FOC_2PI * M7_FBW;
    const float kp  = MOTOR_L * wbw;               /* ≈ 5.53 V/A */
    const float ki  = MOTOR_R * wbw;               /* ≈ 20169 V/(A·s) */
    printf("M7: current loop. Kp=%.3f V/A Ki=%.0f V/(A*s), id step %.2f -> %.2f A, %d repeats\r\n",
           (double)kp, (double)ki, (double)M7_ID_LO, (double)M7_ID_HI, M7_REPEAT);
#if APP_MODE == APP_M8A_IQ_SPIN
    printf("M8a: iq spin test. iq=%.2f A for %d ms, then iq=0 coast %d ms\r\n",
           (double)M8_IQ, M8_T_RUN_MS, M8_T_COAST_MS);
#endif

    /* ① 电流采样准备：与 M3/M4 相同 */
    cs_init();
    __HAL_TIM_ENABLE(&htim1);
    HAL_Delay(100);
    {
      uint32_t c0 = cs_isr_count();
      HAL_Delay(1000);
      printf("[1] ISR rate = %lu /s (expect 20000)\r\n", (unsigned long)(cs_isr_count() - c0));
    }
    if (!cs_calibrate_offset(4000)) printf("!! offset calibration suspicious\r\n");
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    pwm_start_zero();
    HAL_Delay(300);

    /* ② 开环对齐到 θe = 0（先 90° 再 0°，避开 180° 不稳定点），记下此刻的编码器计数 */
    g_ctl_ud = M7_U_ALIGN; g_ctl_uq = 0.0f; g_ctl_theta = FOC_PI / 2.0f; g_ctl_mode = CTL_HOLD;
    HAL_Delay(400);
    g_ctl_theta = 0.0f;
    HAL_Delay(800);
    g_enc_cnt0 = __HAL_TIM_GET_COUNTER(&htim4);
    printf("[2] aligned: enc cnt0 = %lu\r\n", (unsigned long)g_enc_cnt0);

    /* ③ 角度自检：开环把转子依次拉到 90°、180°、270°、0°，比较"指令角"和"编码器算出的角"。
     *    两者应相差几度以内。若差很多或方向相反，说明极对数 / ENC_DIR / cnt0 有问题，不要进入闭环。 */
    {
      const float deg[4] = { 90.0f, 180.0f, 270.0f, 0.0f };
      int ok = 1;
      for (int k = 0; k < 4 && !g_fault; k++) {
        g_ctl_theta = deg[k] * FOC_PI / 180.0f;
        HAL_Delay(500);
        float th  = theta_e_from_encoder() * 180.0f / FOC_PI;
        float err = th - deg[k];
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        if (fabsf(err) > 20.0f) ok = 0;
        printf("[3] cmd %5.1f deg  enc %5.1f deg  err %+5.1f deg\r\n", (double)deg[k], (double)th, (double)err);
      }
      if (!ok) {
        g_ctl_mode = CTL_IDLE;
        const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
        pwm_set_duty(&zero);
        printf("!! angle check FAILED: check MOTOR_POLE_PAIRS / ENC_DIR. Closed loop NOT started.\r\n");
        while (1) { }
      }
    }

    /* ④ 切到电流闭环：先清积分，再一次性切换模式。保持 id = ID_LO、iq = 0 半秒，打印稳态 */
    foc_cur_init(&g_cur, kp, ki, 1.0f / PWM_HZ, M2_VDC, M2_DMAX);
    foc_cur_reset(&g_cur);
    g_id_ref = M7_ID_LO; g_iq_ref = 0.0f;
    g_ctl_mode = CTL_CURRENT;                       /* 中断下一次就按电流闭环运行 */
    HAL_Delay(500);
    printf("[4] closed loop hold: theta_e=%.1f deg  id=%.3f A  iq=%.3f A  vd=%.3f V  vq=%.3f V  (v_max %.2f V)\r\n",
           (double)(g_theta_e * 180.0f / FOC_PI), (double)g_cur_out.id, (double)g_cur_out.iq,
           (double)g_cur_out.vd, (double)g_cur_out.vq, (double)g_cur.v_max);

#if APP_MODE == APP_M8A_IQ_SPIN
    /* ⑤ q 轴电流测试【Claude 代写，待复核】
     *    id = 0、iq = M8_IQ：电流矢量始终超前转子 90°，电机加速；M8_T_RUN_MS 后 iq = 0，
     *    电流环仍在运行（id = iq = 0，不产生转矩也不刹车），电机靠惯性滑行、被摩擦减速。
     *    转速：主循环每 1 ms 读一次编码器展开成累计位置 g_enc_pos，取约 10 ms 的位置差。
     *    正转速 = θe 增大的方向（编码器计数减小，所以乘 ENC_DIR）。 */
    {
      int32_t  ring_pos[M8_RPM_WIN];
      uint32_t ring_t[M8_RPM_WIN];
      int      ri = 0;
      enc_update(); enc_update();                   /* 第一次调用只记起点 */
      uint32_t t0 = HAL_GetTick(), last_tick = t0, last_log = t0;
      for (int k = 0; k < M8_RPM_WIN; k++) { ring_pos[k] = g_enc_pos; ring_t[k] = t0; }

      printf("---- M8A IQ SPIN (t_ms, pos_cnt, rpm, id_A, iq_A, vd_V, vq_V) ----\r\n");
      g_id_ref = 0.0f; g_iq_ref = M8_IQ;            /* 两个目标同时改：中断里先读哪个都无所谓 */
      while (!g_fault) {
        uint32_t now = HAL_GetTick();
        if (now == last_tick) continue;
        last_tick = now;
        enc_update();
        uint32_t t = now - t0;
        if (t >= M8_T_RUN_MS) g_iq_ref = 0.0f;
        if (now - last_log >= M8_LOG_MS) {
          last_log = now;
          int      old = ri;                          /* 环形缓冲里最老的一条 */
          float    dt  = (float)(now - ring_t[old]) * 1e-3f;
          float    rpm = (dt > 0.0f)
                       ? (float)(ENC_DIR * (g_enc_pos - ring_pos[old])) / (float)ENC_CPR / dt * 60.0f
                       : 0.0f;
          ring_pos[ri] = g_enc_pos; ring_t[ri] = now; ri = (ri + 1) % M8_RPM_WIN;
          if (fabsf(rpm) > M8_RPM_MAX) g_iq_ref = 0.0f;
          printf("%lu,%ld,%.1f,%.3f,%.3f,%.3f,%.3f\r\n", (unsigned long)t, (long)g_enc_pos, (double)rpm,
                 (double)g_cur_out.id, (double)g_cur_out.iq, (double)g_cur_out.vd, (double)g_cur_out.vq);
        }
        if (t >= M8_T_RUN_MS + M8_T_COAST_MS) break;
      }
    }

    /* ⑥ 结束：此时电机应已基本停下，再回零矢量（零矢量会把三相短接，转得快时会刹车） */
    g_ctl_mode = CTL_IDLE;
    {
      const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
      pwm_set_duty(&zero);
    }
    if (g_fault) printf("!! OVERCURRENT TRIP\r\n");
    else         printf("---- END ----\r\n");
#else
    /* ⑤ d 轴电流阶跃 LO → HI，重复叠加 */
    for (int k = 0; k < M7_LEN; k++) { g_m7_id[k] = 0.0f; g_m7_iq[k] = 0.0f; g_m7_vd[k] = 0.0f; }
    g_m7_n = 0; g_m7_rep = 0; g_m7_run = 1;
    while (g_m7_run && !g_fault) { }               /* 64 × 10 ms ≈ 0.64 s */

    /* ⑥ 结束：关闭闭环，回零矢量 */
    g_ctl_mode = CTL_IDLE;
    {
      const foc_abc_t zero = { 0.5f, 0.5f, 0.5f };
      pwm_set_duty(&zero);
    }
    if (g_fault) {
      printf("!! OVERCURRENT TRIP\r\n");
    } else {
      printf("---- M7 STEP (k, t_us, id_ref, id_A, iq_A, vd_V) ; id_ref steps at k=%d ----\r\n", M7_PRE);
      for (int k = 0; k < M7_LEN; k++) {
        float ref = (k < M7_PRE) ? M7_ID_LO : M7_ID_HI;
        printf("%d,%d,%.2f,%.5f,%.5f,%.4f\r\n", k, (k - M7_PRE) * 50, (double)ref,
               (double)(g_m7_id[k] / (float)M7_REPEAT), (double)(g_m7_iq[k] / (float)M7_REPEAT),
               (double)(g_m7_vd[k] / (float)M7_REPEAT));
      }
      printf("---- END ----\r\n");
    }
#endif  /* APP_M8A_IQ_SPIN / APP_M7_CURRENT */
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
