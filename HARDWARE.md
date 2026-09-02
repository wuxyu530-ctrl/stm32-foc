# B-G431B-ESC1 硬件常数与配置理由

> 目标板：**B-G431B-ESC1**（母板 MB1419-G431CBU6-**C01**，产品标识 `BG431BESC1$AU3`，硅片版本 X）
> MCU：**STM32G431CBU6**（UFQFPN48，128 KB Flash，32 KB SRAM，Cortex-M4F @170 MHz）
> CubeMX 工程：`port/stm32g431/stm32g431.ioc`

参考文档（均在项目根目录）：

| 简称 | 文件 |
|---|---|
| **UM2516** | `um2516-...-stm32g431cb-stmicroelectronics.pdf`（开发板用户手册） |
| **DS12589** | `stm32g431c6.pdf`（MCU 数据手册） |
| **RM0440** | `rm0440-stm32g4-series-...pdf`（G4 参考手册） |
| **原理图** | `mb1419-g431cbu6-c01_schematic.pdf` |
| **BOM** | `mb1419_bom/MB1419-G431CBU6-C01_BOM.xlsx` |

---

## 1. 硬件常数速查（写代码直接用）

### 1.1 电流采样链

| 参数 | 值 | 来源 |
|---|---|---|
| 采样电阻 $R_{shunt}$ | **0.003 Ω**（3 mΩ），3 W | 原理图 p.5，R54 / R55 / R56 |
| 运放增益 $G$ | **×16**（片内 PGA） | 软件选择，见 §3.4 |
| 输入网络分压 | $R_{th}=857\ \Omega$ | R64 22k / R80 2.2k / R67 1.5k |
| 零点偏置 | **0.1286 V**（在 VINP 上） | 见 §3.4 推导 |
| 信号衰减系数 | **0.5714** | 同上 |
| ADC 参考 $V_{ref}$ | **3.3 V**（外部，VREFBUF 关闭） | UM2516 表 4 第 20 脚 |
| 1 LSB | **805.7 µV** | 3.3 / 4096 |

**换算公式：**

```c
/* ADC 码值 → 相电流（安培） */
#define FOC_ADC_TO_AMP    0.029372f   /* A / LSB @ G=16, Rshunt=3mΩ, Vref=3.3V */

i_phase = (float)(adc_code - code0) * FOC_ADC_TO_AMP;   /* 符号需实测确认 */
```

- `code0` 理论值 ≈ **2553**，但**必须软件标定**（三相断电时连采 1000 次求平均）
  - 理由：运放失调 ±3 mV（全温全压，DS12589 表 74）→ 等效 **1.0 A** 的凭空误差
  - 失调温漂 ±10 µV/°C → 建议**每次启动电机前重新标定**，把温漂跟掉
- **符号**：按低边采样物理推导，相电流流出逆变器时 $V_{shunt}<0$，ADC 码值下降 → 代码里应取负号。**开环拖动时用示波器/波形实测确认**，别照抄。
- 可测量程（G=16）：**±45.3 A** 对称，正向可到 +75 A（不对称）
- 电流分辨率：**29.4 mA / LSB**

### 1.2 母线电压检测

| 参数 | 值 | 来源 |
|---|---|---|
| 分压 | R68 = 169 kΩ（上）/ R76 = 18 kΩ（下） | 原理图 p.6 |
| 分压比 | **0.09626** | 18 / 187 |
| 源阻抗 | **16.3 kΩ**（169k ∥ 18k） | —— |
| 可测上限 | **34.3 V** | 3.3 / 0.09626 |
| 引脚 / 通道 | PA0 = **ADC1_IN1** | —— |

```c
v_bus = (float)adc_code * 0.008370f;   /* V */
```

⚠️ **必须放规则组，不能放注入组**：源阻抗 16.3 kΩ，ADC 采样保持电容需 ≈734 ns 才能充满（12 位精度），至少要 **47.5 cycles** 采样时间。塞进注入组会把 000 零矢量窗口撑爆。
bring-up 阶段直接写死：`#define VDC_VOLT 12.0f`。

### 1.3 功率级

| 器件 | 型号 / 值 | 位号 |
|---|---|---|
| 功率 MOSFET | STL180N6F7（60 V / 120 A）×6 | Q2 ~ Q7 |
| 栅极驱动 | **L6387ED** ×3（自带上下管互锁） | U10 / U11 / U13 |
| 栅极串联电阻 | 33 Ω | R34/38/40/43/47/50 |
| 母线电容 | 15 µF ×多只 | C27~C40 |

### 1.4 其它外设网络

| 功能 | 网络 | 引脚 |
|---|---|---|
| 霍尔/编码器 | 1.8 kΩ 串联 + 10 kΩ 上拉到 3.3V + 10 pF + 钳位二极管 | PB6 / PB7 / PB8 |
| 反电动势检测 | 10 kΩ / 2.2 kΩ 分压，底端由 GPIO_BEMF 开关 | PA4 / PC4 / PB11（+ PB5 = GPIO_BEMF） |
| 温度（NTC） | RT1 10 kΩ NTC + R60 4.7 kΩ | PB14 |
| 电位器 | —— | PB12（= ADC1_IN11） |
| 晶振 | Y2 = 8 MHz | PF0 / PF1 |
| CAN | TCAN330 + 120 Ω 终端 | PA11(RX) / PB9(TX)，PC11=SHDN，PC14=TERM |

---

## 2. 引脚映射（UM2516 表 4，不可更改）

| 功能 | 引脚 | 备注 |
|---|---|---|
| TIM1_CH1 / CH2 / CH3（上桥） | PA8 / PA9 / PA10 | |
| TIM1_CH1N / CH2N / CH3N（下桥） | **PC13 / PA12 / PB15** | ⚠️ 非 CubeMX 默认，必须手动指定 |
| OPAMP1（相1） | VINP=PA1，VINM0=PA3 | 输出走内部通道 |
| OPAMP2（相2） | VINP=PA7，VINM0=PA5 | 同上 |
| OPAMP3（相3） | VINP=PB0，VINM0=PB2 | 同上 |
| 母线电压 | PA0 | ADC1_IN1 |
| SWD | PA13 / PA14 | |
| USART2 | PB3(TX) / PB4(RX) | 已引到排针 |
| 霍尔 / 编码器 | PB6 / PB7 / PB8 | TIM4_CH1/2/3 |
| 调试观测脚 | **PC11**（= TP2） | ⚠️ 焊盘为 DNF，需自行飞线 |

> **所有关键引脚已在 CubeMX 里做 Signal Pinning 钉住**（12 个）。
> 原因：CubeMX 会自动搬移未钉住的信号。曾发生过三个 OPAMP 的 VINP 连锁错位（PA1→PA7→PB0→PB13），
> 且 PB13 在板上是 N.C.，第三相电流会读到悬空噪声。**新增外设后务必复查引脚。**

---

## 3. CubeMX 配置理由（按重要性分层）

### 🔴 A 层：必须理解（不懂会烧硬件或静默失败）

#### 3.1 死区时间 `DeadTime = 149`（≈1000 ns）

- **填 0 = 上下管直通 = 母线短路 = MOS 瞬间炸。唯一会物理损坏硬件的参数。**
- DTG 编码是**分段非线性**的（RM0440 `TIMx_BDTR`），不能按比例换算：

  | DTG[7:5] | 公式 | 覆盖范围 @170 MHz |
  |---|---|---|
  | `0xx` | $DTG \times t_{DTS}$ | 0 ~ 747 ns（步长 5.88 ns） |
  | `10x` | $(64 + DTG_{[5:0]}) \times 2 t_{DTS}$ | 753 ~ 1494 ns |
  | `110` | $(32 + DTG_{[4:0]}) \times 8 t_{DTS}$ | 1506 ~ 2965 ns |
  | `111` | $(32 + DTG_{[4:0]}) \times 16 t_{DTS}$ | 3012 ~ 5929 ns |

  $t_{DTS} = 1/170\text{MHz} = 5.882$ ns（因为 `CKD = No Division`）

- 常用值：34→200ns，85→500ns，127→747ns，**149→1000ns**，166→1200ns
- **1 μs 是保守起步值。**最终值 = L6387ED 传播延迟失配 + STL180N6F7 关断时间 + 余量，**必须示波器实测上下管 Vgs**后再往下压。

#### 3.2 ADC 触发点必须落在 000 零矢量窗口内

三电阻低边采样，**只有三个下管同时导通时才有电流流过采样电阻**。中心对齐 + PWM mode 1 下，计数器**顶点**（CNT=ARR）正是 000 零矢量的正中心。

```
     000 窗口开启          触发         顶点        000 窗口关闭
   (CNT=maxCCR 上行)    (CNT=4150)   (CNT=4250)   (CNT=maxCCR 下行)
         │←── t_ring ──→│←─ 588ns ─→│
         └──────────────────── 2W ────────────────────┘
```

- `TIM1 TRGO2 = OC4REF`，CH4 用 **PWM mode 2** + `CCR4 = 4150` → 上升沿落在顶点前 **588 ns**
- CH4 选 **"PWM Generation No Output"**：TIM1_CH4 默认引脚是 PA11，而 PA11 是 CAN_RX
- 两条约束（$W$ = 窗口半宽）：
  - **A（避开振铃）**：$W \ge 588\ \text{ns} + t_{ring}$
  - **B（转换完成）**：$W \ge T_{conv} - 588\ \text{ns}$
  - **A 恒主导**，所以 ADC 快慢不影响最大占空比
- **最大占空比限幅**（`core/svpwm.c` 里必须做，做成可配参数）：

  | $t_{ring}$ 实测 | $\max CCR$ | $d_{max}$ |
  |---|---|---|
  | 300 ns | 4099 | 96.4% |
  | **500 ns（暂用）** | **4065** | **95.6%** |
  | 1000 ns | 3980 | 93.6% |

#### 3.3 引脚映射由板子决定，不可协商

见 §2。CubeMX 的默认分配**几乎总是错的**（它不知道你用的是哪块板）。

---

### 🟡 B 层：调试时自然会懂（现在照做即可）

#### 3.4 OPAMP：PGA 模式 ×16 —— 为什么不是 Standalone

**关键证据**：原理图里 `OP1_OUT` / `OP2_OUT` / `OP3_OUT` **只出现在 MCU 页**，从未进入采样电路页 → **板上没有外部反馈电阻** → Standalone 模式会开环（RM0440 图 171 脚注：*"the gain cannot be set by an external resistors"*）。

实际电路（原理图 p.6 `SHUNT SENSING CIRCUIT`）：

```
                      +3.3V
                        │
                     [R64 22k]
                        │
  Curr_fdbk1_OPAmp+ ────┼────[R67 1.5k]──── Vshunt_1+
     (PA1, VINP)        │
                     [R80 2.2k]
                        │
                       GND

  Curr_fdbk1_OPAmp- ─────────────────────── Vshunt_1-   （开尔文接法，抑制地弹）
     (PA3, VINM0)
```

对应 **RM0440 图 175 / 图 177**：*PGA mode, non-inverting gain with external bias*。
CubeMX 选项：**`PGA Internally connected_IO0_BIAS`**

戴维南（三源并联）：
$$V_{INP} = 0.1286\ \text{V} + 0.5714 \times V_{shunt}$$

$$V_{OUT} = G \times (0.1286 + 0.5714 \times I \times 0.003)$$

| PGA 增益 | 零点 | 灵敏度 | 对称量程 | 分辨率 |
|---|---|---|---|---|
| ×8 | 1.029 V | 13.7 mV/A | ±75 A | 58.7 mA/LSB |
| **×16** | **2.057 V** | **27.4 mV/A** | **±45.3 A** | **29.4 mA/LSB** |
| ×32 | 4.11 V | —— | **零点超量程，不可用** | |

→ **选 ×16**：可用档中分辨率最好；±45 A 与硬件能力匹配（3 mΩ / 3 W 采样电阻，持续 $\sqrt{3/0.003}=31.6$ A rms）。

**内部输出**（`OPAINTOEN=1`）：PGA 模式下反馈在片内，断开 VOUT 引脚无副作用。
→ PA2 / PA6 / PB1 释放，ADC 走**内部专用通道**，路径最短、抗干扰最好。

#### 3.5 TIM1 时基

| 参数 | 值 | 理由 |
|---|---|---|
| `CounterMode` | Center Aligned mode1 | 对称 PWM，000 窗口位于周期中心 |
| `Prescaler` | 0 | 吃满 170 MHz，死区与占空比分辨率最细 |
| **`ARR`** | **4250** | ⚠️ 中心对齐周期 = **2×ARR**（不是 2×(ARR+1)，RM0440 p.1160）→ $170\text{e}6/(2\times4250)$ = **20.000 kHz 整** |
| `RepetitionCounter` | 1 | 中心对齐下 RCR=0 会每半周期一次 UEV；=1 才是每周期一次 |
| `AutoReloadPreload` | Enable | |
| `CKD` | No Division | 死区分辨率最细 |
| CH1/2/3 | PWM mode 1，Pulse 0，极性 High/High，**Idle State Reset/Reset** | Idle=Reset → 故障时六管全关 |
| CH4 | PWM mode 2，Pulse **4150**，No Output | ADC 触发源，见 §3.2 |
| `Output compare preload` | Enable | **三相占空比在 UEV 原子性同时生效**；无此项会输出残缺脉冲 |

**1.5 拍延迟**（设计电流环时必须算进去）：

$$\varphi_{延迟} = 1.5 \times T_s \times \omega_{bw}$$

| 电流环带宽 | = PWM/ | 延迟相位滞后 | 剩余相位裕度 |
|---|---|---|---|
| 1000 Hz | 20 | 27.0° | 63.0° |
| **1500 Hz** | 13 | 40.5° | **49.5°** ← 推荐 |
| 2000 Hz | 10 | 54.0° | 36.0° |
| 3000 Hz | 6.7 | 81.0° | 9.0° ⚠️ |
| 4000 Hz | 5 | 108.0° | **−18° 发散** |

→ 这就是"电流环带宽取 PWM 频率 1/10~1/20"的**理论来源**。
→ **Python 仿真里必须建这个 1.5·Ts 延迟模型**（两级 buffer），否则仿真与实测对不上。

#### 3.6 电流环 PI 参数（可直接算，不用试凑）

被控对象 $G(s)=\dfrac{1}{Ls+R}$，用 **PI 零点对消电机极点**（令 $K_i/K_p = R/L$），开环塌缩为 $K_p/(Ls)$，闭环成为**一阶惯性环节**：

$$\boxed{K_p = L \cdot \omega_{bw}, \qquad K_i = R \cdot \omega_{bw}}$$

> 面试常问"电流环为什么可以近似成一阶惯性环节" —— 答案不是"因为它快"，而是**你用零极点对消把它设计成了一阶**。

#### 3.7 ADC

| 参数 | 值 | 理由 |
|---|---|---|
| 通道 | ADC1: `VOPAMP1`（IN13）<br>ADC2: `VOPAMP2`（IN16）+ `VOPAMP3`（IN18） | RM0440 表 204 的内部通道 |
| 转换组 | **注入组**（Injected），规则组 Disable | 独立触发源与数据寄存器，不占 DMA，确定性强 |
| 触发源 | `TIM1 TRGO2`，**Rising edge**（两个 ADC 同源） | 硬件同步启动 |
| **采样时间** | **12.5 cycles** | ⚠️ DS12589 表 74：读运放输出时 $T_{S\_OPAMP\_VOUT} \ge$ **200 ns**。6.5 cycles 只有 115 ns，**不合规** |
| `Clock Prescaler` | **Synchronous divided by 4**（42.5 MHz） | 12.5 cycles = 294 ns（余量 +47%）；与 TIM1 锁相**零抖动**；最大占空比与异步模式相同 |
| 中断 | **ADC2 的 JEOS**（ADC2 转 2 路，最后完成） | 回调里判断 `hadc->Instance == ADC2` |
| NVIC 优先级 | **抢占 0**（全系统最高） | 电流环晚一拍 = 控制失效 |

**⚠️ CubeMX 不生成、必须手写的两行**（在使能 ADC 之前）：

```c
HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
```

漏了它，后面所有零点标定都建在沙子上。

**中断读法**：

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2) return;   /* ADC2 最后完成 */
    DBG_PIN_HIGH();                       /* PC11，示波器看触发点位置 + ISR 耗时 */

    uint16_t raw1 = ADC1->JDR1;   /* 相1 —— 早已完成 */
    uint16_t raw2 = ADC2->JDR1;   /* 相2 */
    uint16_t raw3 = ADC2->JDR2;   /* 相3 */
    /* → 减 offset、乘系数 → foc_step() → 写 CCR */

    DBG_PIN_LOW();
}
```

#### 3.8 时钟树 170 MHz

| 项 | 值 | 约束（DS12589 表 45） |
|---|---|---|
| HSE | 8 MHz 晶振（PF0/PF1） | —— |
| PLLM | **/2** → 4 MHz | PLL 输入必须在 **2.66 ~ 16 MHz** |
| PLLN | **×85** → VCO 340 MHz | VCO 必须 ≤ **344 MHz** |
| PLLR | **/2** → **170 MHz** | Range1 Boost 下上限 170 MHz |
| AHB / APB1 / APB2 | 全 **/1** | **APB2 timer clocks = 170 MHz**（TIM1 吃这个，ARR=4250 按它算） |

**为什么 M 只能是 /2**：R 只能取 2/4/6/8 → VCO 必须 340（其余全超 344）→ VCO=输入×N=340。若 M=/1（输入 8 MHz），N 得是 42.5，非整数 → 排除。**M=/2 是唯一解。**

---

### ⚪ C 层：配对了就不用再想

| 项 | 值 | 一句话理由 |
|---|---|---|
| Power Regulator Voltage Scale | **Range 1 boost** | RM0440 表 9：Range1 Normal 上限只有 150 MHz |
| Flash Latency | **4 WS** | 同上表，Range1 Boost + ≤170 MHz |
| Prefetch Buffer | Enabled | 4 个等待周期下减少取指停顿 |
| HSE CSS | Enabled | 晶振停振时自动切 HSI + 触发 NMI（NMI 里应强制关 PWM） |
| VREFBUF | **Disable** | VREF+ 已在板上硬接 3.3V，开启会与电源轨对顶 |
| UCPD dead battery | **保持禁用**（勾选状态） | ⚠️ 见下 |
| OPAMP Power Mode | **High Speed** | 压摆率 2.5→18 V/µs（min），运放每周期都要重新建立 |
| GPIO 六路 PWM | Speed **Low** + **Pull-down** | PC13 受 3 mA / 2 MHz 限制（DS12589 p.60 脚注 2）；六路统一才对称；下拉在 MOE=0 时保证关断 |
| ⚠️ RCC / SYS 的黄色三角 | 忽略 | RCC = I2S_CKIN 因 PA12 被占；SYS = "无参数可配"，均无害 |

**UCPD dead battery 为什么绝对不能取消**（DS12589 p.60 脚注 6）：

| 引脚 | UCPD 角色 | 板上实际用途 |
|---|---|---|
| PA9 | UCPD1_DBCC1（控制端） | **TIM1_CH2 — B 相 PWM** |
| PA10 | UCPD1_DBCC2（控制端） | **TIM1_CH3 — C 相 PWM** |
| PB6 | UCPD1_CC1（被拉低端） | **霍尔 H1** |
| PB4 | UCPD1_CC2（被拉低端） | **USART2_RX** |

不禁用的话，**B 相 PWM 每次翻高就在霍尔信号上挂一个 5.1 kΩ 下拉，C 相 PWM 每次翻高就拉低串口 RX**。生成的代码里必须有 `HAL_PWREx_DisableUCPDDeadBattery();`。

---

## 4. 待实测参数（硬件到货后）

| 参数 | 现值 | 怎么测 |
|---|---|---|
| 开关振铃时间 $t_{ring}$ | 估 500 ns | 示波器看相电压，下管导通后到波形稳定 |
| ADC 触发点 `CCR4` | 4150（估） | PC11 在 ISR 里翻转，和三相 PWM 一起看 |
| 最大占空比 $d_{max}$ | 94%（保守） | 由 $t_{ring}$ 反算，见 §3.2 |
| 死区 | 149（1 μs） | 实测上下管 Vgs 重叠，逐步下压 |
| 电流零点 `code0` | 理论 2553 | 上电三相断电时采 1000 次平均 |
| 电流符号 | 待定 | 开环拖动看波形与电角度是否对应 |
| 电机 $L$、$R$ | —— | LCR 表或阶跃法辨识，用于算 PI 参数 |

---

## 5. 调试手段（这块板的限制）

⚠️ **DAC 输出观测方案在本板上不可用**：G4 的三个 DAC 输出引脚 PA4 / PA5 / PA6 分别被 BEMF1、OPAMP2_VINM0、OPAMP2_VOUT 占用（RM0440 p.715）。

替代方案：

| 手段 | 用途 | 引脚 |
|---|---|---|
| **GPIO 翻转 + 示波器** | 看 ADC 触发点位置、量 ISR 耗时 | **PC11**（TP2，焊盘 DNF 需飞线） |
| **RAM 环形缓冲 + 串口 dump** | 抓电流阶跃、θ 曲线等瞬态 | USART2 = PB3/PB4（已引到排针），建议 921600 |
| **STM32CubeMonitor** | 看慢变量（转速、母线电压、温度） | SWD |

---

## 6. 工程结构约定

```
3-phase-foc/
├── core/                 # 纯 C，零 HAL 依赖，可编成 .so 给 Python 仿真调用
│   ├── foc_types.h
│   ├── transform.c/h     # Clarke / Park / 逆变换
│   ├── svpwm.c/h         # 含 d_max 限幅（可配置，别写死）
│   ├── pid.c/h           # 带抗积分饱和
│   └── foc.c/h           # 电流环 step 函数
├── port/stm32g431/       # CubeMX 工程 + 硬件适配层
├── sim/                  # Python PMSM 模型 + ctypes 台架（必须建 1.5·Ts 延迟）
├── test/                 # 纯数学单元测试，PC 上跑
└── HARDWARE.md           # 本文件
```

核心接口只进出数字，不碰任何寄存器 —— 同一份 `.c` 同时跑在仿真和 MCU 上：

```c
typedef struct {
    float ia, ib, ic;      /* 输入：相电流 A */
    float theta_e;         /* 输入：电角度 rad */
    float vdc;             /* 输入：母线电压 V */
    float id_ref, iq_ref;  /* 输入：指令 A */
    float da, db, dc;      /* 输出：占空比 0~1 */
} foc_ctx_t;

void foc_step(foc_ctx_t *ctx);
```

**SVPWM 自验证**（不需要硬件，现在就能写）：遍历 θ∈[0,2π)、幅值 m，从占空比**反算** $U_\alpha,U_\beta$，断言与输入误差 < 1e-5；断言 $d_a+d_b+d_c$ 恒定（七段式特征）；断言 $0\le d\le 1$。这个反算能抓出扇区判断、作用时间分配、零矢量分配的**所有**错误。

---

## 7. 当前进度

**✅ 已完成（CubeMX）**
- 时钟树 170 MHz（HSE 8M → M2/N85/R2），Boost + 4WS + Prefetch + CSS
- TIM1：中心对齐、ARR 4250（20.000 kHz）、RCR 1、死区 149、TRGO2=OC4REF、CH4 PWM2/4150
- 六路 PWM 引脚 + Low speed + Pull-down
- OPAMP1/2/3：PGA `IO0_BIAS` 模式、×16、High Speed、内部输出
- ADC1（注入 1 路 VOPAMP1）+ ADC2（注入 2 路 VOPAMP2/VOPAMP3），12.5 cycles，TIM1_TRGO2 上升沿
- NVIC：`ADC1_2_IRQn` 抢占优先级 0
- 12 个关键引脚已 Signal Pinning

**⏳ 待办**
1. TIM4：霍尔（Hall Sensor Mode）或编码器（Encoder Mode），PB6/PB7/PB8
2. USART2：PB3/PB4，921600
3. PC11 配成 GPIO_Output（调试观测脚）
4. Project Manager：勾 "Generate peripheral initialization as a pair of .c/.h files"
5. 生成代码，验证 `SystemClock_Config()` 里有 boost + FLASH_LATENCY_4，`MX_OPAMP1_Init()` 六项参数，`HAL_PWREx_DisableUCPDDeadBattery()`
6. `git init`（`.gitignore` 排除 PDF）
7. 搭 `core/` 骨架 + SVPWM 单元测试（不需要硬件）
8. Python 仿真台架
