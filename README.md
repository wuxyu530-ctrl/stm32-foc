# stm32-foc

  从零实现的三相 PMSM 磁场定向控制（FOC）算法库，运行在 STM32G431 上。
  不依赖任何第三方电机库，Clarke/Park、SVPWM、电流环、观测器全部自己实现。

  ## 硬件平台
  - MCU: STM32G431CBU6 (Cortex-M4F @170MHz)
  - 开发板: ST B-G431B-ESC1 (MB1419-C01)
  - 功率级: STL180N6F7 ×6 + L6387ED ×3，3 mΩ 三电阻采样
  - 位置传感: MT6701 磁编码器，ABZ 1024 线

  ## 关键设计参数
  | 项 | 值 | 依据 |
  |---|---|---|
  | PWM 频率 | 20.000 kHz | 中心对齐，ARR=4250 @170MHz |
  | 电流采样 | 三电阻低边，TIM1_TRGO2 触发注入组 | 采样点位于 000 零矢量中心 |
  | 电流分辨率 | 29.4 mA/LSB | 3 mΩ × PGA×16，±45 A 量程 |
  | 死区 | 1.0 µs | 起步值，待实测优化 |
  | 目标电流环带宽 | 1.5 kHz | 受 1.5·Ts 采样延迟限制，相位裕度 49.5° |

  ## 目录结构
  core/   平台无关算法（纯 C，可编译到 PC 做仿真与单元测试）
  port/   STM32G431 硬件适配层 + CubeMX 工程
  sim/    Python 电机模型 + ctypes 台架（HIL 前置验证）
  test/   单元测试

  ## 进度
  - [x] 硬件配置与工程搭建
  - [ ] core 骨架 + Clarke/Park + SVPWM
  - [ ] Python 仿真台架
  - [ ] 开环 SVPWM 验证
  - [ ] 电流环闭合
  - [ ] 速度环
  - [ ] 无感观测器

  详细的硬件常数与配置推导见 [HARDWARE.md](HARDWARE.md)。
