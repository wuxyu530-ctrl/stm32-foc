#!/usr/bin/env bash
# 通过 ST-LINK 硬件复位 MCU（不重新烧录），程序从头跑。
# 板上的 SW1 是用户按键，不接 NRST，所以按它不会复位。
# CLI 路径可用环境变量 STM32_CLI 覆盖。
CLI="${STM32_CLI:-/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe}"
"$CLI" -c port=SWD mode=HOTPLUG -hardRst 2>&1 | tr -d '\r' | grep -E "Error|reset|Reset|ST-LINK SN" || true
