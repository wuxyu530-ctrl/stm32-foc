#!/usr/bin/env bash
# 从 WSL 读取 Windows 串口（经 PowerShell），可选先复位 MCU，再录 N 秒输出打印到终端。
# 用法：tools/serial_capture.sh [秒数=12] [COM口=COM27] [波特率=921600] [--no-reset]
# 注意：串口同一时间只能被一个程序占用，运行前先断开 VOFA+ 等串口工具。
SECS="${1:-12}"; PORT="${2:-COM27}"; BAUD="${3:-921600}"; RESET=1
[ "${4:-}" = "--no-reset" ] && RESET=0
CLI='C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
powershell.exe -NoProfile -Command "
\$p = New-Object System.IO.Ports.SerialPort '$PORT',$BAUD,'None',8,'One'
\$p.ReadTimeout = 200
try { \$p.Open() } catch { Write-Output ('!! cannot open $PORT : ' + \$_.Exception.Message); exit 1 }
\$p.DiscardInBuffer()
if ($RESET) { & '$CLI' -c port=SWD mode=HOTPLUG -hardRst | Out-Null }
\$end = (Get-Date).AddSeconds($SECS)
while ((Get-Date) -lt \$end) { \$s = \$p.ReadExisting(); if (\$s) { [Console]::Write(\$s) }; Start-Sleep -Milliseconds 50 }
\$p.Close()
" | tr -d '\r'
