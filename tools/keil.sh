#!/usr/bin/env bash
# 从 WSL 里调用 Keil 命令行，编译 / 烧录 port/stm32g431 工程。
#
# 用法：tools/keil.sh build     增量编译（= Keil 里的 F7）
#       tools/keil.sh rebuild   全部重编（= Rebuild all）
#       tools/keil.sh flash     编译并下载到板子（= Load）
#
# 注意：Keil GUI 里如果同时开着这个工程，命令行编译会和它抢文件锁。
#       用本脚本时把 Keil GUI 关掉，或者只在 GUI 里调试、不在 GUI 里编译。
#
# UV4.exe 路径可用环境变量 KEIL_UV4 覆盖。
set -u

UV4="${KEIL_UV4:-/mnt/c/Users/22101/AppData/Local/Keil_v5/UV4/UV4.exe}"
PROJ_DIR="$(cd "$(dirname "$0")/../port/stm32g431/MDK-ARM" && pwd)"
PROJ="stm32g431.uvprojx"
MODE="${1:-build}"
LOG="uv4_${MODE}.log"          # UV4 的 -o 是相对工程文件目录的

case "$MODE" in
    build)   FLAG=-b ;;
    rebuild) FLAG=-r ;;
    flash)   FLAG=-f ;;
    *) echo "用法: $0 build|rebuild|flash" >&2; exit 2 ;;
esac

[ -x "$UV4" ] || { echo "找不到 UV4.exe: $UV4" >&2; exit 2; }

cd "$PROJ_DIR"
rm -f "$LOG"
"$UV4" $FLAG "$(wslpath -w "$PROJ_DIR/$PROJ")" -j0 -o "$LOG"
RC=$?

# Keil 的日志是 CRLF，去掉 \r 再打印，VS Code 的 problemMatcher 才能匹配
[ -f "$LOG" ] && tr -d '\r' < "$LOG"

# UV4 退出码：0 无错误无警告  1 有警告  2 有错误  3 致命错误  11 打不开工程  20 找不到调试器
case $RC in
    0) echo "== $MODE 完成，0 警告 ==" ;;
    1) echo "== $MODE 完成，有警告 ==" ;;
    2) echo "== $MODE 失败：有错误 ==" ;;
    11) echo "== 打不开工程（Keil GUI 里是不是开着同一个工程？） ==" ;;
    20) echo "== 找不到调试器 / 板子没插 USB ==" ;;
    *) echo "== UV4 退出码 $RC ==" ;;
esac
[ $RC -le 1 ]
