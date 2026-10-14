#!/bin/bash
# 项目 k 实机一键启动 (CUADC 伺服投弹复现台; 2026-10-13 新建)
#   用法 (Jetson): bash ~/fly/k.sh   [--no-build] [--no-log]
#   前置: 板载 k.py 与 see.py 同目录; 补偿当前**归零** (还没做过投放点真实测量)
#
#   2026-10-13 追加: **自动落盘日志** —— 首飞最关键的就是日志, 而 `ros2 run` 的输出
#   一关终端就没了。现在默认同时写 `~/fly/log/k_<时间戳>.log` 并 `tee` 到屏幕;
#   `--no-log` 可关掉 (纯前台)。
#   落地后把该日志拷回本地, 跑:
#     python3 scripts/analyze_k_flight_20261013.py k_<时间戳>.log [--ulog <x.ulg>]
set -e
cd "$(dirname "$0")"

NO_LOG=0
for a in "$@"; do
    [ "$a" = "--no-log" ] && NO_LOG=1
done

if [ "$1" != "--no-build" ]; then
    source /opt/ros/humble/setup.bash
    colcon build --packages-select fly 2>&1 | tail -5
fi
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ "$NO_LOG" = "1" ]; then
    exec ros2 run fly k
fi

LOG_DIR="$HOME/fly/log"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/k_$(date +%Y%m%d_%H%M%S).log"
echo "================================================================"
echo " 日志同时写入: $LOG"
echo " 降落后拷回本地跑:"
echo "   python3 scripts/analyze_k_flight_20261013.py $(basename "$LOG") [--ulog <x.ulg>]"
echo "================================================================"
exec ros2 run fly k 2>&1 | tee "$LOG"
