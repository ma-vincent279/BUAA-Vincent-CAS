#!/bin/bash
# ============================================================================
# DDS + see (多桶检测) + final_mission (多桶投放) 一键启动脚本
# 用法: ./launch_final.sh [--log]
#   --log: 将各节点输出记录到 ~/fly_logs/ 目录
#
# Ctrl+C 停止所有后台进程。final_mission 异常退出时自动清理其余进程。
# ============================================================================
set -e

# ---- 路径 & 环境 ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DDS_AGENT="MicroXRCEAgent"
DDS_DEV="/dev/ttyTHS1"
DDS_BAUD="921600"
CONDA_ENV="yolov8"
CONDA_PYTHONPATH="$HOME/miniforge3/envs/yolov8/lib/python3.10/site-packages"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# ---- 日志目录 ----
LOG_DIR="$HOME/fly_logs"
if [[ "$1" == "--log" ]]; then
    mkdir -p "$LOG_DIR"
    LOG_DDS="$LOG_DIR/dds_$TIMESTAMP.log"
    LOG_SEE="$LOG_DIR/see_$TIMESTAMP.log"
    LOG_MISSION="$LOG_DIR/final_mission_$TIMESTAMP.log"
    echo "[$(date)] Logging to $LOG_DIR/"
fi

# ---- 清理函数 ----
_CLEANED=false
cleanup() {
    if $_CLEANED; then return; fi
    _CLEANED=true
    echo ""
    echo "=== 正在停止所有后台进程... ==="
    [[ -n "$WATCHDOG_PID" ]] && kill "$WATCHDOG_PID" 2>/dev/null && echo "  watchdog ($WATCHDOG_PID) 已终止"
    [[ -n "$SEE_PID" ]] && kill "$SEE_PID" 2>/dev/null && echo "  see ($SEE_PID) 已终止"
    [[ -n "$DDS_PID" ]] && kill "$DDS_PID" 2>/dev/null && echo "  dds  ($DDS_PID) 已终止"
    [[ -n "$MISSION_PID" ]] && kill "$MISSION_PID" 2>/dev/null && echo "  final_mission ($MISSION_PID) 已终止"
    echo "=== 清理完成 ==="
}

trap cleanup INT TERM EXIT

# ---- 后台进程存活监控 ----
watchdog() {
    while true; do
        if ! kill -0 "$SEE_PID" 2>/dev/null; then
            echo "[FATAL] see 节点意外退出!" >&2
            kill "$$" 2>/dev/null
            exit 1
        fi
        if ! kill -0 "$DDS_PID" 2>/dev/null; then
            echo "[FATAL] DDS Agent 意外退出!" >&2
            kill "$$" 2>/dev/null
            exit 1
        fi
        sleep 2
    done
}

# ---- 检查前置条件 ----
echo "=== 检查环境 ==="

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[FATAL] ROS 2 Humble 未安装在 /opt/ros/humble"
    exit 1
fi

if ! command -v "$DDS_AGENT" &>/dev/null; then
    echo "[FATAL] DDS Agent 未找到: $DDS_AGENT"
    exit 1
fi

if [ ! -e "$DDS_DEV" ]; then
    echo "[FATAL] DDS 串口设备不存在: $DDS_DEV"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "[FATAL] 工作空间未编译, 请先运行 colcon build"
    exit 1
fi

if ! command -v conda &>/dev/null; then
    echo "[FATAL] conda 未安装或不在 PATH 中"
    exit 1
fi

echo "  ROS 2 Humble       : OK"
echo "  DDS Agent          : $DDS_AGENT"
echo "  DDS Device         : $DDS_DEV @ $DDS_BAUD baud"
echo "  Conda env          : $CONDA_ENV"
echo "  Workspace          : $SCRIPT_DIR"

# ---- Source ROS 2 & workspace ----
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ---- 1. 启动 DDS Agent ----
echo ""
echo "=== 1/3 启动 DDS Agent ==="

EXISTING=$(pgrep -f "MicroXRCEAgent serial" 2>/dev/null || true)
if [[ -n "$EXISTING" ]]; then
    echo "  ⚠ 检测到已有 DDS Agent 在运行 (PID: $EXISTING), 先终止..."
    kill $EXISTING 2>/dev/null
    sleep 1
fi

if [[ -n "$LOG_DDS" ]]; then
    sg dialout -c "$DDS_AGENT serial --dev $DDS_DEV -b $DDS_BAUD" > "$LOG_DDS" 2>&1 &
else
    sg dialout -c "$DDS_AGENT serial --dev $DDS_DEV -b $DDS_BAUD" &
fi
DDS_PID=$!
echo "  DDS Agent PID=$DDS_PID"
sleep 5

if ! kill -0 "$DDS_PID" 2>/dev/null; then
    echo "[FATAL] DDS Agent 启动后立即退出"
    exit 1
fi
echo "  DDS Agent 运行正常"

# ---- 2. 启动 see (多桶检测) ----
echo ""
echo "=== 2/3 启动 see 多桶检测节点 ==="
eval "$(conda shell.bash hook)"
conda activate "$CONDA_ENV"
export PYTHONPATH="$CONDA_PYTHONPATH:$PYTHONPATH"
echo "  conda: $(which python3)"
echo "  PYTHONPATH 已设置"

if [[ -n "$LOG_SEE" ]]; then
    ros2 run fly see > "$LOG_SEE" 2>&1 &
else
    ros2 run fly see &
fi
SEE_PID=$!
echo "  see PID=$SEE_PID"
sleep 8

if ! kill -0 "$SEE_PID" 2>/dev/null; then
    echo "[FATAL] see 节点启动后立即退出 (检查 PYTHONPATH 和摄像头)"
    exit 1
fi
echo "  see 节点运行正常"

# ---- 启动 Watchdog ----
watchdog &
WATCHDOG_PID=$!

# ---- 3. 启动 final_mission (前台) ----
echo ""
echo "=== 3/3 启动 final_mission 多桶投放任务 (前台) ==="
echo "    按 Ctrl+C 停止所有节点"
echo ""

if [[ -n "$LOG_MISSION" ]]; then
    ros2 run fly final_mission > "$LOG_MISSION" 2>&1 &
    MISSION_PID=$!
    echo "  final_mission PID=$MISSION_PID (日志模式, 后台运行)"
    wait "$MISSION_PID"
else
    ros2 run fly final_mission
fi

cleanup
