#!/bin/bash
# ============================================================================
# DDS + YOLO + Drop 一键启动脚本
# 用法: ./launch_drop.sh [--log]
#   --log: 将各节点输出记录到 ~/fly_logs/ 目录
#
# Ctrl+C 停止所有后台进程。drop 异常退出时自动清理其余进程。
# ============================================================================
set -e

# ---- 路径 & 环境 ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DDS_AGENT="MicroXRCEAgent"  # 系统安装, 在 PATH 中
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
    LOG_YOLO="$LOG_DIR/yolo_$TIMESTAMP.log"
    LOG_DROP="$LOG_DIR/drop_$TIMESTAMP.log"
    echo "[$(date)] Logging to $LOG_DIR/"
fi

# ---- 清理函数 ----
_CLEANED=false
cleanup() {
    if $_CLEANED; then return; fi
    _CLEANED=true
    echo ""
    echo "=== 正在停止所有后台进程... ==="
    # 先停 watchdog，避免它在我们 kill 其他进程时误报
    [[ -n "$WATCHDOG_PID" ]] && kill "$WATCHDOG_PID" 2>/dev/null && echo "  watchdog ($WATCHDOG_PID) 已终止"
    [[ -n "$YOLO_PID" ]] && kill "$YOLO_PID" 2>/dev/null && echo "  yolo ($YOLO_PID) 已终止"
    [[ -n "$DDS_PID" ]] && kill "$DDS_PID" 2>/dev/null && echo "  dds  ($DDS_PID) 已终止"
    # drop 在前台运行，不需要显式 kill；但如果以 --log 后台运行则清理
    [[ -n "$DROP_PID" ]] && kill "$DROP_PID" 2>/dev/null && echo "  drop ($DROP_PID) 已终止"
    echo "=== 清理完成 ==="
}

# EXIT 确保 drop 异常退出 (set -e) 时也能清理后台进程。
# _CLEANED 防止 INT → cleanup → EXIT 重复执行。
trap cleanup INT TERM EXIT

# ---- 后台进程存活监控 ----
watchdog() {
    while true; do
        if ! kill -0 "$YOLO_PID" 2>/dev/null; then
            echo "[FATAL] YOLO 节点意外退出!" >&2
            kill "$$" 2>/dev/null  # 终止整个脚本 (触发 EXIT trap)
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
    echo "[FATAL] DDS Agent 未找到: $DDS_AGENT (请确认已安装到 PATH)"
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

# 检查是否已有 Agent 在运行
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

# ---- 2. 启动 YOLO ----
echo ""
echo "=== 2/3 启动 YOLO 检测节点 ==="
eval "$(conda shell.bash hook)"
conda activate "$CONDA_ENV"
export PYTHONPATH="$CONDA_PYTHONPATH:$PYTHONPATH"
echo "  conda: $(which python3)"
echo "  PYTHONPATH 已设置"

if [[ -n "$LOG_YOLO" ]]; then
    ros2 run fly yolo > "$LOG_YOLO" 2>&1 &
else
    ros2 run fly yolo &
fi
YOLO_PID=$!
echo "  YOLO PID=$YOLO_PID"
sleep 8

if ! kill -0 "$YOLO_PID" 2>/dev/null; then
    echo "[FATAL] YOLO 节点启动后立即退出 (检查 PYTHONPATH 和摄像头)"
    exit 1
fi
echo "  YOLO 节点运行正常"

# ---- 启动 Watchdog ----
watchdog &
WATCHDOG_PID=$!

# ---- 3. 启动 Drop (前台运行, Ctrl+C 退出) ----
echo ""
echo "=== 3/3 启动 Drop 投放任务节点 (前台) ==="
echo "    按 Ctrl+C 停止所有节点"
echo ""

if [[ -n "$LOG_DROP" ]]; then
    ros2 run fly drop > "$LOG_DROP" 2>&1 &
    DROP_PID=$!
    echo "  Drop PID=$DROP_PID (日志模式, 后台运行)"
    wait "$DROP_PID"
else
    ros2 run fly drop
fi

# 正常退出时清理 (drop 前台运行不会到达这里, 但 --log 模式会)
cleanup
