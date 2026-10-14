#!/bin/bash
# ================================================================
# 单桶投弹任务一键启动
# 用法: bash ~/SITL/scripts/single_drop_start.sh
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"

echo "=== 1. 清理残留进程 ==="
for port in 11345 8888; do
    for pid in $(lsof -ti:$port 2>/dev/null); do
        kill -9 $pid 2>/dev/null && echo "  killed PID $pid (port $port)" || true
    done
done
pkill -9 -x "px4" 2>/dev/null || true
pkill -9 -x "gzserver" 2>/dev/null || true
pkill -9 -x "gzclient" 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "sitl_run" 2>/dev/null || true
pkill -9 -f "make px4_sitl" 2>/dev/null || true
pkill -9 -f "drone_drl" 2>/dev/null || true
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 2>/dev/null
sleep 2
echo "  进程清理完成"

echo ""
echo "=== 2. 清理旧日志 + 重置 PX4 持久化状态 ==="
PX4_LOG_DIR="$PX4_ROOTFS/log"
if [ -d "$PX4_LOG_DIR" ]; then
    OLD_LOG_COUNT=$(find "$PX4_LOG_DIR" -name "*.ulg" 2>/dev/null | wc -l)
    if [ "$OLD_LOG_COUNT" -gt 0 ]; then
        rm -rf "$PX4_LOG_DIR"/*/
        echo "  已清理 $OLD_LOG_COUNT 个历史 ULog 飞行日志"
    fi
fi
rm -f "$PX4_BUILD/dataman"
rm -f "$PX4_ROOTFS/dataman"
rm -rf "$PX4_ROOTFS/eeprom"
rm -f "$PX4_ROOTFS/parameters.bson"
rm -f "$PX4_ROOTFS/parameters_backup.bson"
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_COM_OF_LOSS_T=10.0
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_PARAM_COM_RCL_EXCEPT=4
export PX4_PARAM_NAV_RCL_ACT=0
export PX4_SITL_WORLD=/home/vincent/SITL/worlds/single_bucket.world
echo "  PX4 状态已重置"
echo "  World: single_bucket.world"

echo ""
echo "=== 3. 编译 ==="
cd "$SITL_DIR"
colcon build --packages-select drone_drl 2>&1 | tail -5
source "$SITL_DIR/install/setup.bash"

echo ""
echo "=== 4. 启动 MicroXRCEAgent ==="
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2
echo "  Agent PID=$AGENT_PID"

echo ""
echo "=== 5. 启动 PX4 SITL + Gazebo ==="
cd "$PX4_DIR"
export GAZEBO_MODEL_DATABASE_URI=""
make px4_sitl gazebo-classic_iris_mono_cam_down &
PX4_PID=$!
echo "  等待初始化 (30s)..."
sleep 30
pgrep -x gzserver > /dev/null || { echo "ERROR: gzserver failed"; exit 1; }
pgrep -x px4 > /dev/null || { echo "ERROR: px4 failed"; exit 1; }
echo "  PX4 + Gazebo 就绪"

echo ""
echo "=== 6. 等待 DDS 话题 ==="
source "$SITL_DIR/install/setup.bash"
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        echo "  DDS 就绪"
        break
    fi
    sleep 1
done

echo ""
echo "=== 7. 启动检测器 (后台) ==="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl single_bucket_detector &
DETECTOR_PID=$!
sleep 1
echo "  Detector PID=$DETECTOR_PID"

echo ""
echo "=== 8. 启动单桶投弹任务 ==="
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl single_drop_mission --ros-args -p use_sim:=true 2>&1 | tee /tmp/single_drop.log

# 清理
kill $DETECTOR_PID 2>/dev/null || true
kill $AGENT_PID 2>/dev/null || true
echo ""
echo "完成喵~ (single drop mission)"
