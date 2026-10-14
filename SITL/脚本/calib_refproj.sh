#!/bin/bash
# ================================================================
# 标定工具 B 的一键启动 (仿真飞行版)
#   清理 → 重置 PX4 状态 → 编译 → Agent → PX4+Gazebo → 检测器 → 标定节点
#
# 跑完会输出:
#   /tmp/calib_refproj.log   终端全文 (每个网格点的 RESULT + 回归)
#   /tmp/calib_refproj.csv   逐帧明细
#
# 用法: bash ~/SITL/scripts/calib_refproj.sh
# 注意: 只跑标定, 不跑 a/b/c/d/e 任何任务节点; 不改动任何飞行代码
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_FILE="/tmp/calib_refproj.log"

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
# 注意: 这里用 [c] 方括号技巧, 否则 pkill -f 会匹配到本脚本自己的命令行 → 自杀 (SIGKILL)
pkill -9 -f "[c]alib_refproj\.py" 2>/dev/null || true
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 2>/dev/null
sleep 2
echo "  进程清理完成"

echo ""
echo "=== 2. 重置 PX4 持久化状态 ==="
rm -rf "$PX4_ROOTFS/log"/*/ 2>/dev/null || true
rm -f "$PX4_BUILD/dataman" "$PX4_ROOTFS/dataman"
rm -rf "$PX4_ROOTFS/eeprom"
rm -f "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson"
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_COM_OF_LOSS_T=10.0
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_SITL_WORLD="$SITL_DIR/worlds/e.world"
echo "  World: e.world   桶 B3 真值 NED(0.00,34.00)  (= 卷尺真值)"

echo ""
echo "=== 3. 编译 ==="
cd "$SITL_DIR"
colcon build --packages-select drone_drl 2>&1 | tail -3
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
echo "  等待初始化 (轮询, 最多 600s)..."
for i in $(seq 1 120); do
    if pgrep -x gzserver > /dev/null && pgrep -x px4 > /dev/null; then
        echo "  初始化完成 (第 $((i*5))s)"
        break
    fi
    sleep 5
done
pgrep -x gzserver > /dev/null || { echo "ERROR: gzserver failed"; exit 1; }
pgrep -x px4 > /dev/null || { echo "ERROR: px4 failed"; exit 1; }

echo ""
echo "=== 6. 等待 DDS 话题 ==="
source "$SITL_DIR/install/setup.bash"
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        echo "  DDS 就绪"; break
    fi
    sleep 1
done

echo ""
echo "=== 7. 启动检测器 (与任务完全相同的节点) ==="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true \
    -p conf_threshold:=0.3 \
    -p bucket_count:=3 \
    -p bucket1_gz_x:=32.51 -p bucket1_gz_y:=-0.52 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=34.01 -p bucket2_gz_y:=2.98 -p bucket2_gz_z:=0.25 \
    -p bucket3_gz_x:=35.01 -p bucket3_gz_y:=0.98 -p bucket3_gz_z:=0.25 &
sleep 8

echo ""
echo "=== 8. 启动标定节点 (工具 B) ==="
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
python3 "$SITL_DIR/scripts/calib_refproj.py" 2>&1 | tee "$LOG_FILE"

echo ""
echo "=== 9. 清理 ==="
kill $AGENT_PID 2>/dev/null || true
sleep 1
echo "B 完成 → $LOG_FILE / /tmp/calib_refproj.csv"
