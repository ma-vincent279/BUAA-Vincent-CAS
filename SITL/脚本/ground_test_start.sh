#!/bin/bash
# ================================================================
# 地面测试 — 一键启动 (不装桨, 验证 ARM→OFFBOARD→检测→投放)
# 用法: bash ~/SITL/scripts/ground_test_start.sh
#
# 飞机不移动，桶在飞机正下方 (Gazebo 1.01, 0.98, 0.25)。
# 检测到桶 → 等 1s → AUX5 DROP (PWM 2000us) → 1s 后恢复 LOCK。
# Ctrl-C 手动结束。
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
    rm -rf "$PX4_LOG_DIR"/*/ 2>/dev/null
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
# AUX5 舵机控制必需的 PX4 参数
export PX4_PARAM_PWM_AUX_FUNC5=1024     # RC AUX 5
export PX4_PARAM_RC_MAP_AUX5=0          # Unassigned, 解除 RC 占用
export PX4_SITL_WORLD="$SITL_DIR/worlds/drop_opt.world"
echo "  PX4 状态已重置"
echo "  World: drop_opt.world (地面测试复用, 检测器参数独立配置桶位置)"

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
echo "=== 7. 启动 YOLOv8 桶检测器 (sim 模式) ==="
source "$SITL_DIR/install/setup.bash"
# 桶 GT 投影位置: 飞机在地面时 NED z≈0.75, 桶需 dz≥0.3m 才通过检测器检查.
# bucket_gz_z=0.07 → NED z=0.76, 飞机 NED z≈0.72 → dz≈0.04 (太近,不通过)
# 把虚拟桶放到地面以下: bucket_gz_z=-0.5 → NED z=1.33 → dz≈0.6 (通过)
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true \
    -p bucket_gz_x:=1.01 -p bucket_gz_y:=0.98 -p bucket_gz_z:=-0.5 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 -p spawn_gz_z:=0.83 \
    -p conf_threshold:=0.30 \
    &
DETECTOR_PID=$!
sleep 2
echo "  Detector PID=$DETECTOR_PID"

echo ""
echo "=== 8. 启动地面测试任务 ==="
echo "  =========================================="
echo "  等待 ARM + OFFBOARD..."
echo "  桶在飞机正下方，YOLOv8 sim 模式 GT 投影检测..."
echo "  检测到桶 → 1s 后 AUX5 DROP → 1s 后 LOCK → 等待下一次..."
echo "  按 Ctrl-C 退出"
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl ground_test_mission

# Ctrl-C 后的清理
echo ""
echo "=== 9. 清理 ==="
kill $AGENT_PID 2>/dev/null || true
kill $DETECTOR_PID 2>/dev/null || true
sleep 1
echo ""
echo "地面测试完成喵~"
