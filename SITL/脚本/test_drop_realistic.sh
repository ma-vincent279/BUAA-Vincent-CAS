#!/bin/bash
# ================================================================
# 投放精度优化 — 真实环境仿真一键启动 (风力 + YOLOv8检测 + 质量突变)
# 用法: bash ~/SITL/scripts/test_drop_realistic.sh
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_FILE="/tmp/drop_opt_realistic_output.log"

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
# 使用带风力的真实仿真世界
export PX4_SITL_WORLD="$SITL_DIR/worlds/drop_opt_realistic.world"
echo "  PX4 状态已重置"
echo "  World: drop_opt_realistic.world (风力 0.8m/s 稳态 + 阵风 2.0m/s)"

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
echo "=== 5. 启动 PX4 SITL + Gazebo (真实风力世界) ==="
cd "$PX4_DIR"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_PLUGIN_PATH="$PX4_BUILD/build_gazebo-classic:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH"
make px4_sitl gazebo-classic_iris_mono_cam_down &
PX4_PID=$!
echo "  等待初始化 (30s)..."
sleep 30
pgrep -x gzserver > /dev/null || { echo "ERROR: gzserver failed"; exit 1; }
pgrep -x px4 > /dev/null || { echo "ERROR: px4 failed"; exit 1; }
echo "  PX4 + Gazebo 就绪 (风力已激活)"

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
echo "=== 7. 启动辅助节点 ==="
source "$SITL_DIR/install/setup.bash"

# YOLOv8 桶检测器 (sim 模式: 合成图像 + YOLOv8 推理)
# 与实机共享同一代码路径, 迁移时只需改 use_sim:=false + camera_device:=/dev/video0
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true \
    -p bucket_gz_x:=11.01 -p bucket_gz_y:=0.98 -p bucket_gz_z:=0.25 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 -p spawn_gz_z:=0.83 \
    -p model_path:=/home/vincent/SITL/yolo_v8/best2.pt \
    -p conf_threshold:=0.30 \
    -p infer_hz:=10.0 &
sleep 2

# 细化裁判 (可选, 需要 /model_states)
ros2 run drone_drl drop_opt_judge --ros-args \
    -p bucket_gz_x:=11.01 -p bucket_gz_y:=0.98 -p bucket_gz_z:=0.25 \
    -p bucket_radius:=0.125 &
sleep 1
echo "  辅助节点已启动"

echo ""
echo "=== 8. 启动投放优化任务 (真实仿真模式) ==="
echo "  特性: 风力扰动 | YOLOv8 检测 | 投放质量突变 | 位置监视 2Hz"
echo "  sim->real 迁移: 仅需改 common.PX4_STATUS_SUFFIX + 检测器 use_sim/camera_device"
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
# 注意: MASS_LOSS_ENABLE 和 MONITOR_HZ 在 drop_opt_mission.py 顶部参数块中设置
# 如需关闭质量突变: 修改 MASS_LOSS_ENABLE = False
ros2 run drone_drl drop_opt_mission --ros-args \
    -p mass_loss_enable:=true \
    -p monitor_hz:=2.0 \
    2>&1 | tee "$LOG_FILE"

# 任务结束后的清理
echo ""
echo "=== 9. 清理 ==="
kill $AGENT_PID 2>/dev/null || true
sleep 1
echo ""
echo "完成喵~ 日志已保存到 $LOG_FILE"
