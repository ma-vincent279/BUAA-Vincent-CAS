#!/bin/bash
# ================================================================
# SITL 全任务干净启动 v2 — A3 中心悬停 + 视觉面积排序选目标
# 竞赛场地 competition_field.world, 投放区(35m) + 侦察区(57m), 3 桶 + 5 白筒
# 新检测器 yolo_bucket_detector: 多 bbox → 位置记忆匹配 → 自动选最大两桶
# 新任务节点 full_mission_v2: 取消硬编码航点, 纯视觉选目标
# 用法: bash ~/SITL/scripts/clean_start_v2.sh
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
for port in 11345 8888; do
    ss -tlnp 2>/dev/null | grep -q ":$port" && echo "  ERROR: port $port still in use!" && exit 1
done
echo "  进程清理完成"

echo ""
echo "=== 2. 清理旧日志 + 重置 PX4 持久化状态 + SITL 参数 ==="
# 清理 PX4 SITL 历史飞行日志 (每次运行自动生成 .ulg, 不清理会累积数十 GB)
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
export PX4_SITL_WORLD=/home/vincent/SITL/worlds/competition_field.world
echo "  PX4 状态已重置 + SITL 参数已导出"
echo "  World: competition_field.world (竞赛场地: 3桶投放区~35m + 5筒侦察区~60m)"

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
echo "=== 7. 启动辅助节点 ==="
source "$SITL_DIR/install/setup.bash"
# 载荷投放
ros2 run drone_drl payload_dropper &
sleep 0.5
# 桶检测器 v2 (A3: 多 bbox 仿真 YOLO → 中心悬停初始化 → 位置记忆追踪 → 单 ROI)
ros2 run drone_drl yolo_bucket_detector --ros-args \
    -p bucket1_gz_x:=35.51 -p bucket1_gz_y:=2.0 -p bucket1_gz_z:=0.15 \
    -p bucket2_gz_x:=37.01 -p bucket2_gz_y:=1.0 -p bucket2_gz_z:=0.15 \
    -p bucket3_gz_x:=35.51 -p bucket3_gz_y:=0.0 -p bucket3_gz_z:=0.15 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 -p spawn_gz_z:=0.83 &
sleep 0.5
# 筒检测器 (竞赛场地平移后: 侦察区 X≈59~64)
ros2 run drone_drl scout_detector --ros-args \
    -p c0_gz_x:=59.51 -p c0_gz_y:=2.5 -p c0_gz_z:=0.075 \
    -p c1_gz_x:=60.51 -p c1_gz_y:=-0.5 -p c1_gz_z:=0.075 \
    -p c2_gz_x:=61.51 -p c2_gz_y:=2.0 -p c2_gz_z:=0.075 \
    -p c3_gz_x:=62.51 -p c3_gz_y:=0.0 -p c3_gz_z:=0.075 \
    -p c4_gz_x:=63.51 -p c4_gz_y:=1.0 -p c4_gz_z:=0.075 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 -p spawn_gz_z:=0.83 &
sleep 0.5
# 统一裁判节点 (含着陆区参数, 平移后着陆区 Gazebo(1.01,1) 对齐 spawn)
ros2 run drone_drl mission_judge --ros-args \
    -p bucket1_gz_x:=35.51 -p bucket1_gz_y:=2.0 \
    -p bucket2_gz_x:=37.01 -p bucket2_gz_y:=1.0 \
    -p bucket3_gz_x:=35.51 -p bucket3_gz_y:=0.0 \
    -p landing_gz_x:=1.01 -p landing_gz_y:=1.0 \
    -p landing_radius:=2.5 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 &
sleep 1
echo "  辅助节点已启动"

echo ""
echo "=== 8. 启动全任务节点 v2 (A3 中心悬停) ==="
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl full_mission_v2 2>&1 | tee /tmp/full_mission_v2.log
# 任务结束后的清理
kill $AGENT_PID 2>/dev/null || true
echo ""
echo "完成喵~ (v2 A3 center-hover)"
