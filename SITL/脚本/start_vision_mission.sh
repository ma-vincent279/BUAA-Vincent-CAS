#!/bin/bash
# ================================================================
# 视觉识别定点仿真一键启动脚本 (v2 兼容版)
# 启动顺序: MicroXRCEAgent → PX4+Gazebo → ROS2 节点
# 飞机全程偏航角锁定，前飞搜索桶，检测到后悬停。
# 桶位置通过 ROS 参数配置，不依赖 gazebo_bridge。
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
WORLD_FILE="${SITL_DIR}/worlds/bucket_search.world"
MODEL="iris_mono_cam_down"
BUCKET1_GZ_X=6.0
BUCKET1_GZ_Y=4.0
BUCKET1_GZ_Z=0.25
BUCKET2_GZ_X=6.0
BUCKET2_GZ_Y=-2.0
BUCKET2_GZ_Z=0.25
# PX4 固定生成点 (sitl_run.sh line 130)
SPAWN_GZ_X=1.01
SPAWN_GZ_Y=0.98
SPAWN_GZ_Z=0.83

echo "=== 1. 清理残留进程 ==="
pkill -9 -x "px4" 2>/dev/null || true
pkill -9 -x "gzserver" 2>/dev/null || true
pkill -9 -x "gzclient" 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "sitl_run" 2>/dev/null || true
pkill -9 -f "drone_drl" 2>/dev/null || true
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 2>/dev/null
sleep 1

echo "=== 2. 清理 PX4 持久化状态 ==="
rm -f "$PX4_BUILD/dataman" "$PX4_ROOTFS/dataman" 2>/dev/null
rm -rf "$PX4_ROOTFS/eeprom" "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson" 2>/dev/null
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_COM_OF_LOSS_T=10.0
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_SITL_WORLD="${WORLD_FILE}"

echo "=== 3. 编译 ==="
cd "$SITL_DIR"
colcon build --packages-select drone_drl 2>&1 | tail -3
source "$SITL_DIR/install/setup.bash"

# ---- Terminal 1: MicroXRCEAgent (必须先于 PX4) ----
echo "=== Terminal 1: XRCE-DDS bridge ==="
gnome-terminal -- bash -c "
    MicroXRCEAgent udp4 -p 8888
    exec bash
" &
sleep 3

# ---- Terminal 2: PX4 + Gazebo ----
echo "=== Terminal 2: PX4 SITL + Gazebo (${MODEL}) ==="
gnome-terminal -- bash -c "
    export PX4_SITL_WORLD=${WORLD_FILE}
    export PX4_PARAM_CBRK_SUPPLY_CHK=894281
    export PX4_PARAM_CBRK_IO_SAFETY=22027
    export PX4_PARAM_CBRK_USB_CHK=197848
    export GAZEBO_MODEL_DATABASE_URI=\"\"
    cd ${PX4_DIR} && make px4_sitl gazebo-classic_${MODEL}
    exec bash
" &
sleep 8

# ---- Terminal 3: ROS2 nodes ----
echo "=== Terminal 3: ROS2 mission nodes ==="
gnome-terminal -- bash -c "
    source ${SITL_DIR}/install/setup.bash
    sleep 10

    echo 'Starting bucket vision detector (v2, compatible with /bucket/target_id)...'
    ros2 run drone_drl yolo_bucket_detector \
        --ros-args -p bucket1_gz_x:=${BUCKET1_GZ_X} \
                   -p bucket1_gz_y:=${BUCKET1_GZ_Y} \
                   -p bucket1_gz_z:=${BUCKET1_GZ_Z} \
                   -p bucket2_gz_x:=${BUCKET2_GZ_X} \
                   -p bucket2_gz_y:=${BUCKET2_GZ_Y} \
                   -p bucket2_gz_z:=${BUCKET2_GZ_Z} \
                   -p spawn_gz_x:=${SPAWN_GZ_X} \
                   -p spawn_gz_y:=${SPAWN_GZ_Y} \
                   -p spawn_gz_z:=${SPAWN_GZ_Z} &
    sleep 2

    echo 'Starting visual servo mission...'
    ros2 run drone_drl visual_servo_mission

    exec bash
" &

echo "=== All processes launched ==="
echo "  Wait ~15s for PX4 to initialize, then mission begins automatically."
echo "  Aircraft will fly forward (no yaw rotation), hover when bucket detected."
echo "  Buckets at Gazebo (${BUCKET1_GZ_X}, ${BUCKET1_GZ_Y}) and (${BUCKET2_GZ_X}, ${BUCKET2_GZ_Y})"
