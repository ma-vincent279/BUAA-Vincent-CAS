#!/bin/bash
# 3 轮 YOLOv8 仿真验证
SITL=/home/vincent/SITL
PX4D=/home/vincent/PX4-Autopilot
PX4B=$PX4D/build/px4_sitl_default

for round in 1 2 3; do
    echo "=============================================="
    echo "  Round $round/3"
    echo "=============================================="

    # 清理
    pkill -9 -f "drone_drl" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -x "px4" 2>/dev/null || true
    pkill -9 -x "gzserver" 2>/dev/null || true
    pkill -9 -x "gzclient" 2>/dev/null || true
    for port in 11345 8888; do
        for pid in $(lsof -ti:$port 2>/dev/null); do kill -9 $pid 2>/dev/null; done
    done
    sleep 2

    # PX4 状态重置
    rm -f $PX4B/dataman $PX4B/rootfs/dataman
    rm -rf $PX4B/rootfs/eeprom
    rm -f $PX4B/rootfs/parameters.bson $PX4B/rootfs/parameters_backup.bson

    # Agent
    MicroXRCEAgent udp4 -p 8888 &>/tmp/agent_r${round}.log &
    sleep 2

    # PX4+Gazebo
    cd $PX4D
    PX4_SITL_WORLD=$SITL/worlds/single_bucket.world \
    GAZEBO_MODEL_DATABASE_URI="" \
    PX4_PARAM_CBRK_SUPPLY_CHK=894281 \
    PX4_PARAM_CBRK_IO_SAFETY=22027 \
    PX4_PARAM_CBRK_USB_CHK=197848 \
    PX4_PARAM_COM_RCL_EXCEPT=4 \
    make px4_sitl gazebo-classic_iris_mono_cam_down &>/tmp/px4_r${round}.log &
    echo "  Waiting PX4+Gazebo (28s)..."
    sleep 28

    if ! pgrep -x gzserver >/dev/null; then echo "  FAIL: gzserver"; continue; fi
    if ! pgrep -x px4 >/dev/null; then echo "  FAIL: px4"; continue; fi

    # DDS
    source $SITL/install/setup.bash
    for i in $(seq 1 20); do
        ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4" && break
        sleep 1
    done

    # Detector
    source $SITL/install/setup.bash
    ros2 run drone_drl yolov8_bucket_detector --ros-args -p use_sim:=true -p conf_threshold:=0.35 &>/tmp/det_r${round}.log &
    sleep 6  # Wait for YOLO model warm-up

    # Mission
    LOG=/tmp/mission_r${round}.log
    echo "  [Mission running...]"
    timeout 130 ros2 run drone_drl single_drop_mission --ros-args -p use_sim:=true 2>&1 | tee $LOG | grep -E "(→ |DROP IMPACT|Hover stable|Land|Home:|Takeoff done|Returned|disarm|Ready|TRACK conv|HOVER conv|Climb done|Detection during|At drop|Search timeout)"

    # 结果
    IMPACT=$(grep "DROP IMPACT" $LOG | grep -oP 'NED\([^)]+\)' || echo "NO_DROP")
    DISARM=$(grep -c "disarm" $LOG || echo 0)
    echo "  >> Round $round: $IMPACT  disarm=$DISARM"
    echo ""

    # Cleanup for next round
    pkill -9 -f "drone_drl" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -x "px4" 2>/dev/null || true
    pkill -9 -x "gzserver" 2>/dev/null || true
    pkill -9 -x "gzclient" 2>/dev/null || true
    sleep 2
done

echo "=============================================="
echo "  All rounds complete. Results:"
for f in /tmp/mission_r*.log; do
    rn=$(basename $f .log | sed 's/mission_r//')
    impact=$(grep "DROP IMPACT" $f | grep -oP 'NED\([^)]+\)' || echo "NO_DROP")
    land=$(grep -c "disarm" $f || echo 0)
    echo "  Round $rn: $impact  Land:$land"
done
