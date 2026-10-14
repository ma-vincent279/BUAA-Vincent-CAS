#!/bin/bash
# 多轮仿真验证 — YOLOv8 检测器 + 单桶投弹
# 用法: bash ~/SITL/scripts/multi_verify.sh [轮数, 默认5]

RUNS=${1:-5}
SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_DIR="/tmp/yolo_verify_runs"
mkdir -p "$LOG_DIR"

echo "=============================================="
echo "  YOLOv8 检测器 + 单桶投弹 — ${RUNS} 轮仿真验证"
echo "=============================================="
echo ""

for run in $(seq 1 $RUNS); do
    echo "##############################################"
    echo "  第 ${run}/${RUNS} 轮"
    echo "##############################################"

    # ---- 1. 清理 ----
    echo "[清理] 残留进程..."
    for port in 11345 8888; do
        for pid in $(lsof -ti:$port 2>/dev/null); do
            kill -9 $pid 2>/dev/null || true
        done
    done
    pkill -9 -x "px4" 2>/dev/null || true
    pkill -9 -x "gzserver" 2>/dev/null || true
    pkill -9 -x "gzclient" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "drone_drl" 2>/dev/null || true
    sleep 2

    # ---- 2. PX4 状态重置 ----
    rm -f "$PX4_BUILD/dataman" "$PX4_ROOTFS/dataman"
    rm -rf "$PX4_ROOTFS/eeprom"
    rm -f "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson"

    # ---- 3. 编译 ----
    cd "$SITL_DIR"
    colcon build --packages-select drone_drl 2>&1 | tail -2

    # ---- 4. Agent ----
    MicroXRCEAgent udp4 -p 8888 &>/tmp/agent_${run}.log &
    AGENT_PID=$!
    sleep 2

    # ---- 5. PX4 + Gazebo (GUI) ----
    cd "$PX4_DIR"
    export PX4_SITL_WORLD="$SITL_DIR/worlds/single_bucket.world"
    export GAZEBO_MODEL_DATABASE_URI=""
    export PX4_PARAM_CBRK_SUPPLY_CHK=894281
    export PX4_PARAM_CBRK_IO_SAFETY=22027
    export PX4_PARAM_CBRK_USB_CHK=197848
    export PX4_PARAM_COM_RCL_EXCEPT=4
    make px4_sitl gazebo-classic_iris_mono_cam_down &>/tmp/px4_${run}.log &
    PX4_PID=$!

    echo "[等待] PX4+Gazebo 初始化 (25s)..."
    sleep 25
    pgrep -x gzserver > /dev/null || { echo "ERROR: gzserver failed"; continue; }
    pgrep -x px4 > /dev/null || { echo "ERROR: px4 failed"; continue; }
    echo "  PX4+Gazebo 就绪"

    # ---- 6. DDS 等待 ----
    source "$SITL_DIR/install/setup.bash"
    for i in $(seq 1 20); do
        if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
            break
        fi
        sleep 1
    done

    # ---- 7. YOLOv8 检测器 ----
    source "$SITL_DIR/install/setup.bash"
    LOGFILE="$LOG_DIR/run_${run}.log"
    ros2 run drone_drl yolov8_bucket_detector --ros-args -p use_sim:=true -p conf_threshold:=0.35 &>/tmp/detector_${run}.log &
    DET_PID=$!
    sleep 3

    # ---- 8. 任务 ----
    echo "[运行] 单桶投弹任务..."
    timeout 120 ros2 run drone_drl single_drop_mission --ros-args -p use_sim:=true 2>&1 | tee "$LOGFILE" | grep -E "(→ |DROP IMPACT|Hover stable|Land|Home:|Takeoff|Returned|disarm|Ready|NED.*DET.*err=)"


    # ---- 9. 提取结果 ----
    IMPACT=$(grep "DROP IMPACT" "$LOGFILE" | head -1 | grep -oP 'NED\([^)]+\)' || echo "N/A")
    DISARM=$(grep -c "disarm" "$LOGFILE" || echo "0")
    echo ""
    echo "  结果: $IMPACT  disarm=$DISARM"

    # ---- 10. 清理 ----
    kill $DET_PID 2>/dev/null || true
    kill $AGENT_PID 2>/dev/null || true
    for port in 11345 8888; do
        for pid in $(lsof -ti:$port 2>/dev/null); do
            kill -9 $pid 2>/dev/null || true
        done
    done
    pkill -9 -x "px4" 2>/dev/null || true
    pkill -9 -x "gzserver" 2>/dev/null || true
    pkill -9 -x "gzclient" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "drone_drl" 2>/dev/null || true
    sleep 3
    echo ""
done

echo ""
echo "=============================================="
echo "  ${RUNS} 轮仿真验证完成"
echo "=============================================="
echo ""
echo "结果汇总:"
for f in "$LOG_DIR"/run_*.log; do
    rn=$(basename "$f" .log | sed 's/run_//')
    impact=$(grep "DROP IMPACT" "$f" | head -1 | grep -oP 'NED\([^)]+\)' || echo "N/A")
    land=$(grep -c "disarm" "$f" || echo "0")
    dur=$(grep "Height fallback\|Land complete" "$f" | tail -1 | head -1)
    echo "  第${rn}轮: $impact  Land:${land}"
done
echo ""
echo "详细日志: $LOG_DIR/"
