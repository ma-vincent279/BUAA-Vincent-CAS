#!/bin/bash
# ================================================================
# 投放精度优化 — 噪声注入测试
# 用法: bash ~/SITL/scripts/test_drop_noise.sh [NOISE_CONFIG]
#
# noise config:
#   baseline   — 无噪声 (默认)
#   drop       — 随机丢帧 30%
#   falsepos   — 假阳性 每秒10%概率, 持续0.8s
#   jitter     — ROI抖动 40px
#   combined   — 组合噪声 (丢帧20% + 假阳性5% + 抖动20px)
# ================================================================
set +e

NOISE=${1:-baseline}
SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_FILE="/tmp/drop_opt_noise_output.log"

case "$NOISE" in
    baseline)
        DROP_RATE=0.0; FP_RATE=0.0; JITTER=0.0; FP_DUR=0.8
        ;;
    drop)
        DROP_RATE=0.3; FP_RATE=0.0; JITTER=0.0; FP_DUR=0.8
        ;;
    falsepos)
        DROP_RATE=0.0; FP_RATE=0.1; JITTER=0.0; FP_DUR=0.8
        ;;
    jitter)
        DROP_RATE=0.0; FP_RATE=0.0; JITTER=40.0; FP_DUR=0.8
        ;;
    combined)
        DROP_RATE=0.2; FP_RATE=0.05; JITTER=20.0; FP_DUR=0.8
        ;;
    *)
        echo "Unknown: $NOISE (use: baseline drop falsepos jitter combined)"
        exit 1
        ;;
esac

echo "============================================"
echo "  Noise Test: $NOISE"
echo "  drop_rate=$DROP_RATE  fp_rate=$FP_RATE  jitter=$JITTER px"
echo "============================================"

# 1. 清理
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
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 2>/dev/null
sleep 2

# 2. 重置 PX4
rm -f "$PX4_BUILD/dataman" "$PX4_ROOTFS/dataman" "$PX4_ROOTFS/parameters.bson"
rm -rf "$PX4_ROOTFS/eeprom"
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_COM_OF_LOSS_T=10.0
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_SITL_WORLD="$SITL_DIR/worlds/drop_opt.world"

# 3. 编译
cd "$SITL_DIR"
colcon build --packages-select drone_drl 2>&1 | tail -3
source install/setup.bash

# 4. Agent
MicroXRCEAgent udp4 -p 8888 &
AGENT=$!
sleep 2

# 5. PX4 + Gazebo
cd "$PX4_DIR"
export GAZEBO_MODEL_DATABASE_URI=""
make px4_sitl gazebo-classic_iris_mono_cam_down &
sleep 30

# 6. 等待 DDS
source "$SITL_DIR/install/setup.bash"
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        break
    fi
    sleep 1
done

# 7. 辅助节点
source "$SITL_DIR/install/setup.bash"

# 裁判
ros2 run drone_drl drop_opt_judge --ros-args \
    -p bucket_gz_x:=11.01 -p bucket_gz_y:=0.98 -p bucket_gz_z:=0.25 \
    -p bucket_radius:=0.125 &

# 检测器 → raw topic (噪声注入器前)
ros2 run drone_drl bucket_vision_detector --ros-args \
    -r /bucket/detection:=/bucket/detection_raw \
    -p bucket1_gz_x:=11.01 -p bucket1_gz_y:=0.98 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=11.01 -p bucket2_gz_y:=0.98 -p bucket2_gz_z:=0.25 &

sleep 1

# 噪声注入器 (raw → detection)
ros2 run drone_drl detection_noise_injector --ros-args \
    -p drop_rate:=$DROP_RATE \
    -p false_pos_rate:=$FP_RATE \
    -p false_pos_duration:=$FP_DUR \
    -p jitter_px:=$JITTER &
sleep 0.5

echo ""
echo "=== 噪声注入器就绪: $NOISE ==="
echo "  管道: detector → /bucket/detection_raw → [noise] → /bucket/detection → mission"

# 8. 任务
echo ""
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl drop_opt_mission 2>&1 | tee "$LOG_FILE"

# 9. 清理
kill $AGENT 2>/dev/null || true
sleep 1
echo ""
echo "完成喵~ 日志: $LOG_FILE"
