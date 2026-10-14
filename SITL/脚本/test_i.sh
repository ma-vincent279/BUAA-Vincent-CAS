#!/bin/bash
# ================================================================
# 子项目 i — 一键启动 (基底 = 项目 e, 只改释放判据) (投放区 30m + 侦察区动作序列 + 返航)
# ----------------------------------------------------------------
# 与 e 的唯一差异 = 两级释放判据 (方案 B, 2026-09-24):
#   Tier1 优选: conv_m<5cm + 低谷(<=本段最小+2cm) + 准静止(vxy<0.12), 确认 0.5s
#   Tier2 保底: e 的旧判据 conv_m<10cm 持续 1.0s (先给 Tier1 10s 优先窗口)
#   Tier3 兜底: DROP_AFTER_DELAY=35s 无条件释放 (未改)
# 用法: bash ~/SITL/scripts/test_i.sh
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_FILE="/tmp/i_output.log"

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
export PX4_SITL_WORLD="$SITL_DIR/worlds/i.world"
echo "  PX4 状态已重置"
echo "  World: i.world (投放区 30m×5m, 侦察区 55m×5m, 3 桶)"

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
# YOLOv8 多桶检测器 (sim 模式, 3 桶, 投放区 NED y=30~35)
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true \
    -p conf_threshold:=0.3 \
    -p bucket_count:=3 \
    -p bucket1_gz_x:=32.51 -p bucket1_gz_y:=-0.52 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=34.01 -p bucket2_gz_y:=2.98 -p bucket2_gz_z:=0.25 \
    -p bucket3_gz_x:=35.01 -p bucket3_gz_y:=0.98 -p bucket3_gz_z:=0.25 &
sleep 0.5
# 独立裁判 (final_judge, 传入桶坐标). 输出重定向到日志文件供事后查看打分
JUDGE_LOG="/tmp/i_judge.log"
rm -f "$JUDGE_LOG"
ros2 run drone_drl final_judge --ros-args \
    -p bucket1_gz_x:=32.51 -p bucket1_gz_y:=-0.52 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=34.01 -p bucket2_gz_y:=2.98 -p bucket2_gz_z:=0.25 \
    -p bucket3_gz_x:=35.01 -p bucket3_gz_y:=0.98 -p bucket3_gz_z:=0.25 \
    -p bucket_radius:=0.125 > "$JUDGE_LOG" 2>&1 &
sleep 1
echo "  辅助节点已启动 (裁判日志: $JUDGE_LOG)"

echo ""
echo "=== 8. 启动 F 任务 ==="
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl i 2>&1 | tee "$LOG_FILE"

# 任务结束后的清理
echo ""
echo "=== 9. 清理 ==="
kill $AGENT_PID 2>/dev/null || true
sleep 1
echo ""
echo "=== 裁判打分 ==="
if [ -f "$JUDGE_LOG" ]; then
    cat "$JUDGE_LOG"
else
    echo "  (无裁判日志)"
fi
echo ""
echo "f 完成喵~ 任务日志: $LOG_FILE / 裁判日志: $JUDGE_LOG"
