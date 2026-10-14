#!/bin/bash
# ================================================================
# 外接盘 "f r3" (SITL 导入版 f_r3) — 一键仿真验证  2026-10-07
# ----------------------------------------------------------------
# 目的: 验证 ①frame_seq 补丁是否让"低空限速伺服 + 速度预判制动"真正动起来
#              ②释放走 aligned_fresh_frames 还是 timeout_force
#              ③LOW-BRAKE hold/resume 行为、落点、总时长
# 节点 = drone_drl/f_r3.py (= 外接盘 f.py + 6 处最小适配, 控制逻辑一行未改)
# 检测器 = yolov8_bucket_detector (sim_gt_direct 默认 true, 已打 frame_seq 补丁)
# 世界 = worlds/i.world (投放区 30m×5m + 侦察区 55m×5m, 3 桶, 与 i/a/f 世界同布局)
# 用法: bash ~/SITL/scripts/test_f_r3.sh
# ================================================================
set -e

SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
LOG_FILE="/tmp/f_r3_output.log"
JUDGE_LOG="/tmp/f_r3_judge.log"
DETS_LOG="/tmp/f_r3_dets.log"

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
pkill -9 -f "drone_drl/lib/drone_drl" 2>/dev/null || true   # 2026-10-07 教训: 按可执行文件路径杀
pkill -9 -f "ros2 run drone_drl" 2>/dev/null || true
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
# YOLOv8 多桶检测器 (sim 直发 GT, 已打 frame_seq 补丁)
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true \
    -p conf_threshold:=0.3 \
    -p bucket_count:=3 \
    -p bucket1_gz_x:=32.51 -p bucket1_gz_y:=-0.52 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=34.01 -p bucket2_gz_y:=2.98 -p bucket2_gz_z:=0.25 \
    -p bucket3_gz_x:=35.01 -p bucket3_gz_y:=0.98 -p bucket3_gz_z:=0.25 &
sleep 0.5
rm -f "$JUDGE_LOG"
ros2 run drone_drl final_judge --ros-args \
    -p bucket1_gz_x:=32.51 -p bucket1_gz_y:=-0.52 -p bucket1_gz_z:=0.25 \
    -p bucket2_gz_x:=34.01 -p bucket2_gz_y:=2.98 -p bucket2_gz_z:=0.25 \
    -p bucket3_gz_x:=35.01 -p bucket3_gz_y:=0.98 -p bucket3_gz_z:=0.25 \
    -p bucket_radius:=0.125 > "$JUDGE_LOG" 2>&1 &
sleep 3
echo "  检测器/裁判已启动"
echo -n "  /bucket/detections 发布者数: "
ros2 topic info /bucket/detections -v 2>/dev/null | grep -i "publisher count" || echo "?"
NPUB=$(ros2 topic info /bucket/detections 2>/dev/null | grep -i "Publisher count" | awk '{print $NF}')
if [ "${NPUB:-0}" != "1" ]; then
    echo "  ⚠ 发布者数不是 1 (${NPUB:-?}) —— 按 2026-10-07 教训, 结论作废, 先清理残留节点"
fi

echo ""
echo "=== 8. 启动 f_r3 任务 (后台同时抓 frame_seq 证据) ==="
rm -f "$DETS_LOG"
( timeout 900 ros2 topic echo /bucket/detections 2>/dev/null \
    | grep --line-buffered -o '"frame_seq": [0-9]*' | head -40 > "$DETS_LOG" ) &
echo "  frame_seq 采样 -> $DETS_LOG"
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
ros2 run drone_drl f_r3 2>&1 | tee "$LOG_FILE"

echo ""
echo "=== 9. 清理 (按可执行文件路径杀, 防残留) ==="
kill $AGENT_PID 2>/dev/null || true
pkill -9 -f "drone_drl/lib/drone_drl" 2>/dev/null || true
pkill -9 -f "ros2 run drone_drl" 2>/dev/null || true
sleep 1
echo -n "  残留任务节点数: "
ps -eo args | grep "[d]rone_drl/lib/drone_drl" | wc -l
echo "  残留发布者数: $(ros2 topic info /bucket/detections 2>/dev/null | grep -i 'Publisher count' || echo 'n/a (DDS 已停)')"

echo ""
echo "=== 10. frame_seq 证据 (检测器实际发出的序号) ==="
if [ -s "$DETS_LOG" ]; then
    echo -n "  前 5 个: "; head -5 "$DETS_LOG" | tr '\n' ' '; echo
    echo -n "  后 3 个: "; tail -3 "$DETS_LOG" | tr '\n' ' '; echo
    echo "  取样条数: $(wc -l < "$DETS_LOG")"
else
    echo "  ⚠ 没抓到 (可能 echo 起得太晚 / 检测器没发多桶 JSON)"
fi

echo ""
echo "=== 11. 裁判打分 ==="
if [ -f "$JUDGE_LOG" ]; then
    cat "$JUDGE_LOG"
else
    echo "  (无裁判日志)"
fi
echo ""
echo "f_r3 完成喵~ 任务日志: $LOG_FILE / 裁判日志: $JUDGE_LOG / frame_seq: $DETS_LOG"
