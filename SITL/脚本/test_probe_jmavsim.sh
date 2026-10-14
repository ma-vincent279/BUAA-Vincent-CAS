#!/usr/bin/env bash
# ============================================================================
# 判决实验 (2026-10-12): **同一份 PX4, 换 jMAVSim 物理后端**, 跑同一个定点悬停探针。
#
# 目的: 在 Gazebo Classic 里实测到「真值−设定点 RMS 11.7/33.1/18.6cm, 真值漂移
#   0.98~3.52 cm/s」, 且 **PX4 自己的 AUTO_LOITER 也一样漂** ⇒ 怀疑是 Gazebo
#   Classic 的物理。jMAVSim 是 PX4 自带的**另一套物理引擎**(Java, 简化气动),
#   在同一份 PX4 固件/同一套控制器下跑:
#     · jMAVSim 停得住  ⇒ 罪魁 = Gazebo Classic 物理 (该换后端, 不必抄他们的栈)
#     · jMAVSim 也停不住 ⇒ 是 PX4 SITL 的共性问题 (才值得考虑 ArduPilot 栈)
#
# 注意: jMAVSim 没有 /model_states ⇒ **拿不到 Gazebo 真值**, 因此这里的判据是
#   `EKF − 设定点`(Gazebo 侧同口径为 9.6/26.0/18.2cm)。EKF 已被证明忠实于真值
#   (Δ 仅 2.5~11.2cm), 所以这个口径可比。
# ============================================================================
set -u
SITL_DIR="$HOME/SITL"
PX4_DIR="$HOME/PX4-Autopilot"
PX4_B="$PX4_DIR/build/px4_sitl_default"
RUN_DIR="$SITL_DIR/runs/jmav_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
echo "RUN_DIR=$RUN_DIR"

cleanup() {
    echo ""
    echo "=== 清理 ==="
    pkill -9 -f "jmavsim_run" 2>/dev/null || true
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "make px4_sitl" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    rm -f /tmp/cuadc_test_k.lock
    echo "  已清理"
}
trap cleanup EXIT INT TERM

echo "=== 1. 清理残留 ==="
pkill -9 -f "jmavsim_run" 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
sleep 2

echo ""
echo "=== 2. 重置 PX4 持久化状态 ==="
rm -f "$PX4_B/dataman" "$PX4_B/rootfs/dataman" "$PX4_B/rootfs/parameters.bson"
rm -rf "$PX4_B/rootfs/eeprom"
echo "  已重置"

echo ""
echo "=== 3. 启动 MicroXRCEAgent ==="
MicroXRCEAgent udp4 -p 8888 > "$RUN_DIR/agent.log" 2>&1 &
sleep 1

echo ""
echo "=== 4. 启动 PX4 + jMAVSim ==="
( cd "$PX4_DIR" && make px4_sitl ${JMAV_TARGET:-jmavsim_iris} ) > "$RUN_DIR/px4.log" 2>&1 &
for i in $(seq 1 60); do
    if pgrep -x px4 > /dev/null && pgrep -f jmavsim_run > /dev/null; then
        echo "  就绪 (第 $((i*5))s)"; break
    fi
    sleep 5
done
pgrep -x px4 > /dev/null || { echo "ERROR: px4 未起来"; tail -20 "$RUN_DIR/px4.log"; exit 1; }
pgrep -f jmavsim_run > /dev/null || { echo "ERROR: jmavsim 未起来"; tail -20 "$RUN_DIR/px4.log"; exit 1; }

echo ""
echo "=== 5. 等 DDS 话题 ==="
source "$SITL_DIR/install/setup.bash"
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        echo "  DDS 就绪 (第 ${i}s)"; break
    fi
    sleep 1
done

echo ""
echo "=== 6. 跑定点悬停探针 (与 Gazebo 侧同一组 holds) ==="
HOLDS_ARG="${PROBE_HOLDS:-0,0,12;2,32,12;0,0,12}"
echo "  holds=$HOLDS_ARG"
ros2 run drone_drl hover_probe --ros-args \
    -p "csv_path:=$RUN_DIR/hover.csv" \
    -p "holds:=$HOLDS_ARG" \
    -p "hold_mode:=${PROBE_MODE:-offboard}" \
    2>&1 | tee "$RUN_DIR/mission.log"

echo ""
echo "=== 7. 结束 ==="
echo "  产物: $RUN_DIR"
