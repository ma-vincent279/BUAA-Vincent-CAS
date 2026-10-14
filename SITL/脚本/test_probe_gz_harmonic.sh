#!/usr/bin/env bash
# ============================================================================
# 判决实验 Phase 1 (2026-10-12): **同一份 PX4 换 Gazebo Harmonic (gz-sim8) 后端**,
# 跑同一个定点悬停探针, 与 Gazebo Classic 的基准直接对比。
#
# 为什么做: 在 Gazebo Classic 里实测到「真值−设定点 RMS 11.7/33.1/18.6cm, 真值漂移
#   0.98~3.52cm/s」, 且 **PX4 自己的 AUTO_LOITER 也一样漂** ⇒ 怀疑物理后端。
#   Gazebo Harmonic 是 PX4 现役后端, 也是他们那套仿真用的引擎。
#
# 判据: 探针只用 /fmu/out/* (uXRCE-DDS), **不需要世界/相机/裁判** ⇒ 这一步很小、很干净。
#   · Harmonic 停得住  ⇒ 换后端有效, 进入 Phase 2 (迁移我们的场地与判定链)
#   · 也停不住        ⇒ 问题在 PX4 的估计/控制链路, 后端换不掉
#
# 前置: sudo apt install gz-harmonic   (装完首次 make 会自动生成 gz 目标)
#
# 用法:
#   bash scripts/test_probe_gz_harmonic.sh
#   GZ_MODEL=gz_x500_mono_cam_down bash scripts/test_probe_gz_harmonic.sh
#   PX4_PARAMS="EKF2_GPS_P_NOISE=0.02" bash scripts/test_probe_gz_harmonic.sh
# ============================================================================
set -u
SITL_DIR="$HOME/SITL"
PX4_DIR="$HOME/PX4-Autopilot"
PX4_B="$PX4_DIR/build/px4_sitl_default"
GZ_MODEL="${GZ_MODEL:-gz_x500}"
RUN_DIR="$SITL_DIR/runs/gz_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
echo "RUN_DIR=$RUN_DIR"
echo "机型   =$GZ_MODEL"

cleanup() {
    echo ""
    echo "=== 清理 ==="
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gz-sim" 2>/dev/null || true
    exec 3>&- 2>/dev/null || true
    rm -f /tmp/px4_gz_fifo.* /tmp/cuadc_test_k.lock
    # pxh 在非 tty stdin 下会狂刷 "pxh>" 提示符, px4.log 能涨到 160MB ⇒ 瘦身
    if [ -f "$RUN_DIR/px4.log" ]; then
        SIZE=$(stat -c%s "$RUN_DIR/px4.log" 2>/dev/null || echo 0)
        if [ "$SIZE" -gt 2000000 ]; then
            tr '\r' '\n' < "$RUN_DIR/px4.log" 2>/dev/null \
                | grep -av '^pxh>' | grep -av '^\[2K' | grep -av '^$' \
                > "$RUN_DIR/px4.filtered.log" 2>/dev/null || true
            mv -f "$RUN_DIR/px4.filtered.log" "$RUN_DIR/px4.log" 2>/dev/null || true
            echo "  px4.log 已瘦身 ($((SIZE/1048576))MB → $(($(stat -c%s "$RUN_DIR/px4.log" 2>/dev/null || echo 0)/1024))KB)"
        fi
    fi
    echo "  已清理"
}
trap cleanup EXIT INT TERM

echo ""
echo "=== 0. 前置检查 ==="
if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 找不到 gz 命令 —— 请先安装 Gazebo Harmonic:"
    echo "  sudo apt install -y gz-harmonic"
    exit 2
fi
# ⚠ 必须区分 Classic 的 `gz` CLI 与 Harmonic 的 `gz sim`:
#   Classic 的 /usr/bin/gz 对 `gz sim --version` 返回 exit 255, 且系统里没有 libgz-sim*
GZ_OK=0
if command -v gz >/dev/null 2>&1 && gz sim --version >/dev/null 2>&1; then
    GZ_OK=1; echo "  gz sim: $(gz sim --version 2>/dev/null | head -1)"
elif ls /usr/lib/x86_64-linux-gnu/libgz-sim*.so* >/dev/null 2>&1; then
    GZ_OK=1; echo "  检测到 libgz-sim*"
fi
if [ "$GZ_OK" -eq 0 ]; then
    echo "ERROR: 没检测到 **Gazebo Harmonic** (系统里只有 Gazebo Classic)"
    echo "  /usr/bin/gz 是 Classic 的 CLI; 没有 libgz-sim* / gz-sim 包"
    echo ""
    echo "  安装方法 (需要 sudo):"
    echo "    sudo apt install -y curl lsb-release gnupg"
    echo "    sudo curl https://packages.osrfoundation.org/gazebo.gpg \\"
    echo "         --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
    echo "    echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \\"
    echo "         http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main\" \\"
    echo "         | sudo tee /etc/apt/sources.list.d/gazebo-stable.list"
    echo "    sudo apt update && sudo apt install -y gz-harmonic"
    exit 2
fi
if [ ! -f "$PX4_B/rootfs/gz_env.sh" ]; then
    echo "  ⚠ 构建目录尚未配置 gz (缺 rootfs/gz_env.sh)"
    echo "    ⇒ 需要强制一次 CMake 重配:  touch src/modules/simulation/gz_bridge/CMakeLists.txt"
    echo "      然后 cd ~/PX4-Autopilot && make px4_sitl_default"
    exit 3
fi
echo "  gz 已配置 ✓ (build/px4_sitl_default/rootfs/gz_env.sh)"

echo ""
echo "=== 1. 清理残留 ==="
pkill -9 -x px4 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-sim" 2>/dev/null || true
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
echo "=== 4. 启动 PX4 + Gazebo Harmonic ==="
# ⚠ HEADLESS=1 才会跳过 `gz sim -g` (GUI) —— 见 ROMFS/.../init.d-posix/px4-rc.gzsim 第 52 行
#    另: PX4_GZ_WORLD / PX4_GZ_WORLDS / PX4_GZ_MODELS / PX4_GZ_MODEL_POSE 都是环境变量,
#    Phase 2 可用它们挂我们自己的世界与相机模型, **不必改 PX4 树**
FIFO=""
if [ -n "${PX4_PARAMS:-}" ]; then
    FIFO=/tmp/px4_gz_fifo.$$
    rm -f "$FIFO"; mkfifo "$FIFO"
    exec 3<> "$FIFO"          # 必须读写方式打开, 用 `3>` 会阻塞等读者 ⇒ 死锁
    ( cd "$PX4_DIR" && HEADLESS=1 make px4_sitl "$GZ_MODEL" ) < "$FIFO" > "$RUN_DIR/px4.log" 2>&1 &
else
    ( cd "$PX4_DIR" && HEADLESS=1 make px4_sitl "$GZ_MODEL" ) > "$RUN_DIR/px4.log" 2>&1 &
fi

for i in $(seq 1 120); do
    if pgrep -x px4 > /dev/null 2>&1; then
        echo "  px4 起来了 (第 $((i*5))s)"; break
    fi
    if grep -qiE "unknown target|error:" "$RUN_DIR/px4.log" 2>/dev/null; then
        echo "ERROR: PX4 启动失败:"; tail -15 "$RUN_DIR/px4.log"; exit 1
    fi
    sleep 5
done
pgrep -x px4 > /dev/null 2>&1 || { echo "ERROR: px4 未起来"; tail -20 "$RUN_DIR/px4.log"; exit 1; }
echo "  PX4 就绪 (Gazebo 可能仍在加载世界)"

echo ""
echo "=== 5. 等 DDS 话题 ==="
# ⚠ install/setup.bash 会引用未绑定变量 (COLCON_TRACE), 必须临时关掉 set -u
set +u
source "$SITL_DIR/install/setup.bash"
set -u
for i in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        echo "  DDS 就绪 (第 ${i}s)"; break
    fi
    sleep 1
done

echo ""
echo "=== 5.5 PX4 参数注入 (可选, PX4_PARAMS) ==="
if [ -n "${PX4_PARAMS:-}" ]; then
    for kv in $PX4_PARAMS; do echo "param set ${kv%%=*} ${kv#*=}" >&3; done
    for kv in $PX4_PARAMS; do echo "param show ${kv%%=*}" >&3; done
    sleep 2
    echo "  已注入: $PX4_PARAMS"
else
    echo "  (未注入)"
fi

echo ""
echo "=== 6. 跑定点悬停探针 (与 Classic 侧同一组 holds) ==="
HOLDS_ARG="${PROBE_HOLDS:-0,0,12;2,32,12;0,0,12}"
echo "  holds=$HOLDS_ARG"
ros2 run drone_drl hover_probe --ros-args \
    -p "csv_path:=$RUN_DIR/hover.csv" \
    -p "holds:=$HOLDS_ARG" \
    -p "hold_mode:=${PROBE_MODE:-offboard}" \
    2>&1 | tee "$RUN_DIR/mission.log"

echo ""
echo "=== 7. 本次结果 ==="
python3 "$SITL_DIR/scripts/analyze_probe_gate_20261012.py" "$RUN_DIR/hover.csv" 2>/dev/null || \
    python3 "$SITL_DIR/scripts/analyze_hover_probe.py" "$RUN_DIR/hover.csv" 2>/dev/null || \
    echo "  (分析脚本未跑通, CSV 在 $RUN_DIR/hover.csv)"

cat <<'REF'

────────────────────────────────────────────────────────────────────────
对照基准 (Gazebo Classic, 同 holds, 同探针口径):
  默认参数        HOLD0  9.6cm / HOLD1 26.0cm / HOLD2 18.2cm   ≤3cm 最长 1.35s
  EKF2_GPS_P_NOISE=0.02
                  HOLD0  4.6cm / HOLD1 15.4cm / HOLD2  8.5cm   ≤3cm 最长 0.00s
  (Gazebo Classic 的真值漂移 0.98~3.52 cm/s; PX4 自己的 LOITER 同样漂)
⇒ 若本轮 Harmonic 的数值明显低于上面两行 ⇒ 物理后端确实是瓶颈之一
────────────────────────────────────────────────────────────────────────
REF

echo ""
echo "=== 8. 结束 ==="
echo "  产物: $RUN_DIR"
