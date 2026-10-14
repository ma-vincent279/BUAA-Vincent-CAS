#!/bin/bash
# ================================================================
# 项目 k — 一键启动 (2026-10-11 新建, 基底 = i, 只替换 DROP 段)
# ----------------------------------------------------------------
# 与 test_i.sh 的差异 (均为本次对比后按规则/按他们的做法对齐):
#   1. 场地 = worlds/k.world, 由 `scripts/gen_world_cuadc.py` **生成**,
#      3 个桶 15/20/25cm (规则三档), 高 30cm, 白色, 落在投放区 NED y∈[30,35]
#   2. 检测器与裁判的桶坐标**都从 `worlds/k.truth.json` 读** —— 不再手抄三处
#   3. 裁判换成 `cuadc_judge` (规则原样计分: 落桶内 = 500/300/100, 距桶心≤0.5m = 50)
#   4. 跑一轮落一个**独立产物目录** runs/k_<seed>_<ts>/ (任务日志/裁判/真值/摘要),
#      且加 `trap` 按进程组收尾 (旧脚本只杀 agent, 会留 PX4+gzserver)
# 用法: bash ~/SITL/scripts/test_k.sh            (默认 = Gazebo Harmonic)
#       SIM=classic bash scripts/test_k.sh     (旧引擎, 需自行装 gazebo classic)
#       SIM_GUI=1 bash scripts/test_k.sh       (打开 Gazebo 图形界面)
#       GT_JUDGE=1 bash scripts/test_k.sh      (真值口径打分, 额外起 pose 桥 + impact_gt)
#       bash scripts/test_k.sh hover_probe    (悬停特性探针)
# 在哪个目录跑都行 (脚本内部用绝对路径 SITL_DIR)。
# ================================================================
set -e

# ★ 自定位: 脚本在哪, 仓库根就在哪 (别人 clone 到别的路径也能跑)
#   可用环境变量覆盖: SITL_DIR=... PX4_DIR=... bash scripts/test_k.sh
SITL_DIR="${SITL_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_ROOTFS="$PX4_BUILD/rootfs"
# 世界可覆盖 (隔离实验用): WORLD_FILE=/abs/path/world.world bash scripts/test_k.sh hover_probe
WORLD="${WORLD_FILE:-$SITL_DIR/worlds/k.world}"
TRUTH="${TRUTH_FILE:-$SITL_DIR/worlds/k.truth.json}"   # 多 seed 复现: TRUTH_FILE=… 覆盖

[ -f "$WORLD" ] || { echo "ERROR: 缺 $WORLD —— 先跑 python3 scripts/gen_world_cuadc.py --name k --seed 7"; exit 2; }
[ -f "$TRUTH" ] || { echo "ERROR: 缺 $TRUTH"; exit 2; }

# ---- 产物目录 (每次运行独立, 不互相覆盖) ----
SEED=$(python3 -c "import json;print(json.load(open('$TRUTH'))['seed'])")
TS=$(date +%Y%m%d_%H%M%S)
RUN_DIR="$SITL_DIR/runs/k_${SEED}_${TS}"
mkdir -p "$RUN_DIR"
cp "$TRUTH" "$RUN_DIR/truth.json"
cp "$WORLD" "$RUN_DIR/world.sdf"
echo "RUN_DIR=$RUN_DIR" | tee "$RUN_DIR/RUN_INFO.txt"

AGENT_PID=""
cleanup() {
    echo ""
    echo "=== 9. 清理 (trap) ==="
    [ -n "$AGENT_PID" ] && kill "$AGENT_PID" 2>/dev/null || true
    # 真节点是包装进程的子进程 ⇒ 必须按可执行路径补杀 (2026-10-09 追加 33 的教训)
    pkill -9 -f "drone_drl" 2>/dev/null || true
    pkill -9 -x "px4" 2>/dev/null || true
    pkill -9 -x "gzserver" 2>/dev/null || true
    pkill -9 -x "gzclient" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    exec 3>&- 2>/dev/null || true
    rm -f /tmp/px4_pxh_fifo.* /tmp/cuadc_test_k.lock
    echo "  已清理"
}
trap cleanup EXIT INT TERM

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

# ---- 并发守卫 (2026-10-12 加): 两个实例同时跑会让裁判收到重复落点, 分数作废 ----
#  起因: 有一次本脚本的 `pkill -f drone_drl` 把自己的父 shell 杀了, 但脚本本身继续跑,
#        变成孤儿进程又跑了一整轮 ⇒ 同一 DDS 域里两个节点, 裁判收到 4 条落点。
LOCK=/tmp/cuadc_test_k.lock
if [ -f "$LOCK" ]; then
    OLD_PID=$(cat "$LOCK" 2>/dev/null)
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "ERROR: 已有另一个 test_k.sh 在跑 (PID $OLD_PID) ⇒ 拒绝启动 (并发会污染裁判)"
        exit 10
    fi
    echo "  (清理陈旧的锁文件: PID $OLD_PID 已不存在)"
fi
echo $$ > "$LOCK"
# 清理后若仍有 PX4/gzserver 存活 ⇒ 说明有并行实例或顽固残留, 拒绝启动
if pgrep -x px4 > /dev/null || pgrep -x gzserver > /dev/null; then
    echo "ERROR: 清理后仍有 px4/gzserver 存活 ⇒ 拒绝启动 (可能有并发实例)"
    rm -f "$LOCK"
    exit 11
fi

echo ""
echo "=== 2. 清理旧日志 + 重置 PX4 持久化状态 ==="
rm -rf "$PX4_ROOTFS/log"/*/ 2>/dev/null || true
rm -f "$PX4_BUILD/dataman" "$PX4_ROOTFS/dataman" "$PX4_ROOTFS/parameters.bson" \
      "$PX4_ROOTFS/parameters_backup.bson"
rm -rf "$PX4_ROOTFS/eeprom"
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
export PX4_PARAM_COM_OF_LOSS_T=10.0
export PX4_PARAM_COM_LOW_BAT_ACT=0
export PX4_SITL_WORLD="$WORLD"
echo "  PX4 状态已重置; World: $WORLD (seed=$SEED)"

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

# ---------------------------------------------------------------------------
# 后端选择 (2026-10-12 新增): SIM=classic (默认, 逐字节保持原行为) | SIM=gz (Gazebo Harmonic)
#   gz 侧用的是**我们自己的**模型与场地:
#     PX4_GZ_MODELS=$SITL_DIR/gz_models      → x500 = iris 质量惯量 + 我们的 130° 下视相机
#     PX4_GZ_WORLDS=$SITL_DIR/gz_worlds      → k_gz.sdf = 由 gen_world_cuadc_gz.py 从 k.world 转来的场地
#     PX4_GZ_MODEL_POSE=1.01,0.98,0.83       → 与 Classic 生成点对齐 ⇒ 全部坐标与 truth.json 不用改
#   两个 gz 专属参数 (写成 default, 可用 PX4_PARAMS 覆盖):
#     CBRK_SUPPLY_CHK=894281  gz 的 x500 没有电源模块, 关掉电源检查
#     NAV_DLL_ACT=0           gz 机型硬设成 2(启用数据链检查), 没有 GCS 就拒绝解锁
# ---------------------------------------------------------------------------
# ⚠ 2026-10-14: 默认由 `classic` 改成 **`gz`**。
#   原因: 2026-10-12 已整体迁到 **Gazebo Harmonic (gz-sim8)**, Classic 不再安装 ——
#   默认走 classic 时 PX4 会在 `sitl_run` 里直接死掉
#   (`You need to have gazebo simulator installed!`), 而且是在**轮询 600s 之后**才暴露
#   ⇒ 看上去就像"仿真启动不了"。要用旧引擎: `SIM=classic bash scripts/test_k.sh`。
SIM="${SIM:-gz}"
if [ "$SIM" = "classic" ] && [ ! -x /usr/bin/gzserver ]; then
    # 提前判掉 —— 不要等 PX4 轮询 600s 才失败
    echo "ERROR: 指定了 SIM=classic, 但本机没装 Gazebo Classic。" >&2
    echo "       Gazebo Classic 已于 2026-10-12 弃用 (整体迁到 Harmonic)。" >&2
    echo "       ⇒ 去掉 SIM=classic 即可走默认的 gz(Harmonic):  bash scripts/test_k.sh" >&2
    exit 2
fi
if [ "$SIM" = "gz" ]; then
    PX4_MODEL="${PX4_MODEL:-gz_x500}"
    export PX4_GZ_MODELS="${PX4_GZ_MODELS:-$SITL_DIR/gz_models}"
    export PX4_GZ_WORLDS="${PX4_GZ_WORLDS:-$SITL_DIR/gz_worlds}"
    export PX4_GZ_WORLD="${GZ_WORLD:-k_gz}"
    # GT_JUDGE=1 ⇒ 额外起一条**真值链**: ros_gz_bridge 把 gz 的 /world/<w>/pose/info
    #   (SceneBroadcaster 发布的全部模型真值姿态) 转成 TFMessage, impact_gt 在每次投放时
    #   记下飞机的**真值**位置并发布 /drop/impact_gt ⇒ judge 用它打分, 得到**真值口径**分数。
    #   目的: 把"视觉链路误差"与"EKF 自报位置误差"分开 (SITL 的 /drop/impact_rel 是 EKF 口径)。
    export GT_JUDGE="${GT_JUDGE:-0}"
   # 多 seed 复现: GZ_WORLD=… 覆盖
    # 生成点可覆盖 (跑**他们的场地**时要用他们的原点 (0,0); 默认与 Classic 对齐)
    export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-1.01,0.98,0.83}"
    # SIM_GUI=1 ⇒ **打开 Gazebo 图形界面** (默认无界面); 前台跑时可肉眼观察
    if [ "${SIM_GUI:-0}" = "1" ]; then
        echo "  ⚠ SIM_GUI=1 ⇒ 会拉起 Gazebo GUI (需要 DISPLAY)"
    else
        export HEADLESS=1
    fi
    PX4_PARAMS="${PX4_PARAMS:-} CBRK_SUPPLY_CHK=894281 NAV_DLL_ACT=0"
    echo "  后端: gz (Gazebo Harmonic)"
    echo "    PX4_GZ_MODELS = $PX4_GZ_MODELS"
    echo "    PX4_GZ_WORLDS= $PX4_GZ_WORLDS   world=$PX4_GZ_WORLD"
    echo "    PX4_GZ_MODEL_POSE = $PX4_GZ_MODEL_POSE (与 Classic 生成点一致)"
else
    # 机型可覆盖 (判决实验用): PX4_MODEL=gazebo-classic_iris bash scripts/test_k.sh hover_probe
    PX4_MODEL="${PX4_MODEL:-gazebo-classic_iris_mono_cam_down}"
    echo "  后端: classic (Gazebo Classic)"
fi
echo "  机型: $PX4_MODEL"
# 2026-10-12: 可选的 **PX4 参数注入** (零污染: 每轮都清 eeprom/parameters.bson)
#   用法: PX4_PARAMS="EKF2_GPS_P_NOISE=0.02" bash scripts/test_k.sh hover_probe
FIFO=""
if [ -n "${PX4_PARAMS:-}" ]; then
    FIFO=/tmp/px4_pxh_fifo.$$
    rm -f "$FIFO"; mkfifo "$FIFO"
    exec 3<> "$FIFO"         # ⚠ 必须用读写方式打开: `3>` 会阻塞等读者 ⇒ 死锁
                            #   (3<> 以 O_RDWR 打开, 不阻塞, 同时满足读/写两端)
    make px4_sitl "$PX4_MODEL" < "$FIFO" > "$RUN_DIR/px4.log" 2>&1 &
else
    make px4_sitl "$PX4_MODEL" &
fi
echo "  等待初始化 (轮询, 最多 600s)..."
if [ "$SIM" = "gz" ]; then
    WAIT_PROC="px4"          # gz (Harmonic) 没有 gzserver 进程, 只有 px4 + gz sim
else
    WAIT_PROC="gzserver"
fi
for i in $(seq 1 120); do
    if pgrep -x "$WAIT_PROC" > /dev/null && pgrep -x px4 > /dev/null; then
        echo "  初始化完成 (第 $((i*5))s)"; break
    fi
    sleep 5
done
pgrep -x "$WAIT_PROC" > /dev/null || { echo "ERROR: $WAIT_PROC failed"; exit 1; }
pgrep -x px4 > /dev/null || { echo "ERROR: px4 failed"; exit 1; }
echo "  PX4 + Gazebo 就绪 ($SIM)"

echo ""
echo "=== 6. 等待 DDS 话题 ==="
source "$SITL_DIR/install/setup.bash"
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status_v4"; then
        echo "  DDS 就绪"; break
    fi
    sleep 1
done

echo ""
echo "=== 6.5 PX4 参数注入 (可选, PX4_PARAMS) ==="
if [ -n "${PX4_PARAMS:-}" ]; then
    for kv in $PX4_PARAMS; do
        echo "param set ${kv%%=*} ${kv#*=}" >&3
    done
    for kv in $PX4_PARAMS; do
        echo "param show ${kv%%=*}" >&3
    done
    sleep 2
    echo "  已注入: $PX4_PARAMS"
    grep -aE '^EKF2_|^MPC_' "$RUN_DIR/px4.log" | tail -8 | sed 's/^/    回读: /'
else
    echo "  (未注入)"
fi

echo ""
echo "=== 7. 启动辅助节点 (参数全部由 truth.json 机器生成) ==="
source "$SITL_DIR/install/setup.bash"
# 桶参数: ned → gazebo (gz_x = ned_y + spawn_x, gz_y = ned_x + spawn_y)
BUCKET_ARGS=$(python3 - "$TRUTH" <<'PY'
import json, sys
t = json.load(open(sys.argv[1]))
sx, sy = t['spawn_gz']['x'], t['spawn_gz']['y']
out = []
for i, b in enumerate(t['buckets'], 1):
    gx, gy = b['ned_y'] + sx, b['ned_x'] + sy
    out += [f"-p bucket{i}_gz_x:={gx:.3f}", f"-p bucket{i}_gz_y:={gy:.3f}",
            f"-p bucket{i}_gz_z:=0.15", f"-p bucket{i}_radius:={b['radius']:.4f}"]
print(" ".join(out))
PY
)
echo "  桶参数: $BUCKET_ARGS"

# shellcheck disable=SC2086
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true -p conf_threshold:=0.3 -p bucket_count:=3 \
    -p focal_px:=171.205 -p img_width:=640 -p img_height:=360 \
    $BUCKET_ARGS > "$RUN_DIR/detector.log" 2>&1 &
sleep 0.5

# ---- GT_JUDGE=1: 真值链 (判定"视觉误差"vs"EKF 自报位置误差") ----
if [ "${GT_JUDGE:-0}" = "1" ]; then
    # gz 的真值姿态 (SceneBroadcaster) → TFMessage
    # ⚠ 用 **dynamic_pose** 而不是 pose/info: 后者每帧发**全部 107 个实体**, DDS 负载大到
    #   把 OFFBOARD setpoint 流挤掉 ⇒ 起飞段跑飞 (`Mode timeout`)。dynamic_pose 只发动态实体。
    # ⚠ **立即启动, 不要延迟** —— 曾经以为"起飞时挂桥会把 setpoint 流挤掉"而加了 30s 延迟,
    #   事后证明那是**误判**: 真凶是前几轮留下的 14 个 ros_gz_bridge 累积在灌 DDS
    #   (清干净后 GT 链与任务完全共存)。延迟反而让第 1 投时还没有真值位姿。
    ( ros2 run ros_gz_bridge parameter_bridge \
        "/world/$PX4_GZ_WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V" \
        > "$RUN_DIR/gz_pose_bridge.log" 2>&1 &
    sleep 1
    ros2 run drone_drl impact_gt --ros-args \
        -p pose_topic:="/world/$PX4_GZ_WORLD/dynamic_pose/info" \
        -p pose_topic_alt:="/world/$PX4_GZ_WORLD/pose/info" \
        -p model_name:=x500_0 -p trigger_topic:=/drop/impact_rel \
        -p out_topic:=/drop/impact_gt -p debug_frames:=1 \
        > "$RUN_DIR/impact_gt.log" 2>&1 &
    ) &
    sleep 0.5
    echo "  真值链已启动 (bridge + impact_gt); 裁判用 /drop/impact_gt"
fi
GT_IMPACT="${GT_IMPACT_TOPIC:-/drop/impact_rel}"
[ "${GT_JUDGE:-0}" = "1" ] && GT_IMPACT="/drop/impact_gt"

ros2 run drone_drl cuadc_judge --ros-args \
    -p truth_file:="$TRUTH" \
    -p impact_topic:="$GT_IMPACT" \
    -p allow_reuse:=false \
    -p summary_file:="$RUN_DIR/summary.json" \
    > "$RUN_DIR/judge.log" 2>&1 &
sleep 1
echo "  裁判判据口径: $GT_IMPACT"
echo "  辅助节点已启动 (裁判: $RUN_DIR/judge.log)"

echo ""
echo "=== 8. 启动 K 任务 (机体坐标系闭环伺服投弹) ==="
echo "  =========================================="
source "$SITL_DIR/install/setup.bash"
set +e
NODE="${1:-k}"                        # 首个参数 = 要跑的节点 (默认 k; 传 hover_probe 即悬停探针)
if [ "$NODE" = "hover_probe" ]; then
    HOLDS_ARG="${PROBE_HOLDS:-0,0,12;2,32,12;5,28,12;0,0,12}"
    ros2 run drone_drl "$NODE" --ros-args -p "csv_path:=$RUN_DIR/hover.csv" \
        -p "holds:=$HOLDS_ARG" -p "hold_mode:=${PROBE_MODE:-offboard}" \
        2>&1 | tee "$RUN_DIR/mission.log"
else
    ros2 run drone_drl "$NODE" 2>&1 | tee "$RUN_DIR/mission.log"
fi
set -e

echo ""
echo "=== 裁判打分 (规则: A区=桶内 500/300/100, B区=距桶心0.5m 50) ==="
cat "$RUN_DIR/judge.log" 2>/dev/null || echo "  (无裁判日志)"
echo ""
if [ -f "$RUN_DIR/summary.json" ]; then
    echo "=== 摘要 ==="
    cat "$RUN_DIR/summary.json"
fi
echo ""
echo "k 完成喵~ 产物目录: $RUN_DIR"
