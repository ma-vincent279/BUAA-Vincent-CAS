#!/bin/bash
# ================================================================
# 部署 2026-10-09 静态标定常量到 Jetson Orin —— k.py + l.py 两份
# ----------------------------------------------------------------
# 标定账本 (全部来自 2026-10-09 现场静态实测, 推导见 docs/reports/calibration_to_code_20261009.md):
#   TARGET_PLANE_H = A − R = 0.022 − 0.300 = −0.278 m   (落地时镜头离地 − 桶口离地)
#   FX = FY = 355.05 px/m × 0.905 m = 321.1 px           (ρ 扫描四点 LSQ, 残差 1.2/−3.6/+2.9/−0.6 px)
#        ⇒ HFOV = 2·atan(320/321.1) = 89.80°
#   c_right = 32.30/355.05 = 9.10 cm ;  c_fwd = 2.08 cm
#   Y_B1 = c_right + 0.055 = +0.146 ;  Y_B2 = c_right − 0.055 = +0.036   (两瓶镜像挂载 ±5.5cm)
#   XF_B1 = XF_B2 = −c_fwd = −0.021
#
# ⚠ FX 只在【see.py 裁剪后的 640×360 发布帧】里成立:
#      see.py crop_fov=True, ratio = tan(120/2)/tan(136/2) = 0.6998
#      ⇒ 发布帧 FOV = 2·atan(0.6998·tan(110/2)) = 89.97°  ≈ 实测 89.80°  ✓ 同一帧
#   ⇒ 若把 crop_fov 关掉或改比值, 发布帧 FOV 变成相机全幅 FOV, FX 需整体重标。
#   本脚本第 8 步会核对 see.py 的 crop_fov/ratio, 不符则报警。
#
# 用法:
#   bash scripts/deploy_calib_jetson_20261009.sh              # 全流程 (自检 → 备份 → 推送 → 编译 → 校验)
#   bash scripts/deploy_calib_jetson_20261009.sh --precheck   # 只做部署前自检 (本地 + 板载只读), 不写任何东西
#   bash scripts/deploy_calib_jetson_20261009.sh --rollback   # 用最近一次 tar 还原板载 src/fly + install/fly 并重编
#
# 幂等: 重复运行只刷新 k.py/l.py 并复核, 不会重复插入 setup.py 入口, 不会重复改航向行。
# 保现场值: 板载 HEADING_DEG=212.0 由第 4 步规范化保住; 仓库 fly 基线仍是 180.0。
# ================================================================
set -e

JETSON="nvidia@192.168.98.136"
export SSHPASS="${SSHPASS:?SET_SSHPASS}"
SSOC="StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SSH="sshpass -e ssh -o $SSOC $JETSON"
SCP="sshpass -e scp -o $SSOC"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_K="$HERE/fly/src/fly/fly/k.py"
LOCAL_L="$HERE/fly/src/fly/fly/l.py"
SRC_K=/home/nvidia/fly/src/fly/fly/k.py
SRC_L=/home/nvidia/fly/src/fly/fly/l.py
# 与板载 k.py/i.py 逐字一致的航向行 (沿用 2026-10-08 的规范形式, 保证 k.py 的该行不再变动)
CANON='HEADING_DEG = 212.0  # 2026-10-08: 现场航向 (与板载 i.py 一致); 仓库 fly 基线是 180.0'

MODE=full
case "$1" in
    --precheck) MODE=precheck ;;
    --rollback) MODE=rollback ;;
    "") ;;
    *) echo "✗ 未知参数: $1"; exit 2 ;;
esac

if [ "$MODE" = rollback ]; then
    echo "=== 回退: 用最近一次 tar 还原 ~/fly 的 src/fly + install/fly 并重新编译 ==="
    $SSH 'T=$(ls -t ~/fly_backups/fly_src_install_*.tar.gz | head -1); echo "使用备份: $T";
          tar xzf "$T" -C ~/fly && echo "  还原完成";
          source /opt/ros/humble/setup.bash && cd ~/fly && colcon build --packages-select fly 2>&1 | tail -3
          echo "  还原后 sha:"; for f in i k l; do
            s=$(sha256sum ~/fly/src/fly/fly/$f.py 2>/dev/null | cut -c1-16)
            echo "    $f: src=${s:-无}"
          done'
    exit 0
fi

[ -f "$LOCAL_K" ] || { echo "✗ 本地 k.py 不存在: $LOCAL_K"; exit 1; }
[ -f "$LOCAL_L" ] || { echo "✗ 本地 l.py 不存在: $LOCAL_L"; exit 1; }

echo "=================================================================="
echo " 部署 2026-10-09 标定常量 → Jetson ($MODE)"
echo "=================================================================="

# ----------------------------------------------------------------
# 0. 本地自检 (常量一致性 + 与测量账本对账 + install sha + 期望板载 sha)
# ----------------------------------------------------------------
echo
echo "=== 0. 本地自检 ==="
python3 - "$HERE" <<'PY'
import ast, hashlib, math, pathlib, re, sys

ROOT = pathlib.Path(sys.argv[1]).resolve()
FILES = {
    "SITL k": ROOT / "src/drone_drl/drone_drl/k.py",
    "SITL l": ROOT / "src/drone_drl/drone_drl/l.py",
    "fly  k": ROOT / "fly/src/fly/fly/k.py",
    "fly  l": ROOT / "fly/src/fly/fly/l.py",
}
CAL = ["TARGET_PLANE_H", "HFOV", "VFOV", "FX", "FY",
       "PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_Y_B2",
       "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2"]
fail = []


def load(path):
    src = path.read_text(encoding="utf-8")
    ns = {"math": math}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in CAL + ["IMG_W", "IMG_H", "HEADING_DEG"] for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    return ns


V = {tag: load(p) for tag, p in FILES.items()}

# ---- 0.1 同侧两份文件一致 + 环境常量分侧各自正确 ----
# 2026-10-09: 其中 3 个常量是**环境/仿真器决定的**, SITL 与实机【应当不同】
#   TARGET_PLANE_H / HFOV / VFOV  (+ FX/FY 由 HFOV/VFOV 推导)
#   SITL: 0.0 / 85° / 54°  = 检测器 ground 投影与硬编码内参 (见 patch_sitl_env_20261009.py)
#   实机: -0.278 / 89.8° / 58.5° = 本次静态标定
#   机器核对: python3 scripts/check_drop_portability_20261009.py k l  (A2 节往返误差 ≤1px)
ENV = ("TARGET_PLANE_H", "HFOV", "VFOV", "FX", "FY")
bad = []
for side, tags in (("实机 fly", ["fly  k", "fly  l"]), ("SITL", ["SITL k", "SITL l"])):
    for n in CAL:
        if len({V[t][n] for t in tags}) != 1:
            bad.append(f"{side} 的 k/l 不一致: {n}")
if bad:
    fail.append(f"同侧标定常量不一致: {bad}")
print(f"  {'✓' if not bad else '✗'} 同侧 k/l 的 {len(CAL)} 个常量一致 (fly k==fly l; SITL k==SITL l)")

# 环境常量分侧核对
side_exp = {
    "SITL": {"TARGET_PLANE_H": 0.0, "HFOV_DEG": 85.0, "VFOV_DEG": 54.0},
    "fly": {"TARGET_PLANE_H": -0.278, "HFOV_DEG": 89.8, "VFOV_DEG": 58.5},
}
env_bad = []
for tag in FILES:
    e = side_exp[tag.split()[0]]
    v = V[tag]
    if abs(v["TARGET_PLANE_H"] - e["TARGET_PLANE_H"]) > 1e-9 or \
       abs(math.degrees(v["HFOV"]) - e["HFOV_DEG"]) > 1e-6 or \
       abs(math.degrees(v["VFOV"]) - e["VFOV_DEG"]) > 1e-6:
        env_bad.append(f"{tag}: plane={v['TARGET_PLANE_H']} HFOV={math.degrees(v['HFOV']):.1f} "
                       f"VFOV={math.degrees(v['VFOV']):.1f} (期望 {e})")
if env_bad:
    fail += env_bad
print(f"  {'✓' if not env_bad else '✗'} 环境常量分侧正确: SITL=TARGET_PLANE_H 0.0 / 85° / 54°; "
      f"实机=-0.278 / 89.8° / 58.5°")
for b in env_bad:
    print(f"      {b}")

# ---- 0.2 与测量账本对账 ----
ck = V["fly  k"]
A, R, SLOPE, H_EFF, PORT = 0.022, 0.300, 355.05, 0.905, 0.055
exp = {
    "TARGET_PLANE_H = A − R": (ck["TARGET_PLANE_H"], A - R, 1e-9),
    "FX = 斜率 × 深度": (ck["FX"], SLOPE * H_EFF, 1.0),
    "VFOV = 2atan((H/2)/FX)": (ck["VFOV"], 2 * math.atan((ck["IMG_H"] / 2) / ck["FX"]), 1e-3),
    "HFOV = 2atan((W/2)/FX)": (ck["HFOV"], 2 * math.atan((ck["IMG_W"] / 2) / ck["FX"]), 1e-6),
    "Y_B1 = c_right + 0.055": (ck["PAYLOAD_OFFSET_BODY_Y_B1"], 32.30 / SLOPE + PORT, 2e-3),
    "Y_B2 = c_right − 0.055": (ck["PAYLOAD_OFFSET_BODY_Y_B2"], 32.30 / SLOPE - PORT, 2e-3),
    "XF = −c_fwd": (ck["PAYLOAD_OFFSET_BODY_XF_B1"], -(7.375 * H_EFF / (SLOPE * H_EFF)), 2e-3),
}
for label, (got, want, tol) in exp.items():
    ok = abs(got - want) <= tol
    if not ok:
        fail.append(f"{label}: 实际 {got} vs 账本 {want}")
    print(f"  {'✓' if ok else '✗'} {label:<26} = {got:+.4f}   (账本 {want:+.4f}, 容差 {tol})")
if ck["PAYLOAD_OFFSET_BODY_XF_B1"] != ck["PAYLOAD_OFFSET_BODY_XF_B2"]:
    fail.append("XF_B1 != XF_B2")
print(f"  {'✗' if ck['PAYLOAD_OFFSET_BODY_XF_B1'] != ck['PAYLOAD_OFFSET_BODY_XF_B2'] else '✓'} "
      f"两瓶纵向前向常数相同 (XF_B1 == XF_B2)")

# ---- 0.3 SITL install 副本 == src ----
for letter in ("k", "l"):
    src = ROOT / f"src/drone_drl/drone_drl/{letter}.py"
    ins = sorted(ROOT.glob(f"install/drone_drl/lib/python3*/site-packages/drone_drl/{letter}.py"))
    a = hashlib.sha256(src.read_bytes()).hexdigest()
    b = hashlib.sha256(ins[0].read_bytes()).hexdigest() if ins else ""
    ok = a == b and bool(ins)
    if not ok:
        fail.append(f"SITL install/{letter}.py != src (需 colcon build)")
    print(f"  {'✓' if ok else '✗'} SITL install 副本与 src 一致: {letter}.py  {a[:16]}")

# ---- 0.4 期望板载 sha (本地 fly 文件把航向行换成 CANON 之后) ----
CANON = ('HEADING_DEG = 212.0  # 2026-10-08: 现场航向 (与板载 i.py 一致);'
         ' 仓库 fly 基线是 180.0')
env = []
for letter, p in (("k", FILES["fly  k"]), ("l", FILES["fly  l"])):
    t = p.read_text(encoding="utf-8")
    lines = [l for l in t.split("\n") if l.startswith("HEADING_DEG")]
    if len(lines) != 1:
        fail.append(f"{p.name}: HEADING_DEG 命中 {len(lines)} 次")
        continue
    if not lines[0].startswith("HEADING_DEG = 180.0"):
        fail.append(f"{p.name}: 仓库基线航向不是 180.0 ⇒ 规范化前提不成立")
    t2 = t.replace(lines[0], CANON, 1)
    h = hashlib.sha256(t2.encode("utf-8")).hexdigest()
    print(f"  ✓ 期望板载 {letter}.py sha (航向规范为 212.0 之后) = {h[:16]}")
    env.append(f"EXPECT_{letter.upper()}={h}")
pathlib.Path("/tmp/dsh_expected_sha.env").write_text("\n".join(env) + "\n", encoding="utf-8")

if fail:
    print("\n✗ 本地自检失败:")
    for f in fail:
        print("   -", f)
    sys.exit(1)
print("  ── 本地自检全部通过 ──")
PY

# ----------------------------------------------------------------
# 1. 板载只读自检 (记录将被覆盖的现场值; 核对 FX 有效性的前提)
# ----------------------------------------------------------------
echo
echo "=== 1. 板载只读自检 (不改任何东西) ==="
$SSH 'set -e
echo "  磁盘可用: $(df -h ~ | awk "NR==2{print \$4}")"
echo "  ── 将被覆盖的文件现值 ──"
for f in k l; do
  p=~/fly/src/fly/fly/$f.py
  if [ -f "$p" ]; then echo "    $f.py 存在  sha=$(sha256sum $p | cut -c1-16)  $(grep -m1 "^HEADING_DEG" $p | cut -d"#" -f1)"; \
  else echo "    $f.py 不存在 (首次部署)"; fi
done
echo "  ── 必须保住 / 不许动的文件 ──"
echo "    i.py sha=$(sha256sum ~/fly/src/fly/fly/i.py | cut -c1-16)  $(grep -m1 "^HEADING_DEG" ~/fly/src/fly/fly/i.py | cut -d"#" -f1)"
echo "    see.py sha=$(sha256sum ~/fly/src/fly/fly/see.py | cut -c1-16)"
echo "  ── FX 有效前提: see.py 裁剪 ──"
python3 - <<\PY
import math, re, pathlib
t = pathlib.Path("/home/nvidia/fly/src/fly/fly/see.py").read_text(encoding="utf-8")
def par(name):
    m = re.search(r"declare_parameter\(.%s.,\s*([0-9.]+|True|False)\)" % name, t)
    return m.group(1)
crop, old, new = par("crop_fov"), float(par("crop_fov_old")), float(par("crop_fov_new"))
r = math.tan(math.radians(old / 2)) / math.tan(math.radians(new / 2))
print(f"    crop_fov={crop}  {old:g}->{new:g}  ratio={r:.4f}")
pub = 2 * math.degrees(math.atan(r * math.tan(math.radians(110 / 2))))
print(f"    ⇒ 若相机 110° ⇒ 发布帧 FOV={pub:.2f}°  (标定实测 89.80° ⇒ 同一帧 ✓)")
if crop != "True" or abs(r - 0.6998) > 0.01:
    print("    ⚠⚠ crop_fov 与标定假设不符 ⇒ FX=321.1 失效, 必须先解决再飞!")
PY'

if [ "$MODE" = precheck ]; then
    echo
    echo "=== --precheck 完成: 未写入任何东西 ==="
    exit 0
fi

# ----------------------------------------------------------------
# 2. 板载备份
# ----------------------------------------------------------------
echo
echo "=== 2. 板载备份 (src/fly + install/fly) ==="
$SSH 'TS=$(date +%Y%m%d_%H%M%S); mkdir -p ~/fly_backups
      tar czf ~/fly_backups/fly_src_install_${TS}.tar.gz -C ~/fly src/fly install/fly
      echo "  ✓ ~/fly_backups/fly_src_install_${TS}.tar.gz ($(du -h ~/fly_backups/fly_src_install_${TS}.tar.gz | cut -f1))"'

# ----------------------------------------------------------------
# 3. 推送 k.py + l.py
# ----------------------------------------------------------------
echo
echo "=== 3. 推送 k.py + l.py ==="
$SCP "$LOCAL_K" "$JETSON:$SRC_K"
$SCP "$LOCAL_L" "$JETSON:$SRC_L"
for f in k l; do
    echo "  $f.py  本地 $(sha256sum "$HERE/fly/src/fly/fly/$f.py" | cut -c1-16)  →  板载 $($SSH "sha256sum /home/nvidia/fly/src/fly/fly/$f.py | cut -c1-16")"
done

# ----------------------------------------------------------------
# 4. 航向规范化 (→ 212.0, 与板载 i.py 一致)
# ----------------------------------------------------------------
echo
echo "=== 4. 航向规范化 (HEADING_DEG → 212.0) ==="
$SSH "python3 - <<'PY'
import shutil, datetime, pathlib
CANON = '$CANON'
for f in ('k', 'l'):
    p = pathlib.Path(f'/home/nvidia/fly/src/fly/fly/{f}.py')
    t = p.read_text(encoding='utf-8')
    lines = [l for l in t.split(chr(10)) if l.startswith('HEADING_DEG')]
    assert len(lines) == 1, f'{f}.py: HEADING_DEG 命中 {len(lines)} 次'
    if lines[0] == CANON:
        print(f'  · {f}.py 航向行已是规范形式, 跳过')
        continue
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy2(p, f'{p}.bak.{ts}_heading212')
    t2 = t.replace(lines[0], CANON, 1)
    assert CANON in t2 and len([l for l in t2.split(chr(10)) if l.startswith('HEADING_DEG')]) == 1
    p.write_text(t2, encoding='utf-8')
    print(f'  ✓ {f}.py 航向规范化为 212.0 (备份 {p.name}.bak.{ts}_heading212)')
PY"

echo "  ── 逐字节核对: 板载 == 本地(仅航向行换成 212.0) ──"
. /tmp/dsh_expected_sha.env
$SSH "sha256sum $SRC_K $SRC_L" > /tmp/dsh_board_sha.txt
BK=$(grep -F '/k.py' /tmp/dsh_board_sha.txt | awk '{print $1}')
BL=$(grep -F '/l.py' /tmp/dsh_board_sha.txt | awk '{print $1}')
[ "$BK" = "$EXPECT_K" ] && echo "    ✓ k.py 与本地基线只差航向行 (该行=212.0)" || { echo "    ✗ k.py 板载 sha $BK != 期望 $EXPECT_K"; exit 1; }
[ "$BL" = "$EXPECT_L" ] && echo "    ✓ l.py 与本地基线只差航向行 (该行=212.0)" || { echo "    ✗ l.py 板载 sha $BL != 期望 $EXPECT_L"; exit 1; }
$SSH 'grep -h "^HEADING_DEG" ~/fly/src/fly/fly/{i,k,l}.py | sed "s/^/    /"'

# ----------------------------------------------------------------
# 5. setup.py 入口 (k / l, 幂等)
# ----------------------------------------------------------------
echo
echo "=== 5. setup.py 入口 ==="
$SSH "python3 - <<'PY'
import shutil, datetime, pathlib
p = pathlib.Path('/home/nvidia/fly/src/fly/setup.py'); t = p.read_text(encoding='utf-8')
need = []
if \"'k = fly.k:main'\" not in t: need.append(('k', \"'i = fly.i:main',\"))
if \"'l = fly.l:main'\" not in t: need.append(('l', None))
if not need:
    print('  · 已有 k / l 入口, 跳过')
else:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy2(p, f'{p}.bak.{ts}_kl_entry')
    for f, anchor in need:
        anchor = anchor or \"'k = fly.k:main',\"
        assert t.count(anchor) == 1, f'{f} 入口锚点不唯一: {anchor}'
        t = t.replace(anchor, anchor + chr(10) + \"            '%s = fly.%s:main',\" % (f, f), 1)
        print(f'  ✓ 插入 {f} 入口 (锚点 {anchor})')
    p.write_text(t, encoding='utf-8')
    print(f'  (备份 setup.py.bak.{ts}_kl_entry)')
PY"
$SSH 'grep -n "fly.\(i\|k\|l\):main" ~/fly/src/fly/setup.py | sed "s/^/    /"'

# ----------------------------------------------------------------
# 6. 生成 l.sh (由板载 i.sh 派生, 保留现场专有路径/环境)
# ----------------------------------------------------------------
echo
echo "=== 6. 生成 ~/fly/l.sh ==="
$SSH "python3 - <<'PY'
import pathlib
src = pathlib.Path('/home/nvidia/fly/i.sh'); dst = pathlib.Path('/home/nvidia/fly/l.sh')
t = src.read_text(encoding='utf-8')
REP = [
    ('# DDS + see + i 一键启动脚本', '# DDS + see + l 一键启动脚本 (l = 全任务; 2026-10-09 标定常量)'),
    ('# 用法: ./i.sh', '# 用法: ./l.sh'),
    ('fly_logs/i_mission_', 'fly_logs/l_mission_'),
    ('i_mission_\$TIMESTAMP', 'l_mission_\$TIMESTAMP'),
    ('# ---- 3. 启动 i 任务节点', '# ---- 3. 启动 l 任务节点'),
    ('启动 i 双桶投水+侦察任务节点 (前台)', '启动 l 全任务节点 (投放区 30m + 侦察区动作序列) (前台)'),
    ('ros2 run fly i &', 'ros2 run fly l &'),
    ('I_PID', 'L_PID'),
    ('  i (', '  l ('),
    ('  i PID=', '  l PID='),
]
for a, b in REP:
    assert a in t, f'锚点未命中: {a}'
    t = t.replace(a, b)
assert 'ros2 run fly l &' in t and 'i_mission_' not in t and 'I_PID' not in t
dst.write_text(t, encoding='utf-8')
dst.chmod(0o755)
print('  ✓ ~/fly/l.sh 已生成 (755), 由 i.sh 派生')
PY"
$SSH 'bash -n ~/fly/l.sh && echo "    ✓ l.sh 语法 OK"; echo "    i.sh → l.sh 差异:"; diff ~/fly/i.sh ~/fly/l.sh | grep "^[<>]" | sed "s/^/      /"'

# ----------------------------------------------------------------
# 7. 编译
# ----------------------------------------------------------------
echo
echo "=== 7. colcon build ==="
$SSH 'source /opt/ros/humble/setup.bash && cd ~/fly && colcon build --packages-select fly 2>&1 | tail -3'

# ----------------------------------------------------------------
# 8. 校验: src==install / 可执行 / 运行时常量回读 / see.py 前提
# ----------------------------------------------------------------
echo
echo "=== 8. 板载校验 ==="
$SSH 'set -e
source /opt/ros/humble/setup.bash
cd ~/fly
for f in i k l; do
  A=$(sha256sum src/fly/fly/$f.py | cut -d" " -f1)
  INS=$(find install/fly -name "$f.py" -path "*fly*" | head -1)
  B=$(sha256sum "$INS" 2>/dev/null | cut -d" " -f1)
  SZ=$(stat -c%s "$INS" 2>/dev/null || echo 0)
  if [ "$A" = "$B" ] && [ "$SZ" -gt 1000 ]; then echo "  ✓ $f.py  src==install (${A:0:16}, ${SZ}B)"; else echo "  ✗ $f.py  src/install 不一致 (src=${A:0:16} inst=${B:0:16} sz=$SZ)"; exit 1; fi
done
echo "  install 可执行: $(ls install/fly/lib/fly/ | grep -cx k) 个 k, $(ls install/fly/lib/fly/ | grep -cx l) 个 l"
source install/setup.bash
python3 - <<\PY
import math, sys
import fly.k as K, fly.l as L
BAD = []
for m, tag in ((K, "k"), (L, "l")):
    exp = dict(TARGET_PLANE_H=(-0.278, 1e-6), Y_B1=(+0.146, 1e-6), Y_B2=(+0.036, 1e-6),
               XF_B1=(-0.021, 1e-6), XF_B2=(-0.021, 1e-6), HFOV=(math.radians(89.8), 1e-6),
               VFOV=(math.radians(58.5), 1e-6), FX=(321.119, 1e-3), HEADING_DEG=(212.0, 0.0))
    got = dict(TARGET_PLANE_H=m.TARGET_PLANE_H, Y_B1=m.PAYLOAD_OFFSET_BODY_Y_B1,
               Y_B2=m.PAYLOAD_OFFSET_BODY_Y_B2, XF_B1=m.PAYLOAD_OFFSET_BODY_XF_B1,
               XF_B2=m.PAYLOAD_OFFSET_BODY_XF_B2, HFOV=m.HFOV, VFOV=m.VFOV,
               FX=m.FX, HEADING_DEG=m.HEADING_DEG)
    for key, (want, tol) in exp.items():
        if abs(got[key] - want) > tol:
            BAD.append(f"{tag}.{key}={got[key]} != {want}")
    fx = (m.IMG_W / 2) / math.tan(m.HFOV / 2)
    if abs(fx - m.FX) > 1e-3:
        BAD.append(f"{tag}: FX={m.FX} 与 HFOV={math.degrees(m.HFOV):.2f}deg 不自洽 ({fx:.3f})")
    print(f"  {tag}  TARGET_PLANE_H={m.TARGET_PLANE_H:+.3f}  HFOV={math.degrees(m.HFOV):.2f}deg  "
          f"FX={m.FX:.1f}  Y_B1={m.PAYLOAD_OFFSET_BODY_Y_B1:+.3f}  Y_B2={m.PAYLOAD_OFFSET_BODY_Y_B2:+.3f}  "
          f"XF_B1={m.PAYLOAD_OFFSET_BODY_XF_B1:+.3f}  XF_B2={m.PAYLOAD_OFFSET_BODY_XF_B2:+.3f}  "
          f"HEADING_DEG={m.HEADING_DEG}")
if BAD:
    print("  FAIL 运行时常量回读失败:"); [print("     -", b) for b in BAD]; sys.exit(1)
print("  OK 运行时读回: 9 个标定常量 + 航向 212.0 全部与账本一致")
PY
echo "  see.py 前提: $(grep -c "declare_parameter(.crop_fov., True)" src/fly/fly/see.py) 处 crop_fov=True (需为 1)"'

echo
echo "=================================================================="
echo "部署完成"
echo "  现场启动: cd ~/fly && ./k.sh   (小场地投放精度台, 单桶 3m 两投 5s/30s)"
echo "            cd ~/fly && ./l.sh   (全任务: 投放区 30m + 侦察区动作序列)"
echo "  日志: ~/fly_logs/{k,l}_mission_*.log"
echo "  回退: bash scripts/deploy_calib_jetson_20261009.sh --rollback"
echo "  几何开关: TARGET_PLANE_H=-0.278 生效; L_BODY_GATE_M=0.0 / J_DEV_GATE_M=0.00 仍关闭"
# ⚠ 剖面文案**从板载代码动态读**, 不再写死 (前几轮写死的横幅误导过三次)
$SSH "cd ~/fly && source install/setup.bash >/dev/null 2>&1 && python3 -c \"import fly.k as m; print(f'  k 剖面: 巡航 {m.K_FLIGHT_ALT}m -> 前进 {m.FORWARD_DIST}m @ {m.K_FLIGHT_SPEED}m/s -> 降到 {m.PRE_SERVO_HEIGHT}m 投弹 (drop {m.DROP_HEIGHT}m); 视觉门 {m.K_VISUAL_GATE_M}m; 无进展保护 {m.K_FWD_STALL_S}s/{m.K_FWD_STALL_M}m')\""
echo "=================================================================="
