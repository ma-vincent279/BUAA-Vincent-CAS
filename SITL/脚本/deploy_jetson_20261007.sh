#!/usr/bin/env bash
# ============================================================================
# 2026-10-07 现场部署：把最新 i 与外接盘 r3 版 f 装到 Jetson 并编译
#   ① 远端先备份 (整目录 tar + 单文件 .bak)
#   ② i.py  <- 本地 fly/src/fly/fly/i.py (sha 3322880f, 含 10-07 方案A/面积选桶/身份0.60)
#              + 仅把 HEADING_DEG 改成现场值 213
#   ③ f.py  <- 外接盘交付版 r3 (反向还原后 sha 已与交付清单 e1a32bf1 逐字节一致)
#              + 仅把 HEADING_DEG 改成现场值 213
#   ④ colcon build --packages-select fly,  并核对 src/install 逐位一致 + 关键常量
# 用法: bash scripts/deploy_jetson_20261007.sh
# 回滚: 见脚本末尾打印的命令 (单文件 .bak + 整包 tar)
# ============================================================================
set -euo pipefail

JETSON="nvidia@192.168.98.136"; PW="${SSHPASS:?SET_SSHPASS}"
REMOTE="/home/nvidia/fly"
RSRC="$REMOTE/src/fly/fly"
RINST="$REMOTE/install/fly/lib/python3.10/site-packages/fly"
STAMP="$(date +%Y%m%d_%H%M%S)"
HEADING="213.0"                     # 现场航向 (用户给定 213 度)
LOCAL_FLY="/home/vincent/SITL/fly/src/fly/fly"
OUT="/tmp/deploy_$STAMP"; mkdir -p "$OUT"
SSH=(sshpass -p "$PW" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$JETSON")
SCP=(sshpass -p "$PW" scp -o StrictHostKeyChecking=no -o ConnectTimeout=10)

echo "=== 0) 生成现场版两份文件 (本地) ==="
python3 - "$HEADING" "$OUT" <<'PY'
import hashlib, subprocess, sys
from pathlib import Path
heading, out = sys.argv[1], Path(sys.argv[2])
ROOT = Path('/home/vincent/SITL')

# ---- i.py: 本地最新版 + 只改 HEADING_DEG ----
i_src = ROOT / 'fly/src/fly/fly/i.py'
t = i_src.read_text(encoding='utf-8')
old = [l for l in t.split('\n') if l.startswith('HEADING_DEG =')]
assert len(old) == 1, old
new_line = (f'HEADING_DEG = {heading}  # 2026-10-07 现场航向 (用户给定 213°); '
            f'原 180.0 (仓库) / 161.0 (10-06 实飞)')
t2 = t.replace(old[0], new_line, 1)
assert t2 != t and t2.count(new_line) == 1
(out / 'i.py').write_text(t2, encoding='utf-8')
print(f"  i.py  : {old[0][:40]}...  ->  HEADING_DEG = {heading}")

# ---- f.py: 从 SITL 导入版反向还原成交付原版 (sha 必须等于交付清单值) ----
f_r3 = (ROOT / 'src/drone_drl/drone_drl/f_r3.py').read_text(encoding='utf-8')
lines = f_r3.split('\n')
k = 1
while lines[k].startswith('#'):
    k += 1
recon = '\n'.join(['#!/usr/bin/env python3'] + lines[k:])
REV = [
    ("class FR3Mission(Node):", "class FMission(Node):"),
    ("        super().__init__('f_r3_mission')", "        super().__init__('f_mission')"),
    ("    node = FR3Mission()", "    node = FMission()"),
    ('PX4_STATUS_SUFFIX = "_v4"   # 2026-10-07 SITL 导入: SITL 固件发布 vehicle_status_v4 (_v1=实机)',
     'PX4_STATUS_SUFFIX = "_v1"'),
    ("HEADING_DEG = 92.0  # 2026-10-07 SITL 导入: i.world 场地航向 (与 SITL i.py 一致); 原实机值 161.0",
     "HEADING_DEG = 161.0  # 沿用 2026-10-06 17:40 日志的场地方向；0=北、90=东"),
    ('MISSION_REVISION = "competition-water-20261006-r3-f-continuous-brake-hold-sitl-import"',
     'MISSION_REVISION = "competition-water-20261006-r3-f-continuous-brake-hold"'),
]
for a, b in REV:
    assert recon.count(a) == 1, f"反向锚点 {recon.count(a)} 次: {a[:50]}"
    recon = recon.replace(a, b, 1)
h = hashlib.sha256(recon.encode()).hexdigest()
assert h == 'e1a32bf1facaebe7533285614e87d9c8750704fbacfda20130df9131c77e3f9f', h
print(f"  f.py  : 反向还原 sha 与交付清单一致 ✓ {h[:16]}…")
oldf = [l for l in recon.split('\n') if l.startswith('HEADING_DEG =')]
assert len(oldf) == 1
recon = recon.replace(oldf[0],
    f'HEADING_DEG = {heading}  # 2026-10-07 现场部署: 161.0 -> 213.0 (现场航向由用户给定)',
    1)
(out / 'f.py').write_text(recon, encoding='utf-8')
for f in ('i.py', 'f.py'):
    r = subprocess.run([sys.executable, '-m', 'py_compile', str(out / f)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    print(f"  {f}: py_compile OK  sha256={hashlib.sha256((out/f).read_bytes()).hexdigest()}")
PY

echo
echo "=== 1) 远端现状 (覆盖前先确认, 防覆盖更新的东西) ==="
"${SSH[@]}" "sha256sum $RSRC/i.py $RSRC/f.py $RSRC/see.py 2>/dev/null; grep -n '^HEADING_DEG' $RSRC/i.py $RSRC/f.py; echo '--- install:'; sha256sum $RINST/i.py $RINST/f.py 2>/dev/null"

echo
echo "=== 2) 远端备份 ==="
"${SSH[@]}" "mkdir -p ~/fly_backups && \
  tar czf ~/fly_backups/fly_src_install_${STAMP}.tar.gz \
     -C $REMOTE src/fly/fly install/fly/lib/python3.10/site-packages/fly 2>/dev/null && \
  cd $RSRC && cp -v i.py i.py.bak.${STAMP}_predeploy && cp -v f.py f.py.bak.${STAMP}_predeploy && \
  cp -v see.py see.py.bak.${STAMP}_predeploy && \
  ls -la ~/fly_backups/ | tail -3"

echo
echo "=== 3) 上传两份新文件 ==="
"${SCP[@]}" "$OUT/i.py" "$OUT/f.py" "$JETSON:$RSRC/"
"${SSH[@]}" "sha256sum $RSRC/i.py $RSRC/f.py; grep -n '^HEADING_DEG' $RSRC/i.py $RSRC/f.py"
echo "  本地期望: i.py $(sha256sum "$OUT/i.py" | cut -c1-64) / f.py $(sha256sum "$OUT/f.py" | cut -c1-64)"

echo
echo "=== 4) 远端编译 ==="
"${SSH[@]}" "cd $REMOTE && source /opt/ros/humble/setup.bash && colcon build --packages-select fly 2>&1 | tail -6"

echo
echo "=== 5) 核对 src vs install 逐位一致 ==="
"${SSH[@]}" "sha256sum $RSRC/i.py $RINST/i.py $RSRC/f.py $RINST/f.py"

echo
echo "=== 6) 核对关键常量 (install 副本, 即实际运行的代码) ==="
"${SSH[@]}" "echo '--- i.py:'; grep -nE '^HEADING_DEG|^PAYLOAD_OFFSET_BODY_Y_B1|^PAYLOAD_OFFSET_BODY_Y_B2|^PAYLOAD_OFFSET_BODY_XF_B1|^POSITION_REFINE_IDENTITY_MAX|^POSITION_REFINE_ID_MATCH_M' $RINST/i.py; \
  echo \"  方案A/面积选桶标记: \$(grep -c '纯几何: 保持点-桶估计' $RINST/i.py)/\$(grep -c '面积降序取前2' $RINST/i.py)\"; \
  echo '--- f.py (r3):'; grep -nE '^MISSION_REVISION|^HEADING_DEG|^DROP_ARRIVE_M|^DROP_FORCE_AFTER|^LOW_SERVO_MAX_SPEED|^PAYLOAD_OFFSET_BODY_Y_B1|^PAYLOAD_OFFSET_BODY_Y_B2|^POSITION_REFINE_IDENTITY_MAX' $RINST/f.py"

echo
echo "=== 7) 回滚命令 (若需要) ==="
"${SSH[@]}" "echo '  cd $RSRC && cp i.py.bak.${STAMP}_predeploy i.py && cp f.py.bak.${STAMP}_predeploy f.py'; \
  echo '  cd $REMOTE && source /opt/ros/humble/setup.bash && colcon build --packages-select fly'; \
  echo '  或整包还原: tar xzf ~/fly_backups/fly_src_install_${STAMP}.tar.gz -C $REMOTE'"
echo
echo "✅ 部署完成 (时间戳 $STAMP)"
