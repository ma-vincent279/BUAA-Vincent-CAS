#!/bin/bash
# ================================================================
# 部署项目 k 到 Jetson Orin (2026-10-08) —— i 的高精度调参台
# ----------------------------------------------------------------
# 幂等: 重复运行只会刷新 k.py 与校验, 不会重复插入 setup.py 入口
# 五步:
#   ① 板载备份 (src/fly + install/fly 打 tar.gz 到 ~/fly_backups/)
#   ② scp 本地 fly k.py → ~/fly/src/fly/fly/k.py
#   ③ HEADING_DEG 180.0 → 212.0 (**与板载 i.py 一致**, 场地航向; 本地仓库保持 180 基线)
#   ④ ~/fly/src/fly/setup.py 插入入口 'k = fly.k:main' (备份 setup.py)
#   ⑤ colcon build --packages-select fly + 校验:
#        src==install 且 install 副本非空 / install 里有 k 可执行 / 运行时 import fly.k 读回常量
# 回退: bash scripts/deploy_k_jetson_20261008.sh --rollback   (用最近一次 tar 还原, 再编译)
# ================================================================
set -e

JETSON="nvidia@192.168.98.136"
export SSHPASS="${SSHPASS:?SET_SSHPASS}"
SSOC="StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SSH="sshpass -e ssh -o $SSOC $JETSON"
SCP="sshpass -e scp -o $SSOC"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_K="$HERE/fly/src/fly/fly/k.py"
SRC=/home/nvidia/fly/src/fly/fly/k.py

if [ ! -f "$LOCAL_K" ]; then echo "✗ 本地 k.py 不存在: $LOCAL_K"; exit 1; fi

if [ "$1" = "--rollback" ]; then
    echo "=== 回退: 用最近一次 tar 还原 ~/fly 的 src/fly + install/fly ==="
    $SSH 'T=$(ls -t ~/fly_backups/fly_src_install_*.tar.gz | head -1); echo "使用备份: $T";
          tar xzf "$T" -C ~/fly && echo "  还原完成";
          source /opt/ros/humble/setup.bash && cd ~/fly && colcon build --packages-select fly 2>&1 | tail -3'
    exit 0
fi

echo "=== 1. 板载备份 ==="
$SSH 'TS=$(date +%Y%m%d_%H%M%S); mkdir -p ~/fly_backups
      tar czf ~/fly_backups/fly_src_install_${TS}.tar.gz -C ~/fly src/fly install/fly
      echo "  ✓ ~/fly_backups/fly_src_install_${TS}.tar.gz ($(du -h ~/fly_backups/fly_src_install_${TS}.tar.gz | cut -f1))"'

echo "=== 2. 推送 k.py ==="
$SCP "$LOCAL_K" "$JETSON:$SRC"
echo "  本地 sha: $(sha256sum "$LOCAL_K" | cut -c1-16)"
echo "  板载 sha: $($SSH "sha256sum $SRC | cut -c1-16")"

echo "=== 3. 航向对齐 (HEADING_DEG → 212.0, 与板载 i.py 一致) ==="
$SSH "python3 - <<'PY'
import re, shutil, datetime, pathlib
CANON = 'HEADING_DEG = 212.0  # 2026-10-08: 现场航向 (与板载 i.py 一致); 仓库 fly 基线是 180.0'
p = pathlib.Path('$SRC'); t = p.read_text(encoding='utf-8')
ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
lines = [l for l in t.split(chr(10)) if l.startswith('HEADING_DEG')]
assert len(lines) == 1, f'HEADING_DEG 行命中 {len(lines)} 次'
if lines[0] == CANON:
    print('  · 航向行已是规范形式, 跳过')
else:
    shutil.copy2(p, f'{p}.bak.{ts}_heading212')
    t2 = t.replace(lines[0], CANON, 1)
    assert CANON in t2 and len([l for l in t2.split(chr(10)) if l.startswith('HEADING_DEG')]) == 1
    p.write_text(t2, encoding='utf-8')
    print('  ✓ 航向行规范化为: ' + CANON)
PY"

echo "=== 4. setup.py 入口 ==="
$SSH "python3 - <<'PY'
import shutil, datetime, pathlib
p = pathlib.Path('/home/nvidia/fly/src/fly/setup.py'); t = p.read_text(encoding='utf-8')
if \"'k = fly.k:main'\" in t:
    print('  · 已有 k 入口, 跳过')
else:
    assert t.count(\"'i = fly.i:main',\") == 1, '锚点 i 入口不唯一'
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy2(p, f'{p}.bak.{ts}_k_entry')
    t = t.replace(\"'i = fly.i:main',\", \"'i = fly.i:main',\n            'k = fly.k:main',\", 1)
    p.write_text(t, encoding='utf-8')
    print(f'  ✓ 已插入 k 入口 (备份 setup.py.bak.{ts}_k_entry)')
PY"

echo "=== 4b. 生成 k.sh (从板载 i.sh 派生, 现场一键启动) ==="
$SSH "python3 - <<'PY'
import pathlib, re
src = pathlib.Path('/home/nvidia/fly/i.sh'); dst = pathlib.Path('/home/nvidia/fly/k.sh')
t = src.read_text(encoding='utf-8')
REP = [
    ('# DDS + see + i 一键启动脚本', '# DDS + see + k 一键启动脚本 (k = i 的高精度调参台: 单桶 3m 正前方, 两投 5s/30s)'),
    ('# 用法: ./i.sh', '# 用法: ./k.sh'),
    ('fly_logs/i_mission_', 'fly_logs/k_mission_'),
    ('i_mission_\$TIMESTAMP', 'k_mission_\$TIMESTAMP'),
    ('# ---- 3. 启动 i 任务节点', '# ---- 3. 启动 k 任务节点'),
    ('启动 i 双桶投水+侦察任务节点 (前台)', '启动 k 单桶两投测试台 (投放链同 i, 不飞侦察区) (前台)'),
    ('ros2 run fly i &', 'ros2 run fly k &'),
    ('I_PID', 'K_PID'),
    ('  i (', '  k ('),
]
for a, b in REP:
    assert a in t, f'锚点未命中: {a}'
    t = t.replace(a, b)
assert 'ros2 run fly k &' in t and 'i_mission_' not in t and 'I_PID' not in t
dst.write_text(t, encoding='utf-8')
dst.chmod(0o755)
print('  ✓ ~/fly/k.sh 已生成 (755), 由 i.sh 派生')
PY"
$SSH 'bash -n ~/fly/k.sh && echo "  ✓ k.sh 语法 OK"; echo "  差异审计 (i.sh → k.sh):"; diff ~/fly/i.sh ~/fly/k.sh | grep "^[<>]" | sed "s/^/    /" | head -20'

echo "=== 5. 编译 ==="
$SSH 'source /opt/ros/humble/setup.bash && cd ~/fly && colcon build --packages-select fly 2>&1 | tail -4'

echo "=== 6. 校验 ==="
$SSH 'source /opt/ros/humble/setup.bash
INS=/home/nvidia/fly/install/fly/lib/python3.10/site-packages/fly/k.py
A=$(sha256sum /home/nvidia/fly/src/fly/fly/k.py | cut -d" " -f1)
B=$(sha256sum $INS 2>/dev/null | cut -d" " -f1)
SZ=$(stat -c%s $INS 2>/dev/null || echo 0)
echo "  src  sha: ${A:0:16}"
echo "  inst sha: ${B:0:16}  (大小 ${SZ} 字节)"
[ -n "$A" ] && [ "$A" = "$B" ] && [ "$SZ" -gt 1000 ] && echo "  ✓ src==install 且非空" || { echo "  ✗ src/install 不一致或 install 为空"; exit 1; }
echo "  install 可执行: $(ls /home/nvidia/fly/install/fly/lib/fly/ 2>/dev/null | grep -cx k) 个"
cd ~/fly && source install/setup.bash && python3 -c "
import fly.k as m
print(f\"  ✓ 运行时读回: K_DWELL_S1={m.K_DWELL_S1} K_DWELL_S2={m.K_DWELL_S2} FORWARD_DIST={m.FORWARD_DIST}\")
print(f\"    J_DEV_GATE_M={m.J_DEV_GATE_M} J_DEV_REFREEZE_M={m.J_DEV_REFREEZE_M} J_MEASURED_R={m.J_MEASURED_R}\")
print(f\"    HEADING_DEG={m.HEADING_DEG} 补偿B1=({m.PAYLOAD_OFFSET_BODY_Y_B1},{m.PAYLOAD_OFFSET_BODY_XF_B1}) 补偿B2=({m.PAYLOAD_OFFSET_BODY_Y_B2},{m.PAYLOAD_OFFSET_BODY_XF_B2})\")
print(f\"    I_RELEASE_TIMEOUT={m.I_RELEASE_TIMEOUT} SERVED_EXCLUDE_M={m.SERVED_EXCLUDE_M} MISSION_REVISION={m.MISSION_REVISION}\")
"'
echo ""
echo "完成: 现场 cd ~/fly && ./k.sh (DDS + see + k 一键; 日志 ~/fly_logs/k_mission_*.log)"; echo "     板载 k.py 与 i.py 同航向 212; i.py 未动"
