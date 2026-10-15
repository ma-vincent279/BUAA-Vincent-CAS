#!/bin/bash
# ================================================================
# 部署 i 的"试飞改动"到 Jetson (2026-10-09): 横向补偿右移 3cm + 停留 5s→2s
# ----------------------------------------------------------------
# 幂等五步 (与 deploy_k_jetson_20261008.sh 同构):
#   ① 板载整包备份 (src/fly + install/fly → ~/fly_backups/)
#   ② scp 本地 fly/i.py → 板载
#   ③ HEADING_DEG 对齐 212.0 (板载 i.py 的现场航向; 仓库 fly 基线是 180.0 —— 不这样会被刷回去!)
#   ④ colcon build --packages-select fly
#   ⑤ 校验: src==install 且非空 / 运行时回读 6 个常量 / ros2 入口存在
# 回退: bash scripts/deploy_i_20261009.sh --rollback   (用最近一次 tar 还原 + 重新编译)
#       或本地回退源码: bash scripts/rollback_i_comp3_dwell2_20261009.sh
# ================================================================
set -e

JETSON="${JETSON:-nvidia@192.168.98.136}"
export SSHPASS="${SSHPASS:?请先 export SSHPASS=<你的 Jetson 密码>}"   # ★ 不再硬编码 ✗
SSOC="StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SSH="sshpass -e ssh -o $SSOC $JETSON"
SCP="sshpass -e scp -o $SSOC"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_I="$HERE/fly/src/fly/fly/i.py"
SRC=/home/nvidia/fly/src/fly/fly/i.py

[ -f "$LOCAL_I" ] || { echo "✗ 本地 i.py 不存在"; exit 1; }

if [ "$1" = "--rollback" ]; then
    echo "=== 回退: 用最近一次 fly_src_install tar 还原 + 重编译 ==="
    $SSH 'T=$(ls -t ~/fly_backups/fly_src_install_*.tar.gz | head -1); echo "  使用 $T"
          tar xzf "$T" -C ~/fly && source /opt/ros/humble/setup.bash && cd ~/fly \
          && colcon build --packages-select fly 2>&1 | tail -2 && echo "  还原完成"'
    exit 0
fi

echo "=== 1. 板载备份 ==="
$SSH 'TS=$(date +%Y%m%d_%H%M%S); mkdir -p ~/fly_backups
      tar czf ~/fly_backups/fly_src_install_${TS}.tar.gz -C ~/fly src/fly install/fly
      echo "  ✓ ~/fly_backups/fly_src_install_${TS}.tar.gz ($(du -h ~/fly_backups/fly_src_install_${TS}.tar.gz | cut -f1))"'

echo "=== 2. 推送 i.py ==="
echo "  本地前: $($SSH "grep -m1 -E '^PAYLOAD_OFFSET_BODY_Y_B1' $SRC | cut -c1-40") | $($SSH "grep -m1 -E '^I_DWELL_S' $SRC | cut -c1-30")"
$SCP "$LOCAL_I" "$JETSON:$SRC"
echo "  推送后: $($SSH "grep -m1 -E '^PAYLOAD_OFFSET_BODY_Y_B1' $SRC | cut -c1-40") | $($SSH "grep -m1 -E '^I_DWELL_S' $SRC | cut -c1-30")"

echo "=== 3. 现场航向对齐 (HEADING_DEG = 212.0) ==="
$SSH "python3 - <<'PY'
import re, shutil, datetime, pathlib
CANON = 'HEADING_DEG = 212.0  # 2026-10-09: 现场航向 (与板载 i.py 一致); 仓库 fly 基线是 180.0'
p = pathlib.Path('$SRC'); t = p.read_text(encoding='utf-8')
lines = [l for l in t.split(chr(10)) if l.startswith('HEADING_DEG')]
assert len(lines) == 1, f'HEADING_DEG 行 {len(lines)} 条'
if lines[0] == CANON:
    print('  · 已是 212.0 规范行, 跳过')
else:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy2(p, f'{p}.bak.{ts}_heading212')
    p.write_text(t.replace(lines[0], CANON, 1), encoding='utf-8')
    print('  ✓ 航向行规范化为 212.0 (原: ' + lines[0][:40] + ')')
PY"

echo "=== 4. 编译 ==="
$SSH 'source /opt/ros/humble/setup.bash && cd ~/fly && colcon build --packages-select fly 2>&1 | tail -3'

echo "=== 5. 校验 ==="
$SSH 'source /opt/ros/humble/setup.bash
INS=/home/nvidia/fly/install/fly/lib/python3.10/site-packages/fly/i.py
A=$(sha256sum /home/nvidia/fly/src/fly/fly/i.py | cut -d" " -f1); B=$(sha256sum $INS | cut -d" " -f1)
SZ=$(stat -c%s $INS)
echo "  src  sha: ${A:0:16}"; echo "  inst sha: ${B:0:16}  (${SZ} 字节)"
[ "$A" = "$B" ] && [ "$SZ" -gt 1000 ] && echo "  ✓ src==install 且非空" || { echo "  ✗ 不一致"; exit 1; }
cd ~/fly && source install/setup.bash && python3 -c "
import fly.i as m
print(f\"  ✓ 运行时回读: Y_B1={m.PAYLOAD_OFFSET_BODY_Y_B1} Y_B2={m.PAYLOAD_OFFSET_BODY_Y_B2} XF={m.PAYLOAD_OFFSET_BODY_XF_B1}/{m.PAYLOAD_OFFSET_BODY_XF_B2}\")
print(f\"    I_DWELL_S={m.I_DWELL_S} I_PRED_M={m.I_PRED_M} I_VMAX={m.I_VMAX} I_RELEASE_TIMEOUT={m.I_RELEASE_TIMEOUT} HEADING_DEG={m.HEADING_DEG}\")
"
echo "  入口: $(ros2 pkg executables fly 2>/dev/null | grep -cx "fly i") 个 fly i"'
echo ""
echo "完成: 现场 cd ~/fly && ./i.sh  (日志 ~/fly_logs/i_mission_*.log)"
echo "回退: bash scripts/deploy_i_20261009.sh --rollback"
