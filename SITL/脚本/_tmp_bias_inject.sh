#!/bin/bash
# ================================================================
# 2026-09-13 临时验证脚本: 给 SITL 的 e.py 人为注入"反投影系统偏置"
# ----------------------------------------------------------------
# 目的: 在仿真里复现实机 9-13 的失效条件 (反投影估计系统性偏离真值),
#       验证新增的"像素残差积分闭环"能否把它抵消掉 (旧逻辑会原样变成投弹偏差).
# 只改 SITL e.py, 不动 d/ 不动 fly/ 不动别的节点.
# 用法:
#   bash scripts/_tmp_bias_inject.sh apply    # 注入 0.30m 偏置 + 重新编译
#   bash scripts/_tmp_bias_inject.sh revert    # 还原 (md5 校验) + 重新编译
#   bash scripts/_tmp_bias_inject.sh status
# ================================================================
set -e
SRC=/home/vincent/SITL/src/drone_drl/drone_drl/e.py
BAK=${SRC}.bak.20260913_pre_biastest
BIAS="${BIAS_M:-0.30}"
cd /home/vincent/SITL

case "$1" in
apply)
    [ -f "$BAK" ] || cp -p "$SRC" "$BAK"
    python3 - "$SRC" "$BIAS" <<'PY'
import sys
p, bias = sys.argv[1], sys.argv[2]
s = open(p, encoding='utf-8').read()
assert 'SERVO_TEST_BIAS_M' not in s, '已经注入过了'
a = 'SERVO_BIAS_ERR_MAX_M   = 1.0'
s = s.replace(a, a + f'\nSERVO_TEST_BIAS_M      = {bias}   # TEMP 验证注入的反投影偏置 (m), 验证完必须还原', 1)
old = """                    est = self._roi_to_ned(roi)
                    anchor_x, anchor_y = self._current_target_ned"""
new = """                    est = self._roi_to_ned(roi)
                    if SERVO_TEST_BIAS_M and est is not None:   # TEMP 2026-09-13: 人为反投影偏置
                        est = (est[0] + SERVO_TEST_BIAS_M, est[1])
                    anchor_x, anchor_y = self._current_target_ned"""
assert s.count(old) == 1, 'DROP 段锚点定位失败'
s = s.replace(old, new, 1)
open(p, 'w', encoding='utf-8').write(s)
print('  注入完成: SERVO_TEST_BIAS_M =', bias)
PY
    colcon build --packages-select drone_drl 2>&1 | tail -2
    ;;
revert)
    [ -f "$BAK" ] || { echo "没有备份 $BAK"; exit 1; }
    cp -p "$BAK" "$SRC"
    # 关键: cp -p 会保留旧 mtime, colcon 会以为源文件没变而跳过安装 → 安装树里留着注入版!
    touch "$SRC"
    if diff -q "$BAK" "$SRC" >/dev/null; then echo "  还原完成 (与备份逐字节一致, 已 touch 保证 colcon 重新安装)"; fi
    grep -c 'SERVO_TEST_BIAS_M' "$SRC" || true
    colcon build --packages-select drone_drl 2>&1 | tail -2
    ;;
status)
    grep -n 'SERVO_TEST_BIAS_M' "$SRC" || echo "  未注入"
    ;;
*)
    echo "用法: $0 apply|revert|status"; exit 1;;
esac
