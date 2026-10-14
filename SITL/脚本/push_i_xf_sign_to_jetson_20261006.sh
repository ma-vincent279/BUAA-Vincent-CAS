#!/usr/bin/env bash
# 把 XF 符号修正落到 Jetson 上的 i.py —— **只改远端那两个常量, 绝不整份覆盖**
#
# 为什么不能整份 scp: 本地 fly/src/fly/fly/i.py 的 HEADING_DEG=180.0, 而 Jetson 上
# 10-06 实飞用的是 161deg (航向是现场按场地填的, 日志 `HEADING=161deg` 可查) ⇒
# 整份覆盖会把航向改掉, 飞机朝错方向飞。所以本脚本只在远端原地替换两行常量。
#
# 流程: 远端时间戳备份 → 打印改前常量(含 HEADING) → 只替换 XF_B1/B2 两行(断言唯一命中)
#       → 复核四个常量与 HEADING 未变 → colcon build → 校验 install 副本
# 用法: bash /home/vincent/SITL/scripts/push_i_xf_sign_to_jetson_20261006.sh
set -euo pipefail

JETSON="nvidia@192.168.98.136"
PW="${SSHPASS:?SET_SSHPASS}"
REMOTE_WS="/home/nvidia/fly"
REMOTE_PY="$REMOTE_WS/src/fly/fly/i.py"
INSTALLED="$REMOTE_WS/install/fly/lib/python3.10/site-packages/fly/i.py"
STAMP="$(date +%Y%m%d_%H%M%S)"
SSH=(sshpass -p "$PW" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$JETSON")

echo "== 1) 远端现状 + 时间戳备份 =="
"${SSH[@]}" "cp -v '$REMOTE_PY' '$REMOTE_PY.bak.${STAMP}_xf_sign' && echo '--- 改前:' && \
  grep -n 'PAYLOAD_OFFSET_BODY_Y_B\|PAYLOAD_OFFSET_BODY_XF_B\|^HEADING_DEG' '$REMOTE_PY'"

echo "== 2) 远端原地替换两行常量 (只动 XF_B1/XF_B2) =="
"${SSH[@]}" "python3 -" <<'PY'
import re, sys
p = "/home/nvidia/fly/src/fly/fly/i.py"
s = open(p, encoding="utf-8").read()
NEW = "-0.05"
out = s
for tag in ("PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2"):
    pat = re.compile(r"^(%s\s*=\s*)([+-][\d.]+)(.*)$" % tag, re.M)
    hits = pat.findall(s)
    if len(hits) != 1:
        sys.exit("断言失败: %s 命中 %d 次" % (tag, len(hits)))
    cur = hits[0][1]
    if cur == NEW:
        print("  %s 已是 %s (跳过)" % (tag, NEW))
        continue
    if cur != "+0.12":
        sys.exit("断言失败: %s 现值 %s 不是 +0.12, 拒绝改" % (tag, cur))
    out = pat.sub(lambda m: m.group(1) + NEW + m.group(3), out, count=1)
    print("  %s: %s -> %s" % (tag, cur, NEW))
if out != s:
    open(p, "w", encoding="utf-8").write(out)
    print("  已写回")
else:
    print("  无需改动")
PY

echo "== 3) 复核: 四个常量 + HEADING (HEADING 必须保持现场原值) =="
"${SSH[@]}" "grep -n 'PAYLOAD_OFFSET_BODY_Y_B\|PAYLOAD_OFFSET_BODY_XF_B\|^HEADING_DEG' '$REMOTE_PY'"

echo "== 4) 远端编译 (没有 --symlink-install, 必须 build 才进 install/) =="
"${SSH[@]}" "cd '$REMOTE_WS' && source /opt/ros/humble/setup.bash && colcon build --packages-select fly"

echo "== 5) 校验 install 副本 =="
"${SSH[@]}" "grep -n 'PAYLOAD_OFFSET_BODY_XF_B\|PAYLOAD_OFFSET_BODY_Y_B\|^HEADING_DEG' '$INSTALLED' && \
  sha256sum '$REMOTE_PY' '$INSTALLED'"

echo
echo "✅ 期望结果: XF_B1/XF_B2 = -0.05 ; Y_B1=+0.13 / Y_B2=+0.12 ; HEADING_DEG 仍是现场那个值 (10-06 是 161)"
echo "   节点启动日志里必须看到: F Mission ready ... offset_前_B1=-0.050 B2=-0.050  且 HEADING=<现场值>deg"
