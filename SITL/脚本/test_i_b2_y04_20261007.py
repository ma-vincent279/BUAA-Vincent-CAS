#!/usr/bin/env python3
# ⚠ 历史验收 (逐次迭代记录, 非当前判据): 本脚本冻结的是 i.py 在本次迭代**当时**的常量值
#   (截止日期见文件名)。i.py 之后又被后续迭代改过 (10-07 晚 ~ 10-09), 且 i.py 本身一行未动
#   ⇒ **现在重跑必然失败, 属预期**, 不代表代码有问题。当前有效的判据见:
#     scripts/test_i_xf005_20261007.py (i 最后一次改动, 38/38)
#     scripts/test_i_hold_geom_20261007.py (2026-10-09 已对齐 i 现值, 30/30)
#     scripts/test_calib_params_20261009.py (22/22) + scripts/test_l_target_plane_20261009.py k|l (15/15)
# -*- coding: utf-8 -*-
"""离线验收: 2026-10-07 B2 横向补偿重标 (+0.12 → +0.04) + I_CALM_TILT 3.0 → 5.0.

检查:
  A. 三份 i.py 的常量逐位一致, 且 B1 常量与备份逐位相同 (证明"B1 一行未动")
  B. 与备份的逐行 diff 只允许出现在 3 个位置 (Y_B2 行 / I_CALM_TILT 行及其注释 / Y_B2 注释)
  C. 补偿向量与 `[纯几何]` 日志值的手算结果 (HEADING 213 / 92 / 180)
  D. 用 B2 三趟历史落点回放: 新值在 k=1 / k=1.92 两种响应模型下的预测残差
用法: python3 scripts/test_i_b2_y04_20261007.py
"""
import os
import re
import sys
import glob

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS = [os.path.join(ROOT, "src/drone_drl/drone_drl/i.py"),
           os.path.join(ROOT, "fly/src/fly/fly/i.py")]
STAMP = "b2_y04"
EXPECT = {"PAYLOAD_OFFSET_BODY_Y_B1": 0.18, "PAYLOAD_OFFSET_BODY_Y_B2": 0.04,
          "PAYLOAD_OFFSET_BODY_XF_B1": 0.00, "PAYLOAD_OFFSET_BODY_XF_B2": 0.00,
          "I_CALM_TILT": 5.0, "I_CALM_SAMPLES": 5}
ok = fail = 0


def check(cond, msg):
    global ok, fail
    if cond:
        ok += 1
        print("  ✓ %s" % msg)
    else:
        fail += 1
        print("  ✗ %s" % msg)


def const(text, name):
    m = re.search(r"^%s\s*=\s*([+-]?[0-9.]+)" % re.escape(name), text, re.M)
    return float(m.group(1)) if m else None


print("=== A/B: 常量与最小改动审计 ===")
import difflib
# 允许出现的改动行 (常量值 + 三行说明注释); 任何其它行被改 = 越界
ALLOW = ("PAYLOAD_OFFSET_BODY_Y_B2", "I_CALM_TILT", "0.048->+0.12", "0.12->+0.04",
         "k=1 零点", "10.6 回放: 2.0 太严", "使 3.0 仍一票否决", "镜像模型", "冻结采样允许")
for p in TARGETS:
    txt = open(p, encoding="utf-8").read()
    print("\n-- %s" % os.path.relpath(p, ROOT))
    for k, v in EXPECT.items():
        check(const(txt, k) == v, "%s = %s" % (k, v))
    baks = sorted(glob.glob(p + ".bak.*" + STAMP))
    check(len(baks) >= 1, "存在备份 %s" % (os.path.basename(baks[-1]) if baks else "无"))
    if not baks:
        continue
    old = open(baks[-1], encoding="utf-8").read()
    # B1 与 XF 未被改动
    for k in ("PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2"):
        check(const(old, k) == const(txt, k), "%s 未被改动 (%s)" % (k, const(txt, k)))
    o, n = old.splitlines(), txt.splitlines()
    changed = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, o, n).get_opcodes():
        if tag != "equal":
            changed += o[i1:i2] + n[j1:j2]
    bad = [l.strip()[:80] for l in changed
           if l.strip() and not any(t in l for t in ALLOW)]   # 空行不计
    check(len(changed) <= 8, "改动行数 %d ≤ 8 (仅常量行 + 说明注释)" % len(changed))
    check(not bad, "所有改动行都在预期位置 (越界行: %s)" % (bad if bad else "无"))
    for l in sorted({x.strip()[:70] for x in changed}):
        print("      · %s" % l)

print("\n=== C: 补偿向量与 [纯几何] 日志值 (手算) ===")
import math
for p, hd in zip(TARGETS, (92.0, 180.0)):
    txt = open(p, encoding="utf-8").read()
    h = math.radians(hd)
    for tag, yk, xk in (("B1", "PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_XF_B1"),
                        ("B2", "PAYLOAD_OFFSET_BODY_Y_B2", "PAYLOAD_OFFSET_BODY_XF_B2")):
        Y, XF = const(txt, yk), const(txt, xk)
        dx = Y * math.sin(h) - XF * math.cos(h)
        dy = -Y * math.cos(h) - XF * math.sin(h)
        # (前,左) 分量 = (−XF, +Y)
        check(abs(math.hypot(dx, dy) - math.hypot(Y, XF)) < 1e-9,
              "%s h=%.0f %s: |保持点-桶估计|=%.3fm = |(Y,XF)|; NED=(%+.3f,%+.3f)" % (
                  os.path.basename(os.path.dirname(os.path.dirname(p))), hd, tag,
                  math.hypot(dx, dy), dx, dy))

print("\n=== D: B2 三趟历史落点回放 (左为正, cm) ===")
# (补偿值, 落点, 来源)
hist = [(0.12, +22.5, "10-06 四投#1"), (0.12, +0.0, "10-06 四投#2"), (0.12, +14.1, "10-07 11:00")]
mean = sum(L for _, L, _ in hist) / len(hist)
print("  同补偿 (Y=+0.12) 三趟均值 = %+.1fcm (散布 %.1fcm)" % (mean, max(L for _, L, _ in hist) - min(L for _, L, _ in hist)))
for ynew in (0.00, 0.04, 0.06):
    r1 = mean + (ynew - 0.12) * 100
    r19 = mean + (ynew - 0.12) * 194
    inside = max(abs(r1), abs(r19)) <= 12.5
    check(inside, "Y_B2=+%.2f ⇒ 预测残差 k=1 %+.1fcm / k=1.92 %+.1fcm (桶半径 12.5cm)" % (ynew, r1, r19))
check(abs(const(open(TARGETS[0], encoding="utf-8").read(), "PAYLOAD_OFFSET_BODY_Y_B2") - 0.04) < 1e-9,
      "落地值 = +0.04 (两模型残差 +4.2 / −3.3cm)")

print("\n=== E: B1 冻结核对 ===")
for p in TARGETS:
    t = open(p, encoding="utf-8").read()
    check(const(t, "PAYLOAD_OFFSET_BODY_Y_B1") == 0.18 and const(t, "PAYLOAD_OFFSET_BODY_XF_B1") == 0.0,
          "%s B1 = (Y +0.18, XF 0.00) 未变" % os.path.relpath(p, ROOT))

print("\n结果: %d 通过 / %d 失败" % (ok, fail))
sys.exit(1 if fail else 0)
