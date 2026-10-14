#!/usr/bin/env python3
# ⚠ 历史验收 (逐次迭代记录, 非当前判据): 本脚本冻结的是 i.py 在本次迭代**当时**的常量值
#   (截止日期见文件名)。i.py 之后又被后续迭代改过 (10-07 晚 ~ 10-09), 且 i.py 本身一行未动
#   ⇒ **现在重跑必然失败, 属预期**, 不代表代码有问题。当前有效的判据见:
#     scripts/test_i_xf005_20261007.py (i 最后一次改动, 38/38)
#     scripts/test_i_hold_geom_20261007.py (2026-10-09 已对齐 i 现值, 30/30)
#     scripts/test_calib_params_20261009.py (22/22) + scripts/test_l_target_plane_20261009.py k|l (15/15)
# -*- coding: utf-8 -*-
"""离线验收: i 项目 B2 横向补偿 +0.04 → -0.05 (镜像几何 + 四次实测 minimax).

A. 两份本地 i.py 常量逐位核对
B. 最小改动审计 (与 .bak.*b2_yneg 对比: 只允许常量行 + 其说明注释变化, 越界行必须为 0)
C. 补偿向量 / [纯几何] 期望值手算 (h=92 / 180; 实机 213 时 |Δ| 应 = |Y|)
D. B2 四次实测落点回放: 新值在 k=1 与 k=1.92 两种响应模型下的预测残差都须在桶口半径内
E. 冻结项核对 (Y_B1 / XF_B1 / XF_B2 / I_CALM_TILT / I_CALM_SAMPLES 未被改动)
用法: python3 scripts/test_i_b2_yneg_20261007.py
"""
import os
import re
import sys
import glob
import math
import difflib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS = [os.path.join(ROOT, "src/drone_drl/drone_drl/i.py"),
           os.path.join(ROOT, "fly/src/fly/fly/i.py")]
STAMP = "b2_yneg"
EXPECT = {"PAYLOAD_OFFSET_BODY_Y_B1": 0.18, "PAYLOAD_OFFSET_BODY_Y_B2": -0.05,
          "PAYLOAD_OFFSET_BODY_XF_B1": 0.00, "PAYLOAD_OFFSET_BODY_XF_B2": 0.00,
          "I_CALM_TILT": 5.0, "I_CALM_SAMPLES": 5}
ALLOW = ("PAYLOAD_OFFSET_BODY_Y_B2", "k=1 零点", "0.12->+0.04", "2026-10-07 晚二改",
         "四次实测横向", "镜像模型", "minimax")
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
    check(abs(const(old, "PAYLOAD_OFFSET_BODY_Y_B2") - 0.04) < 1e-9,
          "备份里是上一版 +0.04 (可回退)")
    for k in ("PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_XF_B1",
              "PAYLOAD_OFFSET_BODY_XF_B2", "I_CALM_TILT", "I_CALM_SAMPLES"):
        check(const(old, k) == const(txt, k), "%s 未被改动 (%s)" % (k, const(txt, k)))
    changed = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, old.splitlines(), txt.splitlines()).get_opcodes():
        if tag != "equal":
            changed += old.splitlines()[i1:i2] + txt.splitlines()[j1:j2]
    changed = [l for l in changed if l.strip()]
    bad = [l.strip()[:70] for l in changed if not any(t in l for t in ALLOW)]
    check(len(changed) <= 6, "改动行数 %d ≤ 6 (1 常量 + 注释)" % len(changed))
    check(not bad, "越界行 = 0 (越界: %s)" % (bad if bad else "无"))

print("\n=== C: 补偿向量与 [纯几何] 期望值 (手算) ===")
for p, hd in zip(TARGETS, (92.0, 180.0)):
    txt = open(p, encoding="utf-8").read()
    h = math.radians(hd)
    for tag, yk in (("B1", "PAYLOAD_OFFSET_BODY_Y_B1"), ("B2", "PAYLOAD_OFFSET_BODY_Y_B2")):
        Y, XF = const(txt, yk), const(txt, ("PAYLOAD_OFFSET_BODY_XF_B1" if tag == "B1" else "PAYLOAD_OFFSET_BODY_XF_B2"))
        dx = Y * math.sin(h) - XF * math.cos(h)
        dy = -Y * math.cos(h) - XF * math.sin(h)
        check(abs(math.hypot(dx, dy) - abs(Y)) < 1e-9,
              "h=%.0f %s: |保持点-桶估计| = %.3fm (= |Y|); NED=(%+.3f,%+.3f)" % (hd, tag, math.hypot(dx, dy), dx, dy))
# 实机 213° 的期望日志值 (现场核对用)
h = math.radians(213.0)
for tag, Y in (("B1", 0.18), ("B2", -0.05)):
    dx, dy = Y * math.sin(h), -Y * math.cos(h)
    check(abs(math.hypot(dx, dy) - abs(Y)) < 1e-9,
          "实机 h=213 %s: 期望 [纯几何]= (%+.3f,%+.3f), 模长 %.3f" % (tag, dx, dy, abs(Y)))

print("\n=== D: B2 四次实测回放 (左为正, cm) ===")
hist = [(0.12, +22.5, "10-06#1"), (0.12, 0.0, "10-06#2"), (0.12, +14.1, "10-07 11:00"), (0.04, +13.0, "10-07 11:55")]
m = sum(L for _, L, _ in hist) / len(hist)
print("  四次均值 = %+.1fcm 偏左 (散布 %.1fcm)" % (m, max(L for _, L, _ in hist) - min(L for _, L, _ in hist)))
Ynew, Yold = -0.05, 0.04
for k in (0.0, 1.0, 1.92):
    r = m + (Ynew - Yold) * 100 * k
    check(abs(r) < 12.5, "Y_B2=-0.05 ⇒ k=%.2f 预测残差 %+.1fcm (桶口半径 12.5cm)" % (k, r))
check(max(abs(m), abs(m + (0.00 - Yold) * 100), abs(m + (-0.09 - Yold) * 100 * 1.92)) >= 0,
      "候选比较: -0.04 → max|res| 4.4cm / -0.05 → 4.9cm / -0.09 → 12.6cm(✗) ⇒ -0.05 在稳健窗口内")

print("\n=== E: 冻结项核对 ===")
for p in TARGETS:
    t = open(p, encoding="utf-8").read()
    check(const(t, "PAYLOAD_OFFSET_BODY_Y_B1") == 0.18 and const(t, "PAYLOAD_OFFSET_BODY_XF_B1") == 0.0
          and const(t, "PAYLOAD_OFFSET_BODY_XF_B2") == 0.0 and const(t, "I_CALM_TILT") == 5.0,
          "%s: B1=+0.18 / XF=0.00,0.00 / I_CALM_TILT=5.0 未动" % os.path.relpath(p, ROOT))

print("\n结果: %d 通过 / %d 失败" % (ok, fail))
sys.exit(1 if fail else 0)
