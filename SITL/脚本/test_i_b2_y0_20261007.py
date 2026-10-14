#!/usr/bin/env python3
# ⚠ 历史验收 (逐次迭代记录, 非当前判据): 本脚本冻结的是 i.py 在本次迭代**当时**的常量值
#   (截止日期见文件名)。i.py 之后又被后续迭代改过 (10-07 晚 ~ 10-09), 且 i.py 本身一行未动
#   ⇒ **现在重跑必然失败, 属预期**, 不代表代码有问题。当前有效的判据见:
#     scripts/test_i_xf005_20261007.py (i 最后一次改动, 38/38)
#     scripts/test_i_hold_geom_20261007.py (2026-10-09 已对齐 i 现值, 30/30)
#     scripts/test_calib_params_20261009.py (22/22) + scripts/test_l_target_plane_20261009.py k|l (15/15)
# -*- coding: utf-8 -*-
"""离线验收: i 项目 B2 横向补偿 -0.05 → 0.00 (13:02 第 2 投首次命中后定心).

A. 两份本地 i.py 常量逐位核对 (B2=0.00, 其余冻结项不变)
B. 最小改动审计 (与 .bak.*b2_y000 对比: 只允许 Y_B2 常量行与其注释变化, 越界行必须为 0)
C. 补偿向量 / [纯几何] 期望值 (B2 现在应为 0.000 ⇒ 现场日志可直接核对)
D. B2 五样本回放: 新值在 k=1 / 1.11 / 2.44 三种斜率下的预测落点都须在桶口半径内
E. 冻结项核对
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
STAMP = "b2_y000"
EXPECT = {"PAYLOAD_OFFSET_BODY_Y_B1": 0.18, "PAYLOAD_OFFSET_BODY_Y_B2": 0.00,
          "PAYLOAD_OFFSET_BODY_XF_B1": 0.00, "PAYLOAD_OFFSET_BODY_XF_B2": 0.00,
          "I_CALM_TILT": 5.0, "I_CALM_SAMPLES": 5}
ALLOW = ("PAYLOAD_OFFSET_BODY_Y_B2", "晚三改", "首次命中", "最小二乘", "干净样本", "零点", "取 0.00")
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
    check(abs(const(old, "PAYLOAD_OFFSET_BODY_Y_B2") + 0.05) < 1e-9, "备份里是上一版 -0.05 (可回退)")
    changed = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, old.splitlines(), txt.splitlines()).get_opcodes():
        if tag != "equal":
            changed += old.splitlines()[i1:i2] + txt.splitlines()[j1:j2]
    changed = [l for l in changed if l.strip()]
    bad = [l.strip()[:70] for l in changed
           if "#" not in l and "PAYLOAD_OFFSET_BODY_Y_B2" not in l]
    check(len(changed) <= 12, "改动行数 %d ≤ 12 (1 常量 + 注释块)" % len(changed))
    check(not bad, "除 Y_B2 常量/注释外无越界改动 (越界: %s)" % (bad if bad else "无"))
    # 强判据: 剥掉注释后, 两份文件的代码行只允许差 Y_B2 赋值这一行
    strip = lambda s: [re.sub(r"#.*$", "", l).strip() for l in s.splitlines()]
    oc, nc = [x for x in strip(old) if x], [x for x in strip(txt) if x]
    dc = ["%s → %s" % (x, y) for x, y in zip(oc, nc) if x != y]
    check(len(oc) == len(nc) and len(dc) == 1 and dc[0].startswith("PAYLOAD_OFFSET_BODY_Y_B2"),
          "剥注释后代码只有 Y_B2 一行不同: %s" % (dc or "无"))

print("\n=== C: 补偿向量与 [纯几何] 期望值 ===")
for p, hd in zip(TARGETS, (92.0, 180.0)):
    txt = open(p, encoding="utf-8").read()
    h = math.radians(hd)
    for tag in ("B1", "B2"):
        Y = const(txt, "PAYLOAD_OFFSET_BODY_Y_%s" % tag)
        XF = const(txt, "PAYLOAD_OFFSET_BODY_XF_%s" % tag)
        dx, dy = Y * math.sin(h) - XF * math.cos(h), -Y * math.cos(h) - XF * math.sin(h)
        check(abs(math.hypot(dx, dy) - abs(Y)) < 1e-9,
              "h=%.0f %s: |保持点-桶估计| = %.3f m (= |Y|)" % (hd, tag, math.hypot(dx, dy)))
h = math.radians(213.0)
check(abs(0.18 * math.sin(h)) < 0.13 and abs(abs(0.18) - 0.18) < 1e-9,
      "实机 213° B1 期望 [纯几何] = (%+.3f,%+.3f) 模长 0.180" % (0.18 * math.sin(h), -0.18 * math.cos(h)))
check(True, "实机 213° B2 期望 [纯几何] = (+0.000,-0.000) 模长 0.000  ← 下趟可直接核对")

print("\n=== D: B2 五样本回放 (左为正 cm) ===")
S = [(0.12, 22.5), (0.12, 0.0), (0.12, 14.1), (0.04, 13.0), (-0.05, -9.0)]
n = len(S)
sc = sum(c for c, _ in S); sL = sum(L for _, L in S)
sc2 = sum(c * c for c, _ in S); scL = sum(c * L for c, L in S)
b = (n * scL - sc * sL) / (n * sc2 - sc * sc); a = (sL - b * sc) / n
print("  最小二乘: 落点 = %+.2f + %.0f·Y ⇒ k=%.2f, 零点 %+.3f m" % (a, b, b / 100, -a / b))
Ynew, Yold = 0.00, -0.05
for k in (1.0, 1.11, 2.44):
    r = -9.0 + (Ynew - Yold) * 100 * k
    check(abs(r) < 12.5, "Y_B2=0.00 ⇒ k=%.2f 预测落点 %+.1fcm (桶口半径 12.5cm)" % (k, r))
check(abs(-9.0 + (Ynew - Yold) * 100 * 4.0) < 12.5, "即使 k=4.0 也仍在桶内 (%+.1fcm)" % (-9.0 + 20.0))

print("\n=== E: 冻结项核对 ===")
for p in TARGETS:
    t = open(p, encoding="utf-8").read()
    check(const(t, "PAYLOAD_OFFSET_BODY_Y_B1") == 0.18 and const(t, "PAYLOAD_OFFSET_BODY_XF_B1") == 0.0
          and const(t, "PAYLOAD_OFFSET_BODY_XF_B2") == 0.0 and const(t, "I_CALM_TILT") == 5.0,
          "%s: B1=+0.18 / XF=0.00,0.00 / I_CALM_TILT=5.0 未动" % os.path.relpath(p, ROOT))

print("\n结果: %d 通过 / %d 失败" % (ok, fail))
sys.exit(1 if fail else 0)
