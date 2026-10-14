#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_probe_gate_20261012.py — 探针 CSV 上直接算他们那条判据

他们的门: `|err| ≤ 0.03 m` **连续保持 2.0 s**。探针 CSV 是 20Hz (比 ulg 的 10Hz 细一倍),
所以这里算出的"≤3cm 最长连续秒数"分辨率更高、更有说服力。

用法: python3 scripts/analyze_probe_gate_20261012.py <csv> [<csv> ...]
"""
import csv, sys, numpy as np, math, os

TOL, HOLD = 0.03, 2.0

def run(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    by = {}
    for r in rows:
        by.setdefault(r['step'], []).append(r)
    print("=" * 92)
    print(f"{os.path.basename(os.path.dirname(csv_path))}/{os.path.basename(csv_path)}")
    print(f"{'阶段':<9}{'时长':<8}{'中位(cm)':<10}{'p90(cm)':<10}{'≤3cm占比':<11}{'≤3cm最长(s)':<13}{'vxy均值'}")
    allrun = []
    for st in sorted(k for k in by if k.startswith('HOLD')):
        sel = by[st]
        if len(sel) < 40: continue
        f = lambda k: np.array([float(x[k]) for x in sel])
        t = f('t'); dt = float(np.median(np.diff(t)))
        e = np.hypot(f('x') - np.median(f('sp_x')), f('y') - np.median(f('sp_y')))
        v = np.hypot(f('vx'), f('vy'))
        best = cur = 0.0
        for m in (e <= TOL):
            cur = cur + dt if m else 0.0
            best = max(best, cur)
        allrun.append(best)
        print(f"{st:<9}{len(sel)*dt:<8.1f}{np.median(e)*100:<10.1f}{np.percentile(e,90)*100:<10.1f}"
              f"{100*(e<=TOL).mean():<11.1f}{best:<13.2f}{v.mean():.3f}")
    if allrun:
        print(f"  ⇒ 满足『≤3cm 连续 {HOLD}s』的阶段: **{sum(1 for x in allrun if x>=HOLD)}/{len(allrun)}**"
              f"   最长 {max(allrun):.2f}s")

for p in sys.argv[1:]:
    run(p)
