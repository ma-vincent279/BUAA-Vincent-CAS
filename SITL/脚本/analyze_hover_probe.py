#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_hover_probe.py — 定点悬停探针结果分析 (2026-10-12, v04 多点版)

把 `hover_probe.py` 写出的 CSV 归纳成"这个 SITL 的位置保持地板":
  · 每个定点阶段 (HOLD0 / HOLD1 / ...) 的 设定点 vs 实际位置
  · 稳态偏移 (均值) / 抖动 (标准差) / 峰峰值 / 均方根 / 速度

多点序列的价值: 把"地板差"的三种可能原因分开 ——
  时间/EKF 漂移 (末尾回起飞点 vs 开头) · 离原点距离 · 桶本身

用法: python3 scripts/analyze_hover_probe.py <csv> [<csv> ...]
"""
import csv
import math
import pathlib
import statistics as st
import sys


def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: (v if k == 'step' else float(v)) for k, v in r.items()})
            except (TypeError, ValueError):
                continue
    return rows


def stats(sel):
    if len(sel) < 40:
        return None
    sp = (st.median(r['sp_x'] for r in sel),
          st.median(r['sp_y'] for r in sel),
          st.median(r['sp_z'] for r in sel))
    dx = [r['x'] - sp[0] for r in sel]
    dy = [r['y'] - sp[1] for r in sel]
    dz = [r['z'] - sp[2] for r in sel]
    d2 = [math.hypot(a, b) for a, b in zip(dx, dy)]
    v2 = [math.hypot(r['vx'], r['vy']) for r in sel]
    return dict(n=len(sel), dur=sel[-1]['t'] - sel[0]['t'], sp=sp,
                mx=st.mean(dx), my=st.mean(dy), mz=st.mean(dz),
                sx=st.pstdev(dx), sy=st.pstdev(dy), sz=st.pstdev(dz),
                pp2=max(d2), rms2=math.sqrt(sum(d * d for d in d2) / len(d2)),
                med2=st.median(d2), meanv=st.mean(v2), maxv=max(v2),
                min2=min(d2))


def main():
    for p in (sys.argv[1:] or ['/tmp/hover_probe.csv']):
        p = pathlib.Path(p)
        if not p.exists():
            print(f"--- {p} 不存在"); continue
        rows = load(p)
        holds = sorted({r['step'] for r in rows if r['step'].startswith('HOLD')},
                       key=lambda s: int(s[4:]))
        print("=" * 92)
        print(f"{p.name}   样本 {len(rows)}   定点阶段 {holds}")
        print(f"\n{'阶段':<8}{'时长':<7}{'设定点 NED':<26}{'稳态偏移(cm)':<16}{'抖动σ(cm)':<14}"
              f"{'距离 中位/RMS/峰峰(cm)':<26}{'vxy 均值'}")
        summary = []
        for h in holds:
            s = stats([r for r in rows if r['step'] == h])
            if not s:
                continue
            summary.append((h, s))
            off = f"({s['mx'] * 100:+.1f},{s['my'] * 100:+.1f})"
            sig = f"({s['sx'] * 100:.1f},{s['sy'] * 100:.1f})"
            print(f"{h:<8}{s['dur']:<7.1f}({s['sp'][0]:7.2f},{s['sp'][1]:7.2f},"
                  f"{s['sp'][2]:6.2f})  {off:<16}{sig:<14}"
                  f"{s['med2'] * 100:5.1f} /{s['rms2'] * 100:6.1f} /{s['pp2'] * 100:6.1f}   "
                  f"{s['meanv']:.3f}  (峰 {s['maxv']:.3f})")
        if summary:
            worst = max(summary, key=lambda x: x[1]['rms2'])
            best = min(summary, key=lambda x: x[1]['rms2'])
            print(f"\n  最好 {best[0]}: RMS {best[1]['rms2'] * 100:.1f}cm   "
                  f"最差 {worst[0]}: RMS {worst[1]['rms2'] * 100:.1f}cm   "
                  f"比值 {worst[1]['rms2'] / max(best[1]['rms2'], 1e-6):.1f}×")
            print(f"  小桶半径 7.5cm / 中桶 10.0cm / 大桶 12.5cm  ← 与上表 RMS 对比即可判断"
                  f"'能不能进 A 区'")
        print()


if __name__ == '__main__':
    main()
