#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_fc_hold_20261012.py — 用实飞 .ulg 回答「我们的飞控够不够格」

问题: 他们的释放判据是 `|err| ≤ 0.03 m` **连续保持 2.0 s**（`release_visual_xy_tolerance_m` +
`release_stabilization_s`）。这等价于要求**飞控把飞机停在自己的位置设定点上、误差 ≤3cm 且能撑住 2 秒**。
本脚本直接量这个能力: 从 PX4 ULog 取 `vehicle_local_position`（EKF 位置）与
`vehicle_local_position_setpoint`（飞控正在跟踪的设定点），在 **OFFBOARD(nav_state=14)**
且**低空**（投放段）区间内算水平误差。

输出每人关心的三个数:
  · 中位/p90 误差      —— 稳态精度
  · **≤3cm 时间占比**   —— 门限可达性
  · **≤3cm 最长连续秒数** —— **能不能撑住 2.0s（决定性指标）**

用法: python3 scripts/analyze_fc_hold_20261012.py [ulg ...]
"""
import glob
import os
import sys

import numpy as np
from pyulog import ULog

TOL = 0.03          # 他们的 release_visual_xy_tolerance_m
HOLD = 2.0          # 他们的 release_stabilization_s


def longest_run(mask, dt):
    best = cur = 0.0
    for m in mask:
        cur = cur + dt if m else 0.0
        best = max(best, cur)
    return best


def analyze(path):
    u = ULog(path)
    pos = sp = st = None
    for d in u.data_list:
        if d.name == 'vehicle_local_position':
            pos = d.data
        elif d.name == 'vehicle_local_position_setpoint':
            sp = d.data
        elif d.name == 'vehicle_status':
            st = d.data
    if pos is None or sp is None:
        return None
    t = pos['timestamp'] / 1e6
    x, y, z = pos['x'], pos['y'], pos['z']
    vxy = np.hypot(pos['vx'], pos['vy'])
    # 设定点按时间插值到位置时间轴上
    ts = sp['timestamp'] / 1e6
    order = np.argsort(ts)
    ts = ts[order]
    sx = np.interp(t, ts, sp['x'][order], left=np.nan, right=np.nan)
    sy = np.interp(t, ts, sp['y'][order], left=np.nan, right=np.nan)
    e = np.hypot(x - sx, y - sy)
    # OFFBOARD 且低空 (投放段): z 取最深 35% 区间
    offb = np.zeros_like(t, dtype=bool)
    if st is not None:
        tst = st['timestamp'] / 1e6
        ns = st['nav_state']
        o = np.argsort(tst)
        offb = np.interp(t, tst[o], ns[o], left=0, right=0) == 14
    ok = offb & np.isfinite(e)
    if ok.sum() < 50:
        return None
    z_ok = z[ok]
    thr = np.percentile(z_ok, 65)          # 更深(更负)的一侧 = 低空
    low = ok & (z <= thr)
    dt = float(np.median(np.diff(t)))
    # 只保留「设定点近似静止」的窗口 —— 那才是"停住"的能力 (他们的 3cm 门问的就是这个)
    # 设定点速度 > 2cm/s 视为在机动, 不计入
    svx = np.gradient(sx, t); svy = np.gradient(sy, t)
    static = np.hypot(np.nan_to_num(svx), np.nan_to_num(svy)) < 0.02
    low_static = low & static
    out = {'file': os.path.basename(path), 'n': int(low_static.sum()),
           'dur': float(low_static.sum() * dt)}
    for tag, m in (('全部OFFBOARD', ok), ('低空投放段', low), ('低空且设定点静止', low_static)):
        ee = e[m]
        out[tag] = dict(
            med=float(np.median(ee)) * 100, p90=float(np.percentile(ee, 90)) * 100,
            mx=float(np.max(ee)) * 100, frac3=float((ee <= TOL).mean()) * 100,
            run3=longest_run(ee <= TOL, dt), vxy=float(np.mean(vxy[m])))
    return out


def main():
    files = sys.argv[1:] or sorted(glob.glob('*.ulg') + glob.glob('*/*.ulg'))
    rows = [r for r in (analyze(f) for f in files) if r]
    print(f"{'日志':<34}{'段时长':<9}{'中位':<8}{'p90':<8}{'最大':<9}{'≤3cm占比':<11}"
          f"{'≤3cm最长':<10}{'vxy均值'}")
    for r in rows:
        d = r['低空且设定点静止']
        print(f"{r['file']:<34}{r['dur']:<9.1f}{d['med']:<8.1f}{d['p90']:<8.1f}{d['mx']:<9.1f}"
              f"{d['frac3']:<11.1f}{d['run3']:<10.2f}{d['vxy']:.3f}")
    if rows:
        med = np.median([r['低空且设定点静止']['med'] for r in rows])
        p90 = np.median([r['低空且设定点静止']['p90'] for r in rows])
        f3 = np.median([r['低空且设定点静止']['frac3'] for r in rows])
        r3 = np.median([r['低空且设定点静止']['run3'] for r in rows])
        ok = sum(1 for r in rows if r['低空且设定点静止']['run3'] >= HOLD)
        print(f"\n【低空·设定点静止 · {len(rows)} 份日志汇总】中位误差 {med:.1f}cm / p90 {p90:.1f}cm / "
              f"≤3cm 占比 {f3:.1f}% / ≤3cm 最长连续 {r3:.2f}s")
        print(f"  满足他们的『≤3cm 连续 {HOLD}s』条件: **{ok}/{len(rows)} 份日志**")




# ---------------------------------------------------------------------------
# 追加 (2026-10-12): **最有利口径** —— 每份日志取最长的「设定点静止」窗口,
# 只统计这一段。若连最有利的一段都满足不了 3cm/2.0s, 结论就无法再被"选样本"动摇。
# ---------------------------------------------------------------------------
def best_window(path):
    u = ULog(path)
    pos = sp = None
    for d in u.data_list:
        if d.name == 'vehicle_local_position':
            pos = d.data
        elif d.name == 'vehicle_local_position_setpoint':
            sp = d.data
    if pos is None or sp is None:
        return None
    t = pos['timestamp'] / 1e6
    x, y = pos['x'], pos['y']
    vxy = np.hypot(pos['vx'], pos['vy'])
    ts = sp['timestamp'] / 1e6
    o = np.argsort(ts); ts = ts[o]
    sx = np.interp(t, ts, sp['x'][o]); sy = np.interp(t, ts, sp['y'][o])
    e = np.hypot(x - sx, y - sy)
    sv = np.hypot(np.gradient(sx, t), np.gradient(sy, t))
    dt = float(np.median(np.diff(t)))
    static = (sv < 0.02) & np.isfinite(e)
    # 找最长的连续静态窗口 (允许 0.5s 的空洞: 单帧抖动不算断)
    gap = max(1, int(0.5 / dt))
    best = (0, 0, 0); i = 0; n = len(static)
    while i < n:
        if not static[i]:
            i += 1; continue
        j = i; miss = 0
        while j < n and miss <= gap:
            miss = 0 if static[j] else miss + 1
            j += 1
        seg = (i, j - miss)
        if seg[1] - seg[0] > best[1] - best[0]:
            best = (seg[0], seg[1], 0)
        i = j
    a, b = best[0], best[1]
    if b - a < 10:
        return None
    seg = slice(a, b)
    ee = e[seg]
    return dict(file=os.path.basename(path), dur=float((b - a) * dt),
                med=float(np.median(ee)) * 100, p90=float(np.percentile(ee, 90)) * 100,
                mn=float(np.min(ee)) * 100,
                frac3=float((ee <= TOL).mean()) * 100, run3=longest_run(ee <= TOL, dt),
                vxy=float(np.mean(vxy[seg])))


def main_best():
    files = sorted(glob.glob('*.ulg') + glob.glob('*/*.ulg'))
    rows = [r for r in (best_window(f) for f in files) if r]
    print("\n" + "=" * 104)
    print("【最有利口径】每份日志取**最长的「设定点静止」窗口**（允许 0.5s 空洞）")
    print(f"{'日志':<34}{'窗口时长':<10}{'中位':<8}{'p90':<8}{'最小':<8}{'≤3cm占比':<11}{'≤3cm最长':<10}{'vxy'}")
    for r in rows:
        print(f"{r['file']:<34}{r['dur']:<10.1f}{r['med']:<8.1f}{r['p90']:<8.1f}{r['mn']:<8.1f}"
              f"{r['frac3']:<11.1f}{r['run3']:<10.2f}{r['vxy']:.3f}")
    if rows:
        ok = sum(1 for r in rows if r['run3'] >= HOLD)
        print(f"\n  {len(rows)} 份日志 · 中位误差 {np.median([r['med'] for r in rows]):.1f}cm / "
              f"≤3cm 时间占比中位 {np.median([r['frac3'] for r in rows]):.1f}% / "
              f"≤3cm 最长连续中位 {np.median([r['run3'] for r in rows]):.2f}s / "
              f"最好的一份 {max(r['run3'] for r in rows):.2f}s")
        print(f"  ⇒ 满足他们『≤3cm 连续 {HOLD}s』: **{ok}/{len(rows)}**")


if __name__ == '__main__':
    main()
    main_best()
