#!/usr/bin/env python3
"""f 为什么振荡 / 为什么有时投不下去 —— 50Hz ULog 对比核查 (2026-10-06 夜)

对每趟航班的每次投放, 在 DROP 段内用 vehicle_local_position(_setpoint) 计算:
  · 设定点(瞄准点)本身动了多少  ← f 的瞄准点每帧重建, i 的瞄准点是冻结常数
  · 误差 err(t) = |飞机(t) − 设定点(t)|   ← 就是 f 的 `等效地面`, 释放门限用的量
  · 速度 vxy(t)
  · 门限可行性: err<5cm 的那些样本, 它们的 vxy 是多少?
                (门限要求 err<5cm 且 vxy<0.12 且持续 0.5s ⇒ 若两者互斥则永不触发)
用法: python3 scripts/compare_f_i_hold_20261006.py
"""

import math
import re
import numpy as np
from pyulog import ULog

FLIGHTS = [
    ("10.6/f_mission_20261006_144934.log", "10.6/log_443_2026-10-6-14-52-36.ulg", "f  10-06 (振荡)"),
    ("10.3/f_mission_20261003_142704.log", "10.3/log_439_2026-10-3-14-40-44.ulg", "f  10-03a"),
    ("10.3/f_mission_20261003_163952.log", "10.3/log_440_2026-10-3-16-43-12.ulg", "f  10-03b"),
    ("9.27/f_mission_20260927_110915.log", "9.27/log_430_2026-9-27-11-12-20.ulg", "f  09-27"),
    ("10.6/i_mission_20261006_170652.log", "10.6/log_444_2026-10-6-17-09-14.ulg", "i  10-06a (冻结)"),
    ("10.6/i_mission_20261006_171748.log", "10.6/log_445_2026-10-6-17-19-44.ulg", "i  10-06b (冻结)"),
]
RE_TS = re.compile(r"\[(\d+\.\d+)\]")


def pulses(u):
    out = []
    for d in u.data_list:
        if d.name != 'actuator_outputs' or getattr(d, 'multi_id', 0) != 1:
            continue
        t = d.data['timestamp'] / 1e6
        o = d.data['output[4]']
        hit = t[(o > 1800) | ((o > 900) & (o < 1100))]
        if len(hit) == 0:
            continue
        s = p = hit[0]
        for x in hit[1:]:
            if x - p > 0.3:
                out.append(s)
                s = x
            p = x
        out.append(s)
    return sorted(out)


def events(path, i_mode):
    """返回 drops(释放墙钟), starts(DROP 段起点墙钟), freezes(i 的冻结时刻)"""
    drops, starts, freezes = [], [], []
    for line in open(path, "r", encoding="utf-8", errors="replace"):
        m = RE_TS.search(line)
        if not m:
            continue
        t = float(m.group(1))
        if '>>> DROP' in line:
            drops.append(t)
        elif 'STEP VISUAL_SERVO->DROP' in line:
            starts.append(t)
        elif i_mode and '一次性修正 delta=' in line:
            freezes.append(t)
    return drops, starts, freezes


def series(u, name):
    for d in u.data_list:
        if d.name == name and getattr(d, 'multi_id', 0) == 0:
            return d.data
    return None


def analyse(u, t0, t1):
    lp, sp = series(u, 'vehicle_local_position'), series(u, 'vehicle_local_position_setpoint')
    if lp is None or sp is None:
        return None
    t = lp['timestamp'] / 1e6
    ts = sp['timestamp'] / 1e6
    m = (ts >= t0) & (ts <= t1)
    if m.sum() < 10:
        return None
    ts, sx, sy = ts[m], sp['x'][m], sp['y'][m]
    x = np.interp(ts, t, lp['x'])
    y = np.interp(ts, t, lp['y'])
    err = np.hypot(sx - x, sy - y)
    vxy = np.hypot(np.interp(ts, t, lp['vx']), np.interp(ts, t, lp['vy']))
    sv = np.hypot(np.diff(sx), np.diff(sy)) / np.maximum(np.diff(ts), 1e-3)
    dur = t1 - t0
    # 每 5s 一段的"误差峰峰", 看是不是在长大 (正反馈)
    seg = []
    for a in np.arange(t0, t1 - 2.0, 5.0):
        mm = (ts >= a) & (ts <= min(a + 5.0, t1))
        if mm.sum() > 5:
            seg.append(100 * (err[mm].max() - err[mm].min()))
    lo5 = err < 0.05
    lo10 = err < 0.10
    return dict(
        dur=dur, seg=seg,
        sp_pp=100 * max(sx.max() - sx.min(), sy.max() - sy.min()),
        sp_v=sv.mean(), sp_vmax=sv.max(),
        err_mean=100 * err.mean(), err_max=100 * err.max(), err_min=100 * err.min(),
        vxy_mean=vxy.mean(),
        f5=100.0 * lo5.mean(), v5=(vxy[lo5].mean() if lo5.any() else float('nan')),
        f10=100.0 * lo10.mean(),
        f_t1=100.0 * (lo5 & (vxy < 0.12)).mean(),
        f_t2=100.0 * (lo10 & (vxy < 0.12)).mean())


def main():
    for node_log, ulog, tag in FLIGHTS:
        i_mode = '/i_' in node_log
        drops, starts, freezes = events(node_log, i_mode)
        if not drops or not starts:
            print(f"{tag}: 日志缺少 DROP 行")
            continue
        u = ULog(ulog)
        pl = pulses(u)
        off = drops[0] - pl[0]
        print(f"\n===== {tag}   (对齐偏移 {off:.2f}s, 脉冲数 {len(pl)})")
        for k, tw in enumerate(drops):
            t1 = tw - off
            if i_mode:
                fr = [f for f in freezes if f < tw]
                t0 = (fr[-1] - off) if fr else starts[0] - off
                kind = "冻结→释放"
            else:
                st = [s for s in starts if s < tw]
                t0 = (st[-1] - off) if st else t1 - 20
                kind = "进 DROP→释放"
            r = analyse(u, t0, t1)
            if r is None:
                continue
            print(f"  --- 第 {k+1} 投 [{kind} {r['dur']:.1f}s]")
            print(f"      瞄准点: 峰峰 {r['sp_pp']:5.1f}cm  移动速度 均 {r['sp_v']:.3f} 最大 {r['sp_vmax']:.3f} m/s")
            print(f"      误差err: 均 {r['err_mean']:5.1f} 最大 {r['err_max']:5.1f} 最小 {r['err_min']:5.1f} cm"
                  f" | 飞机 vxy 均 {r['vxy_mean']:.3f} m/s")
            print(f"      每 5s 误差峰峰: " + " → ".join(f"{v:.0f}" for v in r['seg']) + " cm")
            print(f"      门限可行性: err<5cm 占 {r['f5']:.0f}% (这些样本 vxy 均 {r['v5']:.3f} m/s)"
                  f"; err<5 且 vxy<0.12 占 {r['f_t1']:.0f}%"
                  f"; err<10 且 vxy<0.12 占 {r['f_t2']:.0f}%")


if __name__ == "__main__":
    main()
