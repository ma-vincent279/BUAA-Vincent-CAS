#!/usr/bin/env python3
"""投弹瞬间稳定性核查 (2026-10-06 夜) — 用 50Hz ULog 直接回答"飞机投弹时稳不稳"

对齐方式: 节点日志的 wall clock 时刻 - ULog 里 AUX5 脉冲起点 = 固定偏移 (同一航班两投差 <0.05s ⇒ 对齐可信)
输出: 每投在"末 5s"和"冻结→释放整段"里的
      |vxy| 均值/最大 · 水平位置 x/y 峰峰 · 平均漂移速度 · roll/pitch 峰峰 · 角速率峰峰
      · 与实际位置设定点的跟踪误差(若 ULog 有 vehicle_local_position_setpoint)
用法: python3 scripts/check_release_stability_20261006.py
"""

import math
import re
import numpy as np
from pyulog import ULog

# (节点日志, ULog, 说明) —— 航班对应关系由 AUX5 脉冲间隔匹配确定
FLIGHTS = [
    ("10.6/i_mission_20261006_170652.log", "10.6/log_444_2026-10-6-17-09-14.ulg", "i #1"),
    ("10.6/i_mission_20261006_171748.log", "10.6/log_445_2026-10-6-17-19-44.ulg", "i #2"),
    ("10.6/f_mission_20261006_144934.log", "10.6/log_443_2026-10-6-14-52-36.ulg", "f (振荡参照)"),
]


def aux5_pulses(u):
    """返回 AUX5 (multi_id=1) 上 >1800us 的脉冲起点列表 (ULog 时间, s)"""
    out = []
    for d in u.data_list:
        if d.name != 'actuator_outputs' or getattr(d, 'multi_id', 0) != 1:
            continue
        t = d.data['timestamp'] / 1e6
        o = d.data['output[4]']
        hi = t[o > 1800]
        if len(hi) == 0:
            continue
        s = p = hi[0]
        for x in hi[1:]:
            if x - p > 0.3:
                out.append(s)
                s = x
            p = x
        out.append(s)
    return sorted(out)


RE_TS = re.compile(r"\[(\d+\.\d+)\]")


def ts_of(line):
    m = RE_TS.search(line)
    return float(m.group(1)) if m else None


def node_events(path):
    """返回 (drop_wall_times, freeze_wall_times, home)"""
    drops, freezes, home = [], [], None
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if 'Home locked' in line:
                i = line.index('NED(') + 4
                x, y, z = line[i:line.index(')', i)].split(',')
                home = (float(x), float(y), float(z))
            if '>>> DROP' in line:
                t = ts_of(line)
                if t:
                    drops.append(t)
            if '一次性修正 delta=' in line:
                t = ts_of(line)
                if t:
                    freezes.append(t)
    return drops, freezes, home


def win(t, a, b):
    return (t >= a) & (t <= b)


def stats(u, t0, t1, label):
    def get(name):
        for d in u.data_list:
            if d.name == name and getattr(d, 'multi_id', 0) == 0:
                return d.data
        return None
    lp = get('vehicle_local_position')
    at = get('vehicle_attitude')
    av = get('vehicle_angular_velocity')
    sp = get('vehicle_local_position_setpoint')
    if lp is None:
        print(f"    {label}: 无 vehicle_local_position")
        return
    t = lp['timestamp'] / 1e6
    m = win(t, t0, t1)
    xf_all, yf_all = lp['x'], lp['y']
    x, y = xf_all[m], yf_all[m]
    vx, vy = lp['vx'][m], lp['vy'][m]
    vxy = np.hypot(vx, vy)
    dur = t1 - t0
    # 平均漂移 = 段内首尾位移 / 时长 (只看整体走没走, 不看抖动)
    drift = math.hypot(x[-1] - x[0], y[-1] - y[0]) / dur if dur > 0 else 0.0
    roll = pitch = None
    if at is not None:
        ta = at['timestamp'] / 1e6
        ma = win(ta, t0, t1)
        q = [at['q[%d]' % i][ma] for i in range(4)]      # w,x,y,z
        w, qx, qy, qz = q
        roll = np.degrees(np.arctan2(2 * (w * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (w * qy - qz * qx), -1, 1)))
    gx = gy = None
    if av is not None:
        tv = av['timestamp'] / 1e6
        mv = win(tv, t0, t1)
        gx = np.degrees(av['xyz[0]'][mv])
        gy = np.degrees(av['xyz[1]'][mv])
    trk = None
    if sp is not None:
        tsp = sp['timestamp'] / 1e6
        msp = win(tsp, t0, t1)
        if msp.sum() > 5:
            # 设定点与飞机同一采样时刻插值比较
            xi = np.interp(tsp[msp], t, xf_all)
            yi = np.interp(tsp[msp], t, yf_all)
            trk = np.hypot(sp['x'][msp] - xi, sp['y'][msp] - yi)
    spd = ""
    if sp is not None:
        tsp = sp['timestamp'] / 1e6
        msp = win(tsp, t0, t1)
        if msp.sum() > 5:
            sx, sy = sp['x'][msp], sp['y'][msp]
            dt = np.diff(tsp[msp])
            sv = np.hypot(np.diff(sx), np.diff(sy)) / np.maximum(dt, 1e-3)
            spd = (f" || 设定点(瞄准点) 峰峰 {100 * max(sx.max() - sx.min(), sy.max() - sy.min()):5.1f}cm"
                   f" 移动速度 均 {sv.mean():.3f} 最大 {sv.max():.3f} m/s")
    s = (f"    {label:<26} |vxy| 均 {vxy.mean():.3f} 最大 {vxy.max():.3f} m/s"
         f" | x 峰峰 {100 * (x.max() - x.min()):5.1f} y 峰峰 {100 * (y.max() - y.min()):5.1f} cm"
         f" | 首尾位移 {100 * drift:4.1f} cm/s")
    if roll is not None:
        s += f" | roll 峰峰 {roll.max() - roll.min():4.2f}° pitch 峰峰 {pitch.max() - pitch.min():4.2f}°"
    if gx is not None:
        s += f" | 角速率峰峰 {max(gx.max() - gx.min(), gy.max() - gy.min()):5.1f}°/s"
    if trk is not None:
        s += f" | 跟踪误差均值 {100 * trk.mean():.1f} 最大 {100 * trk.max():.1f} cm"
    print(s + spd)


def main():
    for node_log, ulog, tag in FLIGHTS:
        drops, freezes, home = node_events(node_log)
        if not drops:
            print(f"== {tag}: 没找到 DROP 行")
            continue
        u = ULog(ulog)
        pulses = aux5_pulses(u)
        offset = drops[0] - pulses[0]
        print(f"\n===== {tag}: {node_log.split('/')[-1]}  ↔  {ulog.split('/')[-1]}")
        chk = (f"第2投 预期 {drops[1] - offset:.2f}s vs 实测脉冲 {pulses[1]:.2f}s"
               f" (差 {abs((drops[1] - offset) - pulses[1]):.2f}s)") if len(pulses) > 1 else "只有 1 个脉冲"
        print(f"  对齐偏移 = {offset:.2f}s (用第 1 投标定); 校核: {chk}")
        for k, tw in enumerate(drops):
            t_rel = tw - offset
            fr = [f for f in freezes if f < tw]
            t_frz = (fr[-1] - offset) if fr else None
            print(f"  --- 第 {k + 1} 投 (释放 @ ULog {t_rel:.2f}s)")
            stats(u, t_rel - 3.0, t_rel, "投弹前 3s")
            stats(u, t_rel - 5.0, t_rel, "投弹前 5s")
            if t_frz and t_rel - t_frz > 1.0:
                stats(u, t_frz, t_rel, f"冻结→释放({t_rel - t_frz:.1f}s)")


if __name__ == "__main__":
    main()
