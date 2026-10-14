#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""比对 6 趟实飞: 飞机是否准确停在"设定点保持不动"的位置 (飞控日志 setpoint vs 实际位置)
做法: 找出所有"设定点恒定(±1cm)持续>=3s"的窗口(即所有预定悬停/停留点: 巡航点/投放悬停/侦察区停留/返航悬停),
      逐窗算 |实际-设定| 的均值与峰峰。
"""
import os, numpy as np
from pyulog import ULog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
F = [('9-19 d', 'log_422_2026-9-19-18-04-34.ulg'), ('9-23 e', 'log_424_2026-9-23-18-00-28.ulg'),
     ('9-24 e', '9.24/log_425_2026-9-24-17-39-10.ulg'), ('9-27 e', '9.27/log_427_2026-9-27-10-57-56.ulg'),
     ('9-27 f', '9.27/log_430_2026-9-27-11-12-20.ulg'), ('9-30 g', 'log_431_2026-9-30-10-22-22.ulg')]


def get(ul, n, mid=0):
    for d in ul.data_list:
        if d.name == n and d.multi_id == mid:
            return d
    return None


print('%-8s %8s %10s %10s %10s %10s' % ('航班', '窗口数', '停偏均值', '停偏最坏', '窗内漂移', '最大漂移'))
for lab, rel in F:
    p = os.path.join(ROOT, rel)
    if not os.path.exists(p):
        continue
    ul = ULog(p)
    t0 = None
    for m in ul.logged_messages:
        if 'Armed' in m.message:
            t0 = m.timestamp / 1e6
            break
    if t0 is None:
        t0 = ul.start_timestamp / 1e6
    lp, sp = get(ul, 'vehicle_local_position'), get(ul, 'vehicle_local_position_setpoint')
    T = lp.data['timestamp'] / 1e6 - t0
    X, Y = np.asarray(lp.data['x']), np.asarray(lp.data['y'])
    Ts = sp.data['timestamp'] / 1e6 - t0
    SX, SY = np.asarray(sp.data['x']), np.asarray(sp.data['y'])
    # 统一到 10Hz 网格
    g = np.arange(0, min(T[-1], Ts[-1]), 0.1)
    x, y = np.interp(g, T, X), np.interp(g, T, Y)
    sx, sy = np.interp(g, Ts, SX), np.interp(g, Ts, SY)
    good = np.isfinite(sx) & np.isfinite(sy)
    errs, pps = [], []
    i, N = 0, len(g)
    while i < N - 30:
        if not good[i]:
            i += 1
            continue
        w = slice(i, i + 30)
        if good[w].all() and np.ptp(sx[w]) < 0.01 and np.ptp(sy[w]) < 0.01:
            d = np.hypot(x[w] - sx[w], y[w] - sy[w])
            if d[0] > 0.5:      # 只统计"飞机已到位后保持不动"的窗口 (排除返航/前飞途中设定点已到位的情况)
                i += 5
                continue
            errs.append(d.mean())
            pps.append(np.ptp(x[w] - x[w].mean()) + np.ptp(y[w] - y[w].mean()))
            i += 30
        else:
            i += 5
    if errs:
        print('%-8s %8d %9.1fcm %9.1fcm %9.1fcm %9.1fcm' % (
            lab, len(errs), np.mean(errs) * 100, np.max(errs) * 100,
            np.mean(pps) * 100, np.max(pps) * 100))
    else:
        print('%-8s  无恒定设定点窗口' % lab)
