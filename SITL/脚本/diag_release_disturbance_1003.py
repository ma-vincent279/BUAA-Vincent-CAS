#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第一步诊断 (10.3 两趟 4 投): 脱钩前后 2s 的扰动力矩/角速率对比
右瓶=投1/3, 左瓶=投2/4。姿态被飞控稳住 => 扰动力矩(torque_setpoint)是外力的最干净代理。
另按 9-27 方法用滚转力矩反推"载荷摆角"量级。
"""
import numpy as np, math, os
from pyulog import ULog

ROOT = '/home/vincent/SITL/10.3'
F = [('10-3#1 老(右瓶)', 'log_439_2026-10-3-14-40-44.ulg'), ('10-3#1 老(左瓶)', 'log_439_2026-10-3-14-40-44.ulg'),
     ('10-3#2 新(右瓶)', 'log_440_2026-10-3-16-43-12.ulg'), ('10-3#2 新(左瓶)', 'log_440_2026-10-3-16-43-12.ulg')]


def get(ul, n, mid=0):
    for d in ul.data_list:
        if d.name == n and d.multi_id == mid:
            return d
    return None


def pulses(ul, t0):
    ao = get(ul, 'actuator_outputs', 1)
    Ta = ao.data['timestamp'] / 1e6 - t0
    aux = np.asarray(ao.data['output[4]'])
    out = []
    for mask in (aux > 1600, aux < 1400):
        idx = np.where(mask)[0]
        if len(idx):
            s = idx[0]
            for k in range(1, len(idx)):
                if Ta[idx[k]] - Ta[idx[k - 1]] > 0.5:
                    out.append(Ta[s]); s = idx[k]
            out.append(Ta[s])
    return sorted(set(round(p, 2) for p in out if p > 20))[:2]


# 各航班只解析一次
cache = {}
for lab, rel in F:
    if rel not in cache:
        ul = ULog(os.path.join(ROOT, rel))
        t0 = None
        for m in ul.logged_messages:
            if 'Armed' in m.message:
                t0 = m.timestamp / 1e6
                break
        t0 = t0 or ul.start_timestamp / 1e6
        ts = get(ul, 'vehicle_torque_setpoint')
        T = ts.data['timestamp'] / 1e6 - t0
        tx = np.asarray(ts.data['xyz[0]']); ty = np.asarray(ts.data['xyz[1]'])
        va = get(ul, 'vehicle_angular_velocity')
        Tv = va.data['timestamp'] / 1e6 - t0
        gx = np.degrees(np.asarray(va.data['xyz[0]'])); gy = np.degrees(np.asarray(va.data['xyz[1]']))
        cache[rel] = dict(T=T, tx=tx, ty=ty, Tv=Tv, gx=gx, gy=gy, tp=pulses(ul, t0))
    c = cache[rel]
    idx = [i for i, (l, r) in enumerate(F) if r == rel and l == lab]
    which = 0 if ('右瓶' in lab and '老' in lab) or ('右瓶' in lab and '新' in lab and '2' in lab) else 0
    # 按标签选出该航班第几次脉冲: 老航班右瓶=第1次, 左瓶=第2次; 新航班同
    k = 0 if '右瓶' in lab else 1
    tp = c['tp'][k] if len(c['tp']) > k else None
    if tp is None:
        continue
    T = c['T']
    pre = (T >= tp - 1.2) & (T <= tp - 0.2)
    post = (T >= tp + 0.3) & (T <= tp + 1.8)
    vpre = (c['Tv'] >= tp - 1.2) & (c['Tv'] <= tp - 0.2)
    vpost = (c['Tv'] >= tp + 0.3) & (c['Tv'] <= tp + 1.8)
    dtx = c['tx'][post].mean() - c['tx'][pre].mean()
    dty = c['ty'][post].mean() - c['ty'][pre].mean()
    pp_tx = np.ptp(c['tx'][post]) - np.ptp(c['tx'][pre])
    pp_gy = np.ptp(c['gy'][vpost]) - np.ptp(c['gy'][vpre])
    pp_gx = np.ptp(c['gx'][vpost]) - np.ptp(c['gx'][vpre])
    print('%-14s Δroll力矩=%+7.4f  Δpitch力矩=%+7.4f | 力矩峰峰变化=%+7.4f | roll角速率峰峰=%+6.1f°/s  pitch角速率峰峰=%+6.1f°/s'
          % (lab, dtx, dty, pp_tx, pp_gx, pp_gy))
