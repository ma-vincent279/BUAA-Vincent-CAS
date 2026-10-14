#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""10.3 两趟实飞: 投弹保持段"晃不晃"量化 (4 投) —— 位置漂移 / 姿态摆动 / 速度 / 相机残差"""
import numpy as np, math, os
from pyulog import ULog

ROOT = '/home/vincent/SITL/10.3'
F = [('10-3 #1 (老)', 'log_439_2026-10-3-14-40-44.ulg'), ('10-3 #2 (新)', 'log_440_2026-10-3-16-43-12.ulg')]


def get(ul, n, mid=0):
    for d in ul.data_list:
        if d.name == n and d.multi_id == mid:
            return d
    return None


print('%-13s %5s %8s %8s %8s %7s %9s %8s %8s' % ('航班/投次', '保持s', 'Δx峰峰', 'Δy峰峰', 'conv峰峰', 'vxy均', 'vxy峰', 'roll峰峰', 'pitch峰峰'))
for lab, rel in F:
    ul = ULog(os.path.join(ROOT, rel))
    t0 = None
    for m in ul.logged_messages:
        if 'Armed' in m.message:
            t0 = m.timestamp / 1e6
            break
    t0 = t0 or ul.start_timestamp / 1e6
    lp, sp = get(ul, 'vehicle_local_position'), get(ul, 'vehicle_local_position_setpoint')
    T = lp.data['timestamp'] / 1e6 - t0
    X, Y = np.asarray(lp.data['x']), np.asarray(lp.data['y'])
    VX, VY = np.asarray(lp.data['vx']), np.asarray(lp.data['vy'])
    H = -(np.asarray(lp.data['z']) - np.asarray(lp.data['z'])[0])
    Ts = sp.data['timestamp'] / 1e6 - t0
    SX, SY = np.asarray(sp.data['x']), np.asarray(sp.data['y'])
    att = get(ul, 'vehicle_attitude')
    Ta = att.data['timestamp'] / 1e6 - t0
    q = [np.asarray(att.data['q[%d]' % i]) for i in range(4)]
    w, x_, y_, z_ = q
    roll = np.degrees(np.arctan2(2 * (w * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y_ - z_ * x_), -1, 1)))
    ao = get(ul, 'actuator_outputs', 1)
    Tao = ao.data['timestamp'] / 1e6 - t0
    aux = np.asarray(ao.data['output[4]'])
    pulses = []
    for mask in (aux > 1600, aux < 1400):
        idx = np.where(mask)[0]
        if len(idx):
            s = idx[0]
            for k in range(1, len(idx)):
                if Tao[idx[k]] - Tao[idx[k - 1]] > 0.5:
                    pulses.append(Tao[s]); s = idx[k]
            pulses.append(Tao[s])
    pulses = sorted(set(round(p, 2) for p in pulses if p > 20))[:2]
    for i, tp in enumerate(pulses):
        m = (T <= tp) & (T > tp - 25) & (H < 1.42)
        idx = np.where(m)[0]
        if len(idx) < 5:
            continue
        st = idx[0]
        for k in range(len(idx) - 1, 0, -1):
            if T[idx[k]] - T[idx[k - 1]] > 0.4:
                st = idx[k]
                break
        g = np.arange(T[st], tp, 0.1)
        x = np.interp(g, T, X); y = np.interp(g, T, Y)
        vx = np.interp(g, T, VX); vy = np.interp(g, T, VY)
        vxy = np.hypot(vx, vy)
        sx = np.interp(g, Ts, SX); sy = np.interp(g, Ts, SY)
        conv = np.hypot(x - sx, y - sy)
        r = np.interp(g, Ta, roll); p = np.interp(g, Ta, pitch)
        print('%-13s %5.1f %7.1fcm %7.1fcm %7.1fcm %6.3f %8.3f %7.2f° %7.2f°' % (
            lab + ' #%d' % (i + 1), g[-1] - g[0], np.ptp(x) * 100, np.ptp(y) * 100,
            np.ptp(conv) * 100, vxy.mean(), vxy.max(), np.ptp(r), np.ptp(p)))
