#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
项目 h 离线验收: 6 趟实飞 ULog × 12 次投放 回放对比
===============================================================
对每次投放, 在"DROW 保持段"内逐帧(10Hz)计算:
  t_eff(t)  = DROP_PRED_LEAD_S(0.25) + sqrt(2*max(0.05, H-0.25)/9.81)      # 指令→脱钩→落地
  pred(t)   = (pos(t) - aim(t)) + v(t) * t_eff(t)                          # h 的判据用的预测量
  truth(t)  = (pos(t+0.25) - aim(t)) + v(t+0.25) * t_fall(t)                # 由真实轨迹反算的落点偏差
输出:
  历史实际释放时刻的 |truth|   vs   h 新判据(预测<3cm + 软门控)选中的时刻的 |truth|
  以及该段的理论下限 min|truth| (说明还有多少余量)
aim 取 vehicle_local_position_setpoint (= 节点发布的瞄准点, 已含补偿).
用法: python3 scripts/test_h_drop_logic_20260930.py
"""
import numpy as np, math, os
from pyulog import ULog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [('9-19 d', 'log_422_2026-9-19-18-04-34.ulg'), ('9-23 e', 'log_424_2026-9-23-18-00-28.ulg'),
         ('9-24 e', '9.24/log_425_2026-9-24-17-39-10.ulg'), ('9-27 e', '9.27/log_427_2026-9-27-10-57-56.ulg'),
         ('9-27 f', '9.27/log_430_2026-9-27-11-12-20.ulg'), ('9-30 g', 'log_431_2026-9-30-10-22-22.ulg')]
LEAD = 0.25          # DROP_PRED_LEAD_S
FALL_H = 0.25        # DROP_PRED_FALL_H
PRED_ERR = 0.04      # DROP_PRED_ERR_M
CONFIRM = 0.20       # DROP_PRED_CONFIRM_S
VMAX = 0.20          # DROP_PRED_VMAX
H_TOL = 0.15         # DROP_HEIGHT_TOL (H >= 1.2-0.15 = 1.05)
VZ_TOL = 0.20        # DROP_VZ_TOL
DROP_HEIGHT = 1.2


def get(ul, n, mid=0):
    for x in ul.data_list:
        if x.name == n and x.multi_id == mid:
            return x
    return None


def hold_window(Tlp, H, tp, thr=1.42):
    """返回 DROP 保持段起点索引 (H 首次 < thr 并一直保持到 tp)"""
    m = (Tlp <= tp) & (Tlp > tp - 25) & (H < thr)
    idx = np.where(m)[0]
    if len(idx) < 5:
        return None
    start = idx[0]
    for k in range(len(idx) - 1, 0, -1):
        if Tlp[idx[k]] - Tlp[idx[k - 1]] > 0.4:
            start = idx[k]
            break
    return start, idx[-1]


rows = []
for lab, rel in FILES:
    path = os.path.join(ROOT, rel)
    if not os.path.exists(path):
        print(f"MISSING {rel}")
        continue
    ul = ULog(path)
    t0 = None
    for msg in ul.logged_messages:
        if 'Armed' in msg.message:
            t0 = msg.timestamp / 1e6
            break
    if t0 is None:
        t0 = ul.start_timestamp / 1e6
    lp = get(ul, 'vehicle_local_position'); Tlp = lp.data['timestamp'] / 1e6 - t0
    X = np.asarray(lp.data['x']); Y = np.asarray(lp.data['y'])
    VX = np.asarray(lp.data['vx']); VY = np.asarray(lp.data['vy']); VZ = np.asarray(lp.data['vz'])
    H = -(np.asarray(lp.data['z']) - np.asarray(lp.data['z'])[0])
    sp = get(ul, 'vehicle_local_position_setpoint'); Tsp = sp.data['timestamp'] / 1e6 - t0
    SX = np.asarray(sp.data['x']); SY = np.asarray(sp.data['y'])
    ao = get(ul, 'actuator_outputs', 1)
    Tao = ao.data['timestamp'] / 1e6 - t0
    aux = np.asarray(ao.data['output[4]'])
    # 两道脉冲起点
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
        w = hold_window(Tlp, H, tp)
        if w is None:
            continue
        j0, j1 = w
        T = np.arange(Tlp[j0], tp, 0.1)          # 10Hz 网格
        n = len(T)
        if n < 8:
            continue
        pos = np.vstack([np.interp(T, Tlp, X), np.interp(T, Tlp, Y)])
        vel = np.vstack([np.interp(T, Tlp, VX), np.interp(T, Tlp, VY)])
        aim = np.vstack([np.interp(T, Tsp, SX), np.interp(T, Tsp, SY)])
        hh = np.interp(T, Tlp, H)
        vz = np.interp(T, Tlp, VZ)
        vxy = np.hypot(vel[0], vel[1])
        teff = LEAD + np.sqrt(2.0 * np.maximum(0.05, hh - FALL_H) / 9.81)
        tfall = np.sqrt(2.0 * np.maximum(0.05, hh - FALL_H) / 9.81)
        # 预测: 用当前帧状态外推
        pred = np.hypot((pos[0] - aim[0]) + vel[0] * teff,
                        (pos[1] - aim[1]) + vel[1] * teff)
        # 真值: 用 0.25s 后的真实状态 + 那段时刻的落高
        T2 = T + LEAD
        pos2 = np.vstack([np.interp(T2, Tlp, X), np.interp(T2, Tlp, Y)])
        vel2 = np.vstack([np.interp(T2, Tlp, VX), np.interp(T2, Tlp, VY)])
        hh2 = np.interp(T2, Tlp, H)
        tfall2 = np.sqrt(2.0 * np.maximum(0.05, hh2 - FALL_H) / 9.81)
        truth = np.hypot((pos2[0] - aim[0]) + vel2[0] * tfall2,
                         (pos2[1] - aim[1]) + vel2[1] * tfall2)
        # 历史: 脉冲时刻附近
        hist = float(np.interp(tp, T, truth)) if T[-1] < tp else float(truth[-1])
        # h 新判据: pred<3cm 持续 0.2s + 软门控
        ok = (pred < PRED_ERR) & (vxy < VMAX) & (hh >= DROP_HEIGHT - H_TOL) & (np.abs(vz) < VZ_TOL)
        t_new, err_new = None, None
        run = 0
        for k in range(len(T)):
            run = run + 1 if ok[k] else 0
            if run * 0.1 >= CONFIRM:
                t_new = T[k - run + 1]; err_new = float(truth[k - run + 1]); break
        floor = float(truth.min())
        rows.append((lab, i + 1, tp - T[0], hist, t_new, err_new, floor, T[-1] - T[0]))
        print(f"{lab:7s} #{i+1}  等待段 {T[-1]-T[0]:4.1f}s | 历史释放 @+{tp-T[0]:5.1f}s 误差 {hist*100:5.1f}cm"
              f" | h 释放 " + (f"@+{t_new-T[0]:5.1f}s 误差 {err_new*100:5.1f}cm" if t_new is not None else "未触发(→重进)")
              + f" | 段内理论下限 {floor*100:4.1f}cm")

print("\n=== 汇总 ===")
hist = np.array([r[3] for r in rows]) * 100
newf = np.array([r[5] for r in rows if r[5] is not None]) * 100
fl = np.array([r[6] for r in rows]) * 100
print(f"  投放次数 {len(rows)}  历史实际释放 |落点偏差|: 中位 {np.median(hist):.1f}cm  最坏 {hist.max():.1f}cm")
if len(newf):
    print(f"  h 新判据释放  |落点偏差|: 中位 {np.median(newf):.1f}cm  最坏 {newf.max():.1f}cm  "
          f"({len(newf)}/{len(rows)} 投在等待段内直接触发)")
print(f"  段内理论下限 min|落点偏差|: 中位 {np.median(fl):.1f}cm  最坏 {fl.max():.1f}cm")
print(f"  ⇒ 最坏值 {hist.max():.1f}cm → {newf.max() if len(newf) else float('nan'):.1f}cm"
      f" (改善 {hist.max() - (newf.max() if len(newf) else hist.max()):.1f}cm)")
