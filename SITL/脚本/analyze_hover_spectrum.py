#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_hover_spectrum.py — 定点悬停误差的**频谱**分析 (2026-10-12)

为什么做: 定点悬停的水平 RMS 16cm / 垂直 2.5cm, 这个"垂直好、水平差"的组合
有两种完全不同的机理, 而**频域能一眼分开**:

  · **控制极限环 (欠阻尼位置环)** → 误差是**窄带**的: PSD 在某一个频率上有尖峰,
    自相关缓慢衰减, 信号近乎周期 ⇒ 这是控制器/执行机构的问题, 调增益可治。
  · **估计噪声驱动 (EKF/GPS 水平估计)** → 误差是**宽带**的, 且会呈现随机游走
    (低频 1/f²抬升) ⇒ 这是观测/滤波的问题, 调增益治不了。

同时对比**垂直**通道作对照 (已知它只有 2.5cm)。

用法: python3 scripts/analyze_hover_spectrum.py <csv> [<csv> ...]
"""
import csv
import math
import pathlib
import sys

import numpy as np

FS = 20.0   # 探针 log_hz


def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: (v if k == 'step' else float(v)) for k, v in r.items()})
            except (TypeError, ValueError):
                continue
    return rows


def band_stats(path, step):
    rows = [r for r in load(path) if r['step'] == step]
    if len(rows) < 200:
        return None
    t = np.array([r['t'] for r in rows])
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    out = {}
    for tag, key in (('X', 'x'), ('Y', 'y'), ('Z', 'z'), ('vxy', None)):
        if key is None:
            sig = np.hypot(np.array([r['vx'] for r in rows]),
                           np.array([r['vy'] for r in rows]))
        else:
            sp = np.array([r['sp_' + key] for r in rows])
            sig = np.array([r[key] for r in rows]) - sp
        sig0 = sig.copy()
        # 关键: 先去**线性趋势** —— 否则窗内缓慢漂移会在 1/窗长 处伪装成"窄带极限环"
        tt = np.arange(len(sig)) * dt
        A = np.vstack([tt, np.ones_like(tt)]).T
        coef, *_ = np.linalg.lstsq(A, sig, rcond=None)
        drift = float(coef[0]) * 100.0          # cm/s
        sig = sig - A @ coef
        n = len(sig)
        w = np.hanning(n)
        sw = sig * w
        f = np.fft.rfftfreq(n, d=dt)
        P = np.abs(np.fft.rfft(sw)) ** 2
        P[0] = 0.0
        tot = P.sum()
        k = int(np.argmax(P))
        fpk = f[k]
        # 峰附近 ±20% 的能量占比 (窄带性)
        m = (f > fpk * 0.8) & (f < fpk * 1.2)
        frac = float(P[m].sum() / tot) if tot > 0 else 0.0
        # 一阶自相关衰减 (到 1/e 的滞后)
        ac = np.correlate(sig, sig, 'full')[n - 1:]
        ac = ac / ac[0]
        tau = next((i * dt for i, v in enumerate(ac) if v < 1.0 / math.e), None)
        out[tag] = dict(rms=float(np.std(sig)) * 100, fpk=float(fpk), frac=frac, tau=tau,
                        drift=drift, raw_rms=float(np.std(sig0)) * 100)
    return out


def main():
    for p in (sys.argv[1:] or []):
        p = pathlib.Path(p)
        if not p.exists():
            continue
        rows = load(p)
        steps = sorted({r['step'] for r in rows if r['step'].startswith('HOLD')})
        print("=" * 96)
        print(f"{p.parent.name}/{p.name}   定点阶段 {steps}")
        print(f"\n{'阶段':<9}{'通道':<6}{'漂移(cm/s)':<12}{'总RMS':<9}{'去趋势RMS':<11}{'主频(Hz)':<10}{'峰占比':<9}{'1/e':<8}{'判读'}")
        for s in steps:
            r = band_stats(p, s)
            if not r:
                continue
            for tag in ('X', 'Y', 'Z'):
                d = r[tag]
                verdict = ''
                if tag in ('X', 'Y'):
                    verdict = ('窄带 ⇒ 极限环' if d['frac'] > 0.25 else
                               ('宽带 ⇒ 噪声驱动/随机游走' if d['frac'] < 0.12 else '介于两者'))
                print(f"{s:<9}{tag:<6}{d['drift']:<+12.3f}{d['raw_rms']:<9.1f}{d['rms']:<11.1f}"
                      f"{d['fpk']:<10.2f}{d['frac']:<9.2f}"
                      f"{('%.2fs' % d['tau']) if d['tau'] else '>窗长':<8}{verdict}")
            print()
        print()


if __name__ == '__main__':
    main()
