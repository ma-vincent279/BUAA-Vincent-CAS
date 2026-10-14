#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停留段"实时反算桶位置"一致性分析 (2026-10-07)

问题: 为什么 16:02 那趟两瓶都进桶(偏 5cm), 16:27 那趟两瓶都偏(偏 15cm)?

方法(用与被测节点 i.py **完全相同**的相机模型, 因此不含 FX 不确定性):
  对 DROP 段每一行 1Hz 状态行, 反算"桶此刻在 NED 哪里":
      est_live(t) = 飞机相对 home 位置(t) + R(heading(t))·camera_pixel_to_level(px(t),pit(t),rol(t))·H(t)
  这就是 i.py 里 _roi_to_ned() 的逐帧版本(冻结估计 = 其中 5 帧的中值)。
  若桶不动、且飞控自报位置无误, est_live(t) 在停留段应是一条水平线。
  它自己跑多远 = "桶真的被推走了" 或 "飞控位置报偏了" 的量 —— 日志本身无法区分这两者,
  但两者对落点的影响完全一样(瞄准点相对桶偏了)。

用法: python3 analyze_dwell_consistency_20261007.py [日志...] [--fx X] [--fy Y]
"""
import argparse
import math
import re
from pathlib import Path

LOG_DIR = Path('/home/vincent/SITL/10.7')
DEFAULT_LOGS = [
    'i_mission_20261007_130234.log',
    'i_mission_20261007_141457.log',
    'i_mission_20261007_160201.log',
    'i_mission_20261007_162730.log',
]
# 用户现场卷尺真值 (桶心为参照; 正=左, 正=前/远) —— 用于最后一栏对照
TRUTH = {
    ('130234', 1): '偏很多(未量)', ('130234', 2): '偏右9 偏下9 (命中)',
    ('141457', 1): '偏右6 偏前11', ('141457', 2): '偏前12',
    ('160201', 1): '偏右5 (进桶)', ('160201', 2): '偏右5 偏下5 (进桶)',
    ('162730', 1): '偏左14 偏下14', ('162730', 2): '偏左15 偏下15',
}

RE_EPOCH = re.compile(r'\[(INFO|WARN|ERROR)\] \[([\d.]+)\]')
RE_STATUS = re.compile(
    r'\[t=\s*([\d.]+)\s+(\w+)\s+ph=\s*([\d.]+)s\]\s+'
    r'NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+H=\s*([\d.]+)\s+'
    r'hdg=\s*([-\d.]+)\s+pit=\s*([-+\d.]+)\s+rol=\s*([-+\d.]+)\s+'
    r'vNED=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\)\s+det=(\d+)\s+age=\s*([\d.]+)'
    r'.*?px=\(([-+\d]+),([-+\d]+)\)err=([\d]+)px')
RE_FREEZE = re.compile(
    r'i模式: 1\.2m 一次性修正 delta=\(([-+\d.]+),([-+\d.]+)\)m '
    r'\(桶估计=\(([-+\d.eE]+),\s*([-+\d.eE]+)\)\) → 冻结保持点 \(([-+\d.]+),([-+\d.]+)\) '
    r'\[纯几何: 保持点-桶估计=\(([-+\d.]+),([-+\d.]+)\)\] 补偿(B\d)=\(([-+\d.]+),([-+\d.]+)\)')
RE_DROP = re.compile(
    r'>>> DROP #(\d)!.*?NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+触发方式=(\S+)'
    r'.*?到位err=([\d.]+)cm pred=([\d.]+)cm 停留([\d.]+)s vxy=([\d.]+)m/s')


def cam_px_to_level(cx, cy, pitch, roll, fx, fy, imw=640.0, imh=360.0):
    """与 i.py camera_pixel_to_level 逐行一致."""
    CX, CY = imw / 2.0, imh / 2.0
    fwd = (CY - cy) / fy
    right = (cx - CX) / fx
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rolled_right = cr * right - sr
    rolled_down = sr * right + cr
    level_fwd = cp * fwd + sp * rolled_down
    level_down = -sp * fwd + cp * rolled_down
    if level_down <= 1e-6:
        return None
    return level_fwd / level_down, rolled_right / level_down


def body_to_ned(h, fwd, lat):
    """与 i.py _body_to_ned 一致 (heading 弧度, NED)."""
    ch, sh = math.cos(h), math.sin(h)
    return fwd * ch - lat * sh, fwd * sh + lat * ch


def to_fwd_right(h, dx, dy):
    """NED 相对向量 -> (前后, 左右); 右: 正=机体右侧."""
    ch, sh = math.cos(h), math.sin(h)
    return dx * ch + dy * sh, -dx * sh + dy * ch


def parse(path):
    home, frees, drops, rows = None, [], [], []
    for ln in path.read_text(errors='ignore').splitlines():
        me = RE_EPOCH.search(ln)
        epoch = float(me.group(2)) if me else None
        m = re.search(r'Home locked: NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', ln)
        if m:
            home = tuple(float(v) for v in m.groups())
        m = RE_FREEZE.search(ln)
        if m:
            frees.append({'epoch': epoch, 'est': (float(m.group(3)), float(m.group(4))),
                          'hold': (float(m.group(5)), float(m.group(6))),
                          'geom': (float(m.group(7)), float(m.group(8))),
                          'bottle': m.group(9), 'comp': (float(m.group(10)), float(m.group(11)))})
        m = RE_DROP.search(ln)
        if m:
            drops.append({'epoch': epoch, 'n': int(m.group(1)),
                          'mode': m.group(5), 'arrive_err': float(m.group(6)),
                          'pred': float(m.group(7)), 'dwell': float(m.group(8)), 'vxy': float(m.group(9))})
        m = RE_STATUS.search(ln)
        if m:
            g = m.groups()
            rows.append({'epoch': epoch, 't': float(g[0]), 'phase': g[1],
                         'ned': tuple(float(v) for v in g[3:6]), 'H': float(g[6]),
                         'hdg': float(g[7]), 'pit': math.radians(float(g[8])),
                         'rol': math.radians(float(g[9])), 'det': int(g[13]),
                         'age': float(g[14]), 'ex': float(g[15]), 'ey': float(g[16])})
    return home, frees, drops, rows


def live_est(row, home, h, fx, fy):
    prj = cam_px_to_level(320.0 + row['ex'], 180.0 - row['ey'], row['pit'], row['rol'], fx, fy)
    if prj is None:
        return None
    cf, cr_ = (v * row['H'] for v in prj)
    dx, dy = body_to_ned(h, cf, cr_)
    return (row['ned'][0] - home[0] + dx, row['ned'][1] - home[1] + dy)


def analyze(path, fx, fy, summary):
    home, frees, drops, rows = parse(path)
    tag = path.stem.split('_')[-1]
    print('=' * 104)
    print(f'{path.name}   home={tuple(round(v,2) for v in home)}')
    for fr, dr in zip(frees, drops):
        # DROP 段: 冻结前 2s 到释放时刻之间的新鲜行
        seg = [r for r in rows if r['phase'] == 'DROP' and r['epoch'] is not None
               and fr['epoch'] - 2.0 <= r['epoch'] <= dr['epoch']
               and r['age'] <= 0.35 and r['det'] >= 1 and r['H'] > 0.3]
        if len(seg) < 2:
            continue
        h = math.radians(seg[-1]['hdg'])
        f_row = min(seg, key=lambda r: abs(r['epoch'] - fr['epoch']))
        r_row = seg[-1]
        live = [(r, live_est(r, home, h, fx, fy)) for r in seg]
        live = [(r, p) for r, p in live if p is not None]
        rel = live[-1]
        fz = min(live, key=lambda rp: abs(rp[0]['epoch'] - fr['epoch']))
        xs = [p[0] for _, p in live]; ys = [p[1] for _, p in live]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        # 反算桶 — 冻结估计
        df = to_fwd_right(h, rel[1][0] - fr['est'][0], rel[1][1] - fr['est'][1])
        # 桶(反算) 相对 飞机 —— 冻结帧 / 释放帧
        bfz = to_fwd_right(h, fz[1][0] - (fz[0]['ned'][0] - home[0]), fz[1][1] - (fz[0]['ned'][1] - home[1]))
        brel = to_fwd_right(h, rel[1][0] - (rel[0]['ned'][0] - home[0]), rel[1][1] - (rel[0]['ned'][1] - home[1]))
        print(f"\n--- 第{dr['n']}投 {fr['bottle']}  补偿={fr['comp']}  到位err={dr['arrive_err']}cm "
              f"pred={dr['pred']}cm 停留{dr['dwell']}s vxy={dr['vxy']}  方式={dr['mode'][:26]}")
        print(f"    est_frozen={tuple(round(v,3) for v in fr['est'])}  hold={tuple(round(v,3) for v in fr['hold'])}"
              f"  [纯几何]={fr['geom']}   冻结帧px=({f_row['ex']:+.0f},{f_row['ey']:+.0f}) H={f_row['H']:.2f}"
              f" pit={math.degrees(f_row['pit']):+.1f}")
        print(f"    (1) 冻结→释放, 反算桶位置自己跑了 = {spread*100:5.1f} cm"
              f"   |  反算桶(释放帧) − est_frozen = 前{df[0]*100:+6.1f}cm 右{df[1]*100:+6.1f}cm"
              f"  |{math.hypot(*df)*100:5.1f}|cm")
        print(f"    (2) 冻结帧 桶(反算)相对飞机 = 前{bfz[0]*100:+6.1f}cm 右{bfz[1]*100:+6.1f}cm"
              f"  |{math.hypot(*bfz)*100:5.1f}|cm     释放帧 前{brel[0]*100:+6.1f}cm 右{brel[1]*100:+6.1f}cm"
              f"  |{math.hypot(*brel)*100:5.1f}|cm")
        print(f"    (3) 用户卷尺真值: {TRUTH.get((tag, dr['n']), '?')}")
        summary.append((tag, dr['n'], fr['bottle'], math.hypot(*bfz), spread, math.hypot(*df),
                        dr['arrive_err'], TRUTH.get((tag, dr['n']), '?')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logs', nargs='*', default=None)
    ap.add_argument('--fx', type=float, default=640.0 / (2.0 * math.tan(math.radians(85) / 2.0)))
    ap.add_argument('--fy', type=float, default=360.0 / (2.0 * math.tan(math.radians(54) / 2.0)))
    a = ap.parse_args()
    print(f'相机模型: FX={a.fx:.1f} FY={a.fy:.1f}  (i.py: HFOV=85deg VFOV=54deg, 640x360)')
    logs = [Path(p) for p in a.logs] or [LOG_DIR / n for n in DEFAULT_LOGS]
    summary = []
    for p in logs:
        if p.exists():
            analyze(p, a.fx, a.fy, summary)
    print('\n' + '=' * 104)
    print(f"{'趟':8s} {'投':2s} {'瓶':3s} {'冻结帧|桶−机|':>12s} {'停留段漂移':>10s} {'释放帧偏差':>10s} "
          f"{'到位err':>8s}   卷尺真值")
    for tag, n, bottle, g, spread, dev, ae, truth in summary:
        print(f"{tag:8s} {n:^2d} {bottle:3s} {g*100:11.1f}cm {spread*100:9.1f}cm {dev*100:9.1f}cm "
              f"{ae:7.1f}cm   {truth}")


if __name__ == '__main__':
    main()
