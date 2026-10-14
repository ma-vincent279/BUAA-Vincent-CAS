#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""j 项目"相机系释放判据"的离线回放验收 (2026-10-08)

问题: i 模式释放判据 `_pred = |(est_frozen + comp) − 飞机位置 − v·t_eff|` **全在飞控坐标系里**,
      对"冻结之后参照漂移"无感 —— 实测 `dev = 释放帧 est_live − est_frozen`, 且冻结估计误差 `E ≈ −dev`
      (6 投吻合到 1~3.4cm); 今天 14 投: `dev ≥22cm` 的 7 投**全部偏差 ≥12cm**, `dev ≤14cm` 的 7 投**全部 ≤10cm**。

j 改动①: 判据里的 `est_frozen` → **当前帧相机反算的 `est_live`** (相机确认此刻真在补偿后站位上),
      停稳仍由飞控速度 `vxy` 把关。**瞄准点不动** ⇒ 飞机轨迹与历史日志相同 ⇒
      "同一条轨迹上换一个释放时刻"的回放**因果严格成立** (不含重锚选项)。

保真度门槛 (必须先过): 用同一套判据回放 **旧** 口径, 释放时刻应与日志实测一致 (±0.6s),
      且该样本的 old_pred 应与日志 `pred=` 相符 (±2cm) —— 否则说明我的反算/时序有错, 结论不可用。

落点预测 (无假设): `落点 = −dev(释放时刻) + C` ⇒ `C = 实测落点 + dev(实际释放)`
      ⇒ `预测落点 = 实测落点 + dev(实际) − dev(新判据释放)`  ⇒ **改善量只由 |dev| 下降决定, 与 C 无关**。

用法: python3 scripts/replay_j_release_gate_20261008.py [-v] [--json out.json]
"""
import argparse
import json
import math
import re
from pathlib import Path

LOG_DIR = Path('/home/vincent/SITL/10.7')
I_DWELL_S, I_VMAX, I_PRED_M, I_PRED_TEFF = 5.0, 0.15, 0.09, 0.70
I_RELEASE_TIMEOUT, I_DEADLINE_WIN = 20.0, 5.0
BUCKET_R = 12.5      # cm
FX = 640.0 / (2.0 * math.tan(math.radians(85) / 2.0))
FY = 360.0 / (2.0 * math.tan(math.radians(54) / 2.0))

# 现场真值 (左=+, 前=+; 右→负, "下"=后方→负); None = 该轴未量
TRUTH = {
    ('130234', 1): (None, None), ('130234', 2): (-9.0, -9.0),
    ('141457', 1): (-6.0, +11.0), ('141457', 2): (None, +12.0),
    ('160201', 1): (-5.0, None), ('160201', 2): (-5.0, -5.0),
    ('162730', 1): (+14.0, -14.0), ('162730', 2): (+15.0, -15.0),
    ('181852', 1): (-10.0, None), ('181852', 2): (0.0, None),
    ('184418', 1): (0.0, None), ('184418', 2): (-10.0, None),
    ('093428', 1): (+9.9, -9.8), ('093428', 2): (+12.0, 0.0),
}
LOGS = ['i_mission_20261007_130234.log', 'i_mission_20261007_141457.log',
        'i_mission_20261007_160201.log', 'i_mission_20261007_162730.log',
        'i_mission_20261007_181852.log', 'i_mission_20261007_184418.log',
        'i_mission_20261008_093428.log']

RE_EPOCH = re.compile(r'\[(?:INFO|WARN|ERROR)\] \[([\d.]+)\]')
RE_HOME = re.compile(r'Home locked: NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)')
RE_STATUS = re.compile(
    r'\[t=\s*([\d.]+)\s+(\w+)\s+ph=\s*([\d.]+)s\]\s+'
    r'NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+H=\s*([\d.]+)\s+'
    r'hdg=\s*([-\d.]+)\s+pit=\s*([-+\d.]+)\s+rol=\s*([-+\d.]+)\s+'
    r'vNED=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\)\s+det=(\d+)\s+age=\s*([\d.]+)'
    r'.*?px=\(([-+\d]+),([-+\d]+)\)err=([\d]+)px')
RE_FREEZE = re.compile(
    r'i模式: 1\.2m 一次性修正 delta=.*?\(桶估计=\(([-+\d.eE]+),\s*([-+\d.eE]+)\)\) → 冻结保持点 '
    r'\(([-+\d.]+),([-+\d.]+)\) \[纯几何: 保持点-桶估计=\(([-+\d.]+),([-+\d.]+)\)\] '
    r'补偿(B\d)=\(([-+\d.]+),([-+\d.]+)\)')
RE_ARRIVE = re.compile(r'已到达保持点 err=([\d.]+)cm')
RE_DROP = re.compile(
    r'>>> DROP #(\d)!.*?NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)\s+触发方式=(\S+)'
    r'.*?到位err=([\d.]+)cm pred=([\d.]+)cm 停留([\d.]+)s vxy=([\d.]+)m/s')


def px_to_level(ex, ey, pitch, roll):
    fwd, right = ey / FY, ex / FX
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rr = cr * right - sr
    rd = sr * right + cr
    lf = cp * fwd + sp * rd
    ld = -sp * fwd + cp * rd
    return None if ld <= 1e-6 else (lf / ld, rr / ld)


def b2n(h, fwd, lat):
    ch, sh = math.cos(h), math.sin(h)
    return fwd * ch - lat * sh, fwd * sh + lat * ch


def to_fr(h, dx, dy):
    ch, sh = math.cos(h), math.sin(h)
    return dx * ch + dy * sh, -dx * sh + dy * ch


def parse(path):
    home, rows, frees, drops, arrives = None, [], [], [], []
    for ln in path.read_text(errors='ignore').splitlines():
        me = RE_EPOCH.search(ln)
        ep = float(me.group(1)) if me else None
        m = RE_HOME.search(ln)
        if m:
            home = tuple(float(v) for v in m.groups())
        m = RE_FREEZE.search(ln)
        if m:
            frees.append({'epoch': ep, 'est': (float(m.group(1)), float(m.group(2))),
                          'hold': (float(m.group(3)), float(m.group(4))),
                          'comp': (float(m.group(8)), float(m.group(9))), 'bottle': m.group(7)})
        m = RE_ARRIVE.search(ln)
        if m:
            arrives.append({'epoch': ep, 'err': float(m.group(1))})
        m = RE_DROP.search(ln)
        if m:
            drops.append({'epoch': ep, 'n': int(m.group(1)), 'arrive_err': float(m.group(6)),
                          'pred': float(m.group(7)), 'dwell': float(m.group(8)),
                          'vxy': float(m.group(9)), 'ned': tuple(float(v) for v in m.groups()[1:4])})
        m = RE_STATUS.search(ln)
        if m:
            g = m.groups()
            rows.append({'epoch': ep, 't': float(g[0]), 'phase': g[1],
                         'ned': tuple(float(v) for v in g[3:6]), 'H': float(g[6]),
                         'hdg': float(g[7]), 'pit': math.radians(float(g[8])),
                         'rol': math.radians(float(g[9])),
                         'vx': float(g[10]), 'vy': float(g[11]),
                         'vxy': math.hypot(float(g[10]), float(g[11])),
                         'det': int(g[13]), 'age': float(g[14]),
                         'ex': float(g[15]), 'ey': float(g[16])})
    return home, rows, frees, drops, arrives


def replay(path, verbose):
    tag = path.stem.split('_')[-1]
    home, rows, frees, drops, arrives = parse(path)
    # 第 i 个 freeze / arrive / drop 属于第 i 投 (按出现顺序配对, 避免时间窗误配)
    assert len(frees) == len(drops) == len(arrives), f'{tag}: freeze/arrive/drop 数量不一致'
    out = []
    for i, (fr, dr, ar) in enumerate(zip(frees, drops, arrives)):
        comp_y, comp_xf = fr['comp']                       # (Y 左移, XF 后移)
        seg = [r for r in rows if r['phase'] == 'DROP' and r['epoch'] is not None
               and fr['epoch'] - 2.5 <= r['epoch'] <= dr['epoch'] + 1.5
               and r['age'] <= 0.35 and r['det'] >= 1 and r['H'] > 0.3]
        if len(seg) < 2:
            continue
        h = math.radians(seg[-1]['hdg'])
        lx, ly = math.sin(h), -math.cos(h)
        fx_, fy_ = math.cos(h), math.sin(h)

        def hold_of(est):
            return (est[0] + comp_y * lx - comp_xf * fx_, est[1] + comp_y * ly - comp_xf * fy_)

        recs = []
        for r in seg:
            prj = px_to_level(r['ex'], r['ey'], r['pit'], r['rol'])
            if prj is None:
                continue
            cf, cr = prj[0] * r['H'], prj[1] * r['H']
            dx, dy = b2n(h, cf, cr)
            est_live = (r['ned'][0] - home[0] + dx, r['ned'][1] - home[1] + dy)
            p = (r['ned'][0] - home[0], r['ned'][1] - home[1])
            hl, hn = hold_of(fr['est']), hold_of(est_live)
            # pred = |hold − (飞机位置 + v·t_eff)|   (i.py: hold − 飞机位置 − v·t_eff)
            po = (p[0] + r['vx'] * I_PRED_TEFF, p[1] + r['vy'] * I_PRED_TEFF)
            of, orr = to_fr(h, po[0] - hl[0], po[1] - hl[1])
            nf, nrr = to_fr(h, po[0] - hn[0], po[1] - hn[1])
            df, drr = to_fr(h, est_live[0] - fr['est'][0], est_live[1] - fr['est'][1])
            recs.append({'epoch': r['epoch'], 't': r['t'], 'vxy': r['vxy'], 'px': (r['ex'], r['ey']),
                         'H': r['H'], 'dev': (df, drr), 'old_pred': math.hypot(of, orr),
                         'new_pred': math.hypot(nf, nrr)})
        if len(recs) < 2:
            continue
        # 实际释放帧 = 释放时刻前最后一个样本
        rel = [s for s in recs if s['epoch'] <= dr['epoch']][-1]

        def gate(key):
            for s in recs:
                if s['epoch'] >= ar['epoch'] + I_DWELL_S and s['vxy'] < I_VMAX and s[key] < I_PRED_M:
                    return s, 'gate'
            dl = [s for s in recs if s['epoch'] >= fr['epoch'] + I_RELEASE_TIMEOUT]
            if dl:                                   # 20s 兜底后的'挑好时机'窗口
                win = [s for s in dl if s['epoch'] <= dl[0]['epoch'] + I_DEADLINE_WIN] or dl
                return min(win, key=lambda s: s[key]), 'deadline'
            return min(recs, key=lambda s: s[key]), '未复现(异常)'
        s_old, how_old = gate('old_pred')
        s_new, how_new = gate('new_pred')
        d_act, d_new = rel['dev'], s_new['dev']
        fid = {'t_err': abs((s_old['epoch'] - fr['epoch']) - (dr['epoch'] - fr['epoch'])),
               'pred_err': abs(s_old['old_pred'] * 100 - dr['pred'])}
        # 落点预测 (逐轴, 只在该轴有真值时)
        pl = {}
        for nm, idx, sgn in (('左', 1, -1.0), ('前', 0, +1.0)):
            tv = TRUTH[tag, dr['n']][idx]
            pl[nm] = (tv, None if tv is None else sgn * (sgn * tv + (d_act[idx] - d_new[idx]) * 100))
        out.append({'tag': tag, 'drop': dr['n'], 'bottle': fr['bottle'], 'comp': fr['comp'],
                    'dev_act': (d_act[0] * 100, d_act[1] * 100), 'dev_new': (d_new[0] * 100, d_new[1] * 100),
                    'dev_min_mag': math.hypot(min(abs(s['dev'][0]) for s in recs),
                                              min(abs(s['dev'][1]) for s in recs)) * 100,
                    'how_old': how_old, 'how_new': how_new, 't_old': s_old['epoch'] - fr['epoch'],
                    't_new': s_new['epoch'] - fr['epoch'], 't_act': dr['epoch'] - fr['epoch'],
                    'pred_log': dr['pred'], 'old_pred': s_old['old_pred'] * 100,
                    'new_pred': s_new['new_pred'] * 100, 'fid': fid,
                    'truth': TRUTH[tag, dr['n']], 'pred_land': pl, 'n': len(recs)})
        if verbose:
            print(f"\n-- {tag} #{dr['n']} {fr['bottle']} comp=(Y{comp_y:+.3f},XF{comp_xf:+.3f}) 样本{len(recs)}")
            print(f"   实际: t={dr['epoch']-fr['epoch']:.1f}s pred={dr['pred']}cm vxy={dr['vxy']}")
            print(f"   旧判据回放: {how_old} t={s_old['epoch']-fr['epoch']:.1f}s old_pred={s_old['old_pred']*100:.1f}cm"
                  f"  [保真 t 差{fid['t_err']:.2f}s / pred 差{fid['pred_err']:.2f}cm]")
            print(f"   新判据回放: {how_new} t={s_new['epoch']-fr['epoch']:.1f}s new_pred={s_new['new_pred']*100:.1f}cm")
            print(f"   |dev| 实际 {math.hypot(*d_act)*100:.1f}cm → 新 {math.hypot(*d_new)*100:.1f}cm")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    recs = []
    for n in LOGS:
        p = LOG_DIR / n
        if p.exists():
            recs += replay(p, a.verbose)
    print('\n' + '=' * 116)
    print(f"{'趟':8s}{'投':^3s}{'瓶':4s}{'|dev|实际':>10s}{'|dev|新':>9s}{'段min':>7s}"
          f"{'t 实/旧/新(s)':>18s}{'旧判据':>9s}{'新判据':>9s}{'保真差':>9s}   真值→预测 (左,前)")
    bad_fid = 0
    for r in recs:
        if r['fid']['t_err'] > 0.6 or r['fid']['pred_err'] > 2.0:
            bad_fid += 1
        f = lambda x: '  --  ' if x is None else f'{x:+5.1f}'
        pl, t = r['pred_land'], r['truth']
        print(f"{r['tag']:8s}{r['drop']:^3d}{r['bottle']:4s}"
              f"{math.hypot(*r['dev_act']):9.1f}cm{math.hypot(*r['dev_new']):8.1f}cm{r['dev_min_mag']:6.1f}cm"
              f"{r['t_act']:7.1f}{r['t_old']:6.1f}{r['t_new']:6.1f}"
              f"{r['how_old'][:8]:>9s}{r['how_new'][:8]:>9s}"
              f"{r['fid']['t_err']:8.2f}s   ({f(t[0])},{f(t[1])}) → ({f(pl['左'][1])},{f(pl['前'][1])})")
    print('=' * 116)
    ok = tot = 0
    lines = []
    for r in recs:
        t = r['truth']
        old_vals = [abs(v) for v in t if v is not None]
        new_vals = [abs(r['pred_land'][k][1]) for k in ('左', '前') if r['pred_land'][k][1] is not None]
        if not old_vals or not new_vals:
            continue
        e_o, e_n = math.hypot(*old_vals), math.hypot(*new_vals)
        tot += 1
        old_in, new_in = e_o <= BUCKET_R, e_n <= BUCKET_R
        ok += new_in
        lines.append(f"  {r['tag']} #{r['drop']}: |实测| {e_o:5.1f}cm {'进' if old_in else '出'}"
                     f"  →  |预测| {e_n:5.1f}cm {'进' if new_in else '出'}")
    print('\n'.join(lines))
    old_in = sum(1 for r in recs if (lambda v: bool(v) and math.hypot(*[abs(x) for x in v]) <= BUCKET_R)(
        [x for x in r['truth'] if x is not None]))
    print(f"\n保真度: {len(recs)} 投中 {len(recs)-bad_fid} 投满足 (释放时刻差 ≤0.6s 且 pred 差 ≤2cm)"
          f"{'  ✓ 回放可信' if bad_fid == 0 else f'  ⚠ 有 {bad_fid} 投不达标'}")
    print(f"进桶: 旧 {old_in}/{tot} → 新 {ok}/{tot}   (桶口半径 {BUCKET_R}cm; 未量轴按 0 计, 两栏同口径可比)")
    if a.json:
        Path(a.json).write_text(json.dumps(recs, ensure_ascii=False, indent=1))
        print(f'明细 → {a.json}')


if __name__ == '__main__':
    main()
