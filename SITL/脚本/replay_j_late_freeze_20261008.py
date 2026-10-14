#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""j 项目设计验证: "晚冻结 + 尽快投" 能否把落点误差压下来 (2026-10-08)

发现 (回放 + 真值): 判据全在飞控坐标系里 ⇒ 看不见"冻结之后参照漂移"。而实测
  `落点误差 L(t) = C − dev(t)`  (dev = 当前帧相机反算 est_live − 冻结 est; C = ε + 补偿 + r 为常数)
⇒ **冻结到释放之间的漂移 dev 直接进落点**。i 模式现在是"早冻结(还在收敛时) + 停留 5s + 投"
   ⇒ 冻结→释放间隔 5~6.5s, 把这几秒的漂移全吃进落点。

本脚本: 1) 打印每投的 dev(t) 序列与漂移率;  2) 用**实测后释放段的样本**估计"晚冻结+1.5s 内投"
   会吃到多少漂移, 并给出改造后的落点预测:
      L_new = L_实测 + dev(实际释放) − Δcomp − [dev(t_act+1.5s) − dev(t_act)]
   其中 Δcomp = 补偿改动量 (B1: Y 0.18→0.35 ⇒ 右向 −17cm; B2: Y 0.00→0.05 ⇒ 右向 −5cm)
   注意: 晚冻结不动飞机 (冻结时刻的 est 只是重算, 飞机仍停在原保持点) ⇒ 轨迹近似不变, 回放成立。

用法: python3 scripts/replay_j_late_freeze_20261008.py [-v]
"""
import argparse
import math
from pathlib import Path

import importlib.util
_spec = importlib.util.spec_from_file_location(
    'rp', str(Path(__file__).with_name('replay_j_release_gate_20261008.py')))
rp = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(rp)

GAP = 1.5                      # 晚冻结 → 释放的间隔 (s)
DCOMP = {('130234', 'B1'): -0.00, ('141457', 'B1'): -0.00,   # 老补偿 (Y=0) 已是基准
         ('160201', 'B1'): -0.17, ('162730', 'B1'): -0.17, ('160201', 'B2'): -0.00,
         ('162730', 'B2'): -0.00, ('181852', 'B1'): -0.17, ('184418', 'B1'): -0.17,
         ('181852', 'B2'): -0.05, ('184418', 'B2'): -0.05,
         ('093428', 'B1'): -0.17, ('093428', 'B2'): -0.05,
         ('130234', 'B2'): +0.05}   # 13:02 B2 当时 Y=−0.05 ⇒ 改成 0 是 +5cm


def series(path, tag):
    home, rows, frees, drops, arrives = rp.parse(path)
    out = []
    for i, (fr, dr) in enumerate(zip(frees, drops)):
        seg = [r for r in rows if r['phase'] == 'DROP' and r['epoch'] is not None
               and fr['epoch'] - 2.5 <= r['epoch'] <= dr['epoch'] + 4.0
               and r['age'] <= 0.35 and r['det'] >= 1 and r['H'] > 0.3]
        if len(seg) < 3:
            continue
        # 用每行自己的航向/姿态反算 (与节点一致), dev 相对"早冻结"的 est
        pts = []
        for r in seg:
            prj = rp.px_to_level(r['ex'], r['ey'], r['pit'], r['rol'])
            if prj is None:
                continue
            h = math.radians(r['hdg'])
            cf, cr = prj[0] * r['H'], prj[1] * r['H']
            dx, dy = rp.b2n(h, cf, cr)
            est = (r['ned'][0] - home[0] + dx, r['ned'][1] - home[1] + dy)
            f_, rr = rp.to_fr(h, est[0] - fr['est'][0], est[1] - fr['est'][1])
            pts.append({'tau': r['epoch'] - dr['epoch'], 't': r['epoch'] - fr['epoch'],
                        'dev': (f_ * 100, rr * 100), 'age': r['age'], 'H': r['H']})
        out.append({'tag': tag, 'drop': dr['n'], 'bottle': fr['bottle'], 'pts': pts,
                    'truth': rp.TRUTH.get((tag, dr['n']), (None, None))})
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()
    recs = []
    for n in rp.LOGS:
        p = rp.LOG_DIR / n
        if p.exists():
            recs += series(p, p.stem.split('_')[-1])
    print('=' * 112)
    print(f"{'趟':8s}{'投':^3s}{'瓶':4s}{'冻结→释放':>10s}{'dev@释放':>11s}{'漂移率':>9s}"
          f"{'释放后再看+1.5s':>16s}{'预测(左,前)':>18s}{'实测(左,前)':>16s}")
    for r in recs:
        pts = r['pts']
        if not pts:
            continue
        pre = [p for p in pts if p['tau'] <= 0.2]
        post = [p for p in pts if 0.2 < p['tau'] <= 0.2 + GAP + 0.8]
        if not pre:
            continue
        rel = pre[-1]
        dev_rel = math.hypot(*rel['dev'])
        gap_t = rel['t']
        rate = dev_rel / gap_t if gap_t > 0.1 else float('nan')
        if post:
            after = post[-1]
            extra = (after['dev'][0] - rel['dev'][0], after['dev'][1] - rel['dev'][1])
            extra_txt = f"({extra[0]:+5.1f},{extra[1]:+5.1f})@{after['tau']:.1f}s"
        else:
            extra = (0.0, 0.0)
            extra_txt = '   --(无后释放样本)'
        # 预测: L_new = L_实测 + dev@释放(=dev_rel) − Δcomp − extra   (逐轴, (前,右) 分量)
        dcomp = DCOMP.get((r['tag'], r['bottle']), 0.0) * 100.0      # cm, 右向
        pv = {}
        for nm, idx, sgn in (('左', 1, -1.0), ('前', 0, +1.0)):
            tv = r['truth'][idx]
            if tv is None:
                pv[nm] = (None, None); continue
            t_pr = sgn * tv
            d_comp = dcomp if idx == 1 else 0.0
            new_pr = t_pr + rel['dev'][idx] - d_comp - extra[idx]
            pv[nm] = (tv, sgn * new_pr)
        f = lambda x: '  --  ' if x is None else f'{x:+5.1f}'
        print(f"{r['tag']:8s}{r['drop']:^3d}{r['bottle']:4s}{gap_t:9.1f}s{dev_rel:10.1f}cm"
              f"{rate*100:8.1f}c/s{extra_txt:>16s}"
              f"   ({f(pv['左'][1])},{f(pv['前'][1])})   ({f(r['truth'][0])},{f(r['truth'][1])})")
        if a.verbose:
            print('      dev 序列 (tau s: 前/右 cm): ' +
                  '  '.join(f"{p['tau']:+.1f}:{p['dev'][0]:+.0f}/{p['dev'][1]:+.0f}" for p in pts))
    print('=' * 112)
    tot = old_in = new_in = 0
    for r in recs:
        t = r['truth']
        if all(v is None for v in t):
            continue
        pts = [p for p in r['pts'] if p['tau'] <= 0.2]
        if not pts:
            continue
        rel = pts[-1]
        post = [p for p in r['pts'] if 0.2 < p['tau'] <= 0.2 + GAP + 0.8]
        extra = ((post[-1]['dev'][0] - rel['dev'][0], post[-1]['dev'][1] - rel['dev'][1])
                 if post else (0.0, 0.0))
        dcomp = DCOMP.get((r['tag'], r['bottle']), 0.0) * 100.0
        vals_o, vals_n = [], []
        for nm, idx, sgn in (('左', 1, -1.0), ('前', 0, +1.0)):
            tv = t[idx]
            if tv is None:
                continue
            vals_o.append(abs(tv))
            vals_n.append(abs(sgn * (sgn * tv + rel['dev'][idx] - (dcomp if idx == 1 else 0.0) - extra[idx])))
        if not vals_o:
            continue
        tot += 1
        eo, en = math.hypot(*vals_o), math.hypot(*vals_n)
        old_in += eo <= 12.5
        new_in += en <= 12.5
        print(f"  {r['tag']} #{r['drop']}: 实测 {eo:5.1f}cm {'进' if eo<=12.5 else '出'}"
              f"  →  预测 {en:5.1f}cm {'进' if en<=12.5 else '出'}")
    print(f"\n进桶: 现状 {old_in}/{tot} → 改造后(晚冻结+1.5s内投+补偿重标) {new_in}/{tot}"
          f"   [桶口半径 12.5cm; 未量轴按 0 计, 两栏同口径]")


if __name__ == '__main__':
    main()
