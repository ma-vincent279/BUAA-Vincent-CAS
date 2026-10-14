#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低空"实时视觉闭环"振荡复盘 (2026-10-08) —— 机理重新量化

背景: 2026-10-06 那趟 f (14:49) 两投 DROP 段都拖到 36.1s 兜底, 当时的结论是
      "姿态→瞄准点→飞机→姿态 的低频正反馈". 本脚本把该趟 DROP 段的**三路日志**
      (1Hz 状态行 / DROP-SERVO 2Hz / TRK-DBG 检测框) 逐点对齐后重新量化, 结论修正为:
      主因 = ①设定点每帧重建且含飞机自身位置 ②检测框贴边被 see.py 裁剪(只裁不剔)导致
      估计跳变 ③外环有效增益过小(0.105 1/s ⇒ 100px 只给 3.6cm/s) ⇒ 门限不可达 ⇒ 兜底;
      姿态项 (2.1cm/°) 只是放大器, 不是主因.

用法: python3 scripts/analyze_f_oscillation_20261008.py
输出: A 各趟 DROP 段时长 / B 14:49 第1投逐点 / C 派生量(增益·误差·检测稳定性) / D i 对照 / E 结论
"""
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FX, FY = 349.0, 353.0          # i/f 的相机内参 (飞控日志按假定值)
IMG_W, CX, CY = 640, 320.0, 180.0
KP_H, CRUISE_HEIGHT = 0.0015, 5.0   # f.py: KP_H / CRUISE_HEIGHT (height_scale = H/CRUISE)

RE_T = re.compile(
    r'\[t=\s*([\d.]+) DROP\s+ph=\s*([\d.]+)s\] NED\(\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)\) '
    r'H=\s*([\d.]+) hdg=\s*([\d.]+) pit=\s*([-+][\d.]+) rol=\s*([-+][\d.]+) '
    r'vNED=\(([-+][\d.]+),([-+][\d.]+),([-+][\d.]+)\) det=(\d) age=([\d.]+) B(\d)/2 '
    r'tgt=NED\(\s*(-?[\d.]+),\s*(-?[\d.]+)\) px=\((-?\d+),(-?\d+)\)err=(\d+)px')
RE_S = re.compile(
    r'DROP-SERVO err=(\d+)px ex=(-?\d+) ey=(-?\d+) v_fwd=(-?[\d.]+) v_lat=(-?[\d.]+)'
    r'\(bias-input\) H=([\d.]+)m .*?target=\((-?[\d.]+),(-?[\d.]+)\) '
    r'px_raw=\((-?\d+),(-?\d+),err=(\d+)px\).*?dist=([\d.]+)m? bias=\(([-+][\d.]+),([-+][\d.]+)\) '
    r'pitch=([-+][\d.]+) roll=([-+][\d.]+) vxy=([\d.]+)m/s')
RE_B = re.compile(r'nbox=(\d+) boxes=\[\((\d+),(\d+)\)w(\d+)→NED\((-?[\d.]+),(-?[\d.]+)\)\] '
                  r'trackers=(\d+)')


def seg_lines(lines, nth=1):
    """取第 nth 个 DROP 段的日志行"""
    out, seen, on = [], 0, False
    for ln in lines:
        if 'VISUAL_SERVO->DROP' in ln:
            seen += 1
            if seen > nth:
                break
            on = True
            continue
        if on:
            if 'DROP->CLIMB' in ln or 'DROP->' in ln and 'CLIMB' in ln:
                break
            out.append(ln)
    return out


def section_a():
    print('=== A. 各趟实飞的 DROP 段时长 + 释放方式 (按各族自己的兜底窗口判定) ===')
    print(f"{'日志':<44} {'DROP 时长(s)':<20} {'触发方式':<34} {'判定'}")
    # 各族兜底窗口: d/e=25s, f/f_r3=35s, i=20s(10-08 前)/45s(k)
    def thr_of(name):
        if name.startswith('i_'):   return 20.0
        if name.startswith('f_r3'): return 35.0
        if name.startswith('f_'):   return 35.0
        if name.startswith('e_'):   return 25.0
        if name.startswith('d_'):   return 25.0
        return 35.0
    files = []
    for d in ['9.24', '9.27', '10.3', '10.6', '10.7', '.']:
        if (ROOT / d).is_dir():
            files += sorted((ROOT / d).glob('*mission*.log'))
    for f in files:
        t = f.read_text(encoding='utf-8', errors='ignore')
        durs = [float(x) for x in re.findall(r'DROP->CLIMB \(prev ([\d.]+)s\)', t)]
        if not durs:
            continue
        trig = re.findall(r'触发方式=([^\s]+)', t)
        trig = [x[:10] for x in trig] or ['-']
        thr = thr_of(f.name)
        nbad = sum(1 for d in durs if d >= thr - 1.5)
        verdict = f'兜底 {nbad}/{len(durs)}' if nbad else '全部收敛 ✓'
        print(f"{f.relative_to(ROOT).as_posix():<44} {str(durs):<20} {','.join(trig):<34} {verdict}")
    print('  ⇒ i 的"冻结+定点保持"实机全部 7~12s 收敛; d/e 的 26s 和 f 系的 36s 都是兜底(未达门限)')
    print('     ⚠ f 系(每帧重建瞄准点)在 10-06 出现 36.1s 兜底 ⇒ 闭环没收敛, 见 B/C 节的机理\n')


def load(fname, nth=1):
    lines = (ROOT / fname).read_text(encoding='utf-8', errors='ignore').split('\n')
    seg = seg_lines(lines, nth)
    rows, servos, boxes = [], [], []
    for ln in seg:
        if (m := RE_T.search(ln)):
            g = m.groups()
            rows.append(dict(t=float(g[0]), x=float(g[2]), y=float(g[3]), H=float(g[5]),
                             hdg=float(g[6]), pit=float(g[7]), rol=float(g[8]),
                             vx=float(g[9]), vy=float(g[10]), tx=float(g[15]), ty=float(g[16]),
                             px=int(g[17]), py=int(g[18]), err=int(g[19])))
        if (m := RE_S.search(ln)):
            g = m.groups()
            servos.append(dict(err=int(g[0]), ex=int(g[1]), ey=int(g[2]), vf=float(g[3]),
                               vl=float(g[4]), H=float(g[5]), tx=float(g[6]), ty=float(g[7]),
                               rpx=int(g[8]), rpy=int(g[9]), rerr=int(g[10]), dist=float(g[11]),
                               pit=float(g[14]), rol=float(g[15]), vxy=float(g[16])))
        if (m := RE_B.search(ln)):
            g = m.groups()
            boxes.append(dict(nbox=int(g[0]), cx=int(g[1]) + int(g[3]) // 2, w=int(g[3]),
                              ex=float(g[4]), ey=float(g[5]), ntrk=int(g[6])))
    return rows, servos, boxes


def section_bc(fname='10.6/f_mission_20261006_144934.log'):
    rows, servos, boxes = load(fname, 1)
    if not rows:
        print('!! 解析失败:', fname)
        return
    h = math.radians(rows[0]['hdg'])
    cf, sf = math.cos(h), math.sin(h)
    fwd = lambda dx, dy: dx * cf + dy * sf
    lft = lambda dx, dy: dx * sf - dy * cf
    print(f'=== B. {fname} 第 1 投逐点 (DROP 段 {rows[-1]["t"]-rows[0]["t"]:.0f}s) ===')
    print(f"{'t(s)':>5} {'飞机-锚点:前':>12} {'左':>8} {'距(cm)':>7} {'H':>5} {'pit°':>6} {'rol°':>6}"
          f" {'px':>5} {'py':>5} {'等效地面cm':>10}")
    for r in rows:
        d = math.hypot(r['x'] - r['tx'], r['y'] - r['ty'])
        print(f"{r['t']:5.0f} {fwd(r['x']-r['tx'], r['y']-r['ty'])*100:12.1f} "
              f"{lft(r['x']-r['tx'], r['y']-r['ty'])*100:8.1f} {d*100:7.0f} {r['H']:5.2f} "
              f"{r['pit']:6.1f} {r['rol']:6.1f} {r['px']:5d} {r['py']:5d} "
              f"{r['err']*r['H']/FX*100:10.0f}")
    print('\n=== C. 派生量 (为什么"进不了门限") ===')
    dists = [math.hypot(r['x'] - r['tx'], r['y'] - r['ty']) for r in rows]
    eq = [s['err'] * s['H'] / FX for s in servos]
    pid = [math.hypot(s['vf'], s['vl']) for s in servos]
    vxy = [s['vxy'] for s in servos]
    print(f"  飞机↔锚点距离: {min(dists)*100:.0f}~{max(dists)*100:.0f}cm "
          f"(均值 {sum(dists)/len(dists)*100:.0f}cm) ⇒ 整段没接近过")
    print(f"  像素误差 err: {min(s['err'] for s in servos)}~{max(s['err'] for s in servos)}px "
          f"⇒ 等效地面 {min(eq)*100:.0f}~{max(eq)*100:.0f}cm (门限 Tier1=5cm / Tier2=10cm)")
    print(f"  PID 输出 |v|: {min(pid)*100:.1f}~{max(pid)*100:.1f}cm/s  ← 外环几乎不发力")
    print(f"  实际 vxy  : {min(vxy):.3f}~{max(vxy):.3f} m/s (中位 {sorted(vxy)[len(vxy)//2]:.3f})")
    print(f"    ⇒ 实际运动 >> PID 输出 ⇒ 飞机是在**追位置设定点**, 不是被像素环驱动")
    Hm = sum(r['H'] for r in rows) / len(rows)
    kp_eff = KP_H * (Hm / CRUISE_HEIGHT)
    K = KP_H * FX / CRUISE_HEIGHT
    print(f"  外环有效增益: KP_H={KP_H} × height_scale={Hm/CRUISE_HEIGHT:.2f} = {kp_eff:.6f} (m/s)/px")
    print(f"    ⇒ 100px 误差 = {kp_eff*100*100:.1f}cm/s;  环路增益 K=KP·FX/CRUISE={K:.3f} 1/s"
          f" (与高度无关, 时间常数 ≈{1/K:.0f}s) ⇒ 走 1m 需 {1/K:.0f}s+, 实测 36.1s 兜底对得上")
    att = [max(abs(r['pit']), abs(r['rol'])) * math.pi / 180 * r['H'] for r in rows]
    print(f"  姿态项 H·tan(θ): 最大 {max(att)*100:.1f}cm (pitch {min(r['pit'] for r in rows):+.1f}~"
          f"{max(r['pit'] for r in rows):+.1f}°, roll {min(r['rol'] for r in rows):+.1f}~"
          f"{max(r['rol'] for r in rows):+.1f}°) ⇒ **次级项**, 实测误差 {min(eq)*100:.0f}~{max(eq)*100:.0f}cm")
    if boxes:
        edge = sum(1 for b in boxes if b['cx'] + b['w'] / 2 >= IMG_W - 8 or b['cx'] - b['w'] / 2 <= 8)
        nj, jd = 0, []
        for a, b in zip(boxes, boxes[1:]):
            d = math.hypot(a['ex'] - b['ex'], a['ey'] - b['ey'])   # 框反算 NED 的样本间跳变
            jd.append(d)
            if d > 0.3:
                nj += 1
        jd.sort()
        print(f"  检测框: 宽 {min(b['w'] for b in boxes)}~{max(b['w'] for b in boxes)}px, "
              f"框心 x {min(b['cx'] for b in boxes)}~{max(b['cx'] for b in boxes)} (画面宽 {IMG_W}), "
              f"贴边(距边<8px) {edge}/{len(boxes)} 样本")
        print(f"    框反算 NED 的**样本间跳变**: >0.3m 的 {nj}/{len(boxes)-1} 次, "
              f"中位 {jd[len(jd)//2]*100:.0f}cm, 最大 {jd[-1]*100:.0f}cm  ← 估计在跳 = 误差信号被污染")
        print(f"    同一桶一次出现追踪器 ≥2 个的样本: {sum(1 for b in boxes if b['ntrk'] >= 2)}"
              f" (最多 {max(b['ntrk'] for b in boxes)} 个)")
        print(f"    ⇒ see.py 对越界框**只裁不剔** ⇒ 框中点由「裁掉多少」决定, 而裁剪量又取决于飞机站位")
        print(f"      ⇒ 这就是闭环里的正反馈源 (姿态项只是次级)")
    if len(servos) >= 3:
        sp = [math.hypot(b['tx'] - a['tx'], b['ty'] - a['ty']) for a, b in zip(servos, servos[1:])]
        sp.sort()
        print(f"  位置设定点(target) 样本间位移: 中位 {sp[len(sp)//2]*100:.0f}cm, 最大 {sp[-1]*100:.0f}cm"
              f" (DROP-SERVO 约 0.5~1s 一条 ⇒ 设定点速度量级 0.1~0.3 m/s, 与飞机实际 vxy 同量级)")
        print(f"    ⇒ 「设定点在动」与「飞机在动」同量级 ⇒ 不是飞机追不上一个静止点, 而是**设定点自己在走**")
    print()


def section_d():
    print('=== D. 对照: 同一相机/同一场地的 i (冻结+定点保持) ===')
    for f in sorted((ROOT / '10.7').glob('i_mission_20261007_16*.log')) + \
             sorted((ROOT / '10.7').glob('i_mission_20261007_18*.log')):
        t = f.read_text(encoding='utf-8', errors='ignore')
        durs = re.findall(r'DROP->CLIMB \(prev ([\d.]+)s\)', t)
        geo = re.findall(r'\[纯几何: 保持点-桶估计=\(([-+][\d.]+),([-+][\d.]+)\)\]', t)
        errs = re.findall(r'到位err=([\d.]+)cm', t)
        print(f"  {f.name}: DROP {durs}s  到位err={errs}cm  "
              f"纯几何模长={[round(math.hypot(float(a),float(b)),3) for a,b in geo]}")
    print('  ⇒ 冻结结构 (不让估计进环路) 在实机 7~12s 收敛; 差异不在相机, 在环路结构\n')


def main():
    print('=== 低空视觉闭环振荡复盘 (2026-10-08) ===')
    print(f'脚本: {Path(__file__).relative_to(ROOT)}   数据: 10.6/f_mission_20261006_144934.log 等\n')
    section_a()
    section_bc()
    section_d()
    print('=== E. 结论 ===')
    print('  1) 测量链没坏 (9-23 实飞 corr=0.90/0.95 ⇒ 反投影准; i 的 [纯几何] 自检逐位对)')
    print('  2) 坏在环路: 设定点 = 0.7·(飞机位置+反投影) + 0.3·锚点, 每帧重建 ⇒ 含自身状态 ⇒ 正反馈结构')
    print('  3) 该趟的主激励是"检测框贴边被裁剪 + 身份跳变"(12/34 样本 est 跳>0.3m), 不是姿态')
    print('  4) 外环有效增益 0.105 1/s (100px⇒3.6cm/s) ⇒ 1.5m 偏差 30s+ 走不完 ⇒ 36.1s 兜底 (必然)')
    print('  5) 姿态项 2.1cm/° (该趟 ≤7cm) 是放大器; 真正咬人的姿态场合是"冻结被拽走"(13:02 的 58cm)')
    print('  6) 修法 (按性价比): L1 脏估计不入环 / L2 改像素残差积分 / L3 α 软重锚 / L4 时间尺度分离 / L5 机械标定')
    return 0


if __name__ == '__main__':
    sys.exit(main())
