#!/usr/bin/env python3
# ================================================================
# 2026-09-23 e 改动后 SITL 复验分析 (①两轴瞄准补偿 + ②米制释放判据)
#   输入: /tmp/e_output.log (任务日志) + /tmp/e_judge.log (独立裁判)
#   输出: 每次投放的 触发方式 / 释放时残差 / 机体系分解 —— 用**独立 GT(裁判)** 校验
#         "相机测的偏移" 与 "真值偏移" 是否一致, 以及补偿方向/量级是否落地
#   用法: python3 scripts/analyze_e_sitl_20260923_aim_comp.py
# ================================================================
import math
import re
import sys

FX, FY = 349.22, 353.27
OUT, JUDGE = '/tmp/e_output.log', '/tmp/e_judge.log'
XF_C, Y_C = 0.040, 0.048          # 本次改动写入的补偿 (前 / 右, m)
DROP = re.compile(r'>>> DROP #(?P<n>\d+)!.*?NED\((?P<x>[-\d.]+),(?P<y>[-\d.]+),(?P<z>[-\d.]+)\) '
                  r'heading=(?P<h>[-\d.]+)deg aim=NED\((?P<ax>[-\d.]+),(?P<ay>[-\d.]+)\) '
                  r'vehicle_to_anchor=(?P<va>[\d.]+)m pitch=(?P<p>[-\d.]+).*?'
                  r'触发方式=(?P<way>[^ ]+).*?last_err=(?P<ep>\d+)px\(ex=(?P<ex>-?\d+),ey=(?P<ey>-?\d+)@H=(?P<hh>[\d.]+)m\)')
IMP = re.compile(r'Impact #(?P<n>\d+): NED\((?P<x>[-\d.]+),(?P<y>[-\d.]+)\)')
JUDGE_R = re.compile(r'RESULT (?P<tag>\w+) =+\s*\n\s*距离: (?P<d>[\d.]+)m, 得分: (?P<s>\d+)')
BUCKET = re.compile(r'Bucket(?P<i>\d) GZ\([-\d.]+,[-\d.]+,[-\d.]+\) → NED\((?P<x>[-\d.]+),(?P<y>[-\d.]+),')


def body(dx, dy, hdg_deg):
    """NED 位移 → 机体 (前, 右)"""
    h = math.radians(hdg_deg)
    return dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)


def main():
    txt = open(OUT, encoding='utf-8', errors='replace').read()
    jtxt = open(JUDGE, encoding='utf-8', errors='replace').read()
    drops = [m.groupdict() for m in DROP.finditer(txt)]
    imps = [m.groupdict() for m in IMP.finditer(jtxt)]
    buckets = {m.group('i'): (float(m.group('x')), float(m.group('y'))) for m in BUCKET.finditer(jtxt)}
    tags = re.findall(r'RESULT (\w+)', jtxt)
    ds = re.findall(r'距离: ([\d.]+)m, 得分: (\d+)', jtxt)
    scores = [(t, float(d), int(s)) for t, (d, s) in zip(tags, ds)]

    print(f'投放 {len(drops)} 次, 裁判落点 {len(imps)} 次, 桶 {len(buckets)} 个')
    if not drops:
        return print('未找到投放记录') or 1
    total = 0
    for d, im in zip(drops, imps):
        x, y, hh = float(d['x']), float(d['y']), float(d['hh'])
        ex, ey = int(d['ex']), int(d['ey'])
        rg = ex * hh / FX          # 残留 右 (已含补偿)
        fg = ey * hh / FY          # 残留 前
        conv = math.hypot(rg, fg)
        raw_r, raw_f = rg + Y_C, fg + XF_C      # 相机原始: 桶相对飞机的 (右, 前)
        bx, by = buckets[im['n'] if im['n'] in buckets else '1']
        # 找最近桶 (裁判的桶编号与本投一致)
        tgt = min(buckets.items(), key=lambda kv: (float(im['x']) - kv[1][0]) ** 2 + (float(im['y']) - kv[1][1]) ** 2)
        bx, by = tgt[1]
        dx, dy = float(im['x']) - bx, float(im['y']) - by
        gt_f, gt_r = body(dx, dy, float(d['h']))
        print(f'\n=== DROP #{d["n"]}  heading={d["h"]}°  触发方式={d["way"]}')
        print(f'  释放残差(等效地面): conv={conv*100:.1f}cm   (门限 10cm)   last_err={d["ep"]}px')
        print(f'  相机原始偏移(桶-飞机): 前 {raw_f*100:+.1f} 右 {raw_r*100:+.1f} cm   '
              f'⇒ 飞机相对桶: 前 {-raw_f*100:+.1f} 右 {-raw_r*100:+.1f} cm')
        print(f'  裁判 GT  (飞机-桶)   : 前 {gt_f*100:+.1f} 右 {gt_r*100:+.1f} cm   '
              f'(距桶心 {math.hypot(dx,dy)*100:.1f}cm)')
        print(f'  相机 vs GT 差        : 前 {(gt_f+raw_f)*100:+.1f} 右 {(gt_r+raw_r)*100:+.1f} cm')
        # 理想落点 = 飞机位置 (SITL 无机械偏置; 裁判也按飞机地面投影算)
        # 设计意图: 飞机应停在 aim = 桶 + (前 -XF_C, 右 -Y_C) ⇒ 期望 GT 偏移 = (前 -Xf, 右 -Yc)
        print(f'  设计期望(飞机-桶)    : 前 {-XF_C*100:+.1f} 右 {-Y_C*100:+.1f} cm  '
              f'⇒ 与 GT 差 前 {(gt_f+XF_C)*100:+.1f} 右 {(gt_r+Y_C)*100:+.1f} cm '
              f'(= 释放瞬间残差向量, 应≈ conv 的分解)')
    print('\n=== 裁判打分 ===')
    for tag, dist, s in scores:
        print(f'  {tag}: 距离 {dist*100:.2f}cm → {s} 分')
        total += s
    print(f'  合计 {total}/{10*len(scores)} 分')
    print('\n注: SITL 的"落点" = 释放瞬间飞机地面投影 (裁判取 /drop/impact_point), 且仿真里')
    print('    没有真实的机械/挂载偏置 ⇒ 6.25cm 补偿会让飞机**故意**停在桶的左后方,')
    print('    SITL 分数因此含 6.25cm 的固定"罚分", 这是设计使然 (实机上正是这 6.25cm 抵消系统偏置).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
