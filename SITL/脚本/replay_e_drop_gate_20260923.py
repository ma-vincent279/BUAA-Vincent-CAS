#!/usr/bin/env python3
# ================================================================
# 2026-09-23 e 首次实飞: 回放 25s DROP 段的 1Hz 误差轨迹, 为"释放判据改米制"选门限
#   数据: e_mission_20260923_175741.log 的 DROP-SERVO 行 (44 条, 两段, 中间 17.9s 间隔)
#   用法: python3 scripts/replay_e_drop_gate_20260923.py
# ----------------------------------------------------------------
# 两个变体:
#   raw  : 现行 (无补偿) 的等效地面误差 |(ex·H/FX, ey·H/FY)|
#   comp : 加上本次要落的 (前 +4.0cm, 右 +4.8cm) 机体补偿后的误差 —— 也就是"预测落点
#          与桶心的距离"的代理量 (补偿生效后伺服会把该量驱动到 0)
# 说明: 日志 1Hz 节流, 真实控制环 10~20Hz ⇒ 本回放**低估**达标次数 (样本间也会低于门限).
# ================================================================
import math
import os
import re
import sys

FX, FY = 349.22, 353.27
LOG = '/home/vincent/SITL/e_mission_20260923_175741.log'
XF_C, Y_C = 0.040, 0.048   # 拟采用的补偿 (m): 前向 / 横向

LINE = re.compile(
    r'\[(?P<ts>\d+\.\d+)\].*DROP-SERVO err=(?P<err>-?\d+)px ex=(?P<ex>-?\d+) ey=(?P<ey>-?\d+)'
    r'.*H=(?P<h>[\d.]+)m')
RELEASE = re.compile(r'\[(?P<ts>\d+\.\d+)\].*>>> DROP #(?P<n>\d+)!.*heading=(?P<hdg>[-\d.]+)deg')


def parse():
    s = []
    rel = []
    for ln in open(LOG, encoding='utf-8'):
        m = LINE.search(ln)
        if m:
            s.append((float(m.group('ts')), int(m.group('ex')), int(m.group('ey')), float(m.group('h'))))
        m = RELEASE.search(ln)
        if m:
            rel.append((float(m.group('ts')), int(m.group('n')), float(m.group('hdg'))))
    # 按时间间隔 > 5s 切段
    segs, cur = [], []
    for r in s:
        if cur and r[0] - cur[-1][0] > 5.0:
            segs.append(cur); cur = []
        cur.append(r)
    if cur:
        segs.append(cur)
    return segs, rel


def replay(samples, thr_m, dwell_s, xf=0.0, y=0.0):
    """返回 (是否触发, 触发相对段首秒数, 触发时误差cm, 该次连续时长s)"""
    t0 = samples[0][0]
    run_start = None
    for ts, ex, ey, h in samples:
        rg = ex * h / FX - y
        fg = ey * h / FY - xf
        e = math.hypot(rg, fg)
        if e < thr_m:
            if run_start is None:
                run_start = ts
            if ts - run_start >= dwell_s - 1e-9:
                return True, ts - t0, e * 100, ts - run_start
        else:
            run_start = None
    return False, None, None, None


def fires_within(samples, thr_m, dwell_s, xf=0.0, y=0.0):
    """返回最长连续达标时长 (s) 与首次达标时刻, 不要求达到 dwell"""
    t0 = samples[0][0]
    best = 0.0
    run_start, first = None, None
    for ts, ex, ey, h in samples:
        rg = ex * h / FX - y
        fg = ey * h / FY - xf
        e = math.hypot(rg, fg)
        if e < thr_m:
            if run_start is None:
                run_start, first = ts, first if first is not None else ts
            best = max(best, ts - run_start)
        else:
            run_start = None
    return best, first


def main():
    segs, rel = parse()
    print(f'DROP-SERVO 样本 {sum(len(s) for s in segs)} 条, {len(segs)} 段')
    for i, s in enumerate(segs, 1):
        errs = sorted(math.hypot(ex * h / FX, ey * h / FY) * 100 for _, ex, ey, h in s)
        print(f'\n=== 第 {i} 投 DROP 段: {len(s)} 样本, 时长 {s[-1][0]-s[0][0]:.1f}s, '
              f'误差 min {errs[0]:.1f} / p25 {errs[len(errs)//4]:.1f} / 中位 {errs[len(errs)//2]:.1f} / max {errs[-1]:.1f} cm')
        print('   最低 5 个: ' + ', '.join(f'{v:.1f}' for v in errs[:5]))
    relt = {n: ts for ts, n, _ in rel}
    hdg = {n: h for _, n, h in rel}
    print('\n释放时刻: ' + ', '.join(f'#{n} t={ts:.3f} heading={h:.1f}°' for n, (ts, h) in
                                     ((n, (relt[n], hdg[n])) for n in sorted(relt))))

    print('\n' + '=' * 96)
    print('门限回放 (每段独立; "达标"= 该段 25s 内出现连续 <门限 且≥确认时间)')
    print('=' * 96)
    for tag, (xf, y) in (('raw  (无补偿)', (0.0, 0.0)), (f'comp (前+{XF_C*100:.0f}/右+{Y_C*100:.0f}cm)', (XF_C, Y_C))):
        print(f'\n--- {tag} ---')
        print(f'{"门限":>6} {"确认":>5} | ' + ' | '.join(f'第{i}投' for i in range(1, len(segs) + 1)) + ' | 结论')
        for thr_cm in (3, 4, 5, 6, 8, 10, 12, 15):
            for dwell in (0.5, 1.0, 1.5):
                cells, ok_all = [], True
                for s in segs:
                    ok, dt, e, run = replay(s, thr_cm / 100.0, dwell, xf, y)
                    if ok:
                        cells.append(f'{dt:5.1f}s 时 e={e:4.1f}cm 连续{run:4.1f}s')
                    else:
                        best, first = fires_within(s, thr_cm / 100.0, dwell, xf, y)
                        cells.append(f'不触发(最长连续{best:4.1f}s)')
                        ok_all = False
                verdict = '✓ 两投都触发' if ok_all else ('部分/不触发')
                print(f'{thr_cm:5.0f}cm {dwell:4.1f}s | ' + ' | '.join(cells) + f' | {verdict}')

    print('\n注: 1Hz 节流 ⇒ 真实 10~20Hz 控制环达标更容易, 上表是保守下界喵~')


if __name__ == '__main__':
    sys.exit(main())
