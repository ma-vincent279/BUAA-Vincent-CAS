#!/usr/bin/env python3
# ================================================================
# 2026-09-24 项目 f 离线验收: 用 8 份历史实机飞行日志回放"两级释放判据"
#   - 解析每份日志 DROP 段 1Hz 的 DROP-SERVO 行 -> 等效地面误差 conv(cm) [+ vxy]
#   - 按 f 的判据 (Tier1 5cm/低谷/准静止 + Tier2 10cm 保底 + 35s 兜底) 逐步回放
#   - 对照该次实际释放 (触发方式 + last_err) 给出"若用 f 会何时投、投在多少 cm"
#
#   ⚠ 近似说明: 日志里的 DROP-SERVO 行是 1Hz, 而判据在 4Hz 上跑 =>
#     回放里"1 个样本"即代表该秒内条件成立; 故同时给出
#     [乐观] 1 样本即确认 与 [保守] 需连续 2 样本 两种口径.
#   ⚠ 序列在"实际释放"处结束 => 9-24 两次只能判断"会不会在更早或同一时刻触发".
#
#   用法: python3 scripts/replay_f_drop_gate_20260924.py
# ================================================================
import math
import os
import re

ROOT = '/home/vincent/SITL'
FX, FY = 349.0, 353.0        # 实机相机内参 (2026-08-06 统一)

# 与 f.py 逐字一致的判据常量
DROP_ARRIVE_M, DROP_ARRIVE_M_LATE = 0.05, 0.10
DROP_CONVERGE_EARLY, DROP_CONVERGE_LATE = 0.5, 1.0
DROP_CONV_MIN_BAND, DROP_CONV_MIN_FLOOR = 0.02, 0.02
DROP_ARRIVE_VMAX, DROP_PREFER_WINDOW = 0.12, 10.0

LOGS = [
    ('9-11  d', 'd_mission_20260911_170203.log'),
    ('9-13a d', 'd_mission_20260913_145458.log'),
    ('9-13b d', 'd_mission_20260913_150944.log'),
    ('9-15  d', 'd_mission_20260915_133901.log'),
    ('9-18  d', 'd_mission_20260918_175509.log'),
    ('9-19  d', 'd_mission_20260919_180109.log'),
    ('9-23  e', 'e_mission_20260923_175741.log'),
    ('9-24  e', '9.24/e_mission_20260924_173608.log'),
]

SERVO_RE = re.compile(r'DROP-SERVO err=(\d+)px ex=(-?\d+) ey=(-?\d+).*?H=([\d.]+)m(.*)$')
DROP_RE = re.compile(r'>>> DROP #(\d+)!.*?(?:触发方式=(\S+))?(.*)$')
VXY_RE = re.compile(r'vxy=([\d.]+)m/s')


def parse(path):
    """返回 [(conv_cm, vxy|None), ...] 的投放段列表."""
    segs, cur = [], []
    for line in open(path, encoding='utf-8', errors='replace'):
        m = SERVO_RE.search(line)
        if m:
            ex, ey, H = int(m.group(2)), int(m.group(3)), float(m.group(4))
            conv = math.hypot(ex * H / FX, ey * H / FY) * 100.0
            v = VXY_RE.search(m.group(5))
            cur.append((conv, float(v.group(1)) if v else None))
        if '>>> DROP' in line:
            segs.append(cur)
            cur = []
    return [s for s in segs if s]


def replay(samples, strict):
    """按 f 判据回放; 返回 (tier, 时刻s, conv_cm, 段内最小cm) 或 None."""
    need_samples = 2 if strict else 1
    cmin, tier, run, run_tier = None, None, 0, None
    for i, (conv_cm, vxy) in enumerate(samples):
        conv = conv_cm / 100.0
        t = float(i)
        cmin = conv if cmin is None else max(DROP_CONV_MIN_FLOOR, min(cmin, conv))
        in_valley = conv <= cmin + DROP_CONV_MIN_BAND
        stay_still = (vxy is None) or (vxy < DROP_ARRIVE_VMAX)
        new_tier = None
        if conv < DROP_ARRIVE_M and in_valley and stay_still:
            new_tier = 1
        elif t >= DROP_PREFER_WINDOW and conv < DROP_ARRIVE_M_LATE:
            new_tier = 2
        if new_tier == run_tier and new_tier is not None:
            run += 1
        else:
            run_tier, run = new_tier, (1 if new_tier is not None else 0)
        tier = new_tier
        if tier is not None and run >= need_samples:
            return tier, t, conv_cm, cmin * 100
    return None


def actual_of(path):
    """提取每次投放的实际结果: (触发方式, 残差cm, vxy)."""
    out = []
    for line in open(path, encoding='utf-8', errors='replace'):
        if '>>> DROP' not in line:
            continue
        w = re.search(r'触发方式=(\S+)', line)
        le = re.search(r'last_err=(\d+)px\(ex=(-?\d+),ey=(-?\d+)@H=([\d.]+)m', line)
        v = VXY_RE.search(line)
        conv = (math.hypot(int(le.group(2)) * float(le.group(4)) / FX,
                           int(le.group(3)) * float(le.group(4)) / FY) * 100) if le else None
        out.append((w.group(1) if w else '?', conv, float(v.group(1)) if v else None))
    return out


def main():
    print('=' * 100)
    print('项目 f 两级释放判据 — 8 份历史实机日志回放 (1Hz 样本近似)')
    print('  Tier1: conv<5cm + 低谷(<=本段最小+2cm) + 准静止(vxy<0.12)   期望确认 0.5s')
    print('  Tier2: conv<10cm (=e 旧判据), 但须等过 DROP_PREFER_WINDOW=10s   确认 1.0s')
    print('=' * 100)
    tot = {'t1': 0, 't2': 0, 'none': 0, 'n': 0}
    conv_new, conv_act = [], []
    for tag, rel in LOGS:
        path = os.path.join(ROOT, rel)
        if not os.path.exists(path):
            print(f'\n### {tag}: 缺日志 {rel}')
            continue
        segs, acts = parse(path), actual_of(path)
        print(f'\n### {tag}  ({rel})  投放段 {len(segs)} 个')
        for i, s in enumerate(segs):
            a = acts[i] if i < len(acts) else ('?', None, None)
            r1, r2 = replay(s, False), replay(s, True)
            tot['n'] += 1
            act_s = f'{a[0]} 残差={a[1]:.1f}cm' if a[1] is not None else f'{a[0]} 残差=?'
            if a[1] is not None:
                conv_act.append(a[1])
            line = f'  第{i+1}投: 段内最低={min(c for c, _ in s):5.1f}cm (样本{len(s):2d}) | 实际: {act_s:28s} | f 判据: '
            if r1 is None:
                line += '段内未触发 -> 走 35s 兜底'
                tot['none'] += 1
            else:
                tier, t, conv, cmin = r1
                tot['t1' if tier == 1 else 't2'] += 1
                conv_new.append(conv)
                line += f'Tier{tier} @{t:.0f}s 残差={conv:5.2f}cm (段内最小={cmin:4.1f}cm)'
                if r2 is None:
                    line += '  [保守口径: 段内未触发]'
                elif (r2[0], r2[1]) != (tier, t):
                    line += f'  [保守口径: Tier{r2[0]} @{r2[1]:.0f}s]'
            print(line)
    print('\n' + '=' * 100)
    print(f'汇总: 共 {tot["n"]} 投 | Tier1 优选触发 {tot["t1"]} | Tier2 保底触发 {tot["t2"]} | 段内未触发(兜底) {tot["none"]}')
    if conv_new and conv_act:
        n = min(len(conv_new), len(conv_act))
        print(f'      释放残差: 实际中位 {sorted(conv_act)[len(conv_act)//2]:.1f}cm'
              f' -> f 判据中位 {sorted(conv_new)[len(conv_new)//2]:.1f}cm  (可比 {n} 投)')
    print('=' * 100)


if __name__ == '__main__':
    main()
