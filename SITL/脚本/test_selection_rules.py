#!/usr/bin/env python3
# ================================================================
# 2026-09-20 选桶健壮性规则验证 (离线, 用真实飞行日志里的框尺寸)
#   规则① 残缺框剔除: 最小边 < max(8px, 本帧最小边中位数×0.5) 的框丢掉
#   规则② 共同可见: 两个被选目标的支撑框中心像素相距须 ≥25px (否则顺延/继续搜索)
# 用法: python3 scripts/test_selection_rules.py [日志路径]
# ================================================================
import math
import re
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e_output_diag.log'

# --- 从真实日志里收集"每帧的框尺寸" (TRK-DBG 行) ---
frames = []
for ln in open(LOG, encoding='utf-8', errors='ignore'):
    m = re.search(r'TRK-DBG.*nbox=(\d+) boxes=\[(.*?)\]', ln)
    if not m or int(m.group(1)) == 0:
        continue
    frames.append([int(w) for w in re.findall(r'w(\d+)', m.group(2))])

# TRK-DBG 只打印了宽 w; h 从 TRK-NEW 行补 (那里有 w,h)
newboxes = [tuple(int(x) for x in m.groups())
            for m in re.finditer(r'from box\(cx=\d+,cy=\d+,w=(\d+),h=(\d+)\)',
                                 open(LOG, encoding='utf-8', errors='ignore').read())]
allsides = sorted(set([min(w, h) for w, h in newboxes] + [w for f in frames for w in f]))
print('日志:', LOG)
print('新追踪器的来源框 (w,h) 共 %d 个:' % len(newboxes))
sliver = [(w, h) for w, h in newboxes if min(w, h) < 8]
print('  其中"最小边<8px"的残缺框: %d 个 → %s' % (len(sliver), sliver))

# --- 规则①: 用日志里出现过的尺寸构造几帧, 看剔除是否符合预期 ---
CASES = [
    ('正常一帧 (3 个完整框)', [(18, 18), (18, 17), (17, 18)]),
    ('含右侧残框 (真实出现过)', [(18, 18), (17, 5), (5, 23)]),
    ('含上边残框 (真实出现过)', [(18, 18), (17, 4), (18, 16)]),
    ('只有 1 个框且很小 (远端单桶)', [(6, 6)]),
]
ok = True
for tag, boxes in CASES:
    sides = sorted(min(w, h) for w, h in boxes)
    thr = max(8.0, 0.5 * sides[len(sides) // 2]) if len(boxes) >= 2 else 8.0
    kept = [b for b in boxes if min(b) >= thr]
    drop = [b for b in boxes if min(b) < thr]
    good = (tag.startswith('正常') and len(kept) == 3) or \
           ('残框' in tag and len(drop) >= 1) or ('很小' in tag and len(kept) == 0)
    ok &= good
    print('  ① %-22s 阈值=%2.0fpx  保留=%s 剔除=%s  %s'
          % (tag, thr, kept, drop, '✔' if good else '✘'))

# --- 规则②: 共同可见 (支撑框中心像素相距 ≥25px) ---
def pick(trackers):
    picked = []
    for t in trackers:
        if all(math.hypot(t['c'][0] - p['c'][0], t['c'][1] - p['c'][1]) >= 25.0 for p in picked):
            picked.append(t)
        if len(picked) == 2:
            break
    return picked

SC = [
    ('两个真桶 (框中心相距 140px)', [{'n': 'B3', 'c': (216, 159)}, {'n': 'B2', 'c': (76, 235)}], 2),
    ('真桶 + 同一个框的幽灵 (相距 3px)', [{'n': 'B3', 'c': (216, 159)}, {'n': 'B3-ghost', 'c': (219, 161)}], 1),
    ('真桶 + 边缘残框幽灵 (相距 60px)', [{'n': 'B3', 'c': (216, 159)}, {'n': 'sliver', 'c': (276, 159)}], 2),
]
for tag, trk, want in SC:
    got = pick(trk)
    good = len(got) == want
    ok &= good
    print('  ② %-32s → 选中 %d 个 %s  (%s)'
          % (tag, len(got), [g['n'] for g in got], '✔' if good else '✘ 期望 %d' % want))
print()
print('结论:', '全部通过' if ok else '存在失败')
sys.exit(0 if ok else 1)
