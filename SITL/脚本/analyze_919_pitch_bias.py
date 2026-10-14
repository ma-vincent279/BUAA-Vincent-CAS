#!/usr/bin/env python3
# ================================================================
# 2026-09-20 用 9-19 实飞日志验证"PID 俯仰补偿符号反了"的根因 (只读分析)
# ----------------------------------------------------------------
# 机理: d 的 `ey -= FY*tan(pitch)` 使 PID 报出假的像素偏差 -2·FY·tan(pitch),
#       DROP 段的"像素残差积分器"会把这个假偏差积掉 → 瞄准点被稳定推离桶
#       ≈ 2·H·tan(pitch) - 死区折算(5px·H/FY).
# 日志证据: DROP-SERVO 行带 pitch= 和 bias=(x,y) (NED); 把 bias 投影到机体前向
#           就能看出瞄准点被推到哪里; DROP #n 行给出该投的 heading.
# 用法: python3 scripts/analyze_919_pitch_bias.py [日志路径]
# ================================================================
import math
import re
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else '/home/vincent/SITL/d_mission_20260919_180109.log'
FY = 353.27
DEAD_PX = 5.0

pat = re.compile(
    r'DROP-SERVO err=(?P<err>\d+)px .*?H=(?P<h>[\d.]+)m .*?'
    r'bias=\((?P<bx>[+-][\d.]+),(?P<by>[+-][\d.]+)\) pitch=(?P<pitch>[+-][\d.]+)')
drop_pat = re.compile(
    r'>>> DROP #(?P<n>\d)! .*?heading=(?P<hd>[-\d.]+)deg.*?等效地面=(?P<conv>[\d.]+)cm')

# 按行序收集事件, 再把每条 DROP-SERVO 归属到"其后最近的一次投弹"的 heading
events = []
for ln in open(LOG, encoding='utf-8', errors='replace'):
    m = drop_pat.search(ln)
    if m:
        events.append(('drop', dict(n=int(m.group('n')), conv=float(m.group('conv')),
                                    hd=math.radians(float(m.group('hd'))))))
        continue
    m = pat.search(ln)
    if m:
        events.append(('servo', dict(err=int(m.group('err')), h=float(m.group('h')),
                                     bx=float(m.group('bx')), by=float(m.group('by')),
                                     pitch=float(m.group('pitch')))))

drops = [e[1] for e in events if e[0] == 'drop']
last_hd = drops[0]['hd'] if drops else 0.0
servo = []
for i, (kind, d) in enumerate(events):
    if kind != 'servo':
        continue
    idx = next((j for j in range(i + 1, len(events)) if events[j][0] == 'drop'), None)
    d['hd'] = events[idx][1]['hd'] if idx is not None else servo[-1]['hd'] if servo else last_hd
    d['drop'] = (events[idx][1]['n'] if idx is not None
                 else servo[-1]['drop'] if servo else 1)
    servo.append(d)

print('=' * 96)
print('9-19 实飞 DROP-SERVO 行分析: 积分器把瞄准点推离桶多远 (bias 投影到机体前向)')
print('  机体前向分量 fwd_bias = bx·cos(hd) + by·sin(hd)   正=向前, 负=向后(瞄准点落后于桶)')
print('  理论预测(符号反了时) = 2·H·tan(pitch) - 5px·H/FY')
print('=' * 96)
print('%-4s %-5s %-7s %-8s %-8s %-9s %-9s %-12s %-10s' %
      ('投', '#', 'H(m)', 'pitch°', 'err(px)', 'bias_x', 'bias_y', 'fwd_bias(m)', '理论(m)'))
print('-' * 96)
prev_drop = None
for i, r in enumerate(servo):
    if r['drop'] != prev_drop:
        prev_drop = r['drop']
        dd = next((x for x in drops if x['n'] == prev_drop), None)
        print('--- 以下各行 → 其后最近一次投弹 = DROP #%d (heading=%.1f°, 释放时等效地面=%.1fcm)'
              % (prev_drop, math.degrees(r['hd']), dd['conv'] if dd else float('nan')))
    fwd = r['bx'] * math.cos(r['hd']) + r['by'] * math.sin(r['hd'])
    theory = 2 * r['h'] * math.tan(abs(r['pitch'])) - DEAD_PX * r['h'] / FY
    print('%-4d %-5d %-7.1f %-8.2f %-8d %+-9.2f %+-9.2f %+-12.3f %+-10.3f' %
          (r['drop'], i + 1, r['h'], math.degrees(abs(r['pitch'])), r['err'],
           r['bx'], r['by'], fwd, theory))

print()
print('解读:')
print('  * fwd_bias 越负 = 瞄准点被推到飞机后方越多 = 落点越偏后(脱靶方向).')
print('  * 若"符号反了"是根因, |fwd_bias| 应随 pitch 增大而增大 (~2·H·tan(pitch)).')
print('  * 第 1 投 pitch 2.8~4.8° → 理论 0.10~0.20m; 第 2 投 pitch 1.1~1.5° → 理论 0.03~0.05m.')
