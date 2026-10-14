#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多组数据: DROP 段 1Hz 日志的 "相机残差 / bias 积分" 时间演化
目的: 验证"前几秒稳 -> bias 上发条 -> 残差越来越大(自激)"这一机制, 并给出停留时长依据。
数据: 实飞节点日志 9.24 / 9.27(两趟) / 10.3(两趟) = 10 次投放。
"""
import re, os, glob

LOGS = ['9.24/e_mission_20260924_173608.log', '9.27/e_mission_20260927_105447.log',
        '9.27/f_mission_20260927_110915.log', '10.3/f_mission_20261003_142704.log',
        '10.3/f_mission_20261003_163952.log']
ROOT = '/home/vincent/SITL'
TRE = re.compile(r'\[(\d+\.\d+)\].*DROP-SERVO')
ERE = re.compile(r'等效地面=([\d.]+)cm')
BRE = re.compile(r'bias=\(([-+\d.]+),([-+\d.]+)\)')
VRE = re.compile(r'vxy=([\d.]+)m/s')

print('%-26s %5s %7s %8s %8s %9s %9s' % ('航班/投次', '段长s', '前5s残差', '后段残差', '残差峰峰', 'bias起点', 'bias终点'))
for rel in LOGS:
    p = os.path.join(ROOT, rel)
    if not os.path.exists(p):
        continue
    lab = rel.split('/')[0] + '/' + rel.split('_')[1][-6:-4]
    cur = []
    drops = []
    for line in open(p, encoding='utf-8', errors='ignore'):
        if 'STEP ' in line and '->DROP' in line:
            cur = []
        m = TRE.search(line)
        if not m:
            if '>>> DROP #' in line and cur:
                drops.append(cur)
                cur = []
            continue
        e, b = ERE.search(line), BRE.search(line)
        if e and b:
            cur.append((float(m.group(1)), float(e.group(1)),
                        abs(complex(float(b.group(1)), float(b.group(2)))) * 100))
    for k, d in enumerate(drops[:2]):
        t0 = d[0][0]
        early = [x[1] for x in d if x[0] - t0 <= 5.0]
        late = [x[1] for x in d if x[0] - t0 > 5.0]
        if not early:
            continue
        ee = sum(early) / len(early)
        ll = sum(late) / len(late) if late else float('nan')
        pp = (max(x[1] for x in d) - min(x[1] for x in d))
        print('%-26s %5.1f %6.1fcm %7.1fcm %7.1fcm %8.1fcm %8.1fcm'
              % ('%s #%d' % (lab, k + 1), d[-1][0] - t0, ee, ll, pp, d[0][2], d[-1][2]))
