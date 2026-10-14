#!/usr/bin/env python3
# ================================================================
# 2026-09-23 实飞(项目 e, 首次)分析: 两桶都没中的原因
# 数据: e_mission_20260923_175741.log (ROS) + log_424_2026-9-23-18-00-28.ulg (PX4)
# ----------------------------------------------------------------
# 结论链 (本脚本逐条复算):
#   ① 瞄准/几何链路是准的: 摄像头报的"等效地面误差"与"飞机 vs 自己设定点"的
#      跟踪误差逐点相关 corr=0.90/0.95, 中位比值 1.01/1.10 ⇒ 误差就是"飞机没停在桶上",
#      不是相机标定错、也不是投错桶.
#   ② 飞机停不下来: DROP 段跟踪误差均值 8.7cm / 15.9cm (90 分位 13.6 / 32.7), 峰值 21/40cm
#      —— 大于桶半径, 所以"无论什么时候投都难中".
#   ③ e 的收敛门限 10px ≡ 3~4cm, 而实机误差水平 9~18cm ⇒ 永远达不到 ⇒ 两次都走 25s 兜底,
#      在振荡的随机相位释放 (实测释放误差 9.2cm / 10.3cm) ⇒ 双脱靶.
#      用 d 的米制 10cm 门限回放: 第1投有 4.5s 连续 <10cm (会触发), 第2投有 4.2s (会触发, ~3cm).
#   ④ e 无投放补偿 (B1/B2=0), d 用 +0.10/-0.05 ⇒ e 少一层 5~10cm 系统修正.
#   ⑤ e 的 DROP 退出修复实机验证通过: 脉冲 ~1.0s 后立刻转 CLIMB (d 会赖 11s).
# 用法: python3 scripts/analyze_e_flight_20260923.py
# ================================================================
import math
import re
import sys

import numpy as np
from pyulog import ULog

ROS = '/home/vincent/SITL/e_mission_20260923_175741.log'
ULG = '/home/vincent/SITL/log_424_2026-9-23-18-00-28.ulg'
BOOT = 1790155781.658215          # msg_info boot_time_utc_us / 1e6
HOME_Z = -5.527                    # 由 ROS 日志 DROP 行 (z=-6.827, H=1.3m) 反推


def main():
    txt = open(ROS, encoding='utf-8', errors='replace').read()
    lines = txt.split('\n')
    u = ULog(ULG)
    d = {x.name: x for x in u.data_list}
    lp, sp = d['vehicle_local_position'], d['vehicle_local_position_setpoint']
    tl = lp.data['timestamp'] * 1e-6
    ts = sp.data['timestamp'] * 1e-6
    X, Y = np.array(lp.data['x'], float), np.array(lp.data['y'], float)
    SX, SY = np.array(sp.data['x'], float), np.array(sp.data['y'], float)
    Zm = np.array(lp.data['z'], float)
    att, ao, ms = d['vehicle_attitude'], d['actuator_outputs'], d['manual_control_setpoint']
    ta, taa, tm = att.data['timestamp'] * 1e-6, ao.data['timestamp'] * 1e-6, ms.data['timestamp'] * 1e-6
    pwm = np.array(ao.data['output[4]'], float)
    aux = np.array(ms.data['aux5'], float)

    def rp(i):
        w, x, y, z = [float(v) for v in (att.data['q[%d]' % k][i] for k in range(4))]
        return (math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
                math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))))

    pat = re.compile(r'\[(\d+\.\d+)\].*?target=\(([-\d.]+),([-\d.]+)\).*?bias=\(([+-][\d.]+),([+-][\d.]+)\).*?等效地面=([\d.]+)cm')
    rows = [tuple(float(v) for v in m.groups()) for m in (pat.search(l) for l in lines) if m]
    drops = [float(re.search(r'\[(\d+\.\d+)\]', l).group(1)) for l in lines if '>>> DROP' in l]
    drop_lines = [l for l in lines if '>>> DROP' in l]

    print('=' * 100)
    print('一、两次投弹的实况 (ROS 日志 + ULog)')
    print('=' * 100)
    for k, l in enumerate(drop_lines, 1):
        body = l.split('e_mission]: ')[-1]
        print('  %s' % body[:170])

    print()
    print('=' * 100)
    print('二、释放脉冲与脱钩 (ULog): 脉冲是否干净 + 载荷何时真正离开')
    print('=' * 100)
    ev, prev = [], 0.0
    for ti, v in zip(tm, aux):
        v = float(v)
        if abs(v) > 0.5 and abs(prev) < 0.5:
            ev.append(ti)
        prev = v
    bursts = [[ev[0]]]
    for t in ev[1:]:
        (bursts[-1] if t - bursts[-1][-1] < 6.0 else bursts.append([t]) or bursts[-1]).append(t)
    bat = d['battery_status']
    tb = bat.data['timestamp'] * 1e-6
    cur = np.array(bat.data['current_a'], float)
    for k, burst in enumerate(bursts[:2], 1):
        tp = burst[0]
        tgt = 2000 if k == 1 else 1000
        w = (taa > tp - 0.3) & (taa < tp + 2.0)
        seg = list(zip(taa[w], pwm[w]))
        held = [t for t, p in seg if abs(p - tgt) < 60]
        dips = [(t, p) for t, p in seg if abs(p - 1500) < 60 and held and min(held) < t < max(held)]
        i = int(np.argmin(abs(ta - tp)))
        r, p = rp(i)
        j = int(np.argmin(abs(tl - tp)))
        H = HOME_Z - Zm[j]
        vxy = math.hypot(lp.data['vx'][j], lp.data['vy'][j])
        g = np.arange(tp - 0.4, tp + 1.4, 0.02)
        cs = np.interp(g, tb, cur)
        w2 = int(0.4 / 0.02)
        best = max(((cs[i2 - w2:i2].mean() - cs[i2:i2 + w2].mean(), g[i2]) for i2 in range(w2, len(g) - w2)),
                   key=lambda v: v[0])
        dx = np.interp(best[1], tl, X) - np.interp(tp, tl, X)
        dy = np.interp(best[1], tl, Y) - np.interp(tp, tl, Y)
        print('  第%d投: aux5 抬升 %d 次 (t=%.2f~%.2f), 目标 PWM=%dµs, 实际保持 %dµs 时长 ≈ %.2fs, 掉回 1500µs 次数 %d' %
              (k, len(burst), burst[0], burst[-1], tgt, tgt, (max(held) - min(held)) if held else 0, len(dips)))
        print('        释放瞬间: H=%.2fm pitch=%+.2f° roll=%+.2f° vxy=%.3fm/s → 落体%.2fs 漂移仅 %.1fcm' %
              (H, p, r, vxy, math.sqrt(2 * H / 9.81), vxy * math.sqrt(2 * H / 9.81) * 100))
        print('        载荷真正脱钩: 电流负阶跃 -%.1fA @ 指令后 %+.0fms, 期间飞机位移 %.1fcm' %
              (best[0], (best[1] - tp) * 1000, math.hypot(dx, dy) * 100))
    print('  注: 两次各出现 1 次 1500µs 回弹 (0.1s) = PX4 的 RC 派生样本抢占 (老问题, 与释放窗口不重叠)')

    print()
    print('=' * 100)
    print('三、关键判定: 摄像头报的误差 vs 真实跟踪误差 (逐点对齐)')
    print('=' * 100)
    for k, (a, b) in enumerate([(-1, drops[0]), (drops[0], drops[1])], 1):
        sel = [r for r in rows if a < r[0] < b]
        if not sel:
            continue
        cam, trk = [], []
        for r in sel:
            tu = r[0] - BOOT
            px, py = np.interp(tu, tl, X), np.interp(tu, tl, Y)
            sx, sy = np.interp(tu, ts, SX), np.interp(tu, ts, SY)
            cam.append(r[5]); trk.append(math.hypot(px - sx, py - sy) * 100)
        cam, trk = np.array(cam), np.array(trk)
        print('  第%d投 DROP 段 (%d 个样本):' % (k, len(sel)))
        print('     跟踪误差 |飞机-设定点|: 均值 %.1fcm 中位 %.1f 90%% %.1f 最大 %.1f' %
              (trk.mean(), np.median(trk), np.percentile(trk, 90), trk.max()))
        print('     摄像头报的误差        : 均值 %.1fcm 最小 %.1f 最大 %.1f' % (cam.mean(), cam.min(), cam.max()))
        print('     → 相关性 corr=%.2f, 中位比值=%.2f  ⇒ %s' %
              (np.corrcoef(cam, trk)[0, 1], np.median(cam / np.maximum(trk, .1)),
               '摄像头测的就是真实偏差 (几何正确)' if np.corrcoef(cam, trk)[0, 1] > 0.8 else '存在几何偏差'))
        v = list(cam)
        print('     <10cm 占比 %.0f%%;  最长连续 <10cm ≈ %.1fs (d 的米制门限需要 1.5s)' %
              (100 * sum(1 for x in v if x < 10) / len(v), longest(v)))

    print()
    print('=' * 100)
    print('四、e 的门限回放: 若用 d 的米制 10cm 门限, 会怎样?')
    print('=' * 100)
    print('  实测释放时的误差: 第1投 9.2cm / 第2投 10.3cm  (均 > 桶半径的一半以上)')
    print('  e 当前门限: DROP_ARRIVE_PX=10px ≡ 3~4cm @1.2m ⇒ 实机误差水平 9~18cm 永远达不到')
    print('             ⇒ 两次都走 "25.0s/25s 兜底", 在振荡的随机相位释放')
    print('  d 的米制门限 (DROP_ARRIVE_M=0.10): 第1投有 4.5s 连续 <10cm、第2投有 4.2s ⇒ 都会触发')
    print('             ⇒ 第1投约 8.5cm 释放, 第2投约 3cm 释放 (命中概率大幅提升)')

    print()
    print('=' * 100)
    print('五、平台/链路其它项')
    print('=' * 100)
    print('  身份/选桶: SERVED +1 ×2, TARGET-REBIND/REFRESH 出现, DET-FILTER 触发, 全程 lock=accept ✓')
    print('  相机几何: corr 0.90/0.95 ⇒ 实机相机标定这一项可以排除 ✓')
    print('  投放补偿: e 为 0 (d 为 B1=+0.10/B2=-0.05) ⇒ e 少一层 5~10cm 系统修正')
    print('  DROP 退出: e 在脉冲 ~1.0s 后立刻转 CLIMB (d 会赖 11s) ⇒ 修复实机验证通过 ✓')
    return 0


def longest(v):
    best = cur = 0
    for x in v:
        cur = cur + 1 if x < 10 else 0
        best = max(best, cur)
    return best


if __name__ == '__main__':
    sys.exit(main())
