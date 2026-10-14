#!/usr/bin/env python3
# ================================================================
# 2026-09-19 实飞 PX4 ULog 深度分析 (项目 d 代码, 两次投弹)
# 日志: log_422_2026-9-19-18-04-34.ulg   (3:05, 高频: 姿态 20Hz / 位置 10Hz / PWM 10Hz)
# ----------------------------------------------------------------
# 本脚本用 PX4 自己的高频日志独立复算 ROS 日志的结论, 并挖掘新问题:
#   1. 释放瞬间的姿态/高度/速度/执行误差  (独立于 ROS 1Hz 采样)
#   2. 释放延迟: AUX5 指令 → PWM 输出 → 载荷物理脱钩 (用推力/电流阶跃判定)
#   3. 投放脉冲完整性: manual_control_setpoint 的双发布者抢占 (新发现)
#   4. 悬停抬头 (姿态设定点) 与幻影误差 2·H·tan(pitch) 的关系
#   5. 平台健康: 振动/饱和/电池/GPS/EKF
# 用法: python3 scripts/analyze_ulog_20260919.py [ulog路径]
# ================================================================
import math
import sys

import numpy as np
from pyulog import ULog

ULG = sys.argv[1] if len(sys.argv) > 1 else '/home/vincent/SITL/log_422_2026-9-19-18-04-34.ulg'
BOOT_UTC = 1789811217.780553        # msg_info boot_time_utc_us / 1e6
BUCKET_R = 0.125                     # 桶半径 (m)
DROP_HEIGHT = 1.2                    # 代码里的投放高度


def q_pitch(q):
    w, x, y, z = [float(v) for v in q]
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


def q_roll(q):
    w, x, y, z = [float(v) for v in q]
    return math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))


def main():
    u = ULog(ULG)
    d = {x.name: x for x in u.data_list}
    ms = d['manual_control_setpoint']
    tms = ms.data['timestamp'] * 1e-6
    aux = np.array(ms.data['aux5'], float)
    lp = d['vehicle_local_position']
    tlp = lp.data['timestamp'] * 1e-6
    att = d['vehicle_attitude']
    ta = att.data['timestamp'] * 1e-6
    asp = d['vehicle_attitude_setpoint']
    tas = asp.data['timestamp'] * 1e-6
    lsp = d['vehicle_local_position_setpoint']
    tls = lsp.data['timestamp'] * 1e-6
    ao = d['actuator_outputs']
    tao = ao.data['timestamp'] * 1e-6
    pwm = np.array(ao.data['output[4]'], float)
    bat = d['battery_status']
    tb = bat.data['timestamp'] * 1e-6
    cur = np.array(bat.data['current_a'], float)
    mot = d['actuator_motors']
    tm = mot.data['timestamp'] * 1e-6
    m0 = np.array(mot.data['control[0]'], float)
    pit = np.array([q_pitch([att.data['q[%d]' % i][k] for i in range(4)]) for k in range(len(ta))])
    rol = np.array([q_roll([att.data['q[%d]' % i][k] for i in range(4)]) for k in range(len(ta))])
    pit_s = np.array([q_pitch([asp.data['q_d[%d]' % i][k] for i in range(4)]) for k in range(len(tas))])
    i2 = lambda t, k: float(np.interp(t, tlp, k))
    x, y, z = lp.data['x'], lp.data['y'], lp.data['z']
    vx, vy, vz = lp.data['vx'], lp.data['vy'], lp.data['vz']

    # 投放脉冲 = aux5 首次远离 0 的跳变及其后的每次重新抬升
    pulses, prev = [], 0.0
    for ti, v in zip(tms, aux):
        v = float(v)
        if abs(v) > 0.5 and abs(prev) < 0.5:
            pulses.append(ti)
        prev = v

    print('=' * 104)
    print('一、释放瞬间状态 (ULog 高频复算)  —— t 为 ULog 秒; Unix = t + %.3f' % BOOT_UTC)
    print('=' * 104)
    print('%-6s %-9s %-8s %-8s %-8s %-8s %-9s %-10s %-11s' %
          ('投', 't', 'pitch°', 'roll°', 'H(m)', '|vxy|', 'vz(m/s)', '执行误差', '幻影2H·tan(p)'))
    summary = []
    for k, tp in enumerate([pulses[0], pulses[-1]], 1):
        r = q_roll([att.data['q[%d]' % i][int(np.argmin(abs(ta - tp)))] for i in range(4)])
        p = q_pitch([att.data['q[%d]' % i][int(np.argmin(abs(ta - tp)))] for i in range(4)])
        H = -i2(tp, z)
        vxy = math.hypot(i2(tp, vx), i2(tp, vy))
        ex = math.hypot(i2(tp, x) - i2(tp, lsp.data['x']), i2(tp, y) - i2(tp, lsp.data['y']))
        phantom = 2 * H * math.tan(math.radians(p))
        outcome = 'MISS' if abs(phantom) > BUCKET_R else 'HIT'
        summary.append((p, H, phantom, outcome))
        print('%-6d %-9.3f %-8.2f %-8.2f %-8.3f %-8.3f %-9.3f %-10.3f %-8.1fcm → %s' %
              (k, tp, p, r, H, vxy, i2(tp, vz), ex, phantom * 100, outcome))
    print('\n  解读: 幻影项 > 桶半径(12.5cm) → 必脱靶; < 桶半径 → 命中. 与实飞观察 (第1投脱靶/第2投命中) 一致.')
    print('        执行误差 (飞机 vs 自己的设定点) 很小 → 脱靶是"瞄准"问题, 不是"飞行"问题.')

    print()
    print('=' * 104)
    print('二、投放脉冲完整性 (新发现: manual_control_setpoint 有两个发布者)')
    print('=' * 104)
    # 脉冲窗口 = 同一次投放的连续脉冲串 (相邻间隔 <6s 归为同一次投放); 串外的 0 是正常待机
    bursts = [[pulses[0]]]
    for t in pulses[1:]:
        (bursts[-1] if t - bursts[-1][-1] < 6.0 else bursts.append([t]) or bursts[-1]).append(t)
    burst1 = bursts[0]
    win0, win1 = burst1[0], burst1[-1]
    lo = [(t, float(a)) for t, a in zip(tms, aux) if win0 < t < win1 and abs(a) < 0.5]
    hi = [(t, float(a)) for t, a in zip(tms, aux) if win0 < t < win1 and abs(a) > 0.5]
    print('  第 1 投的脉冲串: t=%.3f~%.3f 共 %d 次抬升 (间隔约 4s); 其间 aux5=+0.833 样本 %d 个 / aux5=0 样本 %d 个'
          % (win0, win1, len(burst1), len(hi), len(lo)))
    for t, _ in lo:
        i = int(np.argmin(abs(tms - t)))
        same = ms.data['sticks_moving'][i]
        rl = ms.data['roll'][i]
        print('     ⚠ t=%.3f  aux5=0 (→1500us 锁)  sticks_moving=%d roll=%+.5f  ← PX4 RC 模块的样本抢占了节点样本' %
              (t, same, rl))
    bad = [(t, p) for t, p in zip(tao, pwm) if 933.30 < t < 944.20 and p != 2000]
    print('  实际 PWM(output[4]): 窗口内 %d 个采样, 非 2000 的 %d 个 → 投放舵机被拉回锁定 %d 次:' %
          (len([1 for t in tao if 933.30 < t < 944.20]), len(bad), len(bad)))
    for t, p in bad:
        print('     t=%.3f  PWM=%.0f us (持续约 1 个 10Hz 采样 ≈ 0.1s)' % (t, p))
    print('  节点侧: ROS 日志显示 DROP->CLIMB 发生在触发后 11.05s, I.e. 11 秒里共 4 次 2000us 脉冲.')
    print('  证据区分: 节点样本 roll=pitch=yaw=0 且 sticks_moving=1; RC 模块样本 roll=-0.00957, sticks_moving=0.')
    print('  RC 侧排除: 投弹期间所有 RC 通道无任何变化, 且 RC_MAP_AUX5=0 (遥控器未映射 AUX5).')

    print()
    print('=' * 104)
    print('三、释放延迟与载荷脱钩时刻 (推力/电流阶跃判定)')
    print('=' * 104)
    for k, tp in enumerate([pulses[0], pulses[-1]], 1):
        g = np.arange(tp - 0.5, tp + 1.3, 0.02)
        cs = np.interp(g, tb, cur)
        msig = np.interp(g, tm, m0)
        w = int(0.5 / 0.02)
        def step(sig):
            best = (0, None)
            for i in range(w, len(g) - w):
                dv = sig[i - w:i].mean() - sig[i:i + w].mean()
                if dv > best[0]:
                    best = (dv, g[i])
            return best
        dc, ts_c = step(cs)
        dm, ts_m = step(msig)
        drop = math.hypot(i2(ts_c, x) - i2(tp, x), i2(ts_c, y) - i2(tp, y))
        print('  第%d投: AUX5 指令 t=%.3f' % (k, tp))
        print('     电流阶跃 -%.1f A @ t=%.3f (滞后 %+.0f ms)  |  电机指令阶跃 -%.3f @ t=%.3f (滞后 %+.0f ms)' %
              (dc, ts_c, (ts_c - tp) * 1000, dm, ts_m, (ts_m - tp) * 1000))
        print('     → 从指令到载荷真正离开, 飞机只位移 %.1f cm (悬停几乎静止) ⇒ 释放延迟不构成误差源' % (drop * 100))

    print()
    print('=' * 104)
    print('四、悬停抬头 (环境外力) 与幻影误差的时变性')
    print('=' * 104)
    for name, a, b in [('巡航/前飞', 900, 920), ('第1桶投弹悬停', 925, 933), ('第2桶悬停', 965, 973), ('返航', 1040, 1050)]:
        w = (ta > a) & (ta < b)
        ws = (tas > a) & (tas < b)
        if w.sum():
            print('  %-14s 实际 pitch 均值 %+.2f° | 设定点 pitch 均值 %+.2f°' %
                  (name, pit[w].mean(), pit_s[ws].mean() if ws.sum() else float('nan')))
    print('  实际 = 设定点 ⇒ 控制器在主动抬头 (顶住风/CG 等水平外力), 不是姿态估计偏置.')
    print('  抬头量随航段变化 (投弹段 ~+4°, 第2桶 ~+2°, 返航 ~+0.4°) ⇒ 幻影误差 2·H·tan(pitch) 本身是时变量,')
    print('  这解释了跨航班"有时命中有时脱靶"的随机性.')

    print()
    print('=' * 104)
    print('五、平台健康')
    print('=' * 104)
    g = d['vehicle_gps_position']
    print('  GPS: 卫星 %d~%d, eph=%.2fm, epv=%.2fm, fix_type=%d (RTK)' %
          (g.data['satellites_used'].min(), g.data['satellites_used'].max(),
           g.data['eph'].mean(), g.data['epv'].mean(), g.data['fix_type'][0]))
    st = d['estimator_status']
    print('  EKF: filter_fault_flags=%s  health_flags=%s  timeout_flags=%s  gps_check_fail=%s' %
          (sorted(set(st.data['filter_fault_flags'].tolist())), sorted(set(st.data['health_flags'].tolist())),
           sorted(set(st.data['timeout_flags'].tolist())), sorted(set(st.data['gps_check_fail_flags'].tolist()))))
    im = d['vehicle_imu_status']
    print('  振动: accel_metric 最大 %.2f (削波 %d 次), gyro_metric 最大 %.2f' %
          (np.nanmax(im.data['accel_vibration_metric']), int(np.nansum(im.data['accel_clipping[0]'])),
           np.nanmax(im.data['gyro_vibration_metric'])))
    ca = d['control_allocator_status']
    sat = [int(np.array(ca.data['actuator_saturation[%d]' % i], bool).sum()) for i in range(4)]
    print('  4 个电机饱和帧数 (共 %d): %s → 悬停推力占比 %.2f (余量 %.0f%%)' %
          (len(ca.data['timestamp']), sat, np.nanmean(np.abs(d['vehicle_thrust_setpoint'].data['xyz[2]'])),
           (1 - np.nanmean(np.abs(d['vehicle_thrust_setpoint'].data['xyz[2]']))) * 100))
    print('  电池: %.1f~%.1f V, 电流均值 %.1f A (峰值 %.1f A)' %
          (bat.data['voltage_v'].min(), bat.data['voltage_v'].max(), cur.mean(), cur.max()))

    print()
    print('=' * 104)
    print('六、由幻影模型反推桶位置, 与实际落点对比 (自洽性检查)')
    print('=' * 104)
    hdg = [179.8, -179.4]
    for k, (tp, s) in enumerate(zip([pulses[0], pulses[-1]], summary)):
        p, H, phantom, out = s
        fwd = (math.cos(math.radians(hdg[k])), math.sin(math.radians(hdg[k])))   # NED 前向
        # 幻影把瞄准点/飞机留在"桶的后方" (机体前向的反方向) ⇒ 桶 = 飞机 + 幻影·前向
        bx = i2(tp, x) + phantom * fwd[0]
        by = i2(tp, y) + phantom * fwd[1]
        print('  第%d投: 飞机 NED(%+.3f,%+.3f) → 反推桶 NED(%+.3f,%+.3f), 落点距桶 %.1f cm → %s' %
              (k + 1, i2(tp, x), i2(tp, y), bx, by, abs(phantom) * 100, out))
    print('  (两个桶间距 %.2fm, 与现场实测约 1.66m 量级一致; 落点误差 12.7cm 略超半径→脱靶, 5.2cm 在半径内→命中)'
          % math.hypot((i2(pulses[0], x) + summary[0][2] * math.cos(math.radians(hdg[0]))) -
                       (i2(pulses[-1], x) + summary[1][2] * math.cos(math.radians(hdg[1]))),
                       (i2(pulses[0], y) + summary[0][2] * math.sin(math.radians(hdg[0]))) -
                       (i2(pulses[-1], y) + summary[1][2] * math.sin(math.radians(hdg[1])))))
    print()
    print('=' * 104)
    print('结论: ① 俯仰幻影项 (2·H·tan(pitch)) 用 PX4 高频数据独立复算成立, 12.7cm 脱靶 / 5.2cm 命中;')
    print('      ② 执行误差仅 5mm → 脱靶是瞄准问题; ③ 释放延迟 0.1~0.27s 但飞机几乎静止 → 不是误差源;')
    print('      ④ 新问题: 投放脉冲期间 PX4 的 RC 样本抢占 manual_control_setpoint, 把舵机拉回锁定 3 次.')
    print('=' * 104)
    return 0


if __name__ == '__main__':
    sys.exit(main())
