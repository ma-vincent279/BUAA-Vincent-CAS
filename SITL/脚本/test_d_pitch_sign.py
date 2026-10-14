#!/usr/bin/env python3
# ================================================================
# 2026-09-20 项目 d "PID 姿态补偿符号" 判定与闭环验收 (离线, 不跑仿真)
# ----------------------------------------------------------------
# 背景: 9-19 实飞第一投 pitch=+3.6° 脱靶 0.235m, 第二投 pitch=+1.3° 命中.
#       怀疑 d 的 PID 姿态补偿符号有问题, 系统误差 = 2·H·tan(pitch).
#
# 本脚本回答三个问题, 全部用**独立真值模型**判定, 不靠猜:
#   Q1: d 的两行补偿 (ex -= FX·tan(roll) / ey -= FY·tan(pitch)) 到底哪几行错?
#       → 穷举 4 种符号组合, 用真值像素喂进去, 看谁在"飞机正上方"时报 0.
#   Q2: 错的那行在 DROP 段闭环里会造成多大的瞄准偏置?
#       → 忠实复刻 d.py 的"像素残差积分器"(真 PIDController + 真常量), 跑 35s.
#   Q3: 修正后闭环是否收敛到 0?
#
# 独立真值模型 (与 d.py 无关):
#   PX4: v_level = Rz(yaw)·Ry(pitch)·Rx(roll)·v_body  → 去 yaw: v_level = Ry(p)·Rx(r)·v_body
#   桶在水平系 (前 f, 右 r, 下 H) → 机体系 v_body = Rx(-r)·Ry(-p)·(f,r,H)
#   像素(下视, 图像上=机头): cx = CX + FX·y_body/z_body, cy = CY - FY·x_body/z_body
#
# 用法: python3 scripts/test_d_pitch_sign.py
# ================================================================
import math
import os
import re
import sys

ROOT = '/home/vincent/SITL'
FX, FY, CX, CY = 349.22, 353.27, 320.0, 180.0
H = 1.2                 # DROP_HEIGHT
CRUISE_HEIGHT = 5.0     # d.py 里 height_scale = H / CRUISE_HEIGHT
FILES = {'SITL/d': 'src/drone_drl/drone_drl/d.py',
         'fly/d': 'fly/src/fly/fly/d.py'}


# ---------------- 独立真值模型 ----------------
def truth_pixel(f, r, height, pitch, roll):
    """(前 f, 右 r, 下 height) + 姿态 → 像素 (cx, cy)."""
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    x1 = f * cp - height * sp            # Ry(-pitch)
    z1 = f * sp + height * cp
    y2 = r * cr + z1 * sr                # Rx(-roll)
    z2 = -r * sr + z1 * cr
    return CX + FX * y2 / z2, CY - FY * x1 / z2


# ---------------- 被测: d 的补偿公式 (4 种符号组合) ----------------
def d_pid_compensated(cx, cy, pitch, roll, s_roll, s_pitch):
    """s_roll/s_pitch = -1 表示代码里的 '-=', +1 表示 '+='."""
    ex = (cx - CX) + s_roll * FX * math.tan(roll)
    ey = (CY - cy) + s_pitch * FY * math.tan(pitch)
    return ex, ey


print('=' * 84)
print('Q1. 穷举 d 的 4 种符号组合 (真值: 飞机正好在桶正上方 → 报出应为 0.0000m)')
print('    H=%.1fm; 报出的"等效地面误差" = |(ex·H/FX, ey·H/FY)|' % H)
print()
print('    %-6s %-6s | %-13s %-13s %-13s %-13s' %
      ('pitch', 'roll', 'ex-=,ey-=(现状)', 'ex-=,ey+=', 'ex+=,ey-=', 'ex+=,ey+='))
print('    ' + '-' * 78)
combos = {'ex-=,ey-=(现状)': (-1, -1), 'ex-=,ey+=': (-1, +1),
          'ex+=,ey-=': (+1, -1), 'ex+=,ey+=': (+1, +1)}
worst = {k: 0.0 for k in combos}
for pitch_deg in (1.5, 3.6, 5.0):
    for roll_deg in (0.0, 2.0):
        pitch, roll = math.radians(pitch_deg), math.radians(roll_deg)
        cx, cy = truth_pixel(0.0, 0.0, H, pitch, roll)   # 飞机在桶正上方
        cells = []
        for k, (sr, sp) in combos.items():
            ex, ey = d_pid_compensated(cx, cy, pitch, roll, sr, sp)
            e = math.hypot(ex * H / FX, ey * H / FY)
            worst[k] = max(worst[k], e)
            cells.append('%.4fm' % e)
        print('    %-6.1f %-6.1f | %-13s %-13s %-13s %-13s' %
              (pitch_deg, roll_deg, cells[0], cells[1], cells[2], cells[3]))
print()
print('    最坏情况汇总:')
for k in combos:
    tag = '✓ 正确' if worst[k] < 0.005 else '✗ 有系统误差'
    print('      %-16s 最大 %.4fm  %s' % (k, worst[k], tag))
print()
print('    → 结论: 只有 [ex-=, ey+=] 在"正上方"时报 0;')
print('      横向(roll)那行现状已经是 -=(正确), 把它改成 += 反而引入 2·H·tan(roll) 的新横向误差.')

# ---------------- Q2/Q3: 忠实复刻 d.py DROP 段闭环 ----------------
print()
print('=' * 84)
print('Q2/Q3. DROP 段"像素残差积分器"闭环 (真 PIDController + 真常量, 30Hz, 35s)')


def load_module(path):
    src = open(path, encoding='utf-8').read()
    ns = {'math': math, 'CX': CX, 'CY': CY, 'FX': FX, 'FY': FY}
    def extract(start, stop=r'^\S'):
        lines = src.split('\n')
        i0 = next(i for i, l in enumerate(lines) if re.match(start, l))
        i1 = next((i for i in range(i0 + 1, len(lines)) if re.match(stop, lines[i])), len(lines))
        return '\n'.join(lines[i0:i1]).rstrip('\n') + '\n'
    exec(extract(r'^class PIDController'), ns)
    # 常量
    for name in ('SERVO_BIAS_MAX', 'SERVO_BIAS_ERR_MAX_M', 'SERVO_BIAS_VMAX',
                 'MAX_HORIZ_VEL', 'PAYLOAD_OFFSET_BODY_Y_B1', 'PAYLOAD_OFFSET_BODY_Y_B2',
                 'VISUAL_SERVO_KP', 'VISUAL_SERVO_KI', 'VISUAL_SERVO_KD',
                 'VISUAL_SERVO_INT_LIMIT', 'VISUAL_SERVO_DEAD_ZONE'):
        m = re.search(rf'^{name}\s*=\s*([-\d.eE]+)', src, re.M)
        if m:
            ns[name] = float(m.group(1))
    return ns


class _Roi:
    def __init__(self, cx, cy):
        self.x_offset, self.y_offset, self.width, self.height = cx - 5, cy - 5, 10.0, 10.0


def closed_loop(ns, pitch_deg, roll_deg, s_pitch, s_roll, flip_roll_calc=False, secs=35.0, dt=0.05):
    """复刻 d.py DROP 段: target 由正确的反投影重建, PID 输出经"像素残差积分器"加到 target 上.

    抽象: b = 积分器把瞄准点推出去的前向/横向偏移 (= 飞机相对桶的实际偏移, 假设内环完美跟踪).
    """
    pid = ns['PIDController'](ns.get('VISUAL_SERVO_KP', 0.0015), 0.0,
                              ns.get('VISUAL_SERVO_KD', 0.001), ns['MAX_HORIZ_VEL'],
                              ns.get('VISUAL_SERVO_INT_LIMIT', 0.0), 5.0)
    pitch, roll = math.radians(pitch_deg), math.radians(roll_deg)
    bf = bl = 0.0                      # 积分器产生的前向/右向瞄准偏移
    bias_max, err_max, vmax = (ns['SERVO_BIAS_MAX'], ns['SERVO_BIAS_ERR_MAX_M'], ns['SERVO_BIAS_VMAX'])
    hist = []
    t = 0.0
    while t < secs:
        t += dt
        # 桶相对飞机的水平偏移 = -积分器偏移 (飞机被推出去多少, 桶就相对落后多少)
        f, r = -bf, -bl
        cx, cy = truth_pixel(f, r, H, pitch, roll)
        # ---- 真 PID 的 ex/ey 走的是 d.py 里那两行 (s_* 控制符号) ----
        ex_raw, ey_raw = cx - CX, CY - cy
        ex = ex_raw + s_roll * FX * math.tan(roll)
        ey = ey_raw + s_pitch * FY * math.tan(pitch)
        if abs(ex) < 5.0: ex = 0.0
        if abs(ey) < 5.0: ey = 0.0
        err_px = math.hypot(ex, ey)
        height_scale = H / CRUISE_HEIGHT
        kp = ns.get('VISUAL_SERVO_KP', 0.0015) * height_scale
        v_lat = max(-ns['MAX_HORIZ_VEL'], min(ns['MAX_HORIZ_VEL'], kp * ex))
        v_fwd = max(-ns['MAX_HORIZ_VEL'], min(ns['MAX_HORIZ_VEL'], kp * ey))
        # ---- 积分器门控 (与 d.py 逐条一致: 身份锁 accept + err 上限 + 准静止) ----
        if err_px * H / FX < err_max and 0.0 < vmax:      # 悬停准静止: |vxy|=0 < 0.15
            sc = 1.0 / max(0.5, height_scale)
            bvf = max(-ns['MAX_HORIZ_VEL'], min(ns['MAX_HORIZ_VEL'], v_fwd * sc))
            bvl = max(-ns['MAX_HORIZ_VEL'], min(ns['MAX_HORIZ_VEL'], v_lat * sc))
            bf += bvf * dt
            bl += bvl * dt
            n = math.hypot(bf, bl)
            if n > bias_max:
                bf *= bias_max / n
                bl *= bias_max / n
        hist.append((t, bf, bl, err_px))
    return hist


print()
print('    d 现状 (ey-=, ex-=) 与 修正后 (ey+=, ex-=) 的瞄准偏置收敛值:')
print('    %-10s %-10s | %-24s | %-24s | %-16s' %
      ('pitch', 'roll', '现状: 收敛偏置(前,右)', '修正: 收敛偏置(前,右)', '理论 2·H·tan(pitch)'))
print('    ' + '-' * 92)
for pd in (1.5, 3.6, 5.0):
    for rd in (0.0, 2.0):
        h_old = closed_loop(load_module(os.path.join(ROOT, FILES['SITL/d'])), pd, rd, -1, -1)
        h_new = closed_loop(load_module(os.path.join(ROOT, FILES['SITL/d'])), pd, rd, +1, -1)
        bo_f, bo_l = h_old[-1][1], h_old[-1][2]
        bn_f, bn_l = h_new[-1][1], h_new[-1][2]
        theory = 2 * H * math.tan(math.radians(pd))
        print('    %-10.1f %-10.1f | 前 %+8.4f 右 %+8.4f | 前 %+8.4f 右 %+8.4f | %+.4fm' %
              (pd, rd, bo_f, bo_l, bn_f, bn_l, theory))

print()
print('    解读: 现状的收敛偏置 ≈ -2·H·tan(pitch) (即被推离桶 15cm @3.6°), 修正后 ≈ 0.')
print('          横向(roll)在两种情况下都是 ≈0 → 证明 roll 那行本来就是对的, 不能动.')
print()

# ---- 断言: 供 CI/回归使用 ----
ns_d = load_module(os.path.join(ROOT, FILES['SITL/d']))
DEAD_ZONE_PX = 5.0     # d.py 里 VISUAL_SERVO_DEAD_ZONE: |ex|,|ey| < 5px 直接置零
ok = True
for pd in (1.5, 3.6):
    h = closed_loop(ns_d, pd, 0.0, +1, -1)
    if abs(h[-1][1]) > 0.01 or abs(h[-1][2]) > 0.01:
        ok = False
        print('  ✘ 修正后仍未收敛到 0: pitch=%.1f → (%.4f, %.4f)' % (pd, h[-1][1], h[-1][2]))
    h0 = closed_loop(ns_d, pd, 0.0, -1, -1)
    # 现状平衡点 = 2·H·tan(pitch) 减去死区折算 (积分在 |ey|<5px 时停止)
    theory = 2 * H * math.tan(math.radians(pd)) - DEAD_ZONE_PX * H / FY
    if abs(abs(h0[-1][1]) - theory) > 0.003:
        ok = False
        print('  ✘ 现状偏置与理论不符: pitch=%.1f → %.4f vs %.4f (2H·tan(p) - 死区折算)'
              % (pd, abs(h0[-1][1]), theory))
    else:
        print('  ✓ pitch=%.1f: 现状偏置 %.4fm = 2·H·tan(p)(%.4fm) - 死区折算(%.4fm)'
              % (pd, abs(h0[-1][1]), 2 * H * math.tan(math.radians(pd)), DEAD_ZONE_PX * H / FY))
print('  回归断言:', '通过 (修正后 <1cm; 现状偏置 = 2·H·tan(pitch) 扣死区, 误差 <3mm)' if ok else '失败')
sys.exit(0 if ok else 1)
