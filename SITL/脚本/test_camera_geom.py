#!/usr/bin/env python3
# ================================================================
# 2026-09-20 相机几何一致性验证 (离线, 不跑仿真)
# ----------------------------------------------------------------
# 目的: 判定"旧 PID 俯仰补偿符号"到底对不对, 以及 GPT 新增的
#       camera_pixel_to_level / level_to_camera_pixel 是否正确.
#
# 独立真值模型 (与被测代码无关):
#   level 系 = 水平系(前/右/下), body 系 = FRD(前/右/下).
#   PX4 约定 q 把 body 旋到 NED: v_level = Rz(yaw)·Ry(pitch)·Rx(roll)·v_body
#   去掉 yaw(由 _body_to_ned 处理) 后: v_level = Ry(pitch)·Rx(roll)·v_body
#   桶在 level 系的位置 = (f, r, H) → 机体系射线 = Ry(-pitch)·Rx(-roll)·(f,r,H)
#   像素(下视, 上=机头): cx = CX + FX·y/z, cy = CY - FY·x/z  (z=下为正)
#
# 被测:
#   A) 我们的旧 PID: ey=(CY-cy)-FY·tan(pitch); ex=(cx-CX)-FX·tan(roll)   ← 待判定
#   B) GPT 新几何:   camera_pixel_to_level(cx,cy,pitch,roll) → (fwd/H, right/H)
# 判据: 用真值像素喂进去, 谁能还原真实的地面偏移(米)就对.
# 用法: python3 scripts/test_camera_geom.py
# ================================================================
import math
import os
import re
import sys

ROOT = '/home/vincent/SITL'
FX, FY, CX, CY = 349.22, 353.27, 320.0, 180.0
GPT_FILE = '/tmp/gpt_d_lf.py'          # GPT 改后的实机 d.py (供提取新几何)


def extract(src, start, stop=r'^\S'):
    lines = src.split('\n')
    i0 = next(i for i, l in enumerate(lines) if re.match(start, l))
    i1 = next((i for i in range(i0 + 1, len(lines)) if re.match(stop, lines[i])), len(lines))
    return '\n'.join(lines[i0:i1]).rstrip('\n') + '\n'


# ---- 被测 A: 我们的旧公式 (从当前 fly/d.py 里抄出真实写法) ----
def old_pid(cx, cy, pitch, roll):
    ex = cx - CX
    ey = CY - cy
    ex -= FX * math.tan(roll)
    ey -= FY * math.tan(pitch)
    return ex, ey


# ---- 被测 B: GPT 的新几何 (直接 exec 它的源码) ----
ns = {'math': math, 'CX': CX, 'CY': CY, 'FX': FX, 'FY': FY}
src = open(GPT_FILE, encoding='utf-8').read()
exec(extract(src, r'^def camera_pixel_to_level'), ns)
exec(extract(src, r'^def level_to_camera_pixel'), ns)
new_pixel_to_level = ns['camera_pixel_to_level']
new_level_to_pixel = ns['level_to_camera_pixel']


def truth_pixel(f, r, H, pitch, roll):
    """真值: 地面偏移 (f 前, r 右, H 下) + 姿态 → 像素.

    PX4: v_level = Rz(yaw)·Ry(pitch)·Rx(roll)·v_body  →  去 yaw 后
    v_body = Rx(-roll)·Ry(-pitch)·v_level   (注意次序: 先 Ry 再 Rx)
    """
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    # Ry(-pitch): (x,z) → (x·cp - z·sp, x·sp + z·cp)
    x1 = f * cp - H * sp
    z1 = f * sp + H * cp
    # Rx(-roll): (y,z) → (y·cr + z·sr, -y·sr + z·cr)
    y2 = r * cr + z1 * sr
    z2 = -r * sr + z1 * cr
    return CX + FX * y2 / z2, CY - FY * x1 / z2


def truth_from_pixel_ref(f, r, H, pitch, roll):
    """用真值模型的反向: 直接算出 (ex, ey) 应当等于什么 (像素域, 与代码同定义)."""
    cx, cy = truth_pixel(f, r, H, pitch, roll)
    # 真值像素对应的水平偏移(米)就是 (f, r); 等价像素误差 = r·FX/H, f·FY/H
    return cx, cy, r * FX / H, f * FY / H


print('=' * 78)
print('0. 逆变换自检 (GPT 的 level_to_camera_pixel ∘ camera_pixel_to_level == 恒等?)')
ok = True
for f, r, H in [(0.0, 0.0, 1.2), (0.18, -0.24, 1.2), (-0.5, 0.35, 1.5), (0.9, 0.6, 2.5)]:
    for pitch in (-0.1, -0.03, 0.0, 0.03, 0.063, 0.12):
        for roll in (-0.05, 0.0, 0.04):
            cx, cy = truth_pixel(f, r, H, pitch, roll)
            got = new_pixel_to_level(cx, cy, pitch, roll)
            if got is None:
                ok = False; continue
            f2, r2 = got[0] * H, got[1] * H
            err = math.hypot(f2 - f, r2 - r)
            if err > 1e-9:
                ok = False
                print(f'   ✘ f={f} r={r} H={H} pitch={pitch} roll={roll} → 误差 {err:.2e}')
print('   →', '通过 (逐点 <1e-9)' if ok else '失败')

print()
print('=' * 78)
print('1. 关键判定: 用真值像素喂进两种公式, 看谁还原出真实地面偏移')
print('   (H=1.2m, 真值偏移 f=+0.0m / r=+0.0m —— 即"飞机正好在桶正上方")')
print()
print('   %-8s %-8s | %-26s | %-26s' % ('pitch', 'roll', '旧公式 (ex,ey) 等效地面', 'GPT 新几何 等效地面'))
print('   ' + '-' * 76)
for pitch_deg in (0.0, 1.5, 3.6, 5.0, 8.0):
    pitch = math.radians(pitch_deg)
    for roll_deg in (0.0, 2.0):
        roll = math.radians(roll_deg)
        cx, cy = truth_pixel(0.0, 0.0, 1.2, pitch, roll)      # 真值: 飞机在桶正上方
        ex_o, ey_o = old_pid(cx, cy, pitch, roll)
        o_r, o_f = ex_o * 1.2 / FX, ey_o * 1.2 / FY           # 旧公式"以为"的偏移(米)
        p = new_pixel_to_level(cx, cy, pitch, roll)
        n_f, n_r = (p[0] * 1.2, p[1] * 1.2) if p else (float('nan'),) * 2
        print('   %-8.1f %-8.1f | 前 %+7.4fm 右 %+7.4fm | 前 %+7.4fm 右 %+7.4fm' %
              (pitch_deg, roll_deg, o_f, o_r, n_f, n_r))
print()
print('   注: 真值应为 前 0.0000m 右 0.0000m; 偏离多少就是该公式引入的系统误差')

print()
print('=' * 78)
print('2. 投弹高度下"符号错"造成的系统误差 (H=1.2m):')
print('   旧式与真值的差 = 2·H·tan(pitch)  (前向), 横向 2·H·tan(roll)')
print('   %-8s %-16s %-16s' % ('pitch', '前向系统误差', '占桶半径比'))
for pitch_deg in (1.0, 2.0, 3.6, 5.0, 8.0):
    e = 2 * 1.2 * math.tan(math.radians(pitch_deg))
    print('   %-8.1f %-16s %-16s' % (pitch_deg, '%+.4f m' % e, '%.0f%%' % (100 * e / 0.125)))
print()
print('   实飞对照: 9-19 第1投 pitch=+3.6° → 15.1cm 系统误差 (桶半径 12.5cm) → 脱靶')
print('             9-19 第2投 pitch=+1.3° →  5.4cm            → 落点 3.3cm, 命中')
# ================================================================
# 3. 直接跑"出厂代码": 从四份 d/e 源码里 exec 真实 PIDController + 几何函数,
#    喂入真值像素 (飞机正好在桶正上方, pitch=3.6°), 看它报出多少地面偏差.
# ================================================================
FILES = {'SITL/d': 'src/drone_drl/drone_drl/d.py', 'fly/d': 'fly/src/fly/fly/d.py',
         'SITL/e': 'src/drone_drl/drone_drl/e.py', 'fly/e': 'fly/src/fly/fly/e.py'}
print()
print('=' * 78)
print('3. 出厂代码实测 (真值: 飞机在桶正上方, 偏差应为 0.0000m; H=1.2m)')


class _Roi:
    def __init__(self, cx, cy):
        self.x_offset, self.y_offset, self.width, self.height = cx - 5, cy - 5, 10.0, 10.0


def load_pid(path):
    src = open(path, encoding='utf-8').read()
    ns2 = {'math': math, 'CX': CX, 'CY': CY, 'FX': FX, 'FY': FY}
    for fn in ('camera_pixel_to_level', 'level_to_camera_pixel'):
        if f'def {fn}' in src:
            exec(extract(src, rf'^def {fn}'), ns2)
    exec(extract(src, r'^class PIDController'), ns2)
    return ns2['PIDController'], ns2


print('   %-8s %-14s %-14s' % ('代码', 'pitch=1.5° 报出', 'pitch=3.6° 报出'))
print('   ' + '-' * 44)
for k, rel in FILES.items():
    cls, ns2 = load_pid(os.path.join(ROOT, rel))
    row = []
    for pd in (1.5, 3.6):
        pitch = math.radians(pd)
        cx, cy = truth_pixel(0.0, 0.0, 1.2, pitch, 0.0)
        pid = cls(0.0015, 0.0, 0.001, 0.3, 0.0, 5.0)
        _, _, err, ex, ey = pid.compute(_Roi(cx, cy), 0.05, 0.0, pitch, 0.0, 0.24)
        row.append(math.hypot(ex * 1.2 / FX, ey * 1.2 / FY))
    tag = '✓ 已修' if max(row) < 0.01 else '✗ 仍偏'
    print('   %-8s %-14s %-14s %s' % (k, '%.4fm' % row[0], '%.4fm' % row[1], tag))
print()
print('   期望: 四份都应报 0.0000m (d 已用 1 行符号修正达成, e 由统一几何达成)')
print('   注意: 本段只喂 roll=0, 抓不到 roll 那行的问题 → 完整符号判定见 scripts/test_d_pitch_sign.py')
sys.exit(0 if ok else 1)
