#!/usr/bin/env python3
# ================================================================
# 2026-09-13 补丁 B: 像素残差积分闭环防"绕圈振荡"
# ----------------------------------------------------------------
# 背景: 补丁 A 的积分项在 SITL 里绕圈振荡 —— d 首投 target 在桶位附近 ±0.4m 摆振,
#       落点 0.236m/6分 (比改动前的 0.212m 更差); e 第2投同样出现 ~10-15s 周期摆振,
#       导致"收敛即投"始终不成立, 只能 25s 兜底.
# 机理: 进 DROP 时飞机还在往目标位置收敛 (0.3~0.4 m/s), 像素误差里混着"追赶迟滞",
#       积分项把这个迟滞当成"反投影偏置"去补 → 目标自己往前走 → 飞机追 → 误差反向 →
#       积分项又反向补 = 绕圈. 内环迟滞越大, 越容易振.
# 修法 (两条, 都只影响"何时/多快"移动重建目标):
#   ① 准静止门控: 飞机水平速度 > SERVO_BIAS_VMAX=0.15m/s 时该帧不积分 (迟滞不会被当偏置)
#   ② 低空反除幅度 3.3x → 2x: 留足相位裕度 (实测 3.3x 会振)
# 用法: python3 scripts/patch_drop_lock_20260913b.py [--check]
# ================================================================
import ast
import os
import sys

ROOT = '/home/vincent/SITL'
CHECK_ONLY = '--check' in sys.argv
TARGETS = ['src/drone_drl/drone_drl/e.py', 'src/drone_drl/drone_drl/d.py',
           'fly/src/fly/fly/e.py', 'fly/src/fly/fly/d.py']

CONST_OLD = 'SERVO_BIAS_ERR_MAX_M   = 1.0'
CONST_ADD = ('\nSERVO_BIAS_VMAX        = 0.15   # 准静止门控: 飞机水平速度超此值该帧不积分\n'
             '                                #   (否则把"追赶迟滞"当成反投影偏置 → 目标自走 → 绕圈振荡)')

GATE_OLD_TAIL = "err_px * dz_h / FX < SERVO_BIAS_ERR_MAX_M:"
GATE_NEW_TAIL = ("err_px * dz_h / FX < SERVO_BIAS_ERR_MAX_M\n"
                 "                            and math.hypot(self.vx, self.vy) < SERVO_BIAS_VMAX):")

SC_OLD_TMPL = ("# 低空 height_scale(=H/CRUISE_HEIGHT=0.3) 会把这步速率压到 1/3.3 → 反除回来:\n"
               "{i}# 该缩放是为防 bbox jitter 引起的高频振荡, 而积分项本身是低通 (jitter 被平均掉),\n"
               "{i}# 不反除则 0.24m 偏置要 ~17s 才收敛, 挤在 25s 兜底边缘. 速率仍受 MAX_HORIZ_VEL 限幅.\n"
               "{i}sc = 1.0 / max(0.25, height_scale)")
SC_NEW_TMPL = ("# 低空 height_scale(=H/CRUISE_HEIGHT=0.3) 会把速率压到 1/3.3, 部分反除 (最多 2x):\n"
               "{i}# 该缩放是为防 bbox jitter 引起的高频振荡, 而积分项本身低通 (jitter 被平均掉);\n"
               "{i}# 但不反除则 0.24m 偏置要 ~17s 才收敛. 实测 3.3x 反除会与内环迟滞绕圈振荡\n"
               "{i}# (2026-09-13 SITL d 首投 target ±0.4m 摆振 → 0.236m/6分), 故 2x + 准静止门控.\n"
               "{i}sc = 1.0 / max(0.5, height_scale)")


def rep(text, old, new, tag):
    cnt = text.count(old)
    assert cnt == 1, f'[{tag}] 期望 1 次, 实际 {cnt} 次:\n{old[:160]}'
    return text.replace(old, new, 1)


plan = []
for rel in TARGETS:
    p = os.path.join(ROOT, rel)
    s = open(p, encoding='utf-8').read()
    assert 'SERVO_BIAS_VMAX' not in s, f'[{rel}] 已打过补丁 B'
    s = rep(s, CONST_OLD, CONST_OLD + CONST_ADD, rel + ' const')
    # 门控: 缩进随文件 (SITL 20 空格 / fly 24 空格)
    ind = None
    for ln in s.split('\n'):
        if GATE_OLD_TAIL in ln and 'if ' in ln and 'lock_mode' in ln:
            ind = len(ln) - len(ln.lstrip())
    assert ind is not None, f'[{rel}] 找不到积分门控行'
    gate_old = 'if lock_mode' + (' ' * 1) + "in ('accept', 'reanchor') and " + GATE_OLD_TAIL
    gate_new = ('if (lock_mode in (\'accept\', \'reanchor\')\n'
                + ' ' * (ind + 8) + 'and ' + GATE_NEW_TAIL)
    s = rep(s, gate_old, gate_new, rel + ' gate')
    # 反除幅度 3.3x → 2x (注释 + 代码一起换, 注释缩进 = 代码行缩进)
    rows = [ln for ln in s.split('\n') if 'sc = 1.0 / max(0.25, height_scale)' in ln]
    assert len(rows) == 1, f'[{rel}] sc 行 {len(rows)} 条'
    i = ' ' * (len(rows[0]) - len(rows[0].lstrip()))
    s = rep(s, SC_OLD_TMPL.format(i=i), SC_NEW_TMPL.format(i=i), rel + ' sc')
    ast.parse(s)
    plan.append((p, s))

for p, s in plan:
    tag = os.path.relpath(p, ROOT)
    if CHECK_ONLY:
        print(f'  [check ok] {tag}')
    else:
        open(p, 'w', encoding='utf-8').write(s)
        print(f'  [written]  {tag}')
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
