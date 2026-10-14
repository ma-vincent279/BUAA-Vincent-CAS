#!/usr/bin/env python3
# ================================================================
# 2026-09-19 项目 d 改动 (按用户要求, 仅 SITL + 实机两份 d.py):
#   ① 收敛判据从"固定 10px"改为**米制: 等效地面 10cm**  (H=1.2m 时 ≈29px)
#   ② 兜底投弹限时 DROP_AFTER_DELAY 25s → 35s
#   ③ 日志补 "等效地面=x.xcm" 与 "触发方式=收敛即投/35s兜底" (便于下次实飞验收)
# 每处替换断言"恰好 1 次", 任一失败则整体不写盘.
# 用法: python3 scripts/patch_d_conv_20260919.py [--check]
# ================================================================
import ast
import os
import re
import sys

ROOT = '/home/vincent/SITL'
CHECK_ONLY = '--check' in sys.argv
FILES = ['src/drone_drl/drone_drl/d.py', 'fly/src/fly/fly/d.py']

CONST_NEW = """DROP_ARRIVE_M  = 0.10    # 2026-09-18: "收敛即投"判据改**米制** —— 等效地面 10cm (按用户要求)
                         #   H=1.2m 时 ≈29px (旧值 10px 只等效 3.4cm, 实机 ±13cm 摆动下不可能满足);
                         #   米制使判据不随投弹高度变化; 折算 = hypot(ex·H/FX, ey·H/FY)"""

DELAY_NEW = ('DROP_AFTER_DELAY = 35.0  # 兜底: 未收敛时最长等待 '
             '(2026-09-18: 25→35, 按用户要求给收敛更多时间; 收敛则仍走"收敛即投"提前投弹)')

CRIT_OLD = """            early_ok = False
            if (self.z > drop_z - 0.2 and self._last_drop_err is not None
                    and self._last_drop_err[0] < DROP_ARRIVE_PX):"""
CRIT_NEW = """            # 2026-09-18: 收敛判据改米制 —— 把最近一帧 (ex, ey, H) 折算成"等效地面距离"
            conv_m = float('inf')
            if self._last_drop_err is not None:
                _ep, _exl, _eyl, _hh = self._last_drop_err
                conv_m = math.hypot(_exl * _hh / FX, _eyl * _hh / FY)
            early_ok = False
            if self.z > drop_z - 0.2 and conv_m < DROP_ARRIVE_M:"""

LOG_OLD = ('                            f"bias=({self._servo_bias_x:+.2f},{self._servo_bias_y:+.2f}) '
           'pitch={self._pitch:+.3f}")')
LOG_NEW = ('                            f"bias=({self._servo_bias_x:+.2f},{self._servo_bias_y:+.2f}) '
           'pitch={self._pitch:+.3f} "\n'
           '                            f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm'
           '(门限{DROP_ARRIVE_M * 100:.0f}cm)")')

ERR_OLD = '                    err_str = f" last_err={ep:.0f}px(ex={exl:.0f},ey={eyl:.0f}@H={hh:.1f}m)"'
ERR_NEW = ('                    err_str = (f" last_err={ep:.0f}px(ex={exl:.0f},ey={eyl:.0f}@H={hh:.1f}m)"\n'
           '                               f" 等效地面={math.hypot(exl * hh / FX, eyl * hh / FY) * 100:.1f}cm")\n'
           '                drop_way = ("收敛即投" if early_ok\n'
           '                            else f"{self._step_elapsed():.1f}s/{DROP_AFTER_DELAY:.0f}s 兜底")')

EVT_OLD = 'heading={math.degrees(self.heading):.1f}deg{err_str}")'
EVT_NEW = 'heading={math.degrees(self.heading):.1f}deg 触发方式={drop_way}{err_str}")'

# 旧注释里对 DROP_ARRIVE_PX 的引用
C1_OLD = '# 第 2 桶进 DROP 后第一帧只要 err<DROP_ARRIVE_PX 且已到高度'
C1_NEW = '# 第 2 桶进 DROP 后第一帧只要"等效地面距离<门限"且已到高度'
C2_OLD = ('            # 2026-08-31: 收敛即投 — err<DROP_ARRIVE_PX 持续 DROP_CONVERGE_EARLY 则提前投弹 '
          '(防 SITL MPC 漂移);')
C2_NEW = ('            # 2026-08-31: 收敛即投 — err 折算"等效地面距离"<DROP_ARRIVE_M(10cm) 持续 '
          'DROP_CONVERGE_EARLY 则提前投弹;')
C3_OLD = ('            # 2026-08-31: 收敛即投 — err<DROP_ARRIVE_PX 持续 DROP_CONVERGE_EARLY 则提前投弹 '
          '(防漂移);')
C3_NEW = ('            # 2026-08-31: 收敛即投 — err 折算"等效地面距离"<DROP_ARRIVE_M(10cm) 持续 '
          'DROP_CONVERGE_EARLY 则提前投弹;')


def rep(s, old, new, tag):
    n = s.count(old)
    assert n == 1, f'[{tag}] 期望 1 次, 实际 {n} 次:\n{old[:150]}'
    return s.replace(old, new, 1)


plan = []
for rel in FILES:
    p = os.path.join(ROOT, rel)
    s = open(p, encoding='utf-8').read()
    assert 'DROP_ARRIVE_M' not in s, f'[{rel}] 已打过补丁'
    s = re.sub(r'^DROP_ARRIVE_PX = 10\.0.*$', CONST_NEW.replace('\\', '\\\\'), s, count=1, flags=re.M)
    assert 'DROP_ARRIVE_M  = 0.10' in s, f'[{rel}] 常量替换失败'
    s = re.sub(r'^DROP_AFTER_DELAY = 25\.0.*$', DELAY_NEW, s, count=1, flags=re.M)
    assert 'DROP_AFTER_DELAY = 35.0' in s, f'[{rel}] 限时替换失败'
    s = rep(s, CRIT_OLD, CRIT_NEW, rel + ' 判据')
    s = rep(s, LOG_OLD, LOG_NEW, rel + ' 日志行')
    s = rep(s, ERR_OLD, ERR_NEW, rel + ' 投弹日志')
    s = rep(s, EVT_OLD, EVT_NEW, rel + ' 事件行')
    s = rep(s, C1_OLD, C1_NEW, rel + ' 注释1')
    s = rep(s, C2_OLD, C2_NEW, rel + ' 注释2') if C2_OLD in s else rep(s, C3_OLD, C3_NEW, rel + ' 注释2(fly)')
    ast.parse(s)
    plan.append((p, s))

for p, s in plan:
    if CHECK_ONLY:
        print('  [check ok]', os.path.relpath(p, ROOT))
    else:
        open(p, 'w', encoding='utf-8').write(s)
        print('  [written] ', os.path.relpath(p, ROOT))
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
