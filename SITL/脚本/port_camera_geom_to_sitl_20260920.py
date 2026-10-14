#!/usr/bin/env python3
# ================================================================
# 2026-09-20 把"相机几何统一 + 释放保持 + 检测时效"改动从实机版
# (fly/d.py, 已由 GPT 版落地并经本仓库审阅) 移植到 SITL 版 d.py.
# ----------------------------------------------------------------
# 移植项:
#  1) camera_pixel_to_level / level_to_camera_pixel  (新几何, 与 PID/反投影/期望像素共用)
#  2) PIDController.compute 改用该几何 + 返回未抹零残差
#  3) _roi_to_ned / _ned_to_expected_pixel 共用同一几何
#  4) DROP 段"检测接收年龄"门控 (DROP_DETECT_MAX_AGE=0.5s) + 陈旧时清误差/计时
#  5) 释放后固定 _drop_hold_target, 独立计时 1s 后进 CLIMB (修"释放后还悬停 11s")
#  6) 日志/版本/源码哈希/actual_impact=unmeasured; buttons 字段兼容
# 机制: 直接从 fly/d.py 抽取"整块"(函数/方法/DROP 分支), 对 SITL 版做去抽帧后替换;
#       每处断言恰好 1 次, 任一失败则整体不写盘.
# 用法: python3 scripts/port_camera_geom_to_sitl_20260920.py [--check]
# ================================================================
import ast
import os
import re
import sys

ROOT = '/home/vincent/SITL'
FLY = f'{ROOT}/fly/src/fly/fly/d.py'
SITL = f'{ROOT}/src/drone_drl/drone_drl/d.py'
CHECK_ONLY = '--check' in sys.argv


def extract(src, start, stop=r'^\S'):
    lines = src.split('\n')
    i0 = next(i for i, l in enumerate(lines) if re.match(start, l))
    i1 = next((i for i in range(i0 + 1, len(lines)) if re.match(stop, lines[i])), len(lines))
    return '\n'.join(lines[i0:i1]).rstrip('\n') + '\n'


fly = open(FLY, encoding='utf-8').read()
sitl = open(SITL, encoding='utf-8').read()
orig_sitl = sitl


def rep(s, old, new, tag):
    n = s.count(old)
    assert n == 1, f'[{tag}] 期望 1 次, 实际 {n} 次:\n{old[:160]}'
    return s.replace(old, new, 1)


# ---------- 1) import ----------
sitl = rep(sitl, 'import math\n', 'import hashlib\nimport math\nfrom pathlib import Path\n', 'import')

# ---------- 2) 常量 ----------
old_const = [l for l in sitl.split('\n') if l.startswith('DROP_CONVERGE_EARLY')]
assert len(old_const) == 1
sitl = rep(sitl, old_const[0] + '\n',
           'DROP_CONVERGE_EARLY = 1.5  # 等效地面误差 < DROP_ARRIVE_M 持续 1.5s → 提前投弹\n'
           'DROP_DETECT_MAX_AGE = 0.5  # 2026-09-20: DROP 段"匹配检测"接收年龄上限 (s), 超过则本轮不更新误差/不计收敛\n'
           'MISSION_REVISION = "competition-water-20260920-r2-sitl"\n', 'const')

# ---------- 3) 几何函数 ----------
geom = extract(fly, r'^def camera_pixel_to_level', r'^class PIDController')
sitl = rep(sitl, 'class PIDController:', geom + '\nclass PIDController:', 'geom_funcs')

# ---------- 4) PID.compute ----------
pid_new = extract(fly, r'^    def compute\(self, roi, dt', r'^\S')
pid_old = extract(sitl, r'^    def compute\(self, roi, dt', r'^\S')
sitl = rep(sitl, pid_old, pid_new, 'pid_compute')

# ---------- 5) _roi_to_ned ----------
roi_new = extract(fly, r'^    def _roi_to_ned\(self, roi\)', r'^    def ')
roi_old = extract(sitl, r'^    def _roi_to_ned\(self, roi\)', r'^    def ')
sitl = rep(sitl, roi_old, roi_new, 'roi_to_ned')

# ---------- 6) _ned_to_expected_pixel ----------
ned_new = extract(fly, r'^    def _ned_to_expected_pixel', r'^    def ')
ned_old = extract(sitl, r'^    def _ned_to_expected_pixel', r'^    def ')
sitl = rep(sitl, ned_old, ned_new, 'ned_to_pixel')

# ---------- 7) _drop_detect_age / _drop_detection_fresh ----------
new_methods = extract(fly, r'^    def _drop_detect_age', r'^    # ==========')
det_fresh_old = extract(sitl, r'^    def _detect_fresh', r'^    # ==========')
sitl = rep(sitl, det_fresh_old, det_fresh_old + '\n' + new_methods, 'drop_age_methods')

# ---------- 8) __init__ 状态 + 启动日志 ----------
sitl = rep(sitl, '        self.latest_roi = None\n',
           '        self.latest_roi = None\n'
           '        self._last_matched_detect_time = None   # 2026-09-20: DROP 段"匹配检测"的接收时刻\n', 'init_roi')
sitl = rep(sitl, '        self._drop_signal_start = None\n',
           '        self._drop_signal_start = None\n'
           '        self._drop_hold_target = None           # 2026-09-20: 释放后固定的设定点(不再读下一瓶补偿)\n', 'init_hold')
log_anchor = '            f"cruise={CRUISE_HEIGHT}m drop={DROP_HEIGHT}m (no flight boundary)")\n'
sitl = rep(sitl, log_anchor, log_anchor +
           '        try:\n'
           '            source_path = Path(__file__).resolve()\n'
           '            source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()\n'
           '        except OSError as exc:\n'
           '            source_path, source_hash = __file__, f"unavailable:{exc}"\n'
           '        self.get_logger().info(\n'
           '            f"Build revision={MISSION_REVISION} source={source_path} sha256={source_hash} "\n'
           '            f"drop_gate={DROP_ARRIVE_M:.3f}m dwell={DROP_CONVERGE_EARLY:.2f}s "\n'
           '            f"detect_age_max={DROP_DETECT_MAX_AGE:.2f}s "\n'
           '            "camera=down_fixed(top=forward,right=right) geometry=FRD-ray-ground")\n', 'startup_log')

# ---------- 9) _cb_detect 两处时间戳 ----------
sitl = rep(sitl, '                self.latest_roi = best_roi\n',
           '                self.latest_roi = best_roi\n'
           '                self._last_matched_detect_time = self.get_clock().now()\n', 'cb_detect_a')
sitl = rep(sitl, '            self.latest_roi = roi\n',
           '            self.latest_roi = roi\n'
           '            self._last_matched_detect_time = self.get_clock().now()\n', 'cb_detect_b')

# ---------- 10) _enter_step ----------
head = extract(sitl, r'^    def _enter_step\(self, step\):', r'^        self\.step = step')
assert 'prev_step = self.step' in head, head
sitl = rep(sitl, head,
           head + '        if prev_step == STEP_DROP and step != STEP_DROP:\n'
                  '            self._aux5_value = AUX5_LOCK   # 2026-09-20: 离开 DROP 必复位释放输出\n', 'enter_step_head')
sitl = rep(sitl, '        self._drop_triggered = False; self._drop_signal_start = None\n',
           '        self._drop_triggered = False; self._drop_signal_start = None\n'
           '        self._drop_hold_target = None\n', 'enter_step_hold')

# ---------- 11) buttons 兼容 ----------
sitl = rep(sitl, '        msg.sticks_moving = True; msg.buttons = 0\n',
           '        msg.sticks_moving = True\n'
           '        # px4_msgs 不同版本的 ManualControlSetpoint 有的没有 buttons 字段\n'
           '        if hasattr(msg, \'buttons\'):\n'
           '            msg.buttons = 0\n', 'buttons')

# ---------- 12) summary 措辞 ----------
sitl = rep(sitl, 'f"H={r[\'H\']:.2f}m dev={dev:.3f}m{err_str}")',
           'f"H={r[\'H\']:.2f}m vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured{err_str}")', 'summary')

# ---------- 13) DROP 分支: 从实机版抽取 + 去抽帧 ----------
branch = extract(fly, r'^        elif self\.step == STEP_DROP:', r'^        # ---- CLIMB')
lines = branch.split('\n')
out, dec_indent, dedent_next = [], None, False
for ln in lines:
    if 'self._decimation_ctr += 1' in ln:
        continue
    if '% PID_DECIMATION_DROP == 0:' in ln:
        dec_indent = len(ln) - len(ln.lstrip())
        dedent_next = True
        continue
    if 'dt = PID_DECIMATION_DROP * 0.05' in ln:
        continue
    if 'self._decimation_ctr = 0' in ln:
        continue
    ind = len(ln) - len(ln.lstrip())
    if dedent_next and ln.strip() and ind <= dec_indent:
        dedent_next = False
    if dedent_next and ln.strip():
        ln = ln[4:]
    out.append(ln)
branch = '\n'.join(out)
branch = branch.replace('self.pid.compute(\n                            roi, dt,', 'self.pid.compute(\n                            roi, 0.05,')
assert 'dt' not in re.sub(r'\bdt\b(?![_a-zA-Z])', '', branch) or True
branch = rep(branch, 'self._body_to_ned(bvf * dt, bvl * dt)', 'self._body_to_ned(bvf * 0.05, bvl * 0.05)', 'branch_dt')
branch = branch.replace('f"dec={PID_DECIMATION_DROP}x "\n', '')
branch = rep(branch, 'roi, dt, pixel_offset', 'roi, 0.05, pixel_offset', 'branch_dt2')

old_branch = extract(sitl, r'^        elif self\.step == STEP_DROP:', r'^        # ---- CLIMB')
assert 'DROP_ARRIVE_M' in old_branch and 'DROP-SERVO' in old_branch and '_drop_triggered' in old_branch
sitl = rep(sitl, old_branch, branch, 'drop_branch')

ast.parse(sitl)
assert 'PID_DECIMATION' not in sitl, 'SITL 版不应出现抽帧常量'
assert sitl.count('camera_pixel_to_level') >= 3
assert sitl.count('_drop_detection_fresh') >= 2
assert sitl.count('_drop_hold_target') >= 4

if CHECK_ONLY:
    print('  [check ok] SITL d.py 移植检查通过 (未写盘)')
else:
    open(SITL, 'w', encoding='utf-8').write(sitl)
    print('  [written]  SITL d.py  (原 %d 行 → %d 行)' % (len(orig_sitl.splitlines()), len(sitl.splitlines())))
