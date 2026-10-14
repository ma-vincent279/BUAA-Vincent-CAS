#!/usr/bin/env python3
# ================================================================
# 2026-09-20 把"相机几何统一 + 释放保持 + 检测时效"这套改动 (原落在 d)
# 移植到**项目 e** (SITL + fly)。e 自己的东西保持不动:
#   - e 仍是 DROP_ARRIVE_PX=10px 判据 / DROP_AFTER_DELAY=25s (9-19 的米制+35s 是 d 专属)
#   - e 的 DROP/VISUAL_SERVO 日志字段 (px_raw / 收敛计时 / drop_z剩余 等) 全部保留
#   - e 的 VISUAL_SERVO 仍是 c 原样 (增量累加) —— 不动
# 共享代码块 (几何函数 / PID.compute / _roi_to_ned / _ned_to_expected_pixel /
#   _drop_detect_age+_drop_detection_fresh) 直接从当前 fly/d.py 抽取, 避免手抄出错。
# 每处替换断言恰好 1 次; 任一失败整体不写盘。
# 用法: python3 scripts/port_geom_to_e_20260920.py [--check]
# ================================================================
import ast
import os
import sys

ROOT = '/home/vincent/SITL'
FLY_D = f'{ROOT}/fly/src/fly/fly/d.py'
TARGETS = [f'{ROOT}/src/drone_drl/drone_drl/e.py', f'{ROOT}/fly/src/fly/fly/e.py']
CHECK_ONLY = '--check' in sys.argv

GEOM_START = r'^def camera_pixel_to_level'
PID_START = r'^    def compute\(self, roi, dt'
ROI_START = r'^    def _roi_to_ned\(self, roi\)'
NED_START = r'^    def _ned_to_expected_pixel'
AGE_START = r'^    def _drop_detect_age'


def extract(src, start, stop=r'^\S'):
    lines = src.split('\n')
    i0 = next(i for i, l in enumerate(lines) if __import__('re').match(start, l))
    i1 = next((i for i in range(i0 + 1, len(lines)) if __import__('re').match(stop, lines[i])), len(lines))
    return '\n'.join(lines[i0:i1]).rstrip('\n') + '\n'


def rep(s, old, new, tag):
    n = s.count(old)
    assert n == 1, f'[{tag}] 期望 1 次, 实际 {n} 次:\n{old[:170]}'
    return s.replace(old, new, 1)


flyd = open(FLY_D, encoding='utf-8').read()
BLOCK_GEOM = extract(flyd, GEOM_START, r'^class PIDController')
BLOCK_PID = extract(flyd, PID_START, r'^\S')
BLOCK_ROI = extract(flyd, ROI_START, r'^    def ')
BLOCK_NED = extract(flyd, NED_START, r'^    def ')
BLOCK_AGE = extract(flyd, AGE_START, r'^    # ==========')


def port(s, is_fly):
    tag = 'fly' if is_fly else 'sitl'
    ind = ' ' * (24 if is_fly else 20)          # DROP 段内语句缩进
    ind2 = ' ' * (28 if is_fly else 24)         # 更深一层
    dec = '                self._decimation_ctr = 0\n' if is_fly else ''   # else 分支体内缩进 16

    # 1) import
    s = rep(s, 'import math\n', 'import hashlib\nimport math\nfrom pathlib import Path\n', tag + '/import')

    # 2) 常量
    anchor = [l for l in s.split('\n') if l.startswith('DROP_CONVERGE_EARLY')]
    assert len(anchor) == 1
    s = rep(s, anchor[0] + '\n', anchor[0] + '\n'
            'DROP_DETECT_MAX_AGE = 0.5  # 2026-09-20: DROP 段"匹配检测"接收年龄上限 (s)\n'
            'MISSION_REVISION = "competition-water-20260920-r3-e"\n', tag + '/const')

    # 3) 几何函数 (插在 class PIDController 之前)
    s = rep(s, 'class PIDController:', BLOCK_GEOM + '\nclass PIDController:', tag + '/geom')

    # 4) PID.compute
    s = rep(s, extract(s, PID_START, r'^\S'), BLOCK_PID, tag + '/pid')

    # 5) _roi_to_ned / _ned_to_expected_pixel
    s = rep(s, extract(s, ROI_START, r'^    def '), BLOCK_ROI, tag + '/roi')
    s = rep(s, extract(s, NED_START, r'^    def '), BLOCK_NED, tag + '/ned')

    # 6) _drop_detect_age / _drop_detection_fresh (接在 _detect_fresh 之后)
    det = extract(s, r'^    def _detect_fresh', r'^    # ==========')
    s = rep(s, det, det + '\n' + BLOCK_AGE, tag + '/age')

    # 7) __init__ 状态
    s = rep(s, '        self.latest_roi = None\n',
            '        self.latest_roi = None\n'
            '        self._last_matched_detect_time = None   # 2026-09-20: DROP 段"匹配检测"接收时刻\n', tag + '/init1')
    s = rep(s, '        self._drop_signal_start = None\n',
            '        self._drop_signal_start = None\n'
            '        self._drop_hold_target = None           # 2026-09-20: 释放后固定的设定点\n', tag + '/init2')

    # 8) 启动日志 (e 的门限仍是 px, 显示时折算等效地面)
    lg = '            f"cruise={CRUISE_HEIGHT}m drop={DROP_HEIGHT}m (no flight boundary)")\n'
    s = rep(s, lg, lg +
            '        try:\n'
            '            source_path = Path(__file__).resolve()\n'
            '            source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()\n'
            '        except OSError as exc:\n'
            '            source_path, source_hash = __file__, f"unavailable:{exc}"\n'
            '        self.get_logger().info(\n'
            '            f"Build revision={MISSION_REVISION} source={source_path} sha256={source_hash} "\n'
            '            f"drop_gate={DROP_ARRIVE_PX:.0f}px(≈{DROP_ARRIVE_PX * DROP_HEIGHT / FX * 100:.0f}cm)"\n'
            '            f" dwell={DROP_CONVERGE_EARLY:.2f}s detect_age_max={DROP_DETECT_MAX_AGE:.2f}s "\n'
            '            "camera=down_fixed(top=forward,right=right) geometry=FRD-ray-ground")\n', tag + '/startlog')

    # 9) _cb_detect 时间戳
    s = rep(s, '                self.latest_roi = best_roi\n',
            '                self.latest_roi = best_roi\n'
            '                self._last_matched_detect_time = self.get_clock().now()\n', tag + '/cb1')
    s = rep(s, '            self.latest_roi = roi\n',
            '            self.latest_roi = roi\n'
            '            self._last_matched_detect_time = self.get_clock().now()\n', tag + '/cb2')

    # 10) _enter_step
    head = extract(s, r'^    def _enter_step\(self, step\):', r'^        self\.step = step')
    s = rep(s, head, head + '        if prev_step == STEP_DROP and step != STEP_DROP:\n'
                           '            self._aux5_value = AUX5_LOCK   # 2026-09-20: 离开 DROP 必复位释放输出\n',
            tag + '/enter_head')
    s = rep(s, '        self._drop_triggered = False; self._drop_signal_start = None\n',
            '        self._drop_triggered = False; self._drop_signal_start = None\n'
            '        self._drop_hold_target = None\n', tag + '/enter_hold')

    # 11) buttons 兼容
    s = rep(s, '        msg.sticks_moving = True; msg.buttons = 0\n',
            '        msg.sticks_moving = True\n'
            '        # px4_msgs 不同版本的 ManualControlSetpoint 有的没有 buttons 字段\n'
            "        if hasattr(msg, 'buttons'):\n"
            '            msg.buttons = 0\n', tag + '/buttons')

    # 12) SUMMARY 措辞
    s = rep(s, 'f"H={r[\'H\']:.2f}m dev={dev:.3f}m{err_str}")',
            'f"H={r[\'H\']:.2f}m vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured{err_str}")', tag + '/summary')

    # 13) DROP 分支: 释放后保持块 (放在分支最前)
    first = '        elif self.step == STEP_DROP:\n'
    s = rep(s, first, first +
            '            if self._drop_triggered:\n'
            '                # 2026-09-20: 释放后固定本次设定点, 独立计时; 不读下一瓶补偿/不再判收敛\n'
            '                self._pub_pos(*self._drop_hold_target)\n'
            '                held_s = (self.get_clock().now() - self._drop_signal_start).nanoseconds * 1e-9\n'
            '                if held_s >= AUX5_DROP_HOLD:\n'
            '                    self._aux5_value = AUX5_LOCK\n'
            '                    self.get_logger().info(f"DROP signal reset after {held_s:.2f}s (command hold)")\n'
            '                    self._enter_step(STEP_CLIMB)\n'
            '                return\n', tag + '/hold')

    # 14) 检测时效门控
    s = rep(s, 'if self._servo_active and self._detect_fresh():',
            'if self._servo_active and self._drop_detection_fresh():', tag + '/fresh')

    # 15) 陈旧分支: 保持最后伺服目标 + 清误差/计时
    s = rep(s, ('            else:\n'
                '                self._pub_pos(self.home_x + self._current_target_ned[0], '
                'self.home_y + self._current_target_ned[1], drop_z, self.heading)\n'),
            ('            else:\n'
             '                # 2026-09-20: 检测陈旧 → 保持最后伺服目标(不要跳回锚点), 清误差与收敛计时\n'
             '                self._last_drop_err = None\n'
             '                self._drop_converge_start = None\n' + dec +
             '                self._pub_pos(self.home_x + self._servo_target_x, '
             'self.home_y + self._servo_target_y, drop_z, self.heading)\n'), tag + '/stale')

    # 16) 积分门控改米制 (与 d 一致)
    s = rep(s, 'err_px * dz_h / FX < SERVO_BIAS_ERR_MAX_M',
            'math.hypot(ex * dz_h / FX, ey * dz_h / FY) < SERVO_BIAS_ERR_MAX_M', tag + '/biasgate')

    # 17) 判据: release_valid
    s = rep(s, ('            early_ok = False\n'
                '            if (self.z > drop_z - 0.2 and self._last_drop_err is not None\n'
                '                    and self._last_drop_err[0] < DROP_ARRIVE_PX):\n'),
            ('            release_valid = (self._drop_detection_fresh() and self._last_drop_err is not None\n'
             '                             and self.z > drop_z - 0.2)\n'
             '            early_ok = False\n'
             '            if (release_valid and self._last_drop_err[0] < DROP_ARRIVE_PX):\n'), tag + '/crit')

    # 18) 兜底与超时策略
    s = rep(s, '            if not early_ok and self._step_elapsed() < DROP_AFTER_DELAY: return\n',
            ('            if not self._drop_triggered and not early_ok and self._step_elapsed() < DROP_AFTER_DELAY: return\n'
             '            if not release_valid:\n'
             '                # 2026-09-20: 只有"高度门控不成立"才放弃释放; 检测陈旧时仍按原行为兜底释放\n'
             '                if not (self.z > drop_z - 0.2):\n'
             '                    self.get_logger().warn("DROP timeout without valid height; skip release and climb")\n'
             '                    self._aux5_value = AUX5_LOCK\n'
             '                    self._enter_step(STEP_CLIMB)\n'
             '                    return\n'
             '                self.get_logger().warn("DROP timeout with stale detection; fallback release at held aim")\n'),
            tag + '/timeout')

    # 19) 触发时保存保持目标
    s = rep(s, '                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()\n',
            '                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()\n'
            '                self._drop_hold_target = (self.home_x + self._servo_target_x,\n'
            '                                          self.home_y + self._servo_target_y, drop_z, self.heading)\n', tag + '/holdtarget')

    # 20) 日志: 1Hz 行加 roll/vxy/det_age, (log-only)→(bias-input)
    s = rep(s, 'f"v_fwd={v_fwd:.3f} v_lat={v_lat:.3f}(log-only) "',
            'f"v_fwd={v_fwd:.3f} v_lat={v_lat:.3f}(bias-input) "', tag + '/log_v')
    s = rep(s, 'f"bias=({self._servo_bias_x:+.2f},{self._servo_bias_y:+.2f}) pitch={self._pitch:+.3f}")',
            'f"bias=({self._servo_bias_x:+.2f},{self._servo_bias_y:+.2f}) pitch={self._pitch:+.3f} "\n'
            + ind2 + 'f"roll={self._roll:+.3f} vxy={math.hypot(self.vx, self.vy):.3f}m/s "\n'
            + ind2 + 'f"det_age={self._drop_detect_age():.3f}s "\n'
            + ind2 + 'f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm'
                     '(门限{DROP_ARRIVE_PX * dz_h / FX * 100:.0f}cm)")', tag + '/log2')

    # 21) 投弹事件行加字段
    s = rep(s, 'f"aim={aim_str} dev={dev:.3f}m "',
            'f"aim={aim_str} vehicle_to_anchor={dev:.3f}m "\n'
            + ind + 'f"pitch={self._pitch:+.3f} roll={self._roll:+.3f} "\n'
            + ind + 'f"vxy={math.hypot(self.vx, self.vy):.3f}m/s det_age={self._drop_detect_age():.3f}s "\n'
            + ind + 'f"hold_target={self._drop_hold_target} event=release_command "', tag + '/evt')

    ast.parse(s)
    for need in ('camera_pixel_to_level', '_drop_detection_fresh', '_drop_hold_target',
                 'DROP_DETECT_MAX_AGE', 'MISSION_REVISION'):
        assert need in s, f'[{tag}] 缺少 {need}'
    return s


plan = []
for p in TARGETS:
    src = open(p, encoding='utf-8').read()
    assert 'camera_pixel_to_level' not in src, f'{p} 已移植过'
    plan.append((p, port(src, is_fly='fly/src' in p)))

for p, s in plan:
    if CHECK_ONLY:
        print('  [check ok]', os.path.relpath(p, ROOT))
    else:
        open(p, 'w', encoding='utf-8').write(s)
        print('  [written] ', os.path.relpath(p, ROOT))
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
