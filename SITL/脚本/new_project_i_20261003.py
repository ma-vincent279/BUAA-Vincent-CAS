#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新建项目 i (2026-10-03): 从 f 脱胎, 低空段改为"一次性相对修正 + 定点保持 + 停留 5s + 必投"
设计要点(用户确认):
  ① 1.2m 处做一次相对修正:  S1 = S0 + (桶估计 - 飞机当前位置) + 两瓶补偿
     (代数: 位置环稳态偏差 b 近似固定 => P1 = S1 + b = B, b 自动抵消, 且无反馈 => 不会自激)
  ② 停留 5s + |vxy| < 0.12 纯位置/速度判据 (相机只做只读日志, 不参与控制)
  ③ 失败也一定要投 (带载荷进侦察区=0分): 检测超时 -> 就地冻结并走同一释放流程
  ④ 下降段不做增量伺服 (避免俯冲段积分漂移), 只垂直下降
e/f/g/h/f_test 均不动。
"""
import os, re, sys, shutil, datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
S_SITL, S_FLY = os.path.join(ROOT, 'src/drone_drl/drone_drl/f.py'), os.path.join(ROOT, 'fly/src/fly/fly/f.py')
D_SITL, D_FLY = os.path.join(ROOT, 'src/drone_drl/drone_drl/i.py'), os.path.join(ROOT, 'fly/src/fly/fly/i.py')

CONST = '''# ===== 2026-10-03 项目 i: 低空"一次性相对修正 + 定点保持" =====
I_LOW_ALT_FREEZE = True    # True=i模式(低空不做闭环伺服); False=退回 f 原行为
I_DWELL_S        = 5.0     # 1.2m 处修正后停留时间 (s)
I_VMAX           = 0.15    # 释放判据: 水平速度上限 (m/s; 0.12 太贴实测悬停速度 0.08~0.12)
I_ARRIVE_M       = 0.15    # 到位门限: |飞机位置 - 冻结保持点| (m)
I_RELEASE_TIMEOUT = 20.0   # 从进 DROP 起的兜底 (s): 超时无论有没有检测都强制投放 (必投)
'''

FAST = '''            # ===== 2026-10-03 项目 i: 低空一次性相对修正 + 定点保持 + 停留 + 必投 =====
            # 设计: S1 = S0 + (桶估计 - 飞机当前位置) + 两瓶补偿. 因位置环稳态偏差 b 近似固定,
            # 飞机随后停在 S1 + b = 桶上 (b 自动抵消), 且全程无反馈 => 不会自激振荡.
            if I_LOW_ALT_FREEZE:
                drop_z = self.home_z - DROP_HEIGHT   # i模式: 本块先于原赋值执行, 自行定义(原处再赋一次同样的值)
                if not getattr(self, '_i_frozen', False):
                    _est = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est = self._roi_to_ned(self.latest_roi)
                    _forced = (sim_t - (self._i_entry_t or sim_t)) > I_RELEASE_TIMEOUT
                    if _est is not None or _forced:
                        _dy = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
                        _dx = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
                        _ddx = (_est[0] - (self.x - self.home_x)) if _est is not None else 0.0
                        _ddy = (_est[1] - (self.y - self.home_y)) if _est is not None else 0.0
                        self._i_hold_x = (self._servo_target_x + _ddx
                                          + _dy * math.sin(self.heading) - _dx * math.cos(self.heading))
                        self._i_hold_y = (self._servo_target_y + _ddy
                                          - _dy * math.cos(self.heading) - _dx * math.sin(self.heading))
                        self._i_frozen = True
                        self._i_freeze_t = sim_t
                        self._i_arrive_t = None
                        self.get_logger().warn(
                            f"i模式: 1.2m 一次性修正 delta=({_ddx:+.3f},{_ddy:+.3f})m "
                            f"(桶估计={_est}) → 冻结保持点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) "
                            f"补偿B{self._drop_count + 1}=({_dy:+.3f},{_dx:+.3f}){' [兜底:无检测]' if _est is None else ''}")
                    else:
                        self._pub_pos(self.x, self.y, drop_z, self.heading)   # 等检测: 原地保持
                        return
                if not self._drop_triggered:
                    self._pub_pos(self.home_x + self._i_hold_x, self.home_y + self._i_hold_y, drop_z, self.heading)
                    _dwell = sim_t - self._i_freeze_t
                    _vxy = math.hypot(self.vx, self.vy)
                    _err = math.hypot(self.x - self.home_x - self._i_hold_x,
                                      self.y - self.home_y - self._i_hold_y)
                    if _err < I_ARRIVE_M and getattr(self, '_i_arrive_t', None) is None:
                        self._i_arrive_t = sim_t
                        self.get_logger().warn(f"i模式: 已到达保持点 err={_err * 100:.1f}cm → 开始停留 {I_DWELL_S:.0f}s")
                    _settled = (getattr(self, '_i_arrive_t', None) is not None
                                and (sim_t - self._i_arrive_t) >= I_DWELL_S)
                    _deadline = _dwell >= I_RELEASE_TIMEOUT
                    if (_settled and _vxy < I_VMAX) or _deadline:
                        self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()
                        self._served_targets.append((self._i_hold_x, self._i_hold_y))
                        self._drop_hold_target = (self.home_x + self._i_hold_x, self.home_y + self._i_hold_y,
                                                  drop_z, self.heading)
                        _dv = AUX5_DROP if self._drop_count == 0 else AUX5_DROP2
                        self._aux5_value = _dv
                        self._drop_count += 1
                        self.impact_pub.publish(Point(x=float(self.x), y=float(self.y), z=float(self.home_z)))
                        self._drop_records.append({
                            "idx": self._drop_count,
                            "aim": (self.home_x + self._i_hold_x, self.home_y + self._i_hold_y),
                            "pos": (self.x, self.y), "H": self.home_z - self.z,
                            "last_err": self._last_drop_err, "tier": "i",
                            "conv_m": math.hypot(self.x - self.home_x - self._i_hold_x,
                                                 self.y - self.home_y - self._i_hold_y),
                            "conv_min": None, "pred_m": None, "t_eff": None,
                            "vxy": _vxy, "reappr": 0, "actual_impact": None,
                        })
                        self.get_logger().info(
                            f">>> DROP #{self._drop_count}! PWM={1500 + _dv * 600:.0f}us "
                            f"NED({self.x:.3f},{self.y:.3f},{self.z:.3f}) 触发方式=i模式(1.2m一次性修正 "
                            f"到位err={_err * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s "
                            f"hold=({self._i_hold_x:+.3f},{self._i_hold_y:+.3f})"
                            f"{' [兜底:未到位强制投放]' if _deadline and not _settled else ''})")
                    return
            if getattr(self, '_i_frozen', False) and not self._drop_triggered:
                return    # 已冻结: 后续由上方 i 分支处理 (防止落入原闭环逻辑)

'''

# 进 DROP 时记录进入时刻 (i 模式的兜底计时用)
ENTRY_OLD = "        self._drop_triggered = False; self._drop_signal_start = None\n        self._drop_hold_target = None\n"
ENTRY_NEW = ("        self._drop_triggered = False; self._drop_signal_start = None\n"
             "        self._drop_hold_target = None\n"
             "        if step == STEP_DROP:      # 2026-10-03 i: 记录进入时刻(必投兜底计时) + 复位冻结\n"
             "            self._i_entry_t = self.get_clock().now().nanoseconds * 1e-9\n"
             "            self._i_frozen = False\n")

INC_OLD = ("                    self._servo_target_x += dx_ned\n"
           "                    self._servo_target_y += dy_ned\n")
INC_NEW = ("                    if not I_LOW_ALT_FREEZE:   # 2026-10-03 i: 下降段不做增量伺服(避免俯冲漂移)\n"
           "                        self._servo_target_x += dx_ned\n"
           "                        self._servo_target_y += dy_ned\n")

ANCHOR = ("            drop_z = self.home_z - DROP_HEIGHT\n"
          "            if self._servo_active and self._drop_detection_fresh():\n")


def patch(t):
    lines = t.split('\n')
    i = [k for k, l in enumerate(lines) if l.startswith('DROP_DETECT_MAX_AGE = 0.5')]
    assert len(i) == 1, '常量锚点 %d' % len(i)
    lines.insert(i[0] + 1, CONST.rstrip('\n'))
    t = '\n'.join(lines)
    assert t.count(ENTRY_OLD) == 1, 'ENTRY 锚点 %d' % t.count(ENTRY_OLD)
    t = t.replace(ENTRY_OLD, ENTRY_NEW, 1)
    _m = re.search(r'([ \t]*)self\._servo_target_x \+= dx_ned\n[ \t]*self\._servo_target_y \+= dy_ned\n', t)
    assert _m and len(re.findall(r'self\._servo_target_x \+= dx_ned', t)) == 1, 'INC 锚点未命中/多次'
    _ind = _m.group(1)
    t = (t[:_m.start()]
         + _ind + "if not I_LOW_ALT_FREEZE:   # 2026-10-03 i: 下降段不做增量伺服(避免俯冲漂移)\n"
         + _ind + "    self._servo_target_x += dx_ned\n"
         + _ind + "    self._servo_target_y += dy_ned\n"
         + t[_m.end():])
    assert t.count(ANCHOR) == 1, 'DROP 分支锚点 %d' % t.count(ANCHOR)
    t = t.replace(ANCHOR, '\n'.join(FAST.split('\n')[:-1]) + '\n' + ANCHOR, 1)
    for must in ('I_LOW_ALT_FREEZE', '_i_hold_x', '触发方式=i模式'):
        assert must in t, '缺少 %s' % must
    return t


def rename(t):
    n1 = len(re.findall(r'\bFMission\b', t)); t = re.sub(r'\bFMission\b', 'IMission', t)
    n2 = len(re.findall(r"'f_mission'", t)); t = t.replace("'f_mission'", "'i_mission'")
    assert n1 >= 2 and n2 == 1, 'rename %d/%d' % (n1, n2)
    m = re.search(r'MISSION_REVISION = "([^"]+)"', t)
    t = t.replace(m.group(0), 'MISSION_REVISION = "%s-i"' % m.group(1))
    return t.replace('子项目 f —', '子项目 i — (f 副本; 低空改为一次性相对修正+定点保持+停留5s+必投)')


def main():
    for src, dst in ((S_SITL, D_SITL), (S_FLY, D_FLY)):
        t = open(src, encoding='utf-8').read()
        new = patch(rename(t))
        if CHECK:
            print('OK(check)', dst); continue
        if os.path.exists(dst):
            shutil.copy2(dst, dst + '.bak.' + ST)
        open(dst, 'w', encoding='utf-8').write(new); print('写出', dst)
    if CHECK:
        return
    w = os.path.join(ROOT, 'worlds/i.world')
    if not os.path.exists(w):
        shutil.copy2(os.path.join(ROOT, 'worlds/f.world'), w); print('+', w)
    sg, sd = os.path.join(ROOT, 'scripts/test_f.sh'), os.path.join(ROOT, 'scripts/test_i.sh')
    if not os.path.exists(sd):
        s = open(sg, encoding='utf-8').read()
        s = s.replace('test_f.sh', 'test_i.sh').replace('子项目 f —', '子项目 i —')
        s = s.replace('f.world', 'i.world').replace('/tmp/f_output.log', '/tmp/i_output.log')
        s = s.replace('/tmp/f_judge.log', '/tmp/i_judge.log')
        s = s.replace('ros2 run drone_drl f ', 'ros2 run drone_drl i ')
        s = s.replace('f_mission', 'i_mission').replace('f_judge', 'i_judge')
        open(sd, 'w', encoding='utf-8').write(s); os.chmod(sd, 0o755); print('+', sd)
    fg, fd = os.path.join(ROOT, 'fly/f.sh'), os.path.join(ROOT, 'fly/i.sh')
    if not os.path.exists(fd):
        s = open(fg, encoding='utf-8').read()
        s = s.replace('f_mission', 'i_mission').replace('fly/f.py', 'fly/i.py')
        open(fd, 'w', encoding='utf-8').write(s); os.chmod(fd, 0o755); print('+', fd)
    for sp, pat in ((os.path.join(ROOT, 'src/drone_drl/setup.py'), "'f = drone_drl.f:main',"),
                    (os.path.join(ROOT, 'fly/src/fly/setup.py'), "'f = fly.f:main',")):
        s = open(sp, encoding='utf-8').read()
        i = s.find(pat); assert i >= 0, 'setup 锚点 %s' % pat
        line = s[i:s.find('\n', i)]
        newline = line.replace("'f = ", "'i = ").replace('.f:main', '.i:main')
        if newline.strip() not in s:
            indent = line[:len(line) - len(line.lstrip())]
            shutil.copy2(sp, sp + '.bak.' + ST)
            open(sp, 'w', encoding='utf-8').write(s.replace(line, line + '\n' + indent + newline.strip(), 1))
            print('+入口', sp)


if __name__ == '__main__':
    main(); print('\n完成' if not CHECK else '\n[--check] 未写盘')
