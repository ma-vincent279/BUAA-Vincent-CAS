#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 h 的"侦察区改动"移植到 f (2026-09-30, 用户指示)
① 平移限速: 子阶段 3/5 的位置阶跃 -> _scout_pos_limited() (0.35m/s + 末段线性减速), SCOUT_SUB_TIMEOUT 10->14
② 判稳后计时: 子阶段 4/6 的"到位即计时" -> _scout_settled() (|vxy|<0.15 持续 0.5s 才开始 5s 计时, 8s 兜底)
只动 SCOUT 子阶段 3/4/5/6 + 常量/两个 helper; 投放判据/通道/补偿/选桶 一律不碰 (f 保持实飞稳定版本)。
"""
import re, shutil, datetime, sys

ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

CONST = '''SCOUT_MOVE_SPEED = 0.35   # (2026-09-30 自 h 移植) 侦察区平移峰值速度 (m/s): 3m 约 9s (原位置阶跃=全速冲刺->硬刹车)
SCOUT_MOVE_RAMP  = 0.50   # 末段线性减速距离 (m)
SCOUT_SETTLE_VMAX = 0.15  # 判稳门限: 水平速度 (m/s) (0.10 太紧, 实测悬停 |vxy| 本就 0.08~0.12)
SCOUT_SETTLE_HOLD = 0.50  # 判稳持续 (s), 之后才开始 SCOUT_PAUSE_TIME 计时
SCOUT_PAUSE_MAX  = 8.0    # 停留总时长上限 (s), 兜底'''

HELPERS = '''    def _scout_pos_limited(self, tx, ty, z, yaw):
        # (2026-09-30 自 h 移植) 侦察区平移限速: 把"位置阶跃"换成 SCOUT_MOVE_SPEED 等速平移 + 末段线性减速,
        # 避免 PX4 位置环全速冲刺/硬刹车造成俯视相机晃动。参考点每帧朝子目标推进, 再以位置 setpoint 发布。
        now = self.get_clock().now()
        key = (getattr(self, '_scout_sub', None), round(tx, 3), round(ty, 3))
        if getattr(self, '_scout_lim_key', None) != key:
            self._scout_lim_key = key
            self._scout_ref_x = self.x - self.home_x
            self._scout_ref_y = self.y - self.home_y
        dt = min(0.2, max(0.01, (now - getattr(self, '_scout_lim_t', now)).nanoseconds * 1e-9))
        self._scout_lim_t = now
        dx, dy = tx - self._scout_ref_x, ty - self._scout_ref_y
        rem = math.hypot(dx, dy)
        if rem > 1e-6:
            v = SCOUT_MOVE_SPEED * min(1.0, rem / max(0.05, SCOUT_MOVE_RAMP))
            step = min(rem, v * dt)
            self._scout_ref_x += dx / rem * step
            self._scout_ref_y += dy / rem * step
        self._pub_pos(self.home_x + self._scout_ref_x, self.home_y + self._scout_ref_y, z, yaw)

    def _scout_settled(self, sub_elapsed):
        # (2026-09-30 自 h 移植) 判稳后才开始 SCOUT_PAUSE_TIME 计时: 先要 |vxy| < SCOUT_SETTLE_VMAX 持续
        # SCOUT_SETTLE_HOLD, 之后才开始计时; 总时长超 SCOUT_PAUSE_MAX 兜底退出。
        now = self.get_clock().now()
        if getattr(self, '_scout_still_sub', None) != getattr(self, '_scout_sub', None):
            self._scout_still_sub = getattr(self, '_scout_sub', None)
            self._scout_still_start = None
        if math.hypot(self.vx, self.vy) < SCOUT_SETTLE_VMAX:
            if getattr(self, '_scout_still_start', None) is None:
                self._scout_still_start = now
        else:
            self._scout_still_start = None
        st = getattr(self, '_scout_still_start', None)
        if sub_elapsed > SCOUT_PAUSE_MAX:
            return True
        if st is None:
            return False
        return (now - st).nanoseconds * 1e-9 - SCOUT_SETTLE_HOLD > SCOUT_PAUSE_TIME


'''

RE_PUB = re.compile(r'self\._pub_pos\(self\.home_x \+ ([^,]+), self\.home_y \+ ([^,]+), ([^,]+), ([^)]+)\)')


def patch(t):
    lines = t.split('\n')
    i = [k for k, l in enumerate(lines) if l.startswith('SCOUT_PAUSE_TIME = 5.0')]
    assert len(i) == 1, 'SCOUT_PAUSE_TIME 锚点 %d' % len(i)
    lines.insert(i[0] + 1, CONST)
    j = [k for k, l in enumerate(lines) if l.startswith('SCOUT_SUB_TIMEOUT = 10.0')]
    assert len(j) == 1, 'SCOUT_SUB_TIMEOUT 锚点 %d' % len(j)
    lines[j[0]] = 'SCOUT_SUB_TIMEOUT = 14.0   # 2026-09-30: 10->14 (3m @0.35m/s ~ 8.6s, 防贴超时线)'
    h = [k for k, l in enumerate(lines) if re.match(r'^    def _scout_(target|edge_target)\(', l)]
    assert h, '未找到 _scout_target 定义'
    lines[h[0]:h[0]] = HELPERS.rstrip('\n').split('\n')
    t = '\n'.join(lines)
    for sub in (3, 5):
        pat = re.compile(r'(?ms)^(            if self\._scout_sub == %d:\n)(.*?)(?=^            if self\._scout_sub == %d:\n|^            # \u5b50\u9636\u6bb5|^        # ----)' % (sub, sub + 1))
        m = pat.search(t)
        assert m, 'sub==%d 块未找到' % sub
        blk2 = RE_PUB.sub(r'self._scout_pos_limited(\1, \2, \3, \4)', m.group(2))
        assert blk2 != m.group(2), 'sub==%d 未替换到限速发布' % sub
        t = t[:m.start(2)] + blk2 + t[m.end(2):]
        old = 'if sub_elapsed >= SCOUT_PAUSE_TIME:'
        assert t.count(old) >= 1, '未找到停留判据 (sub%d)' % (sub + 1)
        t = t.replace(old, 'if self._scout_settled(sub_elapsed):   # 2026-09-30 自 h: 判稳后才开始计时', 1)
    assert t.count('_scout_pos_limited(') >= 3 and t.count('_scout_settled(') >= 3
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/f.py', 'fly/src/fly/fly/f.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_scoutFromH')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
