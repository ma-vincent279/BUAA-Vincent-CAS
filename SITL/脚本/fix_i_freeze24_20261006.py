#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 改进 (2026-10-06 晚): 把"平静采样 + 一次性修正"提前到下降段 (H<=2.4m) 执行
目的: 1.2m 处不再出现 0.3~0.5m 的"动一下"; 冻结后下降 = 纯垂直, DROP 段只做 保持+到位+停留+释放。
实现: 在 STEP_VISUAL_SERVO 里插入同一套冻结逻辑(自包含), 冻结后只按冻结点下降;
      DROP 段原有 i 分支看到 _i_frozen=True 就直接走保持/停留/释放, 不会重复冻结。
"""
import shutil, datetime, sys
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

C_OLD = "I_SETTLE_MAX     = 5.0     # 先停稳: 最长等待 (s), 超时照常进入采样"
C_NEW = (C_OLD + "\n"
         "I_FREEZE_AT_H    = 2.4     # 提前冻结高度 (m): 降到此高度即做平静采样+一次性修正(1.2m 处不再挪动)")

A_OLD = "            drop_z = self.home_z - DROP_HEIGHT\n            target_z = max(drop_z, self.z - SERVO_DESCENT_SPEED * 0.05)\n"
A_NEW = A_OLD + '''            # 2026-10-06: 提前冻结 (i) —— 在下降段就把"瞄准点一次性修正"做完, 之后纯垂直下降到 1.2m
            if I_LOW_ALT_FREEZE and not getattr(self, '_i_frozen', False):
                _dh = self.home_z - self.z
                if _dh <= I_FREEZE_AT_H:
                    _est2 = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est2 = self._roi_to_ned(self.latest_roi)
                    _calm2 = (abs(self._roll) < math.radians(I_CALM_TILT)
                              and abs(self._pitch) < math.radians(I_CALM_TILT)
                              and math.hypot(self.vx, self.vy) < I_CALM_VXY)
                    if not hasattr(self, '_i_samp'):
                        self._i_samp = []
                    if _est2 is not None and _calm2:
                        self._i_samp.append(_est2)
                    if len(self._i_samp) >= I_CALM_SAMPLES:
                        _xs2 = sorted(p[0] for p in self._i_samp)
                        _ys2 = sorted(p[1] for p in self._i_samp)
                        _em = (_xs2[len(_xs2) // 2], _ys2[len(_ys2) // 2])
                        _dy2 = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
                        _dx2 = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
                        _ddx2 = _em[0] - (self.x - self.home_x)
                        _ddy2 = _em[1] - (self.y - self.home_y)
                        self._i_hold_x = (self._servo_target_x + _ddx2
                                          + _dy2 * math.sin(self.heading) - _dx2 * math.cos(self.heading))
                        self._i_hold_y = (self._servo_target_y + _ddy2
                                          - _dy2 * math.cos(self.heading) - _dx2 * math.sin(self.heading))
                        self._i_frozen = True
                        self._i_freeze_t = sim_t
                        self._i_arrive_t = None
                        self._i_nsamp = len(self._i_samp)
                        self._i_mode = "提前冻结@%.1fm(中值%d帧)" % (_dh, self._i_nsamp)
                        self.get_logger().warn(
                            f"i模式: 下降段 %.1fm 处一次性修正 delta=({_ddx2:+.3f},{_ddy2:+.3f})m "
                            f"→ 冻结点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) 补偿B{self._drop_count + 1}"
                            f"=({_dy2:+.3f},{_dx2:+.3f}) [纯垂直下降]")
                    if not self._i_frozen:
                        self._pub_pos(self.x, self.y, target_z, self.heading)   # 等平静样本: 原地保持
                        return
'''


def patch(t):
    for old, new in ((C_OLD, C_NEW), (A_OLD, A_NEW)):
        assert t.count(old) == 1, '锚点 %d: %r' % (t.count(old), old[:60])
        t = t.replace(old, new, 1)
    assert 'I_FREEZE_AT_H' in t and '提前冻结@' in t
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/i.py', 'fly/src/fly/fly/i.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_freeze24')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
