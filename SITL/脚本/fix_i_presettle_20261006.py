#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 改进: 先停稳再冻结 (2026-10-06, 针对首投段)
现象: 第 1 投进 1.2m 时仍带速度(vxy 0.21~0.235), 保持点附近绕 => 到不了位/pred 压不下来 => 20s 必投兜底。
改法: 进 DROP 后先对"当前位置"定点保持, 等 |vxy| < 0.10 持续 1s (最长 5s), 然后才做平静帧采样 + 一次性修正。
"""
import shutil, datetime, sys
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

C_OLD = "I_PRED_TEFF      = 0.70    # D: 指令->脱钩->落地 等效时间 (s)"
C_NEW = (C_OLD + "\n"
         "I_SETTLE_VXY     = 0.10    # 先停稳: 采样前要求 |vxy| 低于此值 (m/s)\n"
         "I_SETTLE_HOLD    = 1.0     # 先停稳: 持续时间 (s)\n"
         "I_SETTLE_MAX     = 5.0     # 先停稳: 最长等待 (s), 超时照常进入采样")

A_OLD = """                if not getattr(self, '_i_frozen', False):
                    if not hasattr(self, '_i_samp'):
                        self._i_samp = []"""
A_NEW = """                if not getattr(self, '_i_frozen', False):
                    if not hasattr(self, '_i_samp'):
                        self._i_samp = []
                    # 2026-10-06: 先停稳再冻结 —— 首投段进 1.2m 时常带 0.2m/s 速度,
                    # 直接冻结会把"还在漂"的状态定死 => 到不了位。先对当前位置保持, 等静下来。
                    _pkey = (self._drop_count, round(self._i_entry_t or 0.0, 2))
                    if getattr(self, '_i_pre_key', None) != _pkey:
                        self._i_pre_key = _pkey
                        self._i_pre_start = sim_t
                        self._i_settle_t = None
                    if math.hypot(self.vx, self.vy) < I_SETTLE_VXY:
                        if getattr(self, '_i_settle_t', None) is None:
                            self._i_settle_t = sim_t
                    else:
                        self._i_settle_t = None
                    _pre_ok = ((getattr(self, '_i_settle_t', None) is not None
                                and sim_t - self._i_settle_t >= I_SETTLE_HOLD)
                               or (sim_t - getattr(self, '_i_pre_start', sim_t) >= I_SETTLE_MAX))
                    if not _pre_ok:
                        self._pub_pos(self.x, self.y, drop_z, self.heading)   # 原地等静
                        return"""


def patch(t):
    for old, new in ((C_OLD, C_NEW), (A_OLD, A_NEW)):
        assert t.count(old) == 1, '锚点 %d: %r' % (t.count(old), old[:50])
        t = t.replace(old, new, 1)
    assert 'I_SETTLE_VXY' in t and '_pre_ok' in t
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/i.py', 'fly/src/fly/fly/i.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_presettle')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
