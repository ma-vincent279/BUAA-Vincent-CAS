#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 最后的首投段改进 (2026-10-06):
① I_ARRIVE_M 0.25 -> 0.35 (首投段实测就在 25cm 附近徘徊, 让它能走正常路径, 由 pred 把关)
② 20s 兜底不再"到时就投", 而是到期后在 I_DEADLINE_WIN=5s 窗口内挑 pred 最小(或首次优于最优 5%)的时刻投
   —— 仍保证必投(窗口结束必发), 但不会挑最差瞬间。
"""
import shutil, datetime, sys
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

C_OLD = "I_ARRIVE_M       = 0.25    # 到位门限 (m): 0.15 比位置环自身稳态偏差(10~26cm)还严, 会永远到不了 -> 放宽, 准不准交给 pred"
C_NEW = ("I_ARRIVE_M       = 0.35    # 到位门限 (m): 首投段实测在 25cm 附近徘徊 -> 0.25→0.35, 准不准仍由 pred 把关\n"
         "I_DEADLINE_WIN   = 5.0     # 20s 兜底后的'挑好时机'窗口 (s): 窗口内 pred 最优(或优于最优5%)即投, 窗口结束必投")

R_OLD = """                    if (_settled and _vxy < I_VMAX and _pred_ok) or _deadline:"""
R_NEW = """                    # ② 兜底挑时机: 到 20s 后不立刻投, 先在 I_DEADLINE_WIN 内等 pred 较优的一刻(窗口结束必投)
                    if _deadline and getattr(self, '_i_dl_t0', None) is None:
                        self._i_dl_t0 = sim_t
                        self._i_dl_best = _pred
                    if _deadline:
                        self._i_dl_best = min(getattr(self, '_i_dl_best', _pred), _pred)
                    _dl_pick = bool(_deadline and (
                        sim_t - getattr(self, '_i_dl_t0', sim_t) >= I_DEADLINE_WIN
                        or _pred <= max(0.03, getattr(self, '_i_dl_best', 9.9) * 1.05)))
                    if (_settled and _vxy < I_VMAX and _pred_ok) or _dl_pick:"""


def patch(t):
    for old, new in ((C_OLD, C_NEW), (R_OLD, R_NEW)):
        assert t.count(old) == 1, '锚点 %d: %r' % (t.count(old), old[:50])
        t = t.replace(old, new, 1)
    assert 'I_DEADLINE_WIN' in t and '_dl_pick' in t
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/i.py', 'fly/src/fly/fly/i.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_dlwin')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
