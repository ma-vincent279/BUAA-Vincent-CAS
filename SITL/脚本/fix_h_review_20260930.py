#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""h 审查修复 (2026-09-30) — 全部改为"单行替换/单行插入", 避免缩进风险
F1 _enter_step 复位 _reappr_start (审查Bug1: 第2桶重进空转)
F2 _dt_aim 0.05 -> 0.25 (审查Bug2: 实际块频率远低于 20Hz, 写死 0.05 会削弱 τ/限速)
F3 重进不抢断正在确认的档位 (审查3): 加 _drop_tier is None / 超窗 2s 放宽
F4 _drop_hist 注释与真实频率对齐 (审查4)
F5 死代码 (审查5): 删 _drop_pred_start; _release_cmd_t 接入落点记录
"""
import shutil, datetime, sys

ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
REPL = [
    ("        self._drop_hold_target = None\n",
     "        self._drop_hold_target = None\n        self._reappr_start = None      # F1: 复位重进计时\n"),
    ("                        _dt_aim = 0.05\n",
     "                        _dt_aim = 0.25   # F2: 本块仅在\"检测新鲜\"帧运行, 0.05 会削弱 τ/限速数倍\n"),
    ("                    and tier != 2):\n",
     "                    and tier != 2 and (self._drop_tier is None or self._step_elapsed() > DROP_PREFER_WINDOW + 2.0)):   # F3\n"),
    ("                while len(self._drop_hist) > 40:\n",
     "                while len(self._drop_hist) > 40:   # F4: 仅新检测帧追加(~2Hz), 40条≈20s\n"),
    ("        self._drop_pred_start = None          # 预测判据确认计时起点\n", ""),
    ("            self._drop_pred = None; self._drop_pred_start = None\n", "            self._drop_pred = None\n"),
    ('                    "t_eff": self._release_teff,\n',
     '                    "t_eff": self._release_teff,\n                    "t_cmd": self._release_cmd_t,   # F5b\n'),
]


def patch(t):
    for old, new in REPL:
        assert t.count(old) >= 1, '未命中: %r' % old[:50]
        t = t.replace(old, new, 1)
    assert '_drop_pred_start' not in t and t.count('_reappr_start = None') >= 2
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/h.py', 'fly/src/fly/fly/h.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_review2')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
