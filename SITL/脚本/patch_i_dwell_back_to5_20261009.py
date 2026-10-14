#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回滚 i 的"停留 5→2s"，保留"横向补偿右移 3cm" (2026-10-09 用户决定)

用户原话: "对于侦察区保留, 对于投放, 保留向右3cm, 去掉另一个, 回滚"
⇒ 投放类改动有两项: ①横向补偿右移 3cm (`Y_B1 +0.18→+0.15` / `Y_B2 0.00→−0.03`) ②停留 `I_DWELL_S 5.0→2.0`
⇒ **保留 ①, 回滚 ②** (侦察区简化那项按"保留"处理, 不动)。

改动 (两份 i.py, 只 1 行):
  I_DWELL_S  2.0 → 5.0   (恢复原值; 其余常量不动)

为什么回滚停留 (数据留档): 实测"到位后残差"+2s 最小(~5cm)、停留期 dev 以 ~5.3cm/s 增长 ⇒ 本预期省 8~15cm;
但用户决定先只试 3cm 这一项 (变量单一, 便于归因) ⇒ 停留回 5s ✓

用法: python3 scripts/patch_i_dwell_back_to5_20261009.py [--check | --rollback]
"""
import datetime
import glob
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
ROLLBACK = '--rollback' in sys.argv
TARGETS = [os.path.join(ROOT, t, 'i.py') for t in ('src/drone_drl/drone_drl', 'fly/src/fly/fly')]

OLD = re.compile(r'^I_DWELL_S\s*=\s*2\.0[^\n]*', re.M)
NEW = ('I_DWELL_S        = 5.0     # 2026-10-09: 停留回滚到 5s (用户决定: 只保留"横向补偿右移 3cm"这一项试飞)'
       '\n#                              注: 缩短停留的实测依据 (到位后 +2s 残差最小 ~5cm / dev ~5.3cm/s) 见追加 40')


def main():
    if ROLLBACK:
        n = 0
        for p in TARGETS:
            baks = sorted(glob.glob(p + '.bak.*_dwell5'))
            if baks:
                shutil.copy2(baks[-1], p); n += 1
                print(f'  ↩ {os.path.relpath(p, ROOT)}')
        print(f'回退完成 ({n} 份)')
        return 0
    print('=== patch_i_dwell_back_to5_20261009 (停留回滚 2s→5s, 保留 3cm) ===')
    todo, ok, bad = [], [], []
    for p in TARGETS:
        t = open(p, encoding='utf-8').read()
        rel = os.path.relpath(p, ROOT)
        if re.search(r'^I_DWELL_S\s*=\s*5\.0', t, re.M) and '停留回滚到 5s' in t:
            ok.append(rel); continue
        if len(OLD.findall(t)) != 1:
            bad.append(rel); continue
        todo.append((rel, t))
    print(f'  待改 {len(todo)} / 已符合 {len(ok)} / 锚点失败 {len(bad)}')
    if bad:
        print('  锚点失败 (找不到 I_DWELL_S = 2.0): ' + ', '.join(bad))
        return 1
    if not todo:
        print('  无需改动'); return 0
    if CHECK:
        print('  --check: 只校验, 未落盘'); return 0
    for rel, t in todo:
        p = os.path.join(ROOT, rel)
        shutil.copy2(p, f'{p}.bak.{ST}_dwell5')
        open(p, 'w', encoding='utf-8').write(OLD.sub(NEW, t, count=1))
        print(f'  ✓ {rel} (备份 {os.path.basename(p)}.bak.{ST}_dwell5)')
    print('\n完成: I_DWELL_S 回到 5.0; Y_B1=+0.15 / Y_B2=-0.03 保留')
    print('回退: python3 scripts/patch_i_dwell_back_to5_20261009.py --rollback')
    return 0


if __name__ == '__main__':
    sys.exit(main())
