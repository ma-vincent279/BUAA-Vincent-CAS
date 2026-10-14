#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侦察区动作回滚到 c 世代 (2026-10-09) —— 取消 2026-09-30 自 h 移植的"平移限速 + 判稳门控"

用户反馈 (2026-10-09): 侦察区动作**太慢**, 且**有点飞过**。
现场实测 (10-08 09:34 那趟 i, 侦察总 51.7s) 的分段账:
    直飞近边 ~13.7s / 降 1m 1s / 10s 缓移 10.0s / **左移 1.5m 5s** / 停 6s / **右移 3m 9s** / 停 7s
其中"左移 5s、右移 9s、两次停各多 1~2s"全部来自 h 移植的两个机制:
    `_scout_pos_limited`  (SCOUT_MOVE_SPEED=0.35 + SCOUT_MOVE_RAMP=0.5 ⇒ 3m 要 ~9s)
    `_scout_settled`      (判稳才计时 ⇒ 每次停多等 1~2s)
回滚后 (c 世代 = 直接位置设定点 + 到点即计时), 预计侦察总时长 **~52s → ~38s (−27%)**。

改动 (每个文件 3 处代码 + 1 处注释, 只碰侦察段; 投放链/门控/标定一字不动):
  P1 (×2) `self._scout_pos_limited(tx, ty, scout_z, yaw_locked)` → `self._pub_pos(home_x+tx, home_y+ty, scout_z, yaw_locked)`
  P2 (×2) `if self._scout_settled(sub_elapsed): ...`                → `if sub_elapsed >= SCOUT_PAUSE_TIME:`
  P3 (×1) `SCOUT_SUB_TIMEOUT = 14.0`                                → `10.0` (c 世代值)
  P4 (×1) 给已不再使用的 5 个 h 常量加注释 (保留定义: k/l 常量一致性检查会比对它们)

目标文件 (共享侦察代码, 统一回滚以保持家族一致): i/j/k/l × 两棵树 = 8 份。
**不动**: f/f_test/h (历史版本, 按项目约定冻结) 与 c/d/e/g (c 是回滚基准)。
注: k 不飞侦察区 (全返航), 这里只是与 l 保持代码一致 (check_drop_portability 要求)。

用法:
  python3 scripts/patch_scout_c_rollback_20261009.py --check   # 只校验锚点
  python3 scripts/patch_scout_c_rollback_20261009.py           # 落盘 (自动备份)
回退: cp <文件>.bak.<时间戳>_scout_c <文件>   (8 份都要还原)
"""
import datetime
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
TREES = ['src/drone_drl/drone_drl', 'fly/src/fly/fly']
NAMES = ['i.py', 'j.py', 'k.py', 'l.py']

TARGETS = [os.path.join(ROOT, t, n) for t in TREES for n in NAMES]

UNUSED_NOTE = ("# 2026-10-09 侦察回滚到 c 世代: 以下 SCOUT_MOVE_SPEED/RAMP + SCOUT_SETTLE_*/PAUSE_MAX\n"
               "#   已**不再被侦察段调用** (保留定义: check_drop_portability 会比对 k/l 的常量集)\n")

# (正则, 替换, 说明, 期望命中数)
PATCHES = [
    (r'self\._scout_pos_limited\(tx, ty, scout_z, yaw_locked\)',
     'self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)'
     '   # c 世代: 直接位置设定点 (取消 h 移植的平移限速)',
     'P1 左/右移取消限速', 2),
    (r'if self\._scout_settled\(sub_elapsed\):[^\n]*',
     'if sub_elapsed >= SCOUT_PAUSE_TIME:   # c 世代: 到点即计时 (取消 h 移植的判稳门控)',
     'P2 停 5s 取消判稳门控', 2),
    (r'SCOUT_SUB_TIMEOUT = 14\.0[^\n]*',
     'SCOUT_SUB_TIMEOUT = 10.0   # 2026-10-09 回滚 c 世代 (14→10: 直移 1~3s 到位, 不再需要长超时)',
     'P3 子动作超时 14→10', 1),
    (r'(?<!# )SCOUT_MOVE_SPEED = 0\.35[^\n]*\n',
     lambda m: UNUSED_NOTE + m.group(0),
     'P4 标注已停用的 h 常量', 1),
]


def apply(txt, name, check_only):
    for rx, rep, desc, n in PATCHES:
        hits = re.findall(rx, txt)
        if len(hits) != n:
            print(f'  ✗ {name}: {desc} 命中 {len(hits)} 次 (应为 {n})')
            return None
        if not check_only:
            txt = re.sub(rx, rep, txt, count=n)
    return txt


def already_done(txt):
    """幂等判据: 没有 _scout_pos_limited 调用 且 没有 _scout_settled 调用 且 超时已是 10.0"""
    return ('_scout_pos_limited(tx' not in txt
            and 'if self._scout_settled(sub_elapsed)' not in txt
            and re.search(r'^SCOUT_SUB_TIMEOUT = 10\.0', txt, re.M) is not None)


def main():
    print('=== patch_scout_c_rollback_20261009 (侦察回滚到 c 世代) ===')
    todo, done, bad = [], [], []
    for p in TARGETS:
        if not os.path.exists(p):
            continue
        t = open(p, encoding='utf-8').read()
        rel = os.path.relpath(p, ROOT)
        if already_done(t):
            done.append(rel)
            continue
        if apply(t, os.path.basename(p), True) is None:
            bad.append(rel)
            continue
        todo.append((rel, t))
    print(f'\n待改 {len(todo)} 份 / 已符合 {len(done)} 份 / 锚点失败 {len(bad)} 份')
    for r in done:
        print(f'  · {r} 已回滚, 跳过')
    if bad:
        print('锚点校验失败 ⇒ 不落盘: ' + ', '.join(bad))
        return 1
    if not todo:
        print('无需改动')
        return 0
    if CHECK:
        print('--check: 只校验 (未落盘)')
        return 0
    for rel, t in todo:
        p = os.path.join(ROOT, rel)
        shutil.copy2(p, f'{p}.bak.{ST}_scout_c')
        open(p, 'w', encoding='utf-8').write(apply(t, os.path.basename(p), False))
        print(f'  ✓ {rel}  (备份 {os.path.basename(p)}.bak.{ST}_scout_c)')
    print(f'\n完成: 侦察段已回到 c 世代 (直接位置设定点 + 到点即计时 + 超时 10s)')
    print('回退: ' + ' && '.join(f'cp {r}.bak.{ST}_scout_c {r}' for r, _ in
                                 [(r, None) for r, _ in todo]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
