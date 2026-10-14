#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侦察区再简化 (2026-10-09 用户要求): 去掉左右移动与两次停留, "在区内待 10s 后直接返航"

用户原话: "本来有个左移1.5, 然后右移动3, 再返航, 改成到达侦察区后去掉左右移动直接飞回,
          就是一共在侦察区待10秒就返航"

新时序 (7 子阶段 → 3 子阶段):
  sub0  速度控制直飞**侦察区近边 57m**        (未动, 防过冲)
  sub1  原地**下降 1m**                        (未动)
  sub2  **10s 缓移**到侦察区中心 59.5m ⇒ **直接返航**   ← 这 10s 就是"在侦察区待 10s"
  (删除) sub3 左移 1.5m / sub4 停 5s / sub5 右移 3m / sub6 停 5s

预期: 侦察段 实测 51.7s (10-08 09:34 实飞) → 约 25s (13.7 进场 + 1 降高 + 10 缓移)。
做法: sub2 完成时写 summary 记账 (`_scout_done` / `_scout_elapsed`) 后 `_enter_step(STEP_RETURN)`;
      整段 sub3~sub6 原样删除 (锚点文本从参考文件抽取, 逐文件断言唯一命中)。
目标: i/j/k/l × 两棵树 = 8 份 (侦察代码共享; k 不飞侦察区, 只为与 l 保持一致)。
不动的常量: SCOUT_LEFT_DIST/SCOUT_RIGHT_DIST/SCOUT_PAUSE_TIME (保留定义, 已无调用)。

用法: python3 scripts/patch_scout_return10s_20261009.py --check | (无参数落盘)
回退: python3 scripts/patch_scout_return10s_20261009.py --rollback   (用最近一次 _scout10s 备份还原)
"""
import datetime
import glob
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
ROLLBACK = '--rollback' in sys.argv
REF = os.path.join(ROOT, 'fly/src/fly/fly/i.py')
TARGETS = [os.path.join(ROOT, t, n) for t in ('src/drone_drl/drone_drl', 'fly/src/fly/fly')
           for n in ('i.py', 'j.py', 'k.py', 'l.py')]

SUB2_OLD = ('                if sub_elapsed >= SCOUT_SLOW_MOVE_TIME:\n'
            '                    self._scout_sub = 3; self._scout_sub_start = self.get_clock().now()\n'
            '                    self.get_logger().info("Scout: slow move done, moving left 1.5m")\n')
SUB2_NEW = ('                if sub_elapsed >= SCOUT_SLOW_MOVE_TIME:\n'
            '                    # 2026-10-09 用户要求: 去掉左右移动 + 两次停留, "在区内待 10s 后直接返航"\n'
            '                    #   (这 10s = 从近边缓移到中心的这一段)\n'
            '                    self._scout_done = True                    # 写入 MISSION SUMMARY\n'
            '                    self._scout_elapsed = self._step_elapsed()\n'
            '                    self.get_logger().info(f"Scout maneuver complete "\n'
            '                                           f"({self._scout_elapsed:.1f}s, 飞近边+降1m+10s缓移), returning home")\n'
            '                    self._enter_step(STEP_RETURN)\n')
DEL_NOTE = ('            # 2026-10-09 已删除: 子阶段3/4/5/6 (左移1.5m / 停5s / 右移3m / 停5s)\n'
            '            #   用户要求"去掉左右移动, 在侦察区待10s就返航" ⇒ sub2 结束即返航\n')


def region(txt):
    """抽取 sub3~sub6 的原文 (以参考文件为准)"""
    i = txt.index('            if self._scout_sub == 3:\n')
    j = txt.index('                self._enter_step(STEP_RETURN)\n', i) + len('                self._enter_step(STEP_RETURN)\n')
    return txt[i:j]


def do_rollback():
    n = 0
    for p in TARGETS:
        baks = sorted(glob.glob(p + '.bak.*_scout10s'))
        if not baks:
            continue
        shutil.copy2(baks[-1], p)
        n += 1
        print(f'  ↩ {os.path.relpath(p, ROOT)} (从 {os.path.basename(baks[-1])} 还原)')
    print(f'回退完成 ({n} 份)。板载需重新部署。')
    return 0


def main():
    if ROLLBACK:
        return do_rollback()
    print('=== patch_scout_return10s_20261009 (侦察区: 去左右移动, 区内 10s 即返航) ===')
    ref = open(REF, encoding='utf-8').read()
    DEL_OLD = region(ref)
    # 参考文件本身已改过 ⇒ 用备份重建
    refbaks = sorted(glob.glob(REF + '.bak.*'))
    for b in reversed(refbaks):
        t = open(b, encoding='utf-8').read()
        if '            if self._scout_sub == 3:\n' in t and '                self._enter_step(STEP_RETURN)\n' in t:
            DEL_OLD = region(t)
            break
    print(f'  待删除段落 {len(DEL_OLD.splitlines())} 行 (sub3~sub6)')
    todo, skipped, bad = [], [], []
    for p in TARGETS:
        if not os.path.exists(p):
            continue
        t = open(p, encoding='utf-8').read()
        rel = os.path.relpath(p, ROOT)
        if 'sub3/4/5/6' in t and SUB2_OLD not in t:
            skipped.append(rel)
            continue
        if t.count(SUB2_OLD) != 1 or t.count(DEL_OLD) != 1:
            bad.append(f'{rel} (sub2锚点 {t.count(SUB2_OLD)}, 删除段锚点 {t.count(DEL_OLD)})')
            continue
        todo.append((rel, t))
    print(f'  待改 {len(todo)} / 已符合 {len(skipped)} / 锚点失败 {len(bad)}')
    if bad:
        print('  锚点失败: ' + '; '.join(bad))
        return 1
    if not todo:
        print('  无需改动')
        return 0
    if CHECK:
        print('  --check: 只校验, 未落盘')
        return 0
    for rel, t in todo:
        p = os.path.join(ROOT, rel)
        shutil.copy2(p, f'{p}.bak.{ST}_scout10s')
        t = t.replace(SUB2_OLD, SUB2_NEW, 1).replace(DEL_OLD, DEL_NOTE, 1)
        open(p, 'w', encoding='utf-8').write(t)
        print(f'  ✓ {rel} (备份 {os.path.basename(p)}.bak.{ST}_scout10s)')
    print('\n完成: 侦察区 = 飞近边 → 降1m → 10s缓移到中心 → 直接返航')
    print('回退: python3 scripts/patch_scout_return10s_20261009.py --rollback')
    return 0


if __name__ == '__main__':
    sys.exit(main())
