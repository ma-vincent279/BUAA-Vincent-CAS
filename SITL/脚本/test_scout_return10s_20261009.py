#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侦察区简化验收 (2026-10-09): 去掉左右移动与两次停留, "在区内待 10s 后直接返航"

A. 结构: 8 份文件里 sub3/4/5/6 **全不存在**; sub2 完成即 `_enter_step(STEP_RETURN)`; summary 记账保留
B. 常量: SCOUT_SLOW_MOVE_TIME=10.0 (那 10s 就是"在区内待 10s"); LEFT/RIGHT/PAUSE 仍有定义但侦察段零调用
C. 最小改动审计: 与 `*.bak.*_scout10s` 比 (剥行尾注释) ⇒ 只允许"sub2 完成块改写 + sub3~sub6 整段删除"
D. 时长预算: 51.7s (10-08 09:34 实飞) → ~25s (进场 13.7 + 降高 1 + 缓移 10)
"""
import difflib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PASS = FAIL = 0


def ck(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f'  ✓ {msg}')
    else:
        FAIL += 1
        print(f'  ✗ {msg}')


def strip(s):
    return [re.sub(r'\s*#.*$', '', l).rstrip() for l in s.split('\n') if re.sub(r'\s*#.*$', '', l).strip()]


def scout_block(path):
    L = path.read_text(encoding='utf-8').split('\n')
    i0 = [i for i, l in enumerate(L) if l.strip().startswith('elif self.step == STEP_SCOUT:')][-1]
    out = []
    for l in L[i0:]:
        if out and re.match(r'^        (elif self\.step|def |# =====|else:)', l):
            break
        out.append(l)
    return '\n'.join(strip('\n'.join(out)))


FILES = [ROOT / t / n for t in ('src/drone_drl/drone_drl', 'fly/src/fly/fly')
         for n in ('i.py', 'j.py', 'k.py', 'l.py')]

print('=== A. 结构 (8 份) ===')
for f in FILES:
    t = f.read_text(encoding='utf-8')
    blk = scout_block(f)
    ck(not re.search(r'if self\._scout_sub == [3456]:', blk), f'{f.parent.name}/{f.name}: sub3/4/5/6 已删除')
    ck('self._enter_step(STEP_RETURN)' in blk, f'{f.parent.name}/{f.name}: sub2 后直接返航')
    ck('self._scout_done = True' in blk and 'self._scout_elapsed' in blk,
       f'{f.parent.name}/{f.name}: MISSION SUMMARY 记账保留')
    ck('SCOUT_SLOW_MOVE_TIME' in blk, f'{f.parent.name}/{f.name}: 10s 缓移仍在 (它就是"区内待 10s")')
    ck('SCOUT_LEFT_DIST' not in blk and 'SCOUT_RIGHT_DIST' not in blk,
       f'{f.parent.name}/{f.name}: 侦察段已无左右移动调用')

print('\n=== B. 常量 ===')
for f in FILES[:1] + FILES[4:5]:
    t = f.read_text(encoding='utf-8')
    for k, v in (('SCOUT_SLOW_MOVE_TIME', '10.0'), ('SCOUT_LEFT_DIST', '1.5'), ('SCOUT_RIGHT_DIST', '3.0')):
        m = re.search(rf'^{k}\s*=\s*([\d.]+)', t, re.M)
        ck(m and m.group(1) == v, f'{f.name}: {k} = {m.group(1) if m else "缺"} (={v}, 保留定义)')

print('\n=== C. 最小改动审计 (vs _scout10s 备份) ===')
try:
    sys.path.insert(0, str(ROOT / 'scripts'))
    from _later_patch_marks import LATER_PATCH_MARKERS_RE as LR
except Exception:
    LR = ()
# 允许的新增行 = 新 sub2 完成块 (剥注释后逐字); 允许的删除行 = 旧 sub3~sub6 + 旧 sub2 尾巴两行
ADD_EXPECT = {
    'self._scout_done = True',
    'self._scout_elapsed = self._step_elapsed()',
    'self.get_logger().info(f"Scout maneuver complete "',
    'f"({self._scout_elapsed:.1f}s, 飞近边+降1m+10s缓移), returning home")',
    'self._enter_step(STEP_RETURN)',
}


def block_lines(path):
    L = path.read_text(encoding='utf-8').split('\n')
    i0 = [i for i, l in enumerate(L) if l.strip().startswith('elif self.step == STEP_SCOUT:')][-1]
    out = []
    for l in L[i0:]:
        if out and re.match(r'^        (elif self\.step|def |# =====|else:)', l):
            break
        out.append(l)
    return set(strip('\n'.join(out)))


for f in FILES:
    baks = sorted(f.parent.glob(f'{f.name}.bak.*_scout10s'))
    if not baks:
        ck(False, f'{f.name}: 缺备份')
        continue
    bkt = baks[-1].read_text(encoding='utf-8')
    old_blk, new_blk = block_lines(baks[-1]), block_lines(f)
    add = new_blk - old_blk
    dele = old_blk - new_blk
    # 旧 sub3~sub6 原文 (从备份抽) + 旧 sub2 尾巴两行
    i = bkt.index('            if self._scout_sub == 3:\n')
    j = bkt.index('                self._enter_step(STEP_RETURN)\n', i) + len('                self._enter_step(STEP_RETURN)\n')
    DEL_EXPECT = set(strip(bkt[i:j])) | {
        'self._scout_sub = 3; self._scout_sub_start = self.get_clock().now()',
        'self.get_logger().info("Scout: slow move done, moving left 1.5m")',
    }
    _nw = lambda s: re.sub(r'\s+', ' ', s).strip()
    add_n, dele_n = {_nw(x) for x in add}, {_nw(x) for x in dele}
    AE = {_nw(x) for x in ADD_EXPECT}; DE = {_nw(x) for x in DEL_EXPECT}
    bad_a, bad_r = sorted(add_n - AE), sorted(dele_n - DE)
    ck(not bad_a, f'{f.name}: 新增 {len(add)} 行 ⊆ 新完成块 (越界 {[x[:40] for x in bad_a][:2]})')
    ck(not bad_r, f'{f.name}: 删除 {len(dele)} 行 ⊆ 旧 sub3~sub6 + sub2 尾巴 (越界 {[x[:45] for x in bad_r][:2]})')
    ck(len(dele) >= 25, f'{f.name}: 删除 {len(dele)} 行 (≥25 = 确实是整段 sub3~sub6 被拿掉)')

print('\n=== D. 时长预算 ===')
old = 13.7 + 1.0 + 10.0 + 5.0 + 6.0 + 9.0 + 7.0      # 10-08 09:34 实飞实测
new = 13.7 + 1.0 + 10.0
print(f'  改动前 (实测): {old:.1f}s = 进场 13.7 + 降 1 + 缓移 10 + 左移 5 + 停 6 + 右移 9 + 停 7')
print(f'  改动后 (预算): {new:.1f}s = 进场 13.7 + 降 1 + 缓移 10 (这 10s = 在区内待的时间)')
ck(new <= old - 20, f'侦察段省 {old-new:.1f}s ({(old-new)/old*100:.0f}%) ⇒ 每趟少飞 ~{old-new:.0f}s')

print(f'\n=== 结果: {PASS} 通过 / {FAIL} 失败 ===')
sys.exit(1 if FAIL else 0)
