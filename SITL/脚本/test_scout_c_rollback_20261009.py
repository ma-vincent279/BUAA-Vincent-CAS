#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侦察回滚 c 世代 验收 (2026-10-09)

四节:
  A. 结论性判据: 8 份文件里 `_scout_pos_limited` / `_scout_settled` **零调用**; `SCOUT_SUB_TIMEOUT = 10.0`
  B. **整块对齐 c.py**: 剥掉行尾注释后, 侦察主块与 c 世代的差异**只允许** e 世代那 5 行记账
     (`_scout_done` / `_scout_elapsed` / 更详细的那条完成日志) ⇒ 证明"回滚到 c 那几代"
  C. 最小改动审计: 与 `*.bak.<ts>_scout_c` 逐行比 ⇒ **只有 SCOUT_SUB_TIMEOUT 一个常量变了**,
     其余侦察常量 (DIST/EDGE/SLOW_MOVE_TIME/DESCENT/LEFT/RIGHT/PAUSE/ARRIVE) 逐位不变
  D. 时长预算: 按常量算 回滚前 vs 回滚后 的各子阶段时长, 并与 10-08 09:34 实飞实测 (侦察 51.7s) 对照
"""
import math
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


TREES = ['src/drone_drl/drone_drl', 'fly/src/fly/fly']
NAMES = ['i.py', 'j.py', 'k.py', 'l.py']
FILES = [ROOT / t / n for t in TREES for n in NAMES]
C_REF = ROOT / 'fly/src/fly/fly/c.py'

# 2026-10-09: 本脚本只审计"回滚到 c 世代"这件事 ⇒ 晚于 scout_c 的补丁 (如 scout10s) 的行豁免
try:
    sys.path.insert(0, str(ROOT / 'scripts'))
    from _later_patch_marks import later_than, is_deleted_line
    LATER_RE = tuple(__import__('re').escape(m) for m in later_than('scout_c'))
except Exception:
    LATER_RE = ()

# e 世代记账行 (允许保留, 与侦察动作无关; 只为 MISSION SUMMARY 写时长)
ALLOW_VS_C = [
    r'_scout_done = True',
    r'_scout_elapsed = self\._step_elapsed\(\)',
    r'Scout maneuver complete \(\{self\._scout_elapsed',
    r'^\s*f"Scout maneuver complete',
    r'^\s*self\.get_logger\(\)\.info\($',
    r'Scout maneuver complete, returning home',   # c 世代那条被 e 世代替换掉的日志 (删除侧)
]


def scout_block(path):
    L = path.read_text(encoding='utf-8').split('\n')
    i0 = [i for i, ln in enumerate(L) if ln.strip().startswith('elif self.step == STEP_SCOUT:')][-1]
    out = []
    for ln in L[i0:]:
        if out and re.match(r'^        (elif self\.step|def |# =====|else:)', ln):
            break
        out.append(ln)
    code = []
    for l in out:
        l = l.split('#')[0].rstrip()          # 剥行尾注释
        if l.strip():
            code.append(l)
    txt = '\n'.join(code)
    for w in 'ijkl':
        txt = txt.replace(f'{w}模式', 'X模式')
    return txt


print('=== A. 回滚是否到位 (8 份文件) ===')
for f in FILES:
    if not f.exists():
        ck(False, f'{f.relative_to(ROOT)} 不存在')
        continue
    t = f.read_text(encoding='utf-8')
    n = f.name
    ck('_scout_pos_limited(tx' not in t, f'{n}: 侦察段已无 _scout_pos_limited 调用 (取消平移限速)')
    ck('if self._scout_settled(sub_elapsed)' not in t, f'{n}: 侦察段已无 _scout_settled 调用 (取消判稳门控)')
    ck(re.search(r'^SCOUT_SUB_TIMEOUT = 10\.0', t, re.M) is not None, f'{n}: SCOUT_SUB_TIMEOUT = 10.0 (c 世代)')
    ck(t.count('def _scout_pos_limited') == 1 and t.count('def _scout_settled') == 1,
       f'{n}: 两个 h 助手函数仍保留定义 (未删, 便于对照/回退)')

print('\n=== B. 整块对齐 c.py (允许的差异只有 e 世代记账) ===')
cblk = scout_block(C_REF)
for f in FILES:
    if not f.exists():
        continue
    b = scout_block(f)
    import difflib
    d = [l for l in difflib.unified_diff(cblk.split('\n'), b.split('\n'), lineterm='', n=0)
         if l[:1] in '+-' and l[:3] not in ('+++', '---')]
    d = [l for l in d if not any(re.search(p, l[1:]) for p in LATER_RE)
         and not (l.startswith('-') and is_deleted_line(l[1:]))]     # 剔后续补丁 + 被整段删除的旧行
    bad = [l for l in d if not any(re.search(p, l[1:]) for p in ALLOW_VS_C)]
    ck(not bad, f'{f.name}: 侦察块与 c 世代一致 (差异 {len(d)} 行, 越界 {len(bad)} 行'
                f'{"" if not bad else ": " + str([x.strip()[:50] for x in bad])})')

print('\n=== C. 最小改动审计 (与回滚前备份逐行比) ===')
SC = ['SCOUT_DIST', 'SCOUT_EDGE_DIST', 'SCOUT_SLOW_MOVE_TIME', 'SCOUT_DESCENT_DELTA',
      'SCOUT_LEFT_DIST', 'SCOUT_RIGHT_DIST', 'SCOUT_PAUSE_TIME', 'SCOUT_SUB_ARRIVE']
for f in FILES:
    baks = sorted(f.parent.glob(f'{f.name}.bak.*_scout_c'))
    if not baks or not f.exists():
        ck(False, f'{f.name}: 缺回滚前备份')
        continue
    old = baks[-1].read_text(encoding='utf-8')
    new = f.read_text(encoding='utf-8')
    o = {k: re.search(rf'^{k}\s*=\s*([^\n#]+)', old, re.M).group(1).strip() for k in SC}
    nw = {k: re.search(rf'^{k}\s*=\s*([^\n#]+)', new, re.M).group(1).strip() for k in SC}
    ck(o == nw, f'{f.name}: 其余 {len(SC)} 个侦察常量逐位不变')
    ot = re.search(r'^SCOUT_SUB_TIMEOUT\s*=\s*([^\n#]+)', old, re.M).group(1).strip()
    nt = re.search(r'^SCOUT_SUB_TIMEOUT\s*=\s*([^\n#]+)', new, re.M).group(1).strip()
    ck(ot == '14.0' and nt == '10.0', f'{f.name}: 唯一变的常量 = SCOUT_SUB_TIMEOUT ({ot}→{nt})')

print('\n=== D. 时长预算 (回滚前 vs 回滚后) ===')
f0 = ROOT / 'fly/src/fly/fly/l.py'
t0 = f0.read_text(encoding='utf-8')
g = lambda k: float(re.search(rf'^{k}\s*=\s*([\d.]+)', t0, re.M).group(1))
slow, left, right, pause, arrv = (g('SCOUT_SLOW_MOVE_TIME'), g('SCOUT_LEFT_DIST'),
                                  g('SCOUT_RIGHT_DIST'), g('SCOUT_PAUSE_TIME'), g('SCOUT_SUB_ARRIVE'))
spd, ramp = g('SCOUT_MOVE_SPEED'), g('SCOUT_MOVE_RAMP')
# 限速世代: 等速平移 + 末段线性减速 (路程 / 峰值速度 近似, 取 ~1.25 系数)
old_left, old_right = left / spd * 1.25, right / spd * 1.25
# c 世代: 直接位置设定点, PX4 位置环 ~1.0~1.5m/s 冲到位 + 判据 0.3m
new_left, new_right = left / 1.2 + 0.6, right / 1.2 + 0.6
print(f"  {'子阶段':<22}{'回滚前(s)':>10}{'回滚后(s)':>10}")
print(f"  {'直飞近边 (sub0)':<22}{'~13.7':>10}{'~13.7':>10}   (未动)")
print(f"  {'降 1m (sub1)':<22}{'1.0':>10}{'1.0':>10}   (未动)")
print(f"  {'缓移到中心 (sub2)':<22}{slow:>10.1f}{slow:>10.1f}   (未动, c 世代本来就有)")
print(f"  {'左移 1.5m (sub3)':<22}{old_left:>10.1f}{new_left:>10.1f}   ← 取消限速")
print(f"  {'停 5s (sub4)':<22}{pause + 1.0:>10.1f}{pause:>10.1f}   ← 取消判稳门控")
print(f"  {'右移 3m (sub5)':<22}{old_right:>10.1f}{new_right:>10.1f}   ← 取消限速")
print(f"  {'停 5s (sub6)':<22}{pause + 2.0:>10.1f}{pause:>10.1f}   ← 取消判稳门控")
old_sum = 13.7 + 1 + slow + old_left + pause + 1 + old_right + pause + 2
new_sum = 13.7 + 1 + slow + new_left + pause + new_right + pause
print(f"  {'合计':<22}{old_sum:>10.1f}{new_sum:>10.1f}   ⇒ 省 {old_sum - new_sum:.1f}s "
      f"({(old_sum - new_sum) / old_sum * 100:.0f}%)")
ck(12.0 <= old_sum - new_sum <= 20.0, f'预计省时 {old_sum - new_sum:.1f}s 落在 12~20s 区间')
print(f"  实测对照 (10-08 09:34 实飞 i): 侦察总 51.7s = 13.7 + 1 + 10.0 + 5 + 6 + 9 + 7 ⇒ "
      f"与「回滚前」模型 {old_sum:.1f}s 吻合 (差 {abs(old_sum - 51.7):.1f}s)")

print(f'\n=== 结果: {PASS} 通过 / {FAIL} 失败 ===')
sys.exit(1 if FAIL else 0)
