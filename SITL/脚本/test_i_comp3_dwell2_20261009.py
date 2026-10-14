#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 试飞改动验收 (2026-10-09): 补偿右移 3cm + 停留 5s→2s

A. 常量落地: 两份 i.py 里 Y_B1=+0.15 / Y_B2=-0.03 / I_DWELL_S=2.0 (AST 解析, 不 import)
B. 最小改动审计: 与 `*.bak.*_comp3dwell2` 逐行比 (剥行尾注释) ⇒ 只允许那 3 行 + 已登记的后续补丁行
C. 释放链完整性: 仍是 `停留 ∧ pred<9cm ∧ vxy<0.15` 的 AND, 且 20s 必投兜底仍在 ⇒ 缩短停留不会产生"投不出去"
D. 预期量化 (用 17 投实测 + 到位后收敛曲线):
   ⚠ 2026-10-09 用户决定: 只保留「横向补偿右移 3cm」, 停留 5→2s 已回滚 ⇒ 停留部分只作留档
   · 3cm 平移: 落点均值 +1.5 → −1.5cm, 桶口 12.5cm 命中率 58.3% → 58.3% (Δ −0.0pt, 不亏不赚)
   · 停留 5→2s: 到位后残差在 +2s 最小(实测 8 投均值 ~5cm), 而 dev 以 ~3~6cm/s 增长 ⇒ 省 8~15cm,
     且 pred<9cm 门仍在把关 ⇒ 预期显著为正
"""
import ast
import difflib
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


def norm(txt):
    return [re.sub(r'\s*#.*$', '', l).rstrip() for l in txt.split('\n') if re.sub(r'\s*#.*$', '', l).strip()]


def consts(path):
    """AST 取模块级常量 (不执行文件)"""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except Exception:
                pass
    return out


FILES = [ROOT / 'src/drone_drl/drone_drl/i.py', ROOT / 'fly/src/fly/fly/i.py']

print('=== A. 常量落地 ===')
for f in FILES:
    c = consts(f)
    ck(c.get('PAYLOAD_OFFSET_BODY_Y_B1') == 0.15, f'{f.parent.name}/i.py: Y_B1 = {c.get("PAYLOAD_OFFSET_BODY_Y_B1")} (=+0.15)')
    ck(c.get('PAYLOAD_OFFSET_BODY_Y_B2') == -0.03, f'{f.parent.name}/i.py: Y_B2 = {c.get("PAYLOAD_OFFSET_BODY_Y_B2")} (=-0.03)')
    ck(c.get('I_DWELL_S') == 5.0,
       f'{f.parent.name}/i.py: I_DWELL_S = {c.get("I_DWELL_S")} (=5.0 —— 2026-10-09 用户决定回滚, 只留 3cm)')
    ck(c.get('PAYLOAD_OFFSET_BODY_XF_B1') == 0.05 and c.get('PAYLOAD_OFFSET_BODY_XF_B2') == 0.05,
       f'{f.parent.name}/i.py: XF 纵向未动 (=+0.05/+0.05)')
    ck(c.get('I_RELEASE_TIMEOUT') == 20.0, f'{f.parent.name}/i.py: 20s 必投兜底未动')

print('\n=== B. 最小改动审计 (vs 改动前备份) ===')
try:
    sys.path.insert(0, str(ROOT / 'scripts'))
    from _later_patch_marks import LATER_PATCH_MARKERS_RE as LATER_RE, is_deleted_line
except Exception:
    LATER_RE = ()
skip = lambda rows: [r for r in rows
                    if not any(re.search(p, r) for p in LATER_RE) and not is_deleted_line(r)]
ALLOW = {'PAYLOAD_OFFSET_BODY_Y_B1 = +0.15', 'PAYLOAD_OFFSET_BODY_Y_B2 = -0.03', 'I_DWELL_S        = 2.0',
         'PAYLOAD_OFFSET_BODY_Y_B1 = +0.18', 'PAYLOAD_OFFSET_BODY_Y_B2 = 0.00', 'I_DWELL_S        = 5.0'}
for f in FILES:
    baks = sorted(f.parent.glob(f'{f.name}.bak.*_comp3dwell2'))
    ck(bool(baks), f'{f.name}: 存在改动前备份')
    if not baks:
        continue
    o, n = norm(baks[-1].read_text(encoding='utf-8')), norm(f.read_text(encoding='utf-8'))
    d = [x[1:].strip() for x in difflib.unified_diff(o, n, lineterm='', n=0)
         if x[:1] in '+-' and x[:3] not in ('+++', '---')]
    d = skip(d)
    bad = [x for x in d if x not in ALLOW]
    ck(not bad, f'{f.name}: 只改 3 个常量 (差异 {len(d)} 行, 越界 {len(bad)}: {bad[:2]})')

print('\n=== C. 释放链完整性 ===')
for f in FILES:
    t = f.read_text(encoding='utf-8')
    ck('I_DWELL_S' in t and re.search(r'_dwell\s*=\s*sim_t - self\._i_freeze_t', t) is not None,
       f'{f.name}: 停留计时仍存在')
    ck(re.search(r'if \(_settled and _vxy < I_VMAX and _pred_ok\)', t) is not None,
       f'{f.name}: 释放仍是 AND 门 (停留 ∧ vxy ∧ pred)')
    ck(re.search(r'_deadline = _dwell >= I_RELEASE_TIMEOUT', t) is not None,
       f'{f.name}: 20s 兜底仍在 ⇒ 不会出现"投不出去"')
    ck(re.search(r'if _forced:\s*$', t, re.M) is not None or '_forced = _t_wait > I_RELEASE_TIMEOUT' in t,
       f'{f.name}: 冻结端也有兜底强制路径')

print('\n=== D. 预期量化 ===')
# 实测: 17 投 dev 横向 (右为正, cm) 与 11 投卷尺真值解出的常数项
DEV = [-8.4, -23.3, 16.2, 0.5, -0.4, 31.0, 23.8, 5.8, -1.1, 14.0, -3.7, 22.0, 20.8, -5.2, -3.3, 15.6, 13.1]
C_M, C_S = 8.4, 5.9
mdev = sum(DEV) / len(DEV)
sdev = math.sqrt(sum((x - mdev) ** 2 for x in DEV) / (len(DEV) - 1))
P = lambda m, s: 0.5 * (math.erf((12.5 - m) / (s * math.sqrt(2))) - math.erf((-12.5 - m) / (s * math.sqrt(2))))
s = math.sqrt(sdev ** 2 + C_S ** 2)
m0 = C_M - mdev
print(f'  dev 横向: 均值 {mdev:+.1f}cm σ {sdev:.1f}cm;  常数项 C: {C_M:+.1f}cm σ {C_S:.1f}cm')
print(f'  改动前: 落点均值 {m0:+.1f}cm, σ {s:.1f}cm ⇒ 命中率 {P(m0, s)*100:.1f}%')
print(f'  ③ 3cm 右移后: 均值 {m0-3:+.1f}cm ⇒ {P(m0-3, s)*100:.1f}%  (Δ{(P(m0-3, s)-P(m0, s))*100:+.1f}pt)')
ck(abs((P(m0 - 3, s) - P(m0, s)) * 100) < 5.0, '3cm 平移本身是"不亏不赚"量级 (|Δ|<5pt)')
# 到位后残差实测 (8 投): +1s / +2s / +3s / +4s
CONV = [(11.4, 4.8, 1.3, 5.1), (13.9, 4.4, 12.2, 11.2), (12.4, 3.8, 6.0, 2.9), (13.4, 3.1, 9.2, 7.3),
        (4.3, 19.1, 17.9, 6.6), (4.2, 2.0, 3.9, 3.9), (19.4, 2.4, 4.6, 8.3), (25.5, 3.7, 7.3, 5.3)]
mean_at = lambda i: sum(c[i] for c in CONV) / len(CONV)
print(f'  到位后残差实测(8 投均值): +1s {mean_at(0):.1f}cm  **+2s {mean_at(1):.1f}cm**  +3s {mean_at(2):.1f}cm  +4s {mean_at(3):.1f}cm')
ck(mean_at(1) <= mean_at(3), '"+2s 残差 ≤ +4s 残差" ⇒ 早投不损位置精度')
DRIFT = [34.5 / 6.0, 17.5 / 5.8, 25.6 / 5.0, 30.5 / 5.0, 35.6 / 5.8, 28.1 / 5.0]   # 停留期漂移速率 (cm/s)
rd = sum(DRIFT) / len(DRIFT)
print(f'  停留期参照漂移速率(6 投实测): 均值 {rd:.1f}cm/s ⇒ 少停 3s 省 ≈{rd*3:.0f}cm 的 dev')
ck(rd * 3 > 5.0, f'留档: 停留若缩短 3s 预期省 {rd*3:.0f}cm (本次已回滚)')

print(f'\n=== 结果: {PASS} 通过 / {FAIL} 失败 ===')
sys.exit(1 if FAIL else 0)
