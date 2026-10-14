#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""项目 j 离线验收 (2026-10-08)

三节:
  A. 源码不变量: 两份 j.py 的常量/门/兜底度量/失效开放/日志都在; 且与 i.py 的**代码差异**
     (剥除注释与字符串重命名后) 只在本改动允许的集合内 —— 防止"顺手改了别的东西".
  B. 判据行为: 用今天 14 投的 dev/落点真值表断言核心结论 (大 dev ⇒ 偏; 小 dev ⇒ 准),
     并断言 j 的门"只会推迟、不会提前"释放 (release 条件是"旧条件 ∧ 新条件"; 兜底路径结构未变).
  C. 失效开放: 拿不到相机值 (dev=None) 时门不拦 ⇒ 行为退回 i (不引入新风险).
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


# ---------- 今天 14 投: (趟, 投, |dev| cm, 落点合成误差 cm) ----------
# dev 见 docs/project_log.md 追加 20/21/22; 落点 = 用户现场卷尺 (未量轴按 0 计)
DATA = [
    ('130234', 1, 68.3, 30.0), ('130234', 2, 11.2, 12.7),
    ('141457', 1, 32.1, 12.5), ('141457', 2, 37.2, 12.0),
    ('160201', 1, 0.9, 5.0), ('160201', 2, 7.5, 7.1),
    ('162730', 1, 35.6, 19.8), ('162730', 2, 28.1, 21.2),
    ('181852', 1, 6.4, 10.0), ('181852', 2, 4.2, 0.0),
    ('184418', 1, 14.0, 0.0), ('184418', 2, 3.8, 10.0),
    ('093428', 1, 22.9, 13.9), ('093428', 2, 29.8, 12.0),
]

print('=== A. 源码不变量 ===')
SRC = [ROOT / 'src/drone_drl/drone_drl/j.py', ROOT / 'fly/src/fly/fly/j.py']
ISRC = [ROOT / 'src/drone_drl/drone_drl/i.py', ROOT / 'fly/src/fly/fly/i.py']
for p in SRC:
    ck(p.exists(), f'{p.relative_to(ROOT)} 存在')
    t = p.read_text(encoding='utf-8')
    ck('class JMission(Node):' in t, f'{p.parent.name}: 类名 JMission')
    ck('node = JMission()' in t and 'IMission' not in t and 'i_mission' not in t,
       f'{p.parent.name}: main() 实例化 JMission 且无 IMission/i_mission 残留 (SITL 崩过一次的坑)')
    ck("super().__init__('j_mission')" in t, f'{p.parent.name}: 节点名 j_mission')
    ck('J_DEV_GATE_M   = 0.00' in t,
       f'{p.parent.name}: J_DEV_GATE_M 默认 0.00 = 门关闭 (纯仪表, 行为与 i 一致)')
    ck('J_MEASURED_R   = False' in t, f'{p.parent.name}: 补偿重标默认关 (保持 i 的值)')
    ck('PAYLOAD_OFFSET_BODY_Y_B1 = +0.35' in t, f'{p.parent.name}: 重标开关内含实测值 0.35')
    ck('self._i_est_f = (_hx0, _hy0)' in t, f'{p.parent.name}: 冻结时记下 est (dev 基准)')
    ck('_dev_ok = (J_DEV_GATE_M <= 0.0) or (_dev_m is None) or (_dev_m <= J_DEV_GATE_M)' in t,
       f'{p.parent.name}: 门 + 失效开放')
    ck('and _pred_ok and _dev_ok) or _dl_pick' in t, f'{p.parent.name}: 释放条件 = 旧条件 ∧ 新条件')
    ck('_pick = _dev_m if (J_DEV_DEADLINE_PICK and _dev_m is not None) else _pred' in t,
       f'{p.parent.name}: 兜底改用 dev 度量 (无相机值时退回 pred)')
    ck('偏差=' in t and 'dev=(' in t, f'{p.parent.name}: dev 日志 (释放行 + 状态行)')
    ck('i模式' not in t, f'{p.parent.name}: 日志前缀已改 j模式')

    # 代码差异审计: 剥注释后, 逐行 diff 只允许本改动引入的内容
    def code_lines(txt):
        out = []
        for ln in txt.splitlines():
            s = re.sub(r'\s+#.*$', '', ln).strip()      # 去掉整行注释与行尾注释
            if not s or s.startswith('#'):
                continue
            out.append(s)
        return out
    a, b = code_lines(ISRC[SRC.index(p)].read_text(encoding='utf-8')), code_lines(t)
    import difflib
    allowed = re.compile(
        r'J_DEV|J_LOG|J_PICK|J_MEASURED_R|_pick|JMission|_i_dl_best|_pred <= max|_dev_m|_dev_ok|_i_est_f|_efz|_estl|_dF|_dR|'
        r'class JMission|j_mission|MISSION_REVISION|偏差=|dev=\(|水平方向|PAYLOAD_OFFSET_BODY_Y_B[12] = \+0\.(35|05)|'
        r'and self\.latest_roi is not None|_target_tracker_ok\(\) and self\._i_roi_inside|j模式|子项目 [ij]|IMission|i_mission|i模式')
    bad = []
    for ln in difflib.unified_diff(a, b, lineterm='', n=0):
        if ln.startswith(('---', '+++', '@@')):
            continue
        body = ln[1:].strip()
        if ln.startswith(('+', '-')) and not allowed.search(body):
            bad.append(ln)
    ck(not bad, f'{p.parent.name}: 代码差异全部在允许集合内 (越界 {len(bad)} 行)')
    for x in bad[:6]:
        print('     !', x)

print('\n=== B. 判据行为 (今天 14 投真值) ===')
big = [d for d in DATA if d[2] >= 22.0]
small = [d for d in DATA if d[2] <= 14.0]
ck(len(big) == 7 and len(small) == 7, f'分组: dev ≥22cm 有 {len(big)} 投, ≤14cm 有 {len(small)} 投')
ck(all(e >= 12.0 for _, _, _, e in big), 'dev ≥22cm 的 7 投落点全部 ≥12cm (全部出桶)')
ok_small = [d for d in small if d[3] <= 10.0]
ck(len(ok_small) >= 6, f'dev ≤14cm 的 7 投里 {len(ok_small)} 投 ≤10cm (唯一例外 13:02 #2 = 12.7cm 擦边)')
ck(all(e >= 12.0 for _, _, _, e in big), 'dev 大的一组无一投落点在桶内 ⇒ 可用作"报警器"判据')
# "只会推迟": 旧判据放行时刻集合 ⊇ 新判据放行时刻集合 (新条件是合取 + 一项), 且兜底必投不变
for p in SRC:
    t = p.read_text(encoding='utf-8')
    ck('I_RELEASE_TIMEOUT = 20.0' in t and 'I_DEADLINE_WIN   = 5.0' in t,
       f'{p.parent.name}: 20s 必投兜底与 5s 挑时机窗口未改 (必投保证不变)')
    ck('_dl_pick = bool(_deadline and (' in t, f'{p.parent.name}: 兜底路径结构未改')

print('\n=== C. 失效开放 (相机值缺失 ⇒ 退回 i) ===')
GATE = 0.10
def dev_ok(dev_m):
    return (GATE <= 0.0) or (dev_m is None) or (dev_m <= GATE)
ck(dev_ok(None) is True, 'dev=None (丢检/贴边/追踪器未确认) ⇒ 门不拦, 按旧判据放行')
ck(dev_ok(0.05) is True, 'dev=5cm ⇒ 放行')
ck(dev_ok(0.35) is False, 'dev=35cm ⇒ 拦 (16:27 那两投会被拦)')
ck(dev_ok(0.10) is True and dev_ok(0.10001) is False, '门限边界按 ≤ 处理')

print(f'\n结果: {PASS} 通过 / {FAIL} 失败')
sys.exit(1 if FAIL else 0)
