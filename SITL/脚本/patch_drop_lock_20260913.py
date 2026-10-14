#!/usr/bin/env python3
# ================================================================
# 2026-09-13 补丁: d/e (SITL + fly) DROP 段身份锁改造 + 像素残差积分闭环
# ----------------------------------------------------------------
# 动机 (实机 9-13 两次实飞日志 d_mission_20260913_145458 / _150944):
#   * 释放瞬间 err = 52px / 55px, 落点偏差 0.235m, 两次都脱靶
#   * DROP 段 25s 兜底投弹, err 始终 51~95px, 从未收敛
#   * 逐帧重建: 执行误差 (飞机 vs 自身上一秒目标) 仅 0.02~0.11m,
#     但"目标 vs 相机真值"始终 ≥0.23m → 是**瞄准**误差而不是**执行**误差
#   * 同一个静止桶的反投影估计随高度漂 0.65m (5.16m→−4.21 / 2.8m→−4.82 / 1.5m→−4.86)
# 两处缺陷:
#   ① 身份锁 (SERVO_GEO_LOCK_MAX=0.5) 只在"距离超限"时硬退回锚点 →
#      真实的高度漂移被当成误检 → target 永远停在 5m 时代的旧锚点 → 不收敛
#   ② DROP 段像素 PID 输出 (v_fwd/v_lat) 只打日志不参与控制 → 该段是"纯反投影开环",
#      反投影的任何系统性偏置 (内参/镜头/框中心定义) 100% 变成投弹偏差
# 本补丁:
#   ① 身份锁改"连续一致即重锚定" (geo_lock_decide 纯函数, 可离线单测)
#   ② 新增像素残差积分项 _servo_bias_x/y: 把 PID 输出的积分加在重建目标上, 使 err_px
#      真正收敛到死区 → 释放瞬间桶在画面中心 (对反投影偏置免疫, 不依赖地面标定)
# 每处替换都断言"原文本恰好出现 1 次"; 任一断言失败 → 全部不写盘.
# 用法: python3 scripts/patch_drop_lock_20260913.py [--check]
# ================================================================
import ast
import os
import re
import sys

ROOT = '/home/vincent/SITL'
CHECK_ONLY = '--check' in sys.argv

# (路径, 像素 PID 积分步长表达式)  fly 版 DROP 抽帧 PID_DECIMATION_DROP=5 → dt=0.25
TARGETS = [
    (f'{ROOT}/src/drone_drl/drone_drl/e.py', '0.05'),
    (f'{ROOT}/src/drone_drl/drone_drl/d.py', '0.05'),
    (f'{ROOT}/fly/src/fly/fly/e.py', 'dt'),
    (f'{ROOT}/fly/src/fly/fly/d.py', 'dt'),
]

CONSTS = """
# --- 2026-09-13 实机投弹精度修复 (9-13 两次实飞: 释放瞬间 err=52/55px, 落点偏 0.235m, DROP 25s 未收敛) ---
# 机制: DROP 段 target 每帧由反投影重建, 但"距锚点超 0.5m 即冻结在锚点"会把随高度变化的反投影
# 漂移 (实机同一静止桶 5m→1.5m 漂 0.65m) 当成误检 → 永远停在旧锚点 → err 不收敛 → 只能 25s 兜底.
SERVO_GEO_REANCHOR_N   = 4      # 连续 N 帧"一致地超门限" → 认可新位置 (不再被陈旧锚点锁死)
SERVO_GEO_REANCHOR_RAD = 0.30   # "一致"判据: 相邻两帧被拒估计相距 < 0.30m
SERVO_GEO_REANCHOR_MAX = 1.2    # 超此距离永不重锚 (实测高度偏置 ≤0.7m < 1.2m < 最小真实桶间距 2.24m)
SERVO_BIAS_MAX         = 1.0    # 像素残差积分项上限 (m): 抵消反投影系统性偏置, 超限饱和并告警
SERVO_BIAS_ERR_MAX_PX  = 120.0  # 像素偏差超此值视为"看到别的桶", 该帧不积分 (防误检拉偏)
"""

FUNC = '''
# ============================================================
# 2026-09-13: 身份锁决策 (纯函数, 便于离线单测) — 实机 9-13 投弹偏差修复
# ============================================================
def geo_lock_decide(est_x, est_y, anchor_x, anchor_y, gate, weight,
                    prev_est, reject_n, reanchor_n, reanchor_rad, reanchor_max):
    """反投影估计 vs 锚点 → 目标位置. 返回 (mode, tx, ty, reject_n, prev_est).

    accept   : 距锚点 < gate → 采信 (weight*反投影 + (1-weight)*锚点)
    reanchor : 连续 reanchor_n 帧一致地超 gate 且 < reanchor_max → 认可新位置
               (同一个桶因高度变化引起的反投影漂移, 不是误检跳变)
    reject   : 超 gate 但只出现 1~reanchor_n-1 帧 → 暂用锚点, 记账等待一致性确认
    block    : 超 reanchor_max → 疑似别的桶, 恒用锚点且永不重锚
    """
    dist = math.hypot(est_x - anchor_x, est_y - anchor_y)
    if dist < gate:
        return ('accept', weight * est_x + (1.0 - weight) * anchor_x,
                weight * est_y + (1.0 - weight) * anchor_y, 0, None)
    if dist > reanchor_max:
        return ('block', anchor_x, anchor_y, 0, None)
    if prev_est is not None and math.hypot(est_x - prev_est[0], est_y - prev_est[1]) < reanchor_rad:
        n = reject_n + 1
    else:
        n = 1
    if n >= reanchor_n:
        return ('reanchor', est_x, est_y, 0, None)
    return ('reject', anchor_x, anchor_y, n, (est_x, est_y))


'''

INIT_STATE = """        # 2026-09-13 实机投弹精度修复: 身份锁一致性记账 + 像素残差积分 (DROP 闭环)
        self._geo_reject_n = 0          # 连续被拒帧数
        self._geo_pending_est = None    # 上一帧被拒的反投影估计
        self._geo_mode = None           # 上一帧身份锁模式 (模式变化时打日志)
        self._servo_bias_x = 0.0        # 像素残差积分项 (抵消反投影系统性偏置)
        self._servo_bias_y = 0.0
        self._bias_sat_logged = False   # 积分项饱和只告警一次
"""

STEP_RESET = """            # 2026-09-13: 身份锁记账 + 像素残差积分 每桶复位
            self._geo_reject_n = 0
            self._geo_pending_est = None
            self._geo_mode = None
            self._servo_bias_x = 0.0
            self._servo_bias_y = 0.0
            self._bias_sat_logged = False
"""

DROP_BLOCK = '''est = self._roi_to_ned(roi)
anchor_x, anchor_y = self._current_target_ned
lock_mode = 'no_est'; lock_dist = float('nan')
est_x = est_y = float('nan')
if est is not None:
    est_x, est_y = est
    lock_dist = math.hypot(est_x - anchor_x, est_y - anchor_y)
    # 2026-09-13 身份锁改造 (实机 9-13 投弹偏差修复): 原"超门限即冻结在锚点"会把随高度变化的
    # 反投影漂移 (同一静止桶 5m→1.5m 漂 0.65m) 判成误检 → target 永远停在旧锚点 → err 不收敛
    # → 只能 25s 兜底投弹, 落点偏 0.235m. 改为"连续一致即重锚定"(疑桶仍由 reanchor_max 挡住).
    lock_mode, lx, ly, self._geo_reject_n, self._geo_pending_est = geo_lock_decide(
        est_x, est_y, anchor_x, anchor_y, SERVO_GEO_LOCK_MAX, SERVO_EST_WEIGHT,
        self._geo_pending_est, self._geo_reject_n,
        SERVO_GEO_REANCHOR_N, SERVO_GEO_REANCHOR_RAD, SERVO_GEO_REANCHOR_MAX)
    if lock_mode == 'reanchor':
        # 认可新位置: 锚点同步更新 (此后检测不新鲜时回退目标也用新锚点)
        self._current_target_ned = (est_x, est_y)
        anchor_x, anchor_y = est_x, est_y
    self._servo_target_x, self._servo_target_y = lx, ly
else:
    self._servo_target_x, self._servo_target_y = anchor_x, anchor_y
if lock_mode != self._geo_mode:
    self._geo_mode = lock_mode
    self.get_logger().info(
        f"  DROP-LOCK {lock_mode} est=({est_x:.2f},{est_y:.2f}) "
        f"anchor=({anchor_x:.2f},{anchor_y:.2f}) dist={lock_dist:.2f}m "
        f"reject_n={self._geo_reject_n} err={err_px:.0f}px")
# 2026-09-13 像素残差积分 (闭环项): v_fwd/v_lat 是像素 PID 输出, 原本只打日志 → DROP 段
# 是"纯反投影开环", 反投影的系统性偏置 (内参/镜头/框中心定义) 100% 变成投弹偏差. 这里把它
# 的积分加到重建目标上, 让 err_px 真正收敛 → 释放瞬间桶在画面中心 (不依赖地面标定).
# 只在身份锁采信/重锚 (确认还是同一个桶) 且像素偏差合理时积分, 防误检把目标拖偏.
if lock_mode in ('accept', 'reanchor') and err_px < SERVO_BIAS_ERR_MAX_PX:
    bdx, bdy, _ = self._body_to_ned(v_fwd * __DT__, v_lat * __DT__)
    self._servo_bias_x += bdx
    self._servo_bias_y += bdy
    bias_norm = math.hypot(self._servo_bias_x, self._servo_bias_y)
    if bias_norm > SERVO_BIAS_MAX:
        k = SERVO_BIAS_MAX / bias_norm
        self._servo_bias_x *= k
        self._servo_bias_y *= k
        if not self._bias_sat_logged:
            self._bias_sat_logged = True
            self.get_logger().warn(
                f"DROP bias saturated {SERVO_BIAS_MAX:.2f}m (err={err_px:.0f}px) "
                f"— 反投影偏置超预期, 建议实机地面标定相机内参")
self._servo_target_x += self._servo_bias_x
self._servo_target_y += self._servo_bias_y
# 投放补偿公式保留 (实机按落点标定后再填), 当前 B1/B2 均 0 → 无偏移
self._servo_target_x += offset_body_y * math.sin(self.heading)
self._servo_target_y -= offset_body_y * math.cos(self.heading)
'''

# 1 Hz DROP-SERVO 日志行: 追加 身份锁模式 / 距离 / 积分项 / 俯仰角
# 规则: 把锚点尾部的 `")` 换成 ` "` + 换行 + 新字段 f-string 链
LOG_TAIL = ('f"lock={lock_mode} dist={lock_dist:.2f}m "\n' + ' ' * 28 +
            'f"bias=({self._servo_bias_x:+.2f},{self._servo_bias_y:+.2f}) pitch={self._pitch:+.3f}")')
# path → (锚点, 是否取最后一处)   d 的 VISUAL_SERVO 与 DROP 日志行文本相同 → 取最后一处
LOG_PATCH = {
    'src/drone_drl/drone_drl/e.py': ('收敛计时={conv}")', False),
    'src/drone_drl/drone_drl/d.py':
        ('f"target=({self._servo_target_x:.2f},{self._servo_target_y:.2f})")', True),
    'fly/src/fly/fly/e.py': ('收敛计时={conv} dec={PID_DECIMATION_DROP}x")', False),
    'fly/src/fly/fly/d.py': ('f"dec={PID_DECIMATION_DROP}x")', False),
}


def indent(text, n):
    pad = ' ' * n
    return '\n'.join((pad + ln) if ln.strip() else ln for ln in text.split('\n'))


def rep(text, old, new, tag):
    cnt = text.count(old)
    assert cnt == 1, f'[{tag}] 期望出现 1 次, 实际 {cnt} 次:\n{old[:200]}'
    return text.replace(old, new)


def patch(path, dt_expr):
    tag = os.path.relpath(path, ROOT)
    src = open(path, encoding='utf-8').read()
    lines = src.split('\n')

    # --- 1. 常量: 插在 SERVO_GEO_LOCK_MAX 行之后 ---
    idx = [i for i, ln in enumerate(lines) if ln.startswith('SERVO_GEO_LOCK_MAX')]
    assert len(idx) == 1, f'[{tag}] SERVO_GEO_LOCK_MAX 行数 {len(idx)}'
    lines.insert(idx[0] + 1, CONSTS.rstrip('\n'))

    # --- 2. 纯函数: 插在 class ...(Node) 之前 ---
    idx = [i for i, ln in enumerate(lines) if re.match(r'^class \w+\(Node\):', ln)]
    assert len(idx) == 1, f'[{tag}] class Node 行数 {len(idx)}'
    lines.insert(idx[0], FUNC.strip('\n') + '\n')

    text = '\n'.join(lines)

    # --- 3. __init__ 状态 (唯一带注释的 _drop_converge_start 行) ---
    rows = text.split('\n')
    m = [i for i, ln in enumerate(rows)
         if re.match(r'^\s*self\._drop_converge_start = None\s+#', ln)]
    assert len(m) == 1, f'[{tag}] __init__ 锚点 {len(m)}'
    rows.insert(m[0] + 1, INIT_STATE.rstrip('\n'))

    # --- 4. STEP_DROP 复位 (12 空格缩进的裸复位行 = _enter_step 内, 取第一处) ---
    m = [i for i, ln in enumerate(rows)
         if re.match(r'^ {12}self\._drop_converge_start = None\s*$', ln)]
    assert len(m) == 1, f'[{tag}] STEP_DROP 复位行 {len(m)}'
    rows.insert(m[0] + 1, STEP_RESET.rstrip('\n'))
    text = '\n'.join(rows)

    # --- 5. DROP 段身份锁 + 闭环积分 (取文件中最后一处 est = self._roi_to_ned(roi) = DROP 段) ---
    rows = text.split('\n')
    anchors = [i for i, ln in enumerate(rows) if ln.strip() == 'est = self._roi_to_ned(roi)']
    assert anchors, f'[{tag}] 找不到反投影锚点行'
    i0 = anchors[-1]
    ind = len(rows[i0]) - len(rows[i0].lstrip())
    i1 = None
    for j in range(i0 + 1, len(rows)):
        if rows[j].strip() == 'self._servo_target_y -= offset_body_y * math.cos(self.heading)':
            i1 = j
            break
    assert i1 is not None, f'[{tag}] 找不到 DROP 段结束行'
    old_block = '\n'.join(rows[i0:i1 + 1])
    assert 'SERVO_GEO_LOCK_MAX' in old_block, f'[{tag}] DROP 段未含身份锁, 定位错误'
    assert old_block.count('\n') < 20, f'[{tag}] DROP 段过长, 定位可疑'
    new_block = indent(DROP_BLOCK.strip('\n').replace('__DT__', dt_expr), ind)
    text = rep(text, old_block + '\n', new_block + '\n', tag + ' drop_block')

    # --- 6. 1 Hz DROP-SERVO 日志行追加字段 ---
    rel = os.path.relpath(path, ROOT)
    old, last_only = LOG_PATCH[rel]
    new = old[:-2] + ' "' + '\n' + ' ' * 28 + LOG_TAIL
    cnt = text.count(old)
    assert cnt >= 1, f'[{tag}] 日志行锚点缺失: {old}'
    if last_only:
        i = text.rindex(old)
        text = text[:i] + new + text[i + len(old):]
    else:
        text = rep(text, old, new, tag + ' log')

    ast.parse(text)   # 语法必须通过
    return text


plan = []
for path, dt_expr in TARGETS:
    plan.append((path, patch(path, dt_expr)))

for path, text in plan:
    if CHECK_ONLY:
        print(f'  [check ok] {os.path.relpath(path, ROOT)}')
    else:
        open(path, 'w', encoding='utf-8').write(text)
        print(f'  [written]  {os.path.relpath(path, ROOT)}  ({len(text.splitlines())} 行)')
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
