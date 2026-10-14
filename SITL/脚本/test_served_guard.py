#!/usr/bin/env python3
# ================================================================
# 2026-09-20 晚 A+C 规则离线验证 (用**真实日志里的数值**回放)
# ----------------------------------------------------------------
# 数据来源: 2026-09-20 晚 e 仿真 (修复前那一轮, 10/20) 的 /tmp/e_output.log.prev_*
#   - 选桶: Selected 2 buckets: NED(-0.5,34.6), NED(1.2,33.7)   ← B3/B2 的偏估计
#   - 切目标: Next bucket 2/2: NED(1.2,33.7)                     ← 第 2 投瞄准点被冻结
#   - refine: REFINE candidates vs target(1.2,33.7):
#             (2.2,32.9)d=1.2 [B2], (-1.3,31.5)d=3.4 [B1], (0.2,33.9)d=1.0 [B3] best=1.0
#             Position refined: (1.23,33.74) -> (0.16,33.93)  ← 就近选中了已投过的 B3
#   - 真值: B1(-1.50,31.50) B2(2.00,33.00) B3(0.00,34.00); 第 1 投 aim=(0.00,33.96)
# 本测试直接 exec 出厂 e.py 里**真实的** _is_served / _refresh_target_from_tracker,
# 用同一批数字判定修复前后行为.
# 用法: python3 scripts/test_served_guard.py
# ================================================================
import math
import os
import re
import sys

ROOT = '/home/vincent/SITL'
FILES = {'SITL/e': 'src/drone_drl/drone_drl/e.py', 'fly/e': 'fly/src/fly/fly/e.py'}

# ---- 真实日志数值 ----
SEL_B3 = (-0.50, 34.62)      # B3 的选桶估计 (偏 0.78m)
SEL_B2 = (1.23, 33.74)       # B2 的选桶估计 (偏 1.06m)
FROZEN = (1.20, 33.70)       # 第 2 个目标被冻结的值
REFINE_CANDS = [(2.2, 32.9, 1.2), (-1.3, 31.5, 3.4), (0.2, 33.9, 1.0)]  # (est, d) 来自日志
AIM_1 = (0.00, 33.96)        # 第 1 投实际瞄准点 (已投过 → 应被登记为 served)
TRUTH = {'B1': (-1.50, 31.50), 'B2': (2.00, 33.00), 'B3': (0.00, 34.00)}
CLIMB_TRACKERS = [(-0.00, 33.99), (2.00, 32.99)]   # CLIMB 段两个活追踪器 (= 真值, 准到 1~2cm)


def extract(src, start, stop=r'^\S'):
    lines = src.split('\n')
    i0 = next(i for i, l in enumerate(lines) if re.match(start, l))
    i1 = next((i for i in range(i0 + 1, len(lines)) if re.match(stop, lines[i])), len(lines))
    return '\n'.join(lines[i0:i1]).rstrip('\n') + '\n'


class FakeLogger:
    def __init__(self):
        self.msgs = []

    def info(self, m):
        self.msgs.append(m)

    def warn(self, m):
        self.msgs.append('WARN ' + m)


class FakeClock:
    class _T:
        def __init__(self, ns):
            self.nanoseconds = ns

        def __sub__(self, other):
            return FakeClock._T(self.nanoseconds - other.nanoseconds)

    def now(self):
        return self._T(int(_now[0] * 1e9))


_now = [0.0]


def load(path):
    """从出厂 e.py 里 exec 出真实方法, 装到一个最小假对象上."""
    src = open(path, encoding='utf-8').read()
    ns = {'math': math}
    for fn in ('camera_pixel_to_level', 'level_to_camera_pixel'):
        if f'def {fn}' in src:
            exec(extract(src, rf'^def {fn}'), ns)
    # 常量
    for ln in src.split('\n'):
        if re.match(r'^[A-Z][A-Z0-9_]*(?:\s*,\s*[A-Z][A-Z0-9_]*)*\s*=', ln):
            try:
                exec(ln, ns)
            except Exception:
                pass

    class Obj:
        pass

    o = Obj()
    o.__dict__.update({k: v for k, v in ns.items() if k.isupper()})
    o.get_clock = lambda: FakeClock()
    o._start_wall = FakeClock._T(0)
    o.get_logger = lambda: o.log
    o.log = FakeLogger()
    # 真实方法绑定 (提取 4 空格缩进的方法体 → 去掉缩进后 exec 成顶层函数, 再绑到假对象上)
    for fn in ('_is_served', '_refresh_target_from_tracker', '_rebind_target_id'):
        m = re.search(rf'^    def {fn}\(self.*?(?=^    def |\Z)', src, re.S | re.M)
        assert m, f'{path}: 找不到 {fn}'
        body = '\n'.join(l[4:] if l.startswith('    ') else l for l in m.group(0).split('\n'))
        exec(body, ns)
        setattr(o, fn, ns[fn].__get__(o, Obj))
    return o, ns


print('=' * 88)
print('A+C 规则离线验证 (数值全部取自 2026-09-20 晚那次 10/20 的真实日志)')
print('=' * 88)
ok = True

for key, rel in FILES.items():
    o, ns = load(os.path.join(ROOT, rel))
    print(f'\n--- {key}  (SERVED_EXCLUDE_M={o.SERVED_EXCLUDE_M}, '
          f'TARGET_REFRESH_MAX_M={o.TARGET_REFRESH_MAX_M}, '
          f'TARGET_REBIND_MAX_M={o.TARGET_REBIND_MAX_M}) ---')

    # ---------- A1: 修复前 refine 会选 B3 ----------
    best_before = min(REFINE_CANDS, key=lambda c: c[2])
    tag = 'B3 ✗(修复前的失效)' if abs(best_before[0] - TRUTH['B3'][0]) < 0.5 else '?'
    print(f'  A0 修复前 refine 最近邻 = ({best_before[0]},{best_before[1]}) d={best_before[2]} → {tag}')

    # ---------- A2: 登记 served 后, B3 的框必须被排除 ----------
    o._served_targets = [AIM_1]
    kept = [(bx, by) for bx, by, d in REFINE_CANDS if not o._is_served(bx, by)]
    best_after = min(kept, key=lambda c: math.hypot(c[0] - FROZEN[0], c[1] - FROZEN[1])) if kept else None
    good = best_after is not None and math.hypot(best_after[0] - TRUTH['B2'][0],
                                                 best_after[1] - TRUTH['B2'][1]) < 1.5
    print(f'  A1 登记 served=({AIM_1[0]},{AIM_1[1]}) 后, refine 候选剩 {len(kept)}/{len(REFINE_CANDS)} 个'
          f' → 选中 ({best_after[0]},{best_after[1]}) ' + ('→ B2 ✓' if good else '✗'))
    ok &= good

    # ---------- A3: 实机间距 1.66m 不能误伤 ----------
    o._served_targets = [TRUTH['B3']]
    b2_166 = (TRUTH['B2'][0] - 0.30, TRUTH['B2'][1] + 0.10)   # 带噪声的 B2 框估计
    not_hurt = not o._is_served(*b2_166)
    print(f'  A2 实机间距 1.66m 情形 (B3 已投, B2 框估计 {b2_166}): '
          + ('未被误排除 ✓' if not_hurt else '✗ 误排除'))
    ok &= not_hurt

    # ---------- A4: 已投桶的框必须被排除 ----------
    hit = o._is_served(0.2, 33.9)
    print(f'  A3 已投桶 B3 的框 (0.2,33.9) 被排除: ' + ('✓' if hit else '✗'))
    ok &= hit

    # ---------- C1: 用追踪器 id 刷新 → 精确到真值 ----------
    o2, _ = load(os.path.join(ROOT, rel))
    o2._trackers = [{"id": 0, "ned": CLIMB_TRACKERS[0], "confirmed": True},
                    {"id": 1, "ned": CLIMB_TRACKERS[1], "confirmed": True}]
    o2._served_targets = [AIM_1]          # 第 1 投已投 B3
    o2._target_ids = [0, 1]               # 第 1 个目标 = tracker#0(B3), 第 2 个 = tracker#1(B2)
    o2._bucket_idx = 1
    o2._current_target_ned = FROZEN       # 冻结的偏估计
    o2._last_refresh_log = 0.0
    changed = o2._refresh_target_from_tracker()
    tgt = o2._current_target_ned
    err_frozen = math.hypot(FROZEN[0] - TRUTH['B2'][0], FROZEN[1] - TRUTH['B2'][1])
    err_after = math.hypot(tgt[0] - TRUTH['B2'][0], tgt[1] - TRUTH['B2'][1])
    good_c = changed and err_after < 0.05
    print(f'  C1 追踪器 id 刷新: 冻结 {FROZEN}(偏 {err_frozen:.2f}m) → ({tgt[0]:.2f},{tgt[1]:.2f})'
          f'(偏 {err_after:.2f}m) ' + ('✓' if good_c else '✗'))
    ok &= good_c

    # ---------- C2: 不许刷到已投过的桶 ----------
    o3, _ = load(os.path.join(ROOT, rel))
    o3._trackers = [{"id": 5, "ned": TRUTH['B3'], "confirmed": True}]
    o3._served_targets = [AIM_1]
    o3._target_ids = [5]
    o3._bucket_idx = 0
    o3._current_target_ned = (0.1, 33.8)
    o3._last_refresh_log = 0.0
    refused = (not o3._refresh_target_from_tracker()) and o3._current_target_ned == (0.1, 33.8)
    print(f'  C2 目标追踪器本身就是已投桶 → 拒绝刷新: ' + ('✓' if refused else '✗'))
    ok &= refused

    # ---------- C3: 追踪器偏太多 (身份可疑) → 不刷 ----------
    o4, _ = load(os.path.join(ROOT, rel))
    o4._trackers = [{"id": 7, "ned": (5.0, 30.0), "confirmed": True}]
    o4._served_targets = []
    o4._target_ids = [7]
    o4._bucket_idx = 0
    o4._current_target_ned = FROZEN
    o4._last_refresh_log = 0.0
    refused2 = (not o4._refresh_target_from_tracker()) and o4._current_target_ned == FROZEN
    print(f'  C3 追踪器离冻结目标 4m (> {o4.TARGET_REFRESH_MAX_M}m) → 不刷: '
          + ('✓' if refused2 else '✗'))
    ok &= refused2

    # ---------- C4: id 失效时不许"猜身份" (现场 1.66m 间距会认错桶) ----------
    o5, _ = load(os.path.join(ROOT, rel))
    o5._trackers = [{"id": 99, "ned": (1.9, 32.9), "confirmed": True}]   # 已重建, 换了 id
    o5._served_targets = [AIM_1]
    o5._target_ids = [42]                     # 旧 id 已不存在
    o5._bucket_idx = 0
    o5._current_target_ned = FROZEN
    o5._last_refresh_log = 0.0
    idle = (not o5._refresh_target_from_tracker()) and o5._current_target_ned == FROZEN
    print(f'  C4 id 失效 (旧 42 / 新 99) → 拒绝猜身份, 目标不动: ' + ('✓' if idle else '✗'))
    ok &= idle

    # ---------- C5: refine 把瞄准点修准后, 重新认领追踪器 id ----------
    o6, _ = load(os.path.join(ROOT, rel))
    o6._trackers = [{"id": 99, "ned": (1.92, 32.93), "confirmed": True},
                    {"id": 98, "ned": (-2.40, 31.60), "confirmed": True}]
    o6._served_targets = [AIM_1]
    o6._target_ids = [42]
    o6._bucket_idx = 0
    o6._current_target_ned = (2.15, 32.81)     # refine 修正后的准确瞄准点 (实测值)
    o6._last_refresh_log = 0.0
    rebound = o6._rebind_target_id() and o6._target_ids[0] == 99
    print(f'  C5 refine 修准后重新认领: id 42 -> {o6._target_ids[0]} (期望 99) '
          + ('✓' if rebound else '✗'))
    ok &= rebound

    # ---------- C6: 两个候选都贴住瞄准点 → 不认领 ----------
    o7, _ = load(os.path.join(ROOT, rel))
    o7._trackers = [{"id": 11, "ned": (2.05, 32.95), "confirmed": True},
                    {"id": 12, "ned": (2.35, 32.70), "confirmed": True}]   # 相距 0.4m < 0.3? 见下
    o7._served_targets = []
    o7._target_ids = [42]
    o7._bucket_idx = 0
    o7._current_target_ned = (2.15, 32.81)
    o7._last_refresh_log = 0.0
    d1 = math.hypot(2.05 - 2.15, 32.95 - 32.81); d2 = math.hypot(2.35 - 2.15, 32.70 - 32.81)
    amb = not o7._rebind_target_id() and o7._target_ids[0] == 42
    print(f'  C6 两候选到瞄准点的距离差 {abs(d2-d1):.2f}m (< {o7.TARGET_REBIND_MARGIN}m 无歧义门限) → 不认领: '
          + ('✓' if amb else '✗'))
    ok &= amb

    # ---------- C7: 已投过的桶不许被认领 ----------
    o8, _ = load(os.path.join(ROOT, rel))
    o8._trackers = [{"id": 21, "ned": (0.05, 33.95), "confirmed": True}]
    o8._served_targets = [AIM_1]
    o8._target_ids = [42]
    o8._bucket_idx = 0
    o8._current_target_ned = (0.02, 33.92)
    o8._last_refresh_log = 0.0
    no_served = not o8._rebind_target_id()
    print(f'  C7 已投桶的追踪器不许被认领: ' + ('✓' if no_served else '✗'))
    ok &= no_served

print()
print('=' * 88)
print('结论:', '全部通过 ✓ (A 挡住"就近选中已投桶"; C 把瞄准点从偏 1.06m 修到 0.01m)' if ok else '存在失败项 ✗')
sys.exit(0 if ok else 1)
