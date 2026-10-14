#!/usr/bin/env python3
# ================================================================
# 2026-09-20 晚 项目 e: 修"第 2 投又打回第 1 个桶" (A + C, SITL + fly 两份一起改)
# ----------------------------------------------------------------
# 实测根因 (2026-09-20 晚 e 仿真, 日志逐条):
#   1) 选桶那一刻存下的目标点是**单点估计**, 运动中偏 ~1m:
#      B2 真值(2.00,33.00) 存成 (1.2,33.7); B3 真值(0.00,34.00) 存成 (-0.5,34.6)
#   2) 第 1 投被 POSITION_REFINE 修正回真值 → 1.66cm 满分 ✓
#   3) 第 2 个目标冻结在偏了 1.06m 的旧估计上 → `Next bucket 2/2: NED(1.2,33.7)`
#   4) 第 2 次 refine 的"最近邻"在 B3(框估计 d=1.0) 与 B2(自己的框估计 d=1.2) 之间
#      **选中了已投过的 B3** → `Position refined: (1.23,33.74) -> (0.16,33.93)`
#   5) 之后 PRE_SERVO/VISUAL_SERVO/DROP 全瞄 B3 → 第 2 投落在离 B3 仅 0.5cm 处 → 0 分
#
# 本次修复 (互补两条):
#   A. **已投过的桶硬禁止**: 记下已投桶位置, 匹配/refine/选桶一律排除其 0.6m 内的框;
#      没有别的候选就保持等待 (不再"就近"选到已投过的桶).
#   C. **目标点用活追踪器刷新**: 选桶时记下该目标对应的**追踪器 id** (身份, 不随列表增删变化),
#      CLIMB/TRACK/REFINE/PRE_SERVO 每帧用这个追踪器的实时 NED 覆盖瞄准点;
#      id 已失效时退回"最近的已确认且未投过的追踪器", 但要求它比第二名近 ≥0.5m (无歧义).
#
# 用法: python3 scripts/fix_e_served_bucket_20260920.py [--check]
# ================================================================
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = '/home/vincent/SITL'
FILES = ['src/drone_drl/drone_drl/e.py', 'fly/src/fly/fly/e.py']
STAMP = time.strftime('%Y%m%d_%H%M%S')
CHECK = '--check' in sys.argv

# ---------------- 1) 常量 ----------------
CONST_OLD = 'TRACKER_NED_MATCH     = 1.5   # NED 最近邻匹配距离 (m)'
CONST_NEW = CONST_OLD + """
# 2026-09-20 晚 (A+C): 已投过的桶不再参与任何匹配/瞄准; 目标点用活追踪器刷新
SERVED_EXCLUDE_M      = 0.60  # A: 已投过的桶: 落在此半径内的框一律不作候选 (桶半径 0.125m)
TARGET_REFRESH_MAX_M  = 2.00  # C: 用追踪器刷新瞄准点时允许的最大偏差 (仅作合理性门控)
TARGET_REFRESH_MARGIN = 0.50  # C: 退回"最近追踪器"时要与第二名拉开的最小间距 (防歧义)"""

# ---------------- 2) __init__ 状态 ----------------
INIT_OLD = '        self._target_buckets = []        # 选定的目标桶: [(ned_x, ned_y), ...]'
INIT_NEW = INIT_OLD + """
        # 2026-09-20 晚 (A+C):
        self._served_targets = []        # A: 已投过的桶位置 (home 相对 NED), 后续匹配一律排除
        self._target_ids = []            # C: 被选目标的追踪器 id (= 桶身份, 不随追踪器列表增删变化)
        self._next_tracker_id = 0        # C: 追踪器 id 发号器
        self._last_refresh_log = 0.0     # C: 刷新日志限流"""

# ---------------- 3) 两个辅助方法 (插在 _update_trackers 之前) ----------------
HELPERS_OLD = '    def _update_trackers(self, bucket_ests):'
HELPERS_NEW = '''    # ------------------------------------------------------------------
    # 2026-09-20 晚 (A+C): 桶身份保护 —— 已投过的不再瞄, 瞄准点用活追踪器刷新
    # ------------------------------------------------------------------
    def _is_served(self, nx, ny):
        """A: 该位置是否已经投过弹 (0.6m 内视为同一个桶)."""
        for sx, sy in self._served_targets:
            if math.hypot(nx - sx, ny - sy) < SERVED_EXCLUDE_M:
                return True
        return False

    def _refresh_target_from_tracker(self):
        """C: 用"活追踪器"刷新当前目标的瞄准点; 返回是否真的改了.

        身份 = 选桶时记下的 tracker id. 原因见脚本头: 选桶瞬间的单点估计在运动中偏 ~1m,
        照它飞会停在两桶中间, 再被 refine 的最近邻锁到已投过的桶上.
        """
        if self._current_target_ned is None or self._bucket_idx >= len(self._target_ids):
            return False
        tid = self._target_ids[self._bucket_idx]
        ox, oy = self._current_target_ned

        cand = None
        if tid is not None and tid >= 0:
            for t in self._trackers:
                if t.get("id") == tid and t.get("confirmed"):
                    cand = t
                    break
        else:
            # id 失效 (追踪器被裁剪后重建): 退回"最近的已确认且未投过的追踪器", 但要求无歧义
            near = []
            for t in self._trackers:
                if not t.get("confirmed") or self._is_served(t["ned"][0], t["ned"][1]):
                    continue
                near.append((math.hypot(t["ned"][0] - ox, t["ned"][1] - oy), t))
            near.sort(key=lambda v: v[0])
            if near and (len(near) < 2 or near[1][0] - near[0][0] >= TARGET_REFRESH_MARGIN):
                cand = near[0][1]

        if cand is None:
            return False
        nx, ny = cand["ned"]
        if self._is_served(nx, ny):
            return False                                   # 该追踪器就是已投过的桶 → 拒绝刷新
        delta = math.hypot(nx - ox, ny - oy)
        if delta > TARGET_REFRESH_MAX_M:
            return False                                   # 差太多 → 追踪器身份可疑, 保持原目标点
        self._current_target_ned = (nx, ny)
        if delta > 0.10:
            t_now = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if t_now - self._last_refresh_log > 2.0:
                self._last_refresh_log = t_now
                self.get_logger().info(
                    f"  TARGET-REFRESH id=#{cand.get('id')} NED({ox:.2f},{oy:.2f}) -> "
                    f"NED({nx:.2f},{ny:.2f}) delta={delta:.2f}m")
        return True

    def _update_trackers(self, bucket_ests):'''

# ---------------- 4) 追踪器加 id ----------------
TID_OLD = '''                self._trackers.append({
                    "ned": (bx, by),'''
TID_NEW = '''                self._trackers.append({
                    "id": self._next_tracker_id,          # C: 稳定 id (= 桶身份)
                    "ned": (bx, by),'''
TID_INC_OLD = '''                    "area": float(roi.width * roi.height),
                })'''
TID_INC_NEW = '''                    "area": float(roi.width * roi.height),
                })
                self._next_tracker_id += 1'''

# ---------------- 5) _cb_detect: A 排除已投过的框 ----------------
DET_OLD = '''            for bx, by, roi, d, cx, cy in valid:
                geo_dist = math.hypot(bx - self._current_target_ned[0],
                                      by - self._current_target_ned[1])
                if geo_dist < best_geo_dist:'''
DET_NEW = '''            n_skip_served = 0
            for bx, by, roi, d, cx, cy in valid:
                if self._is_served(bx, by):
                    n_skip_served += 1
                    continue                                  # A: 已投过的桶不再作为瞄准候选
                geo_dist = math.hypot(bx - self._current_target_ned[0],
                                      by - self._current_target_ned[1])
                if geo_dist < best_geo_dist:'''

# ---------------- 6) refine 候选: A 排除已投过的框 ----------------
RF_OLD = '''                best_est = None; best_dist = float('inf')
                for bx, by, roi, d, cx, cy in self._bucket_ests:
                    geo_dist = math.hypot(bx - tx, by - ty)'''
RF_NEW = '''                best_est = None; best_dist = float('inf')
                for bx, by, roi, d, cx, cy in self._bucket_ests:
                    if self._is_served(bx, by):
                        continue                                  # A: 已投过的桶不参与 refine
                    geo_dist = math.hypot(bx - tx, by - ty)'''

# ---------------- 7) 选桶: 目标记 id + 排除已投过的候选 ----------------
SEL_OLD = '''            selected = [(t["ned"][0], t["ned"][1]) for t in picked]
            self._target_buckets = selected'''
SEL_NEW = '''            selected = [(t["ned"][0], t["ned"][1]) for t in picked]
            self._target_buckets = selected
            self._target_ids = [t.get("id", -1) for t in picked]   # C: 记下桶身份 (追踪器 id)'''

# ---------------- 8) 刷新调用点 ----------------
CLIMB_OLD = '''            if self._bucket_idx < len(self._target_buckets):
                self._current_target_ned = self._target_buckets[self._bucket_idx]
                self._target_img_idx = -1'''
CLIMB_NEW = '''            if self._bucket_idx < len(self._target_buckets):
                self._current_target_ned = self._target_buckets[self._bucket_idx]
                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)
                self._target_img_idx = -1'''

TRACK_OLD = '''        elif self.step == STEP_TRACK:'''
TRACK_NEW = '''        elif self.step == STEP_TRACK:
            self._refresh_target_from_tracker()            # C: 逼近途中持续用活追踪器刷瞄准点'''

REFINE_OLD = '''            tx, ty = self._current_target_ned
            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, self.heading)'''
REFINE_NEW = '''            self._refresh_target_from_tracker()            # C: 用活追踪器刷新后再做候选选择
            tx, ty = self._current_target_ned
            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, self.heading)'''

PRESERVO_OLD = '''        elif self.step == STEP_PRE_SERVO:'''
PRESERVO_NEW = '''        elif self.step == STEP_PRE_SERVO:
            self._refresh_target_from_tracker()            # C: 下降前再刷一次瞄准点'''

# ---------------- 9) 投弹时登记"已投过" ----------------
DROP_OLD = '''            if not self._drop_triggered:
                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()'''
DROP_NEW = '''            if not self._drop_triggered:
                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()
                # A: 登记已投过的桶 (home 相对), 之后任何匹配/refine/选桶都不再考虑它
                self._served_targets.append((self._servo_target_x, self._servo_target_y))
                self.get_logger().info(
                    f"  SERVED +1 bucket NED({self._servo_target_x:.2f},{self._servo_target_y:.2f}) "
                    f"→ 共 {len(self._served_targets)} 个已投")'''

EDITS = [
    ('常量', CONST_OLD, CONST_NEW),
    ('__init__ 状态', INIT_OLD, INIT_NEW),
    ('辅助方法', HELPERS_OLD, HELPERS_NEW),
    ('追踪器 id 字段', TID_OLD, TID_NEW),
    ('追踪器 id 递增', TID_INC_OLD, TID_INC_NEW),
    ('_cb_detect 排除已投', DET_OLD, DET_NEW),
    ('refine 排除已投', RF_OLD, RF_NEW),
    ('选桶记 id', SEL_OLD, SEL_NEW),
    ('CLIMB 刷新', CLIMB_OLD, CLIMB_NEW),
    ('TRACK 刷新', TRACK_OLD, TRACK_NEW),
    ('REFINE 刷新', REFINE_OLD, REFINE_NEW),
    ('PRE_SERVO 刷新', PRESERVO_OLD, PRESERVO_NEW),
    ('投弹登记已投', DROP_OLD, DROP_NEW),
]


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()[:8]


def main():
    print('=== e: A(已投桶硬禁止) + C(目标点用活追踪器刷新) ===\n')
    todo = []
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        print(f'{rel}  md5={md5(p)}')
        done = sum(1 for _, o, n in EDITS if n in src and o not in src)
        if done == len(EDITS):
            print('   → 已全部落地, 跳过\n')
            continue
        n_ok = 0
        for name, old, new in EDITS:
            c = src.count(old)
            if c != 1:
                print(f'   ✘ 锚点「{name}」命中 {c} 次 (期望 1) → 中止')
                return 1
            n_ok += 1
        print(f'   {n_ok}/{len(EDITS)} 个锚点全部唯一命中 ✓')
        todo.append((p, src))
        print()
    if not todo:
        print('无需改动.')
        return 0
    if CHECK:
        print('[--check] 仅检查, 未写入.')
        return 0

    for p, src in todo:
        rel = os.path.relpath(p, ROOT)
        bak = f'{p}.bak.{STAMP}_served_guard'
        shutil.copy2(p, bak)
        print(f'备份 → {os.path.relpath(bak, ROOT)}')
        for name, old, new in EDITS:
            src = src.replace(old, new, 1)
        open(p, 'w', encoding='utf-8').write(src)
        os.utime(p, None)                       # 刷新 mtime, 防 colcon 跳过重装
        print(f'已改 → {rel}  md5={md5(p)}')

    print('\n--- 复验 ---')
    bad = 0
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        r = subprocess.run([sys.executable, '-m', 'py_compile', p], capture_output=True, text=True)
        checks = {name: (new in src) for name, _, new in EDITS}
        miss = [k for k, v in checks.items() if not v]
        print(f'{rel}: py_compile={"OK" if r.returncode == 0 else "FAIL"}  '
              f'落地={len(checks)-len(miss)}/{len(checks)}  md5={md5(p)}'
              + (f'  缺: {miss}' if miss else ''))
        if r.returncode or miss:
            bad += 1
            if r.stderr:
                print(r.stderr)
    print('\n结果:', '全部通过 (记得 colcon build)' if bad == 0 else f'{bad} 个文件异常')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
