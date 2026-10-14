#!/usr/bin/env python3
# ================================================================
# 2026-09-20 诊断日志: 追踪器/检测框明细 (项目 e, SITL+fly)
# ----------------------------------------------------------------
# 目的: 钉死"同一个桶被记成两个追踪器"的来源 —— 是检测器一帧给了两个框,
#       还是一个框被多次匹配, 还是追踪器超时重建。
# 只加日志, 不改任何控制/判据行为。
# 用法: python3 scripts/add_tracker_diag_e_20260920.py [--check]
# ================================================================
import ast
import os
import sys

ROOT = '/home/vincent/SITL'
TARGETS = [f'{ROOT}/src/drone_drl/drone_drl/e.py', f'{ROOT}/fly/src/fly/fly/e.py']
CHECK_ONLY = '--check' in sys.argv

# 1) 追踪器新建时立刻打日志 (含来源框)
OLD_NEW = """            else:
                self._trackers.append({
                    "ned": (bx, by),
                    "first_seen": now_s,
                    "last_seen": now_s,
                    "confirmed": False,
                    "roi_center": (cx, cy),
                    "area": float(roi.width * roi.height),
                })"""
NEW_NEW = """            else:
                self._trackers.append({
                    "ned": (bx, by),
                    "first_seen": now_s,
                    "last_seen": now_s,
                    "confirmed": False,
                    "roi_center": (cx, cy),
                    "area": float(roi.width * roi.height),
                })
                # 2026-09-20 诊断: 新追踪器 + 来源框 (找重复计数来源)
                self.get_logger().info(
                    f"  TRK-NEW #{len(self._trackers) - 1} NED({bx:.2f},{by:.2f}) "
                    f"from box(cx={cx:.0f},cy={cy:.0f},w={roi.width:.0f},h={roi.height:.0f}) "
                    f"alive={len(self._trackers)}")"""

# 2) 大距离匹配 (疑似蹭到别的桶) 时打日志
OLD_MATCH = """            if best_t is not None and best_d < TRACKER_NED_MATCH:
                alpha = 0.3"""
NEW_MATCH = """            if best_t is not None and best_d < TRACKER_NED_MATCH:
                if best_d > 0.8:   # 2026-09-20 诊断: 距离偏大的匹配
                    self.get_logger().info(
                        f"  TRK-PULL #{self._trackers.index(best_t)} dist={best_d:.2f}m "
                        f"box(cx={cx:.0f},cy={cy:.0f}) NED({bx:.2f},{by:.2f}) "
                        f"← tracker NED({best_t['ned'][0]:.2f},{best_t['ned'][1]:.2f})")
                alpha = 0.3"""

# 3) 每帧末尾: 1Hz 打印"本帧全部框 + 全部追踪器"
OLD_TAIL = """        self._trackers = [t for t in self._trackers
                          if (now_s - t["last_seen"] < TRACKER_TIMEOUT)
                          or id(t) in matched]"""
NEW_TAIL = """        self._trackers = [t for t in self._trackers
                          if (now_s - t["last_seen"] < TRACKER_TIMEOUT)
                          or id(t) in matched]
        # 2026-09-20 诊断: 1Hz 打印本帧全部检测框 + 全部存活追踪器
        if now_s - getattr(self, '_trk_dbg_t', 0.0) > 1.0:
            self._trk_dbg_t = now_s
            box_txt = " ".join(
                f"({cx:.0f},{cy:.0f})w{w:.0f}→NED({bx:.2f},{by:.2f})"
                for bx, by, _roi, _d, cx, cy, w in
                [(bx, by, roi, d, cx, cy, roi.width) for bx, by, roi, d, cx, cy in bucket_ests])
            trk_txt = " ".join(
                f"#{i}{'C' if t['confirmed'] else '-'}NED({t['ned'][0]:.2f},{t['ned'][1]:.2f})"
                f"age{now_s - t['first_seen']:.1f}s"
                for i, t in enumerate(self._trackers))
            self.get_logger().info(
                f"  TRK-DBG step={STEP_NAME.get(self.step, self.step)} nbox={len(bucket_ests)} "
                f"boxes=[{box_txt}] trackers={len(self._trackers)}[{trk_txt}]")"""

plan = []
for p in TARGETS:
    s = open(p, encoding='utf-8').read()
    assert 'TRK-DBG' not in s, f'{p} 已打过诊断补丁'
    for old, new, tag in ((OLD_NEW, NEW_NEW, 'new'), (OLD_MATCH, NEW_MATCH, 'pull'), (OLD_TAIL, NEW_TAIL, 'dbg')):
        n = s.count(old)
        assert n == 1, f'[{os.path.basename(p)}/{tag}] 期望 1 次, 实际 {n} 次'
        s = s.replace(old, new, 1)
    ast.parse(s)
    plan.append((p, s))

for p, s in plan:
    if CHECK_ONLY:
        print('  [check ok]', os.path.relpath(p, ROOT))
    else:
        open(p, 'w', encoding='utf-8').write(s)
        print('  [written] ', os.path.relpath(p, ROOT))
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
