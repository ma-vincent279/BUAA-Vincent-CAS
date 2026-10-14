#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""h 修正① (2026-09-30): 切桶时校验"冻结估计"是否落在已投桶附近 -> 重新 HOVER_SEARCH 选桶
根因: 选桶瞬间的反投影偏估计会退化成已投桶位置(实测偏 2.2m, 离已投桶仅 0.1m), 而 CLIMB 里
     "直接采用 _target_buckets[idx]" 这条路径绕过了 SERVED 守卫 -> 第 2 投打回第 1 桶
     (SITL 实证: DROP-LOCK block dist=1.49m -> 判据永不满足 -> 35s 兜底硬投 -> 0 分)。
"""
import shutil, datetime, sys

ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

ANCHOR = """                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)
"""
INSERT = """                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)
                # 2026-09-30 h 修正①: 校验"冻结估计"是否落在已投桶附近。选桶瞬间的反投影偏估计会退化成
                # 已投桶的位置(实测偏 2.2m, 离已投桶仅 0.1m), 而这条"CLIMB 直接采用冻结估计"的路径**
                # 绕过了 SERVED 守卫** ⇒ 第 2 投瞄准第 1 桶(9-30 SITL: DROP-LOCK block dist=1.49m ⇒
                # 判据永不满足 ⇒ 35s 兜底硬投 ⇒ 0 分)。改为重新走带 SERVED 守卫的 HOVER_SEARCH 选桶。
                _bad_est = any(math.hypot(self._current_target_ned[0] - _sx,
                                          self._current_target_ned[1] - _sy) < SERVED_EXCLUDE_M
                               for _sx, _sy in self._served_targets)
                if _bad_est and getattr(self, '_bad_est_retry', 0) < 2:
                    self._bad_est_retry = getattr(self, '_bad_est_retry', 0) + 1
                    self.get_logger().warn(
                        f"NEXT-BUCKET 估计 NED({self._current_target_ned[0]:.2f},"
                        f"{self._current_target_ned[1]:.2f}) 落在已投桶 {self._served_targets} 附近 "
                        f"(偏估计) → 重新 HOVER_SEARCH 选桶 (第 {self._bad_est_retry}/2 次)")
                    self._enter_step(STEP_HOVER_SEARCH)
                    return
"""

REL_OLD = """                self._release_pred = pred_m; self._release_teff = t_eff; self._release_cmd_t = sim_t
                self._drop_reappr = 0"""
REL_NEW = """                self._release_pred = pred_m; self._release_teff = t_eff; self._release_cmd_t = sim_t
                self._drop_reappr = 0
                self._bad_est_retry = 0      # 2026-09-30 h 修正①: 成功投出后复位"坏估计重选"计数"""


def patch(t):
    assert t.count(ANCHOR) == 1, 'CLIMB 锚点 %d' % t.count(ANCHOR)
    t = t.replace(ANCHOR, INSERT, 1)
    assert t.count(REL_OLD) == 1, '释放块锚点 %d' % t.count(REL_OLD)
    t = t.replace(REL_OLD, REL_NEW, 1)
    assert '_bad_est_retry' in t and 'NEXT-BUCKET 估计' in t
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/h.py', 'fly/src/fly/fly/h.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if '--check' in sys.argv:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_bucketsel')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
