#!/usr/bin/env python3
# ================================================================
# 2026-09-20 晚 (二) 项目 e: C 改成"严格安全"版 (SITL + fly 两份一起改)
# ----------------------------------------------------------------
# 第一版 C 的问题 (当晚仿真实测): 追踪器在**低空投弹那 ~16s 丢检**会被 TRACKER_TIMEOUT=2s
# 裁剪, CLIMB 段重建时**id 已变** → id 查找失败; 而我把"猜身份"的兜底只写在 `id < 0` 分支里
# → 整个 C 空转 (日志里一条 TARGET-REFRESH 都没有).
#
# 本版改动:
#   ① 删掉"从冻结的旧估计猜最近的桶"这条兜底 —— 现场两桶只隔 1.66m, 猜错就是投错桶,
#     宁可不刷 (DROP 段本来每帧用反投影重建 target).
#   ② 新增 `_rebind_target_id()`: 在 **POSITION_REFINE 把瞄准点修正准确之后**, 用 0.6m 紧门限
#     + 0.3m 无歧义间距 + 排除已投过的桶, 重新认领该目标当前的追踪器 id. 之后 PRE_SERVO/
#     VISUAL_SERVO 的刷新就又能按 id 工作了.
#
# 用法: python3 scripts/fix_e_target_rebind_20260920.py [--check]
# ================================================================
import hashlib
import os
import shutil
import subprocess
import sys
import time

ROOT = '/home/vincent/SITL'
FILES = ['src/drone_drl/drone_drl/e.py', 'fly/src/fly/fly/e.py']
STAMP = time.strftime('%Y%m%d_%H%M%S')
CHECK = '--check' in sys.argv

# ---- ① 常量: 去掉已无用的 MARGIN, 换成 rebind 的两个参数 ----
C_OLD = 'TARGET_REFRESH_MARGIN = 0.50  # C: 退回"最近追踪器"时要与第二名拉开的最小间距 (防歧义)'
C_NEW = '''TARGET_REBIND_MAX_M   = 0.60  # C: refine 后重新认领追踪器 id 的最大距离 (紧门限, << 桶间距)
TARGET_REBIND_MARGIN  = 0.30  # C: 认领时要求比第二名近这么多 (防歧义)'''

# ---- ② 刷新函数: 删掉有风险的"猜身份"兜底 ----
F_OLD = '''        cand = None
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
            return False'''
F_NEW = '''        if tid is None or tid < 0:
            return False                     # 未绑定身份 → 不刷 (DROP 段本来每帧反投影重建)
        cand = None
        for t in self._trackers:
            if t.get("id") == tid and t.get("confirmed"):
                cand = t
                break
        if cand is None:
            # id 已失效 —— 低空投弹期间该桶丢检, 追踪器被 TRACKER_TIMEOUT 裁剪后**重建会换 id**.
            # 这里**刻意不猜身份** (从冻结的旧估计猜"最近的桶"在现场 1.66m 间距下有认错风险),
            # 等 POSITION_REFINE 把瞄准点修正准确后由 _rebind_target_id() 重新认领.
            return False'''

# ---- ③ 新增 _rebind_target_id (插在 _update_trackers 之前) ----
R_OLD = '    def _update_trackers(self, bucket_ests):'
R_NEW = '''    def _rebind_target_id(self):
        """C: 用"刚被 refine 修正过的准确瞄准点"重新认领目标的追踪器 id.

        追踪器在低空投弹期间会因丢检被裁剪, CLIMB 重建后 id 已变 → 这里重新认领.
        因为锚点已被 refine 修正准确 (选桶瞬间 ~1m 的偏差已消掉), 用 0.6m 紧门限最近邻
        认领是安全的 (现场桶间距 1.66m >> 0.6m), 并要求与第二名拉开 0.3m (无歧义),
        同时排除已投过的桶.
        """
        if self._current_target_ned is None or self._bucket_idx >= len(self._target_ids):
            return False
        ox, oy = self._current_target_ned
        best = None; best_d = float('inf'); second_d = float('inf')
        for t in self._trackers:
            if not t.get("confirmed") or self._is_served(t["ned"][0], t["ned"][1]):
                continue
            d = math.hypot(t["ned"][0] - ox, t["ned"][1] - oy)
            if d < best_d:
                second_d, best_d, best = best_d, d, t
            elif d < second_d:
                second_d = d
        if best is None or best_d > TARGET_REBIND_MAX_M:
            return False
        if second_d - best_d < TARGET_REBIND_MARGIN:
            return False                     # 有两个候选贴在瞄准点附近 → 不认领
        old = self._target_ids[self._bucket_idx]
        if best.get("id") == old:
            return False
        self._target_ids[self._bucket_idx] = best.get("id", -1)
        self.get_logger().info(
            f"  TARGET-REBIND id #{old} -> #{best.get('id')} aim=NED({ox:.2f},{oy:.2f}) "
            f"tracker=NED({best['ned'][0]:.2f},{best['ned'][1]:.2f}) d={best_d:.2f}m")
        return True

    def _update_trackers(self, bucket_ests):'''

# ---- ④ refine 成功后调用 rebind ----
RF_OLD = '''                self._current_target_ned = (avg_x, avg_y)
                self.get_logger().info(
                    f"Position refined: ({old_x:.2f},{old_y:.2f}) -> ({avg_x:.2f},{avg_y:.2f}) "
                    f"delta=({avg_x-old_x:.3f},{avg_y-old_y:.3f})m n={len(ests)}")'''
RF_NEW = '''                self._current_target_ned = (avg_x, avg_y)
                self.get_logger().info(
                    f"Position refined: ({old_x:.2f},{old_y:.2f}) -> ({avg_x:.2f},{avg_y:.2f}) "
                    f"delta=({avg_x-old_x:.3f},{avg_y-old_y:.3f})m n={len(ests)}")
                self._rebind_target_id()          # C: 瞄准点已准确 → 重新认领该桶的追踪器 id'''

EDITS = [('rebind 常量', C_OLD, C_NEW), ('去掉猜身份兜底', F_OLD, F_NEW),
         ('新增 _rebind_target_id', R_OLD, R_NEW), ('refine 后调用 rebind', RF_OLD, RF_NEW)]


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()[:8]


def main():
    print('=== e: C 改严格安全版 (id 刷新 + refine 后重新认领, 不猜身份) ===\n')
    todo = []
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        done = sum(1 for _, o, n in EDITS if n in src and o not in src)
        print(f'{rel}  md5={md5(p)}')
        if done == len(EDITS):
            print('   → 已落地, 跳过\n')
            continue
        for name, old, new in EDITS:
            c = src.count(old)
            if c != 1:
                print(f'   ✘ 锚点「{name}」命中 {c} 次 (期望 1) → 中止')
                return 1
        print(f'   {len(EDITS)}/{len(EDITS)} 个锚点唯一命中 ✓\n')
        todo.append((p, src))
    if not todo:
        print('无需改动.')
        return 0
    if CHECK:
        print('[--check] 仅检查, 未写入.')
        return 0

    for p, src in todo:
        bak = f'{p}.bak.{STAMP}_target_rebind'
        shutil.copy2(p, bak)
        print(f'备份 → {os.path.relpath(bak, ROOT)}')
        for _, old, new in EDITS:
            src = src.replace(old, new, 1)
        open(p, 'w', encoding='utf-8').write(src)
        os.utime(p, None)
        print(f'已改 → {os.path.relpath(p, ROOT)}  md5={md5(p)}')

    print('\n--- 复验 ---')
    bad = 0
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        r = subprocess.run([sys.executable, '-m', 'py_compile', p], capture_output=True, text=True)
        miss = [n for n, _, new in EDITS if new not in src]
        nouse = 'TARGET_REFRESH_MARGIN' in src
        print(f'{rel}: py_compile={"OK" if r.returncode == 0 else "FAIL"}  '
              f'落地={len(EDITS)-len(miss)}/{len(EDITS)}  旧常量残留={"有✘" if nouse else "无✓"}  md5={md5(p)}'
              + (f'  缺: {miss}' if miss else ''))
        if r.returncode or miss or nouse:
            bad += 1
            if r.stderr:
                print(r.stderr)
    print('\n结果:', '全部通过 (记得 colcon build)' if bad == 0 else f'{bad} 个文件异常')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
