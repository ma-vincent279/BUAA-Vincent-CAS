#!/usr/bin/env python3
"""项目 i: 收紧 POSITION_REFINE 身份门限 + 身份优先 (2026-10-07) — 干净版

背景 (SITL 判别性仿真已复现): 选桶正确选了"大+中"并抛弃最小的桶, 但第 2 投仍落进了
被抛弃的那个小桶. 日志链:
    Next bucket 2/2: NED(1.1,33.8)                                   ← 目标选对了(中桶)
    Position refined: (1.07,33.85) -> (-0.08,34.25) delta=(-1.149,0.402)m   ← 瞄准点被挪 1.22m
    TARGET-REBIND id #4 -> #6 aim=NED(-0.08,34.25)                   ← 身份认领到小桶
根因: POSITION_REFINE 样本选择 = "所有非已投框里离当前瞄准点最近的那个", 门限
`POSITION_REFINE_IDENTITY_MAX = 1.5m` (注释写"< 最小桶间距 2.2m"); 但现场实测最小桶间距
只有 1.66m (SITL 估计值之间甚至 1.42m) ⇒ 1.5m 门限允许它跳到隔壁桶.

改动 (两份 i.py 同步, 只动 POSITION_REFINE 的样本选择):
  A. POSITION_REFINE_IDENTITY_MAX 1.5 -> 0.60; 新增 POSITION_REFINE_ID_MATCH_M = 0.25
  B. 身份优先: 目标追踪器的框可见时只认它; 身份不可见时只收 <0.6m 的样本, 收不到就不修正
  C. 日志: "Position refined ... n=41" 追加 "身份框=x/41"
"""
import datetime
import difflib
import py_compile
import shutil
import sys

STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
FILES = ["/home/vincent/SITL/src/drone_drl/drone_drl/i.py",
         "/home/vincent/SITL/fly/src/fly/fly/i.py"]

OLD_CONST = 'POSITION_REFINE_IDENTITY_MAX = 1.5   # 几何身份锁门限 (m), v6: 2.0->1.5 < 最小桶间距 2.2m'
NEW_CONST = ('POSITION_REFINE_IDENTITY_MAX = 0.60  # 2026-10-07: 1.5->0.60. 原注释"< 最小桶间距 2.2m"不成立\n'
             '                                     #   (9-27 现场实测最小间距 1.66m, SITL 估计值间甚至 1.42m)\n'
             '                                     #   ⇒ 1.5m 门限会让 refine 跳到隔壁桶 (SITL 已复现)\n'
             'POSITION_REFINE_ID_MATCH_M   = 0.25  # 检测框反投影与目标追踪器 ned 相距 < 此值 ⇒ 同一个桶')

OLD_INIT = """            if not hasattr(self, '_refine_ests'):
                self._refine_ests = []
"""
NEW_INIT = """            if not hasattr(self, '_refine_ests'):
                self._refine_ests = []
                self._refine_id_hits = 0
"""

OLD_LOGIC = """                best_est = None; best_dist = float('inf')
                for bx, by, roi, d, cx, cy in self._bucket_ests:
                    if self._is_served(bx, by):
                        continue                                  # A: 已投过的桶不参与 refine
                    geo_dist = math.hypot(bx - tx, by - ty)
                    if geo_dist < best_dist:
                        best_dist = geo_dist; best_est = (bx, by)
                if best_est is not None and best_dist < POSITION_REFINE_IDENTITY_MAX:
                    self._refine_ests.append(best_est)
"""

NEW_LOGIC = """                # 2026-10-07: 身份优先 + 门限 1.5->0.6m.
                #   SITL 复现: 原逻辑在"所有非已投框"里挑"离瞄准点最近"的, 门限 1.5m, 而现场最小桶
                #   间距只有 1.66m (估计值之间甚至 1.42m) ⇒ refine 会跳到隔壁桶、把瞄准点搬过去,
                #   之后 TARGET-REBIND 认领错桶 ⇒ 投进"没被选中/已抛弃"的桶.
                #   新规则: 目标追踪器的框可见时**只认它**; 身份不可见时只收 0.6m 内的样本,
                #   收不到就不修正 (不猜身份).
                _tid = -1
                _tids = getattr(self, '_target_ids', None)
                if _tids and 0 <= self._bucket_idx < len(_tids):
                    _tid = _tids[self._bucket_idx]
                _id_ned = None
                for _t in self._trackers:
                    if _t.get("id", -2) == _tid:
                        _id_ned = _t.get("ned")
                        break
                best_est = None; best_dist = float('inf'); best_is_id = False
                for bx, by, roi, d, cx, cy in self._bucket_ests:
                    if self._is_served(bx, by):
                        continue                                  # A: 已投过的桶不参与 refine
                    geo_dist = math.hypot(bx - tx, by - ty)
                    if geo_dist >= POSITION_REFINE_IDENTITY_MAX:
                        continue                                  # 门限收紧: 只收近的样本
                    same_id = (_id_ned is not None and math.hypot(
                        bx - _id_ned[0], by - _id_ned[1]) < POSITION_REFINE_ID_MATCH_M)
                    if same_id and not best_is_id:
                        best_est, best_dist, best_is_id = (bx, by), geo_dist, True
                    elif same_id == best_is_id and geo_dist < best_dist:
                        best_est, best_dist = (bx, by), geo_dist
                if best_est is not None:
                    self._refine_ests.append(best_est)
                    if best_is_id:
                        self._refine_id_hits = getattr(self, '_refine_id_hits', 0) + 1
"""

OLD_LOG = """                    f"delta=({avg_x-old_x:.3f},{avg_y-old_y:.3f})m n={len(ests)}")"""
NEW_LOG = """                    f"delta=({avg_x-old_x:.3f},{avg_y-old_y:.3f})m n={len(ests)} "
                    f"身份框={getattr(self, '_refine_id_hits', 0)}/{len(ests)}")"""

OLD_RESET = "\n        self._refine_ests = []\n"
NEW_RESET = "\n        self._refine_ests = []\n        self._refine_id_hits = 0\n"

PAIRS = ((OLD_CONST, NEW_CONST), (OLD_INIT, NEW_INIT), (OLD_LOGIC, NEW_LOGIC),
         (OLD_LOG, NEW_LOG), (OLD_RESET, NEW_RESET))


def main():
    ok = True
    for path in FILES:
        src = open(path, encoding="utf-8").read()
        if "best_is_id" in src:
            print(f"  [SKIP] {path}: 已打过补丁")
            continue
        out = src
        bad = False
        for old, new in PAIRS:
            n = out.count(old)
            if n != 1:
                print(f"  [FAIL] {path}: 锚点命中 {n} 次 (应为 1): {old.splitlines()[0][:70]}…")
                bad = True
                break
            out = out.replace(old, new)
        if bad:
            ok = False
            continue
        # 防重叠重复: 关键行数量必须不变
        checks = [("if self._bucket_ests:", src.count("if self._bucket_ests:") ==
                   out.count("if self._bucket_ests:")),
                  ("if not hasattr(self, '_refine_ests'):", src.count("if not hasattr(self, '_refine_ests'):") ==
                   out.count("if not hasattr(self, '_refine_ests'):")),
                  ("best_is_id 无重复", out.count("best_is_id") == NEW_LOGIC.count("best_is_id")),
                  ("门限=0.60", "POSITION_REFINE_IDENTITY_MAX = 0.60" in out)]
        for label, good in checks:
            if not good:
                print(f"  [FAIL] {path}: 复核失败 — {label}")
                ok = False
                bad = True
        if bad:
            continue
        shutil.copy2(path, f"{path}.bak.{STAMP}_refine_id")
        open(path, "w", encoding="utf-8").write(out)
        try:
            py_compile.compile(path, doraise=True)
        except py_compile.PyCompileError as e:
            shutil.copy2(f"{path}.bak.{STAMP}_refine_id", path)
            print(f"  [FAIL] {path}: 编译失败已回滚 — {e}")
            ok = False
            continue
        print(f"  [OK  ] {path.split('/')[-2]}/{path.split('/')[-1]}: 已改+编译通过, "
              f"备份 *.bak.{STAMP}_refine_id")
        d = [l for l in difflib.unified_diff(src.splitlines(True), out.splitlines(True), n=0)
             if l.startswith(("+", "-")) and not l.startswith(("+++", "---"))]
        print("".join("        " + l for l in d))
    print("\n" + ("=== ALL OK ===" if ok else "=== 有失败项 ==="))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
