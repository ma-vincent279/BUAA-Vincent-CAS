#!/usr/bin/env python3
"""差分验收: i.py 的 refine 兜底判据 (改前 vs 改后), 用真实实飞数据回放.

做法: 从**改前备份**与**改后文件**里各自抽出 refine 采样判据块原文 (从
`best_est = None; best_dist = ...` 到 `if best_est is not None:` 之前), exec 成函数,
喂同一组输入, 比较"选了哪个框".

场景 (数值全部取自真实日志):
  S1 10-07 实飞第 2 趟 refine 窗口 (i_mission_20261007_090632.log):
       aim=(-29.07,-17.42); 框: (-29.99,-16.55)(真目标A, d=1.3) / (-32.84,-16.16)(d=4.0)
       / (-28.76,-17.87)(邻居B, d=0.5) / (-27.77,-19.68)(d=2.6); 身份框丢失 (_id_ned=None)
       已选目标: A=(-29.99,-16.47) 索引0, B=(-28.80,-17.74) 索引1
       ⇒ 改前: 采纳 B (把瞄准点搬到邻居 = 今天的 bug); 改后: **不采纳任何框**
  S2 10-06 实飞 (成功那两趟) 的身份路径: 身份框在场 (与 _id_ned 相距 0.10m)
       ⇒ 改前/改后都必须采纳它 (不能破坏好路径)
  S3 10-07 SITL 第 2 桶的兜底场景: 唯一候选距离 0.48m, 无邻居
       ⇒ 改前采纳; 改后不采纳 (宁可不修正, 由 1.2m 冻结兜住 —— 与 10-07 文档一致)

用法: python3 scripts/test_i_bucket_merge_20261007.py
"""
import math
import textwrap
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path("/home/vincent/SITL")
NEW = [ROOT / "src/drone_drl/drone_drl/i.py", ROOT / "fly/src/fly/fly/i.py"]
OLD = sorted(ROOT.glob("src/drone_drl/drone_drl/i.py.bak.*_bucket_merge"))
START = "best_est = None; best_dist = float('inf'); best_is_id = False"
END = "if best_est is not None:"

PASS, FAIL = [], []


def check(cond, msg):
    (PASS if cond else FAIL).append(msg)
    print(("  ✓ " if cond else "  ✗ ") + msg)


def build_picker(path: Path):
    """抽出判据块原文, exec 成 pick(self, tx, ty, _id_ned, match_m, id_max)."""
    s = path.read_text(encoding="utf-8")
    i = s.rfind("\n", 0, s.index(START)) + 1      # 回退到该行行首 (保留原始缩进)
    j = s.index(END, i)
    j = s.rfind("\n", 0, j) + 1
    block = textwrap.dedent(s[i:j])
    src = ("def pick(self, tx, ty, _id_ned, POSITION_REFINE_ID_MATCH_M, "
           "POSITION_REFINE_IDENTITY_MAX):\n"
           + textwrap.indent(block, "    ")
           + "    return best_est, best_is_id, best_dist\n")
    scope = {}
    exec(compile(src, str(path), "exec"), {"math": math}, scope)
    return scope["pick"], block


def make_self(boxes, targets, served, idx):
    def is_served(bx, by):
        return any(math.hypot(bx - sx, by - sy) < 0.60 for sx, sy in served)
    return SimpleNamespace(_bucket_ests=boxes, _target_buckets=targets,
                           _bucket_idx=idx, _is_served=is_served)


def run(pick, sc, id_max):
    """id_max 必须是**该版本当时**的常量值: 改前 0.60 / 改后 0.25."""
    return pick(sc["self"], sc["tx"], sc["ty"], sc["id_ned"], 0.25, id_max)


def main():
    if not OLD:
        sys.exit("[FAIL] 找不到改前备份 i.py.bak.*_bucket_merge")
    old_path = OLD[-1]
    print(f"改前: {old_path.relative_to(ROOT)}")
    pick_old, block_old = build_picker(old_path)
    for p in NEW:
        pick_new, block_new = build_picker(p)
        print(f"改后: {p.relative_to(ROOT)}")

    check("_amb" not in block_old, "改前判据块不含明确性检查")
    for p in NEW:
        _, b = build_picker(p)
        check("_amb = True" in b, f"{p.name}: 改后判据块含明确性检查")
        check("POSITION_REFINE_IDENTITY_MAX = 0.25" in p.read_text(encoding="utf-8"),
              f"{p.name}: 兜底门限已是 0.25")

    # ---------- S1 今天实飞的真实数据 ----------
    print("\n[S1] 10-07 实飞: 4 个框 / 身份框丢失 / 邻居 0.5m, 真目标 1.3m")
    boxes = [(-29.99, -16.55, None, None, None, None),
             (-32.84, -16.16, None, None, None, None),
             (-28.76, -17.87, None, None, None, None),
             (-27.77, -19.68, None, None, None, None)]
    sc = {"self": make_self(boxes, [(-29.99, -16.47), (-28.80, -17.74)], [], 0),
          "tx": -29.07, "ty": -17.42, "id_ned": None}
    o = run(pick_old, sc, 0.60)      # 改前该常量 = 0.60
    n = run(pick_new, sc, 0.25)
    check(o[0] is not None and math.hypot(o[0][0] + 28.80, o[0][1] + 17.74) < 0.15,
          f"改前采纳了邻居 B {o[0]} ⇒ 复现今天的 bug")
    check(n[0] is None, f"改后不采纳任何框 {n[0]} ⇒ 瞄准点不被搬走")
    # 门限放宽到 0.60 (改前语义) 时, 即使加了明确性判据也应拒绝 B
    n_loose = pick_new(sc["self"], sc["tx"], sc["ty"], None, 0.25, 0.60)
    check(n_loose[0] is None, "改后即使门限回到 0.60, 明确性判据仍拒绝邻居 B")

    # ---------- S2 10-06 身份路径 ----------
    print("\n[S2] 10-06 成功那两趟的口径: 身份框在场 (id_ned 就在旁边)")
    boxes2 = [(-30.71, 11.27, None, None, None, None), (-31.86, 9.56, None, None, None, None)]
    sc2 = {"self": make_self(boxes2, [(-30.71, 11.27), (-31.86, 9.56)], [], 0),
           "tx": -30.87, "ty": 11.08, "id_ned": (-30.75, 11.20)}
    o2, n2 = run(pick_old, sc2, 0.60), run(pick_new, sc2, 0.25)
    check(o2[0] == n2[0] == (-30.71, 11.27) and o2[1] and n2[1],
          f"改前/改后都采纳身份框 {n2[0]} (好路径不受影响)")

    # ---------- S3 10-07 SITL 的 0.48m 兜底 ----------
    print("\n[S3] 10-07 SITL 第 2 桶口径: 唯一候选 0.48m, 无邻居")
    boxes3 = [(-28.80, -17.74, None, None, None, None)]
    sc3 = {"self": make_self(boxes3, [(-28.80, -17.74), (-29.90, -16.50)], [], 0),
           "tx": -28.80, "ty": -17.26, "id_ned": None}
    o3, n3 = run(pick_old, sc3, 0.60), run(pick_new, sc3, 0.25)
    check(o3[0] is not None, f"改前采纳 (0.48m < 0.60) {o3[0]}")
    check(n3[0] is None, "改后不采纳 (0.48m > 0.25) ⇒ 跳过修正, 由 1.2m 冻结兜住")

    print(f"\n结果: {len(PASS)} 项通过, {len(FAIL)} 项失败")
    if FAIL:
        for f in FAIL:
            print("  ✗ " + f)
        return 1
    print("✅ 差分验收通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
