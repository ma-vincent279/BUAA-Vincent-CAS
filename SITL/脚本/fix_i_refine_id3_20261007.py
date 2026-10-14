#!/usr/bin/env python3
"""项目 i: REFINE 身份框不受 0.6m 门限限制 (2026-10-07, 用户要求)

上一版 (fix_i_refine_id2) 把 POSITION_REFINE_IDENTITY_MAX 1.5->0.6 并对**所有**检测框生效;
SITL 复跑发现副作用: 第 2 投"选桶时的旧瞄准点"与"refine 时的新估计"差 0.85m ⇒ 一个样本都
收不到 (`best_dist=inf n=0`) ⇒ refine 被跳过、瞄准点误差只能靠 1.2m 冻结兜住。

本次改成:
  · 身份框 (= 本目标追踪器正在跟的那个框, |框反投影 − 追踪器 ned| < POSITION_REFINE_ID_MATCH_M)
    **不受 0.6m 门限限制** —— 瞄准点陈旧正是 refine 要修的东西;
  · 非身份框 (追踪器不可见时的兜底) 仍要求 < POSITION_REFINE_IDENTITY_MAX = 0.6m (防抢桶);
  · 防抢桶的另一道保险不变: `POSITION_REFINE_MAX_CORRECTION = 1.5m` 仍对**最终平均修正量**设上限,
    超出直接 REJECTED (身份混淆)。
"""
import datetime
import difflib
import py_compile
import shutil
import sys

STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
FILES = ["/home/vincent/SITL/src/drone_drl/drone_drl/i.py",
         "/home/vincent/SITL/fly/src/fly/fly/i.py"]

OLD = """                    geo_dist = math.hypot(bx - tx, by - ty)
                    if geo_dist >= POSITION_REFINE_IDENTITY_MAX:
                        continue                                  # 门限收紧: 只收近的样本
                    same_id = (_id_ned is not None and math.hypot(
                        bx - _id_ned[0], by - _id_ned[1]) < POSITION_REFINE_ID_MATCH_M)
"""

NEW = """                    geo_dist = math.hypot(bx - tx, by - ty)
                    same_id = (_id_ned is not None and math.hypot(
                        bx - _id_ned[0], by - _id_ned[1]) < POSITION_REFINE_ID_MATCH_M)
                    # 2026-10-07: 身份框不看 0.6m 门限 —— 瞄准点陈旧(选桶时/上一状态留下的)正是
                    #   refine 要修的东西; 用 0.6m 卡它会出现"0 样本跳过"(SITL 实测差 0.85m 时发生).
                    #   防抢桶靠"只认身份框" + 结尾 POSITION_REFINE_MAX_CORRECTION=1.5m 总修正上限.
                    #   只有非身份框(追踪器不可见时的兜底)才要求 < POSITION_REFINE_IDENTITY_MAX.
                    if not same_id and geo_dist >= POSITION_REFINE_IDENTITY_MAX:
                        continue
"""

OLD_CMT = ('POSITION_REFINE_IDENTITY_MAX = 0.60  # 2026-10-07: 1.5->0.60. 原注释"< 最小桶间距 2.2m"不成立\n'
           '                                     #   (9-27 现场实测最小间距 1.66m, SITL 估计值间甚至 1.42m)\n'
           '                                     #   ⇒ 1.5m 门限会让 refine 跳到隔壁桶 (SITL 已复现)')
NEW_CMT = ('POSITION_REFINE_IDENTITY_MAX = 0.60  # 2026-10-07: 1.5->0.60, 且**只管非身份框**.\n'
           '                                     #   原注释"< 最小桶间距 2.2m"不成立 (9-27 实测 1.66m,\n'
           '                                     #   SITL 估计值间甚至 1.42m) ⇒ 1.5m 会让 refine 跳到隔壁桶;\n'
           '                                     #   身份框(本目标追踪器的框)不受此门限, 防抢桶靠身份匹配')


def main():
    ok = True
    for path in FILES:
        src = open(path, encoding="utf-8").read()
        if "if not same_id and geo_dist >= POSITION_REFINE_IDENTITY_MAX:" in src:
            print(f"  [SKIP] {path}: 已打过补丁")
            continue
        out = src
        bad = False
        for old, new in ((OLD_CMT, NEW_CMT), (OLD, NEW)):
            n = out.count(old)
            if n != 1:
                print(f"  [FAIL] {path}: 锚点命中 {n} 次 (应为 1): {old.splitlines()[0][:70]}…")
                bad = True
                break
            out = out.replace(old, new)
        if bad:
            ok = False
            continue
        shutil.copy2(path, f"{path}.bak.{STAMP}_refine_id_unlimited")
        open(path, "w", encoding="utf-8").write(out)
        try:
            py_compile.compile(path, doraise=True)
        except py_compile.PyCompileError as e:
            shutil.copy2(f"{path}.bak.{STAMP}_refine_id_unlimited", path)
            print(f"  [FAIL] {path}: 编译失败已回滚 — {e}")
            ok = False
            continue
        print(f"  [OK  ] {path.split('/')[-2]}/{path.split('/')[-1]}: 已改+编译通过, "
              f"备份 *.bak.{STAMP}_refine_id_unlimited")
        d = [l for l in difflib.unified_diff(src.splitlines(True), out.splitlines(True), n=0)
             if l.startswith(("+", "-")) and not l.startswith(("+++", "---"))]
        print("".join("        " + l for l in d))
    print("\n" + ("=== ALL OK ===" if ok else "=== 有失败项 ==="))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
