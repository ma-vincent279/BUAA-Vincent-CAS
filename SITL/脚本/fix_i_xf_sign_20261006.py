#!/usr/bin/env python3
"""项目 i: 修正两轴补偿中 XF(纵向)的符号 (+0.12 -> -0.05) — 2026-10-06

依据 (10-06 实飞四投 + i.py 代码语义逐行核对):
  1. 代码语义: 冻结点 hold = 桶估计 + Y*(sin h, -cos h) - XF*(cos h, sin h)
     => Y>0 = 瞄准点左移;  XF>0 = 瞄准点朝"起飞点(近侧)"移.
     冻结点就是位置设定点, 落点跟着 1:1 平移 (外加飞机自身游走 b).
  2. 10-06 四投实际跑的补偿 = B1=(Y+0.250, XF+0.040), B2=(Y+0.120, XF+0.040)
     (日志 `offset_横_B1=0.250 B2=0.120 offset_前_B1=0.040 B2=0.040`).
     用户现场真值: 四投落点全部偏"近侧(朝起飞点)" 8.5~22.5cm.
  3. 飞机自身游走 b 可扣除, 已用日志 home 复算验证:
     `到位err` == |飞机 - home - 冻结点|, 四投 1.3/5.1/8.3/14.6cm vs 日志 1.5/5.4/8.3/15.0cm.
     纵向 b 分量均值 ≈ -0.015m (飞机平均偏远侧 1.5cm) => b 不解释"偏近侧".
  4. => "偏近侧" 必须"减小"XF; 按项目自己的公式 新值 = 旧值 - 实测偏移 + b
     ≈ +0.04 - 0.08 - 0.015 ≈ **-0.05**.
     10-06 那次重标写成了 +0.12 (= 旧值 + 实测) => 符号相反.
     (同一公式 Y 用对了: 0.25 - 0.12 = +0.13 ✓)

只改两个常量 + 两条注释.  e/f/g/h/f_test 一行不动, i 的其余逻辑一行不动.
"""

import datetime
import difflib
import hashlib
import os
import re
import shutil
import sys

FILES = [
    "/home/vincent/SITL/src/drone_drl/drone_drl/i.py",
    "/home/vincent/SITL/fly/src/fly/fly/i.py",
]

OLD_XF_B1 = "PAYLOAD_OFFSET_BODY_XF_B1 = +0.12    # 2026-10-06 按 4 投实测重标: 全部偏'近侧(短)'8.5~22.5cm"
OLD_XF_B2 = "PAYLOAD_OFFSET_BODY_XF_B2 = +0.12    # 2026-10-06 同上 (两瓶共用纵向值)"

NEW_XF_B1 = (
    "PAYLOAD_OFFSET_BODY_XF_B1 = -0.05    # 2026-10-06 符号修正: 10-06 四投在 XF=+0.04 下全偏'近侧(朝起飞点)'8.5~22.5cm\n"
    "                                     #   XF>0 = 瞄准点/落点朝起飞点移 => 偏近侧须\"减小\"XF; 0.04-0.08-0.015≈-0.05"
)
NEW_XF_B2 = "PAYLOAD_OFFSET_BODY_XF_B2 = -0.05    # 2026-10-06 同上 (两瓶共用纵向值)"

STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def sha(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def main():
    ok = True
    for path in FILES:
        if not os.path.isfile(path):
            print(f"[FAIL] 找不到 {path}")
            ok = False
            continue
        with open(path, "r", encoding="utf-8") as fh:
            src = fh.read()

        before_sha = sha(path)
        print(f"\n=== {path}\n    sha256(before) = {before_sha}")

        cur = re.findall(r"PAYLOAD_OFFSET_BODY_XF_B1 = ([+-][\d.]+)", src)
        y_vals = re.findall(r"PAYLOAD_OFFSET_BODY_Y_B([12]) = ([+-][\d.]+)", src)
        print(f"    现 XF_B1 = {cur}, Y = {y_vals}")

        if cur == ["-0.05"]:
            print("    [SKIP] 已是 -0.05 (幂等)")
            continue
        if cur != ["+0.12"]:
            print(f"    [FAIL] XF_B1 现值不是 +0.12, 拒绝改 (避免误伤): {cur}")
            ok = False
            continue
        if y_vals != [("1", "+0.13"), ("2", "+0.12")]:
            print(f"    [FAIL] Y 常量与预期不符, 拒绝改: {y_vals}")
            ok = False
            continue

        # 断言锚点唯一命中
        for name, anchor in (("XF_B1", OLD_XF_B1), ("XF_B2", OLD_XF_B2)):
            n = src.count(anchor)
            if n != 1:
                print(f"    [FAIL] 锚点 {name} 命中 {n} 次 (应为 1)")
                ok = False
        if not ok:
            continue

        # 统计改前的用法点数 (改完必须一样多, 证明只动了常量)
        n_use = src.count("PAYLOAD_OFFSET_BODY_XF_B")
        n_lines_before = src.count("\n")

        # 备份
        bak = f"{path}.bak.{STAMP}_xf_sign"
        shutil.copy2(path, bak)
        print(f"    备份 -> {bak}")

        out = src.replace(OLD_XF_B1, NEW_XF_B1).replace(OLD_XF_B2, NEW_XF_B2)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(out)

        # 复核
        with open(path, "r", encoding="utf-8") as fh:
            new = fh.read()
        new_xf = re.findall(r"PAYLOAD_OFFSET_BODY_XF_B([12]) = ([+-][\d.]+)", new)
        new_y = re.findall(r"PAYLOAD_OFFSET_BODY_Y_B([12]) = ([+-][\d.]+)", new)
        new_use = new.count("PAYLOAD_OFFSET_BODY_XF_B")
        checks = [
            ("XF 两值 = -0.05", new_xf == [("1", "-0.05"), ("2", "-0.05")]),
            ("Y 未被改动", new_y == [("1", "+0.13"), ("2", "+0.12")]),
            ("用法点数量不变", new_use == n_use),
            ("仅多 1 行(新增注释)", new.count("\n") == n_lines_before + 1),
        ]
        for label, good in checks:
            print(f"    [{'OK ' if good else 'FAIL'}] {label}")
            ok = ok and good

        d = list(difflib.unified_diff(src.splitlines(True), new.splitlines(True),
                                      fromfile=path + " (before)", tofile=path + " (after)", n=1))
        sys.stdout.write("".join(d))
        print(f"    sha256(after)  = {sha(path)}")

    print("\n" + ("=== ALL OK ===" if ok else "=== 有失败项, 请检查 ==="))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
