#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""new_project_k_fly_20261013.py — 生成**板载(实机)版** `fly/src/fly/fly/k.py`

为什么单独一个脚本
------------------
`k` = `i.py` 逐字节复制 + 29(+4) 处差异。板载的 `i.py` 与 SITL 的 `i.py` 之间有
**452 行差异**（QoS/话题内联并 `_v4`→`_v1`、`GROUND_NED_Z 0.83→0.0`、
`SCOUT_EDGE_DIST 55→57`、`PID_DECIMATION_*` 抽帧、`HEADING_DEG` 等）。
所以板载版 `k.py` **不是**把 SITL 的 `k.py` 拷过去，而是**以板载 `i.py` 为源**
跑同一个生成器的 `build()` —— 这样板载的 452 行适配被 100% 继承，
而 k 的差异（DROP 段、判据、常量块）也一模一样地打上去。

本脚本在生成之后再做 3 处**环境常量翻到实机侧**（生成器产出的是 SITL 侧值）：
  · `K_SIM_NO_BALLISTIC`  True → **False**   （实机载荷有弹道, 落点 ≠ 释放瞬间站位）
  · `K_SIM_IDEAL_CAMERA`  True → **False**   （实机两瓶有机械偏置, 补偿不能忽略）
  · `K_TARGET_PLANE_H`    0.0  → **-0.278**  （实机求交平面 = 起飞点下方 27.8cm）

补偿保持**归零**（`K_ZERO_COMPENSATION = True`，按用户要求 —— 还没做过投放点真实测量）。

用法
----
  python3 scripts/new_project_k_fly_20261013.py            # 生成 (自动备份)
  python3 scripts/new_project_k_fly_20261013.py --check    # 只校验, 不写
"""
import argparse
import datetime
import pathlib
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
GEN = REPO / "scripts/new_project_k_20261011.py"
FLY_I = REPO / "fly/src/fly/fly/i.py"
FLY_K = REPO / "fly/src/fly/fly/k.py"

# 生成之后要翻到实机侧的环境常量: (正则, 期望的 SITL 值, 实机值, 说明)
ENV_FLIPS = [
    (r"(?m)^K_SIM_NO_BALLISTIC\s*=\s*True", False,
     "实机载荷**有弹道**: 落点 = 释放瞬间站位 + v·t_fall ⇒ 判据必须含速度/预测项"),
    (r"(?m)^K_SIM_IDEAL_CAMERA\s*=\s*True", False,
     "实机两瓶**有机械偏置** ⇒ 补偿不可忽略 (SITL 才归零)"),
    (r"(?m)^K_TARGET_PLANE_H\s*=\s*0\.0", -0.278,
     "实机求交平面 = 起飞点下方 0.278m (SITL 的 0.0 是与检测器 ground 模式互逆用的)"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只校验与报告, 不写文件")
    a = ap.parse_args()

    if not FLY_I.exists():
        raise SystemExit(f"板载 i.py 不存在: {FLY_I}")

    # ---- 1) 以板载 i.py 为源跑主生成器 ----
    tmp = pathlib.Path("/tmp/fly_k_from_fly_i.py")
    r = subprocess.run([sys.executable, str(GEN), "--src", str(FLY_I), "--dst", str(tmp)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        raise SystemExit("主生成器失败")
    src = tmp.read_text(encoding="utf-8")
    n_main = len(src.splitlines())

    # ---- 2) 环境常量翻到实机侧 ----
    import re
    flips = []
    for pat, _, why in ENV_FLIPS:
        m = re.search(pat, src)
        if not m:
            raise SystemExit(f"环境常量锚点未命中: {pat}")
        flips.append((m.group(0), why))
    src = re.sub(ENV_FLIPS[0][0], "K_SIM_NO_BALLISTIC = False", src, count=1)
    src = re.sub(ENV_FLIPS[1][0], "K_SIM_IDEAL_CAMERA = False", src, count=1)
    src = re.sub(ENV_FLIPS[2][0], "K_TARGET_PLANE_H = -0.278", src, count=1)

    # ---- 3) 报告 + 校验 ----
    checks = [
        ("板载话题机制 `_v1`", src.count("PX4_STATUS_SUFFIX") >= 1),
        ("不依赖 SITL 的 common.py", "from drone_drl.common" not in src),
        ("PID 抽帧 (实机抑谐振)", "PID_DECIMATION" in src),
        ("GROUND_NED_Z = 0.0 (实机)", "GROUND_NED_Z = 0.0" in src),
        ("K_SIM_NO_BALLISTIC = False", "K_SIM_NO_BALLISTIC = False" in src),
        ("K_SIM_IDEAL_CAMERA = False", "K_SIM_IDEAL_CAMERA = False" in src),
        ("K_TARGET_PLANE_H = -0.278", "K_TARGET_PLANE_H = -0.278" in src),
        ("补偿归零 K_ZERO_COMPENSATION = True", "K_ZERO_COMPENSATION = True" in src),
        ("D30 判据 (达标即投 / 超时强制投)", "达标即投" in src and "超时强制投" in src),
        ("D31 积分关闭 K_AIM_KI = 0.00", "K_AIM_KI          = 0.00" in src),
        ("D32 高度比例增益关闭", "height_scale = 1.0" in src),
        ("节点名 k_mission / 类 KMission", "k_mission" in src and "KMission" in src),
    ]
    print(f"板载 i.py {len(FLY_I.read_text(encoding='utf-8').splitlines())} 行"
          f" → 板载 k.py {n_main} 行")
    print("环境常量翻到实机侧:")
    for (old, why) in flips:
        print(f"  · {old.strip()}  ⇒  {why}")
    print("校验:")
    bad = 0
    for name, ok in checks:
        print(f"  {'✓' if ok else '✗'} {name}")
        bad += 0 if ok else 1

    # 语法
    try:
        compile(src, str(FLY_K), "exec")
        print("  ✓ Python 语法")
    except SyntaxError as e:
        print(f"  ✗ Python 语法: {e}")
        bad += 1

    if a.check:
        print(f"\n--check: {'全部通过' if bad == 0 else f'{bad} 项不通过'}")
        return 1 if bad else 0

    if bad:
        raise SystemExit(f"{bad} 项校验不通过, 拒绝写入")

    if FLY_K.exists():
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bak = FLY_K.with_suffix(f".py.bak.{ts}_newk")
        shutil.copy2(FLY_K, bak)
        print(f"[backup] {bak.name}")
    FLY_K.write_text(src, encoding="utf-8")
    print(f"[write] fly/src/fly/fly/k.py  {len(src.splitlines())} 行 ({len(src.encode())} B)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
