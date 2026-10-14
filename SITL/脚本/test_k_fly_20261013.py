#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_k_fly_20261013.py — **实机版 (fly.k) 离线验收** (2026-10-13)

为什么需要单独一个脚本 (而不是复用仿真的 test_k_servo_20261011.py)
------------------------------------------------------------------
`k` 有**两条平行血统**, 各自从自己的 `i.py` 生成:
    仿真: `src/drone_drl/drone_drl/i.py` → `.../k.py`
    实机: `fly/src/fly/fly/i.py`          → `.../k.py`
所以"实机 k 与仿真 k 的 diff"**没有意义**(差 154 行, 全是实机该有的适配:
内联 common / PID 抽帧 / GROUND_NED_Z=0 / 航向 180° 等)。
**真正该审的是两件事**:
  1. 每一版 vs **它自己的** `i.py` —— 差异必须就是那套 D1~D37 改动;
  2. **两版之间的『逻辑常量』不许漂移** —— 这条最容易出事(改了一版忘了另一版),
     所以本脚本把全部 `K_*` 常量抓出来逐项比对, 只允许 `K_TARGET_PLANE_H` 不同。

用法:
    python3 scripts/test_k_fly_20261013.py          # 全量验收
    python3 scripts/test_k_fly_20261013.py --diff   # 额外打印两版常量对照表
"""
import argparse
import difflib
import pathlib
import py_compile
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SIM_K = ROOT / "src/drone_drl/drone_drl/k.py"
SIM_I = ROOT / "src/drone_drl/drone_drl/i.py"
FLY_K = ROOT / "fly/src/fly/fly/k.py"
FLY_I = ROOT / "fly/src/fly/fly/i.py"
FLY_SH = ROOT / "fly/k.sh"
FLY_SETUP = ROOT / "fly/src/fly/setup.py"

_n_ok = _n_bad = 0


def check(name, cond):
    global _n_ok, _n_bad
    if cond:
        _n_ok += 1
        print(f"  ✓ {name}")
    else:
        _n_bad += 1
        print(f"  ✗ {name}")
    return bool(cond)


def code_only(p):
    """剥注释/空行 —— 审计"逻辑差异"而不是"注释差异"。"""
    out = []
    for line in pathlib.Path(p).read_text(encoding="utf-8").splitlines():
        s = line.split("#")[0].rstrip()
        if s.strip():
            out.append(s)
    return out


def consts(p):
    txt = pathlib.Path(p).read_text(encoding="utf-8")
    return dict(re.findall(r"^(K_[A-Z0-9_]+)\s*=\s*([-\d.]+)", txt, re.M))


def dmarkers(p):
    txt = pathlib.Path(p).read_text(encoding="utf-8")
    return set(re.findall(r"\bD(\d{1,2})\b", txt))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--diff", action="store_true", help="打印两版 K_* 常量对照表")
    a = ap.parse_args()

    fly = FLY_K.read_text(encoding="utf-8")
    sim = SIM_K.read_text(encoding="utf-8")

    print("=" * 78)
    print(" 实机版 (fly.k) 离线验收")
    print("=" * 78)

    # ---------------- A. 文件与入口 ----------------
    print("\n[A] 文件与入口")
    check("实机版 k.py 存在", FLY_K.exists())
    check("实机版 i.py 存在 (血统源)", FLY_I.exists())
    check("启动器 fly/k.sh 存在", FLY_SH.exists())
    sh = FLY_SH.read_text(encoding="utf-8") if FLY_SH.exists() else ""
    check("fly/k.sh 调的是 `ros2 run fly k`", "ros2 run fly k" in sh or "run fly k" in sh)
    st = FLY_SETUP.read_text(encoding="utf-8") if FLY_SETUP.exists() else ""
    check("setup.py 注册了 `k = fly.k:main`", "'k = fly.k:main'" in st or '"k = fly.k:main"' in st)
    check("`def main()` 存在", "def main(" in fly)
    try:
        py_compile.compile(str(FLY_K), doraise=True)
        check("Python 语法通过", True)
    except py_compile.PyCompileError as e:
        check(f"Python 语法通过 ({e})", False)

    # ---------------- B. 实机侧环境常量 ----------------
    print("\n[B] 环境常量必须翻到『实机侧』(这是仿真/实机的分水岭)")
    env = [
        ("K_SIM_NO_BALLISTIC", "False",
         "SITL 载荷无弹道; **实机有弹道** ⇒ 判据须含预测落点"),
        ("K_SIM_IDEAL_CAMERA", "False",
         "实机是真实相机/有挂载偏置 ⇒ 补偿必须生效 (仿真侧为 True)"),
        ("K_ZERO_COMPENSATION", "True",
         "D26 用户要求: 补偿一律归零 —— 与 D25 换原生 130° 画幅配套"),
        ("K_TARGET_PLANE_H", "-0.278",
         "求交平面下移到桶高; 仿真侧 = 0.0 (与检测器 ground 模式互逆)"),
        ("K_AIM_KI", "0.00",
         "D31: 关闭设定点积分 (他们没有, 我们对齐)"),
    ]
    for k, want, why in env:
        m = re.search(rf"^{k}\s*=\s*(\S+)", fly, re.M)
        got = m.group(1) if m else "(缺失)"
        check(f"{k} = {got:<8} (期望 {want}) — {why}", got == want)

    # ---------------- C. 跨版本『逻辑常量』不许漂移 ----------------
    print("\n[C] 跨版本常量一致性 (最容易出事: 改了一版忘了另一版)")
    ca, cb = consts(SIM_K), consts(FLY_K)
    only_sim = sorted(set(ca) - set(cb))
    only_fly = sorted(set(cb) - set(ca))
    check(f"两版 K_* 常量集合相同 (各 {len(ca)}/{len(cb)} 个; 仅 sim {only_sim or '∅'}; 仅 fly {only_fly or '∅'})",
          not only_sim and not only_fly)
    diff = sorted(k for k in set(ca) & set(cb) if ca[k] != cb[k])
    check(f"仅允许 K_TARGET_PLANE_H 不同 ⇒ 实际不同: {diff or '∅'}",
          diff == ["K_TARGET_PLANE_H"])
    # 逐条点名几个"改了就出事"的
    for k, want, why in (("K_DIA_IDENTITY_TOL", "0.03", "D36 直径身份门"),
                         ("K_FWD_RAMPIN_M", "3.0", "D35 起步斜坡"),
                         ("K_COARSE_ERR_M", "0.15", "D34 粗对准门"),
                         ("K_FINE_ERR_M", "0.08", "D34 细对准门"),
                         ("K_RELEASE_MIN_M", "0.03", "他们 3cm 释放门"),
                         ("K_RELEASE_CONFIRM", "2.00", "他们 2.0s 释放面强制投"),
                         ("K_DEADLINE_S", "12.0", "必投兜底")):
        check(f"{k} = {cb.get(k, '(缺失)')} (期望 {want}) — {why}", cb.get(k) == want)
    if a.diff:
        print("\n    ---- 两版 K_* 常量对照 ----")
        for k in sorted(set(ca) | set(cb)):
            mark = "  " if ca.get(k) == cb.get(k) else "≠ "
            print(f"    {mark}{k:<24} sim={ca.get(k, '—'):<8} fly={cb.get(k, '—')}")

    # ---------------- D. D1~D37 两版都在 ----------------
    print("\n[D] D1~D37 改动标记两版都要有 (防血统漂移)")
    ms, mf = dmarkers(SIM_K), dmarkers(FLY_K)
    missing_sim = sorted(mf - ms)
    missing_fly = sorted(ms - mf)
    check(f"标记集合一致 (sim {len(ms)} / fly {len(mf)}; 仅 sim {missing_sim or '∅'}; 仅 fly {missing_fly or '∅'})",
          not missing_sim and not missing_fly)
    for d, why in (("34", "两段对准门 (粗 15cm/细 8cm)"),
                   ("35", "FORWARD 起步加速斜坡 (消抖)"),
                   ("36", "直径身份门 (防串桶)"),
                   ("37", "透视补偿分析 (反推直径用垂直高度)")):
        check(f"两版都有 D{d} 标记 — {why}", d in ms and d in mf)

    # ---------------- E. 实机专属适配 (该有的要有) ----------------
    print("\n[E] 实机专属适配 —— 该有的必须有")
    check("GROUND_NED_Z = 0.0 (实机无 Gazebo 生成点偏移)",
          re.search(r"^GROUND_NED_Z\s*=\s*0\.0", fly, re.M) is not None)
    check("**不**依赖 drone_drl.common (板载自包含移植性)",
          "from drone_drl.common import" not in fly)
    check("内联了 QOS_BE (板载自包含)", "QOS_BE = QoSProfile(" in fly)
    check("内联了 send_arm (板载自包含)", "def send_arm(" in fly)
    check("有 PID 抽帧 (板载 CPU 保护)",
          "PID_DECIMATION" in fly and re.search(r"PID_DECIMATION_SERVO\s*=\s*\d+", fly) is not None)
    hd = re.search(r"^HEADING_DEG\s*=\s*([\d.]+)", fly, re.M)
    check(f"HEADING_DEG 是场地值 ({hd.group(1) if hd else '缺失'}°) — 实机固定航向",
          hd is not None and float(hd.group(1)) > 0)
    check("有 MISSION_REVISION (可核对板载版本)",
          re.search(r'^MISSION_REVISION\s*=\s*"', fly, re.M) is not None)

    # ---------------- F. 危险项 ----------------
    print("\n[F] 危险项 (这些出现在实机版上就是 bug)")
    # ⚠ 这些一律**剥注释后再查** —— 注释里提到某个词是无害的 (实测 `truth.json` 只在
    #   3 条注释里出现, 一开始搜原文导致假失败)。
    fly_code = "\n".join(code_only(FLY_K))
    check("实机版**没有**把 K_SIM_NO_BALLISTIC 留成 True",
          re.search(r"^\s*K_SIM_NO_BALLISTIC\s*=\s*True", fly_code, re.M) is None)
    check("实机版**没有**把 K_SIM_IDEAL_CAMERA 留成 True",
          re.search(r"^\s*K_SIM_IDEAL_CAMERA\s*=\s*True", fly_code, re.M) is None)
    check("实机版**没有** sim_gt_direct / 仿真专用订阅 (代码里)",
          "sim_gt_direct" not in fly_code and "/bucket/synthetic_image" not in fly_code)
    check("实机版**没有**硬编码 SITL 绝对路径 (代码里)",
          "/home/vincent/SITL" not in fly_code)
    check("实机版**没有** truth.json 依赖 (代码里; 实机无真值文件)",
          "truth.json" not in fly_code)
    check("实机版**没有** 仿真专用话题 (/drop/impact_gt 是诊断链的)",
          "/drop/impact_gt" not in fly_code)

    # ---------------- G. 血统审计: k vs 自己的 i ----------------
    print("\n[G] 血统审计: 每一版 k 与**它自己的** i.py 的差异规模")
    for name, i_p, k_p in (("仿真", SIM_I, SIM_K), ("实机", FLY_I, FLY_K)):
        if not i_p.exists():
            check(f"{name}: 血统源 {i_p.name} 存在", False)
            continue
        ci, ck = code_only(i_p), code_only(k_p)
        d = [l for l in difflib.unified_diff(ci, ck, lineterm="", n=0)
             if l[:1] in "+-" and l[:2] not in ("++", "--")]
        n_add = sum(1 for l in d if l[0] == "+")
        n_rem = sum(1 for l in d if l[0] == "-")
        check(f"{name}: {i_p.name}({len(ci)} 行) → {k_p.name}({len(ck)} 行), "
              f"改动 {len(d)} 行 (+{n_add}/-{n_rem}) — 即那套 D1~D37",
              len(d) > 0 and n_add >= 400)
    # 两版改动规模应接近 (血统同一套 D 改动)
    def dsize(i_p, k_p):
        ci, ck = code_only(i_p), code_only(k_p)
        return len([l for l in difflib.unified_diff(ci, ck, lineterm="", n=0)
                    if l[:1] in "+-" and l[:2] not in ("++", "--")])
    if SIM_I.exists() and FLY_I.exists():
        ds, df = dsize(SIM_I, SIM_K), dsize(FLY_I, FLY_K)
        check(f"两版的『D 改动规模』接近 (仿真 {ds} 行 / 实机 {df} 行, 差 {abs(ds-df)})",
              abs(ds - df) <= 0.15 * max(ds, df))

    print("\n" + "=" * 78)
    print(f" 结果: {_n_ok} 项通过 / {_n_bad} 项失败")
    print("=" * 78)
    return 1 if _n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
