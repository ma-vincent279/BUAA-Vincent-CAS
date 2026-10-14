#!/usr/bin/env python3
"""第 1 步 / 只读回放 — 冻结瞬间离轴距离 ρ_est 与"目标平面"误差预算 (2026-10-09)

为什么做
--------
P0 已证明: 任务节点 `_roi_to_ned` 用 `H = home_z - z` 与射线求交, 交在**起飞原点平面**,
而相机像素来自**桶口**。两者相差一个未知的平面偏移 `h` (SITL 里 = 0.455m):

    est = ρ_true · H / (H + h)          ⇒ 估计被乘以 H/(H+h)
    一次移动后的残差 = ρ_est · h / H     (ρ_est = 日志里的 |delta|)

i 模式 = "1.2m 冻结一次 + 开环飞过去", 所以这项 **1:1 进落点**。
本脚本把"冻结瞬间离轴距离"这个从未被统计过的量, 从全部历史日志里量出来,
并给出**不同 h 下的误差预算**, 供判断这一项占总误差多少。

重要 (必须说清楚)
----------------
`h` 是**硬件事实**, 不是常数: `h = 落地时相机离地高 − 桶口离地高`。
  · SITL: 相机在模型原点 (生成点 z=0.83), 桶口 0.375 ⇒ **h = +0.455**
  · 实机: 相机约在机腹 ~0.15m、竞赛桶口 ~0.30m ⇒ **h ≈ −0.15** (符号可能相反!)
⇒ 本脚本**同时**给出 h=+0.455 / 0 / −0.15 三档, 并明确标出"哪一档与现场实测吻合"
  需要一次 10 分钟的静态标定才能定; **不能用 SITL 的 h 直接推实机**。

用法: python3 scripts/analyze_freeze_offset_20261009.py [--verbose]
"""
import argparse
import glob
import math
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
H_FREEZE = 1.2          # 冻结高度 (I_FREEZE_AT_H=0.0 ⇒ 固定 1.2m), 单位 m

# --- 现场卷尺实测落点 (从 CLAUDE.md 各"追加"条目抄录; 单位 m; 机体坐标 前+/右+) ---
#   键 = 日志文件名片段; 值 = [(drop_idx, 前向, 右向, 说明)]
FIELD_TRUTH = {
    "20261007_100630": [(1, None, None, "第1投进桶, 偏差 3cm (追加10)")],
    "20261007_110059": [(1, None,  0.05, "B1 偏右 5cm (追加12)"),
                        (2, None, -0.141, "B2 偏左 14.1cm (追加12)")],
    "20261007_115517": [(1, None, None, "第1投命中 (追加13)"),
                        (2, 0.00, -0.130, "B2 偏左 13.0cm, 前后≈0 (追加13)")],
    "20261007_130234": [(1, None, None, "B1 失准 (单帧冻结, 追加14)"),
                        (2, -0.09, 0.09, "B2 偏右 9cm 偏下 9cm (追加14)")],
    "20261007_141457": [(1, -0.11, None, "偏前 11cm (追加17)"),
                        (2, -0.12, None, "偏前 12cm (追加17)")],
    "20261007_160201": [(1, None,  0.05, "B1 偏右 5cm (追加19)"),
                        (2, 0.05,  0.05, "B2 偏右 5cm 偏下 5cm (追加19)")],
    "20261007_162730": [(1, -0.14, -0.15, "偏左 15cm 偏后 14cm (追加20)"),
                        (2, -0.15, -0.14, "偏左 14cm 偏后 15cm (追加20)")],
}

RE_FREEZE = re.compile(
    r"一次性修正 delta=\(([-+0-9.]+),([-+0-9.]+)\)m"
    r"(?: \(桶估计=\(([-+0-9.]+),\s*([-+0-9.]+)\)\))?"
    r"(?: → 冻结保持点 \(([-+0-9.]+),([-+0-9.]+)\))?")
RE_COMP = re.compile(r"补偿B(\d)=\(([-+0-9.]+),([-+0-9.]+)\)")
RE_DROP = re.compile(
    r">>> DROP #(\d+)!.*?NED\(([-+0-9.]+),([-+0-9.]+),([-+0-9.]+)\).*?"
    r"到位err=([0-9.]+)cm pred=([0-9.]+)cm 停留([0-9.]+)s vxy=([0-9.]+)m/s")
RE_HOME = re.compile(r"Home: NED\(([-+0-9.]+),([-+0-9.]+),([-+0-9.]+)\)")
RE_HDG = re.compile(r"hdg=([-+0-9.]+)")


def parse_log(path):
    """返回 (home, [freeze...], [drop...])。"""
    home = None
    freezes, drops = [], []
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    last_hdg = None
    for ln in lines:
        if home is None:
            m = RE_HOME.search(ln)
            if m:
                home = tuple(float(x) for x in m.groups())
        m = RE_HDG.search(ln)
        if m:
            last_hdg = math.radians(float(m.group(1)))
        if "一次性修正 delta=" in ln:
            m = RE_FREEZE.search(ln)
            if not m:
                continue
            dx, dy = float(m.group(1)), float(m.group(2))
            est = (float(m.group(3)), float(m.group(4))) if m.group(3) else None
            hold = (float(m.group(5)), float(m.group(6))) if m.group(5) else None
            c = RE_COMP.search(ln)
            comp = (float(c.group(2)), float(c.group(3))) if c else None
            freezes.append({"delta": (dx, dy), "rho": math.hypot(dx, dy),
                            "est": est, "hold": hold, "comp": comp,
                            "hdg": last_hdg, "nsamp": None})
        m = RE_DROP.search(ln)
        if m:
            drops.append({"idx": int(m.group(1)),
                          "pos": (float(m.group(2)), float(m.group(3)), float(m.group(4))),
                          "err": float(m.group(5)) / 100.0,
                          "pred": float(m.group(6)) / 100.0,
                          "dwell": float(m.group(7)), "vxy": float(m.group(8))})
    return home, freezes, drops


def field_truth_for(path):
    name = Path(path).stem
    for key, val in FIELD_TRUTH.items():
        if key in name:
            return dict((d, (f, r, note)) for d, f, r, note in val)
    return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logs = sorted(glob.glob(str(ROOT / "10.*/i_mission_*.log")))
    if not logs:
        sys.exit("找不到 i_mission 日志")

    rows = []
    for path in logs:
        home, freezes, drops = parse_log(path)
        if not freezes and not drops:
            continue
        ft = field_truth_for(path)
        for i, fz in enumerate(freezes):
            d = drops[i] if i < len(drops) else None
            rows.append({
                "log": Path(path).stem, "drop": i + 1,
                "rho": fz["rho"], "delta": fz["delta"], "comp": fz["comp"],
                "hdg": fz["hdg"], "err": d["err"] if d else None,
                "pred": d["pred"] if d else None,
                "ft": ft.get(i + 1),
            })

    print("=" * 96)
    print("第 1 步 — 冻结瞬间离轴距离 ρ_est 与目标平面误差预算 (只读回放, 2026-10-09)")
    print(f"  日志 {len(logs)} 份, 冻结事件 {len(rows)} 个   (冻结高度 H = {H_FREEZE} m)")
    print("=" * 96)

    if not rows:
        sys.exit("没有解析到冻结事件")

    rhos = [r["rho"] for r in rows]
    print("\n[A] ρ_est 分布 (冻结瞬间「桶估计 − 飞机」的水平距离)")
    print(f"    中位 {statistics.median(rhos)*100:5.1f}cm   均值 {statistics.fmean(rhos)*100:5.1f}cm"
          f"   最小 {min(rhos)*100:5.1f}cm   最大 {max(rhos)*100:5.1f}cm")
    hist = {}
    for v in rhos:
        b = int(v * 100 // 10) * 10
        hist[b] = hist.get(b, 0) + 1
    for b in sorted(hist):
        print(f"    {b:3d}-{b+10:3d}cm  {'#' * hist[b]:<20} {hist[b]}")

    print("\n[B] 目标平面误差预算:  残差 = ρ_est · h / H   (h = 相机落地高 − 桶口离地高)")
    print("    h 是硬件事实, SITL=+0.455, 实机待测; 三档并排给出:")
    hs = [("SITL  +0.455", 0.455), ("零(不生效) 0.000", 0.0), ("实机估值 −0.150", -0.15)]
    print(f"    {'日志':<26}{'投':>3}{'ρ_est':>8}" +
          "".join(f"{n:>20}" for n, _ in hs) + f"{'到位err':>9}{'pred':>8}")
    for r in rows:
        line = f"    {r['log'][13:]:<26}{r['drop']:>3}{r['rho']*100:7.1f}cm"
        for _, h in hs:
            line += f"{r['rho']*h/H_FREEZE*100:19.1f}cm"
        line += (f"{(r['err']*100 if r['err'] is not None else float('nan')):8.1f}cm"
                 f"{(r['pred']*100 if r['pred'] is not None else float('nan')):7.1f}cm")
        print(line)

    print("\n[C] 现场卷尺实测 vs 平面模型预测")
    print("    推导: 飞机起点 A, 桶真值 B, 估计 est = A→B 的测量值 = ρ_est, "
          "飞机飞到 L = A + est + comp")
    print("          ⇒ 落点−桶心 = est + comp − ρ_true = comp − ρ_est·(h/H)   (ρ_true = ρ_est(H+h)/H)")
    fitted = []
    for r in rows:
        if not r["ft"] or r["hdg"] is None:
            continue
        fwd_t, right_t, note = r["ft"]
        measured = []
        if fwd_t is not None:
            measured.append(("前", fwd_t))
        if right_t is not None:
            measured.append(("右", right_t))
        if not measured:
            continue
        # delta 在机体前/右方向的分量
        dx, dy = r["delta"]
        c, s = math.cos(r["hdg"]), math.sin(r["hdg"])
        d_fwd = dx * c + dy * s
        d_right = -(dx * s) + dy * c
        comp_fwd = comp_right = 0.0
        if r["comp"]:
            Y, XF = r["comp"]
            comp_fwd = -XF
            comp_right = -Y
        pred = {}
        for name, h in (("SITL+0.455", 0.455), ("实机−0.150", -0.15)):
            pred[name] = {"前": comp_fwd - d_fwd * h / H_FREEZE,
                          "右": comp_right - d_right * h / H_FREEZE}
        seg = f"    {r['log'][13:]:<22}#{r['drop']}  ρ={r['rho']*100:5.1f}cm  "
        seg += "  ".join(f"实测{m}{v*100:+6.1f}cm" for m, v in measured)
        seg += "   "
        for name in ("SITL+0.455", "实机−0.150"):
            seg += f"|{name}: " + " ".join(f"{m}{pred[name][m]*100:+5.1f}" for m, _ in measured) + " "
        print(seg)
        fitted.append((r, measured, pred))

    if fitted:
        print("\n    拟合 h (模型值随 h 线性: 落点−桶心 = comp − d_ax·(h/H)):")

        def solve(r, axis):
            """返回 (d_ax, comp_ax) 或 None。"""
            if r["hdg"] is None:
                return None
            dx, dy = r["delta"]
            c, s = math.cos(r["hdg"]), math.sin(r["hdg"])
            d_ax = (dx * c + dy * s) if axis == "前" else (-(dx * s) + dy * c)
            comp_ax = 0.0
            if r["comp"]:
                Y, XF = r["comp"]
                comp_ax = -XF if axis == "前" else -Y
            return d_ax, comp_ax

        for axis in ("前", "右"):
            num = den = 0.0
            n_used = 0
            for r, measured, _ in fitted:
                mv = dict(measured).get(axis)
                if mv is None:
                    continue
                sc = solve(r, axis)
                if sc is None or abs(sc[0]) < 0.05:   # 该轴几乎无分量 ⇒ 解不出 h
                    continue
                d_ax, comp_ax = sc
                # mv = comp_ax − d_ax·(h/H)  ⇒  h = H·(comp_ax − mv)/d_ax
                num += d_ax * (comp_ax - mv)
                den += d_ax * d_ax
                n_used += 1
            if den > 1e-9:
                h_fit = (num / den) * H_FREEZE
                print(f"      {axis}向: h = {h_fit:+.3f} m   (n={n_used}; SITL 参考 +0.455)")
            else:
                print(f"      {axis}向: 样本不足, 无法拟合")

        print("\n    逐点反解 h 的散布 (若平面项主导, 同轴内应彼此一致):")
        for axis in ("前", "右"):
            hs = []
            for r, measured, _ in fitted:
                mv = dict(measured).get(axis)
                if mv is None:
                    continue
                sc = solve(r, axis)
                if sc is None or abs(sc[0]) < 0.05:
                    continue
                d_ax, comp_ax = sc
                hs.append(H_FREEZE * (comp_ax - mv) / d_ax)
            if hs:
                med = statistics.median(hs)
                mad = statistics.median([abs(v - med) for v in hs])
                print(f"      {axis}向: n={len(hs)} 中位 h={med:+.3f} m, MAD={mad:.3f} m"
                      f"  (范围 {min(hs):+.2f} … {max(hs):+.2f})")

    print("\n[D] 结论 (含对本假设的否证, 必须如实记录)")
    print("    ✅ 1) ρ_est (冻结离轴距离) 是实测量, 从未被统计过 —— 见 [A]: 中位 23.6cm / 最大 57.1cm。")
    print("           i 模式的\"一次性修正\"其实是一次 ~24cm 的平移, 这是任何按比例误差的放大器。")
    print("    ✅ 2) 平面项量级与残差同阶: SITL h=+0.455 时 4~22cm, 实机 h=−0.15 时 1~7cm —— 不能忽略。")
    print("    ❌ 3) **但平面项不是主导项, 本假设被数据否证**: 见 [C] 逐点反解 h 的散布 ——")
    print("          前向 MAD=1.06m (范围 −1.33…+1.80), 右向 MAD=0.49m (范围 −2.12…+0.66)。")
    print("          h 是硬件常数 (量级必须落在 −0.3…+0.8m), 而逐点反解散布 ±1m ⇒ **模型解释不了落点**。")
    print("          两轴拟合还给出相反符号 (前 +0.32 / 右 −0.29) ⇒ 更不可能是同一个物理 h。")
    print("    ⇒ **修正 2026-10-09 早先的口头结论**: 我之前说\"这一项很可能是当前最大单项误差\"——")
    print("      量级上成立, 但\"主导\"是错的。真实落点由 ρ_est(平面) + 姿态项 + 弹道漂移 + comp 残余")
    print("      + dev(参照漂移) **多项目同量级叠加**, 单靠落点数据无法把任何一项分开。")
    print("    ⇒ 4) 因此 P1 (把求交平面改成桶口) 应诚实定位为 **\"消掉一个已知系统项 + 让 SITL 几何自洽\"**,")
    print("          而不是\"提精度的大招\"; 它的精确贡献只能靠离线确定性验收量, 不能靠飞行数据估。")
    print("    ⇒ 5) 决定性的一步仍是 **10 分钟静态标定** (卸桨/桶摆正下方/读 H 与 est):")
    print("          h = H·(ρ_true/est − 1)。这是唯一能把 h 定的方法, 也是 CUADC 那套标定在做的事。")
    print("=" * 96)


if __name__ == "__main__":
    main()
