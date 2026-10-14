#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_k_report.py — 汇总 runs/ 里**所有当前代码**的 k 仿真轮次 (2026-10-13)

为什么要单独一个脚本:
  `bench_k_runs.sh` 只汇总它自己刚跑的那一批; 要看"累计多少轮/命中率/误差分布",
  得把历史轮次一起扫。判据是 mission.log 里有没有 **D36 身份门**的标记
  (即当前 k.py 版本), 旧版本的轮次自动排除。

用法:
  python3 scripts/bench_k_report.py              # 汇总全部当前代码轮次
  python3 scripts/bench_k_report.py --list       # 只列轮次清单
"""
import argparse
import json
import math
import pathlib
import re
import statistics as st
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
RUNS = ROOT / "runs"

RX_DROP = re.compile(r">>> DROP #(\d+)!.*?桶NED\(\+?(-?[\d.]+),\+?(-?[0-9.]+)\).*?残差=([\d.]+)cm")
RX_TRIG = re.compile(r"触发方式=([^\s]+)")
RX_ALIGN = re.compile(r"粗对准(\S+?): xy=([\d.]+)cm\(门\d+\) dz=([\d.]+)cm")


def load(rp):
    try:
        ml = (rp / "mission.log").read_text(errors="replace")
    except OSError:
        return None                       # 不完整/被清理的轮次
    if "D36 直径身份门" not in ml and "D36" not in ml:
        return None                       # 不是当前代码的轮次
    try:
        summ = json.load(open(rp / "summary.json"))
        truth = json.load(open(rp / "truth.json"))
    except Exception:
        return None
    rad = {b["id"]: b["radius"] for b in truth["buckets"]}
    tb = {b["id"]: (b["ned_x"], b["ned_y"]) for b in truth["buckets"]}
    est = {int(m.group(1)): (float(m.group(2)), float(m.group(3)), float(m.group(4)))
           for m in RX_DROP.finditer(ml)}
    drops = []
    for x in summ.get("drops", []):
        i = x["drop"] - 1
        r = rad.get(x["bucket"], 0.125)
        rec = dict(bucket=x["bucket"], dist=x["dist_m"] * 100, radius=r * 100,
                   zone=x["zone"], score=x["score"], a_zone=(x["dist_m"] <= r))
        if i in est and x["bucket"] in tb:
            ex, ey, resid = est[i]
            tx, ty = tb[x["bucket"]]
            rec["est_err"] = math.hypot(ex - tx, ey - ty) * 100
            imp = x.get("impact_ned")
            rec["resid"] = resid
            if imp:
                rec["resid_calc"] = math.hypot(imp[0] - ex, imp[1] - ey) * 100
        drops.append(rec)
    end = re.search(r"End reason: (\S+)", ml)
    tot = re.search(r"Total time: ([\d.]+)s", ml)
    return dict(dir=rp.name, seed=truth.get("seed"), drops=drops,
                total=summ.get("total_score", 0),
                trig=[t[:12] for t in RX_TRIG.findall(ml)],
                align=[(a[0], float(a[1]), float(a[2])) for a in RX_ALIGN.findall(ml)],
                d36=len(re.findall(r"D36 直径身份门拒收", ml)),
                end=end.group(1) if end else "?",
                dur=float(tot.group(1)) if tot else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    recs = []
    for rp in sorted(RUNS.glob("k_*")):
        r = load(rp)
        if r and r["drops"]:
            recs.append(r)
    if not recs:
        print("没有找到当前代码的轮次"); return 1

    if a.list:
        for r in recs:
            print(f"  {r['dir']:<28} seed={r['seed']} {r['total']}/800")
        return 0

    print("=" * 112)
    print(f" k 仿真汇总 — 当前代码 (含 D34 两段对准 / D35 起步斜坡 / D36 直径身份门) 共 {len(recs)} 轮")
    print("=" * 112)
    print(f"\n{'轮次':>4} {'seed':>5} {'第1投':>8} {'第2投':>8} {'合计':>9} {'触发方式':>26} "
          f"{'D36':>5} {'时长':>7} {'结束':>7}")
    print("-" * 112)
    for i, r in enumerate(recs, 1):
        ds = [f"{x['dist']:6.2f}cm" + ("" if x["a_zone"] else "!") for x in r["drops"]]
        while len(ds) < 2:
            ds.append("     --")
        tg = ",".join(r["trig"]) or "--"
        print(f"{i:>4} {r['seed']:>5} {ds[0]:>8} {ds[1]:>8} {r['total']:>5}/800 "
              f"{tg[:26]:>26} {r['d36']:>5} {r['dur']:>6.1f}s {r['end']:>7}")

    drops = [x for r in recs for x in r["drops"]]
    dist = [x["dist"] for x in drops]
    azon = [x for x in drops if x["a_zone"]]
    full = [r for r in recs if r["total"] == 800]
    print("-" * 112)
    print(f"\n  ★ **满分轮次 {len(full)}/{len(recs)} = {100*len(full)/len(recs):.0f}%**"
          f"   平均分 {st.mean([r['total'] for r in recs]):.0f}/800")
    print(f"  ★ **A 区命中 {len(azon)}/{len(drops)} = {100*len(azon)/len(drops):.0f}%**"
          f"  (A 区 = 落点 ≤ 该桶半径; 15/20/25cm 桶半径分别 7.5/10/12.5cm)")
    print(f"\n  单投落点 n={len(dist)}: 中位 **{st.median(dist):.2f}cm**  均值 {st.mean(dist):.2f}cm  "
          f"最好 {min(dist):.2f}cm  最坏 {max(dist):.2f}cm")
    for th in (3, 5, 7.5, 10, 12.5):
        print(f"    ≤{th:>4.1f}cm: {sum(1 for x in dist if x <= th):>2}/{len(dist)}"
              f"  ({100*sum(1 for x in dist if x <= th)/len(dist):>3.0f}%)")
    miss = [x for x in drops if not x["a_zone"]]
    if miss:
        print(f"\n  ✗ 未进 A 区: " + ", ".join(f"{x['dist']:.2f}cm > r{x['radius']:.1f}({x['bucket']})"
                                              for x in miss))
    # 误差拆解
    ee = [x["est_err"] for x in drops if "est_err" in x]
    rr = [x["resid"] for x in drops if "resid" in x]
    if ee and rr:
        print(f"\n  **误差拆解** (落点距真桶 = 释放残差 ⊕ 桶估计误差):")
        print(f"    · 释放残差   n={len(rr)} 中位 {st.median(rr):.2f}cm  最坏 {max(rr):.2f}cm")
        print(f"    · 桶估计误差 n={len(ee)} 中位 {st.median(ee):.2f}cm  最坏 {max(ee):.2f}cm")
        print(f"    · 合成(实测) n={len(dist)} 中位 {st.median(dist):.2f}cm")
    # 触发方式
    tg = [t for r in recs for t in r["trig"]]
    kinds = {}
    for t in tg:
        kinds[t] = kinds.get(t, 0) + 1
    print(f"\n  触发方式: " + ", ".join(f"{k}×{v}" for k, v in sorted(kinds.items())))
    al = [a[1] for r in recs for a in r["align"]]
    ald = [a[2] for r in recs for a in r["align"]]
    if al:
        print(f"  粗对准门: n={len(al)} xy 中位 {st.median(al):.2f}cm 最坏 {max(al):.2f}cm (门15) | "
              f"dz 中位 {st.median(ald):.2f}cm 最坏 {max(ald):.2f}cm (门15)")
    print(f"  D36 直径身份门: 累计拒收 {sum(r['d36'] for r in recs)} 次 "
          f"(平均 {sum(r['d36'] for r in recs)/len(recs):.1f} 次/轮)")
    print(f"  单轮时长: 中位 {st.median([r['dur'] for r in recs]):.1f}s")
    print(f"  结束状态: " + ", ".join(f"{k}×{v}" for k, v in
                                     sorted({e: sum(1 for r in recs if r['end'] == e)
                                             for e in {r['end'] for r in recs}}.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
