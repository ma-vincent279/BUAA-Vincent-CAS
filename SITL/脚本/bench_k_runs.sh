#!/usr/bin/env bash
# bench_k_runs.sh — 连跑多轮 k 仿真并汇总 (2026-10-13)
#
# 用法:
#   bash scripts/bench_k_runs.sh 33 7 101           # 按顺序各跑一轮
#   bash scripts/bench_k_runs.sh 33 33 33 7 7 101   # 重复跑, 看可复现性
#
# 为什么要有这个脚本 (而不是手敲几遍):
#   1) **每轮之间必须清进程** —— 残留的 `gz sim` 会让下一轮 PX4 以为"世界已在运行",
#      报 `gz_bridge failed to start and spawn model`, 飞机根本起不来、0 投、一直挂到超时。
#   2) pkill 的模式串写进**交互式命令行**会自伤 (调用者自己的 cmdline 含该串) ⇒
#      放进脚本文件里执行才安全。
#   3) 汇总要读每轮的 summary.json + mission.log, 手抄容易错。
set -u
cd "$(dirname "$0")/.."
SITL_DIR="$PWD"

cleanup_sim() {
    # ⚠ 直接复用共享清理脚本 —— 里面**连 ROS 侧 (ros_gz_bridge/impact_gt/drone_drl) 一起清**。
    #   2026-10-13 踩过: 只清 gz/px4 时, 前几轮 GT_JUDGE 留下的 **14 个 ros_gz_bridge**
    #   一直在灌 DDS ⇒ 后续每轮 OFFBOARD 流被饿死 (解锁 24s、飞机飞 39m、Mode timeout)。
    bash "$SITL_DIR/scripts/cleanup_sim.sh"
}

seed_files() {   # $1=seed → 回显 "WORLD TRUTH GZWORLD"
    local s="$1"
    if [ "$s" = "33" ]; then
        echo "$SITL_DIR/worlds/k.world $SITL_DIR/worlds/k.truth.json k_gz"
    else
        echo "$SITL_DIR/worlds/k${s}.world $SITL_DIR/worlds/k${s}.truth.json k${s}_gz"
    fi
}

echo "============================================================"
echo " k 仿真批量跑分 — 共 $# 轮: $*"
echo "============================================================"
mkdir -p /tmp/bench_k
: > /tmp/bench_k/runs.txt

n=0
for S in "$@"; do
    n=$((n+1))
    read -r W T G <<< "$(seed_files "$S")"
    [ -f "$W" ] || { echo "  [$n] seed=$S 缺世界 $W —— 先跑 gen_world_cuadc.py"; continue; }
    echo ""
    echo "---- 第 $n/$# 轮: seed=$S ----"
    cleanup_sim
    SIM=gz WORLD_FILE="$W" TRUTH_FILE="$T" GZ_WORLD="$G" \
        bash scripts/test_k.sh > "/tmp/bench_k/run_${n}_s${S}.log" 2>&1
    R=$(ls -dt runs/k_${S}_* 2>/dev/null | head -1)
    if [ -z "$R" ] || [ ! -f "$R/summary.json" ]; then
        echo "  ✗ 本轮失败 (无 summary.json), 日志 /tmp/bench_k/run_${n}_s${S}.log"
        continue
    fi
    echo "$S|$R" >> /tmp/bench_k/runs.txt
    echo "  ✓ $R"
done

echo ""
echo "============================================================"
echo " 汇总"
echo "============================================================"
python3 - "$SITL_DIR" <<'PY'
import json, re, sys, pathlib
root = pathlib.Path(sys.argv[1])
recs = []
for line in (pathlib.Path("/tmp/bench_k/runs.txt")).read_text().split("\n"):
    if not line.strip():
        continue
    s, r = line.split("|", 1)
    rp = root / r
    try:
        d = json.load(open(rp / "summary.json"))
    except Exception:
        continue
    # 按 truth.json 的**每个桶自己的半径**判 A/B 区 (15cm 桶只有 7.5cm!)
    try:
        tj = json.load(open(rp / "truth.json"))
        rad = {b["id"]: b["radius"] for b in tj["buckets"]}
    except Exception:
        rad = {}
    ml = (rp / "mission.log").read_text(errors="replace")
    trig = re.findall(r"触发方式=([^\s]+)", ml)
    align = re.findall(r"粗对准(\S+?): xy=([\d.]+)cm\(门\d+\) dz=([\d.]+)cm", ml)
    nd36 = len(re.findall(r"D36 直径身份门拒收", ml))
    end = re.search(r"End reason: (\S+)", ml)
    tot = re.search(r"Total time: ([\d.]+)s", ml)
    nd = len(d.get("drops", []))
    # 拆解误差: 落点 − 真值桶心 = (落点 − 释放时的桶估计) + (桶估计 − 真值)
    #   ← 前者 = 释放判据的残差 (小), 后者 = **估计误差** (往往是主因)
    _est = {}
    for m2 in re.finditer(r">>> DROP #(\d+)!.*?桶NED\(\+?(-?[\d.]+),\+?(-?[0-9.]+)\)", ml):
        _est[int(m2.group(1))] = (float(m2.group(2)), float(m2.group(3)))
    _tb = {b["id"]: (b["ned_x"], b["ned_y"]) for b in tj.get("buckets", [])} if rad else {}
    for x in d.get("drops", []):
        x["_r"] = rad.get(x.get("bucket"), 0.125)
        _i = x["drop"] - 1
        _e = _est.get(_i)
        _t = _tb.get(x.get("bucket"))
        if _e and _t:
            import math as _m
            x["_esterr"] = _m.hypot(_e[0] - _t[0], _e[1] - _t[1]) * 100      # cm
            _imp = x.get("impact_ned")
            if _imp:
                x["_resid"] = _m.hypot(_imp[0] - _e[0], _imp[1] - _e[1]) * 100
        else:
            x["_esterr"] = None
    recs.append(dict(seed=int(s), dir=rp.name, drops=d.get("drops", []),
                     total=d.get("total_score", 0), trig=trig, align=align,
                     d36=nd36, end=(end.group(1) if end else "?"),
                     tot_s=(float(tot.group(1)) if tot else 0.0), nd=nd))

if not recs:
    print("  没有可用轮次"); sys.exit()

print(f"\n{'#':>2} {'seed':>5} {'第1投':>8} {'第2投':>8} {'合计':>8} {'触发方式':>22} "
      f"{'粗对准xy':>10} {'D36拒收':>8} {'时长':>7} {'结束':>8}")
print("-" * 106)
all_d, all_sc, aligns = [], [], []
for i, r in enumerate(recs, 1):
    ds = [f"{x['dist_m']*100:6.2f}cm" for x in r["drops"]]
    while len(ds) < 2:
        ds.append("     --")
    tg = ",".join(t[:10] for t in r["trig"]) or "--"
    ax = ",".join(a[1] for a in r["align"]) or "--"
    print(f"{i:>2} {r['seed']:>5} {ds[0]:>8} {ds[1]:>8} "
          f"{r['total']:>4}/{800:<3} {tg:>22} {ax:>10} {r['d36']:>8} "
          f"{r['tot_s']:>6.1f}s {r['end']:>8}")
    all_d += [x["dist_m"]*100 for x in r["drops"]]
    all_sc.append(r["total"])
    aligns += [float(a[1]) for a in r["align"]]

print("-" * 106)
import statistics as st
print(f"\n  轮次 {len(recs)} | 满分 {sum(1 for x in all_sc if x == 800)}/{len(all_sc)} "
      f"| 平均分 {st.mean(all_sc):.0f}/800")
if all_d:
    print(f"  单投落点 n={len(all_d)}: 中位 {st.median(all_d):.2f}cm  均值 {st.mean(all_d):.2f}cm  "
          f"最好 {min(all_d):.2f}cm  最坏 {max(all_d):.2f}cm")
    _z = [(x["dist_m"]*100, x["_r"]*100, x["bucket"]) for r in recs for x in r["drops"]]
    _a = [t for t in _z if t[0] <= t[1]]
    _m = [t for t in _z if t[0] > t[1]]
    print(f"  **A 区 (落点 <= 该桶半径)**: {len(_a)}/{len(_z)}" +
          ("" if not _m else "   ✗ 未进 A: " + ", ".join(
              f"{d:.2f}cm>r{r:.1f}({b})" for d, r, b in _m)))
    print(f"     (注意: **15cm 桶的半径只有 7.5cm** —— 3cm 释放容差 + 飞机游走很容易顶到边)")
    _ee = [x["_esterr"] for r in recs for x in r["drops"] if x.get("_esterr") is not None]
    _rr = [x["_resid"] for r in recs for x in r["drops"] if x.get("_resid") is not None]
    if _ee:
        print(f"\n  **误差拆解** (落点距真桶 = 释放残差 ⊕ 桶估计误差):")
        print(f"    · 释放残差 (落点 − 释放时的估计): n={len(_rr)} 中位 "
              f"{st.median(_rr):.2f}cm 最坏 {max(_rr):.2f}cm" if _rr else "")
        print(f"    · **桶估计误差 (估计 − 真值)**: n={len(_ee)} 中位 {st.median(_ee):.2f}cm "
              f"最坏 {max(_ee):.2f}cm  ← **这一项才是主因**")
        print(f"    · 两者合成 = 落点距真桶 (中位 {st.median(all_d):.2f}cm)")
if aligns:
    print(f"  粗对准 xy 误差 n={len(aligns)}: 中位 {st.median(aligns):.2f}cm  最坏 {max(aligns):.2f}cm (门 15cm)")
print(f"  D36 直径身份门: 累计拒收 {sum(r['d36'] for r in recs)} 次 "
      f"(平均每轮 {sum(r['d36'] for r in recs)/len(recs):.0f} 次)")
print(f"  触发方式分布: " + ", ".join(f"{t}={sum(1 for r in recs for x in r['trig'] if x == t)}"
                                     for t in sorted({x for r in recs for x in r['trig']})))
PY
