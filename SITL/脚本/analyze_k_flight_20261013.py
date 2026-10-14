#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_k_flight_20261013.py — 实机飞行日志分析: **能不能满足投弹的稳定条件?**

用途 (用户 2026-10-13: 先准备手动改完参数后先飞, 到时候读一下飞行日志,
看看能否满足投弹的稳定条件)
------------------------------------------------------------------------------
实机的释放判据是 **他们的**: 残差 <= 3cm 且保持 2.0s (达标即投); 否则到释放面后
2.0s 强制投; 再不行 12s 必投兜底。首飞之后要回答的就是**一句话**:

    DROP 段里「残差连续 <= 3cm 保持 2.0s」这个条件, 到底能不能达到? 差多少?

本脚本读 **节点自己的 stdout 日志**(`ros2 run fly k > k_<日期>.log`) 就能回答, 不依赖 ULog;
若同时给 `--ulog`, 再补一层**飞控侧**归因 (悬停漂移 / vxy), 用来判断「差」是估计问题还是控制问题。

用法:
    python3 scripts/analyze_k_flight_20261013.py <节点日志>
    python3 scripts/analyze_k_flight_20261013.py <节点日志> --ulog <x.ulg>
    python3 scripts/analyze_k_flight_20261013.py <节点日志> --dump
"""
import argparse
import math
import pathlib
import re
import statistics as st
import sys

# ---- 日志行格式 (与板载 k.py 的 f-string 逐字对应) ----
RX_ENTER = re.compile(r"\[K-SERVO\]\s*进入 DROP #(\d+)")
RX_SERVO = re.compile(
    r"\[K-SERVO\]\s+t=\s*([\d.]+)s\s+win=\s*([\d.]+)s\s+H=\s*([\d.]+)m\s+"
    r"残差=\s*(--|[\d.]+)cm\s+门限=(\d+)cm\s+vxy=([\d.]+)\s+"
    r"更新=(\d+)/拒绝=(\d+).*?pred=(--|[\d.]+)cm")
RX_DROP = re.compile(
    r">>> DROP #(\d+)!\s+PWM=(\d+)us.*?落点相对home=\(\s*([-+0-9.]+),\s*([-+0-9.]+)\).*?"
    r"pitch=\s*([-+0-9.]+)\s+roll=\s*([-+0-9.]+)\s+vxy=([\d.]+)m/s\s+"
    r"残差=\s*(--|[\d.]+)cm")
RX_WAY = re.compile(r"触发方式=([^\s]+)")
RX_ALIGN = re.compile(
    r"\[K-ALIGN\]\s*粗对准(完成|超时放行):\s*xy=\s*([\d.]+)cm\(门(\d+)\)\s*"
    r"dz=\s*([\d.]+)cm\(门(\d+)\)\s*稳定=\s*([\d.]+)s")

TOL_DEFAULT = 3.0
NEED_DEFAULT = 2.0


def parse(log_path):
    """把节点日志拆成: 每位 DROP 的残差/vxy/H 序列 + 释放行 + 粗对准行。"""
    txt = pathlib.Path(log_path).read_text(errors="replace")
    drops, cur = [], None
    for ln in txt.split("\n"):
        m = RX_ENTER.search(ln)
        if m:
            if cur is not None:
                drops.append(cur)
            cur = dict(idx=int(m.group(1)), samples=[], drop_line=None, way=None, align=None)
            continue
        if cur is None:
            continue
        m = RX_SERVO.search(ln)
        if m:
            cur["samples"].append(dict(
                t=float(m.group(1)),
                win=float(m.group(2)),          # ⭐ 节点**自己算的保持窗** —— 最权威的判据证据
                resid=None if m.group(4) == "--" else float(m.group(4)),
                H=float(m.group(3)), tol=float(m.group(5)), vxy=float(m.group(6)),
                pred=None if m.group(9) == "--" else float(m.group(9))))
            continue
        m = RX_ALIGN.search(ln)
        if m and cur["align"] is None:
            cur["align"] = dict(ok=(m.group(1) == "完成"), xy=float(m.group(2)),
                                dz=float(m.group(4)), stable=float(m.group(6)))
            continue
        m = RX_DROP.search(ln)
        if m:
            cur["drop_line"] = dict(idx=int(m.group(1)), pwm=int(m.group(2)),
                                    impact=(float(m.group(3)), float(m.group(4))),
                                    pitch=float(m.group(5)), roll=float(m.group(6)),
                                    vxy=float(m.group(7)),
                                    resid=None if m.group(8) == "--" else float(m.group(8)))
            # `触发方式=` 与 `>>> DROP` 在**同一行末尾** ⇒ 单独抓 (值里带空格, 抓到行尾)
            w = re.search(r"触发方式=(.+?)\s*$", ln)
            if w:
                cur["way"] = w.group(1).strip()
            continue
        m = RX_WAY.search(ln)
        if m and cur["way"] is None:
            cur["way"] = m.group(1)
    if cur is not None:
        drops.append(cur)
    return drops, txt


def longest_window(samples, tol):
    """最长「连续残差 <= tol」时长 (采样间隔 = 日志节流 0.5s)。返回 (时长, 起始 t)。"""
    best, best_t, run, run_t = 0.0, None, 0.0, None
    n = len(samples)
    for i, s in enumerate(samples):
        if s["resid"] is not None and s["resid"] <= tol:
            if run == 0.0:
                run_t = s["t"]
            run += (samples[i + 1]["t"] - s["t"]) if i + 1 < n else 0.5
            if run > best:
                best, best_t = run, run_t
        else:
            run, run_t = 0.0, None
    return best, best_t


def q(vals, p):
    if not vals:
        return None
    v = sorted(vals)
    k = (len(v) - 1) * p
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] if lo == hi else v[lo] + (v[hi] - v[lo]) * (k - lo)


def fmt(x, unit="cm", nd=2):
    return "--" if x is None else ("%.*f%s" % (nd, x, unit))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="节点 stdout 日志 (ros2 run fly k > k_<日期>.log)")
    ap.add_argument("--ulog", help="可选: 飞控 ULog, 补一层飞控侧归因")
    ap.add_argument("--dump", action="store_true", help="打印每位投的残差全序列")
    ap.add_argument("--need", type=float, default=NEED_DEFAULT, help="稳定窗要求 (s)")
    a = ap.parse_args()

    p = pathlib.Path(a.log)
    if not p.exists():
        print("✗ 日志不存在: %s" % p)
        return 1
    drops, txt = parse(p)

    print("=" * 82)
    print(" k 实机飞行日志分析 — %s" % p.name)
    print("=" * 82)

    if not drops:
        print("")
        print("  ⚠ 日志里没有 `[K-SERVO] 进入 DROP` —— 这趟**没进投放段**。")
        print("     先确认下面这些记录在不在:")
        for key, pat in (("解锁 ARM OK", r"ARM OK"), ("进 OFFBOARD", r"OFFBOARD"),
                         ("到 FORWARD", r"STEP FORWARD"), ("检出桶", r"TRK-NEW"),
                         ("粗对准", r"K-ALIGN")):
            mark = "✓" if re.search(pat, txt) else "✗"
            print("       %s %s" % (mark, key))
        return 1

    print("")
    print("  共 %d 次投放" % len(drops))

    verdicts = []
    for d in drops:
        s = [x for x in d["samples"] if x["resid"] is not None]
        res = [x["resid"] for x in s]
        tol = d["samples"][0]["tol"] if d["samples"] else TOL_DEFAULT
        win, win_t = longest_window(d["samples"], tol)
        ok = win >= a.need

        print("")
        # ⚠ `进入 DROP #N` 是 **1-based**, 而 `>>> DROP #N!` 是 **0-based** (节点自身的既有不一致)
        #   ⇒ 这里按"第几投"直接显示 `进入` 行的编号, 不要再 +1。
        print("  ── 第 %d 投 ──" % d["idx"])
        al = d["align"]
        if al:
            print("    粗对准: %s xy=%.1fcm dz=%.1fcm 稳定=%.2fs"
                  % ("完成" if al["ok"] else "**超时放行**", al["xy"], al["dz"], al["stable"]))
        dl = d["drop_line"]
        if dl:
            print("    释放: PWM=%dus 落点相对home=(%+.3f,%+.3f) pitch=%+.3f roll=%+.3f vxy=%.3f"
                  % (dl["pwm"], dl["impact"][0], dl["impact"][1], dl["pitch"], dl["roll"], dl["vxy"]))
        if d["way"]:
            print("    触发方式: **%s**" % d["way"])

        # ⚠ `win=` 是**到达后已等待的时长**(K_WINDOW_S=3.0s 之后容差会放宽), **不是**稳定窗 ——
        #   真正的判据证据是释放行里的 `触发方式=`. 两者都打出来, 但**判定只用 触发方式**。
        wins = [x["win"] for x in d["samples"]] if d["samples"] else []
        if wins:
            print("    到达后已等待 win: 最大 %.1fs (3.0s 后容差放宽 +3cm)" % max(wins))

        # ⭐ 判定: **以 `触发方式` 为准** (节点自己判的, 最权威)
        way = d["way"] or ""
        if "达标即投" in way:
            ok, why = True, "达标即投 ⇒ **门限内释放成功** ✓"
        elif ("释放面" in way) or ("强制" in way):
            ok, why = False, "走的是**释放面强制投** ⇒ 残差没能在门限内保持"
        elif "兜底" in way:
            ok, why = False, "走的是**必投兜底** ⇒ 判据完全没满足"
        elif way:
            ok, why = None, "触发方式未识别: %s" % way
        else:
            ok, why = None, "(日志里没有 `触发方式=`, 只能看残差)"
        if why:
            print("    ⇒ %s" % why)

        if not res:
            tol = d["samples"][0]["tol"] if d["samples"] else TOL_DEFAULT
            if d["samples"]:
                print("    (残差全是 `--`: 该段没有「新鲜残差」帧)")
            elif ok:
                print("    ✓ 该段**没打逐拍行** = 释放得比 0.5s 节流还快,")
                print("      通常意味着**一进 DROP 残差就在门限内** (好现象)。")
            else:
                print("    ⚠ 该段**没有逐拍行** —— 但这轮又不是达标释放, 所以不是「释放太快」;")
                print("      更可能是: ①这轮早于逐拍日志版本 ②DROP 段短到只够打释放行。")
                print("      ⇒ 这轮**无法从残差序列归因**, 只能看 触发方式 本身。")
            if ok is None:
                ok = False
            verdicts.append(dict(ok=ok, idx=d["idx"], way=d["way"], n=0, tol=tol))
            continue

        tol = d["samples"][0]["tol"]
        # 残差序列给出的"最长连续达标" —— 只作**诊断**(0.5s 节流, 分辨率粗), 判定不靠它
        lw, lw_t = longest_window(d["samples"], tol)

        print("    残差样本 %d 个 (节流 0.5s) | 中位 **%.2fcm** 最小 **%.2fcm** "
              "p25=%.2f p75=%.2f 最大 %.2f"
              % (len(res), st.median(res), min(res), q(res, .25), q(res, .75), max(res)))
        print("    门限 %.0fcm | 残差序列的**最长连续达标 %.1fs**%s (诊断用, 分辨率 0.5s)"
              % (tol, lw, (" @t=%.1fs" % lw_t) if lw_t is not None else ""))

        if min(res) > tol:
            print("    ⇒ **残差从来没进过门限**: 地板 %.2fcm, 距门限差 **%.2fcm**"
                  % (min(res), min(res) - tol))
        elif ok is False:
            print("    ⇒ 进过门限但**没保持住** (残差序列最长连续 %.1fs)" % lw)

        best = min(s, key=lambda x: x["resid"])
        print("    最优时刻 t=%.1fs 残差=%.2fcm vxy=%.3f H=%.2fm"
              % (best["t"], best["resid"], best["vxy"], best["H"]))
        if a.dump:
            parts = []
            for x in d["samples"]:
                parts.append("%.0fs:%s" % (x["t"], "--" if x["resid"] is None else "%.1f" % x["resid"]))
            print("    残差序列: " + " ".join(parts))
        verdicts.append(dict(ok=bool(ok), idx=d["idx"], way=d["way"], lw=lw,
                             med=st.median(res), mn=min(res), tol=tol))

    # ---------------- 总判定 ----------------
    ok_n = sum(1 for v in verdicts if v["ok"])
    print("")
    print("=" * 82)
    print(" 总判定")
    print("=" * 82)
    print("")
    print("  **能满足 %.0fcm/%.1fs 的投放: %d/%d**"
          % (verdicts[0].get("tol", TOL_DEFAULT), a.need, ok_n, len(verdicts)))
    ways = {}
    for v in verdicts:
        k = v["way"] or "(未记录)"
        ways[k] = ways.get(k, 0) + 1
    print("  触发方式分布: " + ", ".join("%s×%d" % (k, v) for k, v in sorted(ways.items())))

    if ok_n == len(verdicts):
        print("")
        print("  ✅ **判据完全够用** —— 实机伺服收敛质量达到了他们的 3cm/2.0s。")
        print("     首飞建议: 判据不动; 把现场**卷尺落点**与日志里的 `落点相对home` 对照,")
        print("     标定两瓶机械偏置 (见 docs/reports/k_flight_prep_20261013.md 第 6 节)。")
    elif ok_n > 0:
        print("")
        print("  ⚠ **判据部分够用** (%d/%d)。未达标那几投按两种情形分别处理:" % (ok_n, len(verdicts)))
        print("       · 残差没进过门限 ⇒ **稳态精度**问题 (估计或控制), 门限本身不该动;")
        print("       · 进过但保持不住 ⇒ **飞机停不下来** (vxy 摆动), 看 --ulog 的归因。")
    else:
        meds = [v["med"] for v in verdicts if v.get("med") is not None]
        mns = [v["mn"] for v in verdicts if v.get("mn") is not None]
        print("")
        print("  ✗ **判据偏紧** (0/%d)。" % len(verdicts))
        if meds:
            print("     残差中位 %.2fcm / 地板 %.2fcm (门限 %.0fcm)"
                  % (st.median(meds), min(mns), verdicts[0]["tol"]))
        print("     ⇒ 实机没有仿真里那种「干净」的定点 (仿真地板 ~3cm)。三步走 (按优先级):")
        print("       ① 先别改判据 —— 看 --ulog 归因: 是 vxy 摆动 (控制) 还是残差地板高 (估计);")
        print("       ② 若是摆动: 延长 DROP 段的稳定等待 / 降低下降末速 (给飞机更多停稳时间);")
        print("       ③ 若地板就是高: 3cm 永远达不到 ⇒ 只能走「释放面 2.0s 强制投」这条兜底路径,")
        print("          此时唯一能提精度的就是**在释放面那一刻挑残差最小的一帧**, 而非硬等满 2.0s。")

    # ---------------- 可选: 飞控侧归因 ----------------
    if a.ulog:
        print("")
        print("=" * 82)
        print(" 飞控侧归因 (ULog)")
        print("=" * 82)
        try:
            from pyulog import ULog
        except ImportError:
            print("  (未装 pyulog, 跳过; pip install pyulog)")
            return 0
        try:
            lp = ULog(a.ulog).get_dataset("vehicle_local_position")
            vx, vy = lp.data["vx"], lp.data["vy"]
            vxy = [math.hypot(x, y) for x, y in zip(vx, vy)]
            xs, ys = lp.data["x"], lp.data["y"]
            print("")
            print("  样本 %d 个" % len(vxy))
            print("  水平速度 vxy: 中位 %.3f 均值 %.3f p90 %.3f 最大 %.3f m/s"
                  % (st.median(vxy), st.mean(vxy), q(vxy, .9), max(vxy)))
            print("  位置漂移: x 峰峰 %.1fcm / y 峰峰 %.1fcm"
                  % ((max(xs) - min(xs)) * 100, (max(ys) - min(ys)) * 100))
            still = sum(1 for v in vxy if v < 0.15) / len(vxy) * 100
            print("  静定占比 (vxy<0.15): %.0f%%   %s"
                  % (still, "✓ 飞机停得下来" if still > 50 else "✗ 大部分时间在动 ⇒ 残差自然稳不住"))
            print("")
            print("  ⇒ 归因: 静定占比低 ⇒ 问题在**控制/等待时间**, 不在判据;")
            print("     静定占比高却仍达不到 ⇒ 问题在**估计精度** (像素/姿态/高度)。")
        except Exception as e:
            print("  ✗ 读 ULog 失败: %s" % e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
