#!/usr/bin/env python3
"""分析 f_r3 (外接盘 "f r3" SITL 导入版) 的仿真日志.

用法:
  python3 scripts/analyze_f_r3_sitl_20261007.py 10.7/f_r3_mission_*.log \
          [--judge 10.7/f_r3_judge.log]

输出 (逐投):
  · 低空段时长 / 1Hz 样本数
  · raw error 首值·最小·中位·最大 (cm) 与"最小值出现在第几个样本"
  · vxy 中位/最大 + 准静止(<0.10)占比
  · braking=1 占比 + LOW-BRAKE hold/resume 事件序列
  · 释放行 (reason / H / vxy / raw) + 舵机脉冲保持时长 + DROP 段耗时
汇总: 选桶依据 / 阶段耗时 / End reason / 总时长 / 侦察 / 返航判稳 / 着陆距 home / 裁判分数

设计说明 (为什么这么切窗口): LOW-SERVO 1Hz 行不带投号, 因此用 STEP 转换时间戳
(`PRE_SERVO->VISUAL_SERVO` 到 `DROP->CLIMB`) 作为第 n 投低空段的边界 —— 与
`_pub_low_alt_position` 的实际生效区间一致 (VISUAL_SERVO 与 DROP 共用同一个低空伺服).
"""
import argparse
import glob
import re
import statistics as st
import sys

TS_RE = re.compile(r"\[INFO\] \[(\d+\.\d+)\]")
STEP_RE = re.compile(r"STEP (\w+)->(\w+) \(prev ([\d.]+)s\)")
LS_RE = re.compile(
    r"LOW-SERVO raw=([\d.]+)m filtered=([\d.]+)m pred=([\d.]+)m "
    r"target_rate=\(([-+\d.]+),([-+\d.]+)\)m/s vxy=([\d.]+)m/s age=([\d.]+)s braking=(\d)")
DROP_RE = re.compile(
    r">>> DROP B(\d) outlet=(\d)! PWM=(\d+)us reason=(\w+) H=([\d.]+)m "
    r"vxy=([\d.]+)m/s raw=([\d.]+)m")
PULSE_RE = re.compile(r"DROP pulse complete after ([\d.]+)s")
BR_RE = re.compile(r"LOW-BRAKE (hold|resume)")
SEL_RE = re.compile(r"Selection basis \(all (\d+) confirmed[^)]*\): (.*)")
JUDGE_RE = re.compile(r"距离: ([\d.]+)m, 得分: (\d+)")


def ts(line):
    m = TS_RE.search(line)
    return float(m.group(1)) if m else None


def analyse(path, judge_path=None):
    lines = open(path, encoding="utf-8", errors="replace").read().split("\n")
    print(f"\n{'='*78}\n日志: {path}  ({len(lines)} 行)")
    for l in lines:
        if "Build revision=" in l:
            print("  " + l.split("]: ", 1)[-1])
        elif "F Mission ready" in l:
            print("  " + l.split("]: ", 1)[-1])
    if any("Detector has no frame_seq" in l for l in lines):
        print("  ⚠ 检测器无 frame_seq ⇒ 低空伺服/精投被禁用 (本次结论作废)")
    else:
        print("  ✓ 无 'Detector has no frame_seq' 警告 ⇒ 检测器发了 frame_seq")

    steps = [(ts(l), m.group(1), m.group(2), float(m.group(3)))
             for l in lines if (m := STEP_RE.search(l))]
    drops = [(ts(l),) + tuple(m.groups()) for l in lines if (m := DROP_RE.search(l))]
    pulses = [(ts(l), float(m.group(1))) for l in lines if (m := PULSE_RE.search(l))]
    brakes = [(ts(l), m.group(1)) for l in lines if (m := BR_RE.search(l))]
    lowservo = [(ts(l),) + tuple(float(m.group(i)) for i in range(1, 9))
                for l in lines if (m := LS_RE.search(l))]

    for l in lines:
        if (m := SEL_RE.search(l)):
            print(f"\n选桶: {m.group(1)} 个已确认 ⇒ {m.group(2)}")

    vs = [s for s in steps if s[1] == "PRE_SERVO" and s[2] == "VISUAL_SERVO"]
    cl = [s for s in steps if s[1] == "DROP" and s[2] == "CLIMB"]
    for i, d in enumerate(drops):
        a = vs[i][0] if i < len(vs) else None
        b = cl[i][0] if i < len(cl) else float("inf")
        rows = [r for r in lowservo if a is not None and a <= r[0] <= b]
        raw = [r[1] for r in rows]
        vxy = [r[6] for r in rows]
        print(f"\n===== 第 {d[1]} 投 (outlet {d[2]}, PWM {d[3]}us) — "
              f"reason={d[4]} H={d[5]}m vxy={d[6]}m/s raw={float(d[7])*100:.1f}cm")
        if rows:
            span = b - a if b != float("inf") else float("nan")
            print(f"  低空段 {span:.1f}s, {len(rows)} 个 1Hz 样本")
            print(f"  raw error: 首 {raw[0]*100:.1f} 最小 {min(raw)*100:.1f} "
                  f"中位 {st.median(raw)*100:.1f} 最大 {max(raw)*100:.1f} cm  "
                  f"(最小值在第 {raw.index(min(raw))+1}/{len(raw)} 个样本)")
            print(f"  vxy      : 中位 {st.median(vxy):.3f} 最大 {max(vxy):.3f} m/s "
                  f"(准静止<0.10 占 {sum(v < 0.10 for v in vxy)/len(vxy)*100:.0f}%)")
            print(f"  braking=1: {sum(r[8] for r in rows)/len(rows)*100:.0f}%")
            print(f"  最后 4 样本 raw={[round(x*100,1) for x in raw[-4:]]} "
                  f"vxy={[round(x,3) for x in vxy[-4:]]}")
        else:
            print("  (无 LOW-SERVO 样本)")
        br = [x[1] for x in brakes if a is not None and a <= x[0] <= b]
        print(f"  LOW-BRAKE 事件: {br if br else '无'}")
        pl = [p[1] for p in pulses if p[0] >= (d[0] or 0)]
        print(f"  释放脉冲保持: {pl[0]:.2f}s" if pl else "  释放脉冲: 未见完成行")
        dp = [s for s in steps if s[0] > (d[0] or 0) and s[1] == "DROP"]
        if dp:
            print(f"  DROP 段耗时: {dp[0][3]:.1f}s")

    print("\n----- 阶段耗时 -----")
    print("  " + " | ".join(f"{s[1]}->{s[2]} {s[3]:.1f}s" for s in steps))
    for l in lines:
        if "End reason:" in l or "Total time:" in l:
            print("  " + l.split("]: ", 1)[-1])
    if judge_path:
        print("\n----- 裁判 -----")
        try:
            jl = open(judge_path, encoding="utf-8", errors="replace").read().split("\n")
            names, scores = [], []
            for l in jl:
                if "RESULT" in l:
                    names.append(l.split("RESULT", 1)[1].strip().rstrip("=").strip())
                if (m := JUDGE_RE.search(l)):
                    print(f"  {names[-1] if names else '?'}: 距离 {m.group(1)}m → {m.group(2)} 分")
                    scores.append(int(m.group(2)))
            if scores:
                print(f"  总分: {sum(scores)}/{10*len(scores)}")
        except OSError as e:
            print(f"  (读不到裁判日志: {e})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--judge", default=None)
    a = ap.parse_args()
    files = []
    for p in a.logs:
        files.extend(sorted(glob.glob(p)) or [p])
    for f in files:
        analyse(f, a.judge)
    return 0


if __name__ == "__main__":
    sys.exit(main())
