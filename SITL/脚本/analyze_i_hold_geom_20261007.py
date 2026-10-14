#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026-10-07 方案 A 验收/对比脚本: i 的冻结保持点 (纯几何量) 落地前后的落点误差分解

用法:
    python3 scripts/analyze_i_hold_geom_20261007.py \
        --log /tmp/i_sel_output.log --judge /tmp/i_sel_judge.log --label "改前(A 之前)"

对每投输出 (全部量都能从日志逐条对上):
  桶估计 est / 冻结保持点 hold / 补偿 comp = hold − est (应 = 两瓶补偿 0.139m)
  瞄准点 aim (冻结前最后一帧 tracker 瞄准点, 1Hz 日志) / 飞机@冻结 = est − delta
  lag  = aim − 飞机@冻结      ← "追赶余量": 改 A 前它 1:1 进入 hold, 改 A 后不进入
  est 误差 = est − 桶(裁判真值)
  b    = 释放点 − hold (含沿飞行方向分量)   ← 释放瞬间飞机自己没停准
  判据距离 = |释放点 − 桶|   ← 裁判口径 (SITL = 释放瞬间飞机地面投影)
  可归因残差 = |释放点 − 桶 − comp| = |est误差 + b| ← 扣掉"故意挪开"的补偿后真正能改进的部分

SITL 说明: 仿真是"飞机在哪落点就在哪"(无机械偏置), 补偿在实机上才是抵消机械偏置的;
          跨轮比较看"可归因残差"与"合计分", 不要直接拿裁判距离当实机精度.
"""
import argparse
import math
import re
import sys


def _lines(path):
    """按 \\n 与 \\r 双分隔读行 (控制台日志里混有 pxh 的 \\r 刷屏, 不能用 readlines)"""
    with open(path, encoding="utf-8", errors="replace", newline="") as f:
        data = f.read()
    for chunk in data.replace("\r", "\n").split("\n"):
        if chunk:
            yield chunk


def parse_log(path):
    freezes, drops = [], []
    last_aim = None
    last_ac = None
    last_hdg = 0.0
    for line in _lines(path):
        m = re.search(r"NED\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\) H=.*?hdg=\s*([-\d.]+).*tgt=NED\(([-\d.]+),([-\d.]+)\)", line)
        if m:
            last_ac = (float(m.group(1)), float(m.group(2)))
            last_hdg = float(m.group(4))
            last_aim = (float(m.group(5)), float(m.group(6)))
        m = re.search(r"i模式: 1\.2m 一次性修正 delta=\(([+-][\d.]+),([+-][\d.]+)\)m "
                      r"\(桶估计=\(([\d.eE+-]+), ([\d.eE+-]+)\)\) → 冻结保持点 "
                      r"\(([+-][\d.]+),([+-][\d.]+)\) "
                      r"(?:\[纯几何: 保持点-桶估计=\(([+-][\d.]+),([+-][\d.]+)\)\] )?"
                      r"补偿B(\d)=\(([+-][\d.]+),([+-][\d.]+)\)", line)
        if m:
            g = m.groups()
            freezes.append(dict(
                delta=(float(g[0]), float(g[1])),
                est=(float(g[2]), float(g[3])),
                hold=(float(g[4]), float(g[5])),
                comp_log=(float(g[6]), float(g[7])) if g[6] else None,
                bucket_idx=int(g[8]),
                comp_cmd=(float(g[9]), float(g[10])),
                aim=last_aim, ac=last_ac, hdg=last_hdg,
            ))
            continue
        m = re.search(r">>> DROP #(\d+)! PWM=(\d+)us NED\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
                      r"触发方式=(.*)$", line)
        if m:
            g = m.groups()
            tail = g[5]

            def num(pat):
                mm = re.search(pat, tail)
                return float(mm.group(1)) if mm else float("nan")

            drops.append(dict(
                idx=int(g[0]), pwm=int(g[1]),
                ned=(float(g[2]), float(g[3])),
                err_cm=num(r"到位err=([\d.]+)cm"),
                pred_cm=num(r"pred=([\d.]+)cm"),
                dwell=num(r"停留([\d.]+)s"),
                vxy=num(r"vxy=([\d.]+)m/s"),
            ))
    return freezes, drops


def parse_judge(path):
    out = []
    for line in _lines(path):
        m = re.search(r"距离: ([\d.]+)m, 得分: (\d+)", line)
        if m:
            out.append(dict(dist=float(m.group(1)), score=int(m.group(2)), bucket=None))
            continue
        m = re.search(r"桶 NED\(([-\d.]+),([-\d.]+)\)", line)
        if m and out and out[-1]["bucket"] is None:
            out[-1]["bucket"] = (float(m.group(1)), float(m.group(2)))
    return out


def vec(a, b):
    return (a[0] - b[0], a[1] - b[1])


def norm(v):
    return math.hypot(v[0], v[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--judge", required=True)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    freezes, drops = parse_log(a.log)
    judges = parse_judge(a.judge)
    print("=" * 104)
    print("轮次: %s   (%s)" % (a.label or "-", a.log))
    print("冻结 %d 次 / 投放 %d 次 / 裁判 %d 条" % (len(freezes), len(drops), len(judges)))
    print("=" * 104)
    tot_res, tot_score, nres = 0.0, 0, 0
    for i, (fz, dr) in enumerate(zip(freezes, drops)):
        jd = judges[i] if i < len(judges) else dict(dist=float("nan"), score=0, bucket=None)
        est, hold = fz["est"], fz["hold"]
        comp = vec(hold, est)                       # 实际进入 hold 的位移
        ac = vec(est, fz["delta"])                  # 飞机@冻结 = est − delta
        aim = fz["aim"] or (float("nan"), float("nan"))
        lag = vec(aim, ac)
        b = vec(dr["ned"], hold)
        travel = vec(hold, ac)
        tn = norm(travel)
        b_along = (b[0] * travel[0] + b[1] * travel[1]) / tn if tn > 1e-9 else float("nan")
        bk = jd["bucket"]
        est_err = vec(est, bk) if bk else (float("nan"), float("nan"))
        # 补偿指令 (Y=横向, XF=纵向, 机体系) 旋转到 NED: 与代码 hold = est + comp 同一公式
        hy, hx = math.radians(fz["hdg"]), 0.0
        cY, cXF = fz["comp_cmd"]
        comp_ned = (cY * math.sin(hy) - cXF * math.cos(hy), -cY * math.cos(hy) - cXF * math.sin(hy))
        # 可归因残差: 扣掉**补偿指令**(实机上它才抵消机械偏置), 剩下的都是飞机自身/测量误差
        resid = vec(vec(dr["ned"], bk), comp_ned) if bk else (float("nan"), float("nan"))
        # 对照: 相对"实际保持点"的精度 (= est 误差 + 飞机没停准 b)
        resid_hold = vec(vec(dr["ned"], bk), comp) if bk else (float("nan"), float("nan"))
        flag = "  ⚠ 保持点≠桶估计+补偿" if fz["comp_log"] and norm(vec(comp, fz["comp_log"])) > 1e-3 else ""
        print("---- 第 %d 投 (PWM=%dus, 补偿 B%d: Y=%+.3f XF=%+.3f)%s ----"
              % (i + 1, dr["pwm"], fz["bucket_idx"], fz["comp_cmd"][0], fz["comp_cmd"][1], flag))
        print("  桶估计 est        = (%+.3f, %+.3f)      飞机@冻结 = (%+.3f, %+.3f)"
              % (est[0], est[1], ac[0], ac[1]))
        print("  冻结保持点 hold   = (%+.3f, %+.3f)      hold−est = (%+.3f,%+.3f) |%.3fm| (两瓶补偿 |%.3fm|)"
              % (hold[0], hold[1], comp[0], comp[1], norm(comp), norm(fz["comp_cmd"])))
        print("  瞄准点 aim        = (%+.3f, %+.3f)      lag = aim−飞机 = (%+.3f,%+.3f) |%.3fm|"
              % (aim[0], aim[1], lag[0], lag[1], norm(lag)))
        if bk:
            print("  桶(裁判真值)      = (%+.3f, %+.3f)      est 误差 = (%+.3f,%+.3f) |%.3fm|"
                  % (bk[0], bk[1], est_err[0], est_err[1], norm(est_err)))
        print("  释放点            = (%+.3f, %+.3f)      b = 释放点−hold = (%+.3f,%+.3f) |%.3fm| (沿飞行方向 %+.3fm)"
              % (dr["ned"][0], dr["ned"][1], b[0], b[1], norm(b), b_along))
        print("  释放参数: 到位err=%.1fcm pred=%.1fcm 停留=%.1fs vxy=%.3fm/s" % (dr["err_cm"], dr["pred_cm"], dr["dwell"], dr["vxy"]))
        print("  裁判距离 = %.4fm (%d 分)" % (jd["dist"], jd["score"]))
        print("  补偿指令@NED(hdg=%.1f) = (%+.3f,%+.3f) |%.3fm|   实际 hold−est = (%+.3f,%+.3f)"
              % (fz["hdg"], comp_ned[0], comp_ned[1], norm(comp_ned), comp[0], comp[1]))
        print("  可归因残差 |释放点−桶−补偿指令| = %.4fm   (对照: 相对实际保持点 |释放点−hold| = %.4fm)"
              % (norm(resid), norm(vec(dr["ned"], hold))))
        if bk:
            print("  矢量闭合: b + est误差 + comp = (%+.3f,%+.3f)   vs 释放点−桶 = (%+.3f,%+.3f)"
                  % (b[0] + est_err[0] + comp[0], b[1] + est_err[1] + comp[1],
                     dr["ned"][0] - bk[0], dr["ned"][1] - bk[1]))
        print()
        if bk:
            tot_res += norm(resid)
            nres += 1
        tot_score += jd["score"]
    print("合计: 裁判 %d/%d 分, 平均可归因残差 %.4fm" % (tot_score, 10 * max(1, len(judges)), (tot_res / nres) if nres else float("nan")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
