#!/usr/bin/env python3
"""项目 i 落点标定器 (2026-10-07 实飞用) — v2 (2026-10-06 夜, SITL 复核后修正)

输入: 一份或多份 i 任务日志 (Jetson 上 ~/fly_logs/i_mission_*.log) + 现场量的落点偏移。
输出: 每投的分解量 (飞机到位残差 b / 冻结时刻滞后 lag / 总泄漏 E) + 建议的新补偿值。

⚠ 2026-10-07 方案 A 起: `保持点 = 桶估计 + 补偿` ⇒ **lag ≡ 0,  E ≡ b** (冻结滞后项已被删掉);
  逐投应看到 `|lag| ≈ 0`, 若不为 0 说明板上还是旧代码.

误差链 (SITL 两投已逐项验证: r'≈0, 残差只剩弹道漂移):
    落点 - 桶心 = (A - 桶估计) + r' + 弹道漂移
      A     = 释放瞬间飞机位置 (日志 `>>> DROP` 行 NED - home)
      桶估计 = 冻结那一帧的相机估计 (日志 冻结行 `桶估计=(x,y)`)
    而 A - 桶估计 = 补偿 + E,  E ≡ lag + b
      lag = (冻结点 - 桶估计) - 补偿 = 冻结那一刻"飞机还没追上原瞄准点"的滞后
      b   = A - 冻结点               = 释放瞬间到位残差 (日志 到位err 的矢量形式)
    => 落点 - 桶心 = 补偿 + E + r' + 漂移

**补偿更新公式 (要用的就是这条)**:
    新补偿 = 旧补偿 - 实测偏移
  推导: 命中要求 补偿_new + E_next = -r'; 取 E_next ≈ E_now ⇒ E 自动消掉。
  ⚠ v1 曾把"固定机械偏置 r'"直接当成新补偿, 那会漏掉 E(冻结滞后) —— 本站已修正。

现场测量约定 (**必须两个带符号分量**, 不要"左下 11cm"这种描述):
    沿航向 (fwd):  落点相对桶心, 朝"远离起飞点"为正, 朝起飞点为负   [cm]
    横向 (left):   落点相对桶心, 朝机头左手侧为正                   [cm]
用法:
    python3 scripts/calibrate_i_offset_20261007.py 10.7/i_mission_*.log
    python3 scripts/calibrate_i_offset_20261007.py 10.7/i_mission_*.log --meas 1:-8:0 --meas 2:-16:16
    # 量了桶心(相对起飞点)时, 可把 est 误差单独算出来 (2026-10-07 新增):
    python3 scripts/calibrate_i_offset_20261007.py 10.7/i_mission_*.log \
            --meas 1:-8:0 --meas 2:-16:16 --bucket 1:3300:150 --bucket 2:3400:-260
    python3 scripts/calibrate_i_offset_20261007.py 10.6/i_mission_*.log \
            --meas 170652:1:-8:8 --meas 170652:2:-16:16 \
            --meas 171748:1:0:15 --meas 171748:2:-8.5:0     # 带文件名关键字区分航班
"""

import argparse
import glob
import math
import re

G = 9.81

RE_HOME = re.compile(r"Home locked: NED\(([-\d.]+),([-\d.]+),([-\d.]+)\) heading0=([\d.]+)deg")
RE_OFF = re.compile(
    r"offset_横_B1=([-+\d.]+)m B2=([-+\d.]+)m offset_前_B1=([-+\d.]+)m B2=([-+\d.]+)m")
RE_FREEZE = re.compile(
    r"一次性修正 delta=\(([-+\d.]+),([-+\d.]+)\)m \(桶估计=\(([-+\d.]+),\s*([-+\d.]+)\)\)"
    r" → 冻结(?:保持)?点 \(([-+\d.]+),([-+\d.]+)\)"
    r"(?:\s*\[纯几何:[^\]]*\])?"        # 2026-10-07 方案 A 新增字段; 旧日志没有 ⇒ 必须可选
    r" 补偿B(\d)=\(([-+\d.]+),([-+\d.]+)\)")
RE_DROP = re.compile(r">>> DROP #(\d+)! PWM=(\d+)us NED\(([-\d.]+),([-\d.]+),([-\d.]+)\)")
RE_HOLD = re.compile(r"hold=\(([-+\d.]+),([-+\d.]+)\)")
RE_ERR = re.compile(r"到位err=([\d.]+)cm")
RE_PRED = re.compile(r"pred=([\d.]+)cm")
RE_VXY = re.compile(r"vxy=([\d.]+)m/s")
RE_NSAMP = re.compile(r"\[(平静帧中值\(校准(\d+)帧\)|兜底[^\]]*)\]")


def parse_log(path):
    home = hdg = comp = None
    freezes = []          # (行号, est(x,y), hold(x,y), comp(Y,XF))
    drops = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for ln, line in enumerate(fh, 1):
            if home is None:
                m = RE_HOME.search(line)
                if m:
                    home = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                    hdg = math.radians(float(m.group(4)))
                else:
                    m = RE_OFF.search(line)
                    if m:
                        comp = {"B1": (float(m.group(1)), float(m.group(3))),
                                "B2": (float(m.group(2)), float(m.group(4)))}
                    continue
            m = RE_FREEZE.search(line)
            if m:
                freezes.append((ln, (float(m.group(3)), float(m.group(4))),
                                (float(m.group(5)), float(m.group(6))),
                                (float(m.group(8)), float(m.group(9)))))
                continue
            m = RE_DROP.search(line)
            if m:
                mh = RE_HOLD.search(line)
                if not mh:
                    continue
                me, mp, mv, ms = (RE_ERR.search(line), RE_PRED.search(line),
                                  RE_VXY.search(line), RE_NSAMP.search(line))
                drops.append(dict(ln=ln, idx=int(m.group(1)), pwm=int(m.group(2)),
                                  ned=(float(m.group(3)), float(m.group(4)), float(m.group(5))),
                                  hold=(float(mh.group(1)), float(mh.group(2))),
                                  err=float(me.group(1)) if me else None,
                                  pred=float(mp.group(1)) if mp else None,
                                  vxy=float(mv.group(1)) if mv else None,
                                  nsamp=int(ms.group(2)) if (ms and ms.group(2)) else None,
                                  mode=(ms.group(1) if ms else ""),
                                  bottle="B1" if int(m.group(2)) >= 1500 else "B2"))
    if home is None or comp is None:
        raise SystemExit(f"[FAIL] {path}: 缺少 'Home locked:' 或 'offset_横_B1=' 行")
    return home, hdg, comp, freezes, drops


def pick_meas(meas, path, idx):
    if (None, idx) in meas:
        return meas[(None, idx)]
    for (key, i), v in meas.items():
        if i == idx and key and key in path:
            return v
    return None


def analyse(path, meas, buckets=None):
    home, hdg, comp, freezes, drops = parse_log(path)
    cf, sf = math.cos(hdg), math.sin(hdg)
    print(f"\n===== {path}")
    print(f"  home NED=({home[0]:.2f},{home[1]:.2f},{home[2]:.2f})  heading={math.degrees(hdg):.1f}°  "
          f"航班补偿 B1(Y={comp['B1'][0]:+.3f},XF={comp['B1'][1]:+.3f}) | "
          f"B2(Y={comp['B2'][0]:+.3f},XF={comp['B2'][1]:+.3f})")
    print(f"  {'投':<3}{'瓶':<4}{'到位err':>8}{'|b|':>7}{'|lag|':>7}{'|E|':>7}"
          f"{'|A-桶|':>8}{'A-桶_前':>9}{'A-桶_左':>9}{'实测_前':>9}{'实测_左':>9}")
    res = {}
    for d in drops:
        bot = d["bottle"]
        Y, XF = comp[bot]
        fr = [f for f in freezes if f[0] < d["ln"]]
        if not fr:
            continue
        _, est, _fh, fc = fr[-1]
        if fc:                                              # 冻结行自带的补偿 (与 _drop_count 绑定)
            if (fc[0], fc[1]) != (Y, XF):
                print(f"      ⚠ 冻结行补偿 ({fc[0]:+.3f},{fc[1]:+.3f}) 与航班行 ({Y:+.3f},{XF:+.3f}) 不同, 以冻结行为准")
            Y, XF = fc
        cv = (Y * sf - XF * cf, -Y * cf - XF * sf)          # 补偿矢量 → NED
        A = (d["ned"][0] - home[0], d["ned"][1] - home[1])
        b_v = (A[0] - d["hold"][0], A[1] - d["hold"][1])
        lag_v = ((d["hold"][0] - est[0]) - cv[0], (d["hold"][1] - est[1]) - cv[1])
        E_v = (A[0] - est[0] - cv[0], A[1] - est[1] - cv[1])
        AB_v = (A[0] - est[0], A[1] - est[1])

        def rot(v):
            return (v[0] * cf + v[1] * sf, v[0] * sf - v[1] * cf)   # → (fwd, left)
        b, lag, E, AB = rot(b_v), rot(lag_v), rot(E_v), rot(AB_v)
        # 2026-10-07: 现场量了桶心 (相对起飞点) ⇒ 可把"相机低空测量误差 est误差"单独量出来
        bk = (buckets or {}).get(d["idx"])
        est_err_txt = ""
        if bk:
            bf, bl = bk[0] / 100.0, bk[1] / 100.0
            bk_ned = (bf * cf + bl * sf, bf * sf - bl * cf)     # 桶心 → 日志 NED 相对坐标
            ee_f, ee_l = rot((est[0] - bk_ned[0], est[1] - bk_ned[1]))
            est_err_txt = f"  est误差_前={ee_f * 100:+.1f}cm est误差_左={ee_l * 100:+.1f}cm"
        H = home[2] - d["ned"][2]
        drift = (d["vxy"] or 0.0) * math.sqrt(2 * H / G)
        m = pick_meas(meas, path, d["idx"])
        mf, ml = (m[0] / 100.0, m[1] / 100.0) if m else (None, None)
        print(f"  #{d['idx']:<2}{bot:<4}{(d['err'] or 0):>7.1f}c"
              f"{math.hypot(*b) * 100:>6.1f}c{math.hypot(*lag) * 100:>6.1f}c"
              f"{math.hypot(*E) * 100:>6.1f}c{math.hypot(*AB) * 100:>7.1f}c"
              f"{AB[0] * 100:>+8.1f}c{AB[1] * 100:>+8.1f}c"
              + (f"{mf * 100:>+8.1f}c{ml * 100:>+8.1f}c" if m else f"{'-':>9}{'-':>9}")
              + f"  [H={H:.2f}m 漂移≤{drift * 100:.1f}cm 样本={d['nsamp']}]" + est_err_txt)
        if m is not None:
            y_new = Y - ml                        # 旧补偿(左) - 实测(左)
            xf_new = XF + mf                      # comp_fwd = -XF ⇒ comp_fwd_new = -XF - 实测_前 ⇒ XF_new = XF + 实测_前
            res.setdefault(bot, []).append(dict(
                y=y_new, xf=xf_new, rf=mf - AB[0], rl=ml - AB[1], drift=drift,
                lag=math.hypot(*lag), b=math.hypot(*b)))
    if res:
        print("\n  ---- 建议新补偿 (新值 = 旧值 - 实测偏移; E 自动消掉) ----")
        for bot in ("B1", "B2"):
            if bot not in res:
                continue
            v = res[bot]
            n = len(v)
            print(f"  {bot} (n={n}): Y={sum(x['y'] for x in v) / n:+.3f}  "
                  f"XF={sum(x['xf'] for x in v) / n:+.3f}"
                  f"   逐投 Y={['%+.3f' % x['y'] for x in v]}"
                  f" XF={['%+.3f' % x['xf'] for x in v]}")
            if max(x["drift"] for x in v) > 0.02 or max(x["lag"] for x in v) > 0.05:
                print(f"       ⚠ 本瓶最大弹道漂移 {max(x['drift'] for x in v) * 100:.1f}cm(方向未知)"
                      f" / 冻结滞后 {max(x['lag'] for x in v) * 100:.1f}cm"
                      f" (滞后只进 E, 不进新补偿公式)")
            print(f"       诊断 r'(固定机械偏置 = 实测 - (A-桶估计), 应逐投一致): "
                  f"r'_前={['%+.1f' % (x['rf'] * 100) for x in v]}cm "
                  f"r'_左={['%+.1f' % (x['rl'] * 100) for x in v]}cm")
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--meas", action="append", default=[],
                    help="现场落点: [文件关键字:]投号:沿航向cm:左向cm")
    ap.add_argument("--bucket", action="append", default=[],
                    help="现场桶心相对起飞点: 投号:沿航向cm:左向cm (量了它才能单独算出 est 误差)")
    a = ap.parse_args()

    meas = {}
    for s in a.meas:
        p = s.split(":")
        if len(p) == 4:
            key, i, f, l = p
        elif len(p) == 3:
            key, (i, f, l) = None, p
        else:
            raise SystemExit(f"--meas 格式错: {s}")
        meas[(key, int(i))] = (float(f), float(l))

    buckets = {}
    for s2 in a.bucket:
        _i, _f, _l = s2.split(":")
        buckets[int(_i)] = (float(_f), float(_l))

    files = []
    for p in a.logs:
        files.extend(sorted(glob.glob(p)) or [p])
    if not files:
        raise SystemExit("没有日志文件")
    allres = {}
    for p in files:
        for bot, vals in analyse(p, meas, buckets).items():
            allres.setdefault(bot, []).extend(vals)
    if len(files) > 1 and allres:
        print("\n===== 多趟合并 (最终建议) =====")
        for bot in ("B1", "B2"):
            if bot not in allres:
                continue
            v = allres[bot]
            n = len(v)
            print(f"  {bot} (n={n}): Y={sum(x['y'] for x in v) / n:+.3f}  "
                  f"XF={sum(x['xf'] for x in v) / n:+.3f}")
    print("\n提示: 单投 E/lag 是 5~40cm 的随机项 ⇒ 两飞四投取平均; 冻结滞后大的那投权重低")


if __name__ == "__main__":
    main()
