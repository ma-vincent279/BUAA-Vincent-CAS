#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extract_fc_profile_20261014.py — 从实飞 ULog (.ulg) 重建**飞控参数画像**。

为什么需要它:
  真飞控不在手上时, ULog 的 `initial_parameters` 里带着**开机时的完整参数表**
  (实测 1160 项) ⇒ 可以直接从飞行日志重建它的配置, 用于:
    ① 新板/备用板按真机配置排练 (diff 出差异清单)
    ② 真飞控回来后**第一时间核对**"和上次飞的时候比, 参数被动过没有"
    ③ 追查历史架次的配置漂移

用法:
  python3 scripts/extract_fc_profile_20261014.py list                 # 列出所有 ulg + 机型/固件
  python3 scripts/extract_fc_profile_20261014.py profile [ulg...]     # 生成画像 (默认全部, 取最新一份的值)
  python3 scripts/extract_fc_profile_20261014.py profile -o out.txt <ulg>

  # 生成物可直接喂给调参工具对比:
  python3 scripts/fc_param.py diff backups/fc_param/params_profile_real_fc_*.txt

设计要点:
  · ULog 里跨架次必然变化的是 `COM_FLIGHT_UUID` / `LND_FLIGHT_T_LO` (架次计数) 与
    `CAL_*` / `SENS_BOARD_*` (重新标定) —— 这些**不算配置漂移**, 默认忽略。
  · 浮点用 %.9g 输出, float32 可无损回转, 与 `fc_param.py diff` 的比较容差一致。
  · MAVLink 的 PARAM_REQUEST_LIST 只回 "used" 参数(实测 827/1861), 画像有 1160 项
    ⇒ 对比时"缺项"多为未使用参数, `fc_param.py diff` 默认只提示不报错。
"""

import argparse
import glob
import os
import sys

try:
    from pyulog import ULog
except ImportError:
    sys.exit("缺少 pyulog: pip3 install --user pyulog")

DEFAULT_GLOB = os.path.expanduser("~/SITL/**/*.ulg")

# 跨架次必然变化、不算配置漂移的项
VOLATILE = {"COM_FLIGHT_UUID", "LND_FLIGHT_T_LO", "LND_FLIGHT_T_HI",
            "COM_FLIGHT_UUID_HI", "COM_FLIGHT_UUID_LO"}


def is_calibration(name):
    return name.startswith("CAL_") or name.startswith("SENS_BOARD_")


def fmt(name, val):
    if isinstance(val, bool):
        return "%d" % int(val)
    if isinstance(val, int):
        return "%d" % val
    return "%.9g" % float(val)


def collect(files, verbose=True):
    """-> [(path, info_dict, params_dict)]"""
    out = []
    for p in files:
        try:
            ul = ULog(p)
            out.append((p, ul.msg_info_dict, ul.initial_parameters))
        except Exception as e:
            print("  ✗ 读不了 %s: %s" % (os.path.basename(p), e))
    if verbose:
        print("  ✓ 读入 %d 份日志" % len(out))
    return out


def cmd_list(args):
    files = sorted(glob.glob(args.pattern, recursive=True), key=os.path.getmtime)
    if not files:
        print("没找到 ulg (%s)" % args.pattern)
        return 1
    print("%-40s %-8s %-12s %-12s %s"
          % ("日志", "参数数", "固件", "硬件", "机型/电芯"))
    for p, info, ip in collect(files, verbose=False):
        sw = str(info.get("ver_sw", "?"))[:8]
        print("%-40s %-8d %-12s %-12s autostart=%s cells=%s"
              % (os.path.basename(p)[:40], len(ip), sw,
                 info.get("ver_hw", "?"),
                 ip.get("SYS_AUTOSTART"), ip.get("BAT1_N_CELLS")))
    return 0


def cmd_profile(args):
    files = sorted(glob.glob(args.pattern, recursive=True), key=os.path.getmtime)
    if args.ulgs:
        files = [f for f in files
                 if os.path.basename(f) in {os.path.basename(x) for x in args.ulgs}]
    if not files:
        print("没找到 ulg")
        return 1
    rows = collect(files)
    if not rows:
        return 1

    # ---- 跨日志一致性 (忽略必然变化的项与重新标定项)
    common = set(rows[0][2])
    for _, _, ip in rows[1:]:
        common &= set(ip)
    drift = {}
    for k in sorted(common):
        if k in VOLATILE or is_calibration(k):
            continue
        vals = {round(float(ip[k]), 6) if isinstance(ip[k], float) else ip[k]
                for _, _, ip in rows}
        if len(vals) > 1:
            drift[k] = sorted(vals, key=str)
    if drift:
        print("\n  ⚠ 跨 %d 份日志有 %d 项**非标定配置**不一致 (画像取最新一份的值):"
              % (len(rows), len(drift)))
        for k, v in drift.items():
            print("      %-24s %s" % (k, v))
    else:
        print("  ✓ 跨 %d 份日志的非标定配置完全一致 (只有架次计数与重新标定值在变)"
              % len(rows))

    # ---- 取值用最新一份
    newest_path, newest_info, newest = rows[-1]
    import time as _t
    stamp = _t.strftime("%Y%m%d", _t.localtime(os.path.getmtime(newest_path)))
    out_path = args.output or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "backups", "fc_param",
        "params_profile_real_fc_%s.txt" % stamp)
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# 真飞控参数画像 (由实飞 ULog 重建, 非在线读取)\n")
        f.write("# 时间: %s\n" % __import__("time").strftime("%Y-%m-%d %H:%M:%S"))
        f.write("# 来源日志: %s\n" % newest_path)
        f.write("# 依据日志: %d 份 (%s ... %s)\n"
                % (len(rows), os.path.basename(rows[0][0]), os.path.basename(rows[-1][0])))
        f.write("# 固件: ver_sw=%s release=%s\n"
                % (newest_info.get("ver_sw"), newest_info.get("ver_sw_release")))
        f.write("# 硬件: %s / OS %s\n"
                % (newest_info.get("ver_hw"), newest_info.get("sys_os_ver")))
        f.write("# 共 %d 项参数\n" % len(newest))
        f.write("# 注意: 标定类 (CAL_*/SENS_BOARD_*) 是**该机最后一次标定值**, 换机/重标后必然不同\n")
        f.write("# 对比: python3 scripts/fc_param.py diff %s\n" % os.path.basename(out_path))
        for k in sorted(newest):
            f.write("%s %s\n" % (k, fmt(k, newest[k])))
    print("  ✓ 画像已写出: %s (%d 项)" % (out_path, len(newest)))

    # ---- 顺手把"看得懂"的摘要打出来
    hot = ["SYS_AUTOSTART", "SYS_AUTOCONFIG", "MAV_TYPE", "MAV_SYS_ID",
           "EKF2_HGT_REF", "EKF2_MAG_TYPE", "EKF2_GPS_CTRL", "EKF2_GPS_P_NOISE",
           "EKF2_GPS_P_GATE", "EKF2_GPS_V_GATE", "EKF2_GPS_CHECK", "EKF2_GPS_DELAY",
           "COM_RC_IN_MODE", "COM_ARM_MAG_STR", "BAT1_N_CELLS", "BAT1_V_DIV",
           "BAT1_A_PER_V", "BAT1_CAPACITY", "SENS_BARO_QNH", "SDLOG_MODE"]
    print("\n  --- 真飞控关键参数 ---")
    for k in hot:
        if k in newest:
            print("      %-20s = %s" % (k, fmt(k, newest[k])))
    return 0


def main():
    ap = argparse.ArgumentParser(description="从实飞 ULog 重建飞控参数画像")
    ap.add_argument("--pattern", default=DEFAULT_GLOB, help="ulg 搜索模式")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="列出所有 ulg + 机型/固件")
    p = sub.add_parser("profile", help="生成参数画像")
    p.add_argument("ulgs", nargs="*", help="指定日志 (默认全部, 取最新一份的值)")
    p.add_argument("-o", "--output", help="输出文件")
    args = ap.parse_args()
    return {"list": cmd_list, "profile": cmd_profile}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
