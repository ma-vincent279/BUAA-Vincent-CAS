#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fc_param.py — 飞控 (PX4) 直连调参工具, 走 USB/MAVLink, 不需要 QGC。

用法:
  python3 scripts/fc_param.py info                       # 板子信息 + 连接自检
  python3 scripts/fc_param.py list [--filter EKF2_]      # 列出参数
  python3 scripts/fc_param.py get  EKF2_HGT_REF SENS_BARO_QNH
  python3 scripts/fc_param.py set  SENS_BARO_QNH 1005.0  # 改 (RAM)
  python3 scripts/fc_param.py set  SENS_BARO_QNH 1005.0 --save   # 改 + 落盘 flash
  python3 scripts/fc_param.py save                       # 把当前参数存到 flash
  python3 scripts/fc_param.py dump [file]                # 备份全部参数到文本文件
  python3 scripts/fc_param.py diff <file>                # 对比飞控现值与备份
  python3 scripts/fc_param.py restore <file> [--yes]     # 按备份还原 (默认只演练)
  python3 scripts/fc_param.py selftest                   # 全链路往返自检 (只改同值)
  python3 scripts/fc_param.py nsh "param show EKF2_*"    # 一次性 NSH 命令 (安全)
  python3 scripts/fc_param.py shell                      # 交互式 NSH shell (⚠ 见下)

⚠ 关于 NSH shell 的坑 (实测踩过, 会让飞控"假死"):
  PX4 只在收到 **flags=0 (不要求响应) 的 SERIAL_CONTROL** 时才关闭 shell
  (src/modules/mavlink/mavlink_receiver.cpp:2013)。所以 shell 会话被 kill / timeout
  掉就会在 FC 侧留下**孤儿会话**, 症状 = USB 还在、但 0 字节、连写都阻塞。
  此时**软件无法自救**(写方向也堵住了), 只能**拔插 USB / 重新上电**复位。
  ⇒ 脚本批量调参一律用 `nsh` (它 try/finally + atexit + 信号处理, 保证发 close);
  ⇒ `shell` 只用于人工交互, 退出用 Ctrl-D, **不要 kill**, 更不要套 timeout。

设计要点 (踩过的坑):
  · PX4 的 PARAM_VALUE.param_value 是 float 字段, INT32 参数必须做
    "float 位重解释成 int32" —— 否则 EKF2_HGT_REF=1 会读成 1.4e-45。
  · pymavlink 2.4.49 的 add_message 对带 instance 字段的消息会 TypeError, 已打补丁。
  · 解锁 (armed) 状态下拒绝写参数与落盘 —— 飞行中改参数是事故来源。
  · 写参数只进 RAM, 必须 save (MAV_CMD_PREFLIGHT_STORAGE param1=1) 才进 flash。
"""

import argparse
import atexit
import os
import re
import signal
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("缺少 pymavlink: pip3 install --user pymavlink pyserial")

# ---------------------------------------------------------------- MAVLink 常量
PARAM_TYPE_NAMES = {
    1: "UINT8", 2: "INT8", 3: "UINT16", 4: "INT16",
    5: "UINT32", 6: "INT32", 7: "UINT64", 8: "INT64",
    9: "REAL32", 10: "REAL64",
}
INT_TYPES = {1, 2, 3, 4, 5, 6}

MAV_COMP_AUTOPILOT = 1
SYSID = 255
COMPID = 190


# ------------------------------------------------------------------- 小工具
def find_port(explicit=None):
    """优先用 by-id 里带 PX4/CUAV 的口, 否则退回 ttyACM0。"""
    if explicit:
        return explicit
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        cands = sorted(
            os.path.join(by_id, n) for n in os.listdir(by_id)
            if any(k in n.upper() for k in ("PX4", "CUAV", "FMU", "PIXHAWK"))
        )
        if cands:
            return cands[0]
    for p in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"):
        if os.path.exists(p):
            return p
    return None


def decode_param(msg):
    """PARAM_VALUE -> (python 数值, 类型名)。INT32 需要位重解释。"""
    raw = msg.param_value
    ptype = msg.param_type
    if ptype in INT_TYPES:
        try:
            val = struct.unpack("<i", struct.pack("<f", raw))[0]
        except (struct.error, OverflowError):
            val = int(raw)
    else:
        val = raw
    return val, PARAM_TYPE_NAMES.get(ptype, "TYPE%d" % ptype)


def encode_param(value, ptype):
    """把 python 数值编码成 PARAM_SET 需要的 float 字段。"""
    if ptype in INT_TYPES:
        return struct.unpack("<f", struct.pack("<i", int(round(float(value)))))[0]
    return float(value)


def fmt(name, val, tname):
    if tname in ("REAL32", "REAL64"):
        return "%-16s = %-14s  [%s]" % (name, ("%g" % val), tname)
    return "%-16s = %-14d  [%s]" % (name, int(val), tname)


def patch_pymavlink():
    """pymavlink 2.4.49: 带 instance 字段的消息第二次到达时 add_message 抛 TypeError。"""
    orig = mavutil.add_message

    def safe_add(messages, mtype, msg):
        try:
            orig(messages, mtype, msg)
        except TypeError:
            messages[mtype] = msg

    mavutil.add_message = safe_add


# ------------------------------------------------------------------- 连接
class FC:
    def __init__(self, port, baud=57600, verbose=True):
        patch_pymavlink()
        self.port = port
        if verbose:
            print("[连接] %s" % port)
        self.m = mavutil.mavlink_connection(
            port, baud=baud, source_system=SYSID, source_component=COMPID,
            autoreconnect=False,
        )
        hb = self.m.wait_heartbeat(timeout=20)
        if hb is None:
            raise RuntimeError("20s 内没收到 HEARTBEAT —— 检查线/口/是否被 QGC 占用")
        self.sysid = self.m.target_system
        self.compid = MAV_COMP_AUTOPILOT
        self.type = hb.type
        self.autopilot = hb.autopilot
        self.armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.params = {}          # name -> (value, type_name)
        if verbose:
            print("[连接] ✓ sys=%d type=%d autopilot=%d(%s) armed=%s"
                  % (self.sysid, self.type, self.autopilot,
                     "PX4" if self.autopilot == 12 else "?", self.armed))

    # ---------------------------------------------------------- 读
    def read(self, names, timeout=2.0):
        """按名读参数, 返回 {name: (value, tname)}; 不存在的名字不返回。"""
        want = list(names)
        for n in want:
            self.m.mav.param_request_read_send(
                self.sysid, self.compid, n.encode(), -1)
        out = {}
        deadline = time.time() + max(timeout, 0.4 * len(want))
        while want and time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", "ignore").rstrip("\x00")
            val, tname = decode_param(msg)
            self.params[pid] = (val, tname)
            out[pid] = (val, tname)
            if pid in want:
                want.remove(pid)
        return out

    def read_one(self, name):
        got = self.read([name])
        return got.get(name)

    def list_all(self, filt=None, timeout=40.0):
        """拉全表 (827 个约 10~30s)。filt 为子串, 大小写不敏感。"""
        self.m.mav.param_request_list_send(self.sysid, self.compid)
        total, got = None, {}
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                if total is not None and len(got) >= total:
                    break
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", "ignore").rstrip("\x00")
            val, tname = decode_param(msg)
            got[pid] = (val, tname)
            self.params[pid] = (val, tname)
            total = msg.param_count
            if total and len(got) >= total:
                break
        if filt:
            f = filt.upper()
            got = {k: v for k, v in got.items() if f in k.upper()}
        return got, total

    # ---------------------------------------------------------- 写
    def write_one(self, name, value, verify=True, timeout=4.0):
        """写单个参数到 RAM (需再 save 才进 flash)。返回 (ok, old, new, note)"""
        if self.armed:
            return False, None, None, "飞控已解锁, 拒绝写参数"
        cur = self.read_one(name)
        if cur is None:
            return False, None, None, "参数不存在 (读不到 PARAM_VALUE)"
        old, tname = cur
        ptype = _ptype_of(tname)
        self.m.mav.param_set_send(
            self.sysid, self.compid, name.encode(),
            encode_param(value, ptype), ptype,
        )
        # PX4 回一个 PARAM_VALUE 作为确认
        new, acked = None, False
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", "ignore").rstrip("\x00")
            if pid != name:
                continue
            new, _ = decode_param(msg)
            acked = True
            break
        if not acked:
            return False, old, None, "没有收到 PARAM_VALUE 确认 (写入可能仍生效, 请复查)"
        if verify:
            expect = int(round(float(value))) if tname in ("INT32",) else float(value)
            gotv = int(new) if tname in ("INT32",) else float(new)
            if isinstance(expect, int):
                ok = gotv == expect
            else:
                ok = abs(gotv - expect) <= max(1e-6, abs(expect) * 1e-6)
            if not ok:
                return False, old, new, "回读值 %s 与写入值 %s 不一致" % (new, value)
        return True, old, new, "ok"

    def save(self, timeout=6.0):
        """MAV_CMD_PREFLIGHT_STORAGE param1=1 -> param_save_default()。"""
        if self.armed:
            return False, "飞控已解锁, 拒绝写 flash"
        self.m.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0,
            1, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if ack is None:
                continue
            if ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
                res = ack.result
                names = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                         3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
                return res == 0, names.get(res, str(res))
        return False, "没收到 COMMAND_ACK"

    def version(self):
        self.m.mav.command_long_send(
            self.sysid, self.compid,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0)
        v = self.m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=6)
        if v is None:
            return None
        sw = v.flight_sw_version
        return {
            "major": (sw >> 24) & 0xFF, "minor": (sw >> 16) & 0xFF,
            "patch": (sw >> 8) & 0xFF, "type": sw & 0xFF,
            "git_hash": bytes(v.flight_custom_version).hex(),
            "capabilities": v.capabilities,
        }

    # ------------------------------------------------- NSH shell (SERIAL_CONTROL)
    def nsh_write(self, text):
        """把一个字符串按 70 字节切块发到 FC 的 NSH shell。"""
        buf = [ord(c) for c in text]
        i = 0
        while i < len(buf):
            chunk = buf[i:i + 70]
            n = len(chunk)
            self.m.mav.serial_control_send(
                device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                flags=(mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                       | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND),
                timeout=0, baudrate=0, count=n, data=chunk + [0] * (70 - n))
            i += n
            time.sleep(0.03)          # 节流: 别把 FC 侧 shell 管道灌满

    def nsh_read(self, quiet=0.7, maxt=8.0):
        """收 shell 回显, 直到静默 quiet 秒或超时。"""
        out = bytearray()
        t0 = last = time.time()
        while time.time() - t0 < maxt:
            m = self.m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
            if m is None:
                if out and time.time() - last > quiet:
                    break
                continue
            if m.count:
                out += bytes(m.data[:m.count])
                last = time.time()
        return out.decode("utf-8", "replace")

    def nsh_close(self):
        """关键: flags=0 (不要求响应) => PX4 关闭 shell (mavlink_receiver.cpp:2013)。
        不发这条, FC 侧会留下孤儿 shell 会话, 严重时把 mavlink 线程卡死。"""
        self.m.mav.serial_control_send(
            device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
            flags=0, timeout=0, baudrate=0, count=0, data=[0] * 70)


def _ptype_of(tname):
    for k, v in PARAM_TYPE_NAMES.items():
        if v == tname:
            return k
    return mavutil.mavlink.MAV_PARAM_TYPE_REAL32


# ------------------------------------------------------------------- 命令
def cmd_info(fc, args):
    v = fc.version()
    print("\n=========== 飞控信息 ===========")
    print("端口        : %s" % fc.port)
    print("system/comp : %d / %d" % (fc.sysid, fc.compid))
    print("自驾仪      : %s" % ("PX4" if fc.autopilot == 12 else fc.autopilot))
    print("解锁状态    : %s" % ("ARMED ⚠" if fc.armed else "DISARMED ✓ (可安全调参)"))
    if v:
        print("固件版本    : %d.%d.%d (type=%d)" % (v["major"], v["minor"], v["patch"], v["type"]))
        print("git-hash    : %s" % v["git_hash"])
    got, total = fc.list_all(filt=None, timeout=40)
    print("参数表      : 读到 %d / %d 个" % (len(got), total or 0))
    key = ["SYS_AUTOSTART", "MAV_SYS_ID", "COM_ARM_MAG_STR", "EKF2_HGT_REF",
           "SENS_BARO_QNH", "BAT1_N_CELLS", "MAV_TYPE", "COM_FLTMODE1"]
    have = fc.read([k for k in key if k in got])
    print("\n--- 首飞清单相关参数 ---")
    for k in key:
        if k in have:
            print("  " + fmt(k, *have[k]))
        else:
            print("  %-16s = <读不到>" % k)
    return 0


def cmd_list(fc, args):
    got, total = fc.list_all(filt=args.filter, timeout=args.timeout)
    for k in sorted(got):
        print("  " + fmt(k, *got[k]))
    print("\n共 %d 个%s (全表 %s)"
          % (len(got), (" (filter=%s)" % args.filter) if args.filter else "", total))
    return 0


def cmd_get(fc, args):
    got = fc.read(args.names)
    rc = 0
    for n in args.names:
        if n in got:
            print("  " + fmt(n, *got[n]))
        else:
            print("  %-16s = <读不到 (名字错? 或该参数不在本固件)>" % n)
            rc = 1
    return rc


def cmd_set(fc, args):
    ok, old, new, note = fc.write_one(args.name, args.value, verify=not args.no_verify)
    if old is not None:
        print("  %s: %s -> %s" % (args.name, old, new if new is not None else "?"))
    if not ok:
        print("  ✗ 写入失败: %s" % note)
        return 1
    print("  ✓ 写入成功 (RAM)")
    if args.save:
        sok, sres = fc.save()
        print("  %s 落盘 flash: %s" % ("✓" if sok else "✗", sres))
        if not sok:
            return 1
    else:
        print("  ⓘ 仅改了 RAM, 断电会丢 —— 要持久化请加 --save")
    return 0


def cmd_save(fc, args):
    ok, res = fc.save()
    print("  %s 落盘 flash: %s" % ("✓" if ok else "✗", res))
    return 0 if ok else 1


# ------------------------------------------------------------------- 备份/对比
def _valstr(val, tname):
    """写成可回灌的字符串: INT 类给整数, 浮点给 9 位有效数字 (float32 可无损回转)。"""
    if tname in ("REAL32", "REAL64"):
        return "%.9g" % float(val)
    return "%d" % int(val)


def _read_dump(path):
    """读 dump 文件 -> {name: value_str} (忽略 # 注释)。"""
    out = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                out[parts[0]] = parts[1]
    return out


def _same(a_str, b_val, b_type):
    """比较 dump 里的字符串与飞控现值 (浮点容差)。"""
    if b_type in ("REAL32", "REAL64"):
        try:
            return abs(float(a_str) - float(b_val)) <= max(1e-6, abs(float(b_val)) * 1e-6)
        except ValueError:
            return False
    try:
        return int(float(a_str)) == int(b_val)
    except ValueError:
        return False


def cmd_dump(fc, args):
    got, total = fc.list_all(timeout=args.timeout)
    if not got:
        print("✗ 没读到参数")
        return 1
    path = args.file or "params_dump_%s.txt" % time.strftime("%Y%m%d_%H%M%S")
    with open(path, "w", encoding="utf-8") as f:
        f.write("# fc_param.py 参数备份\n")
        f.write("# 时间: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        f.write("# 来源: %s  sys=%d  PX4 %s\n" % (fc.port, fc.sysid, _ver_str(fc)))
        f.write("# 共 %d/%s 个参数\n" % (len(got), total))
        for k in sorted(got):
            f.write("%s %s\n" % (k, _valstr(*got[k])))
    print("  ✓ 已备份 %d 个参数 -> %s" % (len(got), path))
    print("  ⓘ 对比: python3 %s diff %s" % (os.path.basename(__file__), path))
    return 0


def cmd_diff(fc, args):
    want = _read_dump(args.file)
    got, _ = fc.list_all(timeout=args.timeout)
    if not got:
        print("✗ 没读到参数")
        return 1
    changed, missing = [], []
    for name, was in sorted(want.items()):
        if name not in got:
            missing.append(name)
        elif not _same(was, *got[name]):
            changed.append((name, was, _valstr(*got[name]), got[name][1]))
    extra = sorted(set(got) - set(want))
    print("  备份文件: %s (%d 项)" % (args.file, len(want)))
    print("  飞控现值: %d 项" % len(got))
    if not changed and not missing:
        print("\n  ✓ 完全一致, 没有任何改动")
    if changed:
        print("\n  ⚠ 已改动 %d 项  (左 = 文件里的值, 右 = 飞控当前值):" % len(changed))
        for name, was, now, t in changed:
            print("      %-18s 文件=%-14s 飞控=%-14s [%s]" % (name, was, now, t))
    if missing:
        # 说明: MAVLink 的 PARAM_REQUEST_LIST 只回 "used" 参数 (实测 827/1861),
        # 而 ULog 画像有 1160 项 ⇒ 缺项多半是"未使用参数", 不是异常。
        lvl = "⚠" if args.strict else "ⓘ"
        print("\n  %s 画像里有、飞控没上报 %d 项%s"
              % (lvl, len(missing), " (--strict 视为差异)" if args.strict else " (多为未使用参数, 正常)"))
        for n in missing[:10]:
            print("      %s" % n)
        if len(missing) > 10:
            print("      ... 其余 %d 项" % (len(missing) - 10))
    if extra:
        print("\n  ⓘ 飞控有、画像里没有 %d 项 (固件新增或画像未覆盖, 正常)" % len(extra))
    return 1 if (changed or (missing and args.strict)) else 0


def cmd_restore(fc, args):
    """按 dump 文件把不同的项写回飞控。默认只演练, 加 --yes 才真写。"""
    want = _read_dump(args.file)
    got, _ = fc.list_all(timeout=args.timeout)
    todo = [(n, w, _valstr(*got[n]), got[n][1]) for n, w in sorted(want.items())
            if n in got and not _same(w, *got[n])]
    if not todo:
        print("  ✓ 无差异, 不用还原")
        return 0
    print("  需要还原 %d 项:" % len(todo))
    for n, w, now, t in todo:
        print("      %-18s %-12s -> %-12s [%s]" % (n, now, w, t))
    if not args.yes:
        print("\n  ⓘ 这是演练。要真写请加 --yes")
        return 0
    if fc.armed:
        print("\n  ✗ 飞控已解锁, 拒绝还原")
        return 1
    bad = 0
    for n, w, now, t in todo:
        ok, old, new, note = fc.write_one(n, w)
        print("      %s %s -> %s %s" % ("✓" if ok else "✗", n, new, "" if ok else note))
        if not ok:
            bad += 1
    if bad == 0 and args.save:
        sok, sres = fc.save()
        print("  %s 落盘: %s" % ("✓" if sok else "✗", sres))
    return 1 if bad else 0


def _ver_str(fc):
    v = fc.version()
    return "%d.%d.%d" % (v["major"], v["minor"], v["patch"]) if v else "?"


def cmd_selftest(fc, args):
    """全链路往返: 读 -> 写同值 -> 回读比对 -> 落盘(可选)。不改任何实际配置。"""
    print("\n=== 调参链路自检 ===")
    rc = 0
    # 1) HEARTBEAT / 身份
    print("[1/5] MAVLink 心跳         : ✓ sys=%d armed=%s" % (fc.sysid, fc.armed))
    # 2) shell 通道 (SERIAL_CONTROL) 由外部脚本另外验, 这里只验 MAVLink
    print("[2/5] PARAM_REQUEST_READ   : ", end="")
    probe = "SYS_AUTOSTART"
    got = fc.read([probe])
    if probe not in got:
        print("✗ 读不到 %s" % probe)
        return 1
    val, tname = got[probe]
    print("✓ %s" % fmt(probe, val, tname))
    # 3) 整表
    print("[3/5] PARAM_REQUEST_LIST   : ", end="")
    t0 = time.time()
    allp, total = fc.list_all(timeout=args.timeout)
    print("✓ 读到 %d/%d 个 (%.1fs)" % (len(allp), total or 0, time.time() - t0))
    # 4) 写同值往返
    print("[4/5] PARAM_SET 往返(同值)  : ", end="")
    if fc.armed:
        print("跳过 (已解锁)")
    else:
        ok, old, new, note = fc.write_one(probe, val)
        if ok:
            print("✓ %s %s -> %s (值未变)" % (probe, old, new))
        else:
            print("✗ %s" % note)
            rc = 1
    # 5) 落盘通道
    print("[5/5] PREFLIGHT_STORAGE   : ", end="")
    sok, sres = fc.save()
    print("%s %s" % ("✓" if sok else "✗", sres))
    if not sok:
        rc = 1
    print("\n结论: %s" % ("链路可用 ✓" if rc == 0 else "有环节失败 ✗"))
    return rc


def cmd_shell(fc, args):
    """把连接交给 PX4 自带 mavlink_shell.py (真交互, 需 tty)。

    ⚠ 危险: 这个脚本只有在**正常退出**时才发 close。被 kill/timeout 掉会留下
    孤儿 shell 会话, 严重时把 FC 的 mavlink 收发线程卡死 (只能拔插 USB 复位)。
    非交互场景请用 `nsh`。
    """
    fc.m.close()
    sh = os.path.expanduser("~/PX4-Autopilot/Tools/mavlink_shell.py")
    if not os.path.exists(sh):
        print("找不到 %s" % sh)
        return 1
    print("⚠ 退出请用 Ctrl-D (不要 kill); 卡死就拔插 USB")
    print("打开 NSH shell; 常用: param show EKF2_* / param set X 1 / param save")
    return os.execv(sys.executable, [sys.executable, sh, fc.port])


def cmd_nsh(fc, args):
    """一次性执行 NSH 命令, 无论成功失败都保证发 close —— 安全版 shell。"""
    cmd = " ".join(args.command)
    if not cmd.strip():
        print("用法: fc_param.py nsh \"param show EKF2_HGT_REF\"")
        return 2
    closed = {"done": False}

    def _close(*_a):
        if closed["done"]:
            return
        closed["done"] = True
        for _ in range(3):          # 多发几次, 确保到达
            try:
                fc.nsh_close()
            except Exception:
                pass
            time.sleep(0.05)

    atexit.register(_close)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        try:
            signal.signal(sig, lambda s, f: (_close(), sys.exit(128 + s)))
        except (ValueError, OSError):
            pass

    print("nsh> %s" % cmd)
    try:
        fc.nsh_write("\n")                    # 唤醒 shell
        time.sleep(0.4)
        fc.nsh_write(cmd + "\n")
        out = fc.nsh_read(quiet=args.quiet, maxt=args.timeout)
        # 去掉回显与 ANSI 控制符
        clean = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", out).replace("\r", "")
        lines = [l for l in clean.split("\n")]
        for l in lines:
            if l.strip():
                print("  " + l.rstrip())
        if not clean.strip():
            print("  <无回显>")
    finally:
        _close()
    return 0


def main():
    ap = argparse.ArgumentParser(description="PX4 飞控直连调参工具 (USB/MAVLink)")
    ap.add_argument("--port", help="串口, 默认自动探测 /dev/serial/by-id 里的 PX4/CUAV 口")
    ap.add_argument("--baud", type=int, default=57600)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info", help="板子信息 + 参数表")
    p = sub.add_parser("list", help="列出参数")
    p.add_argument("--filter", help="子串过滤, 如 EKF2_")
    p.add_argument("--timeout", type=float, default=40.0)
    p = sub.add_parser("get", help="读参数")
    p.add_argument("names", nargs="+")
    p = sub.add_parser("set", help="写参数 (RAM; --save 才进 flash)")
    p.add_argument("name")
    p.add_argument("value")
    p.add_argument("--save", action="store_true")
    p.add_argument("--no-verify", action="store_true")
    sub.add_parser("save", help="把当前参数落盘 flash")
    p = sub.add_parser("dump", help="把全部参数备份成文本文件")
    p.add_argument("file", nargs="?", help="输出文件, 默认 params_dump_<时间戳>.txt")
    p.add_argument("--timeout", type=float, default=40.0)
    p = sub.add_parser("diff", help="对比飞控现值与备份文件")
    p.add_argument("file")
    p.add_argument("--timeout", type=float, default=40.0)
    p.add_argument("--strict", action="store_true",
                   help="把'画像有、飞控未上报'也当差异 (默认只提示)")
    p = sub.add_parser("restore", help="按备份文件还原差异项 (默认只演练)")
    p.add_argument("file")
    p.add_argument("--yes", action="store_true", help="真的写入 (默认只演练)")
    p.add_argument("--save", action="store_true", default=True, help="写完后落盘")
    p.add_argument("--timeout", type=float, default=40.0)
    p = sub.add_parser("selftest", help="全链路往返自检 (只写同值)")
    p.add_argument("--timeout", type=float, default=40.0)
    p = sub.add_parser("nsh", help="一次性执行 NSH 命令 (安全版 shell, 必发 close)")
    p.add_argument("command", nargs=argparse.REMAINDER, help='如: param show EKF2_HGT_REF')
    p.add_argument("--quiet", type=float, default=0.7)
    p.add_argument("--timeout", type=float, default=8.0)
    sub.add_parser("shell", help="打开交互式 NSH shell (⚠ 被 kill 会卡死 FC, 慎用)")

    args = ap.parse_args()
    port = find_port(args.port)
    if not port:
        print("✗ 没找到串口设备")
        return 2
    fc = FC(port, baud=args.baud)
    return {"info": cmd_info, "list": cmd_list, "get": cmd_get, "set": cmd_set,
            "save": cmd_save, "selftest": cmd_selftest, "shell": cmd_shell,
            "nsh": cmd_nsh, "dump": cmd_dump, "diff": cmd_diff,
            "restore": cmd_restore}[args.cmd](fc, args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except RuntimeError as e:
        print("✗ %s" % e)
        sys.exit(2)
