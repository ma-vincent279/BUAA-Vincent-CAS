#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_ekf_tlm_20261009.py — 给 k/l 的 1Hz 状态行补上「飞控模式 + EKF 健康度」

为什么 (2026-10-09 13:59 实飞复盘):
  这趟日志显示 t=15.8 起「命令 vz=-0.10 (爬升) 而实测 vz=+0.05 (下降)」、姿态几乎冻住
  (rol=+0.9 连续 8s)、且 t=20.8~22.8 位置说下降 0.27m/s 而同帧 vz 说上升 +0.06~+0.16
  ⇒ EKF 内部自相矛盾 (垂向估计失效) 是当前最强假设, 但节点日志里**没有任何 EKF 状态字段**
  ⇒ 无法确认, 也无法确认 t=18.6 的 ARM/OFFBOARD 丢失是「机手 kill switch」还是「飞控 failsafe」。

本补丁只加**观测**, 不改任何控制行为:
  A) __init__ 新增 self._ekf / self._ekf_warn_t
  B) _cb_local 里快照 EKF 健康度: xy/z/vxy/vz 有效性 + dead_reckoning + 4 个复位计数 + 对地距离
  C) 1Hz 行末尾追加 FC(a.. n.. fs.. d..) 与 EKF(.... dr. rst../../../.. dbt=..)
     字段追加在**行尾**, 不移动任何既有字段位置 (既有解析脚本不受影响)
  D) EKF 垂向估计不可信时打节流 WARN (5s), 提示机手准备接管

FC 字段: a=arming_state (1=未解锁 2=已解锁) n=nav_state (0=MANUAL 2=POSCTL 3=AUTO_MISSION
         4=AUTO_LOITER 5=AUTO_RTL 6=AUTO_LAND 14=OFFBOARD) fs=failsafe d=latest_disarming_reason
         (1=摇杆手势 2=RC开关 3=内部指令 4=外部指令 6=kill switch 8=failsafe 9=关机)
EKF 字段: 4 位数字 = xy,z,v_xy,v_z 是否有效 (1=有效) / dr=dead_reckoning / rst=xy/z/vxy/vz 复位计数
         / dbt=对地距离 (m, -- = 无测距可用)。**z 位为 0 或 dr=1 ⇒ 垂向估计不可信。**

用法: python3 scripts/patch_ekf_tlm_20261009.py [--check]
"""
import argparse
import datetime
import hashlib
import pathlib
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = [
    REPO / "src/drone_drl/drone_drl/k.py",
    REPO / "src/drone_drl/drone_drl/l.py",
    REPO / "fly/src/fly/fly/k.py",
    REPO / "fly/src/fly/fly/l.py",
]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_ekf_tlm"

# ── A) __init__ ────────────────────────────────────────────────────────────
A_ANCHOR = "        self.status = None\n        self._control_mode = None\n"
A_INSERT = (
    "        # 2026-10-09: EKF 健康度快照 (1Hz 行 + 垂向失效 WARN 用; 只读, 不参与控制)\n"
    "        self._ekf = None\n"
    "        self._ekf_warn_t = 0.0\n"
)

# ── B) _cb_local ───────────────────────────────────────────────────────────
B_ANCHOR = "        if math.isfinite(m.vz): self.vz = m.vz\n"
B_INSERT = (
    "        # 2026-10-09: EKF 健康度快照 —— 0~3 有效性(xy,z,vxy,vz) 4 dead_reckoning\n"
    "        #            5~8 复位计数(xy,z,vxy,vz) 9 对地距离(None=无测距)\n"
    "        self._ekf = (bool(m.xy_valid), bool(m.z_valid), bool(m.v_xy_valid), bool(m.v_z_valid),\n"
    "                     bool(m.dead_reckoning), int(m.xy_reset_counter), int(m.z_reset_counter),\n"
    "                     int(m.vxy_reset_counter), int(m.vz_reset_counter),\n"
    "                     (float(m.dist_bottom) if bool(m.dist_bottom_valid) else None))\n"
)

# ── C) 1Hz 输出 ────────────────────────────────────────────────────────────
C_ANCHOR = (
    '            self.get_logger().info(\n'
    '                f"[t={t_mission:7.1f} {tag:<15s} ph={self._step_elapsed():5.1f}s] "\n'
    '                f"NED({self.x:7.2f},{self.y:7.2f},{self.z:6.2f}) H={H_str} "\n'
    '                f"hdg={hdg:6.1f} pit={math.degrees(self._pitch):+5.1f} rol={math.degrees(self._roll):+5.1f} "\n'
    '                f"vNED=({self.vx:+.2f},{self.vy:+.2f},{self.vz:+.2f}) "\n'
    '                f"{has_det} age={det_age:4.2f}{extra}")\n'
)
C_REPLACE = (
    '            # ===== 2026-10-09: 飞控状态 + EKF 健康度 (追加在行尾, 既有字段位置不动) =====\n'
    '            # FC(a=arming 1未2已 n=nav 14=OFFBOARD 2=POSCTL 5=RTL 6=LAND fs=failsafe d=disarm原因)\n'
    '            # EKF(xy z vxy vz | dr=dead_reckoning | rst=xy/z/vxy/vz 复位计数 | dbt=对地距离m)\n'
    '            _st = self.status\n'
    '            if _st is None:\n'
    '                fc_str = " FC(--)"\n'
    '            else:\n'
    '                fc_str = (f" FC(a{int(_st.arming_state)} n{int(_st.nav_state)}"\n'
    '                          f" fs{1 if bool(getattr(_st, \'failsafe\', False)) else 0}"\n'
    '                          f" d{int(getattr(_st, \'latest_disarming_reason\', 0))})")\n'
    '            _ek = self._ekf\n'
    '            if _ek is None:\n'
    '                ek_str = " EKF(--)"\n'
    '            else:\n'
    '                _dbt = "--" if _ek[9] is None else f"{_ek[9]:.2f}"\n'
    '                ek_str = (f" EKF({int(_ek[0])}{int(_ek[1])}{int(_ek[2])}{int(_ek[3])}"\n'
    '                          f" dr{int(_ek[4])} rst{_ek[5]}/{_ek[6]}/{_ek[7]}/{_ek[8]} dbt={_dbt})")\n'
    '                # 垂向估计不可信 (z 无效 / vz 无效 / 纯惯导递推) ⇒ 高度环可能失效, 喊人接管\n'
    '                if ((not _ek[1]) or (not _ek[3]) or _ek[4]):\n'
    '                    _now_s = self.get_clock().now().nanoseconds * 1e-9\n'
    '                    if _now_s - self._ekf_warn_t > 5.0:\n'
    '                        self._ekf_warn_t = _now_s\n'
    '                        self.get_logger().warn(\n'
    '                            f"⚠ EKF 垂向估计不可信 (z_valid={int(_ek[1])}"\n'
    '                            f" v_z_valid={int(_ek[3])} dead_reckoning={int(_ek[4])}"\n'
    '                            f" z_reset={_ek[6]}) ⇒ 高度环可能失效 (指令爬升却下沉), 立即准备手操接管!")\n'
    '            self.get_logger().info(\n'
    '                f"[t={t_mission:7.1f} {tag:<15s} ph={self._step_elapsed():5.1f}s] "\n'
    '                f"NED({self.x:7.2f},{self.y:7.2f},{self.z:6.2f}) H={H_str} "\n'
    '                f"hdg={hdg:6.1f} pit={math.degrees(self._pitch):+5.1f} rol={math.degrees(self._roll):+5.1f} "\n'
    '                f"vNED=({self.vx:+.2f},{self.vy:+.2f},{self.vz:+.2f}) "\n'
    '                f"{has_det} age={det_age:4.2f}{extra}{fc_str}{ek_str}")\n'
)

MARK = "self._ekf = None"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path: pathlib.Path, check: bool) -> str:
    src = path.read_text(encoding="utf-8")

    if MARK in src:
        return "SKIP (已打过, 幂等)"

    # 锚点唯一性断言 —— 不唯一立即中止, 绝不猜测
    for name, anchor in (("A", A_ANCHOR), ("B", B_ANCHOR), ("C", C_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"  ✗ {path}: 锚点 {name} 命中 {n} 次 (要求恰 1 次) ⇒ 中止", file=sys.stderr)
            sys.exit(2)

    out = src.replace(A_ANCHOR, A_ANCHOR + A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_ANCHOR + B_INSERT, 1)
    out = out.replace(C_ANCHOR, C_REPLACE, 1)

    # 编译校验 (语法必须过, 否则不落盘)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 打补丁后语法错误 {e} ⇒ 中止", file=sys.stderr)
        sys.exit(3)

    if check:
        return f"CHECK-OK (将改 {len(out) - len(src):+d} 字节)"

    bak = path.with_name(path.name + "." + BAK_SUFFIX)
    shutil.copy2(path, bak)
    path.write_text(out, encoding="utf-8")
    return f"OK  {sha(path)}  (备份 {bak.name})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只校验锚点与语法, 不落盘")
    a = ap.parse_args()
    print(f"=== patch_ekf_tlm_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(4)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
