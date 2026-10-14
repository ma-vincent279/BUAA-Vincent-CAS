#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_stall_guard_20261009.py — k 增补「前飞无进展」保护 + 日志记录飞控 landed (只 k 两份)

为什么 (2026-10-09 14:41 实飞, 用户: "飞机直接在地上拖着飞, 这到底是不是代码的问题?"):
  字段解码 (新加的 FC/EKF 观测起作用了):
    t=0.8~11.8   FC(a2 n14 fs0 d0)   已解锁 + OFFBOARD  ⇒ 节点在控制
    t=12.8~16.8  FC(a2 n1  fs0 d0)   机手切 ALTCTL     ⇒ 飞控不再执行节点指令
    t=17.8~29.8  FC(a1 n1  fs0 d8)   **d8 = kill switch** ⇒ 电机停
    t=17.8→29.8  H 1.07 → 0.22 (vz=+0.07 稳定 12s) —— **电机已停、飞机在地面**,
                 飞控自报高度却"平滑下降" 0.85m ⇒ 这段位移**纯属估计跑了** ⇒
                 **近地高度估计偏高 ~0.85m** ⇒ 命令 1.2m 物理上只剩 ~0.2m ⇒ 贴地拖行
    全程         dist_to_target 3.1→2.9m (19s 只前进 0.2m) 而 vNED 报 0.2m/s ⇒ 位置几乎不动
    → 结论: **不是任务控制逻辑问题**(指令与飞控自报状态自洽; 5m 剖面能飞), 是飞控
      高度估计在近地被污染 (下洗/气压计, `dbt=--` 无测距可纠正)。

  但**节点该做而没做**: 它订阅了 `/fmu/out/vehicle_land_detected` (`self.landed`) 却
  从未记录、从未利用 (全日志 0 次) ⇒ 三趟都在"命令 1.2m / 飞控可能认为已落地"的矛盾里
  拖了十几秒而日志一个字不说。本补丁补两件事:
    A) 常量 K_FWD_STALL_S / K_FWD_STALL_M
    B) FORWARD 段「无进展」保护: 进 FORWARD 超过 K_FWD_STALL_S 而沿航向前进不足
       K_FWD_STALL_M ⇒ ERROR (含 landed/H/vNED) + **发 MAV_CMD_NAV_LAND 让飞控停机**
       (避免电机在地面持续打桨) + 退出 (`forward_stall`)
    C) 1Hz 行 FC 串追加 `L{0/1}` = 飞控 landed 标志 (真飞控自己说"我在地上")

用法: python3 scripts/patch_k_stall_guard_20261009.py [--check]
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
    REPO / "fly/src/fly/fly/k.py",
]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_stall"

# ── A) 常量 ───────────────────────────────────────────────────────────────
A_ANCHOR = "                        #   0.60 ∈ (0.5, 0.673) ✓ 且门控里另有结构性下界 max(·, POS_ARRIVE_DIST)\n"
A_INSERT = (
    "K_FWD_STALL_S = 6.0     # 2026-10-09 追加 36: 进 FORWARD 这么久之后仍无进展 ⇒ 判异常\n"
    "K_FWD_STALL_M = 0.80    #   (沿航向前进不足此值 ⇒ 见 FORWARD 段的无进展保护)\n"
)

# ── B) FORWARD 无进展保护 ─────────────────────────────────────────────────
# ⚠ 锚点只用**代码行**: 两副本的段落注释不同 (SITL 副本被早前脚本改写过, fly 副本仍是
#   i 线原文 `# ---- FORWARD: 沿 heading0 盲飞 32m ----`) ⇒ 带注释的锚点在 fly 命中 0 次。
B_ANCHOR = (
    "        elif self.step == STEP_FORWARD:\n"
    "            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_HOVER_SEARCH); return\n"
)
# ⚠ 必须把 elif 行本身原样写回去 —— 第一版漏了它, 结果 FORWARD 分支的 elif 被删掉,
#   守卫块变成上一个分支的语句 (编译仍通过 ⇒ 只有结构审计才抓得住)。
B_REPLACE_HEAD = "        elif self.step == STEP_FORWARD:\n"
B_REPLACE_TAIL = "            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_HOVER_SEARCH); return\n"
B_INSERT = (
    "            # 2026-10-09 追加 36: 前飞\"无进展\"保护。14:41 实飞飞机**在地面上拖行**:\n"
    "            #   飞控高度估计近地偏高 ~0.85m (kill switch 后电机已停、飞机在地面, 自报 H 仍从\n"
    "            #   1.07 平滑\"降\"到 0.22) ⇒ 命令 1.2m 物理上只有 ~0.2m; 而 FORWARD 19s 只前进\n"
    "            #   0.2m (vNED 却报 0.2m/s)。正常 K_FWD_STALL_S 内应前进 ~2.4m (13:59 实测\n"
    "            #   0.41m/s) ⇒ 前进 < K_FWD_STALL_M 判为异常: 请求降落让飞控停机 (避免电机在\n"
    "            #   地面持续打桨) 并退出, 同时把 landed/H/vNED 写进日志便于复盘。\n"
    "            if self._step_elapsed() > K_FWD_STALL_S and not getattr(self, '_fwd_stall_logged', False):\n"
    "                _st_fwd = ((self.x - self.home_x) * math.cos(self._heading0)\n"
    "                           + (self.y - self.home_y) * math.sin(self._heading0))\n"
    "                if _st_fwd < K_FWD_STALL_M:\n"
    "                    self._fwd_stall_logged = True\n"
    "                    self.get_logger().error(\n"
    "                        f\"⚠ FORWARD 无进展: {self._step_elapsed():.1f}s 只前进 {_st_fwd:.2f}m\"\n"
    "                        f\" (< {K_FWD_STALL_M}m, 正常应 ≥{K_FWD_STALL_S * 0.4:.1f}m); \"\n"
    "                        f\"飞控报 landed={int(bool(self.landed))}\"\n"
    "                        f\" H={self.home_z - self.z:.2f}m\"\n"
    "                        f\" vNED=({self.vx:+.2f},{self.vy:+.2f},{self.vz:+.2f})\"\n"
    "                        f\" ⇒ 高度/位置估计不可信或推力不足 (贴地拖行?), 请求降落并退出任务\")\n"
    "                    self._send_cmd(21, param1=0.0, param2=0.0, param7=float('nan'))\n"
    "                    self._land_cmd_sent = True\n"
    "                    self._shutdown(\"forward_stall\"); return\n"
)

# ── C) FC 串追加 landed 标志 ───────────────────────────────────────────────
C_ANCHOR = '                          f" d{int(getattr(_st, \'latest_disarming_reason\', 0))})")\n'
C_REPLACE = ('                          f" d{int(getattr(_st, \'latest_disarming_reason\', 0))}"\n'
             '                          f" L{1 if self.landed else 0})")\n')

MARK = "K_FWD_STALL_S"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path: pathlib.Path, check: bool) -> str:
    src = path.read_text(encoding="utf-8")
    if MARK in src:
        return "SKIP (已打过, 幂等)"
    for name, anchor in (("A", A_ANCHOR), ("B", B_ANCHOR), ("C", C_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"  ✗ {path}: 锚点 {name} 命中 {n} 次 (要求恰 1 次) ⇒ 中止", file=sys.stderr)
            sys.exit(2)
    out = src.replace(A_ANCHOR, A_ANCHOR + A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_REPLACE_HEAD + B_INSERT + B_REPLACE_TAIL, 1)
    out = out.replace(C_ANCHOR, C_REPLACE, 1)
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
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    print(f"=== patch_k_stall_guard_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(4)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
