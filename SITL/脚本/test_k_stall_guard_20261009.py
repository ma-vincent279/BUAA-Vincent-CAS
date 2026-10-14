#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k 离线验收 — 「前飞无进展」保护 + 日志记录飞控 landed (2026-10-09 patch_k_stall_guard)

背景 (2026-10-09 14:41 实飞, 用户: "飞机直接在地上拖着飞, 这到底是不是代码的问题?"):
  新加的 FC/EKF 字段给出铁证 —— t=17.8 起 `d8=kill switch`(电机已停、飞机在地面), 而飞控
  自报 H 仍从 1.07 平滑"降"到 0.22 (12s, vz=+0.07) ⇒ **近地高度估计偏高 ~0.85m** ⇒
  命令 1.2m 物理上只剩 ~0.2m ⇒ 贴地拖行; 与此同时 FORWARD 19s 只前进 0.2m (dist 3.1→2.9)。
  **不是控制逻辑问题**, 但节点该在"拖行"时报警+停机, 而不是拖十几秒不说话。

四节
----
A 最小改动审计 (vs `*.bak.*_stall`): 新增行全在允许集合, 删除行只能是 FORWARD 段被替换的
  那一行 (STEP_TIMEOUT 行)。
B **真代码路径 exec**: 从 k.py 抽出真实守卫块执行 —— 正常不触发 / 复现 14:41 触发
  (ERROR 含 landed,H,vNED + 发 21 降落 + `_shutdown("forward_stall")`) / 阈值与时间边界 /
  幂等只触发一次。
C **不误触发证明**: 用两趟实飞的实测前进量对比阈值 (13:59 正常 0.41m/s ⇒ 6s≈2.46m;
  14:41 病态 19s 只 0.2m)。
D 降落指令参数与 LAND 段一致 + 两份 k 一致 + 语法编译。

用法: python3 scripts/test_k_stall_guard_20261009.py
"""
import ast
import difflib
import glob
import math
import pathlib
import re
import sys
import textwrap
import types

REPO = pathlib.Path(__file__).resolve().parent.parent
K_PY = REPO / "src/drone_drl/drone_drl/k.py"
K_FLY = REPO / "fly/src/fly/fly/k.py"
BAK_GLOB = "*_stall"

n_pass = n_fail = 0


def check(cond, msg):
    global n_pass, n_fail
    if cond:
        n_pass += 1
        print(f"  ✓ {msg}")
    else:
        n_fail += 1
        print(f"  ✗ {msg}")
    return bool(cond)


def strip_comments(path):
    return [ln.rstrip() for ln in path.read_text(encoding="utf-8").splitlines()
            if not ln.strip().startswith("#")]


def latest_bak(path):
    c = sorted(glob.glob(str(path) + "." + BAK_GLOB))
    return pathlib.Path(c[-1]) if c else None


WANT = {"K_FWD_STALL_S", "K_FWD_STALL_M", "K_FLIGHT_SPEED", "POS_ARRIVE_DIST", "FORWARD_DIST"}


def load_consts(path):
    src = path.read_text(encoding="utf-8")
    ns = {"math": math}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in WANT for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    return {k: ns[k] for k in WANT if k in ns}


# 2026-10-09: 后续就地补丁的标记集中登记 (见 scripts/_later_patch_marks.py)
try:
    from _later_patch_marks import later_than, \
        LATER_PATCH_BAK_SUFFIXES as _SHARED_BAK
except Exception:      # 单独拷出去跑时退化为空表
    _SHARED, _SHARED_BAK = (), ()

# 本脚本是『当前 vs 本补丁前备份』的差集审计 ⇒ 之后任何就地补丁都要按块归因
# ⚠ 本脚本审计 patch_k_stall_guard ⇒ 只归因**晚于它**的补丁
LATER_PATCH_MARKERS = later_than("stall")

ADD_ALLOW_EXACT = (
    "K_FWD_STALL_S = 6.0     # 2026-10-09 追加 36: 进 FORWARD 这么久之后仍无进展 ⇒ 判异常",
    "K_FWD_STALL_M = 0.80    #   (沿航向前进不足此值 ⇒ 见 FORWARD 段的无进展保护)",
    "if self._step_elapsed() > K_FWD_STALL_S and not getattr(self, '_fwd_stall_logged', False):",
    "_st_fwd = ((self.x - self.home_x) * math.cos(self._heading0)",
    "+ (self.y - self.home_y) * math.sin(self._heading0))",
    "if _st_fwd < K_FWD_STALL_M:",
    "self._fwd_stall_logged = True",
    "self.get_logger().error(",
    "f\"⚠ FORWARD 无进展: {self._step_elapsed():.1f}s 只前进 {_st_fwd:.2f}m\"",
    "f\" (< {K_FWD_STALL_M}m, 正常应 ≥{K_FWD_STALL_S * 0.4:.1f}m); \"",
    "f\"飞控报 landed={int(bool(self.landed))}\"",
    "f\" H={self.home_z - self.z:.2f}m\"",
    "f\" vNED=({self.vx:+.2f},{self.vy:+.2f},{self.vz:+.2f})\"",
    "f\" ⇒ 高度/位置估计不可信或推力不足 (贴地拖行?), 请求降落并退出任务\")",
    "self._send_cmd(21, param1=0.0, param2=0.0, param7=float('nan'))",
    "self._land_cmd_sent = True",
    "self._shutdown(\"forward_stall\"); return",
    "f\" d{int(getattr(_st, 'latest_disarming_reason', 0))}\"",
    "f\" L{1 if self.landed else 0})\")",
)
DEL_ALLOW = (
    "if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_HOVER_SEARCH); return",
    "f\" d{int(getattr(_st, 'latest_disarming_reason', 0))})\")",
)


def section_a():
    print("\n[A] 最小改动审计")
    for f in (K_PY, K_FLY):
        bak = latest_bak(f)
        if not check(bak is not None, f"{f.name}: 找到补丁前备份"):
            continue
        a, b = strip_comments(bak), strip_comments(f)
        sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
        added, removed, bad, later = [], [], [], 0
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag == "equal":
                continue
            if any(mk in "\n".join(b[j1:j2] + a[i1:i2]) for mk in LATER_PATCH_MARKERS):
                later += len(b[j1:j2]) + len(a[i1:i2])     # 后续补丁块, 由它自己的验收负责
                continue
            if tag in ("insert", "replace"):
                for ln in b[j1:j2]:
                    added.append(ln)
                    if ln.strip() not in ADD_ALLOW_EXACT:
                        bad.append("+" + ln)
            if tag in ("delete", "replace"):
                for ln in a[i1:i2]:
                    removed.append(ln)
                    if ln.strip() not in DEL_ALLOW:
                        bad.append("-" + ln)
        check(not bad, f"{f.name}: {len(added)} 新增 / {len(removed)} 删除 全在允许集合内"
                       + ("" if not bad else f" (越界 {bad[:2]})"))
        check(len(removed) <= 2, f"{f.name}: 删除 {len(removed)} 行 (≤2, 只替换 FORWARD 首行与 FC 串尾)")
        if later:
            check(True, f"{f.name}: {later} 行归因于后续补丁 (5m 巡航剖面等), 由各自验收覆盖")
        forbid = ("_pub_", "publish", "self.step =", "create_publisher", "self._trackers =")
        hit = [k for k in forbid if k in "\n".join(added)]
        check(not hit, f"{f.name}: 新增行里无发布/状态机写入 (命中 {hit})")


class _Log:
    def __init__(self): self.errors = []

    def error(self, m): self.errors.append(m)


class _Self:
    def __init__(self, elapsed, travel, landed=False, heading0=math.radians(212.0),
                 z=-1.0, home_z=0.0, v=(0.2, 0.17, 0.07)):
        # home=(0,0); 沿 heading0 前进 travel 米
        self.home_x = self.home_y = 0.0
        self.x = travel * math.cos(heading0)
        self.y = travel * math.sin(heading0)
        self._heading0 = heading0
        self.home_z, self.z = home_z, z
        self.vx, self.vy, self.vz = v
        self.landed = landed
        self._t = elapsed
        self.sent = []
        self._land_cmd_sent = False
        self.shutdown_reason = None
        self._log = _Log()

    def _step_elapsed(self): return self._t

    def get_logger(self): return self._log

    def _send_cmd(self, cmd, **p): self.sent.append((cmd, p))

    def _shutdown(self, reason="unknown"): self.shutdown_reason = reason


def _extract_guard():
    src = K_PY.read_text(encoding="utf-8")
    i = src.index('# 2026-10-09 追加 36: 前飞"无进展"保护')
    i = src.rindex("\n", 0, i) + 1
    j = src.index('self._shutdown("forward_stall"); return', i)
    j = src.index("\n", j) + 1
    return textwrap.dedent(src[i:j])


def section_b():
    print("\n[B] 真代码路径 exec (从 k.py 抽出的实际守卫块)")
    block = _extract_guard()
    code = "def _guard(self):\n" + textwrap.indent(block, "    ") + "    return 'CONTINUE'\n"
    ns = {"math": math, "getattr": getattr}
    ns.update(load_consts(K_PY))
    exec(code, ns)
    guard = ns["_guard"]
    S, M = ns["K_FWD_STALL_S"], ns["K_FWD_STALL_M"]
    print(f"      K_FWD_STALL_S={S}s  K_FWD_STALL_M={M}m")

    # B1 正常飞行 (13:59 实测 0.41 m/s ⇒ 6s 走 2.46m)
    s = _Self(elapsed=6.5, travel=2.46)
    check(guard(s) == "CONTINUE" and not s._log.errors and not s.sent and s.shutdown_reason is None,
          "正常 (6.5s 前进 2.46m) ⇒ 不报错/不发降落/不退出")

    # B2 复现 14:41: 6.5s 只前进 0.20m, 飞控报 landed
    s = _Self(elapsed=6.5, travel=0.20, landed=True)
    got = guard(s)
    check(got is None and len(s._log.errors) == 1, "复现 14:41 (6.5s 只前 0.20m) ⇒ 触发 ERROR")
    msg = s._log.errors[0] if s._log.errors else ""
    check("FORWARD 无进展" in msg and "landed=1" in msg and "H=" in msg and "vNED=" in msg,
          f"ERROR 含 landed/H/vNED: '{msg[:88]}…'")
    check(s.sent and s.sent[0][0] == 21, f"发出 MAV_CMD_NAV_LAND (cmd={s.sent[0][0] if s.sent else None})")
    check(s._land_cmd_sent is True, "置 _land_cmd_sent (与 LAND 段同一语义)")
    check(s.shutdown_reason == "forward_stall", f"退出原因 = {s.shutdown_reason}")
    # 参数与 LAND 段一致
    land_src = K_PY.read_text(encoding="utf-8")
    check("self._send_cmd(21, param1=0.0, param2=0.0, param7=float('nan'))" in land_src
          and s.sent[0][1] == {"param1": 0.0, "param2": 0.0, "param7": s.sent[0][1]["param7"]},
          "降落指令参数与 LAND 段一致 (param1=0, param2=0, param7=nan)")

    # B3 阈值边界
    check(guard(_Self(elapsed=6.5, travel=M - 0.01)) is None, f"travel {M - 0.01:.2f}m < {M} ⇒ 触发")
    s = _Self(elapsed=6.5, travel=M + 0.01)
    check(guard(s) == "CONTINUE" and not s._log.errors, f"travel {M + 0.01:.2f}m ≥ {M} ⇒ 不触发")

    # B4 时间边界
    s = _Self(elapsed=S - 0.1, travel=0.1)
    check(guard(s) == "CONTINUE" and not s._log.errors, f"{S - 0.1}s (< {S}s) ⇒ 还不判 (留出正常加速时间)")

    # B5 幂等: 只报一次, 不会重复发降落
    s = _Self(elapsed=7.5, travel=0.2)
    guard(s); n1 = len(s.sent)
    guard(s); guard(s)
    check(n1 == 1 and len(s.sent) == 1 and len(s._log.errors) == 1,
          "幂等: 连续调用只报一次/只发一次降落")

    # B6 横向偏移不该影响判据 (只看沿航向)
    s = _Self(elapsed=6.5, travel=2.46, heading0=math.radians(212.0))
    s.x += 0.5 * math.sin(math.radians(212.0))     # 加横向
    s.y -= 0.5 * math.cos(math.radians(212.0))
    check(guard(s) == "CONTINUE", "加 0.5m 横向偏移后仍按沿航向前进量判定 (不误触发)")


def section_c():
    print("\n[C] 不误触发证明 (用两趟实飞的实测前进量)")
    c = load_consts(K_PY)
    S, M, SPD = c["K_FWD_STALL_S"], c["K_FWD_STALL_M"], c["K_FLIGHT_SPEED"]
    good = 0.41                     # 13:59 那趟 FORWARD 实测速度 (命令 0.6, 实测 0.41)
    print(f"      命令速度 {SPD}m/s, 13:59 实测 {good}m/s ⇒ {S}s 正常前进 ≈ {S * good:.2f}m")
    check(S * good > 2.0 * M, f"正常前进 {S * good:.2f}m > 2×阈值 {2 * M:.2f}m ⇒ 余量充足, 不会误触发")
    bad = 0.20 / 19.0               # 14:41 那趟: 19s 只前进 0.2m
    check(S * bad < 0.5 * M, f"14:41 病态前进 {S * bad:.3f}m (6s) < 阈值一半 ⇒ 必然触发")
    # 即使速度只有命令的 1/3 也不该触发
    worst = SPD / 3.0
    check(S * worst > M, f"最坏假设(达速仅 1/3 = {worst:.2f}m/s) ⇒ {S * worst:.2f}m > {M}m ⇒ 仍不误触发")


def section_d():
    print("\n[D] 一致性与语法")
    # ⚠ 结构不变量: 第一版补丁把 `elif self.step == STEP_FORWARD:` 整行删掉了,
    #   守卫块于是变成**上一个分支**的语句 —— 编译照样通过, 只有这条断言抓得住。
    for f in (K_PY, K_FLY):
        src = f.read_text(encoding="utf-8")
        check(src.count("elif self.step == STEP_FORWARD:") == 1,
              f"{f.name}: `elif self.step == STEP_FORWARD:` 恰 1 处 (分支未被吞)")
        i = src.index("elif self.step == STEP_FORWARD:")
        j = src.index("K_FWD_STALL_S and not getattr", i)
        check(j > i and src[i:j].count("\n        elif ") == 0,
              f"{f.name}: 守卫块在 FORWARD 分支体内 (12 空格缩进, 未越到别的分支)")

    for f in (K_PY, K_FLY):
        try:
            compile(f.read_text(encoding="utf-8"), str(f), "exec")
            check(True, f"{f.name}: 语法编译通过")
        except SyntaxError as e:
            check(False, f"{f.name}: 语法错误 {e}")
    blocks = []
    for f in (K_PY, K_FLY):
        src = f.read_text(encoding="utf-8")
        i = src.index('# 2026-10-09 追加 36: 前飞"无进展"保护')
        i = src.rindex("\n", 0, i) + 1
        j = src.index('self._shutdown("forward_stall"); return', i)
        blocks.append(src[i:src.index("\n", j) + 1])
    check(len(set(blocks)) == 1, "两份 k 的守卫块逐字节相同")
    check(load_consts(K_PY)["K_FWD_STALL_S"] == load_consts(K_FLY)["K_FWD_STALL_S"]
          and load_consts(K_PY)["K_FWD_STALL_M"] == load_consts(K_FLY)["K_FWD_STALL_M"],
          "两份 k 的 K_FWD_STALL_* 一致")
    # landed 字段现在必须进 1Hz 行
    src = K_PY.read_text(encoding="utf-8")
    check('f" L{1 if self.landed else 0})")' in src, "1Hz 行 FC 串已追加 landed 标志 L{0/1}")


def main():
    print("=" * 84)
    print("k 离线验收 — 前飞无进展保护 + landed 记录 (2026-10-09 patch_k_stall_guard)")
    print("=" * 84)
    section_a()
    section_b()
    section_c()
    section_d()
    print("\n" + "=" * 84)
    print(f"结果: {n_pass} 项通过 / {n_fail} 项失败")
    print("=" * 84)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
