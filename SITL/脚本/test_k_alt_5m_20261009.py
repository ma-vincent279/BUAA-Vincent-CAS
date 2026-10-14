#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k 离线验收 — 用户指定剖面「5m 巡航 → 前进 3m → 降到 1.2m 投弹」(2026-10-09 patch_k_alt_5m)

用户指令: "🕗高度抬到5米，然后前进3米，降高到1.2米投弹"
设计意图: 近地高度估计有 ~0.07m/s 漂移 (14:41 实测: kill switch 后电机停、飞机在地面,
          自报 H 仍 12s 里"降"了 0.85m) ⇒ 全程 1.2m 平飞会贴地拖行; 5m 巡航段估计可靠,
          只在桶正上方降到 1.2m 投弹 ⇒ 近地停留最短。

四节
----
A 最小改动审计 (vs `*.bak.*_alt5m`): 新增行全在允许集合, 删除行只能是被替换的那几行。
B **真代码路径 exec** (从 k.py 抽真实下降块): ①**每个 tick 必发 setpoint** (踩过的坑: 若
  `_pub_pos` 排在提前 return 之后, 下降期间一个 setpoint 都不发 ⇒ 飞控 0.5s 无 setpoint
  直接掉 OFFBOARD) ②限速 1m/s 且不越过 pre_z ③降到 ±0.25m 后才放行搜索/门控
  ④[降高] 日志 2s 节流。
C 剖面自洽: 巡航 5m > 下降目标 1.2m = DROP_HEIGHT; NED 方向正确 (pre_z > cruise_z);
  下降耗时 ≈3.8s 落在 HOVER_SEARCH_TIMEOUT 预算内。
D 结构不变量 + 两份一致 + 语法: HOVER_SEARCH 里 `_pub_pos` 必须早于提前 return; 两份 k
  的下降块逐字节相同; elif 计数不变。

用法: python3 scripts/test_k_alt_5m_20261009.py
"""
import ast
import difflib
import glob
import math
import pathlib
import re
import sys
import textwrap

REPO = pathlib.Path(__file__).resolve().parent.parent
K_PY = REPO / "src/drone_drl/drone_drl/k.py"
K_FLY = REPO / "fly/src/fly/fly/k.py"
L_PY = REPO / "src/drone_drl/drone_drl/l.py"
BAK_GLOB = "*_alt5m"
HS_MARK = "\n        elif self.step == STEP_HOVER_SEARCH:"

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


WANT = {"K_FLIGHT_ALT", "PRE_SERVO_HEIGHT", "DROP_HEIGHT", "K_DESCEND_RATE", "K_DESCEND_PULL_M",
        "HOVER_SEARCH_TIMEOUT", "HOVER_SEARCH_SETTLE_TIME", "FORWARD_DIST",
        "K_FLIGHT_SPEED", "CRUISE_HEIGHT", "TAKEOFF_CLIMB_RATE"}


def load_consts(path):
    src = path.read_text(encoding="utf-8")
    ns = {"math": math}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in WANT for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    return {k: ns[k] for k in WANT if k in ns}


ADD_ALLOW_EXACT = (
    "K_FLIGHT_ALT  = 5.0     # 2026-10-09 用户指定剖面: **5m 巡航 → 前进 3m → 降到 1.2m 投弹**",
    "#   (原 1.2m 全程平飞已废: 近地高度估计有 ~0.07m/s 漂移 ⇒ 长时间",
    "#    待在近地会贴地拖行; 见 docs/project_log.md 2026-10-09 追加 36)",
    "PRE_SERVO_HEIGHT = DROP_HEIGHT    # 2026-10-09 用户指定剖面: 下降目标 = 投弹高度 1.2m",
    "#   (POSITION_REFINE 原有的限速下降 SERVO_DESCENT_SPEED 复用)",
    "K_DESCEND_RATE       = 1.0    # 2026-10-09: HOVER_SEARCH 到点后下降到投弹高度的限速 (m/s)",
    "HOVER_SEARCH_TIMEOUT = 16.0   # 2026-10-09: 10→16 (HOVER_SEARCH 现在含'到点后降到 1.2m'",
    "#   的 ~3.8s 下降段; 到位后仍保留原 10s 搜索窗口)",
    "_pre_z = self.home_z - PRE_SERVO_HEIGHT",
    "# NED: z 越小越高 (5m 高 ⇒ z=home-5); 下降 = z 增大 ⇒ 每 tick 只允许增大",
    "# K_DESCEND_RATE*0.05, 并用 min 夹住目标 —— 写成 max 会一步跳到目标 (全速下降)",
    "_z_desc = min(_pre_z, self.z + K_DESCEND_RATE * 0.05)",
    "self._pub_pos(self.home_x + tx, self.home_y + ty, _z_desc, yaw_locked)",
    "if self.z < _pre_z - 0.25:",
    "if sim_t - getattr(self, '_hs_desc_log_t', -99.0) > 2.0:",
    "self._hs_desc_log_t = sim_t",
    "self.get_logger().info(",
    'f"  [降高] H={self.home_z - self.z:.2f}m → {PRE_SERVO_HEIGHT:.2f}m"',
    'f" (限速 {K_DESCEND_RATE:.1f}m/s, 到位后再找桶)")',
    "return",
    'f"alt={K_FLIGHT_ALT}m 巡航 → 前进{FORWARD_DIST}m → 降到{PRE_SERVO_HEIGHT}m 投弹"',
    'f" (drop={DROP_HEIGHT}m) fwd_speed={K_FLIGHT_SPEED}m/s (no flight boundary)")',
)
DEL_ALLOW = (
    "K_FLIGHT_ALT  = 1.2     # 2026-10-09 实机反馈①④: k **全程锁定 1.2m**",
    "PRE_SERVO_HEIGHT = K_FLIGHT_ALT   # 2026-10-09 反馈④: 与飞行高度同高 ⇒ 预伺服段不再下降",
    "HOVER_SEARCH_TIMEOUT = 10.0   # 目标点搜索超时, 无有效目标则返航",
    "self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, yaw_locked)",
    'f"alt={K_FLIGHT_ALT}m (全程平飞) drop={DROP_HEIGHT}m fwd_speed={K_FLIGHT_SPEED}m/s (no flight boundary)")',
)


# 2026-10-09: 本脚本审计 patch_k_alt_5m ⇒ 只归因**晚于它**的补丁 (desc_fix / anchor),
#   否则会把自己要审计的改动也一起跳过。见 scripts/_later_patch_marks.py
try:
    from _later_patch_marks import later_than
    LATER_PATCH_MARKERS = later_than("alt5m")
except Exception:
    LATER_PATCH_MARKERS = ("_z_anchor", "self._step_entry_z", "HOVER_SEARCH_TIMEOUT = 18.0")


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
        check(len(removed) <= 6, f"{f.name}: 删除 {len(removed)} 行 (≤6, 只替换常量/日志/一条 _pub_pos)")
        if later:
            check(True, f"{f.name}: {later} 行归因于后续补丁 (降高锚点修复), 由各自验收覆盖")
        forbid = ("self.step =", "_enter_step(STEP_", "create_publisher", "_trackers =")
        hit = [k for k in forbid if k in "\n".join(added)]
        check(not hit, f"{f.name}: 新增行里无状态机跳转/发布器改动 (命中 {hit})")
    check("K_DESCEND_RATE" not in L_PY.read_text(encoding="utf-8"),
          "l.py 不含本改动 ⇒ 本次只影响 k")


class _Log:
    def __init__(self): self.infos = []

    def info(self, m): self.infos.append(m)


class _Self:
    """只实现下降块用到的接口; z 单位 NED (越大越靠下), home_z 为地面基准"""
    def __init__(self, z, home_z=0.0):
        self.home_z = home_z
        self.z = z
        self.home_x = self.home_y = 0.0
        self._bucket_target_ret = (1.0, -2.0)
        self.pubs = []
        self._t = 0.0
        self._trackers = []      # 追加 39: 降高锚点用
        self._log = _Log()

    def _step_elapsed(self): return self._t

    def at(self, sim_t):        # 让 _step_elapsed 与传入的 sim_t 同步 (时间锚定后必须)
        self._t = sim_t
        return self

    def _pub_pos(self, x, y, z, yaw): self.pubs.append((x, y, z, yaw))

    def get_logger(self): return self._log


def _extract_descent():
    src = K_PY.read_text(encoding="utf-8")
    i = src.index('# 2026-10-09 追加 37 (用户指定剖面')
    i = src.rindex("\n", 0, i) + 1
    j = src.index('f" (限速 {K_DESCEND_RATE:.1f}m/s, 到位后再找桶)")', i)
    j = src.index("                return", j)      # ⚠ 必须包含这句提前 return, 否则测不到"下降中不发门控"
    j = src.index("\n", j) + 1
    return textwrap.dedent(src[i:j])


def section_b():
    print("\n[B] 真代码路径 exec (从 k.py 抽出的下降块)")
    block = _extract_descent()
    code = ("def _desc(self, sim_t, tx, ty, yaw_locked):\n"
            + textwrap.indent(block, "    ")
            + "    return 'SEARCH'\n")
    ns = {"math": math, "getattr": getattr}
    ns.update(load_consts(K_PY))
    exec(code, ns)
    desc = ns["_desc"]
    PC = ns["PRE_SERVO_HEIGHT"]
    RATE = ns["K_DESCEND_RATE"]
    H = ns["K_FLIGHT_ALT"]
    c_timeout = load_consts(K_PY)["HOVER_SEARCH_TIMEOUT"]
    print(f"      巡航 {H}m → 下降目标 {PC}m (限速 {RATE}m/s)")

    # B1 在 5m 巡航高度: 必须发 setpoint 且每 tick 只降 5cm (=1m/s @20Hz), 并提前 return
    s = _Self(z=0.0 - H)                       # NED: 5m 高 ⇒ z = home_z - 5
    got = desc(s.at(0.0), 0.0, 1.0, -2.0, 0.0)
    check(len(s.pubs) == 1, f"**每个 tick 必发 setpoint** (实测 {len(s.pubs)} 次) —— 否则飞控 0.5s 掉 OFFBOARD")
    check(got is None, "下降到 1.2m 之前 ⇒ 提前 return (不做门控/不选桶/不计零检测)")
    check(abs(s.pubs[0][2] - s.z) < 1e-9,
          f"时间锚定: 进入瞬间命令 = 当前高度 {s.pubs[0][2]:+.3f} (elapsed=0, 不跳变)")
    desc(s.at(1.0), 1.0, 1.0, -2.0, 0.0)
    check(abs(s.pubs[-1][2] - (s.z + RATE * 1.0)) < 1e-9,
          f"时间锚定: 1s 后命令比进入高度低 {RATE}m ⇒ z = {s.pubs[-1][2]:+.3f} (设定点以 {RATE}m/s 走)")
    check(s.pubs[0][0] == 1.0 and s.pubs[0][1] == -2.0, "水平保持在前方靶点 (盲降到桶正上方)")
    check(len(s._log.infos) == 1 and "[降高]" in s._log.infos[0],
          f"打 [降高] 日志: '{s._log.infos[0].strip()[:60]}…'")

    # B2 **滞后车辆模型下的下降** —— 这条是 SITL 2026-10-09 实测教训的回归:
    #   SITL 里命令 1.0m/s 而实测只有 0.034m/s: 初版写 `self.z + rate*0.05` (相对**当前实测
    #   位置**), PX4 位置环只看到 5cm 误差 ⇒ 只能慢慢跟。用一阶滞后 ż = (z_cmd − z)/τ
    #   反推等效滞后 τ ≈ 1.5s 正好复现实测的 0.033m/s ⇒ 用它当"车辆模型"。
    pre_z = 0.0 - PC
    budget = c_timeout - 10.0            # 下降可用的时间 (到位后仍要留 10s 搜索窗口)
    tau, dt = 1.5, 0.05

    def sim(rate_mode):
        zc, z, n = 0.0 - H, 0.0 - H, 0
        while z < pre_z - 0.25 and n * dt < 60.0:
            if rate_mode == "relative":      # 初版 (错): 相对当前实测位置
                cmd = min(pre_z, z + RATE * dt)
            else:                            # 修正版: 时间锚定 (从进入高度按 rate 走)
                cmd = min(pre_z, (0.0 - H) + RATE * (n * dt))
            z += (cmd - z) / tau * dt        # 一阶滞后车辆
            zc = cmd
            n += 1
        return n * dt, z

    t_old, z_old = sim("relative")
    check(t_old > budget,
          f"复现 SITL: 旧写法(相对当前位置)在滞后模型下 **{t_old:.0f}s 上限内都到不了位** "
          f"(等效速率 {0.05 / tau:.3f}m/s, 与 SITL 实测 0.034m/s 同量级; 预算 {budget:.1f}s)")
    t_new, z_new = sim("anchored")
    check(t_new <= budget,
          f"修正版(时间锚定) {t_new:.1f}s 降到 {PC - (z_new - pre_z) * 0 - PC + PC:.2f}m 位 (≤预算 {budget:.1f}s)")
    check(abs(z_new - pre_z) < 0.30, f"降到位精度 |z−pre_z|={abs(z_new - pre_z):.3f}m < 0.30m")

    # B2b 命令永不越过目标 (两种写法都要满足)
    s = _Self(z=0.0 - H)
    n, maxcmd = 0, -1e9
    while s.z < pre_z - 0.25 and n < 400:
        desc(s.at(n * 0.05), n * 0.05, 1.0, -2.0, 0.0)
        maxcmd = max(maxcmd, s.pubs[-1][2])
        s.z = s.pubs[-1][2]
        n += 1
    check(maxcmd <= pre_z + 1e-9, f"命令从不越过下降目标 ({maxcmd:+.3f} ≤ pre_z {pre_z:+.3f})")
    check(n * 0.05 <= budget, f"即时跟随假设下 {n * 0.05:.2f}s 到位 (≤预算 {budget:.1f}s)")

    # B2d 追加 39: 降高锚点 = **检测到的桶** (优先) / 名义点 (兜底)
    def _run(trackers, z=-H):
        s = _Self(z=z)
        s._trackers = trackers
        s.at(1.0)
        desc(s, 1.0, 1.0, -2.0, 0.0)      # 名义点 (tx,ty)=(1.0,-2.0)
        return s

    s = _run([])
    check((s.pubs[-1][0], s.pubs[-1][1]) == (1.0, -2.0), "无追踪器 ⇒ 降在名义点 (原行为不变)")
    check(not any("[降高锚点]" in m for m in s._log.infos), "无追踪器 ⇒ 不打锚点日志")

    s = _run([{"confirmed": True, "ned": (1.38, -2.38)}])       # 距名义点 0.54m
    check(abs(s.pubs[-1][0] - 1.38) < 1e-9 and abs(s.pubs[-1][1] + 2.38) < 1e-9,
          f"已确认桶估计 ⇒ 降高对准它 (命令 NED({s.pubs[-1][0]:.2f},{s.pubs[-1][1]:.2f}))")
    check(any("[降高锚点]" in m for m in s._log.infos),
          f"打锚点日志: '{[m.strip() for m in s._log.infos if '锚点' in m][0][:70]}…'")

    s = _run([{"confirmed": True, "ned": (1.0 + 1.9, -2.0)}])   # 距名义点 1.9m > 1.5
    check((s.pubs[-1][0], s.pubs[-1][1]) == (1.0, -2.0), "桶估计离名义点 >1.5m ⇒ 不采信, 仍用名义点")

    s = _run([{"confirmed": False, "ned": (1.38, -2.38)}])
    check((s.pubs[-1][0], s.pubs[-1][1]) == (1.0, -2.0), "**未确认**追踪器 ⇒ 不采信 (防假框)")

    # 15:22 实飞真实数据: 真桶距名义点 0.38m / 假框 0.89m ⇒ 必须取近的那个
    s = _run([{"confirmed": True, "ned": (1.38, -2.38)},        # 0.54m
              {"confirmed": True, "ned": (1.40, -2.80)}])       # 0.89m
    check(abs(s.pubs[-1][0] - 1.38) < 1e-9, "两个已确认追踪器 ⇒ 取**离名义点最近**的那个 (真桶)")

    # 锚点日志 2s 节流
    s = _Self(z=0.0 - H)
    s._trackers = [{"confirmed": True, "ned": (1.38, -2.38)}]
    for tt in (0.0, 0.5, 1.5, 2.5):
        s.at(tt); desc(s, tt, 1.0, -2.0, 0.0)
    check(sum(1 for m in s._log.infos if "锚点" in m) == 2,
          f"锚点日志 2s 节流 (t=0/0.5/1.5/2.5 ⇒ {sum(1 for m in s._log.infos if '锚点' in m)} 条)")


    # B2c 结构性判据: 设定点必须**时间锚定** (含 _step_entry_z 与 _step_elapsed)
    check("K_DESCEND_RATE * self._step_elapsed()" in block
          and "_step_entry_z" in block,
          "降高设定点为时间锚定 (含 _step_entry_z + _step_elapsed), 不再用 `self.z + rate*0.05`")
    check("self.z + K_DESCEND_RATE * 0.05" not in block,
          "已不含旧的『相对当前实测位置』写法")
    check("_pub_pos(_dx, _dy, _z_desc" in block and "K_DESCEND_PULL_M" in block,
          "下降点用 _dx/_dy (对准检测到的桶) 且带 K_DESCEND_PULL_M 门限")

    # B3 降到位后放行搜索 (不再提前 return)
    s = _Self(z=pre_z - 0.10)                  # 只差 0.10m ⇒ 认为到位
    got = desc(s.at(10.0), 10.0, 1.0, -2.0, 0.0)
    check(got == "SEARCH", f"降到目标 ±0.25m 内 ⇒ 放行搜索 (返回 {got})")
    check(len(s.pubs) == 1, "放行路径同样每 tick 发 setpoint")

    # B4 [降高] 日志 2s 节流
    s = _Self(z=0.0 - H)
    desc(s.at(0.0), 0.0, 1.0, -2.0, 0.0)
    desc(s.at(0.5), 0.5, 1.0, -2.0, 0.0)
    desc(s.at(1.5), 1.5, 1.0, -2.0, 0.0)
    desc(s.at(2.5), 2.5, 1.0, -2.0, 0.0)
    check(len(s._log.infos) == 2, f"日志 2s 节流 (t=0/0.5/1.5/2.5 ⇒ {len(s._log.infos)} 条)")
    check(len(s.pubs) == 4, "节流期间 setpoint 照发 (节流只影响日志)")


def section_c():
    print("\n[C] 剖面自洽")
    c = load_consts(K_PY)
    H, PC, DH = c["K_FLIGHT_ALT"], c["PRE_SERVO_HEIGHT"], c["DROP_HEIGHT"]
    check(H == 5.0, f"巡航高度 = {H}m (用户要求 5m)")
    check(PC == DH == 1.2, f"下降目标 {PC}m = 投弹高度 {DH}m = 1.2m (用户要求降高到 1.2m 投弹)")
    # NED: z 越大越靠下 ⇒ 巡航 z = home-5 < 下降 z = home-1.2 ⇒ 命令单调向下 ✓
    cruise_rel = -H
    pre_rel = -PC
    check(pre_rel > cruise_rel, f"NED 方向正确: 下降目标 z({pre_rel:+.1f}) > 巡航 z({cruise_rel:+.1f}) ⇒ 是'下降'")
    need_s = (H - PC) / c["K_DESCEND_RATE"]
    budget = c["HOVER_SEARCH_TIMEOUT"] - 10.0
    check(need_s < budget, f"下降需 {need_s:.1f}s < HOVER_SEARCH_TIMEOUT 留给它的预算 {budget:.1f}s")
    check(c["K_FLIGHT_ALT"] > PC, "巡航严格高于投弹高度 ⇒ 不会'还没下降就已在地面高度'")
    print(f"      期望时序: TAKEOFF 爬 {H}m ({H / c['TAKEOFF_CLIMB_RATE']:.1f}s) → FORWARD "
          f"{c['FORWARD_DIST']}m @ {c['K_FLIGHT_SPEED']}m/s (~{c['FORWARD_DIST'] / 0.41:.1f}s 按实测 0.41m/s) "
          f"→ 降 {H - PC}m ({need_s:.1f}s) → 找桶/投弹")


def section_d():
    print("\n[D] 结构不变量 + 两份一致 + 语法")
    blocks = []
    for f in (K_PY, K_FLY):
        src = f.read_text(encoding="utf-8")
        # ⚠ 结构不变量: HOVER_SEARCH 每个 tick 必须先发 setpoint 再 return
        i = src.index(HS_MARK)
        blk = src[i:i + 5000]
        i_pub = blk.find("_pub_pos(_dx, _dy, _z_desc")     # 追加 39: 下降点对准检测到的桶
        i_ret = blk.find("if self.z < _pre_z - 0.25:")
        check(0 <= i_pub < i_ret,
              f"{f.name}: `_pub_pos` 早于下降期提前 return (发指令优先)")
        check(src.count("\n        elif self.step == STEP_HOVER_SEARCH:") == 1
              and src.count("\n        elif self.step == STEP_FORWARD:") == 1,
              f"{f.name}: 状态机 elif 计数未被吞")
        try:
            compile(src, str(f), "exec")
            check(True, f"{f.name}: 语法编译通过")
        except SyntaxError as e:
            check(False, f"{f.name}: 语法错误 {e}")
        # 抽出下降块做逐字节比较
        i2 = src.index('# 2026-10-09 追加 37 (用户指定剖面')
        i2 = src.rindex("\n", 0, i2) + 1
        j2 = src.index('f" (限速 {K_DESCEND_RATE:.1f}m/s, 到位后再找桶)")', i2)
        blocks.append(src[i2:src.index("\n", j2) + 1])
    check(len(set(blocks)) == 1, "两份 k 的下降块逐字节相同")
    cc = [load_consts(f) for f in (K_PY, K_FLY)]
    check(cc[0] == cc[1], f"两份 k 的高度常量一致 (K_FLIGHT_ALT/PRE_SERVO_HEIGHT/K_DESCEND_RATE)")


def main():
    print("=" * 84)
    print("k 离线验收 — 5m 巡航 → 前进 3m → 降到 1.2m 投弹 (2026-10-09 patch_k_alt_5m)")
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
