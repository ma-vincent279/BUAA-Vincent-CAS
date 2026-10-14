#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k 离线验收 — 视觉门 vs 到位容差死锁修复 (2026-10-09, patch_k_vis_gate_20261009.py)

背景: 13:59 实飞结束后用户指正 **"你根本就没有飞到桶上面当然没有检测结果了"** ——
      相机是正下视 (`k.py` `camera_pixel_to_level`: "相机光轴=机体下") ⇒ H=1.2m 时
      前向视野只有 ±(IMG_H/2)/FY·1.2 ≈ ±0.67m ⇒ 飞机离桶 1.8m 时桶根本不在画面里。
      顺着这条线查出**上一轮引入的死锁**: 到位判据 `dist2d<POS_ARRIVE_DIST(0.5)` 允许
      飞机停在离靶点 0.3~0.5m 处, 而旧视觉门要求已飞 ≥3.0−0.30=2.70m(离靶点 ≤0.30m)
      ⇒ 该带内门永不满足 ⇒ 选不了桶 ⇒ 10s 超时返航。

四节
----
A 最小改动审计: 剥注释后与 `*.bak.*_vis_gate` 对比, 新增行全在允许集合, 删除行只能是
  被替换的那 3 行 (含常量与门控表达式)。
B **无死锁证明**: 用文件里的真实常量在 (沿航向位移, 横向偏移) 网格上穷举 —— 凡
  `dist2d < POS_ARRIVE_DIST` 的样本, 门控**必须**放行; 并给出反事实: 旧值 0.30 会挡掉一部分。
C 几何自洽: `POS_ARRIVE_DIST < K_VISUAL_GATE_M < (IMG_H/2)/FY·K_FLIGHT_ALT`
  (实机副本; SITL 是合成检测器不受视野限制, 只做信息性打印)。
D **真代码路径 exec**: 从 k.py 抽出真实门控块执行, 核对边界 (2.39 拦 / 2.41 放)、
  旧死锁带内的 2.55 现在放行、贴边日志 2s 节流。

用法: python3 scripts/test_k_vis_gate_20261009.py
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
L_PY = REPO / "src/drone_drl/drone_drl/l.py"
BAK_GLOB = "*_vis_gate"

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


WANT = {"K_VISUAL_GATE_M", "FORWARD_DIST", "POS_ARRIVE_DIST", "K_FLIGHT_ALT",
        "IMG_W", "IMG_H", "HFOV", "VFOV", "FX", "FY", "DROP_HEIGHT", "PRE_SERVO_HEIGHT"}


def load_consts(path):
    """exec 该文件里所有目标常量的纯赋值 (依赖 math), 得到真实数值"""
    src = path.read_text(encoding="utf-8")
    ns = {"math": math}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in WANT for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    return {k: ns[k] for k in WANT if k in ns}


# ── A 最小改动审计 ────────────────────────────────────────────────────────
ADD_ALLOW_EXACT = (
    "K_VISUAL_GATE_M = 0.60  # 2026-10-09 实机反馈③(+ 同日死锁修复): 沿航向已飞距离",
    "_vg_need = max(K_VISUAL_GATE_M, POS_ARRIVE_DIST)",
    "if _travel_fwd < FORWARD_DIST - _vg_need:",
)
ADD_ALLOW_RE = (
    r"^#\s*< FORWARD_DIST-该值",
    r"^#\s*前方 3m 处才启动视觉",
    r"^#\s*下界 POS_ARRIVE_DIST",
    r"^#\s*0\.3~0\.5m 时门永不满足",
    r"^#\s*上界 \(IMG_H/2\)/FY·K_FLIGHT_ALT",
    r"^#\s*1\.2m 的前向视野半径",
    r"^#\s*0\.60 ∈ \(0\.5, 0\.673\)",
    r"^f\"  \[视觉门\] 已飞 \{_travel_fwd:\.2f\}m < \{FORWARD_DIST - _vg_need:\.2f\}m\"$",
)
DEL_ALLOW = (
    "K_VISUAL_GATE_M = 0.30  # 2026-10-09 实机反馈③: 沿航向已飞距离 < FORWARD_DIST-该值 时",
    "**禁止选桶/进伺服** ⇒ 必须先真正飞到前方 3m 处才启动视觉",
    "if _travel_fwd < FORWARD_DIST - K_VISUAL_GATE_M:",
    'f"  [视觉门] 已飞 {_travel_fwd:.2f}m < {FORWARD_DIST - K_VISUAL_GATE_M:.2f}m"',
)


# 2026-10-09 后记: 本测试的 A 节是"当前文件 vs 本补丁前备份"的差集审计 ⇒ 之后**任何**
# 就地补丁都会出现在差集里, 必须在此登记 (同 test_ekf_tlm 的做法):
#   patch_k_stall_guard_20261009.py —— 前飞无进展保护 + landed 记录,
#   它自己的验收是 scripts/test_k_stall_guard_20261009.py (32/32)。
# 2026-10-09: 后续就地补丁的标记集中登记 (见 scripts/_later_patch_marks.py)
try:
    from _later_patch_marks import later_than, \
        LATER_PATCH_BAK_SUFFIXES as _SHARED_BAK
except Exception:      # 单独拷出去跑时退化为空表
    _SHARED, _SHARED_BAK = (), ()

# ⚠ 本脚本审计 patch_k_vis_gate ⇒ 只归因**晚于它**的补丁
_LATER = later_than("vis_gate")
LATER_PATCH_MARKERS = _LATER + (
    "K_FWD_STALL", "_fwd_stall_logged", "forward_stall", "无进展",
    # 该补丁还给 FC 串追加了 landed 标志 (动了本补丁写的那两行)
    "L{1 if self.landed")



def section_a():
    print("\n[A] 最小改动审计 (只动视觉门相关 3 处)")
    for f in (K_PY, K_FLY):
        bak = latest_bak(f)
        if not check(bak is not None, f"{f.name}: 找到补丁前备份 ({BAK_GLOB})"):
            continue
        a, b = strip_comments(bak), strip_comments(f)
        sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
        added, removed, bad, later = [], [], [], 0
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag == "equal":
                continue
            # **块级**后续补丁归因: 多行 f-string / 续行不带关键字, 逐行匹配会漏
            if any(mk in "\n".join(b[j1:j2] + a[i1:i2]) for mk in LATER_PATCH_MARKERS):
                later += len(b[j1:j2]) + len(a[i1:i2])
                continue
            if tag in ("insert", "replace"):
                for ln in b[j1:j2]:
                    added.append(ln)
                    s_ = ln.strip()
                    if not (any(s_ == p for p in ADD_ALLOW_EXACT)
                            or any(re.match(p, s_) for p in ADD_ALLOW_RE)):
                        bad.append("+" + ln)
            if tag in ("delete", "replace"):
                for ln in a[i1:i2]:
                    removed.append(ln)
                    if ln.strip() not in DEL_ALLOW:
                        bad.append("-" + ln)
        check(not bad, f"{f.name}: {len(added)} 新增 / {len(removed)} 删除 全在允许集合内"
                       + ("" if not bad else f" (越界 {bad[:2]})"))
        check(len(removed) <= 4, f"{f.name}: 删除 {len(removed)} 行 (≤4, 只替换门控那几行)")
        if later:
            check(True, f"{f.name}: {later} 行归因于后续补丁 (前飞无进展保护), 由 test_k_stall_guard 覆盖")
        # 控制面: 只允许出现 _vg_need / 常量, 不许出现新的发布/状态机写入
        forbid = ("_pub_", "publish", "self.step =", "_enter_step", "create_publisher",
                  "self._trackers =", "self.hold")
        body = "\n".join(added)
        hit = [k for k in forbid if k in body]
        check(not hit, f"{f.name}: 新增行里无发布/状态机写入 (禁用词 {len(forbid)}, 命中 {hit})")
    check("K_VISUAL_GATE_M" not in L_PY.read_text(encoding="utf-8"),
          "l.py 本来就没有该门 ⇒ 本次改动只影响 k (不必动 l)")


# ── B 无死锁证明 ──────────────────────────────────────────────────────────
def section_b():
    print("\n[B] 无死锁证明 — \"FORWARD 到位 ⇒ 视觉门必过\"")
    c = load_consts(K_PY)
    FD, PA, G = c["FORWARD_DIST"], c["POS_ARRIVE_DIST"], c["K_VISUAL_GATE_M"]
    need = FD - max(G, PA)
    print(f"      FORWARD_DIST={FD} POS_ARRIVE_DIST={PA} K_VISUAL_GATE_M={G}"
          f" ⇒ 门控要求已飞 ≥ {need:.2f}m")

    # 网格穷举: a=沿航向位移, l=横向偏移
    worst, n_ok, n_tot = None, 0, 0
    for ia in range(0, 601):
        a_ = ia / 100.0                        # 0.00 ~ 6.00 m
        for il in range(-200, 201):
            l_ = il / 100.0                    # ±2.00 m
            dist2d = math.hypot(FD - a_, l_)
            if dist2d < PA:                    # FORWARD 会判定"到位"
                n_tot += 1
                if a_ >= need:
                    n_ok += 1
                else:
                    worst = (a_, l_, dist2d)
    check(n_tot > 0 and worst is None,
          f"到位样本 {n_tot} 个全部满足门控 (最坏 {worst})")
    # 解析证明: 到位 ⇒ |FD-a|<PA ⇒ a>FD-PA ≥ FD-max(G,PA) ✓
    check(need <= FD - PA + 1e-12,
          f"解析: 门限 {need:.2f} ≤ FD−PA = {FD - PA:.2f} ⇒ 到位必过 ✓")

    # 反事实: 旧值 0.30 会挡掉死锁带内的一部分
    old_need = FD - 0.30
    dead = [(i / 100.0, ) for i in range(0, 601)
            if math.hypot(FD - i / 100.0, 0.0) < PA and i / 100.0 < old_need]
    band = [x for x in dead if x[0] >= FD - PA]
    check(len(band) > 0,
          f"反事实: 旧门限 2.70m 会挡住 {len(band)} 个\"已到位但已飞 <2.70m\"的合法样本"
          f" (带内 {band[0][0]:.2f}~{band[-1][0]:.2f}m) ⇒ 修复确有意义")
    return c


# ── C 几何自洽 ────────────────────────────────────────────────────────────
def section_c():
    print("\n[C] 几何自洽 — 门限必须落在 (到位容差, 正下视前向视野) 之间")
    for tag, f in (("fly (实机正下视相机)", K_FLY), ("SITL (合成检测器)", K_PY)):
        c = load_consts(f)
        G, PA = c["K_VISUAL_GATE_M"], c["POS_ARRIVE_DIST"]
        # ⚠ 可见性判据必须用**选桶时的高度** = 投弹高度 (2026-10-09 起: 先降到 1.2m 再找桶;
        #   巡航 5m 时桶只 16px 根本选不出来) ⇒ 用 DROP_HEIGHT 算前向视野半径才有意义。
        H = c["PRE_SERVO_HEIGHT"]
        r_fwd = (c["IMG_H"] / 2.0) / c["FY"] * H
        r_lat = (c["IMG_W"] / 2.0) / c["FX"] * H
        print(f"      {tag}: FY={c['FY']:.1f} ⇒ H={H}m 地面覆盖 前向 ±{r_fwd:.3f}m / 横向 ±{r_lat:.3f}m")
        check(G > PA, f"{tag}: K_VISUAL_GATE_M({G}) > POS_ARRIVE_DIST({PA}) ⇒ 不死锁")
        if "fly" in str(f.relative_to(REPO)):
            check(G < r_fwd, f"{tag}: K_VISUAL_GATE_M({G}) < 前向视野半径({r_fwd:.3f}) "
                             f"⇒ 放行时桶必在画面内 (余量 {r_fwd - G:.3f}m)")
        else:
            print(f"      · SITL 用 GT 投影检测器, 无\"视野\"约束, 仅记录 {G} vs {r_fwd:.3f}")
    # 桶在正下方时的像素尺寸 (证明"到了就一定看得见")
    c = load_consts(K_FLY)
    # ⚠ 用**投弹高度**(DROP_HEIGHT=1.2m)算, 不是巡航高度
    #   (2026-10-09 起 K_FLIGHT_ALT=5m 只是巡航; 找桶/投弹在 1.2m)
    _h = c["DROP_HEIGHT"]
    px = 0.25 * c["FX"] / _h
    frac = px / c["IMG_H"]
    check(px > 40, f"投弹高度 {_h}m 正下方桶径 0.25m ≈ {px:.0f}px ({frac * 100:.0f}% 画高) ⇒ 足够大")
    _pxc = 0.25 * c["FX"] / c["K_FLIGHT_ALT"]
    print(f"      · 参考: 巡航 {c['K_FLIGHT_ALT']}m 时桶 ≈ {_pxc:.0f}px (5m 时确实小, "
          f"故先降到 {c['PRE_SERVO_HEIGHT']}m 再找桶)")


# ── D 真代码路径 exec ─────────────────────────────────────────────────────
class _Log:
    def __init__(self): self.infos = []

    def info(self, m): self.infos.append(m)


class _Self:
    def __init__(self, x, y, h0=0.0):
        self.x, self.y = x, y
        self.home_x, self.home_y = 0.0, 0.0
        self._heading0 = h0
        self._vg_log_t = -99.0
        self._log = _Log()

    def get_logger(self): return self._log


def _extract_gate():
    src = K_PY.read_text(encoding="utf-8")
    i = src.index("# 2026-10-09 反馈③: 视觉门")
    i = src.rindex("\n", 0, i) + 1
    j = src.index("confirmed = [t for t in self._trackers", i)
    j = src.rindex("\n", 0, j) + 1
    return textwrap.dedent(src[i:j])


def section_d():
    print("\n[D] 真代码路径 exec (从 k.py 抽出的实际门控块)")
    block = _extract_gate()
    check("_vg_need = max(" in block, "抽出的块里含结构性下界 _vg_need = max(...)")
    body = textwrap.indent(block, "    ")
    body = body.replace("\n        return\n", "\n        return 'BLOCK'\n")   # 门控内层 return
    code = "def _gate(self, sim_t):\n" + body + "    return 'PASS'\n"
    ns = {"math": math, "getattr": getattr}
    ns.update({k: v for k, v in load_consts(K_PY).items()
               if k in ("FORWARD_DIST", "POS_ARRIVE_DIST", "K_VISUAL_GATE_M")})
    exec(code, ns)
    gate = ns["_gate"]

    FD, G = ns["FORWARD_DIST"], ns["K_VISUAL_GATE_M"]
    cases = [
        (FD - G - 0.01, "BLOCK", f"已飞 {FD - G - 0.01:.2f}m < 门限 {FD - G:.2f}m ⇒ 拦"),
        (FD - G + 0.01, "PASS", f"已飞 {FD - G + 0.01:.2f}m ≥ 门限 {FD - G:.2f}m ⇒ 放行"),
        (FD - 0.45, "PASS", "已飞 2.55m (旧值 2.70m 的死锁带内, 现在必须放行)"),
        (FD - 0.50, "PASS", "已飞 2.50m (恰在到位边界) ⇒ 放行"),
        (FD - 0.30, "PASS", "已飞 2.70m (旧门限的临界点) ⇒ 放行"),
        (1.20, "BLOCK", "已飞 1.20m (13:59 实飞的真实中止点) ⇒ 拦 (此时桶在 1.8m 外, 本来也看不见)"),
    ]
    for travel, expect, desc in cases:
        s = _Self(travel, 0.0)
        got = gate(s, sim_t=20.0)
        check(got == expect, f"{desc} ⇒ {got}")
    # 日志节流 2s
    s = _Self(1.0, 0.0)
    gate(s, 20.0); n1 = len(s._log.infos)
    gate(s, 20.5); n2 = len(s._log.infos)
    gate(s, 21.5); n3 = len(s._log.infos)
    gate(s, 22.5); n4 = len(s._log.infos)
    check(n1 == 1 and n2 == 1 and n3 == 1 and n4 == 2,
          f"贴边日志 2s 节流 (t=20.0/20.5/21.5/22.5 ⇒ {n1}/{n2}/{n3}/{n4} 条)")
    # 横向偏移不应影响门控 (门只看沿航向位移)
    s = _Self(FD - 0.45, 0.40)
    check(gate(s, 30.0) == "PASS", "横向偏 0.40m 且沿航向到位 ⇒ 仍放行 (门只看沿航向位移)")


def main():
    print("=" * 84)
    print("k 离线验收 — 视觉门/到位容差死锁修复 (2026-10-09 patch_k_vis_gate_20261009.py)")
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
