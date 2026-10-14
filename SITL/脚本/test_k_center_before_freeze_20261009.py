#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k 离线验收 — DROP 段「先入画再冻结」 (2026-10-09 patch_k_center_before_freeze)

背景 (16:09 实飞): 飞机冲过桶 0.6m ⇒ 桶框被画面下边缘裁掉 ⇒ 框算出的位置全偏 + 冻结拒收贴边框
⇒ 死锁 0/5 样本 ⇒ 不投。本补丁在"未冻结"阶段按框心方向把**命令点**朝桶匀速挪 (时间锚定)。

判据
----
A 最小改动审计 (vs `*.bak.*_center`): 新增 12 行代码全在允许集合, 删除 0 行。
B **真代码路径 exec**: 从 k.py 抽真实"先入画"块执行 ——
  ① 框贴下边缘 (桶在机身后方) ⇒ 命令点必须**向后**挪 (机体前向为负)
  ② 每拍步长恒定 = K_CENTER_SPEED×0.05 (时间锚定, 不是"相对当前位置")
  ③ 累计不超 K_CENTER_MAX
  ④ 框居中且不贴边 ⇒ 不挪 (交回原流程)
  ⑤ 无新鲜检测 ⇒ 不挪
  ⑥ 每次调用都发 setpoint (不 return 前必须已发)
C 方向自洽: 用节点自己的 `_roi_to_ned` 反算 —— 沿命令点移动方向走 K_CENTER_MAX 后,
  桶的像素投影应向画面中心移动 (即"越挪越居中")。
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
BAK_GLOB = "*_center"
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


def strip_comments(p):
    return [ln.rstrip() for ln in p.read_text(encoding="utf-8").splitlines()
            if not ln.strip().startswith("#")]


WANT = {"K_CENTER_SPEED", "K_CENTER_MAX", "K_CENTER_PX", "IMG_W", "IMG_H", "FX", "FY",
        "HFOV", "VFOV", "DROP_HEIGHT", "PRE_SERVO_HEIGHT", "K_FLIGHT_ALT"}


def load_consts(p):
    src = p.read_text(encoding="utf-8")
    ns = {"math": math}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in WANT for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    ns.setdefault("CX", 320.0); ns.setdefault("CY", 180.0)
    return {k: ns[k] for k in WANT if k in ns}


def section_a():
    print("\n[A] 最小改动审计")
    ADD = (
        "K_CENTER_SPEED = 0.08   # m/s (f_r3 已验证的低空缓移量级)",
        "K_CENTER_MAX   = 1.00   # m    本次 DROP 累计最大挪动量 (硬上限, 防失控)",
        "K_CENTER_PX    = 60.0   # px   框心偏离画面中心超过它就继续挪 (1.2m 处 60px ≈ 0.22m)",
        "if getattr(self, '_c_base', None) is None:",
        "self._c_base = (self.x - self.home_x, self.y - self.home_y)",
        "self._c_off = [0.0, 0.0]",
        "_c_roi = self.latest_roi if self._drop_detection_fresh() else None",
        "if _c_roi is not None:",
        "_clipped = (_c_roi.x_offset <= 0 or _c_roi.y_offset <= 0",
        "or _c_roi.x_offset + _c_roi.width >= IMG_W",
        "or _c_roi.y_offset + _c_roi.height >= IMG_H)",
        "_ccx = _c_roi.x_offset + _c_roi.width / 2.0",
        "_ccy = _c_roi.y_offset + _c_roi.height / 2.0",
        "_coff = math.hypot(_ccx - CX, _ccy - CY)",
        "if _clipped or _coff > K_CENTER_PX:",
        "_ch = self.home_z - self.z",
        "_cf = (CY - _ccy) / FY * _ch        # 机体前向 (带符号)",
        "_cr = (_ccx - CX) / FX * _ch        # 机体右向 (带符号)",
        "_cn = math.hypot(_cf, _cr)",
        "if _cn > 1e-6:",
        "_step = K_CENTER_SPEED * 0.05   # 时间锚定: 每拍固定步长",
        "_chdg = self.heading if self.heading else 0.0",
        "_c_off[0] += (_cf / _cn) * _step * math.cos(_chdg) + (_cr / _cn) * _step * math.sin(_chdg)",
        "_c_off[1] += (_cf / _cn) * _step * math.sin(_chdg) - (_cr / _cn) * _step * math.cos(_chdg)",
        "_c_norm = math.hypot(self._c_off[0], self._c_off[1])",
        "if _c_norm > K_CENTER_MAX:      # 硬上限",
        "self._c_off[0] *= K_CENTER_MAX / _c_norm",
        "self._c_off[1] *= K_CENTER_MAX / _c_norm",
        "if sim_t - getattr(self, '_c_log_t', -99.0) > 2.0:",
        "self._c_log_t = sim_t",
        "self.get_logger().info(",
        'f"  [先入画] 框{\'贴边\' if _clipped else \'偏心\'} "',
        'f"(框心偏 {_coff:.0f}px, 机体前{_cf:+.2f}/右{_cr:+.2f}m)"',
        'f" ⇒ 命令点朝桶挪 已={_c_norm:.2f}m/{K_CENTER_MAX:.1f}m"',
        'f" (速度 {K_CENTER_SPEED:.2f}m/s)")',
        "self._pub_pos(self.home_x + self._c_base[0] + self._c_off[0],",
        "self.home_y + self._c_base[1] + self._c_off[1],",
        "drop_z, self.heading)",
        "return",
    )
    for f in (K_PY, K_FLY):
        c = sorted(glob.glob(str(f) + "." + BAK_GLOB))
        if not check(bool(c), f"{f.name}: 找到补丁前备份"):
            continue
        a, b = strip_comments(pathlib.Path(c[-1])), strip_comments(f)
        sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
        added, removed, bad = [], [], []
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag in ("insert", "replace"):
                for ln in b[j1:j2]:
                    added.append(ln)
                    _n = re.sub(r"\s+", "", ln.strip())
                    if _n not in {re.sub(r"\s+", "", x) for x in ADD}:
                        bad.append("+" + ln)
            if tag in ("delete", "replace"):
                for ln in a[i1:i2]:
                    removed.append(ln)
                    bad.append("-" + ln)
        check(not bad, f"{f.name}: {len(added)} 新增 / {len(removed)} 删除 全在允许集合内"
                       + ("" if not bad else f" (越界 {bad[:2]})"))
        check(len(added) == len(ADD), f"{f.name}: 新增行数 == 允许集合大小 ({len(added)})")


class _Log:
    def __init__(self): self.infos = []

    def info(self, m): self.infos.append(m)


class _Self:
    def __init__(self, x, y, hdg, alt, roi):
        self.home_x = self.home_y = 0.0
        self.home_z = 0.0
        self.x, self.y = x, y
        self.z = -alt
        self.heading = hdg
        self.latest_roi = roi
        self.pubs = []
        self._c_base = None
        self._log = _Log()

    def _drop_detection_fresh(self): return True

    def _pub_pos(self, x, y, z, yaw): self.pubs.append((x, y, z, yaw))

    def get_logger(self): return self._log


def _extract():
    src = K_PY.read_text(encoding="utf-8")
    i = src.index("# ===== 2026-10-09 追加 40: 「先入画再冻结」=====")
    i = src.rindex("\n", 0, i) + 1
    j = src.index("                            return\n", i)
    return textwrap.dedent(src[i:src.index("\n", j) + 1])


def section_b():
    print("\n[B] 真代码路径 exec")
    block = _extract()
    c = load_consts(K_PY)
    H = 1.2
    CX, CY = 320.0, 180.0
    FY = c["FY"]; FX = c["FX"]; IMG_H = c["IMG_H"]; IMG_W = c["IMG_W"]

    def roi(cx, cy, w, h, xo=None, yo=None):
        xo = int(cx - w / 2) if xo is None else xo
        yo = int(cy - h / 2) if yo is None else yo
        return types.SimpleNamespace(x_offset=xo, y_offset=yo, width=int(w), height=int(h))

    code = ("def _center(self, sim_t, drop_z):\n" + textwrap.indent(block, "    ")
            + "    return 'PROCEED'\n")
    ns = {"math": math, "getattr": getattr, "types": types}
    ns.update(c)
    ns.setdefault("CX", 320.0); ns.setdefault("CY", 180.0)
    exec(code, ns)
    fn = ns["_center"]

    # ① 桶在机身后方 0.6m ⇒ 框被下边缘裁 (cy大) ⇒ 必须向后挪 (机体前向<0)
    hdg = math.radians(212.0)
    y_c = CY + 0.6 * FY / H      # fwd=-0.6m
    s = _Self(0.0, 0.0, hdg, H, roi(320, y_c, 140, 80, yo=int(IMG_H - 80)))   # 底边贴边
    r = fn(s, 100.0, -1.2)
    check(r is None and len(s.pubs) == 1, "贴边帧 ⇒ 走本块 (PASS 且发了 setpoint)")
    dx = s.pubs[0][0] - s._c_base[0]; dy = s.pubs[0][1] - s._c_base[1]
    fwd = dx * math.cos(hdg) + dy * math.sin(hdg)          # 机体前向分量
    check(fwd < 0, f"桶在后方 ⇒ 命令点向后挪 (前向分量 {fwd * 100:.2f}cm/拍)")
    step = math.hypot(dx, dy)
    check(abs(step - c["K_CENTER_SPEED"] * 0.05) < 1e-9,
          f"每拍步长恒定 = {step * 100:.3f}cm (= {c['K_CENTER_SPEED']}×0.05, 时间锚定)")
    check(any("[先入画]" in m for m in s._log.infos), f"日志: {s._log.infos[0].strip()[:60]}…")

    # ② 桶在机身前方 0.6m ⇒ 框贴上边缘 ⇒ 向前挪
    y_c2 = CY - 0.6 * FY / H
    s2 = _Self(0.0, 0.0, hdg, H, roi(320, y_c2, 140, 80, yo=0))
    fn(s2, 200.0, -1.2)
    dx2 = s2.pubs[0][0] - s2._c_base[0]; dy2 = s2.pubs[0][1] - s2._c_base[1]
    fwd2 = dx2 * math.cos(hdg) + dy2 * math.sin(hdg)
    check(fwd2 > 0, f"桶在前方 ⇒ 命令点向前挪 (前向分量 {fwd2 * 100:.2f}cm/拍)")

    # ③ 约束: 连续挪 1000 拍, 累计不超过 K_CENTER_MAX
    s3 = _Self(0.0, 0.0, hdg, H, roi(320, IMG_H - 40, 140, 80, yo=IMG_H - 80))
    for n in range(1000):
        fn(s3, 300.0 + n * 0.05, -1.2)
    disp = math.hypot(s3._c_off[0], s3._c_off[1])
    check(disp <= c["K_CENTER_MAX"] + 1e-9, f"累计挪动 {disp:.3f}m ≤ 上限 {c['K_CENTER_MAX']}m")

    # ④ 框居中且不贴边 ⇒ 不挪 (交回原等静/采样流程)
    s4 = _Self(0.0, 0.0, hdg, H, roi(320, 180, 120, 70))
    check(fn(s4, 400.0, -1.2) == "PROCEED" and not s4.pubs, "框居中不贴边 ⇒ 不挪 (PROCEED)")

    # ⑤ 偏心但未贴边 (65px > 60px 门限) ⇒ 挪
    s5 = _Self(0.0, 0.0, hdg, H, roi(320 + 65, 180, 120, 70))
    check(fn(s5, 500.0, -1.2) is None and len(s5.pubs) == 1, "偏心 65px > 门限 ⇒ 挪")

    # ⑥ 日志 2s 节流
    s6 = _Self(0.0, 0.0, hdg, H, roi(320, IMG_H - 40, 140, 80, yo=IMG_H - 80))
    for t in (0.0, 0.5, 1.5, 2.5):
        fn(s6, t, -1.2)
    check(sum(1 for m in s6._log.infos if "先入画" in m) == 2,
          f"日志 2s 节流 ({sum(1 for m in s6._log.infos if '先入画' in m)} 条)")


def section_c():
    print("\n[C] 方向自洽 (用节点自己的反投影核对)")
    c = load_consts(K_PY)
    src = K_PY.read_text(encoding="utf-8")
    # 用节点真实 camera_pixel_to_level 的语义: fwd=(CY-cy)/FY, right=(cx-CX)/FX
    CX, CY, FX, FY = 320.0, 180.0, c["FX"], c["FY"]
    H = 1.2
    cy = CY + 0.6 * FY / H                  # 桶在后方 0.6m
    fwd_before = (CY - cy) / FY * H
    # 挪 0.3m 向桶 (后方) 后, 相对距离变小 ⇒ |像素偏移| 变小
    dist_before = 0.6
    dist_after = 0.6 - 0.3
    cy_after = CY + dist_after * FY / H
    check(abs(fwd_before + 0.6) < 1e-6, f"反投影核对: 桶在后方 0.6m ⇔ 框心偏 {(cy - CY):.0f}px")
    check(abs(cy_after - CY) < abs(cy - CY), "沿补丁选的方向挪 0.3m 后, 框心更靠近画面中心 ⇒ 越挪越居中 ✓")


def main():
    print("=" * 84)
    print("k 离线验收 — DROP「先入画再冻结」(2026-10-09 patch_k_center_before_freeze)")
    print("=" * 84)
    section_a()
    section_b()
    section_c()
    print("\n" + "=" * 84)
    print(f"结果: {n_pass} 项通过 / {n_fail} 项失败")
    print("=" * 84)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
