#!/usr/bin/env python3
"""可搬运性检查器 — 小场地验证台 k ↔ 全任务节点 l (2026-10-09)

问题
----
"在小场地(k)上标定/验证得到的投放参数, 搬到全任务(l)上还成立吗?"
不能靠"相似度高"回答, 必须机器核对下面三类东西:

  A 几何链 (必须【逐字节】相同) —— 决定标定出来的 TARGET_PLANE_H / Y / XF 能否照搬
      相机模型: camera_pixel_to_level / level_to_camera_pixel
      反投影  : _roi_to_ned / _ned_to_expected_pixel / _body_to_ned / _ned_to_body
      冻结公式: hold = 桶估计 + 两瓶补偿  (_i_hold_x / _i_hold_y 的赋值语句)
  B 投放常量 (必须数值相同) —— 两者不一致 ⇒ 一边的结论不能代表另一边
  C 差异清单 (允许不同, 但必须【显式列出并判定是否影响落点】)

用法:
  python3 scripts/check_drop_portability_20261009.py k l        # 默认 (k=小场地, l=全任务)
  python3 scripts/check_drop_portability_20261009.py k l --fly  # 同时核对 fly 侧
退出码: 0 = 可搬运; 1 = 存在阻断项
"""
import argparse
import ast
import inspect
import math
import re
import sys
import textwrap
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OK = "\u2713"
NG = "\u2717"
DOT = "\u2022"

# ---- A: 必须逐字节相同的东西 ----
MUST_MATCH_FUNCS = ["camera_pixel_to_level", "level_to_camera_pixel",
                    "_roi_to_ned", "_ned_to_expected_pixel",
                    "_body_to_ned", "_ned_to_body"]
MUST_MATCH_LINES = ["self._i_hold_x = (", "self._i_hold_y = ("]

# ---- B: 必须数值相同的常量 ----
MUST_MATCH_CONSTS = [
    "DROP_HEIGHT", "TARGET_PLANE_H",
    "PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_Y_B2",
    "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2",
    "I_ARRIVE_M", "I_VMAX", "I_PRED_M", "I_PRED_TEFF", "I_DEADLINE_WIN",
    "I_CALM_TILT", "I_CALM_VXY", "I_CALM_SAMPLES", "I_CALM_TIMEOUT", "I_CALM_MIN_FALLBACK",
    "I_FREEZE_TRACKER_M", "I_FREEZE_MAX_SPREAD", "I_FREEZE_EDGE_MARGIN", "I_FREEZE_AT_H",
    "I_SETTLE_VXY", "I_SETTLE_HOLD", "I_SETTLE_MAX", "I_SETTLE_HARD_VXY", "I_SETTLE_HARD_MAX",
    "AUX5_LOCK", "AUX5_DROP", "AUX5_DROP2", "AUX5_DROP_HOLD",
    "L_BODY_GATE_M", "L_BODY_GATE_LOG", "L_BODY_REFREEZE_MAX",
    "IMG_W", "IMG_H", "CX", "CY", "FX", "FY",
    "SERVO_DESCENT_SPEED",
]

# ---- C: 已知且允许的差异 + 是否影响落点几何 ----
KNOWN_DIFF = {
    "FORWARD_DIST": ("不影响", "小场地 3m vs 全任务 34m; 冻结发生在 1.2m 处, 转场距离不进入落点几何"),
    "I_RELEASE_TIMEOUT": ("**影响**", "小场地 45s vs 全任务 20s ⇒ '要到 45s 才收敛'的那种趟在小场地算命中、搬过去会走兜底。"
                                     "验证时建议把验证台也设成 20s, 或记录每次实际用时"),
    "I_DWELL_S": ("不影响", "l 用它(5s); 验证台用 K_DWELL_S1/S2 覆盖"),
    "K_DWELL_S1": ("不影响", "仅验证台有, 第 1 投停留"),
    "K_DWELL_S2": ("不影响", "仅验证台有, 第 2 投停留 30s = 量参照漂移的设计, 不改变投放几何"),
    "SCOUT_DIST": ("不影响", "验证台不去侦察区"),
    "SCOUT_EDGE_DIST": ("不影响", "同上"),
    "MISSION_TIMEOUT": ("不影响", "任务总时长"),
    "STEP_TIMEOUT": ("不影响", "单状态超时"),
    "J_DEV_GATE_M": ("**影响**", "只在验证台(j 系)里存在; =0 时等价于 l"),
    "J_DEV_REFREEZE_M": ("**影响**", "验证台的重新冻结杠杆; =0 时等价于 l。**若要用它, l 必须同时开**"),
    "J_DEV_REFREEZE_MAX": ("**影响**", "同上"),
    "J_DEV_DEADLINE_PICK": ("**影响**", "兜底窗口挑时机; l 恒用 pred ⇒ 若验证台开了它, 两边兜底行为不同"),
    "J_LOG_DEV": ("不影响", "纯日志"),
    "PRE_SERVO_HEIGHT": ("**影响**", "2026-10-09 (反馈④): 验证台改为**全程平飞** ⇒ 它的预伺服高度 = K_FLIGHT_ALT(1.2m), "
                                     "取消了'5m→2.5m→1.2m'的中间档; 全任务仍 2.5m。接近段的**高度曲线**不同, "
                                     "但落点几何 (1.2m 冻结 + 两瓶补偿) 逐字相同"),
    "K_FLIGHT_ALT": ("**影响**", "2026-10-09 (反馈①④): **仅验证台**的飞行高度 (1.2m 全程平飞)。它不进落点公式, "
                                 "但决定**桶在画面里多大** (1.2m≈90px vs 5m≈18px) ⇒ 直接决定'能不能检测到/确认'。"
                                 "首飞就是死在检测断续 (5m 高时 YOLO 几乎每帧 nbox=0)"),
    "K_FLIGHT_SPEED": ("**影响**", "2026-10-09 (反馈②): **仅验证台**的前飞/返航限速 (0.6m/s)。决定到 3m 处的过冲量与停稳质量, "
                                   "间接影响冻结前的站位; 不进落点公式"),
    "K_VISUAL_GATE_M": ("不影响", "2026-10-09 (反馈③): **仅验证台**的视觉门 —— 沿航向已飞距离 < FORWARD_DIST-0.3m 时禁止选桶。"
                                  "只推迟'允许选桶'的时机 (本来就在到位之后), 不改变落点公式"),
    "POSITION_REFINE_IDENTITY_MAX": ("不影响", "验证台单桶, 不走多桶身份"),
    "SERVED_EXCLUDE_M": ("不影响", "验证台单桶, 目标不排除"),
    "DET_DUP_MERGE_DIST": ("不影响", "验证台单桶"),
    "SERVED_DUP_M": ("不影响", "验证台单桶, 目标复制成两投同一个桶"),
    "MISSION_REVISION": ("不影响", "版本字符串, 仅用于日志溯源"),
    "J_MEASURED_R": ("**影响**", "j 的'一键换实测补偿'开关; =False 时两边补偿一致"),
    "L_BODY_GATE_LOG": ("不影响", "纯日志"),
    "K_DWELL_S1_MAX": ("不影响", "验证台停留上限"),
}


def norm_func_src(src, fn):
    """函数原文 → 去注释/去空行/去缩进, 用于逐字节比较。"""
    body = textwrap.dedent(ast.get_source_segment(src, fn))
    out = []
    for ln in body.splitlines():
        t = ln.strip()
        if not t or t.startswith("#"):
            continue
        out.append(ln.rstrip())
    return "\n".join(out)


def collect(path):
    src = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(src)
    funcs = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name not in funcs:
            funcs[node.name] = norm_func_src(src, node)
    consts = {}
    order = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = []
            for t in node.targets:
                if isinstance(t, ast.Name):
                    names.append(t.id)
                elif isinstance(t, ast.Tuple):
                    names.extend(e.id for e in t.elts if isinstance(e, ast.Name))
            if not names:
                continue
            try:
                val = ast.literal_eval(node.value)
            except Exception:
                continue
            if len(names) == 1:
                consts[names[0]] = val
                order.append(names[0])
            elif isinstance(val, tuple) and len(val) == len(names):
                for nm, v in zip(names, val):
                    consts[nm] = v
                    order.append(nm)
    lines = [ln.rstrip() for ln in src.splitlines()]
    holds = [ln.strip() for ln in lines if any(k in ln for k in MUST_MATCH_LINES)]
    return {"src": src, "funcs": funcs, "consts": consts, "order": order, "holds": holds}


# ============================================================
# A2: SITL 链路往返精确性 (2026-10-09 新增)
# ============================================================
DET_PATH = ROOT / "src/drone_drl/drone_drl/yolov8_bucket_detector.py"


class _Roi:
    """鸭子类型的 ROI (只需 x_offset/y_offset/width/height, 避免依赖 ROS 消息类型)。"""
    def __init__(self, x, y, w, h):
        self.x_offset, self.y_offset, self.width, self.height = int(x), int(y), int(w), int(h)


class _FakeDet:
    """给 exec 出来的检测器方法一个假 self。"""
    def __init__(self, fx, fy):
        self.img_w, self.img_h = 640, 360
        self.fx, self.fy = fx, fy
        self.heading = 0.0
        self.drone_x = self.drone_y = self.drone_z = 0.0
        self._bucket_neds, self._bucket_radii = [], []


class _FakeNode:
    """给 exec 出来的节点方法一个假 self。"""
    def __init__(self, home, pos, heading=0.0, pitch=0.0, roll=0.0):
        self.home_x, self.home_y, self.home_z = home
        self.x, self.y, self.z = pos
        self.heading, self._pitch, self._roll = heading, pitch, roll
        self._drop_count = 0


def _load_node_geometry(path):
    src = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math}
    want = {"IMG_W", "IMG_H", "CX", "CY", "HFOV", "VFOV", "FX", "FY", "TARGET_PLANE_H"}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        names = []
        for tg in node.targets:
            if isinstance(tg, ast.Name):
                names.append(tg.id)
            elif isinstance(tg, ast.Tuple):
                names += [e.id for e in tg.elts if isinstance(e, ast.Name)]
        if names and all(nm in want for nm in names):
            exec(ast.get_source_segment(src, node), ns)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "camera_pixel_to_level":
            exec(ast.get_source_segment(src, node), ns)
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in (
                    "_roi_to_ned", "_body_to_ned", "_ned_to_body"):
                exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
    return ns


def _load_detector_proj():
    src = DET_PATH.read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math}
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in (
                    "_gt_ground_circles", "_circles_to_boxes"):
                exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
    hf = re.search(r"self\.hfov\s*=\s*math\.radians\(([\d.]+)\)", src)
    vf = re.search(r"self\.vfov\s*=\s*math\.radians\(([\d.]+)\)", src)
    assert hf and vf, "检测器里找不到硬编码的 hfov/vfov"
    return ns, float(hf.group(1)), float(vf.group(1))


def sitl_roundtrip(path, bucket_ned_z=0.58, bucket_r=0.125):
    """检测器 ground 投影框 → 节点 _roi_to_ned → 桶真实水平位置;

    默认 `project_mode=ground` 的检测器把桶当"**参考平面上的圆**"投影
    (`dz = -drone_z`, 不含桶高), 与 `_roi_to_ned` 的平面求交**互为逆运算**
    ⇒ 只要「节点 TARGET_PLANE_H == 0」且「节点 FX/FY == 检测器的 fx/fy」,
       往返误差必须**精确为 0**。任一处与仿真器不一致都会立刻暴露。
    返回 (最大误差 m, 节点常量 ns, (det_fx, det_fy, det_hfov_deg, det_vfov_deg))。
    """
    ns = _load_node_geometry(path)
    det_ns, det_hfov_deg, det_vfov_deg = _load_detector_proj()
    det_fx = ns["IMG_W"] / (2.0 * math.tan(math.radians(det_hfov_deg) / 2.0))
    det_fy = ns["IMG_H"] / (2.0 * math.tan(math.radians(det_vfov_deg) / 2.0))
    for nm, fn in det_ns.items():
        setattr(_FakeDet, nm, fn)
    det = _FakeDet(det_fx, det_fy)
    worst = worst_px = 0.0
    for h in (1.2, 1.5, 2.5, 5.0):
        for rho in (0.05, 0.10, 0.20, 0.30, 0.50):
            det.drone_x, det.drone_y, det.drone_z = 0.0, 0.0, -h
            det._bucket_neds = [(0.0, rho, bucket_ned_z)]
            det._bucket_radii = [bucket_r]
            boxes = det._circles_to_boxes(det._gt_ground_circles())
            if not boxes:
                continue
            x1, y1, x2, y2 = boxes[0]
            roi = _Roi(x1, y1, x2 - x1, y2 - y1)
            m = _FakeNode((0.0, 0.0, 0.0), (0.0, 0.0, -h))
            for nm2, v in ns.items():
                if inspect.isfunction(v):
                    setattr(m, nm2, types.MethodType(v, m))
            e = m._roi_to_ned(roi)
            if e is None:
                continue
            err = max(abs(e[0] - 0.0), abs(e[1] - rho))
            worst = max(worst, err)
            # 归一化成"等效像素": 整数框量化地板 ≈ 1px, 用像素比用米更能说明问题
            worst_px = max(worst_px, err * ns["FX"] / h)
    return worst, worst_px, ns, (det_fx, det_fy, det_hfov_deg, det_vfov_deg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a", nargs="?", default="k", help="验证台节点字母 (小场地)")
    ap.add_argument("b", nargs="?", default="l", help="全任务节点字母")
    ap.add_argument("--fly", action="store_true", help="同时核对 fly 侧")
    args = ap.parse_args()

    pairs = [(args.a, args.b, ROOT / f"src/drone_drl/drone_drl/{x}.py") for x in (args.a, args.b)]
    if args.fly:
        pairs = [(args.a, args.b, ROOT / f"fly/src/fly/fly/{x}.py") for x in (args.a, args.b)]

    fa = Path(pairs[0][2])
    fb = Path(pairs[1][2])
    for f in (fa, fb):
        if not f.exists():
            sys.exit(f"缺少 {f}")

    A, B = collect(fa), collect(fb)
    na, nb = args.a, args.b
    fails, warns = [], []

    print("=" * 90)
    print(f"可搬运性检查:  {na} ({fa.parent.name}/{fa.name})   ↔   {nb} ({fb.name})")
    print("=" * 90)

    # ---------- A 几何链 ----------
    print("\n[A] 几何链 —— 必须逐字节相同 (决定标定参数能否照搬)")
    for fn in MUST_MATCH_FUNCS:
        sa, sb = A["funcs"].get(fn), B["funcs"].get(fn)
        if sa is None or sb is None:
            fails.append(f"A: 两侧都应存在函数 {fn} (a={sa is not None}, b={sb is not None})")
            print(f"  \u2717 {fn:<28} 缺失 (a={sa is not None} b={sb is not None})")
            continue
        ok = sa == sb
        print(f"  {OK if ok else NG} {fn:<28} {'一致' if ok else '**不一致**'} "
              f"({len(sa.splitlines())} 行)")
        if not ok:
            fails.append(f"A: {fn} 两侧不一致 ⇒ 标定的平面/几何不能照搬")
            import difflib
            for d in list(difflib.unified_diff(sa.splitlines(), sb.splitlines(),
                                               lineterm="", n=0))[:12]:
                if d.startswith(("+", "-")) and not d.startswith(("+++", "---")):
                    print("        " + d[:150])

    ha, hb = set(A["holds"]), set(B["holds"])
    # 子集语义: 验证台可以【多】一套等价公式(如 k 既有的重新冻结杠杆, 默认关),
    # 但**必须包含**全任务节点的全部冻结/保持点公式 —— 少一条就是搬不过去。
    missing = hb - ha
    extra = ha - hb
    ok_h = (not missing) and bool(hb)
    print(f"  {OK if ok_h else NG} {'冻结公式 (hold=est+comp)':<28} "
          f"{'验证台是超集' if ok_h else '**验证台缺少公式**'} "
          f"(验证台 {len(ha)} 条 / 全任务 {len(hb)} 条)")
    for x in sorted(missing):
        print(f"        {NG} 全任务有而验证台没有: {x[:140]}")
    for x in sorted(extra):
        print(f"        {DOT} 验证台额外(默认关等价): {x[:140]}")
    if missing:
        fails.append(f"A: 验证台缺少 {len(missing)} 条冻结/保持点公式 ⇒ 搬不过去")

    # ---------- A2 SITL 往返精确性 ----------
    if not args.fly:
        print("\n[A2] SITL 链路往返误差 —— 必须精确为 0 (平面 + 内参与仿真器一致)")
        try:
            wa, pxa, nsa, detv = sitl_roundtrip(fa)
            wb, pxb, nsb, _ = sitl_roundtrip(fb)
            print(f"  {DOT} 检测器模拟相机(硬编码): HFOV={detv[2]:g}° VFOV={detv[3]:g}° "
                  f"⇒ fx={detv[0]:.3f} fy={detv[1]:.3f}")
            ok_plane = ok_intr = True
            for tag, nsv in ((na, nsa), (nb, nsb)):
                print(f"  {DOT} 节点 {tag}: TARGET_PLANE_H={nsv['TARGET_PLANE_H']} "
                      f"HFOV={math.degrees(nsv['HFOV']):g}° VFOV={math.degrees(nsv['VFOV']):g}° "
                      f"⇒ FX={nsv['FX']:.3f} FY={nsv['FY']:.3f}")
                # 判据 1: 平面必须精确 0 (ground 模式检测器把桶放在参考平面上)
                if nsv["TARGET_PLANE_H"] != 0.0:
                    ok_plane = False
                    fails.append(f"A2: 节点 {tag} 的 TARGET_PLANE_H={nsv['TARGET_PLANE_H']} ≠ 0 "
                                 f"⇒ ground 模式 SITL 深度错")
                # 判据 2: 内参必须与检测器的模拟相机逐位相等
                if nsv["FX"] != detv[0] or nsv["FY"] != detv[1]:
                    ok_intr = False
                    fails.append(f"A2: 节点 {tag} 的 FX/FY={nsv['FX']:.3f}/{nsv['FY']:.3f} "
                                 f"≠ 检测器 {detv[0]:.3f}/{detv[1]:.3f} ⇒ 尺度误差 "
                                 f"{max(detv[0], nsv['FX']) / min(detv[0], nsv['FX']):.4f}x")
            print(f"  {OK if ok_plane else NG} 判据 1: 两侧 TARGET_PLANE_H 都 = 0.0 "
                  f"(ground 模式与 _roi_to_ned 互为逆运算的必要条件)")
            print(f"  {OK if ok_intr else NG} 判据 2: 两侧 FX/FY 与检测器模拟相机逐位相等")
            w_px = max(pxa, pxb)
            ok3 = w_px <= 1.0
            if not ok3:
                fails.append(f"A2: SITL 往返误差 {w_px:.3f} px > 1 px (量化地板)")
            print(f"  {OK if ok3 else NG} 判据 3: 往返最大误差 {w_px:.4f} px "
                  f"({max(wa, wb) * 1000:.2f} mm) ≤ 1 px 量化地板")
        except Exception as exc:  # noqa: BLE001
            fails.append(f"A2: 往返检查无法执行: {exc!r}")
            print(f"  {NG} 往返检查无法执行: {exc!r}")
    else:
        print("\n[A2] (--fly: 跳过 SITL 往返检查 —— 该判据只对仿真侧成立)")

    # ---------- B 投放常量 ----------
    print("\n[B] 投放常量 —— 必须数值相同")
    bad = []
    for k in MUST_MATCH_CONSTS:
        va, vb = A["consts"].get(k, "<missing>"), B["consts"].get(k, "<missing>")
        if va != vb:
            bad.append((k, va, vb))
            fails.append(f"B: {k} 不一致 ({va} vs {vb})")
    if bad:
        for k, va, vb in bad:
            print(f"  {NG} {k:<34} {na}={va}   {nb}={vb}")
    else:
        print(f"  {OK} 全部 {len(MUST_MATCH_CONSTS)} 个常量一致 "
              f"(含 TARGET_PLANE_H={A['consts'].get('TARGET_PLANE_H')}, "
              f"Y_B1={A['consts'].get('PAYLOAD_OFFSET_BODY_Y_B1')}, "
              f"Y_B2={A['consts'].get('PAYLOAD_OFFSET_BODY_Y_B2')}, "
              f"XF={A['consts'].get('PAYLOAD_OFFSET_BODY_XF_B1')}, "
              f"L_BODY_GATE_M={A['consts'].get('L_BODY_GATE_M')}, "
              f"DROP_HEIGHT={A['consts'].get('DROP_HEIGHT')})")

    # ---------- C 差异清单 ----------
    print("\n[C] 差异清单 —— 允许不同, 但必须判定是否影响落点几何")
    only_a = [k for k in A["order"] if k not in B["consts"]]
    only_b = [k for k in B["order"] if k not in A["consts"]]
    both_diff = [k for k in A["order"] if k in B["consts"]
                 and A["consts"][k] != B["consts"][k] and k not in MUST_MATCH_CONSTS]
    unknown = []
    for k in sorted(set(both_diff) | set(only_a) | set(only_b)):
        impact, note = KNOWN_DIFF.get(k, (None, None))
        if impact is None:
            unknown.append(k)
            tag, txt = "?", "**未登记** —— 请判定是否影响落点"
        else:
            tag, txt = impact, note
        va = A["consts"].get(k, "<仅" + nb + ">")
        vb = B["consts"].get(k, "<仅" + na + ">")
        print(f"  {DOT} {k:<26} {na}={str(va):<8} {nb}={str(vb):<8} [{tag}] {txt}")
        if tag == "**影响**":
            warns.append(f"C: {k} 会影响投放行为 —— {note}")
    if not unknown and not (both_diff or only_a or only_b):
        print(f"  {OK} 两侧常量完全相同, 无差异")

    # ---------- 结论 ----------
    print("\n" + "=" * 90)
    if fails:
        print(f"结论: {NG} 不可搬运 —— {len(fails)} 项阻断")
        for f in fails:
            print("   " + f)
    else:
        print(f"结论: {OK} 几何链与投放常量全部一致 ⇒ **标定/验证结果可照搬**")
    if warns:
        print(f"\n注意 ({len(warns)} 项已知行为差异, 不阻断但会影响可比性):")
        for w in warns:
            print("   " + w)
    print("=" * 90)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
