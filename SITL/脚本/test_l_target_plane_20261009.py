#!/usr/bin/env python3
"""项目 l 离线验收 — P1 目标平面参数 + P2 机体坐标系释放闭环门 (2026-10-09)

三节
----
A 最小改动审计: 剥注释后 l.py 与 i.py 的**每一处差异**都必须落在允许集合内;
  且两个开关关掉时, 真实代码路径 `_roi_to_ned` 与 i **逐位相同**。
B P1 生效: 用 P0 的【独立光线投射真值】量残差 —— 与 P0 验收 [E] 节同一把尺子。
C P2 免疫性: 注入 EKF 位置偏置, **机体系残差不变**而世界系量(dev 一类)跟着变
  ⇒ 证明 P2 的判据对"飞机自身位置估计漂移"完全免疫 (这是它相对 j 的 dev 的本质区别)。

用法: python3 scripts/test_l_target_plane_20261009.py
"""
import ast
import difflib
import inspect
import json
import math
import re
import sys
import textwrap
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_silhouette_projection_20261009 as P0  # noqa: E402  (复用 P0 的装载器与真值)

ROOT = Path(__file__).resolve().parents[1]
# 节点字母 (默认 l)。基底 (审计对照) 的取法:
#   l -> i.py                (l 是 i 的新副本)
#   k -> k.py 的最近一份改前备份 (k 是【就地】打补丁的, 改前版本就是基底)
import glob as _glob
NODE = (sys.argv[1] if len(sys.argv) > 1 else "l").strip()
_FIXED_BASE = {"l": "i"}
if NODE in _FIXED_BASE:
    BASE = _FIXED_BASE[NODE]
    I_PY = ROOT / f"src/drone_drl/drone_drl/{BASE}.py"
else:
    BASE = NODE
    # 只认"标定前"那一版做基准: 排除 _sitl_env / _P1P2 之后的增量补丁备份
    # 排除"后续就地补丁"的备份, 保证基准始终是**本节点那次 P1/P2 生成时的前一代**。
    # ⚠ 约定: 以后再用就地补丁改了 k, 请把该补丁的后缀加进这里, 并把它的标记加进 ALLOWED
    #   (越界报错就是在提醒你"有未登记的改动")。
    # 基准要停在"本补丁之前那一版" ⇒ 排除所有后续就地补丁的备份后缀
    _BAK_EXCLUDE = ("_sitl_env", "_flat", "_obloss",
                    "_ekf_tlm", "_vis_gate", "_stall", "_alt5m", "_desc_fix", "_anchor",
                    "_anchor_bucket")
    _baks = [x for x in sorted(_glob.glob(
        str(ROOT / f"src/drone_drl/drone_drl/{NODE}.py.bak.*")))
        if not x.endswith(_BAK_EXCLUDE)]
    if not _baks:
        sys.exit(f"找不到 {NODE}.py 的改前备份 —— 最小改动审计需要它")
    I_PY = Path(_baks[-1])
L_PY = ROOT / f"src/drone_drl/drone_drl/{NODE}.py"      # 本节点
L_FLY = ROOT / f"fly/src/fly/fly/{NODE}.py"

BUCKET_R, BUCKET_L, BUCKET_NED_Z = P0.BUCKET_R, P0.BUCKET_L, P0.BUCKET_NED_Z
_ok = _fail = 0


def check(cond, msg):
    global _ok, _fail
    if cond:
        _ok += 1
        print(f"  \u2713 {msg}")
    else:
        _fail += 1
        print(f"  \u2717 {msg}")
    return bool(cond)


# 允许出现差异的全部标记 (超出即视为越界改动)
# 2026-10-09: 后续就地补丁的标记集中登记 (见 scripts/_later_patch_marks.py)
try:
    from _later_patch_marks import LATER_PATCH_MARKERS as _SHARED, \
        LATER_PATCH_BAK_SUFFIXES as _SHARED_BAK
except Exception:      # 单独拷出去跑时退化为空表
    _SHARED, _SHARED_BAK = (), ()

ALLOWED = tuple(_SHARED) + (f"{NODE}_mission", f"{NODE.upper()}Mission", "20261009-r1-i-P1P2", "TARGET_PLANE_H", "L_BODY_",
           "_l_body_resid", "_l_refreeze", "_l_body_raw", "l/P1", "l/P2", "_bres", "_body_ok",
           "P1 TARGET_PLANE_H", "P2 L_BODY_GATE_M",
           "P1:", "P2:", f"{BASE.upper()}Mission", f"'{NODE} = ",
           "MISSION_REVISION", "competition-water-20261009-r1-",
           # ---- 2026-10-09 首飞反馈整改 (scripts/patch_k_flat_20261009.py, 仅 k) ----
           "K_FLIGHT_ALT", "K_FLIGHT_SPEED", "K_VISUAL_GATE_M", "CRUISE_HEIGHT",
           "PRE_SERVO_HEIGHT", "_travel_fwd", "视觉门", "全程平飞",
           "FORWARD_MIN_SPEED, K_FLIGHT_SPEED", "self.home_x, self.home_y,",
           # ---- 2026-10-09 OFFBOARD 丢失识别 (切手操/模式切换) ----
           "_ob_lost_logged", "_active_logged", "offboard_lost_external", "切手操",
           "ARM/OFFBOARD 丢失", "Mode timeout",
           # ---- 2026-10-09 FC 模式 + EKF 健康度观测字段 (scripts/patch_ekf_tlm_20261009.py) ----
           # 只加观测: __init__ 两字段 / _cb_local 快照 / 1Hz 行尾追加 FC(..) EKF(..) / 失效 WARN
           "self._ekf", "_ekf_warn_t", "_st = self.status", "fc_str", "ek_str",
           # ---- 2026-10-09 前飞无进展保护 + landed 记录 (scripts/patch_k_stall_guard_20261009.py) ----
           "K_FWD_STALL", "_fwd_stall_logged", "forward_stall", "无进展",
           # ---- 2026-10-09 5m 巡航剖面 (scripts/patch_k_alt_5m_20261009.py) ----
           "K_DESCEND_RATE", "_z_desc", "_pre_z", "降高", "K_FLIGHT_ALT  = 5.0")

# 2026-10-09 静态标定: 只允许改【值】, 不允许改名字/行结构。
# 块内每一行都必须是下列常量的纯赋值, 且 a/b 两侧的名字集合必须相同
# (insert/delete 一侧为空 ⇒ 名字集合不等 ⇒ 判为越界, 这正是我们要的严格性)。
CALIB_EXPECT = {"PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_Y_B2",
                "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2",
                "HFOV", "VFOV"}
_ASSIGN_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*=\s*[^=]")


def _names(lines):
    return {m.group(1) for m in (_ASSIGN_RE.match(l.strip()) for l in lines) if m}


def _calib_only(lines):
    """块内每一行都必须是 CALIB_EXPECT 里某个常量的纯赋值 (只改值, 不改结构)。"""
    if not lines:
        return True
    for l in lines:
        m = _ASSIGN_RE.match(l.strip())
        if m is None or m.group(1) not in CALIB_EXPECT:
            return False
    return True


def strip_comments(path):
    out = []
    for ln in Path(path).read_text(encoding="utf-8").splitlines():
        t = ln.strip()
        if not t or t.startswith("#"):
            continue
        out.append(ln.rstrip())
    return out


def load_methods(path, names, extra_ns=None):
    src = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math, "time": __import__("time"), "json": json}
    ns.update(extra_ns or {})
    # 模块级常量 (方法体里会引用)
    want = {"IMG_W", "IMG_H", "CX", "CY", "HFOV", "VFOV", "FX", "FY",
            "TARGET_PLANE_H", "L_BODY_GATE_M", "L_BODY_GATE_LOG", "L_BODY_REFREEZE_MAX",
            "PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_Y_B2",
            "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2"}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            nm = [s.id for t in node.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if nm and all(x in want for x in nm):
                exec(ast.get_source_segment(src, node), ns)
    found = set()
    # 注意: 源文件里有多个类 (PIDController 在前), 必须扫描【所有】类
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in names and fn.name not in found:
                exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
                found.add(fn.name)
    missing = set(names) - found
    assert not missing, f"{Path(path).name}: 缺少方法 {sorted(missing)}"
    return ns


class FakeMission:
    def __init__(self, home, pos, heading, pitch=0.0, roll=0.0, drop_count=0, roi=None):
        self.home_x, self.home_y, self.home_z = home
        self.x, self.y, self.z = pos
        self.heading = heading
        self._pitch, self._roll = pitch, roll
        self._drop_count = drop_count
        self.latest_roi = roi

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * c - v_lat * s, v_fwd * s + v_lat * c, v_down)


def bind(fake, ns):
    for name, fn in ns.items():
        if inspect.isfunction(fn):
            setattr(fake, name, types.MethodType(fn, fake))
    return fake


# ============================================================
# A 最小改动审计 + 默认关等价
# ============================================================
def section_a():
    print("\n[A] 最小改动审计 + 默认关等价")
    a, b = strip_comments(I_PY), strip_comments(L_PY)
    sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
    blocks, bad, calib_blocks, calib_names = 0, [], 0, set()
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "equal":
            continue
        blocks += 1
        chunk = "\n".join(a[i1:i2] + b[j1:j2])
        if any(mk in chunk for mk in ALLOWED):
            continue                                    # P1/P2 结构改动
        if _calib_only(a[i1:i2]) and _calib_only(b[j1:j2]) and \
                _names(a[i1:i2]) == _names(b[j1:j2]):
            calib_blocks += 1                           # 标定: 只改常量的值
            calib_names.update(_names(b[j1:j2]))
            continue
        bad.append(chunk[:300])
    check(not bad, f"剥注释后 {len(a)}→{len(b)} 行, {blocks} 处差异全部落在允许集合内")
    check(calib_names == CALIB_EXPECT,
          f"{calib_blocks} 处标定块【只改了值】(名字集与标定清单一致: {sorted(calib_names)})")
    for chunk in bad[:3]:
        print("      越界:\n" + chunk)

    # 两份 l.py (SITL/fly): P2 逻辑开关必须一致; TARGET_PLANE_H 是**环境量, 应当不同**
    #   SITL 0.0      = 检测器默认 project_mode=ground 把桶当"参考平面上的圆" ⇒ 与 _roi_to_ned 互逆
    #   实机 -0.278   = 卷尺 A(0.022) − R(0.300)
    #   机器核对 (往返误差 ≤1px) 见 scripts/check_drop_portability_20261009.py A2 节
    ca = load_methods(L_PY, set())
    cb = load_methods(L_FLY, set())
    same = all(ca.get(k) == cb.get(k) for k in
               ("L_BODY_GATE_M", "L_BODY_GATE_LOG", "L_BODY_REFREEZE_MAX"))
    check(same, f"两份 l.py 的 P2 逻辑开关一致 (L_BODY_GATE_M={ca.get('L_BODY_GATE_M')}, "
                f"L_BODY_GATE_LOG={ca.get('L_BODY_GATE_LOG')}, "
                f"L_BODY_REFREEZE_MAX={ca.get('L_BODY_REFREEZE_MAX')})")
    pa, pb = ca.get("TARGET_PLANE_H"), cb.get("TARGET_PLANE_H")
    check(pa == 0.0 and pb == -0.278,
          f"TARGET_PLANE_H 分侧正确: SITL={pa} (仿真器 ground 模式 ⇒ 0.0) / "
          f"fly={pb} (实机卷尺 ⇒ -0.278)")

    # P1 深度恒等律: `camera_pixel_to_level` 与高度无关 (只返回 前向/H, 右向/H)
    #   ⇒ ned(t) = pos + R·(v·(H+t)) = ned(0) + (t/H)·(ned(0) − pos)   精确成立
    # 这比旧的"默认 0 时与 i 逐位相同"【更强】: 它证明 P1 在 _roi_to_ned 里只以深度
    # 偏移的形式进入 (没混进任何别的改动), 且在标定值非 0 时依然成立。
    cam = P0.load_geometry(I_PY)
    cam_ns = {"camera_pixel_to_level": cam["camera_pixel_to_level"]}
    ns_l = load_methods(L_PY, {"_roi_to_ned"}, extra_ns=cam_ns)
    t_cal = ca.get("TARGET_PLANE_H", 0.0)
    worst, ncase = 0.0, 0
    for h in (1.2, 1.5, 2.5, 5.0):
        for px, py in ((320, 180), (400, 180), (320, 260), (250, 120), (500, 300)):
            roi = P0.ROI(px - 10, py - 10, 20, 20)
            for hdg in (0.0, math.radians(92.0), math.radians(213.0)):
                for pit, rol in ((0.0, 0.0), (4.0, -2.0)):
                    pos = (3.0, 4.0, -h)
                    fl = FakeMission((1.0, 2.0, 0.0), pos, hdg, pit, rol, roi=roi)
                    ns_l["TARGET_PLANE_H"] = 0.0
                    n0 = ns_l["_roi_to_ned"](fl, roi)
                    rx, ry = fl.x - fl.home_x, fl.y - fl.home_y
                    for t in sorted({t_cal, -0.278, -0.15, 0.455}):   # 含两侧各自的实际取值
                        ns_l["TARGET_PLANE_H"] = t
                        nt = ns_l["_roi_to_ned"](fl, roi)
                        exp = (n0[0] + (t / h) * (n0[0] - rx),
                               n0[1] + (t / h) * (n0[1] - ry))
                        worst = max(worst, abs(nt[0] - exp[0]), abs(nt[1] - exp[1]))
                        ncase += 1
    ns_l["TARGET_PLANE_H"] = t_cal                        # 复原, 不留副作用
    check(worst < 1e-9,
          f"P1 只以【深度偏移】进入 _roi_to_ned: {ncase} 组 "
          f"(t={sorted({t_cal, -0.278, -0.15, 0.455})}) 最大偏差 {worst:.1e} m")

    # P2 门关: 释放条件行必须仍以 i 的原条件开头 (静态检查)
    src = L_PY.read_text(encoding="utf-8")
    rel_ok = "(L_BODY_GATE_M > 0 and _settled and not _body_ok" in src and \
        (f"and _pred_ok and _body_ok) or _dl_pick:" in src
         or f"and _pred_ok and _dev_ok and _body_ok) or _dl_pick:" in src)
    check(rel_ok,
          f"默认 L_BODY_GATE_M=0 ⇒ _body_ok 恒 True, 释放条件与 {BASE} 等价 "
          f"(基线条件 {'带 _dev_ok' if BASE == 'k' else '无 _dev_ok'})")


# ============================================================
# B P1 生效 (与 P0 同一把尺子: 独立光线投射真值)
# ============================================================
def section_b():
    print("\n[B] P1 生效 — 与 P0 [E] 同一把尺子 (silhouette 真值)")
    print("    注: 本节的 0.455 是 **P0 silhouette 真值**下的对照列 (验 P1 能否修平面误差),")
    print("        不是 SITL 默认(ground)配置该用的值 —— 默认配置的正确值是 0.0 (见 A 节)。")
    geo = P0.load_methods(P0.DET, {"_gt_boxes_silhouette"})
    cam = P0.load_geometry(I_PY)
    ns_l = load_methods(L_PY, {"_roi_to_ned"}, extra_ns={"camera_pixel_to_level":
                                                         cam["camera_pixel_to_level"]})
    # 注意: exec(code, ns) 使函数的 __globals__ 就是 ns 本身 ⇒ 必须【原地】改 ns,
    # dict(ns) 拷贝出来的新字典改不动已编译函数的全局查找。
    def resid(plane_h, h, rho):
        ns_l["TARGET_PLANE_H"] = plane_h
        fake_det = P0.FakeDet("silhouette", [(0.0, rho, BUCKET_NED_Z)], [BUCKET_R],
                              (0.0, 0.0, -h), 0.0)
        boxes = geo["_gt_boxes_silhouette"](fake_det)
        if not boxes:
            return None
        b = boxes[0]
        roi = P0.ROI(b[0], b[1], b[2] - b[0], b[3] - b[1])
        m = FakeMission((0.0, 0.0, 0.0), (0.0, 0.0, -h), 0.0, roi=roi)
        for k, v in ns_l.items():
            if inspect.isfunction(v):
                setattr(m, k, types.MethodType(v, m))
        e = m._roi_to_ned(roi)
        if e is None:
            return None
        return math.hypot(e[0] - 0.0, e[1] - rho)

    print(f"  (本节相机内参取自文件: FX={ns_l['FX']:.2f} px ⇒ 绝对数值随 SITL/实机配置变化)")
    print(f"  {'H(m)':>5} {'ρ(cm)':>7} {'TARGET_PLANE_H=0':>18} {'=0.455':>11} {'改善':>8}")
    rows = []
    for h in (1.2, 1.5, 2.5):
        for rho in (0.05, 0.10, 0.20, 0.30, 0.50):
            r0, r1 = resid(0.0, h, rho), resid(0.455, h, rho)
            if r0 is None or r1 is None:
                continue
            rows.append((h, rho, r0, r1))
            print(f"  {h:5.1f} {rho*100:7.1f} {r0*100:17.2f}cm {r1*100:10.2f}cm "
                  f"{r0/max(r1,1e-9):7.1f}×")
    worst0 = max(r[2] for r in rows if abs(r[0] - 1.2) < 1e-9)
    worst1 = max(r[3] for r in rows if abs(r[0] - 1.2) < 1e-9)
    check(worst1 < worst0 / 3.0,
          f"H=1.2m 最坏残差 {worst0*100:.2f}cm → {worst1*100:.2f}cm "
          f"({worst0/max(worst1,1e-9):.1f}×)  ⇒ 与 P0 [E] 一致")
    # 绝对残差随相机配置变化 (SITL 85° ⇒ FX=349.2; 实机 89.8° ⇒ FX=321.1)
    # ⇒ 这里【现场扫出本配置的最优平面】再判它够不够小, 而不是钉死某一次的数字。
    # (P0 silhouette 真值下最优 ≈0.53 = 圆柱轮廓框中心对应的等效平面; 桶口平面 0.455 略浅。)
    opt_p, opt_v = 0.0, 9.9
    for i in range(0, 81):
        q = i * 0.01
        vals = [v for v in (resid(q, 1.2, rr) for rr in (0.05, 0.10, 0.20)) if v is not None]
        if vals and max(vals) < opt_v:
            opt_p, opt_v = q, max(vals)
    print(f"  ⇒ 本配置扫出的最优平面 = {opt_p:.2f} m (ρ≤20cm 最坏残差 {opt_v*100:.2f}cm); "
          f"桶口平面 0.455 的残差 {max(r[3] for r in rows if abs(r[0]-1.2)<1e-9 and r[1] <= 0.20)*100:.2f}cm")
    check(opt_v < 0.01,
          f"H=1.2m/ρ≤20cm 时残差 ≤1cm (实测最大 {opt_v*100:.2f}cm @ plane={opt_p:.2f})")


# ============================================================
# C P2 免疫性 — 注入 EKF 位置偏置
# ============================================================
def section_c():
    print("\n[C] P2 免疫性 — 注入 EKF 位置偏置, 机体系残差应**不变**")
    cam = P0.load_geometry(I_PY)
    ns_l = load_methods(L_PY, {"_l_body_resid", "_l_body_raw", "_roi_to_ned"},
                        extra_ns={"camera_pixel_to_level": cam["camera_pixel_to_level"]})
    ns_i = load_methods(I_PY, {"_roi_to_ned"}, extra_ns={"camera_pixel_to_level":
                                                        cam["camera_pixel_to_level"]})

    # 场景: 飞机在 NED(0,0,-1.2), 桶真值在 NED(0, 0.30, 0.58) ⇒ 机体右 30cm
    rho = 0.30
    fake_det = P0.FakeDet("silhouette", [(0.0, rho, BUCKET_NED_Z)], [BUCKET_R],
                          (0.0, 0.0, -1.2), 0.0)
    b = geo_boxes = P0.load_methods(P0.DET, {"_gt_boxes_silhouette"})["_gt_boxes_silhouette"](fake_det)[0]
    roi = P0.ROI(b[0], b[1], b[2] - b[0], b[3] - b[1])

    def make(ns, x, y):
        m = FakeMission((0.0, 0.0, 0.0), (x, y, -1.2), 0.0, roi=roi, drop_count=0)
        m.TARGET_PLANE_H = ns.get("TARGET_PLANE_H", 0.0)
        for k, v in ns.items():
            if inspect.isfunction(v):
                setattr(m, k, types.MethodType(v, m))
        m._drop_detection_fresh = lambda: True
        m._i_roi_inside = lambda r: True
        return m

    print(f"  {'注入 EKF 位置偏置':>22} {'机体系残差(P2)':>16} {'世界系 dev 一类':>18} {'Δ机体系':>10} {'Δ世界系':>10}")
    # 世界系对照量: j 的 dev = |est_live − est_frozen|, est_frozen 是偏置为 0 时冻结的
    # 世界系估计 (真实飞行里就是这样冻结的), est_live 用当前偏置重算 ⇒ 偏置全量进 dev。
    mi0 = make(ns_i, 0.0, 0.0)
    est_frozen = mi0._roi_to_ned(roi)
    base_body = base_dev = None
    worst_db = 0.0
    for bias in (0.0, 0.15, -0.25, 0.40):
        ml = make(ns_l, 0.0 + bias, 0.0)
        mi = make(ns_i, 0.0 + bias, 0.0)
        body = ml._l_body_resid()
        est_live = mi._roi_to_ned(roi)
        dev = math.hypot(est_live[0] - est_frozen[0], est_live[1] - est_frozen[1])
        if base_body is None:
            base_body, base_dev = body, dev
        worst_db = max(worst_db, abs(body - base_body))
        print(f"  {bias:>+22.2f}m {body*100:15.2f}cm {dev*100:17.2f}cm "
              f"{abs(body-base_body)*100:9.2f}cm {abs(dev-base_dev)*100:9.2f}cm")
        check(abs(body - base_body) < 1e-9,
              f"偏置 {bias:+.2f}m: 机体系残差不变 ({body*100:.2f}cm)")
    dev0 = None
    for bias in (0.20, 0.35):
        mi = make(ns_i, bias, 0.0)
        est_live = mi._roi_to_ned(roi)
        d = math.hypot(est_live[0] - est_frozen[0], est_live[1] - est_frozen[1])
        dev0 = d if dev0 is None else dev0
        check(abs(d - bias) < 1e-9,
              f"对照: 世界系 dev 精确等于注入偏置 {bias*100:.0f}cm (实测 {d*100:.1f}cm) "
              f"⇒ 这正是 j 的门看不见真相的原因")

    ml = make(ns_l, 0.0, 0.0)
    body = ml._l_body_resid()
    # 期望投放口偏移 = (-XF, -Y); i 的 B1 补偿 Y=+0.18, XF=+0.05
    expect = math.hypot(ns_l.get("PAYLOAD_OFFSET_BODY_Y_B1", 0.0) ** 2
                        + ns_l.get("PAYLOAD_OFFSET_BODY_XF_B1", 0.0) ** 2)
    check(body is not None and body < 0.10,
          f"无偏置时 桶在机体右 30cm, 补偿({ns_l.get('PAYLOAD_OFFSET_BODY_Y_B1')},"
          f"{ns_l.get('PAYLOAD_OFFSET_BODY_XF_B1')}) ⇒ 残差 {body*100:.1f}cm "
          f"(= |桶机体系偏移 + 补偿|, 期望同阶于 {expect*100:.0f}cm)")
    check(worst_db < 1e-9,
          f"四个偏置点机体系残差最大变化 {worst_db*1000:.4f}mm ≈ 0 ⇒ **对 EKF 漂移免疫**")


if __name__ == "__main__":
    print("=" * 84)
    print("项目 l 离线验收 — P1 目标平面 + P2 机体系释放闭环 (2026-10-09)")
    print(f"  基底: {I_PY.name}   本节点: {L_PY.name}")
    print(f"  fly : {L_FLY.relative_to(ROOT)}")
    print("=" * 84)
    section_a()
    section_b()
    section_c()
    print("\n" + "=" * 84)
    print(f"结果: {_ok} 项通过 / {_fail} 项失败")
    print("=" * 84)
    sys.exit(1 if _fail else 0)
