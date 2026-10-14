#!/usr/bin/env python3
"""P0 离线验收 — 仿真检测器 "真实圆柱轮廓投影" (project_mode=silhouette, 2026-10-09)

背景 (为什么要做 P0)
--------------------
`yolov8_bucket_detector` 的 sim GT 路径一直把桶当成"**参考平面上的一个圆**"投影:
圆心 = 桶轴在该平面上的交点, 半径 = fx·r/H (H = -drone_z)。
而任务节点 `_roi_to_ned` 做的正是"**像素射线与该平面求交**"。
两者**互为逆运算** ⇒ SITL 往返误差恒等于 0 ⇒ **仿真对实机那一类
"目标平面选错"的误差 (桶口平面 vs 地面平面) 结构性失明**。

实机相机看到的是**圆柱轮廓**: 桶顶圆环 (离相机近) + 离轴时可见的桶底圆环。
本脚本验收新的 `project_mode='silhouette'` 是否忠实复现了这一点。

三节
----
  A 回归 — 默认 `project_mode='ground'` 的输出与**改前备份**逐位相同 (旧项目零影响)
  B 真值 — `silhouette` 的框与**独立光线投射真值**一致; 并验证"只采两圆足够"
  C 量化 — 同一几何下, 任务节点(真代码路径)反投影的落点误差表 —— P0 要暴露的那个量

用法: python3 scripts/test_silhouette_projection_20261009.py
"""
import ast
import glob
import inspect
import json
import math
import sys
import textwrap
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DET = ROOT / "src/drone_drl/drone_drl/yolov8_bucket_detector.py"
NODE = ROOT / "src/drone_drl/drone_drl/i.py"

# 改前备份 (由本次改动生成); 取最新的一个
_baks = sorted(glob.glob(str(DET) + ".bak.*_silhouette"))
if not _baks:
    sys.exit(f"找不到改前备份 {DET}.bak.*_silhouette —— 无法做 A 节回归")
DET_OLD = Path(_baks[-1])

# i.world 的桶几何 (与 worlds/i.world 一致)
BUCKET_R = 0.125
BUCKET_L = 0.25
BUCKET_NED_Z = 0.58          # 桶模型中心 NED z (= -(0.25 - 0.83))
IMG_W, IMG_H = 640, 360

_ok = 0
_fail = 0


def check(cond, msg):
    global _ok, _fail
    if cond:
        _ok += 1
        print(f"  \u2713 {msg}")
    else:
        _fail += 1
        print(f"  \u2717 {msg}")
    return bool(cond)


# ============================================================
# 真代码路径装载: 从源文件里抽出方法原文 exec (项目既有手法)
# ============================================================
def load_methods(path, names):
    src = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(src)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    ns = {"math": math, "time": __import__("time"), "json": json}

    class _ROI:            # 单桶路径的 RegionOfInterest 桩
        x_offset = y_offset = width = height = 0

    class _Str:
        def __init__(self, data=""):
            self.data = data

    ns["RegionOfInterest"] = _ROI
    ns["String"] = _Str
    found = set()
    for fn in cls.body:
        if isinstance(fn, ast.FunctionDef) and fn.name in names:
            exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
            found.add(fn.name)
    missing = set(names) - found
    assert not missing, f"{path.name}: 缺少方法 {sorted(missing)}"
    return ns


def load_geometry(path):
    """抽出 i.py 的相机几何纯函数 + 相机常量 (任务节点的真代码路径)."""
    src = Path(path).read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math}
    want_const = {"IMG_W", "IMG_H", "CX", "CY", "HFOV", "VFOV", "FX", "FY"}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [s.id for t in node.targets for s in ast.walk(t)
                     if isinstance(s, ast.Name)]
            if names and all(n in want_const for n in names):
                exec(ast.get_source_segment(src, node), ns)
    missing_c = want_const - set(ns)
    assert not missing_c, f"i.py 缺少相机常量 {sorted(missing_c)}"
    want = {"camera_pixel_to_level", "level_to_camera_pixel"}
    found = set()
    for fn in tree.body:
        if isinstance(fn, ast.FunctionDef) and fn.name in want:
            exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
            found.add(fn.name)
    assert found == want, f"i.py 缺少 {want - found}"
    return ns


class FakeDet:
    """给 exec 出来的检测器方法提供一个假 self (只喂它们真正读的字段)."""
    def __init__(self, project_mode, buckets, radii, drone, heading=0.0,
                 bucket_length=BUCKET_L, samples=144, samples_side=False):
        self.img_w, self.img_h = IMG_W, IMG_H
        self.fx = IMG_W / (2.0 * math.tan(math.radians(85.0) / 2.0))
        self.fy = IMG_H / (2.0 * math.tan(math.radians(54.0) / 2.0))
        self._bucket_neds = list(buckets)
        self._bucket_radii = list(radii)
        self.drone_x, self.drone_y, self.drone_z = drone
        self.heading = heading
        self.visualize = False
        self._sim_gt_direct = True
        self._bucket_count = 3
        self._project_mode = project_mode
        self._bucket_length = bucket_length
        self._silhouette_samples = samples
        self._samples_side = samples_side     # B 节: 额外采样侧壁 (证明两圆已足够)
        self._frame_seq = 0
        self._last_json = None
        self._last_roi = None
        self.captured = []

        class _Pub:
            def __init__(self, out):
                self.out = out

            def publish(self, msg):
                if isinstance(msg.data, str):
                    self.out.append(json.loads(msg.data)["buckets"])
                else:
                    self.out.append(msg)

        self.multi_pub = _Pub(self.captured)
        self.det_pub = _Pub(self.captured)


class FakeMission:
    """给 i.py 的 _roi_to_ned 提供假 self."""

    def __init__(self, home, pos, heading, pitch=0.0, roll=0.0):
        self.home_x, self.home_y, self.home_z = home
        self.x, self.y, self.z = pos
        self.heading = heading
        self._pitch, self._roll = pitch, roll

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * c - v_lat * s, v_fwd * s + v_lat * c, v_down)


class ROI:
    def __init__(self, x, y, w, h):
        self.x_offset, self.y_offset, self.width, self.height = x, y, w, h


def bind(fake, ns):
    """把 exec 出来的方法绑到假 self 上 (让 self.xxx() 调得通)."""
    for name, fn in ns.items():
        if inspect.isfunction(fn):
            setattr(fake, name, types.MethodType(fn, fake))
    return fake


# ============================================================
# 独立真值: 光线投射光栅化 (与实现完全不同的方法)
# ============================================================
def raycast_bbox(fake, bx, by, bz, r, length):
    """对每个像素投射射线, 与"有限圆柱"求交, 返回命中像素的外接框 (真值).

    命中 = 与侧壁在 z 范围内相交  ∪  与顶盖圆盘相交.
    """
    W, H = fake.img_w, fake.img_h
    U, V = np.meshgrid(np.arange(W, dtype=np.float64),
                       np.arange(H, dtype=np.float64))
    right = (U - W / 2.0) / fake.fx
    fwd = (H / 2.0 - V) / fake.fy
    c, s = math.cos(fake.heading), math.sin(fake.heading)
    dx = fwd * c - right * s
    dy = fwd * s + right * c
    dz = 1.0                                  # 参数化深度 = t

    ox = fake.drone_x - bx
    oy = fake.drone_y - by
    a = dx * dx + dy * dy
    b = 2.0 * (ox * dx + oy * dy)
    cq = ox * ox + oy * oy - r * r
    disc = b * b - 4.0 * a * cq
    hit_side = disc > 0.0
    disc = np.maximum(disc, 0.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        t_side = (-b - np.sqrt(disc)) / (2.0 * a)
    z_top, z_bot = bz - length / 2.0, bz + length / 2.0
    z_at = fake.drone_z + t_side * dz
    ok_side = hit_side & (t_side > 0.0) & (z_at >= z_top) & (z_at <= z_bot)

    # 顶盖 (相机在上方, 只有顶盖可能被直接命中)
    t_cap = z_top - fake.drone_z
    ok_cap = np.zeros_like(ok_side)
    if t_cap > 0.0:
        px = fake.drone_x + t_cap * dx
        py = fake.drone_y + t_cap * dy
        ok_cap = ((px - bx) ** 2 + (py - by) ** 2) <= r * r

    hit = ok_side | ok_cap
    if not hit.any():
        return None
    vs, us = np.nonzero(hit)
    return (int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))


# ============================================================
# A 节 — 回归: 默认 ground 模式输出逐位不变
# ============================================================
def section_a():
    print("\n[A] 回归 — project_mode='ground' 与改前备份逐位相同")
    old = load_methods(DET_OLD, {"_tick_sim", "_publish_gt_dets"})
    new = load_methods(DET, {"_tick_sim", "_publish_gt_boxes",
                             "_gt_ground_circles", "_circles_to_boxes"})

    cases = []
    for h in (1.2, 1.5, 2.5, 5.0):
        for rho in (0.0, 0.2, 0.5, 1.0, 2.0):
            for hdg in (0.0, math.radians(92.0)):
                cases.append((h, rho, hdg))

    diff = 0
    for h, rho, hdg in cases:
        buckets = [(0.0, rho, BUCKET_NED_Z), (3.0, rho - 1.0, BUCKET_NED_Z)]
        radii = [BUCKET_R, BUCKET_R]
        drone = (0.0, 0.0, -h)

        fo = bind(FakeDet("ground", buckets, radii, drone, hdg), old)
        old["_tick_sim"](fo)

        fn = bind(FakeDet("ground", buckets, radii, drone, hdg), new)
        new["_tick_sim"](fn)

        if fo.captured != fn.captured:
            diff += 1
            if diff <= 2:
                print(f"    差异 H={h} rho={rho} hdg={hdg:.3f}\n      old={fo.captured}\n      new={fn.captured}")

    check(diff == 0, f"{len(cases)} 个位姿下新旧 ground 输出逐位相同 (差异 {diff} 例)")
    check(len([1 for c in cases]) > 0, f"用例覆盖: {len(cases)} 个 (H×离轴×航向)")


# ============================================================
# B 节 — 真值: silhouette 与光线投射一致
# ============================================================
def section_b():
    print("\n[B] 真值 — project_mode='silhouette' 与独立光线投射一致")
    ns = load_methods(DET, {"_gt_boxes_silhouette"})

    def boxes_of(fake):
        return ns["_gt_boxes_silhouette"](fake)

    cases = [
        ("正上方 H=1.2", 1.2, 0.0, 0.0),
        ("偏右 0.10 H=1.2", 1.2, 0.10, 0.0),
        ("偏右 0.20 H=1.2", 1.2, 0.20, 0.0),
        ("偏右 0.50 H=1.2", 1.2, 0.50, 0.0),
        ("偏右 1.00 H=1.2", 1.2, 1.00, 0.0),
        ("偏前 0.30 H=1.5", 1.5, 0.30, math.pi / 2),
        ("偏右 0.50 H=2.5", 2.5, 0.50, 0.0),
        ("偏右 1.00 H=5.0", 5.0, 1.00, 0.0),
    ]
    worst = 0
    for label, h, rho, hdg in cases:
        buckets = [(0.0, rho, BUCKET_NED_Z)]
        radii = [BUCKET_R]
        drone = (0.0, 0.0, -h)
        fake = FakeDet("silhouette", buckets, radii, drone, hdg)
        bx = boxes_of(fake)
        truth = raycast_bbox(fake, buckets[0][0], buckets[0][1], BUCKET_NED_Z,
                             BUCKET_R, BUCKET_L)
        if not bx or truth is None:
            check(False, f"{label}: 无框 (impl={bx} truth={truth})")
            continue
        got = bx[0]
        d = max(abs(got[i] - truth[i]) for i in range(4))
        worst = max(worst, d)
        check(d <= 2, f"{label}: impl={got} truth={truth} 最大边差 {d}px")

    print(f"  → 全部用例最大边差 {worst}px (采样多边形内接 + 取整的预期量级 ≤1px)")

    # 证明"只采两圆足够": 独立地稠密采样【整个圆柱表面】(含侧壁), bbox 不应更大
    def dense_surface_bbox(fake, bx, by, bz, r, length, nh=80, nk=180):
        c, s = math.cos(fake.heading), math.sin(fake.heading)
        xs, ys = [], []
        zs = [bz - length / 2.0] + [bz - length / 2.0 + length * i / (nh - 1)
                                    for i in range(nh)] + [bz + length / 2.0]
        for zc in zs:
            for k in range(nk):
                a = 2.0 * math.pi * k / nk
                dz = zc - fake.drone_z
                if dz <= 0.05:
                    continue
                dx = bx + r * math.cos(a) - fake.drone_x
                dy = by + r * math.sin(a) - fake.drone_y
                xs.append(fake.img_w / 2.0 + ((-dx * s + dy * c) / dz) * fake.fx)
                ys.append(fake.img_h / 2.0 - ((dx * c + dy * s) / dz) * fake.fy)
        return (min(xs), min(ys), max(xs), max(ys))

    buckets = [(0.0, 0.5, BUCKET_NED_Z)]
    radii = [BUCKET_R]
    drone = (0.0, 0.0, -1.2)
    f_rim = FakeDet("silhouette", buckets, radii, drone, 0.0)
    b_rim = boxes_of(f_rim)
    tb = dense_surface_bbox(f_rim, 0.0, 0.5, BUCKET_NED_Z, BUCKET_R, BUCKET_L)
    dh = max(abs(b_rim[0][i] - tb[i]) for i in range(4))
    check(dh <= 2, f"整表面稠密采样 bbox 与两圆采样 bbox 一致 (最大边差 {dh}px)")

    # 两种模式给任务节点的框中心必须【不同】, 且差异随离轴距离增大
    ns_g = load_methods(DET, {"_gt_ground_circles", "_circles_to_boxes",
                              "_gt_boxes_silhouette"})
    print("  ── 两种模式的框中心差异 (H=1.2) ──")
    prev = -1.0
    monotone = True
    for rho in (0.0, 0.10, 0.20, 0.50, 1.00):
        bk = [(0.0, rho, BUCKET_NED_Z)]
        rd = [BUCKET_R]
        dr = (0.0, 0.0, -1.2)
        fg = FakeDet("ground", bk, rd, dr, 0.0)
        fs = FakeDet("silhouette", bk, rd, dr, 0.0)
        bg = ns_g["_circles_to_boxes"](fg, ns_g["_gt_ground_circles"](fg))[0]
        bs = ns_g["_gt_boxes_silhouette"](fs)[0]
        cg = ((bg[0] + bg[2]) / 2.0, (bg[1] + bg[3]) / 2.0)
        cs = ((bs[0] + bs[2]) / 2.0, (bs[1] + bs[3]) / 2.0)
        d = math.hypot(cg[0] - cs[0], cg[1] - cs[1])
        print(f"     rho={rho*100:5.1f}cm  地面中心=({cg[0]:6.1f},{cg[1]:6.1f})  "
              f"轮廓中心=({cs[0]:6.1f},{cs[1]:6.1f})  差 {d:5.1f}px")
        if d < prev - 1e-9:
            monotone = False
        prev = d
    check(monotone and prev > 1.0,
          f"两种模式的框中心不同且差异随离轴单调增 (最大 {prev:.1f}px)")


# ============================================================
# C 节 — 量化: 任务节点反投影落点误差
# ============================================================
def section_c():
    print("\n[C] 量化 — 任务节点(真代码路径)反投影的落点误差")
    geo = load_methods(DET, {"_gt_boxes_silhouette", "_gt_ground_circles", "_circles_to_boxes"})
    cam = load_geometry(NODE)

    def sim_boxes(mode, buckets, radii, drone, hdg):
        fake = FakeDet(mode, buckets, radii, drone, hdg)
        if mode == "silhouette":
            return geo["_gt_boxes_silhouette"](fake)
        return geo["_circles_to_boxes"](fake, geo["_gt_ground_circles"](fake))

    def node_estimate(box, home, drone, hdg, plane_h=0.0):
        """真 _roi_to_ned 的等价调用 (复用 i.py 的 camera_pixel_to_level)。

        plane_h = 目标平面在"起飞原点平面"【下方】多少米 (即离相机更远多少) (m):
            0.0   = 现行 i/j/k 行为 (求交在起飞原点平面)
            0.455 = SITL 桶口平面 (桶顶圆环 NED z=0.455, home_z≈0) ⇒ 对应 P1 的 TARGET_PLANE_H
        """
        m = FakeMission(home, drone, hdg)
        H = m.home_z - m.z
        if H < 0.3:
            return None
        H_eff = H + plane_h
        if H_eff < 0.3:
            return None
        cx = box[0] + (box[2] - box[0]) / 2.0
        cy = box[1] + (box[3] - box[1]) / 2.0
        p = cam["camera_pixel_to_level"](cx, cy, m._pitch, m._roll)
        if p is None:
            return None
        cf, cr = (v * H_eff for v in p)
        dx, dy, _ = m._body_to_ned(cf, cr)
        return (m.x - m.home_x + dx, m.y - m.home_y + dy)

    for axis, name in ((1, "横向(机体右)"), (0, "纵向(机体前)")):
        print(f"\n  ── {name}: 桶离轴 ρ, 飞机高 H (heading=0, home_z=0) ──")
        print(f"  {'H(m)':>5} {'rho(cm)':>8} {'地面模式误差':>12} {'轮廓模式误差':>12} {'轮廓/真值':>10}")
        rows = []
        for h in (1.2, 1.5, 2.5, 5.0):
            for rho in (0.05, 0.10, 0.20, 0.30, 0.50, 1.00):
                # heading=0 ⇒ 机体前 = NED x, 机体右 = NED y
                bxy = [0.0, 0.0]
                bxy[axis] = rho
                buckets = [(bxy[0], bxy[1], BUCKET_NED_Z)]
                radii = [BUCKET_R]
                home = (0.0, 0.0, 0.0)
                drone = (0.0, 0.0, -h)
                truth = (buckets[0][0] - drone[0], buckets[0][1] - drone[1])
                row = {"h": h, "rho": rho, "g": None, "s": None,
                       "clip_g": False, "clip_s": False}
                for key, mode in (("g", "ground"), ("s", "silhouette")):
                    bs = sim_boxes(mode, buckets, radii, drone, 0.0)
                    if not bs:
                        continue
                    b = bs[0]
                    row["clip_" + key] = (b[0] <= 0 or b[2] >= IMG_W - 1
                                          or b[1] <= 0 or b[3] >= IMG_H - 1)
                    est = node_estimate(b, home, drone, 0.0)
                    if est is None:
                        continue
                    off = (est[0] - (drone[0] - home[0]), est[1] - (drone[1] - home[1]))
                    row[key] = math.hypot(off[0] - truth[0], off[1] - truth[1])
                rows.append(row)
                g_e, s_e = row["g"], row["s"]
                pct = (s_e / rho * 100.0) if (s_e is not None and rho > 0) else float("nan")
                print(f"  {h:5.1f} {rho*100:8.1f} "
                      f"{(f'{g_e*100:11.1f}cm' if g_e is not None else '        n/a')} "
                      f"{(f'{s_e*100:11.1f}cm' if s_e is not None else '        n/a')} "
                      f"{pct:9.1f}%")

        # (i) 未裁剪框: 地面模式往返误差只剩【取整量化】⇒ 印证仿真对该类误差结构性失明
        unclip = [r for r in rows if r["g"] is not None and not r["clip_g"]]
        tol = 1.2 / cam["FX"]                      # 1px @H=1.2 对应的地面米数
        gmax = max((r["g"] for r in unclip), default=None)
        check(gmax is not None and gmax <= 1.5 * 1.2 / cam["FX"],
              f"{name}: 未裁剪框时地面模式往返误差 ≤ {gmax*100:.2f}cm "
              f"(1 像素量化 = {tol*100:.2f}cm) ⇒ SITL 对该类误差失明")
        clipped = [r for r in rows if r["clip_g"]]
        if clipped:
            print(f"     (另有 {len(clipped)} 例地面框被图像边界裁剪, 其中心偏移是裁剪伪影, 不计入)")

        # (ii) 轮廓模式在投放高度暴露大误差
        near = [r for r in rows if abs(r["h"] - 1.2) < 1e-9 and r["s"] is not None]
        smax_near = max(r["s"] for r in near)
        check(smax_near > 0.10, f"{name}: H=1.2m 时轮廓模式暴露最大 {smax_near*100:.1f}cm 误差 (>10cm)")

        # (iii) 同一工况下 轮廓/地面 误差比 (失明程度的直接度量)
        ratio_worst = 0.0
        for r in rows:
            if r["g"] is not None and r["s"] is not None and r["g"] > 1e-9:
                ratio_worst = max(ratio_worst, r["s"] / r["g"])
        check(ratio_worst > 20.0,
              f"{name}: 轮廓/地面 误差比最高 {ratio_worst:.0f}× ⇒ 旧仿真看不见的量被放大出来")

        # (iv) 每个高度内, 轮廓模式误差随 ρ 单调增 (比例型 ⇒ 闭环到零即消失)
        #      容差 = 该高度的 2 像素量化 (低空 1px≈0.34cm, H=5m 1px≈1.4cm)
        mono_all = True
        for h in (1.2, 1.5, 2.5, 5.0):
            grp = [r for r in rows if abs(r["h"] - h) < 1e-9 and r["s"] is not None]
            vals = [r["s"] for r in grp]
            q = 2.0 * h / cam["FX"]
            if not all(vals[i] <= vals[i + 1] + q for i in range(len(vals) - 1)):
                mono_all = False
                print(f"     非单调: H={h} vals={[round(v*100,2) for v in vals]}")
        check(mono_all, f"{name}: 各高度内误差随离轴距离单调增(容差2px) ⇒ 比例型 (闭环到零即消失)")


# ============================================================
# D 节 — 整条发布链 (进位置回调 → tick → 发布 JSON)
# ============================================================
def section_d():
    print("\n[D] 整条发布链 — _cb_local_sim → _tick_sim → _publish_gt_boxes (真代码路径)")
    ns = load_methods(DET, {"_cb_local_sim", "_tick_sim", "_publish_gt_boxes",
                            "_gt_ground_circles", "_circles_to_boxes",
                            "_gt_boxes_silhouette"})

    class Msg:
        def __init__(self, x, y, z, h):
            self.x, self.y, self.z, self.heading = x, y, z, h

    # 桶1 NED(-1.50, 31.50, 0.58) / 桶2 NED(-1.50, 30.50, 0.58);
    # 飞机 (-1.50, 31.00, -1.2) heading=0 ⇒ 桶1 在机体右 0.5m, 桶2 在机体左 0.5m (都在视野内)
    buckets = [(-1.50, 31.50, BUCKET_NED_Z), (-1.50, 30.50, BUCKET_NED_Z)]
    radii = [BUCKET_R, BUCKET_R]

    for mode, expect_note in (("ground", "旧行为"), ("silhouette", "真实轮廓")):
        f = bind(FakeDet(mode, buckets, radii, (0.0, 0.0, 0.0), 0.0), ns)
        f._cb_local_sim(Msg(-1.50, 31.00, -1.2, 0.0))     # 真回调
        f.heading = f.heading                              # noqa (回调已写入)
        f._tick_sim()                                      # 真 tick + 真发布
        ok = len(f.captured) == 1 and len(f.captured[0]) == 2
        check(ok, f"{mode}: tick 发布 1 条 JSON, 含 2 个桶 ({expect_note})")
        if not ok:
            continue
        b0 = f.captured[0][0]
        got = (b0["x"], b0["y"], b0["x"] + b0["w"], b0["y"] + b0["h"])
        # 与离线预测交叉核对
        direct = (ns["_gt_boxes_silhouette"](f)[0] if mode == "silhouette"
                  else ns["_circles_to_boxes"](f, ns["_gt_ground_circles"](f))[0])
        check(got == direct, f"{mode}: 发布链输出 == 离线预测 {got}")

    # 已知几何: 轮廓模式的框中心必然比地面模式更靠近画面中心 (深度更大)
    fg = bind(FakeDet("ground", buckets, radii, (0.0, 0.0, 0.0), 0.0), ns)
    fg._cb_local_sim(Msg(-1.50, 31.00, -1.2, 0.0)); fg._tick_sim()
    fs = bind(FakeDet("silhouette", buckets, radii, (0.0, 0.0, 0.0), 0.0), ns)
    fs._cb_local_sim(Msg(-1.50, 31.00, -1.2, 0.0)); fs._tick_sim()
    cg = fg.captured[0][0]; cs = fs.captured[0][0]
    cxg = cg["x"] + cg["w"] / 2.0
    cxs = cs["x"] + cs["w"] / 2.0
    check(cxs < cxg, f"轮廓框中心 {cxs:.1f}px 比地面框中心 {cxg:.1f}px 更靠画面中心 "
                     f"(深度更大 ⇒ 投影更小)")


# ============================================================
# E 节 — P1 预演: 节点求交平面改"桶口"的确定性收益
# ============================================================
def section_e():
    """用【同一套独立真值】量 P1 的收益: 节点交面从"起飞原点平面"抬到"桶口平面"。

    SITL 几何: 桶顶圆环 NED z = 0.455, home_z ≈ 0 ⇒ 桶口平面比"起飞原点平面"低 0.455m
    (即离相机更远 0.455m)。所以 plane_h = 0.455 就是"节点与仿真认同一个物理平面"的取值。
    """
    print("\n[E] P1 预演 — 任务节点求交平面 plane_h: 0(现行) vs 0.455(SITL 桶口)")
    geo = load_methods(DET, {"_gt_boxes_silhouette", "_gt_ground_circles", "_circles_to_boxes"})
    cam = load_geometry(NODE)
    # h = 相机落地高 − 桶口离地高; SITL: 0.83 − 0.375 = +0.455
    PLANE_NOW, PLANE_RIM = 0.0, 0.455

    def sim_boxes(mode, buckets, radii, drone, hdg):
        fake = FakeDet(mode, buckets, radii, drone, hdg)
        if mode == "silhouette":
            return geo["_gt_boxes_silhouette"](fake)
        return geo["_circles_to_boxes"](fake, geo["_gt_ground_circles"](fake))

    def est(box, home, drone, hdg, plane_h):
        m = FakeMission(home, drone, hdg)
        H = m.home_z - m.z
        H_eff = H + plane_h
        if H < 0.3 or H_eff < 0.3:
            return None
        cx = box[0] + (box[2] - box[0]) / 2.0
        cy = box[1] + (box[3] - box[1]) / 2.0
        p = cam["camera_pixel_to_level"](cx, cy, m._pitch, m._roll)
        if p is None:
            return None
        cf, cr = (v * H_eff for v in p)
        dx, dy, _ = m._body_to_ned(cf, cr)
        return (m.x - m.home_x + dx, m.y - m.home_y + dy)

    home = (0.0, 0.0, 0.0)
    print(f"  {'H(m)':>5} {'ρ(cm)':>7} {'现行(0)':>11} {'桶口(0.455)':>13} {'改善':>8}")
    worst_now = worst_rim = 0.0
    pairs = []
    for h in (1.2, 1.5, 2.5, 5.0):
        for rho in (0.05, 0.10, 0.20, 0.30, 0.50):
            buckets = [(0.0, rho, BUCKET_NED_Z)]
            radii = [BUCKET_R]
            drone = (0.0, 0.0, -h)
            truth = (0.0, rho)                       # 桶真值相对飞机的 NED 偏移 (桶在 NED y=rho)
            bs = sim_boxes("silhouette", buckets, radii, drone, 0.0)
            if not bs:
                continue
            res = {}
            for key, ph in (("now", PLANE_NOW), ("rim", PLANE_RIM)):
                e = est(bs[0], home, drone, 0.0, ph)
                if e is None:
                    res[key] = None
                    continue
                off = (e[0] - drone[0], e[1] - drone[1])
                res[key] = math.hypot(off[0] - truth[0], off[1] - truth[1])
            if res["now"] is None or res["rim"] is None:
                continue
            gain = res["now"] / res["rim"] if res["rim"] > 1e-9 else float("inf")
            pairs.append((h, rho, res["now"], res["rim"], gain))
            if abs(h - 1.2) < 1e-9:
                worst_now = max(worst_now, res["now"])
                worst_rim = max(worst_rim, res["rim"])
            print(f"  {h:5.1f} {rho*100:7.1f} {res['now']*100:10.2f}cm "
                  f"{res['rim']*100:12.2f}cm {gain:7.1f}×")

    check(worst_rim < worst_now / 3.0,
          f"H=1.2m 最坏残差 {worst_now*100:.1f}cm → {worst_rim*100:.1f}cm "
          f"(改善 {worst_now/max(worst_rim,1e-9):.1f}×)")
    # 关键: 桶口平面下, 小离轴时残差应落在厘米以内 (这才是 P1 的真实收益量级)
    small = [row for row in pairs if abs(row[0] - 1.2) < 1e-9 and row[1] <= 0.20]
    check(small and max(row[3] for row in small) < 0.03,
          f"H=1.2m 且 ρ≤20cm 时残差 ≤3cm (实测最大 "
          f"{max(row[3] for row in small)*100:.2f}cm)")
    # 残余项 = 可见侧壁把 bbox 中心拉偏 (不随 plane_h 消失), 应随 ρ 单调增
    rim_rows = [r for r in pairs if abs(r[0] - 1.2) < 1e-9]
    rim_rows.sort(key=lambda r: r[1])
    mono = all(rim_rows[i][3] <= rim_rows[i + 1][3] + 0.005 for i in range(len(rim_rows) - 1))
    check(mono, f"桶口平面下的残余仍随 ρ 单调增 (可见侧壁二阶项, 不随 plane_h 消失)")


if __name__ == "__main__":
    print("=" * 72)
    print("P0 离线验收 — 仿真检测器真实圆柱轮廓投影 (2026-10-09)")
    print(f"  新: {DET.relative_to(ROOT)}")
    print(f"  旧: {DET_OLD.name}")
    print("=" * 72)
    section_a()
    section_b()
    section_c()
    section_d()
    section_e()
    print("\n" + "=" * 72)
    print(f"结果: {_ok} 项通过 / {_fail} 项失败")
    print("=" * 72)
    sys.exit(1 if _fail else 0)
