#!/usr/bin/env python3
"""现场静态标定 `TARGET_PLANE_H` (2026-10-09)

原理 (一句话)
-------------
`_roi_to_ned` 做的是"像素射线 × H 求横向偏移", 其中 H = home_z − z。
而像素真正来自**桶口平面**, 深度是 H + h。所以:

    est = ρ_true · H / (H + h)      ⇒      h = H · (ρ_true/est − 1)

h 是**硬件常数** (= 解锁位置处相机光心离地高 − 桶口离地高), 与飞行高度无关。
本工具不用节点、不用解锁、不用 OFFBOARD —— 只要**检测器在跑** + 一把卷尺。

为什么不用扒节点日志
--------------------
`_roi_to_ned` 依赖 `self.home_z`, 而 `home_z` 只在 **STABILIZE(起飞后)** 才赋值
(见 k.py `_takeoff_*` 在首帧捕获、`home_*` 在 STABILIZE 赋值) ⇒ **飞机在地面时反投影恒为 None**。
故静态标定必须自己算 —— 本工具复用**节点里同一个 `camera_pixel_to_level`**, 保证与飞行同模型。

用法
----
  # 0) 离线自检 (不需要 ROS, 不需要飞机)
  python3 scripts/calibrate_plane_h.py --selftest

  # 1) 检测器起来 (只需检测器, 不需要 ARM/OFFBOARD)
  ros2 run fly see                       # 实机
  # 或 ros2 run drone_drl yolov8_bucket_detector --ros-args -p use_sim:=true -p bucket_count:=3 ...

  # 2) 飞机水平架高 H, 桶摆在相机正下方【横向】偏开 rho
  python3 scripts/calibrate_plane_h.py --H 1.20 --rho 0.30 --duration 5

  # 3) 至少三个偏移量交叉验证
  python3 scripts/calibrate_plane_h.py --H 1.20 --rho 0.20 --duration 5
  python3 scripts/calibrate_plane_h.py --H 1.20 --rho 0.40 --duration 5

判读见 docs/reports/calibration_to_code_20261009.md。
"""
import argparse
import ast
import math
import statistics
import sys
import textwrap
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_camera_model(node="k"):
    """从节点文件里取出【同一个】camera_pixel_to_level 与相机常量。"""
    # 路径可移植: 笔记本用前者, 板上用后者 (工具只需这一个文件)
    cands = [ROOT / f"src/drone_drl/drone_drl/{node}.py",
             ROOT / f"fly/src/fly/fly/{node}.py",
             Path.home() / f"fly/src/fly/fly/{node}.py"]
    path = next((c for c in cands if c.exists()), None)
    if path is None:
        sys.exit("找不到节点文件, 试过:\n  " + "\n  ".join(str(c) for c in cands))
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math}
    want = {"IMG_W", "IMG_H", "CX", "CY", "HFOV", "VFOV", "FX", "FY"}
    for n in tree.body:
        if isinstance(n, ast.Assign):
            names = [s.id for t in n.targets for s in ast.walk(t) if isinstance(s, ast.Name)]
            if names and all(x in want for x in names):
                exec(ast.get_source_segment(src, n), ns)
    missing = want - set(ns)
    assert not missing, f"{path.name} 缺相机常量 {sorted(missing)}"
    for n in tree.body:
        if isinstance(n, ast.FunctionDef) and n.name == "camera_pixel_to_level":
            exec(textwrap.dedent(ast.get_source_segment(src, n)), ns)
            break
    else:
        sys.exit("没找到 camera_pixel_to_level")
    ns["_src"] = path
    return ns


def solve(points, H, rho_right, rho_fwd, pitch, roll, ns):
    """points = [(cx, cy, w, h), ...] → 标定结果 dict。"""
    cx = statistics.median(p[0] for p in points)
    cy = statistics.median(p[1] for p in points)
    p = ns["camera_pixel_to_level"](cx, cy, pitch, roll)
    if p is None:
        return None
    est_fwd, est_right = p[0] * H, p[1] * H

    def h_from(rho, est):
        if abs(est) < 1e-6:
            return None
        return H * (rho / est - 1.0)

    res = {"cx": cx, "cy": cy, "est_right": est_right, "est_fwd": est_fwd,
           "n": len(points), "CX": ns["CX"], "CY": ns["CY"]}
    # 像素散布 (px) —— 稳定性指标
    if len(points) > 1:
        res["mad_cx"] = statistics.median([abs(q[0] - cx) for q in points])
        res["mad_cy"] = statistics.median([abs(q[1] - cy) for q in points])
    if rho_right:
        res["h_right"] = h_from(rho_right, est_right)
        res["resid_right"] = est_right - rho_right
    if rho_fwd:
        res["h_fwd"] = h_from(rho_fwd, est_fwd)
        res["resid_fwd"] = est_fwd - rho_fwd
    return res


def report(res, H, rho_right, rho_fwd, thresh=0.01):
    print("\n" + "=" * 76)
    print(f"  标定结果   H = {H:.3f} m   ρ_true = (前 {rho_fwd:+.3f}, 右 {rho_right:+.3f}) m")
    print("=" * 76)
    print(f"  采样 {res['n']} 帧   框中位 px = ({res['cx']:.1f}, {res['cy']:.1f})"
          f"   画面中心 = ({res['CX']:.0f}, {res['CY']:.0f})")
    if "mad_cx" in res:
        print(f"  像素散布 MAD = ({res['mad_cx']:.2f}, {res['mad_cy']:.2f}) px"
              f"   {'✓ 稳' if max(res['mad_cx'], res['mad_cy']) < 2.0 else '✗ 不稳, 先解决检测'}")
    print(f"\n  反投影 est = (前 {res['est_fwd']:+.3f}, 右 {res['est_right']:+.3f}) m")
    print(f"  残差 est − ρ_true = "
          f"(前 {res.get('resid_fwd', float('nan'))*100:+6.2f}, "
          f"右 {res.get('resid_right', float('nan'))*100:+6.2f}) cm")
    print("\n  ── 反解 TARGET_PLANE_H ──")
    for axis, key in (("横向(右)", "h_right"), ("纵向(前)", "h_fwd")):
        if key in res and res[key] is not None:
            print(f"    {axis}:  TARGET_PLANE_H = {res[key]:+.4f} m")
    h = res.get("h_right") or res.get("h_fwd")
    if h is not None:
        print(f"\n  ⇒ 建议填入 k.py / l.py 的 TARGET_PLANE_H = {h:+.3f} m")
        print(f"     (SITL 侧跑 silhouette 仿真时用 0.455; 实机侧用这个值 —— 两边本来就该不同)")
    # 诊断
    print("\n  ── 自检提示 ──")
    if rho_fwd == 0.0 and abs(res["est_fwd"]) > 0.05:
        print(f"    ⚠ 桶放在正侧方(前向 0), 但反投影出 {res['est_fwd']*100:+.1f}cm 的前向分量")
        print(f"      ⇒ 可能存在【相机安装 yaw/俯仰偏角】, 属另一个参数(XF), 不要混进 TARGET_PLANE_H")
    if abs(res["est_right"]) > 0.9 * H:
        print(f"    ⚠ 离轴已接近视场边缘, 反投影非线性误差变大 ⇒ 把 ρ 调小一点再测")
    print("=" * 76 + "\n")


def selftest():
    """离线自检: 造一个已知 h 的场景, 看能否原样反解回来。"""
    ns = load_camera_model("k")
    ok = fail = 0
    print("=" * 76)
    print(f"离线自检 (相机模型来自 {ns['_src'].name})")
    print("=" * 76)
    for h_true, H, rho_r, rho_f in ((-0.12, 1.20, 0.30, 0.0),
                                    (0.455, 1.20, 0.30, 0.0),
                                    (-0.20, 1.50, 0.20, 0.10),
                                    (0.00, 1.20, 0.40, 0.0)):
        # 正向: 已知 h → 桶在相机坐标下的真值 → 像素 (pitch=roll=0, 纯侧向)
        t = H + h_true
        fwd_ratio, right_ratio = rho_f / t, rho_r / t
        cx = ns["CX"] + ns["FX"] * right_ratio
        cy = ns["CY"] - ns["FY"] * fwd_ratio
        res = solve([(cx, cy, 20.0, 20.0)], H, rho_r, rho_f, 0.0, 0.0, ns)
        h_back = res["h_right"]
        good = abs(h_back - h_true) < 1e-6
        ok, fail = (ok + 1, fail) if good else (ok, fail + 1)
        print(f"  {'✓' if good else '✗'} h_true={h_true:+.3f} H={H} ρ=(前{rho_f},右{rho_r}) "
              f"→ 反解 {h_back:+.6f}  (差 {abs(h_back-h_true)*1e6:.1f}µm)")
    print(f"\n结果: {ok} 通过 / {fail} 失败\n" + "=" * 76)
    return 1 if fail else 0


def _on_multi(msg, buf):
    import json
    try:
        for b in json.loads(msg.data).get("buckets", []):
            buf.append((b["x"] + b["w"] / 2.0, b["y"] + b["h"] / 2.0, b["w"], b["h"]))
    except Exception:
        pass


def _on_roi(msg, buf):
    buf.append((msg.x_offset + msg.width / 2.0, msg.y_offset + msg.height / 2.0,
                msg.width, msg.height))


def _filt(pts):
    if not pts:
        return pts
    med_w = statistics.median(p[2] for p in pts)
    keep = [p for p in pts if 0.4 * med_w <= p[2] <= 2.5 * med_w]
    return keep or pts


def _med(pts):
    return (statistics.median(p[0] for p in pts), statistics.median(p[1] for p in pts),
            statistics.median(p[2] for p in pts), statistics.median(p[3] for p in pts))


def _mount(ns, pts, H):
    cx, cy, w, h = _med(pts)
    dcx, dcy = cx - ns["CX"], cy - ns["CY"]
    print("\n" + "=" * 76)
    print(f"  安装角测量   H = {H:.3f} m   桶/标记应放在【相机光心铅垂线】正下方")
    print("=" * 76)
    print(f"  框中位 px = ({cx:.1f}, {cy:.1f})   画面中心 = ({ns['CX']:.0f}, {ns['CY']:.0f})")
    print(f"  Δ = ({dcx:+.1f}, {dcy:+.1f}) px     宽中位 = {w:.1f}px")
    print(f"\n  ⇒ 等效安装 yaw   ≈ atan(Δcx/FX) = {math.degrees(math.atan(dcx / ns['FX'])):+.2f}°")
    print(f"  ⇒ 等效安装 pitch ≈ atan(Δcy/FY) = {math.degrees(math.atan(dcy / ns['FY'])):+.2f}°")
    print("\n  判读:")
    print("    |Δ| < 5px  ⇒ 相机基本下视且主点居中, 这项可忽略")
    print("    |Δ| ≥ 5px  ⇒ **存在固定的等效像素偏置** ⇒ 反投影会产生 H·tan(δ) 的偏移")
    print(f"                  H=1.2m 时该偏移约 (前 {math.tan(math.radians(math.degrees(math.atan(dcy/ns['FY']))))*H*100:+.1f}cm, "
          f"右 {math.tan(math.radians(math.degrees(math.atan(dcx/ns['FX']))))*H*100:+.1f}cm)")
    print("    再换一个高度复测: 若 Δ 不变 ⇒ 固定偏置 (可标定); 若随 H 变 ⇒ 还有别的项")
    print("=" * 76 + "\n")


def _two_height(ns, r1, r2, c1, c2, rho1, rho2, A, R_tape, D):
    """两次不同高度 ⇒ 反解【桶口离地高 R】与【真实 FX】。

    物理量全部从地面卷尺量 (不涉及"解锁位置平面"):
        像素偏移 Δcx = FX·ρ/(C − R)     C = 相机镜头前端离地高, R = 桶口离地高
    两高度联立 (允许两次的 ρ 不同 ⇒ 相机水平位置不必严格一致):
        R = (Δcx2·C2/ρ2 − Δcx1·C1/ρ1) / (Δcx2/ρ2 − Δcx1/ρ1)
        FX = Δcx1·(C1 − R)/ρ1
    """
    cx1, cy1, w1, _ = _med(r1)
    cx2, cy2, w2, _ = _med(r2)
    u, v = cx1 - ns["CX"], cx2 - ns["CY"] * 0 - ns["CX"]
    print("\n" + "=" * 78)
    print("  两高度法 —— 定 桶口高 R 与 真实 FX")
    print("=" * 78)
    print(f"  第1次 C1={c1:.3f}m  ρ1={rho1:.3f}m   框心 px=({cx1:.1f},{cy1:.1f})  Δcx={u:+.1f}px  宽={w1:.1f}px")
    print(f"  第2次 C2={c2:.3f}m  ρ2={rho2:.3f}m   框心 px=({cx2:.1f},{cy2:.1f})  Δcx={v:+.1f}px  宽={w2:.1f}px")

    den = (v / rho2) - (u / rho1)
    if abs(den) < 1e-9 or abs(u) < 2.0:
        print("  ✗ 两次的 Δcx 太接近 ⇒ 高度差太小 / 桶太靠近光轴 ⇒ 加大高度差或加大 ρ 重做")
        return
    R_fit = ((v * c2 / rho2) - (u * c1 / rho1)) / den
    FX = u * (c1 - R_fit) / rho1

    print(f"\n  ── 反解结果 ──")
    print(f"   桶口离地高   R_fit = {R_fit:+.3f} m      (卷尺量 {R_tape:+.3f} m  "
          f"差 {abs(R_fit-R_tape)*100:.1f}cm {'✓ 一致' if abs(R_fit-R_tape) < 0.03 else '✗ 不一致, 重做'})")
    print(f"   真实焦距     FX_true = {FX:.1f} px")
    hfov = math.degrees(2 * math.atan(ns["IMG_W"] / (2 * FX)))
    print(f"   等效 HFOV    = {hfov:.1f}°")
    print(f"   节点现值     FX = {ns['FX']:.1f} px (HFOV=85°)   "
          f"比值 k = FX_true/FX_node = {FX / ns['FX']:.3f}")
    if abs(FX / ns["FX"] - 1.0) > 0.08:
        print(f"\n   ⚠ **内参不一致 (k={FX/ns['FX']:.3f})** ⇒ 应把节点里的 HFOV 从 85 改成 **{hfov:.1f}°**")
        print(f"     (改这一行, FY 会自动跟着 16:9 比例; 然后 TARGET_PLANE_H 再单独标)")
    else:
        print(f"\n   ✓ 内参与现值一致 (±8%) ⇒ 只改 TARGET_PLANE_H 即可")

    print(f"\n  ── TARGET_PLANE_H ──")
    print(f"   用反解 R:  h = A − R_fit = {A:.3f} − {R_fit:.3f} = {A - R_fit:+.4f} m")
    print(f"   用卷尺 R:  h = A − R_tape= {A:.3f} − {R_tape:.3f} = {A - R_tape:+.4f} m")

    if D > 0 and abs(w1 - w2) > 0.5:
        w_den = (w2 / D) - (w1 / D)
        R_w = ((w2 * c2 / D) - (w1 * c1 / D)) / w_den
        FX_w = w1 * (c1 - R_w) / D
        print(f"\n   ── 用框宽独立复核 (D={D:.3f}m) ──")
        print(f"     R_fit = {R_w:+.3f} m    FX_true = {FX_w:.1f} px   "
              f"(两条路线应接近; 差大 ⇒ 框宽不等于桶真实投影宽 ⇒ D 或检测框有问题)")
    print("=" * 78 + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--H", type=float, help="相机光心相对【解锁位置】的高度 (m), 卷尺量")
    ap.add_argument("--c1", type=float, help="两高度法: 第1次【相机镜头前端离地高】(m), 卷尺")
    ap.add_argument("--c2", type=float, help="两高度法: 第2次【相机镜头前端离地高】(m), 卷尺")
    ap.add_argument("--A", type=float, help="飞机放在地面时【相机镜头前端离地高】(m), 卷尺")
    ap.add_argument("--R", type=float, default=0.0, help="【桶口最高一圈离地高】(m), 卷尺")
    ap.add_argument("--rho2", type=float, default=None, help="第二次的 ρ (不给则沿用 --rho)")
    ap.add_argument("--rho", type=float, default=0.0,
                    help="桶心相对相机光轴正下方地面点的【横向】距离 (m), 正=机体右侧")
    ap.add_argument("--fwd", type=float, default=0.0, help="前向距离 (m), 正=机头方向")
    ap.add_argument("--D", type=float, default=0.0, help="桶真实直径 (m), 用于独立复核")
    ap.add_argument("--pitch", type=float, default=0.0, help="机体俯仰 (deg), 水平时填 0")
    ap.add_argument("--roll", type=float, default=0.0, help="机体横滚 (deg), 水平时填 0")
    ap.add_argument("--duration", type=float, default=5.0, help="每个高度采样时长 (s)")
    ap.add_argument("--wait", type=float, default=60.0, help="两高度法: 中间挪动飞机的等待秒数")
    ap.add_argument("--node", default="k", help="从哪个节点文件取相机模型 (k/l/i)")
    ap.add_argument("--selftest", action="store_true", help="离线自检, 不需要 ROS")
    ap.add_argument("--count-only", action="store_true", help="只确认链路上能收到检测框")
    ap.add_argument("--mount", action="store_true", help="安装角模式 (标记放相机正下方)")
    ap.add_argument("--two-height", action="store_true", help="两高度法: 同时定 h 与 FX")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    ns = load_camera_model(args.node)
    print(f"相机模型: {ns['_src'].name}   FX={ns['FX']:.1f} FY={ns['FY']:.1f} "
          f"CX={ns['CX']:.0f} CY={ns['CY']:.0f}")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import RegionOfInterest
    from std_msgs.msg import String

    rclpy.init()
    node = Node("calib_static")
    buf = []
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, depth=10)
    node.create_subscription(String, "/bucket/detections", lambda m: _on_multi(m, buf), qos)
    node.create_subscription(RegionOfInterest, "/bucket/detection", lambda m: _on_roi(m, buf), qos)

    def collect(dur, label):
        buf.clear()
        print(f"\n── {label}: 采样 {dur:.0f}s ──")
        t0, last = time.time(), 0.0
        while rclpy.ok() and time.time() - t0 < dur:
            rclpy.spin_once(node, timeout_sec=0.2)
            el = time.time() - t0
            if el - last >= 1.0:
                last = el
                print(f"  t={el:4.1f}s  已收 {len(buf)} 个框")
        out = _filt(list(buf))
        if not out:
            print("  ✗ 没收到框 —— 检查检测器/话题/桶是否在画面内/光照")
        return out

    try:
        if args.two_height:
            if args.c1 is None or args.c2 is None or args.A is None or args.rho == 0.0:
                ap.error("--two-height 需要 --c1 --c2 --A --rho")
            rho2 = args.rho2 if args.rho2 else args.rho
            p1 = collect(args.duration, f"第1次  C1={args.c1:.2f} m  (相机离地高)")
            if not p1:
                return 1
            print(f"\n>>> 现在把飞机挪到 C2={args.c2:.2f} m（相机离地高）")
            print(f"    ⚠ 用铅垂线确认相机仍在同一根垂线上; 用水平尺确认水平; 桶【不要动】")
            print(f"    等待 {args.wait:.0f}s …")
            t0 = time.time()
            while time.time() - t0 < args.wait:
                time.sleep(1.0)
            p2 = collect(args.duration, f"第2次  C2={args.c2:.2f} m  (相机离地高)")
            if not p2:
                return 1
            _two_height(ns, p1, p2, args.c1, args.c2, args.rho, rho2,
                        args.A, args.R, args.D)
            return 0

        if args.H is None:
            ap.error("需要 --H (除 --selftest / --two-height 外)")
        pts = collect(args.duration, f"H={args.H:.2f} m  ρ=(前{args.fwd:.2f},右{args.rho:.2f}) m")
        if not pts:
            return 1
        if args.count_only:
            cx, cy, w, _ = _med(pts)
            print(f"\n✓ 链路正常: 收到 {len(pts)} 个框, 框中位 px=({cx:.1f},{cy:.1f}) 宽中位={w:.1f}px")
            print(f"  画面中心 ({ns['CX']:.0f},{ns['CY']:.0f}) ⇒ Δ=({cx-ns['CX']:+.1f},{cy-ns['CY']:+.1f})px")
            return 0
        if args.mount:
            _mount(ns, pts, args.H)
            return 0
        if args.rho == 0.0 and args.fwd == 0.0:
            ap.error("需要 --rho 或 --fwd 非零")
        res = solve(pts, args.H, args.rho, args.fwd, math.radians(args.pitch),
                    math.radians(args.roll), ns)
        if res is None:
            sys.exit("反投影返回 None (射线平行/俯仰过大)")
        report(res, args.H, args.rho, args.fwd)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
