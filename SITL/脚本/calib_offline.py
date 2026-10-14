#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定工具 A(离线): 检测器"投影像素"模型 vs 任务节点"_roi_to_ned 反投影"模型的偏差曲线.

目的
----
定位"桶位置算不准"这条误差源. 做法是把两边**真实代码**的数学对接一遍做往返测试:

    真值桶位置 --(检测器投影公式)--> 像素框 --(任务节点 _roi_to_ned)--> 估计位置
                                          |
                                     与真值相减 = 偏差

其中:
  * 投影公式 = yolov8_bucket_detector.py 的 sim 分支 (逐行照抄, 运行时打印出处)
  * 反投影   = 直接从 e.py 源码里抽出来的 **真实** _roi_to_ned / _body_to_ned (不复制逻辑)

扫描维度: 高度 H × 侧偏 R × 俯仰 pitch  → 输出偏差曲线 + CSV.

注意(重要): 仿真检测器是理想针孔模型, **没有镜头畸变**.
  所以本工具量的是"代码/模型层面的系统性偏差", 量不出真实镜头的桶形畸变
  (畸变要用同一套工具 + 实机 see.py 数据来标 -> 见 scripts/calib_refproj.py 的实机用法).
"""

import csv
import math
import os
import textwrap

# ============================================================
# 1) 常量 — 与 e.py / yolov8_bucket_detector.py 完全一致
# ============================================================
IMG_W, IMG_H = 640, 360
CX, CY = IMG_W / 2.0, IMG_H / 2.0
HFOV = math.radians(85.0)
VFOV = math.radians(54.0)
FX = IMG_W / (2.0 * math.tan(HFOV / 2.0))
FY = IMG_H / (2.0 * math.tan(VFOV / 2.0))
BUCKET_R = 0.125                      # 桶半径 (m), 检测器 sim 用的值
H0 = math.radians(92.0)               # 航向 (SITL e.world / test_e.sh 的 HEADING_DEG)
BUCKET_NED = (0.00, 34.00)            # 桶 B3 的真实 NED (仿真世界精确已知 = "卷尺真值")

E_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "src", "drone_drl", "drone_drl", "e.py")
DET_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "src", "drone_drl", "drone_drl", "yolov8_bucket_detector.py")


# ============================================================
# 2) 从 e.py 抽出真实的 _roi_to_ned / _body_to_ned
# ============================================================
def _extract_method(src_text, name):
    i = src_text.index("    def %s(self" % name)
    j = src_text.index("\n    def ", i + 10)
    return textwrap.dedent(src_text[i:j])


def _load_node_methods():
    src = open(E_SRC, encoding="utf-8").read()
    ns = {"math": math, "FX": FX, "FY": FY, "CX": CX, "CY": CY}
    for m in ("_roi_to_ned", "_body_to_ned"):
        exec(_extract_method(src, m), ns)          # noqa: S102 - 故意执行真实源码
    return ns["_roi_to_ned"], ns["_body_to_ned"]


_roi_to_ned, _body_to_ned = _load_node_methods()


class _Stub(object):
    """给真实 _roi_to_ned 提供它需要的属性 (home 设在原点, 于是返回的就是绝对 NED)."""

    # 挂成普通方法: 描述符协议会自动把 self 绑进去, 与 e.py 里的 self._body_to_ned(...) 一致
    _body_to_ned = _body_to_ned

    def __init__(self, x, y, z, heading, pitch, roll=0.0):
        self.x, self.y, self.z = x, y, z
        self.home_x, self.home_y, self.home_z = 0.0, 0.0, 0.0
        self.heading = heading
        self._pitch = pitch
        self._roll = roll


class _ROI(object):
    def __init__(self, x_off, y_off, w, h):
        self.x_offset, self.y_offset, self.width, self.height = x_off, y_off, w, h


# ============================================================
# 3) 检测器投影模型 — 逐行照抄 yolov8_bucket_detector.py sim 分支
#    (源: _tick_sim(), 见文件顶部打印的出处行号)
# ============================================================
def det_project(bucket_ned, drone_x, drone_y, drone_z, heading):
    """返回 (px, py, radius_px) 或 None. 注意: 该模型**不含姿态** (没有 pitch/roll)."""
    bx, by = bucket_ned[0], bucket_ned[1]
    dx = bx - drone_x
    dy = by - drone_y
    dz = -drone_z                            # 飞机→地面
    if dz < 0.3:
        return None
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    body_fwd = dx * cos_h + dy * sin_h
    body_right = -dx * sin_h + dy * cos_h
    px = IMG_W / 2.0 + (body_right / dz) * FX
    py = IMG_H / 2.0 - (body_fwd / dz) * FY
    radius_px = FX * BUCKET_R / dz
    if px < -50 or px > IMG_W + 50 or py < -50 or py > IMG_H + 50:
        return None
    return (px, py, max(3.0, radius_px))


def det_project_source_lines():
    src = open(DET_SRC, encoding="utf-8").read().splitlines()
    keys = ("dz = -self.drone_z", "body_fwd = dx * cos_h", "body_right = -dx * sin_h",
            "px = self.img_w / 2.0", "py = self.img_h / 2.0", "radius_px = self.fx")
    out = []
    for n, line in enumerate(src, 1):
        if any(k in line for k in keys):
            out.append((n, line.strip()))
    return out


# ============================================================
# 4) 往返: 真值 -> 像素 -> 估计
# ============================================================
def place_drone(H, fwd, right, heading=H0):
    """把飞机放在"相对桶 (前方 fwd, 右侧 right)、离地 H"处, 返回飞机 NED."""
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    ox = fwd * cos_h - right * sin_h
    oy = fwd * sin_h + right * cos_h
    # home=(0,0,0) → 飞机 z = -H (NED 向下为正)
    return (BUCKET_NED[0] - ox, BUCKET_NED[1] - oy, -H)


def roundtrip(H, fwd, right, pitch, roll=0.0, quantize=False):
    """真值 -> 检测器像素 -> 真实 _roi_to_ned 反投影. 返回 dict."""
    ax, ay, az = place_drone(H, fwd, right)
    proj = det_project(BUCKET_NED, ax, ay, az, H0)
    if proj is None:
        return None
    px, py, r = proj
    if quantize:                              # 检测器发的是 int 框 (x,y,w,h)
        x_off = int(math.floor(px - r)); y_off = int(math.floor(py - r))
        w = int(round(2 * r)); h = int(round(2 * r))
    else:
        x_off = px - r; y_off = py - r
        w = 2 * r; h = 2 * r
    roi = _ROI(x_off, y_off, w, h)

    stub = _Stub(ax, ay, az, H0, pitch, roll)
    est = _roi_to_ned(stub, roi)
    est0 = None
    if pitch != 0.0:                          # 同一帧数据, 但把俯仰补偿关掉
        est0 = _roi_to_ned(_Stub(ax, ay, az, H0, 0.0, roll), roi)

    err = (est[0] - BUCKET_NED[0], est[1] - BUCKET_NED[1])
    cos_h, sin_h = math.cos(H0), math.sin(H0)
    e_fwd = err[0] * cos_h + err[1] * sin_h
    e_right = -err[0] * sin_h + err[1] * cos_h
    out = {
        "H": H, "fwd": fwd, "right": right, "pitch_deg": math.degrees(pitch),
        "px": px, "py": py, "radius_px": r,
        "px_off_center": px - CX, "py_off_center": py - CY,
        "pct_of_half_fov_h": abs(px - CX) / (IMG_W / 2.0) * 100.0,
        "est": est, "truth": BUCKET_NED,
        "err_fwd": e_fwd, "err_right": e_right,
        "err_mag": math.hypot(*err),
    }
    if est0 is not None:
        out["err_fwd_nopitch"] = (est0[0] - BUCKET_NED[0]) * cos_h + (est0[1] - BUCKET_NED[1]) * sin_h
        out["err_right_nopitch"] = -(est0[0] - BUCKET_NED[0]) * sin_h + (est0[1] - BUCKET_NED[1]) * cos_h
    return out


# ============================================================
# 5) 报告
# ============================================================
def hr(t):
    print("\n" + "=" * 78)
    print(t)
    print("=" * 78)


def main():
    print("离线反投影标定 (工具 A)")
    print("  相机: IMG %dx%d  HFOV %.0f°  VFOV %.0f°  FX=%.2f FY=%.2f CX=%.1f CY=%.1f"
          % (IMG_W, IMG_H, math.degrees(HFOV), math.degrees(VFOV), FX, FY, CX, CY))
    print("  桶真值(卷尺): NED(%.2f, %.2f)  航向 %.0f°  桶半径 %.3fm"
          % (BUCKET_NED[0], BUCKET_NED[1], math.degrees(H0), BUCKET_R))
    print("  反投影: 直接执行 %s 里的 _roi_to_ned (真实代码)" % os.path.normpath(E_SRC))

    hr("检测器投影模型出处 (逐行照抄, 不含姿态)")
    for n, line in det_project_source_lines():
        print("  %s:%d  %s" % (os.path.basename(DET_SRC), n, line))

    rows = []

    # ---- 用例 0: 往返自检 (俯仰=0, 水平悬停) ----
    hr("用例0  俯仰=0 往返自检 (应当 ≈0 → 证明除姿态项外两模型互逆)")
    print("  %-6s %-8s %8s %8s %10s %10s %10s" %
          ("H(m)", "侧偏R(m)", "%半视场", "px-CX", "err_fwd", "err_right", "err_mag"))
    for H in (5.0, 3.0, 2.0, 1.5):
        for R in (0.0, 0.5, 1.0, 2.0, 3.0):
            if abs(R) > 0.64 * H:
                continue
            d = roundtrip(H, 0.0, R, 0.0)
            if d is None:
                continue
            rows.append(d)
            print("  %-6.1f %-8.1f %7.0f%% %8.0f %10.4f %10.4f %10.4f" %
                  (H, R, d["pct_of_half_fov_h"], d["px_off_center"],
                   d["err_fwd"], d["err_right"], d["err_mag"]))

    # ---- 用例 1: 俯仰项 (R=0, H=5m, 扫 pitch) ----
    hr("用例1  俯仰项: H=5m, 正下方, 扫俯仰角 (核心嫌疑)")
    print("  %-8s %8s %10s %12s %14s %12s" %
          ("pitch°", "px-CX", "err_fwd", "err_right", "err_fwd|pitch=0", "err_mag"))
    for pdeg in (-6.0, -4.0, -3.0, -2.0, 0.0, 2.0, 3.0, 4.0, 6.0):
        d = roundtrip(5.0, 0.0, 0.0, math.radians(pdeg))
        if d is None:
            continue
        rows.append(d)
        nopitch = d.get("err_fwd_nopitch", d["err_fwd"])
        print("  %-8.1f %8.1f %10.4f %12.4f %14.4f %12.4f" %
              (pdeg, d["px_off_center"], d["err_fwd"], d["err_right"], nopitch, d["err_mag"]))

    # ---- 用例 2: 俯仰项 vs 高度 ----
    hr("用例2  俯仰项 vs 高度: pitch=+3° (HOVER_SEARCH 实测常见值), R=0")
    print("  %-6s %10s %18s %10s" % ("H(m)", "err_fwd", "理论 tan3°·H", "err_mag"))
    for H in (5.0, 4.0, 3.0, 2.5, 2.0, 1.7, 1.5, 1.2):
        d = roundtrip(H, 0.0, 0.0, math.radians(3.0))
        if d is None:
            continue
        rows.append(d)
        print("  %-6.1f %10.4f %18.4f %10.4f" %
              (H, d["err_fwd"], math.tan(math.radians(3.0)) * H, d["err_mag"]))

    # ---- 用例 3: 侧偏项 (pitch=+3°, 各高度的侧偏扫描) ----
    hr("用例3  侧偏项: pitch=+3°, 各高度扫侧偏 (看是否随侧偏放大 → 畸变/比例项)")
    print("  %-6s %-8s %8s %10s %10s %10s" %
          ("H(m)", "侧偏R(m)", "%半视场", "err_fwd", "err_right", "err_mag"))
    for H in (5.0, 3.0, 2.0, 1.5):
        for R in (0.0, 0.5, 1.0, 2.0, 3.0):
            if abs(R) > 0.64 * H:
                continue
            d = roundtrip(H, 0.0, R, math.radians(3.0))
            if d is None:
                continue
            rows.append(d)
            print("  %-6.1f %-8.1f %7.0f%% %10.4f %10.4f %10.4f" %
                  (H, R, d["pct_of_half_fov_h"], d["err_fwd"], d["err_right"], d["err_mag"]))

    # ---- 用例 4: 真实检测框是整数框 (量化误差) ----
    hr("用例4  整数框量化 + 俯仰0: 看像素量化贡献多少")
    print("  %-6s %-8s %10s %10s" % ("H(m)", "侧偏R(m)", "err_mag(浮点框)", "err_mag(整数框)"))
    for H in (5.0, 2.0, 1.5):
        for R in (0.0, 1.0, 2.0):
            if abs(R) > 0.64 * H:
                continue
            d1 = roundtrip(H, 0.0, R, 0.0, quantize=False)
            d2 = roundtrip(H, 0.0, R, 0.0, quantize=True)
            if d1 and d2:
                print("  %-6.1f %-8.1f %16.4f %16.4f" % (H, R, d1["err_mag"], d2["err_mag"]))
                rows.append(d2)

    # ---- CSV ----
    out = "/tmp/calib_offline.csv"
    keys = ["H", "fwd", "right", "pitch_deg", "px", "py", "radius_px", "px_off_center",
            "py_off_center", "pct_of_half_fov_h", "err_fwd", "err_right", "err_mag"]
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("\nCSV 已写出: %s (%d 行)" % (out, len(rows)))


if __name__ == "__main__":
    main()
