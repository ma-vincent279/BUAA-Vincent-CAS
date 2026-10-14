#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定工具 B(仿真飞行): 让飞机自己飞到"固定高度 × 固定侧偏"的网格点悬停,
实时读取 **任务节点真实的 _roi_to_ned** 输出, 与桶的真值(=卷尺)对比, 输出偏差曲线.

与工具 A 的区别:
  A: 用检测器投影公式离线往返, 纯数学模型, 无噪声
  B: 真跑 Gazebo + 真检测器(YOLOv8) + 真 MPC 姿态, 含 MPU 漂移/姿态/像素量化
     → 用来验证 A 的结论(偏差 = tan(pitch)·H)在真实链路里是否成立

反投影仍然直接调用 drone_drl.e 里**真实的** _roi_to_ned, 不复制逻辑.

输出:
  /tmp/calib_refproj.csv   逐帧明细 (含 phase/pitch/误差/理论俯仰项/残差)
  终端: 每个网格点的统计表 + 全样本回归 (err_fwd 对 tan(pitch)·H)

用法(由 scripts/calib_refproj.sh 调用, 也可 ros2 run 之外直接 python3 运行):
  source ~/SITL/install/setup.bash && python3 ~/SITL/scripts/calib_refproj.py
"""

import csv
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleAttitude, VehicleControlMode,
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import String

# 真实的 _roi_to_ned (直接来自项目代码)
from drone_drl.e import EMission

# ============================================================
# 常量 (与 e.py / 检测器 一致)
# ============================================================
IMG_W, IMG_H = 640, 360
CX, CY = IMG_W / 2.0, IMG_H / 2.0
HFOV = math.radians(85.0)
VFOV = math.radians(54.0)
FX = IMG_W / (2.0 * math.tan(HFOV / 2.0))
FY = IMG_H / (2.0 * math.tan(VFOV / 2.0))
BUCKET_R = 0.125

H0 = math.radians(92.0)                 # 与 test_e.sh / e.world 一致的机头方向
BUCKET_NED = (0.00, 34.00)              # 桶 B3 真值 (仿真世界精确已知)
HOME = (0.0, 0.0, 0.0)                  # 用 PX4 绝对 NED 作 home 参考 (SITL home≈0)

QOS_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, depth=10)

# 网格: (高度, 侧偏列表)  —— 侧偏上限按 0.64×半视场宽度取, 保证桶在画面内
GRID = [
    (5.0, [0.0, 0.5, -0.5, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0]),
    (3.0, [0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5]),
    (2.0, [0.0, 0.5, -0.5, 1.0, -1.0]),
    (1.5, [0.0, 0.5, -0.5, 0.9, -0.9]),
]
SETTLE_TOL = 0.20          # 到位判据 (m)
SETTLE_HOLD = 1.2          # 到位后保持多久才开始统计 (s)
SAMPLE_TIME = 2.0          # 每点采样时长 (s)
WP_TIMEOUT = 12.0          # 单点超时 (s)


def body_to_ned(v_fwd, v_lat, heading):
    c, s = math.cos(heading), math.sin(heading)
    return (v_fwd * c - v_lat * s, v_fwd * s + v_lat * c)


class CalibStub(object):
    """给真实 _roi_to_ned / _body_to_ned 提供实时状态."""
    _body_to_ned = EMission._body_to_ned
    _roi_to_ned = EMission._roi_to_ned

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.home_x, self.home_y, self.home_z = HOME
        self.heading = H0
        self._pitch = 0.0
        self._roll = 0.0


class CalibNode(Node):
    def __init__(self):
        super().__init__('calib_refproj')

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self._cb_local, QOS_BE)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self._cb_att, QOS_BE)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode',
                                 self._cb_mode, QOS_BE)
        self.create_subscription(String, '/bucket/detections', self._cb_det, 10)

        self.x = self.y = self.z = None
        self.vx = self.vy = self.vz = 0.0
        self.heading = None
        self.pitch = self.roll = 0.0
        self.armed = False
        self.offboard = False
        self.dets = []                     # [(x,y,w,h,conf)]
        self.det_t = 0.0

        self.stub = CalibStub()
        self.rows = []
        self.cur = None                    # 当前网格点信息
        self.phase = "idle"
        self.land_cmd = False

        self.timer = self.create_timer(0.05, self._tick)
        self.t0 = time.time()
        self.get_logger().info(
            "Calib B ready. 桶真值 NED(%.2f,%.2f) 航向 %.0f°  网格 %d 点"
            % (BUCKET_NED[0], BUCKET_NED[1], math.degrees(H0),
               sum(len(g[1]) for g in GRID)))

    # ---------------- 回调 ----------------
    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if math.isfinite(m.vx): self.vx = m.vx
        if math.isfinite(m.vy): self.vy = m.vy
        if math.isfinite(m.vz): self.vz = m.vz
        if math.isfinite(m.heading): self.heading = m.heading

    def _cb_att(self, m):
        w, x, y_, z_ = float(m.q[0]), float(m.q[1]), float(m.q[2]), float(m.q[3])
        self.roll = math.atan2(2.0 * (w * x + y_ * z_), 1.0 - 2.0 * (x * x + y_ * y_))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y_ - z_ * x))))

    def _cb_mode(self, m):
        self.armed = bool(m.flag_armed)
        self.offboard = bool(m.flag_control_offboard_enabled)

    def _cb_det(self, m):
        try:
            data = json.loads(m.data)
        except Exception:
            return
        ds = []
        for d in data.get("buckets", []):
            ds.append((float(d["x"]), float(d["y"]), float(d["w"]), float(d["h"]),
                       float(d.get("conf", 0.0))))
        self.dets = ds
        self.det_t = time.time()

    # ---------------- 发布 ----------------
    def _cmd(self, command, **kw):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=command, target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True, **kw))

    def _pub_pos(self, tx, ty, tz):
        self.mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, position=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=[float(tx), float(ty), float(tz)],
            yaw=float(H0), yawspeed=float('nan')))

    # ---------------- 采样 ----------------
    def _expected_pixel(self):
        """桶真值在本帧应当出现的像素位置 (用理想针孔模型, 只为挑出正确的那个框)."""
        if self.x is None or self.heading is None:
            return None
        H = HOME[2] - self.z
        if H < 0.3:
            return None
        dx = BUCKET_NED[0] - self.x
        dy = BUCKET_NED[1] - self.y
        fwd, right = EMission._ned_to_body(self.stub, dx, dy)
        return (CX + right * FX / H, CY - fwd * FY / H)

    def _pick_roi(self):
        exp = self._expected_pixel()
        if exp is None or not self.dets or (time.time() - self.det_t) > 0.6:
            return None, None
        best, bd = None, float('inf')
        for (x, y, w, h, c) in self.dets:
            cx, cy = x + w / 2.0, y + h / 2.0
            d = math.hypot(cx - exp[0], cy - exp[1])
            if d < bd:
                bd, best = d, (x, y, w, h, c, cx, cy)
        if best is None or bd > 120.0:      # 离预期太远的框不当成目标桶
            return None, None
        return best, bd

    def _sample(self):
        """采一帧: 真值 vs 真实 _roi_to_ned 估计."""
        if self.x is None or self.heading is None or self.z is None:
            return
        H = HOME[2] - self.z
        if H < 0.3:
            return
        roi_det, pxdev = self._pick_roi()
        if roi_det is None:
            self._zero_det = getattr(self, '_zero_det', 0) + 1
            return
        x, y, w, h, conf, cx, cy = roi_det
        roi = RegionOfInterest()
        roi.x_offset, roi.y_offset = int(x), int(y)
        roi.width, roi.height = int(w), int(h)

        # 真值: 桶相对飞机的机体系偏移
        dx = BUCKET_NED[0] - self.x
        dy = BUCKET_NED[1] - self.y
        truth_fwd, truth_right = EMission._ned_to_body(self.stub, dx, dy)

        # 真实反投影
        self.stub.x, self.stub.y, self.stub.z = self.x, self.y, self.z
        self.stub.home_x, self.stub.home_y, self.stub.home_z = HOME
        self.stub.heading = self.heading
        self.stub._pitch = self.pitch
        self.stub._roll = self.roll
        est = EMission._roi_to_ned(self.stub, roi)
        if est is None:
            return
        ex, ey = est[0] - self.x, est[1] - self.y      # 估计的机体系偏移
        est_fwd, est_right = EMission._ned_to_body(self.stub, ex, ey)

        e_fwd = est_fwd - truth_fwd
        e_right = est_right - truth_right
        pred = math.tan(self.pitch) * H                # 工具A给出的理论俯仰项
        self.rows.append({
            "t": round(time.time() - self.t0, 2), "phase": self.phase,
            "H": round(H, 3), "pitch_deg": round(math.degrees(self.pitch), 2),
            "roll_deg": round(math.degrees(self.roll), 2),
            "speed": round(math.hypot(self.vx, self.vy), 3),
            "truth_fwd": round(truth_fwd, 4), "truth_right": round(truth_right, 4),
            "est_fwd": round(est_fwd, 4), "est_right": round(est_right, 4),
            "err_fwd": round(e_fwd, 4), "err_right": round(e_right, 4),
            "err_mag": round(math.hypot(e_fwd, e_right), 4),
            "pred_tanpitch_H": round(pred, 4),
            "resid_fwd": round(e_fwd - pred, 4),
            "px_right": round(cx - CX, 1), "py_fwd": round(CY - cy, 1),
            "roi_w": int(w), "roi_h": int(h), "conf": round(conf, 3),
            "pxdev": round(pxdev, 1),
        })

    # ---------------- 主循环 ----------------
    def _tick(self):
        el = time.time() - self.t0
        heading = self.heading if self.heading is not None else H0

        # OFFBOARD 请求前持续发 setpoint
        if self.x is None:
            self.mode_pub.publish(OffboardControlMode(
                timestamp=self.get_clock().now().nanoseconds // 1000, velocity=True))
            self.traj_pub.publish(TrajectorySetpoint(
                timestamp=self.get_clock().now().nanoseconds // 1000,
                velocity=[0.0, 0.0, 0.0], yaw=float(heading)))
            return

        if not self.armed:
            if not getattr(self, '_arm_req', False) or el - getattr(self, '_last_arm', 0) > 1.0:
                self._arm_req = True; self._last_arm = el
                self._cmd(400, param1=1.0, param2=2989.0)
                self.get_logger().info("Requesting ARM...")
            self._pub_hold()
            return
        if not self.offboard:
            if not getattr(self, '_mode_req', False):
                self._mode_req = True
                self._cmd(176, param1=1.0, param2=6.0)
                self.get_logger().info("ARM OK, requesting OFFBOARD...")
            self._pub_hold()
            return

        if self.land_cmd:
            return

        # ---- 网格遍历 ----
        if self.cur is None:
            self._wp_list = [(H, R) for (H, lst) in GRID for R in lst]
            self._wp_i = 0
            self.cur = self._wp_list[0]
            self._wp_t0 = time.time()
            self._settled_at = None
            self._samp_end = None
            H, R = self.cur
            bx = BUCKET_NED[0] - body_to_ned(0.0, R, H0)[0]
            by = BUCKET_NED[1] - body_to_ned(0.0, R, H0)[1]
            self._wp_target = (bx, by, -H)
            self._wp_row0 = len(self.rows)
            self.phase = "transit"
            self.get_logger().info("WP %d/%d  H=%.1fm R=%+.1fm → NED(%.2f,%.2f)"
                                   % (self._wp_i + 1, len(self._wp_list), H, R, bx, by))
            return

        bx, by, bz = self._wp_target
        self._pub_pos(bx, by, bz)
        d2 = math.hypot(self.x - bx, self.y - by)
        dh = abs(self.z - bz)

        if self._settled_at is None:
            self.phase = "transit"
            self._sample()
            if d2 < SETTLE_TOL and dh < SETTLE_TOL:
                if getattr(self, '_ok_since', None) is None:
                    self._ok_since = time.time()
                elif time.time() - self._ok_since > SETTLE_HOLD:
                    self._settled_at = time.time()
                    self._samp_end = time.time() + SAMPLE_TIME
                    self.phase = "settled"
                    self.get_logger().info("  settled (d=%.2f dh=%.2f) sampling %.1fs"
                                           % (d2, dh, SAMPLE_TIME))
            else:
                self._ok_since = None
            if time.time() - self._wp_t0 > WP_TIMEOUT and self._settled_at is None:
                self.get_logger().warn("  WP timeout, skip")
                self._next_wp()
            return

        # 采样窗口
        self.phase = "settled"
        self._sample()
        if time.time() >= self._samp_end:
            self._next_wp()

    def _pub_hold(self):
        ref_z = self.z if self.z is not None else -1.0
        self._pub_pos(self.x, self.y, min(ref_z, -1.0))

    def _next_wp(self):
        # 本点小结 (只看本点开始后采到的帧, 避免混入同高度的上一个点)
        rows = [r for r in self.rows[self._wp_row0:] if r["phase"] == "settled"]
        if rows:
            mf = sum(r["err_fwd"] for r in rows) / len(rows)
            mr = sum(r["err_right"] for r in rows) / len(rows)
            mm = sum(r["err_mag"] for r in rows) / len(rows)
            mp = sum(r["pitch_deg"] for r in rows) / len(rows)
            self.get_logger().info(
                "  RESULT H=%.1f R=%+.1f  n=%d  pitch=%+.2f°  err_fwd=%+.4f  "
                "err_right=%+.4f  err_mag=%.4f"
                % (self.cur[0], self.cur[1], len(rows), mp, mf, mr, mm))

        self._wp_i += 1
        if self._wp_i >= len(self._wp_list):
            self._finish()
            return
        self.cur = self._wp_list[self._wp_i]
        self._wp_t0 = time.time()
        self._settled_at = None
        self._samp_end = None
        self._ok_since = None
        H, R = self.cur
        off = body_to_ned(0.0, R, H0)
        self._wp_target = (BUCKET_NED[0] - off[0], BUCKET_NED[1] - off[1], -H)
        self._wp_row0 = len(self.rows)
        self.get_logger().info("WP %d/%d  H=%.1fm R=%+.1fm" %
                               (self._wp_i + 1, len(self._wp_list), H, R))

    def _finish(self):
        self.phase = "done"
        self._write_csv()
        self._report()
        self.get_logger().info("Calibration done → landing")
        self._cmd(21, param1=0.0, param2=0.0, param7=float('nan'))
        self.land_cmd = True
        time.sleep(1.0)
        raise SystemExit(0)

    def _write_csv(self):
        keys = ["t", "phase", "H", "pitch_deg", "roll_deg", "speed",
                "truth_fwd", "truth_right", "est_fwd", "est_right",
                "err_fwd", "err_right", "err_mag", "pred_tanpitch_H", "resid_fwd",
                "px_right", "py_fwd", "roi_w", "roi_h", "conf", "pxdev"]
        with open("/tmp/calib_refproj.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                w.writerow(r)
        self.get_logger().info("CSV → /tmp/calib_refproj.csv (%d 帧)" % len(self.rows))

    def _report(self):
        rs = [r for r in self.rows if r["phase"] == "settled"]
        L = self.get_logger().info
        L("=" * 72)
        L("标定结果 B (仿真飞行) — 桶真值 NED(%.2f,%.2f)" % BUCKET_NED)
        L("=" * 72)
        if not rs:
            L("  无有效样本"); return
        # 分组: 高度 × 侧偏
        L("  %-6s %-8s %5s %8s %10s %10s %10s %10s" %
          ("H(m)", "R(m)", "n", "pitch°", "err_fwd", "err_right", "err_mag", "pred_fwd"))
        groups = {}
        for r in rs:
            k = (round(r["H"] * 2) / 2.0, )
            groups.setdefault(k, []).append(r)
        for k in sorted(groups, key=lambda x: -x[0]):
            g = groups[k]
            n = len(g)
            L("  %-6.1f %-8s %5d %8.2f %10.4f %10.4f %10.4f %10.4f" %
              (k[0], "-", n,
               sum(x["pitch_deg"] for x in g) / n,
               sum(x["err_fwd"] for x in g) / n,
               sum(x["err_right"] for x in g) / n,
               sum(x["err_mag"] for x in g) / n,
               sum(x["pred_tanpitch_H"] for x in g) / n))
        # 回归: err_fwd ~ tan(pitch)*H
        xs = [r["pred_tanpitch_H"] for r in rs]
        ys = [r["err_fwd"] for r in rs]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        sxx = sum((v - mx) ** 2 for v in xs)
        sxy = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
        slope = sxy / sxx if sxx > 1e-12 else float('nan')
        resid = [b - a for a, b in zip(xs, ys)]
        rms_res = math.sqrt(sum(v * v for v in resid) / len(resid))
        rms_y = math.sqrt(sum(v * v for v in ys) / len(ys))
        L("-" * 72)
        L("  回归 err_fwd ≈ k·tan(pitch)·H :  k=%.3f  (k=1 → 完全由俯仰项解释)" % slope)
        L("  原始 err_fwd RMS = %.4f m ; 扣掉俯仰项后残差 RMS = %.4f m" % (rms_y, rms_res))
        # 横向
        er = [r["err_right"] for r in rs]
        L("  err_right RMS = %.4f m (最大 %.4f)" %
          (math.sqrt(sum(v * v for v in er) / len(er)), max(abs(v) for v in er)))
        # 侧偏相关性
        L("  侧偏影响: |err| 在 px_right<50 与 >150 两组的均值 → 见 CSV / 分组表")
        L("=" * 72)


def main():
    rclpy.init()
    node = CalibNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:                      # 保证 CSV 一定落盘
        node.get_logger().error("calib exited with: %r" % (e,))
    if rclpy.ok():
        node._write_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
