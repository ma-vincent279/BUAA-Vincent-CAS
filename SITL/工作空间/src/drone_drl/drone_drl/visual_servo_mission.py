#!/usr/bin/env python3
"""
视觉伺服定点任务 — 双桶依次伺服投弹。
状态机: TAKEOFF → FORWARD(飞往桶1区域) → SEARCH → TRACK(PID+高度锁定)
        → DESCEND(PID锁定+匀速下降至2m) → HOVER(低空悬停收敛)
        → DROP(投弹, 桶1) → MOVE_B2(飞往桶2区域)
        → SEARCH → TRACK → DESCEND → HOVER → DROP(投弹, 桶2)
        → RESULT(投放完成) → RETURN(返航) → LAND
全程偏航角锁定，仅平移。通过 /bucket/target_id Int32 话题切换检测器跟踪目标。
任务节点不获取桶精确坐标，落点判定由独立裁判节点 drop_judge 完成。
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from sensor_msgs.msg import RegionOfInterest
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 状态 ----
(STEP_TAKEOFF, STEP_FORWARD, STEP_SEARCH, STEP_TRACK, STEP_DESCEND,
 STEP_HOVER, STEP_DROP, STEP_MOVE_B2, STEP_RESULT, STEP_RETURN,
 STEP_LAND) = range(11)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD: "FORWARD",
    STEP_SEARCH: "SEARCH", STEP_TRACK: "TRACK", STEP_DESCEND: "DESCEND",
    STEP_HOVER: "HOVER", STEP_DROP: "DROP", STEP_MOVE_B2: "MOVE_B2",
    STEP_RESULT: "RESULT", STEP_RETURN: "RETURN", STEP_LAND: "LAND",
}

# ---- 参数 ----
TAKEOFF_HEIGHT   = 3.5    # 巡航高度 (降低以缩小投放距离)
DROP_HEIGHT      = 2.0    # 投放高度 (在此高度投弹)
FORWARD_DIST     = 5.0    # 前飞距离 (两桶大致在 5m 前向线上)
BUCKET_SIDE      = 3.0    # 桶1在右 3m, 桶2在左 3m (相距 6m)
CRUISE_SPEED      = 1.0
SEARCH_SPEED     = 0.5     # 搜索时前飞速度
STEP_TIMEOUT     = 120.0

# PID 像素 → 机体速度
KP_H = 0.004      # (m/s) / pixel
KI_H = 0.00005
KD_H = 0.002
MAX_HORIZ_VEL = 1.5
INTEGRAL_LIMIT = 80.0

# 检测 & 伺服的阈值
DETECT_TIMEOUT   = 3.0    # 丢 target 后悬停等待 s
TRACK_ARRIVE_PX  = 40.0   # 像素误差阈值，认作已对准
STABILIZE_TIME   = 2.0    # 稳定对准 N 秒后切位置保持
HOVER_DROP_DELAY = 3.0    # 悬停后等 N 秒自动投放
IMG_W = 640
IMG_H = 480

# ---- 投放 ----
# 落点判定由独立节点 drop_judge 完成

# 航点到达 / 降落
POS_ARRIVE_DIST  = 0.5
LAND_DWELL_TIME  = 3.0


class VisualServoController:
    """基于 ROI 像素偏差的 PID 机体速度控制器"""

    def __init__(self):
        self.kp, self.ki, self.kd = KP_H, KI_H, KD_H
        self.max_vel = MAX_HORIZ_VEL
        self.int_limit = INTEGRAL_LIMIT
        self.prev_ey = self.prev_ex = 0.0
        self.int_ex = self.int_ey = 0.0

    def compute(self, roi, dt):
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        ex = cx - IMG_W / 2.0   # + = target right
        ey = IMG_H / 2.0 - cy  # 下视摄像头: TOP=前, 前方物体 cy<CY, ey>0→前进

        dt = max(dt, 0.001)
        self.int_ex += ex * dt
        self.int_ey += ey * dt
        self.int_ex = max(-self.int_limit, min(self.int_limit, self.int_ex))
        self.int_ey = max(-self.int_limit, min(self.int_limit, self.int_ey))

        dex = (ex - self.prev_ex) / dt
        dey = (ey - self.prev_ey) / dt

        v_lat = self.kp * ex + self.ki * self.int_ex + self.kd * dex
        v_fwd = self.kp * ey + self.ki * self.int_ey + self.kd * dey
        v_lat = max(-self.max_vel, min(self.max_vel, v_lat))
        v_fwd = max(-self.max_vel, min(self.max_vel, v_fwd))

        self.prev_ex, self.prev_ey = ex, ey
        return v_fwd, v_lat, math.hypot(ex, ey)


class VisualServoMission(Node):
    def __init__(self):
        super().__init__('visual_servo_mission')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        # Pub
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Sub — PX4 用 BEST_EFFORT 发布所有 /fmu/out 话题
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._cb_local, qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._cb_land, qos)
        self.detect_sub = self.create_subscription(RegionOfInterest, '/bucket/detection', self._cb_detect, 10)

        # 飞机状态
        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self.heading = None
        self.landed = False
        self.status = None

        # 检测
        self.latest_roi = None
        self._last_detect_time = self.get_clock().now()

        # 任务
        self.step = STEP_TAKEOFF
        self._step_start_time = None
        self.controller = VisualServoController()
        self._hover_target = None
        self._stabilize_start = None

        # 投放
        self.drop_client = self.create_client(Trigger, '/drop_payload')
        self._drop_future = None
        self._bucket_target = 1     # 当前目标桶: 1 或 2

        # 检测器切换 (v2: Int32 topic 兼容 yolo_bucket_detector)
        self.bucket_target_pub = self.create_publisher(Int32, '/bucket/target_id', 10)

        # 降落
        self._land_dwell_start = None

        # 模式
        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._offboard_time = None
        self._last_arm_attempt = -10.0
        self._init_wait_start = None
        self._initial_wait_done = False

        self._print_ctr = 0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            "Visual servo mission ready: dual-bucket, yaw-locked, PID visual-servo, "
            f"auto-drop after {HOVER_DROP_DELAY}s hover per bucket")

    # ========== 回调 ==========
    def _cb_status(self, m):
        if self.status is None:
            self.get_logger().info(
                f"First status: arm={m.arming_state} nav={m.nav_state} "
                f"failsafe={m.failsafe} preflight_ok={m.pre_flight_checks_pass} "
                f"safety_off={m.safety_off}")
        elif m.failsafe != self.status.failsafe:
            self.get_logger().warn(
                f"FAILSAFE changed to {m.failsafe} nav={m.nav_state} "
                f"arm={m.arming_state} preflight={m.pre_flight_checks_pass} "
                f"safety_off={m.safety_off}")
        elif m.nav_state != self.status.nav_state:
            self.get_logger().info(
                f"nav_state: {self.status.nav_state} → {m.nav_state} "
                f"failsafe={m.failsafe} arm={m.arming_state} "
                f"safety_off={m.safety_off}")
        self.status = m
    def _cb_land(self, m):   self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading   # 持续更新，与 detector 同步
        if self.home_x is None and m.xy_valid:
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.get_logger().info(
                f"Home: ({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                f"heading={math.degrees(self.heading):.0f}deg (LOCKED)")

    def _cb_detect(self, m):
        self.latest_roi = m
        self._last_detect_time = self.get_clock().now()

    # ========== 发布 ==========
    def _pub_vel(self, vx, vy, vz, yaw):
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, velocity=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            velocity=[float(vx), float(vy), float(vz)], yaw=float(yaw),
            position=[float('nan')]*3, acceleration=[float('nan')]*3))

    def _pub_pos(self, x, y, z, yaw):
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, position=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=[float(x), float(y), float(z)], yaw=float(yaw),
            velocity=[float('nan')]*3, acceleration=[float('nan')]*3))

    def _send_cmd(self, cmd, **p):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=cmd, target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True, **p))

    def _enter_step(self, step):
        self.step = step
        self._step_start_time = self.get_clock().now()

    def _detect_fresh(self):
        if self.latest_roi is None:
            return False
        return (self.get_clock().now() - self._last_detect_time).nanoseconds * 1e-9 < DETECT_TIMEOUT

    def _switch_target(self, bucket_id):
        """切换检测器目标桶 (1 或 2)，发布 Int32 到 /bucket/target_id"""
        self._bucket_target = bucket_id
        msg = Int32()
        msg.data = 0 if bucket_id == 1 else 1  # 0=最大桶, 1=第二大桶
        self.bucket_target_pub.publish(msg)
        self.get_logger().info(f"Detector target → bucket {bucket_id} (idx={msg.data})")

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * cos_h - v_lat * sin_h,
                v_fwd * sin_h + v_lat * cos_h,
                v_down)

    def _dist_to(self, x, y, z):
        return math.hypot(self.x - x, self.y - y, self.z - z)

    # ========== 主循环 ==========
    def control_loop(self):
        if self.home_x is None or self.heading is None:
            return

        self._print_ctr += 1
        if self._print_ctr >= 20:  # ~1Hz
            tag = STEP_NAME.get(self.step, "???")
            hdg = math.degrees(self.heading) if self.heading else 0
            has_det = "DET" if self._detect_fresh() else "no-det"
            # 统一坐标输出
            extra = ""
            if self._detect_fresh() and self.step in (STEP_TRACK, STEP_DESCEND, STEP_HOVER, STEP_DROP, STEP_RESULT):
                roi = self.latest_roi
                cx = roi.x_offset + roi.width / 2.0
                cy = roi.y_offset + roi.height / 2.0
                ex, ey = cx - IMG_W/2.0, cy - IMG_H/2.0
                extra = f" px_err=({ex:.0f},{ey:.0f}) bbox={roi.width}x{roi.height}"
            elif self.step == STEP_FORWARD:
                cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
                tx = self.home_x + FORWARD_DIST * cos_h + BUCKET_SIDE * sin_h
                ty = self.home_y + FORWARD_DIST * sin_h - BUCKET_SIDE * cos_h
                d = math.hypot(self.x - tx, self.y - ty)
                extra = f" →target({tx:.1f},{ty:.1f}) dist={d:.1f}m"
            elif self.step == STEP_MOVE_B2:
                cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
                tx = self.home_x + FORWARD_DIST * cos_h - BUCKET_SIDE * sin_h
                ty = self.home_y + FORWARD_DIST * sin_h + BUCKET_SIDE * cos_h
                d = math.hypot(self.x - tx, self.y - ty)
                extra = f" →target({tx:.1f},{ty:.1f}) dist={d:.1f}m"
            elif self.step == STEP_RETURN:
                d = math.hypot(self.x - self.home_x, self.y - self.home_y)
                extra = f" →home dist={d:.1f}m"
            self.get_logger().info(
                f"[{tag}] NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                f"h={hdg:.0f}deg {has_det}{extra}")
            self._print_ctr = 0

        target_z = self.home_z - TAKEOFF_HEIGHT

        # ---- 初始等待 — 持续发零速 setpoint + 等 preflight checks pass ----
        if not self._initial_wait_done:
            if self._init_wait_start is None:
                self._init_wait_start = self.get_clock().now()
            self._pub_vel(0.0, 0.0, 0.0, self.heading if self.heading else 0.0)
            elapsed = (self.get_clock().now() - self._init_wait_start).nanoseconds * 1e-9
            if self.status is not None and self.status.pre_flight_checks_pass and elapsed > 3.0:
                self._initial_wait_done = True
                self.get_logger().info("Initial wait done, preflight passed")
            elif elapsed > 30.0:
                self.get_logger().error(
                    f"Preflight timeout (elapsed={elapsed:.0f}s "
                    f"preflight={self.status.pre_flight_checks_pass if self.status else 'N/A'})")
                self.timer.cancel()
            return

        # ---- 解锁 & 切模式 ----
        if self.status is None:
            self._pub_vel(0, 0, 0, self.heading)
            return
        if self._mode_req_start is None:
            self._mode_req_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._mode_req_start).nanoseconds * 1e-9

        if self.status.arming_state == 2 and self.status.nav_state == 14:
            pass  # 一切就绪，进入状态机
        elif elapsed > 30.0:
            self.get_logger().error("Mode switch timeout, aborting")
            self.timer.cancel()
            return
        # 1) 先解锁（在 AUTO_LOITER 模式下），然后再切 OFFBOARD
        elif self.status.arming_state != 2:
            if not self._arm_req or elapsed - self._last_arm_attempt > 1.0:
                self._arm_req = True
                self._last_arm_attempt = elapsed
                self._send_cmd(400, param1=1.0, param2=2989.0)
                self.get_logger().info(f"Requesting ARM in nav={self.status.nav_state} (t={elapsed:.1f}s)")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading if self.heading else 0.0)
            return
        # 2) 已解锁 → 切 OFFBOARD
        elif self.status.nav_state != 14:
            if not self._mode_req:
                self._mode_req = True
                self._send_cmd(5300, param1=0.0)
                self._send_cmd(176, param1=1.0, param2=6.0)
                self.get_logger().info(f"Armed, requesting OFFBOARD (nav={self.status.nav_state})")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading if self.heading else 0.0)
            return
        else:
            return  # 不应到达

        # ================================================================
        # 状态机
        # ================================================================
        if self.step == STEP_TAKEOFF:
            if self._step_start_time is None:
                self._step_start_time = self.get_clock().now()
                self._tkoff_x = self.x
                self._tkoff_y = self.y
                self._tkoff_z_start = self.z
            tkoff_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            # 前 3s 原地悬停 stabilization，然后以 0.5m/s 缓慢爬升
            if tkoff_elapsed < 3.0:
                self._pub_pos(self._tkoff_x, self._tkoff_y, self._tkoff_z_start, self.heading)
            else:
                ramp_t = tkoff_elapsed - 3.0
                z_ramp = self._tkoff_z_start - min(TAKEOFF_HEIGHT, ramp_t * 0.5)
                self._pub_pos(self._tkoff_x, self._tkoff_y, z_ramp, self.heading)
            if abs(self.z - target_z) < 0.8 and abs(self.z - self.home_z) > 1.5:
                self._enter_step(STEP_FORWARD)
                cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
                tx = self.home_x + FORWARD_DIST * cos_h + BUCKET_SIDE * sin_h
                ty = self.home_y + FORWARD_DIST * sin_h - BUCKET_SIDE * cos_h
                self.get_logger().info(
                    f"Takeoff complete -> FORWARD to bucket1 area "
                    f"NED({tx:.1f},{ty:.1f})")

        # ---- 飞往桶1区域 (前进 5m + 右侧 3m) ----
        elif self.step == STEP_FORWARD:
            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            # 桶1 近似位置：forward + right
            fwd_target_x = self.home_x + FORWARD_DIST * cos_h + BUCKET_SIDE * sin_h
            fwd_target_y = self.home_y + FORWARD_DIST * sin_h - BUCKET_SIDE * cos_h

            if self._detect_fresh():
                self.get_logger().info("Detected during FORWARD → TRACK")
                self.controller.prev_ex = self.controller.prev_ey = 0.0
                self.controller.int_ex = self.controller.int_ey = 0.0
                self._enter_step(STEP_TRACK)
                return

            dist_to_target = math.hypot(self.x - fwd_target_x, self.y - fwd_target_y)
            if dist_to_target < POS_ARRIVE_DIST:
                self._enter_step(STEP_SEARCH)
                self.get_logger().info(
                    f"FORWARD done (at bucket1 area: "
                    f"fwd={FORWARD_DIST:.0f}m right={BUCKET_SIDE:.0f}m) → SEARCH")
                return

            self._pub_pos(fwd_target_x, fwd_target_y, target_z, self.heading)

        # ---- 搜索 (低速前飞 + 等待检测) ----
        elif self.step == STEP_SEARCH:
            elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if elapsed > STEP_TIMEOUT:
                self.get_logger().warn("Search timeout, hovering in place")
                self._hover_target = (self.x, self.y, self.z)
                self._enter_step(STEP_HOVER)
                return

            if self._detect_fresh():
                self.get_logger().info("Detected during SEARCH → TRACK")
                self.controller.prev_ex = self.controller.prev_ey = 0.0
                self.controller.int_ex = self.controller.int_ey = 0.0
                self._enter_step(STEP_TRACK)
                return

            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            self._pub_vel(SEARCH_SPEED * cos_h, SEARCH_SPEED * sin_h, 0.0, self.heading)

        # ---- 视觉伺服 (PID + 巡航高度锁定, 稳定后下降) ----
        elif self.step == STEP_TRACK:
            if not self._detect_fresh():
                self.get_logger().warn("Detection lost in TRACK, returning to SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_SEARCH)
                return

            roi = self.latest_roi
            v_fwd, v_lat, err_px = self.controller.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = (target_z - self.z) * 0.5   # 锁定巡航高度, 防漂移
            self._pub_vel(vx, vy, vz, self.heading)

            if err_px < TRACK_ARRIVE_PX:
                if self._stabilize_start is None:
                    self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now() - self._stabilize_start).nanoseconds > STABILIZE_TIME * 1e9:
                    self.get_logger().info(
                        f"TRACK converged (err={err_px:.0f}px) → DESCEND to {DROP_HEIGHT:.0f}m")
                    self._enter_step(STEP_DESCEND)
            else:
                self._stabilize_start = None

        # ---- 下降 (PID 水平锁定 + 匀速下降至投放高度) ----
        elif self.step == STEP_DESCEND:
            drop_z = self.home_z - DROP_HEIGHT
            if not self._detect_fresh():
                self.get_logger().warn("Detection lost during DESCEND, returning to SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_SEARCH)
                return

            roi = self.latest_roi
            v_fwd, v_lat, _ = self.controller.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = 0.5   # NED: z+ = 向下, 0.5 m/s 匀速下降
            self._pub_vel(vx, vy, vz, self.heading)

            if self.z > drop_z - 0.2:
                self._hover_target = (self.x, self.y, self.z)
                self._stabilize_start = None
                self.get_logger().info(
                    f"At drop altitude ({self.z:.1f}m) → HOVER to converge")
                self._enter_step(STEP_HOVER)

        # ---- 低空悬停锁定 (PID 视觉伺服, 像素收敛后自动投放) ----
        elif self.step == STEP_HOVER:
            drop_z = self.home_z - DROP_HEIGHT
            if not self._detect_fresh():
                self.get_logger().warn("Detection lost in HOVER, returning to SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_SEARCH)
                return

            roi = self.latest_roi
            v_fwd, v_lat, err_px = self.controller.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = (drop_z - self.z) * 0.8            # 紧锁投放高度
            vz = max(-0.3, min(0.3, vz))
            self._pub_vel(vx, vy, vz, self.heading)

            # 低空要求更严：像素误差 < 20px 且稳定 2s
            if err_px < 20:
                if self._stabilize_start is None:
                    self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now() - self._stabilize_start).nanoseconds > STABILIZE_TIME * 1e9:
                    self.get_logger().info(
                        f"HOVER converged (err={err_px:.0f}px) → DROP")
                    self._enter_step(STEP_DROP)
            else:
                self._stabilize_start = None

        # ---- 投放 (PID 视觉锁定, 保持精度) ----
        elif self.step == STEP_DROP:
            drop_z = self.home_z - DROP_HEIGHT
            if self._detect_fresh():
                roi = self.latest_roi
                v_fwd, v_lat, _ = self.controller.compute(roi, 0.05)
                vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            else:
                vx, vy = 0.0, 0.0
            vz = (drop_z - self.z) * 0.8
            vz = max(-0.2, min(0.2, vz))
            self._pub_vel(vx, vy, vz, self.heading)

            if not self._drop_future:
                if not self.drop_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error("/drop_payload not available")
                    self._enter_step(STEP_RETURN)
                    return
                self._drop_future = self.drop_client.call_async(Trigger.Request())
                self.get_logger().info(
                    f"DROPPING for bucket{self._bucket_target} at "
                    f"NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"heading={math.degrees(self.heading):.0f}deg")
            elif self._drop_future.done():
                result = self._drop_future.result()
                if result is not None and result.success:
                    self.get_logger().info(
                        f"Drop bucket{self._bucket_target} OK: {result.message}")
                else:
                    msg = result.message if result else "no response"
                    self.get_logger().warn(
                        f"Drop bucket{self._bucket_target} failed: {msg}")
                self._drop_future = None

                if self._bucket_target == 1:
                    # 桶1 投完 → 切检测器 → 飞往桶2
                    self._switch_target(2)
                    self._enter_step(STEP_MOVE_B2)
                else:
                    self._enter_step(STEP_RESULT)

        # ---- 飞往桶2区域 (绝对位置: 前进5m + 左侧3m from home) ----
        elif self.step == STEP_MOVE_B2:
            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            # 桶2 近似位置：forward 5m + left 3m
            b2_target_x = self.home_x + FORWARD_DIST * cos_h - BUCKET_SIDE * sin_h
            b2_target_y = self.home_y + FORWARD_DIST * sin_h + BUCKET_SIDE * cos_h

            dist = math.hypot(self.x - b2_target_x, self.y - b2_target_y)
            if dist < POS_ARRIVE_DIST:
                self.get_logger().info(
                    f"MOVE_B2 done (at bucket2 area: "
                    f"fwd={FORWARD_DIST:.0f}m left={BUCKET_SIDE:.0f}m) → SEARCH")
                self._enter_step(STEP_SEARCH)
                return

            self._pub_pos(b2_target_x, b2_target_y, target_z, self.heading)

        # ---- 投放完成，短暂悬停后返航 ----
        elif self.step == STEP_RESULT:
            tx, ty, tz = self._hover_target
            self._pub_pos(tx, ty, tz, self.heading)
            result_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if result_elapsed > 1.0:
                self.get_logger().info("双桶投放完成 → RETURN")
                self._enter_step(STEP_RETURN)

        # ---- 返航 (飞回 home) ----
        elif self.step == STEP_RETURN:
            rx, ry = self.home_x, self.home_y
            rz = self.home_z - TAKEOFF_HEIGHT
            self._pub_pos(rx, ry, rz, self.heading)
            if self._dist_to(rx, ry, rz) < POS_ARRIVE_DIST:
                self.get_logger().info("Returned home → LAND")
                self._enter_step(STEP_LAND)

        # ---- 降落 ----
        elif self.step == STEP_LAND:
            self._pub_pos(self.home_x, self.home_y, self.home_z, self.heading)
            if self.landed:
                if self._land_dwell_start is None:
                    self._land_dwell_start = self.get_clock().now()
                elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                    self._send_cmd(400, param1=0.0)
                    self.timer.cancel()
                    self.get_logger().info("Mission complete — disarmed (landed)")
            elif abs(self.z - self.home_z) < 0.15:
                if self._land_dwell_start is None:
                    self._land_dwell_start = self.get_clock().now()
                elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                    self._send_cmd(400, param1=0.0)
                    self.timer.cancel()
                    self.get_logger().info("Mission complete — disarmed (altitude)")
            else:
                self._land_dwell_start = None


def main():
    rclpy.init()
    node = VisualServoMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
