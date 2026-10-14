#!/usr/bin/env python3
"""
侦察任务节点 — 5 个白筒视觉伺服悬停侦察。
状态机: TAKEOFF → FORWARD(飞往 55m 侦察区) → (SEARCH → TRACK → HOVER) × 5
        → RESULT(侦察完成) → RETURN(返航) → LAND
全程偏航角锁定，仅平移。通过 /scout/target_id 切换检测器跟踪目标。
任务节点不获取筒精确坐标，完全依赖 /scout/detection 的 ROI 进行视觉伺服。
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 状态 ----
(STEP_TAKEOFF, STEP_FORWARD, STEP_SEARCH, STEP_TRACK,
 STEP_HOVER, STEP_RESULT, STEP_RETURN, STEP_LAND) = range(8)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD: "FORWARD",
    STEP_SEARCH: "SEARCH", STEP_TRACK: "TRACK",
    STEP_HOVER: "HOVER", STEP_RESULT: "RESULT",
    STEP_RETURN: "RETURN", STEP_LAND: "LAND",
}

# ---- 参数 ----
TAKEOFF_HEIGHT   = 3.5
SCOUT_DIST       = 50.0   # FORWARD 停在筒群前方 5m，避免飞过
CRUISE_SPEED     = 1.0
SEARCH_SPEED     = 0.5
STEP_TIMEOUT     = 120.0

# PID 像素 → 机体速度
KP_H = 0.0015
KI_H = 0.00003
KD_H = 0.0005
MAX_HORIZ_VEL = 0.6
INTEGRAL_LIMIT = 40.0

DETECT_TIMEOUT   = 3.0
TRACK_ARRIVE_PX  = 40.0
STABILIZE_TIME   = 2.0
HOVER_SCOUT_TIME = 2.0   # 侦察悬停时间
IMG_W = 640
IMG_H = 480

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
        ex = cx - IMG_W / 2.0
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


class ScoutMission(Node):
    def __init__(self):
        super().__init__('scout_mission')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        # Pub
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.target_pub = self.create_publisher(Int32, '/scout/target_id', 10)

        # Sub
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._cb_local, qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._cb_land, qos)
        self.detect_sub = self.create_subscription(RegionOfInterest, '/scout/detection', self._cb_detect, 10)

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
        self._scout_index = 0       # 当前侦察的筒 (0-4)
        self.controller = VisualServoController()
        self._hover_target = None
        self._stabilize_start = None
        self._hover_scout_start = None

        # 降落
        self._land_dwell_start = None

        # 模式
        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._last_arm_attempt = -10.0
        self._init_wait_start = None
        self._initial_wait_done = False

        self._print_ctr = 0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            "Scout mission ready: 5-cylinder visual-servo scout, "
            f"area ~{SCOUT_DIST:.0f}m forward")

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

    def _cb_land(self, m):
        self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading   # 持续更新，与 detector 同步
        if self.home_x is None and m.xy_valid:
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.get_logger().info(
                f"Home: ({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                f"heading={math.degrees(self.heading):.0f}deg")

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

    def _switch_target(self, idx):
        """切换检测器到指定筒 (0-4)"""
        self._scout_index = idx
        msg = Int32()
        msg.data = idx
        self.target_pub.publish(msg)
        self.get_logger().info(f"Detector target → cylinder {idx}")

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
            hdg = math.degrees(self.heading)
            has_det = "DET" if self._detect_fresh() else "no-det"
            extra = ""
            if self._detect_fresh() and self.step in (STEP_TRACK, STEP_HOVER):
                roi = self.latest_roi
                cx = roi.x_offset + roi.width / 2.0
                cy = roi.y_offset + roi.height / 2.0
                ex, ey = cx - IMG_W/2.0, cy - IMG_H/2.0
                extra = f" cyl={self._scout_index} px_err=({ex:.0f},{ey:.0f}) bbox={roi.width}x{roi.height}"
            elif self.step == STEP_FORWARD:
                tx = self.home_x
                ty = self.home_y + SCOUT_DIST
                d = math.hypot(self.x - tx, self.y - ty)
                extra = f" →scout_area({tx:.1f},{ty:.1f}) dist={d:.1f}m"
            elif self.step == STEP_RETURN:
                d = math.hypot(self.x - self.home_x, self.y - self.home_y)
                extra = f" →home dist={d:.1f}m"
            self.get_logger().info(
                f"[{tag}] NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                f"h={hdg:.0f}deg {has_det}{extra}")
            self._print_ctr = 0

        target_z = self.home_z - TAKEOFF_HEIGHT

        # ---- 初始等待 ----
        if not self._initial_wait_done:
            if self._init_wait_start is None:
                self._init_wait_start = self.get_clock().now()
            self._pub_vel(0.0, 0.0, 0.0, self.heading)
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
            pass
        elif elapsed > 30.0:
            self.get_logger().error("Mode switch timeout, aborting")
            self.timer.cancel()
            return
        elif self.status.arming_state != 2:
            if not self._arm_req or elapsed - self._last_arm_attempt > 1.0:
                self._arm_req = True
                self._last_arm_attempt = elapsed
                self._send_cmd(400, param1=1.0, param2=2989.0)
                self.get_logger().info(f"Requesting ARM in nav={self.status.nav_state} (t={elapsed:.1f}s)")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return
        elif self.status.nav_state != 14:
            if not self._mode_req:
                self._mode_req = True
                self._send_cmd(5300, param1=0.0)
                self._send_cmd(176, param1=1.0, param2=6.0)
                self.get_logger().info(f"Armed, requesting OFFBOARD (nav={self.status.nav_state})")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return
        else:
            return

        # ================================================================
        # 状态机
        # ================================================================
        if self.step == STEP_TAKEOFF:
            if self._step_start_time is None:
                self._step_start_time = self.get_clock().now()
                self._tkoff_x = self.x
                self._tkoff_y = self.y
                self._tkoff_z_start = self.z
                self._switch_target(0)
            tkoff_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if tkoff_elapsed < 3.0:
                self._pub_pos(self._tkoff_x, self._tkoff_y, self._tkoff_z_start, self.heading)
            else:
                ramp_t = tkoff_elapsed - 3.0
                z_ramp = self._tkoff_z_start - min(TAKEOFF_HEIGHT, ramp_t * 0.5)
                self._pub_pos(self._tkoff_x, self._tkoff_y, z_ramp, self.heading)
            if abs(self.z - target_z) < 0.8 and abs(self.z - self.home_z) > 1.5:
                self._enter_step(STEP_FORWARD)
                tx = self.home_x
                ty = self.home_y + SCOUT_DIST
                self.get_logger().info(
                    f"Takeoff complete → FORWARD to scout area NED({tx:.1f},{ty:.1f})")

        # ---- 飞往侦察区 (正东 55m，不依赖 heading) ----
        elif self.step == STEP_FORWARD:
            scout_x = self.home_x     # 不偏移北向
            scout_y = self.home_y + SCOUT_DIST  # 正东 55m

            if self._detect_fresh():
                self.get_logger().info("Detected during FORWARD → TRACK")
                self.controller.prev_ex = self.controller.prev_ey = 0.0
                self.controller.int_ex = self.controller.int_ey = 0.0
                self._stabilize_start = None
                self._enter_step(STEP_TRACK)
                return

            dist = math.hypot(self.x - scout_x, self.y - scout_y)
            if dist < 10.0:
                # 距离 <10m 时减速接近，防止冲过头
                speed = 0.5
                scale = speed / max(dist, 0.5)
                vx = (scout_x - self.x) * scale
                vy = (scout_y - self.y) * scale
                self._pub_vel(vx, vy, 0.0, self.heading)
                if dist < POS_ARRIVE_DIST:
                    self._enter_step(STEP_SEARCH)
                    self.get_logger().info(
                        f"FORWARD done (at scout area {SCOUT_DIST:.0f}m east) → SEARCH for cylinder {self._scout_index}")
                    return
            else:
                self._pub_pos(scout_x, scout_y, target_z, self.heading)

        # ---- 搜索当前目标筒 ----
        elif self.step == STEP_SEARCH:
            elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if elapsed > STEP_TIMEOUT:
                self.get_logger().warn(f"Search timeout for cylinder {self._scout_index}, skipping")
                self._scout_index += 1
                if self._scout_index < 5:
                    self._switch_target(self._scout_index)
                    self._enter_step(STEP_SEARCH)
                else:
                    self._enter_step(STEP_RESULT)
                return

            if self._detect_fresh():
                self.get_logger().info(f"Detected cylinder {self._scout_index} → TRACK")
                self.controller.prev_ex = self.controller.prev_ey = 0.0
                self.controller.int_ex = self.controller.int_ey = 0.0
                self._stabilize_start = None
                self._enter_step(STEP_TRACK)
                return

            # 慢速前飞搜索，y 超过 home_y+60 则悬停
            if self.y < self.home_y + 60.0:
                self._pub_vel(0.0, SEARCH_SPEED, 0.0, self.heading)
            else:
                self._pub_vel(0.0, 0.0, 0.0, self.heading)

        # ---- 视觉伺服 (PID + 高度锁定) ----
        elif self.step == STEP_TRACK:
            if not self._detect_fresh():
                self.get_logger().warn(f"Detection lost for cylinder {self._scout_index}, returning to SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_SEARCH)
                return

            roi = self.latest_roi
            v_fwd, v_lat, err_px = self.controller.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = (target_z - self.z) * 0.5
            self._pub_vel(vx, vy, vz, self.heading)

            if err_px < TRACK_ARRIVE_PX:
                if self._stabilize_start is None:
                    self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now() - self._stabilize_start).nanoseconds > STABILIZE_TIME * 1e9:
                    self._hover_target = (self.x, self.y, self.z)
                    self._hover_scout_start = None
                    self.get_logger().info(
                        f"TRACK converged (err={err_px:.0f}px) → HOVER for cylinder {self._scout_index}")
                    self._enter_step(STEP_HOVER)
            else:
                self._stabilize_start = None

        # ---- 悬停侦察 (位置保持 2s, 打印完成) ----
        elif self.step == STEP_HOVER:
            tx, ty, tz = self._hover_target
            self._pub_pos(tx, ty, tz, self.heading)

            if self._hover_scout_start is None:
                self._hover_scout_start = self.get_clock().now()

            hover_elapsed = (self.get_clock().now() - self._hover_scout_start).nanoseconds * 1e-9
            if hover_elapsed > HOVER_SCOUT_TIME:
                self.get_logger().info(f"侦察点 {self._scout_index + 1} 完成")
                self._scout_index += 1
                if self._scout_index < 5:
                    self._switch_target(self._scout_index)
                    self._enter_step(STEP_SEARCH)
                else:
                    self._enter_step(STEP_RESULT)

        # ---- 侦察完成 ----
        elif self.step == STEP_RESULT:
            # 保持当前位置，等待打印完成后切 RETURN
            tx, ty = self.x, self.y
            self._pub_pos(tx, ty, target_z, self.heading)
            result_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if result_elapsed > 1.0:
                self.get_logger().info("侦察任务完成")
                self._enter_step(STEP_RETURN)

        # ---- 返航 ----
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
                    self.get_logger().info("Scout mission complete — disarmed (landed)")
            elif abs(self.z - self.home_z) < 0.15:
                if self._land_dwell_start is None:
                    self._land_dwell_start = self.get_clock().now()
                elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                    self._send_cmd(400, param1=0.0)
                    self.timer.cancel()
                    self.get_logger().info("Scout mission complete — disarmed (altitude)")
            else:
                self._land_dwell_start = None


def main():
    rclpy.init()
    node = ScoutMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
