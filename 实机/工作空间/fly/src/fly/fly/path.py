#!/usr/bin/env python3
"""
轨迹复现节点 — 纯位置控制复现 v2 全任务飞行轨迹（真机版）。
平滑限速版: setpoint 以固定速度移向目标, PX4 跟随平缓无剧烈机动。

状态机: TAKEOFF → MISSION(航点序列) → HOVER_LAND → LAND(PX4 auto-land)

真机适配：车辆状态话题改为 /fmu/out/vehicle_status_v1
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleControlMode, VehicleLocalPosition, VehicleLandDetected,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 状态 ----
(STEP_TAKEOFF, STEP_MISSION, STEP_HOVER_LAND, STEP_LAND) = range(4)

# ---- 航点序列 (机体系: x=前 m, y=右 m, z=绝对 NED, 悬停秒, 标签) ----
# 运行时自动绕初始 heading0 旋转到 NED: North=x*cos(h0)-y*sin(h0), East=x*sin(h0)+y*cos(h0)
WAYPOINTS = [
    (35.0,  0.0, -3.5, 3.0, "center_hover"),
    (34.5, -1.0, -3.5, 2.0, "b0_approach"),
    (34.5, -1.0, -1.0, 5.0, "b0_low"),
    (34.5, -1.0, -3.5, 2.0, "b0_climb"),
    (36.0,  0.0, -3.5, 2.0, "b1_approach"),
    (36.0,  0.0, -1.0, 5.0, "b1_low"),
    (36.0,  0.0, -3.5, 2.0, "b1_climb"),
    (57.5,  0.0, -3.5, 5.0, "scout_hover"),
    (0.0,   0.0, -3.5, 0.0, "return_home"),
]

# ---- 参数 ----
MAX_HORIZ_SPEED = 2.0          # setpoint 水平移动速度上限 (m/s)
MAX_VERT_SPEED = 0.35          # setpoint 垂直移动速度上限 (m/s), 平缓下降
CRUISE_HEIGHT = 3.5
ARRIVE_DIST = 0.5
TAKEOFF_STABILIZE = 3.0
HOVER_LAND_STABLE_TIME = 2.0
HOVER_LAND_DRIFT_THRESH = 0.1
LAND_DWELL_TIME = 3.0
STEP_TIMEOUT = 300.0
ARM_DELAY = 2.0
TIMER_PERIOD = 0.05


class PathFollow(Node):

    def __init__(self):
        super().__init__("path_follow")

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10,
        )

        # ---------- 订阅者（vehicle_status_v1 有 DDS type checksum 不匹配, 改用 control_mode）----------
        self.control_mode_sub = self.create_subscription(
            VehicleControlMode, "/fmu/out/vehicle_control_mode", self._cb_control_mode, qos_be)
        self.local_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
            self._cb_local, qos_be)
        self.land_sub = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self._cb_land, qos_be)

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.x = self.y = self.z = 0.0
        self.heading = 0.0
        self.heading0 = 0.0           # 起飞时机头朝向, 用于航点旋转
        self.home_x = self.home_y = self.home_z = 0.0
        self.home_set = False
        self._control_mode = None
        self.landed = False

        self.step = STEP_TAKEOFF
        self._step_start = None
        self._wp_idx = 0
        self._hold_start = None
        self._wp_reached = False
        self._land_cmd_sent = False
        self._land_dwell_start = None
        self._offboard_started = False
        self._arm_attempted = False
        self._stabilize_start = None
        self._start_wall = self.get_clock().now()
        self._setpoint_start = self.get_clock().now()

        # 平滑 setpoint 状态
        self._sp_x = self._sp_y = self._sp_z = 0.0
        self._sp_ready = False

        self._timer = self.create_timer(TIMER_PERIOD, self._loop)

        self.get_logger().info(
            f"PathFollow ready — {len(WAYPOINTS)} waypoints, "
            f"max_h={MAX_HORIZ_SPEED}m/s max_v={MAX_VERT_SPEED}m/s")

    # ---- 回调 ----
    def _cb_control_mode(self, msg):
        self._control_mode = msg

    def _cb_local(self, msg):
        # 室内无 GPS, xy_valid 常为 false, 移除该条件
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        self.heading = msg.heading
        if not self.home_set:
            self.home_x, self.home_y, self.home_z = msg.x, msg.y, msg.z
            self.heading0 = msg.heading
            self._sp_x, self._sp_y, self._sp_z = msg.x, msg.y, msg.z
            self.home_set = True
            self._sp_ready = True
            self.get_logger().info(
                f"Home: NED({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                f"heading0={math.degrees(self.heading0):.0f}deg")

    def _cb_land(self, msg):
        self.landed = msg.landed

    # ---- 工具 ----
    def _elapsed(self, start_time):
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9

    def _dist_to(self, tx, ty, tz):
        return math.sqrt((self.x - tx) ** 2 + (self.y - ty) ** 2 + (self.z - tz) ** 2)

    def _update_setpoint(self, tx, ty, tz):
        """以限速平滑移动内部 setpoint 向目标靠近"""
        if not self._sp_ready:
            return
        dt = TIMER_PERIOD
        dx, dy, dz = tx - self._sp_x, ty - self._sp_y, tz - self._sp_z

        # 水平限速
        h_dist = math.hypot(dx, dy)
        max_h = MAX_HORIZ_SPEED * dt
        if h_dist > max_h:
            r = max_h / h_dist
            self._sp_x += dx * r
            self._sp_y += dy * r
        else:
            self._sp_x, self._sp_y = tx, ty

        # 垂直限速
        max_v = MAX_VERT_SPEED * dt
        if abs(dz) > max_v:
            self._sp_z += math.copysign(max_v, dz)
        else:
            self._sp_z = tz

    def _sp_dist_to(self, tx, ty, tz):
        """平滑 setpoint 到目标的距离"""
        return math.sqrt((self._sp_x - tx) ** 2 + (self._sp_y - ty) ** 2 + (self._sp_z - tz) ** 2)

    def _pub_sp(self):
        """发布当前平滑 setpoint"""
        self.mode_pub.publish(OffboardControlMode(position=True))
        self.sp_pub.publish(TrajectorySetpoint(
            position=[float(self._sp_x), float(self._sp_y), float(self._sp_z)],
            yaw=float(self.heading0)))

    def _send_cmd(self, cmd, param1=0.0, param2=0.0):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=cmd, param1=param1, param2=param2,
            target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True))

    def _enter_step(self, step):
        self.step = step
        self._step_start = self.get_clock().now()

    # ---- 主循环 ----
    def _loop(self):
        if not self.home_set:
            return

        target_z = self.home_z - CRUISE_HEIGHT

        if self.step == STEP_TAKEOFF:
            self._run_takeoff(target_z)
        elif self.step == STEP_MISSION:
            self._run_mission(target_z)
        elif self.step == STEP_HOVER_LAND:
            self._run_hover_land(target_z)
        elif self.step == STEP_LAND:
            self._run_land(target_z)

        self._guard_mode()

    # ---- 状态 ----
    def _run_takeoff(self, target_z):
        if self._step_start is None:
            self._step_start = self.get_clock().now()

        elapsed = self._elapsed(self._step_start)

        if elapsed < TAKEOFF_STABILIZE:
            self._update_setpoint(self.home_x, self.home_y, self.home_z)
        else:
            self._update_setpoint(self.home_x, self.home_y, target_z)

        self._pub_sp()

        if abs(self.z - target_z) < 0.8 and abs(self.z - self.home_z) > 1.5:
            self.get_logger().info(
                f"Takeoff done → MISSION ({len(WAYPOINTS)} wp)")
            self._wp_idx = 0
            self._wp_reached = False
            self._hold_start = None
            self._enter_step(STEP_MISSION)

    def _run_mission(self, target_z):
        if self._wp_idx >= len(WAYPOINTS):
            self.get_logger().info("All waypoints done → HOVER_LAND")
            self._enter_step(STEP_HOVER_LAND)
            return

        wx, wy, wz, hold_s, label = WAYPOINTS[self._wp_idx]
        # 机体系(前=wx, 右=wy) → NED(北, 东), 绕初始 heading 旋转
        c, s = math.cos(self.heading0), math.sin(self.heading0)
        tx = self.home_x + wx * c - wy * s
        ty = self.home_y + wx * s + wy * c
        tz = self.home_z + wz

        self._update_setpoint(tx, ty, tz)
        self._pub_sp()

        sp_dist = self._sp_dist_to(tx, ty, tz)
        aircraft_dist = self._dist_to(tx, ty, tz)

        # 平滑 setpoint 到达 + 飞机跟上才算到达
        if sp_dist < 0.05 and aircraft_dist < ARRIVE_DIST:
            if not self._wp_reached:
                self.get_logger().info(
                    f"WP[{self._wp_idx}] {label} "
                    f"NED({tx:.1f},{ty:.1f},{tz:.1f}) "
                    f"dist={aircraft_dist:.2f}m")
                self._wp_reached = True
                if hold_s > 0:
                    self._hold_start = self.get_clock().now()
                    self.get_logger().info(f"  holding {hold_s:.0f}s...")

            if hold_s <= 0:
                self._wp_idx += 1
                self._wp_reached = False
                self._hold_start = None
            elif self._hold_start and self._elapsed(self._hold_start) > hold_s:
                self.get_logger().info(f"  hold done → next")
                self._wp_idx += 1
                self._wp_reached = False
                self._hold_start = None
        else:
            self._wp_reached = False
            self._hold_start = None

        if self._elapsed(self._step_start) > STEP_TIMEOUT:
            self.get_logger().error(
                f"Mission timeout at wp[{self._wp_idx}] → HOVER_LAND")
            self._enter_step(STEP_HOVER_LAND)

    def _run_hover_land(self, target_z):
        lx, ly = self.home_x, self.home_y
        self._update_setpoint(lx, ly, target_z)
        self._pub_sp()

        drift = math.hypot(self.x - lx, self.y - ly)
        if drift < HOVER_LAND_DRIFT_THRESH:
            if not self._stabilize_start:
                self._stabilize_start = self.get_clock().now()
            elif self._elapsed(self._stabilize_start) > HOVER_LAND_STABLE_TIME:
                self.get_logger().info(f"Hover stable (drift={drift:.2f}m) → LAND")
                self._enter_step(STEP_LAND)
                return
        else:
            self._stabilize_start = None

    def _run_land(self, target_z):
        if not self._land_cmd_sent:
            self._send_cmd(21)  # MAV_CMD_NAV_LAND
            self._land_cmd_sent = True
            self.get_logger().info("MAV_CMD_NAV_LAND sent")

        self._update_setpoint(self.home_x, self.home_y, self.z)
        self._pub_sp()

        if self.landed:
            if not self._land_dwell_start:
                self._land_dwell_start = self.get_clock().now()
            elif self._elapsed(self._land_dwell_start) > LAND_DWELL_TIME:
                self.get_logger().info("Land complete — disarming")
                self._send_cmd(400, 0.0)
                rclpy.shutdown()

        if self.z > self.home_z - 0.15 and self._land_cmd_sent:
            if not hasattr(self, "_fb_start"):
                self._fb_start = self.get_clock().now()
            elif self._elapsed(self._fb_start) > 3.0:
                self.get_logger().info("Height fallback disarm")
                self._send_cmd(400, 0.0)
                rclpy.shutdown()

    # ---- 模式守卫 (VehicleControlMode, 已验证 DDS 可用) ----
    def _guard_mode(self):
        if self.step == STEP_LAND:
            return

        cm = self._control_mode
        if cm is None:
            return

        sp_elapsed = self._elapsed(self._setpoint_start)
        if sp_elapsed < ARM_DELAY:
            return

        if not self._offboard_started:
            if not cm.flag_armed:
                now = self._elapsed(self._start_wall)
                if not hasattr(self, "_last_arm_attempt"):
                    self._last_arm_attempt = -10.0
                if now - self._last_arm_attempt > 1.0:
                    self._send_cmd(400, param1=1.0, param2=2989.0)
                    self._last_arm_attempt = now
                    if not self._arm_attempted:
                        self._arm_attempted = True
                        self.get_logger().info("Arming...")
                return

            if not cm.flag_control_offboard_enabled:
                self._send_cmd(176, param1=1.0, param2=6.0)
                return

            self._offboard_started = True
            self.get_logger().info("OFFBOARD mode active")
            return

        if not cm.flag_control_offboard_enabled:
            self._send_cmd(176, param1=1.0, param2=6.0)
            self.get_logger().warn("Lost OFFBOARD, re-requesting")


def main(args=None):
    rclpy.init(args=args)
    node = PathFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()