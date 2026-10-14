#!/usr/bin/env python3
"""
单桶投弹任务节点 — 机体坐标系，前飞 5m → 视觉伺服 → 投弹 → 返航着陆。

状态机:
  TAKEOFF → FORWARD → SEARCH → TRACK → DESCEND → HOVER → DROP
  → CLIMB → RETURN → HOVER_LAND → LAND

仿真/实机双模式:
  use_sim=true  : /spawn_entity 生成自由落体 (Gazebo 仿真)
  use_sim=false : VehicleCommand MAV_CMD_DO_SET_ACTUATOR 控制 AUX5 PWM (实机舵机)

检测源:
  仿真: single_bucket_detector (Ground Truth 投影)
  实机: YOLOv8 检测节点 (发布 /bucket/detection RegionOfInterest)
  任务节点不关心检测来源，只订阅 /bucket/detection
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected,
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from drone_drl.common import (
    QOS_BE, status_topic, local_pos_topic, send_arm, send_offboard,
    send_land, send_disarm,
)

# ---- 状态 ----
(STEP_TAKEOFF, STEP_FORWARD, STEP_SEARCH, STEP_TRACK,
 STEP_DESCEND, STEP_HOVER, STEP_DROP, STEP_CLIMB,
 STEP_RETURN, STEP_HOVER_LAND, STEP_LAND) = range(11)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD: "FORWARD",
    STEP_SEARCH: "SEARCH", STEP_TRACK: "TRACK",
    STEP_DESCEND: "DESCEND", STEP_HOVER: "HOVER",
    STEP_DROP: "DROP", STEP_CLIMB: "CLIMB",
    STEP_RETURN: "RETURN", STEP_HOVER_LAND: "HOV_LAND",
    STEP_LAND: "LAND",
}

# ---- 机体系航点: (前 m, 右 m, 绝对 NED z, 标签) ----
# 桶在机体正前方 5m 处
BODY_FORWARD_WP = (5.0, 0.0)

# ---- 参数 ----
CRUISE_HEIGHT = 3.5
DROP_HEIGHT = 1.0
MAX_HORIZ_SPEED = 1.5          # setpoint 水平速度 (m/s) — 平缓
MAX_VERT_SPEED = 0.30          # setpoint 垂直速度 (m/s) — 平缓
ARRIVE_DIST = 0.5
TAKEOFF_STABILIZE = 3.0
HOVER_LAND_STABLE_TIME = 2.0
HOVER_LAND_DRIFT_THRESH = 0.1
LAND_DWELL_TIME = 3.0
STEP_TIMEOUT = 120.0
ARM_DELAY = 2.0
TIMER_PERIOD = 0.05
SEARCH_SPEED = 0.5             # SEARCH 前飞速度 (m/s)
SEARCH_TIMEOUT = 30.0          # SEARCH 超时直接返航

# PID — 降低最大速度防止 overshoot，限制积分防止 windup
KP, KI, KD = 0.005, 0.00003, 0.0005
MAX_VEL = 1.0                  # 伺服最大速度 (m/s), 1.5→1.0 减少 overshoot
INT_LIMIT = 50.0               # 积分上限, 80→50 减少 windup

# 视觉伺服
TRACK_ARRIVE_PX = 50.0
HOVER_ARRIVE_PX = 25.0         # 低空锁定, 20→25 平衡精度与收敛速度
STABILIZE_TIME = 2.5           # 稳定计时, 3.0→2.5 提效
DESCEND_SPEED = 0.30           # 下降速度, 0.35→0.30 更平缓
TRACK_VZ_KP = 0.5
HOVER_VZ_KP = 0.8
DETECT_TIMEOUT = 3.0

# AUX5 PWM (实机)
AUX5_SERVO_INDEX = 5    # param1 for MAV_CMD_DO_SET_ACTUATOR (AUX5)
AUX5_PWM_LOCK = 1000.0  # microseconds — 锁定
AUX5_PWM_DROP = 1500.0  # microseconds — 投放

IMG_W, IMG_H = 640, 480


class VisualServoController:
    """PID 机体速度控制器 (像素偏差→机体速度)"""

    def __init__(self, kp, ki, kd, max_vel, int_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_vel = max_vel
        self.int_limit = int_limit
        self.reset()

    def reset(self):
        self.prev_ex = self.prev_ey = 0.0
        self.int_ex = self.int_ey = 0.0

    def compute(self, roi, dt):
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        ex = cx - IMG_W / 2.0
        ey = cy - IMG_H / 2.0

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
        return v_fwd, v_lat, math.hypot(ex, ey), ex, ey


class SingleDropMission(Node):
    def __init__(self):
        super().__init__('single_drop_mission')

        self.declare_parameter('use_sim', True)
        self._use_sim = self.get_parameter('use_sim').value
        self.declare_parameter('require_xy_valid', False)

        # PX4 输入
        self.status_sub = self.create_subscription(
            VehicleStatus, status_topic(), self._cb_status, QOS_BE)
        self.local_sub = self.create_subscription(
            VehicleLocalPosition, local_pos_topic(),
            self._cb_local, QOS_BE)
        self.land_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._cb_land, QOS_BE)

        # 视觉输入
        self.bucket_det_sub = self.create_subscription(
            RegionOfInterest, '/bucket/detection', self._cb_bucket_det, 10)
        self._bucket_roi = None
        self._last_bucket_det = self.get_clock().now()

        # 输出
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 飞机状态
        self.x = self.y = self.z = 0.0
        self.heading = 0.0
        self.heading0 = 0.0
        self.home_x = self.home_y = self.home_z = 0.0
        self.home_set = False
        self.nav_state = 0
        self.armed = False
        self.landed = False

        # 任务
        self.step = STEP_TAKEOFF
        self._step_start = None
        self._stabilize_start = None
        self._start_wall = self.get_clock().now()
        self._setpoint_start = self.get_clock().now()

        # 平滑 setpoint
        self._sp_x = self._sp_y = self._sp_z = 0.0
        self._sp_ready = False

        # PID
        self.ctrl = VisualServoController(KP, KI, KD, MAX_VEL, INT_LIMIT)

        # 投放
        self._drop_done = False
        self._drop_release_start = None

        # 落点记录 (取代 spawn_entity 的物理模拟)
        self.impact_pub = self.create_publisher(Point, '/drop/impact_point', 10)

        # AUX5 PWM 信号监控 (仿真+实机均可观测)
        self.aux5_pwm_pub = self.create_publisher(Float32, '/drop/aux5_pwm', 10)
        self._aux5_pwm_val = AUX5_PWM_LOCK
        self._publish_aux5("INIT", self._aux5_pwm_val)

        # 着陆
        self._land_cmd_sent = False
        self._land_dwell_start = None

        # SEARCH 状态
        self._search_sp_synced = False

        # 模式守卫
        self._offboard_started = False
        self._arm_attempted = False
        self._mode_req_start = self.get_clock().now()
        self._last_arm_attempt = -10.0

        self._print_ctr = 0
        self._last_pid_print = 0.0
        self._timer = self.create_timer(TIMER_PERIOD, self._loop)

        mode_str = "SIM" if self._use_sim else "REAL (AUX5 PWM)"
        self.get_logger().info(
            f"Single drop mission ready [{mode_str}] — "
            f"forward {BODY_FORWARD_WP[0]:.0f}m, "
            f"cruise={CRUISE_HEIGHT}m, drop_h={DROP_HEIGHT}m")

    # ===================== 回调 =====================

    def _cb_status(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == 2

    def _cb_local(self, msg):
        if self.get_parameter('require_xy_valid').value and not msg.xy_valid:
            return
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

    def _cb_bucket_det(self, msg):
        self._bucket_roi = msg
        self._last_bucket_det = self.get_clock().now()

    def _detect_fresh(self):
        if self._bucket_roi is None:
            return False
        dt = (self.get_clock().now() - self._last_bucket_det).nanoseconds * 1e-9
        return dt < DETECT_TIMEOUT

    # ===================== 工具 =====================

    def _elapsed(self, start_time):
        if start_time is None:
            return 0.0
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9

    def _dist_to(self, tx, ty, tz):
        return math.hypot(self.x - tx, self.y - ty, self.z - tz)

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        h = self.heading  # 视觉伺服用实时 heading
        c, s = math.cos(h), math.sin(h)
        return (v_fwd * c - v_lat * s,
                v_fwd * s + v_lat * c,
                v_down)

    def _wp_ned(self, wx, wy, wz_abs_ned):
        """机体系 (前=wx, 右=wy) → NED (北, 东), 绕 heading0 旋转"""
        c, s = math.cos(self.heading0), math.sin(self.heading0)
        tx = self.home_x + wx * c - wy * s
        ty = self.home_y + wx * s + wy * c
        tz = self.home_z + wz_abs_ned
        return tx, ty, tz

    def _update_setpoint(self, tx, ty, tz):
        if not self._sp_ready:
            return
        dt = TIMER_PERIOD
        dx, dy, dz = tx - self._sp_x, ty - self._sp_y, tz - self._sp_z

        h_dist = math.hypot(dx, dy)
        max_h = MAX_HORIZ_SPEED * dt
        if h_dist > max_h:
            r = max_h / h_dist
            self._sp_x += dx * r
            self._sp_y += dy * r
        else:
            self._sp_x, self._sp_y = tx, ty

        max_v = MAX_VERT_SPEED * dt
        if abs(dz) > max_v:
            self._sp_z += math.copysign(max_v, dz)
        else:
            self._sp_z = tz

    def _sp_dist_to(self, tx, ty, tz):
        return math.hypot(self._sp_x - tx, self._sp_y - ty, self._sp_z - tz)

    def _pub_sp(self):
        self.mode_pub.publish(OffboardControlMode(position=True))
        self.sp_pub.publish(TrajectorySetpoint(
            position=[float(self._sp_x), float(self._sp_y), float(self._sp_z)],
            yaw=float(self.heading0)))

    def _pub_vel(self, vx, vy, vz, yaw):
        self.mode_pub.publish(OffboardControlMode(velocity=True))
        self.sp_pub.publish(TrajectorySetpoint(
            velocity=[float(vx), float(vy), float(vz)],
            yaw=float(yaw),
            position=[float('nan')] * 3,
            acceleration=[float('nan')] * 3))

    def _pub_pos(self, x, y, z, yaw):
        self.mode_pub.publish(OffboardControlMode(position=True))
        self.sp_pub.publish(TrajectorySetpoint(
            position=[float(x), float(y), float(z)],
            yaw=float(yaw),
            velocity=[float('nan')] * 3,
            acceleration=[float('nan')] * 3))

    def _send_cmd(self, cmd, param1=0.0, param2=0.0, param3=0.0,
                  param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=cmd,
            param1=param1, param2=param2, param3=param3, param4=param4,
            param5=param5, param6=param6, param7=param7,
            target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True))

    def _enter_step(self, step):
        self.step = step
        self._step_start = self.get_clock().now()
        self.get_logger().info(f"→ {STEP_NAME.get(step, '???')}")

    def _sync_sp(self):
        """将内部平滑 setpoint 同步到无人机当前位置 (速度→位置控制切换时调用)"""
        self._sp_x, self._sp_y, self._sp_z = self.x, self.y, self.z
        self._sp_ready = True

    def _publish_aux5(self, label, pwm_us):
        """发布 AUX5 PWM 信号到 /drop/aux5_pwm 话题并打印醒目日志"""
        self.aux5_pwm_pub.publish(Float32(data=float(pwm_us)))
        bar = "=" * 56
        self.get_logger().info(bar)
        self.get_logger().info(
            f"  AUX5 PWM [{label}]: {self._aux5_pwm_val:.0f}us → {pwm_us:.0f}us "
            f"({'DROP !!!' if pwm_us == AUX5_PWM_DROP else 'LOCK' if pwm_us == AUX5_PWM_LOCK else '---'})")
        self.get_logger().info(bar)
        self._aux5_pwm_val = pwm_us

    # ===================== 主循环 =====================

    def _loop(self):
        if not self.home_set:
            return

        target_z = self.home_z - CRUISE_HEIGHT

        if self.step == STEP_TAKEOFF:
            self._run_takeoff(target_z)
        elif self.step == STEP_FORWARD:
            self._run_forward(target_z)
        elif self.step == STEP_SEARCH:
            self._run_search(target_z)
        elif self.step == STEP_TRACK:
            self._run_track(target_z)
        elif self.step == STEP_DESCEND:
            self._run_descend()
        elif self.step == STEP_HOVER:
            self._run_hover()
        elif self.step == STEP_DROP:
            self._run_drop()
        elif self.step == STEP_CLIMB:
            self._run_climb(target_z)
        elif self.step == STEP_RETURN:
            self._run_return(target_z)
        elif self.step == STEP_HOVER_LAND:
            self._run_hover_land(target_z)
        elif self.step == STEP_LAND:
            self._run_land(target_z)

        self._guard_mode()

        # 1Hz 日志
        self._print_ctr += 1
        if self._print_ctr >= 20:
            self._print_ctr = 0
            tag = STEP_NAME.get(self.step, "???")
            det = "DET" if self._detect_fresh() else "no-det"
            extra = ""
            if self.step in (STEP_TRACK, STEP_DESCEND, STEP_HOVER, STEP_DROP):
                if self._detect_fresh():
                    roi = self._bucket_roi
                    cx = roi.x_offset + roi.width / 2.0
                    cy = roi.y_offset + roi.height / 2.0
                    extra = f" err=({cx - IMG_W/2:.0f},{cy - IMG_H/2:.0f})px"
            elif self.step == STEP_FORWARD:
                wx, wy = BODY_FORWARD_WP
                tx, ty, _ = self._wp_ned(wx, wy, 0)
                extra = f" →fwd({tx:.1f},{ty:.1f}) dist={self._dist_to(tx, ty, self.z):.1f}m"
            elif self.step == STEP_RETURN:
                extra = f" →home dist={math.hypot(self.x - self.home_x, self.y - self.home_y):.1f}m"
            self.get_logger().info(
                f"[{tag}] NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                f"hdg={math.degrees(self.heading):.0f} {det}{extra}")

    # ===================== 各状态 =====================

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
            wx, wy = BODY_FORWARD_WP
            tx, ty, _ = self._wp_ned(wx, wy, 0)
            self.get_logger().info(
                f"Takeoff done → FORWARD to NED({tx:.1f},{ty:.1f})")
            self._enter_step(STEP_FORWARD)

    def _run_forward(self, target_z):
        """飞往机体前方 5m 航点，途中检测到桶则直接切入 TRACK"""
        wx, wy = BODY_FORWARD_WP
        tx, ty, _ = self._wp_ned(wx, wy, 0)
        tz = target_z

        self._update_setpoint(tx, ty, tz)
        self._pub_sp()

        # 途中检测到桶直接伺服
        if self._detect_fresh():
            self.get_logger().info("Detection during FORWARD → TRACK")
            self.ctrl.reset()
            self._stabilize_start = None
            self._enter_step(STEP_TRACK)
            return

        sp_dist = self._sp_dist_to(tx, ty, tz)
        aircraft_dist = self._dist_to(tx, ty, tz)

        if sp_dist < 0.05 and aircraft_dist < ARRIVE_DIST:
            self.get_logger().info("Forward wp reached, no detection → SEARCH")
            self._enter_step(STEP_SEARCH)

        if self._elapsed(self._step_start) > STEP_TIMEOUT:
            self.get_logger().error("FORWARD timeout → RETURN")
            self._enter_step(STEP_RETURN)

    def _run_search(self, target_z):
        """以预期桶位置(机体前方5m)为圆心、半径1m内螺旋搜索。

        阶段:
          0-2s:  原地悬停等检测恢复
          2-7s:  飞向圆心
          7-30s: 螺旋展开 r:0→1m, 角速度 0.5rad/s
        """
        if self._detect_fresh():
            self.get_logger().info("Detection found in SEARCH → TRACK")
            self.ctrl.reset()
            self._stabilize_start = None
            self._search_sp_synced = False
            self._enter_step(STEP_TRACK)
            return

        # 圆心: 机体前方 5m (桶预期位置)
        wx, wy = BODY_FORWARD_WP
        cx, cy, _ = self._wp_ned(wx, wy, 0)

        # 首次进入 SEARCH: 从速度控制同步 setpoint
        if not self._search_sp_synced:
            self._sync_sp()
            self._search_sp_synced = True

        elapsed = self._elapsed(self._step_start)

        if elapsed < 2.0:
            # 原地悬停, 等检测恢复
            self._update_setpoint(self.x, self.y, target_z)
            self._pub_sp()
        elif elapsed < 7.0:
            # 飞向圆心
            self._update_setpoint(cx, cy, target_z)
            self._pub_sp()
        else:
            # 螺旋: r: 0→1m (20s), θ: 0.5rad/s
            t = elapsed - 7.0
            r = min(1.0, t * 0.05)
            theta = t * 0.5
            sx = cx + r * math.cos(theta)
            sy = cy + r * math.sin(theta)
            self._update_setpoint(sx, sy, target_z)
            self._pub_sp()

        if elapsed > SEARCH_TIMEOUT:
            self._search_sp_synced = False
            self.get_logger().warn(
                f"SEARCH timeout ({SEARCH_TIMEOUT}s) → RETURN")
            self._enter_step(STEP_RETURN)

    def _run_track(self, target_z):
        if not self._detect_fresh():
            self.get_logger().warn("Detection lost in TRACK → SEARCH")
            self._stabilize_start = None
            self._enter_step(STEP_SEARCH)
            return

        roi = self._bucket_roi
        v_fwd, v_lat, err_px, ex, ey = self.ctrl.compute(roi, TIMER_PERIOD)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        vz = (target_z - self.z) * TRACK_VZ_KP
        self._pub_vel(vx, vy, vz, self.heading)

        sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
        if sim_t - self._last_pid_print > 1.0:
            self.get_logger().info(
                f"[TRACK] px_err=({ex:.0f},{ey:.0f}) total={err_px:.0f}px "
                f"vel=({vx:.3f},{vy:.3f})")
            self._last_pid_print = sim_t

        if err_px < TRACK_ARRIVE_PX:
            if self._stabilize_start is None:
                self._stabilize_start = self.get_clock().now()
            elif self._elapsed(self._stabilize_start) > STABILIZE_TIME:
                self.get_logger().info(
                    f"TRACK converged (err={err_px:.0f}px) → DESCEND")
                self._enter_step(STEP_DESCEND)
        else:
            self._stabilize_start = None

    def _run_descend(self):
        drop_z = self.home_z - DROP_HEIGHT

        if not self._detect_fresh():
            # 丢检测 → 悬停保持位置等检测恢复 (不前飞!)
            self._pub_vel(0.0, 0.0, 0.0, self.heading0)
            if self._elapsed(self._last_bucket_det) > 5.0:
                self.get_logger().warn("Detection lost 5s in DESCEND → climb")
                self._enter_step(STEP_CLIMB)
            return

        roi = self._bucket_roi
        v_fwd, v_lat, _, ex, ey = self.ctrl.compute(roi, TIMER_PERIOD)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        self._pub_vel(vx, vy, DESCEND_SPEED, self.heading)

        sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
        if sim_t - self._last_pid_print > 1.0:
            self.get_logger().info(
                f"[DESCEND] px_err=({ex:.0f},{ey:.0f}) "
                f"z={self.z:.2f}→{drop_z:.1f}")
            self._last_pid_print = sim_t

        if self.z > drop_z - 0.2:
            self._stabilize_start = None
            self.get_logger().info(
                f"At drop altitude (z={self.z:.1f}m) → HOVER")
            self._enter_step(STEP_HOVER)

    def _run_hover(self):
        drop_z = self.home_z - DROP_HEIGHT

        if not self._detect_fresh():
            # 丢检测 → 悬停保持位置等检测恢复
            self._pub_vel(0.0, 0.0, 0.0, self.heading0)
            self._stabilize_start = None
            if self._elapsed(self._last_bucket_det) > 5.0:
                self.get_logger().warn("Detection lost 5s in HOVER → climb")
                self._enter_step(STEP_CLIMB)
            return

        roi = self._bucket_roi
        v_fwd, v_lat, err_px, ex, ey = self.ctrl.compute(roi, TIMER_PERIOD)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        vz = (drop_z - self.z) * HOVER_VZ_KP
        vz = max(-0.3, min(0.3, vz))
        self._pub_vel(vx, vy, vz, self.heading)

        sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
        if sim_t - self._last_pid_print > 1.0:
            self.get_logger().info(
                f"[HOVER] px_err=({ex:.0f},{ey:.0f}) total={err_px:.0f}px "
                f"z={self.z:.2f}")
            self._last_pid_print = sim_t

        if err_px < HOVER_ARRIVE_PX:
            if self._stabilize_start is None:
                self._stabilize_start = self.get_clock().now()
            elif self._elapsed(self._stabilize_start) > STABILIZE_TIME:
                self.get_logger().info(
                    f"HOVER converged (err={err_px:.0f}px) → DROP")
                self._drop_done = False
                self._enter_step(STEP_DROP)
        else:
            self._stabilize_start = None

    def _run_drop(self):
        drop_z = self.home_z - DROP_HEIGHT

        # 保持水平 PID 锁定
        if self._detect_fresh():
            roi = self._bucket_roi
            v_fwd, v_lat, _, _, _ = self.ctrl.compute(roi, TIMER_PERIOD)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        else:
            vx, vy = 0.0, 0.0
        vz = (drop_z - self.z) * HOVER_VZ_KP
        vz = max(-0.2, min(0.2, vz))
        self._pub_vel(vx, vy, vz, self.heading)

        if not self._drop_done:
            if self._use_sim:
                self._drop_sim()
            else:
                self._drop_real()
            self._drop_done = True

        # 投放后悬停 2s 等载荷分离
        if self._elapsed(self._step_start) > 2.0:
            self.get_logger().info("Drop complete → CLIMB")
            self._enter_step(STEP_CLIMB)

    def _drop_sim(self):
        """仿真: 记录飞机几何中心在地面的投影作为投放落点 (不生成物理载荷)"""
        impact_x = self.x
        impact_y = self.y
        impact_z = self.home_z  # 地面高度

        # 发布落点
        self.impact_pub.publish(Point(x=impact_x, y=impact_y, z=impact_z))

        self.get_logger().info(
            f"DROP IMPACT → NED({impact_x:.3f},{impact_y:.3f}) ground_z={impact_z:.2f}")

        # AUX5 PWM 信号输出
        self._send_cmd(
            187,  # VEHICLE_CMD_DO_SET_ACTUATOR
            param1=float(AUX5_SERVO_INDEX),
            param2=AUX5_PWM_DROP)
        self._drop_release_start = self.get_clock().now()
        self._publish_aux5("DROP", AUX5_PWM_DROP)

    def _drop_real(self):
        """实机: AUX5 PWM 1500us 触发舵机投放"""
        self._send_cmd(
            187,  # VEHICLE_CMD_DO_SET_ACTUATOR
            param1=float(AUX5_SERVO_INDEX),
            param2=AUX5_PWM_DROP)
        self._drop_release_start = self.get_clock().now()
        self._publish_aux5("DROP", AUX5_PWM_DROP)

    def _run_climb(self, target_z):
        """爬升回巡航高度，锁定水平位置防止漂移"""
        # 进入 CLIMB 时锁定当前水平位置 + 同步 setpoint (从速度控制切到位置控制)
        if not hasattr(self, '_climb_locked_x') or self._climb_locked_x is None:
            self._climb_locked_x = self.x
            self._climb_locked_y = self.y
            self._sync_sp()
            self.get_logger().info(
                f"CLIMB locked at NED({self._climb_locked_x:.2f},{self._climb_locked_y:.2f})")

        # 投放后恢复锁定 PWM (仿真+实机均发送，方便观测信号变化)
        if self._drop_release_start is not None:
            if self._elapsed(self._drop_release_start) > 0.5:
                self._send_cmd(
                    187,
                    param1=float(AUX5_SERVO_INDEX),
                    param2=AUX5_PWM_LOCK)
                self._drop_release_start = None
                self._publish_aux5("LOCK", AUX5_PWM_LOCK)

        self._update_setpoint(self._climb_locked_x, self._climb_locked_y, target_z)
        self._pub_sp()

        if abs(self.z - target_z) < 0.8:
            self.get_logger().info("Climb done → RETURN")
            self._climb_locked_x = self._climb_locked_y = None
            self._enter_step(STEP_RETURN)

        if self._elapsed(self._step_start) > STEP_TIMEOUT:
            self.get_logger().warn("CLIMB timeout → RETURN")
            self._climb_locked_x = self._climb_locked_y = None
            self._enter_step(STEP_RETURN)

    def _run_return(self, target_z):
        # 首次进入 RETURN 时同步 setpoint (从 CLIMB 的位置控制可能不同步)
        if not hasattr(self, '_return_sp_synced') or not self._return_sp_synced:
            self._sync_sp()
            self._return_sp_synced = True
            self.get_logger().info(
                f"RETURN sp synced from NED({self.x:.2f},{self.y:.2f},{self.z:.2f})")
        self._update_setpoint(self.home_x, self.home_y, target_z)
        self._pub_sp()

        dist = self._dist_to(self.home_x, self.home_y, target_z)
        if dist < ARRIVE_DIST:
            self.get_logger().info(f"Returned home (dist={dist:.2f}m) → HOVER_LAND")
            self._stabilize_start = None
            self._enter_step(STEP_HOVER_LAND)

        if self._elapsed(self._step_start) > STEP_TIMEOUT:
            self.get_logger().error("RETURN timeout → HOVER_LAND")
            self._enter_step(STEP_HOVER_LAND)

    def _run_hover_land(self, target_z):
        lx, ly = self.home_x, self.home_y
        self._update_setpoint(lx, ly, target_z)
        self._pub_sp()

        drift = math.hypot(self.x - lx, self.y - ly)
        if drift < HOVER_LAND_DRIFT_THRESH:
            if self._stabilize_start is None:
                self._stabilize_start = self.get_clock().now()
            elif self._elapsed(self._stabilize_start) > HOVER_LAND_STABLE_TIME:
                self.get_logger().info(
                    f"Hover stable (drift={drift:.2f}m) → LAND")
                self._enter_step(STEP_LAND)
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
            if self._land_dwell_start is None:
                self._land_dwell_start = self.get_clock().now()
            elif self._elapsed(self._land_dwell_start) > LAND_DWELL_TIME:
                self.get_logger().info("Land complete — disarming")
                self._send_cmd(400, param1=0.0)
                rclpy.shutdown()

        if self.z > self.home_z - 0.15 and self._land_cmd_sent:
            if not hasattr(self, '_fb_start'):
                self._fb_start = self.get_clock().now()
            elif self._elapsed(self._fb_start) > 3.0:
                self.get_logger().info("Height fallback disarm")
                self._send_cmd(400, param1=0.0)
                rclpy.shutdown()

    # ===================== 模式守卫 =====================

    def _guard_mode(self):
        if self.step == STEP_LAND:
            return

        sp_elapsed = self._elapsed(self._setpoint_start)
        if sp_elapsed < ARM_DELAY:
            return

        if not self._offboard_started:
            if not self.armed:
                now = self._elapsed(self._start_wall)
                if not hasattr(self, '_last_arm_try'):
                    self._last_arm_try = -10.0
                if now - self._last_arm_try > 1.0:
                    self._send_cmd(400, param1=1.0, param2=2989.0)
                    self._last_arm_try = now
                    if not self._arm_attempted:
                        self._arm_attempted = True
                        self.get_logger().info("Arming...")
                return

            if self.nav_state != 14:
                self._send_cmd(176, param1=1.0, param2=6.0)
                return

            self._offboard_started = True
            self.get_logger().info("OFFBOARD mode active")
            return

        if self.nav_state not in (14, 18):
            self._send_cmd(176, param1=1.0, param2=6.0)
            self.get_logger().warn(
                f"Lost OFFBOARD (nav={self.nav_state}), re-requesting")


def main(args=None):
    rclpy.init(args=args)
    node = SingleDropMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
