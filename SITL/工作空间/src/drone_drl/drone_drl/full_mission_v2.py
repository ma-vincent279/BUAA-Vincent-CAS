#!/usr/bin/env python3
"""
全任务节点 v2 — A3 方案: 中心悬停 + 视觉面积排序选择投弹目标。
投放阶段: 飞到三桶几何中心 → 悬停等 detector 初始化选最大两桶 → 依次伺服投弹。
侦察阶段: 蛇形扫描 5 筒 (不变)。
使用 yolo_bucket_detector (多 bbox → 位置记忆匹配 → 单 ROI)。

状态机:
  TAKEOFF → FORWARD_CENTER → CENTER_SEARCH → TRACK → DESCEND → HOVER → DROP(目标0)
  → CLIMB_SWITCH → CENTER_SEARCH → TRACK → DESCEND → HOVER → DROP(目标1)
  → RESULT_DROP → FORWARD_SCOUT → SCAN → RESULT_SCOUT → RETURN
  → HOVER_LAND(降落点悬停稳定) → LAND(纯垂直两段式)

与 v1 的区别:
  - 取消 B1_WP/B2_WP 硬编码航点，无人机不预知桶位置
  - 新增 CENTER_SEARCH: 在三桶几何中心悬停，等待 detector 自主初始化选目标
  - 新增 CLIMB_SWITCH: 投完目标0 后爬升回巡航高度 + 切换目标1
  - 桶目标选择权完全在 detector 侧 (面积排序)，任务节点仅等待 ROI
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 状态 ----
(STEP_TAKEOFF, STEP_FORWARD_CENTER, STEP_CENTER_SEARCH, STEP_TRACK,
 STEP_DESCEND, STEP_HOVER, STEP_DROP, STEP_CLIMB_SWITCH,
 STEP_RESULT_DROP, STEP_FORWARD_SCOUT, STEP_SCAN, STEP_RESULT_SCOUT,
 STEP_RETURN, STEP_HOVER_LAND, STEP_LAND) = range(15)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD_CENTER: "FORWARD_CTR",
    STEP_CENTER_SEARCH: "CTR_SEARCH", STEP_TRACK: "TRACK",
    STEP_DESCEND: "DESCEND", STEP_HOVER: "HOVER", STEP_DROP: "DROP",
    STEP_CLIMB_SWITCH: "CLIMB_SW", STEP_RESULT_DROP: "RESULT_DROP",
    STEP_FORWARD_SCOUT: "FWD_SCOUT", STEP_SCAN: "SCAN",
    STEP_RESULT_SCOUT: "RESULT_SCOUT", STEP_RETURN: "RETURN",
    STEP_HOVER_LAND: "HOV_LAND", STEP_LAND: "LAND",
}

# ---- 任务阶段 ----
(PHASE_BUCKET0, PHASE_BUCKET1, PHASE_SCOUT) = range(3)

# ---- 三桶几何中心 NED 偏移 (相对 home) ----
# 桶1 NED(1.02, 34.50), 桶2 NED(0.02, 36.00), 桶3 NED(-0.98, 34.50)
# 中心: (0.02, 35.00) ≈ (0, 35)
CENTER_X = 0.0
CENTER_Y = 35.0

# ---- 侦察区入口 ----
SCOUT_WP_X = 0.0
SCOUT_WP_Y = 57.5

# ---- 侦察蛇形扫描航点 (NED 偏移, target_id) ----
ZIGZAG_SCAN = [
    (2.5, 58.5, 0),
    (-2.5, 59.5, 1),
    (2.5, 60.5, 2),
    (-2.5, 61.5, 3),
    (2.0, 62.5, 4),
    (0.0, 63.5, -1),
]

# ---- 着陆区 ----
LAND_WP_X = 0.0
LAND_WP_Y = 0.0

# ---- 参数 ----
CRUISE_HEIGHT = 3.5
DROP_HEIGHT = 1.0
CRUISE_SPEED = 1.0
CLIMB_SPEED = 0.5          # 爬升速度 (m/s, NED z-)
SEARCH_SPEED = 0.5
STEP_TIMEOUT = 120.0
SCAN_ARRIVE_DIST = 2.0
SCAN_SPEED = 2.0
POS_ARRIVE_DIST = 0.5
LAND_DWELL_TIME = 3.0
CENTER_TIMEOUT = 30.0      # 中心悬停等待 detector 初始化的超时
CLIMB_TIMEOUT = 20.0       # 爬升+切换目标超时

# PID 桶（近程）
KP_B, KI_B, KD_B = 0.005, 0.00003, 0.0005  # iter2 最优: 精度0.05m + 197s
MAX_VEL_B = 1.5
INT_LIMIT_B = 80.0

# PID 筒（远程）
KP_S, KI_S, KD_S = 0.0015, 0.00003, 0.0005
MAX_VEL_S = 0.6
INT_LIMIT_S = 40.0

DETECT_TIMEOUT = 3.0
TRACK_ARRIVE_PX = 50.0
HOVER_ARRIVE_PX = 25.0
STABILIZE_TIME = 3.0
DESCEND_SPEED = 0.35
TRACK_VZ_KP = 0.5
HOVER_VZ_KP = 0.8
HOVER_SCOUT_TIME = 2.0

# 降落
HOVER_LAND_STABLE_TIME = 2.0
HOVER_LAND_DRIFT_THRESH = 0.1
IMG_W = 640
IMG_H = 480


class VisualServoController:
    """PID 机体速度控制器"""

    def __init__(self, kp, ki, kd, max_vel, int_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_vel = max_vel
        self.int_limit = int_limit
        self.prev_ey = self.prev_ex = 0.0
        self.int_ex = self.int_ey = 0.0

    def reset(self):
        self.prev_ex = self.prev_ey = 0.0
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
        return v_fwd, v_lat, math.hypot(ex, ey), ex, ey


class FullMissionV2(Node):
    def __init__(self):
        super().__init__('full_mission_v2')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        # Pub
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.scout_reached_pub = self.create_publisher(Int32, '/scout/point_reached', 10)
        self.bucket_target_pub = self.create_publisher(Int32, '/bucket/target_id', 10)
        self.scout_target_pub = self.create_publisher(Int32, '/scout/target_id', 10)

        # Sub — PX4
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._cb_local, qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._cb_land, qos)

        # Sub — 检测
        self.bucket_det_sub = self.create_subscription(RegionOfInterest, '/bucket/detection', self._cb_bucket_det, 10)
        self.scout_det_sub = self.create_subscription(RegionOfInterest, '/scout/detection', self._cb_scout_det, 10)
        self._bucket_roi = None
        self._scout_roi = None
        self._last_bucket_det = self.get_clock().now()
        self._last_scout_det = self.get_clock().now()

        # Sub — 侦察识别结果
        self.scout_result_sub = self.create_subscription(String, '/scout/result', self._cb_scout_result, 10)
        self._scout_result = None

        # 飞机状态
        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self.heading = None
        self.landed = False
        self.status = None

        # 任务
        self.step = STEP_TAKEOFF
        self._task_phase = PHASE_BUCKET0
        self._scout_index = 0
        self._step_start_time = None
        self._zigzag_idx = 0
        self._scout_reported = set()

        # PID 控制器
        self.ctrl_bucket = VisualServoController(KP_B, KI_B, KD_B, MAX_VEL_B, INT_LIMIT_B)
        self.ctrl_scout = VisualServoController(KP_S, KI_S, KD_S, MAX_VEL_S, INT_LIMIT_S)
        self._ctrl = self.ctrl_bucket

        self._hover_target = None
        self._stabilize_start = None

        # 投放
        self.drop_client = self.create_client(Trigger, '/drop_payload')
        self._drop_future = None

        # 计时
        self._arm_time = None
        self._disarm_time = None
        self._final_summary_printed = False

        # 降落
        self._land_dwell_start = None
        self._land_cmd_sent = False

        # 模式
        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._last_arm_attempt = -10.0
        self._init_wait_start = None
        self._initial_wait_done = False

        self._print_ctr = 0
        self._start_wall = self.get_clock().now()
        self._last_pid_print = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            "Full mission v2 ready: A3 center-hover + visual size-sort → "
            "drop zone ~35m (dual-bucket) → scout zone ~57m (5-cyl)")

    # ===================== 回调 =====================

    def _cb_status(self, m):
        if self.status is None:
            self.get_logger().info(
                f"First status: arm={m.arming_state} nav={m.nav_state} "
                f"failsafe={m.failsafe} preflight_ok={m.pre_flight_checks_pass}")
        elif m.failsafe != self.status.failsafe:
            self.get_logger().warn(
                f"FAILSAFE changed to {m.failsafe} nav={m.nav_state} arm={m.arming_state}")
        elif m.nav_state != self.status.nav_state:
            self.get_logger().info(f"nav_state: {self.status.nav_state} → {m.nav_state}")
        if self._arm_time is None and self.status is not None:
            if self.status.arming_state != 2 and m.arming_state == 2:
                self._arm_time = self.get_clock().now()
                self.get_logger().info(">>> TIMER START (armed) <<<")
        if self._arm_time is not None and self._disarm_time is None:
            if self.status.arming_state == 2 and m.arming_state != 2:
                self._disarm_time = self.get_clock().now()
                self.get_logger().info(">>> TIMER STOP (disarmed) <<<")
        self.status = m

    def _cb_land(self, m):
        self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading
        if self.home_x is None and m.xy_valid:
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.get_logger().info(
                f"Home: NED({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                f"heading={math.degrees(self.heading):.0f}deg")

    def _cb_bucket_det(self, m):
        self._bucket_roi = m
        self._last_bucket_det = self.get_clock().now()

    def _cb_scout_det(self, m):
        self._scout_roi = m
        self._last_scout_det = self.get_clock().now()

    def _cb_scout_result(self, m):
        self._scout_result = m.data

    # ===================== 发布 / 辅助 =====================

    def _pub_vel(self, vx, vy, vz, yaw):
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, velocity=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            velocity=[float(vx), float(vy), float(vz)], yaw=float(yaw),
            position=[float('nan')] * 3, acceleration=[float('nan')] * 3))

    def _pub_pos(self, x, y, z, yaw):
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, position=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=[float(x), float(y), float(z)], yaw=float(yaw),
            velocity=[float('nan')] * 3, acceleration=[float('nan')] * 3))

    def _send_cmd(self, cmd, **p):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=cmd, target_system=1, target_component=1,
            source_system=1, source_component=1, from_external=True, **p))

    def _enter_step(self, step):
        self.step = step
        self._step_start_time = self.get_clock().now()

    def _active_roi(self):
        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            return self._bucket_roi
        return self._scout_roi

    def _active_last_det(self):
        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            return self._last_bucket_det
        return self._last_scout_det

    def _detect_fresh(self):
        roi = self._active_roi()
        if roi is None:
            return False
        return (self.get_clock().now() - self._active_last_det()).nanoseconds * 1e-9 < DETECT_TIMEOUT

    def _switch_bucket_target(self, idx):
        msg = Int32()
        msg.data = idx
        self.bucket_target_pub.publish(msg)
        self.get_logger().info(f"Bucket target → {idx}")

    def _switch_scout_target(self, idx):
        msg = Int32()
        msg.data = idx
        self.scout_target_pub.publish(msg)
        self.get_logger().info(f"Scout target → cylinder {idx}")

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * cos_h - v_lat * sin_h,
                v_fwd * sin_h + v_lat * cos_h,
                v_down)

    def _dist_to(self, x, y, z):
        return math.hypot(self.x - x, self.y - y, self.z - z)

    # ===================== 主循环 =====================

    def control_loop(self):
        if self.home_x is None or self.heading is None:
            return

        target_z = self.home_z - CRUISE_HEIGHT

        # 1Hz 日志
        self._print_ctr += 1
        if self._print_ctr >= 20:
            tag = STEP_NAME.get(self.step, "???")
            hdg = math.degrees(self.heading)
            det = "DET" if self._detect_fresh() else "no-det"
            extra = ""
            if self._detect_fresh() and self.step in (STEP_TRACK, STEP_DESCEND, STEP_HOVER, STEP_DROP):
                roi = self._active_roi()
                cx = roi.x_offset + roi.width / 2.0
                cy = roi.y_offset + roi.height / 2.0
                ex, ey = cx - IMG_W / 2.0, cy - IMG_H / 2.0
                phase = {PHASE_BUCKET0: "B0", PHASE_BUCKET1: "B1", PHASE_SCOUT: f"S{self._scout_index}"}
                extra = f" {phase.get(self._task_phase, '?')} px_err=({ex:.0f},{ey:.0f})"
            elif self.step == STEP_FORWARD_CENTER:
                tx = self.home_x + CENTER_X
                ty = self.home_y + CENTER_Y
                extra = f" →center({tx:.1f},{ty:.1f}) dist={math.hypot(self.x - tx, self.y - ty):.1f}m"
            elif self.step == STEP_RETURN:
                extra = f" →home dist={math.hypot(self.x - self.home_x, self.y - self.home_y):.1f}m"
            elif self.step == STEP_HOVER_LAND:
                drift = math.hypot(self.x - self.home_x - LAND_WP_X, self.y - self.home_y - LAND_WP_Y)
                extra = f" hover@land drift={drift:.2f}m"
            elif self.step == STEP_FORWARD_SCOUT:
                tx = self.home_x + SCOUT_WP_X
                ty = self.home_y + SCOUT_WP_Y
                extra = f" →scout({tx:.0f},{ty:.0f}) dist={math.hypot(self.x - tx, self.y - ty):.1f}m"
            elif self.step == STEP_SCAN:
                idx = min(self._zigzag_idx, len(ZIGZAG_SCAN) - 1)
                wx, wy, tid = ZIGZAG_SCAN[idx]
                tx = self.home_x + wx
                ty = self.home_y + wy
                extra = f" zig{idx}/{len(ZIGZAG_SCAN)} →({tx:.0f},{ty:.0f}) t={tid} dist={math.hypot(self.x - tx, self.y - ty):.1f}m"
            dur = ""
            if self._arm_time and not self._disarm_time:
                d = (self.get_clock().now() - self._arm_time).nanoseconds * 1e-9
                dur = f" dur={d:.0f}s"
            self.get_logger().info(
                f"[{tag}] NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                f"hdg={hdg:.0f}deg {det}{extra}{dur}")
            self._print_ctr = 0

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
                self.get_logger().error("Preflight timeout")
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
        elif self.step == STEP_LAND:
            pass  # MAV_CMD_NAV_LAND puts PX4 in LAND mode (nav=18)
        elif elapsed > 30.0:
            self.get_logger().error("Mode switch timeout, aborting")
            self.timer.cancel()
            return
        elif self.status.arming_state != 2:
            if not self._arm_req or elapsed - self._last_arm_attempt > 1.0:
                self._arm_req = True
                self._last_arm_attempt = elapsed
                self._send_cmd(400, param1=1.0, param2=2989.0)
                self.get_logger().info(f"Requesting ARM (t={elapsed:.1f}s)")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return
        elif self.status.nav_state != 14:
            if not self._mode_req:
                self._mode_req = True
                self._send_cmd(5300, param1=0.0)
                self._send_cmd(176, param1=1.0, param2=6.0)
                self.get_logger().info(f"Requesting OFFBOARD (nav={self.status.nav_state})")
            self._pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return
        else:
            return

        # ================================================================
        # 状态机
        # ================================================================
        if self._task_phase == PHASE_SCOUT and self._scout_result is not None:
            if not hasattr(self, '_last_logged_result') or self._last_logged_result != self._scout_result:
                self.get_logger().info(f">>> {self._scout_result} <<<")
                self._last_logged_result = self._scout_result

        if self.step == STEP_TAKEOFF:
            self._run_takeoff(target_z)
        elif self.step == STEP_FORWARD_CENTER:
            self._run_forward_center(target_z)
        elif self.step == STEP_CENTER_SEARCH:
            self._run_center_search(target_z)
        elif self.step == STEP_TRACK:
            self._run_track(target_z)
        elif self.step == STEP_DESCEND:
            self._run_descend()
        elif self.step == STEP_HOVER:
            self._run_hover()
        elif self.step == STEP_DROP:
            self._run_drop()
        elif self.step == STEP_CLIMB_SWITCH:
            self._run_climb_switch(target_z)
        elif self.step == STEP_RESULT_DROP:
            self._run_result_drop(target_z)
        elif self.step == STEP_FORWARD_SCOUT:
            self._run_forward_scout(target_z)
        elif self.step == STEP_SCAN:
            self._run_scan(target_z)
        elif self.step == STEP_RESULT_SCOUT:
            self._run_result_scout(target_z)
        elif self.step == STEP_RETURN:
            self._run_return(target_z)
        elif self.step == STEP_HOVER_LAND:
            self._run_hover_land(target_z)
        elif self.step == STEP_LAND:
            self._run_land(target_z)

    # ===================== 各状态实现 =====================

    def _run_takeoff(self, target_z):
        if self._step_start_time is None:
            self._step_start_time = self.get_clock().now()
            self._tkoff_x = self.x
            self._tkoff_y = self.y
            self._tkoff_z_start = self.z
            # 先切到目标0 (detector 默认为 0)
            self._switch_bucket_target(0)
        tkoff_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if tkoff_elapsed < 3.0:
            self._pub_pos(self._tkoff_x, self._tkoff_y, self._tkoff_z_start, self.heading)
        else:
            ramp_t = tkoff_elapsed - 3.0
            z_ramp = self._tkoff_z_start - min(CRUISE_HEIGHT, ramp_t * 0.5)
            self._pub_pos(self._tkoff_x, self._tkoff_y, z_ramp, self.heading)
        if abs(self.z - target_z) < 0.8 and abs(self.z - self.home_z) > 1.5:
            tx = self.home_x + CENTER_X
            ty = self.home_y + CENTER_Y
            self.get_logger().info(f"Takeoff → FORWARD_CENTER to NED({tx:.1f},{ty:.1f})")
            self._enter_step(STEP_FORWARD_CENTER)

    def _run_forward_center(self, target_z):
        """飞往三桶几何中心 NED(0, 35)。纯东方向飞行，不依赖 heading。"""
        fwd_x = self.home_x + CENTER_X
        fwd_y = self.home_y + CENTER_Y

        dist = math.hypot(self.x - fwd_x, self.y - fwd_y)
        if dist < 10.0:
            speed = 0.5
            scale = speed / max(dist, 0.5)
            vx = (fwd_x - self.x) * scale
            vy = (fwd_y - self.y) * scale
            self._pub_vel(vx, vy, 0.0, self.heading)
            if dist < 0.3:
                self.get_logger().info("Arrived at center (dist<0.3m) → CENTER_SEARCH (waiting detector init)")
                self._enter_step(STEP_CENTER_SEARCH)
                return
        else:
            self._pub_pos(fwd_x, fwd_y, target_z, self.heading)

    def _run_center_search(self, target_z):
        """三桶几何中心悬停，等待 yolo_bucket_detector 初始化并开始发布 ROI。"""
        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

        # 悬停在中心
        cx = self.home_x + CENTER_X
        cy = self.home_y + CENTER_Y
        self._pub_pos(cx, cy, target_z, self.heading)

        if self._detect_fresh():
            self.get_logger().info(
                f"Detector ROI received after {elapsed:.1f}s → TRACK (target 0 = largest bucket)")
            self._ctrl.reset()
            self._stabilize_start = None
            self._enter_step(STEP_TRACK)
            return

        if elapsed > CENTER_TIMEOUT:
            self.get_logger().warn(
                f"CENTER_SEARCH timeout ({CENTER_TIMEOUT}s), no detection — "
                f"detector may have failed to initialize")
            # 容错: 切换到侦察阶段，跳过投放
            self.get_logger().warn("Skipping drop phase → FORWARD_SCOUT")
            self._task_phase = PHASE_SCOUT
            self._scout_index = 0
            self._zigzag_idx = 0
            self._scout_reported = set()
            self._ctrl = self.ctrl_scout
            self._ctrl.reset()
            self._switch_scout_target(0)
            self._enter_step(STEP_FORWARD_SCOUT)

    def _run_track(self, target_z):
        if not self._detect_fresh():
            self.get_logger().warn("Detection lost in TRACK → SEARCH")
            self._stabilize_start = None
            if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
                self._enter_step(STEP_CENTER_SEARCH)
            else:
                self._enter_step(STEP_SCAN)
            return

        roi = self._active_roi()
        v_fwd, v_lat, err_px, ex, ey = self._ctrl.compute(roi, 0.05)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        vz = (target_z - self.z) * TRACK_VZ_KP
        self._pub_vel(vx, vy, vz, self.heading)

        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if sim_t - self._last_pid_print > 1.0:
                self.get_logger().info(
                    f"[TRACK] px_err=({ex:.0f},{ey:.0f}) total={err_px:.0f}px "
                    f"pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"vel=({vx:.3f},{vy:.3f},{vz:.3f})")
                self._last_pid_print = sim_t

        if err_px < TRACK_ARRIVE_PX:
            if self._stabilize_start is None:
                self._stabilize_start = self.get_clock().now()
            elif (self.get_clock().now() - self._stabilize_start).nanoseconds > STABILIZE_TIME * 1e9:
                if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
                    self.get_logger().info(f"TRACK converged → DESCEND to {DROP_HEIGHT:.0f}m")
                    self._enter_step(STEP_DESCEND)
                else:
                    self._hover_target = (self.x, self.y, self.z)
                    self._scout_result = None
                    self._result_logged = False
                    self.get_logger().info(f"TRACK converged → HOVER for cylinder {self._scout_index}")
                    self._enter_step(STEP_HOVER)
        else:
            self._stabilize_start = None

    def _run_descend(self):
        drop_z = self.home_z - DROP_HEIGHT
        if not self._detect_fresh():
            self.get_logger().warn("Detection lost in DESCEND → CENTER_SEARCH")
            self._stabilize_start = None
            self._enter_step(STEP_CENTER_SEARCH)
            return

        roi = self._active_roi()
        v_fwd, v_lat, _, ex, ey = self._ctrl.compute(roi, 0.05)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        self._pub_vel(vx, vy, DESCEND_SPEED, self.heading)

        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if sim_t - self._last_pid_print > 1.0:
                self.get_logger().info(
                    f"[DESCEND] px_err=({ex:.0f},{ey:.0f}) "
                    f"pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"vel=({vx:.3f},{vy:.3f},{DESCEND_SPEED:.3f}) z→{drop_z:.1f}")
                self._last_pid_print = sim_t

        if self.z > drop_z - 0.2:
            self._stabilize_start = None
            self.get_logger().info(f"At drop altitude ({self.z:.1f}m) → HOVER")
            self._enter_step(STEP_HOVER)

    def _run_hover(self):
        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            drop_z = self.home_z - DROP_HEIGHT
            if not self._detect_fresh():
                self.get_logger().warn("Detection lost in HOVER → CENTER_SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_CENTER_SEARCH)
                return

            roi = self._active_roi()
            v_fwd, v_lat, err_px, ex, ey = self._ctrl.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = (drop_z - self.z) * HOVER_VZ_KP
            vz = max(-0.3, min(0.3, vz))
            self._pub_vel(vx, vy, vz, self.heading)

            sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if sim_t - self._last_pid_print > 1.0:
                self.get_logger().info(
                    f"[HOVER] px_err=({ex:.0f},{ey:.0f}) total={err_px:.0f}px "
                    f"pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"vel=({vx:.3f},{vy:.3f},{vz:.3f}) drop_z={drop_z:.2f}")
                self._last_pid_print = sim_t

            if err_px < HOVER_ARRIVE_PX:
                if self._stabilize_start is None:
                    self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now() - self._stabilize_start).nanoseconds > STABILIZE_TIME * 1e9:
                    self.get_logger().info(f"HOVER converged (err={err_px:.0f}px) → DROP")
                    self._enter_step(STEP_DROP)
            else:
                self._stabilize_start = None
        else:
            tx, ty, tz = self._hover_target
            self._pub_pos(tx, ty, tz, self.heading)

            if not hasattr(self, '_scout_req_sent') or not self._scout_req_sent:
                msg = Int32()
                msg.data = self._scout_index
                self.scout_reached_pub.publish(msg)
                self._scout_req_sent = True
                self._scout_result = None

            if self._scout_result is not None and not getattr(self, '_result_logged', False):
                self.get_logger().info(f">>> {self._scout_result} <<<")
                self._result_logged = True

            hover_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
            if hover_elapsed > HOVER_SCOUT_TIME:
                self._scout_req_sent = False
                self._advance_scout()

    def _run_drop(self):
        drop_z = self.home_z - DROP_HEIGHT
        if self._detect_fresh():
            roi = self._active_roi()
            v_fwd, v_lat, _, ex, ey = self._ctrl.compute(roi, 0.05)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        else:
            vx, vy, ex, ey = 0.0, 0.0, 0, 0
        vz = (drop_z - self.z) * HOVER_VZ_KP
        vz = max(-0.2, min(0.2, vz))
        self._pub_vel(vx, vy, vz, self.heading)

        if self._task_phase in (PHASE_BUCKET0, PHASE_BUCKET1):
            sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if sim_t - self._last_pid_print > 1.0:
                self.get_logger().info(
                    f"[DROP] px_err=({ex:.0f},{ey:.0f}) pos=({self.x:.2f},{self.y:.2f},{self.z:.2f})")
                self._last_pid_print = sim_t

        if not self._drop_future:
            if not self.drop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("/drop_payload not available")
                self._advance_after_drop()
                return
            bname = "B0(largest)" if self._task_phase == PHASE_BUCKET0 else "B1(second)"
            self.get_logger().info(f"DROPPING {bname} at NED({self.x:.2f},{self.y:.2f},{self.z:.2f})")
            self._drop_future = self.drop_client.call_async(Trigger.Request())
        elif self._drop_future.done():
            result = self._drop_future.result()
            ok = result is not None and result.success
            self.get_logger().info(f"Drop done: {'OK' if ok else 'FAIL'}")
            self._drop_future = None
            self._advance_after_drop()

    def _run_climb_switch(self, target_z):
        """投完目标0 后爬升回巡航高度 + 切换 detector 到目标1。"""
        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

        # 水平位置保持 + 爬升
        vz = -(CLIMB_SPEED)  # NED z- = climb
        self._pub_vel(0.0, 0.0, vz, self.heading)

        # 切换 detector 到目标1
        self._switch_bucket_target(1)

        at_cruise = abs(self.z - target_z) < 1.0
        if at_cruise and self._detect_fresh():
            self.get_logger().info(f"Climb done + target 1 acquired → CENTER_SEARCH (re-center then track)")
            self._ctrl.reset()
            self._stabilize_start = None
            self._enter_step(STEP_CENTER_SEARCH)
            return

        if elapsed > CLIMB_TIMEOUT:
            self.get_logger().warn(f"CLIMB_SWITCH timeout — climbing to cruise + retrying")
            self._enter_step(STEP_CENTER_SEARCH)

    def _advance_after_drop(self):
        if self._task_phase == PHASE_BUCKET0:
            self._task_phase = PHASE_BUCKET1
            self._ctrl.reset()
            self.get_logger().info("Bucket 0 dropped → CLIMB_SWITCH to cruise height + switch to target 1")
            self._enter_step(STEP_CLIMB_SWITCH)
        else:
            self._enter_step(STEP_RESULT_DROP)

    def _run_result_drop(self, target_z):
        self._pub_pos(self.x, self.y, target_z, self.heading)
        result_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if result_elapsed > 1.0:
            self.get_logger().info("双桶投放完成 → FORWARD_SCOUT to scout zone")
            self._task_phase = PHASE_SCOUT
            self._scout_index = 0
            self._zigzag_idx = 0
            self._scout_reported = set()
            self._ctrl = self.ctrl_scout
            self._ctrl.reset()
            self._switch_scout_target(0)
            self._enter_step(STEP_FORWARD_SCOUT)

    def _run_forward_scout(self, target_z):
        scout_x = self.home_x + SCOUT_WP_X
        scout_y = self.home_y + SCOUT_WP_Y

        dist = math.hypot(self.x - scout_x, self.y - scout_y)
        if dist < 10.0:
            speed = 0.5
            scale = speed / max(dist, 0.5)
            vx = (scout_x - self.x) * scale
            vy = (scout_y - self.y) * scale
            self._pub_vel(vx, vy, 0.0, self.heading)
            if dist < POS_ARRIVE_DIST:
                self.get_logger().info("FORWARD_SCOUT done → SCAN zigzag")
                self._zigzag_idx = 0
                self._scout_reported = set()
                self._scout_index = ZIGZAG_SCAN[0][2]
                self._switch_scout_target(self._scout_index)
                self._enter_step(STEP_SCAN)
                return
        else:
            self._pub_pos(scout_x, scout_y, target_z, self.heading)

    def _run_scan(self, target_z):
        if self._zigzag_idx >= len(ZIGZAG_SCAN):
            self.get_logger().info("Zigzag scan complete → RESULT_SCOUT")
            self._enter_step(STEP_RESULT_SCOUT)
            return

        wx, wy, tid = ZIGZAG_SCAN[self._zigzag_idx]
        wp_x = self.home_x + wx
        wp_y = self.home_y + wy
        wp_z = self.home_z - CRUISE_HEIGHT

        dist = math.hypot(self.x - wp_x, self.y - wp_y)
        if dist > 0.2:
            scale = SCAN_SPEED / max(dist, 0.1)
            vx = (wp_x - self.x) * scale
            vy = (wp_y - self.y) * scale
            vz = (wp_z - self.z) * 0.5
            self._pub_vel(vx, vy, vz, self.heading)
        else:
            self._pub_vel(0.0, 0.0, 0.0, self.heading)

        if dist < SCAN_ARRIVE_DIST:
            if tid >= 0 and tid not in self._scout_reported:
                self._scout_index = tid
                if self._detect_fresh():
                    self._scout_reported.add(tid)
                    msg = Int32()
                    msg.data = tid
                    self.scout_reached_pub.publish(msg)
                    self.get_logger().info(f"Scout reported cylinder {tid} at NED({self.x:.1f},{self.y:.1f})")
                else:
                    self.get_logger().info(f"No detection for cylinder {tid} at NED({self.x:.1f},{self.y:.1f})")

            self._zigzag_idx += 1
            if self._zigzag_idx < len(ZIGZAG_SCAN):
                next_tid = ZIGZAG_SCAN[self._zigzag_idx][2]
                if next_tid >= 0:
                    self._scout_index = next_tid
                    self._switch_scout_target(next_tid)
                    self.get_logger().info(f"→ zigzag wp {self._zigzag_idx}/{len(ZIGZAG_SCAN)} target={next_tid}")

    def _run_result_scout(self, target_z):
        self._pub_pos(self.x, self.y, target_z, self.heading)
        result_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if result_elapsed > 1.0:
            self.get_logger().info("侦察任务完成 → RETURN")
            self._enter_step(STEP_RETURN)

    def _advance_scout(self):
        self._scout_index += 1
        if self._scout_index < 5:
            self._switch_scout_target(self._scout_index)
            self._stabilize_start = None
            self._enter_step(STEP_SCAN)
        else:
            self._enter_step(STEP_RESULT_SCOUT)

    def _run_return(self, target_z):
        rx = self.home_x + LAND_WP_X
        ry = self.home_y + LAND_WP_Y
        rz = self.home_z - CRUISE_HEIGHT
        self._pub_pos(rx, ry, rz, self.heading)
        if self._dist_to(rx, ry, rz) < POS_ARRIVE_DIST:
            self.get_logger().info("Returned to landing zone → HOVER_LAND (stabilize then descend)")
            self._stabilize_start = None
            self._enter_step(STEP_HOVER_LAND)

    def _run_hover_land(self, target_z):
        """降落点上空悬停，位置稳定后才发送降落指令"""
        lx = self.home_x + LAND_WP_X
        ly = self.home_y + LAND_WP_Y
        self._pub_pos(lx, ly, target_z, self.heading)

        drift = math.hypot(self.x - lx, self.y - ly)

        if drift < HOVER_LAND_DRIFT_THRESH:
            if self._stabilize_start is None:
                self._stabilize_start = self.get_clock().now()
            elif (self.get_clock().now() - self._stabilize_start).nanoseconds > HOVER_LAND_STABLE_TIME * 1e9:
                self.get_logger().info(f"Hover stable at landing zone (drift={drift:.2f}m) -> LAND (cmd=21)")
                self._stabilize_start = None
                self._enter_step(STEP_LAND)
                return
        else:
            self._stabilize_start = None

    def _run_land(self, target_z):
        """发送 MAV_CMD_NAV_LAND，PX4 内部自主降落"""
        lx = self.home_x + LAND_WP_X
        ly = self.home_y + LAND_WP_Y

        if not self._land_cmd_sent:
            self._send_cmd(21)
            self._land_cmd_sent = True
            self.get_logger().info("MAV_CMD_NAV_LAND sent -> PX4 auto-land")

        self._pub_pos(lx, ly, self.z, self.heading)

        if self.landed:
            if self._land_dwell_start is None:
                self._land_dwell_start = self.get_clock().now()
            elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                self._send_cmd(400, param1=0.0)
                self.timer.cancel()
                self._print_final_summary()
                self.get_logger().info("Full mission v2 complete — disarmed (landed)")
        elif self.z > self.home_z - 0.15:  # 单侧: 不高于起飞点 0.15m 即触地 (允许 PX4 auto-land 推入地面以下)
            if self._land_dwell_start is None:
                self._land_dwell_start = self.get_clock().now()
            elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                self._send_cmd(400, param1=0.0)
                self.timer.cancel()
                self._print_final_summary()
                self.get_logger().info("Full mission v2 complete — disarmed (altitude)")
        else:
            self._land_dwell_start = None

    def _print_final_summary(self):
        if self._final_summary_printed:
            return
        self._final_summary_printed = True
        if self._arm_time and self._disarm_time:
            dur = (self._disarm_time - self._arm_time).nanoseconds * 1e-9
        elif self._arm_time:
            dur = (self.get_clock().now() - self._arm_time).nanoseconds * 1e-9
        else:
            dur = 0
        self.get_logger().info(
            f"====== FULL MISSION V2 COMPLETE ======\n"
            f"  Duration: {dur:.1f}s\n"
            f"  Final NED: ({self.x:.2f},{self.y:.2f},{self.z:.2f})\n"
            f"  Landing zone NED: ({self.home_x + LAND_WP_X:.2f},{self.home_y + LAND_WP_Y:.2f})\n"
            f"  Landing dist: {math.hypot(self.x - self.home_x - LAND_WP_X, self.y - self.home_y - LAND_WP_Y):.3f}m\n"
            f"====================================")


def main(args=None):
    rclpy.init(args=args)
    node = FullMissionV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
