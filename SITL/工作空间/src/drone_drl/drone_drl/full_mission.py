#!/usr/bin/env python3
"""
全任务节点 — 投放区(30m)双桶 + 侦察区(55m)五筒 串联执行。
竞赛场地 competition_field.world 适配版。
状态机:
  TAKEOFF → FORWARD(飞投放区30m) → SEARCH_B1 → TRACK → DESCEND → HOVER → DROP(桶1)
  → MOVE_B2 → SEARCH_B2 → TRACK → DESCEND → HOVER → DROP(桶2)
  → RESULT_DROP → FORWARD_SCOUT(飞55m) → (SEARCH_S → TRACK_S → HOVER_S) × 5
  → RESULT_SCOUT → RETURN → LAND
偏航全程锁定。投放阶段订阅 /bucket/detection，侦察阶段订阅 /scout/detection。
计时从 unlock/arm 瞬间开始到 disarm 停止。
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
(STEP_TAKEOFF, STEP_FORWARD, STEP_SEARCH, STEP_TRACK, STEP_DESCEND,
 STEP_HOVER, STEP_DROP, STEP_MOVE_B2, STEP_RESULT_DROP,
 STEP_FORWARD_SCOUT, STEP_SCAN, STEP_RESULT_SCOUT, STEP_RETURN, STEP_LAND) = range(14)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD: "FORWARD",
    STEP_SEARCH: "SEARCH", STEP_TRACK: "TRACK", STEP_DESCEND: "DESCEND",
    STEP_HOVER: "HOVER", STEP_DROP: "DROP", STEP_MOVE_B2: "MOVE_B2",
    STEP_RESULT_DROP: "RESULT_DROP", STEP_FORWARD_SCOUT: "FORWARD_SCOUT",
    STEP_SCAN: "SCAN", STEP_RESULT_SCOUT: "RESULT_SCOUT",
    STEP_RETURN: "RETURN", STEP_LAND: "LAND",
}

# ---- 任务阶段 ----
(PHASE_BUCKET1, PHASE_BUCKET2, PHASE_SCOUT) = range(3)

# ---- 竞赛场地硬编码 NED 航点偏移 (PX4 生成点 1.01,0.98,0.83 不变) ----
# 场地绕 Gazebo Z 轴顺时针旋转 90°，沿 X 轴 (东) 宽 8m，平移 +3.51 使起降圈对齐 spawn
CRUISE_HEIGHT   = 3.5     # 统一巡航高度 (m)
DROP_HEIGHT     = 1.0     # 投放高度 (m)

# 桶1 Gazebo(35.51,2,0.15) → NED x=2-0.98=1.02, y=35.51-1.01=34.50
B1_WP_X = 1.0
B1_WP_Y = 34.5
# 桶2 Gazebo(37.01,1,0.15) → NED x=1-0.98=0.02, y=37.01-1.01=36.00
B2_WP_X = 0.0
B2_WP_Y = 36.0
# 桶3 Gazebo(35.51,0,0.15) → NED x=0-0.98=-0.98, y=35.51-1.01=34.50 — 不投弹，仅存在
# 侦察区入口 Gazebo(58.51,1) → NED x=1-0.98=0.02, y=58.51-1.01=57.50
SCOUT_WP_X = 0.0
SCOUT_WP_Y = 57.5

# ---- 侦察蛇形扫描航点 (NED 偏移, target_id) ----
# 跑道宽 8m (NED x≈-3~3)，筒分布在 NED x≈-1.5~1.5, y≈58~63
ZIGZAG_SCAN = [
    (2.5, 58.5, 0),    # 右 → 筒0 (NED 1.5,58.5)
    (-2.5, 59.5, 1),   # 左 → 筒1 (NED -1.5,59.5)
    (2.5, 60.5, 2),    # 右 → 筒2 (NED 1.0,60.5)
    (-2.5, 61.5, 3),   # 左 → 筒3 (NED -1.0,61.5)
    (2.0, 62.5, 4),    # 右 → 筒4 (NED 0.0,62.5)
    (0.0, 63.5, -1),   # 中心 结束
]
SCAN_ARRIVE_DIST = 2.0   # 到达航点距离阈值 (m)
SCAN_SPEED = 2.0         # 扫描飞行速度 (m/s)

# 着陆区 Gazebo(1.01,1) → NED x=1-0.98=0.02, y=1.01-1.01=0.00
LAND_WP_X = 0.0
LAND_WP_Y = 0.0

CRUISE_SPEED    = 1.0
SEARCH_SPEED    = 0.5
STEP_TIMEOUT    = 120.0

# PID 桶（近程，drop_opt 优化值）
KP_B, KI_B, KD_B = 0.006, 0.00005, 0.002
MAX_VEL_B = 1.5
INT_LIMIT_B = 80.0

# PID 筒（远程）
KP_S, KI_S, KD_S = 0.0015, 0.00003, 0.0005
MAX_VEL_S = 0.6
INT_LIMIT_S = 40.0

DETECT_TIMEOUT   = 3.0
TRACK_ARRIVE_PX  = 40.0    # TRACK 收敛像素阈值
HOVER_ARRIVE_PX  = 25.0    # HOVER 收敛像素阈值
STABILIZE_TIME   = 5.0     # 稳定对准时间 (s)
DESCEND_SPEED    = 0.25    # 下降速度 (m/s, NED z+)
TRACK_VZ_KP      = 0.5     # 巡航高度锁定 P 增益
HOVER_VZ_KP      = 0.8     # 投放高度锁定 P 增益
HOVER_SCOUT_TIME = 2.0     # 侦察悬停时间
IMG_W = 640
IMG_H = 480

POS_ARRIVE_DIST  = 0.5
LAND_DWELL_TIME  = 3.0


class VisualServoController:
    """PID 机体速度控制器，根据 ROI 像素偏差计算机体速度"""

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


class FullMission(Node):
    def __init__(self):
        super().__init__('full_mission')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        # Pub
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.scout_reached_pub = self.create_publisher(Int32, '/scout/point_reached', 10)
        self.bucket_target_pub = self.create_publisher(Int32, '/bucket_target_id', 10)
        self.scout_target_pub = self.create_publisher(Int32, '/scout/target_id', 10)

        # Sub — PX4
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._cb_local, qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._cb_land, qos)

        # Sub — 检测 (两个话题，按阶段切换使用)
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
        self._task_phase = PHASE_BUCKET1
        self._scout_index = 0
        self._step_start_time = None
        self._zigzag_idx = 0
        self._scout_reported = set()

        # PID 控制器（双参数集）
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
            "Full mission ready: competition_field — drop zone ~30m (dual-bucket) "
            "→ scout zone ~55m (5-cyl), yaw-locked, PID visual-servo")

    # ===================== 回调 =====================

    def _cb_status(self, m):
        if self.status is None:
            self.get_logger().info(
                f"First status: arm={m.arming_state} nav={m.nav_state} "
                f"failsafe={m.failsafe} preflight_ok={m.pre_flight_checks_pass}")
        elif m.failsafe != self.status.failsafe:
            self.get_logger().warn(
                f"FAILSAFE changed to {m.failsafe} nav={m.nav_state} "
                f"arm={m.arming_state}")
        elif m.nav_state != self.status.nav_state:
            self.get_logger().info(
                f"nav_state: {self.status.nav_state} → {m.nav_state}")
        # 计时
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

    def _active_roi(self):
        """返回当前阶段的有效 ROI"""
        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
            return self._bucket_roi
        return self._scout_roi

    def _active_last_det(self):
        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
            return self._last_bucket_det
        return self._last_scout_det

    def _detect_fresh(self):
        roi = self._active_roi()
        if roi is None:
            return False
        return (self.get_clock().now() - self._active_last_det()).nanoseconds * 1e-9 < DETECT_TIMEOUT

    def _switch_bucket_target(self, bucket_idx):
        """bucket_idx: 0=桶1, 1=桶2, 2=桶3"""
        msg = Int32()
        msg.data = bucket_idx
        self.bucket_target_pub.publish(msg)
        self.get_logger().info(f"Bucket target → {bucket_idx} (bucket {bucket_idx + 1})")

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
                ex, ey = cx - IMG_W/2.0, cy - IMG_H/2.0
                phase = {PHASE_BUCKET1: "B1", PHASE_BUCKET2: "B2", PHASE_SCOUT: f"S{self._scout_index}"}
                extra = f" {phase.get(self._task_phase,'?')} px_err=({ex:.0f},{ey:.0f})"
            elif self.step == STEP_FORWARD:
                tx = self.home_x + B1_WP_X
                ty = self.home_y + B1_WP_Y
                extra = f" →B1({tx:.1f},{ty:.1f}) dist={math.hypot(self.x-tx,self.y-ty):.1f}m"
            elif self.step == STEP_MOVE_B2:
                tx = self.home_x + B2_WP_X
                ty = self.home_y + B2_WP_Y
                extra = f" →B2({tx:.1f},{ty:.1f}) dist={math.hypot(self.x-tx,self.y-ty):.1f}m"
            elif self.step == STEP_RETURN:
                extra = f" →home dist={math.hypot(self.x-self.home_x,self.y-self.home_y):.1f}m"
            elif self.step == STEP_FORWARD_SCOUT:
                tx = self.home_x + SCOUT_WP_X
                ty = self.home_y + SCOUT_WP_Y
                extra = f" →scout({tx:.0f},{ty:.0f}) dist={math.hypot(self.x-tx,self.y-ty):.1f}m"
            elif self.step == STEP_SCAN:
                idx = min(self._zigzag_idx, len(ZIGZAG_SCAN) - 1)
                wx, wy, tid = ZIGZAG_SCAN[idx]
                tx = self.home_x + wx
                ty = self.home_y + wy
                extra = f" zig{idx}/{len(ZIGZAG_SCAN)} →({tx:.0f},{ty:.0f}) t={tid} dist={math.hypot(self.x-tx,self.y-ty):.1f}m"
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
            pass  # ready
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
        # 侦察阶段异步结果日志
        if self._task_phase == PHASE_SCOUT and self._scout_result is not None:
            if not hasattr(self, '_last_logged_result') or self._last_logged_result != self._scout_result:
                self.get_logger().info(f">>> {self._scout_result} <<<")
                self._last_logged_result = self._scout_result

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

        elif self.step == STEP_MOVE_B2:
            self._run_move_b2(target_z)

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

        elif self.step == STEP_LAND:
            self._run_land()

    # ===================== 各状态实现 =====================

    def _run_takeoff(self, target_z):
        if self._step_start_time is None:
            self._step_start_time = self.get_clock().now()
            self._tkoff_x = self.x
            self._tkoff_y = self.y
            self._tkoff_z_start = self.z
            self._switch_bucket_target(0)
        tkoff_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if tkoff_elapsed < 3.0:
            self._pub_pos(self._tkoff_x, self._tkoff_y, self._tkoff_z_start, self.heading)
        else:
            ramp_t = tkoff_elapsed - 3.0
            z_ramp = self._tkoff_z_start - min(CRUISE_HEIGHT, ramp_t * 0.5)
            self._pub_pos(self._tkoff_x, self._tkoff_y, z_ramp, self.heading)
        if abs(self.z - target_z) < 0.8 and abs(self.z - self.home_z) > 1.5:
            tx = self.home_x + B1_WP_X
            ty = self.home_y + B1_WP_Y
            self.get_logger().info(f"Takeoff → FORWARD to bucket1 NED({tx:.1f},{ty:.1f})")
            self._enter_step(STEP_FORWARD)

    def _run_forward(self, target_z):
        fwd_target_x = self.home_x + B1_WP_X
        fwd_target_y = self.home_y + B1_WP_Y

        if self._detect_fresh():
            self.get_logger().info("Detected during FORWARD → SEARCH")
            self._ctrl.reset()
            self._enter_step(STEP_SEARCH)
            return

        dist = math.hypot(self.x - fwd_target_x, self.y - fwd_target_y)
        if dist < 10.0:
            speed = 0.5
            scale = speed / max(dist, 0.5)
            vx = (fwd_target_x - self.x) * scale
            vy = (fwd_target_y - self.y) * scale
            self._pub_vel(vx, vy, 0.0, self.heading)
            if dist < POS_ARRIVE_DIST:
                self._enter_step(STEP_SEARCH)
                self.get_logger().info(f"FORWARD done at bucket1 area → SEARCH")
                return
        else:
            self._pub_pos(fwd_target_x, fwd_target_y, target_z, self.heading)

    def _run_search(self, target_z):
        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
            if elapsed > STEP_TIMEOUT:
                self.get_logger().warn("Bucket search timeout, skipping to next phase")
                self._advance_after_search_fail()
                return

            if self._detect_fresh():
                self.get_logger().info("Detected bucket → TRACK")
                self._ctrl.reset()
                self._stabilize_start = None
                self._enter_step(STEP_TRACK)
                return

            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            self._pub_vel(SEARCH_SPEED * cos_h, SEARCH_SPEED * sin_h, 0.0, self.heading)

        else:  # PHASE_SCOUT
            if elapsed > STEP_TIMEOUT:
                self.get_logger().warn(f"Scout search timeout for cylinder {self._scout_index}, skipping")
                self._advance_scout()
                return

            if self._detect_fresh():
                self.get_logger().info(f"Detected scout cylinder {self._scout_index} → TRACK")
                self._ctrl.reset()
                self._stabilize_start = None
                self._enter_step(STEP_TRACK)
                return

            if self.y < self.home_y + SCOUT_MAX_Y:
                self._pub_vel(0.0, SEARCH_SPEED, 0.0, self.heading)
            else:
                self._pub_vel(0.0, 0.0, 0.0, self.heading)

    def _run_track(self, target_z):
        if not self._detect_fresh():
            self.get_logger().warn("Detection lost in TRACK → SEARCH")
            self._stabilize_start = None
            self._enter_step(STEP_SEARCH)
            return

        roi = self._active_roi()
        v_fwd, v_lat, err_px, ex, ey = self._ctrl.compute(roi, 0.05)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        vz = (target_z - self.z) * TRACK_VZ_KP
        self._pub_vel(vx, vy, vz, self.heading)

        # 投放阶段 1Hz PID 监控日志
        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
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
                if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
                    self.get_logger().info(f"TRACK converged → DESCEND to {DROP_HEIGHT:.0f}m")
                    self._enter_step(STEP_DESCEND)
                else:
                    self._hover_target = (self.x, self.y, self.z)
                    self._scout_result = None
                    self.get_logger().info(f"TRACK converged → HOVER for cylinder {self._scout_index}")
                    self._result_logged = False
                    self._enter_step(STEP_HOVER)
        else:
            self._stabilize_start = None

    def _run_descend(self):
        drop_z = self.home_z - DROP_HEIGHT
        if not self._detect_fresh():
            self.get_logger().warn("Detection lost in DESCEND → SEARCH")
            self._stabilize_start = None
            self._enter_step(STEP_SEARCH)
            return

        roi = self._active_roi()
        v_fwd, v_lat, _, ex, ey = self._ctrl.compute(roi, 0.05)
        vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
        vz = DESCEND_SPEED
        self._pub_vel(vx, vy, vz, self.heading)

        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
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
        """桶: 低空精密收敛; 筒: 悬停侦察识别"""
        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
            drop_z = self.home_z - DROP_HEIGHT
            if not self._detect_fresh():
                self.get_logger().warn("Detection lost in HOVER → SEARCH")
                self._stabilize_start = None
                self._enter_step(STEP_SEARCH)
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

        else:  # PHASE_SCOUT
            tx, ty, tz = self._hover_target
            self._pub_pos(tx, ty, tz, self.heading)

            # 首次进入时发送识别请求
            if not hasattr(self, '_scout_req_sent') or not self._scout_req_sent:
                msg = Int32()
                msg.data = self._scout_index
                self.scout_reached_pub.publish(msg)
                self._scout_req_sent = True
                self._scout_result = None

            # 异步等待识别结果（由 _cb_scout_result 回调设置）
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

        if self._task_phase in (PHASE_BUCKET1, PHASE_BUCKET2):
            sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if sim_t - self._last_pid_print > 1.0:
                self.get_logger().info(
                    f"[DROP] px_err=({ex:.0f},{ey:.0f}) "
                    f"pos=({self.x:.2f},{self.y:.2f},{self.z:.2f})")
                self._last_pid_print = sim_t

        if not self._drop_future:
            if not self.drop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("/drop_payload not available")
                self._advance_after_drop()
                return
            bname = "B1" if self._task_phase == PHASE_BUCKET1 else "B2"
            self.get_logger().info(f"DROPPING {bname} at NED({self.x:.2f},{self.y:.2f},{self.z:.2f})")
            self._drop_future = self.drop_client.call_async(Trigger.Request())

        elif self._drop_future.done():
            result = self._drop_future.result()
            ok = result is not None and result.success
            self.get_logger().info(f"Drop done: {'OK' if ok else 'FAIL'}")
            self._drop_future = None
            self._advance_after_drop()

    def _advance_after_drop(self):
        if self._task_phase == PHASE_BUCKET1:
            self._task_phase = PHASE_BUCKET2
            self._switch_bucket_target(1)
            self._ctrl = self.ctrl_bucket
            self._ctrl.reset()
            self._enter_step(STEP_MOVE_B2)
        else:
            self._enter_step(STEP_RESULT_DROP)

    def _advance_after_search_fail(self):
        if self._task_phase == PHASE_BUCKET1:
            self._task_phase = PHASE_BUCKET2
            self._switch_bucket_target(1)
            self._ctrl.reset()
            self._enter_step(STEP_MOVE_B2)
        elif self._task_phase == PHASE_BUCKET2:
            self._enter_step(STEP_RESULT_DROP)
        else:
            self._advance_scout()

    def _advance_scout(self):
        self._scout_index += 1
        if self._scout_index < 5:
            self._switch_scout_target(self._scout_index)
            self._stabilize_start = None
            self._enter_step(STEP_SEARCH)
        else:
            self._enter_step(STEP_RESULT_SCOUT)

    def _run_move_b2(self, target_z):
        b2_target_x = self.home_x + B2_WP_X
        b2_target_y = self.home_y + B2_WP_Y

        dist = math.hypot(self.x - b2_target_x, self.y - b2_target_y)
        if dist < POS_ARRIVE_DIST:
            self.get_logger().info(f"MOVE_B2 done → SEARCH for bucket 2")
            self._enter_step(STEP_SEARCH)
            return

        self._pub_pos(b2_target_x, b2_target_y, target_z, self.heading)

    def _run_result_drop(self, target_z):
        self._pub_pos(self.x, self.y, target_z, self.heading)
        result_elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        if result_elapsed > 1.0:
            self.get_logger().info("双桶投放完成 → FORWARD_SCOUT to 55m")
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
        """蛇形扫描侦察区 — 位置控制飞航点，途径筒时记录识别结果"""
        if self._zigzag_idx >= len(ZIGZAG_SCAN):
            self.get_logger().info("Zigzag scan complete → RESULT_SCOUT")
            self._enter_step(STEP_RESULT_SCOUT)
            return

        wx, wy, tid = ZIGZAG_SCAN[self._zigzag_idx]
        wp_x = self.home_x + wx
        wp_y = self.home_y + wy
        wp_z = self.home_z - CRUISE_HEIGHT

        # 速度控制飞往航点
        dist = math.hypot(self.x - wp_x, self.y - wp_y)
        if dist > 0.2:
            scale = SCAN_SPEED / max(dist, 0.1)
            vx = (wp_x - self.x) * scale
            vy = (wp_y - self.y) * scale
            vz = (wp_z - self.z) * 0.5
            self._pub_vel(vx, vy, vz, self.heading)
        else:
            self._pub_vel(0.0, 0.0, 0.0, self.heading)

        # 到达航点 → 记录检测结果并切换下一个
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

    def _run_return(self, target_z):
        rx = self.home_x + LAND_WP_X
        ry = self.home_y + LAND_WP_Y
        rz = self.home_z - CRUISE_HEIGHT
        self._pub_pos(rx, ry, rz, self.heading)
        if self._dist_to(rx, ry, rz) < POS_ARRIVE_DIST:
            self.get_logger().info("Returned to landing zone → LAND")
            self._enter_step(STEP_LAND)

    def _run_land(self):
        lx = self.home_x + LAND_WP_X
        ly = self.home_y + LAND_WP_Y
        self._pub_pos(lx, ly, self.home_z, self.heading)
        if self.landed:
            if self._land_dwell_start is None:
                self._land_dwell_start = self.get_clock().now()
            elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                self._send_cmd(400, param1=0.0)
                self.timer.cancel()
                self._print_final_summary()
                self.get_logger().info("Full mission complete — disarmed (landed)")
        elif abs(self.z - self.home_z) < 0.15:
            if self._land_dwell_start is None:
                self._land_dwell_start = self.get_clock().now()
            elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                self._send_cmd(400, param1=0.0)
                self.timer.cancel()
                self._print_final_summary()
                self.get_logger().info("Full mission complete — disarmed (altitude)")
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
            f"====== FULL MISSION COMPLETE ======\n"
            f"  Duration: {dur:.1f}s\n"
            f"  Final NED: ({self.x:.2f},{self.y:.2f},{self.z:.2f})\n"
            f"  Landing zone NED: ({self.home_x + LAND_WP_X:.2f},{self.home_y + LAND_WP_Y:.2f})\n"
            f"  Landing dist: {math.hypot(self.x - self.home_x - LAND_WP_X, self.y - self.home_y - LAND_WP_Y):.3f}m\n"
            f"====================================")


def main():
    rclpy.init()
    node = FullMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
