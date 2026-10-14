#!/usr/bin/env python3
"""
投放精度优化任务 — 单桶最简状态机 (v2 稳定锁航向 + 盲飞到位)。
状态机: TAKEOFF → STABILIZE → FORWARD → HOVER_SEARCH → TRACK → DESCEND → DROP → CLIMB → RETURN → LAND

v2 改动:
- 起飞后竖直爬升到巡航高度, 稳定悬停 3s 后再锁定 heading0 和 home
- FORWARD 阶段盲飞正前方 10m, 全程忽略视觉检测, 到点才切 HOVER_SEARCH
- HOVER_SEARCH 在目标点悬停才开始接收检测, 10s 无有效目标 → 返航
- 盲飞阶段 yaw=heading0 保方向, 视觉伺服阶段 yaw=live heading 不干扰 PID

基于实机飞行验证成功的代码 (KP=0.0015, KD=0.001, MAX=0.3, 姿态补偿)。
SITL 适配: 话题后缀 _v4, xy_valid 检查, VehicleControlMode+VehicleStatus 双源 ARM/OFFBOARD。
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected,
    ManualControlSetpoint, VehicleControlMode, VehicleAttitude,
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from drone_drl.common import (
    QOS_BE, status_topic, local_pos_topic, control_mode_topic,
    send_arm,
)

# ============================================================
# 可调参数 — 实机验证版
# ============================================================
# 地面 NED z 坐标常量 — 已废弃, 仅保留作为 SITL 参考值。
# 所有高度计算改用 self.home_z (起飞时记录的 NED z), 确保 SITL/实机均正确。
# SITL: home_z ≈ 0.83 (Gazebo 生成点), 实机: home_z = PX4 EKF 原点的地面 z。
GROUND_NED_Z = 0.83  # deprecated, use self.home_z

CRUISE_HEIGHT = 3.0
DROP_HEIGHT   = 1.0
FORWARD_DIST  = 9.5

KP_H = 0.0015          # 纯 PD, 极低增益保安全
KI_H = 0.0
KD_H = 0.001
MAX_HORIZ_VEL = 0.3
INTEGRAL_LIMIT = 0.0
DEAD_ZONE_PX = 5.0

TRACK_ARRIVE_PX  = 40.0
HOVER_ARRIVE_PX  = 25.0
STABILIZE_TIME   = 5.0
TRACK_MAX_TIME   = 60.0
HOVER_MAX_TIME   = 30.0

CRUISE_SPEED    = 2.0
FORWARD_UNDERSHOOT = 0.5  # 目标瞄近 0.5m, MPC overshoot 后到桶上方
FORWARD_RAMP_DIST = 3.0
FORWARD_MIN_SPEED = 0.5
SEARCH_SPEED     = 0.3
DESCEND_SPEED    = 0.25
HOVER_VZ_KP      = 0.8
TRACK_VZ_KP      = 0.5
CLIMB_VZ         = -0.8

DETECT_TIMEOUT = 3.0
STEP_TIMEOUT   = 120.0

TRACK_ENTRY_DIST_MAX     = 3.0
TRACK_ENTRY_CONFIRM_TIME = 1.5
TRACK_ENTRY_ROI_DRIFT    = 120.0
TRACK_ENTRY_GEO_ERR_MAX  = 1.5
STABILIZE_LOST_FRAMES_MAX = 10

POS_ARRIVE_DIST  = 0.5
LAND_DWELL_TIME  = 3.0
PAYLOAD_OFFSET_BODY_Y = 0.0
DROP_AFTER_DELAY = 0.0

AUX5_LOCK  =  0.0
AUX5_DROP  =  0.83333
AUX5_DROP_HOLD = 1.0
MANUAL_TIMER_PERIOD = 0.005

IMG_W, IMG_H = 640, 360
CX, CY = IMG_W / 2.0, IMG_H / 2.0
HFOV = math.radians(85)
VFOV = math.radians(54)
FX = IMG_W / (2.0 * math.tan(HFOV / 2.0))
FY = IMG_H / (2.0 * math.tan(VFOV / 2.0))

# v2 新增参数
STABILIZE_HOVER_TIME = 3.0    # 起飞后稳定悬停时间, 之后锁定 heading0
HOVER_SEARCH_TIMEOUT = 10.0
HOVER_SEARCH_SETTLE_TIME = 1.5  # 等 MPC 初始刹车, 剩余由门控过滤   # 目标点搜索超时, 无有效目标则返航
TAKEOFF_CLIMB_RATE   = 0.8    # 竖直爬升速率 (m/s)

# ============================================================
# 状态
# ============================================================
(STEP_TAKEOFF, STEP_STABILIZE, STEP_FORWARD, STEP_HOVER_SEARCH,
 STEP_TRACK, STEP_DESCEND, STEP_HOVER, STEP_DROP,
 STEP_CLIMB, STEP_RETURN, STEP_LAND) = range(11)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_STABILIZE: "STABILIZE",
    STEP_FORWARD: "FORWARD", STEP_HOVER_SEARCH: "HOVER_SEARCH",
    STEP_TRACK: "TRACK", STEP_DESCEND: "DESCEND",
    STEP_HOVER: "HOVER", STEP_DROP: "DROP", STEP_CLIMB: "CLIMB",
    STEP_RETURN: "RETURN", STEP_LAND: "LAND",
}


class PIDController:
    def __init__(self, kp, ki, kd, max_vel, int_limit, dead_zone_px=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_vel, self.int_limit = max_vel, int_limit
        self.dead_zone_px = dead_zone_px
        self.prev_ex = self.prev_ey = 0.0
        self.int_ex = self.int_ey = 0.0

    def reset(self):
        self.prev_ex = self.prev_ey = 0.0
        self.int_ex = self.int_ey = 0.0

    def compute(self, roi, dt, pixel_offset_x=0.0, pitch=0.0, roll=0.0, height_scale=1.0):
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        ex = cx - CX - pixel_offset_x
        ey = CY - cy  # 下视摄像头: TOP=前

        # 姿态补偿 (实机验证: YOLOv8 真摄像头需要)
        ex -= FX * math.tan(roll)
        ey -= FY * math.tan(pitch)

        if abs(ex) < self.dead_zone_px: ex = 0.0
        if abs(ey) < self.dead_zone_px: ey = 0.0

        dt = max(dt, 0.001)
        self.int_ex += ex * dt
        self.int_ey += ey * dt
        self.int_ex = max(-self.int_limit, min(self.int_limit, self.int_ex))
        self.int_ey = max(-self.int_limit, min(self.int_limit, self.int_ey))

        dex = (ex - self.prev_ex) / dt
        dey = (ey - self.prev_ey) / dt

        # KP/KD 按高度缩放: 低空像素/物理比变大, 缩小有效增益防过调
        kp = self.kp * height_scale
        kd = self.kd * height_scale
        v_lat = kp * ex + self.ki * self.int_ex + kd * dex
        v_fwd = kp * ey + self.ki * self.int_ey + kd * dey
        v_lat = max(-self.max_vel, min(self.max_vel, v_lat))
        v_fwd = max(-self.max_vel, min(self.max_vel, v_fwd))

        self.prev_ex, self.prev_ey = ex, ey
        return v_fwd, v_lat, math.hypot(ex, ey), ex, ey


class DropOptMission(Node):
    def __init__(self):
        super().__init__('drop_opt_mission')

        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.manual_pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', 10)
        self.aux5_pwm_pub = self.create_publisher(Float32, '/drop/aux5_pwm', 10)
        self.impact_pub = self.create_publisher(Point, '/drop/impact_point', 10)

        self.status_sub = self.create_subscription(VehicleStatus, status_topic(), self._cb_status, QOS_BE)
        self.local_sub = self.create_subscription(VehicleLocalPosition, local_pos_topic(), self._cb_local, QOS_BE)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self._cb_land, QOS_BE)
        self.detect_sub = self.create_subscription(RegionOfInterest, '/bucket/detection', self._cb_detect, 10)
        self.control_mode_sub = self.create_subscription(VehicleControlMode, control_mode_topic(), self._cb_control_mode, QOS_BE)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._cb_attitude, QOS_BE)

        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self._takeoff_x = self._takeoff_y = self._takeoff_z = None
        self.heading = None
        self._heading0 = None
        self._stabilize_yaw = None
        self.landed = False
        self.status = None
        self._control_mode = None

        self.latest_roi = None
        self._last_detect_time = self.get_clock().now()
        self._last_roi_key = None
        self._roi_dup_count = 0

        self._pitch = 0.0
        self._roll = 0.0

        self.step = STEP_TAKEOFF
        self._step_start_time = None
        self.pid = PIDController(KP_H, KI_H, KD_H, MAX_HORIZ_VEL, INTEGRAL_LIMIT, DEAD_ZONE_PX)
        self._stabilize_start = None
        self._detect_loss_logged = False
        self._detect_loss_start = None
        self._confirm_start_time = None
        self._confirm_first_cx = 0.0
        self._confirm_first_cy = 0.0
        self._stabilize_lost_frames = 0

        self._aux5_value = AUX5_LOCK
        self._drop_triggered = False
        self._drop_signal_start = None

        self._land_dwell_start = None
        self._land_cmd_sent = False

        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._last_arm_attempt = -10.0
        self._init_wait_start = None
        self._initial_wait_done = False

        self._track_max_time = TRACK_MAX_TIME
        self._hover_max_time = HOVER_MAX_TIME

        self._print_ctr = 0
        self._last_pid_print = 0.0
        self._last_vel_x = 0.0
        self._last_vel_y = 0.0
        self._last_vel_z = 0.0
        self._start_wall = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.manual_timer = self.create_timer(MANUAL_TIMER_PERIOD, self._publish_manual_control)
        self.get_logger().info(
            f"Drop Opt Mission v2 ready [real-drone-proven params]. "
            f"KP={KP_H} KD={KD_H} MAX={MAX_HORIZ_VEL} "
            f"fwd={FORWARD_DIST}m cruise={CRUISE_HEIGHT}m drop={DROP_HEIGHT}m "
            f"stabilize={STABILIZE_HOVER_TIME}s search_to={HOVER_SEARCH_TIMEOUT}s "
            f"gate: dist<={TRACK_ENTRY_DIST_MAX}m geo<={TRACK_ENTRY_GEO_ERR_MAX}m")

    # ========== 回调 ==========
    def _cb_status(self, m): self.status = m
    def _cb_control_mode(self, m): self._control_mode = m
    def _cb_land(self, m): self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading
        # v2: 仅记录起飞参考点, heading0 和 home 延后到 STABILIZE 锁定
        if self._takeoff_x is None and m.xy_valid and self.heading is not None:
            self._takeoff_x, self._takeoff_y, self._takeoff_z = self.x, self.y, self.z
            self.get_logger().info(
                f"Takeoff ref: NED({self._takeoff_x:.2f},{self._takeoff_y:.2f},{self._takeoff_z:.2f})")

    def _cb_attitude(self, m):
        w, x, y_, z_ = float(m.q[0]), float(m.q[1]), float(m.q[2]), float(m.q[3])
        self._roll  = math.atan2(2.0*(w*x + y_*z_), 1.0 - 2.0*(x*x + y_*y_))
        self._pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y_ - z_*x))))

    def _cb_detect(self, m):
        roi_key = (m.x_offset, m.y_offset, m.width, m.height)
        if roi_key == self._last_roi_key:
            self._roi_dup_count += 1
            if self._roi_dup_count < 5:
                self._last_detect_time = self.get_clock().now()
        else:
            self._roi_dup_count = 0
            self._last_roi_key = roi_key
            self._last_detect_time = self.get_clock().now()
        self.latest_roi = m

    # ========== 发布 ==========
    def _pub_vel(self, vx, vy, vz, yaw):
        self._last_vel_x = vx; self._last_vel_y = vy; self._last_vel_z = vz
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

    # ========== 门控 ==========
    def _confirm_detection_stable(self):
        if not self._detect_fresh(): self._confirm_start_time = None; return False
        roi = self.latest_roi
        cx = roi.x_offset + roi.width / 2.0; cy = roi.y_offset + roi.height / 2.0
        if self._confirm_start_time is None:
            self._confirm_start_time = self.get_clock().now()
            self._confirm_first_cx = cx; self._confirm_first_cy = cy; return False
        drift = math.hypot(cx - self._confirm_first_cx, cy - self._confirm_first_cy)
        if drift > TRACK_ENTRY_ROI_DRIFT: self._confirm_start_time = None; return False
        return (self.get_clock().now() - self._confirm_start_time).nanoseconds * 1e-9 >= TRACK_ENTRY_CONFIRM_TIME

    def _enter_step(self, step):
        self.step = step; self._step_start_time = self.get_clock().now()
        self._stabilize_start = None
        self._detect_loss_logged = False; self._detect_loss_start = None
        self._confirm_start_time = None; self._stabilize_lost_frames = 0
        self._drop_triggered = False; self._drop_signal_start = None
        if step == STEP_DESCEND: self.pid.reset()
        if step == STEP_STABILIZE: self._stabilize_yaw = None

    def _step_elapsed(self):
        if self._step_start_time is None: return 0.0
        return (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

    def _detect_fresh(self):
        if self.latest_roi is None: return False
        return (self.get_clock().now() - self._last_detect_time).nanoseconds * 1e-9 < DETECT_TIMEOUT

    # ========== 几何工具 ==========
    def _roi_to_ned(self, roi):
        if self.heading is None or self.home_z is None: return None
        H = self.home_z - self.z
        if H < 0.3: return None
        cos_p = math.cos(self._pitch)
        if cos_p < 0.5: return None
        H_eff = H / cos_p; pitch_shift_px = FY * math.tan(self._pitch)
        cx = roi.x_offset + roi.width / 2.0; cy = roi.y_offset + roi.height / 2.0
        cam_right = (cx - CX) * H_eff / FX
        cam_fwd   = (CY + pitch_shift_px - cy) * H_eff / FY
        dx, dy, _ = self._body_to_ned(cam_fwd, cam_right)
        return (self.x + dx, self.y + dy)

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * cos_h - v_lat * sin_h,
                v_fwd * sin_h + v_lat * cos_h, v_down)

    def _bucket_target(self):
        if self.home_x is None or self._heading0 is None: return (0.0, 0.0)
        cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
        return (self.home_x + FORWARD_DIST * cos_h0,
                self.home_y + FORWARD_DIST * sin_h0)

    def _fly_toward(self, tx, ty, speed, vz=0.0):
        dist = math.hypot(self.x - tx, self.y - ty)
        if dist > 0.1:
            ratio = max(FORWARD_MIN_SPEED / CRUISE_SPEED, min(1.0, dist / 3.0))
            vx = (tx - self.x) / dist * speed * ratio
            vy = (ty - self.y) / dist * speed * ratio
        else: vx = vy = 0.0
        return vx, vy, dist

    def _handle_detect_loss(self, target_z):
        if not self._detect_loss_logged:
            self._detect_loss_logged = True
            self.get_logger().warn(f"Detection lost in {STEP_NAME[self.step]}, hovering...")
        self._pub_vel(0.0, 0.0, (target_z - self.z) * TRACK_VZ_KP, self.heading)

    # ========== ManualControlSetpoint ==========
    def _publish_manual_control(self):
        msg = ManualControlSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.valid = True; msg.data_source = ManualControlSetpoint.SOURCE_RC
        msg.roll = msg.pitch = msg.yaw = 0.0; msg.throttle = -1.0; msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = 0.0
        msg.aux5 = float(self._aux5_value); msg.aux6 = 0.0
        msg.sticks_moving = True; msg.buttons = 0
        self.manual_pub.publish(msg)
        if not hasattr(self, '_aux5_pwm_ctr'): self._aux5_pwm_ctr = 0
        self._aux5_pwm_ctr += 1
        if self._aux5_pwm_ctr >= 20:
            self._aux5_pwm_ctr = 0
            self.aux5_pwm_pub.publish(Float32(data=float(1500.0 + self._aux5_value * 600.0)))

    # ================================================================
    # 主循环
    # ================================================================
    def control_loop(self):
        # v2: 用 _takeoff_x 做入口守卫 (home 延后到 STABILIZE 才锁定)
        if self._takeoff_x is None or self.heading is None: return

        sim_t = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
        self._print_ctr += 1
        if self._print_ctr >= 20:
            tag = STEP_NAME.get(self.step, "???"); hdg = math.degrees(self.heading) if self.heading else 0
            has_det = "DET" if self._detect_fresh() else "no-det"; extra = ""
            if self.step == STEP_STABILIZE:
                extra = f" stabilizing {self._step_elapsed():.1f}s/{STABILIZE_HOVER_TIME}s"
            elif self.step == STEP_FORWARD and self.home_x is not None:
                tx, ty = self._bucket_target()
                extra = f" dist_to_target={math.hypot(self.x-tx,self.y-ty):.1f}m"
            elif self.step == STEP_HOVER_SEARCH:
                extra = f" search {self._step_elapsed():.1f}s/{HOVER_SEARCH_TIMEOUT}s"
            elif self.step == STEP_RETURN and self.home_x is not None:
                extra = f" ->home dist={math.hypot(self.x-self.home_x,self.y-self.home_y):.1f}m"
            self.get_logger().info(f"[{tag}] NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) h={hdg:.0f}deg {has_det}{extra}")
            self._print_ctr = 0

        # 初始等待 (DDS 就绪)
        if not self._initial_wait_done:
            if self._init_wait_start is None: self._init_wait_start = self.get_clock().now()
            self._pub_vel(0.0, 0.0, 0.0, self.heading)
            elapsed = (self.get_clock().now() - self._init_wait_start).nanoseconds * 1e-9
            dds_ready = self._control_mode is not None or (self.status is not None and self.status.pre_flight_checks_pass)
            if dds_ready and elapsed > 3.0: self._initial_wait_done = True
            elif elapsed > 30.0: self.timer.cancel(); return
            return

        # ARM / OFFBOARD (v2: 使用 _takeoff_x/y 而非 home)
        cm = self._control_mode; st = self.status
        armed = (cm.flag_armed if cm is not None else (st.arming_state == 2 if st is not None else False))
        offboard = (cm.flag_control_offboard_enabled if cm is not None else (st.nav_state == 14 if st is not None else False))
        if cm is None and st is None: self._pub_vel(0, 0, 0, self.heading); return
        if self._mode_req_start is None: self._mode_req_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._mode_req_start).nanoseconds * 1e-9
        ref_x, ref_y = self._takeoff_x, self._takeoff_y
        ref_z = self._takeoff_z - CRUISE_HEIGHT
        yaw_live = self.heading if self.heading else 0.0
        if self.step == STEP_LAND and self._land_cmd_sent: pass
        elif armed and offboard: pass
        elif elapsed > 30.0: self.get_logger().error("Mode timeout"); self.timer.cancel(); return
        elif not armed:
            if not self._arm_req or elapsed - self._last_arm_attempt > 1.0:
                self._arm_req = True; self._last_arm_attempt = elapsed; send_arm(self.cmd_pub, self)
            self._pub_pos(ref_x, ref_y, self._takeoff_z, yaw_live); return
        elif not offboard:
            if not self._mode_req: self._mode_req = True; self._send_cmd(176, param1=1.0, param2=6.0)
            self._pub_pos(ref_x, ref_y, self._takeoff_z, yaw_live); return

        # ============================================================
        # 状态机
        # ============================================================
        cruise_z = self._takeoff_z - CRUISE_HEIGHT
        yaw_live = self.heading if self.heading is not None else 0.0
        yaw_locked = self._heading0 if self._heading0 is not None else yaw_live

        # ---- TAKEOFF: 位置保持 → 竖直爬升到巡航高度 ----
        if self.step == STEP_TAKEOFF:
            if self._step_start_time is None:
                self._enter_step(STEP_TAKEOFF)
            elapsed_tk = self._step_elapsed()
            if elapsed_tk < 2.0:
                # 初始位置保持, 让飞机稳定
                self._pub_pos(self._takeoff_x, self._takeoff_y, self._takeoff_z, yaw_live)
            else:
                # 竖直爬升
                climb_elapsed = elapsed_tk - 2.0
                z_ramp = self._takeoff_z - min(CRUISE_HEIGHT, climb_elapsed * TAKEOFF_CLIMB_RATE)
                self._pub_pos(self._takeoff_x, self._takeoff_y, z_ramp, yaw_live)
            if abs(self.z - cruise_z) < 0.5 and elapsed_tk > 4.0:
                self.get_logger().info("Reached cruise height, entering STABILIZE")
                self._enter_step(STEP_STABILIZE)

        # ---- STABILIZE: 悬停稳姿态 → 锁定 heading0 和 home ----
        elif self.step == STEP_STABILIZE:
            # 进入时捕捉固定 yaw, 悬停让 heading 真正收敛
            if self._stabilize_yaw is None:
                self._stabilize_yaw = self.heading if self.heading else 0.0
            self._pub_pos(self._takeoff_x, self._takeoff_y, cruise_z, self._stabilize_yaw)
            if self._step_elapsed() >= STABILIZE_HOVER_TIME:
                # 锁定 heading0 和 home 位置 (home 用起飞点地面高度, 非巡航高度)
                self._heading0 = self.heading
                self.home_x, self.home_y = self._takeoff_x, self._takeoff_y
                self.home_z = self._takeoff_z
                bx, by = self._bucket_target()
                self.get_logger().info(
                    f"Home locked: NED({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                    f"heading0={math.degrees(self._heading0):.0f}deg target NED({bx:.1f},{by:.1f})")
                self._enter_step(STEP_FORWARD)

        # ---- FORWARD: 沿 heading0 直线飞行 (速度+yaw 同一方向, 0偏角) ----
        elif self.step == STEP_FORWARD:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_HOVER_SEARCH); return
            tx, ty = self._bucket_target()
            dist_to_target = math.hypot(self.x - tx, self.y - ty)
            ratio = max(FORWARD_MIN_SPEED / CRUISE_SPEED, min(1.0, dist_to_target / FORWARD_RAMP_DIST))
            speed = CRUISE_SPEED * ratio
            cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
            vx = speed * cos_h0
            vy = speed * sin_h0
            self._pub_vel(vx, vy, (cruise_z - self.z) * TRACK_VZ_KP, self._heading0)
            if dist_to_target < POS_ARRIVE_DIST:
                self.get_logger().info(f"Arrived at target, dist={dist_to_target:.2f}m, entering HOVER_SEARCH")
                self._enter_step(STEP_HOVER_SEARCH)

        # ---- HOVER_SEARCH: 目标点悬停, 搜索有效目标 ----
        elif self.step == STEP_HOVER_SEARCH:
            if self._step_elapsed() > HOVER_SEARCH_TIMEOUT:
                self.get_logger().info("No valid target for 10s, returning home")
                self._enter_step(STEP_RETURN); return
            tx, ty = self._bucket_target()
            # 驻留期: 纯位置保持, 等 MPC 彻底收敛再开始搜目标
            if self._step_elapsed() < HOVER_SEARCH_SETTLE_TIME:
                self._pub_pos(tx, ty, cruise_z, yaw_locked)
                return
            dist_to_bucket = math.hypot(self.x - tx, self.y - ty)
            # 四层门控
            gate_open = self._detect_fresh() and dist_to_bucket < TRACK_ENTRY_DIST_MAX
            if gate_open:
                est = self._roi_to_ned(self.latest_roi)
                if est is not None: gate_open = math.hypot(est[0] - tx, est[1] - ty) < TRACK_ENTRY_GEO_ERR_MAX
            if gate_open:
                if self._confirm_detection_stable():
                    self.get_logger().info("Valid target confirmed, entering TRACK")
                    self.pid.reset(); self._enter_step(STEP_TRACK); return
            else:
                self._confirm_start_time = None
            # 位置保持在目标点
            self._pub_pos(tx, ty, cruise_z, yaw_locked)

        # ---- TRACK: PID 视觉伺服 ----
        elif self.step == STEP_TRACK:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_RETURN); return
            if not self._detect_fresh():
                if self._step_elapsed() > self._track_max_time: self._enter_step(STEP_DESCEND); return
                self._handle_detect_loss(cruise_z); return
            self._detect_loss_logged = False
            H = max(0.3, self.home_z - self.z); height_scale = H / CRUISE_HEIGHT
            v_fwd, v_lat, err_px, ex, ey = self.pid.compute(self.latest_roi, 0.05, pitch=self._pitch, roll=self._roll, height_scale=height_scale)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            self._pub_vel(vx, vy, (cruise_z-self.z)*TRACK_VZ_KP, self.heading)
            if err_px < TRACK_ARRIVE_PX:
                self._stabilize_lost_frames = 0
                if self._stabilize_start is None: self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now()-self._stabilize_start).nanoseconds > STABILIZE_TIME*1e9: self._enter_step(STEP_DESCEND)
            elif self._step_elapsed() > self._track_max_time: self._enter_step(STEP_DESCEND)
            else:
                self._stabilize_lost_frames += 1
                if self._stabilize_lost_frames > STABILIZE_LOST_FRAMES_MAX: self._stabilize_start = None; self._stabilize_lost_frames = 0

        # ---- DESCEND: 下降同时水平伺服 ----
        elif self.step == STEP_DESCEND:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_DROP); return
            drop_z = self.home_z - DROP_HEIGHT
            if not self._detect_fresh():
                if self._detect_loss_start is None: self._detect_loss_start = self.get_clock().now()
                loss_t = (self.get_clock().now()-self._detect_loss_start).nanoseconds*1e-9
                if loss_t >= 3.0:
                    if self.z > drop_z-0.2: self._enter_step(STEP_DROP)
                    else: self._pub_vel(0, 0, DESCEND_SPEED, self.heading)
                else: self._pub_vel(0, 0, DESCEND_SPEED, self.heading)
                return
            self._detect_loss_start = None; self._detect_loss_logged = False
            dz = max(0.3, self.home_z - self.z); pixel_offset = PAYLOAD_OFFSET_BODY_Y*FX/dz
            v_fwd, v_lat, err_px, ex, ey = self.pid.compute(self.latest_roi, 0.05, pixel_offset, self._pitch, self._roll, height_scale=dz/CRUISE_HEIGHT)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            self._pub_vel(vx, vy, DESCEND_SPEED, self.heading)
            if self.z > drop_z-0.2: self._enter_step(STEP_DROP)

        # ---- HOVER: 投放前低空稳定 ----
        elif self.step == STEP_HOVER:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_DROP); return
            drop_z = self.home_z - DROP_HEIGHT; hover_elapsed = self._step_elapsed()
            if not self._detect_fresh():
                if self._detect_loss_start is None: self._detect_loss_start = self.get_clock().now()
                if (self.get_clock().now()-self._detect_loss_start).nanoseconds*1e-9 >= 3.0: self._enter_step(STEP_DROP)
                else:
                    vz = (drop_z-self.z)*HOVER_VZ_KP; vz = max(-0.3, min(0.3, vz))
                    self._pub_vel(0, 0, vz, self.heading)
                return
            self._detect_loss_start = None; self._detect_loss_logged = False
            if hover_elapsed > self._hover_max_time: self._enter_step(STEP_DROP); return
            dz = max(0.3, self.home_z - self.z); pixel_offset = PAYLOAD_OFFSET_BODY_Y*FX/dz
            v_fwd, v_lat, err_px, ex, ey = self.pid.compute(self.latest_roi, 0.05, pixel_offset, self._pitch, self._roll, height_scale=dz/CRUISE_HEIGHT)
            vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            vz = (drop_z-self.z)*HOVER_VZ_KP; vz = max(-0.3, min(0.3, vz))
            self._pub_vel(vx, vy, vz, self.heading)
            if err_px < HOVER_ARRIVE_PX:
                self._stabilize_lost_frames = 0
                if self._stabilize_start is None: self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now()-self._stabilize_start).nanoseconds > STABILIZE_TIME*1e9: self._enter_step(STEP_DROP)
            else:
                self._stabilize_lost_frames += 1
                if self._stabilize_lost_frames > STABILIZE_LOST_FRAMES_MAX: self._stabilize_start = None; self._stabilize_lost_frames = 0

        # ---- DROP: 投弹 ----
        elif self.step == STEP_DROP:
            if self._step_elapsed() > STEP_TIMEOUT: self._aux5_value = AUX5_LOCK; self._enter_step(STEP_CLIMB); return
            drop_z = self.home_z - DROP_HEIGHT
            dz = max(0.3, self.home_z - self.z); pixel_offset = PAYLOAD_OFFSET_BODY_Y*FX/dz
            if self._detect_fresh():
                v_fwd, v_lat, _, ex, ey = self.pid.compute(self.latest_roi, 0.05, pixel_offset, self._pitch, self._roll, height_scale=dz/CRUISE_HEIGHT)
                vx, vy, _ = self._body_to_ned(v_fwd, v_lat)
            else: vx, vy, ex, ey = 0, 0, 0, 0
            vz = (drop_z-self.z)*HOVER_VZ_KP; vz = max(-0.2, min(0.2, vz))
            self._pub_vel(vx, vy, vz, self.heading)
            if self._step_elapsed() < DROP_AFTER_DELAY: return
            if not self._drop_triggered:
                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now(); self._aux5_value = AUX5_DROP
                self.impact_pub.publish(Point(x=float(self.x), y=float(self.y), z=float(self.home_z)))
                self.get_logger().info(f">>> DROP! NED({self.x:.3f},{self.y:.3f},{self.z:.3f}) heading={math.degrees(self.heading):.1f}deg"); return
            if (self.get_clock().now()-self._drop_signal_start).nanoseconds*1e-9 < AUX5_DROP_HOLD: return
            self._aux5_value = AUX5_LOCK; self._drop_triggered = False; self._drop_signal_start = None
            self._enter_step(STEP_CLIMB)

        # ---- CLIMB: 投弹后爬升回巡航高度 ----
        elif self.step == STEP_CLIMB:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_RETURN); return
            vz = CLIMB_VZ + (cruise_z-self.z)*TRACK_VZ_KP
            self._pub_vel(0, 0, vz, yaw_locked)
            if abs(self.z-cruise_z) < 0.5: self._enter_step(STEP_RETURN)

        # ---- RETURN: 返航 ----
        elif self.step == STEP_RETURN:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_LAND); return
            vx, vy, dist_to_home = self._fly_toward(self.home_x, self.home_y, CRUISE_SPEED)
            self._pub_vel(vx, vy, (cruise_z-self.z)*TRACK_VZ_KP, yaw_locked)
            if dist_to_home < POS_ARRIVE_DIST and abs(self.z-cruise_z) < 0.5: self._enter_step(STEP_LAND)

        # ---- LAND: 着陆 ----
        elif self.step == STEP_LAND:
            if self._step_elapsed() > STEP_TIMEOUT: self._send_cmd(400, param1=0.0); self.timer.cancel(); return
            if not self._land_cmd_sent:
                low_z = self.home_z - 0.5
                self._pub_pos(self.home_x, self.home_y, low_z, yaw_locked)
                if abs(self.z-low_z) < 0.5:
                    self._land_cmd_sent = True; self._send_cmd(21, param1=0.0, param2=0.0, param7=float('nan'))
            else:
                if self.landed:
                    if self._land_dwell_start is None: self._land_dwell_start = self.get_clock().now()
                    elif (self.get_clock().now()-self._land_dwell_start).nanoseconds > LAND_DWELL_TIME*1e9: self._send_cmd(400, param1=0.0); self.timer.cancel()
                elif abs(self.home_z - self.z) < 0.15:
                    if self._land_dwell_start is None: self._land_dwell_start = self.get_clock().now()
                    elif (self.get_clock().now()-self._land_dwell_start).nanoseconds > LAND_DWELL_TIME*1e9: self._send_cmd(400, param1=0.0); self.timer.cancel()
                else: self._land_dwell_start = None


def main():
    rclpy.init()
    node = DropOptMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
