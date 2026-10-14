#!/usr/bin/env python3
# ============================================================================
# 【SITL 导入版 — 2026-10-07】外接盘 "f r3" (predictive / continuous-brake-hold)
#   源文件   : /media/vincent/25390BC89B897322/f_20261006_predictive/f.py
#   源 sha256: e1a32bf1facaebe7533285614e87d9c8750704fbacfda20130df9131c77e3f9f
#   本文件   = 源文件 + 下列 6 处最小适配 (控制逻辑一行未改)
#     ① 类名 FMission -> FR3Mission   ② 节点名 f_mission -> f_r3_mission
#     ③ main() 实例化同名            ④ PX4_STATUS_SUFFIX "_v1" -> "_v4" (SITL 固件)
#     ⑤ HEADING_DEG 161.0 -> 92.0 (i.world 场地航向)
#     ⑥ MISSION_REVISION 追加 "-sitl-import"
#   设计要点(原样保留): 低空 XY 位置目标限速 8cm/s + 按实测速度提前 0.6s 制动 +
#     制动时钉固定刹停点; 释放判据 = 两帧新帧 ≤5cm 且 |vxy|≤0.10 (需检测器发 frame_seq);
#     35s 无条件放水; 补偿 ±0.0567 / 前后 0 (台架几何, 未经实飞标定)
#   ⚠ 检测器必须发 frame_seq, 否则低空伺服完全不动作 (设定点锚死 + 35s 兜底)
# ============================================================================
"""
f 2026-10-06: 低空限速视觉伺服 + 速度预判制动 + 每桶超时必发放水指令。

基底是仓库 e/r7（SHA256 173ee2e5...7eda1），不是声称恢复了 17:40 的精确 f 源码。
VISUAL_SERVO/DROP: XY 位置设定点缓移并提前制动，Z 位置保持；新帧更新米制误差，
连续两帧对准即投，35s 未对准也释放，完整保持舵机脉冲后才离开 DROP。
原任务的选桶、PRE_SERVO 下降、侦察、返航流程沿用基底。
"""

import collections
import json
import hashlib
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected,
    ManualControlSetpoint, VehicleControlMode, VehicleAttitude,
)
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point

# ============================================================
# 实机 QoS & 话题 (内联, 不依赖 common.py)
# ============================================================

QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)
QOS_LATEST = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

# 实机 PX4 固件使用 _v1 后缀 (fly/ 已验证)
PX4_STATUS_SUFFIX = "_v4"   # 2026-10-07 SITL 导入: SITL 固件发布 vehicle_status_v4 (_v1=实机)
PX4_LOCAL_POS_SUFFIX = "_v1"

STATUS_TOPIC = f"/fmu/out/vehicle_status{PX4_STATUS_SUFFIX}"
LOCAL_POS_TOPIC = f"/fmu/out/vehicle_local_position{PX4_LOCAL_POS_SUFFIX}"
LAND_DETECTED_TOPIC = "/fmu/out/vehicle_land_detected"
CONTROL_MODE_TOPIC = "/fmu/out/vehicle_control_mode"

def send_arm(cmd_pub, node):
    cmd_pub.publish(VehicleCommand(
        timestamp=node.get_clock().now().nanoseconds // 1000,
        command=400, param1=1.0, param2=2989.0,
        target_system=1, target_component=1,
        source_system=1, source_component=1, from_external=True))

# ============================================================
# 可调参数 — 实机版
# ============================================================
MISSION_TIMEOUT = 600.0    # 全局超时 10min, 超时强制着陆

# 地面 NED z 坐标。实机: 地面 ≈ 起飞点 ≈ EKF 原点
GROUND_NED_Z = 0.0          # 实机默认 0; SITL iris spawn 0.83

CRUISE_HEIGHT = 5.0
DROP_HEIGHT   = 1.2     # 2026-09-13: 1.5→1.2 (按用户要求降低投弹高度). 低空像素/米比更小 → 同样 10px 收敛门限 ≈3.4cm (原 4.2cm), 反投影随 H 同比偏小;
                        #   注意: 视野覆盖降到 ±1.11m, 下洗更强 — 复飞重点看 DROP 段是否丢检测
FORWARD_DIST  = 34.0      # 2026-08-14: 32→34, 实机起飞点比假定位置后退 2m, 所有长度+2m

SCOUT_DIST = 59.5          # 2026-08-14: 57.5→59.5, 起飞点后退 2m. 侦察区正上方(预定点)
SCOUT_EDGE_DIST = 57.0     # 2026-08-15: 侦察区第一根线(近边)沿 heading0 距离, 直飞目标改近边防过冲 (SITL=55.0)
SCOUT_SLOW_MOVE_TIME = 10.0  # 2026-08-15: 近边→预定点 缓慢移动时长 (s), 0.45m/s
SCOUT_DESCENT_DELTA = 1.0  # 2026-08-15: 到达近边后先下降 1m (悬停5s子阶段已取消)
SCOUT_LEFT_DIST = 1.5      # 2026-08-15: 1.0→1.5, 机体左移 1.5m
SCOUT_RIGHT_DIST = 3.0     # 2026-08-15: 2.0→3.0, 再右移 3m (终点=预定点右侧1.5m)
SCOUT_PAUSE_TIME = 5.0     # 2026-08-15: 左右移动后各停 5s
SCOUT_SUB_ARRIVE = 0.3     # 侦察子动作到位判据 (m)
SCOUT_SUB_TIMEOUT = 10.0   # 侦察子动作超时 (s)

RETURN_STAB_DRIFT_MAX = 0.3  # 2026-08-15: 返航后稳定判据 — 窗口内水平漂移 <0.3m
RETURN_STAB_WINDOW    = 3.0  # 漂移监测窗口时长 (s)
RETURN_STAB_TIMEOUT   = 10.0 # 稳定等待超时, 强制进入 LAND (s)
LAND_DESCENT_SPEED    = 0.4  # 2026-08-15: LAND 前下降限速 (m/s), 时间锚定, 防降太猛

TRACK_ARRIVE_PX  = 40.0
HOVER_ARRIVE_PX  = 25.0
STABILIZE_TIME   = 5.0
TRACK_MAX_TIME   = 60.0
HOVER_MAX_TIME   = 30.0

CRUISE_SPEED    = 2.0
RETURN_SPEED    = 3.0
FORWARD_RAMP_DIST = 3.0
FORWARD_MIN_SPEED = 0.5
FORWARD_EXIT_CONFIRM_FRAMES = 5  # 连续 N 帧满足退出条件才真正退出, 防御 GPS 瞬时跳变
SEARCH_SPEED     = 0.3
SERVO_DESCENT_SPEED = 0.25   # 基底遗留参数：旧 max() 下降式不构成实际限速
PRE_SERVO_HEIGHT = 2.5     # 预伺服下降目标高度, 低空 pinhole 更准
HOVER_VZ_KP      = 0.8
TRACK_VZ_KP      = 0.5
CLIMB_VZ         = -0.8

# 低空横向闭环：限制位置目标的移动速度，不再每帧跳到反投影位置。
VISUAL_SERVO_TIMEOUT = 35.0
LOW_SERVO_MAX_SPEED = 0.08       # m/s，XY 位置目标移动速度总限幅（不等于机体硬限速）
LOW_SERVO_SLOW_RADIUS_M = 0.20   # v_cap = 0.08*d/(d+0.20); d=0.12m 时 0.03m/s
LOW_SERVO_KP = 0.60             # 1/s，作用于米制预判误差
LOW_SERVO_BRAKE_LOOKAHEAD = 0.60 # s，误差预判 e_pred=e-v_actual*T；是调参起点
LOW_SERVO_BRAKE_ENTER_SPEED = 0.06  # m/s，满足几何制动条件时的进入速度门限
LOW_SERVO_BRAKE_RESUME_SPEED = 0.04 # m/s，低于进入门限，避免制动/微调反复切换
LOW_SERVO_BRAKE_RESUME_SPAN = 0.20  # s，连续新帧低速确认时间
LOW_SERVO_BRAKE_RESUME_MARGIN = 0.02 # m，恢复时预计剩余距离至少 2cm
LOW_SERVO_FILTER_TAU = 0.10     # s，按新帧实际 dt 计算滤波权重
LOW_SERVO_DETECT_MAX_AGE = 0.50 # s，短暂帧间隔允许预测；精投仍使用 0.25s 门限
LOW_SERVO_MAX_LEAD_M = 0.20     # m，位置目标最多领先机体 20cm，防止累积漂移
LOW_SERVO_MAX_ACCEL = 0.15      # m/s²，目标移动速度变化限制；丢检保留最后位置目标

DETECT_TIMEOUT = 3.0
STEP_TIMEOUT   = 120.0

# 2026-09-12 e: 飞行边界软墙按用户指示完全移除 (自 d 移植).
# 原因 (实机 d 试飞): 目标桶侧向超出 ±4m 时软墙每 1-2s 触发, 用 _pub_vel 抢控制权并 `return`
# 跳过整个状态机 → 视觉伺服 _pub_pos / 收敛判定 / 投弹计时全部停摆 → err 发散, 任务中止.
# (原 LATERAL_BOUNDARY=4.0 / BOUNDARY_PUSH_SPEED=0.5 常量已随机制删除)

HOVER_SEARCH_ZERO_DETECT_TIMEOUT = 5.0  # 搜索阶段零检测超时 (s)

TRACK_ENTRY_DIST_MAX     = 3.0
TRACK_ENTRY_CONFIRM_TIME = 1.5
TRACK_ENTRY_ROI_DRIFT    = 120.0
TRACK_ENTRY_GEO_ERR_MAX  = 1.5
STABILIZE_LOST_FRAMES_MAX = 10

POS_ARRIVE_DIST  = 0.5
LAND_DWELL_TIME  = 3.0
# 相机位于两出水口正中; 右侧(2000us)和左侧(1000us)均距相机 56.7mm.
# 水实际落点还会受风、机体姿态和出水方向影响; 场地俯视的右前偏差包含旧版放水时的瞄准误差,
# 未测得独立的水流偏置, 不叠加 10–20cm 估计值以免重复补偿.
PAYLOAD_OFFSET_BODY_Y_B1 =  0.0567
PAYLOAD_OFFSET_BODY_Y_B2 = -0.0567
PAYLOAD_OFFSET_BODY_X_B1 = 0.0   # 相机与出水口无前后距离
PAYLOAD_OFFSET_BODY_X_B2 = 0.0
DROP_FORCE_AFTER = 35.0         # 每桶从进入 DROP 计时，到时无条件执行本桶释放流程
DROP_ARRIVE_M = 0.05            # 当前原始图像所估出水口到桶中心的平面距离
DROP_RELEASE_SPEED_MPS = 0.10   # 防止高速穿过中心被误判为对准
DROP_RELEASE_VZ_MPS = 0.15
DROP_HEIGHT_TOL_M = 0.20
DROP_CONFIRM_FRAMES = 2         # 独立的新匹配帧；不以控制 tick 代替检测帧
DROP_CONFIRM_MIN_SPAN = 0.08    # s，默认 10Hz 下约 0.1s 确认
DROP_CONFIRM_MAX_GAP = 0.25     # s，坏帧/陈旧帧打断确认
DROP_DETECT_MAX_AGE = 0.25       # 接收年龄，不能代表相机曝光/推理总延迟
MISSION_REVISION = "competition-water-20261006-r3-f-continuous-brake-hold-sitl-import"

AUX5_LOCK  =  0.0
AUX5_DROP  =  0.83333   # B1: PWM 2000us
AUX5_DROP2 = -0.83333   # B2: PWM 1000us
AUX5_DROP_HOLD = 1.0
MANUAL_TIMER_PERIOD = 0.005

# 固定航向 (不再起飞前锁机头, 直接测场地填)
HEADING_DEG = 92.0  # 2026-10-07 SITL 导入: i.world 场地航向 (与 SITL i.py 一致); 原实机值 161.0

IMG_W, IMG_H = 640, 360
CX, CY = IMG_W / 2.0, IMG_H / 2.0
HFOV = math.radians(85)
VFOV = math.radians(54)
FX = IMG_W / (2.0 * math.tan(HFOV / 2.0))
FY = IMG_H / (2.0 * math.tan(VFOV / 2.0))

# 多桶追踪器 (v9): 每个候选桶独立时间确认, 防御瞬时误检
TRACKER_CONFIRM_TIME  = 1.5   # 持续观测 ≥1.5s 才算确认
TRACKER_TIMEOUT       = 2.0   # 超过此时间未更新则丢弃追踪器
TRACKER_NED_MATCH     = 1.5   # NED 最近邻匹配距离 (m)
# 2026-09-20 晚 (A+C): 已投过的桶不再参与任何匹配/瞄准; 目标点用活追踪器刷新
SERVED_EXCLUDE_M      = 0.60  # A: 已投过的桶: 落在此半径内的框一律不作候选 (桶半径 0.125m)
TARGET_REFRESH_MAX_M  = 2.00  # C: 用追踪器刷新瞄准点时允许的最大偏差 (仅作合理性门控)
TARGET_REBIND_MAX_M   = 0.60  # C: refine 后重新认领追踪器 id 的最大距离 (紧门限, << 桶间距)
TARGET_REBIND_MARGIN  = 0.30  # C: 认领时要求比第二名近这么多 (防歧义)
DET_DUP_MERGE_DIST   = 0.8   # 2026-09-12: 同帧内两检测框 NED 估计相距 < 此值视为同一个桶, 只保留较大的框

# v2 新增参数
STABILIZE_HOVER_TIME = 3.0    # 起飞后稳定悬停时间, 之后锁定 heading0
HOVER_SEARCH_TIMEOUT = 10.0   # 目标点搜索超时, 无有效目标则返航
HOVER_SEARCH_SETTLE_TIME = 1.5  # 驻留等 MPC 刹车, 期间跳过门控
TAKEOFF_CLIMB_RATE   = 0.8    # 竖直爬升速率 (m/s)

# POSITION_REFINE — TRACK 到位后悬停重新估算桶坐标 (v4)
POSITION_REFINE_DURATION   = 2.0   # 悬停收集时间
POSITION_REFINE_MIN_SAMPLES = 5    # 最少有效样本数
POSITION_REFINE_CENTER_MAX = 150.0 # 图像中心附近门限 (px)
POSITION_REFINE_MAX_CORRECTION = 1.5  # 最大修正距离 (m), 超出视为身份混淆
POSITION_REFINE_IDENTITY_MAX = 1.5   # 几何身份锁门限 (m), v6: 2.0->1.5 < 最小桶间距 2.2m

# ============================================================
# 状态
# ============================================================
(STEP_TAKEOFF, STEP_STABILIZE, STEP_FORWARD, STEP_HOVER_SEARCH,
 STEP_TRACK, STEP_POSITION_REFINE, STEP_PRE_SERVO, STEP_VISUAL_SERVO, STEP_DROP,
 STEP_CLIMB, STEP_SCOUT, STEP_RETURN, STEP_LAND) = range(13)

STEP_NAME = {
    STEP_TAKEOFF: "TAKEOFF", STEP_STABILIZE: "STABILIZE",
    STEP_FORWARD: "FORWARD", STEP_HOVER_SEARCH: "HOVER_SEARCH",
    STEP_TRACK: "TRACK", STEP_POSITION_REFINE: "POSITION_REFINE",
    STEP_PRE_SERVO: "PRE_SERVO", STEP_VISUAL_SERVO: "VISUAL_SERVO",
    STEP_DROP: "DROP", STEP_CLIMB: "CLIMB", STEP_SCOUT: "SCOUT",
    STEP_RETURN: "RETURN", STEP_LAND: "LAND",
}


def camera_pixel_to_level(cx, cy, pitch, roll):
    """下视像素射线 -> 水平航向坐标中的 (前向/H, 右向/H)。

    固定安装假设：图像上=机头，图像右=机体右，相机光轴=机体下。
    PX4 FRD -> NED 使用 Rz(yaw) Ry(pitch) Rx(roll)。这里去除
    roll/pitch，yaw 由 _body_to_ned 处理。视觉误差和反投影共用本模型。
    """
    if not all(math.isfinite(v) for v in (cx, cy, pitch, roll)):
        return None
    fwd = (CY - cy) / FY
    right = (cx - CX) / FX
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rolled_right = cr * right - sr
    rolled_down = sr * right + cr
    level_fwd = cp * fwd + sp * rolled_down
    level_down = -sp * fwd + cp * rolled_down
    if level_down <= 1e-6:  # 射线平行于地面或指向天空
        return None
    return level_fwd / level_down, rolled_right / level_down


def level_to_camera_pixel(fwd, right, height, pitch, roll):
    """camera_pixel_to_level 的逆变换，返回像素 (cx, cy) 或 None。"""
    if not all(math.isfinite(v) for v in (fwd, right, height, pitch, roll)) or height <= 0.0:
        return None
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    body_fwd = cp * fwd - sp * height
    unpitched_down = sp * fwd + cp * height
    body_right = cr * right + sr * unpitched_down
    body_down = -sr * right + cr * unpitched_down
    if body_down <= 1e-6:
        return None
    return CX + FX * body_right / body_down, CY - FY * body_fwd / body_down

def low_alt_velocity(error_n, error_e, velocity_n, velocity_e):
    """NED 米制误差 -> 限幅速度；用测得速度提前制动，避免对噪声求导。"""
    if not all(math.isfinite(v) for v in (error_n, error_e, velocity_n, velocity_e)):
        return 0.0, 0.0
    distance = math.hypot(error_n, error_e)
    limit = LOW_SERVO_MAX_SPEED * distance / (distance + LOW_SERVO_SLOW_RADIUS_M)
    vn = LOW_SERVO_KP * (error_n - velocity_n * LOW_SERVO_BRAKE_LOOKAHEAD)
    ve = LOW_SERVO_KP * (error_e - velocity_e * LOW_SERVO_BRAKE_LOOKAHEAD)
    norm = math.hypot(vn, ve)
    if norm > limit:
        vn *= limit / norm
        ve *= limit / norm
    return vn, ve


class FR3Mission(Node):
    def __init__(self):
        super().__init__('f_r3_mission')

        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.manual_pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', 10)
        self.aux5_pwm_pub = self.create_publisher(Float32, '/drop/aux5_pwm', 10)
        self.impact_pub = self.create_publisher(Point, '/drop/impact_point', 10)

        self.status_sub = self.create_subscription(VehicleStatus, STATUS_TOPIC, self._cb_status, QOS_BE)
        self.local_sub = self.create_subscription(VehicleLocalPosition, LOCAL_POS_TOPIC, self._cb_local, QOS_LATEST)
        self.land_sub = self.create_subscription(VehicleLandDetected, LAND_DETECTED_TOPIC, self._cb_land, QOS_BE)
        self.detect_sub = self.create_subscription(String, '/bucket/detections', self._cb_detect, 1)
        self.control_mode_sub = self.create_subscription(VehicleControlMode, CONTROL_MODE_TOPIC, self._cb_control_mode, QOS_BE)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._cb_attitude, QOS_LATEST)

        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self._takeoff_x = self._takeoff_y = self._takeoff_z = None
        self.heading = None
        self._heading0 = None
        self._stabilize_yaw = None
        self.landed = False
        self.status = None
        self._control_mode = None

        # ---- 多桶追踪 ----
        self.latest_roi = None
        self._last_detector_frame_seq = None
        self._detector_frame_has_seq = False
        self._detector_frame_seq_warned = False
        self._last_matched_detect_time = None   # 2026-09-20: DROP 段"匹配检测"接收时刻
        self._last_detect_time = self.get_clock().now()
        self._last_roi_key = None
        self._roi_dup_count = 0
        self._target_buckets = []        # 选定的目标桶: [(ned_x, ned_y), ...]
        # 2026-09-20 晚 (A+C):
        self._served_targets = []        # A: 已投过的桶位置 (home 相对 NED), 后续匹配一律排除
        self._target_ids = []            # C: 被选目标的追踪器 id (= 桶身份, 不随追踪器列表增删变化)
        self._next_tracker_id = 0        # C: 追踪器 id 发号器
        self._last_refresh_log = 0.0     # C: 刷新日志限流
        self._bucket_idx = 0             # 当前处理第几个目标
        self._current_target_ned = None  # 当前伺服的桶的估计 NED 位置
        self._bucket_ests = []           # [(ned_x, ned_y, roi, d, cx, cy), ...]
        self._prev_bucket_ests = None     # 上一帧 NED 列表, 用于帧间稳定性检查
        self._trackers = []               # v9: 每桶独立追踪器 [{ned, first_seen, last_seen, confirmed, roi_center}]

        self._target_img_idx = -1  # unused; kept for compatibility with _enter_step

        self._pitch = 0.0
        self._roll = 0.0

        self.step = STEP_TAKEOFF
        self._step_start_time = None
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
        self._drop_hold_target = None           # 2026-09-20: 释放后固定的设定点
        self._drop_count = 0  # 第几次投放 (0=B1, 1=B2)
        self._last_drop_err = None
        self._reset_low_alt_servo()
        self._drop_good_frames = 0
        self._drop_good_since = None
        self._drop_good_rx = None
        self._drop_release_reason = None

        # VISUAL_SERVO 纯视觉伺服状态 (v5)
        self._servo_active = False
        self._servo_target_x = 0.0
        self._servo_target_y = 0.0
        self._servo_converge_start = None

        self._land_dwell_start = None
        self._land_cmd_sent = False
        self._land_cmd_time = None

        # ===== 2026-09-12 e: 全量日志/复盘状态 (自 d 移植) =====
        self.vx = self.vy = self.vz = 0.0     # NED 速度 (供日志)
        self._mission_start_time = None       # TAKEOFF 首次进入时刻
        self.done = False                     # 2026-09-12 e: _shutdown() 置位, main() 据此退出 spin 循环
        self._drop_records = []               # 每次投放: idx/aim/pos/H/last_err
        self._drop_skips = []                 # 质量门控拒绝投放的桶及原因
        self._scout_done = False              # SCOUT 7 子阶段是否完整走完
        self._scout_elapsed = None            # SCOUT 阶段总耗时
        self._return_stab_info = None         # (判稳耗时, 漂移)
        self._land_pos = None                 # 着陆时 NED 水平位置
        self._search_failed = False           # 搜索无目标转侦察
        self._mission_timeout_forced = False  # 全局超时强制着陆
        self._desc_log_t = 0.0                # PRE_SERVO/VISUAL_SERVO 下降速率日志节流
        self._sel_info = None                 # 选桶决策文本 (写入 MISSION SUMMARY)

        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._last_arm_attempt = -10.0
        self._init_wait_start = None
        self._initial_wait_done = False

        self._track_max_time = TRACK_MAX_TIME
        self._hover_max_time = HOVER_MAX_TIME

        self._fwd_exit_frames = 0    # FORWARD 过冲确认帧计数 (GPS 噪声防护)
        self._fwd_arrive_frames = 0  # FORWARD 到达确认帧计数

        # SCOUT 状态
        self._scout_hover_start = None
        self._scout_sub = 0
        self._scout_sub_start = None

        # RETURN 稳定悬停 + LAND 限速 (2026-08-15)
        self._return_sub = 0
        self._return_sub_start = None
        self._stab_pos_hist = collections.deque(
            maxlen=max(1, int(RETURN_STAB_WINDOW / 0.05)))
        self._land_entry_z = None

        self._print_ctr = 0
        self._last_pid_print = 0.0
        self._last_vel_x = 0.0
        self._last_vel_y = 0.0
        self._last_vel_z = 0.0
        self._start_wall = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop)
        self.manual_timer = self.create_timer(MANUAL_TIMER_PERIOD, self._publish_manual_control)
        self.get_logger().info(
            f"F Mission ready. HEADING={HEADING_DEG:.0f}deg "
            f"outlet_right_B1={PAYLOAD_OFFSET_BODY_Y_B1:+.4f}m outlet_right_B2={PAYLOAD_OFFSET_BODY_Y_B2:+.4f}m "
            f"FORWARD_DIST={FORWARD_DIST}m SCOUT_DIST={SCOUT_DIST}m "
            f"cruise={CRUISE_HEIGHT}m drop={DROP_HEIGHT}m (no flight boundary)")
        try:
            source_path = Path(__file__).resolve()
            source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
        except OSError as exc:
            source_path, source_hash = __file__, f"unavailable:{exc}"
        self.get_logger().info(
            f"Build revision={MISSION_REVISION} source={source_path} sha256={source_hash} "
            f"drop_gate={DROP_ARRIVE_M:.3f}m speed<={DROP_RELEASE_SPEED_MPS:.2f}m/s "
            f"frames={DROP_CONFIRM_FRAMES} force={DROP_FORCE_AFTER:.0f}s "
            f"xy_target_rate_max={LOW_SERVO_MAX_SPEED:.2f}m/s brake_T={LOW_SERVO_BRAKE_LOOKAHEAD:.2f}s "
            f"fx={FX:.1f} fy={FY:.1f} (assumed; verify actual see crop) "
            f"detect_age_max={DROP_DETECT_MAX_AGE:.2f}s "
            "camera=down_fixed(top=forward,right=right) geometry=FRD-ray-ground")

    # ========== 回调 ==========
    def _cb_status(self, m): self.status = m
    def _cb_control_mode(self, m): self._control_mode = m
    def _cb_land(self, m): self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        # 2026-09-12 e: 记录 NED 速度供 1Hz 状态行 (判断飞机实际在怎么动)
        if math.isfinite(m.vx): self.vx = m.vx
        if math.isfinite(m.vy): self.vy = m.vy
        if math.isfinite(m.vz): self.vz = m.vz
        if math.isfinite(m.heading):
            self.heading = m.heading
        # 记录起飞参考点 (航向用硬编码 HEADING_DEG, 不再锁机头)
        # 实机无 xy_valid (室内无 GPS), 有 heading 即可记录参考点
        if self._takeoff_x is None and self.heading is not None:
            self._takeoff_x, self._takeoff_y, self._takeoff_z = self.x, self.y, self.z
            self.get_logger().info(
                f"Takeoff ref: NED({self._takeoff_x:.2f},{self._takeoff_y:.2f},{self._takeoff_z:.2f}) "
                f"HEADING_DEG={HEADING_DEG:.0f}deg (hardcoded)")

    def _cb_attitude(self, m):
        w, x, y_, z_ = float(m.q[0]), float(m.q[1]), float(m.q[2]), float(m.q[3])
        self._roll  = math.atan2(2.0*(w*x + y_*z_), 1.0 - 2.0*(x*x + y_*y_))
        self._pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y_ - z_*x))))

    def _cb_detect(self, msg):
        """接收 JSON 多桶检测.
        HOVER_SEARCH (静止): ROI→NED 估算桶坐标 → 锁定.
        TRACK/DESCEND (移动): 已知坐标→期望像素投影, 像素空间最近邻匹配.
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        frame_seq = data.get("frame_seq")
        if frame_seq is not None:
            self._detector_frame_has_seq = True
            if frame_seq == self._last_detector_frame_seq:
                return  # see 重发的缓存结果不算新图像; 不刷新检测年龄或对准帧计数
            self._last_detector_frame_seq = frame_seq
        else:
            self._detector_frame_has_seq = False
            if not self._detector_frame_seq_warned:
                self._detector_frame_seq_warned = True
                self.get_logger().warn("Detector has no frame_seq; precision servo/release disabled, timed release remains enabled")
        detections = data.get("buckets", [])

        valid = []
        for d in detections:
            roi = RegionOfInterest()
            roi.x_offset = int(d["x"]); roi.y_offset = int(d["y"])
            roi.width = int(d["w"]); roi.height = int(d["h"])
            roi.do_rectify = False
            est = self._roi_to_ned(roi)
            if est is not None:
                cx = roi.x_offset + roi.width / 2.0
                cy = roi.y_offset + roi.height / 2.0
                valid.append((est[0], est[1], roi, d, cx, cy))

        # 2026-09-12: 同帧重复框合并 (防同一个桶被喂出两个追踪器 → 两个目标其实是同一个桶)
        valid = self._dedup_dets(valid)
        # 2026-09-20: 剔除"残缺框"(部分出画/贴边) —— 其中心不是桶的投影中心, 反投影可偏 0.2~3m,
        #   偏出 1.5m 就会自建追踪器 → 候选池多出"真桶附近的幽灵" → 可能占满前两名.
        if len(valid) >= 2:
            sides = sorted(min(r.width, r.height) for _x, _y, r, _d, _cx, _cy in valid)
            _thr = max(8.0, 0.5 * sides[len(sides) // 2])
        else:
            _thr = 8.0
        _kept = [v for v in valid if min(v[2].width, v[2].height) >= _thr]
        if len(_kept) != len(valid):
            self.get_logger().info(
                f"  DET-FILTER 剔除残缺框 {len(valid) - len(_kept)}/{len(valid)} "
                f"(阈值{_thr:.0f}px, 本帧最小边="
                f"{[min(v[2].width, v[2].height) for v in valid]})")
        valid = _kept
        self._bucket_ests = valid
        self._update_trackers(valid)  # v9: 每帧更新追踪器
        if not valid:
            return

        if self._current_target_ned is not None:
            # TRACK/DESCEND/DROP: 几何身份锁 — 找 NED 估计最接近目标坐标的 ROI
            # (不用 nearest-center, 防止多桶时追错目标)
            best_roi = None; best_d = None
            best_geo_dist = float('inf')
            GEO_IDENTITY_MAX = 1.5  # 几何身份门限 (m), v6: 2.5->1.5 < 最小桶间距 2.2m
            n_skip_served = 0
            for bx, by, roi, d, cx, cy in valid:
                if self._is_served(bx, by):
                    n_skip_served += 1
                    continue                                  # A: 已投过的桶不再作为瞄准候选
                geo_dist = math.hypot(bx - self._current_target_ned[0],
                                      by - self._current_target_ned[1])
                if geo_dist < best_geo_dist:
                    best_geo_dist = geo_dist
                    best_roi = roi; best_d = d
            if best_roi is not None and best_geo_dist < GEO_IDENTITY_MAX:
                roi_key = (best_d["x"], best_d["y"], best_d["w"], best_d["h"])
                self._update_fresh(roi_key)
                self.latest_roi = best_roi
                self._last_matched_detect_time = self.get_clock().now()
            # 找不到匹配: 不更新 latest_roi, 触发丢检保护
        else:
            # HOVER_SEARCH: 取离图像中心最近的检测, 确保选到正下方的桶
            best = min(valid, key=lambda v: math.hypot(v[4] - CX, v[5] - CY))
            _, _, roi, d, _, _ = best
            roi_key = (d["x"], d["y"], d["w"], d["h"])
            self._update_fresh(roi_key)
            self.latest_roi = roi
            self._last_matched_detect_time = self.get_clock().now()

    def _update_fresh(self, roi_key):
        """更新检测新鲜度 (内联去重逻辑)."""
        if roi_key == self._last_roi_key:
            self._roi_dup_count += 1
            # 视觉伺服时帧间移动 <1px 很常见, 提高阈值防误判
            if self._roi_dup_count < 25:
                self._last_detect_time = self.get_clock().now()
        else:
            self._roi_dup_count = 0
            self._last_roi_key = roi_key
            self._last_detect_time = self.get_clock().now()

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

    # ========== 多桶追踪器 (v9) ==========
    def _dedup_dets(self, ests):
        """2026-09-12: 同帧内重复检测框合并.

        动机 (d 2026-09-12 仿真): HOVER_SEARCH 期间 3 个桶检测出 5 个框 (DETx5), 多出的框
        经 _roi_to_ned 得到不同 NED → 追踪器把同一个桶记成两个 tracker, 各自通过 1.5s 确认
        → 选桶选出两个"影子桶" (实测两目标间距 1.78m, 而场上最小真实桶间距 2.24m)
        → 两次投弹落在同一个桶 → 有效得分 10/20. 实机 YOLO 低置信度重复框同理.

        判据: 同帧内两框的 NED 估计相距 < DET_DUP_MERGE_DIST 即视为同一个桶, 保留检测框较大者.
        安全性: 同一帧/同一机位下, 同一个桶的两个框系统性偏差完全相同 (差异仅几 cm~0.3m);
        而最小真实桶间距 2.24m 即使各带 ±0.4m 估计噪声仍有 1.44m > 0.8m, 不会误合并真实桶.
        """
        if len(ests) <= 1:
            return ests
        kept = []
        for e in sorted(ests, key=lambda v: -(v[2].width * v[2].height)):
            if all(math.hypot(e[0] - k[0], e[1] - k[1]) >= DET_DUP_MERGE_DIST for k in kept):
                kept.append(e)
        n_drop = len(ests) - len(kept)
        if n_drop != getattr(self, '_dedup_last_n', 0):
            self._dedup_last_n = n_drop
            self.get_logger().info(
                f"  同帧重复框合并: {len(ests)} -> {len(kept)} "
                f"(相距 < {DET_DUP_MERGE_DIST}m 视为同一个桶)")
        return kept

    # ------------------------------------------------------------------
    # 2026-09-20 晚 (A+C): 桶身份保护 —— 已投过的不再瞄, 瞄准点用活追踪器刷新
    # ------------------------------------------------------------------
    def _is_served(self, nx, ny):
        """A: 该位置是否已经投过弹 (0.6m 内视为同一个桶)."""
        for sx, sy in self._served_targets:
            if math.hypot(nx - sx, ny - sy) < SERVED_EXCLUDE_M:
                return True
        return False

    def _refresh_target_from_tracker(self):
        """C: 用"活追踪器"刷新当前目标的瞄准点; 返回是否真的改了.

        身份 = 选桶时记下的 tracker id. 原因见脚本头: 选桶瞬间的单点估计在运动中偏 ~1m,
        照它飞会停在两桶中间, 再被 refine 的最近邻锁到已投过的桶上.
        """
        if self._current_target_ned is None or self._bucket_idx >= len(self._target_ids):
            return False
        tid = self._target_ids[self._bucket_idx]
        ox, oy = self._current_target_ned

        if tid is None or tid < 0:
            return False                     # 未绑定身份 → 不刷 (DROP 段本来每帧反投影重建)
        cand = None
        for t in self._trackers:
            if t.get("id") == tid and t.get("confirmed"):
                cand = t
                break
        if cand is None:
            # id 已失效 —— 低空投弹期间该桶丢检, 追踪器被 TRACKER_TIMEOUT 裁剪后**重建会换 id**.
            # 这里**刻意不猜身份** (从冻结的旧估计猜"最近的桶"在现场 1.66m 间距下有认错风险),
            # 等 POSITION_REFINE 把瞄准点修正准确后由 _rebind_target_id() 重新认领.
            return False
        nx, ny = cand["ned"]
        if self._is_served(nx, ny):
            return False                                   # 该追踪器就是已投过的桶 → 拒绝刷新
        delta = math.hypot(nx - ox, ny - oy)
        if delta > TARGET_REFRESH_MAX_M:
            return False                                   # 差太多 → 追踪器身份可疑, 保持原目标点
        self._current_target_ned = (nx, ny)
        if delta > 0.10:
            t_now = (self.get_clock().now() - self._start_wall).nanoseconds * 1e-9
            if t_now - self._last_refresh_log > 2.0:
                self._last_refresh_log = t_now
                self.get_logger().info(
                    f"  TARGET-REFRESH id=#{cand.get('id')} NED({ox:.2f},{oy:.2f}) -> "
                    f"NED({nx:.2f},{ny:.2f}) delta={delta:.2f}m")
        return True

    def _rebind_target_id(self):
        """C: 用"刚被 refine 修正过的准确瞄准点"重新认领目标的追踪器 id.

        追踪器在低空投弹期间会因丢检被裁剪, CLIMB 重建后 id 已变 → 这里重新认领.
        因为锚点已被 refine 修正准确 (选桶瞬间 ~1m 的偏差已消掉), 用 0.6m 紧门限最近邻
        认领是安全的 (现场桶间距 1.66m >> 0.6m), 并要求与第二名拉开 0.3m (无歧义),
        同时排除已投过的桶.
        """
        if self._current_target_ned is None or self._bucket_idx >= len(self._target_ids):
            return False
        ox, oy = self._current_target_ned
        best = None; best_d = float('inf'); second_d = float('inf')
        for t in self._trackers:
            if not t.get("confirmed") or self._is_served(t["ned"][0], t["ned"][1]):
                continue
            d = math.hypot(t["ned"][0] - ox, t["ned"][1] - oy)
            if d < best_d:
                second_d, best_d, best = best_d, d, t
            elif d < second_d:
                second_d = d
        if best is None or best_d > TARGET_REBIND_MAX_M:
            return False
        if second_d - best_d < TARGET_REBIND_MARGIN:
            return False                     # 有两个候选贴在瞄准点附近 → 不认领
        old = self._target_ids[self._bucket_idx]
        if best.get("id") == old:
            return False
        self._target_ids[self._bucket_idx] = best.get("id", -1)
        self.get_logger().info(
            f"  TARGET-REBIND id #{old} -> #{best.get('id')} aim=NED({ox:.2f},{oy:.2f}) "
            f"tracker=NED({best['ned'][0]:.2f},{best['ned'][1]:.2f}) d={best_d:.2f}m")
        return True

    def _update_trackers(self, bucket_ests):
        """v9 多桶追踪器: 每帧 NED 最近邻匹配, 每桶独立时间确认 ≥1.5s.

        每个物理桶对应一个追踪器 {ned, first_seen, last_seen, confirmed, roi_center}.
        匹配: 当前帧各检测的 NED 估计与已有追踪器做最近邻匹配 (<1.5m),
              匹配成功 → 指数平滑更新位置 (alpha=0.3), 刷新时间, 检查确认.
              未匹配 → 为新检测创建新追踪器.
        清理: 超时 >2s 未更新且本轮未匹配 → 丢弃.

        防御 YOLO 瞬时误检: 单个检测必须持续 ≥1.5s 才被确认(confirmed=True),
        瞬时的背景干扰检测不会存活足够久.
        """
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        matched = set()

        for bx, by, roi, d, cx, cy in bucket_ests:
            best_t, best_d = None, float('inf')
            for t in self._trackers:
                dist = math.hypot(bx - t["ned"][0], by - t["ned"][1])
                if dist < best_d:
                    best_d = dist
                    best_t = t
            if best_t is not None and best_d < TRACKER_NED_MATCH:
                if best_d > 0.8:   # 2026-09-20 诊断: 距离偏大的匹配
                    self.get_logger().info(
                        f"  TRK-PULL #{self._trackers.index(best_t)} dist={best_d:.2f}m "
                        f"box(cx={cx:.0f},cy={cy:.0f}) NED({bx:.2f},{by:.2f}) "
                        f"← tracker NED({best_t['ned'][0]:.2f},{best_t['ned'][1]:.2f})")
                alpha = 0.3
                old_x, old_y = best_t["ned"]
                best_t["ned"] = (old_x + alpha * (bx - old_x),
                                 old_y + alpha * (by - old_y))
                best_t["last_seen"] = now_s
                best_t["roi_center"] = (cx, cy)
                best_t["area"] = float(roi.width * roi.height)
                if not best_t["confirmed"] and (now_s - best_t["first_seen"]) >= TRACKER_CONFIRM_TIME:
                    best_t["confirmed"] = True
                    self.get_logger().info(
                        f"Tracker confirmed: NED({best_t['ned'][0]:.1f},{best_t['ned'][1]:.1f}) "
                        f"after {now_s - best_t['first_seen']:.1f}s")
                matched.add(id(best_t))
            else:
                self._trackers.append({
                    "id": self._next_tracker_id,          # C: 稳定 id (= 桶身份)
                    "ned": (bx, by),
                    "first_seen": now_s,
                    "last_seen": now_s,
                    "confirmed": False,
                    "roi_center": (cx, cy),
                    "area": float(roi.width * roi.height),
                })
                self._next_tracker_id += 1
                # 2026-09-20 诊断: 新追踪器 + 来源框 (找重复计数来源)
                self.get_logger().info(
                    f"  TRK-NEW #{len(self._trackers) - 1} NED({bx:.2f},{by:.2f}) "
                    f"from box(cx={cx:.0f},cy={cy:.0f},w={roi.width:.0f},h={roi.height:.0f}) "
                    f"alive={len(self._trackers)}")

        self._trackers = [t for t in self._trackers
                          if (now_s - t["last_seen"] < TRACKER_TIMEOUT)
                          or id(t) in matched]
        # 2026-09-20 诊断: 1Hz 打印本帧全部检测框 + 全部存活追踪器
        if now_s - getattr(self, '_trk_dbg_t', 0.0) > 1.0:
            self._trk_dbg_t = now_s
            box_txt = " ".join(
                f"({cx:.0f},{cy:.0f})w{w:.0f}→NED({bx:.2f},{by:.2f})"
                for bx, by, _roi, _d, cx, cy, w in
                [(bx, by, roi, d, cx, cy, roi.width) for bx, by, roi, d, cx, cy in bucket_ests])
            trk_txt = " ".join(
                f"#{i}{'C' if t['confirmed'] else '-'}NED({t['ned'][0]:.2f},{t['ned'][1]:.2f})"
                f"age{now_s - t['first_seen']:.1f}s"
                for i, t in enumerate(self._trackers))
            self.get_logger().info(
                f"  TRK-DBG step={STEP_NAME.get(self.step, self.step)} nbox={len(bucket_ests)} "
                f"boxes=[{box_txt}] trackers={len(self._trackers)}[{trk_txt}]")

    def _pub_low_alt_position(self, vn, ve, z, yaw):
        """Move a persistent XY position target slowly, with bounded lead and early braking."""
        now_ns = self.get_clock().now().nanoseconds
        dt = min(0.1, max(0.0, (now_ns-self._low_publish_ns)*1e-9)) \
            if self._low_publish_ns is not None else 0.05
        self._low_publish_ns = now_ns
        if self._low_target_xy is None:
            self._low_target_xy = (self.x, self.y)
        if self._low_braking:
            # Capture once on entry; keep that point even if the aircraft drifts.
            self._low_cmd = (0.0, 0.0)
            if (self._low_raw_error is None
                    or not self._drop_detection_fresh(LOW_SERVO_DETECT_MAX_AGE)):
                self._low_brake_stable_since = None
            elif self._low_rx != self._low_brake_last_rx:
                rx = self._low_rx
                if (self._low_brake_last_rx is None
                        or (rx-self._low_brake_last_rx)*1e-9 > LOW_SERVO_DETECT_MAX_AGE):
                    self._low_brake_stable_since = None
                self._low_brake_last_rx = rx
                en = self._low_raw_error[0] - (self.x-self._low_observation_xy[0])
                ee = self._low_raw_error[1] - (self.y-self._low_observation_xy[1])
                distance = math.hypot(en, ee)
                closing_speed = max(0.0, (en*self.vx+ee*self.vy)/distance) if distance > 0 else 0.0
                resume = (math.hypot(self.vx, self.vy) <= LOW_SERVO_BRAKE_RESUME_SPEED
                          and distance-closing_speed*LOW_SERVO_BRAKE_LOOKAHEAD
                          >= LOW_SERVO_BRAKE_RESUME_MARGIN)
                if not resume:
                    self._low_brake_stable_since = None
                elif self._low_brake_stable_since is None:
                    self._low_brake_stable_since = rx
                elif (rx-self._low_brake_stable_since)*1e-9 >= LOW_SERVO_BRAKE_RESUME_SPAN:
                    self._low_braking = False
                    self._low_brake_stable_since = None
                    self.get_logger().info("LOW-BRAKE resume: fresh frames confirm low speed and margin")
            # Resume movement on the next tick from this same target, with slew limiting.
            self._pub_pos(*self._low_target_xy, z, yaw)
            return
        if self._low_raw_error is None:
            # Keep the last position target through a camera gap; re-anchoring to
            # the drifting vehicle on every gap discards the position correction.
            self._low_loss_hold = True
        else:
            self._low_loss_hold = False
            en = self._low_raw_error[0] - (self.x-self._low_observation_xy[0])
            ee = self._low_raw_error[1] - (self.y-self._low_observation_xy[1])
            # Stop moving the target before inertia carries the outlet through center.
            closing = en*self.vx + ee*self.vy
            old_lead_dot_error = ((self._low_target_xy[0]-self.x)*en
                                  + (self._low_target_xy[1]-self.y)*ee)
            brake = (math.hypot(self.vx, self.vy) >= LOW_SERVO_BRAKE_ENTER_SPEED
                     and (old_lead_dot_error < 0.0 or (closing > 0.0 and
                          en*en + ee*ee <= closing*LOW_SERVO_BRAKE_LOOKAHEAD)))
            if brake:
                self._low_target_xy = (self.x, self.y)
                self._low_cmd = (0.0, 0.0)
                self._low_braking = True
                self._low_brake_last_rx = self._low_rx
                self._low_brake_stable_since = None
                self.get_logger().info(
                    f"LOW-BRAKE hold NED({self.x:.3f},{self.y:.3f}) "
                    f"vxy={math.hypot(self.vx,self.vy):.3f}m/s")
            else:
                tx = self._low_target_xy[0] + vn*dt
                ty = self._low_target_xy[1] + ve*dt
                dx, dy = tx-self.x, ty-self.y
                lead = math.hypot(dx, dy)
                if lead > LOW_SERVO_MAX_LEAD_M:
                    scale = LOW_SERVO_MAX_LEAD_M / lead
                    tx, ty = self.x+dx*scale, self.y+dy*scale
                self._low_target_xy = (tx, ty)
        self._pub_pos(*self._low_target_xy, z, yaw)

    def _reset_low_alt_servo(self):
        self._low_rx = None
        self._low_raw_error = None
        self._low_filtered_error = None
        self._low_observation_xy = None
        self._low_cmd = (0.0, 0.0)
        self._low_control_ns = None
        self._low_log_ns = None
        self._low_publish_ns = None
        self._low_target_xy = None
        self._low_loss_hold = False
        self._low_bucket_idx = self._bucket_idx
        self._low_braking = False
        self._low_brake_last_rx = None
        self._low_brake_stable_since = None

    def _update_low_alt_servo(self):
        """Consume each matched frame once; predict between frames from measured motion."""
        now_ns = self.get_clock().now().nanoseconds
        dt = min(0.1, max(0.0, (now_ns - self._low_control_ns) * 1e-9)) \
            if self._low_control_ns is not None else 0.05
        self._low_control_ns = now_ns
        if not self._drop_detection_fresh(LOW_SERVO_DETECT_MAX_AGE):
            self._low_rx = None
            self._low_raw_error = self._low_filtered_error = None
            self._low_observation_xy = None
            self._low_cmd = (0.0, 0.0)
            self._last_drop_err = None
            return False
        rx_ns = self._last_matched_detect_time.nanoseconds
        new_frame = rx_ns != self._low_rx
        if new_frame:
            roi = self.latest_roi
            h = self.home_z - self.z
            ray = camera_pixel_to_level(roi.x_offset + roi.width / 2.0,
                                        roi.y_offset + roi.height / 2.0,
                                        self._pitch, self._roll)
            if ray is None or not math.isfinite(h) or h < 0.3:
                self._low_raw_error = self._low_filtered_error = None
                self._low_cmd = (0.0, 0.0)
                self._last_drop_err = None
                return False
            ox, oy = self._outlet_offsets()
            fwd, right = ray[0] * h - ox, ray[1] * h - oy
            en, ee, _ = self._body_to_ned(fwd, right)
            if not all(math.isfinite(v) for v in (en, ee, self.x, self.y, self.vx, self.vy)):
                self._low_raw_error = self._low_filtered_error = None
                self._low_cmd = (0.0, 0.0)
                return False
            # Filter level-ground error, not pixels from different camera attitudes.
            if self._low_filtered_error is None or self._low_rx is None:
                filtered = (en, ee)
            else:
                frame_dt = max(0.0, (rx_ns - self._low_rx) * 1e-9)
                alpha = 1.0 - math.exp(-frame_dt / LOW_SERVO_FILTER_TAU)
                old_n = self._low_filtered_error[0] - (self.x - self._low_observation_xy[0])
                old_e = self._low_filtered_error[1] - (self.y - self._low_observation_xy[1])
                filtered = (old_n + alpha * (en - old_n), old_e + alpha * (ee - old_e))
            self._low_rx = rx_ns
            self._low_raw_error = (en, ee)
            self._low_filtered_error = filtered
            self._low_observation_xy = (self.x, self.y)
            ex, ey = right * FX / h, fwd * FY / h
            self._last_drop_err = (math.hypot(ex, ey), ex, ey, h)
        if self._low_filtered_error is None:
            self._low_cmd = (0.0, 0.0)
            return False
        en = self._low_filtered_error[0] - (self.x - self._low_observation_xy[0])
        ee = self._low_filtered_error[1] - (self.y - self._low_observation_xy[1])
        desired = (0.0, 0.0) if self._low_braking else low_alt_velocity(en, ee, self.vx, self.vy)
        dn, de = desired[0] - self._low_cmd[0], desired[1] - self._low_cmd[1]
        delta = math.hypot(dn, de)
        max_delta = LOW_SERVO_MAX_ACCEL * dt
        scale = min(1.0, max_delta / delta) if delta > 0.0 else 1.0
        self._low_cmd = (self._low_cmd[0] + scale * dn, self._low_cmd[1] + scale * de)
        # Explicitly log command and measurement; no fabricated impact coordinate.
        if self._low_log_ns is None or now_ns - self._low_log_ns >= 1_000_000_000:
            self._low_log_ns = now_ns
            self.get_logger().info(
                f"LOW-SERVO raw={math.hypot(*self._low_raw_error):.3f}m "
                f"filtered={math.hypot(en, ee):.3f}m "
                f"pred={math.hypot(en-self.vx*LOW_SERVO_BRAKE_LOOKAHEAD, ee-self.vy*LOW_SERVO_BRAKE_LOOKAHEAD):.3f}m "
                f"target_rate=({self._low_cmd[0]:+.3f},{self._low_cmd[1]:+.3f})m/s "
                f"vxy={math.hypot(self.vx,self.vy):.3f}m/s age={self._drop_detect_age():.3f}s "
                f"braking={int(self._low_braking)}")
        return new_frame

    def _trigger_drop(self, reason):
        """Exactly one release pulse per DROP; timeout never depends on vision validity."""
        if self._drop_triggered:
            return
        self._drop_triggered = True
        self._drop_signal_start = self.get_clock().now()
        self._drop_release_reason = reason
        self._low_cmd = (0.0, 0.0)
        # Freeze at current vehicle position: no last noisy target jump while releasing.
        self._drop_hold_target = (self.x, self.y, self.home_z - DROP_HEIGHT, self.heading)
        outlet = self._drop_count + 1
        self._aux5_value = AUX5_DROP if self._drop_count == 0 else AUX5_DROP2
        self._drop_count += 1
        aim = (self.home_x + self._current_target_ned[0],
               self.home_y + self._current_target_ned[1]) if self._current_target_ned else None
        if self._current_target_ned is not None:
            self._served_targets.append(self._current_target_ned)
        self._drop_records.append(dict(idx=self._bucket_idx + 1, outlet=outlet, aim=aim,
            pos=(self.x, self.y), H=self.home_z-self.z, last_err=self._last_drop_err,
            reason=reason))
        self.impact_pub.publish(Point(x=float(self.x), y=float(self.y), z=float(self.home_z)))
        self._pub_pos(*self._drop_hold_target)
        self._publish_manual_control()  # send now; existing 200Hz timer holds the pulse
        self.get_logger().info(
            f">>> DROP B{self._bucket_idx+1} outlet={outlet}! "
            f"PWM={1500+self._aux5_value*600:.0f}us reason={reason} "
            f"H={self.home_z-self.z:.2f}m vxy={math.hypot(self.vx,self.vy):.3f}m/s "
            f"raw={math.hypot(*self._low_raw_error) if self._low_raw_error else float('nan'):.3f}m "
            f"event=release_command actual_impact=unmeasured")

    def _run_drop(self):
        if self._drop_triggered:
            self._pub_pos(*self._drop_hold_target)
            held = (self.get_clock().now() - self._drop_signal_start).nanoseconds * 1e-9
            if held >= AUX5_DROP_HOLD:
                self._aux5_value = AUX5_LOCK
                self._publish_manual_control()
                self.get_logger().info(f"DROP pulse complete after {held:.2f}s")
                self._enter_step(STEP_LAND if self._mission_timeout_forced else STEP_CLIMB)
            return
        # These exits run before every vision/quality check, including missing frame_seq.
        if self._mission_timeout_forced or self._step_elapsed() >= DROP_FORCE_AFTER:
            self._trigger_drop('mission_timeout' if self._mission_timeout_forced else 'timeout_force')
            return
        new_frame = self._update_low_alt_servo()
        self._pub_low_alt_position(*self._low_cmd, self.home_z-DROP_HEIGHT, self.heading)
        if self._low_raw_error is None or not self._drop_detection_fresh():
            self._drop_good_frames = 0
            self._drop_good_since = self._drop_good_rx = None
            return
        if not new_frame:
            return
        rx = self._low_rx * 1e-9
        good = (math.hypot(*self._low_raw_error) <= DROP_ARRIVE_M
                and math.hypot(self.vx, self.vy) <= DROP_RELEASE_SPEED_MPS
                and abs(self.vz) <= DROP_RELEASE_VZ_MPS
                and abs(self.home_z-self.z-DROP_HEIGHT) <= DROP_HEIGHT_TOL_M)
        if not good:
            self._drop_good_frames = 0
            self._drop_good_since = self._drop_good_rx = None
            return
        if self._drop_good_rx is None or rx - self._drop_good_rx > DROP_CONFIRM_MAX_GAP:
            self._drop_good_frames = 0
            self._drop_good_since = rx
        self._drop_good_frames += 1
        self._drop_good_rx = rx
        if (self._drop_good_frames >= DROP_CONFIRM_FRAMES
                and rx - self._drop_good_since >= DROP_CONFIRM_MIN_SPAN):
            self._trigger_drop('aligned_fresh_frames')

    def _enter_step(self, step):
        # 2026-09-12 e: 记录前一阶段与耗时, 供下方阶段切换日志使用 (必须先于 self.step 赋值)
        prev_step = self.step
        prev_elapsed = self._step_elapsed()
        if prev_step == STEP_DROP and step != STEP_DROP:
            self._aux5_value = AUX5_LOCK   # 2026-09-20: 离开 DROP 必复位释放输出
        self.step = step; self._step_start_time = self.get_clock().now()
        self._stabilize_start = None
        self._detect_loss_logged = False; self._detect_loss_start = None
        self._confirm_start_time = None; self._stabilize_lost_frames = 0
        self._drop_triggered = False; self._drop_signal_start = None
        self._drop_hold_target = None
        if step == STEP_POSITION_REFINE:
            self._refine_ests = []
            self._roi_dup_count = 0; self._last_roi_key = None
            self._last_detect_time = self.get_clock().now()
        if step in (STEP_VISUAL_SERVO, STEP_DROP):
            self._servo_active = True
            self._servo_target_x = self.x - self.home_x
            self._servo_target_y = self.y - self.home_y
            same_bucket_drop = (prev_step == STEP_VISUAL_SERVO and step == STEP_DROP
                                and self._low_bucket_idx == self._bucket_idx)
            if not same_bucket_drop:
                self._reset_low_alt_servo()
        if step == STEP_DROP:
            self._last_drop_err = None
            self._drop_good_frames = 0
            self._drop_good_since = None
            self._drop_good_rx = None
            self._drop_release_reason = None
        if step == STEP_FORWARD:
            self._fwd_exit_frames = 0
            self._fwd_arrive_frames = 0
        if step == STEP_STABILIZE: self._stabilize_yaw = None
        if step == STEP_HOVER_SEARCH:
            self._search_good_frames = 0
            self._trackers = []  # v9: 新搜索, 清空旧追踪器
        if step == STEP_RETURN:
            self._current_target_ned = None
            self._target_img_idx = -1
            self._target_buckets = []
            self._return_sub = 0
            self._return_sub_start = None
            self._stab_pos_hist.clear()
        if step == STEP_LAND:
            self._land_entry_z = None
        if step == STEP_CLIMB:
            self._climb_done = False
        if step == STEP_SCOUT:
            self._scout_hover_start = None
            self._scout_sub = 0
            self._scout_sub_start = None
        if step == STEP_TAKEOFF and self._mission_start_time is None:
            self._mission_start_time = self.get_clock().now()  # 任务计时起点 (2026-09-12 e)
        # 2026-09-12 e: 阶段切换日志 (自 d) — 每阶段进入位置/高度 + 前一阶段耗时
        if self.x is not None and self.home_z is not None:
            if prev_step == step:
                self.get_logger().info(f"STEP {STEP_NAME[step]} re-entered")
            else:
                self.get_logger().info(
                    f"STEP {STEP_NAME[prev_step]}->{STEP_NAME[step]} "
                    f"(prev {prev_elapsed:.1f}s) NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"H={self.home_z-self.z:.2f}m")

    def _step_elapsed(self):
        if self._step_start_time is None: return 0.0
        return (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9

    def _detect_fresh(self):
        if self.latest_roi is None: return False
        return (self.get_clock().now() - self._last_detect_time).nanoseconds * 1e-9 < DETECT_TIMEOUT

    def _px_raw(self):
        """当前目标 ROI 的**原始**像素偏差 (未做姿态补偿/死区), 供 1Hz 状态行复盘.

        返回 (ex, ey, err): ex>0 = 桶在图像右侧, ey>0 = 桶在机头前方 (TOP=前).
        与 PID 内部 ex/ey 的差别仅在于此函数不做 tan(roll)/tan(pitch) 补偿、不置死区,
        因此它代表"相机实际看到的偏差", 而 PID 输出的是补偿后的控制量.
        """
        roi = self.latest_roi
        if roi is None or roi.width <= 0 or roi.height <= 0: return None
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        ex = cx - CX
        ey = CY - cy
        return (ex, ey, math.hypot(ex, ey))

    def _px_raw_s(self):
        """_px_raw() 的定宽字符串形式, 供日志行内嵌 (无检测 → '--')."""
        px = self._px_raw()
        if px is None: return "--"
        return f"({px[0]:+.0f},{px[1]:+.0f},err={px[2]:.0f}px)"

    def _drop_detect_age(self):
        if self.latest_roi is None or self._last_matched_detect_time is None:
            return float('inf')
        return (self.get_clock().now() - self._last_matched_detect_time).nanoseconds * 1e-9

    def _drop_detection_fresh(self, max_age=DROP_DETECT_MAX_AGE):
        return (self._detector_frame_has_seq
                and 0.0 <= self._drop_detect_age() <= max_age)

    def _outlet_offsets(self):
        """下一次实际放水所用出水口的机体前/右坐标 (m)."""
        if self._drop_count == 0:
            return PAYLOAD_OFFSET_BODY_X_B1, PAYLOAD_OFFSET_BODY_Y_B1
        return PAYLOAD_OFFSET_BODY_X_B2, PAYLOAD_OFFSET_BODY_Y_B2

    # ========== 几何工具 ==========
    def _roi_to_ned(self, roi):
        """像素 → 相对home的NED偏移 (非PX4绝对坐标).

        与 PID 共用 camera_pixel_to_level；H 是起飞参考平面相对高度。
        前/右距离经航向旋转到 NED；无有效地面交点时返回 None。
        """
        if self.heading is None or self.home_z is None or self.home_x is None: return None
        H = self.home_z - self.z
        if not math.isfinite(H) or H < 0.3: return None
        cx = roi.x_offset + roi.width / 2.0; cy = roi.y_offset + roi.height / 2.0
        projected = camera_pixel_to_level(cx, cy, self._pitch, self._roll)
        if projected is None: return None
        cam_fwd, cam_right = (v * H for v in projected)
        dx, dy, _ = self._body_to_ned(cam_fwd, cam_right)
        return (self.x - self.home_x + dx, self.y - self.home_y + dy)

    def _body_to_ned(self, v_fwd, v_lat, v_down=0.0):
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        return (v_fwd * cos_h - v_lat * sin_h,
                v_fwd * sin_h + v_lat * cos_h, v_down)

    def _ned_to_body(self, dx, dy):
        """NED 世界偏移 → 机体坐标系 (fwd, right)."""
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        fwd = dx * cos_h + dy * sin_h
        lat = -dx * sin_h + dy * cos_h
        return fwd, lat

    def _ned_to_expected_pixel(self, target_ned):
        """已知桶 NED → 期望像素坐标 (用于 TRACK 阶段身份锁定)."""
        if self.heading is None or self.home_z is None or self.home_x is None:
            return None
        H = self.home_z - self.z
        if H < 0.3:
            return None
        dx = (self.home_x + target_ned[0]) - self.x
        dy = (self.home_y + target_ned[1]) - self.y
        cam_fwd, cam_right = self._ned_to_body(dx, dy)
        return level_to_camera_pixel(cam_fwd, cam_right, H, self._pitch, self._roll)

    def _finalize_refine(self):
        """平均收集到的 ROI→NED 估计, 更新 _current_target_ned.
        如果修正量 > POSITION_REFINE_MAX_CORRECTION 则拒绝 (防止身份混淆).
        """
        ests = getattr(self, '_refine_ests', [])
        if len(ests) >= POSITION_REFINE_MIN_SAMPLES:
            avg_x = sum(e[0] for e in ests) / len(ests)
            avg_y = sum(e[1] for e in ests) / len(ests)
            old_x, old_y = self._current_target_ned
            correction = math.hypot(avg_x - old_x, avg_y - old_y)
            if correction > POSITION_REFINE_MAX_CORRECTION:
                self.get_logger().warn(
                    f"Position refine REJECTED: correction {correction:.2f}m > "
                    f"{POSITION_REFINE_MAX_CORRECTION}m (identity confusion)")
            else:
                self._current_target_ned = (avg_x, avg_y)
                self.get_logger().info(
                    f"Position refined: ({old_x:.2f},{old_y:.2f}) -> ({avg_x:.2f},{avg_y:.2f}) "
                    f"delta=({avg_x-old_x:.3f},{avg_y-old_y:.3f})m n={len(ests)}")
                self._rebind_target_id()          # C: 瞄准点已准确 → 重新认领该桶的追踪器 id
        else:
            self.get_logger().info(
                f"Position refine skipped: only {len(ests)} samples (need {POSITION_REFINE_MIN_SAMPLES})")
        self._refine_ests = []

    def _bucket_target(self):
        """FORWARD 目标点, 相对 home 的 NED 偏移."""
        if self._heading0 is None: return (0.0, 0.0)
        cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
        return (FORWARD_DIST * cos_h0,
                FORWARD_DIST * sin_h0)

    def _scout_target(self):
        """侦察区中心, 相对 home 的 NED 偏移."""
        if self._heading0 is None: return (0.0, 0.0)
        cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
        return (SCOUT_DIST * cos_h0,
                SCOUT_DIST * sin_h0)

    def _scout_edge_target(self):
        """侦察区第一根线(近边)中点, 相对 home 的 NED 偏移. 2026-08-15: 直飞目标改近边防过冲."""
        if self._heading0 is None: return (0.0, 0.0)
        cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
        return (SCOUT_EDGE_DIST * cos_h0,
                SCOUT_EDGE_DIST * sin_h0)

    def _fly_toward(self, tx, ty, speed, vz=0.0):
        """速度控制飞向绝对NED坐标 (tx, ty)。
        tx, ty 是绝对 NED 坐标 (不是相对 home 偏移)。
        调用方: SCOUT 传 home_x+sx, RETURN 传 home_x, home_y.
        """
        dist = math.hypot(tx - self.x, ty - self.y)
        if dist > 0.1:
            # 距离越近速度越低, dist<3m 时线性 ramp-down, 最低 FORWARD_MIN_SPEED
            ratio = max(FORWARD_MIN_SPEED / CRUISE_SPEED, min(1.0, dist / 3.0))
            vx = (tx - self.x) / dist * speed * ratio
            vy = (ty - self.y) / dist * speed * ratio
        else: vx = vy = 0.0
        return vx, vy, dist

    # 2026-09-12 e 按用户指示移除 _enforce_boundaries (软墙机制), 自 d 移植:
    # 实机 d 试飞 — 目标桶侧向 4.21m 超出 ±4m 软墙, 软墙每 1-2s 接管并跳过状态机,
    # 伺服 / 收敛判定 / 投弹计时全部停摆 → err 74→218px 发散 → 任务中止 (未投弹).

    def _handle_detect_loss(self, target_z):
        if not self._detect_loss_logged:
            self._detect_loss_logged = True
            self.get_logger().warn(f"Detection lost in {STEP_NAME[self.step]}, hovering...")
        self._pub_vel(0.0, 0.0, (target_z - self.z) * TRACK_VZ_KP, self.heading)

    def _shutdown(self, reason="unknown"):
        self.timer.cancel()
        self.manual_timer.cancel()
        self._print_summary(reason)
        self.get_logger().info("Mission complete, shutting down")
        self.destroy_node()
        # 2026-09-12 e: 只置退出标志, 由 main() 退出 spin 循环后再 destroy/shutdown.
        # **不能**在回调内直接 rclpy.shutdown(): Executor.shutdown() 会 _work_tracker.wait(None)
        # 等"正在执行的回调"结束 —— 即等自己 → 自死锁, 进程挂住 (c 继承的遗留问题)
        self.done = True

    def _print_summary(self, reason):
        """任务总结报告 (2026-09-12 e, 自 d 移植): 结束原因/选桶/投弹明细/侦察/返航/着陆/总时长.

        目的: 飞行结束后只看这一段就能拿到任务级结论; 逐秒过程看 1Hz 状态行与
        PRE-SERVO / SERVO / DROP-SERVO 明细行.
        """
        L = self.get_logger().info
        L("=" * 72)
        L("MISSION SUMMARY (project f)")
        L(f"  End reason: {reason}")
        if self._mission_start_time is not None:
            total = (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9
            L(f"  Total time: {total:.1f}s")
        if self.home_x is not None:
            L(f"  Home: NED({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
              f"HEADING_DEG={HEADING_DEG:.0f}deg")
        if getattr(self, '_sel_info', None):
            L(f"  Selection: {self._sel_info}")
        L(f"  Buckets dropped: {len(self._drop_records)}/{getattr(self, '_target_count', 0)}")
        for skip in self._drop_skips:
            L(f"  Drop skipped B{skip['idx']}: {skip['reason']}")
        for r in self._drop_records:
            aim_str = f"NED({r['aim'][0]:.2f},{r['aim'][1]:.2f})" if r["aim"] else "unknown"
            pos = r["pos"]
            dev = math.hypot(pos[0] - r["aim"][0], pos[1] - r["aim"][1]) if r["aim"] else float("nan")
            err_str = ""
            if r["last_err"] is not None:
                ep, exl, eyl, hh = r["last_err"]
                err_str = f" last_err={ep:.0f}px(ex={exl:.0f},ey={eyl:.0f}@H={hh:.2f}m)"
            L(f"  Drop B{r['idx']} via outlet {r['outlet']}: aim {aim_str} at NED({pos[0]:.3f},{pos[1]:.3f}) "
              f"H={r['H']:.2f}m vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured reason={r.get('reason', 'unknown')}{err_str}")
        if self._scout_done and self._scout_elapsed is not None:
            L(f"  Scout: completed ({self._scout_elapsed:.1f}s, 7 sub-phases)")
        elif self.step == STEP_SCOUT:
            L(f"  Scout: incomplete (aborted at sub={self._scout_sub})")
        else:
            L("  Scout: not reached")
        if self._return_stab_info is not None:
            stab_t, drift = self._return_stab_info
            L(f"  Return: stabilized in {stab_t:.1f}s (drift {drift:.2f}m)")
        else:
            L("  Return: no stabilization (forced land)")
        if self._land_pos is not None and self.home_x is not None:
            dist = math.hypot(self._land_pos[0] - self.home_x, self._land_pos[1] - self.home_y)
            L(f"  Landing: NED({self._land_pos[0]:.2f},{self._land_pos[1]:.2f}) dist_from_home={dist:.2f}m")
        else:
            L("  Landing: not reached")
        if self._search_failed:
            L("  NOTE: no valid targets in HOVER_SEARCH, skipped drops")
        if self._mission_timeout_forced:
            L("  NOTE: mission timeout forced landing")
        L("=" * 72)

    # ========== ManualControlSetpoint ==========
    def _publish_manual_control(self):
        msg = ManualControlSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.valid = True; msg.data_source = ManualControlSetpoint.SOURCE_RC
        msg.roll = msg.pitch = msg.yaw = 0.0; msg.throttle = -1.0; msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = 0.0
        msg.aux5 = float(self._aux5_value); msg.aux6 = 0.0
        msg.sticks_moving = True
        # px4_msgs 不同版本的 ManualControlSetpoint 有的没有 buttons 字段
        if hasattr(msg, 'buttons'):
            msg.buttons = 0
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
            # ===== 2026-09-12 e: 1Hz 全量状态行 (固定字段顺序, 便于人读 + 脚本解析复盘) =====
            # 字段: t=任务计时  TAG  阶段耗时  NED  H(离地高)  hdg/pit/rol  vNED(实际速度)  det/age(检测数/新鲜度)  阶段附加信息
            tag = STEP_NAME.get(self.step, "???")
            hdg = math.degrees(self.heading) if self.heading else 0.0
            H = (self.home_z - self.z) if self.home_z is not None else float("nan")
            H_str = f"{H:5.2f}" if self.home_z is not None else "   --"  # home 未锁定时 H 无意义, 不打印 nan
            t_mission = sim_t
            if self._mission_start_time is not None:
                t_mission = (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9
            det_age = (self.get_clock().now() - self._last_detect_time).nanoseconds * 1e-9 \
                if self._last_detect_time is not None else -1.0
            has_det = f"det={len(self._bucket_ests)}" if self._bucket_ests else "det=0"
            extra = ""
            if self.step in (STEP_TRACK, STEP_VISUAL_SERVO, STEP_DROP) and self._bucket_idx < len(self._target_buckets):
                extra = f" B{self._bucket_idx+1}/{len(self._target_buckets)}"
            if self.step == STEP_VISUAL_SERVO and self._servo_active:
                extra += " SERVO"
            if self.step == STEP_POSITION_REFINE:
                n = len(getattr(self, '_refine_ests', []))
                extra = f" refine_samples={n} elapsed={self._step_elapsed():.1f}s"
            if self.step == STEP_STABILIZE:
                extra = f" stabilizing {self._step_elapsed():.1f}s/{STABILIZE_HOVER_TIME}s"
            elif self.step == STEP_FORWARD and self.home_x is not None:
                tx, ty = self._bucket_target()
                extra = f" dist_to_target={math.hypot((self.x-self.home_x)-tx,(self.y-self.home_y)-ty):.1f}m"
            elif self.step == STEP_HOVER_SEARCH:
                conf = sum(1 for t in self._trackers if t["confirmed"])
                extra = f" search {self._step_elapsed():.1f}s confirmed={conf}/{len(self._trackers)}"
            elif self.step == STEP_SCOUT:
                extra = f" scout sub={self._scout_sub}"
            elif self.step == STEP_RETURN and self.home_x is not None:
                dist_h = math.hypot(self.x-self.home_x, self.y-self.home_y)
                if self._return_sub == 0:
                    extra = f" ->home dist={dist_h:.1f}m"
                else:
                    drift = 0.0
                    if self._stab_pos_hist:
                        xs = [p[0] for p in self._stab_pos_hist]; ys = [p[1] for p in self._stab_pos_hist]
                        drift = max(max(xs)-min(xs), max(ys)-min(ys))
                    extra = f" stabilizing dist={dist_h:.2f}m drift={drift:.2f}m"
            # 伺服装 (TRACK/PRE_SERVO/VISUAL_SERVO/DROP): 附当前瞄准点 + 原始像素偏差
            if self.step in (STEP_TRACK, STEP_PRE_SERVO, STEP_VISUAL_SERVO, STEP_DROP) and self._current_target_ned:
                tx, ty = self._current_target_ned
                extra += f" tgt=NED({tx:.2f},{ty:.2f})"
                px = self._px_raw()
                if px is not None:
                    ex, ey, err = px
                    extra += f" px=({ex:+.0f},{ey:+.0f})err={err:.0f}px"
            self.get_logger().info(
                f"[t={t_mission:7.1f} {tag:<15s} ph={self._step_elapsed():5.1f}s] "
                f"NED({self.x:7.2f},{self.y:7.2f},{self.z:6.2f}) H={H_str} "
                f"hdg={hdg:6.1f} pit={math.degrees(self._pitch):+5.1f} rol={math.degrees(self._roll):+5.1f} "
                f"vNED=({self.vx:+.2f},{self.vy:+.2f},{self.vz:+.2f}) "
                f"{has_det} age={det_age:4.2f}{extra}")
            self._print_ctr = 0

        # 初始等待 (DDS 就绪)
        if not self._initial_wait_done:
            if self._init_wait_start is None: self._init_wait_start = self.get_clock().now()
            self._pub_vel(0.0, 0.0, 0.0, self.heading)
            elapsed = (self.get_clock().now() - self._init_wait_start).nanoseconds * 1e-9
            dds_ready = self._control_mode is not None
            if dds_ready and elapsed > 3.0: self._initial_wait_done = True
            elif elapsed > 30.0: self._shutdown("dds_timeout"); return
            return

        # ARM / OFFBOARD (实机: VehicleControlMode 单源, 不用 VehicleStatus fallback)
        cm = self._control_mode
        armed = cm.flag_armed if cm is not None else False
        offboard = cm.flag_control_offboard_enabled if cm is not None else False
        if cm is None: self._pub_vel(0, 0, 0, self.heading); return
        if self._mode_req_start is None: self._mode_req_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._mode_req_start).nanoseconds * 1e-9
        ref_x, ref_y = self._takeoff_x, self._takeoff_y
        ref_z = self._takeoff_z - CRUISE_HEIGHT
        yaw_live = self.heading if self.heading else 0.0
        if self.step == STEP_LAND and self._land_cmd_sent: pass
        elif armed and offboard:
            # 2026-09-12 e: 解锁+OFFBOARD 链路日志 (自 d) — 确认模式切换实际耗时
            if not getattr(self, '_active_logged', False):
                self._active_logged = True
                self.get_logger().info(f"ARM + OFFBOARD active (t={elapsed:.1f}s), entering mission")
        elif elapsed > 30.0: self.get_logger().error("Mode timeout"); self._shutdown("mode_timeout"); return
        elif not armed:
            if not self._arm_req or elapsed - self._last_arm_attempt > 1.0:
                if not self._arm_req:
                    self.get_logger().info(f"Requesting ARM... (t={elapsed:.1f}s)")
                self._arm_req = True; self._last_arm_attempt = elapsed; send_arm(self.cmd_pub, self)
            elif self._arm_req and not getattr(self, '_arm_ok_logged', False):
                self._arm_ok_logged = True
                self.get_logger().info(f"ARM request sent, waiting for armed flag... (t={elapsed:.1f}s)")
            self._pub_pos(ref_x, ref_y, self._takeoff_z, yaw_live); return
        elif not offboard:
            if not self._mode_req:
                self._mode_req = True; self._send_cmd(176, param1=1.0, param2=6.0)
                self.get_logger().info(f"ARM OK, requesting OFFBOARD mode... (t={elapsed:.1f}s)")
            self._pub_pos(ref_x, ref_y, self._takeoff_z, yaw_live); return

        # ============================================================
        # 状态机
        # ============================================================
        cruise_z = self._takeoff_z - CRUISE_HEIGHT
        yaw_live = self.heading if self.heading is not None else 0.0
        yaw_locked = self._heading0 if self._heading0 is not None else yaw_live

        # 2026-09-12 e: 飞行边界已完全移除 (原 _enforce_boundaries 调用点 → 软墙接管会导致伺服停摆)

        # 已进入 DROP 时先完成本桶舵机脉冲，再响应全局超时着陆。
        if not self._land_cmd_sent and not self._mission_timeout_forced and sim_t > MISSION_TIMEOUT:
            self.get_logger().error(f"Mission timeout ({MISSION_TIMEOUT}s)")
            self._mission_timeout_forced = True
            if self.step != STEP_DROP:
                self._enter_step(STEP_LAND)

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
                # 使用硬编码航向, home 用起飞点地面高度
                self._heading0 = math.radians(HEADING_DEG)
                self.home_x, self.home_y = self._takeoff_x, self._takeoff_y
                self.home_z = self._takeoff_z
                bx, by = self._bucket_target()
                self.get_logger().info(
                    f"Home locked: NED({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                    f"heading0={HEADING_DEG:.0f}deg (hardcoded) target NED({bx:.1f},{by:.1f})")
                self._enter_step(STEP_FORWARD)

        # ---- FORWARD: 沿 heading0 盲飞 32m, 全程忽略视觉检测, 保证轨迹笔直 ----
        elif self.step == STEP_FORWARD:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_HOVER_SEARCH); return
            tx, ty = self._bucket_target()
            cos_h0, sin_h0 = math.cos(self._heading0), math.sin(self._heading0)
            # 一维标量投影判断飞行进度 (区分方向: 正=接近, 负=飞过头)
            travel = (self.x - self.home_x) * cos_h0 + (self.y - self.home_y) * sin_h0
            remain = FORWARD_DIST - travel
            # 过冲检测: 飞过目标 0.5m 以上, 连续 N 帧确认 (防 GPS 跳变)
            if remain < -POS_ARRIVE_DIST:
                self._fwd_exit_frames += 1
                if self._fwd_exit_frames >= FORWARD_EXIT_CONFIRM_FRAMES:
                    self.get_logger().info(f"Overshot target travel={travel:.2f}m > {FORWARD_DIST}m, entering HOVER_SEARCH")
                    self._enter_step(STEP_HOVER_SEARCH); return
            else:
                self._fwd_exit_frames = 0
            # 非对称 ramp-down: effective_dist = max(0, remain), 飞过头不加速 (防止越飞越远)
            effective_dist = max(0.0, remain)
            ratio = max(FORWARD_MIN_SPEED / CRUISE_SPEED, min(1.0, effective_dist / FORWARD_RAMP_DIST))
            speed = CRUISE_SPEED * ratio
            vx = speed * cos_h0; vy = speed * sin_h0
            self._pub_vel(vx, vy, (cruise_z - self.z) * TRACK_VZ_KP, self._heading0)
            # 二维欧氏距离兜底 — 连续 N 帧确认
            dist2d = math.hypot((self.x - self.home_x) - tx, (self.y - self.home_y) - ty)
            if dist2d < POS_ARRIVE_DIST:
                self._fwd_arrive_frames += 1
                if self._fwd_arrive_frames >= FORWARD_EXIT_CONFIRM_FRAMES:
                    self.get_logger().info(f"Arrived at target, dist={dist2d:.2f}m, entering HOVER_SEARCH")
                    self._enter_step(STEP_HOVER_SEARCH)
            else:
                self._fwd_arrive_frames = 0

        # ---- HOVER_SEARCH: 目标点悬停, 搜索全部桶, 选最近2个 ----
        elif self.step == STEP_HOVER_SEARCH:
            if self._step_elapsed() > HOVER_SEARCH_TIMEOUT:
                self._search_failed = True
                self.get_logger().info("No valid targets, going to scout zone")
                self._enter_step(STEP_SCOUT); return
            tx, ty = self._bucket_target()
            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, yaw_locked)

            # 驻留等 MPC 刹车, 期间跳过门控
            if self._step_elapsed() < HOVER_SEARCH_SETTLE_TIME:
                return

            # 零检测超时: 连续 N 秒一个桶都没看到 → 判定此处无目标, 转侦察区
            if not self._detect_fresh() or not self._bucket_ests:
                if not hasattr(self, '_zero_detect_start'): self._zero_detect_start = self.get_clock().now()
                if hasattr(self, '_zero_detect_start') and self._zero_detect_start is not None:
                    zero_elapsed = (self.get_clock().now() - self._zero_detect_start).nanoseconds * 1e-9
                    if zero_elapsed > HOVER_SEARCH_ZERO_DETECT_TIMEOUT:
                        self._search_failed = True
                        self.get_logger().info(
                            f"No detection for {zero_elapsed:.1f}s > {HOVER_SEARCH_ZERO_DETECT_TIMEOUT}s, going to scout zone")
                        self._enter_step(STEP_SCOUT); return
            else:
                self._zero_detect_start = None

            # 从已确认的追踪器中选检测框面积最大的 2 个桶
            confirmed = [t for t in self._trackers if t["confirmed"]]
            if len(confirmed) < 2:
                return  # 确认不足 2 个, 继续等待

            # 2026-09-12 e 选桶策略改版 (自 d): "检测框面积降序" → "优先选离航向中线最近的桶".
            # 动机 (实机 d_mission_20260911_170203.log): 面积最大 ≠ 最合适 —— 该轮按面积选中
            # 侧向 lateral -4.21m 的桶当第一目标, 几乎正前方的 -0.42m 桶反被丢弃, 既贴飞行边界
            # 又落进 85° 广角镜头像场边缘 (定位偏差最大) 的区域.
            if self._heading0 is not None:
                s_h0, c_h0 = math.sin(self._heading0), math.cos(self._heading0)
                # 主键 |lateral| 升序 (正值=heading0 右侧); 次键 面积降序 (两侧向接近时仍优先更近/更大的桶)
                confirmed.sort(key=lambda t: (
                    abs(-t["ned"][0] * s_h0 + t["ned"][1] * c_h0),
                    -t.get("area", 0)))
            else:
                confirmed.sort(key=lambda t: t.get("area", 0), reverse=True)
            # 2026-09-20: 两个目标必须由画面里"两个不同的框"支撑 (防真桶+残框幽灵占满两名).
            #   追踪器记有 roi_center(最近一次匹配框中心); 两框中心相距 <25px 视为同一个框.
            picked = []
            for _t in confirmed:
                if all(math.hypot(_t["roi_center"][0] - _p["roi_center"][0],
                                  _t["roi_center"][1] - _p["roi_center"][1]) >= 25.0
                       for _p in picked):
                    picked.append(_t)
                if len(picked) == 2:
                    break
            if len(picked) < 2:
                # 没有两个"由不同框支撑"的已确认目标: 继续悬停搜索, 不用同一桶的两个候选占位
                self.get_logger().info(
                    f"  SELECT-HOLD 已确认 {len(confirmed)} 个但只有 {len(picked)} 个由不同框支撑, 继续搜索")
                return
            selected = [(t["ned"][0], t["ned"][1]) for t in picked]
            self._target_buckets = selected
            self._target_ids = [t.get("id", -1) for t in picked]   # C: 记下桶身份 (追踪器 id)
            self._target_count = len(selected)      # 2026-09-12 e: MISSION SUMMARY 分母
            self._bucket_idx = 0
            self._current_target_ned = self._target_buckets[self._bucket_idx]
            self._target_img_idx = -1
            lateral_txt = ", ".join(
                f"lat={-t['ned'][0]*math.sin(self._heading0)+t['ned'][1]*math.cos(self._heading0):+.2f}m"
                for t in picked) if self._heading0 is not None else ""
            self.get_logger().info(
                f"Selected {len(selected)} buckets: "
                + ", ".join(f"NED({bx:.1f},{by:.1f})"
                          for bx, by in selected)
                + (f"  [{lateral_txt}]" if lateral_txt else ""))
            # 2026-09-12 e: 记录选桶依据 (全部已确认桶的 lateral + 面积), 复盘时判断选桶是否合理
            if self._heading0 is not None:
                all_txt = ", ".join(
                    f"NED({t['ned'][0]:.2f},{t['ned'][1]:.2f})"
                    f"[lat={-t['ned'][0]*math.sin(self._heading0)+t['ned'][1]*math.cos(self._heading0):+.2f}m"
                    f",area={t.get('area', 0):.0f}]"
                    for t in confirmed)
                self.get_logger().info(f"  Selection basis (all {len(confirmed)} confirmed, |lateral| asc): {all_txt}")
                self._sel_info = (f"{len(selected)}/{len(confirmed)} chosen: "
                                  + ", ".join(f"NED({bx:.2f},{by:.2f})" for bx, by in selected)
                                  + f"  [{lateral_txt}]")
            self._enter_step(STEP_TRACK)

        # ---- TRACK: 位置控制飞向已知桶坐标 + 到位后视觉确认 ----
        elif self.step == STEP_TRACK:
            self._refresh_target_from_tracker()            # C: 逼近途中持续用活追踪器刷瞄准点
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_RETURN); return
            tx, ty = self._current_target_ned
            dist_to_target = math.hypot((self.x - self.home_x) - tx, (self.y - self.home_y) - ty)
            # 全程位置控制, 让 PX4 自己处理导航和刹车
            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, self.heading)
            # 到位判断: 接近目标即可 (视觉确认在到位后)
            if dist_to_target < POS_ARRIVE_DIST:
                self._stabilize_lost_frames = 0
                if self._stabilize_start is None: self._stabilize_start = self.get_clock().now()
                elif (self.get_clock().now()-self._stabilize_start).nanoseconds > STABILIZE_TIME*1e9: self._enter_step(STEP_POSITION_REFINE)
            elif self._step_elapsed() > self._track_max_time: self._enter_step(STEP_POSITION_REFINE)
            else:
                self._stabilize_lost_frames += 1
                if self._stabilize_lost_frames > STABILIZE_LOST_FRAMES_MAX: self._stabilize_start = None; self._stabilize_lost_frames = 0

        # ---- POSITION_REFINE: 悬停稳定后重新估算桶坐标 ----
        elif self.step == STEP_POSITION_REFINE:
            if self._step_elapsed() > 5.0:
                self._finalize_refine()
                self._enter_step(STEP_PRE_SERVO); return

            self._refresh_target_from_tracker()            # C: 用活追踪器刷新后再做候选选择
            tx, ty = self._current_target_ned
            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, self.heading)

            if not hasattr(self, '_refine_ests'):
                self._refine_ests = []

            # 几何身份锁: 从当前帧检测中找 NED 估计最接近原始目标的那一个
            if self._bucket_ests:
                best_est = None; best_dist = float('inf')
                for bx, by, roi, d, cx, cy in self._bucket_ests:
                    if self._is_served(bx, by):
                        continue                                  # A: 已投过的桶不参与 refine
                    geo_dist = math.hypot(bx - tx, by - ty)
                    if geo_dist < best_dist:
                        best_dist = geo_dist; best_est = (bx, by)
                if best_est is not None and best_dist < POSITION_REFINE_IDENTITY_MAX:
                    self._refine_ests.append(best_est)
                # 每秒打印候选信息
                if not hasattr(self, '_last_refine_log'):
                    self._last_refine_log = 0.0
                if sim_t - self._last_refine_log > 1.0:
                    self._last_refine_log = sim_t
                    cands = ", ".join(f"({bx:.1f},{by:.1f})d={math.hypot(bx-tx,by-ty):.1f}"
                                     for bx, by, *_ in self._bucket_ests)
                    self.get_logger().info(
                        f"  REFINE candidates vs target({tx:.1f},{ty:.1f}): {cands} "
                        f"best_dist={best_dist:.1f} n={len(self._refine_ests)}")

            if self._step_elapsed() >= POSITION_REFINE_DURATION:
                self._finalize_refine()
                self._enter_step(STEP_PRE_SERVO)

        # ---- PRE_SERVO: 下降到低空提升 pinhole 精度, 再切 VISUAL_SERVO (v7.1) ----
        elif self.step == STEP_PRE_SERVO:
            self._refresh_target_from_tracker()            # C: 下降前再刷一次瞄准点
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_VISUAL_SERVO); return
            pre_z = self.home_z - PRE_SERVO_HEIGHT
            # 匀速下降到目标高度, 位置保持在当前目标
            target_z = max(pre_z, self.z - SERVO_DESCENT_SPEED * 0.05)
            tx, ty = self._current_target_ned
            self._pub_pos(self.home_x + tx, self.home_y + ty, target_z, self.heading)
            # 2026-09-12 e: PRE_SERVO 下降速率观测 (e 保留 c 的"快速俯冲"行为, 必须可复盘)
            # 每秒一行: 目标高度 / 当前高度 / 瞬时下降率, 用于判断是否"目标瞬间跳变→全速俯冲"
            if sim_t - self._desc_log_t > 1.0:
                self._desc_log_t = sim_t
                down_rate = self.vz  # NED z 向下为正 → vz>0 即下降; 正数=下降速率
                self.get_logger().info(
                    f"  PRE-SERVO H={self.home_z-self.z:5.2f}m target_z-GND={pre_z-self.z:+.2f}m "
                    f"cmd_z={target_z:+.2f} 下降率={down_rate:.2f}m/s "
                    f"tgt=NED({tx:.2f},{ty:.2f}) 横向偏差={math.hypot((self.x-self.home_x)-tx,(self.y-self.home_y)-ty):.2f}m")
            if self.z > pre_z - 0.3:
                # 低空了, 用最新 ROI 更新桶坐标 (pinhole 更准), 切 VISUAL_SERVO
                if self._detect_fresh() and self.latest_roi is not None and self._bucket_ests:
                    # 几何身份锁: 找离当前目标最近的检测, 更新 NED 估计
                    best_est = None; best_dist = float('inf')
                    for bx, by, roi, d, cx, cy in self._bucket_ests:
                        geo_dist = math.hypot(bx - tx, by - ty)
                        if geo_dist < best_dist:
                            best_dist = geo_dist; best_est = (bx, by)
                    if best_est is not None and best_dist < 2.0:
                        self._current_target_ned = best_est
                        self.get_logger().info(
                            f"PRE_SERVO update target: ({best_est[0]:.2f},{best_est[1]:.2f}) "
                            f"H={self.home_z-self.z:.1f}m")
                self._enter_step(STEP_VISUAL_SERVO)

        # ---- VISUAL_SERVO: 低空 XY 位置目标限速，沿用基底的下降高度命令 ----
        elif self.step == STEP_VISUAL_SERVO:
            drop_z = self.home_z - DROP_HEIGHT
            target_z = max(drop_z, self.z - SERVO_DESCENT_SPEED * 0.05)
            self._update_low_alt_servo()
            self._pub_low_alt_position(*self._low_cmd, target_z, self.heading)
            if self.z > drop_z - 0.2 or self._step_elapsed() > VISUAL_SERVO_TIMEOUT:
                self._enter_step(STEP_DROP)

        # ---- DROP: 新帧对准即投；超时先释放再离开 ----
        elif self.step == STEP_DROP:
            self._run_drop()

        # ---- CLIMB: 投弹后爬升 → 下一桶或侦察区 ----
        elif self.step == STEP_CLIMB:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_RETURN); return
            # 阶段1: 竖直爬升到巡航高度
            if not hasattr(self, '_climb_done'): self._climb_done = False
            if not self._climb_done:
                vz = CLIMB_VZ + (cruise_z-self.z)*TRACK_VZ_KP
                self._pub_vel(0, 0, vz, yaw_locked)
                if abs(self.z-cruise_z) < 0.5:
                    self._climb_done = True
                    self._bucket_idx += 1
                return
            # 阶段2: 直接进 TRACK 飞向下一个桶 (用最初锁定的坐标, 不重搜)
            self._climb_done = False
            if self._bucket_idx < len(self._target_buckets):
                self._current_target_ned = self._target_buckets[self._bucket_idx]
                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)
                self._target_img_idx = -1
                self.get_logger().info(
                    f"Next bucket {self._bucket_idx+1}/{len(self._target_buckets)}: "
                    f"NED({self._current_target_ned[0]:.1f},{self._current_target_ned[1]:.1f})")
                self._enter_step(STEP_TRACK)
            else:
                self.get_logger().info("All buckets done, flying to SCOUT zone")
                self._enter_step(STEP_SCOUT)

        # ---- SCOUT (2026-08-15 重写): 直飞近边 → 降1m → 10s缓移至预定点 → 左1.5m → 停5s → 右3m → 停5s → 返航 ----
        elif self.step == STEP_SCOUT:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_RETURN); return
            sx, sy = self._scout_target()       # 预定点 (侦察区正上方)
            ex, ey = self._scout_edge_target()   # 第一根线(近边)中点
            if self._scout_sub == 0:
                # 子阶段0: 速度控制直飞近边 (不再直冲预定点, 防过冲出侦察区)
                dist_to_edge = math.hypot((self.x - self.home_x) - ex, (self.y - self.home_y) - ey)
                if dist_to_edge > POS_ARRIVE_DIST:
                    vx, vy, _ = self._fly_toward(self.home_x + ex, self.home_y + ey, CRUISE_SPEED)
                    self._pub_vel(vx, vy, (cruise_z - self.z) * TRACK_VZ_KP, yaw_locked)
                    return
                self._scout_sub = 1
                self._scout_sub_start = self.get_clock().now()
                self.get_logger().info("Scout: arrived at zone near edge, descending 1m first")
                return
            sub_elapsed = (self.get_clock().now() - self._scout_sub_start).nanoseconds * 1e-9
            # 机体左/右向单位向量 (基于锁定航向)
            left_x, left_y = math.sin(yaw_locked), -math.cos(yaw_locked)
            scout_z = cruise_z + SCOUT_DESCENT_DELTA
            if self._scout_sub == 1:
                # 子阶段1: 先下降 1m (5m→4m)
                self._pub_pos(self.home_x + ex, self.home_y + ey, scout_z, yaw_locked)
                if sub_elapsed > SCOUT_SUB_TIMEOUT:
                    self.get_logger().warn("Scout descend timeout, starting slow move")
                    self._scout_sub = 2; self._scout_sub_start = self.get_clock().now(); return
                if abs(self.z - scout_z) < SCOUT_SUB_ARRIVE:
                    self._scout_sub = 2; self._scout_sub_start = self.get_clock().now()
                    self.get_logger().info(
                        f"Scout: descended 1m, slow moving {SCOUT_SLOW_MOVE_TIME:.0f}s to zone center")
                return
            if self._scout_sub == 2:
                # 子阶段2: 10s 缓慢移动到预定点 (时间锚定线性插值, 0.45m/s 防过冲)
                t = min(1.0, sub_elapsed / SCOUT_SLOW_MOVE_TIME)
                tx = ex + (sx - ex) * t
                ty = ey + (sy - ey) * t
                self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)
                if sub_elapsed >= SCOUT_SLOW_MOVE_TIME:
                    self._scout_sub = 3; self._scout_sub_start = self.get_clock().now()
                    self.get_logger().info("Scout: slow move done, moving left 1.5m")
                return
            if self._scout_sub == 3:
                # 子阶段3: 机体左移 1.5m
                tx, ty = sx + left_x * SCOUT_LEFT_DIST, sy + left_y * SCOUT_LEFT_DIST
                self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)
                if sub_elapsed > SCOUT_SUB_TIMEOUT:
                    self.get_logger().warn("Scout left-move timeout, pausing")
                    self._scout_sub = 4; self._scout_sub_start = self.get_clock().now(); return
                if math.hypot((self.x - self.home_x) - tx, (self.y - self.home_y) - ty) < SCOUT_SUB_ARRIVE:
                    self._scout_sub = 4; self._scout_sub_start = self.get_clock().now()
                    self.get_logger().info("Scout: left 1.5m done, pausing 5s")
                return
            if self._scout_sub == 4:
                # 子阶段4: 停 5s (位置保持)
                tx, ty = sx + left_x * SCOUT_LEFT_DIST, sy + left_y * SCOUT_LEFT_DIST
                self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)
                if sub_elapsed >= SCOUT_PAUSE_TIME:
                    self._scout_sub = 5; self._scout_sub_start = self.get_clock().now()
                    self.get_logger().info("Scout: pause done, moving right 3m")
                return
            if self._scout_sub == 5:
                # 子阶段5: 机体右移 3m (终点=预定点右侧1.5m)
                tx = sx + left_x * SCOUT_LEFT_DIST - left_x * SCOUT_RIGHT_DIST
                ty = sy + left_y * SCOUT_LEFT_DIST - left_y * SCOUT_RIGHT_DIST
                self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)
                if sub_elapsed > SCOUT_SUB_TIMEOUT:
                    self.get_logger().warn("Scout right-move timeout, pausing")
                    self._scout_sub = 6; self._scout_sub_start = self.get_clock().now(); return
                if math.hypot((self.x - self.home_x) - tx, (self.y - self.home_y) - ty) < SCOUT_SUB_ARRIVE:
                    self._scout_sub = 6; self._scout_sub_start = self.get_clock().now()
                    self.get_logger().info("Scout: right 3m done, pausing 5s")
                return
            # 子阶段6: 停 5s → 返航
            tx = sx + left_x * SCOUT_LEFT_DIST - left_x * SCOUT_RIGHT_DIST
            ty = sy + left_y * SCOUT_LEFT_DIST - left_y * SCOUT_RIGHT_DIST
            self._pub_pos(self.home_x + tx, self.home_y + ty, scout_z, yaw_locked)
            if sub_elapsed >= SCOUT_PAUSE_TIME:
                self._scout_done = True                    # 2026-09-12 e: 写入 MISSION SUMMARY
                self._scout_elapsed = self._step_elapsed()
                self.get_logger().info(
                    f"Scout maneuver complete ({self._scout_elapsed:.1f}s, 7 sub-phases), returning home")
                self._enter_step(STEP_RETURN)

        # ---- RETURN (2026-08-15): 飞回 home → 位置保持判稳 (漂移<0.3m/3s) → LAND ----
        elif self.step == STEP_RETURN:
            if self._step_elapsed() > STEP_TIMEOUT: self._enter_step(STEP_LAND); return
            dist_to_home = math.hypot(self.x - self.home_x, self.y - self.home_y)
            if self._return_sub == 0:
                # 子阶段0: 速度控制飞回 home 上方 (末段 ramp-down)
                vx, vy, _ = self._fly_toward(self.home_x, self.home_y, RETURN_SPEED)
                self._pub_vel(vx, vy, (cruise_z-self.z)*TRACK_VZ_KP, yaw_locked)
                if dist_to_home < POS_ARRIVE_DIST and abs(self.z-cruise_z) < 0.5:
                    self._return_sub = 1
                    self._return_sub_start = self.get_clock().now()
                    self.get_logger().info("Return: over home, stabilizing before land")
                return
            # 子阶段1: 位置保持 home 上方 (MPC 隐式修正残余偏差), 漂移窗口判稳
            sub_elapsed = (self.get_clock().now() - self._return_sub_start).nanoseconds * 1e-9
            self._pub_pos(self.home_x, self.home_y, cruise_z, yaw_locked)
            self._stab_pos_hist.append((self.x, self.y))
            if sub_elapsed > RETURN_STAB_TIMEOUT:
                self.get_logger().warn(
                    f"Return stabilize timeout {sub_elapsed:.1f}s > {RETURN_STAB_TIMEOUT}s, landing anyway")
                self._return_stab_info = (sub_elapsed, float("nan"))  # 2026-09-12 e: 记录超时未判稳
                self._enter_step(STEP_LAND); return
            if sub_elapsed >= RETURN_STAB_WINDOW:
                xs = [p[0] for p in self._stab_pos_hist]
                ys = [p[1] for p in self._stab_pos_hist]
                drift = max(max(xs)-min(xs), max(ys)-min(ys))
                if drift < RETURN_STAB_DRIFT_MAX:
                    self.get_logger().info(
                        f"Return: stable (drift {drift:.2f}m over {RETURN_STAB_WINDOW:.0f}s), landing")
                    self._return_stab_info = (sub_elapsed, drift)  # 2026-09-12 e: 写入 MISSION SUMMARY
                    self._enter_step(STEP_LAND)

        # ---- LAND (2026-08-15: 时间锚定限速下降 0.4m/s, 防降太猛) ----
        elif self.step == STEP_LAND:
            if self._step_elapsed() > STEP_TIMEOUT: self._send_cmd(400, param1=0.0); self._shutdown("land_step_timeout"); return
            if not self._land_cmd_sent:
                low_z = self.home_z - 0.5
                if self._land_entry_z is None:
                    self._land_entry_z = self.z
                # 时间锚定: 目标从入口高度以 LAND_DESCENT_SPEED 匀速下移, min() 封顶 (独立于飞机实时位置)
                target_z = min(low_z, self._land_entry_z + LAND_DESCENT_SPEED * self._step_elapsed())
                self._pub_pos(self.home_x, self.home_y, target_z, yaw_locked)
                if abs(self.z-low_z) < 0.5:
                    self._land_cmd_sent = True
                    self._land_cmd_time = self.get_clock().now()
                    # 2026-09-12 e: LAND 命令日志 (自 d) + 下降段实际速率复盘
                    self.get_logger().info(
                        f"Sending MAV_CMD_NAV_LAND at NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                        f"H={self.home_z-self.z:.2f}m (下降用时 {self._step_elapsed():.1f}s, "
                        f"实测 {self.vz:.2f}m/s, 限速 {LAND_DESCENT_SPEED:.2f}m/s)")
                    self._send_cmd(21, param1=0.0, param2=0.0, param7=float('nan'))
            elif (self.get_clock().now() - self._land_cmd_time).nanoseconds * 1e-9 > 15.0:
                self.get_logger().info("LAND timeout after cmd, forcing disarm")
                self._land_pos = (self.x, self.y)  # 2026-09-12 e
                self._send_cmd(400, param1=0.0); self._shutdown("land_timeout"); return
            else:
                if self.landed:
                    if self._land_dwell_start is None:
                        self._land_dwell_start = self.get_clock().now()
                        self._land_pos = (self.x, self.y)  # 2026-09-12 e
                    elif (self.get_clock().now()-self._land_dwell_start).nanoseconds > LAND_DWELL_TIME*1e9:
                        self._land_pos = (self.x, self.y)
                        self.get_logger().info(
                            f"Landed (vehicle_land_detected), NED({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                            f"dist_from_home={math.hypot(self.x-self.home_x,self.y-self.home_y):.2f}m")
                        self._send_cmd(400, param1=0.0); self._shutdown("normal")
                elif abs(self.home_z - self.z) < 0.15:
                    if self._land_dwell_start is None:
                        self._land_dwell_start = self.get_clock().now()
                        self._land_pos = (self.x, self.y)  # 2026-09-12 e
                    elif (self.get_clock().now()-self._land_dwell_start).nanoseconds > LAND_DWELL_TIME*1e9:
                        self._land_pos = (self.x, self.y)
                        self.get_logger().info(
                            f"Landed (height fallback H={self.home_z-self.z:.2f}m), "
                            f"NED({self.x:.2f},{self.y:.2f}) "
                            f"dist_from_home={math.hypot(self.x-self.home_x,self.y-self.home_y):.2f}m")
                        self._send_cmd(400, param1=0.0); self._shutdown("normal")
                else: self._land_dwell_start = None


def main():
    rclpy.init()
    node = FR3Mission()
    try:
        # 2026-09-12 e: 用 spin_once 循环替代 rclpy.spin —— 便于 _shutdown() 置标志后立即收尾退出
        # (rclpy.spin 会一直阻塞到 context 关闭, 而 context 不能在回调内关闭 → 进程挂住)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C)")
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
