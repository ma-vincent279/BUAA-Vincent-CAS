#!/usr/bin/env python3
"""
对称横向扫描任务 — 锁定航向，定点前飞并在两个扫描点各执行一次
左5m/右5m/回中的对称扫描，最后返航降落。
"""

import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---- 任务阶段常量 ----
(STEP_IDLE, STEP_TAKEOFF, STEP_FORWARD, STEP_SCAN,
 STEP_NEXT_FWD, STEP_RETURN, STEP_LAND) = range(7)

STEP_NAME = {
    STEP_IDLE: "IDLE", STEP_TAKEOFF: "TAKEOFF", STEP_FORWARD: "FWD",
    STEP_SCAN: "SCAN", STEP_NEXT_FWD: "NEXT_FWD", STEP_RETURN: "RETURN",
    STEP_LAND: "LAND",
}

# ---- 参数 ----
TAKEOFF_HEIGHT   = 5.0    # 起飞高度 (m)
FORWARD_STEP     = 10.0   # 每次前飞距离 (m)
SCAN_HALF_WIDTH  = 5.0    # 扫描半宽 (m)，左/右各 5m
NUM_SCANS        = 2      # 扫描次数
CRUISE_SPEED     = 1.0    # 巡航速度 (m/s)
SCAN_SPEED       = 0.8    # 横向扫描速度因子
ACCEL            = 0.4    # 加速度 (m/s²)
VEL_TO_POS_SWITCH = 2.5   # 速度控→位置控切换距离 (m)
POS_ARRIVE_DIST  = 0.3    # 到达判定距离 (m)
STEP_TIMEOUT     = 30.0   # 单步超时 (s)，参照 cwkj_ws 实机安全策略
LAND_DWELL_TIME  = 3.0    # 降落到地后持续 disarm 前等待 (s)


class OffboardMission(Node):
    """锁定航向的对称横向扫描任务"""

    def __init__(self):
        super().__init__('offboard_mission')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local, qos)
        self.land_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._cb_land, qos)

        # ---- 状态变量 ----
        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self.heading = None
        self.landed = False
        self.status = None

        # 任务控制
        self.step = STEP_IDLE
        self.sub_step = 0
        self.scan_origin_x = self.scan_origin_y = 0.0
        self.scan_count = 0

        # 模式切换
        self._mode_req = self._arm_req = False
        self._mode_req_start = None
        self._init_wait_start = None
        self._initial_wait_done = False

        # 混合控制
        self._final_approach = False
        self._pos_target = None

        # 超时保护
        self._step_start_time = None
        self._step_start_step = None

        # 降落
        self._land_dwell_start = None

        # 调试
        self._print_ctr = 0
        self._dbg_target = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Offboard mission ready "
                               f"(cruise={CRUISE_SPEED}m/s scan={SCAN_HALF_WIDTH}m)")

    # ================================================================
    # 回调
    # ================================================================
    def _cb_status(self, m): self.status = m
    def _cb_land(self, m): self.landed = m.landed

    def _cb_local(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if self.home_x is None and m.xy_valid and math.isfinite(m.heading):
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.heading = m.heading
            self.get_logger().info(
                f"Home: ({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}) "
                f"heading={math.degrees(self.heading):.1f}°")

    # ================================================================
    # 控制发布方法 — 参照 cwkj_ws OffboardControl 的设计
    # ================================================================
    def _pub_vel(self, vx, vy, vz, yaw):
        """NED 坐标系速度控制"""
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, velocity=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            velocity=[float(vx), float(vy), float(vz)],
            yaw=float(yaw),
            position=[float('nan')] * 3, acceleration=[float('nan')] * 3))

    def _pub_pos(self, x, y, z, yaw):
        """NED 坐标系位置控制"""
        self.offboard_mode_pub.publish(OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000, position=True))
        self.traj_pub.publish(TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=[float(x), float(y), float(z)],
            yaw=float(yaw),
            velocity=[float('nan')] * 3, acceleration=[float('nan')] * 3))

    def _pub_body_vel_posz(self, vel_fwd, vel_lat, target_z, yaw):
        """机体坐标系水平速度 + NED 高度保持
        参照 cwkj_ws send_body_velxy_posz_setpoint。
        用于视觉跟踪场景，当前任务暂未使用，预留给后续开发。"""
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        vx = vel_fwd * cos_h - vel_lat * sin_h
        vy = vel_fwd * sin_h + vel_lat * cos_h
        self._pub_vel(vx, vy, 0.0, yaw)
        # 高度通过 _pub_pos 单独控制

    def _send_cmd(self, command, **params):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=command, target_system=1, target_component=1, **params))

    # ================================================================
    # 路径计算
    # ================================================================
    def _forward_point(self, d):
        return (self.home_x + d * math.cos(self.heading),
                self.home_y + d * math.sin(self.heading))

    def _lateral_point(self, d):
        """d: 左正右负的横向偏移量"""
        return (self.scan_origin_x + d * math.sin(self.heading),
                self.scan_origin_y - d * math.cos(self.heading))

    def _dist_to(self, x, y, z):
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2 + (self.z - z) ** 2)

    # ================================================================
    # 距离基闭环速度剖面
    # ================================================================
    def _trapezoidal_step(self, tx, ty, tz, cruise_spd):
        """基于剩余距离的闭环速度控制，保证收敛"""
        dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 0.05:
            return (0.0, 0.0, 0.0)

        decel_dist = (cruise_spd ** 2) / (2 * ACCEL)
        spd = cruise_spd if dist > decel_dist else math.sqrt(2 * ACCEL * dist)
        ux, uy, uz = dx / dist, dy / dist, dz / dist
        return (ux * spd, uy * spd, uz * spd)

    def _move_to_target(self, tx, ty, tz, cruise_spd):
        """混合控制：速度接近 → 位置精调"""
        if not self._final_approach:
            vx, vy, vz = self._trapezoidal_step(tx, ty, tz, cruise_spd)
            self._pub_vel(vx, vy, vz, self.heading)
            if self._dist_to(tx, ty, tz) < VEL_TO_POS_SWITCH:
                self._final_approach = True
                self._pos_target = (tx, ty, tz)
        else:
            pt = self._pos_target
            self._pub_pos(pt[0], pt[1], pt[2], self.heading)

    def _check_arrival(self):
        if self._pos_target is None or not self._final_approach:
            return False
        pt = self._pos_target
        return self._dist_to(pt[0], pt[1], pt[2]) < POS_ARRIVE_DIST

    def _reset_move(self):
        self._final_approach = False
        self._pos_target = None

    # ================================================================
    # 超时 & 步进管理
    # ================================================================
    def _enter_step(self, step):
        self.step = step
        self._step_start_time = self.get_clock().now()
        self._step_start_step = step

    def _check_step_timeout(self):
        """返回 True 表示当前步已超时，需要强制跳过"""
        if self.step <= STEP_IDLE:
            return False
        elapsed = (self.get_clock().now() - self._step_start_time).nanoseconds * 1e-9
        return elapsed > STEP_TIMEOUT

    # ================================================================
    # 主循环
    # ================================================================
    def control_loop(self):
        if self.home_x is None or self.heading is None:
            return

        # ---- 调试打印 ----
        self._print_ctr += 1
        if self._print_ctr >= 20:
            tgt = self._dbg_target
            tag = STEP_NAME.get(self.step, "???")
            if tgt is not None:
                err = self._dist_to(tgt[0], tgt[1], tgt[2])
                self.get_logger().info(
                    f"[{tag}] Pos:({self.x:.2f},{self.y:.2f},{self.z:.2f}) "
                    f"Tgt:({tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f}) err={err:.2f}m")
            else:
                self.get_logger().info(
                    f"[{tag}] Pos:({self.x:.2f},{self.y:.2f},{self.z:.2f}) Tgt:None")
            self._print_ctr = 0

        target_z = self.home_z - TAKEOFF_HEIGHT

        # ---- 初始等待 ----
        if not self._initial_wait_done:
            if self._init_wait_start is None:
                self._init_wait_start = self.get_clock().now()
            if (self.get_clock().now() - self._init_wait_start).nanoseconds < 3e9:
                self._pub_vel(0, 0, 0, self.heading)
                return
            self._initial_wait_done = True

        # ---- 模式切换与解锁 (5s 重试间隔，参照 cwkj_ws) ----
        if self.status and self.status.arming_state != 2:
            if self._mode_req_start is None:
                self._mode_req_start = self.get_clock().now()
            if (self.get_clock().now() - self._mode_req_start).nanoseconds * 1e-9 > 15.0:
                self.get_logger().error("Mode switch timeout — aborting")
                self.timer.cancel()
                return
            if self.status.nav_state != 14 and not self._mode_req:
                self._mode_req = True
                self._send_cmd(176, param1=1.0, param2=6.0)
            elif self.status.nav_state == 14 and not self._arm_req:
                self._arm_req = True
                self._send_cmd(400, param1=1.0)
            self._pub_vel(0, 0, 0, self.heading)
            return
        if not self.status or self.status.arming_state != 2:
            self._pub_vel(0, 0, 0, self.heading)
            return

        # ---- 步进超时检查 ----
        if self._check_step_timeout():
            self.get_logger().warn(
                f"[{STEP_NAME.get(self.step,'?')}] Step timeout — forcing advance")
            if self.step == STEP_LAND:
                self._send_cmd(400, param1=0.0)  # 强制 disarm
                self.timer.cancel()
                return
            self._reset_move()
            self.step += 1
            self.sub_step = 0
            self._enter_step(self.step)

        # ================================================================
        # 任务状态机
        # ================================================================
        if self.step == STEP_IDLE:
            self._enter_step(STEP_TAKEOFF)

        # ---- 起飞 ----
        elif self.step == STEP_TAKEOFF:
            self._dbg_target = (self.home_x, self.home_y, target_z)
            if abs(self.z - target_z) < 0.2:
                self._reset_move()
                self._enter_step(STEP_FORWARD)
                self.get_logger().info("Takeoff complete → forward")
            else:
                self._pub_pos(self.home_x, self.home_y, target_z, self.heading)

        # ---- 前飞 ----
        elif self.step == STEP_FORWARD:
            tx, ty = self._forward_point(FORWARD_STEP)
            self._dbg_target = (tx, ty, target_z)
            self._move_to_target(tx, ty, target_z, CRUISE_SPEED)
            if self._check_arrival():
                self.scan_origin_x, self.scan_origin_y = self.x, self.y
                self.scan_count = 0
                self.sub_step = 0
                self._reset_move()
                self._enter_step(STEP_SCAN)
                self.get_logger().info("Forward done → scan 1")

        # ---- 横向扫描 ----
        elif self.step == STEP_SCAN:
            # 左5 / 右5(从中心) / 回中心 = [5, -5, 0]
            lateral_seq = [SCAN_HALF_WIDTH, -SCAN_HALF_WIDTH, 0.0]
            idx = self.sub_step
            if idx < 3:
                tx, ty = self._lateral_point(lateral_seq[idx])
                self._dbg_target = (tx, ty, target_z)
                self._move_to_target(tx, ty, target_z, CRUISE_SPEED * SCAN_SPEED)
                if self._check_arrival():
                    self.sub_step += 1
                    self._reset_move()
                    if self.sub_step >= 3:
                        self.scan_count += 1
                        if self.scan_count < NUM_SCANS:
                            self._enter_step(STEP_NEXT_FWD)
                            self.sub_step = 0
                        else:
                            self._enter_step(STEP_RETURN)
                        self.get_logger().info(
                            f"Scan {self.scan_count}/{NUM_SCANS} done")

        # ---- 前飞到下一扫描点 ----
        elif self.step == STEP_NEXT_FWD:
            total_d = (self.scan_count + 1) * FORWARD_STEP
            tx, ty = self._forward_point(total_d)
            self._dbg_target = (tx, ty, target_z)
            self._move_to_target(tx, ty, target_z, CRUISE_SPEED)
            if self._check_arrival():
                self.scan_origin_x, self.scan_origin_y = self.x, self.y
                self.sub_step = 0
                self._reset_move()
                self._enter_step(STEP_SCAN)

        # ---- 返航 ----
        elif self.step == STEP_RETURN:
            self._dbg_target = (self.home_x, self.home_y, target_z)
            self._move_to_target(self.home_x, self.home_y, target_z, CRUISE_SPEED)
            if self._check_arrival():
                self._reset_move()
                self._enter_step(STEP_LAND)
                self.get_logger().info("Returned home → landing")

        # ---- 降落 (位置控降到 home_z, landed 检测 + 高度回退双保险) ----
        elif self.step == STEP_LAND:
            self._dbg_target = (self.home_x, self.home_y, self.home_z)
            self._pub_pos(self.home_x, self.home_y, self.home_z, self.heading)

            # 主检测：飞控 land detector
            if self.landed:
                if self._land_dwell_start is None:
                    self._land_dwell_start = self.get_clock().now()
                elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                    self._send_cmd(400, param1=0.0)
                    self.timer.cancel()
                    self.get_logger().info("Mission complete — disarmed (landed)")
                    return

            # 回退检测：SITL 中 land detector 可能不触发，
            #   若高度持续贴近 home_z 则视为已落地
            if abs(self.z - self.home_z) < 0.15:
                if self._land_dwell_start is None:
                    self._land_dwell_start = self.get_clock().now()
                elif (self.get_clock().now() - self._land_dwell_start).nanoseconds > LAND_DWELL_TIME * 1e9:
                    self._send_cmd(400, param1=0.0)
                    self.timer.cancel()
                    self.get_logger().info("Mission complete — disarmed (altitude)")
                    return
            else:
                self._land_dwell_start = None  # 高度不够，重置计时


def main(args=None):
    rclpy.init(args=args)
    node = OffboardMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
