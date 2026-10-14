#!/usr/bin/env python3
"""
优化版 Offboard 任务（仿真）
适用于 PX4 SITL + Gazebo，DDS 通信
话题后缀：status_v4
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

class OffboardMissionOpt(Node):
    def __init__(self):
        super().__init__('offboard_mission_opt')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 发布者
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 订阅者（唯一区别：状态话题后缀）
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, qos)  # ← 仿真用 _v4
        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_cb, qos)
        self.land_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_cb, qos)

        # 状态变量
        self.status = None
        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self.heading = None
        self.landed = False

        self.step = 0
        self.takeoff_height = 3.0
        self.forward_speed = 2.0
        self.forward_time = 5.0

        self.forward_start_time = None
        self.hover_start_time = None
        self.mission_complete = False

        # 模式切换相关
        self.mode_switch_requested = False
        self.arm_requested = False
        self.mode_request_start = None

        # 初始等待
        self.initial_wait_done = False
        self.init_wait_start = None

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Optimized offboard mission (Simulation) started')

    def status_cb(self, m): self.status = m
    def land_cb(self, m): self.landed = m.landed
    def local_cb(self, m):
        self.x, self.y, self.z = m.x, m.y, m.z
        if self.home_x is None:
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.heading = m.heading

    def pub_pos(self, x, y, z, yaw):
        off = OffboardControlMode(timestamp=self.get_clock().now().nanoseconds//1000, position=True)
        traj = TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds//1000,
            position=[float(x), float(y), float(z)],
            yaw=float(yaw),
            velocity=[float('nan')]*3,
            acceleration=[float('nan')]*3
        )
        self.offboard_mode_pub.publish(off)
        self.traj_pub.publish(traj)

    def cmd(self, c, **p):
        self.cmd_pub.publish(VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds//1000,
            command=c, target_system=1, target_component=1, **p))

    def loop(self):
        if self.home_x is None: return

        target_z = self.home_z - self.takeoff_height

        # ----- 初始等待：让估计稳定（3 秒）-----
        if not self.initial_wait_done:
            if self.init_wait_start is None:
                self.init_wait_start = self.get_clock().now()
                self.get_logger().info("Waiting for position estimate to stabilize...")
            if (self.get_clock().now() - self.init_wait_start).nanoseconds < 3e9:
                self.pub_pos(self.x, self.y, self.z, self.heading)
                return
            self.initial_wait_done = True
            self.get_logger().info("Initial wait done.")

        # ----- 模式切换与解锁（带超时）-----
        if self.status and self.status.arming_state != 2:
            if self.mode_request_start is None:
                self.mode_request_start = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.mode_request_start).nanoseconds * 1e-9
            if elapsed > 15.0:
                self.get_logger().error("Timeout: could not enter offboard or arm!")
                self.timer.cancel()
                return

            if self.status.nav_state != 14 and not self.mode_switch_requested:
                self.mode_switch_requested = True
                self.get_logger().info("Requesting Offboard mode...")
                self.cmd(176, param1=1.0, param2=6.0)
            elif self.status.nav_state == 14 and not self.arm_requested:
                self.arm_requested = True
                self.get_logger().info("Sending arm command...")
                self.cmd(400, param1=1.0)
            # 关键：持续发送目标起飞位置
            self.pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return

        if not self.status or self.status.arming_state != 2:
            self.pub_pos(self.home_x, self.home_y, target_z, self.heading)
            return

        # ----- 任务状态机 -----
        if self.step == 0:          # 起飞
            if self.z <= target_z + 0.1:
                self.step = 1
                self.forward_start_time = self.get_clock().now()
                self.get_logger().info("Takeoff complete, flying forward")
            self.pub_pos(self.home_x, self.home_y, target_z, self.heading)

        elif self.step == 1:        # 前飞
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds * 1e-9
            if elapsed < self.forward_time:
                dist = self.forward_speed * elapsed
                target_x = self.home_x + dist * math.cos(self.heading)
                target_y = self.home_y + dist * math.sin(self.heading)
                self.pub_pos(target_x, target_y, target_z, self.heading)
            else:
                self.step = 2
                self.hover_start_time = self.get_clock().now()
                self.get_logger().info("Forward flight complete, hovering")

        elif self.step == 2:        # 悬停
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds * 1e-9
            if elapsed < 2.0:
                dist = self.forward_speed * self.forward_time
                target_x = self.home_x + dist * math.cos(self.heading)
                target_y = self.home_y + dist * math.sin(self.heading)
                self.pub_pos(target_x, target_y, target_z, self.heading)
            else:
                self.step = 3
                self.get_logger().info("Starting auto-land")

        elif self.step == 3:        # 降落
            if not self.mission_complete:
                self.get_logger().info("Sending land command")
                self.cmd(21)
                self.mission_complete = True
            self.pub_pos(self.x, self.y, target_z, self.heading)
            if self.landed:
                self.cmd(400, param1=0.0)
                self.timer.cancel()
                self.get_logger().info("Landed, disarmed. Mission complete")

def main():
    rclpy.init()
    rclpy.spin(OffboardMissionOpt())
    rclpy.shutdown()

if __name__ == '__main__':
    main()