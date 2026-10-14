#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleLandDetected,
)
import math

class OffboardMissionNode(Node):
    def __init__(self):
        super().__init__('offboard_mission_node')

        qos = qos_profile_sensor_data

        # ---------- 发布者 ----------
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ---------- 订阅者（vehicle_status_v1 有 DDS type checksum 不匹配，改用 control_mode）----------
        self.control_mode_sub = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos)
        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, qos)

        # ---------- 内部状态 ----------
        self._control_mode = None
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.home_x, self.home_y, self.home_z = None, None, None
        self.locked_heading = None
        self.landed_state = False

        self.mission_step = 0
        self.takeoff_height = 3.0
        self.forward_speed = 2.0
        self.forward_time = 5.0

        self.forward_start_time = None
        self.hover_start_time = None
        self.mission_complete = False

        self.mode_switch_requested = False
        self.arm_requested = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Offboard mission node started for Orin NX.')

    # ---------- 回调 ----------
    def control_mode_callback(self, msg):
        self._control_mode = msg

    def local_pos_callback(self, msg):
        self.current_x, self.current_y, self.current_z = msg.x, msg.y, msg.z
        if self.home_x is None:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.locked_heading = msg.heading
            self.get_logger().info(
                f"Home: ({self.home_x:.2f},{self.home_y:.2f},{self.home_z:.2f}), "
                f"Heading: {math.degrees(self.locked_heading):.2f}°"
            )

    def land_detected_callback(self, msg):
        self.landed_state = msg.landed

    # ---------- 辅助函数 ----------
    def publish_position(self, x, y, z, yaw):
        offboard_msg = OffboardControlMode(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=True
        )
        traj = TrajectorySetpoint(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            position=[float(x), float(y), float(z)],
            yaw=float(yaw),
            velocity=[float('nan')]*3,
            acceleration=[float('nan')]*3
        )
        self.offboard_mode_pub.publish(offboard_msg)
        self.traj_pub.publish(traj)

    def send_vehicle_command(self, command, **params):
        cmd = VehicleCommand(
            timestamp=self.get_clock().now().nanoseconds // 1000,
            command=command,
            target_system=1,
            target_component=1,
            **params
        )
        self.cmd_pub.publish(cmd)

    # ---------- 主控制循环 ----------
    def control_loop(self):
        if self.home_x is None or self.locked_heading is None:
            return

        target_z = self.home_z - self.takeoff_height

        # ---- 模式切换与解锁 (VehicleControlMode, 已验证 DDS 可用) ----
        cm = self._control_mode
        if cm is None:
            return  # 等待首个 control_mode 消息

        if not cm.flag_armed:
            if not cm.flag_control_offboard_enabled and not self.mode_switch_requested:
                self.mode_switch_requested = True
                self.get_logger().info("Requesting Offboard mode...")
                self.send_vehicle_command(176, param1=1.0, param2=6.0)
            elif cm.flag_control_offboard_enabled and not self.arm_requested:
                self.arm_requested = True
                self.get_logger().info("Sending arm command...")
                self.send_vehicle_command(400, param1=1.0)
            else:
                self.publish_position(self.current_x, self.current_y, self.current_z, self.locked_heading)
            return

        # ---- 阶段 0: 起飞 ----
        if self.mission_step == 0:
            if self.current_z <= target_z + 0.1:
                self.mission_step = 1
                self.forward_start_time = self.get_clock().now()
                self.get_logger().info("Takeoff complete, flying forward")
            self.publish_position(self.home_x, self.home_y, target_z, self.locked_heading)

        # ---- 阶段 1: 沿锁定航向前飞 ----
        elif self.mission_step == 1:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds * 1e-9
            if elapsed < self.forward_time:
                dist = self.forward_speed * elapsed
                target_x = self.home_x + dist * math.cos(self.locked_heading)
                target_y = self.home_y + dist * math.sin(self.locked_heading)
                self.publish_position(target_x, target_y, target_z, self.locked_heading)
            else:
                self.mission_step = 2
                self.hover_start_time = self.get_clock().now()
                self.get_logger().info("Hovering")

        # ---- 阶段 2: 悬停 ----
        elif self.mission_step == 2:
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds * 1e-9
            if elapsed < 2.0:
                dist = self.forward_speed * self.forward_time
                target_x = self.home_x + dist * math.cos(self.locked_heading)
                target_y = self.home_y + dist * math.sin(self.locked_heading)
                self.publish_position(target_x, target_y, target_z, self.locked_heading)
            else:
                self.mission_step = 3
                self.get_logger().info("Initiating land")

        # ---- 阶段 3: 降落 ----
        elif self.mission_step == 3:
            if not self.mission_complete:
                self.get_logger().info("Sending land command")
                self.send_vehicle_command(21)  # VEHICLE_CMD_NAV_LAND
                self.mission_complete = True
            # 保持 offboard 心跳，直到飞控完全接管并上锁
            self.publish_position(self.current_x, self.current_y, target_z, self.locked_heading)
            if self.landed_state:
                self.get_logger().info("Landed, disarming")
                self.send_vehicle_command(400, param1=0.0)
                self.timer.cancel()
                self.get_logger().info("Mission complete, node stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()