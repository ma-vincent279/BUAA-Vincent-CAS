#!/usr/bin/env python3
"""
DDS 链路调试监听 — 订阅已验证可用的 PX4 话题。
vehicle_status_v1 有 DDS type checksum 不匹配，改用 vehicle_control_mode。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleLandDetected

QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class DDSListener(Node):
    def __init__(self):
        super().__init__('dds_listener')
        self._ctrl_count = 0
        self._pos_count = 0

        # 已验证可用的话题
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self._cb_control_mode, QOS_BE)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local_pos, QOS_BE)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._cb_land, QOS_BE)

        self._timer = self.create_timer(2.0, self._print_status)
        self.get_logger().info(
            'DDS Listener started — monitoring 3 topics (/fmu/out/vehicle_control_mode, '
            '_v1, land_detected)')

    def _cb_control_mode(self, msg):
        self._ctrl_count += 1

    def _cb_local_pos(self, msg):
        self._pos_count += 1

    def _cb_land(self, msg):
        pass

    def _print_status(self):
        self.get_logger().info(
            f'DDS alive: control_mode={self._ctrl_count} msgs, '
            f'local_pos={self._pos_count} msgs')


def main(args=None):
    rclpy.init(args=args)
    node = DDSListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
