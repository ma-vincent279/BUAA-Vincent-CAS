#!/usr/bin/env python3
"""
模拟侦察检测节点 — 5 个白筒 Ground Truth 投影。
筒位置通过 ROS 参数配置（Gazebo 坐标系），内部转换为 NED。
通过 /scout/target_id 话题 (std_msgs/Int32) 切换跟踪目标 (0-4)。
每次只发布一个筒的 ROI 到 /scout/detection。
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int32
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ScoutDetector(Node):
    def __init__(self):
        super().__init__('scout_detector')

        # 5 个白筒的 Gazebo 坐标参数 (competition_field 旋转+平移后)
        defaults = [
            ('c0_gz_x', 59.51), ('c0_gz_y', 2.5), ('c0_gz_z', 0.075),
            ('c1_gz_x', 60.51), ('c1_gz_y', -0.5), ('c1_gz_z', 0.075),
            ('c2_gz_x', 61.51), ('c2_gz_y', 2.0), ('c2_gz_z', 0.075),
            ('c3_gz_x', 62.51), ('c3_gz_y', 0.0), ('c3_gz_z', 0.075),
            ('c4_gz_x', 63.51), ('c4_gz_y', 1.0), ('c4_gz_z', 0.075),
            ('spawn_gz_x', 1.01), ('spawn_gz_y', 0.98), ('spawn_gz_z', 0.83),
        ]
        for name, default in defaults:
            self.declare_parameter(name, default)

        sgx = self.get_parameter('spawn_gz_x').value
        sgy = self.get_parameter('spawn_gz_y').value
        sgz = self.get_parameter('spawn_gz_z').value

        def gz_to_ned(gx, gy, gz):
            return (gy - sgy, gx - sgx, -(gz - sgz))

        self.cylinders = []
        for i in range(5):
            gx = self.get_parameter(f'c{i}_gz_x').value
            gy = self.get_parameter(f'c{i}_gz_y').value
            gz = self.get_parameter(f'c{i}_gz_z').value
            self.cylinders.append({
                'gz': (gx, gy, gz),
                'ned': gz_to_ned(gx, gy, gz),
            })
            n = self.cylinders[-1]['ned']
            self.get_logger().info(
                f"Cylinder {i} Gazebo:({gx:.1f},{gy:.1f},{gz:.3f}) "
                f"→ NED:({n[0]:.2f},{n[1]:.2f},{n[2]:.2f})")

        self._active = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local, qos)

        self.det_pub = self.create_publisher(
            RegionOfInterest, '/scout/detection', 10)

        self.target_sub = self.create_subscription(
            Int32, '/scout/target_id', self._cb_target, 10)

        # 摄像头参数
        self.img_w = 640
        self.img_h = 480
        self.hfov = 1.047
        self.vfov = 2.0 * math.atan(math.tan(self.hfov / 2.0) * self.img_h / self.img_w)
        self.fx = self.img_w / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.img_h / (2.0 * math.tan(self.vfov / 2.0))

        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.heading = None

        self.timer = self.create_timer(0.05, self._publish_detection)
        self.get_logger().info("Scout detector ready — /scout/target_id (Int32, 0-4)")

    def _cb_target(self, msg):
        if 0 <= msg.data < 5:
            self._active = msg.data
            self.get_logger().info(f"Target switched → cylinder {self._active}")
        else:
            self.get_logger().warn(f"Invalid target_id={msg.data}, ignoring")

    def _cb_local(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading

    def _publish_detection(self):
        if self.heading is None:
            return

        bx, by, bz = self.cylinders[self._active]['ned']
        dx = bx - self.drone_x
        dy = by - self.drone_y
        dz = bz - self.drone_z

        if dz < 0.3:
            return

        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        body_fwd = dx * cos_h + dy * sin_h
        body_right = -dx * sin_h + dy * cos_h

        px = self.img_w / 2.0 + (body_right / dz) * self.fx
        py = self.img_h / 2.0 - (body_fwd / dz) * self.fy  # 下视摄像头: TOP=前

        cyl_radius = 0.10
        box_w = max(5, min(200, int(2.0 * self.fx * cyl_radius / dz)))
        box_h = max(5, min(200, int(2.0 * self.fy * cyl_radius / dz)))

        if px < -box_w or px > self.img_w + box_w:
            return
        if py < -box_h or py > self.img_h + box_h:
            return

        roi = RegionOfInterest()
        roi.x_offset = max(0, int(px - box_w / 2))
        roi.y_offset = max(0, int(py - box_h / 2))
        roi.width = box_w
        roi.height = box_h
        self.det_pub.publish(roi)


def main():
    rclpy.init()
    node = ScoutDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
