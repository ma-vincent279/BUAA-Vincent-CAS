#!/usr/bin/env python3
"""
模拟视觉检测节点 — 支持三桶，一次只跟踪一个。
桶位置通过 ROS 参数配置（Gazebo 坐标系），内部转换为 NED。
/bucket_target_id 话题 (std_msgs/Int32): 0=桶1, 1=桶2, 2=桶3
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int32
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class BucketVisionDetector(Node):
    def __init__(self):
        super().__init__('bucket_vision_detector')

        # 桶1 位置参数 (Gazebo 坐标系) — 直径 0.15m，半径 0.075m
        self.declare_parameter('bucket1_gz_x', 35.51)
        self.declare_parameter('bucket1_gz_y', 2.0)
        self.declare_parameter('bucket1_gz_z', 0.15)
        # 桶2 位置参数 — 直径 0.20m，半径 0.10m
        self.declare_parameter('bucket2_gz_x', 37.01)
        self.declare_parameter('bucket2_gz_y', 1.0)
        self.declare_parameter('bucket2_gz_z', 0.15)
        # 桶3 位置参数 — 直径 0.25m，半径 0.125m
        self.declare_parameter('bucket3_gz_x', 35.51)
        self.declare_parameter('bucket3_gz_y', 0.0)
        self.declare_parameter('bucket3_gz_z', 0.15)
        # 飞机 Gazebo 生成点
        self.declare_parameter('spawn_gz_x', 1.01)
        self.declare_parameter('spawn_gz_y', 0.98)
        self.declare_parameter('spawn_gz_z', 0.83)

        sgx = self.get_parameter('spawn_gz_x').value
        sgy = self.get_parameter('spawn_gz_y').value
        sgz = self.get_parameter('spawn_gz_z').value

        def gz_to_ned(gx, gy, gz):
            return (gy - sgy, gx - sgx, -(gz - sgz))

        b1 = (self.get_parameter('bucket1_gz_x').value,
              self.get_parameter('bucket1_gz_y').value,
              self.get_parameter('bucket1_gz_z').value)
        b2 = (self.get_parameter('bucket2_gz_x').value,
              self.get_parameter('bucket2_gz_y').value,
              self.get_parameter('bucket2_gz_z').value)
        b3 = (self.get_parameter('bucket3_gz_x').value,
              self.get_parameter('bucket3_gz_y').value,
              self.get_parameter('bucket3_gz_z').value)

        self.buckets = [
            {'gz': b1, 'ned': gz_to_ned(*b1), 'radius': 0.075},
            {'gz': b2, 'ned': gz_to_ned(*b2), 'radius': 0.100},
            {'gz': b3, 'ned': gz_to_ned(*b3), 'radius': 0.125},
        ]
        self._active = 0  # 默认跟踪桶1

        for i, b in enumerate(self.buckets):
            self.get_logger().info(
                f"Bucket{i+1} Gazebo:({b['gz'][0]:.1f},{b['gz'][1]:.1f},{b['gz'][2]:.2f}) "
                f"radius={b['radius']:.3f}m "
                f"→ NED:({b['ned'][0]:.2f},{b['ned'][1]:.2f},{b['ned'][2]:.2f})")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local, qos)

        self.det_pub = self.create_publisher(
            RegionOfInterest, '/bucket/detection', 10)

        self.target_sub = self.create_subscription(
            Int32, '/bucket_target_id', self._cb_target, 10)

        # 摄像头参数
        self.img_w = 640
        self.img_h = 360
        self.hfov = math.radians(85)
        self.vfov = math.radians(54)
        self.fx = self.img_w / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.img_h / (2.0 * math.tan(self.vfov / 2.0))

        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.heading = None

        self.timer = self.create_timer(0.05, self._publish_detection)
        self.get_logger().info("Triple-bucket vision detector ready — /bucket_target_id (Int32, 0-2)")

    def _cb_target(self, msg):
        if 0 <= msg.data < 3:
            self._active = msg.data
            self.get_logger().info(f"Target switched → bucket {self._active + 1}")
        else:
            self.get_logger().warn(f"Invalid bucket_target_id={msg.data}, ignoring")

    def _cb_local(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading

    def _publish_detection(self):
        if self.heading is None:
            return

        b = self.buckets[self._active]
        bx, by, bz = b['ned']
        bucket_radius = b['radius']
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

        box_w = max(5, min(200, int(2.0 * self.fx * bucket_radius / dz)))
        box_h = max(5, min(200, int(2.0 * self.fy * bucket_radius / dz)))

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
    node = BucketVisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
