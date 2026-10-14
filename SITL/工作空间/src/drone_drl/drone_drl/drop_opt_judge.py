#!/usr/bin/env python3
"""
投放精度优化裁判 — 细化计分规则。
d = 落点与桶中心水平距离 (m)
  d ≤ 0.05    → 10分
  0.05 < d ≤ 0.15 → 8分
  0.15 < d ≤ 0.30 → 6分
  0.30 < d ≤ 0.50 → 4分
  0.50 < d ≤ 0.75 → 2分
  0.75 < d ≤ 1.00 → 1分
  d > 1.00    → 0分

通过 ROS 参数配置桶位置和半径。
打印: 距离: X.XXm, 得分: X
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


class DropOptJudge(Node):
    def __init__(self):
        super().__init__('drop_opt_judge')

        self.declare_parameter('bucket_gz_x', 11.01)
        self.declare_parameter('bucket_gz_y', 0.98)
        self.declare_parameter('bucket_gz_z', 0.25)
        self.declare_parameter('bucket_radius', 0.125)

        self.bucket_x = self.get_parameter('bucket_gz_x').value
        self.bucket_y = self.get_parameter('bucket_gz_y').value
        self.bucket_z = self.get_parameter('bucket_gz_z').value
        self.bucket_radius = self.get_parameter('bucket_radius').value

        self._judged = set()
        self._payload_data = {}

        self.model_states_sub = self.create_subscription(
            ModelStates, '/model_states', self._cb_model_states, 10)

        self.timer = self.create_timer(1.0, self._judge_tick)

        self.get_logger().info(
            f"Drop opt judge ready: bucket gz({self.bucket_x:.2f},{self.bucket_y:.2f},"
            f"{self.bucket_z:.2f}) radius={self.bucket_radius:.3f}m")

    def _score(self, dist):
        if dist <= 0.05:
            return 10
        elif dist <= 0.15:
            return 8
        elif dist <= 0.30:
            return 6
        elif dist <= 0.50:
            return 4
        elif dist <= 0.75:
            return 2
        elif dist <= 1.00:
            return 1
        else:
            return 0

    def _cb_model_states(self, m):
        now = self.get_clock().now()
        for name, pose in zip(m.name, m.pose):
            if not name.startswith('dropped_payload_') or name in self._judged:
                continue
            if name not in self._payload_data:
                self._payload_data[name] = {'history': [], 'latest_pose': None}
                self.get_logger().info(f"Tracking {name}...")
            self._payload_data[name]['history'].append((now, pose.position.z))
            self._payload_data[name]['latest_pose'] = pose
            cutoff = now - rclpy.time.Duration(seconds=3)
            self._payload_data[name]['history'] = [
                (t, z) for t, z in self._payload_data[name]['history'] if t > cutoff]

    def _judge_tick(self):
        now = self.get_clock().now()
        for name, data in list(self._payload_data.items()):
            if name in self._judged:
                continue
            history = data['history']
            if len(history) < 3:
                continue

            recent = [(t, z) for t, z in history
                      if (now - t).nanoseconds * 1e-9 < 1.5]
            if len(recent) < 2:
                continue

            z_vals = [z for _, z in recent]
            z_now = z_vals[-1]
            z_range = max(z_vals) - min(z_vals)

            if not (z_now < 0.3 and z_range < 0.03):
                continue

            pose = data['latest_pose']
            gx, gy = pose.position.x, pose.position.y
            dist = math.hypot(gx - self.bucket_x, gy - self.bucket_y)
            score = self._score(dist)

            self._judged.add(name)
            del self._payload_data[name]

            self.get_logger().info(
                f"========== RESULT ==========")
            self.get_logger().info(
                f"  距离: {dist:.4f}m, 得分: {score}")
            self.get_logger().info(
                f"  落点 gz({gx:.4f},{gy:.4f}) 桶 gz({self.bucket_x:.2f},{self.bucket_y:.2f})")
            self.get_logger().info(
                f"=============================")


def main():
    rclpy.init()
    node = DropOptJudge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
