#!/usr/bin/env python3
"""
独立裁判节点 — 通过 /gazebo/model_states 获取载荷落点，判定投放精度。
参数: bucket_gz_x/y/z (桶 Gazebo 坐标), bucket_radius (桶半径，默认 0.125m)
判定: 水平距离 < bucket_radius → A区 2分，< 1.0m → B区 1分，否则 0分
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


class DropJudge(Node):
    def __init__(self):
        super().__init__('drop_judge')

        self.declare_parameter('bucket_gz_x', 11.2)
        self.declare_parameter('bucket_gz_y', 0.5)
        self.declare_parameter('bucket_gz_z', 0.25)
        self.declare_parameter('bucket_radius', 0.125)

        self.bucket_x = self.get_parameter('bucket_gz_x').value
        self.bucket_y = self.get_parameter('bucket_gz_y').value
        self.bucket_z = self.get_parameter('bucket_gz_z').value
        self.bucket_radius = self.get_parameter('bucket_radius').value

        self._judged = set()
        self._total_score = 0
        self._payload_count = 0
        # {name: {'history': [(time, z), ...], 'latest_pose': Pose}}
        self._payload_data = {}

        self.model_states_sub = self.create_subscription(
            ModelStates, '/model_states', self._cb_model_states, 10)

        self.timer = self.create_timer(1.0, self._judge_tick)

        self.get_logger().info(
            f"Drop judge ready: bucket at gz({self.bucket_x:.2f},{self.bucket_y:.2f},"
            f"{self.bucket_z:.2f}) radius={self.bucket_radius:.3f}m")

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
            # 只保留最近 3s
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

            # 稳定判定：z < 0.3m 且 1.5s 内波动 < 0.03m
            if not (z_now < 0.3 and z_range < 0.03):
                continue

            pose = data['latest_pose']
            gx, gy = pose.position.x, pose.position.y
            dist = math.hypot(gx - self.bucket_x, gy - self.bucket_y)

            if dist < self.bucket_radius:
                zone, score = "A", 2
            elif dist < 1.0:
                zone, score = "B", 1
            else:
                zone, score = "Miss", 0

            self._total_score += score
            self._payload_count += 1
            self._judged.add(name)

            self.get_logger().info(
                f"[{name}] gz({gx:.3f},{gy:.3f}) dist={dist:.3f}m "
                f"→ {zone}区 {score}分 | 累计: {self._total_score}分/{self._payload_count}枚")

            # 清理已判定载荷数据
            del self._payload_data[name]


def main():
    rclpy.init()
    rclpy.spin(DropJudge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
