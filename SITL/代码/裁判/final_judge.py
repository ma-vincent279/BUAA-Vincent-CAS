#!/usr/bin/env python3
"""
终极项目裁判 — 多桶投放计分。
通过 /drop/impact_point 获取投放瞬间飞机 NED 坐标，
与桶 NED 位置比较计算水平距离和得分。

d = 落点与桶中心水平距离 (m)
  d ≤ 0.05    → 10分
  0.05 < d ≤ 0.15 → 8分
  0.15 < d ≤ 0.30 → 6分
  0.30 < d ≤ 0.50 → 4分
  0.50 < d ≤ 0.75 → 2分
  0.75 < d ≤ 1.00 → 1分
  d > 1.00    → 0分

参数: 3 桶 Gazebo 坐标 + 飞机 spawn 偏移 → 内部转 NED
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class FinalJudge(Node):
    def __init__(self):
        super().__init__('final_judge')

        self.declare_parameter('spawn_gz_x', 1.01)
        self.declare_parameter('spawn_gz_y', 0.98)
        self.declare_parameter('spawn_gz_z', 0.83)

        self.declare_parameter('bucket1_gz_x', 12.51)
        self.declare_parameter('bucket1_gz_y', -0.52)
        self.declare_parameter('bucket1_gz_z', 0.25)
        self.declare_parameter('bucket2_gz_x', 14.01)
        self.declare_parameter('bucket2_gz_y', 2.98)
        self.declare_parameter('bucket2_gz_z', 0.25)
        self.declare_parameter('bucket3_gz_x', 15.01)
        self.declare_parameter('bucket3_gz_y', 0.98)
        self.declare_parameter('bucket3_gz_z', 0.25)
        self.declare_parameter('bucket_radius', 0.125)
        self.declare_parameter('match_max_dist', 3.0)  # 落点与桶的最大匹配距离
        # 2026-10-09: 比赛里每个桶只投一次, 默认沿用"一桶只评一次";
        #   k 验证台是"单桶两投" ⇒ 需要 allow_reuse:=true, 否则第 2 投永远匹配不上
        #   (会报 No matching bucket, min_dist = 到远处占位桶的距离)。
        self.declare_parameter('allow_reuse', False)

        sx = self.get_parameter('spawn_gz_x').value
        sy = self.get_parameter('spawn_gz_y').value
        sz = self.get_parameter('spawn_gz_z').value
        self._match_max = self.get_parameter('match_max_dist').value
        self._allow_reuse = bool(self.get_parameter('allow_reuse').value)
        self._bucket_radius = self.get_parameter('bucket_radius').value

        # Gazebo → NED: ned_x=gz_y-sy, ned_y=gz_x-sx
        self._buckets_ned = []
        self._buckets_gz = []
        for i in [1, 2, 3]:
            gx = self.get_parameter(f'bucket{i}_gz_x').value
            gy = self.get_parameter(f'bucket{i}_gz_y').value
            gz = self.get_parameter(f'bucket{i}_gz_z').value
            nx = gy - sy
            ny = gx - sx
            nz = -(gz - sz)
            self._buckets_ned.append((nx, ny))
            self._buckets_gz.append((gx, gy, gz))
            self.get_logger().info(f"Bucket{i} GZ({gx:.2f},{gy:.2f},{gz:.2f}) → NED({nx:.2f},{ny:.2f},{nz:.2f})")

        self._impacts = []  # [(ned_x, ned_y, t), ...]
        self._scored_buckets = {}  # bucket_idx → (dist, score)
        self._drop_count = 0

        self.impact_sub = self.create_subscription(
            Point, '/drop/impact_point', self._cb_impact, 10)

        self.get_logger().info(
            f"Final judge ready: 3 buckets, radius={self._bucket_radius:.3f}m, "
            f"match_max={self._match_max:.1f}m")

    def _score(self, dist):
        if dist <= 0.05:   return 10
        elif dist <= 0.15: return 8
        elif dist <= 0.30: return 6
        elif dist <= 0.50: return 4
        elif dist <= 0.75: return 2
        elif dist <= 1.00: return 1
        else:              return 0

    def _cb_impact(self, msg):
        """接收投放落点 (NED 坐标)."""
        self._drop_count += 1
        ned_x, ned_y = msg.x, msg.y
        self.get_logger().info(f"Impact #{self._drop_count}: NED({ned_x:.3f},{ned_y:.3f})")

        # 找最近的桶 (默认跳过已评分的; allow_reuse=True 时允许同一桶被多次评分)
        best_idx, best_dist = None, float('inf')
        for idx, (bx, by) in enumerate(self._buckets_ned):
            if not self._allow_reuse and idx in self._scored_buckets:
                continue
            d = math.hypot(ned_x - bx, ned_y - by)
            if d < best_dist:
                best_dist, best_idx = d, idx

        if best_idx is not None and best_dist < self._match_max:
            s = self._score(best_dist)
            if self._allow_reuse:
                self._scored_buckets[self._drop_count] = (best_dist, s)   # 键用投数, 允许同桶多次
            else:
                self._scored_buckets[best_idx] = (best_dist, s)
            gx, gy, gz = self._buckets_gz[best_idx]
            self.get_logger().info(
                f"========== RESULT B{best_idx+1}"
                f"{' (第 %d 投)' % self._drop_count if self._allow_reuse else ''} ==========")
            self.get_logger().info(
                f"  距离: {best_dist:.4f}m, 得分: {s}")
            self.get_logger().info(
                f"  落点 NED({ned_x:.4f},{ned_y:.4f}) "
                f"桶 NED({self._buckets_ned[best_idx][0]:.2f},{self._buckets_ned[best_idx][1]:.2f}) "
                f"GZ({gx:.2f},{gy:.2f})")
        else:
            self.get_logger().warn(
                f"  No matching bucket (min_dist={best_dist:.2f}m > {self._match_max:.1f}m)")

        # 打印累计成绩
        if self._scored_buckets:
            total = sum(v[1] for v in self._scored_buckets.values())
            avg_dist = sum(v[0] for v in self._scored_buckets.values()) / len(self._scored_buckets)
            self.get_logger().info(
                f"  Cumulative: {len(self._scored_buckets)} "
                f"{'drops' if self._allow_reuse else 'buckets'}, "
                f"avg_dist={avg_dist:.4f}m, total_score={total}")
        self.get_logger().info(f"=============================")


def main():
    rclpy.init()
    node = FinalJudge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
