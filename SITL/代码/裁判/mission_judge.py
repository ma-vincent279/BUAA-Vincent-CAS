#!/usr/bin/env python3
"""
全任务统一裁判节点 — 投放 + 侦察 + 着陆 + 计时 全部在一条 log 打印总结。
竞赛场地 competition_field.world 适配版。
  - 投放分: 细化计分 10/8/6/4/2/1/0, 两枚分别计 (0-20)
  - 侦察分: 正确识别有标识得 1分, 空筒 0分, 总分 0-3
  - 着陆分: 降落在起降点圆内 (d<2.5m) → 10分, 否则 0分
  - 计时: 从 arming_state 2→1 的间隔 (arm→disarm)
"""

import math
import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class MissionJudge(Node):
    def __init__(self):
        super().__init__('mission_judge')

        # ---- ROS 参数 ----
        # 投放桶 Gazebo 坐标 (competition_field 旋转+平移后)
        self.declare_parameter('bucket1_gz_x', 35.51)
        self.declare_parameter('bucket1_gz_y', 2.0)
        self.declare_parameter('bucket1_gz_z', 0.15)
        self.declare_parameter('bucket2_gz_x', 37.01)
        self.declare_parameter('bucket2_gz_y', 1.0)
        self.declare_parameter('bucket2_gz_z', 0.15)
        self.declare_parameter('bucket3_gz_x', 35.51)
        self.declare_parameter('bucket3_gz_y', 0.0)
        self.declare_parameter('bucket3_gz_z', 0.15)
        self.declare_parameter('landing_gz_x', 1.01)
        self.declare_parameter('landing_gz_y', 1.0)
        self.declare_parameter('landing_radius', 2.5)
        self.declare_parameter('spawn_gz_x', 1.01)
        self.declare_parameter('spawn_gz_y', 0.98)

        self.buckets = [
            (self.get_parameter('bucket1_gz_x').value,
             self.get_parameter('bucket1_gz_y').value),
            (self.get_parameter('bucket2_gz_x').value,
             self.get_parameter('bucket2_gz_y').value),
            (self.get_parameter('bucket3_gz_x').value,
             self.get_parameter('bucket3_gz_y').value),
        ]
        self.landing_gz_x = self.get_parameter('landing_gz_x').value
        self.landing_gz_y = self.get_parameter('landing_gz_y').value
        self.landing_radius = self.get_parameter('landing_radius').value
        self.spawn_gz_x = self.get_parameter('spawn_gz_x').value
        self.spawn_gz_y = self.get_parameter('spawn_gz_y').value

        # 着陆区 NED 坐标 (Gazebo → NED 转换)
        self.landing_ned_x = self.landing_gz_y - self.spawn_gz_y
        self.landing_ned_y = self.landing_gz_x - self.spawn_gz_x

        # ---- 侦察 ground truth (随机预设) ----
        # 5 个筒中随机选 3 个有标识，类型随机
        all_indices = list(range(5))
        random.shuffle(all_indices)
        self._marker_indices = sorted(all_indices[:3])
        marker_types = ['火焰', '腐蚀', '有毒']
        random.shuffle(marker_types)
        self._marker_types = {}
        for i, idx in enumerate(self._marker_indices):
            self._marker_types[idx] = marker_types[i]

        self.get_logger().info(
            f"Scout ground truth: markers at cylinders {self._marker_indices}")
        for idx in self._marker_indices:
            self.get_logger().info(
                f"  Cylinder {idx}: {self._marker_types[idx]}")

        # ---- 状态 ----
        self._arm_time = None
        self._disarm_time = None
        self._last_x = self._last_y = 0.0
        self._last_status = None

        # 投放
        self._drop_score = 0
        self._drop_count = 0
        self._judged_payloads = set()
        self._payload_data = {}  # name → {'history': [(t,z),...], 'pose'}

        # 侦察
        self._scout_score = 0
        self._scout_count = 0

        # 着陆
        self._landing_score = 0
        self._landing_scored = False

        # ---- QoS ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        # ---- Sub ----
        self.model_sub = self.create_subscription(
            ModelStates, '/model_states', self._cb_models, 10)
        self.scout_reached_sub = self.create_subscription(
            Int32, '/scout/point_reached', self._cb_scout_reached, 10)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._cb_status, qos)
        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._cb_local, qos)

        # ---- Pub ----
        self.result_pub = self.create_publisher(String, '/scout/result', 10)

        # ---- Timer ----
        self.timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"Mission judge ready: buckets at "
            f"({self.buckets[0][0]:.1f},{self.buckets[0][1]:.1f}), "
            f"({self.buckets[1][0]:.1f},{self.buckets[1][1]:.1f}), "
            f"({self.buckets[2][0]:.1f},{self.buckets[2][1]:.1f}), "
            f"landing zone at GZ({self.landing_gz_x:.1f},{self.landing_gz_y:.1f}) "
            f"→ NED({self.landing_ned_x:.2f},{self.landing_ned_y:.2f}) "
            f"radius={self.landing_radius}m, "
            f"scout markers={self._marker_indices}")

    # ===================== 回调 =====================

    def _cb_models(self, m):
        now = self.get_clock().now()
        for name, pose in zip(m.name, m.pose):
            if not name.startswith('dropped_payload_') or name in self._judged_payloads:
                continue
            if name not in self._payload_data:
                self._payload_data[name] = {'history': [], 'pose': None}
                self.get_logger().info(f"Tracking {name}...")
            self._payload_data[name]['history'].append((now, pose.position.z))
            self._payload_data[name]['pose'] = pose
            cutoff = now - rclpy.time.Duration(seconds=3)
            self._payload_data[name]['history'] = [
                (t, z) for t, z in self._payload_data[name]['history'] if t > cutoff]

    def _cb_scout_reached(self, msg):
        idx = msg.data
        if not (0 <= idx < 5):
            self.get_logger().warn(f"Invalid scout index {idx}")
            return

        self._scout_count += 1
        if idx in self._marker_indices:
            mtype = self._marker_types[idx]
            text = f"筒{idx}: 有标识({mtype})"
            self._scout_score += 1
        else:
            text = f"筒{idx}: 空"
        self.get_logger().info(f">>> {text} <<< (scout_score={self._scout_score}/{self._scout_count})")
        result = String()
        result.data = text
        self.result_pub.publish(result)

    def _cb_status(self, m):
        if self._last_status is not None:
            if self._last_status.arming_state != 2 and m.arming_state == 2:
                self._arm_time = self.get_clock().now()
                self.get_logger().info(">>> TIMER START (armed) <<<")
            elif self._last_status.arming_state == 2 and m.arming_state != 2:
                self._disarm_time = self.get_clock().now()
                self.get_logger().info(">>> TIMER STOP (disarmed) <<<")
                self._judge_landing()
        self._last_status = m

    def _cb_local(self, m):
        self._last_x, self._last_y = m.x, m.y

    # ===================== 判定 =====================

    def _tick(self):
        now = self.get_clock().now()
        for name, data in list(self._payload_data.items()):
            if name in self._judged_payloads:
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

            # 落点判定：找最近的桶
            pose = data['pose']
            gx, gy = pose.position.x, pose.position.y
            best_dist = float('inf')
            best_bucket = None
            for i, (bx, by) in enumerate(self.buckets):
                d = math.hypot(gx - bx, gy - by)
                if d < best_dist:
                    best_dist = d
                    best_bucket = i

            if best_dist <= 0.05:
                zone, score = "10分", 10
            elif best_dist <= 0.15:
                zone, score = "8分", 8
            elif best_dist <= 0.30:
                zone, score = "6分", 6
            elif best_dist <= 0.50:
                zone, score = "4分", 4
            elif best_dist <= 0.75:
                zone, score = "2分", 2
            elif best_dist <= 1.00:
                zone, score = "1分", 1
            else:
                zone, score = "Miss", 0

            self._drop_score += score
            self._drop_count += 1
            self._judged_payloads.add(name)
            self.get_logger().info(
                f"[{name}] gz({gx:.3f},{gy:.3f}) → bucket{best_bucket+1} "
                f"dist={best_dist:.3f}m → {zone} | "
                f"投放累计: {self._drop_score}分/{self._drop_count}枚")
            del self._payload_data[name]

    def _judge_landing(self):
        if self._landing_scored:
            return
        self._landing_scored = True
        # 距着陆区 NED 中心的水平距离
        dist = math.hypot(self._last_x - self.landing_ned_x,
                          self._last_y - self.landing_ned_y)
        if dist < self.landing_radius:
            self._landing_score = 10
            self.get_logger().info(f">>> 着陆判定: 起降点内 ({dist:.3f}m < {self.landing_radius}m) → 10分 <<<")
        else:
            self._landing_score = 0
            self.get_logger().info(f">>> 着陆判定: 起降点外 ({dist:.3f}m ≥ {self.landing_radius}m) → 0分 <<<")

        self._print_final_score()

    def _print_final_score(self):
        dur = 0.0
        if self._arm_time and self._disarm_time:
            dur = (self._disarm_time - self._arm_time).nanoseconds * 1e-9

        total = self._drop_score + self._scout_score + self._landing_score
        self.get_logger().info(
            f"======= MISSION SCORE =======\n"
            f"  总时间:   {dur:.1f}s\n"
            f"  投放分:   {self._drop_score}/20 (10/8/6/4/2/1/0)\n"
            f"  侦察分:   {self._scout_score}/3 (正确识别有标识)\n"
            f"  着陆分:   {self._landing_score}/10 (着陆区半径{self.landing_radius}m内)\n"
            f"  ─────────────────────────\n"
            f"  总分:     {total}/33\n"
            f"==============================")


def main():
    rclpy.init()
    rclpy.spin(MissionJudge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
