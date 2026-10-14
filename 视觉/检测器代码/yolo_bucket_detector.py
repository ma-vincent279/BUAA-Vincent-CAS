#!/usr/bin/env python3
"""
模拟 YOLO 多目标桶检测器 — A3 方案: 中心悬停 + 位置记忆 + 面积排序。
仿真阶段: Ground Truth 投影生成三桶 bbox + 抖动模拟 YOLO 不确定性。
不支持训练好的 .pt 文件时此为仿真替代品；实机时替换 _simulate_yolo_detections() 即可。

状态机:
  INIT      — 收集多帧观测 → 空间聚类 → 按 bbox 面积取 top 2 → TRACKING
  TRACKING  — 反投影匹配已记忆目标位置 → 发布单 ROI 到 /bucket/detection

接口:
  Pub:  /bucket/detection (RegionOfInterest)  单目标 ROI
  Sub:  /bucket/target_id (Int32)             0=最大桶, 1=第二大桶
        /fmu/out/vehicle_local_position_v1    无人机位置+heading
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Int32
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class YoloBucketDetector(Node):
    def __init__(self):
        super().__init__('yolo_bucket_detector')

        # ---- 桶位置参数 (Gazebo 坐标系) ----
        self.declare_parameter('bucket1_gz_x', 35.51)
        self.declare_parameter('bucket1_gz_y', 2.0)
        self.declare_parameter('bucket1_gz_z', 0.15)
        self.declare_parameter('bucket2_gz_x', 37.01)
        self.declare_parameter('bucket2_gz_y', 1.0)
        self.declare_parameter('bucket2_gz_z', 0.15)
        self.declare_parameter('bucket3_gz_x', 35.51)
        self.declare_parameter('bucket3_gz_y', 0.0)
        self.declare_parameter('bucket3_gz_z', 0.15)
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
            {'gz': b1, 'ned': gz_to_ned(*b1), 'radius': 0.075,  'id': 0},
            {'gz': b2, 'ned': gz_to_ned(*b2), 'radius': 0.100,  'id': 1},
            {'gz': b3, 'ned': gz_to_ned(*b3), 'radius': 0.125,  'id': 2},
        ]

        for b in self.buckets:
            self.get_logger().info(
                f"Bucket{b['id']+1} GZ({b['gz'][0]:.1f},{b['gz'][1]:.1f}) "
                f"Φ{2*b['radius']:.2f}m → NED({b['ned'][0]:.2f},{b['ned'][1]:.2f})")

        # ---- 摄像头参数 ----
        self.img_w, self.img_h = 640, 480
        self.hfov = 1.047
        self.vfov = 2.0 * math.atan(math.tan(self.hfov / 2.0) * self.img_h / self.img_w)
        self.fx = self.img_w / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.img_h / (2.0 * math.tan(self.vfov / 2.0))

        # ---- 飞机状态 ----
        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.heading = None

        # ---- 检测器状态机 ----
        self.STATE_INIT = 0
        self.STATE_TRACKING = 1
        self.state = self.STATE_INIT

        # INIT 阶段观测缓冲区: [(ned_x, ned_y, area, bucket_id), ...]
        self._obs_buffer = []
        self._init_frame_count = 0
        self._last_cluster_attempt = 0

        # TRACKING 阶段目标记忆
        # targets = [{'ned': (x, y), 'area': float, 'bucket_id': int}, ...]  按面积降序
        self.targets = []
        self._active = 0  # 0=最大桶, 1=第二大桶

        # 时序滤波
        self._last_best_dist = None
        self._hold_counter = 0
        self._last_roi = None

        # ---- ROS 接口 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local, qos)

        self.det_pub = self.create_publisher(
            RegionOfInterest, '/bucket/detection', 10)

        self.target_sub = self.create_subscription(
            Int32, '/bucket/target_id', self._cb_target, 10)

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "YOLO bucket detector ready — A3: center-hover init + position-memory tracking")

    # ===================== 回调 =====================

    def _cb_local(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading

    def _cb_target(self, msg):
        if 0 <= msg.data < len(self.targets):
            self._active = msg.data
            t = self.targets[self._active]
            self.get_logger().info(
                f"Active target → [{self._active}] bucket{t['bucket_id']+1} "
                f"at NED({t['ned'][0]:.1f},{t['ned'][1]:.1f}) area={t['area']:.0f}px²")
        elif self.state == self.STATE_TRACKING:
            self.get_logger().warn(f"Invalid target_id={msg.data}, have {len(self.targets)} targets")

    # ===================== 主循环 =====================

    def _tick(self):
        if self.heading is None:
            return

        # 仿真 YOLO 多目标检测 (实机时替换为 model(image) 调用)
        detections = self._simulate_yolo_detections()

        if self.state == self.STATE_INIT:
            self._run_init(detections)
        else:
            self._run_tracking(detections)

    # ===================== GT 投影仿真 (实机时删除此方法) =====================

    def _simulate_yolo_detections(self):
        """Ground Truth 投影模拟 YOLO 多目标输出。
        返回: [(px, py, box_w, box_h, bucket_id, ned_x, ned_y), ...]
        每个桶有 10% 概率漏检，bbox 中心 ±4px 抖动，尺寸 ±8px 抖动。
        """
        dets = []
        rng = random.Random()
        for b in self.buckets:
            if rng.random() < 0.0:  # 仿真关闭漏检 (实机 YOLO 99%+ 可靠)
                continue

            bx, by, bz = b['ned']
            dx = bx - self.drone_x
            dy = by - self.drone_y
            dz = bz - self.drone_z

            if dz < 0.3:
                continue

            cos_h = math.cos(self.heading)
            sin_h = math.sin(self.heading)
            body_fwd = dx * cos_h + dy * sin_h
            body_right = -dx * sin_h + dy * cos_h

            px = self.img_w / 2.0 + (body_right / dz) * self.fx
            py = self.img_h / 2.0 - (body_fwd / dz) * self.fy  # 下视摄像头: TOP=前
            box_w = 2.0 * self.fx * b['radius'] / dz
            box_h = 2.0 * self.fy * b['radius'] / dz

            # 模拟 YOLO 抖动
            px += rng.gauss(0, 4)
            py += rng.gauss(0, 4)
            box_w += rng.gauss(0, 6)
            box_h += rng.gauss(0, 6)
            box_w = max(4, min(200, box_w))
            box_h = max(4, min(200, box_h))

            if px + box_w/2 < 0 or px - box_w/2 > self.img_w:
                continue
            if py + box_h/2 < 0 or py - box_h/2 > self.img_h:
                continue

            # ned 反投影 (用无噪声值，后续用于匹配)
            dz_clean = bz - self.drone_z
            body_r_clean = (px - self.img_w/2) * dz_clean / self.fx
            body_f_clean = (py - self.img_h/2) * dz_clean / self.fy
            ned_x = self.drone_x + body_f_clean * cos_h - body_r_clean * sin_h
            ned_y = self.drone_y + body_f_clean * sin_h + body_r_clean * cos_h

            dets.append((px, py, box_w, box_h, b['id'], ned_x, ned_y))

        return dets

    # ===================== INIT 阶段 =====================

    def _run_init(self, detections):
        """收集多帧观测，空间聚类，按面积选 top 2。"""
        self._init_frame_count += 1

        for px, py, bw, bh, bid, nx, ny in detections:
            self._obs_buffer.append((nx, ny, bw * bh, bid))

        # 每 40 帧 (~2s) 尝试聚类一次
        if self._init_frame_count - self._last_cluster_attempt < 40:
            return
        self._last_cluster_attempt = self._init_frame_count

        if len(self._obs_buffer) < 20:  # 至少 3 个桶 × ~7 帧
            return

        clusters = self._cluster_observations(self._obs_buffer, eps=1.5)

        if len(clusters) < 3:
            self.get_logger().info(
                f"INIT: {len(clusters)} clusters (need ≥3), "
                f"obs={len(self._obs_buffer)} frames={self._init_frame_count}")
            return

        # 每簇: (avg_area, avg_ned_x, avg_ned_y, most_common_bucket_id)
        scored = []
        for indices in clusters:
            areas = [self._obs_buffer[i][2] for i in indices]
            xs = [self._obs_buffer[i][0] for i in indices]
            ys = [self._obs_buffer[i][1] for i in indices]
            bids = [self._obs_buffer[i][3] for i in indices]
            scored.append((
                sum(areas) / len(areas),
                sum(xs) / len(xs),
                sum(ys) / len(ys),
                max(set(bids), key=bids.count),
            ))

        scored.sort(key=lambda s: s[0], reverse=True)
        self.targets = [
            {'ned': (s[1], s[2]), 'area': s[0], 'bucket_id': s[3]}
            for s in scored[:2]
        ]

        self.state = self.STATE_TRACKING
        self._obs_buffer.clear()

        self.get_logger().info("======== INIT COMPLETE ========")
        for i, t in enumerate(self.targets):
            rad = self.buckets[t['bucket_id']]['radius']
            self.get_logger().info(
                f"  Target[{i}]: bucket{t['bucket_id']+1} Φ{2*rad:.2f}m "
                f"at NED({t['ned'][0]:.2f},{t['ned'][1]:.2f}) area={t['area']:.0f}px²")
        self.get_logger().info(f"  => Will drop bucket{self.targets[0]['bucket_id']+1} then bucket{self.targets[1]['bucket_id']+1}")

    def _cluster_observations(self, obs, eps):
        """简单空间聚类: 两点 NED 距离 < eps 即归为同簇。返回 [[idx, ...], ...]。"""
        n = len(obs)
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj

        # O(n²) 对少量观测足够
        for i in range(n):
            for j in range(i + 1, n):
                dx = obs[i][0] - obs[j][0]
                dy = obs[i][1] - obs[j][1]
                if dx * dx + dy * dy < eps * eps:
                    union(i, j)

        clusters = {}
        for i in range(n):
            root = find(i)
            clusters.setdefault(root, []).append(i)

        # 过滤: 每簇至少 3 个点
        return [v for v in clusters.values() if len(v) >= 3]

    # ===================== TRACKING 阶段 =====================

    def _run_tracking(self, detections):
        """位置记忆匹配: 反投影每个 bbox → 选离 active target 最近的 → 发 ROI。
        加入时序滤波: 若最佳匹配突然跳变 (>1.5m), 短暂保持上帧结果防止误匹配。"""
        if not self.targets:
            return

        tx, ty = self.targets[self._active]['ned']
        best_dist = float('inf')
        best = None

        for px, py, bw, bh, bid, nx, ny in detections:
            d = math.hypot(nx - tx, ny - ty)
            if d < best_dist:
                best_dist = d
                best = (px, py, bw, bh)

        if best is None or best_dist > 2.0:  # 2m 重捕获半径 (排除相邻桶误匹配)
            self._hold_counter += 1
            if self._hold_counter > 10:  # 0.5s 后放弃保持
                self._last_roi = None
            if self._last_roi is not None:
                self.det_pub.publish(self._last_roi)
            return

        # 时序滤波: 若匹配距离突然跳变 (>1.5m) 且超过 2 帧, 短暂保持
        if self._last_best_dist is not None and best_dist > self._last_best_dist + 1.5:
            self._hold_counter += 1
            if self._hold_counter < 6 and self._last_roi is not None:
                self.det_pub.publish(self._last_roi)
                return
        else:
            self._hold_counter = 0

        self._last_best_dist = best_dist

        px, py, bw, bh = best
        roi = RegionOfInterest()
        roi.x_offset = max(0, int(px - bw / 2))
        roi.y_offset = max(0, int(py - bh / 2))
        roi.width = min(self.img_w - roi.x_offset, int(bw))
        roi.height = min(self.img_h - roi.y_offset, int(bh))
        roi.do_rectify = False
        self.det_pub.publish(roi)
        self._last_roi = roi


def main(args=None):
    rclpy.init(args=args)
    node = YoloBucketDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
