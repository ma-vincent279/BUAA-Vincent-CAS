#!/usr/bin/env python3
"""
检测噪声注入器 — 透明代理，模拟 YOLO 检测不稳定性。

管道: detector → /bucket/detection_raw → [本节点] → /bucket/detection → mission

通过 ROS2 参数控制噪声类型和强度:
  drop_rate          — 随机丢帧概率 (0.0-1.0)
  drop_burst_max     — 连续丢帧最大帧数 (1-30)
  false_pos_rate     — 每秒假阳性出现概率 (0.0-1.0)
  false_pos_duration — 假阳性持续时间 (s)
  false_pos_jitter   — 假阳性 ROI 随机跳动范围 (px)
  jitter_px          — 真检测 ROI 抖动幅度 (px)
  multi_detect_rate  — 用假检测替换真检测的概率 (0.0-1.0)
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest


class DetectionNoiseInjector(Node):
    def __init__(self):
        super().__init__('detection_noise_injector')

        # --- ROS2 可调参数 ---
        self.declare_parameter('drop_rate', 0.0)
        self.declare_parameter('drop_burst_max', 5)
        self.declare_parameter('false_pos_rate', 0.0)
        self.declare_parameter('false_pos_duration', 0.8)
        self.declare_parameter('false_pos_jitter', 100.0)
        self.declare_parameter('jitter_px', 0.0)
        self.declare_parameter('multi_detect_rate', 0.0)

        self._drop_rate = self.get_parameter('drop_rate').value
        self._drop_burst_max = self.get_parameter('drop_burst_max').value
        self._false_pos_rate = self.get_parameter('false_pos_rate').value
        self._false_pos_duration = self.get_parameter('false_pos_duration').value
        self._false_pos_jitter = self.get_parameter('false_pos_jitter').value
        self._jitter_px = self.get_parameter('jitter_px').value
        self._multi_detect_rate = self.get_parameter('multi_detect_rate').value

        # --- 内部状态 ---
        self._latest_real_roi = None
        self._last_real_time = self.get_clock().now()
        self._drop_burst_count = 0       # 当前连续丢帧计数
        self._last_dropped = False

        # 假阳性状态
        self._false_pos_active = False
        self._false_pos_start = None
        self._false_pos_roi = None

        # 假阳性定时器 (高频触发，每秒检查是否该生成)
        self._fp_check_hz = 20.0
        self._fp_timer = self.create_timer(1.0 / self._fp_check_hz, self._fp_tick)

        # --- Pub / Sub ---
        self.pub = self.create_publisher(RegionOfInterest, '/bucket/detection', 10)
        self.sub = self.create_subscription(
            RegionOfInterest, '/bucket/detection_raw', self._cb_raw, 10)

        self.get_logger().info(
            f"Noise Injector ready. "
            f"drop={self._drop_rate:.2f}(burst<={self._drop_burst_max}) "
            f"fp={self._false_pos_rate:.2f}/s(fp_dur={self._false_pos_duration:.1f}s) "
            f"jitter={self._jitter_px:.0f}px multi={self._multi_detect_rate:.2f}")

    def _cb_raw(self, m):
        self._latest_real_roi = m
        self._last_real_time = self.get_clock().now()

    def _is_real_fresh(self):
        if self._latest_real_roi is None:
            return False
        dt = (self.get_clock().now() - self._last_real_time).nanoseconds * 1e-9
        return dt < 3.0

    def _jitter_roi(self, roi, jitter):
        """对 ROI 中心施加随机像素抖动"""
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        cx += random.uniform(-jitter, jitter)
        cy += random.uniform(-jitter, jitter)
        # 裁剪到图像内
        cx = max(0, min(639, cx))
        cy = max(0, min(479, cy))
        w2, h2 = roi.width / 2.0, roi.height / 2.0
        x = max(0, int(cx - w2))
        y = max(0, int(cy - h2))
        w = min(640 - x, roi.width)
        h = min(480 - y, roi.height)
        if w <= 0 or h <= 0:
            return None
        out = RegionOfInterest()
        out.x_offset = x
        out.y_offset = y
        out.width = w
        out.height = h
        out.do_rectify = roi.do_rectify
        return out

    def _gen_false_roi(self, base_roi=None):
        """生成随机位置的假 ROI"""
        if base_roi is not None:
            cx = base_roi.x_offset + base_roi.width / 2.0
            cy = base_roi.y_offset + base_roi.height / 2.0
            w = base_roi.width
            h = base_roi.height
        else:
            cx = random.uniform(100, 540)
            cy = random.uniform(80, 400)
            w = random.randint(30, 80)
            h = random.randint(30, 80)

        cx += random.uniform(-self._false_pos_jitter, self._false_pos_jitter)
        cy += random.uniform(-self._false_pos_jitter, self._false_pos_jitter)
        cx = max(0, min(639, cx))
        cy = max(0, min(479, cy))

        x = max(0, int(cx - w / 2.0))
        y = max(0, int(cy - h / 2.0))
        w = min(640 - x, int(w))
        h = min(480 - y, int(h))
        if w <= 0 or h <= 0:
            return None
        out = RegionOfInterest()
        out.x_offset = x
        out.y_offset = y
        out.width = w
        out.height = h
        out.do_rectify = False
        return out

    def _should_drop(self):
        """决定是否丢帧，支持连续丢帧 (burst)"""
        if self._drop_rate <= 0:
            return False
        if self._drop_burst_count > 0:
            self._drop_burst_count -= 1
            return True
        if random.random() < self._drop_rate:
            self._drop_burst_count = random.randint(0, self._drop_burst_max - 1)
            return True
        return False

    def _fp_tick(self):
        """20Hz 定时器：噪声注入 + 转发"""
        now = self.get_clock().now()

        # --- 假阳性生命周期管理 ---
        if self._false_pos_active:
            elapsed = (now - self._false_pos_start).nanoseconds * 1e-9
            if elapsed >= self._false_pos_duration:
                self._false_pos_active = False
                self._false_pos_start = None
                self._false_pos_roi = None
                self.get_logger().info("FP ended")

        if not self._false_pos_active and self._false_pos_rate > 0:
            # 每秒有 false_pos_rate 概率触发一个新假阳性
            p_per_tick = self._false_pos_rate / self._fp_check_hz
            if random.random() < p_per_tick:
                self._false_pos_active = True
                self._false_pos_start = now
                self._false_pos_roi = self._gen_false_roi(self._latest_real_roi)
                self.get_logger().warn(
                    f"FP started: ROI({self._false_pos_roi.x_offset},{self._false_pos_roi.y_offset},"
                    f"{self._false_pos_roi.width},{self._false_pos_roi.height}) "
                    f"dur={self._false_pos_duration:.1f}s")

        # --- 决定发布什么 ---
        # 优先级: 假阳性 > 真检测(含丢帧/抖动/误替换)

        if self._false_pos_active and self._false_pos_roi is not None:
            # 假阳性活跃: 每 tick 抖动假 ROI
            fp = self._gen_false_roi(self._latest_real_roi)
            if fp is not None:
                self.pub.publish(fp)
            return

        # 假阳性不活跃, 处理真检测
        if not self._is_real_fresh():
            return  # 真检测超时, 不发

        if self._should_drop():
            return  # 丢帧

        roi = self._latest_real_roi

        # 多检测误替换: 偶尔用假 ROI 替换真检测
        if self._multi_detect_rate > 0 and random.random() < self._multi_detect_rate:
            fake = self._gen_false_roi(roi)
            if fake is not None:
                roi = fake

        # 像素抖动
        if self._jitter_px > 0:
            jittered = self._jitter_roi(roi, self._jitter_px)
            if jittered is not None:
                self.pub.publish(jittered)
        else:
            self.pub.publish(roi)


def main():
    rclpy.init()
    node = DetectionNoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
