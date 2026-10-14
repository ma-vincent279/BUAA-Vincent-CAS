#!/usr/bin/env python3
"""
YOLOv8 桶检测器（真机版）— sim/real 共享 YOLOv8 推理链路，发布 /bucket/detection。

real 模式 (use_sim:=false，默认):
  真实摄像头图像 → YOLOv8 推理 → ROI 发布。
  支持两种摄像头接入方式：
    1. ROS 2 话题订阅（需 camera_topic）
    2. 直读摄像头（需 camera_device，如 /dev/video0）

sim 模式 (use_sim:=true):
  GT 投影 → 合成俯视图像 → YOLOv8 推理 → ROI 发布。

ROS 参数:
  use_sim            是否使用仿真合成图像（默认 false）
  model_path         YOLOv8 模型路径（TensorRT 引擎）
  camera_topic       摄像头话题（仅 real 且 camera_device 为空时）
  camera_device      直读摄像头设备路径（如 /dev/video0）
  conf_threshold     置信度阈值（默认 0.5）
  target_class       目标类别名（默认 "bucket"）
  infer_hz           推理频率上限（默认 10Hz）
  bucket_gz_x/y/z    桶 Gazebo 坐标（仅 sim）
  spawn_gz_x/y/z     飞机生成点（仅 sim）
  img_width/height   图像尺寸（默认 640x480）
  visualize          弹出可视化窗口（默认 false）
"""

import math
import time
import threading
import numpy as np
import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest, Image
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class YOLOv8BucketDetector(Node):
    def __init__(self):
        super().__init__('yolov8_bucket_detector')

        # ---- 参数 ----
        self.declare_parameter('use_sim', False)                     # 真机模式
        self.declare_parameter('model_path',
                               os.path.join(os.path.expanduser('~'), 'c.engine'))
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_device', '/dev/video0')       # 直读摄像头
        self.declare_parameter('conf_threshold', 0.65)
        self.declare_parameter('target_class', 'bucket')
        self.declare_parameter('infer_hz', 10.0)
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 480)
        self.declare_parameter('visualize', True)

        # sim 参数（保留但不影响真机模式）
        self.declare_parameter('bucket_gz_x', 6.01)
        self.declare_parameter('bucket_gz_y', 0.98)
        self.declare_parameter('bucket_gz_z', 0.25)
        self.declare_parameter('spawn_gz_x', 1.01)
        self.declare_parameter('spawn_gz_y', 0.98)
        self.declare_parameter('spawn_gz_z', 0.83)

        self.img_w = self.get_parameter('img_width').value
        self.img_h = self.get_parameter('img_height').value
        self._use_sim = self.get_parameter('use_sim').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.infer_hz = self.get_parameter('infer_hz').value
        self.visualize = self.get_parameter('visualize').value

        # ---- 加载 YOLOv8 模型 (sim / real 共用) ----
        self._load_model()

        # ---- 检测输出 ----
        self.det_pub = self.create_publisher(
            RegionOfInterest, '/bucket/detection', 10)

        # ---- 缓存上次检测 + 推理节流 ----
        self._last_roi = None
        self._last_infer_time = 0.0
        self._infer_interval = 1.0 / self.infer_hz

        # ---- 按模式初始化 ----
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)

        if self._use_sim:
            self._init_sim(qos_be)
        else:
            self._init_real()

        mode_str = "SIM (synthetic img + YOLOv8)" if self._use_sim else "REAL (camera + YOLOv8)"
        self.get_logger().info(
            f"Ready [{mode_str}] model={self._model_names} "
            f"conf={self.conf_threshold} infer={self.infer_hz}Hz "
            f"image={self.img_w}x{self.img_h}")

    # ======================== 模型加载 (共用) ========================

    def _load_model(self):
        model_path = self.get_parameter('model_path').value
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)  # 支持 .pt 和 .engine
        except ImportError:
            self.get_logger().fatal(
                "ultralytics not installed. "
                "Workstation: pip3 install ultralytics. "
                "Jetson: conda activate <env>")
            raise

        # 预热推理 (首次推理可能耗时较长)
        import time as _time
        _t0 = _time.time()
        dummy = np.random.randint(0, 255, (self.img_h, self.img_w, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        self.get_logger().info(
            f"Model warm-up: {(_time.time() - _t0)*1000:.0f}ms")

        self._model_names = getattr(self.model, 'names', {})
        self._target_cls_idx = None
        if not self._model_names:
            self.get_logger().error(
                "Model has no class names! "
                "TensorRT .engine may have been exported without metadata. "
                "ALL detections will be treated as target — this is dangerous!")
        elif self.target_class:
            for idx, name in self._model_names.items():
                if name == self.target_class:
                    self._target_cls_idx = idx
                    break
            if self._target_cls_idx is not None:
                self.get_logger().info(
                    f"Target '{self.target_class}' -> cls_idx={self._target_cls_idx}")
            else:
                self.get_logger().warn(
                    f"Class '{self.target_class}' not in model {list(self._model_names.values())}, "
                    f"falling back to any detection")

    # ======================== 推理 + 检测提取 (共用) ========================

    def _infer_and_publish(self, bgr_image):
        """对图像做 YOLOv8 推理，提取最佳检测框并发布。"""
        now = time.time()
        if now - self._last_infer_time < self._infer_interval:
            if self._last_roi is not None:
                self.det_pub.publish(self._last_roi)
            return

        self._last_infer_time = now
        try:
            results = self.model(bgr_image, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=5)
            return

        roi = self._best_detection(results)
        self._last_roi = roi
        if roi is not None:
            self.det_pub.publish(roi)

    def _best_detection(self, results):
        """从 YOLOv8 结果中找最佳匹配 — 向量化 + 局部变量缓存。"""
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        conf = boxes.conf.cpu().numpy().ravel()
        cls_arr = boxes.cls.cpu().numpy().ravel()

        # 局部变量缓存，避免重复 self. 属性查找
        thresh = self.conf_threshold
        target_cls = self._target_cls_idx
        imw = self.img_w
        imh = self.img_h

        mask = conf >= thresh
        if target_cls is not None:
            mask &= (cls_arr == target_cls)
        if not mask.any():
            return None

        xyxy = boxes.xyxy.cpu().numpy()
        valid_idx = np.where(mask)[0]
        conf_v = conf[valid_idx]
        areas = (xyxy[valid_idx, 2] - xyxy[valid_idx, 0]) * (xyxy[valid_idx, 3] - xyxy[valid_idx, 1])

        # 负置信度 + 负面积 → lexsort 取最大置信度/最大面积
        best_i = np.lexsort((-areas, -conf_v))[0]
        idx = valid_idx[best_i]

        x1 = max(0, int(xyxy[idx, 0]))
        y1 = max(0, int(xyxy[idx, 1]))
        x2 = min(imw - 1, int(xyxy[idx, 2]))
        y2 = min(imh - 1, int(xyxy[idx, 3]))
        if x2 <= x1 or y2 <= y1:
            return None

        roi = RegionOfInterest()
        roi.x_offset = x1
        roi.y_offset = y1
        roi.width = x2 - x1
        roi.height = y2 - y1
        roi.do_rectify = False
        return roi

    # ======================== sim 模式 ========================

    def _init_sim(self, qos_be):
        sgx = self.get_parameter('spawn_gz_x').value
        sgy = self.get_parameter('spawn_gz_y').value
        sgz = self.get_parameter('spawn_gz_z').value
        bgx = self.get_parameter('bucket_gz_x').value
        bgy = self.get_parameter('bucket_gz_y').value
        bgz = self.get_parameter('bucket_gz_z').value

        self.bucket_ned = (bgy - sgy, bgx - sgx, -(bgz - sgz))
        self.bucket_radius = 0.125

        self.hfov = 1.047
        self.vfov = 2.0 * math.atan(
            math.tan(self.hfov / 2.0) * self.img_h / self.img_w)
        self.fx = self.img_w / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.img_h / (2.0 * math.tan(self.vfov / 2.0))

        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.heading = None

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local_sim, qos_be)

        self.syn_img_pub = self.create_publisher(
            Image, '/bucket/synthetic_image', 10)

        self.timer = self.create_timer(1.0 / self.infer_hz, self._tick_sim)

        self._syn_bg = self._make_grass_bg()
        self.get_logger().info("SIM grass background pre-generated")

        self.get_logger().info(
            f"SIM bucket NED({self.bucket_ned[0]:.2f},"
            f"{self.bucket_ned[1]:.2f},{self.bucket_ned[2]:.2f}) "
            f"r={self.bucket_radius:.2f}m")

    def _cb_local_sim(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading

    def _tick_sim(self):
        if self.heading is None:
            return
        bx, by, bz = self.bucket_ned
        dx = bx - self.drone_x
        dy = by - self.drone_y
        dz = bz - self.drone_z
        if dz < 0.3:
            return
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        body_fwd = dx * cos_h + dy * sin_h
        body_right = -dx * sin_h + dy * cos_h
        if body_fwd < -0.5:
            return
        px = self.img_w / 2.0 + (body_right / dz) * self.fx
        py = self.img_h / 2.0 + (body_fwd / dz) * self.fy
        radius_px = self.fx * self.bucket_radius / dz
        if px < -50 or px > self.img_w + 50 or py < -50 or py > self.img_h + 50:
            return
        syn_img = self._gen_synthetic(px, py, max(3.0, radius_px))
        self._infer_and_publish(syn_img)

        if self.visualize:
            if self._last_roi is not None:
                r = self._last_roi
                cv2.rectangle(syn_img,
                              (r.x_offset, r.y_offset),
                              (r.x_offset + r.width, r.y_offset + r.height),
                              (0, 255, 0), 2)
            cv2.circle(syn_img, (int(px), int(py)), int(radius_px), (0, 0, 255), 1)
            cv2.putText(syn_img, "SIM", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('YOLOv8 Detector (SIM)', syn_img)
            cv2.waitKey(1)

    def _make_grass_bg(self):
        rng = np.random.RandomState(42)
        img = rng.randint(80, 150, (self.img_h, self.img_w, 3)).astype(np.uint8)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        return img

    def _gen_synthetic(self, cx, cy, radius_px):
        img = self._syn_bg.copy()
        cx_i, cy_i = int(round(cx)), int(round(cy))
        r_i = max(25, min(int(round(radius_px)), 120))
        cv2.circle(img, (cx_i, cy_i), r_i, (230, 230, 230), -1)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        return img

    # ======================== real 模式 ========================

    def _init_real(self):
        camera_device = self.get_parameter('camera_device').value

        if camera_device:
            self._cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                self.get_logger().fatal(f"Cannot open camera: {camera_device}")
                raise RuntimeError(f"Cannot open camera: {camera_device}")
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_w)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
            self.get_logger().info(
                f"Camera opened: {camera_device} "
                f"({self._cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
                f"{self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f})")

            # 线程化相机: 后台持续读帧, 定时器回调取最新帧 (零阻塞)
            self._frame = None
            self._frame_lock = threading.Lock()
            self._cam_running = True
            self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self._cam_thread.start()

            self._cam_timer = self.create_timer(
                1.0 / self.infer_hz, self._tick_camera)
        else:
            camera_topic = self.get_parameter('camera_topic').value
            self.camera_sub = self.create_subscription(
                Image, camera_topic, self._cb_image, 10)
            self.get_logger().info(f"Subscribed to camera topic: {camera_topic}")

    def _camera_loop(self):
        """后台线程: 持续读取摄像头, 保证最新帧立即可用。"""
        while self._cam_running:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._frame_lock:
                    self._frame = frame

    def _tick_camera(self):
        # 取最新帧 (无阻塞)
        with self._frame_lock:
            frame = self._frame
        if frame is None:
            return

        # 直接推理 (定时器已节流, 无需重复检查)
        self._last_infer_time = time.time()
        try:
            results = self.model(frame, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=5)
            return

        roi = self._best_detection(results)
        self._last_roi = roi
        if roi is not None:
            self.det_pub.publish(roi)

        # 可视化 — 直接在原帧上画, 省掉 copy()
        if self.visualize:
            if roi is not None:
                cv2.rectangle(frame,
                              (roi.x_offset, roi.y_offset),
                              (roi.x_offset + roi.width, roi.y_offset + roi.height),
                              (0, 255, 0), 2)
            cv2.putText(frame, "REAL (direct)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('YOLOv8 Detector (REAL)', frame)
            cv2.waitKey(1)

    def _cb_image(self, msg):
        try:
            h, w = msg.height, msg.width
            if msg.encoding == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif msg.encoding == 'mono8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            else:
                self.get_logger().warn(
                    f"Encoding {msg.encoding} unsupported", throttle_duration_sec=5)
                return
        except Exception as e:
            self.get_logger().error(f"Decode error: {e}", throttle_duration_sec=5)
            return

        self._infer_and_publish(arr)

        if self.visualize:
            if self._last_roi is not None:
                r = self._last_roi
                cv2.rectangle(arr,
                              (r.x_offset, r.y_offset),
                              (r.x_offset + r.width, r.y_offset + r.height),
                              (0, 255, 0), 2)
            cv2.putText(arr, "REAL (topic)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('YOLOv8 Detector (REAL)', arr)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8BucketDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_cam_running'):
            node._cam_running = False
        if hasattr(node, '_cap') and node._cap is not None:
            node._cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()