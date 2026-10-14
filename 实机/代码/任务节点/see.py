#!/usr/bin/env python3
"""
多桶检测器 see.py — YOLOv8 (TensorRT) 真机推理, 发布所有检测框到 /bucket/detections.
配合 a.py / final_mission.py 等多桶任务节点使用。

与 yolo.py 的区别:
- yolo.py: 只选一个最佳检测框 → /bucket/detection (RegionOfInterest, 单目标)
- see.py:  发布所有检测框 → /bucket/detections (String JSON, 多目标)
  JSON 格式: {"buckets": [{"x": x1, "y": y1, "w": w, "h": h}, ...]}

ROS 参数:
  model_path         YOLOv8 模型路径 (TensorRT .engine, 默认 balloon.engine)
  camera_device      直读摄像头设备路径 (如 /dev/video0, V4L2+MJPG)
  conf_threshold     置信度阈值 (默认 0.4, 实机建议 0.4~0.65)
  target_class       目标类别名 (默认 "balloon", 用于过滤无关检测)
  infer_hz           推理频率上限 (默认 10Hz, 跳帧时重发缓存 JSON)
  img_width/height   推理输入尺寸 (默认 640x360, 与 a.py 相机参数一致)
  visualize          弹出 OpenCV 可视化窗口 (默认 true)
"""

import json
import math
import time
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class MultiBucketDetector(Node):
    def __init__(self):
        super().__init__('see')

        # ---- 参数 ----
        self.declare_parameter('model_path',
                               '/home/nvidia/balloon.engine')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('target_class', 'balloon')
        self.declare_parameter('infer_hz', 10.0)
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 360)
        self.declare_parameter('visualize', True)

        self.img_w = self.get_parameter('img_width').value
        self.img_h = self.get_parameter('img_height').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.infer_hz = self.get_parameter('infer_hz').value
        self.visualize = self.get_parameter('visualize').value

        # ---- 加载 YOLOv8 ----
        self._load_model()

        # ---- 多桶检测输出 ----
        self.det_pub = self.create_publisher(
            String, '/bucket/detections', 10)

        # ---- 推理节流 ----
        self._frame_seq = 0        # 2026-10-07: 每完成一次真推理递增 (节流重发沿用旧序号)
        self._last_detections_json = "{}"
        self._last_infer_time = 0.0
        self._infer_interval = 1.0 / self.infer_hz

        # ---- 初始化摄像头 ----
        self._init_camera()

        self.get_logger().info(
            f"see ready [Multi-Bucket Detector] model={self._model_names} "
            f"conf={self.conf_threshold} infer={self.infer_hz}Hz "
            f"image={self.img_w}x{self.img_h}")

    # ======================== 模型加载 ========================

    def _load_model(self):
        model_path = self.get_parameter('model_path').value
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except ImportError:
            self.get_logger().fatal(
                "ultralytics not installed. Jetson: conda activate <env>")
            raise

        t0 = time.time()
        dummy = np.random.randint(0, 255, (self.img_h, self.img_w, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        self.get_logger().info(f"Model warm-up: {(time.time()-t0)*1000:.0f}ms")

        self._model_names = getattr(self.model, 'names', {})
        self._target_cls_idx = None
        if not self._model_names:
            self.get_logger().error(
                "Model has no class names! ALL detections will be treated as target!")
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
                    f"Class '{self.target_class}' not in model "
                    f"{list(self._model_names.values())}, falling back to any")

    # ======================== 摄像头 ========================

    def _init_camera(self):
        camera_device = self.get_parameter('camera_device').value
        self._cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().fatal(f"Cannot open camera: {camera_device}")
            raise RuntimeError(f"Cannot open camera: {camera_device}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.get_logger().info(
            f"Camera opened: {camera_device} "
            f"({self._cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
            f"{self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}) "
            f"-> resize to {self.img_w}x{self.img_h}")

        # 线程化相机读取
        self._frame = None
        self._frame_lock = threading.Lock()
        self._cam_running = True
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

        period = 0.1 if self.infer_hz <= 0 else 1.0 / self.infer_hz
        self._cam_timer = self.create_timer(period, self._tick)

    def _camera_loop(self):
        """Daemon 线程持续读帧, 主线程取最新帧零阻塞. 避免 cap.read() 阻塞推理循环."""
        while self._cam_running:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._frame_lock:
                    self._frame = frame

    # ======================== 推理 ========================

    def _tick(self):
        """定时器回调 (10Hz): 取帧 → resize → YOLO 推理 → 发布 JSON → 可视化.
        推理节流: 如果距上次推理不足 infer_interval, 重发缓存 JSON 维持 a.py 的检测流.
        """
        with self._frame_lock:
            frame = self._frame
        if frame is None:
            return

        frame = cv2.resize(frame, (self.img_w, self.img_h))
        now = time.time()

        if now - self._last_infer_time < self._infer_interval:
            # 节流: 重发上次结果
            self.det_pub.publish(String(data=self._last_detections_json))
            return

        self._last_infer_time = now
        try:
            results = self.model(frame, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=5)
            return

        buckets = self._all_detections(results)
        self._frame_seq += 1  # 2026-10-07: 真推理成功才递增; 节流重发缓存时序号不变
        self._last_detections_json = json.dumps(
            {"frame_seq": self._frame_seq, "buckets": buckets})
        self.det_pub.publish(String(data=self._last_detections_json))

        # 可视化
        if self.visualize:
            for b in buckets:
                x, y, w, h = b["x"], b["y"], b["w"], b["h"]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"see: {len(buckets)} buckets", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('see Multi-Bucket Detector', frame)
            cv2.waitKey(1)

    def _all_detections(self, results):
        """YOLO 输出 → JSON 检测列表.
        过滤: conf >= threshold, cls == target_class, bbox 有效 (w>0, h>0).
        边界裁剪: bbox 超出图像范围时裁剪到图像边界内.
        """
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        conf = boxes.conf.cpu().numpy().ravel()
        cls_arr = boxes.cls.cpu().numpy().ravel()

        thresh = self.conf_threshold
        target_cls = self._target_cls_idx
        imw, imh = self.img_w, self.img_h

        mask = conf >= thresh
        if target_cls is not None:
            mask &= (cls_arr == target_cls)
        if not mask.any():
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        valid_idx = np.where(mask)[0]

        buckets = []
        for idx in valid_idx:
            x1 = max(0, int(xyxy[idx, 0]))
            y1 = max(0, int(xyxy[idx, 1]))
            x2 = min(imw - 1, int(xyxy[idx, 2]))
            y2 = min(imh - 1, int(xyxy[idx, 3]))
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            buckets.append({"x": x1, "y": y1, "w": w, "h": h})

        return buckets


def main(args=None):
    rclpy.init(args=args)
    node = MultiBucketDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cam_running = False
        if hasattr(node, '_cap') and node._cap is not None:
            node._cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
