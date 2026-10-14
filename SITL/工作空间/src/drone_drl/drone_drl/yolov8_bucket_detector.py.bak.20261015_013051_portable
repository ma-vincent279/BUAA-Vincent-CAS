#!/usr/bin/env python3
"""
YOLOv8 桶检测器 — sim/real 共享 YOLOv8 推理链路，
  发布 /bucket/detection (RegionOfInterest, 单桶兼容) +
        /bucket/detections (String/JSON, 多桶模式).

sim 模式 (use_sim:=true):
  GT 投影 → 合成俯视图像(白桶) → YOLOv8 推理 → ROI 发布。
  仿真中用合成图像替代 Gazebo 摄像头（Gazebo Classic 不渲染3D物体）。

real 模式 (use_sim:=false):
  真实摄像头图像 → YOLOv8 推理 → ROI 发布。
  用于 Jetson Orin 实机部署。

ROS 参数:
  use_sim            true=合成图像 / false=真实摄像头
  model_path         YOLOv8 模型路径
  camera_topic       摄像头话题 (仅 real)
  conf_threshold     置信度阈值 (默认 0.5)
  target_class       目标类别名 (默认 "bucket")
  infer_hz           YOLOv8 推理频率上限 (默认 10Hz)
  bucket_gz_x/y/z    桶 Gazebo 坐标 (仅 sim, 单桶兼容)
  spawn_gz_x/y/z     飞机生成点 (仅 sim)
  img_width/height   图像尺寸 (默认 640x480)
  visualize          弹出可视化窗口 (默认 false)
  bucket_count       桶数量 (默认 1, 单桶兼容; >1 时启用 JSON 多桶发布)
  bucket{N}_gz_x/y/z 第 N 个桶的 Gazebo 坐标 (N=1,2,3)
"""

import json
import math
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest, Image
from std_msgs.msg import Header, String
from px4_msgs.msg import VehicleLocalPosition
from drone_drl.common import QOS_BE, local_pos_topic


class YOLOv8BucketDetector(Node):
    def __init__(self):
        super().__init__('yolov8_bucket_detector')

        # ---- 参数 ----
        self.declare_parameter('use_sim', True)
        self._use_sim = self.get_parameter('use_sim').value

        self.declare_parameter('model_path',
                               '/home/vincent/SITL/yolo_v8/best2.pt')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_device', '')  # 如 /dev/video0，直接读摄像头
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('infer_hz', 10.0)
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 360)
        self.declare_parameter('visualize', False)
        # 2026-10-12: 焦距可直接给定 (与任务节点同款相机模型)。
        #   默认 -1 ⇒ 用下面的 hfov/vfov 旧路径 (a~j 等项目行为不变)。
        #   给正值时 fx = fy = focal_px, 用于「原生画幅」相机 (单目/无畸变/对角 130°/16:9
        #   ⇒ d_px = hypot(640,360) = 734.30, f = 367.15/tan(65°) = 171.21 px)。
        self.declare_parameter('focal_px', -1.0)

        # sim 参数 — 单桶向前兼容, 多桶用 bucket_count
        self.declare_parameter('bucket_gz_x', 6.01)
        self.declare_parameter('bucket_gz_y', 0.98)
        self.declare_parameter('bucket_gz_z', 0.25)
        self.declare_parameter('spawn_gz_x', 1.01)
        self.declare_parameter('spawn_gz_y', 0.98)
        self.declare_parameter('spawn_gz_z', 0.83)
        self.declare_parameter('bucket_count', 1)  # 桶数量: 1=单桶兼容, >1=JSON 多桶
        # 2026-09-12: sim 模式"GT 直发"快通道 —— 直接把合成图算出的圆心当检测框发出,
        # **跳过 CPU 上的 YOLOv8 推理**。原因: 实测单帧推理 3.46s (有效 0.3Hz, 名义 10Hz),
        # 任务节点拿到的是 1~3.5s 前的框却用当前位姿反投影 → 瞄准误差 = 这期间的飞机位移
        # (标定工具 B 实测悬停帧 154/154 满足 |err| = pxdev·H/FX; 0.36m/s × 1.8s ≈ 0.65m)。
        # 该延迟纯属仿真环境算力问题, 实机 see.py 是 TensorRT + 10Hz (陈旧 0.1s → 3.6cm)。
        # 置 false 可退回"合成图 + YOLOv8"旧路径 (用于复现旧行为)。
        self.declare_parameter('sim_gt_direct', True)

        # 2026-10-09 P0: GT 投影的"目标平面"模式.
        #   'ground'     = 旧行为 (默认, 逐位不变): 把桶当成"参考平面上的一个圆"投影,
        #                  圆心 = 桶轴在该平面上的交点, 半径 = fx*r/H。
        #                  **它与任务节点 _roi_to_ned 的"平面求交反投影"互为逆运算 ⇒
        #                  SITL 往返误差恒为 0, 对实机那一类"目标平面选错"的误差结构性失明。**
        #   'silhouette' = 真实圆柱轮廓: 桶顶圆环与桶底圆环各按**自身真实深度**投影,
        #                  取两圆投影点集的并集外接矩形 (等价于轮廓的外接矩形)。
        #                  这才是下视相机真正看到的东西 (离轴时侧壁可见 ⇒ 近端极值下移到桶底)。
        # 只影响 sim 模式; 实机走 YOLO 框, 与本参数无关。
        self.declare_parameter('project_mode', 'ground')
        # 桶圆柱几何 (仅 'silhouette' 模式使用; 与 worlds/*.world 中 <cylinder> 一致)
        self.declare_parameter('bucket_length', 0.25)
        self.declare_parameter('silhouette_samples', 144)

        # 多桶位置参数 (N=1,2,3)
        for n in (1, 2, 3):
            for axis in ('x', 'y', 'z'):
                self.declare_parameter(f'bucket{n}_gz_{axis}', 0.0)
            # 2026-10-07: 单桶半径参数 (默认 0.125 = 旧固定值). 仅用于 SITL 造"大小不同的桶"
            # 来验证选桶策略; 实机走 YOLO 框, 不需要这个参数.
            self.declare_parameter(f'bucket{n}_radius', 0.125)

        self.img_w = self.get_parameter('img_width').value
        self.img_h = self.get_parameter('img_height').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.infer_hz = self.get_parameter('infer_hz').value
        self.visualize = self.get_parameter('visualize').value
        self._bucket_count = self.get_parameter('bucket_count').value
        self._sim_gt_direct = bool(self.get_parameter('sim_gt_direct').value)
        self._project_mode = str(self.get_parameter('project_mode').value).strip().lower()
        if self._project_mode not in ('ground', 'silhouette'):
            raise ValueError(
                f"project_mode 必须是 'ground' 或 'silhouette', 收到 {self._project_mode!r}")
        self._bucket_length = float(self.get_parameter('bucket_length').value)
        self._silhouette_samples = int(self.get_parameter('silhouette_samples').value)

        # ---- 加载 YOLOv8 模型 (sim / real 共用) ----
        self._load_model()

        # ---- 检测输出 ----
        self.det_pub = self.create_publisher(
            RegionOfInterest, '/bucket/detection', 10)
        if self._bucket_count > 1:
            self.multi_pub = self.create_publisher(
                String, '/bucket/detections', 10)
        else:
            self.multi_pub = None

        # ---- 缓存上次检测 + 推理节流 ----
        self._frame_seq = 0   # 2026-10-07: 每完成一次真推理递增 (节流重发沿用旧序号)
        self._last_roi = None
        self._last_json = None
        self._last_infer_time = 0.0
        self._infer_interval = 1.0 / self.infer_hz

        # ---- 按模式初始化 ----
        if self._use_sim:
            self._init_sim()
        else:
            self._init_real()

        mode_str = "SIM (synthetic img + YOLOv8)" if self._use_sim else "REAL (camera + YOLOv8)"
        if self._use_sim and self._sim_gt_direct:
            mode_str = "SIM (GT direct, 跳过 YOLO: 框零延迟)"
        self.get_logger().info(
            f"Ready [{mode_str}] model={self._model_names} "
            f"conf={self.conf_threshold} infer={self.infer_hz}Hz "
            f"image={self.img_w}x{self.img_h}")
        if self._use_sim:
            self.get_logger().info(
                f"SIM project_mode={self._project_mode} "
                f"bucket_length={self._bucket_length:.3f}m "
                f"samples={self._silhouette_samples}")
            if self._project_mode == 'silhouette' and not self._sim_gt_direct:
                self.get_logger().warn(
                    "project_mode=silhouette 与 sim_gt_direct=false 不搭配: "
                    "合成图仍按旧的地面圆绘制, 与发布框不一致 (建议 sim_gt_direct:=true)")

    # ======================== 模型加载 (共用) ========================

    def _load_model(self):
        model_path = self.get_parameter('model_path').value
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except ImportError:
            self.get_logger().fatal(
                "ultralytics not installed. "
                "Workstation: pip3 install ultralytics. "
                "Jetson: conda activate <env>")
            raise

        # 预热推理 (首次推理可能耗时 1-2s, 避免运行时卡顿)
        import time as _time
        _t0 = _time.time()
        dummy = np.random.randint(0, 255, (self.img_h, self.img_w, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        self.get_logger().info(
            f"Model warm-up: {(_time.time() - _t0)*1000:.0f}ms")

        self._model_names = getattr(self.model, 'names', {})
        if self._model_names:
            self.get_logger().info(f"Model classes: {list(self._model_names.values())}")

    # ======================== 推理 + 检测提取 (共用) ========================

    def _infer_and_publish(self, bgr_image):
        """对图像做 YOLOv8 推理，提取检测框并发布 (单桶ROI + 多桶JSON)."""
        now = time.time()
        if now - self._last_infer_time < self._infer_interval:
            # 推理节流：未到下次推理时间，重发上次结果
            if self._bucket_count > 1:
                if self._last_json is not None:
                    self.multi_pub.publish(self._last_json)
            elif self._last_roi is not None:
                self.det_pub.publish(self._last_roi)
            return

        self._last_infer_time = now
        try:
            results = self.model(bgr_image, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=5)
            return

        # 提取所有有效检测
        all_dets = self._all_detections(results)
        self._frame_seq += 1   # 2026-10-07: 真推理成功才算新一帧 (节流重发时不递增)

        if self._bucket_count > 1:
            # 多桶模式: 发布 JSON 数组
            payload = json.dumps({"ts": now, "frame_seq": self._frame_seq, "buckets": all_dets})
            msg = String(data=payload)
            self.multi_pub.publish(msg)
            self._last_json = msg
        else:
            # 单桶兼容: 发布最佳匹配
            roi = None
            if all_dets:
                best = all_dets[0]
                roi = RegionOfInterest()
                roi.x_offset = int(best["x"])
                roi.y_offset = int(best["y"])
                roi.width = int(best["w"])
                roi.height = int(best["h"])
                roi.do_rectify = False
            self._last_roi = roi
            if roi is not None:
                self.det_pub.publish(roi)

    def _all_detections(self, results):
        """从 YOLOv8 结果中提取全部有效检测框, 按置信度降序排列."""
        dets = []
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return dets

        for i in range(len(boxes)):
            conf = float(boxes.conf[i].item())
            if conf < self.conf_threshold:
                continue
            xyxy = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = xyxy.astype(int)
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(self.img_w - 1, x2)
            y2 = min(self.img_h - 1, y2)
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            dets.append({"x": int(x1), "y": int(y1), "w": int(w), "h": int(h),
                         "conf": round(conf, 4)})

        dets.sort(key=lambda d: d["conf"], reverse=True)
        return dets

    # ======================== sim 模式 ========================

    def _init_sim(self):
        sgx = self.get_parameter('spawn_gz_x').value
        sgy = self.get_parameter('spawn_gz_y').value
        sgz = self.get_parameter('spawn_gz_z').value

        self.bucket_radius = 0.125
        self.hfov = math.radians(85)
        self.vfov = math.radians(54)
        _focal = float(self.get_parameter('focal_px').value)
        if _focal > 0.0:
            # 2026-10-12: 与任务节点同一个相机模型 —— 由焦距反推视场 (原生画幅, 方形像素)
            self.fx = self.fy = _focal
            self.hfov = 2.0 * math.atan(self.img_w / 2.0 / self.fx)
            self.vfov = 2.0 * math.atan(self.img_h / 2.0 / self.fy)
        else:
            self.fx = self.img_w / (2.0 * math.tan(self.hfov / 2.0))
            self.fy = self.img_h / (2.0 * math.tan(self.vfov / 2.0))

        # 收集所有桶的 NED 坐标
        self._bucket_neds = []
        self._bucket_radii = []
        if self._bucket_count == 1:
            bgx = self.get_parameter('bucket_gz_x').value
            bgy = self.get_parameter('bucket_gz_y').value
            bgz = self.get_parameter('bucket_gz_z').value
            self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
            self._bucket_radii.append(float(self.get_parameter('bucket1_radius').value))
        else:
            for n in (1, 2, 3):
                bgx = self.get_parameter(f'bucket{n}_gz_x').value
                bgy = self.get_parameter(f'bucket{n}_gz_y').value
                bgz = self.get_parameter(f'bucket{n}_gz_z').value
                if bgx == 0.0 and bgy == 0.0 and bgz == 0.0:
                    continue  # 未配置的桶
                self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
                self._bucket_radii.append(float(self.get_parameter(f'bucket{n}_radius').value))

        self.drone_x = self.drone_y = self.drone_z = 0.0
        self.heading = None

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, local_pos_topic(),
            self._cb_local_sim, QOS_BE)

        # 合成图像发布 (可选, 调试用)
        self.syn_img_pub = self.create_publisher(
            Image, '/bucket/synthetic_image', 10)

        self.timer = self.create_timer(1.0 / self.infer_hz, self._tick_sim)

        # 预生成草地背景 (固定纹理, 避免随机变化导致 YOLO 偶尔检测不到)
        self._syn_bg = self._make_grass_bg()
        self.get_logger().info("SIM grass background pre-generated")

        for i, ned in enumerate(self._bucket_neds):
            self.get_logger().info(
                f"SIM bucket[{i}] NED({ned[0]:.2f},{ned[1]:.2f},{ned[2]:.2f}) "
                f"r={self._bucket_radii[i]:.3f}m")

    def _cb_local_sim(self, m):
        self.drone_x, self.drone_y, self.drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self.heading = m.heading

    def _gt_ground_circles(self):
        """旧路径: 把每个桶当成"参考平面上的一个圆", 返回 [(px, py, r_px), ...].

        2026-08-31: 投影高度与任务节点 _roi_to_ned 统一 (飞机→地面, SITL home_z≈0).
        原 bz-drone_z 含桶高 0.58m, 低空 (H=1.7m) 投影比例偏差 25% →
        反投影系统偏差 (est 偏近) → 伺服假稳态 → 投弹偏 0.43m (8-29/8-31 实测).
        **注意 (2026-10-09 P0)**: 这条路径与 _roi_to_ned 的平面求交互为逆运算,
        SITL 往返误差恒为 0 ⇒ 看不见实机"目标平面选错"那一类误差。见 project_mode。
        """
        circles = []  # (px, py, radius_px)
        for _bi, ned in enumerate(self._bucket_neds):
            bx, by, bz = ned
            dx = bx - self.drone_x
            dy = by - self.drone_y
            dz = -self.drone_z
            if dz < 0.3:
                continue
            cos_h = math.cos(self.heading)
            sin_h = math.sin(self.heading)
            body_fwd = dx * cos_h + dy * sin_h
            body_right = -dx * sin_h + dy * cos_h
            # 下视摄像头: 不设 body_fwd 硬门槛, 像素边界检查已足够
            px = self.img_w / 2.0 + (body_right / dz) * self.fx
            py = self.img_h / 2.0 - (body_fwd / dz) * self.fy
            radius_px = self.fx * self._bucket_radii[_bi] / dz
            if px < -50 or px > self.img_w + 50 or py < -50 or py > self.img_h + 50:
                continue
            circles.append((px, py, max(3.0, radius_px)))
        return circles

    def _gt_boxes_silhouette(self):
        """真实圆柱轮廓的检测框 (2026-10-09 P0), 返回 [(x1,y1,x2,y2), ...] 已裁剪.

        为什么需要: 真实下视相机看到的不是"地面上的一个圆", 而是**圆柱轮廓** ——
        桶顶圆环 (离相机近) 与桶底圆环 (离轴时侧壁可见, 离相机远) 共同构成轮廓。
        只把桶当平面圆投影, 会让仿真与任务节点的平面反投影互为逆运算, 从而
        **结构性掩盖实机误差**。

        几何: 轮廓 = 桶顶圆 ∪ 桶底圆 在像平面上的投影点集的凸包边界; 其外接矩形
        等于两圆投影点集并集的外接矩形 (轮廓的侧壁母线连接两圆上的点, 落在两圆
        投影的凸包内) ⇒ 直接对两圆各采 N 点、按**各自真实深度**投影后取 bbox。
        """
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        half = self._bucket_length / 2.0
        n = max(16, int(self._silhouette_samples))
        # 与旧路径同一高度门限 (参考平面高度)
        if -self.drone_z < 0.3:
            return []
        boxes = []
        for bi, ned in enumerate(self._bucket_neds):
            bx, by, bz = ned
            r = self._bucket_radii[bi]
            xs, ys = [], []
            for zc in (bz - half, bz + half):   # NED z 向下为正 ⇒ 顶圆 z 更小
                for k in range(n):
                    a = 2.0 * math.pi * k / n
                    wx = bx + r * math.cos(a)
                    wy = by + r * math.sin(a)
                    dz = zc - self.drone_z       # 相机(机体原点) → 该圆环的真实深度
                    if dz <= 0.05:
                        continue
                    dx = wx - self.drone_x
                    dy = wy - self.drone_y
                    body_fwd = dx * cos_h + dy * sin_h
                    body_right = -dx * sin_h + dy * cos_h
                    xs.append(self.img_w / 2.0 + (body_right / dz) * self.fx)
                    ys.append(self.img_h / 2.0 - (body_fwd / dz) * self.fy)
            if len(xs) < 3:
                continue
            px0, px1 = min(xs), max(xs)
            py0, py1 = min(ys), max(ys)
            if px1 < -50 or px0 > self.img_w + 50 or py1 < -50 or py0 > self.img_h + 50:
                continue
            x1 = max(0, int(round(px0)))
            y1 = max(0, int(round(py0)))
            x2 = min(self.img_w - 1, int(round(px1)))
            y2 = min(self.img_h - 1, int(round(py1)))
            if x2 - x1 <= 0 or y2 - y1 <= 0:
                continue
            boxes.append((x1, y1, x2, y2))
        return boxes

    def _circles_to_boxes(self, circles):
        """旧路径的圆 → 外接方框 (逐字保留旧公式, 保证默认模式输出不变)。"""
        boxes = []
        for px, py, rpx in circles:
            r = max(3.0, rpx)
            x1 = max(0, int(round(px - r)))
            y1 = max(0, int(round(py - r)))
            x2 = min(self.img_w - 1, int(round(px + r)))
            y2 = min(self.img_h - 1, int(round(py + r)))
            boxes.append((x1, y1, x2, y2))
        return boxes

    def _tick_sim(self):
        """每 tick: 所有桶 GT 投影 → 合成图像 → YOLOv8 推理 → 发布。"""
        if self.heading is None:
            return

        circles = self._gt_ground_circles()   # 合成图/可视化仍用旧的地面圆
        if self._project_mode == 'silhouette':
            boxes = self._gt_boxes_silhouette()
            if not boxes:
                return
        else:
            if not circles:
                return
            boxes = self._circles_to_boxes(circles)

        # ---- 2026-09-12: GT 直发快通道 (默认) — 跳过 CPU YOLOv8, 框零延迟 ----
        if self._sim_gt_direct:
            self._publish_gt_boxes(boxes)
            if self.visualize and circles:
                self._show_synth(circles, self._gen_synthetic(circles))
            return

        if not circles:
            return
        syn_img = self._gen_synthetic(circles)
        self._infer_and_publish(syn_img)

        if self.visualize:
            self._show_synth(circles, syn_img)

    def _publish_gt_boxes(self, boxes):
        """把 GT 框直接发布 (跳过 YOLO).

        框 = project_mode 决定 (ground: 投影圆的外接方框; silhouette: 真实轮廓),
        已裁剪到图像内, 与 YOLO 路径 (int + clip) 保持一致.
        """
        now = time.time()
        dets = []
        for x1, y1, x2, y2 in boxes:
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            dets.append({"x": x1, "y": y1, "w": w, "h": h, "conf": 1.0})

        if self._bucket_count > 1:
            self._frame_seq += 1   # 2026-10-07: GT 直发每 tick 都是一次新投影 = 新一帧
            msg = String(data=json.dumps({"ts": now, "frame_seq": self._frame_seq, "buckets": dets}))
            self.multi_pub.publish(msg)
            self._last_json = msg
        else:
            roi = None
            if dets:
                b = dets[0]
                roi = RegionOfInterest()
                roi.x_offset, roi.y_offset = b["x"], b["y"]
                roi.width, roi.height = b["w"], b["h"]
                roi.do_rectify = False
            self._last_roi = roi
            if roi is not None:
                self.det_pub.publish(roi)

    def _show_synth(self, circles, syn_img):
        """可视化窗口 (GT 直发模式下也用合成图显示, 便于肉眼确认桶在画面里的位置)."""
        display = syn_img.copy()
        if self._bucket_count > 1 and self._last_json is not None:
            try:
                for d in json.loads(self._last_json.data).get("buckets", []):
                    cv2.rectangle(display, (d["x"], d["y"]),
                                  (d["x"] + d["w"], d["y"] + d["h"]), (0, 255, 0), 2)
            except Exception:
                pass
        elif self._last_roi is not None:
            r = self._last_roi
            cv2.rectangle(display, (r.x_offset, r.y_offset),
                          (r.x_offset + r.width, r.y_offset + r.height), (0, 255, 0), 2)
        for px, py, rpx in circles:
            cv2.circle(display, (int(px), int(py)), int(rpx), (0, 0, 255), 1)
        cv2.putText(display, f"SIM x{len(circles)} GT-direct", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow('Detector (SIM)', display)
        cv2.waitKey(1)

    # ======================== 合成图像生成 ========================

    def _make_grass_bg(self):
        """预生成低纹理噪声背景 (固定种子, 高检测率 >99%)"""
        rng = np.random.RandomState(42)
        img = rng.randint(80, 150, (self.img_h, self.img_w, 3)).astype(np.uint8)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        return img

    def _gen_synthetic(self, circles):
        """在预生成背景上叠加白桶圆圈, 固定纹理确保 YOLOv8 检测稳定。
        circles: [(cx, cy, radius_px), ...]"""
        img = self._syn_bg.copy()
        for cx, cy, radius_px in circles:
            cx_i, cy_i = int(round(cx)), int(round(cy))
            r_i = max(30, min(int(round(radius_px)), 120))
            cv2.circle(img, (cx_i, cy_i), r_i, (230, 230, 230), -1)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        return img

    # ======================== real 模式 ========================

    def _init_real(self):
        camera_device = self.get_parameter('camera_device').value

        if camera_device:
            # 直读摄像头 — 必须 V4L2+MJPG 才能设分辨率, fly/ Known Issue #2
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
            self._cam_timer = self.create_timer(
                1.0 / self.infer_hz, self._tick_camera)
        else:
            # ROS2 话题订阅 (需要独立的摄像头驱动节点如 usb_cam)
            camera_topic = self.get_parameter('camera_topic').value
            self.camera_sub = self.create_subscription(
                Image, camera_topic, self._cb_image, 10)
            self.get_logger().info(
                f"Subscribed to camera topic: {camera_topic}")

    def _tick_camera(self):
        """定时器回调: 从 cv2.VideoCapture 读帧 → 推理 → 发布。"""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Camera read failed", throttle_duration_sec=5)
            return
        self._infer_and_publish(frame)

        if self.visualize:
            display = frame.copy()
            if self._last_roi is not None:
                r = self._last_roi
                cv2.rectangle(display,
                              (r.x_offset, r.y_offset),
                              (r.x_offset + r.width, r.y_offset + r.height),
                              (0, 255, 0), 2)
            cv2.putText(display, "REAL (direct)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('YOLOv8 Detector (REAL)', display)
            cv2.waitKey(1)

    def _cb_image(self, msg):
        """ROS2 话题回调: 解码 → YOLOv8 推理 → 发布。"""
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
            display = arr.copy()
            if self._last_roi is not None:
                r = self._last_roi
                cv2.rectangle(display,
                              (r.x_offset, r.y_offset),
                              (r.x_offset + r.width, r.y_offset + r.height),
                              (0, 255, 0), 2)
            cv2.putText(display, "REAL (topic)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('YOLOv8 Detector (REAL)', display)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8BucketDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_cap') and node._cap is not None:
            node._cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
