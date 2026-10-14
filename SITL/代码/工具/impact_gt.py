#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""impact_gt.py — 用 **Gazebo 真值姿态** 记录投放落点 (2026-10-13)

为什么需要它 (诊断结论的关键一步)
----------------------------------
SITL 里 `cuadc_judge` 拿 `/drop/impact_rel` 当落点, 而这个量是 **飞机自报的 EKF 位置**。
于是一旦 EKF 水平位置有误差, 它会 **1:1 变成"脱靶"**, 哪怕真实的几何关系是准的。

实测证据 (seed 33, `runs/k_33_20261013_133448`, 第 1 投):
  · t=48.8 飞机 EKF 位置 (2.19,31.61) vs 真值桶 (2.187,31.631) ⇒ **2.1cm**
  · 同一时刻像素误差 **px=(+0,+2) = 2px** ⇒ H=1.77m 时 **2.1cm**  ⇒ **视觉链路完全正确** ✓
  · 但释放瞬间 EKF 位置 (2.110,31.663) vs 真值桶 ⇒ **8.3cm** ⇒ 全部来自 EKF/参考系
⇒ 需要一个**独立于 EKF 的落点真值**来把这两者分开。Gazebo 的 SceneBroadcaster
  (`gz-sim-scene-broadcaster-system`, PX4 的 server.config 已加载) 发布
  `/world/<world>/pose/info` (gz.msgs.Pose_V) = **所有模型的真值姿态**。

本节点
------
订阅 ros_gz_bridge 转出来的 `/world/<world>/pose/info` (tf2_msgs/TFMessage),
记住目标模型 (默认 `x500_0`) 的最新真值位姿; 每当 `/drop/impact_rel` 收到一投,
就把**同一时刻的真值位姿**换算成"相对生成点的 NED"并发布到 `/drop/impact_gt`
⇒ 直接把 `cuadc_judge` 的 `impact_topic` 换成它, 就得到**真值口径**的分数。

坐标换算 (gz ENU → NED, 与 worlds/*.truth.json 同系):
    ned_x = gz_y - spawn_y ;  ned_y = gz_x - spawn_x

用法:
    ros2 run drone_drl impact_gt --ros-args \
        -p pose_topic:=/world/k_gz/pose/info -p model_name:=x500_0
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage


class ImpactGT(Node):
    def __init__(self):
        super().__init__("impact_gt")
        self.declare_parameter("pose_topic", "/world/k_gz/dynamic_pose/info")
        self.declare_parameter("model_name", "x500_0")
        self.declare_parameter("trigger_topic", "/drop/impact_rel")
        self.declare_parameter("out_topic", "/drop/impact_gt")
        self.declare_parameter("spawn_gz_x", 1.01)   # 与 PX4_GZ_MODEL_POSE 一致
        self.declare_parameter("spawn_gz_y", 0.98)
        self.declare_parameter("pose_topic_alt", "/world/k_gz/pose/info")
        self.declare_parameter("lock_radius", 0.6)   # 认定飞机: 距生成点 (x,y) 不超过这个半径
        self.declare_parameter("lock_z_min", 0.5)    # 且高度不低于这个值 (地面/场地都压不到)
        self.declare_parameter("debug_frames", 3)
        self.declare_parameter("debounce_s", 1.0)   # 同一次投放的重复触发忽略窗   # 前 N 条打印帧名 (诊断用)

        self.model = str(self.get_parameter("model_name").value)
        self.sx = float(self.get_parameter("spawn_gz_x").value)
        self.sy = float(self.get_parameter("spawn_gz_y").value)
        self._pose = None
        self._n = 0
        self._idx = None
        self._active_src = None
        self._last_trigger = 0.0
        self._lock_r = float(self.get_parameter("lock_radius").value)
        self._z_min = float(self.get_parameter("lock_z_min").value)
        self._dbg = int(self.get_parameter("debug_frames").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        # 同时挂两个候选话题 (SceneBroadcaster 的 all-entities / dynamic-only 两路都试)
        _topics = [str(self.get_parameter("pose_topic").value)]
        _alt = str(self.get_parameter("pose_topic_alt").value)
        if _alt and _alt not in _topics:
            _topics.append(_alt)
        # ⚠ 两个话题的**帧下标不通用** (dynamic_pose 只有几个实体, pose/info 有 107 个),
        #   而且动态/静态实体的顺序不同 ⇒ 若同时用会把 `_idx` 反复打回 None、来回重锁。
        #   因此: **只用最先锁定成功的那一个话题**, 之后另一个的回调直接忽略。
        from functools import partial
        for _t in _topics:
            self.create_subscription(
                TFMessage, _t, partial(self._cb_pose, _src=_t), qos)
        self.get_logger().info(f"订阅真值话题: {_topics}")
        self.create_subscription(
            Point, str(self.get_parameter("trigger_topic").value), self._cb_trigger, 10)
        self.pub = self.create_publisher(Point, str(self.get_parameter("out_topic").value), 10)
        self.get_logger().info(
            f"impact_gt ready: pose={self.get_parameter('pose_topic').value} "
            f"model={self.model} spawn=({self.sx},{self.sy}) "
            f"⇒ {self.get_parameter('out_topic').value}")

    def _cb_pose(self, m, _src=None):
        """⚠ 实测 (2026-10-13): ros_gz_bridge 把 Pose_V→TFMessage 时 **child_frame_id 全为空**,
        所以**不能按帧名找飞机**。改用**索引 + 位置**: gz 的 Pose_V 顺序稳定,
        开局时"离生成点 (spawn_gz_x/y) 最近且 z>0.1"的那一帧就是飞机 (桶都在 x≈31~36m 外,
        地面/场地在 z≈0) ⇒ 记下它的下标, 之后一直跟这个下标。"""
        if self._active_src is not None and _src != self._active_src:
            return                              # 已经用另一个话题锁定了 ⇒ 忽略这个
        if self._idx is None:
            # ⚠ 不能"取离生成点最近的那一帧"就算数 —— 首帧到达时飞机**还没生成**,
            #   会错锁到场地实体 (实测锁到 gz(0.17,0.17,0.30))。
            #   只在**真的有实体落在生成点附近**时才锁定: x,y 距离 ≤ lock_radius 且 z ≥ lock_z_min。
            best, bd = None, 1e9
            for i, tr in enumerate(m.transforms):
                t = tr.transform.translation
                if t.z < self._z_min:
                    continue
                d = math.hypot(t.x - self.sx, t.y - self.sy)
                if d <= self._lock_r and d < bd:
                    bd, best = d, i
            if best is None:
                return                      # 飞机还没出现 ⇒ 等下一帧
            self._idx = best
            self._active_src = _src
            t = m.transforms[best].transform.translation
            self.get_logger().info(
                f"[Pose_V] 锁定飞机 = 第 {best} 帧 (离生成点 {bd:.2f}m, "
                f"gz=({t.x:.2f},{t.y:.2f},{t.z:.2f})) ⇒ 开始记录真值")
        if self._idx >= len(m.transforms):
            self._idx = None
            return
        t = m.transforms[self._idx].transform.translation
        self._pose = (t.x, t.y, t.z)

    def _cb_trigger(self, _msg):
        """一次投放可能触发**多次** (实测每投 2 次, 相隔 7~46ms, 位姿略不同) ——
        根因未完全定位 (k.py 只有 1 个发布点、impact_gt 也只起 1 个), 这里**去抖**:
        距上次触发 < debounce_s 的当作同一投忽略, 否则裁判会多记几投、把重复项
        判成 215/260cm 的"脱靶"(实测过一次)。
        """
        import time as _t
        _now = _t.monotonic()
        if _now - self._last_trigger < float(self.get_parameter("debounce_s").value):
            self.get_logger().info(
                f"忽略重复触发 (距上次 {( _now - self._last_trigger) * 1000:.0f}ms)")
            return
        self._last_trigger = _now
        if self._pose is None:
            self.get_logger().warn("收到投放但还没有真值位姿 (桥接没起来?)")
            return
        gx, gy, gz = self._pose
        self._n += 1
        self.pub.publish(Point(x=float(gy - self.sy), y=float(gx - self.sx), z=float(gz)))
        self.get_logger().info(
            f"DROP #{self._n} 真值落点 (相对生成点 NED): "
            f"({gy - self.sy:+.3f},{gx - self.sx:+.3f})  [gz=({gx:.3f},{gy:.3f},{gz:.3f})]")


def main():
    rclpy.init()
    node = ImpactGT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
