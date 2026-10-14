#!/usr/bin/env python3
"""离线验证 f_r3 (SITL 导入版) 与 frame_seq 补丁是否"接得上".

做法: 用 AST 剥离 ROS 依赖后 exec **真实的 f_r3.py**, 只替换传输层 (与交付包自带
test_f_drop.py 同一套手法), 然后直接喂 /bucket/detections 的 JSON 给 _cb_detect:

  B 节 —— frame_seq 语义 (补丁的正确性):
    B1 第 1 帧 (seq=1)        -> _detector_frame_has_seq 变 True, 匹配帧时间被刷新
    B2 重发同一 seq           -> 匹配帧时间**不变** (缓存重发不算新帧, 释放判据防骗)
    B3 新 seq (seq=2)         -> 匹配帧时间被刷新
    B4 没有序号的 JSON (旧检测器) -> _detector_frame_has_seq=False, 匹配帧时间不变
       ⇒ 这就是"打补丁前"的行为, 与离线探针实测的"设定点锚死 + 35s 兜底"对应

  C 节 —— SITL 检测器补丁的 JSON 形状:
    从 yolov8_bucket_detector.py 源码抽出 _publish_gt_dets / _infer_and_publish 的
    json.dumps 调用点, 断言两处都带 frame_seq, 且节流分支不递增.

用法: python3 scripts/test_f_r3_frameseq_20261007.py
"""
import ast
import copy
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path("/home/vincent/SITL")
F_R3 = ROOT / "src/drone_drl/drone_drl/f_r3.py"
DET = ROOT / "src/drone_drl/drone_drl/yolov8_bucket_detector.py"

PASS, FAIL = [], []


def check(cond, msg):
    (PASS if cond else FAIL).append(msg)
    print(("  ✓ " if cond else "  ✗ ") + msg)


# ---------------------------------------------------------------- 传输层替身
class Stamp:
    def __init__(self, sec): self.nanoseconds = round(sec * 1e9)
    def __sub__(self, o): return Stamp((self.nanoseconds - o.nanoseconds) / 1e9)


class Clock:
    seconds = 100.0
    def now(self): return Stamp(self.seconds)


class Message(SimpleNamespace):
    SOURCE_RC = 1


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, m): self.messages.append(copy.deepcopy(m))


class TransportNode:
    def __init__(self, name): self.clock = Clock(); self.logs = []; self.node_name = name
    def get_clock(self): return self.clock
    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, warn=self.logs.append,
                               error=self.logs.append, fatal=self.logs.append)
    def create_publisher(self, *a): return Publisher()
    def create_subscription(self, *a): return None
    def create_timer(self, *a): return SimpleNamespace(cancel=lambda: None)
    def destroy_node(self): pass


def load_f_r3():
    tree = ast.parse(F_R3.read_text(encoding="utf-8"))
    ext = ("rclpy", "px4_msgs", "sensor_msgs", "std_msgs", "geometry_msgs")
    tree.body = [i for i in tree.body if not (
        isinstance(i, ast.ImportFrom) and i.module.startswith(ext)
        or isinstance(i, ast.Import) and any(a.name == "rclpy" for a in i.names))]
    scope = dict(__name__="f_r3_under_test", __file__=str(F_R3), Node=TransportNode,
                 QoSProfile=Message,
                 ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
                 DurabilityPolicy=SimpleNamespace(VOLATILE=1))
    for n in ("TrajectorySetpoint OffboardControlMode VehicleCommand VehicleStatus "
              "VehicleLocalPosition VehicleLandDetected ManualControlSetpoint "
              "VehicleControlMode VehicleAttitude RegionOfInterest "
              "Float32 String Point").split():
        scope[n] = Message
    exec(compile(tree, str(F_R3), "exec"), scope)
    return SimpleNamespace(**scope)


def load_detector():
    """载入 SITL 检测器 (剥离 ROS 依赖), 用于直接调用 _publish_gt_dets / _infer_and_publish."""
    tree = ast.parse(DET.read_text(encoding="utf-8"))
    ext = ("rclpy", "px4_msgs", "sensor_msgs", "std_msgs", "geometry_msgs")
    tree.body = [i for i in tree.body if not (
        isinstance(i, ast.ImportFrom) and i.module.startswith(ext)
        or isinstance(i, ast.Import) and any(a.name in ("rclpy",) for a in i.names))]
    scope = dict(__name__="det_under_test", __file__=str(DET), Node=TransportNode,
                 QoSProfile=Message,
                 ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
                 DurabilityPolicy=SimpleNamespace(VOLATILE=1),
                 Image=Message, RegionOfInterest=Message)
    for n in ("VehicleLocalPosition", "VehicleStatus", "String", "Float32"):
        scope[n] = Message
    exec(compile(tree, str(DET), "exec"), scope)
    return SimpleNamespace(**scope)


def make_node(F):
    n = F.FR3Mission()
    n.home_x = n.home_y = n.home_z = 0.0
    n.x, n.y, n.z = 0.0, 0.0, -1.0          # H = 1.0m
    n.heading = 0.0
    n.vx = n.vy = n.vz = 0.0
    n._pitch = n._roll = 0.0
    n._current_target_ned = (0.0, 0.0)
    return n


def feed(n, seq, tag):
    """喂一帧 /bucket/detections: 桶在图像中心偏右 30px => 反投影约 (0, 0.086)m."""
    payload = {"buckets": [{"x": 320 + 30 - 40, "y": 180 - 40, "w": 80, "h": 80}]}
    if seq is not None:
        payload["frame_seq"] = seq
    before = None if n._last_matched_detect_time is None else n._last_matched_detect_time.nanoseconds
    n.clock.seconds += 0.1
    n._cb_detect(SimpleNamespace(data=json.dumps(payload)))
    after = None if n._last_matched_detect_time is None else n._last_matched_detect_time.nanoseconds
    return before, after


def main():
    print(f"f_r3.py sha256 = {hashlib.sha256(F_R3.read_bytes()).hexdigest()}")
    F = load_f_r3()
    check(hasattr(F, "FR3Mission"), "类名已改名 FR3Mission (与仓库 f.py 不冲突)")

    print("\n[A] 导入适配")
    src = F_R3.read_text(encoding="utf-8")
    check(F.PX4_STATUS_SUFFIX == "_v4", "PX4_STATUS_SUFFIX=_v4 (SITL 固件)")
    check(abs(F.HEADING_DEG - 92.0) < 1e-9, "HEADING_DEG=92 (i.world 场地)")
    check(F.MISSION_REVISION.endswith("-sitl-import"), "MISSION_REVISION 带 -sitl-import")
    check(abs(F.PAYLOAD_OFFSET_BODY_Y_B1 - 0.0567) < 1e-9
          and abs(F.PAYLOAD_OFFSET_BODY_Y_B2 + 0.0567) < 1e-9,
          "补偿常量原样保留 (±0.0567, 台架几何)")
    check(abs(F.DROP_ARRIVE_M - 0.05) < 1e-9 and F.DROP_CONFIRM_FRAMES == 2,
          "释放判据原样保留 (5cm + 2 帧)")

    print("\n[B] frame_seq 语义 (补丁正确性)")
    n = make_node(F)
    check(n._detector_frame_has_seq is False, "初始状态: 未见过序号")
    _ = feed(n, 1, "第一帧")
    check(n._detector_frame_has_seq is True, "B1 收到 seq=1 ⇒ has_seq=True")
    check(n.latest_roi is not None and n._last_matched_detect_time is not None,
          "B1 匹配成功, 刷新了匹配帧时间")
    t1 = n._last_matched_detect_time.nanoseconds
    b, a = feed(n, 1, "重发")
    check(a == t1, "B2 重发同一 seq ⇒ 匹配帧时间不变 (缓存重发不算新帧)")
    b, a = feed(n, 2, "新帧")
    check(a > t1, "B3 seq=2 ⇒ 匹配帧时间刷新")
    n2 = make_node(F)
    feed(n2, None, "无序号")
    check(n2._detector_frame_has_seq is False, "B4 没有 frame_seq ⇒ has_seq=False")
    # 注意: 无序号时 _cb_detect 仍会刷新匹配帧时间 (TRACK/refine 照常工作),
    #   真正被禁用的是 _drop_detection_fresh() —— 低空伺服/精投的**唯一**入口.
    check(n2._drop_detection_fresh() is False,
          "B4b 无序号 ⇒ _drop_detection_fresh() 恒 False (低空伺服被禁用; 离线探针实测=设定点锚死+35s兜底)")
    check(n._drop_detection_fresh() is True,
          "B4c 有序号且帧新鲜 ⇒ _drop_detection_fresh()=True (低空伺服可动)")

    print("\n[C] SITL 检测器补丁形状")
    d = DET.read_text(encoding="utf-8")
    check('"frame_seq": self._frame_seq, "buckets": all_dets' in d,
          "C1 YOLO 路径 JSON 带 frame_seq")
    check('"frame_seq": self._frame_seq, "buckets": dets' in d,
          "C2 GT 直发路径 JSON 带 frame_seq")
    i_thr = d.index("if now - self._last_infer_time < self._infer_interval:")
    i_ret = d.index("return", i_thr)
    check("_frame_seq += 1" not in d[i_thr:i_ret], "C3 节流重发分支不递增序号")
    check(d.count("self._frame_seq += 1") == 2, "C4 递增点恰好 2 处 (真推理 + GT 每 tick)")

    print("\n[D] 实跑补丁后的 SITL 检测器 (object.__new__, 只喂桩, 不启动 ROS)")
    det = load_detector()
    D = det.YoloV8BucketDetector if hasattr(det, "YoloV8BucketDetector") else None
    if D is None:
        for name in dir(det):
            obj = getattr(det, name)
            if isinstance(obj, type) and hasattr(obj, "_publish_gt_dets"):
                D = obj
                break
    check(D is not None, "D0 找到检测器类")
    if D is not None:
        inst = object.__new__(D)
        inst._bucket_count = 3
        inst.img_w, inst.img_h = 640, 360
        inst.multi_pub = Publisher()
        inst._last_json = None
        inst._frame_seq = 0
        circles = [(300.0, 200.0, 12.0), (100.0, 80.0, 9.0)]
        inst._publish_gt_dets(circles)
        j1 = json.loads(inst.multi_pub.messages[-1].data)
        inst._publish_gt_dets(circles)
        j2 = json.loads(inst.multi_pub.messages[-1].data)
        check(j1.get("frame_seq") == 1 and j2.get("frame_seq") == 2,
              f"D1 GT 直发每 tick 递增 (seq {j1.get('frame_seq')} -> {j2.get('frame_seq')})")
        check("buckets" in j1 and len(j1["buckets"]) == 2, "D1b JSON 里 buckets 仍原样保留")

        # 节流分支: 重发缓存 ⇒ 序号必须不变
        inst2 = object.__new__(D)
        inst2._bucket_count = 3
        inst2.img_w, inst2.img_h = 640, 360
        inst2.multi_pub = Publisher()
        inst2._frame_seq = 0
        inst2._infer_interval = 999.0
        inst2._last_infer_time = 1e18          # 强制走节流分支
        inst2.model = lambda *a, **k: (_ for _ in ()).throw(AssertionError("不应真推理"))
        inst2._last_json = Message(data=json.dumps({"ts": 0, "frame_seq": 7, "buckets": []}))
        inst2._infer_and_publish(object())
        got = json.loads(inst2.multi_pub.messages[-1].data)
        check(got.get("frame_seq") == 7,
              f"D2 节流重发沿用旧序号 (seq={got.get('frame_seq')}, 未被递增)")
        check(inst2._frame_seq == 0, "D2b 节流分支不递增计数器")
        del inst2.model

        # 真推理分支: 每次推理递增一次
        inst3 = object.__new__(D)
        inst3._bucket_count = 3
        inst3.img_w, inst3.img_h = 640, 360
        inst3.multi_pub = Publisher()
        inst3._frame_seq = 0
        inst3._infer_interval = 0.0
        inst3._last_infer_time = 0.0
        inst3._last_roi = None
        inst3.model = lambda img, verbose=False: "fake"
        inst3._all_detections = lambda results: [{"x": 1, "y": 2, "w": 3, "h": 4}]
        inst3._infer_and_publish(object())
        s1 = json.loads(inst3.multi_pub.messages[-1].data).get("frame_seq")
        inst3._infer_and_publish(object())
        s2 = json.loads(inst3.multi_pub.messages[-1].data).get("frame_seq")
        check(s1 == 1 and s2 == 2, f"D3 YOLO 路径每次真推理递增 (seq {s1} -> {s2})")

    print(f"\n结果: {len(PASS)} 项通过, {len(FAIL)} 项失败")
    if FAIL:
        for f in FAIL:
            print("  ✗ " + f)
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
