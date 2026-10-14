#!/usr/bin/env python3
"""给检测器补 frame_seq (每次真推理递增, 节流重发沿用旧序号).

背景 (2026-10-07): 外接盘那份 "f r3" 的低空伺服/精投依赖 `_drop_detection_fresh()`,
它要求检测器每帧发一个**递增序号** `frame_seq`; 缺序号时 `_detector_frame_has_seq=False`
⇒ 低空 XY 伺服完全不动作, 设定点在进入 VISUAL_SERVO 时被锚死, 只剩 35s 兜底释放
(离线实测: 落点偏差 = 进场时离桶的距离, 例 50.0cm).

本脚本改两个文件, 逻辑一致:
  A) src/drone_drl/drone_drl/yolov8_bucket_detector.py  (SITL 仿真检测器, 本轮仿真要用)
       · _publish_gt_dets()  : GT 直发路径, 每 tick 都是一次新投影 ⇒ 递增
       · _infer_and_publish(): 真推理后递增; 节流分支重发 _last_json ⇒ 序号不变
  B) fly/src/fly/fly/see.py                             (实机检测器, 等你烧到 Jetson)
       · 推理节流重发 self._last_detections_json ⇒ 序号同样不变

关键点 (写错就白打): 必须"每完成一次真推理递增一次", 不能"每次 publish 递增";
否则任务节点里"重发的缓存帧不算新帧"的去重失效, 同一张旧图会骗过释放判据
(外接盘自带测试 test_detection_callback_repeated_sequence_does_not_refresh_age 就是防这个).

用法: python3 scripts/patch_frame_seq_20261007.py
回滚: 用打印出来的 .bak.<时间戳>_frame_seq 覆盖回去 (每个文件一行 cp), 再 colcon build
"""
import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/home/vincent/SITL")
STAMP = time.strftime("%Y%m%d_%H%M%S")

# ---------------- A) SITL 检测器 ----------------
A = ROOT / "src/drone_drl/drone_drl/yolov8_bucket_detector.py"
A_PATCHES = [
    (
        "        self._last_roi = None\n"
        "        self._last_json = None\n",
        "        self._frame_seq = 0   # 2026-10-07: 每完成一次真推理递增 (节流重发沿用旧序号)\n"
        "        self._last_roi = None\n"
        "        self._last_json = None\n",
        "A1 新增 _frame_seq 计数器",
    ),
    (
        "        # 提取所有有效检测\n"
        "        all_dets = self._all_detections(results)\n",
        "        # 提取所有有效检测\n"
        "        all_dets = self._all_detections(results)\n"
        "        self._frame_seq += 1   # 2026-10-07: 真推理成功才算新一帧 (节流重发时不递增)\n",
        "A2 真推理后递增",
    ),
    (
        '            payload = json.dumps({"ts": now, "buckets": all_dets})',
        '            payload = json.dumps({"ts": now, "frame_seq": self._frame_seq, "buckets": all_dets})',
        "A3 YOLO 路径 JSON 带序号",
    ),
    (
        "        if self._bucket_count > 1:\n"
        '            msg = String(data=json.dumps({"ts": now, "buckets": dets}))',
        "        if self._bucket_count > 1:\n"
        "            self._frame_seq += 1   # 2026-10-07: GT 直发每 tick 都是一次新投影 = 新一帧\n"
        '            msg = String(data=json.dumps({"ts": now, "frame_seq": self._frame_seq, "buckets": dets}))',
        "A4 GT 直发路径递增 + JSON 带序号",
    ),
]

# ---------------- B) 实机 see.py ----------------
B = ROOT / "fly/src/fly/fly/see.py"
B_PATCHES = [
    (
        '        self._last_detections_json = "{}"\n',
        "        self._frame_seq = 0        # 2026-10-07: 每完成一次真推理递增 (节流重发沿用旧序号)\n"
        '        self._last_detections_json = "{}"\n',
        "B1 新增 _frame_seq 计数器",
    ),
    (
        "        buckets = self._all_detections(results)\n"
        '        self._last_detections_json = json.dumps({"buckets": buckets})',
        "        buckets = self._all_detections(results)\n"
        "        self._frame_seq += 1  # 2026-10-07: 真推理成功才递增; 节流重发缓存时序号不变\n"
        '        self._last_detections_json = json.dumps(\n'
        '            {"frame_seq": self._frame_seq, "buckets": buckets})',
        "B2 真推理后递增 + JSON 带序号",
    ),
]


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def apply(path: Path, patches, label: str) -> bool:
    print(f"\n===== {label}: {path.relative_to(ROOT)}")
    if not path.is_file():
        sys.exit(f"[FAIL] 文件不存在: {path}")
    before = sha(path)
    print(f"  改前 sha256 = {before}")
    s = path.read_text(encoding="utf-8")
    if '"frame_seq"' in s:
        print("  [跳过] 已包含 frame_seq (疑似已打过补丁)")
        return False
    for old, new, tag in patches:
        n = s.count(old)
        if n != 1:
            sys.exit(f"[FAIL] {tag}: 锚点命中 {n} 次 (要求 1): {old[:70]!r}")
        s = s.replace(old, new, 1)
        print(f"  {tag}: OK")
    bak = path.with_suffix(path.suffix + f".bak.{STAMP}_frame_seq")
    shutil.copy2(path, bak)
    path.write_text(s, encoding="utf-8")
    try:
        compile(s, str(path), "exec")
    except SyntaxError as e:
        shutil.copy2(bak, path)
        sys.exit(f"[FAIL] 语法错误, 已回滚: {e}")
    r = subprocess.run([sys.executable, "-m", "py_compile", str(path)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        shutil.copy2(bak, path)
        sys.exit(f"[FAIL] py_compile 失败, 已回滚: {r.stderr}")
    after = sha(path)
    print(f"  备份 = {bak.name}")
    print(f"  改后 sha256 = {after}   py_compile OK")
    return True


def main() -> int:
    changed_a = apply(A, A_PATCHES, "A) SITL 检测器")
    changed_b = apply(B, B_PATCHES, "B) 实机 see.py")

    print("\n===== 语义自检 (在源码上验证 '节流重发不递增') =====")
    src = A.read_text(encoding="utf-8")
    # 节流分支 (return 之前) 不得出现递增
    i_thr = src.index("if now - self._last_infer_time < self._infer_interval:")
    i_ret = src.index("return", i_thr)
    seg = src[i_thr:i_ret]
    ok1 = "_frame_seq += 1" not in seg
    print(f"  A 节流分支不含递增: {'OK' if ok1 else 'FAIL'}")
    ok2 = src.count("self._frame_seq += 1") == 2
    print(f"  A 递增点恰好 2 处 (YOLO 路径 + GT 直发路径): {'OK' if ok2 else 'FAIL'}")
    sb = B.read_text(encoding="utf-8")
    i_thr = sb.index("if now - self._last_infer_time < self._infer_interval:")
    i_ret = sb.index("return", i_thr)
    ok3 = "_frame_seq += 1" not in sb[i_thr:i_ret]
    print(f"  B 节流分支不含递增: {'OK' if ok3 else 'FAIL'}")
    ok4 = sb.count("self._frame_seq += 1") == 1
    print(f"  B 递增点恰好 1 处: {'OK' if ok4 else 'FAIL'}")
    if not all((ok1, ok2, ok3, ok4)):
        sys.exit("[FAIL] 语义自检未通过")

    if changed_a or changed_b:
        print("\n下一步: cd ~/SITL && colcon build --packages-select drone_drl")
    print("\n✅ frame_seq 补丁完成")
    print("   回滚命令:")
    for p in (A, B):
        bk = list(p.parent.glob(p.name + f".bak.{STAMP}_frame_seq"))
        if bk:
            print(f"     cp {bk[0]} {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
