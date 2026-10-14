#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_vis_gate_20261009.py — 修 k 的"视觉门 vs 到位容差"死锁 (只改 k 两份)

背景 (2026-10-09 13:59 实飞复盘 + 用户指正"你根本没飞到桶上面"):
  相机是**正下视** (k.py `camera_pixel_to_level` 文档串: "相机光轴=机体下"),
  ⇒ 地面覆盖 = 像素范围反算: 前向 ±(IMG_H/2)/FY·H, 横向 ±(IMG_W/2)/FX·H
  ⇒ H=K_FLIGHT_ALT=1.2m 时 **前向只有 ±0.673 m** (横向 ±1.196 m)。
  本轮结束 dist_to_target=1.8m ⇒ 桶远在视野外 ⇒ 两趟实飞 0 检测的**直接原因**
  (飞不到桶上方就什么都看不到; 5m 那趟视野 ±2.8m 能看到但桶只 18~24px ⇒ conf=0.5 不出框).

本补丁修的是**另一个真 bug** (上一轮 `patch_k_flat` 引入):
  FORWARD 到位判据 = `dist2d < POS_ARRIVE_DIST (0.5)` ⇒ 飞机可在离靶点 0.3~0.5m 处合法停下;
  而视觉门要求 `已飞 ≥ FORWARD_DIST − K_VISUAL_GATE_M (0.30) = 2.70m` (离靶点 ≤0.30m)
  ⇒ 落在 0.30~0.50m 带里时 **视觉门永不满足 ⇒ 选不了桶 ⇒ 10s 超时返航** ✗

修法 (可行区间被几何夹死):
  下界 = POS_ARRIVE_DIST            (0.5)   —— 否则"到位了也不放行" ← 死锁
  上界 = (IMG_H/2)/FY·K_FLIGHT_ALT  (0.673) —— 否则"放行了却看不见桶"
  ⇒ 取 K_VISUAL_GATE_M = 0.60 (区间内, 上下各留 ~0.07/0.10m 余量)
  并且门控里加**结构性下界** `max(K_VISUAL_GATE_M, POS_ARRIVE_DIST)`:
  数学上保证 "FORWARD 到位 ⇒ 门控必过" (到位 ⇒ 3−a < 0.5 ≤ G ⇒ a > 3−G ⇒ 门控满足),
  以后谁把常量调小也不会再死锁。

用法: python3 scripts/patch_k_vis_gate_20261009.py [--check]
"""
import argparse
import datetime
import hashlib
import pathlib
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = [
    REPO / "src/drone_drl/drone_drl/k.py",
    REPO / "fly/src/fly/fly/k.py",
]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_vis_gate"

# ── A) 常量: 0.30 → 0.60 (+ 记录可行区间推导) ──────────────────────────────
A_ANCHOR = (
    "K_VISUAL_GATE_M = 0.30  # 2026-10-09 实机反馈③: 沿航向已飞距离 < FORWARD_DIST-该值 时\n"
    "                        #   **禁止选桶/进伺服** ⇒ 必须先真正飞到前方 3m 处才启动视觉\n"
)
A_INSERT = (
    "K_VISUAL_GATE_M = 0.60  # 2026-10-09 实机反馈③(+ 同日死锁修复): 沿航向已飞距离\n"
    "                        #   < FORWARD_DIST-该值 时 **禁止选桶/进伺服** (必须先真正飞到\n"
    "                        #   前方 3m 处才启动视觉)。取值被几何夹死, 只能落在:\n"
    "                        #     下界 POS_ARRIVE_DIST(0.5)        —— 否则 FORWARD 合法停在\n"
    "                        #        0.3~0.5m 时门永不满足 ⇒ 到了也不选桶(10s 超时返航)\n"
    "                        #     上界 (IMG_H/2)/FY·K_FLIGHT_ALT ≈ 0.673m —— 正下视相机在\n"
    "                        #        1.2m 的前向视野半径; 超了就会\"放行了却看不见桶\"\n"
    "                        #   0.60 ∈ (0.5, 0.673) ✓ 且门控里另有结构性下界 max(·, POS_ARRIVE_DIST)\n"
)

# ── B) 门控: 加结构性下界 ─────────────────────────────────────────────────
B_ANCHOR = "            if _travel_fwd < FORWARD_DIST - K_VISUAL_GATE_M:\n"
B_REPLACE = (
    "            # 2026-10-09 死锁修复: 下界夹到 POS_ARRIVE_DIST ⇒ 数学上保证\n"
    "            #   \"FORWARD 到位(dist2d<POS_ARRIVE_DIST) ⇒ 门控必过\" ⇒ 永不空转到超时\n"
    "            _vg_need = max(K_VISUAL_GATE_M, POS_ARRIVE_DIST)\n"
    "            if _travel_fwd < FORWARD_DIST - _vg_need:\n"
)

# ── C) 门控日志同步用 _vg_need ────────────────────────────────────────────
C_ANCHOR = '                        f"  [视觉门] 已飞 {_travel_fwd:.2f}m < {FORWARD_DIST - K_VISUAL_GATE_M:.2f}m"\n'
C_REPLACE = '                        f"  [视觉门] 已飞 {_travel_fwd:.2f}m < {FORWARD_DIST - _vg_need:.2f}m"\n'

MARK = "K_VISUAL_GATE_M = 0.60"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path: pathlib.Path, check: bool) -> str:
    src = path.read_text(encoding="utf-8")
    if MARK in src:
        return "SKIP (已打过, 幂等)"
    for name, anchor in (("A", A_ANCHOR), ("B", B_ANCHOR), ("C", C_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"  ✗ {path}: 锚点 {name} 命中 {n} 次 (要求恰 1 次) ⇒ 中止", file=sys.stderr)
            sys.exit(2)
    out = src.replace(A_ANCHOR, A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_REPLACE, 1)
    out = out.replace(C_ANCHOR, C_REPLACE, 1)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 打补丁后语法错误 {e} ⇒ 中止", file=sys.stderr)
        sys.exit(3)
    if check:
        return f"CHECK-OK (将改 {len(out) - len(src):+d} 字节)"
    bak = path.with_name(path.name + "." + BAK_SUFFIX)
    shutil.copy2(path, bak)
    path.write_text(out, encoding="utf-8")
    return f"OK  {sha(path)}  (备份 {bak.name})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    print(f"=== patch_k_vis_gate_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(4)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
