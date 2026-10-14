#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_descend_anchor_to_bucket_20261009.py — 降高点从"名义 3m 点"改为"检测到的桶" (只 k 两份)

15:22 实飞暴露 (在降高爬行修好之后就会立刻撞上的第二个问题):
  飞机 `Arrived at target, dist=0.40m` 停在名义点附近, 但 5m 高度的检测给出的桶位是
  NED(-2.56,-1.97) ⇒ **飞机离桶 ~0.9m**; 而 1.2m 时正下视相机**前向只看 ±0.672m**
  ⇒ 桶会落在画面外 ⇒ 降到 1.2m 也看不到桶 (白降) ✗

修法: 降高时**优先用已确认追踪器给出的桶位**当下降锚点 (5m 高度已经能稳定检测到桶,
  误差约 ±0.3m), 只有"离名义点 < K_DESCEND_PULL_M"的追踪器才采信 ⇒ 防第二条假框把飞机带偏
  (15:22 那趟就有 2 个追踪器: 真桶距名义点 0.38m / 假框 0.89m ⇒ 取最近的 = 真桶 ✓)。
  没有已确认追踪器 ⇒ 退回名义点 (原行为, 不变)。

用法: python3 scripts/patch_k_descend_anchor_to_bucket_20261009.py [--check]
"""
import argparse
import datetime
import hashlib
import pathlib
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = [REPO / "src/drone_drl/drone_drl/k.py", REPO / "fly/src/fly/fly/k.py"]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_anchor_bucket"

# A) 常量
A_ANCHOR = "K_DESCEND_RATE       = 1.0    # 2026-10-09: HOVER_SEARCH 到点后下降到投弹高度的限速 (m/s)\n"
A_INSERT = ("K_DESCEND_PULL_M     = 1.5    # 2026-10-09: 降高锚点允许被\"已确认桶估计\"拉多远 (m)\n"
            "                              #   只采信离名义点这么近的追踪器 ⇒ 防第二条假框带偏\n")

# B) 下降锚点 = 检测到的桶 (优先) / 名义点 (兜底)
B_ANCHOR = "            _pre_z = self.home_z - PRE_SERVO_HEIGHT\n"
B_INSERT = (
    "            # 2026-10-09 追加 39: 降高**对准检测到的桶**, 不只名义 3m 点 ——\n"
    "            #   15:22 实飞: 飞机停在离桶 ~0.9m 处, 而 1.2m 时相机前向只看 ±0.672m ⇒ 桶在画面外;\n"
    "            #   5m 高度已能检测到桶 (±0.3m) ⇒ 用它把下降点拉过去。只认离名义点\n"
    "            #   < K_DESCEND_PULL_M 的**已确认**追踪器 (防第二条假框把飞机带偏); 没有则退回名义点。\n"
    "            _dx, _dy = self.home_x + tx, self.home_y + ty\n"
    "            _conf_trk = [t for t in self._trackers if t.get(\"confirmed\")]\n"
    "            _pull = None\n"
    "            if _conf_trk:\n"
    "                _bt = min(_conf_trk, key=lambda t: math.hypot(t[\"ned\"][0] - tx, t[\"ned\"][1] - ty))\n"
    "                _bd = math.hypot(_bt[\"ned\"][0] - tx, _bt[\"ned\"][1] - ty)\n"
    "                if _bd < K_DESCEND_PULL_M:\n"
    "                    _pull = _bd\n"
    "                    _dx, _dy = self.home_x + _bt[\"ned\"][0], self.home_y + _bt[\"ned\"][1]\n"
    "            if _pull is not None and sim_t - getattr(self, '_desc_anchor_log_t', -99.0) > 2.0:\n"
    "                self._desc_anchor_log_t = sim_t\n"
    "                self.get_logger().info(\n"
    "                    f\"  [降高锚点] 用已确认桶估计 NED({_dx - self.home_x:.2f},{_dy - self.home_y:.2f})\"\n"
    "                    f\" (距名义点 {_pull:.2f}m) 代替名义点 NED({tx:.2f},{ty:.2f})\")\n"
)

# C) 下降/保持点改用 _dx/_dy
C_ANCHOR = "            self._pub_pos(self.home_x + tx, self.home_y + ty, _z_desc, yaw_locked)\n"
C_REPLACE = "            self._pub_pos(_dx, _dy, _z_desc, yaw_locked)   # 追加 39: 对准检测到的桶\n"

MARK = "K_DESCEND_PULL_M"


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
    out = src.replace(A_ANCHOR, A_ANCHOR + A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_INSERT + B_ANCHOR, 1)
    out = out.replace(C_ANCHOR, C_REPLACE, 1)
    # 结构不变量: 每 tick 仍必须发 setpoint (且 _pub_pos 早于下降期提前 return)
    i = out.index("\n        elif self.step == STEP_HOVER_SEARCH:")
    blk = out[i:i + 8000]
    if not (0 <= blk.find("_pub_pos(_dx, _dy, _z_desc") < blk.find("if self.z < _pre_z - 0.25:")):
        print(f"  ✗ {path}: _pub_pos(_dx,_dy...) 必须早于下降期 return ⇒ 中止", file=sys.stderr)
        sys.exit(3)
    for f_ in ("elif self.step == STEP_HOVER_SEARCH:", "elif self.step == STEP_FORWARD:"):
        if out.count(f"\n        {f_}") != src.count(f"\n        {f_}"):
            print(f"  ✗ {path}: 结构不变量破坏 ({f_})", file=sys.stderr)
            sys.exit(4)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 语法错误 {e} ⇒ 中止", file=sys.stderr)
        sys.exit(5)
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
    print(f"=== patch_k_descend_anchor_to_bucket_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(6)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
