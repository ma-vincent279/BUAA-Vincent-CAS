#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_descent_fix_20261009.py — 修 [降高] 段"爬行"bug: 限速改**时间锚定** (只 k 两份)

SITL 实测 (2026-10-09 15:19, 板载/本机同一份代码跑 test_k.sh):
    [降高] H=4.98m → 1.20m (限速 1.0m/s …)   ← t=13.9s 进入 HOVER_SEARCH
    [降高] H=4.94m                             ← 2.0s 后
    [降高] H=4.89m / 4.81 / 4.73 / 4.66 / 4.59 / 4.50
    STEP HOVER_SEARCH->RETURN (prev 16.0s)     ← 16s 超时, 只降到 4.43m
    End reason: normal / Buckets dropped: 0/0  ← 从没降到 1.2m, 也没进伺服

根因: 初版写成 `_z_desc = min(_pre_z, self.z + K_DESCEND_RATE * 0.05)` —— 相对**当前实测位置**
      每拍只给 5cm ⇒ PX4 位置环看到的是 5cm 的微小误差 ⇒ 只能"慢慢跟" ⇒ 实测 **0.034 m/s**
      (命令 1.0 的 3.4%)。**限速必须时间锚定**: 设定点从**进入本步时的高度**按 1.0 m/s 走,
      误差才会累积到足以让位置环跟上 (本项目 LAND 段的"时间锚定限速"就是同一个道理)。

改动:
  A) `_enter_step`: 记录 `self._step_entry_z = self.z` (通用, 供各步限速用)
  B) 降高行改为时间锚定: `min(_pre_z, self._step_entry_z + K_DESCEND_RATE * self._step_elapsed())`
  C) `HOVER_SEARCH_TIMEOUT` 16 → 18s (下降 3.8s + 位置环跟随滞后 ~1.5s = ~5.3s, 之后仍留原 10s 搜索)

用法: python3 scripts/patch_k_descent_fix_20261009.py [--check]
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
BAK_SUFFIX = f"bak.{TS}_desc_fix"

# ── A) _enter_step 记录进入高度 ──────────────────────────────────────────
A_ANCHOR = "        self.step = step; self._step_start_time = self.get_clock().now()\n"
A_INSERT = ("        # 2026-10-09 追加 38: 记录进入本步时的高度, 供\"时间锚定限速\"用\n"
            "        #   (相对当前实测位置限速会让位置环只跟 5cm 误差 ⇒ 实测只降 0.03m/s)\n"
            "        self._step_entry_z = self.z\n")

# ── B) 降高行改时间锚定 ──────────────────────────────────────────────────
# ⚠ 锚点要**逐字**含上一版补丁插入的两行注释 (它们就在 _pre_z 与 _z_desc 之间)
B_ANCHOR = (
    "            _pre_z = self.home_z - PRE_SERVO_HEIGHT\n"
    "            # NED: z 越小越高 (5m 高 ⇒ z=home-5); 下降 = z 增大 ⇒ 每 tick 只允许增大\n"
    "            # K_DESCEND_RATE*0.05, 并用 min 夹住目标 —— 写成 max 会一步跳到目标 (全速下降)\n"
    "            _z_desc = min(_pre_z, self.z + K_DESCEND_RATE * 0.05)\n"
)
B_REPLACE = (
    "            _pre_z = self.home_z - PRE_SERVO_HEIGHT\n"
    "            # ⚠ 必须**时间锚定** (从进入本步的高度按 1m/s 走), 不能写 `self.z + rate*0.05`:\n"
    "            #   后者相对\"当前实测位置\"每拍只给 5cm ⇒ PX4 位置环只看到 5cm 误差 ⇒ 实测\n"
    "            #   只降到 0.034 m/s (SITL 2026-10-09 实测, 16s 只从 4.98 降到 4.43 就超时返航)。\n"
    "            _z_anchor = getattr(self, '_step_entry_z', None)\n"
    "            if _z_anchor is None:\n"
    "                _z_anchor = self.z\n"
    "            _z_desc = min(_pre_z, _z_anchor + K_DESCEND_RATE * self._step_elapsed())\n"
)

# ── C) HOVER_SEARCH_TIMEOUT 16 → 18 ─────────────────────────────────────
C_ANCHOR = ("HOVER_SEARCH_TIMEOUT = 16.0   # 2026-10-09: 10→16 (HOVER_SEARCH 现在含'到点后降到 1.2m'\n"
            "                              #   的 ~3.8s 下降段; 到位后仍保留原 10s 搜索窗口)\n")
C_REPLACE = ("HOVER_SEARCH_TIMEOUT = 18.0   # 2026-10-09: 10→16→18 (含'到点后降到 1.2m'的 ~3.8s\n"
             "                              #   下降段 + 位置环跟随滞后 ~1.5s; 到位后仍留原 10s 搜索窗口)\n")

MARK = "K_DESCEND_RATE * self._step_elapsed()"


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
    out = out.replace(B_ANCHOR, B_REPLACE, 1)
    out = out.replace(C_ANCHOR, C_REPLACE, 1)
    # 结构不变量: 每 tick 仍必须发 setpoint; elif 不被吞
    i = out.index("\n        elif self.step == STEP_HOVER_SEARCH:")
    blk = out[i:i + 6000]
    if not (0 <= blk.find("_pub_pos(self.home_x + tx, self.home_y + ty, _z_desc")
            < blk.find("if self.z < _pre_z - 0.25:")):
        print(f"  ✗ {path}: HOVER_SEARCH 的 _pub_pos 必须早于下降期提前 return ⇒ 中止", file=sys.stderr)
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
    print(f"=== patch_k_descent_fix_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(6)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
