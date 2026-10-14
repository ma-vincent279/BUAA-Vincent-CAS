#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_alt_5m_20261009.py — k 改成用户指定剖面: **5m 巡航 → 前进 3m → 降到 1.2m 投弹** (只 k 两份)

用户指令 (2026-10-09): "🕗高度抬到5米，然后前进3米，降高到1.2米投弹"

为什么这样做 (追加 36 的实测结论):
  近地高度估计存在 ~0.07m/s 级漂移 (14:41 趟: kill switch 后电机已停、飞机在地面,
  飞控自报 H 仍在 12s 里"降"了 0.85m) ⇒ **全程 1.2m 平飞 = 长时间待在近地坏估计区**,
  物理高度会一路偏低直到贴地拖行; 5m 巡航段估计可靠 (首飞自报 4.88~5.10 与现场目视一致),
  只在**桶正上方**降到 1.2m 投弹 ⇒ 近地停留时间最短 (这正是 10-07 用 i 线 1.2m 投放 2/2 命中的剖面)。

改动 (全部只动 k 的高度链路, 不碰任何落点几何/补偿/门控):
  A) `K_FLIGHT_ALT` 1.2 → **5.0** (巡航高度: TAKEOFF 直接爬到 5m, FORWARD/HOVER_SEARCH/TRACK/CLIMB 都在 5m)
  B) `PRE_SERVO_HEIGHT` `= K_FLIGHT_ALT` → `= DROP_HEIGHT` (**1.2m**): 下降目标 = 投弹高度
     (POSITION_REFINE 原有的限速下降 `max(pre_z, z - SERVO_DESCENT_SPEED*0.05)` 继续复用)
  C) HOVER_SEARCH: 到点后**先限速降到 1.2m 再找桶** (新增 `K_DESCEND_RATE=1.0` m/s), 下降期间不做
     任何门控/不计"零检测" —— 否则会卡在 5m 空等 (首飞实测: 5m 时桶只 18~24px, conf=0.5 出不了框)
  D) `HOVER_SEARCH_TIMEOUT` 10.0 → **16.0** (给下降 ~3.8s 留出预算, 到位后仍保留原 10s 搜索窗口)
  E) 启动日志行同步 (原文案写死"全程平飞", 会误导)

用法: python3 scripts/patch_k_alt_5m_20261009.py [--check]
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
BAK_SUFFIX = f"bak.{TS}_alt5m"

# ── A) 巡航高度 1.2 → 5.0 ────────────────────────────────────────────────
A_ANCHOR = "K_FLIGHT_ALT  = 1.2     # 2026-10-09 实机反馈①④: k **全程锁定 1.2m**\n"
A_INSERT = (
    "K_FLIGHT_ALT  = 5.0     # 2026-10-09 用户指定剖面: **5m 巡航 → 前进 3m → 降到 1.2m 投弹**\n"
    "                        #   (原 1.2m 全程平飞已废: 近地高度估计有 ~0.07m/s 漂移 ⇒ 长时间\n"
    "                        #    待在近地会贴地拖行; 见 docs/project_log.md 2026-10-09 追加 36)\n"
)

# ── B) 下降目标 = 投弹高度 ───────────────────────────────────────────────
B_ANCHOR = "PRE_SERVO_HEIGHT = K_FLIGHT_ALT   # 2026-10-09 反馈④: 与飞行高度同高 ⇒ 预伺服段不再下降\n"
B_INSERT = (
    "PRE_SERVO_HEIGHT = DROP_HEIGHT    # 2026-10-09 用户指定剖面: 下降目标 = 投弹高度 1.2m\n"
    "                                  #   (POSITION_REFINE 原有的限速下降 SERVO_DESCENT_SPEED 复用)\n"
)

# ── C) 下降限速常量 ─────────────────────────────────────────────────────
C_ANCHOR = "TAKEOFF_CLIMB_RATE   = 0.8    # 竖直爬升速率 (m/s)\n"
C_INSERT = "K_DESCEND_RATE       = 1.0    # 2026-10-09: HOVER_SEARCH 到点后下降到投弹高度的限速 (m/s)\n"

# ── D) HOVER_SEARCH_TIMEOUT 10 → 16 (给下降留预算) ───────────────────────
D_ANCHOR = "HOVER_SEARCH_TIMEOUT = 10.0   # 目标点搜索超时, 无有效目标则返航\n"
D_INSERT = ("HOVER_SEARCH_TIMEOUT = 16.0   # 2026-10-09: 10→16 (HOVER_SEARCH 现在含'到点后降到 1.2m'\n"
            "                              #   的 ~3.8s 下降段; 到位后仍保留原 10s 搜索窗口)\n")

# ── E) HOVER_SEARCH: 到点先降高 ──────────────────────────────────────────
# ⚠ 锚点只用**代码行**: 两副本的注释不同 (fly 副本 HOVER_SEARCH 多了
#   `# 驻留等 MPC 刹车, 期间跳过门控` 这类注释) ⇒ 带注释/带空行的锚点在 fly 命中 0 次。
E_ANCHOR = "            self._pub_pos(self.home_x + tx, self.home_y + ty, cruise_z, yaw_locked)\n"
E_REPLACE = (
    "            # 2026-10-09 追加 37 (用户指定剖面 5m巡航→前进3m→降到1.2m投弹):\n"
    "            #   到点后**先限速降到 PRE_SERVO_HEIGHT(1.2m) 再找桶** —— 5m 巡航时桶只有 18~24px,\n"
    "            #   see.py conf=0.5 出不了框 (首飞实测) ⇒ 若\"等看到桶才下降\"会卡在 5m 空等超时;\n"
    "            #   本验证台的桶固定在正前方 3m ⇒ 可盲降到 1.2m 再看 (1.2m 正下方桶 ≈67px),\n"
    "            #   同时把近地停留时间压到最短 (近地估计有 ~0.07m/s 漂移, 见追加 36)。\n"
    "            #   限速写法与 POSITION_REFINE 的 `max(pre_z, z - SERVO_DESCENT_SPEED*0.05)` 同源。\n"
    "            _pre_z = self.home_z - PRE_SERVO_HEIGHT\n"
    "            # NED: z 越小越高 (5m 高 ⇒ z=home-5); 下降 = z 增大 ⇒ 每 tick 只允许增大\n"
    "            # K_DESCEND_RATE*0.05, 并用 min 夹住目标 —— 写成 max 会一步跳到目标 (全速下降)\n"
    "            _z_desc = min(_pre_z, self.z + K_DESCEND_RATE * 0.05)\n"
    "            self._pub_pos(self.home_x + tx, self.home_y + ty, _z_desc, yaw_locked)\n"
    "            if self.z < _pre_z - 0.25:\n"
    "                # 下降期间不做门控/不选桶, 也不计\"零检测\" (桶还没进视野, 计数无意义)\n"
    "                if sim_t - getattr(self, '_hs_desc_log_t', -99.0) > 2.0:\n"
    "                    self._hs_desc_log_t = sim_t\n"
    "                    self.get_logger().info(\n"
    "                        f\"  [降高] H={self.home_z - self.z:.2f}m → {PRE_SERVO_HEIGHT:.2f}m\"\n"
    "                        f\" (限速 {K_DESCEND_RATE:.1f}m/s, 到位后再找桶)\")\n"
    "                return\n"
)

# ── F) 启动日志行同步 ────────────────────────────────────────────────────
F_ANCHOR = ('            f"alt={K_FLIGHT_ALT}m (全程平飞) drop={DROP_HEIGHT}m '
            'fwd_speed={K_FLIGHT_SPEED}m/s (no flight boundary)")\n')
F_REPLACE = ('            f"alt={K_FLIGHT_ALT}m 巡航 → 前进{FORWARD_DIST}m → 降到{PRE_SERVO_HEIGHT}m 投弹"\n'
             '            f" (drop={DROP_HEIGHT}m) fwd_speed={K_FLIGHT_SPEED}m/s (no flight boundary)")\n')

MARK = "K_FLIGHT_ALT  = 5.0"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path: pathlib.Path, check: bool) -> str:
    src = path.read_text(encoding="utf-8")
    if MARK in src:
        return "SKIP (已打过, 幂等)"
    for name, anchor in (("A", A_ANCHOR), ("B", B_ANCHOR), ("C", C_ANCHOR),
                         ("D", D_ANCHOR), ("E", E_ANCHOR), ("F", F_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"  ✗ {path}: 锚点 {name} 命中 {n} 次 (要求恰 1 次) ⇒ 中止", file=sys.stderr)
            sys.exit(2)
    out = src.replace(A_ANCHOR, A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_INSERT, 1)
    out = out.replace(C_ANCHOR, C_ANCHOR + C_INSERT, 1)
    out = out.replace(D_ANCHOR, D_INSERT, 1)
    out = out.replace(E_ANCHOR, E_REPLACE, 1)
    out = out.replace(F_ANCHOR, F_REPLACE, 1)
    # 结构不变量: elif 行不能被吞 (上一轮踩过 — 编译仍通过, 只有审计抓得住)
    # ⚠ 结构不变量(踩过): HOVER_SEARCH 每个 tick 必须先发 setpoint 再 return,
    #   否则下降期间一个 setpoint 都不发 ⇒ 飞控 0.5s 无 setpoint 直接掉 OFFBOARD。
    #   注意: 必须从**状态机**那一处 (8 空格缩进) 起算, 不能用裸子串 (会命中 1Hz 日志块)
    for blk in [out[out.index("\n        elif self.step == STEP_HOVER_SEARCH:"):][:5000]]:
        _i_pub = blk.find("_pub_pos(self.home_x + tx, self.home_y + ty, _z_desc")
        _i_ret = blk.find("if self.z < _pre_z - 0.25:")
        if not (0 <= _i_pub < _i_ret):
            print(f"  ✗ {path}: HOVER_SEARCH 的 _pub_pos 必须早于下降期提前 return ⇒ 中止", file=sys.stderr)
            sys.exit(3)
    for f_ in ("elif self.step == STEP_HOVER_SEARCH:", "elif self.step == STEP_FORWARD:",
               "elif self.step == STEP_POSITION_REFINE:"):
        if out.count(f_) != src.count(f_):
            print(f"  ✗ {path}: 结构不变量破坏 ({f_}) ⇒ 中止", file=sys.stderr)
            sys.exit(3)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 打补丁后语法错误 {e} ⇒ 中止", file=sys.stderr)
        sys.exit(4)
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
    print(f"=== patch_k_alt_5m_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(5)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
