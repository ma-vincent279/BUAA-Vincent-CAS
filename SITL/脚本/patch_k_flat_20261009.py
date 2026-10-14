#!/usr/bin/env python3
"""k 台架按首飞反馈整改 (2026-10-09): 全程 1.2m 平飞 + 前飞限速 + 视觉门

**依据**: 首飞日志 `k_mission_20261009_131112.log` (13:11) + 用户四条反馈。

| 用户反馈 | 根因 (日志证据) | 本补丁 |
|---|---|---|
| ①"没飞到 1.2m 就停下, 飞的还很高" | 飞机**确实飞到了 3m 点**(相对 home 到桶只差 0.22m), 但**桶始终没被确认** (`confirmed=0/1`→`0/0`, `TRK-DBG nbox=0` 几乎每帧) ⇒ `HOVER_SEARCH` 10s 超时 ⇒ 返航,**从没进过下降段**。5m 高时桶只有 18~24px, YOLO 断续 | 全程 1.2m ⇒ 桶在正下方约 **90px**(大 4~5 倍), 检测变稳; 且 1.2m 就是投放高度 ⇒ 不需要下降 |
| ②"前飞太剧烈, 要限速" | FORWARD 段用 `CRUISE_SPEED=2.0` 起 ramp, 3m 段实测冲到 **1.53 m/s** | 新增 `K_FLIGHT_SPEED=0.6`, 前飞与返航都限速 |
| ③"要先飞到前方 3m 才能启动视觉, 不能没到 3m 就因为看到桶进伺服" | HOVER_SEARCH 一进来就允许选桶(结构上虽在到位后, 但无显式门、无法审计) | 新增 `K_VISUAL_GATE_M=0.30`: 沿航向已飞距离 `travel < FORWARD_DIST-0.30` 时**禁止选桶/进伺服**, 并每 2s 打一行 `[视觉门]` 日志 |
| ④"全程锁定 1.2m, 不要一上一下" | 原: 爬 5m 巡航 → 降到 1.2m 投 → **爬回 5m** → 再降 → 两上两下 | 新增 `K_FLIGHT_ALT=1.2` 作为**飞行高度**; `PRE_SERVO_HEIGHT` 跟它同高 ⇒ 取消下降段; 投完 CLIMB 段高度已够 ⇒ 立即完成, 全趟平飞 |

**⚠ 为什么不动 `CRUISE_HEIGHT`**: 它同时是伺服 PID 的**增益基准** (`height_scale = dz_h/CRUISE_HEIGHT`,
见 VISUAL_SERVO / DROP 段) ⇒ 把它从 5.0 改 1.2 会让低空增益放大 **4.2 倍**。所以保持 5.0 只当增益基准,
飞行高度另立 `K_FLIGHT_ALT`。

**只改 k** (SITL + fly 两份); `l` = 全任务 (34m 转场 + 侦察区) 不适用低空平飞, 保持原样。
`check_drop_portability` 的差异清单已同步登记 (见 KNOWN_DIFF)。

用法:
    python3 scripts/patch_k_flat_20261009.py            # 就地打补丁 (自动备份)
    python3 scripts/patch_k_flat_20261009.py --check    # 只检查, 不改任何东西
"""
import ast
import datetime
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
TARGETS = [ROOT / "src/drone_drl/drone_drl/k.py",
           ROOT / "fly/src/fly/fly/k.py"]

# (标签, 锚点(字面串), 替换串) —— 每处锚点必须恰好命中 1 次
PATCHES = [
    # ---- ④ 全程 1.2m: 新增飞行高度常量 (并说明为什么不动 CRUISE_HEIGHT) ----
    ("④ 新增 K_FLIGHT_ALT",
     "CRUISE_HEIGHT = 5.0",
     "CRUISE_HEIGHT = 5.0     # ⚠ 不要把它当'飞行高度'改! 它同时是伺服 PID 的**增益基准**\n"
     "                        #   (height_scale = dz_h / CRUISE_HEIGHT, 见 VISUAL_SERVO / DROP 段)\n"
     "                        #   ⇒ 改它会按比例改 KP/KD。飞行高度请改 K_FLIGHT_ALT。\n"
     "K_FLIGHT_ALT  = 1.2     # 2026-10-09 实机反馈①④: k **全程锁定 1.2m**\n"
     "                        #   原: 爬 5m 巡航 → 降到 1.2m 投 → 爬回 5m → 再降 (两上两下)\n"
     "                        #   1.2m 也是投放高度 ⇒ 不需要下降段; 且桶在正下方约 90px (5m 时仅 ~18px)"),

    ("④ cruise_z 用 K_FLIGHT_ALT",
     "        cruise_z = self._takeoff_z - CRUISE_HEIGHT",
     "        cruise_z = self._takeoff_z - K_FLIGHT_ALT"),

    ("④ TAKEOFF 爬升目标用 K_FLIGHT_ALT",
     "z_ramp = self._takeoff_z - min(CRUISE_HEIGHT, climb_elapsed * TAKEOFF_CLIMB_RATE)",
     "z_ramp = self._takeoff_z - min(K_FLIGHT_ALT, climb_elapsed * TAKEOFF_CLIMB_RATE)"),

    ("④ PRE_SERVO_HEIGHT 跟飞行高度同高 (取消下降段)",
     "PRE_SERVO_HEIGHT = 2.5     # 预伺服下降目标高度, 低空 pinhole 更准",
     "PRE_SERVO_HEIGHT = K_FLIGHT_ALT   # 2026-10-09 反馈④: 与飞行高度同高 ⇒ 预伺服段不再下降\n"
     "                            #   (原 2.5m: 5m 巡航 → 降到 2.5 → 再降到 1.2, 一上一下)"),

    # ---- ② 前飞/返航限速 ----
    ("② 新增 K_FLIGHT_SPEED",
     "CRUISE_SPEED    = 2.0",
     "CRUISE_SPEED    = 2.0\n"
     "K_FLIGHT_SPEED  = 0.6     # 2026-10-09 实机反馈②: k 场地小, 前飞/返航限速\n"
     "                          #   原 FORWARD 段用 CRUISE_SPEED=2.0 起 ramp, 3m 段实测冲到 1.53m/s (太剧烈)"),

    ("② FORWARD 段用 K_FLIGHT_SPEED",
     "            ratio = max(FORWARD_MIN_SPEED / CRUISE_SPEED, min(1.0, effective_dist / FORWARD_RAMP_DIST))\n"
     "            speed = CRUISE_SPEED * ratio",
     "            # 2026-10-09 反馈②: 前进段限速 (原 CRUISE_SPEED=2.0 ⇒ 3m 段实测 1.53m/s)\n"
     "            ratio = max(min(FORWARD_MIN_SPEED, K_FLIGHT_SPEED) / K_FLIGHT_SPEED,\n"
     "                        min(1.0, effective_dist / FORWARD_RAMP_DIST))\n"
     "            speed = K_FLIGHT_SPEED * ratio"),

    ("② 返航段也用 K_FLIGHT_SPEED",
     "vx, vy, _ = self._fly_toward(self.home_x, self.home_y, RETURN_SPEED)",
     "vx, vy, _ = self._fly_toward(self.home_x, self.home_y,\n"
     "                                          min(RETURN_SPEED, K_FLIGHT_SPEED))  # 反馈②: 低空小场地限速"),

    # ---- ③ 视觉门: 必须先飞到前方 FORWARD_DIST 处 ----
    ("③ 新增 K_VISUAL_GATE_M",
     "HOVER_SEARCH_SETTLE_TIME = 1.5  # 驻留等 MPC 刹车, 期间跳过门控",
     "HOVER_SEARCH_SETTLE_TIME = 1.5  # 驻留等 MPC 刹车, 期间跳过门控\n"
     "K_VISUAL_GATE_M = 0.30  # 2026-10-09 实机反馈③: 沿航向已飞距离 < FORWARD_DIST-该值 时\n"
     "                        #   **禁止选桶/进伺服** ⇒ 必须先真正飞到前方 3m 处才启动视觉"),

    ("③ HOVER_SEARCH 加视觉门",
     '            confirmed = [t for t in self._trackers if t["confirmed"]]',
     "            # 2026-10-09 反馈③: 视觉门 —— 必须先飞满 FORWARD_DIST 才允许选桶进伺服,\n"
     "            #   不允许'还没到就因为看到桶'提前进伺服。travel = 沿航向(HEADING_DEG)已飞距离。\n"
     "            _travel_fwd = ((self.x - self.home_x) * math.cos(self._heading0)\n"
     "                           + (self.y - self.home_y) * math.sin(self._heading0))\n"
     "            if _travel_fwd < FORWARD_DIST - K_VISUAL_GATE_M:\n"
     "                if sim_t - getattr(self, '_vg_log_t', -99.0) > 2.0:\n"
     "                    self._vg_log_t = sim_t\n"
     "                    self.get_logger().info(\n"
     "                        f\"  [视觉门] 已飞 {_travel_fwd:.2f}m < {FORWARD_DIST - K_VISUAL_GATE_M:.2f}m\"\n"
     "                        f\" ⇒ 暂不选桶/不进伺服 (等到位)\")\n"
     "                return\n"
     '            confirmed = [t for t in self._trackers if t["confirmed"]]'),

    # ---- 启动日志行改报飞行高度 ----
    ("启动日志报 K_FLIGHT_ALT",
     'f"cruise={CRUISE_HEIGHT}m drop={DROP_HEIGHT}m (no flight boundary)")',
     'f"alt={K_FLIGHT_ALT}m (全程平飞) drop={DROP_HEIGHT}m '
     'fwd_speed={K_FLIGHT_SPEED}m/s (no flight boundary)")'),

    # ---- 版本号 ----
    ("MISSION_REVISION → k-flat",
     'MISSION_REVISION = "competition-water-20261009-r1-k-P1P2"',
     'MISSION_REVISION = "competition-water-20261009-r1-k-flat"'),
]


def main():
    check_only = "--check" in sys.argv
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    todo_total = 0

    for path in TARGETS:
        assert path.exists(), f"不存在: {path}"
        text = path.read_text(encoding="utf-8")
        todo = []
        for name, anchor, repl in PATCHES:
            if repl in text:                               # 幂等: 目标形式已在 ⇒ 跳过
                print(f"  · {path.name}: {name} 已是目标形式, 跳过")
                continue
            n = text.count(anchor)
            assert n == 1, f"{path.name}: 锚点 '{name}' 命中 {n} 次 (要求恰好 1)"
            todo.append((name, anchor, repl))
        if not todo:
            print(f"  ✓ {path.name}: 无需改动")
            continue
        if check_only:
            for name, _a, _r in todo:
                print(f"  ✗ {path.name}: {name} 需要改")
            todo_total += len(todo)
            continue

        shutil.copy2(path, f"{path}.bak.{ts}_flat")
        new = text
        for name, anchor, repl in todo:
            new = new.replace(anchor, repl, 1)
            print(f"  ✓ {path.name}: {name}")
        # 二次自检: 每条替换都只出现 1 次, 且文件仍能解析
        for name, anchor, repl in PATCHES:
            assert new.count(repl) == 1, f"{name}: 目标形式出现 {new.count(repl)} 次 (要求 1)"
        ast.parse(new)
        path.write_text(new, encoding="utf-8")
        print(f"    (备份 {path.name}.bak.{ts}_flat)")
        todo_total += len(todo)

    print()
    if check_only:
        print(f"检查完成: {todo_total} 处待改" if todo_total else "检查完成: 已是目标形式 ✓")
        return 1 if todo_total else 0
    print("完成。下一步:")
    print("  colcon build --packages-select drone_drl")
    print("  python3 scripts/check_drop_portability_20261009.py k l")
    print("  python3 scripts/check_drop_portability_20261009.py k l --fly")
    print("  bash scripts/test_k.sh        # SITL 验一轮 (全程 1.2m 平飞)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
