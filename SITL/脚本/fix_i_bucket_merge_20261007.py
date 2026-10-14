#!/usr/bin/env python3
"""2026-10-07 晚: 修 i 的"第 2 投被 refine 合并到第 1 个桶" + 按实飞落点更新 B1 补偿.

实飞证据 (10.7/i_mission_20261007_090632.log, 板载 i.py sha 9321fb43):
  · 5m 选桶: A=(-29.99,-16.47)[area450] / B=(-28.80,-17.74)[area360], 相距 1.74m
  · refine 时 `身份框=0/40` (目标自己的追踪器已超时消失) ⇒ 走 0.60m 兜底:
      真目标 A 的框在 1.3m 处被拒, 邻居 B 的框在 0.5m 处被采纳 ⇒ 瞄准点被搬到 B (Δ=1.32m)
  · 1.2m 冻结估计也在 B ⇒ 第 1 发落在 B; 第 2 桶 target=B (与已投 hold 点差 0.57m)
    ⇒ TRACK 原地不动 ⇒ 悬停在已投桶上方, 唯一的框被 SERVED_EXCLUDE_M 排除 ⇒ 必双投
对比 10-06 两趟 (成功): 3 个确认框 / 两目标间距 2.06m / refine 修正量仅 1~3cm (身份框一直在)

本脚本(两份 i.py 同步) :
  P1 `POSITION_REFINE_IDENTITY_MAX` 0.60 -> 0.25 (收紧到"同桶判据" ID_MATCH 量级)
  P2 兜底框新增"明确性"判据: 必须比"其它已选目标"更近本目标, 否则不采纳 (不猜身份)
  P3 新增 `SERVED_DUP_M = 1.00`
  P4 CLIMB->TRACK 出发前: 新目标距已投桶 < 1.0m ⇒ WARN + 放弃本桶去侦察区 (不再白等 20s 兜底)
  P5 i 模式释放路径补 `SERVED +1` 日志 (此前只有 Tier 路径打)
  P6 i 模式等样本时每 5s 打一条诊断 WARN (提示"画面里只剩已投桶")
  P7 补偿按两点拟合更新: `Y_B1` +0.13->+0.18, `XF_B1/B2` -0.05->0.00 (`Y_B2` 无新数据, 不动)

用法: python3 scripts/fix_i_bucket_merge_20261007.py
回滚: 用打印的 .bak.<时间戳>_bucket_merge 覆盖回两份 i.py, 再 colcon build
"""
import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/home/vincent/SITL")
FILES = [ROOT / "src/drone_drl/drone_drl/i.py", ROOT / "fly/src/fly/fly/i.py"]
STAMP = time.strftime("%Y%m%d_%H%M%S")

PATCHES = [
    # ---------------- P1 门限收紧 ----------------
    ("POSITION_REFINE_IDENTITY_MAX = 0.60  # 2026-10-07: 1.5->0.60, 且**只管非身份框**.",
     "POSITION_REFINE_IDENTITY_MAX = 0.25  # 2026-10-07 晚: 0.60->0.25 (收紧到\"同桶判据\" ID_MATCH 量级).\n"
     "                                     #   实飞证据: 身份框失效时, 兜底 0.60m 认了 0.5m 处的**邻居桶**,\n"
     "                                     #   而真目标自己的框在 1.3m 处被拒 ⇒ 瞄准点被搬到隔壁 ⇒ 两目标合并\n"
     "                                     #   ⇒ 第 2 投悬停在已投桶上方 (必废). 见 docs/project_log 10-07 追加 8.",
     "P1 兜底门限 0.60 -> 0.25"),

    # ---------------- P2 兜底"明确性" ----------------
    ("                    if not same_id and geo_dist >= POSITION_REFINE_IDENTITY_MAX:\n"
     "                        continue\n",
     "                    if not same_id and geo_dist >= POSITION_REFINE_IDENTITY_MAX:\n"
     "                        continue\n"
     "                    # 2026-10-07 晚: 兜底框还须\"明确属于本目标\" —— 它到本目标瞄准点的距离必须小于\n"
     "                    #   它到\"其它已选目标\"的距离, 否则宁可不修正 (不猜身份).\n"
     "                    if not same_id:\n"
     "                        _amb = False\n"
     "                        for _oi, _ot in enumerate(getattr(self, '_target_buckets', None) or []):\n"
     "                            if _oi == self._bucket_idx:\n"
     "                                continue\n"
     "                            if math.hypot(bx - _ot[0], by - _ot[1]) < geo_dist:\n"
     "                                _amb = True\n"
     "                                break\n"
     "                        if _amb:\n"
     "                            continue\n",
     "P2 兜底框明确性判据"),

    # ---------------- P3 新常量 ----------------
    ("SERVED_EXCLUDE_M      = 0.60  # A: 已投过的桶: 落在此半径内的框一律不作候选 (桶半径 0.125m)\n",
     "SERVED_EXCLUDE_M      = 0.60  # A: 已投过的桶: 落在此半径内的框一律不作候选 (桶半径 0.125m)\n"
     "SERVED_DUP_M          = 1.00  # 2026-10-07 晚: 新目标距\"已投过的桶\"小于此值 ⇒ 判为双投同一桶,\n"
     "                              #   放弃该桶 (现场桶距 ~1.7m; 估计收敛到一起时第 2 投必废)\n",
     "P3 新增 SERVED_DUP_M"),

    # ---------------- P4 撞车早判 ----------------
    ("                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)\n"
     "                self._target_img_idx = -1\n",
     "                self._refresh_target_from_tracker()        # C: 立刻用活追踪器修正 (选桶估计可能偏 ~1m)\n"
     "                # 2026-10-07 晚: 新目标若与\"已投过的桶\"重合 ⇒ 再投就是双投同一个桶 (无意义),\n"
     "                #   直接放弃本桶去侦察区, 不再白等 20s 兜底. (10-07 实飞: 两目标只差 0.57m)\n"
     "                _dup_d = None\n"
     "                for _sx, _sy in self._served_targets:\n"
     "                    _dd = math.hypot(self._current_target_ned[0] - _sx,\n"
     "                                     self._current_target_ned[1] - _sy)\n"
     "                    if _dd < SERVED_DUP_M:\n"
     "                        _dup_d = _dd\n"
     "                        break\n"
     "                if _dup_d is not None:\n"
     "                    self.get_logger().warn(\n"
     "                        f\"⚠ 目标与已投桶重合 d={_dup_d:.2f}m < {SERVED_DUP_M:.1f}m \"\n"
     "                        f\"(target NED({self._current_target_ned[0]:.2f},{self._current_target_ned[1]:.2f})) \"\n"
     "                        f\"⇒ 放弃本桶, 直接去侦察区 (防双投同一桶)\")\n"
     "                    self._enter_step(STEP_SCOUT)\n"
     "                    return\n"
     "                self._target_img_idx = -1\n",
     "P4 目标撞已投桶早判"),

    # ---------------- P5 i 模式 SERVED 日志 ----------------
    ("                        self._served_targets.append((self._i_hold_x, self._i_hold_y))\n",
     "                        self._served_targets.append((self._i_hold_x, self._i_hold_y))\n"
     "                        self.get_logger().info(\n"
     "                            f\"  SERVED +1 (i模式) hold=NED({self._i_hold_x:.3f},{self._i_hold_y:.3f}) \"\n"
     "                            f\"→ 共 {len(self._served_targets)} 个已投\")\n",
     "P5 i模式补 SERVED 日志"),

    # ---------------- P6 等样本诊断 ----------------
    ("                        self._pub_pos(self.x, self.y, drop_z, self.heading)   # 等检测: 原地保持\n",
     "                        # 2026-10-07 晚: 诊断 —— 等不到样本时, 说明画面里可能只剩\"已投过的桶\"\n"
     "                        #   (那些框被 SERVED_EXCLUDE_M 排除 ⇒ 永不新鲜 ⇒ 只能等 20s 兜底释放)\n"
     "                        if (_t_wait > 5.0\n"
     "                                and sim_t - getattr(self, '_i_wait_log_t', -99.0) > 5.0):\n"
     "                            self._i_wait_log_t = sim_t\n"
     "                            _nsrv = sum(1 for _b in self._bucket_ests\n"
     "                                        if self._is_served(_b[0], _b[1]))\n"
     "                            self.get_logger().warn(\n"
     "                                f\"⚠ i模式等待样本 {_t_wait:.1f}s \"\n"
     "                                f\"(样本 {len(self._i_samp)}/{I_CALM_SAMPLES}): 画面 \"\n"
     "                                f\"{len(self._bucket_ests)} 框中 {_nsrv} 个属已投桶; \"\n"
     "                                f\"{I_RELEASE_TIMEOUT:.0f}s 将无样本兜底释放\")\n"
     "                        self._pub_pos(self.x, self.y, drop_z, self.heading)   # 等检测: 原地保持\n",
     "P6 等样本诊断 WARN"),

    # ---------------- P7 补偿 ----------------
    ("PAYLOAD_OFFSET_BODY_Y_B1 = +0.13    # 2026-10-06 按 4 投实测重标: +0.25 时右瓶落点偏左 11~15cm (补偿约1:1生效)",
     "PAYLOAD_OFFSET_BODY_Y_B1 = +0.18    # 2026-10-07 晚两点拟合: Y=+0.25→落点偏左 13cm (10-06 四投);\n"
     "                                    #   Y=+0.13→落点偏右 10cm (10-07 实飞, 距桶心 11.2cm 擦边进桶)\n"
     "                                    #   ⇒ 斜率≈1.9 ⇒ 零点 ≈ +0.18 (1:1 模型给 +0.23; 取经验拟合值)",
     "P7a Y_B1 +0.13 -> +0.18"),

    ("PAYLOAD_OFFSET_BODY_XF_B1 = -0.05    # 2026-10-06 符号修正: 10-06 四投在 XF=+0.04 下全偏'近侧(朝起飞点)'8.5~22.5cm",
     "PAYLOAD_OFFSET_BODY_XF_B1 = 0.00    # 2026-10-07 晚两点拟合: XF=+0.04→偏近侧 8cm (10-06 四投);\n"
     "                                    #   XF=-0.05→偏远侧 5cm (10-07 实飞) ⇒ 零点 ≈ -0.015 ⇒ 取 0.00",
     "P7b XF_B1 -0.05 -> 0.00"),

    ("PAYLOAD_OFFSET_BODY_XF_B2 = -0.05    # 2026-10-06 同上 (两瓶共用纵向值)",
     "PAYLOAD_OFFSET_BODY_XF_B2 = 0.00    # 2026-10-07 晚: 同 XF_B1 (两瓶前后几何相同); 纵向证据来自 10-06 四投\n"
     "                                    #   (两瓶落点都偏近侧) 与 10-07 单投 (偏远侧 5cm)",
     "P7c XF_B2 -0.05 -> 0.00 (Y_B2 无新数据, 不动)"),
]


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> int:
    print(f"=== 补丁时间戳 {STAMP} ===")
    before = {}
    for p in FILES:
        if not p.is_file():
            sys.exit(f"[FAIL] 缺文件 {p}")
        before[p] = sha(p)
        print(f"  改前 {p.relative_to(ROOT)}  sha256={before[p]}")

    texts = {}
    for p in FILES:
        s = p.read_text(encoding="utf-8")
        if "_target_buckets', None) or []" in s:
            print(f"[跳过] {p.name} 似乎已打过本补丁")
            continue
        for old, new, tag in PATCHES:
            n = s.count(old)
            if n != 1:
                sys.exit(f"[FAIL] {p.name} / {tag}: 锚点命中 {n} 次 (要求 1)")
            s = s.replace(old, new, 1)
            print(f"  {p.relative_to(ROOT)}  {tag}: OK")
        texts[p] = s

    if not texts:
        print("无需改动"); return 0

    # 语法 + 备份 + 写回
    for p, s in texts.items():
        try:
            compile(s, str(p), "exec")
        except SyntaxError as e:
            sys.exit(f"[FAIL] {p.name} 语法错误, 未写盘: {e}")
        bak = p.with_suffix(p.suffix + f".bak.{STAMP}_bucket_merge")
        shutil.copy2(p, bak)
        p.write_text(s, encoding="utf-8")
        r = subprocess.run([sys.executable, "-m", "py_compile", str(p)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            shutil.copy2(bak, p)
            sys.exit(f"[FAIL] {p.name} py_compile 失败, 已回滚: {r.stderr}")
        print(f"  {p.relative_to(ROOT)}  写回 OK, 备份 {bak.name}, sha256={sha(p)}")

    # 两份同步检查: 抽出被改的块逐字比较
    print("\n=== 两份同步检查 (抽 5 个改动块逐字比较) ===")
    blocks = {
        "P1": "POSITION_REFINE_IDENTITY_MAX = 0.25",
        "P2": "如果宁可不修正" if False else "_amb = True",
        "P3": "SERVED_DUP_M          = 1.00",
        "P4": "⇒ 放弃本桶, 直接去侦察区 (防双投同一桶)",
        "P5": "SERVED +1 (i模式)",
        "P6": "i模式等待样本",
        "P7a": "PAYLOAD_OFFSET_BODY_Y_B1 = +0.18",
        "P7b": "PAYLOAD_OFFSET_BODY_XF_B1 = 0.00",
        "P7c": "PAYLOAD_OFFSET_BODY_XF_B2 = 0.00",
    }
    txt = {p: p.read_text(encoding="utf-8") for p in FILES}
    ok = True
    for k, needle in blocks.items():
        cnt = [t.count(needle) for t in txt.values()]
        good = all(c == 1 for c in cnt)
        ok &= good
        print(f"  {k}: 两份各命中 {cnt} {'OK' if good else 'FAIL'}")
    if not ok:
        sys.exit("[FAIL] 同步检查未通过")

    print("\n=== 补偿常量现状 (两份) ===")
    for p in FILES:
        for line in txt[p].split("\n"):
            if line.startswith(("PAYLOAD_OFFSET_BODY_Y_B1", "PAYLOAD_OFFSET_BODY_Y_B2",
                                "PAYLOAD_OFFSET_BODY_XF_B1", "PAYLOAD_OFFSET_BODY_XF_B2",
                                "POSITION_REFINE_IDENTITY_MAX", "SERVED_DUP_M")):
                print(f"  {p.name}: {line.split('#')[0].strip()}")

    print("\n下一步: python3 scripts/test_i_bucket_merge_20261007.py  (差分验收) "
          "→ colcon build --packages-select drone_drl")
    print("回滚:")
    for p in texts:
        print(f"  cp {p}.bak.{STAMP}_bucket_merge {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
