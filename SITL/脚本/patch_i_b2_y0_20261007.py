#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-07 追加 14: i 项目 —— B2 横向补偿 -0.05 → 0.00 (13:02 第 2 投首次命中后的定心).

现场真值 (用户): 13:02 第 2 投**命中**, 落点 **偏右 9cm / 偏下 9cm**; 第 1 投精度很差 (未给数值).

依据 (B2 横向, 左为正 cm; 全部 213° 航班):
  (0.12, +22.5) (0.12, 0.0) (0.12, +14.1) (0.04, +13.0) (-0.05, -9.0)
  ⇒ 5 样本最小二乘: 落点 = +0.38 + 111·Y  ⇒ **k≈1.11, 零点 Y* = -0.003 m**, RMS 8.3cm
  ⇒ 只取两趟干净样本 (11:55 +0.04→左13.0 / 13:02 -0.05→右9.0): **k≈2.44, 零点 Y* = -0.013 m**
  ⇒ 两个零点都在 0.00 附近 ⇒ 现值 -0.05 **偏右 4~5cm**.
  候选 minimax (以本趟实测 -9.0cm 为基准外推, 覆盖 k=1 / 1.11 / 2.44):
      Y=0.00 → -4.0 / -3.5 / +3.2 cm (max 4.0) ✓✓ 取它
      Y=-0.01 → -5.0 / -4.6 / +0.8 (max 5.0) ; 保持 -0.05 → -9.0 (max 9.0) ✗
  物理含义: 该瓶"机械左偏 + 估计偏置"合起来已互相抵消, 故最优补偿就是 0 (不是镜像模型原始给的 -7cm) —— 以实测为准.

不动: Y_B1 (+0.18) / XF_B1 / XF_B2 (0.00) / I_CALM_TILT (5.0) / I_CALM_SAMPLES (5) / 释放判据 / 投放通道.
第 1 投本趟失准已定性为"未确认追踪器 + 单帧冻结 + 贴边裁剪"链路问题 (非补偿), 待用户批准后再动冻结门控.
用法: python3 scripts/patch_i_b2_y0_20261007.py <i.py> [<i.py> ...]
"""
import os
import re
import sys
import time
import hashlib
import shutil

STAMP = time.strftime("%Y%m%d_%H%M%S")
SUFFIX = ".bak.%s_b2_y000" % STAMP

RE_B2 = re.compile(r"^PAYLOAD_OFFSET_BODY_Y_B2 = -0\.05.*?\n(?=\n?PAYLOAD_OFFSET_BODY_XF_B1)", re.S | re.M)

NEW_B2 = (
    "PAYLOAD_OFFSET_BODY_Y_B2 = 0.00    # 2026-10-07 晚三改 (13:02 实飞第 2 投命中后): -0.05 -> 0.00\n"
    "                                   #   13:02 第 2 投首次命中: 偏右 9cm / 偏下 9cm (前两趟同瓶均偏左 13~14cm);\n"
    "                                   #   5 样本最小二乘: 落点 = +0.4 + 111·Y (k≈1.11, RMS 8.3cm) ⇒ 零点 ≈ -0.003;\n"
    "                                   #   两趟干净样本(+0.04→左13.0 / -0.05→右9.0): k≈2.44 ⇒ 零点 ≈ -0.013;\n"
    "                                   #   ⇒ 现值 -0.05 偏右 4~5cm; 取 0.00: 残差 k=1→-4.0 / k=1.11→-3.5 / k=2.44→+3.2cm\n"
    "\n"
)

FROZEN = {
    "PAYLOAD_OFFSET_BODY_Y_B1": "+0.18",
    "PAYLOAD_OFFSET_BODY_XF_B1": "0.00",
    "PAYLOAD_OFFSET_BODY_XF_B2": "0.00",
    "I_CALM_TILT": "5.0",
    "I_CALM_SAMPLES": "5",
}
EXPECT = dict(FROZEN, PAYLOAD_OFFSET_BODY_Y_B2="0.00")


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def const_of(text, name):
    m = re.search(r"^%s\s*=\s*([+-]?[0-9.]+)" % re.escape(name), text, re.M)
    return m.group(1) if m else None


def patch_one(path):
    print("=== %s (sha %s) ===" % (path, sha(path)))
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    for k, v in FROZEN.items():
        assert const_of(src, k) == v, "%s 应为 %s, 实为 %s —— 目标文件不符, 中止" % (k, v, const_of(src, k))

    hits = RE_B2.findall(src)
    assert len(hits) == 1, "Y_B2 常量+注释块命中 %d 次 (应为 1) —— 中止" % len(hits)
    src = RE_B2.sub(NEW_B2, src, count=1)
    print("  ✓ PAYLOAD_OFFSET_BODY_Y_B2 -0.05 → 0.00 (并刷新注释)")

    for k, v in EXPECT.items():
        assert const_of(src, k) == v, "补丁后 %s = %s, 期望 %s —— 中止" % (k, const_of(src, k), v)
    compile(src, path, "exec")

    bak = path + SUFFIX
    shutil.copy2(path, bak)
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    os.utime(path, None)
    print("  备份 %s\n  新 sha %s" % (os.path.basename(bak), sha(path)))


def main(argv):
    targets = [p for p in argv[1:] if p.endswith(".py")]
    if not targets:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        targets = [os.path.join(root, "src/drone_drl/drone_drl/i.py"),
                   os.path.join(root, "fly/src/fly/fly/i.py")]
    for t in targets:
        assert os.path.isfile(t), "找不到 %s" % t
        patch_one(t)
    print("\n全部完成, 备份后缀 %s" % SUFFIX)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
