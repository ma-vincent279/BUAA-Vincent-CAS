#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-07 追加 18: 只改纵向补偿 —— `XF_B1 = XF_B2 = 0.00 → +0.05` (方案 A).

用户决定: "先只做 A, 尝试通过补偿的方式修掉偏差"。(B: 冻结时优选更平的姿态 —— 本轮不做)

依据 (14:14 实飞, 板载 e305e4db):
  现场真值: 第 1 投 偏右 6cm / **偏上 11cm**; 第 2 投 横向 0 / **偏上 12cm** ⇒ 两投同向同幅偏前。
  独立通道 1 (画面原始像素): 释放前 `px=(−4,−72)` / `px=(+18,−116)` ⇒ 桶压在十字线下方 ⇒ 飞机偏前 ✓
  独立通道 2 (用户卷尺真值 vs 日志): 第 2 投飞机站位比真桶心**前 17cm** (> 桶口半径 12.5cm) ⇒ 瞄准自身已超界。
  机制: 相机光轴固定在机体前下方 ⇒ 画面中心对应地面点比机身正下方**前 `H·tan(pitch)`**;
        本趟 pitch +4.2~5.6° ⇒ 该项 ≈ **9~13cm**, 与落点 +11/+12cm 同量级 (13:02 pitch 仅 1.2~2.9° 时落点是 −9cm ⇒ 符号也对得上)。
  符号: `hold = est + Y·l̂ − XF·f̂` ⇒ **XF>0 = 保持点朝起飞点(后)移 = 落点后移** ⇒ 偏前要**加大** XF ✓
  取值: 半量 +0.05 ⇒ 预测残差 k=1 → +6.5cm / k=1.44 → +4.3cm / k=1.93 → +1.9cm (均在桶口 12.5cm 内);
        即使遇到"本来偏下"的日子 (13:02 型 −9cm) 也只会变成 −14 / −16 / −19cm —— 故只取半量, 且先飞一趟验证。

不动: `Y_B1=+0.18` / `Y_B2=0.00` / 追加 16 的全部冻结门控 / 释放判据 / 投放通道 / see.py。
用法: python3 scripts/patch_i_xf005_20261007.py <i.py> [<i.py> ...]
"""
import os
import re
import sys
import time
import shutil
import hashlib

STAMP = time.strftime("%Y%m%d_%H%M%S")
SUFFIX = ".bak.%s_xf005" % STAMP

RE_XF_B1 = re.compile(r"[ \t]*PAYLOAD_OFFSET_BODY_XF_B1 = 0\.00.*?\n"
                      r"(?=[ \t]*PAYLOAD_OFFSET_BODY_XF_B2)", re.S)
RE_XF_B2 = re.compile(r"[ \t]*PAYLOAD_OFFSET_BODY_XF_B2 = 0\.00.*?\n"
                      r"(?=\n*DROP_AFTER_DELAY)", re.S)

NEW_XF_B1 = """PAYLOAD_OFFSET_BODY_XF_B1 = +0.05    # 2026-10-07 晚五改 (14:14 实飞后, 方案 A: 只用补偿修前向偏差)
                                    #   两投落点均偏前 +11/+12cm; 画面原始像素也显示桶压在十字线下方 (飞机偏前);
                                    #   姿态项 H·tan(pitch) 本趟 ≈ +9~13cm (pitch +4.2~5.6°) 是主因 ⇒
                                    #   `XF>0` = 保持点朝起飞点移 = 落点后移 ⇒ 取 +0.05 (半量, 压掉约 5~10cm);
                                    #   预测残差 k=1→+6.5 / k=1.44→+4.3 / k=1.93→+1.9cm (桶口半径 12.5cm)
"""

NEW_XF_B2 = """PAYLOAD_OFFSET_BODY_XF_B2 = +0.05    # 2026-10-07 晚五改: 同 XF_B1 (两瓶前后几何相同; 本趟两投同向偏前)
"""

FROZEN = {"PAYLOAD_OFFSET_BODY_Y_B1": "+0.18", "PAYLOAD_OFFSET_BODY_Y_B2": "0.00",
          "I_CALM_TILT": "5.0", "I_CALM_TIMEOUT": "6.0", "I_CALM_MIN_FALLBACK": "3",
          "I_FREEZE_TRACKER_M": "1.0", "I_FREEZE_MAX_SPREAD": "0.10", "I_FREEZE_EDGE_MARGIN": "8",
          "I_SETTLE_HARD_VXY": "0.25", "I_SETTLE_HARD_MAX": "8.0", "I_SETTLE_MAX": "5.0",
          "I_PRED_M": "0.09", "I_DWELL_S": "5.0", "I_RELEASE_TIMEOUT": "20.0",
          "DROP_AFTER_DELAY": "35.0"}
EXPECT = dict(FROZEN, PAYLOAD_OFFSET_BODY_XF_B1="+0.05", PAYLOAD_OFFSET_BODY_XF_B2="+0.05")
MARKERS = ["def _i_target_tracker_ok", "def _i_roi_inside", "def _i_samp_recent",
           "def _i_samp_spread", "_agree2", "如果 _enough or _forced".replace("如果", "if")]


def sha(p):
    with open(p, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def const_of(text, name):
    m = re.search(r"^%s\s*=\s*([+-]?[0-9.]+)" % re.escape(name), text, re.M)
    return m.group(1) if m else None


def main(argv):
    targets = [p for p in argv[1:] if p.endswith(".py")]
    if not targets:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        targets = [os.path.join(root, "src/drone_drl/drone_drl/i.py"),
                   os.path.join(root, "fly/src/fly/fly/i.py")]
    print("阶段 1: 校验锚点与冻结项")
    texts = {}
    for t in targets:
        assert os.path.isfile(t), "找不到 %s" % t
        texts[t] = open(t, encoding="utf-8").read()
        print("  %s (sha %s)" % (t, sha(t)))
        for k, v in FROZEN.items():
            assert const_of(texts[t], k) == v, "%s 应为 %s, 实为 %s —— 中止" % (k, v, const_of(texts[t], k))
        for nm, rx in (("XF_B1 块", RE_XF_B1), ("XF_B2 块", RE_XF_B2)):
            n = len(rx.findall(texts[t]))
            assert n == 1, "%s 命中 %d 次 (应为 1) —— 中止" % (nm, n)
            print("    ✓ 锚点唯一: %s" % nm)
    print("\n阶段 2: 应用补丁")
    for t in targets:
        src = RE_XF_B1.sub(lambda m: NEW_XF_B1, texts[t], count=1)
        src = RE_XF_B2.sub(lambda m: NEW_XF_B2, src, count=1)
        for k, v in EXPECT.items():
            assert const_of(src, k) == v, "补丁后 %s = %s, 期望 %s —— 中止" % (k, const_of(src, k), v)
        for mk in MARKERS:
            assert mk in src, "缺少标记 %s —— 中止" % mk
        compile(src, t, "exec")
        bak = t + SUFFIX
        shutil.copy2(t, bak)
        with open(t, "w", encoding="utf-8") as f:
            f.write(src)
        os.utime(t, None)
        print("  %s\n    备份 %s\n    新 sha %s" % (t, os.path.basename(bak), sha(t)))
    print("\n全部完成, 备份后缀 %s" % SUFFIX)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
