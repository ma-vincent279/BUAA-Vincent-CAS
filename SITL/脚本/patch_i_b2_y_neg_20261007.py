#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-07 追加 13: i 项目 —— B2 横向补偿改为负值 (+0.04 → -0.05).

依据 (11:55 实飞, 现场真值: 第 1 投命中 / 第 2 投偏正左 13.0cm, 前后≈0):
  1) B2 四次实测横向落点 (左为正): +22.5 / 0.0 / +14.1 / +13.0cm ⇒ 均值 +12.4cm 偏左 (散布 22.5cm)
     ⇒ 现值 +0.04 下仍然系统性偏左 ⇒ 需要把保持点再往右挪.
  2) 最近两趟 (都带 5 帧中值门控) 是干净 A/B: Y=+0.12→+14.1cm / Y=+0.04→+13.0cm
     ⇒ ΔY = -8cm 只换来 -1.1cm ⇒ 单点不能定斜率, 因此取"两模型都安全"的 minimax 值.
  3) 镜像几何 (两瓶镜像安装: 右瓶释放点在相机轴右 5.5cm / 左瓶左 5.5cm; 9-27 实测动态项后 ρ_eff≈12.5cm,
     共有右偏 E≈-5.5cm) ⇒ B1 零点 = ρ-E = +18cm (与 B1 五点实测一致 ✓), B2 零点 = -ρ-E = **-7cm** ✓
     ⇒ **B2 必须为负** (对左瓶: 载荷本来就落在飞机左侧, 所以瞄准点要往右压).
  4) minimax (k=1 与 k=1.92 两种响应模型都落在桶口半径 12.5cm 内):
       +0.00 → +8.4/+4.7 ; -0.04 → +4.4/-3.0 ; **-0.05 → +3.4/-4.9** ; -0.07 → +1.4/-8.7 ; -0.09 → -0.6/-12.6 ✗
     ⇒ 稳健窗口 [-0.02, -0.07], 取中值 **-0.05** (最坏残差 4.9cm; k≈0 时与现状等价, 不会变差).
  5) 纵向 XF 不动 (本次"前后≈0" vs 11:00"+14.1cm" ⇒ 纵向是可变项/俯仰幻影, 不是常数偏置) ⇒ 0.00 保持不变
     —— 本趟结果同时验证了上一次"只改横向、不动 XF"的决定是对的.

不动: Y_B1 (+0.18, 本趟命中) / XF_B1 / XF_B2 / I_CALM_TILT (本趟两投都取到 5 帧中值 ✓) / 其它任何常量.
用法: python3 scripts/patch_i_b2_y_neg_20261007.py <i.py> [<i.py> ...]
"""
import os
import re
import sys
import time
import hashlib
import shutil

STAMP = time.strftime("%Y%m%d_%H%M%S")
SUFFIX = ".bak.%s_b2_yneg" % STAMP

RE_B2 = re.compile(
    r"^PAYLOAD_OFFSET_BODY_Y_B2 = \+0\.04[^\n]*\n"
    r"[^\n]*k=1 零点[^\n]*\n", re.M)

NEW_B2 = (
    "PAYLOAD_OFFSET_BODY_Y_B2 = -0.05    # 2026-10-07 晚二改 (11:55 实飞后): +0.04 -> -0.05\n"
    "                                   #   四次实测横向均偏左 (22.5/0/14.1/13.0cm, 均值 +12.4cm) ⇒ 需往右压;\n"
    "                                   #   镜像模型: B1 零点 +18cm(实测一致) ⇒ B2 零点 -7cm (左瓶载荷落在左侧, 瞄准点须右移);\n"
    "                                   #   minimax(k=1→+3.4cm / k=1.92→-4.9cm, 桶口 12.5cm)\n")

FROZEN = {
    "PAYLOAD_OFFSET_BODY_Y_B1": "+0.18",
    "PAYLOAD_OFFSET_BODY_XF_B1": "0.00",
    "PAYLOAD_OFFSET_BODY_XF_B2": "0.00",
    "I_CALM_TILT": "5.0",
    "I_CALM_SAMPLES": "5",
}
EXPECT = dict(FROZEN, PAYLOAD_OFFSET_BODY_Y_B2="-0.05")


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

    for k, v in FROZEN.items():                     # 前置: 冻结项必须是原值
        assert const_of(src, k) == v, "%s 应为 %s, 实为 %s —— 目标文件不符, 中止" % (k, v, const_of(src, k))

    hits = RE_B2.findall(src)
    assert len(hits) == 1, "Y_B2 常量+注释块命中 %d 次 (应为 1) —— 中止" % len(hits)
    src = RE_B2.sub(NEW_B2, src, count=1)
    print("  ✓ PAYLOAD_OFFSET_BODY_Y_B2 +0.04 → -0.05 (并刷新注释)")

    for k, v in EXPECT.items():                     # 后置: 全部常量逐位核对
        assert const_of(src, k) == v, "补丁后 %s = %s, 期望 %s —— 中止" % (k, const_of(src, k), v)
    compile(src, path, "exec")

    bak = path + SUFFIX
    shutil.copy2(path, bak)
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    os.utime(path, None)                            # 防 colcon 按时间戳跳过
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
