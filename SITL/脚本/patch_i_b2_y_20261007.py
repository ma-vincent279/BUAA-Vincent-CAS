#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-07 追加 11: i 项目 —— 只重标 B2 横向补偿 + 放宽平静帧门控.

依据 (10-07 11:00 实飞, 同趟两投对照):
  B1 (Y=+0.18) 落点 偏右 5.0cm      —— 不动 (五点零点 k=1 → +0.172 / k=1.92 → +0.185, 现值已在最优 1cm 内)
  B2 (Y=+0.12) 落点 偏左 14.1cm     —— B2 三趟同补偿值均偏左 (10-06: +22.5 / ≈0; 10-07: +14.1, 均值 +12.2cm)
      ⇒ 两瓶所需补偿相差 ~22cm (镜像挂载 ±11cm + 共有右偏 ~15cm), 而现值只差 6cm ⇒ 错的是 B2.
      ⇒ 零点: k=1 → 0.00 / k=1.92 → +0.057; 取 +0.04 (两模型残差 +4.2 / −3.3cm, 都在桶内; 镜像模型独立给 +0.04)
  XF_B1 / XF_B2 不动 (0.00):
      纵向证据不稳定 (B2: 10-06 后 8.5cm → 10-07 前 14.1cm, 变号; B1: −8 / +5 / +3), 且两瓶前后几何相同
      (9-27 结论) ⇒ 无"稳定的每瓶纵向偏置"可言, 不按单发调常数.
  I_CALM_TILT 3.0 → 5.0:
      11:00 两投冻结全部只用单帧 (B1 `[兜底:无平静样本]`, B2 `(平静帧中值(校准1帧))`) —— pitch 中值 3.7° 把
      5 帧窗口一票否决 ⇒ 单帧估计噪声 (~±10cm) 直接进落点. 放宽后能真正取到 5 帧中值.

不动: Y_B1 / XF_B1 / XF_B2 / 释放判据 / 释放通道 / 供弹机构 / 其它任何常量.
用法: python3 scripts/patch_i_b2_y_20261007.py <i.py> [<i.py> ...]
"""
import os
import re
import sys
import time
import hashlib
import shutil

STAMP = time.strftime("%Y%m%d_%H%M%S")
SUFFIX = ".bak.%s_b2_y04" % STAMP

# (锚点旧串, 新串, 说明) —— 必须每份文件恰好命中 1 次
PATCHES = [
    ("PAYLOAD_OFFSET_BODY_Y_B2 = +0.12",
     "PAYLOAD_OFFSET_BODY_Y_B2 = +0.04",
     "B2 横向补偿 +0.12 → +0.04"),
    ("I_CALM_TILT      = 3.0",
     "I_CALM_TILT      = 5.0",
     "平静帧门控 3.0° → 5.0°"),
]

# 注释刷新 (锚点行尾的原注释整段替换, 保证"读代码即知依据")
COMMENT_OLD_B2 = "    # 2026-10-03 保守(方案C): 0.048->+0.12 (实测左瓶偏右 ~0.10m)"
COMMENT_NEW_B2 = ("    # 2026-10-07 晚: 0.12->+0.04 —— 三趟同值落点均偏左 (10-06 +22.5/≈0, 10-07 +14.1, 均值 +12.2cm);\n"
                  "                                   # k=1 零点 0.00 / k=1.92 零点 +0.057; 镜像模型(右偏~15 共有 + 挂载 ±11) 独立给 +0.04\n")
COMMENT_OLD_CALM = "    # A/B: 冻结采样允许的最大 |roll|/|pitch| (deg; 10.6 回放: 2.0 太严, 无 5 帧平静窗口)"
COMMENT_NEW_CALM = ("    # A/B: 冻结采样允许的最大 |roll|/|pitch| (deg; 10.6 回放: 2.0 太严; 10-07 11:00 pitch 中值 3.7°\n"
                    "             #      使 3.0 仍一票否决 ⇒ 两投退化为单帧冻结, 5.0 以取到真 5 帧中值)")

# 不许被改动的常量 (冻结 B1 / XF)
FROZEN = {
    "PAYLOAD_OFFSET_BODY_Y_B1": "+0.18",
    "PAYLOAD_OFFSET_BODY_XF_B1": "0.00",
    "PAYLOAD_OFFSET_BODY_XF_B2": "0.00",
}
# 改后必须成立的值
EXPECT = {
    "PAYLOAD_OFFSET_BODY_Y_B1": "+0.18",
    "PAYLOAD_OFFSET_BODY_Y_B2": "+0.04",
    "PAYLOAD_OFFSET_BODY_XF_B1": "0.00",
    "PAYLOAD_OFFSET_BODY_XF_B2": "0.00",
    "I_CALM_TILT": "5.0",
    "I_CALM_SAMPLES": "5",
}


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

    # 0) 前置断言: 改之前 Frozen 常量必须是原值, 防止在错的文件上打补丁
    for k, v in FROZEN.items():
        have = const_of(src, k)
        assert have == v, "%s 应为 %s, 实为 %s —— 目标文件不符, 中止" % (k, v, have)

    # 1) 常量补丁
    for old, new, desc in PATCHES:
        n = src.count(old)
        assert n == 1, "锚点 '%s' 命中 %d 次 (应为 1) —— 中止" % (old, n)
        src = src.replace(old, new)
        print("  ✓ %s" % desc)

    # 2) 注释刷新
    for old, new, what in ((COMMENT_OLD_B2, COMMENT_NEW_B2, "Y_B2 注释"),
                           (COMMENT_OLD_CALM, COMMENT_NEW_CALM, "I_CALM_TILT 注释")):
        n = src.count(old)
        assert n == 1, "%s 锚点命中 %d 次 (应为 1) —— 中止" % (what, n)
        src = src.replace(old, new)

    # 3) 后置断言: 全部常量逐位核对
    for k, v in EXPECT.items():
        have = const_of(src, k)
        assert have == v, "补丁后 %s = %s, 期望 %s —— 中止" % (k, have, v)
    # 语法自检
    compile(src, path, "exec")

    bak = path + SUFFIX
    shutil.copy2(path, bak)
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    os.utime(path, None)   # 防 colcon 因时间戳跳过
    print("  备份 %s\n  新 sha %s" % (os.path.basename(bak), sha(path)))
    return True


def main(argv):
    targets = [p for p in argv[1:] if p.endswith(".py")]
    if not targets:
        here = os.path.dirname(os.path.abspath(__file__))
        root = os.path.dirname(here)
        targets = [os.path.join(root, "src/drone_drl/drone_drl/i.py"),
                   os.path.join(root, "fly/src/fly/fly/i.py")]
    for t in targets:
        assert os.path.isfile(t), "找不到 %s" % t
        patch_one(t)
    print("\n全部完成, 备份后缀 %s" % SUFFIX)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
