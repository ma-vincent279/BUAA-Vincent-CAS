#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026-10-07 (方案 A, 用户批准): i 的冻结保持点改成纯几何量
    hold = 桶估计 + 补偿

原式 (SITL 1706 / fly 1805, 以及下降段的 SITL 1574 / fly 1663):
    hold = 瞄准点 + (桶估计 − 飞机) + 补偿
         ≡ 桶估计 + 补偿 + (瞄准点 − 飞机)
最后那一项 = "冻结瞬间飞机还没追上瞄准点的那一段追赶余量", 会被 1:1 加到落点上.
当初加它的意图是"位置环稳态偏差 b 自动抵消"(CLAUDE.md 2026-10-06 i 条目), 但实测:

  SITL 干净轮 (2026-10-07, 344.6s, Publisher count=1):
    第 1 投: 该项 +12.2cm, 飞机随后相对保持点的沿飞行方向真实偏差 −5.4cm (符号相反!)
    第 2 投: 该项 +25.0cm, 同上 −1.8cm
  ⇒ 冻结发生时飞机正在"赶路"(还在下降/还在追瞄准点), 瞬时差距 ≠ 稳态差距 ⇒ 纯过补偿.
  10-6 实飞四投同机理 (E = lag + b, lag 4.8~13.0cm, b 1.3~14.6cm).

改动后: 落点误差 = (桶估计误差) + (释放瞬间飞机相对保持点的偏差 b) + (补偿 + 机械偏置 r)
       且释放判据 pred 变成真正的"落点预测"(原来预测的是飞到被污染的目标).
无检测兜底分支: 没有桶估计可用 ⇒ 仍退回 瞄准点 + 补偿 (与原来一致).

只改这两处 (每个文件 2 处: 下降段冻结 + 1.2m 冻结), 其余一行不动.
"""
import hashlib
import os
import py_compile
import re
import shutil
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    os.path.join(ROOT, "src/drone_drl/drone_drl/i.py"),
    os.path.join(ROOT, "fly/src/fly/fly/i.py"),
]
STAMP = time.strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = ".bak.%s_hold_geom" % STAMP

# ---------------- 锚点 1: 下降段冻结 (I_FREEZE_AT_H>0 时才活; 当前 0.0 = 关闭, 一并改保持一致) ----
OLD_CODE1 = """                        _ddx2 = _em[0] - (self.x - self.home_x)
                        _ddy2 = _em[1] - (self.y - self.home_y)
                        self._i_hold_x = (self._servo_target_x + _ddx2
                                          + _dy2 * math.sin(self.heading) - _dx2 * math.cos(self.heading))
                        self._i_hold_y = (self._servo_target_y + _ddy2
                                          - _dy2 * math.cos(self.heading) - _dx2 * math.sin(self.heading))
"""

NEW_CODE1 = """                        # 2026-10-07 (A): 保持点 = 桶估计 + 补偿 (纯几何量).
                        #   原式 `瞄准点 + (桶估计−飞机) + 补偿` ≡ `桶估计 + 补偿 + (瞄准点−飞机)`,
                        #   会把"冻结瞬间飞机还没追上瞄准点的那一段"1:1 加进落点; SITL 实测 12.2/25.0cm,
                        #   而飞机随后相对保持点的沿飞行方向真实偏差只有 5.4/1.8cm (符号还相反) ⇒ 删除.
                        _ddx2 = _em[0] - (self.x - self.home_x)      # 仅用于日志: 飞机 → 桶估计
                        _ddy2 = _em[1] - (self.y - self.home_y)
                        self._i_hold_x = (_em[0]
                                          + _dy2 * math.sin(self.heading) - _dx2 * math.cos(self.heading))
                        self._i_hold_y = (_em[1]
                                          - _dy2 * math.cos(self.heading) - _dx2 * math.sin(self.heading))
"""

OLD_LOG1 = """                            f"i模式: 下降段 %.1fm 处一次性修正 delta=({_ddx2:+.3f},{_ddy2:+.3f})m "
                            f"→ 冻结点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) 补偿B{self._drop_count + 1}"
                            f"=({_dy2:+.3f},{_dx2:+.3f}) [纯垂直下降]")
"""

NEW_LOG1 = """                            f"i模式: 下降段 %.1fm 处一次性修正 delta=({_ddx2:+.3f},{_ddy2:+.3f})m "
                            f"(桶估计={_em}) → 冻结点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) "
                            f"[纯几何: 冻结点-桶估计=({self._i_hold_x - _em[0]:+.3f},"
                            f"{self._i_hold_y - _em[1]:+.3f})] 补偿B{self._drop_count + 1}"
                            f"=({_dy2:+.3f},{_dx2:+.3f}) [纯垂直下降]")
"""

# ---------------- 锚点 2: 1.2m 冻结 (当前生效路径) ----------------
OLD_CODE2 = """                        _ddx = (_est[0] - (self.x - self.home_x)) if _est is not None else 0.0
                        _ddy = (_est[1] - (self.y - self.home_y)) if _est is not None else 0.0
                        self._i_hold_x = (self._servo_target_x + _ddx
                                          + _dy * math.sin(self.heading) - _dx * math.cos(self.heading))
                        self._i_hold_y = (self._servo_target_y + _ddy
                                          - _dy * math.cos(self.heading) - _dx * math.sin(self.heading))
"""

NEW_CODE2 = """                        # 2026-10-07 (A): 保持点 = 桶估计 + 补偿 (纯几何量) —— 删掉原来的
                        #   `+ (瞄准点 − 飞机)` 项 (它 ≡ `瞄准点 + (桶估计−飞机) + 补偿`).
                        #   实测 (SITL 干净轮): 该项 12.2cm/25.0cm, 而飞机随后相对保持点的沿飞行
                        #   方向真实偏差只有 5.4cm/1.8cm (符号还相反) ⇒ 纯过补偿.
                        #   无检测兜底: 没有桶估计可用, 仍退回 瞄准点 + 补偿.
                        _ddx = (_est[0] - (self.x - self.home_x)) if _est is not None else 0.0
                        _ddy = (_est[1] - (self.y - self.home_y)) if _est is not None else 0.0
                        _hx0, _hy0 = _est if _est is not None else (self._servo_target_x, self._servo_target_y)
                        self._i_hold_x = (_hx0
                                          + _dy * math.sin(self.heading) - _dx * math.cos(self.heading))
                        self._i_hold_y = (_hy0
                                          - _dy * math.cos(self.heading) - _dx * math.sin(self.heading))
"""

OLD_LOG2 = """                            f"i模式: 1.2m 一次性修正 delta=({_ddx:+.3f},{_ddy:+.3f})m "
                            f"(桶估计={_est}) → 冻结保持点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) "
                            f"补偿B{self._drop_count + 1}=({_dy:+.3f},{_dx:+.3f}){' [兜底:无检测]' if _est is None else ''}")
"""

NEW_LOG2 = """                            f"i模式: 1.2m 一次性修正 delta=({_ddx:+.3f},{_ddy:+.3f})m "
                            f"(桶估计={_est}) → 冻结保持点 ({self._i_hold_x:+.3f},{self._i_hold_y:+.3f}) "
                            f"[纯几何: 保持点-桶估计=({self._i_hold_x - _hx0:+.3f},"
                            f"{self._i_hold_y - _hy0:+.3f})] "
                            f"补偿B{self._drop_count + 1}=({_dy:+.3f},{_dx:+.3f}){' [兜底:无检测]' if _est is None else ''}")
"""

PATCHES = [
    ("下降段 保持点", OLD_CODE1, NEW_CODE1),
    ("下降段 日志", OLD_LOG1, NEW_LOG1),
    ("1.2m 保持点", OLD_CODE2, NEW_CODE2),
    ("1.2m 日志", OLD_LOG2, NEW_LOG2),
]

# 必须保持不动的常量 (改前改后逐字比对)
KEEP = [
    "PAYLOAD_OFFSET_BODY_Y_B1 = +0.13",
    "PAYLOAD_OFFSET_BODY_Y_B2 = +0.12",
    "PAYLOAD_OFFSET_BODY_XF_B1 = -0.05",
    "PAYLOAD_OFFSET_BODY_XF_B2 = -0.05",
    "POSITION_REFINE_IDENTITY_MAX = 0.60",
    "POSITION_REFINE_ID_MATCH_M   = 0.25",
]


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    for path in FILES:
        if not os.path.exists(path):
            sys.exit("ERROR: 缺少文件 %s" % path)
        src = open(path, encoding="utf-8").read()
        print("=" * 78)
        print("文件: %s\n  改前 sha256 = %s" % (path.replace(ROOT + "/", ""), sha(path)[:16]))

        for c in KEEP:
            if src.count(c) != 1:
                sys.exit("ERROR: 常量锚点命中 %d 次 (期望 1): %s  ← 说明文件已被改过, 停止" % (src.count(c), c))

        # 1) 逐个锚点断言唯一命中
        for name, old, _new in PATCHES:
            n = src.count(old)
            if n != 1:
                sys.exit("ERROR: 锚点 '%s' 命中 %d 次 (期望 1) —— 未做任何修改" % (name, n))
        # 2) 备份 + 替换
        bak = path + BAK_SUFFIX
        shutil.copy2(path, bak)
        new = src
        for name, old, rep in PATCHES:
            new = new.replace(old, rep, 1)
            print("  ✓ 替换: %s" % name)

        # 3) 结构与数量检查
        checks = [
            ("self._i_hold_x = (_em[0]", 1),
            ("self._i_hold_x = (_hx0", 1),
            ("[纯几何: 冻结点-桶估计=", 1),
            ("[纯几何: 保持点-桶估计=", 1),
            ("self._servo_target_x + _ddx", 0),
            ("self._servo_target_y + _ddy", 0),
            ("_hx0, _hy0 = _est if _est is not None else", 1),
        ]
        for pat, want in checks:
            got = new.count(pat)
            if got != want:
                shutil.copy2(bak, path)
                sys.exit("ERROR: 检查失败 '%s' 命中 %d (期望 %d) —— 已回滚" % (pat, got, want))

        if new == src:
            shutil.copy2(bak, path)
            sys.exit("ERROR: 替换后文件无变化 —— 已回滚")

        open(path, "w", encoding="utf-8").write(new)

        # 4) 语法编译 (失败即回滚)
        try:
            py_compile.compile(path, doraise=True)
        except py_compile.PyCompileError as e:
            shutil.copy2(bak, path)
            sys.exit("ERROR: py_compile 失败, 已回滚: %s" % e)
        print("  ✓ py_compile 通过")
        print("  改后 sha256 = %s" % sha(path)[:16])
        print("  备份: %s" % bak.replace(ROOT + "/", ""))

    # 5) 两份文件保持同步 (去掉节点名差异后逐行比对这两块)
    a = open(FILES[0], encoding="utf-8").read()
    b = open(FILES[1], encoding="utf-8").read()
    for pat in ["self._i_hold_x = (_em[0]", "self._i_hold_x = (_hx0", "[纯几何: 保持点-桶估计="]:
        if a.count(pat) != b.count(pat) or a.count(pat) != 1:
            sys.exit("ERROR: SITL 与 fly 两份不同步: '%s' %d vs %d" % (pat, a.count(pat), b.count(pat)))
    print("=" * 78)
    print("完成: 两份 i.py 已同步, 保持点 = 桶估计 + 补偿 (纯几何量)")


if __name__ == "__main__":
    main()
