#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026-10-07 方案 A (冻结保持点改纯几何量) 离线验收 + 自检脚本

用法:
    python3 scripts/test_i_hold_geom_20261007.py

四节 (共 40 项检查, 全过才返回 0):
  A 源码审计   —— 两份 i.py 的改动块逐字一致 / 旧式写法已消失 / 兜底行存在 / 6 个常量未被顺手改动
  B 公式等价   —— 用旧式与新式各自算一遍: ①无检测兜底时两者**逐位相同**(证明没改坏兜底分支)
                              ②有桶估计时 新式 ≡ 桶估计+补偿, 旧式 − 新式 ≡ (瞄准点 − 飞机)
  C 日志回放   —— 两轮真实 SITL 日志: 改后轮 |hold − est − 补偿指令| ≈ 0 (mm 级) 且 |lag|≈0;
                  改前轮同量 = 冻结滞后项; 另做**独立交叉核对** —— 裁判节点自报的距离必须等于
                  我从任务日志算的 |释放点 − 桶| (两条链路, 防我自己读错日志)
  D 下游工具兼容性 —— 标定器必须同时能解析"新格式(有 [纯几何: …] 字段)"与"旧格式(10-6 实飞)"日志,
                  且两种都解析出 ≥2 投 (2026-10-07 真的踩过: 新字段把标定器正则弄瞎, 只打表头不报错)
"""
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    os.path.join(ROOT, "src/drone_drl/drone_drl/i.py"),
    os.path.join(ROOT, "fly/src/fly/fly/i.py"),
]
# 本表的用途: 证明"落地 hold=纯几何量"这次改动没有顺手动到别的常量。
# 2026-10-09 校正: 原表写的是 10-07 上午的值 (0.13/0.12/-0.05/0.60), 而当天晚些的
# 追加 12/13/14/16/18/19 已把 i 的补偿改成 0.18/0.00/+0.05 并把身份门限改到 0.25
# ⇒ 旧表从 10-07 晚起就一直是假失败。这里对齐 i.py 的**当前实际值**(i.py 自 10-07
# 之后一行未动, sha 1bc27e2a…), 恢复该断言的原本用途。
EXPECT_CONST = [
    ("PAYLOAD_OFFSET_BODY_Y_B1", "0.18"),
    ("PAYLOAD_OFFSET_BODY_Y_B2", "0.00"),
    ("PAYLOAD_OFFSET_BODY_XF_B1", "0.05"),
    ("PAYLOAD_OFFSET_BODY_XF_B2", "0.05"),
    ("POSITION_REFINE_IDENTITY_MAX", "0.25"),
    ("POSITION_REFINE_ID_MATCH_M", "0.25"),
]
PRE_A_LOG = "/tmp/i_sel_run_preA.log"
A_LOG = "/tmp/i_sel_run_A.log"
OK, BAD = [], []


def check(cond, msg):
    (OK if cond else BAD).append(msg)
    print(("  ✓ " if cond else "  ✗ ") + msg)


def block_at(path, pat, n=3):
    lines = open(path, encoding="utf-8").read().split("\n")
    for i, l in enumerate(lines):
        if re.search(pat, l):
            return [x.strip() for x in lines[i:i + n]]
    return None


# ---------------------------------------------------------------- A 源码审计
def section_a():
    print("=" * 92)
    print("A 源码审计")
    blocks = {}
    for path in FILES:
        rel = path.replace(ROOT + "/", "")
        src = open(path, encoding="utf-8").read()
        print(" -- %s" % rel)
        check(src.count("self._i_hold_x = (_em[0]") == 1, "下降段: 新式 `hold_x = _em[0] + 补偿` 命中 1 次")
        check(src.count("self._i_hold_x = (_hx0") == 1, "1.2m : 新式 `hold_x = _hx0 + 补偿` 命中 1 次")
        check(src.count("self._servo_target_x + _ddx") == 0, "旧式 `servo_target_x + _ddx` 已消失")
        check(src.count("self._servo_target_y + _ddy") == 0, "旧式 `servo_target_y + _ddy` 已消失")
        check(src.count("_hx0, _hy0 = _est if _est is not None else "
                        "(self._servo_target_x, self._servo_target_y)") == 1,
              "无检测兜底行存在且只有一处")
        check(src.count("[纯几何: 保持点-桶估计=") == 1, "新增核对日志存在 (1.2m 冻结)")
        for name, val in EXPECT_CONST:
            m = re.search(r"^%s\s*=\s*([+-]?[\d.]+)" % name, src, re.M)
            check(m is not None and float(m.group(1)) == float(val),
                  "常量 %s = %s (未被顺手改动)" % (name, val))
        blocks[rel] = (block_at(path, r"_i_hold_x = \(_em\[0\]"), block_at(path, r"_i_hold_x = \(_hx0"))
    check(blocks[FILES[0].replace(ROOT + "/", "")] == blocks[FILES[1].replace(ROOT + "/", "")],
          "两份 i.py 的改动块逐字一致 (SITL == fly)")


# ---------------------------------------------------------------- B 公式等价
def old_formula(servo, est, plane, comp, hdg_deg):
    """旧式: hold = 瞄准点 + (桶估计 − 飞机) + 补偿   (est=None ⇒ delta=0)"""
    if est is None:
        ddx = ddy = 0.0
    else:
        ddx, ddy = est[0] - plane[0], est[1] - plane[1]
    h = math.radians(hdg_deg)
    cy, cxf = comp
    return (servo[0] + ddx + cy * math.sin(h) - cxf * math.cos(h),
            servo[1] + ddy - cy * math.cos(h) - cxf * math.sin(h))


def new_formula(servo, est, comp, hdg_deg):
    """新式: hold = (桶估计 或 瞄准点) + 补偿"""
    h = math.radians(hdg_deg)
    cy, cxf = comp
    base = est if est is not None else servo
    return (base[0] + cy * math.sin(h) - cxf * math.cos(h),
            base[1] - cy * math.cos(h) - cxf * math.sin(h))


def section_b():
    print("=" * 92)
    print("B 公式等价 (B1: 无检测兜底必须与旧行为逐位相同 / B2: 新式等于删掉了'追赶余量')")
    cases = [
        ((1.23, 4.56), None, (1.20, 4.50), (0.13, -0.05), 92.0),
        ((-2.55, 33.80), None, (-2.60, 33.81), (0.12, -0.05), 161.0),
        ((0.0, 0.0), None, (0.0, 0.0), (0.0, 0.0), 0.0),
        ((2.66, 33.39), None, (2.70, 33.20), (0.12, -0.05), -37.5),
    ]
    worst = 0.0
    for servo, est, plane, comp, hdg in cases:
        o = old_formula(servo, est, plane, comp, hdg)
        n = new_formula(servo, est, comp, hdg)
        d = math.hypot(o[0] - n[0], o[1] - n[1])
        worst = max(worst, d)
    check(worst < 1e-12, "无检测兜底: 旧式 vs 新式 最大差 %.1e m (<1e-12 ⇒ 行为未变)" % worst)

    # 有检测: 新式 = 桶估计 + 补偿; 旧式 − 新式 = 瞄准点 − 飞机 (被删项)
    cases2 = [
        ((1.23, 4.56), (1.20, 4.40), (1.15, 4.35), (0.13, -0.05), 92.0),
        ((2.66, 33.39), (2.62, 33.40), (2.55, 33.10), (0.12, -0.05), 93.0),
    ]
    w1 = w2 = 0.0
    for servo, est, plane, comp, hdg in cases2:
        o = old_formula(servo, est, plane, comp, hdg)
        n = new_formula(servo, est, comp, hdg)
        e = new_formula(est, est, comp, hdg)                 # 新式 = est + 补偿
        w1 = max(w1, math.hypot(n[0] - e[0], n[1] - e[1]))
        # 旧式 − 新式 应等于 (瞄准点 − 飞机)
        gap_true = (servo[0] - plane[0], servo[1] - plane[1])
        gap_got = (o[0] - n[0], o[1] - n[1])
        w2 = max(w2, math.hypot(gap_true[0] - gap_got[0], gap_true[1] - gap_got[1]))
    check(w1 < 1e-12, "有检测: 新式 ≡ 桶估计 + 补偿 (最大差 %.1e m)" % w1)
    check(w2 < 1e-12, "旧式 − 新式 ≡ 瞄准点 − 飞机 (最大差 %.1e m)" % w2)


# ---------------------------------------------------------------- C 日志回放
def section_c():
    print("=" * 92)
    print("C 真实日志回放 (两轮 SITL, 各 2 投)")
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    try:
        import analyze_i_hold_geom_20261007 as an
    except Exception as e:                                    # pragma: no cover
        check(False, "无法导入分析模块: %s" % e)
        return
    for label, log, judge, expect_zero in [
        ("改后 A (纯几何)", A_LOG, A_LOG.replace(".log", "_judge.log"), True),
        ("改前 (旧公式)", PRE_A_LOG, PRE_A_LOG.replace(".log", "_judge.log"), False),
    ]:
        if not (os.path.exists(log) and os.path.exists(judge)):
            print("  (跳过 %s: 日志不存在)" % label)
            continue
        freezes, drops = an.parse_log(log)
        judges = an.parse_judge(judge)
        print(" -- %s  (%d 冻结 / %d 投)" % (label, len(freezes), len(drops)))
        for i, (fz, dr) in enumerate(zip(freezes, drops)):
            bk = judges[i]["bucket"] if i < len(judges) else None
            est, hold = fz["est"], fz["hold"]
            h = math.radians(fz["hdg"])
            cy, cxf = fz["comp_cmd"]
            comp_ned = (cy * math.sin(h) - cxf * math.cos(h), -cy * math.cos(h) - cxf * math.sin(h))
            extra = (hold[0] - est[0] - comp_ned[0], hold[1] - est[1] - comp_ned[1])   # 被删掉的那一项
            aim = fz["aim"]
            gap_aim = (aim[0] - fz["ac"][0], aim[1] - fz["ac"][1]) if aim and fz["ac"] else None
            if expect_zero:
                check(math.hypot(*extra) < 1e-3,
                      "第%d投: hold − est − 补偿 = %.4fm (<1mm ⇒ 保持点确实是纯几何量)"
                      % (i + 1, math.hypot(*extra)))
            else:
                ok = math.hypot(*extra) > 0.05 and gap_aim is not None \
                    and abs(math.hypot(*extra) - math.hypot(*gap_aim)) < 0.06
                check(ok, "第%d投: hold − est − 补偿 = %.3fm ≈ 冻结滞后 |瞄准点−飞机| = %.3fm (两种算法互相印证)"
                      % (i + 1, math.hypot(*extra), math.hypot(*gap_aim) if gap_aim else float("nan")))
            if bk:
                # 独立交叉核对: 裁判节点自己算的"距离" 必须等于 任务节点日志里的 释放点 − 桶
                # (两条链路、两个进程, 不是同一份数据自证 —— 用来证明我读日志的方式没错)
                d_mine = math.hypot(dr["ned"][0] - bk[0], dr["ned"][1] - bk[1])
                check(abs(d_mine - judges[i]["dist"]) < 5e-3,
                      "第%d投: 裁判距离 %.4fm == 我按日志算的 |释放点−桶| %.4fm (差 %.1e m)"
                      % (i + 1, judges[i]["dist"], d_mine, abs(d_mine - judges[i]["dist"])))


def section_d():
    """下游工具兼容性: 新增日志字段不能把标定器/分析器弄瞎 —— 这次真的踩过.
       2026-10-07: 冻结行插入 `[纯几何: ...]` 后, `calibrate_i_offset_20261007.py` 的冻结行正则
       直接匹配不上 ⇒ 表格一行都不打 (静默失效). 现在两种格式都必须解析出 2 投."""
    print("=" * 92)
    print("D 下游工具兼容性 (标定器: 新格式 + 旧格式都要能解析)")
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    try:
        import importlib
        cal = importlib.import_module("calibrate_i_offset_20261007")
    except Exception as e:                                    # pragma: no cover
        check(False, "无法导入标定器: %s" % e)
        return
    for label, log in [("新格式 (方案 A 之后)", A_LOG),
                       ("旧格式 (10-6 实飞, 回归)", os.path.join(ROOT, "10.6/i_mission_20261006_170652.log"))]:
        if not os.path.exists(log):
            print("  (跳过 %s: 日志不存在)" % label)
            continue
        try:
            home, hdg, comp, freezes, drops = cal.parse_log(log)
        except SystemExit as e:                               # pragma: no cover
            check(False, "%s: 标定器解析失败 %s" % (label, e))
            continue
        check(len(freezes) >= 2 and len(drops) >= 2,
              "%s: 解析出 %d 冻结 / %d 投 (都须 ≥2)" % (label, len(freezes), len(drops)))
        if drops:
            check(all(d["hold"] and d["err"] is not None for d in drops),
                  "%s: 每投都拿到 hold 与 到位err (列不会空)" % label)


def main():
    print("方案 A 离线验收 (hold = 桶估计 + 补偿) —— 2026-10-07")
    section_a()
    section_b()
    section_c()
    section_d()
    print("=" * 92)
    print("结果: 通过 %d 项, 失败 %d 项" % (len(OK), len(BAD)))
    for b in BAD:
        print("  ✗ %s" % b)
    return 0 if not BAD else 1


if __name__ == "__main__":
    sys.exit(main())
