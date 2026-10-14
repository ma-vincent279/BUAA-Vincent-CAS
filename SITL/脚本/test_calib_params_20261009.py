#!/usr/bin/env python3
"""静态标定参数离线验收 (2026-10-09)

四节
----
A 值核对   : 4 个节点文件的 7 个常量都等于现场实测值
B 最小改动 : 剥注释后与改前备份**只差这 7 行**, 且越界行 = 0
C 完整性   : 相机模型函数 / 冻结公式 / P1-P2 开关 / HEADING_DEG 未被改动
D 自洽验算 : 用现场 4 点原始像素回算 ⇒ 文件里的 HFOV/Y/XF 与实测数据自洽

用法: python3 scripts/test_calib_params_20261009.py
"""
import ast
import difflib
import glob
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILES = [("SITL", ROOT / "src/drone_drl/drone_drl/k.py"),
         ("SITL", ROOT / "src/drone_drl/drone_drl/l.py"),
         ("fly ", ROOT / "fly/src/fly/fly/k.py"),
         ("fly ", ROOT / "fly/src/fly/fly/l.py")]

# 现场实测原值 (C=1.205m 相机离地, 桶前向 0, 桶口离地 0.300, 桶径 0.200)
MEAS = [(0.00, 33.5, 8.5), (0.15, 82.0, 7.5), (0.25, 124.0, 2.5), (0.35, 156.0, 11.0)]
D_DEPTH = 1.205 - 0.300            # 相机 → 桶口 真实深度
MECH_Y = 0.055                     # 铅垂线: 释放口在相机右/左各 5.5cm
MECH_XF = 0.0                      # 铅垂线: 前向机械偏移 = 0

# ⚠ 2026-10-09: 其中 3 个常量是**环境/仿真器决定的**, SITL 与实机**应当不同**
#   TARGET_PLANE_H : SITL 检测器默认 project_mode=ground 把桶当"参考平面上的圆"
#                    (`dz = -drone_z`, 不含桶高) ⇒ 与 _roi_to_ned 互为逆运算 ⇒ 0.0
#   HFOV / VFOV    : yolov8_bucket_detector 硬编码 85°/54° 并用 self.fx 生成像素
#                    ⇒ 节点必须同参, 否则凭空多出 349.22/321.12 = 1.0875 的尺度误差
#   机器核对见 scripts/check_drop_portability_20261009.py 的 A2 节 (往返误差 ≤1px)。
EXPECT_REAL = {"TARGET_PLANE_H": -0.278, "HFOV_DEG": 89.8, "VFOV_DEG": 58.5,
               "PAYLOAD_OFFSET_BODY_Y_B1": 0.146, "PAYLOAD_OFFSET_BODY_Y_B2": 0.036,
               "PAYLOAD_OFFSET_BODY_XF_B1": -0.021, "PAYLOAD_OFFSET_BODY_XF_B2": -0.021}
EXPECT_SITL = dict(EXPECT_REAL)          # 机制/几何常量两侧相同
EXPECT_SITL.update({"TARGET_PLANE_H": 0.0, "HFOV_DEG": 85.0, "VFOV_DEG": 54.0})


def expect_for(tag):
    return EXPECT_REAL if tag.strip() == "fly" else EXPECT_SITL

_ok = _fail = 0


def check(cond, msg):
    global _ok, _fail
    if cond:
        _ok += 1
        print(f"  \u2713 {msg}")
    else:
        _fail += 1
        print(f"  \u2717 {msg}")
    return bool(cond)


def consts(path):
    """取模块级数值常量 (支持多元赋值)。"""
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns, head = {}, 10
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        names = []
        for t in node.targets:
            if isinstance(t, ast.Name):
                names.append(t.id)
            elif isinstance(t, ast.Tuple):
                names.extend(e.id for e in t.elts if isinstance(e, ast.Name))
        line = src.splitlines()[node.lineno - 1] if node.lineno <= len(src.splitlines()) else ""
        if line and not line[0].isspace() and node.lineno < head:
            pass
        try:
            v = ast.literal_eval(node.value)
        except Exception:
            # math.radians(x) 之类
            if isinstance(node.value, ast.Call) and getattr(node.value.func, "attr", "") == "radians":
                try:
                    v = math.radians(ast.literal_eval(node.value.args[0]))
                except Exception:
                    continue
            else:
                continue
        if len(names) == 1:
            ns[names[0]] = v
        elif isinstance(v, tuple) and len(v) == len(names):
            ns.update(dict(zip(names, v)))
    return ns


def strip_comments(path):
    out = []
    for ln in path.read_text(encoding="utf-8").splitlines():
        t = ln.strip()
        if not t or t.startswith("#"):
            continue
        out.append(ln.rstrip())
    return out


def newest_bak(path):
    baks = sorted(glob.glob(str(path) + ".bak.*_calib20261009"))
    return Path(baks[-1]) if baks else None


def has_later_patch(path, calib_bak):
    """该文件在标定之后是否又被别的就地补丁改过 (如 _flat)。
    有的话, "相对标定备份只差 7 行" 这条断言就必然失效 —— 那是**后续补丁**的功劳,
    由那个补丁器自己的锚点断言 + 越界审计负责, 这里跳过并说明。"""
    later = [x for x in glob.glob(str(path) + ".bak.*")
             if x != str(calib_bak) and not x.endswith("_sitl_env")
             and _mtime(x) > _mtime(calib_bak)]
    return sorted(later)


def _mtime(p):
    import os
    return os.path.getmtime(p)


def section_a():
    print("\n[A] 值核对 —— 各侧等于各自的真值 (实机=现场实测; SITL=仿真器自洽值)")
    for tag, f in FILES:
        c = consts(f)
        bad = []
        for k, v in expect_for(tag).items():
            if k == "HFOV_DEG":
                got = math.degrees(c.get("HFOV", float("nan")))
            elif k == "VFOV_DEG":
                got = math.degrees(c.get("VFOV", float("nan")))
            else:
                got = c.get(k, float("nan"))
            if abs(got - v) > 5e-4:
                bad.append(f"{k} = {got:.4f} (期望 {v})")
        check(not bad, f"[{tag}] {f.name}: 7 个常量全部一致 ({'实机实测' if tag.strip() == 'fly' else 'SITL 自洽'}) "
                       f"(TARGET_PLANE_H={c.get('TARGET_PLANE_H')}, "
                       f"HFOV={math.degrees(c.get('HFOV', 0)):.2f}°, "
                       f"Y=({c.get('PAYLOAD_OFFSET_BODY_Y_B1')},{c.get('PAYLOAD_OFFSET_BODY_Y_B2')}), "
                       f"XF=({c.get('PAYLOAD_OFFSET_BODY_XF_B1')},{c.get('PAYLOAD_OFFSET_BODY_XF_B2')}), "
                       f"FX={640/(2*math.tan(c.get('HFOV',1)/2)):.1f})")
        for b in bad:
            print(f"      ✗ {b}")


def section_b():
    print("\n[B] 最小改动审计 —— 剥注释后只差这 7 行")
    names = ["TARGET_PLANE_H", "HFOV", "VFOV", "PAYLOAD_OFFSET_BODY_Y_B1",
             "PAYLOAD_OFFSET_BODY_Y_B2", "PAYLOAD_OFFSET_BODY_XF_B1",
             "PAYLOAD_OFFSET_BODY_XF_B2"]
    for tag, f in FILES:
        bak = newest_bak(f)
        if bak is None:
            check(False, f"[{tag}] {f.name}: 找不到改前备份, 无法审计")
            continue
        later = has_later_patch(f, bak)
        if later:
            print(f"  · [{tag}] {f.name}: 标定之后又被就地补丁改过 "
                  f"({', '.join(Path(x).name.split('.bak.')[-1] for x in later)}) "
                  f"⇒ 跳过'只差 7 行'审计 (由该补丁器的锚点断言负责)")
            continue
        a, b = strip_comments(bak), strip_comments(f)
        sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
        changed, illegal = [], []
        for t, i1, i2, j1, j2 in sm.get_opcodes():
            if t == "equal":
                continue
            for ln in b[j1:j2]:
                changed.append(ln)
                if not any(ln.startswith(nm) for nm in names):
                    illegal.append(ln)
            for ln in a[i1:i2]:
                if not any(ln.startswith(nm) for nm in names):
                    illegal.append(ln)
        check(len(changed) == 7 and not illegal,
              f"[{tag}] {f.name}: 改动 {len(changed)} 行, 越界 {len(illegal)} 行")
        for ln in illegal[:4]:
            print(f"      ✗ 越界: {ln[:110]}")


def section_c():
    print("\n[C] 完整性 —— 几何链/开关/航向未被改动")
    keys = ["camera_pixel_to_level", "level_to_camera_pixel"]
    for tag, f in FILES:
        src = f.read_text(encoding="utf-8")
        c = consts(f)
        ok = (all(k in src for k in keys)
              and "L_BODY_GATE_M" in c and "L_BODY_REFREEZE_MAX" in c
              and "I_FREEZE_TRACKER_M" in c and "I_CALM_TOLERANCE" not in c
              and c.get("L_BODY_GATE_M") == 0.0 and c.get("L_BODY_REFREEZE_MAX") == 3
              and c.get("I_FREEZE_TRACKER_M") == 1.0)
        check(ok, f"[{tag}] {f.name}: 相机模型函数 + P1/P2 开关 (L_BODY_GATE_M=0, "
                 f"REFREEZE_MAX=3, I_FREEZE_TRACKER_M=1.0) 均正常")
    # 航向: 仓库 4 份不应被改 (板载 212 由部署脚本处理)
    for tag, f in FILES:
        h = consts(f).get("HEADING_DEG")
        check(h in (92.0, 180.0), f"[{tag}] {f.name}: HEADING_DEG = {h} (仓库基线, 未被本次改动)")


def section_d():
    print("\n[D] 自洽验算 —— 用现场 4 点原始像素回算")
    n = len(MEAS)
    mx = sum(p[0] for p in MEAS) / n
    my = sum(p[1] for p in MEAS) / n
    a = sum((p[0] - mx) * (p[1] - my) for p in MEAS) / sum((p[0] - mx) ** 2 for p in MEAS)
    b = my - a * mx
    fx_meas = a * D_DEPTH
    c_right = b / fx_meas * D_DEPTH
    c_fwd = (sum(p[2] for p in MEAS) / n) / fx_meas * D_DEPTH
    print(f"    拟合 Δcx = {a:.2f}·ρ + {b:.2f} (残差 "
          f"{[round(p[1]-(a*p[0]+b),2) for p in MEAS]} px)")
    print(f"    ⇒ 实测 FX = {fx_meas:.1f} px   常数: 右 {c_right*100:.2f}cm / 前 {c_fwd*100:.2f}cm")

    # 账本是**实机**实测 ⇒ 只对 fly 侧核对 (SITL 侧的平面/内参由仿真器决定, 见 A 节注释)
    c = consts(FILES[2][1])
    fx_file = 640 / (2 * math.tan(c["HFOV"] / 2))
    check(abs(fx_file - fx_meas) < 1.0,
          f"文件 FX={fx_file:.1f} vs 实测 FX={fx_meas:.1f} (差 {abs(fx_file-fx_meas):.2f}px)")
    check(abs(c["PAYLOAD_OFFSET_BODY_Y_B1"] - (c_right + MECH_Y)) < 2e-3,
          f"Y_B1={c['PAYLOAD_OFFSET_BODY_Y_B1']} = 常数 {c_right:.4f} + 机械 {MECH_Y} "
          f"= {c_right+MECH_Y:.4f}")
    check(abs(c["PAYLOAD_OFFSET_BODY_Y_B2"] - (c_right - MECH_Y)) < 2e-3,
          f"Y_B2={c['PAYLOAD_OFFSET_BODY_Y_B2']} = 常数 {c_right:.4f} − 机械 {MECH_Y} "
          f"= {c_right-MECH_Y:.4f}")
    check(abs(c["PAYLOAD_OFFSET_BODY_XF_B1"] - (-c_fwd + MECH_XF)) < 2e-3,
          f"XF_B1={c['PAYLOAD_OFFSET_BODY_XF_B1']} = −前向常数 {c_fwd:.4f} + 机械 {MECH_XF} "
          f"= {-c_fwd+MECH_XF:.4f}")
    check(abs(c["TARGET_PLANE_H"] - (0.022 - 0.300)) < 1e-6,
          f"TARGET_PLANE_H={c['TARGET_PLANE_H']} = A(0.022) − R(0.300) = {0.022-0.300}")

    # 用新参数算工作点残差 (ρ=0.15/0.25/0.35 相对【相机】测, 转成相对飞机需 +c_right)
    print("\n    工作点残差 (est − ρ_true, 取 ρ_true = ρ_cam + 常数):")
    worst = 0.0
    for rho_cam, dcx, _ in MEAS:
        est = dcx / fx_file * D_DEPTH                 # 节点反投影 (H_eff = 真实深度)
        rho_true = rho_cam + c_right                   # 相对飞机原点
        # 补偿把 (est − rho_true) 的常数消掉, 残留的是斜率误差
        resid = est - rho_true - (c_right * 0)         # 常数项由 Y 吸收
        worst = max(worst, abs(resid))
    check(worst < 0.035,
          f"去掉常数项后的最大残差 {worst*100:.1f}cm < 3.5cm (每投释放散布 ±3cm 同量级)")


if __name__ == "__main__":
    print("=" * 88)
    print("静态标定参数离线验收 (2026-10-09)")
    print("=" * 88)
    section_a()
    section_b()
    section_c()
    section_d()
    print("\n" + "=" * 88)
    print(f"结果: {_ok} 项通过 / {_fail} 项失败")
    print("=" * 88)
    sys.exit(1 if _fail else 0)
