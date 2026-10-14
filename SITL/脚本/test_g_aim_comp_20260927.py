#!/usr/bin/env python3
# ================================================================
# 2026-09-27 项目 g 离线验收: 两瓶横向补偿按 9-27 实飞落点重标
#   A. 常量核对 (两份 g.py 逐位一致; B1=+0.113 / B2=-0.025 / XF=+0.040)
#   B. 真实代码路径: exec 两份 g.py 的 PIDController + camera/level 几何,
#      验证"桶在补偿后瞄准点上时 PID 报 0"(含 B2 的负值反号路径), 并给出
#      若仍用 f 的旧常量时该均衡点的误差 (= 需要修正的量).
#   C. 9-27 四次真实落点回放 (首次落点, 不计弹跳):
#      e#1 偏右 5cm / e#2 偏左 12cm / f#1 偏右 8cm / f#2 偏左 2~3cm
#      => 新落点 = 实测 + (旧补偿 - 新补偿)
#   用法: python3 scripts/test_g_aim_comp_20260927.py     (需 source install/setup.bash)
# ================================================================
import math
import os
import re
import sys

ROOT = '/home/vincent/SITL'
FILES = {'SITL': 'src/drone_drl/drone_drl/g.py', 'fly': 'fly/src/fly/fly/g.py'}
F_OLD = {'B1': +4.8, 'B2': +4.8}          # f/e 时代的横向补偿 (cm, 正值=瞄准点左移)
G_NEW = {'B1': None, 'B2': None}          # 从 g.py 读出
XF = None                                 # 纵向补偿 (cm)
FX = FY = None
# 9-27 实飞: 首次落点 (cm, 正=偏右). payload 按投放次序: 第1投=右瓶, 第2投=左瓶
CASES = [
    ('e 航班 第 1 投 (右瓶)', 'B1', +5.0),
    ('e 航班 第 2 投 (左瓶)', 'B2', -12.0),
    ('f 航班 第 1 投 (右瓶)', 'B1', +8.0),
    ('f 航班 第 2 投 (左瓶)', 'B2', -2.5),
]
FAIL = []


def check(ok, msg):
    print(('  ✓ ' if ok else '  ✗ ') + msg)
    if not ok:
        FAIL.append(msg)


def build_ns(rel):
    """按源码顺序 exec 顶层赋值 + 需要的函数/类 => 拿到与出厂完全一致的常量与几何"""
    import ast
    s = open(os.path.join(ROOT, rel), encoding='utf-8').read()
    tree = ast.parse(s)
    want = {'camera_pixel_to_level', 'level_to_camera_pixel', 'PIDController'}
    ns = {'math': math}
    for node in tree.body:
        if isinstance(node, ast.Assign):                      # 含元组赋值 (IMG_W, IMG_H = 640, 360)
            try:
                exec(compile(ast.Module([node], []), rel, 'exec'), ns)
            except Exception:
                pass                                          # 依赖 ROS 的常量 (QOS_BE 等) 跳过
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in want:
            exec(compile(ast.Module([node], []), rel, 'exec'), ns)
    return ns


def load_consts(rel):
    ns = build_ns(rel)
    s = open(os.path.join(ROOT, rel), encoding='utf-8').read()
    return s, dict(Y1=ns['PAYLOAD_OFFSET_BODY_Y_B1'], Y2=ns['PAYLOAD_OFFSET_BODY_Y_B2'],
                   XF1=ns['PAYLOAD_OFFSET_BODY_XF_B1'], XF2=ns['PAYLOAD_OFFSET_BODY_XF_B2'],
                   FX=ns['FX'], FY=ns['FY'])


class ROI:  # 极简 ROI 替身
    def __init__(self, cx, cy, w=70.0, h=70.0):
        self.x_offset = cx - w / 2.0
        self.y_offset = cy - h / 2.0
        self.width = w
        self.height = h


def pid_err(ns, bucket_y_cm, bucket_xf_cm, comp_y_cm, comp_xf_cm, H=1.20, pitch=0.03, roll=0.02):
    """把桶放在机体水平系 (右 bucket_y_cm, 前 bucket_xf_cm) 处, 用补偿常量 comp 跑真实 PID.
    返回 (err_px, 横向误差cm, 纵向误差cm); 误差 = 桶位 − 补偿 ⇒ 0 表示"飞机正停在补偿后瞄准点上".
    """
    right_m, fwd_m = bucket_y_cm / 100.0, bucket_xf_cm / 100.0
    px = ns['level_to_camera_pixel'](fwd_m, right_m, H, pitch, roll)
    if px is None:
        return None
    pid = ns['PIDController'](ns['KP_H'], ns['KI_H'], ns['KD_H'], ns['MAX_HORIZ_VEL'],
                              ns['INTEGRAL_LIMIT'], ns['DEAD_ZONE_PX'])
    pox = comp_y_cm / 100.0 * ns['FX'] / H
    poy = comp_xf_cm / 100.0 * ns['FY'] / H
    vf, vl, err, ex, ey = pid.compute(ROI(*px), 0.25, pox, pitch, roll, H / ns['CRUISE_HEIGHT'], poy)
    return err, ex * H / ns['FX'] * 100.0, ey * H / ns['FY'] * 100.0   # err_px, 横向cm, 纵向cm


def main():
    print('=' * 96)
    print('项目 g 离线验收 — 两瓶横向补偿重标 (9-27 实飞落点标定)')
    print('=' * 96)

    print('\n[A] 常量核对')
    ref = None
    for name, rel in FILES.items():
        s, c = load_consts(rel)
        print(f'   {name}: Y_B1={c["Y1"]:+.3f} Y_B2={c["Y2"]:+.3f} XF_B1={c["XF1"]:+.3f} XF_B2={c["XF2"]:+.3f}')
        check(abs(c['Y1'] - 0.113) < 1e-9, f'{name}: PAYLOAD_OFFSET_BODY_Y_B1 == +0.113')
        check(abs(c['Y2'] + 0.025) < 1e-9, f'{name}: PAYLOAD_OFFSET_BODY_Y_B2 == -0.025 (左瓶反号)')
        check(abs(c['XF1'] - 0.040) < 1e-9 and abs(c['XF2'] - 0.040) < 1e-9,
              f'{name}: 纵向 XF 保持 +0.040 (9-27 实测无前后偏差)')
        check('PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2' in s,
              f'{name}: 第 1 投用 B1 / 第 2 投用 B2 的分派逻辑仍在')
        if ref is None:
            ref = c
        else:
            check(all(abs(ref[k] - c[k]) < 1e-12 for k in ('Y1', 'Y2', 'XF1', 'XF2')),
                  f'{name}: 与 SITL 版常量逐位一致')

    print('\n[B] 真实代码路径: 均衡点 (= 让"载荷落点在桶心"的桶位) 与补偿常量一致 (含 B2 负值反号)')
    for name, rel in FILES.items():
        ns = build_ns(rel)
        for payload, comp_new in [('B1(右瓶)', +11.3), ('B2(左瓶)', -2.5)]:
            # B1: 均衡点 = 桶在飞机右 11.3cm;  B2: 均衡点 = 桶在飞机左 2.5cm (飞机在桶右侧)
            r0 = pid_err(ns, comp_new, 4.0, comp_new, 4.0)
            check(r0 is not None and r0[0] < 1.0 and abs(r0[1]) < 0.5,
                  f'{name} {payload}: 桶在补偿点 {comp_new:+.1f}cm 处 => err={r0[0]:.2f}px (横{r0[1]:+.2f}cm) 均衡 ✓')
            # 旧常量 (f) 看同一个物理桶位 -> 偏差应等于"需要的修正量"
            r1 = pid_err(ns, comp_new, 4.0, +4.8, 4.0)
            need = comp_new - 4.8
            check(abs(r1[1] - need) < 0.3,
                  f'{name} {payload}: 同一桶位用旧常量 +4.8cm 时报 {r1[1]:+.2f}cm '
                  f'(= 需要的修正量 {need:+.1f}cm) ⇒ 旧值确实偏了这么多')
        # B2 反号验证: 若桶仍在"旧符号"那一侧 (桶在飞机右 2.5cm), 误差应为 +5cm
        r2 = pid_err(ns, +2.5, 4.0, -2.5, 4.0)
        check(abs(r2[1] - 5.0) < 0.3,
              f'{name} B2(左瓶): 桶在旧符号侧(右 2.5cm) 时报 {r2[1]:+.2f}cm ⇒ 反号是必要的 (原来差了 5cm)')

    print('\n[C] 9-27 四次真实落点回放 (实测 + (旧补偿 - 新补偿))')
    print('   ' + '-' * 88)
    print(f'   {"投放":26s} {"载荷":10s} {"实测落点":>8s} {"旧补偿":>7s} {"新补偿":>7s} {"新落点":>8s}  判定')
    worst_old = worst_new = 0.0
    for tag, payload, obs in CASES:
        new_c = 11.3 if payload == 'B1' else -2.5
        new_land = obs + (F_OLD[payload] - new_c)
        worst_old = max(worst_old, abs(obs))
        worst_new = max(worst_new, abs(new_land))
        ok = abs(new_land) <= 5.0
        print(f'   {tag:26s} {payload:10s} {obs:+7.1f}cm {F_OLD[payload]:+6.1f} {new_c:+6.1f} '
              f'{new_land:+7.1f}cm  {"桶内(±5cm)" if ok else "偏大"}')
        check(ok, f'{tag}: 改后落点 {new_land:+.1f}cm 在 ±5cm 内')
    print('   ' + '-' * 88)
    check(worst_new <= 5.0 and worst_old > 10.0,
          f'最坏偏移 {worst_old:.1f}cm -> {worst_new:.1f}cm (桶口半径 12.5cm, 脱靶风险显著下降)')

    print('\n[D] 结论')
    if FAIL:
        print(f'   ✗ 失败 {len(FAIL)} 项:')
        for m in FAIL:
            print('     -', m)
        sys.exit(1)
    print('   ✓ 全部通过: 新常量 = "把 9-27 四次实测落点的均值挪到桶心", 且纵向保持不动')


if __name__ == '__main__':
    main()
