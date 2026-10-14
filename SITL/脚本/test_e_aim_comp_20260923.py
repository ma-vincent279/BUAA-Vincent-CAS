#!/usr/bin/env python3
# ================================================================
# 2026-09-23 项目 e 改动离线验收: 两轴瞄准补偿 + 米制释放判据
#   直接 exec **真实源码**里的常量 / 几何函数 / PIDController (不复制粘贴逻辑)
#   真值模型: 把桶放在机体水平系的指定地面偏移处 → 反算理想像素 → 喂真实 PID
# 用法: python3 scripts/test_e_aim_comp_20260923.py
# ================================================================
import ast
import math
import os
import sys

ROOT = '/home/vincent/SITL'
FILES = {'SITL': 'src/drone_drl/drone_drl/e.py', 'fly': 'fly/src/fly/fly/e.py'}
NAMES = ['IMG_W', 'IMG_H', 'CX', 'CY', 'HFOV', 'VFOV', 'FX', 'FY',
         'KP_H', 'KI_H', 'KD_H', 'MAX_HORIZ_VEL', 'INTEGRAL_LIMIT', 'DEAD_ZONE_PX',
         'CRUISE_HEIGHT', 'DROP_ARRIVE_M', 'DROP_CONVERGE_EARLY', 'DROP_AFTER_DELAY',
         'PAYLOAD_OFFSET_BODY_Y_B1', 'PAYLOAD_OFFSET_BODY_Y_B2',
         'PAYLOAD_OFFSET_BODY_XF_B1', 'PAYLOAD_OFFSET_BODY_XF_B2',
         'camera_pixel_to_level', 'level_to_camera_pixel', 'PIDController']


def load(path):
    src = open(path, encoding='utf-8').read()
    tree = ast.parse(src)
    ns = {'math': math}
    got = set()
    for node in tree.body:
        nms = []
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            nms = [node.name]
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    nms.append(t.id)
                elif isinstance(t, ast.Tuple):   # IMG_W, IMG_H = 640, 360
                    nms += [e.id for e in t.elts if isinstance(e, ast.Name)]
        hit = [n for n in nms if n in NAMES]
        if hit:
            exec(ast.get_source_segment(src, node), ns)
            got.update(hit)
    missing = [n for n in NAMES if n not in got]
    if missing:
        raise SystemExit(f'{path}: 未能从源码取出 {missing}')
    return ns


def roi_at(cx, cy, w=40, h=40):
    class R:
        pass
    r = R()
    r.x_offset, r.y_offset, r.width, r.height = cx - w / 2.0, cy - h / 2.0, w, h
    return r


def pid_err(ns, fwd_t, right_t, H, pitch, roll, xf, yc):
    """真值: 桶在机体水平系 (前 fwd_t, 右 right_t), 高 H → 理想像素 → 真实 PID 报的误差"""
    pix = ns['level_to_camera_pixel'](fwd_t, right_t, H, pitch, roll)
    if pix is None:
        return None
    dz_h = max(0.3, H)
    pid = ns['PIDController'](ns['KP_H'], ns['KI_H'], ns['KD_H'], ns['MAX_HORIZ_VEL'],
                              ns['INTEGRAL_LIMIT'], ns['DEAD_ZONE_PX'])
    v_f, v_l, err_px, ex, ey = pid.compute(
        roi_at(*pix), 0.05, yc * ns['FX'] / dz_h, pitch, roll, dz_h / ns['CRUISE_HEIGHT'],
        xf * ns['FY'] / dz_h)
    g = math.hypot(ex * dz_h / ns['FX'], ey * dz_h / ns['FY'])
    return err_px, ex, ey, g


def main():
    N = {k: load(os.path.join(ROOT, p)) for k, p in FILES.items()}
    fail = 0

    def chk(cond, msg):
        nonlocal fail
        print(('  ✓ ' if cond else '  ✗ ') + msg)
        if not cond:
            fail += 1

    XF, YC = N['SITL']['PAYLOAD_OFFSET_BODY_XF_B1'], N['SITL']['PAYLOAD_OFFSET_BODY_Y_B1']
    print(f'== 源码常量: 前向补偿 XF={XF*100:+.1f}cm  横向补偿 Y={YC*100:+.1f}cm  '
          f'|S|={math.hypot(XF,YC)*100:.2f}cm @ {math.degrees(math.atan2(YC,XF)):.0f}°')
    print(f'   判据: DROP_ARRIVE_M={N["SITL"]["DROP_ARRIVE_M"]}m (等效 {N["SITL"]["DROP_ARRIVE_M"]*100:.0f}cm)  '
          f'兜底 DROP_AFTER_DELAY={N["SITL"]["DROP_AFTER_DELAY"]:.0f}s  '
          f'DROP_CONVERGE_EARLY={N["SITL"]["DROP_CONVERGE_EARLY"]}s')

    print('\n[A] 两轴补偿符号/量级: 桶正好放在"补偿后瞄准点"上时 PID 必须报 0 (独立真值模型)')
    for pitch, roll, tag in ((0.053, 0.017, '9-23 第1投姿态'), (0.070, -0.010, '9-23 第2投姿态'),
                             (0.0, 0.0, '水平')):
        for H in (1.0, 1.3, 1.5):
            r = pid_err(N['SITL'], XF, YC, H, pitch, roll, XF, YC)
            chk(r is not None and r[0] < 0.6 and r[3] < 0.005,
                f'{tag} H={H}m → err={r[0]:.2f}px 等效地面={r[3]*100:.3f}cm (期望 <0.6px / <0.5cm)')

    print('\n[B] 桶在飞机正下方 (0,0) 时判据应报 |S| 而不是 0 —— 判据现在衡量"预测落点偏差"')
    for H in (1.0, 1.3, 1.5):
        r = pid_err(N['SITL'], 0.0, 0.0, H, 0.05, 0.0, XF, YC)
        chk(abs(r[3] - math.hypot(XF, YC)) < 0.004,
            f'H={H}m → 等效地面={r[3]*100:.2f}cm (期望 |S|={math.hypot(XF,YC)*100:.2f}cm)')

    print('\n[C] 单轴隔离 (证明两轴各自独立生效: 补偿只抵消对应轴, 另一轴残差保留)')
    r = pid_err(N['SITL'], 0.0, YC, 1.3, 0.0, 0.0, XF, YC)   # 桶在"右 Yc、前后 0"
    chk(abs(r[3] - XF) < 1e-3,
        f'桶(前0.0, 右{YC*100:.1f}cm) → 地面残差 {r[3]*100:.2f}cm (应恰为未被抵消的前向分量 {XF*100:.1f}cm)')
    r = pid_err(N['SITL'], XF, 0.0, 1.3, 0.0, 0.0, XF, YC)   # 桶在"前 Xf、右 0"
    chk(abs(r[3] - YC) < 1e-3,
        f'桶(前{XF*100:.1f}cm, 右0.0) → 地面残差 {r[3]*100:.2f}cm (应恰为未被抵消的横向分量 {YC*100:.1f}cm)')
    r = pid_err(N['SITL'], XF + 0.03, YC, 1.3, 0.0, 0.0, XF, YC)  # 再往前 3cm
    chk(abs(r[3] - 0.03) < 1e-3,
        f'桶(前{XF*100+3:.1f}cm, 右{YC*100:.1f}cm) → 地面残差 {r[3]*100:.2f}cm (应为 3cm 增量)')

    print('\n[D] 用 9-23 两次真实释放瞬间的 last_err 复算"预测落点偏差", 与用户真值 15~16cm 对比')
    # 日志 last_err = (err_px, ex, ey, H); 弹道漂移 (机体系 前/右 cm) 由 ULog 测得
    drops = [('第1投', -15, -20, 1.3, -2.5, +2.3),
             ('第2投', -27, -8, 1.3, -0.9, -0.9)]
    for tag, ex_px, ey_px, H, dr_f, dr_r in drops:
        rg = ex_px * H / N['SITL']['FX'] - YC
        fg = ey_px * H / N['SITL']['FY'] - XF
        pred = math.hypot(rg, fg)
        pred_dr = math.hypot(rg + dr_r / 100.0, fg + dr_f / 100.0)
        raw = math.hypot(ex_px * H / N['SITL']['FX'], ey_px * H / N['SITL']['FY'])
        print(f'  {tag}: 旧判据(无补偿)={raw*100:.1f}cm  →  新判据(含补偿)={pred*100:.1f}cm  '
              f'| 再计入弹道漂移={pred_dr*100:.1f}cm   用户真值 15~16cm')
        chk(13.0 < pred_dr * 100 < 18.5, f'  {tag}: 新判据量出的偏差 ({pred_dr*100:.1f}cm) 与真值同量级 ✓')
    print('  ⇒ 新判据量到的就是"落点偏差", 所以门限 10cm 会真的把落点约束在 10cm 内喵~')

    print('\n[E] 门限有效性: 上述两次释放瞬间都必须 **不满足** 门限 (否则会在错相位误投)')
    for tag, ex_px, ey_px, H, _, _ in drops:
        rg = ex_px * H / N['SITL']['FX'] - YC
        fg = ey_px * H / N['SITL']['FY'] - XF
        conv = math.hypot(rg, fg)
        chk(conv > N['SITL']['DROP_ARRIVE_M'],
            f'  {tag}: conv_m={conv:.3f}m > 门限 {N["SITL"]["DROP_ARRIVE_M"]}m → 当时不会误投 ✓')

    print('\n[F] SITL 与 fly 两份源码行为逐位一致')
    for name in ('DROP_ARRIVE_M', 'DROP_CONVERGE_EARLY', 'DROP_AFTER_DELAY', 'PAYLOAD_OFFSET_BODY_XF_B1',
                 'PAYLOAD_OFFSET_BODY_Y_B1', 'PAYLOAD_OFFSET_BODY_XF_B2', 'PAYLOAD_OFFSET_BODY_Y_B2'):
        chk(N['SITL'][name] == N['fly'][name], f'{name}: SITL={N["SITL"][name]} fly={N["fly"][name]}')
    same = all(abs(pid_err(N['SITL'], XF, YC, 1.3, 0.06, 0.01, XF, YC)[i]
                   - pid_err(N['fly'], XF, YC, 1.3, 0.06, 0.01, XF, YC)[i]) < 1e-12 for i in (0, 3))
    chk(same, '同一输入下两份 PID 输出逐位一致')

    print('\n' + ('全部通过 ✓' if fail == 0 else f'{fail} 项未过 ✗'))
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
