#!/usr/bin/env python3
# ================================================================
# 2026-09-23 项目 e: 按 9-23 实飞标定修 (1) 两轴瞄准补偿 + (2) 释放判据改米制
#                    —— SITL + fly 两份一起改 (按项目规矩: 先备份 -> 改 -> 仿真验证)
# ----------------------------------------------------------------
# 数据依据 (e_mission_20260923_175741.log + log_424_2026-9-23-18-00-28.ulg,
#           分析脚本 scripts/analyze_e_flight_20260923.py / scripts/replay_e_drop_gate_20260923.py):
#   两次投弹都走 "25.0s/25s 兜底" (不是收敛即投); 释放 err 25px/28px 等效 9.2/10.3cm.
#   用户现场真值: 两次落点都在桶"右上方"、离桶心 15~16cm.
#   矢量分解 (机体系 前/右, cm):
#     第1投: 释放瞬间 桶-飞机 = (前-7.3, 右-5.5) + 弹道漂移(-2.5,+2.3) -> 预测 9.2cm
#     第2投:                (前-2.9, 右-10.1) + 弹道漂移(-0.9,-0.9) -> 预测 9.6cm
#     两投联立 (要求同一固定机体偏置 S):  S 约 (前 +4.0, 右 +4.8) cm = 6.2cm @ 50 度 (右上前方)
#   => 误差预算 约 60% 释放相位 (随机) + 40% 固定机体偏置 (可标定)
#
# (1) 瞄准补偿扩成两轴并填本次实测值 (原只有横向一轴且恒为 0):
#      _Y_  = +0.048  (正值 = 瞄准点左移 4.8cm; 语义与既有横向补偿完全一致)
#      _XF_ = +0.040  (新增; 正值 = 瞄准点后移 4.0cm)
#    作用: (a) VISUAL_SERVO / DROP 两处 PID 像素偏置 -> 伺服平衡点=补偿后的瞄准点
#          (b) DROP 段目标点平移 (与既有横向平移同一处, 方向 = 后向单位向量)
# (2) 释放判据 10px (等效 3~4cm, 实机误差 9~18cm => 永远达不到 => 必然 25s 兜底随机相位释放)
#    -> **米制 10cm** (与 d 在 9-19 实飞验证过的一致: 那两次收敛即投在 3.5/3.3cm 触发),
#    确认时间 1.5s -> 1.0s.
#    门限回放证据 (scripts/replay_e_drop_gate_20260923.py, 44 条 1Hz DROP-SERVO 样本):
#      raw 10cm/1.0s -> 第1投 8.8s 触发(e=8.9cm) 第2投 7.7s 触发(e=3.1cm)
#      补偿后(悲观代理) 10cm/1.0s -> 第1投 12.5s(e=8.5cm) 第2投 9.0s(e=7.7cm)
#      (1.5s 在"补偿后"下第2投只到 1.2s 连续 => 不触发, 故降到 1.0s)
#    捕获逻辑: conv_m = |(ex*H/FX, ey*H/FY)| 即"飞机到补偿后瞄准点的地面距离",
#              也就是**预测落点与桶心的距离**的代理量 => 门限直接约束落点偏差.
#
# 未改 (按用户决定): 投放通道 / 释放时刻通道 / 释放脉冲形态 一律不动.
# 用法: python3 scripts/fix_e_aim_comp_20260923.py [--check]
# ================================================================
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = '/home/vincent/SITL'
FILES = ['src/drone_drl/drone_drl/e.py', 'fly/src/fly/fly/e.py']
STAMP = time.strftime('%Y%m%d_%H%M%S')
CHECK = '--check' in sys.argv

C1_OLD = 'PAYLOAD_OFFSET_BODY_Y_B1 =  0.0\nPAYLOAD_OFFSET_BODY_Y_B2 =  0.0'
C1_NEW = '''# 2026-09-23 实飞标定 (e 首次实飞, 两投落点都在桶"右上方" 15~16cm):
# 矢量分解得固定机体偏置 S = (前 +4.0cm, 右 +4.8cm) => 瞄准点反向平移 (前 -4.0, 右 -4.8).
# 语义与 pixel_offset 一致: _Y>0 = 瞄准点左移;  _XF>0 = 瞄准点后移.
PAYLOAD_OFFSET_BODY_Y_B1 = +0.048   # 横向 (落点偏右 4.8cm -> 瞄准点左移)
PAYLOAD_OFFSET_BODY_Y_B2 = +0.048
PAYLOAD_OFFSET_BODY_XF_B1 = +0.040  # 前向 (2026-09-23 新增): 瞄准点后移 4.0cm
PAYLOAD_OFFSET_BODY_XF_B2 = +0.040'''

C2_OLD = 'DROP_ARRIVE_PX   = 10.0  # 2026-09-12 e: DROP 收敛即投阈值 (比 VISUAL_SERVO 的 8px 宽松 — 8px 边缘 8<8=False 卡阈值)'
C2_NEW = '''DROP_ARRIVE_M    = 0.10   # 2026-09-23 实飞标定: 判据改**米制 10cm** (原 10px 等效 3~4cm@1.2m 而实机误差
                          #   9~18cm => 永远达不到 => 两次都走 25s 兜底、随机相位释放 => 双脱靶).
                          #   与 d 在 9-19 实飞验证过的一致 (那两次收敛即投在 3.5/3.3cm 触发).
DROP_ARRIVE_PX   = 10.0   # 仅保留作历史参数留档, 不再是判据 (判据见 DROP_ARRIVE_M)'''

C3_OLD = 'DROP_CONVERGE_EARLY = 1.5  # 2026-09-12 e: err<DROP_ARRIVE_PX 持续 1.5s → 提前投弹 (防未收敛就投)'
C3_NEW = ('DROP_CONVERGE_EARLY = 1.0  # 2026-09-23: 1.5->1.0s (1Hz 回放显示补偿后 1.5s 未必凑得齐; '
          '实测摄像头误差与真实偏差逐点 corr 0.90/0.95 => 低谷=真对齐)')

P1_OLD = '    def compute(self, roi, dt, pixel_offset_x=0.0, pitch=0.0, roll=0.0, height_scale=1.0):'
P1_NEW = ('    def compute(self, roi, dt, pixel_offset_x=0.0, pitch=0.0, roll=0.0, height_scale=1.0,\n'
          '                pixel_offset_y=0.0):')
P2_OLD = '        ey = fwd_per_h * FY'
P2_NEW = '        ey = fwd_per_h * FY - pixel_offset_y   # 2026-09-23: 前向补偿 (正值=瞄准点后移 4cm)'

G_OLD = '''            release_valid = (self._drop_detection_fresh() and self._last_drop_err is not None
                             and self.z > drop_z - 0.2)
            early_ok = False
            if (release_valid and self._last_drop_err[0] < DROP_ARRIVE_PX):'''
G_NEW = '''            release_valid = (self._drop_detection_fresh() and self._last_drop_err is not None
                             and self.z > drop_z - 0.2)
            # 2026-09-23: 判据改米制 —— 把最近一帧 (ex, ey, H) 折算成"到补偿后瞄准点的地面距离"
            # (= 预测落点与桶心距离的代理量). ex/ey 已含 2026-09-23 两轴瞄准补偿.
            conv_m = float('inf')
            if self._last_drop_err is not None:
                _ep, _exl, _eyl, _hh = self._last_drop_err
                conv_m = math.hypot(_exl * _hh / FX, _eyl * _hh / FY)
            early_ok = False
            if (release_valid and conv_m < DROP_ARRIVE_M):'''

L1_OLD = ('f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm'
          '(门限{DROP_ARRIVE_PX * dz_h / FX * 100:.0f}cm)")')
L1_NEW = ('f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm'
          '(门限{DROP_ARRIVE_M * 100:.0f}cm)")')
L2_OLD = 'f"drop_gate={DROP_ARRIVE_PX:.0f}px(≈{DROP_ARRIVE_PX * DROP_HEIGHT / FX * 100:.0f}cm)"'
L2_NEW = 'f"drop_gate={DROP_ARRIVE_M * 100:.0f}cm(米制, 原 10px≈{10 * DROP_HEIGHT / FX * 100:.0f}cm)"'
L3_OLD = 'f"offset_B1={PAYLOAD_OFFSET_BODY_Y_B1:.2f}m offset_B2={PAYLOAD_OFFSET_BODY_Y_B2:.2f}m "'
L3_NEW = ('f"offset_横_B1={PAYLOAD_OFFSET_BODY_Y_B1:.3f}m B2={PAYLOAD_OFFSET_BODY_Y_B2:.3f}m "\n'
          '            f"offset_前_B1={PAYLOAD_OFFSET_BODY_XF_B1:.3f}m B2={PAYLOAD_OFFSET_BODY_XF_B2:.3f}m "')

# 两处 PID 调用点: SITL 与 fly 的缩进 (16/20 vs 20/24) 与 dt 表达式 (0.05 vs dt) 不同
# -> 正则匹配并**保留原缩进与 dt 表达式**, 只插入前向补偿
CALL_RE = re.compile(
    r'(?P<ind>[ ]*)offset_body_y = PAYLOAD_OFFSET_BODY_Y_B1 if self\._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2\n'
    r'(?P=ind)pixel_offset = offset_body_y \* FX / dz_h\n'
    r'(?P=ind)height_scale = dz_h / CRUISE_HEIGHT\n'
    r'(?P=ind)v_fwd, v_lat, err_px, ex, ey = self\.pid\.compute\(\n'
    r'(?P=ind)    roi, (?P<dt>[^,]+), pixel_offset, self\._pitch, self\._roll, height_scale\)')


def call_repl(m):
    i, dt = m.group('ind'), m.group('dt')
    return (f'{i}offset_body_y = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2\n'
            f'{i}offset_body_xf = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2\n'
            f'{i}pixel_offset = offset_body_y * FX / dz_h\n'
            f'{i}pixel_offset_y = offset_body_xf * FY / dz_h\n'
            f'{i}height_scale = dz_h / CRUISE_HEIGHT\n'
            f'{i}v_fwd, v_lat, err_px, ex, ey = self.pid.compute(\n'
            f'{i}    roi, {dt}, pixel_offset, self._pitch, self._roll, height_scale, pixel_offset_y)')


COMMON = [('常量: 两轴补偿', C1_OLD, C1_NEW), ('常量: 米制门限', C2_OLD, C2_NEW),
          ('常量: 确认时间', C3_OLD, C3_NEW), ('PID 签名 +pixel_offset_y', P1_OLD, P1_NEW),
          ('PID ey 应用前向偏置', P2_OLD, P2_NEW), ('释放判据改米制', G_OLD, G_NEW),
          ('DROP 日志门限', L1_OLD, L1_NEW), ('启动日志门限', L2_OLD, L2_NEW),
          ('启动日志补偿值', L3_OLD, L3_NEW)]


def target_anchor(src):
    """DROP 段目标点平移 (缩进随文件自动识别): 在后向单位向量上追加前向补偿"""
    key = 'self._servo_target_x += offset_body_y * math.sin(self.heading)'
    lines = src.split('\n')
    i = next(i for i, l in enumerate(lines) if l.strip() == key)
    ind = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
    old = (f'{ind}{key}\n'
           f'{ind}self._servo_target_y -= offset_body_y * math.cos(self.heading)')
    new = (f'{ind}{key}\n'
           f'{ind}self._servo_target_y -= offset_body_y * math.cos(self.heading)\n'
           f'{ind}# 2026-09-23: 前向补偿 (正值=瞄准点后移) — 后向单位向量 = (-cos h, -sin h)\n'
           f'{ind}self._servo_target_x -= offset_body_xf * math.cos(self.heading)\n'
           f'{ind}self._servo_target_y -= offset_body_xf * math.sin(self.heading)')
    return old, new


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()[:8]


def main():
    print('=== e: (1)两轴瞄准补偿(+4.0cm 前/+4.8cm 右) + (2)释放判据改米制 10cm (SITL + fly) ===\n')
    todo = []
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        done = (all(n in src for _, _, n in COMMON)
                and src.count('pixel_offset_y') >= 4 and 'offset_body_xf' in src)
        print(f'{rel}  md5={md5(p)}')
        if done:
            print('   -> 已全部落地, 跳过\n')
            continue
        t_old, t_new = target_anchor(src)
        exact = COMMON + [('DROP 目标点两轴平移', t_old, t_new)]
        ncall = len(CALL_RE.findall(src))
        bad = [(n, src.count(o)) for n, o, _ in exact if src.count(o) != 1]
        if bad or ncall != 2:
            print(f'   X 锚点异常: {bad}  PID 调用点命中 {ncall} 次 (期望 2) -> 中止')
            return 1
        print(f'   {len(exact)}/{len(exact)} 精确锚点唯一 + PID 调用点 {ncall} 处 OK\n')
        todo.append((p, src, exact))
    if not todo:
        print('无需改动.')
        return 0
    if CHECK:
        print('[--check] 仅检查锚点, 未写入.')
        return 0

    for p, src, exact in todo:
        bak = f'{p}.bak.{STAMP}_aim_comp'
        if not os.path.exists(bak):
            shutil.copy2(p, bak)
            print(f'备份 -> {os.path.relpath(bak, ROOT)}')
        for _, old, new in exact:
            src = src.replace(old, new, 1)
        src = CALL_RE.sub(call_repl, src, count=2)
        open(p, 'w', encoding='utf-8').write(src)
        os.utime(p, None)   # 防 colcon 因 mtime 相同跳过安装
        print(f'已改 -> {os.path.relpath(p, ROOT)}  md5={md5(p)}')

    print('\n--- 复验 ---')
    bad = 0
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        r = subprocess.run([sys.executable, '-m', 'py_compile', p], capture_output=True, text=True)
        miss = [n for n, _, new in COMMON if new not in src]
        checks = {
            'pixel_offset_y>=4': src.count('pixel_offset_y') >= 4,
            'offset_body_xf>=4': src.count('offset_body_xf') >= 4,
            'DROP_ARRIVE_M 判据': 'conv_m < DROP_ARRIVE_M' in src,
            '调用点已改 2 处': src.count('height_scale, pixel_offset_y)') == 2,
            '两轴常量值正确': ('PAYLOAD_OFFSET_BODY_XF_B1 = +0.040' in src
                          and 'PAYLOAD_OFFSET_BODY_Y_B1 = +0.048' in src),
        }
        badc = [k for k, v in checks.items() if not v]
        print(f'{rel}: py_compile={"OK" if r.returncode == 0 else "FAIL"}  '
              f'落地={len(COMMON) - len(miss)}/{len(COMMON)}  md5={md5(p)}'
              + (f'  未过: {badc + miss}' if (badc or miss) else '  OK'))
        if r.returncode or badc or miss:
            bad += 1
            if r.stderr:
                print(r.stderr)
    print('\n结果:', '全部通过 (下一步: colcon build)' if bad == 0 else f'{bad} 个文件异常')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
