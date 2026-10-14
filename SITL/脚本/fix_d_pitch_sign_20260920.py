#!/usr/bin/env python3
# ================================================================
# 2026-09-20 项目 d: 修 PID 俯仰补偿符号 (1 行, SITL + fly 两份一起改)
# ----------------------------------------------------------------
# 根因 (离线真值模型 + 闭环复刻双重确认, 见 scripts/test_d_pitch_sign.py):
#   d 的 PID 里横向那行 `ex -= FX*tan(roll)` 是**对的**;
#   纵向那行 `ey -= FY*tan(pitch)` 符号反了, 引入系统误差 2·H·tan(pitch):
#     3.6° → 15.1cm (桶半径 12.5cm)  → 9-19 实飞第 1 投脱靶 0.235m
#     1.5° →  6.3cm                  → 9-19 实飞第 2 投命中
#   DROP 段"像素残差积分器"会把这个假的像素偏差积掉 → 瞄准点被稳定推到
#   离桶 2·H·tan(pitch) - 死区折算 的位置 (闭环实测 0.1346m @3.6°).
#
# ⚠️ 注意: 不要动 roll 那行. 穷举 4 种符号组合证明: 把 roll 也改成 += 会在
#    "飞机正上方"时报出 0.0838m 的**新横向误差** (原先 0.0000m).
#
# 用法: python3 scripts/fix_d_pitch_sign_20260920.py [--check]
# ================================================================
import hashlib
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = '/home/vincent/SITL'
FILES = ['src/drone_drl/drone_drl/d.py', 'fly/src/fly/fly/d.py']
OLD = '        ey -= FY * math.tan(pitch)'
NEW = '        ey += FY * math.tan(pitch)   # 2026-09-20: 符号修正 (原 -= 反了 → 系统误差 2·H·tan(pitch))'
STAMP = time.strftime('%Y%m%d_%H%M%S')
CHECK = '--check' in sys.argv


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()[:8]


def main():
    print('=== 修 PID 俯仰补偿符号 (1 行) ===\n')
    plans = []
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        n_old = src.count(OLD)
        n_new = src.count(NEW)
        print(f'{rel}\n   md5={md5(p)}  "{OLD.strip()}" ×{n_old}  已修写法 ×{n_new}')
        if n_new and not n_old:
            print('   → 已修, 跳过')
            continue
        if n_old != 1:
            print(f'   ✘ 期望恰好 1 处, 实际 {n_old} 处 → 中止'); return 1
        plans.append((p, src))
    if not plans:
        print('\n无需改动.'); return 0
    if CHECK:
        print('\n[--check] 仅检查, 未写入.'); return 0

    for p, src in plans:
        bak = f'{p}.bak.{STAMP}_pitch_sign'
        shutil.copy2(p, bak)
        print(f'\n备份 → {os.path.relpath(bak, ROOT)}')
        open(p, 'w', encoding='utf-8').write(src.replace(OLD, NEW))
        os.utime(p, None)          # 关键: 刷新 mtime, 否则 colcon 可能跳过重装
        print(f'已改 → {os.path.relpath(p, ROOT)}  md5={md5(p)}')

    print('\n--- 复验 ---')
    bad = 0
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        r = subprocess.run([sys.executable, '-m', 'py_compile', p], capture_output=True, text=True)
        roll_ok = '        ex -= FX * math.tan(roll)' in src
        pitch_ok = NEW in src and OLD not in src
        print(f'{rel}: py_compile={"OK" if r.returncode == 0 else "FAIL"}  '
              f'roll行保持"-="={"OK" if roll_ok else "FAIL"}  pitch行="+="={"OK" if pitch_ok else "FAIL"}  md5={md5(p)}')
        if r.returncode or not roll_ok or not pitch_ok:
            bad += 1
            if r.stderr: print(r.stderr)
    print('\n结果:', '全部通过' if bad == 0 else f'{bad} 个文件异常')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
