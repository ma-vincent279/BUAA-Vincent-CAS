#!/usr/bin/env python3
# ================================================================
# 2026-09-23 (第二轮, 按用户指示) 项目 e: DROP 收敛窗口 25s → 35s
#   "收敛时间延长到35秒, 然后收敛的偏差再放宽到等效10cm"
#   - 收敛时间: DROP_AFTER_DELAY 25 → 35 (给伺服更多时间自己收敛, 再走兜底)
#   - 收敛偏差: 判据已是 **米制 DROP_ARRIVE_M=0.10 (等效 10cm)** ← 本脚本只做核对, 不再改
#   SITL + fly 两份一起改; 自动备份; 改后 py_compile + 常量核对
# 用法: python3 scripts/fix_e_drop35_20260923.py [--check]
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

A_OLD = ('DROP_AFTER_DELAY = 25.0  # 2026-09-12 e: 15→25 (同 d), 给反投影伺服更多收敛窗口; '
         '收敛则走"收敛即投"提前投弹')
A_NEW = ('DROP_AFTER_DELAY = 35.0  # 2026-09-12 e: 15→25 (同 d); 2026-09-23 按用户指示 25→35:\n'
         '                          #   把"自己收敛"的窗口再拉长 10s (兜底只在真收敛不了时才用).\n'
         '                          #   注意: 收敛偏差判据 = DROP_ARRIVE_M=0.10 (等效 10cm, 见下)')

B_OLD = '# 只判 err 会高空投弹. 不满足则 DROP_AFTER_DELAY=25s 兜底.'
B_NEW = '# 只判 err 会高空投弹. 不满足则 DROP_AFTER_DELAY=35s 兜底.'


def md5(p):
    return hashlib.md5(open(p, 'rb').read()).hexdigest()[:8]


def main():
    print('=== e: DROP 收敛窗口 25s → 35s (SITL + fly) ===\n')
    todo = []
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        if 'DROP_AFTER_DELAY = 35.0' in src:
            print(f'{rel}  md5={md5(p)}  → 已是 35s, 跳过\n')
            continue
        bad = [(n, src.count(o)) for n, o in (('常量行', A_OLD), ('兜底注释', B_OLD)) if src.count(o) != 1]
        if bad:
            print(f'{rel}  ✘ 锚点异常: {bad} → 中止')
            return 1
        print(f'{rel}  md5={md5(p)}  锚点 2/2 唯一 ✓\n')
        todo.append((p, src))
    if not todo:
        print('无需改动.')
    elif CHECK:
        print('[--check] 未写入.')
    else:
        for p, src in todo:
            bak = f'{p}.bak.{STAMP}_drop35'
            shutil.copy2(p, bak)
            src = src.replace(A_OLD, A_NEW, 1).replace(B_OLD, B_NEW, 1)
            open(p, 'w', encoding='utf-8').write(src)
            os.utime(p, None)
            print(f'备份 → {os.path.relpath(bak, ROOT)}')
            print(f'已改 → {os.path.relpath(p, ROOT)}  md5={md5(p)}')

    if CHECK or not todo:
        pass
    print('\n--- 核对 (两份都要满足) ---')
    bad = 0
    for rel in FILES:
        p = os.path.join(ROOT, rel)
        src = open(p, encoding='utf-8').read()
        r = subprocess.run([sys.executable, '-m', 'py_compile', p], capture_output=True, text=True)
        checks = {
            'DROP_AFTER_DELAY = 35.0': re.search(r'^DROP_AFTER_DELAY = 35\.0', src, re.M) is not None,
            'DROP_ARRIVE_M = 0.10 (等效10cm)': re.search(r'^DROP_ARRIVE_M\s+= 0\.10', src, re.M) is not None,
            '判据用 conv_m < DROP_ARRIVE_M': 'conv_m < DROP_ARRIVE_M' in src,
            '确认时间 DROP_CONVERGE_EARLY = 1.0': re.search(r'^DROP_CONVERGE_EARLY = 1\.0', src, re.M) is not None,
            'STEP_TIMEOUT 仍 > 35s (不会提前强推)': re.search(r'^STEP_TIMEOUT\s+= (\d+\.\d+)', src, re.M)
                                          and float(re.search(r'^STEP_TIMEOUT\s+= (\d+\.\d+)', src, re.M).group(1)) > 35.0,
            'MISSION_TIMEOUT 仍 > 两投 × 35s': re.search(r'^MISSION_TIMEOUT = (\d+)\.0', src, re.M)
                                          and float(re.search(r'^MISSION_TIMEOUT = (\d+)\.0', src, re.M).group(1)) > 70.0,
        }
        badc = [k for k, v in checks.items() if not v]
        print(f'{rel}: py_compile={"OK" if r.returncode == 0 else "FAIL"}  md5={md5(p)}'
              + ('  ✓ ' + ' | '.join(checks) if not badc else f'  ✘ 未过: {badc}'))
        if r.returncode or badc:
            bad += 1
            if r.stderr:
                print(r.stderr)
    print('\n结果:', '全部通过 (下一步: colcon build + SITL 复验)' if bad == 0 else f'{bad} 个文件异常')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
