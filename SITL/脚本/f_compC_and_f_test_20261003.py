#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-03: ① f 采用保守补偿(方案C)  ② 新建 f_test = f 副本 + 补偿全归零 (测原始散布用)
f   : PAYLOAD_OFFSET_BODY_Y_B1 0.048->+0.25, Y_B2 0.048->+0.12 (横向), XF 保持 +0.040
f_test: 四个补偿常量全部归零 (含 XF), 只改 节点名/类名/入口名/MISSION_REVISION + 常量; 其余与 f 完全一致
"""
import os, re, sys, shutil, datetime, hashlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv

F_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/f.py')
F_FLY = os.path.join(ROOT, 'fly/src/fly/fly/f.py')
T_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/f_test.py')
T_FLY = os.path.join(ROOT, 'fly/src/fly/fly/f_test.py')

# ---------- ① f 保守补偿 ----------
F_REPL = [
    ('PAYLOAD_OFFSET_BODY_Y_B1 = +0.048   # 横向 (落点偏右 4.8cm -> 瞄准点左移)',
     'PAYLOAD_OFFSET_BODY_Y_B1 = +0.25    # 2026-10-03 保守(方案C): 0.048->+0.25 (换机架后实测右瓶持续偏右 0.35~0.42m)'),
    ('PAYLOAD_OFFSET_BODY_Y_B2 = +0.048',
     'PAYLOAD_OFFSET_BODY_Y_B2 = +0.12    # 2026-10-03 保守(方案C): 0.048->+0.12 (实测左瓶偏右 ~0.10m)'),
]

# ---------- ② f_test 归零 ----------
Z_REPL = [
    ('PAYLOAD_OFFSET_BODY_Y_B1 = +0.048   # 横向 (落点偏右 4.8cm -> 瞄准点左移)',
     'PAYLOAD_OFFSET_BODY_Y_B1 = 0.0      # 2026-10-03 f_test: 归零 (测原始机械+动态偏移散布)'),
    ('PAYLOAD_OFFSET_BODY_Y_B2 = +0.048', 'PAYLOAD_OFFSET_BODY_Y_B2 = 0.0'),
    ('PAYLOAD_OFFSET_BODY_XF_B1 = +0.040  # 前向 (2026-09-23 新增): 瞄准点后移 4.0cm',
     'PAYLOAD_OFFSET_BODY_XF_B1 = 0.0     # 2026-10-03 f_test: 归零'),
    ('PAYLOAD_OFFSET_BODY_XF_B2 = +0.040', 'PAYLOAD_OFFSET_BODY_XF_B2 = 0.0'),
    ('子项目 f —', '子项目 f_test — (f 副本, 四个投放补偿常量全部归零, 只用于测原始落点散布/辅助调参)'),
]


def rename(t):
    n1 = len(re.findall(r'\bFMission\b', t)); t = re.sub(r'\bFMission\b', 'FTestMission', t)
    n2 = len(re.findall(r"'f_mission'", t)); t = t.replace("'f_mission'", "'f_test_mission'")
    assert n1 >= 2 and n2 == 1, 'rename 失败 %d/%d' % (n1, n2)
    m = re.search(r'MISSION_REVISION = "([^"]+)"', t)
    assert m, 'MISSION_REVISION 未找到'
    t = t.replace(m.group(0), 'MISSION_REVISION = "%s-f_test"' % m.group(1))
    return t


def apply(t, repl):
    for old, new in repl:
        assert t.count(old) == 1, '锚点命中 %d: %r' % (t.count(old), old[:50])
        t = t.replace(old, new, 1)
    return t


def main():
    # ① f
    for p in (F_SITL, F_FLY):
        t = open(p, encoding='utf-8').read()
        new = apply(t, F_REPL)
        if CHECK:
            print('OK(check) f', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_compC')
        open(p, 'w', encoding='utf-8').write(new)
        print('f 保守补偿已应用', p)
    # ② f_test
    for src, dst in ((F_SITL, T_SITL), (F_FLY, T_FLY)):
        t = open(src, encoding='utf-8').read()
        new = rename(t)
        for _k in ('Y_B1', 'Y_B2', 'XF_B1', 'XF_B2'):
            new, _n = re.subn(r'(?m)^PAYLOAD_OFFSET_BODY_%s = [^\n]*$' % _k,
                              'PAYLOAD_OFFSET_BODY_%s = 0.0      # 2026-10-03 f_test: 归零 (测原始落点散布)' % _k,
                              new, count=1)
            assert _n == 1, '零化失败 %s' % _k
        for _o, _nn in Z_REPL[4:]:
            if _o in new:
                new = new.replace(_o, _nn, 1)
        assert new.count('0.0      # 2026-10-03 f_test') or 'PAYLOAD_OFFSET_BODY_Y_B1 = 0.0' in new
        if CHECK:
            print('OK(check) f_test', dst); continue
        if os.path.exists(dst):
            shutil.copy2(dst, dst + '.bak.' + ST)
        open(dst, 'w', encoding='utf-8').write(new)
        print('写出', dst)
    if CHECK:
        return
    # 世界 / 启动脚本 / setup 入口
    w = os.path.join(ROOT, 'worlds/f_test.world')
    if not os.path.exists(w):
        shutil.copy2(os.path.join(ROOT, 'worlds/f.world'), w); print('+', w)
    sg, sd = os.path.join(ROOT, 'scripts/test_f.sh'), os.path.join(ROOT, 'scripts/test_f_test.sh')
    if not os.path.exists(sd):
        s = open(sg, encoding='utf-8').read()
        s = s.replace('test_f.sh', 'test_f_test.sh').replace('子项目 f —', '子项目 f_test —')
        s = s.replace('f.world', 'f_test.world').replace('/tmp/f_output.log', '/tmp/f_test_output.log')
        s = s.replace('/tmp/f_judge.log', '/tmp/f_test_judge.log')
        s = s.replace('ros2 run drone_drl f ', 'ros2 run drone_drl f_test ')
        s = s.replace('f_mission', 'f_test_mission').replace('f_judge', 'f_test_judge')
        open(sd, 'w', encoding='utf-8').write(s); os.chmod(sd, 0o755); print('+', sd)
    fg, fd = os.path.join(ROOT, 'fly/f.sh'), os.path.join(ROOT, 'fly/f_test.sh')
    if not os.path.exists(fd):
        s = open(fg, encoding='utf-8').read()
        s = s.replace('f_mission', 'f_test_mission').replace('fly/f.py', 'fly/f_test.py')
        open(fd, 'w', encoding='utf-8').write(s); os.chmod(fd, 0o755); print('+', fd)
    for sp, pat in ((os.path.join(ROOT, 'src/drone_drl/setup.py'), "'f = drone_drl.f:main',"),
                    (os.path.join(ROOT, 'fly/src/fly/setup.py'), "'f = fly.f:main',")):
        s = open(sp, encoding='utf-8').read()
        i = s.find(pat)
        assert i >= 0, 'setup 未找到 %s' % pat
        line = s[i:s.find('\n', i)]
        newline = line.replace("'f = ", "'f_test = ").replace('.f:main', '.f_test:main')
        if newline.strip() not in s:
            indent = line[:len(line) - len(line.lstrip())]
            s = s.replace(line, line + '\n' + indent + newline.strip(), 1)
            shutil.copy2(sp, sp + '.bak.' + ST)
            open(sp, 'w', encoding='utf-8').write(s); print('+入口', sp)


if __name__ == '__main__':
    main()
    print('\n完成' if not CHECK else '\n[--check] 未写盘')
