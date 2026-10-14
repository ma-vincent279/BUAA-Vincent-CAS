#!/usr/bin/env python3
# ================================================================
# 2026-09-27 新建项目 g = 项目 f 的完整副本 + 按 9-27 实飞落点重标"两瓶横向补偿"
#   用户要求: e、f 都不动, 新建 g 并做仿真验证
#   改动只有两个常量 (其余全部沿用 f):
#     第 1 瓶(右瓶, 释放点在相机轴右 +5.5cm): +0.048 -> +0.113  (9-27 两次实测落点 +5/+8cm)
#     第 2 瓶(左瓶, 释放点在相机轴左 -5.5cm): +0.048 -> -0.025  (**方向必须反号**, 原来把落点推到左 10cm)
#     纵向 XF=+0.040 保持不变 (用户确认落点基本不偏前后 => 该补偿正确)
#   用法: python3 scripts/new_project_g_20260927.py
# ================================================================
import ast, difflib, os, re, shutil, sys, time
ROOT='/home/vincent/SITL'; STAMP=time.strftime('%Y%m%d_%H%M%S')
PAIRS=[('src/drone_drl/drone_drl/f.py','src/drone_drl/drone_drl/g.py'),
       ('fly/src/fly/fly/f.py','fly/src/fly/fly/g.py')]
DOC_RE=re.compile(r'(#!/usr/bin/env python3\n)"""(.*?)"""\n', re.S)
DOC_NEW='''#!/usr/bin/env python3
"""
子项目 g — 投放区 30m + 侦察区动作序列 + 返航 (基底 = 项目 f, 只改"两瓶横向补偿").

来历 (2026-09-27 新建, 用户要求: e/f 都不动, 新建 g):
  - 基底 = f.py 逐字节复制 (f 一行未改), 只替换 节点名/类名/入口名/MISSION_REVISION + 两个常量
  - 唯一实质改动 = PAYLOAD_OFFSET_BODY_Y_B1/B2 (横向瞄准补偿), 依据 9-27 双航班实飞落点

依据 (9-27 实飞, e 一次 + f 一次, 落点为首次落点不计弹跳):
  两瓶挂载是镜像的: 第 1 瓶挂在机体右侧(释放点在相机轴 右+5.5cm), 第 2 瓶挂在左侧(左-5.5cm),
  而 e/f 对两瓶用同一个横向补偿(+0.048 = 瞄准点左移), 对第 2 瓶方向是反的.
    第 1 瓶落点: e 偏右 5cm, f 偏右 8cm   => 需要 +0.098~+0.128 (均值 +0.113)
    第 2 瓶落点: e 偏左 12cm, f 偏左 2~3cm => 需要 -0.072~+0.023 (均值 -0.025)
  纵向: 用户确认基本不偏 => XF=+0.040 正确, 保持不变.
  预期改后落点: 第 1 瓶 -1.5/+1.5cm, 第 2 瓶 -4.8/+4.8cm (桶口半径 12.5cm).

其余 (两级释放判据 Tier1/Tier2/Tier3、DROP 段锚点+反投影伺服、俯冲段、侦察 7 子阶段、
RETURN 判稳、LAND 限速) 与 f 完全一致.
"""
'''
CONST_OLD='''PAYLOAD_OFFSET_BODY_Y_B1 = +0.048   # 横向 (落点偏右 4.8cm -> 瞄准点左移)
PAYLOAD_OFFSET_BODY_Y_B2 = +0.048
PAYLOAD_OFFSET_BODY_XF_B1 = +0.040  # 前向 (2026-09-23 新增): 瞄准点后移 4.0cm
PAYLOAD_OFFSET_BODY_XF_B2 = +0.040'''
CONST_NEW='''# ===== 2026-09-27 项目 g: 两瓶横向补偿按 9-27 实飞落点分别重标 =====
# 两瓶镜像挂载: 第 1 瓶在机体右(释放点=相机轴右 +5.5cm), 第 2 瓶在左(-5.5cm).
# 9-27 实测首次落点: 第 1 瓶 e 偏右 5cm / f 偏右 8cm; 第 2 瓶 e 偏左 12cm / f 偏左 2~3cm.
#   => 第 1 瓶需要 +0.113 (原 0.048 只有一半); 第 2 瓶需要 -0.025 (**必须反号**)
# 纵向: 9-27 实测基本不偏前后 => XF=+0.040 正确, 保持不变.
PAYLOAD_OFFSET_BODY_Y_B1 = +0.113   # 第 1 瓶(右瓶): 0.048 -> 0.113
PAYLOAD_OFFSET_BODY_Y_B2 = -0.025   # 第 2 瓶(左瓶): +0.048 -> -0.025 (方向反号, 原值把落点推到左 ~10cm)
PAYLOAD_OFFSET_BODY_XF_B1 = +0.040  # 纵向: 保持 9-23 标定 (9-27 实测无前后偏差)
PAYLOAD_OFFSET_BODY_XF_B2 = +0.040'''
RENAMES=[('FMission','GMission',False),(r'\bf_mission\b','g_mission',True),
         ('"MISSION SUMMARY (project f)"','"MISSION SUMMARY (project g)"',False),
         ('f"F Mission ready. HEADING=','f"G Mission ready. HEADING=',False),
         ('MISSION_REVISION = "competition-water-20260924-r1-f"','MISSION_REVISION = "competition-water-20260927-r1-g"',False)]

def patch(src,path):
    m=DOC_RE.match(src); assert m, f'{path}: 无文件头 docstring'
    src=DOC_NEW+src[m.end():]
    for old,new,isre in RENAMES:
        pat=old if isre else re.escape(old)
        src,n=re.subn(pat,lambda mm:new,src); assert n>=1, f'{path}: 命名 {old!r} 未命中'
        print(f'    rename {old!r} -> {new!r} x{n}')
    assert src.count(CONST_OLD)==1, f'{path}: 常量锚点 {src.count(CONST_OLD)} 次'
    src=src.replace(CONST_OLD,CONST_NEW,1)
    return src

def main():
    print('=== 1. f -> g (两份) ===')
    outs=[]
    for a,b in PAIRS:
        pa,pb=os.path.join(ROOT,a),os.path.join(ROOT,b)
        if os.path.exists(pb):
            shutil.copy2(pb,pb+f'.bak.{STAMP}'); print(f'   {b} 已存在 -> 备份')
        out=patch(open(pa,encoding='utf-8').read(),b); ast.parse(out)
        open(pb,'w',encoding='utf-8').write(out); os.utime(pb,None); outs.append(b)
        print(f'   OK {b}  {len(out.splitlines())} 行')
    print('=== 2. worlds/g.world  scripts/test_g.sh  fly/g.sh ===')
    shutil.copy2(os.path.join(ROOT,'worlds/f.world'),os.path.join(ROOT,'worlds/g.world'))
    sh=open(os.path.join(ROOT,'scripts/test_f.sh'),encoding='utf-8').read()
    sh=sh.replace('子项目 f — 一键启动 (基底 = 项目 e, 只改释放判据)','子项目 g — 一键启动 (基底 = 项目 f, 只改两瓶横向补偿)')
    sh=sh.replace('''# 与 e 的唯一差异 = 两级释放判据 (方案 B, 2026-09-24):''','''# 与 f 的唯一差异 = 两瓶横向补偿按 9-27 实飞落点重标 (2026-09-27):
#   第 1 瓶(右瓶) +0.113, 第 2 瓶(左瓶) -0.025 (反号; 原 f 对两瓶都用 +0.048)
# f 的两级释放判据 (方案 B, 2026-09-24) 原样保留:''')
    for a,b in [('test_f.sh','test_g.sh'),('/tmp/f_output.log','/tmp/g_output.log'),('/tmp/f_judge.log','/tmp/g_judge.log'),
                ('worlds/f.world','worlds/g.world'),('World: f.world','World: g.world'),
                ('=== 8. 启动 F 任务 ===','=== 8. 启动 G 任务 ==='),('ros2 run drone_drl f 2>&1','ros2 run drone_drl g 2>&1'),
                ('f 完成喵~','g 完成喵~')]:
        sh=sh.replace(a,b)
    assert 'g.world' in sh and 'drone_drl g' in sh and 'f.world' not in sh
    open(os.path.join(ROOT,'scripts/test_g.sh'),'w',encoding='utf-8').write(sh)
    os.chmod(os.path.join(ROOT,'scripts/test_g.sh'),0o755)
    fsh=open(os.path.join(ROOT,'fly/f.sh'),encoding='utf-8').read().replace('子项目 f 实机一键启动','子项目 g 实机一键启动').replace('ros2 run fly f','ros2 run fly g').replace('bash ~/fly/f.sh','bash ~/fly/g.sh').replace('fly/f.sh','fly/g.sh')
    open(os.path.join(ROOT,'fly/g.sh'),'w',encoding='utf-8').write(fsh); os.chmod(os.path.join(ROOT,'fly/g.sh'),0o755)
    print('   OK')
    print('=== 3. setup.py 入口 ===')
    for rel,anchor,entry,marker in [('src/drone_drl/setup.py',"        'f = drone_drl.f:main',\n","        'g = drone_drl.g:main',\n","'g = drone_drl.g:main'"),
                                    ('fly/src/fly/setup.py',"            'f = fly.f:main',\n","            'g = fly.g:main',\n","'g = fly.g:main'")]:
        p=os.path.join(ROOT,rel); t=open(p,encoding='utf-8').read()
        if marker in t: print(f'   {rel}: 已存在'); continue
        assert t.count(anchor)==1, f'{rel}: 锚点 {t.count(anchor)}'
        shutil.copy2(p,p+f'.bak.{STAMP}'); open(p,'w',encoding='utf-8').write(t.replace(anchor,anchor+entry)); os.utime(p,None)
        print(f'   {rel}: + g 入口')
    print('=== 4. 校验 ===')
    for idx,b in enumerate(outs):
        s=open(os.path.join(ROOT,b),encoding='utf-8').read(); ast.parse(s)
        for k in ['PAYLOAD_OFFSET_BODY_Y_B1 = +0.113','PAYLOAD_OFFSET_BODY_Y_B2 = -0.025',
                  'PAYLOAD_OFFSET_BODY_XF_B1 = +0.040','PAYLOAD_OFFSET_BODY_XF_B2 = +0.040',
                  'DROP_ARRIVE_M    = 0.05','DROP_ARRIVE_M_LATE = 0.10','DROP_ARRIVE_VMAX','_drop_conv_min']:
            assert k in s, f'{b} 缺 {k}'
        for bad in ['FMission','f_mission','project f']: assert bad not in s, f'{b} 仍含 {bad}'
        assert 'self._drop_records = []' in s and 'self._drop_records.append' in s
        base=open(os.path.join(ROOT,PAIRS[idx][0]),encoding='utf-8').read()
        d=[l for l in difflib.unified_diff(base.splitlines(),s.splitlines(),n=0) if l[:1] in '+-' and l[:3] not in ('---','+++')]
        assert len(d)<160, f'{b} 与 f 差异过大 ({len(d)} 行)'
        print(f'   {b}: 与 f 差异 {len(d)} 行 (仅命名+两常量+注释)')
    bl=[]
    for b in outs:
        s=open(os.path.join(ROOT,b),encoding='utf-8').read()
        i=s.index('# ===== 2026-09-24 f: 两级释放判据 (方案 B) ====='); j=s.index('if not self._drop_triggered and not early_ok',i)
        bl.append(s[i:j])
    assert bl[0]==bl[1], '两份 g.py 释放判据不一致'
    print('   两份 g.py: 语法/常量/结构/判据一致性 全部 OK')
    print(f'\n完成 (备份后缀 .bak.{STAMP})')
if __name__=='__main__': main()
