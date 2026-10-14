#!/usr/bin/env python3
# ================================================================
# 2026-09-24 新建项目 f = 项目 e 的完整副本 + 方案 B (两级释放判据)
#   - e 一行都不动 (只读复制)
#   - 生成: src/drone_drl/drone_drl/f.py, fly/src/fly/fly/f.py,
#           worlds/f.world, scripts/test_f.sh, 两份 setup.py 入口, fly/f.sh
#   - 改动 (方案 B, 只动"何时投"):
#       1) DROP_ARRIVE_M   0.10 -> 0.05      (Tier1 优选门限)
#       2) DROP_CONVERGE_EARLY 1.0 -> 0.5    (Tier1 确认时间; 历史低位窗口仅 1~3s)
#       3) 低谷判定 (新增)  conv_m <= 本段最小值 + 2cm, 最小值下限夹 2cm
#       4) 准静止门控 (新增) vxy < 0.12 m/s  (弹道漂移封顶 ~6cm)
#       5) Tier2 保底 = e 的旧判据 (10cm/1.0s), 先给 Tier1 10s 优先窗口
#       6) Tier3 兜底 = DROP_AFTER_DELAY 35s 不变
#   用法: python3 scripts/new_project_f_20260924.py [--check]
# ================================================================
import ast
import os
import re
import shutil
import sys
import time

ROOT = '/home/vincent/SITL'
STAMP = time.strftime('%Y%m%d_%H%M%S')
CHECK = '--check' in sys.argv

PAIRS = [
    ('src/drone_drl/drone_drl/e.py', 'src/drone_drl/drone_drl/f.py'),
    ('fly/src/fly/fly/e.py', 'fly/src/fly/fly/f.py'),
]

# ---------- 1. 文件头 docstring ----------
DOC_OLD_RE = re.compile(r'(#!/usr/bin/env python3\n)"""(.*?)"""\n', re.S)
DOC_NEW = '''#!/usr/bin/env python3
"""
子项目 f — 投放区 30m + 侦察区动作序列 + 返航 (基底 = 项目 e, 只改"释放判据").

来历 (2026-09-24 新建, 用户要求: 新建 f, e 不动):
  - 基底 = e.py 逐字节复制 (e 一行未改), 只替换 节点名/类名/入口名/MISSION_REVISION
  - 改动 = 两级释放判据 (Tier1 优选相位 + Tier2 保底 + Tier3 35s 兜底), 见下方常量块

动机 (9-24 实飞 ULog 复盘):
  e 的判据只要求 conv_m < 10cm, 而最坏情况 = 10cm(残差) + 3~7.6cm(弹道漂移) > 桶半径 12.5cm
  => 最坏情况会脱靶; 9-24 两次命中是运气 (2.6/1.8cm, 且漂移方向恰好抵消残差).
依据 (8 份历史实飞日志回放): 几何正确时残差每轮都会穿过 2~4cm 低谷, 但"待在 5cm 以内"只有 1~3s.
改法:
  Tier1 (优选): conv_m<5cm 且处于本段低谷(<=本段最小值+2cm) 且准静止(vxy<0.12), 确认 0.5s
  Tier2 (保底): e 的旧判据 conv_m<10cm 持续 1.0s, 但先给 Tier1 DROP_PREFER_WINDOW=10s 机会
  Tier3 (兜底): DROP_AFTER_DELAY=35s 无条件释放 (不变)
  => 形态上"永不比 e 更差": Tier2 与 e 的判据逐字相同, 只是延后到 10s 之后才生效.
其余 (DROP 段锚点+反投影伺服 / 两轴瞄准补偿 / 俯冲段 / 侦察 7 子阶段 / RETURN 判稳 / LAND 限速)
全部与 e 完全一致.
"""
'''

# ---------- 2. 常量块 ----------
CONST_OLD = '''DROP_ARRIVE_M    = 0.10   # 2026-09-23 实飞标定: 判据改**米制 10cm** (原 10px 等效 3~4cm@1.2m 而实机误差
                          #   9~18cm => 永远达不到 => 两次都走 25s 兜底、随机相位释放 => 双脱靶).
                          #   与 d 在 9-19 实飞验证过的一致 (那两次收敛即投在 3.5/3.3cm 触发).
DROP_ARRIVE_PX   = 10.0   # 仅保留作历史参数留档, 不再是判据 (判据见 DROP_ARRIVE_M)
DROP_CONVERGE_EARLY = 1.0  # 2026-09-23: 1.5->1.0s (1Hz 回放显示补偿后 1.5s 未必凑得齐; 实测摄像头误差与真实偏差逐点 corr 0.90/0.95 => 低谷=真对齐)'''

CONST_NEW = '''# ===== 2026-09-24 项目 f: 两级释放判据 (方案 B) =====
# 释放精度 = 释放那一刻 conv_m (到补偿后瞄准点的地面距离). e 只要求 <10cm =>
# 最坏 10cm + 弹道漂移 3~7.6cm > 桶半径 12.5cm => 最坏会脱靶 (9-24 命中是运气).
# 历史 8 份实飞日志: 残差每轮都穿过 2~4cm 低谷, 但停留 <5cm 只有 1~3s => 确认时间须缩到 0.5s.
DROP_ARRIVE_M    = 0.05   # Tier1 优选门限 (米制): conv_m < 5cm
DROP_ARRIVE_M_LATE = 0.10  # Tier2 保底门限 = e 旧判据 (逐字相同, 保证永不比 e 更差)
DROP_CONVERGE_EARLY = 0.5  # Tier1 确认时间 (2026-09-24 f: 1.0->0.5s; 低位窗口仅 1~3s)
DROP_CONVERGE_LATE  = 1.0  # Tier2 确认时间 (同 e)
DROP_CONV_MIN_BAND  = 0.02 # 低谷带: 只允许在 conv_m <= 本段最小值 + 2cm 时释放
DROP_CONV_MIN_FLOOR = 0.02 # 本段最小值下限夹 2cm (防单帧噪声把门槛带偏)
DROP_ARRIVE_VMAX    = 0.12 # Tier1 准静止门控: 释放瞬间水平速度上限 (m/s) => 漂移 <= ~6cm
DROP_PREFER_WINDOW  = 10.0 # Tier1 优先窗口 (s): 前 10s 只认 Tier1, 之后 Tier2 也放行
DROP_ARRIVE_PX   = 10.0   # 仅保留作历史参数留档, 不再是判据'''

# ---------- 3. 状态复位 (__init__ 与 _enter_step) ----------
STATE_ANCHOR = '        self._drop_records = []'
STATE_ADD = '''        # 2026-09-24 f: 两级释放判据状态
        self._drop_conv_min = None            # 本段 DROP 残差最小值 (m, 下限夹 DROP_CONV_MIN_FLOOR)
        self._drop_tier = None                # 当前候选档位: 1=Tier1 优选 / 2=Tier2 保底 / None
        self._drop_records = []'''

RESET_ANCHOR = '''            self._drop_converge_start = None
            # 2026-09-13: 身份锁记账 + 像素残差积分 每桶复位'''
RESET_NEW = '''            self._drop_converge_start = None
            # 2026-09-24 f: 两级判据复位 (每桶/每段 DROP 重新记低谷)
            self._drop_conv_min = None
            self._drop_tier = None
            # 2026-09-13: 身份锁记账 + 像素残差积分 每桶复位'''

# ---------- 4. 释放判据 ----------
GATE_OLD = '''            early_ok = False
            if (release_valid and conv_m < DROP_ARRIVE_M):
                if self._drop_converge_start is None:
                    self._drop_converge_start = sim_t
                elif sim_t - self._drop_converge_start > DROP_CONVERGE_EARLY:
                    early_ok = True
            else:
                self._drop_converge_start = None'''

GATE_NEW = '''            # ===== 2026-09-24 f: 两级释放判据 (方案 B) =====
            # 低谷判定: 本段残差最小值 (下限夹 DROP_CONV_MIN_FLOOR 防单帧噪声把门槛带偏)
            if math.isfinite(conv_m):
                _prev_min = self._drop_conv_min if self._drop_conv_min is not None else conv_m
                self._drop_conv_min = max(DROP_CONV_MIN_FLOOR, min(_prev_min, conv_m))
            in_valley = (self._drop_conv_min is not None
                         and conv_m <= self._drop_conv_min + DROP_CONV_MIN_BAND)
            stay_still = math.hypot(self.vx, self.vy) < DROP_ARRIVE_VMAX
            tier = None
            if release_valid and conv_m < DROP_ARRIVE_M and in_valley and stay_still:
                tier = 1     # Tier1 优选: 5cm + 低谷 + 准静止
            elif (release_valid and self._step_elapsed() >= DROP_PREFER_WINDOW
                  and conv_m < DROP_ARRIVE_M_LATE):
                tier = 2     # Tier2 保底: e 的旧判据 (10cm/1.0s), 先让 Tier1 用完优先窗口
            if tier != self._drop_tier:          # 档位变化即重启确认计时 (避免混档提前放行)
                self._drop_tier = tier
                self._drop_converge_start = sim_t if tier is not None else None
            early_ok = False
            if tier is not None and self._drop_converge_start is not None:
                _need_t = DROP_CONVERGE_EARLY if tier == 1 else DROP_CONVERGE_LATE
                if sim_t - self._drop_converge_start > _need_t:
                    early_ok = True'''

# ---------- 5. 日志/总结 ----------
# 两份 e.py 的缩进不同 (SITL 24 空格 / fly 28 空格), 故 CONV 行用正则保留缩进
CONV_RE = re.compile(r'( *)conv = f"\{sim_t - self\._drop_converge_start:\.1f\}s" '
                     r'if self\._drop_converge_start is not None else "--"')


def conv_repl(m):
    ind = m.group(1)
    return (ind + 'conv = f"{sim_t - self._drop_converge_start:.1f}s/T{self._drop_tier}" '
                  'if self._drop_converge_start is not None else "--"\n'
            + ind + '_cmin_s = f"{self._drop_conv_min * 100:.1f}cm" '
                     'if self._drop_conv_min is not None else "--"')

GATE_LOG_OLD = '''cm(门限{DROP_ARRIVE_M * 100:.0f}cm)'''
GATE_LOG_NEW = '''cm(门限T1={DROP_ARRIVE_M * 100:.0f}cm/T2={DROP_ARRIVE_M_LATE * 100:.0f}cm)'''

REC_OLD = '''                self._drop_records.append({
                    "idx": self._drop_count,
                    "aim": aim_abs,
                    "pos": (self.x, self.y),
                    "H": self.home_z - self.z,
                    "last_err": self._last_drop_err,
                })'''
REC_NEW = '''                self._drop_records.append({
                    "idx": self._drop_count,
                    "aim": aim_abs,
                    "pos": (self.x, self.y),
                    "H": self.home_z - self.z,
                    "last_err": self._last_drop_err,
                    "tier": self._drop_tier,          # 2026-09-24 f: 释放档位 (1=优选/2=保底)
                    "conv_m": conv_m,                 # 释放瞬间 conv_m (m)
                    "conv_min": self._drop_conv_min,  # 本段残差最小值 (m)
                })'''

WAY_OLD = '''                drop_way = "收敛即投" if early_ok else f"{self._step_elapsed():.1f}s/{DROP_AFTER_DELAY:.0f}s 兜底"'''
WAY_NEW = '''                if early_ok:
                    _cminways = f"{self._drop_conv_min * 100:.1f}cm" if self._drop_conv_min is not None else "--"
                    _tname = "5cm低谷准静止" if self._drop_tier == 1 else "10cm保底"
                    drop_way = f"收敛即投(Tier{self._drop_tier} {_tname} 段内最小={_cminways})"
                else:
                    drop_way = f"{self._step_elapsed():.1f}s/{DROP_AFTER_DELAY:.0f}s 兜底"'''

SUM_OLD = '''vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured{err_str}'''
SUM_NEW = '''vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured tier=T{r.get('tier')} conv_min={r.get('conv_min') if r.get('conv_min') is None else round(r['conv_min'] * 100, 1)}cm{err_str}'''

# ---------- 6. 命名替换 ----------
RENAMES = [
    ('EMission', 'FMission', False),
    (r'\be_mission\b', 'f_mission', True),
    ('"MISSION SUMMARY (project e)"', '"MISSION SUMMARY (project f)"', False),
    ('f"E Mission ready. HEADING=', 'f"F Mission ready. HEADING=', False),
    ('MISSION_REVISION = "competition-water-20260920-r3-e"',
     'MISSION_REVISION = "competition-water-20260924-r1-f"', False),
]


def patch(src: str, path: str) -> str:
    """按锚点顺序打补丁, 每处断言唯一命中."""
    def rep1(text, old, new, tag):
        n = text.count(old)
        assert n == 1, f'{path}: 锚点 {tag} 命中 {n} 次 (期望 1)'
        return text.replace(old, new)

    # 1) 文件头 docstring
    m = DOC_OLD_RE.match(src)
    assert m, f'{path}: 未找到文件头 docstring'
    src = DOC_NEW + src[m.end():]

    # 2) 命名
    for old, new, is_re in RENAMES:
        pat = old if is_re else re.escape(old)
        src, n = re.subn(pat, lambda m: new, src)
        assert n >= 1, f'{path}: 命名替换 {old!r} 命中 0 次'
        print(f'    rename {old!r} -> {new!r} x{n}')

    # 3) 常量块
    src = rep1(src, CONST_OLD, CONST_NEW, 'CONST')
    # 4) 状态初始化
    src = rep1(src, STATE_ANCHOR, STATE_ADD, 'STATE')
    # 5) DROP 复位
    src = rep1(src, RESET_ANCHOR, RESET_NEW, 'RESET')
    # 6) 释放判据
    src = rep1(src, GATE_OLD, GATE_NEW, 'GATE')
    # 7) 日志
    src, n = CONV_RE.subn(conv_repl, src)
    assert n == 1, f'{path}: CONV 锚点命中 {n} 次 (期望 1)'
    n = src.count('收敛计时={conv}')
    assert n == 1, f'{path}: 收敛计时 锚点命中 {n} 次 (期望 1)'
    src = src.replace('收敛计时={conv}', '收敛计时={conv} 段内最小={_cmin_s}')
    src = rep1(src, GATE_LOG_OLD, GATE_LOG_NEW, 'GATELOG')
    src = rep1(src, REC_OLD, REC_NEW, 'REC')
    src = rep1(src, WAY_OLD, WAY_NEW, 'WAY')
    src = rep1(src, SUM_OLD, SUM_NEW, 'SUM')
    return src


def main():
    if CHECK:
        print('--check: 只检查目标文件是否已存在')
        for _, dst in PAIRS:
            print(f'   {dst}: {"存在" if os.path.exists(os.path.join(ROOT, dst)) else "不存在"}')
        return

    print(f'=== 1. 复制 e -> f ({len(PAIRS)} 份) + 打补丁 ===')
    outputs = []
    for rel_src, rel_dst in PAIRS:
        p_src, p_dst = os.path.join(ROOT, rel_src), os.path.join(ROOT, rel_dst)
        assert os.path.exists(p_src), f'缺少基底 {p_src}'
        if os.path.exists(p_dst):
            bak = p_dst + f'.bak.{STAMP}'
            shutil.copy2(p_dst, bak)
            print(f'   {rel_dst} 已存在 -> 备份 {os.path.basename(bak)}')
        src = open(p_src, encoding='utf-8').read()
        print(f'   patching {rel_src} -> {rel_dst}')
        out = patch(src, rel_dst)
        ast.parse(out)                      # 语法自检
        open(p_dst, 'w', encoding='utf-8').write(out)
        os.utime(p_dst, None)               # 防 colcon 按 mtime 跳过
        outputs.append(rel_dst)
        print(f'   OK {rel_dst}  {len(out.splitlines())} 行')

    print('\n=== 2. worlds/f.world (复制 e.world) ===')
    w_src, w_dst = os.path.join(ROOT, 'worlds/e.world'), os.path.join(ROOT, 'worlds/f.world')
    shutil.copy2(w_src, w_dst)
    print(f'   {os.path.getsize(w_dst)} B  (e.world {os.path.getsize(w_src)} B)')

    print('\n=== 3. scripts/test_f.sh (由 test_e.sh 改造) ===')
    t_src = os.path.join(ROOT, 'scripts/test_e.sh')
    t_dst = os.path.join(ROOT, 'scripts/test_f.sh')
    sh = open(t_src, encoding='utf-8').read()
    sh = sh.replace('子项目 e — 一键启动', '子项目 f — 一键启动 (基底 = 项目 e, 只改释放判据)')
    sh = sh.replace('''# 基于 c, 5 处改动 (2026-09-12):
#   1. DROP 段伺服 增量累加 → 锚点+反投影重建 (自 d); 俯冲段保持 c 原样
#   2. 飞行边界软墙完全移除
#   3. 选桶策略 → 优先离航向中线最近
#   4. 投放补偿全部去掉 (B1/B2 = 0)
#   5. 投弹 = DROP_AFTER_DELAY 25s 兜底 + 收敛即投 (带高度门控)''',
                   '''# 与 e 的唯一差异 = 两级释放判据 (方案 B, 2026-09-24):
#   Tier1 优选: conv_m<5cm + 低谷(<=本段最小+2cm) + 准静止(vxy<0.12), 确认 0.5s
#   Tier2 保底: e 的旧判据 conv_m<10cm 持续 1.0s (先给 Tier1 10s 优先窗口)
#   Tier3 兜底: DROP_AFTER_DELAY=35s 无条件释放 (未改)''')
    sh = sh.replace('test_e.sh', 'test_f.sh')
    sh = sh.replace('/tmp/e_output.log', '/tmp/f_output.log')
    sh = sh.replace('/tmp/e_judge.log', '/tmp/f_judge.log')
    sh = sh.replace('worlds/e.world', 'worlds/f.world')
    sh = sh.replace('World: e.world', 'World: f.world')
    sh = sh.replace('=== 8. 启动 E 任务 ===', '=== 8. 启动 F 任务 ===')
    sh = sh.replace('ros2 run drone_drl e 2>&1', 'ros2 run drone_drl f 2>&1')
    sh = sh.replace('e 完成喵~', 'f 完成喵~')
    assert 'f.world' in sh and 'drone_drl f' in sh and 'e.world' not in sh, 'test_f.sh 替换不完整'
    open(t_dst, 'w', encoding='utf-8').write(sh)
    os.chmod(t_dst, 0o755)
    print(f'   OK scripts/test_f.sh  {len(sh.splitlines())} 行')

    print('\n=== 4. setup.py 入口 (带备份) ===')
    for rel, anchor, entry, marker in [
        ('src/drone_drl/setup.py', "        'e = drone_drl.e:main',\n",
         "        'f = drone_drl.f:main',\n", "'f = drone_drl.f:main'"),
        ('fly/src/fly/setup.py', "            'e = fly.e:main',\n",
         "            'f = fly.f:main',\n", "'f = fly.f:main'"),
    ]:
        p = os.path.join(ROOT, rel)
        txt = open(p, encoding='utf-8').read()
        if marker in txt:
            print(f'   {rel}: f 入口已存在, 跳过')
            continue
        assert txt.count(anchor) == 1, f'{rel}: e 入口锚点命中 {txt.count(anchor)} 次'
        bak = p + f'.bak.{STAMP}'
        shutil.copy2(p, bak)
        open(p, 'w', encoding='utf-8').write(txt.replace(anchor, anchor + entry))
        os.utime(p, None)
        print(f'   {rel}: + f 入口 (备份 {os.path.basename(bak)})')

    print('\n=== 5. fly/f.sh 一键启动 (由 fly 的 e 启动方式生成) ===')
    fly_sh = os.path.join(ROOT, 'fly/f.sh')
    open(fly_sh, 'w', encoding='utf-8').write('''#!/bin/bash
# 子项目 f 实机一键启动 (基底 = 项目 e, 只改释放判据; 2026-09-24 新建)
# 用法 (Jetson): bash ~/fly/f.sh   [--no-build]
set -e
cd "$(dirname "$0")"
if [ "$1" != "--no-build" ]; then
    source /opt/ros/humble/setup.bash
    colcon build --packages-select fly 2>&1 | tail -5
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 run fly f
''')
    os.chmod(fly_sh, 0o755)
    print(f'   OK fly/f.sh')

    print('\n=== 6. 校验 ===')
    diffs = {}
    for (rel_src, rel_dst) in PAIRS:
        src = open(os.path.join(ROOT, rel_dst), encoding='utf-8').read()
        ast.parse(src)
        for k in ['DROP_ARRIVE_M    = 0.05', 'DROP_ARRIVE_M_LATE = 0.10', 'DROP_CONVERGE_EARLY = 0.5',
                  'DROP_CONV_MIN_BAND', 'DROP_CONV_MIN_FLOOR', 'DROP_ARRIVE_VMAX', 'DROP_PREFER_WINDOW',
                  '_drop_conv_min', 'in_valley', 'stay_still', '_drop_tier']:
            assert k in src, f'{rel_dst} 缺 {k}'
        for bad in ['EMission', 'e_mission', 'project e']:
            assert bad not in src, f'{rel_dst} 仍含 {bad}'
        # 结构完整性 (防"锚点替换把原行吃掉"这类自伤)
        assert 'self._drop_records = []' in src, f'{rel_dst} 丢了 _drop_records 初始化'
        assert 'self._drop_records.append' in src and 'for r in self._drop_records:' in src
        assert src.count('self._drop_conv_min') >= 6, f'{rel_dst} _drop_conv_min 出现次数不足'
        base = open(os.path.join(ROOT, rel_src), encoding='utf-8').read()
        d = [l for l in __import__('difflib').unified_diff(base.splitlines(), src.splitlines(), n=0)
             if l[:1] in '+-' and l[:3] not in ('---', '+++')]
        diffs[rel_dst] = len(d)
        assert len(d) < 160, f'{rel_dst} 与基底差异过大 ({len(d)} 行) — 疑似锚点误伤'
        print(f'   {rel_dst}: 与 e.py 差异 {len(d)} 行 (全部为预期改动)')
    print('   两份 f.py: 语法 OK / 新常量与新逻辑齐全 / 结构完整 / e 命名已清干净')

    # 两份 f.py 的释放判据块逐字一致
    blocks = []
    for rel_dst in outputs:
        src = open(os.path.join(ROOT, rel_dst), encoding='utf-8').read()
        i = src.index('# ===== 2026-09-24 f: 两级释放判据 (方案 B) =====')
        j = src.index('if not self._drop_triggered and not early_ok', i)
        blocks.append(src[i:j])
    assert blocks[0] == blocks[1], '两份 f.py 的释放判据块不一致'
    print('   两份 f.py 释放判据块逐字一致 ✓')

    print(f'\n全部完成 (备份后缀 .bak.{STAMP})')


if __name__ == '__main__':
    main()
