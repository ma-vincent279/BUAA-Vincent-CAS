#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新建项目 k (2026-10-08): 定点投弹测试台 —— 由 j.py 分叉, 只改"飞多远/投哪/停多久"

用户确认的实验台 (H=1.2m, D=3m, 两投停留 5s/30s, **同一个桶**):
  TAKEOFF → 升到 1.2m → 前进 3m → 视觉对准桶 → 停 5s → 投(1 号瓶)
          → 爬升回位 → 再对准**同一个桶** → 停 30s → 投(2 号瓶) → 返航降落
目的: 一趟同时量出 ① 两瓶对同一桶 (分开 r(B1)/r(B2)) ② 5s vs 30s (停留期参照漂移贡献)
      + 每投日志里的 dev/偏差 (j 的仪表已带) ⇒ 现场只需卷尺量"桶心/飞机投影/落点"三点。

只改 7 处 (i.py / j.py 一行不动):
  1) FORWARD_DIST 34.0 → 3.0            (前进距离)
  2) I_DWELL_S 处新增 K_DWELL_S1/S2 = 5.0 / 30.0
  3) _settled 的停留判据改用 K_DWELL_S1/S2 (第 1 投 5s, 第 2 投 30s)
  4) I_RELEASE_TIMEOUT 20.0 → 45.0      (必须 > 30s 停留, 否则 20s 必投兜底会抢跳)
  5) SERVED_EXCLUDE_M 0.60 → 0.0        (允许再次匹配"已投过的桶")
  6) SERVED_DUP_M → 0.0                 (关掉"目标与已投桶重合就放弃"的拦截)
  7) 选桶后把第 2 个目标复制成第 1 个 (两瓶打同一桶); 投完不再去侦察区, 直接返航
"""
import datetime
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
S_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/j.py')
S_FLY = os.path.join(ROOT, 'fly/src/fly/fly/j.py')
D_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/k.py')
D_FLY = os.path.join(ROOT, 'fly/src/fly/fly/k.py')

K_CONST = '''K_DWELL_S1      = 5.0    # k 测试台: 第 1 投停留 (s)
K_DWELL_S2      = 30.0   # k 测试台: 第 2 投停留 (s) —— 用来量"停留期参照漂移"的贡献
'''
# P9: 精度杠杆 —— dev 超限时用当前帧重新冻结 (默认关, 行为与 i 一致)
K_LEVER_CONST = '''# **精度杠杆 (k 的核心用途)**: i 在 1.2m 冻结后是开环, 对"冻结之后参照又变了"无感知 ⇒ dev 直接
#   变成落点误差 (实测 L ≈ −dev)。dev 超限时用**当前帧**像素重算 est 并重新对准 (= 把该项消掉),
#   并重置停留计时; 次数上限 + I_RELEASE_TIMEOUT 硬兜底 ⇒ 不影响"必投"。
#   **默认 0.00 = 关闭 ⇒ 行为与 i 完全一致**; SITL 里 dev≈0 永不触发 (只能验链路, 收益须现场验)。
J_DEV_REFREEZE_M   = 0.00   # 停留期 dev 超过此值 ⇒ 重新冻结 (0=关; 实测分界 ≈ 桶口半径 0.125)
J_DEV_REFREEZE_MAX = 3      # 单投最多重新冻结次数
'''
# P11: 重新冻结块 (插在 _dev_ok 之后; 与首次冻结同一公式, 但不重置 _i_freeze_t ⇒ 必投兜底不变)
REFREEZE_BLOCK = '''                    # ===== k(2026-10-08): 精度杠杆 —— dev 超限 ⇒ 用当前帧重新冻结 (默认关) =====
                    #   机制: 保持点改为"当前帧桶估计 + 两瓶补偿"(与首次冻结同一公式), 停留计时重置,
                    #   飞控位置偏置在同一坐标系内自动抵消 ⇒ 对"桶被吹动"和"EKF 漂移"两种机制都有效.
                    #   不重置 _i_freeze_t ⇒ I_RELEASE_TIMEOUT 仍从首次冻结起算 ⇒ 必投保证不变.
                    if (J_DEV_REFREEZE_M > 0.0 and _dev_m is not None
                            and _dev_m > J_DEV_REFREEZE_M and _estl is not None
                            and getattr(self, '_k_frz_n', 0) < J_DEV_REFREEZE_MAX):
                        _dy2 = (PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0
                                else PAYLOAD_OFFSET_BODY_Y_B2)
                        _dx2 = (PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0
                                else PAYLOAD_OFFSET_BODY_XF_B2)
                        self._i_est_f = (_estl[0], _estl[1])
                        self._i_hold_x = (_estl[0] + _dy2 * math.sin(self.heading)
                                          - _dx2 * math.cos(self.heading))
                        self._i_hold_y = (_estl[1] - _dy2 * math.cos(self.heading)
                                          - _dx2 * math.sin(self.heading))
                        self._i_arrive_t = None          # 重新走"到位 → 停留" (本帧不放行)
                        self._k_frz_n = getattr(self, '_k_frz_n', 0) + 1
                        self.get_logger().warn(
                            f"k模式: 重新冻结 #{self._k_frz_n} dev={_dev_m * 100:.1f}cm > "
                            f"{J_DEV_REFREEZE_M * 100:.0f}cm → 保持点 ({self._i_hold_x:+.3f},"
                            f"{self._i_hold_y:+.3f}) (桶估计={_estl}) 停留计时重置")
                        return
'''
# (正则, 替换, 说明, 期望命中数)
PATCHES = [
    (r'FORWARD_DIST\s*=\s*34\.0', 'FORWARD_DIST  = 3.0', 'P1 前进 3m', 1),
    (r'(I_DWELL_S\s*=\s*5\.0[^\n]*\n)', r'\1' + K_CONST, 'P2 新增停留常量', 1),
    (r"I_RELEASE_TIMEOUT\s*=\s*20\.0", 'I_RELEASE_TIMEOUT = 45.0', 'P3 兜底 20→45s', 1),
    (r'SERVED_EXCLUDE_M\s*=\s*0\.60', 'SERVED_EXCLUDE_M      = 0.00', 'P4 允许复用已投桶', 1),
    (r'SERVED_DUP_M\s*=\s*[\d.]+', 'SERVED_DUP_M      = 0.00', 'P5 关掉重合拦截', 1),
    # P6a/b: 场地上只摆 1 个桶时也要能选 (i 的逻辑是"≥2 个才选", 单桶场地必须放宽这一处)
    (r'if len\(confirmed\) < 2:', 'if len(confirmed) < 1:   # k: 单桶场地也允许选', 'P6a 单桶可选', 1),
    (r'if len\(picked\) < 2:', 'if len(picked) < 1:   # k: 单桶场地也允许选', 'P6b 单桶可选', 1),
    (r"                    if _err < I_ARRIVE_M and getattr\(self, '_i_arrive_t', None\) is None:\n"
     r"                        self\._i_arrive_t = sim_t\n"
     r"                        self\.get_logger\(\)\.warn\(f\"j模式: 已到达保持点 err=\{_err \* 100:\.1f\}cm → 开始停留 \{I_DWELL_S:\.0f\}s\"\)\n"
     r"                    _settled = \(getattr\(self, '_i_arrive_t', None\) is not None\n"
     r"                                and \(sim_t - self\._i_arrive_t\) >= I_DWELL_S\)\n",
     "                    _dwell_need = (K_DWELL_S1 if self._drop_count == 0 else K_DWELL_S2)\n"
     "                    if _err < I_ARRIVE_M and getattr(self, '_i_arrive_t', None) is None:\n"
     "                        self._i_arrive_t = sim_t\n"
     "                        self.get_logger().warn(f\"j模式: 已到达保持点 err={_err * 100:.1f}cm → 开始停留 {_dwell_need:.0f}s\")\n"
     "                    _settled = (getattr(self, '_i_arrive_t', None) is not None\n"
     "                                and (sim_t - self._i_arrive_t) >= _dwell_need)\n",
     'P6 逐投停留时长 + WARN 用真实时长', 1),
    (r"            self\._target_count = len\(selected\)[^\n]*\n",
     lambda m: m.group(0) +
     "            # k(2026-10-08): 两瓶打同一个桶 (配合 SERVED_EXCLUDE_M=0 / SERVED_DUP_M=0)\n"
     "            if self._target_buckets:\n"
     "                self._target_buckets = [self._target_buckets[0]] * 2\n"
     "                self._target_ids = ([self._target_ids[0]] * 2) if self._target_ids else []\n"
     "                self._target_count = len(self._target_buckets)   # k: 计划投 2 次 ⇒ 分母也变 2\n"
     "                self.get_logger().info('  [k] 两瓶打同一个桶: ' + str(self._target_buckets[0]))\n",
     'P7 同桶两投 + SUMMARY 分母', 1),
    (r'J_LOG_DEV      = True[^\n]*\n',
     lambda m: m.group(0) + K_LEVER_CONST,
     'P9 精度杠杆常量', 1),
    (r'                    _dev_m = None\n'
     r'                    if \(J_DEV_GATE_M > 0\.0 or J_LOG_DEV\) and self\._drop_detection_fresh\(\) \\\n',
     '                    _dev_m = None\n'
     '                    _estl = None\n'
     '                    if (J_DEV_GATE_M > 0.0 or J_LOG_DEV or J_DEV_REFREEZE_M > 0.0) \\\n'
     '                            and self._drop_detection_fresh() \\\n',
     'P10 dev 采样门为重新冻结放开', 1),
    (r'                    _dev_ok = \(J_DEV_GATE_M <= 0\.0\) or \(_dev_m is None\) or \(_dev_m <= J_DEV_GATE_M\)\n',
     lambda m: m.group(0) + REFREEZE_BLOCK,
     'P11 dev 超限重新冻结 (默认关)', 1),
    (r'                self\.get_logger\(\)\.info\("All buckets done, flying to SCOUT zone"\)\n'
     r'                self\._enter_step\(STEP_SCOUT\)\n',
     '                self.get_logger().info("All buckets done, returning (k 测试台, 不飞侦察区)")\n'
     '                self._enter_step(STEP_RETURN)\n',
     'P8 投完直接返航', 1),
    # P8b/c: 另外 3 处"去侦察区"也要改成返航 (搜索超时 2 处 + 目标重合放弃 1 处)
    (r'self\._enter_step\(STEP_SCOUT\); return\n',
     'self._enter_step(STEP_RETURN)   # k: 测试台不飞侦察区, 一律返航; return\n',
     'P8b 搜索超时直接返航', 2),
    (r'                        f"⇒ 放弃本桶, 直接去侦察区 \(防双投同一桶\)"\)\n'
     r'                    self\._enter_step\(STEP_SCOUT\)\n',
     '                        f"⇒ 放弃本桶, 直接去侦察区 (防双投同一桶)")\n'
     '                    self._enter_step(STEP_RETURN)   # k: 测试台不飞侦察区, 一律返航\n',
     'P8c 重合放弃直接返航', 1),
]
SETUP = {
    os.path.join(ROOT, 'src/drone_drl/setup.py'): ("'j = drone_drl.j:main',", "'k = drone_drl.k:main',"),
    os.path.join(ROOT, 'fly/src/fly/setup.py'): ("'j = fly.j:main',", "'k = fly.k:main',"),
}


def apply(txt, name, check_only):
    for rx, rep, desc, n in PATCHES:
        hits = re.findall(rx, txt)
        if len(hits) != n:
            print(f'  ✗ {name}: {desc} 命中 {len(hits)} 次 (应为 {n})')
            return None
        if not check_only:
            txt = re.sub(rx, rep, txt, count=n)
    if not check_only:
        txt = txt.replace('j模式', 'k模式').replace('JMission', 'KMission').replace('j_mission', 'k_mission')
        txt = txt.replace('MISSION_REVISION = "competition-water-20260924-r1-f-i-j"',
                          'MISSION_REVISION = "competition-water-20260924-r1-f-i-j-k"')
    return txt


def main():
    print('=== new_project_k_20261008 ===')
    srcs = {}
    ok = True
    for s in (S_SITL, S_FLY):
        txt = open(s, encoding='utf-8').read()
        out = apply(txt, os.path.basename(s), True)
        if out is None:
            ok = False
        srcs[s] = txt
    for path, (a, b) in SETUP.items():
        t = open(path, encoding='utf-8').read()
        if b.strip("', ") in t:
            continue
        if t.count(a) != 1:
            print(f'  ✗ {os.path.basename(path)}: 锚点 {a} 命中 {t.count(a)} 次'); ok = False
    if not ok:
        print('锚点校验失败 ⇒ 不落盘'); sys.exit(1)
    print('锚点校验通过')
    if CHECK:
        print('--check: 只校验'); return
    for s, dst in ((S_SITL, D_SITL), (S_FLY, D_FLY)):
        t = apply(srcs[s], os.path.basename(s), False)
        open(dst, 'w', encoding='utf-8').write(t)
        print(f'  ✓ {os.path.relpath(dst, ROOT)} 写入 {len(t)} 字节')
    for path, (a, b) in SETUP.items():
        if b.strip("', ") in open(path, encoding='utf-8').read():
            print(f'  · {os.path.relpath(path, ROOT)} 已有 k 入口, 跳过'); continue
        shutil.copy2(path, f'{path}.bak.{ST}_newproject_k')
        open(path, 'w', encoding='utf-8').write(
            open(path, encoding='utf-8').read().replace(a, a + '\n        ' + b, 1))
        print(f'  ✓ {os.path.relpath(path, ROOT)} 加入入口 k')
    # world: **只有一个桶**, 摆在起飞点正前方 3m (用户 2026-10-08 要求)
    #   SITL 生成点 gz(1.01,0.98) + 3m ⇒ gz x=4.01; 用 drop_opt.world (本身就只有一个桶)
    w = open(os.path.join(ROOT, 'worlds/drop_opt.world'), encoding='utf-8').read()
    w2 = w.replace('<pose>11.01 0.98 0.25', '<pose>4.01 0.98 0.25')
    assert w2 != w, 'world 里没找到桶的 pose'
    open(os.path.join(ROOT, 'worlds/k.world'), 'w', encoding='utf-8').write(w2)
    print('  ✓ worlds/k.world (单个桶, 起飞点正前方 3m)')
    print('完成: i/j 一行未动')


if __name__ == '__main__':
    main()
