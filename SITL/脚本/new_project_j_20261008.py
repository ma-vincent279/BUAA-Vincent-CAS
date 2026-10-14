#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新建项目 j (2026-10-08): 从 i 脱胎, 只加"相机确认释放门" + 全量 dev 日志 (+ 补偿重标开关)

动机 (docs/project_log.md 追加 21/22 的真值反解):
  i 模式释放判据 `_pred = |(est_frozen + comp) − 飞机位置 − v·t_eff|` **全在飞控坐标系里**,
  而实测落点误差 `L ≈ −dev`, `dev = 当前帧相机反算 est_live − 冻结 est` ⇒ 冻结之后飞控的
  绝对位置参照一漂, 判据完全看不见 (10-08 那趟 `到位err=0.6cm` 却偏 13.9cm 就是它).
  14 投统计: `dev ≥22cm` 的 7 投 **全部 ≥12cm 偏差**; `dev ≤14cm` 的 7 投 **全部 ≤10cm**.

j 的改动 (i 一行未动, 另存为 j.py):
  ① 释放判据加 **相机确认门 (AND)**: 停稳 + 在保持点上 + `_pred<I_PRED_M` + `dev ≤ J_DEV_GATE_M`
     —— 必须与"飞机确实在保持点上"同时成立, 否则会在飞机还在收敛时**假达标**
     (离线回放已复现过这种假达标: 会给出 −40~−60cm 的荒诞预测)。
  ② 20s 兜底窗口里改挑 **|dev| 最小** 的一刻 (拿不到相机值就退回 pred); 窗口结束必投不变。
  ③ 1Hz 状态行在 DROP 段附加 `dev=(前,右)cm`; 释放行附加 `偏差=xxcm` ⇒ 以后每趟自带 dev 序列。
  ④ 补偿重标开关 `J_MEASURED_R` (默认 False = 保持 i 的 0.18/0.00): 打开后按实测 r 重标
     (Y_B1=+0.35 / Y_B2=+0.05, 依据 33~37cm 与 0±5cm 的反解) —— **等 j 第一趟实测数据再决定**,
     因为现值 0.18 与 dev 的符号相反、量级相当 (两个误差在对冲), 单改补偿会暴露 dev。
  ⚠ 本改动**不改瞄准点/冻结时机**, 只改"何时放行" ⇒ 飞机轨迹与 i 同构, 可安全回滚。

用法: python3 scripts/new_project_j_20261008.py [--check]
"""
import datetime
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
S_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/i.py')
S_FLY = os.path.join(ROOT, 'fly/src/fly/fly/i.py')
D_SITL = os.path.join(ROOT, 'src/drone_drl/drone_drl/j.py')
D_FLY = os.path.join(ROOT, 'fly/src/fly/fly/j.py')
SETUP_SITL = os.path.join(ROOT, 'src/drone_drl/setup.py')
SETUP_FLY = os.path.join(ROOT, 'fly/src/fly/setup.py')
W_SRC = os.path.join(ROOT, 'worlds/i.world')
W_DST = os.path.join(ROOT, 'worlds/j.world')

J_CONST = '''
# ===== 2026-10-08 项目 j: 相机确认释放门 (方案 = docs/project_log.md 追加 23) =====
#   dev = 当前帧相机反算 est_live − 冻结 est (机体 前/右 分量, 与节点同一相机模型, 不含 FX 不确定性).
#   实测: 落点误差 L ≈ −dev (6 投吻合 1~3.4cm); 14 投里 dev ≥22cm 的 7 投全部 ≥12cm 偏差,
#   dev ≤14cm 的 7 投全部 ≤10cm ⇒ dev 是"这一投会不会偏"的直接指示, 而 i 的判据看不见它.
J_DEV_GATE_M   = 0.00   # **默认 0 = 关闭该门** (慎用 0.10): 实测坏趟 dev 是单调长大的
                        #   ⇒ 拦住只会把释放推到更晚 ⇒ |dev| 更大 ⇒ 落点更差; 门要等
                        #   '停留期内 dev 回到窗口' 的实测证据才敢打开。
                        #   0.10 = 释放前要求 dev ≤ 0.10m (停稳 + 在保持点上 仍为必要条件)
J_DEV_DEADLINE_PICK = True   # 20s 兜底窗口改挑 |dev| 最小的一刻 (无相机值时退回 pred)
J_LOG_DEV      = True   # 1Hz 状态行在 DROP 段附加 dev=(前,右)cm; 释放行附加 偏差=xxcm
# 补偿重标开关: False = 保持 i 的现值 (Y_B1=+0.18 / Y_B2=0.00)。
#   反解 (追加 21/22): r(B1) = −37.2/−34.3/−33.0cm (三趟一致 ⇒ 真实右偏 ≈35cm, 代码只补 18cm);
#   r(B2) = +2.1/−7.5/−11.2cm (≈0)。**但现值 0.18 恰与 dev 反号 ⇒ 两个误差在对冲**,
#   单改补偿会把 dev 暴露出来 (最坏 25cm) ⇒ 默认关, 等 j 第一趟 偏差= 与落点真值出来再打开。
J_MEASURED_R   = False
if J_MEASURED_R:
    PAYLOAD_OFFSET_BODY_Y_B1 = +0.35
    PAYLOAD_OFFSET_BODY_Y_B2 = +0.05
'''

J_DEV_BLOCK = '''                    # ===== j(2026-10-08): 相机确认项 =====
                    #   dev = 当前帧 px 反算的桶估计 − 冻结用的桶估计 (机体 前/右 分量).
                    #   只在"检测新鲜 + 追踪器已确认 + 框不贴边"时才算 (与冻结采样同一套门控).
                    _dev_m = None
                    if (J_DEV_GATE_M > 0.0 or J_LOG_DEV) and self._drop_detection_fresh() \\
                            and self.latest_roi is not None \\
                            and self._i_target_tracker_ok() and self._i_roi_inside(self.latest_roi):
                        _estl = self._roi_to_ned(self.latest_roi)
                        _efz = getattr(self, '_i_est_f', None)
                        if _estl is not None and _efz is not None:
                            _dF, _dR = self._ned_to_body(_estl[0] - _efz[0], _estl[1] - _efz[1])
                            _dev_m = math.hypot(_dF, _dR)
                    #   失效开放: 拿不到相机值时**不拦** (退回 i 的旧判据), 否则检测一丢就会被卡到
                    #   20s+5s 兜底; 这样门只会拦住"确实测到偏了"的那一投, 不引入新风险.
                    _dev_ok = (J_DEV_GATE_M <= 0.0) or (_dev_m is None) or (_dev_m <= J_DEV_GATE_M)
'''

J_PICK = '''                    _pick = _dev_m if (J_DEV_DEADLINE_PICK and _dev_m is not None) else _pred
'''

J_STATUS = '''                if self.step == STEP_DROP and J_LOG_DEV:
                    _efz2 = getattr(self, '_i_est_f', None)
                    if _efz2 is not None and self.latest_roi is not None:
                        _estl2 = self._roi_to_ned(self.latest_roi)
                        if _estl2 is not None:
                            _d2F, _d2R = self._ned_to_body(_estl2[0] - _efz2[0], _estl2[1] - _efz2[1])
                            extra += f" dev=({_d2F * 100:+.0f},{_d2R * 100:+.0f})cm"
'''

# (锚点, 替换, 说明) —— 锚点必须在两份文件里各出现恰好 1 次
PATCHES = [
    ('子项目 i — (f 副本', '子项目 j — (i 副本 + 相机确认释放门',
     'P1 文档头'),
    ('MISSION_REVISION = "competition-water-20260924-r1-f-i"',
     'MISSION_REVISION = "competition-water-20260924-r1-f-i-j"',
     'P2 revision'),
    ('class IMission(Node):', 'class JMission(Node):', 'P3 类名'),
    ("super().__init__('i_mission')", "super().__init__('j_mission')", 'P4 节点名'),
    ('I_RELEASE_TIMEOUT = 20.0   # 从进 DROP 起的兜底 (s): 超时无论有没有检测都强制投放 (必投)\n',
     'I_RELEASE_TIMEOUT = 20.0   # 从进 DROP 起的兜底 (s): 超时无论有没有检测都强制投放 (必投)\n' + J_CONST,
     'P5 j 常量块'),
    ("                        _hx0, _hy0 = _est if _est is not None else (self._servo_target_x, self._servo_target_y)\n",
     "                        _hx0, _hy0 = _est if _est is not None else (self._servo_target_x, self._servo_target_y)\n"
     "                        self._i_est_f = (_hx0, _hy0)          # j: 记下冻结用的桶估计 (算 dev 的基准)\n",
     'P6 冻结记下 est'),
    ('''                    # D(2026-10-06): 预测落点误差 (EKF 量, 相机不参与控制)
                    _pred = math.hypot((self._i_hold_x - (self.x - self.home_x)) - self.vx * I_PRED_TEFF,
                                       (self._i_hold_y - (self.y - self.home_y)) - self.vy * I_PRED_TEFF)
                    _pred_ok = _pred < I_PRED_M
''',
     '''                    # D(2026-10-06): 预测落点误差 (EKF 量, 相机不参与控制)
                    _pred = math.hypot((self._i_hold_x - (self.x - self.home_x)) - self.vx * I_PRED_TEFF,
                                       (self._i_hold_y - (self.y - self.home_y)) - self.vy * I_PRED_TEFF)
                    _pred_ok = _pred < I_PRED_M
''' + J_DEV_BLOCK,
     'P7 相机确认门'),
    ('                    # ② 兜底挑时机: 到 20s 后不立刻投, 先在 I_DEADLINE_WIN 内等 pred 较优的一刻(窗口结束必投)\n',
     J_PICK + '                    # ② 兜底挑时机: 到 20s 后不立刻投, 先在 I_DEADLINE_WIN 内等"更优"的一刻(窗口结束必投)\n',
     'P8 兜底度量'),
    ('                    if _deadline and getattr(self, \'_i_dl_t0\', None) is None:\n'
     '                        self._i_dl_t0 = sim_t\n'
     '                        self._i_dl_best = _pred\n'
     '                    if _deadline:\n'
     '                        self._i_dl_best = min(getattr(self, \'_i_dl_best\', _pred), _pred)\n'
     '                    _dl_pick = bool(_deadline and (\n'
     '                        sim_t - getattr(self, \'_i_dl_t0\', sim_t) >= I_DEADLINE_WIN\n'
     '                        or _pred <= max(0.03, getattr(self, \'_i_dl_best\', 9.9) * 1.05)))\n',
     '                    if _deadline and getattr(self, \'_i_dl_t0\', None) is None:\n'
     '                        self._i_dl_t0 = sim_t\n'
     '                        self._i_dl_best = _pick\n'
     '                    if _deadline:\n'
     '                        self._i_dl_best = min(getattr(self, \'_i_dl_best\', _pick), _pick)\n'
     '                    _dl_pick = bool(_deadline and (\n'
     '                        sim_t - getattr(self, \'_i_dl_t0\', sim_t) >= I_DEADLINE_WIN\n'
     '                        or _pick <= max(0.03, getattr(self, \'_i_dl_best\', 9.9) * 1.05)))\n',
     'P9 兜底选时刻改用 _pick'),
    ('                    if (_settled and _vxy < I_VMAX and _pred_ok) or _dl_pick:\n',
     '                    if (_settled and _vxy < I_VMAX and _pred_ok and _dev_ok) or _dl_pick:\n',
     'P10 释放条件加 _dev_ok'),
    ('                            f"pred={_pred * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s "\n',
     '                            f"pred={_pred * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s "\n'
     '                            f"偏差={\'--\' if _dev_m is None else \'%.1fcm\' % (_dev_m * 100)} "\n',
     'P11 释放行加偏差'),
    ('                px = self._px_raw()\n'
     '                if px is not None:\n'
     '                    ex, ey, err = px\n'
     '                    extra += f" px=({ex:+.0f},{ey:+.0f})err={err:.0f}px"\n',
     '                px = self._px_raw()\n'
     '                if px is not None:\n'
     '                    ex, ey, err = px\n'
     '                    extra += f" px=({ex:+.0f},{ey:+.0f})err={err:.0f}px"\n'
     + J_STATUS,
     'P12 状态行加 dev'),
]

SETUP_ANCHORS = {
    SETUP_SITL: ("'i = drone_drl.i:main',", "'j = drone_drl.j:main',"),
    SETUP_FLY: ("'i = fly.i:main',", "'j = fly.j:main',"),
}


def validate():
    ok = True
    for src in (S_SITL, S_FLY):
        txt = open(src, encoding='utf-8').read()
        for anchor, _rep, name in PATCHES:
            n = txt.count(anchor)
            if n != 1:
                print(f'  ✗ {os.path.basename(src)}: {name} 锚点出现 {n} 次 (应为 1)')
                ok = False
        for a in ('i模式', 'i_mission'):
            pass
    for path, (a, _b) in SETUP_ANCHORS.items():
        if not os.path.exists(path):
            print(f'  ✗ 缺文件 {path}'); ok = False; continue
        t = open(path, encoding='utf-8').read()
        if t.count(a) != 1:
            print(f'  ✗ {os.path.basename(path)}: 入口锚点 {a} 出现 {t.count(a)} 次')
            ok = False
    if not os.path.exists(W_SRC):
        print(f'  ✗ 缺 {W_SRC}'); ok = False
    return ok


def build_one(src, dst):
    txt = open(src, encoding='utf-8').read()
    orig = txt
    for anchor, rep, _n in PATCHES:
        txt = txt.replace(anchor, rep, 1)
    txt = txt.replace('i模式', 'j模式')          # 日志前缀统一 (解析脚本已同步支持 [ij]模式)
    txt = txt.replace('IMission', 'JMission')    # 类名: 定义处 + main() 实例化处必须一起改
    txt = txt.replace('i_mission', 'j_mission')  # 节点名
    open(dst, 'w', encoding='utf-8').write(txt)
    # 最小改动审计: 剥注释后统计差异行
    def strip(t):
        return [l for l in t.splitlines() if l.strip() and not l.strip().startswith('#')]
    a, b = strip(orig), strip(txt)
    diff = sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))
    print(f'  ✓ {os.path.relpath(dst, ROOT)} 写入 {len(txt)} 字节 (剥注释后差异行 {diff})')
    return diff


def main():
    print('=== new_project_j_20261008 ===')
    if not validate():
        print('锚点校验失败 ⇒ 不落盘'); sys.exit(1)
    print('锚点校验通过 (两份源文件 + 两份 setup.py + world)')
    if CHECK:
        print('--check: 只校验, 不写盘'); return
    for path, (a, b) in SETUP_ANCHORS.items():
        if b.strip("', ") in open(path, encoding='utf-8').read():
            print(f'  · {os.path.relpath(path, ROOT)} 已有 j 入口, 跳过')
            continue
        shutil.copy2(path, f'{path}.bak.{ST}_newproject_j')
        t = open(path, encoding='utf-8').read().replace(a, a + '\n        ' + b, 1)
        open(path, 'w', encoding='utf-8').write(t)
        print(f'  ✓ {os.path.relpath(path, ROOT)} 加入入口 j (备份 .bak.{ST}_newproject_j)')
    d1 = build_one(S_SITL, D_SITL)
    d2 = build_one(S_FLY, D_FLY)
    shutil.copy2(W_SRC, W_DST)
    print(f'  ✓ worlds/j.world = i.world 副本')
    print(f'完成: 差异行 SITL={d1} fly={d2} (预期 ~22~26 行: 常量块 + 门 + 日志)')


if __name__ == '__main__':
    main()
