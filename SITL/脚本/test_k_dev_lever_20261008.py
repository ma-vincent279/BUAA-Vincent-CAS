#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""项目 k 离线验收 (2026-10-08, 精度调参台)

k 的定位 = **i 的高精度调参台**: 在 k 上反复调、调到可复现, 再一行级回灌 i。
因此本测试的判据是"**投放链与 i 逐位一致 + 测试台改动只在允许集合内 + 新杠杆默认关**":

  A. 与 i 的投放链一致性 (机器核对, 不靠人眼):
     - 投放关键常量在 i.py / k.py 中逐位相同 (排除 3 个**有意**不同的: 前进距离/停留时长/兜底时长)
     - 冻结公式 (保持点 = 桶估计 + 两瓶补偿) 逐字相同
     - 释放条件 = i 的条件 ∧ `_dev_ok` ∧ `_body_ok`, 且两门默认关 ⇒ 恒真 ⇒ 等价于 i
     - 2026-10-09: k 就地打了 P1/P2 并写入静态标定常量 ⇒ 4 个 Y/XF 常量改走标定账本
       (其余投放常量仍与 i 逐位相同); 差异块允许集合同步追加 P1/P2 + 标定标记
     - 与 j 的差异只在 k 补丁集合内 (dwell/_pick/lever/日志)
  B. 测试台补丁: 单桶×2 / 一律返航 (0 处 STEP_SCOUT) / SUMMARY 分母 = 2投 / 停留 WARN 用真实时长
  C. 精度杠杆 (dev 超限重新冻结): 抽出 k.py 真实代码块 exec 验证行为
     - 关闭 (0.00) ⇒ 状态一点不变 (与 i 一致)  - 打开 ⇒ 保持点 = 当前帧 est + 补偿, 停留计时重置
     - 不重置 _i_freeze_t (必投兜底仍从首次冻结起算)  - dev 未超限/达次数上限/dev=None ⇒ 不重冻
"""
import difflib
import math
import re
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PASS = FAIL = 0


def ck(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f'  ✓ {msg}')
    else:
        FAIL += 1
        print(f'  ✗ {msg}')


def norm(t, word):
    """剥注释 + 把项目前缀归一, 便于跨文件比对"""
    t = '\n'.join(l for l in t.split('\n') if not l.lstrip().startswith('#'))
    return t.replace(f'{word}模式', 'X模式').replace(f'{word}_mission', 'x_mission')


# k 相对 j 允许的改动 (生成脚本 new_project_k_20261008.py 的补丁集合)
ALLOW_J_TO_K = [
    r'FORWARD_DIST\s*=\s*3\.0', r'I_RELEASE_TIMEOUT\s*=\s*45\.0', r'K_DWELL_S1', r'K_DWELL_S2',
    r'_dwell_need', r'SERVED_EXCLUDE_M\s*=\s*0\.00', r'SERVED_DUP_M\s*=\s*0\.00',
    r'len\(confirmed\) < 1', r'len\(picked\) < 1', r'_target_buckets\[0\]\] \* 2',
    r'_target_ids\[0\]\] \* 2', r'_target_count = len\(self\._target_buckets\)',
    r'两瓶打同一个桶', r'STEP_RETURN', r'不飞侦察区', r'_pick', r'_dev_ok', r'_estl',
    r'_k_frz_n', r'_dy2', r'_dx2', r'_i_est_f = \(_estl', r'_i_hold_[xy] = \(_estl',
    r'_i_arrive_t = None', r'J_DEV_REFREEZE', r'精度杠杆', r'重新冻结',
    r'^\+ *return$', r'^\+ *$', r'class KMission', r'node = KMission', r'MISSION_REVISION',
    r'if self\._target_buckets:', r'and self\._drop_detection_fresh\(\) \\',
    r'else PAYLOAD_OFFSET_BODY_[YX]F?_B2\)', r'self\.get_logger\(\)\.warn\($',
]

# 2026-10-09 追加 (与 test_l_target_plane_20261009.py 的 ALLOWED 同源):
#   P1/P2 就地补丁 + 静态标定常量 (只改值)。块级匹配对续行鲁棒。
# 2026-10-09: 后续就地补丁的标记集中登记 (见 scripts/_later_patch_marks.py)
try:
    from _later_patch_marks import LATER_PATCH_MARKERS_RE as _SHARED, \
        LATER_PATCH_BAK_SUFFIXES as _SHARED_BAK
except Exception:      # 单独拷出去跑时退化为空表
    _SHARED, _SHARED_BAK = (), ()   # 本脚本把标记当**正则**用 ⇒ 用转义版

ALLOW_20261009 = list(_SHARED) + [
    r'TARGET_PLANE_H', r'L_BODY_', r'_l_body_resid', r'_l_refreeze', r'_l_body_raw',
    r'l/P1', r'l/P2', r'P1 TARGET_PLANE_H', r'P2 L_BODY_GATE_M', r'l 补丁集',
    r'_bres', r'机体残差',
    # 2026-10-09 首飞反馈整改 (scripts/patch_k_flat_20261009.py, 仅 k)
    r'K_FLIGHT_ALT', r'K_FLIGHT_SPEED', r'K_VISUAL_GATE_M', r'PRE_SERVO_HEIGHT',
    r'CRUISE_HEIGHT = 5\.0', r'_travel_fwd', r'视觉门', r'全程平飞',
    # 2026-10-09 OFFBOARD 丢失识别 (切手操/模式切换)
    r'_ob_lost_logged', r'_active_logged', r'offboard_lost_external', r'切手操', r'ARM/OFFBOARD 丢失',
    r'基底 = k\.py', r'搬运检查', r'PAYLOAD_OFFSET_BODY_[YX]F?_B[12]\s*=\s*[-+]?[\d.]+',
    r'HFOV\s*=\s*math\.radians\([\d.]+\)', r'VFOV\s*=\s*math\.radians\([\d.]+\)',
    r'TARGET_PLANE_H\s*=', r'SITL',
    # 2026-10-09 FC 模式 + EKF 健康度观测字段 (scripts/patch_ekf_tlm_20261009.py):
    # 只加观测 —— __init__ 两字段 / _cb_local 快照 / 1Hz 行尾追加 FC(..) EKF(..) / 垂向失效 WARN
    r'self\._ekf', r'_ekf_warn_t', r'_st = self\.status', r'fc_str', r'ek_str',
    # 2026-10-09 前飞无进展保护 + landed 记录 (scripts/patch_k_stall_guard_20261009.py)
    r'K_FWD_STALL', r'_fwd_stall_logged', r'forward_stall', r'无进展',
    # 2026-10-09 5m 巡航剖面 (scripts/patch_k_alt_5m_20261009.py)
    r'K_DESCEND_RATE', r'_z_desc', r'_pre_z', r'降高', r'K_FLIGHT_ALT  = 5\.0', r'PRE_SERVO_HEIGHT = DROP_HEIGHT',
]

# 2026-10-09 静态标定账本 (见 docs/reports/calibration_to_code_20261009.md)
CALIB_2026_1009 = {
    'PAYLOAD_OFFSET_BODY_Y_B1': 0.146, 'PAYLOAD_OFFSET_BODY_Y_B2': 0.036,
    'PAYLOAD_OFFSET_BODY_XF_B1': -0.021, 'PAYLOAD_OFFSET_BODY_XF_B2': -0.021,
}

KS = [ROOT / 'src/drone_drl/drone_drl/k.py', ROOT / 'fly/src/fly/fly/k.py']
IS = [ROOT / 'src/drone_drl/drone_drl/i.py', ROOT / 'fly/src/fly/fly/i.py']
JS = [ROOT / 'src/drone_drl/drone_drl/j.py', ROOT / 'fly/src/fly/fly/j.py']
KT = {p: norm(p.read_text(encoding='utf-8'), 'k') for p in KS}
IT = {p: norm(p.read_text(encoding='utf-8'), 'i') for p in IS}
JT = {p: norm(p.read_text(encoding='utf-8'), 'j') for p in JS}

# ============ A. 与 i 的投放链一致性 ============
print('=== A. 投放链与 i 逐位一致 (k 的立足点) ===')
# 有意不同的 3 项 (其余必须一致)
DROP_CONSTS = [
    'PAYLOAD_OFFSET_BODY_Y_B1', 'PAYLOAD_OFFSET_BODY_Y_B2',
    'PAYLOAD_OFFSET_BODY_XF_B1', 'PAYLOAD_OFFSET_BODY_XF_B2',
    'I_CALM_SAMPLES', 'I_CALM_TILT', 'I_CALM_VXY', 'I_CALM_TIMEOUT', 'I_CALM_MIN_FALLBACK',
    'I_FREEZE_TRACKER_M', 'I_FREEZE_EDGE_MARGIN', 'I_FREEZE_MAX_SPREAD',
    'I_SETTLE_VXY', 'I_SETTLE_HOLD', 'I_SETTLE_MAX', 'I_SETTLE_HARD_VXY', 'I_SETTLE_HARD_MAX',
    'I_ARRIVE_M', 'I_PRED_M', 'I_PRED_TEFF', 'I_VMAX', 'I_DEADLINE_WIN', 'DROP_HEIGHT',
    'I_LOW_ALT_FREEZE', 'FX', 'FY',
]


def consts(txt, names):
    out = {}
    for n in names:
        m = re.search(rf'^{n}\s*=\s*([^\n#]+)', txt, re.M)
        if m:
            out[n] = m.group(1).strip()
    return out


for kp, ip in zip(KS, IS):
    n = kp.name
    ci, ck_ = consts(IT[ip], DROP_CONSTS), consts(KT[kp], DROP_CONSTS)
    common = sorted(set(ci) & set(ck_))
    rest = [x for x in common if x not in CALIB_2026_1009]
    diff = [x for x in rest if ci[x] != ck_[x]]
    ck(len(common) >= 20, f'{n}: 可比对的投放常量 {len(common)} 个 (≥20)')
    ck(not diff, f'{n}: 除 2026-10-09 标定常量外, 其余 {len(rest)} 个投放常量与 i 逐位相同 '
                  f'(不同项: {diff or "无"})')
    off = [f'{x}: {ck_.get(x)} ≠ {v}' for x, v in CALIB_2026_1009.items()
           if x not in ck_ or abs(float(ck_[x]) - v) > 1e-9]
    ck(not off, f'{n}: 4 个标定常量 = 2026-10-09 账本 ({off or "全部一致"})')
    # 冻结公式 (保持点 = 桶估计 + 补偿) 逐字相同
    fz = ('self._i_hold_x = (_hx0\n'
          '                                          + _dy * math.sin(self.heading) - _dx * math.cos(self.heading))\n'
          '                        self._i_hold_y = (_hy0\n'
          '                                          - _dy * math.cos(self.heading) - _dx * math.sin(self.heading))')
    ck(fz in IT[ip], f'{n}: i 的冻结公式存在')
    ck(fz in KT[kp], f'{n}: k 的冻结公式与 i 逐字相同 (投放几何未动)')
    # 释放条件 = i 的条件 ∧ _dev_ok
    rel_i = 'if (_settled and _vxy < I_VMAX and _pred_ok) or _dl_pick:'
    rel_k = ('if (_settled and _vxy < I_VMAX and _pred_ok and _dev_ok and _body_ok) '
             'or _dl_pick:')
    ck(rel_i in IT[ip] and rel_k in KT[kp],
       f'{n}: 释放条件 = i 的条件 ∧ _dev_ok ∧ _body_ok (P2 门也默认关 ⇒ 恒真)')
    ck('_dev_ok = (J_DEV_GATE_M <= 0.0) or (_dev_m is None) or (_dev_m <= J_DEV_GATE_M)' in KT[kp],
       f'{n}: 门默认关 + 失效开放 ⇒ _dev_ok 恒真 ⇒ 好趟与 i 等价')
    ck('J_DEV_GATE_M   = 0.00' in KT[kp], f'{n}: J_DEV_GATE_M 默认 0.00')
    # 有意不同的 3 项
    ck('FORWARD_DIST  = 3.0' in KT[kp], f'{n}: 有意差异① 前进 3m (i 是 34m)')
    ck('I_RELEASE_TIMEOUT = 45.0' in KT[kp], f'{n}: 有意差异② 必投兜底 45s (i 是 20s)')
    ck('K_DWELL_S1      = 5.0' in KT[kp] and 'K_DWELL_S2      = 30.0' in KT[kp],
       f'{n}: 有意差异③ 两投停留 5s/30s (i 是固定 5s)')
    # 与 j 的差异 = 只允许生成脚本 new_project_k_20261008.py 里的补丁集合 (最小改动审计)
    a_, b_ = JT[JS[KS.index(kp)]].split('\n'), KT[kp].split('\n')
    sm = difflib.SequenceMatcher(None, a_, b_, autojunk=False)
    blocks = [b_[j1:j2] for tag, i1, i2, j1, j2 in sm.get_opcodes() if tag != 'equal']
    added = sum(1 for blk in blocks for l in blk if l.strip())
    bad = [blk for blk in blocks
           if not any(re.search(p, '\n'.join(blk)) for p in ALLOW_J_TO_K + ALLOW_20261009)]
    ck(not bad, f'{n}: 相对 j 的 {len(blocks)} 处差异块 ({added} 行) 全部在允许集合内 '
                f'(越界 {len(bad)} 块: {[x[0].strip()[:40] for x in bad]})')

# ============ B. 测试台补丁 ============
print('=== B. 测试台补丁 ===')
for kp in KS:
    t = KT[kp]
    n = kp.name
    ck('class KMission(Node):' in t and 'node = KMission()' in t, f'{n}: 类名/入口 KMission')
    ck('i_mission' not in t and 'IMission' not in t and 'j_mission' not in t and 'JMission' not in t,
       f'{n}: 无 i/j 残留命名 (SITL 崩过 NameError 的坑)')
    ck(t.count('_enter_step(STEP_SCOUT)') == 0, f'{n}: 0 处去侦察区 (投完/搜索超时/重合放弃 全部直接返航)')
    ck(t.count('[self._target_buckets[0]] * 2') == 1, f'{n}: 第 2 个目标复制成第 1 个 (两瓶打同一个桶)')
    ck('self._target_count = len(self._target_buckets)' in t, f'{n}: SUMMARY 分母 = 计划投 2 次 (原来显示 2/1)')
    i_dw = t.index('_dwell_need = (K_DWELL_S1 if self._drop_count == 0 else K_DWELL_S2)')
    i_warn = t.index('→ 开始停留 {_dwell_need:.0f}s')
    ck(i_dw < i_warn, f'{n}: _dwell_need 定义在 WARN 之前 ⇒ WARN 能报真实时长 (原来硬编码 5s)')
    ck('开始停留 {I_DWELL_S' not in t, f'{n}: WARN 不再用 I_DWELL_S')
    ck('and (sim_t - self._i_arrive_t) >= _dwell_need)' in t, f'{n}: 停留判据用逐投时长')
    ck(t.count('SERVED_EXCLUDE_M      = 0.00') == 1 and t.count('SERVED_DUP_M      = 0.00') == 1,
       f'{n}: SERVED 门归零 (允许再投同一个桶)')

# ============ C. 精度杠杆 (真实代码块 exec) ============
print('=== C. 精度杠杆 (dev 超限重新冻结) ===')


class _Log:
    def __init__(self):
        self.msgs = []

    def warn(self, m):
        self.msgs.append(m)

    def info(self, m):
        self.msgs.append(m)


class _Self:
    def __init__(self, heading, drop_count=0):
        self.heading = heading
        self._drop_count = drop_count
        self.home_x = 0.0
        self.home_y = 0.0
        self.x = 0.0
        self.y = 0.0
        self._i_hold_x = 9.9
        self._i_hold_y = 9.9
        self._i_est_f = (0.0, 0.0)
        self._i_arrive_t = 100.0
        self._i_freeze_t = 50.0
        self._k_frz_n = 0
        self.log = _Log()

    def get_logger(self):
        return self.log


def load_lever(kp):
    """从 k.py 抽出真实的重冻代码块, 包成函数 exec (不复制逻辑, 测的就是即将上机的代码)"""
    t = kp.read_text(encoding='utf-8')
    m = re.search(r'( *)# ===== k\(2026-10-08\): 精度杠杆.*?\n(?:.*?\n)*? *return\n', t)
    assert m, '没找到重冻代码块'
    body = textwrap.indent(textwrap.dedent(m.group(0)), '    ')
    ns = {'math': math, 'PAYLOAD_OFFSET_BODY_Y_B1': 0.18, 'PAYLOAD_OFFSET_BODY_Y_B2': 0.00,
          'PAYLOAD_OFFSET_BODY_XF_B1': 0.05, 'PAYLOAD_OFFSET_BODY_XF_B2': 0.05,
          'J_DEV_REFREEZE_M': 0.00, 'J_DEV_REFREEZE_MAX': 3}
    exec('def _lever(self, sim_t, _dev_m, _estl):\n' + body + '    return False\n', ns)
    return ns


def run_lever(kp, gate, dev, est, maxn=3, heading=0.0, drop_count=0, freeze_t=50.0):
    ns = load_lever(kp)
    ns['J_DEV_REFREEZE_M'], ns['J_DEV_REFREEZE_MAX'] = gate, maxn
    s = _Self(heading, drop_count)
    s._k_frz_n = 0
    s._i_freeze_t = freeze_t
    ret = ns['_lever'](s, 999.0, dev, est)
    return s, ret


kp = KS[1]
ns0 = load_lever(kp)
ck('J_DEV_REFREEZE_M   = 0.00' in KT[kp], '默认 0.00 = 关闭 (与 i 行为一致)')
ck('J_DEV_REFREEZE_MAX = 3' in KT[kp], '次数上限 3 (硬兜底 I_RELEASE_TIMEOUT 仍保证必投)')
# 关闭 ⇒ 状态一点不变
s, ret = run_lever(kp, 0.0, 0.25, (0.30, 0.30))
ck(ret is False and (s._i_hold_x, s._i_hold_y) == (9.9, 9.9) and s._i_est_f == (0.0, 0.0)
   and s._i_arrive_t == 100.0 and s._k_frz_n == 0 and not s.log.msgs,
   '关闭: dev=25cm 也完全不动 (hold/est/arrive_t/计数/日志 全不变 ⇒ 与 i 逐位一致)')
# 打开 + 超限 ⇒ 重冻
for hdg, tag in ((0.0, 'heading=0'), (math.radians(212.0), 'heading=212°')):
    s, ret = run_lever(kp, 0.10, 0.25, (0.30, -0.20), heading=hdg)
    exp_x = 0.30 + 0.18 * math.sin(hdg) - 0.05 * math.cos(hdg)
    exp_y = -0.20 - 0.18 * math.cos(hdg) - 0.05 * math.sin(hdg)
    ck(ret is None or ret is False, f'打开+超限 ({tag}): 本帧不放行 (避免用旧 _settled 立刻释放)')
    ck(abs(s._i_hold_x - exp_x) < 1e-12 and abs(s._i_hold_y - exp_y) < 1e-12,
       f'打开+超限 ({tag}): 保持点 = 当前帧 est + B1 补偿(0.18/0.05) (与 i 冻结公式同一算式)')
    ck(s._i_est_f == (0.30, -0.20), f'打开+超限 ({tag}): dev 基准 est 一并更新')
    ck(s._i_arrive_t is None, f'打开+超限 ({tag}): 停留计时重置 (重新走 到位→停留)')
    ck(s._i_freeze_t == 50.0, f'打开+超限 ({tag}): **不重置 _i_freeze_t** ⇒ 必投兜底仍从首次冻结起算')
    ck(s._k_frz_n == 1 and '重新冻结 #1' in (s.log.msgs[0] if s.log.msgs else ''),
       f'打开+超限 ({tag}): 计数 +1 且打 WARN (含 dev 与新的保持点)')
# 第 2 投用 B2 补偿
s, _ = run_lever(kp, 0.10, 0.25, (0.30, -0.20), drop_count=1)
ck(abs(s._i_hold_x - (0.30 - 0.05)) < 1e-12 and abs(s._i_hold_y - (-0.20)) < 1e-12,
   '第 2 投: 用 B2 补偿 (0.00/0.05), 与首次冻结的选瓶逻辑一致')
# 边界
s, ret = run_lever(kp, 0.10, 0.10, (0.30, 0.0))
ck(ret is False and s._k_frz_n == 0, 'dev 恰等于门限 (10cm): 不重冻 (严格 >)')
s, ret = run_lever(kp, 0.10, 0.25, None)
ck(ret is False and s._k_frz_n == 0, 'dev=None (无相机值): 不重冻 (失效开放)')
s, ret = run_lever(kp, 0.10, 0.25, (0.30, 0.0), maxn=0)
ck(ret is False and s._k_frz_n == 0, '次数上限 0: 不重冻 (上限可当总开关)')
s = _Self(0.0)
s._k_frz_n = 3
ns = load_lever(kp)
ns['J_DEV_REFREEZE_M'] = 0.10
ret = ns['_lever'](s, 999.0, 0.25, (0.30, 0.0))
ck(ret is False and s._k_frz_n == 3, '已达上限 3 次: 不再重冻 (防无限重对准)')

print(f'\n=== 结果: {PASS} 通过 / {FAIL} 失败 ===')
sys.exit(1 if FAIL else 0)
