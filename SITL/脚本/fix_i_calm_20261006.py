#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 改进 A+B+D+G (2026-10-06, 基于 10.6 振荡复盘):
A 冻结估计用多帧中值 (0.5~1s, >=5 帧) —— 压掉单帧抖动(10.6 实测 ±10cm)
B 冻结采样加"平静门控": |roll|,|pitch| < 2°, |vxy| < 0.10 m/s (反投影含姿态项, roll 7° 就推十几厘米)
D 释放判据加"预测落点": |(hold-pos) - v*t_eff| < 6cm (用 EKF 量, 相机仍只读)
G 记录补齐: 采样帧数/冻结时 roll,pitch,H/delta/pred → 实飞后可算 落点-保持点
直接改 i.py 两份 (含备份), 不动 f/f_test/h/g/e。
"""
import os, re, sys, shutil, datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv

C_OLD = "I_ARRIVE_M       = 0.15    # 到位门限: |飞机位置 - 冻结保持点| (m)"
C_NEW = (C_OLD + "\n"
         "I_CALM_TILT      = 2.0     # A/B: 冻结采样允许的最大 |roll|/|pitch| (deg)\n"
         "I_CALM_VXY       = 0.10    # A/B: 冻结采样允许的最大水平速度 (m/s)\n"
         "I_CALM_SAMPLES   = 5       # A: 冻结前最少采集帧数 (取中值)\n"
         "I_CALM_TIMEOUT   = 3.0     # A/B: 采样等待上限 (s), 超时用已有样本冻结\n"
         "I_PRED_M         = 0.06    # D: 预测落点误差门限 (m)\n"
         "I_PRED_TEFF      = 0.70    # D: 指令->脱钩->落地 等效时间 (s)")

FRZ_OLD = """                if not getattr(self, '_i_frozen', False):
                    _est = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est = self._roi_to_ned(self.latest_roi)
                    _forced = (sim_t - (self._i_entry_t or sim_t)) > I_RELEASE_TIMEOUT
                    if _est is not None or _forced:
                        _dy = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
                        _dx = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
                        _ddx = (_est[0] - (self.x - self.home_x)) if _est is not None else 0.0
                        _ddy = (_est[1] - (self.y - self.home_y)) if _est is not None else 0.0"""

FRZ_NEW = """                if not getattr(self, '_i_frozen', False):
                    _est = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est = self._roi_to_ned(self.latest_roi)
                    # A/B(2026-10-06): 只在"平静"帧采样, 采够 I_CALM_SAMPLES 帧取中值后冻结
                    _calm = (abs(self._roll) < math.radians(I_CALM_TILT)
                             and abs(self._pitch) < math.radians(I_CALM_TILT)
                             and math.hypot(self.vx, self.vy) < I_CALM_VXY)
                    if _est is not None and _calm:
                        self._i_samp.append(_est)
                    _t_wait = sim_t - (self._i_entry_t or sim_t)
                    _forced = _t_wait > I_RELEASE_TIMEOUT
                    _enough = (len(self._i_samp) >= I_CALM_SAMPLES
                               or (len(self._i_samp) > 0 and _t_wait > I_CALM_TIMEOUT))
                    if _enough or _forced:
                        if self._i_samp:
                            _xs = sorted(s[0] for s in self._i_samp)
                            _ys = sorted(s[1] for s in self._i_samp)
                            _est = (_xs[len(_xs) // 2], _ys[len(_ys) // 2])
                        self._i_nsamp = len(self._i_samp)
                        self._i_frz_tilt = (self._roll, self._pitch)
                        self._i_frz_h = (self.home_z - self.z)
                        self._i_mode = "平静帧中值(校准%d帧)" % self._i_nsamp if self._i_samp else "兜底:无平静样本"
                        _dy = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
                        _dx = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
                        _ddx = (_est[0] - (self.x - self.home_x)) if _est is not None else 0.0
                        _ddy = (_est[1] - (self.y - self.home_y)) if _est is not None else 0.0"""

REL_OLD = """                    _deadline = _dwell >= I_RELEASE_TIMEOUT
                    if (_settled and _vxy < I_VMAX) or _deadline:"""
REL_NEW = """                    _deadline = _dwell >= I_RELEASE_TIMEOUT
                    # D(2026-10-06): 预测落点误差 (EKF 量, 相机不参与控制)
                    _pred = math.hypot((self._i_hold_x - (self.x - self.home_x)) - self.vx * I_PRED_TEFF,
                                       (self._i_hold_y - (self.y - self.home_y)) - self.vy * I_PRED_TEFF)
                    _pred_ok = _pred < I_PRED_M
                    if (_settled and _vxy < I_VMAX and _pred_ok) or _deadline:"""

REC_OLD = '"tier": "i",'
REC_NEW = ('"tier": "i", "mode": getattr(self, \'_i_mode\', \'i\'), '
           '"nsamp": getattr(self, \'_i_nsamp\', 0), '
           '"frz_tilt": getattr(self, \'_i_frz_tilt\', None), '
           '"frz_h": getattr(self, \'_i_frz_h\', None), '
           '"pred_m": _pred,')

LOG_OLD = """                            f"到位err={_err * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s \""""
LOG_NEW = """                            f"[{getattr(self, '_i_mode', 'i')}] 到位err={_err * 100:.1f}cm "
                            f"pred={_pred * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s \""""

SAMP_OLD = "                        self._i_arrive_t = None\n"
SAMP_NEW = ("                        self._i_arrive_t = None\n"
            "                        self._i_samp = []\n")

PAT = [(C_OLD, C_NEW), (FRZ_OLD, FRZ_NEW), (REL_OLD, REL_NEW), (REC_OLD, REC_NEW),
       (LOG_OLD, LOG_NEW), (SAMP_OLD, SAMP_NEW)]


def patch(t):
    for old, new in PAT:
        n = t.count(old)
        assert n == 1, '锚点命中 %d: %r' % (n, old[:60])
        t = t.replace(old, new, 1)
    for must in ('I_CALM_SAMPLES', '_i_samp', 'I_PRED_M', '_pred_ok', 'frz_tilt'):
        assert must in t, '缺少 %s' % must
    return t


if __name__ == '__main__':
    for p in ('src/drone_drl/drone_drl/i.py', 'fly/src/fly/fly/i.py'):
        t = open(p, encoding='utf-8').read()
        new = patch(t)
        if CHECK:
            print('OK(check)', p); continue
        shutil.copy2(p, p + '.bak.' + ST + '_calm')
        open(p, 'w', encoding='utf-8').write(new)
        print('patched', p)
