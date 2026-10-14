#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026-10-07 追加 16: 第 1 投冻结质量修复 (P1+P2+P3+P4+P5), 全部在 i.py 内.

复盘依据 (10.7/i_mission_20261007_130234.log, 第 1 投"偏了很多"):
  第 1 投 进 DROP 时飞机仍以 0.45m/s 平移 ⇒ 停稳门控(I_SETTLE_MAX=5.0)超时硬放行 ⇒
  3s 后(I_CALM_TIMEOUT)用 **1 帧** 冻结, 而那帧来自 **刚建 1.1s、尚未确认** 的追踪器
  (1.55s 后才 confirmed); 冻结值 vs 之后收敛值差 **58cm**(前20.8/左58.2), 而前几趟只差 2~8cm。
  直接病因: 检测框跑到画面边缘被 see.py **裁剪**(只裁不剔: x2=min(imw-1,..)) ⇒ 框中心偏向画面中心
  ⇒ 估计被拽走 (冻结帧 (163,292)w142 底边≈363>359 被裁; 随后 (556,62)w165 右边 638/640 被裁)。

改动 (5 条, 全部只影响"何时/用哪些帧冻结", 不动补偿/释放判据/投放通道):
  P1 冻结样本只认"已确认"的目标追踪器 (I_FREEZE_TRACKER_M=1.0)
  P2 采样兜底也要 ≥3 帧 (I_CALM_MIN_FALLBACK=3), 等待上限 3.0→6.0s
  P3 贴边(距边界<8px)的框不作冻结样本 (I_FREEZE_EDGE_MARGIN=8, 用任务自带 IMG_W/IMG_H)
  P4 停稳超时不再硬放行: 仍快于 I_SETTLE_HARD_VXY=0.25m/s 就继续等 (绝对上限 I_SETTLE_HARD_MAX=8.0s)
  P5 冻结前要求估计一致: ①优先"最近 5 帧互差 ≤10cm"取整窗中值; ②退化"最近 2 帧互差 ≤10cm"取两帧均值
     (13:02 回放: 漂移段整窗 70cm 不一致, 但收敛后最后两帧只差 9.5cm ⇒ 后者才可用)
  日志: 冻结行加"停稳x.xs 追踪器=确认/未确认", 冻结模式串加"散布xxcm", 新增节流 WARN"⚠ i模式暂不采样(...)"

安全边界: 20s 必投兜底 (`_forced`) 与 20s+5s 的 deadline 挑时机路径**完全没动** ⇒ "必投"保证不变。
用法: python3 scripts/patch_i_firstdrop_20261007.py <i.py> [<i.py> ...]
"""
import os
import re
import sys
import time
import shutil
import hashlib

STAMP = time.strftime("%Y%m%d_%H%M%S")
SUFFIX = ".bak.%s_firstdrop_fix" % STAMP


def rx(snippet):
    """把代码片段变成"忽略每行首尾空白与换行缩进"的宽松正则 (够特异, 又不怕空格差异)."""
    parts = [re.escape(l.strip()) for l in snippet.strip("\n").split("\n") if l.strip()]
    return re.compile(r"[ \t]*" + r"[ \t]*\n[ \t]*".join(parts))


# ---------- 常量块 ----------
P_CONST_CALM_OLD = """
I_CALM_TIMEOUT   = 3.0     # A/B: 采样等待上限 (s), 超时用已有样本冻结
"""
P_CONST_CALM_NEW = """
I_CALM_TIMEOUT   = 6.0     # 采样等待上限 (s): 3.0 会在飞机未停稳时用"单帧"冻结 (10-07 13:02 复盘)
I_CALM_MIN_FALLBACK = 3    # P2: 即使超时也至少要这么多帧 (1 帧横向噪声 ~±10cm 级)
I_FREEZE_TRACKER_M  = 1.0  # P1: 冻结样本只认"已确认"的目标追踪器 (到瞄准点须在此范围内)
I_FREEZE_MAX_SPREAD = 0.10 # P5: 冻结要求最近 I_CALM_SAMPLES 帧内 max-min ≤ 此值 (m); 在漂就继续采样
I_FREEZE_EDGE_MARGIN = 8   # P3: 冻结样本要求检测框距画面边界 ≥8px (see.py 对越界框只裁剪不剔除)
"""

P_CONST_SETTLE_OLD = """
I_SETTLE_MAX     = 5.0     # 先停稳: 最长等待 (s), 超时照常进入采样
"""
P_CONST_SETTLE_NEW = """
I_SETTLE_MAX     = 5.0     # 先停稳: 常规最长等待 (s)
I_SETTLE_HARD_VXY = 0.25   # P4: 超时后的硬门限 (m/s): 仍快于此值就继续等, 不在"还在漂"时冻结
I_SETTLE_HARD_MAX = 8.0    # P4: "等停稳"的绝对上限 (s): 超过就照常进入采样 (仍有 20s 必投兜底)
"""

# ---------- P4: 停稳门控 ----------
P4_OLD = """
                    _pre_ok = ((getattr(self, '_i_settle_t', None) is not None
                                and sim_t - self._i_settle_t >= I_SETTLE_HOLD)
                               or (sim_t - getattr(self, '_i_pre_start', sim_t) >= I_SETTLE_MAX))
"""
P4_NEW = """
                    # P4(2026-10-07 13:02 复盘): 超时不再"硬放行" —— 当时飞机还在 0.45m/s 平移就冻结,
                    #   之后还要移动 32cm 才到位, 释放相位 pred 被顶到 8.9cm。现在要求真正慢下来;
                    #   I_SETTLE_HARD_MAX 保证不会卡死 (之后仍有 20s 必投兜底)。
                    _slow = math.hypot(self.vx, self.vy) < I_SETTLE_HARD_VXY
                    _pre_wait = sim_t - getattr(self, '_i_pre_start', sim_t)
                    _pre_ok = ((getattr(self, '_i_settle_t', None) is not None
                                and sim_t - self._i_settle_t >= I_SETTLE_HOLD)
                               or (_pre_wait >= I_SETTLE_MAX and _slow)
                               or _pre_wait >= I_SETTLE_HARD_MAX)
"""

# ---------- P1/P2/P3/P5: 采样 + 冻结决策 ----------
P1235_OLD = """
                    _est = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est = self._roi_to_ned(self.latest_roi)
                    # A/B(2026-10-06): 只在"平静"帧采样, 采够 I_CALM_SAMPLES 帧取中值后冻结
                    _calm = (abs(self._roll) < math.radians(I_CALM_TILT)
                             and abs(self._pitch) < math.radians(I_CALM_TILT)
                             and math.hypot(self.vx, self.vy) < I_CALM_VXY)
                    if not hasattr(self, '_i_samp'):
                        self._i_samp = []
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
"""
P1235_NEW = """
                    _est = None
                    if self._drop_detection_fresh() and self.latest_roi is not None:
                        _est = self._roi_to_ned(self.latest_roi)
                    # A/B(2026-10-06): 只在"平静"帧采样, 采够 I_CALM_SAMPLES 帧取中值后冻结
                    _calm = (abs(self._roll) < math.radians(I_CALM_TILT)
                             and abs(self._pitch) < math.radians(I_CALM_TILT)
                             and math.hypot(self.vx, self.vy) < I_CALM_VXY)
                    # P1/P3(2026-10-07 13:02 复盘): 样本还须来自"已确认的目标追踪器", 且检测框
                    #   不能贴画面边界 (see.py 对越界框只裁剪 ⇒ 框中点偏移可达几十 cm)。
                    _trk_ok = self._i_target_tracker_ok()
                    _edge_ok = self._i_roi_inside(self.latest_roi)
                    if not hasattr(self, '_i_samp'):
                        self._i_samp = []
                    _rej = (("追踪器未确认" if not _trk_ok else "")
                            + ("/框贴边" if not _edge_ok else "")
                            + ("/非平静" if not _calm else ""))
                    if _est is not None and _calm and _trk_ok and _edge_ok:
                        self._i_samp.append(_est)
                    _t_wait = sim_t - (self._i_entry_t or sim_t)
                    _forced = _t_wait > I_RELEASE_TIMEOUT
                    # P5 两级判据 (13:02 回放: 整窗 70cm 不一致, 但收敛后最后两帧只差 9.5cm):
                    #   ① 优先: 最近 I_CALM_SAMPLES 帧互差 ≤ I_FREEZE_MAX_SPREAD ⇒ 用整窗中值;
                    #   ② 退化: 最近 2 帧互差 ≤ 同门限 (且总样本 ≥ I_CALM_MIN_FALLBACK) ⇒ 用这两帧均值
                    #      (估计在漂时, 越靠近释放时刻的值越可信; 绝不跨整段漂移取中值)
                    _samp_use = self._i_samp_recent()
                    _spread = self._i_samp_spread()
                    _last2 = self._i_samp[-2:] if len(self._i_samp) >= 2 else []
                    _agree2 = (len(_last2) == 2
                               and math.hypot(_last2[0][0] - _last2[1][0],
                                              _last2[0][1] - _last2[1][1]) <= I_FREEZE_MAX_SPREAD)
                    _stable = (len(_samp_use) >= I_CALM_SAMPLES and _spread <= I_FREEZE_MAX_SPREAD)
                    _enough = (_stable
                               or (_agree2 and len(self._i_samp) >= I_CALM_MIN_FALLBACK
                                   and _t_wait > I_CALM_TIMEOUT))
                    if (_est is not None and _rej
                            and sim_t - getattr(self, '_i_rej_log_t', -99.0) > 1.0):
                        self._i_rej_log_t = sim_t
                        self.get_logger().warn(
                            f"⚠ i模式暂不采样 ({_rej.lstrip('/')}) 帧数 {len(_samp_use)} "
                            f"窗口散布 {_spread * 100:.0f}cm "
                            f"追踪器={'确认' if _trk_ok else '未确认'}")
                    if _enough or _forced:
                        _use = _samp_use if _stable else (_last2 if _last2 else _samp_use)
                        if len(_use) == 2:      # 退化路径: 两帧取均值 (中值取"较大者"会引入偏置)
                            _est = (0.5 * (_use[0][0] + _use[1][0]), 0.5 * (_use[0][1] + _use[1][1]))
                        elif _use:
                            _xs = sorted(s[0] for s in _use)
                            _ys = sorted(s[1] for s in _use)
                            _est = (_xs[len(_xs) // 2], _ys[len(_ys) // 2])
                        self._i_nsamp = len(_use)
                        self._i_nspread = _spread
"""

# ---------- 冻结模式串 (加散布) ----------
MODE_OLD = """
                        self._i_mode = "平静帧中值(校准%d帧)" % self._i_nsamp if self._i_samp else "兜底:无平静样本"
"""
MODE_NEW = """
                        _sp_txt = ("%.0fcm" % (self._i_nspread * 100)
                                   if (self._i_nspread is not None and math.isfinite(self._i_nspread))
                                   else "n/a")
                        self._i_mode = (("平静帧中值(校准%d帧,散布%s)" % (self._i_nsamp, _sp_txt))
                                        if self._i_samp else "兜底:无平静样本")
"""

# ---------- 冻结日志追加停稳/追踪器信息 ----------
LOG_OLD = """
                            f"补偿B{self._drop_count + 1}=({_dy:+.3f},{_dx:+.3f}){' [兜底:无检测]' if _est is None else ''}")
"""
LOG_NEW = """
                            f"补偿B{self._drop_count + 1}=({_dy:+.3f},{_dx:+.3f})"
                            f"{' [兜底:无检测]' if _est is None else ''} "
                            f"[停稳{_pre_wait:.1f}s 追踪器={'确认' if _trk_ok else '未确认'}]")
"""

# ---------- 新增 4 个辅助函数 ----------
HELPERS_OLD = """
    def _drop_detection_fresh(self):
"""
HELPERS_NEW = '''
    def _i_target_tracker_ok(self):
        """P1: 冻结样本只认"已确认"的目标追踪器 (到瞄准点须在 I_FREEZE_TRACKER_M 内).

        10-07 13:02 第 1 投: 冻结用的那帧来自刚建 1.1s、未确认的追踪器 (1.55s 后才 confirmed)。
        """
        if self._current_target_ned is None:
            return False
        _best, _bd = None, 1e9
        for t in self._trackers:
            d = math.hypot(t["ned"][0] - self._current_target_ned[0],
                           t["ned"][1] - self._current_target_ned[1])
            if d < _bd:
                _bd, _best = d, t
        return (_best is not None and _bd <= I_FREEZE_TRACKER_M
                and bool(_best.get("confirmed")))

    def _i_roi_inside(self, roi):
        """P3: 检测框必须整框在画面内 (see.py 对越界框只裁剪 ⇒ 被裁框中心偏向画面中心)."""
        if roi is None:
            return False
        m = I_FREEZE_EDGE_MARGIN
        return (roi.x_offset >= m and roi.y_offset >= m
                and roi.x_offset + roi.width <= IMG_W - m
                and roi.y_offset + roi.height <= IMG_H - m)

    def _i_samp_recent(self):
        """P5: 只用最近 I_CALM_SAMPLES 个样本 (估计在漂时, 中值不应跨整段漂移取)."""
        n = max(1, I_CALM_SAMPLES)
        return list(self._i_samp[-n:]) if len(self._i_samp) > n else list(self._i_samp)

    def _i_samp_spread(self):
        """P5: 采样窗口内最大两两间距 (m); 样本 <2 视为 inf (不可用)."""
        s = self._i_samp_recent()
        if len(s) < 2:
            return float('inf')
        return max(math.hypot(p[0] - q[0], p[1] - q[1]) for p in s for q in s)

    def _drop_detection_fresh(self):
'''

PATCHES = [(P_CONST_CALM_OLD, P_CONST_CALM_NEW, "常量: 采样/冻结门限"),
           (P_CONST_SETTLE_OLD, P_CONST_SETTLE_NEW, "常量: 停稳硬门限"),
           (P4_OLD, P4_NEW, "P4 停稳超时不硬放行"),
           (P1235_OLD, P1235_NEW, "P1/P2/P3/P5 采样与冻结决策"),
           (MODE_OLD, MODE_NEW, "冻结模式串加散布"),
           (LOG_OLD, LOG_NEW, "冻结日志加停稳/追踪器"),
           (HELPERS_OLD, HELPERS_NEW, "新增 4 个辅助函数")]

FROZEN = {"PAYLOAD_OFFSET_BODY_Y_B1": "+0.18", "PAYLOAD_OFFSET_BODY_Y_B2": "0.00",
          "PAYLOAD_OFFSET_BODY_XF_B1": "0.00", "PAYLOAD_OFFSET_BODY_XF_B2": "0.00",
          "I_CALM_TILT": "5.0", "I_CALM_SAMPLES": "5", "I_CALM_VXY": "0.15",
          "I_PRED_M": "0.09", "I_RELEASE_TIMEOUT": "20.0", "I_SETTLE_HOLD": "1.0"}
EXPECT = {"I_CALM_TIMEOUT": "6.0", "I_CALM_MIN_FALLBACK": "3", "I_FREEZE_TRACKER_M": "1.0",
          "I_FREEZE_MAX_SPREAD": "0.10", "I_FREEZE_EDGE_MARGIN": "8",
          "I_SETTLE_HARD_VXY": "0.25", "I_SETTLE_HARD_MAX": "8.0", "I_SETTLE_MAX": "5.0"}
MARKERS = ["def _i_target_tracker_ok", "def _i_roi_inside", "def _i_samp_recent",
           "def _i_samp_spread", "_trk_ok = self._i_target_tracker_ok()",
           "_edge_ok = self._i_roi_inside(self.latest_roi)", "_slow = math.hypot(self.vx, self.vy) < I_SETTLE_HARD_VXY"]


def sha(p):
    with open(p, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def const_of(text, name):
    m = re.search(r"^%s\s*=\s*([+-]?[0-9.]+)" % re.escape(name), text, re.M)
    return m.group(1) if m else None


def main(argv):
    targets = [p for p in argv[1:] if p.endswith(".py")]
    if not targets:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        targets = [os.path.join(root, "src/drone_drl/drone_drl/i.py"),
                   os.path.join(root, "fly/src/fly/fly/i.py")]
    # ---------- 阶段 1: 全部文件、全部锚点先校验 ----------
    print("阶段 1: 校验锚点")
    texts, pats = {}, {}
    for t in targets:
        assert os.path.isfile(t), "找不到 %s" % t
        texts[t] = open(t, encoding="utf-8").read()
        print("  %s (sha %s)" % (t, sha(t)))
        for k, v in FROZEN.items():
            assert const_of(texts[t], k) == v, "%s 应为 %s, 实为 %s —— 中止" % (k, v, const_of(texts[t], k))
        for old, new, desc in PATCHES:
            pats[(t, desc)] = rx(old)
            n = len(pats[(t, desc)].findall(texts[t]))
            assert n == 1, "锚点[%s] 在 %s 命中 %d 次 (应为 1) —— 中止" % (desc, t, n)
            print("    ✓ 锚点唯一: %s" % desc)
    # ---------- 阶段 2: 统一应用 ----------
    print("\n阶段 2: 应用补丁")
    for t in targets:
        src = texts[t]
        for old, new, desc in PATCHES:
            src = pats[(t, desc)].sub(lambda m: new.strip("\n").replace("\n", "\n" + " " * 0), src, count=1)
        for k, v in EXPECT.items():
            assert const_of(src, k) == v, "补丁后 %s = %s, 期望 %s —— 中止" % (k, const_of(src, k), v)
        for mk in MARKERS:
            assert mk in src, "缺少标记 %s —— 中止" % mk
        for k, v in FROZEN.items():
            assert const_of(src, k) == v, "补丁后冻结项 %s 被改动 —— 中止" % k
        compile(src, t, "exec")
        bak = t + SUFFIX
        shutil.copy2(t, bak)
        with open(t, "w", encoding="utf-8") as f:
            f.write(src)
        os.utime(t, None)
        print("  %s\n    备份 %s\n    新 sha %s" % (t, os.path.basename(bak), sha(t)))
    print("\n全部完成, 备份后缀 %s" % SUFFIX)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
