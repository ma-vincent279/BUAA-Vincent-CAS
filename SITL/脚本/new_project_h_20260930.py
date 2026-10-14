#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
新建子项目 h = 项目 g 的完整副本 + 9-30 实飞复盘后的全部投放代码优化 (2026-09-30)
===============================================================
用户指示 (2026-09-30): 全部改动应用到新项目 h, e/f/g 一行不动, 不做逐项验证。

改动清单 (全部集中在"何时投 / 伺服行为 / 记录", 不碰释放通道/补偿数值/飞控参数):
  ① 预测落点释放      pred_m = |(瞄准点-飞机) - v*t_eff| < 3cm 确认 0.2s  (Tier0 优选)
  ② 记录字段补齐      pred/t_eff/vxy/H/tier/reappr + 释放前 4s 快照
  ③ 瞄准点平滑+限速   SERVO_EST_WEIGHT 0.7->0.5, 一阶低通 τ=0.30s, 限速 10cm/s
  ④ 到位即静          残差<3cm 持续 0.5s -> 冻结像素残差积分
  ⑤ 现场质量提示      Tier2/兜底/检测陈旧释放时打 WARN 并打印 pred/conv/vxy
  ⑥ 保底档改"重进"    优先窗口内未触发 -> 后退1m+抬升0.8m 重进, 最多 2 次后才允许 Tier2
  ⑦ 提前对准          下降到 drop_z+0.4m 以下前, 残差>8cm 则保持高度对准 (超时 5s 放行)
  ⑧ 判据参数          DROP_CONVERGE_EARLY 0.5->0.25 / _LATE 1.0->0.4; 低谷最小值改 2s 滑窗
  ⑨ 软门控            释放时要求 H>=DROP_HEIGHT-0.15m 且 |vz|<0.15m/s

生成方式: 逐字节复制 g.py (SITL + fly 两份), 只替换 节点名/类名/入口名/MISSION_REVISION
          + 上述锚点补丁 (每处断言唯一命中), 自动备份被改文件。
用法: python3 scripts/new_project_h_20260930.py [--check]
"""
import os, re, sys, shutil, hashlib, datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = "--check" in sys.argv

SITL_SRC = os.path.join(ROOT, "src/drone_drl/drone_drl/g.py")
SITL_DST = os.path.join(ROOT, "src/drone_drl/drone_drl/h.py")
FLY_SRC = os.path.join(ROOT, "fly/src/fly/fly/g.py")
FLY_DST = os.path.join(ROOT, "fly/src/fly/fly/h.py")

REPORT = []


def md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def reindent(new, base_src, base_dst):
    """把 new (以 base_src 为基准缩进写出) 调整到 base_dst 基准缩进"""
    d = len(base_dst) - len(base_src)
    if d == 0:
        return new
    out = []
    for ln in new.split("\n"):
        if not ln.strip():
            out.append(ln)
        elif d > 0:
            out.append(" " * d + ln)
        else:
            out.append(ln[-d:] if len(ln) >= -d else ln.lstrip())
    return "\n".join(out)


def patch(text, name, anchor, new, base_indent=12, expect=1, is_re=False):
    """锚点替换 (先精确匹配, 失败则按缩进灵活匹配); 断言命中次数"""
    if is_re:
        hits = list(re.finditer(anchor, text))
        if len(hits) != expect:
            raise AssertionError(f"[{name}] 正则命中 {len(hits)} 次 (期望 {expect})")
        text = re.sub(anchor, new, text, count=expect)
        REPORT.append((name, expect))
        return text
    n = text.count(anchor)
    if n == expect:
        text = text.replace(anchor, new, expect)
        REPORT.append((name, n))
        return text
    # 缩进灵活匹配: 逐行拼接, 行首空白用 \s*
    if n == 0:
        lines = anchor.split("\n")
        pat = r"^([ \t]*)" + r"[ \t]*\n[ \t]*".join(re.escape(l.strip()) for l in lines)
        hits = list(re.finditer(pat, text, re.M))
        if len(hits) == expect:
            m = hits[0]
            text = text[:m.start()] + reindent(new, " " * base_indent, m.group(1)) + text[m.end():]
            REPORT.append((name + "(flex)", expect))
            return text
        raise AssertionError(f"[{name}] 锚点命中 {n} 次 / 灵活匹配 {len(hits)} 次 (期望 {expect})\n--- 锚点 ---\n{anchor[:300]}")
    raise AssertionError(f"[{name}] 锚点命中 {n} 次 (期望 {expect})")


# ================================================================
# 补丁定义
# ================================================================
CONST_BLOCK = '''
# ============================================================
# 2026-09-30 h: 预测落点释放 + 到位即静 + 瞄准点平滑 + 重进 (9-30 实飞复盘后全部改动)
# ============================================================
DROP_PRED_LEAD_S    = 0.25   # 释放指令 → 物理脱钩延迟 (s). 9-19 实测 +0.10/+0.22~0.28s; 9-30 推力/电流台阶 +0.2~0.3s
DROP_PRED_FALL_H    = 0.25   # 载荷下落终点高度 (m, 桶口)
DROP_PRED_ERR_M     = 0.04   # 预测落点误差门限 (m): |(瞄准点-飞机) - v*t_eff| < 4cm (SITL 实测 pred 最低 3.5cm ⇒ 3→4)
DROP_PRED_CONFIRM_S = 0.20   # 预测判据确认时间 (s)
DROP_PRED_VMAX      = 0.20   # 预测判据安全上限: 水平速度 (m/s), 只防极端
DROP_HEIGHT_TOL     = 0.15   # 软门控: 释放时要求 H >= DROP_HEIGHT - 该值 (m)
DROP_VZ_TOL         = 0.20   # 软门控: 释放时要求 |vz| < 该值 (m/s)
DROP_QUIET_M        = 0.03   # "到位即静": 残差 < 3cm 视为到位
DROP_QUIET_T        = 0.50   # 到位持续 (s) → 冻结像素残差积分
AIM_FILTER_TAU      = 0.30   # DROP 段瞄准点一阶低通时间常数 (s) (0=关闭)
AIM_MAX_RATE        = 0.10   # DROP 段瞄准点最大移动率 (m/s)        (0=关闭)
DROP_CONV_WINDOW    = 2.0    # 低谷带"段内最小值"改为最近 N 秒滑窗 (原: 全段最小值只降不升, 会被单帧绑架)
DROP_REAPPROACH_MAX = 1      # 优先窗口内未触发 → "重进"次数上限 (9-30 SITL: 重进耗时约 25s/次 ⇒ 2→1)
DROP_REAPPROACH_BACK = 1.0   # 重进: 沿航向后退距离 (m)
DROP_REAPPROACH_UP  = 0.80   # 重进: 抬升高度 (m)
DROP_REAPPR_TIMEOUT = 8.0    # 重进: 单次重进到位超时 (s)
DROP_ALIGN_M        = 0.08   # 下降前对准门限 (m): H 高于 drop_z+0.4m 时残差 > 该值则保持高度
DROP_ALIGN_TIMEOUT  = 5.0    # 下降前对准门控最长等待 (s), 超时放行下降
'''

INIT_BLOCK = '''        self._drop_records = []               # 每次投放: idx/aim/pos/H/last_err
        # ===== 2026-09-30 h: 预测落点 / 到位即静 / 瞄准点平滑 / 重进 状态 =====
        self._drop_pred = None                # 最近一帧预测落点误差 (m)
        self._drop_pred_start = None          # 预测判据确认计时起点
        self._drop_quiet_start = None         # 残差 < DROP_QUIET_M 起点
        self._drop_quiet = False              # 到位即静: 冻结像素残差积分
        self._drop_cmin_hist = []             # (t, conv_m) 滑窗 → 低谷带最小值
        self._aim_filt = None                 # (x, y) 瞄准点低通/限速状态
        self._drop_hist = []                  # 释放前快照 (10Hz, 最多 40 条) 供事后标定 t_eff
        self._drop_reappr = 0                 # 本次投放已重进次数
        self._drop_reappr_total = 0           # 全局重进次数 (摘要用)
        self._reappr_start = None             # 重进子阶段计时
        self._release_pred = None             # 释放瞬间预测落点误差 (m)
        self._release_teff = None             # 释放瞬间 t_eff (s)
        self._release_cmd_t = None            # 释放命令时刻 (sim_t)
        self._last_servo_err_m = None         # VISUAL_SERVO 段最近一帧残差 (m), 供下降对准门控
        self._align_start = None              # 下降对准门控计时'''

RESET_BLOCK = '''            self._drop_conv_min = None
            self._drop_tier = None
            # 2026-09-30 h: 预测判据/到位即静/滑窗低谷/瞄准点平滑 复位 (重进时 _drop_reappr 保留)
            self._drop_pred = None; self._drop_pred_start = None
            self._drop_quiet_start = None; self._drop_quiet = False
            self._drop_cmin_hist = []; self._aim_filt = None; self._drop_hist = []
            self._align_start = None'''

GATE_ANCHOR = '''            # 低谷判定: 本段残差最小值 (下限夹 DROP_CONV_MIN_FLOOR 防单帧噪声把门槛带偏)
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
                    early_ok = True
            if not self._drop_triggered and not early_ok and self._step_elapsed() < DROP_AFTER_DELAY: return'''

GATE_NEW = '''            # ===== 2026-09-30 h ⑧: 低谷带改用"最近 DROP_CONV_WINDOW 秒滑窗最小值" =====
            # 原实现"全段最小值只降不升 + 下限夹 2cm" ⇒ 单帧极值会把后续所有帧的门限锁死:
            # 9-30 第 2 投 +75.3s 出现 1.8cm 后, 后面 4.5~6.6cm 全部不合格, 被一帧绑架 9s.
            if math.isfinite(conv_m):
                self._drop_cmin_hist.append((sim_t, conv_m))
                while self._drop_cmin_hist and sim_t - self._drop_cmin_hist[0][0] > DROP_CONV_WINDOW:
                    self._drop_cmin_hist.pop(0)
                self._drop_conv_min = max(DROP_CONV_MIN_FLOOR,
                                          min(c for _, c in self._drop_cmin_hist))
            in_valley = (self._drop_conv_min is not None
                         and conv_m <= self._drop_conv_min + DROP_CONV_MIN_BAND)
            vxy = math.hypot(self.vx, self.vy)
            # ===== 2026-09-30 h ①: 预测落点判据 (核心) =====
            # 载荷落点 ≈ 释放点 + 释放瞬间速度 × t_eff (t_eff = 脱钩延迟 + 下落时间).
            # 释放条件: |(瞄准点 - 飞机) - v × t_eff| < DROP_PRED_ERR_M (矢量式, 横向/前向统一).
            # 动机 (9-30 实测): "残差小"与"速度小"在摆动下互斥 —— 穿越瞄准点必然带 2πfA ≈ 0.11 m/s
            # 速度, 旧判据误差下限 ≈ A·ω·t_eff ≈ 7.7cm; 9-30 第 2 投在残差 1.8cm 时因 vxy=0.124 被挡,
            # 掉到 10cm 保底档 ⇒ 脉冲发出时已摆到 12.9cm. 预测式按速度反推提前量, 让弹道漂移把落点送进桶.
            pred_m = float('inf'); t_eff = DROP_PRED_LEAD_S
            if math.isfinite(conv_m):
                t_eff = DROP_PRED_LEAD_S + math.sqrt(
                    2.0 * max(0.05, dz_h_now - DROP_PRED_FALL_H) / 9.81)
                _aim_x = self.home_x + self._servo_target_x
                _aim_y = self.home_y + self._servo_target_y
                pred_m = math.hypot((_aim_x - self.x) - self.vx * t_eff,
                                    (_aim_y - self.y) - self.vy * t_eff)
            self._drop_pred = pred_m
            if math.isfinite(conv_m):
                self._drop_hist.append((sim_t, conv_m, pred_m, vxy,
                                        self.home_x + self._servo_target_x,
                                        self.home_y + self._servo_target_y,
                                        self.x, self.y, dz_h_now))
                while len(self._drop_hist) > 40:
                    self._drop_hist.pop(0)
            # ⑨ 软门控: 释放时高度不能明显偏高、不能在大幅升降中
            height_ok = (dz_h_now >= DROP_HEIGHT - DROP_HEIGHT_TOL) and (abs(self.vz) < DROP_VZ_TOL)
            pred_ok = (release_valid and math.isfinite(pred_m) and pred_m < DROP_PRED_ERR_M
                       and vxy < DROP_PRED_VMAX and height_ok)
            # ④ 到位即静: 残差 < 3cm 持续 0.5s ⇒ 冻结像素残差积分 (不再往瞄准点里注入运动)
            if math.isfinite(conv_m) and conv_m < DROP_QUIET_M:
                if self._drop_quiet_start is None:
                    self._drop_quiet_start = sim_t
                elif sim_t - self._drop_quiet_start > DROP_QUIET_T:
                    self._drop_quiet = True
            else:
                self._drop_quiet_start = None
                if self._drop_quiet and (not math.isfinite(conv_m) or conv_m > 2 * DROP_QUIET_M):
                    self._drop_quiet = False
            stay_still = vxy < DROP_ARRIVE_VMAX
            # 档位: 0=预测落点(优选) / 1=Tier1(5cm 低谷准静止) / 2=Tier2(10cm 保底, 需重进用尽)
            tier = None
            if pred_ok:
                tier = 0
            elif release_valid and conv_m < DROP_ARRIVE_M and in_valley and stay_still:
                tier = 1     # Tier1 优选: 5cm + 低谷 + 准静止
            elif (release_valid and self._step_elapsed() >= DROP_PREFER_WINDOW
                  and conv_m < DROP_ARRIVE_M_LATE
                  and self._drop_reappr >= DROP_REAPPROACH_MAX):
                tier = 2     # Tier2 保底: 只在"重进次数用尽"之后才允许 (h 起)
            if tier != self._drop_tier:          # 档位变化即重启确认计时 (避免混档提前放行)
                self._drop_tier = tier
                self._drop_converge_start = sim_t if tier is not None else None
            early_ok = False
            if tier is not None and self._drop_converge_start is not None:
                _need_t = (DROP_PRED_CONFIRM_S if tier == 0
                           else (DROP_CONVERGE_EARLY if tier == 1 else DROP_CONVERGE_LATE))
                if sim_t - self._drop_converge_start > _need_t:
                    early_ok = True
            # ⑥ 优先窗口内既未预测放行也未 Tier1 放行 ⇒ 先"重进", 不硬投保底档
            if (not self._drop_triggered and not early_ok
                    and self._step_elapsed() >= DROP_PREFER_WINDOW
                    and self._drop_reappr < DROP_REAPPROACH_MAX
                    and tier != 2):
                self._drop_reappr += 1
                self._drop_reappr_total += 1
                self.get_logger().warn(
                    f"DROP 优先窗口内未收敛 (conv={conv_m * 100:.1f}cm pred={pred_m * 100:.1f}cm "
                    f"vxy={vxy:.3f}m/s) → 第 {self._drop_reappr}/{DROP_REAPPROACH_MAX} 次重进 "
                    f"(后退 {DROP_REAPPROACH_BACK:.1f}m + 抬升 {DROP_REAPPROACH_UP:.2f}m)")
                self._aux5_value = AUX5_LOCK
                self._enter_step(STEP_REAPPROACH)
                return
            if not self._drop_triggered and not early_ok and self._step_elapsed() < DROP_AFTER_DELAY: return'''

BIAS_ANCHOR = '''                    if (lock_mode in ('accept', 'reanchor')
                            and math.hypot(ex * dz_h / FX, ey * dz_h / FY) < SERVO_BIAS_ERR_MAX_M
                            and math.hypot(self.vx, self.vy) < SERVO_BIAS_VMAX):'''
BIAS_NEW = '''                    if (lock_mode in ('accept', 'reanchor')
                            and math.hypot(ex * dz_h / FX, ey * dz_h / FY) < SERVO_BIAS_ERR_MAX_M
                            and math.hypot(self.vx, self.vy) < SERVO_BIAS_VMAX
                            and not self._drop_quiet):   # 2026-09-30 h ④: 到位即静 → 冻结积分'''

AIM_ANCHOR = '''                        self._servo_target_x, self._servo_target_y = lx, ly
                    else:
                        self._servo_target_x, self._servo_target_y = anchor_x, anchor_y'''
AIM_NEW = '''                        self._servo_target_x, self._servo_target_y = lx, ly
                    else:
                        self._servo_target_x, self._servo_target_y = anchor_x, anchor_y
                    # 2026-09-30 h ③: 瞄准点一阶低通 + 限速 (抑制"追一个自己在跳的点")
                    # 依据: 9-30 第 2 投等待段瞄准点自身在 x 方向峰峰 17.2cm (第 1 投仅 8.2cm),
                    # 残差中位 10.3 vs 9.5cm ⇒ 飞机在追一个自己抖动的目标点.
                    if AIM_FILTER_TAU > 0.0 or AIM_MAX_RATE > 0.0:
                        if self._aim_filt is None:
                            self._aim_filt = (self._servo_target_x, self._servo_target_y)
                        _fx, _fy = self._aim_filt
                        _dt_aim = 0.05
                        if AIM_FILTER_TAU > 0.0:
                            _a = _dt_aim / (AIM_FILTER_TAU + _dt_aim)
                            _fx += (self._servo_target_x - _fx) * _a
                            _fy += (self._servo_target_y - _fy) * _a
                        if AIM_MAX_RATE > 0.0:
                            _mx = AIM_MAX_RATE * _dt_aim
                            _fx += max(-_mx, min(_mx, self._servo_target_x - _fx))
                            _fy += max(-_mx, min(_mx, self._servo_target_y - _fy))
                        self._aim_filt = (_fx, _fy)
                        self._servo_target_x, self._servo_target_y = _fx, _fy'''


VSERVO_ANCHOR = '''            drop_z = self.home_z - DROP_HEIGHT
            target_z = max(drop_z, self.z - SERVO_DESCENT_SPEED * 0.05)'''
VSERVO_NEW = '''            drop_z = self.home_z - DROP_HEIGHT
            target_z = max(drop_z, self.z - SERVO_DESCENT_SPEED * 0.05)
            # 2026-09-30 h ⑦: 下降前对准门控 —— H 高于 drop_z+0.4m 时, 若残差仍 > DROP_ALIGN_M 先保持高度对准
            # (动机: 9-30 两次 DROP 段低空等待 6.7s/11.5s, 而 9-27 e 只用 3.39s ⇒ 把收敛提前到下降段完成)
            if self.z < drop_z - 0.4 and self._last_servo_err_m is not None \\
                    and self._last_servo_err_m > DROP_ALIGN_M:
                if self._align_start is None:
                    self._align_start = self.get_clock().now()
                if (self.get_clock().now() - self._align_start).nanoseconds * 1e-9 < DROP_ALIGN_TIMEOUT:
                    target_z = self.z          # 保持当前高度, 先对准再下降
            else:
                self._align_start = None'''

SERR_ANCHOR = '''                v_fwd, v_lat, err_px, ex, ey = self.pid.compute(
                    roi, 0.05, pixel_offset, self._pitch, self._roll, height_scale, pixel_offset_y)
                dx_ned, dy_ned, _ = self._body_to_ned(v_fwd * 0.05, v_lat * 0.05)'''
SERR_NEW = '''                v_fwd, v_lat, err_px, ex, ey = self.pid.compute(
                    roi, 0.05, pixel_offset, self._pitch, self._roll, height_scale, pixel_offset_y)
                self._last_servo_err_m = math.hypot(ex * dz_h / FX, ey * dz_h / FY)   # 2026-09-30 h ⑦
                dx_ned, dy_ned, _ = self._body_to_ned(v_fwd * 0.05, v_lat * 0.05)'''

RELEASE_ANCHOR = '''                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()'''
RELEASE_NEW = '''                self._drop_triggered = True; self._drop_signal_start = self.get_clock().now()
                # 2026-09-30 h ②: 记录释放瞬间的预测量 (供事后标定 t_eff / 补偿)
                self._release_pred = pred_m; self._release_teff = t_eff; self._release_cmd_t = sim_t
                self._drop_reappr = 0'''

WAY_ANCHOR = '''                if early_ok:
                    _cminways = f"{self._drop_conv_min * 100:.1f}cm" if self._drop_conv_min is not None else "--"
                    _tname = "5cm低谷准静止" if self._drop_tier == 1 else "10cm保底"
                    drop_way = f"收敛即投(Tier{self._drop_tier} {_tname} 段内最小={_cminways})"
                else:
                    drop_way = f"{self._step_elapsed():.1f}s/{DROP_AFTER_DELAY:.0f}s 兜底"'''
WAY_NEW = '''                if early_ok:
                    _cminways = f"{self._drop_conv_min * 100:.1f}cm" if self._drop_conv_min is not None else "--"
                    _tname = {0: f"预测落点<{DROP_PRED_ERR_M * 100:.0f}cm", 1: "5cm低谷准静止",
                              2: "10cm保底(重进用尽)"}.get(self._drop_tier, "?")
                    drop_way = (f"收敛即投(Tier{self._drop_tier} {_tname} 段内最小={_cminways} "
                                f"pred={pred_m * 100:.1f}cm t_eff={t_eff:.2f}s vxy={vxy:.3f})")
                    if self._drop_tier == 2:
                        self.get_logger().warn(
                            f"⚠ 本投为 Tier2 保底档 (重进已用尽): pred={pred_m * 100:.1f}cm "
                            f"conv={conv_m * 100:.1f}cm vxy={vxy:.3f}m/s ⇒ 预计落点偏差偏大")
                else:
                    drop_way = f"{self._step_elapsed():.1f}s/{DROP_AFTER_DELAY:.0f}s 兜底"
                    self.get_logger().warn(
                        f"⚠ 本投为超时兜底释放: pred={pred_m * 100:.1f}cm conv={conv_m * 100:.1f}cm "
                        f"vxy={vxy:.3f}m/s H={dz_h_now:.2f}m ⇒ 预计落点偏差偏大")
                # 2026-09-30 h ②: 释放前 4s 快照 (10Hz), 供事后标定 t_eff / 补偿 (DROP-TRACE 行)
                if self._drop_hist:
                    _tr = " ".join(f"{int(max(0.0, sim_t - h[0]) * 10)}|{h[1] * 100:.0f}|{h[2] * 100:.0f}|{h[3] * 100:.0f}"
                                   for h in self._drop_hist[-24:])
                    self.get_logger().info(f"  DROP-TRACE (dt|conv|pred|vxy, 单位 cm & cm/s): {_tr}")'''

REC_ANCHOR = '''                    "conv_min": self._drop_conv_min,  # 本段残差最小值 (m)
                })'''
REC_NEW = '''                    "conv_min": self._drop_conv_min,  # 本段残差最小值 (m)
                    # 2026-09-30 h: 预测落点闭环字段 (②)
                    "pred_m": self._release_pred,
                    "t_eff": self._release_teff,
                    "vxy": math.hypot(self.vx, self.vy),
                    "reappr": self._drop_reappr_total,
                    "actual_impact": None,            # 实飞落点真值 (人工/场地测量后填)
                })'''

REAPPR_ANCHOR = '''            self._aux5_value = AUX5_LOCK; self._drop_triggered = False; self._drop_signal_start = None
            self._enter_step(STEP_CLIMB)

        # ---- CLIMB: 投弹后爬升 → 下一桶或侦察区 ----'''
REAPPR_NEW = '''            self._aux5_value = AUX5_LOCK; self._drop_triggered = False; self._drop_signal_start = None
            self._enter_step(STEP_CLIMB)

        # ---- REAPPROACH (2026-09-30 h ⑥ 新增): 优先窗口内未收敛 → 后退+抬升 → 重新下降对准 ----
        elif self.step == STEP_REAPPROACH:
            if self._reappr_start is None:
                self._reappr_start = self.get_clock().now()
                self.get_logger().info(
                    f"REAPPROACH #{self._drop_reappr}: 后退 {DROP_REAPPROACH_BACK:.1f}m + "
                    f"抬升到 H={DROP_HEIGHT + DROP_REAPPROACH_UP:.2f}m, 然后重新 VISUAL_SERVO")
            _el = (self.get_clock().now() - self._reappr_start).nanoseconds * 1e-9
            _bx = -math.cos(self.heading) * DROP_REAPPROACH_BACK
            _by = -math.sin(self.heading) * DROP_REAPPROACH_BACK
            _tx = self.home_x + self._servo_target_x + _bx
            _ty = self.home_y + self._servo_target_y + _by
            _tz = self.home_z - (DROP_HEIGHT + DROP_REAPPROACH_UP)
            self._pub_pos(_tx, _ty, _tz, self.heading)
            _dist = math.hypot(self.x - _tx, self.y - _ty)
            if _dist < POS_ARRIVE_DIST or _el > DROP_REAPPR_TIMEOUT:
                self.get_logger().info(
                    f"REAPPROACH 到位 (dist={_dist:.2f}m, {_el:.1f}s) → 重新下降对准")
                self._enter_step(STEP_VISUAL_SERVO)
            return

        # ---- CLIMB: 投弹后爬升 → 下一桶或侦察区 ----'''

SUMMARY_ANCHOR = """              f"H={r['H']:.2f}m vehicle_to_anchor={dev:.3f}m actual_impact=unmeasured tier=T{r.get('tier')} conv_min={r.get('conv_min') if r.get('conv_min') is None else round(r['conv_min'] * 100, 1)}cm{err_str}")"""
SUMMARY_NEW = """              f"H={r['H']:.2f}m vehicle_to_anchor={dev:.3f}m actual_impact={r.get('actual_impact') or 'unmeasured'} "
              f"tier=T{r.get('tier')} conv_min={r.get('conv_min') if r.get('conv_min') is None else round(r['conv_min'] * 100, 1)}cm "
              f"pred={r.get('pred_m') if r.get('pred_m') is None else round(r['pred_m'] * 100, 1)}cm "
              f"t_eff={r.get('t_eff') if r.get('t_eff') is None else round(r['t_eff'], 2)}s "
              f"vxy={r.get('vxy') if r.get('vxy') is None else round(r['vxy'], 3)}m/s{err_str}")"""

SUMMARY_TAIL_ANCHOR = """        L(f"  Buckets dropped: {len(self._drop_records)}/{getattr(self, '_target_count', 0)}")"""
SUMMARY_TAIL_NEW = """        L(f"  Buckets dropped: {len(self._drop_records)}/{getattr(self, '_target_count', 0)} "
          f"重进次数={getattr(self, '_drop_reappr_total', 0)} (h: 优先窗口内未收敛不硬投, 重进最多 {DROP_REAPPROACH_MAX} 次)")"""

SERVO_LOG_ANCHOR = """                        f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm(门限T1={DROP_ARRIVE_M * 100:.0f}cm/T2={DROP_ARRIVE_M_LATE * 100:.0f}cm)")"""
SERVO_LOG_NEW = """                        f"等效地面={math.hypot(ex * dz_h / FX, ey * dz_h / FY) * 100:.1f}cm(门限P={DROP_PRED_ERR_M * 100:.0f}cm/T1={DROP_ARRIVE_M * 100:.0f}cm/T2={DROP_ARRIVE_M_LATE * 100:.0f}cm) "
                        f"pred={self._drop_pred * 100 if self._drop_pred is not None else float('nan'):.1f}cm "
                        f"quiet={self._drop_quiet} reappr={self._drop_reappr}")"""

HEADER_ANCHOR = """子项目 g — 投放区 30m + 侦察区动作序列 + 返航 (基底 = 项目 f, 只改"两瓶横向补偿")."""
HEADER_NEW = """子项目 h — 投放区 30m + 侦察区动作序列 + 返航 (基底 = 项目 g, 只改"投放代码", 9-30 实飞复盘后).

2026-09-30 h: 按 9-30 实飞 (g, 主供电线烧断坠机前两次投放) 复盘结论, 一次性落地全部投放代码优化.
  e/f/g 一行未改; 本文件 = g.py 逐字节复制 + 下列补丁 (释放通道/补偿数值/飞控参数均未动):
  ① 预测落点释放   pred_m = |(瞄准点-飞机) - v*t_eff| < 4cm 确认 0.2s  → 新增 Tier0 优选档
                   (依据: 9-30 第 2 投残差 1.8cm 时因 vxy=0.124 被准静止门控挡住 → 掉保底档 → 12.9cm 释放)
  ② 记录字段补齐   pred/t_eff/vxy/tier/reappr/actual_impact + 释放前 4s DROP-TRACE 快照
  ③ 瞄准点平滑      SERVO_EST_WEIGHT 0.7→0.5 + 一阶低通 τ=0.30s + 限速 10cm/s
  ④ 到位即静        残差<3cm 持续 0.5s → 冻结像素残差积分 (不再往瞄准点注入运动)
  ⑤ 现场质量提示    Tier2/兜底/陈旧检测释放时打 WARN 并打印 pred/conv/vxy
  ⑥ 保底档改重进    优先窗口(10s)内未触发 → 后退 1m+抬升 0.8m 重进 1 次, 仍不行才允许 Tier2
  ⑦ 提前对准        下降到 drop_z+0.4m 以下前, 残差>8cm 则保持高度对准 (超时 5s 放行)
  ⑧ 判据参数        CONVERGE_EARLY 0.5→0.25 / _LATE 1.0→0.4; 低谷最小值改 2s 滑窗
  ⑨ 软门控          释放时要求 H >= DROP_HEIGHT-0.15m 且 |vz| < 0.20m/s"""


def patch_file(path, is_sitl):
    src = SITL_SRC if is_sitl else FLY_SRC
    with open(src, "r", encoding="utf-8") as f:
        t = f.read()
    before = t

    # ---- 重命名 ----
    n_cls = len(re.findall(r"\bGMission\b", t))
    t = re.sub(r"\bGMission\b", "HMission", t)
    n_node = len(re.findall(r"'g_mission'", t))
    t = t.replace("'g_mission'", "'h_mission'")
    if n_cls < 2 or n_node != 1:
        raise AssertionError(f"[重命名] GMission={n_cls} (期望≥2), 'g_mission'={n_node} (期望1)")
    REPORT.append((f"rename GMission->HMission x{n_cls}, g_mission->h_mission", 1))
    t = patch(t, "MISSION_REVISION", 'MISSION_REVISION = "competition-water-20260927-r1-g"',
              'MISSION_REVISION = "competition-water-20260930-r1-h"')
    t = patch(t, "header 描述", HEADER_ANCHOR, HEADER_NEW, base_indent=0)
    t = patch(t, "constants 小节标题",
              "# ===== 2026-09-27 项目 g: 两瓶横向补偿按 9-27 实飞落点分别重标 =====",
              "# ===== 2026-09-27 项目 g: 两瓶横向补偿按 9-27 实飞落点分别重标 (h 原样保留) =====", base_indent=0)

    # ---- ③ SERVO_EST_WEIGHT ----
    t = patch(t, "③ SERVO_EST_WEIGHT 0.7->0.5",
              "SERVO_EST_WEIGHT   = 0.7   # 反投影权重: target = 0.7*反投影 + 0.3*锚点 (每帧 _roi_to_ned 重建)",
              "SERVO_EST_WEIGHT   = 0.5   # 2026-09-30 h③: 0.7->0.5 (提高锚点权重, 抑制瞄准点自身抖动)",
              base_indent=0)

    # ---- ⑧ 判据参数 ----
    t = patch(t, "⑧ DROP_CONVERGE_EARLY 0.5->0.25",
              "DROP_CONVERGE_EARLY = 0.5  # Tier1 确认时间 (2026-09-24 f: 1.0->0.5s; 低位窗口仅 1~3s)",
              "DROP_CONVERGE_EARLY = 0.25 # 2026-09-30 h⑧: 0.5->0.25 (缩短确认延迟吃掉的精度)", base_indent=0)
    t = patch(t, "⑧ DROP_CONVERGE_LATE 1.0->0.4",
              "DROP_CONVERGE_LATE  = 1.0  # Tier2 确认时间 (同 e)",
              "DROP_CONVERGE_LATE  = 0.4  # 2026-09-30 h⑧: 1.0->0.4 (9-30 第 2 投确认期间摆走 3~5cm)",
              base_indent=0)

    # ---- 新增常量块 (插在 DROP_DETECT_MAX_AGE 之后) ----
    a = 'DROP_DETECT_MAX_AGE = 0.5'
    i = t.find(a)
    if i < 0:
        raise AssertionError("[const] 未找到 DROP_DETECT_MAX_AGE")
    j = t.find("\n", i) + 1
    t = t[:j] + CONST_BLOCK + t[j:]
    REPORT.append(("新增常量块 (h)", 1))

    # ---- 状态机: STEP_REAPPROACH ----
    t = patch(t, "状态机 range(13)->14",
              " STEP_CLIMB, STEP_SCOUT, STEP_RETURN, STEP_LAND) = range(13)",
              " STEP_CLIMB, STEP_SCOUT, STEP_RETURN, STEP_LAND,\n"
              " STEP_REAPPROACH) = range(14)", base_indent=1)
    t = patch(t, "STEP_NAME",
              '    STEP_DROP: "DROP", STEP_CLIMB: "CLIMB", STEP_SCOUT: "SCOUT",',
              '    STEP_DROP: "DROP", STEP_CLIMB: "CLIMB", STEP_SCOUT: "SCOUT",\n'
              '    STEP_REAPPROACH: "REAPPROACH",', base_indent=4)

    # ---- 状态初始化 ----
    t = patch(t, "状态初始化 (h)",
              "        self._drop_records = []               # 每次投放: idx/aim/pos/H/last_err",
              INIT_BLOCK, base_indent=8)

    # ---- _enter_step 复位 ----
    t = patch(t, "_enter_step DROP 复位 (h)",
              "            self._drop_conv_min = None\n            self._drop_tier = None",
              RESET_BLOCK, base_indent=12)

    # ---- 新增 dz_h_now (dz_h 仅在检测新鲜分支内定义, 判据块不可直接用) ----
    t = patch(t, "DROP 段 dz_h_now",
              "            drop_z = self.home_z - DROP_HEIGHT\n"
              "            if self._servo_active and self._drop_detection_fresh():",
              "            drop_z = self.home_z - DROP_HEIGHT\n"
              "            # 2026-09-30 h: dz_h 只在检测新鲜分支内定义, 释放判据需要本帧高度 ⇒ 单独取\n"
              "            dz_h_now = max(0.3, self.home_z - self.z)\n"
              "            if self._servo_active and self._drop_detection_fresh():", base_indent=12)

    # ---- ① 核心判据块 ----
    t = patch(t, "① 释放判据块 (预测落点+滑窗低谷+到位即静+重进)", GATE_ANCHOR, GATE_NEW, base_indent=12)

    # ---- ④ 冻结积分 ----
    t = patch(t, "④ 到位即静冻结积分", BIAS_ANCHOR, BIAS_NEW, base_indent=20)

    # ---- ③ 瞄准点平滑 ----
    t = patch(t, "③ 瞄准点平滑+限速", AIM_ANCHOR, AIM_NEW, base_indent=24)

    # ---- ⑦ 下降对准门控 ----
    t = patch(t, "⑦ 下降前对准门控", VSERVO_ANCHOR, VSERVO_NEW, base_indent=12)
    # ⑦ VISUAL_SERVO 残差记录: 在唯一的 dx_ned/_body_to_ned 行之前插入 (两版 dt 写法/缩进不同)
    _ln = t.split("\n")
    _ix = [i for i, l in enumerate(_ln) if "dx_ned, dy_ned, _ = self._body_to_ned(" in l]
    if len(_ix) != 1:
        raise AssertionError(f"[⑦] dx_ned 锚点命中 {len(_ix)} 次 (期望 1)")
    _i = _ix[0]
    _ind = re.match(r"[ \t]*", _ln[_i]).group(0)
    _ln.insert(_i, _ind + "self._last_servo_err_m = math.hypot(ex * dz_h / FX, ey * dz_h / FY)"
                    + "   # 2026-09-30 h⑦: 下降对准门控用")
    t = "\n".join(_ln)
    REPORT.append(("⑦ VISUAL_SERVO 残差记录", 1))

    # ---- ② 释放瞬间记录 ----
    t = patch(t, "② 释放瞬间记录", RELEASE_ANCHOR, RELEASE_NEW, base_indent=16)

    # ---- ⑤ 触发方式文案 + WARN + TRACE ----
    t = patch(t, "⑤ 触发方式/Tier 文案 + WARN + DROP-TRACE", WAY_ANCHOR, WAY_NEW, base_indent=16)
    t = patch(t, "② 落点记录字段", REC_ANCHOR, REC_NEW, base_indent=20)

    # ---- ⑥ REAPPROACH 分支 ----
    t = patch(t, "⑥ REAPPROACH 状态分支", REAPPR_ANCHOR, REAPPR_NEW, base_indent=12)

    # ---- ② MISSION SUMMARY ----
    t = patch(t, "② MISSION SUMMARY 落点行", SUMMARY_ANCHOR, SUMMARY_NEW, base_indent=14)
    t = patch(t, "② MISSION SUMMARY 摘要行", SUMMARY_TAIL_ANCHOR, SUMMARY_TAIL_NEW, base_indent=8)

    # ---- 日志行补 pred/quiet ----
    t = patch(t, "DROP-SERVO 日志补 pred/quiet/reappr", SERVO_LOG_ANCHOR, SERVO_LOG_NEW, base_indent=24)

    # ---- 结构断言 ----
    for must in ("DROP_PRED_ERR_M", "STEP_REAPPROACH", "pred_m", "self._drop_reappr",
                 "HMission", "'h_mission'", "DROP-TRACE", "actual_impact"):
        if must not in t:
            raise AssertionError(f"[结构] 缺少 {must}")
    for mustnot in ("GMission", "'g_mission'", "competition-water-20260927-r1-g"):
        if mustnot in t:
            raise AssertionError(f"[结构] 残留 {mustnot}")
    _n_ra = t.count("STEP_REAPPROACH")
    if _n_ra < 4:
        raise AssertionError(f"[结构] STEP_REAPPROACH 出现 {_n_ra} 次 (期望≥4: 枚举/名称/派发/触发)")
    if "STEP_REAPPROACH) = range(14)" not in t:
        raise AssertionError("[结构] 枚举未扩到 range(14)")
    if 'STEP_REAPPROACH: "REAPPROACH"' not in t:
        raise AssertionError("[结构] STEP_NAME 未加 REAPPROACH")
    if "elif self.step == STEP_REAPPROACH:" not in t:
        raise AssertionError("[结构] REAPPROACH 分支未插入")
    REPORT.append(("结构断言全部通过", 1))
    return t, before


def main():
    for src in (SITL_SRC, FLY_SRC):
        if not os.path.exists(src):
            raise SystemExit(f"源文件缺失: {src}")
    print(f"源: {SITL_SRC}  md5={md5(SITL_SRC)}")
    print(f"源: {FLY_SRC}  md5={md5(FLY_SRC)}")
    out = {}
    for is_sitl, dst in ((True, SITL_DST), (False, FLY_DST)):
        t, before = patch_file(dst, is_sitl)
        out[dst] = t
        if CHECK:
            continue
        if os.path.exists(dst):
            shutil.copy2(dst, dst + f".bak.{STAMP}")
        with open(dst, "w", encoding="utf-8") as f:
            f.write(t)
        print(f"写出 {dst}  {len(t.splitlines())} 行 (源 {len(before.splitlines())} 行, "
              f"净增 {len(t.splitlines()) - len(before.splitlines())})")
    print("\n=== 补丁清单 ===")
    for name, n in REPORT:
        print(f"  ✓ {name}")
    if CHECK:
        print("\n[--check] 未写盘")

    if CHECK:
        return
    # ---- 世界 / 启动脚本 / setup.py ----
    files = []
    gw, hw = os.path.join(ROOT, "worlds/g.world"), os.path.join(ROOT, "worlds/h.world")
    if not os.path.exists(hw):
        shutil.copy2(gw, hw); files.append(hw)
    tg, th = os.path.join(ROOT, "scripts/test_g.sh"), os.path.join(ROOT, "scripts/test_h.sh")
    if not os.path.exists(th):
        s = open(tg, encoding="utf-8").read()
        s = s.replace("子项目 g —", "子项目 h —").replace("test_g.sh", "test_h.sh")
        s = s.replace("g.world", "h.world").replace("/tmp/g_output.log", "/tmp/h_output.log")
        s = s.replace("/tmp/g_judge.log", "/tmp/h_judge.log").replace("g_mission", "h_mission")
        s = s.replace("ros2 run drone_drl g", "ros2 run drone_drl h")
        open(th, "w", encoding="utf-8").write(s)
        os.chmod(th, 0o755); files.append(th)
    fg, fh = os.path.join(ROOT, "fly/g.sh"), os.path.join(ROOT, "fly/h.sh")
    if not os.path.exists(fh):
        s = open(fg, encoding="utf-8").read()
        s = s.replace("g_mission", "h_mission").replace("fly g", "fly h")
        s = s.replace("fly/g.py", "fly/h.py").replace(" g ", " h ")
        open(fh, "w", encoding="utf-8").write(s)
        os.chmod(fh, 0o755); files.append(fh)
    for sp, pat in ((os.path.join(ROOT, "src/drone_drl/setup.py"), "        'g = drone_drl.g:main',"),
                    (os.path.join(ROOT, "fly/src/fly/setup.py"), "            'g = fly.g:main',")
                    ):
        s = open(sp, encoding="utf-8").read()
        if pat not in s:
            raise AssertionError(f"[setup] 未找到 g 入口: {sp}")
        shutil.copy2(sp, sp + f".bak.{STAMP}")
        new = pat.replace("'g = ", "'h = ").replace(".g:main'", ".h:main'")
        if new not in s:
            s = s.replace(pat, pat + "\n" + new)
        open(sp, "w", encoding="utf-8").write(s)
        files.append(sp)
    print("\n=== 其他文件 ===")
    for f in files:
        print(f"  + {f}")
    print("\n完成. 下一步: 离线验收 (scripts/test_h_drop_logic_20260930.py) → colcon build → SITL")


if __name__ == "__main__":
    main()
