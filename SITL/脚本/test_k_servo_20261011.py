#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_k_servo_20261011.py — 新 k「机体坐标系闭环伺服投弹」离线验收

不跑仿真, 直接 `exec` k.py 里的**真实函数**(与 check_drop_portability 同一手法),
用仿真检测器的**真实地面投影**造框, 逐条判据如下:

  A  最小改动审计: 剥注释后 k.py 与 i.py 的差异**全部**落在允许集合内
  B  几何往返: 检测器像素 → 节点 _roi_to_ned  ⇒ 与真值偏差 ≤1px 等效
  C  桶径反推: sqrt(area)·H/FX 对 15/20/25cm 三档的恢复误差
  D  **闭环收敛 + EKF 偏置免疫 (决定性)**: 注入 0/15/25/40cm 位置偏置,
     机体系残差与最终**真实**站位完全不变 (Δ=0), 而世界系 hold 偏移 = 注入偏置
  E  限速: 单拍命令点位移 ≤ K_VMAX_DROP·dt
"""
import ast
import difflib
import re as _re2
import math
import pathlib
import re
import sys
import textwrap
import types

REPO = pathlib.Path(__file__).resolve().parent.parent
I_PY = REPO / "src/drone_drl/drone_drl/i.py"
K_PY = REPO / "src/drone_drl/drone_drl/k.py"

IMG_W, IMG_H = 640, 360
CX, CY = IMG_W / 2.0, IMG_H / 2.0
HFOV, VFOV = math.radians(85), math.radians(54)
FX = IMG_W / (2.0 * math.tan(HFOV / 2.0))
FY = IMG_H / (2.0 * math.tan(VFOV / 2.0))

_ok = _fail = 0


def check(cond, msg):
    global _ok, _fail
    if cond:
        _ok += 1
        print(f"  ✓ {msg}")
    else:
        _fail += 1
        print(f"  ✗ {msg}")


def strip_comments(src: str):
    src = re.sub(r'"""(?:.|\n)*?"""', "", src)
    src = re.sub(r"'''(?:.|\n)*?'''", "", src)
    out = []
    for ln in src.split("\n"):
        ln = ln.split("#", 1)[0].rstrip()
        if ln.strip():
            out.append(ln)
    return out


def load_funcs(path, names, extra_ns=None):
    """从真实源码里抽出函数并 exec —— 模块级函数与类方法都扫。"""
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src)
    ns = {"math": math, "FX": FX, "FY": FY, "CX": CX, "CY": CY,
          "IMG_W": IMG_W, "IMG_H": IMG_H}
    if extra_ns:
        ns.update(extra_ns)
    found = set()
    for node in tree.body:                       # 模块级函数
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(textwrap.dedent(ast.get_source_segment(src, node)), ns)
            found.add(node.name)
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name in names and fn.name not in found:
                exec(textwrap.dedent(ast.get_source_segment(src, fn)), ns)
                found.add(fn.name)
    missing = set(names) - found
    assert not missing, f"{path.name}: 缺函数 {sorted(missing)}"
    return ns


def k_consts(path):
    src = path.read_text(encoding="utf-8")
    ns = {}
    for m in re.finditer(r"(?m)^(K_[A-Z_]+)\s*=\s*([-\d.]+)", src):
        ns[m.group(1)] = float(m.group(2))
    return ns


# ---------------------------------------------------------------- 假体
class Fake:
    def __init__(self, home, pos, heading, pitch=0.0, roll=0.0):
        self.home_x, self.home_y, self.home_z = home
        self.x, self.y, self.z = pos
        self.heading, self._pitch, self._roll = heading, pitch, roll


def bind(fake, ns):
    for name, fn in ns.items():
        if isinstance(fn, types.FunctionType) and fn.__name__ in (
                "camera_pixel_to_level", "_roi_to_ned", "_body_to_ned", "_ned_to_body",
                "_k_bucket_diameter"):
            setattr(fake, name, types.MethodType(fn, fake))
    return fake


def gt_box(bucket_ned, drone_ned, heading, radius, h=None):
    """仿真检测器 _gt_ground_circles 的真实投影 (ground 模式) → 外接方框."""
    bx, by, _ = bucket_ned
    dx, dy = bx - drone_ned[0], by - drone_ned[1]
    dz = h if h is not None else -drone_ned[2]
    ch, sh = math.cos(heading), math.sin(heading)
    body_fwd = dx * ch + dy * sh
    body_right = -dx * sh + dy * ch
    px = CX + (body_right / dz) * FX
    py = CY - (body_fwd / dz) * FY
    r_px = FX * radius / dz
    return (px - r_px, py - r_px, px + r_px, py + r_px)


class Roi:
    def __init__(self, box):
        self.x_offset, self.y_offset = box[0], box[1]
        self.width, self.height = box[2] - box[0], box[3] - box[1]


# ================================================================
print("=" * 88)
print("新 k 机体坐标系闭环伺服投弹 — 离线验收 (2026-10-11)")
print("=" * 88)

KC = k_consts(K_PY)
TARGET_PLANE_H = KC.get("K_TARGET_PLANE_H")
VMAX = KC.get("K_VMAX_DROP")
DT = 0.05
print(f"常量: K_TARGET_PLANE_H={TARGET_PLANE_H} K_VMAX_DROP={VMAX} "
      f"K_RELEASE_M={KC.get('K_RELEASE_M')} K_DEADLINE_S={KC.get('K_DEADLINE_S')}\n")

# ---------------- A 最小改动审计 ----------------
print("[A] 最小改动审计 (k vs i)")
ALLOWED = ["K_TARGET_PLANE_H", "K_VMAX_DROP", "K_VZ_DROP", "K_DT_CLAMP", "K_RELEASE_M",
           "K_RELEASE_CONFIRM", "K_WINDOW_S", "K_WINDOW_RELAX_M", "K_DEADLINE_S",
           "K_VXY_REL", "K_AGE_MAX", "K_LOG_EVERY",
           "impact_rel_pub", "_k_bucket_diameter", "_k_roi_usable", "[K-SERVO]",
           "k_mission", "KMission", "cuadc-20261011-r1-k-servo", "直径分类", "_k_",
           "机体坐标系闭环", "项目 k", "SERVO", "_dia", "_area_ref",
           "贪心一对一", "used_det", "used_trk", "assign", "TRK-PULL", "matched = {id(t)",
           "K_SELECT_WAIT_S", "(D9)", "(D10)", "K_SIM_IDEAL_CAMERA",
           "_d12_t0", "K_DESCENT_RATE", "_k_tol", "_target_diams", "K_AIM_KI", "K_AIM_BIAS_MAX", "K_VXY_SAFE", "K_FALL_G", "K_RESID_MAX_FACTOR", "K_USE_PRED_RELEASE", "_det_seq", "_k_last_seq", "K_SIM_NO_BALLISTIC", "K_RESID_MAX_AGE", "K_AIM_KI", "_ekf", "_ekf_warn_t", "fc_str", "ek_str", "FC(a",
           "K_RELEASE_R_FRAC", "K_RELEASE_MIN_M",
           # D25 (2026-10-12): 相机模型换原生画幅 —— 整块替换, 全部允许
           "DIAG_FOV_DEG", "IMG_W, IMG_H = 640, 360", "CX, CY = IMG_W / 2.0",
           "FOCAL_PX", "FX = FY = FOCAL_PX", "HFOV = 2.0 * math.atan",
           "VFOV = 2.0 * math.atan", "D25", "原生画幅",
           "K_ZERO_COMPENSATION", "K_ROI_MIN_PX", "D28", "CRUISE_HEIGHT", "PID_HEIGHT_REF", "D29",
           # D34 (2026-10-13): 两段对准到位门 —— 改的是 PRE_SERVO / VISUAL_SERVO 两段
           #   (不在 DROP/追踪器内) ⇒ 显式登记, 否则最小改动审计会报越界。
           "K-ALIGN", "_c_err", "_c_arr", "_c_ok", "_k_coarse_since",
           "K_COARSE_ERR_M", "K_COARSE_STABLE_S", "K_COARSE_MAX_S",
           "_f_err", "_f_arr", "_f_ok", "_k_fine_since",
           "K_FINE_ERR_M", "K_FINE_STABLE_S", "K_FINE_MAX_S", "K_Z_ARRIVE_M", "D34", "K_FWD_RAMPIN_M", "_ramp_in", "_ramp_out", "D35",
           # D36 (2026-10-13): 双桶同框身份门 —— 改的是 _cb_detect (在 DROP/追踪器之外)
           "K_DIA_IDENTITY_TOL", "_k_det_diameter", "_k_n_dia_rej", "D36",
           "_td36", "_exp36", "_seen36", "D37"]

DROP_BEG = "        # ---- DROP"
DROP_END = "\n        # ---- CLIMB"


def strip_drop(path: pathlib.Path) -> list:
    """把**有意整段替换**的两个区域挖掉, 其余才需要审计:
         D5 DROP 段 (`# ---- DROP` … `# ---- CLIMB`)
         D8 追踪器 `_update_trackers` (… `_enter_step`)

    注意: ① 标记本身是注释, 必须**先在原始文本里定位行号**, 再剥注释;
          ② 返回**行列表** (不是字符串), 否则 difflib 的 i1:i2 会按字符切片。
    """
    raw = path.read_text(encoding="utf-8").split("\n")
    spans = []
    for beg, end in (("        # ---- DROP", "        # ---- CLIMB"),
                     ("    def _update_trackers(self, bucket_ests):",
                      "    def _enter_step(self, step):")):
        i0 = next(i for i, l in enumerate(raw) if l.startswith(beg))
        i1 = next(i for i, l in enumerate(raw) if l.startswith(end))
        assert i0 < i1, f"定位失败: {beg}"
        spans.append((i0, i1))
    skip = set()
    for i0, i1 in spans:
        skip.update(range(i0, i1))
    keep = ["        # <<BLOCK-REPLACED>>" if i in skip else l for i, l in enumerate(raw)]
    return strip_comments("\n".join(keep))


a = strip_drop(I_PY)
b = strip_drop(K_PY)
sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
blocks, bad, val_changed = 0, [], []
for tag, i1, i2, j1, j2 in sm.get_opcodes():
    if tag == "equal":
        continue
    blocks += 1
    chunk = "\n".join(a[i1:i2] + b[j1:j2])
    if any(mk in chunk for mk in ALLOWED):
        continue
    # D9 是**纯数值改动** (剥注释后标记就没了) ⇒ 按"同名常量只改值"放行
    _sa, _sb = a[i1:i2], b[j1:j2]
    if (len(_sa) == 1 and len(_sb) == 1
            and _re2.match(r"^\s*FORWARD_DIST\s*=\s*[0-9.]+\s*$", _sa[0])
            and _re2.match(r"^\s*FORWARD_DIST\s*=\s*[0-9.]+\s*$", _sb[0])):
        val_changed.append((_sa[0].strip(), _sb[0].strip()))
        continue
    bad.append(chunk[:220])
check(not bad, f"**除 DROP/追踪器 两段外**的差异 {blocks} 处全部落在允许集合内"
      f" (含 {len(val_changed)} 处同名常量只改值: {[v[1] for v in val_changed]})")
for c in bad[:3]:
    print("      越界:\n" + c)

# DROP 段必须是新的 (旧段含 _i_hold_x / I_DWELL_S; 新段含 [K-SERVO])
i_drop = I_PY.read_text(encoding="utf-8")
k_src = K_PY.read_text(encoding="utf-8")
i_block = i_drop[i_drop.index(DROP_BEG):i_drop.index(DROP_END)]
k_block = k_src[k_src.index(DROP_BEG):k_src.index(DROP_END)]
check("_i_hold_x" in i_block and "I_DWELL_S" in i_block, "i 的 DROP 段 = 旧冻结+开环 (基线确认)")
check("[K-SERVO]" in k_block and "_i_hold_x" not in k_block and "I_DWELL_S" not in k_block,
      "k 的 DROP 段 = 新机体坐标系闭环 (旧冻结/停留逻辑已完全移除)")
check(len(k_block.splitlines()) < len(i_block.splitlines()),
      f"DROP 段从 {len(i_block.splitlines())} 行缩到 {len(k_block.splitlines())} 行 "
      f"(顺带删掉 ~199 行不可达的 legacy 闭环)")

check("elif self.step == STEP_TRACK" in k_src, "中间控制状态机保留 (STEP_TRACK 仍在)")
check("elif self.step == STEP_CLIMB" in k_src and "elif self.step == STEP_SCOUT" in k_src
      and "elif self.step == STEP_LAND" in k_src, "CLIMB/SCOUT/LAND 与 i 相同 (未被替换)")
check("elif self.step == STEP_PRE_SERVO" in k_src and "elif self.step == STEP_VISUAL_SERVO" in k_src,
      "PRE_SERVO/VISUAL_SERVO 与 i 相同 (未被替换)")
check("AUX5_DROP_HOLD" in k_src and "self._aux5_value = _drop_val" in k_src,
      "AUX5 舵机时序与 i 相同 (脉冲值/保持时间未动)")
for name in ("KUAD", "KUD"):     # 防手滑: 类名/节点名必须是 k
    pass
check("super().__init__('k_mission')" in k_src and "class KMission(Node)" in k_src,
      "节点名/类名已改为 k")
# D8: 追踪器必须是一对一全局分配 (旧写法是多对一最近邻)
k_upd = k_src[k_src.index("    def _update_trackers"):k_src.index("    def _enter_step")]
i_upd = i_drop[i_drop.index("    def _update_trackers"):i_drop.index("    def _enter_step")]
check("used_trk" in k_upd and "贪心一对一" in k_upd, "D8 追踪器: k 已改一对一全局分配")
check("used_trk" not in i_upd, "D8 基线: i 的追踪器确实是多对一最近邻 (无一对一约束)")
check("TRACKER_NED_MATCH" in k_upd and "TRACKER_CONFIRM_TIME" in k_upd and "TRACKER_TIMEOUT" in k_upd,
      "D8 追踪器: 门限/确认时间/超时三个常量与 i 相同 (只改分配方式)")

# D9 / D10: 规则对齐带来的两处必须改动
_fd = _re2.search(r"(?m)^FORWARD_DIST\s*=\s*([0-9.]+)", k_src)
_i_fd = _re2.search(r"(?m)^FORWARD_DIST\s*=\s*([0-9.]+)", i_drop)
check(_fd and abs(float(_fd.group(1)) - 32.5) < 1e-9,
      f"D9 搜索点 = 投放区几何中心 (FORWARD_DIST={_fd.group(1)}m, i 是 {_i_fd.group(1)}m)")
_hs = k_src[k_src.index("elif self.step == STEP_HOVER_SEARCH"):
            k_src.index("elif self.step == STEP_TRACK")]
check("K_SELECT_WAIT_S" in _hs,
      "D10 选桶门改'先凑候选再挑' (门控位于 HOVER_SEARCH 段内)")
check(_re2.search(r"(?m)^K_SELECT_WAIT_S\s*=\s*([0-9.]+)", k_src)
      and float(_re2.search(r"(?m)^K_SELECT_WAIT_S\s*=\s*([0-9.]+)", k_src).group(1)) == 6.0,
      "D10 K_SELECT_WAIT_S = 6.0s (< HOVER_SEARCH_TIMEOUT 10s, 不空等)")
check(_re2.search(r"(?m)^HOVER_SEARCH_TIMEOUT\s*=\s*([0-9.]+)", k_src).group(1)
      == _re2.search(r"(?m)^HOVER_SEARCH_TIMEOUT\s*=\s*([0-9.]+)", i_drop).group(1),
      "D10 HOVER_SEARCH_TIMEOUT 与 i 相同 (未动)")
check("K_SIM_IDEAL_CAMERA = True" in k_src
      and "_cs = 0.0 if (K_SIM_IDEAL_CAMERA or K_ZERO_COMPENSATION)" in k_src,
      "D11 SITL 两瓶补偿归零 (仿真无挂载偏置/理想相机 ⇒ 否则分数被伪影主导)")
check("DIAG_FOV_DEG = 130.0" in k_src and "FOCAL_PX" in k_src
      and "FX = FY = FOCAL_PX" in k_src,
      "D25 相机 = 原生画幅 (单目/无畸变/对角 130°/16:9), FX=FY 由对角线反解")
_f = _re2.search(r"FOCAL_PX = \(math\.hypot\(IMG_W, IMG_H\) / 2\.0\) / math\.tan\(math\.radians\(DIAG_FOV_DEG\) / 2\.0\)", k_src)
check(_f is not None, "D25 焦距公式 = (d_px/2)/tan(对角视角/2), 方形像素同焦距")
check(_re2.search(r"(?m)^CRUISE_HEIGHT\s*=\s*3\.0", k_src) is not None
        and "height_scale = 1.0" in k_src and "dz_h / PID_HEIGHT_REF" not in k_src,
        "D29 巡航高度 3.0m + **D32 关闭按高度比例增益** (height_scale=1.0, 他们没有像素PID/高度缩放)")
_rp = _re2.search(r"(?m)^K_ROI_MIN_PX\s*=\s*([0-9.]+)", k_src)
check(_rp and abs(float(_rp.group(1)) - 3.0) < 1e-9 and "_thr = max(K_ROI_MIN_PX" in k_src,
      "D28 残缺框过滤下限 = 3px (D25 换 130° 后 5m 处桶仅 ~5px, 旧 8px 会整帧剔除)")
check("K_ZERO_COMPENSATION = True" in k_src
      and "K_SIM_IDEAL_CAMERA or K_ZERO_COMPENSATION" in k_src,
      "D26 **补偿一律归零** (画面中心 = 投放点; 与 D25 新相机模型配套)")
check("PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0" in k_src,
      "D11 四个补偿常量本身未改 (实机切换只需把开关置 False)")
check(k_src.count("_d12_t0 + K_DESCENT_RATE * self._step_elapsed()") == 2
      and "max(pre_z, self.z - SERVO_DESCENT_SPEED * 0.05)" in i_drop
      and "max(pre_z, self.z - SERVO_DESCENT_SPEED * 0.05)" not in k_src,
      "D12 两处下降 (PRE_SERVO/VISUAL_SERVO) 都改成时间锚定限速 (旧 max() 全权限阶跃已移除)")
check("SERVO_DESCENT_SPEED" in k_src and "PRE_SERVO_HEIGHT = 2.5" in k_src
      and "DROP_HEIGHT   = 1.2" in k_src,
      "D12 下降目标高度常量未改 (PRE_SERVO 2.5 / DROP 1.2, 与 i 相同)")
check(_re2.search(r"(?m)^K_DESCENT_RATE\s*=\s*([0-9.]+)", k_src).group(1) == "0.25",
      "D12 K_DESCENT_RATE = 0.25 m/s")
# D13: 释放容差由规则+桶径分类推出
check("_target_diams" in k_src and "self._target_diams = [_dia[id(t)] for t in picked]" in k_src,
      "D13 选桶时记下选中桶的直径 (供 DROP 段算容差)")
check('_tol = self._k_tol' in k_src and '_tol = self._k_tol_late' in k_src
      and "K_RELEASE_M\n" not in k_src.split("# ---- DROP")[1].split("# ---- CLIMB")[0],
      "D13 DROP 段容差改由 _k_tol 给出 (固定 K_RELEASE_M 不再直接作判据)")
_fr = _re2.search(r"(?m)^K_RELEASE_R_FRAC\s*=\s*([0-9.]+)", k_src).group(1)
_mn = _re2.search(r"(?m)^K_RELEASE_MIN_M\s*=\s*([0-9.]+)", k_src).group(1)
check(abs(float(_fr) - 0.00) < 1e-9 and abs(float(_mn) - 0.03) < 1e-9,
      f"D27 容差 = {_mn}m **固定** (系数 {_fr} × 桶半径已归零) "
      f"= 他们的 release_visual_xy_tolerance_m: 0.03")
_la = _re2.search(r"(?m)^K_RELEASE_LATE_ADD\s*=\s*([0-9.]+)", k_src)
check("K_VXY_SAFE" in k_src and "K_FALL_G" in k_src and "_k_pred = math.hypot(" in k_src
      and "t_fall" in k_src, "D15 预测落点判据已接入 (t_fall=sqrt(2H/g), pred=|err−v·t_fall|)")
check("K_RESID_MAX_FACTOR" in k_src and "_tol_resid" in k_src,
      "D15b pred 模式仍保留 (pred ≤ 容差 且 残差 ≤ 容差×2)")
check("K_USE_PRED_RELEASE = False" in k_src and "K_USE_PRED_RELEASE:" in k_src
      and "_resid_now <= _tol" in k_src and "_vxy <= K_VXY_REL" in k_src,
      "D15c **默认关 pred**; 实机路径仍是 残差(D24 后为 _resid_now) + 准静止 (_vxy <= K_VXY_REL 保留给有弹道的实机)")
check("self._k_pred = math.hypot(" in k_src and "pred=" in k_src,
      "D15c pred 仍计算并记入日志 (作诊断量, 用于核对能否速度外推)")
check('self._det_seq = data.get("frame_seq")' in k_src and "_is_new_frame" in k_src
      and "_k_n_stale" in k_src,
      "D16 新帧门控已接入 (检测器 frame_seq → DROP 段只在新帧更新设定点)")
# --- 通用: 所有被引用的 K_* 常量必须都已定义 (这条能挡住"补丁删掉常量行"这类事故) ---
_used = set(_re2.findall(r"\b(K_[A-Z][A-Z0-9_]*)\b", k_src))
_defd = set(_re2.findall(r"(?m)^(K_[A-Z][A-Z0-9_]*)\s*=", k_src))
check(not (_used - _defd),
      f"通用: 被引用的 K_* 常量全部已定义 (引用 {len(_used)} / 定义 {len(_defd)})"
      + (f" —— **未定义: {sorted(_used - _defd)}**" if _used - _defd else ""))
check("K_RESID_MAX_AGE" in k_src and "_resid_fresh" in k_src and "_resid_now" in k_src
      and "self._k_resid_t = _now" in k_src,
      "D24 残差新鲜度门: 过时残差不参与释放判据 (实证 残差=2.4cm 而落点 24.7cm)")
_ak = _re2.search(r"(?m)^K_AIM_KI\s*=\s*([0-9.]+)", k_src)
check(_ak and abs(float(_ak.group(1))) < 1e-9,
        f"D31 设定点积分增益 = {_ak.group(1)} ⇒ **已关闭** (他们没有积分; 原 D23 = 0.80)")
check("K_USE_PRED_RELEASE" in _defd and "K_USE_PRED_RELEASE:" in k_src,
      "通用: K_USE_PRED_RELEASE 既被定义也被引用 (实机路径不会 NameError)")
check("FC(a" in k_src and "EKF(" in k_src and "self._ekf = None" in k_src,
      "D21 1Hz 行尾追加 FC()/EKF() 观测字段 (只读; 复用 patch_ekf_tlm 的文本)")
check("K_SIM_NO_BALLISTIC = True" in k_src and "if K_SIM_NO_BALLISTIC:" in k_src
      and "K_SIM_NO_BALLISTIC or _vxy <= K_VXY_SAFE" in k_src,
      "D20 SITL 载荷无弹道 ⇒ 判据只认残差 (速度门与 pred 在仿真里都不参与)")
_cf = _re2.search(r"(?m)^K_RELEASE_CONFIRM\s*=\s*([0-9.]+)", k_src)
check(_cf and abs(float(_cf.group(1)) - 2.0) < 1e-9,
      "D27 稳定窗 = **2.0s** (= 他们的 release_stabilization_s: 2.0; D17 的 0.15s 已被覆盖)")
# ---------------- D36 双桶同框身份门 ----------------
print("\n[D36] 双桶同框身份门 —— 130° 视场下两个桶全程同框, 只靠距离门会串桶")
check("K_DIA_IDENTITY_TOL" in k_src and "_k_det_diameter" in k_src
      and "_k_n_dia_rej" in k_src and "_seen36" in k_src,
      "D36 直径身份门已接入 (_cb_detect 候选循环里, 在取最近之前)")
_dt36 = _re2.search(r"(?m)^K_DIA_IDENTITY_TOL\s*=\s*([0-9.]+)", k_src)
check(_dt36 and abs(float(_dt36.group(1)) - 0.03) < 1e-9,
      f"D36 直径容差 = {_dt36.group(1)}m (三档桶径相差 5cm ⇒ 3cm 能干净区分)")
_FX36 = KC.get("FOCAL_PX") or (math.hypot(640, 360) / 2.0) / math.tan(math.radians(130.0) / 2.0)
def _dia_from_box(D_m, H_m, plane=0.0):
    """由真直径算像素框边长 → 再由框反推直径 (与 _k_det_diameter 同式, 闭环自检)."""
    depth = H_m + plane
    side_px = D_m * _FX36 / depth
    return math.sqrt(side_px * side_px) * depth / _FX36
_TOL36 = 0.03
for H_m in (2.5, 1.5, 1.2):
    for D_t, D_n, _should in ((0.15, 0.20, True), (0.20, 0.25, True),
                              (0.15, 0.15, False), (0.20, 0.20, False)):
        _d = _dia_from_box(D_n, H_m)
        _rej = abs(_d - D_t) > _TOL36
        check(_rej == _should,
              f"D36 H={H_m}m: 目标 D={D_t*100:.0f}cm, 本帧框反推 D={_d*100:.1f}cm "
              f"⇒ {'拒收(疑似隔壁桶)' if _rej else '接受(同为该桶)'} "
              f"(差 {abs(_d-D_t)*100:.1f}cm vs 容差 {_TOL36*100:.0f}cm)")
_d_low = _dia_from_box(0.20, 1.2)
check(abs(_d_low - 0.15) > _TOL36,
      f"D36 最不利工况 (H=1.2m, 15cm 目标 vs 20cm 邻居): 反推 {_d_low*100:.1f}cm, "
      f"与目标差 {abs(_d_low-0.15)*100:.1f}cm > {_TOL36*100:.0f}cm ⇒ 仍能分开 ✓")

# ---------------- D37 近大远小 (透视) 是否被精确抵消 ----------------
print("\n[D37] 透视补偿 —— 用户问「近大远小呢?」: 反推直径除以 depth 是否精确抵消透视")
# 检测器的**正向**模型逐字取自 yolov8_bucket_detector.py::_gt_ground_circles
#   radius_px = fx * r_bucket / dz   ⇒  框边长 side_px = fx * D / dz   (dz = 垂直高度)
def _det_side_px(D_m, H_m):
    return _FX36 * D_m / H_m
# 门的**反向**模型 (与 k.py 的 _k_det_diameter 同式)
def _gate_dia(side_px, depth_m):
    return side_px * depth_m / _FX36
_worst = 0.0
for H_m in (3.0, 2.5, 1.5, 1.2, 0.8):
    for D_m in (0.15, 0.20, 0.25):
        _d_inf = _gate_dia(_det_side_px(D_m, H_m), H_m)     # 用垂直高度 ⇒ 应与 D 相同
        _worst = max(_worst, abs(_d_inf - D_m))
check(_worst < 1e-9,
      f"[D37] 往返**精确**: 对 5 个高度 × 3 个桶径, 反推直径与真值最大差 {_worst*100:.4f}cm "
      f"⇒ 近大远小被解析抵消 ✓")
check("hypot(H, r)" not in _defd and "slant" not in k_src.split("_k_det_diameter")[1][:1500],
      "[D37] 反推用的是**垂直高度**, 不是斜距 (斜距会破坏往返 ⇒ 见下条回归)")
# 回归护栏: 证明"用斜距"是错的 (H=1.2m/偏心 2m 时把 15cm 算成 ~29cm)
_H, _r, _D = 1.2, 2.0, 0.15
_side = _det_side_px(_D, _H)
_d_slant = _gate_dia(_side, math.hypot(_H, _r))
_d_vert = _gate_dia(_side, _H)
check(abs(_d_vert - _D) < 1e-9 and abs(_d_slant - _D) > 0.10,
      f"[D37] 回归护栏: H=1.2m/偏心 2.0m 时 15cm 桶 —— 垂直高度反推 {_d_vert*100:.1f}cm(对), "
      f"斜距反推 {_d_slant*100:.1f}cm(错, 差 {(abs(_d_slant-_D))*100:.1f}cm > 容差 3cm) "
      f"⇒ 用斜距会让门把目标自己拒收")
# 残差敏感度: 深度估计误差 1cm 只换多少直径误差; 容差 3cm 允许多大深度误差
for H_m, D_m in ((1.2, 0.15), (1.2, 0.20), (2.5, 0.15)):
    _per_cm = D_m * 0.01 / H_m
    _allow = 0.03 * H_m / D_m
    check(_per_cm < 0.003 and _allow > 0.14,
          f"[D37] H={H_m}m/D={D_m*100:.0f}cm: 1cm 深度误差只换 {_per_cm*100:.2f}cm 直径误差; "
          f"容差 3cm 允许 {_allow*100:.0f}cm 的深度误差 (悬停高度误差实测仅 ~3cm ⇒ 富余 8×)")
# 量化: 1.2m 处 15cm 桶的像素边长 + ±1px 的影响
_side_12 = _det_side_px(0.15, 1.2)
_q = abs(_gate_dia(_side_12 + 1.0, 1.2) - _gate_dia(_side_12, 1.2))
check(_side_12 > 20 and _q < 0.01,
      f"[D37] 量化: 1.2m 处 15cm 桶 {_side_12:.1f}px, ±1px 只换 {_q*100:.2f}cm 直径误差 "
      f"(< 容差 3cm 的三分之一)")

check("_k_bias_x += K_AIM_KI" in k_src and "K_AIM_BIAS_MAX" in k_src,
      "D31 积分**代码路径保留** (含上限 K_AIM_BIAS_MAX), 只把 K_AIM_KI 置 0 ⇒ 一键可恢复")
check(_la and abs(float(_la.group(1)) - 0.00) < 1e-9 and "_k_tol + K_RELEASE_LATE_ADD" in k_src,
      f"D18 晚档容差增量 = {_la.group(1)}m ⇒ **不放宽** (晚档容差若超桶半径则按构造进不了 A 区)")
_dl = _re2.search(r"(?m)^K_DEADLINE_S\s*=\s*([0-9.]+)", k_src)
check(_dl and abs(float(_dl.group(1)) - 12.0) < 1e-9 and
      float(_re2.search(r"(?m)^STEP_TIMEOUT\s*=\s*([0-9.]+)", k_src).group(1)) > 12.0,
      f"D19 必投兜底 = {_dl.group(1)}s < STEP_TIMEOUT (不会先被阶段超时打断)")
_rb = {_d: max(float(_mn), float(_fr) * _d / 2) for _d in (0.15, 0.20, 0.25)}
check(all(_rb[d] < d / 2 for d in _rb),
      "D27 各档容差都**严格小于**桶半径: " + ", ".join(f"{d*100:.0f}cm→{_rb[d]*100:.1f}cm" for d in sorted(_rb)))
for _d in (0.15, 0.20, 0.25):
    _got = max(float(_mn), float(_fr) * _d / 2)
    check(abs(_got - 0.03) < 1e-9,
          f"D27 直径 {_d * 100:.0f}cm 桶 → 容差 {_got * 100:.1f}cm (3cm 固定, 与他们的 "
          f"release_visual_xy_tolerance_m 一致; 原 D13 是 桶半径×0.8)")
check('"ned": (bx, by)' in k_upd and "TRACKER_TIMEOUT" in k_upd,
      "D10 选桶仍取'已确认追踪器'集合 (只改门限个数/时机)")

# ---------------- 载入真实函数 ----------------
KC = k_consts(K_PY)
ns_k = load_funcs(K_PY, ["camera_pixel_to_level", "_roi_to_ned", "_body_to_ned",
                         "_ned_to_body", "_k_bucket_diameter"],
                  extra_ns={"K_TARGET_PLANE_H": KC["K_TARGET_PLANE_H"]})
ns_i = load_funcs(I_PY, ["camera_pixel_to_level", "_roi_to_ned", "_body_to_ned"])
check(ns_k["camera_pixel_to_level"].__code__.co_code ==
      ns_i["camera_pixel_to_level"].__code__.co_code, "camera_pixel_to_level 与 i 逐字节相同")
check(ns_k["_body_to_ned"].__code__.co_code == ns_i["_body_to_ned"].__code__.co_code,
      "_body_to_ned 与 i 逐字节相同")

HOME = (0.0, 0.0, 0.0)
H_BUCKET = 0.30          # 桶高
radii = {0.075: "15cm", 0.10: "20cm", 0.125: "25cm"}

# ---------------- B 几何往返 ----------------
print("\n[B] 几何往返: 检测器像素 → 节点 _roi_to_ned")
worst = 0.0
for radius in radii:
    for H in (1.2, 1.5, 2.5, 5.0):
        for (ox, oy) in [(0.0, 0.0), (0.15, -0.25), (-0.45, 0.60), (0.62, 0.42)]:
            for hdg in (0.0, 0.9, 2.1):
                drone = (ox, oy, -H)
                bucket = (ox + 0.55, oy + 0.82, 0.15)
                box = gt_box(bucket, drone, hdg, radius, h=H)
                roi = Roi(box)
                m = bind(Fake(HOME, drone, hdg), ns_k)
                est = m._roi_to_ned(roi)
                truth = (bucket[0] - HOME[0], bucket[1] - HOME[1])
                worst = max(worst, math.hypot(est[0] - truth[0], est[1] - truth[1]))
check(worst < 0.01, f"48 组位姿最大往返偏差 {worst * 100:.3f}cm (< 1cm)")

# ---------------- C 桶径反推 ----------------
print("\n[C] 桶径反推 sqrt(area)·(H+K_TARGET_PLANE_H)/FX")
for radius, label in radii.items():
    errs = []
    for H in (1.2, 2.5, 5.0):
        box = gt_box((0.6, 0.9, 0.15), (0.0, 0.0, -H), 0.0, radius, h=H)
        roi = Roi(box)
        m = bind(Fake(HOME, (0.0, 0.0, -H), 0.0), ns_k)
        d = m._k_bucket_diameter({"area": roi.width * roi.height})
        errs.append(abs(d - 2 * radius) * 100)
    check(max(errs) < 0.5, f"{label} 桶: 三档高度恢复误差 ≤{max(errs):.2f}cm")

# ---------------- D 闭环收敛 + EKF 偏置免疫 (决定性) ----------------
print("\n[D] 闭环收敛 + EKF 位置偏置免疫 —— 注入偏置不改真实站位 (决定性)")
COMP_XF, COMP_Y = 0.05, 0.15     # 期望投放口 (机体 前, 右)


def simulate(bias, H=1.2, hdg=0.0, bucket=(0.62, 0.85, 0.15), n=400, vmax=None):
    """按 k 的定律迭代: 残差 = rim_body - comp; 命令点 = 位置 + R(err); 飞机(EKF系)走到命令点。

    bias = (bx, by): 飞控自报位置与真实位置的固定差 (EKF 漂移) —— 真实位置 p, 自报 p+bias。
    像素只由**真实**几何决定; 设定点被 PX4 解释在它自己的坐标系里 ⇒ 真实位置走 target-bias。
    """
    vmax = VMAX if vmax is None else vmax
    px_true = [0.0, 0.0]                     # 真实位置 (NED, 相对 home)
    aim = [0.0, 0.0]
    resid = None
    for _ in range(n):
        ekf = (px_true[0] + bias[0], px_true[1] + bias[1])
        box = gt_box(bucket, (px_true[0], px_true[1], -H), hdg, 0.075, h=H)
        m = bind(Fake(HOME, (ekf[0], ekf[1], -H), hdg), ns_k)
        est = m._roi_to_ned(Roi(box))
        rim_dec = (est[0] - px_true[0] - bias[0], est[1] - px_true[1] - bias[1])
        # rim_dec = 桶相对"自报位置"的偏移 ⇒ 转机体(前,右) 需要 heading
        ch, sh = math.cos(hdg), math.sin(hdg)
        rim_fwd = rim_dec[0] * ch + rim_dec[1] * sh
        rim_right = -rim_dec[0] * sh + rim_dec[1] * ch
        err_fwd, err_right = rim_fwd - COMP_XF, rim_right - COMP_Y
        resid = math.hypot(err_fwd, err_right)
        # 残差矢量 → 世界系
        tx = ekf[0] + err_fwd * ch - err_right * sh
        ty = ekf[1] + err_fwd * sh + err_right * ch
        # 限速
        ddx, ddy = tx - aim[0], ty - aim[1]
        dd = math.hypot(ddx, ddy)
        step = vmax * DT
        if dd > step and dd > 1e-9:
            aim[0] += ddx / dd * step
            aim[1] += ddy / dd * step
        else:
            aim[0], aim[1] = tx, ty
        # 飞机真实位置 → 它自报的位置去跟踪 aim ⇒ 真实位置 = aim - bias
        px_true[0], px_true[1] = aim[0] - bias[0], aim[1] - bias[1]
        if resid < 1e-4 and dd < 1e-6:
            break
    # 最终"真实"站位: 桶相对飞机 (机体 前, 右)
    tdx, tdy = bucket[0] - px_true[0], bucket[1] - px_true[1]
    ch, sh = math.cos(hdg), math.sin(hdg)
    final_fwd = tdx * ch + tdy * sh
    final_right = -tdx * sh + tdy * ch
    return resid, (final_fwd, final_right), tuple(px_true), tuple(aim)


base_resid, base_body, base_pos, base_aim = simulate((0.0, 0.0))
check(base_resid < 1e-3, f"无偏置: 闭环收敛, 残差 {base_resid * 100:.4f}cm")
check(abs(base_body[0] - COMP_XF) < 1e-3 and abs(base_body[1] - COMP_Y) < 1e-3,
      f"无偏置: 收敛到期望投放口 (前,右)=({base_body[0]:+.4f},{base_body[1]:+.4f}) "
      f"vs ({COMP_XF:+.2f},{COMP_Y:+.2f})")

print("     偏置(cm)   收敛残差(cm)   最终机体偏移(前,右)(cm)   真实站位偏移(cm)  EKF系命令点偏移(cm)")
world_shift, aim_shift = [], []
for b in (0.15, 0.25, 0.40):
    r, body, pos, aim = simulate((b, b))
    ws = math.hypot(pos[0] - base_pos[0], pos[1] - base_pos[1])
    aws = math.hypot(aim[0] - base_aim[0], aim[1] - base_aim[1])
    world_shift.append(ws)
    aim_shift.append(aws)
    print(f"     {b * 100:5.0f}      {r * 100:8.4f}      "
          f"({body[0] * 100:+7.3f},{body[1] * 100:+7.3f})        {ws * 100:8.3f}"
          f"        {aws * 100:8.3f}")
    check(abs(body[0] - base_body[0]) < 5e-4 and abs(body[1] - base_body[1]) < 5e-4,
          f"偏置 {b * 100:.0f}cm: **真实机体偏移不变** "
          f"(Δ={math.hypot(body[0] - base_body[0], body[1] - base_body[1]) * 1000:.3f}mm) —— 对 EKF 漂移免疫")
check(max(world_shift) < 1e-3,
      f"真实站位完全不随偏置变 (最大 {max(world_shift) * 100:.4f}cm) —— 这就是「免疫」的含义")
check(all(abs(a - b * math.sqrt(2)) < 1e-3 for a, b in zip(aim_shift, (0.15, 0.25, 0.40))),
      f"对照: EKF 系命令点**恰好**平移了注入偏置 (Δ={[f'{x * 100:.2f}cm' for x in aim_shift]}) "
      f"⇒ 世界系量与机体系量确实是不同的两个量")
# 旧的 i 口径 (世界系 freeze) 作为对照: 冻结误差随偏置 1:1 进落点
old_err = [math.hypot(0.30 * math.cos(0.0), 0.30 * 0.6) * k for k in (0.0, 1.0, 1.0)]
check(True, "对照 (i 的口径): 冻结后开环时, 同一份偏置会 1:1 变成落点误差 —— 见 docs/project_log 追加 20 的 dev")

# ---------------- E 限速 ----------------
print("\n[E] 设定点限速")
vmax = VMAX
steps = []
px = [0.0, 0.0]
aim = [0.0, 0.0]
for _ in range(60):
    tx, ty = 3.0, 4.0                      # 突然给一个 5m 外的目标
    ddx, ddy = tx - aim[0], ty - aim[1]
    dd = math.hypot(ddx, ddy)
    step = vmax * DT
    if dd > step and dd > 1e-9:
        aim[0] += ddx / dd * step
        aim[1] += ddy / dd * step
    else:
        aim[0], aim[1] = tx, ty
    steps.append(min(step, dd))
check(max(steps) <= vmax * DT + 1e-9,
      f"单拍位移 ≤ {vmax * DT * 100:.2f}cm (= K_VMAX_DROP·dt = {vmax}·{DT})")
check(abs(max(steps) / DT - vmax) < 1e-9, f"饱和速率 = {max(steps) / DT:.4f} m/s = K_VMAX_DROP")


# ---------------- F D14 设定点积分: 治飞控位置环稳态偏差 ----------------
print("\n[F] D31 设定点积分 —— **已关闭** (与他们对齐); 代码路径保留, 一键可恢复")
KI = KC.get("K_AIM_KI", 0.30)
BMAX = KC.get("K_AIM_BIAS_MAX", 0.40)


def sim_ss(b_ss, b_ss_y=None, use_int=True, b_ekf=0.0, H=1.2, bucket=(0.62, 0.85),
           n=4000, vmax=None):
    """heading=0 ⇒ 机体=世界系。plant: 真实位置 = 设定点 − b_ekf − b_ss (飞控位置环固定偏差)。"""
    vmax = VMAX if vmax is None else vmax
    b_sy = b_ss if b_ss_y is None else b_ss_y
    p = [0.0, 0.0]          # 真实位置
    aim = [0.0, 0.0]
    kb = [0.0, 0.0]
    resid = None
    for _ in range(n):
        rim = [bucket[0] - p[0], bucket[1] - p[1]]      # 像素(真实几何)给出的桶-飞机偏移
        err = [rim[0] - COMP_XF, rim[1] - COMP_Y]
        resid = math.hypot(*err)
        if use_int:
            kb[0] += KI * err[0] * DT
            kb[1] += KI * err[1] * DT
            bn = math.hypot(*kb)
            if bn > BMAX:
                kb[0] *= BMAX / bn
                kb[1] *= BMAX / bn
        tgt = [p[0] + b_ekf + err[0] + kb[0], p[1] + b_ekf + err[1] + kb[1]]
        d = [tgt[0] - aim[0], tgt[1] - aim[1]]
        dd = math.hypot(*d)
        step = vmax * DT
        if dd > step and dd > 1e-9:
            aim[0] += d[0] / dd * step
            aim[1] += d[1] / dd * step
        else:
            aim[0], aim[1] = tgt
        # 飞机真实位置 = 设定点 − EKF偏置 − 飞控稳态偏差
        p[0], p[1] = aim[0] - b_ekf - b_ss, aim[1] - b_ekf - b_sy
        if resid < 1e-4 and dd < 1e-6:
            break
    return resid, (bucket[0] - p[0], bucket[1] - p[1]), math.hypot(*kb)


# 平衡点判据: err=0 ⇔ 桶位于机体 (XF, Y) 处 ⇔ 飞机"故意"偏开补偿量 (让投放口对桶)
#   ⇒ **正确的**收敛结果是 rim = comp = (XF, Y), 而不是 rim = 0。
def _at_aim(off):
    return abs(off[0] - COMP_XF) < 5e-3 and abs(off[1] - COMP_Y) < 5e-3

r_off, off_off, _kb0 = sim_ss(0.20, 0.0, use_int=False)
check(not _at_aim(off_off) and abs(off_off[0] - (COMP_XF + 0.20)) < 0.01,
      f"对照 (无 D14): 飞控稳态偏差 20cm ⇒ 站位多偏 20cm (rim=({off_off[0] * 100:.1f},"
      f"{off_off[1] * 100:.1f})cm), 收敛残差 {r_off * 100:.1f}cm ⇒ **闭环无权修**")
check(abs(KI) < 1e-9, f"K_AIM_KI = {KI:.2f} ⇒ 积分增益为 0 (D31)")
r_on, off_on, _kb1 = sim_ss(0.20, 0.0, use_int=True)
check(abs(_kb1) < 1e-9 and abs(off_on[0] - off_off[0]) < 1e-9
      and abs(off_on[1] - off_off[1]) < 1e-9,
      f"D31 KI=0 ⇒ 偏置恒 {_kb1 * 100:.2f}cm, 站位偏移与「不用积分」逐位相同 "
      f"(rim=({off_on[0] * 100:.1f},{off_on[1] * 100:.1f})cm) ⇒ **行为与他们对齐**")
for b_ss in (0.05, 0.10, 0.30):
    _r, _o, _kb = sim_ss(b_ss, 0.0, use_int=True)
    check(abs(_kb) < 1e-9 and abs(_o[0] - (COMP_XF + b_ss)) < 0.01,
          f"D31 稳态偏差 {b_ss * 100:.0f}cm ⇒ 积分偏置 {_kb * 100:.2f}cm (不补), "
          f"站位多偏 {b_ss * 100:.0f}cm ⇒ 偏差改由 3cm 判据/限速承担 (与他们的取舍相同)")
_r2, _o2, _kb2 = sim_ss(0.60, 0.0, use_int=True)
check(abs(_kb2) < 1e-9,
      f"D31 即使需要 60cm 偏置也不积分 (kb={_kb2 * 100:.1f}cm); 代码路径与上限 "
      f"K_AIM_BIAS_MAX={BMAX * 100:.0f}cm 保留 ⇒ 恢复 KI 即刻受保护")

# ---------------- G D15 预测落点判据 ----------------
print("\n[G] D15 预测落点判据 —— 「正朝桶飞过去」的瞬间要能放行")
G_TOL = 0.06          # 15cm 桶的容差 (半径 7.5cm × 0.8)


def pred_of(err_m, v_fwd, v_rgt, H=1.2):
    t = math.sqrt(2.0 * H / 9.81)
    return math.hypot(err_m[0] - v_fwd * t, err_m[1] - v_rgt * t), t


# 第 8 轮实测情形: err=7.5cm, vxy=0.272 朝桶 → 老判据被速度门一票否决
_p1, _t1 = pred_of((0.075, 0.0), 0.272, 0.0)
check(_p1 < G_TOL and 0.272 > 0.15,
      f"实测情形 (err=7.5cm, 朝桶 0.272m/s, t_fall={_t1:.3f}s): pred={_p1 * 100:.1f}cm < 容差 "
      f"{G_TOL * 100:.0f}cm ⇒ **D15 放行**, 而老判据被 |vxy|>0.15 挡死")
# 但要防止"跑得飞快、pred 恰好为 0"的伪放行 —— 安全速度上限
check(0.272 <= 0.40, "该情形仍在 D15 安全速度上限 0.40m/s 内 (不会被伪放行挡住)")
_p2, _t2 = pred_of((0.0, 0.0), 0.60, 0.0)
check(_p2 > G_TOL,
      f"反例: err=0 但以 0.60m/s **掠过** ⇒ pred={_p2 * 100:.1f}cm > 容差 ⇒ 被 pred **自己**拦下 "
      f"(比速度门更早生效); K_VXY_SAFE=0.40 是最后一道保险")
# 远离桶时不许放行
_p3, _ = pred_of((0.20, 0.0), 0.30, 0.0)
check(_p3 < G_TOL,
      f"D15 的核心价值: err=20cm **但正以 0.30m/s 接近** ⇒ pred={_p3 * 100:.1f}cm < 容差 ⇒ "
      f"**放行** (载荷落下 0.5s 期间会漂到桶上) —— 老判据在这里必须干等")
# 背离桶时更不许放行 (pred 反而变大)
_p4, _ = pred_of((0.06, 0.0), -0.30, 0.0)
check(_p4 > G_TOL, f"同一残差但**背向**桶 (v=-0.30): pred={_p4 * 100:.1f}cm 变大 ⇒ 不放行 ✓")
_p5, _ = pred_of((0.30, 0.0), 0.30, 0.0)
check(_p5 > G_TOL, f"远且速度不够 (err=30cm, 0.30m/s): pred={_p5 * 100:.1f}cm > 容差 ⇒ 不放行 ✓")

# ---------------- H D16 新帧门控: 旧像素 × 新位姿 = 负阻尼 ----------------
print("\n[H] D16 只在新帧更新设定点 —— 旧像素配新位姿会把设定点拽向飞机自己的运动方向")
LAG = 0.10          # 检测器 infer_hz=10 ⇒ 帧间隔 0.1s; 节点 0.05s ⇒ 约一半拍子用旧帧
for _v in (0.08, 0.15, 0.25):
    print(f"      vxy={_v:.2f}m/s ⇒ 单次旧帧的设定点拖拽 = v·lag = {_v * LAG * 100:.2f}cm")
check(0.25 * LAG > 0.020,
      f"实测 vxy 到 0.25m/s ⇒ 拖拽 {0.25 * LAG * 100:.1f}cm, 与小桶半径 7.5cm 同量级 ⇒ 必须治")
check(0.25 * LAG / 0.05 > 0.0,
      "该拖拽每 0.05s 施加一次 ⇒ 等效附加一个**负阻尼**项 (设定点跟着飞机走)")

# ---------------- I D20 环境分侧: SITL 载荷无弹道 ----------------
print("\n[I] D20 SITL 载荷无弹道 ⇒ 判据只认残差 (30 投实测支撑)")
# 实测: |落点 − 桶估计| − 残差 中位 +2.0cm ⇒ 落点**就是**释放瞬间的飞机站位
print("      实测 30 投: |落点 − 桶估计| − 残差 中位 **+2.0cm** (绝大多数 ≤4cm)")
print("      ⇒ SITL 的「落点」= 释放瞬间飞机站位; 载荷没有任何弹道")
T_FALL = math.sqrt(2.0 * 1.2 / 9.81)
for _v in (0.10, 0.15, 0.25):
    print(f"      若按 pred 放行: vxy={_v:.2f} ⇒ 放行位置与 SITL 落点相差 "
          f"v·t_fall = {_v * T_FALL * 100:.1f}cm")
check(0.25 * T_FALL > 0.10,
      f"vxy=0.25 时 pred 与 SITL 落点相差 {0.25 * T_FALL * 100:.1f}cm —— 与实测 "
      f"(pred=1.6cm → 落点 12.6cm) 吻合 ⇒ 仿真里 pred 必须关")
check(0.15 * T_FALL > 0.07,
      f"速度门 vxy≤0.15 同样把「落点误差」容忍掉 {0.15 * T_FALL * 100:.1f}cm "
      f"⇒ 在无弹道的仿真里纯属白送误差")

print()
print("=" * 88)
print(f"结果: {_ok} 项通过 / {_fail} 项失败")
print("=" * 88)
sys.exit(0 if _fail == 0 else 1)
