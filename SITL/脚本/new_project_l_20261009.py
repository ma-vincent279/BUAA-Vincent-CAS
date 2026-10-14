#!/usr/bin/env python3
"""生成子项目 l = i + 【P1: 目标平面参数】 + 【P2: 机体坐标系释放闭环】

背景 (2026-10-09)
-----------------
P0 已让 SITL 能看见"目标平面选错"(仿真检测器 `project_mode=silhouette`)。离线验收
`scripts/test_silhouette_projection_20261009.py` [E] 节量出: 只把任务节点的求交平面从
"起飞原点平面"改成"桶口平面"(SITL 为 +0.455m), H=1.2m 处残差 **5.74cm → 0.33cm (17×)**。

第 1 步回放 (`scripts/analyze_freeze_offset_20261009.py`) 同时给出两条**必须一起带走**的约束:
  · ρ_est (冻结离轴距离) 中位 **23.6cm** —— i 的"一次性修正"其实是一次 ~24cm 平移;
  · 但平面项在实机上**不是主导项**: 逐点反解 h 的散布 MAD 0.5~1.1m (h 必须是硬件常数)
    ⇒ 落点由 平面项 + 姿态项 + 弹道漂移 + comp 残余 + dev **多项目同量级叠加**。
  ⇒ 所以本节点的两个开关**默认全关**, 且 P1 的收益必须按"SITL 几何"读, 不能当实机承诺。

改动 (相对 i.py, 全部是【新增开关 + 默认关】, 关掉时行为与 i 逐位相同)
--------------------------------------------------------------------
P1  `TARGET_PLANE_H` (默认 0.0): 目标平面在"起飞原点平面"下方多少米 (= 离相机更远多少)。
    SITL 桶口 = 0.455 (桶顶 NED z=0.455, home_z≈0); 实机需**静态标定**:
    h = (落地时相机离地高) − (桶口离地高)。
    作用于 4 处共用相机模型:
      ① `_roi_to_ned`  (像素 → NED)
      ② `_ned_to_expected_pixel` (NED → 期望像素, 身份锁定用)
      ③④ 两处 `pixel_offset` 换算 (俯冲段 / DROP 段)

P2  `L_BODY_GATE_M` (默认 0.0 = 关): **机体坐标系**释放闭环门。
    残差 = |观测到的桶 在机体(前/右) 的偏移 − 期望的投放口偏移|,
    其中期望偏移 = (−XF, −Y) (由 hold = est + Y·l̂ − XF·f̂ 反推)。
    这个量与 j 的 `dev` 的**本质区别**: `dev` 是世界系 (含飞机自身位置 ⇒ 含 EKF 漂移),
    本量**只用 像素 + 高度 + 姿态**, 对 EKF 漂移 / 飞机自身站位**完全免疫**。
    超门限 ⇒ 用**当前帧**像素重算保持点并重新对准 (闭环), 次数上限 `L_BODY_REFREEZE_MAX`,
    且**不重置** `_i_freeze_t` ⇒ 20s 必投兜底不变。

用法: python3 scripts/new_project_l_20261009.py [--check]
"""
import argparse
import hashlib
import pathlib
import shutil
import sys
from datetime import datetime

ROOT = pathlib.Path(__file__).resolve().parents[1]
TS = datetime.now().strftime("%Y%m%d_%H%M%S")

# 基底 -> 锚点表 (补丁集与基底无关, 只有 4 个锚点随基底变化)
BASE_ANCHORS = {
    "i": {"revision": 'MISSION_REVISION = "competition-water-20260924-r1-f-i"',
          "consts": "I_RELEASE_TIMEOUT = 20.0",
          "release": "                    if (_settled and _vxy < I_VMAX and _pred_ok) or _dl_pick:"},
    "k": {"revision": 'MISSION_REVISION = "competition-water-20260924-r1-f-i-j-k"',
          "consts": "I_RELEASE_TIMEOUT = 45.0",
          "release": "                    if (_settled and _vxy < I_VMAX and _pred_ok and _dev_ok) or _dl_pick:"},
}
REVISION = {b: a["revision"] for b, a in BASE_ANCHORS.items()}
RELEASE_ANCHOR = {b: a["release"] for b, a in BASE_ANCHORS.items()}
CONSTS_ANCHOR = {b: a["consts"] for b, a in BASE_ANCHORS.items()}

# 由 --base/--name 在 main() 里填充
SITL_SRC = SITL_DST = FLY_SRC = FLY_DST = None
WORLD_SRC = WORLD_DST = TEST_SRC = TEST_DST = SHSRC = SHDST = None
SITL_SETUP = ROOT / "src/drone_drl/setup.py"
FLY_SETUP = ROOT / "fly/src/fly/setup.py"

PLANE_CONST = '''
# ============================================================
# ==== 2026-10-09 项目 l 新增开关 (默认全关 ⇒ 行为与 i 逐位相同) ====
# ============================================================
# ---- P1: 目标平面 (治"求交平面选错") ----
# 语义: 目标平面在【起飞原点平面】下方多少米 (= 离相机更远多少米), 单位 m。
#   求交用深度 = (home_z - self.z) + TARGET_PLANE_H
#   · SITL: 桶顶圆环 NED z = 0.455 (worlds/l.world: 桶 <pose> z=0.25 + cylinder 0.25),
#           home_z ≈ 0 ⇒ 桶口平面比原点平面低 0.455m ⇒ TARGET_PLANE_H = 0.455
#   · 实机: h = (落地时相机离地高) − (桶口离地高), **必须静态标定**(见
#           scripts/analyze_freeze_offset_20261009.py [D]-5): 卸桨/桶摆正下方/读 H 与 est,
#           h = H·(ρ_true/est − 1)。**不可用 SITL 的 0.455 直接推实机**。
# 离线量出的收益 (SITL 几何): H=1.2m 处残差 5.74cm→0.33cm (ρ=20cm) / 15.47→2.37cm (ρ=50cm)
TARGET_PLANE_H = 0.0

# ---- P2: 机体坐标系释放闭环门 (治 "dev/参照漂移" 与"存在性误差") ----
# 残差 = |观测桶在机体(前/右)的偏移 − 期望投放口偏移|, 期望 = (-XF, -Y)。
# **只用 像素+高度+姿态, 不含飞机自身位置** ⇒ 对 EKF 漂移免疫 (j 的 dev 是世界系, 不免疫)。
# 超门限 ⇒ 用当前帧像素重算保持点并重新对准; 次数上限 L_BODY_REFREEZE_MAX,
# 且不重置 _i_freeze_t ⇒ 20s 必投兜底不变。
L_BODY_GATE_M      = 0.0     # 0.0 = 关 (默认); 建议起始 0.06~0.10 (桶口半径 12.5cm)
L_BODY_GATE_LOG    = True    # DROP 段 1Hz 日志附 机体残差 (零风险, 建议常开)
L_BODY_REFREEZE_MAX = 3      # 重新对准次数上限 (闭环)
'''


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def replace_once(text, old, new, tag):
    n = text.count(old)
    if n != 1:
        sys.exit(f"锚点断言失败 [{tag}]: 期望 1 处, 实际 {n} 处\n---\n{old[:200]}")
    return text.replace(old, new)


def build_py(src, dst, is_fly, write=True, base="i", name="l"):
    s = pathlib.Path(src).read_text(encoding="utf-8")

    # --- 1. 文档头 ---
    s = replace_once(
        s,
        "来历 (2026-09-24 新建, 用户要求: 新建 f, e 不动):",
        f'l 补丁集 P1+P2 摘要 (全部"新增开关+默认关"; 关掉时与基底 {base} 逐位相同):\n'
        f'  P1 TARGET_PLANE_H : 求交平面从"起飞原点平面"改为"目标平面" (SITL=0.455, 实机需静态标定)\n'
        f'  P2 L_BODY_GATE_M  : 机体坐标系释放残差门 + 超限重新对准 (对 EKF 漂移免疫)\n'
        f'基底 = {base}.py; 生成器 scripts/new_project_l_20261009.py --base {base} --name {name}\n'
        f'搬运检查: scripts/check_drop_portability_20261009.py\n\n'
        "来历 (2026-09-24 新建, 用户要求: 新建 f, e 不动):",
        "docstring",
    )

    # --- 2. 节点名 / 类名 / 版本 ---
    UP = name.upper()
    BUP = base.upper()
    s = replace_once(s, f"super().__init__('{base}_mission')",
                     f"super().__init__('{name}_mission')", "node name")
    s = replace_once(s, f"class {BUP}Mission(Node):", f"class {UP}Mission(Node):", "class name")
    s = replace_once(s, REVISION[base],
                     f'MISSION_REVISION = "competition-water-20261009-r1-{name}-P1P2"', "revision")
    s = replace_once(s, f"{BUP}Mission()", f"{UP}Mission()", "main instantiation")

    # --- 3. 新常量 ---
    s = replace_once(s, CONSTS_ANCHOR[base],
                     CONSTS_ANCHOR[base] + PLANE_CONST, "constants")

    # --- 4. P1: 共用相机模型的 4 个作用点 ---
    s = replace_once(
        s,
        "        cx = roi.x_offset + roi.width / 2.0; cy = roi.y_offset + roi.height / 2.0\n"
        "        projected = camera_pixel_to_level(cx, cy, self._pitch, self._roll)\n"
        "        if projected is None: return None\n"
        "        cam_fwd, cam_right = (v * H for v in projected)",
        "        cx = roi.x_offset + roi.width / 2.0; cy = roi.y_offset + roi.height / 2.0\n"
        "        projected = camera_pixel_to_level(cx, cy, self._pitch, self._roll)\n"
        "        if projected is None: return None\n"
        "        # 2026-10-09 l/P1: 交面 = 起飞原点平面【下方 TARGET_PLANE_H 米】(= 桶口平面)\n"
        "        cam_fwd, cam_right = (v * (H + TARGET_PLANE_H) for v in projected)",
        "P1 _roi_to_ned",
    )
    s = replace_once(
        s,
        "        cam_fwd, cam_right = self._ned_to_body(dx, dy)\n"
        "        return level_to_camera_pixel(cam_fwd, cam_right, H, self._pitch, self._roll)",
        "        cam_fwd, cam_right = self._ned_to_body(dx, dy)\n"
        "        # 2026-10-09 l/P1: 与 _roi_to_ned 同一平面\n"
        "        return level_to_camera_pixel(cam_fwd, cam_right, H + TARGET_PLANE_H,\n"
        "                                     self._pitch, self._roll)",
        "P1 _ned_to_expected_pixel",
    )
    n = s.count("pixel_offset = offset_body_y * FX / dz_h")
    if n != 2:
        sys.exit(f"锚点断言失败 [P1 pixel_offset]: 期望 2 处, 实际 {n} 处")
    s = s.replace(
        "pixel_offset = offset_body_y * FX / dz_h",
        "pixel_offset = offset_body_y * FX / (dz_h + TARGET_PLANE_H)   # l/P1: 同一平面")
    s = s.replace(
        "pixel_offset_y = offset_body_xf * FY / dz_h",
        "pixel_offset_y = offset_body_xf * FY / (dz_h + TARGET_PLANE_H)   # l/P1")

    # --- 5. P2: 机体残差方法 (插在 _i_target_tracker_ok 之前) ---
    body_method = '''    def _l_body_resid(self):
        """机体坐标系释放残差 (2026-10-09 l/P2)。

        = |观测到的桶 在机体(前,右) 的偏移 − 期望的投放口偏移|
        期望偏移由 hold = est + Y·l̂ − XF·f̂ 反推 ⇒ 桶在机体系 (前,右) 应为 (+XF, +Y)。

        **只用到 像素 + 高度 + 姿态, 不含飞机自身位置** ⇒ 对 EKF 位置漂移/自身站位免疫,
        这正是它相对 j 的 `dev`(世界系, 含飞机位置) 的本质区别。

        返回 None = 失效开放 (无检测 / 贴边框 / 未确认 ⇒ 不拦, 交给 20s 必投兜底)。
        """
        try:
            if not self._drop_detection_fresh():
                return None
            roi = self.latest_roi
            if roi is None or self.heading is None or self.home_z is None:
                return None
            if not self._i_roi_inside(roi):
                return None
            H = (self.home_z - self.z) + TARGET_PLANE_H
            if not math.isfinite(H) or H < 0.3:
                return None
            cx = roi.x_offset + roi.width / 2.0
            cy = roi.y_offset + roi.height / 2.0
            p = camera_pixel_to_level(cx, cy, self._pitch, self._roll)
            if p is None:
                return None
            b_fwd, b_right = p[0] * H, p[1] * H
            dy = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
            dx = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
            # 期望: 飞机飞到 hold = est + Y·l̂ − XF·f̂ ⇒ 桶相对飞机的机体系偏移应为
            # (fwd, right) = (+XF, +Y) (Y>0 = 瞄准点左移 ⇒ 桶出现在飞机右侧)。
            return math.hypot(b_fwd - dx, b_right - dy)
        except Exception:
            return None

    def _l_refreeze(self, sim_t):
        """P2: 用【当前帧】像素在机体系重算保持点并重新对准 (闭环)。

        保持点公式与首次冻结完全相同: hold = 桶估计 + 两瓶补偿 (纯几何)。
        只读当前像素, 与飞机自身位置无关 ⇒ 对 EKF 漂移免疫。
        不重置 `_i_freeze_t` ⇒ 20s 必投兜底与 deadline 路径均不变。
        """
        _bo = self._l_body_raw()
        if _bo is None:
            return False
        _em = (self.x - self.home_x + _bo[0], self.y - self.home_y + _bo[1])
        dy = PAYLOAD_OFFSET_BODY_Y_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_Y_B2
        dx = PAYLOAD_OFFSET_BODY_XF_B1 if self._drop_count == 0 else PAYLOAD_OFFSET_BODY_XF_B2
        self._i_hold_x = _em[0] + dy * math.sin(self.heading) - dx * math.cos(self.heading)
        self._i_hold_y = _em[1] - dy * math.cos(self.heading) - dx * math.sin(self.heading)
        self._i_arrive_t = None
        self._l_refreeze_n = getattr(self, '_l_refreeze_n', 0) + 1
        self.get_logger().warn(
            f"l/P2 机体残差超限 → 重新对准 #{self._l_refreeze_n} "
            f"新保持点=({self._i_hold_x:+.3f},{self._i_hold_y:+.3f})")
        return True

    def _l_body_raw(self):
        """当前帧桶相对飞机的机体(前,右)偏移 (m), 不含任何飞机位置量。"""
        roi = self.latest_roi
        if roi is None or self.heading is None or self.home_z is None:
            return None
        H = (self.home_z - self.z) + TARGET_PLANE_H
        if not math.isfinite(H) or H < 0.3:
            return None
        cx = roi.x_offset + roi.width / 2.0
        cy = roi.y_offset + roi.height / 2.0
        p = camera_pixel_to_level(cx, cy, self._pitch, self._roll)
        if p is None:
            return None
        return (p[0] * H, p[1] * H)

'''
    s = replace_once(s, "    def _i_target_tracker_ok(self):",
                     body_method + "    def _i_target_tracker_ok(self):", "P2 methods")

    # --- 6. P2: 释放判据加门 + 超限重新对准 ---
    rel = RELEASE_ANCHOR[base]
    s = replace_once(
        s,
        rel,
        "                    # 2026-10-09 l/P2: 机体坐标系残差门 (L_BODY_GATE_M=0 ⇒ _body_ok 恒 True)\n"
        "                    _bres = self._l_body_resid() if (L_BODY_GATE_M > 0 or L_BODY_GATE_LOG) else None\n"
        "                    _bres_txt = (f'{_bres * 100:.1f}cm' if _bres is not None else 'n/a')\n"
        "                    _body_ok = (L_BODY_GATE_M <= 0) or (_bres is None) or (_bres <= L_BODY_GATE_M)\n"
        "                    if (L_BODY_GATE_M > 0 and _settled and not _body_ok\n"
        "                            and getattr(self, '_l_refreeze_n', 0) < L_BODY_REFREEZE_MAX):\n"
        "                        if self._l_refreeze(sim_t):\n"
        "                            return\n"
        + rel.replace(") or _dl_pick:", " and _body_ok) or _dl_pick:"),
        "P2 gate",
    )

    # --- 7. 日志: 释放行附机体残差 ---
    s = replace_once(
        s,
        'f"pred={_pred * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s "',
        'f"pred={_pred * 100:.1f}cm 停留{_dwell:.1f}s vxy={_vxy:.3f}m/s "\n'
        '                            f"机体残差={_bres_txt} "',
        "P2 release log",
    )

    if write:
        pathlib.Path(dst).write_text(s, encoding="utf-8")
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只做锚点检查, 不写文件")
    ap.add_argument("--base", default="i", choices=sorted(BASE_ANCHORS), help="基底节点")
    ap.add_argument("--name", default="l", help="新节点字母")
    args = ap.parse_args()
    global SITL_SRC, SITL_DST, FLY_SRC, FLY_DST, WORLD_SRC, WORLD_DST
    global TEST_SRC, TEST_DST, SHSRC, SHDST
    b, nm = args.base, args.name
    SITL_SRC = ROOT / f"src/drone_drl/drone_drl/{b}.py"
    SITL_DST = ROOT / f"src/drone_drl/drone_drl/{nm}.py"
    FLY_SRC = ROOT / f"fly/src/fly/fly/{b}.py"
    FLY_DST = ROOT / f"fly/src/fly/fly/{nm}.py"
    WORLD_SRC = ROOT / f"worlds/{b}.world"
    WORLD_DST = ROOT / f"worlds/{nm}.world"
    TEST_SRC = ROOT / f"scripts/test_{b}.sh"
    TEST_DST = ROOT / f"scripts/test_{nm}.sh"
    # fly/k.sh 只存在于板上 (仓库没有) ⇒ 回退用 fly/i.sh 当模板
    SHSRC = ROOT / f"fly/{b}.sh"
    if not SHSRC.exists():
        SHSRC = ROOT / "fly/i.sh"
    SHDST = ROOT / f"fly/{nm}.sh"

    for f in (SITL_SRC, FLY_SRC, WORLD_SRC, TEST_SRC, SHSRC):
        if not f.exists():
            sys.exit(f"缺少源文件 {f}")
    print(f"源 {b}.py sha: SITL {sha(SITL_SRC)[:12]}  fly {sha(FLY_SRC)[:12]}")

    # 备份 setup.py
    if not args.check:
        for sp in (SITL_SETUP, FLY_SETUP):
            bak = sp.with_suffix(f".py.bak.{TS}_newproject_l")
            shutil.copy2(sp, bak)
            print(f"备份 {bak.relative_to(ROOT)}")
        # 就地补丁 (src == dst): **必须先备份节点本身** (改前版本 = 验收审计的基底)
        for src, dst, tag in ((SITL_SRC, SITL_DST, "SITL"), (FLY_SRC, FLY_DST, "fly")):
            if src.resolve() == dst.resolve():
                bak = src.with_suffix(f".py.bak.{TS}_P1P2")
                shutil.copy2(src, bak)
                print(f"备份 {bak.relative_to(ROOT)}  ({tag} 就地补丁的改前版本)")

    s_sitl = build_py(SITL_SRC, SITL_DST, False, write=not args.check, base=b, name=nm)
    s_fly = build_py(FLY_SRC, FLY_DST, True, write=not args.check, base=b, name=nm)

    core_sitl = s_sitl.replace("class LMission", "class IMission").split("PLANE_CONST")[0]
    print(f"{nm}.py (SITL) {len(s_sitl.splitlines())} 行 / (fly) {len(s_fly.splitlines())} 行")

    if args.check:
        print("--check 通过 (未写文件)")
        return

    # world / 启动脚本 (就地补丁时这些已存在, 不动)
    if WORLD_SRC.resolve() != WORLD_DST.resolve():
        shutil.copy2(WORLD_SRC, WORLD_DST)
    else:
        print(f"就地补丁: {WORLD_DST.relative_to(ROOT)} / {TEST_DST.relative_to(ROOT)} 已存在, 跳过复制")
    t = TEST_SRC.read_text(encoding="utf-8").replace(f"{b}.world", f"{nm}.world") \
        .replace(f"{b}_output.log", f"{nm}_output.log").replace(f"{b}_judge.log", f"{nm}_judge.log") \
        .replace(f"drone_drl {b}", f"drone_drl {nm}")
    if TEST_SRC.resolve() != TEST_DST.resolve():
        TEST_DST.write_text(t, encoding="utf-8")
        TEST_DST.chmod(0o755)
    sh = SHSRC.read_text(encoding="utf-8").replace("子项目 f", f"子项目 {nm}") \
        .replace(f"run fly {b}", f"run fly {nm}").replace(f"~/fly/{b}.sh", f"~/fly/{nm}.sh")
    if not SHDST.exists():          # 就地补丁时 fly/k.sh 可能还没有 (仓库侧)
        SHDST.write_text(sh, encoding="utf-8")
        SHDST.chmod(0o755)
        print(f"新增 {SHDST.relative_to(ROOT)}")
    print(f"新增 {WORLD_DST.relative_to(ROOT)} / {TEST_DST.relative_to(ROOT)} / {SHDST.relative_to(ROOT)}")

    # setup.py 入口
    for sp, entry, anchor in (
            (SITL_SETUP, f"'{nm} = drone_drl.{nm}:main',", "'h = drone_drl.h:main',"),
            (FLY_SETUP, f"'{nm} = fly.{nm}:main',", "'h = fly.h:main',")):
        txt = sp.read_text(encoding="utf-8")
        if entry in txt:
            print(f"  {sp.relative_to(ROOT)}: l 入口已存在, 跳过")
            continue
        new = replace_once(txt, anchor, anchor + "\n        " + entry, f"setup {sp.name}")
        sp.write_text(new, encoding="utf-8")
        print(f"  {sp.relative_to(ROOT)}: 已加 {entry}")

    print(f"\n完成。下一步:")
    print(f"  colcon build --packages-select drone_drl")
    print(f"  python3 scripts/test_l_target_plane_20261009.py {nm}")
    print(f"  bash scripts/test_{nm}.sh          # SITL (默认开关全关 = 与 {b} 同行为)")
    print(f"  python3 scripts/check_drop_portability_20261009.py {nm} {'l' if nm != 'l' else 'm'}")


if __name__ == "__main__":
    main()
