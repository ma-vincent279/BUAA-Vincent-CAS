#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_center_before_freeze_20261009.py — k 模式「先入画再冻结」: 修 DROP 段贴边死锁 (只 k 两份)

16:09 实飞实证 (第一趟走完 5m→降高→锚点→1.2m 检测→TRACK/REFINE/PRE_SERVO/VISUAL_SERVO/DROP 的飞行):
  DROP 段 `px=(-146,-140) / (-46,-160) / (+10,-164)` ⇒ 桶在画面中心下方 ~160px ≈ 0.6m
  ⇒ 飞机**冲过桶 0.6m**; 而 1.2m 时相机前向只有 ±0.672m ⇒ 检测框 `cy=324~344,h=50~71`
  **底边超出画面 (360) 被裁** ⇒ 日志刷「⚠ k模式暂不采样 (框贴边)」⇒ **0/5 样本** ⇒ 冻结不了
  ⇒ 45s 兜底前机手切手操 ⇒ 0/2。根因: 低空瞄准点由 **5m 检测**推得 (那趟两个 TRK-NEW 互差 0.45m),
  而 1.2m 视野只有 ±0.672m ⇒ 0.6m 偏差就把框顶到画面外沿 ⇒ **越裁越偏、越偏越裁的死锁**。

修法 (只在 DROP 段"未冻结"阶段, 不碰冻结/释放判据):
  若当前帧的桶框**贴边**或**框心偏离画面中心 > K_CENTER_PX**, 把**命令点**朝桶方向以
  `K_CENTER_SPEED` 匀速挪 (累计 ≤ `K_CENTER_MAX`), 挪到"框完整入画且居中"后才进入原有的
  等静/采样/冻结流程 ⇒ 冻结样本拿到的是**未被裁的框** ⇒ 估计正确。
  ① 方向取自框心 (即使被裁, 框心仍在画面内 ⇒ 符号可靠), 只取**方向**不取大小 ⇒ 不怕偏差;
  ② **时间锚定**: 累加在"命令偏移"上 (不是每拍相对当前位置) —— 与降高同一个道理, 否则位置环
     只跟微小误差 ⇒ 爬行 (降高段已踩过);
  ③ 速率 8cm/s (f_r3 已验证的低空缓移量级) + 累计上限 1.0m ⇒ 不会失控/自激。

用法: python3 scripts/patch_k_center_before_freeze_20261009.py [--check]
"""
import argparse
import datetime
import hashlib
import pathlib
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = [REPO / "src/drone_drl/drone_drl/k.py", REPO / "fly/src/fly/fly/k.py"]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_center"

# A) 常量 (放在 I_FREEZE_EDGE_MARGIN 之后)
A_ANCHOR = "I_FREEZE_EDGE_MARGIN = 8   # P3: 冻结样本要求检测框距画面边界 ≥8px (see.py 对越界框只裁剪不剔除)\n"
A_INSERT = (
    "# 2026-10-09 追加 40: DROP 段「先入画再冻结」—— 框贴边/偏心时把命令点朝桶匀速挪\n"
    "K_CENTER_SPEED = 0.08   # m/s (f_r3 已验证的低空缓移量级)\n"
    "K_CENTER_MAX   = 1.00   # m    本次 DROP 累计最大挪动量 (硬上限, 防失控)\n"
    "K_CENTER_PX    = 60.0   # px   框心偏离画面中心超过它就继续挪 (1.2m 处 60px ≈ 0.22m)\n"
)

# B) DROP 段: 未冻结前先入画
B_ANCHOR = "                    _pkey = (self._drop_count, round(self._i_entry_t or 0.0, 2))\n"
B_INSERT = (
    "                    # ===== 2026-10-09 追加 40: 「先入画再冻结」=====\n"
    "                    #   16:09 实飞: 飞机冲过桶 0.6m ⇒ 框被画面下边缘裁掉 ⇒ 由它算出的位置全偏,\n"
    "                    #   而冻结拒收贴边框 ⇒ 死锁 0/5 样本 ⇒ 不投。这里先按框心方向把命令点\n"
    "                    #   匀速挪过去 (时间锚定), 入画且居中后再走原有的等静/采样/冻结。\n"
    "                    if getattr(self, '_c_base', None) is None:\n"
    "                        self._c_base = (self.x - self.home_x, self.y - self.home_y)\n"
    "                        self._c_off = [0.0, 0.0]\n"
    "                    _c_roi = self.latest_roi if self._drop_detection_fresh() else None\n"
    "                    if _c_roi is not None:\n"
    "                        _clipped = (_c_roi.x_offset <= 0 or _c_roi.y_offset <= 0\n"
    "                                    or _c_roi.x_offset + _c_roi.width >= IMG_W\n"
    "                                    or _c_roi.y_offset + _c_roi.height >= IMG_H)\n"
    "                        _ccx = _c_roi.x_offset + _c_roi.width / 2.0\n"
    "                        _ccy = _c_roi.y_offset + _c_roi.height / 2.0\n"
    "                        _coff = math.hypot(_ccx - CX, _ccy - CY)\n"
    "                        if _clipped or _coff > K_CENTER_PX:\n"
    "                            _ch = self.home_z - self.z\n"
    "                            _cf = (CY - _ccy) / FY * _ch        # 机体前向 (带符号)\n"
    "                            _cr = (_ccx - CX) / FX * _ch        # 机体右向 (带符号)\n"
    "                            _cn = math.hypot(_cf, _cr)\n"
    "                            if _cn > 1e-6:\n"
    "                                _step = K_CENTER_SPEED * 0.05   # 时间锚定: 每拍固定步长\n"
    "                                _chdg = self.heading if self.heading else 0.0\n"
    "                                # 机体(前,右) → NED: 前 = (cos h, sin h), 右 = (sin h, -cos h)\n"
    "                                # ⚠ 必须 self._c_off (跨 tick 累积); 写成局部 _c_off 会 NameError\n"
    "                                self._c_off[0] += ((_cf / _cn) * _step * math.cos(_chdg)"
    "                                                    + (_cr / _cn) * _step * math.sin(_chdg))\n"
    "                                self._c_off[1] += ((_cf / _cn) * _step * math.sin(_chdg)"
    "                                                    - (_cr / _cn) * _step * math.cos(_chdg))\n"
    "                                _c_norm = math.hypot(self._c_off[0], self._c_off[1])\n"
    "                                if _c_norm > K_CENTER_MAX:      # 硬上限\n"
    "                                    self._c_off[0] *= K_CENTER_MAX / _c_norm\n"
    "                                    self._c_off[1] *= K_CENTER_MAX / _c_norm\n"
    "                                if sim_t - getattr(self, '_c_log_t', -99.0) > 2.0:\n"
    "                                    self._c_log_t = sim_t\n"
    "                                    self.get_logger().info(\n"
    "                                        f\"  [先入画] 框{'贴边' if _clipped else '偏心'} \"\n"
    "                                        f\"(框心偏 {_coff:.0f}px, 机体前{_cf:+.2f}/右{_cr:+.2f}m)\"\n"
    "                                        f\" ⇒ 命令点朝桶挪 已={_c_norm:.2f}m/{K_CENTER_MAX:.1f}m\"\n"
    "                                        f\" (速度 {K_CENTER_SPEED:.2f}m/s)\")\n"
    "                            self._pub_pos(self.home_x + self._c_base[0] + self._c_off[0],\n"
    "                                          self.home_y + self._c_base[1] + self._c_off[1],\n"
    "                                          drop_z, self.heading)\n"
    "                            return\n"
)

MARK = "K_CENTER_SPEED"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path: pathlib.Path, check: bool) -> str:
    src = path.read_text(encoding="utf-8")
    if MARK in src:
        return "SKIP (已打过, 幂等)"
    for name, anchor in (("A", A_ANCHOR), ("B", B_ANCHOR)):
        n = src.count(anchor)
        if n != 1:
            print(f"  ✗ {path}: 锚点 {name} 命中 {n} 次 (要求恰 1 次) ⇒ 中止", file=sys.stderr)
            sys.exit(2)
    out = src.replace(A_ANCHOR, A_ANCHOR + A_INSERT, 1)
    out = out.replace(B_ANCHOR, B_INSERT + B_ANCHOR, 1)
    # 结构不变量: DROP 段入口 (self._i_samp 初始化) 仍在; 新块必须早于 _pkey
    # 结构不变量: 新块内不得出现裸 `_c_off` (必须 self._c_off, 否则跨 tick 不累积/NameError)
    _blk = out[out.index("# ===== 2026-10-09 追加 40"):out.index("_pkey = (self._drop_count")]
    import re as _re
    _blk_code = "\n".join(l for l in _blk.splitlines() if not l.strip().startswith("#"))
    if _re.search(r"(?<!self\.)\b_c_off\b", _blk_code):
        print(f"  ✗ {path}: 出现裸 `_c_off` (须 self._c_off) ⇒ 中止", file=sys.stderr)
        sys.exit(3)
    i = out.index("\n        elif self.step == STEP_DROP:")
    blk = out[i:i + 12000]
    if not (0 <= blk.find("[先入画]") < blk.find("_pkey = (self._drop_count")):
        print(f"  ✗ {path}: 「先入画」块必须早于 _pkey 等静逻辑 ⇒ 中止", file=sys.stderr)
        sys.exit(3)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 语法错误 {e} ⇒ 中止", file=sys.stderr)
        sys.exit(4)
    if check:
        return f"CHECK-OK (将改 {len(out) - len(src):+d} 字节)"
    bak = path.with_name(path.name + "." + BAK_SUFFIX)
    shutil.copy2(path, bak)
    path.write_text(out, encoding="utf-8")
    return f"OK  {sha(path)}  (备份 {bak.name})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    print(f"=== patch_k_center_before_freeze_20261009 {'(CHECK)' if a.check else ''} ===")
    for f in FILES:
        if not f.exists():
            print(f"  ✗ 缺文件 {f}", file=sys.stderr)
            sys.exit(5)
        print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
    print("=== done ===")


if __name__ == "__main__":
    main()
