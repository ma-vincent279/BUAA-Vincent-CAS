#!/usr/bin/env python3
"""项目 i: 选桶策略改为"按大小取大的两个" (2026-10-07, 用户要求)

改动:
  A. src/drone_drl/drone_drl/i.py + fly/src/fly/fly/i.py
     选桶排序: "|lateral| 升序(优先靠航向中线)" → "检测框面积降序取前 2 (大的+中的, 抛弃最小的)"
     面积同级 (相对差 ≤10%) 时仍按 |lateral| 升序兜底, 保留"优先靠中线"的旧优点.
     防幽灵守卫 (两目标须由不同框支撑, ≥25px) / 已投桶守卫 全部不动.
  B. src/drone_drl/drone_drl/yolov8_bucket_detector.py (仅 SITL 仿真检测器)
     新增 bucket{n}_radius 参数 (默认 0.125 = 旧行为), 让仿真能造出"三个桶大小不同"的框,
     用于验证新选桶策略 (真实实飞用的是 YOLO 框, 不需要这个参数).
     另: 实机 fly 包里没有这个检测器文件, 不受影响.
"""
import datetime
import difflib
import re
import shutil
import sys

STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
I_FILES = ["/home/vincent/SITL/src/drone_drl/drone_drl/i.py",
           "/home/vincent/SITL/fly/src/fly/fly/i.py"]
DET = "/home/vincent/SITL/src/drone_drl/drone_drl/yolov8_bucket_detector.py"

OLD_SORT = '''            # 2026-09-12 e 选桶策略改版 (自 d): "检测框面积降序" → "优先选离航向中线最近的桶".
            # 动机 (实机 d_mission_20260911_170203.log): 面积最大 ≠ 最合适 —— 该轮按面积选中
            # 侧向 lateral -4.21m 的桶当第一目标, 几乎正前方的 -0.42m 桶反被丢弃, 既贴飞行边界
            # 又落进 85° 广角镜头像场边缘 (定位偏差最大) 的区域.
            if self._heading0 is not None:
                s_h0, c_h0 = math.sin(self._heading0), math.cos(self._heading0)
                # 主键 |lateral| 升序 (正值=heading0 右侧); 次键 面积降序 (两侧向接近时仍优先更近/更大的桶)
                confirmed.sort(key=lambda t: (
                    abs(-t["ned"][0] * s_h0 + t["ned"][1] * c_h0),
                    -t.get("area", 0)))
            else:
                confirmed.sort(key=lambda t: t.get("area", 0), reverse=True)
'''

NEW_SORT = '''            # 2026-10-07 (用户要求): 选桶策略 = "按大小取大的两个" —— 检测框面积降序取前 2
            #   (大的 + 中的), 抛弃最小的那个. 面积按最大值分 10 档, 同档 (相对差 ≤10%) 时
            #   按 |lateral| 升序兜底, 保留"优先靠航向中线"的旧优点 (避免选中贴飞行边界/
            #   85° 像场边缘的桶). 历史: 2026-09-12 e 曾把"面积降序"改成"|lateral| 升序",
            #   本次按用户要求改回并以 |lateral| 作为同级兜底.
            s_h0 = math.sin(self._heading0) if self._heading0 is not None else 0.0
            c_h0 = math.cos(self._heading0) if self._heading0 is not None else 0.0
            _area_ref = max(1.0, max((float(t.get("area", 0.0)) for t in confirmed), default=1.0))
            confirmed.sort(key=lambda t: (
                -round(float(t.get("area", 0.0)) / _area_ref * 10.0),   # 面积降序 (0.1 档)
                abs(-t["ned"][0] * s_h0 + t["ned"][1] * c_h0)))         # 同档 → |lateral| 升序
'''

OLD_BASIS = 'f"  Selection basis (all {len(confirmed)} confirmed, |lateral| asc): {all_txt}")'
NEW_BASIS = ('f"  Selection basis (all {len(confirmed)} confirmed, 面积降序取前2): {all_txt}")\n'
             '                _small = [t for t in confirmed if t not in picked]\n'
             '                if _small:\n'
             '                    self.get_logger().info("  抛弃(较小): " + ", ".join(\n'
             '                        f"NED({t[\'ned\'][0]:.2f},{t[\'ned\'][1]:.2f})"\n'
             '                        f"[area={t.get(\'area\', 0):.0f}]" for t in _small))')

OLD_SELINFO = 'f"{len(selected)}/{len(confirmed)} chosen: "'
NEW_SELINFO = 'f"{len(selected)}/{len(confirmed)} chosen (面积降序, 抛弃较小的): "'


def patch(path, pairs, tag):
    src = open(path, encoding="utf-8").read()
    if all(new in src for _, new in pairs):
        print(f"  [SKIP] {tag}: 已打过补丁")
        return True
    out = src
    for old, new in pairs:
        n = out.count(old)
        if n != 1:
            print(f"  [FAIL] {tag}: 锚点命中 {n} 次 (应为 1): {old.splitlines()[0][:60]}…")
            return False
        out = out.replace(old, new)
    shutil.copy2(path, f"{path}.bak.{STAMP}_select_big")
    open(path, "w", encoding="utf-8").write(out)
    print(f"  [OK  ] {tag}: 已改 ({path.split('/')[-1]}), 备份 *.bak.{STAMP}_select_big")
    d = list(difflib.unified_diff(src.splitlines(True), out.splitlines(True), n=0))
    print("".join("        " + l for l in d if l.startswith(("+", "-")) and not l.startswith(("+++", "---"))))
    return True


def main():
    ok = True
    print("== A) 两份 i.py: 选桶改'面积降序取前2' ==")
    for f in I_FILES:
        ok &= patch(f, [(OLD_SORT, NEW_SORT), (OLD_BASIS, NEW_BASIS), (OLD_SELINFO, NEW_SELINFO)],
                    f.split("/")[-2] + "/" + f.split("/")[-1])

    print("== B) SITL 检测器: 新增 bucket{n}_radius ==")
    ok &= patch(DET, [
        ("""            for axis in ('x', 'y', 'z'):
                self.declare_parameter(f'bucket{n}_gz_{axis}', 0.0)
""",
         """            for axis in ('x', 'y', 'z'):
                self.declare_parameter(f'bucket{n}_gz_{axis}', 0.0)
            # 2026-10-07: 单桶半径参数 (默认 0.125 = 旧固定值). 仅用于 SITL 造"大小不同的桶"
            # 来验证选桶策略; 实机走 YOLO 框, 不需要这个参数.
            self.declare_parameter(f'bucket{n}_radius', 0.125)
"""),
        ("""        self._bucket_neds = []
        if self._bucket_count == 1:""",
         """        self._bucket_neds = []
        self._bucket_radii = []
        if self._bucket_count == 1:"""),
        ("""            self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
        else:""",
         """            self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
            self._bucket_radii.append(float(self.get_parameter('bucket1_radius').value))
        else:"""),
        ("""                self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
""",
         """                self._bucket_neds.append((bgy - sgy, bgx - sgx, -(bgz - sgz)))
                self._bucket_radii.append(float(self.get_parameter(f'bucket{n}_radius').value))
"""),
        ("""        for i, ned in enumerate(self._bucket_neds):
            self.get_logger().info(
                f"SIM bucket[{i}] NED({ned[0]:.2f},{ned[1]:.2f},{ned[2]:.2f}) "
                f"r={self.bucket_radius:.2f}m")""",
         """        for i, ned in enumerate(self._bucket_neds):
            self.get_logger().info(
                f"SIM bucket[{i}] NED({ned[0]:.2f},{ned[1]:.2f},{ned[2]:.2f}) "
                f"r={self._bucket_radii[i]:.3f}m")"""),
        ("""        for ned in self._bucket_neds:
            bx, by, bz = ned""",
         """        for _bi, ned in enumerate(self._bucket_neds):
            bx, by, bz = ned"""),
        ("""            radius_px = self.fx * self.bucket_radius / dz""",
         """            radius_px = self.fx * self._bucket_radii[_bi] / dz"""),
    ], "yolov8_bucket_detector.py")

    print("\n" + ("=== ALL OK ===" if ok else "=== 有失败项 ==="))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
