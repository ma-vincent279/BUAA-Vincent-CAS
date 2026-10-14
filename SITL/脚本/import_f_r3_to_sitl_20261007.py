#!/usr/bin/env python3
"""把外接盘那份 "f r3 (predictive / continuous-brake-hold)" 导入 SITL 工作空间.

来源: /media/vincent/25390BC89B897322/f_20261006_predictive/f.py
      sha256 e1a32bf1facaebe7533285614e87d9c8750704fbacfda20130df9131c77e3f9f (交付清单已校验)

目标: src/drone_drl/drone_drl/f_r3.py + setup.py 新增入口 `f_r3`

只做 6 处**最小适配改动** (逻辑一行未动):
  ① class FMission -> FR3Mission            (避免与仓库 f.py 的类名混淆)
  ② 节点名 'f_mission' -> 'f_r3_mission'
  ③ main() 里 FMission() -> FR3Mission()
  ④ PX4_STATUS_SUFFIX "_v1" -> "_v4"        (SITL PX4 固件发布 vehicle_status_v4)
  ⑤ HEADING_DEG 161.0 -> 92.0               (i.world 场地航向, 与 SITL i.py 一致)
  ⑥ MISSION_REVISION 追加 "-sitl-import"
另在文件头插入来源/改动说明注释块.

未改 (有意保留): 低空限速伺服 / 速度预判制动 / 固定刹停点 / 35s 必发 /
  释放判据 DROP_ARRIVE_M=0.05 + DROP_CONFIRM_FRAMES=2 / 补偿 ±0.0567 + 前后 0 /
  POSITION_REFINE_IDENTITY_MAX=1.5 / 选桶 |lateral| 升序 / GROUND_NED_Z 常量(未被使用)

用法: python3 scripts/import_f_r3_to_sitl_20261007.py
回滚: rm src/drone_drl/drone_drl/f_r3.py + 去掉 setup.py 里 'f_r3 = ...' 那一行 + colcon build
"""
import difflib
import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

SRC = Path("/media/vincent/25390BC89B897322/f_20261006_predictive/f.py")
SRC_SHA = "e1a32bf1facaebe7533285614e87d9c8750704fbacfda20130df9131c77e3f9f"
ROOT = Path("/home/vincent/SITL")
DST = ROOT / "src/drone_drl/drone_drl/f_r3.py"
SETUP = ROOT / "src/drone_drl/setup.py"
STAMP = time.strftime("%Y%m%d_%H%M%S")

HEADER = """#!/usr/bin/env python3
# ============================================================================
# 【SITL 导入版 — 2026-10-07】外接盘 "f r3" (predictive / continuous-brake-hold)
#   源文件   : {src}
#   源 sha256: {sha}
#   本文件   = 源文件 + 下列 6 处最小适配 (控制逻辑一行未改)
#     ① 类名 FMission -> FR3Mission   ② 节点名 f_mission -> f_r3_mission
#     ③ main() 实例化同名            ④ PX4_STATUS_SUFFIX "_v1" -> "_v4" (SITL 固件)
#     ⑤ HEADING_DEG 161.0 -> 92.0 (i.world 场地航向)
#     ⑥ MISSION_REVISION 追加 "-sitl-import"
#   设计要点(原样保留): 低空 XY 位置目标限速 8cm/s + 按实测速度提前 0.6s 制动 +
#     制动时钉固定刹停点; 释放判据 = 两帧新帧 ≤5cm 且 |vxy|≤0.10 (需检测器发 frame_seq);
#     35s 无条件放水; 补偿 ±0.0567 / 前后 0 (台架几何, 未经实飞标定)
#   ⚠ 检测器必须发 frame_seq, 否则低空伺服完全不动作 (设定点锚死 + 35s 兜底)
# ============================================================================
"""

# (锚点, 替换, 说明) —— 每个锚点必须**唯一命中**
PATCHES = [
    ("class FMission(Node):",
     "class FR3Mission(Node):",
     "① 类名"),
    ("        super().__init__('f_mission')",
     "        super().__init__('f_r3_mission')",
     "② 节点名"),
    ("    node = FMission()",
     "    node = FR3Mission()",
     "③ main 实例化"),
    ('PX4_STATUS_SUFFIX = "_v1"',
     'PX4_STATUS_SUFFIX = "_v4"   # 2026-10-07 SITL 导入: SITL 固件发布 vehicle_status_v4 (_v1=实机)',
     "④ 状态话题后缀"),
    ("HEADING_DEG = 161.0  # 沿用 2026-10-06 17:40 日志的场地方向；0=北、90=东",
     "HEADING_DEG = 92.0  # 2026-10-07 SITL 导入: i.world 场地航向 (与 SITL i.py 一致); 原实机值 161.0",
     "⑤ 航向"),
    ('MISSION_REVISION = "competition-water-20261006-r3-f-continuous-brake-hold"',
     'MISSION_REVISION = "competition-water-20261006-r3-f-continuous-brake-hold-sitl-import"',
     "⑥ 版本号"),
]

ENTRY = "        'f_r3 = drone_drl.f_r3:main',\n"
ENTRY_AFTER = "        'h = drone_drl.h:main',\n"


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> int:
    if not SRC.is_file():
        sys.exit(f"[FAIL] 源文件不存在: {SRC} (外接盘是否挂载?)")
    got = sha(SRC)
    if got != SRC_SHA:
        sys.exit(f"[FAIL] 源 sha256 不符\n  期望 {SRC_SHA}\n  实际 {got}")
    print(f"[1/5] 源文件 sha256 校验通过: {got}")

    orig = SRC.read_text(encoding="utf-8")
    lines = orig.split("\n")
    if not lines[0].startswith("#!"):
        sys.exit("[FAIL] 源文件首行不是 shebang, 拒绝改")
    out = HEADER.format(src=SRC, sha=got) + "\n".join(lines[1:])

    print("[2/5] 应用最小适配改动:")
    for old, new, tag in PATCHES:
        n = out.count(old)
        if n != 1:
            sys.exit(f"[FAIL] {tag}: 锚点命中 {n} 次 (要求 1 次): {old[:60]!r}")
        out = out.replace(old, new, 1)
        print(f"    {tag}: OK")

    if DST.exists():
        bak = DST.with_suffix(f".py.bak.{STAMP}_reimport")
        shutil.copy2(DST, bak)
        print(f"[3/5] 目标已存在, 已备份 -> {bak.name}")
    else:
        print("[3/5] 目标文件不存在, 直接新建")
    DST.write_text(out, encoding="utf-8")

    # 语法检查 (内存编译 + py_compile)
    try:
        compile(out, str(DST), "exec")
    except SyntaxError as e:
        DST.unlink(missing_ok=True)
        sys.exit(f"[FAIL] 生成文件语法错误, 已回滚删除: {e}")
    r = subprocess.run([sys.executable, "-m", "py_compile", str(DST)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        DST.unlink(missing_ok=True)
        sys.exit(f"[FAIL] py_compile 失败, 已回滚: {r.stderr}")
    print("[4/5] py_compile 通过")

    # setup.py 入口
    s = SETUP.read_text(encoding="utf-8")
    if "drone_drl.f_r3:main" in s:
        print("[5/5] setup.py 已有 f_r3 入口, 跳过")
    else:
        if s.count(ENTRY_AFTER) != 1:
            sys.exit("[FAIL] setup.py 锚点不唯一, 请手工加 'f_r3 = drone_drl.f_r3:main'")
        bak = SETUP.with_suffix(f".py.bak.{STAMP}_f_r3_entry")
        shutil.copy2(SETUP, bak)
        SETUP.write_text(s.replace(ENTRY_AFTER, ENTRY_AFTER + ENTRY, 1), encoding="utf-8")
        print(f"[5/5] setup.py 已加入口 (备份 {bak.name})")

    print("\n===== 与源文件的差异 (完整) =====")
    for ln in difflib.unified_diff(orig.split("\n"), out.split("\n"),
                                   "source_f.py", "f_r3.py", lineterm="", n=1):
        print("  " + ln)
    print(f"\n✅ 完成: {DST}  sha256={sha(DST)}")
    print("   下一步: cd ~/SITL && colcon build --packages-select drone_drl")
    return 0


if __name__ == "__main__":
    sys.exit(main())
