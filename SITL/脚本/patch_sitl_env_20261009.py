#!/usr/bin/env python3
"""把 SITL 侧的环境相关常量改回"与 SITL 世界自洽"的值 (2026-10-09)

背景
----
2026-10-09 的静态标定把**实机实测值**写进了 k/l 的四份文件, 但其中 3 个常量
(SITL 侧) 本来是由**环境和仿真器**决定的, 不是由实机决定的:

| 常量 | SITL 世界的规定者 | SITL 正确值 | 实机正确值 |
|---|---|---|---|
| `TARGET_PLANE_H` | `yolov8_bucket_detector._gt_ground_circles()`: 它把桶当成**参考平面上的一个圆**投影 (`dz = -drone_z`, 不含桶高) ⇒ 与 `_roi_to_ned` 的平面求交**互为逆运算** | **0.0** | −0.278 |
| `HFOV` / `VFOV` | 同一个检测器**硬编码** `hfov=radians(85) / vfov=radians(54)` (yolov8_bucket_detector.py:261-264) 并用 `self.fx/self.fy` 生成像素 ⇒ 节点必须同参, 否则凭空多出 `349.22/321.12 = 1.0875` 的尺度误差 | **85° / 54°** | 89.8° / 58.5° |

写入实机值后的后果 (实测): 默认 `project_mode=ground` 下节点深度从 `H` 变成
`H−0.278` (H=1.2 时低估 23%), 再叠加 8.75% 的尺度误差 ⇒ SITL 里"落点偏短"
是**仿真失真**, 不是标定坏了, 会误导判断。

本脚本**只改 SITL 两份** (`src/drone_drl/drone_drl/{k,l}.py`), 绝不碰 fly/ 与板载。
`FX`/`FY` 由 `HFOV`/`VFOV` 推导 (k.py:270-271), 因此只改这两个即可。

判据 (由 scripts/check_drop_portability_20261009.py D 节机器核对)
--------------------------------------------------------------
SITL 往返误差 = |`_roi_to_ned`(检测器 ground 投影框) − 桶真实水平位置| 必须 **恒为 0**。

用法
----
    python3 scripts/patch_sitl_env_20261009.py            # 就地打补丁 (自动备份)
    python3 scripts/patch_sitl_env_20261009.py --check    # 只检查, 不改任何东西
"""
import datetime
import pathlib
import re
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
# 只允许这两个文件 —— 严防误伤实机/板载
TARGETS = [ROOT / "src/drone_drl/drone_drl/k.py",
           ROOT / "src/drone_drl/drone_drl/l.py"]

# (常量名, 目标行, 锚点正则)
PATCHES = [
    ("TARGET_PLANE_H",
     "TARGET_PLANE_H = 0.0     "
     "# SITL: 检测器 ground 模式把桶当'参考平面上的圆' ⇒ 与 _roi_to_ned 互为逆运算, 必须 0.0;"
     " 实机静态标定 = -0.278 (镜头离地 0.022 − 桶口离地 0.300)",
     r"^TARGET_PLANE_H\s*=.*$"),
    ("HFOV",
     "HFOV = math.radians(85)   "
     "# SITL: 必须与 yolov8_bucket_detector 的模拟相机一致 (它硬编码 85°/54° 并用 self.fx 生成像素);"
     " 实机实测 = 89.8° (FX=321.3px)",
     r"^HFOV\s*=.*$"),
    ("VFOV",
     "VFOV = math.radians(54)   "
     "# SITL: 同上; 实机实测 = 58.5°",
     r"^VFOV\s*=.*$"),
]


def main():
    check_only = "--check" in sys.argv
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    changed = 0

    for path in TARGETS:
        assert path.exists(), f"不存在: {path}"
        # 安全闸: 只允许 SITL 侧的 drone_drl 包
        assert "fly" not in path.parts, f"拒绝改动非 SITL 文件: {path}"
        text = path.read_text(encoding="utf-8")
        new = text
        todo = []
        for name, target, pat in PATCHES:
            hits = re.findall(pat, new, re.M)
            assert len(hits) == 1, f"{path.name}: {name} 锚点命中 {len(hits)} 次 (要求恰好 1)"
            if hits[0] == target:
                print(f"  · {path.name}: {name} 已是目标形式, 跳过")
            else:
                todo.append((name, target, pat, hits[0]))
        if not todo:
            print(f"  ✓ {path.name}: 无需改动")
            continue

        if check_only:
            for name, target, _p, old in todo:
                print(f"  ✗ {path.name}: {name} 需要改\n      旧: {old[:90]}\n      新: {target[:90]}")
            changed += len(todo)
            continue

        shutil.copy2(path, f"{path}.bak.{ts}_sitl_env")
        for name, target, pat, old in todo:
            new2 = re.sub(pat, lambda _m: target, new, count=1, flags=re.M)
            assert new2 != new, f"{name} 替换无效"
            new = new2
            print(f"  ✓ {path.name}: {name}\n      旧: {old[:88]}\n      新: {target[:88]}")
        # 二次自检: 三个常量都到位, 且行数不变
        for name, target, pat in PATCHES:
            assert re.findall(pat, new, re.M) == [target], f"{name} 未生效"
        assert len(new.split(chr(10))) == len(text.split(chr(10))), "行数变了!"
        path.write_text(new, encoding="utf-8")
        print(f"    (备份 {path.name}.bak.{ts}_sitl_env)")
        changed += len(todo)

    print()
    if check_only:
        print(f"检查完成: {changed} 处需要改" if changed else "检查完成: SITL 侧环境常量已是目标值 ✓")
        return 1 if changed else 0
    print("完成。下一步: python3 scripts/check_drop_portability_20261009.py k l"
          "   (D 节会核对 SITL 往返误差恒为 0)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
