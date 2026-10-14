#!/usr/bin/env python3
"""静态标定参数落地 (2026-10-09) —— 把现场实测的 7 个常量写进 4 个节点文件

依据 (2026-10-09 现场静态标定, 全部实测, 无飞行拟合)
--------------------------------------------------
| 参数 | 新值 | 依据 |
|---|---|---|
| `TARGET_PLANE_H` | **-0.278** | 卷尺 `A(相机镜头离地 0.022) − R(桶口离地 0.300)`; 与相机无关 |
| `HFOV` / `VFOV` | **89.8° / 58.5°** | ρ 扫描 4 点最小二乘 `Δcx = 355.05·ρ + 32.30` (残差 ≤3.6px=1.0cm) × 卷尺深度 0.905 ⇒ **FX=321.3**; 16:9 自洽 ⇒ FY=321.3 |
| `Y_B1` | **+0.146** | 相机常数 0.0910 (安装角实测 Δcx₀=+33.5px) + 机械 +0.055 (铅垂线) |
| `Y_B2` | **+0.036** | 相机常数 0.0910 + 机械 −0.055 |
| `XF_B1` / `XF_B2` | **-0.021** | 相机常数 −0.0208 (Δcy₀=+8.5px) + 机械 0 |

旧值来源: `Y/XF` 为 2026-10-07 的 8 趟实飞拟合; `TARGET_PLANE_H`/`HFOV`/`VFOV` 为历史默认。
**新值把"拟合值"分解成"相机安装常数 + 机械偏移"两项, 两项都直接实测** ⇒ 不再依赖飞行数据。

⚠️ 不动 `HEADING_DEG` (板上为 212.0, 仓库为 180.0, 由部署脚本单独处理)。
⚠️ 运输/拆装后 (相机/机架/投放机构/换桶) 必须重标。

用法: python3 scripts/patch_calib_20261009.py [--check]
"""
import argparse
import hashlib
import pathlib
import re
import shutil
import sys
from datetime import datetime

ROOT = pathlib.Path(__file__).resolve().parents[1]
TS = datetime.now().strftime("%Y%m%d_%H%M%S")

FILES = [
    ROOT / "src/drone_drl/drone_drl/k.py",
    ROOT / "src/drone_drl/drone_drl/l.py",
    ROOT / "fly/src/fly/fly/k.py",
    ROOT / "fly/src/fly/fly/l.py",
]

# (常量名, 新行, 变更说明)  —— 锚点用 ^NAME\s*= 精确匹配定义行, 断言唯一
NEW = [
    ("TARGET_PLANE_H",
     "TARGET_PLANE_H = -0.278   # 2026-10-09 静态实测: 卷尺 A(0.022) − R(0.300); 原 0.0",
     "0.0 -> -0.278"),
    ("HFOV",
     "HFOV = math.radians(89.8)   # 2026-10-09 静态实测 FX=321.3px (ρ 扫描四点拟合); 原 85",
     "85 -> 89.8"),
    ("VFOV",
     "VFOV = math.radians(58.5)   # 2026-10-09 16:9 自洽 (FY=FX=321.3); 原 54",
     "54 -> 58.5"),
    ("PAYLOAD_OFFSET_BODY_Y_B1",
     "PAYLOAD_OFFSET_BODY_Y_B1 = +0.146   # 2026-10-09 实测: 相机常数 +0.0910 & 机械 +0.055; 原 +0.18 (2026-10-07 飞行拟合)",
     "+0.18 -> +0.146"),
    ("PAYLOAD_OFFSET_BODY_Y_B2",
     "PAYLOAD_OFFSET_BODY_Y_B2 = +0.036   # 2026-10-09 实测: 相机常数 +0.0910 & 机械 -0.055; 原 0.00",
     "0.00 -> +0.036"),
    ("PAYLOAD_OFFSET_BODY_XF_B1",
     "PAYLOAD_OFFSET_BODY_XF_B1 = -0.021   # 2026-10-09 实测: 相机常数 -0.0208 & 机械 0; 原 +0.05",
     "+0.05 -> -0.021"),
    ("PAYLOAD_OFFSET_BODY_XF_B2",
     "PAYLOAD_OFFSET_BODY_XF_B2 = -0.021   # 2026-10-09 实测: 同 XF_B1; 原 +0.05",
     "+0.05 -> -0.021"),
]


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def apply_one(path, check_only=False):
    src = path.read_text(encoding="utf-8")
    orig = src
    changed = []
    for name, newline, note in NEW:
        pat = re.compile(rf"^{re.escape(name)}\s*=.*$", re.M)
        hits = pat.findall(src)
        if len(hits) != 1:
            sys.exit(f"[{path.name}] 锚点断言失败: {name} 期望 1 处, 实际 {len(hits)} 处")
        src = pat.sub(lambda m, nl=newline: nl, src, count=1)
        changed.append(f"{name}: {note}")
    # HEADING_DEG 必须原样未动
    if "HEADING_DEG" in orig:
        h_old = [l for l in orig.splitlines() if l.startswith("HEADING_DEG")]
        h_new = [l for l in src.splitlines() if l.startswith("HEADING_DEG")]
        if h_old != h_new:
            sys.exit(f"[{path.name}] HEADING_DEG 被意外改动, 中止")
    if not check_only:
        bak = path.with_suffix(f".py.bak.{TS}_calib20261009")
        shutil.copy2(path, bak)
        path.write_text(src, encoding="utf-8")
    return changed, sha(path) if not check_only else sha(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只做锚点检查, 不写文件")
    args = ap.parse_args()

    for f in FILES:
        if not f.exists():
            sys.exit(f"缺少 {f}")

    print("=" * 84)
    print(f"静态标定参数落地 (2026-10-09)   {'[--check 不写文件]' if args.check else ''}")
    print("=" * 84)
    for f in FILES:
        before = sha(f)
        changed, after = apply_one(f, check_only=args.check)
        tag = "SITL" if "src/drone_drl" in str(f) else "fly "
        print(f"\n  [{tag}] {f.relative_to(ROOT)}")
        print(f"        sha {before[:12]} -> {after[:12]}")
        for c in changed:
            print(f"        · {c}")

    if args.check:
        print("\n--check 通过 (未写文件)")
        return 0

    print("\n" + "=" * 84)
    print("完成。下一步:")
    print("  colcon build --packages-select drone_drl")
    print("  python3 scripts/test_calib_params_20261009.py")
    print("  bash scripts/deploy_calib_jetson_20261009.sh          # 部署 (含 --rollback)")
    print("=" * 84)
    return 0


if __name__ == "__main__":
    sys.exit(main())
