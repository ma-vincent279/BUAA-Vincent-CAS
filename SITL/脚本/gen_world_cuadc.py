#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_world_cuadc.py — 按规则书生成投放场地 + 真值文件 (2026-10-11)

为什么要生成器
--------------
旧做法: `worlds/*.world` 手抄 → `test_*.sh` 里再把同一组桶坐标**手抄两遍**
         (一遍给检测器 `bucketN_gz_*`, 一遍给裁判 `bucketN_gz_*`)。
⇒ 三处必须人工同步, 没有任何机器把关。
他们的 sim 就是这个坑的极端版: `virtual_drop_judge_node.cpp` 把桶硬编码在
`(30,±1.2)`, 而 `generate_scene.py` 每次随机撒桶 (seed 2026 实为 (28.44,0.02)…)
⇒ 偏差 2.0~3.5m, 而 A 区半径只有 7.5~12.5cm ⇒ **每一投都判 0 分**。

本脚本的做法: **生成一次, 写出两份产物, 且真值只有一份** ——
   worlds/<name>.world        Gazebo Classic 世界
   worlds/<name>.truth.json   唯一真值源 (桶位/桶径/分值/区域/生成点偏移)
检测器与裁判都**只准从 truth.json 读坐标**, 不许再手抄。

规则 (2026 CUADC 多旋翼无人机侦察与救援, 与哈工大开源仓库 RULE_MAPPING 一致)
--------------------------------------
  起降点前 30m 为投放区, 尺寸 8m 长(横向) × 5m 纵深, 3 个高 30cm 白色圆筒:
      1 号 直径 15cm → 落桶内 A 区 500 分
      2 号 直径 20cm → 落桶内 A 区 300 分
      3 号 直径 25cm → 落桶内 A 区 100 分
      B 区: 距桶心 0.5m 内 → 50 分
  投放区前方 20m 为侦察区 (距起降点 55m), 5 个高 15cm / 直径 20cm 白筒

用法
----
  python3 scripts/gen_world_cuadc.py --name k --seed 2026        # 生成
  python3 scripts/gen_world_cuadc.py --name k --seed 3           # 换布局
  python3 scripts/gen_world_cuadc.py --name k --print            # 只打印真值
  python3 scripts/gen_world_cuadc.py --list                      # 列出已生成的
"""
import argparse
import json
import math
import pathlib
import random
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
WORLDS = REPO / "worlds"
TEMPLATE = WORLDS / "i.world"          # 复用它的地面/太阳/边框/角标, 不重抄

# ---- Gazebo 世界坐标 ↔ PX4 NED (相对起飞点) 的生成点偏移 (sitl_run.sh:130) ----
SPAWN_GZ = (1.01, 0.98, 0.83)

# ---- 区域 (NED, 相对起飞点; NED 单位 m, y=前=航向) ----
DROP_NED_Y = (30.0, 35.0)              # 投放区纵深 (后边 30, 前边 35)
RECON_NED_Y = (55.0, 60.0)             # 侦察区纵深
ZONE_HALF_W = 4.0                      # 两区横向半宽 (规则 8m 宽)

# ---- 桶规格 (与规则书逐条对应) ----
DROP_SPECS = [                         # (编号, 直径 m, 高 m, A 区分值, 颜色)
    (1, 0.15, 0.30, 500, "white"),
    (2, 0.20, 0.30, 300, "white"),
    (3, 0.25, 0.30, 100, "white"),
]
RECON_SPECS = [(i, 0.20, 0.15) for i in range(1, 6)]
MIN_SEP_DROP = 0.75                    # 投放桶最小间距 (他们的值)
MIN_SEP_RECON = 0.70
EDGE_MARGIN = 0.55                     # 距区域边界的最小留白

B_ZONE_RADIUS = 0.5
B_ZONE_SCORE = 50


# ============================================================
# 坐标换算
# ============================================================
def ned_to_gz(ned_x, ned_y, ned_z=0.0):
    """NED (相对起飞点) → Gazebo 世界坐标."""
    return (ned_y + SPAWN_GZ[0], ned_x + SPAWN_GZ[1], SPAWN_GZ[2] - ned_z)


def gz_to_ned(gx, gy, gz):
    return (gy - SPAWN_GZ[1], gx - SPAWN_GZ[0], SPAWN_GZ[2] - gz)


# ============================================================
# 采样
# ============================================================
def sample_points(rng, y_lo, y_hi, half_w, count, min_sep, margin):
    """在 [y_lo+margin, y_hi-margin] × [-half_w+margin, half_w-margin] 内采样,
    保证两两间距 >= min_sep。失败则抛异常 (不静默退化成重叠布局)。"""
    y_lo, y_hi = y_lo + margin, y_hi - margin
    x_lo, x_hi = -half_w + margin, half_w - margin
    pts = []
    for _ in range(20000):
        if len(pts) == count:
            return pts
        cand = (rng.uniform(x_lo, x_hi), rng.uniform(y_lo, y_hi))
        if all(math.hypot(cand[0] - p[0], cand[1] - p[1]) >= min_sep for p in pts):
            pts.append(cand)
    raise RuntimeError(f"无法在 {count} 个点 / min_sep={min_sep} 下采样 (提高区域或降低间距)")


# ============================================================
# SDF 片段
# ============================================================
def bucket_sdf(name, ned_x, ned_y, diameter, height, color="white", collision=True):
    """地面上的实心圆柱 (底面贴地 ⇒ 圆心 z = height/2)。
    顶部加一圈薄环, 给真实视觉留一个"桶口"特征 (他们的模型也这么做)。"""
    r = diameter / 2.0
    cx, cy, _ = ned_to_gz(ned_x, ned_y, 0.0)
    cz = height / 2.0
    rgb = {"white": "0.92 0.92 0.92 1", "red": "1 0 0 1"}[color]
    col = (f"""        <collision name="bucket_collision">
          <geometry>
            <cylinder><radius>{r:.4f}</radius><length>{height:.4f}</length></cylinder>
          </geometry>
        </collision>
""" if collision else
           "        <!-- 2026-10-12: 无碰撞体变体 (探针实测: 桶的 collision 会让正上方悬停变差 ~9.5cm) -->\n")
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="bucket_link">
{col}
        <visual name="bucket_visual">
          <geometry>
            <cylinder><radius>{r:.4f}</radius><length>{height:.4f}</length></cylinder>
          </geometry>
          <material>
            <ambient>{rgb}</ambient>
            <diffuse>{rgb}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <visual name="bucket_rim">
          <pose>0 0 {height / 2.0:.4f} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{r * 1.04:.4f}</radius><length>0.012</length></cylinder>
          </geometry>
          <material>
            <ambient>0.08 0.08 0.08 1</ambient>
            <diffuse>0.08 0.08 0.08 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(template: str, buckets, recons, world_name: str, collision=True) -> str:
    """把模板里的 3 个 target_bucket_N 块换成新桶, 并在侦察区插入侦察筒。"""
    out = template
    # 世界名
    out = re.sub(r'<world name="[^"]*">', f'<world name="{world_name}">', out, count=1)

    # 投放桶: 整块替换
    blocks = "".join(
        bucket_sdf(f"target_bucket_{i}", b["ned_x"], b["ned_y"], b["diameter"], b["height"],
                   collision=collision)
        for i, b in enumerate(buckets, start=1)
    )
    out, n = re.subn(
        r'    <model name="target_bucket_\d+">.*?</model>\n',
        "", out, flags=re.S,
    )
    assert n == 3, f"模板里 target_bucket 块数 = {n}, 预期 3"

    # 插到投放区角标之后、侦察区之前 (第一个 <model name="scout_border 之前)
    anchor = out.index('    <model name="scout_border')
    banner = ("    <!-- ===== 投放桶 (规则: 直径 15/20/25cm, 高 30cm, 白色; 落桶内 A 区) ===== -->\n")
    out = out[:anchor] + banner + blocks + "\n" + out[anchor:]

    # 侦察筒
    recon_blocks = "".join(
        bucket_sdf(f"recon_bucket_{i}", r["ned_x"], r["ned_y"], r["diameter"], r["height"],
                   collision=collision)
        for i, r in enumerate(recons, start=1)
    )
    tail = out.rindex("  </world>")
    out = (out[:tail]
           + "    <!-- ===== 侦察筒 (规则: 直径 20cm, 高 15cm, 白色) ===== -->\n"
           + recon_blocks + "\n" + out[tail:])
    return out


# ============================================================
# 主流程
# ============================================================
def generate(seed: int):
    rng = random.Random(seed)

    drop_pts = sample_points(rng, DROP_NED_Y[0], DROP_NED_Y[1], ZONE_HALF_W,
                             len(DROP_SPECS), MIN_SEP_DROP, EDGE_MARGIN)
    # 规格按【直径】随机分配到采样点上 —— 否则规则里"1 号小桶"永远待在同一个角落
    specs = DROP_SPECS[:]
    rng.shuffle(specs)
    buckets = []
    for (num, dia, h, score, color), (nx, ny) in zip(specs, drop_pts):
        buckets.append({
            "id": f"drop_{num}", "num": num,
            "ned_x": round(nx, 3), "ned_y": round(ny, 3),
            "diameter": dia, "radius": dia / 2.0, "height": h,
            "a_zone_score": score, "color": color,
        })

    recon_pts = sample_points(rng, RECON_NED_Y[0], RECON_NED_Y[1], ZONE_HALF_W,
                              len(RECON_SPECS), MIN_SEP_RECON, EDGE_MARGIN)
    recons = []
    for (num, dia, h), (nx, ny) in zip(RECON_SPECS, recon_pts):
        recons.append({
            "id": f"recon_{num}", "ned_x": round(nx, 3), "ned_y": round(ny, 3),
            "diameter": dia, "radius": dia / 2.0, "height": h,
        })

    # 升序排列 = 便于人读; 顺序不影响任何逻辑
    buckets.sort(key=lambda b: b["ned_y"])
    recons.sort(key=lambda r: r["ned_y"])

    truth = {
        "scene": "cuadc_2026_drop_recon",
        "seed": seed,
        "generated_by": "scripts/gen_world_cuadc.py",
        "coordinate_note": ("ned_* 为相对起飞点(飞机 Gazebo 生成点)的 NED 偏移; "
                            "gz 坐标 = ned_to_gz(); 世界原点不是 NED 原点"),
        "spawn_gz": {"x": SPAWN_GZ[0], "y": SPAWN_GZ[1], "z": SPAWN_GZ[2]},
        "zones": {
            "drop": {"ned_y_lo": DROP_NED_Y[0], "ned_y_hi": DROP_NED_Y[1],
                     "half_width": ZONE_HALF_W,
                     "length": DROP_NED_Y[1] - DROP_NED_Y[0], "width": 2 * ZONE_HALF_W},
            "recon": {"ned_y_lo": RECON_NED_Y[0], "ned_y_hi": RECON_NED_Y[1],
                      "half_width": ZONE_HALF_W,
                      "length": RECON_NED_Y[1] - RECON_NED_Y[0], "width": 2 * ZONE_HALF_W},
        },
        "scoring": {
            "rule": "d <= bucket.radius -> A 区, 得 bucket.a_zone_score; "
                    "d <= 0.5 -> B 区, 得 50; 否则 0",
            "b_zone_radius": B_ZONE_RADIUS, "b_zone_score": B_ZONE_SCORE,
            "max_score_2_buckets": 800,
            "best_pair": "直径最小的两个桶 (500 + 300)",
        },
        "buckets": buckets,
        "recon_buckets": recons,
    }
    return truth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="k", help="项目名 → worlds/<name>.world")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--print", dest="do_print", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-collision", dest="no_collision", action="store_true",
                    help="桶不带 collision 几何 (只留 visual); 用于隔离「桶对载具的物理扰动」")
    args = ap.parse_args()

    if args.list:
        for p in sorted(WORLDS.glob("*.truth.json")):
            t = json.loads(p.read_text(encoding="utf-8"))
            print(f"{p.name:24s} seed={t['seed']}")
        return 0

    truth = generate(args.seed)
    if args.do_print:
        print(json.dumps(truth, ensure_ascii=False, indent=2))
        return 0

    world_path = WORLDS / f"{args.name}.world"
    truth_path = WORLDS / f"{args.name}.truth.json"

    tpl = TEMPLATE.read_text(encoding="utf-8")
    world_path.write_text(build_world(tpl, truth["buckets"], truth["recon_buckets"],
                                      f"cuadc_{args.name}",
                                      collision=not args.no_collision), encoding="utf-8")
    truth_path.write_text(json.dumps(truth, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")

    print(f"[gen] seed={args.seed}")
    print(f"[gen] {world_path.relative_to(REPO)}  ({world_path.stat().st_size} B)")
    print(f"[gen] {truth_path.relative_to(REPO)}")
    print()
    print("  桶          直径     A区    NED(x, y)")
    for b in truth["buckets"]:
        print(f"  {b['id']:10s} {b['diameter']*100:4.0f}cm  {b['a_zone_score']:4d}    "
              f"({b['ned_x']:+.2f}, {b['ned_y']:.2f})")
    d = [b["diameter"] for b in sorted(truth["buckets"], key=lambda b: b["diameter"])[:2]]
    print(f"\n  最优两桶 (直径最小两个): {d[0]*100:.0f}cm + {d[1]*100:.0f}cm "
          f"⇒ 上限 {sum(b['a_zone_score'] for b in sorted(truth['buckets'], key=lambda b: b['diameter'])[:2])} 分")
    return 0


if __name__ == "__main__":
    sys.exit(main())
