#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_world_cuadc_gz.py — 把 CUADC 场地从 **Gazebo Classic** 机械转换成 **gz-sim (Harmonic)**

原则 (与 `gen_world_cuadc.py` 同一套"单一真值来源"思想):
  · **只改"世界级"的东西**: SDF 版本 / 世界插件 / 太阳 / 地面
  · **桶、边框、角标的几何逐字节保留** ⇒ 与 `worlds/k.truth.json` 仍然一一对应
    (即: 转换后不需要重新测桶坐标, 检测器/裁判的参数一个字都不用改)
  · 幂等: 重复运行结果一致; `--check` 只校验不写

用法:
  python3 scripts/gen_world_cuadc_gz.py                 # k.world → worlds/k_gz.sdf
  python3 scripts/gen_world_cuadc_gz.py --check         # 校验
  python3 scripts/gen_world_cuadc_gz.py --src worlds/k.world --name k_gz

输出还会同步一份到 `~/SITL/gz_worlds/<name>.sdf`(PX4_GZ_WORLDS 用, 不必改 PX4 树)。
"""
import argparse
import pathlib
import re
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GZ_WORLDS = pathlib.Path.home() / "SITL" / "gz_worlds"

# gz-sim (Harmonic) 的世界级插件 —— 与他们的 cuadc_rescue_single.sdf 同款
GZ_PLUGINS = """    <!-- ⚠ 这里**故意不放任何 system 插件** —— PX4 的 gz 用 `server.config`
         (GZ_SIM_SERVER_CONFIG_PATH = src/modules/simulation/gz_bridge/server.config)
         **全局加载** physics/imu/air-pressure/magnetometer/navsat/sensors 等系统。
         若世界文件里再声明一遍会造成重复注册, 实测后果是 **磁罗盘与气压计丢失**
         (`Preflight Fail: Found 0 compass` / `barometer 0 missing`) ⇒ 拒绝解锁。
         PX4 自带的 Tools/simulation/gz/worlds/default.sdf 同样是零插件的。 -->
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.78 0.80 0.82</background>
      <sky/>
    </scene>

    <!-- ⚠ 必须有球面坐标: gz 的 Magnetometer/NavSat 系统用它算地磁与经纬度。
         Classic 的世界没有这个元素, 不加会报 `Preflight Fail: Strong magnetic interference`。
         这里用 PX4/ArduPilot SITL 惯用的 CMAC (Canberra) 原点, 与他们的世界一致。 -->
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <!-- Classic 的 model://sun 在这里换成 inline 平行光源 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.6 0.6 0.6 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.45 0.15 -0.88</direction>
    </light>

    <!-- Classic 的 model://ground_plane 在这里换成 inline 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def convert(src: pathlib.Path) -> str:
    s = src.read_text(encoding="utf-8")

    # 1) SDF 版本 1.6 → 1.9
    s = re.sub(r'<sdf version="1\.6">', '<sdf version="1.9">', s, count=1)

    # 2) 世界插件: 删掉 Classic 的 libgazebo_ros_*.so, 插入 gz-sim 插件组
    s = re.sub(r'^\s*<plugin[^>]*libgazebo[^>]*/>\s*$\n', '', s, flags=re.M)
    m = re.search(r'<world name="([^"]+)">', s)
    if not m:
        raise SystemExit("找不到 <world name=...>, 源文件不是预期格式")
    world_name = m.group(1)
    s = s[:m.end()] + "\n" + GZ_PLUGINS + s[m.end():]

    # 3) 去掉 Classic 的 model://sun / model://ground_plane include (已换成 inline)
    s = re.sub(
        r'\s*<include>\s*<uri>model://(?:sun|ground_plane)</uri>\s*</include>', '', s
    )
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(ROOT / "worlds" / "k.world"))
    ap.add_argument("--name", default="k_gz", help="输出世界名 (PX4_GZ_WORLD 用它)")
    ap.add_argument("--check", action="store_true", help="只校验不写")
    a = ap.parse_args()

    src = pathlib.Path(a.src)
    if not src.exists():
        raise SystemExit(f"源世界不存在: {src}")
    out_text = convert(src)
    # 世界名要与 PX4_GZ_WORLD 一致
    out_text = re.sub(r'<world name="[^"]+">', f'<world name="{a.name}">', out_text, count=1)
    # 幂等: 保证只插入一次插件组
    if 'gz-sim-physics-system' in out_text:
        raise SystemExit("世界文件里不应出现 gz-sim system 插件 (会与 server.config 重复注册)")

    # XML 合法性
    import xml.etree.ElementTree as ET
    try:
        ET.fromstring(out_text)
    except ET.ParseError as e:
        raise SystemExit(f"生成的 XML 不合法: {e}")

    dst = ROOT / "worlds" / f"{a.name}.sdf"
    gz_dst = GZ_WORLDS / f"{a.name}.sdf"

    n_bucket = len(re.findall(r'<model name="target_bucket_', out_text))
    n_border = len(re.findall(r'<model name="\w*border', out_text))
    print(f"源: {src}")
    print(f"世界名: {a.name}")
    print(f"  桶模型: {n_bucket} 个 | 边框模型: {n_border} 个 (几何逐字节保留)")
    print("  SDF 版本: 1.9 | system 插件: 0 个(交给 server.config 全局加载) | 太阳/地面: inline")
    print(f"  输出: {dst}")
    print(f"        {gz_dst}  (PX4_GZ_WORLDS 用)")
    if a.check:
        if dst.exists() and dst.read_text(encoding="utf-8") == out_text:
            print("  --check: 已是最新 ✓")
            return 0
        print("  --check: 与磁盘上的不一致 ✗")
        return 1

    dst.write_text(out_text, encoding="utf-8")
    GZ_WORLDS.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(dst, gz_dst)
    print("  写入完成 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
