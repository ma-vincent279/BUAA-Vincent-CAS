#!/bin/bash
# 把顶层的工作空间同步进归档目录 (改完代码后跑一次)
set -e
cd "$(dirname "$0")"
echo "同步 仿真工作空间 → SITL/工作空间/ ..."
rm -rf SITL/工作空间/src SITL/工作空间/worlds SITL/工作空间/gz_models* SITL/工作空间/gz_worlds*
cp -r src worlds gz_models gz_models_combined gz_models_cuadc gz_worlds gz_worlds_cuadc_px4 SITL/工作空间/
# ★ 归档的 src 会让 colcon 看到重名包 ⇒ 必须标记忽略
touch SITL/工作空间/src/COLCON_IGNORE SITL/工作空间/src/drone_drl/COLCON_IGNORE
echo "同步 脚本 → SITL/脚本/ ..."
rm -rf SITL/脚本 && mkdir -p SITL/脚本 && cp -r scripts/* SITL/脚本/
echo "同步 实机工作空间 → 实机/工作空间/ ..."
rm -rf 实机/工作空间/fly && cp -r fly 实机/工作空间/
touch 实机/工作空间/fly/src/COLCON_IGNORE 实机/工作空间/fly/src/fly/COLCON_IGNORE 2>/dev/null || true
rm -rf 实机/工作空间/fly/src/px4_msgs/.git 实机/工作空间/fly/build 实机/工作空间/fly/install 2>/dev/null || true
echo "同步 实机脚本 → 实机/脚本/ ..."
rm -rf 实机/脚本 && mkdir -p 实机/脚本 && cp -r fly/*.sh 实机/脚本/ 2>/dev/null || true
echo "✓ 同步完成。别忘了 git add -A && git commit && git push"
