#!/bin/bash
# 把顶层的工作空间同步进归档目录 (改完代码后跑一次)
set -e
cd "$(dirname "$0")"
echo "同步 仿真工作空间 → SITL/工作空间/ ..."
rm -rf SITL/工作空间/src SITL/工作空间/worlds SITL/工作空间/gz_models*
cp -r src worlds gz_models gz_models_combined gz_models_cuadc gz_worlds gz_worlds_cuadc_px4 SITL/工作空间/
echo "同步 实机工作空间 → 实机/工作空间/ ..."
rm -rf 实机/工作空间/fly
cp -r fly 实机/工作空间/
rm -rf 实机/工作空间/fly/src/px4_msgs/.git
echo "同步 代码 → SITL/代码/ 与 实机/代码/ ..."
cp src/drone_drl/drone_drl/k.py SITL/代码/任务节点/ 2>/dev/null || true
cp fly/src/fly/fly/k.py 实机/代码/任务节点/ 2>/dev/null || true
cp fly/src/fly/fly/see.py 实机/代码/视觉节点/ 2>/dev/null || true
echo "✓ 同步完成。别忘了 git add -A && git commit"
