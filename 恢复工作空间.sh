#!/bin/bash
# 从归档恢复出**可运行的工作空间** —— 刚 clone 下来之后跑一次
#
# 为什么需要这个: 顶层的工作目录 (src/ fly/ scripts/ worlds/ gz_models*/ gz_worlds*/)
# 是**本地运行用**的, 为了方便阅读, 仓库里把它们整理进了中文目录:
#     SITL/工作空间/   ← 仿真工作空间 (src/ worlds/ gz_models*/ gz_worlds*/)
#     实机/工作空间/    ← 实机工作空间 (fly/)
#     SITL/脚本/       ← 全部脚本
# 本脚本把它们恢复到顶层, 恢复后就能 colcon build / bash scripts/test_k.sh 了。
set -e
cd "$(dirname "$0")"

echo "① 恢复仿真工作空间 → 顶层 ..."
for d in src worlds gz_models gz_models_combined gz_models_cuadc gz_worlds gz_worlds_cuadc_px4; do
  if [ -e "SITL/工作空间/$d" ]; then cp -r "SITL/工作空间/$d" .; echo "   $d ✓"; fi
done

echo "② 恢复脚本 → 顶层 scripts/ ..."
mkdir -p scripts && cp -r SITL/脚本/* scripts/ 2>/dev/null || true
chmod +x scripts/*.sh 2>/dev/null || true
echo "   scripts/ ✓ ($(ls -1 scripts/ | wc -l) 个)"

echo "③ 恢复实机工作空间 → 顶层 fly/ ..."
[ -e "实机/工作空间/fly" ] && { cp -r 实机/工作空间/fly .; echo "   fly/ ✓"; }

echo
echo "✓ 恢复完成。接下来:"
echo "    colcon build --packages-select drone_drl"
echo "    source install/setup.bash"
echo "    python3 scripts/test_k_servo_20261011.py     # 离线验收 104 项"
echo "    bash scripts/test_k.sh                       # 跑一轮仿真"
echo
echo "⚠️ 还没做的一步: Gazebo 模型的网格 (meshes/materials) 需从 PX4 拷贝,"
echo "   见 技术文档/配置仿真环境.md 第三节。"
