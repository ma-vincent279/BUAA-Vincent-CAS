#!/bin/bash
# 子项目 h 实机一键启动 (基底 = 项目 e, 只改释放判据; 2026-09-24 新建)
# 用法 (Jetson): bash ~/fly/g.sh   [--no-build]
set -e
cd "$(dirname "$0")"
if [ "$1" != "--no-build" ]; then
    source /opt/ros/humble/setup.bash
    colcon build --packages-select fly 2>&1 | tail -5
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 run fly h
