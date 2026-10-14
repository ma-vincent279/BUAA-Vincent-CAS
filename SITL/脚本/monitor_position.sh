#!/bin/bash
# ================================================================
# 实时 NED 坐标监视器
#   用法: bash ~/SITL/scripts/monitor_position.sh
#   监视 /fmu/out/vehicle_local_position_v1 话题的实时 NED 坐标。
#   按 Ctrl-C 退出。
# ================================================================
set -e
SITL_DIR="/home/vincent/SITL"
source "$SITL_DIR/install/setup.bash" 2>/dev/null || {
    echo "请先编译: cd ~/SITL && colcon build --packages-select drone_drl && source install/setup.bash"
    exit 1
}
exec python3 "$SITL_DIR/scripts/monitor_position.py"
