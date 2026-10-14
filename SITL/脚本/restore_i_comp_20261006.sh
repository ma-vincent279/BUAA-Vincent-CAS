#!/bin/bash
# 恢复 i.py 的真实补偿 (对照实验后必跑): 从 /tmp/i_py_backup_zerocomp 还原 + 重编
set -e
cd ~/SITL
cp -f /tmp/i_py_backup_zerocomp/drone_i.py src/drone_drl/drone_drl/i.py
cp -f /tmp/i_py_backup_zerocomp/fly_i.py   fly/src/fly/fly/i.py
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_drl 2>&1 | tail -2
grep -nE "^PAYLOAD_OFFSET_BODY_(Y_B1|Y_B2)" src/drone_drl/drone_drl/i.py
