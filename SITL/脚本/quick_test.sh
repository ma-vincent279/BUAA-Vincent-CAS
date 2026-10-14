#!/bin/bash
# Headless 快速仿真验证脚本
SITL_DIR="/home/vincent/SITL"
PX4_DIR="/home/vincent/PX4-Autopilot"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SLEEP_BETWEEN=2

# 清理
for port in 11345 8888; do for pid in $(lsof -ti:$port 2>/dev/null); do kill -9 $pid 2>/dev/null; done; done
pkill -9 -x px4 2>/dev/null; pkill -9 -x gzserver 2>/dev/null; pkill -9 -f MicroXRCEAgent 2>/dev/null
pkill -9 -f drone_drl 2>/dev/null; pkill -9 -f "make.*px4" 2>/dev/null
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 2>/dev/null
rm -f $PX4_BUILD/dataman $PX4_BUILD/rootfs/dataman $PX4_BUILD/rootfs/parameters.bson
rm -rf $PX4_BUILD/rootfs/eeprom
sleep $SLEEP_BETWEEN

source /opt/ros/humble/setup.bash
source $SITL_DIR/install/setup.bash

echo "= Agent ="
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
sleep $SLEEP_BETWEEN

echo "= PX4 headless ="
cd $PX4_DIR
HEADLESS=1 PX4_SITL_WORLD="$SITL_DIR/worlds/drop_opt_realistic.world" \
  PX4_PARAM_CBRK_SUPPLY_CHK=894281 PX4_PARAM_CBRK_IO_SAFETY=22027 \
  PX4_PARAM_CBRK_USB_CHK=197848 PX4_PARAM_COM_OF_LOSS_T=10.0 PX4_PARAM_COM_LOW_BAT_ACT=0 \
  GAZEBO_MODEL_DATABASE_URI="" \
  make px4_sitl gazebo-classic_iris_mono_cam_down > /dev/null 2>&1 &

echo -n "= DDS wait"
for i in $(seq 1 50); do
    sleep 1
    n=$(ros2 topic list 2>/dev/null | grep -c "/fmu/out/" || true)
    if [ "$n" -gt 5 ]; then echo " OK(${i}s)"; break; fi
    echo -n "."
done

echo "= Detector ="
ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=true -p bucket_gz_x:=11.01 -p bucket_gz_y:=0.98 -p bucket_gz_z:=0.25 \
    -p spawn_gz_x:=1.01 -p spawn_gz_y:=0.98 -p spawn_gz_z:=0.83 \
    -p model_path:=/home/vincent/SITL/yolo_v8/best2.pt \
    -p conf_threshold:=0.30 -p infer_hz:=10.0 \
    > /tmp/det_headless.log 2>&1 &
sleep 6
grep "Ready" /tmp/det_headless.log
grep "bucket NED" /tmp/det_headless.log

echo "= Mission (timeout 150s) ="
if timeout 150 ros2 run drone_drl drop_opt_mission --ros-args \
    -p mass_loss_enable:=true -p monitor_hz:=2.0 \
    -p track_max_time:=40.0 -p hover_max_time:=15.0 \
    > /tmp/mission_headless.log 2>&1; then
    echo "MISSION COMPLETED"
else
    echo "MISSION TIMEOUT (may have completed LAND)"
fi

echo ""
echo "=== KEY EVENTS ==="
grep -E "Home|ARM|OFFBOARD|Takeoff|FORWARD|Detected|converged|DESCEND|HOVER|DROP|mass_loss|CLIMB|RETURN|LAND|complete|Detection lost|forced|max time" /tmp/mission_headless.log 2>/dev/null || true

echo ""
echo "=== LAST 5 MONITOR LINES ==="
grep "MONITOR" /tmp/mission_headless.log 2>/dev/null | tail -5 || true

echo ""
echo "=== LAST 5 LINES ==="
tail -5 /tmp/mission_headless.log 2>/dev/null || true

# 清理
kill $AGENT_PID 2>/dev/null || true
pkill -9 -x px4 2>/dev/null; pkill -9 -x gzserver 2>/dev/null
echo "done"
