#!/usr/bin/env bash
# cleanup_sim.sh — 每轮仿真前后清理 (2026-10-13)
#
#   ⚠ **必须作为文件执行**: 把 pkill 的模式串写进交互式命令行会自伤
#     (调用者自己的 cmdline 含该串 ⇒ pkill -f 会杀掉自己)。
#
#   ⚠⚠ 2026-10-13 惨痛教训: 一开始只清 px4/gz/ruby/agent, **没清 ros_gz_bridge 与 impact_gt**
#     ⇒ 前几轮 GT_JUDGE 测试留下的 **14 个 ros_gz_bridge** 一直在往 DDS 灌 250Hz 的位姿,
#     把后续每一轮的 OFFBOARD setpoint 流饿死 (实测: 解锁要 24s、飞机飞到 39m 高、
#     `Model timeout` / `land_timeout`)。**开 GT 链的轮次尤其要清干净**。
pkill -9 -x px4 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz-sim" 2>/dev/null
pkill -9 -x ruby 2>/dev/null
pkill -9 -f "MicroXRCEAgent" 2>/dev/null
# --- ROS 侧残留 (GT 链 / 检测器 / 裁判 / 任务节点) ---
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "impact_gt" 2>/dev/null
pkill -9 -f "drone_drl" 2>/dev/null
pkill -9 -f "ros2 run" 2>/dev/null
pkill -9 -f "ros2-daemon" 2>/dev/null
rm -f /tmp/cuadc_test_k.lock /tmp/px4_gz_fifo.*
sleep 3
# 清完自检: 残留数
_n=$(pgrep -fc "ros_gz_bridge|impact_gt|gz sim" 2>/dev/null || true)
echo "[cleanup] 残留 (ros_gz_bridge/impact_gt/gz): ${_n:-0}"
exit 0
