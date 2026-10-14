#!/bin/bash
# 监控桌面终端里的 d 仿真 (由 agent 后台运行, 不干扰终端窗口)
echo "[monitor] 开始监控 $(date +%H:%M:%S)"
# 阶段1: 等 gzserver (编译完成, 最多 12 分钟)
for i in $(seq 1 48); do
  if pgrep -x gzserver >/dev/null 2>&1; then
    echo "[monitor] gzserver 已启动 ($(date +%H:%M:%S), 编译完成, Gazebo 窗口已弹出)"
    break
  fi
  if ! pgrep -f "bash /home/vincent/SITL/tmp_test_d_fixed" >/dev/null 2>&1; then
    echo "[monitor] 警告: 脚本提前退出且 gzserver 未出现"
    break
  fi
  sleep 15
done
# 阶段2: 等 d 任务结束 (终端窗口脚本退出, 最多 20 分钟)
for i in $(seq 1 80); do
  if ! pgrep -f "bash /home/vincent/SITL/tmp_test_d_fixed" >/dev/null 2>&1; then
    echo "[monitor] 终端脚本已结束 ($(date +%H:%M:%S))"
    break
  fi
  sleep 15
done
echo "=== 裁判打分 (/tmp/d_judge.log) ==="
cat /tmp/d_judge.log 2>/dev/null || echo "(无裁判日志)"
echo ""
echo "=== 任务日志尾部 (/tmp/d_output.log) ==="
tail -100 /tmp/d_output.log 2>/dev/null || echo "(无任务日志)"
echo "[monitor] 监控结束 $(date +%H:%M:%S)"
