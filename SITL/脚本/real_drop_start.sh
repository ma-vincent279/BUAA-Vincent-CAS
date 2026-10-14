#!/bin/bash
# ================================================================
# 实机投放任务 — 一键启动
# 用法: bash ~/SITL/scripts/real_drop_start.sh
# ================================================================
set -e

# ---- 配置 ----
CAMERA_TOPIC="/camera/image_raw"
MODEL_PATH="/home/vincent/SITL/yolo_v8/best2.pt"
CONDA_ENV="yolov8"

echo "=== 1. 启动 MicroXRCEAgent ==="
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2
echo "  Agent PID=$AGENT_PID"

echo ""
echo "=== 2. 启动 YOLOv8 检测器 (conda: $CONDA_ENV) ==="
source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || \
    source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || \
    source /opt/conda/etc/profile.d/conda.sh 2>/dev/null
conda activate $CONDA_ENV

ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=false \
    -p camera_topic:=$CAMERA_TOPIC \
    -p model_path:=$MODEL_PATH \
    -p conf_threshold:=0.5 \
    -p visualize:=true &
DETECTOR_PID=$!
sleep 3
echo "  Detector PID=$DETECTOR_PID"

echo ""
echo "=== 3. 启动投放任务 (系统 Python) ==="
conda deactivate 2>/dev/null || true

ros2 run drone_drl drop_opt_mission

# 任务结束后清理
echo ""
echo "=== 清理 ==="
kill $DETECTOR_PID 2>/dev/null || true
kill $AGENT_PID 2>/dev/null || true
echo "完成喵~"
