# 配置 Jetson 机载环境

**目标硬件**：NVIDIA Jetson Orin NX（ARMv8 aarch64，8 核 Cortex-A78AE，7.4 GB RAM）

> ⚠️ **最快的方式是烧系统镜像**（省掉本文全部步骤）——
> 见 [`../sim/cuadc_px4_sim/docs/JETSON_RESTORE.txt`](../sim/cuadc_px4_sim/docs/JETSON_RESTORE.txt)。
> 本文是从零配的步骤。

---

## 一、系统基线【源码事实】

| 项 | 值 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| L4T | R36.4.3 |
| kernel | 5.15.148-tegra |
| ROS 2 | **Humble** |
| 电源模式 | 15 W（nvpmodel mode 2，4 核 / GPU 612 MHz）|
| 工作空间 | `~/fly`（colcon，含 `fly` 与 `px4_msgs` 两个包）|

## 二、视觉栈

| 组件 | 版本 |
|---|---|
| PyTorch | 2.5（NVIDIA 定制版）|
| TensorRT | 10.7 |
| Ultralytics | 8.3.235 |
| conda 环境 | `yolov8`，位于 `/home/nvidia/miniforge3/envs/yolov8/` |

模型：YOLOv8 检测「桶」（`balloon.engine` 等，640×640，FP16）。

## 三、⚠️ 三个必踩的坑

### 坑 1 · YOLO 启动崩溃（NumPy 冲突）

**现象**：`_ARRAY_API not found` / `Numpy is not available`

**根因**：系统 pip 的 PyTorch/ultralytics（NumPy 1.x 编译）与 conda 的 NumPy 2.x 不兼容；
`ros2 run` 把系统路径放在 conda 前面 ✗

**解决**（启动 yolo 前必设）：

```bash
export PYTHONPATH="/home/nvidia/miniforge3/envs/yolov8/lib/python3.10/site-packages:$PYTHONPATH"
```

### 坑 2 · 摄像头分辨率不生效

**现象**：设了 640×480，日志还是 1920×1080

**根因**：`cv2.VideoCapture(dev)` 默认后端无法对 V4L2 设分辨率 ✗

**解决**：用 `cv2.CAP_V4L2` 后端 + MJPG fourcc ✓

### 坑 3 · 话题名必须带 `_v1` 后缀

此固件的 PX4 发布在**带版本后缀**的话题上：

```
/fmu/out/vehicle_local_position_v1     ✅
/fmu/out/vehicle_local_position        ❌ 订阅回调永远不触发 (静默失败!)
```

## 四、串口 DDS 链路

**【源码事实】** 机载 ↔ 飞控走串口 DDS，**921600 baud**
（与 PX4 参数 `SER_TEL2_BAUD = 921600` 对应）：

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
# 若 dialout 组未生效:
sg dialout -c "MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600"
```

**验证链路**：

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1     # 期望 ~50 Hz
```

## 五、编译

```bash
cd ~/fly
source /opt/ros/humble/setup.bash
colcon build --packages-select fly
source install/setup.bash
```

> ⚠️ **本工作空间没有用 `--symlink-install`** ⇒
> **每次改 Python 都要 `colcon build --packages-select fly`** ✗ 否则跑的还是旧副本。

## 六、检查清单

- [ ] `gz`/仿真不需要（这是机载侧）
- [ ] ROS 2 环境能 source
- [ ] `~/fly` 能 `colcon build --packages-select fly`
- [ ] 串口 DDS 通（`ros2 topic hz` 有数据）
- [ ] 相机能出图（`ros2 run fly yolo` 或 `camera_viewer`）
- [ ] AUX5 舵机能动（**拆桨测！**）
