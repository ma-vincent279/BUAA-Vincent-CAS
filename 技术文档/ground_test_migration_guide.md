# 地面测试 — 仿真到实机迁移指南

本教程涵盖从 SITL 仿真验证到 Jetson Orin 实机部署的完整流程。

---

## 一、需要搬运到实机的文件清单

以下文件必须从开发机（本机）复制到机载电脑（Jetson Orin）。建议用 U 盘或 `scp` 传输。

### 1.1 Python 源代码 (3 个)

| # | 文件 | 用途 |
|---|------|------|
| 1 | `src/drone_drl/drone_drl/ground_test_mission.py` | 地面测试任务节点 (ARM → 检测 → 投放) |
| 2 | `src/drone_drl/drone_drl/yolov8_bucket_detector.py` | YOLOv8 桶检测器 (sim/real 双模式) |
| 3 | `src/drone_drl/setup.py` | 包入口映射 (含 `ground_test_mission` 和 `yolov8_bucket_detector` 两个 console_scripts 入口) |

**注意**：`setup.py` 必须包含以下两个入口（本机 `setup.py` 已有，直接复制即可）：

```python
'ground_test_mission = drone_drl.ground_test_mission:main',
'yolov8_bucket_detector = drone_drl.yolov8_bucket_detector:main',
```

### 1.2 包配置文件 (2 个)

| # | 文件 | 用途 |
|---|------|------|
| 4 | `src/drone_drl/package.xml` | ROS2 包清单 (声明依赖 `rclpy`, `px4_msgs`, `sensor_msgs` 等) |
| 5 | `src/drone_drl/resource/drone_drl` | ament 资源标记文件 (空文件，目录存在即可) |

### 1.3 YOLO 模型 (2 个，二选一)

| # | 文件 | 用途 | 推荐场景 |
|---|------|------|----------|
| 6 | `yolo_v8/best2.pt` | PyTorch 权重 (CPU/GPU 通用) | 初次部署，兼容性最好 |
| 7 | `yolo_v8/best2.engine` | TensorRT 引擎 (Jetson 专用) | 确认 .engine 与 Jetson 上的 TensorRT 版本匹配后使用 |

**关于 .engine 文件**：`yolo_v8/run_camera.py` 和 `yolo_v8/video.py` 加载的是 `best2.engine`（TensorRT 格式），这是 Jetson Orin 上性能最优的选择。但 TensorRT 引擎与硬件/库版本强绑定，如果 `.engine` 是在不同 JetPack 版本上导出的，可能在你的 Jetson 上无法加载。遇到加载失败时，先用 `best2.pt` 验证链路，后续再在 Jetson 上本地导出 `.engine`。

### 1.4 yolo_v8 参考代码（可选，不参与 ROS 运行）

`yolo_v8/` 目录下的脚本是独立测试/参考文件，不参与 ROS 通信。可以一并复制到 Jetson 上作为调试工具和参数对照：

| 文件 | 用途 |
|------|------|
| `yolo_v8/run_camera.py` | 打开摄像头 (`source=0` → `/dev/video0`)，实时 YOLO 推理并显示 |
| `yolo_v8/video.py` | 打开摄像头，实时 YOLO 推理并录制为 `output_detected.avi` |
| `yolo_v8/test.py` | **Jetson 上跑通的检测器真实版本**，参考里面的默认参数值 |
| `yolo_v8/yolo.py` | 同上，与 test.py 内容相同 |

### 1.5 复制到 Jetson 后必须修改的内容

Jetson 上的路径和本机不同。复制文件后，编辑 `yolov8_bucket_detector.py` 顶部参数，把以下三行改成 Jetson 上的值：

```python
# 修改前 (本机)                           # 修改后 (Jetson)
self.declare_parameter('use_sim', True)    → self.declare_parameter('use_sim', False)
self.declare_parameter('model_path',       → self.declare_parameter('model_path',
    '/home/vincent/SITL/yolo_v8/best2.pt')     '/home/nvidia/yolo_v8/best2.engine')
self.declare_parameter('camera_device', '') → self.declare_parameter('camera_device', '/dev/video0')
```

**参考**：`yolo_v8/test.py` 和 `yolo_v8/yolo.py` 是之前在 Jetson 上跑通的真实版本，可以直接对照里面的默认参数值喵~

### 1.6 复制到 Jetson 后的目录结构

```
~/SITL/
├── src/drone_drl/
│   ├── drone_drl/
│   │   ├── __init__.py              # 空文件，标识为 Python 包
│   │   ├── ground_test_mission.py   # ← 文件 1
│   │   └── yolov8_bucket_detector.py # ← 文件 2
│   ├── resource/
│   │   └── drone_drl                # ← 文件 5
│   ├── setup.py                     # ← 文件 3
│   └── package.xml                  # ← 文件 4
└── yolo_v8/
    ├── best2.pt                     # ← 文件 6
    ├── best2.engine                 # ← 文件 7 (可选)
    ├── run_camera.py                # ← 调试用
    └── video.py                     # ← 调试用
```

---

## 二、硬件连接

```
┌──────────────────────┐          ┌──────────────────┐
│  Jetson Orin (机载)    │   USB    │  Pixhawk/CUAV    │
│                      │◄────────►│  飞控             │
│  USB 下视摄像头 ──────┘          │  AUX5 ── 投放舵机 │
│  (/dev/video0)                  │  (PWM 信号)       │
└──────────────────────┘          └──────────────────┘
```

- **飞控 ↔ Jetson**：USB 线连接。飞控端通常为 micro-USB，Jetson 端为 USB-A。在 Jetson 上显示为 `/dev/ttyACM0`。
- **下视摄像头**：USB 摄像头，Jetson 上显示为 `/dev/video0`（只有一个摄像头时）。镜头朝下安装在无人机底部。
- **投放舵机**：信号线接飞控 AUX5 引脚，电源和地线接分电板或 BEC。

---

## 三、飞控参数设置

在飞控上通过 QGroundCenter 或 PX4 shell 设置以下参数。**这些参数只需设置一次，断电不会丢失。**

```
PWM_AUX_FUNC5 = RC AUX 5       # 值 = 1024，AUX5 输出功能设为 RC AUX 直通
RC_MAP_AUX5   = Unassigned     # 值 = 0，解除 RC 接收机对 AUX5 通道的占用
```

**为什么必须设置 `RC_MAP_AUX5 = Unassigned`**：
- RC 接收机的 SBUS 数据流 (~111Hz) 会周期性覆盖通过 DDS 写入的 AUX5 值
- 不解除的话，AUX5 值会在 "DDS 写入值" 和 "RC 接收机默认值" 之间反复跳变，导致舵机抖动或不受控

---

## 四、Jetson Orin 环境准备

### 4.1 确认硬件设备

```bash
# 查看摄像头设备
ls /dev/video*             # 通常出现 /dev/video0 和 /dev/video1
v4l2-ctl --list-devices    # 确认哪个是下视 USB 摄像头

# 查看飞控串口
ls /dev/ttyACM*            # Pixhawk 通常为 /dev/ttyACM0
ls /dev/ttyUSB*            # 如果是 USB-TTL 模块则为 /dev/ttyUSB0
```

### 4.2 安装依赖

```bash
# ROS 2 Humble (如果还没装)
# 参考: https://docs.ros.org/en/humble/Installation.html

# YOLOv8 (需要在 conda 环境中安装，因为 Jetson 的 ultralytics 依赖特定 numpy 版本)
conda activate <your_env_name>   # 或者用 pip3
pip3 install ultralytics

# 确认 ROS2 能 import px4_msgs
python3 -c "from px4_msgs.msg import VehicleCommand, ManualControlSetpoint; print('OK')"
# 如果报 ImportError 说明 px4_msgs 没装:
# sudo apt install ros-humble-px4-msgs
```

### 4.3 编译 drone_drl 包

```bash
cd ~/SITL
colcon build --packages-select drone_drl
source install/setup.bash
```

编译成功后验证入口点：

```bash
ros2 run drone_drl ground_test_mission --help    # 应正常启动 (Ctrl-C 退出)
ros2 run drone_drl yolov8_bucket_detector --help  # 应正常启动 (Ctrl-C 退出)
```

### 4.4 确认 YOLO 模型文件存在

```bash
ls -l ~/SITL/yolo_v8/best2.pt       # PyTorch 模型，必须存在
ls -l ~/SITL/yolo_v8/best2.engine   # TensorRT 引擎，可选
```

### 4.5 单独测试摄像头 + YOLO（推荐，先排除摄像头问题）

在启动 ROS 节点之前，先在 Jetson 上用 `yolo_v8/run_camera.py` 测试摄像头和 YOLO 推理是否正常：

```bash
cd ~/SITL/yolo_v8
python3 run_camera.py
```

**预期行为**：
- 弹出一个窗口，显示摄像头画面
- 画面中检测到的桶被 YOLO 画出边界框
- 按 `Q` 键退出

**如果画面黑屏或报错**：
- 检查 `/dev/video0` 是否存在
- 检查摄像头是否被其他进程占用：`sudo fuser /dev/video0`
- 如果 Jetson 使用 CSI 摄像头而非 USB，需修改 `source=0` 为对应的 GStreamer 管道字符串

---

## 五、手动分步启动（3 个终端）

**前提**：飞控已上电并完成初始化（GPS 锁定、EKF 收敛、preflight checks 通过）。

**安全注意**：测试期间**不装桨**，飞机放在地面上不动。

### 关于摄像头：为什么"免驱"还需要额外步骤？

你的 USB 摄像头是 UVC 免驱的 — Linux 内核自带驱动，插上就能在 `/dev/video0` 读到画面。你可以用 `yolo_v8/run_camera.py` 直接测试：

```bash
python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.read()[0])"
# 输出 True = 摄像头正常工作
```

但在 ROS2 里，节点之间通过**话题**通信，不是直接读设备文件。`yolov8_bucket_detector` 的 real 模式以前只支持订阅 ROS2 Image 话题，所以需要一个中间节点把 `/dev/video0` 的画面搬运到话题上（就是旧指南里那个让你困惑的"摄像头驱动"）。

**现在不用了** — 检测器新增了 `camera_device` 参数，可以直接打开 `/dev/video0` 读帧，省掉了中间人。下面启动时用 `camera_device:=/dev/video0` 即可。

### 终端 1：MicroXRCEAgent

```bash
# 确认飞控串口设备存在
ls /dev/ttyACM0

# 启动 Agent（串口模式，实机常用）
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600

# 如果飞控和 Jetson 通过 UDP 通信，则:
# MicroXRCEAgent udp4 -p 8888
```

保持运行。这是飞控与 ROS2 之间的 DDS 桥接，关掉它 ROS2 侧收不到 `/fmu/out/*` 话题。

**验证**：新开一个终端，确认话题有数据：
```bash
source ~/SITL/install/setup.bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once
# 有数据输出 = Agent 工作正常
```

### 终端 2：YOLOv8 检测器（real 模式，直读摄像头）

```bash
source ~/SITL/install/setup.bash

# camera_device:=/dev/video0  — 检测器直接用 OpenCV 打开摄像头读帧
#                               不用再单独启动 usb_cam / gscam 等节点
# visualize:=true              — 弹出窗口显示检测画面 (调试时开, 正式跑可关)

ros2 run drone_drl yolov8_bucket_detector --ros-args \
    -p use_sim:=false \
    -p camera_device:=/dev/video0 \
    -p model_path:=/home/vincent/SITL/yolo_v8/best2.pt \
    -p conf_threshold:=0.5 \
    -p visualize:=true
```

保持运行。终端打印 "Camera opened: /dev/video0 (640x480)" 表示摄像头已就绪。

**如果使用 TensorRT 引擎**（Jetson 上性能更好）：
```bash
    -p model_path:=/home/vincent/SITL/yolo_v8/best2.engine
```
`.engine` 加载失败时报 "Failed to deserialize CUDA engine"，回退到 `.pt` 即可。

**验证**：
```bash
ros2 topic echo /bucket/detection
# 把桶放到摄像头下方，应该看到 ROI 数据:
#   x_offset: 245
#   y_offset: 180
#   width: 80
#   height: 80
```

**补充说明**：如果你的摄像头不是 `/dev/video0`（比如 Jetson 的 CSI 摄像头或者多个 USB 摄像头），有两种方式指定：

- **直读模式**（推荐）：`camera_device:=/dev/video1` 改成对应的设备路径
- **ROS2 话题模式**（需要额外启动摄像头驱动节点）：把 `camera_device` 留空，设置 `camera_topic:=/your/topic`，检测器会订阅该话题而不是直读摄像头

### 终端 3：地面测试任务

```bash
source ~/SITL/install/setup.bash
ros2 run drone_drl ground_test_mission
```

**预期日志输出**：
```
[INFO] Ground Test Mission ready — ARM → IDLE (AUTO_LOITER) → detect bucket → DROP → LOCK → wait next
[INFO] Home: NED(0.000, 0.000, 0.000) heading=45.3deg
[INFO] Waiting for preflight... (t=0s)
[INFO] Requesting ARM (t=1.0s nav=4)
[INFO] ARMED — entering IDLE, waiting for bucket detection...
```

然后把桶放到无人机摄像头下方，你应该看到：
```
[INFO] Bucket detected → waiting 0.5s...
[INFO] >>> DROP! AUX5=0.833 (PWM 2000us) at NED(...)
[INFO] AUX5 restored to LOCK (0.000, PWM 1500us) → COOLDOWN
[INFO] Detection cleared → IDLE, ready for next
```

移开桶 → 检测消失 → 再次放入 → 下一轮触发。

**按 Ctrl-C 退出**。然后在终端 2、终端 1 按 Ctrl-C 关闭。

---

## 六、验证全链路的测试顺序

不装桨，按以下顺序逐项验证，每步通过后再进到下一步，出了问题容易定位：

1. **仅 ARM** — 只启动终端 1 (Agent) 和终端 3 (mission)，不启动检测器。确认飞控能解锁（电机会怠速旋转）。
2. **ARM + 检测器** — 加启动终端 2 (检测器)，把桶放到摄像头下方，观察终端 3 是否打印 "Bucket detected"。确认 `/bucket/detection` 话题有数据。
3. **ARM + 舵机** — 步骤 2 通过但投放时舵机没动静，检查 `PWM_AUX_FUNC5` 和 `RC_MAP_AUX5` 参数是否生效。可以用万用表或示波器量 AUX5 引脚在投放瞬间有无 PWM 脉冲变化。
4. **端到端** — 三个终端全部启动，桶放下去，0.5 秒延迟后舵机动作 → 投放机构打开 → 1 秒后恢复锁定。

---

## 七、参数调优

实机环境与仿真不同，以下参数可能需要现场调整：

### 7.1 检测器参数 (yolov8_bucket_detector)

```bash
# conf_threshold: 实机光照/背景复杂, 建议从 0.5 开始, 漏检时逐步降低
-p conf_threshold:=0.5   # 调试时可在 0.3~0.7 范围调整

# infer_hz: 检测器每帧都推理可能占用大量 CPU, Jetson 上可降到 5Hz
-p infer_hz:=5.0
```

### 7.2 任务参数 (ground_test_mission.py)

以下参数在 `ground_test_mission.py` 文件顶部直接修改，改完需要重新编译：

```python
DETECT_WAIT    = 1.0   # 检测到桶后等 N 秒再投放 (放宽到 1s 确保稳定)
DROP_HOLD      = 1.5   # DROP 信号保持 N 秒 (舵机动作慢则加大)
DETECT_TIMEOUT = 3.0   # 多久没收到检测算"丢失" (YOLO 推理慢时需放宽)
COOLDOWN_TIME  = 5.0   # 投放后冷却 N 秒才能再次触发
```

### 7.3 AUX5 舵机值

```python
AUX5_LOCK  = 0.0        # aux5 → PWM 1500us (中位/关仓)
AUX5_DROP  = 0.83333    # aux5 → PWM 2000us (投放位/开仓)
```

**PWM 换算公式**：`PWM_us = 1500 + aux5_value * 600`

| aux5_value | PWM | 说明 |
|---|---|---|
| -1.0 | 900us | 最小行程 |
| 0.0 | 1500us | 中位 (关仓锁定) |
| 0.833 | 2000us | 投放位 (开仓释放) |
| 1.0 | 2100us | 最大行程 |

根据你的舵机实际行程调整 `AUX5_DROP` 的值喵~ 不同品牌的舵机对 2000us 的响应角度不同，可能需要实测后微调。

---

## 八、调试指南

### 8.1 检查检测器是否工作

```bash
ros2 topic echo /bucket/detection
# 桶在视野中 → 有 ROI 输出
# 始终无输出 → 检查摄像头话题名、图像编码、模型是否存在
```

### 8.2 检查 AUX5 PWM

```bash
ros2 topic echo /drop/aux5_pwm
# 正常状态: data: 1500.0
# 投放瞬间: data: 2000.0
```

### 8.3 检查飞控 ARM 状态

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once | grep -E "arming_state|pre_flight|nav_state"
# arming_state: 1 = disarmed, 2 = armed
# pre_flight_checks_pass: True = 可以解锁
```

### 8.4 常见问题

| 症状 | 可能原因 | 解决方案 |
|---|---|---|
| Agent 启动后无 `/fmu/out/*` 话题 | 飞控未上电或串口选错 | `ls /dev/ttyACM0` 确认设备存在 |
| 检测器报 "Cannot open camera" | 摄像头设备路径不对 | `ls /dev/video*` 确认，改 `camera_device` |
| 检测器报 "Cannot open camera" | 设备被其他进程占用 | `sudo fuser /dev/video0` 找到占用进程并 kill |
| 检测器直读摄像头画面全黑 | OpenCV 读到了错误的 `/dev/video*` | Jetson 上可能有多个 video 设备，逐个试 `/dev/video0` `/dev/video1` |
| 检测器无输出 | 置信度太高全部被过滤 | 降低 `conf_threshold` 到 0.3 |
| ARM 被拒 | preflight checks 不通过 | 检查 GPS/EKF/磁力计，等待飞控初始化完成 |
| ARM 被拒 | 安全开关未按下 | 在地面站中设置 `CBRK_IO_SAFETY=22027` |
| 舵机不动作 | `RC_MAP_AUX5` 未设为 Unassigned | QGC 参数页检查 |
| 舵机不动作 | `PWM_AUX_FUNC5` 未设为 RC AUX 5 | QGC 参数页检查 |
| 舵机抖动 | DDS 发布频率不够 | 确认 `manual_timer` = 200Hz (0.005s) |
| .engine 加载失败 | JetPack/TensorRT 版本不匹配 | 回退到 `best2.pt`，后续在 Jetson 上本地导出 |

---

## 九、仿真 vs 实机对照速查

| 维度 | 仿真 (SITL) | 实机 (Jetson Orin) |
|---|---|---|
| PX4 | Gazebo SITL 虚拟飞控 | 真实 Pixhawk/CUAV 飞控 |
| Agent 启动 | `MicroXRCEAgent udp4 -p 8888` | `MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600` |
| 摄像头 | GT 投影 → 合成 640x480 白圆 | 真实 USB 下视摄像头 (`/dev/video0`)，检测器直读 |
| Camera 方案 | 不需要 | `camera_device:=/dev/video0` 直读 (无需额外节点) |
| 桶位置 | ROS 参数 (bucket_gz_x/y/z) | 无 — 真实世界中的桶 |
| 舵机控制 | ManualControlSetpoint.aux5 200Hz | 同左 — 完全一样 |
| ARM | VehicleCommand cmd=400 | 同左 — 完全一样 |
| `/bucket/detection` | RegionOfInterest | 同左 — 完全一样 |
