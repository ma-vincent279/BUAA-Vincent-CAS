# 02 · 从零开始的 ROS 生活

> **标记约定**：**【源码事实】/【实测数据】/【经验/建议】/【待验证】**
> **这一章写给**：没写过 ROS 的人。
> **读完你应该能回答**：ROS 在这个系统里干什么？代码怎么组织？为什么这么写？怎么自己跑起来？

---

## 一、ROS 在无人机系统里到底负责什么

先说结论：**ROS 不管飞**。飞是飞控（PX4）的事。
ROS 管的是**"决策"** —— 看画面、算往哪飞、告诉飞控怎么飞。

### 1.1 完整数据链【源码事实】

```
   ┌─────────┐   USB
   │ 摄像头   │──────────────┐
   └─────────┘              ▼
                   ┌──────────────────┐
                   │ see.py           │  YOLOv8 检测
                   │ (视觉节点)        │  → /bucket/detections
                   └────────┬─────────┘      （框的像素坐标）
                            │
                            ▼
                   ┌──────────────────┐
                   │ k 节点            │  ① 像素 → 机体偏移（相机模型）
                   │ (投弹任务节点)     │  ② 选桶（直径最小的两个）
                   │                  │  ③ 状态机（飞行流程）
                   │                  │  ④ 释放判据（残差 ≤3cm 保持 2s）
                   └────────┬─────────┘
                            │ 发布:
                            │  /fmu/in/offboard_control_mode   （声明哪些字段有效）
                            │  /fmu/in/trajectory_setpoint     （位置/速度/偏航设定点）
                            │  /fmu/in/manual_control_input    （AUX5 舵机 PWM, 200Hz）
                            ▼
                   ┌──────────────────┐
                   │ PX4 飞控          │  真正控制电机
                   └──────────────────┘
```

### 1.2 三个容易混淆的层次

| 层次 | 谁负责 | 频率 | 例子 |
|---|---|---|---|
| **内环（姿态/角速度）** | PX4 自己 | 1 kHz | "把 roll 稳定在 0°" |
| **外环（位置/速度）** | PX4 的 MPC，接受**我们发的设定点** | 50 Hz | "去 (2.1, 31.6, -1.2)" |
| **决策（往哪飞、什么时候投）** | **我们的 ROS 节点** | 20 Hz | "桶在那儿，往左挪一点" |

⇒ **我们只写"决策"那一层**。飞控怎么稳住飞机，不用管 ✓

### 1.3 新人必须能回答的几个问题

1. 我们发的 `/fmu/in/trajectory_setpoint` 是**位置**还是**速度**？（答：都可以，由 `offboard_control_mode` 声明）
2. 为什么必须**持续**发设定点？（答：PX4 有 offboard 超时，断了就 failsafe）
3. 视觉节点和任务节点是**两个进程**吗？（答：是，通过话题通信）
4. 为什么判据要在**机体坐标系**算？（答：对飞控位置的绝对漂移免疫）

---

## 二、搭 ROS 2 Humble 环境

### 2.1 固定基线，别追新

**【经验/建议】** 我们整个项目用的是 **ROS 2 Humble**（Ubuntu 22.04）。
不要看到 Jazzy / Rolling 的新功能就往里用 —— 生态包（尤其 `px4_msgs`、`ros_gz`）对版本很敏感 ✗

### 2.2 装 ROS 2

```bash
# 1. 换源 + 装基础工具
sudo apt update && sudo apt install -y curl gnupg lsb-release

# 2. 加 ROS 2 源 (官方文档有最新步骤, 这里示意)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# 3. 装
sudo apt update && sudo apt install -y ros-humble-desktop
```

**【源码事实】** 本项目**额外必需**的一个包（很容易装错）：

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

> ⚠️ **为什么强调这个**：`ros-humble-ros-gz` 默认装的是 **Fortress 变体**
> （链接 `libignition-transport11`），与 **Gazebo Harmonic**（`gz-transport13`）**不兼容** ✗
> 必须是 **`ros-gzharmonic`** 这个变体 ✓

### 2.3 `source` 是什么

**【经验/建议】** 新手最常问的问题："我明明装好了，为什么 `ros2` 命令找不到？"

```bash
source /opt/ros/humble/setup.bash      # 让当前 shell 认识 ROS 的命令和库
source install/setup.bash               # 让你自己的工作空间生效（要在 colcon build 之后）
```

**要点**：
- `source` 只影响**当前这个终端** ⇒ 新开终端要重新 source ✓
- 想省事就写进 `~/.bashrc` ✓
- ⚠️ `source` 一个**脚本** ≠ **执行**一个脚本。`source` 是在**当前 shell** 里跑，
  执行是开一个子 shell（脚本里 `export` 的变量在子 shell 结束后就没了）

---

## 三、本项目用到的 ROS 概念（结合真实代码）

### 3.1 节点 / 话题 / 发布订阅

**【源码事实】** 一个最小的订阅（摘自 `k.py` 的风格）：

```python
self.local_sub = self.create_subscription(
    VehicleLocalPosition, local_pos_topic(), self._cb_local, QOS_BE)
```

**讲三件事**：
| 参数 | 是什么 |
|---|---|
| `VehicleLocalPosition` | **消息类型**（PX4 定义的位置消息）|
| `local_pos_topic()` | **话题名**（本项目是 `/fmu/out/vehicle_local_position_v1`）|
| `self._cb_local` | **回调函数**（收到消息时被调用）|
| `QOS_BE` | **QoS**（见下）|

### 3.2 QoS —— 新手最容易被坑的地方

**【经验/建议】** QoS（服务质量）决定"消息丢了怎么办"：

| 模式 | 语义 | 用在哪 |
|---|---|---|
| `BEST_EFFORT` | 尽力而为，**丢就丢了** | **PX4 的所有 `/fmu/out/*`** ✓ |
| `RELIABLE` | 保证送达，会重传 | 控制指令等 |

⚠️ **如果订阅方用 RELIABLE、发布方用 BEST_EFFORT，就收不到数据** ✗
这是"话题有数据但我回调不触发"的经典原因之一。

**【源码事实】** 本项目的常量：
```python
QOS_BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, depth=10)
```

### 3.3 定时器与"固定频率"

**【源码事实】** 任务节点用定时器跑主循环（20 Hz），而不是"消息来了才跑"：

```python
self.timer = self.create_timer(1.0 / 20.0, self._tick)
```

**为什么**：即使没有新检测，也要**持续发设定点**（否则 PX4 offboard 超时）✓

### 3.4 参数

**【源码事实】** 本项目里**绝大部分常量是 Python 模块级常量**（如 `K_RELEASE_MIN_M = 0.03`），
只有少量用 ROS 参数（如检测器的桶坐标、图像尺寸）。

**【经验/建议】** 两种方式的取舍：
- **模块常量**：改一次要重新 `colcon build` + 重启节点 ⇒ 适合"不常改的"（判据门限）
- **ROS 参数**：`--ros-args -p xxx:=yyy` 就能改 ⇒ 适合"调参时反复改的"（阈值、桶坐标）

### 3.5 包的入口（`setup.py`）

**【源码事实】** 本项目用 `ament_python` 包，入口在 `setup.py`：

```python
entry_points={'console_scripts': [
    'k = fly.k:main',            # ⇒ 用 `ros2 run fly k` 启动
    'see = fly.see:main',
    ...
]}
```

⇒ `ros2 run <包名> <入口名>` 就是这么找到你的程序的 ✓

**【经验/建议】** ⚠️ **改了 Python 代码必须重新 `colcon build`** —— 否则 `ros2 run` 跑的
还是 `install/` 里的**旧副本** ✗（这是我们踩过的坑，见 [`../复现指南.md`](../技术文档/配置仿真环境.md)）

---

## 四、⚠️ 本项目的血泪坑（必须看）

### 4.1 话题名带 `_v1` 后缀

**【源码事实】** 本项目的 PX4 固件发布在**带版本后缀**的话题上：

```
/fmu/out/vehicle_status_v1                    ← 有的是这个
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_control_mode                 ← 这个没有后缀
```

❌ **写 `/fmu/out/vehicle_status` ⇒ 订阅回调永远不触发**（不是报错，是**静默不工作**）✗

> 【实测数据】更麻烦的是：某次 `vehicle_status_v1` 还能因**消息类型校验和不匹配**
> 在 DDS 层被丢弃 ⇒ 后来我们**改用 `vehicle_control_mode`** 判断解锁和模式。
> 详见 `fly/CLAUDE.md`。

### 4.2 节点必须"先启动、持续发"

**【源码事实】** PX4 切 OFFBOARD 的前提是：**已经持续收到设定点（>2 Hz）**。

**后果**：如果节点启动晚了 ⇒ PX4 触发 offboard failsafe ⇒ 进入 AUTO_LAND ⇒ **再也切不回来** ✗

⇒ **启动顺序：Agent → 节点 → PX4**（或至少同时）

### 4.3 `colcon build` 之后要 `source`

```bash
colcon build --packages-select fly
source install/setup.bash        # ← 少了这句, 你还是跑旧代码 ✗
```

### 4.4 `--symlink-install`

**【源码事实】** 本项目**没有**用 `--symlink-install` ⇒ 每次改 Python 都要重新 build ✓
（如果用了 `--symlink-install`，Python 改动可以免 build —— 但会带来别的不一致）

---

## 五、动手练习

### 练习 1：看看有什么话题

```bash
source /opt/ros/humble/setup.bash
source ~/SITL/install/setup.bash
ros2 topic list | grep fmu
```

**你应该能回答**：哪些是 `/fmu/in/`（我们发的），哪些是 `/fmu/out/`（PX4 发的）？

### 练习 2：看话题有没有数据、频率多少

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1     # 应该是 50 Hz 左右
ros2 topic echo --qos-reliability best_effort /fmu/out/vehicle_control_mode --once
```

> ⚠️ `--qos-reliability best_effort` **不能省**，否则 echo 收不到 ✗

### 练习 3：找一处本项目用错 QoS 会怎样

去 `src/drone_drl/drone_drl/k.py` 找所有 `create_subscription`，
看它们用的 QoS 是不是 `QOS_BE`，想想为什么。

### 练习 4：确认"改了代码要重新 build"

1. 改一个日志字符串（比如 `k.py` 里某句 `info` 的内容）
2. **不** build，直接 `ros2 run ...` ⇒ 看输出是不是旧字符串
3. `colcon build --packages-select drone_drl` + `source install/setup.bash` ⇒ 再看

---

## 六、"改了代码却没生效"排查清单

按顺序查：

- [ ] `colcon build` 成功了吗？（看有没有报错）
- [ ] **`source install/setup.bash`** 了吗？（**最常见**）
- [ ] 跑的是**哪个包**？改的是 `drone_drl` 还是 `fly`？（本项目有两套！）
- [ ] 节点的**入口名**对不对？（`ros2 run drone_drl k` vs `ros2 run fly k`）
- [ ] 有没有**残留的旧进程**在跑？（`pgrep -a k_mission`）
- [ ] 改的是**仿真侧**还是**实机侧**？两边是**两条平行血统**，改一边另一边不会变 ✗

---

## 七、下一章

→ [`03_从零开始的仿真生活.md`](从零开始的仿真生活.md)：在电脑上把整个系统跑起来
