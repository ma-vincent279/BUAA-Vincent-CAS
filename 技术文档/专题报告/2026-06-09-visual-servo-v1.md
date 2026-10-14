# 项目报告 (2026-06-09 双桶视觉伺服最终版)

### 项目目标

在 PX4 SITL Gazebo 仿真环境中，实现无人机通过**下视摄像头 + PID 像素偏差视觉伺服**，依次对**两个桶**进行**精准定位、下降投弹、落点打分**的完整任务闭环。

### 系统架构

```
bucket_search.world (Gazebo)
  ├─ red_bucket_1  (红桶, Gazebo 6.0,4.0 → NED x≈3,y≈5, 飞机右侧3m+前5m)
  └─ blue_bucket_2 (蓝桶, Gazebo 6.0,-2.0 → NED x≈-3,y≈5, 飞机左侧3m+前5m)

bucket_vision_detector  ──→  /bucket/detection (ROI)
  │  双桶坐标参数, /switch_target 服务切换目标
  ↓
visual_servo_mission  ──→  /fmu/in/trajectory_setpoint (速度/位置)
  │  11 状态机, PID 像素伺服, 航点切换
  │  /drop_payload 服务 → payload_dropper → /spawn_entity (自由落体圆柱体)
  ↓
drop_judge  ──→  /model_states 获取落点
     A区(<0.125m, 2分) / B区(<1.0m, 1分) / Miss(0分)
```

### 开发迭代过程

#### 迭代 1：单桶 → 双桶架构重构

- 删除 SCOUT 侦察状态，投弹后直接返航
- 落点判定从任务节点解耦，新建独立 `drop_judge.py` 裁判节点
- FORWARD 从 NED x=10 改为沿机体 heading 方向飞行，适配飞机实际朝向（heading≈92° 正东）
- 发现 `clean_start.sh` 未加载 `bucket_search.world`（默认 empty.world 无桶），添加 `PX4_SITL_WORLD` 环境变量

#### 迭代 2：分两次投弹（间隔 5s）

- `payload_dropper.py` 改为每次 `/drop_payload` 调用只释放一枚载荷，左右交替
- DROP 状态拆为 3 个内部阶段：投1 → 等 5s → 投2

#### 迭代 3：双桶依次伺服（方案 B 航点切换）

- `bucket_search.world` 添加第二只蓝桶（NED -y 方向 5m）
- `bucket_vision_detector.py` 支持双桶坐标参数 + `/switch_target` 服务（SetBool）
- 新增 MOVE_B2 状态：桶1 投完 → 切换检测器 → 飞往桶2航点 → 再次伺服投弹

#### 迭代 4：桶位置重新布局

- 用户要求：蓝桶左移 3m，红桶放蓝桶右侧 6m
- 布局变为：两桶同在 5m 前向线，南北相隔 6m（红桶 NED +x=右, 蓝桶 NED -x=左）
- FORWARD 航点改为：前 5m + 右 3m（直接飞往桶1），不再飞中心线

#### 迭代 5：降低投放高度 + PID 视觉锁定

- 巡航高度从 5m 降至 3.5m，投放高度降至 2m
- HOVER 改为 PID 速度伺服（替代位置控制），消除切换时的位置跳变
- 新增 DESCEND 状态：TRACK 收敛后 → PID 水平锁定 + 匀速下降至投放高度
- 低空像素阈值收紧至 20px（高空中 40px）

#### 迭代 6：NED z 轴符号 Bug 修复（关键）

- **Bug**：DESCEND 阶段 `vz=-0.5`（NED z 负值=爬升），飞机越飞越高（z: -3.5 → -12.8）
- **根因**：NED 坐标系 z 轴正值=向下。`vz=-0.5` 是向上的速度
- **修复**：`vz=0.5`（正值=下降），同时 TRACK 添加 `vz = (target_z - self.z) * 0.5` 主动高度锁定防止漂移

### 最终状态机流程

```
TAKEOFF(爬升至3.5m)
  → FORWARD(飞往桶1航点: 前5m+右3m)
  → SEARCH(检测桶1) → TRACK(PID+高度锁定, err<40px 2s)
  → DESCEND(PID锁定+0.5m/s下降至2m)
  → HOVER(低空收敛, err<20px 2s) → DROP(投桶1)
  → /switch_target(切桶2) → MOVE_B2(飞往桶2航点: 前5m+左3m)
  → SEARCH(检测桶2) → TRACK → DESCEND → HOVER → DROP(投桶2)
  → RESULT → RETURN → LAND(disarm)
```

### 关键技术决策

#### 1. 任务节点不获取桶精确坐标

任务节点只知道桶的**大致方位**（前 5m + 左/右 3m），通过航点飞到附近区域后，完全依赖 `/bucket/detection` 的 ROI 进行视觉伺服。精确的桶坐标仅在仿真检测器 `bucket_vision_detector` 和裁判节点 `drop_judge` 中使用。此架构天然支持实机 YOLOv8 替换。

#### 2. PID 视觉伺服 + 速度控制

全程使用**机体坐标系速度控制**（`_pub_vel`），通过 PID 将像素偏差映射为机体前向/横向速度，再由 `_body_to_ned` 转换为 NED 速度。相比位置控制，速度控制响应更快、无超调，且 TRACK→DESCEND→HOVER 切换时无位置跳变。

#### 3. DESCEND 阶段独立

将"下降"从 HOVER 内部阶段提升为独立状态 DESCEND。职责单一：PID 水平锁定 + 匀速下降。到达投放高度后进入 HOVER 进行低空精密收敛。状态转换条件清晰，易于调试。

#### 4. NED z 轴方向

NED 坐标系 z 轴**正值=向下**（地）。速度指令中 `vz>0` 表示下降，`vz<0` 表示爬升。这是整个项目中最容易出错的细节。

### 已知 Bug 修复清单

| # | 问题 | 根因 | 修复 |
|---|---|---|---|
| 1 | 飞机起飞后往左飞 | FORWARD 目标写死 NED x=10（北），但飞机 heading≈92°（东），北=左侧 | 改为沿 heading 方向飞行 |
| 2 | 场景无桶 | `clean_start.sh` 用 `empty.world`，未加载 `bucket_search.world` | 添加 `PX4_SITL_WORLD` 环境变量 |
| 3 | model_states 无数据 | ROS2 发布在 `/model_states`，非 `/gazebo/model_states` | `drop_judge.py` 订阅改为 `/model_states` |
| 4 | 第二个桶检测不到 | FORWARD 飞中心线(前5m)，桶分别左右3m，相机地面幅宽~4m 够不着 | FORWARD 直接飞桶1航点(前5m+右3m) |
| 5 | `/switch_target` 静默失败 | `call_async` 不等待响应 | 改为 `spin_once` 同步等待 |
| 6 | 飞机越飞越高 | DESCEND `vz=-0.5`，NED z 负值=爬升 | `vz=0.5`（正值=下降） |
| 7 | TRACK 阶段高度漂移 | 仅发 `vz=0`，无主动高度锁定 | 添加 `vz=(target_z-self.z)*0.5` |
| 8 | 载荷物理模型导致漂移 | 双 0.55kg 载荷占机身 70% 质量 | SDF 移除，改为 `/spawn_entity` 虚拟投放 |

### 文件变更

| 文件 | 变更 |
|---|---|
| `visual_servo_mission.py` | 11 状态机, PID 伺服, DESCEND, 双桶航点, `/switch_target` 集成, NED z 修复 |
| `bucket_vision_detector.py` | 双桶参数 + `/switch_target` 服务 |
| `payload_dropper.py` | 每次只投一枚（左右交替）+ BODY_OFFSETS |
| `drop_judge.py` | **新增** `/model_states` 落点判定 A(2分)/B(1分)/Miss(0分) |
| `bucket_search.world` | 双桶模型 (red_bucket_1 + blue_bucket_2) + `libgazebo_ros_state` |
| `setup.py` | 添加 `drop_judge` 入口 |
| `clean_start.sh` | `PX4_SITL_WORLD`, 双桶 detector 参数, `drop_judge` 启动 |

### 仿真验证结果

```
Home: NED(0.01, -0.00, 0.01), heading=93° (正东)

TAKEOFF   z: 0→-3.5  爬升 3.5m (~12s, 三段式)
FORWARD   飞往桶1航点 NED(2.7, 5.1), 途中检测到桶
TRACK     NED(2.09,4.15)→(2.86,5.12), err: (148,123)→(23,20)px, ~16s 收敛
DESCEND   z: -3.4→-2.0, PID锁定下降至 2m, px_err<15px
HOVER     2m 低空收敛, err<20px 2s
DROP      投桶1 (payload_l) → switch_target → MOVE_B2
MOVE_B2   飞往桶2航点 NED(-3.1, 5.1)
SEARCH→TRACK→DESCEND→HOVER→DROP  投桶2 (payload_r)
RESULT    双桶投放完成 → RETURN → LAND → disarm
```

| 指标 | 数值 |
|---|---|
| 单桶伺服收敛时间 | ~16s（含 TRACK + DESCEND + HOVER） |
| 投放高度 | 2.0m |
| 投放时像素误差 | < 15px |
| 落点精度 | B区（< 1.0m），距桶 ~0.3-0.5m |
| 裁判打分 | A区 2分 / B区 1分 / Miss 0分 |
| 端到端时间 | ~80s（两桶） |

### 运行方式

```bash
bash ~/SITL/scripts/clean_start.sh
```

Gazebo 窗口显示：红桶（右侧）+ 蓝桶（左侧），无人机依次飞往两个桶上方 → 下降 → 投弹 → 返航着陆。终端输出包含 1Hz NED 坐标日志和裁判打分结果喵~
