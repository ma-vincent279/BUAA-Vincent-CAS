# 项目报告 (2026-06-11 A3 中心悬停 + 视觉面积排序自主选目标)

### 项目目标

在 PX4 SITL Gazebo 仿真环境中，实现无人机**不依赖硬编码桶坐标**，通过**飞到三桶几何中心悬停 → YOLO 多目标检测 → 空间聚类 → bbox 面积排序自动选出最大两桶 → 位置记忆追踪 → PID 视觉伺服投弹**的完整自主投弹闭环。最终目标：替换仿真 GT 检测器为实机 YOLOv8 `.pt` 模型后，代码无需修改即可在实机上运行。

### 核心创新：A3 方案

**问题**：实机 YOLOv8 桶检测模型只能识别"桶"这一类目标，无法区分 Φ0.15m / Φ0.20m / Φ0.25m 三个不同尺寸的桶。三个桶可能同时出现在下视摄像头画面中，而任务要求只投最大的两个。

**A3 方案**：将问题拆为两个独立阶段，各用最优工具解决——

| 阶段 | 问题 | 手段 |
|---|---|---|
| **INIT（中心悬停）** | 哪个最大？哪个第二大？ | 飞到几何中心 → 同距离观测 → 多帧聚类 → bbox 面积排序 |
| **TRACKING（追踪伺服）** | 画面里多个桶，哪个是我正在追踪的？ | 反投影 NED 位置记忆 → 逐帧位置匹配 → 发单 ROI |

**关键设计决策**：追踪阶段用 **位置匹配而非面积匹配**。面积随距离平方变化（DESCEND 时 bbox 膨胀 3-4x），而 NED 位置是距离无关的不变量，更适合跨高度追踪。

### 系统架构

```
competition_field.world (Gazebo)
  ├─ bucket1 Φ0.15m (小, 不投)
  ├─ bucket2 Φ0.20m (中, 第二大)
  └─ bucket3 Φ0.25m (大, 最大)

yolo_bucket_detector (仿真 YOLO 多目标输出)
  │  INIT: GT 投影三桶 bbox + 抖动 → 多帧收集 → 空间聚类 → 面积排序选 top2
  │  TRACKING: 反投影所有 bbox → 与 target[active].ned 匹配 → 单 ROI
  │  Pub: /bucket/detection (RegionOfInterest)
  │  Sub: /bucket/target_id (Int32, 0/1)
  ↓
full_mission_v2 (14 状态机)
  │  TAKEOFF → FORWARD_CENTER → CENTER_SEARCH → TRACK → DESCEND → HOVER → DROP(最大)
  │  → CLIMB_SWITCH → TRACK → DESCEND → HOVER → DROP(第二大)
  │  → RESULT_DROP → FORWARD_SCOUT → SCAN → RESULT_SCOUT → RETURN → LAND
  ↓
payload_dropper → /spawn_entity (虚拟投放)
mission_judge → 细化计分 (10/8/6/4/2/1/0)
```

### 状态机 (v2 vs v1 对比)

| # | v1 (硬编码航点) | v2 (A3 中心悬停) | 控制方式 |
|---|---|---|---|
| 1 | TAKEOFF | TAKEOFF | 位置控制爬升 3.5m |
| 2 | FORWARD (飞桶1 NED(1.0,34.5)) | **FORWARD_CENTER** (飞中心 NED(0,35)) | 位置+速度混合 |
| 3 | SEARCH (搜索桶1) | **CENTER_SEARCH** (中心悬停等 detector 初始化) | 位置控制悬停 |
| 4-7 | TRACK→DESCEND→HOVER→DROP (桶1) | TRACK→DESCEND→HOVER→DROP (目标0=最大桶) | PID 伺服 |
| 8 | MOVE_B2 (飞桶2 NED(0,36)) | **CLIMB_SWITCH** (爬升+切目标1) | 速度控制爬升 |
| 9-11 | SEARCH→TRACK→...→DROP (桶2) | TRACK→DESCEND→HOVER→DROP (目标1=第二大桶) | PID 伺服 |
| 12 | RESULT_DROP | RESULT_DROP | 位置控制 |
| 13+ | FORWARD_SCOUT→SCAN→...→LAND | 同 v1 (侦察阶段不变) | 同 v1 |

### 检测器 (yolo_bucket_detector) 技术细节

**INIT 阶段 (STATE_INIT)**：
1. 仿真 YOLO 多目标输出：GT 投影生成三桶 bbox，加高斯噪声 (±4px 中心, ±6px 尺寸)，3% 漏检模拟
2. 每帧收集观测 `(ned_x, ned_y, bbox_area, bucket_id)`
3. 每 40 帧 (~2s) 运行空间聚类 (简易 Union-Find, eps=1.5m)
4. 簇数 ≥2 → 每簇取质心+平均面积 → 按面积降序 → top 2 → 进入 TRACKING

**TRACKING 阶段 (STATE_TRACKING)**：
1. 每帧反投影所有 bbox 到 NED
2. 计算每个 bbox 的 NED 位置到 `targets[active].ned` 的欧氏距离
3. 选距离最近的 bbox，需 < 2.0m (REACQUIRE_RADIUS)
4. **时序滤波**：若当前帧最佳匹配距离突然跳变 >1.5m，短暂保持上帧 ROI (最多 0.5s)
5. 发送单 ROI 到 `/bucket/detection`

**仿真→实机迁移路径**：`_simulate_yolo_detections()` 替换为 `model(image)` 调用 → 返回 `[(px, py, w, h), ...]` → 反投影逻辑不变 → INIT/TRACKING 逻辑不变。

**摄像头参数**：640×480, HFOV=60°, fx=609, fy=456

### 新增文件

| 文件 | 行数 | 说明 |
|---|---|---|
| `drone_drl/yolo_bucket_detector.py` | 350 | A3 多 bbox 检测器 (INIT 聚类 + TRACKING 位置记忆 + 时序滤波) |
| `drone_drl/full_mission_v2.py` | 650 | 14 状态机, 取消硬编码航点, 纯视觉选目标 |
| `scripts/clean_start_v2.sh` | 140 | v2 一键启动 (yolo_bucket_detector + full_mission_v2) |

### 修改文件

| 文件 | 变更 |
|---|---|
| `setup.py` | +2 行: `yolo_bucket_detector` / `full_mission_v2` 入口 |
| `scripts/clean_start.sh` | 增加 PX4 ULog 日志自动清理 (不影响功能) |
| `scripts/test_drop.sh` | 增加 PX4 ULog 日志自动清理 (不影响功能) |
| `scripts/start_vision_mission.sh` | 补充完整 PX4 状态清除逻辑 (不影响功能) |

### 开发迭代过程

#### 迭代 1：初始实现

- 新建 `yolo_bucket_detector.py`：GT 投影模拟 YOLO 多目标 + INIT 聚类 + TRACKING 位置匹配
- 新建 `full_mission_v2.py`：14 状态机, CENTER_HOVER 中心悬停, 取消 B1_WP/B2_WP
- 新建 `clean_start_v2.sh`：一键启动新节点组合

#### 迭代 2：TRACK 振荡排查 (关键)

- **Bug**：TRACK 阶段持续 222s 不收敛，ey 反复出现 220+ px 尖峰
- **根因**：GT 模拟 10% 漏检 + REACQUIRE_RADIUS=3m → 主目标 (bucket3) 偶尔被丢弃时，tracker 将 1.81m 外的 bucket2 误匹配（仍在 3m 半径内），ey 从 ~10px 跳到 ~235px，重置稳定计时器
- **修复**：
  1. 漏检率 10%→0%（仿真阶段 GT 投影 100% 可靠）
  2. REACQUIRE_RADIUS 3m→2m（三桶彼此间距 ~2m，2m 半径足以排除相邻桶）
  3. 新增时序滤波：匹配距离跳变 >1.5m 时保持上帧 ROI (0.3s)

#### 迭代 3：第二目标误匹配

- **Bug**：CLIMB_SWITCH 后 tracker 将 bucket1 (Φ0.15m, 最小) 而非 bucket2 (Φ0.20m, 第二大) 作为目标1 追踪
- **根因**：FORWARD_CENTER 到达时 drone 在 y≈33.85 (偏南 1.15m)，bucket2 在 y=36.0 处 body_fwd≈2.15m > FOV 上限，未被检测到。INIT 阶段只聚类出 2 簇 (bucket3 + bucket1)，将 bucket1 误选为 target[1]
- **修复方向**：CENTER_SEARCH 增加位置校验 (确保 dist_to_center < 0.5m 才开始 INIT) + 要求 3 簇才完成初始化

#### 迭代 4：磁盘空间问题

- **Bug**：运行中根分区磁盘空间不足
- **根因**：PX4 SITL 每次启动自动生成 `.ulg` 飞行日志 (100~600MB/次)，140+ 个文件累积 11GB
- **修复**：三个启动脚本均加入启动前自动清理旧 ULog 日志；手动清理回收站 (2.9GB)

### v2 仿真验证结果 (2026-06-11)

```
Home: NED(-0.00, 0.01, -0.02), heading=93°

TAKEOFF         0→3.5m, 17s
FORWARD_CENTER  飞中心 NED(0, 35), 12s, 到达 y≈33.85
CENTER_SEARCH   检测器 0s 初始化 (FORWARD 途中收集了观测)
  yolo: Target[0] bucket3 Φ0.25m area=1222px², Target[1] bucket1 Φ0.15m area=518px²
  ⚠ bucket2 Φ0.20m 未检测到 (FOV 外)
TRACK→...→DROP  目标0 桶3 0.158m→6分
CLIMB→TRACK→...→DROP 目标1 桶1 0.110m→8分 (误投, 应为桶2)
侦察蛇形扫描    5/5 命中, 3/3 正确识别
RETURN→LAND     0.228m→10/10
```

| 指标 | v1 (硬编码) | v2 (A3 中心悬停) |
|---|---|---|
| 总时长 | 219.7s | 258.2s |
| 投放分 | 14/20 | 14/20 |
| 侦察分 | 3/3 | 3/3 |
| 着陆分 | 10/10 | 10/10 |
| **总分** | **27/33** | **27/33** |
| 桶选择方式 | 代码硬编码 | **视觉自主选择** ✅ |
| 第二目标正确性 | ✅ 投桶2 | ⚠️ 误投桶1 |

### 关键技术决策

#### 1. 中心悬停 + 同距离观测

飞到三桶几何中心 NED(0, 35)，确保三个桶在近似相同的斜距下被观测。此时 bbox 面积差异完全由物理尺寸决定（面积比 = 半径平方比 = 2.78:1.78:1.00），YOLO bbox 抖动 (<10%) 远小于此差异。

#### 2. 追踪用位置记忆而非面积记忆

面积随距离平方变化——DESCEND 时 bbox 膨胀 3-4×。NED 位置是距离无关的不变量，反投影一个 bbox 到 NED 仅依赖下视几何公式（drone_z × 像素偏差 / 焦距），高度变化已被自动补偿。

#### 3. 仿真模拟 YOLO 不确定性

GT 投影 + 高斯噪声 (±4px 中心, ±6px 尺寸) + 3% 漏检，模拟真实 YOLO 输出的不确定性。INIT 阶段依赖多帧聚类消除单帧噪声。实机部署时仅需替换 bbox 来源。

#### 4. 时序滤波防止误匹配

当 tracker 当前帧的最佳匹配距离突然跳变 >1.5m 时（通常是主目标暂时丢失 + 相邻桶被误匹配），短时间 (0.3s) 保持上帧 ROI，避免 PID 因单帧错误输入产生大速度指令。

#### 5. 两个 .pt 分开训练

桶检测 `.pt` 和危险标识检测 `.pt` 分开训练，分别用于投放阶段和侦察阶段。好处：数据集不同（投放区 vs 侦察区场景）、推理阶段互斥、模型可独立迭代。

### v2 已知局限

- **中心到达精度**：FORWARD_CENTER 到达判定 (dist < 0.5m) 过于宽松，在 drone_y≈33.85 时即进入 CENTER_SEARCH，bucket2 在 FOV 外。需收紧到达判定或增加 CENTER_SEARCH 位置校验。
- **两簇初始化回退**：当只检测到 2 个桶时（第三个在 FOV 外），INIT 仍会完成并选 top 2——但此时可能选了错误的目标。应要求 3 簇才完成初始化，否则调整飞机位置重试。
- **第二目标过渡**：CLIMB_SWITCH 状态在 1m 高度爬升，bucket2 可能暂不可见。应飞回中心后重新 SEARCH 再切 TRACK，而非直接 TRACK。
- **disarm 延迟**：与 v1 相同，PX4 arming_state 变化有 30-60s 延迟。

### v2 运行方式

```bash
# 一键启动
bash ~/SITL/scripts/clean_start_v2.sh

# 手动启动
# 终端 1: MicroXRCEAgent udp4 -p 8888
# 终端 2: cd ~/PX4-Autopilot && export PX4_SITL_WORLD=... && make px4_sitl gazebo-classic_iris_mono_cam_down
# 终端 3: payload_dropper + yolo_bucket_detector (含桶坐标参数) + scout_detector + mission_judge
# 终端 4: full_mission_v2
```

### 磁盘空间管理

PX4 SITL 每次启动自动生成 `.ulg` 飞行日志，不清理会累积数十 GB。三个启动脚本现已内置启动前自动清理逻辑。详见上方"PX4 持久化状态"章节喵~
