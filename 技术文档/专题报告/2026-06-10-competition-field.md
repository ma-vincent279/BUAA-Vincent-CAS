# 项目报告 (2026-06-10 竞赛场地适配 + 蛇形侦察)

### 项目目标

将全任务仿真适配到标准竞赛场地 `worlds/competition_field.world`，实现：无人机从起降圈起飞 → 投放区双桶视觉伺服投弹 → 侦察区蛇形扫描五筒识别 → 返航着陆的完整竞赛任务闭环。

### 竞赛场地设计

竞赛场地 `competition_field.world` 经过两次几何变换从初始布局得到最终版本：

**变换 1：绕 Z 轴顺时针旋转 90°**
- 初始布局沿 Gazebo Y 轴（北），飞机朝东（heading≈92°）飞行方向与场地垂直
- 旋转后跑道沿 Gazebo X 轴（东）展开，飞机起飞后自然向前飞入任务区
- 变换公式：`(x, y) → (y, -x)`

**变换 2：沿 X 轴平移 +3.51m**
- 起降圈圆心从 Gazebo (-2.5, 1) 平移到 (1.01, 1)
- 使起降圈对齐 PX4 飞机 spawn 点，飞机默认位置在起降圈圆心处
- 着陆即返回起飞点，无需额外航点偏移

**场地最终参数：**

| 元素 | Gazebo 坐标 | 说明 |
|---|---|---|
| 跑道边界 | Y=-3 到 Y=5, X≈3.5 到 X≈66.5 | 宽 8m，中心 Y=1 |
| 起降圈 | (1.01, 1) | Φ5m 红色圆圈，圆心=spawn |
| 起飞线 | (3.51, 1) | 绿色横线 |
| 投放区 | X≈35~38 | 黄色半透明标记 |
| 侦察区 | X≈59~64 | 蓝色半透明标记 |
| 桶1 (Φ0.15m) | (35.51, 2, 0.15) | 红桶，r=0.075m |
| 桶2 (Φ0.20m) | (37.01, 1, 0.15) | 红桶，r=0.10m |
| 桶3 (Φ0.25m) | (35.51, 0, 0.15) | 红桶，r=0.125m（不投弹） |
| 筒0~4 | X≈59~64, Y 交替 ±2.5 | 5 白筒，r=0.10m, h=0.15m |

所有桶和筒模型均包含 `<static>true</static>` 和 `<collision>` 几何体，确保 Gazebo 正确渲染。

### NED 航点系统

采用硬编码 NED 偏移量，PX4 生成点 (1.01, 0.98, 0.83) 不变：

```python
# 投放区 (NED y≈34~36)
B1_WP_X = 1.0,  B1_WP_Y = 34.5   # 桶1
B2_WP_X = 0.0,  B2_WP_Y = 36.0   # 桶2

# 侦察区入口
SCOUT_WP_X = 0.0, SCOUT_WP_Y = 57.5

# 着陆区 (起降圈圆心 = spawn 点)
LAND_WP_X = 0.0, LAND_WP_Y = 0.0
```

### 视觉伺服（投放阶段）

投放阶段沿用 drop_opt 优化参数，仅对桶1和桶2进行伺服、下降、投弹：

| 状态 | 控制方式 | 说明 |
|---|---|---|
| TAKEOFF | 位置控制 | 3s 原地稳定 + 0.5m/s 爬升至 3.5m |
| FORWARD | 位置+速度混合 | 飞往桶1 NED(1.0, 34.5) |
| SEARCH | 速度控制 | 沿 heading 前飞搜索，检测到桶 → TRACK |
| TRACK | PID 速度伺服 | 像素偏差→机体速度，巡航高度锁定，err<40px 稳定 5s → DESCEND |
| DESCEND | PID + 匀速下降 | 水平 PID 锁定 + 0.25m/s 下降至 1m |
| HOVER | PID 速度锁定 | 低空像素收敛，err<25px 稳定 5s → DROP |
| DROP | PID 锁定 + 异步 | 调用 `/drop_payload`，桶1→MOVE_B2，桶2→RESULT_DROP |
| MOVE_B2 | 位置控制 | 飞往桶2 NED(0.0, 36.0) |

投放阶段参数：KP=0.006, KI=0.00005, KD=0.002, MAX_VEL=1.5, DROP_HEIGHT=1.0m, DESCEND_SPEED=0.25m/s, STABILIZE_TIME=5s。

### 蛇形扫描（侦察阶段）

**核心创新**：用蛇形扫描替代逐个 PID 伺服悬停，侦察耗时从 ~480s 降至 ~24s，5/5 全部命中。

```
FORWARD_SCOUT(飞侦察区入口 NED(0,57.5))
  → SCAN(蛇形扫描 6 个航点，位置控制 2m/s)
    航点0: (2.5, 58.5) 右 → 筒0
    航点1: (-2.5, 59.5) 左 → 筒1
    航点2: (2.5, 60.5) 右 → 筒2
    航点3: (-2.5, 61.5) 左 → 筒3
    航点4: (2.0, 62.5) 右 → 筒4
    航点5: (0.0, 63.5) 中 → 结束
   每个航点: 切换 scout_target_id，检测到筒 → 发 /scout/point_reached
  → RESULT_SCOUT → RETURN → LAND
```

蛇形扫描关键参数：SCAN_ARRIVE_DIST=2.0m, SCAN_SPEED=2.0m/s。ZIGZAG_SCAN 列表定义 6 个 (ned_x, ned_y, target_id) 航点，target_id=-1 表示终点。

### 接口变更

| 变更项 | 旧接口 | 新接口 |
|---|---|---|
| 桶检测器目标切换 | `/switch_target` (std_srvs/SetBool) | `/bucket_target_id` (std_msgs/Int32, 0/1/2) |
| 侦察检测器目标切换 | `/scout/target_id` (Int32) | 不变 |
| 桶数量 | 2 (红+蓝) | 3 (不同尺寸红桶，仅投 2 个) |
| 侦察方式 | 逐个伺服悬停 (SEARCH→TRACK→HOVER) | 蛇形扫描 (SCAN) |

### 裁判系统

| 项目 | 规则 | 满分 |
|---|---|---|
| 投放分 | d≤0.05→10, 0.05<d≤0.15→8, 0.15<d≤0.30→6, 0.30<d≤0.50→4, 0.50<d≤0.75→2, 0.75<d≤1.00→1, >1.00→0 | 20 (10×2) |
| 侦察分 | 正确识别有标识筒得 1 分 | 3 (5 筒中随机 3 个有标识) |
| 着陆分 | 降落在起降圈内 (d<2.5m) 得 10 分 | 10 |
| 总分 | 投放 + 侦察 + 着陆 | 33 |

### 仿真验证结果 (2026-06-10 最终版 v2，27/33)

```
Home: NED(0.03,-0.01,-0.03), heading=93°
起降圈圆心 = spawn 点 ✓

=== 投放阶段 (~108s) ===
TAKEOFF   0→3.5m, 12s
FORWARD   飞桶1 NED(1.0,34.5), 11s, 途中检测到桶 → TRACK
TRACK B1  252px→37px  收敛仅 5s (KP=0.006)
DESCEND   0.25m/s 下降至 1m, px_err 全程 <50px
HOVER B1  err<15px  低空锁定 4s
DROP B1   NED(1.06,34.52,-1.05) → gz(35.53,2.17) → 距桶1 0.173m → 6分
MOVE_B2   飞桶2 NED(0.0,36.0), 5s
TRACK B2  71px→18px  3s 收敛
DESCEND   px_err 从-30 渐变至+50 (桶2 投影或 heading 微漂)
HOVER B2  err<12px  低空锁定
DROP B2   NED(0.01,35.96,-1.02) → gz(36.97,0.86) → 距桶2 0.141m → 8分
投放合计  14/20 ✓

=== 侦察阶段 (~26s 蛇形扫描) ===
筒0: NED(0.8,57.6)  → 有标识(有毒) ✓
筒1: NED(-0.7,58.7) → 有标识(腐蚀) ✓
筒2: NED(0.7,60.1)  → 空 ✓
筒3: NED(-0.6,61.2) → 空 ✓
筒4: NED(0.1,62.2)  → 有标识(火焰) ✓
侦察合计  3/3 ✓

=== 返航 + 着陆 ===
RETURN    63m 返航 ~17s (4m/s)
LAND      降高至 0m, 9s → disarm
着陆判定  距起降中心 0.445m < 2.5m → 10/10 ✓
  (注: full_mission 176s 发 disarm, judge 219s 确认, 延迟 43s)
```

| 指标 | v1 (17/33) | v2 (27/33) |
|---|---|---|
| 总时长 | 260.8s | **219.7s** |
| 投放分 | 14/20 | 14/20 |
| 侦察分 | 3/3 | 3/3 |
| 着陆分 | 0/10 | **10/10** |
| **总分** | **17/33** | **27/33** |

v2 关键改进：着陆距离从 3.42m 降至 0.445m（起降圈半径从 80cm 放宽至 2.5m 后轻松及格），总时间缩短 41s。

### 文件清单

| 文件 | 说明 |
|---|---|
| `worlds/competition_field.world` | **新建** 竞赛标准场地（旋转+平移，8m 宽跑道，3 桶 5 筒） |
| `full_mission.py` | **重写侦察阶段** 蛇形扫描替代逐个伺服，硬编码 NED 航点，`/bucket_target_id` Int32 话题 |
| `bucket_vision_detector.py` | 3 桶参数 + 每桶独立半径 bbox，`/bucket_target_id` Int32 订阅替代 SetBool 服务 |
| `scout_detector.py` | 5 筒坐标适配竞赛场地 |
| `mission_judge.py` | 3 桶坐标 + 着陆区 Gazebo(1.01,1) 对齐 spawn，着陆分 d<2.5m |
| `payload_dropper.py` | 无改动 |
| `clean_start.sh` | 所有坐标参数更新为竞赛场地版 |

### 手动启动

```bash
# 终端 1: Agent
MicroXRCEAgent udp4 -p 8888

# 终端 2: PX4 + Gazebo
cd ~/PX4-Autopilot
export PX4_SITL_WORLD=/home/vincent/SITL/worlds/competition_field.world
export GAZEBO_MODEL_DATABASE_URI=""
export PX4_PARAM_CBRK_SUPPLY_CHK=894281
export PX4_PARAM_CBRK_IO_SAFETY=22027
export PX4_PARAM_CBRK_USB_CHK=197848
make px4_sitl gazebo-classic_iris_mono_cam_down

# 终端 3: 辅助节点
source ~/SITL/install/setup.bash
ros2 run drone_drl payload_dropper &
ros2 run drone_drl bucket_vision_detector --ros-args \
    -p bucket1_gz_x:=35.51 -p bucket1_gz_y:=2.0 \
    -p bucket2_gz_x:=37.01 -p bucket2_gz_y:=1.0 \
    -p bucket3_gz_x:=35.51 -p bucket3_gz_y:=0.0 &
ros2 run drone_drl scout_detector --ros-args \
    -p c0_gz_x:=59.51 -p c0_gz_y:=2.5 \
    -p c1_gz_x:=60.51 -p c1_gz_y:=-0.5 \
    -p c2_gz_x:=61.51 -p c2_gz_y:=2.0 \
    -p c3_gz_x:=62.51 -p c3_gz_y:=0.0 \
    -p c4_gz_x:=63.51 -p c4_gz_y:=1.0 &
ros2 run drone_drl mission_judge --ros-args \
    -p bucket1_gz_x:=35.51 -p bucket1_gz_y:=2.0 \
    -p bucket2_gz_x:=37.01 -p bucket2_gz_y:=1.0 \
    -p landing_gz_x:=1.01 -p landing_gz_y:=1.0 &

# 终端 4: 任务
source ~/SITL/install/setup.bash
ros2 run drone_drl full_mission 2>&1 | tee /tmp/full_mission.log
```

一键启动：`bash ~/SITL/scripts/clean_start.sh`

### 已知问题

- **着陆 disarm 延迟**：full_mission 通过高度回退 disarm 后，PX4 确认 arming_state 变化有延迟（本次实测 43s，之前最差 68s）。期间飞机水平漂移 0.5~1m，full_mission 记录位置与 judge 实际判定位置有偏差。好在竞赛场地着陆半径 2.5m 足够宽容，不影响及格喵~
- **`visual_servo_mission.py` / `drop_opt_mission.py`** 仍引用旧 `/switch_target` SetBool 服务（bucket_vision_detector 已改为 `/bucket_target_id` Int32），如需使用需同步修改喵~
