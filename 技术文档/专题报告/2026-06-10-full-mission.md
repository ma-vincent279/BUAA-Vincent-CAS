# 项目报告 (2026-06-10 全任务串联)

### 项目目标

在 PX4 SITL Gazebo 仿真环境中，实现无人机**一次起飞**串联执行：投放区(30m)双桶视觉伺服投弹 + 侦察区(55m)五筒视觉伺服侦察，**计时 + 细化计分 (10/8/6/4/2/1/0)** 的完整竞赛任务闭环。

### 系统架构

```
mission_full.world (Gazebo)
  ├─ red_bucket_1  (红桶, Gazebo 31.01,3.68 → NED x≈2.7,y≈30, 投放区)
  ├─ blue_bucket_2 (蓝桶, Gazebo 31.01,-2.12 → NED x≈-3.1,y≈30, 投放区)
  └─ scout_cylinder_1~5 (白筒, Gazebo 53.5~58, 0~2, 侦察区)

bucket_vision_detector ──→ /bucket/detection (ROI, 投放阶段)
scout_detector         ──→ /scout/detection (ROI, 侦察阶段)

full_mission ──→ /fmu/in/trajectory_setpoint (速度/位置)
  │  13 状态机, 串行投弹+侦察, PID 视觉伺服
  │  投放阶段 1Hz PID 监控日志 (TRACK/DESCEND/HOVER/DROP)
  │  /drop_payload → payload_dropper → /spawn_entity
  │  /scout/point_reached → mission_judge → /scout/result
  ↓
mission_judge — 统一打分 (投放细化10/8/6/4/2/1/0 + 侦察 + 着陆 + 计时 + 总分)
```

### 状态机流程

```
TAKEOFF(爬升3.0m, 投放阶段巡航)
  → FORWARD(飞桶1航点: 30m前+3m右)
  → SEARCH(检测桶1) → TRACK(PID+高度锁定, err<40px 稳定5s) → DESCEND(0.25m/s下降至1.0m)
  → HOVER(PID低空收敛, err<25px 稳定5s) → DROP(零速保持+投桶1)
  → MOVE_B2(飞桶2航点: 30m前+3m左)
  → SEARCH(检测桶2) → TRACK → DESCEND → HOVER → DROP(投桶2)
  → RESULT_DROP(投放完成)
  → FORWARD_SCOUT(飞侦察区 NED y=50, 纯东, 巡航3.5m)
  → (SEARCH → TRACK → HOVER)×5 (逐筒侦察识别, 侦察高度3.5m)
  → RESULT_SCOUT → RETURN(返航) → LAND(着陆 disarm)
```

### 参数 (2026-06-10 优化版, 来自 drop_opt 移植)

| 参数 | 数值 |
|---|---|
| 巡航高度 (投放阶段) | 3.0m |
| 巡航高度 (侦察阶段) | 3.5m |
| 投放高度 | 1.0m |
| 投放区距离 | 30m |
| 桶间距 | 6m |
| 侦察区距离 | 55m |
| PID 桶(近程) | KP=0.006, KI=0.00005, KD=0.002, MAX_VEL=1.5 |
| PID 筒(远程) | KP=0.0015, KI=0.00003, KD=0.0005, MAX_VEL=0.6 |
| 下降速度 | 0.25 m/s (NED z+) |
| TRACK 收敛阈值 | 40px |
| HOVER 收敛阈值 | 25px |
| 稳定对准时间 | 5s |
| 着陆区直径 | 80cm |

### 计分规则 (细化版, 移植自 drop_opt_judge)

| 项目 | 规则 | 满分 |
|---|---|---|
| 投放分 | d≤0.05→10, 0.05<d≤0.15→8, 0.15<d≤0.30→6, 0.30<d≤0.50→4, 0.50<d≤0.75→2, 0.75<d≤1.00→1, d>1.00→0 | 20 (10×2) |
| 侦察分 | 正确识别有标识筒得 1 分, 空筒 0 分 | 3 |
| 着陆分 | 降落在起降点 80cm 直径内 | 10 |
| 总分 | 投放 + 侦察 + 着陆 | 33 |

### 侦察识别模拟

- mission_judge 在启动时随机预设 3/5 筒有标识（类型随机：火焰/腐蚀/有毒）
- full_mission 每个筒 HOVER 后发布 `/scout/point_reached` (Int32)
- mission_judge 查 ground truth 返回 `/scout/result` (String: "筒X: 有标识(类型)" 或 "筒X: 空")
- 全任务节点打印识别结果，裁判自动计分

### 计时

- 从 PX4 arming_state 变为 2 (ARMED) 开始
- 到 arming_state 变为非 2 (DISARMED) 停止
- 同时由 full_mission 和 mission_judge 独立计时

### 投放阶段 1Hz PID 监控日志

投放阶段的 TRACK/DESCEND/HOVER/DROP 四个状态每秒打印详细日志（仅 bucket 阶段，侦察阶段不打印）：

```
[TRACK] px_err=(ex,ey) total=err px pos=(x,y,z) vel=(vx,vy,vz)
[DESCEND] px_err=(ex,ey) pos=(x,y,z) vel=(vx,vy,0.250) z→drop_z
[HOVER] px_err=(ex,ey) total=err px pos=(x,y,z) vel=(vx,vy,vz) drop_z=z
[DROP] px_err=(ex,ey) pos=(x,y,z)
```

### 文件

| 文件 | 说明 |
|---|---|
| `worlds/mission_full.world` | 双桶(30m) + 五筒(55m) 世界 (无改动) |
| `full_mission.py` | 全任务节点 (13 状态机) — **已更新**: 投放阶段参数采用 drop_opt 优化值, 投放/侦察巡航高度分离, 1Hz PID 监控日志 |
| `mission_judge.py` | 统一裁判节点 — **已更新**: A/B/Miss 计分 → 细化 10/8/6/4/2/1/0, 总分 17→33 |
| `clean_start.sh` | 一键启动全任务 (无改动) |
| `setup.py` | full_mission / mission_judge 入口 (无改动) |
| `bucket_vision_detector.py` | 通过参数配置 30m (无改动) |
| `scout_detector.py` | 无改动 |
| `payload_dropper.py` | 无改动 |

### 仿真验证结果 (2026-06-10)

```
Home: NED(0.02,-0.01,-0.06), heading=92°

TAKEOFF   z: 0→-3.0  投放阶段 3.0m 巡航 ✓
FORWARD   飞往桶1 NED(1.4,30.1), 30m 投放区
TRACK     PID 收敛 200px→13px ~5s, KP=0.006 生效 ✓
DESCEND   z: -3.0→-1.0  0.25m/s 下降, DESCEND_SPEED 生效 ✓
HOVER     1m 低空收敛, HOVER_ARRIVE_PX=25px ✓
DROP      投桶1 (payload_l) → switch → MOVE_B2
MOVE_B2   飞往桶2 NED(-3.1,30.1)
SEARCH→TRACK→DESCEND→HOVER→DROP  投桶2 (payload_r)
FORWARD_SCOUT  侦察区 3.5m 巡航 (SCOUT_HEIGHT 隔离变量 ✓)
SEARCH→TRACK→HOVER ×3 (筒0空, 筒1有毒, 筒2空, 筒3腐蚀)
筒4 搜索超时 (tracking 丢失, FOV 边缘)
RESULT_SCOUT → RETURN → LAND → disarm
```

| 指标 | 数值 |
|---|---|
| 总时长 | 384.4s |
| 投放1 落点距离 | 0.151m → 6分 |
| 投放2 落点距离 | 0.148m → 8分 |
| 投放合计 | 14/20 |
| 侦察识别 | 2/3 (筒1有毒✓, 筒3腐蚀✓, 筒4超时漏检) |
| 着陆距离 | 0.879m (超过 80cm 直径) |
| 着陆得分 | 0/10 |
| **总分** | **16/33** |

### 关键发现

1. **投放精度大幅提升** — 30m 投放距离下达到 0.15m 精度（接近 8 分线仅差 0.0005m），远超 drop_opt 在 10m 的 0.21m 成绩。PID KP=0.006 + HOVER_VZ_KP=0.8 在高空风力扰动下也能快速收敛。
2. **投放/侦察巡航高度分离** — `TAKEOFF_HEIGHT=3.0` 用于投放阶段，`SCOUT_HEIGHT=3.5` 用于侦察阶段，通过 `_task_phase` 自动切换。避免侦察阶段过低导致 FOV 覆盖不足。
3. **筒4 丢失** — 第 5 个筒 (index 4) tracking 丢失：短暂捕获后因 FOV 边缘位置检测不稳定，SEARCH 冲出 y>60m 上限后超时。白筒 NED x 散布 ±1m 在 55m 远处接近 FOV 边界。
4. **着陆偏差** — 0.879m 超过 80cm 着陆区直径。LAND 阶段使用位置控制 `_pub_pos(home_x, home_y, home_z)` 无视觉辅助，精度依赖 GPS/ekf2 漂移。

### 运行方式

```bash
bash ~/SITL/scripts/clean_start.sh
```

一键完成：进程清理 → PX4 状态重置 → 编译 → Agent → PX4+mission_full.world → DDS等待 → payload_dropper + bucket_vision_detector(30m) + scout_detector + mission_judge → full_mission

最终输出包含任务时长、各分项得分及总分 (满分 33) 喵~
