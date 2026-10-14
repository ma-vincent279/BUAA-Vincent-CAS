# SETUP — 仿真环境搭建与运行

> 完整版（含依赖安装、PX4 侧改动、排障）见
> [`../../../技术文档/配置仿真环境.md`](../技术文档/配置仿真环境.md)。
> 本文只列**跑起来需要的最小步骤**。

## 依赖

| 依赖 | 版本 | 注意 |
|---|---|---|
| Ubuntu | 22.04 | — |
| ROS 2 | Humble | — |
| Gazebo | **Harmonic (gz-sim8)** | ⚠️ 不是 Classic |
| `ros-humble-ros-gzharmonic` | — | ⚠️ 必须这个变体（默认的 `ros-gz` 是 Fortress，不兼容）|
| PX4-Autopilot | 1.17.0 | `~/PX4-Autopilot`，`make px4_sitl_default` |

## 最小步骤

```bash
# 1. 克隆（注意 --recurse-submodules）
git clone --recurse-submodules git@github.com:ma-vincent279/BUAA-Vincent-CAS.git ~/SITL
cd ~/SITL

# 2. 补 Gazebo 网格（未入库，从 PX4 逐字节拷）
cp -r ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/{meshes,materials} gz_models/x500_base/
cp -r ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/{meshes,materials} gz_models_combined/x500_base/

# 3. PX4 侧 4 行改动（让环境变量能覆盖路径）—— 详见 技术文档/配置仿真环境.md 第 1.4 节

# 4. 编译
colcon build --packages-select drone_drl
source install/setup.bash

# 5. 离线验收（秒级）
python3 scripts/test_k_servo_20261011.py     # 期望: 104 项通过

# 6. 跑一轮
bash scripts/test_k.sh                       # 期望: 后端 gz / 初始化完成 / 800 分

# 7. ⚠ 每轮前清进程
bash scripts/cleanup_sim.sh
```

## 产物

`runs/k_<seed>_<时间戳>/`：

| 文件 | 内容 |
|---|---|
| `mission.log` | 主日志（状态流转 / `[K-SERVO]` 逐拍 / `>>> DROP` 释放行）|
| `detector.log` | 检测器 |
| `judge.log` | 裁判 |
| `summary.json` | **分数**（`total_score`）|
| `world.sdf` / `truth.json` | 本轮场地快照 |

## 开关

```bash
SIM_GUI=1   bash scripts/test_k.sh    # 打开 Gazebo 界面
GT_JUDGE=1  bash scripts/test_k.sh    # 真值口径打分
bash scripts/test_k.sh hover_probe    # 悬停特性探针
```
