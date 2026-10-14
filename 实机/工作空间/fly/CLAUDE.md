# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session Rules (用户要求)

1. **每一步操作都要留下备份。** 修改任何文件前，先 `cp` 生成带时间戳的备份（如 `.bak.20260723_164219`）。
2. **每一步操作都要可逆。** 给出回滚方法（反向 cp + rebuild 命令）。
3. **只做最小化修改，不做额外重构。**
4. **说中文。**
5. **代码审查：审查逻辑正确性和 bug，不做风格/命名/nitpick 评论。**

## 项目概述

**无人机机载伴侣计算机系统**，运行于 NVIDIA Jetson Orin NX，通过 DDS/ROS 2 与 PX4 1.17.0 飞控通信。完成**视觉检测 → 视觉伺服追踪 → 自主投放**的完整自主任务。

### 硬件平台

- **Jetson Orin NX** (ARMv8 aarch64, 8-core Cortex-A78AE), 7.4GB RAM
- **飞控**: PX4 1.17.0, 通过 `/dev/ttyTHS1` 串口 DDS 通信
- **摄像头**: USB 摄像头 `/dev/video0`, MJPG/YUYV, 最高 1920x1080@30fps
- **投放舵机**: AUX5 PWM 通道, 1500μs(锁)/2000μs(放)
- **电源模式**: 15W (nvpmodel mode 2), 4 CPU cores, GPU 612MHz

### 软件栈

- **OS**: Ubuntu 22.04.5 LTS, NVIDIA L4T R36.4.3, kernel 5.15.148-tegra
- **ROS 2**: Humble at `/opt/ros/humble`
- **ML**: PyTorch 2.5 (NVIDIA custom), TensorRT 10.7, Ultralytics 8.3.235
- **Conda**: miniforge3, YOLO env `yolov8` at `/home/nvidia/miniforge3/envs/yolov8/`

## 项目成果总览

### 节点清单

| Entry point | File | 用途 | 实飞验证 |
|-------------|------|------|:---:|
| `offboard` | `offboard.py` | Offboard 起降任务 | yes |
| `path` | `path.py` | 9 航点路径跟随 | yes |
| `yolo` | `yolo.py` | YOLOv8 TensorRT 桶检测 + 可视化 (51 FPS) | **ground** |
| `test` | `test.py` | 地面测试: ARM→检测→投放→循环 | **ground** |
| `dj` | `dj.py` | AUX5 PWM 三角波测试 (900~2100μs) | yes |
| `drop` | `drop.py` | **完整自主投放任务** (10 步状态机) | **no** |
| `drop_wide` | `drop_wide.py` | **广角版投放** (130° HFOV) | **no** |
| `dds_listener` | `dds_listener.py` | DDS 话题调试监听 (已验证可用) | yes |
| `b` | `b.py` | **子项目 b 实机版** 投放区+侦察区悬停 (固定航向 HEADING=54° + 分桶补偿) | **2026-08-14 实飞** |
| `c` | `c.py` | **子项目 c 实机版** 基于 b 分叉 (DROP_HEIGHT 1.5m + 距离+2m + 侦察 7 子阶段防过冲 + RETURN 判稳 + LAND 限速 + B1补偿移除) | **2026-08-15 实飞** (发现问题后已修复, SITL 复验着陆 0.20m, 待复飞) |
| `d` | `d.py` | **子项目 d 实机版** 基于 c 分叉 (下降限速时间锚定修复 + B1补偿+0.10 + 阶段日志增强 + 2026-08-27 合并 c 侦察 7 子阶段/RETURN 判稳/LAND 限速/DROP_AFTER_DELAY 15) | **no** (2026-08-15 新建, SITL 已验证 18/20 分, 合并后待复验) |
| `see` | `see.py` | **视觉节点** (YOLO 检测 → `/bucket/detections`; 1920×1080 全帧 `resize` 到 640×360, **不裁剪**) | **yes** |
| `e` | `e.py` | 子项目 e (低空段去闭环伺服, 改锚点+反投影) | 2026-09-23/24 实飞 |
| `f` | `f.py` | 子项目 f (两级释放判据 Tier1/Tier2/Tier3) | 2026-09-24/27 实飞 |
| `g` | `g.py` | 子项目 g (两瓶横向补偿重标: Y_B1 +0.113 / Y_B2 −0.025) | 2026-09-27 实飞 |
| `h` | `h.py` | 子项目 h (预测落点 Tier0 / 重进 / 释放软门控 等 9 项) | SITL 16/20 |
| `i` | `i.py` | **子项目 i** (低空"冻结+定点保持+必投"; 14 投实测 12 趟 7~12s 收敛) | 2026-10-06~09 多趟 |
| `j` | `j.py` | 子项目 j (i + 相机确认门 `dev ≤ J_DEV_GATE_M`, 默认关) | no |
| **`k`** | **`k.py`** | ★ **CUADC 伺服投弹任务** (= i + D1~D37; DROP 段改**机体坐标系闭环**, 释放判据换成他们的 **3cm/2.0s**; 相机换原生 130° 画幅; 补偿归零) | **待首飞** (见下) |

### ★ k (实机版) —— 上板流程与首飞

**代码位置**: `k` 有**两条平行血统**, 改哪边要清楚 ——
| | 仿真 | 实机 |
|---|---|---|
| 工作空间 | `~/SITL/src/drone_drl/` | `~/SITL/fly/` → 板载 `~/fly/` |
| 任务节点 | `drone_drl/k.py` | `fly/src/fly/fly/k.py` |
| 血统源 | `drone_drl/i.py` | `fly/src/fly/fly/i.py` |
| 启动 | `bash scripts/test_k.sh` | `bash ~/fly/k.sh` |

⇒ "实机 k 与仿真 k 的 diff"**没有意义** (差 154 行, 全是实机该有的适配: 内联 common / PID 抽帧 /
`GROUND_NED_Z=0` / 航向 180°)。**该保证的是**: ①每版 vs **它自己的** `i.py`;
②**两版之间 34 个 `K_*` 常量不许漂移** —— `scripts/test_k_fly_20261013.py` 在查这个 (目前 42/42)。

**上板** (在本机 `~/SITL` 下跑):
```bash
python3 scripts/test_k_fly_20261013.py          # ① 离线验收 42 项 (常量/环境侧/D标记/危险项)
bash scripts/deploy_k_jetson_20261008.sh        # ② scp + HEADING_DEG=212 + 建入口 + 编译 + 校验
bash scripts/deploy_k_jetson_20261008.sh --rollback   # 回退
```
部署后**必须核对** `~/fly/src/fly/fly/k.py` 与 `~/fly/install/.../fly/k.py` **sha 一致**
(不一致 = 改了源码但跑的还是旧产物)。

**首飞**:
```bash
bash ~/fly/k.sh                 # 自动落盘 ~/fly/log/k_<时间戳>.log (--no-log 可关)
```
- 完整清单: `~/SITL/docs/reports/k_flight_prep_20261013.md` (现场可照做)
- 现场通常只核对 **`HEADING_DEG`**(部署脚本自动改 212.0) 与 **`FORWARD_DIST`**(32.5)
- 其余保持默认: `K_ZERO_COMPENSATION=True`(补偿归零) / `K_TARGET_PLANE_H=-0.278` /
  `K_SIM_NO_BALLISTIC=False` / `K_SIM_IDEAL_CAMERA=False` / `K_RELEASE_MIN_M=0.03`(他们的 3cm) /
  `K_RELEASE_CONFIRM=2.00`(他们的 2.0s) / `K_DEADLINE_S=12.0`

**降落后** (回答"能不能满足投弹稳定条件"):
```bash
python3 scripts/analyze_k_flight_20261013.py <日志> [--ulog <x.ulg>]
```
⚠ 判定看 **`触发方式=`**(节点自报), **不是** `win=` —— `win` 是"到达后已等待时长"
(3.0s 后容差放宽 +3cm), 不是那个"3cm 保持 2.0s"的稳定窗。

### 项目阶段

| 阶段 | 内容 | 状态 |
|------|------|:--:|
| 1. 基础飞控通信 | offboard 起降、path 航点、DDS 链路验证 | done |
| 2. 视觉检测 | YOLOv8 TensorRT 部署、摄像头配置、检测发布 | done |
| 3. 舵机控制 | AUX5 ManualControlSetpoint 200Hz PWM 控制 | done |
| 4. 地面联调 | test.py + yolo.py 地面检测-投放闭环 | done |
| 5. 自主投放 | drop.py 完整飞行投放状态机 | 代码完成，待实飞 |
| 6. 可视化调试 | yolo.py 实时检测画面显示 | done |

## 技术方案

### 坐标系统

- **NED 帧**: x=北, y=东, z=下。`home_z` 为负值(地面)，巡航高度 `target_z = home_z - 3.0`(更负)
- **机体坐标系**: x=前, y=右, z=下。`_body_to_ned()` 用当前 heading 做旋转
- **下视摄像头**: 图像上方(Top)=机头前方。`ey = CY - cy`，前方物体 ey>0→v_fwd>0→前进
- **投放位置估计**: `_roi_to_ned()` 使用当前航向做 body→NED 转换

### 视觉检测 (yolo.py)

- TensorRT 引擎: `/home/nvidia/balloon.engine` (640x640, FP16)
- 输入: 640x480, 推理节流 10Hz
- 置信度阈值: 0.65
- 目标类别: `"balloon"` (热气球/桶)
- 摄像头: `cv2.CAP_V4L2` + MJPG fourcc
- **可视化**: 默认开启 (`visualize:=true`)，`cv2.imshow` 显示检测框叠加画面
- **性能优化** (2026-07-31):
  - 线程化摄像头: `_camera_loop()` daemon 线程零阻塞读帧
  - 向量化后处理: `np.lexsort((-areas, -conf_v))` 替代 Python for 循环
  - 局部变量缓存: 减少 `self.` 属性查找
  - 推理延迟: 25.4ms → 19.4ms (40 → 51 FPS)
  - 480x480 引擎反而更慢 (预处理下采样开销 > 推理节省), 640x640 最优

### PID 视觉伺服 (drop.py)

- 像素误差 → 机体速度 (KP=0.0015, KI=0.0, KD=0.001) — 2026-08-02 实飞调参
- 限幅: MAX_HORIZ_VEL=0.3m/s, INTEGRAL_LIMIT=0.0 (纯 PD, 无 I 增益)
- 投放偏移补偿: `pixel_offset = PAYLOAD_OFFSET_BODY_Y * FX / dz`

### AUX5 舵机控制

- 发布 `ManualControlSetpoint` 到 `/fmu/in/manual_control_input` @ 200Hz
- 必须超过 SBUS RC 更新率 (~111Hz) 才能覆盖接收机信号
- `aux5=0.0`→1500μs(锁), `aux5=0.83333`→2000μs(放)
- PWM = 1500 + aux5_value × 600

### Drop 投放状态机 (v4, 2026-08-07)

```
TAKEOFF → STABILIZE → FORWARD → HOVER_SEARCH → TRACK → DESCEND → DROP → CLIMB → RETURN → LAND
```

11 步状态机，高度 ≤ 3m。v4 关键设计:

- **TAKEOFF**: 位置保持 2s → 竖直爬升到巡航高度 3m
- **STABILIZE**: 固定 yaw 悬停 3s 稳姿态 → 锁定 heading0 + home
- **FORWARD** (v4): `_pub_vel` 速度控制, `vx,vy=speed*cos_h0,speed*sin_h0` 沿 heading0 直飞, ramp-down 刹车。v3 的 `_pub_pos` 因实机方向错误已回退
- **HOVER_SEARCH** (v3): `_pub_pos` 位置保持 + 驻留 1.5s 等 MPC 收敛，然后四层门控搜目标
- **TRACK→DESCEND→DROP**: 视觉伺服 + 抽帧控制 + AUX5 投放。PID height_scale 按 H/CRUISE_HEIGHT 缩放增益
- **Yaw 策略**: 盲飞阶段 yaw=heading0，视觉伺服阶段 yaw=live heading
- **TRACK 切入门控**: 四条件 — (1)检测新鲜 (2)距桶预期位置<3m (3)地理位置一致性<1.5m (4)ROI 中心漂移<120px 稳定 1.5s
- **丢检测恢复**: 原地悬停修正高度。TRACK 丢检测 wait; DESCEND 丢检测 3s 后盲降到投放高度→直接 DROP
- **超时兜底**: STEP_TIMEOUT/_track_max_time/HOVER_SEARCH_TIMEOUT 强制推进

### 关键可调参数 (v4 2026-08-07)

| 参数 | 值 | 说明 |
|------|-----|------|
| KP_H | **0.0015** | 像素→机体速度 P 增益 (实飞纯 PD 极低增益) |
| KI_H | **0.0** | 纯 PD 控制, 去除 I 增益防积分 windup |
| KD_H | **0.001** | 轻阻尼 |
| MAX_HORIZ_VEL | **0.3** m/s | 极低速限幅 |
| DEAD_ZONE_PX | **5** px | PID 死区, 误差<5px 不输出 |
| **height_scale** | **H/CRUISE_HEIGHT** | KP/KD 按高度缩放, 低空防摇晃 (v3) |
| PID_DECIMATION_TRACK | **4** | 抽帧率, 20Hz/4=**5Hz** 有效修正 |
| PID_DECIMATION_DESCEND | **5** | 20Hz/5=**4Hz** |
| PID_DECIMATION_HOVER | **5** | ~~20Hz/5=4Hz~~ (HOVER 已跳过, 未使用) |
| CRUISE_HEIGHT | 3.0m | 巡航高度 |
| DROP_HEIGHT | 1.0m | 投放高度 |
| FORWARD_DIST | 10.0m | 前飞距离 |
| **FORWARD_RAMP_DIST** | **3.0m** | FORWARD 速度 ramp-down 距离 (v4) |
| **FORWARD_MIN_SPEED** | **0.5** m/s | FORWARD ramp-down 最低速度 (v4) |
| **HOVER_SEARCH_SETTLE_TIME** | **1.5s** | HOVER_SEARCH 驻留等 MPC 收敛 (v3) |
| CRUISE_SPEED | **2.0** m/s | 巡航/返航速度 |
| SEARCH_SPEED | 0.3m/s | 未使用 (HOVER_SEARCH 已改 _pub_pos) |
| DESCEND_SPEED | 0.25m/s | 下降速度 |
| CLIMB_VZ | -0.8 m/s | 爬升速度 (NED z-) |
| TAKEOFF_CLIMB_RATE | **0.8** m/s | 起飞竖直爬升速率 |
| TRACK_ARRIVE_PX | 40px | TRACK 收敛阈值 |
| HOVER_ARRIVE_PX | 25px | ~~HOVER 精调阈值~~ (DESCEND 直达 DROP, 未使用) |
| STABILIZE_TIME | 5.0s | 收敛稳定计时 |
| TRACK_MAX_TIME | 60.0s | TRACK 强制推进 |
| HOVER_MAX_TIME | 30.0s | ~~HOVER 强制推进~~ (DESCEND 直达 DROP, 未使用) |
| STABILIZE_HOVER_TIME | **3.0** s | 起飞后稳定悬停时间 |
| HOVER_SEARCH_TIMEOUT | **10.0** s | 到点搜索超时, 无目标→返航 |
| DETECT_TIMEOUT | 3.0s | 检测新鲜度超时 |
| TRACK_ENTRY_DIST_MAX | 3.0m | TRACK 切入距离门控 |
| TRACK_ENTRY_GEO_ERR_MAX | 1.5m | TRACK 切入地理一致性 |
| TRACK_ENTRY_CONFIRM_TIME | 1.5s | TRACK 切入确认时间 |
| TRACK_ENTRY_ROI_DRIFT | 120px | TRACK 切入 ROI 漂移门控 |

## 已解决的问题 (2026-07-23 ~ 2026-07-27)

### 1. NumPy 版本冲突 → yolo 启动崩溃

**现象**: `ros2 run fly yolo` 报 `_ARRAY_API not found` / `Numpy is not available`

**根因**: 系统 pip PyTorch/ultralytics (NumPy 1.x 编译) 与 conda NumPy 2.2.6 不兼容。`ros2 run` 把系统路径放在 conda 前面。

**修复**: 启动 yolo 前设置 PYTHONPATH:
```bash
export PYTHONPATH="/home/nvidia/miniforge3/envs/yolov8/lib/python3.10/site-packages:$PYTHONPATH"
```

### 2. 摄像头分辨率不生效

**现象**: `Camera opened: /dev/video0 (1920x1080)` 尽管配置了 640x480

**根因**: `cv2.VideoCapture(camera_device)` 默认后端无法对 V4L2 设置分辨率

**修复** (`yolo.py`, 备份 `yolo.py.bak.20260723_164219`): 使用 `cv2.CAP_V4L2` 后端 + MJPG fourcc

### 3. vehicle_status_v1 DDS 类型不匹配 → 5个节点回调不触发 (2026-07-31 全面修复)

**现象**: `/fmu/out/vehicle_status_v1` DDS 层有数据但 ROS 2 订阅回调不触发。`vehicle_local_position_v1` 和 `vehicle_control_mode` 正常。

**根因**: PX4 固件 VehicleStatus 消息定义与 px4_msgs 包 type checksum 不匹配，DDS 在 type-level 丢弃消息。

**受影响及修复**:

| 文件 | 问题 | 修复 |
|------|------|------|
| `dds_listener.py` | 仅订阅 vehicle_status_v1 → 回调从不触发 | 重写: 订阅 vehicle_control_mode + vehicle_local_position_v1 + vehicle_land_detected |
| `dj.py` | `self.armed = msg.arming_state` → 永远 False | `VehicleControlMode.flag_armed` |
| `offboard.py` | `arming_state` + `nav_state` → 模式切换永远不触发 | `VehicleControlMode.flag_armed` + `flag_control_offboard_enabled` |
| `path.py` | 同上 + `xy_valid` 室内常 False → home 无法设置 | VehicleControlMode + 移除 xy_valid 条件 |
| `test.py` | 死代码 `_cb_status` + `pre_flight_checks_pass` 检查 | 删除死代码和 VehicleStatus 引用 |

`drop.py` 和 `drop_wide.py` 此前已使用 VehicleControlMode，无需修改。

**根源修复需要重新同步 px4_msgs 与 PX4 固件版本。**

### 4. xy_valid=false 导致 home_x 无法设置

**现象**: test 节点在 `if self.home_x is None` 处永久阻塞

**根因**: 室内无 GPS，`xy_valid` 始终 false。原代码要求 `xy_valid == true` 才设置 home。

**修复** (`test.py`): 移除 `m.xy_valid` 条件，使用 dead-reckoning 坐标。

### 5. YOLO 模型路径更新

`model_path` 从 `best2.engine` 更新为 `balloon.engine` (`/home/nvidia/yolo_v8/balloon.engine`)。

### 6. PID 下视摄像头 ey 符号修正 (drop.py 2026-07-27)

**问题**: 旧备份 `drop.py.bak.20260726_duplicated` 中 `PIDController.compute` 的 `ey = cy - IMG_H / 2.0`，与 `_roi_to_ned` 中 `CY - cy` 符号不一致。

**修复**: `ey = IMG_H / 2.0 - cy`，下视摄像头 TOP=前，前方物体 cy<CY, ey>0→前进。与 `_roi_to_ned` 统一。

### 7. 备份文件整理 (2026-07-27)

旧备份 `drop.py.bak.20260726_duplicated` 为 74KB 重复拼接文件（两份代码首尾相连），已删除并用当前版本重建。

### 8. yolo.py 可视化默认开启 (2026-07-27)

`visualize` 参数默认值从 `False` 改为 `True`。启动 yolo 节点自动弹出检测画面窗口。可通过 `--ros-args -p visualize:=false` 关闭。

### 9. drop.py FORWARD/SEARCH 新增地理一致性门控 (2026-07-27)

新增 `TRACK_ENTRY_GEO_ERR_MAX = 1.5m`，用 `_roi_to_ned()` 估计检测物的 NED 坐标，与桶预期位置比较，防止远处 YOLO 误检触发 TRACK 切入。

### 10. yolo.py 性能优化 (2026-07-31)

**优化前**: ~25.4ms/帧 (40 FPS)，其中 `cap.read()` 阻塞、for 循环后处理、frame.copy() 开销。
**优化后**: ~19.4ms/帧 (51 FPS)，提升 27%。

具体优化:
- 线程化摄像头: `_camera_loop()` daemon 线程持续读帧, 主线程取最新帧零阻塞
- 向量化 `_best_detection()`: `np.lexsort((-areas, -conf_v))` 替代 Python 循环
- 移除 `frame.copy()`: 直接绘制在原帧上 (视觉线程已有锁保护)
- 局部变量缓存: `thresh/conf_threshold`, `target_cls/_target_cls_idx`, `imw/img_w` 等减少 self. 属性查找
- 480x480 引擎反而更慢 (下采样 5.6ms 开销 > 推理节省的 1.2ms), 640x640 保持最优

模型路径: `/home/nvidia/balloon.engine` (640x640 TensorRT FP16)
备份: `/home/nvidia/balloon_640.engine`

### 11. drop_wide.py 广角版投放 (2026-07-31)

从 SSD2 复制, 130° HFOV 广角版 drop.py。注册为 `drop_wide` 入口点。setup.py 新增对应 entry point。

### 12. dds_listener.py 重写 (2026-07-31)

旧版仅订阅 `vehicle_status_v1` (回调从不触发)。重写后订阅 3 个验证过的话题:
- `/fmu/out/vehicle_control_mode` (2Hz, flag_armed + flag_control_offboard_enabled)
- `/fmu/out/vehicle_local_position_v1` (50Hz, 位置+航向)
- `/fmu/out/vehicle_land_detected` (着陆检测)

每 2s 打印状态摘要，确认 DDS 链路正常。

### 13. launch_drop.sh 一键启动 (2026-07-31)

`~/fly/launch_drop.sh` 一键启动完整链路:
- 自动检测/清理已有 Agent
- `sg dialout` 启动 DDS Agent → 等 5s → yolo.py → 等 8s → drop.py
- 预检: 串口存在、话题注册、摄像头可用
- watchdog 监控进程存活
- `--log` 模式保存日志
- Ctrl+C 清理所有子进程

### 14. 实机 TRACK PID 振荡 → 纯 PD 极低 KP (2026-08-02)

**问题**: 实机 5.3kg + 15 寸桨飞行中，SITL 调出的 KP=0.006 导致 TRACK 阶段 PID 严重振荡，ey peak-to-peak ~500px，周期 2-3s。

**修复**:
- KP_H: 0.006 → 0.0015 (降至 1/4)
- KI_H: 0.00005 → 0.0 (纯 PD, 去 I 增益防积分 windup)
- KD_H: 0.002 → 0.001
- MAX_HORIZ_VEL: 1.0 → 0.3 m/s

**验证**: SITL (Iris 1.5kg) TRACK 13.8s 收敛，零振荡。纯 PD 在 HOVER 阶段有轻微稳态漂移 (ex 0→83px 慢周期)，不影响投放。

**诊断增强**: TRACK/DESCEND/HOVER/DROP 日志新增 bbox、检测年龄、姿态角、稳定器进度字段。

备份: `drop.py.bak.20260802_*`, `drop_wide.py.bak.20260802_*`

### 15. SITL 物理参数匹配工具 (2026-08-02)

创建 `~/SITL/scripts/set_drone_params.sh` 用于调整 Gazebo Iris 模型物理参数以匹配实机。

```bash
bash ~/SITL/scripts/set_drone_params.sh mass=5.3 prop_inch=15 wheelbase_mm=550 --list
```

**结论**: 仅改 Gazebo 物理不够 — PX4 飞控内环增益需同步重调。视觉伺服外环 PID 对物理差异不敏感，SITL 用默认 Iris 物理即可。

### 16. PID 抽帧控制 — 抑制 YOLO bbox jitter 引发的高频振动 (2026-08-03)

**问题**: 实机飞行中 YOLO 检测框存在帧间 jitter (1-3px)，20Hz PID 每次更新都触发速度修正。5.3kg 无人机物理响应时间 (~0.3-0.5s) 远大于 0.05s 控制周期，高频修正激起谐振。

**修复**:
- 新增抽帧控制模块常量: PID_DECIMATION_TRACK=4 (5Hz), PID_DECIMATION_DESCEND=5 (4Hz), PID_DECIMATION_HOVER=5 (4Hz)
- dt 同步缩放: `dt = decimation * 0.05`，保证导数项/积分项计算正确
- 抽帧间隙重发上次速度，维持 PX4 offboard 连接 (setpoint >2Hz)
- SITL (`drop_opt_mission.py`) 预留框架但默认不抽 (decimation=1, 20Hz)，因 GT 投影无 jitter

**验证**: 2026-08-03 两次实飞测试 (气球目标)，pid_hz 确认生效。HOVER 振荡根因为气球被旋翼下洗气流吹动 (测试替身问题)，最终目标为静止桶不需修改。

备份: `drop_opt_mission.py.bak.20260803_decimation`

### 17. FORWARD `_pub_pos` 位置控制方向错误 → 回退 `_pub_vel` 速度控制 (2026-08-07)

**问题**: v3 将 FORWARD 从 `_pub_vel`(速度控制) 改为 `_pub_pos`(位置控制) 后，实机试飞发现飞行方向偏离 locked heading0。

**根因**: `_pub_pos` 交由 PX4 MPC 自行导航到目标坐标，MPC 路径规划可能走曲线或产生 crab angle，不受代码直接控制。

**修复 (v4)**:
- FORWARD 回退 `_pub_vel`: `vx,vy = speed*cos_h0, speed*sin_h0`，速度向量+yaw 都用 heading0，三者严格一致，数学上不可能侧偏
- 仅修改 FORWARD 阶段，其余 v3 改进 (GROUND_NED_Z, PID height_scale, HOVER_SEARCH `_pub_pos`, DESCEND 直达 DROP, 抽帧控制) 全部保留
- 影响: `drop.py` + `drop_opt_mission.py` (SITL 同步)

**SITL 验证**: heading0=92°, 沿 y+ 方向直线飞行, 投放精度 0.26m, 全流程通过

备份: `drop.py.bak.20260807_*_v3_to_v2`

## Build

```bash
# Source ROS 2 (required before any colcon or ros2 command)
source /opt/ros/humble/setup.bash

# Build all packages
cd /home/nvidia/fly
colcon build

# Build only the fly package (after Python changes)
colcon build --packages-select fly

# After build, source the workspace overlay
source /home/nvidia/fly/install/setup.bash
```

没有 `--symlink-install`。每次 Python 修改后需 `colcon build --packages-select fly`。

## Launch Procedures

### 一键启动 (推荐)

```bash
cd ~/fly
./launch_drop.sh           # 前台: DDS → YOLO → Drop
./launch_drop.sh --log     # 保存日志到 ~/fly/log/
```

脚本自动处理: 环境 source、PYTHONPATH、sg dialout、预检、进程监控、Ctrl+C 清理。

### DDS Agent (终端 1)

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
# 或 (如 dialout 未生效):
sg dialout -c "MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600"
```

### YOLO 检测节点 (终端 2) — 需要 conda yolov8 环境

```bash
source /opt/ros/humble/setup.bash
source ~/fly/install/setup.bash
conda activate yolov8
export PYTHONPATH="/home/nvidia/miniforge3/envs/yolov8/lib/python3.10/site-packages:$PYTHONPATH"
ros2 run fly yolo
```

**PYTHONPATH 是必须的。** 系统 `/usr/local/lib/python3.10/dist-packages/` 中的 PyTorch/ultralytics 是 NumPy 1.x 编译的，在 conda NumPy 2.2.6 下会崩溃。

### 地面测试任务节点 (终端 3) — 系统 Python 即可

```bash
source /opt/ros/humble/setup.bash
source ~/fly/install/setup.bash
ros2 run fly test
```

### 投放任务节点 (终端 4) — 系统 Python 即可

```bash
source /opt/ros/humble/setup.bash
source ~/fly/install/setup.bash
ros2 run fly drop
```

**注意**: 配合 `yolo` 使用，会控制无人机起飞、飞行和投放。**仅实飞使用，不可用于无桨地面测试。**

### 调试用 DDS 监听

```bash
source ~/fly/install/setup.bash
ros2 run fly dds_listener
```

### 关闭可视化

```bash
ros2 run fly yolo --ros-args -p visualize:=false
```

## Architecture

colcon workspace。两个包:

- **`fly`** (`src/fly`) — ament_python, 所有任务节点
- **`px4_msgs`** (`src/px4_msgs`) — PX4 消息定义, 来自 PX4-Autopilot 1.17.0 (`MESSAGE_VERSION = 1`)

### Critical PX4 topic name convention

此板 PX4 固件发布在 **带版本后缀** 的话题名上:

```
/fmu/out/vehicle_status_v1        (不是 /fmu/out/vehicle_status)
/fmu/out/vehicle_local_position_v1
```

`_v1` 后缀是必须的。发布话题 (`/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_command`, `/fmu/in/manual_control_input`) 无后缀。

### drop.py 用 VehicleControlMode 替代 VehicleStatus

因为 vehicle_status_v1 DDS type checksum 不匹配，`drop.py` 的初始等待、解锁判断、模式切换全部使用 `VehicleControlMode` (`/fmu/out/vehicle_control_mode`)，不用 `VehicleStatus`。

## 调试技巧

```bash
# 查看某个话题是否有数据
ros2 topic hz /fmu/out/vehicle_status_v1

# 查看话题 QoS 详情（排查 DDS 匹配问题）
ros2 topic info /fmu/out/vehicle_status_v1 -v

# 查看话题内容（注意 QoS 匹配）
ros2 topic echo --qos-reliability best_effort /fmu/out/vehicle_control_mode --once

# 查看谁在占用摄像头
fuser /dev/video0

# 检查某个话题的发布者/订阅者
ros2 topic info /fmu/out/vehicle_status_v1
```

## 当前备份文件

| 文件 | 备份 | 说明 |
|------|------|------|
| `yolo.py` | `yolo.py.bak.20260723_164219` | 修复摄像头分辨率前的版本 |
| `yolo.py` | `yolo.py.bak.20260727_230517` | visualize 改为 True 前的版本 |
| `yolo.py` | `yolo.py.bak.20260731_optimize` | 性能优化前的版本 (2026-07-31) |
| `test.py` | `test.py.bak.20260723_170523` | 修复 DDS/xy_valid 前的版本 |
| `drop.py` | `drop.py.bak.20260727_230702` | 当前版本备份 (2026-07-27) |
| `drop.py` | `drop.py.bak.20260805_*` | **v2 迁移前备份 (2026-08-05)** |
| `path.py` | `path.py.bak.20260731_dds` | VehicleControlMode + xy_valid 修复前 |
| `dj.py` | `dj.py.bak.20260731_dds` | VehicleControlMode 修复前 |
| `offboard.py` | `offboard.py.bak.20260731_dds` | VehicleControlMode 修复前 |
| `dds_listener.py` | `dds_listener.py.bak.20260731` | 重写前的版本 |
| `b.py` | `b.py.bak.20260814_224912` | **2026-08-14 实飞版** (回退目标, 含 HEADING=54°/±0.05/延迟10s) |
| `c.py` | `c.py.bak.20260815_195445_scout_return_rework` | **2026-08-15 晚侦察防过冲+判稳+限速改动前** (实飞反馈修复) |
| `d.py` | 无 (源文件 c.py 未动) | 2026-08-15 新建, 复制自 c.py 后改造 |
| `d.py` | `d.py.bak.20260827_merge_c` | **2026-08-27 合并 c 晚间改动前** (侦察 7 子阶段/RETURN 判稳/LAND 限速/DROP_AFTER_DELAY 15) |
| `balloon.engine` | `/home/nvidia/balloon_640.engine` | 640 引擎备份 |

### yolo_v8 独立测试

```bash
cd ~/yolo_v8
./run.sh                    # 相当于: PYTHONPATH=... python3 camera.py
./run.sh --model other.engine --conf 0.5
```

直接编辑 `camera.py` 顶部 `CFG_*` 变量来换模型/参数 (不需要命令行)。

## 回滚所有修改

```bash
# yolo.py 回滚到最初版本
cp /home/nvidia/fly/src/fly/fly/yolo.py.bak.20260723_164219 /home/nvidia/fly/src/fly/fly/yolo.py

# test.py 回滚到最初版本
cp /home/nvidia/fly/src/fly/fly/test.py.bak.20260723_170523 /home/nvidia/fly/src/fly/fly/test.py

# drop.py 回滚到当前备份
cp /home/nvidia/fly/src/fly/fly/drop.py.bak.20260727_230702 /home/nvidia/fly/src/fly/fly/drop.py

cd ~/fly && colcon build --packages-select fly
```
