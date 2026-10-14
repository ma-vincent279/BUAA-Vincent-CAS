#!/usr/bin/env python3
# ================================================================
# 2026-09-24 文档同步: CLAUDE.md (f 节点 + 9-24 复盘 + 已知问题) + docs/project_log.md 置顶条目
#   用法: python3 scripts/update_docs_f_20260924.py
# ================================================================
import os
import re
import shutil
import time

ROOT = '/home/vincent/SITL'
STAMP = time.strftime('%Y%m%d_%H%M%S')

# ---------------- CLAUDE.md ----------------
SECT_2024 = '''最新开发迭代 (2026-09-24)：
- **项目 e 第二次实飞: 两发都中 (用户现场确认), 释放相位误差 2.60/1.78cm** (日志 `9.24/e_mission_20260924_173608.log` + `9.24/log_425_2026-9-24-17-39-10.ulg`, 双链路逐点对齐; 节点 NED 与 ULog `vehicle_local_position` 同系, `vehicle_local_position_setpoint` 与节点 `target` 精确一致 ⇒ 反投影→锚点混合→bias→两轴补偿→`_pub_pos` 全链路可复原). **实况**: 两次都是 **`触发方式=收敛即投`** (第 7.2s / 第 3.5s), 释放残差 **7px=2.60cm / 5px=1.78cm** (DROP 段残差序列 57→41→46→51→36→29→**7px**, 期间最差一帧等效地面 **32cm** ⇒ 门限是"飞机穿过瞄准点那一刻"才放行的), 高度 1.32/1.22m, vxy 0.076/0.079m/s. **落点推算**: 弹道漂移 (vxy×0.48s) 前−1.4/右−3.4 与 前−0.2/右+3.7cm; 加上 9-23 标定的两轴补偿 ⇒ 预测落点偏差 **5.3cm / 5.4cm**, 与"两发都中"一致 ⇒ **9-23 用两次脱靶数据反解的两轴瞄准补偿首次在实机被独立验证 (S 不需重标)**. **投放脉冲干净**: `actuator_outputs[4]` 2000µs 保持 **1.099s** / 1000µs 保持 **1.001s**, 无 1500µs 掉线; 载荷脱钩有独立证据 (第 2 投推力台阶 **−5.6%**、电流 −7A、竖直 +9.3cm/0.5s; 两枚载荷合计占整机 7.7%). **全流程正常**: `End reason: normal`, 160.8s, 侦察 7 子阶段 13.3/1.3/10.0/2.0/5.1/2.0/5.1s, 返航 20.4s(≈2.9m/s)+判稳 5.5s(漂移0.28m), 着陆距 home **0.10m**, 无 WARN/ERROR. **仍存在的物理事实**: DROP 保持段飞机水平漂移峰峰 **15.9/23.6cm** 与 9.1/13.8cm、速度均值 0.105/0.083m/s ⇒ "飞机停不下来"未变, 只是判据把它兜住了. **新发现问题**: ① `vehicle_to_anchor`(0.201/0.170m) **不是脱靶量**(它算"飞机↔DROP 入口冻结锚点", 含飞机自身漂移) ⇒ 复盘别用它; ② **N1 (RC 派生样本抢占) 真的出现过一次** (t=875.94, 脉冲开始后 0.10s, `aux5=0/roll=−0.00957` 签名与 9-19 完全相同), 本次没夹断脉冲属运气; ③ **e 的判据最坏情况会脱靶**: 10cm 门限 + 3~7.6cm 弹道漂移 > 桶半径 12.5cm.
- **新建项目 f = e 的完整副本 + "方案 B" 两级释放判据 (用户指示: 新建 f, e 不动)** — 生成脚本 `scripts/new_project_f_20260924.py` (从 e.py 复制后只打 12 处锚点补丁, 断言唯一命中 + 备份; 差异审计 SITL 112 行 / fly 119 行); 新增 `src/drone_drl/drone_drl/f.py`、`fly/src/fly/fly/f.py`、`worlds/f.world`、`scripts/test_f.sh`、`fly/f.sh`、两份 `setup.py` 的 `f` 入口. **改动只有"何时投"**: `DROP_ARRIVE_M` 0.10→**0.05** (Tier1), `DROP_CONVERGE_EARLY` 1.0→**0.5s**, 新增**低谷判定** `conv_m <= 本段最小值+2cm` (最小值下限夹 2cm 防单帧噪声) 与**准静止门控** `vxy<0.12m/s`, 并把 e 的旧判据降级为 **Tier2 保底** (10cm/1.0s, 但须等过 `DROP_PREFER_WINDOW=10s`), `DROP_AFTER_DELAY=35s` 兜底不变 ⇒ **形态上"永不比 e 更差"**. **验收**: ① 真代码路径离线测试 `scripts/test_f_gate_logic_20260924.py` (抽出两份 f.py 的判据块 exec) 5 场景全过 (低谷穿越→Tier1@3.75s / 地板 6~9cm→Tier2@11.25s / 地板 15~25cm→不触发退兜底 / 单帧 0.5cm 噪声不误投 / vxy=0.20 被挡到 0.08 才放行); ② 历史实飞日志回放 `scripts/replay_f_drop_gate_20260924.py` (8 份日志 13 投): **Tier1 10 投 / Tier2 1 投 / 段内不触发 2 投** (后两个是 pitch 符号 bug 期地板 19~23cm 的轮次), **释放残差中位 9.5cm → 4.0cm**; 9-23 那两次脱靶在实际日志里是 9.2/10.5cm 兜底, 用 f 会在 **3.27/3.15cm** 释放; 9-24 两次仍会在同一时刻附近触发 (2.62/4.02cm). ③ SITL 复验见下条.

'''

CLAUDE_SEARCH = '最新开发迭代 (2026-09-23)：'

KNOWN_ADD = '''- **`vehicle_to_anchor` 不是脱靶量 (2026-09-24 复盘澄清)**：`>>> DROP` 行与 MISSION SUMMARY 里的 `vehicle_to_anchor` 算的是"飞机 ↔ `_current_target_ned` 锚点"的距离, 而锚点在进入 DROP 时冻结、飞机自己还在漂 (9-24 第 1 投 7.2s 内水平漂移峰峰 23.6cm), 所以 9-24 该值 0.201/0.170m 远大于真实释放相位误差 (`last_err` 7/5px = 2.60/1.78cm)。**复盘判精度请看 `last_err`/`conv_m` 与 `等效地面=`, 不要用 `vehicle_to_anchor`** (9-23 分析曾把它当"飞机偏移"用, 会高估)。
- **N1 (投放脉冲被 RC 派生样本抢占) 在 9-24 复飞中又出现了一次, 只是没夹断脉冲 (2026-09-24)**：`9.24/log_425_2026-9-24-17-39-10.ulg` 的 `manual_control_setpoint` 在 **t=875.94** (2000µs 脉冲开始后 0.10s) 出现 **PX4 RC 派生样本** (`aux5=0.000, roll=−0.00957, yaw=+0.00238`, 与 9-19 抓到的签名完全相同), 而节点自己的样本是 `aux5=+0.833` 且三轴全零; 本次 10Hz `actuator_outputs[4]` **未抓到** 1500µs 掉线 (脉冲 875.841→876.940 完整 1.099s) ⇒ **风险仍在**: 若 Jetson 恰在释放后 0.1~0.3s 停顿, 脉冲会被拉回锁定。彻底修法仍是投放改 PX4 一等公民通道 (`MAV_CMD_DO_SET_ACTUATOR` / `PWM_AUX_FUNC`+`MAV_CMD_DO_SET_SERVO`) 或关 RC manual 输入 (`COM_RC_IN_MODE`) + 回读 PWM 校验。
- **e 的释放判据"最坏情况会脱靶" (2026-09-24 量化, 已在项目 f 修正)**：`DROP_ARRIVE_M=0.10` ⇒ 最坏释放残差 10cm + 弹道漂移 3~7.6cm (`vxy`×0.5s) > 桶半径 12.5cm; 9-24 两次命中 (预测 5.3/5.4cm) 是"残差小 + 漂移方向恰好抵消"的结果, **不是设计保证**。项目 f 用两级判据 (Tier1 5cm+低谷+准静止 / Tier2 10cm 保底 / 35s 兜底) 解决, 见 f 章节; **e 本身按用户指示不动**。
- **⚠️ 文档漂移 (2026-09-24 发现并就地修正)**：CLAUDE.md 的 d/e 参数块此前仍写 `DROP_HEIGHT=1.5`, 而四份代码自 2026-09-13 起已是 **1.2m**; `scripts/test_e.sh` 头部注释也仍写"投放补偿全部去掉/25s 兜底/10px 判据"(实际为两轴补偿/35s/米制 10cm) —— **e 的脚本按"e 不动"原则未改, 仅在此记录**。

'''

KNOWN_ANCHOR = '### PX4 自定义模型'

ACTIVE_ADD = '''- **f v1** (`src/drone_drl/drone_drl/f.py`): **子项目 f** 基于 e (2026-09-24 新建, **e 一行未动**), 唯一改动 = **两级释放判据** (Tier1 `conv_m<5cm`+低谷`<=本段最小+2cm`+准静止`vxy<0.12` 确认 0.5s / Tier2 = e 旧判据 10cm 确认 1.0s 但须等过 10s / Tier3 35s 兜底), 其余与 e 完全一致。验收: `scripts/test_f_gate_logic_20260924.py` (真代码路径 5 场景全过) + `scripts/replay_f_drop_gate_20260924.py` (8 份历史实飞日志 13 投回放: Tier1 10/Tier2 1/兜底 2, 释放残差中位 9.5→4.0cm)。
- **fly/f v1** (`fly/src/fly/fly/f.py`): 实机版 f, 与 SITL 版本判据块逐字一致 (HEADING_DEG=180.0, PID 抽帧 4Hz, `_v1` 话题), 实机启动 `bash ~/fly/f.sh`。
- 启动 f: `bash ~/SITL/scripts/test_f.sh`
'''

ACTIVE_ANCHOR = '- 启动 drop_opt: `bash ~/SITL/scripts/test_drop.sh`'

F_SECTION = '''
### f — 子项目 f v1 (基于 e, 两级释放判据, 2026-09-24)

**来历**: 用户指示"新建项目 f, e 不要动" ⇒ `f.py` = `e.py` **逐字节复制** (只替换 节点名 `f_mission` / 类名 `FMission` / 入口名 `f` / `MISSION_REVISION`), 唯一实质改动是**释放判据**; 生成脚本 `scripts/new_project_f_20260924.py` (12 处锚点断言 + 自动备份 + 差异审计: SITL 与 e 差 112 行, fly 差 119 行).

**动机 (9-24 实飞 ULog 复盘)**: e 的 `DROP_ARRIVE_M=0.10` 允许的最坏释放残差 10cm, 加上落地前弹道漂移 3~7.6cm (`vxy`×0.5s) 就超过桶半径 12.5cm ⇒ **最坏情况会脱靶**; 9-24 两次命中 (预测落点偏差 5.3/5.4cm) 是残差小 + 漂移方向恰好抵消的运气。判据的另一半问题是**释放相位**: 9-23 的残差在 DROP 段内明明出现过 2.3~2.8cm, 却因当时判据 (10px≡3.4cm + 1.5s) 凑不满而拖到 25s 兜底, 投在 9.2/10.3cm 上 ⇒ **"投在低谷"比"门限数值"更关键**。

**两级释放判据 (只改"何时投", 不碰释放通道/脉冲形态/两轴瞄准补偿)**:

| 档 | 条件 | 确认时间 | 作用 |
|---|---|---|---|
| **Tier1 优选** | `conv_m < 5cm` **且** 低谷 (`conv_m <= 本段最小值 + 2cm`, 最小值下限夹 2cm 防单帧噪声) **且** 准静止 (`vxy < 0.12m/s`) | **0.5s** (历史低位窗口只有 1~3s) | 把释放钉在"这一段最好的相位", 最坏残差 5cm |
| **Tier2 保底** | `conv_m < 10cm` (= e 的旧判据), 但须等过 `DROP_PREFER_WINDOW=10s` 优先窗口 | 1.0s (同 e) | 地板够不到 5cm 时仍能投, **永不比 e 更差** |
| **Tier3 兜底** | `DROP_AFTER_DELAY = 35s` | — | 防挂死 (未改) |

档位变化会**重启确认计时** (防混档提前放行); 释放瞬间的水平速度上限让弹道漂移封顶 ~6cm。

**当前参数 (与 e 的差异只有这些)**:
```
DROP_ARRIVE_M       = 0.05   # Tier1 门限 (e=0.10)
DROP_ARRIVE_M_LATE  = 0.10   # Tier2 = e 旧判据
DROP_CONVERGE_EARLY = 0.5    # Tier1 确认 (e=1.0)
DROP_CONVERGE_LATE  = 1.0    # Tier2 确认
DROP_CONV_MIN_BAND  = 0.02   # 低谷带
DROP_CONV_MIN_FLOOR = 0.02   # 低谷最小值下限夹 (防噪声)
DROP_ARRIVE_VMAX    = 0.12   # 准静止门控 (m/s)
DROP_PREFER_WINDOW  = 10.0   # Tier1 优先窗口 (s)
DROP_AFTER_DELAY    = 35.0   # 兜底 (未改)
# 其余 (DROP_HEIGHT 1.2 / FORWARD_DIST 34 / SCOUT_* / 两轴补偿 Y=+0.048 XF=+0.040 / PID / 相机 640x360 85°/54°)
# 与 e 完全一致; HEADING_DEG: SITL=92.0 / fly=180.0
```

**日志新增**: `DROP-SERVO` 1Hz 行加 `收敛计时=Xs/T{档}` 与 `段内最小=XX.Xcm`, 门限打印改 `门限T1=5cm/T2=10cm`; `>>> DROP #n` 行与 MISSION SUMMARY 加 `tier=T{1|2} conv_min=XX.Xcm`, `触发方式=收敛即投(Tier1 5cm低谷准静止 段内最小=3.3cm)` / `(Tier2 10cm保底 ...)` / `X.Xs/35s 兜底`。

**验收 (三层)**:
1. 真代码路径离线测试 `scripts/test_f_gate_logic_20260924.py` — 把两份 f.py 的判据块原文 exec 成函数, 5 场景全过: 低谷穿越→Tier1@3.75s / 地板 6~9cm→Tier2@11.25s / 地板 15~25cm→不触发退兜底 / 单帧 0.5cm 噪声不误投 / `vxy=0.20` 被挡到 0.08 才放行。
2. 历史实飞日志回放 `scripts/replay_f_drop_gate_20260924.py` (8 份日志 / 13 投): **Tier1 10 投, Tier2 1 投, 段内不触发 2 投** (两个是 pitch 符号 bug 期地板 19~23cm 的轮次) ⇒ **释放残差中位 9.5cm → 4.0cm**; 9-23 两次脱靶 (实际 9.2/10.5cm 兜底) 用 f 会在 **3.27/3.15cm** 释放; 9-24 两次仍在同一时刻附近触发 (2.62/4.02cm)。
3. SITL 复验: `bash ~/SITL/scripts/test_f.sh` (见 docs/project_log.md 当次记录)。

**环境/启动**: SITL `bash ~/SITL/scripts/test_f.sh` (world=`worlds/f.world`, 日志 `/tmp/f_output.log`, 裁判 `/tmp/f_judge.log`); 实机 `bash ~/fly/f.sh` (Jetson 编译 + `ros2 run fly f`)。
'''

PARAM_FIX = [
    ('DROP_HEIGHT=1.5  FORWARD_DIST=34.0  SCOUT_DIST=59.5\n',
     'DROP_HEIGHT=1.2  FORWARD_DIST=34.0  SCOUT_DIST=59.5   # 2026-09-13 起 1.5→1.2 (文档同步修正)\n'),
    ('DROP_HEIGHT=1.5  FORWARD_DIST=34.0  SCOUT_DIST=59.5  SCOUT_EDGE_DIST=55.0 (fly=57.0)\n',
     'DROP_HEIGHT=1.2  FORWARD_DIST=34.0  SCOUT_DIST=59.5  SCOUT_EDGE_DIST=55.0 (fly=57.0)   # 2026-09-13 起 1.5→1.2\n'),
]

# ---------------- docs/project_log.md ----------------
LOG_ENTRY = '''## 2026-09-24 — 项目 e 第二次实飞 (两发都中) 复盘 + 新建项目 f (两级释放判据)

> 数据: `9.24/e_mission_20260924_173608.log` (ROS, 1072 行) + `9.24/log_425_2026-9-24-17-39-10.ulg` (PX4, 161.8s, 115 话题). 分析工具: 本文件同目录脚本 `scripts/replay_f_drop_gate_20260924.py` / `scripts/test_f_gate_logic_20260924.py`; ULog 侧直接 `pyulog` 解析 (时间轴对齐: 节点 NED 与 ULog `vehicle_local_position` 同系, 释放瞬间 `(-36.960,-0.995,37.654)` ↔ `(-36.959,-0.990,37.634)`; ULog `vehicle_local_position_setpoint` `(-36.972,-1.035)` 与节点 `target` 精确一致 ⇒ 反投影→锚点混合→bias→两轴补偿→`_pub_pos` 全链路可复原).

### 一、实况: 两次都"收敛即投", 两发都中

| 项目 | 第 1 投 (B1, 2000µs) | 第 2 投 (B2, 1000µs) |
|---|---|---|
| 触发方式 | **收敛即投** (第 7.2s) | **收敛即投** (第 3.5s) |
| 释放残差 | 7px = **2.60cm** | 5px = **1.78cm** |
| 释放高度 / vxy | 1.32m / 0.076m/s | 1.22m / 0.079m/s |
| 弹道漂移 (vxy×0.48s) | 前 −1.4 / 右 −3.4cm | 前 −0.2 / 右 +3.7cm |
| 预测落点偏差 | **5.3cm** | **5.4cm** |
| 实测 | **命中 (用户确认)** | **命中 (用户确认)** |

- 与 9-23 对比: 触发方式 **25s 兜底 ×2 → 收敛即投 ×2**, 释放残差 **9.2/10.3cm → 2.60/1.78cm**, 落点 **15~16cm 脱靶 → 命中** ⇒ 9-23 当天的两项改动 (两轴瞄准补偿 + 米制判据) 全部生效。
- **两轴补偿首次被实机独立验证**: 9-23 由两次脱靶落点反解的固定偏置 S=(右+4.8, 前+4.2)cm 在本次两次命中中得到支持 ⇒ **S 不需重标** (此前只有 SITL 复验, 而 SITL 无机械偏置、分数带 6.25cm 固定罚分, 证明不了)。
- **门限是在"飞机穿过瞄准点那一刻"放行的**: 第 1 投 DROP 段残差序列 57→41→46→51→36→29→**7px**, 期间最差一帧等效地面 **32cm** (飞机漂到最远处) ⇒ 命中不是"飞得准", 而是"判据等到了好相位"。
- **全流程正常**: `End reason: normal`, 160.8s; 侦察 7 子阶段 13.3/1.3/10.0/2.0/5.1/2.0/5.1s; 返航 20.4s (≈2.9m/s) + 判稳 5.5s (漂移 0.28m); 着陆距 home **0.10m**; 无 WARN/ERROR。

### 二、链路体检 (ULog 独立复核)

- **投放脉冲干净**: `actuator_outputs[4]` 2000µs 保持 **1.099s** (875.841→876.940), 1000µs 保持 **1.001s** (896.240→897.241), 中间无 1500µs 掉线。
- **载荷脱钩有独立证据**: 第 2 投保持窗口推力 −0.7468→−0.7052 = **−5.6%**、电流 −7A、竖直方向 +9.3cm/0.5s; 两枚载荷合计占整机 **7.7%** (5m 平飞窗口: 投前 −0.7720 → 投后 −0.7123); 第 1 投仅 −0.2% (本次换轻水, 属配置差异)。
- **竖直方向有 ~±10cm / 0.5Hz 极限环** (释放高度 1.21~1.32m 波动), 与水平漂移同源 (MPC 悬停), 非代码缺陷。
- 实机 160.8s vs 同版本 SITL 383.9s ⇒ SITL 的 MPC 明显更迟钝, **SITL 分数依旧不能代表实机收益**。

### 三、新发现的三个问题

1. **`vehicle_to_anchor` 不是脱靶量**: 该值 (0.201/0.170m) 算的是"飞机 ↔ 进入 DROP 时冻结的锚点", 含飞机自身漂移 (第 1 投 7.2s 内漂 23.6cm), 真实释放相位误差是 `last_err` 2.60/1.78cm。9-23 的分析曾把它当"飞机偏移"用 ⇒ 会高估误差。
2. **N1 (RC 派生样本抢占投放脉冲) 又出现一次**: t=**875.94** (脉冲开始后 0.10s) 的 `manual_control_setpoint` 是 PX4 RC 派生样本 (`aux5=0.000, roll=−0.00957, yaw=+0.00238`, 签名与 9-19 完全相同), 本次 10Hz PWM 未抓到掉线 ⇒ **没夹断属运气**, 风险仍在 (Jetson 若在释放后 0.1~0.3s 停顿即被拉回锁定)。
3. **e 的判据最坏情况会脱靶**: 允许残差 10cm + 弹道漂移 3~7.6cm > 桶半径 12.5cm。本次命中是"残差小 + 漂移方向恰好抵消"。

### 四、新建项目 f (= e 的完整副本 + 只改"何时投")

- **生成**: `scripts/new_project_f_20260924.py` (从 e.py 复制, 12 处锚点断言 + 自动备份 + 差异审计: SITL 112 行 / fly 119 行); 新增 `src/drone_drl/drone_drl/f.py`、`fly/src/fly/fly/f.py`、`worlds/f.world`、`scripts/test_f.sh`、`fly/f.sh`、两份 `setup.py` 的 `f` 入口。**e 一行未动** (md5 仍 `f2bb21d1` / `0daa0712`)。node/类名 `f_mission` / `FMission`, `MISSION_REVISION=competition-water-20260924-r1-f`。
- **两级释放判据 (方案 B)**: `DROP_ARRIVE_M` 0.10→**0.05**; `DROP_CONVERGE_EARLY` 1.0→**0.5s**; 新增**低谷判定** `conv_m <= 本段最小值+2cm` (最小值下限夹 2cm) 与**准静止门控** `vxy<0.12m/s`; e 的旧判据降级为 **Tier2 保底** (`DROP_ARRIVE_M_LATE=0.10` / 1.0s, 须等过 `DROP_PREFER_WINDOW=10s`); `DROP_AFTER_DELAY=35s` 兜底不变 ⇒ 形态上 **"永不比 e 更差"**。日志新增 `收敛计时=Xs/T{档}`、`段内最小=XX.Xcm`、`tier/conv_min` 字段。
- **验收 ① 真代码路径离线测试** `scripts/test_f_gate_logic_20260924.py` (把两份 f.py 的判据块原文 exec 成函数, 两份逐字一致): 5 场景全过 — 低谷穿越→Tier1@3.75s; 地板 6~9cm→Tier2@11.25s; 地板 15~25cm→不触发 (退兜底); 单帧 0.5cm 噪声不误投 (下限夹生效); `vxy=0.20` 被挡到 0.08 才放行。
- **验收 ② 历史实飞日志回放** `scripts/replay_f_drop_gate_20260924.py` (8 份日志 / 13 投, 1Hz 样本近似, 同时给"1 样本"与"连续 2 样本"两种口径): **Tier1 10 投 / Tier2 1 投 / 段内不触发 2 投** (两个是 pitch 符号 bug 期地板 19~23cm 的轮次, 与 e 一样退兜底), **释放残差中位 9.5cm → 4.0cm**; 细看: 9-23 两次 (实际 9.2/10.5cm 兜底) → **3.27/3.15cm**; 9-24 两次 → **2.62/4.02cm** (同一时刻附近触发); 9-19 两次 → 4.69/3.38cm (实际 3.4/3.4cm); 9-15 第 1 投 (地板 5.6cm, Tier1 够不到) → Tier2@10s 5.64cm (实际 15.8cm)。
- **验收 ③ SITL 复验**: 见本轮 SITL 记录 (`/tmp/f_output.log` + `/tmp/f_judge.log`)。

### 五、文档同步

- CLAUDE.md: 包表新增 `f` 行 + Gazebo 世界新增 `worlds/f.world` + 新增"### f — 子项目 f v1"架构/参数章节 + "当前活跃版本"新增 f/fly-f + 已知问题新增 3 条 (vehicle_to_anchor 语义 / N1 复现 / e 判据最坏情况) + 本次复盘摘要。
- **就地修正文档漂移**: d/e 参数块的 `DROP_HEIGHT` 1.5 → **1.2** (代码自 2026-09-13 起已是 1.2); `scripts/test_e.sh` 头部注释的过时描述 (补偿/兜底/判据) **按"e 不动"原则未改, 仅在已知问题中记录**。

---

'''

def patch(path, subs, must=True):
    src = open(os.path.join(ROOT, path), encoding='utf-8').read()
    bak = os.path.join(ROOT, path) + f'.bak.{STAMP}'
    shutil.copy2(os.path.join(ROOT, path), bak)
    for old, new, tag in subs:
        n = src.count(old)
        if must:
            assert n == 1, f'{path}: 锚点 {tag} 命中 {n} 次 (期望 1)'
        if n == 0:
            print(f'   ! {path}: {tag} 未命中, 跳过')
            continue
        src = src.replace(old, new, 1)
        print(f'   {path}: {tag} OK')
    open(os.path.join(ROOT, path), 'w', encoding='utf-8').write(src)
    print(f'   {path}: 已写回 (备份 {os.path.basename(bak)})')


def main():
    print('=== 1. CLAUDE.md ===')
    subs = [
        (CLAUDE_SEARCH, SECT_2024 + CLAUDE_SEARCH, '阶段性总结: 插入 2026-09-24 迭代'),
        (KNOWN_ANCHOR, KNOWN_ADD + KNOWN_ANCHOR, '已知问题: 追加 3 条'),
        (ACTIVE_ANCHOR, ACTIVE_ADD + ACTIVE_ANCHOR, '当前活跃版本: 追加 f'),
    ]
    for old, new in PARAM_FIX:
        subs.append((old, new, f'参数块 DROP_HEIGHT 修正 ({old[:40]}…)'))
    patch('CLAUDE.md', subs, must=False)

    # 文末追加 f 章节 (不能用 replace('```') —— 会命中文件开头的代码块)
    p = os.path.join(ROOT, 'CLAUDE.md')
    src = open(p, encoding='utf-8').read()
    if '### f — 子项目 f v1' not in src:
        src = src.rstrip('\n') + '\n' + F_SECTION
        open(p, 'w', encoding='utf-8').write(src)
        print('   CLAUDE.md: 文末追加 f 章节 OK')
    else:
        print('   CLAUDE.md: f 章节已存在, 跳过')

    print('\n=== 2. docs/project_log.md ===')
    p = os.path.join(ROOT, 'docs/project_log.md')
    src = open(p, encoding='utf-8').read()
    shutil.copy2(p, p + f'.bak.{STAMP}')
    anchor = '## 2026-09-23 — 项目 e **首次实飞**'
    assert src.count(anchor) == 1, 'project_log 锚点命中异常'
    src = src.replace(anchor, LOG_ENTRY + anchor, 1)
    open(p, 'w', encoding='utf-8').write(src)
    print(f'   docs/project_log.md: 置顶插入 2026-09-24 条目 (备份 project_log.md.bak.{STAMP})')

    print('\n=== 3. 校验 ===')
    c = open(os.path.join(ROOT, 'CLAUDE.md'), encoding='utf-8').read()
    assert '| `f` | `f.py`' in c and '### f — 子项目 f v1' in c and 'worlds/f.world' in c
    assert 'vehicle_to_anchor` 不是脱靶量' in c and 'N1 (投放脉冲被 RC 派生样本抢占) 在 9-24' in c
    assert 'DROP_HEIGHT=1.5' not in c, '仍有过时的 DROP_HEIGHT=1.5'
    print('   CLAUDE.md: f 行 / f 章节 / f.world / 3 条已知问题 / DROP_HEIGHT 修正 全部就位')
    l = open(os.path.join(ROOT, 'docs/project_log.md'), encoding='utf-8').read()
    assert '## 2026-09-24 — 项目 e 第二次实飞' in l and l.index('## 2026-09-24') < l.index('## 2026-09-23')
    print('   project_log.md: 2026-09-24 条目已置顶')
    print('\n完成')


if __name__ == '__main__':
    main()
