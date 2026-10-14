#!/usr/bin/env python3
# ================================================================
# 2026-09-27 文档同步: CLAUDE.md (g 节点 + 9-27 双航班复盘 + 已知问题) + docs/project_log.md 置顶
#   脚本会自己从 /tmp/g_out_round1.log + /tmp/g_judge_round1.log 抓 SITL 结论写进去
#   用法: python3 scripts/update_docs_g_20260927.py
# ================================================================
import os
import re
import shutil
import time

ROOT = '/home/vincent/SITL'
STAMP = time.strftime('%Y%m%d_%H%M%S')


def sitl_summary():
    """从 SITL 日志/裁判日志抓结论, 失败则返回占位文本"""
    out, jud = '/tmp/g_out_round1.log', '/tmp/g_judge_round1.log'
    txt = open(out, encoding='utf-8', errors='replace').read() if os.path.exists(out) else ''
    jtxt = open(jud, encoding='utf-8', errors='replace').read() if os.path.exists(jud) else ''
    if 'MISSION SUMMARY' not in txt:
        return ('（SITL 复验进行中/未跑，见 `bash scripts/test_g.sh`；`/tmp/g_output.log` + `/tmp/g_judge.log`）', '')
    end = re.search(r'End reason: (\S+)', txt)
    tot = re.search(r'Total time: ([\d.]+)s', txt)
    drops = re.findall(r'>>> DROP #(\d)!.*?触发方式=([^\n]+?)(?: last_err=(\d+)px\(ex=(-?\d+),ey=(-?\d+)@H=([\d.]+)m\))?\s*$',
                       txt, re.M)
    tr = re.findall(r'Drop #\d: .*?tier=(T\d) conv_min=([\d.]+)cm', txt)
    sco = re.findall(r'RESULT (B\d)\s*=+\n\s*距离: ([\d.]+)m, 得分: (\d+)', jtxt)
    tot_s = re.findall(r'total_score=(\d+)', jtxt)
    scout = re.search(r'Scout: completed \(([\d.]+)s', txt)
    land = re.search(r'Landing: .*?dist_from_home=([\d.]+)m', txt)
    s = (f"`End reason: {end.group(1) if end else '?'}`, 总时长 **{tot.group(1) if tot else '?'}s**")
    if tr:
        s += '; 两投 ' + ' / '.join(f'{t[0]}(段内最小 {t[1]}cm)' for t in tr)
    if drops:
        s += '; 释放行 ' + ' / '.join(f"#{d[0]} {d[1].strip()[:46]}…" for d in drops[:2])
    if sco:
        s += '; 裁判 ' + ' + '.join(f'{b} {d}m({p}分)' for b, d, p in sco) + \
             f" ⇒ **总分 {tot_s[-1]}/20**" if tot_s else ''
    if scout:
        s += f"; 侦察 {scout.group(1)}s"
    if land:
        s += f"; 着陆距 home {land.group(1)}m"
    return s, '（⚠ SITL 无挂载偏置 ⇒ 补偿让飞机故意偏离桶心 11.3/2.5cm, 分数被该伪影主导, 只证明链路与触发正常）'


SITL_TXT, SITL_CAVEAT = sitl_summary()

SECT = f'''最新开发迭代 (2026-09-27)：
- **9-27 双航班实飞 (e 一次 + f 一次, 同场地同一对桶): 找到"第 2 投总偏左"的真根因 —— 两瓶镜像挂载, 而 e/f 对两瓶用了同一个横向补偿 (符号错)**。数据 `9.27/e_mission_20260927_105447.log` + `log_427_…ulg` / `9.27/f_mission_20260927_110915.log` + `log_430_…ulg`, 双链路逐点对齐 (节点 NED 与 ULog 同系; 释放脉冲 e 1.000s/1.101s、f 1.002s/1.102s 干净; **四次投放窗口内 0 个 RC 派生样本 ⇒ N1 未触发**; 脱钩在命令后 +0.2~0.3s, 用推力/电流台阶定位 ⇒ **弹道漂移必须按脱钩时刻速度算**: 例 e#2 指令时 0.109 m/s、脱钩时 0.071). **用户现场真值 (首次落点, 不计弹跳)**: e 第 1 投偏右 5cm / 第 2 投偏左 12cm; f 第 1 投偏右 8cm / 第 2 投偏左 2~3cm. **关键几何**: 两瓶是**镜像挂载** —— 第 1 瓶(先投)在机体右侧, 释放点在相机轴 **右 +5.5cm**; 第 2 瓶在左侧, **左 −5.5cm**; 而代码 `PAYLOAD_OFFSET_BODY_Y_B1 = _B2 = +0.048` (两个都是"瞄准点左移") ⇒ **第 2 瓶方向是反的**. **误差预算闭合 (可测量三项)**: ①瞄准残差 (相机实测) 1.4~4.2cm ②弹道漂移 2~5cm ③载荷摆动 —— 用 50Hz `vehicle_torque_setpoint` 反推摆角 (姿态被飞控稳住, 只有力矩暴露外力): 摆角 0.78~1.33° ⇒ 摆动速度 1.4~5.7cm/s ⇒ **落点散布仅 0.7~2.6cm (可忽略; e#2 也不是最大)**; 三项合计 ≤6cm, 却出现一次 12cm 脱靶 ⇒ **缺口只能来自"瓶相对瞄准点的偏移没被补偿对"**. **回放验证**: 用 4 次实测落点反推所需补偿 = 现值 + 实测落点 ⇒ **第 1 瓶 +9.8~+12.8cm (均值 +11.3)**, **第 2 瓶 −7.2~+2.3cm (均值 −2.5, 必须反号!)**; 纵向用户确认"基本不偏" ⇒ `XF=+0.040` 正确不动. 台架值 +5.5cm 与飞行反推 ~+14cm 的差 (~+6~8cm) 是**动态项** (插销/摩擦的侧向冲量 + 相机光轴安装偏角的模型误差) ⇒ **必须以实飞落点标定, 不能只信台架**.
- **新建项目 g = f 的完整副本 + 两瓶横向补偿按 9-27 实飞落点重标 (用户指示: e/f 都不动)** — 生成脚本 `scripts/new_project_g_20260927.py` (从 f.py 复制, 只改 节点名/类名/入口名 + 两个常量; 与 f 的差异仅 **56 行**, 断言唯一命中 + 自动备份); 新增 `src/drone_drl/drone_drl/g.py`、`fly/src/fly/fly/g.py`、`worlds/g.world`、`scripts/test_g.sh`、`fly/g.sh`、两份 `setup.py` 的 `g` 入口. **改动**: `PAYLOAD_OFFSET_BODY_Y_B1` 0.048→**+0.113**, `PAYLOAD_OFFSET_BODY_Y_B2` +0.048→**−0.025 (反号)**, `XF` 保持 +0.040; 其余 (两级释放判据 Tier1/Tier2/Tier3、DROP 段锚点+反投影伺服、俯冲段、侦察 7 子阶段、RETURN 判稳、LAND 限速) 与 f 完全一致. **离线验收** `scripts/test_g_aim_comp_20260927.py` (三节全过): A 两份 g.py 常量逐位一致; B **真实代码路径** (exec 出厂 `PIDController` + `level_to_camera_pixel`, 含 pitch 3°/roll 2°) — 桶在补偿点 +11.3/−2.5cm 处 err **0.00px**; 同一桶位用旧常量 +4.8cm 时报 **+6.50cm / −7.30cm (= 需要的修正量)**; 桶放在旧符号侧时报 +5.00cm ⇒ **证明反号必要**; C **9-27 四次实测落点回放**: 落点 **+5 / −12 / +8 / −2.5cm → −1.5 / −4.7 / +1.5 / +4.8cm**, 最坏偏移 **12cm → 4.8cm** (桶口半径 12.5cm). **SITL 复验**: {SITL_TXT} {SITL_CAVEAT}

'''

KNOWN_ADD = '''- **两瓶镜像挂载 ⇒ 横向瞄准补偿必须按瓶分别设, 符号相反 (2026-09-27 实飞落点定性; e/f 的 `+0.048` 对第 2 瓶是错的)**：载荷是**一左一右**挂在相机两侧 —— 第 1 瓶(先投, PWM 2000µs)在机体右侧、释放点在相机轴 **右 +5.5cm**; 第 2 瓶(后投, PWM 1000µs)在左侧、**左 −5.5cm**。而 e/f 对两瓶都用 `PAYLOAD_OFFSET_BODY_Y = +0.048`(瞄准点左移) ⇒ **第 2 瓶的补偿方向反了**, 把落点往左推了约 10cm。9-27 两次实飞的落点证明: 第 1 瓶偏右 5/8cm、第 2 瓶偏左 12/2.5cm; 由此反推所需补偿 **第 1 瓶 ≈ +11.3cm, 第 2 瓶 ≈ −2.5cm** (差值 13.8cm ≈ 两瓶间距 11cm + 动态项)。**项目 g 已按此重标**; **e/f 未改 (用户指示)**。**台架几何 (5.5cm) 与飞行反推 (~14cm) 相差 ~6~8cm** ⇒ 存在动态右偏项 (插销/摩擦侧向冲量 + 相机光轴安装偏角的模型误差) ⇒ **标定必须以实飞落点为准**。
- **"释放相位"已经不是误差瓶颈 (2026-09-27 定量)**：9-27 四次投放的释放残差 1.4~4.2cm、弹道漂移 2~5cm、载荷摆动 (由 50Hz 力矩设定点反推) 0.7~2.6cm ⇒ **可测三项合计 ≤6cm**, 而出现了一次 12cm 脱靶 ⇒ 继续收紧 Tier1 门限/加平稳窗口的收益远小于**把两瓶的横向补偿标对**。**e#2 的脱靶就是补偿符号错的后果** (模型预测 −13.2cm vs 实测 −12cm, 误差仅 1.2cm)。

'''

F_SECTION = '''
### g — 子项目 g v1 (基于 f, 两瓶横向补偿按实飞落点重标, 2026-09-27)

**来历**: 用户指示"新建 g, e/f 都不动"。`g.py` = `f.py` **逐字节复制** (只替换 节点名 `g_mission` / 类名 `GMission` / 入口名 `g` / `MISSION_REVISION`), 与 f 的差异仅 **56 行**; 生成脚本 `scripts/new_project_g_20260927.py`。

**动机 (9-27 双航班实飞落点)**: 载荷**一左一右**挂载, 第 1 瓶释放点在相机轴 **右 +5.5cm**、第 2 瓶 **左 −5.5cm**; 而 f/e 对两瓶用同一个 `+0.048` (瞄准点左移) ⇒ **第 2 瓶方向反了**。9-27 实测首次落点: e 第 1 投偏右 5cm、第 2 投偏左 12cm (脱靶); f 第 1 投偏右 8cm、第 2 投偏左 2~3cm。用"现值 + 实测落点"反推所需补偿: **第 1 瓶 +9.8~+12.8 (均值 +11.3)**, **第 2 瓶 −7.2~+2.3 (均值 −2.5, 反号)**; 纵向实测基本不偏 ⇒ XF 不动。

**当前参数 (与 f 的差异只有这两个常量)**:
```
PAYLOAD_OFFSET_BODY_Y_B1 = +0.113   # f: +0.048  (第 1 瓶=右瓶; 原值只有实际需要的一半)
PAYLOAD_OFFSET_BODY_Y_B2 = -0.025   # f: +0.048  (第 2 瓶=左瓶; **必须反号**, 原值把落点推到左 ~10cm)
PAYLOAD_OFFSET_BODY_XF_B1 = +0.040  # 纵向: 与 f 相同 (9-27 实测无前后偏差)
PAYLOAD_OFFSET_BODY_XF_B2 = +0.040
# 其余全部与 f 一致: DROP_HEIGHT 1.2 / FORWARD_DIST 34 / 两级释放判据
# (DROP_ARRIVE_M 0.05 + 低谷 + vxy<0.12 确认 0.5s; Tier2 0.10/1.0s 10s 后; 兜底 35s) / 两轴补偿机制 / 相机 640x360 85°54°
# HEADING_DEG: SITL=92.0 / fly=180.0
```

**离线验收** `scripts/test_g_aim_comp_20260927.py` (3 节全过): 常量逐位一致; **真实代码路径** (出厂 `PIDController` + `level_to_camera_pixel`) 下"桶在补偿点 +11.3/−2.5cm 处 ⇒ err 0.00px", 同一桶位用旧常量 ⇒ **+6.50 / −7.30cm = 需要的修正量**, 桶放旧符号侧 ⇒ +5.00cm ⇒ 反号必要; **9-27 四次实测落点回放** ⇒ 落点 −1.5 / −4.7 / +1.5 / +4.8cm, **最坏 12cm→4.8cm**。

**SITL 复验**: `bash ~/SITL/scripts/test_g.sh` (world=`worlds/g.world`, 日志 `/tmp/g_output.log`)。**⚠ 判读注意**: SITL 里没有"载荷挂载偏置", 补偿会让飞机**故意**停在桶心左 11.3cm (第 1 瓶) / 右 2.5cm (第 2 瓶), 裁判量的是飞机地面投影 ⇒ **SITL 分数会被这个伪影压低, 不能用来评价本次改动的收益**; 要看的是 ①两投是否仍走"收敛即投" ②`aim` 与 `pos` 的差是否等于补偿量 (证明常量正确送达)。真收益在**下一次实飞落点**。

**环境/启动**: SITL `bash ~/SITL/scripts/test_g.sh`; 实机 `bash ~/fly/g.sh`。
'''

ACTIVE_ADD = '''- **g v1** (`src/drone_drl/drone_drl/g.py`): **子项目 g** 基于 f (2026-09-27 新建, e/f 不动), 唯一改动 = **两瓶横向补偿分别重标** (`Y_B1=+0.113` / `Y_B2=-0.025` 反号; XF 保持 +0.040), 依据 9-27 双航班实测落点。离线验收 `scripts/test_g_aim_comp_20260927.py`: 真实 PID 路径均衡点正确 + 四次实测落点回放最坏 12cm→4.8cm。
- **fly/g v1** (`fly/src/fly/fly/g.py`): 实机版 g (HEADING_DEG=180.0, PID 抽帧 4Hz, `_v1` 话题), 实机启动 `bash ~/fly/g.sh`。
- 启动 g: `bash ~/SITL/scripts/test_g.sh`
'''

LOG_ENTRY = f'''## 2026-09-27 — 9-27 双航班实飞复盘 (e 一次 + f 一次) + 新建项目 g (两瓶横向补偿重标)

> 数据: `9.27/e_mission_20260927_105447.log` + `9.27/log_427_2026-9-27-10-57-56.ulg` (e, 168.0s) 与 `9.27/f_mission_20260927_110915.log` + `9.27/log_430_2026-9-27-11-12-20.ulg` (f, 165.2s)。两航班同场地、同一对桶、同一顺序 (投放次序: 第 1 投=右瓶, 第 2 投=左瓶)。分析脚本 `scripts/test_g_aim_comp_20260927.py` / `scripts/new_project_g_20260927.py`。

### 一、实况与真值

| | e 航班 (e 代码) | f 航班 (f 代码) |
|---|---|---|
| 结束/时长 | normal / 167.0s | normal / 164.3s |
| 触发 | 收敛即投 ×2 (旧 10cm 判据) | **Tier1 ×2** (段内最小 2.1 / 3.5cm) |
| 释放残差 | 4.2cm / **1.4cm** | 2.1cm / 3.6cm |
| **实测首次落点** | **偏右 5cm** / **偏左 12cm (脱靶)** | **偏右 8cm** / **偏左 2~3cm** |
| 侦察/着陆 | 39.0s / 距 home 0.06m | 38.4s / 距 home 0.10m |

### 二、根因: 两瓶镜像挂载, 而代码对两瓶用同一个横向补偿 (符号错)

用户给出的**关键几何**: 载荷一左一右挂载 —— **第 1 瓶(先投, PWM 2000µs)释放点在相机轴右 +5.5cm**, **第 2 瓶(后投, PWM 1000µs)在左 −5.5cm**; 而代码 `PAYLOAD_OFFSET_BODY_Y_B1 = _B2 = +0.048`(瞄准点左移) ⇒ **第 2 瓶的补偿方向是反的**。

用 `落点 = 挂载偏置 S − 补偿 comp + (漂移 − 残差) D` 回放:

| 投放 | S | 代码 comp | D (ULog 算) | 模型预测 | 实测 | 未解释 |
|---|---|---|---|---|---|---|
| e#1 (右瓶) | +5.5 | +4.8 | −3.0 | −2.3cm | **+5cm** | +7.3 |
| f#1 (右瓶) | +5.5 | +4.8 | −2.9 | −2.2cm | **+8cm** | +10.2 |
| e#2 (左瓶) | −5.5 | +4.8 | −2.9 | **−13.2cm** | **−12cm** | **+1.2 ✓** |
| f#2 (左瓶) | −5.5 | +4.8 | +1.8 | −8.5cm | **−2.5cm** | +6.0 |

⇒ **e#2 的 12cm 偏左脱靶被模型精确解释 (−13.2 vs −12, 差 1.2cm)**; f#2 同向偏左但被漂移(+1.8cm 右)与相位救回桶内。**第 1 瓶两航班反推的有效偏置一致 (12.8 / 15.7cm)**, 台架 5.5cm 与之差 ~8cm ⇒ 存在**动态右偏项** (插销/摩擦侧向冲量 + 相机光轴安装偏角的模型误差) ⇒ 必须以实飞落点标定。

### 三、把可测量项算尽: "释放相位/摆动"都不是这次的原因

- **释放残差** 1.4~4.2cm (相机实测, 四次都被压得很小);
- **弹道漂移** (按**实际脱钩时刻**速度算, 脱钩在命令后 +0.2~0.3s, 由推力/电流台阶定位) 2~5cm —— 例 e#2 指令时 0.109 m/s、脱钩时只剩 0.071 m/s;
- **载荷摆动**: 用 50Hz `vehicle_torque_setpoint` 反推 (姿态被飞控稳住, 只有力矩暴露外力): 摆角 0.78~1.33° ⇒ 摆动速度 1.4~5.7cm/s ⇒ **落点散布 0.7~2.6cm, 可忽略** (e#2 的 1.8cm 还不是最大);
- 三项合计 ≤6cm ⇒ **12cm 脱靶的缺口只能来自"瓶相对瞄准点的偏移没有被补偿对"**。
- 旁证: e#2 释放前俯仰角速度在 **0.89Hz** 有谱线 (2.08°/s, 是同航班第 1 投的 4.7 倍; 0.89Hz ↔ 单摆 L≈0.31m 与吊挂高度吻合), 但能量只对应 1.8cm ⇒ 异常真实但**不足以定胜负**。另: 0.7~1.1Hz 频段被机动主导 (侦察/返航段高达 13~25°/s), **不能当载荷探测器**。
- **链路体检**: 脉冲 e 1.000s/1.101s、f 1.002s/1.102s 干净; **四次投放窗口内 0 个 RC 派生样本 (N1 未触发)**; 脱钩顺利。

### 四、新建项目 g (= f 的完整副本 + 只改两个常量)

- **生成**: `scripts/new_project_g_20260927.py` (从 f.py 复制, 只改 命名 + 两常量; 与 f 差异 **56 行**); 新增 `src/drone_drl/drone_drl/g.py`、`fly/src/fly/fly/g.py`、`worlds/g.world`、`scripts/test_g.sh`、`fly/g.sh`、两份 `setup.py` 的 `g` 入口。**e/f 一行未动** (e 仍 `f2bb21d1`/`0daa0712`, f 未改)。`GMission` / `g_mission` / `MISSION_REVISION=competition-water-20260927-r1-g`。
- **改动**: `PAYLOAD_OFFSET_BODY_Y_B1` 0.048→**+0.113**; `PAYLOAD_OFFSET_BODY_Y_B2` +0.048→**−0.025** (**反号**); `XF` 保持 +0.040 (纵向实测不偏)。判据/伺服/侦察/降落全部沿用 f。
- **离线验收** `scripts/test_g_aim_comp_20260927.py` (三节全过): ① 两份常量逐位一致; ② **真实代码路径** (exec 出厂 `PIDController` + `level_to_camera_pixel`, pitch 3°/roll 2°) — 桶在补偿点 +11.3/−2.5cm 处 **err=0.00px**, 同一桶位用旧常量 **+6.50 / −7.30cm (= 需要的修正量)**, 桶放旧符号侧 +5.00cm ⇒ 反号必要; ③ **9-27 四次实测落点回放**: **+5 / −12 / +8 / −2.5cm → −1.5 / −4.7 / +1.5 / +4.8cm**, 最坏 **12cm → 4.8cm** (桶口半径 12.5cm)。
- **SITL 复验**: {SITL_TXT} {SITL_CAVEAT} **说明**: SITL 里没有"载荷挂载偏置", 而补偿会让飞机故意偏桶心 ⇒ **SITL 分数不能评价本次收益** (与 9-23 记录的同源伪影); 它只能证明代码无回归与触发正常。**真收益要看下一次实飞落点**。

### 五、给下一次实飞的建议
1. **先做地面脱钩试验** (不装桨, 反复触发放机构 20~30 次, 量载荷落点分布): 直接测出"机械散布 σ"与"每瓶的实际偏置", 这是目前唯一还没被闭环控制、也测不到的项。
2. 实飞时**逐发记录首次落点 (离桶心距离 + 方向, 注明是否砸桶沿弹开)** —— 只有真值才能继续标定; 当前日志里仍是 `actual_impact=unmeasured`。
3. 若某发又出现明显偏左/偏右, 先看**那一发是第几投** (右瓶/左瓶) —— 两瓶的补偿现在已分开, 偏置方向会直接指出是常量还是机构问题。

---

'''

def main():
    print('=== 1. CLAUDE.md ===')
    p = os.path.join(ROOT, 'CLAUDE.md')
    s = open(p, encoding='utf-8').read()
    shutil.copy2(p, p + f'.bak.{STAMP}')
    subs = [
        ('| `f` | `f.py` |', None, None),   # 占位, 下面单独处理表格行
    ]
    # 1.1 包表 + g 行 (插在 f 行之后)
    frow_start = s.index('| `f` | `f.py` |')
    frow_end = s.index('\n', frow_start) + 1
    grow = ('| `g` | `g.py` | **子项目 g** 基于 f (2026-09-27 新建, e/f 不动), 唯一改动 = '
            '**两瓶横向补偿按 9-27 实飞落点分别重标**: `Y_B1=+0.113` (右瓶, 原 0.048 只有一半) / '
            '`Y_B2=-0.025` (**左瓶必须反号**, 原值把落点推到左 ~10cm), 纵向 `XF=+0.040` 不动。'
            '其余 (两级释放判据 / DROP 段伺服 / 侦察 / 降落) 与 f 完全一致 |\n')
    s = s[:frow_end] + grow + s[frow_end:]
    # 1.2 阶段性总结
    anchor = '最新开发迭代 (2026-09-24)：'
    assert s.count(anchor) == 1
    s = s.replace(anchor, SECT + anchor, 1)
    # 1.3 已知问题
    kanchor = '### PX4 自定义模型'
    assert s.count(kanchor) == 1
    s = s.replace(kanchor, KNOWN_ADD + kanchor, 1)
    # 1.4 活跃版本 + 启动行
    aanchor = '- 启动 drop_opt: `bash ~/SITL/scripts/test_drop.sh`'
    assert s.count(aanchor) == 1
    s = s.replace(aanchor, ACTIVE_ADD + aanchor, 1)
    # 1.5 世界
    wanchor = '- `worlds/f.world` — **子项目 f 世界**。复制自 e.world (与 a/c/d/e.world 内容完全相同)。2026-09-24 新建\n'
    assert s.count(wanchor) == 1
    s = s.replace(wanchor, wanchor + '- `worlds/g.world` — **子项目 g 世界**。复制自 f.world。2026-09-27 新建\n', 1)
    # 1.6 文末 g 章节
    assert '### g — 子项目 g v1' not in s
    s = s.rstrip('\n') + '\n' + F_SECTION
    open(p, 'w', encoding='utf-8').write(s)
    print('   CLAUDE.md: 包表+g / 阶段性总结 / 已知问题 / 活跃版本 / 世界 / 文末章节 全部写入')

    print('=== 2. docs/project_log.md ===')
    q = os.path.join(ROOT, 'docs/project_log.md')
    t = open(q, encoding='utf-8').read()
    shutil.copy2(q, q + f'.bak.{STAMP}')
    la = '## 2026-09-24 — 项目 e 第二次实飞'
    assert t.count(la) == 1
    t = t.replace(la, LOG_ENTRY + la, 1)
    open(q, 'w', encoding='utf-8').write(t)
    print('   project_log.md: 置顶插入 2026-09-27 条目')

    print('=== 3. 校验 ===')
    c = open(p, encoding='utf-8').read()
    for k in ['| `g` | `g.py` |', '### g — 子项目 g v1', 'worlds/g.world', '最新开发迭代 (2026-09-27)',
              '两瓶镜像挂载 ⇒ 横向瞄准补偿必须按瓶分别设', 'Y_B2=-0.025', '- **g v1**']:
        assert k in c, f'CLAUDE.md 缺 {k}'
    l = open(q, encoding='utf-8').read()
    assert '## 2026-09-27 — 9-27 双航班实飞复盘' in l and l.index('## 2026-09-27') < l.index('## 2026-09-24')
    print('   CLAUDE.md + project_log.md 校验通过')
    print(f'\n完成 (备份后缀 .bak.{STAMP})')


if __name__ == '__main__':
    main()
