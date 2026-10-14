# CONFIGURATION — 参数与配置

## 一、环境变量（`test_k.sh` 会设）

| 变量 | 值 | 作用 |
|---|---|---|
| `PX4_GZ_MODELS` | `~/SITL/gz_models` | 模型目录 |
| `PX4_GZ_WORLDS` | `~/SITL/gz_worlds` | 世界目录 |
| `PX4_GZ_WORLD` | `k_gz` | 世界名（**必须与文件名匹配**）|
| `PX4_GZ_MODEL_POSE` | `1.01,0.98,0.83` | 飞机生成点（与 Classic 对齐）|
| `HEADLESS` | `1`（默认）| 无界面；`SIM_GUI=1` 时不设 |
| `PX4_SITL_WORLD` | `worlds/k.world` | 世界文件绝对路径 |

## 二、gz 专属 PX4 参数（写在 `test_k.sh` 里）

```
CBRK_SUPPLY_CHK = 894281    # gz 的 x500 没有电源模块 ⇒ 关掉电源检查
NAV_DLL_ACT     = 0         # 没有地面站时否则拒绝解锁
```

## 三、我们的飞行器模型（`models/x500_base/`）

**【源码事实】** 基于 PX4 的 `x500_base`，我们改了 `model.sdf`：

| 项 | 值 | 为什么 |
|---|---|---|
| `mass` | **1.5**（原 2.0）| 对齐 SITL iris 的动力学 |
| `ixx, iyy` | **0.029125** | 同上 |
| `izz` | **0.055225** | 同上 |
| **相机** | **加装了 130° 下视相机** | 与实机相机一致 |

⚠️ `meshes/` 与 `materials/` 是**逐字节拷自 PX4**，未入库，复现命令见 `SETUP.md`。

## 四、相机参数（仿真 = 实机）

| 参数 | 值 |
|---|---|
| 视场 | 对角线 **130°**（HFOV 123.70° / VFOV 92.87°）|
| 内参 | `FX = FY = 171.21 px` @ 640×360 |
| 畸变 | 无 |
| 安装 | 固定下视 |

`FX` 的算法：`FX = (√(640²+360²)/2) / tan(65°)` ✓

## 五、场地（`worlds/`）

| 文件 | 说明 |
|---|---|
| `k.world` | 由 `scripts/gen_world_cuadc.py --name k --seed 33` 生成 |
| `k.truth.json` | **与 `.world` 同一次生成** ⇒ 桶坐标唯一来源 |
| `k_nc.world` | 同桶位但**桶无 collision**（用于隔离物理扰动）|
| `*_gz.sdf` | 转成 gz 格式的世界（PX4 用）|

**场地规格**【源码事实】：
- 三档白桶 **15 / 20 / 25 cm**，高 **0.30 m**
- 投放区 NED `y ∈ [30, 35]`
- 侦察区 NED `y ∈ [55, 60]`

## 六、任务参数（`k.py` 的常量）

关键值（完整 34 个 `K_*` 见源码）：

| 常量 | 值 | 含义 |
|---|---|---|
| `K_RELEASE_MIN_M` | **0.03** | 释放容差 3 cm（对齐参考项目）|
| `K_RELEASE_CONFIRM` | **2.00** | 稳定窗 2.0 s |
| `K_DEADLINE_S` | **12.0** | 必投兜底 |
| `K_VMAX_DROP` | **0.20** | DROP 段限速 m/s |
| `K_RESID_MAX_AGE` | **0.15** | 残差新鲜度门 s |
| `K_DIA_IDENTITY_TOL` | **0.03** | 直径身份门 m |
| `K_FWD_RAMPIN_M` | **3.0** | FORWARD 起步斜坡 m |
| `K_AIM_KI` | **0.00** | 瞄准点积分增益（关闭）|
| `K_ZERO_COMPENSATION` | **True** | 补偿归零（等实机标定）|
