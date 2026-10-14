# 部署代码到 Jetson

## 一、一条命令

```bash
# 在本机 ~/SITL 下
bash scripts/deploy_k_jetson_20261008.sh
```

**【源码事实】** 它自动做这些事：

1. **备份** Jetson 上现有的 `k.py`（带时间戳）
2. `scp` 上传新的 `k.py` 到 `~/fly/src/fly/fly/k.py`
3. **自动改场地航向**：`HEADING_DEG` → **212.0**
4. 注册 entry point（`setup.py`）
5. `colcon build --packages-select fly`
6. **校验 `src` 与 `install` 两个副本的 sha 一致** ✓

**回退**：

```bash
bash scripts/deploy_k_jetson_20261008.sh --rollback
```

## 二、⚠️ 部署后必须核对

```bash
# 在 Jetson 上核对两个副本是否一致
sha256sum ~/fly/src/fly/fly/k.py
sha256sum ~/fly/install/fly/lib/python3.10/site-packages/fly/k.py
```

**两个 sha 必须相同** ✗ 不一致 = **你改了源码，但跑的还是旧产物**。

> 【经验/建议】这是最容易犯的错：`scp` 完忘了 `colcon build`，
> 或者 build 了但没 `source install/setup.bash` ⇒ 跑的是旧代码 ✗

## 三、上板前先离线验收

```bash
# 在本机
python3 scripts/test_k_fly_20261013.py
```
**期望**：`结果: 42 项通过 / 0 项失败`

它检查：文件与入口、**5 个实机侧环境常量**、**跨版本 34 个 `K_*` 常量不许漂移**、
**D1~D37 标记各 37 处**、实机专属适配、6 项危险检查、血统审计 ✓

## 四、实机侧的环境常量（与仿真不同）

| 常量 | 实机值 | 仿真值 | 为什么不同 |
|---|---|---|---|
| `K_SIM_NO_BALLISTIC` | **False** | True | **实机有真实弹道** ⇒ 判据要考虑下落漂移 |
| `K_SIM_IDEAL_CAMERA` | **False** | True | 实机相机有噪声/延迟 |
| `K_ZERO_COMPENSATION` | **True** | True | 都归零，等实机标定 |
| `K_TARGET_PLANE_H` | **−0.278** | 0.0 | 实机地面高度基准不同 |
| `K_AIM_KI` | **0.00** | 0.00 | 一致 |

## 五、现场要核对的

| 项 | 值 |
|---|---|
| **`HEADING_DEG`** | 部署脚本自动改 **212.0**（**换场地要改这个**）|
| **`FORWARD_DIST`** | **32.5** |
| `K_RELEASE_MIN_M` | 0.03（他们的 3 cm）|
| `K_RELEASE_CONFIRM` | 2.00（他们的 2.0 s）|
| `K_DEADLINE_S` | 12.0 |

## 六、启动

```bash
# 在 Jetson 上
bash ~/fly/k.sh                 # 自动落盘 ~/fly/log/k_<时间戳>.log
bash ~/fly/k.sh --no-log        # 不落盘
```

## 关联

- [`飞行操作与首飞清单.md`](飞行操作与首飞清单.md)
- [`日志分析与误差归因.md`](日志分析与误差归因.md)
- [`../技术文档/项目速查/CLAUDE.md`](项目速查/CLAUDE.md) —— 完整实机流程
