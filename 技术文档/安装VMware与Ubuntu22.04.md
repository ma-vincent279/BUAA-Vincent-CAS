# 安装 VMware 与 Ubuntu 22.04

> **写给**：电脑上还是 Windows、完全没装过 Linux 的人。
> 照本文做完，你会有一个**能跑本项目的 Ubuntu 22.04**。
>
> ⚠️ 本项目要求 **Ubuntu 22.04**（因为 ROS 2 Humble 的官方支持版本就是它）✗ 别装 24.04。

---

## 一、先决定：虚拟机 还是 双系统？

| 方案 | 优点 | 缺点 | 适合谁 |
|---|---|---|---|
| **虚拟机 (VMware)** ★ | 装错删了重来**不影响 Windows**；随时快照回滚 | 性能有损耗（尤其 3D/仿真）| **推荐先用这个上手** |
| **双系统** | 性能完整（仿真跑得快）| 分区有风险；切换要重启 | 确定要长期用 |

> **【经验/建议】** 本项目**仿真（Gazebo）在虚拟机里会明显卡**，
> 如果只是学 Linux、写代码、跑离线验收 ⇒ 虚拟机够用 ✓
> 如果要跑仿真 ⇒ 建议**双系统**或直接装 Ubuntu。

---

## 二、VMware 安装

### 2.1 下载

VMware Workstation **Player**（个人使用免费）
→ 官网搜 "VMware Workstation Player" 下载 Windows 版 `.exe`

### 2.2 安装

双击 → 下一步到底 → 需要重启就重启 → **不用输序列号**（选个人非商业用途）

---

## 三、下载 Ubuntu 22.04 镜像

**一定要下 22.04 LTS 的 Desktop 版**：

```
https://releases.ubuntu.com/22.04/
→ ubuntu-22.04.x-desktop-amd64.iso   （约 4.7 GB）
```

> ⚠️ 别下 Server 版（没有图形界面，新手会很痛苦）✗

---

## 四、创建虚拟机

### 4.1 硬件配置建议

| 项 | 建议值 | 说明 |
|---|---|---|
| 内存 | **8 GB 以上**（宿主 16 GB 时给 8）| Gazebo 很吃内存 |
| CPU | **4 核以上** | |
| 硬盘 | **80 GB 以上**（拆成多个文件 ✓）| Ubuntu + ROS + PX4 会吃掉 40 GB+ |
| 网络 | **NAT**（默认）| 能上网，不用配 |

> **【经验/建议】** 勾上「**将虚拟磁盘拆分成多个文件**」—— 便于迁移 ✓
> 再开「**虚拟化引擎 → 虚拟化 Intel VT-x/EPT**」—— 虚拟机里跑仿真需要它 ✓

### 4.2 步骤

1. VMware Player → **Create a New Virtual Machine**
2. 选 **Installer disc image file (iso)** → 指向下载的 `.iso`
3. 填一个简单的用户名/密码（**记住密码** ✓）
4. 磁盘给 **80 GB**，勾「拆分成多个文件」
5. **Customize Hardware** → 内存 **8 GB** / 处理器 **4 核**
6. **Finish** → 自动开机安装

---

## 五、Ubuntu 安装过程

1. 选语言 → **Install Ubuntu**
2. 键盘布局：默认 English (US) ✓
3. 更新选项：**建议不勾**「安装时下载更新」（会慢很多，装完再更新 ✓）
4. 安装类型：**Erase disk and install Ubuntu**（这是**虚拟机的虚拟盘**，安全 ✓）
5. 时区：**Shanghai**
6. 用户名/密码：**记住**
7. 等约 20 分钟 → **Restart Now**

### 5.1 装完后第一件事

**打开终端**：`Ctrl + Alt + T`

```bash
# 换国内源（可选，但国内会快很多）
sudo sed -i 's|//archive.ubuntu.com|//mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list
sudo sed -i 's|//security.ubuntu.com|//mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# 更新
sudo apt update && sudo apt upgrade -y
```

### 5.2 装 VMware Tools（让剪贴板/分辨率正常）

```bash
sudo apt install -y open-vm-tools open-vm-tools-desktop
# 然后重启
sudo reboot
```
装好之后，**宿主机与虚拟机之间可以复制粘贴** ✓ 分辨率也能自适应 ✓

---

## 六、快照（非常重要）

**装完系统、装好 ROS 之后，各做一次快照** ✓

VMware → **VM → Snapshot → Take Snapshot**

> **【经验/建议】** 快照是虚拟机的最大优势：
> 把环境搞坏了，**几秒钟回滚** ✓ 而不是重装一天 ✗
>
> 建议在两个时间点各拍一次：
> ① 刚装完系统 + VMware Tools
> ② 装好 ROS 2 之后

---

## 七、验证装好了

```bash
# 看版本
lsb_release -a          # 应该是 Ubuntu 22.04.x LTS

# 看硬件
free -h                 # 内存
nproc                   # CPU 核数
df -h /                 # 磁盘

# 看能不能上网
ping -c 3 www.baidu.com
```

全部正常 ⇒ 环境就绪，可以继续：

→ [`配置仿真环境.md`](配置仿真环境.md)（装 ROS 2 + Gazebo + PX4）
→ [`../从零开始的CUADC生活/从零开始的Linux生活.md`](../从零开始的CUADC生活/从零开始的Linux生活.md)（Linux 基础）

---

## 八、常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| 虚拟机里**特别卡** | 没开虚拟化 / 内存给太少 | 进 BIOS 开 VT-x；内存给到 8 GB |
| **不能复制粘贴** | 没装 VMware Tools | `sudo apt install open-vm-tools-desktop` 后重启 |
| 分辨率**不能全屏** | 同上 | 同上 |
| 装完**黑屏** | 显卡驱动 | 重启时按住 `Shift` 进 GRUB → 选 `nomodeset` 启动 |
| `apt update` **很慢** | 源在国外 | 按 5.1 换清华源 |
| 磁盘**满了** | 默认磁盘太小 | 关机 → 虚拟机设置 → 扩展磁盘 → 系统里 `gparted` 调整 |
