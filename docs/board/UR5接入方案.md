# UR5 接入方案（Jetson 做真实硬件接入端，替代 VMware 虚拟机方案）

前提已确认：**有 UR 控制箱**，机械臂本体+控制箱齐全。
这台 Jetson 的定位：只做"真实机械臂的接入端"，平时开发在别的电脑的 WSL2 里以仿真为主。

环境事实（已实测）：

| 项目 | 值 |
|---|---|
| Jetson 系统 | Ubuntu 22.04.5 / aarch64 / ROS 2 **Humble** |
| apt 里的 UR 驱动 | `ros-humble-ur` **2.14.0**（含 ur-robot-driver / ur-calibration / ur-moveit-config） |
| MoveIt | 源里有 `ros-humble-moveit` 2.5.10（**当前未安装**） |
| ros2_control | 源里有 2.54.2（**当前未安装**） |
| 默认 RMW | Fast DDS（`rmw_fastrtps_cpp`），CycloneDDS 未装 |
| ROS_DOMAIN_ID | 99（已全局设置） |
| 编译工具链 | colcon / cmake / g++ / rsync 齐全 |
| 网络 | `eno1 = <host-ip>/23`，网关 `<ip>`（VRRP 虚拟 MAC，企业级） |
| 磁盘余量 | 169 GB 可用 |

---

## 1. 整体架构

```
┌──────────────────────────────┐        ┌────────────────────────┐        ┌───────────────────┐
│ 开发机 (Windows + WSL2)      │        │ Jetson Orin NX 8GB     │        │ UR5 + 控制箱      │
│                              │        │                        │        │                   │
│  ● 代码开发 / git            │        │  ● ur_robot_driver     │ RTDE   │  伺服环 500Hz     │
│  ● RViz2 可视化              │◄──────►│  ● ros2_control        │ TCP    │  硬件安全回路     │
│  ● MoveIt 规划（建议放这里） │  ROS2  │  ● ur_calibration      │ 30004  │  急停/保护停      │
│  ● URSim 仿真（Docker）      │  DDS   │  ● 不开 RViz，省内存   │◄──────►│  Dashboard 29999  │
│    domain id = 99            │        │    domain id = 99      │        │                   │
└──────────────────────────────┘        └────────────────────────┘        └───────────────────┘
```

分工原则：**实时性和硬件接口相关的留在 Jetson，可视化与规划放到开发机。**
原因很直接：8 GB 内存被桌面+Codex 占掉一大半，RViz2 还要 400–700 MB；而规划本身
不需要挨着硬件，放到开发机上还顺手。

---

## 2. 三个必须先确认的信息

这三个信息决定后面装哪个 URCap、用哪个 `ur_type`、驱动版本兼容性如何。

| # | 要确认的 | 在哪看 | 影响 |
|---|---|---|---|
| 1 | 是 **UR5（CB3）** 还是 **UR5e** | 控制箱外观 / 示教器型号 | `ur_type:=ur5` 还是 `ur5e`；URCap 版本不同 |
| 2 | **PolyScope 版本** | 示教器 设置 → 系统 → 关于 | 决定 External Control URCap 的版本、`servoj` 参数支持 |
| 3 | 控制箱当前网络设置 | 示教器 设置 → 网络 | 决定是沿用现有 IP 还是重设 |

---

## 3. 网络方案（二选一）

先说一个容易搞错的概念：**RTDE 走的是 TCP（30004），不是 UDP**。
所以交换机/办公网完全可以胜任 125 Hz 的指令下发；专用网卡的价值在于
**隔离和可预测性**，不是硬性要求。别走 WiFi 就行。

### 方案 A：接进办公网（零新增硬件，建议先用这个）

- UR 控制箱网口接到办公网交换机；
- 在示教器上给 UR 设**静态 IP**，并且这个地址要**落在 DHCP 池之外**（问一下网管，或选一个明显靠后的地址，先用 `arping` 确认没人占用）；
- Jetson 和 UR 在同一网段，DDS 发现和 RTDE 都不依赖路由器转发，最省事。

注意：办公网网关是 VRRP 对（`<ip>`），对其它网段的转发策略未知。
实测 `<ip>` 现在是"经网关转发"而不是二层直达，所以**不要把 UR 放在和 Jetson 不同的网段**去指望路由器帮你打通。

### 方案 B：专用控制网（隔离最好）

- 插一块 **USB 3.0 千兆网卡**，网卡设静态 `<ip>/24`；
- UR 设静态 `<ip>/24`，**网关和 DNS 都留空**（不让机械臂想上网）；
- 直连或用小交换机。控制流量完全不经过办公网。

先 A 后 B：A 验证链路，如果以后发现 RTDE 有周期性抖动再上 B。

---

## 4. Jetson 侧要装什么

```bash
sudo apt install ros-humble-ur ros-humble-ur-msgs
```

会带上 `ur-robot-driver`、`ur-calibration`、`ur-description` 等。
如果规划放开发机，`ur-moveit-config` 和 `ros-humble-moveit` **不用装在 Jetson 上**，
能省不少内存和磁盘；如果想让板子独立跑完整栈，就一起装。

---

## 5. 示教器侧要做什么

1. **装 External Control URCap**：从 UR 官方驱动仓库拿对应 PolyScope 版本的 `.urcap`，
   插 U 盘在示教器上安装（设置 → 系统 → URCap）。
2. **建一个程序**：里面只放 External Control 节点，命名固定，设为 **开机自动加载**；
   驱动要靠这个程序接管控制权。
3. **切成 Remote Control 模式**（设置 → 系统 → 远程控制）。
4. 记下控制箱 IP，确认它能被 Jetson ping 通。

⚠️ External Control 程序一旦加载运行，机械臂就会听上位机的指令动。
第一次调试务必：速度缩放到 10–20%、示教器旁边有人守着急停、机械臂工作空间里别放东西。

---

## 6. 联调顺序（从空跑到真机）

### 第 0 步：空跑，不接机械臂

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 use_fake_hardware:=true
```

先用假硬件把 `ros2_control` 链路和 controller 起来，确认话题、service、
controller 名称都对。这一步完全不碰机械臂，零风险。

### 第 1 步：连真机

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=<控制箱IP>
```

驱动启动时会打印固件兼容性提示，按提示对齐版本。

### 第 2 步：标定（决定精度，别跳）

```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<控制箱IP>
```

生成的运动学标定文件要放进驱动的配置目录，否则 TCP 位置会有毫米级偏差。

### 第 3 步：验证

```bash
ros2 topic hz /joint_states        # 期望 ~125 Hz
ros2 control list_controllers      # 期望 joint_state_broadcaster 等 active
```

### 第 4 步：再上规划

先手写一个小的关节空间目标点让机械臂小幅动一下，确认方向和限位都对，
再引入 MoveIt 做笛卡尔规划。

---

## 7. 开发机（WSL2）侧怎么接进来

### 7.1 最大的坑：WSL2 默认网络模式下 DDS 发现不到局域网节点

WSL2 默认是 NAT 网络，虚拟机里的多播出不了 Windows 的虚拟交换机，
所以 WSL2 里的 `ros2 topic list` 看不到 Jetson 上的节点。三种解法：

**解法 1（推荐，最省事）**：Windows 11 22H2+ 的镜像网络模式

在 `%UserProfile%\.wslconfig` 里加：

```ini
[wsl2]
networkingMode=mirrored
```

然后 `wsl --shutdown` 重启。WSL2 直接共享主机的网卡，多播可用，DDS 基本即插即用。
注意：如果开发机上开着 VPN，镜像模式偶尔会冲突，遇到就退到解法 2。

**解法 2（最通用）**：Fast DDS Discovery Server

```bash
# Jetson 上
fastdds discovery -i 0 -l 0.0.0.0 -p 11811

# 两边都设
export ROS_DOMAIN_ID=99
export ROS_DISCOVERY_SERVER=<Jetson的IP>:11811
```

这是官方推荐用于跨 NAT / 容器 / 多机的方案，对 WSL2 特别有效。

**解法 3**：两边都换 CycloneDDS，写显式单播 peer 的 XML（要装
`ros-humble-rmw-cyclonedds-cpp`，Jetson 上目前没装）。

### 7.2 两个必须对齐的东西

**ROS 2 版本**：Jetson 是 Humble / Ubuntu 22.04，WSL2 也用 Humble / 22.04 最稳
（`osrf/ros:humble-desktop` 这个 Docker 镜像最干净）。混用 Jazzy/24.04 虽然线协议
能通，但消息定义和 RMW 版本差异会制造很难排查的问题。

**CPU 架构**：WSL2 是 x86_64，Jetson 是 aarch64，**colcon 编出来的二进制不能互拷**。
所以只有两条路：

- 源码用 git/rsync 同步到 Jetson，**在板子上编译**（你现在的 `yahboom_ws` 就是这么干的，保持即可）；
- 或者用 `docker buildx` 做 aarch64 交叉编译（复杂度高，除非包很多否则不值得）。

纯 Python 包理论上可以直接拷，但只要有 `.msg/.srv` 或者 C++，就得按架构重编。

### 7.3 建议的仓库结构（仿真/真机一套代码）

放在一个仓库里，靠 launch 参数区分，不要维护两份代码：

```
arm_bringup/
  launch/
    sim.launch.py     # URSim 或 use_fake_hardware:=true
    real.launch.py    # robot_ip + 真标定文件
    common.launch.py  # 两边共用的 controller 配置、速度限制
  config/
    controllers.yaml
    ur_description/   # 仿真与真机共用同一份，除了标定文件
```

仿真的价值在于：**把速度/加速度缩放值和真机设成一样**，
仿真里能跑通的轨迹到真机上才是同一个数量级，不然仿真过、真机超速是很常见的翻车点。

### 7.4 URSim

- **e 系列（UR5e）**：官方有 Docker 镜像，在 WSL2 里直接跑，映射 30001–30004 端口，
  驱动连 `127.0.0.1` 即可，体验最好。
- **CB3（UR5）**：URSim 是较老的 VirtualBox 镜像，在 WSL2 里跑比较重，
  建议这台开发机上用 VirtualBox/VMware 单独跑，或者干脆直接在 Jetson 上用
  `use_fake_hardware:=true` 做链路级验证。
- 无论哪种，**URSim 的 PolyScope 版本要和真机一致**，否则 URCap 和驱动行为会不一致。

---

## 8. 一句话总结这条链路的关键决策

| 决策点 | 建议 |
|---|---|
| 谁连机械臂 | 只有 Jetson，通过 RTDE/Dashboard |
| 谁做伺服环 | UR 控制箱（Jetson 不承担硬实时） |
| 谁跑 RViz/MoveIt | 开发机（WSL2），Jetson 上不开 |
| UR 网络放哪 | 先跟 Jetson 同网段（方案 A），有抖动再上专用网卡 |
| 仿真和真机怎么统一 | 一个仓库两套 launch，速度限制共用 |
| 编译在哪 | 源码同步到 Jetson 上编译（架构不同） |
| DDS 怎么跨 WSL2 | 镜像网络模式，或 Fast DDS Discovery Server |
