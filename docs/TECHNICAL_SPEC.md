# YOLO + ROS2 视觉抓取系统 — 技术方案文档

> **项目目标**：在 Gazebo Harmonic 仿真环境中，实现 YOLO 视觉感知 → 3D 空间定位 → 逆运动学求解 → 机器人抓取的完整闭环。

---

> # ⚠️ 本文是**设计稿**，不是当前实现的描述（P1-5）
>
> 本文写在动手实现**之前**。当时的选型与实际落地的环境**不一致**，若照着本文
> 的命令操作会失败（装 Jazzy、找不存在的文件）。实测对照（2026-09-27 本机）：
>
> | 本文写的 | 实际是 | 怎么核实的 |
> |---|---|---|
> | Ubuntu 24.04 (Noble) | **Ubuntu 22.04.5 LTS (jammy)** | `cat /etc/os-release` |
> | ROS 2 Jazzy | **ROS 2 Humble** | `ls /opt/ros/` → `humble` |
> | Python 3.12 (Conda) | **Python 3.10.21，conda 环境**（miniforge3 派生的 `robot-vision`） | `~/robotics/venvs/robot-vision/bin/python --version`；该目录有 `conda-meta/`，故是 conda env 而非 venv。**`which conda` 查不到**（conda 不在 PATH 上），别据此以为没用 conda |
> | Gazebo Harmonic (gz-sim8) | ✅ 一致 | `gz sim --versions` → 8.15.0 |
> | `ros-jazzy-ros-gz` | **`ros-humble-ros-gzharmonic-sim`** | 注意是 gzharmonic 变体，非默认 `ros-gz` |
> | `ros-jazzy-moveit` | **`ros-humble-moveit`** | |
>
> **为什么从 Jazzy 回落到 Humble**：选 Jazzy 的理由是「24.04 官方 LTS」，但本项目的
> **部署目标是 Orin（JetPack 6.2 / aarch64）**，其上官方支持的 ROS 2 发行版是
> **Humble**。以真机为准，故全线回落。详见 `MIGRATION_HANDOFF.md`、
> `RESTRUCTURE_PLAN.md` 的 P2/P4 组。
>
> ## 本文中提到、但**并不存在**的东西（勿照做）
>
> | 文中位置 | 写的是 | 实际 |
> |---|---|---|
> | §6.4、README | `/trigger_grasp` 服务 | **不存在**。全仓库 `grep create_service` 无命中。规划器是**自主**的：订阅 `/targets_3d`，每来一帧就选一个目标去抓，无需外部触发 |
> | §4 包结构 | `yolo_bringup/config/moveit_config.yaml` | **不存在**；实际是 `config/ros_gz_bridge.yaml` |
> | §4、§6 | `yolo_bringup/launch/simulation.launch.py` | **已删除**（第九轮）。它是个**没有机械臂**的世界，抓取链路接上后已无意义 |
> | §4 包结构树 | 整个 `ros2_ws/src/` 树 | 与实际有出入（缺 `arm_grasp_sim`、`perception_3d`、`grasp_planner` 的细节）——以 `ls ros2_ws/src/` 为准 |
> | §3.5 | MoveIt2 → **cuRobo** | cuRobo **未使用**；规划器直接调 MoveIt 的 `/compute_ik` |
> | §5 | `pipeline.launch.py mode:=mock` | `mode` 参数**已删除**（D41）。ROS 2 对未声明的参数覆盖**静默忽略**，故旧命令不报错也不生效 |
>
> ## 当前正确的用法（唯一入口）
>
> ```bash
> # 仿真：Gazebo 世界 + UR5 + MoveIt + 感知 + 抓取，一并起来
> ros2 launch yolo_bringup bringup.launch.py camera_source:=sim
>
> # 真机：不起仿真，由 realsense2_camera 提供 /camera/*
> ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense
> ```
>
> 仿真前提：`gz_ros2_control` 需 Harmonic 源码编译版（apt 的 humble 版依赖
> Fortress，与本机 gz-sim8 **不兼容**），放在外部叠加工作区 `~/grasp_ws`。
>
> **仍然有效、值得一读的章节**：§3.3（2D→3D 投影数学）、§9（环境坑：
> 清华源 403、CoppeliaSim Qt5、numpy 2.x 与 cv_bridge）。
> 其余章节请对照上表**打折阅读**。
>
> 权威现状见 `RESTRUCTURE_PLAN.md`（缺陷台账 + 改造计划 + 验收证据）。

---

## 1. 系统架构概述

```
┌──────────────────────────────────────────────────────────────────┐
│                      Gazebo Harmonic 仿真环境                      │
│  ┌──────────┐   ┌──────────────┐   ┌────────────────────────┐   │
│  │ 机械臂模型 │   │ RGBD 相机     │   │ 待抓取物体（YCB/自定义） │   │
│  │ UR5/UR10 │   │ RealSense D435│   │                        │   │
│  └──────────┘   └──────┬───────┘   └────────────────────────┘   │
└────────────────────────┼────────────────────────────────────────┘
                         │ /rgb, /depth, /camera_info
                         ▼
┌──────────────────────────────────────────────────────────────────┐
│                       ROS2 Jazzy 节点流水线                        │
│                                                                  │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────────┐    │
│  │ yolo_detector│───▶│ perception_3d│───▶│  grasp_planner   │    │
│  │ 2D 检测框    │    │ 2D→3D 投影   │    │ 抓取位姿+IK求解   │    │
│  └─────────────┘    └──────────────┘    └────────┬─────────┘    │
│                                                   │              │
│                                          ┌────────▼─────────┐   │
│                                          │  moveit_controller│   │
│                                          │  轨迹执行         │   │
│                                          └──────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. 环境与依赖

| 组件 | 版本 | 说明 |
|------|------|------|
| OS | Ubuntu 24.04 (Noble) | WSL2 环境，WSLg 提供 GUI |
| ROS2 | Jazzy Jalisco | 24.04 官方 LTS |
| Gazebo | Harmonic (gz-sim8) | 新一代 Gazebo，原生支持 24.04 |
| ROS-Gazebo 桥 | ros-jazzy-ros-gz | ROS2 ↔ Gazebo 消息桥接 |
| MoveIt2 | ros-jazzy-moveit | 运动规划与 IK |
| YOLO | ultralytics (YOLOv8) | Python 推理 |
| 深度相机 | RealSense D435 (仿真模型) | 通过 ros_gz 桥接 |
| Python | 3.12 (Conda) | **必须 3.12**（ROS2 Jazzy rclpy 为 Python 3.12 编译） |
| CUDA | 13.2 | GPU 加速 YOLO 推理 |
| numpy | < 2.0 | ROS2 cv_bridge 基于 numpy 1.x 编译，不兼容 2.x |

---

## 3. 详细技术路线

### 3.1 RGBD 相机选型与仿真

**选型：Intel RealSense D435**

- ROS2 中有 `realsense2_camera` 包提供硬件驱动
- Gazebo 中通过 SDF 模型 + `ros_gz_bridge` 模拟 RGBD 数据
- 输出话题：
  - `/camera/color/image_raw` — RGB 图像 (sensor_msgs/Image)
  - `/camera/depth/image_raw` — 深度图像 (sensor_msgs/Image)
  - `/camera/color/camera_info` — 相机内参 (sensor_msgs/CameraInfo)

**仿真方案**：使用 Gazebo Harmonic 的 `rgbd_camera` 传感器插件，通过 `ros_gz_bridge` 将 Gazebo 话题桥接到 ROS2。

```
Gazebo rgbd_camera sensor
    │
    ▼ (gz transport)
ros_gz_bridge (parameter_bridge)
    │
    ▼ (ROS2 topics)
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
```

### 3.2 YOLO 检测框

**节点：`yolo_detector`**

- 订阅 `/camera/color/image_raw`，运行 YOLOv8 推理
- 发布自定义消息 `Detection2DArray`（或标准 `vision_msgs/Detection2DArray`）
- 每帧输出：`[ {class_id, class_name, confidence, bbox_center_x, bbox_center_y, bbox_width, bbox_height} ]`

**实现要点**：
- 使用 `ultralytics` Python 库加载预训练模型（COCO 或自定义权重）
- 通过 `cv_bridge` 将 ROS Image 转为 numpy 数组
- 支持 GPU 推理（CUDA）以降低延迟
- 发布频率：~15-30 Hz（取决于 GPU）

### 3.3 2D → 3D 空间映射

**节点：`perception_3d`**

- 输入：
  - `/camera/depth/image_raw` — 深度图
  - `/camera/color/camera_info` — 相机内参 K
  - `/detections` — YOLO 2D 检测结果
- 输出：
  - `/target_3d` — 目标在相机坐标系下的 3D 位置 (geometry_msgs/PoseStamped)

**投影公式**（针孔相机模型）：

$$
\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = Z \cdot K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

其中：
- $(u, v)$ 为检测框中心像素坐标
- $Z$ 为深度图中对应位置的深度值（取检测框中心邻域中位数，抗噪）
- $K$ 为相机内参矩阵

**关键细节**：
- 对深度值进行滤波（邻域中位数），避免单像素噪声
- 处理深度为 0/NaN 的情况
- 后续通过 TF 变换到机器人基坐标系

### 3.4 坐标系变换（手眼标定）

**变换链**：

```
camera_color_optical_frame
    │ (静态 TF: 手眼标定结果)
    ▼
tool0 / wrist_3_link (末端法兰)
    │ (正运动学 TF: 由 robot_state_publisher 发布)
    ▼
base_link (机器人基座)
```

**Eye-in-Hand（眼在手上）配置**：
- 相机固定在机械臂末端执行器上
- 标定得到 $T^{tool}_{camera}$，即相机在末端坐标系下的位姿
- 仿真中该变换由 URDF/SDF 中 joint 定义，无需真实标定

**变换实现**：
- 使用 ROS2 `tf2` 库查询变换
- `tf2_ros.Buffer.lookup_transform(target_frame, source_frame, time)`

### 3.5 逆运动学求解（MoveIt2 → cuRobo）

**第一阶段：MoveIt2 IK**

- 使用 MoveIt2 的 `computeIK()` 服务
- 输入：目标位姿 (geometry_msgs/PoseStamped)
- 输出：关节角度 (sensor_msgs/JointState)
- 通过 `MoveGroupInterface` 规划并执行轨迹

**第二阶段（后续）：cuRobo**

- NVIDIA cuRobo 提供 GPU 加速的全局 IK 求解
- 支持碰撞感知，速度远超传统 IK
- 通过 `curobo` Python/C++ 绑定调用

**当前实现**：
```
/base_link 下的目标抓取位姿
    │
    ▼
MoveIt2 MoveGroupInterface
  ├── setPoseTarget(target_pose)
  ├── plan() → 轨迹规划
  └── execute() → 发送关节指令
```

### 3.6 Gazebo 仿真场景

**仿真组件**：

1. **机械臂**：UR5e 或 UR10（`ur_description` + `ur_simulation_gz`）
2. **末端执行器**：Robotiq 2F-85 二指夹爪
3. **RGBD 相机**：RealSense D435 模型
4. **抓取对象**：简单几何体（方块/圆柱）+ YCB 物体

**Gazebo Harmonic 特点**：
- 使用 SDFormat (`.sdf`) 描述场景
- `ros_gz_bridge` 进行 ROS2 ↔ Gazebo 话题桥接
- 物理引擎：DART / Bullet

---

## 4. ROS2 包结构

```
ros2_ws/src/
├── yolo_detector/          # YOLO 检测节点
│   ├── yolo_detector/
│   │   ├── __init__.py
│   │   └── detector_node.py
│   ├── launch/
│   │   └── detector.launch.py
│   └── package.xml
│
├── perception_3d/          # 2D→3D 投影 + TF 变换
│   ├── perception_3d/
│   │   ├── __init__.py
│   │   └── projector_node.py
│   ├── launch/
│   │   └── perception.launch.py
│   └── package.xml
│
├── grasp_planner/          # 抓取规划 + MoveIt2 IK
│   ├── grasp_planner/
│   │   ├── __init__.py
│   │   └── planner_node.py
│   ├── launch/
│   │   └── planner.launch.py
│   └── package.xml
│
└── yolo_bringup/           # 顶层启动文件 + Gazebo 场景
    ├── launch/
    │   ├── bringup.launch.py    # ← 唯一入口（camera_source 决定后端）
    │   └── pipeline.launch.py   # ← 感知+抓取节点的唯一定义处
    ├── config/
    │   ├── ros_gz_bridge.yaml
    │   └── grasping.rviz
    ├── worlds/
    │   ├── color_objects_world.sdf
    │   └── block_grid_world.sdf
    └── package.xml
```

> ⚠️ 上面这棵树是**设计稿的设想**，与实际不一致：缺了 `arm_grasp_sim`
> （世界 + UR5 + 控制器 + MoveIt，仿真入口真正的持有者）、缺了
> `perception_3d`。`simulation.launch.py` 与 `moveit_config.yaml` **不存在**。
> 以 `ls ros2_ws/src/` 为准。

---

## 5. 消息与话题流

| 话题 | 消息类型 | 发布者 | 订阅者 |
|------|----------|--------|--------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | ros_gz_bridge | yolo_detector |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | ros_gz_bridge | perception_3d |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | ros_gz_bridge | perception_3d |
| `/detections` | `vision_msgs/Detection2DArray` | yolo_detector | perception_3d |
| `/target_3d` | `geometry_msgs/PoseStamped` | perception_3d | grasp_planner |
| `/grasp_pose` | `geometry_msgs/PoseStamped` | grasp_planner | MoveIt2 |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo / robot_state_publisher | MoveIt2, RViz |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | 所有节点 |

---

## 6. 安装与启动流程

### 6.1 系统依赖安装

```bash
# 1. 配置 ROS2 源（如使用国内镜像，将 archive.ubuntu.com 换为 mirrors.aliyun.com）
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

# 2. ROS2 Jazzy Desktop（注意：不要装 python3-colcon-common-extensions，与 Conda pip 版本冲突）
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base python3-rosdep

# 3. Gazebo Harmonic（通过 ros-jazzy-ros-gz-sim 自动拉取，无需单独 gz-harmonic）
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image

# 4. MoveIt2 + 感知依赖
sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-visual-tools \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-image-geometry ros-jazzy-tf2-geometry-msgs ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro

# 5. rosdep 初始化
sudo rosdep init && rosdep update
```

### 6.2 Python 依赖环境

> ⚠️ 本节原写的 `conda env create -f environment.yml` **已不可用**：`environment.yml`
> 已删除（第十轮）。原因有二——它硬编码 Python 3.12 / noble，与 Orin 的 3.10 / jammy
> 不符（D9）；且本项目真正使用的是**既有的** `robot-vision` 环境，不是本仓库创建的环境。
>
> 实际环境：conda 环境 `~/robotics/venvs/robot-vision`（Python 3.10.21，
> numpy 1.26.4 / torch 2.5.1+cu121 / ultralytics 8.3.99）。运行时叠加：
>
> ```bash
> source /opt/ros/humble/setup.bash
> export PYTHONPATH=~/robotics/venvs/robot-vision/lib/python3.10/site-packages:$PYTHONPATH
> ```
>
> ⚠️ 只在**运行期**叠加。构建期必须保持纯净 ROS 环境，否则触发 E2。

### 6.3 构建工作区

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.4 运行完整 Pipeline

```bash
# 单条命令即可，仿真与真机只差一个参数
ros2 launch yolo_bringup bringup.launch.py camera_source:=sim        # 仿真
ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense  # 真机
```

> ⚠️ 本节原写的三条命令**都不可用**：`simulation.launch.py` 已删除；
> `/trigger_grasp` 服务不存在（规划器是自主的，无需触发）；`pipeline.launch.py`
> 也不该在仿真里单独调用——世界与机械臂由 `arm_grasp_sim` 提供，单独起它
> 只会有节点没有臂。分步调试请用 `skip_planner:=true` / `skip_detector:=true`。

---

## 7. 最小可行实验（Milestone）

**目标**：Gazebo 中放置一个彩色方块，机械臂通过 YOLO 检测 → 3D 定位 → IK → 抓取。

**成功条件**：
1. YOLO 正确检测方块并发布 2D 框
2. 深度投影得到正确的 3D 坐标（误差 < 2cm）
3. TF 变换正确转换到 base_link 坐标系
4. MoveIt2 成功规划并执行抓取轨迹
5. 夹爪闭合抓住物体

---

## 8. 后续扩展方向

- 替换 MoveIt2 IK 为 cuRobo（GPU 加速）
- 加入抓取姿态生成网络（如 GraspNet）
- 支持多物体场景 + 语义推理
- 实物部署（RealSense 实感相机 + UR 真实机械臂）
- 加入力控/柔顺控制完成更稳定抓取

---

## 9. 已知问题与排查记录

### 9.1 清华镜像源 403

**现象**：`E: Failed to fetch http://mirrors.tuna.tsinghua.edu.cn/... 403 Forbidden`

**解决**：替换为阿里云镜像（Ubuntu 24.04 使用 deb822 格式）：

```bash
sudo sed -i 's|mirrors.tuna.tsinghua.edu.cn|mirrors.aliyun.com|g' /etc/apt/sources.list.d/ubuntu.sources
sudo apt update
```

### 9.2 CoppeliaSim Qt5 库冲突

**现象**：`version 'Qt_5.15' not found (required by libgz-sim8-gz.so)`

**原因**：`~/.bashrc` 中 `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT` 导致 CoppeliaSim 自带的旧版 Qt5 库覆盖了系统 Qt5。

**解决**：将 `~/.bashrc` 中的全局环境变量改为按需加载的 alias：

```bash
# 替换这三行：
# export COPPELIASIM_ROOT=...
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
# export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT

# 为这一行：
alias coppeliasim='$HOME/workspace/CoppeliaSim/CoppeliaSim_Edu_V4_1_0/coppeliaSim.sh'
```

**临时修复**（不关终端的情况下）：

```bash
QT_QPA_PLATFORM_PLUGIN_PATH= \
LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | sed 's|:/home/<user>/workspace/CoppeliaSim/CoppeliaSim_Edu_V4_1_0||g') \
gz sim -r src/yolo_bringup/worlds/color_objects_world.sdf
```

### 9.3 Python 版本不匹配（3.10 vs 3.12）

**现象**：`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`

**原因**：ROS2 Jazzy 的 rclpy 仅为 Python 3.12 编译。Conda 环境若使用 Python 3.10 则无法导入。

**解决**：Conda 环境必须使用 Python 3.12：

```yaml
# environment.yml
dependencies:
  - python=3.12
```

### 9.4 numpy 2.x 不兼容 ROS2 cv_bridge

**现象**：`A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

**原因**：ROS2 Jazzy 的 `cv_bridge` 和 `image_geometry` 使用 numpy 1.x ABI 编译，不兼容 numpy 2.x。

**解决**：在 Conda 环境中降级 numpy：

```bash
pip install "numpy<2" deprecated
```

### 9.5 Gazebo 启动命令

**注意**：Gazebo Harmonic 在 ROS2 Jazzy 下通过 vendor 包提供，不是独立的 `gz-harmonic` deb 包。可执行文件位于：

```
/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
```

需要先 `source /opt/ros/jazzy/setup.bash` 才能正确解析库路径。

---

## 10. 参考资料

- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic
- MoveIt2: https://moveit.picknik.ai/
- ultralytics YOLOv8: https://docs.ultralytics.com/
- cuRobo: https://curobo.org/
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros
