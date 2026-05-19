# YOLO + ROS2 视觉抓取系统 — 技术方案文档

> **项目目标**：在 Gazebo Harmonic 仿真环境中，实现 YOLO 视觉感知 → 3D 空间定位 → 逆运动学求解 → 机器人抓取的完整闭环。

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
| Python | 3.10 (Conda) | YOLO 与 ROS2 Python 节点 |
| CUDA | 13.2 | GPU 加速 YOLO 推理 |

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
    │   ├── simulation.launch.py
    │   └── pipeline.launch.py
    ├── config/
    │   ├── ros_gz_bridge.yaml
    │   └── moveit_config.yaml
    ├── worlds/
    │   └── grasping_world.sdf
    └── package.xml
```

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
# ROS2 Jazzy (Ubuntu 24.04)
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Gazebo Harmonic
sudo apt install -y gz-harmonic

# ROS-Gazebo 桥接
sudo apt install -y ros-jazzy-ros-gz

# MoveIt2
sudo apt install -y ros-jazzy-moveit

# 其他依赖
sudo apt install -y ros-jazzy-vision-msgs ros-jazzy-cv-bridge \
  ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs
```

### 6.2 Conda 环境

```bash
conda env create -f environment.yml
conda activate yolo_ros
```

### 6.3 构建工作区

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.4 运行完整 Pipeline

```bash
# 终端 1: 启动 Gazebo 仿真
ros2 launch yolo_bringup simulation.launch.py

# 终端 2: 启动感知+抓取流水线
ros2 launch yolo_bringup pipeline.launch.py

# 终端 3: 手动触发抓取
ros2 service call /trigger_grasp std_srvs/srv/Trigger
```

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

## 9. 参考资料

- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic
- MoveIt2: https://moveit.picknik.ai/
- ultralytics YOLOv8: https://docs.ultralytics.com/
- cuRobo: https://curobo.org/
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros
