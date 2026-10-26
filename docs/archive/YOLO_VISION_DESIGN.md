# yolo_vision — YOLO 视觉检测 ROS2 子项目

> 独立的 ROS2 视觉检测包，在 Gazebo Harmonic 仿真中实现 YOLO 实时目标检测与标注显示。  
> 通信架构参照 [Moveit2YoloObb](https://github.com/laoxue888/Moveit2YoloObb)，使用 ROS2 标准消息类型。

---

## 目录

1. [概述](#1-概述)
2. [架构设计](#2-架构设计)
3. [文件结构](#3-文件结构)
4. [通信拓扑](#4-通信拓扑)
5. [detector_node 详解](#5-detector_node-详解)
6. [Launch 文件](#6-launch-文件)
7. [构建与运行](#7-构建与运行)
8. [配置参数](#8-配置参数)
9. [数据集与模型](#9-数据集与模型)
10. [故障排查](#10-故障排查)

---

## 1. 概述

`yolo_vision` 是一个**独立的 ROS2 ament_python 包**，不依赖父项目的 `ros2_ws`，可单独构建、部署和运行。

### 核心能力

| 功能 | 实现 |
|------|------|
| 相机接入 | 通过 `ros_gz_bridge` 桥接 Gazebo RGB 相机 |
| YOLO 推理 | `ultralytics YOLOv8`，GPU 加速 (CUDA) |
| 检测发布 | 标准 `vision_msgs/Detection2DArray` |
| 可视化 | `/detections_image` 话题 + `rqt_image_view` 查看 |
| 仿真集成 | 内置 `block_grid_world.sdf` 5×5 方阵世界 |
| 一键启动 | `sim_detect.launch.py` 启动 Gazebo+Bridge+Detector |

### 与父项目的关系

```
yolo_ros/                        ← 父项目（ROS2 workspace + 训练脚本 + 文档）
├── yolo_vision/                 ← ★ 本子项目（独立 ROS2 包）
│   ├── detector_node.py         ← 核心检测节点
│   ├── worlds/block_grid_world.sdf
│   └── models/best.pt
├── ros2_ws/                     ← 旧 ROS2 工作区（不再依赖）
├── block_dataset/               ← 训练数据集
├── scripts/                     ← 训练/标注工具
└── docs/                        ← 文档
```

---

## 2. 架构设计

### 2.1 参照来源

通信架构直接参照 `Moveit2YoloObb/moveit2_obb/src/yolov8_obb/`：

| 参照项目 | 本包 | 变更理由 |
|----------|------|----------|
| `yolov8_obb_publisher.py` + `yolov8_obb_subscriber.py` 两个节点 | `detector_node.py` 单个节点 | 检测+画框一次完成，减少进程间通信 |
| 自定义 `yolov8_msgs` (OBB 多边形) | 标准 `vision_msgs/Detection2DArray` | 我们的方块用轴对齐框即可，无需 OBB |
| `/Yolov8_Inference` 话题 | `/detections` | ROS2 标准命名 |
| `/inference_result` (Image) | `/detections_image` | 语义更清晰的命名 |

### 2.2 设计原则

- **单节点**：一个进程完成订阅→推理→发布，降低延迟
- **标准消息**：用 `vision_msgs` 而非自定义消息，兼容 ROS2 生态工具
- **自包含**：包内自带世界文件、模型、启动配置
- **BEST_EFFORT QoS**：与 Gazebo 的 QoS 策略匹配，避免消息丢失

### 2.3 数据流

```
┌──────────────────────┐
│  Gazebo Harmonic      │
│  block_grid_world.sdf │
│                       │
│  RGBD 俯瞰相机         │
│  640×480 @ 15fps      │
└─────────┬─────────────┘
          │ gz.msgs.Image
          ▼
┌──────────────────────┐
│  ros_gz_bridge        │
│  GZ → ROS2 单向       │
│  /camera/color/image_raw
└─────────┬─────────────┘
          │ sensor_msgs/Image
          ▼
┌──────────────────────┐
│  yolo_detector 节点    │
│                       │
│  YOLOv8 推理          │
│  ├─ 2D 检测框         │
│  └─ 标注图像          │
│                       │
│  Pub: /detections     │  → vision_msgs/Detection2DArray
│  Pub: /detections_image│  → sensor_msgs/Image (带框)
└─────────┬─────────────┘
          │
          ▼
┌──────────────────────┐
│  可视化               │
│  rqt_image_view       │
│  /detections_image    │
└──────────────────────┘
```

---

## 3. 文件结构

```
yolo_vision/
├── package.xml                         # ROS2 包元数据
├── setup.py                            # Python 安装 + 入口点 + 数据文件
├── setup.cfg                           # Python 开发/安装选项
├── resource/
│   └── yolo_vision                     # ament 索引标记
├── config/
│   └── gz_bridge.yaml                  # ros_gz_bridge 参数化配置
├── launch/
│   ├── detect.launch.py                # 仅启动检测节点
│   └── sim_detect.launch.py            # ★ 一键启动：Gazebo+Bridge+Detector
├── worlds/
│   └── block_grid_world.sdf            # 5×5 方阵仿真世界
├── models/
│   └── best.pt                         # YOLOv8 训练权重 (mAP 99.5%)
└── yolo_vision/
    ├── __init__.py
    └── detector_node.py                # ★ 核心检测节点
```

**无冗余**：不含测试、demo、训练脚本——仅保留运行必需的 11 个文件。

---

## 4. 通信拓扑

### 4.1 话题一览

| 话题 | 类型 | 方向 | QoS | 说明 |
|------|------|------|-----|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 订阅 | BEST_EFFORT | Gazebo RGB 相机原始帧 |
| `/detections` | `vision_msgs/Detection2DArray` | 发布 | RELIABLE | YOLO 2D 检测框数组 |
| `/detections_image` | `sensor_msgs/Image` | 发布 | RELIABLE | 标注后的图像（BGR8） |

### 4.2 节点

| 节点 | 包 | 说明 |
|------|-----|------|
| `bridge_camera` | `ros_gz_bridge` | Gazebo 相机话题 → ROS2 话题 |
| `yolo_detector` | `yolo_vision` | YOLO 推理 + 结果发布 |

### 4.3 启动时序

```
t=0s    Gazebo 启动 (gz sim -r block_grid_world.sdf)
t=3s    Bridge 启动 (ros_gz_bridge)
t=5s    Detector 启动 (yolo_vision detector)
t=8s    模型加载完成 → YOLO Detector ready
t=8s+   /detections_image 开始发布 → rqt_image_view 可查看
```

> `sim_detect.launch.py` 使用 `TimerAction` 自动编排时序，无需手动等待。

---

## 5. detector_node 详解

### 5.1 类结构

```python
class DetectorNode(Node):
    def __init__(self):
        # 1. 参数声明：model, confidence, device
        # 2. 模型加载：ultralytics YOLO（优先包内 models/，支持绝对路径）
        # 3. 发布者：/detections (Detection2DArray), /detections_image (Image)
        # 4. 订阅者：/camera/color/image_raw (BEST_EFFORT QoS)

    def callback(self, msg: Image):
        # 1. cv_bridge: ROS Image → numpy (BGR)
        # 2. YOLO 推理
        # 3. 构建 Detection2DArray（归一化坐标）
        # 4. 发布检测结果
        # 5. 生成标注图像 → 发布 /detections_image
```

### 5.2 模型加载策略

```
1. 如果 model 参数是绝对路径 → 直接使用
2. 否则 → get_package_share_directory('yolo_vision')/models/ + model
3. 如果仍找不到 → 回退到项目根目录 runs/detect/block_grid_demo/weights/best.pt
```

### 5.3 坐标转换

YOLO 输出像素坐标 `(x1, y1, x2, y2)` → `vision_msgs/BoundingBox2D` 归一化坐标：

```python
center.position.x = (x1 + x2) / 2.0 / image_width
center.position.y = (y1 + y2) / 2.0 / image_height
size_x = (x2 - x1) / image_width
size_y = (y2 - y1) / image_height
```

> **注意**：ROS2 Jazzy 的 `vision_msgs/Pose2D` 使用 `center.position.x`（嵌套 Point2D），
> 而非旧版的 `center.x`。代码已适配 Jazzy。

### 5.4 QoS 策略

| 方向 | QoS | 原因 |
|------|-----|------|
| 相机订阅 | `BEST_EFFORT`, depth=1 | Gazebo 相机使用不可靠传输，匹配发送端 |
| 检测发布 | `RELIABLE`, depth=10 | 确保下游节点不丢失检测结果 |

---

## 6. Launch 文件

### 6.1 detect.launch.py

仅启动检测节点，适用于 Gazebo 和 Bridge 已在其他终端运行的场景。

```bash
ros2 launch yolo_vision detect.launch.py

# 参数覆盖
ros2 launch yolo_vision detect.launch.py model:=/path/to/custom.pt confidence:=0.7
```

参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model` | `best.pt` | 模型文件名或绝对路径 |
| `confidence` | `0.5` | 检测置信度阈值 |
| `device` | `cuda:0` | 推理设备 (cuda:0 / cpu) |

### 6.2 sim_detect.launch.py

一键启动完整流水线：Gazebo 仿真 + Bridge + 检测节点。

```bash
ros2 launch yolo_vision sim_detect.launch.py
ros2 launch yolo_vision sim_detect.launch.py world:=color_objects_world.sdf
```

参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `world` | `block_grid_world.sdf` | yolo_vision/worlds/ 中的世界文件 |
| `model` | `best.pt` | 模型文件 |
| `confidence` | `0.5` | 置信度阈值 |

时序编排（`TimerAction`）：

```
t=0s    ExecuteProcess: gz sim -r <world>
t=3s    Node: ros_gz_bridge parameter_bridge
t=5s    Node: yolo_vision detector
```

---

## 7. 构建与运行

### 7.1 环境要求

| 组件 | 版本 | 来源 |
|------|------|------|
| ROS2 | Jazzy | `/opt/ros/jazzy/setup.bash` |
| Python | 3.12 | Conda env `yolo_ros` |
| ultralytics | ≥8.0 | pip |
| Gazebo | Harmonic 8.11 | ros-jazzy-gz-sim-vendor |
| ros_gz_bridge | 1.0 | ros-jazzy-ros-gz-bridge |
| rqt_image_view | 1.3 | ros-jazzy-rqt-image-view |

### 7.2 构建

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
cd yolo_vision
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 使 Python 代码修改立即生效，无需重新构建。

### 7.3 分步启动

```bash
# 终端 1: Gazebo 方阵
source /opt/ros/jazzy/setup.bash
gz sim -r yolo_vision/worlds/block_grid_world.sdf

# 终端 2: Bridge
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
    /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# 终端 3: YOLO 检测
source /opt/ros/jazzy/setup.bash && conda activate yolo_ros
source yolo_vision/install/setup.bash
ros2 run yolo_vision detector

# 终端 4: 查看检测画面
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /detections_image
```

### 7.4 一键启动

```bash
source /opt/ros/jazzy/setup.bash && conda activate yolo_ros
source yolo_vision/install/setup.bash
ros2 launch yolo_vision sim_detect.launch.py

# 另开终端查看
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view /detections_image
```

### 7.5 运行验证

```bash
# 查看话题
ros2 topic list | grep -E "camera|detect"

# 查看检测帧率
ros2 topic hz /detections_image

# 查看节点
ros2 node list
# 输出: /bridge_camera, /yolo_detector

# 查看检测结果（JSON）
ros2 topic echo /detections --once
```

---

## 8. 配置参数

所有参数通过 ROS2 parameter 机制设置：

```bash
# 命令行覆盖
ros2 run yolo_vision detector --ros-args -p model:=yolov8n.pt -p confidence:=0.3

# launch 文件覆盖
ros2 launch yolo_vision detect.launch.py model:=yolov8n.pt confidence:=0.7 device:=cpu
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model` | string | `best.pt` | 模型路径（相对于包内 models/，或绝对路径） |
| `confidence` | double | `0.5` | YOLO 置信度阈值（低于此值的结果被丢弃） |
| `device` | string | `cuda:0` | 推理设备：`cuda:0` / `cpu` / `cuda:1` |

---

## 9. 数据集与模型

### 9.1 方阵世界（`block_grid_world.sdf`）

```
5×5 = 25 方块，排列于 80×60cm 桌面：

         x=0.18  x=0.32  x=0.46  x=0.60  x=0.74
y=-0.20    ■       ■       ■       ■       ■     红系 (5种深浅)
y=-0.10    ■       ◯       ■       ■       ■     蓝系
y= 0.00    ■       ■       ■       ◯       ■     绿系
y= 0.10    ■       ■       ◯       ■       ■     橙系
y= 0.20    ■       ■       ■       ■       ■     紫系

■ 正方体 (22)   ◯ 圆柱体 (3)
方块 3cm | 桌面 75cm 高 | RGBD 相机 1.5m 俯瞰
```

### 9.2 模型（`best.pt`）

| 属性 | 值 |
|------|-----|
| 基座模型 | YOLOv8n (3M 参数) |
| 训练集 | 100 张合成方阵图像 + 自动标注 |
| mAP50 | 0.995 |
| mAP50-95 | 0.983 |
| 推理速度 | ~19ms/帧 (RTX 2060) |
| 模型大小 | 6.2 MB |
| 类别数 | 1 (`block`) |

### 9.3 重新生成世界

修改父项目中 `scripts/generate_grid_world.py` 的参数，运行后复制到 `yolo_vision/worlds/`。

### 9.4 重新训练模型

参见 `docs/YOLO_TUTORIAL.md` 第 3-5 节。

---

## 10. 故障排查

### 10.1 Gazebo 窗口空白

WSL2/WSLg 下 OGRE2 渲染的已知问题。仿真核心正常，不影响相机发布。

**验证**：`gz topic -e -t /camera/color/image_raw -n 1` 有输出即正常。

### 10.2 rqt_image_view 无画面或黑屏

1. 确认 detector 已输出 `YOLO Detector ready`
2. 确认 bridge 日志有 `Creating GZ->ROS Bridge`
3. 确认话题存在：`ros2 topic list | grep detect`

如果是 WSL2 下 OpenCV 窗口黑屏：`rqt_image_view` 使用 Qt 渲染，不受此问题影响。

### 10.3 Bridge 语法错误

```bash
# ✓ 正确：GZ→ROS2 单向
/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# ✗ 错误：双向模式会在 ROS→GZ 方向阻塞
/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### 10.4 AttributeError: 'Pose2D' object has no attribute 'x'

ROS2 Jazzy 的 `vision_msgs/Pose2D` 使用 `position.x`。已修复，重新 `colcon build` 即可。

### 10.5 conda 环境找不到 rclpy

顺序很重要：**先 source ROS2，再 conda activate**。

```bash
source /opt/ros/jazzy/setup.bash    # ← 先
conda activate yolo_ros              # ← 后
```

### 10.6 两个 Gazebo 实例冲突

`sim_detect.launch.py` 会启动新 Gazebo。如已有实例在运行，先 `pkill gz` 再重新 launch。

---

## 附录：与参照项目的完整对照

| 维度 | Moveit2YoloObb | yolo_vision |
|------|----------------|-------------|
| **检测类型** | OBB (旋转框) | 轴对齐框 |
| **检测节点** | publisher + subscriber 分开 | 单节点 |
| **消息格式** | 自定义 `yolov8_msgs` | 标准 `vision_msgs` |
| **可视化** | GraphExecuter (Qt) | rqt_image_view (ROS2 标准) |
| **仿真对象** | Panda 机械臂 + 螺丝/方块 | 5×5 方块方阵 |
| **世界文件** | `arm_on_the_table.sdf` (SDF 1.6) | `block_grid_world.sdf` (SDF 1.11) |
| **相机** | `/image_raw` (需外部 bridge) | `/camera/color/image_raw` (内置 bridge) |
| **模型** | OBB 训练权重 | YOLOv8n 标准检测权重 |
| **启动方式** | `ros2 launch yolov8_obb yolov8_obb.launch.py` | `ros2 launch yolo_vision sim_detect.launch.py` |
