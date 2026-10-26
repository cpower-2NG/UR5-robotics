# YOLO 目标检测：训练与使用指南

> 从训练 YOLO 模型到 Gazebo 仿真中使用的完整工作流。

---

## 1. 概述

本项目用 YOLO 检测 Gazebo 仿真中的物体（当前：螺丝/螺栓，目标：立方体方块）。
检测结果通过 ROS2 topic 传给下游的 3D 感知和抓取规划节点。

### 数据流

```
Gazebo 仿真                    YOLO 检测                    3D 感知
    │                             │                           │
    ├─ /camera/color/image_raw ──→ yolo_detector              │
    │                             │                           │
    │                         YOLOv8 推理                     │
    │                             │                           │
    │                         /detections ────────────────────→ perception_3d
    │                        (vision_msgs/Detection2DArray)    │
    │                                                         ↓
    │                                                    深度图 → 3D 坐标
    │                                                         │
    │                                                    /target_3d → grasp_planner
```

### 关键 topic

| topic | 类型 | 方向 |
|-------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo → YOLO |
| `/detections` | `vision_msgs/Detection2DArray` | YOLO → perception_3d |
| `/detections_image` | `sensor_msgs/Image` | YOLO → 可视化 |

---

## 2. 文件结构

```
yolo_ros/
├── block_dataset/                  ← 训练数据集（已创建）
│   ├── data.yaml                   ← YOLO 配置（类别定义）
│   ├── train/images/               ← 采集的截图
│   ├── train/labels/               ← labelImg 生成的标注
│   └── val/                        ← 验证集（自动拆分）
│
├── scripts/
│   ├── collect_training_data.py    ← 采集工具
│   └── train_yolo_block.py         ← 训练脚本
│
├── yolov8n.pt                      ← 当前使用的模型（会被 block_best.pt 替换）
├── block_best.pt                   ← 训练输出的方块模型
│
└── ros2_ws/src/yolo_detector/      ← ROS2 检测节点
    └── yolo_detector/
        └── detector_node.py        ← 推理节点（不改）
```

---

## 3. 训练流程（三步）

### 步骤一：采集图像

**前提**：Gazebo 仿真正在运行，相机话题有数据。

```bash
# 1. 启动 Gazebo 仿真（参照项目）
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
source ~/workspace/yolo_ros/Moveit2YoloObb/moveit2_obb/install/setup.bash
ros2 launch panda_moveit_config moveit_gazebo_obb.py

# 2. 新终端：启动采集工具
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
python3 scripts/collect_training_data.py
```

**操作**：
- 在 Gazebo 中**移动/旋转/增减物体**，改变视角和光照
- `Space` — 保存当前帧到 `block_dataset/train/images/`
- `S` — 开启自动连续采集（0.5s/张）
- `Q` — 退出

**目标**：200-500 张，覆盖不同物体位姿、光照角度、桌面位置。

> 如果 ROS2 不可用，脚本会自动回退到本地摄像头模式。

---

### 步骤二：标注

```bash
pip install labelImg
labelImg block_dataset/train/images  block_dataset/data.yaml
```

**快捷键**：

| 操作 | 按键 |
|------|------|
| 画框 | `W` |
| 保存 | `Ctrl+S` |
| 下一张 | `D` |
| 上一张 | `A` |
| 切换标注/查看模式 | `Ctrl+E` |

**标注规范**：
- 框紧贴物体边缘，包含全部可见部分
- 被遮挡的物体也要标注可见区域
- 类别名输入 `block`

每张图标注完成后会自动在 `block_dataset/train/labels/` 生成同名 `.txt`。

---

### 步骤三：训练

```bash
python3 scripts/train_yolo_block.py
```

脚本执行流程：
1. 检查标注进度（提示有多少张已标注）
2. 随机抽取 20% 图像到 `val/` 作为验证集
3. 加载 `yolov8n.pt` 预训练权重
4. 训练 100 epoch（20 epoch 无提升则早停）
5. 输出 `block_best.pt`
6. **自动替换** `yolov8n.pt`（备份原文件为 `yolov8n_backup.pt`）

**训练输出**：
```
runs/block_detect/
└── weights/
    ├── best.pt     ← 最佳模型
    └── last.pt     ← 最后一轮
```

---

## 4. 在仿真中使用

### 启动完整管线

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros

# 构建 ROS2 工作区
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 启动（Gazebo + YOLO + perception_3d + grasp_planner）
ros2 launch yolo_bringup bringup.launch.py
```

### 仅启动感知管线（仿真已运行）

```bash
ros2 launch yolo_bringup pipeline.launch.py
```

### 切换模型

训练完成后，`train_yolo_block.py` 会自动替换 `yolov8n.pt`。
如需手动切换：

```bash
# 使用方块模型
cp block_best.pt yolov8n.pt

# 恢复原模型
cp yolov8n_backup.pt yolov8n.pt
```

或者通过 launch 参数指定模型：

```bash
ros2 launch yolo_bringup pipeline.launch.py model:=block_best.pt
```

---

## 5. 参照项目 vs 本项目

| | 参照项目 (Moveit2YoloObb) | 本项目 (yolo_ros) |
|---|---|---|
| **YOLO 节点** | `yolov8_obb_publisher.py` | `yolo_detector/detector_node.py` |
| **消息类型** | 自定义 `Yolov8Inference` | 标准 `vision_msgs/Detection2DArray` |
| **检测方式** | OBB（带方向框） | 标准 bounding box |
| **模型** | `best.pt`（螺丝） | 当前 `yolov8n.pt`，将替换为 `block_best.pt` |
| **相机话题** | `/image_raw` | `/camera/color/image_raw` |
| **2D→3D** | GUI 硬编码针孔 | `perception_3d` 深度图投影 |
| **物体选择** | PyQt5 GUI 手动点 | 自动取第一个检测 |

**参照项目仅用于参考其 MoveIt2 抓取逻辑**，YOLO 部分使用本项目自己的 `yolo_detector` 包。

---

## 6. 验证检测效果

```bash
# 查看检测话题
ros2 topic echo /detections --once

# 查看标注图像（可视化检测框）
ros2 run rqt_image_view rqt_image_view /detections_image

# 查看节点状态
ros2 node list | grep yolo
```

---

## 7. 常见问题

### 采集时看不到图像
- 确认 Gazebo 仿真正在运行
- 检查相机话题：`ros2 topic list | grep image_raw`

### labelImg 打开后无图像
- 确认路径正确：`ls block_dataset/train/images/` 有 jpg 文件
- 如果 labelImg 崩溃，尝试 `pip install --upgrade labelImg`

### 训练时 CUDA out of memory
- 减小 batch size：`batch=8`（在 `train_yolo_block.py` 中修改）
- 或使用 CPU：`device=cpu`

### 检测不到物体
- 检查置信度阈值参数：默认 0.5，可下调到 0.3
- 确认模型文件路径正确
- 查看 `/detections_image` 可视化确认 YOLO 推理结果
