# YOLO 方阵检测 — 完整教程

> 在 Gazebo Harmonic 仿真中训练并运行 YOLO 方块检测的完整指南。

---

## 目录

1. [环境概览](#1-环境概览)
2. [启动方阵仿真](#2-启动方阵仿真)
3. [采集训练数据](#3-采集训练数据)
4. [自动标注](#4-自动标注)
5. [训练 YOLO 模型](#5-训练-yolo-模型)
6. [实时检测](#6-实时检测)
7. [ROS2 节点方式（推荐）](#7-ros2-节点方式推荐)
8. [常见问题](#8-常见问题)

---

## 1. 环境概览

### 1.1 架构

```
Gazebo Harmonic (方阵世界)          ROS2 Bridge            YOLO 检测
┌─────────────────────────┐    ┌──────────────┐    ┌──────────────┐
│  RGBD 俯瞰相机            │    │ ros_gz_bridge│    │ yolo_detector│
│  /camera/color/image_raw │───▶│  GZ → ROS2   │───▶│ YOLOv8 推理   │
│  5×5 方块方阵 (25个)      │    └──────────────┘    │ /detections   │
└─────────────────────────┘                         │ /detections_image│
                                                    └──────────────┘
```

### 1.2 所需组件

| 组件 | 位置 | 说明 |
|------|------|------|
| Gazebo | `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` | Harmonic 8.11 |
| 方阵世界 | `ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf` | 5×5 方块 |
| YOLO 模型 | `runs/detect/block_grid_demo/weights/best.pt` | mAP 99.5% |
| Conda 环境 | `yolo_ros` | Python 3.12 + ultralytics |

### 1.3 激活环境

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
```

---

## 2. 启动方阵仿真

### 2.1 世界文件

方阵世界文件：`ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf`

布局：
```
         x=0.18  x=0.32  x=0.46  x=0.60  x=0.74
y=-0.20    ■       ■       ■       ■       ■     红系 (5种深浅)
y=-0.10    ■       ◯       ■       ■       ■     蓝系
y= 0.00    ■       ■       ■       ◯       ■     绿系
y= 0.10    ■       ■       ◯       ■       ■     橙系
y= 0.20    ■       ■       ■       ■       ■     紫系

■ = 正方体 (22个)   ◯ = 圆柱体 (3个)   共 25 个方块
```

相机：桌面正上方 1.5m，垂直下视，640×480 @ 15fps。

### 2.2 启动命令

```bash
# 终端 1：启动 Gazebo
source /opt/ros/jazzy/setup.bash
gz sim -r ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf
```

### 2.3 修改方阵参数

编辑 `scripts/generate_grid_world.py`，修改顶部配置变量后重新生成：

```python
GRID_ROWS = 5          # 行数
GRID_COLS = 5          # 列数
BLOCK_SIZE = 0.03      # 方块尺寸 (米)
X_SPACING = 0.14       # 列间距 (米)
Y_SPACING = 0.10       # 行间距 (米)
CYLINDER_POSITIONS = {(1,1), (2,3), (3,2)}  # 圆柱位置
```

```bash
python3 scripts/generate_grid_world.py
```

---

## 3. 采集训练数据

### 3.1 Bridge 桥接（必须先启动）

```bash
# 终端 2：启动 ros_gz_bridge
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
    /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

> **关键**：`[` 表示 GZ→ROS2 单向，不是 `@`（双向）。

### 3.2 采集图像

```bash
# 终端 3
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
python3 scripts/collect_training_data.py
```

操作：
| 按键 | 功能 |
|------|------|
| `Space` | 保存当前帧 |
| `S` | 连续模式（每 0.5 秒自动保存） |
| `Q` / `ESC` | 退出 |

图像保存到 `block_dataset/train/images/`。

---

## 4. 自动标注

由于方阵中每个方块的世界坐标是已知的，通过相机投影可直接计算 2D 边界框，**无需手动标注**。

### 4.1 原理

```
世界坐标 (wx, wy, wz) → 相机坐标 (Xc, Yc, Zc) → 像素坐标 (u, v)
                                              ↓
                                    方块 8 顶点投影
                                              ↓
                                    取外接矩形 → YOLO 标注
```

### 4.2 执行

```bash
python3 scripts/auto_label_grid.py
```

标注文件输出到 `block_dataset/train/labels/`，每张图像 25 个标注框。

### 4.3 手动标注（备用）

如需对非方阵数据标注：

```bash
python3 scripts/label_blocks.py
```

操作：`Alt+拖动` 画框，`D` 下一张，`A` 上一张。

---

## 5. 训练 YOLO 模型

### 5.1 数据集结构

```
block_dataset/
├── data.yaml          # 数据集配置
├── train/
│   ├── images/        # 训练图像 (.jpg)
│   └── labels/        # YOLO 标注 (.txt)
└── val/
    ├── images/        # 验证图像
    └── labels/        # 验证标注
```

`data.yaml`：
```yaml
path: block_dataset
train: train/images
val: val/images
names:
  0: block
nc: 1
```

### 5.2 训练命令

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
python3 scripts/train_yolo_block.py
```

训练参数（可在 `train_yolo_block.py` 中调整）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| model | yolov8n.pt | YOLOv8 Nano 预训练权重 |
| epochs | 100 | 训练轮数 |
| imgsz | 640 | 输入尺寸 |
| batch | 16 | 批次大小 |
| device | cuda:0 | GPU/CPU |

### 5.3 快速演示（无 Gazebo）

不需要 Gazebo，用合成图像快速验证流水线：

```bash
conda activate yolo_ros
python3 scripts/demo_pipeline.py
```

此脚本生成 100 张合成方阵图像 → 自动标注 → 训练 → 验证。在 RTX 2060 上约 2 分钟完成，mAP50 可达 99.5%。

### 5.4 训练输出

```
runs/detect/block_grid_demo/
├── weights/
│   ├── best.pt         # ★ 最佳模型（用于推理）
│   └── last.pt         # 最后一轮
├── results.png         # 训练曲线
└── confusion_matrix.png
```

---

## 6. 实时检测

### 6.1 快速截图检测

捕捉一帧 → YOLO 推理 → 保存标注图像：

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
python3 scripts/capture_and_detect.py
```

输出：`runs/block_detect/capture_detect.jpg`（用任意图片查看器打开）。

### 6.2 实时窗口检测

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
python3 scripts/yolo_detect_simple.py \
    --model runs/detect/block_grid_demo/weights/best.pt
```

> **注意**：OpenCV `imshow` 在 WSL2 下可能渲染异常（黑屏）。如遇此问题，改用截图方式（6.1）。

---

## 7. ROS2 节点方式（推荐）

参照 `Moveit2YoloObb` 项目的标准做法：用 ROS2 节点发布标注图像，通过 RViz2 查看。

### 7.1 构建工作区

```bash
source /opt/ros/jazzy/setup.bash
conda activate yolo_ros
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 7.2 启动流水线

```bash
# 终端 1：Gazebo
gz sim -r ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf

# 终端 2：Bridge
ros2 run ros_gz_bridge parameter_bridge \
    /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# 终端 3：YOLO 检测节点
source install/setup.bash
conda activate yolo_ros
ros2 run yolo_detector detector_node \
    --ros-args -p model:=runs/detect/block_grid_demo/weights/best.pt

# 终端 4：查看标注图像（RViz2 或 rqt）
ros2 run rqt_image_view rqt_image_view /detections_image
```

### 7.3 话题说明

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | GZ→ROS2 | Gazebo RGB 相机 |
| `/detections` | `vision_msgs/Detection2DArray` | yolo_detector 发布 | 2D 检测框数组 |
| `/detections_image` | `sensor_msgs/Image` | yolo_detector 发布 | 标注后的图像（可直接查看） |

### 7.4 一键启动

```bash
ros2 launch yolo_bringup bringup.launch.py world:=block_grid_world.sdf
```

---

## 8. 常见问题

### Q: Gazebo 窗口空白？

A: WSL2/WSLg 下 OGRE2 渲染的已知问题。仿真核心功能正常（相机、物理均在运行）。世界文件已移除不兼容的 `<gui>` 元素，exit code 为 0 即正常。

### Q: 检测窗口黑屏？

A: OpenCV `cv2.imshow` 在 WSL2 下渲染不稳定。解决方法：
1. 使用 `capture_and_detect.py` 截图后查看
2. 使用 ROS2 节点 + `rqt_image_view` 或 RViz2
3. 使用 matplotlib 替代 OpenCV 显示

### Q: Bridge 不工作？

A: 确保 syntax 正确——单向用 `[`，不是 `@`：
```bash
# ✓ 正确（GZ → ROS2 单向）
... /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# ✗ 错误（双向，ROS→GZ 方向无发布者会导致异常）
... /camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### Q: rclpy 找不到？

A: 确保先 `source /opt/ros/jazzy/setup.bash` 再 `conda activate yolo_ros`。顺序不能反——conda 需要继承 ROS2 的环境变量。

### Q: 想用更多方块/不同布局？

A: 编辑 `scripts/generate_grid_world.py` 中的配置参数，运行即可重新生成 SDF。

---

## 文件索引

| 文件 | 用途 |
|------|------|
| `scripts/generate_grid_world.py` | 方阵世界生成器 |
| `scripts/auto_label_grid.py` | 自动标注（相机投影） |
| `scripts/collect_training_data.py` | Gazebo 图像采集 |
| `scripts/label_blocks.py` | 手动标注工具（备用） |
| `scripts/train_yolo_block.py` | YOLO 训练脚本 |
| `scripts/demo_pipeline.py` | 端到端演示（合成数据） |
| `scripts/capture_and_detect.py` | 单帧截图+检测 |
| `scripts/yolo_detect_simple.py` | 实时窗口检测 |
| `ros2_ws/src/yolo_detector/` | ROS2 检测节点（标准方式） |
| `ros2_ws/src/yolo_bringup/worlds/` | Gazebo 世界文件 |
| `block_dataset/data.yaml` | 数据集配置 |
| `docs/GRID_ENV_DESIGN.md` | 方阵环境设计文档 |
