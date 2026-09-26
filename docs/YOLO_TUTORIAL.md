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
│  3×3 方块网格 (9个)       │    └──────────────┘    │ /detections   │
└─────────────────────────┘                         │ /detections_image│
                                                    └──────────────┘
```

### 1.2 所需组件

| 组件 | 位置 | 说明 |
|------|------|------|
| Gazebo | `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` | Harmonic 8.11 |
| 方阵世界 | `ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf` | 3×3 网格，9 个方块 |
| YOLO 模型 | `block_best.pt`（仓库根） | 1 类 `block` 方块检测器 |
| Conda 环境 | `/home/<user>/robotics/venvs/robot-vision` | torch 2.5.1+cu121 + ultralytics 8.3.99 |

### 1.3 激活环境

```bash
source /opt/ros/jazzy/setup.bash
conda activate /home/<user>/robotics/venvs/robot-vision
```

> ⚠️ **本教程旧版通篇写的 `conda activate yolo_ros` 是错的**：本机并不存在名为
> `yolo_ros`（也不存在 `yolo_coco` / `yolo_vision`）的环境。实际带 torch + ultralytics
> 的是 **`robot-vision`**（torch 2.5.1+cu121 / ultralytics 8.3.99），位于
> `/home/<user>/robotics/venvs/`。
>
> 这些是 **conda prefix 环境**，`conda activate robot-vision` 会报
> `EnvironmentNameNotFound`——**必须写全路径**。
>
> 上表路径是**工作站**的；**Orin 上的环境名/路径尚未确认**，部署时需另行核对。

---

## 2. 启动方阵仿真

### 2.1 世界文件

方阵世界文件：`ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf`

布局（当前提交的世界是 `--fixed` 3×3 网格；`block_grid_world_meta.json` 记 `seed=824517619`）：

```
         x=0.34   x=0.50   x=0.66
y=-0.12    ■        ■        ■      第 0 行 (红系)
y= 0.00    ■        ◯        ■      第 1 行 (蓝系)
y= 0.12    ■        ■        ■      第 2 行 (绿系)

■ = 正方体 (8个)   ◯ = 圆柱体 (1个)   共 9 个方块，边长 0.04 m
```

相机：桌面（`z=0.765`）正上方 1.5 m，垂直下视，640×480，水平 FOV 60°。

### 2.2 启动命令

```bash
# 终端 1：启动 Gazebo
source /opt/ros/jazzy/setup.bash
gz sim -r ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf
```

### 2.3 修改方阵参数

`generate_grid_world.py` 有**两种模式**：

| 模式 | 命令 | 布局 |
|---|---|---|
| 随机散布（**默认**） | `python3 scripts/generate_grid_world.py --num_blocks 9 --pos_jitter 0.06` | N 个方块随机放置、互不重叠，随机分配颜色与形状（1~3 个圆柱） |
| 固定网格（`--fixed`，调试用） | `python3 scripts/generate_grid_world.py --fixed` | 3×3 |

> ⚠️ **当前提交的 `block_grid_world.sdf` 是 `--fixed` 产物**（`meta.json` 里 `fixed_mode: true`，
> `seed=824517619`），而脚本默认是随机模式。不带 `--seed` 重新生成会得到**另一个场景**；
> 要复现当前世界请用 `--fixed`，要复现某个随机场景请带上 `--seed`。

固定网格模式的参数在顶部常量区：

```python
BLOCK_SIZE = 0.04                # 方块边长 (米) —— 仅此一处为真值
# generate_fixed_grid() 内:
GRID_ROWS, GRID_COLS = 3, 3      # 行/列数
X_SPACING = 0.16                 # 列间距 (米)
Y_SPACING = 0.12                 # 行间距 (米)
CYLINDER_POSITIONS = {(1, 1)}    # 圆柱位置 (行, 列)
```

> ⚠️ 本教程旧版此处写的是 `BLOCK_SIZE = 0.03` / 5×5 —— 与实际生成器（0.04 / 3×3）**不符**，
> 已按 `generate_grid_world.py` 的真值更正。改尺寸后记得同步
> `auto_label_grid.py` 与 `generate_synthetic_data.py` 里的 `BLOCK_SIZE`，否则标注框会错位。

```bash
python3 scripts/generate_grid_world.py --fixed
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
conda activate /home/<user>/robotics/venvs/robot-vision
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

标注文件输出到 `block_dataset/train/labels/`，每张图像对应世界里的方块数（当前世界为 9 个）。

> ℹ️ `auto_label_grid.py` 的 `BLOCK_SIZE` 必须与 `generate_grid_world.py` 一致（当前均为 `0.04`），
> 否则投影出来的标注框会整体偏大/偏小。

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
conda activate /home/<user>/robotics/venvs/robot-vision
python3 scripts/train_yolo_block.py
```

训练参数（可在 `train_yolo_block.py` 中调整）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| model | `yolov8n_coco.pt` | 微调起点 = 真 COCO yolov8n（**不要写 `yolov8n.pt`**，见 §6.1 注） |
| epochs | 100 | 训练轮数 |
| imgsz | 640 | 输入尺寸 |
| batch | 16 | 批次大小 |
| device | cuda:0 | GPU/CPU |

### 5.3 快速演示（无 Gazebo）

不需要 Gazebo，用合成图像快速验证流水线：

```bash
conda activate /home/<user>/robotics/venvs/robot-vision
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

> ⚠️ **本节已修正**：原先引用的 `scripts/capture_and_detect.py` 与 `scripts/yolo_detect_simple.py`
> **在本仓库中并不存在**（详见 `docs/RESTRUCTURE_PLAN.md` D10）。下面改用**实际存在**的入口。

### 6.1 无 ROS 的逻辑自检

不依赖 ROS2 / Gazebo，一次性验证三件事：模型能否加载、2D→3D 投影数学、深度中值滤波。

```bash
python3 scripts/test_minimal.py
```

> ✅ 该脚本现在**显式加载仓库根的 `block_best.pt`**（1 类 `block`）。因此
> **看到「类别数: 1」是预期的**，不代表环境坏了——它验证的正是本项目实际部署的方块模型。
>
> 旧版此处写的是 `YOLO("yolov8n.pt")` 却打印「COCO 类别示例」：那个名字在仓库根其实
> 指向方块模型，真 COCO 被挤到 `yolov8n_backup.pt`（现已改名 `yolov8n_coco.pt`），
> 这正是 D6「同名不同物」。该名字已删除，脚本也已改为显式路径。

### 6.2 实时检测（走 ROS2 节点）

本项目实时检测的正规路径是 ROS2 节点 + RViz2，见 §7：

```bash
ros2 launch yolo_detector detector.launch.py
```

> 原「OpenCV `imshow` 实时窗口」脚本已不存在；因此 WSL2 下 `imshow` 黑屏的注意事项也不再适用。

---

## 7. ROS2 节点方式（推荐）

参照 `Moveit2YoloObb` 项目的标准做法：用 ROS2 节点发布标注图像，通过 RViz2 查看。

### 7.1 构建工作区

```bash
source /opt/ros/jazzy/setup.bash
conda activate /home/<user>/robotics/venvs/robot-vision
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
conda activate /home/<user>/robotics/venvs/robot-vision
ros2 run yolo_detector detector_node \
    --ros-args -p model:=block_best.pt

# 注：detector_node 会从 CWD 逐级上溯查找该文件，故只要在仓库内启动即可命中
#     仓库根的 block_best.pt。找不到时它会**报错退出**，不会静默联网下载 COCO。

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
1. 使用 ROS2 节点 + `rqt_image_view` 或 RViz2（推荐，见 §6.2 / §7）
2. 把结果写成图片文件再查看（不要在窗口里显示）
3. 使用 matplotlib 替代 OpenCV 显示

> 原先第 1 条提到的 `capture_and_detect.py` 在本仓库中不存在（D10），已移除。

### Q: Bridge 不工作？

A: 确保 syntax 正确——单向用 `[`，不是 `@`：
```bash
# ✓ 正确（GZ → ROS2 单向）
... /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# ✗ 错误（双向，ROS→GZ 方向无发布者会导致异常）
... /camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### Q: rclpy 找不到？

A: 确保先 `source /opt/ros/jazzy/setup.bash` 再 `conda activate /home/<user>/robotics/venvs/robot-vision`。顺序不能反——conda 需要继承 ROS2 的环境变量。

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
| `scripts/test_minimal.py` | 最小验证测试（YOLO + 3D 投影 + 深度滤波，无需 ROS） |
| `scripts/move_block.py` | 移动仿真方块（`set_pose` 服务） |
| `ros2_ws/src/yolo_detector/` | ROS2 检测节点（标准方式） |
| `ros2_ws/src/yolo_bringup/worlds/` | Gazebo 世界文件 |
| `block_dataset/data.yaml` | 数据集配置 |
| `docs/GRID_ENV_DESIGN.md` | 方阵环境设计文档 |
