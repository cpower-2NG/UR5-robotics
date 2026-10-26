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
| 固定网格（`--fixed`，调试用） | `python3 scripts/generate_grid_world.py --fixed` | 3×3，**一行一类** |

两者都接受 `--classes`（逗号分隔，默认 `red,orange,yellow,cyan,purple`）——
类别名与类别号会一并写进 meta，供自动标注与建数据集读取（见 §4.4 多类别）。
颜色取值集中在 `scripts/block_classes.py` 的 `PALETTE`，生成器不再自带一份颜色池。
> ⚠️ 旧的颜色池里有成对的**同色相、只差明度**的颜色（蓝/钢蓝、绿/深绿、橙/深橙），
> 那种对子在检测端本来就分不开 —— 现在只取**色相分得开**的颜色。

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
python3 scripts/label_blocks.py --classes red,yellow,cyan
```

操作：`Alt+拖动` 画框（归**当前类别**），`1..9` 切类别，`D` 下一张，`A` 上一张，
`Backspace` 撤销。启动横幅会打出当前的类别表；重新打开已有的标签会**读回类别号**
（不会抹成 0 —— 自检 `test_label_blocks.py` 专门钉住这一点）。

### 4.4 多类别（分拣用）

方块按**颜色**分类，类别表由 `scripts/block_classes.py` **单点定义**（生成器、标注器、
建数据集、手工标注四处都从它取，不再各写一份）。固定网格模式下**一行一类**：

```bash
python3 scripts/generate_grid_world.py --fixed --classes red,yellow,cyan
python3 scripts/auto_label_grid.py            # 类别号从 meta 里读，不必再传
python3 scripts/build_dataset.py --class-names red,yellow,cyan
```

> ⚠️ **类别号 = 类别表里的下标**。同一个列表、同一个顺序要贯穿
> 生成器 → 标注 → 建数据集 → 权重文件；顺序变了**标签不会报错，只是语义全反**
> （每个号仍然落在 `[0, nc)` 里，所以没有一处会报警）。规划器侧的
> `place_slot_classes` 因此用**类别名**而不是类别号 —— 名字对不上是看得见的。
>
> ⚠️ 调色板只有 8 个颜色名（见 `block_classes.PALETTE`），手工标注工具的切换键
> 只有 `1..9`。选类别时以**实测色相**为准：`block_classes.py` 的文档里记着
> 上一批方块的实测直方图（红 ~5°、暖色 15–60°、青 ~190°、紫 300–325°，
> **65°–180° 是空的 —— 没有绿色**）。那批数据来自前负责人，你自己的方块要重新拍一份。
>
> **别猜类别数**，用尺子量 —— `scripts/palette_survey.py`（离线看图，不连机器人）：
>
> ```bash
> python3 scripts/palette_survey.py --images ~/block_photos --out /tmp/palette.png
> ```
>
> 它自己给结论：每个色团按「**图内展开 vs 跨图漂移**」判一遍
> （展开 ≫ 漂移 ⇒ 里面藏着多种颜色，该拆），最后打出建议类别数与可直接粘的
> `--classes …`。复核图**一定要打开看** —— 多出来的色团是积木还是旁边的水果，
> 只有看图才知道。命令与拍法见 `docs/待执行命令.md` 的 **A2** 一节。

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
| `scripts/block_classes.py` | **类别表的单点定义**（调色板 + 校验 + 颜色名↔类别号） |
| `scripts/palette_survey.py` | **量现场有几种颜色** —— 定类别数的尺子（离线看图，不连机器人） |
| `scripts/generate_grid_world.py` | 方阵世界生成器（`--classes` 决定类别表） |
| `scripts/auto_label_grid.py` | 自动标注（相机投影，类别号从 meta 读） |
| `scripts/build_dataset.py` | 建数据集（`--class-names` 写进 data.yaml 与 manifest） |
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
