# 方阵仿真训练环境 — 设计与流程文档

> **目标**：搭建一个系统化的方块方阵仿真环境，用于高效训练 YOLO 视觉检测模型，
> 并构建完整的「仿真 → 采集 → 自动标注 → 训练 → 部署」流水线。

---

## 1. 设计背景与动机

### 1.1 原有方案的局限

参照项目 `Moveit2YoloObb` 的 `arm_on_the_table.sdf` 仅在桌面上放置 **3 个散落方块**（红/绿/蓝），
每张图像只能提供 3 个训练标注框。手工标注成本高，数据效率低。

### 1.2 方阵方案的核心优势

| 维度 | 原有方案 | 方阵方案 |
|------|----------|----------|
| 单图标注框数 | 3 | **25**（8 倍提升） |
| 标注方式 | 手动 labelImg | **自动投影标注**（零成本） |
| 颜色多样性 | 3 种 | **25 种**（5 色系 × 5 深浅） |
| 形状多样性 | 仅正方体 | 正方体 + 圆柱体混合 |
| 空间规律性 | 散乱 | 5×5 有序网格（便于验证） |

---

## 2. 系统架构

```
┌───────────────────────────────────────────────────────────────────┐
│                    Gazebo Harmonic 仿真环境                        │
│                                                                   │
│  ┌──────────────────┐   ┌───────────────────────────────────┐    │
│  │ RGBD 俯瞰相机      │   │ 5×5 方阵桌面                        │    │
│  │ (0.5, 0, 1.5)    │   │                                    │    │
│  │ pitch=90° 垂直下视 │   │  y=-0.20  ■  ■  ■  ■  ■  红系    │    │
│  │ 640×480 @15fps   │   │  y=-0.10  ■  ◯  ■  ■  ■  蓝系    │    │
│  └────────┬─────────┘   │  y= 0.00  ■  ■  ■  ◯  ■  绿系    │    │
│           │              │  y= 0.10  ■  ■  ◯  ■  ■  橙系    │    │
│           ▼              │  y= 0.20  ■  ■  ■  ■  ■  紫系    │    │
│  /camera/color/image_raw │                                    │    │
│  /camera/depth/image_raw │  25 方块 (22 正方体 + 3 圆柱体)      │    │
└───────────┬──────────────┴───────────────────────────────────┘    │
            │                                                       │
            ▼                                                       │
┌───────────────────────────────────────────────────────────────────┐
│                    数据采集与训练流水线                              │
│                                                                   │
│  ┌──────────────────┐    ┌──────────────────┐                    │
│  │ collect_training │───▶│ auto_label_grid  │                    │
│  │ _data.py         │    │ .py              │                    │
│  │ 截图采集          │    │ 相机投影自动标注   │                    │
│  └──────────────────┘    └────────┬─────────┘                    │
│                                   │                               │
│                                   ▼                               │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │            train_yolo_block.py                            │    │
│  │            YOLOv8n 微调训练                                │    │
│  │            100 张图 × 25 框 = 2500 标注                    │    │
│  └──────────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────────┘
```

---

## 3. 项目文件结构

```
yolo_ros/
├── ros2_ws/src/yolo_bringup/worlds/
│   ├── grasping_world.sdf          # 原有：3 个散落方块
│   └── block_grid_world.sdf        # ★ 新建：3×3 方阵环境（425 行，block_0..8）
│
├── scripts/
│   ├── generate_grid_world.py      # ★ 方阵世界生成器（可调参数）
│   ├── auto_label_grid.py          # ★ 自动标注脚本（相机投影）
│   ├── demo_pipeline.py            # ★ 端到端演示脚本（合成数据）
│   ├── collect_training_data.py    # Gazebo 图像采集
│   ├── label_blocks.py             # 手动标注工具（备用）
│   └── train_yolo_block.py         # YOLO 训练脚本
│
├── block_dataset/
│   ├── data.yaml                   # 数据集配置
│   ├── train/
│   │   ├── images/                 # 训练图像 (.jpg)
│   │   └── labels/                 # YOLO 标注 (.txt)
│   └── val/
│       ├── images/                 # 验证图像
│       └── labels/                 # 验证标注
│
└── runs/detect/block_grid_demo/    # 训练输出
    └── weights/best.pt             # 最佳模型权重
```

---

## 4. 方阵世界设计详解

### 4.1 物理布局

| 参数 | 值 | 说明 |
|------|-----|------|
| 桌面尺寸 | 80×60×3 cm | 木色，距地 75cm |
| 方块边长 | 3 cm | 与参照项目一致 |
| 方阵排列 | 5 行 × 5 列 | 共 25 个 |
| X 间距 | 14 cm | 5 列跨度 56cm，居中桌面 |
| Y 间距 | 10 cm | 5 行跨度 40cm，居中桌面 |
| X 起始 | 0.18 m | 第一列 X 坐标 |
| Y 起始 | -0.20 m | 第一行 Y 坐标 |
| Z 高度 | 0.78 m | 桌面表面 0.765 + 半径 0.015 |

### 4.2 颜色方案

按行区分主色系，按列微调深浅（因子 0.7~1.3），共 25 种可区分颜色：

```
          Col 0    Col 1    Col 2    Col 3    Col 4
Row 0    深红     红       正红     亮红     浅红      (红色系)
Row 1    深蓝     蓝 ◯     正蓝     亮蓝     浅蓝      (蓝色系)
Row 2    深绿     绿       正绿     亮绿 ◯   浅绿      (绿色系)
Row 3    深橙     橙       橙 ◯     亮橙     浅橙      (橙色系)
Row 4    深紫     紫       正紫     亮紫     浅紫      (紫色系)
```

◯ = 圆柱体（用于形状多样性，共 3 个）

### 4.3 相机配置

| 参数 | 值 |
|------|-----|
| 位置 | (0.5, 0, 1.5) — 桌面中心正上方 |
| 姿态 | pitch=π/2（光轴垂直向下） |
| 分辨率 | 640×480 |
| 帧率 | 15 fps |
| 水平 FOV | 1.047 rad (60°) |
| RGB 格式 | R8G8B8 |
| 深度格式 | R_FLOAT32 |

### 4.4 SDF 结构

```xml
<sdf version="1.11">
  <world name="block_grid_world">
    <physics type="dart" />          <!-- 物理引擎 -->
    <plugin>gz-sim-sensors-system</plugin>  <!-- 传感器插件 -->
    <light type="directional" />     <!-- 方向光 -->
    <model name="ground_plane" />    <!-- 地面 -->
    <model name="table">             <!-- 80×60cm 桌子 -->
      <link name="table_top" />
      <link name="leg_fl/fr/bl/br" />
    </model>
    <model name="camera_model">      <!-- RGBD 俯瞰相机 -->
      <sensor name="rgb_camera" type="camera" />
      <sensor name="depth_camera" type="depth_camera" />
    </model>
    <!-- 25 个方块 -->
    <model name="block_r0_c0"> ... </model>
    ...
    <model name="block_r4_c4"> ... </model>
  </world>
</sdf>
```

---

## 5. 自动标注原理

### 5.1 核心思想

方阵中每个方块的 3D 位姿是**已知且固定的**。通过针孔相机模型，可以将方块的 8 个顶点从世界坐标投影到像素坐标，取外接矩形即得 YOLO 格式的 2D 边界框。

### 5.2 坐标变换

```
世界坐标 (wx, wy, wz)
       │
       ▼  (相机位于 (0.5, 0, 1.5)，光轴朝 -Z)
相机坐标 Xc = wx - 0.5
         Yc = wy - 0
         Zc = 1.5 - wz    (深度，正值)
       │
       ▼  针孔投影
像素坐标 u = fx × Xc/Zc + cx
         v = fy × Yc/Zc + cy

其中 fx = (W/2) / tan(FOV/2) = 554.4
     fy = fx (正方形像素假设)
     cx = 320, cy = 240
```

### 5.3 边界框计算

对方块的 8 个顶点分别投影，取 `(u_min, v_min, u_max, v_max)` 作为外接矩形，
归一化后输出 YOLO 格式：
```
0 x_center y_center width height
```
（`0` = block 类 ID）

### 5.4 自动标注 vs 手动标注

| 方式 | 耗时/张 | 精度 | 一致性 |
|------|---------|------|--------|
| 手动 (labelImg) | ~2 分钟 | 受人为偏差影响 | 不一致 |
| **自动 (投影)** | **< 1 秒** | **像素级精确** | **完全一致** |

---

## 6. 训练流水线

### 6.1 完整流程（Gazebo 真实模式）

```bash
# Step 1: 启动方阵仿真（第十轮：专用轻量启动器已恢复）
ros2 launch yolo_bringup grid_world.launch.py
#   world:=grasping_world.sdf  换用三色物体世界
#   headless:=true             无 GUI（采集仍可用，只是看不到窗口）
#
# 这个启动器**刻意不含机械臂**：本训练链只要「相机画面」，起 UR5/MoveIt 纯属浪费。
# 它做三件事：gz sim -r 载入 world + ros_gz_bridge 桥接相机话题 + 发布
# world→camera 静态 TF（默认开，可用 publish_camera_tf:=false 关掉）。
#
# 历史：原命令是 simulation.launch.py，第九轮随"入口收口到 bringup"被删——但它同时
# 是这两个 world 的**唯一**启动器，删后 block_grid_world.sdf / grasping_world.sdf
# 成了孤儿。第十轮把这个「训练世界」职责单独拆出来恢复，并改了名以免与旧入口混淆。

# Step 2: 采集训练图像
python3 scripts/collect_training_data.py

# Step 3: 自动标注
python3 scripts/auto_label_grid.py

# Step 4: 训练 YOLO
python3 scripts/train_yolo_block.py
```

### 6.2 快速演示模式（无 Gazebo）

```bash
# 一键端到端演示（合成图像 → 标注 → 训练 → 验证）
python3 scripts/demo_pipeline.py
```

此模式用 OpenCV 生成仿真的俯瞰方阵图像，完整演示流水线，
适用于无 Gazebo 环境的开发/调试。

### 6.3 演示训练结果

在 RTX 2060 (6GB) 上，100 张合成图像（80 train / 20 val），50 epochs：

| 指标 | 值 |
|------|-----|
| mAP50 | **0.995** |
| mAP50-95 | **0.983** |
| Precision | 1.000 |
| Recall | 1.000 |
| 推理速度 | 19 ms/image |
| 模型大小 | 6.2 MB (best.pt) |
| 每张图检测框 | 25/25 (100%) |

### 6.4 训练参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 模型 | yolov8n.pt | YOLOv8 Nano 预训练权重 |
| epochs | 50~100 | 合成数据 50 足够，真实数据建议 100 |
| imgsz | 640 | 输入图像尺寸 |
| batch | 16 | 批次大小 |
| device | cuda:0 | 自动回退 CPU |
| 数据增强 | 默认 | mosaic, hsv, flip 等 |

---

## 7. 生成器脚本说明

### `generate_grid_world.py`

可配置参数生成不同的方阵世界文件：

```python
GRID_ROWS = 5          # 行数
GRID_COLS = 5          # 列数
BLOCK_SIZE = 0.03      # 方块尺寸 (m)
X_SPACING = 0.14       # 列间距 (m)
Y_SPACING = 0.10       # 行间距 (m)
CYLINDER_POSITIONS     # 圆柱位置集合
ROW_COLORS             # 各行星系基准色
COL_VARIATION          # 列间深浅变化因子
```

修改参数后运行 `python3 scripts/generate_grid_world.py` 即可重新生成。

---

## 8. ROS2 集成

### 8.1 启动命令

```bash
# 完整流水线（仿真 + YOLO + 3D 感知 + 抓取规划）——现在的唯一入口
ros2 launch yolo_bringup bringup.launch.py camera_source:=sim

# 只起仿真+感知，不抓取
ros2 launch yolo_bringup bringup.launch.py camera_source:=sim skip_planner:=true
```

> ⚠️ 本节原写的命令是 `simulation.launch.py world:=block_grid_world.sdf`，**已不可用**：
> 该文件第九轮删除，且 `bringup.launch.py` 与 `grid_world.launch.py` **都没有 `world`
> 参数以外的世界选择能力**——带臂的世界由 `arm_grasp_sim/sim.launch.py` 独有（它必须
> 与那份 URDF/SRDF/控制器配置成套，单独换 world 会对不上臂）。
>
> **想只开世界、不开臂** → 用 §6.1 的 `grid_world.launch.py`。
> **想跑抓取** → 用上面的 `bringup.launch.py`（唯一入口）。二者别同时起，
> 会争同一个 `camera_model/camera_link/rgb_camera` 帧名。

### 8.2 关键 Topic

| Topic | 类型 | 方向 |
|-------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo → ROS2 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo → ROS2 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS2 |
| `/detections` | `vision_msgs/Detection2DArray` | yolo_detector → 下游 |

### 8.3 桥接配置

`ros_gz_bridge` 通过 `parameter_bridge` 模式自动转发 Gazebo topic 到 ROS2，
配置见 `ros2_ws/src/yolo_bringup/config/ros_gz_bridge.yaml`。

---

## 9. 扩展方向

1. **多类别方阵**：不同形状/颜色对应不同 YOLO 类别（block_red, cylinder_blue 等）
2. **动态场景**：加入随机移动/旋转的方块，训练模型对姿态变化的鲁棒性
3. **域随机化**：随机光照、桌面纹理、相机位置，提高 sim-to-real 迁移能力
4. **更大方阵**：7×7 或 10×10 网格，增加单图标注密度
5. **自动数据增强流水线**：采集 + 标注 + 训练 + 评估全自动化

---

## 10. 参考资料

- [Ultralytics YOLOv8 文档](https://docs.ultralytics.com/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [Moveit2YoloObb 参考项目](https://github.com/laoxue888/Moveit2YoloObb)
- 参照项目世界文件: `Moveit2YoloObb/moveit2_obb/src/panda_moveit_config/worlds/arm_on_the_table.sdf`
