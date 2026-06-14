# YOLO + ROS2 视觉抓取系统 (YOLO-ROS)

> 基于 ROS2 Jazzy + YOLOv8 + Gazebo Harmonic + MoveIt2 的机器人视觉抓取系统。  
> 目标：在仿真中复现 YOLO 检测 → 3D 定位 → IK 求解 → 抓取执行的完整闭环。

---

## 系统架构

```
Gazebo Harmonic 仿真
    │
    ├── RGBD 相机 (RealSense D435)
    │       ↓ /rgb, /depth, /camera_info
    ├── yolo_detector (YOLOv8 2D 检测)
    │       ↓ /detections
    ├── perception_3d (2D→3D 投影 + TF 变换)
    │       ↓ /target_3d
    └── grasp_planner (MoveIt2 IK + 轨迹执行)
            ↓ 关节指令
        机械臂抓取
```

---

## 环境要求

| 组件 | 版本 | 说明 |
|------|------|------|
| OS | Ubuntu 24.04 | WSL2 + WSLg |
| ROS2 | Jazzy Jalisco | 系统级 apt 安装，**必须 Python 3.12** |
| Gazebo | Harmonic (gz-sim8) | 通过 ros-jazzy-ros-gz-sim vendor 包提供 |
| Conda Python | 3.12 | rclpy 仅为 Python 3.12 编译 |
| numpy | < 2.0 | cv_bridge 基于 numpy 1.x 编译 |
| CUDA | 13.2 | GPU 加速 YOLO |
| YOLO | YOLOv8 (ultralytics) | pip 安装 |

---

## 快速上手

### 1. 一键配置

```bash
# 安装系统依赖 (ROS2 + Gazebo + MoveIt2)
bash scripts/install_deps.sh

# 创建 Conda 环境 + 依赖
bash scripts/create_env.sh
conda activate yolo_ros

# 运行最小验证测试 (无需 ROS2/Gazebo)
python scripts/test_minimal.py
```

### 2. 构建 ROS2 工作区

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动仿真抓取

```bash
# 完整启动 (Gazebo + 感知流水线)
ros2 launch yolo_bringup bringup.launch.py

# 分步启动:
# 终端 1: Gazebo 仿真
ros2 launch yolo_bringup simulation.launch.py

# 终端 2: 感知流水线
ros2 launch yolo_bringup pipeline.launch.py mode:=mock

# 终端 3: 触发抓取
ros2 service call /trigger_grasp std_srvs/srv/Trigger
```

---

## 目录结构

```
yolo_ros/
├── README.md
├── environment.yml                  # Conda 环境定义
├── docs/
│   ├── TECHNICAL_SPEC.md           # 详细技术方案文档
│   ├── GRID_ENV_DESIGN.md          # 方阵环境设计文档
│   ├── YOLO_TUTORIAL.md            # YOLO 训练+检测完整教程
│   └── YOLO_VISION_DESIGN.md       # ★ yolo_vision 子项目详细设计
├── scripts/
│   ├── install_deps.sh             # 一键安装系统依赖
│   ├── create_env.sh               # 创建 Conda 环境
│   ├── setup_workspace.sh          # 初始化 ROS2 工作区
│   ├── generate_grid_world.py      # 方阵世界生成器
│   ├── collect_training_data.py    # Gazebo 图像采集
│   ├── auto_label_grid.py          # 自动标注（相机投影）
│   ├── train_yolo_block.py         # YOLO 训练
│   ├── demo_pipeline.py            # 端到端演示
│   ├── capture_and_detect.py       # 单帧截图+检测
│   └── test_minimal.py             # 最小验证测试
└── ros2_ws/src/
    ├── yolo_detector/              # YOLOv8 检测节点
    ├── perception_3d/              # 2D→3D 投影节点
    ├── grasp_planner/              # 抓取规划节点
    └── yolo_bringup/               # 顶层启动 + Gazebo 场景
        ├── launch/                 # 启动文件
        ├── config/                 # ros_gz_bridge / RViz 配置
        └── worlds/                 # Gazebo 仿真场景
```

---

## 开发路线

- [x] 环境配置 + 技术文档
- [x] YOLOv8 检测 ROS2 节点
- [x] 2D→3D 投影 + TF 坐标变换
- [x] Mock 抓取规划节点 (测试用)
- [x] Gazebo Harmonic 仿真场景（静态物体）
- [x] 3 节点流水线联调通过（`ros2 node list` 验证）
- [ ] Gazebo 中接入 RGBD 相机传感器 + ros_gz_bridge 桥接
- [ ] YOLO 实时检测 Gazebo 场景中的物体
- [ ] MoveIt2 完整 IK 集成
- [ ] UR5/UR10 机械臂 URDF
- [ ] 端到端抓取闭环测试
- [ ] cuRobo IK 替换

## 已知问题

| 问题 | 解决方案 |
|------|----------|
| 清华镜像 403 | `sed -i` 替换为阿里云 `mirrors.aliyun.com` |
| CoppeliaSim Qt5 冲突 | `~/.bashrc` 改为 alias，不设全局 `LD_LIBRARY_PATH` |
| Python 3.10 → 3.12 | Conda 环境必须用 3.12（ROS2 Jazzy rclpy 要求） |
| numpy 2.x 不兼容 | `pip install "numpy<2" deprecated` |

详见 [`docs/TECHNICAL_SPEC.md` §9](docs/TECHNICAL_SPEC.md)。

---

## 参考资料

- [详细技术方案](docs/TECHNICAL_SPEC.md)
- [ROS2 Jazzy 文档](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [MoveIt2 文档](https://moveit.picknik.ai/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)

