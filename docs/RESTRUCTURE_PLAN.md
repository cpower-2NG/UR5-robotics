# YOLO-ROS 代码整理与迁移计划书

编写日期：2026-09-26
基线提交：`f250af11b77b1629b4e867eb7820912db66dbc2c`（`main`，8 个提交）
状态：**P1 主体已完成，P2 感知链路已修复并端到端验证**（在 `migrate/humble-orin` 分支；`main` 仍停在基线 `f250af11b`，**未改动、未推送**）
最近更新：2026-09-26 第七轮（**P2 余项收尾 + 链路首次端到端实证**：P2-4 设备退化、P2-5 冻结像素约定并加护栏、P2-7 `camera_source` 开关（含 `bringup` 透传与「真机不起 Gazebo」）；新增 `scripts/test_pipeline_e2e.py`，**实测 `/target_3d` 数值与解析预期一致**，确认 D33 确已修复且 P2-6 在真实链路上生效。前一轮为 D33/D1/D2/D8/P2-6 的修复）

---

## 0. 一句话结论

仓库里存在**两套互不连通的实现**：一套是当前主线的「纯 YOLO 检测」（可用、自洽、仅仿真），另一套是**完整可用的 UR5 + MoveIt2 抓取栈，以教学附件压缩包的形式藏在 `f250af11b` 提交里**——后者才是 Orin 迁移的正确起点，而不是 `ros2_ws/src/grasp_planner`（那是个 212 行的 mock 桩）。

整理的核心不是「复活历史代码」，而是**做减法**：确定单一主线、把附件里的抓取栈提到一等公民位置、修掉感知链路上几个会让真机直接失败的具体缺陷。

---

## 1. 事实基线（已核实）

### 1.1 仓库现状：三套并存

| 目录 | 性质 | 状态 | 位置 |
|---|---|---|---|
| `yolo_vision/` | 独立 ament_python 包，自带 world + 模型 + 启动 | **可用**，仿真内 YOLO 检测闭环 | 仓库根 |
| `ros2_ws/src/` | 4 包工作区：`yolo_detector`、`perception_3d`、`grasp_planner`、`yolo_bringup` | 感知可用；`grasp_planner` 是空壳 | 仓库根 |
| `yolo_coco/` | 独立 COCO 训练/评估工程 | 可用但与主线无关 | 仓库根 |
| `scripts/` vs `yolo_vision/scripts/` | 6 个同名训练脚本 | **已分叉**（内容全部不同） | 两处 |
| `04-Chapter（参考）+官方统一要求/` | 教材附件（docx/pptx/zip/mp4） | **内含完整抓取栈** | 仓库根 |

全仓库约 18 GB、251,146 个跟踪文件——体积主要来自 `.git` 历史、64 MB 演示视频与数据集。

### 1.2 关于「历史抓取代码」的重要更正

你记忆中的「最初 YOLO + MoveIt 完成完整仿真内感知抓取」——**MoveIt 部分属实，CoppeliaSim 部分在本仓库中没有任何证据**。

- **MoveIt 抓取栈确实存在，而且是完整的**，但不在 `ros2_ws` 的提交历史里。它在 `f250af11b 标准化`（**最后一个提交**）才加入的附件压缩包里：
  `04-Chapter（参考）+官方统一要求/ur5-yolo-grasp最终提交包/ur5-yolo-grasp.zip`
- **CoppeliaSim 在本仓库中不存在**。全仓库检索仅在 `docs/TECHNICAL_SPEC.md:339-362` 找到一处环境变量冲突记录（`libgz-sim8-gz.so` 与 CoppeliaSim 自带 Qt5 冲突），说明 CoppeliaSim 曾是**本地安装的工具**，其场景/脚本从未入库。
- `Moveit2YoloObb` 在 Git 树中是 **gitlink**（`160000 ce16f5a5…`），仓库内**没有 `.gitmodules`**，子模块源码与 URL 均未恢复。据 `docs/GRID_ENV_DESIGN.md:327-328`，它是 GitHub 上的 `github.com/laoxue888/Moveit2YoloObb`（**Panda 臂**，非 UR5），仅作为通信架构参照——这一点在 `docs/YOLO_GUIDE.md:200` 也明确写了「参照项目仅用于参考其 MoveIt2 抓取逻辑」。

> **需要你确认**：CoppeliaSim 阶段是否发生在更早的、未导入本 bundle 的工作目录里？如果是，那部分资产是否还在？（这会影响「是否值得找回」的判断，但不影响本计划的执行。）

### 1.3 附件里的抓取栈：比预期完整得多

`ur5-yolo-grasp.zip` 解压清单（共 29 个文件，约 6.3 MB）：

```
grasp_sim/                     ament_cmake 包
├── config/  ros2_control.yaml · kinematics.yaml · ur5_gripper.srdf.xacro
│            gz_bridge.yaml · hardware_plugins.xml
├── models/  ur5_with_gripper.urdf.xacro        (9.2 KB, UR5 + Robotiq 2F-85)
├── worlds/  grasp_world.sdf                    (13 KB, 桌子 + 9 物体 + 内联相机)
├── launch/  sim.launch.py (9.9 KB) · full.launch.py (3.9 KB)
└── scripts/ grasp_planner.py (591 行, 25 KB) · view_camera.py
yolo_vision/                   ament_python 包（检测部分，含 best.pt）
LAB.md (37 KB) · TEXTBOOK.md (41 KB) · README.md (8.5 KB)
```

**关键事实：它的目标环境就是你的目标环境**——README 首行写明 `ROS2 Humble + Gazebo Harmonic + MoveIt2`、`Ubuntu 22.04`。这不是一个需要从 Jazzy 回迁的旧项目，而是一个**已经按 Humble 写好、可直接对齐 Orin 的完整实现**。

**附件同时是一份现成的环境配置脚本（本轮新查，价值很高）**：

- `README.md:52`：`ros-humble-ros-gzharmonic gz-harmonic` —— Humble 配 Harmonic 的取包方式。
- `README.md:77` + `TEXTBOOK.md:747`：`gz_ros2_control` **必须源码编译**，apt 版只支持 Fortress；**humble 分支同时兼容 Fortress 与 Harmonic，无需改代码**。
- `README.md:97,112,125`：`export GZ_VERSION=harmonic`（每次新终端都要设）。
- `TEXTBOOK.md:225`：Harmonic 下插件名是 `gz_ros2_control/GazeboSimSystem`（Fortress 时代是 `ign_ros2_control/IgnitionSystem`）。
- `TEXTBOOK.md:771`：**完整启动时序**（t=0 xacro → t=2 `gz sim` → t=5 设 P 增益 → t=8 `robot_state_publisher` → t=9 spawn UR5 → t=12 `move_group` → t=14 spawn 控制器 → t=16 桥接）。
- `grasp_sim/config/gz_bridge.yaml`：**正确的** `ros_gz_bridge` 配置写法，可直接拿来修主线的 D19。

`grasp_planner.py` 是一个真正的状态机，不是桩：

```
常量:  GRIPPER_OPEN/CLOSE, LIFT_HEIGHT, APPROACH_CLEARANCE, CAMERA_PITCH,
       HOME_JOINTS, LEFT_DROP_JOINTS, IK_SEED
方法:  _publish_camera_tf()          ← 发布相机 TF（当前主线缺失）
       _optical_to_world()           ← 光学坐标系→世界坐标
       _call_ik()                    ← 调 MoveIt compute_ik
       _send_trajectory()            ← JointTrajectoryController
       _detection_cb / _process_detections   ← 带位置去重
流程:  approach → descend → close → lift → home → left_drop → place → open → home
```

它用**单一 `joint_trajectory_controller` 统管 7 个关节**（6 臂 + 1 夹爪 mimic 关节），依赖 `gz_ros2_control`（需源码编译，apt 版仅支持 Fortress）。

### 1.4 已核实的具体缺陷

以下每一条都是我实际读代码确认的，不是推断：

| # | 位置 | 问题 | 影响 |
|---|---|---|---|
| **D1** ✅**已修** | `perception_3d/projector_node.py:101-109, 144-146, 190-207` | 深度图 `passthrough` 取出后**直接当作米**参与投影，无 `depth_scale`、无编码判断。**本轮补证（解释了此 bug 为何一直没被发现）**：仿真侧深度相机是 `R_FLOAT32`（**米**，见 `grasping_world.sdf:222`），而真机 D435i 是 `16UC1`（**毫米**） | RealSense 16UC1 单位是**毫米** → 坐标放大 **1000 倍**，真机必炸。仿真里因为恰好是 float32 米，代码**蒙对了**——所以这个错误在仿真阶段**永远不会暴露**，只在接真机时爆。✅ **已修（第六轮）**：按 `msg.encoding` 查表换算（`16UC1`/`mono16` → `×0.001`，`32FC1`/`64FC1` → `×1.0`），**两种编码同时兼容**；未知编码报 ERROR 并回退；`depth_scale` 参数（默认 `-1`＝自动）可强制覆盖；在入口一次性换算成 float32 米 |
| **D2** ✅**已修** | 同上 `:54, 101-119` | 深度图独立缓存，与 detection 时间戳**无同步**；用彩色 `camera_info` 配**未对齐**的深度像素 | 运动/快速场景下取错深度；color/depth 视差导致偏差。✅ **已修（第六轮）**：改为**带时间戳环形缓冲**（`deque(maxlen=10)`）+ 按 stamp **就近配对**；超 `max_sync_dt`（默认 0.1 s）**丢弃并告警**；启动时提示真机需 `align_depth:=true` 且订阅**对齐后**的深度话题。⚠️ **未采用 `message_filters`**——理由见 §3 P2 小节（会引入新的「静默丢弃」） |

| D3 | `grasp_planner/planner_node.py:76-89, 183-195` | `moveit` 模式恒返回 `False`，`_init_moveit()` 直接置 `moveit_available=False` | 该包**没有任何抓取能力**，只是日志打印 |
| D4 | `yolo_bringup/worlds/grasping_world.sdf` | 场景内只有 `ground_plane / table / camera_model / red_cube / blue_cube / green_cylinder`，**没有机械臂** | README 宣称的「Gazebo 中复现完整闭环」不成立 |
| D5 | `ros2_ws/src/` 全局 | 全仓库无 URDF / moveit_config / ros2_control / 夹爪包 | 抓取链路缺全部机器人侧资产 |
| **D6** ✅**已修** | `pipeline.launch.py:17`、`bringup.launch.py:24,52`、`yolo_detector/launch/detector.launch.py:16`、`detector_node.py:30` | `model` 默认 `'yolov8n.pt'`，而 `detector_node.py` 把这个值**直接传给 `YOLO()`，不做任何路径解析**，于是行为**取决于进程 CWD**：① 仓库根 → 命中 `./yolov8n.pt`（其实是方块模型 A）；② `ros2_ws/` → 命中真 COCO；③ 都没有 → **ultralytics 静默联网下载 COCO**。<br>**🎯 根因（第五轮查明）**：这一切是 `scripts/train_yolo_block.py:143-151` **自己造出来的**——它训练完把新模型 `shutil.copy` **覆盖到仓库根的 `yolov8n.pt`**，并把被覆盖的文件备份为 `yolov8n_backup.pt`。所以方块模型占用了 `yolov8n.pt` 这个名字，真 COCO 反被赶进 `*_backup.pt`。「同名不同物」不是历史意外，是这段代码的**必然产物** | ✅ **已修（P1-3）**：① 检测端默认改为 `block_best.pt`（4 个文件 5 处）；② 删除仓库根 `yolov8n.pt`（内容与 `block_best.pt` **同一 blob** `e917eeaf1`，属纯重复）与 `ros2_ws/yolov8n.pt`（COCO 副本，其**路径本身**就是 CWD 陷阱）；③ COCO 基线改名 `yolov8n_backup.pt` → **`yolov8n_coco.pt`**；④ `train_yolo_block.py` 删掉那段覆盖逻辑，微调起点改为显式 `yolov8n_coco.pt`；⑤ `detector_node.py` 新增 `_resolve_model()`：绝对路径 → `$YOLO_MODEL_DIR` → CWD 及各级父目录 → 包 `models/`，**找不到就报错退出，绝不联网兜底**，并打印类别数、非 1 类时告警 |
| D7 | `yolo_vision/detector_node.py:98-100` vs `yolo_detector/detector_node.py:125-128` | 前者发布**归一化** bbox，后者发布**像素** bbox，而 `projector_node.py:129-130` 按像素使用 | 两个包**不可混用**，混用则投影全错。当前是潜在陷阱（未触发） |
| **D8** ✅**已修**（可见性部分） | `perception_3d/projector_node.py:158-163` | 依赖 `camera_frame → target_frame` 的 TF，但仓库内无任何静态 TF / robot_state_publisher 发布相机外参 | 真机上 TF 查询必然失败，节点静默跳过。✅ **已修（第六轮）**：失败由 `warn`+静默 `continue` 升级为 **ERROR + 连续失败计数**，并给出「先做手眼标定 / 发布静态 TF」的可操作提示；新增 `tf_required` 参数（默认 `False`），置真则**直接抛错退出**。<br>⚠️ **边界**：这只让失败**可见且可诊断**，**外参本身仍需标定**——**TF 不在代码层可修**，别把这条当成「已解决」 |
| D9 | `environment.yml`、`scripts/install_deps.sh` | 硬编码 Python 3.12 / `ros-jazzy-*` / Ubuntu 24.04 / Noble | 在 Orin（22.04 + Humble + Python 3.10）上**不可直接使用** |
| D10 | **本轮全仓扫描后扩大**：不止原先记的两处 | 对 `README.md` 与 `docs/*.md` 里所有 `scripts/*.py` 引用做「存在性」校验，共发现 **7 处死引用**：<br>① **活的、已修**：`docs/YOLO_TUTORIAL.md` 的 `capture_and_detect.py`（§6.1/§6.2 正文 + §8 文件表 + 常见问题）、`yolo_detect_simple.py`；`README.md:103` 的 `capture_and_detect.py`。<br>② **随章死亡的、无需单独修**：`docs/chapter03_yolo_perception.md:203,266,267,284,292,315,339,342,345` 引用的 `scripts/train.py`、`evaluate.py`、`detect.py`、`prepare_coco10.py` —— 这些是该章**自带**的 `yolo_coco` 项目布局，而该章已定**全删**（§5 问题 4），故随之消亡。<br>（曾误记 `scripts/grasp_planner.py` 也是死引用——本轮用 `git grep HEAD -- '*.md'` 复核，**它从未在任何文档中被引用**，该条已撤销。抓取脚本的真实位置是附件 `docs/reference/ur5-yolo-grasp/grasp_sim/scripts/grasp_planner.py`。） | 文档指向**不存在的文件**，照做必然失败 |
| D11 | `scripts/` vs `yolo_vision/scripts/` | 6 个同名脚本（`auto_label_grid` / `collect_training_data` / `demo_pipeline` / `generate_grid_world` / `label_blocks` / `train_yolo_block`）**内容全部不同** | 无法判断哪份权威，改错版本会静默失效 |
| D12 | `grasp_planner/package.xml` | 代码 `:80` `from moveit_msgs.msg import MoveItErrorCodes`（在 `_init_moveit()` 的防御性 `try` 内），但 `package.xml` **未声明 `moveit_msgs`**；反向地，`tf2_ros` / `tf2_geometry_msgs` **已声明却在代码中完全未使用** | 清单与代码**双向不符**。另：`MoveItErrorCodes` 导入后从未被引用，属残留 |
| **D13** ✅**已修** | 模型权重曾散落 **21 个 `.pt`**（全量哈希+加载实测） | 除 **A/B 两份方块模型**外，还有**真 COCO yolov8n**、**yolo26n（80 类）**、**1 份损坏文件**、10 个训练中间盘 | ✅ **已收敛（P1-3）**：删 14 个（根 `yolov8n.pt`、`ros2_ws/yolov8n.pt`、损坏件、10 个 `epoch*.pt`）、改名 2 个 → 现存 **7 个 `.pt`**，且**每个文件名都与其内容相符**：`block_best.pt`(A)、`runs/block_detect/weights/{best,last}.pt`、`runs/detect/block_grid_demo/weights/{best,last}.pt`(B)、`yolov8n_coco.pt`(COCO)、`yolo26n.pt`(COCO-26) |
| D18 | 模型 A / B 并存 | **A**（域随机化，较新）`a926d959…` 在 `block_best.pt`、`runs/block_detect/weights/best.pt`；**B**（方阵演示，较旧）`b9baf6f1…` 在 `runs/detect/block_grid_demo/weights/best.pt` 与**附件**的 `yolo_vision/models/best.pt`。~~`yolo_vision/models/best.pt`~~ 已随该包删除 | 附件抓取栈自带的是**旧模型 B**；P3 接入时应改用模型 A，勿为「与附件一致」而回退 |
| D14 | `yolo_bringup/launch/simulation.launch.py:45` | `bridge_config` 计算后**从未使用**（桥接全靠硬编码参数） | 配置文件形同虚设，改 YAML 无效果 |
| D15 | 同上 `:28-31, :37-41` | `gui_arg` 声明后 `gz_sim` 未引用；`condition=None` 恒真 | `gui:=false` 无效 |
| D16 | 同上 `:108-130` | `rviz_node` 定义但**未加入** `LaunchDescription` 返回值。**补充（本轮新查）**：`setup.py:17` 只装 `glob('config/*.yaml')`，`grasping.rviz` **根本没被安装到 `share/`** | RViz 永不启动；**即使接上也找不到配置文件**——修 D16 必须同时补 `setup.py` 的安装规则 |
| D17 | 同上 `:120-121` | `TimerAction(period=3.0)` 等 Gazebo 就绪 | 固定延时。**参照系已找到**：附件 `TEXTBOOK.md:771` 给出的可用时序是 **t=0/2/5/8/9/12/14/16 共 8 段**（spawn UR5、move_group、控制器、桥接各占一拍），3 秒远不够 |
| **D19** | `yolo_bringup/config/ros_gz_bridge.yaml` | **⚠️ 本轮收窄了此缺陷（原先写过头了）**：真正的、致命的错**只有一处**——顶层被包了一层 `ros__parameters:`，而 `parameter_bridge` 的 `config_file` 要求顶层是**桥接条目列表**。实测证实：旧文件 `yaml.safe_load` 得到 `dict`，**不是 `list`**。~~「未给 `gz_topic_name`」~~ **不是缺陷**：本项目 world 已把 Gazebo 话题**显式命名成与 ROS 同名**（`grasping_world.sdf:197,215` 的 `<topic>/camera/color/image_raw</topic>`），故 `topic_name` **直通写法在此处是对的**。 | 照旧文件配置桥接会**直接失败**。**❗重要修正**：附件 `grasp_sim/config/gz_bridge.yaml` **只能借鉴其「顶层列表」结构，绝不能抄它的话题名**——它的 `gz_topic_name: /rgbd_camera/...` 与 `/world/grasp_world/...` 是**它自己 world** 的名字，本项目里并不存在，照抄会把桥接接到空气上（这正是本轮差点犯的错）。<br>**已修（P1-8）**：改为顶层 list，保留直通话题名，并补了深度点云条目 |
| **D20** | 本机环境（实证） | 本机是 **Humble + Gazebo Classic 11.10.2 + `gazebo_ros2_control`**：`/usr/bin/gz` 属 `gazebo` 包（Classic 的 CLI），故 `gz sim --versions` 报 `Invalid arguments`；`ros_gz_bridge`、`gz_ros2_control`、`gz-harmonic` **全部未安装**，`ros-humble-ros-gz*` 计数为 0 | 主线的 `simulation.launch.py`（`gz sim -r` + `ros_gz_bridge`）**在本机根本跑不起来**。P3 的第一步不是写代码，而是**按附件 README 配置 Harmonic 环境** |
| **D21** | `docs/chapter03_yolo_perception.md` ←→ 两个待删包 | 教材章节正文**依赖**这两个包：§3.4「实验一」= `yolo_coco`（`cd yolo_coco`、`conda activate yolo_coco`，内嵌 3 张 `yolo_coco/assets/*` 图）；§3.5「实验二」= `yolo_vision`（`ros2 launch yolo_vision sim_detect.launch.py`，内嵌 2 张 `yolo_vision/images/*` 图）。**5 张图当前全部存在且有效** | 删包会打断教材正文：5 处图链失效、两个实验小节（约 620 行中的 360 行）失去对应代码。**这是「删包」决策的新增约束**——见 §5 待确认 |
| **D22** | `docs/chapter03_yolo_perception.md:146,162` 对比附件/TEXTBOOK | 教材教的是 **Jazzy + Gazebo Harmonic**；附件 `TEXTBOOK.md` 与迁移文档是 **Humble + Harmonic**；两者 ROS 发行版不同 | 学生照教材装 Jazzy，与 Orin（Humble）不兼容；教材需与附件统一到 Humble |
| **D23** | 本机 Python 环境实测 | 系统 `python3` = `/usr/bin/python3`，numpy **1.21.5**、matplotlib 3.5.1、setuptools **59.6.0**；`ultralytics`/`torch`/`opencv-python` **均未安装**。而附件 `README.md:56-75` 警告：装 ultralytics 会把 numpy 升到 **2.x**，与 apt 的 matplotlib 3.5 不兼容，故须连 `"matplotlib>=3.8"` 一起装；且 ultralytics 要求 setuptools≥70，而 **ament_python 不支持 79+**，须锁回 `==68.0.0` | **两个后果**：① 当前 YOLO 检测节点在本机**根本跑不起来**（无 torch/ultralytics）；② 主线 `yolo_detector` 与 `perception_3d` 都**用 cv_bridge**，而 Humble 的 cv_bridge 是按 numpy 1.x 编的——升到 numpy 2.x 有 ABI 风险。附件正是为此**刻意绕开 cv_bridge**、手写 numpy 转换（见 `ATTACHMENT_MANIFEST.md` §4）。P2 需在「锁 numpy<2」与「改成手工转换」之间选一条 |

| **D24** ✅**已修** | `yolo_detector/setup.py:9-14`（并 `grasp_planner`、`perception_3d` 同构） | `data_files` 只装 `resource/`、`package.xml`、`launch/`，**没有 `models/`**；包内也**确无任何 `.pt`**。而 `detector_node.py` 把 `model` 参数**原样丢给 `YOLO()`** | 在干净环境（尤其 **Orin 上全新部署**）启动后，`YOLO('yolov8n.pt')` 会**联网下载官方 COCO 模型**——真机上表现为「**什么都检测不到，且不报错**」 | ✅ **已修（P1-3）**：`detector_node.py` 的 `_resolve_model()` 会依次查绝对路径 / `$YOLO_MODEL_DIR` / **CWD 及各级父目录** / 包 `models/`，命中真实文件才加载，**找不到即 `FileNotFoundError` 退出**。模型不必随包复制（避免又一次 6 MB 重复），但**联网兜底这条路径已被彻底堵死**。另加类别自检：非 1 类 `block` 时 `WARN`，让「用错模型」在真机上**可见** |
| **D25** ✅**已修** | `block_dataset/yolov8n.pt` | SHA `4a36e372a4ba16d5`，4,333,568 B。`torch.load` 报 `PytorchStreamReader failed reading zip archive: failed finding central directory` | **仓库里存在一个加载即失败的权重文件**（下载中断残件，4.3 MB < 应有 6.5 MB）。会诱使后来者以为「有个可用的 yolov8n 基线」，实际一用就崩。✅ **已删（P1-3）** |
| **D26** | `Moveit2YoloObb/`（仓库根） | 被登记为**子模块指针**（`mode 160000` → `ce16f5a51235371a780671943e635ba80dfd42c0`，于 `61703a46d` 加入），**却无 `.gitmodules`**；该对象不在对象库、无 `.git/modules/`、全盘无副本。磁盘上只剩**空目录** | ✅ **已解决（本轮）**：仓库内两处文档记着它的 URL（`docs/GRID_ENV_DESIGN.md:327`、`docs/YOLO_VISION_DESIGN.md:4`）→ **`https://github.com/laoxue888/Moveit2YoloObb`**。该 commit **在上游仍存在**，已按原版本精确检出到 `docs/reference/Moveit2YoloObb/`（43 MB，带 `PROVENANCE.md`，`.git` 已删以免再嵌套仓库）。**详见 D28** |
| **D27** ⬇️**已降级为已知限制** | `yolo_detector/detector_node.py:118-128`、`perception_3d`、`grasp_planner/planner_node.py:142,156,173` | 感知链**没有朝向**：检测框是轴对齐的 `BoundingBox2D`；`Detection2D.bbox.center` 本是有 `theta` 的 `Pose2D` 却留空；抓取 `orientation` 三处**硬编码 `w=1.0`** | **⚠️ 第五轮查证后，本条原先的「P5 必修前置」判断被推翻了**：<br>`generate_grid_world.py` 的 `generate_random_blocks()` **只扰动 x/y，没有 yaw 字段**；`block_grid_world.sdf:205` 的方块几何是 `<size>0.0400 0.0400 0.0400</size>`（**立方体**）；`assign_colors_and_shapes()` 只产出 `box` 或 `cylinder`。**立方体对 90° 偏航不变，圆柱对任意偏航不变**——当前物体集里**没有一个物体有可辨朝向**。<br>② 因此平行夹爪**不需要**对齐：沿着任一对面夹持都成立。<br>③ 更关键：在这个数据集上训 OBB 模型会**学不到任何东西**（偏航方差恒为 0）。<br>**结论（你已确认）**：降级为「已知限制」，**不实施**。<br>**何时会重新变成必需**（写进文档备查）：目标物体**非立方**（长方体/细长件）、或需要**绕开邻近物体**择向接近时。真机上若出现这类物体，届时**必须先给世界生成器加 yaw 随机化**，否则仿真无法验证朝向 |
| **D28** | `docs/reference/Moveit2YoloObb/`（新恢复） | 该参考项目是 **Franka Panda + YOLOv8-OBB + MoveIt2**，**不是 UR5**。核心可借鉴两点：① `InferenceResult.coordinates` = `box.xyxyxyxy[0]` 的 8 个浮点（旋转框四角）→ 提供**朝向**（正好补 D27）；② `panda_moveit_config/scripts/arm_control_from_UI.py` 的 `listener_callback` 是一套完整**抓放序列**（接近→开爪→下降→闭爪→抬升→平移→开爪），含三个高度常量。❌ **不可照抄**：硬编码绝对模型路径 `/workspaces/...`（D6/D24 同类反模式）、非标准的四元数写法（`x=1.0, y=angle, z=0, w=0`）、自定义 msg、以及 Panda 专属名称与常量 | 它是**架构与思路**参考，**移植基底仍是 `docs/reference/ur5-yolo-grasp/`**（UR5 + 夹爪 + IK + SRDF + Humble/Harmonic，与本项目一致）。两者互补 |
| **D29** ✅**已修**（本文档涉及的） | 全部文档：`README.md:50`、`docs/YOLO_TUTORIAL.md`（7 处）、`TECHNICAL_SPEC.md:277`、`YOLO_VISION_DESIGN.md:288,309,321`、`chapter03:181,265,278,431,444,501,550` | 通篇教用户 `conda activate yolo_ros`（另有两个变体 `yolo_coco` / `yolo_vision`），共 **25+ 处** | **这三个环境在本机一个都不存在**。实测 `conda env list` 只有 `base` + `robot-pose` / `robot-tools` / `robot-vision` / `yolo-grasp`。真正带 torch+ultralytics 的是 **`robot-vision`**。且这些是 **conda prefix 环境**，`conda activate robot-vision` 会报 `EnvironmentNameNotFound`，**必须写全路径** `/home/<user>/robotics/venvs/robot-vision`。照文档做第一步就失败。<br>✅ 已修 `README.md` + `YOLO_TUTORIAL.md`（含一段说明）；**`chapter03` 与本文件残留部分待随该章处置**。⚠️ **Orin 上的环境名尚未确认**——板上无 conda，属另一套（系统 Python 3.10） |
| **D30** ✅**已修** | `auto_label_grid.py:24-25`、`generate_synthetic_data.py:29-32`、`demo_pipeline.py:25-30` | **三个标注器写同一个 `block_dataset/{train,val}/{images,labels}`**，且文件名规则相同（`img_XXXX`）。但 `demo_pipeline.py` 用 `BLOCK_SIZE=0.03` + 5×5 布局，另两者用 `0.04` + 3×3 | **会产出脏数据集**：先跑 A 再跑 B，图像被覆盖而标注可能残留（或反之），得到「图像与标注来自**不同几何假设**」的样本，且**没有任何标记能区分**。训练出来的模型精度无声劣化，排查极难。<br>✅ 已修：`demo_pipeline.py` 的 `DATASET_DIR` 改为独立的 `block_dataset_demo/`（一行改动即完全隔离），并加入 `.gitignore`；同时在两个标注器与教程 §4.2 注明「`BLOCK_SIZE` 必须与 `generate_grid_world.py` 一致」 |
| **D31** ✅**已修** | `yolo_bringup/launch/bringup.launch.py:24,39,52` | `world_arg` / `model_arg` / `mode_arg` / `device_arg` **四个 launch 参数都声明了，但下游 Include 全部硬编码**（`'world': 'grasping_world.sdf'`、`'model': 'yolov8n.pt'`、`'mode': 'mock'`、`'device': 'cuda:0'`） | `ros2 launch yolo_bringup bringup.launch.py world:=别的.sdf` **静默无效**——用户以为换了世界/模型，实际用的还是硬编码值。这是「配置项看起来能用但其实不能用」的典型陷阱，且**没有任何报错**。<br>✅ 已修：四处改用 `LaunchConfiguration(...)`，并补 `from launch.substitutions import LaunchConfiguration` |
| **D32** ✅**已修** | `docs/YOLO_TUTORIAL.md` §2.1/§2.3 ←→ `scripts/demo_pipeline.py:33-52` | 教程把**「5×5、25 个方块、BLOCK_SIZE=0.03、X_SPACING=0.14、Y_SPACING=0.10、CYLINDER_POSITIONS={(1,1),(2,3),(3,2)}」**当作「方阵世界」的参数教给读者 | **这组数字与 `demo_pipeline.py` 的常量逐字相同，而与真正的世界生成器 `generate_grid_world.py`（3×3 或随机、0.04、0.16/0.12）完全不同**——教程把**合成演示脚本的私有常量**当成了仿真世界的常量。照它改世界必然对不上，也解释了 D30 里 0.03/0.04 分叉的由来。<br>✅ 已修：§2.1 布局图按真实世界（3×3、8 方 + 1 圆柱、0.04 m、相机 1.5 m/60° FOV）重画；§2.3 改为如实描述**两种模式**（随机＝默认、`--fixed`＝调试），并显式警告「当前提交的世界是 `--fixed` 产物、seed=824517619，不带 `--seed` 重新生成会得到另一个场景」 |
| **D33** ✅**已修** ⚠️**本轮新发现，严重度高于 D1/D2/D8** | `ros2_ws/src/perception_3d/perception_3d/projector_node.py:166`（原行号） | 调用 `tf2_geometry_msgs.do_transform_pose(cam_pose, transform)`，其中 `cam_pose` 是 **`PoseStamped`**。但 Humble 的签名是 `do_transform_pose(pose: **Pose**, transform: TransformStamped) -> Pose` | **`/target_3d` 从未被发布过——一次都没有。**<br>实测（`source /opt/ros/humble/setup.bash` 后直接 `inspect.signature`）：传 `PoseStamped` → `AttributeError: 'PoseStamped' object has no attribute 'position'`；传 `Pose` → 正常返回。而该处的 `except` **只捕获 `LookupException` / `ConnectivityException` / `ExtrapolationException`**，`AttributeError` **不在其中** → 异常**逃出 `detection_callback`**，每来一个检测就抛一次，发布语句永远走不到。<br>**这解释了为什么 README 的「3 节点流水线联调通过」具有误导性**：那次验证用的是 `ros2 node list` / 话题存在性，**只证明了节点起得来，没有验证过一个字节的数据流过**。也就是说，**整条感知链路此前从未端到端跑通过**——这比 D1（尺度差 1000 倍）更靠前，因为 D1 至少还需要「发布」这一步能发生。<br>✅ 已修：改为 `do_transform_pose_stamped(cam_pose, transform)`（Humble 已提供，签名 `(PoseStamped, TransformStamped) -> PoseStamped`，实测存在），并保留一条拆 `Pose` 再装回 header 的兜底；**同时补了回归测试**（见下） |

> **D23 的解法倾向**：跟附件走「手工转换、不依赖 cv_bridge」更稳——既能避开 numpy ABI，也让感知节点在 Orin（JetPack 自带 numpy 版本可能与 apt 不同）上少一个变量。这条与 P2-5（坐标约定）一起做比较省事。

> **权重实测总表（P1-3 依据，2026-09-26 全量哈希 + 逐份 `torch.load` 验证）**——`find . -name '*.pt' -not -path './.git/*' -not -path './docs/reference/*'` 原有 **21 个文件**，去重后 **7 份互异模型**；**P1-3 执行后剩 7 个文件**：
>
> | 代号 | SHA-256(前16) | 字节 | `nc` / `names` | 现存文件（P1-3 后） | 判定 |
> |---|---|---|---|---|---|
> | **A** | `a926d959b5aa9f9e` | 6,242,538 | **1** / `['block']` | **`block_best.pt`**、`runs/block_detect/weights/best.pt` | **在用的方块模型**（域随机化版）。✅ 名字与内容终于一致 |
> | A-last | `7d8dcac22ee91d1c` | 6,242,538 | 1 / `['block']` | `runs/block_detect/weights/last.pt` | A 的末轮（与 best 同尺寸但**哈希不同**） |
> | **B** | `b9baf6f168dceebd` | 6,236,074 | 1 / `['block']` | `runs/detect/block_grid_demo/weights/best.pt` | **旧方块模型**（方阵演示版）；附件抓取栈自带的也是它（副本已随 `yolo_vision/` 删除） |
> | B-last | `9ee577feffec2d4f` | 6,236,074 | 1 / `['block']` | `runs/detect/block_grid_demo/weights/last.pt` | B 的末轮 |
> | COCO | `f59b3d833e2ff32e` | 6,549,796 | **80** | **`yolov8n_coco.pt`**（原 `yolov8n_backup.pt`） | **真 COCO yolov8n**——唯一的 COCO 基线。✅ 已改名，不再冒充方块模型 |
> | COCO-26 | `9b09cc8bf347f0fc` | 5,544,453 | **80** | `yolo26n.pt` | **yolo26n**——yolov8 的**下一代**（yolo26）COCO 模型；与板上 `~/yolo26_data` 目录呼应 |
>
> **已删除的 14 个**：根 `yolov8n.pt`（与 `block_best.pt` **同一 blob** `e917eeaf1`，纯重复）、`ros2_ws/yolov8n.pt`（COCO 副本，其**路径**即 D6 的 CWD 陷阱）、`block_dataset/yolov8n.pt`（✗损坏，D25）、10 个 `epoch{0..90}.pt`（~245 MB，P1-7）。
>
> **P1-3 的结果**：**每个现存文件名都与其内容相符**，且**仓库内已无任何代码引用 `yolov8n.pt`**（`runs/*/args.yaml` 里的那两处是 ultralytics 自动写下的**历史训练记录**，属不可改的实证，故保留）。
>
> 另注：D13/D18 原先依据「字节数一致」推断，已全部改用 **SHA-256 实测**，结论随之修正。

> **D19/D20 的来源**：附件 `TEXTBOOK.md:747` 明确「apt 包是为 Fortress 编的，Harmonic 下插件名变了……本系统用 humble 分支源码编译，**humble 分支同时兼容 Fortress 和 Harmonic**」。而附件 `sim.launch.py:1` 的 docstring 写的是「Launch Gazebo **Fortress**」——**该 docstring 是过期的**，实际代码用 `GZ_VERSION=harmonic` + `ros_gz_sim` + `gz_ros2_control/GazeboSimSystem`，即 **Harmonic**。已核实，勿据那句 docstring 判断后端。

---

## 2. 目标架构（建议）

```
┌─ 硬件层 ─────────────────────────────────────────────┐
│  Orin Nano Super / Ubuntu 22.04 / JetPack 6.2        │
│  RealSense D435i ── USB3 ──┐                         │
│  UR5 ── 网络 RTDE ─────────┤  (已确认 UR5)            │
└────────────────────────────┼─────────────────────────┘
                             │
┌─ 感知层（本次整理重点）─────▼─────────────────────────┐
│  realsense2_camera  →  align_depth  →  /camera/...   │
│  yolo_detector      →  /detections (像素坐标, 锁定)   │
│  perception_3d      →  /target_3d  (D33/D1/D2/D8 修) │
└────────────────────────────┬─────────────────────────┘
                             │
┌─ 抓取层（来自附件，需移植）─▼─────────────────────────┐
│  grasp_sim/grasp_planner.py  →  MoveIt2 compute_ik   │
│  joint_trajectory_controller →  UR5 + Robotiq 2F-85  │
└──────────────────────────────────────────────────────┘
```

**四条结构性决定：**

1. **单一主线 = `ros2_ws/src/`**。`yolo_vision/` 与 `yolo_coco/` 均为教学包，**已确认删除**（用户有独立备份，复用价值低）。⚠️ **但本轮发现这两个包是教材 `chapter03` 两个实验的载体（D21）——删除前需先处理教材正文，见 §5 问题 9。**
2. **抓取层采用附件的 `grasp_sim`，而非扩展 `ros2_ws/src/grasp_planner`**。后者是空壳（D3/D5），改造它等于从零写；前者 591 行可直接用，且环境本就对齐。
   **本轮补记（该附件比原先认为的更完整）**：`docs/reference/ur5-yolo-grasp/grasp_sim/` 自带**整条抓取栈**，不只一个脚本——
   `scripts/grasp_planner.py`（IK 抓取主逻辑）、`models/ur5_with_gripper.urdf.xacro`、`config/ur5_gripper.srdf.xacro`（SRDF）、`config/kinematics.yaml`、`config/ros2_control.yaml`、`config/hardware_plugins.xml`、`config/gz_bridge.yaml`（**即 D19 的正确范本**）、`launch/full.launch.py` + `sim.launch.py`、`worlds/grasp_world.sdf`。
   **这一点在 Orin 上尤其关键**：P4-1 实测板上**缺 `ros-humble-moveit` 与 `ros-humble-robotiq-description`**，而附件**自带 UR5+夹爪的 xacro 与 SRDF**，可**不依赖 `robotiq-description` 包**独立描述机器人。故 P5 移植时，附件这份 xacro/SRDF 是**首选基底**，apt 包只作补充——这也降低了在 Orin 上安装 MoveIt 全家的压力。
   ⚠️ 但要注意：附件抓取栈自带的是**旧模型 B**（D18），移植时**必须换成模型 A**，勿为「与附件一致」而回退。
   **两个参考的分工（本轮厘清，勿混用）**：
   | 参考 | 是什么 | 在本计划中的角色 |
   |---|---|---|
   | `docs/reference/ur5-yolo-grasp/` | **UR5** + 夹爪 + IK + SRDF + Gazebo Harmonic/Humble | **移植基底**——机器人与发行版都与本项目一致，xacro/SRDF 可直接当底子 |
   | `docs/reference/Moveit2YoloObb/` | **Panda** + YOLOv8-**OBB** + MoveIt2 | **架构与思路参考**——借它的 **OBB 朝向**（补 D27）与**抓放序列**，代码不可直接搬 |
   **关于 D27（朝向）——第五轮已降级，不再是 P5 的前置阻塞**：附件那套抓取栈确实建立在「已有朝向」的前提上（它的 `[x, y, θ]` 含 θ），本项目也确实**没有任何朝向**。但第五轮查证发现：本仿真**所有物体都朝向对称**（方块是立方体、圆柱天然旋转对称，且世界生成器根本没有 yaw 字段），平行夹爪**不需要对齐**即可夹持。故 D27 降级为**已知限制**而非必修项——**P5 可以直接移植 planner**。只有当目标物体变成非立方体（长方体/细长件）时，才需要回头补朝向，届时**必须先给世界生成器加 yaw 随机化**，否则仿真无法验证（详见 D27）。
3. **仿真后端 = Humble + Gazebo Harmonic，且只在工作站用**（= 附件的既有选择，已获用户确认）。理由：附件全栈已按此写好并跑通；`ros-humble-ros-gzharmonic` 官方支持该组合；Orin 是 Humble/22.04，与之一致。**不要**选 Gazebo Classic（11.x 已 EOL，24.04 无包），也**不要**为追教材里的 Jazzy 而升级发行版（与 Orin 冲突，见 D22）。
   **重要澄清（P4-1 实测后）**：**Orin 上不需要任何仿真器**——它的任务是接**真机**，已装的是 `ur-robot-driver` 而非 Gazebo。仿真只在工作站做（P3），Orin 走真机链路（P4/P5）。所以「本机装 Harmonic」与「Orin 装 MoveIt/UR 驱动」是两件互不冲突的事。
4. **验收路径 = 先仿真跑通附件栈，再做 Orin 感知，最后真机**。不要跳过仿真直接在板上调抓取。

**已确认的决策（2026-09-26）：**

| 决策项 | 结论 |
|---|---|
| P3-6 投影归属 | **路线 (B)**：抽出统一 `perception_3d`，抓取节点只消费 `/target_3d` |
| `yolo_vision/` | **删除**（教学包，有备份）——⚠️ 受 D21 约束，待 §5 问题 9 确认执行方式 |
| `yolo_coco/` | **删除**（教学包，有备份）——⚠️ 受 D21 约束，待 §5 问题 9 确认执行方式 |
| 仿真后端 | **Humble + Gazebo Harmonic**（待你确认，见 §5 问题 10） |
| 下一步 | P0（已完成）→ P1（进行中） |

> 决策 (B) 得到新证据支持：附件的 `yolo_vision` 自称做 3D，但核对后确认它**同样没有处理深度尺度**（`detector_node.py:126, 233` 原样取用深度当米），与主线 D1/D2 是同类缺陷。换用附件实现**不能规避**修复，两条路都得修——因此把修复集中到 `perception_3d` 做一次才是对的。详见 `docs/ATTACHMENT_MANIFEST.md` §5。

---

## 3. 分阶段计划

### P0 — 基线冻结与分支 ✅ 已完成（2026-09-26）

| 任务 | 产物 | 状态 |
|---|---|---|
| 建迁移分支，`main` 保持只读 | 分支 `migrate/humble-orin` | ✅ |
| 记录附件哈希 | `docs/ATTACHMENT_MANIFEST.md`（4 个附件 + 2 个 Git blob） | ✅ |
| 附件解压到隔离目录 | `docs/reference/ur5-yolo-grasp/`（27 文件），已加入 `.gitignore` | ✅ |
| 权重身份确认 | 发现**两个不同版本的方块模型**（见清单 §3） | ✅ |
| 清点残留物 | 发现 `.cph-ng/yolo_vision/README.md.bin`（268 B，**已被 Git 跟踪**的 IDE 残留） | ✅ |

**结果**：`main` 未改动；迁移分支 `git status` 仅含预期的新增文档；解压副本不会入库。

> `.cph-ng/yolo_vision/README.md.bin` 已跟踪且无意义，列入 P1 清理。

### P1 — 仓库整理（减法）

| 编号 | 任务 | 说明 |
|---|---|---|
| P1-1 | 确定 `scripts/` 唯一权威副本 | 对 6 个分叉脚本逐一 diff（D11），保留较新/更完整的一份，另一份删除 |
| P1-2 | 清理死的文档引用 | ✅ **已完成**（D10）：`YOLO_TUTORIAL.md` §6 重写为「无 ROS 自检（`test_minimal.py`）+ ROS2 节点实时检测」，§8 文件表与常见问题同步更正；`README.md` 脚本树删去不存在的 `capture_and_detect.py`、补上真实存在的 `label_blocks.py`/`move_block.py`/`generate_synthetic_data.py`。chapter03 内的 5 处死引用**随该章删除而消亡**，无需单独处理。<br>**第五轮追加**：`README.md` 列的 12 个脚本**逐一实测存在**（无死链）；并修掉 `YOLO_TUTORIAL.md` 的 §1.2/§1.3/§2/§4/§5/§6/§7 共 7 类陈旧内容（环境名、块尺寸、网格规模、模型名、`--fixed` 语义），详见 D29/D32 |
| P1-3 | 模型权重收敛 | ✅ **已完成**：21 个 `.pt` → **7 个**，文件名全部与内容相符（见 §1.4 实测总表）。根 `yolov8n.pt` 与 `ros2_ws/yolov8n.pt` 删除、COCO 改名 `yolov8n_coco.pt`、损坏件与 10 个 `epoch*.pt` 删除；**D6 根因（`train_yolo_block.py` 的覆盖逻辑）一并拆除**，检测端默认值 + 路径解析 + 类别自检全部落地（D6/D24） |
| P1-4 | 补 `package.xml` 依赖 | 至少修 D12（`grasp_planner` 声明 `moveit_msgs`）；核对 `perception_3d` 的 `numpy` |
| P1-5 | 修正误导性文档 | `README.md` 头部宣称的 MoveIt2 闭环与 D4 不符；`TECHNICAL_SPEC.md` 需标注「设计目标 vs 已实现」 |
| P1-6 | 处理 `Moveit2YoloObb` gitlink | ✅ **内容已找回**（D26/D28）：URL 是 `github.com/laoxue888/Moveit2YoloObb`，原 commit `ce16f5a5…` 仍在上游，已检出到 `docs/reference/Moveit2YoloObb/`（43 MB，不入库）。**剩下只差一个处置决定**（见 §5 问题 11）：是①删除这个悬空 gitlink 条目、把参考留在 `docs/reference/`；还是②补 `.gitmodules` 让它成为可用的真子模块；还是③保持现状仅加注释 |
| P1-7 | 大文件出库 | 64 MB 视频 + `runs/**/epoch*.pt`（24.4–24.9 MB × 10 ≈ **245 MB**，已核实）考虑 Git LFS 或移出仓库 |
| P1-8 | 修 D19 坏的桥接 YAML | ✅ **本轮已改**：改为顶层 list；保留**直通话题名**（因为 world 已把 gz 话题命名成与 ROS 同名）；补深度点云条目。⚠️ **不要抄附件的话题名**（见 D19 修正） |
| P1-9 | 修 D16 的安装规则 | ✅ **本轮已改**：`yolo_bringup/setup.py` 补 `glob('config/*.rviz')`，否则 RViz 修好也找不到配置 |

**P1 已完成（2026-09-26，均在 `migrate/humble-orin` 上，未提交）：**

| 项 | 动作 | 证据 |
|---|---|---|
| P1-1 前置 | **先切断 `yolo_vision/` 依赖**。三处根脚本原本读写 `yolo_vision/worlds/`，删包会让它们失效、**并且 `generate_grid_world.py` 会重新创建被删目录** | 已改 `scripts/generate_grid_world.py`（`OUTPUT_DIR` 合并、去掉 `OUTPUT_DIR_ALT`、SDF 与 meta 写入收敛为单目录）、`scripts/auto_label_grid.py`（`META_PATHS` 收敛）、`scripts/generate_synthetic_data.py`（`META_PATH` 重指向） |
| P1-1 验证 | 三脚本 `py_compile` 通过；生成器实跑，输出落 `ros2_ws/src/yolo_bringup/worlds/` | ✅ 已实测 |
| P1-4 | `grasp_planner/package.xml` 补 `moveit_msgs`，用 **`exec_depend`**（运行时防御性导入，构建期不需要，避免强制装 MoveIt2） | ✅ 已改 |
| P1-10 | 删除 `.cph-ng/`（268 B IDE 残留，已跟踪、零引用）。**本轮澄清：它不是 CoppeliaSim**——解码后是 VS Code *Competitive Programming Helper* 的元数据（`{"version":"0.7.8",...,"src":{"path":".../yolo_vision/README.md"}}`） | ✅ `git rm` 完成，删对了 |
| P1-8 | `ros_gz_bridge.yaml` 改为顶层 list + 保留直通话题名 + 补点云条目 | ✅ 已改；**已用 `yaml.safe_load` 校验**：顶层为 `list`、6 条、每条要么 `topic_name` 直通要么 `ros_`/`gz_` 成对。旧文件实测为 `dict`（即 D19 的实证） |
| P1-9 | `yolo_bringup/setup.py` 补 `glob('config/*.rviz')` | ✅ 已改；已验证 glob 命中 `config/grasping.rviz`，`py_compile` 通过 |
| P1-1 删包 | `yolo_vision/` + `yolo_coco/`（250,393 个文件）用户已决定**全删**，见 §5 问题 4 | ✅ 已完成（详见上表 P1-1 前置/验证） |
| D19/D25/D26 核实 | 旧桥接 YAML 确为 `dict`（D19）；`block_dataset/yolov8n.pt` 确无法 `torch.load`（D25）；gitlink `ce16f5a5…` 对象确认不在库中（D26） | ✅ 均已实测确认 |
| **P1-3** | **权重收敛**：删 14（根 `yolov8n.pt`＝与 `block_best.pt` 同 blob 的纯重复、`ros2_ws/yolov8n.pt`＝COCO 副本且路径即 CWD 陷阱、损坏件、10×`epoch*.pt`）；改名 `yolov8n_backup.pt`→`yolov8n_coco.pt` | ✅ 21→7 个，`find` 实测；现存 7 个文件名**全部与内容相符** |
| **D6/D24 修复** | `detector_node.py` 新增 `_resolve_model()`（绝对路径→`$YOLO_MODEL_DIR`→CWD 及各级父目录→包 `models/`），**找不到即报错退出、禁止联网兜底**；加载后打印类别数并对非 1 类 `block` 告警。4 个文件 5 处默认值改为 `block_best.pt` | ✅ 已实测：从仓库根 / `ros2_ws/` / `ros2_ws/src/yolo_detector/` 三种 CWD 均解析到 `block_best.pt`；`/tmp` 下返回 `None`（即会报错而**不是**下载 COCO） |
| **D6 根因拆除** | `train_yolo_block.py`：删掉「把新模型覆盖到根 `yolov8n.pt`、原文件备份为 `yolov8n_backup.pt`」那段——**这正是「同名不同物」的制造机**；微调起点改为显式常量 `BASE_WEIGHTS = yolov8n_coco.pt` 并校验存在性 | ✅ `py_compile` 通过；旧行为已无法复现 |
| **其余脚本同步** | `test_minimal.py` 改为显式加载 `block_best.pt`（原会触发联网下载）；`demo_pipeline.py` 的 `MODEL_NAME` 改为 `yolov8n_coco.pt` 绝对路径 | ✅ 均已 `py_compile` |
| **D30 隔离** | `demo_pipeline.py` 的 `DATASET_DIR` 改到独立 `block_dataset_demo/`（原与另两个标注器共用 `block_dataset/` 且块尺寸不同 → 脏数据集） | ✅ 一行隔离；已加 `.gitignore` |
| **D31 修复** | `bringup.launch.py` 四个被声明却硬编码忽略的 launch 参数改用 `LaunchConfiguration` | ✅ 已补 import；`world:=` / `model:=` / `mode:=` / `device:=` 现在真正生效 |
| **D32 文档更正** | `YOLO_TUTORIAL.md` §2.1 布局图与 §2.3 参数按 `generate_grid_world.py` 真值重写（3×3、8 方+1 圆柱、0.04 m；随机＝默认、`--fixed`＝调试、当前世界 seed=824517619） | ✅ 已改 |
| **Q11 结案** | 删除 `Moveit2YoloObb` 悬空 gitlink 条目（你已授权「可以删掉条目」），参考内容保留在 `docs/reference/Moveit2YoloObb/` | ✅ `git rm --cached` + 清空目录；已核实参考内容完好 |

**仍被阻塞：**

| 项 | 阻塞原因 |
|---|---|
| P1-5 | 「设计目标 vs 已实现」的标注需逐文档核对，属收尾工作；`TECHNICAL_SPEC.md` 未动 |
| P1-7 | 大文件出库涉及**历史**（**8.4 GB** `.git`）。注意：**删工作区文件不会让仓库变小**——`epoch*.pt` 与视频的对象仍在历史里，真正瘦身必须改写 `main`，需你授权（＝§5 问题 7） |

**已解除：** ~~P1-6~~（Q10/Q11 均已结案：内容找回 + 条目删除）、~~P1-2~~（D10 已完成）、~~P1-3~~（已完成）。

**验收**：`git status` 干净；无文档引用不存在文件；每个模型权重可追溯来源；仓库体积显著下降。

> ⚠️ **关于体积的一个硬事实**：`yolo_coco/` 9.3 GB（其中 `data/coco` 7.2 GB + `data/coco10/_raw` 1.6 GB 全是**公开 COCO 数据**，可重新下载；`assets/` 仅 1.3 MB 但是教材配图）、`yolo_coco/outputs/` 138 MB。**从工作区删除不会让仓库变小**——这些对象仍在 `.git` 历史里（`main` 上的 251,146 个跟踪文件）。要真正瘦身必须改写历史，而 `MIGRATION_HANDOFF.md` 明确要求「未经用户确认不得改写 `main`」。因此删包只买到干净的工作区，**买不到体积**。

### P2 — 感知链路修复（真机阻塞项）

按优先级，**D33/D1/D2/D8 不修则真机 100% 失败**；其中 **D33 是「链路根本没通」**，
其余三条是「通了也是错的」。

> **第六轮进展（2026-09-26）**：`projector_node.py` 已按 P2-1/2/3/6 重写并**通过回归测试**。
> 新增 `scripts/test_perception_3d.py`（20 项断言，不需仿真器/不需 `colcon build`），
> 覆盖 D1 尺度、D2 同步、D33 TF 类型、P2-6 最近簇四组行为。
> 命令：`source /opt/ros/humble/setup.bash && python3 scripts/test_perception_3d.py`（退出码 0 = 全通过）。
>
> **第七轮进展（2026-09-26）——「链路端到端跑通」首次被实证**：
> P2 余项（P2-4/5/7）完成，并新增 `scripts/test_pipeline_e2e.py`：它**真起话题、真走 DDS**，
> 把合成的 `camera_info` + 深度图 + 检测框喂进**正在运行的** pipeline，再断言 `/target_3d` 的**具体数值**。
> 实测结果（`camera_source:=external`，TF 为单位阵）：
>
> | 场景 | 解析预期 | 实测 `/target_3d` |
> |---|---|---|
> | 方块居中、深 0.5 m | `(0, 0, 0.5)` | `(0.0000, 0.0000, 0.5000)` ✅ |
> | 方块移至 (120, 85) | `x=0.14440, y=0.09025, z=0.5` | `(0.1444, 0.0903, 0.5000)` ✅ |
>
> 两条断言各自的含义：① **D33 确已修复**——`/target_3d` 真的被发布出来了（此前**一次都没有**）；
> ② **P2-6 在真实链路生效**——测试框内 3/4 面积是 1.0 m 桌面、中央 1/4 是 0.5 m 方块，
> 输出取到 **0.5 而非 1.0**，即「取最近簇」确实剔掉了背景平面；
> ③ D1 尺度在真实链路上量级正确（`32FC1 → 1.0 m/px`，`z=0.5`）。
>
> ⚠️ **这次验证的边界（别读过头）**：喂进去的是**合成**的检测框与深度，
> 不是真实相机、也不是 Gazebo 世界；相机外参用的是**单位阵**（即假设标定完全准确）。
> 所以它证明的是「**给定正确的输入，链路算得对且发得出来**」，
> 而**不**证明「检测框本身正确」或「外参已标定」——后两者分别归 YOLO 模型与手眼标定。

| 编号 | 任务 | 实现要点 | 状态 |
|---|---|---|---|
| P2-0 | **修 D33 TF 类型错误** | `do_transform_pose` 要的是 `Pose`，原码传的是 `PoseStamped`，且 `AttributeError` 不被 `except` 捕获 → `/target_3d` **从未发布**。改用 `do_transform_pose_stamped` | ✅ **已完成** |
| P2-1 | **修 D1 深度尺度** | 按 `msg.encoding` 查表：`16UC1`/`mono16` → `×0.001`（毫米），`32FC1`/`64FC1` → `×1.0`（米）。未知编码**报 ERROR** 并回退 1.0；`depth_scale` 参数（默认 `-1`＝自动）可强制覆盖。**在入口一次性换算成 float32 米**，下游不再关心编码 | ✅ **已完成** |
| P2-2 | **修 D2 对齐与同步** | 改为**带时间戳的环形缓冲**（`deque(maxlen=depth_buffer_len)`，默认 10 帧），按 stamp **就近配对**；超过 `max_sync_dt`（默认 0.1 s）**丢弃并告警**——宁可不发，也不发一个基于错误时刻深度的坐标。另在启动时提示真机需 `align_depth:=true` | ✅ **已完成**（未采用 `message_filters`，理由见下） |
| P2-3 | **修 D8 相机外参 TF** | 缺 TF 时由 `warn`+静默 `continue` 升级为 **ERROR + 连续失败计数**，并给出「先做手眼标定/发布静态 TF」的可操作提示；新增 `tf_required` 参数（默认 `False`），置真则直接抛错退出 | ✅ **已完成** |
| P2-4 | 修 D6 默认模型 | `pipeline.launch.py` 默认改为方块模型 ✅；`device` 退化：抽出**模块级** `resolve_device(requested, allow_fallback, logger)`，`cpu` / `cuda` / `cuda:N` 逐一校验（`torch` 是否可导入、`cuda.is_available()`、序号是否越界、序号是否合法），不可用则**告警并退回 cpu**——旧实现 `self.model.to(device)` 会直接让**整个节点崩溃**。新增 `device_fallback` 参数（默认 `True`），置 `False` 则抛 `RuntimeError`（真机上用来暴露环境问题）。刻意做成模块级函数以便**不装 torch 也能测**（`scripts/test_detector_device.py`，19 项） | ✅ **已完成**（RTX A2000 实测：`cuda:0` → 用 GPU；`cuda:9` → 告警退回 cpu） |
| P2-5 | 消除 D7 约定冲突 | 选**方案一（冻结像素约定）**：`detector_node.py` 与 `projector_node.py` 两侧 docstring 均写明「像素坐标、原点左上、**不归一化**」，并列出下游依赖点。**并加运行时护栏**：`projector_node` 若发现 `center` 与 `size` 全部落在 `(0,1]`（归一化的典型特征），立即打一次 ERROR 点名「上游可能发了归一化坐标」——D7 的坑从**静默算错**变成**可见告警**（只报一次，不刷屏） | ✅ **已完成** |
| P2-6 | 深度有效性增强 | 取检测框内 **5×5 多区域**采样（±60% 半宽内，避开混入背景的框边缘），再取**最近簇**中位数（`depth_cluster_tol`，默认 0.03 m）。俯视下物体顶面必比其身后桌面更近，故「最近的一簇」即物体本身 | ✅ **已完成**（**已在真实链路上验证**：框内 3/4 是 1.0 m 桌面、中央 1/4 是 0.5 m 方块，输出取到 **0.5**） |
| P2-7 | 加真机启动配置 | **设计变更（见下）**：原计划加 `use_sim` / `use_realsense` **两个布尔量**，实现时改为**单个枚举** `camera_source` ∈ {`sim`, `realsense`, `external`}。`pipeline.launch.py` 按它决定是否拉起 `realsense2_camera`（`align_depth.enable:=true`，D2 的硬要求）并**自动把 `depth_topic` 默认值切到对齐话题**；三个节点的深度/内参/检测话题全部参数化。`bringup.launch.py` 透传该参数，并**据此决定是否启动 Gazebo**；仿真才需要 6 s 延时，真机取 0 | ✅ **已完成** |

> **为何没用 `message_filters`（P2-2 的原方案）**：`ApproximateTimeSynchronizer` 要求
> **两个话题的消息都到齐**才触发回调，而本项目 `/detections` 的帧率远低于深度（检测是
> 推理瓶颈）。一旦用严格同步，**深度帧会被迫等待检测**，实际变成「以检测为准的抽样」，
> 且**首帧必然互相等待**；更麻烦的是它会**吃掉检测回调的时序语义**（同步器自己维护队列，
> 超过队列长度的消息**静默丢弃**，又是一个「不报错的失效」——正是 D8 那类问题）。
> 环形缓冲 + 就近配对实现了同样的「按时间戳关联」，但**失配时能明确告警**，
> 且不改变回调的驱动源。若后续需要精确到毫秒的对齐，再换 `message_filters` 不迟。

> **为何 P2-7 不用两个布尔量（原计划 `use_sim` / `use_realsense`）**：
> 两个布尔量存在「**都为真**」这种无意义状态，必须额外写校验代码去拒绝它；
> 枚举 `camera_source` 从类型上就不存在这个状态。三者对上层节点**完全透明**
> （话题名一致），所以 `yolo_detector` / `perception_3d` / `grasp_planner` 的参数不受影响。

#### P2 附带查明的环境事实（本轮实测，直接影响「能不能跑起来」）

这几条不在原缺陷清单里，但不写下来会反复踩：

| # | 事实 | 后果与对策 |
|---|---|---|
| **E1** | **`ultralytics`/`torch` 在 conda 环境 `robot-vision` 里，`rclpy` 在 `/opt/ros/humble` 里，两者互不可见**。| 直接 `ros2 launch` 会看到 `detector_node` 崩于 `No module named 'ultralytics'`。**两个 3.10 解释器可以叠起来用**（实测 `rclpy` + `torch 2.5.1+cu121` + `ultralytics 8.3.99` + `cv2` 同时可用）：<br>`source /opt/ros/humble/setup.bash` 后 `export PYTHONPATH=<venv>/lib/python3.10/site-packages:$PYTHONPATH`。**Orin 上应改为直接把 ultralytics 装进系统 Python，不要依赖这层叠加** |
| **E2** | **加了 venv 的 `PYTHONPATH` 后 `colcon build` 会失败**：`error: option --uninstall not recognized` | venv 里 `setuptools 84.0.0` 已移除 `setup.py develop --uninstall`，而 colcon 对 ament_python 包会先调它；系统 `setuptools 59.6.0` 还有。**结论：构建用纯净 ROS 环境，运行才叠 venv**（两者分开）。另 `--symlink-install` 同样会触发该错误，**用普通 `colcon build`** |
| **E3** | **WSL2 上 DDS 组播发现不稳定**：同一份 e2e 脚本「一次全过、一次一条消息都收不到」 | 表现为极难排查的假失败。**对策：`export ROS_LOCALHOST_ONLY=1`**（两个终端都要）。⚠️ **这是工作站开发用的规避手段，不要写进仓库的 launch 文件**——Orin 上机械臂/上位机可能要跨机通信，loopback-only 会切断 |
| **E4** | **float32 深度图（640×480 = 1.23 MB）把本机 DDS 通道压到约 9 MB/s**（实测：目标 20 Hz，实发 7.6 Hz，订阅端只收到 3.6 Hz） | 直接后果是「检测↔深度」时间戳差达 **431 ms** > `max_sync_dt`（100 ms），检测被**全部丢弃**——看起来像链路 bug，实为传输吞吐。**对策**：e2e 脚本改用 160×120（内参同比缩放，投影公式结果不变）；**真机侧 D435i 是 `16UC1`（600 KB），比 float32 轻一半**，但仍建议留意 `max_sync_dt` 与帧率是否匹配 |
| **E5** | 本机 GPU 实为 **RTX A2000**，而教材文本写的是「RTX 2060」 | 教材与实际硬件不符（D22 同类）。不影响代码，但写文档/报环境时别照抄教材 |

> 另有两条**未能证实、故不作为缺陷记录**的观察，留待仿真跑通后复核：
> ① `use_sim_time` 在本仓库**从未被设置过**（launch/参数/节点里都搜不到），而
> `simulation.launch.py` 又桥接了 `/clock`——若仿真下时间戳不一致，D2 的「就近配对」
> 会整体失配。**这需要一次真实仿真运行才能确认，本轮不据推测下结论。**
> ② `projector_node.py` 的 `depth_val <= 0.0` 判断**拦不住 NaN**（NaN 的比较恒为 False）。
> 已实测 `_robust_depth()` 的采样用 `np.isfinite` 过滤过，**当前不会**返回 NaN，
> 故这不是一个可触发的 bug；但仍加了一条 `math.isfinite` 兜底——真机上下游若拿到
> NaN 目标位姿，MoveIt 轻则报错、重则规划出 NaN 轨迹。

**验收**：仿真中已知世界坐标的方块，`/target_3d` 误差 < 2 cm；真机上对尺子量取的目标误差 < 2 cm（D435i 在 0.5–1 m 量程）。

> ⚠️ **验收现状（2026-09-26，第七轮）**：
> - ✅ **「链路算得对且发得出来」已达成并留证**：`scripts/test_pipeline_e2e.py` 真起话题、
>   真走 DDS，`/target_3d` 数值与解析预期一致（见上表）。这补上了此前唯一的空白——
>   在此之前**整条链路从未端到端跑通过**。
> - ⏳ **「误差 < 2 cm」尚未验收**，因为它的前提还不具备：需要①有真实相机（仿真或 D435i）、
>   ②有可信的 `base_link → camera_color_optical_frame` 外参。**真机下这一步必须做手眼标定，
>   代码层面修不了**（D8 只让失败可见，不产生外参）。
> - ⏳ 因此 `scripts/test_perception_3d.py`（纯逻辑）与 `test_pipeline_e2e.py`（合成输入）
>   全绿，**都还不等于「真机可用」**。下一个真实里程碑是**仿真里对着已知世界坐标量误差**。

### P3 — 抓取链路引入（仿真）

| 编号 | 任务 | 说明 |
|---|---|---|
| P3-1 | 解压 `grasp_sim` + 其 `yolo_vision` 到工作区 | 注意：**用附件内的 `yolo_vision` 保持其内部坐标约定自洽**，或适配为 P2-5 决定的统一约定——二选一，不要混 |
| P3-2 | 源码编译 `gz_ros2_control`（`humble` 分支） | apt 版仅支持 Fortress，Harmonic 必须源码编译 |
| P3-3 | 锁定 `setuptools==68.0.0` | 附件 README 已记录：≥79 会破 `ament_python` |
| P3-4 | 预生成 URDF/SRDF | `xacro … -o /tmp/ur5.urdf` + `-o /tmp/ur5.srdf` |
| P3-5 | 跑通 `full.launch.py` 仿真抓取 | 验证 9 物体依次抓取 |
| P3-6 | 接入主线感知 | 让 `perception_3d` 的输出风格与 `grasp_planner` 的输入对齐（附件版内部自己做投影，需决定是**复用**还是**替换**——见下） |

> **P3-6 是本次整理最需要你拍板的技术点**：附件的 `grasp_planner.py` 内部自带 `_optical_to_world()` 与 `_publish_camera_tf()`，即**它自己做了 2D→3D**，与 `ros2_ws/src/perception_3d` 职责重叠。两种走法：
> - **(A) 复用附件内部投影**：改动最小，直接跑通；但 3D 逻辑散落在抓取节点里，无法单独测试，且不做 D1/D2 修复的话真机仍会错。
> - **(B) 抽出统一 `perception_3d`**：把附件的投影逻辑并入 `perception_3d` 并加上 D1/D2/D8 修复，抓取节点只消费 `/target_3d`。**架构更干净，也是真机唯一可行路径**，但改动量大。
>
> 我建议 **(B)**，理由：真机上深度对齐与尺度转换必须在感知层统一处理一次，散在抓取节点里无法复用也无法验证。

**验收**：仿真中 9 个物体全部抓取成功；`/target_3d` 可被独立 echo 验证；抓取节点可单独重启不影响感知。

### P4 — Orin 迁移

| 编号 | 任务 | 说明 |
|---|---|---|
| P4-1 | **板端只读盘点** ✅ **已完成 2026-09-26** | 见下方实测清单。全程只读，未做任何修改、未驱动机械臂 |
| P4-2 | 参数化环境脚本 | 修 D9：把 `install_deps.sh` / `environment.yml` 的发行版与 Python 版本参数化，或另写 `install_deps_humble.sh` |
| P4-3 | 建立 OS 层 Python 环境规范 | **禁止 conda 参与 ROS**（附件 README 已强调）；`which python3` 必须是 `/usr/bin/python3` |
| P4-4 | Jetson 版 PyTorch | 用 NVIDIA 官方 aarch64 wheel，**不要用通用 pip torch**——通用轮子不含 Jetson CUDA |
| P4-5 | TensorRT 加速（可选但推荐） | 在**板上**生成 engine；8 GB 显存下 YOLOv8n FP16 预期显著优于 PyTorch |
| P4-6 | 相机标定 | 用 `camera_calibration` 实测内参，替换仿真内参；做手眼标定得到 P2-3 的静态 TF |
| P4-7 | 真机感知单独验收 | 只跑 D435i + 检测 + 投影，**不接机械臂**，验证 D1/D2/D8 修复 |

**P4-1 实测清单（2026-09-26，SSH 只读）**

板子：用户 `jetson@<板载 IP>`。

> 🔒 **板地址、主机名与 SSH 接入方式（别名、密钥文件名、`~/.ssh/config` 位置）按
> `MIGRATION_HANDOFF.md` 的安全约定「不入库」**，已从本文件移除——Git 历史是永久的，
> 即使当前不推送也不应落入。这些值在**本地笔记**里，需要时自行查阅。
> 同理，任何命令、日志、提交信息中都不要带上它们。

| 项 | 实测值 | 与计划的对照 |
|---|---|---|
| 架构 / 内核 | `aarch64` / `5.15.148-tegra` | — |
| OS | **Ubuntu 22.04.5 LTS**（jammy） | ✅ 与 `MIGRATION_HANDOFF.md` 一致，**不是**教材写的 24.04 |
| L4T / JetPack | **R36.4.3** = **JetPack 6.2**，`TEGRA_CHIPID 0x23`（T234 = Orin） | ✅ 与迁移文档一致 |
| 系统 Python | `/usr/bin/python3` = **3.10.12** | ✅ |
| numpy | **1.26.4** | ✅ 已是 `<2`，**cv_bridge 兼容性天然满足**（见 D23） |
| setuptools | 59.6.0（apt 自带） | ✅ 未受 ultralytics 影响 |
| ROS | **humble** | ✅ |
| CUDA | `/usr/local/cuda-12.6` | ✅ |
| TensorRT | **10.7.0** | ✅ P4-5 的前提已具备 |
| torch | **2.5.0a0+872d972e41.nv24.08**，`cuda_ok True` | ✅ 已是 **NVIDIA Jetson 轮子**（非通用 pip），P4-4 **已完成** |
| ultralytics | **8.4.26** | ✅ 比工作站的 8.3.99 还新 |
| RealSense | `librealsense2 2.58.4` + **`realsense2-camera 4.58.4`** 均已装 | ✅ P4-6 前提具备 |
| 磁盘 | 233 G 总 / 57 G 已用 / **166 G 可用** | ✅ 充裕 |
| 内存 | **7.4 GiB** 总，已用 3.2 Gi，free 仅 568 Mi（available 4.0 Gi） | ⚠️ 偏紧——板上已有 `arm-isolation-sudo.sh` 在做资源隔离 |
| 仿真器 | **完全没有**：`gz`/`ign`/`gazebo` 可执行均不存在，`ros_gz_*`、`gz-harmonic`、Gazebo Classic 全未装 | ✅ **符合预期**：板子接真机，不需要仿真 |
| conda | **无**（不在 PATH，家目录也无 miniforge） | ✅ 少一个坑 |

**板上已装的真机链路（关键发现——比我预想的完整）：**

| 已装 ✅ | 缺失 ❌ |
|---|---|
| `ur-robot-driver`、`ur-client-library`、`ur-msgs`、`ur-calibration` | `ros-humble-moveit`（P5-3 规划验证要用） |
| `controller-manager`、`joint-trajectory-controller` | `ros-humble-robotiq-description` |
| `xacro`、`ur-description`、`ur-moveit-config` | `ros-humble-ros2-control` / `ros2-controllers`（**元包**未装，但**实际组件 `controller-manager` + `joint-trajectory-controller` 已在**） |
| `joint-state-publisher`、`robot-state-publisher`、`cv-bridge` | 任何仿真器（不需要） |
| `colcon-common-extensions`、`pip`、`git` | |

> **P4-4（Jetson 版 PyTorch）实际上已经满足**——板上的 torch 是 `nv24.08` 的 NVIDIA 构建且 CUDA 可用。计划里这条可以降级为「已具备，勿用通用 pip 覆盖」。

**板上其他资产（非本项目，但需知道）：**

- `/home/<user>/yahboom_ws/`（`camera` / `interfaces` / `largemodel` / `text_chat`）、`workspaces/isaac_ros-dev`（**NVIDIA Isaac ROS** 工作区）、`ultralytics/`（源码克隆）、`torchvision/`、`yolo26_data/`、`qr_code_recognition/`。
- **Docker 已装**（`docker0` = 172.17.0.1/16）。
- `/home/<user>/arm-*.sh` 六个脚本：`arm-ready.sh`（性能调优）、`arm-check.sh`（体检）、`arm-isolation-sudo.sh`（资源隔离：控制栈绑 4-5 号核 + 内存保底 1G；桌面压到 0-3 核）、`arm-headless-check.sh`、`arm-cleanup-sudo.sh`、`arm-grow-root.sh`。**这些是本项目在板上做的部署准备，但不在本仓库里**——`arm-isolation-sudo.sh` 的注释引用了「体检清单 D4」，而那份清单我在板上和仓库里都没找到。建议纳入本仓库 `docs/`（见 §5 问题 11）。
- `/dev/video0..5` 六个视频设备在位；**当前无 `/dev/ttyUSB*`、`/dev/ttyACM*`**。

**验收**：板端 `ros2 topic hz /detections` 稳定 ≥ 15 Hz；对已知位置目标 `/target_3d` 误差 < 2 cm；全程未驱动机械臂。

### P5 — 真机抓取（高风险，需逐次授权）

| 编号 | 任务 | 前置条件 |
|---|---|---|
| P5-1 | 核实 UR5 接入方式 | 控制器型号 / 网络 IP / 示教器配置 / 夹爪接口——**不得假设 USB**，UR 常见为网络 RTDE |
| P5-2 | 只读连通性验证 | `ur_robot_driver` 启动、读关节状态、`dashboard` 查询，**不运动** |
| P5-3 | MoveIt 规划验证 | 仅规划+RViz 可视化，**不执行** |
| P5-4 | 低速单步动作 | 急停可用、工作区隔离、单关节小幅运动 |
| P5-5 | 完整抓取闭环 | 逐次明确授权 |

**安全红线**：每个真机动作都需要你当次明确授权；急停必须可用；工作区必须隔离。当前代码的 `moveit` 模式**不得**在任何情况下被当作可用实现（D3）。

---

## 4. 逐文件改动清单（P1 + P2 摘要）

| 文件 | 动作 | 关联缺陷 |
|---|---|---|
| `ros2_ws/src/perception_3d/perception_3d/projector_node.py` | ✅ **已改（第六轮整文件重写；第七轮续改）**：TF 改用 `do_transform_pose_stamped`；深度按 encoding 换算成米后入**带时间戳环形缓冲**、按 stamp 就近配对并在超差时丢弃告警；框内 5×5 采样取**最近簇**中位数；TF 失败升级为 ERROR + 连续计数 + `tf_required` 开关。第七轮：三处话题名参数化（`camera_info_topic`/`depth_topic`/`detections_topic`）以配合 P2-7；归一化护栏（P2-5）；发布前的 `math.isfinite` 兜底。共 **9 个参数** | D33 D1 D2 D8 P2-5 P2-6 P2-7 |
| `scripts/test_perception_3d.py` | ✅ **新增（第六轮）**：`projector_node` 回归自检，24 项断言，覆盖 D1/D2/D33/D8/P2-5/P2-6；**不需要仿真器、不需要 `colcon build`**（按源码路径直接导入），退出码 0 = 全绿 | D33 D1 D2 D8 P2-5 P2-6 |
| `scripts/test_detector_device.py` | ✅ **新增（第七轮）**：`resolve_device()` 全分支自检，19 项；**不导入 torch、不实例化 ROS 节点**（用假 logger + `sys.modules` 注入假 torch 模拟有/无 GPU）。⚠️ 模拟「torch 未安装」必须用 `sys.modules['torch'] = None`，`pop` 掉会被真装的 torch 重新导入（本轮实测踩到） | P2-4 |
| `scripts/test_pipeline_e2e.py` | ✅ **新增（第七轮）**：**端到端**验证——真起话题、真走 DDS，把合成 `camera_info`+深度+检测框喂进运行中的 pipeline，断言 `/target_3d` 的**具体数值**。这是唯一能回答「链路是否真的通」的脚本 | D33 D1 P2-6 |
| `ros2_ws/src/perception_3d/package.xml` | 补 `numpy` | — |
| `ros2_ws/src/yolo_detector/yolo_detector/detector_node.py` | ✅ **已改**：`_resolve_model()` 路径解析 + 找不到即报错 + 类别自检；默认值 `block_best.pt`。第七轮：抽出**模块级** `resolve_device()` 做设备退化（+`device_fallback` 参数）；坐标约定（**像素、不归一化**）写进 docstring 并列为「冻结」 | D6 D24 D7 P2-4 P2-5 |
| `ros2_ws/src/yolo_bringup/launch/pipeline.launch.py` | ✅ **已改**：默认模型 `block_best.pt`；第七轮加 `camera_source` 枚举（`sim`/`realsense`/`external`），`realsense` 时拉起 `realsense2_camera`（`align_depth.enable:=true`）并**自动把 `depth_topic` 默认值切到对齐话题**；另加 `tf_required` | D6 D2 P2-7 |
| `ros2_ws/src/yolo_bringup/launch/bringup.launch.py` | ✅ **已改**：`world`/`model`/`mode`/`device` 四个原先被硬编码忽略的 launch 参数改用 `LaunchConfiguration`（D31）；第七轮透传 `camera_source`/`target_frame`/`tf_required`，并**据此决定是否启动 Gazebo**（真机不起仿真）与 6 s 延时是否生效 | D31 P2-7 |
| `ros2_ws/src/perception_3d/launch/perception.launch.py` | ✅ **已改（第七轮）**：暴露上述新增参数 | P2-7 |
| `scripts/train_yolo_block.py` | ✅ **已改**：拆除「覆盖根 `yolov8n.pt`／备份为 `yolov8n_backup.pt`」的逻辑（D6 的制造机）；微调起点改为显式 `yolov8n_coco.pt` 并校验存在 | D6 |
| `scripts/test_minimal.py`、`scripts/demo_pipeline.py` | ✅ **已改**：前者显式加载 `block_best.pt`；后者 `MODEL_NAME`→`yolov8n_coco.pt`、`DATASET_DIR`→独立 `block_dataset_demo/` | D6 D30 |
| 权重文件（14 删 / 2 改名） | ✅ **已做**：见 §3 P1 完成表与 §1.4 实测总表 | D6 D13 D25 |
| `ros2_ws/src/yolo_bringup/launch/simulation.launch.py` | **改**：删死变量 `bridge_config`（`:45`）、接上 `gui_arg`（`:28-31`）、把 `rviz_node` 加入返回列表（`:108-114` 未在 `:116-130` 返回）、`TimerAction` 时序参照附件 `TEXTBOOK.md:771` 的 8 段表重做（现为裸 3.0 s）；**并改用 D19 修好的桥接 YAML**；复核 `:89` 的 `JointState@gz.msgs.Model` 映射 | D14 D15 D16 D17 D19 |
| `ros2_ws/src/yolo_bringup/setup.py` | **改**：`config` 的 `data_files` 补 `glob('config/*.rviz')`——否则 D16 修好也读不到 `grasping.rviz` | D16 |
| `ros2_ws/src/yolo_bringup/config/ros_gz_bridge.yaml` | **改**：按附件正确格式重写（顶层 list + `ros_topic_name`/`gz_topic_name`） | D19 |
| `ros2_ws/src/grasp_planner/**` | `package.xml` 已补 `exec_depend moveit_msgs`（✅ 已完成）；其余**决策**：废弃或改造为 P3-6(B) 的薄封装。另：`tf2_ros`/`tf2_geometry_msgs` 声明未用、`MoveItErrorCodes` 导入未用，P3 改造时一并清理 | D3 D12 |
| `ros2_ws/src/yolo_bringup/worlds/grasping_world.sdf` | **改**：要么补机械臂，要么改名去掉 "grasping" | D4 |
| `yolo_vision/**`、`yolo_coco/**` | **删除**——⚠️ 但 `docs/chapter03_yolo_perception.md` 的 §3.4/§3.5 依赖它们，**先处理教材正文与 5 张配图**（§5 问题 9） | D21 |
| `docs/chapter03_yolo_perception.md` | **改**：§3.3.2/§3.3.3 的 Jazzy 改为 Humble（与附件及 Orin 对齐）；§3.4/§3.5 视问题 9 的选择重写或保留 | D21 D22 |
| `docs/YOLO_VISION_DESIGN.md` | **改**：全文按 `yolo_vision` 包写（`:230-466` 多处命令与目录树），删包后需重指向 `ros2_ws/src/yolo_detector` | D21 |
| `scripts/install_deps.sh`、`environment.yml` | **改**：参数化发行版与 Python 版本 | D9 |
| `README.md`、`docs/TECHNICAL_SPEC.md` | **改**：区分「设计目标」与「已实现」 | D4 D10 |
| `docs/YOLO_TUTORIAL.md` | ✅ **已改**：死引用（D10）、环境名（D29）、世界布局与参数（D32）、模型名与 `test_minimal.py` 说明、§7.2 的 `-p model:=` 均按真实情况更正 | D10 D29 D32 |
| `README.md` | ✅ **已改**（部分）：脚本树补全 + 环境名更正。**仍待** P1-5 的「设计目标 vs 已实现」标注 | D10 D29 |
| 6 个分叉脚本 | **收敛**（根 `scripts/` 为权威版，已执行）；`yolo_vision/scripts/` 随包删除 | D11 |
| 13 组模型权重 | **收敛 + 哈希记录** | D13 |

---

## 5. 待确认问题

**已答复：**

| # | 问题 | 结论 |
|---|---|---|
| 1 | P3-6 技术路线 | 路线 **(B)** 抽出统一 `perception_3d` |
| 2 | `yolo_vision/` 去留 | **删除**（教学包，有独立备份） |
| 3 | `yolo_coco/` 去留 | **删除**（教学包，有独立备份） |
| 4 | 教材 `chapter03` 依赖待删包（D21）怎么办 | **全删**。用户明确：**教材项目已完结、也不在此仓库开发**，本仓库现在的定位是「把这个开发过的项目再利用起来」，不是继续写教材。故章节图链失效可接受 |
| 5 | 仿真后端 | **Humble + Gazebo Harmonic，本机装**（已确认）。Orin 上不装仿真器 |
| 6 | Orin 板地址 | ✅ **已自行找到**（🔒 具体地址与 SSH 接入方式**不入库**，见本地笔记）。P4-1 已完成 |
| 7 | Python 环境策略 | 用**独立环境**，不动系统 Python。本机已有 conda（`/home/<user>/robotics/venvs/miniforge3`）及其 `robot-vision` 环境（numpy 1.26.4 / torch 2.5.1+cu121 / ultralytics 8.3.99），**numpy 已是 `<2`，cv_bridge 兼容性天然满足**（D23 的顾虑消解） |
| 8 | 以谁为准 | **以 Orin 的兼容性和版本为准**（用户强调）。文档没写或不确定的，直接 SSH 板子确认 |

**仍待你答复：**

4. ✅ **CoppeliaSim 阶段（本轮已查清，无需再答）**：对**全部 8 个提交**做了路径级检索（`git log --all --name-only`），**没有任何 `coppelia`/`cph` 踪迹**，且 `.cph-ng/` 经解码证实**不是 CoppeliaSim**——它是 VS Code **Competitive Programming Helper** 扩展的元数据（268 B gzip JSON，`{"version":"0.7.8",...,"src":{"path":".../yolo_vision/README.md"}}`），与仿真无关，**删对了**。结论：**CoppeliaSim 阶段不在本仓库历史中**，本仓库里的仿真自始至终是 **Gazebo**。若那段资产存在，只可能在外部目录或其他仓库——见问题 10。
5. ✅ **【已解决】真机是 UR5**。板上 `~/arm-isolation-sudo.sh:3` 直接写明 **`# Slice for robot arm control (UR5 / ROS2)`**，且第 12 行写「**等真正开始跑 UR5 控制节点时再执行更合适**」——这是比 `arm-check.sh` 的串口探测更明确的意图声明。原先的「矛盾」解释为：**CH340/串口探测属 Yahboom 车体外设，与机械臂无关**（板上 `yahboom_ws/src = camera, interfaces, largemodel, text_chat`，另有 `qr_code_recognition`/`mediapipe`/`CH341SER`/`yolo26_data`，是 Yahboom 套件，不是臂）。**故 P5 按 UR5 + Robotiq 2F-85 + 以太网 RTDE 编制。**
   **仍需现场确认的只剩两点**（不影响现在动工）：(a) UR 控制柜当前 IP 属于哪个网段——板上只见 `eno1 = <板载 IP>/23`，**未见** UR 默认的 `192.168.1.x`，说明**臂还没接上或尚未配置**；(b) 现场接线时以哪块网卡对接。
7. **仓库体积**：工作区已从 9.3 GB 降到 **405 MB**（见 §3 P1）。但**历史仍有 8.4 GB 的 `.git`**——要让仓库真正变小必须在**历史**上动刀（`git filter-repo` / LFS 迁移），那会改写 `main`，`MIGRATION_HANDOFF.md` 明确要求需你确认。**是否处理、如何处理，请你定。**
8. **模型 A 的来源确认**：`runs/block_detect/args.yaml` 是否就是模型 A 的训练配置？（P1-3 会据此补全记录）
9. ⚠️ **板上那六个 `arm-*.sh` 要不要纳入本仓库？** 它们是本项目在 Orin 上的部署准备（性能调优 / 资源隔离 / 无头检查 / 清理 / 扩容），**只存在于板子上**，且 `arm-isolation-sudo.sh` 引用了一份我在板上和仓库里都没找到的「**体检清单**」（其 D4 项）。建议连同那份清单一起放进 `docs/`——否则这部分知识只活在板子家目录里，重装即失。
10. ✅ **【已解决】`Moveit2YoloObb` 已找回。** URL 在仓库内两处文档中就有（`docs/GRID_ENV_DESIGN.md:327`、`docs/YOLO_VISION_DESIGN.md:4`）：
    **`https://github.com/laoxue888/Moveit2YoloObb`**。实测该 gitlink 记录的 commit **`ce16f5a5…` 在上游仍然存在**，已按原版本精确检出到 `docs/reference/Moveit2YoloObb/`（43 MB，附 `PROVENANCE.md`，已删其 `.git`）。
    **但它不是 UR5 项目——是 Franka Panda + YOLOv8-OBB + MoveIt2**（详见 D28）。所以它**不能直接移植**，价值在于：
    - **补上 D27 的洞**：它的 `coordinates` 就是 OBB 的四个角点（8 个浮点），**提供了本项目完全缺失的朝向角**；`[x, y, θ]` 三元素即位置+偏航。
    - **提供抓放序列范本**：`arm_control_from_UI.py:listener_callback` 的「接近→开爪→下降→闭爪→抬升→平移→开爪」+ 三个高度常量。
    - 移植基底仍是 **`docs/reference/ur5-yolo-grasp/`**（UR5，与本项目机器人/发行版/仿真后端都一致）；本目录只借架构与思路。
11. ✅ **【已结案】`Moveit2YoloObb` gitlink 条目——已按 (a) 删除。** 你已授权「可以删掉条目」，故执行：
    `git rm --cached Moveit2YoloObb` + 清掉遗留的空目录。**参考内容完整保留在 `docs/reference/Moveit2YoloObb/`**（43 MB，含 `PROVENANCE.md`，该目录本就不入库）。
    理由同原倾向：仓库自洽、不依赖一个**无法解析**的外部指针；需要参考时本地即有。
12. **【新增·已决定，供你复核】D27（朝向）按「已知限制」处理，不实施。** 依据是第五轮的实测反证：
    - `generate_random_blocks()` **只扰动 x/y，没有 yaw 字段**；meta JSON 里也没有任何旋转信息；
    - `block_grid_world.sdf:205` 方块几何是 `<size>0.0400 0.0400 0.0400</size>`——**立方体**；
    - `assign_colors_and_shapes()` 只产出 `box`（立方体）或 `cylinder`（旋转对称）。
    即：**当前物体集中没有一个物体具有可辨朝向**，立方体对 90° 偏航不变、圆柱对任意偏航不变，平行夹爪**不需要对齐**。
    且在这个数据集上训 OBB 模型会**学不到东西**（偏航方差恒为 0）。
    故 **P5 可直接移植 planner，不被朝向阻塞**；D27 转为文档中的「已知限制 + 何时会变必需」。若你后续要抓**非立方**物体，请告知——届时第一步是**给世界生成器加 yaw 随机化**，否则仿真无法验证朝向。

---

## 6. 下一步

**P0 ✅、P4-1 ✅、P1 大部分 ✅**（删包与脚本收敛已完成，见 §3）。**两个前置阻塞都已解除：真机＝UR5（Q5），参考抓取代码已找回（Q10）**，故 P5 的设计工作现在可以真正开始。

**已完成的（本轮新增）：**

- ✅ **找回 `Moveit2YoloObb`**：URL 就在仓库文档里（`GRID_ENV_DESIGN.md:327`、`YOLO_VISION_DESIGN.md:4`）＝ `github.com/laoxue888/Moveit2YoloObb`；其 gitlink 记录的 commit `ce16f5a5…` **在上游仍存在**，已精确检出到 `docs/reference/Moveit2YoloObb/`（43 MB，带 `PROVENANCE.md`）。
- ⚠️ **但它不是 UR5 项目，是 Panda + YOLOv8-OBB + MoveIt2**（D28）→ 定位为**架构参考**，移植基底仍是 `ur5-yolo-grasp`。
- ⬇️ **D27（朝向）经查证后降级为「已知限制」**：原先判定它是「P5 必修前置项」，但实测反证——世界生成器**无 yaw 字段**、方块是**立方体**、形状只有 `box`/`cylinder` 两种（都朝向对称）。故平行夹爪**不需要对齐**，P5 不被阻塞；且该数据集训 OBB 会学不到东西（偏航方差恒 0）。**你已确认按此处理。**
- **解决 Q5**：板上 `arm-isolation-sudo.sh` 自述 `UR5 / ROS2`，确认真机为 **UR5**。
- **权重全量实测**：21 个 `.pt` → 7 份互异模型，含 1 份**损坏文件**（D25）与 1 份**下一代 yolo26n**；**同名不同物**确证（D6/D13/D18 全部修正）。
- **确认附件自带完整抓取栈**（xacro/SRDF/kinematics/ros2_control），可作 Orin 上**不依赖 `robotiq-description`** 的基底（§2 决定 2）。
- **排除 CoppeliaSim**：全历史无踪迹；`.cph-ng` 实为 CPH 扩展元数据，删对了（§5 Q4）。

**第五轮（2026-09-26）新增完成项：**

- ✅ **P1-3 权重收敛完成**：**21 个 `.pt` → 7 个**。删 14（根 `yolov8n.pt`、`ros2_ws/yolov8n.pt`、损坏件、10×`epoch*.pt`）、改名 2（`yolov8n_backup.pt`→`yolov8n_coco.pt`）。**每个现存文件名都与内容相符**，仓库内**再无任何代码引用 `yolov8n.pt`**。
- 🎯 **D6 根因查明并拆除**：不是历史意外，而是 `train_yolo_block.py` **主动把新模型覆盖到根 `yolov8n.pt`、把原文件备份成 `yolov8n_backup.pt`**——「同名不同物」是这段代码的必然产物。已删除该逻辑，微调起点改为显式 `yolov8n_coco.pt`。
- ✅ **D6/D24 部署侧修复**：`detector_node.py` 新增路径解析（绝对路径→`$YOLO_MODEL_DIR`→CWD 及父目录→包 `models/`），**找不到即报错退出，联网兜底被彻底堵死**；加载后打印类别数，非 1 类 `block` 时告警。
- ✅ **新增 D31**：`bringup.launch.py` 四个 launch 参数「声明了但被硬编码忽略」——`world:=` / `model:=` **静默无效**，已改用 `LaunchConfiguration`。
- ✅ **新增 D30**：三个标注器**共用同一个 `block_dataset/`** 却用不同块尺寸（0.03 vs 0.04）→ 会产出「图像与标注来自不同几何假设」的**脏数据集**。已把 `demo_pipeline.py` 隔离到 `block_dataset_demo/`。
- ✅ **新增 D32**：教程 §2 把 **`demo_pipeline.py` 的私有常量**（5×5 / 0.03 / `CYLINDER_POSITIONS={(1,1),(2,3),(3,2)}`）当成了**世界生成器**的常量（真值 3×3 或随机 / 0.04）——这解释了 D30 里数值分叉的由来。已按真值重写。
- ✅ **新增 D29**：文档通篇教的 `conda activate yolo_ros`（及 `yolo_coco`/`yolo_vision`）**三个环境一个都不存在**；实际是 `robot-vision`，且因是 **prefix 环境必须写全路径**。已修 `README.md` + `YOLO_TUTORIAL.md`。
- ✅ **Q11 结案**：删除 `Moveit2YoloObb` 悬空 gitlink 条目，参考保留在 `docs/reference/`。

**第六轮（2026-09-26）新增完成项：**

- ⚠️ **新增 D33（本轮最重的发现）**：`projector_node.py` 把 `PoseStamped` 传给只接受 `Pose` 的 `do_transform_pose`，抛出的 `AttributeError` **不在 `except` 捕获范围内** ⇒ **`/target_3d` 从未被发布过一次**。README 里「3 节点流水线联调通过」只验证了节点起得来，**没有一个字节的数据流过**。换句话说，**整条感知链此前从未端到端跑通**，D1/D2/D8 都排在它后面。已改 `do_transform_pose_stamped` 并实测签名。
- ✅ **P2-0/P2-1/P2-2/P2-3/P2-6 完成**，`projector_node.py` 整文件重写：深度按编码换算成米、带时间戳环形缓冲就近配对（超差丢弃告警）、框内 5×5 采样取最近簇、TF 失败升级为 ERROR + 连续计数 + `tf_required` 开关。新增六个参数。
- ✅ **新增 `scripts/test_perception_3d.py`**：20 项断言，覆盖 D1/D2/D33/P2-6，**不需仿真器、不需 `colcon build`**，实测**全绿、退出码 0**。
- 📌 **P2-2 没走原方案的 `message_filters`**，理由已写进 P2 小节（会引入新的「静默丢弃」）。若日后需要毫秒级对齐再换。

**第七轮（2026-09-26）新增完成项：**

- ✅ **P2 余项全部收尾**：P2-5（冻结「像素、不归一化」约定 + 归一化护栏）、P2-4（`resolve_device()` 设备退化）、P2-7（`camera_source` 枚举开关，`bringup` 透传，**真机不起 Gazebo**）。
- ✅ **链路首次端到端实证**：新增 `scripts/test_pipeline_e2e.py`，实测 `/target_3d` = `(0.0000, 0.0000, 0.5000)` 与 `(0.1444, 0.0903, 0.5000)`，**与解析预期逐位吻合**。这直接证明 **D33 确已修复**（此前该话题从未被发布），并证明 **P2-6 的「最近簇」在真实链路上生效**（框内 3/4 是 1.0 m 桌面，仍取到 0.5 m 的方块）。
- ✅ **新增 `scripts/test_detector_device.py`**：19 项，全分支覆盖，不装 torch 也能跑。
- 🎯 **查明 E1–E5 五条环境事实**（见 P2 小节）：`ultralytics` 与 `rclpy` 分居两个解释器、venv 的 setuptools 84 会让 `colcon build` 失败、WSL2 组播发现不稳需 `ROS_LOCALHOST_ONLY=1`、1.23 MB float32 深度把 DDS 压到约 9 MB/s、本机 GPU 实为 **RTX A2000**（教材写的 RTX 2060 有误）。
- 📌 **P2-7 实现时改了设计**：原计划两个布尔量 `use_sim`/`use_realsense` → 实际用**单枚举** `camera_source`（避免「都为真」的无意义状态）。

**下一步可立即做的（均不阻塞）：**

1. ~~**P2 余项**~~ ✅ 已完成（P2-4/5/7 全部收尾，链路已端到端验证）。
2. **P1-5** 文档「设计目标 vs 已实现」标注（`TECHNICAL_SPEC.md` 未动）。
3. **P5 移植抓取栈**：D27 已降级，朝向**不再是前置阻塞**，可直接以 `docs/reference/ur5-yolo-grasp/` 为基底移植（注意换成模型 A）。
4. ~~**P1-3** 权重收敛~~ ✅ 已完成（21→7，文件名与内容全部对齐）。
5. **装 Harmonic**（你已确认）：sudo 部分已给你命令；装完我做 `~/grasp_ws` 工作区 + `gz_ros2_control` 源码编译。

> ⚠️ **端到端的边界（第七轮更新后，边界缩小了但仍存在）**：
> `test_pipeline_e2e.py` 已经证明「**给定正确的检测框与深度，链路算得对、也发得出来**」——
> 这补上了此前最大的空白。但它喂的是**合成**输入、用的是**单位阵**外参，所以：
> - 「链路代码是否通」→ ✅ **已证**；
> - 「检测框本身是否准」→ 归 YOLO 模型，未在真机/仿真上量过；
> - 「外参是否准」→ **归手眼标定，代码层修不了**（D8 只让失败可见，不产生外参）。
>
> 下一个真实里程碑因此是：**在仿真里对已知世界坐标的方块量 `/target_3d` 误差**。
> 那需要先装好 Harmonic（上面第 5 条）。

> ⚠️ **尚未提交**：第五、六轮所有改动都在 `migrate/humble-orin` 的工作区/暂存区，**没有任何提交**，`main` 仍停在 `f250af11b`。要不要提交、如何分组，等你示下。

**等你的两件事：**

| # | 事项 | 影响 |
|---|---|---|
| 7 | 仓库**历史** 8.4 GB 是否改写（会动 `main`，需明确授权） | 仓库体积；不改也能正常工作。**注意：本轮删掉的 245 MB `epoch*.pt` 仍留在历史里**，只有改写历史才能真瘦身 |
| 9 | 板上六个 `arm-*.sh` 与那份「体检清单」是否纳入 `docs/` | 避免板子重装即失传 |

**Orin 侧的 P4 剩余项**（P4-1 已证明板子基础很好）：缺 `ros-humble-moveit` 与 `ros-humble-robotiq-description`；`torch`/`ultralytics`/RealSense/UR 驱动**都已就绪**。P4-4（Jetson 版 PyTorch）实际已完成，勿用通用 pip 覆盖。
