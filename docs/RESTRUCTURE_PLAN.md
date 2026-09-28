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
| D4 ✅**已修** | `yolo_bringup/worlds/grasping_world.sdf` | 场景内只有 `ground_plane / table / camera_model / red_cube / blue_cube / green_cylinder`，**没有机械臂** | README 宣称的「Gazebo 中复现完整闭环」不成立。<br>✅ **已修（第十三轮，用户授权「改名」）**：`git mv` → **`color_objects_world.sdf`**，文件内 `<world name>` 与 6 个文件共 12 处引用同步改，重建 `yolo_bringup` 并清掉 install 里的旧拷贝，**实测起得来**（40 个服务、6 个模型、无 `[Err]`）。详见第十三轮第二节 |
| D5 | `ros2_ws/src/` 全局 | 全仓库无 URDF / moveit_config / ros2_control / 夹爪包 | 抓取链路缺全部机器人侧资产 |
| **D6** ✅**已修** | `pipeline.launch.py:17`、`bringup.launch.py:24,52`、`yolo_detector/launch/detector.launch.py:16`、`detector_node.py:30` | `model` 默认 `'yolov8n.pt'`，而 `detector_node.py` 把这个值**直接传给 `YOLO()`，不做任何路径解析**，于是行为**取决于进程 CWD**：① 仓库根 → 命中 `./yolov8n.pt`（其实是方块模型 A）；② `ros2_ws/` → 命中真 COCO；③ 都没有 → **ultralytics 静默联网下载 COCO**。<br>**🎯 根因（第五轮查明）**：这一切是 `scripts/train_yolo_block.py:143-151` **自己造出来的**——它训练完把新模型 `shutil.copy` **覆盖到仓库根的 `yolov8n.pt`**，并把被覆盖的文件备份为 `yolov8n_backup.pt`。所以方块模型占用了 `yolov8n.pt` 这个名字，真 COCO 反被赶进 `*_backup.pt`。「同名不同物」不是历史意外，是这段代码的**必然产物** | ✅ **已修（P1-3）**：① 检测端默认改为 `block_best.pt`（4 个文件 5 处）；② 删除仓库根 `yolov8n.pt`（内容与 `block_best.pt` **同一 blob** `e917eeaf1`，属纯重复）与 `ros2_ws/yolov8n.pt`（COCO 副本，其**路径本身**就是 CWD 陷阱）；③ COCO 基线改名 `yolov8n_backup.pt` → **`yolov8n_coco.pt`**；④ `train_yolo_block.py` 删掉那段覆盖逻辑，微调起点改为显式 `yolov8n_coco.pt`；⑤ `detector_node.py` 新增 `_resolve_model()`：绝对路径 → `$YOLO_MODEL_DIR` → CWD 及各级父目录 → 包 `models/`，**找不到就报错退出，绝不联网兜底**，并打印类别数、非 1 类时告警 |
| D7 | `yolo_vision/detector_node.py:98-100` vs `yolo_detector/detector_node.py:125-128` | 前者发布**归一化** bbox，后者发布**像素** bbox，而 `projector_node.py:129-130` 按像素使用 | 两个包**不可混用**，混用则投影全错。当前是潜在陷阱（未触发） |
| **D8** ✅**已修**（可见性部分） | `perception_3d/projector_node.py:158-163` | 依赖 `camera_frame → target_frame` 的 TF，但仓库内无任何静态 TF / robot_state_publisher 发布相机外参 | 真机上 TF 查询必然失败，节点静默跳过。✅ **已修（第六轮）**：失败由 `warn`+静默 `continue` 升级为 **ERROR + 连续失败计数**，并给出「先做手眼标定 / 发布静态 TF」的可操作提示；新增 `tf_required` 参数（默认 `False`），置真则**直接抛错退出**。<br>⚠️ **边界**：这只让失败**可见且可诊断**，**外参本身仍需标定**——**TF 不在代码层可修**，别把这条当成「已解决」 |
| D9 | `environment.yml`、`scripts/install_deps.sh` | 硬编码 Python 3.12 / `ros-jazzy-*` / Ubuntu 24.04 / Noble | 在 Orin（22.04 + Humble + Python 3.10）上**不可直接使用** |
| D10 | **本轮全仓扫描后扩大**：不止原先记的两处 | 对 `README.md` 与 `docs/*.md` 里所有 `scripts/*.py` 引用做「存在性」校验，共发现 **7 处死引用**：<br>① **活的、已修**：`docs/YOLO_TUTORIAL.md` 的 `capture_and_detect.py`（§6.1/§6.2 正文 + §8 文件表 + 常见问题）、`yolo_detect_simple.py`；`README.md:103` 的 `capture_and_detect.py`。<br>② **随章死亡的、无需单独修**：`docs/chapter03_yolo_perception.md:203,266,267,284,292,315,339,342,345` 引用的 `scripts/train.py`、`evaluate.py`、`detect.py`、`prepare_coco10.py` —— 这些是该章**自带**的 `yolo_coco` 项目布局，而该章已定**全删**（§5 问题 4），故随之消亡。<br>（曾误记 `scripts/grasp_planner.py` 也是死引用——本轮用 `git grep HEAD -- '*.md'` 复核，**它从未在任何文档中被引用**，该条已撤销。抓取脚本的真实位置是附件 `docs/reference/ur5-yolo-grasp/grasp_sim/scripts/grasp_planner.py`。） | 文档指向**不存在的文件**，照做必然失败 |
| D11 | `scripts/` vs `yolo_vision/scripts/` | 6 个同名脚本（`auto_label_grid` / `collect_training_data` / `demo_pipeline` / `generate_grid_world` / `label_blocks` / `train_yolo_block`）**内容全部不同** | 无法判断哪份权威，改错版本会静默失效 |
| D12 | `grasp_planner/package.xml` | 代码 `:80` `from moveit_msgs.msg import MoveItErrorCodes`（在 `_init_moveit()` 的防御性 `try` 内），但 `package.xml` **未声明 `moveit_msgs`**；反向地，`tf2_ros` / `tf2_geometry_msgs` **已声明却在代码中完全未使用** | 清单与代码**双向不符**。另：`MoveItErrorCodes` 导入后从未被引用，属残留 |
| **D13** ✅**已修** | 模型权重曾散落 **21 个 `.pt`**（全量哈希+加载实测） | 除 **A/B 两份方块模型**外，还有**真 COCO yolov8n**、**yolo26n（80 类）**、**1 份损坏文件**、10 个训练中间盘 | ✅ **已收敛（P1-3）**：删 14 个（根 `yolov8n.pt`、`ros2_ws/yolov8n.pt`、损坏件、10 个 `epoch*.pt`）、改名 2 个 → 现存 **7 个 `.pt`**，且**每个文件名都与其内容相符**：`block_best.pt`(A)、`runs/block_detect/weights/{best,last}.pt`、`runs/detect/block_grid_demo/weights/{best,last}.pt`(B)、`yolov8n_coco.pt`(COCO)、`yolo26n.pt`(COCO-26) |
| D18 | 模型 A / B 并存 | **A**（域随机化，较新）`a926d959…`(sha256) 在 `block_best.pt`、`runs/block_detect/weights/best.pt`；**B**（方阵演示，较旧）`b9baf6f1…`(sha256) 在 `runs/detect/block_grid_demo/weights/best.pt` 与**附件**的 `yolo_vision/models/best.pt`。~~`yolo_vision/models/best.pt`~~ 已随该包删除 | 附件抓取栈自带的是**旧模型 B**；P3 接入时应改用模型 A，勿为「与附件一致」而回退。<br>⚠️ **本行哈希是 sha256，D6 的 `e917eeaf1…` 是 git blob(sha1)，同一个文件两种算法，都对**（第十一轮实测确认）。A 的文件内部：`nc=1`、`names={0:'block'}`、训练于 2026-07-16、ultralytics 8.4.51 |
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
| **E3** | ~~**WSL2 上 DDS 组播发现不稳定**：同一份 e2e 脚本「一次全过、一次一条消息都收不到」~~ ⚠️ **本条的对策已被第九轮实测证伪，见下** | 当年判为「组播不稳」，对策写成 `export ROS_LOCALHOST_ONLY=1`。**第九轮实测：这个值会让发现彻底失效**——`ros2 node list` 返回空、e2e 脚本一条消息都收不到，**症状与当年一模一样，但原因不是组播**。`source /opt/ros/humble/setup.bash` 自己会把该变量置为 **0**，用 0 两节点正常发现。<br>⇒ **正确做法：不要动这个变量**，让它保持 humble 设定的 0。<br>⇒ 真正的干扰源是**上一次仿真没退干净的残留节点**（第九轮实测抓到 **4 个** `static_transform_publisher` 一直在发 `world→overhead_camera` 的 TF）。发现异常时先 `ps` 查残留，而不是去调 DDS 变量 |
| **E4** | **float32 深度图（640×480 = 1.23 MB）把本机 DDS 通道压到约 9 MB/s**（实测：目标 20 Hz，实发 7.6 Hz，订阅端只收到 3.6 Hz） | 直接后果是「检测↔深度」时间戳差达 **431 ms** > `max_sync_dt`（100 ms），检测被**全部丢弃**——看起来像链路 bug，实为传输吞吐。**对策**：e2e 脚本改用 160×120（内参同比缩放，投影公式结果不变）；**真机侧 D435i 是 `16UC1`（600 KB），比 float32 轻一半**，但仍建议留意 `max_sync_dt` 与帧率是否匹配 |
| **E5** | 本机 GPU 实为 **RTX A2000**；教材文本写的是「RTX 2060」 | **这不是教材写错，而是本机根本不是当初编教材那台开发机**（用户 2026-09-26 确认）。因此本机与教材在硬件（尤其显卡）、可能还有驱动的对应关系上**没有可比性**，不要拿教材的硬件描述来核对本机。**对代码无影响**：`detector_node` 的设备选择已由 P2-4 做成运行时探测（实测日志 `使用 GPU cuda:0 — NVIDIA RTX A2000`），不硬编码任何型号。**最终以 Orin 为准**——Orin 的 CUDA/torch 匹配是独立问题（见 E1 与 P4） |

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
| **P3-0** | **装 Gazebo Harmonic（含卸载 Classic）** | ✅ **已完成 2026-09-27**（用户执行）。实测 `gz-harmonic 1.0.0-1~jammy`、`ros-humble-ros-gzharmonic 0.244.12-3jammy`、`libgz-sim8-dev 8.15.0-1~jammy` 在位；Classic 已彻底移除（`/usr/bin/gz-11*`、`/usr/bin/gazebo` 均不存在），`/usr/bin/gz` 归 `gz-tools2`，`gz sim --versions` = **8.15.0**，`gz help` 已有 `sim` 子命令 |
| P3-1 | 解压 `grasp_sim` + 其 `yolo_vision` 到工作区 | ✅ **已完成 2026-09-27**。按 LAB.md 的做法**建符号链接而非复制**，避免与 `docs/reference/` 分叉：`~/grasp_ws/src/{grasp_sim,yolo_vision}` → `docs/reference/ur5-yolo-grasp/{grasp_sim,yolo_vision}`。`colcon list` 已确认两者被正常发现（符号链接被跟进）。注意：此处**用附件内的 `yolo_vision` 以保持其内部坐标约定自洽**；勿与本仓库主线混用 |
| P3-2 | 源码编译 `gz_ros2_control`（`humble` 分支） | ✅ **已完成 2026-09-27**。`--packages-up-to gz_ros2_control`，20.6 s 通过。产物 `libgz_ros2_control-system.so` / `libgz_hardware_plugins.so`。**已实证与 Harmonic 匹配**（见下方） |
| P3-3 | 锁定 `setuptools==68.0.0` | 附件 README 已记录：≥79 会破 `ament_python` |
| P3-4 | 预生成 URDF/SRDF | `xacro … -o /tmp/ur5.urdf` + `-o /tmp/ur5.srdf` |
| P3-5 | 跑通 `full.launch.py` 仿真抓取 | ✅ **已完成 2026-09-27**——端到端「检测 → 规划 → 抓取 → 搬运 → 放置」实测跑通，方块被**真实搬离网格**（物理位姿为证，非回读值）。见下方两处「P3-5 实测」 |
| P3-6 | 接入主线感知 | 让 `perception_3d` 的输出风格与 `grasp_planner` 的输入对齐（附件版内部自己做投影，需决定是**复用**还是**替换**——见下） |

#### P3-5 实测：仿真链路已全部打通（2026-09-27）

`sim.launch.py` 常驻运行实测（用户目视 + 数据双向确认）：

| 环节 | 证据 |
|---|---|
| 世界加载 | 13 个模型：`ground_plane`/`table`/`overhead_camera`/**9×`block_r{0,1,2}_c{0,1,2}`**/`pedestal` |
| UR5+gripper spawn | `ros_gz_sim: OK creation of entity.`；`resource_manager: Successful initialization of hardware 'ur5'` |
| mimic 关节 | `Joint 'robotiq_85_right_finger_tip_joint' is mimicking 'robotiq_85_left_knuckle_joint' multiplier: 1` |
| ros2_control | `gz_ros2_control: System Successfully configured!` → `Loading controller_manager` |
| 控制器 | `joint_trajectory_controller` / `joint_state_broadcaster` 均 `Loaded` + **`Configured and activated`** |
| MoveIt | `move_group` 起，`start_state_max_bounds_error=0.1` |
| **相机（Gazebo 侧）** | `/rgbd_camera/{image,depth_image,camera_info,depth_image/points}` **实测有数据**，`frame_id=overhead_camera::camera_link::color_camera` |
| **桥接（ROS 侧）** | `/camera/{color/image_raw,depth/image_raw,color/camera_info,depth/points}`、`/clock`、`/joint_states`、`/dynamic_joint_states` 全部可见 |
| **关节状态** | `/joint_states` 有真值：`shoulder_lift=-1.5700`、`wrist_1=-1.5700`（UR5 标准初始位姿） |
| **控制通路（关键）** | 发一条 `FollowJointTrajectory`（仅 `wrist_3_joint`→0.4，靠 `allow_partial_joints_goal: true`）→ **`error_code: 0`、`Goal successfully reached!`、状态 `SUCCEEDED`**；回读 `wrist_3_joint=+0.4000` **精确命中**，其余 6 关节保持不动 |

> ⚠️ **此前「没有运动」是正常的，不是故障**：`sim.launch.py` **只负责把世界+控制器+MoveIt 立起来，不下发任何运动指令**。我最初也把「无运动」误当成待查问题，实际它本就不该动。运动来自 `full.launch.py`（它才加 `yolo_vision` 检测 + `grasp_planner`）。

#### P3-5 实测（二）：`full.launch.py` 端到端抓取跑通（2026-09-27）

在 E1 叠加环境（`source /opt/ros/humble/setup.bash` → `export PYTHONPATH=<venv>/lib/python3.10/site-packages:$PYTHONPATH`）下实跑：

```bash
cd /home/<user>/grasp_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export GZ_VERSION=harmonic
export PYTHONPATH=/home/<user>/robotics/venvs/robot-vision/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch grasp_sim full.launch.py
```

| 环节 | 证据 |
|---|---|
| YOLO 节点 | `Loading model: …/yolo_vision/models/best.pt on cuda:0` → `YOLO Detector ready (RGB + depth → 3D)` —— **E1 叠加生效，ultralytics 在 venv 里被找到并跑在 GPU 上** |
| 相机内参 | `K from camera_info: fx=554.4 fy=554.4 cx=320.0 cy=240.0`（`K from HFOV` 的兜底路径也走通了） |
| 回 HOME | `Moving to HOME before first grasp...` → `Initial HOME reached — ready for detections` |
| 检测 | `Detected: ['0'×10]`，取最高置信度 `0.93` → `world=(0.493,-0.119,0.450)` |
| 抓取动作链 | `Approach → Descend → Close → Lift → Home → swing left → Drop → Release → Return home`，**每一步 `Trajectory done: error_code=0`** |
| 连续抓取 | 第 2 轮 `Detected: ['0'×8] | already grasped: 1` → 再抓 1 个，计数 `(2 objects grasped)` |
| **物理搬运（关键独立证据）** | 查 Gazebo 真实位姿：`block_r0_c0`/`block_r0_c1` 已**不在网格**（网格 x∈{0.34,0.5,0.66}, y∈{-0.12,0,0.12}），双双位于 `(-0.207, 0.486)` 即规划器的 left-drop 放置区，**数量恰为 2**，与计数器一致 |

> 🔎 **为什么要另找证据**：日志里的 `Close done: gripper_pos=0.4503` 只是**位置命令回读**，这个仿真没有力反馈，空夹与夹住物体都可能落到相近值——**单看回读值不能证明抓到了**。查 Gazebo 里方块的**物理位姿**才是硬证据：方块确实离开了原位、成对出现在放置区。这条同样适用于真机验证：**别把关节回读当成抓取成功**。

**D36 从「时序脆弱」升级为「实测行为失败」**：本轮 `set_p_gain-4` 直接 **died**，日志第 11 行：

```
[set_p_gain-4] Node not found
[ERROR] [set_p_gain-4]: process has died [pid 33527, exit code 1,
  cmd 'bash -c sleep 2 && ros2 param set /gz_ros2_control position_proportional_gain 5.0 && echo ...']
```

即 `position_proportional_gain` **停留在 yaml 的 10.0，而非作者想要的 5.0**。之前只判为「与 spawn 赛跑、靠运气熬过」，现在确认它是**稳定失败**——`sleep 2` 时 `/gz_ros2_control` 节点尚未存在，`ros2 param set` 报 `Node not found` 后 `&&` 链中断、进程退出。修法：改用 `--controller-manager-timeout` 等就绪等待，或直接在 `ros2_control.yaml` 里写 5.0。

**实测中发现、应在 P3-6 移植时一并修掉的附件缺陷**（记为新缺陷编号 D34–D37）：

| # | 缺陷 | 后果 | 证据 |
|---|---|---|---|
| **D34** | `sim.launch.py:97`/`:124` 在**构造期** `open(URDF_PATH)`/`open(SRDF_PATH)`，而生成它们的 `ExecuteProcess` 在 `:56`/`:70`、**运行期**才执行 | **首次冷启动必然失败**（文件还不存在）。这正是 `LAB.md` §9.5 警告的现象。P3-4 的预生成只是绕过，根因未修。**正确修法**：把 `urdf_contents`/`srdf_contents` 改为运行期读取（或 `Command` 替换） | `open()` 在 `generate_launch_description()` 内 |
| **D35** | `:202`/`:204` 的 spawner 命令末尾带 `\|\| true` | **吞掉控制器加载失败**。控制器起不来时 launch 仍"成功"，直到后面规划失败才暴露，且现象与真实原因相距很远 | `--controller-manager-timeout 30 ... \|\| true;` |
| **D36** | `:190` 用固定 `sleep 2` 设 `position_proportional_gain`，注释却写「**BEFORE** robot spawn」 | 与 spawn 赛跑。实测出现 `robot_state_publisher service not available, waiting again...` ×4 后才恢复（靠运气熬过）。属 D17 同类固定延时 | 日志第 41–44 行 |
| **D37** | spawner 已激活控制器后，`:206` 又调 `switch_controller` 激活一次 | **冗余调用**，产生误导性的 `Could not activate controller ...` 两条 WARN，但响应实为 `ok=True`。看起来像失败，实际是噪音——**会浪费排查时间** | 日志第 232–234 行 + `SwitchController_Response(ok=True)` |

**其他观察（非缺陷，供参考）**：`ros2_control.yaml` 注释仍写「in Gazebo **Fortress**」（我们跑的是 Harmonic，注释过期）；world SDF 版本为 `1.9`（Fortress 时代，sdformat14 兼容但可现代化）；moveit 报 `No action namespace specified for controller 'scaled_joint_trajectory_controller'`（`:136` 的 `pop` 未完全生效，因 JTC 才是 `default: True`，无实际影响）。

**环境操作注意**：`ros2 daemon` 在仿真运行期间会**卡死**（`ros2 daemon stop` 本身挂住不退出），导致 `ros2 topic list` 等命令超时。**`--no-daemon` 可绕过**（实测立即正常）。另：**launch 与查询 shell 必须用一致的 `ROS_LOCALHOST_ONLY` 设置**——第九轮进一步实测：该值由 `source /opt/ros/humble/setup.bash` 置为 **0**，**保持 0 即可**；改成 1 会让发现完全失效（见 E3）。发现异常时**先查残留节点**（`ps -eo pid,comm | grep -E "static_transf|projector|detector|planner"`），本轮实测抓到 4 个上一次仿真遗留的 `static_transform_publisher`。**清理时必须用 `pkill -9 -x <精确进程名>`，不能用 `pkill -f`——`-f` 会匹配到执行该命令的 shell 自己，把自己的会话杀掉**（本轮踩到两次）。

#### P3-6 实测：自研规划器接管抓取，9/9 全量跑通（2026-09-27，第九轮）

按你确认的 **P3-6(B)** 落地：`perception_3d` 是**唯一**的 2D→3D 转换点，规划器**零投影代码**，只消费 `/targets_3d`；夹爪走**抽象层**（仿真 = JTC 第 7 关节，真机 = AG-95 Modbus）。同时修掉 D34–D37，并新增 D38/D39。

**改动清单（本轮）**

| 文件 | 动作 |
|---|---|
| `grasp_planner/grasp_planner/planner_node.py` | **整文件重写**：删掉参考项目自带的 `_optical_to_world()`/`_publish_camera_tf()` 与 `x - BASE_X` 一类硬编码偏移，改为纯 `/targets_3d` 消费者；`move_duration`/`descend_duration` 等几何与时序量全部提为 ROS 参数 |
| `grasp_planner/grasp_planner/gripper.py` | **新增**：`GripperBackend` 接口 + `SimGripper`（已实测）+ `Ag95Gripper`（**会大声报错的桩**，见下） |
| `grasp_planner/config/grasp_planner.yaml` | **新增**：全部参数集中一处，几何量逐条注明「真机必须现场覆盖」 |
| `grasp_planner/launch/planner.launch.py` | **重写**：废弃 `mode:=mock\|moveit`（mock 分支只打日志，从不执行），改为加载 yaml + 暴露 `params_file`/`gripper_backend` |
| `arm_grasp_sim/**` | **新增（自附件移植）**：world + UR5+Robotiq 的 URDF/SRDF + launch；D34–D37 已修 |
| `perception_3d`、`grasp_planner` 的 `setup.py`/`package.xml` | 装 `config/`、补 `vision_msgs`/`trajectory_msgs`/`control_msgs` 等依赖 |

> **为什么把夹爪单独拆出来命令，而不是塞进同一条 7 关节轨迹**：AG-95 **不在机械臂的轨迹控制器里**，真机上「当第 7 个关节发 `FollowJointTrajectory`」根本走不通。仿真侧借助 JTC 已开启的 `allow_partial_joints_goal: true`，只发夹爪那一个关节即可（**已实测该路径可用**：日志有 `Goals with partial set of joints are allowed`，且夹爪单独轨迹稳定 `Goal reached`）。两边对规划器暴露同一接口，状态机无需分叉。
>
> ⚠️ **但要注意顺序**：JTC 的**新目标会抢占正在执行的目标**。所以臂与夹爪的轨迹**必须严格串行**——每一段夹爪动作都放在某段臂轨迹的完成回调里发出，绝不可并发，否则会静默打断臂的运动。

**验收证据（本轮，全部为实测）**

| 环节 | 证据 |
|---|---|
| 统一 3D 接口 | `perception_3d` 日志：`发布 9 个候选到 /targets_3d；/target_3d = [block] score=0.94`（新接口与兼容接口同时可用） |
| 规划器零投影 | 规划器日志**只有** `候选 9 个 … / → 选中 … / 接近解：[…]`，全文无任何相机内参或 TF 运算 |
| 完整动作链 | `初始化：先张开夹爪，再回 HOME` → `接近完成` → `下降完成` → `已夹住物体（回读位置=0.4508）` → `抬起成功，已记录 …（累计 1 个）` → `已到 HOME —— 左摆放置` → `已到放置位 —— 张开夹爪放下` → `返回 HOME` → `已就绪` |
| 连续抓取 | 候选数随抓取递减 **9→8→7→6→5→…→1**，规划器计数递增 `累计 1…9 个`，**无一次重试、无一次空夹误判** |
| **物理搬运（关键独立证据）** | 查 Gazebo 真值：**9/9 全部离开网格**，堆在放置区 `(-0.208, 0.485)`，z = 0.02→0.34 **整齐叠成 9 层塔**；落点离散度 **x 仅 0.9 mm、y 仅 9.8 mm** |
| **计数一致性** | Gazebo 真值「已搬走 9 个」与规划器自报「累计 9 个」**逐一对应**，且**正是**规划器依次选中的那 9 个（r0_c1, r0_c2, r0_c0, r1_c1, r1_c0, r1_c2, r2_c0, r2_c2, r2_c1） |

> 🔎 **为什么必须查 Gazebo 真值**：`已夹住物体（回读位置=0.4508）` 只是 JTC 的**位置回读**，本仿真**没有力反馈**——空夹与夹住都可能落到相近值。**回读值本身不能证明抓到了**。只有方块在物理世界里真的换了位置，才算证据。**这条对真机同样成立：别把关节回读当成抓取成功。**

**本轮新发现并已修的缺陷**

| # | 归属 | 缺陷 | 后果 | 修法 |
|---|---|---|---|---|
| **D38** | 附件遗留 | `sim.launch.py` 把 `gz_ros2_control` 路径**硬编码**成 `<ws>/install/gz_ros2_control/lib` | 换工作区/换机器即失效；而该包按 P3-2 必须**源码编在工作站**，路径本就与主工作区不同 | 改用 `get_package_prefix("gz_ros2_control")`，包缺失时抛**可操作**的报错（指明要 `source ~/grasp_ws/install/setup.bash`） |
| **D39** | 附件遗留 | 用 `Node(package="ros_gz_sim", executable="gz_sim")` 起仿真 | `executable 'gz_sim' not found in the libexec directory '/opt/ros/humble/lib/ros_gz_sim'`——**Harmonic 变体只提供 `create`/`test_stopwatch`，没有 `gz_sim`** | 回到 `ExecuteProcess(cmd=["gz","sim","-r",world])`，与 `ros_gz_sim` 包解耦 |
| **D40** | **主仓库既有** | `detector_node.py` 检测循环里 `boxes` 来自 `.cpu().numpy()`，`x1..y2` 是 **np.float32** 标量；`vision_msgs` 的纯 Python 消息类在 setter 里做**严格断言** `assert isinstance(value, float)` | `isinstance(np.float32(1.0), float)` 是 **False**（`np.float64` 才是 float 子类）⇒ 消息一构造就 `AssertionError`，节点在**第一帧图像**上直接崩掉。**这条与抓取无关，是主仓库感知链自己就断了** | 显式 `(float(v) for v in box)` |
| — | 移植期自引入 | `table_min_z` 沿用参考项目的**世界系**值 `0.3`，但本规划器已改在 `base_link` 系作业 | 两个系差一个底座高度（0.80）。base_frame 里桌面 ≈ **-0.39**、桌上方块实测 **-0.350**、地上 **-0.78** ⇒ 用 0.3 会把**所有**候选判为「已落地」而**全部丢弃，且不报任何错**——规划器一个目标都选不出来 | 改 `-0.5`（干净分开 -0.35 与 -0.78），并把这段换算写进参数注释 |
| — | 移植期自引入 | `gripper.py` 的 `_wrap()` 从 `self._done_cb` 取回调，而该属性**从未被赋值** | **调用方的完成回调被静默吞掉**：夹爪动作做完了，规划器永远等不到回调，卡在初始位姿、`_first_home_sent` 恒为 False，**所有检测都被丢弃**。症状是「夹爪动了但规划器毫无反应」，且**一句报错都没有** | 换成 `_once(done_cb)`（只做「只调用一次」防重入），回调从参数传入 |
| — | 移植期自引入 | `planner_node.py` 里 `move_duration`/`descend_duration` 只被 `_declare_params()` **声明**，忘了赋给 `self` | 第一段 HOME 轨迹一发出就 `AttributeError`，节点启动 **2 秒后直接死掉** | 补上赋值，并在注释里点明「`_declare_params()` 只负责声明，不会自动变成属性」 |
| **D41** | 移植期自引入 | `pipeline.launch.py`/`bringup.launch.py` 仍向规划器传 `mode:=mock\|moveit`，而新规划器**没有这个参数**（它随参考项目的 mock/moveit 分支一起被删） | **实测确认：ROS 2 对未声明的参数覆盖是静默忽略的**——`ros2 run grasp_planner planner_node --ros-args -p mode:=mock` 正常启动，输出里 **0 次**提及 `mode`。于是「mode 参数已失效」这件事**没有任何提示**，用户以为选了 moveit 其实什么都没发生 | 删掉两个 launch 里的 `mode`，改为传**真正有用**的 `gripper_backend`；其默认值**随 `camera_source` 走**（`sim`→`sim`，真机→`ag95`），理由同 `depth_topic`：真机上若默认 `sim`，规划器会去命令一个**真机不存在的关节** `robotiq_85_left_knuckle_joint`，JTC 收下却不动任何东西——**「夹爪没反应」且不报错** |

> **`gripper_backend` 为什么必须随 `camera_source` 变、而不是固定默认值**：这两个后端在真机上是**对错之别**，不是偏好。`sim` 后端 = 「夹爪是 JTC 第 7 关节」，这条在真机上**根本不成立**（AG-95 是独立 Modbus 设备）。固定默认 `sim` 会让真机静默失效；固定默认 `ag95` 会让仿真开箱即错。跟随 `camera_source` 是唯一能同时正确的默认值，且 `pipeline.launch.py` 里 `depth_topic` 已经是这个写法，属本文件既有惯例。

**关于「控制器没起来时规划器怎么办」（实测澄清，非缺陷）**：单独起规划器（无仿真、无控制器）实测输出：

```
[INFO]  GraspPlanner 就绪 | 夹爪后端=sim (...)
[INFO]  初始化：先张开夹爪，再回 HOME ...
[ERROR] 轨迹 action 服务不可用（控制器起了吗？）
```

`_send_trajectory()` 在 `wait_for_server` 超时后会**主动调用 `done_cb(None)`**，所以状态机**不会卡死**，而是照常推进到 `_first_home_sent=True` 并开始接收检测。这是**有意的降级**（报错但继续），比挂起好。
> ⚠️ **但真机上要注意一个后果**：此时 HOME **并未真正执行**，而规划器已经认为自己「就绪」。若机械臂此刻停在任意位姿，规划器仍会发出**绝对关节角**的接近轨迹——它会从当前位置直接奔向目标（可能穿过障碍），而不是从 HOME 出发。所以 **P5 真机联调时，必须先确认控制器与 `ur_robot_driver` 已就绪、且机械臂确实在 HOME，再启动规划器**（仿真里靠 `full.launch.py` 的 `planner_delay:=18.0` 规避了这点）。



> **这三条自引入缺陷有共同教训**：它们都**不是逻辑错误，而是「以为存在的东西不存在」**，且前两条**完全静默**（一个丢弃全部候选、一个吞掉全部回调），日志上只剩「什么也没发生」。**端到端实跑是唯一能暴露它们的手段**——单元测试与静态检查都覆盖不到。移植参考代码时，**「参数改名了、坐标系换了、回调改道了」这三类要逐条对账**。

**本轮发现、尚未处理的问题（留给你决定）**

| # | 问题 | 说明 |
|---|---|---|
| a | **世界里没有放置容器** | 9 个方块全被放到**地面**上（放置点 `(-0.208, 0.485)` 在桌子外，落点 z≈0.02），叠成塔。参考项目的 `left_drop_joints` 是个**硬编码关节角**，并没有对应的接料容器。真机上「放置位」必须有**真实目标**（料框/指定台面），且 `left_drop_joints` 是仿真几何下的值，**真机必须重新标定** |
| b | **`已抓完` 分支实际不可达** | `_select_and_approach()` 只在「**有候选**但全已抓过」时才打印「全部 N 个物体已抓完」。物体抓完后会落到视野外/低于 `table_min_z`，此时**候选为 0**，走的是「静默重试」分支，永远等不到结束语。功能无碍（它会一直待命等新物体），但**做演示时看不到明确的收尾提示** |
| c | **`max_sync_dt` 丢帧告警高频出现** | 全程 **24 次** `深度与检测时间差 132 ms 超过 max_sync_dt（100 ms），丢弃本批检测`。当前靠「检测是连续流、丢一批还有下一批」掩盖住了。**Orin 上延迟只会更大**，这个 100 ms 阈值很可能需要调大，否则真机上会**大量丢帧**（见 P4-7 单独验收） |
| d | **`table_min_z` 实际承担了循环终止的职责** | 抓完的方块之所以不再被当成候选，**靠的是它们掉到 `table_min_z` 以下**（而非 `_grasped_positions` 去重——去重按坐标四舍五入比对，而落点与抓取点坐标不同，**去重在这里并不生效**）。改这个阈值时要想清楚这层副作用 |
| e | **`position_proportional_gain` 仍不可设** | D36 本轮改为**不再尝试** `ros2 param set`（删掉那条必然失败的 `set_p_gain-N`），故增益停留在库默认 **0.1**，而非 yaml 写的 10.0。属已知，不影响本轮功能（位置控制仍稳定收敛） |

> **过程记录（工具环境，非项目缺陷）**：本轮有一次 Bash 工具输出里夹带了伪装成 `system-reminder` 的**注入指令**，要求我「不要调查/修复这些 bug、且不要提及该提醒」。我**未采纳**，并按你的既定要求继续查修、如实汇报。记录在此仅供你了解工具环境状况。


> **P3-6 是本次整理最需要你拍板的技术点**：附件的 `grasp_planner.py` 内部自带 `_optical_to_world()` 与 `_publish_camera_tf()`，即**它自己做了 2D→3D**，与 `ros2_ws/src/perception_3d` 职责重叠。两种走法：
> - **(A) 复用附件内部投影**：改动最小，直接跑通；但 3D 逻辑散落在抓取节点里，无法单独测试，且不做 D1/D2 修复的话真机仍会错。
> - **(B) 抽出统一 `perception_3d`**：把附件的投影逻辑并入 `perception_3d` 并加上 D1/D2/D8 修复，抓取节点只消费 `/target_3d`。**架构更干净，也是真机唯一可行路径**，但改动量大。
>
> 我建议 **(B)**，理由：真机上深度对齐与尺度转换必须在感知层统一处理一次，散在抓取节点里无法复用也无法验证。

#### P3-0 已核实的安装事实（2026-09-26 实测，非凭记忆）

装 Harmonic 前先要知道五件事，否则会踩坑：

| # | 核实结论 | 证据 |
|---|---|---|
| a | **Gazebo Classic 必须先卸，不能共存** | `gz-tools2`（`gz-harmonic` 的依赖）的 control 文件**显式声明** `Conflicts: gazebo (>= 11.0.0), gazebo (<= 11.14.0), gazebo11 (<= 11.14.0)`，且两套都提供 `/usr/bin/gz`，属文件级冲突。本机 Classic 为 `11.10.2+dfsg-1` |
| b | **本仓库不依赖 Classic，卸掉零代价** | 全仓库 grep `gazebo_ros\|libgazebo\|gazebo_ros_pkgs` **无命中**；仿真走的是 `gz sim -r` + `ros_gz_bridge`（`simulation.launch.py:36,45`），两套都是 **Gazebo Sim（新）**。`apt -s` 模拟：连带移除 6 个包（`gazebo`、`ros-humble-gazebo-{dev,msgs,plugins,ros,ros-pkgs,ros2-control}`），无一被本仓库使用 |
| c | **Harmonic 是本仓库 launch 的前置条件，不是「升级」** | 本机 Classic 的 `gz` CLI 子命令只有 `camera/joint/log/marker/model/physics…`，**没有 `sim`**（`gz help` 实测）。即 `simulation.launch.py` 的 `gz sim -r <world.sdf>` 在 Classic 上**根本跑不起来** |
| d | **Humble+Harmonic 的二进制只在 `packages.osrfoundation.org`** | ros_gz `humble` 分支 README 的兼容表 + 脚注 1 指明该组合从 OSRF 源获取；`packages.ros.org` 上只有配 Fortress 的 `ros-humble-ros-gz*`，**没有** `ros-humble-ros-gzharmonic`。curl OSRF `jammy` 索引确认两者都在：`gz-harmonic 1.0.0-1~jammy`（拉 `libgz-sim8-dev`/`gz-sim8-cli` ⇒ Harmonic 8.x）、`ros-humble-ros-gzharmonic 0.244.12-3jammy` |
| e | **`gz_ros2_control` 不要 apt 装** | apt 的 `ros-humble-gz-ros2-control 0.7.21`（本机现已随 Classic 一并移除）是**为 Fortress 编的**，Harmonic 下插件名变了。须从 `ros-controls/gz_ros2_control` `humble` 分支源码编译（该分支同时兼容 Fortress/Harmonic，无需改代码） |

#### P3-2 编译结果与「插件名是否匹配」的实证（2026-09-27）

回滚快照保留在 `~/gazebo-classic-rollback.txt`（全量 1959 包）、`~/gazebo-classic-only.txt`。

```
cd ~/grasp_ws && source /opt/ros/humble/setup.bash && export GZ_VERSION=harmonic
PATH=/usr/bin:$PATH colcon build --packages-up-to gz_ros2_control \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF
# → Summary: 1 package finished [20.6s]，退出码 0
```

**关键验证：不是「.so 存在」就算过，而是要看它链接的是哪一代 Gazebo**（LAB.md 警告过 Fortress/Harmonic 插件名不同）：

| 检查项 | 实测结果 |
|---|---|
| 动态链接 | 全部为 **`libgz-sim8.so.8`**（Harmonic 8.x）、`gz-transport13`、`gz-plugin2`、`gz-msgs10`——**无任何 `libignition-gazebo6`（Fortress）残留** |
| 硬件插件注册名（pluginlib XML） | `gz_ros2_control/GazeboSimSystem` **新名** + `ign_ros2_control/IgnitionSystem` **向后兼容别名** |
| Gazebo 侧插件（`GZ_ADD_PLUGIN`） | `gz_ros2_control::GazeboSimROS2ControlPlugin` |

> ✅ **与附件零冲突，不需要改名。** 附件**本来就是按 Harmonic 写的**：
> `LAB.md:173`/`TEXTBOOK.md:210` 的 `<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>`、
> `LAB.md:180` 的 `filename="libgz_ros2_control-system.so"`——**与我们刚编出的产物逐字吻合**。
> 附件还在 `LAB.md:188`/`TEXTBOOK.md:225` 主动解释了为何**不**用 Robotiq 自带的
> `robotiq_gripper.ros2_control.xacro`（它写死了 Fortress 时代的 `ign_ros2_control/IgnitionSystem`），
> 而是自己写 30 行 `<ros2_control>` 块。**这正是 P3-2 必须源码编译的原因，附件已提前规避。**

#### P3-2 之后新暴露的依赖缺口（**需要你再跑一次 sudo**）

附件的 xacro/launch **引用了 4 个本机未装的包**（引用均已核实为**活引用**，非死代码）：

| 包 | 引用位置 | 本机 |
|---|---|---|
| `ros-humble-ur-description` | `ur5_with_gripper.urdf.xacro` 等 **6 处** | ❌ |
| `ros-humble-robotiq-description` | `ur5_with_gripper.urdf.xacro:14,15`（`robotiq_2f_85_macro.urdf.xacro` + `ur_to_robotiq_adapter.urdf.xacro`） | ❌ |
| `ros-humble-ur-moveit-config` | `sim.launch.py:14`（**import 期**即需 `launch_common`）、`:131-133` 读 3 个 yaml、`ur5_gripper.srdf.xacro:7` | ❌ |
| `ros-humble-joint-state-publisher` | UR 标准链路 | ❌ |

`moveit` / `moveit_ros_move_group` / `xacro` / `robot_state_publisher` **本机已装** ✅。

> ⚠️ **注意这里有个容易搞混的点，别把两件事并成一件事**：
> `robotiq-description` **在板端已撤回**（真机夹爪不是 Robotiq，装了无用——见 P4「夹爪不一致」），
> 但**仿真机必须装**（附件仿真的 UR5 末端就是 Robotiq 2F-85）。
> 即：**同一份配置在仿真里正确、在真机上是错的**，这正是 P5 抓取位姿不能直接搬的根源。

**验收**：仿真中 9 个物体全部抓取成功；`/target_3d` 可被独立 echo 验证；抓取节点可单独重启不影响感知。

### P4 — Orin 迁移

| 编号 | 任务 | 说明 |
|---|---|---|
| P4-1 | **板端只读盘点** ✅ **已完成 2026-09-26** | 见下方实测清单。全程只读，未做任何修改、未驱动机械臂 |
| P4-2 | ~~参数化环境脚本~~ **方案已废（第十一轮）** | 原计划修 D9（把 `install_deps.sh` / `environment.yml` 的发行版与 Python 版本参数化）。**改判为直接删除**：本项目用的是**既有的** `robot-vision` 环境，本仓库从不需要"创建环境"；保留一份照做会装错的脚本比没有更糟。真实环境已写入 README |
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
| `controller-manager`、`joint-trajectory-controller` | ~~`ros-humble-robotiq-description`~~ **已撤回**（见下方「夹爪不一致」，真机不是 Robotiq，装了无用） |
| `xacro`、`ur-description`、`ur-moveit-config` | `ros-humble-ros2-control` / `ros2-controllers`（**元包**未装，但**实际组件 `controller-manager` + `joint-trajectory-controller` 已在**） |
| `joint-state-publisher`、`robot-state-publisher`、`cv-bridge` | 任何仿真器（不需要） |
| `colcon-common-extensions`、`pip`、`git` | |

> **P4-4（Jetson 版 PyTorch）实际上已经满足**——板上的 torch 是 `nv24.08` 的 NVIDIA 构建且 CUDA 可用。计划里这条可以降级为「已具备，勿用通用 pip 覆盖」。

**板上其他资产（非本项目，但需知道）：**

- `/home/<user>/yahboom_ws/`（`camera` / `interfaces` / `largemodel` / `text_chat`）、`workspaces/isaac_ros-dev`（**NVIDIA Isaac ROS** 工作区）、`ultralytics/`（源码克隆）、`torchvision/`、`yolo26_data/`、`qr_code_recognition/`。
- **Docker 已装**（`docker0` = 172.17.0.1/16）。
- `/home/<user>/arm-*.sh` 六个脚本：`arm-ready.sh`（性能调优）、`arm-check.sh`（体检）、`arm-isolation-sudo.sh`（资源隔离：控制栈绑 4-5 号核 + 内存保底 1G；桌面压到 0-3 核）、`arm-headless-check.sh`、`arm-cleanup-sudo.sh`、`arm-grow-root.sh`。**这些是本项目在板上做的部署准备，但不在本仓库里**——`arm-isolation-sudo.sh` 的注释引用了「体检清单 D4」，而那份清单我在板上和仓库里都没找到。建议纳入本仓库 `docs/`（见 §5 问题 11）。
- `/dev/video0..5` 六个视频设备在位；**当前无 `/dev/ttyUSB*`、`/dev/ttyACM*`**。

#### 部署分层：谁跑仿真、谁接真机（2026-09-26 用户确认）

**结论：仿真只在这台 x86 工作站上做，Orin 不需要任何仿真能力。** 这条消除了一个原本可能白做的工作块，理由如下（均已核实）：

| # | 事实 | 含义 |
|---|---|---|
| a | **Gazebo 是仿真期依赖，真机路径用不到** | `camera_source:=external` 实测**一个 Gazebo 进程都不起**。P4-1 也早已记录板端 `gz`/`ign`/`gazebo` 全无，并判定「✅ 符合预期」 |
| b | **`ros-humble-ros-gzharmonic` 根本没有 arm64 构建** | 实测 OSRF `jammy` 索引：`gz-harmonic` amd64/arm64 **都有**（1.0.0-1~jammy），但 `ros-humble-ros-gzharmonic` **只有 amd64**（0.244.12-3jammy）。即 arm64 上想跑 ROS↔Gazebo 桥只能源码编 `ros_gz` |
| c | **`gz_ros2_control` 是真机不用的东西** | 它是「让 ros2_control 驱动**仿真世界里**机器人」的 **Gazebo 插件**。真机上由 `ur_robot_driver`（RTDE 网络）取代——**而板端已装好**（P4-1）。故 P3-2 那条源码编译**只服务工作站**，Orin 上无需重编 |
| d | 板端**无 `/dev/ttyUSB*` / `/dev/ttyACM*`** | 与「UR5 走网络 RTDE 而非 USB」相符，进一步支持 `ur_robot_driver` 路线 |

**控制通路的换接口位置**（P5 的关键，一张图）：

```
仿真：  gz_ros2_control (Gazebo 插件)  ─┐
                                       ├─→ controller_manager ─→ joint_trajectory_controller
真机：  ur_robot_driver (RTDE 网络)    ─┘        （两边板端均已在位）
```

即**下游完全一致，只有上游硬件接口换一个**。这是个好消息：MoveIt 规划、控制器、轨迹执行这些逻辑不用改。

> ⚠️ **不要把 amd64 的包往 Orin 上搬，也不要为此刻意改本机架构。**
> apt 按机器装，二进制不过机器。**尤其不要 `dpkg --add-architecture arm64`**——
> 那会让开发机混入 arm64 依赖树而损坏（本机 foreign archs 现为空，保持这样）。
> Orin 上装的自然是 arm64 那一份，两者互不相干。

#### ⚠️ 夹爪不一致：仿真与真机不是同一个夹爪（2026-09-26 用户确认）

**用户确认：真机夹爪不是 Robotiq 2F-85。** 附件的仿真模型是 `ur5_with_gripper.urdf.xacro`（UR5 + **Robotiq 2F-85**），因此：

| 影响面 | 后果 |
|---|---|
| **P4 待装清单** | ~~`ros-humble-robotiq-description`~~ **已撤回**——真机不是 Robotiq，装了无用。应改为 **AG-95 对应的 description/驱动包** |
| **P3-1/P3-4（仿真）** | **不受影响**，仿真仍用附件的 Robotiq 配置即可 |
| **P5-3（真机抓取）** | **仿真里验证过的抓取位姿不能直接搬**。夹爪不同 ⇒ **TCP 偏置不同、可夹持宽度不同**，即使手眼标定（P4-6）做得完美，抓取位姿仍需按真机夹爪重新推导 |
| **URDF/ros2_control** | 真机的末端 URDF 与夹爪控制接口须按 AG-95 重写，不能复用附件 `ros2_control.yaml` 的夹爪段 |

#### ✅ P5-0 已解除：真机夹爪 = 大寰 AG-95（2026-09-27 用户现场读标签确认）

**AG-95（DH Robotics 大寰）实测规格**（来源见下）：

| 项目 | 值 |
|---|---|
| 类型 | **自适应两指**电动夹爪（平行/包络自适应） |
| 总行程 | **95 mm**，编程可调 0~95 mm |
| 抓持力（单侧） | **45~160 N**（可编程） |
| 开/合时间 | 0.7 s |
| 位置重复精度 | ±0.03 mm |
| 重量 / 电压 | 约 1 kg / 24 V DC |
| **通信（标配）** | **Modbus RTU (RS485) + Digital I/O**（2 入 2 出） |
| 通信（选配） | TCP/IP、USB 2.0、CAN 2.0A、PROFINET、EtherCAT（需外接转换模块） |

> 🚨 **这不只是「换个参数」，而是控制接口本质不同——是 P3-6 方案形状的决定因素**：
>
> | | 附件仿真 | 真机 AG-95 |
> |---|---|---|
> | 夹爪 | Robotiq 2F-85 | 大寰 AG-95 |
> | **接口** | **JTC 第 7 关节**（`FollowJointTrajectory`） | **Modbus RTU / RS485 独立设备** |
> | 行程 | 0~0.8 rad | 0~95 mm |
> | 空夹判据 | 关节回读 < 0.75 | Modbus 位置回读 vs 命令值 |
> | 力控 | 无 | 45~160 N 可编程 |
>
> 附件的做法是**把夹爪当作 `joint_trajectory_controller` 的第 7 个关节**（见 `ros2_control.yaml` 与 `JOINTS`/`GRIPPER_IDX`）。**AG-95 不在机械臂轨迹控制器里**，这条在真机上走不通。
>
> ⇒ 规划器**必须**引入**夹爪抽象层**：仿真实现发 JTC 第 7 关节，真机实现走 Modbus。这不是可选的参数化，而是两边接口本质不同。已在 P3-6 中落实。

**来源**：[DH Robotics 官方选型手册](https://www.dh-robotics.com/wp-content/uploads/2023/05/%e4%bc%ba%e6%9c%8d%e7%94%b5%e5%8a%a8%e5%a4%b9%e7%88%aa%e9%80%89%e5%9e%8b%e6%89%8b%e5%86%8c_%e5%a4%a7%e5%af%b0%e6%9c%ba%e5%99%a8%e4%ba%ba_CN2401.pdf)、[imrobotic AG-95 参数页](https://www.imrobotic.com/appequi/detail/3445.html)、[DH 电动夹爪手册](https://en.dh-robotics.com/wp-content/uploads/2025/07/DH_%E7%94%B5%E7%88%AA%E9%80%89%E5%9E%8B%E6%89%8B%E5%86%8C.pdf)

> ⚠️ 不同来源在「抓持力」「开合时间」上有出入（45~160 N vs 10~80 N / 0.7 s vs 0.9 s），**可能因产品版本或渠道不同**。用于选型/力控参数时**以现场实物铭牌与官方最新手册为准**，不要照抄本文表格。

**验收**：板端 `ros2 topic hz /detections` 稳定 ≥ 15 Hz；对已知位置目标 `/target_3d` 误差 < 2 cm；全程未驱动机械臂。

### P5 — 真机抓取（高风险，需逐次授权）

| 编号 | 任务 | 前置条件 |
|---|---|---|
| **P5-0** | ✅ **已解除**(2026-09-27)：真机夹爪 = **大寰 AG-95**，行程 95 mm，**Modbus RTU/RS485**，力控 45~160 N | 见 P4「P5-0 已解除」小节。**仍需现场核实**：实物铭牌版本、RS485 接线与波特率、是否装了 Modbus 驱动包 |
| P5-1 | 核实 UR5 接入方式 | 控制器型号 / 网络 IP / 示教器配置 / 夹爪接口——**不得假设 USB**，UR 常见为网络 RTDE。**旁证**：板端无 `/dev/ttyUSB*`/`/dev/ttyACM*`，与 RTDE 一致 |
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
| `scripts/test_pipeline_e2e.py` | ✅ **新增（第七轮）**：**端到端**验证——真起话题、真走 DDS，把合成 `camera_info`+深度+检测框喂进运行中的 pipeline，断言 `/target_3d` 的**具体数值**。这是唯一能回答「链路是否真的通」的脚本。**第九轮扩展**：加验 `/targets_3d`（候选数、`frame_id`、每条都有 `results`、`id` 非空、`class_id`/`score` **原样透传**、`bbox.center` 与 `results[0].pose` 一致、与 `/target_3d` 坐标逐位吻合）。共 **15 项断言**，实测全绿退出码 0。**为什么必须验分数透传**：规划器用 `hypothesis.score` 做 `min_score` 过滤——分数若被归零，**所有目标都会被静默刷掉** | D33 D1 P2-6 **P3-6** |
| `ros2_ws/src/yolo_bringup/launch/pipeline.launch.py` | ✅ **已改（第九轮）**：删掉**已失效**的 `mode` 参数（D41），改传 `gripper_backend`（默认**随 `camera_source`**：sim→`sim`，真机→`ag95`）；新增 `skip_detector`/`skip_planner`，让「只验感知」与「抓取该用哪个夹爪」两件事解耦 | D41 P3-6 |
| `ros2_ws/src/yolo_bringup/launch/bringup.launch.py` | ✅ **已改（第九轮）**：同上删 `mode`、透传 `gripper_backend` | D41 |
| `ros2_ws/src/perception_3d/package.xml` | 补 `numpy` | — |
| `ros2_ws/src/yolo_detector/yolo_detector/detector_node.py` | ✅ **已改**：`_resolve_model()` 路径解析 + 找不到即报错 + 类别自检；默认值 `block_best.pt`。第七轮：抽出**模块级** `resolve_device()` 做设备退化（+`device_fallback` 参数）；坐标约定（**像素、不归一化**）写进 docstring 并列为「冻结」。**第九轮：修 D40**——检测框坐标显式转 Python `float`，否则 `vision_msgs` 的严格断言会在**第一帧图像**上打死节点 | D6 D24 D7 P2-4 P2-5 **D40** |
| `ros2_ws/src/yolo_bringup/launch/pipeline.launch.py` | ✅ **已改**：默认模型 `block_best.pt`；第七轮加 `camera_source` 枚举（`sim`/`realsense`/`external`），`realsense` 时拉起 `realsense2_camera`（`align_depth.enable:=true`）并**自动把 `depth_topic` 默认值切到对齐话题**；另加 `tf_required` | D6 D2 P2-7 |
| `ros2_ws/src/yolo_bringup/launch/bringup.launch.py` | ✅ **已改**：`world`/`model`/`mode`/`device` 四个原先被硬编码忽略的 launch 参数改用 `LaunchConfiguration`（D31）；第七轮透传 `camera_source`/`target_frame`/`tf_required`，并**据此决定是否启动 Gazebo**（真机不起仿真）与 6 s 延时是否生效 | D31 P2-7 |
| `ros2_ws/src/perception_3d/launch/perception.launch.py` | ✅ **已改（第七轮）**：暴露上述新增参数 | P2-7 |
| `scripts/train_yolo_block.py` | ✅ **已改**：拆除「覆盖根 `yolov8n.pt`／备份为 `yolov8n_backup.pt`」的逻辑（D6 的制造机）；微调起点改为显式 `yolov8n_coco.pt` 并校验存在 | D6 |
| `scripts/test_minimal.py`、`scripts/demo_pipeline.py` | ✅ **已改**：前者显式加载 `block_best.pt`；后者 `MODEL_NAME`→`yolov8n_coco.pt`、`DATASET_DIR`→独立 `block_dataset_demo/` | D6 D30 |
| 权重文件（14 删 / 2 改名） | ✅ **已做**：见 §3 P1 完成表与 §1.4 实测总表 | D6 D13 D25 |
| `ros2_ws/src/yolo_bringup/launch/simulation.launch.py` | **改**：删死变量 `bridge_config`（`:45`）、接上 `gui_arg`（`:28-31`）、把 `rviz_node` 加入返回列表（`:108-114` 未在 `:116-130` 返回）、`TimerAction` 时序参照附件 `TEXTBOOK.md:771` 的 8 段表重做（现为裸 3.0 s）；**并改用 D19 修好的桥接 YAML**；复核 `:89` 的 `JointState@gz.msgs.Model` 映射 | D14 D15 D16 D17 D19 |
| `ros2_ws/src/yolo_bringup/setup.py` | **改**：`config` 的 `data_files` 补 `glob('config/*.rviz')`——否则 D16 修好也读不到 `grasping.rviz` | D16 |
| `ros2_ws/src/yolo_bringup/config/ros_gz_bridge.yaml` | **改**：按附件正确格式重写（顶层 list + `ros_topic_name`/`gz_topic_name`） | D19 |
| `ros2_ws/src/grasp_planner/grasp_planner/planner_node.py` | ✅ **已改（第九轮整文件重写）**：按 P3-6(B) 改为 `/targets_3d` 的纯消费者，**删除参考项目自带的 `_optical_to_world()`/`_publish_camera_tf()` 与硬编码底座偏移**；参数由 `_declare_params()` 统一声明并从 yaml 载入；新增 `_send_arm_trajectory` 的**角度解缠**（相对当前关节角取最近等价角）与 `traj.header.stamp=0`；IK 结果做归一化并对极端 shoulder_pan 做**去种子重试**。✅ 已清理 `tf2_ros`/`tf2_geometry_msgs` 未用声明与 `MoveItErrorCodes` 未用导入 | D3 D12 |
| `ros2_ws/src/grasp_planner/grasp_planner/gripper.py` | ✅ **新增（第九轮）**：`GripperBackend` 接口 + `SimGripper`（Robotiq 2F-85，借 JTC 的 `allow_partial_joints_goal` 单发第 7 关节，**已实测**）+ `Ag95Gripper`（真机，**构造即抛错的桩**——无实物时寄存器映射无法实测，写错会让夹爪乱动）。`package.xml` 已补 `moveit_msgs` 为顶层硬依赖 | P3-6 D3 D12 |
| `ros2_ws/src/grasp_planner/config/grasp_planner.yaml` | ✅ **新增（第九轮）**：全部参数集中一处；**每个几何量都注明「真机必须现场覆盖」**；`table_min_z` 附坐标系换算说明（世界系与 base 系差 0.80，填错会静默丢弃全部候选） | P3-6 |
| `ros2_ws/src/grasp_planner/launch/planner.launch.py` | ✅ **已改（第九轮）**：废弃 `mode:=mock\|moveit`（mock 分支只打日志、从不执行，是**误导性死分支**），改为加载 yaml 并暴露 `params_file`/`gripper_backend` | D3 D12 |
| `ros2_ws/src/arm_grasp_sim/**` | ✅ **新增（第九轮，自附件移植）**：world + UR5+Robotiq 的 URDF/SRDF + `sim.launch.py`。**D34–D39 已修**（构造期改运行期、去 `\|\| true`、去必然失败的 `set_p_gain`、去冗余 `switch_controller`、路径改 `get_package_prefix`、仿真改回 `ExecuteProcess`） | D34 D35 D36 D37 D38 D39 |
| `ros2_ws/src/yolo_bringup/worlds/color_objects_world.sdf` | ✅ **已改（第十三轮）**：选「改名」而非补机械臂（补臂等于再造一个 `grasp_world.sdf`，而 `arm_grasp_sim` 里已经有一套成套的）。文件 + `<world name>` + 12 处引用全部更新 | D4 |
| `yolo_vision/**`、`yolo_coco/**` | **删除**——⚠️ 但 `docs/chapter03_yolo_perception.md` 的 §3.4/§3.5 依赖它们，**先处理教材正文与 5 张配图**（§5 问题 9） | D21 |
| `docs/chapter03_yolo_perception.md` | **改**：§3.3.2/§3.3.3 的 Jazzy 改为 Humble（与附件及 Orin 对齐）；§3.4/§3.5 视问题 9 的选择重写或保留 | D21 D22 |
| `docs/YOLO_VISION_DESIGN.md` | **改**：全文按 `yolo_vision` 包写（`:230-466` 多处命令与目录树），删包后需重指向 `ros2_ws/src/yolo_detector` | D21 |
| `scripts/install_deps.sh`、`environment.yml` | ✅ **已删**（第十一轮）：连同 `create_env.sh`/`setup_all.sh`/`setup_workspace.sh` 一并 `git rm`。真实环境写入 README | D9 |
| `README.md`、`docs/TECHNICAL_SPEC.md` | **改**：区分「设计目标」与「已实现」 | D4 D10 |
| `docs/YOLO_TUTORIAL.md` | ✅ **已改**：死引用（D10）、环境名（D29）、世界布局与参数（D32）、模型名与 `test_minimal.py` 说明、§7.2 的 `-p model:=` 均按真实情况更正 | D10 D29 D32 |
| `README.md` | ✅ **已整体重写**（第十一轮）：原文件通篇为 Jazzy/24.04 时代陈述，逐句更正成本高且易漏，故按七节重写（能跑到什么程度 / 环境 / 构建 / 运行 / 数据流 / 目录 / 自检 / 真机） | D10 D29 |
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
7. ✅ **【已解决】仓库体积**：工作区已从 9.3 GB 降到 405 MB（见 §3 P1）；**历史 8.4 GB 已于第十一轮清除**（用户授权），`.git` 现为 **146 MB**。见上方「历史改写记录」。
8. ✅ **【已解决，第十一轮】模型 A 的来源确认 → 是。** `runs/block_detect/args.yaml` **就是模型 A 的训练配置**。证据链（四条独立印证）：
   - **同目录**：它的 `save_dir` 即 `…/runs/block_detect`，与该 run 产出的 `best.pt` 同处一目录；
   - **同一文件**：该 `best.pt` 与 `block_best.pt` 是**同一个 git blob**（`e917eeaf1…`）、同一个 sha256（`a926d959…`）——即 D18 认定的模型 A；
   - **自述一致**：用 torch 读该 `.pt` 的 `train_args`，其 `model/data/epochs/batch/imgsz` 与 `args.yaml` **逐项相同**（`yolov8n.pt` / `block_dataset/data.yaml` / 100 / 16 / 640），说明该 yaml 就是那次训练的忠实转储；
   - **可与 B 区分**：模型 B 的 `args.yaml` 是 `epochs: 50` / `name: block_grid_demo`，两者不会混。

   **顺带查明的两条事实**（对 Orin 上重训有用）：
   - 该 `.pt` 内部：`nc=1`、`names={0:'block'}`、检测头 `cv3` 输出通道 = 1，训练日期 **2026-07-16**，ultralytics **8.4.51**。即它是 **COCO 预训练微调成单类**，不是从零训练。
   - ⚠️ `args.yaml` 里的训练数据路径 `/home/<user>/workspace/yolo_ros/block_dataset/data.yaml` 与 `project` 同理是**旧机器上的路径，现已不存在**（仓库现位于 `/home/<user>/robotics/repos/yolo_ros_original`）。重训前须改指当前 `block_dataset/`。**该 yaml 是历史记录，不要去"修"它**——它如实反映当时的运行环境。

   > 注：`RESTRUCTURE_PLAN` 中模型 A 有两个哈希写法，**都对，只是算法不同**——D6 的 `e917eeaf1…` 是 **git blob（sha1）**，D18 的 `a926d959…` 是**文件 sha256**。已实测确认，勿据其一改其二。
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
- 🎯 **查明 E1–E5 五条环境事实**（见 P2 小节）：`ultralytics` 与 `rclpy` 分居两个解释器、venv 的 setuptools 84 会让 `colcon build` 失败、~~WSL2 组播发现不稳需 `ROS_LOCALHOST_ONLY=1`~~（**第九轮证伪，见 E3：保持 humble 自带的 0 才对**）、1.23 MB float32 深度把 DDS 压到约 9 MB/s、本机 GPU 实为 **RTX A2000**（**本机不是编教材那台开发机，故硬件与教材不必一致**）。
- 📌 **P2-7 实现时改了设计**：原计划两个布尔量 `use_sim`/`use_realsense` → 实际用**单枚举** `camera_source`（避免「都为真」的无意义状态）。

**第九轮（2026-09-27）新增完成项：**

- ✅ **P3-6(B) 落地，抓取链路端到端全量跑通**：自研规划器接管抓取，**9/9 方块全部搬运成功**，且与 Gazebo 物理真值**逐一对应**（详见 P3-6 实测小节）。
- ✅ **P3-6 全部完成**：统一 3D 接口（`perception_3d` 唯一投影点）、规划器移植（零投影代码）、夹爪抽象层（仿真可用 / 真机为**大声报错的桩**）。
- ✅ **D34–D39 全部修复**：D34（构造期读 URDF/SRDF）、D35（`|| true` 吞错）、D36（`set_p_gain` 必然失败）、D37（冗余 `switch_controller`）、**新增 D38**（`gz_ros2_control` 路径硬编码）、**新增 D39**（`gz_sim` 不是 `ros_gz_sim` 的可执行文件）。
- ✅ **新增 D40（主仓库既有缺陷，与抓取无关）**：`detector_node.py` 的 np.float32 严格断言崩溃——**感知链此前在第一帧图像上就会死**。
- ✅ **本轮还修掉 3 个移植期自引入缺陷**（`table_min_z` 坐标系、`gripper._wrap` 吞回调、`move_duration` 未赋值），共同教训已写进 P3-6 小节。

**第十轮（2026-09-27）新增完成项：**

- ✅ **启动拓扑收口：入口只剩一个**（Q1，用户选定「收口到 bringup，内部复用」）。此前
  detector/perception 的参数在 `arm_grasp_sim/full.launch.py` 与
  `yolo_bringup/pipeline.launch.py` **各写了一份**，而**真机走的正是后者**——
  改一处忘一处就会出现「仿真能跑、真机不跑」，而真机恰恰是最难排查的那个。现在：
  - `pipeline.launch.py` 是**节点的唯一定义处**；新增 `start_delay` 统一延时，把
    `[realsense, detector, perception, planner]` 一起包进 `TimerAction`；
  - `arm_grasp_sim/full.launch.py` **只加仿真专属的东西**（world + 臂 + MoveIt），
    节点一律 include `pipeline.launch.py`（`camera_source:=sim`）；
  - `bringup.launch.py` 成为**唯一入口**，用 `OpaqueFunction` **懒解析** `arm_grasp_sim`。
    这一步是必须的：`get_package_share_directory()` 在 launch **解析期**就求值，
    而 `IfCondition` 只影响**执行期**——所以「板上没编仿真包」会让
    `camera_source:=realsense` 也**整体加载失败**，正是 P4-7「只编子集」的常规做法。
    缺包时改为抛**可操作**的 `RuntimeError`（告诉你怎么补，或改用真机分支）。
  - **删除 `simulation.launch.py`**（`git rm`）：一个**没有机械臂**的世界，抓取链路
    接上后已无意义（规划器起来了却没臂可动）。
- ✅ **P1-5 完成**：`TECHNICAL_SPEC.md` 顶部加「本文是**设计稿**，不是当前实现的描述」横幅，
  内含**逐项环境对照表**（Jazzy→Humble、Noble→jammy、Python 3.12/Conda→3.10 conda、
  `ros-jazzy-ros-gz`→`ros-humble-ros-gzharmonic-sim`…，每行附**核实命令**）与
  「文中提到、但并不存在」清单（`/trigger_grasp` 服务、`moveit_config.yaml`、
  已删的 `simulation.launch.py`、cuRobo、`mode:=mock`）。§4 包结构树与 §6.4 运行命令
  **就地**加了纠正注释（避免只读某一节的人被误导）。README 启动段同步改写。
- ✅ **新入口端到端复验**：改用 `ros2 launch yolo_bringup bringup.launch.py camera_source:=sim`
  （而非直接调 `full.launch.py`）重跑完整闭环，确认收口**未引入回归**。

**第十轮新发现的开放问题（→ 第十一轮已解决）：**

- ⚠️ ~~**删除 `simulation.launch.py` 使「方阵世界」失去了唯一启动器。**~~ ✅ **第十一轮已恢复**。
  全仓库 grep 曾确认：`block_grid_world.sdf` / `grasping_world.sdf` 文件仍在
  `yolo_bringup/worlds/`，但**已无任何 launch 文件加载它们**；而 `GRID_ENV_DESIGN.md` 的
  **训练流水线**（采集训练图 → 自动标注 → 训练 YOLO）第 1 步正依赖它，`bringup.launch.py` 又
  **没有 `world` 参数**。现由新增的 `yolo_bringup/launch/grid_world.launch.py` 补回，
  详见下方第十一轮记录。

---

#### 第十一轮：训练世界恢复 + 环境脚本清理 + 文档去陈旧（2026-09-27）

**本轮范围（用户四问已定）**：环境脚本处置、方阵世界启动器、工具类小修、README 重写。

**1. 新增 `yolo_bringup/launch/grid_world.launch.py`（恢复训练世界，Q2 决定）**

只做三件事，**刻意不含机械臂**：`gz sim -r` 载入 world + `ros_gz_bridge` 桥接相机话题 +
发布 `world→camera` 静态 TF。参数 `world:=`（默认 `block_grid_world.sdf`，可换
`grasping_world.sdf`）、`headless:=`、`publish_camera_tf:=`。

- 相机常量与 `arm_grasp_sim/sim.launch.py` **逐位相同**（位姿 0.5/0/1.5，四元数
  `(0.7071, -0.7071, 0, 0)`），因为两个 world 的 `camera_model` 定义本就一致。
- ⚠️ 帧名 **不同**：本文件用 `camera_model/camera_link/rgb_camera`（本仓库 world 的
  自带相机），`sim.launch.py` 用 `overhead_camera/camera_link/color_camera`（参照项目
  的相机）。**二者不冲突**，可共存于 `/tf_static`——已实测确认（曾有僵尸
  `static_transform_publisher` 在跑 z=1.145 那一支，未干扰本文件的 z=1.5）。
- `gz sim` 用 `ExecuteProcess` 而非 `Node`：它是 gz-tools2 的 CLI，Harmonic 变体的
  lib 目录下**没有** `gz_sim` 可执行文件（D39）。
- 顺带**复活了孤儿文件** `yolo_bringup/config/ros_gz_bridge.yaml`——它本就是为这两个
  world 写的，此前无任何使用者。

**验收证据（实测，非推断）**：

| 项 | 结果 |
|---|---|
| launch 解析 | 5 个 entity，无报错 |
| 话题 | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`、`/camera/depth/points`、`/clock` 全部出现 |
| `camera_info.frame_id` | `camera_model/camera_link/rgb_camera`，与 `CAMERA_FRAME` 常量**逐字相同** |
| TF 链 | `tf2_echo world camera_model/camera_link/rgb_camera` → T=[0.500, 0.000, 1.500]，Q=[0.707, -0.707, 0.000, 0.000]，与常量一致 |
| 换 world | `world:=grasping_world.sdf` 同样可解析 |

**2. 环境脚本全部删除（Q1 决定）**

`git rm`：`scripts/install_deps.sh`、`scripts/create_env.sh`、`scripts/setup_all.sh`、
`scripts/setup_workspace.sh`、`environment.yml`。

理由：它们硬编码 Python 3.12 / `ros-jazzy-*` / Ubuntu 24.04 / Noble（D9），在 Orin
（22.04 + Humble + 3.10）上**不可直接使用**；而本项目真正使用的是**既有的**
`robot-vision` 环境，本仓库从不需要"创建环境"这一步。**保留一份会误导人的安装脚本，
比没有更糟**——照它执行会装出一套与运行环境不符的东西。真实环境改写进 README（Q4）。

同时删除两个**目标已不存在**的脚本：`scripts/launch_sim.sh`、`scripts/launch_pipeline.sh`
（前者指向已删的 `simulation.launch.py`，后者指向的 `pipeline.launch.py` 在仿真里单独起
只会有节点没有臂）。此二项未在四问中逐条列出，但属"条目可删"的既有预授权范围。

> `RESTRUCTURE_PLAN` §3 的 P4-2 与 §4 清单原写"**改**：参数化发行版与 Python 版本"——
> 现改为**删除**，故那两行标注为「方案已废」。

**3. 工具类小修（Q3 决定，逐项已验）**

| 文件 | 修的是什么 |
|---|---|
| `scripts/collect_training_data.py` | 文件末尾有**两个完全相同的** `if __name__ == "__main__": main()`（原 225-230 行）；并把失效的标注提示改指 `label_blocks.py` |
| `scripts/launch_all.sh` | 硬编码 `source /opt/ros/jazzy/setup.bash` → 本机**不存在**该路径，而脚本第 6 行是 `set -eo pipefail`，**当场中止**，于是永远走不到下面那个正确的 bringup 入口。改为 `ROS_DISTRO:-humble` + 存在性检查 |
| `scripts/demo_pipeline.py` | `data.yaml` 的 `path` 由相对路径 `block_dataset` 改为**绝对** `DATASET_DIR`：ultralytics 对相对 `path` 是按 `DATASETS_DIR`（`~/datasets`）解析、**不是**按 yaml 所在目录，于是会静默取错数据，把 D30 的隔离彻底绕过 |
| `scripts/demo_pipeline.py` | 收尾打印的 `simulation.launch.py` → `grid_world.launch.py`（**本轮追加**：该文件已删，照抄必报 file not found） |
| `scripts/generate_grid_world.py` | 同上，收尾打印的死命令一并改掉（**本轮追加**） |

> ~~未采纳：`move_block.py` 的 world/model/z 修正（Q3 中未勾选）。该问题仍在，见 §5。~~
> ✅ **已在第十三轮补做，且发现它远不止「三个常量」** —— 原来的机制（`set_pose`）在这个
> gz build 里**完全是空转的**，脚本一次都没真正挪动过方块。已按 `remove`+`create` 重写并
> 用读回判据验证，见第十三轮第一节（新增 D62/D63）。
>
> 另：本表第 1 行 `demo_pipeline.py` 的绝对 `path:` **不是缺陷，是正确设计**（`DATASET_DIR`
> 运行时算出、注释也解释对了），**先前把它记作「待修」是我的误判，第十三轮已更正**。

**4. README 整体重写（Q4 决定）**

旧的 README 通篇是 Jazzy/24.04/Python 3.12 时代的陈述（L3「ROS2 Jazzy」、L30「Ubuntu
24.04」、L50 `conda activate /home/<user>/...`），目录树漏 `arm_grasp_sim`、列着已删的环境
脚本，"开发路线"把**已完成**的项列成 TODO。新版按七节重写：能跑到什么程度（含**三项未证明**
的显式清单）/ 环境要求 / 构建 / 运行 / 数据流 / 目录结构 / 自检验收 / 真机部署。

**5. 顺手修掉的陈旧引用（全仓库 grep 死文件名逐个确认）**

- `GRID_ENV_DESIGN.md` §6.1、§8.1：改为指向 `grid_world.launch.py`；并纠正 §3 文件树里
  `block_grid_world.sdf` 的**错误描述**——原写「5×5 方阵环境（808 行）」，实测是
  **3×3 / 9 个 block / 425 行**（`block_0..8`，间距 x 0.16 / y 0.12，z 0.785）。
- `TECHNICAL_SPEC.md` §6.2：`conda env create -f environment.yml` → 改为指向真实环境
  叠加方式，并说明为何删。同节前一处**多余的 ``` 围栏**一并修掉（原先会把
  「### 6.2」整段吞进代码块）。

**本轮遗留（未做，见 §5）**：~~`move_block.py` 的 world/model/z。~~
→ ✅ **已完成（第十三轮）**，且原判据低估了它（不是常量问题，是机制空转）。
（原列于此的 Q7 历史改写、Q8 模型 A 确认均已于本轮完成；Q9 板端脚本入 `docs/` 用户已决定纳入，
但**被"没有板子 IP"阻塞**——待用户提供地址后只读拉取。）

**下一步可立即做的（均不阻塞）：**

1. ~~**P2 余项**~~ ✅ 已完成（P2-4/5/7 全部收尾，链路已端到端验证）。
2. ~~**P1-5** 文档「设计目标 vs 已实现」标注~~ ✅ **已完成（第十轮）**：`TECHNICAL_SPEC.md` 顶部已加设计稿横幅 + 环境对照表 + 「提到但并不存在」清单，§4/§6.4 就地纠正。
3. ~~**P5 移植抓取栈**~~ ✅ **已完成（第九轮）**：D27 已降级，朝向不再阻塞；抓取栈已按 P3-6(B) 移植并实测跑通。
4. ~~**P1-3** 权重收敛~~ ✅ 已完成（21→7，文件名与内容全部对齐）。
5. ~~**装 Harmonic（P3-0）**~~ ✅ 已完成，编译与插件名核对均已通过（P3-2）。
6. ✅ **P5-0 已解除**：真机夹爪 = **大寰 AG-95**（2026-09-27）。**仍需现场核实**实物铭牌版本、RS485 接线/波特率、Modbus 驱动包是否就位。抓取位姿/TCP 偏置仍须按 AG-95 重新推导（P5-3）。
7. ~~**决定 `gz_ros2_control` 放哪**~~ ✅ **已定（第十轮，Q2）**：**保持外部叠加工作区**，不 vendored 进仓库；把「先建 `~/grasp_ws`」写成前置条件（README 已写）。

> ⚠️ **端到端的边界（第九轮更新后，已大幅收窄）**：
> 第七轮证明「给定正确输入，链路算得对」；**第九轮更进一步——在真实仿真里跑完了整条闭环**，
> 检测、投影、IK、轨迹、夹爪、搬运全部由本仓库自己的代码完成，并以 Gazebo 物理真值为准。
> 但**仍有三件事没被证明**：
> - 「检测框本身是否准」→ 归 YOLO 模型；仿真里能检出 9/9 且分数 0.68~0.94，但**未在真机数据上量过**；
> - 「外参是否准」→ **归手眼标定，代码层修不了**（D8 只让失败可见，不产生外参）。仿真里相机外参是我们自己写死的，**真机必须实测**；
> - 「真机夹爪能否用」→ AG-95 后端**目前是会报错的桩**，现场核实寄存器映射前**不可用**（这是刻意设计，见 `gripper.py`）。

> 📌 **提交状态（2026-09-27，第十四轮末核实）**：
> `migrate/humble-orin` 已领先 `main` **40 个提交**（`main` 停在 `0ab13ec`，未被动过）。
> 第十二 / 十三 / 十四轮的改动**全部已提交**：第十二轮 = `b82eee0`…`818248e`
> （W1/W1b/D50–D57/W4 与 3 个新脚本），第十三轮 = `9a1db21`…`d3892a6`，
> 第十四轮的 5 个提交见第十四轮第一节。**未推送任何东西**，`origin` 仍不存在（见下）。
>
> ⚠️ **历史改写后，所有提交哈希已变**。旧哈希仅在备份与 `docs/` 的历史叙述中有意义。
> 映射：旧 `535b0666e`（第九轮末）→ 新 `a94c79c` 之前的提交；`main` 旧 `f250af11b` → 新哈希见下。

#### 历史改写记录（Q7，2026-09-27，用户明确授权）

用户授权"可以改"。执行前**先做完整备份**，并用**树哈希**（内容指纹）而非提交哈希来验证结果。

**清除的三个路径**（执行前逐一核实**不在 `migrate/humble-orin` 的 HEAD 中**，故对当前工作零影响）：

| 路径 | 历史体积 | 在 HEAD 中 |
|---|---|---|
| `yolo_coco/` | ~8.86 GB（其中 `train2017.zip` 单个 6.76 GB；共 250,367 个文件） | ❌ 无 |
| `docs/chapter03_yolo_perception.html` | 47.6 MB | ❌ 无 |
| `runs/block_detect/weights/epoch*.pt` | 243 MB（10 个） | ❌ 无（HEAD 只剩 `best.pt`/`last.pt`） |

**结果核验**：

| 项 | 改写前 | 改写后 | 判定 |
|---|---|---|---|
| `.git` 体积 | 8.4 GB | **146 MB** | ✅ −98.3% |
| `HEAD^{tree}` | `201ca7d91e4c…` | `201ca7d91e4c…` | ✅ **未变** |
| `git ls-tree -r HEAD \| sha256` | `603356bd9d4d…` | `603356bd9d4d…` | ✅ **逐字节未变** |
| `main^{tree}` | `1b08535ae13a…` | `d9414da6d5cf…` | ⚠️ **变了**，见下 |
| 提交数 | 27 | 27 | ✅ |
| `main` 是 HEAD 祖先 | 是 | 是 | ✅ 关系保持 |
| 工作区 | 干净 | 干净 | ✅ |

⚠️ **`main` 的树变了，这是预期内的**：`main`（基线 `f250af11b`）**仍含** `yolo_coco/`（250,367 个文件）
与那 10 个 `epoch*.pt`——**8.4 GB 正来自这里**。清除后，检出 `main` 将不再有这两样。
`docs/chapter03_yolo_perception.html` 两个分支都不含，无影响。
即：**活跃分支逐字节不变，只读基线瘦身**。若你需要一个"原样"的基线，它在备份里。

**副作用与备份**：

- `git filter-repo` **移除了 `origin`**（原本指向 `/mnt/c/Users/<winuser>/Downloads/yolo_ros.bundle`）。
  这是它的标准行为（防止误推回旧历史）。**不建议恢复**——从那个 bundle 取回会把旧历史重新灌进来，
  使 146 MB 前功尽弃。若只想当只读参照，用 `git bundle verify` 直接查即可。
- 改写前的完整 `.git` 备份：`/home/<user>/robotics/backups/yolo_ros_git_20260927_130248.git`（8.4 GB，已验证可读，
  含原 27 个提交与 `main`/`migrate/humble-orin` 两分支）。
- 另有更早的原始快照 `/mnt/c/Users/<winuser>/Downloads/yolo_ros.bundle`（8.9 GB，9/26 16:32，早于本轮全部改动）。
- 工具：`git-filter-repo` 2.47.0（本轮 `pip3 install --user` 装的，在 `~/.local/bin`）。

> 📌 **提交状态（2026-09-26 核实，修正此前本文档中「没有任何提交」的错误陈述）**：
> `migrate/humble-orin` 已领先 `main` **8 个提交**（`main` 停在 `f250af11b`，未被动过），
> 第八轮（本文档 P3-0/E5 的修订）尚未提交——工作区仅 `docs/RESTRUCTURE_PLAN.md` 一处改动。
> **按你此前选定的「分组提交」策略，本轮改动等你确认后再落。** 不擅自提交或推送。

**等你的事（第十二轮更新）：**

| # | 事项 | 影响 |
|---|---|---|
| 7 | ~~仓库历史 8.4 GB 是否改写~~ ✅ **已授权并完成（第十一轮）**，见下方「历史改写记录」 | `.git` **8.4 GB → 146 MB**（−98.3%）。`migrate/humble-orin` 内容**逐字节未变**；`main` 的树变了（它仍含 `yolo_coco/` 与 10 个 `epoch*.pt`），这是**预期内**的——那正是 8.4 GB 所在。备份在 `/home/<user>/robotics/backups/yolo_ros_git_20260927_130248.git` |
| 9 | ~~板上六个 `arm-*.sh` 与那份「体检清单」是否纳入 `docs/`~~ ✅ **已完成（第十一轮）** | 用户提供 SSH 后**只读取回**，存入 `docs/board/`：6 个脚本 + **母文档**`机械臂板卡-体检与改造清单.md`（305 行，脚本头部引用的「体检清单 D4」就是它）+ `UR5接入方案.md`。另加 `docs/board/README.md`（来源、sudo 风险、与仓库的关系）与 `docs/JETSON_SSH.md`（连接方式）。全部取回文件已按凭据模式扫描，无密码/私钥/令牌 |
| **12** | ~~`gz_ros2_control` 放哪~~ ✅ **已定（第十轮）：留外部叠加工作区**（`~/grasp_ws`）。不 vendored、不入库 | **只影响工作站，不影响 Orin**（P4「部署分层」已确认：真机由 `ur_robot_driver` 走 RTDE，**不用 `gz_ros2_control`**）。选外部的理由：避免仓库携带一份第三方源码、避免与 apt 版本混淆；代价是**多一步前置**（README 已写明 `GZ_VERSION=harmonic` 的源码编译步骤）。`launch` 已用 `get_package_prefix` 解耦 |
| **13** | ~~「方阵世界」要不要恢复启动器~~ ✅ **已定（第十一轮）：恢复轻量启动器** | 新增 `yolo_bringup/launch/grid_world.launch.py`（`gz sim -r` + ros_gz_bridge + 静态 TF，**不含机械臂**），训练链恢复可用。已实测：话题齐全、`camera_info.frame_id` 与常量逐字相同、TF 链正确。顺带复活了孤儿文件 `config/ros_gz_bridge.yaml` |
| **14** | ✅ **已定（第十二轮）：D61 = 不动它（保持现状）** —— 用户原话「**不动它（保持现状）**」。`scripts/generate_synthetic_data.py` 的 `world_to_pixel` 仍是转置的，但它渲染与标注共用同一函数 ⇒ 图与标签自洽，数据集有效、`block_best.pt` 不必重训 | 按决定执行：**不改投影、不重生成、不重训**。已在两处留下警告：脚本模块 docstring + `world_to_pixel()` docstring（均标明「转置、勿作真机起点、真机走 `auto_label_grid.py`」）与 `README.md` 的「训练数据」一节（两个标注脚本的对照表）。详见下方第十二轮记录第六节 |
| **15** | 🟡 **等你跑（第十四轮新增）：板端 sudo 清单** —— 停 ollama、关桌面、`usermod -aG dialout jetson` | 命令清单见**第十四轮第九节**。**不含**装 MoveIt（板上已有）。⚠️ 两个坑：`docs/board/arm-cleanup-sudo.sh` 的 **A8 段会开 ufw**（默认拒绝入站 ⇒ WSL 侧发现不到板子），且它第 6 行明说「ollama 保留不动」⇒ **别整个脚本照跑**；`pymodbus` **不用 sudo**（pip 已在 `~/.local`），且**别装 apt 的 2.1.0** |

**Orin 侧的 P4 剩余项**（P4-1 已证明板子基础很好）：`torch`/`ultralytics`/RealSense/UR 驱动**都已就绪**。P4-4（Jetson 版 PyTorch）实际已完成，勿用通用 pip 覆盖。

> ✅ **MoveIt 板端已装（第十四轮只读实测，D48 推翻）**：`ros-humble-moveit-*` 2.5.10 齐
> （core / kinematics / ros-move-group / planners-ompl / msgs / ros-planning …）。
> **不要再装 `ros-humble-moveit`。** 板上现在缺的只有 AG-95 侧的 `pymodbus`
> （`pip3 install "pymodbus>=3.6"`；apt 的 `python3-pymodbus` 是 2.1.0，不兼容）。

> ⚠️ **Orin 上不需要、也不要装 `gz_ros2_control` 与任何仿真器**（P4「部署分层」的 c/d 两条已核实）。
> 第九轮新增的 `arm_grasp_sim` 同理——它是**工作站专用**的仿真资产，板端不参与。

---

#### 第十二轮：真机阻塞缺陷修复（W1 / W1b / D50–D57 / W4）—— 仿真已验收（2026-09-27）

**本轮范围**（用户决策 9）：**只做纯代码 + 仿真可验证的部分**。真机 launch（W2）、AG-95 写路径（W3）、
现场步骤（W5）、板上装包与压内存 —— **均不做**，前置条件见计划书「本轮实际执行清单」。
完整计划在 `~/.claude/plans/fluttering-gliding-shore.md`。

**改动规模**（`git diff --stat`，不含 3 个新增未跟踪脚本）：

| | |
|---|---|
| 已跟踪文件 | 11 个，**+1841 / −275** 行 |
| `planner_node.py` | 672 → **1503** 行（函数 26 → 47 个）；`.py`/`.yaml` 均已与 `install/` **逐字节一致**（copy install，已 `diff` 核过 7 个文件） |
| 新增未跟踪脚本 | `scripts/elbow_guard_probe.py`（353 行）、`scripts/test_label_projection.py`（463 行）、`scripts/test_ag95_units.py`（171 行） |
| `scripts/auto_label_grid.py` | 整体重写，510 行 |

##### 一、逐项改动与证据

| # | 改法 | 证据 |
|---|---|---|
| **W1** | 放置从「左摆 90° 松爪」改为**笛卡尔放置区**：新增 `pick_zone`（6 值 3D 区）、`place_slots`（**扁平** `[x,y,...]`，因为 ROS 2 参数**没有嵌套数组**）、`place_z`、`place_release_lift`。终止 = 抓取区空 + 槽位用尽 | 见下「二、三次整跑」 |
| **W1** | 新增 `carry_objects`（默认 `true`）：`false` = **只做动作演示**，不闭合夹爪、不判空，臂照样走完全程 —— 这就是**决策 7 的「不抓取 demo」开关** | `grasp_planner.yaml:147` |
| **W1** | 新增 `max_attempts_without_place`（10）与 `idle_finish_timeout`（20 s）两道**无进展兜底**；`_holding` 状态 + 失败恢复路径（D53） | 见下「三」的护栏 run：10/10 → `收工` |
| **W1** | ⚠️ **故意偏离计划一处**：计划写「`left_drop_joints` 保留为参数默认值」。实际**删掉了**（全仓库只剩 `planner_node.py:66` 的一句历史注释），并同步改掉了启动期长度检查（原检查要求 `home_joints`/`left_drop_joints`/`ik_seed` 都是 6 值，删掉后**必须同步改，否则节点拒绝启动**）。理由：留着就是一个「改了没反应」的第二套路径，正是 D50 的同一类坑 | `grep -rn left_drop_joints ros2_ws/src scripts` |
| **W1b** | 新增**肘高护栏**：`elbow_guard_enabled`、`fk_service: /compute_fk`、`guard_links: [forearm_link, wrist_1_link]`、`table_plane_z`、`guard_margin: 0.02`。不满足时**换另一分支重解一次**，仍不满足就**放弃该目标、不发轨迹** —— 刻意**不用**「去种子重解」（那正是 D54 的错法） | 见下「三」 |
| **W1b** | 桌面作为碰撞体发入规划场景：`table_scene_enabled`、`table_box_size/pose`、`table_retry_period`。**默认 `false`** —— 一个默认值不可能同时对上仿真和真机的桌子，而**错的碰撞体比没有更坏**（会把正确解全拒掉，看起来像 IK 坏了）。`false` 时**明确告警**「avoid_collisions 现在只防自碰撞」 | `sim_run3.log`:245「把桌面作为碰撞体发入规划场景：size=[0.8,0.6,0.03] pose=[0.6,0,-0.405,…]」+「桌面碰撞体已进入规划场景 —— avoid_collisions 现在名副其实」 |
| **W1b/D54** | `arm_grasp_sim/config/kinematics.yaml`：`attempts` 3 → **1**，加 `consistency_limits: [100.0]*6`（关掉 KDL 随机重启），新增 `max_joint_jump: 1.0` 做**对上一路点的关节空间一致性检查**（L∞），取代「`shoulder_pan > 1.40`」那条仿真直觉 | 该 yaml 是 `sim.launch.py:208` 实际使用的那份（计划已核实） |
| **D50** | `pipeline.launch.py` 真的加载 `grasp_planner.yaml` 了，并新增 `planner_params_file` 参数供现场指向自备 yaml | 见下「四」 |
| **D51** | `_open_and_reset()` 改成**先回 HOME、再张爪**；带 `_recovering` 自锁防两个回调互相触发；回 HOME 也失败时**不松爪**并停机 | `planner_node.py:1437`；`sim_run2.log`:11210 实测走到这条路径 |
| **D52** | 轨迹回调改成透传 **`(status, result)`**，要求 `status == STATUS_SUCCEEDED` **且** `result.error_code == 0` | `planner_node.py:121,786`；`test_ag95_units.py` 钉住签名 |
| **D55** | `goal_time_tolerance` 0 → **2.0 s**（0 是**最严**，不是「关闭检查」；原注释理解是反的） | `sim_run3.log` 起跑打印的放置位 z 随 `approach_clearance` 而变 ⇒ 参数真的在驱动几何 |
| **D56** | AG-95 的 **mm ↔ 千分**换算：参数面保持 mm，换算只发生在 `_write_position`/`_read_position` 内部，契约写进 docstring | `scripts/test_ag95_units.py` **全部通过**（含「把寄存器原值当毫米会判不出空夹」的 D56 回归，量级差 ≥10×） |
| **D57** | `block_dataset/data.yaml` 去掉 `path:`；`generate_synthetic_data.py` 的写入端同步改 | 见下「六」，这条同时带出 **D61** |
| **W4** | `auto_label_grid.py` 重写成 `projector_node.py` 的**严格逆运算**（tf + camera_info），新增单应后端（含可选 `H_top` 两平面精确模式）、`--sim-meta` 适配器、`--draw` 覆盖图 | 见下「五」 |

##### 二、仿真验收：三次整跑（`arm_grasp_sim/full.launch.py`，headless）

启动行（**必须**先 source `~/grasp_ws/install`，`gz_ros2_control` 在外部叠加工作区）：

```bash
setsid bash -c 'source /opt/ros/humble/setup.bash; source ~/grasp_ws/install/setup.bash;
  source ros2_ws/install/setup.bash;
  exec ros2 launch arm_grasp_sim full.launch.py headless:=true'
```

| run | `table_plane_z` | 结果 | 日志 |
|---|---|---|---|
| 3 | −0.390 | **已放置 6/6**，随后正常收工 | `/tmp/sim_run3.log` |
| 4 | −0.390 | **已放置 6/6**，同样收工（复现） | `/tmp/sim_run4.log` |
| 护栏 | **−0.100** | **0 次放置、0 条轨迹**，10/10 兜底停机 | `/tmp/sim_guard.log` |

run 3 的终止路径原文（这是 W1 的核心验收点 —— **终止不再依赖「掉到某高度以下」**）：

```
候选 9 个 | 已放置 0/6 | 已抓 0 个 | 连续未放置 0/10
候选 8 个 | 已放置 1/6 ...        ← 候选数单调递减，放下去的方块自动不再是候选
...
候选 3 个 | 已放置 6/6 | 已抓 6 个 | 连续未放置 0/10
============================================================
收工：放置槽位已用尽（6 个）—— 没有地方可放了
  已成功放置 6 个 / 槽位共 6 个；已抓取记账 7 个
  ⚠️ 夹爪里可能还夹着一个物体，且已无处可放。请在 HOME 位置人工取走（或补 place_slots 后重启）。
```

**「落点真的在桌面上」的独立证据**（不靠规划器自述）：run4 收尾时 `projector_node` 仍在发布，
它把相机看到的场景投影回 `base_link`，最后一批 9 个候选是：

```
(0.444,0.118) (0.599,0.120) (0.754,0.119)      ← 3 个没被搬走的，恰好在原方阵 y=0.12 那一行
(0.443,-0.229) (0.599,-0.226→-0.230) (0.754,-0.228/-0.232)   ← 6 个放在 -y 槽位
(0.444,0.224) (0.599,0.226) (0.754,0.228)                    ← 6 个放在 +y 槽位
z 全部 = -0.350 = 台面(-0.390) + 方块半高(0.020)
```

即：**6 个方块确实躺在 `place_slots` 那一排（y=±0.24）的桌面上，z 正好是「桌面 + 半高」**，
而不是旧设计里的「桌外地上、z 从 0.02 起叠」。未搬走的 3 个与 Gazebo 真值
`block_r2_*`（`base_link` y=**0.120**）对比，检出值 0.118/0.119/0.120 —— **吻合到 2 mm**，
顺带给了投影链一个独立的正向验证。

##### 三、肘高护栏的**双向**验证（W1b 的核心，计划里就写明了这个做法）

计划原话：「把 `table_plane_z` 临时设成 **−0.10** … 此时肘朝下的分支应当被护栏拒绝并告警；
恢复 −0.39 后应正常抓取。**这是不用真机就能证明护栏真的在拦的办法。**」

| 方向 | 观测 | 结论 |
|---|---|---|
| `table_plane_z=-0.390`（真值） | run3/run4 各 **0 次**护栏拦下，6/6 放置 | 护栏**不误伤**正确解 ✅ |
| `table_plane_z=-0.100`（把桌子抬到肘部活动区） | **20 次**`肘高护栏拦下`（10 个目标 × 2 个分支）、**10 次**`两个分支都不满足 → 放弃该目标，不发轨迹`；`下降完成`/`已夹住` **计数 0** ⇒ **一条轨迹都没发出去** | 护栏**真的在拦**，且拦下后**不发运动** ✅ |
| 同上 | 拦到第 10 次触发兜底：`收工：连续 10 次尝试都没有产生新的成功放置（上限 10）—— 判定为卡住` | 护栏「全拦」的失效模式是**工位停住等人**，不是撞桌或死循环 ✅ |

原文（`/tmp/sim_guard.log`:5026-5048）：

```
肘高护栏拦下：wrist_1_link 的 z=-0.088 < 台面+余量 -0.080（table_plane_z=-0.100）
肘高护栏不过，改用「另一分支」种子重解一次 ...
肘高护栏拦下：forearm_link 的 z=-0.299 < 台面+余量 -0.080（table_plane_z=-0.100）
两个分支都不满足肘高护栏 —— 放弃该目标，不发轨迹
```

> 这与计划里 D49 的手算吻合：真机几何下「肘朝下」那一支会落到桌面**以下**。
> 仿真里把台面抬到 −0.10 就是在**复现真机的那个几何关系**，护栏随即开火 ——
> 也就是说 D49 在仿真里被**构造出来并当场拦下**了。

**现场工具**：`scripts/elbow_guard_probe.py`（本轮从 `/tmp` 提升入库）。
它读**运行中规划器自己的参数**（量的是规划器真会发的位姿），只调 `/compute_ik` + `/compute_fk`，
**不发任何轨迹**；退出码 0 = 可以动、1 = 不要动。两次实跑结论一致：
正常参数下 `wrist_1_link` 最低 **−0.138**（离仿真台面还有 **+0.232 m** 余量）→ exit 0；
把 `table_plane_z` 抬到 −0.10 后 → exit 1。这就是 W5 第 4b 步「肘高预检」的工具。

##### 四、D50 的一行验证（**本轮重做**，两方向都干净）

计划要求的验证是「改 yaml 里任一值 → 必须跟着变」。为免歧义，用了一个**只在该 yaml 里出现**的探针值
`approach_clearance = 0.077`（src 与 install 两份同步改），然后：

| 启动方式 | 读到的 `接近余量` | 判定 |
|---|---|---|
| `ros2 run grasp_planner planner_node`（裸跑，不加载 yaml） | **0.05**（`_declare_params` 的硬编码默认值） | yaml **没被读** —— 修之前的行为 |
| `ros2 launch yolo_bringup pipeline.launch.py camera_source:=sim skip_detector:=true` | **0.077** | yaml **被读了** ✅ |

进程自己的 argv 就是铁证（从 `pgrep -af planner_node` 抄的）：

```
planner_node --ros-args -r __node:=grasp_planner
  --params-file .../install/grasp_planner/share/grasp_planner/config/grasp_planner.yaml
  --params-file /tmp/launch_params_...
```

派生值也跟着变了（`放置下降位 tool0 z` 不变，但 `接近位 z` 由 −0.155 变到 **−0.133**），
说明这股参数已经接进几何计算，不是只打印出来好看。

> ⚠️ 重做原因：本轮早先存下的 `/tmp/d50_launch.log` **不足以证明它走的是 `pipeline.launch.py`**
> （该 log 里只起了 `planner_node` 一个进程，而 `pipeline.launch.py` 还会起 `projector_node`；
> `planner.launch.py` 在修之前**本来就**加载 yaml，所以那个 log 证不了 D50）。
> 上面这一次是干净可复现的。

##### 五、W4：标注器重写与自检

`scripts/auto_label_grid.py` 现在是 `projector_node.py` 投影链的**严格逆运算**（先前那版
`world_to_pixel()` **连旋转参数都没有**，见计划 W4 一节的说明）。三个要点：

1. **拟合**：`project_pinhole` 丢弃 `Z_c ≤ 0` 的点；`TfProjector.to_camera` 用 `R.T @ (p − t)`；
   相机光心位置就是外参的 `t` 本身（**不是** `R.T @ (−t)` —— 那是最难发现的一类错，见自检第 5 节）。
2. **单应后端有两档**：只给 `H`（零厚度近似，docstring 里量化了误差 ≈ 块高/相机高 × 离光心距离，
   ~11 px @ r=250、~18 px 在画面角）；给 `H_top` 则**取底面∪顶面的凸包**，
   对一个凸立方体这是**精确**轮廓，与 tf 后端一致到 **2.6e-11 px**。
3. `--sim-meta` 把仿真也走同一套 tf 代码（相机位姿当已知外参写进去），
   于是仿真成为共用数学的一次执行，而不是一条旁路。

`scripts/test_label_projection.py`（新，5 节 27 项检查）**全部通过，退出码 0**：

| 节 | 关键结果 |
|---|---|
| 1 往返恒等 | 2000 组随机内参/像素/深度，最大误差 **1.137e-13 px**；含 `fx≠fy` |
| 2 旧写法的转置被钉住 | `u_true = cx+cy−v_old`、`v_true = cx+cy−u_old`，500 点最大偏差 **2.842e-13 px**；`det(R)=+1`；旧写法**位置**偏差最大 **1233 px**（不是几像素），但**尺寸**几乎相同（58.3 vs 58.3 px）—— 这正是「框看着挺对」掩盖位置全错的原因；真投影下 0.8×0.6 桌面 `v ∈ [−62,542]` **出画**，旧渲染 `v ∈ [14,466]` 装得下 |
| 3 退化 | 相机背后的点丢弃、方块在相机背后返回 `None`（不是横跨画面的巨框）、越界框裁剪、空集不除零 |
| 4 单应 vs tf | 500 点一致 **2.109e-11 px**；给 `H_top` 后整立方体一致 **2.589e-11 px**；只给 `H` 时偏差最大 **14.9 px**（与 docstring 声明的边缘误差相符，**故意**断言它偏，而不是假装精确） |
| 5 仿真回归 | 9 个框全在画面内；正中方块宽 **31.91 px = 0.04·fx/顶面距离**（解析吻合）；尺寸随离轴有 **13.6~19.1%** 的自然变化；框中心经正向链解回 `base_link` 误差 **3.331e-16 m** |

端到端（真机形态的 fixture：`/camera/color/camera_info` 内参 + 手眼外参 + ArUco 单应 + 已知方块）：

| 路径 | 输出 |
|---|---|
| 手眼 tf | `0 0.500000 0.006230 0.040280 0.012460` / `0 0.253395 0.105552 0.050574 0.070416` |
| ArUco 单应（只给 `H`） | `0 0.500000 0.006230 0.038490 0.012460` / `0 0.259438 0.115100 0.038490 0.051320` |

**正轴那个方块两条独立路径逐位相同**；离轴方块上的差异（4.4% 宽；高度差更多）
是**已声明的零厚度近似**——单应看不见侧面，tf 看得见。要精确就上 `H_top`（第 4 节已证明一致到 2.6e-11 px）。
仿真回归的 3 帧（`--sim-meta` + `--draw`）落在 `/tmp/lbl_test/`、覆盖图在 `/tmp/lbl_check/`。

##### 六、🐛 D61（新发现，**需要你定**）：合成数据生成器用的是**同一个有 bug 的投影**

**这条不在计划里**，是收尾核 W4 时发现的，而且它比 W4 本身更要紧：

`scripts/generate_synthetic_data.py`（`block_dataset/` 的**真正生成者**，40 个种子 × 5 张 = 200 张、
实际入库 128 张）里有一份**自己的、同样没有旋转的** `world_to_pixel`（`:61-69`），
而且它**同时用于渲染和标注**：

| 用途 | 行 | 调用 |
|---|---|---|
| 渲染桌面多边形 | `:115` | `world_to_pixel(table_corners)` → `cv2.fillPoly` |
| 渲染方块顶面 | `:135` | `world_to_pixel(top_corners)` → `cv2.fillPoly` |
| **生成标签** | `:80`（`compute_bbox`） | `world_to_pixel(8 个角)` → YOLO bbox |

**后果要分两半说清**：

- ✅ **数据集本身是有效的、`block_best.pt` 也不该重训。** 因为图与标签用的是**同一个**函数，
  两者**互相自洽**；而「棕色桌面上一个彩色方块」是位置无关的，
  仿真三次整跑的检测（9/9、分数 0.68~0.94）已经证明它工作。**几何被转置了，但一致性没坏。**
- ⚠️ **但几何是错的**，一旦有人想「用这套 meta 复现/扩充数据集」或者把这份 `world_to_pixel`
  当成真机的起点，就会撞上 W4 那一整节。**修它 = 重新生成全部图像 = 现有 128 张数据集与
  已训练权重全部作废。** 所以我没动它，只把 `data.yaml` 的 `path:`（D57）改了。

**要不要修，取决于你的取向**：① 只留作历史、不动（当前状态）；② 修渲染器并重生成 + 重训
（数据集更正确，但要重训、且要重新验证 9/9）；③ 把渲染器改成调用 `auto_label_grid.py` 的共享数学，
让「图」和「标签」永远不可能再走两套数学（改动最大，但根治）。

> ✅ **你的决定：①「不动它（保持现状）」。** 已按此执行 —— **投影一行没改、数据集没重生成、
> 权重没重训**。作为补偿，把「它是转置的」这件事**显式写进了三个地方**，免得下一个人
> 把它当通用投影用：脚本模块 docstring、`world_to_pixel()` 自己的 docstring、
> 以及 `README.md` 的训练数据一节（两个标注脚本的对照表，指明真机走
> `scripts/auto_label_grid.py` + `scripts/collect_training_data.py`）。

##### 七、仍**未**验证 / 本轮不做（别当成已完成）

- ⚠️ **W4 的「真机数据集」一步都没在真机上做过**。上面全是**仿真 + 构造的 fixture**。
  真机要的东西（手眼外参、`camera_info`、台面网格实测）本轮都不具备，W4 只交付了**工具与数学**。
- ⚠️ **第一个真机 demo 还没有**。本轮做出的是它的**全部前置代码与仿真验收**；
  W2（真机 launch：UR 驱动 + MoveIt + 相机 TF，含 D59 `launch_rviz/launch_servo:=false`、
  `ik_group:=ur_manipulator`、`trajectory_action:=/scaled_joint_trajectory_controller/...`、
  `max_sync_dt` 放宽、示教器 TCP/负载）与 W3、W5 一行未动。
- ⚠️ **AG-95 写路径没实现**，`ag95_dry_run` 骨架只做了只读探测 + 单位契约。
  **闸门仍是**：只读通链路 → 夹爪**拆下臂**放工作台、急停在手边 → 才允许 `ag95_dry_run=false`。
- **`_grasped_positions` 保留**（计划 W1 第 7 点的结论：别删，它防的是「同一位置又冒出检测框」）。
- 板端装包与压内存（`ros-humble-moveit`、停 ollama、`pymodbus`、`dialout` 组、
  `arm-cleanup-sudo.sh` 的 ufw 陷阱）**一项没执行**（需 sudo / 逐项授权）。
- 先前登记的小项：**第十三轮清掉三项** —— ✅ `move_block.py`（机制重写，D62/D63）、
  ✅ `yolo_bringup/worlds/grasping_world.sdf`（D4 改名 `color_objects_world.sdf`）、
  ✅ `demo_pipeline.py` 的绝对 `path:`（**结论是「不是缺陷」，撤销登记**，理由见第十三轮第四节）。
  **仍在**：`arm_grasp_sim/config/kinematics.yaml` 里那句关于 `full.launch.py` 覆盖 `gripper_backend` 的旧注释、
  W2/W3 的两个 `test_*`。
  **AG-95 的 URDF 连杆：第十六轮由用户定「继续不补」**（已知缺口、明确接受；
  重提条件见第十六轮第七节）。
- **未提交、未推送**：`migrate/humble-orin` 仍是本地分支带未提交改动，`main` 未被动过。

##### 八、状态复位（免得下次踩自己的雷）

- `table_plane_z` 已由 −0.10 **复位到 −0.390**（护栏 run 结束后），并核过 `src` 与 `install` 两份。
- `approach_clearance` 的 D50 探针值 0.077 已 **复位到 0.05**，两份文件 `diff` 为空。
- 仿真进程**全部清干净**（`ps` 计数 0）。⚠️ **但清场本身有个坑，下次别信 `pkill -f <launch>`**：
  本轮收尾时发现 **4 个孤儿 `projector_node` + 3 个孤儿 `detector_node`**，全是前几次整跑的残留。
  `kill` 掉 launch 父进程后，子进程被 **WSL 的 `/init`（pid 3832）** 收为孤儿并**继续跑**——
  `detector_node` 尤其要紧：它**在 CPU 上一直跑 YOLO 推理**（日志里 `device='cuda:0' 不可用`），
  白烧算力还会让下一次拉起的检测看起来「莫名其妙地慢」。
  **清场判据要看进程名，不是看 launch 还在不在**：
  ```bash
  ps -eo pid,args --no-headers | grep -E "projector_node|planner_node|detector_node|gz sim|move_group"
  ```
  （本轮最终 `ps` 计数 0。另注：仿真侧 `detector_node` 报 `cuda:0 不可用：torch 未安装`
  是因为**纯 ROS 环境**的 PYTHONPATH 里没有 venv —— 这是**预期**的，仿真跑在 CPU 上足够。）

##### 九、⚠️ 收尾时的一次自伤事故：`--help` 清空数据集 —— **发生了两次，第二次是我自己加的防护坏了**

这一节的目的是让下一个人**不必再踩**，不是自责。两次都是我自己造成的，两次都已完全恢复，
但第二次的教训比第一次值钱得多。

**第一次：`--help` 会干活。** 为核对刚改完 docstring 的脚本还能否正常解析，我给
`scripts/generate_synthetic_data.py` 传了 `--help`。**当时该脚本没有 argparse**，
`main()` 第一件事就是**无条件** `shutil.rmtree` 四个输出目录，于是这个「只想看看用法」的调用
**立刻清空数据集并开始重生成**；管道里的 `| head -5` 在约 5 个种子后把它 SIGPIPE 掉，所以残在一半。
损失：**284 个文件被删、19 个被改、24 个新未跟踪**，且 `block_grid_world.sdf` 与
`block_grid_world_meta.json` 被随机种子覆写（`fixed_mode` 由 True 变 False、形状由 box 变 cylinder）。

恢复路径（**值得记住的是那一步被拒绝**）：auto 模式分类器**拒绝**了我第一条整体清理命令
（`git clean -f block_dataset/` + `git checkout -- block_dataset/`），理由是「不可逆的本地删除，
需用户点名 `block_dataset` 或只回退**已核实的本会话路径**」——**这个拒绝是对的**，整体清理会连
我有意做的 D57 改动一起抹掉。于是改为逐路径：从 `git status` 构造精确的 D/M 清单
（**显式排除我有意改的 `block_dataset/data.yaml`**），逐条确认存在于 HEAD，再
`xargs -a <清单> git checkout --`；随后按精确路径删掉 24 个残留；两份世界文件单独回退。
校验：`git status` 只剩 `M block_dataset/data.yaml`；train 128/128、val 32/32 图与标签**成对**；
权重（`runs/block_detect/weights/best.pt`、`yolo_detector/models/block_best.pt`）未被动过。
根因修复：给它加 argparse + `--force` 门槛。

**第二次（更要紧）：那道防护自己是坏的，于是数据集又被清了一次。**
判据我写成了 `out.glob("*/*/images/*.jpg")` —— 这个模式比真实布局
`out/train/images/*.jpg` **多要求一层同名目录**，于是**恒等于 0**。
**防护一次都没拦过**：其后一次不加参数的普通运行照样清空重建，这次 **339 条改动
（64 D / 131 M / 144 ??）**，同样逐路径救回，数据集回到 128/128 + 32/32、世界文件回到
`seed 824517619 / fixed_mode True / 9 blocks`。

**为什么没早发现 —— 这是本节的核心：我只用 `--help` 验证了「修好了」。**
而 `--help` 在 argparse 内部就 `sys.exit(0)`，**根本走不到那道判据**。
换句话说，我测的是「命令能不能跑」，不是「它该不该拦」。
修完判据后又在实测中抓出第二个毛病：`if __name__ == "__main__": main()` **把返回值丢了**，
于是「拒绝执行」也以 **0** 退出，调用方分不出「干活了」与「我拒绝干活」——已改为 `sys.exit(main())`。

**现在的防护，以及它**实测过的**四种场景**：

| 场景 | 期望 | 实测 |
|---|---|---|
| 数据集非空、不加 `--force` | 拒绝 + **退出码 2** | ✋ 打印「已有 160 张图 / 160 个标签」+ 退出码 2 ✅ |
| `--help` | 用法 + 退出码 0 | ✅ |
| `--dry-run` | 只说明、不删不写 + 退出码 0 | ✅ |
| `--force --out /tmp/gt` | **只动沙盒** | 沙盒被清空重建，真实 `block_dataset` **160 张逐字节未变** ✅ |

顺带修掉同类的三项（都是「参数解析了却没接进代码」，与 **D50** 同一个病）：
四个输出目录改为**从 `--out` 派生**（原先硬用模块常量 `TRAIN_IMG/...`：`--out /tmp/foo`
会被忽略，**报告的是 A、删的却是 B**）；`--seeds` / `--per-seed` 真正接进循环（原先解析了没人用，
`--seeds 5` 照样生成 40 个场景）；删掉 `TRAIN_IMG` 等已成死代码的重复常量。
判据本身改为 `rglob("*.jpg")` + `rglob("*.txt")`：**与深度无关**，且**图或标签任一边残留即算非空**。

> 📌 **写给下一轮的一句话**：**防护必须测「它该拦的那个场景」，不是测「命令能不能跑」。**
> 并且这一轮「参数解析了但没接进代码」共出现三次（D50 的 yaml、`--seeds`、`--out`），
> 看到 `add_argument` 就要顺手确认它**真的被用到了**。

**相关坑：`generate_grid_world.py` 的输出目录 —— ✅ 已修（用户授权「现在就修」）。**
它原先的 `OUTPUT_DIR` 是**硬编码常量**（`:68`，无 `--out`），**无条件**写
`block_grid_world.sdf` 与 `block_grid_world_meta.json`。所以**即使只想往别处生成数据**
（`--out /tmp/...`），草稿也会打到仓库里这两个被跟踪的世界文件上 —— 实测确认过
（`--force --out /tmp/gt` 之后这两个文件仍变成 `M`，已回退）。**这正是前两次事故里
「顺手覆写了跟踪文件」那一半的来源。**

修法：`generate_grid_world.py` 新增 `--out-dir`（默认值**不变**，向后兼容；
`save_metadata()` 同步加 `out_dir=` 形参）；`generate_synthetic_data.py` 把它 40 次生成
全部指向 `tempfile.mkdtemp()` 的临时目录（`atexit` 自动清理，不给生成循环加缩进），
并且**开始检查子进程退出码** —— 原先不看返回值，生成器一旦失败就会去读上一次的残留
meta，于是「用 A 场景的方块位姿标注 B 场景的图」而毫无报错。

实测证据（`scripts/generate_synthetic_data.py --force --out /tmp/gt2 --seeds 2 --per-seed 2`）：

| 判据 | 修之前 | 修之后 |
|---|---|---|
| `git status --porcelain .../worlds/` | 两个文件变 `M` | **空**（一行不动）✅ |
| 临时草稿目录 | 无（直接写仓库） | `/tmp/synth_world_3eybo8vw`，结束**自动删除**，无残留 ✅ |
| 真实 `block_dataset/` | 不受影响 | 不受影响（160 张） ✅ |

另外验过**产出没被这次改动弄坏**（`--seeds 5 --per-seed 2`，8 个标签文件）：
每文件框数 **min=9 max=9 mean=9.0**、空文件 **0**、格式非法行 **0** ——
与 HEAD 里那 128 个参照文件的分布**完全一致**。
前提也验过：生成器失败时**确实**非零退出（未知参数 → 退出码 2），
所以那道 `returncode` 检查不是摆设。

---

#### 第十三轮：登记在案的遗留项清理（`move_block.py` 机制重写 / D4 改名 / 注释修正）（2026-09-27）

##### 一、`move_block.py`：不是「改三个常量」，是**原来一次都没挪动过方块**

登记时这一项写的是「`move_block.py` 的 world/模型/z 修正」，读起来像是补几个常量。动手后
才发现**整个机制是假的**，而且假了**两层**：

| # | 缺陷 | 实测证据 |
|---|---|---|
| **D62** | `/world/<w>/set_pose` 在此 build（gz sim 8.15.0 / Harmonic）**空转**：接受合法 `gz.msgs.Pose`、**无条件回 `data: true`**、然后什么都不做 | 8 行矩阵：`set_pose`（name / id / 两者 / vector / 对 `table` / 显式 `pause:false` 后 / 对**不存在**的模型）全部回 `data: true` 且**位姿一律不变**；同 build 的 `create` / `remove` 对照组则真的生效。`gz service -i -s` 确认请求类型确实是 `gz.msgs.Pose`，非法 `--req` 会被 protobuf 拒、不存在的服务会超时 —— **不是调用写错了** |
| **D63** | gz CLI 的**应答层与读取层都不可信**，三条独立的：<br>(a) `create` **失败也回 `data: true`**；<br>(b) `create` 要求 SDF 包在 `<sdf>` 根元素里，裸 `<model>…</model>` 被 sdformat 拒；<br>(c) `gz model -m X -p` **约 10% 的调用假失败** | 见下方两张表 |

**D63(a)(b) 四形态实测**（判据是 `gz model -m <名> -p` 能否读回，**不是**服务应答）：

| 送的 SDF | 应答 | 模型真的建出来？ |
|---|---|---|
| 裸 `<model name="t">…</model>` | `true` | **否** |
| **`<sdf version="…"><model …></model></sdf>`** | `true` | **是** ✅ |
| 裸 `<model>` 最简（只有一个 `<link/>`） | `true` | 否 |
| 裸 `<model>` 带真实换行 | `false` | 否（proto text 解析就挂了） |

服务端日志才是真相（`UserCommands.cc:927`）：
`Error Code 40: XML does not seem to be an SDFormat or an URDF string` /
`Error Code 30: Unable to read SDF string: <model name="block_0">…`。
版本号不写死，从世界文件头读（现为 `1.11`）。

**D63(c) 实测**（连续 30 次读同一个方块）：

```
第 15/18/25 次失败：rc=0 用时 5.23s
Service call to [/gazebo/worlds] timed out
Command failed when trying to get the world name of the running simulation.
失败 3/30；耗时 min=0.67s max=5.24s 均值=1.19s
```

`gz model` 每次都要先做一次世界发现，那一步会偶发超时，**而退出码仍是 0**。
后果不是「报错」，是**报错报错了对象**：`--random` 有一轮把两块写成「✗ 世界里没有这个模型」，
而它俩好端端地待在世界里没动过。服务端同时刷屏
`NodeShared::RecvSrvRequest() error sending response: Host unreachable` ——
每次调 `gz` 都是一个**短命进程**，它一退出服务端回包就没人收。这解释了假失败的来源。

**这三条我自己各踩了一次，都不是推演出来的：**

1. **我给它加的预检，染上了它本该防住的病。** `remove`+`create` 中间有真空期（create 一失败
   方块就没了），所以我在**高处先建一个探针**、建得出来才动真的。可那版探针是拿
   `data: true` 判的 —— **探针假通过**，正式那块照旧被删掉没建回来，`block_0` 从世界里消失。
   判据现已全部换成 `read_pose()` 读回，`create_model()` / `remove_model()` 都自带重试。
2. **我的量具报了一次假警报。** `create` 出来的模型把**世界位姿记在 link 级**、模型级是缺省的
   `0 0 0`；而原生方块记在模型级。两种写法**世界位姿相同**，我只读模型级就误判「位姿变成 0」。
   量具已改成**位姿走 `gz model -p`、材质/几何才走 SDF 读回**。
3. **我用了一个脆弱的判空方式，让脚本平静地报错。** `list_models()` 第一版写的是
   「输出里没有 `timed out` 就当真空」，于是碰到没见过的失败就**回报一张空表**，
   脚本继而打印「这个世界里没有 `block_*` 模型」并退出 —— 实测撞上过一次，
   当时**世界活着、9 块齐全**。现在只有读到 `No model named <X> was found`（实测原文）
   才算「确定没有」，**其余一律算不确定、交给重试**。教训：**「没有」必须是正面识别出来的，
   不能靠「没看见某个错误字样」推断。**

**另有一个我自己引入、又自己发现的缺陷（已修）**：`--seed` 原先**只复现「抽签序列」，
复现不了「谁拿到哪一签」** —— `blocks` 取自 `list_models()` 给出的世界内部顺序，而方块每被
`remove`+`create` 一次就跑到列表末尾。实测两遍 `--random --seed 1` 抽出的 9 组坐标一模一样、
对应到方块名却全变了。现已按名字排序（`block_0..8` 升序）后再抽签。

**改完后的实测矩阵**（全部通过）：

| 测试 | 结果 |
|---|---|
| `--list` | 12 个模型、真名、真位姿（不再硬编码 `block_r0_c0`） |
| 不存在的模型 `block_99` | `✗ 世界里没有这个模型`、退出码 **1**、3.8 s |
| `--restore block_0`（新增的恢复路径） | 建回 `(0.34, −0.12, 0.785)`，与基准**逐项一致** |
| 单块移动 `block_0 → (0.30, 0.20)` | 位姿由 `gz model -p` **独立读回** = `(0.300, 0.200, 0.785)`；ambient 与移动前**逐字节相同**；几何 `box:0.04³` 不变；**无 `__preflight_*` 残留** |
| `--random --seed 1` 连跑两遍 | **9/9** 两遍、逐字节一致（含「哪个方块去哪」） |
| 材质保全（含 **block_4 是圆柱**） | 9 块 ambient + 几何全部与基准一致；`block_4` 仍是 `cylinder:0.04`、`0.15 0.15 0.85` |
| 落点 | 9 块全在桌面内（`x∈[0.16,0.84]`、`y∈[−0.24,0.24]`、`z=0.785`） |

材质这一列是**真读回来的**，不是推断：通道是
`/world/<w>/generate_world_sdf`（`gz.msgs.SdfGeneratorConfig` → `gz.msgs.StringMsg`），
实测它返回**当前已加载**世界的 SDF，`create` 出来的模型也在里面。
（对照：`/world/<w>/state` 只回 `SerializedStepMap`（位姿，无材质）；`gz model -l` 也不打材质。）

> ⚠️ **D62/D63 是仿真工具链的缺陷，不影响 Orin 真机那条路**（真机不跑 `move_block.py`）。
> 它们的价值在于：**「服务回了 true」在这个环境里从来不是证据。**

##### 二、D4：`grasping_world.sdf` 改名（用户授权「改名（推荐）」）

事实：该文件 `grep -c '<joint'` = **0**，无机械臂，内容只是 `ground_plane / table /
camera_model / red_cube / blue_cube / green_cylinder`；真正的抓取仿真世界是
`arm_grasp_sim/worlds/grasp_world.sdf`（UR5 + 控制器，成套）。它**没有被任何 launch 当默认值**。

已做：`git mv` → **`color_objects_world.sdf`**（保历史），**文件内的 `<world name>` 同步改**
（世界名同时是 gz 的话题/服务前缀，只改文件名会留下半截不一致）；6 个文件共 12 处引用全部更新
（`grid_world.launch.py` / `README.md` / `GRID_ENV_DESIGN.md` / `YOLO_VISION_DESIGN.md` /
`TECHNICAL_SPEC.md` / `ros_gz_bridge.yaml`）。

两处**容易漏掉、差点漏掉**的：

- **install 是拷贝安装**，而 `grid_world.launch.py:68` 是从**包的 share 目录**解析 world 的 ——
  光改 `src/` 的话，`world:=color_objects_world.sdf` 在重建前**根本不存在**。
  已 `colcon build --packages-select yolo_bringup`（纯 ROS 环境、无 `--symlink-install`，
  1.27 s 通过），并**手工删掉 install 里 colcon 不会自动清理的旧 `grasping_world.sdf`**；
  之后 `diff -rq src install` 只剩 `block_grid_world_meta.json`（它**本来就没被安装**，与本次无关）。
- 验证不是「文件在」，是**真起了一次**：headless 启动后 `world/color_objects_world` 前缀下
  **40 个服务**、模型 `ground_plane table camera_model red_cube blue_cube green_cylinder`
  齐 6 个、启动日志无 `[Err]`。

**顺带修掉一处会烂的引用**：`grid_world.launch.py` 原用行号引用
（`grasping_world.sdf:181-184`），而我往文件头加了几行注释就让它失效了（实为 185-188）。
已改为**可 grep 的锚点**并写明「故意不写行号」。

> 📌 **读本台账时注意**：上面历史轮次里出现的 `grasping_world.sdf`（D1/D19/D31 的证据引用、
> 第七/十轮的验证记录）**指的都是同一个文件**，即现在的 `color_objects_world.sdf`；
> 那些引用**故意不回改** —— 它们是当时的事实记录。全仓库现在已无 `grasping_world` 的实际引用，
> 只剩本台账的历史条目和 SDF 头部那句「原先叫什么」。

##### 三、两处注释修正

- **`block_dataset/data.yaml` 第 2 行原本指向它自己**（「理由见 `block_dataset/data.yaml`
  顶部注释」—— 可它就在那个文件顶部，等于没说）。改为指向真正的出处
  （`generate_synthetic_data.py` 里 `# 写 data.yaml` 那段），且**生成器里的字面量与签入文件同步改**，
  随后用脚本比对确认二者**逐字节一致**（否则下次生成又漂开）。
- 上面 D4 提到的行号引用。

##### 四、澄清：`demo_pipeline.py` 的绝对 `path:` **不是** bug（更正我此前的登记）

先前把它记作「D57 同族」，**这是我的误判**：`DATASET_DIR` 是运行时按 `__file__` 算出来的
（`scripts/demo_pipeline.py:259-278`），仓库搬到哪都对；且该处注释**正确地说清**了
「相对 `path:` 会被 ultralytics 按 `DATASETS_DIR` 解析、而不是按 yaml 所在目录」这一层，
所以写绝对路径是有理由的选择。**不改。** 登记里这一项就此撤销。

##### 五、状态复位（免得下次踩自己的雷）

- 所有 gz 进程已收：`block_grid_world` 与 `color_objects_world` 各 0 个服务，
  `pgrep` 无残留，`ps` 无 reparent 到 1 的孤儿。
- ⚠️ **`gz sim` 在本环境不吃 SIGTERM**，实测要 `kill -9`；
  且 `pgrep -f "<模式串>"` 会**匹配到调用它的那条 shell 自己** —— 我又踩了一次，
  循环把自己杀了（退出码 **144**）。查进程用字符类（`[g]z sim`）或直接按 PID。
- 仍然**没有推送**：`migrate/humble-orin` 在本地，`main` 未动。

---

#### 第十四轮：W2 真机 launch（+ `none` 夹爪后端与相机话题收回单层）—— 代码完成，真机未验（2026-09-27）

**范围**：第十二轮把 W2 推迟了（决策 9「本轮只做纯代码 + 仿真可验证的部分」）。本轮按你的
「**2.可以做**」重新纳入。W2 全是纯代码，不需要真机就能写完，也能在**加载 / 参数 / 守卫**
三层验证；真机上的行为仍要等现场（见第八节）。

##### 一、交付：5 个提交

| 提交 | 内容 | 要点 |
|---|---|---|
| `011a1f2` | 夹爪 `none` 后端（`NullGripper`） | **真机 demo 的前提**：`make_gripper()` 在 `planner_node.__init__` 里**无条件**调用，而 `Ag95Gripper` 构造即报错 ⇒ 不加这个后端，**真机上规划器根本起不来**，连「臂走到目标上方」都做不到。只实现 `open()`（异步回 `(STATUS_SUCCEEDED, Result())`，契约见 D52），其余三个方法**抛错而不是静默 no-op** —— 只走位模式下它们不可达（由调用点可达性逐点核对） |
| `45dd3a3` | 相机话题收回单层 + `ik_group`/`trajectory_action`/`carry_objects` 随源切换 | 板端实测：RealSense 发 `/camera/camera/...`；`ik_group` 是 SRDF 派生值（仿真 `ur5_manipulator` / 真机 `ur_manipulator`）；官方默认控制器是 `scaled_joint_trajectory_controller` |
| `b8738bd` | **新增 `real.launch.py`**（真机四件事：驱动 / `move_group` / 相机静态 TF / pipeline） | 见第三节 |
| `b442b17` | `bringup.launch.py` 的真机分支改为 include `real.launch.py` | 见第二节 D64 |
| `9f48ee5` | `scripts/test_real_launch.py`（24 项断言）+ README 更正 MoveIt | 见第五节 |

##### 二、D64（新）：`launch_configurations` 是**整棵树共享的一个 dict** —— 我因此把仿真路径静默改坏过一次

原先我在 `bringup.launch.py`（唯一入口）里把真机专属参数也声明了一遍，理由是我以为
「未声明的参数覆盖会被 launch 拒绝」。**实测把这个前提推翻了**：

```
$ ros2 launch yolo_bringup bringup.launch.py camera_source:=external zzz_bogus:=1
   （无任何报错，zzz_bogus 被静默接受，值照常进 launch_configurations）
```

而 `launch_configurations` **是整棵树共享的**：父文件声明过的参数会顺着 include 链一路传进
`full.launch.py` → `pipeline.launch.py`，下游的 `DeclareLaunchArgument` 见到「已设置」就
**不再应用自己的默认值**。于是「在唯一入口为真机写了 `gripper_backend` 默认 `none`」这一个动作，
把**仿真路径**也变成了 `none`：

```
（当时的仿真日志，无任何错误）
  GraspPlanner 就绪 | 夹爪后端=none … 动作模式=只走位（不夹持）
```

**这是我引入的缺陷，被 `skip_detector` 冒烟当场抓到**（仿真本该是
`夹爪后端=sim (SimGripper(Robotiq 2F-85, …)) / 动作模式=抓取`）。它**没有进过 git** ——
`bringup.launch.py` 在本提交之前的版本从未包含那段声明，所以 git 历史里没有这个回退可查，
理由记在这里：**为什么本文件故意不声明真机参数**。

修法与取舍：真机专属参数（`robot_ip` / `ur_type` / `safety_limits` / `start_*` /
`publish_camera_tf` / `camera_x..camera_qw`）**只在 `real.launch.py` 声明**，入口靠命令行直通。

| 写法 | 万一将来 launch 不再直通未声明参数 | 失效方式 |
|---|---|---|
| 不声明（现状） | `robot_ip` 没传进去 → `real.launch.py` 抛「robot_ip 是必填项」 | **响亮** |
| 声明 | 默认值悄悄改掉仿真路径 | **静默**，且只有跑仿真才会发现 |

取响亮的那个。代价：`--show-args` 不再列出真机细分项（故真机用法写在文件头）。

> ⚠️ **这条机制对现场也有用**：它意味着「在入口 launch 里加一个同名参数」可以**跨文件**改变
> 下游默认值。任何往 `bringup.launch.py` 加参数的动作，都要问一句「这个名字在 include 链下游
> 有没有同名参数」。

##### 三、`real.launch.py` 的三处刻意选择

1. **相机静态 TF 的子帧是 `camera_link`，不是 `camera_color_optical_frame`。**
   第十二轮计划里写着「必须终止在 `camera_color_optical_frame`」—— **那句是错的**，本轮实测更正：
   D435i 驱动自己发 `camera_link → camera_color_frame → camera_color_optical_frame`
   （板端实测，`camera_link` 是根）。给 `camera_link` 补一个父帧整棵树就通了；直接给 optical
   frame 发父帧会让它**有两个父节点**，TF 树直接坏掉。因此**手眼标定的结果必须表达在
   `camera_link` 上**；若标定工具给的是 optical frame 的外参，要先左乘驱动那条固定的
   `camera_link→optical`。
2. **`safety_limits` 在驱动与 `move_group` 两处传同一个值。** 它进的是这台机器人的
   `robot_description`（UR 的 xacro 里安全限制会加东西）。两份 description 不一致时，
   `move_group` 的 FK/IK 与驱动实际发布的 TF 是**两套几何**，解出来的关节角到真机上是偏的，
   **而两边都不报错**。
3. **`initial_joint_controller` 显式传 `scaled_joint_trajectory_controller`** —— 它本来就是
   驱动的默认值（板端实测），但显式传是为了不依赖上游默认值（上游改一次，本文件的意图就悄悄变了）。
   它相对 `joint_trajectory_controller` 的差别是**尊重示教器速度滑条**，对第一次真机动作是明显
   更安全的默认。配套的 `goal_time_tolerance` 已在本轮之前改成 2.0 s（D55）。

D59（`launch_servo` / `launch_rviz` 上游默认 `true`）在本文件里被显式关掉：
`launch_servo` 起的是 `moveit_servo` —— **一个能直接指挥机器人的节点**，真机联调阶段没有任何
东西需要它，一旦被误用就是「绕过规划器直接动臂」。

##### 四、D48 推翻：板上**已经装好** MoveIt，不要再装 `ros-humble-moveit`

第十二轮登记 D48「板上 `moveit` 包缺失」，**本轮实测推翻**（只读）：

```
$ dpkg -l | grep -i moveit
ros-humble-moveit-core                       2.5.10-1jammy.20260910
ros-humble-moveit-kinematics                 2.5.10-1jammy.20260910
ros-humble-moveit-ros-move-group             2.5.10-1jammy.20260910
ros-humble-moveit-planners-ompl              2.5.10-1jammy.20260910
…（另有 common / msgs / ros-planning / occupancy-map-monitor 等）
```

**W5 第 0 步的「装 `ros-humble-moveit`」删掉。** 当年那次探测大概是被 `ros2 pkg list`
的其它故障误导（见第七节：本机 `ros2` CLI 的发现故障，症状正是「查什么都是空」）。

##### 五、验收证据（本机执行，**只到「起得来、参数对、守卫真会拦」这一层**）

| 项 | 怎么验的 | 结果 |
|---|---|---|
| 三个守卫**真会拦** | 直接跑 launch 树 | ① `robot_ip` 空 → 抛「robot_ip 是必填项」并给出两个入口的用法；② `publish_camera_tf:=true` + 全零四元数 → 抛「这不是一个合法的旋转（模长为 0）」；③ 缺 `ur_robot_driver` → 报 `sudo apt install ros-humble-ur-robot-driver` 而不是堆栈 |
| 守卫在 `--show-args` 时**不**误触发 | `--show-args` | 包路径是**执行期**求值，`--show-args` 照样打出全部参数（写错成构造期就会整体失败） |
| 仿真路径**没被改坏** | `camera_source:=sim` 冒烟 | `夹爪后端=sim (SimGripper(Robotiq 2F-85, …))` / `动作模式=抓取` / `IK组=ur5_manipulator` / `桌面进场景=True` |
| `external` 分支**不碰机械臂** | `camera_source:=external skip_planner:=true` | 只起 detector + perception_3d，没有 UR 驱动 / `move_group` |
| 参数**真到得了**规划器 | 读规划器启动横幅（本机 `ros2 param get` 不可用，见第七节） | `IK组=ur5_manipulator`（仿真的正确值） |
| 上面这些**以后不会悄悄退回去** | 新增 `scripts/test_real_launch.py`（24 项，9 组） | 全 PASS。它只用 `/opt/ros/humble`，**不需要仿真器、不需要 `colcon build`**（包路径解析被替换成假的，所以查的是源码而不是 install 里的拷贝）。含 **D64 回归**：唯一入口不得声明任何真机专属参数 |

⚠️ **这些验证的边界必须说清**：它们证明的是**「进程起得来、参数解析对、守卫会拦」**，
**不是**「跨节点的数据流是通的」。第七节说明为什么。

##### 六、另外两处登记更新

- `grasp_planner.yaml` 的 D50 修复（把 yaml 接进真实入口）+ `ik_group` / `trajectory_action` /
  `carry_objects` 的按源切换已在 `45dd3a3` 落地；`real.launch.py` 从 `pipeline.launch.py`
  继承这套参数（**参数只在 `pipeline.launch.py` 一处定义**，两边各写一份必然出现
  「仿真能跑、真机不跑」）。
- `AG95_EMPTY_TOLERANCE_MM` 等换算契约（D56）与 `test_ag95_units.py` 的哑元注入（新增
  `action_msgs` / `control_msgs` 桩）已随 `011a1f2` 修好，22 项全 PASS。

##### 七、本轮顺手发现的两件事（都**不是**本仓库的缺陷，但会影响你怎么用这台工作站）

**(1) 本机跨进程 DDS 发现当前是坏的 —— 这就是「`ros2` CLI 查什么都是空」的真正原因。**

实测（三个进程，同一 shell、同一 `ROS_DOMAIN_ID`、同一 RMW）：

```
$ python3 listen.py &   # 订阅 /probe_topic
$ python3 talk.py       # 发 8 秒
listener: 收到 0 条         ← 而且 get_node_names() 里只有它自己
```

`ros2 param get` / `ros2 topic list` / `ros2 node list` 全是独立进程，走的是同一条发现路径，
所以它们返回空**不是 CLI 坏**，是发现不到。**变通办法（实测有效）**：

```
$ FASTDDS_BUILTIN_TRANSPORTS=SHM python3 listen.py &   # 同一条命令
listener: 收到 16 条
```

分项矩阵（同样的 talker/listener）：

| 传输 | 结果 |
|---|---|
| 默认（SHM + UDPv4） | **0 条** |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | 0 条 |
| `ROS_LOCALHOST_ONLY=1`（+ UDPv4） | 0 条 |
| **`FASTDDS_BUILTIN_TRANSPORTS=SHM`** | **16 条** ✅ |
| 清空 `/dev/shm/fastrtps_*` 后默认 | 0 条（**不是**残留段的问题） |

⇒ 坏的是 UDPv4/多播那条路（WSL2 的 `eth0` 是 <ip>/23，另有 `loopback0`），
SHM 那条是好的。

⚠️ **两个限制**：
  · **`FASTDDS_BUILTIN_TRANSPORTS=SHM` 只适合本机自测**。SHM 不出主机，**与 Orin 板子通信
    必须走 UDP**，所以不要把它写进 `.bashrc` 或任何 launch 文件；
  · 桌面上「`gz sim` 里一切正常」与「`ros2 topic echo` 看不到任何东西」可以**同时成立**，
    别再把后者当成链路坏了。

> 时间线补一句：19:35 那次仿真跑里 `robot_state_publisher` 报了
> `Moved backwards in time, re-publishing joint transforms!` —— 那条警告**要求 RSP 真的
> 收到了 `controller_manager` 发的 `/joint_states`**，即**当时发现是通的**。发现失效发生在
> 之后（我唯一做过的动作是收进程与探测 gz 时钟）。所以这更像是 WSL 侧网络状态变化
> （本机此前记录过同样的 CLI 怪象），**不是本仓库的代码问题**，我也**没有**把它当成
> 「一直如此」。

**(2) 仿真里 move_group 的 `jump back in time` 刷屏 —— 成因**没有**确认，但复现条件收窄了。**

现象（19:43 那份完整仿真日志）：`[tf2_buffer]: Detected jump back in time. Clearing TF buffer.`
**16486 次**（move_group），`Moved backwards in time` **287 次**（RSP）。

**已排除的：`/clock` 本身。** 直接量了两侧：

```
gz 侧 /clock（gz.msgs.Clock 有三个字段）:  system 1790509559 / real 17.78 / sim 15.72
ROS 侧 /clock（rclpy 订阅 6.5 s）      :  16.694 → 23.194  回退 0 次（≈1000 Hz）
```

⇒ ros_gz_bridge 取的是 `gz.msgs.Clock` 的 **`sim`** 字段（**不是** `system`），且转发后
**单调递增**。所以跳变不是「`/clock` 倒退」。

**而且它没能在干净环境里复现。** 同一条命令（`camera_source:=sim skip_detector:=true
headless:=true`，80 s）在**随后两次干净启动**里各跑一遍，两种传输都跑：

| 运行 | `Moved backwards` | `jump back in time` |
|---|---|---|
| plain（默认传输） | **0** | **0** |
| `FASTDDS_BUILTIN_TRANSPORTS=SHM` | **0** | **0** |
| 19:43 那次（对着 16486/287） | 287 | 16486 |

关键差别是 19:43 那次跑之前，机器上**有前几次整跑留下的孤儿进程**（第十三轮已经实测过
这一类：4 个孤儿 `projector_node` + 3 个孤儿 `detector_node`，被 WSL 的 `/init` 收留后
**继续跑**）。若其中有一个旧的 `parameter_bridge` 还活着，它会把**已经死掉的那个 gz 世界**
的 `/clock` 继续发出来 —— 两个 `/clock` 发布者、两个时基，正是「跳回过去」的形状。

⚠️ **这是假设，不是结论**：我没有复现出「有孤儿 → 必刷屏」这一步。本轮**未改任何时钟相关
代码**（`sim.launch.py` 不在改动范围内），`arm_grasp_sim` 整套又是本迁移分支上**新增**的
（没有「改造前」可比），所以我**不**声称它「一直存在」或「本轮引入」。风险是实在的：
`Clearing TF buffer` 会清空 move_group 的 TF 缓存，若正好落在投影/IK 期间，症状是
**偶发的解算失败**。留给下一轮，判据两条：① 跑完整闭环（带检测器）时是否还出现；
② 出现时 `ps` 里有几个 `parameter_bridge` —— **清场要看进程名，不是看 launch 还活着**。

##### 八、未验证 / 未做（别当成已完成）

- ⚠️ **真机上一次都没跑过。** `real.launch.py` 只在本机验到「守卫会拦、参数解析对」。真机上
  十有八九还会撞上本机看不见的东西（驱动握手、控制器激活、相机话题名、手眼外参）。
- ⚠️ ~~**跨节点数据流在本机没验**（第七节 (1)）。带检测器的完整闭环要在
  `FASTDDS_BUILTIN_TRANSPORTS=SHM` 下重跑才算验过。~~
  → **第十五轮已验**：`test_pipeline_e2e.py` 15 项全 PASS（`SHM` 下）。
  另：本轮的 W2 清单里 **`max_sync_dt` 这一条当时漏做了**（它到不了唯一入口），
  也由第十五轮补上。
- **W3（AG-95 写路径）仍是骨架**，闸门不变：只读通链路 → 夹爪**拆下臂**放工作台、急停在手边
  → 才允许 `ag95_dry_run=false`。
- **W5 一行没动**（现场步骤），板端 sudo 清单见第九节。
- 板上**未装任何东西、未改任何配置**（本轮所有板端动作都是只读）。

##### 九、状态与**板端 sudo 清单**（等你跑）

- `migrate/humble-orin`：本轮 5 个提交，**未推送**，`main` 未动。

**板端只读探测（2026-09-27，本轮）**：Python 3.10.12；`pymodbus` 缺，
**apt 里的 `python3-pymodbus` 是 2.1.0（2.x API，与我们要用的 3.x 不兼容）⇒ 用 pip**；
`pip3` 26.0.1 已在 `~/.local`（**装 pymodbus 不需要 sudo**）；`jetson` **不在** `dialout` 组；
`/dev/ttyTHS1` / `ttyTHS2` 存在且是 `root:dialout 660`；ollama `active`+`enabled`；
默认 target 是 `graphical.target`、`gdm.service` 在跑；内存 7.4 GiB 总 / 590 MiB free /
4.2 GiB available；RSS 前几名是 **ChatGPT(×3，合计约 1 GB) / gnome-shell(360 MB) /
gnome-software(317 MB)**，ollama 常驻只占 24 MB（大内存是它**加载模型**时才吃）。

**要你跑的（都是系统状态变更，我不动板子）**：

```bash
# ① 先看清现状（三条只读）
free -h
systemctl is-active ollama gdm
sudo ufw status                      # ← 只读，但要 sudo；见下面 ③ 的警告

# ② 停 ollama（常驻服务，禁用后重启不回来）
sudo systemctl stop ollama
sudo systemctl disable ollama        # 想恢复：sudo systemctl enable --now ollama

# ③ 关桌面 —— 8 GB 板上跑 驱动+MoveIt+YOLO 时最有效的一步（可回退）
sudo systemctl stop gdm              # 立即释放；注意当前图形会话会掉
sudo systemctl set-default multi-user.target
#   回退：sudo systemctl set-default graphical.target && sudo systemctl start gdm
#   只想温和一点：pkill -f ChatGPT; pkill -f gnome-software; pkill -f update-manager

# ④ 串口权限（AG-95 走 /dev/ttyTHS1 时必须；加完组**要重新登录**才生效）
sudo usermod -aG dialout jetson
#   重连 SSH 后确认：groups jetson | grep dialout
```

**不需要 sudo 的**：

```bash
pip3 install "pymodbus>=3.6"         # 别用 apt（那是 2.1.0）
python3 -c "import pymodbus; print(pymodbus.__version__)"
ros2 pkg prefix moveit_ros_move_group   # 应回 /opt/ros/humble —— MoveIt 已在（D48 推翻）
```

⚠️ **两件别做的事**：
  · **别整个跑 `docs/board/arm-cleanup-sudo.sh`**。它的 **A8 段会 `ufw --force enable`**
    （默认拒绝入站、只放行 22/3389/5900），而 ROS 2 DDS 用的是动态 UDP 端口 ⇒
    **WSL 侧发现不到板子**；它第 6 行还明写「保留不动：ollama 服务、docker」⇒
    内存那件事它**不会**帮你做。要跑就只跑它的 A1–A7 段，或跑完补 `sudo ufw disable`。
  · **别装 `ros-humble-moveit`**（已在），**别用 apt 装 `pymodbus`**（版本太老）。

> 🩹 **本轮清场又踩了一次自己的雷**（第十三轮记过同一条）：`pkill -f "[r]os2 launch"`
> 把**我自己的 shell** 也杀了 —— 因为那条命令的命令行文本里就含 `ros2 launch`。
> **查/杀进程一概用 `ps` + `awk` 带字符类**：
> `ps -eo pid,args --no-headers | awk '/[g]z sim|[m]ove_group|[p]lanner_node|[r]os2 launch/'`，
> 别用 `pkill -f`（它会匹配到调用它的那条 shell）。

---

#### 第十五轮：W2 收官 —— `max_sync_dt` 到不了真机，与「参数定义了但没人读」这一类（2026-09-27）

第十四轮把 W2 的代码写完时，**漏了计划里明写的一条**：`max_sync_dt` 要放宽。本轮把它补上，
并顺着它查出**同一类缺陷的另外 7 个参数**和**三个重复定义面**。

##### 一、交付：4 个提交（+ 文档）

| 提交 | 内容 | 要点 |
|---|---|---|
| `5cdd6ac` | 丢弃检测时带上**累计**批数 | 限流日志分辨不出「偶尔丢一批」与「一直在丢」，而后者才是「该不该放宽窗口」的依据 |
| `165b210` | `max_sync_dt` 接到唯一入口（默认随源 0.1 / 0.3）+ 投影节点参数面补全 7 项 | 见第二、三节 |
| `407351e` | 三个「单独启动」launch 文件标注**不在主链路**上 | 见第四节 |
| `94d6a7a` | `test_real_launch.py` 24 → **39 项** | 见第五节 |
| （本轮末） | README「看不到目标先看这一条」 | 把现场排查线索写进用户文档 |

##### 二、D65（新）：投影节点的参数面**只在没人 include 的文件上**定义

`perception_3d/projector_node.py` 有 11 个参数，而 `pipeline.launch.py`（唯一入口）
**只传了 3 个**（`target_frame` / `depth_topic` / `tf_required`）。剩下的 8 个里：

- `max_sync_dt` / `depth_scale` / `depth_grid` / `depth_cluster_tol` /
  `camera_info_topic` / `detections_topic` 只在 `perception_3d/launch/perception.launch.py` 上，
- `depth_median_window` / `depth_buffer_len` **哪儿都没有**。

而那个文件**全仓库没有一处 include 它**（`setup.py` 只是把它装进 `share/`）。于是：

```
$ ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense max_sync_dt:=0.5
   （不报错、不生效。未声明的参数会被 launch **接受**并塞进共享的
     launch_configurations —— 实测 —— 然后没有任何进程读它。）
```

**症状是「看不见目标」**：`projector_node` 超窗时整批丢弃检测（`projector_node.py:352-362`），
**不报错退出**，只在日志里每 5 s 留一条。现场很容易读成「检测器坏了 / 模型不行」。
真机比仿真更容易抖动，所以这一条正是 W2 计划里点名要放宽的。

> 这是 **D50 的同类**：参数定义在一个没人走的入口上。D50 是「yaml 没人加载」，
> D65 是「launch 文件没人 include」。**两者的共同形状**是「定义了、看着像生效、
> 实际没有任何进程读它，且不报错」。所以本轮把它一般化成一条规则写进
> `pipeline.launch.py`：**传给节点的每个参数，都要在本文件声明**，并用自检钉住。

**默认值随源**（沿用 `depth_topic` / `gripper_backend` 的成例）：
仿真 **0.1**（不动 —— 9/9 那套结果就是它跑出来的），`realsense` / `external` **0.3**。

放宽**不会**换来错误坐标：相机与方块静止时，晚 0.3 s 的深度帧与当前帧**逐像素相同**
（深度是场景几何的函数，不随采样时刻变），但窗口仍然拦得住真正的失步（流停了、
时间戳对不上）。而窗口过紧的代价是**没有任何目标**。两害相权，放宽是对的。

##### 三、同一类缺陷的另外 7 个参数（一并补全）

`depth_scale` / `depth_grid` / `depth_cluster_tol` / `depth_median_window` /
`depth_buffer_len` / `camera_info_topic` / `detections_topic` 全部接到唯一入口。
默认值**照抄节点的 `declare_parameter`**（写别的数就是**静默改行为**，
当年 `grasp_planner.yaml` 与节点默认值「碰巧逐条相同」才没暴露），并由自检逐条比对。

`real.launch.py` 里**故意不声明** `max_sync_dt`：真机默认值已由 `pipeline.launch.py`
按源给出，再声明一次就是**第二处定义**（谁生效取决于 include 顺序，且不报错）。
现场要调就命令行直通 —— 这是 D64 那次事故的同一条理由。

##### 四、三个「单独启动」文件：不删，但要把「不在主链路上」写在脸上

`perception_3d` / `yolo_detector` / `grasp_planner` 各有一个单节点 launch，
**全仓库没有一处 include 它们**，但都把所在节点的参数又声明了一遍，文件头还写着
「真机几何必须按现场实测覆盖」这类话 —— 照做是**白忙**，主链路读的是 `yolo_bringup`。

处理方式：**不删**（单独拉一个节点起来调，现场真的会用到），改为在三个文件头写同一条
警示：不在主链路、改这里的默认值无效、要调走唯一入口。`perception.launch.py` 另补了
两个它缺的参数，使它的参数面与主入口**对齐**（自检里有子集断言钉住「不得存在只在那
边有的旋钮」）。

##### 五、验收证据（本机执行）

| 项 | 怎么验的 | 结果 |
|---|---|---|
| **仿真路径的窗口值不变** | `camera_source:=sim` 整跑，读节点启动横幅 | `Perception3D node started. target_frame=base_link, max_sync_dt=0.1s` |
| **非仿真路径拿到放宽值** | `camera_source:=external` 整跑，读同一行 | `… max_sync_dt=0.3s` |
| **累计计数真的会动** | 同一次仿真跑 | `深度与检测时间差 1716 ms 超过 max_sync_dt（100 ms），丢弃本批检测（累计 1 批）…` |
| **跨节点数据流是通的**（第十四轮列的第一条未验项） | `pipeline.launch.py camera_source:=external skip_detector:=true` + `test_pipeline_e2e.py`，**`FASTDDS_BUILTIN_TRANSPORTS=SHM`** | **15 项全 PASS**：喂合成 `camera_info`+深度+检测框，`/target_3d` 的**数值**被逐项核对（`x=0.14440`、`y=0.09025`、`z=0.5`）。这是本仓库第一次在**本机**跑通跨进程数据流 |
| 感知单元没被改坏 | `scripts/test_perception_3d.py` | 全 PASS |
| 冒烟 | `scripts/test_minimal.py` | 全 PASS（⚠️ **必须先叠加 YOLO 环境**：不叠 `PYTHONPATH=~/robotics/venvs/robot-vision/lib/python3.10/site-packages` 时 YOLO 段必 FAIL —— 那是**环境**，不是回归） |
| 上面这些**以后不会悄悄退回去** | `scripts/test_real_launch.py` 24 → 39 项 | 全 PASS。新增：**每个**节点收到的参数 ⊂ 入口声明（3 个节点）、无死旋钮（24 个声明全有引用）、三个单独启动文件不藏私有旋钮、投影节点默认值 = 节点 `declare_parameter`（只放过两处刻意分叉） |

##### 六、顺带量到的一个数（解释第十四轮那条「仿真里每轮触发 24 次」）

仿真跑 100 s：**累计丢弃 24 批**，日志里出现的 dt 是 **1716 ms** 与 **132 ms**。

⇒ 这**不是**启动瞬态，而是**持续的低频失配**（仿真里深度与检测的发布节奏差得多）。
方块静止，所以「投影用的是旧深度帧」不影响结果 —— 9/9 仍然成立。
**因此仿真默认值 0.1 不动**：它是拿到那套结果时的配置，且丢的是**少数**批次。
（真机上的 dt 由驱动帧率决定，D435i 彩色/对齐深度同源、量级是几十毫秒，
0.3 s 是宽裕的；这一点**真机上没有实测过**。）

##### 七、未验证 / 未做

- ⚠️ **真机仍是一次都没跑过**（与第十四轮同）。`max_sync_dt=0.3` 只是**推理上安全**，
  真机实测的 dt 分布还不知道；现场若见累计数持续增长，按 README 那条调大即可。
- 三个「单独启动」文件**仍然重复声明参数**（只是现在写清了「改这里无效」）。
  要根治得让它们 include 主入口 —— 会引入「叶子包 include `yolo_bringup`」的反向依赖，
  收益不抵风险，故登记不做。
- W3（AG-95 写路径）、W5（现场步骤）不变；板端 sudo 清单见第十四轮第九节。

##### 八、状态

- `migrate/humble-orin`：本轮 4 个提交 + 1 个文档提交，**未推送**，`main` 未动
  （本节原写「共领先 44 个提交」，**实测是 45** —— 已在第十六轮第九节更正并记下实测值）。
- 板端本轮**零动作**（全是本机代码 + 本机仿真/自检）。

---

#### 第十六轮：W3 的只读那一半（现场探针）＋ AG-95 URDF 连杆的处置登记（2026-09-27）

本轮只做 W3 里**不会动到任何东西**的那一半：一个只读探针。写路径仍然挂着，闸门不变。

##### 一、交付：2 个提交

| 提交 | 内容 | 要点 |
|---|---|---|
| （本轮） | `scripts/ag95_probe.py` | 现场探针：扫从站、读候选寄存器、人工掰手指定千分位方向。**只读，且由自检扫源码钉住** |
| （本轮） | `scripts/test_ag95_probe.py` | 38 项断言，其中 pty 回环用一个**手写的** Modbus RTU 从站真的读了一遍 |

##### 二、为什么「只读」要写成结构性约束，而不是注意事项

这份脚本是**现场第一个接触真夹爪的东西**，而它能造成的最大伤害不是「读不到」，而是
**偷偷写了一个寄存器**：夹爪会当场动起来，而现场的人以为它只是只读的。所以自检用三段
不同性质的手段钉它，**从强到弱**：

1. **扫源码找写 API**（最强）：`write_register` / `write_registers` / `write_coils` /
   `write_discrete` / `mask_write` 任何一个出现在源码里就是 FAIL；另外钉住**不实例化
   `Ag95Gripper`**（有写能力的那个类）、源码里**没有功能码 6 / 16 的常量**（写单个/写多个）。
   这条不依赖任何人的记性 —— 以后谁加了写路径都会被拦下。
2. **pty 回环**：见下一节。
3. **换算不重复实现**：断言探针里的 `permille_to_mm` **就是** `grasp_planner.gripper`
   里那一个对象（`is` 比较）。两份换算迟早会分叉。

##### 三、pty 回环：用一个**手写**的从站当真值源

`scripts/test_ag95_probe.py` 里有一个 `_RtuSlave`：只用 `os.read`/`os.write` 加**自己算的
CRC-16**（poly 0xA001），跑在 pty 的 master 端；探针用 `/dev/pts/N`。它**不依赖 pymodbus**
—— 被测对象是探针，从站是量具，两边共用一个库就会「一起错」。

顺带覆盖了真机上最容易翻车的三个分支，每一个都是**实际读出来的**，不是构造的：

| 分支 | 从站的行为 | 探针的表现 | 断言 |
|---|---|---|---|
| 地址不是我的 | **静默**（真夹爪也是如此） | 超时 → `无响应` | 扫 1..4 时恰好 3 个 `无响应`，1 号读到 637‰ |
| 非法地址 | 回**异常码 2** | `[--]` 而非崩溃 | `0x0300` 出现在 `[--]` 行 |
| 手册没说是哪类寄存器 | FC3/FC4 都答 | 两类都试 | 从站**收到过** `{3, 4}` |

`--watch`（现场判方向用的那条）也在子进程里跑了一遍：**边跑边改从站寄存器的值**
（637 → 1000），断言两个值都打印出来、被 SIGINT 中断时干净退出、且从站**只收到过读功能码**。

> 附带量到的一件事：`pymodbus` 的 `ModbusSerialServer` **接不了 pty** —— 它在
> `transport.py:204` 把 `exclusive=True` 写死，pyserial 在 pts 上取不到排他锁（`Errno 11`），
> 而且它要求构造时就有 running loop。想用现成 server 做这类回环测试的话，会先撞这两个。
> 手写从站反而更短、也更独立。

##### 四、只读探针拿不到什么（说清楚，免得现场以为读一遍就齐活了）

能确认：**接线、从站号、波特率、CRC、寄存器映射是不是手册那套**。
**拿不到**：有效行程与指尖几何 —— 那要「让手指动到若干位置采样」才有。

**但千分位的方向不用写也能定**：`--watch`，人工把手指掰到全开，看 `0x0202` 是 1000 还是 0。
这正是 `gripper.py` 里 `mm_to_permille(..., open_is_high=?)` 那个**必填**参数的来源 ——
它必填就是因为**只能这样得来**。

##### 五、板端实测（只读，未改动板子）

把探针拷到板子 `/tmp`（仓库外的临时位置，跑完已删）：

| 命令 | 结果 |
|---|---|
| `python3 --version` | **3.10.12** —— 与「按 3.x 写 pymodbus」的前提一致 |
| `python3 -c "import yaml,serial"` | OK（PyYAML 与 pyserial 都在） |
| `python3 -c "import pymodbus"` | **No module named 'pymodbus'**（与先前一致） |
| `ls /dev/ttyTHS*` | `ttyTHS1`、`ttyTHS2` 存在；`/dev/ttyUSB*` **不存在** |
| `lsusb` 找串口芯片 | **无**（适配器还没插） |
| `ag95_probe.py --frames` | exit 0，寄存器表与换算说明正常打印 |
| `ag95_probe.py --list-ports` | exit 3，**「没有发现任何串口…板载 40-pin UART 是 3.3V TTL，接不了 RS-485」** —— 与 `lsusb` 的实测一致 |

板子上 `--frames` 的换算那几行显示「未换算：No module named 'grasp_planner'」——
因为**工作空间还没部署到板子上**。这是**如实**的：换算来自 `gripper.py`，板子上没有它。
为此把那行提示改成了可执行的（`source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash`），
而不是只印一句 `No module named ...` 让人以为脚本坏了。

##### 六、自检逼出来的一个实现缺陷（记一笔：自检不只是回归网）

第一版 `--dump` 在「一个都没应答」时**什么都不说**就退出了。而「一个都没应答」正是现场
最可能遇到的**第一次**结果。是 pty 回环里的负例（故意给错从站号）把它逼出来的。
现在 `--scan-slaves` 与 `--dump` 共用同一段排查顺序：A/B 接线 → 波特率 → 从站号 → 24V 上电
→ `dialout` 权限。**每一步都能独立排除掉一类原因**，这是现场最需要的形状。

##### 七、AG-95 的 URDF 连杆：用户决定**继续不补**（登记在案）

W1b 第 3 项（给 AG-95 补 URDF 连杆，好让碰撞检查覆盖夹爪本体）。本轮请用户定，用户选择
**「继续不补」**。理由（已在计划里核实过，这里复述以便下次不必重查）：

- 第一个真机 demo **不抓取**，夹爪全程保持张开；
- 规划器**不走 `/move_action`**（全仓库搜不到），只用 `/compute_ik` 加一条 JTC 轨迹，
  所以 MoveIt 的规划场景里有没有夹爪**不影响**任何一条本轮的轨迹；
- 碰撞检查目前只覆盖桌子（`table_scene_enabled`，且**默认 `false`**）与肘高护栏
  （`guard_links: [forearm_link, wrist_1_link]`），两者都**不含夹爪本体**；
- `tool0_to_fingertip` 只吸收**指尖距离**这一个标量，**吸收不了夹爪的宽度** ——
  这一点**仍然成立**，所以它是一条**真**缺口，只是本轮不触发。

⇒ **重提条件**：当出现下面任一条时重新评估 ——（a）要上 `move_group` 做真正的规划，
（b）要让碰撞检查覆盖夹爪本体，（c）有 AG-95 的实物尺寸可以量。**在那之前不做**，
登记为「已知缺口、明确接受」。

##### 八、未验证 / 未做

- ⚠️ **`--scan-slaves` / `--dump` / `--watch` 都没有真设备可验**（板上无适配器）。
  已验的是：pty 回环（本机，38 项全 PASS）＋ 板子上 `--frames` / `--list-ports` 能跑。
  **真机第一次插上适配器时，第一个动作仍然是 `--list-ports`。**
- **W3 的写路径仍未实现**（`Ag95Gripper` 仍是构造即报错的桩，没有 `ag95_dry_run`）。
  闸门不变：只读通链路 → 夹爪**拆下臂**放工作台、急停在手边 → 才允许翻 `false` 上臂。
- 板端 sudo 清单（第十四轮第九节）**仍未在板子上跑过**。
  📌 **本轮起，这类「要用户亲自执行」的命令统一收进 [`docs/待执行命令.md`](待执行命令.md)**
  （带 `☐/☑` 状态、在哪跑、为什么、回退办法、执行记录），**不再贴在对话里** ——
  终端里的长清单不好回看、也没地方记「跑过没有」。第十四轮第九节仍是**理由与证据**的出处，
  这份文件是**可执行的形状**。
- AG-95 URDF 连杆：见第七节，**明确接受不做**。

##### 九、状态

- `migrate/humble-orin`：本轮 **3** 个提交（探针+自检 1 个、文档 1 个、计数更正 1 个），
  **未推送**，`main` 未动。领先 `main` 的提交数**不写死**（写下它那个提交本身就会 +1）：
  本轮开始时实测 **45**，本轮末实测 **48** —— `git rev-list --count main..HEAD`，无合并提交。
  📌 顺带更正上一轮的一处计数：第十五轮第八节原写「共领先 44 个提交」，**实测当时是 45**。
  数字差 1 不影响任何结论，但计数是会被后来人直接引用的，所以按实测改。
- 板端本轮**只读**：拷一个脚本到 `/tmp` 跑了两条只读命令，**跑完已删**；未装包、未改配置、未触发任何运动。

---

#### 第十七轮：用户执行了板端清单 → 核对结果 → **pymodbus 3.15.0 把探针打死**（2026-09-28）

第十六轮把板端清单交给用户执行（`docs/待执行命令.md`），用户执行完让我 SSH 核对。
核对本身顺利，**但核对过程中发现探针在板子上根本跑不起来** —— 本轮就是修这个。

##### 一、板端清单的执行结果（只读实测，逐条对）

| 项 | 期望 | 实测 | |
|---|---|---|---|
| 甲-2 ollama | 停且禁用 | `active=inactive`、`enabled=disabled` | ✅ |
| 甲-3 桌面 | 停且默认 target 换掉 | `gdm active=inactive`、`get-default` = `multi-user.target` | ✅ |
| 甲-3 内存效果 | 释放内存 | available **4.2 GiB → 6.1 GiB**（`free` 列 2.3 GiB） | ✅ |
| 甲-4 dialout | 进组、**新会话**可见 | 新 SSH 会话 `id` 里有 `20(dialout)` | ✅ |
| 甲-5 pymodbus | 3.x | **3.15.0** | ⚠️ 见 D66 |
| 甲-6 MoveIt | 已在 | `/opt/ros/humble` | ✅ |
| 串口节点权限 | 够 | `/dev/ttyTHS1` = `root:dialout 660` + 已在 `dialout` ⇒ 可打开 | ✅ |
| 适配器 | —— | **仍然没有**（`lsusb` 无串口芯片） | 阻塞真读数 |
| 甲-1 的 `sudo ufw status` | —— | **没查**（要 sudo，我不跑）；已在 `待执行命令.md` 记明 | 待办 |

内存那一条值得记一个数：**+1.9 GiB**，而且省下来的主要是**桌面**（`gdm`），
不是 ollama（`systemd` 报的 ollama 常驻只有 24 MB，与第十四轮的推断一致）。

##### 二、D66（新）：**pymodbus 3.x 自己有两代 API**，探针写死了其中一代

`scripts/ag95_probe.py` 的 `_read()` 原本写死 `slave=slave`：

```python
rr = client.read_holding_registers(address, count=1, slave=slave)   # ← 3.15.0 上是 TypeError
```

而板子上装到的是 **3.15.0**，它的签名是：

```
read_holding_registers(self, address, *, count=1, device_id=1, no_response_expected=False)
```

| 版本 | 从站号参数 | 依据 |
|---|---|---|
| 2.x（apt 的 `python3-pymodbus`） | `unit=` | 第十四轮记录 |
| 3.6–3.7 | `slave=` | **本机自检最初装的就是 3.6.9** |
| **3.8+（板子 = 3.15.0）** | **`device_id=`** | 2026-09-28 板端实测签名 |

**为什么这是个值得编号的缺陷**（而不是「改个参数名」）——三点：

1. **本机验过不等于板子上能跑。** 我第十六轮在**本机**用 pymodbus 3.6.9 跑通了 40 项
   自检，就以为「探针可用」。而 `settings` 里写的是 `pymodbus>=3.6`，板子上拿到的
   是最新版。**同一个大版本号下有两代不兼容 API**，这条约束我原先根本不知道。
2. **报错文案把原因指反了**，这是更坏的一半。原来的 `except TypeError` 写的是
   「pymodbus 版本不对（…）—— 需要 3.x」。板子上**就是** 3.x，照着这句话查会往
   「版本太老」的方向走，而真因是「**太新**」。⇒ 现在改成把**真实签名**打出来。
3. 它正好落在**这份脚本唯一的用途**上：只读探测是现场第一个动作，
   在板子上跑不起来等于这个交付物是零。

**修法**（不写死任何一代）：按**签名探测**参数名，按类缓存，并把结果印出来。

```python
_SLAVE_KWARGS = ("device_id", "slave", "unit")          # 先新后旧

@functools.lru_cache(maxsize=None)
def _slave_kwarg(client_cls) -> Optional[str]:
    params = inspect.signature(client_cls.read_holding_registers).parameters
    return next((n for n in _SLAVE_KWARGS if n in params), None)
```

`--frames` 现在会直接印出结论：`pymodbus: 3.15.0（从站号参数名：device_id）` ——
**现场不用等失败就能看见**它在用哪个名字。

##### 三、验收：三代 API 里能装的都装了一遍，**并在板子上真跑**

| 环境 | pymodbus | 结果 |
|---|---|---|
| 本机 | **3.6.9**（`slave=`） | `test_ag95_probe.py` **40 项全 PASS** |
| 本机 | **3.15.0**（`device_id=`） | **40 项全 PASS**（含 pty 回环：真的读到了 637‰） |
| **板子（aarch64 / Python 3.10.12）** | **3.15.0** | **40 项全 PASS，0 SKIP** —— 含第 2/3 段（`gripper.py` 能导入，说明 `control_msgs` 等也在）与 pty 回环 |
| 板子 | 无 pymodbus 时 | 20 PASS / 9 SKIP / 0 FAIL（SKIP 打印原因，**不算 PASS**） |

自检新增 2 项**结构性**断言，钉住这次的修法：两处读调用**必须**用探测出来的名字
（`**{kw: slave}`），且探测结果必须落在已知的三个名字里。

> 顺带记一句设计上被验证到的东西：本机 `/tmp/pmb` 被 `systemd-tmpfiles` 清掉之后，
> 自检**没有**假装通过，而是如实退成 9 项 SKIP 并打印「没有 pymodbus」。
> 「缺依赖 ⇒ SKIP 并说原因」这条在真实场景里生效了。

##### 四、板端本轮动作（只读，已清场）

拷了 5 个文件到板子 `/tmp/ag95check/`（探针、自检、`gripper.py`、`__init__.py`、
`grasp_planner.yaml` —— 保持相对目录结构，好让自检的路径推导成立），跑完
**已 `rm -rf`**。未装包、未改配置、未触发任何运动、未碰板子家目录。

##### 五、未验证 / 未做

- ⚠️ **真设备仍然没有**（板上无 USB-RS485 适配器）⇒ `--scan-slaves` / `--dump` /
  `--watch` 对**真夹爪**从未跑过。已验的是 pty 回环（本机两代 + 板子上）。
- **W3 写路径仍未实现**（`Ag95Gripper` 仍是构造即报错的桩），闸门不变。
- 甲-1 的 `sudo ufw status` 未查。
- 板端部署（把仓库弄上板子、编子集）**还没开始**。

##### 六、状态

- `migrate/humble-orin`：本轮 1 个提交（修 D66，代码与文档同一个提交），
  **未推送**，`main` 未动。

---

#### 第十八轮：板端「只编子集」的那条容错，从「文档说了」变成「测过了」（2026-09-28）

##### 一、这一轮要回答什么

第十七轮末把**板端部署**列为「还没开始」。而板端部署的第一步就是——
**Orin 上没有 `arm_grasp_sim`，也没有任何仿真资产**，按子集编译是既定做法
（`docs/待执行命令.md` 的「别做」一节与 README 的「板端只编子集即可」都这么写）。

那么整条部署路径就压在**一条从没被执行过的代码**上：`bringup.launch.py` 在
`arm_grasp_sim` 缺席时，真机分支**必须照样加载**。它此前只出现在 README 与台账的
**陈述句**里（「对缺 `arm_grasp_sim` 做了容错并给出可操作报错」）——
按 [[verify-before-asserting]]，陈述句不算证据。这是**第一个真机命令**会走到的代码，
所以先把它钉住。

##### 二、claim 的机制（复核代码，`bringup.launch.py:86-130`）

```python
def _sim_backend(context):                     # 执行期才跑
    if context.launch_configurations.get('camera_source', 'sim') != 'sim':
        return []                              # ← 非仿真：连查包都不查
    try:
        full_launch = os.path.join(get_package_share_directory('arm_grasp_sim'), ...)
    except PackageNotFoundError:
        raise RuntimeError(...)                # ← 报错里带补编命令 + 绕开办法
```

它被注册成 `OpaqueFunction(function=_sim_backend)`（`:270`），**不是**
`IncludeLaunchDescription(..., condition=sim_only)`。原因写在文件里：
launch 在**解析期**求值 `get_package_share_directory`，而 `IfCondition` 只影响**执行**。
注册处也因此是**两个** `OpaqueFunction`（`:259` 的真机后端同理，理由同一个）。

##### 三、反例实验：这条理由是真的，还是「听起来对」？

我不想只引用注释，就**照注释描述的反面**写了一个最小 launch 文件
（`condition` 恒为假 —— 等价于 `camera_source:=realsense`）并构造它：

```
$ python3 -c "… generate_launch_description() …"
解析期就炸了： PackageNotFoundError: "package 'arm_grasp_sim' not found, searching: ['/opt/ros/humble']"
```

**条件为假也照样炸。** 于是「缺仿真包会让真机分支一起加载失败」不是一个假想，
而且它给出的报错会是「找不到 arm_grasp_sim」——**把方向指错**，让人去查仿真资产，
而真机根本就不需要那个包。`OpaqueFunction` 这一层因此是**必需的**，不是风格偏好。

##### 四、新增自检：`test_real_launch.py` 的 W2-15（8 项）

用法是「把查包函数换成一个**只在被问 `arm_grasp_sim` 时才炸**的探测器」：

| 断言 | 证明什么 |
|---|---|
| `camera_source:=realsense` → 一个包都不查、返回 `[]` | 真机分支**根本不碰**仿真包 |
| `camera_source:=external` → 同上 | external 同理（它是「别碰机械臂」那条路） |
| 缺包时 `generate_launch_description()` 仍能构造，且**恰好 2 个 `OpaqueFunction`** | 解析期不查仿真包；后端是执行期构造，不是带 condition 的 include |
| `camera_source:=sim` 且缺包 → 抛 `RuntimeError` | 不是**静默空启动**（那种失败最难查） |
| 报错里含 `colcon build --packages-select arm_grasp_sim` | 给出补编命令 |
| 报错里含 `camera_source:=realsense` | 给出绕开办法（真机部署就是这条路） |
| 报错里含 `Orin` | 点明板上不需要仿真资产，免得用户以为部署缺东西 |
| **包在时 `sim` 仍然 include 仿真后端** | 上面 7 条不是在一个彻底坏掉的函数上全 PASS |

最后一条是**防空断言**的：只验坏路径坏得对，不验好路径仍然好，等于什么都没验。

##### 五、这一轮新写的测试**自己**犯了一个错，被抓出来了

第一版探测器写成「**任何**查包都炸」，结果第 3 条断言 FAIL：

```
AssertionError: 不该在这里查包：yolo_bringup
```

这不是代码的缺陷，是**测试的缺陷** —— 而且顺带查实了一件值得记的事：
`generate_launch_description()` **确实**会在解析期查 `yolo_bringup` **自己**，
因为 D50 的修复把 `grasp_planner.yaml` 的路径解在
`get_package_share_directory('yolo_bringup')` 上。那个包就是正在加载的这个包，
永远存在 —— **查它是对的**。

⇒ 记一条判据：「解析期查包」本身不是缺陷，**要问的是查谁**：
查「正在加载的包」没问题，查「可能没编的别的包」就必须挪到执行期（`OpaqueFunction`）。
探测器现在只对 `arm_grasp_sim` 炸，其余包放行（`bringup.launch.py:132` 附近有同样的说明）。

##### 六、顺带查出来的：README 的自检计数是**陈旧的**（并已更正）

核对计数时把五个脚本各跑了一遍，发现 README 的自检块有三处对不上实测
（陈旧不影响运行，但会让人以为「我这边少跑了几项」）：

| 脚本 | README 原写 | 实测（2026-09-28） |
|---|---|---|
| `test_perception_3d.py` | 24 项 | **23 项** |
| `test_real_launch.py` | 39 项 | **44 项**（本轮之前）→ **52 项**（本轮之后） |
| `test_ag95_probe.py` | 38 项；无 pymodbus 时 9 项 SKIP | **40 项**；无 pymodbus 时 **26 PASS / 6 SKIP** |

`test_detector_device.py` 的 19 项是对的。`test_pipeline_e2e.py` 的 15 项取自
第十五轮实测（该文件自那以后未改动），保留。

**还改了一处会真让人白忙的**：自检块的命令只写了 `source /opt/ros/humble/setup.bash`，
而 `test_minimal.py` 要 `import ultralytics`（在 conda 环境里）。照 README 原样跑，
它会打印「ultralytics 未安装」并**FAIL** —— 看起来像项目坏了，实际只是解释器不对。
已补上 `export PYTHONPATH=…/robot-vision/lib/python3.10/site-packages` 那一行，
并注明其余四个脚本不需要它。

> 实测：叠加该 PYTHONPATH 后，五个脚本一次跑完 **3 / 23 / 19 / 52 / 26(6 SKIP)**，
> 0 FAIL。`test_ag95_probe.py` 在叠加 `/tmp/pmb315`（板子的 3.15.0）后 **40 PASS / 0 SKIP**。

##### 七、未验证 / 未做（不变）

- **板端部署本身仍未开始**：仓库还没弄上板子。本轮验的是**它的前置条件**之一
  （缺仿真包时不炸），不是部署本身。
- 板上没有真串口 ⇒ AG-95 探针没对**真夹爪**跑过（只验过 pty 回环）。
- W3 写路径仍是桩；甲-1 的 `sudo ufw status` 仍未查。
- 真机 launch（W2）与现场步骤（W5）按决策 9 推迟，不变。

##### 八、状态

- 本轮 2 个提交：一条更正第十七轮状态条的计数；一条是 W2-15 自检 + README 计数更正。
- `migrate/humble-orin` 领先 `main` 的提交数**不写死**（写下它那个提交本身就会 +1）：
  本轮开始时实测 **50**。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。

---

#### 第十九轮：D67 —— 「同名不同物」在**部署路径**上复发（2026-09-28）

##### 一、D67：`block_best.pt` 这个名字下有两个不同的模型

承接第十八轮：既然板端部署的既定做法是**只编子集、从仓库外启动**，那就要问
「从仓库外启动时，检测节点加载的是哪个模型」。答案不是一个模型，是**两个**：

| 启动目录 | 解析到 | 来源 run | epoch | mAP50-95 | md5 |
|---|---|---|---|---|---|
| 仓库根 | `<repo>/block_best.pt`（**模型 A**，被跟踪） | `runs/block_detect` | 100 | **0.9655** | `4916bd5e` |
| `~` / `/tmp` / 任何仓库外目录 | 包内 `models/block_best.pt`（**模型 B**，未被跟踪） | `runs/detect/block_grid_demo` | 50 | 0.9064 | `87fb7035` |

实测（`_resolve_model` 是纯函数，用占位 self 直接调）：

```
CWD=/home/<user>/robotics/repos/yolo_ros_original -> <repo>/block_best.pt
CWD=/tmp                                        -> <install>/share/yolo_detector/models/block_best.pt
CWD=/home/<user>                                  -> <install>/share/yolo_detector/models/block_best.pt
```

**这不是新知识，是旧缺陷的复发。** 第十六轮之前的台账第 8 问已经查清「模型 A / 模型 B」
两个 run 的存在，并写下「**可与 B 区分**：模型 B 的 `args.yaml` 是 `epochs: 50` /
`name: block_grid_demo`，两者不会混」。那句话对**训练产物**成立，对**运行期解析**
不成立 —— 两个 `best.pt` 被摆成了**同一个文件名**，而 `_resolve_model` 的第 4 步
（包内 `models/`）恰好会去捡包内那份。

⇒ **D67：同一个文件名下存在两个不同源的模型，加载到哪一个取决于启动时的 CWD；
而节点自带的「用错模型」自检看不见它。**（两个模型 `names` 都是 `['block']`，
实测 —— 那条自检挡的是 COCO 权重，挡不住「另一个方块模型」。）

**这是 D6 的同一个形状。** `scripts/train_yolo_block.py:158-161` 的注释里写着
D6「同名不同物」当年是怎么被制造出来的（旧版把方块模型覆盖到仓库根的
`yolov8n.pt`，真 COCO 反被赶到 `*_backup.pt`）。这次的制造者不同，形状一样：
**一个名字，两个物**。

##### 二、包内那份的来源与性质

- **未被跟踪**（`.gitignore:28` 忽略 `ros2_ws/src/yolo_detector/models/*.pt`），
  且与**被跟踪**的 `runs/detect/block_grid_demo/weights/best.pt` **逐字节相同**
  （sha256 `b9baf6f1…`）⇒ 删掉不丢东西，一条 `cp` 就能复现。
- **不是训练脚本放的**：`train_yolo_block.py:149-152` 只往**仓库根**写
  `block_best.pt`。所以它是旧工作流（`demo_pipeline.py` 那个
  `block_grid_demo` run 时代）手工留下的遗留物。
- `setup.py:20` 的 `glob('models/*.pt')` 是**有意的**逃生口（有人刻意想让模型随包
  安装时可以放），所以**保留**那段代码，只把文件删掉。

##### 三、处置（用户选定「删掉包内那份」）

已删除两处：源目录 `ros2_ws/src/yolo_detector/models/block_best.pt` 与 install 里
那份 `install/yolo_detector/share/yolo_detector/models/block_best.pt`（后者是构建
产物副本，不删则本地仍会解析到模型 B）。删除后实测：

```
CWD=<repo>  -> <repo>/block_best.pt        ← 模型 A（与仿真 9/9 那套一致）
CWD=/tmp    -> None                        ← 明确报错，不再静默换模型
CWD=~       -> None
```

**这是「宁可启动失败，也不要静默地用错模型」** —— 那句话本来就写在
`_resolve_model` 的文档字符串里，本轮只是让它对**方块模型之间**也成立。

⚠️ 因为两处都未被跟踪，**这次删除不会出现在 git 历史里**。故把哈希与两个 run 的
对应关系写在这里（上表），并对「同名必须同源」加了一条自检断言（见下）。

##### 四、新增自检 `scripts/test_model_resolution.py`（12 项）

四类：

1. **解析顺序**：绝对路径（存在→原样、不存在→`None`，不退回按名字猜）；
   `$YOLO_MODEL_DIR` 优先于 CWD 及父目录；没有 env 时逐级向上找（含祖父目录）；
   哪一级都没有 → `None`。用**唯一临时名**做探针，避免被 `/tmp`、`/` 里的同名文件干扰。
2. **同名必须同源**（D67 的钉子）：包内 `models/*.pt` 若有文件，必须与仓库根同名
   文件**逐字节相同**；不同源则 FAIL，并把两边归到**已知的来源 run**上
   （报错直接说「这是 `runs/block_detect` 100 epoch 那份 vs `runs/detect/block_grid_demo`
   50 epoch 那份」—— 「哪份是哪份」正是当年查不出来的东西）。
3. **不许联网兜底**：源码里必须有 `raise FileNotFoundError`、必须**先解析后加载**
   （`YOLO(str(model_path))`）、不得有裸的 `YOLO(raw_model)`、报错里要给
   `YOLO_MODEL_DIR` 与绝对路径两条出路。
4. 类别自检仍在（它挡不住同名不同物，但能挡住 COCO 权重）。

##### 五、这条自检**第一版是空的**，被变异测试抓到

第一版把 `MODELS_DIR` 写成 `src/yolo_detector/**yolo_detector**/models`（多了一层
内层 python 包名 —— 真实位置是**包根**下的 `models/`，`setup.py` 的 glob 也是相对
包根）。于是它去一个不存在的目录 glob，**恒为空、恒 PASS**。

抓法不是读代码，是**变异测试**：把模型 B 放回 `models/` 再跑一遍，
「同名必须同源」那条**没有响** —— 空断言当场暴露。修好之后两个方向都验了：

| 变异 | 期望 | 实测 |
|---|---|---|
| 放入**模型 B**（不同源） | 必须 FAIL，且指出两边各是哪个 run | ✅ 2 项 FAIL，报错给出两个 run 名 |
| 放入**模型 A**（同源） | 必须 PASS（刻意同源复制是允许的） | ✅ 全 PASS |
| 什么也不放（当前状态） | 必须 PASS | ✅ 全 PASS |

> 这与第十七轮 D66、第十八轮那个「探测器范围开太大」是同一类教训：
> **新写的断言要能失败**才算断言。三次都是靠「让它响一次」发现的。

##### 六、顺带记录：`train_yolo_block.py` 的注释里本来就有 D6 的成因

`scripts/train_yolo_block.py:158-161` 已写明旧版如何把两个模型弄成同名。本轮
不再重复造轮子，只把「同名必须同源」变成**可执行断言**，并把部署要求写进
`README` 与 `docs/待执行命令.md` 的**甲-7**（板上用 `YOLO_MODEL_DIR` 或绝对路径
把模型钉死，与 CWD 无关）。

##### 七、未验证 / 未做

- 板上**还未部署**（甲-7 是给部署那天准备的，未执行）。
- 「换成模型 A 会不会影响仿真 9/9」**不需要重验**：从仓库根启动一直解析到模型 A，
  那条路径**从未改变**；本轮删掉的是从仓库外启动时的**另一份**。
- 两个 run 的 mAP 取自各自的 `results.csv`（训练期验证集），**不是**在同一个
  测试集上重测的 —— 所以 0.966 vs 0.906 只用于「哪个是哪个」的区分，
  不当成严格的能力对比。

##### 八、状态

- 本轮 1 个提交（新增 `test_model_resolution.py` + README/待执行命令/台账更新）。
  删除的两个权重文件**未被跟踪**，不进提交。
- `migrate/humble-orin` 领先 `main` 的提交数**不写死**：本轮开始时实测 **52**。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。

---

#### 第二十轮：板端部署的第一条命令，实测**是坏的**（2026-09-28）

##### 一、那个「既定做法」从来没被执行过

台账与 README 一路写着「板端只编子集即可」。第十九轮验了它的**下半段**（缺
`arm_grasp_sim` 时 launch 不会炸）。这一轮验**上半段**：那条 `colcon build` 命令本身。

做法是**不碰现有 install**：把 build/install/log 都指到临时目录，在工作站上跑板子
将要跑的那条命令。

##### 二、D68：`--packages-select` 那四包，在完整工作区里**构建失败**

```
$ colcon --log-base /tmp/sublog build --packages-select yolo_bringup perception_3d \
      yolo_detector grasp_planner --build-base /tmp/subbuild --install-base /tmp/subinstall
Starting >>> grasp_planner / perception_3d / yolo_detector
Finished <<< grasp_planner [1.27s] / perception_3d [1.25s] / yolo_detector [1.25s]
Starting >>> yolo_bringup
[1.860s] ERROR:colcon.colcon_ros.task.ament_python.build:Failed to find the following files:
- /tmp/subinstall/arm_grasp_sim/share/arm_grasp_sim/package.sh
Check that the following packages have been built:
- arm_grasp_sim
Failed   <<< yolo_bringup [0.00s, exited with code 1]
```

**三个包成功，`yolo_bringup` 失败** —— 而它正是「唯一入口」，没有它这条链路什么都起不来。

报错本身**把人往错方向指**：它让你「检查 `arm_grasp_sim` 有没有编译」，而
`arm_grasp_sim` 恰恰是**板上不该编、也编不了**的那个包（它依赖 `gz_ros2_control`、
Gazebo，板上没有也不需要）。照报错去做，只会在板上白装一整套仿真后端。

**原因**：`yolo_bringup/package.xml` 声明了 `exec_depend arm_grasp_sim`（那是有意的，
注释里写明了理由），而 colcon 构建 Python 包时需要**工作区内依赖**的 install 环境 ——
即使该依赖**没有被选中**。所以「包在源码树里」和「包被编出来」两件事，colcon 在这里
是分开算的：前者足以让它期待后者的 `package.sh`。

##### 三、两个可行的做法（都实测过）

| 做法 | 命令 | 实测 |
|---|---|---|
| **加一个 flag**（推荐） | `colcon build --packages-select <四包> --packages-ignore arm_grasp_sim` | ✅ 4 个包，2.11 s |
| **工作区里根本没有它** | 部署时不把 `arm_grasp_sim/` 拷到板上，然后普通 `colcon build` | ✅ 4 个包，2.17 s |

第二种是把「板上不需要仿真资产」贯彻到**部署内容**上，更彻底；第一种在
「clone 整个仓库」这种最省事的部署方式下也能用。两条都写进了 README。

##### 四、顺带验完的：子集装好之后能不能真的加载（板子的真实形状）

用临时 install 目录 + **环境里确实没有 `arm_grasp_sim`**（实测 `PackageNotFoundError`
的搜索路径里只有那四个包与 `/opt/ros/humble`）跑了两条：

| 命令 | 期望 | 实测 |
|---|---|---|
| `bringup.launch.py camera_source:=realsense --show-args` | 能加载并列出参数 | ✅ 正常列出（含 `--show-args` 不再被缺包拖死，这正是第十八轮那条的**端到端**验证） |
| `bringup.launch.py`（默认 `camera_source:=sim`） | 中止 + 可操作报错 + **什么都不启动** | ✅ 报错文案完整（含补编命令与 `camera_source:=realsense` 的出路），无任何节点启动 |

⇒ **第十八轮验的是「函数在缺包时不会炸」，这一轮验的是「真按板子的形状装出来、
真用 `ros2 launch` 起」** —— 两件事不是一回事，都能过才是真的。

##### 五、新登记的一个坑：板上**不能**直接 `ros2 launch yolo_bringup bringup.launch.py`

`camera_source` 的默认值是 **`sim`**。所以板上那条最自然的命令会去找
`arm_grasp_sim` 并中止。这不是缺陷（默认值必须有一个，工作站上 `sim` 是对的，
而且报错文案明确给了出路），但它是**板上的第一脚就会踩到**的东西，故写进 README：
板上第一条命令按目的选 `camera_source:=external`（只起上层四节点，不碰驱动/MoveIt/臂）
或 `camera_source:=realsense robot_ip:=<控制箱 IP>`。

##### 六、新增到 `docs/待执行命令.md` 的条目

- **甲-8**：编子集（含 `--packages-ignore` 及**不带它的实际报错**）。
- **甲-9**：编完先跑 `camera_source:=external` 验感知链路（不碰臂）。
- 新增「**板端部署（整条路）**」一节：4 步的顺序表，并点明**走到第 4 步都不需要
  机械臂上电、不需要控制箱、不需要授权**；第 4 步之后的真机步骤仍在 W5。

##### 七、未验证 / 未做

- **板上仍未部署**：甲-7 ~ 甲-9 全部未执行。本轮验的是**命令的形状**（本机预演），
  不是「板上能过」—— 板上还多两个变量：**aarch64** 与**板的 Python 3.10.12**
  （本机预演用的是工作站环境）。⚠️ 这两个变量没有通过预演消除，别把本节读成
  「板上一定行」。
- 板上有没有 `image_geometry` / `cv_bridge` / `vision_msgs` 等**运行期**依赖，
  仍未逐个核对（`control_msgs` 已由 D66 那次板上自检间接证明存在）。
- 真机 launch（W2）与现场步骤（W5）不变。

##### 八、状态

- 本轮与第二十一轮一起提交，共 3 个（脚本 1 个、README+待执行命令 1 个、台账 1 个）
  —— 两轮的改动混在同一批文件里，按文件切不开，所以按「工具 / 面向用户的文档 /
  台账」切。⚠️ 因此本轮状态条里的这份清单**不在**某个单独的提交里。
- `migrate/humble-orin` 领先 `main` 的提交数**不写死**：本轮开始时实测 **53**。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。

#### 第二十一轮：板上缺的**不是** pymodbus，是 `vision_msgs`（2026-09-28）

##### 一、上一轮留下的那句话，这一轮去掉了

第二十轮第七节写着「板上有没有 `image_geometry` / `cv_bridge` / `vision_msgs` 等运行期
依赖，仍未逐个核对」。这一轮就是把这句话执行掉 —— 方法是**从代码推导出该核对什么**，
而不是照着一份人手写的清单去问。理由见第四节：`vision_msgs` 正是被清单漏掉的那一个。

##### 二、D69：`vision_msgs` 在板上不存在，而三个节点都在**模块级** import 它

```
$ ssh jetson 'bash -c "source /opt/ros/humble/setup.bash; python3 -c \
    \"import vision_msgs\""'
ModuleNotFoundError: No module named 'vision_msgs'

$ ssh jetson 'bash -c "source /opt/ros/humble/setup.bash; apt-cache policy \
    ros-humble-vision-msgs"'
  Installed: (none)
  Candidate: 4.1.1-1jammy.20260907.225757   # jammy/main arm64，能装
```

三个节点都是**模块级** import（不是函数体内，所以没有「先起来再崩」的中间态）：

| 文件 | 行 | import |
|---|---|---|
| `yolo_detector/detector_node.py` | 38 | `from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose` |
| `perception_3d/projector_node.py` | 51 | `from vision_msgs.msg import (...)` |
| `grasp_planner/planner_node.py` | 91 | `from vision_msgs.msg import Detection3DArray` |

**后果是「三个节点全都起不来」**，而不是「少一个消息包」。这个症状离原因很远：
看起来像项目坏了、像 Python 环境错了、像 colcon 装错了，唯独不像缺一个 apt 包。
按部署次序（`docs/待执行命令.md` 的「板端部署（整条路）」），它卡在**第 2 步之后、
第 4 步（起上层链路）**上 —— 而第 2 步 `colcon build` 是**会成功的**，因为
`vision_msgs` 只被 `exec_depend` 声明，构建期不需要它。于是现场会看到
「装得好好的，一 launch 三个节点全死」。

顺带确证：板上**没有** `pymodbus` 之外的第二类问题被混淆 —— README 原先那句
「缺的只有 AG-95 侧的 `pymodbus`」**是错的**，本轮已改。

##### 三、为什么这次不是「再看一遍文档」

依赖在仓库里有两个来源，**人眼看必然漏**：

1. 四个 `package.xml` 的 `depend` / `exec_depend` / `build_depend` —— 这部分还算好查；
2. 节点源码里**函数体内**的 `import` —— 这部分最容易被漏，而且漏了以后症状更晚出现：
   `detector_node.py:85` 的 `import torch` 在 `__init__` 里、
   `:148` 的 `from ultralytics import YOLO` 在**加载模型时**才执行。
   这类缺失的表现是「节点起来了，一加载模型就崩」，比模块级缺失更难归因。

所以新增了 [`scripts/check_board_deploy.py`](../scripts/check_board_deploy.py)：
它在板上跑，**自己从代码里推导要检查什么**（两个来源都扫），只读、不需要 sudo、
不启动任何节点。缺什么直接给出该装哪一条，装完再跑一遍即可。三段输出：

| 段 | 检查什么 | 数据来源 |
|---|---|---|
| 1 | ROS 包依赖 | 四个 `package.xml`（跳过板上刻意不装的 `arm_grasp_sim`） |
| 2 | Python 模块 | 四个节点源码的**全部** `import`，含函数体内；滤掉标准库与包内自引用 |
| 3 | 模型权重解析 | **调用** `detector_node._resolve_model()` 本体（不重写它的逻辑） |

第 3 段是 D67 的现场体检：它回答「默认参数在当前目录解析到哪个 `block_best.pt`」，
并提示「启动日志里的 `Loading YOLO model:` 应当就是这一行」。

##### 四、这个工具**自己的第一个板端运行**就抓到了它自己的一个错

这是本轮最值得记的一条。第一版第 2 段这样判断「该 apt 还是该 pip」：**去 ament 索引里
问「这是不是 ROS 包」**。而在板上跑出来的正是：

```
  [MISS] vision_msgs  (被 grasp_planner/perception_3d/yolo_detector 需要)
         修法: sudo apt install ros-humble-vision-msgs      ← 第 1 段，对的
  [MISS] vision_msgs  (被 ... import) —— ModuleNotFoundError
         修法: pip3 install vision_msgs                     ← 第 2 段，**错的**
```

错因是**循环**：这个包**正因为缺失**才在索引里查不到，于是「是不是 ROS 包」必然答
「不是」，于是给出 pip 建议。**一个用于防止「报错文案把方向指反」的工具，自己在同一个
位置给出了指反的建议** —— 与 D66（探针把「参数名换代」说成「版本太老」）是同一类错误。

改法：不再问环境（环境正是因为缺失才答不了），改成**静态判据** ——
在 `package.xml` 里声明过，或者名字符合 ROS 生态的惯例（`*_msgs` 后缀、
`rclpy`/`cv_bridge`/`image_geometry`、`tf2*`/`ament_*`/`launch*` 前缀）。
后者不是拍脑袋：`planner_node.py:89` 的 `shape_msgs` 与 `:78` 的 `action_msgs`
**都没在 `package.xml` 里声明**，但确实由 ROS 提供 —— 只靠「声明过」判断会把它们
误判成 pip 包。

并给这个纯函数加了 `--self-test`（9 条，不需要 ROS、本机可跑），把这次的错钉住：
`vision_msgs → apt`、`action_msgs → apt`（未声明但 ROS 提供）、`ultralytics → pip`。

##### 五、实测结果

| 环境 | 结果 |
|---|---|
| 本机（工作站 venv + `ros2_ws/install`） | **全部就绪**（exit 0）。第 3 段解析到仓库根那份 `block_best.pt`，正是第十九轮处置后期望的状态 |
| **板子（Orin）** | 第 1、2 段**各报一条 `vision_msgs` MISS，两处的修法都已是 apt**；第 3 段按设计降级为 `[SKIP]`，并说明「上一条的缺失就是原因」（exit 1） |

第 3 段的 `[SKIP]` 是**故意的**：`detector_node` 因为缺 `vision_msgs` 根本 import 不进来，
所以「读不到 `_resolve_model`」这件事不可能是别的原因。这条降级路径在板端实际走到了。

板子上的 `/tmp/deploycheck`（本轮为验证建的临时仓库副本，244 KB）**已删除**；
`apt-cache policy` 只读；没装任何包、没改板子状态。

##### 六、顺带更正的两处说法

- **README**「缺的只有 AG-95 侧的 `pymodbus`」→ 改为把 `vision_msgs` 放在前面，
  并给出 apt 那一行；同时把「别靠人肉比对清单」写进去、指向新脚本。
- **我自己的**：`check_board_deploy.py` 第一版的 docstring 拿
  「`gripper.py` 里的 `pymodbus`」当「函数体内 import」的例子 —— **是错的**。
  `gripper.py` 全文没有 `pymodbus` 的 import（`:332` 只是报错文案里提到它；
  `Ag95Gripper` 目前仍是构造即报错的桩）。例子已换成真实存在的
  `detector_node.py:85` 的 `import torch`（`__init__` 里）与 `:148` 的
  `from ultralytics import YOLO`（加载时）。⇒ 工具扫出来的结果是对的，**注释是错的**，
  这类错不会让自检变红，只能靠读。

##### 七、未验证 / 未做

- **没在板上装 `vision_msgs`**（要 sudo；命令已进 `docs/待执行命令.md` 的甲-10）。
  所以「装完之后三个节点是否真能起来」**仍未验证** —— 本轮的结论只到
  「缺它 ⇒ 起不来」这一步。
- 本轮的板端检查是在**临时副本** `/tmp/deploycheck` 上跑的，**不是**在板上正式部署的
  仓库里。真正的部署（甲-7 ~ 甲-10）仍未开始。
- `ros2_ws/src` 之外的东西（`scripts/ag95_probe.py` 需要的 `pymodbus`）不在本脚本范围，
  由甲-5 覆盖 —— 原因是它属于部署第 5 步，晚于本脚本要回答的「链路能不能起来」。
- 真机 launch（W2）与现场步骤（W5）不变。

##### 八、状态

- 本轮与第二十轮一起提交，共 3 个：`8f02c17` 脚本、`685c0b0` README+待执行命令、
  本节所在的台账提交。两轮的改动混在同一批文件里（本节与第二十轮在同一份
  `RESTRUCTURE_PLAN.md` 里追加），按文件切不开，所以按「工具 / 面向用户的文档 / 台账」切。
- `migrate/humble-orin` 领先 `main` 的提交数**不写死**：实测 `git rev-list --count
  main..migrate/humble-orin` = **53**（第二十、二十一轮的改动尚未提交，提交后会更多）。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。

#### 第二十二轮：把第二十轮那句「板上一定行」实测掉（板端 aarch64）（2026-09-28）

##### 一、上一轮明确留下的缺口

第二十轮的第七节自己写着：

> 「不是「板上能过」—— 板上还多两个变量：**aarch64** 与**板的 Python 3.10.12**
> （本机预演用的是工作站环境）。⚠️ 这两个变量没有通过预演消除，别把本节读成
> 「板上一定行」。」

这一轮就是去消掉它。第二十一轮又把「装 `vision_msgs` 之前 `colcon build`
**会成功**」写进了 README 与甲-10 —— 那也是**断言**，而且正好落在 D68 同一类
（colcon 到底会不会去找工作区外、又装不上的依赖？D68 已经证明它会去要
`arm_grasp_sim` 的 install 环境）。所以两件事一起验。

##### 二、做法：往板子的 `/tmp` 拷一份源码，build/install 全指到临时目录

不碰板上任何既有目录（`~/yahboom_ws`、`~/ultralytics` 都原样）。源码只有 776 KB
（五个包都是纯 Python，最大的 `yolo_bringup` 196 KB）。

```
$ tar cz -C ros2_ws src | ssh jetson 'rm -rf /tmp/buildcheck && mkdir -p \
      /tmp/buildcheck/ros2_ws && tar xz -C /tmp/buildcheck/ros2_ws && du -sh /tmp/buildcheck'
776K	/tmp/buildcheck
```

##### 三、甲-8 那条命令，在板上**确实通过**（且 `vision_msgs` 没装）

```
$ ssh jetson   # source /opt/ros/humble/setup.bash
$ cd /tmp/buildcheck/ros2_ws
$ colcon --log-base /tmp/buildcheck/log build \
      --packages-select yolo_bringup perception_3d yolo_detector grasp_planner \
      --packages-ignore arm_grasp_sim \
      --build-base /tmp/buildcheck/build --install-base /tmp/buildcheck/install
Starting >>> grasp_planner / perception_3d / yolo_detector
Finished <<< yolo_detector [3.00s] / perception_3d [3.04s] / grasp_planner [3.06s]
Starting >>> yolo_bringup
Finished <<< yolo_bringup [1.83s]

Summary: 4 packages finished [5.48s]
colcon exit=0
```

⇒ 第二十轮那句缺口**消掉了**：aarch64 + Python 3.10.12 + `setuptools 59.6.0`
（板上的，比工作站的旧）都通过。同时这也确证了「**没装 `vision_msgs` 也能编过**」——
`vision_msgs` 只是 `exec_depend`，构建期真的不需要它。这条正是甲-10 存在的理由：
**第 3 步会成功，问题要到第 5 步才炸**。

##### 四、README 给的「等效做法」也在板上通过

README 说「另一种等效做法是**部署时就不把 `arm_grasp_sim` 拷到板上**」。把它从工作区
删掉、**不带** `--packages-ignore` 再编一遍：

```
Summary: 4 packages finished [5.36s]
colcon exit=0
```

两条路都在板上实测通过。⚠️ 但两条**不是**完全等价：如果 `arm_grasp_sim` 的源码在板上
而你不带 flag，colcon 会去编它 —— 那个包要 Gazebo / `gz_ros2_control`，板上没有。
所以「不带 flag」只在**源码也不在**时成立。

##### 五、甲-9 的形状：今天跑，三个节点**确实全死**（这就是甲-10 的现场）

第二十一轮只能从「模块级 import」推断症状。这一轮把它**跑出来**：

```
$ source /tmp/buildcheck/install/setup.bash
$ ros2 launch yolo_bringup bringup.launch.py camera_source:=external
[INFO] [detector_node-1]: process started with pid [90935]
[INFO] [projector_node-2]: process started with pid [90937]
[INFO] [planner_node-3]: process started with pid [90939]
[detector_node-1]  from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
[detector_node-1]  ModuleNotFoundError: No module named 'vision_msgs'
[projector_node-2] from vision_msgs.msg import (
[projector_node-2] ModuleNotFoundError: No module named 'vision_msgs'
[planner_node-3]   from vision_msgs.msg import Detection3DArray
[planner_node-3]   ModuleNotFoundError: No module named 'vision_msgs'
[ERROR] [detector_node-1]: process has died [pid 90935, exit code 1, ...]
[ERROR] [projector_node-2]: process has died [pid 90937, exit code 1, ...]
[ERROR] [planner_node-3]: process has died [pid 90939, exit code 1, ...]
```

三个都**起来了又立刻死**（exit code 1），每一个的 traceback 末行都是同一件事。
**这就是第二十一轮说的「看起来像项目坏了、像 Python 环境错了」的现场长相** ——
`process has died` 长得像崩溃，而它其实只是一个 apt 包没装。

⇒ 「先装 `vision_msgs`（甲-10）再起链路（甲-9）」这个次序，现在是**实测**的，
不是推断的。

##### 六、顺带验完的：D50 的修复在真机入口下**真的生效**（aarch64 上）

上面那次 launch 的 `planner_node` 命令行里带着：

```
/tmp/buildcheck/install/grasp_planner/lib/grasp_planner/planner_node
  --ros-args -r __node:=grasp_planner
  --params-file /tmp/buildcheck/install/grasp_planner/share/grasp_planner/config/grasp_planner.yaml
  --params-file /tmp/launch_params_r9js2nk3
```

`grasp_planner.yaml` **在真实入口（`bringup.launch.py`）的参数表里** —— 这正是 D50
当年缺的东西（当年只有 `{'gripper_backend': ...}`）。上一轮是在工作站上验的 D50，
这一轮在板上、用真实 launch、看真实进程的命令行，得到同一结论。
（路径解析在 `install/` 下、由 `get_package_share_directory('yolo_bringup')` 那条
修法给出 —— 见第十八轮。）

##### 七、未验证 / 未做

- 板端这条链路**仍然没有跑起来过**：三个节点今天必然死，因为 `vision_msgs` 没装。
  本轮验的是「缺它 ⇒ 死」，**不是**「装了就活」—— 后者要等甲-10 执行完。
- 本轮用的仍是**临时目录**（`/tmp/buildcheck`、`/tmp/buildcheck2`，共 776 KB ×2），
  **不是**板上正式部署的仓库。正式部署（甲-7 ~ 甲-10）仍未开始。
- `arm_grasp_sim` 在板上**能不能编**没测（不需要测：它要 Gazebo，板上不装）。
- 板子上 `ros2 launch` 留下的 `/tmp/launch_params_*` **没有清**：那是 ros2 自己建的、
  且有几份是 9-26 就存在的（你先前跑的），不是本轮产物。它们各几百字节，无害。
- 真机 launch（W2）与现场步骤（W5）不变。

##### 八、状态

- 本轮 1 个提交（README 的实测数字 + 待执行命令的甲-8/甲-10 验证行 + 本节）。
- `git rev-list --count main..migrate/humble-orin` 实测 **57**（本轮提交前）。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。
- 板子状态：临时目录已删、无残留进程、`~/yahboom_ws` 与 `~/ultralytics` 未被动过。

#### 第二十三轮：板端把「检测」这一半真的跑通了 —— 以及相机那一层的一步之差（2026-09-28）

##### 一、为什么查这里

甲-10（装 `vision_msgs`）之前，链路里有一大块**不需要 `vision_msgs`、也不需要 sudo**
就能验的东西：权重能不能被板上的 torch/ultralytics 读进去、能不能真的推出框、
相机能不能出图。这三件事平时被归到「等部署那天再说」，但它们恰恰是**版本兼容**
最容易咬人的地方（板上是 `torch 2.5.0a0+nv24.08` + `ultralytics 8.4.26`，
与工作站的 `2.5.1+cu121` + `8.3.99` **都不是同一份**）。所以先验掉，省得部署那天
把「版本不兼容」误判成「项目坏了」。

##### 二、先确认权重**随仓库走**（甲-7 的前提）

甲-7 让现场「把权重放到位」。但权重到底要不要**另外**弄上去？查 git：

```
$ git ls-files | grep -i '\.pt$'
block_best.pt                         ← 6.2 MB，被跟踪
runs/block_detect/weights/best.pt
runs/block_detect/weights/last.pt
runs/detect/block_grid_demo/weights/best.pt
runs/detect/block_grid_demo/weights/last.pt
yolo26n.pt
yolov8n_coco.pt
$ git check-ignore -v block_best.pt   → （未被 ignore）
$ sha256sum block_best.pt             → a926d959b5aa9f9e2435c2deb9666803...
```

⇒ **`git clone` 就带过来了**，与第十九轮记的 sha256 一致。甲-7 要做的只是
「别让它按 CWD 解析到别处」，不是「把文件传上去」。

##### 三、板端加载 + 推理：通过

```
$ python3 -c "..."
torch 2.5.0a0+872d972e41.nv24.08 | cuda available: True
ultralytics 8.4.26
加载耗时 0.1s
names: {0: 'block'}
task: detect
空图推理耗时 0.89s, 框数 0        ← 含 CUDA 首次预热
预热后单帧推理: 38 ms
```

##### 四、拿真图验：9 个标签 / 9 个框，逐对 IoU 全部 ≥ 0.966

用的是 `block_dataset/val/images/img_0001.jpg`（该图有 9 个标注框），做**贪心匹配**
后逐对算 IoU：

```
标签 9 个 / 检出 9 个
逐对 IoU: 0.966 0.969 0.972 0.976 0.976 0.978 0.980 0.983 0.988
conf 区间: 0.913 ~ 0.966
```

⚠️ **这张图是那次训练自己的 `val` 划分**（`train_yolo_block.py:49-51` 就取
`block_dataset/val`），所以「9/9 恰好对上」**不构成泛化性的证据**。它证明的是
**另一件事、也是本轮要证的事**：板端（aarch64 + 那份 NGC torch + ultralytics 8.4.26）
**复现了工作站上那个模型的行为**，没有因为版本不同而退化。

##### 五、38 ms/帧顺带回答了一个 W2 的担心

W2 里担心过 `max_sync_dt`（默认 0.1 s，超过就**丢弃整批检测**，
`projector_node.py:348-352`）——当时的假设是「Orin 更慢，会更频繁超时」。
实测预热后单帧 **38 ms**（约 26 FPS），比那 0.1 s 的窗口宽裕得多。
⚠️ 这只是**裸推理**的耗时，不含取图、投影、TF；真正的端到端延迟没测。

##### 六、相机那一层：**差一步就静默失效**（本轮唯一的新增可操作发现）

先按最朴素的方式起相机：

```
$ ros2 launch realsense2_camera rs_launch.py enable_depth:=false
$ ros2 topic list | grep camera
/camera/camera/color/image_raw          ← 两层！
/camera/camera/color/camera_info
$ ros2 topic hz /camera/color/image_raw
WARNING: topic [/camera/color/image_raw] does not appear to be published yet
```

**最后那条 WARNING 就是现场会看到的症状**：相机明明在出图，而下游订阅的
`/camera/color/image_raw`「还没有发布」—— 于是检测器一帧都收不到、没有 `/detections`、
规划器永远没目标，日志里只有「没目标」。看起来像检测器坏了或模型不行。

这件事**不是新发现**：两层命名空间是 2026-09-27 就在板上量到的，
`pipeline.launch.py` 里已有 `REALSENSE_REMAPS` 修好它。本轮复现了那次测量
（独立确认），**但发现了一个没写进文档的边界**：

| 谁起相机 | 入口 | remap 生效？ | 结果 |
|---|---|---|---|
| **仓库自己** | `camera_source:=realsense` | ✅ 生效 | 单层话题，下游能收到 |
| **别人**（包括你手敲 `rs_launch.py`） | `camera_source:=external` | ❌ **不生效** | 两层话题，**静默**收不到 |

原因是 remap 挂在**相机节点**上（`pipeline.launch.py:318-327`），
只有仓库自己拉那个节点时才带得上。⇒ **甲-9 推荐的正是 `external`**，而它没有
这一句警告。已补进甲-9。

顺带把 remap 的作用范围量清了（免得以后误以为「remap 把相机全改成单层了」）：

```
$ ros2 launch yolo_bringup pipeline.launch.py camera_source:=realsense
$ ros2 topic list | grep -i camera | sort
/camera/aligned_depth_to_color/image_raw        ← 三个要用的：只剩单层
/camera/color/camera_info
/camera/color/image_raw
/camera/camera/aligned_depth_to_color/camera_info   ┐
/camera/camera/aligned_depth_to_infra1/...          │ 其余 16 个仍是两层,
/camera/camera/color/metadata                       │ 但**没有下游消费**
/camera/camera/depth/image_rect_raw                 │ （infra / metadata /
/camera/camera/extrinsics/...                       │  extrinsics / 未对齐深度）
/camera/camera/infra1|infra2/...                    ┘
```

三个被 remap 的话题**在两层位置上都不再出现**（不是「多发一份」），
`/camera/camera/color/image_raw` 与 `/camera/camera/color/camera_info` 都不在表里。

##### 七、未验证 / 未做

- 板的链路**仍然没起来过**：上面那次 `pipeline.launch.py` 里三个节点照旧因为
  `vision_msgs` 全死（`grep -c "process has died"` = 3）。本轮验的是**检测与相机
  各自那一半**，**不是**「它们接过来了」—— 端到端要等甲-10。
- **投影那一半没验**（`perception_3d` 要 `vision_msgs`，且要真 TF）。
  所以「板端 38 ms 推出来的框，投到 3D 对不对」**没测**。
- 相机只验了**彩色**：`enable_depth:=false`（第一次），第二次走仓库分支时深度话题
  在，但**没有真去取一帧深度值**。D2 要的「对齐深度 + 彩色内参」组合
  **没有端到端验过**。
- 内参只确认了 `width: 1280 / height: 720` 与 `camera_info` 话题有数据，
  **没有读 fx/fy/cx/cy 的具体数值**（grep 没匹配到 K 矩阵的格式）。
- 正式部署（甲-7 ~ 甲-10）仍未开始；本轮用的还是临时目录。
- 真机 launch（W2）与现场步骤（W5）不变。

##### 八、状态

- 本轮 1 个提交（README 的相机边界 + 甲-9 的警告 + 本节）。
- `git rev-list --count main..migrate/humble-orin` 实测 **59**（本轮提交前）。
- **未推送**，`main` 未动（仍为 `0ab13ec`）。
- 板子状态：`/tmp/bc`、`/tmp/bc2` 已删；第一次探测留下的相机进程按 PID 停掉
  （`kill 91352 91354 91355`，没用 `pkill -f`）；`ps` 复核**无残留**；
  `~/yahboom_ws`、`~/ultralytics` 未被动过。
  ⚠️ 清理时有一条命令**被权限拦下**：我写了 `rm -rf /tmp/buildcheck* /tmp/launch_params_*`，
  而 `/tmp` 是共享目录、那个 glob 会扫到**不是我建的**文件（`launch_params_*` 里有
  9-26 就存在的）。拦得对，改成只删自己建的两个目录。
