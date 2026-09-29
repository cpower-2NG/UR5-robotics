# YOLO + ROS2 视觉抓取系统 (YOLO-ROS)

> YOLOv8 检测 → 2D→3D 投影 → MoveIt2 逆解 → UR5 抓取，**仿真闭环已全量跑通**。
> 部署目标：**NVIDIA Orin（JetPack 6.2 / aarch64）接真机 UR5**。

---

## 现在能跑到什么程度

**仿真里 9/9 方块全自动搬运成功**，且经 Gazebo 物理真值核对（不是关节回读）：

| 证据 | 实测值 |
|---|---|
| 搬运数量 | 9/9，与规划器日志「累计 9 个」逐一对应 |
| 落点一致性 | x 跨度 0.7 mm、y 跨度 10.1 mm |
| 堆叠高度 | z 0.020 → 0.340（九层，每层 40 mm 方块） |

**尚未证明的三件事**（别把上表读成「全都行了」）：

- **检测框本身准不准** → 归 YOLO 模型。仿真里 9/9 检出、分数 0.68~0.94，但**未在真机数据上量过**。
- **相机外参准不准** → 归**手眼标定**。仿真里相机位姿是我们自己写死的常量，真机必须实测；代码层修不了。
- **真机夹爪能不能用** → AG-95 后端的**代码侧已就绪**（读/写/到位轮询都在 `gripper.py`，
  写路径由 `ag95_dry_run` 挡着、默认只读）。挡着的是**硬件**：板上还没有 USB-RS485
  适配器，后端一构造就去开串口、打不开就抛。适配器到位后按 `docs/待执行命令.md`
  的 **丙-1** 走（只读探映射 → 离臂试写 → 上臂）。

> 真机动作属高风险操作，需当次明确授权与安全条件确认后才可启动。

---

## 环境要求

实测环境（**以 Orin 的兼容性为准**，故全线用 Humble）：

| 组件 | 实际版本 | 核实命令 |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS (jammy) | `cat /etc/os-release` |
| ROS 2 | **Humble** | `ls /opt/ros/` |
| Gazebo | Harmonic，`gz sim 8.15.0` | `gz sim --versions` |
| ROS↔GZ 桥 | `ros-humble-ros-gzharmonic-sim` | 注意是 **gzharmonic 变体**，非默认 `ros-gz` |
| MoveIt2 | `ros-humble-moveit` | |
| Python | **3.10.21**，conda 环境 `robot-vision` | `~/robotics/venvs/robot-vision/bin/python --version` |
| 关键包 | numpy 1.26.4 / torch 2.5.1+cu121 / ultralytics 8.3.99 | 该环境内 `python -c "import numpy,torch,ultralytics"` |

> ⚠️ 本仓库的 `docs/TECHNICAL_SPEC.md` 写作时假定的是 **Jazzy / 24.04 / Python 3.12**，
> 与上表**不符**。该文档顶部有完整对照表，请对照阅读。

**两个解释器问题（必须理解，否则节点起不来）**：`ultralytics`/`torch` 装在 conda 环境里，
而 `rclpy` 在 `/opt/ros/humble`。运行时要把两者**叠加**（见下方"运行"）。

---

## 构建

```bash
cd ros2_ws
colcon build                 # 不要加 --symlink-install
source install/setup.bash
```

两个坑：

- **构建必须在纯净 ROS 环境下做**。若先叠加了 Python 依赖环境，`colcon build` 会因
  新版 setuptools 移除了 `setup.py develop --uninstall` 而失败。构建期用干净环境，
  **运行期**才叠加。
- **不要用 `--symlink-install`**，它同样会触发上面那条路径。

---

## 运行

### 仿真

```bash
# 1) 先叠加 gz_ros2_control（见下，只需准备一次）
source ~/grasp_ws/install/setup.bash

# 2) 叠加驱动 YOLO 的 Python 环境
source /opt/ros/humble/setup.bash
export PYTHONPATH=~/robotics/venvs/robot-vision/lib/python3.10/site-packages:$PYTHONPATH

# 3) 起车（唯一入口）
source ros2_ws/install/setup.bash
ros2 launch yolo_bringup bringup.launch.py camera_source:=sim
```

约 20 秒后机械臂开始自主抓取。**无需外部触发**——规划器订阅 `/targets_3d`，
每来一帧就选一个目标去抓（按 Y 从左到右、跳过已抓过的），抓完自动进入下一轮。

常用开关：

```bash
... camera_source:=sim skip_planner:=true    # 只起仿真+感知，调 YOLO 用
... camera_source:=sim headless:=true        # 无 GUI
... camera_source:=sim model:=其他.pt device:=cpu
```

### `gz_ros2_control` 前置（仿真必需，真机不需要）

apt 上的 `ros-humble-gz-ros2-control` **依赖 Fortress（gz-sim6）**，与 Harmonic
（gz-sim8）ABI 不通，装了也跑不起来。必须用 `GZ_VERSION=harmonic` 源码编译，且
**放在外部叠加工作区**（本仓库不携带这份第三方源码）：

```bash
mkdir -p ~/grasp_ws/src && cd ~/grasp_ws/src
git clone -b humble https://github.com/ros-controls/gz_ros2_control
cd ~/grasp_ws && export GZ_VERSION=harmonic
colcon build --packages-up-to gz_ros2_control
```

### 训练数据（方阵世界，可选）

`docs/GRID_ENV_DESIGN.md` 的训练流水线（采集图 → 标注 → 训 YOLO）需要**没有机械臂**
的方阵世界：

```bash
ros2 launch yolo_bringup grid_world.launch.py                    # 3×3 方阵
ros2 launch yolo_bringup grid_world.launch.py world:=color_objects_world.sdf
```

⚠️ **两个「标注」脚本不要混用，它们不是一回事**：

| 脚本 | 适用 | 状态 |
|---|---|---|
| `scripts/generate_synthetic_data.py` | **只看仿真**：自己用 OpenCV 渲染图 + 自动出标签，生成 `block_dataset/` | ⚠️ **它的投影是转置的（D61）** —— 但它**渲染和标注共用同一个函数**，图与标签自洽，所以数据集本身有效、`block_best.pt` 不必重训。**不要拿它当真机的起点**（它的相机模型**没有旋转**，表达不了手眼外参） |
| `scripts/auto_label_grid.py` | **真机（和仿真）**：`perception_3d/projector_node.py` 投影链的**严格逆运算**，走 tf + `/camera/color/camera_info`，含单应后端 | ✅ 自检 `scripts/test_label_projection.py` 全过。采集端用现成的 `scripts/collect_training_data.py`（订阅 ROS 话题，真机可直接用） |

详见 `docs/RESTRUCTURE_PLAN.md` 第十二轮的 W4 一节与 D61 一条。

### 真机首验（相机来源三选一）

`camera_source` 决定**谁提供 `/camera/*` 话题**，也顺带决定起不起仿真：

| 值 | 起相机驱动？ | 起 Gazebo？ | 用途 |
|---|---|---|---|
| `sim`（默认） | — | ✅ | 仿真 |
| `realsense` | ✅ RealSense 驱动 | ❌ | 接真机 RealSense |
| `external` | ❌ | ❌ | 话题已由别处提供（rosbag / 自建 launch） |

```bash
# 真机 + RealSense，只验感知不抓取（先跑通这条再谈抓取）
ros2 launch yolo_bringup bringup.launch.py \
    camera_source:=realsense skip_planner:=true tf_required:=true
```

`tf_required:=true` 是**真机建议值**：查不到 TF 就让 `perception_3d` 报错退出，
而不是发一帧坐标可疑的数据下去——仿真里默认 `false`（TF 树起得慢，早几帧查不到是正常的）。
⚠️ **`gripper_backend` 的默认值随入口变，两个真机入口不一样**（见上）：

| 入口 | `gripper_backend` 默认 | 要额外传吗 |
|---|---|---|
| `camera_source:=realsense`（→ `real.launch.py`） | **`none`**（只走位，不报错） | 不用 |
| `camera_source:=external`（→ `pipeline.launch.py`） | **`ag95`** | **必须**传 `gripper_backend:=none`，否则规划器启动即死（见下） |

> **「看不到目标」先看这一条**：`perception_3d` 会把**检测↔深度时间差超窗**的批次整批丢掉
> （不报错退出），日志每 5 s 一条 `…丢弃本批检测（累计 N 批）`。真机默认窗口已经是 **0.3 s**
> （仿真 0.1 s），若 **N 持续增长**就调大：`max_sync_dt:=0.5`。调小它的症状恰好也是
> 「一直没目标」，而那时日志里**只有这一条线索**。

> **`external` 入口是唯一要提防的**：它落到 `ag95`，而后端一构造就去开串口 ——
> 板上没有 USB-RS485 适配器，于是 `planner_node` **启动就死**（`RuntimeError: 打不开
> 串口 /dev/ttyUSB0`）。所以 `external` 一定要带 `gripper_backend:=none`。
> 这只影响「只验感知」这条路：`realsense` 入口的默认已经是 `none`，不会报错。

---

## 数据流

```
yolo_detector/detector_node
    ↑ /camera/color/image_raw        ↓ /detections
perception_3d/projector_node         （唯一的 2D→3D 投影点）
    ↑ /camera/depth/image_raw + /camera/*/camera_info + TF
    ↓ /targets_3d   Detection3DArray — 全部候选（含 id/类别/分数）
    ↓ /target_3d    PoseStamped     — 最高分那一个（向后兼容）
grasp_planner/planner_node           （零投影代码，只消费 /targets_3d）
    ↓ FollowJointTrajectory → joint_trajectory_controller
UR5 + 夹爪
```

**为什么是两个话题**：规划器要按 Y 排序、按世界坐标去重、取最高分，这三件事都需要
「全部候选」；而一个 `PoseStamped` 只装得下一个目标且不含 id/分数，订阅者无法分辨
哪条是哪个物体。投影**只在 `perception_3d` 里发生一次**，真机与仿真共用同一条路径。

---

## 目录结构

```
yolo_ros/
├── ros2_ws/src/
│   ├── yolo_detector/     # YOLOv8 检测；设备退化、权重路径解析
│   ├── perception_3d/     # 2D→3D 投影（唯一投影点）；深度尺度/时间戳配对/TF
│   ├── grasp_planner/     # 抓取规划；夹爪抽象层（sim / ag95）
│   ├── arm_grasp_sim/     # 仿真世界 + UR5 + 控制器 + MoveIt  ← 工作站专用
│   └── yolo_bringup/      # 唯一入口 bringup + pipeline + 方阵世界
├── scripts/               # 自检、训练、数据合成脚本
└── docs/
    ├── RESTRUCTURE_PLAN.md   # ★ 缺陷台账 + 改造计划 + 验收证据（先读这个）
    ├── 待执行命令.md          # ★ 要**你亲自执行**的命令（sudo / 板子 / 真机），带勾选与执行记录
    ├── TECHNICAL_SPEC.md     # 设计稿（含环境对照表，注意其前提已过时）
    ├── GRID_ENV_DESIGN.md    # 方阵世界与训练流水线
    ├── MIGRATION_HANDOFF.md  # Orin 迁移交接
    ├── JETSON_SSH.md         # 板子 SSH 连接方式（含连接信息与字段说明）
    ├── board/                # 板端资产留档：6 个 arm-*.sh + 体检清单 + UR5 接入方案
    ├── YOLO_TUTORIAL.md, YOLO_VISION_DESIGN.md
    └── reference/            # 外部参考项目（不入库）
```

---

## 自检与验收

七个脚本，**层层递进**，都不需要 Gazebo（`test_ag95_probe.py`、`test_ag95_gripper.py`
与 `test_real_launch.py` 连真夹爪、真串口都不需要 —— 前两个用一个手写的 Modbus 从站
接在 pty 上验）：

```bash
source /opt/ros/humble/setup.bash
# test_minimal 要 import ultralytics，所以必须叠加 conda 环境（其余六个不需要，
# 但叠了一起跑没问题）。不加这一行，test_minimal 会报「ultralytics 未安装」，
# 看起来像项目坏了 —— 其实只是解释器不对。
export PYTHONPATH=~/robotics/venvs/robot-vision/lib/python3.10/site-packages:$PYTHONPATH

python3 scripts/test_minimal.py          # 冒烟：模型加载 + 投影数学（3 项）
python3 scripts/test_perception_3d.py    # 感知单元：23 项断言
python3 scripts/test_detector_device.py  # 设备退化分支：19 项断言
python3 scripts/test_real_launch.py      # launch 参数面 + 收工卫生：80 项（本机，另有 3 项 SKIP）
                                         #   板上是 81 项、2 SKIP（ur_robot_driver 只有板上有）
                                         #   本机的三个 SKIP：没装 ur_robot_driver；
                                         #   planner_node 与 fake_joint_states 回调里
                                         #   没有 publish（那条竞态断言不适用）
                                         #   其中 9 项是「收工时必须是 cleanly」（W2-9，D70）
python3 scripts/test_model_resolution.py # 加载的是哪个模型：12 项断言（D67）
python3 scripts/test_ag95_probe.py       # AG-95 只读探针：40 项断言
                                         #   没有 pymodbus 时 → 26 PASS / 6 SKIP
python3 scripts/test_ag95_gripper.py     # AG-95 驱动（读 / dry-run 闸门 / 写）：51 项断言
                                         #   没有 pymodbus 时 → **退出码 2**（见下）
```

> 上面这些数字是**实测**值（2026-09-28）。它们会随断言增加而变，改脚本时请顺手核对
> —— 写错不影响运行，但会让人以为「我这边少跑了几项」。
> ⚠️ **三个脚本缺依赖时的行为不一样，别只看退出码**：
> - `test_ag95_probe.py`：只有第 2 段要 pymodbus，所以缺了会**降级**跑其余部分，
>   报 `26 PASS / 6 SKIP` 且**退出码 0** —— 这次运行覆盖不全，不能当「验过了」。
> - `test_ag95_gripper.py`：每一项都走 pymodbus 的串口客户端，**没有可降级的部分**，
>   所以缺了直接**退出码 2**（按 `test_real_launch.py` / `test_model_resolution.py`
>   的先例）。它**不假装跑了一部分**。
> - 全跑的办法都一样：`pip3 install "pymodbus>=3.6"`。

**Ctrl-C 收工验收**（**在板子上跑**，不是 WSL）——「正常收工」这件事只有真被 Ctrl-C
一次才知道，静态检查只能保证写法对：

```bash
python3 scripts/board_ctrlc_check.py     # 真起 pipeline，25 秒后写 \x03（真 Ctrl-C）
                                         # 判据：4 个节点全报 `finished cleanly`、
                                         #       无 Traceback、launch 退出码 0
```

它用 pty 复现真终端的中断，因为 `timeout` 和 `命令 &` **都会给出假结论**（整组信号
＋ `SIGINT` 继承），理由写在脚本 docstring 里 —— 那条值得一读，否则容易照着假故障去
改本来正确的代码。

**无臂 MoveIt 预检**（**在板子上跑**）——机械臂还没接线时，把 MoveIt 链路
除驱动外的部分整条验一遍，顺带拿到抓取区/放置区的肘高余量：

```bash
python3 scripts/moveit_only_check.py     # 起无驱动 real.launch + RSP + 假 /joint_states，
                                         # 再跑肘探针。零运动风险（没有 controller_manager）
```

⚠️ 它用**未标定** URDF，结论是「链路通＋余量级」；上电前的最终判读必须用
`ur_calibration` 校准后的 URDF 重跑。两个实测踩过的坑（少 RSP ⇒ `FRAME_TRANSFORM_
FAILURE`、少 `/joint_states` ⇒ `/compute_ik` **挂住不回话**）都写在脚本注释里。

**板端全链路吞吐**（**在板子上跑**）——不接臂、不进仿真器，用**实物俯视相机**
把感知链路整条拉起来测吞吐，回答「这块板子扛不扛得住真机那条链路」：

```bash
python3 scripts/board_chain_check.py     # 起无驱动 real.launch + RSP + 假 /joint_states
                                         # + 相机占位 TF，逐级测相机→检测→三维目标
```

实测（Orin NX，2026-09-29）：相机 25.1 FPS → `/detections` **22.2 FPS** →
`/targets_3d` **22.6 FPS**；运行中内存 **2.2 / 7.4 GiB**、GPU 73%、结温 57.6 °C、
整板 **11.5 W**。规划器只要 1 Hz 级 ⇒ 余量十几倍。

⚠️ 两个要点：① 它用**占位**相机外参（相机在上方 1 m 朝下看），只为接通 TF 树，
所以 `/targets_3d` 的**数值不可信**，它断言的是链路通不通、跑多快；真外参等手眼标定。
② `planner_node` 会报「轨迹 action 服务不可用」并停住 —— **预期**，没有驱动就没有
控制器。**本脚本不覆盖运动**，那一步要真臂或仿真器。

⚠️ **要在板子上跑仿真，先看 `docs/待执行命令.md` 甲-14。** arm64 上**没有**
`ros-humble-ros-gzharmonic`（ROS↔Gazebo 桥只有 amd64 构建），而板上的
`ros-humble-ros-gz-*` 是配 **Fortress** 的、与本项目的 **Harmonic** ABI 不通
⇒ 必须加 OSRF 源装 `gz-harmonic`，再**源码编译** `ros_gz` 与 `gz_ros2_control`
（`GZ_VERSION=harmonic`）。上游自己也把 Humble + Harmonic 列为 "build from source"。

**板端录包 → 搬回开发机看**（板子的图形界面是关掉的，这样看最省事、零 sudo）：

```bash
# 板子上（链路要另开终端起着；本脚本只录不起链路）
python3 scripts/board_bag_record.py --profile perception --seconds 30 --out ~/bags/pick_demo
# 开发机上
rsync -avP --info=progress2 jetson:~/bags/pick_demo ~/bags/
ros2 bag play -l --clock ~/bags/pick_demo &
ros2 run image_transport republish compressed raw --ros-args \
    -r in/compressed:=/camera/color/image_raw/compressed -r out:=/camera/color/image_raw &
rviz2 -d ros2_ws/src/yolo_bringup/config/grasping.rviz --ros-args -p use_sim_time:=true
```

实测 20 秒 perception 档 = **36.5 MB**，rsync 回来 **8.1 MB/s**；JPEG 后 **130 KB/帧**
（原图 2.76 MB，小 21 倍）。⚠️ 三个坑都在 **甲-15** 里写清了：latched 话题要 QoS 覆盖
（否则 `/tf_static` 空、回放 TF 断根）、`--clock` 与 `use_sim_time` 必须配对
（否则模型僵住）、**RViz 的 `Transport Hint` 实测被忽略**（所以要用上面那条 republish 桥）。

**端到端**（需要 pipeline 正在运行）——这是唯一能回答「链路是否真的通」的脚本：

```bash
# 终端 1
ros2 launch yolo_bringup pipeline.launch.py camera_source:=external
# 终端 2
python3 scripts/test_pipeline_e2e.py     # 15 项断言
```

它真起话题、真走 DDS，把合成的 `camera_info`+深度+检测框喂进运行中的 pipeline，
再断言 `/target_3d` 与 `/targets_3d` 的**具体数值**。

> ⚠️ 跑之前先清残留节点（上一次仿真没退干净的进程会发假 TF，症状酷似链路 bug）：
> `ps -eo pid,comm | grep -E "static_transf|projector|detector|planner"`。
> 清理用 `pkill -9 -x <精确进程名>`，**不要用 `pkill -f`**——它会匹配到执行该命令的
> shell 自己。

---

## 真机部署（Orin）

- 部署分层：真机由 `ur_robot_driver` 走 **RTDE（以太网）**，**不装 Gazebo /
  `gz_ros2_control` / `arm_grasp_sim`**（那些是工作站专用）。
- **MoveIt 板上已经装好**（`ros-humble-moveit-*` 2.5.10，2026-09-27 只读实测）—— **不用再装**。
  `torch`/`ultralytics`/RealSense/UR 驱动同样就绪。
- ⚠️ **但板上还缺一个会挡住整条链路的包：`vision_msgs`**（2026-09-28 实测）。
  三个节点都在**模块级** `from vision_msgs.msg import ...`，所以缺它的表现是
  **三个节点全都起不来** —— 看起来像项目坏了或 Python 环境错了，唯独不像「少装一个消息包」。
  修法是 apt，不是 pip（`pip3 install vision_msgs` 会白忙）：

  ```bash
  sudo apt install ros-humble-vision-msgs        # 板上有 4.1.1-1jammy arm64
  ```

  另缺 AG-95 侧的 `pymodbus`（⚠️ 用 `pip3 install "pymodbus>=3.6"`，**不要**用 `apt` 的
  `python3-pymodbus` —— 那是 2.1.0，API 不兼容）。
- ✅ **检测这一半已在板上跑通**（2026-09-28 实测）：权重 `block_best.pt` **被 git 跟踪**，
  `clone` 即得（sha256 `a926d959…`）；板上 `torch 2.5.0a0+nv24.08` + `ultralytics 8.4.26`
  加载 0.1 s、`names={0:'block'}`、CUDA 可用；拿数据集里一张真图（9 个标注框）推理，
  **9 个框全部对上、逐对 IoU ≥ 0.966**；预热后**单帧 38 ms**（约 26 FPS）。
- 📷 **相机话题的「两层命名空间」陷阱**（真机部署最容易白忙一场的地方）：
  `realsense2_camera` 发的是 **`/camera/camera/color/image_raw`**（`camera_namespace` 与
  `camera_name` 默认都是 `camera`），而下游三处都写死单层。仓库里那组
  `REALSENSE_REMAPS`（`pipeline.launch.py`）把它收回单层，**但它挂在相机节点上** ——
  ⇒ 只有 `camera_source:=realsense`（**仓库自己拉相机**）才生效；
  用 `camera_source:=external` 而自己手敲 `rs_launch.py` 起相机，**remap 不会生效**。
  症状是**静默的**：`ros2 topic hz /camera/color/image_raw` 报
  `does not appear to be published yet`，而相机其实在出图 —— 于是没有 `/detections`，
  看起来像检测器坏了。2026-09-28 板端实测复现；`camera_namespace:=""` 改不掉（被 rcl 拒），
  只能用 remap。
- **别靠人肉比对清单**：依赖散在四个 `package.xml` 和若干**函数体内**的 `import` 里，
  靠眼睛看必然漏（`vision_msgs` 就是这么漏掉的）。用
  [`scripts/check_board_deploy.py`](scripts/check_board_deploy.py) 在板上跑一遍 ——
  它**自己从代码推导**该查什么，缺什么直接给出该装哪一条：

  ```bash
  source /opt/ros/humble/setup.bash
  python3 scripts/check_board_deploy.py          # 0 = 就绪；1 = 有缺失（逐条给修法）
  ```
- **板端只编四个包**，且**必须带 `--packages-ignore arm_grasp_sim`**（实测，见下）：

  ```bash
  cd ros2_ws
  colcon build --packages-select yolo_bringup perception_3d yolo_detector grasp_planner \
      --packages-ignore arm_grasp_sim
  ```

  ✅ **这条命令已在板上实测通过**（Orin aarch64 / Python 3.10.12 / setuptools 59.6.0，
  `4 packages finished [5.48s]`，2026-09-28）。注意当时 `vision_msgs` **还没装** ——
  它只是 `exec_depend`，**构建期不需要它**。所以「编过了」不等于「能跑」，
  见上面那条。

  ⚠️ **不带那个 flag 会构建失败**，报错是
  `Failed to find ... arm_grasp_sim/share/arm_grasp_sim/package.sh` /
  `Check that the following packages have been built: arm_grasp_sim` —— 而
  **那个包正是你不想在板上编的**。原因：`yolo_bringup` 声明了
  `exec_depend arm_grasp_sim`，colcon 构建 Python 包时需要**工作区内依赖**的 install
  环境，即使它没被选中。另一种等效做法是**部署时就不把 `arm_grasp_sim` 的源码拷到板上**
  （也在板上实测通过：`4 packages finished [5.36s]`）。⚠️ 但两条**不是**完全等价 ——
  「不带 flag」只在**源码也不在**时成立；源码在而你不带 flag，colcon 会去编它，
  而它要 Gazebo / `gz_ros2_control`，板上没有。
  （工作区里没有它，自然不用 ignore）。
- `bringup.launch.py` 对缺 `arm_grasp_sim` 做了容错并给出可操作报错
  （由 `test_real_launch.py` 的 W2-15 钉住）。
- ⚠️ **`camera_source` 的默认值是 `sim`**，所以在板上**不能**直接
  `ros2 launch yolo_bringup bringup.launch.py` —— 那会去找 `arm_grasp_sim` 并中止
  （实测：报错文案里就写着该改用哪条命令，且**什么都没启动**）。板上的第一条命令按目的选：

  ```bash
  # 只想验感知链路（不碰驱动、不碰 MoveIt、不碰臂）—— 板上最安全的第一步
  ros2 launch yolo_bringup bringup.launch.py camera_source:=external

  # 完整真机链路（需要控制箱 IP；臂没接上/没配好时先别跑这条）
  ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense robot_ip:=<控制箱 IP>
  ```
- **模型权重必须显式指定，别依赖「按目录找」**（D67）。`block_best.pt` 在仓库根、
  被跟踪，所以能随 clone 过去；但 `model:=block_best.pt` 这个**相对名**是按
  **启动时的 CWD** 解析的 —— 从仓库根启动才找得到。板上的推荐写法是
  `export YOLO_MODEL_DIR=<放权重的目录>`（或直接传绝对路径），这样与 CWD 无关。
  包内**刻意不放任何权重**：找不到时会抛出可操作的 `FileNotFoundError`，
  而不是静默换一个别的模型。
- 夹爪 = **大寰 AG-95**（Modbus RTU / RS485），是**独立设备**，不在 JTC 关节里。
- 前置：手眼标定（外参）、AG-95 寄存器映射现场核实、抓取位姿/TCP 偏置按 AG-95 重新推导。

**AG-95 的第一步是只读探测**（在写路径可用之前，先用它把接线/从站号/波特率确认下来）。
`scripts/ag95_probe.py` **一个字节都不写**，且这条由自检**扫源码**钉住：

```bash
python3 scripts/ag95_probe.py --frames                    # 不接硬件：看要读哪些寄存器
python3 scripts/ag95_probe.py --list-ports                # 适配器被认出来了吗
python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --scan-slaves 1 16   # 从站号是几
python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --slave 1 --dump    # 读全部候选寄存器
python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --slave 1 --watch   # 用手掰手指，定千分位方向
```

`--watch` 是现场**判方向**的那一条：把手指**用手**掰到全开，看 `0x0202` 读到 1000 还是 0
—— 这就是 `gripper.py` 里 `open_is_high` 那个必填参数的来源，全程一个字节都没写。
⚠️ 2026-09-27 在板子上实测过 `--frames` / `--list-ports`（能跑，且如实报「没有发现任何串口」，
与 `lsusb` 无 USB-串口芯片一致）；**`--scan-slaves` / `--dump` / `--watch` 还没有真设备可验**。

详见 [`docs/MIGRATION_HANDOFF.md`](docs/MIGRATION_HANDOFF.md) 与
[`docs/RESTRUCTURE_PLAN.md`](docs/RESTRUCTURE_PLAN.md)（P4/P5 组）。

---

## 参考资料

- [改造计划与缺陷台账](docs/RESTRUCTURE_PLAN.md) ← **先读这个**
- [技术方案文档](docs/TECHNICAL_SPEC.md)（设计稿，注意顶部前提对照）
- [方阵环境与训练流水线](docs/GRID_ENV_DESIGN.md)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [MoveIt2 文档](https://moveit.picknik.ai/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
