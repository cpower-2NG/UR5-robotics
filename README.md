# ur5-vision-grasp — UR5 视觉感知抓取（YOLO + ROS 2）

> YOLOv8 检测 → 2D→3D 投影 → MoveIt2 逆解 → UR5 抓取，**仿真闭环已全量跑通**。
> 部署目标：**NVIDIA Orin（JetPack 6.2 / aarch64）接真机 UR5**。
>
> 本仓是**公开**副本。硬件序列号、内网地址、用户名等身份信息已从全部历史中
> 剔除，并且由 [`scripts/test_deidentified.py`](scripts/test_deidentified.py)
> 把守 —— 提交前请跑它。相机序列号等**本机专有**的值放在仓库之外的
> `~/.config/ur5-vision-grasp/`，见下文「相机序列号」。

> 📖 **接手本项目 / 让 AI 改造本项目，先读 [`AGENTS.md`](AGENTS.md)。**
> 那里是这个项目的**核心约定**：目标与范围、红线、跨阶段不变的结构性约定、
> 阶段梯子、以及「四层文档各以什么时钟维护」。
> README（本文件）是**公开仓的门面**，讲「这仓库里有什么」；
> **项目怎么约定、下一步做什么，以 `AGENTS.md` 为准。**

---

## 现在能跑到什么程度

**仿真**：9/9 方块全自动搬运成功，经 Gazebo 物理真值核对（不是关节回读）。

**真机**：✅ **2026-10-05 通过 A1 验收** —— 连续抓放 **4/4**，且收工走的是
「放置槽位已用尽」这条正常路径（判分见 [`scripts/board_a1.py`](scripts/board_a1.py)）。

**尚未证明的三件事**（别读成「全都行了」）：

- **检测框在真机数据上准不准** —— 仿真里 9/9 检出、分数 0.68~0.94，真机数据**未量过**。
- **真机相机外参的长期稳定性** —— 手眼标定会**无声过期**（实测一天漂 0.945°）。
- **夹爪写路径** —— 代码侧就绪，`ag95_dry_run` 默认只读；解锁要求「拆下臂 + 工作台 +
  急停在手边」（见 [`AGENTS.md`](AGENTS.md) §2）。

> ⚠️ **本节的数字不在这里维护。** 逐项的**当前值、核对命令与保质期**见
> [`docs/当前状态.md`](docs/当前状态.md)——状态写进 README 就会烂，本项目已经
> 付过一次代价（一份维护得最认真的参考文档，它的现状段三天全错）。

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

> 🚀 **要真机抓取（不是只验感知）→ 只照
> [`docs/待执行命令.md`](docs/待执行命令.md) 的「A1 · 真机『抓 4 放 4 + 槽位用尽收工』」一节。**
> 那一节是**唯一**被真机跑通过的完整序列（参数集、次序、预检四项、收工判据都在那儿；
> 2026-10-05 A1 验收 **4/4** 通过）。本节只回答「`camera_source` 这个枚举怎么选」，
> **不重复那套序列** —— 本项目曾经有五份文档各讲一套启动流程，代价是十三条互相矛盾的说法。

`camera_source` 决定**谁提供 `/camera/*` 话题**，也顺带决定起不起仿真：

| 值 | 起相机驱动？ | 起 Gazebo？ | 用途 |
|---|---|---|---|
| `sim`（默认） | — | ✅ | 仿真 |
| `realsense` | ✅ RealSense 驱动 | ❌ | 接真机 RealSense |
| `external` | ❌ | ❌ | 话题已由别处提供（rosbag / 自建 launch） |

```bash
# 真机 + RealSense，只验感知不抓取（先跑通这条再谈抓取）
# ⚠️ robot_ip 在真机入口是**必填**：`real.launch.py` 的启动前检查（`_check`）
#    会直接抛错 ⇒ 不给 IP 的后果是「命令失败、什么都没起」，
#    **不是**「安静地跳过驱动」（那是很容易想反的一侧）。
export ROS_DOMAIN_ID=99   # ⚠️ 板上现场跑在 99（`~/.bashrc` 里有，但新开的 shell 里别指望）
ros2 launch yolo_bringup bringup.launch.py \
    camera_source:=realsense robot_ip:=<控制箱IP> \
    skip_planner:=true tf_required:=true
```

> ⚠️ **`ROS_DOMAIN_ID=99` 漏了不会报错**，症状是「`ros2 topic list` 空的」，
> 极易误判成「栈没起来」——那是**查询侧**的问题，不是板子的。2026-10-05 真机踩过。

`tf_required:=true` 是**真机建议值**：查不到 TF 就让 `perception_3d` 报错退出，
而不是发一帧坐标可疑的数据下去——仿真里默认 `false`（TF 树起得慢，早几帧查不到是正常的）。

> ⚠️ **这个开关有 3 个定义处，「默认值」不是一份事实。**
> `bringup.launch.py` 按 `camera_source` **算**（sim→`false`、其余→`true`）、
> `real.launch.py` 写死 `true`、`pipeline.launch.py` 写死 `false`；而
> `launch_configurations` 是**整棵树共享**的、**先声明者胜** ⇒ 直接调
> `pipeline.launch.py camera_source:=external` 时默认拿到的是 **`false`**，
> 与「真机建议 true」正好相反。只有走 `bringup.launch.py` 的真机那两支是 `true`。
> 三处并存是本项目的已知结构债（`scripts/test_real_launch.py` 的 D64 一条把守着
> 其中的一半）。**别在其他文档里再写第 4 处默认值**。

⚠️ **`gripper_backend` 的默认值随入口变** —— 它在代码里**不是写死的常量**，
而是 `'sim' if camera_source == 'sim' else 'none'`（`pipeline.launch.py`）：

| 入口 | `gripper_backend` 默认 | 要额外传吗 |
|---|---|---|
| `camera_source:=sim` | **`sim`**（JTC 第 7 关节） | 不用 |
| 其余入口（含 `realsense` / `external`） | **`none`**（只走位，不报错） | 不用 |

（上表是 `camera_source` 那套入口。单独起 `grasp_planner/planner.launch.py` 时
默认仍是 `sim` —— 它不走 `camera_source`，别把这行结论套过去。）

> **「看不到目标」先看这一条**：`perception_3d` 会把**检测↔深度时间差超窗**的批次整批丢掉
> （不报错退出），日志每 5 s 一条 `…丢弃本批检测（累计 N 批）`。真机默认窗口已经是 **0.3 s**
> （仿真 0.1 s），若 **N 持续增长**就调大：`max_sync_dt:=0.5`。调小它的症状恰好也是
> 「一直没目标」，而那时日志里**只有这一条线索**。

> **这条默认值是 2026-10 才改成现在这样的**（D71）。改之前非仿真入口默认 `ag95`，
> 代码注释给的理由是「后端一构造就去开串口，板上没适配器时**大声报错**」——
> **那个前提已经不成立**：适配器早在位、`pymodbus` 已装，`ag95` 会**安静地连上真夹爪**。
> 而 `planner_node` 在只走位模式（`carry_objects:=false`）下**仍会调用 `gripper.open()`**
> （初始化那一步 `_start_initial_home`）⇒ 真正要提防的不是「启动即死」，
> 是**「启动即写寄存器」**。所以默认值改成了 `none`。
>
> 要真夹取，**必须显式三件一起给**：
> `gripper_backend:=ag95 carry_objects:=true`，再另配 `ag95_dry_run:=false`。
> ⚠️ 写寄存器 = **真机动作**，仅限「夹爪已拆下臂 + 放工作台 + 急停在手边」，
> 见 [`AGENTS.md`](AGENTS.md) §2 红线 2。

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
ur5-vision-grasp/
├── AGENTS.md              # ★ 项目核心约定：红线 / 不变量 / 阶段梯子。**接手先读这一份**
├── CLAUDE.md              # 只有一行 @AGENTS.md（给 Claude Code 用的转发）
├── ros2_ws/src/
│   ├── yolo_detector/     # YOLOv8 检测；设备退化、权重路径解析
│   ├── perception_3d/     # 2D→3D 投影（唯一投影点）；深度尺度/时间戳配对/TF
│   ├── grasp_planner/     # 抓取规划；夹爪抽象层（sim / ag95）
│   ├── arm_grasp_sim/     # 仿真世界 + UR5 + 控制器 + MoveIt  ← 工作站专用
│   └── yolo_bringup/      # 唯一入口 bringup + pipeline + 方阵世界
├── scripts/               # 自检、训练、数据合成脚本（每个会自己报自己的项数）
└── docs/
    ├── 当前状态.md          # ★ 这台系统**现在**什么样 —— 唯一写现状的地方，带保质期
    ├── 文档地图.md          # ★ 全仓索引：分层、是否现行、**旧文件名别名**
    ├── 机械臂系统参考.md     # 恒久：设备与接口（只写不随任务变化的东西）
    ├── 经验总结-问题模式与处置.md  # 恒久：复发问题模式（跨阶段累积）
    ├── 待执行命令.md         # 要**你亲自执行**的命令 —— 只放「还没做的」
    ├── 待执行命令-留档.md     # 上面那份的全部历史（甲-1…）+ 每条背后的论证
    ├── RESTRUCTURE_PLAN.md   # 流水：缺陷台账 + 改造计划 + 验收证据（历史，全文保留）
    ├── 阶段/阶段1-点对点抓放/  # 已完成阶段的产出：契约 / 技术文档 / 实验记录
    ├── calib/                # 标定产物与标定方法（外参、TCP、探针长度）
    ├── board/                # 板端资产留档：6 个 arm-*.sh + 体检清单 + UR5 接入方案
    ├── archive/              # 已归档文档（正文一字未改，仅供追溯）
    ├── TECHNICAL_SPEC.md     # 设计稿（含环境对照表，注意其前提已过时）
    ├── JETSON_SSH.md         # 板子 SSH 连接方式（含字段说明）
    ├── PROBE_LENGTH.md       # 探针长度标定
    └── GRID_ENV_DESIGN.md, YOLO_*.md   # 仿真/YOLO 训练流水线，**与本仓主线关系待定**
```

> **哪份讲哪一层、搬过家的叫什么** → [`docs/文档地图.md`](docs/文档地图.md)。
> 那份地图由 `scripts/test_docs_map.py` 把守：每份文档恰好登记一次、自报层级一致、链接不断。

---

## 自检与验收

这些脚本**层层递进**，都不需要 Gazebo（`test_ag95_probe.py`、`test_ag95_gripper.py`、
`test_ag95_init_close.py` 与 `test_real_launch.py` 连真夹爪、真串口都不需要 ——
前三个用一个手写的 Modbus 从站接在 pty 上验；`test_ur5_fk_check.py`、
`test_board_a1.py`、`test_board_lib.py`、`test_board_gate.py`、`test_board_run.py`、
`test_board_tree_manifest.py`、`test_deidentified.py`、`test_docs_map.py`、
`test_startup_docs.py` 与 `state_check.py` 只要 Python 标准库）：

```bash
source /opt/ros/humble/setup.bash
# test_minimal 要 import ultralytics，所以必须叠加 conda 环境（其余不需要，
# 但叠了一起跑没问题）。不加这一行，test_minimal 会报「ultralytics 未安装」，
# 看起来像项目坏了 —— 其实只是解释器不对。
export PYTHONPATH=~/robotics/venvs/robot-vision/lib/python3.10/site-packages:$PYTHONPATH

python3 scripts/test_minimal.py          # 冒烟：模型加载 + 投影数学
python3 scripts/test_perception_3d.py    # 感知单元
python3 scripts/test_detector_device.py  # 设备退化分支
python3 scripts/test_real_launch.py      # launch 参数面 + 收工卫生 + 相机序列号守卫
python3 scripts/test_model_resolution.py # 加载的是哪个模型（D67）
python3 scripts/test_ag95_probe.py       # AG-95 只读探针
python3 scripts/test_ag95_gripper.py     # AG-95 驱动（读 / dry-run 闸门 / 写）
python3 scripts/test_ag95_init_close.py  # AG-95 初始化/合拢脚本
                                         #   重点只有一条：**不给 --go 时从站一次写都没收到**
python3 scripts/test_ur5_fk_check.py     # UR5 正运动学 + 「示教器 Base 是不是机器人 base」
                                         #   的判决工具（含变异测试）
                                         #   只要 numpy，不需要 ROS / 真机 / 示教器
python3 scripts/test_board_a1.py         # A1 真机验收的**判分逻辑 + 起栈命令**（含变异测试）
                                         #   纯离线，拿真机日志原文当夹具，不碰 ROS / 臂 / 串口
                                         #   判分错了的代价不是「测试红一下」，而是误判整轮
                                         #   真机实验 —— 所以这里连判据本身也要测
python3 scripts/test_board_lib.py        # 站点配置 / 外参时效 / 残留进程的判定（含变异检查）
                                         #   纯离线；「顶层不 import ROS」本身是一条断言
python3 scripts/test_board_gate.py       # 闸门会不会放行坏的东西（含变异检查）
                                         #   测的是判定吃不吃劲，不是「板子现在通不通」
python3 scripts/test_board_run.py        # 启动器的判定与参数拼装：拼错的参数拦在起栈之前
                                         #   板端那两档用假 ros2，不替人做真机动作的主
python3 scripts/test_board_tree_manifest.py  # 全树哈希：跳过规则 + 「多 / 少 / 改」三分类
                                         #   半截清单不算一致（fail-closed）
python3 scripts/test_deidentified.py     # 公开仓脱敏把关：0 = 干净，1 = 有命中
                                         #   扫**全部被跟踪的文本文件**、**未跟踪（未忽略）的文件**
                                         #   与**全部提交信息**
                                         #   （后两者都不是多余的：--replace-text 只改 blob、
                                         #   不改提交信息；未跟踪的文件在 git add 之前对
                                         #   ls-files 是隐形的 —— 两条路径都真的漏过）
                                         #   只要标准库；**不需要** source ROS
python3 scripts/test_docs_map.py         # 文档地图全覆盖 + 层声明一致 + 相对链接不断
python3 scripts/test_startup_docs.py     # 真机启动文档守卫：runbook（`待执行命令.md` 的 A1
                                         #   一节）里写的 launch 参数集，必须 == 启动器
                                         #   `scripts/board_a1.py` 实际会传的那一套
                                         #   内含真实的变异反例（参数少一个/多一个、
                                         #   入口换成 pipeline.launch.py，都必须判红）
                                         #   只要标准库；**不需要** source ROS
python3 scripts/state_check.py           # 状态层过期检查（把「哪些行超期」算出来）
```

> ⚠️ **每个脚本会自己报自己的项数 —— 这里故意不写数字。**
> 断言数随开发一直在涨（本仓实测过同一脚本从 24 涨到 81 项），把数字抄进文档
> 就等于给自己埋一个必然过期的东西。想知道跑了多少项，看它自己打印的输出。
>
> ⚠️ **几个脚本缺依赖时的行为不一样，别只看退出码**：
> - `test_ag95_probe.py`：只有第 2 段要 pymodbus，所以缺了会**降级**跑其余部分，
>   报 `26 PASS / 6 SKIP` 且**退出码 0** —— 这次运行覆盖不全，不能当「验过了」。
> - `test_ag95_gripper.py`：每一项都走 pymodbus 的串口客户端，**没有可降级的部分**，
>   所以缺了直接**退出码 2**（按 `test_real_launch.py` / `test_model_resolution.py`
>   的先例）。它**不假装跑了一部分**。
> - `test_ag95_init_close.py`：只有第 1 段（扫源码）能离线跑，其余要 pymodbus，
>   缺了报 `1 PASS / 10 SKIP` 且**退出码 0** —— 同样**不能当「验过了」**。
>   而它唯一的实质断言（「预演不写寄存器」）恰好就落在 SKIP 的那一段里。
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

**A1 真机验收**（**在板子上跑**）—— 自己起 `carry_objects:=true` 的整栈、
盯日志、按判据判分。**它会真的让臂动起来**：

```bash
export UR_ROBOT_IP=<控制箱IP>          # 公开仓不写现场 IP，所以没有默认值
python3 scripts/board_a1.py --params $HOME/site_real.yaml
#   判据 1：出现 `已放置 4/4 个`
#   判据 2：收工原因是 `放置槽位已用尽（4 个）—— 没有地方可放了`
#   前置检查：无残留进程 / 串口空闲 / AG-95 已初始化 / 示教器在 PLAYING
#   `--no-launch --log <文件>` 只对已有日志判分，不起栈、不动臂
#   `--start-delay <秒>`（默认 25）规划器推迟启动，等 UR 驱动先连上
#     ⚠️ A1 一节手敲的那两段用的 **120**（实测「一次 State tolerances failed
#     都没有」）。25 是够按 PLAY 的保守下限，120 是建议值 —— 见该节的说明
```

**2026-10-05 真机通过**（5 块 / 4 槽）：

```
候选 5 个 | 已放置 0/4 | 连续未放置 0/10
已放置 1/4 → 2/4 → 3/4 → 4/4            （每循环约 23 s）
候选 1 个 ['block'] | 已放置 4/4 | 已抓 4 个
收工：放置槽位已用尽（4 个）—— 没有地方可放了
  已成功放置 4 个 / 槽位共 4 个；已抓取记账 4 个
```

候选数轨迹 `5(0) → 4(1) → 3(2) → 2(3) → 1(4)`；夹爪回读
25.08 / 25.46 / 26.13 mm（全部 ≫ 0 = 真夹住）；`连续未放置 0/10`。

⚠️ **判据 2 需要抓取区里摆 5 块（方块数 > 槽位数）**，见
[`scripts/board_a1.py`](scripts/board_a1.py) 文件头的推导 —— 4 块配 4 槽时
第 4 块放下的同一刻区里也空了，收工走的是空闲看门狗那条路。

⚠️ **起栈时规划器要晚于 UR 驱动**：`start_delay` 包的是
`[realsense, detector, perception, planner]`、**不含 UR 驱动**，所以
`--start-delay` 的语义正是「驱动先起 → 规划器后起」。不加时规划器的启动 HOME
会撞上 `ur_ros2_control_node` 的状态容差
（`State tolerances failed for joint 5`、`error_code=-4`），实测开局即
`连续未放置 2/10`；加了之后一次都没有。

⚠️ 两个要点：① 它用**占位**相机外参（相机在上方 1 m 朝下看），只为接通 TF 树，
所以 `/targets_3d` 的**数值不可信**，它断言的是链路通不通、跑多快；真外参等手眼标定。
② `planner_node` 会报「轨迹 action 服务不可用」并停住 —— **预期**，没有驱动就没有
控制器。**本脚本不覆盖运动**，那一步要真臂或仿真器。

⚠️ **要在板子上跑仿真，先看 `docs/待执行命令-留档.md` 甲-14。** arm64 上**没有**
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

### 入口脚本速查 —— 哪个命令回答哪个问题

**这一节是「哪条命令回答哪个问题」的唯一索引。** 范围是**人要起起来或挂上去**的入口
脚本；上面那批 `test_*.py` 自检**不在表里**（它们是上一段的那份清单）。

每一行都写清它**动不动东西**（起栈 / 清进程 / 写硬件 / 只读）与**退出码**。本仓的常规
是 `0/1/2`（`2` = 环境或前置不满足，fail-closed）——偏离的只有三处，表里逐个标了出来。

表的读法：**第一格 = 脚本名 + 它回答的问题**；最后一格说明它是 `board_run` 的哪一段
引擎、还是一个与它无关的**独立**入口。

| 脚本 —— 它回答的问题 | 何时跑 | 动不动东西（含退出码） | 与 `board_run` 的关系 |
|---|---|---|---|
| **`scripts/board_run.py`** —— 这一轮能不能起、起完算不算过（任务无关的统一入口，读 `config/tasks/<名>.yaml`） | 真机跑一整轮 | **起栈 + 收栈**（跑完 `teardown` 自己的进程组；`--stop` 收掉**别人**起的栈）；起栈前发现残留则**拒绝起栈**，不替你清。默认只读，写路径只在 `--repair-controller`（要交互终端 + 当次 `--authorize-motion`）之后。**0/1/2**：`0` 只有**真跑完一轮**才拿得到，离线整跑必然 `2` | 本表的**编排者**：P1 调 `check_board_deploy` / `board_tree_manifest`，P3/P4 调 `board_gate`，P5 调 `board_a1:grade`。判定一行都不自己实现 |
| **`scripts/board_gate.py`** —— 已经起着的那套栈健康吗（控制器 / 相机 / 目标 / 检测器 / 遥测 / 参数） | 栈起着、还没交棒 | **不起栈、不收栈**；默认一个字节不写，写路径只在 `--repair-controller`（交互终端敲 `yes`）后面。**0/1/2**（有 SKIP 就不算通过） | **就是 `board_run` 的 P3/P4 引擎** —— 子进程调用 + 读它的 JSON 记录，判定不重抄 |
| **`scripts/board_a1.py`** —— 阶段 1 的出口判据过没过（`已放置 4/4 个` + 收工原因是「放置槽位已用尽」） | 跑 A1 验收；`--no-launch --log <文件>` 只对已有日志判分（那一档不动臂） | **自己起 `carry_objects:=true` 的整栈（臂会真的动）**，跑完收栈。**0/1/2**（`2` = 前置没过、不起栈） | **前身 / 第二个启动器** —— 作为启动器已被 `board_run --task a1` 取代；但它的 `grade()` 是 `board_run` 的 P5 判分**本体**，它本身又是阶段 1 契约点名的出口判据 ⇒ **不废**（见注三） |
| **`scripts/check_board_deploy.py`** —— 板上缺什么（从 `package.xml` 与源码 `import` **推导**，不抄清单） | 部署完、起栈前 | **只读**：不装包、不写文件、不起节点、不要 sudo。**0/1/2** | **`board_run` 的 P1「部署前置」引擎**（子进程） |
| **`scripts/board_tree_manifest.py`** —— 板上那份代码树和本机是不是同一份（全树哈希，不抽查） | 同步到板上前后（本机 `--write` 立清单，那边 `--check` 对） | `--write` 只写**仓库外**的清单文件；`--check` 纯只读；不碰栈。**0/1/2**（`2` = 清单不在 ⇒ fail-closed） | **`board_run` 的 P1「板子树」引擎**（子进程） |
| **`scripts/board_lib.py`**（CLI 模式）—— 现在生效的站点配置是哪一份、合不合法、外参过没过期 | 配站点时；怀疑配置没生效时 | CLI **只读**（打印）。**0/1/2**（库里那个 `teardown()` 会 killpg，是给调用方用的，CLI 不走它） | **`board_run` 的前置层** —— 被 `import`（不是子进程）：站点配置 / 残留进程 / 串口 / 夹爪 / 控制柜只读查询只有这一份 |
| **`scripts/state_check.py`** —— `docs/当前状态.md` 里哪些行已经超期 | 提交前；引用状态层结论之前 | **纯离线只读**（不需要 ROS）。**0/1/2** | **独立**（文档层守卫） |
| **`scripts/real_chain_check.py`** —— 已经在跑的那套图，服务 / TF / IK / FK 通不通（`/compute_ik` 与 `/compute_fk`、`/joint_states`、`base_link→tool0` 逐跳 TF、**真解一次 IK 再用 FK 读回来**） | 栈起着、要验几何链路 | **只读；不起任何节点**；结构上不可能动臂（不建 action client）。**只有 0/2**（没有 `1` —— 没通过一律记「环境不满足」） | **独立**；与 `board_gate` / `no_arm_stack --check` 是**同一问题的三个深度**（见注一） |
| **`scripts/board_chain_check.py`** —— 这块板子扛不扛得住真机那条链路（真相机 → 检测 → 三维目标的逐级吞吐） | 臂还没接、要验板子余量 | **自己起**无驱动真机入口 + RSP + 假关节角，跑完收栈；没有 `controller_manager` ⇒ 物理上不可能动臂。**0/1/2** | **独立** |
| **`scripts/moveit_only_check.py`** —— MoveIt 链路通不通 + 肘 / 大臂余量（**复合问题**：加 `--envelope-z` 时**再扫一整片桌面**画「撞桌 / 够不到」的边界，给现场划区用） | 臂没接线时的预检；要划区才加 `--envelope-z` | **自己起** `real.launch.py start_ur_driver:=false` + RSP + 假关节角，跑完收栈；零运动。**0/1/2**（包线自己的结论不进退出码：图上有撞桌格是结论不是失败） | **独立**（见注四） |
| **`scripts/no_arm_stack.py`** —— **三种模式 = 三个问题**：不带开关「起一套只读的无臂栈并留在前台」；`--stop`「清干净上一份」；`--check`「现在这套栈能不能解 IK」（不起任何东西） | 要跑探针 / 包线之前先起它 | 起栈（无驱动 + 假关节角）或清场；`--check` 纯只读。**只有 0/2** | **独立**；`--check` 与 `real_chain_check` 是同一问题的不同深度（见注一、注二） |
| **`scripts/board_state_machine_check.py`** —— 规划器的状态机整条走不走得完（决策链、IK、肘高护栏、槽位记账、终止与收工路径、遥测） | 没有硬件但要验「运动」那一段 | **自己起**无驱动入口 + RSP + **本进程里的假臂**（假 `FollowJointTrajectory` action + 会跟着走的 `/joint_states`）；那个 action 是桩 ⇒ 零运动；不碰串口 / RTDE / 急停。**0/1/2** | **独立** |
| **`scripts/board_ctrlc_check.py`** —— Ctrl-C 收工干不干净 | 在板子上；动过收工 / 信号相关代码之后 | **真起一个不含机械臂的 pipeline**，跑一小段后用 pty 写一次**真** Ctrl-C（默认 25 秒，`--seconds` 可改；`timeout` 与 `命令 &` 都会给假结论）。**0/1/2** | **独立** |
| **`scripts/board_bag_record.py`** —— 把板上的画面 / 数据搬回开发机看 | 板上的链路已由别的方式起着 | **只录、不起链路**；写 bag 文件。**0/2/3**（多一个 `3` = 录完了但关键话题一条消息都没有） | **独立** |
| **`scripts/topcam_drift_check.py`** —— 俯视外参还作不作数（相隔很久的两帧比全幅光流，分开「靶纸被碰」与「相机被碰」） | 每次要用俯视外参之前 | **纯离线比图**；不碰 ROS、不碰硬件。**0/1/2**（`1` = 外参作废，重跑 ⓪①） | **独立** |

#### 注一 · 三件「已经跑着的栈能不能用」是一个问题的三个深度

表里那三行（`real_chain_check` / `no_arm_stack --check` / `board_gate`）单看一句
「看栈通不通」是分不开的。按**深度**读：

| 深度 | 命令 | 问到哪一层 |
|---|---|---|
| 最浅 | `scripts/no_arm_stack.py --check` | 只问 `/compute_ik` 与 `/compute_fk` **在不在** |
| 中 | `scripts/real_chain_check.py` | 上面那两个 + `/joint_states` + `base_link→tool0` 逐跳 TF + **真解一次 IK 再用 FK 读回来** |
| 最广 | `scripts/board_gate.py` | 控制器 / 相机 / 目标 / 检测器 / 遥测 / 参数 —— 整条**运行态** |

⚠️ **但最广的那个并不包含中间那个**：`board_gate` 的那几项里**没有** `/compute_ik`、
也没有 TF ⇒ `real_chain_check` **不是**它的子集，两个都得能跑。
反过来，`no_arm_stack --check` 是 `real_chain_check` 的**真子集**（少 TF、少 IK/FK 往返）。

⇒ 口诀：「IK 解不解得出来」问 `real_chain_check`；「这一轮起栈健康吗」问 `board_gate`；
「手上有没有一个能用的栈」问 `no_arm_stack --check`（最省，也最容易看漏）。

#### 注二 · `no_arm_stack.py` 一行写不下

它是本表唯一一个「一行一个问题」装不下的脚本：`--check` 是**问**，不带开关是**起**，
`--stop` 是**清**。

另外，`--check` 报「另有 N 个进程在跑」时**不替你判**那是「上一份栈」还是「没人管的
残留」—— 两者的处置正好相反（前者别动、后者要清），所以它只报事实。这不是漏写，
脚本自己的注释里写明了这条。

#### 注三 · A1 那个问题由**两条**命令回答

别靠「哪个名字短」选：

| 你要的 | 用哪条 |
|---|---|
| 起栈 + 判分（完整一轮） | **`board_run.py --task a1`** —— 参数集来自 `config/tasks/a1.yaml`，不再手抄；闸门 / 授权 / 判分是一条链 |
| 只判分、不动臂 | **`board_a1.py --no-launch --log <文件>`** —— `board_run` 没有这一档 |

`board_a1.py` 是 `board_run` 在 A1 上的**前身**（同一套参数集写死在它自己的
`build_cmd()` 里）。但**它不是废件**，两条理由：

1. `config/tasks/a1.yaml` 的 `score.callable` 就是 `board_a1:grade` —— 判分**本体**在
   它那儿；删了它，`board_run` 的 P5 就没有判分了。
2. 阶段 1 的契约（[`docs/阶段/阶段1-点对点抓放/契约.md`](docs/阶段/阶段1-点对点抓放/契约.md)）
   点名的**出口判据**就是它。

⇒ **留着，别「顺手合并掉」。**

#### 注四 · `moveit_only_check.py` 一行里塞了两个问题

前半句（MoveIt 链路通不通 + 肘 / 大臂余量）是默认行为；`--envelope-z` 加的
**桌面包线扫描**（「哪些 (x, y) 会把大臂压到台面下」）是另一个问题，给现场**划区**用。

⚠️ 包线扫的范围比任何合理的区都大 ⇒ **它自己发现「图上有撞桌格」不算失败**（不进
退出码）；只有「一个位姿都没量出来」才退 `2`。别看前半句就以为它只验链路。

#### 退出码为什么有三处不一样（都是**故意**的，别去「统一」）

- `real_chain_check.py` 与 `no_arm_stack.py` **只有 0/2，没有 1** —— 它们只回答
  「现在能不能用」，不区分「看到了、它不对」与「我没看成」，两者一律记「环境不满足」。
  这是自洽的，且调用方按这个判读。
- `board_bag_record.py` 多一个 **3** = 录完了但关键话题一条消息都没有 ——「录了」
  ≠「录到了」。
- 其余一律 **0/1/2**。

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
- ✅ **检测这一半已在板上跑通**（2026-09-28 实测）：权重 `block_best.pt`
  **不在本仓**（6 MB，属构建产物，按规范走外链；sha256 `a926d959…` 可用于核对
  你手上那份是不是同一个）；板上 `torch 2.5.0a0+nv24.08` + `ultralytics 8.4.26`
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

  # 真机链路起来（需要控制箱 IP；臂没接上/没配好时先别跑这条）
  ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense robot_ip:=<控制箱 IP>
  ```

  > **上面两条只是「起得来」。**
  > **完整抓取的参数集、次序（起栈 → 示教器按 PLAY → 拉控制器）、预检四项、
  > 摆件要求与收工判据，一律以
  > [`docs/待执行命令.md`](docs/待执行命令.md) 的 A1 一节为准** ——
  > 本节讲的是「板上装了什么、缺什么、怎么编」，不重复那套序列。
  >
  > **「哪条命令回答哪个问题」（入口脚本速查）在上一节末尾**，本节不重复那张表 ——
  > 那份表把每个入口脚本的「动不动东西 / 退出码 / 与 `board_run` 的关系」列在一处。
- **模型权重必须显式指定，别依赖「按目录找」**（D67）。⚠️ `block_best.pt`
  **不在仓库里**（6 MB，按规范走外链；需要它请见下面「模型权重」一节）。
  `model:=block_best.pt` 这个**相对名**是按**启动时的 CWD** 解析的 ——
  从放权重的那一层启动才找得到。推荐写法是
  `export YOLO_MODEL_DIR=<放权重的目录>`（或直接传绝对路径），这样与 CWD 无关。
  包内**刻意不放任何权重**：找不到时会抛出可操作的 `FileNotFoundError`，
  而不是静默换一个别的模型。

### 模型权重（不在本仓）

`block_best.pt`（单类 `block` 检测器）是**构建产物**，按仓库约定不入库
（`.gitignore` 里 `ros2_ws/src/yolo_detector/models/*.pt` 同理）。取到它之后：

```bash
mkdir -p ~/models && cp block_best.pt ~/models/
export YOLO_MODEL_DIR=~/models          # 或把绝对路径直接传给 model:=
sha256sum ~/models/block_best.pt        # 应为 a926d959…（与本文档记录一致）
```

> 外链地址：**待补** —— 发布前请把权重放到一个可公开访问的位置并在此写明。
> 校验和 `a926d959…` 是这份权重的身份，换文件前请先比对。

### 相机序列号（本机专有，不入库）

真机要钉死用**哪一台** D435i，靠的是 `serial_no`；而序列号是**设备身份**，
和 IP、用户名同类，所以公开仓里只有**模板**
（[`config/realsense_camera.yaml`](ros2_ws/src/yolo_bringup/config/realsense_camera.yaml)），
序列号位是占位符。用之前先建一份自己的：

```bash
mkdir -p ~/.config/ur5-vision-grasp
cp ros2_ws/src/yolo_bringup/config/realsense_camera.yaml \
   ~/.config/ur5-vision-grasp/realsense_camera.yaml
# 编辑它，把 serial_no 换成你这台相机**外壳标签**上的号（引号要留着）
```

`pipeline.launch.py` 的解析顺序是
`UR5_REALSENSE_PARAMS` 环境变量 → 上面那个本机文件 → 包内模板。

> ⚠️ **忘了填不会被无视，会被拦下**：起 `camera_source:=realsense` 时若序列号仍是
> 占位符，守卫会**当场抛错**并给出填法。放行的话症状是**静默**的 —— 驱动每 6 秒
> 重试一次、永不退出，「进程都在、就是没有 `/camera` 图像」。
> 为什么序列号不能像别的参数那样在 launch 里传，见那份 yaml 的文件头
> （一句话：纯数字串在两侧被判成了不同类型，**没有中间态**）。
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

详见 [`docs/archive/MIGRATION_HANDOFF.md`](docs/archive/MIGRATION_HANDOFF.md) 与
[`docs/RESTRUCTURE_PLAN.md`](docs/RESTRUCTURE_PLAN.md)（P4/P5 组）。

### 站点配置与任务描述（本机专有，不入库）

真机那一轮要的东西分两份，**都在仓库外**：

- **站点配置** —— 控制柜 IP、夹爪串口与波特率、俯视外参、规划器参数文件路径，
  外加两个**强制的**外参时效字段（`verified` + `ttl_days`）。模板是
  [`config/site.example.yaml`](config/site.example.yaml)。
- **任务描述** —— 起哪个入口、`camera_source`、`start_delay`、轨迹 action、
  判分函数（`score.callable`），在 [`config/tasks/`](config/tasks/) 下（如 `a1.yaml`）。
  启动器 `board_run.py` **一行代码都不认识 A1**：换场景 = 加一份同形状的 yaml。

```bash
mkdir -p ~/.config/ur5-vision-grasp
cp config/site.example.yaml ~/.config/ur5-vision-grasp/site.yaml
$EDITOR ~/.config/ur5-vision-grasp/site.yaml
python3 scripts/board_lib.py     # 自查：生效的是哪份路径、逐条合法性、外参过没过期
```

⚠️ **模板里的取值故意留成占位符** —— 占位符过不了 `float()` / 日期解析，于是配置不
合法时**大声报错**，而不是拿着一份看起来像真的配置静默跑成另一回事。这份「故意不合法」
本身由 `scripts/test_board_lib.py` 钉住。

仓内那两份任务描述（`a1.yaml` / `a2.yaml`）也是**模板**：现场要改的话，拷到
`~/.config/ur5-vision-grasp/tasks/` 改，**同名时现场那份优先**。

---

## 参考资料

- [改造计划与缺陷台账](docs/RESTRUCTURE_PLAN.md) ← **先读这个**
- [技术方案文档](docs/TECHNICAL_SPEC.md)（设计稿，注意顶部前提对照）
- [方阵环境与训练流水线](docs/GRID_ENV_DESIGN.md)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)
- [MoveIt2 文档](https://moveit.picknik.ai/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)

---

## 许可

[MIT](LICENSE)。

本仓是内部工程仓的公开副本，**不含**任何第三方教材/数据集原文，也不含模型权重
（权重按上文「模型权重」走外链）。引用到的 RealSense / UR / MoveIt / Gazebo 等
均为各自所有者的商标，此处仅作指代。
