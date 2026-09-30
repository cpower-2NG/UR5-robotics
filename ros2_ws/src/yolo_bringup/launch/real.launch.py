"""真机启动器：UR 驱动 + MoveIt + 相机外参 TF + 感知/抓取。

用法
====
  # 整套（第一次真机联调；robot_ip **必填**）
  ros2 launch yolo_bringup real.launch.py robot_ip:=<ip>

  # 分阶段验收（W5 的第 4 步「只读验证」就是这么做的）：
  ros2 launch yolo_bringup real.launch.py robot_ip:=<ip> start_moveit:=false \
      start_pipeline:=false                    # ① 只起驱动：看 /joint_states 与控制器
  ros2 launch yolo_bringup real.launch.py robot_ip:=<ip> start_ur_driver:=false \
      start_pipeline:=false                    # ② 只起 MoveIt：看 /compute_ik
  ros2 launch yolo_bringup real.launch.py robot_ip:=<ip> \
      start_ur_driver:=false start_moveit:=false  # ③ 只起相机 TF + 感知

经由唯一入口启动（本文件由 `bringup.launch.py camera_source:=realsense` include）：
  ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense robot_ip:=<ip>

它补齐的四件事（真机链路原先一件都没有，见 D45）
================================================
  1. `ur_robot_driver/ur_control.launch.py` —— 驱动 + 控制器（含 JTC action）
  2. `ur_moveit_config/ur_moveit.launch.py` —— `move_group`（`/compute_ik` 的来源）
  3. `base_link → camera_link` 静态 TF —— 手眼标定的落地处
  4. `pipeline.launch.py`（`camera_source:=realsense`）

⚠️ 机械臂动作、控制器切换、真机抓取属于**高风险操作**：未经当次明确授权且
急停就绪，不要启动真实运动。本文件只做「起节点」这件事，不改任何安全设置，
所有保护（`safety_limits`、示教器速度滑条、急停）都在机器人侧。

⚠️ 第一个真机 demo **不含抓取**，所以本文件的默认值就是那个组合：
`gripper_backend:=none` + `carry_objects:=false`（只走位）。
要真夹取，AG-95 的**代码侧已经就绪**（`ag95` 后端的读/写/到位轮询都在
`grasp_planner/gripper.py`，写路径由 `ag95_dry_run` 卡着，默认只读）——
挡着的**只有硬件**：板上还没有 USB-RS485 适配器，`_connect()` 打不开串口。
硬件到位后按 `docs/待执行命令-留档.md` 的 **丙-1**（判决方向 → 离臂试写 → 上臂）逐步走。
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 官方 launch 里机器人型号的合法取值（ur_control.launch.py 的 choices）。
# 这里不复制那份清单，只做「非空」校验 —— 型号写错由驱动的 choices 报错，
# 比在本文件里维护第二份清单可靠。
_NO_DEFAULT = ""

# ⚠️ 初始控制器为什么显式写成 scaled_（而它本来就是驱动的默认值）
# =============================================================
# `ur_control.launch.py` 的 `initial_joint_controller` **默认就是**
# `scaled_joint_trajectory_controller`（板端实测，见 RESTRUCTURE_PLAN 的 W2 记录）。
# 但仍然**显式传一遍**：这个选择关系到「第一次真机动作有多快」，不该依赖
# 上游默认值 —— 上游改一次默认，本文件的意图就悄悄变了。
#
# 它相对 `joint_trajectory_controller` 的差别是**尊重示教器速度滑条**：
# 现场把滑条调慢，臂就跟着慢。对第一次真机动作是明显更安全的默认。
#
# 配套约束（规划器侧已经处理，这里记着免得回退）：
# 规划器的 `time_from_start` 是它自己拍的死值（move_duration / descend_duration），
# 而 `goal_time_tolerance` 曾经是 0（= **最严**：一到点就必须已在容差内，
# 没有任何宽限）。滑条一慢，每条轨迹都可能被判 GOAL_TOLERANCE_VIOLATED。
# 现已在 grasp_planner.yaml 里改成 2.0 s（D55）。
INITIAL_JOINT_CONTROLLER = "scaled_joint_trajectory_controller"


def _package_launch(pkg: str, filename: str, *, why: str) -> str:
    """取另一个包的 launch 文件绝对路径；包不在就抛出**可操作**的报错。

    ⚠️ 必须在**执行期**调用（放在 OpaqueFunction 里），不能写在模块顶层或
    IncludeLaunchDescription 的构造里：launch 在**解析期**就会求值
    `get_package_share_directory`，而 IfCondition 只影响**执行**。
    写成构造期求值的话，「本机没装 ur_robot_driver」会让
    `ros2 launch ... real.launch.py --show-args` 也整体失败 —— 连用法都看不到。
    这与 `bringup.launch.py::_sim_backend` 是同一个坑、同一套解法。
    """
    try:
        return os.path.join(
            get_package_share_directory(pkg), "launch", filename
        )
    except PackageNotFoundError:
        raise RuntimeError(
            f"真机链路需要 {pkg} 包，但当前环境里找不到它（{why}）。\n"
            f"  它属于 ROS 2 官方 UR 支持，不在本仓库里：\n"
            f"      sudo apt install ros-humble-{pkg.replace('_', '-')}\n"
            f"  装完重新 source /opt/ros/humble/setup.bash 再启动。\n"
            f"  Orin 上这四个包（ur_robot_driver / ur_moveit_config / "
            f"ur_description / realsense2_camera）**已经装好**，"
            f"缺的大概是没 source。"
        )


def _check(context):
    """启动前的**纯检查**，不满足就抛错，绝不「先跑起来再说」。

    查两件**没有合理默认值**的输入：必填的 `robot_ip`、以及相机外参那四个
    四元数分量。猜一个的后果都是「臂去了一个你以为不是那里的地方」。
    `start_ur_driver:=false` 时不需要 `robot_ip`（那时驱动不起，IP 无人使用）。
    """
    if context.launch_configurations.get("start_ur_driver", "true") == "true":
        ip = context.launch_configurations.get("robot_ip", "").strip()
        if not ip:
            raise RuntimeError(
                "robot_ip 是必填项：真机入口必须知道控制箱的 IP。\n"
                "  用法（两个入口都行，robot_ip 会直通到这里）：\n"
                "      ros2 launch yolo_bringup real.launch.py "
                "robot_ip:=<控制箱 IP>\n"
                "      ros2 launch yolo_bringup bringup.launch.py "
                "camera_source:=realsense robot_ip:=<控制箱 IP>\n"
                "  在示教器上确认 IP：设置 → 系统 → 网络。\n"
                "  只想起 MoveIt / 感知（不起驱动）时加 start_ur_driver:=false，"
                "那时不需要 robot_ip。"
            )

    if context.launch_configurations.get("publish_camera_tf", "false") == "true":
        quat = [
            context.launch_configurations.get(k, "0.0")
            for k in ("camera_qx", "camera_qy", "camera_qz", "camera_qw")
        ]
        if all(_is_zero(v) for v in quat):
            raise RuntimeError(
                "publish_camera_tf:=true 但相机外参的四元数还是默认的全零，"
                "**这不是一个合法的旋转**（模长为 0）。\n"
                "  该四元数必须来自**手眼标定**，不能猜：相机装歪一点，"
                "投影出来的抓取点就偏同样多，而整个链路上没有任何一步会报错。\n"
                "  三选一：\n"
                "    · 标定完再传 camera_x..camera_qw（推荐）；\n"
                "    · 不起相机 TF：publish_camera_tf:=false，"
                "用外部已有的 TF 源；\n"
                "    · 确知相机与基座重合时显式传单位四元数 "
                "camera_qw:=1.0（一般不是你要的）。"
            )
    return []


def _is_zero(value: str) -> bool:
    try:
        return abs(float(value)) < 1e-12
    except (TypeError, ValueError):
        # 非字面量（理论上不会出现，都是 DeclareLaunchArgument 的默认值或
        # 命令行字面量）。当作「不是零」，把判断让给下游，不在这里误报。
        return False


def _driver(context):
    """UR 驱动 + 控制器。`start_ur_driver:=false` 时什么都不做。"""
    if context.launch_configurations.get("start_ur_driver", "true") != "true":
        return []
    path = _package_launch(
        "ur_robot_driver", "ur_control.launch.py",
        why="没有它就没有控制器，也就没有臂轨迹的 action",
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                # 官方默认就是 true，显式传是为了让「本文件要不要 RViz」
                # 一眼可见：Orin 上 7.4 GiB 内存，RViz 白占（也少一个能
                # 显示机器人状态的窗口，但真机看的是示教器和实物）。
                "launch_rviz": "false",
                "initial_joint_controller": INITIAL_JOINT_CONTROLLER,
                # 安全限制控制器：官方默认 true、k_position=20。**没在示教器上
                # 配好负载/重心就会在动作中途保护性停机**（AG-95 约 1 kg、
                # 离法兰 0.13 m+）。现场必须先在示教器上配 TCP 与负载，再跑动作。
                # 只有在「负载尚未配、且你接受少一层保护」时才传 false。
                "safety_limits": LaunchConfiguration("safety_limits"),
            }.items(),
        ),
    ]


def _moveit(context):
    """`move_group` —— 规划器的 `/compute_ik` 从这里来。"""
    if context.launch_configurations.get("start_moveit", "true") != "true":
        return []
    path = _package_launch(
        "ur_moveit_config", "ur_moveit.launch.py",
        why="没有它 /compute_ik 无人应答，规划器解不出任何目标",
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                # ⚠️ 两个默认都是 true（`:394` 与 `:397`），在这里都关掉：
                #   launch_rviz  —— Orin 上白占内存；
                #   launch_servo —— **一个能直接指挥机器人的节点**（moveit_servo）。
                #                   真机联调阶段没有任何东西需要它，而它一旦被
                #                   误用就是「绕过规划器直接动臂」（D59）。
                "launch_rviz": "false",
                "launch_servo": "false",
                # ⚠️ 必须与驱动传**同一个值**：它进的是这台机器人的
                # robot_description（UR 的 xacro 里安全限制会加东西）。两份
                # description 不一致时，move_group 的 FK/IK 与驱动实际发布的
                # TF 是**两套几何**，IK 解出来的关节角到真机上是偏的，而两边
                # 都不报错。ur_moveit.launch.py 自己也声明了这个参数。
                "safety_limits": LaunchConfiguration("safety_limits"),
            }.items(),
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory("yolo_bringup")

    args = [
        # ---- 必填 ----
        DeclareLaunchArgument(
            "robot_ip", default_value=_NO_DEFAULT,
            description="**必填**（start_ur_driver:=false 时不需要）：UR 控制箱 IP",
        ),
        # ---- 机器人 ----
        DeclareLaunchArgument(
            "ur_type", default_value="ur5",
            description="UR 型号。本仓库的 SRDF/URDF/几何参数都按 UR5 配的，"
                        "换型号要连 grasp_planner.yaml 的指尖偏置一起改",
        ),
        DeclareLaunchArgument(
            "safety_limits", default_value="true",
            description="安全限制控制器。⚠️ 示教器上没配负载/重心就上动作会在"
                        "中途保护性停机；配好之前不要为了「先跑起来」关掉它",
        ),
        # ---- 分阶段 ----
        DeclareLaunchArgument(
            "start_ur_driver", default_value="true",
            description="false = 不起驱动（只起 MoveIt / 感知，用于分阶段验收）",
        ),
        DeclareLaunchArgument(
            "start_moveit", default_value="true",
            description="false = 不起 move_group（只起驱动 / 感知）",
        ),
        DeclareLaunchArgument(
            "start_pipeline", default_value="true",
            description="false = 不起感知与抓取（只起驱动 / MoveIt）",
        ),
        # ---- 相机外参（手眼标定结果）----
        DeclareLaunchArgument(
            "publish_camera_tf", default_value="false",
            description="true = 发布 base_link→camera_link 静态 TF。"
                        "需同时给 camera_x..camera_qw（标定结果），否则报错退出",
        ),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_qx", default_value="0.0"),
        DeclareLaunchArgument("camera_qy", default_value="0.0"),
        DeclareLaunchArgument("camera_qz", default_value="0.0"),
        DeclareLaunchArgument("camera_qw", default_value="0.0"),
        # ---- 透传给感知/抓取 ----
        DeclareLaunchArgument("model", default_value="block_best.pt"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument(
            "target_frame", default_value="base_link",
            description="目标输出坐标系。必须与相机 TF 的父帧一致，否则投影"
                        "查不到 TF",
        ),
        DeclareLaunchArgument(
            "tf_required", default_value="true",
            description="真机建议 true：查不到 TF 就让 perception_3d 报错退出，"
                        "而不是发一堆坐标算错的目标",
        ),
        DeclareLaunchArgument(
            "gripper_backend", default_value="none",
            description="sim | ag95 | none。真机默认 none（只走位）—— 不是 ag95"
                        "没实现（代码侧已就绪），而是板上还没有 USB-RS485 适配器，"
                        "ag95 现在构造就会因打不开串口而报错；见文件头的说明",
        ),
        DeclareLaunchArgument(
            "carry_objects", default_value="false",
            description="false = 只走位、不夹持。须与 gripper_backend:=none 成对",
        ),
        DeclareLaunchArgument(
            "start_delay", default_value="0.0",
            description="感知/抓取节点延迟启动秒数。驱动与真机控制箱握手约需"
                        "十余秒；若日志里规划器先于控制器就绪，调大它（如 20）",
        ),
    ]

    # ---- 相机静态 TF ----
    # ⚠️ 父帧 base_link、子帧 **camera_link** —— 这个选法不是随意的：
    #   · D435i 驱动的 TF 是 camera_link → camera_color_frame →
    #     camera_color_optical_frame，**camera_link 是根**（板端实测）。
    #     所以给 camera_link 补一个父帧，整棵树就通了，optical frame 自然可达
    #     （perception_3d 用的正是 `/camera/color/camera_info` 的 frame_id =
    #     camera_color_optical_frame）。
    #   · **不能**直接发 base_link → camera_color_optical_frame：那会给
    #     optical frame 两个父节点，TF 树直接坏掉。
    #   · 因此手眼标定的结果必须**表达在 camera_link 上**。若标定工具给的是
    #     optical frame 的外参，要先左乘驱动那条固定的 camera_link→optical
    #     （一个固定的轴约定旋转，x 右/y 下/z 向前），换成 camera_link 再填这里。
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--qx", LaunchConfiguration("camera_qx"),
            "--qy", LaunchConfiguration("camera_qy"),
            "--qz", LaunchConfiguration("camera_qz"),
            "--qw", LaunchConfiguration("camera_qw"),
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    # ---- 感知/抓取 ----
    # 与仿真共用同一份参数定义，只在 pipeline.launch.py 一处（它的文件头写了
    # 为什么）—— 两边各写一份的话，改一处忘一处就是「仿真能跑、真机不跑」。
    # ⚠️ 这里**故意不写** max_sync_dt：它的真机默认值（0.3）已经由 pipeline.launch.py
    # 按 camera_source 给出（sim 0.1 / 其余 0.3），本文件再声明一次就是第二处定义 ——
    # 谁生效取决于 include 顺序，而那种错**不会报错**（D50/D64 都是这一类）。
    # 现场要调就命令行直通：`... bringup.launch.py camera_source:=realsense max_sync_dt:=0.5`
    # （未声明的参数会直接进 launch_configurations，实测如此）。
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "pipeline.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_pipeline")),
        launch_arguments={
            "camera_source": "realsense",
            "model": LaunchConfiguration("model"),
            "device": LaunchConfiguration("device"),
            "target_frame": LaunchConfiguration("target_frame"),
            "tf_required": LaunchConfiguration("tf_required"),
            "gripper_backend": LaunchConfiguration("gripper_backend"),
            "carry_objects": LaunchConfiguration("carry_objects"),
            "start_delay": LaunchConfiguration("start_delay"),
        }.items(),
    )

    return LaunchDescription(args + [
        # 先做纯检查（robot_ip、四元数），不通过就什么都不起。
        # 放在最前面：让「参数没给对」在**任何节点启动之前**就失败，
        # 而不是等驱动去连一个空 IP 超时。
        OpaqueFunction(function=_check),
        OpaqueFunction(function=_driver),
        OpaqueFunction(function=_moveit),
        camera_tf,
        pipeline,
    ])
