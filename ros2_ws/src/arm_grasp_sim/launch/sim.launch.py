"""Launch Gazebo Harmonic + UR5 + ros2_control + RGBD camera + ros_gz_bridge + MoveIt2.

从参考项目 grasp_sim 迁入主线（P3-6）。迁移时修掉了 4 个缺陷（D34–D37），
每条都在代码处以「D3x」标注了原因与修法，详见 docs/RESTRUCTURE_PLAN.md。

用法：
  ros2 launch arm_grasp_sim sim.launch.py
  ros2 launch arm_grasp_sim sim.launch.py headless:=true     # 无 GUI（CI/远程）
"""
import os
import subprocess

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur_moveit_config.launch_common import load_yaml

URDF_PATH = "/tmp/ur5.urdf"
SRDF_PATH = "/tmp/ur5.srdf"
COMBINED_XACRO = "models/ur5_with_gripper.urdf.xacro"

# ── 仿真相机的真实位姿（取自 worlds/grasp_world.sdf 的 overhead_camera）──────
# <pose>0.5 0.0 1.145 0 1.57 0</pose>
#
# 旋转部分不是「rpy(0,1.57,0) 直接转四元数」——那只是 Gazebo 的**视觉模型**
# 朝向。图像像素 (u,v)+深度 Z 走的是**光学约定**（x 右、y 下、z 向前），
# 两者之间差一个固定的轴重排。把该重排与模型朝向合成后，结果是一个
# **合法旋转**（det=+1），四元数为 (x=0.7071, y=-0.7071, z=0, w=0)：
#
#     光心 x+ → world -y      光心 y+ → world -x      光心 z+ → world -z
#
# 已用实跑数据反证：相机(0.5,0,1.145) + R·(0.119,0.007,0.695) = (0.493,-0.119,0.450)，
# 与 grasp_planner 日志里那条成功抓取的世界坐标**精确吻合**。
# 参考项目把它写成硬编码常量、并注释称「无法用纯旋转表示」——那句注释是错的。
CAMERA_FRAME = "overhead_camera/camera_link/color_camera"
CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW = 0.7071067811865476, -0.7071067811865476, 0.0, 0.0


def _run_xacro(argv):
    """同步执行 xacro 并返回 stdout。

    D34 修法：参考项目把 URDF/SRDF 的**生成**放在 ExecuteProcess（运行期），
    却在 generate_launch_description() 里用 open() **构造期**读取，于是
    「首次冷启动必然失败」——文件那一刻还不存在。参考项目的 LAB.md §9.5
    也警告过该现象。
    这里改为在**解析 launch 文件时同步执行** xacro：生成完成才返回，
    后续的 open()/写文件天然安全，竞态从根上消失，也不再需要
    ExecuteProcess 与 TimerAction 去猜时序。
    """
    result = subprocess.run(argv, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"xacro 执行失败（{result.returncode}）：{' '.join(argv)}\n"
            f"stderr:\n{result.stderr}"
        )
    return result.stdout


def generate_launch_description():
    pkg = get_package_share_directory("arm_grasp_sim")
    # 用**源码路径**取 xacro：改了 xacro 不用重新 colcon build 就能生效
    # get_package_share_directory 返回 .../install/arm_grasp_sim/share/arm_grasp_sim
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg))))
    src_pkg = os.path.join(ws_root, "src", "arm_grasp_sim")

    world_path = os.path.join(pkg, "worlds", "grasp_world.sdf")
    bridge_yaml = os.path.join(pkg, "config", "gz_bridge.yaml")
    ros2_control_yaml = os.path.join(pkg, "config", "ros2_control.yaml")
    combined_xacro_path = os.path.join(src_pkg, COMBINED_XACRO)
    srdf_xacro_path = os.path.join(src_pkg, "config", "ur5_gripper.srdf.xacro")

    headless = LaunchConfiguration("headless")

    # Add ROS share dirs to Gazebo resource path so mesh model:// URIs resolve
    ros_share = "/opt/ros/humble/share"
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = f"{ros_share}" + (f":{existing_gz_path}" if existing_gz_path else "")
    set_gz_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path)

    xacro_common = [
        "name:=ur5", "ur_type:=ur5", "safety_limits:=false",
        "sim_ignition:=true", "sim_gazebo:=false",
        f"simulation_controllers:={ros2_control_yaml}",
    ]

    # ── D34：解析期同步生成，消除「构造期读运行期文件」的竞态 ──────────
    urdf_contents = _run_xacro(["xacro", combined_xacro_path] + xacro_common)
    srdf_contents = _run_xacro(["xacro", srdf_xacro_path, "name:=ur5"])
    # 写盘供 ros_gz_sim create -file 使用（它只吃文件路径，不吃字符串）
    with open(URDF_PATH, "w") as f:
        f.write(urdf_contents)
    with open(SRDF_PATH, "w") as f:
        f.write(srdf_contents)

    # Gazebo Sim（headless 时加 -s，只跑物理不起 GUI）
    #
    # D38：原写法把 gz_ros2_control 的路径硬编码成
    #   <本 ws>/install/gz_ros2_control/lib
    # 一旦它不在本 ws（例如作为独立第三方 ws 叠加上来），这个目录并不存在，
    # GZ_SIM_SYSTEM_PLUGIN_PATH / LD_LIBRARY_PATH 就指向空目录，Gazebo 加载
    # libgz_ros2_control-system.so 失败——而报错只在 Gazebo 侧，离原因很远。
    # 改为**问 ament 要**：谁提供 gz_ros2_control 都能找到。
    #
    # 注意：不能用 apt 的 ros-humble-gz-ros2-control——它依赖 libignition-*
    # （Fortress / gz-sim6），而本项目跑的是 Harmonic（gz-sim8），两者 ABI 不通。
    # 必须用 GZ_VERSION=harmonic 源码编译的版本。
    ros2_lib = "/opt/ros/humble/lib"
    try:
        gz_ros2_control_lib = os.path.join(
            get_package_prefix("gz_ros2_control"), "lib"
        )
    except PackageNotFoundError:
        raise RuntimeError(
            "找不到 gz_ros2_control。Gazebo 加载 ros2_control 插件需要它，且必须是\n"
            "用 GZ_VERSION=harmonic 源码编译的版本（apt 的 ros-humble-gz-ros2-control\n"
            "依赖 Fortress/gz-sim6，与本项目的 Harmonic/gz-sim8 不兼容）。\n"
            "  git clone -b humble https://github.com/ros-controls/gz_ros2_control\n"
            "  export GZ_VERSION=harmonic && colcon build --packages-up-to gz_ros2_control\n"
            "若它建在别的 ws，把那个 ws 的 install/setup.bash 先 source 再 source 本 ws。"
        )
    ws_lib = gz_ros2_control_lib
    gz_plugin_path = f"{ws_lib}:{ros2_lib}"
    # D39：**不能**写成 Node(package="ros_gz_sim", executable="gz_sim")。
    # 本机装的是 Harmonic 变体 ros-humble-ros-gzharmonic-sim，它的
    # /opt/ros/humble/lib/ros_gz_sim/ 下只有 create 和 test_stopwatch，
    # **没有 gz_sim 这个可执行文件**，用 Node 会在解析期直接报
    #   executable 'gz_sim' not found on the libexec directory ...
    # 且 ros2 launch 任何包都起不来。
    #
    # `gz sim` 是 Gazebo 自己的 CLI（来自 gz-tools2，在 PATH 上），不是
    # ros_gz_sim 的节点，所以只能用 ExecuteProcess 起。
    # 这也是原参考项目 grasp_sim 的做法（本文件在迁移时曾误改成 Node，已改回）。
    gz_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        "LD_LIBRARY_PATH": f"{ws_lib}:{ros2_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}",
    }
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        name="gz_sim",
        output="screen",
        additional_env=gz_env,
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_path],
        name="gz_sim",
        output="screen",
        additional_env=gz_env,
        condition=IfCondition(headless),
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_contents, "use_sim_time": True}],
    )

    spawn_ur5 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ur5",
        output="screen",
        arguments=[
            "-world", "grasp_world", "-name", "ur5", "-file", URDF_PATH,
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    # ── 相机 TF（仿真侧的地面真值）─────────────────────────────────
    # perception_3d 靠 TF 把像素投影到 base_link，所以相机帧**必须**在 TF 树里。
    # 参考项目从不发布它：把相机位姿硬编码在抓取节点里，另发一个自称
    # 「approximate、无法用纯旋转表示」的假 TF 供可视化——那条 TF 与真实
    # 投影用的变换并不一致。迁入主线后统一由这里发布。
    #
    # 真机侧：同一接口改由**手眼标定结果**提供（静态 TF）；若相机是
    # 眼在手上（eye-in-hand），则必须来自机器人 URDF/TF 而非静态发布。
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_publisher",
        output="screen",
        arguments=[
            "--x", "0.5", "--y", "0.0", "--z", "1.145",
            "--qx", str(CAMERA_QX), "--qy", str(CAMERA_QY),
            "--qz", str(CAMERA_QZ), "--qw", str(CAMERA_QW),
            "--frame-id", "world",
            "--child-frame-id", CAMERA_FRAME,
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ── MoveIt2 move_group ───────────────────────────────────────
    kinematics_params = os.path.join(pkg, "config", "kinematics.yaml")
    joint_limits_yaml = load_yaml("ur_moveit_config", "config/joint_limits.yaml")
    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers_yaml.pop("scaled_joint_trajectory_controller", None)
    controllers_yaml["joint_trajectory_controller"]["default"] = True

    ompl_config = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl_config.update(ompl_yaml)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            {"robot_description": urdf_contents},
            {"robot_description_semantic": srdf_contents},
            {"publish_robot_description_semantic": True},
            kinematics_params,
            {"robot_description_planning": joint_limits_yaml},
            {"move_group": ompl_config},
            {
                "moveit_simple_controller_manager": controllers_yaml,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
            {
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
                "trajectory_execution.execution_duration_monitoring": False,
            },
            {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
            {"use_sim_time": True},
        ],
    )

    # ── 控制器 spawner ───────────────────────────────────────────
    # JTC 管全部 7 个关节（6 臂 + 夹爪）
    #
    # D35：参考项目在两条 spawner 命令后都加了 `|| true`，于是**控制器加载失败
    # 会被吞掉**——launch 照样「成功」，直到后面规划失败才暴露，且现象与真实
    # 原因相距极远。这里去掉 `|| true`，让失败真的失败。
    #
    # D37：参考项目在 spawner 之后又调了一次 switch_controller 来「激活」控制器。
    # 但 spawner 默认已经激活（它自己就会 configure+activate），这次重复调用
    # 只会产生两条误导性的 `Could not activate controller ...` WARN，
    # 而响应其实是 ok=True。纯噪音，删掉。
    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_controllers",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--controller-manager-timeout", "30",
            "--ros-args", "-p", "use_sim_time:=true",
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
    )

    # ── D36：删掉了 set_p_gain ────────────────────────────────────
    # 参考项目用一个 `sleep 2 && ros2 param set /gz_ros2_control
    # position_proportional_gain 5.0` 想把增益设成 5.0，实测**必然失败**
    # （sleep 2 时节点还不存在 → `Node not found` → exit 1）。
    #
    # 但更关键的是：**这个增益根本没法这样设**。查 gz_ros2_control 源码：
    #   - gz_ros2_control_plugin.cpp:357 用自己的 node options 建节点
    #     `rclcpp::Node::make_shared("gz_ros2_control", ns)`，**不接收参数覆盖**；
    #   - URDF 的 `<ros><parameter>` 与 yaml 的 `--params-file` 是在之后
    #     （:381–396）注入**全局 context** 的，只对随后创建的 controller_manager
    #     生效 —— 这就是为什么 yaml 里 `controller_manager:` 段有效
    #     （实测 update_rate=100、allow_partial_joints_goal=True），
    #     而 `gz_ros2_control:` 段完全无效。
    #   - gz_system.cpp:273 在 configure 时读一次进成员变量，之后再没重读，
    #     所以**事后 `ros2 param set` 也不会改变行为**。
    # 结论：该参数恒为库默认 **0.1**（gz_system.cpp:270 `default_gain = 0.1`），
    # 实测日志 `The position_proportional_gain has been set to: 0.1` 与之一致。
    # URDF 里那个 100.0 与 yaml 里那个 10.0 **从来没有生效过** —— 所有成功的
    # 仿真抓取都是在 0.1 下完成的。故此处删除该进程，并保留本段说明以免
    # 后来者再去「修」一个不存在的问题。若确需调大增益，得改 gz_ros2_control 源码。

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="true = 只跑物理不起 Gazebo GUI（-s）",
        ),
        set_gz_path,
        TimerAction(period=2.0, actions=[gz_sim, gz_sim_headless]),        # t=2
        TimerAction(period=4.0, actions=[robot_state_pub]),               # t=4
        TimerAction(period=5.0, actions=[spawn_ur5]),                     # t=5
        TimerAction(period=6.0, actions=[camera_tf]),                     # t=6
        TimerAction(period=8.0, actions=[move_group]),                    # t=8 (MoveIt2 IK)
        TimerAction(period=10.0, actions=[spawn_controllers]),            # t=10
        TimerAction(period=12.0, actions=[gz_bridge]),                    # t=12
    ])
