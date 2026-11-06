"""阶段 4 仿真栈：Gazebo Harmonic + UR5 + AG-95 + 双 RGBD + ros2_control + MoveIt2。

与阶段 2/3 仿真底座的区别（全部按真机实测改）：

- 桌面 z=0（真机 base 系；示教器读数 `基座 z=+3.88 mm`）；
- 方块 29.5 mm，拾取区 [0.25,0.575]×[0.17,0.42] m；
- 夹爪换成简化 AG-95（95 mm 行程，tool0→指尖 204.2 mm）；
- 腕部相机按实测 TF 装到 tool0；俯视相机按实测外参装到 base_link；
- 场景由 `scripts/gen_scene.py` 用固定种子生成（S0–S4，各 5 布局）。

用法：
  ros2 launch arm_grasp_sim sim.launch.py                      # S0#0 + GUI
  ros2 launch arm_grasp_sim sim.launch.py headless:=true scene:=S4:2
  ros2 launch arm_grasp_sim sim.launch.py world:=/path/to/x.sdf
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

# ── 俯视相机外参（真机实测）─────────────────────────────────────────────
# 来源 docs/calib/extrinsics-2026-10-03-base_link.json：
#   parent base_link, child camera_link；camera_link 用 RealSense 约定
#   （x 前 = 光轴 / y 左 / z 上）。光轴 54.27° 下俯，落在台面 (0.374, 0.406)。
TOPCAM_XYZ = (0.34694477757210807, 0.8693043936314018, 0.6454397556505314)
TOPCAM_Q = (0.32913450910630487, 0.31637465543060517,
            -0.61852813688270547, 0.63953146615387635)
# 内参：分辨率/焦距按真机实测（1280×720，fx≈917.6），主点用**仿真实际渲染值**。
# ⚠️ Gazebo 的主点固定在图像正中心 (W/2, H/2)，无法配置偏移；真机 D435 的
#    cx/cy = (649.55, 344.91)。若照抄真机主点，整条链会带 ~10 px ≈ 1 cm 的系统偏差
#    （2026-11-06 实测：块顶面反投影偏 +11 mm）。所以仿真必须发布自己真实的内参，
#    否则「算法在仿真里对、上真机偏」会被误判成外参问题。
TOPCAM_W, TOPCAM_H = 1280, 720
TOPCAM_FX, TOPCAM_FY = 917.62046813964844, 917.62052536010742  # Gazebo 实测（hfov 由真机 fx 反算）
TOPCAM_CX, TOPCAM_CY = TOPCAM_W / 2.0, TOPCAM_H / 2.0

# ── 腕部相机内参 ────────────────────────────────────────────────────────
# 真机实测 1280×720, fx 908.152 / fy 908.375 / cx 639.435 / cy 363.314, D=0；
# 仿真渲染实测 fx=fy=908.9544、主点 (640,360)（见上面俯视相机的说明）。
WRIST_W, WRIST_H = 1280, 720
WRIST_FX, WRIST_FY = 908.95439147949219, 908.95437240600586
WRIST_CX, WRIST_CY = WRIST_W / 2.0, WRIST_H / 2.0


def _run(argv):
    result = subprocess.run(argv, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"命令失败（{result.returncode}）：{' '.join(argv)}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return result.stdout


def generate_launch_description():
    pkg = get_package_share_directory("arm_grasp_sim")
    # 用源码路径取 xacro / 生成器：改了不用 colcon build 就生效
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg))))
    src_pkg = os.path.join(ws_root, "src", "arm_grasp_sim")

    bridge_yaml = os.path.join(pkg, "config", "gz_bridge.yaml")
    ros2_control_yaml = os.path.join(pkg, "config", "ros2_control.yaml")
    combined_xacro_path = os.path.join(src_pkg, COMBINED_XACRO)
    srdf_xacro_path = os.path.join(src_pkg, "config", "ur5_gripper.srdf.xacro")
    gen_scene = os.path.join(src_pkg, "scripts", "gen_scene.py")
    adapter_py = os.path.join(src_pkg, "scripts", "camera_adapter.py")

    headless = LaunchConfiguration("headless")
    scene = LaunchConfiguration("scene")
    gripper = LaunchConfiguration("gripper")
    world_override = LaunchConfiguration("world")
    # 物理引擎见下方 physics 参数声明处的实测说明。
    physics = LaunchConfiguration("physics")

    # ── 场景：解析期用固定种子生成，产出 world + 真值 ────────────────
    # scene 形如 "S4:2"；world:= 直接指定现成 SDF 时优先。
    # （launch 的 LaunchConfiguration 在解析期取不到值，这里直接读 argv。）
    argv = {a.split(":=", 1)[0]: a.split(":=", 1)[1]
            for a in os.sys.argv if ":=" in a and a.split(":=", 1)[0] in
            ("scene", "world", "gripper", "headless")}
    explicit_world = argv.get("world", "")
    if explicit_world:
        world_path = explicit_world
        truth_path = os.environ.get("SCENE_TRUTH", "")
    else:
        scene_str = argv.get("scene", "S0:0")
        try:
            batch, index = scene_str.split(":")
            index = int(index)
        except ValueError:
            raise RuntimeError(f"scene 参数应为 '<batch>:<index>'，收到 {scene_str!r}")
        scene_dir = f"/tmp/grasp_scene_{batch}_{index}"
        _run(["python3", gen_scene, "--batch", batch, "--index", str(index),
              "--out-dir", scene_dir])
        world_path = os.path.join(scene_dir, "grasp_world.sdf")
        truth_path = os.path.join(scene_dir, "scene_truth.json")

    # ── URDF / SRDF：解析期同步生成 ─────────────────────────────────
    xacro_common = [
        "name:=ur5", "ur_type:=ur5", "safety_limits:=false",
        "sim_ignition:=true", "sim_gazebo:=false",
        f"gripper:={argv.get('gripper', 'ag95')}", "with_wrist_camera:=true",
        f"simulation_controllers:={ros2_control_yaml}",
    ]
    urdf_contents = _run(["xacro", combined_xacro_path] + xacro_common)
    srdf_contents = _run(["xacro", srdf_xacro_path, "name:=ur5"])
    with open(URDF_PATH, "w") as f:
        f.write(urdf_contents)
    with open(SRDF_PATH, "w") as f:
        f.write(srdf_contents)

    # ── Gazebo ─────────────────────────────────────────────────────
    ros_share = "/opt/ros/humble/share"
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = f"{ros_share}" + (f":{existing_gz_path}" if existing_gz_path else "")
    set_gz_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path)

    ros2_lib = "/opt/ros/humble/lib"
    try:
        gz_ros2_control_lib = os.path.join(get_package_prefix("gz_ros2_control"), "lib")
    except PackageNotFoundError:
        raise RuntimeError(
            "找不到 gz_ros2_control（必须是用 GZ_VERSION=harmonic 源码编译的版本；"
            "apt 的 ros-humble-gz-ros2-control 依赖 Fortress/gz-sim6，ABI 不通）。")
    gz_plugin_path = f"{gz_ros2_control_lib}:{ros2_lib}"
    gz_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        "LD_LIBRARY_PATH": f"{gz_ros2_control_lib}:{ros2_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}",
    }
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "--render-engine", "ogre2",
             "--physics-engine", physics, world_path],
        name="gz_sim", output="screen", additional_env=gz_env,
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "--render-engine", "ogre2",
             "--physics-engine", physics, world_path],
        name="gz_sim", output="screen", additional_env=gz_env,
        condition=IfCondition(headless),
    )

    robot_state_pub = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": urdf_contents, "use_sim_time": True}],
    )

    spawn_ur5 = Node(
        package="ros_gz_sim", executable="create", name="spawn_ur5", output="screen",
        arguments=["-world", "grasp_world", "-name", "ur5", "-file", URDF_PATH,
                   "-x", "0.0", "-y", "0.0", "-z", "0.0"],
    )

    # ── 俯视相机 TF（真机外参；不是随手编的近似值）──────────────────
    topcam_link = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="topcam_link_tf", output="screen",
        arguments=["--x", str(TOPCAM_XYZ[0]), "--y", str(TOPCAM_XYZ[1]),
                   "--z", str(TOPCAM_XYZ[2]),
                   "--qx", str(TOPCAM_Q[0]), "--qy", str(TOPCAM_Q[1]),
                   "--qz", str(TOPCAM_Q[2]), "--qw", str(TOPCAM_Q[3]),
                   "--frame-id", "base_link", "--child-frame-id", "topcam_camera_link"],
        parameters=[{"use_sim_time": True}],
    )
    topcam_optical = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="topcam_optical_tf", output="screen",
        arguments=["--roll", "-1.5707963267948966", "--pitch", "0",
                   "--yaw", "-1.5707963267948966",
                   "--frame-id", "topcam_camera_link",
                   "--child-frame-id", "topcam_optical_frame"],
        parameters=[{"use_sim_time": True}],
    )

    # ── MoveIt2 ────────────────────────────────────────────────────
    kinematics_params = os.path.join(pkg, "config", "kinematics.yaml")
    joint_limits_yaml = load_yaml("ur_moveit_config", "config/joint_limits.yaml")
    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers_yaml.pop("scaled_joint_trajectory_controller", None)
    controllers_yaml["controller_names"] = [
        n for n in controllers_yaml.get("controller_names", [])
        if n != "scaled_joint_trajectory_controller"]
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
        package="moveit_ros_move_group", executable="move_group", name="move_group",
        output="screen",
        parameters=[
            {"robot_description": urdf_contents},
            {"robot_description_semantic": srdf_contents},
            {"publish_robot_description_semantic": True},
            kinematics_params,
            {"robot_description_planning": joint_limits_yaml},
            {"move_group": ompl_config},
            {"moveit_simple_controller_manager": controllers_yaml,
             "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"moveit_manage_controllers": False,
             "trajectory_execution.allowed_execution_duration_scaling": 1.2,
             "trajectory_execution.allowed_goal_duration_margin": 0.5,
             "trajectory_execution.allowed_start_tolerance": 0.01,
             "trajectory_execution.execution_duration_monitoring": False},
            {"publish_planning_scene": True, "publish_geometry_updates": True,
             "publish_state_updates": True, "publish_transforms_updates": True},
            {"use_sim_time": True},
        ],
    )

    spawn_controllers = Node(
        package="controller_manager", executable="spawner", name="spawn_controllers",
        output="screen",
        arguments=["joint_state_broadcaster", "joint_trajectory_controller",
                   "--controller-manager-timeout", "30",
                   "--ros-args", "-p", "use_sim_time:=true"],
    )

    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
    )

    # ── 相机适配：编码/frame_id/内参 全部按真机契约 ──────────────────
    wrist_adapter = Node(
        package="arm_grasp_sim", executable="camera_adapter.py", name="wrist_camera_adapter",
        output="screen",
        parameters=[{
            "camera_name": "wrist",
            "wrist_in_color": "/sim/wrist/color",
            "wrist_in_depth": "/sim/wrist/depth",
            "wrist_in_info": "",
            # ⚠️ 腕部相机**不能**占用 /camera/*：主线 pipeline（检测器、投影器）
            #    认的 /camera 是**俯视**那一路（`realsense_camera.yaml` 钉的是俯视
            #    SDK 序列号，projector 的 54° 斜视注释也指它）。2026-11-07 之前这里
            #    把腕部发到 /camera、俯视发到 /topcam，于是 `pipeline.launch.py`
            #    在仿真里默认读的 /camera/depth 其实是**腕部**视角 —— 典型症状是
            #    「仿真能跑、真机不跑」。现在按真机契约：/camera = 俯视，
            #    腕部另立 /wrist（真机若接腕部相机也照这个名字）。
            "wrist_out_color": "/wrist/color/image_raw",
            "wrist_out_depth": "/wrist/depth/image_raw",
            "wrist_out_info": "/wrist/color/camera_info",
            "wrist_optical_frame": "wrist_camera_optical_frame",
            "wrist_width": WRIST_W, "wrist_height": WRIST_H,
            "wrist_fx": WRIST_FX, "wrist_fy": WRIST_FY,
            "wrist_cx": WRIST_CX, "wrist_cy": WRIST_CY,
            "wrist_publish_info": True,
        }],
    )
    topcam_adapter = Node(
        package="arm_grasp_sim", executable="camera_adapter.py", name="topcam_camera_adapter",
        output="screen",
        parameters=[{
            "camera_name": "topcam",
            "topcam_in_color": "/sim/topcam/color",
            "topcam_in_depth": "/sim/topcam/depth",
            "topcam_in_info": "",
            # 俯视 = 主线契约里的 /camera/*（理由见上面 wrist_adapter 的注释）
            "topcam_out_color": "/camera/color/image_raw",
            "topcam_out_depth": "/camera/depth/image_raw",
            "topcam_out_info": "/camera/color/camera_info",
            "topcam_optical_frame": "topcam_optical_frame",
            "topcam_width": TOPCAM_W, "topcam_height": TOPCAM_H,
            "topcam_fx": TOPCAM_FX, "topcam_fy": TOPCAM_FY,
            "topcam_cx": TOPCAM_CX, "topcam_cy": TOPCAM_CY,
            "topcam_publish_info": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("scene", default_value="S0:0",
                              description="场景批次:布局，如 S0:0 / S4:2"),
        DeclareLaunchArgument("gripper", default_value="ag95"),
        # 2026-11-06 实测：DART 下夹爪的**移动副**关节收不到 JointVelocityCmd ——
        # gz_ros2_control 位置分支每步算出 -0.275 m/s 写进组件，手指纹丝不动
        # （转动副的手臂正常）。换 bullet-featherstone 后同一条命令 1 s 内到位。
        # 另有：DART 也不支持 <mimic> 约束（Physics.cc 报错后静默不建），
        # bullet-featherstone 支持 —— 右指靠 URDF mimic 由物理层跟随左指。
        DeclareLaunchArgument(
            "physics", default_value="gz-physics-bullet-featherstone-plugin",
            description="gz 物理引擎插件；DART 驱动不了移动副 + 不支持 mimic"),
        DeclareLaunchArgument("world", default_value="",
                              description="直接指定 SDF（优先于 scene）"),
        set_gz_path,
        TimerAction(period=2.0, actions=[gz_sim, gz_sim_headless]),
        TimerAction(period=4.0, actions=[robot_state_pub]),
        TimerAction(period=5.0, actions=[spawn_ur5]),
        TimerAction(period=6.0, actions=[topcam_link, topcam_optical]),
        TimerAction(period=8.0, actions=[move_group]),
        TimerAction(period=10.0, actions=[spawn_controllers]),
        TimerAction(period=12.0, actions=[gz_bridge, wrist_adapter, topcam_adapter]),
    ])
