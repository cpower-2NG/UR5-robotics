"""方阵世界启动器：Gazebo Harmonic + 相机桥接（**无机械臂**）。

用途
====
给 `docs/GRID_ENV_DESIGN.md` 的**训练流水线**用——采集训练图 → 自动标注 →
训练 YOLO。这条链只需要「相机画面」和「方块位姿」，不需要机械臂，所以本文件
**刻意不起** UR5 / MoveIt / ros2_control（那些在 `arm_grasp_sim/sim.launch.py`）。

抓取演示请用 `arm_grasp_sim/full.launch.py`（或唯一入口
`ros2 launch yolo_bringup bringup.launch.py camera_source:=sim`）。

为什么会有这个文件（历史）
==========================
第九轮删掉了 `simulation.launch.py`——它是个**没有机械臂**的世界，抓取链路接上
后作为「仿真入口」已无意义（规划器起来了却没臂可动）。但它同时还是**训练世界**
的唯一启动器，删掉后 `block_grid_world.sdf` / `color_objects_world.sdf` 就**没有
任何 launch 文件加载**了，而训练链第 1 步正依赖它。故这里把「训练世界」这一
职责单独拆出来保留，且只保留它。（第十轮）

用法
====
  ros2 launch yolo_bringup grid_world.launch.py
  ros2 launch yolo_bringup grid_world.launch.py world:=color_objects_world.sdf
  ros2 launch yolo_bringup grid_world.launch.py headless:=true

两个 world 都可用：`block_grid_world.sdf`（3×3 方阵，方块名 `block_0..8`，
训练主用）与 `color_objects_world.sdf`（红/蓝方块 + 绿圆柱）。二者相机参数一致
（见下方 CAMERA_* 常量），故共用同一套桥接与 TF。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── 相机（两个 world 的 camera_model 完全一致）──────────────────────────
# 两边都有一段 `<model name="camera_model">`（grep 定位；**故意不写行号**——
# 行号会随编辑漂，本轮就因为往文件头加了注释而失效过一次）：
#   <pose>0.5 0.0 1.5 0.0 1.5708 0.0</pose>
#   <link name="camera_link"> 内挂 rgb_camera / depth_camera 两个 sensor
# 故帧名遵循 gz 的 `<model>/<link>/<sensor>` 约定。
#
# ⚠️ 旋转为什么是这个四元数（与 arm_grasp_sim/sim.launch.py 同一套推导）：
# SDF 里的 rpy(0, 1.5708, 0) 只是**视觉模型**朝向；而像素 (u,v)+深度 Z 走的是
# **光学约定**（x 右、y 下、z 向前），两者之间差一个固定的轴重排。把该重排与
# 模型朝向合成后得到一个合法旋转（det=+1），四元数为 (0.7071, -0.7071, 0, 0)：
#     光心 x+ → world -y      光心 y+ → world -x      光心 z+ → world -z
# 该式已在抓取仿真中用于实跑反证（见 sim.launch.py 的注释），此处相机位姿与
# 之**逐位相同**，故直接复用。
CAMERA_FRAME = "camera_model/camera_link/rgb_camera"
CAMERA_X, CAMERA_Y, CAMERA_Z = 0.5, 0.0, 1.5
CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW = 0.7071067811865476, -0.7071067811865476, 0.0, 0.0


def generate_launch_description():
    pkg = get_package_share_directory("yolo_bringup")
    world_name = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    # world 是文件名而非路径，拼成绝对路径传给 gz
    world_path = [os.path.join(pkg, "worlds") + "/", world_name]
    bridge_yaml = os.path.join(pkg, "config", "ros_gz_bridge.yaml")

    # ── Gazebo ────────────────────────────────────────────────────
    # 用 ExecuteProcess 而非 Node：`gz sim` 是 Gazebo 自己的 CLI（来自 gz-tools2，
    # 在 PATH 上），**不是** ros_gz_sim 的节点——本机装的 Harmonic 变体
    # ros-humble-ros-gzharmonic-sim 的 lib 目录下并没有 `gz_sim` 这个可执行文件
    # （D39：写成 Node 会在解析期直接报 executable not found）。
    #
    # 这两个 world 只用 box 几何 + gz 自带插件，**不引用任何 model:// 网格**，
    # 所以不需要设 GZ_SIM_RESOURCE_PATH（抓取世界需要，见 sim.launch.py）。
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        name="gz_sim",
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_path],
        name="gz_sim",
        output="screen",
        condition=IfCondition(headless),
    )

    # ── 相机 TF ───────────────────────────────────────────────────
    # 只在你想让 perception_3d 也投影这个世界的画面时才需要（训练链用 meta
    # JSON 标注，不依赖 TF）。但发布它没有代价，且能让「采集到的图」与
    # 「perception_3d 算出的坐标」互相印证，故默认发。
    #
    # ⚠️ 真机侧对应的是**手眼标定结果**，不是这里写死的常量。
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        arguments=[
            "--x", str(CAMERA_X), "--y", str(CAMERA_Y), "--z", str(CAMERA_Z),
            "--qx", str(CAMERA_QX), "--qy", str(CAMERA_QY),
            "--qz", str(CAMERA_QZ), "--qw", str(CAMERA_QW),
            "--frame-id", "world",
            "--child-frame-id", CAMERA_FRAME,
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ── 相机桥接 ──────────────────────────────────────────────────
    # 复用 yolo_bringup/config/ros_gz_bridge.yaml——它本就是**为这两个 world**
    # 写的（话题名同名直通，理由见该文件头部注释）。本文件同时是那个 YAML 的
    # 唯一使用者，此前它一直是个孤儿。
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
    )

    # ── 时序 ──────────────────────────────────────────────────────
    # 桥接必须等 gz 把话题发出来；给 4 秒（与 arm_grasp_sim 的取值一致）。
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="block_grid_world.sdf",
            description="worlds/ 下的文件名：block_grid_world.sdf（3×3 方阵，"
                        "训练主用）| color_objects_world.sdf（三色物体）",
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="true = 只跑物理不起 Gazebo GUI（-s）",
        ),
        DeclareLaunchArgument(
            "publish_camera_tf", default_value="true",
            description="是否发布 world→相机的静态 TF（perception_3d 需要）",
        ),
        TimerAction(period=2.0, actions=[gz_sim, gz_sim_headless]),
        TimerAction(period=4.0, actions=[gz_bridge, camera_tf]),
    ])
