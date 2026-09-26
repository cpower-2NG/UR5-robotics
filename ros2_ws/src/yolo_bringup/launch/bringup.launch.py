"""
Launch 文件: 完整启动（Gazebo + 感知流水线 + 抓取规划）
用法:
  # 仿真：起 Gazebo，等 6 秒让世界就绪，再起感知流水线
  ros2 launch yolo_bringup bringup.launch.py

  # 真机：**不起 Gazebo**，由 realsense2_camera 提供 /camera/*
  ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense

  # 相机由外部提供（已有 rosbag / 别的驱动）
  ros2 launch yolo_bringup bringup.launch.py camera_source:=external

`camera_source` 同时决定**要不要起 Gazebo**：真机上再起一个仿真世界既浪费
算力，又会让 /clock、/tf、相机话题和真机抢同一个话题名（D2/D7 那类静默算错
的温床）。所以这不是「相机的来源」一个参数，而是「整车跑在哪个后端」。

仿真才需要 TimerAction 的 6 秒延迟（等 Gazebo 把 /clock 和相机话题发出来）；
真机没有可等的东西，故延迟按 camera_source 取 0。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_bringup')

    # ---- 参数 ----
    world_arg = DeclareLaunchArgument(
        'world', default_value='grasping_world.sdf',
        description='Gazebo world (仅 camera_source:=sim 时有效)',
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='block_best.pt',
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
        description='mock | moveit — 真机上用 moveit 才会真动',
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
    )
    camera_source_arg = DeclareLaunchArgument(
        'camera_source', default_value='sim',
        description='sim | realsense | external — 谁提供 /camera/* 话题，'
                    '以及是否启动 Gazebo',
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='目标输出坐标系（真机上通常是机械臂基座）',
    )
    tf_required_arg = DeclareLaunchArgument(
        'tf_required', default_value='false',
        description='true = 查不到 TF 就让 perception_3d 报错退出（真机建议 true）',
    )

    camera_source = LaunchConfiguration('camera_source')

    # 注意：比较必须发生在**求值期**。写 camera_source == 'sim' 是构造期的
    # Python 比较，会立刻得到 bool，IfCondition/TimerAction 都不接受。
    sim_only = IfCondition(PythonExpression([
        "'", camera_source, "' == 'sim'",
    ]))

    # ---- 仿真（仅 sim）----
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'simulation.launch.py')
        ),
        condition=sim_only,
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # ---- 感知流水线（仿真时延迟等 Gazebo 就绪；真机立即起）----
    pipeline_delay = PythonExpression([
        "'6.0' if '", camera_source, "' == 'sim' else '0.0'",
    ])

    pipeline_launch = TimerAction(
        period=pipeline_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'pipeline.launch.py')
                ),
                launch_arguments={
                    'model': LaunchConfiguration('model'),
                    'mode': LaunchConfiguration('mode'),
                    'device': LaunchConfiguration('device'),
                    'camera_source': camera_source,
                    'target_frame': LaunchConfiguration('target_frame'),
                    'tf_required': LaunchConfiguration('tf_required'),
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        mode_arg,
        device_arg,
        camera_source_arg,
        target_frame_arg,
        tf_required_arg,
        simulation_launch,
        pipeline_launch,
    ])
