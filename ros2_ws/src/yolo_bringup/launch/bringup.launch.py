"""
Launch 文件: 完整启动（Gazebo + 感知流水线 + 可选 RViz）
用法:
  ros2 launch yolo_bringup bringup.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_bringup')

    # ---- 参数 ----
    world_arg = DeclareLaunchArgument(
        'world', default_value='grasping_world.sdf',
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='yolov8n.pt',
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
    )

    # ---- 仿真 ----
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': 'grasping_world.sdf',
        }.items(),
    )

    # ---- 感知流水线（延迟启动，等仿真就绪） ----
    pipeline_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'pipeline.launch.py')
                ),
                launch_arguments={
                    'model': 'yolov8n.pt',
                    'mode': 'mock',
                    'device': 'cuda:0',
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        mode_arg,
        device_arg,
        simulation_launch,
        pipeline_launch,
    ])
