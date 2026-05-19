"""
Launch 文件: 启动 Gazebo Harmonic 仿真环境
用法:
  ros2 launch yolo_bringup simulation.launch.py world:=grasping_world.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_bringup')

    # ---- 参数 ----
    world_arg = DeclareLaunchArgument(
        'world', default_value='grasping_world.sdf',
        description='Gazebo world file name (located in worlds/)'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI'
    )

    world_path = PathJoinSubstitution([pkg_dir, 'worlds', LaunchConfiguration('world')])

    # ---- Gazebo Harmonic 启动 ----
    # gz sim -r <world.sdf>
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=None,  # 始终启动
    )

    # ---- ros_gz_bridge: 桥接 Gazebo ↔ ROS2 话题 ----
    # 使用 parameter_bridge 模式
    bridge_config = os.path.join(pkg_dir, 'config', 'ros_gz_bridge.yaml')

    # RGB 图像
    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_rgb',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '--ros-args', '-r', '__node:=bridge_rgb'
        ],
        output='screen',
    )

    # 深度图像
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth',
        arguments=[
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '--ros-args', '-r', '__node:=bridge_depth'
        ],
        output='screen',
    )

    # 相机信息
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_info',
        arguments=[
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', '-r', '__node:=bridge_camera_info'
        ],
        output='screen',
    )

    # 关节状态
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_joint_states',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '--ros-args', '-r', '__node:=bridge_joint_states'
        ],
        output='screen',
    )

    # 时钟
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '--ros-args', '-r', '__node:=bridge_clock'
        ],
        output='screen',
    )

    # ---- RViz2 可视化（可选） ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'grasping.rviz')],
        condition=None,
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        gz_sim,
        TimerAction(
            period=3.0,  # 等 Gazebo 启动
            actions=[
                bridge_rgb,
                bridge_depth,
                bridge_camera_info,
                bridge_joint_states,
                bridge_clock,
            ]
        ),
    ])
