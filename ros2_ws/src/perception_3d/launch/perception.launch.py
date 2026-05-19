"""
Launch 文件: 单独启动 3D 感知节点
用法:
  ros2 launch perception_3d perception.launch.py target_frame:=base_link
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='Target TF frame for 3D projection output'
    )

    projector_node = Node(
        package='perception_3d',
        executable='projector_node',
        name='perception_3d',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
        }],
    )

    return LaunchDescription([
        target_frame_arg,
        projector_node,
    ])
