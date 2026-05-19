"""
Launch 文件: 单独启动抓取规划节点
用法:
  ros2 launch grasp_planner planner.launch.py mode:=mock
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
        description='Grasp mode: mock (test) or moveit (full IK)'
    )

    planner_node = Node(
        package='grasp_planner',
        executable='planner_node',
        name='grasp_planner',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
        }],
    )

    return LaunchDescription([
        mode_arg,
        planner_node,
    ])
