"""
Launch 文件: 单独启动 3D 感知节点
用法:
  ros2 launch perception_3d perception.launch.py target_frame:=base_link
  # 真机（RealSense D435i，深度为 16UC1 毫米，由 encoding 自动判定）：
  ros2 launch perception_3d perception.launch.py target_frame:=base_link tf_required:=true
  # 若深度源编码非标准，可强制指定尺度（米/像素）：
  ros2 launch perception_3d perception.launch.py depth_scale:=0.001
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'target_frame', default_value='base_link',
            description='Target TF frame for 3D projection output'
        ),
        # 深度尺度：-1 = 按 msg.encoding 自动判定（16UC1→0.001, 32FC1→1.0）
        DeclareLaunchArgument(
            'depth_scale', default_value='-1.0',
            description='Depth pixel→metre scale. -1 = auto from msg.encoding'
        ),
        DeclareLaunchArgument(
            'max_sync_dt', default_value='0.1',
            description='Max allowed |detection − depth| stamp difference (s)'
        ),
        DeclareLaunchArgument(
            'depth_grid', default_value='5',
            description='Sampling grid size inside the detection box (5 → 5×5)'
        ),
        DeclareLaunchArgument(
            'depth_cluster_tol', default_value='0.03',
            description='Tolerance (m) for the nearest-depth cluster'
        ),
        DeclareLaunchArgument(
            'tf_required', default_value='false',
            description='true = abort if the camera→target TF is unavailable'
        ),
    ]

    projector_node = Node(
        package='perception_3d',
        executable='projector_node',
        name='perception_3d',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'depth_scale': LaunchConfiguration('depth_scale'),
            'max_sync_dt': LaunchConfiguration('max_sync_dt'),
            'depth_grid': LaunchConfiguration('depth_grid'),
            'depth_cluster_tol': LaunchConfiguration('depth_cluster_tol'),
            'tf_required': LaunchConfiguration('tf_required'),
        }],
    )

    return LaunchDescription(declared_args + [projector_node])
