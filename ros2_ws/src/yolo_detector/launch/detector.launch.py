"""
Launch 文件: 单独启动 YOLO 检测节点
用法:
  ros2 launch yolo_detector detector.launch.py model:=block_best.pt
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    model_arg = DeclareLaunchArgument(
        'model', default_value='block_best.pt',
        description='YOLO model file (block_best.pt, or an absolute path)'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='Minimum confidence threshold'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Inference device (cuda:0 / cpu)'
    )

    # YOLO 检测节点
    detector_node = Node(
        package='yolo_detector',
        executable='detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
        }],
    )

    return LaunchDescription([
        model_arg,
        conf_arg,
        device_arg,
        detector_node,
    ])
