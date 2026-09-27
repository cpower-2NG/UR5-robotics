"""
Launch 文件: 单独启动 YOLO 检测节点

⚠️ 本文件**不在主链路上**：全仓库没有一处 include 它（`setup.py` 只是把它装进
`share/`）。仿真与真机的唯一入口是 `yolo_bringup`（`bringup.launch.py` →
`pipeline.launch.py`），`model` / `confidence_threshold` / `device` 在那里也有定义。
本文件的存在意义只是「把检测节点单独拉起来调」；**改这里的默认值对主链路无效**。

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
