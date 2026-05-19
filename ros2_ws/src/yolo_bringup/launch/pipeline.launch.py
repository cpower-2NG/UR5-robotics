"""
Launch 文件: 启动感知+抓取流水线 (不含 Gazebo)
用法:
  ros2 launch yolo_bringup pipeline.launch.py mode:=mock

前置条件: 仿真环境已运行（相机话题就绪）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 参数 ----
    model_arg = DeclareLaunchArgument(
        'model', default_value='yolov8n.pt',
        description='YOLO model file'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='Detection confidence threshold'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Inference device'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
        description='Grasp mode: mock | moveit'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='TF frame for target output'
    )

    # ---- YOLO 检测节点 ----
    detector = Node(
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

    # ---- 3D 感知节点 ----
    perception = Node(
        package='perception_3d',
        executable='projector_node',
        name='perception_3d',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
        }],
    )

    # ---- 抓取规划节点 ----
    planner = Node(
        package='grasp_planner',
        executable='planner_node',
        name='grasp_planner',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
        }],
    )

    return LaunchDescription([
        model_arg,
        conf_arg,
        device_arg,
        mode_arg,
        target_frame_arg,
        detector,
        perception,
        planner,
    ])
