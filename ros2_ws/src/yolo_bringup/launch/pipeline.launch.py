"""
Launch 文件: 启动感知+抓取流水线 (不含 Gazebo)
用法:
  # 仿真：相机话题由 simulation.launch.py 的 ros_gz_bridge 提供
  ros2 launch yolo_bringup pipeline.launch.py camera_source:=sim

  # 真机：自己拉起 realsense2_camera（需要 ros-humble-realsense2-camera）
  ros2 launch yolo_bringup pipeline.launch.py camera_source:=realsense

  # 相机由别人提供（外部驱动/已有 rosbag），本文件只起上层节点
  ros2 launch yolo_bringup pipeline.launch.py camera_source:=external

相机来源用**单个枚举**而不是 use_sim/use_realsense 两个布尔量：
两个布尔量存在「都为真」这种无意义且必须额外校验的状态，枚举没有。
三者对上层节点完全透明——话题名一致，故 yolo_detector / perception_3d /
grasp_planner 的参数不变。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# realsense2_camera 开启对齐后，深度发在这个话题上（不是 /camera/depth/image_raw，
# 那个是**未对齐**的）。D2 要求用对齐深度配彩色内参，故真机走这个。
REALSENSE_ALIGNED_DEPTH = '/camera/aligned_depth_to_color/image_raw'
SIM_DEPTH = '/camera/depth/image_raw'


def generate_launch_description():
    # ---- 参数 ----
    model_arg = DeclareLaunchArgument(
        'model', default_value='block_best.pt',
        description='YOLO model file (block detector; resolved by detector_node)'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5',
        description='Detection confidence threshold'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Inference device: cpu | cuda:N (falls back to cpu unless '
                    'device_fallback:=false)'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='mock',
        description='Grasp mode: mock | moveit'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='TF frame for target output'
    )
    camera_source_arg = DeclareLaunchArgument(
        'camera_source', default_value='sim',
        description='sim | realsense | external — who provides /camera/* topics'
    )
    # depth_topic 的默认值**随 camera_source 变**：选 realsense 就自动指向
    # 对齐后的话题。这不是锦上添花——若默认写死成 /camera/depth/image_raw，
    # 用户选了 realsense 却拿到**未对齐**深度，投影会带视差偏差且**不报错**，
    # 正是 D2 那个坑。显式传 depth_topic:=... 仍可覆盖。
    depth_topic_default = PythonExpression([
        "'" + REALSENSE_ALIGNED_DEPTH + "' if '",
        LaunchConfiguration('camera_source'),
        "' == 'realsense' else '" + SIM_DEPTH + "'",
    ])
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value=depth_topic_default,
        description='Depth image topic. Must be colour-ALIGNED (see D2). '
                    'Defaults to the aligned topic when camera_source:=realsense.'
    )
    tf_required_arg = DeclareLaunchArgument(
        'tf_required', default_value='false',
        description='true = perception_3d aborts if the camera→target TF is missing'
    )

    # 注意：不能写 LaunchConfiguration('camera_source') == 'realsense' ——
    # 那是**构造期**的 Python 比较，会立刻得到 bool，IfCondition 会报
    # "TypeError: 'bool' object is not iterable"。必须让比较在**求值期**发生。
    use_realsense = IfCondition(PythonExpression([
        "'", LaunchConfiguration('camera_source'), "' == 'realsense'",
    ]))

    # ---- 相机驱动（仅真机）----
    # align_depth.enable:=true 是 D2 的硬要求：把深度对齐到彩色，才能用彩色
    # 内参做针孔投影。不开它 /camera/aligned_depth_to_color/image_raw 不存在。
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        condition=use_realsense,
        parameters=[{
            'align_depth.enable': True,
        }],
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
            'depth_topic': LaunchConfiguration('depth_topic'),
            'tf_required': LaunchConfiguration('tf_required'),
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
        camera_source_arg,
        depth_topic_arg,
        tf_required_arg,
        realsense,
        detector,
        perception,
        planner,
    ])
