"""
Launch 文件: 单独启动 3D 感知节点

⚠️ 本文件**不在主链路上**：全仓库没有一处 include 它（`setup.py` 只是把它装进
`share/`）。仿真与真机的唯一入口是 `yolo_bringup`（`bringup.launch.py` →
`pipeline.launch.py`），**`max_sync_dt` 等参数只在 `pipeline.launch.py` 一处定义**
（那里给了真机放宽值 0.3，见该文件 max_sync_dt_arg 的注释）。
本文件的存在意义只是「把投影节点单独拉起来调」；**改这里的默认值对主链路无效** ——
现场要调参数请走唯一入口，或命令行直通。参数面已经与主入口对齐（主入口声明了
全部 11 个参数），所以两边不会再出现「一个有、一个没有」。

用法:
  ros2 launch perception_3d perception.launch.py target_frame:=base_link
  # 真机（RealSense D435i，深度为 16UC1 毫米，由 encoding 自动判定）：
  #   注意真机必须用**与彩色对齐**的深度话题，否则彩色内参配未对齐深度会有视差偏差（D2）
  ros2 launch perception_3d perception.launch.py target_frame:=base_link tf_required:=true \
      depth_topic:=/camera/aligned_depth_to_color/image_raw
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
            'depth_median_window', default_value='5',
            description='Neighbourhood median window inside a sample point (denoise)'
        ),
        DeclareLaunchArgument(
            'depth_buffer_len', default_value='10',
            description='Depth ring buffer length (timestamp pairing)'
        ),
        DeclareLaunchArgument(
            'tf_required', default_value='false',
            description='true = abort if the camera→target TF is unavailable'
        ),
        # 以下三个话题名在第七轮参数化，以便与 pipeline.launch.py 的
        # camera_source 开关配合（真机要用**对齐后**的深度话题，见 D2）。
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='Depth image topic. Must be colour-ALIGNED on the real robot '
                        '(D435i: /camera/aligned_depth_to_color/image_raw)'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/color/camera_info',
            description='CameraInfo topic supplying fx/fy/cx/cy'
        ),
        DeclareLaunchArgument(
            'detections_topic', default_value='/detections',
            description='vision_msgs/Detection2DArray input topic'
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
            'depth_median_window': LaunchConfiguration('depth_median_window'),
            'depth_buffer_len': LaunchConfiguration('depth_buffer_len'),
            'tf_required': LaunchConfiguration('tf_required'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
        }],
    )

    return LaunchDescription(declared_args + [projector_node])
