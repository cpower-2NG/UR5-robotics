"""
Launch 文件: 启动感知+抓取流水线 (不含 Gazebo)
用法:
  # 仿真：相机话题由 arm_grasp_sim 的 ros_gz_bridge 提供。
  # 一般**不直接调本文件**跑仿真——用 bringup.launch.py camera_source:=sim，
  # 它会先起世界与机械臂，再 include 本文件。直接调只适合「世界已在跑」的场合。
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
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
    # 夹爪后端的默认值**随 camera_source 变**，理由同 depth_topic：
    #   sim        → 夹爪就是 JTC 的第 7 个关节（Robotiq 2F-85），发轨迹即可
    #   realsense/ → 夹爪是**独立的 Modbus 设备**（大寰 AG-95），不在 JTC 里。
    #   external
    # 真机上若仍默认 sim，规划器会去命令一个**真机不存在的关节**
    # robotiq_85_left_knuckle_joint——JTC 收下这条轨迹却不动任何东西，
    # 于是「夹爪没反应」而不报错。故真机默认 ag95（其当前实现是**会大声报错的桩**，
    # 现场核实 Modbus 寄存器映射前不可用，这是刻意设计，见 grasp_planner/gripper.py）。
    gripper_backend_default = PythonExpression([
        "'sim' if '", LaunchConfiguration('camera_source'),
        "' == 'sim' else 'ag95'",
    ])
    gripper_backend_arg = DeclareLaunchArgument(
        'gripper_backend', default_value=gripper_backend_default,
        description='sim | ag95 — 夹爪后端。sim = JTC 第 7 关节；'
                    'ag95 = 独立 Modbus 设备（真机）。默认随 camera_source。'
    )
    # 感知单测 / 真机分阶段验收需要**只起感知、不起抓取**。
    # 这与 gripper_backend 的默认值是两个正交的问题：前者是「本轮验不验抓取」，
    # 后者是「抓取该用哪个夹爪」。若不提供这个开关，跑感知单测时就只能
    # 连带把 ag95 桩拉起来（它会按设计**立刻报错退出**），把日志搞脏。
    skip_detector_arg = DeclareLaunchArgument(
        'skip_detector', default_value='false',
        description='true = 不起 YOLO（喂外部 /detections，如自检脚本）'
    )
    skip_planner_arg = DeclareLaunchArgument(
        'skip_planner', default_value='false',
        description='true = 不起抓取规划器（只验感知）'
    )
    # 统一延时：仿真里这些节点要等 Gazebo 把 /clock、相机话题发出来，
    # 还要等 MoveIt 的 /compute_ik 与 JTC 的 action 就绪；真机上是 0。
    # 之所以是**一个**延时而不是 detector/planner 两档：本文件是节点的
    # **唯一定义处**，而「几秒起步」是仿真专属的调度细节，不该在这里长成
    # 两个可调旋钮。仿真侧传一个够用的值即可（见 arm_grasp_sim/full.launch.py）。
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='0.0',
        description='所有节点统一延迟启动（秒）。仿真传 18；真机保持 0'
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
        condition=UnlessCondition(LaunchConfiguration('skip_detector')),
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
    # 其余参数（几何、时序、IK 种子等）由 grasp_planner 自己的
    # config/grasp_planner.yaml 提供，planner_node 已内置同名默认值，
    # 故此处只需覆盖后端选择。
    planner = Node(
        package='grasp_planner',
        executable='planner_node',
        name='grasp_planner',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('skip_planner')),
        parameters=[{
            'gripper_backend': LaunchConfiguration('gripper_backend'),
        }],
    )

    return LaunchDescription([
        model_arg,
        conf_arg,
        device_arg,
        target_frame_arg,
        camera_source_arg,
        depth_topic_arg,
        tf_required_arg,
        gripper_backend_arg,
        skip_detector_arg,
        skip_planner_arg,
        start_delay_arg,
        TimerAction(
            period=LaunchConfiguration('start_delay'),
            actions=[realsense, detector, perception, planner],
        ),
    ])
