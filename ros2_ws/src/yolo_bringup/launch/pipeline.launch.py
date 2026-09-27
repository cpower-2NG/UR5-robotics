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

import os
import sys

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# realsense2_camera 开启对齐后，深度发在这个话题上（不是 /camera/depth/image_raw，
# 那个是**未对齐**的）。D2 要求用对齐深度配彩色内参，故真机走这个。
REALSENSE_ALIGNED_DEPTH = '/camera/aligned_depth_to_color/image_raw'
SIM_DEPTH = '/camera/depth/image_raw'


def _default_planner_params_file():
    """抓取规划器的默认参数文件；包没编出来时返回空串。

    板端「只编子集 + skip_planner:=true 只验感知」是 README 文档化的用法，
    那时 grasp_planner 可能不在。不要因此让整个 launch 起不来（与
    bringup.launch.py 对 arm_grasp_sim 的容错保持同一种态度），
    但**必须出声**——静默退回节点内置默认值正是本函数要防的那个 bug。
    """
    try:
        return os.path.join(
            get_package_share_directory('grasp_planner'),
            'config', 'grasp_planner.yaml',
        )
    except PackageNotFoundError:
        print(
            '[pipeline.launch.py] 警告: 找不到 grasp_planner 包，规划器将退回'
            '节点内置默认参数。若你正要跑抓取（不是 skip_planner:=true），'
            '这不是你想要的。',
            file=sys.stderr,
        )
        return ''


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
    # 抓取规划器的几何/时序/IK 参数**全部**在 grasp_planner/config/grasp_planner.yaml。
    # 真机上线必须能按现场实测覆盖它（`planner.launch.py:11`、
    # `docs/RESTRUCTURE_PLAN.md` 都这么要求），所以这里给出这条入口。
    planner_params_default = _default_planner_params_file()
    planner_params_arg = DeclareLaunchArgument(
        'planner_params_file', default_value=planner_params_default,
        description='抓取规划器参数文件。真机现场覆盖几何时指向自备的 yaml'
    )
    # 桌子的碰撞体**由谁打开**：grasp_planner 的 `table_scene_enabled`，
    # yaml 里默认 false（一个默认值不可能同时对上仿真和真机的桌子，而**错的
    # 碰撞体比没有更坏**——它可能把正确解全拒掉，看起来像 IK 坏了）。
    # 这里单开一个同名 launch 参数，让**后端自己声明**要不要那块桌子：
    #   仿真 → arm_grasp_sim/full.launch.py 传 true（它知道 grasp_world.sdf 的桌面尺寸）
    #   真机 → 暂不打开，等现场实测出台面尺寸再传 true（W2）
    # 之所以不直接写进 planner_params_file 指的那份 yaml：那份 yaml 是**共用**的，
    # 而这一个开关是**后端属性**，属于 launch 该管的事。
    table_scene_arg = DeclareLaunchArgument(
        'table_scene_enabled', default_value='false',
        description='true = 把工作台作为碰撞体发进 MoveIt 规划场景'
                    '（几何取 planner yaml 的 table_box_*）。仿真侧传 true'
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
    # ⚠️ 这里的 params 文件**必须显式加载**（D50）。
    # 曾经这里只传了 gripper_backend，而注释写着「其余参数由 grasp_planner.yaml
    # 提供」——**那句是假的**：planner_node 的 _declare_params 全是硬编码
    # d(name, default)，它自己不读 yaml；yaml 只被 grasp_planner 自己的
    # planner.launch.py 加载，而全仓库没有一处 include 它。
    # 之所以一直没暴露：那 34 条默认值与 yaml 的 35 条**逐条相同**，跑起来
    # 「碰巧是对的」。一旦按文档去改 yaml 覆盖真机几何，就会静默无效。
    # 若包不在（板端只编子集 + skip_planner），用空 dict 占位，不引用该参数。
    # ⚠️ 这里的 'true'/'false' 字符串会被 launch_ros 按 **yaml 规则**求值成
    # Python bool（launch_ros/utilities/evaluate_parameters.py:79 的
    # yaml.safe_load），正好对上节点里 `declare_parameter(..., False)` 声明出的
    # bool 类型。若哪天要传**字符串**参数，两边都得加引号。
    planner_params = [
        LaunchConfiguration('planner_params_file') if planner_params_default else {},
        {
            'gripper_backend': LaunchConfiguration('gripper_backend'),
            'table_scene_enabled': LaunchConfiguration('table_scene_enabled'),
        },
    ]
    planner = Node(
        package='grasp_planner',
        executable='planner_node',
        name='grasp_planner',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('skip_planner')),
        parameters=planner_params,
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
        planner_params_arg,
        table_scene_arg,
        start_delay_arg,
        TimerAction(
            period=LaunchConfiguration('start_delay'),
            actions=[realsense, detector, perception, planner],
        ),
    ])
