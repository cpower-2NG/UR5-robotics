"""
Launch 文件: 唯一入口 —— 按 camera_source 决定整车跑在哪个后端
用法:
  # 仿真：起 Gazebo 世界 + UR5 + MoveIt + 感知 + 抓取
  ros2 launch yolo_bringup bringup.launch.py camera_source:=sim

  # 真机：**不起 Gazebo**，由 realsense2_camera 提供 /camera/*
  ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense

  # 相机由外部提供（已有 rosbag / 别的驱动）
  ros2 launch yolo_bringup bringup.launch.py camera_source:=external

`camera_source` 同时决定**要不要起仿真**：真机上再起一个仿真世界既浪费
算力，又会让 /clock、/tf、相机话题和真机抢同一个话题名（D2/D7 那类静默算错
的温床）。所以这不是「相机的来源」一个参数，而是「整车跑在哪个后端」。

分工（单一真源）
================
  camera_source:=sim         → arm_grasp_sim/full.launch.py
                                 ├─ sim.launch.py       世界 + UR5 + 控制器 + MoveIt
                                 └─ pipeline.launch.py  感知与抓取节点
  camera_source:=realsense   → pipeline.launch.py
  camera_source:=external       （同上）

即：**detector / perception_3d / grasp_planner 的参数只在
`pipeline.launch.py` 一处定义**，仿真与真机共用同一份。这是刻意的——
两边各写一份的话，改一处忘一处就会出现「仿真能跑、真机不跑」，而真机
恰恰是最难排查的那个。

（历史：本文件曾有一支 `simulation.launch.py`——一个**没有机械臂**的世界。
抓取链路接上后那一支已无意义：规划器起来了却没有臂可动。第九轮删除。）
"""

import os
from ament_index_python.packages import (
    get_package_share_directory, PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _sim_backend(context):
    """在**执行期**构造仿真后端；不选 sim 就什么都不做。

    ⚠️ 为什么必须用 OpaqueFunction，而不是像 real 分支那样直接写
    `IncludeLaunchDescription(..., condition=sim_only)`：
    launch 在**解析期**就会求值 `get_package_share_directory('arm_grasp_sim')`，
    而 `IfCondition` 只影响**执行**、不影响解析——于是「没编 arm_grasp_sim」
    会让 `camera_source:=realsense` 也**整体加载失败**。

    这不是假想：P4-7「真机感知单独验收」的正常做法就是在板上只编子集
    （`colcon build --packages-select yolo_bringup perception_3d ...`），
    而 Orin 上本来就不需要任何仿真资产。缺包时给出**可操作**的报错即可。
    """
    if context.launch_configurations.get('camera_source', 'sim') != 'sim':
        return []

    try:
        full_launch = os.path.join(
            get_package_share_directory('arm_grasp_sim'),
            'launch', 'full.launch.py',
        )
    except PackageNotFoundError:
        raise RuntimeError(
            "camera_source:=sim 需要 arm_grasp_sim 包，但当前环境里找不到它。\n"
            "  补法：在 ros2_ws 下执行\n"
            "      colcon build --packages-select arm_grasp_sim\n"
            "  再重新 source ros2_ws/install/setup.bash。\n"
            "  若你**不打算跑仿真**（真机部署就是如此），改用\n"
            "      ros2 launch yolo_bringup bringup.launch.py "
            "camera_source:=realsense\n"
            "  仿真资产只在工作站需要，Orin 上不装也不影响真机链路。"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={
                'model': LaunchConfiguration('model'),
                'device': LaunchConfiguration('device'),
                'headless': LaunchConfiguration('headless'),
                'start_delay': LaunchConfiguration('sim_start_delay'),
            }.items(),
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_bringup')

    # ---- 参数 ----
    # 注意：**没有 `world` 参数**了。世界由 arm_grasp_sim/sim.launch.py 拥有
    # （它必须与那份 URDF/SRDF/控制器配置成套，单独换一个 world 会对不上臂）。
    model_arg = DeclareLaunchArgument(
        'model', default_value='block_best.pt',
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
    )
    camera_source_arg = DeclareLaunchArgument(
        'camera_source', default_value='sim',
        description='sim | realsense | external — 谁提供 /camera/* 话题，'
                    '以及是否启动仿真',
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='目标输出坐标系（真机上通常是机械臂基座）',
    )
    tf_required_arg = DeclareLaunchArgument(
        'tf_required', default_value='false',
        description='true = 查不到 TF 就让 perception_3d 报错退出（真机建议 true）',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='仅 camera_source:=sim 有效：true = 不起 Gazebo GUI',
    )
    sim_start_delay_arg = DeclareLaunchArgument(
        'sim_start_delay', default_value='18.0',
        description='仅 camera_source:=sim 有效：上层节点延迟启动秒数',
    )

    camera_source = LaunchConfiguration('camera_source')

    # 注意：比较必须发生在**求值期**。写 camera_source == 'sim' 是构造期的
    # Python 比较，会立刻得到 bool，IfCondition 不接受。
    real_only = IfCondition(PythonExpression([
        "'", camera_source, "' != 'sim'",
    ]))

    # ---- 真机后端：不起任何仿真，上层节点立即起 ----
    # start_delay 保持默认 0：真机没有可等的东西。但要清楚**规划器不会等
    # ur_robot_driver**——它在控制器缺席时会报错并降级继续（见 P3-6 的说明），
    # 所以真机上应先确认驱动与控制器就绪，再启动本文件。
    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'pipeline.launch.py')
        ),
        condition=real_only,
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'camera_source': camera_source,
            'target_frame': LaunchConfiguration('target_frame'),
            'tf_required': LaunchConfiguration('tf_required'),
        }.items(),
    )

    return LaunchDescription([
        model_arg,
        device_arg,
        camera_source_arg,
        target_frame_arg,
        tf_required_arg,
        headless_arg,
        sim_start_delay_arg,
        # 仿真后端按需构造（见 _sim_backend 的说明）
        OpaqueFunction(function=_sim_backend),
        real_launch,
    ])
