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
  camera_source:=realsense   → real.launch.py
                                 ├─ ur_robot_driver     驱动 + 控制器
                                 ├─ ur_moveit_config    move_group（/compute_ik）
                                 ├─ 静态 TF             base_link → camera_link
                                 └─ pipeline.launch.py  感知与抓取节点
  camera_source:=external    → pipeline.launch.py
                                 （相机与机械臂都由外部提供，本文件只起上层节点）

即：**detector / perception_3d / grasp_planner 的参数只在
`pipeline.launch.py` 一处定义**，仿真与真机共用同一份。这是刻意的——
两边各写一份的话，改一处忘一处就会出现「仿真能跑、真机不跑」，而真机
恰恰是最难排查的那个。

⚠️ 真机入口在 2026-09-27 之前**只起 pipeline.launch.py**（D45）：没有驱动、
没有 MoveIt、没有控制器、没有相机 TF。那样起来的规划器解不出任何 IK，
却也不报错——「真机链路」当时在代码里基本不存在。现在它由 `real.launch.py`
补齐。**`robot_ip` 在真机分支是必填项**，这里透传下去。

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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# ── 真机细分项：**不在本文件声明，靠命令行直通** ──────────────────────
# `robot_ip` / `ur_type` / `safety_limits` / `start_ur_driver` / `start_moveit` /
# `start_pipeline` / `publish_camera_tf` / `camera_x..camera_qw` 这些**真机专属**
# 参数只在 `real.launch.py` 里声明，本文件**故意不声明它们**。
#
# ⚠️ 为什么不在这里声明（这一条是实测纠正过来的，原先我写反了）
# ============================================================
# 曾以为「未声明的参数覆盖会被 launch 拒绝」，于是把它们也声明了一遍。实测
# （Humble，2026-09-27）：`ros2 launch pkg file.launch.py 未声明的名字:=值`
# **不报错、被静默接受**，值照常进 `launch_configurations`。
#
# 而 launch_configurations 是**整棵树共享**的：本文件声明过的参数，会顺着
# include 链一路传进 `full.launch.py` → `pipeline.launch.py`，而下游的
# `DeclareLaunchArgument` 见到「已设置」就**不再应用自己的默认值**。
# 于是「在唯一入口为真机声明 gripper_backend 默认 none」这一个动作，把**仿真
# 路径**也改成了 none —— 仿真跑出来是「只走位、不夹持」，且不报错。
# （这一版真就发出去了，被 skip_detector 冒烟测试当场抓到：规划器日志写着
# 「夹爪后端=none … 动作模式=只走位」。）
#
# 所以这里的取舍是「响亮失败 vs 静默失败」：
#   · 不声明（本文件的现状）：万一将来 launch 不再直通未声明参数，症状是
#     `robot_ip` 没传进去 → `real.launch.py` 抛「robot_ip 是必填项」→ **响亮**；
#   · 声明：默认值悄悄改掉仿真路径 → **静默**，且只有跑仿真才会发现。
# 两者都可能出问题，取响亮的那个。
#
# 代价：`--show-args` 不再列出真机细分项。故真机用法在这里写清楚——
#   ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
#       robot_ip:=<控制箱 IP> [publish_camera_tf:=true camera_x:=.. camera_qw:=..]
#   （完整清单：ros2 launch yolo_bringup real.launch.py --show-args）


def _sim_backend(context):
    """在**执行期**构造仿真后端；不选 sim 就什么都不做。

    ⚠️ 为什么必须用 OpaqueFunction，而不是直接写
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


def _real_backend(context):
    """在**执行期**构造真机/外部后端；`camera_source:=sim` 时什么都不做。

    两个非仿真分支的差别：
      realsense → `real.launch.py`：驱动 + MoveIt + 相机 TF + 上层节点
      external  → `pipeline.launch.py`：**只起上层节点**（相机与机械臂都由
                  外部提供，比如已有 rosbag 或别人的驱动）

    ⚠️ `external` 那一支**必须留着**。曾把它并进 realsense 分支（`!= 'sim'`
    就 include real.launch.py），于是 `camera_source:=external` 会去起 UR 驱动
    ——而 external 的全部意义就是「别碰机械臂」。反过来，写成
    `== 'realsense'` 就返回、`external` 什么都不起，则是**静默的空启动**：
    命令成功、日志干净、什么也没跑。两条岔路都得显式写出来。

    与 `_sim_backend` 同一个理由用 OpaqueFunction：real 分支要 include
    `ur_robot_driver` / `ur_moveit_config` 的 launch 文件，而解析期求值
    `get_package_share_directory` 会让「工作站上没装这两个包」连
    `camera_source:=sim` 或 `--show-args` 都跑不起来。
    """
    source = context.launch_configurations.get('camera_source', 'sim')
    if source == 'sim':
        return []

    common = {
        'model': LaunchConfiguration('model'),
        'device': LaunchConfiguration('device'),
        'target_frame': LaunchConfiguration('target_frame'),
        'tf_required': LaunchConfiguration('tf_required'),
    }

    if source == 'external':
        # 顶层命令行的其余参数（如 skip_detector）不必在这里转 —— 它们已在
        # launch_configurations 里，会被 include 进来的文件直接读到（见文件头
        # 关于「直通」的说明）。
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('yolo_bringup'),
                        'launch', 'pipeline.launch.py',
                    )
                ),
                launch_arguments={
                    **common, 'camera_source': 'external',
                }.items(),
            ),
        ]

    # realsense：真机细分项（robot_ip 等）由 real.launch.py 自己声明并取默认值；
    # 命令行给的会直通到它，这里不必也不该再声明一遍（见文件头的说明）。
    # 也不必传 camera_source：real.launch.py 不读它，它 include pipeline 时
    # 自己写死 'realsense'。
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch', 'real.launch.py',
                )
            ),
            launch_arguments=common.items(),
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
    # 默认值**随 camera_source 变**，理由同 pipeline.launch.py 的 depth_topic /
    # ik_group：真机上「查不到 TF」几乎一定是配置错（相机外参没发、帧名不对），
    # 那时发一堆坐标算错的目标比直接退出危险得多；仿真里 TF 一定在，true 只是
    # 让一次笔误变成启动失败。固定默认值必然有一边是错的。
    #
    # ⚠️ 这里还有个**必须知道**的耦合：本文件声明过的参数会顺着 include 链
    # 传下去（launch_configurations 是整棵树共享的），而下游见到「已设置」
    # 就不再应用自己的默认值。所以本文件给 tf_required 的默认值，就是
    # `pipeline.launch.py` 和 `real.launch.py` 实际拿到的值。详见文件头。
    tf_required_default = PythonExpression([
        "'false' if '", LaunchConfiguration('camera_source'),
        "' == 'sim' else 'true'",
    ])
    tf_required_arg = DeclareLaunchArgument(
        'tf_required', default_value=tf_required_default,
        description='true = 查不到 TF 就让 perception_3d 报错退出。'
                    '默认随 camera_source：仿真 false、真机/外部 true',
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='仅 camera_source:=sim 有效：true = 不起 Gazebo GUI',
    )
    sim_start_delay_arg = DeclareLaunchArgument(
        'sim_start_delay', default_value='18.0',
        description='仅 camera_source:=sim 有效：上层节点延迟启动秒数',
    )
    # 真机专属参数（robot_ip 等）**不在这里声明** —— 理由见文件头。真机用法：
    #   ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
    #       robot_ip:=<控制箱 IP>

    camera_source = LaunchConfiguration('camera_source')

    # ---- 真机/外部后端：不起任何仿真 ----
    # 用的是**执行期**构造（见 _real_backend），而不是像先前那样直接写
    # IncludeLaunchDescription(...)：real 分支现在也要 include 别的包的
    # launch 文件，而解析期求值 get_package_share_directory 会让「本机没装
    # ur_robot_driver」连 `--show-args` 都跑不起来（与 _sim_backend 同一个坑）。
    #
    # 真机上先确认驱动与控制器就绪，再启动本文件 —— **规划器不会等
    # ur_robot_driver**：控制器缺席时它报错并降级继续（见 P3-6），
    # 于是一串「目标解不出来」的日志会被误读成感知坏了。
    real_launch = OpaqueFunction(function=_real_backend)

    return LaunchDescription([
        model_arg,
        device_arg,
        camera_source_arg,
        target_frame_arg,
        tf_required_arg,
        headless_arg,
        sim_start_delay_arg,
        # 两个后端都按需构造（见各自的说明）
        OpaqueFunction(function=_sim_backend),
        real_launch,
    ])
