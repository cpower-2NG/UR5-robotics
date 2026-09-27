"""Launch 文件: 单独启动抓取规划节点。

⚠️ 本文件**不在主链路上**：全仓库没有一处 include 它（`setup.py` 只是把它装进
`share/`）。仿真与真机的唯一入口是 `yolo_bringup`（`bringup.launch.py` →
`pipeline.launch.py`），yaml 也由那里加载（`planner_params_file`，D50 的修法）。
本文件的存在意义只是「把规划节点单独拉起来调」；**改这里的默认值对主链路无效**。

用法:
  ros2 launch grasp_planner planner.launch.py
  ros2 launch grasp_planner planner.launch.py gripper_backend:=ag95

过去的 ``mode:=mock|moveit`` 已经取消：mock 分支只会打日志不做事，
留着容易让人以为抓取跑通了。现在只有一条实现路径，仿真/真机的差别
由 ``gripper_backend`` 和后端参数决定（见 grasp_planner/gripper.py）。

参数默认值全部来自 config/grasp_planner.yaml，**真机几何必须按现场实测覆盖**。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("grasp_planner"), "config", "grasp_planner.yaml"
    )

    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="参数文件（真机上线时指向现场的 yaml）",
    )
    backend_arg = DeclareLaunchArgument(
        "gripper_backend", default_value="sim",
        description="sim = 仿真 Robotiq 2F-85（JTC 关节）| ag95 = 真机大寰 AG-95（Modbus，尚未实现）",
    )

    planner_node = Node(
        package="grasp_planner",
        executable="planner_node",
        name="grasp_planner",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"gripper_backend": LaunchConfiguration("gripper_backend")},
        ],
    )

    return LaunchDescription([
        params_arg,
        backend_arg,
        planner_node,
    ])
