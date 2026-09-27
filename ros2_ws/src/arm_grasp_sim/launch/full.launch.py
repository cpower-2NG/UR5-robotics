"""仿真侧端到端抓取演示：世界 + UR5 + MoveIt + 感知 + 抓取规划。

分工（**单一真源**，见 docs/RESTRUCTURE_PLAN.md 的 P3-6）
=====================================================
本文件**只加仿真专属的东西**：Gazebo 世界、UR5+夹爪、控制器、MoveIt。
感知与抓取节点一律由 ``yolo_bringup/pipeline.launch.py`` 提供——
detector / perception_3d / grasp_planner 的参数**只在那一处定义**。

为什么这样拆（第九轮，用户确认）
--------------------------------
此前 detector/perception 的参数在本文件与 pipeline.launch.py 里**各写了一份**，
改一处忘一处就会让「仿真能跑、真机不跑」或反之。真机用的正是
pipeline.launch.py，所以它的那份必须是唯一的一份。

数据流：
  yolo_detector/detector_node → /detections
  perception_3d/projector_node → /targets_3d   （全部候选，含 id/类别/分数）
  grasp_planner/planner_node  ← 只消费 /targets_3d，自己不做任何投影

用法：
  ros2 launch arm_grasp_sim full.launch.py
  ros2 launch arm_grasp_sim full.launch.py skip_planner:=true    # 只起仿真+感知，调试用
  ros2 launch arm_grasp_sim full.launch.py headless:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("arm_grasp_sim")
    bringup = get_package_share_directory("yolo_bringup")

    model_arg = DeclareLaunchArgument(
        "model", default_value="block_best.pt",
        description="YOLO 权重名（由 detector_node 按 $YOLO_MODEL_DIR → CWD 逐级向上 "
                    "→ 包内 models/ 解析；也可直接给绝对路径）",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="cuda:0",
        description="推理设备：cpu | cuda:N",
    )
    skip_detector_arg = DeclareLaunchArgument(
        "skip_detector", default_value="false",
        description="true = 不起 YOLO（喂外部 /detections，如自检脚本）",
    )
    skip_planner_arg = DeclareLaunchArgument(
        "skip_planner", default_value="false",
        description="true = 不起抓取规划器（只测感知）",
    )
    # 一个延时覆盖全部上层节点。仿真里它要同时满足两件事：
    #   1) Gazebo 已把 /clock 与相机话题发出来；
    #   2) MoveIt 的 /compute_ik 与 JTC 的 action 已就绪。
    # 取两者中较大的那个即可，故用单值而非两档（原先的 14/18 两档已合并为 18）。
    # 注意这里**不是**越长越安全：延时过长只是启动慢，延时不足才会让规划器
    # 对着还没起来的控制器发轨迹（它会报错并降级继续，见 P3-6 的说明）。
    start_delay_arg = DeclareLaunchArgument(
        "start_delay", default_value="18.0",
        description="上层节点（YOLO/感知/规划）的统一延迟启动秒数",
    )

    # ---- 仿真专属：世界 + 机械臂 + 控制器 + MoveIt ----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )

    # ---- 上层节点：**不在本文件定义**，全部复用主线 ----
    # camera_source:=sim 会同时决定：深度话题用仿真那条、夹爪后端用 sim
    # （夹爪后端随 camera_source 走，见 pipeline.launch.py 的说明）。
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "pipeline.launch.py")
        ),
        launch_arguments={
            "camera_source": "sim",
            "model": LaunchConfiguration("model"),
            "device": LaunchConfiguration("device"),
            "skip_detector": LaunchConfiguration("skip_detector"),
            "skip_planner": LaunchConfiguration("skip_planner"),
            "start_delay": LaunchConfiguration("start_delay"),
        }.items(),
    )

    return LaunchDescription([
        model_arg,
        device_arg,
        skip_detector_arg,
        skip_planner_arg,
        start_delay_arg,
        DeclareLaunchArgument("headless", default_value="false"),
        sim,
        pipeline,
    ])
