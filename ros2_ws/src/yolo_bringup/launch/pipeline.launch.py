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
对上层节点 yolo_detector / perception_3d / grasp_planner 而言**话题名一致**，
故它们的参数不变。

⚠️ 「话题名一致」不是天生的，是**本文件保证**的：仿真的 ros_gz_bridge 同名
直通，而真机 realsense2_camera 会多一层 `/camera/camera/...`，靠这里的一组
remap 收回单层（实测与理由见下方 REALSENSE_REMAPS）。**真机与仿真的差异
只在本文件里被吸收**，别把这一层泄漏到下游节点的默认值里。
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

# ── 真机上相机话题名多一层（实测）───────────────────────────────────────
# 本文件头写着「三者对上层节点完全透明——话题名一致」。仿真那一半成立
# （ros_gz_bridge 的同名直通，见 config/ros_gz_bridge.yaml），
# **但真机那一半原先不成立**：realsense2_camera 的话题是
# `<camera_namespace>/<camera_name>/...`，两个默认值都是 `camera`，
# 于是它发的是 `/camera/camera/color/image_raw` —— 比下游期望的多一层。
#
# 2026-09-27 在板子（Orin，D435i 实物）上 `ros2 topic list` 实测：
#     /camera/camera/color/image_raw
#     /camera/camera/aligned_depth_to_color/image_raw
#     /camera/camera/color/camera_info
# 而下游三处都写死成单层：
#     detector_node 的彩色订阅（硬编码 `/camera/color/image_raw`）
#     projector_node 的 `camera_info_topic` 默认 `/camera/color/camera_info`
#     本文件的 REALSENSE_ALIGNED_DEPTH
#
# 失效症状是**静默的**：检测器一帧都收不到 → 没有 /detections → 规划器
# 一直没有目标，日志里只有「没目标」，看起来像「检测器坏了」或「模型不行」。
# 所以在这里**一次性**把话题名改回单层，而不是去改三个下游节点的默认值：
# 后者要动 detector_node 的硬编码订阅，且真机/仿真的默认值会分叉。
#
# ⚠️ `camera_namespace:=` 是**改不掉的**：实测传 `-p camera_namespace:=""`
# 被 rcl 直接拒（`RCLInvalidROSArgsError`），用 params 文件传空串虽被接受
# 但**话题前缀不变**。所以 remap 是这里唯一可行的做法。
REALSENSE_REMAPS = [
    ('/camera/camera/color/image_raw', '/camera/color/image_raw'),
    ('/camera/camera/aligned_depth_to_color/image_raw', REALSENSE_ALIGNED_DEPTH),
    ('/camera/camera/color/camera_info', '/camera/color/camera_info'),
]

# ── IK 组名与轨迹 action：真机与仿真**不同名** ─────────────────────────
# 组名是 SRDF 里的**派生值**（`${prefix}${name}_manipulator`），不是固定串：
#     仿真  ur5_gripper.srdf.xacro 的 name:=ur5  → ur5_manipulator
#     真机  ur_moveit.launch.py **硬编码** name:=ur → ur_manipulator
# 两边各自的 kinematics.yaml 与自己的 SRDF 对齐，所以 grasp_planner.yaml 里
# 写的 `ur5_manipulator` 在仿真里是**对的**，真机必须覆盖。
# 官方那行还留了句注释解释为什么不用 ur_type：「then the planning group
# names in yaml configs has to be updated!」
#
# 轨迹 action 同理：`joint_trajectory_controller` 在真机驱动里存在，但官方
# 默认起的是 **scaled_** 那个（ur_control.launch.py:581-585），它尊重示教器
# 速度滑条 —— 对第一次真机动作是明显更安全的默认。真机若不覆盖，action
# 会指向一个**没起来的控制器**，症状是「找不到 action server」。
#
# `ik_link: tool0` 两侧一致（官方 SRDF 的 tip_link 就是 tool0），无需覆盖。
SIM_IK_GROUP = 'ur5_manipulator'
REAL_IK_GROUP = 'ur_manipulator'
SIM_TRAJECTORY_ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
REAL_TRAJECTORY_ACTION = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
# tool0 → 指尖的距离（m）。**和上面两项一样是后端属性**，理由完全一样：
# 仿真用 Robotiq 2F-85（0.13 是从参考项目 `arm-grasping` 抄来的 2F-85 推导值，
# 两处同一个常数），真机用 AG-95 —— **指尖长度不同**。
#
# 真机值 **0.174** 的来源：示教器上 4 点法标出的 `TCP_1` 的 z = **173.99 mm**
#（x = 47.46、y = 6.8，rpy 全 0 —— 姿态为零所以三个数就是法兰系里的偏移，
# 不用换算）。x ≈ 47.5 = 半行程，说明碰的是**指尖角**，即 z 正是法兰→指尖。
# ⚠️ 它依赖「夹爪停在标定时那个开度（全开）」，**驱动一次夹爪这个值就失效**。
#
# 🔴 **示教器上现在配着的那个 `TCP_1` 不是这个数** —— 后来为了 ② 的点探法，
# 在指面上**绑了针尖**重标了一轮，得到 z = **194.55 mm**（x 44.54 / y −11.36）。
# 那是**针尖**的距离，**不是指尖的**。两者差 27.6 mm，正好是针相对指尖角的几何。
#   · 想从针尖那轮推 `tool0_to_fingertip`：**要减掉针尖伸出量**
#     （`194.55 − 伸出量`，伸出量用卡尺量那一小截 20 mm 即可，比沿 174 mm 量准得多）；
#   · **千万别把 194.55 填进来** —— 那会让指尖落在该到位置**上方** 20.6 mm，抓空。
# 详见 `docs/待执行命令.md` §丙-2。
#
# 差 44 mm 的后果不是「差一点」，是**方向确定地往桌子里扎**：
#   planner_node.py 把**法兰**放在 `wz + tool0_to_fingertip`（`:1288` 接近点、
#   `:1317` 抓取点），好让指尖落到 `wz`。于是指尖实际落在 **`wz` − 44 mm**。
#   而 `wz` 是**感知给的点**（`projector_node.py:382` 用检测框中心那条视线
#   配深度）—— 它打在方块**可见面上**，所以最高不过方块顶（40 mm 方块 ⇒ 台面上方
#   ≤ 40 mm）。**⇒ 指尖必然落到台面以下**，深浅看 `wz` 打在哪个高度：
#     打在方块顶 ⇒ 入台 4 mm；打在方块中心 ⇒ 入台 24 mm；打在台面 ⇒ 入台 44 mm。
#   接近点（approach_z）同样整体低 44 mm。
# ⚠️ 这个数**尚未经 Δ 验收**（`scripts/tcp_check.py`）：同一笔点换 ≥30° 的姿态
#   再碰一次，Δ ≤ 1 mm 才算数。验收没过就要重标、这个值跟着改。
SIM_TOOL0_TO_FINGERTIP = '0.13'
REAL_TOOL0_TO_FINGERTIP = '0.174'


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


def _default_realsense_params_file():
    """真机 RealSense 的参数文件路径；包没编出来时返回空串。

    这个文件存在的**唯一**理由是 `serial_no` 的引号（理由见
    generate_launch_description 里那段长注释）。丢了它**不会报错** ——
    驱动会退回「枚举到的第一台」，插着两台相机时就是**随机**认一台。
    所以这里必须出声，不能静默。
    """
    try:
        return os.path.join(
            get_package_share_directory('yolo_bringup'),
            'config', 'realsense_camera.yaml',
        )
    except PackageNotFoundError:
        print(
            '[pipeline.launch.py] 警告: 找不到 yolo_bringup 包，无法钉死相机'
            '序列号（config/realsense_camera.yaml）。插着两台 D435i 时，'
            '驱动会**静默**取枚举到的第一台。',
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
    # ── 相机认亲：钉死是哪一台 D435i ─────────────────────────────────────
    # ⚠️ 序列号**不在这个文件里传**，在 `config/realsense_camera.yaml` ——
    #    只有那样才传得过去。这是一个**两侧判断相反**的坑，完整推导见那份
    #    yaml 的文件头，这里只留结论：
    #
    #      launch 用 PyYAML 写参数字典，PyYAML 认为 `<topcam-serial>` 是**普通
    #      字符串**，于是**不加引号**地落盘；而节点侧 rcl 的 **C 版** YAML
    #      解析器把没引号的全数字读成**数字**（全 0-7 → 八进制 int；含 8/9 →
    #      double），节点报 `invalid type {double}` 并**退出**。
    #      ⇒ launch 这条路对「纯数字串」是**结构性**走不通的。
    #
    #    两条看着该管用、实测都救不了的路（D60 踩过，别再试）：
    #      · `ParameterValue(..., value_type=str)` —— 落盘仍是无引号。
    #      · 命令行 `camera_serial:="'014…'"` —— 引号被**原样保留**进值里，
    #        驱动拿着带引号的串去找设备，报 NOT found。
    #
    #    本机两台 D435i（腕部 + 俯视）各报两个号，`serial_no` 认 **SDK 号**：
    #        俯视  <topcam-serial> (ASIC <topcam-asic-serial>)
    #        腕部  <wristcam-serial> (ASIC <wristcam-asic-serial>)
    #    用 ASIC 号**不报错退出**，而是每 6 秒重试一次 ⇒ 「进程都在、永远没有
    #    图像」；且驱动做**字符串精确比较、不支持正则**（实测 `.*22072963`
    #    被当字面量）。详见那份 yaml 与 `docs/RESTRUCTURE_PLAN.md` 本轮记录。
    realsense_params_default = _default_realsense_params_file()
    realsense_params_arg = DeclareLaunchArgument(
        'realsense_params_file', default_value=realsense_params_default,
        description='真机 RealSense 参数文件（含**带引号**的 serial_no）。'
                    '换相机改它，别用命令行传 serial_no —— 纯数字串在 launch '
                    '里传不过去，理由见本文件注释与 config/realsense_camera.yaml'
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
    # max_sync_dt：检测↔深度帧的最大时间差，超过就**丢弃整批检测**（不报错退出）。
    # ⚠️ 这一项原先**到不了**：`projector_node` 认得它（默认 0.1），但只有
    # `perception_3d/launch/perception.launch.py` 暴露它，而那个文件**全仓库没有
    # 一处 include**（setup.py 只把它装进 share/）—— 于是现场无论怎么传都无效，
    # 正是 D50 那一类「参数定义在一个没人走的入口上」。这里补上唯一入口的定义。
    #
    # 为什么真机默认放宽到 0.3：窗口太紧的症状是**看不见目标**（日志里每 5 s 才
    # 一条「丢弃本批检测」，很像检测器坏了），而真机的相机/桥接比仿真更容易抖动。
    # 放宽到 0.3 不会换来错误坐标：相机静止、方块静止时，晚 0.3 s 的深度帧与当前
    # 帧**逐像素相同**（深度是场景几何的函数，不随采样时刻变），所以「配对到旧帧」
    # 对静止物体是无害的；它仍然拦得住真正的失步（流停了、时间戳对不上）。
    # 仿真保持 0.1 不动 —— 这是拿到 9/9 那套结果时用的值，不为了让日志干净去改它。
    max_sync_dt_default = PythonExpression([
        "'0.1' if '", LaunchConfiguration('camera_source'), "' == 'sim' else '0.3'",
    ])
    max_sync_dt_arg = DeclareLaunchArgument(
        'max_sync_dt', default_value=max_sync_dt_default,
        description='检测↔深度 最大时间差（秒），超过则丢弃整批检测。'
                    '默认：仿真 0.1 / 真机 0.3。设小了症状是「看不到目标」'
    )
    # ── 3D 感知的采样/配准旋钮：也在唯一入口上给全 ─────────────────────
    # 规则（本文件对**它自己起的三个节点**都适用）：**传给节点的每个参数，
    # 都要在本文件里声明**。不声明的话现场传了 `depth_grid:=7` 会**静默无效** ——
    # 未声明的参数会被 launch **接受**并塞进共享的 `launch_configurations`（实测），
    # 只是没有任何人读它，于是「我明明传了」与「一点反应都没有」同时成立。
    # 默认值一律**照抄节点 `declare_parameter` 的值**（改了就是静默改行为）；
    # 下面 7 个只有真机才需要动，仿真的值全部与节点默认值相同。
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale', default_value='-1.0',
        description='深度 像素→米 的尺度。-1 = 按 msg.encoding 自动判定'
                    '（16UC1→0.001 / 32FC1→1.0）。非标准编码的深度源才需要强制'
    )
    depth_grid_arg = DeclareLaunchArgument(
        'depth_grid', default_value='5',
        description='检测框内的采样网格边长（5 → 5×5）。真机深度更脏时可调'
    )
    depth_cluster_tol_arg = DeclareLaunchArgument(
        'depth_cluster_tol', default_value='0.03',
        description='取「最近深度簇」的容差（米），抗框内背景干扰'
    )
    depth_median_window_arg = DeclareLaunchArgument(
        'depth_median_window', default_value='5',
        description='每个采样点的邻域中值窗口边长（去深度噪点）'
    )
    depth_buffer_len_arg = DeclareLaunchArgument(
        'depth_buffer_len', default_value='10',
        description='深度帧环形缓冲长度（按时间戳就近配对用）'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info',
        description='提供 fx/fy/cx/cy 的 CameraInfo 话题（真机与仿真同名）'
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic', default_value='/detections',
        description='vision_msgs/Detection2DArray 输入话题'
    )
    # 夹爪后端的默认值**随 camera_source 变**，理由同 depth_topic：
    #   sim        → 夹爪就是 JTC 的第 7 个关节（Robotiq 2F-85），发轨迹即可
    #   realsense/ → 夹爪是**独立的 Modbus 设备**（大寰 AG-95），不在 JTC 里。
    #   external
    # 真机上若仍默认 sim，规划器会去命令一个**真机不存在的关节**
    # robotiq_85_left_knuckle_joint——JTC 收下这条轨迹却不动任何东西，
    # 于是「夹爪没反应」而不报错。故真机默认 ag95（它**会大声报错**：后端一构造就去
    # 开串口，板上没适配器时抛 RuntimeError，见 grasp_planner/gripper.py）——
    # 这里取的是「响亮失败」，不是「能跑」。⚠️ 只起上层节点时（external）记得传
    # gripper_backend:=none，否则规划器启动即死。
    gripper_backend_default = PythonExpression([
        "'sim' if '", LaunchConfiguration('camera_source'),
        "' == 'sim' else 'ag95'",
    ])
    gripper_backend_arg = DeclareLaunchArgument(
        'gripper_backend', default_value=gripper_backend_default,
        description='sim | ag95 | none — 夹爪后端。sim = JTC 第 7 关节；'
                    'ag95 = 独立 Modbus 设备（真机）；none = 不驱动夹爪，'
                    '只配 carry_objects:=false。默认随 camera_source。'
    )
    # IK 组名与轨迹 action 的默认值同样**随 camera_source 变**，理由与
    # gripper_backend / depth_topic 一致：这两项在真机与仿真上是**不同的名字**，
    # 固定默认值必然是「仿真对、真机错」或反过来。详见文件头 SIM_/REAL_ 常量。
    ik_group_default = PythonExpression([
        "'" + SIM_IK_GROUP + "' if '", LaunchConfiguration('camera_source'),
        "' == 'sim' else '" + REAL_IK_GROUP + "'",
    ])
    ik_group_arg = DeclareLaunchArgument(
        'ik_group', default_value=ik_group_default,
        description='MoveIt 规划组名。仿真 ur5_manipulator / 真机 ur_manipulator'
                    '（SRDF 派生名，两侧不同）。默认随 camera_source。'
    )
    trajectory_action_default = PythonExpression([
        "'" + SIM_TRAJECTORY_ACTION + "' if '",
        LaunchConfiguration('camera_source'),
        "' == 'sim' else '" + REAL_TRAJECTORY_ACTION + "'",
    ])
    trajectory_action_arg = DeclareLaunchArgument(
        'trajectory_action', default_value=trajectory_action_default,
        description='臂 FollowJointTrajectory action。仿真 joint_trajectory_'
                    'controller / 真机 scaled_joint_trajectory_controller'
                    '（尊重示教器速度滑条）。默认随 camera_source。'
    )
    # tool0 → 指尖同样**随后端变**（见文件头 SIM_/REAL_TOOL0_TO_FINGERTIP）。
    # 它必须在这里覆盖，不能只改 `grasp_planner.yaml`：那份 yaml 是**共用的**，
    # 改它会把仿真一起改坏。
    # 覆盖之所以有效：这一项落在 `planner_params` 的**第二个**元素（字典）里，
    # launch_ros 把列表里每一项各写成一个 `--params-file`，**按顺序给节点**
    # （`launch_ros/utilities/evaluate_parameters.py:155-165` 保序 →
    # `launch_ros/actions/node.py:446` 每个字典一个临时文件）。
    # 已实测：两份文件有同名键时**后一份赢**，且只在前一份里的键**照样保留** ——
    # 所以这里只需写要覆盖的那几项，不必抄全 35 条。
    tool0_to_fingertip_default = PythonExpression([
        "'" + SIM_TOOL0_TO_FINGERTIP + "' if '",
        LaunchConfiguration('camera_source'),
        "' == 'sim' else '" + REAL_TOOL0_TO_FINGERTIP + "'",
    ])
    tool0_to_fingertip_arg = DeclareLaunchArgument(
        'tool0_to_fingertip', default_value=tool0_to_fingertip_default,
        description='tool0 到指尖的距离（m）。仿真 2F-85 0.13 / 真机 AG-95 0.174'
                    '（由 TCP_1 的 z = 173.99 mm 实测）。默认随 camera_source。'
    )
    # 只走位演示（第一个真机 demo）要用 carry_objects:=false，否则规划器会去
    # 闭合一个不存在的夹爪。它**必须**和后端一起给：none 后端 + true 会在
    # planner_node 启动时直接抛错（刻意如此，见 gripper.py 的 NullGripper）。
    carry_objects_arg = DeclareLaunchArgument(
        'carry_objects', default_value='true',
        description='false = 只走位、不夹持（第一个真机 demo）。'
                    '须与 gripper_backend:=none 成对使用。'
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
        parameters=[
            # ⚠️ 顺序有意义：**文件在前、覆盖项在后**。`serial_no` **只在文件里**
            # （带引号才传得过去，理由见上面那段注释）—— 别把它加到下面这个
            # dict 里，加了就是无引号 → 被读成 double → 节点退出。
            # 包不在时用空 dict 占位（与 planner_params 同一种写法）。
            LaunchConfiguration('realsense_params_file') if realsense_params_default else {},
            {
                'align_depth.enable': True,
            },
        ],
        # 把 `/camera/camera/...` 收回单层，理由与实测见文件头 REALSENSE_REMAPS。
        remappings=REALSENSE_REMAPS,
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
            # 默认值随 camera_source 变（仿真 0.1 / 真机 0.3），见上面 max_sync_dt_arg
            'max_sync_dt': LaunchConfiguration('max_sync_dt'),
            # 采样/配准旋钮：全部在参数面上出现，理由见上面那一组 DeclareLaunchArgument
            'depth_scale': LaunchConfiguration('depth_scale'),
            'depth_grid': LaunchConfiguration('depth_grid'),
            'depth_cluster_tol': LaunchConfiguration('depth_cluster_tol'),
            'depth_median_window': LaunchConfiguration('depth_median_window'),
            'depth_buffer_len': LaunchConfiguration('depth_buffer_len'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
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
            # 这两项在真机与仿真上**名字不同**（SRDF 派生名 + 官方默认控制器），
            # 必须覆盖，理由见文件头 SIM_/REAL_ 常量。
            'ik_group': LaunchConfiguration('ik_group'),
            'trajectory_action': LaunchConfiguration('trajectory_action'),
            'tool0_to_fingertip': LaunchConfiguration('tool0_to_fingertip'),
            # 只走位演示：规划器不闭合夹爪，但仍走完「接近→下降→抬起→HOME→
            # 放置位→下降→抬起→HOME」的完整动作序列。
            'carry_objects': LaunchConfiguration('carry_objects'),
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
        realsense_params_arg,
        depth_topic_arg,
        tf_required_arg,
        max_sync_dt_arg,
        depth_scale_arg,
        depth_grid_arg,
        depth_cluster_tol_arg,
        depth_median_window_arg,
        depth_buffer_len_arg,
        camera_info_topic_arg,
        detections_topic_arg,
        gripper_backend_arg,
        ik_group_arg,
        trajectory_action_arg,
        tool0_to_fingertip_arg,
        carry_objects_arg,
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
