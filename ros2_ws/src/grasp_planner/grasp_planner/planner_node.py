"""抓取规划节点 (grasp_planner) —— 仿真与真机共用同一份代码。

从参考项目 grasp_sim/scripts/grasp_planner.py 移植（P3-6，2026-09-27）。
移植时**刻意删掉了它自带的一切 2D→3D 投影**：坐标转换统一由 perception_3d
负责，本节点只消费已经落在 base_link 下的 3D 结果。

订阅:
  /targets_3d   (vision_msgs/Detection3DArray) — **全部**候选，已含 id/类别/分数

调用:
  /compute_ik                      (moveit_msgs/GetPositionIK)   — 求臂关节角
  /joint_trajectory_controller/follow_joint_trajectory
                                   (control_msgs/FollowJointTrajectory)

状态机（一个抓取周期）
======================
    初始: 张开夹爪 → HOME
    ┌─→ 选目标(pick_zone 内 / Y 排序 / 去重 / 最高分) → IK → 接近
    │      → 下降 → 闭合夹爪 → 判空 → 抬起 → HOME
    │      → 下一个放置槽位 → IK → 接近 → 下降 → 张开(放下) → 抬起 → HOME ─┐
    └──────────────────────────────────────────────────────────────────────┘

终止 / 收工（四条出路，**都不依赖「物体掉到某高度以下」这种副作用**）
=====================================================================
  1. 放置槽位用尽 → ``_finish()``（放好的物体落在 pick_zone 之外，自动出局，
     见 ``_validate_zones`` —— 这就是终止机制本身）；
  2. 连续 ``max_attempts_without_place`` 次尝试没有产生成功放置 → ``_finish()``
     （抓取失败本该重试，但没有上限就是死循环）；
  3. 空闲 ``idle_finish_timeout`` 秒都没有**发起过任何一次尝试** → ``_finish()``
     （**抓取区已空**的唯一出路。上面第 2 条只在「选中了目标」时才计数，
     兜不住「一批候选全都不可行动」——那时一次尝试都不发起，见 ``_idle_check``。
     判据必须是「没发起尝试」而不是「没收到检测」：后者会被 10 Hz 的检测流架空）；
  4. 失败恢复流程自身失败 → ``_finish()``（交给人，不再自动重试）。
  ``_finish()`` 之后**不再接受新目标**，但节点不退出（便于现场看日志与诊断）。

为什么订阅 ``/targets_3d`` 而不是单个 ``/target_3d``
======================================================
抓取选择需要**全部候选**才能做三件事：
  1. 按 Y 排序（从左到右抓，避免机械臂横扫过其他物体）；
  2. 跳过已经抓过的（按世界坐标去重）；
  3. 取最高分。
单条 ``PoseStamped`` 装不下这些，也不含 id/类别/分数，订阅者分不清哪个是哪个。
详见 perception_3d 的模块文档与 RESTRUCTURE_PLAN 的 P3-6。

臂与夹爪**必须串行**发（重要）
==============================
JTC 的 ``allow_partial_joints_goal: true`` 让一条轨迹可以只命令部分关节，代价是
**新目标会抢占当前目标**——同一时刻 JTC 只认一条轨迹。

参考项目把 6 个臂关节 + 1 个夹爪关节塞进同一条轨迹，天然串行。本节点把夹爪拆成
独立命令后，如果「臂在动的同时发夹爪命令」，夹爪那条轨迹会**静默顶掉臂轨迹**，
臂就停在半路。所以下面所有 ``gripper.*()`` 都放在**臂轨迹的 done 回调里**，
绝不与臂轨迹并发。见 gripper.py 的模块文档。

移植自参考项目、但**已删除**的东西
===================================
  - ``_optical_to_world()``：硬编码相机位姿 + 手推轴向翻转，**完全不用 TF**。
    真机上相机一换位置就静默算错。现在由 perception_3d 查 TF 完成。
  - ``_publish_camera_tf()``：发一个自称「approximate」的假相机 TF 供可视化，
    与它自己投影用的变换并不一致。现在由 arm_grasp_sim/sim.launch.py 发**真值**
    TF（已用实跑数据反证），真机侧由手眼标定结果提供。
  - ``_call_ik()`` 里的 ``x - BASE_X`` 等基准偏移：那是仿真世界坐标转 base_link
    的换算。``/targets_3d`` 已经在 base_link 下，再减一次就是**双重偏移**。
  - ``_send_to_place()`` / ``PLACE_X/Y/Z``：未被状态机调用的死代码（状态机走的
    是「左摆放下」）。删掉，避免两套放置逻辑对不上。
  - ``left_drop_joints``（关节空间硬编码的「左摆 90° 松爪」）：整套设计已由
    ``place_slots`` 取代，参数本身也一并删除 —— 一个**没有任何代码读**的参数
    比没有更坏：它让人以为「现场改它就能退回左摆」，而实际上改了没反应
    （正是 D50 那个「yaml 不生效」的同一类病）。
"""

import math
import threading
from typing import Callable, List, Optional, Tuple

import rclpy
import rclpy.duration
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

from grasp_planner.gripper import make_gripper

JointAngles = List[float]

#: goal status → 人话。用于日志（数字本身在现场毫无可读性）。
_GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def traj_succeeded(status, result) -> bool:
    """这条轨迹是否**真的**成功走完了。

    ⚠️ 只看 ``result.error_code`` 是**不够的**，这是本项目踩过的坑（D52）：
    控制器进程死掉、目标被取消、action server 消失时，中间件交回来的
    ``result`` 是**默认填充的空壳**，而它的 ``error_code`` 恰好是 **0**
    （0 = SUCCEEDED）。只读它就会把「机器人根本没动」读成「到位了」，
    状态机带着错位继续往下走 —— 这类错误在真机上比「报错停下」危险得多。

    所以两个条件都要满足：外层的 goal status 是 SUCCEEDED，**且**
    内层的 ``error_code`` 是 0。
    """
    return (status == GoalStatus.STATUS_SUCCEEDED
            and result is not None
            and result.error_code == 0)


def traj_status_str(status, result) -> str:
    """给日志用的「为什么失败」。"""
    name = _GOAL_STATUS_NAMES.get(status, f"<未知 {status}>")
    code = "None" if result is None else result.error_code
    return f"goal_status={name} error_code={code}"



def rpy_to_quat(rpy) -> Tuple[float, float, float, float]:
    """rpy（弧度）→ 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_angle(a: float) -> float:
    """把角度归一化到 [-π, π]。

    为什么需要：同一个姿态，IK 可能返回 π 也可能返回 -π。若相邻两次目标一个
    落在 +π 附近、一个落在 -π 附近，JTC 会老老实实走 350° 的大回转。
    """
    return math.atan2(math.sin(a), math.cos(a))


def in_zone(zone, x: float, y: float, z: float) -> bool:
    """点是否落在 ``[xmin, xmax, ymin, ymax, zmin, zmax]`` 盒子里（闭区间）。"""
    xmin, xmax, ymin, ymax, zmin, zmax = zone
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax


def zone_str(zone) -> str:
    xmin, xmax, ymin, ymax, zmin, zmax = zone
    return (f"x[{xmin:.3f},{xmax:.3f}] y[{ymin:.3f},{ymax:.3f}] "
            f"z[{zmin:.3f},{zmax:.3f}]")


class GraspPlanner(Node):
    def __init__(self):
        super().__init__("grasp_planner")

        p = self._declare_params()

        self.arm_joints: List[str] = list(p["arm_joint_names"])
        self.base_frame: str = p["base_frame"]
        self.ik_group: str = p["ik_group"]
        self.ik_link: str = p["ik_link"]
        self.ik_timeout: float = p["ik_timeout"]
        self.grasp_rpy = list(p["grasp_rpy"])
        self.home_joints = list(p["home_joints"])
        self.ik_seed = list(p["ik_seed"])
        self.tool0_to_fingertip: float = p["tool0_to_fingertip"]
        self.approach_clearance: float = p["approach_clearance"]
        self.lift_height: float = p["lift_height"]
        self.table_min_z: float = p["table_min_z"]
        self.min_score: float = p["min_score"]
        self.retry_delay: float = p["retry_delay"]
        self.close_settle: float = p["close_settle"]
        # ⚠️ 这两个是**轨迹时长**，在 _send_arm_trajectory 的调用点上用了 8 次。
        # 它们曾经只被 `d(...)` 声明、忘了赋给 self，于是第一段 HOME 轨迹
        # 一发出就 AttributeError，节点在启动 2 秒后直接死掉。
        # 教训：`_declare_params()` 只负责**声明**，不会自动变成属性；
        # 加参数时必须同时在这里赋值，两者不会互相提醒。
        self.move_duration: float = p["move_duration"]
        self.descend_duration: float = p["descend_duration"]
        self.goal_time_tolerance: float = p["goal_time_tolerance"]

        # ---- 抓取区 / 放置槽位 ----
        # ⚠️ `place_slots` 是**扁平**的 [x0,y0,x1,y1,...]，不是 [[x,y],...]。
        # 不是审美选择：ROS 2 的参数类型系统**没有嵌套数组**，
        # `declare_parameter('x', [[1.0,2.0],[3.0,4.0]])` 直接抛
        # `TypeError: The given value is not a list of one of the allowed types`
        # （rcl_interfaces 只有扁平的 double_array_value），节点会**起不来**；
        # launch 侧按 yaml 规则求值时同样会被 check_sequence_type_is_allowed 拒掉。
        # 所以「每两个一组」这层结构只能靠约定 + 下面的启动期长度检查来保证。
        self.pick_zone = [float(v) for v in p["pick_zone"]]
        # 扁平原值留一份：`zip(flat[0::2], flat[1::2])` 会**悄悄丢掉**落单的尾巴，
        # 而奇数长度正是启动期要拒绝的（`_validate_zones` 查的就是这一份）。
        self.place_slots_flat = [float(v) for v in p["place_slots"]]
        self.place_slots = list(zip(self.place_slots_flat[0::2],
                                   self.place_slots_flat[1::2]))
        self.place_z: float = float(p["place_z"])
        self.place_release_lift: float = float(p["place_release_lift"])
        self.max_attempts_without_place: int = int(p["max_attempts_without_place"])
        self.idle_finish_timeout: float = float(p["idle_finish_timeout"])
        self.carry_objects: bool = bool(p["carry_objects"])

        # ---- 姿态安全 ----
        self.elbow_guard_enabled: bool = bool(p["elbow_guard_enabled"])
        self.guard_links = list(p["guard_links"])
        self.table_plane_z: float = float(p["table_plane_z"])
        self.guard_margin: float = float(p["guard_margin"])
        self.max_joint_jump: float = float(p["max_joint_jump"])
        self.table_scene_enabled: bool = bool(p["table_scene_enabled"])
        self.table_box_size = [float(v) for v in p["table_box_size"]]
        self.table_box_pose = [float(v) for v in p["table_box_pose"]]
        self.table_retry_period: float = float(p["table_retry_period"])

        self._n_arm = len(self.arm_joints)
        for name, vec in (("home_joints", self.home_joints),
                          ("ik_seed", self.ik_seed)):
            if len(vec) != self._n_arm:
                raise ValueError(
                    f"参数 {name} 有 {len(vec)} 个值，但 arm_joint_names 有 "
                    f"{self._n_arm} 个关节——两者必须一致。"
                )
        self._validate_zones(float(p["ag95_stroke_mm"]))

        # ---- 关节状态（仅用于日志；夹爪回读由后端自己订阅）----
        self._current_joints = {}

        # ---- 状态机 ----
        self._busy = False
        self._busy_lock = threading.Lock()
        self._pending_msg: Optional[Detection3DArray] = None
        self._first_home_sent = False
        self._grasped_positions: set = set()
        self._approach_arm: JointAngles = [0.0] * self._n_arm
        self._grasp_target: Optional[Tuple[float, float, float]] = None
        # 放置槽位游标：**只增不减**，放成功一个才前进一个（解不出来的槽位跳过）
        self._place_index = 0
        self._place_target: Optional[Tuple[float, float]] = None
        # 「夹爪里可能夹着东西」——原设计**根本没有这个状态**（D53），
        # 于是放置段失败时方块留在爪里，下一次接近会带着它去撞别的目标。
        self._holding = False
        # 收工标志：槽位用尽 / 连续失败超限后置位，之后不再接受新目标。
        self._finished = False
        # 「正在走失败恢复流程」的自锁：防止 _open_and_reset 自己失败后
        # 递归调用自己（两个回调互相触发 = 永远打下去的循环）。
        self._recovering = False
        # 看门狗计数：连续多少次「选了目标」却没有产生成功放置（见
        # max_attempts_without_place）。抓取失败本该重试，但没有上限就是死循环。
        self._attempts_since_place = 0
        # 空闲看门狗（见 idle_finish_timeout）：`_attempt_active` = 有目标正在走，
        # `_idle_since` = 最近一次**发起/结束了一次尝试**的时刻（不是「收到检测」
        # 的时刻 —— 那样会被 10 Hz 的检测流架空）。
        # 初值 None = 还没起算；启动后第一次 `_reset_busy()` 会给它起点。
        self._attempt_active = False
        self._idle_since = None

        # ---- IK 客户端（Reentrant：状态机在自己的回调里发起并等待 IK）----
        self._ik_client = self.create_client(
            GetPositionIK, p["ik_service"], callback_group=ReentrantCallbackGroup()
        )
        # ---- FK 客户端：肘高护栏用（与 /compute_ik 同一个 move_group 提供）----
        self._fk_client = self.create_client(
            GetPositionFK, p["fk_service"], callback_group=ReentrantCallbackGroup()
        )
        # ---- 规划场景客户端：把桌子作为碰撞体发进去（见 _start_table_scene）----
        self._scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene",
            callback_group=ReentrantCallbackGroup(),
        )
        self._table_published = False
        self._table_tries = 0
        self._table_timer = None

        # ---- 轨迹 action ----
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, p["trajectory_action"]
        )

        # ---- 夹爪 ----
        # ⚠️ 这组搭配在**启动时**就要查，不能等到第一次 close()（半分钟后、
        #    臂已经悬在目标上方）才炸。`none` 后端只实现 open()，配合
        #    carry_objects=true 会在 `_evaluate_grasp` 抛错，而那时臂正在动。
        if p["gripper_backend"] == "none" and self.carry_objects:
            raise ValueError(
                "配置矛盾：gripper_backend:=none 不驱动夹爪，无法夹取，"
                "但它被要求 carry_objects:=true（真抓取）。"
                "两者必须成对：只走位演示用 "
                "`carry_objects:=false gripper_backend:=none`；"
                "要真夹取请用 sim 或 ag95。"
            )
        self._gripper = make_gripper(
            self, p["gripper_backend"], p, self._send_trajectory
        )

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(
            Detection3DArray, p["targets_topic"], self._targets_cb, 10
        )

        self.get_logger().info(
            f"GraspPlanner 就绪 | 夹爪后端={p['gripper_backend']} "
            f"({self._gripper.describe()})\n"
            f"  订阅 {p['targets_topic']}（全部候选，{self.base_frame} 下）\n"
            f"  臂关节 {self.arm_joints}\n"
            f"  目标帧={self.base_frame} IK组={self.ik_group} 末端={self.ik_link}\n"
            f"  指尖偏置={self.tool0_to_fingertip} "
            f"接近余量={self.approach_clearance} 抬起高度={self.lift_height}\n"
            f"  动作模式={'抓取' if self.carry_objects else '只走位（不夹持）'} "
            f"| 肘高护栏={self.elbow_guard_enabled}"
            f"（台面 {self.table_plane_z:.3f}）"
            f" | 桌面进场景={self.table_scene_enabled}"
        )

        # 先张开夹爪、再回 HOME。**串行**：夹爪完成回调里才发臂轨迹。
        self._home_timer = self.create_timer(2.0, self._start_initial_home)
        # 空闲看门狗：**唯一**在看门狗计数之外兜底的收工路径。
        self.create_timer(1.0, self._idle_check)
        # 桌面碰撞体：move_group 起得晚，内部带周期重试。
        self._start_table_scene()

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _declare_params(self) -> dict:
        # ⚠️ `d()` 自己记账，返回值由它生成 —— 原先这里另有一份手写的 keys 清单，
        #    两份清单必须逐字对齐，而漏一处的症状有两种：
        #      · 只在 keys 里 → 启动即 KeyError（响亮，能立刻发现）；
        #      · 只在 d() 里 → **参数声明了却读不到，静默无效**（就是 D50 那一类）。
        #    本轮就踩了后者的近亲：新加的 idle_finish_timeout 只写进 d()，
        #    节点启动直接 KeyError。从结构上把第二份清单删掉，这类错就没了。
        declared: List[str] = []

        def d(name, default):
            declared.append(name)
            return self.declare_parameter(name, default)

        d("targets_topic", "/targets_3d")
        d("ik_service", "/compute_ik")
        d("trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory")
        d("base_frame", "base_link")
        d("ik_group", "ur5_manipulator")
        d("ik_link", "tool0")
        d("ik_timeout", 2.0)

        d("arm_joint_names", [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ])

        # 末端姿态（弧度）。π,0,0 = 工具朝下。
        d("grasp_rpy", [math.pi, 0.0, 0.0])
        # 折叠位姿：机械臂两次抓取之间回到这里
        d("home_joints", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        # 前伸 IK 种子。**必须固定**：用 _current_joints 当种子会在回到 HOME 后
        # 求解出极端的 shoulder_pan，参考项目实测如此。
        d("ik_seed", [-0.551, -0.429, 2.045, 3.094, -1.571, -2.122])

        d("tool0_to_fingertip", 0.13)   # tool0 到指尖的距离（m）
        d("approach_clearance", 0.05)   # 接近时在抓取高度之上再抬多少（m）
        d("lift_height", 0.15)          # 抓取后抬起高度（m）
        # 低于此高度视为已落地物体，跳过（**单位是 base_frame 的 z**，不是世界系！）
        #
        # ⚠️ 这个值在不同坐标系里差一个底座高度，极易搞错。参考项目在**世界系**
        # 里用 0.3（桌面顶面 0.41）。迁到 base_frame 后必须换算：
        #   UR5 底座在世界系 z=0.80，桌面 0.41 → base_frame 里桌面 ≈ **-0.39**
        #   桌上的方块顶面实测投影为 base=(-0.350)，与换算吻合
        #   掉到地上的方块（世界系 ≈0.02）→ base_frame ≈ **-0.78**
        # 所以取 -0.5：干净地分开「桌上」(-0.35) 与「地上」(-0.78)。
        #
        # 若沿用世界系的 0.3，则**所有**候选都被判为「已落地」而全部丢弃，
        # 规划器一个目标都选不出来，且没有任何报错——本项目实际踩过这个坑。
        # 真机上线时按现场底座与桌面的实际高度重新标定。
        d("table_min_z", -0.5)
        # ---- 抓取区 / 放置槽位（base_frame 下的 **3D 盒子**）----
        # 格式：[xmin, xmax, ymin, ymax, zmin, zmax]
        #
        # 为什么必须是 3D 而不是 XY 矩形：这个过滤同时在干两件事 ——
        #   1. 决定「谁还能被抓」。**终止机制就建立在这上面**（见 place_slots）；
        #   2. 剔除掉到桌外/地上的物体。XY 矩形挡不住「滚到地上、但 XY 恰好落在
        #      矩形投影里」的方块 —— 那会让机械臂收到一条**扎向地面**的轨迹。
        d("pick_zone", [0.30, 0.90, -0.18, 0.16, -0.45, -0.30])
        # 放置槽位（base_frame 下的 x,y 列表，**按顺序消费**）。
        #
        # ⚠️ 两条硬约束，启动时会逐条检查（不满足直接拒绝启动）：
        #   1. **槽位必须落在 pick_zone 之外**。放好的物体因此自动不再是候选，
        #      这就是终止机制本身（取代旧的「靠掉到 table_min_z 以下」）。
        #      一旦重叠 = 放下去又被抓起来 = 死循环。
        #   2. **间距 ≥ 夹爪张开宽度**。AG-95 行程 95 mm，而抓取用的是开着的
        #      爪子：槽位挨得太近，爪子会去压相邻槽位里已经放好的物体。
        #
        # 默认 6 个 = 2 排 × 3 列，两轴间距都是 0.16 m（> 0.095）。
        # 仿真里给了 6 个而方块有 9 个 —— 这是**故意**的：跑满 6 个就会走到
        # 「槽位用尽」那条收工路径，正好能在仿真里验证它。
        #
        # ⚠️ 格式是**扁平**的 [x0,y0, x1,y1, ...]，不是 [[x,y],...]：
        # ROS 2 参数类型系统没有嵌套数组，嵌套写法会让节点**起不来**
        # （详见 __init__ 里 place_slots 那段注释）。长度必须是偶数，
        # 奇数会在 _validate_zones 里被当成硬错误拒绝。
        d("place_slots", [
            0.44, 0.24, 0.60, 0.24, 0.76, 0.24,
            0.44, -0.24, 0.60, -0.24, 0.76, -0.24,
        ])
        # 放置参考高度（base_frame 系）：**与抓取段同一个约定** —— perception
        # 报的那个点落到槽位上时的 z，即方块静止时**可见顶面**的高度，不是中心。
        # 仿真：桌面顶面 -0.39 + 方块高 0.04 = -0.35。
        # ⚠️ 写「中心」(-0.37) 会让夹爪多下 2 cm、指尖插进桌面碰撞体，
        #    六个放置位的下降 IK 全被拒（-31），症状是「能抓不能放」。
        # 真机（臂装在工作台桌面上）：同样按「桌面实测高度 + 方块高」标定。
        d("place_z", -0.35)
        # 放置下降时在「方块理论落点」之上**额外**留的高度（m）。
        # ⚠️ 夹爪**指尖比方块还低**：抓取把方块夹在顶面高度上，方块整体吊在
        #   指尖下方，而 2F-85 的指尖几何在**闭合**时还会再低 11.9 mm（实测 FK），
        #   最低点恰好与方块底面齐平（碰撞网格是真实 STL）。于是「方块底面贴桌面」
        #   这个位置指尖必然同时压在桌面上 —— 夹爪闭合时下降 IK 被拒（-31）。
        #   实测边界：张开 tool0=-0.225 可解/-0.230 拒；**闭合** -0.215 可解/-0.220 拒。
        #   而 place_z=-0.35 正对应 tool0=-0.220 —— 抓取段（张开）看着好好的，
        #   只有手里夹着东西时才发作。
        # 真机：AG-95 指尖长度不同，这个值与 place_z 都要按实物重标。
        d("place_release_lift", 0.01)
        # 连续多少次尝试都没产生新的成功放置 → 判定为「卡住了」，收工告警。
        # 抓取失败（方块掉回抓取区）本该重试，但**没有上限就是死循环**。
        d("max_attempts_without_place", 10)
        # 空闲多久**没有发起过任何一次抓取尝试**就收工（s）。
        # ⚠️ 上面那个计数器**只在「选中了一个目标」时才加一**，所以它兜不住
        #   「一批候选全都不可行动」这种情形：候选一个都不剩（抓取区空了）、
        #   或者剩下的全在 _grasped_positions 里（已抓过）、或者分数都低于
        #   min_score —— 此时一次尝试都不会发起，计数器永远不动，于是节点
        #   以检测频率（~10 Hz）无限刷「已就绪，等待下一次抓取」而**永不收工**。
        #   实测就是这样卡住的：一次抓取失败把方块丢回了抓取区，那批候选全被
        #   判为「已处理过」，日志刷了上千行、什么也没发生。
        # 这是**唯一的收工路径**能覆盖「抓取区已空」的地方，也是这个 demo 能被
        # 判定「跑完了」的依据。真机上若 YOLO 首帧很慢，把它调大。
        #
        # ⚠️ 计时起点是「上一次**尝试**」，不是「上一次**收到检测**」。
        #   第一版写成后者，被 10 Hz 的检测流彻底架空：每批都「有候选 → 全被
        #   滤掉 → 复位计时器」，日志里 `待命中 0s/20s` 永远停在 0s，收工路径
        #   形同不存在。判据见 _reset_busy / _select_and_approach 里的 _idle_since。
        d("idle_finish_timeout", 20.0)
        # false = **只做动作演示**：不闭合夹爪、不判空、不夹持，臂照样走完
        # 「接近 → 下降 → 抬起 → HOME → 放置位 → 下降 → 抬起 → HOME」。
        # 这是给「第一个真机 demo」用的（夹爪硬件还没接通，见 RESTRUCTURE_PLAN）。
        # 注意：记账（_grasped_positions）**照常**，否则同一个方块会被反复选。
        d("carry_objects", True)

        d("min_score", 0.3)             # 最低置信度
        d("move_duration", 2.0)         # 常规位姿切换时长（s）
        d("descend_duration", 1.5)      # 下降时长（s）——比常规慢一点，减少撞击
        # 轨迹到点后的**宽限时间**（s）。0 是最严（一到点就必须已在容差内），
        # 真机把示教器速度滑条调慢时会因此整条判失败。见 _send_trajectory（D55）。
        d("goal_time_tolerance", 2.0)
        d("gripper_duration", 3.0)      # 夹爪开合时长（s）——仿真 4-bar 连杆要时间收敛
        d("close_settle", 0.5)          # 闭合后等物理沉降再判空（s）
        d("retry_delay", 0.5)           # 没有可抓目标时，多久后重新接受检测（s）

        # ---- 夹爪 ----
        d("gripper_backend", "sim")     # sim | ag95 | none
        # 仅 backend=none 时有意义：写进启动日志的「为什么没有夹爪」。
        # 真机第一个 demo（只走位、不夹持）由 real.launch.py 填实际情况。
        d("gripper_none_reason", "")
        d("sim_gripper_joint", "robotiq_85_left_knuckle_joint")
        d("sim_gripper_open", 0.0)
        d("sim_gripper_close", 0.8)
        d("sim_gripper_empty_threshold", 0.75)
        d("sim_gripper_barely_moved", 0.02)
        d("ag95_port", "/dev/ttyUSB0")
        d("ag95_baudrate", 115200)
        d("ag95_slave_id", 1)
        d("ag95_force", 50)
        d("ag95_stroke_mm", 95.0)
        d("ag95_open_mm", 95.0)
        d("ag95_close_mm", 0.0)
        # ⚠️ **默认只读**：写寄存器会让手指动，而寄存器映射与千分位方向都还
        # 没有实物复核过。翻成 false 的前提是「夹爪已从臂上拆下」（见 W3）。
        d("ag95_dry_run", True)
        # 千分位方向：1000 是全开（true）还是全闭（false）。**手册没写**，
        # 现场手动掰到全开读一次 0x0202 判决。只有 dry_run=false 时才影响动作。
        d("ag95_open_is_high", True)

        # ---- 姿态安全：肘高护栏（D49）----
        # `_ik_request` 里的 `avoid_collisions=True` **只在规划场景里有东西时才
        # 有意义**。全仓库没有任何地方往场景里放碰撞物（桌子、夹爪、方块都没
        # 有），所以它实际只在防自碰撞 —— **桌子对 MoveIt 是不可见的**。
        # 而「臂装在工作台桌面上」这个真机几何会让**肘朝下**的 IK 解落到桌面
        # 以下约 21 cm（详见 RESTRUCTURE_PLAN 的手算）。仿真因为底座坐在 0.80 m
        # 立柱上，永远解不到那个分支，所以从未暴露。
        #
        # 这里用 FK 取关节链上若干连杆的高度，要求全部在台面之上。
        # ⚠️ **不是**「解不满足就换种子重解」—— 去种子正是让 KDL 翻转肘/腕分支
        # 的操作（D54）。不满足就换**明确的另一分支种子**再解一次，两个都不行
        # 就放弃这个目标。
        d("elbow_guard_enabled", True)
        d("fk_service", "/compute_fk")
        # UR5 的连杆名（已核对 ur_description/urdf/ur_macro.xacro）：
        # **没有 elbow_link**，肘关节的子树根就是 forearm_link。
        d("guard_links", ["forearm_link", "wrist_1_link"])
        # 台面在 base_frame 里的高度。真机（臂装在工作台上）≈ 0；仿真 ≈ -0.39。
        d("table_plane_z", -0.39)
        d("guard_margin", 0.02)         # 护栏余量（m）
        # 同一段内相邻路点的最大关节跳变（rad，L∞）。超过就认为解到了另一个
        # 分支 —— 见 _consistent_with。
        d("max_joint_jump", 1.0)

        # ---- 姿态安全：把桌子放进 MoveIt 规划场景（D49 的正解）----
        # 场景里有桌子之后，`avoid_collisions=True` 才**名副其实**：
        # /compute_ik 会拿机器人模型（含夹爪）去和场景做碰撞检查，
        # 肘插进桌面的解会被 MoveIt 自己拒掉 —— 比上面的手写护栏更准。
        #
        # 默认**关闭**：仿真与真机的桌子尺寸/位置不同，一个默认值不可能两边都对，
        # 而**错的碰撞体比没有更坏**（可能把正确的解全拒掉，看起来像 IK 坏了）。
        # 仿真侧在 arm_grasp_sim 的 launch 里打开；真机侧在真机 launch 里打开。
        d("table_scene_enabled", False)
        d("table_box_size", [0.8, 0.6, 0.03])           # 全尺寸 (x,y,z)
        d("table_box_pose", [0.6, 0.0, -0.405, 0.0, 0.0, 0.0])  # base_frame 下
        d("table_retry_period", 2.0)    # move_group 没起来时的重发周期（s）

        return {k: self.get_parameter(k).value for k in declared}

    # ------------------------------------------------------------------
    # 参数校验（启动期，宁可拒绝启动也不要跑起来才发现）
    # ------------------------------------------------------------------
    def _validate_zones(self, gripper_stroke_mm: float):
        """检查抓取区与放置槽位。**重叠 = 死循环**，故是硬错误。

        终止机制的完整逻辑：物体一旦被放进槽位，它的坐标就落在 ``pick_zone``
        之外，于是 ``_select_and_approach`` 的过滤会自动把它排除掉 ——
        不再需要「靠掉到某个高度以下退出候选」这种副作用式的判据。
        这个推论**只在槽位与抓取区不重叠时成立**。
        """
        if len(self.pick_zone) != 6:
            raise ValueError(
                f"pick_zone 必须是 6 个数 [xmin,xmax,ymin,ymax,zmin,zmax]，"
                f"现在是 {len(self.pick_zone)} 个：{self.pick_zone}"
            )
        xmin, xmax, ymin, ymax, zmin, zmax = self.pick_zone
        if xmin >= xmax or ymin >= ymax or zmin >= zmax:
            raise ValueError(
                f"pick_zone 的 min 必须小于 max：{zone_str(self.pick_zone)}"
            )
        if not self.place_slots:
            raise ValueError("place_slots 是空的——没有任何放置位，规划器无事可做。")
        # 扁平格式的配对检查：奇数个值 = 最后一个 x 没有配对的 y。
        # 这里必须**报错**而不是丢掉尾巴：丢尾巴等于静默少一个槽位，
        # 而槽位数正是终止条件（少一个就少放一个物体，且没有任何提示）。
        if len(self.place_slots_flat) % 2 != 0:
            raise ValueError(
                f"place_slots 必须成对 [x0,y0, x1,y1, ...]，现在是 "
                f"{len(self.place_slots_flat)} 个值（奇数）—— "
                "最后一个 x 没有配对的 y。"
            )

        for i, (sx, sy) in enumerate(self.place_slots):
            if in_zone(self.pick_zone, sx, sy, self.place_z):
                raise ValueError(
                    f"place_slots[{i}] = ({sx}, {sy}) 与 place_z={self.place_z} "
                    f"落在 pick_zone ({zone_str(self.pick_zone)}) **之内**。\n"
                    "放好的物体会立刻重新变成抓取候选 → 无限重复抓取。\n"
                    "请把槽位移到抓取区之外，或把 pick_zone 收窄。"
                )

        # 间距：抓取用的是**张开**的爪子去放，槽位挨太近会压到相邻槽位里
        # 已经放好的物体。参考量是夹爪行程（AG-95 为 95 mm）。
        # ⚠️ 这一条是**告警**不是错误：它与物体尺寸、夹爪型号都有关，
        # 是现场可判断的物理风险，不该由代码一刀切。
        min_pitch = gripper_stroke_mm / 1000.0
        for i in range(len(self.place_slots)):
            for j in range(i + 1, len(self.place_slots)):
                dx = self.place_slots[i][0] - self.place_slots[j][0]
                dy = self.place_slots[i][1] - self.place_slots[j][1]
                pitch = math.hypot(dx, dy)
                if pitch < min_pitch:
                    self.get_logger().warn(
                        f"place_slots[{i}] 与 [{j}] 间距 {pitch*1000:.1f} mm "
                        f"< 夹爪行程 {gripper_stroke_mm:.1f} mm —— 张着爪子去放时"
                        "可能会压到相邻槽位里已放好的物体。"
                    )

        # `place_release_lift` 必须为正 —— 它守的是「指尖别插进桌面」。
        # 推导：place_z = 桌面高 + 方块高，下降位 tool0 = place_z + 指尖偏移 + lift；
        # 而指尖几何最低点约在 tool0 - 0.17（2F-85 实测，比方块还低一点点）。
        # 于是「指尖最低点 > 桌面」这件事**恰好等价于** lift > 0。
        # ⇒ lift = 0 不是「保守取值」，而是指尖与桌面齐平的**临界**；负数就是直接撞。
        # 这条是硬错误：真机重标参数时最可能的错法就是把这个值调成 0 或负。
        if self.place_release_lift <= 0.0:
            raise ValueError(
                f"place_release_lift={self.place_release_lift:.3f} 必须为正 —— "
                "它守的是「夹爪指尖不插进桌面」。指尖几何最低点与方块底面齐平，"
                "place_z 本身对应的高度正好是指尖贴桌面的临界点，"
                "所以下降高度必须再抬高一个正余量。详见该参数的注释。"
            )

        self.get_logger().info(
            f"抓取区 {zone_str(self.pick_zone)} | 放置槽位 {len(self.place_slots)} 个"
            f" @ place_z={self.place_z:.3f}（间距下限告警 {min_pitch*1000:.0f} mm）"
        )
        self.get_logger().info(
            f"放置下降位 tool0 z={self._place_descend_z:.3f}"
            f"（= place_z {self.place_z:.3f} + 指尖偏移 {self.tool0_to_fingertip:.3f}"
            f" + 释放余量 {self.place_release_lift:.3f}）；"
            f"接近位 z={self._place_descend_z + self.approach_clearance:.3f}，"
            f"抬起位 z={self._place_descend_z + self.lift_height:.3f}"
        )

    # ------------------------------------------------------------------
    # 规划场景：把桌子作为碰撞体发进去（D49 的正解）
    # ------------------------------------------------------------------
    def _start_table_scene(self):
        """按参数决定是否把桌面发进 MoveIt 规划场景。

        为什么这件事非做不可：``_ik_request`` 里那行 ``avoid_collisions = True``
        读起来像「有碰撞检查」，但**全仓库没有任何地方往场景里放过碰撞物** ——
        没有桌子、没有夹爪、没有方块。所以它实际只在防自碰撞（靠 SRDF 里那
        19 条 ``disable_collisions`` 构成的 ACM），**桌子对 MoveIt 是不可见的**。

        把桌子发进去之后，``/compute_ik`` 会拿机器人模型（含夹爪连杆）去和场景
        做碰撞检查，肘插进桌面的解会被 MoveIt 自己拒掉 —— 这比手写的肘高护栏
        更准，因为用的是真实的连杆几何。两者都保留：护栏不依赖场景服务。
        """
        if not self.table_scene_enabled:
            self.get_logger().warn(
                "未往规划场景里加桌面碰撞体（table_scene_enabled=false）："
                "avoid_collisions 现在**只防自碰撞**，桌子对 MoveIt 不可见。"
                f"肘高护栏={self.elbow_guard_enabled}。"
                "真机上线前请按现场桌面尺寸打开它。"
            )
            return
        self.get_logger().info(
            f"把桌面作为碰撞体发入规划场景：size={self.table_box_size} "
            f"pose={self.table_box_pose}（{self.base_frame} 下）"
        )
        # move_group 通常比本节点起得晚，且它一重启场景就丢，所以带周期重试。
        self._table_timer = self.create_timer(
            self.table_retry_period, self._try_publish_table
        )

    def _try_publish_table(self):
        if self._table_published:
            self._table_timer.cancel()
            return
        # ⚠️ 这里**不能**用 wait_for_service 阻塞：本回调与状态机同在默认
        # （互斥）回调组里，一阻塞就把状态机一起卡住。
        if not self._scene_client.service_is_ready():
            self._table_tries += 1
            if self._table_tries % 5 == 1:
                self.get_logger().warn(
                    "/apply_planning_scene 还没就绪（move_group 起了吗？），继续等 ..."
                )
            return

        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = "table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(self.table_box_size)
        pose = Pose()
        pose.position.x = float(self.table_box_pose[0])
        pose.position.y = float(self.table_box_pose[1])
        pose.position.z = float(self.table_box_pose[2])
        qx, qy, qz, qw = rpy_to_quat(self.table_box_pose[3:])
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects = [obj]
        self._scene_client.call_async(req).add_done_callback(
            self._on_table_scene_done
        )

    def _on_table_scene_done(self, fut):
        resp = fut.result()
        if resp is not None and resp.success:
            self._table_published = True
            self.get_logger().info(
                "桌面碰撞体已进入规划场景 —— avoid_collisions 现在名副其实。"
            )
        else:
            self.get_logger().error(
                "桌面碰撞体下发失败（/apply_planning_scene 返回 success=false）"
            )

    # ------------------------------------------------------------------
    # 关节状态 / 初始位姿
    # ------------------------------------------------------------------
    def _joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.arm_joints:
                self._current_joints[name] = pos

    def _start_initial_home(self):
        self._home_timer.cancel()
        self.get_logger().info("初始化：先张开夹爪，再回 HOME ...")
        self._gripper.open(done_cb=self._on_initial_open_done)

    def _on_initial_open_done(self, _status, _result):
        self._send_arm_trajectory(
            self.home_joints, self.move_duration, done_cb=self._on_initial_home_done
        )

    def _on_initial_home_done(self, status, result):
        self._traj_failed("初始 HOME", status, result)
        # 即便失败也放行：目标是**能动起来**，卡在初始位姿连诊断都做不了。
        # 失败已经在上面记了 error 日志。
        self._first_home_sent = True
        self._reset_busy()

    # ------------------------------------------------------------------
    # 轨迹
    # ------------------------------------------------------------------
    def _traj_failed(self, what: str, status, result) -> bool:
        """轨迹是否失败；失败时打一条**统一格式**的错误日志。

        所有 done 回调都用它，避免每个回调各写一遍判据 —— 少写一半
        （只看 ``error_code``）就是 D52 那个把「没动」读成「到位」的坑。
        """
        if traj_succeeded(status, result):
            return False
        self.get_logger().error(f"{what}失败：{traj_status_str(status, result)}")
        return True

    def _send_trajectory(self, joint_names: List[str], positions: List[float],
                         duration: float, done_cb: Optional[Callable] = None):
        """发一条 ``FollowJointTrajectory``。

        也被仿真夹爪后端复用（只发它那一个关节，靠 JTC 的
        ``allow_partial_joints_goal: true``——已实测可用）。

        ⚠️ 新目标会**抢占**正在执行的轨迹，所以调用方必须保证：上一条轨迹的
        ``done_cb`` 已经触发之后，才发下一条。

        ``done_cb`` 的签名是 ``(goal_status, result)``，**两个参数都要看** ——
        见 :func:`traj_succeeded` 与 D52。失败时两者都可能是 ``None``。
        """
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("轨迹 action 服务不可用（控制器起了吗？）")
            if done_cb:
                done_cb(None, None)
            return

        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        # stamp=0：让 JTC 立刻执行，而不是等一个未来的时间点
        traj.header.stamp = Time(sec=0, nanosec=0)

        point = JointTrajectoryPoint()
        # 把每个目标角「展开」到离当前位置最近的等价角，保证 JTC 走短路。
        # 这是防 360° 回转的**第二道**防线：第一道是 IK 侧的归一化，
        # 但两次调用之间机械臂可能已经转过大半圈，所以这里再按当前位置校正一次。
        unwrapped = [float(v) for v in positions]
        for i, name in enumerate(joint_names):
            if name in self._current_joints:
                curr = self._current_joints[name]
                diff = unwrapped[i] - curr
                unwrapped[i] = curr + math.atan2(math.sin(diff), math.cos(diff))
        point.positions = unwrapped
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9)
        )
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # ⚠️ 这里曾经写 0，注释是「0 = 关闭 goal time 检查」——**理解是反的**（D55）。
        # 在 FollowJointTrajectory 里 goal_time_tolerance 是**额外的宽限时间**：
        # 轨迹的 time_from_start 到了之后，控制器还有多久可以进入位置容差。
        # 0 是**最严**：一到点必须已经在容差内，否则判 GOAL_TOLERANCE_VIOLATED。
        # 而 time_from_start 是本节点自己拍的死值（move_duration 等），不受
        # 示教器速度滑条影响 —— 真机把滑条调慢，每条轨迹都会被判失败。
        # 给 1~2 s 的宽限严格比 0 宽松，对仿真（容差全 0）和真机都更稳。
        goal.goal_time_tolerance = Duration(
            sec=int(self.goal_time_tolerance),
            nanosec=int((self.goal_time_tolerance % 1) * 1e9),
        )

        if done_cb is None:
            self._traj_client.send_goal_async(goal)
            return

        def _on_goal_response(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().error("轨迹目标被拒绝")
                done_cb(None, None)
                return
            handle.get_result_async().add_done_callback(_on_traj_result)

        def _on_traj_result(f):
            # GetResult wrapped response：.status 是 goal status，.result 是
            # FollowJointTrajectory.Result。**两个都往下传**（D52）。
            wrapped = f.result()
            done_cb(wrapped.status, wrapped.result)

        self._traj_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    def _send_arm_trajectory(self, arm: JointAngles, duration: float,
                             done_cb: Optional[Callable] = None):
        self._send_trajectory(self.arm_joints, arm, duration, done_cb)

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------
    def _call_ik(self, x: float, y: float, z: float,
                 seed: Optional[JointAngles] = None,
                 prev: Optional[JointAngles] = None) -> Optional[JointAngles]:
        """求 ``base_frame`` 下 (x, y, z) 的臂关节角；不安全或无解时返回 ``None``。

        注意参考项目在这里做了 ``x - BASE_X`` 之类的减法，那是把仿真世界坐标
        换算到 base_link。``/targets_3d`` **已经是** base_link 下的坐标，
        再减一次会变成双重偏移，故删除。

        ``prev`` 是**本运动段的上一个路点**（解算时用它的解当种子传进来）。
        它换来一道 D54 的检查：见 :meth:`_consistent_with`。

        ⚠️ 这里曾经有一条「``shoulder_pan`` > 1.40 rad 就去掉种子重解」的判据。
        **它有两个毛病，已删除**：
          (a) 「``shoulder_pan`` 大 = 向后够」是**仿真直觉**，不成立 —— 基座螺栓
              固定在工作台上时，侧向的抓取/放置**合法地**需要大于 80° 的
              ``shoulder_pan``，护栏会对**正确解**开火；
          (b) 它的补救是**去掉种子重解**，而「去种子」恰恰是让 KDL 翻转肘/腕
              分支的操作 —— 等于把正确解换成一个分支翻转的危险解。
        取代它的是**肘高护栏**（真的去查连杆高度）+ **换分支时用明确的另一支
        种子**，两者都不再依赖关节角的绝对值大小。
        """
        base_seed = seed if seed is not None else self.ik_seed
        angles = self._ik_request(x, y, z, base_seed)
        if angles is None:
            return None

        if self._posture_ok(angles):
            return angles if self._consistent_with(prev, angles) else None

        # 姿态不过：换**明确的另一分支**再解一次。**不是**去种子（那是 D54）。
        other = self._other_branch_seed(base_seed)
        self.get_logger().warn("肘高护栏不过，改用「另一分支」种子重解一次 ...")
        retry = self._ik_request(x, y, z, other)
        if retry is None:
            self.get_logger().error(
                f"另一分支也解不出来 @ ({x:.3f}, {y:.3f}, {z:.3f}) —— 放弃该目标，不发轨迹"
            )
            return None
        if not self._posture_ok(retry):
            self.get_logger().error("两个分支都不满足肘高护栏 —— 放弃该目标，不发轨迹")
            return None
        return retry if self._consistent_with(prev, retry) else None

    def _consistent_with(self, prev: Optional[JointAngles],
                         arm: JointAngles) -> bool:
        """这个解与上一个路点在关节空间里是否**离得够近**（L∞）。

        为什么需要：``_send_trajectory`` 发的是**单点**关节空间轨迹，控制器从
        当前位置直接插值过去。解到另一个分支时（肘从 +2.0 翻到 −2.0 那种）是
        一次真刀真枪的 4 rad 大摆动。

        注意这里先用 ``normalize_angle`` 取差：±π 附近的**假跳变**（wrist 从
        +175° 到 −175°，其实是绕了 10°）会被算成 0.17 rad，不会误报；而真正
        的分支翻转不会被归一化消掉。两者必须分开，否则要么误报要么漏报。
        """
        if prev is None:
            return True
        jump = max(abs(normalize_angle(a - b)) for a, b in zip(arm, prev))
        if jump > self.max_joint_jump:
            self.get_logger().error(
                f"关节空间跳变过大：与上一路点最大差 {jump:.2f} rad "
                f"> max_joint_jump={self.max_joint_jump:.2f} —— 疑似解到了另一个"
                "分支。单点关节轨迹跟不了这种大摆动，拒绝这条轨迹。"
            )
            return False
        return True

    def _other_branch_seed(self, seed: JointAngles) -> JointAngles:
        """把一个解翻到「另一分支」当种子：**肘关节取反**。

        平面 2R 的两个解就是 ±θ2，所以把肘角取反是最省事也最好解释的换分支
        办法。关键在于它与原种子**相差很大**，KDL 会收敛到另一个构型 ——
        这正是「去种子重解」做不到的：去种子是随机的，而这是我们主动挑的另一支。
        """
        try:
            i = self.arm_joints.index("elbow_joint")
        except ValueError:
            i = 2       # UR5 的固定顺序：pan, lift, elbow, wrist_1..3
        other = [normalize_angle(v) for v in seed]
        other[i] = normalize_angle(-other[i])
        return other

    def _posture_ok(self, arm: JointAngles) -> bool:
        """肘高护栏：被检查的连杆是否**都在台面之上**（D49）。

        ⚠️ FK 查不通时**拒绝**（fail-closed）。理由：护栏开着却查不到高度，
        等于「以为有保护、其实没有」—— 那正是 D49 的病因本身，不能重演。
        仿真/真机都由 move_group 提供 ``/compute_fk``，正常情况下查得到；
        真要绕过它，请显式 ``elbow_guard_enabled:=false``。
        """
        if not self.elbow_guard_enabled:
            return True
        floor = self.table_plane_z + self.guard_margin
        for link in self.guard_links:
            p = self._call_fk(link, arm)
            if p is None:
                self.get_logger().error(
                    f"肘高护栏无法判定（{link} 的 FK 查不到）—— 保守拒绝这个位姿。"
                    "若是 FK 服务缺失，请检查 move_group。"
                )
                return False
            if p[2] < floor:
                self.get_logger().warn(
                    f"肘高护栏拦下：{link} 的 z={p[2]:.3f} < 台面+余量 "
                    f"{floor:.3f}（table_plane_z={self.table_plane_z:.3f}）"
                )
                return False
        return True

    def _call_fk(self, link: str,
                 joints: JointAngles) -> Optional[Tuple[float, float, float]]:
        """求某个连杆在 ``base_frame`` 下的位置（``/compute_fk``）。

        与 ``/compute_ik`` 由同一个 move_group 提供，不需要新依赖。
        """
        if not self._fk_client.wait_for_service(timeout_sec=self.ik_timeout):
            self.get_logger().error("FK 服务 /compute_fk 不可用（MoveIt 起了吗？）")
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(self.arm_joints)
        req.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(req)
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while not future.done():
            if self.get_clock().now() > deadline:
                self.get_logger().error("FK 请求超时（5s）")
                return None
            threading.Event().wait(0.02)

        resp = future.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            code = resp.error_code.val if resp else "None"
            self.get_logger().warn(f"FK 无结果（{link}，error_code={code}）")
            return None
        p = resp.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def _ik_request(self, x: float, y: float, z: float,
                    seed: Optional[JointAngles]) -> Optional[JointAngles]:
        """发一次 IK 请求。``seed=None`` 表示**不带种子**。"""
        if not self._ik_client.wait_for_service(timeout_sec=self.ik_timeout):
            self.get_logger().error("IK 服务 /compute_ik 不可用（MoveIt 起了吗？）")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.ik_group
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        qx, qy, qz, qw = rpy_to_quat(self.grasp_rpy)
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw

        # ⚠️ 种子放在 robot_state.joint_state 里，**不是** ik_seed_state——
        # PositionIKRequest 根本没有 ik_seed_state 这个字段（实测字段表：
        # group_name, robot_state, constraints, avoid_collisions, ik_link_name,
        # pose_stamped, ik_link_names, pose_stamped_vector, timeout）。
        if seed is not None:
            req.ik_request.robot_state.joint_state.name = list(self.arm_joints)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]

        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        req.ik_request.timeout.nanosec = 0

        future = self._ik_client.call_async(req)
        # 本函数在回调线程里被调用，**不能**在这里 spin executor；
        # IK 客户端在 ReentrantCallbackGroup 里，响应由别的线程处理。
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while not future.done():
            if self.get_clock().now() > deadline:
                self.get_logger().error("IK 请求超时（5s）")
                return None
            threading.Event().wait(0.02)

        resp = future.result()
        if resp is None or resp.error_code.val != 1:   # 1 = SUCCESS
            code = resp.error_code.val if resp else "None"
            self.get_logger().warn(
                f"IK 无解（error_code={code}）@ ({x:.3f}, {y:.3f}, {z:.3f})"
            )
            return None

        solution = {n: p for n, p in zip(
            resp.solution.joint_state.name, resp.solution.joint_state.position)}
        try:
            # 归一化到 [-π, π]，否则 π 与 -π 附近的解会让下游算出 360° 的大回转
            return [normalize_angle(solution[n]) for n in self.arm_joints]
        except KeyError as e:
            self.get_logger().error(f"IK 解里缺少关节 {e}")
            return None

    # ------------------------------------------------------------------
    # 选目标
    # ------------------------------------------------------------------
    def _targets_cb(self, msg: Detection3DArray):
        # 始终缓存最新一批：忙的时候也不要错过最新的世界坐标
        if msg.detections:
            self._pending_msg = msg

        if self._finished:
            return          # 已收工（槽位用尽 / 连续失败超限）
        if not self._first_home_sent:
            return

        with self._busy_lock:
            if self._busy:
                return
            self._busy = True

        latest = self._pending_msg
        self._pending_msg = None
        if latest is None or not latest.detections:
            self._reset_busy()
            return

        self._select_and_approach(latest)

    def _reset_busy(self):
        if self._finished:
            return          # 收工后不要再喊「已就绪」
        # 丢掉过期缓存：本轮结束后要等**新的**检测，而不是拿旧坐标再抓一次
        self._pending_msg = None
        with self._busy_lock:
            self._busy = False
        self._recovering = False    # 干净地回到待命 = 恢复流程已结束
        # 空闲看门狗的计时起点。⚠️ **只在「真的跑完了一次尝试」或「启动后第一次
        # 复位」时重置**，不能每批复位都重置 —— 10 Hz 的检测流下，一个「有候选
        # 但全被滤掉」的批次也会走到这里，那样计时器永远停在 0，看门狗名存实亡
        # （实测：日志里 `待命中 0s/20s` 一直不动，收工路径形同不存在）。
        if self._attempt_active or self._idle_since is None:
            self._idle_since = self.get_clock().now()
        self._attempt_active = False
        # ⚠️ 10 Hz 的检测频率下这个函数在**闲着的时候每批都会被调一次**，所以
        #    这行日志必须限流，否则卡住时日志会以每秒几十行的速度涨（实测过：
        #    3.4 MB 全是这一句）。
        self.get_logger().info("已就绪，等待下一次抓取", throttle_duration_sec=5.0)

    def _idle_check(self):
        """空闲看门狗：闲太久 = 没有可抓的目标 → 收工（``idle_finish_timeout``）。

        没有它，节点在「抓取区已空 / 候选全已抓过 / 分数都不够」时会**静默地
        永远待命** —— 一次尝试都不发起，``max_attempts_without_place`` 这个
        计数器永远不动。这个 demo 也就没有「跑完了」这个状态。

        判据是「多久**没有发起过尝试**」，不是「多久**没收到检测**」：后者会被
        检测流架空（见 :meth:`_reset_busy` 里的说明）。
        """
        if self._finished or self._attempt_active or self._idle_since is None:
            return
        idle = (self.get_clock().now() - self._idle_since).nanoseconds / 1e9
        if idle >= self.idle_finish_timeout:
            self._finish(
                f"空闲 {idle:.0f}s 没有发起过任何一次抓取尝试"
                f"（上限 idle_finish_timeout={self.idle_finish_timeout:.0f}s）"
                " —— 抓取区已空 / 候选都已抓过 / 分数都不够"
            )
            return
        self.get_logger().info(
            f"待命中 {idle:.0f}s/{self.idle_finish_timeout:.0f}s："
            "还没有可抓的目标",
            throttle_duration_sec=5.0,
        )

    def _finish(self, reason: str):
        """收工：打日志 + **停止接受新目标**。

        ⚠️ 这里**不能**调 ``_reset_busy()``：那会让节点继续接受检测，而槽位
        已经放满了 —— 于是同一批方块被反复抓起来又因为没地方放而放下，
        无限循环。这是 ``_next_place_slot()`` 返回 ``None`` 时必须有的**显式**
        出路（另一条错路是把 ``_busy`` 永久置位 → 静默死锁，连日志都没有）。

        节点本身不退出：留着进程便于现场看日志、也便于 ros2 param/ros2 topic
        继续诊断。要停就 Ctrl-C（或由 launch 统一收）。
        """
        self._finished = True
        self._attempt_active = False    # 空闲看门狗就此闭嘴
        with self._busy_lock:
            self._busy = False
        self._pending_msg = None
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"收工：{reason}")
        self.get_logger().info(
            f"  已成功放置 {self._place_index} 个 / 槽位共 {len(self.place_slots)} 个"
            f"；已抓取记账 {len(self._grasped_positions)} 个"
        )
        self.get_logger().info("  **不再接受新目标**。节点仍在运行，Ctrl-C 退出。")
        if self._holding:
            self.get_logger().warn(
                "  ⚠️ 夹爪里可能还夹着一个物体，且已无处可放。"
                "请在 HOME 位置人工取走（或补 place_slots 后重启）。"
            )
        self.get_logger().info("=" * 60)

    def _select_and_approach(self, msg: Detection3DArray):
        """从全部候选里选一个未抓过的目标 → IK → 接近。"""
        candidates = []
        for d in msg.detections:
            if not d.results:
                continue
            label = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            pos = d.results[0].pose.pose.position
            wx, wy, wz = pos.x, pos.y, pos.z

            # ① 粗筛：低于 table_min_z 的一律不要（挡住投影噪声产生的 z≈0
            #    或负值）。它的**终止职责已经卸掉**了 —— 现在只当安全护栏，
            #    真正决定「谁还能被抓」的是下面的 pick_zone。
            if wz < self.table_min_z:
                continue
            # ② 真正的候选过滤：必须落在 pick_zone 这个 **3D 盒子**里。
            #    放好的物体落在槽位上（在 pick_zone 之外），于是自动出局 ——
            #    **这就是终止机制本身**，不再依赖「掉到某高度以下」这种副作用。
            if not in_zone(self.pick_zone, wx, wy, wz):
                continue
            candidates.append((score, label, wx, wy, wz))

        # 按 Y 排序（从左到右），避免机械臂横扫过其他物体
        candidates.sort(key=lambda c: (c[3], -c[0]))

        # 每批都打这一行会刷屏（闲着时检测频率就是 ~10 Hz）。限流到 2 s：
        # 正常跑动时一个周期远超 2 s，所以真正的进展一条都不会被吞掉。
        self.get_logger().info(
            f"候选 {len(candidates)} 个 {[c[1] for c in candidates]}"
            f" | 已放置 {self._place_index}/{len(self.place_slots)}"
            f" | 已抓 {len(self._grasped_positions)} 个"
            f" | 连续未放置 {self._attempts_since_place}"
            f"/{self.max_attempts_without_place}",
            throttle_duration_sec=2.0,
        )

        best = None
        for score, label, wx, wy, wz in candidates:
            if (round(wx, 2), round(wy, 2), round(wz, 2)) in self._grasped_positions:
                continue
            if score < self.min_score:
                continue
            best = (score, label, wx, wy, wz)
            break

        if best is None:
            ungrasped = [
                c for c in candidates
                if (round(c[2], 2), round(c[3], 2), round(c[4], 2))
                not in self._grasped_positions
            ]
            if not ungrasped and candidates:
                # 候选都在 _grasped_positions 里：本批没有新目标。**不是**收工 ——
                # 收工交给空闲看门狗（_idle_check）。这里只回到待命，并让
                # _reset_busy 的限流日志说明状况，不再每批打一行。
                self._reset_busy()
            else:
                # 有未抓的但分数不够：等下一批检测，别急着重试
                threading.Timer(self.retry_delay, self._reset_busy).start()
            return

        # ③ 槽位闸门：**起步之前**先确认还有地方可放。
        #    这条闸门必须在这里，不能只在 _on_lift_done 里判 —— 否则节点会
        #    先抓起来、再发现无处可放，收工时夹爪里吊着一个方块，还白跑了一整
        #    个抓取周期（实测 run 3：6 个槽位放满后仍有第 7 次抓取，日志出现
        #    「⚠️ 夹爪里可能还夹着一个物体，且已无处可放」）。
        #    抓取是整条链上**唯一不可撤销**的一步：接近/下降/抬起失败都能原地
        #    恢复，唯独「夹起来了」没法靠重试退回。所以闸门要设在它前面。
        #    _on_lift_done 里那道检查保留，作为「手上有物体」的兜底。
        if self._next_place_slot() is None:
            self._finish(
                f"放置槽位已用尽（{len(self.place_slots)} 个）—— 没有地方可放了"
            )
            return

        # 看门狗：从这里开始算「一次尝试」。成功放置时清零（见 _on_place_released）。
        self._attempts_since_place += 1
        # 也让空闲看门狗闭嘴：一个周期要跑几十秒，中间不能判「闲太久」。
        # 两头都要起算：开始起算（这条），结束也起算（_reset_busy）—— 一次尝试
        # 若比 idle_finish_timeout 还长，只靠开头那次起算会在它刚结束时立刻误判收工。
        self._attempt_active = True
        self._idle_since = self.get_clock().now()
        if self._attempts_since_place > self.max_attempts_without_place:
            self._finish(
                f"连续 {self._attempts_since_place - 1} 次尝试都没有产生新的成功"
                f"放置（上限 {self.max_attempts_without_place}）—— 判定为卡住"
            )
            return

        score, label, wx, wy, wz = best
        self._grasp_target = (wx, wy, wz)
        self.get_logger().info(
            f"→ 选中 [{label}] score={score:.2f} "
            f"{self.base_frame}=({wx:.3f}, {wy:.3f}, {wz:.3f})"
        )

        approach_z = wz + self.tool0_to_fingertip + self.approach_clearance
        # 固定用 ik_seed：用 _current_joints（此时是 HOME）当种子会解出极端 shoulder_pan
        arm = self._call_ik(wx, wy, approach_z, seed=self.ik_seed)
        if arm is None:
            self.get_logger().warn("接近位姿 IK 失败，放弃该目标，等下一批检测")
            self._open_and_reset()
            return

        self.get_logger().info(
            "  接近解：" + str([f"{math.degrees(j):.1f}°" for j in arm])
        )
        # 记下接近解：下降与抬起都用同一解支当种子，避免 wrist_1 从 +175° 翻到
        # -175°（那会变成 350° 的大回转）。
        self._approach_arm = list(arm)

        self._send_arm_trajectory(arm, self.move_duration,
                                  done_cb=self._on_approach_done)

    # ------------------------------------------------------------------
    # 状态机：接近 → 下降 → 闭合 → 抬起 → HOME → 放置位 → 下降 → 张开 → 抬起 → HOME
    # ------------------------------------------------------------------
    def _on_approach_done(self, status, result):
        if self._traj_failed("接近轨迹", status, result):
            self._open_and_reset()
            return

        self.get_logger().info("接近完成 —— 下降到抓取高度")
        wx, wy, wz = self._grasp_target
        arm = self._call_ik(wx, wy, wz + self.tool0_to_fingertip,
                            seed=self._approach_arm, prev=self._approach_arm)
        if arm is None:
            self.get_logger().warn("抓取高度 IK 失败")
            self._open_and_reset()
            return
        self._send_arm_trajectory(arm, self.descend_duration,
                                  done_cb=self._on_descend_done)

    def _on_descend_done(self, status, result):
        if self._traj_failed("下降轨迹", status, result):
            self._open_and_reset()
            return
        if not self.carry_objects:
            # 只走位模式（第一个真机 demo：夹爪硬件还没接通）：
            # 不闭合、不判空，但**照样走完**抬起 → HOME → 放置位 → 抬起 → HOME，
            # 把完整动作序列演示出来。
            self.get_logger().info("下降完成 ——（只走位模式：不闭合夹爪）抬起")
            self._lift_and_go()
            return
        self.get_logger().info("下降完成 —— 闭合夹爪")
        self._gripper.close(done_cb=self._on_close_done)

    def _on_close_done(self, _status, _result):
        # 等物理沉降：仿真的 mimic + 4-bar 连杆在轨迹结束后还需要一点时间
        # 才收敛到接触力，立刻读会读到还在动的中间值。
        threading.Timer(self.close_settle, self._evaluate_grasp).start()

    def _evaluate_grasp(self):
        pos = self._gripper.current_position()
        pretty = "未收到回读" if pos is None else f"{pos:.4f}"
        if self._gripper.is_empty():
            self.get_logger().warn(
                f"夹爪疑似空夹（回读位置={pretty}）——张开后重试，本次不计入已抓"
            )
            # **不**记入 _grasped_positions：没夹到就该能重试
            self._holding = False
            self._open_and_reset()
            return

        self._holding = True     # D53：从这里开始「手里可能有东西」
        self.get_logger().info(f"已夹住物体（回读位置={pretty}）—— 抬起")
        self._lift_and_go()

    def _lift_and_go(self):
        """从抓取高度抬起一点，再回 HOME。

        抬起位姿解不出来时**直接回 HOME**：那会从抓取高度横着扫过去，风险比
        抬起大，但比卡在原地（状态机停摆）小。两害相权取此，并打告警。
        """
        wx, wy, wz = self._grasp_target
        arm = self._call_ik(wx, wy, wz + self.tool0_to_fingertip + self.lift_height,
                            seed=self._approach_arm, prev=self._approach_arm)
        if arm is None:
            self.get_logger().warn("抬起位姿 IK 失败，直接回 HOME")
            self._send_arm_trajectory(self.home_joints, self.move_duration,
                                      done_cb=self._on_lift_done)
            return
        self._send_arm_trajectory(arm, self.move_duration, done_cb=self._on_lift_done)

    def _on_lift_done(self, status, result):
        if self._traj_failed("抬起轨迹", status, result):
            self._open_and_reset()
            return
        # 抬起来才算真的抓到了，这时候才记账（移植自参考项目）
        if self._grasp_target is not None:
            wx, wy, wz = self._grasp_target
            self._grasped_positions.add((round(wx, 2), round(wy, 2), round(wz, 2)))
            self.get_logger().info(
                f"抬起成功，已记录 {self.base_frame}=({wx:.3f}, {wy:.3f}, {wz:.3f})"
                f"（累计 {len(self._grasped_positions)} 个）"
            )
        self.get_logger().info("返回 HOME ...")
        self._send_arm_trajectory(self.home_joints, self.move_duration,
                                  done_cb=self._on_home_with_object_done)

    # ---- 放置段：抓取段的镜像 ----
    @property
    def _place_descend_z(self) -> float:
        """放置下降的 tool0 目标高度（``base_frame`` 系）。

        = 方块静止时的顶面高度 ``place_z`` + 指尖偏移 + **释放余量**。

        最后那一项不是「保守取值」，是几何上**必须**的：指尖最低点与方块底面
        齐平（夹爪闭合时还会再低一点），所以「方块底面正好贴到桌面」这个位姿
        指尖必然同时压在桌面上，IK 会被桌子挡掉。推导与实测见
        ``place_release_lift`` / ``place_z`` 两个参数的注释。
        """
        return self.place_z + self.tool0_to_fingertip + self.place_release_lift

    def _on_home_with_object_done(self, status, result):
        if self._traj_failed("携带物体回 HOME", status, result):
            self._open_and_reset()
            return

        # 逐个试剩余槽位：某个槽位解不出来（够不着 / 肘高护栏不过）就**跳过它**
        # 继续试下一个。不跳过的话会卡死在同一个槽上，而槽位表就那么长。
        approach_z = self._place_descend_z + self.approach_clearance
        while True:
            slot = self._next_place_slot()
            if slot is None:
                # ⚠️ 走到这里时**方块已经在夹爪里**了 —— 和 _select_and_approach
                # 里那道槽位闸门不同：那道闸门是「没地方放就别去抓」，防的是白跑
                # 一个周期；这一条是「抓都抓了，却没地方放」，只能收工并告警。
                # 它是可达的：下面每个解不出来的槽位都会 _place_index += 1 跳过，
                # 跳着跳着就可能把槽位表耗尽。
                self._finish(
                    f"放置槽位已用尽（{len(self.place_slots)} 个）—— 没有地方可放了"
                )
                return
            sx, sy = slot
            arm = self._call_ik(sx, sy, approach_z, seed=self.ik_seed)
            if arm is not None:
                break
            self.get_logger().warn(
                f"槽位[{self._place_index}] ({sx:.3f}, {sy:.3f}) 接近位姿解不出来，"
                "跳过它"
            )
            self._place_index += 1      # 放弃这个槽位，继续试下一个

        self._place_target = (sx, sy)
        # 放置段全程用这个解当种子，理由与抓取段相同（见 _select_and_approach）
        self._approach_arm = list(arm)
        self.get_logger().info(
            f"已到 HOME —— 去放置位[{self._place_index}] "
            f"({sx:.3f}, {sy:.3f})"
        )
        self._send_arm_trajectory(arm, self.move_duration,
                                  done_cb=self._on_place_approach_done)

    def _next_place_slot(self) -> Optional[Tuple[float, float]]:
        """下一个槽位；用尽返回 ``None``。**不消费** —— 消费在放下成功时。"""
        if self._place_index >= len(self.place_slots):
            return None
        return self.place_slots[self._place_index]

    def _on_place_approach_done(self, status, result):
        if self._traj_failed("放置位接近轨迹", status, result):
            self._open_and_reset()
            return
        sx, sy = self._place_target
        arm = self._call_ik(sx, sy, self._place_descend_z,
                            seed=self._approach_arm, prev=self._approach_arm)
        if arm is None:
            self.get_logger().warn("放置高度 IK 失败，放弃这个槽位")
            self._open_and_reset()
            return
        self.get_logger().info(
            f"放置位接近完成 —— 下降到 place_z={self.place_z:.3f}"
        )
        self._send_arm_trajectory(arm, self.descend_duration,
                                  done_cb=self._on_place_descend_done)

    def _on_place_descend_done(self, status, result):
        if self._traj_failed("放置下降轨迹", status, result):
            self._open_and_reset()
            return
        if not self.carry_objects:
            self.get_logger().info("已到放置高度 ——（只走位模式：不张开夹爪）")
            self._on_place_released(None, None)
            return
        self.get_logger().info("已到放置高度 —— 张开夹爪放下")
        self._gripper.open(done_cb=self._on_place_released)

    def _on_place_released(self, _status, _result):
        """放下完成：消费槽位、清 ``_holding``、清看门狗，然后抬起并回 HOME。"""
        self._place_index += 1
        self._holding = False
        self._attempts_since_place = 0
        self.get_logger().info(
            f"已放置 {self._place_index}/{len(self.place_slots)} 个"
        )
        # 先抬起再回 HOME：避免张着的爪子贴着台面横移。
        sx, sy = self._place_target
        arm = self._call_ik(
            sx, sy, self._place_descend_z + self.lift_height,
            seed=self._approach_arm, prev=self._approach_arm,
        )
        if arm is None:
            self.get_logger().warn("放置后抬起 IK 失败，直接回 HOME")
            self._send_arm_trajectory(self.home_joints, self.move_duration,
                                      done_cb=self._on_place_retreat_done)
            return
        self._send_arm_trajectory(arm, self.move_duration,
                                  done_cb=self._on_place_retreat_done)

    def _on_place_retreat_done(self, status, result):
        if self._traj_failed("放置后抬起轨迹", status, result):
            self._open_and_reset()
            return
        self.get_logger().info("返回 HOME ...")
        self._send_arm_trajectory(self.home_joints, self.move_duration,
                                  done_cb=self._on_home_done)

    def _on_open_done(self, _status, _result):
        self._holding = False
        self.get_logger().info("返回 HOME ...")
        self._send_arm_trajectory(self.home_joints, self.move_duration,
                                  done_cb=self._on_home_done)

    def _on_home_done(self, status, result):
        if self._traj_failed("回 HOME 轨迹", status, result):
            self._open_and_reset()
            return
        threading.Timer(self.retry_delay, self._reset_busy).start()

    def _open_and_reset(self):
        """失败/空夹的收尾：**先回 HOME、再张开夹爪**。

        ⚠️ 顺序不能反（D51）。这里是**所有**轨迹失败与空夹的汇聚点（十几处），
        原先写的是「先开爪、再回 HOME」，于是每次失败都把手里的物体**丢在
        当场**：失败发生时机械臂停在哪儿是不可预料的 —— 可能悬在桌面正上方，
        也可能紧贴着某个已经放好的物体，在那里松爪等于制造一次无控的砸落/碰撞。
        先回 HOME 是回到一个**已知**的姿态，再松爪。

        带自锁 ``_recovering``：恢复流程**自己**失败时不再递归调用自己 ——
        两个回调互相触发会变成永远打下去的循环。此时直接收工，交给人。
        """
        if self._recovering:
            self.get_logger().error(
                "恢复流程（回 HOME → 张开夹爪）自身又失败了 —— 不再自动重试，"
                "请人工确认机械臂状态。"
            )
            self._finish("失败恢复流程自身失败")
            return
        self._recovering = True

        if self._holding:
            self.get_logger().warn(
                "夹爪里可能还有物体 —— **先回 HOME**，到了 HOME 再张开"
                "（不在当前位置松爪）。"
            )

        def _then_open(status, result):
            if self._traj_failed("失败后回 HOME", status, result):
                # HOME 都没走到：**不要**在这里松爪（位置未知），也不再重试。
                self.get_logger().error(
                    "回 HOME 失败，夹爪保持原状不动 —— 停止本轮自动重试。"
                )
                self._finish("失败恢复时回 HOME 也失败")
                return
            self.get_logger().info("已回 HOME —— 张开夹爪")
            self._gripper.open(done_cb=self._on_open_done)

        self._send_arm_trajectory(self.home_joints, self.move_duration,
                                  done_cb=_then_open)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # 两个都要接，且**不是**防御性写法：Ctrl-C 下 rclpy 先由自己的信号
        # 处理器关掉 context，然后**哪一个异常先到是竞态** —— 信号处理器抛的
        # `KeyboardInterrupt`，或等待被中断时 `spin_once` 抛的
        # `ExternalShutdownException`。2026-09-28 板端实测：同一份 launch 里
        # 本节点走了 KeyboardInterrupt 那条（报 cleanly），而用
        # `rclpy.spin()` 的 detector / projector 走了另一条（报 died）。
        # 只接一个就等于把「收工干不干净」交给运气。
        pass
    finally:
        node._gripper.shutdown()
        executor.shutdown()
        node.destroy_node()
        # ⚠️ 必须先 `rclpy.ok()`：SIGINT/SIGTERM（`ros2 launch` 下的 Ctrl-C、
        # 或者超时杀进程）会先由 rclpy 自己的信号处理器调一次 rcl_shutdown，
        # 之后再无条件调一次会抛
        #     RCLError: failed to shutdown: rcl_shutdown already called
        # 并打出一整段 traceback。功能上无害，但**每次正常收工都像出了故障**：
        # 演示时操作员看到 traceback 会以为刚才那一轮失败了。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
