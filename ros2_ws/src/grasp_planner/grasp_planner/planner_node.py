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
    ┌─→ 选目标(Y 排序 / 去重 / 最高分) → IK → 接近
    │      → 下降 → 闭合夹爪 → 判空 → 抬起 → HOME
    │      → 左摆 → 张开(放下) → HOME ─┐
    └──────────────────────────────────┘

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
"""

import math
import threading
from typing import Callable, List, Optional, Tuple

import rclpy
import rclpy.duration
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

from grasp_planner.gripper import make_gripper

JointAngles = List[float]


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
        self.left_drop_joints = list(p["left_drop_joints"])
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

        self._n_arm = len(self.arm_joints)
        for name, vec in (("home_joints", self.home_joints),
                          ("left_drop_joints", self.left_drop_joints),
                          ("ik_seed", self.ik_seed)):
            if len(vec) != self._n_arm:
                raise ValueError(
                    f"参数 {name} 有 {len(vec)} 个值，但 arm_joint_names 有 "
                    f"{self._n_arm} 个关节——两者必须一致。"
                )

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

        # ---- IK 客户端（Reentrant：状态机在自己的回调里发起并等待 IK）----
        self._ik_client = self.create_client(
            GetPositionIK, p["ik_service"], callback_group=ReentrantCallbackGroup()
        )

        # ---- 轨迹 action ----
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, p["trajectory_action"]
        )

        # ---- 夹爪 ----
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
            f"接近余量={self.approach_clearance} 抬起高度={self.lift_height}"
        )

        # 先张开夹爪、再回 HOME。**串行**：夹爪完成回调里才发臂轨迹。
        self._home_timer = self.create_timer(2.0, self._start_initial_home)

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _declare_params(self) -> dict:
        d = self.declare_parameter
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
        # 左侧放置位姿：与 HOME 同为折叠，但 shoulder 左转 90°
        d("left_drop_joints", [1.57, -1.57, 1.57, -1.57, -1.57, 0.0])
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
        d("min_score", 0.3)             # 最低置信度
        d("move_duration", 2.0)         # 常规位姿切换时长（s）
        d("descend_duration", 1.5)      # 下降时长（s）——比常规慢一点，减少撞击
        d("gripper_duration", 3.0)      # 夹爪开合时长（s）——仿真 4-bar 连杆要时间收敛
        d("close_settle", 0.5)          # 闭合后等物理沉降再判空（s）
        d("retry_delay", 0.5)           # 没有可抓目标时，多久后重新接受检测（s）

        # ---- 夹爪 ----
        d("gripper_backend", "sim")     # sim | ag95
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

        keys = [
            "targets_topic", "ik_service", "trajectory_action", "base_frame",
            "ik_group", "ik_link", "ik_timeout", "arm_joint_names", "grasp_rpy",
            "home_joints", "left_drop_joints", "ik_seed", "tool0_to_fingertip",
            "approach_clearance", "lift_height", "table_min_z", "min_score",
            "move_duration", "descend_duration", "gripper_duration",
            "close_settle", "retry_delay", "gripper_backend",
            "sim_gripper_joint", "sim_gripper_open", "sim_gripper_close",
            "sim_gripper_empty_threshold", "sim_gripper_barely_moved",
            "ag95_port", "ag95_baudrate", "ag95_slave_id", "ag95_force",
            "ag95_stroke_mm", "ag95_open_mm", "ag95_close_mm",
        ]
        return {k: self.get_parameter(k).value for k in keys}

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

    def _on_initial_open_done(self, _result):
        self._send_arm_trajectory(
            self.home_joints, self.move_duration, done_cb=self._on_initial_home_done
        )

    def _on_initial_home_done(self, result):
        if result is None or result.error_code != 0:
            code = "None" if result is None else result.error_code
            self.get_logger().error(f"初始 HOME 失败：error_code={code}")
        self._first_home_sent = True
        self._reset_busy()

    # ------------------------------------------------------------------
    # 轨迹
    # ------------------------------------------------------------------
    def _send_trajectory(self, joint_names: List[str], positions: List[float],
                         duration: float, done_cb: Optional[Callable] = None):
        """发一条 ``FollowJointTrajectory``。

        也被仿真夹爪后端复用（只发它那一个关节，靠 JTC 的
        ``allow_partial_joints_goal: true``——已实测可用）。

        ⚠️ 新目标会**抢占**正在执行的轨迹，所以调用方必须保证：上一条轨迹的
        ``done_cb`` 已经触发之后，才发下一条。
        """
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("轨迹 action 服务不可用（控制器起了吗？）")
            if done_cb:
                done_cb(None)
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
        # 0 = 关闭 goal time 检查，避免因轻微超时被判失败
        goal.goal_time_tolerance = Duration(sec=0, nanosec=0)

        if done_cb is None:
            self._traj_client.send_goal_async(goal)
            return

        def _on_goal_response(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().error("轨迹目标被拒绝")
                done_cb(None)
                return
            handle.get_result_async().add_done_callback(
                lambda f: done_cb(f.result().result)
            )

        self._traj_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    def _send_arm_trajectory(self, arm: JointAngles, duration: float,
                             done_cb: Optional[Callable] = None):
        self._send_trajectory(self.arm_joints, arm, duration, done_cb)

    # ------------------------------------------------------------------
    # IK
    # ------------------------------------------------------------------
    def _call_ik(self, x: float, y: float, z: float,
                 seed: Optional[JointAngles] = None) -> Optional[JointAngles]:
        """求 ``base_frame`` 下 (x, y, z) 的臂关节角。

        注意参考项目在这里做了 ``x - BASE_X`` 之类的减法，那是把仿真世界坐标
        换算到 base_link。``/targets_3d`` **已经是** base_link 下的坐标，
        再减一次会变成双重偏移，故删除。
        """
        angles = self._ik_request(x, y, z, seed if seed is not None else self.ik_seed)
        if angles is None:
            return None

        # shoulder_pan 超过 ~80° 说明机械臂在**向后**够。参考项目实测：种子的
        # 关节角会把解带到这个分支上，此时**去掉种子**重解一次，让 MoveIt 用它
        # 内部维护的当前状态，通常能拉回正向。只重试一次。
        if abs(angles[0]) > 1.40:
            self.get_logger().warn(
                f"shoulder_pan={math.degrees(angles[0]):.1f}° 过大（向后够），"
                "去掉种子重解一次 ..."
            )
            retry = self._ik_request(x, y, z, None)
            if retry is not None:
                return retry
            self.get_logger().warn("去种子重解仍失败，沿用原解")
        return angles

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
        # 丢掉过期缓存：本轮结束后要等**新的**检测，而不是拿旧坐标再抓一次
        self._pending_msg = None
        with self._busy_lock:
            self._busy = False
        self.get_logger().info("已就绪，等待下一次抓取")

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

            # 只抓桌面上的。低于 table_min_z 的已经在放置点掉到地上了。
            # （perception_3d 已经把「投影失败/深度无效」的样本丢掉了，
            #   所以这里不需要再判 pose.z == 0。）
            if wz < self.table_min_z:
                continue
            candidates.append((score, label, wx, wy, wz))

        # 按 Y 排序（从左到右），避免机械臂横扫过其他物体
        candidates.sort(key=lambda c: (c[3], -c[0]))

        self.get_logger().info(
            f"候选 {len(candidates)} 个 {[c[1] for c in candidates]}"
            f" | 已抓 {len(self._grasped_positions)} 个"
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
                self.get_logger().info(
                    f"全部 {len(self._grasped_positions)} 个物体已抓完。结束。"
                )
                self._reset_busy()
            else:
                # 有未抓的但分数不够：等下一批检测，别急着重试
                threading.Timer(self.retry_delay, self._reset_busy).start()
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
    # 状态机：接近 → 下降 → 闭合 → 抬起 → HOME → 左摆 → 张开 → HOME
    # ------------------------------------------------------------------
    def _on_approach_done(self, result):
        if result is None or result.error_code != 0:
            code = "None" if result is None else result.error_code
            self.get_logger().error(f"接近轨迹失败：error_code={code}")
            self._open_and_reset()
            return

        self.get_logger().info("接近完成 —— 下降到抓取高度")
        wx, wy, wz = self._grasp_target
        arm = self._call_ik(wx, wy, wz + self.tool0_to_fingertip,
                            seed=self._approach_arm)
        if arm is None:
            self.get_logger().warn("抓取高度 IK 失败")
            self._open_and_reset()
            return
        self._send_arm_trajectory(arm, self.descend_duration,
                                  done_cb=self._on_descend_done)

    def _on_descend_done(self, result):
        if result is None or result.error_code != 0:
            code = "None" if result is None else result.error_code
            self.get_logger().error(f"下降轨迹失败：error_code={code}")
            self._open_and_reset()
            return
        self.get_logger().info("下降完成 —— 闭合夹爪")
        self._gripper.close(done_cb=self._on_close_done)

    def _on_close_done(self, _result):
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
            self._open_and_reset()
            return

        self.get_logger().info(f"已夹住物体（回读位置={pretty}）—— 抬起")
        wx, wy, wz = self._grasp_target
        arm = self._call_ik(wx, wy, wz + self.tool0_to_fingertip + self.lift_height,
                            seed=self._approach_arm)
        if arm is None:
            self.get_logger().warn("抬起位姿 IK 失败，直接回 HOME")
            self._send_arm_trajectory(self.home_joints, self.move_duration,
                                      done_cb=self._on_lift_done)
            return
        self._send_arm_trajectory(arm, self.move_duration, done_cb=self._on_lift_done)

    def _on_lift_done(self, result):
        if result is None or result.error_code != 0:
            code = "None" if result is None else result.error_code
            self.get_logger().error(f"抬起轨迹失败：error_code={code}")
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

    def _on_home_with_object_done(self, _result):
        self.get_logger().info("已到 HOME —— 左摆放置")
        self._send_arm_trajectory(self.left_drop_joints, self.move_duration,
                                  done_cb=self._on_left_drop_done)

    def _on_left_drop_done(self, _result):
        self.get_logger().info("已到放置位 —— 张开夹爪放下")
        self._gripper.open(done_cb=self._on_open_done)

    def _on_open_done(self, _result):
        self.get_logger().info("返回 HOME ...")
        self._send_arm_trajectory(self.home_joints, self.move_duration,
                                  done_cb=self._on_home_done)

    def _on_home_done(self, _result):
        threading.Timer(self.retry_delay, self._reset_busy).start()

    def _open_and_reset(self):
        """失败/空夹的收尾：张开夹爪，然后回 HOME。

        串行：夹爪张开的 done_cb 里才发 HOME 轨迹。
        """
        def _then_home(_result):
            self._send_arm_trajectory(self.home_joints, self.move_duration,
                                      done_cb=self._on_home_done)

        self._gripper.open(done_cb=_then_home)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._gripper.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
