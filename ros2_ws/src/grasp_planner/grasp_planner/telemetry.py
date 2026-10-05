"""planner_node 的**只读**遥测通道 —— 让规划器的决策看得见。

为什么需要它
============
``planner_node`` 原先**一个发布者都没有**（1553 行里 ``create_publisher``
出现 **0** 次）。它的全部决策只走 ``/rosout`` 文本日志，于是：

  · 现场跑真机时只能读日志猜「它现在在干什么」；
  · 录回来的 bag 里**看不到**规划器瞄准了哪里、往哪个槽位放、为什么收工 ——
    而这些恰恰是事后分析最需要的东西；
  · RViz 里没法把「相机看到的检测框」与「规划器实际用的 3D 点」叠起来看，
    而两者对不上（投影错了、或目标被 ``pick_zone`` 滤掉了）正是最常见的故障。

只读，不参与决策
================
本模块**只 publish**。它不读状态机的变量（计数靠 ``snapshot`` 回调取一份
快照），更不改动它。**加发布者不会改变状态机行为** —— 这是它可以随时摘掉、
也敢直接上真机的原因。

三个话题
========
======================  ==================================  ============================
话题                    类型                                回答的问题
======================  ==================================  ============================
``.../current_target``  ``geometry_msgs/PoseStamped``       它现在瞄的是哪个 3D 点？
``.../place_slots``     ``visualization_msgs/MarkerArray``  哪些槽位空着/用过/被跳过？
``.../telemetry``       ``diagnostic_msgs/DiagnosticArray`` 状态时间线、计数、**收工原因**
======================  ==================================  ============================

QoS 一律 **transient_local（latched）+ reliable + depth 1**：晚加入的 RViz / rqt
也能立刻拿到最后一条值，不必干等下一次状态变化（一个抓取周期几十秒，等不起）。
在 bag 里则因为**每次变化都发一条**，天然形成时间线。

⚠️⚠️ **订阅方也必须声明 transient_local，否则锁存对它毫无意义。**
这是实测踩到的坑（2026-09-29，``scripts/test_planner_telemetry.py`` 的第一版就是
栽在这里）：发布方 latched 只表示「它留了一份给晚到者」，而一个 **VOLATILE**
的订阅者只收**实时**消息，永远拿不到那份锁存 —— **不报错，只是空着**。
rclpy 的 ``create_subscription(..., 10)`` 走的就是默认的 VOLATILE，所以：

  · 用脚本看这些话题时，QoS 要显式给 ``durability=TRANSIENT_LOCAL``；
  · RViz 里要选 ``Durability Policy: Transient Local``（``grasping.rviz`` 里
    这几个显示都已配好）。**默认是 Volatile**，漏配的症状是「这个显示一直
    空着」，而 ``ros2 topic echo``（它默认也是 Volatile）同样看不到东西 ——
    非常容易被误判成「话题没在发」。

``current_target`` 尤其要紧：它一个抓取周期才发一次（几十秒），配成 Volatile
的话 RViz 里绝大多数时间都是空的。

⚠️ 三者都**不是**给控制回路用的。任何「根据遥测做决策」的用法都是错的 ——
它丢一条、晚一条，都不该影响状态机。

关于 ``current_target`` 为什么用 ``PoseStamped`` 而不是 Marker
==============================================================
不需要「清除」这条语义：一个目标只在一次尝试期间存在，而**失败被放弃的目标
恰恰是最该看到的**（它就是「为什么这次没抓成」的答案）。所以这里保留**最近
一个**目标，由 ``telemetry`` 里的 ``state`` / ``holding`` 告诉消费者它是否还
有效。用有类型的 ``PoseStamped`` 还顺带让它能被 rqt / 脚本直接消费。

``place_slots`` 的三种颜色
==========================
| 颜色 | 含义 |
|---|---|
| 绿 | 空着，可以放 |
| 灰 | **规划器已经在这个槽位上走完了一次放置动作** |
| 橙 | **被跳过** —— 接近位姿 IK 解不出来，已放弃（见 ``_on_home_with_object_done``） |

灰与橙必须分开：一个槽位会因「放成功」和「解不出而跳过」两种原因被消费掉，
拿任何**单一计数器**去反推占用（比如「下标小于游标的都算用过」）都会把
跳过的槽位错报成已占用。占用状态现在存在 ``_place_used[]`` 里，且**由状态机
在真的发生时逐格标记**（``mark_slot``），不从计数器推。

（A2 之前这里写的是「``_place_index`` 在两种情况下都会自增」—— 那个自增游标
已经不存在了，现在 ``_place_index`` 只是「正在处理的那一格」，占用看
``_place_used``。结论没变，理由换了。）

⚠️ 灰的准确含义是「**规划器消费掉了这个槽位**」，不是「这里物理上有一个方块」：
``carry_objects:=false``（只走位模式，第一个真机 demo 用）下夹爪不夹取也不张开，
但状态机照样一个槽位一个槽位地走完 —— 那时灰色槽位上其实什么都没有。用
``telemetry`` 里的 ``carry_objects`` 可以分辨这两种情形。
"""

from typing import Callable, Dict, List, Optional, Sequence, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from visualization_msgs.msg import Marker, MarkerArray

# 槽位状态（对应上面表格里的三种颜色）
SLOT_FREE = "free"
SLOT_USED = "used"
SLOT_SKIPPED = "skipped"

_SLOT_RGBA = {
    SLOT_FREE: (0.15, 0.85, 0.25, 0.55),      # 绿
    SLOT_USED: (0.45, 0.45, 0.50, 0.85),      # 灰
    SLOT_SKIPPED: (0.95, 0.55, 0.10, 0.60),   # 橙
}

# 槽位方块标记：5 cm 见方、6 mm 厚的板，顶面与 place_z 齐平（place_z 是
# 「方块静止时的顶面高度」，见 planner_node 的参数注释）。
_SLOT_PLATE_SIDE = 0.05
_SLOT_PLATE_THICK = 0.006

# 两种标记（板 + 序号文字）共用一个命名空间，好让一条 DELETEALL 全部清掉；
# 序号文字用 id + 这个偏移，与板的 id 不撞。
_MARKER_NS = "grasp_planner_slots"
_TEXT_ID_OFFSET = 1000


def _latched() -> QoSProfile:
    """每次新建：QoSProfile 是可变对象，共享一份容易被下游改坏。"""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class PlannerTelemetry:
    """把 planner_node 的状态机、目标点、槽位表挂到 ROS 图上。

    ``snapshot`` 是**无参回调**，返回一个 ``{键: 值}`` 字典，在每次发布遥测时
    被调用。用回调而不是让调用方逐个 set，是为了让数字**永远和状态机同步** ——
    调用方不必记得在十几处更新计数（那种写法一定会漏）。
    """

    def __init__(
        self,
        node: Node,
        base_frame: str,
        place_slots: Sequence[Tuple[float, float]],
        place_z: float,
        snapshot: Optional[Callable[[], Dict[str, object]]] = None,
        namespace: str = "grasp_planner",
    ):
        self._node = node
        self._base_frame = base_frame
        self._place_slots: List[Tuple[float, float]] = [
            (float(x), float(y)) for x, y in place_slots
        ]
        self._place_z = float(place_z)
        self._snapshot = snapshot if snapshot is not None else (lambda: {})

        # 状态机侧的事实（由 planner_node 推动）
        self._state = "STARTING"
        self._finished = False
        self._finish_ok = True
        self._reason = ""
        # 槽位占用情况。**只有这里知道「用过」与「跳过」的区别** ——
        # planner_node 的 _place_used[] 只记「这一格还能不能用」，不区分原因
        # （见模块文档）。
        self._slot_status: Dict[int, str] = {}

        self._pub_target = node.create_publisher(
            PoseStamped, f"/{namespace}/current_target", _latched()
        )
        self._pub_slots = node.create_publisher(
            MarkerArray, f"/{namespace}/place_slots", _latched()
        )
        self._pub_diag = node.create_publisher(
            DiagnosticArray, f"/{namespace}/telemetry", _latched()
        )

        # 一上来就把槽位图挂出去：RViz 在节点刚起、还没开始抓的时候就能看到
        # 计划往哪儿放，而不是等到第一次放置。
        self.publish_slots()
        self.publish_diag()

    # ------------------------------------------------------------------
    # 状态机侧：这些是 planner_node 唯一需要调用的东西
    # ------------------------------------------------------------------
    def set_state(self, state: str) -> None:
        """记下当前状态；**状态真的变了**才发一条遥测。

        ⚠️ 变化检测不是优化，是**必需**：状态机有几处每秒被调很多次
        （``_reset_busy`` 在闲着时会被 10 Hz 的检测流每批调一次），无条件发布
        会让这个话题变成每秒几十条的水龙头 —— 而且把 bag 里真正的状态时间线
        彻底淹掉。过滤之后，这个话题就是一份干净的「状态变化记录」。

        代价：同一状态内的计数变化不会单独发布。可以接受 —— 每次状态切换都会
        带上**当时**的全部计数，而 ``note_finished()`` 会补一条最终快照。
        需要「状态没变但要把计数推出去」的场合，用 :meth:`set_flags`。
        """
        if state == self._state:
            return
        self._state = state
        self.publish_diag()

    def set_target(self, x: float, y: float, z: float) -> None:
        """发布当前瞄准的 3D 点（``base_frame`` 系）。

        朝向一律用单位四元数：**这个目标本来就没有朝向**（它是一个点，
        ``/compute_ik`` 自己按 seed 解姿态），编一个假的 rpy 只会误导。
        """
        msg = PoseStamped()
        msg.header.frame_id = self._base_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._pub_target.publish(msg)

    def mark_slot(self, index: int, status: str) -> None:
        """标记某个槽位**用过**或**被跳过**，并立即重画。

        越界的 index 会被忽略并告警 —— 这是画图，不该让一个显示问题把
        状态机带崩。
        """
        if not 0 <= index < len(self._place_slots):
            self._node.get_logger().warn(
                f"遥测：槽位下标 {index} 越界（共 {len(self._place_slots)} 个），忽略"
            )
            return
        self._slot_status[index] = status
        self.publish_slots()
        self.publish_diag()

    def note_finished(self, reason: str, ok: bool = True) -> None:
        """收工。

        ``ok=False`` 表示**不是正常收工**（卡住 / 恢复流程自身失败 / 抓在手里
        却没地方放）—— 此时诊断级别是 ERROR，且**保持** ERROR，方便在
        rqt_robot_monitor 或事后脚本里一眼挑出有问题的那些 run。
        """
        self._finished = True
        self._finish_ok = bool(ok)
        self._reason = reason
        self._state = "FINISHED"
        self.publish_diag()

    # ------------------------------------------------------------------
    # 订阅方（状态机之外）可以调，用来同步纯显示用的事实
    # ------------------------------------------------------------------
    def set_flags(self, holding: Optional[bool] = None,
                  recovering: Optional[bool] = None) -> None:
        """手工同步 ``holding`` / ``recovering`` 这两个标志。

        ⚠️ 正常情况下**不需要**调它们：``snapshot`` 里带这两个键就够了。
        只有「改了标志但状态名没变」的时刻（例如 ``_evaluate_grasp`` 里判空夹
        后清 ``_holding``）才需要补一次，否则诊断级别会停在过期的 WARN 上。
        """
        snap = self._snapshot()
        if holding is not None:
            snap["holding"] = bool(holding)
        if recovering is not None:
            snap["recovering"] = bool(recovering)
        self.publish_diag(snap)

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def publish_slots(self) -> None:
        arr = MarkerArray()

        # 先 DELETEALL：万一槽位表变短（重配参数），旧标记不会留在 RViz 里。
        clear = Marker()
        clear.header.frame_id = self._base_frame
        clear.header.stamp = self._node.get_clock().now().to_msg()
        clear.ns = _MARKER_NS
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        stamp = self._node.get_clock().now().to_msg()
        for i, (sx, sy) in enumerate(self._place_slots):
            status = self._slot_status.get(i, SLOT_FREE)
            rgba = _SLOT_RGBA[status]

            plate = Marker()
            plate.header.frame_id = self._base_frame
            plate.header.stamp = stamp
            plate.ns = _MARKER_NS
            plate.id = i
            plate.type = Marker.CUBE
            plate.action = Marker.ADD
            plate.pose.position.x = sx
            plate.pose.position.y = sy
            # 板顶面与 place_z 齐平（place_z 是方块顶面高度）
            plate.pose.position.z = self._place_z - _SLOT_PLATE_THICK / 2.0
            plate.pose.orientation.w = 1.0
            plate.scale.x = _SLOT_PLATE_SIDE
            plate.scale.y = _SLOT_PLATE_SIDE
            plate.scale.z = _SLOT_PLATE_THICK
            plate.color.r, plate.color.g, plate.color.b, plate.color.a = rgba
            arr.markers.append(plate)

            # 序号：现场对着日志里的 `去放置位[N]` 能直接对上
            text = Marker()
            text.header.frame_id = self._base_frame
            text.header.stamp = stamp
            text.ns = _MARKER_NS
            text.id = _TEXT_ID_OFFSET + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = sx
            text.pose.position.y = sy
            text.pose.position.z = self._place_z + 0.03
            text.pose.orientation.w = 1.0
            text.scale.z = 0.025          # 字高（TEXT_VIEW_FACING 只用 z）
            text.color.r = text.color.g = text.color.b = 0.95
            text.color.a = 0.9
            text.text = f"{i} {status}"
            arr.markers.append(text)

        self._pub_slots.publish(arr)

    def publish_diag(self, snap: Optional[Dict[str, object]] = None) -> None:
        if snap is None:
            snap = self._snapshot()

        used = sum(1 for s in self._slot_status.values() if s == SLOT_USED)
        skipped = sum(1 for s in self._slot_status.values() if s == SLOT_SKIPPED)

        status = DiagnosticStatus()
        status.name = "grasp_planner"
        status.hardware_id = self._base_frame
        status.values = [KeyValue(key="state", value=self._state)]
        for key in sorted(snap):
            value = snap[key]
            if isinstance(value, bool):
                value = "true" if value else "false"
            elif isinstance(value, float):
                value = f"{value:.4f}"
            status.values.append(KeyValue(key=key, value=str(value)))
        status.values.extend([
            KeyValue(key="slots_total", value=str(len(self._place_slots))),
            KeyValue(key="slots_used", value=str(used)),
            KeyValue(key="slots_skipped", value=str(skipped)),
            KeyValue(key="slots_free",
                     value=str(len(self._place_slots) - used - skipped)),
        ])
        if self._reason:
            status.values.append(KeyValue(key="finish_reason", value=self._reason))

        status.level = self._level(snap)
        status.message = self._reason if self._reason else self._state

        arr = DiagnosticArray()
        arr.header.stamp = self._node.get_clock().now().to_msg()
        arr.status = [status]
        self._pub_diag.publish(arr)

    def _level(self, snap: Dict[str, object]) -> int:
        """诊断级别。**故意做得粗**：真正有信息量的是 ``finish_reason`` 文本。"""
        if self._finished and not self._finish_ok:
            return DiagnosticStatus.ERROR     # 卡住 / 恢复失败 —— 要人来看
        if snap.get("recovering"):
            return DiagnosticStatus.WARN      # 正在走失败恢复
        if snap.get("holding"):
            return DiagnosticStatus.WARN      # 夹爪里可能夹着东西
        return DiagnosticStatus.OK
