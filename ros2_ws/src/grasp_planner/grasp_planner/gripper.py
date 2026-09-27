"""夹爪抽象层：把「仿真夹爪」与「真机夹爪」统一到同一个接口下。

为什么需要这一层（P3-6 的决定，2026-09-27）
============================================
仿真与真机的夹爪**控制接口本质不同**，不是换几个参数能解决的：

=================  ==========================  ============================
                  仿真（附件沿用）              真机（现场读标签确认）
=================  ==========================  ============================
夹爪              Robotiq 2F-85               大寰 AG-95（自适应两指）
接口              JTC 的第 7 个关节            Modbus RTU / RS485 独立设备
行程              0 ~ 0.8 rad                  0 ~ 95 mm（可编程）
力控              无                           45 ~ 160 N（可编程）
空夹判据          关节回读 < 0.75              Modbus 位置回读 vs 命令值
=================  ==========================  ============================

AG-95 **不在机械臂的轨迹控制器里**，所以「把夹爪当第 7 个关节一起发
`FollowJointTrajectory`」这条在真机上根本走不通。

设计：分开命令，而不是合并成一条 7 关节轨迹
=============================================
参考项目的做法是把 6 个臂关节 + 1 个夹爪关节塞进**同一条**轨迹。这既不符合
真机结构，也让夹爪动作与臂动作的失败无法分开诊断。

这里改为**分开命令**：
  - 臂：`GripperBackend.arm_joint_names` 之外的 6 个关节，由规划器发 JTC
  - 夹爪：`gripper.open()` / `gripper.close()`，各后端自己实现

仿真后端借助 JTC 已开启的 ``allow_partial_joints_goal: true``，只发**夹爪那一个
关节**的轨迹——已实测该路径可用（见 RESTRUCTURE_PLAN 的 P3-5 实测）。
真机后端改走 Modbus。两边对规划器暴露同一个接口，规划器的状态机完全不必分叉。

⚠️ 真机后端的现状
==================
`Ag95Gripper` 目前是**会大声报错的桩**，不是可用实现。原因不是偷懒，而是：

  1. 手边没有 AG-95 实物，也没有它的 Modbus 驱动包，**寄存器映射无法实测**；
  2. 错误的寄存器写入会让夹爪产生**不可预期的动作**——这属于有风险操作，
     在未确认接线、波特率、从站地址之前不应猜测；
  3. MIGRATION_HANDOFF.md 明确要求夹爪接口「必须现场核实，不能写死」。

所以这里刻意让它在被调用时**立刻抛出可操作的错误**，而不是写一段看起来能跑、
实际没人验证过的 Modbus 代码。现场确认后在此实现 `_write_position()` 与
`_read_position()` 即可，规划器侧无需改动。
"""

import math
import threading
from typing import Callable, List, Optional

from sensor_msgs.msg import JointState


class GripperBackend:
    """夹爪后端接口。规划器只依赖这几个方法。"""

    #: 参与臂轨迹的关节名。真机夹爪不在 JTC 里，故默认为空。
    arm_joint_names: List[str] = []

    def describe(self) -> str:
        return type(self).__name__

    def open(self, done_cb: Optional[Callable] = None) -> None:
        raise NotImplementedError

    def close(self, done_cb: Optional[Callable] = None) -> None:
        raise NotImplementedError

    def is_empty(self) -> bool:
        """夹爪是否空夹（没夹到东西）。

        用途：抓取后判断是否真的夹住了。**注意**：仿真没有力反馈，只能靠位置
        回读推断；真机 AG-95 有电流/力矩信息，判断可以更可靠。
        """
        raise NotImplementedError

    def current_position(self) -> Optional[float]:
        raise NotImplementedError

    def shutdown(self) -> None:
        pass


class SimGripper(GripperBackend):
    """仿真夹爪：Robotiq 2F-85，作为 JTC 的一部分。

    与真机最大的不同：**它有力反馈的替代品——位置回读**。
    gz_ros2_control 用位置控制驱动夹爪，闭合时若夹到物体，关节会被挡住停在
    中途；空夹则能走到命令位。所以「回读位置 < close_pos - 余量」即判为夹到。
    这个判据是**间接推断**，不是真的测到了力，日志里也如实这么写。
    """

    def __init__(self, node, joint_name: str, open_pos: float, close_pos: float,
                 empty_threshold: float, send_joint_trajectory: Callable,
                 duration: float = 3.0, barely_moved_threshold: float = 0.02):
        self._node = node
        self._joint = joint_name
        self._open_pos = open_pos
        self._close_pos = close_pos
        self._empty_threshold = empty_threshold
        self._barely_moved = barely_moved_threshold
        self._send = send_joint_trajectory   # (joint_names, positions, duration, done_cb)
        self._duration = duration

        self._pos: Optional[float] = None

        self.arm_joint_names = []   # 夹爪单独发轨迹，不混进臂轨迹

        node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == self._joint:
                self._pos = pos

    def _command(self, target: float, done_cb: Optional[Callable]):
        # 只发夹爪这一个关节：靠 JTC 的 allow_partial_joints_goal（已实测可用）
        self._send([self._joint], [target], self._duration, done_cb)

    @staticmethod
    def _once(done_cb: Optional[Callable]) -> Optional[Callable]:
        """包一层「只调用一次」，防止轨迹回调被重复触发时把状态机推进两次。

        ⚠️ 这里曾经写成从 ``self._done_cb`` 取回调，但那个属性从来没被赋值过，
        于是**调用方的回调被静默吞掉**——夹爪动作做完了，规划器却永远等不到
        完成回调，卡在初始位姿、``_first_home_sent`` 一直是 False，所有检测
        都被丢弃。症状是「夹爪动了但规划器毫无反应」，且一句报错都没有。
        """
        if done_cb is None:
            return None
        called = threading.Event()

        def _cb(result):
            if called.is_set():
                return
            called.set()
            done_cb(result)

        return _cb

    def open(self, done_cb: Optional[Callable] = None) -> None:
        self._command(self._open_pos, self._once(done_cb))

    def close(self, done_cb: Optional[Callable] = None) -> None:
        self._command(self._close_pos, self._once(done_cb))

    def current_position(self) -> Optional[float]:
        return self._pos

    def is_empty(self) -> bool:
        """位置回读推断是否空夹。两个判据（均移植自参考项目）：

        1. 回读 ≥ ``empty_threshold``：命令值是 close_pos（0.8），若一路走到
           0.75 以上说明**没被挡住** ⇒ 空夹。
        2. 回读 ≤ ``barely_moved_threshold``：几乎没动过（仍停在张开位）⇒ 轨迹
           根本没生效，同样按空夹处理，否则会带着空夹爪白抬一趟。

        中间那一段（0.02 ~ 0.75）才是「夹到东西、被挡住停在中途」。
        """
        if self._pos is None:
            # 还没收到 /joint_states。宁可当作「夹到了」，也不要因为读不到状态
            # 就把已经抓起来的物体在半空中松开。
            self._node.get_logger().warn(
                "尚未收到夹爪关节状态，无法判断空夹；保守判定为「已夹住」。"
            )
            return False
        return self._pos >= self._empty_threshold or self._pos <= self._barely_moved

    def describe(self) -> str:
        return (f"SimGripper(Robotiq 2F-85, joint={self._joint}, "
                f"open={self._open_pos}, close={self._close_pos}, "
                f"empty_threshold={self._empty_threshold}, "
                f"barely_moved={self._barely_moved})")


class Ag95Gripper(GripperBackend):
    """真机夹爪：大寰 AG-95（Modbus RTU / RS485）——**尚未实现，调用即报错**。

    现场核实后需要补的三件事：
      1. `_write_position()`：按 AG-95 的 Modbus 寄存器表写目标位置与力；
      2. `_read_position()`：读实际位置（回读值 0~95 mm），用于 `is_empty()`；
      3. 串口/总线参数（端口、波特率、从站地址）从 ROS 参数读入。

    行程 0~95 mm：`close` 对应 0 mm、`open` 对应 95 mm（或按现场实测标定）。
    与仿真不同，AG-95 **有力控**（45~160 N 可编程），判空可以比位置推断更可靠：
    夹到物体时电流/力矩会上升，可直接作为判据。
    """

    arm_joint_names: List[str] = []   # 真机夹爪不在 JTC 里

    def __init__(self, node, port: str, baudrate: int, slave_id: int,
                 force: int, stroke_mm: float, open_mm: float, close_mm: float):
        self._node = node
        self._port = port
        self._baudrate = baudrate
        self._slave_id = slave_id
        self._force = force
        self._stroke_mm = stroke_mm
        self._open_mm = open_mm
        self._close_mm = close_mm
        self._connect()

    def _connect(self):
        self._node.get_logger().error(
            "=" * 72 + "\n"
            "AG-95 夹爪后端尚未实现，已拒绝启动真机抓取。\n"
            f"  配置：port={self._port} baudrate={self._baudrate} "
            f"slave_id={self._slave_id} force={self._force} stroke={self._stroke_mm}mm\n"
            "为什么是桩而不是实现：没有实物与驱动包时寄存器映射无法实测，\n"
            "写错寄存器会让夹爪产生不可预期的动作（属有风险操作）。\n"
            "需要现场确认后补：\n"
            "  1) RS485 接线、波特率、从站地址；\n"
            "  2) 采用的 Modbus 驱动（自写 pymodbus 或现成 ROS 驱动包）；\n"
            "  3) 目标位置/力对应的寄存器地址与缩放。\n"
            "详见 grasp_planner/grasp_planner/gripper.py 的模块文档。\n"
            + "=" * 72
        )
        raise RuntimeError(
            "AG-95 后端未实现：请先现场核实 Modbus 接线与寄存器映射，"
            "再在 Ag95Gripper 中实现 _write_position/_read_position。"
            "在此之前请用 gripper_backend:=sim（仅仿真）。"
        )

    def _write_position(self, mm: float):
        raise NotImplementedError

    def _read_position(self) -> Optional[float]:
        raise NotImplementedError

    def open(self, done_cb: Optional[Callable] = None) -> None:
        self._write_position(self._open_mm)

    def close(self, done_cb: Optional[Callable] = None) -> None:
        self._write_position(self._close_mm)

    def current_position(self) -> Optional[float]:
        return self._read_position()

    def is_empty(self) -> bool:
        pos = self._read_position()
        if pos is None:
            return False
        # 命令闭合（close_mm），若回读仍接近 open_mm ⇒ 中途被物体挡住 ⇒ 夹到了。
        # 阈值 5 mm 是保守起点，现场用实测值替换。
        return math.isclose(pos, self._open_mm, abs_tol=5.0)

    def describe(self) -> str:
        return (f"Ag95Gripper(port={self._port}, baudrate={self._baudrate}, "
                f"slave_id={self._slave_id}, stroke={self._stroke_mm}mm, "
                f"force={self._force}N)")


def make_gripper(node, backend: str, params: dict, send_joint_trajectory: Callable):
    """按 `gripper_backend` 参数构造夹爪后端。

    backend:
      ``sim``       —— 仿真：Robotiq 2F-85，作为 JTC 的一个关节
      ``ag95``      —— 真机：大寰 AG-95，Modbus（尚未实现，构造即报错）
    """
    if backend == "sim":
        return SimGripper(
            node,
            joint_name=params["sim_gripper_joint"],
            open_pos=params["sim_gripper_open"],
            close_pos=params["sim_gripper_close"],
            empty_threshold=params["sim_gripper_empty_threshold"],
            send_joint_trajectory=send_joint_trajectory,
            barely_moved_threshold=params["sim_gripper_barely_moved"],
            duration=params["gripper_duration"],
        )
    if backend == "ag95":
        return Ag95Gripper(
            node,
            port=params["ag95_port"],
            baudrate=params["ag95_baudrate"],
            slave_id=params["ag95_slave_id"],
            force=params["ag95_force"],
            stroke_mm=params["ag95_stroke_mm"],
            open_mm=params["ag95_open_mm"],
            close_mm=params["ag95_close_mm"],
        )
    raise ValueError(
        f"未知的 gripper_backend: {backend!r}（可选：sim | ag95）"
    )
