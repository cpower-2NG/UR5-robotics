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

单位契约：参数面是毫米，寄存器面是千分位（D56）
=================================================
**Python 这一侧一律用毫米**：参数 `ag95_stroke_mm` / `ag95_open_mm` /
`ag95_close_mm`、`_write_position()` 的入参、`_read_position()` 的返回值、
`is_empty()` 里的比较，全都是毫米，彼此同量纲。

**只有** `_write_position()` / `_read_position()` 内部与寄存器打交道时才换算成
**千分位（0–1000）** —— AG-95 的 `0x0103`（目标位置）与 `0x0202`（实时位置）
都是千分位，**不是毫米**：

    往 0x0103 写 500  ⇒  走一半行程（95 mm 的夹爪上是 47.5 mm），而不是 500 mm

这个坑很安静：本文件旧版的文档字符串写着「回读值 0~95 mm」，照着它实现就会
把寄存器原值（0–1000）当毫米返回，于是 `is_empty()` 里的
`isclose(回读, 95)` 恒为 False —— **规划器会永远以为夹爪里夹着东西**，
空夹也照样往放置区搬。换算统一走 `mm_to_permille()` / `permille_to_mm()`
两个纯函数，可在没有硬件、没有 ROS 的情况下离线自测
（`scripts/test_ag95_units.py`）。

⚠️ 千分位的**方向**（1000 是全开还是全闭）大寰手册没有写明，网上两种说法都有，
**不要猜**：现场手动把手指掰到全开、读一次 `0x0202` 即可判决（读到 1000 就是
1000=全开）。两个换算函数因此把方向做成**必填的关键字参数**，不给默认值 ——
写实现的人必须自己确认过一次才能写出调用。

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

import threading
from typing import Callable, List, Optional

from sensor_msgs.msg import JointState

#: AG-95 位置寄存器（0x0103 目标 / 0x0202 实时）的满量程，单位**千分位**。
#: 手册写的是「0–1000 千分比」，对应 16 进制 0x0000–0x03E8。
AG95_PERMILLE_FULL = 1000

#: `is_empty()` 判定「手指到头了」的容差（毫米）。保守起点，现场用实测值替换。
AG95_EMPTY_TOLERANCE_MM = 5.0


def validate_ag95_geometry(open_mm: float, close_mm: float,
                           stroke_mm: float) -> None:
    """检查三个毫米参数自洽，不自洽就抛 ``ValueError``。

    做成独立的纯函数（而不是塞在 ``Ag95Gripper.__init__`` 里）是为了**现在就能测**：
    ``Ag95Gripper`` 是构造即报错的桩，构造函数里的任何检查在今天都是死代码。

    报错而不是夹取：这三个值全部来自参数文件，填错属于配置错误，
    而且症状是「夹爪行程不对」——一个只有撞坏东西才会被发现的错误。
    """
    if stroke_mm <= 0:
        raise ValueError(f"ag95_stroke_mm 必须为正，收到 {stroke_mm!r}")
    if close_mm < 0 or open_mm > stroke_mm:
        raise ValueError(
            f"ag95 行程越界：要求 0 <= close_mm <= open_mm <= stroke_mm，"
            f"收到 close={close_mm!r} open={open_mm!r} stroke={stroke_mm!r}"
        )
    if close_mm > open_mm:
        raise ValueError(
            f"ag95 close_mm({close_mm!r}) 不应大于 open_mm({open_mm!r})："
            "本驱动约定「数值 = 两指开口宽度」，闭合的开口不会比张开还大"
        )


def mm_to_permille(mm: float, stroke_mm: float, *, open_is_high: bool) -> int:
    """开口宽度（毫米）→ AG-95 位置寄存器的千分位值。

    :param mm: 两指开口宽度，0 = 全闭、``stroke_mm`` = 全开
    :param stroke_mm: 全行程（AG-95 为 95.0）
    :param open_is_high: 千分位的方向。**必须现场实测确认，无默认值**——
        手册只写了「0–1000 千分比」，没写 1000 对应全开还是全闭。
        确认方法：手动把手指掰到**全开**，读 ``0x0202``：
        读到 1000 ⇒ ``open_is_high=True``；读到 0 ⇒ ``open_is_high=False``。
    :raises ValueError: 毫米值超出 ``[0, stroke_mm]`` —— 越界**不静默饱和**：
        一个悄悄夹到 1000 的命令，现场看起来就是「夹爪行程莫名不对」。
    """
    if stroke_mm <= 0:
        raise ValueError(f"stroke_mm 必须为正，收到 {stroke_mm!r}")
    if not 0.0 <= mm <= stroke_mm:
        raise ValueError(
            f"开口 {mm!r} mm 超出 [0, {stroke_mm!r}] mm；"
            "请检查 ag95_open_mm/ag95_close_mm 与 ag95_stroke_mm 是否自洽"
        )
    frac = mm / stroke_mm
    if not open_is_high:
        frac = 1.0 - frac
    return int(round(frac * AG95_PERMILLE_FULL))


def permille_to_mm(raw: int, stroke_mm: float, *, open_is_high: bool) -> float:
    """AG-95 位置寄存器的千分位值 → 开口宽度（毫米）。

    方向参数的含义与越界行为见 :func:`mm_to_permille`。这里对超出 0–1000 的
    回读值**夹到边界并照常返回**：回读噪声不该让规划器在某一步崩掉，
    而 `is_empty()` 的两个判据都在边界上，夹取不会改变结论。
    """
    if stroke_mm <= 0:
        raise ValueError(f"stroke_mm 必须为正，收到 {stroke_mm!r}")
    raw = min(max(int(raw), 0), AG95_PERMILLE_FULL)
    frac = raw / AG95_PERMILLE_FULL
    if not open_is_high:
        frac = 1.0 - frac
    return frac * stroke_mm


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

        ⚠️ 签名必须与 `_send_trajectory` 的 ``done_cb`` 契约一致：**两个参数**
        ``(goal_status, result)``（D52）。写成单参数 ``_cb(result)`` 会让每次
        夹爪动作都以 ``TypeError`` 收场——而回调是在 action 的 future 里调的，
        异常不会让节点退出，只会让状态机**停在原地**，又一次「动了但没反应」。
        """
        if done_cb is None:
            return None
        called = threading.Event()

        def _cb(status, result):
            if called.is_set():
                return
            called.set()
            done_cb(status, result)

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
      2. `_read_position()`：读实际位置，**返回值必须是毫米**（见模块文档的
         单位契约；寄存器里是千分位，用 `permille_to_mm()` 换算）；
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
        # 先校验几何再连硬件：三个毫米参数彼此矛盾属于**配置错误**，
        # 应该在碰串口之前就报出来（而且今天这条路径的下一行必然抛错，
        # 这个校验是给将来实现 _connect 的人留的门槛）。
        validate_ag95_geometry(open_mm, close_mm, stroke_mm)
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
        """把**毫米**开口宽度写进 AG-95。

        实现要点（现场核实后照此写）：
          1. 先用 `mm_to_permille(mm, self._stroke_mm, open_is_high=...)`
             换算成千分位，**不要**直接把毫米写进寄存器；
          2. 顺序是「先写力（0x0101）再写位置（0x0103）」，位置一旦写入手指
             立即移动；未初始化（0x0100）时位置回读没有意义；
          3. 这是**会动**的写，属有风险操作：必须满足「夹爪已从机械臂上拆下、
             放在工作台、急停在手边」才允许调用（见 RESTRUCTURE_PLAN 的 W3）。
        """
        raise NotImplementedError

    def _read_position(self) -> Optional[float]:
        """读 AG-95 实时位置（`0x0202`，只读，不会产生任何动作）。

        **返回值单位是毫米**，与 `self._open_mm` / `self._close_mm` 同量纲 ——
        寄存器读回来的千分位值必须先经
        `permille_to_mm(raw, self._stroke_mm, open_is_high=...)` 换算。
        读不到（超时 / CRC 错 / 未初始化）返回 ``None``。
        """
        raise NotImplementedError

    def open(self, done_cb: Optional[Callable] = None) -> None:
        self._write_position(self._open_mm)

    def close(self, done_cb: Optional[Callable] = None) -> None:
        self._write_position(self._close_mm)

    def current_position(self) -> Optional[float]:
        return self._read_position()

    def is_empty(self) -> bool:
        """位置回读推断是否空夹。判据与 `SimGripper.is_empty` **同构**，
        但方向相反 —— AG-95 的数值是「两指开口宽度」，越大越开：

        1. 回读 ≤ ``close_mm + 容差``：手指**走到了闭合位**，中间没东西挡着
           ⇒ 空夹。**这是最常见的空夹情形**，旧实现漏掉了它。
        2. 回读 ≥ ``open_mm - 容差``：几乎没动过（仍停在张开位）⇒ 命令根本没
           生效，同样按空夹处理，否则会带着空夹爪白抬一趟。

        中间那一段才是「夹到东西、被手指挡住停在中途」。

        ⚠️ 这里曾写成 `isclose(pos, self._open_mm)`，只覆盖第 2 条：夹到空气
        一路闭合到 0 mm 时它判「夹住了」。再加上模块文档说的单位错位
        （回读值当毫米用），实际效果是 `is_empty()` **恒 False**。
        """
        pos = self._read_position()
        if pos is None:
            # 与 SimGripper 同一种保守取向：读不到状态时宁可当作「夹到了」，
            # 也不要因为一次读取失败就把已经抓起来的物体在半空中松开。
            return False
        return (pos <= self._close_mm + AG95_EMPTY_TOLERANCE_MM
                or pos >= self._open_mm - AG95_EMPTY_TOLERANCE_MM)

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
