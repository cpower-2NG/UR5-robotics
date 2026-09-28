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

⚠️ 真机后端：**读路径已实现，写路径默认关闭**（`ag95_dry_run`）
==============================================================
`Ag95Gripper` 现在是一个**能连、能读**的实现：串口打开、寄存器读回、
「读不到就返回 None」都已就位。但它的**写**被 `ag95_dry_run` 卡着，默认 `true`：

================================  =========================  ==========================
模式                               寄存器动作                  适用状态
================================  =========================  ==========================
``ag95_dry_run: true``（默认）      **只读**                  接线/从站号/波特率排障
``ag95_dry_run: false``            写 0x0101 + 0x0103（会动）  夹爪**已从臂上拆下**
================================  =========================  ==========================

为什么默认不写：**错误的寄存器写入会让夹爪产生不可预期的动作**，而寄存器映射与
千分位方向（`open_is_high`）都还没有实物复核过。MIGRATION_HANDOFF.md 也要求
夹爪接口「必须现场核实，不能写死」。所以顺序是：

  1. `scripts/ag95_probe.py` 只读探针确认接线 / 从站号 / 波特率 / 寄存器；
  2. 手动掰手指、读一次 `0x0202`，**判决 `ag95_open_is_high`**；
  3. 夹爪拆下臂、放工作台、急停在手边 → 才允许 `ag95_dry_run:=false`。

⚠️ **dry-run 下规划器永远认为夹爪是空的**：手指没真的闭合，`is_empty()` 按
「几乎没动 ⇒ 空夹」判定，于是每次抓取都走失败恢复。这是**正确的**（它确实没夹住），
但意味着 dry-run 是**链路验证模式**，不是抓取模式。要真抓取必须关掉 dry-run。

⚠️ **`open_is_high` 猜反了没有自动检查能抓**（这一点容易想错）：方向约定同时作用在
写入与回读两路上，猜反时命令值、回读值、`is_empty()` 的结论**彼此自洽**，只有实际
手指是反的 —— 寄存器流里没有第二个观察者。判决它**只有一条路**：现场手动把手指掰到
全开、读一次 `0x0202`（`scripts/ag95_probe.py --watch` 就是为这个动作设计的）。
写入后那条一致性检查（`_check_write_took_effect`）管的是另一件事：回读有没有朝
**写下去的值**走。

⚠️ 后端一览（`gripper_backend` 参数）
=====================================
==============  ==========================================================
``sim``         仿真：Robotiq 2F-85，作为 JTC 的第 7 个关节
``ag95``        真机：大寰 AG-95，Modbus —— 读已实现，写由 `ag95_dry_run` 控制
``none``        不驱动夹爪：只用于 ``carry_objects:=false`` 的**只走位**演示
==============  ==========================================================

`none` 是给「第一个真机 demo」用的：那时 AG-95 的 RS485 适配器还不在板上，
而 `ag95` 后端**打不开串口就会拒绝启动** —— 若真机入口按 `camera_source` 默认成
`ag95`，规划器**根本起不来**，连「臂走到目标上方」都做不到。详见 `NullGripper`。
"""

import functools
import inspect
import threading
import time
from typing import Callable, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

#: AG-95 位置寄存器（0x0103 目标 / 0x0202 实时）的满量程，单位**千分位**。
#: 手册写的是「0–1000 千分比」，对应 16 进制 0x0000–0x03E8。
AG95_PERMILLE_FULL = 1000

#: `is_empty()` 判定「手指到头了」的容差（毫米）。保守起点，现场用实测值替换。
AG95_EMPTY_TOLERANCE_MM = 5.0

#: 单次 Modbus 读写的串口超时（秒）。**不能大**：`is_empty()` 是在 ROS 回调里
#: 同步读的，超时多长就阻塞执行器多久。115200 波特率下 8 字节往返是亚毫秒级，
#: 0.2 s 已经是很宽的余量。
AG95_SERIAL_TIMEOUT_S = 0.2

#: 写完目标位置后等手指走到位的上限（秒）。AG-95 全行程 0.5–3 s（手册），
#: 取 3 s 覆盖最慢的一档；超时不报错，只记一条 warn —— 手指可能只是慢，
#: 而 `is_empty()` 会用**当时的真实回读**判断，不依赖这一步「等到位」。
AG95_MOVE_TIMEOUT_S = 3.0

#: 等手指到位时的轮询间隔（秒）。
AG95_POLL_INTERVAL_S = 0.1

# ---------------------------------------------------------------- Modbus 寄存器
#: ⚠️ **这份表是现场必须复核的对象，不是既成事实。** 来源是大寰 AG 系列手册，
#: 但接线、从站号、缩放都要在实物上确认（`scripts/ag95_probe.py` 就是为此写的）。
#: 名字与取值刻意与探针里的 `REG_*` 保持一致 —— 两份若漂移，
#: `scripts/test_ag95_gripper.py` 有一条断言会 FAIL。
AG95_REG_INIT_STATE = 0x0100    # 初始化状态/命令：**写它会动**（本驱动不写它）
AG95_REG_FORCE = 0x0101         # 力（45~160 N）
AG95_REG_TARGET_POS = 0x0103    # 目标位置（千分位）：**写它会动**
AG95_REG_ACTUAL_POS = 0x0202    # 实时位置（千分位）：**只读，不动**

#: 从站号参数在历代 pymodbus 里的名字，**按顺序探测**，先新后旧。
#: 3.8+ 是 `device_id=`、3.6/3.7 是 `slave=`、2.x 是 `unit=`；
#: 板子上实测是 **3.15.0**（`device_id=`）。写死任何一个都会在别的版本上 TypeError。
AG95_SLAVE_KWARGS = ("device_id", "slave", "unit")


@functools.lru_cache(maxsize=None)
def ag95_slave_kwarg(client_cls) -> Optional[str]:
    """探测这一版 pymodbus 用哪个关键字传从站号；认不出返回 ``None``。

    用**签名探测**而不是 try/except：探测出的名字能直接印给现场看，
    失败时也能说清「我看到的签名长什么样」，而不是只说一句「版本不对」。
    与 `ag95_probe.py` 同一策略（那边先踩过这个坑：写死 `slave=` 在 3.15.0 上是
    `TypeError`，而报错文案还把人指成「需要 3.x」—— 板子上明明是 3.x）。
    """
    try:
        params = inspect.signature(client_cls.read_holding_registers).parameters
    except (ValueError, AttributeError):                      # pragma: no cover
        return None
    for name in AG95_SLAVE_KWARGS:
        if name in params:
            return name
    return None


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
    """真机夹爪：大寰 AG-95（Modbus RTU / RS485）。

    读路径已实现（串口、0x0202 回读、毫米换算）；写路径由 ``dry_run`` 控制，
    默认**只读** —— 理由与现场顺序见模块文档的「真机后端」一节。

    行程 0~95 mm：`close` 对应 0 mm、`open` 对应 95 mm（或按现场实测标定）。
    与仿真不同，AG-95 **有力控**（45~160 N 可编程）；本实现只用位置回读判空，
    「夹到物体时力矩上升」这条更强的判据留待有实物时再加（需要读电流寄存器，
    地址未复核，不猜）。
    """

    arm_joint_names: List[str] = []   # 真机夹爪不在 JTC 里

    def __init__(self, node, port: str, baudrate: int, slave_id: int,
                 force: int, stroke_mm: float, open_mm: float, close_mm: float,
                 dry_run: bool = True, open_is_high: bool = True):
        self._node = node
        self._port = port
        self._baudrate = baudrate
        self._slave_id = slave_id
        self._force = force
        self._stroke_mm = stroke_mm
        self._open_mm = open_mm
        self._close_mm = close_mm
        self._dry_run = bool(dry_run)
        self._open_is_high = bool(open_is_high)
        self._client = None
        # 先校验几何再连硬件：三个毫米参数彼此矛盾属于**配置错误**，
        # 应该在碰串口之前就报出来。
        validate_ag95_geometry(open_mm, close_mm, stroke_mm)
        self._connect()

    # ------------------------------------------------------------ 连接
    def _connect(self):
        """开串口。**开不了就抛错**，不退化 —— 理由见下。

        为什么 dry-run 也要求串口能开：dry-run 的用途是「验证这条链路」。
        如果端口打不开还照常启动，`_read_position()` 会一直返回 ``None``，
        而 `is_empty()` 在 ``None`` 时保守地当作「夹着东西」—— 规划器于是
        以为每次抓取都成功了，一路搬着**空气**去放置区。那正是 D56 那一类
        静默错误，只是换了个触发方式。

        所以此处抛错，并在文案里直接给出**真正该用的后端**：
        只想让臂走位、不需要夹爪的演示用 ``gripper_backend:=none``。
        """
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError as exc:
            raise RuntimeError(
                f"没有 pymodbus（{exc}）。装 3.x：\n"
                f'  pip3 install "pymodbus>=3.6"\n'
                f"⚠️ 不要用 apt 的 python3-pymodbus（2.1.0，API 不兼容，"
                f"且从站号参数名不同）。\n"
                f"若本来就只打算让臂走位（第一个真机 demo 就是这种），"
                f"用 gripper_backend:=none + carry_objects:=false，不需要 pymodbus。"
            ) from exc

        kw = ag95_slave_kwarg(ModbusSerialClient)
        self._slave_kwarg = kw
        if kw is None:
            raise RuntimeError(
                f"认不出这一版 pymodbus 的从站号参数名（见过的名字只有 "
                f"{AG95_SLAVE_KWARGS}）；照此下去每次读寄存器都会 TypeError。"
                f"请把 pymodbus 版本记下来反馈。"
            )

        self._client = ModbusSerialClient(
            port=self._port, baudrate=self._baudrate, bytesize=8,
            parity="N", stopbits=1, timeout=AG95_SERIAL_TIMEOUT_S,
        )
        if not self._client.connect():
            raise RuntimeError(
                f"打不开串口 {self._port}（波特率 {self._baudrate}）。"
                f"按这个顺序查：\n"
                f"  1) 适配器插上了吗（`lsusb`；板上目前**没有** USB-串口芯片）\n"
                f"  2) 端口名对不对（USB 适配器通常是 /dev/ttyUSB0，"
                f"板载 40-pin 是 /dev/ttyTHS1）\n"
                f"  3) 权限（`dialout` 组，加完**要重新登录**）\n"
                f"  4) 有没有别的进程占着它\n"
                f"只想让臂走位、不需要夹爪的演示请用："
                f"gripper_backend:=none carry_objects:=false。"
            )

        log = self._node.get_logger()
        if self._dry_run:
            log.warn(
                f"AG-95 **DRY-RUN**（只读）：不会写任何寄存器，"
                f"open/close 只在日志里说明「本应写什么」。"
                f"port={self._port} slave={self._slave_id} "
                f"pymodbus 从站号参数={kw}\n"
                f"⚠️ 此模式下手指不会真的动，因此每次抓取都会判为失败 —— "
                f"它是**链路验证**模式，不是抓取模式。"
            )
        else:
            log.error(
                f"AG-95 **真实写入已启用**（ag95_dry_run:=false）："
                f"open/close 会真的写 0x{AG95_REG_FORCE:04X} 与 "
                f"0x{AG95_REG_TARGET_POS:04X}，**手指会动**。\n"
                f"前提：夹爪已从机械臂上拆下、放在工作台、急停在手边；"
                f"且 ag95_open_is_high 已现场实测确认（当前设的是 "
                f"{self._open_is_high}）。"
            )

    # ------------------------------------------------------------ 寄存器读写
    def _read_raw(self, address: int) -> Tuple[bool, object]:
        """读 1 个保持寄存器。返回 ``(ok, 原值或错误说明)``。**只读，不动手**。"""
        if self._client is None:                              # pragma: no cover
            return False, "串口未连接"
        try:
            rr = self._client.read_holding_registers(
                address, count=1, **{self._slave_kwarg: self._slave_id})
        except TypeError as exc:
            # 签名探测与实际调用不符（pymodbus 又换版本了）。把真实签名带出来，
            # 而不是只说一句「版本不对」——探针那边踩过这个坑。
            try:
                sig = inspect.signature(type(self._client).read_holding_registers)
            except (ValueError, AttributeError):              # pragma: no cover
                sig = "（连签名都读不到）"
            return False, f"调用签名不符：{exc}；实测签名：read_holding_registers{sig}"
        except Exception as exc:                              # 串口/CRC/超时
            return False, f"{type(exc).__name__}: {exc}"
        if rr is None:
            return False, "无响应（None）"
        if rr.isError():
            return False, str(rr)
        try:
            return True, int(rr.registers[0])
        except Exception as exc:                              # pragma: no cover
            return False, f"回包解析失败：{exc}"

    def _write_raw(self, address: int, value: int) -> Tuple[bool, str]:
        """写 1 个保持寄存器（FC6）。返回 ``(ok, 说明)``。**会动**。"""
        if self._client is None:                              # pragma: no cover
            return False, "串口未连接"
        try:
            rr = self._client.write_register(
                address, value, **{self._slave_kwarg: self._slave_id})
        except Exception as exc:                              # 串口/CRC/超时
            return False, f"{type(exc).__name__}: {exc}"
        if rr is None:
            return False, "无响应（None）"
        if rr.isError():
            return False, str(rr)
        return True, f"0x{address:04X} ← {value}"

    def _read_position(self) -> Optional[float]:
        """读 AG-95 实时位置（`0x0202`，只读，不会产生任何动作）。

        **返回值单位是毫米**，与 `self._open_mm` / `self._close_mm` 同量纲 ——
        寄存器读回来的千分位值经
        `permille_to_mm(raw, self._stroke_mm, open_is_high=...)` 换算。
        读不到（超时 / CRC 错 / 串口断了）返回 ``None``。
        """
        ok, raw = self._read_raw(AG95_REG_ACTUAL_POS)
        if not ok:
            self._node.get_logger().warn(
                f"AG-95 读位置失败（0x{AG95_REG_ACTUAL_POS:04X}）：{raw}")
            return None
        return permille_to_mm(int(raw), self._stroke_mm,
                              open_is_high=self._open_is_high)

    def _write_position(self, mm: float):
        """把**毫米**开口宽度写进 AG-95。

        顺序是「先写力（0x0101）再写位置（0x0103）」—— 位置一旦写入手指立即移动，
        所以力必须先到位。**不写 0x0100（初始化）**：那会让夹爪做一次全行程自检
        动作，属于另一类动作，不在本驱动的职责里。

        ``dry_run`` 时**一个寄存器都不写**，只把「本应写什么」打进日志 ——
        日志里带上毫米值、千分位值、寄存器地址与力，现场可以拿它和探针的
        回读值对照。
        """
        permille = mm_to_permille(mm, self._stroke_mm,
                                  open_is_high=self._open_is_high)
        log = self._node.get_logger()

        if self._dry_run:
            log.info(
                f"AG-95 DRY-RUN 本应写：力 {self._force} N → "
                f"0x{AG95_REG_FORCE:04X}；开口 {mm:.1f} mm → {permille} ‰ → "
                f"0x{AG95_REG_TARGET_POS:04X}（未写，手指不动）")
            return

        before = self._read_raw(AG95_REG_ACTUAL_POS)
        ok, msg = self._write_raw(AG95_REG_FORCE, int(self._force))
        if not ok:
            raise RuntimeError(f"AG-95 写力失败：{msg}")
        ok, msg = self._write_raw(AG95_REG_TARGET_POS, permille)
        if not ok:
            raise RuntimeError(f"AG-95 写目标位置失败：{msg}")
        log.info(f"AG-95 已写：力 {self._force} N、开口 {mm:.1f} mm "
                 f"（{permille} ‰）")
        self._wait_until(mm)
        self._check_write_took_effect(mm, before)

    def _wait_until(self, mm: float) -> None:
        """等手指走到目标开口（或超时）。**只读轮询，不发任何命令**。

        超时**不抛错**：AG-95 全行程最慢 3 s，超过可能只是慢，也可能是被物体
        挡住了（那正是「夹住了」的正常情形）。判空由 `is_empty()` 用当时的
        真实回读决定，不依赖这一步。
        """
        deadline = time.monotonic() + AG95_MOVE_TIMEOUT_S
        tol = max(AG95_EMPTY_TOLERANCE_MM, 0.02 * self._stroke_mm)
        last = None
        while time.monotonic() < deadline:
            pos = self._read_position()
            if pos is not None:
                last = pos
                if abs(pos - mm) <= tol:
                    return
            time.sleep(AG95_POLL_INTERVAL_S)
        self._node.get_logger().warn(
            f"AG-95 等手指到位超时（{AG95_MOVE_TIMEOUT_S:.1f}s）："
            f"目标 {mm:.1f} mm，最后回读 "
            f"{'读不到' if last is None else f'{last:.1f} mm'}。"
            f"可能是被物体挡住了（正常），也可能是行程/方向标定不对。")

    def _check_write_took_effect(self, commanded_mm: float, before) -> None:
        """写完之后核对「回读有没有朝**写下去的那个值**走」。

        ⚠️ **这条检查抓不到 `open_is_high` 猜反**，虽然它看起来像能抓。原因：
        方向这个约定**同时**作用在写入与回读两路上（`mm_to_permille` 与
        `permille_to_mm` 都拿它换算），所以猜反时命令的值、回读的值、`is_empty()`
        的结论**彼此完全自洽**，只有**实际手指**的方向是反的 —— 而寄存器流里
        没有第二个观察者能揭示这件事。**当初这一段的文档就写错过这个因果**
        （写成「手指朝反方向动 ⇒ open_is_high 猜反了」），那属于 D66 那一类
        「报错文案把方向指反」的错误，只是这次错在**声称有保护**。

        所以这条检查实际管的是另一件事：**写下去的值，设备有没有朝它走**。
        不一致说明是链路上的问题（没写进去、从站号串了、固件本身把方向反着实现），
        与「方向约定怎么选」无关。

        `open_is_high` 怎么判决，只有一条路：**现场手动把手指掰到全开，读一次
        0x0202** —— `scripts/ag95_probe.py --watch` 就是为这个动作设计的。
        """
        if not before[0]:
            return                       # 写前没读到，无从比较（不是错误）
        after = self._read_raw(AG95_REG_ACTUAL_POS)
        if not after[0]:
            return
        raw_before, raw_after = int(before[1]), int(after[1])
        if raw_after == raw_before:
            return                       # 没动：可能已在该位置，也可能被挡住
        want = mm_to_permille(commanded_mm, self._stroke_mm,
                              open_is_high=self._open_is_high)
        if (want - raw_before) * (raw_after - raw_before) < 0:
            self._node.get_logger().error(
                f"AG-95 回读**背离**了写下去的值：写 {want} ‰（{commanded_mm:.1f} mm），"
                f"写前回读 {raw_before} ‰、写后 {raw_after} ‰。\n"
                f"⇒ 这是**链路/固件**层面的问题（没写进去、从站号串了、"
                f"或设备把方向反着实现），**不是** ag95_open_is_high 选错了 —— "
                f"后者在寄存器流里是自洽的，抓不到（见本函数文档）。\n"
                f"⚠️ 在查清之前**不要**把夹爪装回机械臂。")

    # ------------------------------------------------------------ 对外接口
    def _async(self, work: Callable[[], None], done_cb: Optional[Callable],
               what: str) -> None:
        """在**另一个线程**里做会阻塞的串口 I/O，做完回调 ``done_cb``。

        为什么不在调用栈里同步做：`open()`/`close()` 的调用点
        （`_on_lift_done` / `_on_place_approach_done`）本身就在 ROS 执行器线程里，
        而一次写 + 等到位最长是 ``AG95_MOVE_TIMEOUT_S``。同步做会把执行器
        阻塞三秒 —— 期间 action 结果、订阅回调全都排不上队。
        这与 `SimGripper`（回调来自 action future，天然异步）的语义也一致。

        失败时回 ``(STATUS_ABORTED, error_code=-1)`` 而不是抛异常：
        调用点在回调里，异常没人接。规划器的 `traj_succeeded()` 要求 status
        是 SUCCEEDED **且** error_code == 0（D52 的教训），所以这里两边都要
        写成失败 —— 只写状态码会被「Result 是默认空壳」那条老路绕过。
        """
        once = SimGripper._once(done_cb)
        if once is None:
            return

        def _run():
            try:
                work()
            except Exception as exc:                          # noqa: BLE001
                self._node.get_logger().error(f"AG-95 {what} 失败：{exc}")
                result = FollowJointTrajectory.Result()
                result.error_code = -1
                result.error_string = f"AG-95 {what} 失败：{exc}"
                once(GoalStatus.STATUS_ABORTED, result)
                return
            once(*NullGripper._succeeded())

        threading.Thread(target=_run, name=f"ag95-{what}", daemon=True).start()

    def open(self, done_cb: Optional[Callable] = None) -> None:
        self._async(lambda: self._write_position(self._open_mm), done_cb, "open")

    def close(self, done_cb: Optional[Callable] = None) -> None:
        self._async(lambda: self._write_position(self._close_mm), done_cb, "close")

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

    def shutdown(self) -> None:
        """关串口。规划器退出时调（不调也不会崩，但端口会占着）。"""
        if self._client is not None:
            try:
                self._client.close()
            except Exception:                                 # pragma: no cover
                pass
            self._client = None

    def describe(self) -> str:
        mode = "DRY-RUN 只读" if self._dry_run else "真实写入"
        return (f"Ag95Gripper(port={self._port}, baudrate={self._baudrate}, "
                f"slave_id={self._slave_id}, stroke={self._stroke_mm}mm, "
                f"force={self._force}N, {mode}, "
                f"open_is_high={self._open_is_high})")


class NullGripper(GripperBackend):
    """**不驱动任何夹爪**：给 ``carry_objects:=false``（只走位）的演示用。

    为什么需要它
    ============
    ``make_gripper()`` 是在 ``planner_node.__init__`` 里**无条件**调用的，而
    ``Ag95Gripper`` 会**打开串口**，打不开就拒绝启动。真机入口按
    ``camera_source`` 默认出 ``ag95``，而板上**没有** USB-串口芯片（AG-95 物理上
    没接），于是**规划器根本起不来** —— 连「臂走到目标上方」都做不到。而第一个
    真机 demo 本来就不含抓取，它需要的恰恰是一个「能起来、能走位、不碰夹爪」的后端。

    ⚠️ 它**不是** ``ag95`` + ``ag95_dry_run:=true`` 的替代品：dry-run 仍然要求
    串口能打开（那是「验证链路」，链路不在就该报错），而本后端根本不碰串口。

    为什么要 ``carry_objects:=false`` 配套
    ======================================
    这个后端**只实现 ``open()``**，因为状态机在只走位模式下只会调它：

    ==========================  ==========================  ============
    调用点                       只走位模式下会不会到         本后端行为
    ==========================  ==========================  ============
    ``open()`` :691 初始化       **会**                      静默 no-op + 回调
    ``open()`` :1473 失败恢复     **会**                      同上
    ``open()`` :1393 放置松开     **不会**（:1390 已提前返回）  —
    ``close()`` :1252            **不会**（:1244 已提前返回）  抛 RuntimeError
    ``is_empty()`` :1262         **不会**                     抛 RuntimeError
    ``current_position()`` :1260 **不会**                     抛 RuntimeError
    ==========================  ==========================  ============

    那张表的右两列是**同一个决定的两面**：只走位模式下这三个方法不可达，所以
    它们应当**抛错而不是静默返回**。若 ``carry_objects`` 被误设成 true，抛出
    的信息会直接指出这个配置矛盾；若改成静默 no-op，规划器会以为夹住了空气、
    一路搬到放置区 —— 一个只在真机上才暴露的静默错误。

    ⚠️ 真机上**不要**用它来「先跑通再说」：它不驱动夹爪，但**也不阻止**规划器
    走完整的抓取流程。它只与 ``carry_objects:=false`` 成对使用。
    """

    def __init__(self, node, reason: str = ""):
        self._node = node
        self._reason = reason or "未配置夹爪硬件"
        self.arm_joint_names = []   # 不参与 JTC

    def _no_grasp(self, what: str) -> RuntimeError:
        return RuntimeError(
            f"gripper_backend=none 不驱动夹爪，无法执行 {what}()。"
            f"该后端只与 carry_objects:=false（只走位、不夹持）配套使用；"
            f"若确实要夹取，请换成 sim（仿真）或 ag95（真机）。"
        )

    @staticmethod
    def _succeeded() -> tuple:
        """回一个与 ``_send_trajectory`` 契约一致的 ``(status, result)``。

        必须是**真类型**而不是 ``(0, None)``：规划器的 ``traj_succeeded()``
        要求 ``result is not None`` 且 ``result.error_code == 0``（D52 的教训）。
        ``FollowJointTrajectory.Result()`` 的 ``error_code`` 默认就是 0。
        """
        return (GoalStatus.STATUS_SUCCEEDED, FollowJointTrajectory.Result())

    def _async_done(self, done_cb: Optional[Callable]) -> None:
        """异步回调一次。

        **不能用同步调用**：调用点（``_start_initial_home``）本身就在一个
        「即将发轨迹」的位置上，同步回调会让 ``_send_arm_trajectory`` 嵌在
        它自己的调用栈里跑。真实后端（``SimGripper``/``Ag95Gripper``）的回调
        来自 action 的 future，都在**另一个线程**；这里用定时器复现同样的
        异步语义，也让两条路径的状态机时序一致。

        复用 ``SimGripper._once`` 的「只调一次」包装：那是为动作回调的重复
        触发写的保险，这里同样适用。
        """
        once = SimGripper._once(done_cb)
        if once is None:
            return
        threading.Timer(0.05, once, args=self._succeeded()).start()

    def open(self, done_cb: Optional[Callable] = None) -> None:
        self._node.get_logger().info(
            f"夹爪后端 none（{self._reason}）：跳过「张开」——不发任何夹爪指令"
        )
        self._async_done(done_cb)

    def close(self, done_cb: Optional[Callable] = None) -> None:
        raise self._no_grasp("close")

    def is_empty(self) -> bool:
        raise self._no_grasp("is_empty")

    def current_position(self) -> Optional[float]:
        raise self._no_grasp("current_position")

    def describe(self) -> str:
        return f"NullGripper(reason={self._reason!r}, 不驱动夹爪)"


def make_gripper(node, backend: str, params: dict, send_joint_trajectory: Callable):
    """按 `gripper_backend` 参数构造夹爪后端。

    backend:
      ``sim``       —— 仿真：Robotiq 2F-85，作为 JTC 的一个关节
      ``ag95``      —— 真机：大寰 AG-95，Modbus（写路径由 ``ag95_dry_run`` 控制）
      ``none``      —— 不驱动夹爪：只配 ``carry_objects:=false`` 的只走位演示
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
            dry_run=params["ag95_dry_run"],
            open_is_high=params["ag95_open_is_high"],
        )
    if backend == "none":
        # 不传 reason 也行，但真机排查时「为什么没夹爪」值得写进日志
        return NullGripper(node, reason=params.get("gripper_none_reason", ""))
    raise ValueError(
        f"未知的 gripper_backend: {backend!r}（可选：sim | ag95 | none）"
    )
