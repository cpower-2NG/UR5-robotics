#!/usr/bin/env python3
"""
AG-95 驱动自检 —— 读路径、**dry-run 闸门**、写路径（不需要硬件）
================================================================
`Ag95Gripper` 现在能连、能读、能写，但写由 `ag95_dry_run` 卡着（默认 true）。
这个闸门是**真机上唯一挡住「错误的寄存器写入让夹爪乱动」的东西**，所以它必须被
钉住 —— 本脚本的核心断言只有一条：

    **dry_run=true 时，一个寄存器都不碰（写没有，读也没有）。**

做法是在 pty 上跑一个**手写的 Modbus RTU 从站**（复用 `test_ag95_probe.py` 里的
`_RtuSlave`，它按真的 RTU 帧应答：CRC 校验、从站号过滤、FC3/FC4 读、FC6 写），
然后直接数**从站收到了什么**。断言写在「从站这一侧」而不是「驱动自己报告了什么」：
驱动说「我没写」不算数，从站没收到才算数。

覆盖的失败模式（每一条都对应真机上的一种坏结果）：

  1. dry-run 里偷偷写了 → 手指在没人准备的时候动；
  2. 真实写入模式下**没写力就写位置** → AG-95 用手册默认力去夹，可能夹坏东西；
  3. 写完不等手指到位就回调 → 规划器立刻 `is_empty()`，读到的是**运动中的中间值**；
  4. 写失败（从站回异常）时回调成**成功** → 正是 D52 那一类「没动却被当成到位了」；
  5. 回调在**调用线程**里同步发生 → `_on_close_done` 会在它自己的调用栈里跑，
     与 `SimGripper`（回调来自 action future）时序不一致；
  6. 千分位方向 `open_is_high` 被**当成能自动检测** → 它检测不了（见第 8 段）。

⚠️ 每一段都必须走 :func:`rig` —— 它在退出时**先停从站再关 fd**。顺序反了就会
造出「僵尸从站读下一段的 pty」这个坑（本脚本第一版 13 条 FAIL 全部来自它，
看起来却像驱动写丢了）。详见 `_RtuSlave.stop()` 的文档。

依赖：`pymodbus` 3.x + ROS 环境（`gripper.py` 会 import `action_msgs`）。

⚠️ **缺依赖时退出码是 2，不是 SKIP/0** —— 这一点与隔壁 `test_ag95_probe.py` 不同，
别以为是坏了。理由：那个脚本只有第 2 段要 pymodbus，缺了还能降级跑别的；本脚本
**每一项**都走 pymodbus 的串口客户端，没有可以降级的部分，所以按
`test_real_launch.py` / `test_model_resolution.py` 的先例退 2（环境不满足）。
**不假装跑了一部分**：报一批 SKIP 再退 0，会让人以为「验过了」。

⚠️ 装 pymodbus 的目录要用**追加**方式进 PYTHONPATH（`PYTHONPATH=$d:$PYTHONPATH`）：
写成 `PYTHONPATH=$d` 会把 `setup.bash` 设的 ROS 路径整个盖掉，于是 `action_msgs`
找不到、`grasp_planner.gripper` 导不进来，本脚本会以一堆 FAIL 的形式**误报
「驱动坏了」**。这个坑在写本脚本时当场踩过一次（见 RESTRUCTURE_PLAN 第二十五轮）。

用法:
    source /opt/ros/humble/setup.bash
    python3 scripts/test_ag95_gripper.py
退出码 0 = 全通过；1 = 有 FAIL；2 = 环境不满足（缺 pymodbus / 缺 ROS / 源码树不在）。
"""

import contextlib
import importlib.util
import os
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
GRIPPER_PY = (REPO / "ros2_ws" / "src" / "grasp_planner"
              / "grasp_planner" / "gripper.py")
NODE_PY = GRIPPER_PY.parent / "planner_node.py"
YAML_PATH = (REPO / "ros2_ws" / "src" / "grasp_planner" / "config"
             / "grasp_planner.yaml")
PROBE_PY = REPO / "scripts" / "ag95_probe.py"
PROBE_TEST_PY = REPO / "scripts" / "test_ag95_probe.py"

PASS = SKIP = FAIL = 0
FAILURES = []
SEEN_WRITES = []    # 「从站确实收到过写」的总账，末尾用它当正控
g = None            # grasp_planner.gripper 模块
probe = None
harness = None


def check(name, cond, detail="", *, skip=False):
    global PASS, SKIP, FAIL
    if skip:
        SKIP += 1
        print(f"  SKIP  {name}" + (f"  —— {detail}" if detail else ""))
        return
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        FAILURES.append(name)
        print(f"  FAIL  {name}" + (f"  —— {detail}" if detail else ""))


def load_module(name, path):
    """按路径加载模块（`scripts/` 不是包，只能这么导）。"""
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


class FakeLogger:
    """把日志收进列表供断言。

    这是本脚本能脱离 rclpy 跑的关键：`Ag95Gripper` 只通过
    `node.get_logger().{info,warn,error}` 说话，所以一个假 node 就够了 ——
    不必起 ROS 图、不必 colcon build。
    """

    def __init__(self):
        self.records = []

    def _add(self, level, msg):
        self.records.append((level, msg))

    def info(self, msg):  self._add("info", msg)
    def warn(self, msg):  self._add("warn", msg)
    def error(self, msg): self._add("error", msg)

    def text(self, level=None):
        return "\n".join(m for lv, m in self.records
                         if level is None or lv == level)

    def errors(self, exclude=()):
        """error 级日志的列表，剔除掉 ``exclude`` 里出现过的那些片段。"""
        return [m for lv, m in self.records
                if lv == "error" and not any(e in m for e in exclude)]


class FakeNode:
    def __init__(self):
        self.log = FakeLogger()

    def get_logger(self):
        return self.log


#: 真实写入模式下构造函数**故意**打的那条 error（模式切换必须大声）。
#: 断言「有没有别的 error」时必须把它剔掉，否则那条检查永远 FAIL。
BANNER = "真实写入已启用"


def default_regs(actual=637):
    """637‰ 让两种方向的解释**不相等**（60.5 / 34.5 mm），方向才看得见。

    ⚠️ `AG95_REG_INIT_STATUS`（**0x0200**）给 1 = 已初始化 —— 驱动现在**发位置
    指令前会查它**（未初始化时静默假成功，见 gripper.py 里的闸门注释）。
    不给这个键，从站会对该地址回「非法地址」⇒ 驱动判「读不到状态」⇒ 所有写
    路径全被拒（第一版就是这么把 21 条打红的）。闸门本身另有专门的用例。
    `AG95_REG_INIT_STATE`（0x0100）保留原样：它只是「读回恒 0 的那个只写命令寄存器」，
    留着正好证明**判据不该用它**。
    """
    return {g.AG95_REG_ACTUAL_POS: actual,
            g.AG95_REG_INIT_STATE: 1,
            g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY,
            g.AG95_REG_TARGET_POS: actual,
            g.AG95_REG_FORCE: 50}


@contextlib.contextmanager
def rig(dry_run, *, open_is_high=True, regs=None, slave_cls=None,
        slave_kw=None, port=None, open_mm=95.0, close_mm=0.0):
    """一套完整的试验台：pty + 假从站 + 真的 ``Ag95Gripper``。

    产出 ``(gripper, slave, node, box)``；`box` 由调用方用 :func:`cb` 造。

    退出时**先 ``slave.stop()``（内部 join）再 ``pair.close()``**。顺序不能反，
    理由见 `_RtuSlave.stop()`。
    """
    pair = harness._PtyPair()
    cls = slave_cls or harness._RtuSlave
    slave = cls(pair.master_fd,
                dict(default_regs() if regs is None else regs),
                slave_id=1, **(slave_kw or {})).start()
    node = FakeNode()
    box = []
    try:
        gr = g.Ag95Gripper(node, port=port or pair.port, baudrate=115200,
                           slave_id=1, force=50, stroke_mm=95.0,
                           open_mm=open_mm, close_mm=close_mm,
                           dry_run=dry_run, open_is_high=open_is_high)
        yield gr, slave, node, box
    finally:
        slave.stop()
        pair.close()


@contextlib.contextmanager
def dead_rig():
    """一个**打不开**的端口：pty 立刻关掉，只留一个不存在的 ``/dev/pts/N`` 名字。

    给「构造必须抛错」那两段用 —— 它们要的就是连不上串口。
    """
    pair = harness._PtyPair()
    port = pair.port
    pair.close()
    yield FakeNode(), port


def cb(box):
    """造一个记录**回调所在线程名**的回调。

    回调契约是两个参数 ``(status, result)``（D52）；签名写错不会让节点退出，
    只会让状态机停在原地，所以这里按真实契约调用。
    """
    return lambda s, r: box.append((s, r, threading.current_thread().name))


def wait_done(box, timeout=8.0):
    """等回调（回调在别的线程里）。返回 ``(status, result, thread)`` 或 None。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if box:
            return box[0]
        time.sleep(0.01)
    return None


def exc_reply(slave_id, fc, code):
    """Modbus 异常应答：功能码最高位置 1 + 异常码，CRC 照常带上。"""
    body = bytes([slave_id, fc | 0x80, code])
    crc = harness._crc16(body)
    return body + bytes([crc & 0xFF, crc >> 8])


def make_slave_classes():
    """两个「坏从站」：写一律被拒 / 回读朝反方向走。

    **必须延迟到 `harness` 加载之后**才能定义 —— 它们的基类是按路径从
    `test_ag95_probe.py` 加载来的，写成模块级的类会在 `harness is None` 的
    时候就求值，直接 `AttributeError`（第一版就是这么崩的）。
    """

    class WriteRefusingSlave(harness._RtuSlave):
        """FC6 一律回异常码 4（从站设备故障）—— 模拟「写了没反应」。"""

        def _handle(self, req):
            sid, fc = req[0], req[1]
            if (sid == self.slave_id and fc == 6
                    and harness._crc16(req[:6]) == (req[7] << 8) | req[6]):
                self.seen.append((fc, (req[2] << 8) | req[3]))
                os.write(self.fd, exc_reply(sid, fc, 0x04))
                return
            super()._handle(req)

    class InvertedSlave(harness._RtuSlave):
        """回读朝写下去的值的**反方向**走（模拟固件把方向反着实现）。"""

        def _step_towards_target(self):
            cur = self.regs.get(self.actual_reg, 0)
            want = self.regs.get(self.target_reg, cur)
            if want < cur:
                self.regs[self.actual_reg] = min(1000, cur + self.step)
            elif want > cur:
                self.regs[self.actual_reg] = max(0, cur - self.step)

    return WriteRefusingSlave, InvertedSlave


def main():
    global g, probe, harness

    if not GRIPPER_PY.is_file():
        print(f"源码树不在：{GRIPPER_PY}")
        return 2

    print("AG-95 驱动自检（不需要硬件；用 pty + 手写 Modbus 从站）")
    print(f"仓库: {REPO}")

    try:
        from pymodbus.client import ModbusSerialClient
    except ImportError as exc:
        print(f"\n缺 pymodbus（{exc}）—— 装 3.x："
              f'\n  pip3 install "pymodbus>=3.6"'
              f"\n⚠️ 别用 apt 的 python3-pymodbus（2.1.0，API 不兼容）。")
        return 2

    # 驱动模块 `from action_msgs.msg import GoalStatus` ⇒ 必须先 source ROS。
    try:
        sys.path.insert(0, str(GRIPPER_PY.parent.parent))
        g = load_module("grasp_planner.gripper", GRIPPER_PY)
    except Exception as exc:                                   # noqa: BLE001
        print(f"\n导不进 {GRIPPER_PY.name}：{type(exc).__name__}: {exc}")
        print("多半是没 source ROS 环境："
              "先 `source /opt/ros/humble/setup.bash` 再跑本脚本。")
        return 2

    probe = load_module("ag95_probe", PROBE_PY)
    harness = load_module("ag95_probe_harness", PROBE_TEST_PY)
    WriteRefusingSlave, InvertedSlave = make_slave_classes()
    OK = g.GoalStatus.STATUS_SUCCEEDED
    print(f"pymodbus 的从站号参数名（签名探测）："
          f"{g.ag95_slave_kwarg(ModbusSerialClient)}")
    print(f"本脚本认定的「成功」状态码：STATUS_SUCCEEDED = {OK}")

    # ------------------------------------------------------------------
    print("\n--- 1. 寄存器表与探针保持一致（两份漂移了就会 FAIL）---")
    for mine, theirs in (("AG95_REG_INIT_STATE", "REG_INIT_STATE"),
                         ("AG95_REG_FORCE", "REG_FORCE"),
                         ("AG95_REG_TARGET_POS", "REG_TARGET_POS"),
                         ("AG95_REG_ACTUAL_POS", "REG_ACTUAL_POS")):
        a, b = getattr(g, mine), getattr(probe, theirs)
        check(f"{mine} == 探针的 {theirs}（0x{a:04X}）", a == b,
              f"驱动 0x{a:04X} vs 探针 0x{b:04X}")

    print("\n--- 2. dry-run 是默认值（看参数面，不只是构造函数默认）---")
    node_src = NODE_PY.read_text(encoding="utf-8")
    check('planner_node 声明 ag95_dry_run 默认 True',
          'd("ag95_dry_run", True)' in node_src,
          '节点里没找到 d("ag95_dry_run", True)')
    check("grasp_planner.yaml 里 ag95_dry_run 也是 true",
          "ag95_dry_run: true" in YAML_PATH.read_text(encoding="utf-8"),
          "yaml 里没找到 ag95_dry_run: true")

    # ------------------------------------------------------------------
    print("\n--- 3. 构造与连接 ---")
    with rig(dry_run=True) as (gr, slave, node, _):
        check("dry_run=true 能构造并连上", gr is not None)
        check("describe() 里写明是 DRY-RUN（现场一眼能看出模式）",
              "DRY-RUN" in gr.describe(), gr.describe())
        check("启动时给出 warn 说明只读", "DRY-RUN" in node.log.text("warn"),
              node.log.text("warn"))
        check("启动日志里印出 pymodbus 的从站号参数名（版本坑的现场证据）",
              any(k in node.log.text("warn") for k in g.AG95_SLAVE_KWARGS),
              node.log.text("warn"))
        check("dry_run=true 时**不**打那条「真实写入已启用」",
              BANNER not in node.log.text("error"), node.log.text("error")[:200])

    # 打不开的端口：必须抛错（不静默退化），且指向正确的替代后端
    with dead_rig() as (node, port):
        try:
            g.Ag95Gripper(node, port=port, baudrate=115200, slave_id=1,
                          force=50, stroke_mm=95.0, open_mm=95.0, close_mm=0.0,
                          dry_run=True)
            check("端口打不开时抛错（不静默退化成「永远读不到」）", False,
                  "居然构造成功了 —— dry-run 会因此静默变成「夹爪永远夹着东西」")
        except RuntimeError as exc:
            check("端口打不开时抛错（不静默退化成「永远读不到」）", True)
            check("报错文案给出**正确**的替代后端（gripper_backend:=none）",
                  "gripper_backend:=none" in str(exc), str(exc)[:200])
            check("报错文案给出排查顺序（含 dialout 与重新登录）",
                  "dialout" in str(exc) and "重新登录" in str(exc),
                  str(exc)[:300])

    with dead_rig() as (node, port):
        try:
            g.Ag95Gripper(node, port=port, baudrate=115200, slave_id=1,
                          force=50, stroke_mm=95.0, open_mm=200.0, close_mm=0.0,
                          dry_run=True)
            check("几何越界的配置在连串口之前就被拒", False, "居然构造成功了")
        except ValueError:
            check("几何越界的配置在连串口之前就被拒", True)
        except RuntimeError as exc:
            check("几何越界的配置在连串口之前就被拒", False,
                  f"报的是串口错误而不是配置错误：{str(exc)[:120]}")

    # 真实写入模式：必须**大声**说手指会动
    with rig(dry_run=False) as (gr, slave, node, _):
        check("dry_run=false 时打一条 error 说明手指会动",
              BANNER in node.log.text("error"), node.log.text("error")[:200])
        check("那条 error 里点明前提（拆下臂 / 工作台 / 急停）",
              "拆下" in node.log.text("error")
              and "急停" in node.log.text("error"),
              node.log.text("error")[:300])
        check("describe() 在真实写入模式下不再说 DRY-RUN",
              "DRY-RUN" not in gr.describe(), gr.describe())

    # ------------------------------------------------------------------
    print("\n--- 4. 读路径：千分位 → 毫米，两个方向都对 ---")
    for oih, want in ((True, 60.5), (False, 34.5)):
        with rig(dry_run=True, open_is_high=oih) as (gr, slave, node, _):
            got = gr.current_position()
            check(f"open_is_high={oih}：637‰ ⇒ {want} mm",
                  got is not None and abs(got - want) < 0.05, f"读到 {got}")

    regs = default_regs()
    regs.pop(g.AG95_REG_ACTUAL_POS)            # 从站会回「非法地址」
    with rig(dry_run=True, regs=regs) as (gr, slave, node, _):
        got = gr.current_position()
        check("读位置失败 → 返回 None（不抛异常）", got is None, f"读到 {got}")
        check("读失败时记一条 warn（现场能从日志看到）",
              "读位置失败" in node.log.text("warn"), node.log.text("warn"))
        check("读不到时 is_empty() 保守判「夹住了」（不半空松手）",
              gr.is_empty() is False)

    # ------------------------------------------------------------------
    print("\n--- 5. **dry-run 闸门：一个寄存器都不碰**（本脚本的核心）---")
    for method in ("open", "close"):
        with rig(dry_run=True,
                 regs={g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY, g.AG95_REG_ACTUAL_POS: 1000,
                       g.AG95_REG_TARGET_POS: 1000,
                       g.AG95_REG_FORCE: 50,
                       g.AG95_REG_INIT_STATE: 1}) as (gr, slave, node, box):
            getattr(gr, method)(done_cb=cb(box))
            got = wait_done(box)
            check(f"dry-run 下 {method}() 仍回调成功（状态机不会卡住）",
                  got is not None and got[0] == OK, f"回调 {got}")
            check(f"dry-run 下 {method}() **从站一个寄存器都没收到**"
                  f"（写没有、读也没有）",
                  slave.writes == [] and slave.seen == [],
                  f"写={slave.writes} 读={slave.seen}")
            check(f"dry-run 下 {method}() 在日志里说明「本应写什么」",
                  "DRY-RUN 本应写" in node.log.text("info")
                  and "0x0103" in node.log.text("info"),
                  node.log.text("info")[:200] or "（没有 info）")
            check(f"dry-run 下 {method}() 日志里带上毫米与千分位两个值",
                  "mm →" in node.log.text("info")
                  and "‰" in node.log.text("info"),
                  node.log.text("info")[:200])

    # ------------------------------------------------------------------
    # 2026-10-03 实测：`0x0200` = 0 未初始化 / 2 正在初始化 / 1 已完成。
    # 下面是**闸门挡的那个场景** —— 加了防护就必须测它，否则等于没加。
    print("\n--- 5b. **未初始化闸门**：0x0200 != 1 时绝不写位置 ---")
    for val, why in ((0, "未初始化"), (2, "正在初始化")):
        with rig(dry_run=False, open_is_high=True,
                 regs={g.AG95_REG_INIT_STATUS: val, g.AG95_REG_ACTUAL_POS: 0,
                       g.AG95_REG_TARGET_POS: 0, g.AG95_REG_FORCE: 50}) as \
                (gr, slave, node, box):
            gr.close(done_cb=cb(box))
            got = wait_done(box)
            check(f"★ 0x0200={val}（{why}）⇒ close() **失败**（不静默假成功）",
                  got is not None and got[0] != OK, f"回调 {got}")
            check(f"★ 0x0200={val} ⇒ 从站**一个力/位置都没收到**",
                  slave.writes == [], f"写={slave.writes}")
            check(f"  —— 但**读过** 0x0200（闸门真的查了）",
                  # ⚠️ `seen` 的元素是 **(功能码, 地址)**，不是 (地址, 功能码)
                  #    —— 我第一版按后者解，两条断言假红。
                  any(addr == g.AG95_REG_INIT_STATUS for _, addr in slave.seen),
                  f"读={slave.seen}")
            check(f"★ 0x0200={val} 的报错文案说得清是**哪一步**、怎么修",
                  "拒绝写位置" in node.log.text("error")
                  and "ag95_init_close" in node.log.text("error"),
                  node.log.text("error")[:200] or "（没有 error）")

    # 只读命令寄存器（0x0100）读到什么**都不算数** —— 它是只写的，读回恒 0。
    with rig(dry_run=False, open_is_high=True,
             regs={g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY,
                   g.AG95_REG_INIT_STATE: 0,      # ← 故意给 0
                   g.AG95_REG_ACTUAL_POS: 0, g.AG95_REG_TARGET_POS: 0,
                   g.AG95_REG_FORCE: 50}) as (gr, slave, node, box):
        gr.close(done_cb=cb(box))
        got = wait_done(box)
        check("★ 判据用的是 0x0200：0x0100 读回 0 **不影响**放行（它本来就是恒 0）",
              got is not None and got[0] == OK, f"回调 {got}")

    # 读不到状态（从站不回这个地址）⇒ 必须**拒绝**，不能当成「已初始化」。
    with rig(dry_run=False, open_is_high=True,
             regs={g.AG95_REG_ACTUAL_POS: 0, g.AG95_REG_TARGET_POS: 0,
                   g.AG95_REG_FORCE: 50}) as (gr, slave, node, box):
        gr.close(done_cb=cb(box))
        got = wait_done(box)
        check("★ 读不到 0x0200（非法地址）⇒ 也拒绝（不当成已初始化）",
              got is not None and got[0] != OK, f"回调 {got}")
        check("  报错说的是**串口/链路**，不是「未初始化」",
              "读不到" in node.log.text("error"), node.log.text("error")[:160])

    # dry-run 的契约优先：未初始化时 dry-run 仍然**一个寄存器都不碰**。
    with rig(dry_run=True, open_is_high=True,
             regs={g.AG95_REG_INIT_STATUS: 0, g.AG95_REG_ACTUAL_POS: 0,
                   g.AG95_REG_TARGET_POS: 0, g.AG95_REG_FORCE: 50}) as \
            (gr, slave, node, box):
        gr.close(done_cb=cb(box))
        got = wait_done(box)
        check("★ dry-run + 未初始化：仍**零流量**（闸门不能破坏 dry-run 契约）",
              slave.writes == [] and slave.seen == [],
              f"写={slave.writes} 读={slave.seen}")
        check("  且 dry-run 不因「没初始化」而报错（预演就是什么都不做）",
              got is not None and got[0] == OK, f"回调 {got}")

    # ------------------------------------------------------------------
    print("\n--- 6. 写路径（dry_run=false）：顺序、值、到位、异步 ---")
    with rig(dry_run=False, open_is_high=True,
             regs={g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY, g.AG95_REG_ACTUAL_POS: 1000, g.AG95_REG_TARGET_POS: 1000,
                   g.AG95_REG_FORCE: 50, g.AG95_REG_INIT_STATE: 1},
             slave_kw={"target_reg": g.AG95_REG_TARGET_POS,
                       "actual_reg": g.AG95_REG_ACTUAL_POS, "step": 250}
             ) as (gr, slave, node, box):
        t0 = time.monotonic()
        gr.close(done_cb=cb(box))
        got = wait_done(box)
        elapsed = time.monotonic() - t0
        SEEN_WRITES.extend(slave.writes)
        check("close() 回调成功", got is not None and got[0] == OK,
              f"回调 {got}")
        check("写的是「先力、后位置」",
              [a for a, _ in slave.writes]
              == [g.AG95_REG_FORCE, g.AG95_REG_TARGET_POS],
              f"实际写到：{[(hex(a), v) for a, v in slave.writes]}")
        check("力写的是配置值 50",
              bool(slave.writes) and slave.writes[0][1] == 50,
              f"{slave.writes}")
        check("close 的目标是 0‰（open_is_high=True）",
              len(slave.writes) > 1 and slave.writes[1][1] == 0,
              f"{slave.writes}")
        check("写路径**不碰** 0x0100（初始化会让夹爪走一次全行程自检）",
              all(a != g.AG95_REG_INIT_STATE for a, _ in slave.writes),
              f"{[hex(a) for a, _ in slave.writes]}")
        check("回调发生在**另一个线程**（与 SimGripper 的异步语义一致）",
              got is not None and got[2] != threading.current_thread().name,
              f"回调线程={None if got is None else got[2]}，"
              f"调用线程={threading.current_thread().name}")
        check("等手指到位是**轮询到位就返回**，不是死等超时",
              elapsed < g.AG95_MOVE_TIMEOUT_S / 2,
              f"耗时 {elapsed:.2f}s（超时上限 {g.AG95_MOVE_TIMEOUT_S}s）")
        check("写完之后 is_empty() 判「空夹」（回读已走到闭合位）",
              gr.is_empty() is True)
        check("除了那条模式告示，全程没有别的 ERROR（一致性检查没误报）",
              node.log.errors(exclude=(BANNER,)) == [],
              f"{node.log.errors(exclude=(BANNER,))}")

    with rig(dry_run=False, open_is_high=False,
             regs={g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY, g.AG95_REG_ACTUAL_POS: 0, g.AG95_REG_TARGET_POS: 0,
                   g.AG95_REG_FORCE: 50, g.AG95_REG_INIT_STATE: 1},
             slave_kw={"target_reg": g.AG95_REG_TARGET_POS,
                       "actual_reg": g.AG95_REG_ACTUAL_POS, "step": 250}
             ) as (gr, slave, node, box):
        gr.close(done_cb=cb(box))
        wait_done(box)
        SEEN_WRITES.extend(slave.writes)
        check("open_is_high=False 时 close() 的目标是 1000‰（方向语义被钉住）",
              len(slave.writes) > 1 and slave.writes[1][1] == 1000,
              f"{slave.writes}")

    # ------------------------------------------------------------------
    print("\n--- 7. 写失败必须是**失败**（D52 那一类：没动被当成到位）---")
    with rig(dry_run=False, open_is_high=True, regs=default_regs(actual=1000),
             slave_cls=WriteRefusingSlave) as (gr, slave, node, box):
        gr.close(done_cb=cb(box))
        got = wait_done(box)
        check("写失败时回调**不是** SUCCEEDED", got is not None and got[0] != OK,
              f"回调 {got}")
        check("写失败时 result.error_code != 0（两个信号都说失败）",
              got is not None and got[1] is not None and got[1].error_code != 0,
              f"error_code={None if got is None else got[1].error_code}")
        check("写失败时 error 文案指出是**哪一步**失败（写力 / 写位置）",
              "写力失败" in node.log.text("error"),
              node.log.text("error")[:200] or "（没有 error）")
        check("写失败时从站确实收到了那条 FC6（不是空断言）",
              any(fc == 6 for fc, _ in slave.seen), f"seen={slave.seen}")

    # ------------------------------------------------------------------
    print("\n--- 8. 那条一致性检查的**边界**：抓不到 open_is_high 猜反 ---")
    saved_timeout = g.AG95_MOVE_TIMEOUT_S
    try:
        # 这一段本来要等满 3 s 超时才看得到结论，临时缩短（函数在调用时读全局）
        g.AG95_MOVE_TIMEOUT_S = 0.4
        with rig(dry_run=False, open_is_high=True,
                 regs={g.AG95_REG_INIT_STATUS: g.AG95_INIT_READY, g.AG95_REG_ACTUAL_POS: 500, g.AG95_REG_TARGET_POS: 500,
                       g.AG95_REG_FORCE: 50, g.AG95_REG_INIT_STATE: 1},
                 slave_cls=InvertedSlave,
                 slave_kw={"target_reg": g.AG95_REG_TARGET_POS,
                           "actual_reg": g.AG95_REG_ACTUAL_POS, "step": 250}
                 ) as (gr, slave, node, box):
            gr.close(done_cb=cb(box))
            deadline = time.monotonic() + 5.0
            while (time.monotonic() < deadline
                   and "背离" not in node.log.text("error")):
                time.sleep(0.02)
            err = node.log.text("error")
            check("回读背离写下去的值时会报 ERROR", "背离" in err,
                  err[:200] or "（没有 error）")
            check("报错指向**链路/固件**（这两个词都在），不是让人去调方向",
                  "链路" in err and "固件" in err, err[:300])
            check("报错里**点名**了不该改的那个参数（ag95_open_is_high）",
                  "ag95_open_is_high 选错了" in err, err[:300])
            check("报错里明说「open_is_high 在寄存器流里是自洽的，抓不到」",
                  "自洽" in err, err[:300])
    finally:
        g.AG95_MOVE_TIMEOUT_S = saved_timeout

    # ------------------------------------------------------------------
    print("\n--- 9. shutdown 与正控 ---")
    with rig(dry_run=True) as (gr, slave, node, _):
        gr.shutdown()
        check("shutdown() 之后不再持有 client", gr._client is None)

    check("正控：上面**真的**往从站写过寄存器（否则第 6 段是空断言）",
          len(SEEN_WRITES) >= 4, f"从站一共收到 {len(SEEN_WRITES)} 次写")

    print("\n" + "=" * 72)
    print(f"PASS {PASS}   SKIP {SKIP}   FAIL {FAIL}")
    if FAILURES:
        print("失败项：")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
