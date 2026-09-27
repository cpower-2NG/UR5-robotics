#!/usr/bin/env python3
"""AG-95 只读探测器：在现场把「接线 / 从站号 / 波特率 / 寄存器映射」确认下来。

为什么需要它（W3 的第 1 步，2026-09-27）
==========================================
`Ag95Gripper` 现在是**构造即报错**的桩，理由写在 `grasp_planner/gripper.py`：
没有实物时寄存器映射无法实测，而**写错寄存器会让夹爪产生不可预期的动作**。
所以现场的第一步不是「跑起来」，而是**只读地把映射问出来**。本脚本干这件事。

⚠️ **本脚本一个字节都不写。** 这是硬约束，不是「注意一下」：

  · 只调用 Modbus 的**读**功能码（FC3 保持寄存器 / FC4 输入寄存器）；
  · 源码里**不含任何 write_* 调用**，且 `scripts/test_ag95_probe.py` 会**扫源码**
    钉住这条（结构性保证 —— 靠人记是记不住的）；
  · 「会动的」寄存器（`0x0100` 初始化、`0x0103` 目标位置、`0x0300` 存 flash）
    在本脚本里**只作为读取对象**存在：读 `0x0100` 能看到初始化状态，读
    `0x0103` 能看到「上一次被命令到哪」，这些都不动手指。

拿不到什么，也得说清（免得现场以为读一遍就齐活了）
--------------------------------------------------
只读探测能确认：**接线通不通、从站号是几、波特率对不对、CRC 过不过、寄存器映射
是不是手册上那套**。它**拿不到**：千分位换算的方向与有效行程（那是要「动到若干
位置采样」才有的曲线），以及指尖几何。所以：

  千分位方向**不用写也能定**：用 `--watch`，让手指**用手掰**到全开/全闭，
  看 `0x0202` 的原始值 —— 读到 1000 就是「1000 = 全开」。这正是
  `gripper.py` 里 `mm_to_permille(..., open_is_high=?)` 那个必填参数的来源。

用法
====
  # ① 不接硬件：只把「将要读哪些寄存器」和换算打印出来（离线自检也跑这条）
  python3 scripts/ag95_probe.py --frames

  # ② 有哪些串口（先确认适配器被认出来了）
  python3 scripts/ag95_probe.py --list-ports

  # ③ 扫从站号（逐个读 0x0202；超时 = 该地址上没有设备）
  python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --scan-slaves 1 16

  # ④ 读一个从站的全部候选寄存器
  python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --slave 1 --dump

  # ⑤ 人工掰手指，看原始值怎么变（定方向、看行程；全程只读）
  python3 scripts/ag95_probe.py --port /dev/ttyTHS1 --slave 1 --watch

依赖：`pymodbus` **3.x**（板子上是 Python 3.10）。
  pip3 install "pymodbus>=3.6"     # ⚠️ 别用 apt：那是 2.1.0，API 不兼容
"""

import argparse
import sys
import time
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

# 寄存器地址（大寰 AG 系列手册）。**读**它们都安全，写它们才动手指 ——
# 这里只读，所以整张表都能列出来给现场对照。
REG_INIT_STATE = 0x0100      # 初始化状态/命令（写 0x01/0xA5 会动，耗时 0.5–3 s）
REG_FORCE = 0x0101           # 力（45~160 N）
REG_TARGET_POS = 0x0103      # 目标位置（千分位；写它会动）—— 读回来是「上次命令到哪」
REG_ACTUAL_POS = 0x0202      # 实时位置（千分位，只读）
REG_SAVE_FLASH = 0x0300      # 存 flash（写才有意义）


def _pad(text: str, width: int) -> str:
    """按**显示宽度**补空格。

    `str.ljust` 按字符数补，而中文字符占两列 —— 这张表全是中文名，直接 ljust
    印出来是参差不齐的。现场要照着这表一个个对寄存器，对齐是有用的。
    """
    used = sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)
    return text + " " * max(1, width - used)


@dataclass(frozen=True)
class ReadCandidate:
    """一个候选读取项：地址、功能码、名字、以及读它有什么用。"""
    address: int
    fc: int
    name: str
    why: str


# 先试 FC3（保持寄存器）再试 FC4（输入寄存器）：手册只给了地址，
# 没写是哪一类；两种都试一遍比猜一种强，现场一眼就能看出哪个通了。
def candidates(fc: int) -> List[ReadCandidate]:
    return [
        ReadCandidate(REG_ACTUAL_POS, fc, "实时位置",
                      "千分位。判空与换算方向都看它（用 --watch 人工掰手指最直观）"),
        ReadCandidate(REG_INIT_STATE, fc, "初始化状态",
                      "只读看它是否已初始化；**未初始化时位置回读没有意义**"),
        ReadCandidate(REG_TARGET_POS, fc, "上次命令的目标位置",
                      "读回来是「上一次被写到哪」，用来确认换算标定的量程"),
        ReadCandidate(REG_FORCE, fc, "力设定",
                      "45~160 N。只读回读，用来确认寄存器表与手册一致"),
        ReadCandidate(REG_SAVE_FLASH, fc, "存 flash 状态",
                      "写才有意义；读它多半报非法地址 —— 报错本身也是信息（说明表对）"),
    ]


# ---------------------------------------------------------------- 单位换算
# ⚠️ **不在这里重写换算**：毫米↔千分位的契约在 `grasp_planner/gripper.py` 里，
# 本脚本直接用它 —— 一份实现、一处定义。现场若发现换算不对，改的是那边。
#
# 代价：`gripper.py` 顶部 import 了 `sensor_msgs` 等 ROS 消息包，所以**没有
# ROS 环境时它导入不了**。这不是错误 —— `--frames`（含自检）不需要换算，
# 缺了就把原始千分位照印，并说明缺的是什么。真正读数的那几条模式请先
# `source /opt/ros/humble/setup.bash`（板子上还要 source 工作空间）。
_SRC = Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "grasp_planner"
permille_to_mm = None
_IMPORT_ERR: Optional[BaseException] = None
if _SRC.is_dir():
    sys.path.insert(0, str(_SRC))
try:
    from grasp_planner.gripper import permille_to_mm  # noqa: E402
except ImportError:
    # 源码树不在（比如脚本被单独拷到板子上）时，退到「已 source 的安装空间」。
    # 先撤掉上面那次插入，否则这次尝试与上次完全一样，等于没退。
    if _SRC.is_dir():
        sys.path.remove(str(_SRC))
    try:
        from grasp_planner.gripper import permille_to_mm  # type: ignore # noqa: E402
    except ImportError as exc2:
        # 记**第二次**的异常：两次失败的原因不同（第一次是找不到源码树，
        # 第二次是 ROS 消息包没导入），后者才是「没 source 环境」这个真因，
        # 印出来给人看才有用。
        _IMPORT_ERR = exc2

# 行程：AG-95 是 95 mm。**只用于把回读值印成人能看的数**，不参与任何命令。
AG95_STROKE_MM = 95.0

# 「一个都没应答」是现场最可能遇到的**第一次**结果，所以排查顺序要印在
# 每一个会读端口的模式里 —— 按这个顺序查，每一步都能独立排除掉一类原因。
_TROUBLESHOOT = (
    "一个都没应答时按这个顺序查：\n"
    "  1) A/B 接线是否反了（RS-485 两根线接反是最常见的）\n"
    "  2) 波特率对不对（夹爪出厂 115200，但可能被改过）\n"
    "  3) 从站号在不在扫描范围里（出厂 1，可被改）\n"
    "  4) 夹爪**上电了没**（AG-95 要 24V，不从 USB 取电）\n"
    "  5) 端口权限（`dialout` 组，加完要重新登录）"
)


def interpret(raw: int, stroke_mm: float = AG95_STROKE_MM) -> str:
    """把一个千分位回读值按**两种方向**都解释一遍。

    这是本脚本存在的核心：方向（1000 = 全开还是全闭）手册没写，网上两种说法
    都有，而它决定了 `mm_to_permille(..., open_is_high=?)` 那个必填参数取什么。
    两种都印出来，配合 `--watch` 里人工掰手指的动作，现场一次就判决了。
    """
    if permille_to_mm is None:
        # 板子上第一次跑（工作空间还没部署）就是这个样子，所以提示要能照着做：
        # 光印 `No module named 'grasp_planner'` 会让人以为脚本坏了。
        return (f"{raw} ‰（千分位原值；未换算：{_IMPORT_ERR}）"
                f"—— 换算要 ROS 环境 + 工作空间：source /opt/ros/humble/setup.bash"
                f" && source ros2_ws/install/setup.bash")
    hi = permille_to_mm(raw, stroke_mm, open_is_high=True)
    lo = permille_to_mm(raw, stroke_mm, open_is_high=False)
    return (f"{raw} ‰ ⇒ 开口 {hi:.1f} mm（若 1000=全开）"
            f" / {lo:.1f} mm（若 1000=全闭）")


# ---------------------------------------------------------------- 串口/参数
def _yaml_defaults() -> dict:
    """从 `grasp_planner.yaml` 读串口默认值 —— 与节点**同一份定义**。

    探针自己要连着哪个口，节点也要连哪个口；两处各写一份必然出现
    「探针通了、节点连不上」这种最难查的分歧。

    ⚠️ 这个函数在**建解析器时**就会被调用，所以它**绝不允许抛异常** —— 否则连
    `--frames`（本来是为了在环境不全时还能看一眼）都会打不开。yaml 模块缺失、
    yaml 文件缺失、yaml 格式坏，三种都退到文档里的默认值并**说明退到了什么**。
    """
    try:
        import yaml
    except ImportError:
        return {"port": "/dev/ttyUSB0", "baudrate": 115200, "slave_id": 1,
                "stroke_mm": AG95_STROKE_MM,
                "source": "（默认值：本机没有 PyYAML，读不了 grasp_planner.yaml）"}

    for base in (Path(__file__).resolve().parent.parent / "ros2_ws" / "src"
                 / "grasp_planner" / "config",
                 Path("/opt/ros/humble/share/grasp_planner/config")):
        f = base / "grasp_planner.yaml"
        if f.exists():
            try:
                data = yaml.safe_load(f.read_text(encoding="utf-8"))
            except Exception:
                continue
            p = (data or {}).get("grasp_planner", {}).get("ros__parameters", {})
            if p:
                return {
                    "port": p.get("ag95_port", "/dev/ttyUSB0"),
                    "baudrate": int(p.get("ag95_baudrate", 115200)),
                    "slave_id": int(p.get("ag95_slave_id", 1)),
                    "stroke_mm": float(p.get("ag95_stroke_mm", AG95_STROKE_MM)),
                    "source": str(f),
                }
    return {"port": "/dev/ttyUSB0", "baudrate": 115200, "slave_id": 1,
            "stroke_mm": AG95_STROKE_MM, "source": "（默认值：没找到 grasp_planner.yaml）"}


def _pymodbus():
    """惰性导入 pymodbus，并把「装错版本」这件事说清楚。

    板子上 apt 的 `python3-pymodbus` 是 **2.1.0**，而 2.x/3.x 的 API 不同
    （`read_holding_registers(..., slave=)` vs `unit=`）。装错的表现是
    `TypeError: unexpected keyword argument` —— 看起来像脚本坏了。
    """
    try:
        import pymodbus
        from pymodbus.client import ModbusSerialClient
    except ImportError:
        print('缺少 pymodbus。装 3.x：\n  pip3 install "pymodbus>=3.6"\n'
              '（⚠️ 不要用 apt 的 python3-pymodbus —— 那是 2.1.0，API 不兼容）',
              file=sys.stderr)
        return None, None
    return pymodbus, ModbusSerialClient


def _make_client(ModbusSerialClient, port: str, baudrate: int, timeout: float):
    """造一个 RTU 客户端。**不 connect**，由调用点决定何时开串口。"""
    return ModbusSerialClient(port=port, baudrate=baudrate, bytesize=8,
                              parity="N", stopbits=1, timeout=timeout)


def _read(client, address: int, fc: int, slave: int):
    """发一次**只读**请求，返回 (ok, raw_or_error)。

    `slave=` 是 pymodbus 3.x 的写法（2.x 叫 `unit=`）。这里**不**做兼容分支：
    混用两个大版本只会让「读到的数与预期不符」更难查 —— 直接要求 3.x。
    """
    try:
        if fc == 3:
            rr = client.read_holding_registers(address, count=1, slave=slave)
        else:
            rr = client.read_input_registers(address, count=1, slave=slave)
    except TypeError as exc:                      # pymodbus 2.x 的典型症状
        return False, f"pymodbus 版本不对（{exc}）—— 需要 3.x"
    except Exception as exc:                      # 串口/CRC/超时都归这里
        return False, f"{type(exc).__name__}: {exc}"
    if rr is None:
        return False, "无响应（None）"
    if rr.isError():
        return False, str(rr)
    try:
        return True, int(rr.registers[0])
    except Exception as exc:                      # pragma: no cover
        return False, f"回包解析失败：{exc}"


# ---------------------------------------------------------------- 各模式
def cmd_frames(args) -> int:
    """**不碰串口**：把要读的寄存器表与换算示例打印出来。

    这条路径可以在没有适配器、没有 pymodbus 的机器上跑（CI/自检用的就是它），
    也是现场「先看一眼脚本要干什么」的入口。
    """
    print("AG-95 只读探测 —— 将要读取的寄存器（**全程不写**）")
    print("=" * 72)
    for fc, label in ((3, "FC3 保持寄存器"), (4, "FC4 输入寄存器")):
        print(f"\n{label}:")
        for c in candidates(fc):
            print(f"  0x{c.address:04X}  {_pad(c.name, 20)}{c.why}")
    print("\n换算示例（两种方向都印，现场用 --watch 判断）")
    print("=" * 72)
    for raw in (0, 250, 500, 750, 1000):
        print(f"  {raw:4d} → {interpret(raw, args.stroke_mm)}")
    pymodbus, _ = _pymodbus()
    print(f"\npymodbus: {'已装 ' + pymodbus.__version__ if pymodbus else '未装'}"
          f"（--frames 不需要它；--dump/--watch 需要）")
    print("串口默认值来源: " + _yaml_defaults()["source"])
    return 0


def cmd_list_ports(args) -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("需要 pyserial：pip3 install pyserial", file=sys.stderr)
        return 2
    ports = list(list_ports.comports())
    if not ports:
        print("没有发现任何串口。⚠️ 板子(`lsusb`)上如果没有 USB-串口芯片，"
              "说明适配器还没插 —— 板载 40-pin UART 是 3.3V TTL，接不了 RS-485。")
        return 3
    for p in ports:
        print(f"{p.device}  {p.description}  [{p.hwid}]")
    return 0


def cmd_scan(args, *, timeout: float = 0.3) -> int:
    """扫从站地址：逐个读 `0x0202`。**超时 = 该地址没有设备**，这本身是信息。"""
    pymodbus, ModbusSerialClient = _pymodbus()
    if pymodbus is None:
        return 2
    lo, hi = args.scan_slaves
    print(f"扫描从站 {lo}..{hi} @ {args.port} {args.baudrate} 8N1"
          f"（只读 0x{REG_ACTUAL_POS:04X}，每次超时 {timeout}s）")
    client = _make_client(ModbusSerialClient, args.port, args.baudrate, timeout)
    if not client.connect():
        print(f"打不开串口 {args.port}。检查：设备在不在、`dialout` 组加了没"
              f"（加完要**重新登录**）、有没有别的进程占着它。", file=sys.stderr)
        return 3
    found = []
    try:
        for sid in range(lo, hi + 1):
            ok, val = _read(client, REG_ACTUAL_POS, 3, sid)
            if ok:
                found.append(sid)
                print(f"  从站 {sid}: 0x{REG_ACTUAL_POS:04X} = {interpret(val, args.stroke_mm)}")
            else:
                print(f"  从站 {sid}: 无响应（{val}）")
    finally:
        client.close()
    print(f"\n响应了的从站：{found if found else '（一个都没有）'}")
    if not found:
        print(_TROUBLESHOOT)
    return 0 if found else 3


def cmd_dump(args) -> int:
    """读一个从站的全部候选寄存器，FC3/FC4 都试。"""
    pymodbus, ModbusSerialClient = _pymodbus()
    if pymodbus is None:
        return 2
    print(f"读从站 {args.slave} @ {args.port} {args.baudrate} 8N1（只读）")
    client = _make_client(ModbusSerialClient, args.port, args.baudrate, args.timeout)
    if not client.connect():
        print(f"打不开串口 {args.port}", file=sys.stderr)
        return 3
    answered = 0
    try:
        for fc in (3, 4):
            print(f"\n--- FC{fc} ---")
            for c in candidates(fc):
                ok, val = _read(client, c.address, fc, args.slave)
                if ok:
                    answered += 1
                    extra = (interpret(val, args.stroke_mm)
                             if c.address == REG_ACTUAL_POS else str(val))
                    print(f"  [OK ] 0x{c.address:04X} {_pad(c.name, 20)}= {extra}")
                else:
                    print(f"  [--] 0x{c.address:04X} {_pad(c.name, 20)}: {val}")
    finally:
        client.close()
    print(f"\n有应答的读取项：{answered}")
    if answered:
        print("下一步：用 --watch 人工掰手指，确认哪一类功能码是位置、以及千分位方向；")
        print("确认后把结论写进 grasp_planner.yaml（ag95_* 那几项）与 gripper.py 的注释。")
    else:
        print(_TROUBLESHOOT)
    return 0 if answered else 3


def cmd_watch(args) -> int:
    """持续打印 `0x0202`，让人工掰手指的动作**在原始值上看得见**。

    这一条是「不用写也能拿到换算」的关键：手指用手掰到全开/全闭，看原始值
    落在 0 还是 1000 —— 方向就定了，全程一个字节都没写。
    """
    pymodbus, ModbusSerialClient = _pymodbus()
    if pymodbus is None:
        return 2
    print(f"持续读从站 {args.slave} 的 0x{REG_ACTUAL_POS:04X}（只读，Ctrl-C 停）")
    print("现在**用手**把两指掰到全开，停 2 秒；再掰到全闭，停 2 秒。")
    client = _make_client(ModbusSerialClient, args.port, args.baudrate, args.timeout)
    if not client.connect():
        print(f"打不开串口 {args.port}", file=sys.stderr)
        return 3
    last = None
    t0 = time.time()
    try:
        while True:
            ok, val = _read(client, REG_ACTUAL_POS, 3, args.slave)
            if ok:
                if val != last:
                    print(f"[{time.time() - t0:7.2f}s] {interpret(val, args.stroke_mm)}")
                    last = val
            else:
                print(f"[{time.time() - t0:7.2f}s] 读取失败：{val}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n停止。判方向：掰到全开时读到 1000 ⇒ open_is_high=True；读到 0 ⇒ False。")
    finally:
        client.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    d = _yaml_defaults()
    p = argparse.ArgumentParser(
        description="AG-95 只读探测器（**只读，绝不写寄存器**）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="模式互斥：--frames / --list-ports / --scan-slaves / --dump / --watch\n"
               "默认（不给模式）等价于 --frames。",
    )
    p.add_argument("--port", default=d["port"],
                   help=f"串口设备（默认取自 grasp_planner.yaml: {d['port']}；"
                        f"板载 UART 是 /dev/ttyTHS1）")
    p.add_argument("--baudrate", type=int, default=d["baudrate"],
                   help=f"波特率（默认 {d['baudrate']}）")
    p.add_argument("--slave", type=int, default=d["slave_id"],
                   help=f"从站号（默认 {d['slave_id']}）")
    p.add_argument("--timeout", type=float, default=1.0, help="单次读超时（秒）")
    p.add_argument("--interval", type=float, default=0.2, help="--watch 的轮询间隔（秒）")
    p.add_argument("--stroke-mm", type=float, default=d["stroke_mm"],
                   help=f"行程（默认 {d['stroke_mm']}），只用于把回读值换算成人看的数")
    p.add_argument("--frames", action="store_true",
                   help="只打印要读的寄存器表与换算示例，**不碰串口**")
    p.add_argument("--list-ports", action="store_true", help="列出串口")
    p.add_argument("--scan-slaves", nargs=2, type=int, metavar=("LO", "HI"),
                   help="扫从站号区间（只读 0x0202）")
    p.add_argument("--dump", action="store_true", help="读一个从站的全部候选寄存器")
    p.add_argument("--watch", action="store_true", help="持续读 0x0202（人工掰手指定方向）")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    modes = [args.frames, args.list_ports, bool(args.scan_slaves), args.dump, args.watch]
    if sum(bool(m) for m in modes) > 1:
        print("一次只能用一个模式（--frames / --list-ports / --scan-slaves / --dump / --watch）",
              file=sys.stderr)
        return 2
    if args.list_ports:
        return cmd_list_ports(args)
    if args.scan_slaves:
        return cmd_scan(args)
    if args.dump:
        return cmd_dump(args)
    if args.watch:
        return cmd_watch(args)
    return cmd_frames(args)          # 默认 = --frames（安全的那条）


if __name__ == "__main__":
    sys.exit(main())
