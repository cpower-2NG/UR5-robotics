#!/usr/bin/env python3
"""AG-95：**初始化**，可选**合拢**。⚠️ **这是全仓库唯一会主动驱动夹爪的脚本。**

为什么需要它
============
AG-95 **断电就丢初始化**（指示灯转红闪、位置回读归零），而
`grasp_planner/gripper.py` **有意不写 `0x0100`**（`_write_position` 的注释：
「不写 0x0100：那会让夹爪做一次全行程自检」）。所以**没有任何自动路径会
初始化它** —— 未初始化时 `0x0202` 恒 0，而 `close()` 会**静默假成功**
（详见 `docs/待执行命令.md` §丙-5）。这个脚本就是那个缺失的手动步骤。

闸门（**默认什么都不做**）
==========================
不带 `--go` / `--close` 时**一个寄存器都不写**，只读状态并打印「本来会写什么」。
真写必须显式给开关。这是本仓库对「会动的东西」的一贯做法
（对照 `ag95_dry_run: true` 的默认值）。

**写之前请确认一件具体的物理事实**（不是「用户说过可以」）：
  · **两指周围悬空**（合拢时指尖会沿法兰轴**再伸出去 15.6 mm**，见下）；
  · 臂使能时是自锁的（写指不会荡整条臂），臂没使能就别写；
  · 指面上没有绑针之类的东西。

方向（**顺序有讲究**）
======================
官方尺寸：`tool0`→指尖 **张开 184.6 / 闭合 200.2** ⇒ **合拢时指尖再探出 15.6 mm**。

| 步 | 指尖怎么走 | 风险 |
|---|---|---|
| ① 初始化（写 `0x0100 = 0x01`） | **收回** 15.6 mm（走到全开） | **低** —— 两指之间即便有东西也会被**放开** |
| ② 合拢（写 `0x0103 = 0`） | **伸出** 15.6 mm | 唯一会夹到/顶到东西的动作，**周围必须空** |

所以本脚本**默认只做 ①**，② 要额外给 `--close`。

**两档可以分开给**（2026-10-07 之前不行 —— `--close` 必须搭 `--go`）：

| 给什么 | 做什么 | 什么时候用 |
|---|---|---|
| `--go` | 只初始化，停在**全开** | 起栈前过 P1 闸门（规划器起栈时自己会 `open()`，见 `planner_node.py`） |
| `--close` | **只合拢**，不碰 `0x0100` | 夹爪**已经初始化**、只要闭合态。**碰点标定**要的就是这一档：TCP `0/0/204.2` 只在**闭合**时成立（张开时真指尖在 188.6，差 **15.6 mm**）。以前只能 `--go --close`，等于为了合拢把**全行程自检**重走一遍 —— 白动一次 |
| `--go --close` | 初始化 + 合拢 | 上电后第一次，一步到位 |

⚠️ `--close` **单独给**时先读 `0x0200`：**不是 1 就拒绝，一个寄存器都不写**。
未初始化时 `0x0202` 恒 0 ⇒ 写目标 0 会「报成功但手指一步没动」（§丙-5 那个静默假成功）。

寄存器（**2026-10-02 实测**，见 `docs/待执行命令.md` §丙-5）
=========================================================
  `0x0100` 初始化命令 —— **只写**，读回恒 0（FC3/FC4 都一样）⇒ **不能用它判状态**
  `0x0200` **疑似**初始化状态（实测初始化后 = 1，**语义未判决**，仅打印参考）
  `0x0201` **疑似**到位/夹取状态（同上，未判决）
  `0x0202` 实时位置（0–1000 千分；`ag95_open_is_high: true` ⇒ **0 = 全闭、1000 = 全开**）

用法
====
    python3 scripts/ag95_init_close.py                # 只读：打印现状 + 本来会写什么
    python3 scripts/ag95_init_close.py --go           # 只做初始化（两指走到全开）
    python3 scripts/ag95_init_close.py --close        # 只合拢（要求 0x0200=1，不碰 0x0100）
    python3 scripts/ag95_init_close.py --go --close   # 初始化 + 合拢到 0 mm

退出码：0 成功、2 环境不满足（串口打不开 / 缺 pymodbus / 写失败 / `--close` 时未初始化）。
"""
import argparse
import inspect
import sys
import time

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0P6UEX-if00-port0"
BAUD, SLAVE = 115200, 1

REG_INIT = 0x0100       # 初始化命令（只写）
REG_FORCE = 0x0101      # 力（45–160 N）
REG_TARGET = 0x0103     # 目标位置（千分）
REG_STATE = 0x0200      # 疑似初始化状态（未判决，仅打印）
REG_GRASP = 0x0201      # 疑似到位状态（未判决，仅打印）
REG_ACTUAL = 0x0202     # 实时位置（千分）
REG_INIT_STATUS = 0x0200  # 初始化状态（**只读**）：0 未初始化 / 2 进行中 / 1 已完成
INIT_READY = 1
INIT_TIMEOUT = 25.0    # 等 0x0200 变 1 的最长秒数（--init-timeout 可覆盖）

FORCE_N = 50            # 与 grasp_planner.yaml 的 ag95_force 一致
TARGET_PERMILLE = 0     # 0 = 全闭（ag95_open_is_high: true）
STROKE_MM = 95.0        # ag95_stroke_mm


def slave_kwarg(cls):
    """pymodbus 3.x 的从站关键字在各版本里改过名，按签名探。"""
    for name in ("device_id", "slave", "unit", "slave_id"):
        if name in inspect.signature(cls.read_holding_registers).parameters:
            return name
    return None


class Gripper:
    """薄封装：读/写各一个寄存器。**唯一会写的地方是 `write()`。**"""

    def __init__(self, client, slave_kw="device_id", slave_id=SLAVE):
        self._c = client
        self._kw = {slave_kw: slave_id}

    def read(self, addr):
        rr = self._c.read_holding_registers(addr, count=1, **self._kw)
        return None if (rr is None or rr.isError()) else int(rr.registers[0])

    def write(self, addr, value):
        rr = self._c.write_register(addr, value, **self._kw)
        return (rr is not None) and not rr.isError()

    def settle(self, timeout=20.0):
        """等 `0x0202` 连续 3 次读数相同（判「停稳了」）。"""
        t0, last, same = time.time(), None, 0
        while time.time() - t0 < timeout:
            pos = self.read(REG_ACTUAL)
            same = same + 1 if pos == last else 0
            if pos is not None:
                last = pos
            if same >= 3:
                return last, time.time() - t0
            time.sleep(0.4)
        return last, time.time() - t0

    def wait_init_done(self, timeout=None):
        """等 **`0x0200` 变成 1**（初始化真正做完）。

        ⚠️ 2026-10-03 实测：**初始化是一次全行程自检，手指张开**又**合拢**都走**
        （用户现场目击），耗时比原来以为的长。当时用 `settle()`（等 `0x0202`
        停稳）判「完成」，而初始化**前半程** `0x0202` 一直是 0 —— 连读三次
        相同就误判成「完成」，于是报「没走到 1000」。真正的完成信号是
        `0x0200` 由 **2（进行中）** 变成 **1（已完成）**。
        返回 ``(状态, 用时)``；超时返回最后一次读到的值。
        """
        timeout = INIT_TIMEOUT if timeout is None else timeout
        t0, last = time.time(), None
        while time.time() - t0 < timeout:
            last = self.read(REG_INIT_STATUS)
            if last == INIT_READY:
                return last, time.time() - t0
            time.sleep(0.4)
        return last, time.time() - t0

    @classmethod
    def open_port(cls):
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError as exc:
            print(f"❌ 缺 pymodbus（{exc}）—— pip install pymodbus")
            return None
        kw = slave_kwarg(ModbusSerialClient)
        if kw is None:
            print("❌ 认不出 pymodbus 的从站关键字")
            return None
        client = ModbusSerialClient(port=PORT, baudrate=BAUD, timeout=1.0,
                                    parity="N", stopbits=1, bytesize=8)
        if not client.connect():
            print(f"❌ 串口打不开：{PORT}")
            return None
        return cls(client, slave_kw=kw)

    def close_port(self):
        self._c.close()


def snapshot(g):
    """只读快照。"""
    return {
        "0x0100 初始化命令(只写)": g.read(REG_INIT),
        "0x0200 疑似初始化状态": g.read(REG_STATE),
        "0x0201 疑似到位状态": g.read(REG_GRASP),
        "0x0202 实时位置": g.read(REG_ACTUAL),
        "0x0101 力": g.read(REG_FORCE),
        "0x0103 目标位置": g.read(REG_TARGET),
    }


def show(title, snap):
    print(f"—— {title} ——")
    for k, v in snap.items():
        extra = ""
        if k.startswith("0x0202") and v is not None:
            extra = f"  （{v / 1000.0 * STROKE_MM:.1f} mm 开口）"
        print(f"  {k:22s} = {v}{extra}")
    print()


def main(argv=None):
    # ⚠️ `global` 必须在**函数最开头**：`INIT_TIMEOUT` 已经在下一条
    #    argparse 的 default 里被引用过，声明晚了就是 SyntaxError。
    global INIT_TIMEOUT
    ap = argparse.ArgumentParser(description="AG-95 初始化（可选合拢）")
    ap.add_argument("--init-timeout", type=float, default=INIT_TIMEOUT,
                    help="等 0x0200 变 1 的最长秒数（初始化是一次全行程"
                         "自检，实测 >1.3 s）")
    ap.add_argument("--go", action="store_true",
                    help="真的写 0x0100 做初始化（两指走到全开）。不给就是只读预演。")
    ap.add_argument("--close", action="store_true",
                    help="合拢到 0 mm。**可以单独给**（只合拢，不重走全行程自检），"
                         "但那时要求 0x0200=1，否则拒绝、一个寄存器都不写")
    args = ap.parse_args(argv)
    # `wait_init_done` 读的是模块级常量，这里按命令行覆盖一次（测试用小的值）。
    INIT_TIMEOUT = args.init_timeout

    g = Gripper.open_port()
    if g is None:
        return 2
    try:
        show("开始前", snapshot(g))

        if not (args.go or args.close):
            print("（未给 --go / --close：**只读预演，一个寄存器都不写**）")
            print("  三种跑法各写什么：")
            print("    --go          0x0100 = 0x01"
                  "                       → 两指走到**全开**（1000）")
            print(f"    --close       0x0101 = {FORCE_N} / 0x0103 = {TARGET_PERMILLE}"
                  f"          → 两指**合拢**到 0 mm（要求 0x0200=1，**不碰** 0x0100）")
            print("    --go --close  上面两段，先开再合")
            print("\n  ⚠️ 合拢时指尖会沿法兰轴**再伸出 15.6 mm** —— 确认周围是空的再给开关。")
            return 0

        # 步骤号按实际会走的段现算：只给 --close 时是从「力」起算的 ①②，不是 ②③。
        circ = "①②③④"
        n = 0

        if args.go:
            # ---- 初始化（安全方向：张开）----
            n += 1
            print(f"{circ[n - 1]} 写 0x0100 = 0x01（单向初始化）→ 两指将**完全张开**…")
            if not g.write(REG_INIT, 0x01):
                print("❌ 初始化写失败")
                return 2
            st, dt = g.wait_init_done()
            pos = g.read(REG_ACTUAL)
            print(f"   初始化{'完成' if st == INIT_READY else '**没等到完成**'}"
                  f"（{dt:.1f}s）：0x0200 = {st}，0x0202 = {pos}  ← 1000 才是完全张开\n")
            if st != INIT_READY:
                print(f"⚠️ 0x0200 = {st}（1 才是已完成；2 = 还在做）—— 多等一会儿再看，"
                      f"别急着下结论")
                return 2
            if pos != 1000:
                print(f"⚠️ 初始化完成了，但 0x0202 读到 {pos}（不是 1000）—— "
                      f"手指可能被东西挡住，查一下")
        else:
            # ---- 只合拢：**先验初始化态**，再发位置指令 ----
            print("（未给 --go：**不写 0x0100**，即**不重走全行程自检**）")
            st = g.read(REG_INIT_STATUS)
            if st is None:
                print("❌ 读不到 0x0200（串口问题）—— 拒绝发位置指令")
                return 2
            if st != INIT_READY:
                name = {0: "未初始化", 2: "初始化进行中"}.get(st, f"未知值 {st}")
                print(f"❌ AG-95 {name}（0x0200={st}）—— **拒绝只合拢**，"
                      f"一个位置指令都不发。")
                print("   未初始化时 0x0202 恒 0，写目标 0 会「报成功但手指一步没动」，"
                      "而且没有任何日志说得出来（§丙-5）。")
                print("   加 --go 一起做："
                      "python3 scripts/ag95_init_close.py --go --close")
                return 2
            print(f"   0x0200 = {st}（已初始化）⇒ 可以只合拢\n")

        # ---- 合拢 ----
        if not args.close:
            show("结果（只做了初始化）", snapshot(g))
            print("（未给 --close：不发位置指令 ⇒ 夹爪停在**全开**）")
            return 0

        n += 1
        print(f"{circ[n - 1]} 写 0x0101 = {FORCE_N}（力）")
        if not g.write(REG_FORCE, FORCE_N):
            print("⚠️ 力写失败，继续（位置才是关键）")
        n += 1
        print(f"{circ[n - 1]} 写 0x0103 = {TARGET_PERMILLE}（目标位置）→ 两指将**合拢**…")
        if not g.write(REG_TARGET, TARGET_PERMILLE):
            print("❌ 位置写失败")
            return 2
        pos, dt = g.settle()
        print(f"   完成（{dt:.1f}s）\n")

        snap = snapshot(g)
        show("结果", snap)
        if snap["0x0202 实时位置"] == 0:
            print("✅ 已合拢到 0.0 mm 且停稳（3 次读数一致）")
        else:
            print(f"⚠️ 停在 {snap['0x0202 实时位置']}（非 0）—— 可能夹着东西，不要硬来")
        print("\n🔴 从现在起**别断电 / 别写 0x0100**，否则这个状态会丢。")
        return 0
    finally:
        g.close_port()


if __name__ == "__main__":
    sys.exit(main())
