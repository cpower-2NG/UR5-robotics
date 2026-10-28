#!/usr/bin/env python3
"""AG-95 速度寄存器 `0x0104` 到底生效没有 —— **用计时判，不用回读判**。

为什么需要它
============
2026-10-27 阶段 2 B2 第 5 轮：`set_speed(40)` 写 `0x0104` 后**回读恒 100**，
写后核对抛错，臂停在抓取位、手指没合。当时的结论是「这台固件不实现 0x0104」。

但**回读不等于生效**：完全可能「写进去了、动作确实慢了、读回的是 flash 里的
出厂默认值」。用户 2026-10-27 提的第二种可能是「速度只能在初始化时指定，
运行时不可变」—— 这两种都要求**再测一次**，而且**判据必须是运动本身**。

所以本脚本的判决口径是：**同样的开合动作，改速度后耗时不耗时**。
  · 20 % 若真生效，同样的行程应慢到约 5 倍；
  · 若耗时不变（比值 ≈ 1），速度就是没生效，与回读无关。

⚠️ 本脚本**会写寄存器、会动手指**（`--go` 之后）。闸门：
  · 不带 `--go`：**一个字节都不写**，只读快照 + 打印本来会做什么；
  · `--go` 之前先确认**物理事实**：两指行程内无手无遮挡、急停就绪；
  · `0x0200 != 1`（未初始化/正在初始化）直接拒，什么都不动。

用法
====
    # 只读：看现在 0x0104/0x0200/0x0202 是多少，以及会做哪几步
    python3 scripts/ag95_speed_probe.py

    # 真跑：先在 500–950 ‰ 之间来回两次做基线，再写 0x0104=20 重复同样动作
    python3 scripts/ag95_speed_probe.py --go

    # 加上「初始化时给速度」这一档（会做一次全行程自检：张开+合拢）
    python3 scripts/ag95_speed_probe.py --go --with-init

    # 冷启动路径：**断电重启后、0x0200 还是 0 时**先写速度再初始化
    # （一初始化就回不去了，只有掉电后这一次机会）
    python3 scripts/ag95_speed_probe.py --go --preinit-speed 20

退出码：0 出了判决、1 有动作失败、2 环境/前置不满足。
"""
import argparse
import inspect
import sys
import time

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0P6UEX-if00-port0"
BAUD, SLAVE = 115200, 1

REG_INIT = 0x0100        # 初始化命令（只写；写它会做一次全行程自检）
REG_FORCE = 0x0101       # 力（45–160 N）
REG_TARGET = 0x0103      # 目标位置（千分位）—— 写它会动
REG_SPEED = 0x0104       # **速度（1–100 %）—— 本次要判的就是它**
REG_INIT_STATUS = 0x0200  # 初始化状态（只读）：0 未初始化 / 2 进行中 / 1 已完成
REG_ACTUAL = 0x0202      # 实时位置（千分，只读）
REG_SAVE = 0x0300        # 参数存 flash（写 1）—— 速度若不存 flash 可能不生效

INIT_READY = 1
INIT_TIMEOUT = 30.0

OPEN_PM, CLOSE_PM = 950, 500   # 测试行程（千分位）：950 全开 ↔ 500 半开
FORCE_N = 50                   # 与 grasp_planner.yaml 的 ag95_force 一致
STROKE_MM = 95.0

#: 判决阈值（耗时比 = 慢速组均值 / 基线组均值）。速度 20 % 若生效约为 5。
RATIO_WORKS = 1.5      # ≥ 它 ⇒ 速度**生效**
RATIO_IGNORED = 1.2    # ≤ 它 ⇒ 速度**没生效**；中间 = 说不清


def slave_kwarg(cls):
    """pymodbus 3.x 的从站关键字各版本改过名，按签名探（与 ag95_init_close 同策）。"""
    for name in ("device_id", "slave", "unit", "slave_id"):
        if name in inspect.signature(cls.read_holding_registers).parameters:
            return name
    return None


class Gripper:
    """薄封装：读/写一个寄存器 + 计时用的「命令并等到位」。"""

    def __init__(self, client, slave_kw="device_id", slave_id=SLAVE):
        self._c = client
        self._kw = {slave_kw: slave_id}

    def read(self, addr):
        rr = self._c.read_holding_registers(addr, count=1, **self._kw)
        return None if (rr is None or rr.isError()) else int(rr.registers[0])

    def write(self, addr, value):
        """写一个寄存器。**从站不回话/异常不抛，返回 False 并说明** ——
        本脚本是探测器，一次失败（例如 0x0300 在这台固件上无应答）不该
        把整轮扫值打断；调用方按返回值自己决定停不停。"""
        try:
            rr = self._c.write_register(addr, value, **self._kw)
        except Exception as exc:                              # 串口/CRC/超时
            print("    !! 写 0x%04X 报错：%s" % (addr, exc))
            return False
        return (rr is not None) and not rr.isError()

    def move_and_time(self, target_pm, timeout=30.0, tol=15, still=10):
        """写目标位置，轮询 `0x0202` 直到到位；返回 (最终位置, 秒, 是否到位)。

        「到位」判据：|实际 − 目标| ≤ `tol` 千分位。若连续 `still` 次读数不变
        且没到位（被夹住/顶住），也提前返回，`是否到位=False` —— 不把「卡住」
        算成「很快」。
        """
        t0 = time.time()
        if not self.write(REG_TARGET, int(target_pm)):
            return None, time.time() - t0, False
        last, same = None, 0
        while time.time() - t0 < timeout:
            pos = self.read(REG_ACTUAL)
            if pos is not None:
                if abs(pos - int(target_pm)) <= tol:
                    return pos, time.time() - t0, True
                same = same + 1 if pos == last else 0
                last = pos
                if same >= still:
                    return pos, time.time() - t0, False
            time.sleep(0.05)
        return last, time.time() - t0, False

    def settle(self, timeout=20.0):
        """等 `0x0202` 连续 3 次读数相同。"""
        t0, last, same = time.time(), None, 0
        while time.time() - t0 < timeout:
            pos = self.read(REG_ACTUAL)
            same = same + 1 if pos == last else 0
            if pos is not None:
                last = pos
            if same >= 3:
                return last, time.time() - t0
            time.sleep(0.2)
        return last, time.time() - t0

    def wait_init_done(self, timeout=INIT_TIMEOUT):
        """等 `0x0200` 变成 1。初始化是**全行程自检**（张开又合拢），别用位置判。"""
        t0, last = time.time(), None
        while time.time() - t0 < timeout:
            last = self.read(REG_INIT_STATUS)
            if last == INIT_READY:
                return last, time.time() - t0
            time.sleep(0.3)
        return last, time.time() - t0

    @classmethod
    def open_port(cls, port=PORT):
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError as exc:
            print("❌ 缺 pymodbus（%s）—— pip install pymodbus" % exc)
            return None
        kw = slave_kwarg(ModbusSerialClient)
        if kw is None:
            print("❌ 认不出 pymodbus 的从站关键字")
            return None
        client = ModbusSerialClient(port=port, baudrate=BAUD, timeout=1.0,
                                    parity="N", stopbits=1, bytesize=8)
        if not client.connect():
            print("❌ 串口打不开：%s" % port)
            return None
        return cls(client, slave_kw=kw)

    def close_port(self):
        self._c.close()


# ---------------------------------------------------------------- 纯逻辑（可测）
def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def judge_ratio(base_s, slow_s):
    """按两组耗时比给判决。返回 ``(比值, 判决串)``。

    比值 ≈1 ⇒ 速度没生效；≈ 100/速度 ⇒ 生效。两组都空 ⇒ 无法判决。
    """
    b, s = mean(base_s), mean(slow_s)
    if not b or not s or b <= 0:
        return None, "无法判决（有效样本不足）"
    r = s / b
    if r >= RATIO_WORKS:
        return r, "**速度生效**（慢 %.2f 倍）" % r
    if r <= RATIO_IGNORED:
        return r, "**速度没生效**（耗时比 %.2f，与基线无异）" % r
    return r, "说不清（耗时比 %.2f，在 %.1f–%.1f 之间）" % (r, RATIO_IGNORED, RATIO_WORKS)


def pm_to_mm(pm):
    return None if pm is None else pm / 1000.0 * STROKE_MM


def run_pair(g, repeat):
    """跑一组「开→关」并把每段耗时记下来。返回 [(动作, 目标‰, 秒, 到位)]。"""
    rows = []
    for i in range(repeat):
        for name, target in (("开 500→950", OPEN_PM), ("关 950→500", CLOSE_PM)):
            pos, dt, ok = g.move_and_time(target)
            rows.append((name, target, dt, ok))
            print("    %-12s 目标 %4d‰ → 实际 %s‰  %5.2f s  %s"
                  % (name, target, pos, dt, "" if ok else "**没到位**"))
            time.sleep(0.3)
    return rows


def run_cold_start(g, args, init_state):
    """冷启动路径：**未初始化时**先写速度、再初始化，然后按耗时判生效。

    为什么要单独一条路：2026-10-27 第一百六十三轮测过「先写 0x0104、再写
    0x0100 初始化」，但那时夹爪**已经初始化**（写速度前 0x0200 = 1）。
    唯一没测过的路径是「**掉电后、未初始化状态下**写速度」——
    断电重启只有一次机会，初始化一旦做过就回不去，所以这里
    `0x0200 == 1` 直接拒，并且**不重试**（重试也测不到这条路径）。

    判据沿用整文件的口径：**看耗时**，不看回读。
    """
    pct = args.preinit_speed
    if not (1 <= pct <= 100):
        print("❌ 速度只接受 1–100 %%，收到 %d" % pct)
        return 2

    if not args.go:
        print("\n  --preinit-speed %d 会做（**未给 --go：一个寄存器都不写**）：" % pct)
        print("    ① 写 0x0104 = %d（此刻 0x0200 ≠ 1，是冷启动状态）" % pct)
        print("    ② 写 0x0100 = 0x01 → 全行程自检（张开+合拢），等 0x0200 变 1")
        print("    ③ 先走到 500‰（不计时，把行程摆正），再计时 500↔950 各一次")
        print("    ④ 判决：实测均值 / 基线 %.2f s ≥%.1f ⇒ 生效；≤%.1f ⇒ 没生效"
              % (args.ref_s, RATIO_WORKS, RATIO_IGNORED))
        return 0

    if init_state == INIT_READY:
        print("\n!! 0x0200 = 1（已初始化）—— **冷启动路径测不了**：初始化一旦做过就回不去。")
        print("   这条路径要的是「掉电后、未初始化时写速度」；"
              "「已初始化时先写速度再初始化」第一百六十三轮测过，无效。")
        return 2

    print("\n① 冷启动写 0x0104 = %d（0x0200 = %r，未初始化）" % (pct, init_state))
    ok = g.write(REG_SPEED, pct)
    print("    写入 %s；回读 = %s（回读只打印，不作判据）"
          % ("成功" if ok else "**失败**", g.read(REG_SPEED)))
    time.sleep(0.3)

    print("② 写 0x0100 = 0x01 → 两指**全行程自检**（张开+合拢），等 0x0200 变 1")
    t0 = time.time()
    if not g.write(REG_INIT, 0x01):
        print("❌ 初始化写失败")
        return 1
    st, dt = g.wait_init_done()
    print("    0x0200 → %s（%.1f s，未计判决——初始化行程与 500↔950 不可比）"
          % (st, dt))
    if st != INIT_READY:
        print("❌ 初始化没完成（0x0200 = %r）—— 别再重复写 0x0100，先查串口/电源" % st)
        return 1
    print("③ 初始化后回读：0x0104 = %s，0x0101 力 = %s，0x0202 = %s‰"
          % (g.read(REG_SPEED), g.read(REG_FORCE), g.read(REG_ACTUAL)))

    print("④ 先走到 %d‰（**不计时**，把下一组的行程摆正）" % CLOSE_PM)
    pos, dt, ok = g.move_and_time(CLOSE_PM)
    print("    → %s‰  %5.2f s  %s" % (pos, dt, "" if ok else "**没到位**"))

    print("⑤ 计时：500↔950 各一次（基线 %.2f s = 第一百六十三轮同行程）" % args.ref_s)
    rows = run_pair(g, 1)
    got = [r[2] for r in rows if r[3]]
    ratio, verdict = judge_ratio([args.ref_s], got)
    print("    实测均值 %s s（n=%d）" % ("—" if not got else "%.3f" % mean(got), len(got)))
    print("    耗时比 = %s ⇒ %s"
          % ("—" if ratio is None else "%.2f" % ratio, verdict))

    print("⑥ 收尾：0x0104 写回 100（%s，回读 %s）"
          % (g.write(REG_SPEED, 100), g.read(REG_SPEED)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="AG-95 速度寄存器 0x0104 生效性判决")
    ap.add_argument("--port", default=PORT, help="串口设备")
    ap.add_argument("--go", action="store_true",
                    help="**真的写寄存器、动手指**（不带它只读）")
    ap.add_argument("--with-init", action="store_true",
                    help="--go 时再试一次「初始化时给速度」（会做全行程自检）")
    ap.add_argument("--speed-pct", type=int, default=20,
                    help="要试的速度百分比（默认 20）")
    ap.add_argument("--repeat", type=int, default=1, help="每组重复次数（默认 1）")
    ap.add_argument("--sweep", default=None,
                    help="逗号分隔的速度值，逐个写+计时（例如 1,20,50,75,100）；"
                         "给了它就不再跑单点判决")
    ap.add_argument("--with-save", action="store_true",
                    help="--go 时：写完速度再写 0x0300=1 存 flash，然后计时")
    ap.add_argument("--preinit-speed", type=int, default=None,
                    help="**冷启动路径**：0x0200 仍为 0（掉电后未初始化）时，先写"
                         "这个速度、再做初始化，然后按耗时判生效。一初始化就回不去，"
                         "所以只有断电重启后能用一次")
    ap.add_argument("--ref-s", type=float, default=0.42,
                    help="冷启动路径的基线秒数（500↔950 各一次；默认取第一百六十"
                         "三轮实测的 0.42 s）")
    args = ap.parse_args(argv)

    if not (1 <= args.speed_pct <= 100):
        print("❌ 速度只接受 1–100 %%，收到 %d" % args.speed_pct)
        return 2

    print("=" * 70)
    print("  AG-95 速度寄存器 0x0104 生效性判决%s" % ("" if args.go else "（只读预演）"))
    print("=" * 70)
    print("  测试行程：500 ↔ 950 千分位（%.1f ↔ %.1f mm），每组 %d 次开+关"
          % (pm_to_mm(CLOSE_PM), pm_to_mm(OPEN_PM), args.repeat))
    if not args.go:
        print("  不带 --go：**一个字节都不写**。--go 时会做：")
        print("    ① 读 0x0200 必须是 1（已初始化），否则直接退出")
        print("    ② 基线：不碰速度，开/关各 %d 次，记耗时" % args.repeat)
        print("    ③ 写 0x0104 = %d，重复同样的开/关" % args.speed_pct)
        print("    ④ 判决：耗时比 ≥%.1f ⇒ 生效；≤%.1f ⇒ 没生效"
              % (RATIO_WORKS, RATIO_IGNORED))
        print("    ⑤ 把 0x0104 写回 100")
        if args.with_init:
            print("    ⑥ --with-init：先写速度、再写 0x0100=1 初始化，"
                  "看 0x0104 是否被锁住")
        print("\n  现在（只读）：")

    g = Gripper.open_port(args.port)
    if g is None:
        return 2
    try:
        snap = {"0x0200 初始化状态": g.read(REG_INIT_STATUS),
                "0x0202 实时位置": g.read(REG_ACTUAL),
                "0x0104 速度设定": g.read(REG_SPEED),
                "0x0101 力设定": g.read(REG_FORCE),
                "0x0103 上次目标位置": g.read(REG_TARGET)}
        for k, v in snap.items():
            extra = "  （%.1f mm 开口）" % pm_to_mm(v) if k.startswith("0x0202") and v is not None else ""
            print("    %-18s = %s%s" % (k, v, extra))
        # 冷启动路径要在「0x0200 必须是 1」那道闸门**之前**分叉：
        # 它测的正是未初始化状态，走下面的常规路径会被直接拒。
        if args.preinit_speed is not None:
            return run_cold_start(g, args, snap["0x0200 初始化状态"])
        if not args.go:
            return 0

        if snap["0x0200 初始化状态"] != INIT_READY:
            print("\n!! 0x0200 = %r ≠ 1 —— 未初始化/正在初始化，**什么都不动**"
                  % snap["0x0200 初始化状态"])
            print("   先跑 python3 scripts/ag95_init_close.py --go（吃单独授权）")
            return 2

        print("\n① 基线（速度寄存器先不动）")
        base = run_pair(g, args.repeat)

        if args.sweep:
            vals = [int(v) for v in args.sweep.split(",")]
            print("\n①' 扫速度值：%s（每组 %d 次开+关）" % (vals, args.repeat))
            base_s = [r[2] for r in base if r[3]]
            print("    基线均值 %.3f s" % (mean(base_s) or -1))
            for v in vals:
                print("  — 0x0104 = %d —" % v)
                print("    写入 %s；回读 = %s" % (g.write(REG_SPEED, v), g.read(REG_SPEED)))
                if args.with_save:
                    print("    写 0x0300=1 存 flash：%s" % g.write(REG_SAVE, 1))
                    time.sleep(0.8)
                    print("    存后回读 0x0104 = %s" % g.read(REG_SPEED))
                rows = run_pair(g, args.repeat)
                r_, verdict = judge_ratio(base_s, [x[2] for x in rows if x[3]])
                print("    耗时比 = %s ⇒ %s"
                      % ("—" if r_ is None else "%.3f" % r_, verdict))
            print("\n  收尾：0x0104 写回 100（%s，回读 %s）"
                  % (g.write(REG_SPEED, 100), g.read(REG_SPEED)))
            if args.with_save:
                # 别把试出来的值留在 flash 里：把 100 也存一次
                print("  再把 100 存一次 flash（0x0300=1）：%s" % g.write(REG_SAVE, 1))
            return 0

        print("\n② 写 0x0104 = %d（写后回读只**打印**，不作为判据）" % args.speed_pct)
        ok = g.write(REG_SPEED, args.speed_pct)
        rb = g.read(REG_SPEED)
        print("    写入 %s；回读 = %s" % ("成功" if ok else "**失败**", rb))
        time.sleep(0.3)

        print("③ 同动作重跑")
        slow = run_pair(g, args.repeat)

        print("\n④ 判决")
        base_s = [r[2] for r in base if r[3]]
        slow_s = [r[2] for r in slow if r[3]]
        ratio, verdict = judge_ratio(base_s, slow_s)
        print("    基线均值 %.2f s（n=%d）；慢速组均值 %.2f s（n=%d）"
              % (mean(base_s) or -1, len(base_s), mean(slow_s) or -1, len(slow_s)))
        print("    耗时比 = %s ⇒ %s" % ("—" if ratio is None else "%.2f" % ratio, verdict))

        print("\n⑤ 把 0x0104 写回 100")
        print("    写入 %s；回读 = %s" % (g.write(REG_SPEED, 100), g.read(REG_SPEED)))

        if args.with_init:
            print("\n⑥ 初始化时给速度：先写 0x0104=%d，再 0x0100=1 做全行程自检"
                  % args.speed_pct)
            print("    写速度 %s" % g.write(REG_SPEED, args.speed_pct))
            print("    写回读 = %s" % g.read(REG_SPEED))
            print("    写初始化 %s（手指会张开+合拢，全行程）" % g.write(REG_INIT, 0x01))
            st, dt = g.wait_init_done()
            print("    0x0200 → %s（%.1f s）" % (st, dt))
            print("    初始化后 0x0104 = %s" % g.read(REG_SPEED))
            g.settle()
            print("    初始化后再计时一次：")
            after = run_pair(g, 1)
            after_s = [r[2] for r in after if r[3]]
            r2, v2 = judge_ratio(base_s, after_s)
            print("    相对基线耗时比 = %s ⇒ %s"
                  % ("—" if r2 is None else "%.2f" % r2, v2))
            print("    写回 0x0104=100：%s（回读 %s）"
                  % (g.write(REG_SPEED, 100), g.read(REG_SPEED)))
        return 0
    finally:
        g.close_port()


if __name__ == "__main__":
    sys.exit(main())
