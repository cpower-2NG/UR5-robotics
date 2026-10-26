#!/usr/bin/env python3
"""**不动臂**，只验夹爪链路：闭合 → 张开，每次都判「手指真的走了吗」。

为什么单独一个脚本
------------------
2026-10-26 第一个真机循环里，夹爪四段动作全是空操作（驱动的 ``_async()`` 不给
``done_cb`` 就 return），臂却完整走了一趟。事后只能靠现场的人说「没夹」才发现。
把夹爪单独拎出来验，能把「夹爪坏了」和「臂的运动坏了」分开，代价只有一次全行程。

判决用 ``gripper_action.check_moved``：写了指令但回读纹丝不动 ⇒ 判红。
默认**只读**；写寄存器必须加 ``--go``。

用法：
  python3 precision_grip_check.py                 # 只读：打印当前开口，不写
  python3 precision_grip_check.py --go            # 闭合 → 张开（会动）
  python3 precision_grip_check.py --go --leave closed   # 验完停在闭合位
"""
import argparse
import os
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripper_action as ga                                       # noqa: E402

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0P6UEX-if00-port0"
OPEN_MM, CLOSE_MM = 95.0, 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true", help="真的写寄存器（会动手指）")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--force", type=int, default=50)
    ap.add_argument("--leave", choices=["open", "closed"], default="open",
                    help="验完停在哪个位置（默认全开，方便把夹爪装回去）")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from grasp_planner.gripper import Ag95Gripper

    rclpy.init()
    node = Node("precision_grip_check")
    g = Ag95Gripper(node, port=args.port, baudrate=115200, slave_id=args.slave,
                    force=args.force, stroke_mm=OPEN_MM, open_mm=OPEN_MM,
                    close_mm=CLOSE_MM, dry_run=not args.go, open_is_high=True)
    print("夹爪：%s" % g.describe())
    p0 = g.current_position()
    print("当前开口回读：%s mm" % ("读不到" if p0 is None else "%.1f" % p0))
    if p0 is None:
        print("!! 读不到 0x0202 —— 串口或从站号有问题，先别跑真机")
        return 2
    if not args.go:
        print("⇒ 只读模式：一个寄存器都没写。要真验加 --go（手指会动）。")
        return 0

    ok_all = True
    for kind, target in (("close", CLOSE_MM), ("open", OPEN_MM)):
        before = g.current_position()
        t0 = time.time()
        try:
            ga.call_sync(g, kind)
        except ga.GripperActionError as exc:
            print("!! %s 失败：%s" % (kind, exc))
            return 1
        time.sleep(0.3)
        after = g.current_position()
        ok, note = ga.check_moved(before, after, kind, target)
        ok_all = ok_all and ok
        print("%-6s %.1f → %.1f mm（%.1f s，目标 %.1f）%s  %s"
              % (kind, before, after, time.time() - t0, target,
                 "  ✓" if ok else "  **判红**", note))
        if not ok:
            break

    if ok_all and args.leave == "closed" and g.current_position() > CLOSE_MM + 5:
        before = g.current_position()
        ga.call_sync(g, "close")
        print("留在闭合位：%.1f → %.1f mm" % (before, g.current_position()))

    node.destroy_node()
    rclpy.shutdown()
    print("⇒ 夹爪链路%s" % ("通（写下去手指确实走了）" if ok_all else "**不通**"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
