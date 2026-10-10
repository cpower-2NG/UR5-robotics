#!/usr/bin/env python3
"""开/关 UR 的**自由驱动**（人能用手拖动臂），从 ROS 侧发，不必按示教器上那个键。

为什么不用示教器那个 freedrive 键
----------------------------------
easy_handeye2 自己的 README（`Moving the robot` 一节）就警告过：

    the robot driver **may not work while the teaching mode button is pressed**

对我们来说这恰恰是最不能接受的一条 —— 采样要的正是「臂的位姿」，驱动一停，
`base_link → tool0` 就没了，那一拍要么采不到、要么采到脏的。
走驱动自己的 freedrive 控制器**没有这个问题**：是驱动自己在驱动自由驱动，
它全程活着，`/joint_states` 和 TF 一直流。这是本脚本存在的唯一理由。

它到底怎么工作的（板上头文件读出来的，不是猜的）
------------------------------------------------
`ur_controllers/FreedriveModeController` 订阅一个 **topic**：

    ~/enable_freedrive_mode        std_msgs/msg/Bool

`~` 展开成控制器名，所以默认是 `/freedrive_mode_controller/enable_freedrive_mode`。
参数 `inactive_timeout` 默认 **1 秒**：超过这么久没收到新消息，
控制器**自己把自由驱动关掉**（`"Freedrive mode will be deactivated since no new
message received."`）。所以：

  · 要一直开着，就得**持续重发**（本脚本 5 Hz）；
  · 反过来，**脚本一死/网一断，1 秒内臂自己就不自由了** —— 这是个安全特性，
    不是缺陷。所以本脚本默认还带一个 `--max-seconds` 硬上限。

前置条件（缺一条就白搭）
------------------------
`FreedriveControlMessage` 走的是 **reverse interface**，驱动头文件原话是
「the reverse socket running in the external control script on the robot」——
**示教器上那个 External Control 程序（`ur5ros.urp`）必须处于 PLAYING**。
换句话说：自由驱动和「臂的 TF」要求的是**同一个状态**，不是二选一。
这也意味着程序必须全程开着 ⇒ 人一共只按两下示教器（开始 PLAY、结束 STOP）。

安全（红线 6）
--------------
开自由驱动 = **臂变成可以被手推的** = 机械臂动作类操作。
需要：当次明确授权 + 急停就绪 + 工作区隔离。上一次的授权不覆盖这一次。

用法（板上）
------------
    # 开：持续 60 秒后自动关（默认）
    python3 scripts/handeye_freedrive.py --on

    # 开：撑 3 分钟
    python3 scripts/handeye_freedrive.py --on --seconds 180

    # 关（脚本万一被 kill，用它收尾；其实 1 秒后也会自己关）
    python3 scripts/handeye_freedrive.py --off

    # 先看看控制器在不在（不动任何东西）
    python3 scripts/handeye_freedrive.py --check
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

DEFAULT_TOPIC = "/freedrive_mode_controller/enable_freedrive_mode"
# 控制器 inactive_timeout 默认 1s。5 Hz（200ms）留了 5 倍余量 ——
# 网络抖一下不至于把人的拖动中途打断。
HOLD_HZ = 5.0


class Freedrive(Node):
    def __init__(self, args):
        super().__init__("handeye_freedrive")
        self.args = args
        # 用默认（volatile）QoS：控制器是按「最近有没有收到」判超时的，
        # 不需要历史，也不该用 transient_local（那会让新订阅者收到旧值）。
        self.pub = self.create_publisher(Bool, args.topic, QoSProfile(depth=1))

    def found_controller(self) -> bool:
        """控制器在不在 —— 数**订阅者**，不发任何东西。

        ⚠️ 别改成 `get_topic_names_and_types()`：本节点自己在同一个话题上建了
        publisher，话题表于是**永远**看得见它，这个检查会变成一句自证
        （实测过：控制器没起、话题上一个人都没有，它照样报 ✅）。
        """
        return self.count_subscribers(self.args.topic) > 0

    def send(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.pub.publish(msg)


def wait_for_subscriber(node: Freedrive, timeout_s: float) -> bool:
    """等控制器把订阅建起来。没人订阅就发布 = 石沉大海。"""
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.pub.get_subscription_count() > 0:
            return True
    return False


def do_check(node: Freedrive) -> int:
    node.get_logger().info(f"在查话题：{node.args.topic}")
    # 给 discovery 一点时间
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.found_controller():
        print(f"✅ 找到了 {node.args.topic}")
        print("   ⇒ 自由驱动控制器在。可以 --on。")
        return 0
    print(f"❌ 没找到 {node.args.topic}")
    print("   ⇒ 多半是 UR 驱动那套没起（自由驱动控制器在 ur_control.launch.py")
    print("      的默认控制器列表里，驱动起了就有）。先跑 handeye_precheck.py 看看。")
    return 1


def do_off(node: Freedrive) -> int:
    """关。多发几次再等一会儿，确保控制器收到。

    这个函数在**收尾路径**上跑（Ctrl-C / SIGTERM / 到时），所以它自己绝不能
    抛异常 —— 一发不出来就静默退化成「靠控制器 1 秒超时自己关」，而不是留个
    半死不活的现场。
    """
    sent = 0
    for _ in range(10):
        if not rclpy.ok():
            break
        try:
            node.send(False)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            break
    if sent:
        print(f"已发 freedrive=False（{sent} 条）。")
    else:
        print("⚠️  一条 False 都没发出去（上下文已失效？）。")
    print("  （就算消息没送到，控制器 1 秒收不到新消息也会自己关）")
    return 0


def do_on(node: Freedrive) -> int:
    seconds = min(float(node.args.seconds), float(node.args.max_seconds))
    if seconds < node.args.seconds:
        print(f"⚠️  --seconds {node.args.seconds} 超过硬上限 --max-seconds "
              f"{node.args.max_seconds}，按上限来。")

    if not wait_for_subscriber(node, timeout_s=5.0):
        print(f"❌ 5 秒内没有订阅者 —— {node.args.topic} 上没人听。")
        print("   ⇒ 没有发出任何指令。先确认 UR 驱动起了（handeye_precheck.py）。")
        return 1

    print(f"🟢 自由驱动已开（{seconds:.0f} 秒后自动关；Ctrl-C 立刻关）")
    print("   现在可以用手拖动机械臂了。松手它就停在原地。")
    print("   到时间或按 Ctrl-C 会自动关掉，不用管这个窗口。")
    period = 1.0 / HOLD_HZ
    t0 = time.time()
    n = 0
    try:
        while rclpy.ok():
            node.send(True)
            n += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
            left = seconds - (time.time() - t0)
            if left <= 0:
                print(f"\n⏱  到时（已发 {n} 条）。正在关自由驱动…")
                break
            # 每 10 秒报一次还剩多久，免得人以为它死了
            if n % int(HOLD_HZ * 10) == 0:
                print(f"   还剩 {left:.0f} 秒…", flush=True)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C。正在关自由驱动…")
    finally:
        do_off(node)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--on", action="store_true", help="开自由驱动（持续重发以保活）")
    g.add_argument("--off", action="store_true", help="关自由驱动")
    g.add_argument("--check", action="store_true", help="只查控制器在不在，不发指令")
    ap.add_argument("--topic", default=DEFAULT_TOPIC,
                    help=f"控制器话题，默认 {DEFAULT_TOPIC}")
    ap.add_argument("--seconds", type=float, default=60.0, help="开多久（默认 60 秒）")
    ap.add_argument("--max-seconds", type=float, default=300.0,
                    help="硬上限，防止忘了关（默认 300 秒）")
    args = ap.parse_args()

    rclpy.init()

    # ⚠️ 顺序要紧：`rclpy.init()` **自己会注册** SIGINT/SIGTERM 处理器，
    # 一收到就 `rclpy.shutdown()`。装在它**前面**会被它覆盖掉，然后收尾那句
    # False 就发不出去了（实测：`RCLError: Failed to publish: publisher's
    # context is invalid`，最后一条消息停在 True）。
    # 所以装在自己之后，把它的盖掉，让两种信号都走 `except KeyboardInterrupt`
    # → `finally: do_off()` 这条路，那时上下文还是好的。
    # 真发不出去也没关系：控制器 1 秒收不到新消息会自己关（见文件头）。
    def _bye(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _bye)
    signal.signal(signal.SIGINT, _bye)
    node = Freedrive(args)
    rc = 1
    try:
        if args.check:
            rc = do_check(node)
        elif args.off:
            rc = do_off(node)
        else:
            rc = do_on(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
