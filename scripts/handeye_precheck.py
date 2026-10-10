#!/usr/bin/env python3
"""手眼标定**现场体检**：一次跑完，逐条报「现在能不能开工」。

它不采点、不碰机器人，只**看**：桥在不在、臂的 TF 在不在、靶板在不在画面里、
easy_handeye2 的服务起没起。这样"哪一条卡住了"是**被查出来的**，
而不是等人挪了半天臂才发现白挪（[[verify-before-asserting]]）。

用法（板上）：
    python3 scripts/handeye_precheck.py
加 `--watch` 就持续刷新，挪臂的时候盯着看（`--watch 2` 每 2 秒刷一次）。
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

OK = "✅"
BAD = "❌"
WARN = "⚠️ "


class Precheck(Node):
    def __init__(self, args):
        super().__init__("handeye_precheck")
        self.args = args
        self.buf = Buffer(cache_time=Duration(seconds=5), node=self)
        self.listener = TransformListener(self.buf, self, spin_thread=False)

    def look(self, target, source):
        try:
            return self.buf.lookup_transform(target, source, Time(), Duration(seconds=1.0))
        except Exception as e:
            return e

    def report(self) -> int:
        bad = 0
        print("=" * 68)
        print("  手眼标定现场体检")
        print("=" * 68)

        # 1. 臂的正解
        r = self.look(self.args.base_frame, self.args.effector_frame)
        if isinstance(r, Exception):
            bad += 1
            print(f"{BAD} 机器人正解 `{self.args.base_frame} → {self.args.effector_frame}` **查不到**")
            print(f"    底层异常：{type(r).__name__}: {r}")
            print("    ⇒ 驱动没起 / 程序没在跑 / 关节状态没上来。这一条不通，采到的样本里没有臂的位姿。")
        else:
            t = r.transform.translation
            print(f"{OK} 机器人正解 `{self.args.base_frame} → {self.args.effector_frame}`："
                  f"法兰在 base 下 = [{t.x * 1000:+.1f}, {t.y * 1000:+.1f}, {t.z * 1000:+.1f}] mm")

        # 2. 桥（相机 ← 靶板）
        r2 = self.look(self.args.camera_frame, self.args.board_frame)
        if isinstance(r2, Exception):
            bad += 1
            print(f"{BAD} 靶板 `{self.args.camera_frame} → {self.args.board_frame}` **查不到**")
            print("    ⇒ 桥没起，或者**这一拍靶板不在画面里**（桥检不到就不发 TF，这是有意的）。")
            print("    先看桥自己的心跳输出了什么 —— 它会说清是「一个码块都没认出」还是「角点不够」。")
        else:
            t = r2.transform.translation
            z = float(np.linalg.norm([t.x, t.y, t.z])) * 1000
            print(f"{OK} 靶板 `{self.args.camera_frame} → {self.args.board_frame}`："
                  f"板心距相机 {z:.0f} mm")

        # 3. 两条都要有，easy_handeye2 才会建服务
        if bad == 0:
            print(f"{OK} **两条 TF 都齐了** —— easy_handeye2 的服务端这才开始建服务")
        else:
            print(f"{BAD} 还缺 {bad} 条 —— easy_handeye2 会一直刷 "
                  f"「Waiting for TF initialization…」，服务列表里什么都没有")
        return bad


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--effector-frame", default="tool0")
    ap.add_argument("--camera-frame", default="camera_color_optical_frame")
    ap.add_argument("--board-frame", default="charuco_board")
    ap.add_argument("--watch", type=float, default=0.0, help="每 N 秒重刷一次；0=只报一次")
    args = ap.parse_args()

    rclpy.init()
    node = Precheck(args)
    try:
        if not args.watch:
            # 只报一次：先让 TF buffer 攒一会儿，否则刚起来的 listener 必然空手
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < 2.0:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.report()
        else:
            last = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.time() - last >= args.watch:
                    last = time.time()
                    node.report()
    except KeyboardInterrupt:
        pass
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
