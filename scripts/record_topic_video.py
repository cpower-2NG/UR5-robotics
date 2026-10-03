#!/usr/bin/env python3
"""把一路图像话题**录成 mp4**（无头；SSH / WSL 都能用）。

为什么不用 `ros2 bag` + ffmpeg：本机（WSL）**没有 ffmpeg**，而
`cv2.VideoWriter` 现成可用。仿真验收要的是「看一遍它怎么动的」，
一路 mp4 比一个几 GB 的 bag 直接得多。

**只订阅，不发布、不发任何指令** —— 录仿真和录真机相机都用它。

跑法：
    python3 scripts/record_topic_video.py --topic /detections_image \\
        --out /tmp/sim.mp4 --max-seconds 600 --fps 10

⚠️ 录**带检测框**的那一路（`/detections_image`）比录原始图有用 ——
一眼能看出「它到底看见了什么」。原始图是 `/camera/color/image_raw`。

退出码：0 = 录到帧了；2 = 一帧都没收到（话题名不对 / 发布方没起）。
"""

import argparse
import os
import sys
import time

sys.dont_write_bytecode = True

try:
    import cv2
except ImportError:                                              # pragma: no cover
    print("!! 没有 cv2 —— 本脚本用 cv2.VideoWriter 写 mp4（本机是 python3-opencv）")
    sys.exit(2)


def open_writer(path, size, fps):
    """建一个 VideoWriter；编解码器**试两个**（环境里未必有 mp4v）。"""
    fourcc_try = ["mp4v", "avc1", "XVID"]
    for tag in fourcc_try:
        w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*tag), fps, size)
        if w.isOpened():
            return w, tag
        w.release()
    return None, None


def main():
    ap = argparse.ArgumentParser(description="录一路图像话题成 mp4（只订阅）")
    ap.add_argument("--topic", default="/detections_image")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=10.0,
                    help="写成多少帧/秒（**不是**采样率：来的帧都写，只是播放速度）")
    ap.add_argument("--max-seconds", type=float, default=600.0, help="最长录多久")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = 不限")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    import rclpy                                                # noqa: E402
    from sensor_msgs.msg import Image                            # noqa: E402
    try:
        from cv_bridge import CvBridge                           # noqa: E402
    except ImportError:
        print("!! 没有 cv_bridge —— 先 source /opt/ros/humble/setup.bash")
        return 2

    rclpy.init()
    node = rclpy.create_node("record_topic_video")
    bridge = CvBridge()
    state = {"n": 0, "size": None, "writer": None, "tag": None, "t0": time.monotonic()}

    def on_img(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:                                   # noqa: BLE001
            if not args.quiet:
                print(f"  跳过一帧（编码 {msg.encoding} 转 bgr8 失败：{e}）")
            return
        if state["writer"] is None:
            h, w = frame.shape[:2]
            state["writer"], state["tag"] = open_writer(args.out, (w, h), args.fps)
            if state["writer"] is None:
                print("!! 没有一个可用的编解码器（试过 mp4v / avc1 / XVID）")
                state["n"] = -1
                return
            state["size"] = (w, h)
            print(f"  第一帧 {w}×{h}，用编解码器 {state['tag']}")
        h, w = frame.shape[:2]
        if (w, h) != state["size"]:      # 分辨率变了就必须换 writer，否则 cv2 静默丢弃
            print(f"  ⚠️ 分辨率从 {state['size']} 变成 {(w, h)} —— 不写了，收工")
            return
        state["writer"].write(frame)
        state["n"] += 1
        if not args.quiet and state["n"] % 50 == 0:
            print(f"  已写 {state['n']} 帧（{time.monotonic() - state['t0']:.0f}s）")

    node.create_subscription(Image, args.topic, on_img, 10)
    print(f"录 {args.topic} → {args.out}（最多 {args.max_seconds:.0f}s / "
          f"{args.max_frames or '∞'} 帧）")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            el = time.monotonic() - state["t0"]
            if el > args.max_seconds:
                print(f"  到 {args.max_seconds:.0f}s 上限，停")
                break
            if args.max_frames and state["n"] >= args.max_frames:
                break
            if state["n"] == -1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠️ **先把视频落盘**（release 会写 moov 尾），再做任何可能抛的事。
        if state["writer"] is not None:
            state["writer"].release()
        try:
            node.destroy_node()
        except Exception:                                        # noqa: BLE001
            pass
        # ⚠️ Ctrl-C 时 rclpy 的**信号处理器已经关过 context**，这里再关一次会抛
        #    `RCLError: failed to shutdown: rcl_shutdown already called` ——
        #    实测就是这样，而且它会**吞掉汇总那行**（第一个版本踩的）。
        #    所以：先问 `rclpy.ok()`，并且把汇总放在 shutdown 之前。
        n = state["n"]
        if n > 0:
            sz = os.path.getsize(args.out) if os.path.exists(args.out) else 0
            print(f"✅ 写了 {n} 帧 → {args.out}（{sz / 1e6:.1f} MB，"
                  f"按 {args.fps:g} fps 播放约 {n / args.fps:.0f}s）", flush=True)
        else:
            print(f"!! 一帧都没录到 —— 话题 {args.topic} 没人在发？"
                  f"（`ros2 topic list --no-daemon` 看一眼）", flush=True)
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if state["n"] > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
