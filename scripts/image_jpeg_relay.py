#!/usr/bin/env python3
"""把原始 `sensor_msgs/Image` 重新编码成 `sensor_msgs/CompressedImage`（JPEG）。

**为什么需要它**（第三十一轮实测的结论）
--------------------------------------
原始彩图 1280×720×3 ≈ **2.76 MB/帧**，30 Hz 就是 **83 MB/s**。而板子到开发机
实测只有 **7.4 MB/s**（已建立 TCP 连接上，见 `RESTRUCTURE_PLAN.md` 第三十一轮）：

  · 录进 bag：30 秒的原始彩图 ≈ **2.5 GB**，rsync 回开发机要 **约 6 分钟**；
  · 实时看：原始彩图根本推不动，只能到 ~2.7 fps。

JPEG 之后同分辨率约 **100–150 KB/帧** ⇒ 10 Hz 才 **1.5 MB/s**，两个问题一起消失。

**为什么不用 `image_transport` 的 republish 节点**
------------------------------------------------
`image_transport_plugins` 提供了现成的 `republish`，但**板上没装**它
（第三十一轮实测：`ros-humble-image-transport-plugins` ❌）。而 **cv2 板上现成**
（`detector_node` 本来就在用），所以这里直接用 `cv2.imencode` —— **零安装、零 sudo**。

⚠️ 输出的**话题名**刻意取成 `<源话题>/compressed`，与 `image_transport` 的约定一致。
于是**开发机**（装了 `image-transport-plugins`）那边可以直接用 RViz 的
Image 显示 + `Transport Hint: compressed` 来解，不需要额外配置。
（板子出 JPEG、开发机解 JPEG，两边各用各的现成件，互不要求对方装东西。）

QoS
---
订阅用 **BEST_EFFORT**：RealSense 的彩色话题就是 BEST_EFFORT 发的，而
**RELIABLE 的订阅者匹配不上 BEST_EFFORT 的发布者**（会静默收不到，不报错）。
发布默认 **RELIABLE**（消息已经很小了，没有用 BEST_EFFORT 的理由），
可用 `--reliable false` 改回 BEST_EFFORT。

用法
----
    # 单话题
    python3 scripts/image_jpeg_relay.py --image-topic /camera/color/image_raw

    # 多话题（逗号分隔）
    python3 scripts/image_jpeg_relay.py \\
        --image-topic /camera/color/image_raw,/detections_image

    # 降分辨率 + 限频（板子很闲时可以省，忙的时候是保险）
    python3 scripts/image_jpeg_relay.py --image-topic /detections_image \\
        --scale 0.5 --max-rate 5 --quality 80

参数
----
  --image-topic   源话题，逗号分隔（`sensor_msgs/Image`）
  --quality       JPEG 质量 1–100，默认 85
  --scale         缩放系数，默认 1.0（不缩放）
  --max-rate      每话题最大发布频率，0 = 不限（默认 0）
  --reliable      发布用 RELIABLE（默认 true）；false 则用 BEST_EFFORT
"""

import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _topic_name(topic: str) -> str:
    """话题名规范化：`foo` → `/foo`，去掉尾部斜杠。"""
    t = topic.strip()
    if not t:
        return t
    if not t.startswith("/"):
        t = "/" + t
    return t.rstrip("/")


class JpegRelay(Node):
    def __init__(self, topics, quality, scale, max_rate, reliable):
        super().__init__("image_jpeg_relay")
        self.bridge = CvBridge()
        self.quality = quality
        self.scale = scale
        self.min_dt = (1.0 / max_rate) if max_rate > 0 else 0.0
        self._last = {}             # 话题 → 上次发布时间
        self._published = {}        # 话题 → 已发帧数（收尾时报数用）

        # ⚠️ 订阅必须 BEST_EFFORT 才能匹配 RealSense 的发布者（见模块 docstring）。
        sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        pub_qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pubs = {}
        for topic in topics:
            out = topic + "/compressed"
            self._pubs[topic] = self.create_publisher(CompressedImage, out, pub_qos)
            self._published[topic] = 0
            self.create_subscription(
                Image, topic, self._make_cb(topic), sub_qos)
            self.get_logger().info(
                f"{topic}  →  {out}   (quality={quality}, scale={scale}, "
                f"max_rate={max_rate or '不限'})")

    def _make_cb(self, topic):
        # 必须用默认参数把 topic 绑进闭包 —— 直接在循环里用 lambda 会全部捕获
        # 最后一个值，N 个订阅者全挤到同一个话题上（经典坑）。
        def cb(msg):
            # ⚠️ 收尾竞态：录制器是用 SIGTERM 收掉本节点的，而 rclpy 默认的
            # 信号处理器会**立刻把 context 置为无效**，此时执行器里可能还有一个
            # 已经派发出去的回调。它接着 `publish()` 就会抛
            # `RCLError: publisher's context is invalid` —— 本来是一次正常退出，
            # 却在日志里留下一串 traceback（实测踩过），下次真出错时会被这堆
            # 噪声淹掉。所以两道都要有：进来先看 context 还在不在。
            if not rclpy.ok():
                return
            if self.min_dt > 0.0:
                import time
                now = time.monotonic()
                if now - self._last.get(topic, 0.0) < self.min_dt:
                    return
                self._last[topic] = now
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:                        # noqa: BLE001
                self.get_logger().warn(f"{topic}: 解码失败，跳过一帧（{e}）")
                return
            if self.scale != 1.0:
                frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                                   interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                self.get_logger().warn(f"{topic}: JPEG 编码失败，跳过一帧")
                return
            out = CompressedImage()
            out.header = msg.header          # 时间戳必须原样带过去，否则对不上深度/检测
            out.format = "jpeg"
            out.data = buf.tobytes()
            try:
                self._pubs[topic].publish(out)
            except Exception:                 # noqa: BLE001
                # context 在我们检查之后、publish 之前被关掉的那一瞬。
                # 正常退出，不值得一条 traceback。
                return
            self._published[topic] += 1
        return cb

    def report(self):
        # ⚠️ 用 print 而不是 get_logger()：走到这里 context 通常已经关了，
        # 而 rclpy 的日志会顺带往 /rosout 发一条 —— 于是收尾时刷出一片
        # 「Failed to publish log message to rosout」（实测踩过）。
        for topic, n in self._published.items():
            print(f"[image_jpeg_relay] 收尾：{topic} 共发 {n} 帧")


def main():
    ap = argparse.ArgumentParser(description="原始图像 → JPEG CompressedImage 中继")
    ap.add_argument("--image-topic", required=True,
                    help="源话题（sensor_msgs/Image），逗号分隔")
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--max-rate", type=float, default=0.0,
                    help="每话题最大发布频率 Hz，0 = 不限")
    ap.add_argument("--reliable", default="true")
    args = ap.parse_args()

    topics = [_topic_name(t) for t in args.image_topic.split(",") if t.strip()]
    if not topics:
        print("--image-topic 是空的。", file=sys.stderr)
        return 2
    if not 1 <= args.quality <= 100:
        print(f"--quality 要在 1–100，收到 {args.quality}", file=sys.stderr)
        return 2
    if not 0.0 < args.scale <= 1.0:
        print(f"--scale 要在 (0, 1]，收到 {args.scale}", file=sys.stderr)
        return 2

    rclpy.init()
    node = JpegRelay(topics, args.quality, args.scale, args.max_rate,
                     args.reliable.lower() in ("1", "true", "yes"))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        try:
            node.destroy_node()
        except Exception:                     # noqa: BLE001
            pass                              # context 已关时 destroy 也会炸，正常
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
