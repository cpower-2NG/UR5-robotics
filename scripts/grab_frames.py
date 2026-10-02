#!/usr/bin/env python3
"""只读连拍：订阅**现成的**相机话题，存 N 帧 + 一份完整的内参 JSON。

不开设备、不起第二个节点、不发任何消息 —— 只订阅。**无头**，SSH 到板子上直接跑。

为什么不直接用 ``collect_training_data.py``
==========================================
那份是**交互式**的（``cv2.namedWindow`` + ``imshow`` + ``waitKey``，空格存图），
**离了显示器就跑不起来** —— SSH 到 Orin 上必然失败。而且我们这边的键盘在
WSL，按不到板子那个窗口上。所以采集要有人代跑时，需要这份无头的。

两者不是替代关系：``collect_training_data.py`` 供人坐在板子前手动按空格，
这份供脚本化地连拍一批。

为什么单独存一份内参 JSON
==========================
标注（``auto_label_grid.py --backend tf``）要 fx/fy/cx/cy。板上那份
``/tmp/grab_one_frame.py`` 只写 ROS 形状的 ``K``/``D``，本地每次都得自己
拆那个 3×3 —— 拆错一个下标（``cx`` 拿成 ``K[1]``）**不会报错**，只会让
**每一张图的每一个标签都偏同一个量**。所以这里一次写全，并且
``scripts/test_grab_frames.py`` 把下标映射钉死。

⚠️ 相机在整个采集期间**不能动**：标注靠外参把靶系坐标投到像素，而俯视外参
实测**一天漂 0.945°**。跨天采集必须重跑 PnP 拿新外参。

用法::

    # 板上
    python3 scripts/grab_frames.py /tmp/ds_r01 25 0.08
    # 本地拉回来
    scp -q jetson-wifi:'/tmp/ds_r01/*' .scratch/rounds/r01/images/
"""
import argparse
import json
import os
import sys
import time

# ROS 只在板上才有。开发机（WSL）上没有 rclpy，但 ``camera_info_to_dict``
# 是纯函数、自检要能在这里跑，所以**不能**在模块顶层硬 import。
# 沿用 collect_training_data.py 的 USE_ROS 惯例。
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    USE_ROS = True
except ImportError:
    USE_ROS = False
    rclpy = None
    Node = object
    CameraInfo = Image = None

COLOR = "/camera/camera/color/image_raw"
INFO = "/camera/camera/color/camera_info"


def camera_info_to_dict(ci):
    """``sensor_msgs/CameraInfo`` → 本仓库那套内参 JSON。

    平铺的 ``fx/fy/cx/cy`` 取自 ROS 的 K（行主序 3×3）::

        K = [fx  0 cx]
            [ 0 fy cy]
            [ 0  0  1]      →  K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    所以 fx=K[0]、cx=K[2]、fy=K[4]、cy=K[5]。**这四个下标是本文件唯一
    会静默出错的地方**（写错了不报错，只是所有标签整体偏），
    ``scripts/test_grab_frames.py`` 专门钉它。
    """
    K = list(ci.k)
    if len(K) != 9:
        raise ValueError("K 应当是 9 个元素，实际 %d 个" % len(K))
    return {
        "width": ci.width,
        "height": ci.height,
        "encoding": "rgb8",
        "frame_id": ci.header.frame_id,
        "stamp": {"sec": ci.header.stamp.sec, "nanosec": ci.header.stamp.nanosec},
        "K": K,
        "D": list(ci.d),
        "info_frame_id": ci.header.frame_id,
        "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
    }


class Grabber(Node):                                        # noqa: F821
    def __init__(self):
        super().__init__("grab_frames_readonly")
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.images = []
        self.info = None
        self.create_subscription(Image, COLOR, self._on_img, 10)
        self.create_subscription(CameraInfo, INFO, self._on_info, 10)

    def _on_img(self, m):
        self.images.append(m)

    def _on_info(self, m):
        self.info = m


def main(argv=None):
    ap = argparse.ArgumentParser("只读连拍（无头；SSH 到板子上用）")
    ap.add_argument("out", help="输出目录")
    ap.add_argument("n", type=int, nargs='?', default=25, help="帧数（默认 25）")
    ap.add_argument("gap", type=float, nargs='?', default=0.08,
                    help="两帧之间的最小间隔秒（默认 0.08）")
    ap.add_argument("--ext", default="png", choices=("png", "jpg"),
                    help="PNG 无损但 1.2 MB/帧；JPG 约 8 倍小。默认 png")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="收图总超时秒（默认 120）")
    a = ap.parse_args(argv)

    if not USE_ROS:
        print("❌ 这台机器没有 rclpy —— 连拍要在**板子**上跑（本文件是无头版，"
              "供 SSH 用）")
        return 2

    import cv2

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Grabber()
    try:
        t0 = time.monotonic()
        while node.info is None and time.monotonic() - t0 < 20:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.info is None:
            print("❌ 20 秒没等到 camera_info —— 话题名或 ROS_DOMAIN_ID 不对？"
                  "（本板实测是双层的 %s）" % INFO)
            return 2

        seen = len(node.images)
        saved = 0
        last = 0.0
        while saved < a.n and time.monotonic() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if len(node.images) > seen and time.monotonic() - last >= a.gap:
                frame = node.bridge.imgmsg_to_cv2(node.images[-1],
                                                  desired_encoding="bgr8")
                cv2.imwrite(os.path.join(a.out, "img_%04d.%s" % (saved, a.ext)),
                            frame)
                saved += 1
                last = time.monotonic()
                seen = len(node.images)

        info = camera_info_to_dict(node.info)
        with open(os.path.join(a.out, "info.json"), "w") as f:
            json.dump(info, f, indent=2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("存了 %d/%d 帧 → %s（fx=%.2f fy=%.2f cx=%.2f cy=%.2f，%d×%d）"
          % (saved, a.n, a.out, info["fx"], info["fy"], info["cx"], info["cy"],
             info["width"], info["height"]))
    if saved < a.n:
        print("⚠️ 没凑够 —— 相机频率不够，或者中途话题断了")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
