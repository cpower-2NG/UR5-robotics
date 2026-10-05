#!/usr/bin/env python3
"""实时推理 + 录像：人手动摆方块，它把**看见的东西**画进画面并录成 mp4。

为什么要有它
============
A2 的验收要「每类召回 + 分拣录像」。分拣录像要等机械臂能动，但**检测这一环
可以先验** —— 人手动移动方块，模型实时推理，录下来给人看。这样在花时间调
规划器之前就知道「它到底认不认得这几类颜色」。

⚠️ **它不碰机械臂，也不起完整 ROS 栈。**
只订阅一路图像话题。起完整栈（`pipeline.launch.py` / `bringup.launch.py`）
会拉起 UR 驱动，占住 29999/30001/30004 —— 实测有把示教器上的程序打成
STOPPED 的前科（见台账「URCap 一连接就 STRICT 锁死轨迹控制器」）。
所以这个脚本**只读**：不发布、不发指令、不连控制器。

跑法（板上，无头）::

    # 相机节点单独起（不要用 pipeline.launch.py）
    ros2 launch realsense2_camera rs_launch.py

    # 另一个终端
    python3 scripts/live_infer_record.py /tmp/live_7cls.mp4 --model block_best_7cls.pt

    # 人这时候手动移动方块；Ctrl-C 收工

**录的是「已画框」的那一帧**，不是原始图 —— 一眼能看出它看见了什么。

⚠️ 关于「实时」两个字
--------------------
视频不是等时的。相机 30 fps，而 CPU 上单帧推理要几十毫秒，所以**大多数帧被丢掉**，
写出来的 mp4 按 `--fps` 播放。收工时会打印「源帧率 / 实际处理帧率 / 丢帧比例」——
别把这段 mp4 当成「它有这么流畅」。GPU 上会好得多，但仍不是等时。
要在报告里引用延迟，用打印出来的那个数，别从视频里估。

类别名从**权重文件**里读（`model.names`），不是从 `block_classes.py` 读 ——
线上以权重为准。启动时会把读到的表打印出来，**先核对一眼再开始摆**：
如果这里打印的类名与 `block_classes.DEFAULT_CLASSES` 不一致，说明权重训练时用的
是另一张类别表，那么「第 3 类」在画面上的名字会是错的，而**不会有任何报错**。

退出码：0 = 录到帧了；2 = 环境没准备好（无 rclpy / 无权重 / 一帧没收到）。
"""

import argparse
import os
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    from block_classes import PALETTE
except ImportError:                                              # pragma: no cover
    PALETTE = {}

try:
    import cv2
except ImportError:                                              # pragma: no cover
    cv2 = None
try:
    import numpy as np
except ImportError:                                              # pragma: no cover
    np = None

# 与 grab_frames.py 同一个坑：驱动发的是**双层**话题，launch 里那句 `-r` 把它
# 重映射成单层。订阅的必须是**发布方实际在用的那个**，所以做成可覆盖的。
COLOR = "/camera/camera/color/image_raw"

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    USE_ROS = True
except ImportError:
    USE_ROS = False
    rclpy = None
    Node = object
    Image = None


def bgr_of(name):
    """类别名 → OpenCV 的 BGR。调色板是 RGB 0~1，这里翻转并放大到 0~255。

    为什么要跟调色板对齐：录像里框的颜色与仿真里方块的颜色一致，人在看视频时
    不需要先在脑子里翻译"这个框是哪个类"。名字不在调色板里就给一个中性的灰 ——
    **不报错**，因为线上真有可能遇到调色板外的类名（权重说了算）。
    """
    r, g, b = PALETTE.get(name, (0.5, 0.5, 0.5))
    return (int(b * 255), int(g * 255), int(r * 255))


def draw(frame, boxes, names, tally, fps_txt):
    """把框、类名、逐类累计计数、帧率画到帧上。**原地改**并返回。"""
    for (x0, y0, x1, y1), cid, conf in boxes:
        nm = names[cid] if 0 <= cid < len(names) else "?%d" % cid
        col = bgr_of(nm)
        cv2.rectangle(frame, (x0, y0), (x1, y1), col, 2)
        label = "%s %.2f" % (nm, conf)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        # 标签底色用同一个颜色，白字压在深色块上 —— 浅色类别（黄/黄绿）
        # 用白字会看不清，所以按亮度选黑或白
        lum = 0.114 * col[0] + 0.587 * col[1] + 0.299 * col[2]
        fg = (0, 0, 0) if lum > 140 else (255, 255, 255)
        cv2.rectangle(frame, (x0, max(0, y0 - th - 4)), (x0 + tw + 4, y0), col, -1)
        cv2.putText(frame, label, (x0 + 2, max(th, y0 - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv2.LINE_AA)

    # 图例 + 累计计数：审核的人要能一眼数出"这一类出现过几次"
    y = 22
    for i, nm in enumerate(names):
        col = bgr_of(nm)
        txt = "%d %s  x%d" % (i, nm, tally.get(nm, 0))
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (8, y - th - 4), (16 + tw, y + 4), col, -1)
        lum = 0.114 * col[0] + 0.587 * col[1] + 0.299 * col[2]
        cv2.putText(frame, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0) if lum > 140 else (255, 255, 255), 2, cv2.LINE_AA)
        y += th + 12
    cv2.putText(frame, fps_txt, (8, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return frame


class Reader(Node):                                              # noqa: F821
    """只订阅。**没有**任何 create_publisher —— 故意的，见文件头。"""

    def __init__(self, topic):
        super().__init__("live_infer_record_readonly")
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        self.latest = None
        self.n_recv = 0
        self.create_subscription(Image, topic, self._on_img, 1)

    def _on_img(self, m):
        # 只留最新一帧：推理比相机慢，排队只会让画面越来越旧 ——
        # 录出来的视频里手已经移开了、框还停在老位置，看着像模型迟钝。
        self.n_recv += 1
        try:
            self.latest = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
        except Exception as e:                                   # pragma: no cover
            self.get_logger().warn("转帧失败：%s" % e)


def resolve_model(path):
    """按 detector_node 的查找顺序找一个权重文件，找不到返回 None。

    与 detector_node 同一套顺序，理由也一样：CWD 与包内都试，**不联网下载** ——
    下载一个 COCO 权重来冒充方块模型，症状是"检测不到方块"而不是报错。
    """
    cands = [path]
    root = os.path.dirname(HERE)
    cands.append(os.path.join(root, path))
    cands.append(os.path.join(root, "ros2_ws", "src", "yolo_detector", "models",
                              os.path.basename(path)))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


def main(argv=None):
    ap = argparse.ArgumentParser("实时推理 + 录像（板上，只读，不碰机械臂）")
    ap.add_argument("out", help="输出 mp4 路径")
    ap.add_argument("--model", default="block_best_7cls.pt", help="权重文件")
    ap.add_argument("--color-topic", default=COLOR,
                    help=f"彩色图话题（默认 {COLOR}；launch 会重映射成单层）")
    ap.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    ap.add_argument("--imgsz", type=int, default=640, help="推理尺寸")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="写进 mp4 的帧率（不是推理帧率，见文件头）")
    ap.add_argument("--max-seconds", type=float, default=600.0,
                    help="最长录制秒数（默认 600 = 10 分钟）")
    ap.add_argument("--device", default=None, help="cuda:0 / cpu（默认自动）")
    a = ap.parse_args(argv)

    if not USE_ROS or cv2 is None or np is None:
        print("❌ 需要 rclpy + cv_bridge + cv2 —— 这个脚本在**板子**上跑（退 2）")
        return 2

    mp = resolve_model(a.model)
    if mp is None:
        print("❌ 找不到权重 %r。查过：CWD、仓库根、包内 models/（退 2）" % a.model)
        print("   ⚠️ 这里**故意不联网下载** —— 拿 COCO 权重冒充的症状是"
              "「检测不到方块」，而不是报错。")
        return 2

    try:
        from ultralytics import YOLO
    except ImportError:
        print("❌ 板上没有 ultralytics（退 2）")
        return 2

    device = a.device
    if device is None:
        try:
            import torch
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        except ImportError:
            device = "cpu"

    print("=" * 68)
    print("  实时推理 + 录像")
    print("=" * 68)
    print("权重    : %s" % mp)
    print("设备    : %s" % device)
    print("话题    : %s" % a.color_topic)
    model = YOLO(mp)
    names = list(model.names.values()) if isinstance(model.names, dict) \
        else list(model.names)
    # ⚠️ 这一行是**必须核对**的：类名来自权重，与 block_classes 不一致时
    #    画面上的名字会整体错位，而没有任何报错。
    print("权重的类别表（%d 类）：%s" % (len(names), " / ".join(
        "%d:%s" % (i, n) for i, n in enumerate(names))))
    print("⚠️ 核对一眼：它必须与 scripts/block_classes.py 的 DEFAULT_CLASSES 逐位相同。")
    print("   不等就先别摆方块 —— 画面上的名字会是错的。")
    print()

    rclpy.init()
    node = Reader(a.color_topic)
    writer = None
    tally = {}
    n_written = n_frames = 0
    t0 = time.time()
    t_last = t0
    fps_txt = "--"
    print("开始收帧（Ctrl-C 收工）...")
    try:
        while rclpy.ok() and time.time() - t0 < a.max_seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest is None:
                continue
            frame = node.latest
            node.latest = None            # 取走即清，下一轮拿新的
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"),
                                         a.fps, (w, h))
                print("首帧 %dx%d，开始写 %s" % (w, h, a.out))
            res = model.predict(frame, conf=a.conf, imgsz=a.imgsz,
                                device=device, verbose=False)[0]
            boxes = []
            for b in res.boxes:
                x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
                cid = int(b.cls[0])
                conf = float(b.conf[0])
                boxes.append(((x0, y0, x1, y1), cid, conf))
                nm = names[cid] if 0 <= cid < len(names) else "?%d" % cid
                tally[nm] = tally.get(nm, 0) + 1
            now = time.time()
            if now - t_last >= 1.0:       # 每秒刷一次帧率
                inst = n_frames / max(now - t_last, 1e-9)
                fps_txt = "推理 %.1f fps  (已处理 %d 帧)" % (inst, n_frames)
                t_last, n_frames = now, 0
            n_frames += 1
            writer.write(draw(frame, boxes, names, tally, fps_txt))
            n_written += 1
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，收工。")
    finally:
        if writer is not None:
            writer.release()
        node.destroy_node()
        rclpy.shutdown()

    dt = time.time() - t0
    print()
    print("=" * 68)
    if n_written == 0:
        print("❌ 一帧都没收到 —— 话题名不对，或相机节点没起。")
        print("   收了 %d 条消息；话题是 %s" % (node.n_recv, a.color_topic))
        print("   查实际话题名：ros2 topic list | grep image")
        return 2
    print("写了 %d 帧 → %s" % (n_written, a.out))
    print("墙上时间 %.1f s，实际处理 %.1f fps；mp4 按 %.1f fps 播放"
          % (dt, n_written / max(dt, 1e-9), a.fps))
    print("⚠️ 这不是等时录像：相机比推理快，被丢掉的帧不会出现在视频里。")
    print("   要在报告里写延迟，用上面这个「实际处理 fps」，别从视频里估。")
    print("逐类累计（整段录制）：")
    for nm in names:
        print("   %-13s %d" % (nm, tally.get(nm, 0)))
    print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
