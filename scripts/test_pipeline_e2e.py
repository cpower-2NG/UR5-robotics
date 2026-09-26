#!/usr/bin/env python3
"""
端到端链路验证：合成 camera_info + 深度 + 检测框喂进**正在运行的** pipeline，
检查 /target_3d 是否真的被发布、数值对不对（D33 / D1 / P2-6 的现场复验）。

与另外两个自检脚本的分工：
  scripts/test_perception_3d.py   —— 纯逻辑单测，不起 ROS 图，不碰真实话题
  scripts/test_detector_device.py —— 纯逻辑单测，不需要 torch
  scripts/test_pipeline_e2e.py    —— **本脚本**：真起话题、真走 DDS、真发布
                                     /target_3d。前两个全绿只说明「单元逻辑正确」，
                                     不等于链路端到端可用——这个脚本才回答后者。

用法（需要 pipeline 已经在跑）:
    # 终端 1
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    ros2 launch yolo_bringup pipeline.launch.py camera_source:=external
    # 终端 2
    python3 scripts/test_pipeline_e2e.py
退出码 0 = 全通过，1 = 有失败项。

⚠ WSL2 上务必让两个终端都 export ROS_LOCALHOST_ONLY=1。否则 DDS 组播发现
不稳定，会出现「同一份脚本一次全过、一次一条消息都收不到」的假失败。
⚠ ultralytics 在 conda 前缀环境里、rclpy 在 /opt/ros 下，两者要叠起来：
    export PYTHONPATH=<venv>/lib/python3.10/site-packages:$PYTHONPATH

v1 用「子线程 rclpy.spin」跑，结果旋起线程后立即抛异常且 traceback 被截断，
行为不确定（一次发布 NaN、一次什么都不发）——那是**测试脚手架**的问题，
不是被测节点的问题。v2 改为在**主线程**跑 spin_once 循环（rclpy 的正统用法），
全部动作由时间表驱动，不做跨线程调用。

  A) 方块居中、深 0.5m          -> /target_3d = (0, 0, 0.5) in base_link
  B) 方块移到 (u=120,v=85)      -> x=(120-80)*0.5/138.5, y=(85-60)*0.5/138.5
  A/B 的框内**大部分是 1.0m 的桌面**，只有中央 1/4 是方块——取到 0.5 而非 1.0，
  才算 P2-6「取最近簇」在真实链路里生效。
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, Pose2D, Point2D
from tf2_ros import StaticTransformBroadcaster

FX = FY = 138.5          # 554 / 4
CX, CY = 80.0, 60.0      # 320/4, 240/4
CAM_FRAME = 'camera_color_optical_frame'
TARGET_FRAME = 'base_link'
# 用 160x120 而不是 640x480：float32 下 1.23MB/帧会把 WSL2 的 DDS 通道压到
# ~9MB/s（实测发布 7.6Hz、订阅 3.6Hz），于是「检测↔深度」时间戳差到 431ms，
# 被 max_sync_dt 丢掉——那是**传输吞吐**问题，不是链路逻辑问题。
# 尺寸缩 4 倍、内参同比缩，投影公式 (u-cx)*z/fx 结果不变，期望值一样。
IMG_H, IMG_W = 120, 160
WALL, BLOCK = 1.0, 0.5


def make_depth(r0, r1, c0, c1):
    d = np.full((IMG_H, IMG_W), WALL, dtype=np.float32)
    d[r0:r1, c0:c1] = BLOCK
    return d


def make_detection(cx, cy, w=40.0, h=40.0):
    m = Detection2DArray()
    d = Detection2D()
    bb = BoundingBox2D()
    bb.center = Pose2D(position=Point2D(x=cx, y=cy), theta=0.0)
    bb.size_x, bb.size_y = w, h
    d.bbox = bb
    m.detections = [d]
    return m


class Driver(Node):
    def __init__(self):
        super().__init__('e2e_driver')
        self.t0 = time.time()
        self.depth = make_depth(50, 70, 70, 90)      # 方块居中 (u=80,v=60)
        self.schedule = [
            (2.5, 'det_A', 80.0, 60.0),
            (4.0, 'move_B', None, None),
            (4.6, 'det_B', 120.0, 85.0),
        ]
        self.fired = set()
        self.received = []

        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.create_subscription(PoseStamped, '/target_3d', self.on_target, 10)

        self.tfb = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = TARGET_FRAME
        t.child_frame_id = CAM_FRAME
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)

        self.create_timer(0.05, self.tick)          # 20 Hz：深度
        self.create_timer(0.5, self.send_camera_info)

    def send_camera_info(self):
        ci = CameraInfo()
        ci.header.stamp = self.get_clock().now().to_msg()
        ci.header.frame_id = CAM_FRAME
        ci.width, ci.height = IMG_W, IMG_H
        ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        ci.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0] * 5
        self.pub_info.publish(ci)

    def tick(self):
        elapsed = time.time() - self.t0
        now = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = now
        img.header.frame_id = CAM_FRAME
        img.height, img.width = IMG_H, IMG_W
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = IMG_W * 4
        img.data = self.depth.tobytes()
        self.pub_depth.publish(img)

        for t_fire, action, a, b in self.schedule:
            if action in self.fired or elapsed < t_fire:
                continue
            self.fired.add(action)
            if action == 'move_B':
                self.depth = make_depth(75, 95, 110, 130)
            else:
                msg = make_detection(a, b)
                msg.header.stamp = now      # 与本 tick 的深度帧同戳
                msg.header.frame_id = CAM_FRAME
                self.pub_det.publish(msg)
                print(f'  [t={elapsed:.1f}s] 发出检测 {action} @ ({a},{b})')

    def on_target(self, msg):
        p = msg.pose.position
        self.received.append((p.x, p.y, p.z, msg.header.frame_id))
        print(f'  [t={time.time()-self.t0:.1f}s] 收到 /target_3d = '
              f'({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) frame={msg.header.frame_id}')

    @property
    def done(self):
        return 'det_B' in self.fired and time.time() - self.t0 > 6.5


def main():
    rclpy.init()
    node = Driver()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)

    failed = []

    def check(name, cond, detail=''):
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
        if not cond:
            failed.append(name)

    def close(a, b, tol=2e-3):
        return abs(a - b) < tol

    print("\n=== 判定 ===")
    check("/target_3d 有输出", len(node.received) > 0,
          f"共收到 {len(node.received)} 条")

    if len(node.received) >= 1:
        x, y, z, frame = node.received[0]
        print("\n--- A: 方块居中，深 0.5m ---")
        check(f"frame = {TARGET_FRAME}", frame == TARGET_FRAME, f"got {frame}")
        check("z = 0.5（取到方块而非 1.0 的桌面）", close(z, 0.5), f"z={z}")
        check("x = 0", close(x, 0.0), f"x={x}")
        check("y = 0", close(y, 0.0), f"y={y}")

    if len(node.received) >= 2:
        x, y, z, _ = node.received[-1]
        want_x = (120.0 - CX) * 0.5 / FX
        want_y = (85.0 - CY) * 0.5 / FY
        print("\n--- B: 方块移到 (120,85) ---")
        check(f"x = {want_x:.5f}", close(x, want_x), f"x={x}")
        check(f"y = {want_y:.5f}", close(y, want_y), f"y={y}")
        check("z = 0.5", close(z, 0.5), f"z={z}")
    elif len(node.received) == 1:
        check("场景 B 也有输出（说明不是只灵一次）", False, "只有 1 条")

    node.destroy_node()
    rclpy.shutdown()
    print("\n" + "=" * 52)
    if failed:
        print(f"FAILED: {len(failed)} -> {failed}")
        return 1
    print("链路端到端通过：/detections + /camera/depth + TF -> /target_3d")
    return 0


if __name__ == '__main__':
    sys.exit(main())
