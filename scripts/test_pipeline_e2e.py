#!/usr/bin/env python3
"""
端到端链路验证：合成 camera_info + 深度 + 检测框喂进**正在运行的** pipeline，
检查 /target_3d 与 /targets_3d 是否真的被发布、数值对不对
（D33 / D1 / P2-6 / P3-6 的现场复验）。

两个输出话题都要验，因为它们服务**不同消费者**：
  /target_3d   —— 单条 PoseStamped，兼容旧接口（最优目标）
  /targets_3d  —— Detection3DArray，**全部候选**，抓取规划器消费这个（P3-6）
若只验 /target_3d，则「候选集是否完整、类别与分数有没有透传」无人把关，
而规划器正是靠 `results[0].hypothesis.score` 做 min_score 过滤、
靠 `class_id` 区分物体的——**这两样一旦在感知层丢了，规划器会静默失效**。

与另外两个自检脚本的分工：
  scripts/test_perception_3d.py   —— 纯逻辑单测，不起 ROS 图，不碰真实话题
  scripts/test_detector_device.py —— 纯逻辑单测，不需要 torch
  scripts/test_pipeline_e2e.py    —— **本脚本**：真起话题、真走 DDS、真发布
                                     /target_3d 与 /targets_3d。前两个全绿只说明
                                     「单元逻辑正确」，不等于链路端到端可用——
                                     这个脚本才回答后者。

用法（需要 pipeline 已经在跑）:
    # 终端 1
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    ros2 launch yolo_bringup pipeline.launch.py camera_source:=external
    # 终端 2
    python3 scripts/test_pipeline_e2e.py
退出码 0 = 全通过，1 = 有失败项。

⚠ **不要设 `ROS_LOCALHOST_ONLY=1`**（本文档早期版本曾如此建议，**已证伪**）。
本机实测：`source /opt/ros/humble/setup.bash` 会把 `ROS_LOCALHOST_ONLY` 置为 **0**，
用这个值两节点可正常发现；一旦**改成 1，发现彻底失效**——`ros2 node list` 返回空，
本脚本会「一条消息都收不到」（曾据此误判为链路失败，实为环境变量所致）。
若确实遇到发现不稳，先查**残留的旧节点**（例如上一次仿真没退干净的
`static_transform_publisher`），那才是真正的干扰源。
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
from vision_msgs.msg import (
    Detection2DArray, Detection2D, BoundingBox2D, Pose2D, Point2D,
    Detection3DArray, ObjectHypothesisWithPose,
)
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

# 每个场景的类别/分数/id。**两个场景故意用不同的分数**，这样「最优目标」
# 的选择才有确定性（投影节点取 score 最大者为 /target_3d），
# 也让 /targets_3d 的透传断言有可区分的值可比对。
SCENARIO = {
    'det_A': dict(class_id='block', score=0.90, det_id='sceneA'),
    'det_B': dict(class_id='block', score=0.75, det_id='sceneB'),
}


def make_depth(r0, r1, c0, c1):
    d = np.full((IMG_H, IMG_W), WALL, dtype=np.float32)
    d[r0:r1, c0:c1] = BLOCK
    return d


def make_detection(cx, cy, w=40.0, h=40.0, class_id='block', score=0.9, det_id=''):
    """构造一条带**类别与分数**的检测。

    类别/分数不是为了像真实 YOLO——是为了验证**感知层有没有把它们透传下去**。
    `/targets_3d` 的消费者（抓取规划器）按 `hypothesis.score` 做 min_score 过滤、
    按 `class_id` 区分物体；若投影节点在这个环节丢字段，规划器会**静默**
    拿不到目标（分数恒 0 则全部被阈值刷掉），而没有断言就查不出来。
    """
    m = Detection2DArray()
    d = Detection2D()
    bb = BoundingBox2D()
    bb.center = Pose2D(position=Point2D(x=cx, y=cy), theta=0.0)
    bb.size_x, bb.size_y = w, h
    d.bbox = bb
    d.id = det_id
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = score
    d.results = [hyp]
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
        self.targets = []            # /targets_3d 的每一批候选（P3-6）

        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.create_subscription(PoseStamped, '/target_3d', self.on_target, 10)
        self.create_subscription(
            Detection3DArray, '/targets_3d', self.on_targets, 10
        )

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
                msg = make_detection(a, b, **SCENARIO[action])
                msg.header.stamp = now      # 与本 tick 的深度帧同戳
                msg.header.frame_id = CAM_FRAME
                self.pub_det.publish(msg)
                print(f'  [t={elapsed:.1f}s] 发出检测 {action} @ ({a},{b})')

    def on_target(self, msg):
        p = msg.pose.position
        self.received.append((p.x, p.y, p.z, msg.header.frame_id))
        print(f'  [t={time.time()-self.t0:.1f}s] 收到 /target_3d = '
              f'({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) frame={msg.header.frame_id}')

    def on_targets(self, msg):
        """收 /targets_3d（全部候选）。只统计**非空批次**——
        投影节点对「本批一个都没投出来」也会发一个空数组，那是正常行为。"""
        if not msg.detections:
            return
        self.targets.append(msg)
        t = time.time() - self.t0
        first = msg.detections[0]
        if not first.results:
            print(f'  [t={t:.1f}s] 收到 /targets_3d = {len(msg.detections)} 个候选；'
                  f'首个 **无 results**（规划器将无法判类别/分数）')
            return
        hyp = first.results[0].hypothesis
        p = first.results[0].pose.pose.position
        print(f'  [t={t:.1f}s] 收到 /targets_3d = {len(msg.detections)} 个候选；'
              f'首个 id={first.id!r} 类别={hyp.class_id!r} score={hyp.score:.2f} '
              f'base=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) frame={msg.header.frame_id}')

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

    # ---- /targets_3d：全部候选接口（P3-6）----
    # 这是抓取规划器**唯一**消费的话题，必须独立把关：
    # 候选集完整、类别与分数透传、坐标与 /target_3d 一致。
    print("\n--- /targets_3d：全部候选（规划器的输入）---")
    check("/targets_3d 有输出", len(node.targets) > 0,
          f"共收到 {len(node.targets)} 批（空批次未计入）")

    if node.targets:
        batch = node.targets[0]
        first = batch.detections[0]
        check(f"header.frame_id = {TARGET_FRAME}", batch.header.frame_id == TARGET_FRAME,
              f"got {batch.header.frame_id!r}")
        check("每条候选都带 results", all(d.results for d in batch.detections),
              f"{sum(1 for d in batch.detections if not d.results)}/"
              f"{len(batch.detections)} 条缺 results")
        check("候选都有非空 id（规划器靠它区分物体）",
              all(d.id for d in batch.detections),
              f"{sum(1 for d in batch.detections if not d.id)} 条 id 为空")
        # 类别与分数必须原样透传：规划器用 score 做 min_score 过滤、用 class_id 分类
        hyp = first.results[0].hypothesis
        check("class_id 透传", hyp.class_id == SCENARIO['det_A']['class_id'],
              f"got {hyp.class_id!r}，期望 {SCENARIO['det_A']['class_id']!r}")
        check("score 透传（不被归零）",
              close(hyp.score, SCENARIO['det_A']['score'], tol=1e-6),
              f"got {hyp.score}，期望 {SCENARIO['det_A']['score']}")
        # bbox.center 与 results[0].pose 必须指向同一位置，否则消费者取哪个都对不上
        c, r = first.bbox.center.position, first.results[0].pose.pose.position
        check("bbox.center 与 results[0].pose 一致",
              close(c.x, r.x) and close(c.y, r.y) and close(c.z, r.z),
              f"bbox=({c.x:.4f},{c.y:.4f},{c.z:.4f}) "
              f"pose=({r.x:.4f},{r.y:.4f},{r.z:.4f})")
        # 与 /target_3d 交叉比对：同一个最优目标，数值应逐位吻合
        if node.received:
            tx, ty, tz, _ = node.received[0]
            check("与 /target_3d 的坐标一致（同一次投影）",
                  close(r.x, tx) and close(r.y, ty) and close(r.z, tz),
                  f"/targets_3d=({r.x:.4f},{r.y:.4f},{r.z:.4f}) "
                  f"/target_3d=({tx:.4f},{ty:.4f},{tz:.4f})")

    if len(node.targets) >= 2:
        hyp2 = node.targets[-1].detections[0].results[0].hypothesis
        check("场景 B 的 score 也透传",
              close(hyp2.score, SCENARIO['det_B']['score'], tol=1e-6),
              f"got {hyp2.score}，期望 {SCENARIO['det_B']['score']}")
    elif node.targets:
        check("场景 B 也进了 /targets_3d", False, "只有 1 批")


    node.destroy_node()
    rclpy.shutdown()
    print("\n" + "=" * 52)
    if failed:
        print(f"FAILED: {len(failed)} -> {failed}")
        return 1
    print("链路端到端通过：/detections + /camera/depth + TF -> /target_3d + /targets_3d")
    return 0


if __name__ == '__main__':
    sys.exit(main())
