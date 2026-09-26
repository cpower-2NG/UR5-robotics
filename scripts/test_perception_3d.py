#!/usr/bin/env python3
"""
perception_3d / projector_node 自检 —— 不需要仿真器，也不需要跑 ROS 图
=====================================================================
只 `rclpy.init()` 后直接实例化节点，验证四类**纯逻辑**行为：

  D1   深度尺度按 encoding 判定（16UC1 毫米 vs 32FC1 米）
  D2   检测↔深度 按时间戳就近配对，环形缓冲有上界
  D33  PoseStamped 经 TF 变换不再抛 AttributeError
  P2-6 检测框内多区域采样取「最近簇」，抗背景平面干扰

用法:
    source /opt/ros/humble/setup.bash
    python3 scripts/test_perception_3d.py

不需要 Gazebo / RViz / 真实相机；不需要 `colcon build`（直接按路径导入源码）。
退出码 0 = 全通过，1 = 有失败项。
"""

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PERCEPTION_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "perception_3d" / "perception_3d"

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped, PoseStamped, Point
except ImportError as e:
    print(f"[SKIP] 未找到 ROS2 Python 环境（{e}）。")
    print("       请先 source /opt/ros/humble/setup.bash 再运行本脚本。")
    sys.exit(0)

sys.path.insert(0, str(PERCEPTION_SRC))
from projector_node import Perception3D, DEPTH_SCALE_BY_ENCODING  # noqa: E402


_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def patch_intrinsics(n):
    """给未接 camera_info 的节点注入一组内参，使投影可算。

    不这样做的话 PinholeCameraModel 未初始化，cx() 会去索引 None。
    """
    n.camera_model.fx = lambda: 554.0
    n.camera_model.fy = lambda: 554.0
    n.camera_model.cx = lambda: 320.0
    n.camera_model.cy = lambda: 240.0
    n.camera_info_received = True
    return n


def main():
    rclpy.init()
    node = Perception3D()

    print("\n--- D1: 深度尺度按 encoding 判定 ---")
    check("16UC1 (D435i) -> 0.001", DEPTH_SCALE_BY_ENCODING['16UC1'] == 0.001)
    check("32FC1 (Gazebo) -> 1.0", DEPTH_SCALE_BY_ENCODING['32FC1'] == 1.0)

    # 模拟 D435i 的 16UC1 深度：1500 mm 应当得到 1.5 m
    mm = np.full((480, 640), 1500, dtype=np.uint16)
    m = mm.astype(np.float32) * DEPTH_SCALE_BY_ENCODING['16UC1']
    check("1500mm -> 1.5m（旧代码会当成 1500m）",
          abs(m[0, 0] - 1.5) < 1e-3, f"got {m[0, 0]:.3f}")

    node.depth_scale = None
    check("未知编码回退 1.0 且不抛异常", node._resolve_depth_scale('weird16') == 1.0)
    node.depth_scale = None

    print("\n--- P2-6: 框内采样取最近簇（抗背景干扰）---")
    depth = np.full((480, 640), 1.0, dtype=np.float32)   # 桌面 1.0 m
    depth[200:280, 280:360] = 0.5                         # 方块顶面 0.5 m（更近）
    # 检测框覆盖方块 + 大量背景：方块只占框内约 1/4 面积
    d = node._robust_depth(depth, u=320.0, v=240.0, size_x=160.0, size_y=160.0)
    check("取到方块顶面而非桌面", d is not None and abs(d - 0.5) < 1e-6, f"got {d}")

    box = depth[160:320, 240:400]
    check("对照：同区域全框中位数会被背景带偏",
          abs(float(np.median(box[box > 0])) - 1.0) < 1e-6)

    check("全无效深度 -> None（不误返回 0）",
          node._robust_depth(np.zeros((480, 640), np.float32), 320.0, 240.0, 80.0, 80.0) is None)
    check("size_x/size_y 缺失 -> 退回单点邻域仍可取到",
          node._robust_depth(depth, 320.0, 240.0, 0.0, 0.0) == 0.5)

    print("\n--- D2: 检测↔深度 时间戳就近配对 ---")
    node.depth_buffer.clear()
    for t, val in [(100.0, 1.0), (100.033, 2.0), (100.066, 3.0)]:
        node.depth_buffer.append((t, np.full((4, 4), val, np.float32)))
    d, dt = node._nearest_depth(100.030)
    check("配对到最近的 100.033 帧",
          d is not None and abs(d[0, 0] - 2.0) < 1e-6, f"dt={dt * 1000:.0f}ms")
    d, dt = node._nearest_depth(100.064)
    check("配对到 100.066 帧", abs(d[0, 0] - 3.0) < 1e-6)

    node.depth_buffer.clear()
    check("缓冲为空 -> (None, None)", node._nearest_depth(1.0) == (None, None))

    for i in range(50):
        node.depth_buffer.append((float(i), np.zeros((4, 4), np.float32)))
    check("环形缓冲有上界（不无限吃内存）",
          len(node.depth_buffer) == 10, f"len={len(node.depth_buffer)}")

    print("\n--- D33: PoseStamped 的 TF 变换不再抛 AttributeError ---")
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'base_link', 'camera_color_optical_frame'
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 1.0, 2.0, 3.0
    t.transform.rotation.w = 1.0

    ps = PoseStamped()
    ps.header.frame_id = 'camera_color_optical_frame'
    ps.pose.position = Point(x=0.1, y=0.2, z=0.3)
    try:
        out = node._transform_pose(ps, t)
        check("_transform_pose 未抛异常", True)
        check("返回 PoseStamped", isinstance(out, PoseStamped), f"type={type(out).__name__}")
        check("平移正确 (0.1,0.2,0.3)+(1,2,3)",
              abs(out.pose.position.x - 1.1) < 1e-9 and
              abs(out.pose.position.y - 2.2) < 1e-9 and
              abs(out.pose.position.z - 3.3) < 1e-9,
              f"got ({out.pose.position.x:.3f},{out.pose.position.y:.3f},"
              f"{out.pose.position.z:.3f})")
    except AttributeError as e:
        # 这正是修复前的行为：AttributeError 逃出 except，/target_3d 永不发布
        check("_transform_pose 未抛异常", False, f"AttributeError: {e}")

    print("\n--- D8: TF 失败处理语义 ---")
    check("tf_required 默认 False（缺 TF 不崩溃，但会记 ERROR）", node.tf_required is False)
    check("连续失败计数器存在", hasattr(node, 'tf_fail_streak'))

    print("\n--- D7 护栏: 归一化坐标必须可见（P2-5）---")
    check("护栏标志初始为 False", node._warned_normalized is False)

    # 模拟上游误发归一化 bbox：center/size 全落在 (0,1]
    # 注意 Humble 的 vision_msgs/Pose2D 是 {position: Point2D, theta}，
    # 没有扁平的 x/y —— 这正是原代码注释里说的「Pose2D 有 API 兼容问题」。
    from vision_msgs.msg import (
        Detection2DArray, Detection2D, BoundingBox2D, Pose2D, Point2D,
    )

    def make_bbox(cx, cy, w, h):
        bb = BoundingBox2D()
        bb.center = Pose2D(position=Point2D(x=cx, y=cy), theta=0.0)
        bb.size_x, bb.size_y = w, h
        return bb

    msg = Detection2DArray()
    msg.header.frame_id = 'camera_color_optical_frame'
    d = Detection2D()
    d.bbox = make_bbox(0.5, 0.5, 0.05, 0.05)      # 归一化
    msg.detections = [d]

    patch_intrinsics(node)
    node.depth_buffer.clear()
    node.depth_buffer.append((0.0, np.full((480, 640), 0.5, np.float32)))
    node.detection_callback(msg)                   # should trip the guard, not raise
    check("归一化输入不抛异常", True)
    check("护栏已触发（_warned_normalized=True）", node._warned_normalized is True)

    # 像素坐标不应误报
    node2 = patch_intrinsics(Perception3D())
    node2.depth_buffer.clear()
    node2.depth_buffer.append((0.0, np.full((480, 640), 0.5, np.float32)))
    msg2 = Detection2DArray()
    msg2.header.frame_id = 'camera_color_optical_frame'
    d2 = Detection2D()
    d2.bbox = make_bbox(320.0, 240.0, 40.0, 40.0)   # 像素
    msg2.detections = [d2]
    node2.detection_callback(msg2)
    check("像素坐标不误报", node2._warned_normalized is False)
    node2.destroy_node()

    print("\n--- 针孔投影数值自洽 ---")
    patch_intrinsics(node)
    x = (400.0 - node.camera_model.cx()) * 1.0 / node.camera_model.fx()
    y = (300.0 - node.camera_model.cy()) * 1.0 / node.camera_model.fy()
    check("偏离 80px @1m -> x≈0.144m", abs(x - 0.14440) < 1e-3, f"x={x:.5f}")
    check("偏离 60px @1m -> y≈0.108m", abs(y - 0.10830) < 1e-3, f"y={y:.5f}")

    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 52)
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
