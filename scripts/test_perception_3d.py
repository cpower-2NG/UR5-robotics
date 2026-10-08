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

import math
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
PERCEPTION_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "perception_3d" / "perception_3d"

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion
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

    print("\n--- 抓取点：框中心 vs 顶面中心（正向模型）---")
    # 这一节不碰真机、不读标定文件：自己造一个「斜视俯视」相机，把一个方块的
    # 8 个角正投影出去、取包围盒，再拿这个包围盒去问算法「方块中心在哪」。
    # 于是**真值是构造出来的**，误差是可以直接算出来的毫米数。
    #
    # 为什么不信手算：这条修正的第一次实现（把两棱中点投影回像素、再与
    # target_plane_z 求交）在正向模型上是**零效果** —— 两列误差一模一样。
    # 原因是中点落在 z=(H+0)/2 上，同一根射线延伸到 z=H 落点是另一个 (x,y)。
    # 肉眼看不出来，只有正向模型能抓出来。见 _top_face_xy 的 docstring。
    h_top = 0.0285                       # 方块高（顶面高度）
    eye = np.array([0.42, 1.067, 0.825])  # 相机在 base 下（真机 topcam 量级）
    aim = np.array([0.47, 0.30, 0.0])     # 光轴指向台面
    fwd = aim - eye
    fwd /= np.linalg.norm(fwd)
    rgt = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    rgt /= np.linalg.norm(rgt)
    dwn = np.cross(fwd, rgt)
    R_co = np.column_stack([rgt, dwn, fwd])   # 光学->base（列 = 光学基）

    def _R_to_quat(Rm):
        """旋转矩阵 -> Quaternion 消息。"""
        tr = float(Rm[0, 0] + Rm[1, 1] + Rm[2, 2])
        w = math.sqrt(max(0.0, 1.0 + tr)) / 2.0
        return Quaternion(w=w,
                          x=float((Rm[2, 1] - Rm[1, 2]) / (4.0 * w)),
                          y=float((Rm[0, 2] - Rm[2, 0]) / (4.0 * w)),
                          z=float((Rm[1, 0] - Rm[0, 1]) / (4.0 * w)))

    tf_geom = TransformStamped()
    tf_geom.transform.rotation = _R_to_quat(R_co)
    tf_geom.transform.translation.x = float(eye[0])
    tf_geom.transform.translation.y = float(eye[1])
    tf_geom.transform.translation.z = float(eye[2])

    check("四元数往返能还原旋转矩阵",
          float(np.abs(node._quat_to_matrix(tf_geom.transform.rotation) - R_co).max()) < 1e-12)

    fx_g, fy_g, cx_g, cy_g = 611.747, 610.951, 326.370, 229.938

    def _bbox(bx, by, s_xy, s_z=None):
        """把 s_xy×s_xy×s_z 的棱柱放在 (bx,by,0)，返回它侧影的包围盒。"""
        s_z = h_top if s_z is None else s_z
        hx = s_xy / 2.0
        pts = np.array([[x, y, z] for x in (bx - hx, bx + hx)
                        for y in (by - hx, by + hx) for z in (0.0, s_z)])
        pc = (pts - eye) @ R_co            # R 正交：cam = R^T (base-t) <=> (base-t)@R
        u = cx_g + fx_g * pc[:, 0] / pc[:, 2]
        v = cy_g + fy_g * pc[:, 1] / pc[:, 2]
        return u.min(), u.max(), v.min(), v.max()

    node.target_plane_z = h_top
    node.table_plane_z = 0.0
    node.camera_model.fx = lambda: fx_g
    node.camera_model.fy = lambda: fy_g
    node.camera_model.cx = lambda: cx_g
    node.camera_model.cy = lambda: cy_g

    e_old, e_new, worst = [], [], 0.0
    for bx, by in [(0.485, 0.301), (0.450, 0.250), (0.520, 0.360), (0.485, 0.400)]:
        for s_xy in (0.0285, 0.0394, 0.045):
            u0, u1, v0, v1 = _bbox(bx, by, s_xy)
            uc, vc, sy = (u0 + u1) / 2.0, (v0 + v1) / 2.0, v1 - v0
            old = node._ray_plane_xy(uc, vc, h_top, R_co, eye)
            new = node._top_face_xy(uc, vc, sy, tf_geom)
            e_old.append(math.hypot(old[0] - bx, old[1] - by) * 1000.0)
            e_new.append(math.hypot(new[0] - bx, new[1] - by) * 1000.0)
            worst = max(worst, e_new[-1])
    check("框中心（旧行为）确实系统性偏 —— 均值 > 10 mm",
          float(np.mean(e_old)) > 10.0, f"均值 {np.mean(e_old):.2f} mm")
    check("顶面中心（新行为）落到亚毫米 —— 最大 < 1 mm",
          worst < 1.0, f"最大 {worst:.2f} mm（旧 {np.mean(e_old):.2f} mm）")

    # 偏差方向：这条钉住「不是随便偏，是偏在视线方向上」。+y 朝相机
    # （相机 y=1.067，方块 y≈0.30），所以旧算法的抓取点落在顶面**靠相机那一侧**。
    u0, u1, v0, v1 = _bbox(0.485, 0.301, 0.0394)
    old = node._ray_plane_xy((u0 + u1) / 2.0, (v0 + v1) / 2.0, h_top, R_co, eye)
    check("旧偏差方向 = +y（朝相机）且横向几乎为零",
          (old[1] - 0.301) > 0.010 and abs(old[0] - 0.485) < 0.003,
          f"Δ=({(old[0] - 0.485) * 1000:+.2f},{(old[1] - 0.301) * 1000:+.2f}) mm")

    # 框不紧（真实 YOLO 框不会像素级贴合）不该被放大成毫米级误差
    loose = []
    for pad in (1.0, 3.0, 5.0):
        u0, u1, v0, v1 = _bbox(0.485, 0.301, 0.0394)
        n_pad = node._top_face_xy((u0 + u1) / 2.0, (v0 + v1) / 2.0,
                                  (v1 + pad) - (v0 - pad), tf_geom)
        loose.append(math.hypot(n_pad[0] - 0.485, n_pad[1] - 0.301) * 1000.0)
    check("框每边松 5 px，误差仍 < 1 mm",
          max(loose) < 1.0, f"最松一档 {max(loose):.2f} mm")

    # 默认模式必须**逐位**不变：这条是给「改了别的路径」上的保险。
    # 深度取该像素射线与顶面交点到相机的距离（真机上 `_robust_depth` 给的
    # 就是这个量级），这样平面模式的高度校验才会放行。
    uu, vv = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    p_on_plane = node._ray_plane_xy(uu, vv, h_top, R_co, eye)
    depth_g = float(np.linalg.norm(np.array([p_on_plane[0], p_on_plane[1], h_top]) - eye))
    x_d = (uu - cx_g) * depth_g / fx_g
    y_d = (vv - cy_g) * depth_g / fy_g
    p_none = node._ray_plane_pose(x_d, y_d, depth_g, tf_geom, uu, vv,
                                  node.get_clock().now().to_msg())
    p_old = node._ray_plane_xy(uu, vv, h_top, R_co, eye)
    check("grasp_xy=None 与老路径逐位一致（默认行为不变）",
          p_none is not None and
          abs(p_none.pose.position.x - p_old[0]) < 1e-12 and
          abs(p_none.pose.position.y - p_old[1]) < 1e-12 and
          abs(p_none.pose.position.z - h_top) < 1e-12,
          f"({p_none.pose.position.x:.9f},{p_none.pose.position.y:.9f})"
          if p_none is not None else "返回 None")

    # 顶面高度填错时，新方法不该比旧的更差（真高 28.5，参数故意填偏）
    for h_wrong in (0.020, 0.035, 0.045):
        node.target_plane_z = h_wrong
        eo, en = [], []
        for bx, by in [(0.485, 0.301), (0.450, 0.250), (0.520, 0.360), (0.485, 0.400)]:
            u0, u1, v0, v1 = _bbox(bx, by, 0.0394)
            uc, vc, sy = (u0 + u1) / 2.0, (v0 + v1) / 2.0, v1 - v0
            o = node._ray_plane_xy(uc, vc, h_wrong, R_co, eye)
            n = node._top_face_xy(uc, vc, sy, tf_geom)
            eo.append(math.hypot(o[0] - bx, o[1] - by) * 1000.0)
            en.append(math.hypot(n[0] - bx, n[1] - by) * 1000.0)
        check(f"target_plane_z 填成 {h_wrong * 1000:.0f}mm 时新方法仍不差于旧方法",
              max(en) < max(eo), f"新 {max(en):.2f}mm vs 旧 {max(eo):.2f}mm")
    node.target_plane_z = h_top

    # 配置写错必须能被看见，而不是静默算错
    from projector_node import grasp_point_config_warnings as _warn
    check("配置正确时不吵人",
          _warn(0.0285, 0.0, 'top_face_center') == []
          and _warn(None, 0.0, 'bbox_center') == [])
    check("table_plane_z >= target_plane_z 会被喊出来（不静默）",
          len(_warn(0.0285, 0.0285, 'top_face_center')) == 1
          and len(_warn(0.0285, 0.05, 'top_face_center')) == 1)
    check("模式名拼错会被喊出来（否则静默退回不修正）",
          len(_warn(0.0285, 0.0, 'topface_center')) == 1
          and len(_warn(0.0285, 0.0, 'top_face_centre')) == 1)
    node.table_plane_z = 0.05          # 故意写反
    _bad = node._top_face_xy((u0 + u1) / 2.0, (v0 + v1) / 2.0, (v1 - v0), tf_geom)
    check("  → 偏量约等于一个方块高",
          _bad is not None and math.hypot(_bad[0] - 0.485, _bad[1] - 0.301) > 0.02,
          f"偏 {math.hypot(_bad[0] - 0.485, _bad[1] - 0.301) * 1000:.1f} mm")
    node.table_plane_z = 0.0

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
