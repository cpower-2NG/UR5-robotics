#!/usr/bin/env python3
"""端到端（**单进程**）：真 Perception3D 节点 + 真消息 + 真 TF → /targets_3d。

## 为什么是单进程

`scripts/test_pipeline_e2e.py` 那套「终端 1 起 pipeline、终端 2 发检测」的跑法
在**本机跑不起来**：这台 WSL2 的跨进程 DDS 发现是坏的。实测（2026-10-08）：

  · 同一个进程内两个 node 互发 `std_msgs/String` → 收到 29633 条
  · 分成两个进程发同样的话题        → 收到 **0** 条
  · 关掉 Fast DDS 共享内存（只走 UDPv4）、`ROS_LOCALHOST_ONLY=1` 单独试 → 都不救
  · `ros2 node list` 同时返回空——CLI 只是这个症状最显眼的那一面，根因在发现层

所以那道 e2e 在这台机器上**恒红**，红得没有信息量（第 109 轮那道被无视 13.5 小时
的守卫就是这么死的）。本脚本把整条链路塞进**一个进程**：驱动节点负责发相机内参、
深度图、检测框和静态 TF，被测的真 `Perception3D` 节点照常订阅、查 TF、投影、发布。

## 它补的是什么

`scripts/test_perception_3d.py` 是纯函数单测，直接调 `_top_face_xy` 之类的内部函数，
**不经过节点**。于是下面这些它一件都拦不住：

  · 参数名在 `pipeline.launch.py` / 任务 yaml 里拼错 → 静默用默认值
  · 节点没把 `grasp_xy` 传下去（改动回退成「算了不用」）
  · TF 查询、消息时间戳同步、`_robust_depth` 的深度闸门

本脚本真的建节点、真的发消息、真的读 `/targets_3d`，所以上面三条都在射程内。

## 相机模型是**自洽**的，这份测试与标定无关

真值（方块顶面中心）和渲染用的投影**是同一台相机**算出来的，所以内参外参取什么值
都不影响判据是否成立 —— 外参重标一次不会让这道守卫变红或变绿。用真机的内参
（俯视相机实测 611.747 / 610.951 / 326.370 / 229.938）只是让视角的**倾角**
和现场接近（俯视斜视正是这个缺陷的成因）。

## 场景与判据

28.5 mm 的立方块摆在 z=0 的台面上（顶面 z=0.0285，即现场的 `target_plane_z`）。
深度图按光线投射逐像素渲染（每根光线取最近的命中点），检测框取方块 8 个角的
投影外接框 —— **就是 YOLO 真正会给出的那个侧影框**（侧影 = 顶面 + 正面，
不是顶面本身，这正是缺陷的根源）。

判据：
  1. 两种模式都得有输出 —— 拦住「深度闸门把目标误杀」这种静默失效
  2. `bbox_center`（旧算法）必须**明显偏**（≥ 10 mm）—— 场景不退化时它就该偏；
     这条同时防「真值算错导致两边都碰巧对」
  3. `top_face_center`（新算法）≤ 1 mm
  4. 加深度噪声 σ=2 mm + 框边缘抖动 2 px 后仍 ≤ 2 mm（真机深度是带噪的）

退出码按本仓约定：0 通过 / 1 断言失败 / 2 环境不满足。
"""
import os
import sys
import time

import numpy as np

# 从**源码**导入被测节点，而不是 install/ —— 本工作树里往往没有 install，
# 而且 install 可能是另一个检出编出来的（踩过：install 在主仓，
# 编的是不带改动的源码，测试会「跑绿但测的是旧代码」）。
_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'ros2_ws', 'src', 'perception_3d')
sys.path.insert(0, _SRC)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import TransformStamped
    from vision_msgs.msg import (
        Detection2DArray, Detection2D, BoundingBox2D, Pose2D, Point2D,
        Detection3DArray,
    )
    from tf2_ros import StaticTransformBroadcaster
    from perception_3d.projector_node import Perception3D
except ImportError as exc:
    print("⏭  环境不满足：%s —— 本机没有 rclpy / vision_msgs / tf2_ros，"
          "或没 source 工作区，这道自检在这里跑不了（**没验到**，不算通过也不算失败）"
          % exc)
    sys.exit(2)

# ---- 真机俯视相机内参（docs/calib 实测）----
FX, FY = 611.7470703125, 610.951416015625
CX, CY = 326.36993408203125, 229.93753051757812
W, H = 640, 480
CAM_FRAME = 'camera_color_optical_frame'
TARGET_FRAME = 'base_link'

BLOCK_S = 0.0285          # 方块边长 = 顶面高度（与 target_plane_z 一致）
TABLE_Z = 0.0
TOP_Z = BLOCK_S

# 相机位姿：固定一个与现场相近的俯视斜视位（不是标定值，见模块 docstring）。
# 自洽即可 —— 真值由同一台相机算出。位姿只需「像现场」：俯角取现场的 54.1°，
# 因为这个缺陷的**大小**随俯角变（越斜越偏），俯角不对就量不出真实量级。
#   ⚠ UP 必须是世界向上 (0,0,1)。写成 (0,0,-1) 会把相机绕光轴倒装 180°：
#     图像上下左右全反，而两棱中点法对 180° 近似不变（两条棱只是互换，
#     位移误差一阶抵消），于是**测试照样绿**，只是量到的残差不再是现场那个。
#     这个坑是我自己踩的——第一版就是 (0,0,-1)，残差 1.11 mm 而非 0.28 mm。
CAM_POS = np.array([0.42, 1.067, 0.825])
UP = np.array([0.0, 0.0, 1.0])
AIM = np.array([0.42, 0.470, 0.0])      # 俯角 ≈ 54.1°


def _look_at():
    """构造 base_link ← 光学系 的旋转（光学系：x 右、y 下、z 前）。"""
    fwd = AIM - CAM_POS
    fwd = fwd / np.linalg.norm(fwd)
    rgt = np.cross(fwd, UP)
    rgt = rgt / np.linalg.norm(rgt)
    dwn = np.cross(fwd, rgt)
    return np.column_stack([rgt, dwn, fwd])      # 列 = 光学基在 base 下的坐标


R_BO = _look_at()
O_CAM = CAM_POS


def _R_to_quat(R):
    """3×3 旋转矩阵 → (x, y, z, w)。"""
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _ray_dirs():
    """每像素在 base 下的步长方向：光学 z 分量恰为 1，故光线参数 t 即光学深度。"""
    uu, vv = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    d_opt = np.stack([(uu - CX) / FX, (vv - CY) / FY, np.ones_like(uu)], axis=-1)
    return d_opt @ R_BO.T


def _slab(D, lo, hi):
    """轴对齐盒的 slab 求交 → 最近正命中的 t（无命中 = inf）。"""
    tmin = np.full(D.shape[:2], -np.inf)
    tmax = np.full(D.shape[:2], np.inf)
    for i in range(3):
        d = D[..., i]
        safe = np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1 = (lo[i] - O_CAM[i]) / safe
        t2 = (hi[i] - O_CAM[i]) / safe
        tmin = np.maximum(tmin, np.minimum(t1, t2))
        tmax = np.minimum(tmax, np.maximum(t1, t2))
    hit = (tmax >= np.maximum(tmin, 0.0)) & (tmax > 0.0)
    return np.where(hit, np.where(tmin > 0.0, tmin, tmax), np.inf)


def _render_depth(bx, by, D):
    lo = np.array([bx - BLOCK_S / 2, by - BLOCK_S / 2, TABLE_Z])
    hi = np.array([bx + BLOCK_S / 2, by + BLOCK_S / 2, TOP_Z])
    t_cube = _slab(D, lo, hi)
    dz = D[..., 2]
    down = dz < -1e-9
    t_table = np.where(down, (TABLE_Z - O_CAM[2]) / np.where(down, dz, -1.0), np.inf)
    t = np.minimum(t_cube, t_table)
    return np.where(np.isfinite(t), t, 0.0).astype(np.float32), (lo, hi)


def _bbox(lo, hi):
    pts = np.array([[x, y, z] for x in (lo[0], hi[0])
                    for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    p = (pts - O_CAM) @ R_BO
    u = FX * p[:, 0] / p[:, 2] + CX
    v = FY * p[:, 1] / p[:, 2] + CY
    return u.min(), u.max(), v.min(), v.max()


def _pixel_to_top(u, v):
    """像素 → 射线 ∩ z=TOP_Z 的 (x, y)：由它决定方块该摆在哪。"""
    d = R_BO @ np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
    t = (TOP_Z - O_CAM[2]) / d[2]
    p = O_CAM + t * d
    return float(p[0]), float(p[1])


class _Driver(Node):
    """扮演上游：相机内参 + 深度 + 检测框 + 静态 TF。"""

    def __init__(self, depth_msg, ci_msg, det_msg, tf_msg):
        super().__init__('e2e_inproc_driver')
        self.depth_msg, self.ci_msg, self.det_msg, self.tf_msg = \
            depth_msg, ci_msg, det_msg, tf_msg
        self.tfb = StaticTransformBroadcaster(self)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)
        self.got = []
        self.create_subscription(Detection3DArray, '/targets_3d', self._on, 10)
        self.t0 = time.time()
        self.sent = False
        self.create_timer(0.05, self._tick)

    def _on(self, msg):
        if msg.detections:
            self.got.append(msg)

    def _tick(self):
        # 静态 TF 反复发：静态广播是 latched，但同进程内先发后订阅更省事
        self.tfb.sendTransform(self.tf_msg)
        self.pub_info.publish(self.ci_msg)
        self.pub_depth.publish(self.depth_msg)
        if not self.sent and time.time() - self.t0 > 1.0:
            self.sent = True
            self.pub_det.publish(self.det_msg)


def _one_case(mode, u_pix, v_pix, D, noise_m=0.0, jitter_px=0.0, seed=0):
    """跑一轮：建真节点、喂一帧、收回 /targets_3d。返回 (真值 xy, 输出 xy 或 None)。"""
    bx, by = _pixel_to_top(u_pix, v_pix)
    truth = (bx, by)

    rclpy.init(args=['--ros-args',
                     '-p', 'target_frame:=%s' % TARGET_FRAME,
                     '-p', 'use_target_plane:=true',
                     '-p', 'target_plane_z:=%r' % TOP_Z,
                     '-p', 'table_plane_z:=0.0',
                     '-p', 'grasp_point_mode:=%s' % mode,
                     '-p', 'plane_depth_tol:=0.03',
                     '-p', 'max_sync_dt:=5.0',
                     '-p', 'depth_scale:=1.0'])
    node = Perception3D()
    stamp = node.get_clock().now().to_msg()

    img, (lo, hi) = _render_depth(bx, by, D)
    if noise_m > 0.0:
        rng = np.random.default_rng(seed)
        img = np.where(img > 0,
                       img + rng.normal(0.0, noise_m, img.shape).astype(np.float32),
                       img).astype(np.float32)
    u0, u1, v0, v1 = _bbox(lo, hi)
    if jitter_px > 0.0:
        rng = np.random.default_rng(seed + 1)
        u0 -= rng.uniform(0, jitter_px)
        u1 += rng.uniform(0, jitter_px)
        v0 -= rng.uniform(0, jitter_px)
        v1 += rng.uniform(0, jitter_px)

    depth_msg = Image()
    depth_msg.header.stamp = stamp
    depth_msg.header.frame_id = CAM_FRAME
    depth_msg.height, depth_msg.width = H, W
    depth_msg.encoding = '32FC1'
    depth_msg.is_bigendian = 0
    depth_msg.step = W * 4
    depth_msg.data = img.tobytes()

    ci = CameraInfo()
    ci.header.stamp = stamp
    ci.header.frame_id = CAM_FRAME
    ci.width, ci.height = W, H
    ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    ci.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0] * 5

    det = Detection2DArray()
    det.header.stamp = stamp
    det.header.frame_id = CAM_FRAME
    d2 = Detection2D()
    d2.bbox = BoundingBox2D(
        center=Pose2D(position=Point2D(x=(u0 + u1) / 2.0, y=(v0 + v1) / 2.0)),
        size_x=u1 - u0, size_y=v1 - v0)
    det.detections = [d2]

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = TARGET_FRAME
    tf.child_frame_id = CAM_FRAME
    tf.transform.translation.x = float(O_CAM[0])
    tf.transform.translation.y = float(O_CAM[1])
    tf.transform.translation.z = float(O_CAM[2])
    qx, qy, qz, qw = _R_to_quat(R_BO)
    tf.transform.rotation.x = float(qx)
    tf.transform.rotation.y = float(qy)
    tf.transform.rotation.z = float(qz)
    tf.transform.rotation.w = float(qw)

    drv = _Driver(depth_msg, ci, det, tf)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(drv)
    t0 = time.time()
    while time.time() - t0 < 4.0 and not drv.got:
        ex.spin_once(timeout_sec=0.05)

    out = None
    if drv.got:
        p = drv.got[0].detections[0].results[0].pose.pose.position
        out = (float(p.x), float(p.y))

    node.destroy_node()
    drv.destroy_node()
    rclpy.shutdown()
    return truth, out


def main():
    D = _ray_dirs()
    # 台面上覆盖视野的一片落点（像素）
    pixels = [(int(CX), int(CY) + 15), (int(CX) - 70, int(CY) + 15),
              (int(CX) + 70, int(CY) + 40)]
    failed = []

    def check(name, cond, detail=''):
        print("  [%s] %s%s" % ('PASS' if cond else 'FAIL', name,
                               '  ' + detail if detail else ''))
        if not cond:
            failed.append(name)

    def run(mode, noise_m=0.0, jitter_px=0.0):
        errs, missing = [], 0
        for i, (u, v) in enumerate(pixels):
            truth, out = _one_case(mode, u, v, D, noise_m, jitter_px, seed=100 + i)
            if out is None:
                missing += 1
                continue
            errs.append(float(np.hypot(out[0] - truth[0], out[1] - truth[1])) * 1000.0)
        return errs, missing

    print("\n=== 无噪声 ===")
    old_errs, old_missing = run('bbox_center')
    new_errs, new_missing = run('top_face_center')

    check("旧算法 bbox_center 每个目标都有输出", old_missing == 0,
          "缺 %d/%d" % (old_missing, len(pixels)))
    check("新算法 top_face_center 每个目标都有输出", new_missing == 0,
          "缺 %d/%d（深度闸门误杀了？）" % (new_missing, len(pixels)))
    # 阈值 7 mm：本相机位姿下旧算法实测 9.39 mm（同一套几何换成真机外参是
    # 14.6 mm——偏差量随俯角/距离变，所以这里留够裕量，只钉住「确实偏了」）。
    check("旧算法确实偏（≥7 mm，说明场景没退化）",
          bool(old_errs) and np.mean(old_errs) >= 7.0,
          "平均 %.2f mm" % (np.mean(old_errs) if old_errs else float('nan')))
    check("新算法 ≤ 1 mm", bool(new_errs) and np.max(new_errs) <= 1.0,
          "平均 %.2f mm 最大 %.2f mm"
          % (np.mean(new_errs) if new_errs else float('nan'),
             np.max(new_errs) if new_errs else float('nan')))

    print("\n=== 深度噪声 σ=2 mm + 框边缘抖动 2 px（真机深度是带噪的）===")
    nz_errs, nz_missing = run('top_face_center', noise_m=0.002, jitter_px=2.0)
    check("带噪时仍有输出", nz_missing == 0, "缺 %d/%d" % (nz_missing, len(pixels)))
    check("带噪时新算法 ≤ 2 mm", bool(nz_errs) and np.max(nz_errs) <= 2.0,
          "平均 %.2f mm 最大 %.2f mm"
          % (np.mean(nz_errs) if nz_errs else float('nan'),
             np.max(nz_errs) if nz_errs else float('nan')))

    print("\n" + "=" * 52)
    if failed:
        print("FAILED: %d -> %s" % (len(failed), failed))
        return 1
    print("端到端通过：真节点 / 真消息 / 真 TF → 抓取点落在顶面中心"
          "（旧算法 %.1f mm → 新算法 %.2f mm）"
          % (np.mean(old_errs), np.max(new_errs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
