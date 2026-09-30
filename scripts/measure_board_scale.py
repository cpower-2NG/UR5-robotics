#!/usr/bin/env python3
"""用**打印出来的标定靶**反过来校验「像素 + 深度 → 米」这条链。

为什么需要这么一个东西
──────────────────────
`perception_3d/projector_node.py` 把检测框变成三维点，用的就是这条链：

    (u,v) + 深度 --[内参]--> 相机光学系三点 --> TF --> base_link

**整条抓取链路的高度、水平位置全部挂在它上面**，而它一旦有系统性的尺度误差，
外观上完全看不出来：投影出来的点依旧整齐、依旧贴着物体，只是**整体偏大或偏小**。
能发现它的只有一样东西 —— 画面里一个**已知尺寸**的实物。

标定靶正是为此打的：方格边长是**已知且可验**的（自带 100.0 mm 校验尺）。
于是：量它的方格在三维里是多少毫米，跟标称比，就得到这条链的尺度误差。

    python3 scripts/measure_board_scale.py --square-mm 30 --cols 12 --rows 7

`--cols/--rows` 给的是**内角点**数（OpenCV `patternSize`），不是方格数。
A3 那块 13×8 格 ⇒ (12, 7)；A4 那块 9×12 格 ⇒ (8, 11)。

⚠️ 两种误差要分开看，本脚本把两种都打出来：
  · **逐点深度**（`deproject`）：用每个角点自己那一点的深度。
  · **单应/平面**（`plane`）：只拟合一张台面，用平面方程求交。
    靶贴在台面上时两者应当接近；差得多，说明台面不平或靶翘了。
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_geometry_survey import depth_to_meters, deproject, ransac_planes  # noqa: E402


class Cam(Node):
    def __init__(self, a):
        super().__init__('measure_board_scale')
        self.info = self.color = self.depth = None
        self.create_subscription(CameraInfo, a.info_topic, self._i, qos_profile_sensor_data)
        self.create_subscription(Image, a.color_topic, self._c, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth_topic, self._d, qos_profile_sensor_data)

    def _i(self, m):
        self.info = m

    def _c(self, m):
        if self.color is None:
            self.color = m

    def _d(self, m):
        self.depth = m

    def _first(self, name):
        """只在第一帧时打一行 —— 排查「一条都收不到」还是「只缺某一路」。"""
        if name not in self._seen:
            self._seen.add(name)
            print('  ← 收到第一帧 %s' % name, flush=True)


def rgb_to_bgr(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return np.ascontiguousarray(a[:, :, ::-1])
    return a.copy()


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--color-topic', default='/camera/color/image_raw')
    ap.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    ap.add_argument('--info-topic', default='/camera/color/camera_info')
    ap.add_argument('--timeout', type=float, default=25.0)
    ap.add_argument('--square-mm', type=float, default=30.0, help='方格标称边长')
    ap.add_argument('--cols', type=int, default=12, help='内角点列数（patternSize 第一个数）')
    ap.add_argument('--rows', type=int, default=7, help='内角点行数')
    ap.add_argument('--save-dir', default='/tmp/board_scale')
    args = ap.parse_args()

    rclpy.init()
    node = Cam(args)
    t0 = time.time()
    while time.time() - t0 < args.timeout and not (
            node.info is not None and node.color is not None and node.depth is not None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.info is None or node.color is None or node.depth is None:
        # 说清**缺哪一样**：全缺多半是 DOMAIN_ID / 发现没配上，
        # 只缺一样则是那一路话题自己的问题，两者的查法完全不同。
        raise SystemExit('没收到帧：camera_info=%s color=%s depth=%s（等 %.0f s）'
                         % (node.info is not None, node.color is not None,
                            node.depth is not None, args.timeout))

    K = node.info.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    img = rgb_to_bgr(node.color)
    depth = depth_to_meters(node.depth)
    print('内参 fx=%.1f fy=%.1f cx=%.1f cy=%.1f  %dx%d'
          % (fx, fy, cx, cy, img.shape[1], img.shape[0]))

    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ⚠️ 两种**朝向**都要试，不能只按 (cols, rows) 试一次。
    # `findChessboardCorners` 对**非正方**图案不是旋转不变的：它按「长边横着」来找。
    # 腕部相机是侧装的，画面里台面**转了约 90°**，那块 13×8 的靶看上去就是 8×13 ——
    # 按 (12, 7) 试会**一个角点都找不到**，而这不是靶的问题、也不是反光的问题
    # （第一版就是这么误判的，差点又去怪打印质量）。
    # 内角点数只是行列互换，量出来的边长集合完全一样，所以掉个个儿没有副作用。
    found = None
    for ps in ((args.cols, args.rows), (args.rows, args.cols)):
        ok, corners = cv2.findChessboardCorners(
            gray, ps,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            found = ps
            break
    if found is None:
        os.makedirs(args.save_dir, exist_ok=True)
        p = os.path.join(args.save_dir, 'no_board.png')
        cv2.imwrite(p, img)
        raise SystemExit('没找到内角点（试过 (%d,%d) 与 (%d,%d) 两种朝向）；原图存 %s'
                         % (args.cols, args.rows, args.rows, args.cols, p))
    if found != (args.cols, args.rows):
        print('注意：靶在画面里转了 90°，按 patternSize=(%d, %d) 解出来的'
              % found)
    args.cols, args.rows = found

    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    C = corners.reshape(args.rows, args.cols, 2)
    print('找到内角点 %dx%d = %d 个' % (args.rows, args.cols, args.rows * args.cols))

    def depth_at(u, v, half=6):
        iu, iv = int(round(u)), int(round(v))
        w = depth[max(0, iv - half):iv + half + 1, max(0, iu - half):iu + half + 1]
        w = w[w > 0.05]
        return float(np.median(w)) if w.size else np.nan

    # ── 相邻角点的三维间距 ────────────────────────────────────────────
    def spacing(axis):
        """axis=1 取横向邻边，axis=0 取纵向邻边。返回每对邻边的三维长度（米）。"""
        out = []
        for r in range(args.rows):
            for c in range(args.cols):
                r2, c2 = (r, c + 1) if axis == 1 else (r + 1, c)
                if r2 >= args.rows or c2 >= args.cols:
                    continue
                u1, v1 = C[r, c]
                u2, v2 = C[r2, c2]
                z1, z2 = depth_at(u1, v1), depth_at(u2, v2)
                if not (np.isfinite(z1) and np.isfinite(z2)):
                    continue
                p1 = np.array([(u1 - cx) * z1 / fx, (v1 - cy) * z1 / fy, z1])
                p2 = np.array([(u2 - cx) * z2 / fx, (v2 - cy) * z2 / fy, z2])
                out.append(np.linalg.norm(p2 - p1))
        return np.array(out)

    h = spacing(1)
    v = spacing(0)
    allsp = np.concatenate([h, v])
    nom = args.square_mm / 1000.0

    print()
    print('由**逐点深度**反投影出来的相邻角点间距：')
    print('  横向 %d 条：中位 %.3f mm（标称 %.1f）' % (len(h), np.median(h) * 1e3, args.square_mm))
    print('  纵向 %d 条：中位 %.3f mm（标称 %.1f）' % (len(v), np.median(v) * 1e3, args.square_mm))
    print('  合计 %d 条：中位 %.3f ± %.3f mm  ⇒ 尺度比 **%.4f**'
          % (len(allsp), np.median(allsp) * 1e3, np.std(allsp) * 1e3,
             np.median(allsp) / nom))
    print('  （1.0000 = 这条链准；>1 = 偏大，即把东西量大了）')

    # ── 对照：只拟合一张台面，用平面方程求交（不含逐点深度噪声）──────
    pts, us, vs = deproject(depth, fx, fy, cx, cy, 0.15, 2.0)
    cands = ransac_planes(pts, thresh=0.008, iters=800, n_planes=3)
    best = max(cands, key=lambda c: int(c['mask'].sum()))
    n = np.asarray(best['n'], float)
    d = float(best['d'])
    if n[2] > 0:
        n, d = -n, -d
    P = []
    for r in range(args.rows):
        for c in range(args.cols):
            u, v_ = C[r, c]
            ray = np.array([(u - cx) / fx, (v_ - cy) / fy, 1.0])
            t = d / float(n @ ray)
            P.append(ray * t)
    P = np.array(P).reshape(args.rows, args.cols, 3)
    dh = np.linalg.norm(np.diff(P, axis=1), axis=2).ravel()
    dv = np.linalg.norm(np.diff(P, axis=0), axis=2).ravel()
    both = np.concatenate([dh, dv])
    print()
    print('由**台面平面求交**得到的间距（不逐点用深度）：')
    print('  合计 %d 条：中位 %.3f mm  ⇒ 尺度比 **%.4f**'
          % (len(both), np.median(both) * 1e3, np.median(both) / nom))

    # 平面拟合残差：靶若翘了，这里会大
    inl = pts[best['mask']]
    res = np.abs(inl @ n - d)
    print('  台面内点 %d 个，残差中位 %.2f mm，95 分位 %.2f mm'
          % (len(inl), np.median(res) * 1e3, np.percentile(res, 95) * 1e3))

    os.makedirs(args.save_dir, exist_ok=True)
    out = os.path.join(args.save_dir, 'detected.png')
    cv2.drawChessboardCorners(img, (args.cols, args.rows), corners, ok)
    cv2.imwrite(out, img)
    print()
    print('标注图 →', out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
