#!/usr/bin/env python3
"""俯视相机几何勘察 —— **纯只读**：只订阅话题，不发布、不写寄存器、不发运动。

## 为什么需要它

真机 demo 的第一件事是搞清楚相机**装在哪、朝哪、看得到多大一块桌面**。
本仓库的相机外参最终要靠手眼标定，而**手眼标定要动机械臂**（高风险操作，
需要当次授权 + 急停就绪）。但下面这些量**靠相机自己就测得到**，不用动臂：

  · 内参 `fx/fy/cx/cy` + 分辨率  → 视场角 FOV
  · 深度图拟合**桌面平面**        → 相机到桌面的距离、光轴与桌面的夹角
                                    （= 安装俯仰/滚转，俯视相机应当接近 0）
  · 四条角射线与桌面平面的交点     → 相机**看得到桌面上哪一块**（footprint）
  · 最近深度                      → 视野里有没有比桌面更近的实体（机械臂本体？
                                    相机支架？），以及它在画面里的位置

这三样合起来，才能回答「该打印什么标定板、放哪、够不够大」——也就是选
标定路线的前提。

## ⚠️ 坐标系：这里的一切都在**相机光学系**里

光学系约定（ROS `camera_*_optical_frame`）：**x 向右、y 向下、z 向前**（沿光轴）。
**不是 `base_link`**。要从光学系落到 `base_link` 必须做手眼标定 ——
而那正是本脚本要帮忙省掉动臂的那一步。所以本脚本的输出**不能**直接当外参用，
它是**外参的输入与验收依据**：标定完之后，标定结果反推出来的「相机到桌面的
距离」应当与本脚本测到的一致，对不上就是标定错了。

## 用法

    source /opt/ros/humble/setup.bash
    python3 scripts/camera_geometry_survey.py --save-dir /tmp/cam_survey

真机相机先起：
    ros2 launch yolo_bringup pipeline.launch.py camera_source:=realsense \\
        skip_detector:=true skip_planner:=true

只依赖 numpy + rclpy（板上都有）；cv2 只用来存 PNG，缺了也能跑完并出数字。
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# 深度编码 → 每单位多少米。与 grasp_planner / projector_node 的 depth_scale
# 约定一致：16UC1 是毫米，32FC1 已经是米。
_ENCODING_SCALE = {'16UC1': 0.001, '32FC1': 1.0}
_DEPTH_DTYPE = {'16UC1': np.uint16, '32FC1': np.float32}


def depth_to_meters(msg):
    """把 sensor_msgs/Image 变成 float32 的米制深度图。不认识的编码直接报错。"""
    if msg.encoding not in _ENCODING_SCALE:
        raise SystemExit(
            '不认识的深度编码 %r：本脚本只处理 %s。'
            '（真机 RealSense 的 depth 通常是 16UC1 毫米）'
            % (msg.encoding, ' / '.join(sorted(_ENCODING_SCALE)))
        )
    buf = np.frombuffer(msg.data, dtype=_DEPTH_DTYPE[msg.encoding])
    expected = msg.height * msg.width
    if buf.size != expected:
        raise SystemExit(
            '深度图字节数与 height*width 对不上：%d vs %d（step=%d，可能有行填充）'
            % (buf.size, expected, msg.step)
        )
    return buf.reshape(msg.height, msg.width).astype(np.float32) * _ENCODING_SCALE[msg.encoding]


def fit_plane(pts, iters=4, tol=0.008):
    """最小二乘拟合平面，迭代剔除离群点。返回 (单位法向 n, 标量 d) 使 n·p = d。

    用 SVD 而不是解正规方程：桌面点云在切向两个方向上铺得很开、在法向上很薄，
    正规方程的条件数会很难看。SVD 取最小奇异向量是这件事的标准做法。

    为什么要剔离群点：框里可能坐着机械臂、夹爪、方块，直接最小二乘会被它们拽偏。
    """
    P = pts
    n = None
    d = 0.0
    for i in range(iters):
        c = P.mean(axis=0)
        # 最后一列右奇异向量 = 法向（点云铺开的两个方向对应前两个奇异向量）
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        n = vt[2]
        # 定符号：让 n 指向相机（z 分量为正），这样 d 就是相机到平面的
        # **有向**距离，读起来就是「相机离桌面多远」。
        if n[2] < 0:
            n = -n
        d = float(n @ c)
        if i < iters - 1:
            keep = np.abs(P @ n - d) < tol
            if keep.sum() < 100:
                break
            P = P[keep]
    return n, d


def deproject(depth, fx, fy, cx, cy, min_z=0.05, max_z=10.0):
    """深度图 → 相机光学系下的点云 (N,3)，并返回保留像素的 (行,列) 索引。"""
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    z = depth
    ok = (z > min_z) & (z < max_z) & np.isfinite(z)
    z = z[ok]
    u = us[ok].astype(np.float32)
    v = vs[ok].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1), us[ok], vs[ok]


class Survey(Node):
    def __init__(self, args):
        super().__init__('camera_geometry_survey')
        self.args = args
        self.info = None
        self.depth = None
        self.color = None
        # 图像用 sensor_data QoS：相机发的是 BEST_EFFORT，
        # 用默认的 RELIABLE 会**一帧都收不到**（而且是静默的）。
        self.create_subscription(
            CameraInfo, args.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, args.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, args.color_topic, self._on_color, qos_profile_sensor_data)

    def _on_info(self, msg):
        self.info = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_color(self, msg):
        if self.color is None:            # 彩色只留第一帧，够看了
            self.color = msg

    def ready(self):
        return self.info is not None and self.depth is not None


def wait_for(node, rclpy_mod, cond, timeout, what):
    """转圈直到 cond() 为真或超时。超时**报错退出**而不是继续用空数据。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy_mod.spin_once(node, timeout_sec=0.1)
        if cond():
            return True
    raise SystemExit(
        '等 %s 超时（%.0f s）。话题名对不对？相机起了没有？'
        '（相机由 pipeline.launch.py camera_source:=realsense 起）' % (what, timeout)
    )


def main():
    ap = argparse.ArgumentParser(
        description='俯视相机几何勘察（只读）。输出数字 + 存一张彩色图供目视。')
    ap.add_argument('--color-topic', default='/camera/color/image_raw')
    ap.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw',
                    help='默认用**对齐到彩色**的深度（D2 的硬要求）')
    ap.add_argument('--info-topic', default='/camera/color/camera_info')
    ap.add_argument('--timeout', type=float, default=20.0, help='每个话题的等待上限（秒）')
    ap.add_argument('--frames', type=int, default=5,
                    help='平面拟合取几帧取中位数（单帧深度有噪点）')
    ap.add_argument('--save-dir', default='/tmp/cam_survey')
    args = ap.parse_args()

    rclpy.init()
    node = Survey(args)
    try:
        wait_for(node, rclpy, node.ready, args.timeout, '相机内参 + 深度帧')

        info = node.info
        # CameraInfo.k 是长度 9 的行主 3x3：k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # 显式按索引取，别写成 `fx = info.k`（那次就是踩了这个，报
        # 「only length-1 arrays can be converted to Python scalars」）。
        k = list(info.k)
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        w, h = info.width, info.height

        # ── 1) 内参与视场角 ────────────────────────────────────────────
        fov_h = 2 * math.degrees(math.atan(w / (2 * fx)))
        fov_v = 2 * math.degrees(math.atan(h / (2 * fy)))

        print('=' * 68)
        print('1) 内参 / 视场角   （frame_id = %s）' % info.header.frame_id)
        print('=' * 68)
        print('  分辨率        %d x %d' % (w, h))
        print('  fx, fy        %.3f , %.3f' % (fx, fy))
        print('  cx, cy        %.3f , %.3f   （画面中心 %d, %d）'
              % (cx, cy, (w - 1) / 2, (h - 1) / 2))
        print('  水平 FOV      %.2f°' % fov_h)
        print('  垂直 FOV      %.2f°' % fov_v)
        print('  畸变模型      %s  %s' % (info.distortion_model,
                                          np.round(info.d, 5).tolist()))

        # ── 2) 桌面平面拟合 ───────────────────────────────────────────
        # 多帧取中位数：单帧的深度噪点会让法向抖几度，而角度正是我们要判断的量。
        normals, dists, near_fracs = [], [], []
        pts_for_report = None
        for _ in range(args.frames):
            rclpy.spin_once(node, timeout_sec=1.0)
            if node.depth is None:
                continue
            depth = depth_to_meters(node.depth)
            pts, us, vs = deproject(depth, fx, fy, cx, cy)
            if pts.shape[0] < 1000:
                print('  ⚠️ 有效深度点只有 %d 个，跳过这一帧' % pts.shape[0])
                continue
            n, d = fit_plane(pts)
            normals.append(n)
            dists.append(d)
            # 比桌面近 >5 cm 的点占比 —— 机械臂/支架/方块会体现在这里
            resid = pts @ n - d
            near_fracs.append(float(np.mean(resid < -0.05)))
            if pts_for_report is None:
                pts_for_report = (pts, us, vs, depth)

        if not normals:
            raise SystemExit('一帧有效的深度都没拿到，无法拟合平面。')

        n = np.median(np.stack(normals), axis=0)
        n = n / np.linalg.norm(n)
        if n[2] < 0:
            n = -n
        dist = float(np.median(dists))

        # 光轴（相机 z 轴）与桌面法向的夹角 = 安装倾斜量。
        # 俯视相机正对桌面时该值 ≈ 0；不为 0 说明装歪了（俯仰或滚转）。
        tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))

        print()
        print('=' * 68)
        print('2) 桌面平面（相机光学系；%d 帧中位数）' % len(dists))
        print('=' * 68)
        print('  平面法向 n    [%.4f, %.4f, %.4f]' % tuple(n))
        print('  相机→桌面距离 %.4f m   （沿法向；正对时≈垂直高度）' % dist)
        print('  光轴与桌面夹角 %.2f°' % tilt)
        if tilt < 1.0:
            print('                ⇒ 基本正对桌面（俯视）。')
        elif tilt < 5.0:
            print('                ⇒ 轻微倾斜，标定时要按真实外参处理，不能假设正下方。')
        else:
            print('                ⇒ **明显倾斜**：任何「相机在正上方」的假设都不成立。')
        print('  比桌面近 >5cm 的像素占比  中位数 %.2f%%'
              % (100 * float(np.median(near_fracs))))

        # ── 3) 视野在桌面上的落点 ─────────────────────────────────────
        # 画面四角的射线打到桌面平面上的交点 = 相机看得到的那块桌面。
        ray_corners = [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]
        hits = []
        for (u, v) in ray_corners:
            dvec = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            dvec /= np.linalg.norm(dvec)
            denom = float(n @ dvec)
            if abs(denom) < 1e-6 or dist / denom <= 0:
                hits.append(None)
                continue
            hits.append(dvec * (dist / denom))

        print()
        print('=' * 68)
        print('3) 视野落桌面上的范围（四角射线 ∩ 桌面平面）')
        print('=' * 68)
        if all(h is not None for h in hits):
            for name, hp in zip(['左上', '右上', '右下', '左下'], hits):
                print('  %s  x=%+.3f  y=%+.3f  z=%+.3f m' % (name, hp[0], hp[1], hp[2]))
            # 沿光轴打到的那个点 = 画面正中心对应的桌面位置。
            # 这是「把标定板摆在相机正下方」时该摆的坐标。
            ctr = np.array([0.0, 0.0, 1.0]) * (dist / float(n[2]))
            arr = np.array(hits)
            print('  ---')
            print('  光轴落点    x=%+.3f  y=%+.3f  z=%+.3f m   ← 画面中心对着的桌面位置'
                  % tuple(ctr))
            print('  横向跨度    %.3f m  （x %.3f ~ %.3f）'
                  % (arr[:, 0].max() - arr[:, 0].min(), arr[:, 0].min(), arr[:, 0].max()))
            print('  纵向跨度    %.3f m  （y %.3f ~ %.3f）'
                  % (arr[:, 1].max() - arr[:, 1].min(), arr[:, 1].min(), arr[:, 1].max()))
            print('  ⚠️ 这是**斜视**时的梯形范围；相机正对桌面时它才是矩形。')
        else:
            print('  ⚠️ 有角射线打不到桌面平面（平面不在视野正前方），跳过。')

        # ── 4) 视野里有没有比桌面更近的实体 ───────────────────────────
        pts, us, vs, depth = pts_for_report
        resid = pts @ n - dist
        near = resid < -0.05
        print()
        print('=' * 68)
        print('4) 视野内的实体（比桌面近 >5 cm 的点）')
        print('=' * 68)
        valid = depth[depth > 0.05]
        if valid.size == 0:
            print('  ⚠️ 这一帧没有有效深度，跳过。')
        else:
            print('  深度范围      %.3f ~ %.3f m' % (float(valid.min()), float(valid.max())))
            print('  中位深度      %.3f m' % float(np.median(valid)))
        if near.sum() < 50:
            print('  几乎没有 → 视野里只看到桌面（没有机械臂本体入镜）。')
        else:
            # resid = 点到平面的**有向**距离，负 = 在相机这一侧（比桌面近）。
            # 所以「比桌面近多少」= -min(resid)，读起来最直观。
            print('  共 %d 个点（%.2f%%），最靠近相机的比桌面近 %.3f m'
                  % (near.sum(), 100 * near.sum() / len(resid),
                     float(-resid[near].min())))
            print('  像素包围盒    u %d~%d , v %d~%d  （画面 %dx%d）'
                  % (us[near].min(), us[near].max(), vs[near].min(), vs[near].max(), w, h))
            print('  ⇒ 有东西入镜，标定和抓取前要确认它是不是机械臂/支架。')

        # ── 5) 存图（彩色供**目视**确认看到的是什么） ──────────────────
        os.makedirs(args.save_dir, exist_ok=True)
        saved = []
        if node.color is not None:
            p = os.path.join(args.save_dir, 'color.png')
            if _save_color(node.color, p):
                saved.append(p)
        p = os.path.join(args.save_dir, 'depth.png')
        if _save_depth_vis(depth, p):
            saved.append(p)

        out = {
            'frame_id': info.header.frame_id,
            'width': w, 'height': h,
            'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
            'fov_h_deg': fov_h, 'fov_v_deg': fov_v,
            'distortion_model': info.distortion_model,
            'd': list(info.d),
            'plane_normal': [float(x) for x in n],
            'distance_m': dist,
            'tilt_deg': tilt,
            'axis_hit_m': [float(x) for x in (np.array([0.0, 0.0, 1.0]) * (dist / float(n[2])))],
            'corner_hits_m': [None if hp is None else [float(x) for x in hp] for hp in hits],
            'near_entity_fraction': float(np.median(near_fracs)),
            'note': '全部在相机光学系（x右/y下/z前），不是 base_link；'
                    '要落到 base_link 必须做手眼标定。',
        }
        p = os.path.join(args.save_dir, 'survey.json')
        with open(p, 'w') as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        saved.append(p)

        print()
        print('存到：')
        for s in saved:
            print('  ' + s)
        print('  （彩色图拿下来目视一眼 —— 数字说不出「画面里到底有什么」）')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _save_color(msg, path):
    """存彩色图。cv2 只用于这一步：缺了不致命，数字部分照样出得来。"""
    try:
        import cv2
    except ImportError:
        print('  （没装 cv2，跳过存图）')
        return False
    if msg.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            a = a[:, :, ::-1]
        cv2.imwrite(path, a)
        return True
    print('  （彩色编码 %r 不认识，跳过存图）' % msg.encoding)
    return False


def _save_depth_vis(depth, path):
    """深度图存成伪彩，纯粹为了目视。"""
    try:
        import cv2
    except ImportError:
        return False
    d = depth.copy()
    d[~np.isfinite(d)] = 0
    d[d <= 0] = 0
    valid = d > 0
    if not valid.any():
        return False
    lo, hi = float(np.percentile(d[valid], 2)), float(np.percentile(d[valid], 98))
    vis = np.clip((d - lo) / max(1e-6, hi - lo), 0, 1)
    vis = (255 * (1.0 - vis)).astype(np.uint8)   # 近=亮，符合直觉
    vis[~valid] = 0
    cv2.imwrite(path, cv2.applyColorMap(vis, cv2.COLORMAP_JET))
    return True


if __name__ == '__main__':
    main()
