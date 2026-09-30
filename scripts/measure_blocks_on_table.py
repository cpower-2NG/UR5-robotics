#!/usr/bin/env python3
"""量工作台上物体的**尺寸**（**纯只读**：只订阅话题，不动臂、不写任何寄存器）。

## 为什么用「顶面高出桌面的高度」，而不是颜色分割
方块是立方体 ⇒ **顶面到桌面的高度 = 棱长**。这个量比按颜色框出来量宽度可靠得多：

  · **不依赖颜色** —— 台上还有同色系的水果（桃子/李子），棋盘格是反光的黑白纸，
    按颜色分割会把它们混进来（第五十五轮就是这么估出「3.3–4.0 cm」的，不可靠）；
  · **对边缘误差不敏感** —— 只取顶面那一小片点，不碰物体轮廓；
  · 用的是与 `perception_3d` / `projector_node.py` **完全相同**的深度 + 内参数学
    ⇒ 顺带把感知链路验一遍。

## 为什么每个物体都要**局部**重拟合桌面
D435i 在这个视场、这个距离上的平面度误差约 **±8 mm**（实测见
`RESTRUCTURE_PLAN.md` 第五十六轮 §二·补）。拿**全局**平面去量一个 29 mm 的方块，
那就是 **28%** 的误差 —— 量出来的数会毫无意义。
所以每个物体都用**它周围一小圈桌面点**重新拟合一个局部平面：平面度误差是个
**缓慢变化**的场，在十几厘米的尺度上小得多。

## 输出
每个物体一行：点数、**顶面高度（= 棱长）**、切向包围盒、等效边长。另存一张
标注图，把每个簇框出来并写上量到的数 —— **数字说不出画面里那个东西是什么**。

## 它不是什么
**不是标定**，也不是外参。输出全部在**相机光学系**的桌面切向坐标里，与
`base_link` 无关。它回答的是「这块方块多大」，不是「它在机器人哪儿」。

用法：
    python3 scripts/measure_blocks_on_table.py --save-dir /tmp/blocks
"""

import argparse
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# 复用勘察脚本里已经验证过的平面拟合与反投影，避免两份实现漂移。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_geometry_survey import depth_to_meters, deproject, ransac_planes  # noqa: E402


def tangent_basis(n):
    """由平面法向 n 造一组切向正交基 (e1, e2)，使 (e1, e2, n) 右手。"""
    tmp = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(n, tmp)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    return e1, e2


class Cam(Node):
    def __init__(self, a):
        super().__init__('measure_blocks_on_table')
        self.info = self.depth = self.color = None
        # 图像必须用 sensor_data QoS：相机发的是 BEST_EFFORT，
        # 用默认的 RELIABLE 会**一帧都收不到**（而且是静默的）。
        self.create_subscription(CameraInfo, a.info_topic, self._i, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth_topic, self._d, qos_profile_sensor_data)
        self.create_subscription(Image, a.color_topic, self._c, qos_profile_sensor_data)

    def _i(self, m):
        self.info = m

    def _d(self, m):
        self.depth = m

    def _c(self, m):
        if self.color is None:
            self.color = m


def table_region_mask(us, vs, is_plane, shape):
    """台面在画面里的**连通区域**（并把摆在上面的东西留下的洞填掉）。

    为什么不能只靠「到平面的距离」：**平面是无限的，桌子不是。**
    越过桌子远边/左边打出去的射线，会落到地板、墙、示教器上，而这些东西在
    「离这张平面多远」这个尺度上可以很接近 —— 尤其是掠射视角，整片地板都可能
    落在「桌面上方」那一侧。RANSAC 的内点只保证「离平面近」，**不保证「在桌子上」**。

    所以取内点掩码里**最大的那个连通块**（桌子是一整片，别的东西是散的），
    再把洞填上 —— 摆在桌面上的东西，在这个掩码里就是洞，填完正好恢复它们的脚印。
    这个区域就是「桌面在画面的哪里」，物体必须落在它里面。
    """
    import cv2
    m = np.zeros(shape, np.uint8)
    m[vs[is_plane], us[is_plane]] = 1
    n_lab, lab = cv2.connectedComponents(m, connectivity=8)
    if n_lab <= 1:
        return m.astype(bool), 0
    size = np.bincount(lab.ravel())
    size[0] = 0                       # 0 号是背景，不算
    cc = (lab == size.argmax()).astype(np.uint8)
    # 洞 = 背景里**没被从画面角落泛洪淹到**的部分。
    inv = (1 - cc).astype(np.uint8)
    holes = np.zeros(shape, bool)
    for sx, sy in ((0, 0), (shape[1] - 1, 0), (0, shape[0] - 1),
                   (shape[1] - 1, shape[0] - 1)):
        if inv[sy, sx] == 0:          # 这个角不是背景，换个角
            continue
        ff = inv.copy()
        cv2.floodFill(ff, np.zeros((shape[0] + 2, shape[1] + 2), np.uint8), (sx, sy), 2)
        holes = (ff == 1)
        break
    return (cc | holes).astype(bool), int(size[1:].sum() - size.max())


def cluster_2d(u, v, cell):
    """把 (u, v)（米）量化成格子后做 8 连通聚类。返回每个输入点的簇号（-1 = 无）。

    用连通分量而不是 KMeans：**点数未知**（台上几个东西事先不知道），
    而且互相挨着的方块本来就该被分开看 —— 连通分量在物理上就是「连成一片的表面」。
    """
    import cv2
    iu = np.floor((u - u.min()) / cell).astype(np.int32)
    iv = np.floor((v - v.min()) / cell).astype(np.int32)
    gw, gh = int(iu.max()) + 1, int(iv.max()) + 1
    if gw * gh > 40_000_000:
        raise SystemExit('格子太密（%d x %d），把 --cell-mm 调大' % (gw, gh))
    grid = np.zeros((gh, gw), np.uint8)
    grid[iv, iu] = 1
    _, lab = cv2.connectedComponents(grid, connectivity=8)
    return lab[iv, iu], (iu, iv)


def hue_group(rgb):
    """按色相把点粗分成几组。

    **颜色不是深度的函数** —— 台上有 5 块积木（黄绿 1 / 紫 1 / 橙红 3），挨着的
    会在深度上粘成一个簇。只报合并后的数，会把「三块 30 mm」和「一块 25 + 一块 35」
    报成同一个数。用颜色把它切开，是一条与深度**完全独立**的第二意见。
    低饱和度的点（灰白的桌面、反光的棋盘格）不参与分色，返回 -1。
    """
    import cv2
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb.reshape(-1, 1, 3)), cv2.COLOR_RGB2HSV)
    hsv = hsv.reshape(-1, 3)
    h = hsv[:, 0].astype(np.int32) * 2          # OpenCV 的 H 是 0..179，乘 2 回到度数
    s, v = hsv[:, 1].astype(np.int32), hsv[:, 2].astype(np.int32)
    g = np.full(len(h), -1)
    ok = (s > 60) & (v > 40)
    g[ok & ((h < 25) | (h >= 330))] = 0         # 红 / 橙
    g[ok & (h >= 25) & (h < 95)] = 1            # 黄 / 绿
    g[ok & (h >= 95) & (h < 200)] = 2           # 青 / 蓝
    g[ok & (h >= 200) & (h < 330)] = 3          # 紫 / 品红
    return g


HUE_NAMES = {0: '红橙', 1: '黄绿', 2: '青蓝', 3: '紫'}


def footprint_pca(Pp, n, hh):
    """顶面脚印：取高度**上半**的点，在桌面切向平面里做 PCA，返回 (长边, 短边, 点数)。

    ⚠️ 为什么不用固定毫米的「顶面带」（`--top-band-mm`）：这个坑是实测出来的 ——
    同一个方块，带从 3 mm 放宽到 16 mm，顶面脚印从 **13.7×17.0 一路涨到 31.1×32.7 mm**，
    而它的**整体**包围盒在四个带下都是 31.2×34.6、p95 都在 27~32，纹丝不动。
    也就是说固定带宽量出来的不是几何，**是带宽本身**：窄于深度噪声只捞到顶面的一角，
    宽于噪声就把侧面也捞进来。当时的症状是「绿块紫块的脚印(20~23)反而小于高度(27~30)」，
    看上去像方块不是立方体 —— 其实是我的尺子在动。「按高度取上半」没有这个旋钮。

    用**协方差主轴**而不是全局切向基：方块摆在台面上通常带偏航，轴对齐包围盒会
    按 cos 误差把边长量小。边长取 1/99 分位而不是 max-min，免得边缘的深度噪声点充气。
    """
    m = hh >= np.percentile(hh, 50)
    if int(m.sum()) < 30:
        return float('nan'), float('nan'), int(m.sum())
    e1, e2 = tangent_basis(n)
    Q = Pp[m]
    p = Q @ e1
    v = Q @ e2
    # ⚠️ 堆成 (N,2) 交给 svd，`vt` 才是 (2,2)、`vt[0]` 才是**二维**的主方向。
    # 堆成 (2,N) 的话 `full_matrices=False` 会让 `vt` 变成 (2,N)，`vt[0]` 是长 N 的
    # 向量，`vt[0] @ A` 直接维度对不上。
    A = np.stack([p - p.mean(), v - v.mean()], axis=1)
    _, _, vt = np.linalg.svd(A, full_matrices=False)
    a, b = A @ vt[0], A @ vt[1]
    a_lo, a_hi = np.percentile(a, [1, 99])
    b_lo, b_hi = np.percentile(b, [1, 99])
    return float(a_hi - a_lo), float(b_hi - b_lo), int(m.sum())


def measure_group(Pp, uu, vv, n, d, table_pts, Tu, Tv, args):
    """量一组点：它高出桌面的**高度**，以及它**顶面自己的脚印**。

    `table_pts`（下标与 `Tu`/`Tv` 对应）是全量桌面点，用来在这个物体**周围**
    重新拟合一个局部桌面平面。平面度误差是缓慢变化的场，十几厘米的尺度上小得多
    —— 本脚本的精度就来自这一步；全局平面在这个距离上只有 ±8 mm。
    """
    cu, cv = float(np.median(uu)), float(np.median(vv))
    r, r0 = args.local_radius_mm / 1000.0, args.local_inner_mm / 1000.0
    dist = np.hypot(Tu - cu, Tv - cv)
    near = (dist > r0) & (dist < r)
    if int(near.sum()) >= 200:
        c = table_pts[near].mean(axis=0)
        _, _, vt = np.linalg.svd(table_pts[near] - c, full_matrices=False)
        nl = vt[2]
        # ⚠️ 用**与全局 n 同向**来定向，不要用 `nl[2] > 0` —— 外面已经翻过符号，
        # 此时桌面法向的 z 分量是**负**的，按 z 定向会把符号翻回去。
        if float(nl @ n) < 0:
            nl = -nl
        dl = float(nl @ c)
        basis = 'local%d' % int(near.sum())
    else:
        nl, dl, basis = n, d, 'global'

    hh = Pp @ nl - dl                        # 相对**局部**桌面的高度
    top = float(np.percentile(hh, 95))       # 粗估：先取高分位，避开侧面的点

    # ── 精估：把**顶面本身**拟合成一个平面 ────────────────────────────
    # 比「取个高分位」强在三点：① 顶面是真实存在的平面，用平面拟合才是用对了模型；
    # ② 顺带得到一个**自检** —— 顶面相对桌面若明显不平行，说明这个簇里混了不同
    # 高度的东西，这时报出来的数就不可信。方块是立方体，顶面倾角应当接近 0。
    band = hh > (top - args.top_band_mm / 1000.0)
    tilt = float('nan')
    if int(band.sum()) >= 30:
        ct = Pp[band].mean(axis=0)
        _, _, vt2 = np.linalg.svd(Pp[band] - ct, full_matrices=False)
        nt = vt2[2]
        if float(nt @ n) < 0:
            nt = -nt
        # 顶面中心到局部桌面平面的**有向距离**（沿桌面法向）。
        # ⚠️ 用 nl 而不是 nt：顶面若略有倾斜，两平面之间没有单一的「垂距」，
        # 要的是**物体中心处**的高度，不是一个无定义的平均。
        top = float(nl @ ct - dl)
        tilt = float(np.degrees(np.arccos(min(1.0, abs(float(nt @ nl))))))

    # ── 第二把尺子：顶面自己的脚印 ────────────────────────────────────
    # 顶面是水平的，它在切向平面上的尺寸**不受俯仰视角影响**（掠射会把高度量歪，
    # 但量一个水平面的宽度是另一回事）。立方体 ⇒ 顶面边长应当**等于**高度。
    # 一个系统性的高度偏差，只有这条路能发现它。
    fu, fv, fn = footprint_pca(Pp, n, hh)
    return {
        'top': top, 'band': band, 'tilt': tilt, 'n': int(band.sum()),
        'du': fu, 'dv': fv, 'fnn': fn,
        'p5': float(np.percentile(hh, 5)), 'p95': float(np.percentile(hh, 95)),
        'cu': cu, 'cv': cv, 'basis': basis,
    }


def px_note(pu, pv, Pp):
    """把一个簇的**像素**尺寸和平均距离也打出来。

    为的是能一眼核对「毫米」是不是自洽的：`边长 ≈ 像素数 × Z / fx`。
    只看毫米数时，几倍的系统性错误（内参错、深度尺度错、簇选错了东西）是看不出来的；
    把像素数和 Z 并排放在同一行，`像素 × Z / fx` 一算就对上了或者对不上。
    """
    if len(pu) == 0:
        return ''
    return '  [%d x %d px, Z=%.3f m]' % (
        int(pu.max() - pu.min()), int(pv.max() - pv.min()),
        float(np.linalg.norm(Pp, axis=1).mean()))


def fmt_row(tag, cnt, r, ext_u, ext_v, note=''):
    side = 0.5 * (r['du'] + r['dv'])
    ratio = ('%.2f' % (r['top'] / side)) if side > 0.001 else ' -- '
    return ('  %-6s %5d  高%6.1f  顶面%5.1f x %5.1f  整体%5.1f x %5.1f  高/边%s  '
            '分位%5.1f/%5.1f  脚印%d点 斜%4.1f°  %s%s'
            % (tag, cnt, r['top'] * 1000, r['du'] * 1000, r['dv'] * 1000,
               ext_u * 1000, ext_v * 1000, ratio, r['p5'] * 1000, r['p95'] * 1000,
               r['fnn'], r['tilt'], r['basis'], note))


def measure(args):
    rclpy.init()
    node = Cam(args)
    t0 = time.time()
    while time.time() - t0 < args.timeout and not (node.info and node.depth):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not (node.info and node.depth):
        raise SystemExit('等相机超时。相机起了没有？（pipeline.launch.py camera_source:=realsense）')

    k = list(node.info.k)               # 别写 `fx = info.k`：那会给整个数组
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    w, h = node.info.width, node.info.height

    # ── 1) 多帧取中位数，定出桌面平面 ────────────────────────────────
    normals, dists, pts_ref = [], [], None
    for _ in range(args.frames):
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.depth is None:
            continue
        depth = depth_to_meters(node.depth)
        pts, us, vs = deproject(depth, fx, fy, cx, cy, max_z=args.max_depth)
        if pts.shape[0] < 5000:
            continue
        cands = ransac_planes(pts, thresh=args.plane_tol,
                              iters=args.ransac_iters, n_planes=1,
                              max_points=args.ransac_points)
        if not cands:
            continue
        normals.append(cands[0]['n'])
        dists.append(cands[0]['d'])
        if pts_ref is None:
            pts_ref = (pts, us, vs, depth)
    if not normals:
        raise SystemExit('没拟合出桌面平面。试试放宽 --max-depth。')

    n = np.median(np.stack(normals), axis=0)
    n /= np.linalg.norm(n)
    if n[2] < 0:
        n = -n
    d = float(np.median(dists))

    # ── ⚠️ 符号：本脚本最容易错的一处，实测栽过一次 ────────────────────
    # `refine_plane` / RANSAC 把法向强制成 n[2] > 0，也就是**沿光轴指向场景深处**。
    # 相机俯视桌面时，「深处」恰恰是**桌面下方**。于是按 P·n−d 算出来的「高度」是反的：
    # 实测积木量成 −27 mm，而地板/背景占满了 +60…+100 mm 那一档，
    # 选出来的「物体」全在地上和墙上（标注图里一目了然，台上一个框都没有）。
    #
    # 相机在原点，它到平面的有向距离是 −d。**相机在桌面之上** ⇒ 「桌面上方」就是
    # 相机所在的那一侧 ⇒ 取
    #       H = d − P·n     （相机处 H = +d > 0）
    # 并把 n、d 一起取反，使后续「+n = 桌面朝上」这个约定继续成立。
    # ⚠️ 由此**不能**再用 `nl[2] > 0` 去定向局部平面（那会把符号翻回去），
    #    必须改成与全局 n 同向 —— 见下面局部重拟合那段。
    n, d = -n, -d

    pts, us, vs, depth = pts_ref

    # 彩色帧（分色用）。取**此刻最新的**彩色帧，而不是上面那个循环里某一帧 ——
    # 循环里取的话，彩色回调可能还没触发过（实测就是这样），col 会是 None，
    # 于是分色**一声不响地不生效**。场景是静止的，晚一帧没有关系。
    color_ref = node.color
    # **必须与深度帧同尺寸**才能按下标取色，尺寸对不上就放弃分色，
    # 而不是错位地取一堆颜色。
    col = None
    if color_ref is not None and color_ref.encoding in ('rgb8', 'bgr8'):
        c = np.frombuffer(color_ref.data, np.uint8).reshape(
            color_ref.height, color_ref.width, 3)
        if color_ref.encoding == 'bgr8':
            c = np.ascontiguousarray(c[:, :, ::-1])
        if c.shape[:2] == depth.shape:
            col = c
        else:
            print('⚠️ 彩色帧 %s 与深度帧 %s 尺寸不一致，跳过分色'
                  % (c.shape[:2], depth.shape))

    e1, e2 = tangent_basis(n)
    U = pts @ e1
    V = pts @ e2
    H = pts @ n - d                     # 有向高度：正 = 在桌面上方（相机那一侧）

    print('=' * 72)
    print('桌面平面：法向 [%.4f %.4f %.4f]   相机→面 %.4f m   光轴夹角 %.2f°'
          % (n[0], n[1], n[2], d, np.degrees(np.arccos(min(1.0, abs(n[2]))))))
    print('深度点 %d 个   高度分布：min %.3f  中位 %.4f  max %.3f m'
          % (len(pts), H.min(), np.median(H), H.max()))

    # ── 1b) 高度直方图 ────────────────────────────────────────────────
    # 掠射视角下这张图很必要：平面是**无限延伸**的，越过桌子远边的射线打到的墙、
    # 臂、支架都落在同一个平面方程上，会在很大的 H 处形成第二团。
    # 看直方图才知道「多少毫米以上就不再是台面上的东西了」，而不是拍一个 --max-height。
    edges = [-40, -20, -10, -5, 0, 5, 10, 15, 20, 25, 30, 40, 60, 100, 200]
    hist, _ = np.histogram(H * 1000.0, bins=edges)
    tot = max(1, len(H))
    print('高度直方图（mm → 点数）：')
    for i in range(len(edges) - 1):
        bar = '#' * int(round(60.0 * hist[i] / hist.max())) if hist.max() else ''
        print('   %+5d…%+5d : %7d  %5.1f%%  %s'
              % (edges[i], edges[i + 1], hist[i], 100.0 * hist[i] / tot, bar))

    # ── 2) 选出「在桌面上方 lo~hi mm」的点 = 台面上的物体 ─────────────
    # 再加一道**相机距离**上限：桌子是有边的，平面没有。越过桌子远边的射线打在
    # 背景墙上，会以「高出桌面很多」的姿态混进来 —— 靠 H 上限切不干净（墙很高），
    # 靠距离上限一刀就干净了。
    # ── 2) 定位：先粗后精，**不能一步到位** ────────────────────────────
    # ⚠️ 这里栽过第二次。**不能**在「桌面上方 3 mm」这么低的阈值上直接选点：
    # 相机对这张桌面的平面模型只有 **±8 mm** 的精度（第五十六轮 §二·补实测），
    # 那一圈涟漪在整张桌面上是**连通**的，会把积木、棋盘格、水果全粘成
    # 一个 500×600 mm 的大团（实测的 #4 簇正是如此），然后再被「太大 ⇒ 背景」丢掉。
    # 于是台上的东西一个都报不出来 —— 症状和选错符号那次一模一样，但原因不同。
    #
    # 所以分两步，各司其职：
    #   · 第一步（定位）：用一个**只负责找到东西在哪**的粗阈值（`--locate-mm`，
    #     默认 12 mm，高于 ±8 mm 的涟漪）分块。它不需要准，只需要不粘连。
    #   · 第二步（测量）：对每个块**局部重拟合**它自己的桌面零点，再量高度。
    #     平面度误差是缓慢变化的场，十几厘米的尺度上小得多 —— 精度来自这一步。
    hi = args.max_height_mm / 1000.0
    range_ok = np.linalg.norm(pts, axis=1) < args.max_range
    in_height = (H > args.locate_mm / 1000.0) & (H < hi)

    # 台面区域闸门 —— 见 table_region_mask 的注释。没有这一道，地板和墙会混进来。
    region, n_other = table_region_mask(us, vs, np.abs(H) < args.plane_tol, depth.shape)
    print('台面区域：占画面 %.1f%%（另有 %d 个游离的平面内点，不属于台面连通块）'
          % (100.0 * region.mean(), n_other))
    in_region = region[vs, us]

    sel = in_height & range_ok & in_region
    if sel.sum() < args.min_points:
        raise SystemExit('桌面上方 %g~%g mm 之间的点只有 %d 个，太少了。'
                         '台上摆东西了吗？' % (args.locate_mm, args.max_height_mm, sel.sum()))
    print('定位阈值 %.0f mm 以上：%d 个 → 距离上限挡掉 %d → **台面区域挡掉 %d** '
          '→ 剩 %d 个（%.2f%%）'
          % (args.locate_mm, int(in_height.sum()),
             int((in_height & ~range_ok).sum()),
             int((in_height & range_ok & ~in_region).sum()),
             sel.sum(), 100 * sel.sum() / len(pts)))

    # ── 3) 切向平面上聚类 ────────────────────────────────────────────
    lab, _ = cluster_2d(U[sel], V[sel], args.cell_mm / 1000.0)
    Su, Sv = U[sel], V[sel]

    table = np.abs(H) < args.plane_tol          # 用来重拟合局部平面的桌面点
    Tu, Tv = U[table], V[table]

    print()
    print('=' * 72)
    print('检测到的物体（每个都用**它周围的局部桌面**重拟合后量的高度）')
    print('=' * 72)
    print('  #    点数   高度=棱长   顶面脚印(mm)      整体包围盒(mm)    高/边  '
          '分位p5/p95   脚印点数 倾角   basis')
    rows = []
    for cid in range(1, int(lab.max()) + 1):
        m = lab == cid
        if int(m.sum()) < args.min_points:
            continue
        uu, vv = Su[m], Sv[m]
        Pp = pts[sel][m]
        ext_u = float(uu.max() - uu.min())
        ext_v = float(vv.max() - vv.min())
        eq = float(np.sqrt(ext_u * ext_v))      # 等效边长（包围盒的几何平均）
        too_big = eq * 1000 > args.max_extent_mm
        r = measure_group(Pp, uu, vv, n, d, pts[table], Tu, Tv, args)
        print(fmt_row(str(cid), int(m.sum()), r, ext_u, ext_v,
                      px_note(us[sel][m], vs[sel][m], Pp)
                      + ('   ← 太大（>%.0f mm），判为背景，不计' % args.max_extent_mm
                         if too_big else '')))

        # 簇内部再按颜色切一刀 —— 与深度完全独立的第二意见，见 hue_group。
        if args.split_color and col is not None:
            g = hue_group(col[vs[sel][m], us[sel][m]])
            counts = {HUE_NAMES[k]: int((g == k).sum()) for k in (0, 1, 2, 3)}
            counts['无色'] = int((g < 0).sum())
            if m.sum() >= 500:
                # 把分色结果**打出来**：分色若不生效，看到的应该是一堆 0 和
                # 一个占满的无色 —— 那说明阈值不对，而不是「这一团颜色很纯」。
                print('       └ #%d 分色: %s' % (cid, counts))
            present = [k for k in (0, 1, 2, 3) if int((g == k).sum()) >= args.min_points
                       and int((g == k).sum()) < 0.95 * m.sum()]
            if len(present) > 1 and not too_big:
                for k in present:
                    mm = g == k
                    r2 = measure_group(Pp[mm], uu[mm], vv[mm], n, d,
                                       pts[table], Tu, Tv, args)
                    print(fmt_row(' %d-%s' % (cid, HUE_NAMES[k]), int(mm.sum()), r2,
                                  float(uu[mm].max() - uu[mm].min()),
                                  float(vv[mm].max() - vv[mm].min()),
                                  px_note(us[sel][m][mm], vs[sel][m][mm], Pp[mm])))

        if too_big:
            continue
        rows.append((cid, int(m.sum()), r['top'], ext_u, ext_v, eq,
                     r['cu'], r['cv'], r['basis']))

    if not rows:
        print('  （没有够大的簇）')

    # ── 4) 存一张标注图：数字说不出画面里那个东西是什么 ────────────────
    os.makedirs(args.save_dir, exist_ok=True)
    if node.color is not None:
        _annotate(node.color, args, sel, lab, U, V, us, vs, rows,
                  os.path.join(args.save_dir, 'annotated.png'))
        print()
        print('标注图 →', os.path.join(args.save_dir, 'annotated.png'),
              '（**拿下来目视**：簇对不对得上实物）')
    node.destroy_node()
    rclpy.shutdown()
    return rows


def _annotate(msg, args, sel, lab, U, V, us, vs, rows, path):
    """把每个簇框在彩色图上并写上量到的数。仅用于目视核对。"""
    try:
        import cv2
    except ImportError:
        print('（没装 cv2，跳过标注图）')
        return
    if msg.encoding not in ('rgb8', 'bgr8'):
        return
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        # ⚠️ `a[:, :, ::-1]` 是**负步长视图**，不是连续内存；直接把它交给 cv2.rectangle
        # 会报「Layout of the output array img is incompatible」。必须再 copy 一次。
        a = np.ascontiguousarray(a[:, :, ::-1])
    else:
        a = a.copy()
    su, sv, sl = us[sel], vs[sel], lab
    for (cid, cnt, top, eu, ev, eq, cu, cv_, basis) in rows:
        m = sl == cid
        x0, x1 = int(su[m].min()), int(su[m].max())
        y0, y1 = int(sv[m].min()), int(sv[m].max())
        cv2.rectangle(a, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.putText(a, '#%d  %.1fmm' % (cid, top * 1000), (x0, max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.imwrite(path, a)


def main():
    ap = argparse.ArgumentParser(
        description='量工作台上物体的尺寸（只读）。核心量是「顶面高出桌面的高度」。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--color-topic', default='/camera/color/image_raw')
    ap.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    ap.add_argument('--info-topic', default='/camera/color/camera_info')
    ap.add_argument('--save-dir', default='/tmp/blocks')
    ap.add_argument('--timeout', type=float, default=25.0)
    ap.add_argument('--frames', type=int, default=3)
    ap.add_argument('--max-depth', type=float, default=2.0,
                    help='只把这么近的点喂给平面拟合')
    ap.add_argument('--plane-tol', type=float, default=0.008)
    ap.add_argument('--ransac-iters', type=int, default=600)
    ap.add_argument('--ransac-points', type=int, default=20000)
    ap.add_argument('--locate-mm', type=float, default=12.0,
                    help='**定位**阈值：只负责找到东西在哪，不负责量它多高。'
                         '必须高于相机对这张桌面的平面模型误差（实测 ±8 mm），'
                         '否则涟漪会把整张桌面连成一片、把所有东西粘成一个大团')
    ap.add_argument('--min-height-mm', type=float, default=None,
                    help='已废弃，等同于 --locate-mm（保留只为老命令行不报错）')
    ap.add_argument('--max-height-mm', type=float, default=70.0,
                    help='高于这个高度当成背景/机械臂，不当物体')
    ap.add_argument('--max-range', type=float, default=1.30,
                    help='距相机这么远的点不要。**桌面有边、平面没有** —— 越过桌子远边'
                         '打到墙上的射线会混进来，靠这个一刀切掉')
    ap.add_argument('--max-extent-mm', type=float, default=250.0,
                    help='切向包围盒等效边长超过这个值的簇判为背景，不计入')
    ap.add_argument('--cell-mm', type=float, default=4.0, help='聚类格边长')
    ap.add_argument('--min-points', type=int, default=40, help='小于这个点数的簇丢掉')
    ap.add_argument('--local-radius-mm', type=float, default=120.0,
                    help='局部重拟合平面用的桌面点半径（**这是精度的关键**）')
    ap.add_argument('--local-inner-mm', type=float, default=35.0,
                    help='这个半径以内不取（那多半是物体自己）')
    ap.add_argument('--top-band-mm', type=float, default=3.0,
                    help='从粗估顶面向下取这么厚的一层点去拟合顶面')
    ap.add_argument('--split-color', action='store_true', default=True,
                    help='簇内再按颜色切分（相邻积木在深度上粘成一团时用）')
    ap.add_argument('--no-split-color', dest='split_color', action='store_false')
    args = ap.parse_args()
    if args.min_height_mm is not None:
        args.locate_mm = args.min_height_mm
    measure(args)


if __name__ == '__main__':
    main()
