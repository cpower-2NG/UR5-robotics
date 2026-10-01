#!/usr/bin/env python3
"""把 `board_extrinsics.py` 用的那个**靶系**画到图上，好让**人**看清
「哪个物理角点才是 ①(0,0)」。

为什么非要有这一步（**这是一条残差抓不到的错**）
================================================
靶系原点不是纸的角，而是 `findChessboardCorners` **自己排出来的第一个内角点**
再往外退一格。角点顺序是确定的，但**图案本身在 180° 下是同一张图案**
（9×12 格转 180° 还是 9×12 格），所以它挑的是哪一头，**只有看图才知道**。

挑反了的后果，是这条方法里**唯一没有内建护栏**的一种：

    四个外角 + 四个内角这**八个点**，在「绕靶心转 180°」这个映射下
    **两两配对、整体落到自己身上**：
        (0,0)↔(180,240)   (180,0)↔(0,240)
        (20,20)↔(160,220) (160,20)↔(20,220)
    所以整体标反时，刚体配准的**残差照样是 0.0 mm**；
    而且**再加点也没用** —— 新点的靶系坐标是跟着原点一起转的，
    一致性天然成立。这不是解算的几何退化，是**语义**歧义：
    「哪个物理角叫 ①」只能由人来定，数学定不了。

    后果有多大：合成出来的 `base_link ← camera_link` 会绕竖直轴多转 180°，
    相机跑到靶的另一侧去 —— 而**第 ① 步的重投影、第 ② 步的残差、
    第 ③ 步的「相机离台面多高」三项全都照样漂亮**。

⇒ 所以碰点之前**必须**看一张带编号的标注图，并确认它就是当前台面上那张纸。
   靶纸一旦被碰动，**这张图就作废，要重拍重生成**。

用法
====
    # 在板子上（要 ROS 环境；相机得先起着）
    python3 scripts/board_extrinsics.py capture --out-dir /tmp/topcam
    python3 scripts/annotate_target_corners.py \
        --image /tmp/topcam/frame.png --info /tmp/topcam/info.json \
        --out /tmp/topcam/corners.png --square-mm 20 --cols 9 --rows 12

`--cols/--rows` 与 `board_extrinsics.py pnp` **同约定**：传**方格数**（A4 那张 9 和 12）。
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

# 8 个碰点的靶系坐标与画图颜色。①②③④ 是外角（**必须**），⑤⑥⑦⑧ 是内角（可选）。
#   ① = 原点。**它到底是纸上哪个角，由 OpenCV 的角点顺序决定，不由我决定** ——
#   本脚本存在的全部意义就是把那件事变得**可见**。
COL_OUT = (0, 0, 255)       # 外角：红
COL_IN = (255, 0, 0)        # 内角：蓝


def find_corners(gray, cols, rows):
    """找内角点；**两种朝向都试**。

    `findChessboardCorners` 对**非正方**图案**不是旋转不变的**，画面里的靶
    转了 90° 就一个角点都找不到 —— 腕部/俯视相机都可能侧装，实测踩过。
    内角点数只是行列互换，量出来的东西完全一样，所以掉个个儿没有副作用。
    """
    for ps in ((cols - 1, rows - 1), (rows - 1, cols - 1)):
        ok, corners = cv2.findChessboardCorners(
            gray, ps,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            return ps, corners
    return None, None


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--image', required=True, help='capture 出来的 frame.png')
    ap.add_argument('--info', required=True, help='capture 出来的 info.json')
    ap.add_argument('--out', default='/tmp/topcam/corners.png')
    ap.add_argument('--square-mm', type=float, default=20.0, help='方格边长')
    ap.add_argument('--cols', type=int, default=9, help='**方格**数（不是内角点数）')
    ap.add_argument('--rows', type=int, default=12, help='**方格**数')
    ap.add_argument('--zoom', type=int, default=2, help='裁剪后放大倍数')
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit('读不到图：%s' % args.image)
    with open(args.info) as fh:
        json.load(fh)                      # 只为确认它是个合法 json；内参用不上

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ps, corners = find_corners(gray, args.cols, args.rows)
    if ps is None:
        raise SystemExit('没找到内角点（试过 (%d,%d) 与 (%d,%d)）'
                         % (args.cols - 1, args.rows - 1,
                            args.rows - 1, args.cols - 1))
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    c_eff, r_eff = ps[0] + 1, ps[1] + 1    # 方格数
    print('patternSize=%s ⇒ 方格 %d×%d' % (ps, c_eff, r_eff))

    # 用**内角点**求「靶系(mm) → 像素」的单应。内角点 (i,j) 落在
    # ((j+1)·s, (i+1)·s)，所以单应的定义域是**靶系毫米**，原点就是我们要的 ①。
    S = args.square_mm
    P = corners.reshape(r_eff - 1, c_eff - 1, 2)
    obj, imgp = [], []
    for i in range(r_eff - 1):
        for j in range(c_eff - 1):
            obj.append([(j + 1) * S, (i + 1) * S])
            imgp.append(P[i, j])
    H, _ = cv2.findHomography(np.float32(obj), np.float32(imgp), 0)

    def proj(x, y):
        v = H @ np.array([x, y, 1.0])
        return np.array([v[0] / v[2], v[1] / v[2]])

    W, Hh = c_eff * S, r_eff * S
    pts = [(1, 0, 0, COL_OUT), (2, W, 0, COL_OUT),
           (3, W, Hh, COL_OUT), (4, 0, Hh, COL_OUT),
           (5, S, S, COL_IN), (6, W - S, S, COL_IN),
           (7, W - S, Hh - S, COL_IN), (8, S, Hh - S, COL_IN)]

    px = np.array([proj(x, y) for _, x, y, _ in pts])
    x0 = int(max(0, px[:, 0].min() - 70))
    x1 = int(min(img.shape[1], px[:, 0].max() + 70))
    y0 = int(max(0, px[:, 1].min() - 70))
    y1 = int(min(img.shape[0], px[:, 1].max() + 70))
    Z = args.zoom
    crop = cv2.resize(img[y0:y1, x0:x1].copy(), None, fx=Z, fy=Z,
                      interpolation=cv2.INTER_CUBIC)

    def cproj(x, y):
        return (proj(x, y) - np.array([x0, y0])) * Z

    for n, x, y, col in pts:
        p = cproj(x, y)
        cv2.circle(crop, tuple(np.round(p).astype(int)), 14, col, 3)
        cv2.putText(crop, str(n), tuple((np.round(p) + [-7, 7]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3, cv2.LINE_AA)
        q = proj(x, y)
        print('  %d  靶系(%6.1f,%6.1f) → 像素(%7.1f,%7.1f)' % (n, x, y, q[0], q[1]))

    o = cproj(0, 0)
    for dx, dy, col, lab in ((S * 2.5, 0, (0, 200, 0), '+x'),
                             (0, S * 2.5, (0, 200, 255), '+y')):
        e = cproj(dx, dy)
        cv2.arrowedLine(crop, tuple(np.round(o).astype(int)),
                        tuple(np.round(e).astype(int)), col, 4, tipLength=0.3)
        cv2.putText(crop, lab, tuple((np.round(e) + [10, -6]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, col, 3, cv2.LINE_AA)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    cv2.imwrite(args.out, crop)
    print('→ %s  （裁自原图 (%d,%d)-(%d,%d)，放大 %d×）'
          % (args.out, x0, y0, x1, y1, Z))
    print()
    print('⚠️ 拿这张图去**核对台面上那张纸**：① 必须是你等下要用 TCP 尖点去碰的')
    print('   那个物理角。**对不上就别碰** —— 标反 180° 时残差照样是 0.0 mm。')


if __name__ == '__main__':
    sys.exit(main())
