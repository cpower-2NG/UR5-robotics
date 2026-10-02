#!/usr/bin/env python3
"""**把标签画在图上对一遍** —— 自动版的「人工过复核图」，专门抓「框和方块对不上」。

为什么必须有这一步
==================
几何标注有一个要命的性质：**它不读图**。好处是光照随便变都不影响标签；
坏处是**图拍错了它也不吭声**。`auto_label_grid.py` 只认 `blocks.json` +
外参，你把一批**别的排布**的照片喂给它，它照样吐出一堆框，退出码 0，
复核图上框整整齐齐 —— 只是**每个框都不在方块上**。

2026-10-02 实拍就是这么翻的车：第 6 轮里 A/C 批与 D/E 批是**两个不同排布**
（隔了约 7 分钟，中途重摆过），29 帧挂着错标签。当时是靠肉眼看复核图发现的。
这份脚本把那一眼变成一条命令。

判据
====
1. 在**标签所在区域**（预测框的并集 + 边距）里，按饱和度独立分割出彩色块 ——
   **完全不参考预测**。
2. 每块找最近的**预测框中心**，取这些距离的中位数。
3. 中位数小 → 框咬着方块；大 → 框在别处（半格间距 ≈ 20~40 px，所以阈值取 15 px）。

⚠️ **这是场景相关的检查，不是通用检测器**：分割靠「方块比桌面饱和」这条先验，
换场景（背景也鲜艳、方块是灰的）就不成立。它的定位是**采集流水线上的守门员**，
不是产品代码。

用法::

    python3 scripts/check_labels_on_image.py \\
        --images .scratch/rounds/r06/E/images --pattern 'img_*' \\
        --blocks .scratch/rounds/r06/blocks.json \\
        --camera-info .scratch/rounds/r06/selected/info.json \\
        --extrinsics .scratch/rounds/r00/labeler_extrinsics.json

退出码：0 = 对得上；**1 = 发现有帧对不上**（就是本文开头那个缺陷）；2 = 输入/环境不对。

按批检查（D/E 那种「同一轮里混了两个排布」正是这么查出来的）::

    for b in A C D E; do
      python3 scripts/check_labels_on_image.py --images .scratch/rounds/r06/$b/images ...
    done
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_label_grid as L                                        # noqa: E402

try:
    import cv2
except ImportError:                                                # pragma: no cover
    print("❌ 需要 opencv-python")
    sys.exit(2)


def nearest_distances(points, centres):
    """``points`` 里每个点到 ``centres`` 里最近者的距离。纯函数，自检用。"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    centres = np.asarray(centres, dtype=float).reshape(-1, 2)
    if len(points) == 0 or len(centres) == 0:
        return np.zeros(0)
    d = np.hypot(points[:, None, 0] - centres[None, :, 0],
                 points[:, None, 1] - centres[None, :, 1])
    return d.min(axis=1)


def verdict(distances, tol_px=15.0, min_blobs=3):
    """返回 ``(ok, 说明)``。``distances`` 是「每块到最近预测框」的距离。

    用**中位数**而不是均值：万一有几块被判错了（幽灵 blob、切边），
    中位数不会被它们带跑 —— 而「整体错位」会让**所有**距离一起变大，
    中位数照样抓得住。
    """
    d = np.asarray(distances, dtype=float)
    if len(d) < min_blobs:
        return False, "只分割出 %d 块（<%d），分不出结论 —— 阈值或场景不对" % (len(d), min_blobs)
    m = float(np.median(d))
    return (m <= tol_px,
            "块数 %d，到最近预测框中心距离：中位 %.1f px（阈值 %.1f）"
            % (len(d), m, tol_px))


def predicted_boxes(blocks, info, extr):
    proj = L.TfProjector(info, extr)
    out = []
    for b in blocks:
        cs = L.box_corners(b["x"], b["y"], b["z"], b["size"])
        pix, _ = L.project_pinhole([proj.to_camera(q) for q in cs],
                                   proj.fx, proj.fy, proj.cx, proj.cy)
        p = np.asarray(pix, dtype=float)
        out.append((p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()))
    return np.asarray(out, dtype=float)


def stretch(img):
    """按 1%/99% 分位拉伸亮度 —— 让**暗帧和过曝帧用同一套分割阈值**。

    不加这一步，暗帧分割出来的全是噪声（饱和度整体偏低），
    过曝帧一块都分不出来（饱和度被冲掉），两种都会把结论带偏。
    """
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    lo, hi = np.percentile(g, [1, 99])
    if hi - lo < 1:
        return img
    return np.clip((img.astype(np.float32) - lo) * (230.0 / (hi - lo)),
                   0, 255).astype(np.uint8)


def segment_cubes(img, roi, size_hint):
    """在 ``roi = (x0,y0,x1,y1)`` 里按饱和度分割彩色块；``size_hint`` 是方块的像素边长。"""
    x0, y0, x1, y1 = roi
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(img.shape[1], x1), min(img.shape[0], y1)
    if x1 - x0 < 10 or y1 - y0 < 10:
        return []
    sub = stretch(img[y0:y1, x0:x1])
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    S = hsv[:, :, 1].astype(float)
    V = hsv[:, :, 2].astype(float)
    mask = ((S > 60) & (V > 40)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    lo_a = 0.25 * size_hint * size_hint
    hi_a = 6.0 * size_hint * size_hint
    lo_w = 0.4 * size_hint
    hi_w = 3.0 * size_hint
    out = []
    for j in range(1, n):
        w, h, a = stats[j, 2], stats[j, 3], stats[j, 4]
        if not (lo_a < a < hi_a and lo_w < w < hi_w and lo_w < h < hi_w):
            continue
        out.append((cent[j][0] + x0, cent[j][1] + y0))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser("把标签画在图上对一遍（抓「框和方块对不上」）")
    ap.add_argument("--images", required=True, help="图像目录")
    ap.add_argument("--pattern", default="img_*", help="文件名通配（默认 img_*）")
    ap.add_argument("--blocks", required=True, help="blocks.json")
    ap.add_argument("--camera-info", required=True)
    ap.add_argument("--extrinsics", required=True)
    ap.add_argument("--tol", type=float, default=15.0, help="距离中位数阈值 px（默认 15）")
    ap.add_argument("--sample", type=int, default=6, help="抽查帧数（默认 6）")
    ap.add_argument("--margin", type=int, default=60, help="标签区域外扩 px（默认 60）")
    a = ap.parse_args(argv)

    info = json.load(open(a.camera_info))
    extr = json.load(open(a.extrinsics))
    blocks = json.load(open(a.blocks))
    boxes = predicted_boxes(blocks, info, extr)
    centres = np.column_stack([(boxes[:, 0] + boxes[:, 2]) / 2,
                               (boxes[:, 1] + boxes[:, 3]) / 2])
    roi = (int(boxes[:, 0].min()) - a.margin, int(boxes[:, 1].min()) - a.margin,
           int(boxes[:, 2].max()) + a.margin, int(boxes[:, 3].max()) + a.margin)
    sizes = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    size_hint = float(np.sqrt(np.median(sizes)))

    files = sorted(glob.glob(os.path.join(a.images, a.pattern)))
    files = [f for f in files if os.path.splitext(f)[1].lower()
             in (".png", ".jpg", ".jpeg")]
    if not files:
        print("❌ %s 里没有匹配 %s 的文件" % (a.images, a.pattern))
        return 2
    step = max(1, len(files) // a.sample)
    sample = files[::step][:a.sample]

    print("%s：%d 帧，抽查 %d 帧；方块像素边长提示 %.0f px" %
          (a.images, len(files), len(sample), size_hint))
    bad = 0
    for f in sample:
        img = cv2.imread(f)
        if img is None:
            continue
        blobs = segment_cubes(img, roi, size_hint)
        ok, why = verdict(nearest_distances(blobs, centres), a.tol)
        if not ok:
            bad += 1
        print("  %-6s %s  %s" % ("OK" if ok else "**不符**",
                                 os.path.basename(f), why))
    if bad:
        print("\n❌ %d/%d 帧对不上 —— **排布与 blocks.json 不是同一个**，"
              "或者外参过期。别拿这批去标注。" % (bad, len(sample)))
        return 1
    print("\n✅ 抽查的帧都对得上。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
