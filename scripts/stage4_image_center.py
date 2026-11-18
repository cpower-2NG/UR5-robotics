#!/usr/bin/env python3
"""块心精度的**图像空间细化**（调研路线②；2026-11-18 用户拍板 2）。

为什么在图像里再量一次
======================
深度链的块心 = **顶面深度点**的方框中心。2026-11-18 S2:5 实测这条路的
顶面点集只有 18–25 mm 见方（真值 29.5）——深度在彩色块面上有洞、且
缺边常是**单侧**的（可见侧面方向），方框中心随之偏 1–3.5 mm（同一帧
六块里五块同向，4× 放大目视可辨：十字偏向可见侧面）。抓贴合对时这点
偏差恰好把一根手指压进 partner 的半空间，是「抓一块带起两块」的直接
来源之一。

彩色图 1280×720 在 survey 位 ≈1.9 px/mm，块顶面四条边在图像里是**高对比
边线**。这里把「已知边长 29.5 mm 的正方形」按候选位姿投影到图像，最大化
四条投影边上的颜色梯度 —— 透视自动正确、边长硬约束、不依赖掩码边界
（掩码只用来给初值）。深度方框中心只当**初值**，结果再与它对照。

做法
====
1. 初值：调用方给 (cx, cy, yaw)（缺省时从掩码最小外接矩形取）；
2. 粗搜 (dx, dy, dθ) 网格 → 围绕最优细搜两级（0.1 mm / 0.02 mm）；
3. 每条投影边取梯度中位（4 条边里取最强的 3 条 —— 贴合对的那条接触边
   没有颜色对比度），向量化一次性算完一批候选；
4. fail-closed：边支撑不足 / 最优解贴在搜索边界 / 位移超过上限 ⇒
   ``ok=False``，只给 ``why``，不给数。

本模块**纯离线**（不 import ROS、不动臂），可单测、可在板上跑回放。
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

BLOCK_SIZE_M = 0.0295
#: 单条边的**中位颜色梯度**下限（3×3 Sobel、三通道平方和）。块顶面边缘
#: 实测 20–300；灰台面/阴影 < 8。低饱和的浅色块（yellowgreen）实测 ~20。
EDGE_MIN = 12.0
#: 取梯度最强的几条边求和（贴合对有一条边无对比度，必须能丢掉）。
SIDE_KEEP = 3
#: 搜索半幅（毫米 / 度）。深度方框中心的实测偏置 ≤3.5 mm，留一倍余量。
SPAN_MM = 5.0
SPAN_DEG = 8.0
#: 结果离初值超过它 ⇒ fail-closed（多半是锁到别的边/别的块上）。
MAX_SHIFT_MM = 6.0
#: 顶面四条边的采样位置（避开角点）。
FRACS = np.linspace(0.12, 0.88, 9)


def _color_gradient(color, sigma=1.2):
    """三通道 Sobel 平方和开方（色相边界也算梯度）。返回 float32 (H,W)。"""
    img = np.asarray(color, np.float32)
    smooth = cv2.GaussianBlur(img, (5, 5), sigma)
    acc = np.zeros(img.shape[:2], np.float32)
    for ch in range(3):
        acc += cv2.Sobel(smooth[:, :, ch], cv2.CV_32F, 1, 0, ksize=3) ** 2
        acc += cv2.Sobel(smooth[:, :, ch], cv2.CV_32F, 0, 1, ksize=3) ** 2
    return np.sqrt(acc)


def _bilinear(field, u, v):
    """双线性采样；u/v 可越界（clamp 到边缘）。"""
    h, w = field.shape
    u = np.clip(u, 0.0, w - 1.001)
    v = np.clip(v, 0.0, h - 1.001)
    u0 = np.floor(u).astype(np.int32)
    v0 = np.floor(v).astype(np.int32)
    fu = u - u0
    fv = v - v0
    return (field[v0, u0] * (1 - fu) * (1 - fv)
            + field[v0, u0 + 1] * fu * (1 - fv)
            + field[v0 + 1, u0] * (1 - fu) * fv
            + field[v0 + 1, u0 + 1] * fu * fv)


def _side_samples(size_m):
    """顶面正方形四条边的采样点（块局部坐标，米）。返回 (M,2) + 边号。"""
    h = size_m / 2.0
    f = FRACS * size_m - h
    side = np.repeat(np.arange(4), len(f))
    loc = np.concatenate([
        np.stack([np.full_like(f, -h), f], -1),      # x = −h
        np.stack([np.full_like(f, h), f], -1),       # x = +h
        np.stack([f, np.full_like(f, -h)], -1),      # y = −h
        np.stack([f, np.full_like(f, h)], -1),       # y = +h
    ], 0)
    return loc, side


def _score(color_grad, cam, poses, top_z, loc, side):
    """一批位姿 ``(N,3) = (cx_mm, cy_mm, theta_rad)`` → 每条边的中位梯度。

    返回 ``(N, 4)`` 数组；投影无效（相机背后/出画）的位姿该边记 0。
    """
    K, R, t = cam["K"], cam["R_bc"], cam["t_bc"]
    n = len(poses)
    m = len(loc)
    cx = poses[:, 0:1] / 1000.0
    cy = poses[:, 1:2] / 1000.0
    c = np.cos(poses[:, 2:3])
    s = np.sin(poses[:, 2:3])
    lx = loc[None, :, 0]
    ly = loc[None, :, 1]
    x = cx + c * lx - s * ly
    y = cy + s * lx + c * ly
    p = np.stack([x, y, np.full_like(x, float(top_z))], -1)      # (N,M,3)
    pc = (p - t) @ np.asarray(R, float)                            # Rᵀ(p−t)
    uv = (pc @ np.asarray(K, float).T)[..., :2] / pc[..., 2:3]
    u, v = uv[..., 0], uv[..., 1]
    h, w = color_grad.shape
    valid = (pc[..., 2] > 1e-6) & (u > -2) & (u < w + 2) \
        & (v > -2) & (v < h + 2)
    g = _bilinear(color_grad, u, v)
    g = np.where(valid, g, 0.0).reshape(n, 4, len(FRACS))
    return np.median(g, axis=2)


def _pose_score(sides):
    """强边求和：四条边都够强就用四条；贴合对有一条接触边没对比度，
    自动降到三条（少于三条强边 ⇒ 该位姿无效）。"""
    strong = sides >= EDGE_MIN
    n = strong.sum(axis=1)
    total = np.where(strong, sides, 0.0).sum(axis=1)
    return np.where(n >= SIDE_KEEP, total, -1.0)


def _best_in_grid(grad, cam, top_z, cx, cy, th, steps, loc, side):
    """在给定网格上求最优；返回 (score, sides, pose)。"""
    dx = np.arange(-steps[0], steps[0] + 1e-9, steps[3])
    dy = np.arange(-steps[1], steps[1] + 1e-9, steps[3])
    dth = np.arange(-steps[2], steps[2] + 1e-9, steps[4])
    gx, gy, gt = np.meshgrid(dx, dy, dth, indexing="ij")
    poses = np.stack([cx + gx.ravel(), cy + gy.ravel(),
                      th + np.radians(gt.ravel())], -1)
    sides = _score(grad, cam, poses, top_z, loc, side)
    score = _pose_score(sides)
    k = int(np.argmax(score))
    return float(score[k]), sides[k], poses[k]


def _mask_init(mask_uv, K, R_bc, t_bc, top_z):
    """掩码像素 ``(N,2)`` → 最小外接矩形初值 ``(cx_mm, cy_mm, yaw_deg)``。

    像素中心按 ``top_z`` 平面反投影回 base（米→毫米）；**图像 yaw 与 base
    yaw 反号**（base +y 在图像里是 −v），这里取负并归一到 [0, 90)。
    """
    from stage4_truth_from_frame import ray_plane_xy
    uv = np.asarray(mask_uv, np.float32)
    (rcx, rcy), (rw, rh), ang = cv2.minAreaRect(uv.reshape(-1, 1, 2))
    xy = ray_plane_xy((rcx, rcy), K, R_bc, t_bc, top_z)
    if xy is None:
        return None
    yaw = ang if rw >= rh else ang + 90.0
    return float(xy[0]) * 1000.0, float(xy[1]) * 1000.0, \
        float((-yaw) % 90.0)


def refine_center(mask_uv, color, K, R_bc, t_bc, top_z,
                  block_size_m=BLOCK_SIZE_M, cx0_mm=None, cy0_mm=None,
                  yaw0_deg=None, span_mm=SPAN_MM, span_deg=SPAN_DEG,
                  grad=None):
    """图像空间块心：把已知边长的正方形边缘对齐到颜色梯度极值。

    ``top_z``：块**顶面**离台面的高度（米）。``cx0_mm/cy0_mm/yaw0_deg``
    缺省时从 ``mask_uv`` 的最小外接矩形取（掩码只提供初值，不参与拟合）。

    返回::

        {ok, x_mm, y_mm, yaw_deg, center_uv, corners_uv, corners_xy_mm,
         size_mm, support, score, shift_mm, init, why}

    ``ok=False`` 时只给 ``why``（边支撑不足/贴搜索边界/位移超限/初值缺失）。
    """
    if cx0_mm is None or cy0_mm is None or yaw0_deg is None:
        if mask_uv is None or len(mask_uv) < 16:
            return {"ok": False, "why": "no init and mask points <16"}
        init = _mask_init(mask_uv, K, R_bc, t_bc, top_z)
        if init is None:
            return {"ok": False, "why": "mask rect center misses plane"}
        mx, my, myaw = init
        cx0_mm = mx if cx0_mm is None else cx0_mm
        cy0_mm = my if cy0_mm is None else cy0_mm
        yaw0_deg = myaw if yaw0_deg is None else yaw0_deg
    cam = {"K": np.asarray(K, float), "R_bc": np.asarray(R_bc, float),
           "t_bc": np.asarray(t_bc, float)}
    if grad is None:
        grad = _color_gradient(color)
    loc, side = _side_samples(float(block_size_m))
    th0 = math.radians(float(yaw0_deg))
    size_mm = float(block_size_m) * 1000.0

    # 粗 → 细 → 最细（0.5 mm / 1° → 0.1 / 0.25° → 0.02 / 0.05°）
    sc, sides, pose = _best_in_grid(grad, cam, top_z, cx0_mm, cy0_mm, th0,
                                    (span_mm, span_mm, span_deg, 0.5, 1.0),
                                    loc, side)
    sc2, sides2, pose2 = _best_in_grid(
        grad, cam, top_z, *pose, (0.5, 0.5, 1.0, 0.1, 0.25), loc, side)
    if sc2 >= sc:
        sc, sides, pose = sc2, sides2, pose2
    sc3, sides3, pose3 = _best_in_grid(
        grad, cam, top_z, *pose, (0.1, 0.1, 0.25, 0.02, 0.05), loc, side)
    if sc3 >= sc:
        sc, sides, pose = sc3, sides3, pose3
    cx, cy, th = float(pose[0]), float(pose[1]), float(pose[2])
    shift = math.hypot(cx - cx0_mm, cy - cy0_mm)
    out = {
        "ok": False, "score": round(float(sc), 1),
        "support": [round(float(v), 1) for v in np.sort(sides)[::-1]],
        "init": [round(float(cx0_mm), 2), round(float(cy0_mm), 2),
                 round(float(yaw0_deg), 2)],
        "shift_mm": round(shift, 2),
        "size_mm": round(size_mm, 2),
        "x_mm": round(cx, 3), "y_mm": round(cy, 3),
        "yaw_deg": round(math.degrees(th) % 90.0, 2),
        "why": "",
    }
    if shift > MAX_SHIFT_MM:
        out["why"] = "shift %.2f mm > %.1f（初值不可信或锁错边）" % (
            shift, MAX_SHIFT_MM)
        return out
    if abs(cx - cx0_mm) >= span_mm - 0.21 or abs(cy - cy0_mm) >= span_mm - 0.21:
        out["why"] = "best pose on search boundary（真解可能在窗外）"
        return out
    strong = int((np.asarray(sides) >= EDGE_MIN).sum()) if sides is not None \
        else 0
    out["strong_sides"] = strong
    if strong < SIDE_KEEP:
        out["why"] = "edge support: only %d sides ≥ %.0f" % (strong, EDGE_MIN)
        return out

    # 位姿 → 四角 / 中心像素（仅用于叠加图与报告）
    c, s = math.cos(th), math.sin(th)
    half = size_mm / 2.0
    corners_xy = [(cx + sx * half * c - sy * half * s,
                   cy + sx * half * s + sy * half * c)
                  for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    corners_uv = []
    for x, y in corners_xy:
        p = np.array([x / 1000.0, y / 1000.0, float(top_z)])
        pc = np.asarray(R_bc, float).T @ (p - np.asarray(t_bc, float))
        uv = np.asarray(K, float) @ pc
        corners_uv.append([round(float(uv[0] / uv[2]), 2),
                           round(float(uv[1] / uv[2]), 2)])
    out.update({
        "ok": True,
        "corners_uv": corners_uv,
        "corners_xy_mm": [[round(x, 2), round(y, 2)] for x, y in corners_xy],
        "center_uv": [round(float(np.mean([p[0] for p in corners_uv])), 2),
                      round(float(np.mean([p[1] for p in corners_uv])), 2)],
    })
    return out


def main(argv=None):
    """板端回放入口：把一段 burst 的每个 SAM 片段各量一次图像块心。

    只读：不碰臂、不写夹爪。需要板上 SAM（``sam_blocks.load_predictor``）。
    用法::

        ~/precision/run.sh ~/stage4-entry/scripts/stage4_image_center.py \
            --burst <burst 目录> --frame 5 [--block-size 0.0295]
    """
    import argparse
    import json
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--burst", required=True)
    ap.add_argument("--frame", type=int, default=5)
    ap.add_argument("--block-size", type=float, default=BLOCK_SIZE_M)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    import graspnet_candidates as gc
    import sam_blocks as sb
    fr = gc.load_frame(args.burst, tip=gc.TIP, chain=gc.CHAIN, idx=args.frame)
    pred = sb.load_predictor()
    grad = _color_gradient(fr["color"])
    rows, _ = sb.find_blocks_sam(fr, pred, return_masks=True)
    out = []
    for r in rows:
        if r.get("low") or "mask_uv" not in r:
            continue
        # ⚠️ ``find_blocks_sam`` 行的 ``top`` 是 **mm**，本模块要 **m**。
        ref = refine_center(r["mask_uv"], fr["color"], fr["K"], fr["R_bc"],
                            fr["t_bc"], float(r["top"]) / 1000.0,
                            block_size_m=args.block_size,
                            cx0_mm=float(r["cx"]), cy0_mm=float(r["cy"]),
                            yaw0_deg=float(r.get("yaw_rect") or r.get("yaw")),
                            grad=grad)
        ref["class"] = r.get("class")
        ref["depth_x_mm"] = r.get("cx")
        ref["depth_y_mm"] = r.get("cy")
        if ref.get("ok"):
            ref["dxy_vs_depth_mm"] = [round(ref["x_mm"] - r["cx"], 2),
                                      round(ref["y_mm"] - r["cy"], 2)]
        out.append(ref)
    doc = {"burst": args.burst, "frame": args.frame, "rows": out}
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1)
    print(json.dumps(doc, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
