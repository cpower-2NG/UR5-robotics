#!/usr/bin/env python3
"""阶段 2 的顶面几何：**像素掩码最小外接矩形** → 平面 yaw。

为什么不在点云上做
──────────────────
`graspnet_candidates.find_blocks` 里的 `yaw_rect` 是纯 numpy 的分位矩形，
在 A3 存档三帧（同一场景、同一相机位姿、时间 12:56/13:35/13:57）上实测：
7 组「同色 + 位置 ≤10 mm」的块里只有 5 组 mod 90 极差 ≤6°（最大 8.5°）。
同一批块改用「顶面像素掩码 → cv2.minAreaRect → 四角反投影到 z=top 平面」
后，7/7 组 ≤6°（最大 4.36°）。

原因：点云在边缘稀疏、还有飞点，分位/凸包都由少数点决定；像素掩码在
1280×720 上每毫米约 2.5 像素，边界稠密且规则，minAreaRect 的角度几乎不抖。

为什么单独一个模块
──────────────────
`graspnet_candidates` 要在 **graspnet venv** 里跑（没有 cv2/scipy），不能 import
本模块；本模块只在开发机的系统 python（cv2 4.5.4）里跑。阶段 2 的 A3 重复性、
主链选块、判分都用这里的结果；`find_blocks` 的 `yaw_rect` 只作无 cv2 时的回退。

接口
────
    frame = graspnet_candidates.load_frame(frame_dir)
    r = block_rect_yaw(frame, block)
    r["yaw_deg"], r["ext_mm"], r["fill"], r["n_pixels"], r["trusted"]
"""

import math
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402


def _unique_pixels(depth_shape, uv):
    mask = np.zeros(depth_shape[:2], dtype=np.uint8)
    uv = np.asarray(uv, dtype=np.int64)
    ok = ((uv[:, 0] >= 0) & (uv[:, 0] < mask.shape[1])
          & (uv[:, 1] >= 0) & (uv[:, 1] < mask.shape[0]))
    mask[uv[ok, 1], uv[ok, 0]] = 255
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def top_face_mask(frame, block, radius_mm=28.0, min_h_mm=15.0):
    """块中心半径内、台面之上 `min_h_mm` 的像素掩码（含顶面与可见侧面）。

    半径 28 mm 的依据：块 36 mm、半对角 25.5 mm；摆件最小中心距 60 mm，
    邻块中心离本块中心至少 60 mm，不会被半径 28 mm 的圆吃进来。
    """
    R, t, K = frame['R_bc'], frame['t_bc'], frame['K']
    P, pix = gc.backproject(frame['depth'], K)
    Pb = P @ R.T + t
    z_top = float(block['top']) / 1000.0
    z_table = z_top - float(block['h']) / 1000.0
    cx, cy = float(block['cx']) / 1000.0, float(block['cy']) / 1000.0
    sel = ((np.hypot(Pb[:, 0] - cx, Pb[:, 1] - cy) < radius_mm / 1000.0)
           & (Pb[:, 2] > z_table + min_h_mm / 1000.0)
           & (Pb[:, 2] < z_top + 0.004))
    if not sel.any():
        return np.zeros(frame['depth'].shape[:2], dtype=np.uint8)
    return _unique_pixels(frame['depth'].shape, pix[sel])


def mask_rect_yaw(mask, K, R, t, z_top_m):
    """像素掩码 → 最大外轮廓的 minAreaRect → 反投影四角到 z=z_top 的平面。

    返回 ``{yaw_deg, ext_mm, fill, n_pixels}``；掩码为空/轮廓太小返回 None。
    ``yaw_deg`` 是**长边**方向，``[0,180)``；判分用 mod 90。
    """
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area_px = float(cv2.contourArea(c))
    if area_px < 50.0:
        return None
    (u, v), (w, h), ang = cv2.minAreaRect(c)
    if w <= 1.0 or h <= 1.0:
        return None
    box = cv2.boxPoints(((u, v), (w, h), ang))

    Kinv = np.linalg.inv(K)
    rays = (Kinv @ np.column_stack([box, np.ones(len(box))]).T).T
    denom = R[2] @ rays.T
    if np.any(np.abs(denom) < 1e-9):
        return None
    s = (z_top_m - t[2]) / denom
    Pb = (R @ (s[:, None] * rays).T).T + t

    sides = []
    for i in range(len(box)):
        d = Pb[(i + 1) % len(box)] - Pb[i]
        sides.append((float(np.linalg.norm(d)) * 1000.0,
                      math.degrees(math.atan2(d[1], d[0])) % 180.0))
    sides.sort(reverse=True)
    yaw = sides[0][1]
    # 相邻两边：排序后第 1 与第 2 长可能是同一条长边的两段，取最短边作另一边。
    ext = (sides[0][0], sides[-1][0])
    return {'yaw_deg': float(yaw),
            'ext_mm': [round(ext[0], 2), round(ext[1], 2)],
            'fill': float(area_px / (w * h)),
            'n_pixels': int(cv2.countNonZero(mask)),
            'rect_px': [round(float(w), 1), round(float(h), 1)],
            'angle_px': round(float(ang), 2)}


def block_rect_yaw(frame, block, radius_mm=28.0, min_h_mm=15.0):
    """单块的像素矩形 yaw；``trusted`` = 轮廓填充率 + 像素数都够。"""
    mask = top_face_mask(frame, block, radius_mm=radius_mm,
                         min_h_mm=min_h_mm)
    z_top = float(block['top']) / 1000.0
    r = mask_rect_yaw(mask, frame['K'], frame['R_bc'], frame['t_bc'], z_top)
    if r is None:
        return {'yaw_deg': None, 'ext_mm': [0.0, 0.0], 'fill': 0.0,
                'n_pixels': 0, 'trusted': False,
                'reason': '掩码/轮廓太小'}
    r['trusted'] = bool(r['fill'] >= 0.70 and r['n_pixels'] >= 200)
    r['reason'] = ''
    return r


def yaw_mod90(deg):
    return float(deg % 90.0)


def ang_span_mod90(vals):
    """一组角度的 mod 90 极差：允许整体平移一个参考角。"""
    v = [float(x) % 90.0 for x in vals if x is not None]
    if not v:
        return None
    best = 90.0
    for ref in v:
        d = [(x - ref) % 90.0 for x in v]
        best = min(best, max(d) - min(d))
    return float(best)
