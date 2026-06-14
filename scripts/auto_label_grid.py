#!/usr/bin/env python3
"""
方阵方块自动标注脚本
====================
利用方块已知 3D 位姿 + 相机参数，自动投影生成 YOLO 标注文件。
适用于 block_grid_world.sdf 方阵仿真环境。

原理:
    相机固定在 (0.5, 0, 1.5) 俯瞰桌面（光轴朝下）。
    已知 25 个方块的 (x, y, z)，通过针孔相机模型投影到 2D 像素坐标，
    自动生成 YOLO 格式的 .txt 标注文件。

用法:
    # 先采集图像
    python3 scripts/collect_training_data.py
    # 再自动标注
    python3 scripts/auto_label_grid.py

输出: block_dataset/train/labels/img_XXXX.txt (YOLO 格式)
"""

import os
import math
import json
from pathlib import Path

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
IMG_DIR = PROJECT_ROOT / "block_dataset" / "train" / "images"
LBL_DIR = PROJECT_ROOT / "block_dataset" / "train" / "labels"

# ---- 相机参数（与 block_grid_world.sdf 一致）----
CAMERA = {
    "position": (0.5, 0.0, 1.5),       # 世界坐标
    "width": 640,
    "height": 480,
    "horizontal_fov": 1.047,            # 弧度 ≈ 60°
}

# ---- 方阵方块参数（与 generate_grid_world.py 一致）----
GRID_ROWS = 5
GRID_COLS = 5
BLOCK_SIZE = 0.03          # 边长 3m → 改为 3cm = 0.03m
TABLE_SURFACE_Z = 0.765
BLOCK_Z = TABLE_SURFACE_Z + BLOCK_SIZE / 2.0  # 0.78
X_SPACING = 0.14
Y_SPACING = 0.10
X_START = 0.18
Y_START = -0.20

# 类别 ID（block = 0）
CLASS_ID = 0


def compute_camera_params():
    """根据水平 FOV 和图像尺寸计算内参。"""
    w, h = CAMERA["width"], CAMERA["height"]
    hfov = CAMERA["horizontal_fov"]
    # 焦距（像素单位）
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    # 假设正方形像素
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


def world_to_pixel(wx, wy, wz, fx, fy, cx, cy):
    """
    将世界坐标点投影到像素坐标。

    相机位姿: (0.5, 0, 1.5, 0, π/2, 0)
    光轴指向世界 -Z（正下方）。
    相机坐标系:
        X_cam = world_x - camera_x
        Y_cam = world_y - camera_y
        Z_cam = camera_z - world_z  (深度，正值)
    """
    cam_x, cam_y, cam_z = CAMERA["position"]

    # 世界 → 相机坐标
    Xc = wx - cam_x
    Yc = wy - cam_y
    Zc = cam_z - wz

    if Zc <= 0:
        return None  # 在相机后方

    # 针孔投影
    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy

    return (u, v)


def compute_bbox(wx, wy, wz, block_size, fx, fy, cx, cy):
    """
    计算一个方块在图像中的 2D 边界框 (YOLO 归一化格式)。

    将方块的 8 个顶点投影到图像，取外接矩形。
    """
    half = block_size / 2.0
    # 8 个顶点（世界坐标）
    corners = [
        (wx - half, wy - half, wz - half),
        (wx + half, wy - half, wz - half),
        (wx + half, wy + half, wz - half),
        (wx - half, wy + half, wz - half),
        (wx - half, wy - half, wz + half),
        (wx + half, wy - half, wz + half),
        (wx + half, wy + half, wz + half),
        (wx - half, wy + half, wz + half),
    ]

    pixels = []
    for cx_w, cy_w, cz_w in corners:
        p = world_to_pixel(cx_w, cy_w, cz_w, fx, fy, cx, cy)
        if p is not None:
            pixels.append(p)

    if not pixels:
        return None

    u_min = min(p[0] for p in pixels)
    u_max = max(p[0] for p in pixels)
    v_min = min(p[1] for p in pixels)
    v_max = max(p[1] for p in pixels)

    # 裁剪到图像范围内
    w_img, h_img = CAMERA["width"], CAMERA["height"]
    u_min = max(0.0, u_min)
    u_max = min(w_img - 1.0, u_max)
    v_min = max(0.0, v_min)
    v_max = min(h_img - 1.0, v_max)

    if u_max <= u_min or v_max <= v_min:
        return None

    # YOLO 格式: class x_center y_center width height (归一化)
    x_center = ((u_min + u_max) / 2.0) / w_img
    y_center = ((v_min + v_max) / 2.0) / h_img
    width = (u_max - u_min) / w_img
    height = (v_max - v_min) / h_img

    return (x_center, y_center, width, height)


def generate_labels_for_image(img_name):
    """为单张图像生成 YOLO 标注。"""
    fx, fy, cx, cy = compute_camera_params()

    lines = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            wx = X_START + col * X_SPACING
            wy = Y_START + row * Y_SPACING
            wz = BLOCK_Z

            bbox = compute_bbox(wx, wy, wz, BLOCK_SIZE, fx, fy, cx, cy)
            if bbox is not None:
                xc, yc, w, h = bbox
                lines.append(f"{CLASS_ID} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    return lines


def main():
    os.makedirs(LBL_DIR, exist_ok=True)

    images = sorted(
        [f for f in os.listdir(IMG_DIR) if f.lower().endswith((".jpg", ".png"))]
    )

    if not images:
        print(f"[ERROR] {IMG_DIR} 中没有图像，请先运行 collect_training_data.py 采集数据。")
        return

    print(f"找到 {len(images)} 张图像，开始自动标注...")

    fx, fy, cx, cy = compute_camera_params()
    print(f"相机内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.1f}, cy={cy:.1f}")
    print(f"方阵: {GRID_ROWS}×{GRID_COLS} = {GRID_ROWS * GRID_COLS} 个方块")

    count = 0
    for img_name in images:
        lines = generate_labels_for_image(img_name)
        stem = os.path.splitext(img_name)[0]
        label_path = os.path.join(LBL_DIR, f"{stem}.txt")

        with open(label_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        count += 1
        if count <= 3 or count % 10 == 0:
            print(f"  [{count}/{len(images)}] {img_name} → {len(lines)} 个框")

    print(f"\n[OK] 完成！共标注 {count} 张图像，输出到 {LBL_DIR}")
    print(f"     每张图像应有 {GRID_ROWS * GRID_COLS} 个边界框")


if __name__ == "__main__":
    main()
