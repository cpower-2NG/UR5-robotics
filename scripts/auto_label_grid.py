#!/usr/bin/env python3
"""
方块自动标注脚本
================
读取 block_grid_world_meta.json（由 generate_grid_world.py 生成），
利用方块已知 3D 位姿 + 针孔相机模型，自动投影生成 YOLO 标注文件。

用法:
    # 先生成场景，再采集图像，最后标注
    python3 scripts/generate_grid_world.py
    python3 scripts/collect_training_data.py
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
META_PATHS = [
    PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "worlds" / "block_grid_world_meta.json",
]

CLASS_ID = 0  # block 类别 ID


def load_metadata():
    """从 JSON 元数据加载相机参数和方块列表。"""
    for p in META_PATHS:
        if p.exists():
            with open(p) as f:
                meta = json.load(f)
            print(f"[INFO] 读取元数据: {p}")
            print(f"       seed={meta['seed']}, blocks={meta['num_blocks']}, "
                  f"fixed_mode={meta.get('fixed_mode', False)}")
            return meta

    raise FileNotFoundError(
        f"未找到 block_grid_world_meta.json。\n"
        f"请先运行 generate_grid_world.py 生成场景和元数据。"
    )


def compute_camera_params(meta):
    """根据水平 FOV 和图像尺寸计算内参。"""
    cam = meta["camera"]
    w, h = cam["width"], cam["height"]
    hfov = cam["horizontal_fov"]
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    fy = fx          # 正方形像素假设
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


def world_to_pixel(wx, wy, wz, cam_x, cam_y, cam_z, fx, fy, cx, cy):
    """将世界坐标点投影到像素坐标。"""
    Xc = wx - cam_x
    Yc = wy - cam_y
    Zc = cam_z - wz  # 深度（正值）

    if Zc <= 0:
        return None

    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy
    return (u, v)


def compute_bbox(block, block_size, cam, fx, fy, cx, cy):
    """计算单个方块在图像中的 2D 边界框（YOLO 归一化格式）。"""
    wx, wy, wz = block["x"], block["y"], block["z"]
    cam_x, cam_y, cam_z = cam["position"]
    half = block_size / 2.0

    # 8 个顶点
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
        p = world_to_pixel(cx_w, cy_w, cz_w, cam_x, cam_y, cam_z, fx, fy, cx, cy)
        if p is not None:
            pixels.append(p)

    if not pixels:
        return None

    u_min = min(p[0] for p in pixels)
    u_max = max(p[0] for p in pixels)
    v_min = min(p[1] for p in pixels)
    v_max = max(p[1] for p in pixels)

    w_img, h_img = cam["width"], cam["height"]
    u_min = max(0.0, u_min)
    u_max = min(w_img - 1.0, u_max)
    v_min = max(0.0, v_min)
    v_max = min(h_img - 1.0, v_max)

    if u_max <= u_min or v_max <= v_min:
        return None

    x_center = ((u_min + u_max) / 2.0) / w_img
    y_center = ((v_min + v_max) / 2.0) / h_img
    width = (u_max - u_min) / w_img
    height = (v_max - v_min) / h_img
    return (x_center, y_center, width, height)


def generate_labels_for_image(meta, fx, fy, cx, cy):
    """为单张图像生成 YOLO 标注（所有方块）。"""
    cam = meta["camera"]
    lines = []

    for block in meta["blocks"]:
        bbox = compute_bbox(block, BLOCK_SIZE, cam, fx, fy, cx, cy)
        if bbox is not None:
            xc, yc, w, h = bbox
            lines.append(f"{CLASS_ID} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    return lines


def main():
    # 加载元数据
    meta = load_metadata()

    # 方块边长（当前固定 4cm；可由 meta 扩展）
    global BLOCK_SIZE
    BLOCK_SIZE = 0.04

    fx, fy, cx, cy = compute_camera_params(meta)
    cam = meta["camera"]
    print(f"相机内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.1f}, cy={cy:.1f}")
    print(f"方块数: {meta['num_blocks']}")

    os.makedirs(LBL_DIR, exist_ok=True)

    images = sorted(
        [f for f in os.listdir(IMG_DIR) if f.lower().endswith((".jpg", ".png"))]
    )

    if not images:
        print(f"[ERROR] {IMG_DIR} 中没有图像，请先运行 collect_training_data.py 采集数据。")
        return

    print(f"找到 {len(images)} 张图像，开始自动标注...")

    count = 0
    for img_name in images:
        lines = generate_labels_for_image(meta, fx, fy, cx, cy)
        stem = os.path.splitext(img_name)[0]
        label_path = os.path.join(LBL_DIR, f"{stem}.txt")

        with open(label_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        count += 1
        if count <= 3 or count % 10 == 0:
            print(f"  [{count}/{len(images)}] {img_name} → {len(lines)} 个框")

    print(f"\n[OK] 完成！共标注 {count} 张图像，输出到 {LBL_DIR}")
    print(f"     每张图像应有 {meta['num_blocks']} 个边界框")


if __name__ == "__main__":
    main()
