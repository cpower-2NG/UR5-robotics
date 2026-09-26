#!/usr/bin/env python3
"""
批量合成训练数据 + 自动标注
============================
循环多个 seed，每个 seed 生成若干张合成图像（OpenCV 渲染），
同时利用 JSON 元数据自动投影生成 YOLO 标注。

用法:
    python3 scripts/generate_synthetic_data.py

输出: block_dataset/train/images/ + labels/ + data.yaml
"""

import os
import sys
import json
import math
import random
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "block_dataset"
TRAIN_IMG = DATASET_DIR / "train" / "images"
TRAIN_LBL = DATASET_DIR / "train" / "labels"
VAL_IMG = DATASET_DIR / "val" / "images"
VAL_LBL = DATASET_DIR / "val" / "labels"
META_PATH = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "worlds" / "block_grid_world_meta.json"

# ---- 图像参数 ----
IMG_W, IMG_H = 640, 480
TABLE_W, TABLE_H = 0.8, 0.6        # 桌面尺寸 (m)
TABLE_X, TABLE_Y = 0.5, 0.0        # 桌面中心世界坐标
CAM_X, CAM_Y, CAM_Z = 0.5, 0.0, 1.5
BLOCK_SIZE = 0.04                  # 方块边长 (m)
BLOCK_Z = 0.785                    # 方块中心 Z

SEEDS = range(1, 41)               # 40 个不同场景
IMAGES_PER_SEED = 5                # 每场景 5 张
VAL_SPLIT = 0.2                    # 20% 验证集


def clamp(v, lo=0.0, hi=1.0):
    return max(lo, min(hi, v))


def compute_intrinsics():
    hfov = 1.047
    fx = (IMG_W / 2.0) / math.tan(hfov / 2.0)
    fy = fx
    cx = IMG_W / 2.0
    cy = IMG_H / 2.0
    return fx, fy, cx, cy


def world_to_pixel(wx, wy, wz, fx, fy, cx, cy):
    Xc = wx - CAM_X
    Yc = wy - CAM_Y
    Zc = CAM_Z - wz
    if Zc <= 0:
        return None
    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy
    return (u, v)


def compute_bbox(block, fx, fy, cx, cy):
    wx, wy, wz = block["x"], block["y"], block["z"]
    half = BLOCK_SIZE / 2.0
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
    pixels = [world_to_pixel(cx_w, cy_w, cz_w, fx, fy, cx, cy)
              for cx_w, cy_w, cz_w in corners]
    pixels = [p for p in pixels if p is not None]
    if not pixels:
        return None
    u_min = max(0, min(p[0] for p in pixels))
    u_max = min(IMG_W - 1, max(p[0] for p in pixels))
    v_min = max(0, min(p[1] for p in pixels))
    v_max = min(IMG_H - 1, max(p[1] for p in pixels))
    if u_max <= u_min or v_max <= v_min:
        return None
    x_center = ((u_min + u_max) / 2.0) / IMG_W
    y_center = ((v_min + v_max) / 2.0) / IMG_H
    width = (u_max - u_min) / IMG_W
    height = (v_max - v_min) / IMG_H
    return (x_center, y_center, width, height)


def render_scene(blocks, noise_level=0.03, brightness=1.0, blur=0):
    """用 OpenCV 渲染俯瞰桌面场景。"""
    img = np.ones((IMG_H, IMG_W, 3), dtype=np.float32) * 0.75  # 浅灰背景

    # 桌面
    fx, fy, cx, cy = compute_intrinsics()
    table_corners = [
        (TABLE_X - TABLE_W/2, TABLE_Y - TABLE_H/2, 0.765),
        (TABLE_X + TABLE_W/2, TABLE_Y - TABLE_H/2, 0.765),
        (TABLE_X + TABLE_W/2, TABLE_Y + TABLE_H/2, 0.765),
        (TABLE_X - TABLE_W/2, TABLE_Y + TABLE_H/2, 0.765),
    ]
    pts = [world_to_pixel(wx, wy, wz, fx, fy, cx, cy) for wx, wy, wz in table_corners]
    pts = [p for p in pts if p is not None]
    if len(pts) >= 4:
        pts_np = np.array(pts, dtype=np.int32)
        cv2.fillPoly(img, [pts_np], (0.45, 0.28, 0.12))  # 木色桌面

    # 方块
    fx, fy, cx, cy = compute_intrinsics()
    for b in blocks:
        wx, wy, wz = b["x"], b["y"], b["z"]
        r, g, bl = b["color"]
        half = BLOCK_SIZE / 2.0

        # 方块顶面四个角
        top_corners = [
            (wx - half, wy - half, wz + half),
            (wx + half, wy - half, wz + half),
            (wx + half, wy + half, wz + half),
            (wx - half, wy + half, wz + half),
        ]
        pts = [world_to_pixel(cx_w, cy_w, cz_w, fx, fy, cx, cy)
               for cx_w, cy_w, cz_w in top_corners]
        pts = [p for p in pts if p is not None]
        if len(pts) >= 4:
            pts_np = np.array(pts, dtype=np.int32)
            cv2.fillPoly(img, [pts_np], (bl, g, r))  # BGR
            # 加边线
            cv2.polylines(img, [pts_np], True, (0.1, 0.1, 0.1), 1)

    # 加噪声
    if noise_level > 0:
        noise = np.random.randn(*img.shape).astype(np.float32) * noise_level
        img = np.clip(img + noise, 0, 1)

    # 亮度变化
    img = np.clip(img * brightness, 0, 1)

    # 高斯模糊
    if blur > 0:
        img = (img * 255).astype(np.uint8)
        img = cv2.GaussianBlur(img, (blur, blur), 0)
        img = img.astype(np.float32) / 255.0

    img = (img * 255).astype(np.uint8)
    return img


def generate_one_scene(seed):
    """生成一个 seed 对应的场景，返回 blocks 列表。"""
    subprocess.run(
        [sys.executable, str(PROJECT_ROOT / "scripts" / "generate_grid_world.py"),
         "--seed", str(seed)],
        capture_output=True, cwd=str(PROJECT_ROOT)
    )
    with open(META_PATH) as f:
        meta = json.load(f)
    return meta["blocks"]


def main():
    # 清空旧数据
    for d in [TRAIN_IMG, TRAIN_LBL, VAL_IMG, VAL_LBL]:
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    all_samples = []  # (img_name, is_val)

    img_idx = 0
    for seed in SEEDS:
        print(f"[seed={seed}] 生成场景...", end=" ", flush=True)
        blocks = generate_one_scene(seed)

        fx, fy, cx, cy = compute_intrinsics()

        for k in range(IMAGES_PER_SEED):
            # 随机扰动参数（模拟采集变化）
            noise = random.uniform(0.01, 0.05)
            brightness = random.uniform(0.85, 1.15)
            blur = random.choice([0, 0, 0, 1, 3])  # 大部分清晰，偶尔模糊

            img = render_scene(blocks, noise_level=noise,
                               brightness=brightness, blur=blur)

            img_name = f"img_{img_idx:04d}.jpg"
            cv2.imwrite(str(TRAIN_IMG / img_name), img)

            # 自动标注
            label_lines = []
            for b in blocks:
                bbox = compute_bbox(b, fx, fy, cx, cy)
                if bbox is not None:
                    xc, yc, w, h = bbox
                    label_lines.append(f"0 {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

            lbl_name = f"img_{img_idx:04d}.txt"
            with open(TRAIN_LBL / lbl_name, 'w') as f:
                f.write('\n'.join(label_lines) + '\n')

            all_samples.append(img_name)
            img_idx += 1

        print(f"完成 ({img_idx} 张)")

    # 划分 train/val
    random.seed(42)
    random.shuffle(all_samples)
    n_val = int(len(all_samples) * VAL_SPLIT)
    val_samples = set(all_samples[:n_val])

    for img_name in all_samples:
        stem = os.path.splitext(img_name)[0]
        if img_name in val_samples:
            shutil.move(str(TRAIN_IMG / img_name), str(VAL_IMG / img_name))
            shutil.move(str(TRAIN_LBL / f"{stem}.txt"), str(VAL_LBL / f"{stem}.txt"))

    # 写 data.yaml
    yaml_content = f"""path: {DATASET_DIR}
train: train/images
val: val/images
names:
  0: block
nc: 1
"""
    with open(DATASET_DIR / "data.yaml", 'w') as f:
        f.write(yaml_content)

    n_train = len(list(TRAIN_IMG.iterdir()))
    n_val = len(list(VAL_IMG.iterdir()))
    print(f"\n[完成] train={n_train} 张, val={n_val} 张, 总计 {img_idx} 张")
    print(f"       输出: {DATASET_DIR}")


if __name__ == "__main__":
    main()
