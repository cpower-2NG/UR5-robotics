#!/usr/bin/env python3
"""
YOLO 方阵训练 - 端到端演示
==========================
在无 Gazebo 环境下，用合成图像演示完整流水线：
  1. 生成俯瞰方阵合成图像（模拟 RGBD 相机视角）
  2. 自动标注（相机投影）
  3. 划分 train/val
  4. 训练 YOLOv8n
  5. 验证与可视化

用法:
    python3 scripts/demo_pipeline.py
"""

import os
import sys
import math
import random
import shutil
from pathlib import Path

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "block_dataset"
TRAIN_IMG = DATASET_DIR / "train" / "images"
TRAIN_LBL = DATASET_DIR / "train" / "labels"
VAL_IMG = DATASET_DIR / "val" / "images"
VAL_LBL = DATASET_DIR / "val" / "labels"
DATA_YAML = DATASET_DIR / "data.yaml"

# ---- 方阵参数 ----
GRID_ROWS = 5
GRID_COLS = 5
BLOCK_SIZE = 0.03          # 3cm
TABLE_SURFACE_Z = 0.765
BLOCK_Z = TABLE_SURFACE_Z + BLOCK_SIZE / 2.0  # 0.78
X_SPACING = 0.14
Y_SPACING = 0.10
X_START = 0.18
Y_START = -0.20

# 颜色方案 (BGR for OpenCV)
ROW_COLORS_BGR = [
    [(40, 40, 220), (50, 35, 200), (60, 30, 180), (55, 38, 210), (45, 42, 230)],   # 红色系
    [(220, 40, 40), (200, 35, 50), (180, 30, 60), (210, 38, 55), (230, 42, 45)],   # 蓝色系
    [(40, 200, 40), (35, 180, 50), (30, 160, 60), (38, 210, 55), (42, 230, 45)],   # 绿色系
    [(30, 180, 230), (40, 160, 210), (50, 140, 190), (35, 170, 220), (25, 190, 240)],  # 橙色系
    [(200, 50, 180), (180, 45, 160), (160, 40, 140), (190, 48, 170), (210, 52, 190)],  # 紫色系
]
CYLINDER_POSITIONS = {(1, 1), (2, 3), (3, 2)}

# ---- 相机参数 ----
CAM_POS = (0.5, 0.0, 1.5)       # 世界坐标
IMG_W, IMG_H = 640, 480
HFOV = 1.047                      # 弧度 ≈ 60°

# ---- 训练参数 ----
NUM_SAMPLES = 100                  # 生成图像数量
EPOCHS = 50
MODEL_NAME = "yolov8n.pt"


# ============================================================
# 第一步：生成合成图像
# ============================================================
def compute_camera_intrinsics():
    w, h = IMG_W, IMG_H
    fx = (w / 2.0) / math.tan(HFOV / 2.0)
    fy = fx
    cx, cy = w / 2.0, h / 2.0
    return fx, fy, cx, cy


def world_to_pixel(wx, wy, wz, fx, fy, cx, cy):
    """世界坐标 → 像素坐标"""
    cam_x, cam_y, cam_z = CAM_POS
    Xc = wx - cam_x
    Yc = wy - cam_y
    Zc = cam_z - wz
    if Zc <= 0.001:
        return None
    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy
    return (u, v)


def generate_synthetic_image(img_index, fx, fy, cx, cy):
    """生成一张俯瞰方阵合成图像 + YOLO标注"""
    import numpy as np
    import cv2

    # 背景：木色桌面
    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    # 模拟桌面颜色（BGR 木色）
    img[:, :] = (80, 140, 180)

    # 添加一些噪声模拟真实纹理
    noise = np.random.randint(-15, 15, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # 光照渐变（中心亮，边缘暗）
    y_grid, x_grid = np.meshgrid(np.arange(IMG_H), np.arange(IMG_W), indexing='ij')
    dist = np.sqrt((x_grid - cx)**2 + (y_grid - cy)**2)
    vignette = 1.0 - 0.2 * (dist / np.max(dist))
    img = np.clip(img * vignette[:, :, np.newaxis], 0, 255).astype(np.uint8)

    labels = []

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            wx = X_START + col * X_SPACING
            wy = Y_START + row * Y_SPACING
            wz = BLOCK_Z

            # 微调位置（模拟轻微随机偏移）
            wx += random.uniform(-0.005, 0.005)
            wy += random.uniform(-0.005, 0.005)

            # 计算方块的 8 个顶点投影
            half = BLOCK_SIZE / 2.0
            corners_world = [
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
            for cw in corners_world:
                p = world_to_pixel(*cw, fx, fy, cx, cy)
                if p is not None:
                    pixels.append(p)

            if not pixels:
                continue

            u_min = max(0, int(min(p[0] for p in pixels)))
            u_max = min(IMG_W - 1, int(max(p[0] for p in pixels)))
            v_min = max(0, int(min(p[1] for p in pixels)))
            v_max = min(IMG_H - 1, int(max(p[1] for p in pixels)))

            if u_max <= u_min or v_max <= v_min:
                continue

            # 绘制方块
            color = ROW_COLORS_BGR[row][col]
            is_cyl = (row, col) in CYLINDER_POSITIONS

            if is_cyl:
                # 圆柱用椭圆近似
                center = ((u_min + u_max) // 2, (v_min + v_max) // 2)
                axes = ((u_max - u_min) // 2, (v_max - v_min) // 2)
                cv2.ellipse(img, center, axes, 0, 0, 360, color, -1)
                # 高光
                cv2.ellipse(img, center,
                            (max(1, axes[0] // 3), max(1, axes[1] // 4)),
                            0, 0, 360, (255, 255, 255), -1)
            else:
                cv2.rectangle(img, (u_min, v_min), (u_max, v_max), color, -1)
                # 添加边框让方块更清晰
                cv2.rectangle(img, (u_min, v_min), (u_max, v_max),
                              (0, 0, 0), 1)
                # 高光面
                highlight_color = tuple(min(255, int(c * 1.3)) for c in color)
                cv2.rectangle(img, (u_min + 2, v_min + 1),
                              (u_max - 1, v_min + (v_max - v_min) // 3),
                              highlight_color, -1)

            # YOLO 标注：class xc yc w h (归一化)
            xc_norm = ((u_min + u_max) / 2.0) / IMG_W
            yc_norm = ((v_min + v_max) / 2.0) / IMG_H
            w_norm = (u_max - u_min) / IMG_W
            h_norm = (v_max - v_min) / IMG_H
            labels.append(f"0 {xc_norm:.6f} {yc_norm:.6f} {w_norm:.6f} {h_norm:.6f}")

    # 轻微模糊模拟真实相机
    img = cv2.GaussianBlur(img, (3, 3), 0.5)

    return img, labels


# ============================================================
# 主流程
# ============================================================
def step1_generate_dataset():
    """第一步：生成合成图像 + 标注"""
    import cv2

    print("=" * 60)
    print("第一步：生成合成方阵图像")
    print("=" * 60)

    # 清理旧数据
    for d in [TRAIN_IMG, TRAIN_LBL, VAL_IMG, VAL_LBL]:
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    fx, fy, cx, cy = compute_camera_intrinsics()
    print(f"相机内参: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    print(f"生成 {NUM_SAMPLES} 张图像...")

    for i in range(NUM_SAMPLES):
        img, labels = generate_synthetic_image(i, fx, fy, cx, cy)

        # 保存图像
        img_name = f"img_{i:04d}.jpg"
        cv2.imwrite(str(TRAIN_IMG / img_name), img)

        # 保存标注
        label_name = f"img_{i:04d}.txt"
        with open(TRAIN_LBL / label_name, 'w') as f:
            f.write('\n'.join(labels) + '\n')

        if (i + 1) % 20 == 0:
            print(f"  已生成 {i+1}/{NUM_SAMPLES} ...")

    print(f"完成！{NUM_SAMPLES} 张图像 → {TRAIN_IMG}")
    print(f"          {NUM_SAMPLES} 个标注 → {TRAIN_LBL}")


def step2_split_val(val_ratio=0.2):
    """第二步：划分 train/val"""
    print()
    print("=" * 60)
    print("第二步：划分 train/val")
    print("=" * 60)

    images = sorted(TRAIN_IMG.glob("*.jpg"))
    random.shuffle(images)
    n_val = max(1, int(len(images) * val_ratio))
    val_images = images[:n_val]

    for img_path in val_images:
        stem = img_path.stem
        # 移动图像
        shutil.move(str(img_path), str(VAL_IMG / img_path.name))
        # 移动对应标注
        lbl = TRAIN_LBL / f"{stem}.txt"
        if lbl.exists():
            shutil.move(str(lbl), str(VAL_LBL / f"{stem}.txt"))

    print(f"train: {len(list(TRAIN_IMG.glob('*.jpg')))} 张")
    print(f"val:   {len(list(VAL_IMG.glob('*.jpg')))} 张")


def step3_create_yaml():
    """确保 data.yaml 存在"""
    yaml_content = """# YOLOv8 方块检测数据集配置
path: block_dataset
train: train/images
val: val/images

# 类别
names:
  0: block
nc: 1
"""
    with open(DATA_YAML, 'w') as f:
        f.write(yaml_content)
    print(f"data.yaml 已更新 → {DATA_YAML}")


def step4_train():
    """第三步：训练 YOLOv8"""
    print()
    print("=" * 60)
    print("第三步：训练 YOLOv8n")
    print("=" * 60)

    from ultralytics import YOLO

    # 加载预训练模型
    model = YOLO(MODEL_NAME)

    print(f"设备: {'cuda:0' if __import__('torch').cuda.is_available() else 'cpu'}")
    print(f"轮数: {EPOCHS}, 图像尺寸: 640")

    results = model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        imgsz=640,
        batch=16,
        name="block_grid_demo",
        exist_ok=True,
        verbose=True,
    )

    print()
    print("训练完成！")
    return model, results


def step5_validate(model):
    """第四步：验证"""
    print()
    print("=" * 60)
    print("第四步：验证模型")
    print("=" * 60)

    from ultralytics import YOLO

    metrics = model.val(split='val')
    print(f"mAP50: {metrics.box.map50:.4f}")
    print(f"mAP50-95: {metrics.box.map:.4f}")


def step6_predict_sample(model):
    """第五步：预测样本并保存可视化"""
    print()
    print("=" * 60)
    print("第五步：预测可视化")
    print("=" * 60)

    val_imgs = sorted(VAL_IMG.glob("*.jpg"))
    if not val_imgs:
        print("无验证集图像")
        return

    sample = val_imgs[0]
    results = model.predict(
        str(sample),
        save=True,
        project=str(PROJECT_ROOT / "runs" / "block_detect"),
        name="demo_predict",
        exist_ok=True,
    )
    print(f"预测结果已保存 → runs/block_detect/demo_predict/")


def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║     YOLO 方阵训练 - 端到端演示流水线                       ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"环境: {sys.executable}")
    print(f"方阵: {GRID_ROWS}×{GRID_COLS} = {GRID_ROWS * GRID_COLS} 方块")
    print()

    # Step 1: 生成合成数据集
    step1_generate_dataset()

    # Step 2: 划分 train/val
    step2_split_val(val_ratio=0.2)

    # Step 3: 确保 data.yaml
    step3_create_yaml()

    # Step 4: 训练
    model, results = step4_train()

    # Step 5: 验证
    step5_validate(model)

    # Step 6: 预测可视化
    step6_predict_sample(model)

    print()
    print("=" * 60)
    print("🎉 端到端演示完成！")
    print("=" * 60)
    print(f"  训练权重: runs/detect/block_grid_demo/weights/best.pt")
    print(f"  预测结果: runs/block_detect/demo_predict/")
    print()
    print("  在真实 Gazebo 环境中使用时：")
    print(f"    1. ros2 launch yolo_bringup simulation.launch.py world:=block_grid_world.sdf")
    print(f"    2. python3 scripts/collect_training_data.py")
    print(f"    3. python3 scripts/auto_label_grid.py")
    print(f"    4. python3 scripts/train_yolo_block.py")


if __name__ == "__main__":
    main()
