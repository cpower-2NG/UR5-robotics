#!/usr/bin/env python3
"""
YOLOv8 方块检测训练脚本
=======================
微调 YOLOv8n 检测 Gazebo 中的立方体方块。

用法:
    python3 scripts/train_yolo_block.py

前提:
    1. 已采集并标注好数据集
    2. 数据集结构:
       block_dataset/
       ├── data.yaml
       ├── train/
       │   ├── images/   ← 采集的 jpg
       │   └── labels/   ← 标注的 txt (labelImg 生成)
       └── val/
           ├── images/   ← 从 train 复制 20%
           └── labels/   ← 对应标注

训练参数:
    model:   yolov8n.pt (预训练权重，微调)
    epochs:  100
    imgsz:   640
    device:  cuda:0 (自动回退 cpu)
"""

import os
import sys
import shutil
import random
from pathlib import Path

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "block_dataset"
TRAIN_IMG = DATASET_DIR / "train" / "images"
TRAIN_LBL = DATASET_DIR / "train" / "labels"
VAL_IMG = DATASET_DIR / "val" / "images"
VAL_LBL = DATASET_DIR / "val" / "labels"
DATA_YAML = DATASET_DIR / "data.yaml"


def split_val(val_ratio=0.2):
    """从 train 中随机抽取 val_ratio 比例作为验证集。"""
    images = sorted(TRAIN_IMG.glob("*.jpg")) + sorted(TRAIN_IMG.glob("*.png"))
    if not images:
        print("[ERROR] train/images/ 中没有图像，请先采集数据。")
        sys.exit(1)

    if len(images) < 5:
        print(f"[INFO] 仅 {len(images)} 张，不足以拆分验证集，全部用于训练。")
        return

    # 清空 val
    for f in VAL_IMG.glob("*"):
        f.unlink()
    for f in VAL_LBL.glob("*"):
        f.unlink()

    random.seed(42)
    val_count = max(1, int(len(images) * val_ratio))
    val_images = random.sample(images, val_count)

    for img_path in val_images:
        shutil.move(str(img_path), str(VAL_IMG / img_path.name))
        label_path = TRAIN_LBL / (img_path.stem + ".txt")
        if label_path.exists():
            shutil.move(str(label_path), str(VAL_LBL / label_path.name))

    print(f"数据拆分: train={len(list(TRAIN_IMG.glob('*.jpg')))} 张, "
          f"val={len(val_images)} 张")


def check_labels():
    """检查标注文件是否就绪。"""
    images = list(TRAIN_IMG.glob("*.jpg")) + list(TRAIN_IMG.glob("*.png"))
    labeled = 0
    for img in images:
        lbl = TRAIN_LBL / (img.stem + ".txt")
        if lbl.exists():
            labeled += 1
    print(f"标注进度: {labeled}/{len(images)} 张已标注")
    if labeled == 0:
        print("[WARN] 尚未标注任何图像，请先运行 labelImg 标注。")
        print("       安装: pip install labelImg")
        print(f"       打开: labelImg {TRAIN_IMG}")
        return False
    if labeled < len(images) * 0.5:
        print("[WARN] 标注不足 50%，建议标注更多后再训练。")
    return True


def train():
    """训练 YOLOv8n。"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] 请安装 ultralytics: pip install ultralytics")
        sys.exit(1)

    device = "cuda:0"
    try:
        import torch
        if not torch.cuda.is_available():
            device = "cpu"
            print("[INFO] CUDA 不可用，使用 CPU 训练（会慢很多）。")
    except ImportError:
        device = "cpu"

    print(f"\n加载预训练模型 yolov8n.pt，设备: {device}")
    model = YOLO("yolov8n.pt")

    print(f"开始训练...")
    results = model.train(
        data=str(DATA_YAML),
        epochs=100,
        imgsz=640,
        batch=16,
        device=device,
        patience=20,          # 20 个 epoch 无提升则早停
        save=True,
        save_period=10,       # 每 10 epoch 保存一次
        project=str(PROJECT_ROOT / "runs"),
        name="block_detect",
        exist_ok=True,
        pretrained=True,
        optimizer="auto",
        verbose=True,
    )

    # 复制最佳模型到项目根目录
    best_src = PROJECT_ROOT / "runs" / "block_detect" / "weights" / "best.pt"
    best_dst = PROJECT_ROOT / "block_best.pt"
    if best_src.exists():
        shutil.copy(str(best_src), str(best_dst))
        print(f"\n✅ 训练完成！最佳模型: {best_dst}")
        print(f"   验证指标: {results.results_dict}")
    else:
        print(f"\n⚠ 训练完成但未找到 best.pt，请检查日志。")

    # 替换 yolo_detector 使用的模型
    detector_model = PROJECT_ROOT / "yolov8n.pt"
    if detector_model.exists():
        backup = PROJECT_ROOT / "yolov8n_backup.pt"
        if not backup.exists():
            shutil.copy(str(detector_model), str(backup))
            print(f"   已备份原模型: {backup}")
        shutil.copy(str(best_dst), str(detector_model))
        print(f"   已替换 yolo_detector 模型: {detector_model}")


if __name__ == "__main__":
    print("=" * 60)
    print("  YOLOv8 方块检测训练")
    print("=" * 60)

    if not check_labels():
        print("\n请先完成标注后再运行训练。")
        sys.exit(0)

    print("\n拆分为 train/val ...")
    split_val()

    train()
