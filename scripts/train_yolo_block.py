#!/usr/bin/env python3
"""
YOLOv8 方块检测训练脚本
=======================
微调 YOLOv8n 检测**真机**俯视相机里的方块。

用法:
    python3 scripts/train_yolo_block.py [--epochs N] [--batch N] [--imgsz N]

先装配数据集（本脚本**不做切分**，见下）::

    python3 scripts/build_dataset.py --out block_dataset \\
        --val r02,r06a2 --jpeg-quality 92

数据集结构（block_dataset/）::

    ├── data.yaml
    ├── train/{images,labels}   ← 156 帧 / 3345 框，5 个排布
    ├── val/{images,labels}     ←  54 帧 / 1350 框，2 个排布
    └── dataset_manifest.json   ← 哪一帧来自哪个排布、来自哪个源文件

⚠️ **切分由 `build_dataset.py` 按「排布」做，本脚本不再切。**
原版这里有个 `split_val()`，随机抽 20% 的**帧**当验证集 —— 对这份数据是假的：
同一个排布下拍的几十帧**布局完全相同**，模型只要背下那 25 个坐标就能在 val 上
刷满分。val 于是衡量的只是「记住没有」，不是「换个摆法还认不认」，而部署时方块
是**当场摆的**。所以那段代码已删除，`check_dataset()` 反过来**断言** train 与
val 的排布不重叠 —— 谁要是手工把同一排布的两帧分别放进两边，这里会拦下来。

训练参数（默认）:
    model:   yolov8n_coco.pt (COCO 预训练权重，微调)
    epochs:  100, patience 20
    imgsz:   640
    device:  cuda:0 (自动回退 cpu)

输出: runs/block_detect/weights/best.pt → 复制为仓库根 block_best.pt
      （yolo_detector 的默认 model 参数就指向它）
"""

import argparse
import sys
import shutil
from pathlib import Path

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "block_dataset"

# 微调起点：**真 COCO yolov8n**（原名为 yolov8n_backup.pt，已改名以免混淆）。
# 注意：旧版此处写的是 "yolov8n.pt"，而仓库根那个文件其实是上一次训练产出的
# 方块模型——于是「微调」实际上是在**上一代方块模型**上继续训，而不是从 COCO
# 起。行为随 CWD 变化，且文件名与内容不符（详见 docs/RESTRUCTURE_PLAN.md D6）。
BASE_WEIGHTS = PROJECT_ROOT / "yolov8n_coco.pt"
TRAIN_IMG = DATASET_DIR / "train" / "images"
TRAIN_LBL = DATASET_DIR / "train" / "labels"
VAL_IMG = DATASET_DIR / "val" / "images"
VAL_LBL = DATASET_DIR / "val" / "labels"
DATA_YAML = DATASET_DIR / "data.yaml"


def images_in(d):
    return sorted(list(d.glob("*.jpg")) + list(d.glob("*.png"))
                  + list(d.glob("*.jpeg")))


def arrangement_of(path):
    """输出文件名形如 ``r06a2__img_0013.jpg`` —— 前缀就是排布名。"""
    return path.name.split("__")[0]


def check_dataset():
    """训练前的体检。**切分不在这里做**，只检查它已经是对的。

    三件事：图与标签一一对应、两边都非空、**train 与 val 的排布不重叠**。
    最后一条是这份数据集唯一不能出错的地方 —— 重叠了 val 指标就不可信，
    而且它不会报错，只会让数字好看。
    """
    ok = True
    arrs = {}
    for split, idir, ldir in (("train", TRAIN_IMG, TRAIN_LBL),
                              ("val", VAL_IMG, VAL_LBL)):
        imgs = images_in(idir)
        if not imgs:
            print(f"[ERROR] {idir}/ 里没有图像 —— 先跑 scripts/build_dataset.py 装配。")
            return False
        missing = [p.name for p in imgs if not (ldir / (p.stem + ".txt")).exists()]
        if missing:
            print(f"[ERROR] {split}: {len(missing)} 张图没有标签，例如 {missing[:3]}")
            ok = False
        arrs[split] = sorted({arrangement_of(p) for p in imgs})
        print(f"  {split:5s} {len(imgs):3d} 帧  排布 {', '.join(arrs[split])}")

    both = set(arrs["train"]) & set(arrs["val"])
    if both:
        print(f"[ERROR] 这些排布同时出现在 train 和 val：{sorted(both)}")
        print("        val 指标会虚高 —— 模型只要背下这些布局就能拿满分。")
        print("        请重跑 scripts/build_dataset.py 重新按排布切分。")
        ok = False

    man = DATASET_DIR / "dataset_manifest.json"
    if man.exists():
        import json
        m = json.load(open(man))
        print(f"  清单（**构建时**的记录）：train {m['split']['train']['boxes']} 框 / "
              f"val {m['split']['val']['boxes']} 框")
        # 清单是装配那一刻的快照。跟现况对一下 —— 对不上说明有人手工动过目录，
        # 那时清单里的出处追溯就不可信了，比数字本身更值得警惕。
        for split, idir in (("train", TRAIN_IMG), ("val", VAL_IMG)):
            now, was = len(images_in(idir)), m["split"][split]["frames"]
            if now != was:
                print(f"  [WARN] {split} 实际 {now} 帧，清单记的是 {was} 帧 "
                      f"—— 目录被手工改过，清单里的出处不再可信。")
    else:
        print("  [WARN] 没有 dataset_manifest.json —— 无法追溯每帧的出处。")
    return ok


def train(epochs, batch, imgsz, device_arg, name, workers):
    """训练 YOLOv8n。"""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] 请安装 ultralytics: pip install ultralytics")
        sys.exit(1)

    device = device_arg
    if device is None:
        device = "cuda:0"
        try:
            import torch
            if not torch.cuda.is_available():
                device = "cpu"
                print("[INFO] CUDA 不可用，使用 CPU 训练（会慢很多）。")
        except ImportError:
            device = "cpu"

    if not BASE_WEIGHTS.is_file():
        print(f"[ERROR] 找不到微调起点权重: {BASE_WEIGHTS}")
        print("        这是 COCO 预训练 yolov8n；缺失时 ultralytics 会联网下载，"
              "结果不可复现。请先补齐该文件。")
        sys.exit(1)

    print(f"\n加载预训练模型 {BASE_WEIGHTS.name}，设备: {device}")
    print(f"epochs={epochs} batch={batch} imgsz={imgsz}")
    model = YOLO(str(BASE_WEIGHTS))

    print("开始训练...")
    results = model.train(
        data=str(DATA_YAML),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        workers=workers,
        device=device,
        patience=20,          # 20 个 epoch 无提升则早停
        save=True,
        save_period=10,       # 每 10 epoch 保存一次
        project=str(PROJECT_ROOT / "runs"),
        name=name,
        exist_ok=True,
        pretrained=True,
        optimizer="auto",
        verbose=True,
    )

    # 复制最佳模型到项目根目录（yolo_detector 的默认 model 参数就指向它）
    best_src = PROJECT_ROOT / "runs" / name / "weights" / "best.pt"
    best_dst = PROJECT_ROOT / "block_best.pt"
    if best_src.exists():
        shutil.copy(str(best_src), str(best_dst))
        print(f"\n✅ 训练完成！最佳模型: {best_dst}")
        print(f"   验证指标: {results.results_dict}")
    else:
        print(f"\n⚠ 训练完成但未找到 best.pt，请检查日志。")

    # 旧版此处会把 best_dst **覆盖到仓库根的 yolov8n.pt**，并把被覆盖的文件
    # 备份成 yolov8n_backup.pt——D6「同名不同物」正是这样被制造出来的：
    # 方块模型占用了 yolov8n.pt 这个名字，真 COCO 反而被赶到 *_backup.pt。
    # 现在检测端默认就是 block_best.pt，这步覆盖已无必要，故删除。


if __name__ == "__main__":
    ap = argparse.ArgumentParser("训练 YOLOv8n 方块检测")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16,
                    help="8 GB 的 Orin 上建议 8 —— 显存与 CPU 共享同一个内存池")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--workers", type=int, default=8,
                    help="dataloader 进程数（默认 8；8 GB 的 Orin 上给 4，"
                         "每个 worker 都持有一份数据集缓存的引用）")
    ap.add_argument("--device", default=None, help="cuda:0 / cpu（默认自动）")
    ap.add_argument("--name", default="block_detect", help="runs/ 下的子目录名")
    a = ap.parse_args()

    print("=" * 60)
    print("  YOLOv8 方块检测训练")
    print("=" * 60)

    print("\n体检数据集（切分已由 build_dataset.py 按排布固化）...")
    if not check_dataset():
        print("\n❌ 数据集不合格，拒绝训练。")
        sys.exit(1)

    train(a.epochs, a.batch, a.imgsz, a.device, a.name, a.workers)
