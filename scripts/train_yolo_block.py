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
# 默认数据集。可以用 --data 换成别的（例如 scripts/relabel_by_color.py 产出的
# 多类别版本），所以下面所有目录都由**传入的数据集目录**派生，不再是模块常量。
DEFAULT_DATASET = PROJECT_ROOT / "block_dataset"

# 微调起点：**真 COCO yolov8n**（原名为 yolov8n_backup.pt，已改名以免混淆）。
# 注意：旧版此处写的是 "yolov8n.pt"，而仓库根那个文件其实是上一次训练产出的
# 方块模型——于是「微调」实际上是在**上一代方块模型**上继续训，而不是从 COCO
# 起。行为随 CWD 变化，且文件名与内容不符（详见 docs/RESTRUCTURE_PLAN.md D6）。
BASE_WEIGHTS = PROJECT_ROOT / "yolov8n_coco.pt"


def images_in(d):
    return sorted(list(d.glob("*.jpg")) + list(d.glob("*.png"))
                  + list(d.glob("*.jpeg")))


def arrangement_of(path):
    """输出文件名形如 ``r06a2__img_0013.jpg`` —— 前缀就是排布名。"""
    return path.name.split("__")[0]


def read_yaml_names(path):
    """只解析 data.yaml 里的 ``names:`` / ``nc:``，不引入 pyyaml 依赖。

    为什么要自己解析：训练机上未必装了 pyyaml，而这几个字段是**必须**在训练前
    核对的东西 —— 为了省一个可选依赖就跳过核对，等于把「类别表漂移」这个静默
    故障放进来。
    """
    names, nc, in_names = {}, None, False
    for raw in open(path, encoding="utf-8"):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if in_names and line.startswith((" ", "\t")):
            k, _, v = line.strip().partition(":")
            if v.strip():
                names[int(k)] = v.strip().strip("'\"")
            continue
        in_names = False
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if key == "names":
            in_names = not val          # `names:` 后跟缩进块；`names: {0: a}` 内联
            if val.startswith("{"):
                for part in val.strip("{} \n").split(","):
                    k, _, v = part.partition(":")
                    if v.strip():
                        names[int(k.strip())] = v.strip().strip("'\"")
        elif key == "nc":
            nc = int(val)
    return names, nc


def check_class_table(ds, strict_known_classes=True):
    """核对 ``data.yaml`` 的类别表，并核对每个标签里的类别号都在范围内。

    这是本脚本**唯一**能拦住「类别表漂移」的地方，而漂移的症状是静默的：标签里
    的类别号与 names 错位之后，训练照样跑完、loss 照样下降，只是模型学到的名字
    全错（详见 scripts/block_classes.py 开头的 ⚠️）。所以宁可在这里拒绝开训。

    ``strict_known_classes``：类别数 >1 时，要求 names 与 block_classes 里那份
    **唯一定义**逐位相同。顺序不同不算通过 —— 顺序即索引，换顺序等于重编所有标签。
    """
    ok = True
    y = ds / "data.yaml"
    if not y.is_file():
        print(f"[ERROR] 没有 {y} —— 数据集不完整。")
        return False
    names, nc = read_yaml_names(y)
    if nc is None:
        nc = len(names)
    if sorted(names) != list(range(nc)):
        print(f"[ERROR] data.yaml 的 names 不是 0..{nc - 1} 的连续编号：{sorted(names)}")
        return False
    ordered = [names[i] for i in range(nc)]
    print(f"  类别表 ({nc} 类)：{ordered}")

    if nc != len(names):
        print(f"[ERROR] nc={nc} 与 names 的项数 {len(names)} 对不上。")
        ok = False

    if nc > 1 and strict_known_classes:
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from block_classes import DEFAULT_CLASSES
        except ImportError:
            print("  [WARN] 找不到 block_classes.py，跳过类别表核对。")
            DEFAULT_CLASSES = None
        if DEFAULT_CLASSES is not None and ordered != list(DEFAULT_CLASSES):
            print("[ERROR] 多类别数据集的类别表与 scripts/block_classes.py 不一致：")
            print(f"        data.yaml  : {ordered}")
            print(f"        唯一定义处 : {list(DEFAULT_CLASSES)}")
            print("        顺序不同也算不一致 —— 类别号 = 下标，换顺序等于重编所有标签，")
            print("        而且**不会报错**。确认这是有意为之再传 --allow-other-classes。")
            ok = False

    # 标签里的类别号越界：写法是「静默错」，所以要逐文件查，别抽查
    bad = {}
    for split in ("train", "val"):
        ldir = ds / split / "labels"
        if not ldir.is_dir():
            continue
        for f in sorted(ldir.glob("*.txt")):
            for ln, line in enumerate(open(f, encoding="utf-8"), 1):
                if not line.strip():
                    continue
                cid = int(line.split()[0])
                if not 0 <= cid < nc:
                    bad.setdefault(str(f.relative_to(ds)), []).append((ln, cid))
    if bad:
        print(f"[ERROR] {len(bad)} 个标签文件里有越界类别号（应当都在 [0,{nc})），例如：")
        for k in sorted(bad)[:3]:
            print(f"        {k}:{bad[k][0][0]} 类别号={bad[k][0][1]}")
        print("        多半是标签用一张类别表写的、data.yaml 用了另一张。")
        ok = False
    return ok


def check_dataset(ds, strict_known_classes=True):
    """训练前的体检。**切分不在这里做**，只检查它已经是对的。

    四件事：图与标签一一对应、两边都非空、**train 与 val 的排布不重叠**、
    类别表自洽。排布不重叠是这份数据集最容易出错的地方 —— 重叠了 val 指标就
    不可信，而且它不会报错，只会让数字好看。
    """
    ok = True
    arrs = {}
    for split in ("train", "val"):
        idir, ldir = ds / split / "images", ds / split / "labels"
        imgs = images_in(idir)
        if not imgs:
            print(f"[ERROR] {idir}/ 里没有图像 —— 先装配数据集。")
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

    if not check_class_table(ds, strict_known_classes):
        ok = False

    # 清单是装配那一刻的快照，两种数据集写出来的**格式不同**（build_dataset.py 按
    # split 统计，relabel_by_color.py 按类别统计），所以这里只做能做的核对，别硬解。
    man = ds / "dataset_manifest.json"
    if man.exists():
        import json
        m = json.load(open(man))
        if "split" in m:
            print(f"  清单（**构建时**的记录）：train {m['split']['train']['boxes']} 框 / "
                  f"val {m['split']['val']['boxes']} 框")
            for split in ("train", "val"):
                now, was = len(images_in(ds / split / "images")), m["split"][split]["frames"]
                if now != was:
                    print(f"  [WARN] {split} 实际 {now} 帧，清单记的是 {was} 帧 "
                          f"—— 目录被手工改过，清单里的出处不再可信。")
        elif "class_counts" in m:
            cc = m["class_counts"]
            print("  清单（重标工具写的）：" +
                  "  ".join(f"{k}={v}" for k, v in cc.items()))
            if m.get("overrides_applied"):
                print(f"  其中 {m['overrides_applied']} 个框是人工复核后改判的，"
                      f"明细见清单的 manual_overrides。")
    else:
        print("  [WARN] 没有 dataset_manifest.json —— 无法追溯每帧的出处。")
    return ok


def train(ds, epochs, batch, imgsz, device_arg, name, workers, weights_out):
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
    print(f"数据集: {ds}")
    print(f"epochs={epochs} batch={batch} imgsz={imgsz}")
    model = YOLO(str(BASE_WEIGHTS))

    print("开始训练...")
    results = model.train(
        data=str(ds / "data.yaml"),
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

    # 复制最佳模型到项目根目录（yolo_detector 的默认 model 参数就指向 block_best.pt）
    best_src = PROJECT_ROOT / "runs" / name / "weights" / "best.pt"
    best_dst = PROJECT_ROOT / weights_out
    if best_src.exists():
        # 覆盖前说一声：单类别与多类别的权重都叫 block_best.pt 系列，覆盖错了
        # 只会表现为「检测出来的名字不对」，不会报错。
        if best_dst.exists():
            print(f"\n[注意] 覆盖已有权重 {best_dst}"
                  f"（{best_dst.stat().st_size / 1e6:.1f} MB）")
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
    ap.add_argument("--data", default=str(DEFAULT_DATASET),
                    help="数据集目录（含 data.yaml）。默认 block_dataset；"
                         "多类别用 scripts/relabel_by_color.py 产出的那份。")
    ap.add_argument("--weights-out", default="block_best.pt",
                    help="训练完把 best.pt 复制到仓库根的这个文件名。"
                         "多类别模型**别**默认覆盖单类别的 block_best.pt。")
    ap.add_argument("--allow-other-classes", action="store_true",
                    help="允许 data.yaml 的类别表与 block_classes.DEFAULT_CLASSES 不同")
    a = ap.parse_args()

    print("=" * 60)
    print("  YOLOv8 方块检测训练")
    print("=" * 60)

    ds = Path(a.data).resolve()
    if not ds.is_dir():
        print(f"\n❌ 数据集目录不存在：{ds}（退 2）")
        sys.exit(2)

    print(f"\n体检数据集（切分已由装配脚本按排布固化）...")
    if not check_dataset(ds, strict_known_classes=not a.allow_other_classes):
        print("\n❌ 数据集不合格，拒绝训练。")
        sys.exit(1)

    train(ds, a.epochs, a.batch, a.imgsz, a.device, a.name, a.workers,
          a.weights_out)
