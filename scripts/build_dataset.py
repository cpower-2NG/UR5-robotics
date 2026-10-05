#!/usr/bin/env python3
"""把 `.scratch/rounds/*` 里标好的帧装成一个 YOLO 数据集，**按排布切分** train/val。

为什么必须按排布切，不能按帧随机切
==================================
`train_yolo_block.py` 原来的 `split_val()` 是「随机抽 20% 的**帧**」。
对这份数据这是**假的**：一个排布下我们拍了几十帧，**每一帧的方块布局完全相同**，
模型只要背下那 25 个坐标就能在 val 上刷满分。val 指标于是很好看，
而它衡量的只是「记住没有」，不是「换个摆法还认不认」。

部署时方块是**当场摆的**，永远是没见过的排布。所以 val 必须整块留出排布，
train 里一帧都不能是它的。这份脚本把这条写死成断言（`plan_split`）。

顺带解决的三件事
================
1. **文件名撞车**：`r00/img_0000.png` 与 `r02/img_0000.png` 同名。
   输出统一加排布前缀 `r00__img_0000.png`，撞不了，也一眼看得出出处。
2. **体积**：`r00`~`r05` 是 720p PNG，约 1.2 MB/张，210 帧共 182 MB。
   `--jpeg-quality` 会把**非 JPEG** 源转成 JPEG（已是 JPEG 的原样复制，
   不做二次有损）。用来把入库体积压到可接受。
3. **标签体检**：格式、类别号、归一化范围、框是否越界，全在这里挡下来 ——
   错标签一旦进数据集，症状是「模型看着还行、就是抓不准」，最难查。

非帧文件（`info.json`、`selected_sources.txt`、`click_frame.jpg`）
不匹配 `img_*`，会被跳过并**打印出来**，不会被当成训练帧静默收进去。

用法::

    python3 scripts/build_dataset.py --out block_dataset \\
        --val r02,r06a2 --jpeg-quality 92
    python3 scripts/build_dataset.py --out block_dataset_multiclass \\
        --class-names red,orange,yellow,cyan,purple

**类别表（多类别）**

`--class-names` 写进 `data.yaml` 的 `names`/`nc`，同时是**类别号的上界**：
`parse_label_line()` 拿 `nc` 校验每一行的类别号。所以顺序错了不会报错 ——
每个号都还在 `[0,nc)` 里，错的是语义。**必须与生成世界时的
`generate_grid_world.py --classes` 传同一个列表、同一顺序。**

退出码：0 = 装好；1 = 数据本身有问题（图/标签对不上、标签越界）；2 = 输入不对。
"""
import argparse
import glob
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from block_classes import parse_classes                             # noqa: E402

try:
    import cv2
except ImportError:                                                # pragma: no cover
    cv2 = None

EXTS = (".jpg", ".jpeg", ".png")

# 排布名 -> (图目录, 标签目录)，相对于 --rounds-root。
# 名字就是「一轮采集 = 一个排布」，切分以它为单位。
SOURCES = [
    ("r00",   "r00/images",              "r00/labels"),
    ("r01",   "r01/images",              "r01/labels"),
    ("r02",   "r02/images",              "r02/labels"),
    ("r03",   "r03/images",              "r03/labels"),
    ("r05",   "r05/images",              "r05/labels"),
    ("r06a1", "r06/selected",            "r06/labels"),
    ("r06a2", "r06/arr2/images",         "r06/arr2/labels"),
]

DEFAULT_VAL = "r02,r06a2"


def stem_of(path):
    return os.path.splitext(os.path.basename(path))[0]


def parse_label_line(line, nc, where="?"):
    """把一行 YOLO 标签解析成 ``(cls, cx, cy, w, h)``；任何不合规都抛 ValueError。"""
    parts = line.split()
    if len(parts) != 5:
        raise ValueError("%s: 一行应有 5 个字段，实得 %d（%r）" % (where, len(parts), line))
    try:
        cls = int(parts[0])
        cx, cy, w, h = (float(p) for p in parts[1:])
    except ValueError:
        raise ValueError("%s: 字段不是数字（%r）" % (where, line))
    if not (0 <= cls < nc):
        raise ValueError("%s: 类别号 %d 不在 [0,%d)" % (where, cls, nc))
    for nm, v in (("cx", cx), ("cy", cy), ("w", w), ("h", h)):
        if not (0.0 <= v <= 1.0):
            raise ValueError("%s: %s=%r 不在 [0,1]" % (where, nm, v))
    if w <= 0 or h <= 0:
        raise ValueError("%s: 宽高必须为正（w=%r h=%r）" % (where, w, h))
    # 框要落在图里（留一点点容差：标注器算的是浮点，边界上差 1e-9 不算错）
    eps = 1e-6
    if (cx - w / 2 < -eps or cy - h / 2 < -eps
            or cx + w / 2 > 1 + eps or cy + h / 2 > 1 + eps):
        raise ValueError("%s: 框越出画面（cx=%.4f cy=%.4f w=%.4f h=%.4f）"
                         % (where, cx, cy, w, h))
    return cls, cx, cy, w, h


def read_label(path, nc):
    """读一份标签文件，逐行校验；空文件合法（负样本），返回 ``[]``。"""
    boxes = []
    with open(path) as f:
        for i, line in enumerate(f, 1):
            if not line.strip():
                continue
            boxes.append(parse_label_line(line, nc, "%s:%d" % (path, i)))
    return boxes


def pair_frames(img_paths, lbl_paths):
    """按 stem 配对，返回 ``(pairs, 只有图, 只有标签)`` —— 三样都返回，不做静默丢帧。"""
    imap, lmap = {}, {}
    for p in img_paths:
        imap.setdefault(stem_of(p), []).append(p)
    for p in lbl_paths:
        lmap.setdefault(stem_of(p), []).append(p)
    pairs = []
    for s in sorted(set(imap) & set(lmap)):
        if len(imap[s]) > 1 or len(lmap[s]) > 1:
            raise ValueError("stem %r 有重名文件：%r / %r" % (s, imap[s], lmap[s]))
        pairs.append((imap[s][0], lmap[s][0]))
    return pairs, sorted(set(imap) - set(lmap)), sorted(set(lmap) - set(imap))


def plan_split(names, val_names):
    """把排布名分成 ``(train, val)``。**同一个排布绝不允许两边都出现。**"""
    names, val_names = list(names), list(val_names)
    unknown = [v for v in val_names if v not in names]
    if unknown:
        raise ValueError("--val 里有不存在的排布：%s（可选：%s）"
                         % (", ".join(unknown), ", ".join(names)))
    if not val_names:
        raise ValueError("--val 不能为空 —— 没有 val 就没法判断模型是不是在背书")
    train = [n for n in names if n not in val_names]
    val = [n for n in names if n in val_names]
    both = set(train) & set(val)
    if both:                                     # 走不到，除非上面的逻辑被改坏
        raise ValueError("排布同时进了 train 和 val：%s" % sorted(both))
    if not train:
        raise ValueError("train 一个排布都不剩")
    return train, val


def collect(rounds_root, sources, nc, log=print):
    """把各排布的帧读进来并校验。

    返回 ``({排布: [(图, 标签, 框数), ...]}, {类别号: 框数})``。
    第二个返回值是**全库**的每类框数，供调用方查「声明了但没数据」的类别。
    """
    out, per_class = {}, {}
    for name, img_rel, lbl_rel in sources:
        idir = os.path.join(rounds_root, img_rel)
        ldir = os.path.join(rounds_root, lbl_rel)
        if not os.path.isdir(idir) or not os.path.isdir(ldir):
            raise ValueError("排布 %s 的目录不存在：%s / %s" % (name, idir, ldir))
        # 只认 img_* + 图片扩展名。**把没被收走的一个个列出来**，不靠「反正 glob 不到」：
        # info.json / selected_sources.txt / click_frame.jpg 是已知的非帧文件，
        # 但万一哪天有人把帧叫成 frame_0080.jpg，静默漏掉一帧是查不出来的。
        imgs, skipped = [], []
        for e in sorted(os.listdir(idir)):
            p = os.path.join(idir, e)
            if not os.path.isfile(p):
                continue
            if e.startswith("img_") and os.path.splitext(e)[1].lower() in EXTS:
                imgs.append(p)
            else:
                skipped.append(e)
        if skipped:
            log("  %s：图目录里没被收走的文件 %d 个（%s）"
                % (name, len(skipped), ", ".join(skipped)))
        lbls = sorted(glob.glob(os.path.join(ldir, "*.txt")))
        pairs, only_img, only_lbl = pair_frames(imgs, lbls)
        if only_img:
            raise ValueError("%s：这些图没有标签：%s" % (name, only_img))
        if only_lbl:
            raise ValueError("%s：这些标签没有图：%s" % (name, only_lbl))
        if not pairs:
            raise ValueError("%s：一帧都没配上" % name)
        frames = []
        for i, l in pairs:
            boxes = read_label(l, nc)
            for cls, _cx, _cy, _w, _h in boxes:
                per_class[cls] = per_class.get(cls, 0) + 1
            frames.append((i, l, len(boxes)))
        out[name] = frames
    # 返回每类的框数：调用方靠它发现「data.yaml 里声明了、数据里一个框都没有」
    # 的类别。那个症状不是报错，而是**混淆矩阵里一整行全零** ——
    # 看着像「这一类召回 0」，实际是「这批数据里根本没有这一类」，两回事。
    return out, per_class


def write_split(frames, split_dir, name_to_out, jpeg_quality=None):
    """把 ``frames`` 写进 ``split_dir/{images,labels}``，返回 ``(帧数, 框数)``。"""
    idir = os.path.join(split_dir, "images")
    ldir = os.path.join(split_dir, "labels")
    os.makedirs(idir, exist_ok=True)
    os.makedirs(ldir, exist_ok=True)
    n_box = 0
    for arr, img, lbl, nbin in frames:
        stem = "%s__%s" % (arr, stem_of(img))
        src_ext = os.path.splitext(img)[1].lower()
        if jpeg_quality and src_ext not in (".jpg", ".jpeg"):
            if cv2 is None:
                raise RuntimeError("--jpeg-quality 需要 opencv-python")
            im = cv2.imread(img)
            if im is None:
                raise ValueError("读不出图：%s" % img)
            dst = os.path.join(idir, stem + ".jpg")
            cv2.imwrite(dst, im, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
        else:
            dst = os.path.join(idir, stem + src_ext)   # 已是 JPEG 的原样复制，不做二次有损
            shutil.copy2(img, dst)
        shutil.copy2(lbl, os.path.join(ldir, stem + ".txt"))
        # 键用**落盘的真实文件名**（带扩展名）—— 不然拿清单里的名字去磁盘上找会找不着
        name_to_out[os.path.basename(dst)] = {"arrangement": arr, "image": img, "label": lbl}
        n_box += nbin                      # 框数是 collect 校验过的，这里不重读
    return len(frames), n_box


def write_yaml(out_dir, nc, names):
    """写 data.yaml。**故意不写 `path:`**（D57）—— 见下面注释。"""
    path = os.path.join(out_dir, "data.yaml")
    with open(path, "w") as f:
        f.write("# YOLO 方块检测数据集（由 scripts/build_dataset.py 从 .scratch/rounds/* 装配）\n"
                "# 故意不写 `path:`（D57）：ultralytics 在缺 `path` 时把相对路径按\n"
                "# **本 yaml 所在目录**解析，于是整份数据集可以整体搬到板上而不用改任何一行。\n"
                "# 写死绝对路径会随仓库/机器搬家而死，写相对路径又会先被解析到 DATASETS_DIR。\n"
                "train: train/images\n"
                "val: val/images\n"
                "names:\n")
        for i, nm in enumerate(names):
            f.write("  %d: %s\n" % (i, nm))
        f.write("nc: %d\n" % nc)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser("把标注好的帧装成 YOLO 数据集（按排布切分）")
    ap.add_argument("--out", required=True, help="输出目录")
    ap.add_argument("--rounds-root", default=".scratch/rounds")
    ap.add_argument("--val", default=DEFAULT_VAL,
                    help="留作 val 的排布名，逗号分隔（默认 %s）" % DEFAULT_VAL)
    ap.add_argument("--jpeg-quality", type=int, default=None,
                    help="把非 JPEG 源转成该质量的 JPEG（默认不转，原样复制）")
    ap.add_argument("--clean", action="store_true",
                    help="先删掉输出目录里已有的 train/ val/（不动 data.yaml 之外的东西）")
    # 两个参数都默认 None，才能区分「没传」与「传了默认值」——
    # 否则 `--class-names block --class-name foo` 这种矛盾调用会被静默放行，
    # 而类别表选错的症状是「标签与 names 整体错位、且一步都不报错」。
    ap.add_argument("--class-names", default=None,
                    help="类别表，逗号分隔，**顺序即类别号**（默认 block）。"
                         "必须与 generate_grid_world.py --classes 传同一个列表。")
    ap.add_argument("--class-name", default=None,
                    help="旧参数，等价于只有一类的 --class-names。"
                         "两个都给会报错（谁赢都是猜）。")
    a = ap.parse_args(argv)

    try:
        if a.class_name is not None and a.class_names is not None:
            raise ValueError(
                "--class-name 与 --class-names 同时给了。请只用一个：\n"
                "    多类别用 --class-names red,orange,yellow,cyan,purple\n"
                "    单类别用 --class-names block（或旧的 --class-name block）")
        spec = a.class_name if a.class_name is not None else a.class_names
        # 都不传 = 单类 "block"（与旧行为一致，别把老命令弄坏）
        # allow_unknown=True：真实数据集的类别名不必在仿真调色板里
        # （那只是仿真渲染用的颜色表）。这里只用它校验名字的**语法**
        # —— 小写、无重复、非空。类别名会进 data.yaml 与 ROS 话题，
        # 名字里有空格或逗号会让下游的 split(",") 静默切错。
        names = parse_classes(spec if spec is not None else "block",
                              allow_unknown=True)
    except ValueError as e:
        print("❌ %s" % e)
        return 2
    nc = len(names)
    val_names = [s.strip() for s in a.val.split(",") if s.strip()]

    try:
        train_names, val_names = plan_split([s[0] for s in SOURCES], val_names)
    except ValueError as e:
        print("❌ %s" % e)
        return 2

    try:
        by_arr, per_class = collect(a.rounds_root, SOURCES, nc)
    except (ValueError, OSError) as e:
        print("❌ %s" % e)
        return 1

    if a.clean:
        for sub in ("train", "val"):
            shutil.rmtree(os.path.join(a.out, sub), ignore_errors=True)

    print("\n排布 → 切分：")
    for name, _i, _l in SOURCES:
        print("  %-6s %s  帧 %2d  框 %4d"
              % (name, "val  " if name in val_names else "train",
                 len(by_arr[name]), sum(b for _i2, _l2, b in by_arr[name])))

    # 类别表与数据是否对得上 —— 这是**本脚本多类别下唯一能做的语义检查**。
    # 号码顺序错位它查不出来（每个号都还在 [0,nc) 里），但「某一类一个框都
    # 没有」查得出来，而那正是最常见的错法：类别表照新方案写了 5 类，
    # 数据却是老的单类数据。
    print("\n类别表 → 框数：")
    zero = []
    for i, nm in enumerate(names):
        n = per_class.get(i, 0)
        print("  %d:%-12s %5d 框" % (i, nm, n))
        if n == 0:
            zero.append(nm)
    if zero:
        print("⚠️ 这些类别**一个框都没有**：%s" % ", ".join(zero))
        print("   不是「召回率 0」，是这批数据里根本没有这一类 —— "
              "训出来的模型对这些类只会瞎猜，而验证集上的混淆矩阵"
              "会显示成整行全零，看着像模型不行。")
        print("   要么这批数据确实缺这些类，要么 --class-names 与标注时的"
              "类别表不是同一套。")

    prov, stat = {}, {}
    for split, arr_names in (("train", train_names), ("val", val_names)):
        frames = [(n, i, l, b) for n in arr_names for (i, l, b) in by_arr[n]]
        nf, nb = write_split(frames, os.path.join(a.out, split), prov, a.jpeg_quality)
        stat[split] = {"frames": nf, "boxes": nb, "arrangements": arr_names}

    yaml_path = write_yaml(a.out, nc, names)
    with open(os.path.join(a.out, "dataset_manifest.json"), "w") as f:
        json.dump({"sources": {n: {"images": i, "labels": l} for n, i, l in SOURCES},
                   "split": stat, "class_names": names,
                   # 每类框数进清单：训练时对不上就能回溯到「这批数据本来
                   # 就没有这一类」，而不是怀疑模型。
                   "class_boxes": {nm: per_class.get(i, 0)
                                   for i, nm in enumerate(names)},
                   "jpeg_quality": a.jpeg_quality,
                   "files": prov}, f, indent=1, ensure_ascii=False)

    tr, va = stat["train"], stat["val"]
    print("\n train  %3d 帧 %4d 框  %s" % (tr["frames"], tr["boxes"], "+".join(tr["arrangements"])))
    print(" val    %3d 帧 %4d 框  %s" % (va["frames"], va["boxes"], "+".join(va["arrangements"])))
    print(" val 占比 %.1f%%（按框）" % (100.0 * va["boxes"] / max(1, tr["boxes"] + va["boxes"])))
    print(" data.yaml → %s" % yaml_path)
    print(" 清单      → %s" % os.path.join(a.out, "dataset_manifest.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
