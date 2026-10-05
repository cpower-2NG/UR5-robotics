#!/usr/bin/env python3
"""把**单类别**方块数据集的框按颜色重标成多类别，另存成一个新数据集。

用法::

    python3 scripts/relabel_by_color.py --src block_dataset --out block_dataset_7cls

**只读 `--src`，不改它一个字节**，全部写进 `--out`（见下面「为什么另存」）。

为什么要另存而不是原地改
========================
原数据集 4695 个框全是 `class_id 0`，是「有方块/没方块」那版。原地改会把那份
**唯一的单类真值**抹掉，而重标是启发式的、可能还要再调一轮。另存之后，
两份都还在，改判据重跑一次即可。

为什么这件事只是 155 个框的问题，不是 4695 个
============================================
同一轮连拍里，**每一帧的标签文件逐字节相同**（实测：7 个标注组各自的 md5 只有一个值）。
所以 `r05` 的 25 帧是同一份标注，重标一次即可，不必逐帧判。分组键取
**文件名里 `__` 之前的前缀**（`r00`、`r06a1` …）——它就是一次采集。
⚠️ 不能按「session 名」分：`r06a1` 与 `r06a2` 是同一次采集的连续帧，
但两组标签**不同**（检测器在中间重跑过），是两个标注组。

判据（三层，前两层都被实测证伪过，见下）
========================================
1. 对每个框，把框内像素逐个判给 **7 个已知类别中心**里最近的那个
   （距离在 `(色相, 归一化饱和度)` 二维上算）；
2. 取「**框心加权**像素数」最多的那一类；
3. 跨帧取多数类，再在同类帧里取环形均值。

⚠️ **不要再退回「找色团」那套做法。** 走过的两条弯路：

  · **按直方图单个 bin 的峰高选主色** —— 棕的峰尖而窄、紫的峰宽而平，
    紫块（723 px）会输给黄邻居（341 px）。改成按「窗口内总像素数」还不够。
  · **窗口跨 0°/360°** —— 会把「紫(340-360)」和「棕(20-35)」并成一个假色团，
    紫块被标成 `H≈5 → red`。Lloyd 迭代聚类也会在跨零点处吸收对面的颜色。

  根本原因：这两条都在**试图发现色团**，于是必须处理合并窗口、跨零、噪声底。
  而我们是**已知**这 7 类的，直接判给最近的类就没有这些问题。

为什么权重是「框心距离」而不是「饱和度」
========================================
YOLO 框以目标为中心，邻居方块只从**边缘**挤进来 —— 所以按到框心的距离
`exp(-d²/2σ²)` 加权。**权重里不能乘饱和度**：紫的 S≈60、棕/橙的 S≈110~200，
乘上去等于把「一小块高饱和邻居压过框里面积更大的紫块」这个 bug 请回来
（第一版正是这样错的，r05 #6 实测 S=204 是橙块却被判成 tan）。

为什么饱和度要先按组归一化
==========================
色相基本不随曝光变，**饱和度会**：实测各组的中位饱和度 168/159/154/156/154/131/125。
不归一化的话，「橙 vs 棕」这种靠饱和度分开的对比会随拍摄批次漂移。
归一化后 `sn = S / 本组中位S`。

类别中心从哪来
==============
`CENTERS` 是在**本机俯视相机**上对用户那批积木实测出来的（7 色 × 2 块）。
拿 A 次测量的中心去标 B 次的图是有风险的做法，所以脚本跑完会
**打印每类的实测分布**（`--report` 默认开）—— 分布与中心明显对不上时，
说明这份数据不该用这套中心，别硬标。

退出码：0 = 标好；1 = 数据本身有问题；2 = 输入不对。
"""
import argparse
import collections
import hashlib
import json
import os
import shutil
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from block_classes import DEFAULT_CLASSES, PALETTE, class_index   # noqa: E402

try:
    import cv2
except ImportError:                                                # pragma: no cover
    cv2 = None
try:
    import numpy as np
except ImportError:                                                # pragma: no cover
    np = None

# 类别中心：(名字, 色相°, 归一化饱和度)。顺序 = 类别号，与 DEFAULT_CLASSES 必须一致。
CENTERS = [
    ("red",          8.0, 1.07),
    ("orange",      20.5, 1.28),
    ("tan",         28.5, 0.95),
    ("yellow",      40.0, 1.17),
    ("yellowgreen", 53.0, 1.01),
    ("cyan",       191.0, 0.85),
    ("purple",     308.0, 0.55),
]

HS = 8.0          # 色相尺度：多少度算「一个单位」
SS = 0.25         # 饱和度尺度。⚠️ 别调回 0.15 —— 那么紧时，红/橙之间
                  # 噪声级别的 sn 差 0.05 就能翻盘，而这两类的饱和度本来就重叠。
SIGMA = 0.20      # 框心权重的高斯 σ（以框宽/高为单位）
S_MIN = 50        # 饱和度下限：白纸 ≈24、木桌面 ≈30，挡在下面
V_MIN, V_MAX = 40, 250
EXTS = (".jpg", ".jpeg", ".png")


# 人工复核后的改判。键是 (采集组, 框序号)，值是新类别名。
#
# 为什么要有这张表：自动判据在「框里同时有本块和邻居」时会数错，而**错的那一票
# 照样会被写进标签、训进权重**，且事后完全看不出来 —— 与 block_classes.py 开头
# 警告的是同一类故障（不报错，只是语义错）。所以人眼看过的结论必须落到代码里，
# 不能只留在某次会话的对话里。
#
# 每条都必须写清**凭什么改**，否则下一个人无从复核，只能选择相信。
MANUAL_OVERRIDES = {
    # 2026-10-05 人工复核（5 倍放大 + 标出框心 40% 区域，见 .scratch/relabel_preview/）
    # 自动判据给 orange（56% 主色占比、逐帧仅 38% 一致），runner 是 purple。
    # 放大图里框心明确落在**紫块**上：该框横跨紫块与邻块，权重最高的框心区是紫的，
    # 只是紫块在该框内面积占比小、饱和又低（S≈62），被面积口径压过去了。
    ("r06a2", 12): "purple",
}


# ----------------------------------------------------------------------
def group_of(filename):
    """``r06a2__img_0051.jpg`` → ``r06a2``。一次采集 = 一个饱和度归一化组。"""
    return os.path.basename(filename).split("__")[0]


def circ_mean(deg):
    """环形均值。**不能**用算术平均：跨 0° 的红（356° 与 4°）会算出 180°。"""
    r = np.radians(np.asarray(deg, dtype=float))
    return float(np.degrees(np.arctan2(np.sin(r).mean(), np.cos(r).mean())) % 360.0)


def circ_dist(a, b):
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


def box_pixels(img, cx, cy, w, h):
    """框内有效像素的 (色相, 饱和度, 明度, 框心权重)；有效像素太少返回 None。"""
    H, W = img.shape[:2]
    x0 = max(0, int(round((cx - w / 2.0) * W)))
    x1 = min(W, int(round((cx + w / 2.0) * W)))
    y0 = max(0, int(round((cy - h / 2.0) * H)))
    y1 = min(H, int(round((cy + h / 2.0) * H)))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    patch = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue = hsv[..., 0].astype(float) * 2.0        # OpenCV 的 H ∈ [0,179] → 度
    sat = hsv[..., 1].astype(float)
    val = hsv[..., 2].astype(float)
    m = (sat >= S_MIN) & (val >= V_MIN) & (val <= V_MAX)
    if int(m.sum()) < 10:
        return None
    ph, pw = patch.shape[:2]
    yy, xx = np.mgrid[0:ph, 0:pw]
    dx = (xx - (pw - 1) / 2.0) / max(pw, 1)
    dy = (yy - (ph - 1) / 2.0) / max(ph, 1)
    cw = np.exp(-(dx * dx + dy * dy) / (2.0 * SIGMA * SIGMA))
    return dict(hue=hue[m], sat=sat[m], val=val[m], cw=cw[m],
                n_px=int(m.sum()), n_box=int(ph * pw))


def classify(px, med_s):
    """把框内像素判给最近的类别中心，返回得票最多的那一类及其统计。"""
    sn = px["sat"] / med_s
    d = np.stack([np.sqrt((circ_dist_h(px["hue"], c[1]) / HS) ** 2
                          + ((sn - c[2]) / SS) ** 2) for c in CENTERS])
    k = np.argmin(d, axis=0)
    # ⚠️ 权重**只**取框心权重，不乘饱和度 —— 见文件头。
    w = px["cw"]
    mass = np.array([w[k == j].sum() for j in range(len(CENTERS))])
    order = np.argsort(-mass)
    j = int(order[0])
    sel = (k == j)
    return dict(cls=CENTERS[j][0],
                hue=circ_mean(px["hue"][sel]),
                sat=float(np.median(px["sat"][sel])),
                val=float(np.median(px["val"][sel])),
                share=float(mass[j] / max(mass.sum(), 1e-9)),
                ratio=float(mass[order[1]] / max(mass[j], 1e-9)),
                runner=CENTERS[order[1]][0],
                n_px=px["n_px"], n_box=px["n_box"])


def circ_dist_h(arr, c):
    d = np.abs(arr - c) % 360.0
    return np.minimum(d, 360.0 - d)


# ----------------------------------------------------------------------
def read_boxes(path):
    out = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 5:
                out.append(tuple(float(v) for v in p))
    return out


def group_frames(idir, ldir):
    """按「标签内容哈希」分组 —— 同组内标签逐字节相同，只需标一次。"""
    imgs = sorted(os.path.join(idir, f) for f in os.listdir(idir)
                  if f.lower().endswith(EXTS))
    groups = collections.OrderedDict()
    for p in imgs:
        lp = os.path.join(ldir, os.path.splitext(os.path.basename(p))[0] + ".txt")
        if not os.path.exists(lp):
            raise RuntimeError("图没有对应标签：%s" % p)
        key = (group_of(p), hashlib.md5(open(lp, "rb").read()).hexdigest())
        groups.setdefault(key, []).append(p)
    return groups


def relabel_split(idir, ldir, log=print):
    """返回 {(组, 帧名): [(cls_id, cx, cy, w, h), ...]} 与逐框诊断。"""
    groups = group_frames(idir, ldir)
    log("  分组：%d 个标注组（%d 帧）" % (len(groups), sum(len(v) for v in groups.values())))

    # 第一遍：收像素。饱和度按**采集组**归一化，所以先要知道各组的中位 S。
    per_group = collections.defaultdict(list)      # 采集组 -> [(框序号, px)]
    boxes_of = {}
    for (g, digest), frames in groups.items():
        lp = os.path.join(ldir, os.path.splitext(os.path.basename(frames[0]))[0] + ".txt")
        boxes_of[(g, digest)] = read_boxes(lp)
        for f in frames:
            img = cv2.imread(f)
            if img is None:
                raise RuntimeError("读不了图：%s" % f)
            for i, (_, cx, cy, w, h) in enumerate(boxes_of[(g, digest)]):
                px = box_pixels(img, cx, cy, w, h)
                if px is not None:
                    per_group[g].append(px)

    med_s = {}
    for g, lst in per_group.items():
        med_s[g] = float(np.median([np.median(p["sat"]) for p in lst]))
    log("  各组中位饱和度：" + "  ".join("%s=%.0f" % (g, med_s[g])
                                        for g in sorted(med_s)))

    # 第二遍：逐框判定
    labels, diag = {}, {}
    for (g, digest), frames in groups.items():
        boxes = boxes_of[(g, digest)]
        acc = collections.defaultdict(list)
        for f in frames:
            img = cv2.imread(f)
            for i, (_, cx, cy, w, h) in enumerate(boxes):
                px = box_pixels(img, cx, cy, w, h)
                if px is not None:
                    acc[i].append(classify(px, med_s[g]))
        rows = []
        for i, (_, cx, cy, w, h) in enumerate(boxes):
            cats = acc.get(i) or []
            if not cats:
                raise RuntimeError("%s 第 %d 个框在所有帧里都没有有效像素" % (g, i))
            votes = collections.Counter(c["cls"] for c in cats)
            auto = votes.most_common(1)[0][0]
            # 人工改判只改**写进标签的那一个类别号**。hue/sat/share 这些统计量
            # 仍按自动判出来的那一类记账 —— 否则 report() 打印的分布会掺进一堆
            # 系统本来判成别的类的像素，看起来「改判后反而更分得开」，把改判
            # 这件事本身藏了起来。
            cls = MANUAL_OVERRIDES.get((g, i), auto)
            same = [c for c in cats if c["cls"] == auto]
            rows.append((class_index(cls, [c[0] for c in CENTERS]), cx, cy, w, h))
            diag[(g, i)] = dict(
                cls=auto, final=cls, overridden=(cls != auto),
                hue=circ_mean([c["hue"] for c in same]),
                sat=float(np.median([c["sat"] for c in same])),
                sn=float(np.median([c["sat"] for c in same]) / med_s[g]),
                share=float(np.median([c["share"] for c in same])),
                ratio=float(np.median([c["ratio"] for c in same])),
                runner=collections.Counter(c["runner"] for c in same).most_common(1)[0][0],
                agree=votes.most_common(1)[0][1] / float(len(cats)),
                n_frames=len(cats))
        for f in frames:
            labels[f] = rows
    return labels, diag


# ----------------------------------------------------------------------
def report(diag, names, log=print):
    log("\n各类实测分布（按组内归一化饱和度 sn）：")
    log("  %-13s %4s  %-22s %-22s" % ("class", "n", "H 中位[min..max]", "sn 中位[min..max]"))
    by = collections.defaultdict(list)
    for d in diag.values():
        by[d["cls"]].append(d)
    for nm in names:
        v = by.get(nm) or []
        if not v:
            log("  %-13s %4d  —— 这一类一个框都没有" % (nm, 0))
            continue
        hs = [x["hue"] for x in v]
        sn = [x["sn"] for x in v]
        log("  %-13s %4d  %6.1f [%5.1f..%5.1f]    %5.2f [%4.2f..%4.2f]"
            % (nm, len(v), np.median(hs), min(hs), max(hs),
               np.median(sn), min(sn), max(sn)))

    # 中心与实测分布对不对得上 —— 中心来自另一次测量，这一步是它的看门人
    log("\n中心与实测的偏差（色相，度）：")
    off = []
    for nm, h0, _ in CENTERS:
        v = by.get(nm) or []
        if not v:
            continue
        d = circ_dist(np.median([x["hue"] for x in v]), h0)
        off.append((d, nm))
    off.sort(reverse=True)
    for d, nm in off:
        flag = "   ← 偏得多，这套中心可能不适用这份数据" if d > 12.0 else ""
        log("  %-13s 偏 %5.1f°%s" % (nm, d, flag))

    need = [((sp, g), i, d) for (sp, g, i), d in sorted(diag.items())
            if d["share"] < 0.75 or d["agree"] < 0.75 or d["ratio"] > 0.55]
    log("\n建议人眼复核的框：%d / %d" % (len(need), len(diag)))
    for (sp, g), i, d in need:
        why = []
        if d["share"] < 0.75:
            why.append("框里混进邻居（主色占 %.0f%%）" % (100 * d["share"]))
        if d["agree"] < 0.75:
            why.append("逐帧仅 %.0f%% 一致" % (100 * d["agree"]))
        if d["ratio"] > 0.55:
            why.append("与 %s 接近（%.0f%%）" % (d["runner"], 100 * d["ratio"]))
        mark = "  ← 已人工改判为 %s" % d["final"] if d.get("overridden") else ""
        log("   %-7s #%-3d %-12s H=%5.1f S=%3.0f  %s%s"
            % (g, i, d["cls"], d["hue"], d["sat"], "、".join(why), mark))
    return need


def copy_images(frames, idir, odir):
    os.makedirs(odir, exist_ok=True)
    for f in frames:
        dst = os.path.join(odir, os.path.basename(f))
        if not os.path.exists(dst):
            shutil.copy2(f, dst)


def write_labels(labels, ldir):
    os.makedirs(ldir, exist_ok=True)
    for f, rows in labels.items():
        stem = os.path.splitext(os.path.basename(f))[0]
        with open(os.path.join(ldir, stem + ".txt"), "w") as fh:
            for cls, cx, cy, w, h in rows:
                fh.write("%d %.6f %.6f %.6f %.6f\n" % (cls, cx, cy, w, h))


def write_yaml(out_dir, names):
    path = os.path.join(out_dir, "data.yaml")
    with open(path, "w") as f:
        f.write("# 由 scripts/relabel_by_color.py 生成（**不是** build_dataset.py）。\n"
                "# 故意不写 `path:`（D57）：相对路径按本 yaml 所在目录解析，\n"
                "# 整份数据集可以整体搬到板上而不用改任何一行。\n"
                "train: train/images\n"
                "val: val/images\n"
                "names:\n")
        for i, nm in enumerate(names):
            f.write("  %d: %s\n" % (i, nm))
        f.write("nc: %d\n" % len(names))
    return path


def main(argv=None):
    ap = argparse.ArgumentParser("按颜色把单类别方块数据集重标成多类别")
    ap.add_argument("--src", required=True, help="源数据集目录（只读）")
    ap.add_argument("--out", required=True, help="输出数据集目录（会写进去）")
    ap.add_argument("--classes", default=None,
                    help="类别表，逗号分隔，顺序即类别号（默认取 block_classes.DEFAULT_CLASSES）")
    a = ap.parse_args(argv)

    if cv2 is None or np is None:
        print("需要 opencv 与 numpy（退 2）")
        return 2

    names = list(DEFAULT_CLASSES) if a.classes is None else \
        [s.strip() for s in a.classes.split(",") if s.strip()]
    if names != [c[0] for c in CENTERS]:
        print("类别表与 CENTERS 对不上（退 2）：")
        print("  --classes 给了 %s" % names)
        print("  CENTERS   是  %s" % [c[0] for c in CENTERS])
        print("  类别号 = 下标，两者必须逐位相同，否则标签与名字整体错位且不报错。")
        return 2
    if not os.path.isdir(a.src):
        print("源目录不存在：%s（退 2）" % a.src)
        return 2
    os.makedirs(a.out, exist_ok=True)

    print("=" * 66)
    print("  按颜色重标：%s  →  %s" % (a.src, a.out))
    print("=" * 66)

    all_diag = {}
    for split in ("train", "val"):
        idir = os.path.join(a.src, split, "images")
        ldir = os.path.join(a.src, split, "labels")
        if not os.path.isdir(idir):
            print("  跳过 %s（源里没有）" % split)
            continue
        print("\n[%s]" % split)
        labels, diag = relabel_split(idir, ldir)
        copy_images(list(labels), idir, os.path.join(a.out, split, "images"))
        write_labels(labels, os.path.join(a.out, split, "labels"))
        # 键里带上 split：同一个采集组要是被切到两边，键也不能撞
        all_diag.update({(split,) + k: v for k, v in diag.items()})
        print("  %d 帧 × %d 个不同框 → 已写" %
              (len(labels), len({k[1] for k in diag})))

    y = write_yaml(a.out, names)
    # 计数按**写进标签的**那一类（final），不是自动判据那一类 —— 报告里给的是
    # 「这份数据集长什么样」，训练集里是什么就得报什么。
    cnt = collections.Counter(d["final"] for d in all_diag.values())
    n_ov = sum(1 for d in all_diag.values() if d["overridden"])
    man = dict(class_names=names,
               class_counts={nm: cnt.get(nm, 0) for nm in names},
               source=a.src, centers=[list(c) for c in CENTERS],
               scales=dict(HS=HS, SS=SS, SIGMA=SIGMA, S_MIN=S_MIN),
               boxes_total=len(all_diag),
               manual_overrides={"%s#%d" % k: v for k, v in sorted(MANUAL_OVERRIDES.items())},
               overrides_applied=n_ov,
               # 自动判据（未改判时）会得到什么 —— 留着好让「改判有没有生效」可查
               auto_class_counts={nm: sum(1 for d in all_diag.values()
                                          if d["cls"] == nm) for nm in names})
    with open(os.path.join(a.out, "dataset_manifest.json"), "w") as f:
        json.dump(man, f, ensure_ascii=False, indent=1)

    need = report(all_diag, names)
    print("\ndata.yaml: %s" % y)
    print("类别计数：" + "  ".join("%s=%d" % (nm, cnt.get(nm, 0)) for nm in names))
    if n_ov:
        print("其中 %d 个框按 MANUAL_OVERRIDES 改判过：" % n_ov)
        for (g, i), v in sorted(MANUAL_OVERRIDES.items()):
            auto = all_diag.get(("train", g, i)) or all_diag.get(("val", g, i)) or {}
            print("   %s #%d  自动判据=%s  人眼复核后写入=%s"
                  % (g, i, auto.get("cls", "?"), v))
    print("共 %d 个不同框，其中 %d 个建议人眼复核。" % (len(all_diag), len(need)))
    missing = [nm for nm in names if not cnt.get(nm)]
    if missing:
        print("⚠️ 这些类一个框都没有：%s —— 训练时它们的召回率不是 0，是没有样本。"
              % ", ".join(missing))
    return 0


if __name__ == "__main__":
    sys.exit(main())
