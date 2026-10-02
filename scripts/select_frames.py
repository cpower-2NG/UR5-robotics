#!/usr/bin/env python3
"""从一堆连拍里挑出**真正互不相同**的帧 —— 相机和积木都不动，连拍大部分是重复的。

为什么需要这一步
================
俯视相机 ~29 Hz。人晃一次光照、或者相机就那么开着，几千帧里能提供的**独立样本**
只有几十个。原样全拿去标注就是拿同一张图训练几十遍：标签数量好看，模型没多学到东西。

实测（`rounds/r06`，2305 帧连拍）：过完这一步剩 **80 帧**。也就是说
**97% 的连拍是冗余**。

为什么标注流程敢这么用
======================
因为**几何标注根本不读图**：标签由「外参 + 一次点击」算出，光照怎么变都不影响。
所以同一排布可以只点一次，然后在**任意多种光照**下连拍，每多挑出一帧就多一份
标签 —— 光照多样性是白捡的。这条性质是这份脚本存在的理由：
`grab_frames.py` 负责多拍，这份负责把多拍里重复的去掉。

判据
====
两帧算「不同」，要**同时**满足两个条件（缺一不可）::

    灰阶差 > LEVELS(25) 的像素数  >  FRAC(2%) × 全图像素数

- 只差几个像素（传感器噪声、飘过的人影边缘）不算。
- 差一大片但每处都很轻微（自动曝光的整体漂移）也不算 —— 那正是要合并掉的那类。
- 先过一道**亮度窗口**（默认 40~225）：全黑和过曝的帧不提供标签价值，
  而且过曝会把方块边界冲掉。

贪心、和**已保留的每一帧**比。（不按「和上一帧比」是因为那种写法会把
A≈B≈C 里的 B、C 都收进来 —— 见自检 [3]。）

⚠️ **贪心的结果依赖输入顺序**，这是固有的：早进来的帧会挡掉后来跟它相似的帧。
所以 CLI 把**所有**路径**全局排一遍序**再选 —— 这样结果只取决于「有哪些文件」，
**不取决于你按什么顺序敲那几个目录**（`--dirs B A` 与 `--dirs A B` 结果相同）。
别依赖这一点去控制挑选结果，但要知道它不是随机的。

用法::

    python3 scripts/select_frames.py --out .scratch/rounds/r06/selected \\
        .scratch/rounds/r06/A/images .scratch/rounds/r06/B/images ...

会写两份东西：``selected/img_%04d.<ext>``（顺序编号，供标注器 glob）
和 ``selected_sources.txt``（每行记明来自哪一帧、亮度多少 —— 事后归因用）。
源目录里若有 ``info.json`` 会一并复制过去（标注器要内参）。
"""
import argparse
import glob
import json
import os
import shutil
import sys

try:
    import numpy as np
    import cv2
except ImportError:                                     # pragma: no cover
    print("❌ 需要 numpy 和 opencv-python")
    sys.exit(2)

EXTS = (".png", ".jpg", ".jpeg")


def mean_brightness(gray):
    """整帧平均灰度。纯函数，自检用。"""
    return float(gray.mean())


def differs(a, b, frac=0.02, levels=25):
    """两帧是否有实质差异 —— 见模块开头的判据。

    差异像素 **严格多于** ``frac`` 比例才算不同（`>`，不是 `>=`）：
    正好卡在阈值上的一对应当合并，否则阈值本身会变成一个随机的分界线。
    """
    if a.shape != b.shape:
        raise ValueError("两张图尺寸不同：%r vs %r" % (a.shape, b.shape))
    n = int(np.count_nonzero(np.abs(a.astype(np.int16) - b.astype(np.int16)) > levels))
    return n > frac * a.size


def select(paths, gray_of, bmin=40.0, bmax=225.0, frac=0.02, levels=25, log=None):
    """贪心选帧。``gray_of(path)`` 返回灰度图；返回 ``[(path, mean, gray), ...]``。

    与 CLI 分开是为了让自检能塞假图进来 —— 不需要相机、不需要磁盘上真有一堆 jpg。
    """
    kept = []
    for p in paths:
        g = gray_of(p)
        if g is None:
            continue
        m = mean_brightness(g)
        if not (bmin < m < bmax):
            if log is not None:
                log.append(("skip-bright", p, m))
            continue
        if any(not differs(g, kg, frac, levels) for (_, _, kg) in kept):
            if log is not None:
                log.append(("skip-dup", p, m))
            continue
        kept.append((p, m, g))
        if log is not None:
            log.append(("keep", p, m))
    return kept


def main(argv=None):
    ap = argparse.ArgumentParser("从连拍里挑出互不相同的帧")
    ap.add_argument("dirs", nargs="+", help="源目录（可多个）")
    ap.add_argument("--out", required=True, help="输出目录")
    ap.add_argument("--pattern", default="img_*", help="文件名通配（默认 img_*）")
    ap.add_argument("--bmin", type=float, default=40.0, help="亮度下限（默认 40）")
    ap.add_argument("--bmax", type=float, default=225.0, help="亮度上限（默认 225）")
    ap.add_argument("--frac", type=float, default=0.02,
                    help="差异像素占比阈值（默认 2%%）")
    ap.add_argument("--levels", type=int, default=25,
                    help="单像素灰阶差阈值（默认 25）")
    ap.add_argument("--ext", default=None,
                    help="另存为这个扩展名（默认沿用源扩展名）")
    a = ap.parse_args(argv)

    paths = []
    for d in a.dirs:
        paths += glob.glob(os.path.join(d, a.pattern))
    # 全局排序（不是每个目录各自排）—— 让结果只取决于文件集合，
    # 不取决于命令行里目录的先后。贪心的挑选顺序见模块开头。
    paths = sorted(p for p in paths
                   if os.path.splitext(p)[1].lower() in EXTS)
    if not paths:
        print("❌ 一个都没匹配上：%s 里没有 %s" % (a.dirs, a.pattern))
        return 2

    cache = {}

    def gray_of(p):
        if p not in cache:
            cache[p] = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        return cache[p]

    log = []
    kept = select(paths, gray_of, a.bmin, a.bmax, a.frac, a.levels, log)

    os.makedirs(a.out, exist_ok=True)
    n_skip_b = sum(1 for e in log if e[0] == "skip-bright")
    n_skip_d = sum(1 for e in log if e[0] == "skip-dup")
    print("候选 %d 帧 → 亮度窗口外 %d、与已选帧重复 %d → **保留 %d 帧**"
          % (len(paths), n_skip_b, n_skip_d, len(kept)))

    lines = []
    for i, (p, m, _) in enumerate(kept):
        ext = a.ext or os.path.splitext(p)[1].lstrip(".")
        dst = os.path.join(a.out, "img_%04d.%s" % (i, ext))
        if os.path.exists(dst):
            os.remove(dst)
        if a.ext and a.ext.lower() != os.path.splitext(p)[1].lstrip(".").lower():
            cv2.imwrite(dst, cache[p])
        else:
            shutil.copy2(p, dst)
        lines.append("img_%04d.%s <- %s  mean=%.1f" % (i, ext, p, m))
    with open(os.path.join(a.out, "selected_sources.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # 标注器要内参。源目录里有就顺手带过去 —— 少一步手工 cp，也少一次抄错。
    for d in a.dirs:
        src = os.path.join(d, "info.json")
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(a.out, "info.json"))
            print("内参一并复制：%s" % src)
            break

    if not kept:
        print("⚠️ 一帧都没留下 —— 亮度窗口或差异阈值设得太紧了？")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
