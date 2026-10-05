#!/usr/bin/env python3
"""量「现场到底有几种颜色的积木」—— 用来定类别数（**纯离线看图，不连机器人**）。

为什么需要它
============
A2 分拣的类别 = **颜色**，而类别数不是拍脑袋定的：它同时受颜色可分性与
台面槽位数两个约束夹着（见 ``block_classes.py`` 的模块 docstring）。
前两次估类别数的分析**都错了** —— 错在把「同一种颜色在不同光照下漂移」
与「真的有好几种颜色」混成一个量。分开量之后结论才稳：

    同一张照片内部的暖色展开中位 **49.3°** ≫ 跨照片漂移 **3.4°**
    ⇒ 暖色那一大团是真有好几种，不是光照漂移。

所以这把尺子的核心不是「数出几个峰」，而是**同时打出这两个数**：

  · **图内展开**（span）—— 一团颜色在一张图里有多宽；
  · **跨图漂移**（drift）—— 同一个色团在不同照片里挪了多少。

判据：**展开 ≫ 漂移 ⇒ 里面藏着多种颜色，该拆；展开 ≈ 漂移 ⇒ 是一种，别拆。**

⚠️ 之前量的是**前负责人那批积木**的数据集。用户手上的积木还没量过 ——
把全部积木摊在俯视相机下拍一张（或几张，越分散越好），跑这个脚本。

怎么拍才算数
============
| # | 要求 | 为什么 |
|---|---|---|
| 1 | **每块之间留出一道缝**（≳ 半块宽） | 贴一起会让边缘混色，色相被拉向中间值 |
| 2 | **一块都别叠** | 阴影会把明度压下去，饱和像素占比掉，权重跟着掉 |
| 3 | **别只拍一遍就完** | 只有一张图量不出「跨图漂移」，那一栏会是空的 |
| 4 | **尽量铺满画面** | 每个色团的像素越多，直方图越不像噪声 |
| 5 | **别开闪光灯、别加滤镜** | 白平衡一变，色相整体平移，量出来的数就不是现场的数了 |

怎么算通过
==========
脚本自己给结论：把每个色团按「展开 vs 漂移」判一遍，再打出可直接粘进
生成器的 ``--classes`` 一行。**看到「建议类别数 = N」就是通过了**；
若它说某两类「只差 X°，余量太小」，就说明那两类该合成一类。

用法::

    python3 scripts/palette_survey.py --images ~/block_photos
    python3 scripts/palette_survey.py --images a.jpg b.jpg --out /tmp/palette.png
    python3 scripts/palette_survey.py --images ~/block_photos --min-share 0.05

退出码：0 = 量出来了；2 = 环境不够（没 cv2 / 一张图都读不进来）。
"""

import argparse
import os
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                # 与其它脚本同一套 import 约定
import block_classes                    # noqa: E402

import numpy as np                      # noqa: E402

# 色相直方图的分辨率：360 个 1° 的格子。
# 为什么是 1° 而不是 2° 或 5°：库里那批同色系积木的实测相邻间隔约 20°，
# 而跨轮次漂移约 3.4° —— 1° 的桶既能分辨 3.4° 的漂移，又不会把噪声当峰。
NBINS = 360

# 判「这一团是不是真的多种颜色」用的漂移基准。
# 实测值（前负责人那批数据）：0.945°/14 h 是**外参**漂移，与此无关；
# 这里指的是同一批积木、不同照片之间的色相漂移，实测 **3.4°**。
# 给到 2 倍余量，作为「展开 ≫ 漂移」里的那个「≫」。
DRIFT_REF_DEG = 3.4


# ----------------------------------------------------------------------
# 纯计算部分：输入 numpy 数组，输出字典 —— 不碰文件、不画图，好测。
# ----------------------------------------------------------------------
def colorful_mask(hsv, s_min, v_min, v_max):
    """挑出「有颜色且曝光正常」的像素。

    三道门各挡一类假信号：
      · ``s >= s_min`` —— 挡灰白背景（白纸、木桌面、金属反光）。
        实测白纸与米色方块 S≈24/27，所以 ``s_min`` 的默认值必须**高于 24**，
        否则背景会混进来而且因为它面积最大，直接变成一个「类别」。
      · ``v >= v_min`` —— 挡黑影与阴影里的缝。
      · ``v <= v_max`` —— 挡高光过曝（过曝处色相会被压向白，读数不可信）。
    """
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = (s >= s_min) & (v >= v_min) & (v <= v_max)
    return mask, s, v


def hue_histogram(hsv, s_min=60, v_min=40, v_max=250):
    """色相加权直方图。**权重 = 饱和度**，不是每像素计 1。

    为什么按饱和度加权：一块积木的高光边缘、阴影侧、反光点都在同一个色相附近，
    但它们**不代表**这块积木的颜色。饱和像素才是「它是什么颜色」的证据。
    不加权的话，一块高光的红积木会把自己拉向橙色。
    """
    mask, s, _v = colorful_mask(hsv, s_min, v_min, v_max)
    if not mask.any():
        return np.zeros(NBINS), 0.0
    hues = hsv[..., 0][mask].astype(np.float64)       # OpenCV: H ∈ [0,180)
    weights = s[mask].astype(np.float64)
    hist, _ = np.histogram(hues * 2.0, bins=NBINS, range=(0.0, 360.0),
                           weights=weights)
    return hist, float(weights.sum())


def _circ_dist(a, b):
    """两个角度（度）之间的最短弧距，落在 [0,180]。"""
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


def _circ_smooth(hist, win):
    """环形滑动平均 —— 直方图的两个端点（红在 0°/360°）必须接得上。"""
    if win <= 1:
        return hist
    k = np.ones(win) / float(win)
    ext = np.concatenate([hist[-win:], hist, hist[:win]])
    return np.convolve(ext, k, mode='same')[win:win + NBINS]


def find_modes(hist, smooth_deg=5, merge_deg=25, min_share=0.03):
    """从色相直方图里找出几个**色团**，返回 [(中心色相, 权重占比), ...]。

    三步，每步都对应一个已知的失效方式：

      1. **环形平滑** —— 不平滑的话，一个平顶的色团会被切成一堆「峰」。
         窗口就是 ``smooth_deg``，默认 5°（远小于 20° 的类别间隔）。
      2. **合并** —— 相隔小于 ``merge_deg`` 的峰是同一个色团（积木表面不均匀、
         光照渐变都会造出假双峰）。默认 25°：**这是「两类到底算不算两类」的
         唯一旋钮**，调大 = 更容易合并 = 类别更少。
      3. **按占比砍** —— 占比小于 ``min_share`` 的当噪声丢掉。默认 3%：
         一张铺满的图里，最小的那一类也该远大于 3%，而噪声峰远小于它。
    """
    total = float(np.sum(hist))
    if total <= 0:
        return []
    sm = _circ_smooth(hist, max(1, int(round(smooth_deg))))
    if float(np.sum(sm)) <= 0:
        return []
    sm = sm / float(np.sum(sm)) * total

    # 局部极大（环形），并去掉低于噪声底的格子
    peak_hi = np.roll(sm, -1)
    peak_lo = np.roll(sm, 1)
    is_peak = (sm >= peak_hi) & (sm >= peak_lo) & (sm > total / NBINS)
    peaks = [(float(i), float(sm[i])) for i in np.nonzero(is_peak)[0]]
    if not peaks:
        # 完全平坦的直方图（例如只有一种颜色但曝光均匀）→ 取加权重心
        idx = np.arange(NBINS, dtype=np.float64)
        return [(float((idx * sm).sum() / sm.sum()), total)]

    # 从重到轻贪心合并
    peaks.sort(key=lambda p: -p[1])
    kept = []
    for c, m in peaks:
        if all(_circ_dist(c, kc) >= merge_deg for kc, _ in kept):
            kept.append((c, m))

    # 用「就近归属」把每个格子划给某个色团，再用加权重心把中心磨一遍
    idx = np.arange(NBINS, dtype=np.float64)
    for _ in range(3):
        if not kept:
            break
        centers = np.array([c for c, _ in kept])
        d = np.abs(idx[:, None] - centers[None, :])
        d = np.minimum(d, 360.0 - d)
        owner = np.argmin(d, axis=1)
        new = []
        for j, (c, _m) in enumerate(kept):
            sel = (owner == j)
            w = sm[sel]
            if w.sum() <= 0:
                new.append((c, 0.0))
                continue
            # 环形加权重心：先平移到以 c 为中心，再取平均
            ang = ((idx[sel] - c + 180.0) % 360.0) - 180.0
            new.append((float((c + (ang * w).sum() / w.sum()) % 360.0),
                        float(w.sum())))
        kept = new

    modes = []
    for c, m in kept:
        share = m / total
        if share < min_share:
            continue
        modes.append({'hue': c, 'share': share, 'weight': m,
                      'lo': 0.0, 'hi': 0.0})
    modes.sort(key=lambda x: x['hue'])
    return modes


def mode_spans(hist, modes):
    """给每个色团量出**图内展开**（5~95 分位，环形）。

    用分位而不是标准差：展开的分布是双峰的（一种颜色 + 几种颜色两种情况），
    标准差会被少数离群像素拽大，分位不会。
    """
    total = float(np.sum(hist))
    if total <= 0 or not modes:
        return modes
    idx = np.arange(NBINS, dtype=np.float64)
    centers = np.array([m['hue'] for m in modes])
    d = np.abs(idx[:, None] - centers[None, :])
    d = np.minimum(d, 360.0 - d)
    owner = np.argmin(d, axis=1)
    for j, m in enumerate(modes):
        sel = (owner == j) & (hist > 0)
        if not sel.any():
            continue
        ang = ((idx[sel] - m['hue'] + 180.0) % 360.0) - 180.0
        w = hist[sel]
        order = np.argsort(ang)
        a, ww = ang[order], w[order]
        cum = np.cumsum(ww) / ww.sum()
        lo = float(a[np.searchsorted(cum, 0.05)])
        hi = float(a[min(len(a) - 1, np.searchsorted(cum, 0.95))])
        m['lo'], m['hi'] = lo, hi
        m['span'] = hi - lo
    return modes


def palette_hue(name):
    """调色板里的 RGB → 色相（度）。用标准 RGB→HSV，与 cv2 同一定义。"""
    import colorsys
    r, g, b = block_classes.color_of(name)
    h, _s, _v = colorsys.rgb_to_hsv(r, g, b)
    return h * 360.0


def nearest_class(hue, names=None):
    """色相 → 最接近的调色板名字，附带差多少度。

    ⚠️ 这只是**取名字**，不是判据。真正定类别数的是上面展开/漂移那两个数；
    这一步只负责把「色相 28°」翻译成「orange」。差得远（> ``--name-tol``）
    说明现场那个颜色调色板里没有，该往 ``PALETTE`` 里加一个。
    """
    names = list(names or block_classes.PALETTE.keys())
    best, best_d = None, 1e9
    for n in names:
        d = _circ_dist(hue, palette_hue(n))
        if d < best_d:
            best, best_d = n, d
    return best, best_d


def survey(hsv_list, s_min=60, v_min=40, v_max=250,
           smooth_deg=5, merge_deg=25, min_share=0.03):
    """把若干张图的 HSV 合成一次勘察，返回一部字典（``--json`` 直接吐它）。

    ``hsv_list`` 里每一项是一张图的 HSV 数组 —— 只有一张时「跨图漂移」是 None，
    这不是错误，是**量不出来**，报告里会明说。
    """
    hists, totals = [], []
    for hsv in hsv_list:
        h, t = hue_histogram(hsv, s_min, v_min, v_max)
        hists.append(h)
        totals.append(t)
    agg = np.sum(np.array(hists), axis=0) if hists else np.zeros(NBINS)
    modes = find_modes(agg, smooth_deg, merge_deg, min_share)
    modes = mode_spans(agg, modes)
    for m in modes:
        m['name'], m['name_delta'] = nearest_class(m['hue'])

    # 跨图漂移：每个色团在各图里的中心色相，取标准差
    for m in modes:
        per = []
        for hsv in hsv_list:
            _h, s, v = colorful_mask(hsv, s_min, v_min, v_max)
            mask = (s >= s_min) & (v >= v_min) & (v <= v_max)
            if not mask.any():
                continue
            hues = hsv[..., 0][mask].astype(np.float64) * 2.0
            w = s[mask].astype(np.float64)
            # 只收「属于这个色团」的像素：到该中心的弧距在一半合并窗口内
            keep = np.array([_circ_dist(x, m['hue']) <= merge_deg / 2.0
                             for x in hues])
            if not keep.any() or w[keep].sum() <= 0:
                continue
            ang = ((hues[keep] - m['hue'] + 180.0) % 360.0) - 180.0
            order = np.argsort(ang)
            a, ww = ang[order], w[keep][order]
            cum = np.cumsum(ww) / ww.sum()
            per.append(float((m['hue'] + a[np.searchsorted(cum, 0.5)]) % 360.0))
        m['per_image'] = per
        # ⚠️ **环形**标准差，不是 np.std(per)。
        # 红色正好跨在 0°/360° 上：三张同色的图会给出 0.5° / 358.7° / 1.2°，
        # 线性标准差算出来是 **168.8°** —— 一个纯粹由「度数的零点在哪」造成的
        # 天文数字，而报告会一本正经地拿它去比展开。
        # 做法：先把每张图的中心平移到以该色团中心为原点（折进 ±180），再取标准差。
        if len(per) >= 2:
            dev = [((x - m['hue'] + 180.0) % 360.0) - 180.0 for x in per]
            m['drift'] = float(np.std(dev))
        else:
            m['drift'] = None

    # 两两之间够不够远 —— 这是「能不能当两类」的直接判据
    for i, a in enumerate(modes):
        a['nearest_gap'] = min([_circ_dist(a['hue'], b['hue'])
                                for j, b in enumerate(modes) if j != i],
                               default=None)

    return {'n_images': len(hsv_list),
            'colorful_px_weight': float(sum(totals)),
            'modes': modes}


# ----------------------------------------------------------------------
# 结论与呈现
# ----------------------------------------------------------------------
def verdict_lines(res, name_tol=25.0, gap_warn=20.0):
    """把上面的数翻成能读的结论。**这一节就是「怎么算通过」的判据本身。**"""
    out = []
    modes = res['modes']
    out.append('量的图：%d 张' % res['n_images'])
    out.append('')
    out.append('  #  中心色相   图内展开   跨图漂移   占比     最接近的名字')
    for i, m in enumerate(modes):
        drift = '  ——  ' if m['drift'] is None else '%5.1f°' % m['drift']
        span = m.get('span', 0.0)
        out.append('  %d  %6.1f°    %5.1f°     %s   %4.1f%%    %s (±%.0f°)'
                   % (i, m['hue'], span, drift, m['share'] * 100.0,
                      m['name'], m['name_delta']))
    out.append('')

    # 逐个色团判「一种还是多种」
    for i, m in enumerate(modes):
        d = m['drift']
        span = m.get('span', 0.0)
        if d is None:
            out.append('  · 色团 %d（%.0f°）：只有一张图，跨图漂移量不出来 ——'
                       ' 想判它是不是多种颜色，**再拍一张**。'
                       % (i, m['hue']))
            continue
        if span > max(3.0 * d, DRIFT_REF_DEG * 3.0):
            out.append('  · 色团 %d（%.0f°）：图内展开 %.1f° ≫ 跨图漂移 %.1f°'
                       ' ⇒ **里面有好几种颜色，该拆**。'
                       % (i, m['hue'], span, d))
        else:
            out.append('  · 色团 %d（%.0f°）：图内展开 %.1f° ≈ 跨图漂移 %.1f°'
                       ' ⇒ 是一种颜色（展开就是光照造成的）。'
                       % (i, m['hue'], span, d))
    out.append('')

    # 名字取不出来的
    off = [m for m in modes if m['name_delta'] > name_tol]
    for m in off:
        out.append('  ⚠️ 色团 %.0f° 离调色板里最近的 %s 还差 %.0f°（> %.0f°）——'
                   ' 调色板里没有这个颜色，要往 block_classes.PALETTE 里加一个。'
                   % (m['hue'], m['name'], m['name_delta'], name_tol))

    # 两类挨太近
    seen = set()
    for i, m in enumerate(modes):
        gap = m['nearest_gap']
        if gap is None or gap >= gap_warn:
            continue
        j = min((k for k in range(len(modes)) if k != i),
                key=lambda k: _circ_dist(m['hue'], modes[k]['hue']))
        pair = (min(i, j), max(i, j))
        if pair in seen:
            continue
        seen.add(pair)
        out.append('  ⚠️ 第 %d 类与第 %d 类只差 %.1f°（< %.0f°）——'
                   ' 实测跨轮次漂移约 %.1f°，余量只剩 %.1f 倍。'
                   '要么把它们合成一类（调大 --merge-deg），'
                   '要么改动现场光照后再量。'
                   % (pair[0], pair[1], gap, gap_warn, DRIFT_REF_DEG,
                      gap / DRIFT_REF_DEG))

    names = [m['name'] for m in modes]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        out.append('  ⚠️ 有两个色团都被叫成 %s —— 这说明**它们是两种不同的颜色，'
                   '但调色板里只有一个名字**。加名字，别硬塞。' % ", ".join(dup))
    out.append('')
    out.append('  建议类别数 = **%d**' % len(modes))
    out.append('  可直接用：--classes %s' % ",".join(names))
    return out


def load_images(paths):
    """把文件/目录展开成 (路径, BGR) 列表。读不进来的**记下来**，不静默跳过。"""
    import cv2
    EXTS = ('.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff')
    files = []
    for p in paths:
        if os.path.isdir(p):
            for fn in sorted(os.listdir(p)):
                if os.path.splitext(fn)[1].lower() in EXTS:
                    files.append(os.path.join(p, fn))
        else:
            files.append(p)
    out, bad = [], []
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_COLOR)
        if img is None:
            bad.append(f)
            continue
        # 两份都留着：HSV 用来算，BGR 用来画复核图（画 HSV 出来是错的）
        out.append((f, img, cv2.cvtColor(img, cv2.COLOR_BGR2HSV)))
    return out, bad


def draw(res, bgr_first, hsv_list, out_path, s_min=60, v_min=40, v_max=250):
    """画一张复核图：原图 + 色相直方图 + 色团标记。

    为什么一定要出图：**数字说不出画面里那个东西是什么**。若直方图上多出一个
    色团，只有看图才知道那是积木、还是旁边的水果/瓶盖/急停按钮。
    """
    try:
        import matplotlib
        matplotlib.use('Agg')           # 无头，不碰 DISPLAY
        import matplotlib.pyplot as plt
    except ImportError:
        print('（没装 matplotlib，跳过画图；上面的数字仍然有效）')
        return None
    n = len(res['modes'])
    fig, axes = plt.subplots(2, 1, figsize=(9, 8))
    ax = axes[0]
    ax.imshow(bgr_first[:, :, ::-1])    # 画 BGR，不是 HSV（画 HSV 出来是错的）
    ax.set_title('first image')
    ax.axis('off')
    ax = axes[1]
    # 与结论用**同一份**聚合直方图（所有图相加），否则图与数字对不上
    parts = [hue_histogram(h, s_min, v_min, v_max)[0] for h in hsv_list]
    h = np.sum(np.array(parts), axis=0) if parts else np.zeros(NBINS)
    ax.bar(np.arange(NBINS), h, width=1.0, color='#888')
    for i, m in enumerate(res['modes']):
        ax.axvline(m['hue'], color='#c0392b', lw=1.5)
        ax.text(m['hue'], ax.get_ylim()[1] * (0.95 - 0.08 * (i % 6)),
                '%d %s' % (i, m['name']), color='#c0392b', fontsize=9,
                rotation=90, va='top')
    ax.set_xlim(0, 360)
    ax.set_xlabel('hue (deg)   red=0/360   cyan~190   purple~300')
    ax.set_ylabel('saturation-weighted px')
    ax.set_title('%d modes' % n)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser(
        description='量现场积木的颜色种类数（离线看图，不连机器人）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('用法::')[-1])
    ap.add_argument('--images', '-i', nargs='+', required=True,
                    help='图片路径，或装了图片的目录（可给多个）')
    ap.add_argument('--s-min', type=int, default=60,
                    help='饱和度下限（默认 60；白纸/木桌面约 24，必须高于它）')
    ap.add_argument('--v-min', type=int, default=40, help='明度下限（默认 40）')
    ap.add_argument('--v-max', type=int, default=250,
                    help='明度上限，挡过曝（默认 250）')
    ap.add_argument('--smooth-deg', type=int, default=5,
                    help='直方图环形平滑窗口（度，默认 5）')
    ap.add_argument('--merge-deg', type=float, default=25.0,
                    help='相隔多少度以内算同一个色团（默认 25）'
                         '—— 这是「算不算两类」的唯一旋钮')
    ap.add_argument('--min-share', type=float, default=0.03,
                    help='色团最小占比，低于它当噪声（默认 0.03）')
    ap.add_argument('--name-tol', type=float, default=25.0,
                    help='与调色板名字的最大容差（度，默认 25）')
    ap.add_argument('--gap-warn', type=float, default=20.0,
                    help='两类间隔小于它就给告警（度，默认 20）')
    ap.add_argument('--out', default=None, help='复核图输出路径（默认不写）')
    args = ap.parse_args()

    try:
        import cv2  # noqa: F401
    except ImportError as e:
        print('需要 opencv（conda activate <robot-vision 环境>）：%s（退 2）' % e)
        return 2

    loaded, bad = load_images(args.images)
    for b in bad:
        print('读不进来（跳过，不算数）：%s' % b)
    if not loaded:
        print('一张图都没读进来。（退 2）')
        return 2

    images = [hsv for _f, _bgr, hsv in loaded]
    res = survey(images, args.s_min, args.v_min, args.v_max,
                 args.smooth_deg, args.merge_deg, args.min_share)
    res['files'] = [f for f, _b, _h in loaded]

    print('=' * 68)
    for line in verdict_lines(res, args.name_tol, args.gap_warn):
        print(line)
    print('=' * 68)
    if res['colorful_px_weight'] <= 0:
        print('⚠️ 一个「有颜色」的像素都没挑出来 —— 阈值太严，或者图里真没东西。')

    if args.out:
        p = draw(res, loaded[0][1], images, args.out,
                 args.s_min, args.v_min, args.v_max)
        if p:
            print('复核图：%s   ← **打开看一眼**，确认每个色团都是积木' % p)
    else:
        print('（没给 --out，没出复核图。强烈建议出一张：'
              '多出来的色团是什么，只有看图才知道）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
