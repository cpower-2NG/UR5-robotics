#!/usr/bin/env python3
"""生成一张**积木边长比对尺** PDF：把积木往方框上一放，哪个框正好就是几毫米。

为什么不是再打一张棋盘格
────────────────────────
棋盘格**没坏**，它在这件事上只是不合适，理由有三条，都是这一轮实测出来的：

  1. **它的格子跟被测物一个量级。** 现有那块是 22.0 mm/格，而积木就在 30 mm 上下。
     拿一个 22 去量一个 30，比例误差直接放大。
  2. **印出来的黑格≠格子节距。** 阈值分割量到的是**墨迹**的大小，而墨迹边缘是
     渐变的、还带纸面反光。实测：黑格 37~39 px，按台面网格折算 **24.5~25.1 mm**，
     而格子节距的真值是 22.0 —— 光这一项就 **+12%**。
  3. **角点检测在它上面不工作**（反光 + 黑白块小），`cv2.findChessboardCorners`
     试了 8 组 patternSize 一个都没过。标定要角点，量边长不要。

所以这张尺子换了做法：**画的是方框的内沿**，标称多少毫米，内沿就是多少毫米
（线宽往**外**让，见 `draw_size_square`）。把积木嵌进去看合不合 ——
比的是积木与**内沿**，不受墨迹宽窄影响。

打印与使用
──────────
  · **按 100%（实际大小）打，别选「适合页面」。**
  · 打完拿尺子量下面那条 **100.0 mm** 校验尺。量出 97 就说明整张缩到 97%，
    此时**每个框的真值 = 标称 × 0.97**，这个数要告诉我。
  · 用法：把积木**平放在框里**（或紧贴框放在纸旁边），看哪个框正好。
    框比积木大 ⇒ 四边露缝；框比积木小 ⇒ 积木压住线。相邻框只差 2 mm，
    肉眼能分辨。
  · 也可以**拍一张照**给我 —— 方框和积木在**同一个平面**上，我可以用同一个
    标定链把两者一起量出来，比上次对着棋盘格猜干净得多。

图上文字一律英文：这台开发机上**没有中文字体**（`fc-list :lang=zh` 为 0），
中文会渲染成豆腐块。
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_calibration_board import build_figure, draw_scale_bar, MM_PER_INCH  # noqa: E402

A4 = (210.0, 297.0)


def draw_size_square(ax, x, y, side_mm, lw_mm=0.8, label=True, color='black'):
    """在左下角 (x, y) 画一个**内沿恰好是 side_mm** 的方框。

    ⚠️ 这是整张纸唯一要紧的细节。matplotlib 的线是以路径为**中心**向两侧各铺
    lw/2 的，所以「画个边长 side 的矩形」得到的内沿只有 `side - lw`，
    比标称小了一个线宽 —— 0.8 mm 就是 2.7% 的误差，而相邻框只差 2 mm，
    足以让判断整个错位。所以路径要放大到 `side + lw`、并把原点外移 lw/2，
    这样内沿才**逐字**落在 [x, x+side] 上。
    """
    ax.add_patch(Rectangle((x - lw_mm / 2.0, y - lw_mm / 2.0),
                           side_mm + lw_mm, side_mm + lw_mm,
                           facecolor='none', edgecolor=color,
                           linewidth=lw_mm * 72.0 / MM_PER_INCH,
                           antialiased=False))
    if label:
        ax.text(x + side_mm / 2.0, y - 2.0, '%g mm' % side_mm,
                ha='center', va='top', fontsize=7, color=color)


def verify_inner_edges(sizes, dpi=600, tol_mm=0.15):
    """把同一张图**光栅化**，量出每个方框的**内沿**，与标称比。

    为什么值得单独验一次：这张纸唯一要紧的细节就是「内沿 = 标称」，
    而它恰恰是最容易写错的一处（线是以路径为中心铺的，画 `side` 得到的是
    `side - lw`）。这个错**看不出来** —— 图上就是几个方框，谁也不会拿尺子
    去量一个还没打印的 PDF。所以让程序自己量。

    只测最大的那个框：线宽相对框边长最大，误差若存在，它最容易超差。
    """
    import numpy as np
    from PIL import Image

    big = max(sizes)
    tmp = '/tmp/_block_size_check_verify.png'
    make(sizes, tmp, preview=False, _verify_render=True)
    img = Image.open(tmp).convert('L')
    a = np.asarray(img)
    px_per_mm = dpi / MM_PER_INCH
    dark = a < 128
    import cv2
    n, lab, stats, _ = cv2.connectedComponentsWithStats(
        dark.astype(np.uint8), connectivity=8)
    best = None
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if w < 0.5 * big * px_per_mm or not (0.9 < w / float(h) < 1.1):
            continue
        if best is None or area > best[4]:
            best = (x, y, w, h, area)
    if best is None:
        print('内沿自检：没找到方框')
        return False
    x, y, w, h, _ = best
    # 取框的水平中线，扫出中间那段白（= 内沿宽度）
    mid = a[y + h // 2, x:x + w]
    run = np.where(mid >= 128)[0]
    inner_w = (run.max() - run.min() + 1) / px_per_mm if len(run) else 0.0
    midc = a[y:y + h, x + w // 2]
    runc = np.where(midc >= 128)[0]
    inner_h = (runc.max() - runc.min() + 1) / px_per_mm if len(runc) else 0.0
    ok = abs(inner_w - big) < tol_mm and abs(inner_h - big) < tol_mm
    print('内沿自检（%d dpi 光栅化）：最大框标称 %g mm，量到 %.2f × %.2f mm  %s'
          % (dpi, big, inner_w, inner_h, 'OK' if ok else '  ← 超差'))
    return ok


def make(sizes, out_pdf, preview=False, _verify_render=False):
    W, H = A4
    fig, ax = build_figure(W, H)
    m = 15.0
    x0 = m

    ax.text(x0, H - 14.0, 'Block Edge Check Sheet', ha='left', va='top',
            fontsize=15, fontweight='bold')
    ax.text(x0, H - 22.0,
            'Print at 100% / ACTUAL SIZE.  Do NOT use "fit to page" or any scaling.',
            ha='left', va='top', fontsize=9, fontweight='bold')
    ax.text(x0, H - 28.0,
            'Lay a block flat on each outline. The frame whose INNER edge the block '
            'fills exactly is its edge length.',
            ha='left', va='top', fontsize=8)
    ax.text(x0, H - 33.0,
            'Frame too big -> gaps at the sides.  Frame too small -> the block covers '
            'the line.  Neighbours differ by 2 mm.',
            ha='left', va='top', fontsize=8)

    # ── 方框阵列：从左下角起一行行摆，行内相邻只留 gap ────────────────
    gap = 9.0
    y = H - 80.0
    row = []
    rows = []
    used = 0.0
    for s in sizes:
        need = s if not row else s
        if used + need + (gap if row else 0.0) > (W - 2 * m):
            rows.append(row)
            row, used = [s], s
        else:
            used += need + (gap if row else 0.0)
            row.append(s)
    rows.append(row)

    for r in rows:
        x = x0
        tallest = max(r)
        for s in r:
            draw_size_square(ax, x, y, s)
            x += s + gap
        y -= tallest + 16.0

    # ── 校验尺 ────────────────────────────────────────────────────────
    # 校验尺的数字标在**尺子上方**（draw_scale_bar 里 y0 + height*2.4），
    # 所以标题要再抬高一点，否则「100」和标题末尾会叠在一起（第一版就叠了）。
    ax.text(x0, y + 8.0, 'VERIFY THE SCALE: this bar must measure exactly 100.0 mm.',
            ha='left', va='top', fontsize=8, fontweight='bold')
    draw_scale_bar(ax, x0, y - 6.0)

    ax.text(x0, y - 26.0,
            'If the bar measures 97 mm the sheet is at 97%, and every frame above is '
            '0.97 x its label.',
            ha='left', va='top', fontsize=8)
    ax.text(x0, y - 31.0,
            'If it measures 100 mm, the labels are the true sizes.',
            ha='left', va='top', fontsize=8)
    ax.text(x0, y - 38.0,
            'Sheet %.0f x %.0f mm, frames drawn by their INNER edge (line width grows '
            'outward).' % (W, H),
            ha='left', va='top', fontsize=7)

    if _verify_render:
        fig.savefig(out_pdf, format='png', dpi=600)
        plt.close(fig)
        return [out_pdf]

    fig.savefig(out_pdf, format='pdf')
    outs = [out_pdf]
    if preview:
        png = os.path.splitext(out_pdf)[0] + '.png'
        fig.savefig(png, format='png', dpi=110)
        outs.append(png)
    plt.close(fig)

    # ── 回读页面尺寸：必须逐字等于 A4 ─────────────────────────────────
    with open(out_pdf, 'rb') as fh:
        head = fh.read(4096)
    box = None
    i = head.find(b'/MediaBox')
    if i >= 0:
        j = head.find(b']', i)
        box = [float(t) for t in head[i + 9:j].replace(b'[', b' ').split()]
    if box and len(box) == 4:
        got = ((box[2] - box[0]) * MM_PER_INCH / 72.0,
               (box[3] - box[1]) * MM_PER_INCH / 72.0)
        ok = abs(got[0] - W) < 0.2 and abs(got[1] - H) < 0.2
        print('PDF 页面 %.2f × %.2f mm（标称 %.0f × %.0f）%s'
              % (got[0], got[1], W, H, 'OK' if ok else '  ← 不符，别用'))
        if not ok:
            sys.exit(1)
    else:
        print('自检：没读到 /MediaBox，请手工确认页面尺寸')
    return outs


def main():
    p = argparse.ArgumentParser(
        description='生成积木边长比对尺 PDF',
        epilog='内沿自检是默认打开的：它量的是这张纸唯一要紧的那个尺寸，'
               '而且那个尺寸错了肉眼看不出来。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--sizes', default='24,26,28,30,32,34,36,38,40',
                   help='方框内沿边长（mm），逗号分隔')
    p.add_argument('--out', default=None)
    p.add_argument('--preview', action='store_true')
    a = p.parse_args()
    sizes = [float(t) for t in a.sizes.split(',') if t.strip()]
    out = a.out or 'block_size_check_A4.pdf'
    for o in make(sizes, out, a.preview):
        print('  →', os.path.abspath(o))
    print('  框内沿：%s mm' % ', '.join('%g' % s for s in sizes))
    if not verify_inner_edges(sizes):
        sys.exit(1)


if __name__ == '__main__':
    main()
