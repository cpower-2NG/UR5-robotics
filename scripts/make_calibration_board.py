#!/usr/bin/env python3
"""生成**精确尺寸**的标定棋盘格 PDF（拿到打印店按 100% 出即可）。

为什么自己生成、而不是随手找一张图
────────────────────────────────────
棋盘格是整条标定链路的**尺度基准**。方格边长错 5%，外参就**整体**错 5%，
而且这个错**不会**被任何重投影误差暴露出来（重投影误差对尺度不敏感，
它会显示得很漂亮）。所以尺寸必须是**可知且可验**的：

  · 网上的图多半是位图；打印店默认「适合页面」会把它重新缩放，
    缩了多少**无从得知**。
  · 本脚本输出**矢量 PDF**，页面尺寸写死在文件里，不经过任何缩放环节。
  · 图面上**自带一条 100.0 mm 的校验尺**。印完拿尺子量它：
    量出 100 mm 就说明没缩放；量出 97 mm 就说明整体被缩到 97%，
    此时**单格真实边长 = 标称值 × 0.97** —— 这个数要喂进标定。

打印须知（写在图面上，也写在这里）
  · 选 **「实际大小」/「100%」**，**不要**选「适合页面 / 缩放以适应」。
  · **务必压平**：纸会翘，而**板不平会把误差直接灌进外参**。
    建议印 A3 后**贴到硬纸板/亚克力/画板上**，四边用胶带压住。
  · 不要覆膜（反光会让角点检测退化，也会毁掉深度）。

用法
────
    python3 scripts/make_calibration_board.py --paper A3 --square-mm 30 --cols 13 --rows 8
    python3 scripts/make_calibration_board.py --paper A4 --square-mm 20 --cols 9  --rows 13
    python3 scripts/make_calibration_board.py --paper A3 --square-mm 30 --cols 13 --rows 8 --preview

`--cols/--rows` 数的是**方格**数（不是内角点数）。OpenCV 要的
`patternSize` 是**内角点**数，即 `(cols-1, rows-1)` —— 脚本会把两个数都印在图上。
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# 图纸短边 × 长边（mm）。只列常打得到的。
PAPERS = {
    'A4': (210.0, 297.0),
    'A3': (297.0, 420.0),
    'A2': (420.0, 594.0),
}

MM_PER_INCH = 25.4


def sheet_size(paper, orient):
    short, long_ = PAPERS[paper]
    return (long_, short) if orient == 'landscape' else (short, long_)


def build_figure(width_mm, height_mm):
    """一张尺寸精确等于 (width_mm, height_mm) 的画布。

    关键三点，缺一就会得到错的物理尺寸：
      1. figsize 用英寸（figsize 的两个数之比必须等于 mm 之比）；
      2. 坐标轴铺满整张画布（add_axes([0,0,1,1])），不留 matplotlib 的边距；
      3. xlim/ylim 直接以 **mm 为单位**，于是「1 数据单位 = 1 mm」。
    因为 (1) 里两个数之比等于 (2) 里两个数之比，x/y 的缩放天生一致，
    **不需要** set_aspect('equal')（那个反而可能改动 axes 盒子的位置）。
    """
    fig = plt.figure(figsize=(width_mm / MM_PER_INCH, height_mm / MM_PER_INCH))
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_xlim(0.0, width_mm)
    ax.set_ylim(0.0, height_mm)
    ax.axis('off')
    return fig, ax


def draw_chessboard(ax, x0, y0, cols, rows, sq, first_is_black=True):
    """在 (x0, y0) 起画 cols×rows 个边长 sq(mm) 的方格。

    只填面、**不描边** —— 描边会让每个格子向外胖出半个线宽，
    整块图案的物理尺寸就不再等于 cols*sq。相邻黑格之间因此可能出现
    发丝级白缝，这是**有意的**：精度优先于观感。
    """
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == (0 if first_is_black else 1):
                ax.add_patch(Rectangle((x0 + c * sq, y0 + r * sq), sq, sq,
                                       facecolor='black', edgecolor='none',
                                       linewidth=0.0, antialiased=False))


def draw_scale_bar(ax, x0, y0, length_mm=100.0, tick_mm=10.0, height_mm=3.0):
    """一条**精确 length_mm 长**的校验尺，每 tick_mm 一个刻度。"""
    ax.add_patch(Rectangle((x0, y0), length_mm, height_mm,
                           facecolor='black', edgecolor='none', antialiased=False))
    n = int(round(length_mm / tick_mm))
    for i in range(n + 1):
        x = x0 + i * tick_mm
        h = height_mm * (2.0 if i % 5 == 0 else 1.4)
        ax.add_patch(Rectangle((x - 0.25, y0), 0.5, h,
                               facecolor='black', edgecolor='none', antialiased=False))
    for i in range(0, n + 1, 5):
        ax.text(x0 + i * tick_mm, y0 + height_mm * 2.4, '%d' % int(i * tick_mm),
                ha='center', va='bottom', fontsize=6)
    ax.text(x0 + length_mm / 2.0, y0 - 1.0, 'mm', ha='center', va='top', fontsize=6)
    return length_mm


def draw_frame(ax, x0, y0, w, h, lw_mm=0.3):
    """图案外框。只是为了目视对位；**任何读数都不靠它**。

    注意线宽以**点**为单位：mm → pt 是 ×72/25.4。线本身会以路径为中心
    向外扩半个线宽（0.15 mm），所以它**不是**一个可量的基准 —— 要量就量
    下面那条校验尺，那条是**实心矩形**，边界由几何决定，不含线宽。
    """
    ax.add_patch(Rectangle((x0, y0), w, h, facecolor='none', edgecolor='black',
                           linewidth=lw_mm * 72.0 / MM_PER_INCH))


def make(paper, orient, square_mm, cols, rows, margin_mm, out_pdf, preview=False):
    W, H = sheet_size(paper, orient)
    pat_w, pat_h = cols * square_mm, rows * square_mm

    # 底部要留出校验尺 + 说明文字的高度
    footer_mm = 34.0
    need_w = pat_w + 2 * margin_mm
    need_h = pat_h + margin_mm + footer_mm
    if need_w > W or need_h > H:
        sys.exit(
            '放不下：%s %s 是 %.0f×%.0f mm，而 %d 格 × %.1f mm = %.0f mm 宽、'
            '加上边距与页脚共需 %.0f×%.0f mm。\n'
            '  → 要么换大一号纸（--paper A2），要么减小 --square-mm，'
            '要么减少 --cols/--rows。'
            % (paper, orient, W, H, cols, square_mm, pat_w, need_w, need_h)
        )

    fig, ax = build_figure(W, H)

    x0 = (W - pat_w) / 2.0
    y0 = H - margin_mm - pat_h            # 图案顶到上边距
    draw_chessboard(ax, x0, y0, cols, rows, square_mm)
    draw_frame(ax, x0, y0, pat_w, pat_h)

    # ── 页脚 ────────────────────────────────────────────────────────
    # 图面文字用**英文**：本机没有任何 CJK 字体，用中文会印成豆腐块；
    # 而且标定靶本来就是国际惯例的英文，打印店不会看错。
    # 中文的打印须知在 docs/待执行命令.md 里，由人转述给打印店。
    fy = y0 - 8.0
    ax.text(x0, fy, 'CHESSBOARD CALIBRATION TARGET   |   square %.1f mm   |   '
                    '%d x %d squares   |   pattern %.1f x %.1f mm'
            % (square_mm, cols, rows, pat_w, pat_h), ha='left', va='top', fontsize=8)
    ax.text(x0, fy - 5.0,
            'OpenCV patternSize (INNER CORNERS) = (%d, %d), total %d   |   '
            'sheet %s %s %.0f x %.0f mm'
            % (cols - 1, rows - 1, (cols - 1) * (rows - 1), paper, orient, W, H),
            ha='left', va='top', fontsize=8)
    ax.text(x0, fy - 10.0,
            'PRINT AT 100% / ACTUAL SIZE.  Do NOT use "fit to page" or any scaling.',
            ha='left', va='top', fontsize=8, fontweight='bold')
    ax.text(x0, fy - 15.0,
            'Mount on a rigid flat board (paper curl goes straight into the '
            'extrinsics). Do not laminate.', ha='left', va='top', fontsize=8)
    ax.text(x0, fy - 20.0,
            'VERIFY THE SCALE: the bar below must measure exactly 100.0 mm.',
            ha='left', va='top', fontsize=8)
    draw_scale_bar(ax, x0, fy - 32.0)

    fig.savefig(out_pdf, format='pdf')
    outs = [out_pdf]
    if preview:
        png = os.path.splitext(out_pdf)[0] + '.png'
        fig.savefig(png, format='png', dpi=110)
        outs.append(png)
    plt.close(fig)

    # ── 回读 PDF 页面尺寸自检：页面必须**逐字**等于标称值 ──────────
    with open(out_pdf, 'rb') as fh:
        head = fh.read(4096)
    box = None
    i = head.find(b'/MediaBox')
    if i >= 0:
        j = head.find(b']', i)
        box = [float(t) for t in head[i + 9:j].replace(b'[', b' ').split()]
    if box and len(box) == 4:
        w_pt, h_pt = box[2] - box[0], box[3] - box[1]
        got = (w_pt * MM_PER_INCH / 72.0, h_pt * MM_PER_INCH / 72.0)
        ok = abs(got[0] - W) < 0.2 and abs(got[1] - H) < 0.2
        print('PDF 页面 %.2f × %.2f mm  （标称 %.0f × %.0f）%s'
              % (got[0], got[1], W, H, 'OK' if ok else '  ← 不符，别用'))
        if not ok:
            sys.exit(1)
    else:
        print('自检：没读到 /MediaBox，请手工确认页面尺寸')
    return outs


def main():
    p = argparse.ArgumentParser(description='生成精确尺寸的棋盘格标定板 PDF',
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--paper', default='A3', choices=sorted(PAPERS))
    p.add_argument('--orient', default='landscape', choices=['landscape', 'portrait'])
    p.add_argument('--square-mm', type=float, default=30.0, help='每个方格的边长')
    p.add_argument('--cols', type=int, default=13, help='方格列数（不是内角点数）')
    p.add_argument('--rows', type=int, default=8, help='方格行数（不是内角点数）')
    p.add_argument('--margin-mm', type=float, default=15.0)
    p.add_argument('--out', default=None)
    p.add_argument('--preview', action='store_true', help='同时输出 PNG 预览')
    a = p.parse_args()

    out = a.out or 'calib_board_%s_%gmm_%dx%d.pdf' % (
        a.paper, a.square_mm, a.cols, a.rows)
    outs = make(a.paper, a.orient, a.square_mm, a.cols, a.rows,
                a.margin_mm, out, a.preview)
    print('图案 %.1f × %.1f mm；内角点 (%d, %d)' %
          (a.cols * a.square_mm, a.rows * a.square_mm, a.cols - 1, a.rows - 1))
    for o in outs:
        print('  →', os.path.abspath(o))


if __name__ == '__main__':
    main()
