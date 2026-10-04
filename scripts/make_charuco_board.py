#!/usr/bin/env python3
"""生成**精确尺寸**的 ChArUco 标定板 PDF（打印店按 100% 出）。

为什么是 ChArUco 而不是棋盘格
─────────────────────────────
棋盘格必须**整块可见**才能检测：被积木压住一角、或有一角出画面，整次检测就废。
本项目要把标记**常驻**在工作台上（周围就是积木），所以需要一种「局部可见也能定位」
的靶：ChArUco 在棋盘格的白格里嵌了 ArUco 标记，**每个标记独立可检**，
角点够用就能出位姿。

它在本项目里的用途（与 `board_extrinsics.py` 的 ① 同一个数学）
──────────────────────────────────────────────────────────
    一次性：用**已标定好的**外参拍一张 → 得到 `cam ← 标记` → 反算出 `base ← 标记`
            （标记固定在台面上，之后不再动）
    每次用：拍一张 → `cam ← 标记` → `base ← cam` = `base←标记` ∘ (`cam←标记`)⁻¹

⇒ 相机（三脚架）漂了就重算一次，**一张图、约 1 秒、不动机械臂**。
   不再需要「碰点重标定」，也不再需要判断外参有没有过期。

打印与张贴
──────────
    · 选**「实际大小 / 100%」**，不要「适合页面」。图面上自带一条 **100.0 mm 校验尺**，
      印完量它：量出 100 mm 说明没缩放；量出 97 mm 就说明整体缩到 97%，
      此时**单格真实边长 = 标称值 × 0.97**，这个数要喂给检测端。
    · **务必压平**：纸翘起来会把误差直接灌进位姿。建议贴到硬卡纸/亚克力上。
    · **不要覆膜**（反光会让角点检测退化）。
    · 贴在**机械臂够不到的位置**，且**别盖住台面上的孔洞**导致纸面塌陷。

用法
────
    python3 scripts/make_charuco_board.py                       # A4 竖版，5×5 格 40 mm
    python3 scripts/make_charuco_board.py --paper A4 --orient landscape \\
        --squares-x 5 --squares-y 5 --square-mm 40 --marker-mm 30
    python3 scripts/make_charuco_board.py --preview             # 同时出 PNG

同时输出一份 `<同名>.json`，里面是检测端必须用的**同一套几何参数** ——
检测时用的方格边长与标记边长只要和打印出来的不一致，位姿就是错的，
而且**重投影误差照样很小**（它对尺度不敏感）。
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402
import numpy as np                                               # noqa: E402
import cv2                                                       # noqa: E402
import cv2.aruco as aruco                                        # noqa: E402

PAPERS = {'A4': (210.0, 297.0), 'A3': (297.0, 420.0)}            # 短边, 长边 (mm)

#: 找不到 CJK 字体时**整体退英文**（画成方框比不画更糟 —— 打印出来不能用）。
#: 候选表与 `plot_table_envelope.py` 保持一致：那台机器实测只有 5 个 CJK 字体，
#: `Noto Sans CJK SC` 排在第一位。
CJK_CANDIDATES = [
    'Noto Sans CJK SC', 'Noto Sans CJK JP', 'Noto Sans SC', 'Source Han Sans SC',
    'WenQuanYi Zen Hei', 'WenQuanYi Micro Hei',
]


def setup_font():
    """设置 CJK 字体；返回 True 表示中文可用，False 表示要退英文。"""
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    for want in CJK_CANDIDATES:
        if want in names:
            plt.rcParams['font.sans-serif'] = [want] + list(
                plt.rcParams['font.sans-serif'])
            plt.rcParams['axes.unicode_minus'] = False
            return True
    return False
SCALE_BAR_MM = 100.0
DICT_NAME = 'DICT_4X4_50'


def build_board(sx, sy, square_mm, marker_mm):
    d = aruco.Dictionary_get(getattr(aruco, DICT_NAME))
    board = aruco.CharucoBoard_create(sx, sy, square_mm / 1000.0,
                                      marker_mm / 1000.0, d)
    if marker_mm >= square_mm:
        raise SystemExit('标记边长必须小于方格边长（白格要留边）')
    return d, board


def make(paper, orient, sx, sy, square_mm, marker_mm, margin_mm, out_pdf,
         preview=False, dpi=600):
    short, long_ = PAPERS[paper]
    page_w, page_h = (long_, short) if orient == 'landscape' else (short, long_)
    board_w, board_h = sx * square_mm, sy * square_mm
    if board_w + 2 * margin_mm > page_w or board_h + 2 * margin_mm + 30 > page_h:
        raise SystemExit(
            f'板 {board_w:.0f}×{board_h:.0f} mm 放不进 {paper} {orient}'
            f'（可用 {page_w:.0f}×{page_h:.0f} mm，还需留边距与说明栏）')

    has_cjk = setup_font()
    d, board = build_board(sx, sy, square_mm, marker_mm)
    # ⚠️ 用 OpenCV 自己生成的板面：标记 ID 的分配由它决定，**自己画容易排错**，
    #    而 ID 排错的表现是「一个标记都检不出来」，很难往这里想。
    px = int(round(board_w / 25.4 * dpi))          # 按 dpi 换算像素
    py = int(round(board_h / 25.4 * dpi))
    # ⚠️ 方法名随版本变：4.7+ 叫 generateImage，本项目两端的 4.5.4 叫 draw。
    #    两者签名相同（尺寸, 边距, 边框位数）。写死其中一个会在另一版本上
    #    AttributeError —— 生成器在开发机、检测端在上位机，版本可能不同。
    _gen = getattr(board, 'generateImage', None) or board.draw
    img = _gen((px, py), 0, 1)

    fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, page_w)
    ax.set_ylim(0, page_h)
    ax.set_aspect('equal')
    ax.axis('off')

    x0 = (page_w - board_w) / 2.0
    y0 = page_h - margin_mm - board_h
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='none',
              extent=[x0, x0 + board_w, y0, y0 + board_h], zorder=1)
    # 外框（裁切对齐用）
    ax.add_patch(Rectangle((x0, y0), board_w, board_h, fill=False,
                           edgecolor='#888888', lw=0.3, zorder=2))

    # ---- 100 mm 校验尺 ----
    sy_ = margin_mm + 8
    ax.add_patch(Rectangle((margin_mm, sy_), SCALE_BAR_MM, 6,
                           facecolor='black', edgecolor='black', zorder=3))
    if has_cjk:
        bar_txt = f'这一条 = {SCALE_BAR_MM:.1f} mm（拿尺子量它；短了说明被缩放）'
        txt = (f'ChArUco {sx}×{sy} · 方格 {square_mm:g} mm · 标记 {marker_mm:g} mm · '
               f'{DICT_NAME}\n'
               f'打印：实际大小 / 100%，不要「适合页面」。印完先量上面那条尺。\n'
               f'张贴：贴硬卡纸压平，不要覆膜；放在机械臂够不到、'
               f'且不压住台面孔洞的位置。')
    else:
        bar_txt = (f'this bar = {SCALE_BAR_MM:.1f} mm '
                   f'(measure it; shorter means the print was scaled)')
        txt = (f'ChArUco {sx}x{sy} - square {square_mm:g} mm - marker {marker_mm:g} mm'
               f' - {DICT_NAME}\n'
               f'PRINT: actual size / 100%, do NOT use "fit to page". '
               f'Measure the bar first.\n'
               f'MOUNT: tape onto rigid card, keep it flat, do not laminate.')
    ax.text(margin_mm + SCALE_BAR_MM + 4, sy_ + 3, bar_txt, fontsize=7, va='center')
    ax.text(margin_mm, y0 - 8, txt, fontsize=7, va='top', linespacing=1.6)

    fig.savefig(out_pdf, format='pdf')
    outs = [out_pdf]
    if preview:
        png = os.path.splitext(out_pdf)[0] + '.png'
        fig.savefig(png, format='png', dpi=110)
        outs.append(png)
    plt.close(fig)
    if not has_cjk:
        print('⚠️ 没找到 CJK 字体 —— 图上的说明文字已退成英文（画成方框更糟）。'
              '想要中文说明，把本脚本拷到**上位机**上跑（那边有 Noto CJK）。')

    meta = {
        'dictionary': DICT_NAME,
        'squares_x': sx, 'squares_y': sy,
        'square_mm': square_mm, 'marker_mm': marker_mm,
        'board_w_mm': board_w, 'board_h_mm': board_h,
        'paper': paper, 'orient': orient,
        'scale_bar_mm': SCALE_BAR_MM,
        'note': '检测端必须用这里的 square_mm / marker_mm / dictionary；'
                '打印被整体缩放时，把量出来的比例乘到 square_mm 与 marker_mm 上',
    }
    meta_path = os.path.splitext(out_pdf)[0] + '.json'
    with open(meta_path, 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=1)
    outs.append(meta_path)
    return outs, meta


def main():
    p = argparse.ArgumentParser(description='生成精确尺寸的 ChArUco 标定板 PDF')
    p.add_argument('--paper', default='A4', choices=sorted(PAPERS))
    p.add_argument('--orient', default='portrait',
                   choices=['portrait', 'landscape'])
    p.add_argument('--squares-x', type=int, default=5, help='方格列数')
    p.add_argument('--squares-y', type=int, default=5, help='方格行数')
    p.add_argument('--square-mm', type=float, default=40.0)
    p.add_argument('--marker-mm', type=float, default=30.0)
    p.add_argument('--margin-mm', type=float, default=20.0)
    p.add_argument('--dpi', type=int, default=600, help='板面位图分辨率（打印用）')
    p.add_argument('--out', default=None)
    p.add_argument('--preview', action='store_true')
    a = p.parse_args()

    out = a.out or 'charuco_%s_%s_%dx%d_%gmm.pdf' % (
        a.paper, a.orient, a.squares_x, a.squares_y, a.square_mm)
    outs, meta = make(a.paper, a.orient, a.squares_x, a.squares_y,
                      a.square_mm, a.marker_mm, a.margin_mm, out,
                      preview=a.preview, dpi=a.dpi)
    print('已写出：')
    for o in outs:
        print('  ', o, f'({os.path.getsize(o)} B)')
    print(f"  板面 {meta['board_w_mm']:.0f}×{meta['board_h_mm']:.0f} mm，"
          f"校验尺 {meta['scale_bar_mm']:.0f} mm，字典 {meta['dictionary']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
