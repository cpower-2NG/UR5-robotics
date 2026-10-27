#!/usr/bin/env python3
"""阶段 2 基准垫（2 页 A4 横向）生成器 —— 宫格 + 两页 ArUco 标记 + 垫系 sidecar。

它解决什么问题
──────────────
阶段 2 的终态判分要回答「块是不是放进了对应颜色的格、边缘是否平行格线、
格内偏了多少」。这三项都需要一个**物理上钉死的格坐标系**。本脚本把
3×3 宫格、两页标记、拼贴关系、格心/颜色顺序一次性写进 PDF 与 sidecar JSON：

  · 第 1 页（左）：3×3 宫格 + 2×5 个 ArUco 标记（DICT_4X4_50，ID 0–9）；
  · 第 2 页（右）：2×5 个 ArUco 标记（DICT_5X5_50，ID 0–9）；
  · 拼贴：第 2 页左缘对齐第 1 页 x=282 mm 的虚线，两页上缘对齐，
    重叠 15 mm；第 1 页压在**上面**（第 2 页左 15 mm 塞到第 1 页下面）；
  · 第 2 页局部 x=110 mm 是可剪虚线；台面窄就剪掉右侧，标记带不受影响。

为什么标记用 28 mm、间隙 6 mm
────────────────────────────
腕部相机在 0.34 m 处 fx≈908 px ⇒ 28 mm 标记约 75 px；4×4 字典每模块约
12.5 px，5×5 约 10.7 px，都在可靠检测范围内。标记带 2 列 × 5 行、
带宽 62 mm、带高 164 mm，两页拼好后标记场约 354×164 mm，落在
0.34 m 视场（约 482×273 mm）内；宫格 180×180 mm 也在视场内。

为什么不用 ChArUco 插值板
────────────────────────
契约写的是「各印 ChArUco 标记」。本脚本用 **ChArUco 板中的 ArUco 标记部分**
（每个标记独立可检、四角已知）+ PnP 解页位姿——这正是 ChArUco 用来定位的
那一半；同时避开两版 OpenCV 的 `CharucoBoard`/`GridBoard` API 差异
（开发机 4.5.4 与推理 venv 4.10 的方法名不同）。若要求严格 ChArUco 插值板，
可换，但检测/解算路径不变。

打印
────
  · A4 横向、**100% / 实际大小**，不要「适合页面」；两页都打；
  · 印完先用尺子量 100 mm 校验尺：量出 100 mm 才说明没缩放；
  · 第 1 页在左、第 2 页在右，第 2 页左缘对齐第 1 页 x=282 虚线，
    上缘对齐，重叠 15 mm；第 1 页压在上面；
  · 不要覆膜（反光会毁掉标记检测）；纸要压平。

用法
────
    python3 scripts/stage2_mat.py
    python3 scripts/stage2_mat.py --grid-order red,orange,tan,yellow,yellowgreen,cyan,purple,spare,spare
    python3 scripts/stage2_mat.py --pitch-mm 55 --preview-only

输出（默认在仓外 notes，避免把重资产/打印件放进公开仓）：
    ~/robotics/notes/ur5-vision-grasp/stage2-mat/stage2-mat.pdf
    ~/robotics/notes/ur5-vision-grasp/stage2-mat/stage2-mat.png   （拼贴预览/自检用）
    ~/robotics/notes/ur5-vision-grasp/stage2-mat/stage2-mat.json  （检测端 sidecar）
"""

import argparse
import hashlib
import json
import os
import sys

sys.dont_write_bytecode = True

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages             # noqa: E402
from matplotlib.font_manager import FontProperties               # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402
import numpy as np                                               # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage2_aruco                                             # noqa: E402
from block_classes import DEFAULT_CLASSES, PALETTE               # noqa: E402

# ------------------------------------------------------------------ 几何常量
MM_PER_INCH = 25.4
PAGE_W_MM = 297.0
PAGE_H_MM = 210.0
PITCH_MIN_MM = 55.0
PITCH_MAX_MM = 60.0
GRID_X0_MM = 90.0                 # 第 1 页宫格左缘（给左侧标记带让位）
GRID_PAD_MM = 0.0                 # 宫格边框加粗量（只影响画法，不影响格心）
MARKER_MM = 28.0
MARKER_GAP_MM = 6.0
MARKER_COLS = 2
MARKER_ROWS = 5
MARKER_X_A_MM = 10.0              # 第 1 页标记带左缘
MARKER_X_B_LOCAL_MM = 20.0        # 第 2 页标记带左缘（页内局部坐标）
MARKER_Y0_MM = 23.0
PAGE_B_OFFSET_MM = 282.0          # 第 2 页左缘 = 第 1 页 x=282
OVERLAP_MM = 15.0
TRIM_LOCAL_MM = 110.0             # 第 2 页可剪虚线（页内局部 x）
MAT_W_MM = PAGE_B_OFFSET_MM + TRIM_LOCAL_MM   # 392
MAT_H_MM = PAGE_H_MM
SCALE_BAR_MM = 100.0
DICT_A = 'DICT_4X4_50'
DICT_B = 'DICT_5X5_50'

DEFAULT_ORDER = ['red', 'orange', 'tan',
                 'yellow', 'yellowgreen', 'cyan',
                 'purple', 'spare', 'spare']

COLOR_CN = {
    'red': '红', 'orange': '橙', 'tan': '棕', 'yellow': '黄',
    'yellowgreen': '黄绿', 'cyan': '青', 'purple': '紫', 'spare': '备用',
}

# 本机 fontconfig 没有 CJK 字体；Windows 字体目录里有微软雅黑。
# 用 FontProperties(fname=...) 显式注册，避免依赖 fontconfig 的家族名。
FONT_CANDIDATES = [
    '/mnt/c/Windows/Fonts/msyh.ttc',
    '/mnt/c/Windows/Fonts/msyhbd.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
]


# ------------------------------------------------------------------ 小工具
def _font_path():
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            return p
    return None


def _font(size, bold=False):
    path = _font_path()
    if path:
        return FontProperties(fname=path, size=size,
                              weight='bold' if bold else 'normal')
    return FontProperties(size=size, weight='bold' if bold else 'normal')


def _has_cjk():
    return _font_path() is not None


def _mm_to_pt(mm):
    return float(mm) * 72.0 / MM_PER_INCH


def _page_figure(w_mm, h_mm):
    """精确页尺寸的画布；1 数据单位 = 1 mm，y 轴向下（页左上为原点）。"""
    fig = plt.figure(figsize=(w_mm / MM_PER_INCH, h_mm / MM_PER_INCH))
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_xlim(0.0, w_mm)
    ax.set_ylim(h_mm, 0.0)                 # y 向下
    ax.axis('off')
    return fig, ax


def _rect(ax, x, y, w, h, **kw):
    # 注意：y 轴向下，Rectangle 的 y 是屏幕上的上边，向 +h 方向长。
    ax.add_patch(Rectangle((x, y), w, h, **kw))


def _text(ax, x, y, s, size=5.0, **kw):
    kw.setdefault('ha', 'left')
    kw.setdefault('va', 'top')
    ax.text(x, y, s, fontproperties=_font(size), **kw)


def _scale_bar(ax, x0, y0, length_mm=SCALE_BAR_MM, height_mm=2.5, tick_mm=10.0):
    """实心 100 mm 校验尺；刻度朝上（屏幕上方 = y 减小）。"""
    _rect(ax, x0, y0, length_mm, height_mm, facecolor='black',
          edgecolor='none', zorder=7)
    n = int(round(length_mm / tick_mm))
    for i in range(n + 1):
        x = x0 + i * tick_mm
        h = height_mm * (2.0 if i % 5 == 0 else 1.4)
        _rect(ax, x - 0.15, y0 - h, 0.3, h, facecolor='black',
              edgecolor='none', zorder=7)
    _text(ax, x0 + length_mm / 2.0, y0 + height_mm + 1.0, '100 mm',
          size=5.0, ha='center', va='top')


def _check_order(order):
    if len(order) != 9:
        raise ValueError('宫格顺序必须 9 个 token，收到 %d 个：%r' %
                         (len(order), order))
    allowed = set(DEFAULT_CLASSES) | {'spare'}
    bad = [x for x in order if x not in allowed]
    if bad:
        raise ValueError('宫格顺序含未知颜色：%r（允许 %s 或 spare）' %
                         (bad, ','.join(DEFAULT_CLASSES)))
    for name in DEFAULT_CLASSES:
        if order.count(name) != 1:
            raise ValueError('七色必须各出现一次；%s 出现 %d 次：%r' %
                             (name, order.count(name), order))
    if order.count('spare') != 2:
        raise ValueError('备用格必须 2 个；收到 %d 个：%r' %
                         (order.count('spare'), order))


def parse_order(spec):
    if spec is None:
        return list(DEFAULT_ORDER)
    order = [x.strip().lower() for x in str(spec).split(',')]
    _check_order(order)
    return order


# ------------------------------------------------------------------ 元数据
def _grid_geometry(pitch):
    if not (PITCH_MIN_MM <= pitch <= PITCH_MAX_MM):
        raise ValueError('pitch 必须在 %.0f–%.0f mm 之间，收到 %g' %
                         (PITCH_MIN_MM, PITCH_MAX_MM, pitch))
    size = 3.0 * pitch
    y0 = (PAGE_H_MM - size) / 2.0
    center_page = [GRID_X0_MM + size / 2.0, PAGE_H_MM / 2.0]
    return {
        'pitch_mm': float(pitch),
        'x0_mm': GRID_X0_MM,
        'y0_mm': y0,
        'size_mm': size,
        'center_page_mm': center_page,
        'center_pad_mm': [0.0, 0.0],
    }


def _page_to_pad(x_page, y_page, center_page):
    """第 1 页坐标 → 垫系：x 右、y 上、原点宫格中心。"""
    return [float(x_page) - center_page[0],
            float(center_page[1]) - float(y_page)]


def _page_b_to_pad(x_local, y_local, center_page):
    """第 2 页局部坐标 → 垫系。局部 x=0 贴在第 1 页 x=282 处。"""
    return _page_to_pad(PAGE_B_OFFSET_MM + float(x_local), y_local, center_page)


def _marker_corners_page(x, y):
    return [[x, y], [x + MARKER_MM, y],
            [x + MARKER_MM, y + MARKER_MM], [x, y + MARKER_MM]]


def build_meta(order, pitch):
    _check_order(order)
    g = _grid_geometry(pitch)
    center = g['center_page_mm']

    cells = []
    for row in range(3):
        for col in range(3):
            idx = row * 3 + col
            cxp = g['x0_mm'] + (col + 0.5) * pitch
            cyp = g['y0_mm'] + (row + 0.5) * pitch
            cells.append({
                'idx': idx, 'row': row, 'col': col,
                'color': order[idx],
                'center_page_mm': [cxp, cyp],
                'center_pad_mm': _page_to_pad(cxp, cyp, center),
            })

    def _markers(x0_local, dict_name, page_b=False):
        out = []
        for row in range(MARKER_ROWS):
            for col in range(MARKER_COLS):
                mid = row * MARKER_COLS + col
                x = x0_local + col * (MARKER_MM + MARKER_GAP_MM)
                y = MARKER_Y0_MM + row * (MARKER_MM + MARKER_GAP_MM)
                corners_page = _marker_corners_page(x, y)
                if page_b:
                    corners_pad = [_page_b_to_pad(cx, cy, center)
                                   for cx, cy in corners_page]
                else:
                    corners_pad = [_page_to_pad(cx, cy, center)
                                   for cx, cy in corners_page]
                out.append({
                    'id': mid,
                    'dictionary': dict_name,
                    'size_mm': MARKER_MM,
                    'page_mm': [x, y],
                    'corners_page_mm': corners_page,
                    'corners_pad_mm': corners_pad,
                    'centroid_page_mm': [x + MARKER_MM / 2.0,
                                         y + MARKER_MM / 2.0],
                    'centroid_pad_mm': [(corners_pad[0][0] + corners_pad[2][0]) / 2.0,
                                        (corners_pad[0][1] + corners_pad[2][1]) / 2.0],
                })
        return out

    markers_a = _markers(MARKER_X_A_MM, DICT_A, page_b=False)
    markers_b = _markers(MARKER_X_B_LOCAL_MM, DICT_B, page_b=True)

    field_page = [[MARKER_X_A_MM, MARKER_Y0_MM],
                  [PAGE_B_OFFSET_MM + MARKER_X_B_LOCAL_MM
                   + MARKER_COLS * MARKER_MM + (MARKER_COLS - 1) * MARKER_GAP_MM,
                   MARKER_Y0_MM + MARKER_ROWS * MARKER_MM
                   + (MARKER_ROWS - 1) * MARKER_GAP_MM]]
    field_pad = [_page_to_pad(field_page[0][0], field_page[1][1], center),
                 _page_to_pad(field_page[1][0], field_page[0][1], center)]

    meta = {
        'version': 1,
        'kind': 'stage2-mat',
        'paper': 'A4',
        'orient': 'landscape',
        'page_w_mm': PAGE_W_MM,
        'page_h_mm': PAGE_H_MM,
        'pitch_mm': float(pitch),
        'grid': {
            'x0_mm': g['x0_mm'], 'y0_mm': g['y0_mm'],
            'size_mm': g['size_mm'],
            'center_page_mm': center,
            'center_pad_mm': [0.0, 0.0],
            'cells': cells,
        },
        'pad': {
            'origin_page_mm': center,
            'x_axis': 'page +x',
            'y_axis': 'page -y (up)',
            'z_axis': 'out of page',
            'page_b_origin_page_mm': [PAGE_B_OFFSET_MM, 0.0],
            'overlap_mm': OVERLAP_MM,
            'trim_local_mm': TRIM_LOCAL_MM,
            'mat_w_mm': MAT_W_MM,
            'mat_h_mm': MAT_H_MM,
        },
        'pages': {
            'a': {
                'page': 'A', 'dictionary': DICT_A,
                'markers': markers_a,
                'marker_strip_page_mm': [
                    MARKER_X_A_MM, MARKER_Y0_MM,
                    MARKER_COLS * MARKER_MM + (MARKER_COLS - 1) * MARKER_GAP_MM,
                    MARKER_ROWS * MARKER_MM + (MARKER_ROWS - 1) * MARKER_GAP_MM,
                ],
            },
            'b': {
                'page': 'B', 'dictionary': DICT_B,
                'markers': markers_b,
                'marker_strip_local_mm': [
                    MARKER_X_B_LOCAL_MM, MARKER_Y0_MM,
                    MARKER_COLS * MARKER_MM + (MARKER_COLS - 1) * MARKER_GAP_MM,
                    MARKER_ROWS * MARKER_MM + (MARKER_ROWS - 1) * MARKER_GAP_MM,
                ],
            },
        },
        'marker_field_page_mm': field_page,
        'marker_field_pad_mm': field_pad,
        'scale_bar_mm': SCALE_BAR_MM,
        'scale_bar_a_page_mm': [90.0, 202.0, SCALE_BAR_MM, 2.5],
        'scale_bar_b_local_mm': [5.0, 202.0, SCALE_BAR_MM, 2.5],
        'colors': {k: list(v) for k, v in PALETTE.items()},
        'grid_order': list(order),
        'notes': {
            'print': 'A4 landscape, 100% / actual size; do not use fit-to-page.',
            'assembly': 'page 1 on top; page 2 left edge at page-1 x=282 mm; '
                        'top edges aligned; 15 mm overlap.',
            'trim': 'page 2 may be trimmed along local x=110 mm; keep markers.',
            'frame': 'pad origin = grid center; pad x = page +x; pad y = page -y; '
                     'z out of page.',
        },
    }
    validate_meta(meta)
    return meta


def _canonical_json(obj):
    return json.dumps(obj, ensure_ascii=False, sort_keys=True,
                      separators=(',', ':'))


def add_checksum(meta):
    meta = dict(meta)
    meta.pop('checksum_sha256', None)
    meta['checksum_sha256'] = hashlib.sha256(
        _canonical_json(meta).encode('utf-8')).hexdigest()
    return meta


def validate_meta(meta):
    """结构自检；变异测试也用它。任何一条不满足都抛 ValueError。"""
    if meta.get('version') != 1 or meta.get('kind') != 'stage2-mat':
        raise ValueError('sidecar 版本/类型不符')
    if abs(meta['page_w_mm'] - PAGE_W_MM) > 1e-9 or \
            abs(meta['page_h_mm'] - PAGE_H_MM) > 1e-9:
        raise ValueError('页尺寸不是 A4 横向 297×210')
    order = meta.get('grid_order') or []
    _check_order(order)
    pitch = float(meta['pitch_mm'])
    g = _grid_geometry(pitch)
    if abs(meta['grid']['center_page_mm'][0] - g['center_page_mm'][0]) > 1e-9 or \
            abs(meta['grid']['center_page_mm'][1] - g['center_page_mm'][1]) > 1e-9:
        raise ValueError('宫格中心与几何常量不符')
    cells = meta['grid']['cells']
    if len(cells) != 9:
        raise ValueError('格数不是 9')
    for i, c in enumerate(cells):
        if c['idx'] != i or c['row'] != i // 3 or c['col'] != i % 3:
            raise ValueError('格索引/行列不连续：%r' % (c,))
        if c['color'] != order[i]:
            raise ValueError('格 %d 颜色与 grid_order 不符' % i)
        cxp = g['x0_mm'] + (c['col'] + 0.5) * pitch
        cyp = g['y0_mm'] + (c['row'] + 0.5) * pitch
        if abs(c['center_page_mm'][0] - cxp) > 1e-6 or \
                abs(c['center_page_mm'][1] - cyp) > 1e-6:
            raise ValueError('格 %d 的页坐标不对' % i)
        want_pad = _page_to_pad(cxp, cyp, g['center_page_mm'])
        if abs(c['center_pad_mm'][0] - want_pad[0]) > 1e-6 or \
                abs(c['center_pad_mm'][1] - want_pad[1]) > 1e-6:
            raise ValueError('格 %d 的垫坐标不对' % i)
    for key, dict_name, page_b in (('a', DICT_A, False), ('b', DICT_B, True)):
        page = meta['pages'][key]
        if page['dictionary'] != dict_name:
            raise ValueError('第 %s 页字典不对' % key)
        markers = page['markers']
        if len(markers) != MARKER_COLS * MARKER_ROWS:
            raise ValueError('第 %s 页标记数不是 %d' %
                             (key, MARKER_COLS * MARKER_ROWS))
        if sorted(m['id'] for m in markers) != list(range(10)):
            raise ValueError('第 %s 页标记 ID 不是 0–9 各一次' % key)
        for m in markers:
            if abs(m['size_mm'] - MARKER_MM) > 1e-9:
                raise ValueError('标记尺寸不是 %.0f mm' % MARKER_MM)
            c_page = m['corners_page_mm']
            if len(c_page) != 4:
                raise ValueError('标记角点数不是 4')
            # 角点必须是轴对齐正方形，顺序为左上/右上/右下/左下
            x, y = m['page_mm']
            want = _marker_corners_page(x, y)
            for got, exp in zip(c_page, want):
                if abs(got[0] - exp[0]) > 1e-9 or abs(got[1] - exp[1]) > 1e-9:
                    raise ValueError('标记 %d 的页角点不对：%r vs %r' %
                                     (m['id'], got, exp))
            c_pad = m['corners_pad_mm']
            for (px, py), (qx, qy) in zip(c_page, c_pad):
                want_pad = (_page_b_to_pad(px, py, meta['grid']['center_page_mm'])
                            if page_b else
                            _page_to_pad(px, py, meta['grid']['center_page_mm']))
                if abs(want_pad[0] - qx) > 1e-9 or abs(want_pad[1] - qy) > 1e-9:
                    raise ValueError('标记 %d 的垫角点不对' % m['id'])
    if abs(meta['pad']['page_b_origin_page_mm'][0] - PAGE_B_OFFSET_MM) > 1e-9:
        raise ValueError('第 2 页偏移不是 %.0f mm' % PAGE_B_OFFSET_MM)
    if abs(meta['pad']['trim_local_mm'] - TRIM_LOCAL_MM) > 1e-9:
        raise ValueError('可剪线不是 %.0f mm' % TRIM_LOCAL_MM)
    if abs(meta['pad']['mat_w_mm'] - MAT_W_MM) > 1e-9:
        raise ValueError('拼贴后垫宽不是 %.0f mm' % MAT_W_MM)
    return True


# ------------------------------------------------------------------ 画页面
def _draw_marker(ax, x, y, dict_name, mid, dpi):
    side = int(round(MARKER_MM / MM_PER_INCH * dpi))
    img = stage2_aruco.draw_marker(dict_name, mid, side)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, interpolation='none',
              origin='upper', aspect='auto',
              extent=[x, x + MARKER_MM, y + MARKER_MM, y], zorder=6)


def _draw_page_a(ax, ox, oy, meta, dpi):
    order = meta['grid_order']
    pitch = meta['pitch_mm']
    gx0 = ox + meta['grid']['x0_mm']
    gy0 = oy + meta['grid']['y0_mm']
    size = meta['grid']['size_mm']

    _rect(ax, ox, oy, PAGE_W_MM, PAGE_H_MM, facecolor='white',
          edgecolor='none', zorder=0)

    # ---- 网格线条 ----
    for i in range(4):
        lw = _mm_to_pt(1.5 if i in (0, 3) else 0.8)
        x = gx0 + i * pitch
        ax.plot([x, x], [gy0, gy0 + size], color='black', lw=lw,
                solid_capstyle='butt', zorder=3)
        y = gy0 + i * pitch
        ax.plot([gx0, gx0 + size], [y, y], color='black', lw=lw,
                solid_capstyle='butt', zorder=3)

    # ---- 格内色块/文字/索引 ----
    patch = 8.0 if pitch >= 60.0 else 6.0
    for cell in meta['grid']['cells']:
        row, col = cell['row'], cell['col']
        cx0 = gx0 + col * pitch
        cy0 = gy0 + row * pitch
        color = cell['color']
        if color == 'spare':
            _rect(ax, cx0 + 1.2, cy0 + 1.2, pitch - 2.4, pitch - 2.4,
                  fill=False, edgecolor='0.45', lw=0.45,
                  linestyle=(0, (3.0, 2.0)), zorder=4)
            _text(ax, cx0 + pitch / 2.0, cy0 + pitch / 2.0,
                  '备用\nspare' if _has_cjk() else 'spare',
                  size=4.6, ha='center', va='center', color='0.35')
        else:
            rgb = PALETTE.get(color, (0.5, 0.5, 0.5))
            _rect(ax, cx0 + 3.0, cy0 + 3.0, patch, patch,
                  facecolor=rgb, edgecolor='black', lw=0.15, zorder=4)
            label = ('%s %s' % (COLOR_CN.get(color, ''), color)).strip() \
                if _has_cjk() else color
            _text(ax, cx0 + 3.0, cy0 + pitch - 2.0, label,
                  size=4.6, ha='left', va='bottom', color='black', zorder=5)
        _text(ax, cx0 + pitch - 2.0, cy0 + pitch - 2.0, '#%d' % cell['idx'],
              size=3.4, ha='right', va='bottom', color='0.45', zorder=5)

    # ---- 标记带 ----
    for m in meta['pages']['a']['markers']:
        x, y = m['page_mm']
        _draw_marker(ax, ox + x, oy + y, DICT_A, m['id'], dpi)
    _text(ax, ox + MARKER_X_A_MM, oy + MARKER_Y0_MM - 3.0,
          'A · %s · ID 0–9' % DICT_A, size=4.4, color='0.35')
    _text(ax, ox + MARKER_X_A_MM,
          oy + MARKER_Y0_MM + MARKER_ROWS * MARKER_MM
          + (MARKER_ROWS - 1) * MARKER_GAP_MM + 1.5,
          'markers / 标记', size=4.0, color='0.45', va='top')

    # ---- 页眉/对齐线/校验尺/页号 ----
    _text(ax, ox + 8.0, oy + 8.0,
          '阶段 2 基准垫 · 第 1 页  /  Stage-2 mat · page 1'
          if _has_cjk() else 'Stage-2 mat · page 1',
          size=7.0, color='black')
    ax.plot([ox + 282.0, ox + 282.0], [oy + 12.0, oy + 198.0],
            color='0.45', lw=0.5, linestyle=(0, (2.0, 2.0)), zorder=4)
    _text(ax, ox + 282.0, oy + 11.0,
          '第 2 页左缘对齐此线\npage-2 left edge' if _has_cjk()
          else 'page-2 left edge', size=4.0, ha='center', va='bottom',
          color='0.45')
    _scale_bar(ax, ox + 90.0, oy + 202.0)
    _text(ax, ox + 287.0, oy + 205.0, 'page 1 / 2', size=4.5,
          ha='right', va='top', color='0.45')


def _draw_page_b(ax, ox, oy, meta, dpi):
    _rect(ax, ox, oy, PAGE_W_MM, PAGE_H_MM, facecolor='white',
          edgecolor='none', zorder=0)

    # 重叠带（第 1 页压在上面，实际会被盖住；这里只画对齐参考）
    ax.plot([ox + 15.0, ox + 15.0], [oy + 12.0, oy + 198.0],
            color='0.55', lw=0.5, linestyle=(0, (2.0, 2.0)), zorder=3)
    _text(ax, ox + 15.0, oy + 191.0,
          '第 1 页右缘\npage-1 right edge' if _has_cjk()
          else 'page-1 right edge', size=4.0, ha='center', va='top',
          color='0.45')

    # 标记带（放在重叠带之外：局部 x=20 起）
    for m in meta['pages']['b']['markers']:
        x, y = m['page_mm']
        _draw_marker(ax, ox + x, oy + y, DICT_B, m['id'], dpi)
    _text(ax, ox + MARKER_X_B_LOCAL_MM, oy + MARKER_Y0_MM - 3.0,
          'B · %s · ID 0–9' % DICT_B, size=4.4, color='0.35')

    # 可剪虚线
    ax.plot([ox + TRIM_LOCAL_MM, ox + TRIM_LOCAL_MM], [oy + 12.0, oy + 198.0],
            color='0.35', lw=0.6, linestyle=(0, (4.0, 2.0)), zorder=3)
    _text(ax, ox + TRIM_LOCAL_MM + 2.0, oy + 105.0,
          '沿此线剪 / trim' if _has_cjk() else 'trim',
          size=4.6, rotation=270, ha='center', va='center', color='0.35')
    _text(ax, ox + TRIM_LOCAL_MM + 14.0, oy + 20.0,
          '右侧留白\n可剪掉' if _has_cjk() else 'blank / trimmable',
          size=5.0, color='0.55')

    _text(ax, ox + 20.0, oy + 8.0,
          '阶段 2 基准垫 · 第 2 页（衬在第 1 页下）  /  page 2 (under page 1)'
          if _has_cjk() else 'Stage-2 mat · page 2 (under page 1)',
          size=7.0, color='black')
    _scale_bar(ax, ox + 5.0, oy + 202.0)
    _text(ax, ox + 287.0, oy + 205.0, 'page 2 / 2', size=4.5,
          ha='right', va='top', color='0.45')


# ------------------------------------------------------------------ 输出
def _check_pdf_mediabox(path, expect=(PAGE_W_MM, PAGE_H_MM)):
    with open(path, 'rb') as fh:
        data = fh.read()
    boxes = []
    pos = 0
    while True:
        i = data.find(b'/MediaBox', pos)
        if i < 0:
            break
        j = data.find(b']', i)
        if j < 0:
            break
        raw = data[i + 9:j].replace(b'[', b' ').replace(b']', b' ')
        try:
            vals = [float(x) for x in raw.split()]
        except ValueError:
            vals = []
        if len(vals) == 4:
            boxes.append((vals[2] - vals[0], vals[3] - vals[1]))
        pos = j + 1
    if len(boxes) < 2:
        raise RuntimeError('PDF 里读到 %d 个 MediaBox，期望 2 页：%s' %
                           (len(boxes), path))
    good = []
    for w_pt, h_pt in boxes:
        w_mm, h_mm = w_pt * MM_PER_INCH / 72.0, h_pt * MM_PER_INCH / 72.0
        good.append(abs(w_mm - expect[0]) < 0.2 and
                    abs(h_mm - expect[1]) < 0.2)
    if not all(good):
        raise RuntimeError('PDF 页面尺寸不符：%r（期望 %s mm）' %
                           (boxes, expect))
    return boxes


def write_pdf(meta, out_pdf, dpi):
    with PdfPages(out_pdf) as pdf:
        fig, ax = _page_figure(PAGE_W_MM, PAGE_H_MM)
        _draw_page_a(ax, 0.0, 0.0, meta, dpi)
        pdf.savefig(fig)
        plt.close(fig)

        fig, ax = _page_figure(PAGE_W_MM, PAGE_H_MM)
        _draw_page_b(ax, 0.0, 0.0, meta, dpi)
        pdf.savefig(fig)
        plt.close(fig)
    boxes = _check_pdf_mediabox(out_pdf)
    print('PDF 页面自检：%d 页，MediaBox %s pt ≈ %.2f×%.2f mm' %
          (len(boxes), boxes[0], boxes[0][0] * MM_PER_INCH / 72.0,
           boxes[0][1] * MM_PER_INCH / 72.0))


def write_mat_png(meta, out_png, dpi):
    """拼贴后的垫子预览（392×210 mm）；第 2 页在下、第 1 页在上。"""
    fig, ax = _page_figure(MAT_W_MM, MAT_H_MM)
    _draw_page_b(ax, PAGE_B_OFFSET_MM, 0.0, meta, dpi)
    _draw_page_a(ax, 0.0, 0.0, meta, dpi)
    fig.savefig(out_png, dpi=dpi, facecolor='white')
    plt.close(fig)
    # 读回来确认尺寸（不是必须，但自检脚本会读它）
    import cv2
    img = cv2.imread(out_png, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError('PNG 写出后读不回来：%s' % out_png)
    return img.shape


def write_json(meta, out_json):
    with open(out_json, 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=1, sort_keys=True)
        fh.write('\n')


def make(order, pitch, out_pdf, out_png, out_json, dpi=600, png_dpi=300,
         write_pdf_file=True, write_png_file=True):
    meta = build_meta(order, pitch)
    meta = add_checksum(meta)
    outs = []
    if write_pdf_file:
        os.makedirs(os.path.dirname(os.path.abspath(out_pdf)), exist_ok=True)
        write_pdf(meta, out_pdf, dpi)
        outs.append(out_pdf)
    if write_png_file:
        os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
        shape = write_mat_png(meta, out_png, png_dpi)
        print('垫子预览 PNG：%s  %dx%d px（%.0f dpi）' %
              (out_png, shape[1], shape[0], png_dpi))
        outs.append(out_png)
    os.makedirs(os.path.dirname(os.path.abspath(out_json)), exist_ok=True)
    write_json(meta, out_json)
    outs.append(out_json)
    return meta, outs


def main():
    base = os.path.expanduser('~/robotics/notes/ur5-vision-grasp/stage2-mat')
    p = argparse.ArgumentParser(
        description='生成阶段 2 基准垫 2 页 A4 横向 PDF + sidecar JSON',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--grid-order', default=','.join(DEFAULT_ORDER),
                   help='9 个格色，行优先；spare 为备用')
    p.add_argument('--pitch-mm', type=float, default=60.0,
                   help='格心距（55–60）')
    p.add_argument('--out-pdf', default=os.path.join(base, 'stage2-mat.pdf'))
    p.add_argument('--out-png', default=os.path.join(base, 'stage2-mat.png'))
    p.add_argument('--out-json', default=os.path.join(base, 'stage2-mat.json'))
    p.add_argument('--dpi', type=int, default=600,
                   help='标记位图分辨率（PDF 用）')
    p.add_argument('--png-dpi', type=int, default=300,
                   help='拼贴预览 PNG 分辨率')
    p.add_argument('--preview-only', action='store_true',
                   help='只出 PNG/JSON，不出 PDF')
    a = p.parse_args()

    order = parse_order(a.grid_order)
    meta, outs = make(order, a.pitch_mm, a.out_pdf, a.out_png, a.out_json,
                      dpi=a.dpi, png_dpi=a.png_dpi,
                      write_pdf_file=not a.preview_only)
    print('宫格顺序（行优先）：')
    for row in range(3):
        print('  ' + '  '.join('%2d:%s' % (row * 3 + c, meta['grid_order'][row * 3 + c])
                               for c in range(3)))
    print('格心距 %.0f mm；宫格 %.0f×%.0f mm；垫子 %.0f×%.0f mm' %
          (meta['pitch_mm'], meta['grid']['size_mm'], meta['grid']['size_mm'],
           meta['pad']['mat_w_mm'], meta['pad']['mat_h_mm']))
    print('标记场（第 1 页坐标）x %.0f–%.0f，y %.0f–%.0f mm' %
          (meta['marker_field_page_mm'][0][0], meta['marker_field_page_mm'][1][0],
           meta['marker_field_page_mm'][0][1], meta['marker_field_page_mm'][1][1]))
    print('已写出：')
    for o in outs:
        print('  ', o, '(%d B)' % os.path.getsize(o))
    if not _has_cjk():
        print('提示：没找到 CJK 字体，图面文字已退英文。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
