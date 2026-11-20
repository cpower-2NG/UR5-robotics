#!/usr/bin/env python3
"""阶段 4 正式批次的 **A3 1:1 摆位图**（现场照着重摆块用）。

为什么需要它
------------
正式轮是「近似手摆、不跑前实测」：块的位置就是 layout manifest 的真值，
人摆歪多少只有跑后才知道。人肉把毫米坐标变成桌上的位置，靠的是**一张 1:1 的图**：

* 图纸与桌面已有的拾取区边界对齐；
* 每块画**真实大小**（29.5 mm 方块）的轮廓 + 朝向对角线 + 中心十字；
* 在中心十字处用铅笔点一个点，**收走图纸再摆块** ——
  纸或胶带压在块下会污染深度与彩色（阶段 3 踩过）。

⚠️ **1:1 是硬要求，不提供缩放。** 放不下就直接报错退出（exit 2），
   绝不悄悄缩小 —— 缩小的图看起来一模一样，但整盘会系统性偏，而且没人会发现。
   打印时必须选「实际大小 / 100%」，打完先用图上的 **100 mm 比例尺**核一遍。

坐标口径：base 系毫米（与 `stage4_layouts` 同一份表）；图上 x 向右、y 向上，
与 base 系同向，只做平移。

跑法：
    # 三个选中布局 → 一份三页 PDF
    python3 scripts/stage4_layout_sheet.py --out /tmp/s0-layouts.pdf
    # 只出一页 PNG
    python3 scripts/stage4_layout_sheet.py --layout 2 --out /tmp/s0-2.png --dpi 300

✅ 不碰 ROS、不连机械臂：本机 WSL 就能跑。
退出码 0 = 出图了；2 = 放不下 / 缺 matplotlib / 写不出文件。
"""
import argparse
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                                  # noqa: E402
import stage4_layouts as lay                                     # noqa: E402
import plot_table_envelope as pte                                # noqa: E402

#: A3 横向：(宽, 高) 毫米。1:1 的前提是页面真的按这个尺寸输出。
PAGE_A3_MM = (420.0, 297.0)
#: 页面到内容的最小留白（毫米）。
MARGIN_MM = 8.0
#: 图与右侧说明栏之间的缝（毫米）。
GUTTER_MM = 5.0
#: 右侧说明栏宽度（毫米）。拾取区 325×250 mm 在 A3 上只剩 95 mm 横向余量，
#: 8+5+74 = 87 mm 之外还剩 8 mm 右边距 —— 再宽就放不下 1:1 了。
INFO_W_MM = 74.0

#: 摆位图上的块的边长：与布局表同源，不再写第二个数。
BLOCK_SIZE_MM = lay.BLOCK_SIZE_M * 1000.0

#: 本机 WSL 实测没有任何 fc-list 中文字体；Windows 的雅黑可直读。
#: 环境变量 ``STAGE4_SHEET_FONT`` 可换一份（板上/别的机器）。
WINDOWS_CJK_FONT = "/mnt/c/Windows/Fonts/msyh.ttc"

#: 颜色中英对照。只影响图上的标注，不影响任何判据。
COLOR_ZH = {
    "red": "红", "orange": "橙", "tan": "棕褐", "yellow": "黄",
    "yellowgreen": "黄绿", "cyan": "青", "purple": "紫",
}

#: CJK 字体按优先级挑 —— 只有一份实现，这里只做别名（AGENTS §3：一个事实一个家）。
pick_font = pte.pick_font


# ----------------------------------------------------------------------
# 纯几何（自检只测这一段，所以**不能**依赖 matplotlib）
# ----------------------------------------------------------------------
def block_corners(x_mm, y_mm, yaw_deg, size_mm=BLOCK_SIZE_MM):
    """块的四角（base 毫米），顺序：局部 (−,−) (+ ,−) (+,+) (−,+) 逆时针。"""
    h = float(size_mm) / 2.0
    t = math.radians(float(yaw_deg))
    c, s = math.cos(t), math.sin(t)
    out = []
    for lx, ly in ((-h, -h), (h, -h), (h, h), (-h, h)):
        out.append((float(x_mm) + c * lx - s * ly,
                    float(y_mm) + s * lx + c * ly))
    return out


def block_diagonal(x_mm, y_mm, yaw_deg, size_mm=BLOCK_SIZE_MM):
    """一条对角线（角 0 → 角 2）：画出来给人对朝向。

    方块转 90° 物理上等价，所以这条线只编码 yaw mod 180° —— 摆块时够用。
    """
    c = block_corners(x_mm, y_mm, yaw_deg, size_mm)
    return c[0], c[2]


def sheet_geometry(zone_mm, page_mm=PAGE_A3_MM, margin_mm=MARGIN_MM,
                   info_w_mm=INFO_W_MM, gutter_mm=GUTTER_MM):
    """页面排版：拾取区 1:1 摆在左侧，说明栏在右侧。放不下就抛 ``ValueError``。

    ``zone_mm`` 的顺序与 `cm.PICK_ZONE` / `stage4_truth_from_frame --zone` 一致：
    **(x0, x1, y0, y1)**，不是矩形四点的绕行顺序。

    返回的 ``offset_mm`` 是 base 毫米 → 纸面毫米的**纯平移**（x、y 各自加一个
    常数），这就是「1:1」。别在这里引入比例因子。
    """
    x0, x1, y0, y1 = [float(v) for v in zone_mm]
    pw, ph = [float(v) for v in page_mm]
    if not (x1 > x0 and y1 > y0):
        raise ValueError("拾取区不是正矩形：%r（顺序 x0,x1,y0,y1）" % (zone_mm,))
    zw, zh = x1 - x0, y1 - y0
    avail_w = pw - 2.0 * margin_mm - gutter_mm - info_w_mm
    avail_h = ph - 2.0 * margin_mm
    if zw > avail_w + 1e-9:
        raise ValueError(
            "拾取区宽 %.1f mm > 页面可用 %.1f mm —— 1:1 放不下；"
            "换更大纸张或缩窄说明栏，不许缩图" % (zw, avail_w))
    if zh > avail_h + 1e-9:
        raise ValueError(
            "拾取区高 %.1f mm > 页面可用 %.1f mm —— 1:1 放不下" % (zh, avail_h))
    zpx = margin_mm
    zpy = margin_mm + (avail_h - zh) / 2.0
    return {
        "page_mm": (pw, ph),
        "zone_mm": (x0, x1, y0, y1),
        "paper_zone_mm": (zpx, zpy, zpx + zw, zpy + zh),
        "offset_mm": (zpx - x0, zpy - y0),
        "scale": 1.0,
        "info_rect_mm": (margin_mm + avail_w + gutter_mm, margin_mm,
                         info_w_mm, avail_h),
    }


def paper_point(x_mm, y_mm, geom):
    """base 毫米 → 纸面毫米（纯平移）。"""
    ox, oy = geom["offset_mm"]
    return (float(x_mm) + ox, float(y_mm) + oy)


def paper_delta(d_mm, geom):
    """图上 1:1 ⇒ 长度直接是它自己。留着这个函数是为了让测试有个断言点。"""
    return float(d_mm) * geom["scale"]


def scale_bar_points(geom, length_mm=100.0):
    """100 mm 比例尺的两端（纸面毫米），钉在拾取区正下方。"""
    x0, y0, _x1, _y1 = geom["paper_zone_mm"]
    y = y0 - 10.0
    return (x0, y), (x0 + paper_delta(length_mm, geom), y)


def check_doc(doc, geom):
    """自检：块在拾取区内、四角与中心都落在纸面上的拾取区框里。返回问题列表。"""
    errs = []
    zx0, zx1, zy0, zy1 = geom["zone_mm"]
    px0, py0, px1, py1 = geom["paper_zone_mm"]
    for b in doc["blocks"]:
        m = doc["block_size_mm"] / 2.0
        if not (zx0 + m <= b["x_mm"] <= zx1 - m
                and zy0 + m <= b["y_mm"] <= zy1 - m):
            errs.append("块 %s 出了拾取区（%.1f, %.1f mm）"
                        % (b["id"], b["x_mm"], b["y_mm"]))
        for cx, cy in block_corners(b["x_mm"], b["y_mm"], b["yaw_deg"],
                                    doc["block_size_mm"]):
            px, py = paper_point(cx, cy, geom)
            if not (px0 - 1e-6 <= px <= px1 + 1e-6
                    and py0 - 1e-6 <= py <= py1 + 1e-6):
                errs.append("块 %s 的角在纸面上越出拾取区框（%.1f, %.1f mm）"
                            % (b["id"], px, py))
                break
    return errs


def sheet_doc(batch, layout):
    """一个布局 → 摆位图用的文档（毫米口径，纯数据）。"""
    blocks = lay.layout_blocks(batch, layout)
    return {
        "batch": batch,
        "layout": int(layout),
        "block_size_mm": BLOCK_SIZE_MM,
        "zone_mm": [v * 1000.0 for v in cm.PICK_ZONE],
        "blocks": [{
            "id": b["id"],
            "color": b["color"],
            "layer": int(b.get("layer", 0)),
            "x_mm": b["x"] * 1000.0,
            "y_mm": b["y"] * 1000.0,
            "yaw_deg": b["yaw"],
        } for b in blocks],
    }


def info_lines(doc, zh=True):
    """说明栏文字。返回行列表（不含排版换行）。"""
    z = doc["zone_mm"]
    rows = []
    if zh:
        rows.append("%s 布局 %d —— %d 块" % (doc["batch"], doc["layout"],
                                             len(doc["blocks"])))
        rows.append("")
        rows.append("拾取区 base x %.0f–%.0f，y %.0f–%.0f mm"
                    % (z[0], z[1], z[2], z[3]))
        rows.append("块边长 %.1f mm（图上是真实大小）" % doc["block_size_mm"])
        rows.append("")
        rows.append("id  颜色       层    x      y    yaw")
        for b in sorted(doc["blocks"], key=lambda d: d["id"]):
            zh_name = COLOR_ZH.get(b["color"], b["color"])
            rows.append("%d   %s %-9s L%d %6.1f %6.1f %5.1f°"
                        % (b["id"], zh_name, b["color"],
                           int(b.get("layer", 0)),
                           b["x_mm"], b["y_mm"], b["yaw_deg"]))
        rows.append("")
        rows += [
            "怎么用",
            "1. 打印选「实际大小 / 100%」，",
            "   关掉「适合页面」。",
            "2. 先用下面 100 mm 比例尺核一遍；",
            "   差 >1 mm 就重打。",
            "3. 图纸与桌上已有的拾取区边界对齐。",
            "4. 在每块中心的十字处用铅笔点一点。",
            "5. 收走图纸，再按点摆块。",
            "   纸和胶带不能压在块下。",
            "6. 摆位只需大致：±3–5 mm / ±10°。",
        ]
    else:
        rows.append("%s layout %d — %d blocks" % (doc["batch"], doc["layout"],
                                                  len(doc["blocks"])))
        rows.append("")
        rows.append("Pick zone base x %.0f-%.0f, y %.0f-%.0f mm"
                    % (z[0], z[1], z[2], z[3]))
        rows.append("Block edge %.1f mm (true size on paper)" % doc["block_size_mm"])
        rows.append("")
        rows.append("id  colour      x      y    yaw")
        for b in sorted(doc["blocks"], key=lambda d: d["id"]):
            rows.append("%d   %-10s %6.1f %6.1f %5.1f"
                        % (b["id"], b["color"], b["x_mm"], b["y_mm"],
                           b["yaw_deg"]))
        rows.append("")
        rows += [
            "How to use",
            "1. Print at 100% (actual size),",
            "   not 'fit to page'.",
            "2. Check the 100 mm scale bar below;",
            "   reprint if off by >1 mm.",
            "3. Align with the pick-zone border",
            "   already taped on the table.",
            "4. Pencil a dot at each centre cross.",
            "5. Remove the sheet, then place",
            "   blocks. No paper under blocks.",
            "6. Rough placement is enough:",
            "   +-3-5 mm / +-10 deg.",
        ]
    return rows


# ----------------------------------------------------------------------
# 画图（matplotlib 只在这里 import）
# ----------------------------------------------------------------------
def setup_font():
    """挑一份 CJK 字体；一份都没有 ⇒ 返回 None（整张图退英文）。"""
    from matplotlib import font_manager
    for path in (os.environ.get("STAGE4_SHEET_FONT") or "", WINDOWS_CJK_FONT):
        if path and os.path.exists(path):
            try:
                font_manager.fontManager.addfont(path)
            except Exception as exc:                      # noqa: BLE001
                print("!! 注册字体失败 %s：%s" % (path, exc))
    return pte.pick_font({f.name for f in font_manager.fontManager.ttflist})

def draw_page(fig, geom, doc, font):
    """把一页画进 ``fig``。坐标单位就是毫米。"""
    from matplotlib.patches import Polygon, Rectangle

    pw, ph = geom["page_mm"]
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax.set_xlim(0.0, pw)
    ax.set_ylim(0.0, ph)
    ax.set_aspect("equal")
    ax.axis("off")

    # 纸面矩形是普通 (x0, y0, x1, y1)；base 口径的区是 (x0, x1, y0, y1)
    zx0, zy0, zx1, zy1 = geom["paper_zone_mm"]
    bx0, _bx1, by0, _by1 = geom["zone_mm"]

    # 拾取区边框（现场对齐用）
    ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0,
                           fill=False, lw=1.6, ec="#222222"))

    # 50 mm 网格（以拾取区左下角为原点）
    gx = zx0
    while gx <= zx1 + 1e-9:
        ax.plot([gx, gx], [zy0, zy1], color="#b8b8b8", lw=0.35, zorder=0)
        gx += 50.0
    gy = zy0
    while gy <= zy1 + 1e-9:
        ax.plot([zx0, zx1], [gy, gy], color="#b8b8b8", lw=0.35, zorder=0)
        gy += 50.0

    # 每 100 mm 在框内侧标一个坐标（浅灰，别挡块）
    fx = 0.0
    while zx0 + fx * 100.0 <= zx1 + 1e-9:
        ax.text(zx0 + fx * 100.0 + 1.2, zy0 + 1.5, "%.0f" % (bx0 + fx * 100.0),
                fontsize=4.5, color="#777777", ha="left", va="bottom")
        fx += 1.0
    fy = 0.0
    while zy0 + fy * 100.0 <= zy1 + 1e-9:
        ax.text(zx0 + 1.2, zy0 + fy * 100.0 + 1.5, "%.0f" % (by0 + fy * 100.0),
                fontsize=4.5, color="#777777", rotation=90,
                ha="left", va="bottom")
        fy += 1.0

    # 块：真实大小轮廓 + 对角标记 + 中心十字 + 编号
    for b in doc["blocks"]:
        corners = [paper_point(cx, cy, geom)
                   for cx, cy in block_corners(b["x_mm"], b["y_mm"],
                                               b["yaw_deg"], doc["block_size_mm"])]
        ax.add_patch(Polygon(corners, closed=True, lw=1.0, ec="#000000",
                             fc=b["color"], alpha=0.30, zorder=2))
        ax.add_patch(Polygon(corners, closed=True, lw=1.0, ec="#000000",
                             fill=False, zorder=3))
        (d0x, d0y), (d1x, d1y) = block_diagonal(
            b["x_mm"], b["y_mm"], b["yaw_deg"], doc["block_size_mm"])
        p0 = paper_point(d0x, d0y, geom)
        p1 = paper_point(d1x, d1y, geom)
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#000000",
                lw=0.6, zorder=3)
        cx, cy = paper_point(b["x_mm"], b["y_mm"], geom)
        ax.plot([cx - 2.5, cx + 2.5], [cy, cy], color="#000000", lw=0.8,
                zorder=4)
        ax.plot([cx, cx], [cy - 2.5, cy + 2.5], color="#000000", lw=0.8,
                zorder=4)
        tag = str(b["id"])
        if int(b.get("layer", 0)) > 0:
            tag += "L%d" % int(b.get("layer", 0))      # 塔上层：俯视图上标 L1
        ax.text(cx, cy + 5.0, tag, fontsize=6, ha="center",
                va="bottom", zorder=4)

    # 100 mm 比例尺（打印后第一个要核对的东西）
    (sx0, sy0), (sx1, sy1) = scale_bar_points(geom)
    ax.plot([sx0, sx1], [sy0, sy1], color="#000000", lw=2.0)
    ax.plot([sx0, sx0], [sy0 - 2.0, sy0 + 2.0], color="#000000", lw=0.8)
    ax.plot([sx1, sx1], [sy0 - 2.0, sy0 + 2.0], color="#000000", lw=0.8)
    ax.text((sx0 + sx1) / 2.0, sy0 - 3.5,
            "100 mm" if font else "100 mm scale",
            fontsize=6, ha="center", va="top")

    # 右栏说明（坐标单位是毫米；字号是磅，行距要按 1 pt = 0.3528 mm 换算）
    ix, iy, iw, ih = geom["info_rect_mm"]
    ax.add_patch(Rectangle((ix, iy), iw, ih, fill=False, lw=0.4,
                           ec="#999999"))
    y = iy + ih - 3.0
    for line in info_lines(doc, zh=bool(font)):
        head = y > iy + ih - 10.0
        size = 9.0 if head else 5.8
        ax.text(ix + 2.0, y, line, fontsize=size,
                fontweight="bold" if head else "normal",
                ha="left", va="top", family=font or "DejaVu Sans")
        y -= size * 0.3528 * 1.30
    return ax


def draw(docs, out, dpi=300):
    """``docs`` 里每个布局一页；``.pdf`` 多页，其他扩展名只允许一页。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    font = setup_font()
    if not font:
        print("!! 没找到中文字体 —— 图上改用英文文案（可用 STAGE4_SHEET_FONT 指定）")
    if len(docs) > 1 and not out.lower().endswith(".pdf"):
        print("!! %d 个布局要输出到非 PDF 文件 —— 只有第一页会被保存" % len(docs))
    if font:
        plt.rcParams["font.sans-serif"] = [font] + \
            list(plt.rcParams.get("font.sans-serif", []))
        plt.rcParams["axes.unicode_minus"] = False

    page_w, page_h = PAGE_A3_MM
    figs = []
    pdf = None
    try:
        from matplotlib.backends.backend_pdf import PdfPages
        if out.lower().endswith(".pdf"):
            pdf = PdfPages(out)
        for i, doc in enumerate(docs):
            geom = sheet_geometry(doc["zone_mm"], PAGE_A3_MM)
            errs = check_doc(doc, geom)
            if errs:
                print("!! %s:%d 摆位图自检不通过：" % (doc["batch"], doc["layout"]))
                for e in errs:
                    print("   " + e)
                return 2
            fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4))
            figs.append(fig)
            draw_page(fig, geom, doc, font)
            if pdf is not None:
                pdf.savefig(fig)
            elif i == 0:
                fig.savefig(out, dpi=dpi)
    finally:
        if pdf is not None:
            pdf.close()
        for fig in figs:
            plt.close(fig)
    print("已写出 %s（%d 页，A3 横向 %.0f×%.0f mm，1:1）"
          % (out, len(docs), page_w, page_h))
    return 0


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
def parse_layouts(values, batch):
    if not values:
        return sorted(lay.BATCHES[batch])
    return sorted(dict.fromkeys(int(v) for v in values))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="阶段 4 摆位图（A3 1:1；打印必须 100%）")
    ap.add_argument("--batch", default="S0", choices=sorted(lay.BATCHES))
    ap.add_argument("--layout", type=int, action="append", default=None,
                    help="可重复；缺省 = 该批全部选中布局")
    ap.add_argument("--out", default="stage4-layout-sheet.pdf",
                    help=".pdf = 每布局一页；其他扩展名只允许一个布局")
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args(argv)

    try:
        layouts = parse_layouts(args.layout, args.batch)
        docs = [sheet_doc(args.batch, l) for l in layouts]
    except (KeyError, ValueError) as exc:
        print("!! %s" % exc)
        return 2
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if not os.path.isdir(out_dir):
        print("!! 输出目录不存在：%s" % out_dir)
        return 2
    try:
        return draw(docs, args.out, dpi=args.dpi)
    except ImportError as exc:
        print("!! 没有 matplotlib：%s" % exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
