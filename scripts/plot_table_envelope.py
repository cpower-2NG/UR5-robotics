#!/usr/bin/env python3
"""把 `table_safety_envelope.py` 的 JSON 画成**一张台面图**（现场「照着划区」用）。

为什么要有它：包线脚本吐的是 ASCII 图 + 每行边界表，**对着终端划区是可以的**，
但要拿它当「桌上摊开的那张图」（打印、贴在工作台边上、或者和相机的可视范围叠一起看）
就不好用。这个脚本只做一件事：把同一个 JSON 画成 PNG。

**它不重新判读任何东西。** 颜色、分类、最坏格全都直接来自 JSON ——
一份判据在 `table_safety_envelope.py` 里，这里只是把它画出来。所以：
改了包线的口径而没重跑，这张图会跟着一起过时（它不校验）。

✅ **不碰 ROS、不连机械臂、不需要 source**：只读 JSON。
   所以**本机 WSL 也能跑**（只要有 matplotlib）。
   matplotlib **只在 `draw()` 里 import** —— 这样 `parse_zone()` 这类纯函数
   能被 `test_plot_table_envelope.py` 在没有 matplotlib 的环境里直接测。

⚠️ 中文字体：matplotlib 默认字体**没有中文**，缺字体时字会变成方框（而且不报错）。
   脚本会自己找一份 CJK 字体；**一份都找不到就整体退回英文文案并打印告警** ——
   宁可换语言，也不给一张满是「豆腐块」的图。
   板上（Orin）实测有 89 个 CJK 字体、matplotlib 3.10.8。

跑法：
    # 本机或板上都行，JSON 是包线那次跑出来的
    python3 scripts/plot_table_envelope.py --json /tmp/env84.json --out docs/桌面安全包线.png
    # 用哪一层：worst（默认，= 包线那张 ASCII 图）或 grasp_approach 这种单层
    python3 scripts/plot_table_envelope.py --json ... --map grasp_approach
    # 把「现在配的区」叠上去看它落在哪儿
    python3 scripts/plot_table_envelope.py --json ... \\
        --zone '现状 pick_zone:0.30,0.90,-0.18,0.16' \\
        --slot 0.44,+0.24 --slot 0.60,+0.24 --slot 0.44,-0.24
退出码 0 = 画出图了；2 = 读不到 JSON / 没有 matplotlib / JSON 里没有可画的格子。
"""

import argparse
import json
import os
import sys

# ----------------------------------------------------------------------
# 分类：与 table_safety_envelope.py 同一套符号，别在这里另发明一套
# ----------------------------------------------------------------------
MISSING = "?"                      # JSON 里没有这个格子（不该发生，但要画得出来）
SYM_ORDER = ["X", "s", "x", "~", ".", MISSING]          # 图例顺序：坏 → 好
SYM_COLOR = {
    "X": "#e8a3a3",                # 撞桌
    "s": "#c6d8f2",                # 种子问题
    "x": "#dcdcdc",                # 够不到
    "~": "#ffe6a6",                # 近限
    ".": "#cfe6cf",                # 安全
    MISSING: "#ffffff",
}
SYM_LABEL_ZH = {
    "X": "撞桌：有连杆低于下限",
    "s": "换种子才可解（种子问题）",
    "x": "解不出 IK（够不到）",
    "~": "近限：安全但离下限 <{band:.2f} m",
    ".": "安全",
    MISSING: "无数据",
}
# 找不到 CJK 字体时的退路：整张图换英文，别画成方框
SYM_LABEL_EN = {
    "X": "BLOCKED: a link below the floor",
    "s": "solvable only with retry seed",
    "x": "no IK solution (out of reach)",
    "~": "near limit (<{band:.2f} m from floor)",
    ".": "safe",
    MISSING: "no data",
}
# 候选中文字体，按优先级。找不到就退英文（见文件头）。
CJK_CANDIDATES = [
    "Noto Sans CJK SC", "Noto Sans CJK JP", "Noto Sans SC", "Source Han Sans SC",
    "WenQuanYi Zen Hei", "WenQuanYi Micro Hei", "Droid Sans Fallback",
    "AR PL UMing CN", "Microsoft YaHei", "SimHei",
]


# ----------------------------------------------------------------------
# 纯函数（自检只测这一段，所以它们**不能**依赖 matplotlib）
# ----------------------------------------------------------------------
def parse_zone(text):
    """`名字:xmin,xmax,ymin,ymax` → (名字, (xmin,xmax,ymin,ymax))。

    ⚠️ 顺序是 **x 的上下界在前、y 的在后** —— 与 `pick_zone` 参数一致，
       与「矩形四点」那种写法**不一样**，写错了框会横竖颠倒而且不报错。
    ⚠️ 名字**不能含冒号**（按第一个冒号切）：`现状:0,1,2,3` 可以，
       `第1步:现状:0,1,2,3` 会在解析时炸 —— 那种名字用下划线。
    """
    if ":" not in text:
        raise ValueError(f"要写成 名字:xmin,xmax,ymin,ymax，收到 {text!r}")
    name, nums = text.split(":", 1)
    vs = [float(v) for v in nums.split(",")]
    if len(vs) != 4:
        raise ValueError(f"要 4 个数（xmin,xmax,ymin,ymax），收到 {len(vs)} 个：{text!r}")
    xmin, xmax, ymin, ymax = vs
    if xmin > xmax or ymin > ymax:
        raise ValueError(f"下界比上界大：{text!r}")
    return name, (xmin, xmax, ymin, ymax)


def parse_slot(text):
    """`x,y` → (x, y)。"""
    vs = [float(v) for v in text.split(",")]
    if len(vs) != 2:
        raise ValueError(f"槽位要 x,y 两个数，收到 {len(vs)} 个：{text!r}")
    return (vs[0], vs[1])


def edges(vals):
    """格心 → 格边（首尾各外推半格）。作图要的是边，不是心。"""
    if not vals:
        return []
    if len(vals) == 1:
        d = 0.05
        return [vals[0] - d / 2, vals[0] + d / 2]
    out = [vals[0] - (vals[1] - vals[0]) / 2]
    for a, b in zip(vals, vals[1:]):
        out.append((a + b) / 2)
    out.append(vals[-1] + (vals[-1] - vals[-2]) / 2)
    return out


def codes(data, which):
    """把 JSON 的 cells/per_level 摊成 (xs, ys, sym 二维表)。

    `which` = `"worst"` 取 `cells`（所有高度取最坏，与 ASCII 图一致），
    否则取 `per_level[which]`。
    """
    src = data["cells"] if which == "worst" else data["per_level"][which]
    xs, ys = list(data["xs"]), list(data["ys"])
    sym = {(float(k.split(",")[0]), float(k.split(",")[1])): v
           for k, v in src.items()}
    return xs, ys, sym


def pick_font(available_names):
    """在**已经拿到的字体名集合**里挑一个 CJK 字体，没有就 None。

    单拎出来是为了能被自检覆盖 —— 「有没有字体」这件事本身没法在测试里假装，
    但「按优先级挑」这段逻辑可以。
    """
    for want in CJK_CANDIDATES:
        if want in available_names:
            return want
    return None


# ----------------------------------------------------------------------
# 作图
# ----------------------------------------------------------------------
LEVEL_ZH = {
    "grasp_descend": "抓取下降", "grasp_approach": "抓取接近",
    "place_descend": "放置下降", "place_approach": "放置接近",
}
LEVEL_EN = {
    "grasp_descend": "grasp descend", "grasp_approach": "grasp approach",
    "place_descend": "place descend", "place_approach": "place approach",
}


def _title_lines(data, which, zh=True):
    """两行标题：(这是什么层, 参数来源)。**不改判读**，只是把 JSON 里的数字摆出来。

    ⚠️ 必须跟着字体走：没有中文字体时**标题也得是英文** —— 第一版把标题写死了
       中文，结果图上下面全英文、顶上两行方框，一眼看上去像图坏了。
    """
    if which == "worst":
        head = (f"桌面安全包线：所有高度取最坏（{len(data['levels'])} 层合成）" if zh else
                f"Table safety envelope: worst over all {len(data['levels'])} heights")
    else:
        name = (LEVEL_ZH if zh else LEVEL_EN).get(which, which)
        head = (f"桌面安全包线：只有「{name}」这一层（{data['levels'][which]:+.4f} m）"
                if zh else
                f"Table safety envelope: only the '{name}' level "
                f"({data['levels'][which]:+.4f} m)")
    # 副标题折成**短行**：一行太长会顶出坐标区、被裁掉（实测第一版就是这样，
    # 最左边那几个字符直接没了 —— 而且 matplotlib 不报错）。
    sub = ((f"台面 z = {data['table_z']:+.4f} m（base_frame）\n"
            f"下限 = 台面 + 余量 {data['guard_margin']:.3f} = {data['floor']:+.4f} m\n"
            f"判据看这两个连杆的原点高度：{', '.join(data['guard_links'])}") if zh else
           (f"table z = {data['table_z']:+.4f} m (base_frame)\n"
            f"floor = table + margin {data['guard_margin']:.3f} = {data['floor']:+.4f} m\n"
            f"judged on the link ORIGIN height of: {', '.join(data['guard_links'])}"))
    return head, sub


def draw(data, which, zones, slots, out, title, dpi):
    import matplotlib
    matplotlib.use("Agg")                       # 无显示环境（板上/无 X）也能出图
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    xs, ys, sym = codes(data, which)
    if not xs or not ys:
        print("!! JSON 里没有格子可画")
        return 2

    # 字体：找不到 CJK 就**整张图换英文**。缺字体时 matplotlib 会把中文画成方框，
    # 而且一句警告都不给 —— 一张满是方框的图比一张英文图糟得多。
    have = {f.name for f in font_manager.fontManager.ttflist}
    font = pick_font(have)
    labels = SYM_LABEL_ZH if font else SYM_LABEL_EN
    if font:
        plt.rcParams["font.sans-serif"] = [font]
        plt.rcParams["axes.unicode_minus"] = False
    else:
        print("!! 没找到中文字体 —— 这张图改用英文文案。"
              "（板上 89 个 CJK 字体，本机 WSL 一个都没有）")
        bad = [n for n, _ in zones if any(ord(c) > 127 for c in n)]
        if bad:
            # 图里自己的文案能换英文，**区名是命令行给的**，换不了 —— 只能提醒
            print(f"!! 而且你给的区名里有非 ASCII 字符 {bad}：没有中文字体时"
                  f"它们会画成方框。区名请用 ASCII，中文说明写在文档里。")

    band = float(data.get("clearance", 0.05))
    labels = {k: v.format(band=band) for k, v in labels.items()}

    order = [s for s in SYM_ORDER if s != MISSING]
    idx = {s: i for i, s in enumerate(order)}
    n_missing = len(order)
    grid = [[idx.get(sym.get((x, y), MISSING), n_missing) for x in xs] for y in ys]
    cmap = ListedColormap([SYM_COLOR[s] for s in order] + [SYM_COLOR[MISSING]])
    norm = BoundaryNorm(range(len(order) + 2), cmap.N)

    fig, ax = plt.subplots(figsize=(9.4, 7.2), dpi=dpi)
    ax.pcolormesh(edges(xs), edges(ys), grid, cmap=cmap, norm=norm,
                  edgecolors="#8f8f8f", linewidth=0.35)

    # 叠区：现场「照着划区」的用法就是把**现在配的那个区**画上来，
    # 一眼看出它有没有压到撞桌格。
    for name, (x0, x1, y0, y1) in zones:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                               edgecolor="#1a3d7c", linewidth=2.0, zorder=5))
        ax.text(x0, y1 + 0.015, name, color="#1a3d7c", fontsize=10,
                fontweight="bold", zorder=6)
    slot_handle = None
    if slots:
        ax.scatter([s[0] for s in slots], [s[1] for s in slots], marker="x", s=55,
                   linewidths=2.0, color="#7a3ba8", zorder=6)
        slot_handle = Line2D([], [], marker="x", linestyle="none", markersize=8,
                             markeredgewidth=2.0, color="#7a3ba8",
                             label=("放置槽位（--slot）" if font else "place slots (--slot)"))

    if data.get("worst_at"):
        wx, wy = data["worst_at"]
        ax.plot([wx], [wy], marker="o", markersize=9, markerfacecolor="none",
                markeredgecolor="#8a1c1c", markeredgewidth=2.0, zorder=7)
        ax.annotate(
            (f"全场最低连杆 {data['worst_link_z']:+.4f} m\n"
             f"（下限 {data['floor']:+.4f}）" if font else
             f"lowest link {data['worst_link_z']:+.4f} m\n(floor {data['floor']:+.4f})"),
            xy=(wx, wy), xytext=(wx + 0.06, wy - 0.12), fontsize=9, color="#8a1c1c",
            zorder=7, arrowprops=dict(arrowstyle="->", color="#8a1c1c", lw=1.2))

    ax.set_aspect("equal")
    ax.set_xlabel("x（base_frame，m，正前方）" if font else "x (base_frame, m)")
    ax.set_ylabel("y（base_frame，m）" if font else "y (base_frame, m)")
    head, sub = _title_lines(data, which, zh=bool(font))
    ax.set_title((title + "\n" if title else "") + head + "\n" + sub,
                 fontsize=10.5, pad=12)

    # 图例：数量从**画出来的这张表**现数，所以单层图和 worst 图各自对得上
    flat = [s for row in grid for s in row]
    handles = []
    for i, s in enumerate(order):
        n = sum(1 for v in flat if v == i)
        if n:
            handles.append(Patch(facecolor=SYM_COLOR[s], edgecolor="#8f8f8f",
                                 label=f"{s}  {labels[s]}   ×{n}"))
    if sum(1 for v in flat if v == n_missing):
        handles.append(Patch(facecolor=SYM_COLOR[MISSING], edgecolor="#8f8f8f",
                             label=labels[MISSING]))
    if slot_handle is not None:
        handles.append(slot_handle)
    for name, _ in zones:
        handles.append(Line2D([], [], color="#1a3d7c", linewidth=2.0, label=name))
    leg = ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0),
                    fontsize=9.5, frameon=False)
    leg.set_title("判读" if font else "reading", prop={"size": 10.5})

    pitch = xs[1] - xs[0] if len(xs) > 1 else 0.0
    retry = data.get("retry_seed")
    if font:
        foot = (f"网格 {len(xs)}×{len(ys)}，格距 ≈ {pitch:.3f} m；"
                f"范围 x∈[{xs[0]:.2f},{xs[-1]:.2f}] y∈[{ys[0]:+.2f},{ys[-1]:+.2f}]"
                f"（base_frame）\n")
        if retry:
            foot += f"`s` 来自重试种子 {[round(v, 3) for v in retry]}\n"
        foot += ("⚠️ 判据只看连杆原点高度（不含连杆/夹爪本体的半径）；"
                 "也不含相机视野 —— 看得见 ≠ 够得到")
    else:
        foot = (f"grid {len(xs)}x{len(ys)}, pitch ~{pitch:.3f} m; "
                f"x in [{xs[0]:.2f},{xs[-1]:.2f}], y in [{ys[0]:+.2f},{ys[-1]:+.2f}]\n")
        if retry:
            foot += f"`s` from retry seed {[round(v, 3) for v in retry]}\n"
        foot += ("link ORIGIN height only (no link/gripper body radius); "
                 "no camera FOV either - visible != reachable")
    fig.text(0.02, 0.015, foot, fontsize=8.5, color="#4a4a4a", va="bottom")

    fig.tight_layout(rect=(0, 0.075 if not font else 0.055, 0.80, 1))
    fig.savefig(out)
    plt.close(fig)
    print(f"图已写出：{out}（{which}，{len(xs)}×{len(ys)} 格）")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="把 table_safety_envelope.py 的 JSON 画成台面图（只读 JSON，不碰 ROS）")
    ap.add_argument("--json", required=True, help="包线脚本 --json 写出来的那份")
    ap.add_argument("--out", default=None,
                    help="输出 PNG（默认：与 --json 同目录、同名 .png）")
    ap.add_argument("--map", default="worst",
                    help="worst（默认，与包线那张 ASCII 图同口径）或单层 key"
                         "（grasp_descend/grasp_approach/place_descend/place_approach）")
    ap.add_argument("--zone", action="append", default=[], metavar="名字:xmin,xmax,ymin,ymax",
                    help="叠一个矩形区（可重复）。顺序是 **x 界在前**")
    ap.add_argument("--slot", action="append", default=[], metavar="x,y",
                    help="叠一个放置槽位点（可重复）")
    ap.add_argument("--title", default=None)
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    try:
        with open(args.json, encoding="utf-8") as f:
            data = json.load(f)
    except OSError as e:
        print(f"!! 读不到 {args.json}：{e}")
        return 2
    except json.JSONDecodeError as e:
        print(f"!! {args.json} 不是合法 JSON：{e}")
        return 2

    which = args.map
    if which != "worst":
        if which not in data.get("per_level", {}):
            print(f"!! --map {which!r} 不在 JSON 里；可选的单层是 "
                  f"{sorted(data.get('per_level', {}))}，或者 worst")
            return 2

    try:
        zone_list = [parse_zone(z) for z in args.zone]
        slot_list = [parse_slot(s) for s in args.slot]
    except ValueError as e:
        print(f"!! 参数格式不对：{e}")
        return 2

    out = args.out or os.path.splitext(args.json)[0] + ".png"
    return draw(data, which, zone_list, slot_list, out, args.title, args.dpi)


if __name__ == "__main__":
    sys.exit(main())
