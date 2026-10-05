#!/usr/bin/env python3
"""
YOLO 标注工具 (matplotlib 版)
=============================
Alt+拖动画框，比 OpenCV 鼠标回调更稳定。

用法:
    python3 scripts/label_blocks.py
    python3 scripts/label_blocks.py --classes red,yellow,cyan

操作:
    Alt+左键拖动  — 画框（画出来的框属于**当前类别**）
    1..9          — 切换当前类别（框的颜色跟着变）
    右键           — 删最后一个框
    Space / D / → — 下一张
    A / ←         — 上一张
    C             — 清除所有框
    Q / ESC       — 退出

**多类别**：新画的框归到「当前类别」，键 1..9 切换。切换前先画完同一类，
比画完再逐个改省事得多。已经落盘的标签重新打开时**会读回类别号**，
不会把类别抹掉。

⚠️ 类别号 = `--classes` 列表里的**下标**，必须与
`generate_grid_world.py --classes` / `build_dataset.py --class-names`
传同一个列表、同一顺序 —— 顺序错了标签不会报错，只是语义全反。

输出: block_dataset/train/labels/img_XXXX.txt
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from block_classes import DEFAULT_CLASSES, PALETTE, parse_classes, describe  # noqa: E402

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(PROJECT_ROOT, "block_dataset", "train", "images")
LBL_DIR = os.path.join(PROJECT_ROOT, "block_dataset", "train", "labels")

_ap = argparse.ArgumentParser("手工标注方块（多类别）")
_ap.add_argument("--classes", default=",".join(DEFAULT_CLASSES),
                 help="类别表，逗号分隔，**顺序即类别号**（默认 %(default)s）")
_ap.add_argument("--images", default=IMG_DIR, help="图像目录")
_ap.add_argument("--labels", default=LBL_DIR, help="标签输出目录")
_args = _ap.parse_args()
try:
    CLASS_NAMES = parse_classes(_args.classes)
except ValueError as e:
    sys.exit("❌ --classes %s" % e)
IMG_DIR, LBL_DIR = _args.images, _args.labels

# 当前类别：新画的框都归它。画框时框的颜色就用该类在调色板里的颜色 ——
# 于是「这一类长什么样」和「我画的是哪一类」在屏幕上是同一个视觉线索，
# 不必去看标题栏的数字。
cur_class = 0


def class_display_color(cid):
    """类别号 → 画框用的显示色（0~1 的 RGB）。调色板里没有就退回灰。"""
    name = CLASS_NAMES[cid] if 0 <= cid < len(CLASS_NAMES) else None
    return PALETTE.get(name, (0.6, 0.6, 0.6))

img_list = sorted([f for f in os.listdir(IMG_DIR) if f.lower().endswith((".jpg", ".png"))])
if not img_list:
    print(f"No images in {IMG_DIR}")
    sys.exit(1)

idx = 0
boxes = []
rects = []
draw_start = None
current_rect = None

fig, ax = plt.subplots(figsize=(10, 8))
fig.canvas.manager.set_window_title("Label - Alt+Drag画框 D=Next A=Prev C=Clear Q=Quit")


def title(i):
    """标题栏：进度 + 框数 + **当前类别**。当前类别必须一直可见 ——
    它是画框时唯一决定标签内容的状态，藏起来就会出现「画了一堆第 0 类」。
    """
    return (f"[{i+1}/{len(img_list)}] {img_list[i]}  ({len(boxes)} boxes)  "
            f"当前类别 {cur_class}={CLASS_NAMES[cur_class]}")


def load(i):
    global boxes, rects
    path = os.path.join(IMG_DIR, img_list[i])
    img = cv2.imread(path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    ax.clear()
    ax.imshow(img_rgb)
    h, w = img.shape[:2]

    boxes = []
    rects = []
    stem = os.path.splitext(img_list[i])[0]
    lbl_path = os.path.join(LBL_DIR, stem + ".txt")
    if os.path.exists(lbl_path):
        with open(lbl_path) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    # ⚠️ 类别号要**读回来**。旧版把它丢了、写回去时统一写 0 ——
                    # 于是「打开一份多类标签、按一下 D、类别全变 0」，
                    # 而且不留任何痕迹。
                    cid = int(parts[0])
                    if not 0 <= cid < len(CLASS_NAMES):
                        print(f"  ⚠️ {stem}.txt 里有类别号 {cid}，超出当前类别表"
                              f"（共 {len(CLASS_NAMES)} 类）—— 按 0 处理。"
                              f"类别表选错了？")
                        cid = 0
                    cx, cy, bw, bh = map(float, parts[1:])
                    boxes.append((cid, cx, cy, bw, bh))
                    x1 = (cx - bw / 2) * w
                    y1 = (cy - bh / 2) * h
                    col = class_display_color(cid)
                    r = Rectangle((x1, y1), bw * w, bh * h,
                                  fill=False, edgecolor=col, linewidth=2)
                    rects.append(r)
                    ax.add_patch(r)
                    ax.text(x1, y1 - 5, CLASS_NAMES[cid], color=col, fontsize=8)

    ax.set_title(title(i), fontsize=10)
    ax.axis("off")
    fig.canvas.draw_idle()


def save():
    if not img_list:
        return
    stem = os.path.splitext(img_list[idx])[0]
    path = os.path.join(LBL_DIR, stem + ".txt")
    with open(path, "w") as f:
        for cid, cx, cy, bw, bh in boxes:
            f.write(f"{cid} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
    counts = {}
    for cid, *_ in boxes:
        counts[cid] = counts.get(cid, 0) + 1
    detail = "  ".join(f"{CLASS_NAMES[c]}={n}" for c, n in sorted(counts.items()))
    print(f"  💾 {stem}.txt ({len(boxes)} boxes)  {detail}")


def on_press(event):
    global draw_start, current_rect
    if event.inaxes != ax:
        return
    if event.key == "alt" and event.button == 1:
        draw_start = (event.xdata, event.ydata)


def on_motion(event):
    global current_rect
    if draw_start is None or event.inaxes != ax:
        return
    x1, y1 = draw_start
    x2, y2 = event.xdata, event.ydata
    rw, rh = abs(x2 - x1), abs(y2 - y1)
    if current_rect:
        current_rect.remove()
    current_rect = Rectangle((min(x1, x2), min(y1, y2)), rw, rh,
                             fill=False, edgecolor="red", linewidth=2, linestyle="--")
    ax.add_patch(current_rect)
    fig.canvas.draw_idle()


def on_release(event):
    global draw_start, current_rect, boxes, rects
    if draw_start is None:
        return
    x1, y1 = draw_start
    x2, y2 = event.xdata, event.ydata
    draw_start = None
    if current_rect:
        current_rect.remove()
        current_rect = None
    rw, rh = abs(x2 - x1), abs(y2 - y1)
    if rw < 5 or rh < 5:
        return
    h, w = ax.images[0].get_array().shape[:2]
    cx = ((min(x1, x2) + max(x1, x2)) / 2) / w
    cy = ((min(y1, y2) + max(y1, y2)) / 2) / h
    bw_box = rw / w
    bh_box = rh / h
    boxes.append((cur_class, cx, cy, bw_box, bh_box))
    rx, ry = min(x1, x2), min(y1, y2)
    col = class_display_color(cur_class)
    r = Rectangle((rx, ry), rw, rh, fill=False, edgecolor=col, linewidth=2)
    rects.append(r)
    ax.add_patch(r)
    ax.text(rx, ry - 5, CLASS_NAMES[cur_class], color=col, fontsize=8)
    ax.set_title(title(idx), fontsize=10)
    fig.canvas.draw_idle()


def undo_one():
    """删最后一个框（框与矩形是并行的两个列表，必须一起动）。"""
    if not boxes:
        return False
    boxes.pop()
    if rects:
        rects[-1].remove()
        rects.pop()
    ax.set_title(title(idx), fontsize=10)
    fig.canvas.draw_idle()
    return True


def on_key(event):
    global idx, cur_class
    if event.key in (" ", "d", "right"):
        save()
        if idx < len(img_list) - 1:
            idx += 1
            load(idx)
    elif event.key in ("a", "left"):
        save()
        if idx > 0:
            idx -= 1
            load(idx)
    # 数字键切换当前类别。matplotlib 把数字键报成 "1".."9"（小键盘是 "1" 等）。
    # ⚠️ 只认 1..9，所以类别数上限是 9 —— 上限本身由 parse_classes 挡不住，
    #    在这里明确说清楚，免得第 10 类画出来却是第 0 类。
    elif event.key and len(event.key) == 1 and event.key.isdigit():
        n = int(event.key)
        if n == 0:
            print("  ⚠️ 类别号从 1 开始编号（按 1 选第一类）；0 不是有效按键")
            return
        if n > len(CLASS_NAMES):
            print(f"  ⚠️ 只有 {len(CLASS_NAMES)} 类，按 {n} 无效")
            return
        cur_class = n - 1
        ax.set_title(title(idx), fontsize=10)
        fig.canvas.draw_idle()
        print(f"  🏷 当前类别 → {cur_class}={CLASS_NAMES[cur_class]}")
    elif event.key == "c":
        boxes.clear()
        rects.clear()
        for p in ax.patches[:]:
            p.remove()
        ax.set_title(title(idx), fontsize=10)
        fig.canvas.draw_idle()
        print("  🗑 cleared")
    elif event.key in ("q", "escape"):
        save()
        plt.close()
    elif event.key in ("backspace", "delete"):
        if undo_one():
            print(f"  ↩ {len(boxes)} boxes")
    elif event.button == 3:
        if undo_one():
            print(f"  ↩ {len(boxes)} boxes")


os.makedirs(LBL_DIR, exist_ok=True)

if len(CLASS_NAMES) > 9:
    sys.exit(f"❌ 类别表有 {len(CLASS_NAMES)} 类，但本工具的切换键只有 1..9。\n"
             f"   要么把类别缩到 9 类以内，要么改这里的按键方案。")
print(f"""
{'='*50}
  YOLO Label Tool (matplotlib) —— 多类别
  {len(img_list)} images, {len(CLASS_NAMES)} 类：{describe(CLASS_NAMES)}
  Alt+Drag = 画框（归当前类别）  |  1..9 = 切类别
  D/→ = next  |  A/← = prev  |  Backspace = undo  |  C = clear  |  Q = quit
{'='*50}
""")

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("key_press_event", on_key)

load(0)
plt.tight_layout()
plt.show()

print(f"\nDone. Labels: {LBL_DIR}")
print("Next: python3 scripts/train_yolo_block.py")
