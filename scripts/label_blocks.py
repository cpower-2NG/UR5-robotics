#!/usr/bin/env python3
"""
YOLO 标注工具 (matplotlib 版)
=============================
Alt+拖动画框，比 OpenCV 鼠标回调更稳定。

用法:
    python3 scripts/label_blocks.py

操作:
    Alt+左键拖动  — 画框
    右键           — 删最后一个框
    Space / D / → — 下一张
    A / ←         — 上一张
    C             — 清除所有框
    Q / ESC       — 退出

输出: block_dataset/train/labels/img_XXXX.txt
"""

import os
import sys
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(PROJECT_ROOT, "block_dataset", "train", "images")
LBL_DIR = os.path.join(PROJECT_ROOT, "block_dataset", "train", "labels")

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
                    cx, cy, bw, bh = map(float, parts[1:])
                    boxes.append((cx, cy, bw, bh))
                    x1 = (cx - bw / 2) * w
                    y1 = (cy - bh / 2) * h
                    r = Rectangle((x1, y1), bw * w, bh * h,
                                  fill=False, edgecolor="lime", linewidth=2)
                    rects.append(r)
                    ax.add_patch(r)
                    ax.text(x1, y1 - 5, "block", color="lime", fontsize=8)

    ax.set_title(f"[{i+1}/{len(img_list)}] {img_list[i]}  ({len(boxes)} boxes)", fontsize=10)
    ax.axis("off")
    fig.canvas.draw_idle()


def save():
    if not img_list:
        return
    stem = os.path.splitext(img_list[idx])[0]
    path = os.path.join(LBL_DIR, stem + ".txt")
    with open(path, "w") as f:
        for cx, cy, bw, bh in boxes:
            f.write(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
    print(f"  💾 {stem}.txt ({len(boxes)} boxes)")


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
    boxes.append((cx, cy, bw_box, bh_box))
    rx, ry = min(x1, x2), min(y1, y2)
    r = Rectangle((rx, ry), rw, rh, fill=False, edgecolor="lime", linewidth=2)
    rects.append(r)
    ax.add_patch(r)
    ax.text(rx, ry - 5, "block", color="lime", fontsize=8)
    ax.set_title(f"[{idx+1}/{len(img_list)}] {img_list[idx]}  ({len(boxes)} boxes)", fontsize=10)
    fig.canvas.draw_idle()


def on_key(event):
    global idx
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
    elif event.key == "c":
        boxes.clear()
        rects.clear()
        for p in ax.patches[:]:
            p.remove()
        ax.set_title(f"[{idx+1}/{len(img_list)}] {img_list[idx]}  (0 boxes)", fontsize=10)
        fig.canvas.draw_idle()
        print("  🗑 cleared")
    elif event.key in ("q", "escape"):
        save()
        plt.close()
    elif event.key in ("backspace", "delete"):
        if boxes:
            boxes.pop()
            if rects:
                rects[-1].remove()
                rects.pop()
            ax.set_title(f"[{idx+1}/{len(img_list)}] {img_list[idx]}  ({len(boxes)} boxes)", fontsize=10)
            fig.canvas.draw_idle()
            print(f"  ↩ {len(boxes)} boxes")
    elif event.button == 3:
        if boxes:
            boxes.pop()
            if rects:
                rects[-1].remove()
                rects.pop()
            ax.set_title(f"[{idx+1}/{len(img_list)}] {img_list[idx]}  ({len(boxes)} boxes)", fontsize=10)
            fig.canvas.draw_idle()
            print(f"  ↩ {len(boxes)} boxes")


os.makedirs(LBL_DIR, exist_ok=True)

print(f"""
{'='*50}
  YOLO Label Tool (matplotlib)
  {len(img_list)} images, class: block
  Alt+Drag = draw  |  D/→ = next  |  A/← = prev
  Backspace = undo |  C = clear    |  Q = quit
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
