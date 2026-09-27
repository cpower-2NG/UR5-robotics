#!/usr/bin/env python3
"""
批量合成训练数据 + 自动标注
============================
循环多个 seed，每个 seed 生成若干张合成图像（OpenCV 渲染），
同时利用 JSON 元数据自动投影生成 YOLO 标注。

用法:
    python3 scripts/generate_synthetic_data.py

输出: block_dataset/train/images/ + labels/ + data.yaml

⚠️⚠️ **已知问题：本脚本的投影是「转置」的（D61），且渲染与标注共用它。**
--------------------------------------------------------------------
``world_to_pixel()``（见下）**不含任何旋转** —— 它写死成「相机垂直向下、
图像 x ∥ 世界 x、图像 y ∥ 世界 y」。而仿真相机的真实光学约定是
「光心 x+ → 世界 −y、光心 y+ → 世界 −x」（``sim.launch.py:47-48``），
两者差一个转置。可检验的等式（fx=fy 时，u_true = cx+cy−v_old、
v_true = cx+cy−u_old）与实测量级见 ``scripts/test_label_projection.py`` 第 2 节。

**为什么至今没出问题，以及为什么不能因此认为它是对的**：
本函数**同时**用于渲染（桌面多边形 :115、方块顶面 :135）**和**生成标签
（``compute_bbox`` :80），于是图与标签**互相自洽**；而「棕桌上一个彩色方块」
是位置无关的，检测器照样能学。**几何错了，一致性没坏** —— 现有
``block_dataset/`` 与 ``block_best.pt`` 因此仍然有效，**不要因为这段注释就去重训**。

**禁止把它当真机的起点。** 真机的手眼外参必然带旋转，本函数表达不了
（元数据里的 ``camera`` 只有 ``position/width/height/horizontal_fov``，**不带姿态**）。
真机采集与标注请走 ``scripts/collect_training_data.py``（订阅 ROS 话题，采集端已可用）
+ ``scripts/auto_label_grid.py``（``projector_node.py`` 投影链的**严格逆运算**，
含 tf 与单应两条后端）。要不要修本脚本、以及随之而来的重生成 + 重训，
用户已在第十二轮明确决定「**不动它（保持现状）**」—— 见
``docs/RESTRUCTURE_PLAN.md`` 第十二轮记录第六节。
"""

import os
import sys
import json
import math
import random
import shutil
import atexit
import argparse
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np

# ---- 路径 ----
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATASET_DIR = PROJECT_ROOT / "block_dataset"
META_PATH = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "worlds" / "block_grid_world_meta.json"

# ---- 图像参数 ----
IMG_W, IMG_H = 640, 480
TABLE_W, TABLE_H = 0.8, 0.6        # 桌面尺寸 (m)
TABLE_X, TABLE_Y = 0.5, 0.0        # 桌面中心世界坐标
CAM_X, CAM_Y, CAM_Z = 0.5, 0.0, 1.5
BLOCK_SIZE = 0.04                  # 方块边长 (m)
BLOCK_Z = 0.785                    # 方块中心 Z

SEEDS = 40                         # 默认场景数（种子 1..N），--seeds 覆盖
IMAGES_PER_SEED = 5                # 默认每场景张数，--per-seed 覆盖
VAL_SPLIT = 0.2                    # 20% 验证集（无开关，改这里）


def clamp(v, lo=0.0, hi=1.0):
    return max(lo, min(hi, v))


def compute_intrinsics():
    hfov = 1.047
    fx = (IMG_W / 2.0) / math.tan(hfov / 2.0)
    fy = fx
    cx = IMG_W / 2.0
    cy = IMG_H / 2.0
    return fx, fy, cx, cy


def world_to_pixel(wx, wy, wz, fx, fy, cx, cy):
    """⚠️ **这个函数是转置的（D61）**，别当通用投影用，也别拿它当真机的起点。

    它只吃相机的**位置**、不含任何旋转，即写死成「相机垂直向下、
    图像 x ∥ 世界 x、图像 y ∥ 世界 y」。仿真相机的真实光学约定是
    「光心 x+ → 世界 −y、光心 y+ → 世界 −x」，差一个转置。

    它之所以能一直用下去：本脚本**渲染和标注都用它**，图与标签互相自洽，
    而方块检测是位置无关的。**但这是自洽，不是正确。**

    要正确的投影（含旋转、含真机手眼外参），用
    ``scripts/auto_label_grid.py`` —— 那是 ``perception_3d/projector_node.py``
    投影链的严格逆运算。
    """
    Xc = wx - CAM_X
    Yc = wy - CAM_Y
    Zc = CAM_Z - wz
    if Zc <= 0:
        return None
    u = fx * (Xc / Zc) + cx
    v = fy * (Yc / Zc) + cy
    return (u, v)


def compute_bbox(block, fx, fy, cx, cy):
    wx, wy, wz = block["x"], block["y"], block["z"]
    half = BLOCK_SIZE / 2.0
    corners = [
        (wx - half, wy - half, wz - half),
        (wx + half, wy - half, wz - half),
        (wx + half, wy + half, wz - half),
        (wx - half, wy + half, wz - half),
        (wx - half, wy - half, wz + half),
        (wx + half, wy - half, wz + half),
        (wx + half, wy + half, wz + half),
        (wx - half, wy + half, wz + half),
    ]
    pixels = [world_to_pixel(cx_w, cy_w, cz_w, fx, fy, cx, cy)
              for cx_w, cy_w, cz_w in corners]
    pixels = [p for p in pixels if p is not None]
    if not pixels:
        return None
    u_min = max(0, min(p[0] for p in pixels))
    u_max = min(IMG_W - 1, max(p[0] for p in pixels))
    v_min = max(0, min(p[1] for p in pixels))
    v_max = min(IMG_H - 1, max(p[1] for p in pixels))
    if u_max <= u_min or v_max <= v_min:
        return None
    x_center = ((u_min + u_max) / 2.0) / IMG_W
    y_center = ((v_min + v_max) / 2.0) / IMG_H
    width = (u_max - u_min) / IMG_W
    height = (v_max - v_min) / IMG_H
    return (x_center, y_center, width, height)


def render_scene(blocks, noise_level=0.03, brightness=1.0, blur=0):
    """用 OpenCV 渲染俯瞰桌面场景。"""
    img = np.ones((IMG_H, IMG_W, 3), dtype=np.float32) * 0.75  # 浅灰背景

    # 桌面
    fx, fy, cx, cy = compute_intrinsics()
    table_corners = [
        (TABLE_X - TABLE_W/2, TABLE_Y - TABLE_H/2, 0.765),
        (TABLE_X + TABLE_W/2, TABLE_Y - TABLE_H/2, 0.765),
        (TABLE_X + TABLE_W/2, TABLE_Y + TABLE_H/2, 0.765),
        (TABLE_X - TABLE_W/2, TABLE_Y + TABLE_H/2, 0.765),
    ]
    pts = [world_to_pixel(wx, wy, wz, fx, fy, cx, cy) for wx, wy, wz in table_corners]
    pts = [p for p in pts if p is not None]
    if len(pts) >= 4:
        pts_np = np.array(pts, dtype=np.int32)
        cv2.fillPoly(img, [pts_np], (0.45, 0.28, 0.12))  # 木色桌面

    # 方块
    fx, fy, cx, cy = compute_intrinsics()
    for b in blocks:
        wx, wy, wz = b["x"], b["y"], b["z"]
        r, g, bl = b["color"]
        half = BLOCK_SIZE / 2.0

        # 方块顶面四个角
        top_corners = [
            (wx - half, wy - half, wz + half),
            (wx + half, wy - half, wz + half),
            (wx + half, wy + half, wz + half),
            (wx - half, wy + half, wz + half),
        ]
        pts = [world_to_pixel(cx_w, cy_w, cz_w, fx, fy, cx, cy)
               for cx_w, cy_w, cz_w in top_corners]
        pts = [p for p in pts if p is not None]
        if len(pts) >= 4:
            pts_np = np.array(pts, dtype=np.int32)
            cv2.fillPoly(img, [pts_np], (bl, g, r))  # BGR
            # 加边线
            cv2.polylines(img, [pts_np], True, (0.1, 0.1, 0.1), 1)

    # 加噪声
    if noise_level > 0:
        noise = np.random.randn(*img.shape).astype(np.float32) * noise_level
        img = np.clip(img + noise, 0, 1)

    # 亮度变化
    img = np.clip(img * brightness, 0, 1)

    # 高斯模糊
    if blur > 0:
        img = (img * 255).astype(np.uint8)
        img = cv2.GaussianBlur(img, (blur, blur), 0)
        img = img.astype(np.float32) / 255.0

    img = (img * 255).astype(np.uint8)
    return img


def generate_one_scene(seed, world_dir):
    """生成一个 seed 对应的场景，返回 blocks 列表。

    ⚠️ `world_dir` 是**临时草稿目录**，不是 `META_PATH` 所在的
    `ros2_ws/src/yolo_bringup/worlds/`。后者**被 git 跟踪**，原先这里没传
    `--out-dir`，于是**每跑一次本脚本就静默覆写 `block_grid_world.sdf` 与
    `block_grid_world_meta.json`** —— 哪怕 `--out` 已经把数据集指到了别处。
    （实测：`--force --out /tmp/gt` 之后那两个文件仍变成 `M`。）

    另：这里**检查子进程退出码**。原先不看返回值，生成器一旦失败就会去读
    `META_PATH` 里**上一次的残留 meta**，于是「用 A 场景的方块位姿标注 B 场景的图」
    而没有任何报错。
    """
    r = subprocess.run(
        [sys.executable, str(PROJECT_ROOT / "scripts" / "generate_grid_world.py"),
         "--seed", str(seed), "--out-dir", str(world_dir)],
        capture_output=True, text=True, cwd=str(PROJECT_ROOT)
    )
    if r.returncode != 0:
        raise RuntimeError(
            f"generate_grid_world.py --seed {seed} 失败（退出码 {r.returncode}）：\n"
            f"{r.stdout}\n{r.stderr}")
    with open(world_dir / META_PATH.name) as f:
        meta = json.load(f)
    return meta["blocks"]


def main():
    # ⚠️⚠️ **本脚本会删掉重建 `block_dataset/`**（其它什么都不会碰 —— 见下）。
    # 这几条防护是踩过之后加的 —— 原先 `main()` 第一件事就是无条件
    # `shutil.rmtree` 四个输出目录，而**脚本完全没有参数解析**，
    # 于是 `python3 scripts/generate_synthetic_data.py --help`
    # 这种「只想看看用法」的调用会**立刻**清空数据集并开始重生成
    # （实测：一次 `--help` 删掉 284 个文件、改了 19 个，数据集残掉一半）。
    # 数据集**不在** .gitignore 里，所以能 `git checkout` 救回来 ——
    # 但前提是你**当场发现**了；第二批生成一旦把图与标签混进去就再也分不开。
    #
    # 防护：① 有真正的 argparse（`--help` 不再执行任何东西）；
    #       ② 数据集非空时必须显式 `--force`，否则**只报告、不动手**；
    #       ③ 世界草稿走临时目录（`--out-dir`），**不再覆写被跟踪的
    #          `yolo_bringup/worlds/block_grid_world.{sdf,meta.json}`**。
    #
    # ⚠️ **② 的第一版是坏的，而且坏在「我以为它生效了」**：判据写成
    # `out.glob("*/*/images/*.jpg")`，比真实布局 `out/train/images/*.jpg`
    # **多要求一层同名目录**，于是**恒等于 0**、防护一次都没拦过 ——
    # 紧接着的下一次运行照样把数据集清空重建（实测 339 条改动，
    # 全部靠 `git checkout --` 逐路径救回）。**教训：只测「`--help` 能跑」
    # 是不够的，必须测「它该拦的那个场景」。** 现在改用 `rglob`，与深度无关。
    ap = argparse.ArgumentParser(
        description="批量合成训练数据 + 自动标注（会**清空重建** --out 指向的目录）",
        epilog="本脚本只在 --out 目录里读写；世界草稿落在临时目录，"
               "不会碰被 git 跟踪的 ros2_ws/src/yolo_bringup/worlds/。",
    )
    ap.add_argument("--force", action="store_true",
                    help="确认清空重建 block_dataset/（数据集非空时必需）")
    ap.add_argument("--seeds", type=int, default=SEEDS,
                    help=f"用多少个场景（种子 1..N），默认 {SEEDS}")
    ap.add_argument("--per-seed", type=int, default=IMAGES_PER_SEED,
                    help=f"每个场景生成几张，默认 {IMAGES_PER_SEED}")
    ap.add_argument("--out", default=str(DATASET_DIR),
                    help=f"输出目录，默认 {DATASET_DIR}")
    ap.add_argument("--dry-run", action="store_true",
                    help="只说明会做什么，不删不写")
    args = ap.parse_args()

    out = Path(args.out)
    # ⚠️ 数的是 `out` 底下**任意深度**的图/标签（`rglob`），不是固定层数。
    # 这里原先写 `out.glob("*/*/images/*.jpg")` —— 那比真实布局
    # `out/train/images/*.jpg` **多要求一层同名目录**，于是恒等于 0，
    # 防护**一次都没生效过**（详见 main() 开头的注释）。
    # 图与标签**都**算：只有一边残留也算「非空」，同样不能静默清掉。
    n_img = sum(1 for _ in out.rglob("*.jpg")) if out.exists() else 0
    n_lbl = sum(1 for _ in out.rglob("*.txt")) if out.exists() else 0
    n_existing = n_img + n_lbl
    if n_existing and not (args.force or args.dry_run):
        print(f"✋ {out} 里已有 {n_img} 张图 / {n_lbl} 个标签，本脚本会**清空重建**它。\n"
              f"   确认要重生成就加 --force；只想看看用法用 --help。\n"
              f"   （世界草稿会落在临时目录，仓库里的 worlds/ 不受影响。）",
              file=sys.stderr)
        return 2
    if args.dry_run:
        print(f"[dry-run] 将会清空重建 {out}，用 {args.seeds} 个场景 × "
              f"{args.per_seed} 张；世界草稿走临时目录，不碰仓库。")
        return 0
    if n_existing:
        print(f"⚠️ 已按 --force 清空重建 {out}"
              f"（原有 {n_img} 张图 / {n_lbl} 个标签）。")

    # 清空旧数据。⚠️ 四个目录一律**从 `out` 派生** —— 原先这里硬用模块常量
    # `TRAIN_IMG/...`，于是 `--out /tmp/foo` 会被忽略：报告的是 /tmp/foo、
    # 删的却是 `block_dataset/`。与 D50 同类的「参数静默不生效」。
    train_img = out / "train" / "images"
    train_lbl = out / "train" / "labels"
    val_img = out / "val" / "images"
    val_lbl = out / "val" / "labels"
    for d in [train_img, train_lbl, val_img, val_lbl]:
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    # 中间产物（世界 SDF + meta）落在临时目录里，**绝不写进被 git 跟踪的
    # `ros2_ws/src/yolo_bringup/worlds/`** —— 见 generate_one_scene 的注释。
    # 用 mkdtemp + atexit 而不是 try/finally，是为了不给下面整个生成循环
    # 再套一层缩进；异常退出时 atexit 同样会清理。
    world_dir = Path(tempfile.mkdtemp(prefix="synth_world_"))
    atexit.register(shutil.rmtree, world_dir, ignore_errors=True)
    print(f"[info] 世界草稿目录: {world_dir}（结束时自动删除，仓库里的 worlds/ 不动）")

    all_samples = []  # (img_name, is_val)

    img_idx = 0
    for seed in range(1, args.seeds + 1):
        print(f"[seed={seed}] 生成场景...", end=" ", flush=True)
        blocks = generate_one_scene(seed, world_dir)

        fx, fy, cx, cy = compute_intrinsics()

        for k in range(args.per_seed):
            # 随机扰动参数（模拟采集变化）
            noise = random.uniform(0.01, 0.05)
            brightness = random.uniform(0.85, 1.15)
            blur = random.choice([0, 0, 0, 1, 3])  # 大部分清晰，偶尔模糊

            img = render_scene(blocks, noise_level=noise,
                               brightness=brightness, blur=blur)

            img_name = f"img_{img_idx:04d}.jpg"
            cv2.imwrite(str(train_img / img_name), img)

            # 自动标注
            label_lines = []
            for b in blocks:
                bbox = compute_bbox(b, fx, fy, cx, cy)
                if bbox is not None:
                    xc, yc, w, h = bbox
                    label_lines.append(f"0 {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

            lbl_name = f"img_{img_idx:04d}.txt"
            with open(train_lbl / lbl_name, 'w') as f:
                f.write('\n'.join(label_lines) + '\n')

            all_samples.append(img_name)
            img_idx += 1

        print(f"完成 ({img_idx} 张)")

    # 划分 train/val
    random.seed(42)
    random.shuffle(all_samples)
    n_val = int(len(all_samples) * VAL_SPLIT)
    val_samples = set(all_samples[:n_val])

    for img_name in all_samples:
        stem = os.path.splitext(img_name)[0]
        if img_name in val_samples:
            shutil.move(str(train_img / img_name), str(val_img / img_name))
            shutil.move(str(train_lbl / f"{stem}.txt"), str(val_lbl / f"{stem}.txt"))

    # 写 data.yaml
    #
    # ⚠️ **故意不写 `path:`**（D57）。原先这里写的是 `path: {DATASET_DIR}`，
    # 一个绝对路径——本机当下能用，但仓库一换位置（这个仓库就从
    # ~/workspace/yolo_ros 搬到过 ~/robotics/repos/yolo_ros_original）
    # 就变成指向不存在目录的死路径，训练在 val 检查时直接报错。
    # 而写成相对路径同样不行：ultralytics 对相对的 path 是按 `DATASETS_DIR`
    # （默认 ~/datasets，且本机配置里它是**相对**的、随 CWD 变）解析，
    # 不是按 yaml 所在目录，于是会去 ~/datasets/block_dataset 找数据。
    # 省略 `path:` ⇒ ultralytics 取 `yaml_file` 的父目录，即本目录。
    yaml_content = """# YOLOv8 方块检测数据集配置（由 scripts/generate_synthetic_data.py 生成）
# 故意不写 `path:`（D57）：理由见 scripts/generate_synthetic_data.py 里
# `# 写 data.yaml` 那一段的注释 —— 绝对路径会随仓库搬家而死，相对路径会被
# ultralytics 按 DATASETS_DIR 解析，两条路都不通。
train: train/images
val: val/images
names:
  0: block
nc: 1
"""
    with open(out / "data.yaml", 'w') as f:
        f.write(yaml_content)

    n_train = len(list(train_img.iterdir()))
    n_val = len(list(val_img.iterdir()))
    print(f"\n[完成] train={n_train} 张, val={n_val} 张, 总计 {img_idx} 张")
    print(f"       输出: {out}")


if __name__ == "__main__":
    # ⚠️ 必须 `sys.exit(main())`：只写 `main()` 会把返回值丢掉，
    # 于是「拒绝执行」也以 **0** 退出，调用方（脚本/CI）分不出
    # 「干活了」和「我拒绝干活」。实测过这条 —— 退出码曾是 0。
    sys.exit(main())
