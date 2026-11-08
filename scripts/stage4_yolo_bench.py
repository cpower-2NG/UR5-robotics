#!/usr/bin/env python3
"""E15 用的 YOLO 检测计时探针（开发机 / 板子都能跑，只读图片，不碰 ROS）。

为什么单独一个：检测节点在板上跑，而 GraspNet/RNGNet 的计时器在各自的离线脚本里。
E15 要的是**同一口径**的三行数：稳态单帧延迟、峰值显存、峰值 RSS。

口径要点（与 e6_graspnet_crop.py / rngnet_offline.py 对齐）：
- 网络只加载一次，第一轮算**预热**、不计入稳态统计（板上首帧含 CUDA 初始化）；
- 输入固定成 1280×720（与俯视相机一致），避免分辨率不同导致数字不可比。

用法：
  python3 scripts/stage4_yolo_bench.py ~/train_7cls/block_best_7cls.pt IMAGE [--iters 20]
输出：stdout 表格 + 同目录 yolo_bench.json
"""
import argparse
import json
import os
import resource
import statistics
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("weights")
    ap.add_argument("image")
    ap.add_argument("--iters", type=int, default=20)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    from PIL import Image
    from ultralytics import YOLO
    import torch

    img = Image.open(args.image).convert("RGB")
    if img.size != (1280, 720):
        img = img.resize((1280, 720))

    t0 = time.time()
    model = YOLO(args.weights)
    load_s = time.time() - t0
    model.fuse()

    times = []
    warm = None
    for i in range(args.iters + 1):
        t = time.time()
        model.predict(img, imgsz=args.imgsz, verbose=False, device=0)
        dt = (time.time() - t) * 1000
        if i == 0:
            warm = dt                      # 预热帧：CUDA/首次前向
        else:
            times.append(round(dt, 1))

    vram_mb = (torch.cuda.max_memory_allocated() / 2 ** 20
               if torch.cuda.is_available() else 0.0)
    rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0

    rec = {
        "kind": "stage4-yolo-bench",
        "weights": args.weights,
        "image": os.path.basename(args.image),
        "imgsz": args.imgsz,
        "model_load_s": round(load_s, 2),
        "warmup_ms": round(warm, 1),
        "infer_ms": times,
        "median_ms": round(statistics.median(times), 1) if times else None,
        "p90_ms": round(sorted(times)[max(0, int(0.9 * len(times)) - 1)], 1)
        if times else None,
        "vram_peak_mb": round(vram_mb, 1),
        "rss_peak_mb": round(rss_mb, 1),
        "torch": torch.__version__,
        "cuda_avail": torch.cuda.is_available(),
    }
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.image)),
                                   "yolo_bench.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rec, f, ensure_ascii=False, indent=1)

    print("加载 %.2f s；预热 %.0f ms" % (load_s, warm))
    print("稳态：n=%d 中位 %s ms  p90 %s ms" %
          (len(times), rec["median_ms"], rec["p90_ms"]))
    print("峰值显存 %.0f MB  RSS %.0f MB  torch %s  cuda %s" %
          (vram_mb, rss_mb, torch.__version__, torch.cuda.is_available()))
    print("→ %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
