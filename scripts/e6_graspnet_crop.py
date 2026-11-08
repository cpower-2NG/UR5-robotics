#!/usr/bin/env python3
"""E6/O2 用的 GraspNet runner：**按指定实例局部裁剪**后推理。

为什么不再是整帧
----------------
设计文档 §5.1 的 GraspNet 输入是「融合点云 + 目标实例局部邻域（含邻块，
供碰撞判断）」，不是整帧。2026-11-07 实测：同一张仿真俯视整帧上，模型给出
47 个候选，离任何块最近的也在 16.6 mm 之外（多数在 300 mm 外）；而裁到
0.30 m 见方的局部点云后，同一块上最近的候选 11.6 mm。整帧失败不是模型坏，
是输入不符合它的设计口径。

预处理与 ``graspnet_candidates.infer`` **逐行同一**（voxel 5 mm、20000 点、
score ≥0.20、倾斜 ≤35°、多 seed 合并），只是模型只加载一次：原型每次
``infer`` 都新建网络，用来量延迟会把「加载权重」算进推理时间。

用法（graspnet venv）：
  python3 scripts/e6_graspnet_crop.py FRAME_DIR [FRAME_DIR ...]
输出：每个帧目录下 graspnet.json（块 → 候选 + 每次推理耗时 + 峰值显存）。
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import graspnet_candidates as gc  # noqa: E402

CROP_HALF_M = 0.15
SEEDS = (0, 1, 2)


def load_net(ckpt=gc.CKPT):
    import torch
    sys.path.insert(0, gc.GRASPNET_ROOT)
    sys.path.insert(0, os.path.join(gc.GRASPNET_ROOT, "pointnet2"))
    sys.path.insert(0, os.path.join(gc.GRASPNET_ROOT, "utils"))
    from models.graspnet import GraspNet

    dev = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net = GraspNet(seed_feat_dim=512, is_training=False).to(dev).eval()
    net.load_state_dict(torch.load(ckpt, map_location="cpu",
                                   weights_only=True)["model_state_dict"],
                        strict=True)
    return net, dev


def infer_once(net, dev, points_cam, seed):
    """与 graspnet_candidates.infer 同一预处理，网络复用。"""
    import torch
    import MinkowskiEngine as ME
    from models.graspnet import pred_decode  # noqa: F401  (sys.path 已设)

    rng = np.random.default_rng(seed)
    pts = points_cam[rng.choice(len(points_cam), gc.NUM_POINTS,
                                replace=len(points_cam) < gc.NUM_POINTS)]
    pts = pts.astype(np.float32)
    coors = np.ascontiguousarray(pts / gc.VOXEL, dtype=np.int32)
    feats = np.ones_like(pts, dtype=np.float32)
    coors_b, feats_b = ME.utils.sparse_collate([coors], [feats])
    coors_b = np.ascontiguousarray(coors_b, dtype=np.int32)
    coors_b, feats_b, _, q2o = ME.utils.sparse_quantize(
        coors_b, feats_b, return_index=True, return_inverse=True)

    def as_tensor(x, dtype):
        t = x if torch.is_tensor(x) else torch.from_numpy(np.asarray(x))
        return t.to(dtype=dtype, device=dev)

    ep = {"point_clouds": torch.from_numpy(pts[None]).to(dev),
          "coors": as_tensor(coors_b, torch.int32),
          "feats": as_tensor(feats_b, torch.float32),
          "quantize2original": as_tensor(q2o, torch.int64)}
    with torch.no_grad():
        return pred_decode(net(ep))[0].cpu()


def run_frame(frame_dir, net, dev):
    fr = gc.load_frame(frame_dir)
    P_cam, _ = gc.backproject(fr["depth"], fr["K"])
    P_base = P_cam @ fr["R_bc"].T + fr["t_bc"]
    truth = json.load(open(os.path.join(frame_dir, "truth.json")))
    out = {"frame": frame_dir, "crop_half_m": CROP_HALF_M,
           "seeds": list(SEEDS), "blocks": [], "infer_ms": []}
    for b in truth["blocks"]:
        cx, cy, top = float(b["x"]), float(b["y"]), float(b["top"])
        sel = ((np.abs(P_base[:, 0] - cx) < CROP_HALF_M)
               & (np.abs(P_base[:, 1] - cy) < CROP_HALF_M)
               & (P_base[:, 2] > -0.03) & (P_base[:, 2] < top + 0.10))
        P_in = P_cam[sel]
        rows = []
        times = []
        for s in SEEDS:
            t0 = time.time()
            g = infer_once(net, dev, P_in, s)
            times.append(round((time.time() - t0) * 1000, 1))
            g = g.numpy()
            cands = gc.candidates_base(g, fr["R_bc"], fr["t_bc"], table_z=0.0)
            for c in cands:
                d = float(np.hypot(c["xyz_base"][0] * 1000 - cx * 1000,
                                   c["xyz_base"][1] * 1000 - cy * 1000))
                c = dict(c)
                c["seed"] = int(s)
                c["dist_mm"] = round(d, 2)
                rows.append(c)
        out["infer_ms"].extend(times)
        out["blocks"].append({
            "idx": int(b["id"]), "class": b["color"],
            "n_points": int(len(P_in)),
            "n_raw": len(rows),
            "n_anchor": sum(1 for r in rows if r["dist_mm"] <= gc.ANCHOR_MM),
            "candidates": rows,
            "infer_ms": times,
        })
        print("  块 #%d %-11s 点 %6d 候选 %3d 锚定 %2d  推理 %s ms"
              % (b["id"], b["color"], len(P_in), len(rows),
                 out["blocks"][-1]["n_anchor"], times))
    out["vram_peak_mb"] = round(
        (__import__("torch").cuda.max_memory_allocated() / 2 ** 20)
        if __import__("torch").cuda.is_available() else 0.0, 1)
    out["rss_peak_mb"] = round(
        __import__("resource").getrusage(
            __import__("resource").RUSAGE_SELF).ru_maxrss / 1024.0, 1)
    json.dump(out, open(os.path.join(frame_dir, "graspnet.json"), "w"),
              ensure_ascii=False, indent=1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="+")
    args = ap.parse_args()
    t0 = time.time()
    net, dev = load_net()
    load_s = round(time.time() - t0, 2)
    print("网络加载 %.2f s；裁剪半宽 %.3f m；seeds %s"
          % (load_s, CROP_HALF_M, SEEDS))
    summary = {"model_load_s": load_s, "frames": []}
    for fd in args.frames:
        print("== %s" % fd)
        r = run_frame(fd, net, dev)
        summary["frames"].append({
            "frame": fd, "infer_ms": r["infer_ms"],
            "vram_peak_mb": r["vram_peak_mb"]})
    json.dump(summary, open(os.path.join(
        os.path.dirname(args.frames[0]), "graspnet_summary.json"), "w"),
        ensure_ascii=False, indent=1)
    import statistics
    ms = [t for f in summary["frames"] for t in f["infer_ms"]]
    print("\n推理耗时：n=%d  中位 %.1f ms  p90 %.1f ms  峰值显存 %.0f MB"
          % (len(ms), statistics.median(ms),
             sorted(ms)[int(0.9 * len(ms)) - 1], summary["frames"][0]["vram_peak_mb"]))


if __name__ == "__main__":
    main()
