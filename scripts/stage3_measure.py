#!/usr/bin/env python3
"""阶段 3 塔顶测量：只量**当前最上层**，并把 z 读数偏差按标定常数扣掉。

为什么只量最上层
────────────────
竖直俯视下，第 k 层放好后第 k+1 层就把它压住了 —— 下层顶面不可见。
逐层指标的唯一来源因此是「该层放好瞬间」的一次测量（``metric_source =
"immediate"``）；终态复测只回答最上层有没有动、塔有没有塌，**不能**反推下层。

测量怎么做
──────────
· ``graspnet_candidates.find_blocks(max_h_mm=220)``：阶段 2 的上限 70 mm 只容
  单块；5 层塔顶约 150 mm，整根柱子要进同一个簇；
· 塔窗：第 1 层 = 中心格心 ±``radius_mm``（默认 50 mm），第 k≥2 层 = 上一层
  实测中心 ±``radius_mm``；窗内取**最高**的合格簇（``top`` 是 base 系绝对 z，
  97 分位，与 ``find_blocks`` 同一口径）；
· yaw 用 ``yaw_rect``（最小外接矩形），``yaw_trusted`` 为假 ⇒ 该层转人评；
· z 标定：``surface = top − block_h − z_bias``。``z_bias`` 的定义是
  「同一位置单块实测顶高 − 真高」，塔位侧由入口实验 1 标定（扣掉垫厚）。

融合
────
同一视点连拍压随机噪声：中心/顶高取中位，yaw 走 ``mean_yaw_mod90``。
两个视点各出一份融合值，再比一致性（默认 ≤10 mm / ≤6° / ≤10 mm）——
不过就转人评，不进机判。
"""
import argparse
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                   # noqa: E402
import stage2_round as s2                                          # noqa: E402

#: 同一批方块的**真高**（用户多次实测 29.5 mm，2026-10 定）。
BLOCK_TRUE_H_MM = 29.5
#: 塔窗半径：中心格心 / 上一层实测中心 ± 这个值。
TOWER_RADIUS_MM = 50.0
#: 找块高度上限：5 层 ≈ 150 mm，留到 220 mm 把整根柱子容进来。
MAX_H_MM = 220.0
#: 低于这个高度的东西不算块（沿用阶段 2）。
MIN_H_MM = 15.0

#: 两视点一致性门（沿用阶段 2 的 6° / 10 mm；z 同按 10 mm 算位置）。
POSE_XY_TOL_MM = 10.0
POSE_YAW_TOL_DEG = 6.0
POSE_Z_TOL_MM = 10.0

#: 中心格 —— 塔基（用户拍板）。
TOWER_CELL_IDX = 4


def grid_center(grid, cell_idx=TOWER_CELL_IDX):
    """``grid.json`` → 某格的 base 系中心 (mm)。"""
    for c in grid["cells"]:
        if int(c["idx"]) == int(cell_idx):
            return [float(c["center_base_mm"][0]), float(c["center_base_mm"][1])]
    raise ValueError("grid.json 里没有格 %d" % cell_idx)


def pad_z_mm(grid):
    """垫面高度（base 系 z，mm）：``T_base_pad[2,3]``。"""
    T = np.asarray(grid["T_base_pad"], dtype=np.float64)
    return round(float(T[2, 3]) * 1000.0, 3)


def pick_tower_top(blocks, window_xy_mm, radius_mm=TOWER_RADIUS_MM,
                   min_h_mm=MIN_H_MM):
    """塔窗内取**最高**的合格簇；返回 ``(block|None, diag)``。

    只认 ``low=False`` 且高度 ≥``min_h_mm`` 的簇 —— 低处的连接件/台面噪声
    不参与。窗内有多簇时取 ``top`` 最大者：塔顶就是最高的那一层。
    """
    wx, wy = float(window_xy_mm[0]), float(window_xy_mm[1])
    cands = []
    for b in blocks:
        if b.get("low") or b.get("h", 0.0) < min_h_mm:
            continue
        d = math.hypot(b["cx"] - wx, b["cy"] - wy)
        if d <= radius_mm:
            cands.append(dict(b, _dist_mm=round(d, 2)))
    diag = {"n_window": len(cands),
            "window_xy_mm": [round(wx, 2), round(wy, 2)],
            "radius_mm": float(radius_mm),
            "candidates": [{"idx": b["idx"], "cx": round(b["cx"], 2),
                            "cy": round(b["cy"], 2), "top": round(b["top"], 2),
                            "dist_mm": b["_dist_mm"]} for b in cands]}
    if not cands:
        return None, diag
    best = max(cands, key=lambda b: b["top"])
    return best, diag


def measure_frame(frame_dir, grid=None, window_xy_mm=None,
                  z_bias_mm=0.0, block_h_mm=BLOCK_TRUE_H_MM,
                  radius_mm=TOWER_RADIUS_MM, max_h_mm=MAX_H_MM,
                  min_h_mm=MIN_H_MM, frame=None):
    """一帧 → 塔顶观测。

    ``grid`` / ``window_xy_mm``：至少给一个；都没给就报错（塔窗不能靠猜）。
    返回的 ``surface_est_mm`` 已扣 ``z_bias_mm``，是**本层放置面**的估计。
    """
    if window_xy_mm is None:
        if grid is None:
            raise ValueError("measure_frame 需要 grid 或 window_xy_mm")
        window_xy_mm = grid_center(grid)
    if frame is None:
        frame = gc.load_frame(frame_dir)
    blocks, table_z = gc.find_blocks(frame, max_h_mm=max_h_mm,
                                     min_h_mm=min_h_mm)
    top, diag = pick_tower_top(blocks, window_xy_mm, radius_mm=radius_mm,
                               min_h_mm=min_h_mm)
    out = {"frame": os.path.basename(os.path.abspath(frame_dir)),
           "found": top is not None,
           "window_xy_mm": [round(float(window_xy_mm[0]), 2),
                            round(float(window_xy_mm[1]), 2)],
           "table_z_mm": round(float(table_z) * 1000.0, 3),
           "z_bias_mm": round(float(z_bias_mm), 3),
           "block_h_mm": round(float(block_h_mm), 3),
           "diag": diag}
    if top is None:
        return out
    yaw = top.get("yaw_rect")
    out.update({
        "center_base_mm": [round(float(top["cx"]), 2),
                           round(float(top["cy"]), 2)],
        # 绝对顶高（base 系）—— 层间增量与稳定性都用它，标定偏差在差分里抵消。
        "top_mm": round(float(top["top"]), 2),
        # 相对**帧内检出台面**的高度，单独留档（垫子会把它抬高）。
        "h_mm": round(float(top["h"]), 2),
        "yaw_deg": None if yaw is None else round(float(yaw), 3),
        "yaw_trusted": bool(top.get("yaw_trusted")),
        "ext_rect_mm": top.get("ext_rect"),
        "rect_fill": top.get("rect_fill"),
        "class": top.get("class"),
        "class_share": top.get("class_share"),
        "surface_est_mm": round(float(top["top"]) - float(block_h_mm)
                                - float(z_bias_mm), 2),
    })
    return out


def fuse_frames(records):
    """同视点连拍（或两视点已融合值）→ 一份融合值 + 离散度。

    中心 / 顶高取中位；yaw 走 mod-90 圆均值（``yaw_deg=None`` 的帧不投票）。
    """
    found = [r for r in records if r.get("found")]
    n = len(records)
    if not found:
        return {"found": False, "n_frames": n, "n_found": 0}
    cx = float(np.median([r["center_base_mm"][0] for r in found]))
    cy = float(np.median([r["center_base_mm"][1] for r in found]))
    top = float(np.median([r["top_mm"] for r in found]))
    yaws = [r["yaw_deg"] for r in found if r.get("yaw_deg") is not None]
    yaw = s2.mean_yaw_mod90(yaws) if yaws else None
    yaw_span = s2.yaw_span_mod90(yaws) if len(yaws) > 1 else None
    tops = [r["top_mm"] for r in found]
    dists = [math.hypot(r["center_base_mm"][0] - cx,
                        r["center_base_mm"][1] - cy) for r in found]
    return {"found": True, "n_frames": n, "n_found": len(found),
            "center_base_mm": [round(cx, 2), round(cy, 2)],
            "top_mm": round(top, 2),
            "top_span_mm": round(max(tops) - min(tops), 2),
            "center_span_mm": round(2.0 * max(dists), 2),
            "yaw_deg": yaw,
            "yaw_span_deg": yaw_span,
            # 任一帧不可信就不进机判：宁可人评，别把烂 yaw 当数据。
            "yaw_trusted": bool(found) and all(r.get("yaw_trusted")
                                               for r in found),
            "frames_used": [r.get("frame") for r in found]}


def pose_consistency(fused_poses, xy_tol_mm=POSE_XY_TOL_MM,
                     yaw_tol_deg=POSE_YAW_TOL_DEG, z_tol_mm=POSE_Z_TOL_MM):
    """两（或多）视点融合值 → 两两最大差 + 是否过门。

    只有一个视点能测到时 ``ok=None``（**证明不了**一致性，转人评）。
    """
    found = [p for p in fused_poses if p.get("found")]
    if not found:
        return {"ok": False, "reason": "no_pose_found",
                "n_poses": len(fused_poses)}
    if len(found) < 2:
        return {"ok": None, "reason": "single_pose", "n_poses": len(found)}
    dxy = dyaw = dz = 0.0
    for i in range(len(found)):
        for j in range(i + 1, len(found)):
            a, b = found[i], found[j]
            dxy = max(dxy, math.hypot(a["center_base_mm"][0]
                                      - b["center_base_mm"][0],
                                      a["center_base_mm"][1]
                                      - b["center_base_mm"][1]))
            dz = max(dz, abs(a["top_mm"] - b["top_mm"]))
            if a.get("yaw_deg") is not None and b.get("yaw_deg") is not None:
                dyaw = max(dyaw, abs(s2.signed_err_mod90(a["yaw_deg"],
                                                         b["yaw_deg"])))
    ok = dxy <= xy_tol_mm and dz <= z_tol_mm and dyaw <= yaw_tol_deg
    return {"ok": bool(ok), "n_poses": len(found),
            "dxy_mm": round(dxy, 2), "dyaw_deg": round(dyaw, 2),
            "dz_mm": round(dz, 2),
            "tol": {"xy_mm": xy_tol_mm, "yaw_deg": yaw_tol_deg,
                    "z_mm": z_tol_mm},
            "reason": "ok" if ok else "pose_mismatch"}


def measure_layer(pose_frames, grid=None, window_xy_mm=None, layer=None,
                  z_bias_mm=0.0, block_h_mm=BLOCK_TRUE_H_MM,
                  radius_mm=TOWER_RADIUS_MM, max_h_mm=MAX_H_MM,
                  min_h_mm=MIN_H_MM, metric_source="immediate"):
    """一层的完整测量：``[(视点名, [帧目录, …]), …]`` → 观测记录。

    逐帧测 → 每视点融合 → 两视点一致性 → 再融合成该层唯一取值。
    ``merged`` 里的 ``center_base_mm`` / ``top_mm`` / ``yaw_deg`` 就是该层指标
    （``metric_source="immediate"``）。

    两个容易混的量，这里分开写：
    · ``surface_est_mm`` = 校正顶高 − 块真高 = **本层块脚下的面**（对账用）；
    · ``next_surface_est_mm`` = 校正顶高 = **下一层要放的表面**。
    拿前者当下一层表面会整整差一块高（29.5 mm）。
    """
    poses = []
    for name, dirs in pose_frames:
        frames = [measure_frame(d, grid=grid, window_xy_mm=window_xy_mm,
                                z_bias_mm=z_bias_mm, block_h_mm=block_h_mm,
                                radius_mm=radius_mm, max_h_mm=max_h_mm,
                                min_h_mm=min_h_mm) for d in dirs]
        poses.append({"name": name, "frames": frames,
                      "fused": fuse_frames(frames)})
    fused_all = [p["fused"] for p in poses]
    cons = pose_consistency(fused_all)
    # 融合时带上视点名，``merged["frames_used"]`` 才不会是一串 None。
    merged = fuse_frames([dict(p["fused"], frame=p["name"]) for p in poses])
    out = {"version": 1, "layer": layer, "metric_source": metric_source,
           "z_bias_mm": round(float(z_bias_mm), 3),
           "block_h_mm": round(float(block_h_mm), 3),
           "window_xy_mm": ([round(float(window_xy_mm[0]), 2),
                             round(float(window_xy_mm[1]), 2)]
                            if window_xy_mm is not None else
                            (grid_center(grid) if grid else None)),
           "poses": poses, "consistency": cons, "merged": merged}
    if merged.get("found"):
        out["surface_est_mm"] = round(merged["top_mm"] - float(block_h_mm)
                                      - float(z_bias_mm), 2)
        out["next_surface_est_mm"] = round(merged["top_mm"]
                                           - float(z_bias_mm), 2)
    return out


def bias_from_single(top_mm, pad_z, block_h_mm=BLOCK_TRUE_H_MM):
    """入口实验 1：塔位单块 → ``z_bias = top − 垫面 − 真高``（mm）。"""
    return round(float(top_mm) - float(pad_z) - float(block_h_mm), 3)


def scan_biases(frame_dirs, block_h_mm=BLOCK_TRUE_H_MM,
                min_h_mm=MIN_H_MM, max_h_mm=70.0):
    """一批帧 → 逐块 ``bias = h − 真高`` 表 + 汇总（入口实验 1 / 历史复算）。

    ``h`` 是相对**帧内检出台面**的高度（`find_blocks` 的 ``h``）。用 ``h`` 而不是
    绝对 ``top``，是因为历史帧的台面读数本身带偏移；``bias`` 的定义就是
    「同一块在深度图里比 29.5 mm 高/低多少」，与台面锚点无关。
    """
    rows = []
    for fd in frame_dirs:
        blocks, table_z = gc.find_blocks(gc.load_frame(fd), max_h_mm=max_h_mm,
                                         min_h_mm=min_h_mm)
        for b in blocks:
            if b.get("low"):
                continue
            rows.append({
                "frame": os.path.basename(os.path.abspath(fd)),
                "dir": os.path.abspath(fd),
                "block_idx": int(b["idx"]),
                "cx_mm": round(float(b["cx"]), 2),
                "cy_mm": round(float(b["cy"]), 2),
                "h_mm": round(float(b["h"]), 2),
                "bias_mm": round(float(b["h"]) - float(block_h_mm), 2),
                "class": b.get("class"),
            })
    vals = [r["bias_mm"] for r in rows]
    summary = {"n": len(vals), "n_blocks": len(rows), "n_frames": len(frame_dirs)}
    if vals:
        arr = np.asarray(vals, dtype=float)
        summary.update({
            "median_mm": round(float(np.median(arr)), 2),
            "mean_mm": round(float(arr.mean()), 2),
            "min_mm": round(float(arr.min()), 2),
            "max_mm": round(float(arr.max()), 2),
            "n_positive": int((arr > 0).sum()),
        })
    return {"version": 1, "block_h_mm": float(block_h_mm), "summary": summary,
            "rows": rows}


# ---------------------------------------------------------------- CLI
def _parse_poses(items):
    out = []
    for spec in items:
        if "=" not in spec:
            raise SystemExit("--pose 要 name=dir1,dir2 形式，收到 %r" % spec)
        name, dirs = spec.split("=", 1)
        out.append((name, [d for d in dirs.split(",") if d]))
    return out


def main():
    ap = argparse.ArgumentParser(description="阶段 3 塔顶测量（只量最上层）")
    ap.add_argument("--pose", action="append", default=None,
                    help="视点：name=frame_dir[,frame_dir…]；可重复")
    ap.add_argument("--scan", action="append", default=None,
                    help="只出偏差表：帧目录（可含通配符）；可重复")
    ap.add_argument("--grid", default=None, help="grid.json（给塔窗与垫面）")
    ap.add_argument("--window", default=None,
                    help="塔窗中心 x,y（mm）；缺省取 grid 中心格")
    ap.add_argument("--layer", type=int, default=None)
    ap.add_argument("--bias-mm", type=float, default=0.0,
                    help="塔位 z 读数偏差（实测顶高 − 真高），从入口实验来")
    ap.add_argument("--block-h-mm", type=float, default=BLOCK_TRUE_H_MM)
    ap.add_argument("--radius-mm", type=float, default=TOWER_RADIUS_MM)
    ap.add_argument("--max-h-mm", type=float, default=MAX_H_MM)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.scan:
        dirs = []
        for spec in args.scan:
            hits = sorted(glob.glob(os.path.expanduser(spec)))
            dirs.extend(hits or [os.path.expanduser(spec)])
        res = scan_biases(dirs, block_h_mm=args.block_h_mm,
                          min_h_mm=MIN_H_MM, max_h_mm=args.max_h_mm)
        text = json.dumps(res, ensure_ascii=False, indent=1)
        if args.out:
            open(args.out, "w").write(text + "\n")
        print(text)
        s = res["summary"]
        print("偏差（h − %.1f mm）：n=%d 中位 %.2f 均值 %.2f 极差 %.2f..%.2f"
              % (args.block_h_mm, s["n"], s.get("median_mm", 0.0),
                 s.get("mean_mm", 0.0), s.get("min_mm", 0.0),
                 s.get("max_mm", 0.0)), file=sys.stderr)
        return 0
    if not args.pose:
        raise SystemExit("要么给 --pose，要么给 --scan")

    grid = json.load(open(args.grid)) if args.grid else None
    window = None
    if args.window:
        window = [float(v) for v in args.window.split(",")]
        if len(window) != 2:
            raise SystemExit("--window 要给 x,y 两个数")
    elif grid is not None:
        window = grid_center(grid)
    else:
        raise SystemExit("要给 --grid 或 --window（塔窗不能靠猜）")

    res = measure_layer(_parse_poses(args.pose), grid=grid,
                        window_xy_mm=window, layer=args.layer,
                        z_bias_mm=args.bias_mm,
                        block_h_mm=args.block_h_mm,
                        radius_mm=args.radius_mm, max_h_mm=args.max_h_mm)
    if grid is not None:
        res["pad_z_mm"] = pad_z_mm(grid)
        if res["merged"].get("found"):
            res["bias_est_mm"] = bias_from_single(
                res["merged"]["top_mm"], res["pad_z_mm"], args.block_h_mm)

    text = json.dumps(res, ensure_ascii=False, indent=1)
    if args.out:
        open(args.out, "w").write(text + "\n")
    print(text)
    m = res["merged"]
    if not m.get("found"):
        print("!! 塔窗内没有合格块 —— 本层 measured_missing", file=sys.stderr)
        return 2
    if res["consistency"].get("ok") is not True:
        print("⚠ 一致性未过（%s）—— 转人评" % res["consistency"].get("reason"),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
