#!/usr/bin/env python3
"""把一次真机俯视 burst 量成 E8 探针能吃的 ``scene_truth.json``。

为什么需要这个工具
------------------
``/targets_3d`` 只有位置 —— ``bbox3d.size`` 全 0、朝向单位阵，喂不了 E8 探针
（探针要 ``id/x/y/z/size/yaw/top``，base 系、米）。块的分割/颜色/朝向来自 E5
感知链（``graspnet_candidates.find_blocks``，用户 2026-11-08 拍板的真值来源），
但**位置不能直接用它的深度反投影**：

    2026-11-08 用垫 PnP（纯彩色、不经过深度）当场量到俯视深度**系统性读远
    3.0–3.3%**（1.5 m 处 ≈ 46 mm，三条 burst 一致）。直接反投影会把台面算到
    −30 mm、块位整体偏出 20–30 mm。

所以位置走**射线 ∩ 平面**（与生产管线 ``projector_node`` 的
``use_target_plane`` 同一套几何，那里写明的精度是「标定水平」而不是深度水平）：

    uv（find_blocks 的顶面中心像素，深度尺度不改变它）
      → 射线 ∩ 平面 z = table_top + 块高
      → x, y

    top = table_top + 块高        size = --block-size
    z   = top − size / 2
    yaw = find_blocks 的顶面 PCA 长边（均匀尺度不改变主轴方向 ⇒ 对深度偏差免疫）

``table_top`` 默认 0.0 —— 生产栈在真机上用的就是 ``target_plane_z = 台面 + 块高``
（runbook 里 28.5 mm），即台面视为 base z 0。示教器碰面读数 +3.88 mm、垫 PnP
的垫顶 ≈ +1.3 mm 都落在这个值的 ±5 mm 内；要更准就传 ``--table-top``。

拾取区按**射线 ∩ 平面后的 x,y** 判（``ZONE_M``，E5 同一常量）—— 不用深度口径，
否则深度偏差会把边界附近的块判错。区外的检测当幻影丢掉并记 ``dropped_out_of_zone``
（它们若进规划场景会变成假障碍，把 E8 的 M1 冤枉成碰撞）。``class == "unknown"``
（find_blocks 的「整簇没有饱和像素」）同理丢掉并记 ``dropped_not_block`` ——
真机那批积木是 7 个彩色类，灰白表面不是块。

深度仍然要，但只用来：(a) 找块/切顶面/判 yaw，(b) 把实测高度 ``h`` 记进
``meas`` 供人核对（读远 ⇒ h 也偏大，别拿它当尺子）。

用法：

    python3 scripts/stage4_truth_from_frame.py \\
        --frame runs/<run>/e8/topcam-e8 --out runs/<run>/e8/scene_truth_real.json

    # 想先看这一 burst 稳不稳（每帧各量一遍，只打印，不写真值）
    python3 scripts/stage4_truth_from_frame.py --frame <dir> --survey

退出码：0 = 真值写出；2 = 输入不足/量不出块（缺 tf、缺 camera_info、0 块）。
"""
import argparse
import glob
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage4_specify as sp                                       # noqa: E402

BLOCK_SIZE_M = 0.0295


def ray_plane_xy(uv, K, R_bc, t_bc, plane_z):
    """像素 → 射线 ∩ 平面 ``z = plane_z``（base 系）。返回 ``(x, y)`` 或 None。

    射线方向只用像素与内参（与 ``projector_node._ray_plane_pose`` 同义：
    借深度反投影点表达方向 ``(x/z, y/z, 1)``，但位置不经过深度）。
    平面在相机背后 / 射线与平面平行时返回 None。
    """
    u, v = float(uv[0]), float(uv[1])
    d_cam = np.array([(u - K[0, 2]) / K[0, 0],
                      (v - K[1, 2]) / K[1, 1], 1.0])
    d = np.asarray(R_bc, dtype=float) @ d_cam
    if abs(d[2]) < 1e-9:
        return None
    lam = (float(plane_z) - float(t_bc[2])) / d[2]
    if lam <= 0.0:
        return None
    p = np.asarray(t_bc, dtype=float) + lam * d
    return float(p[0]), float(p[1])


def in_zone_xy(x, y, zone_m=None):
    """base 系 (x, y) 是否在拾取区（米）。``zone_m`` 缺省用 ``sp.ZONE_M``。"""
    x0, x1, y0, y1 = (sp.ZONE_M if zone_m is None else zone_m)
    return x0 <= x <= x1 and y0 <= y <= y1


def build_truth(rows, cam, table_top=0.0, block_size=BLOCK_SIZE_M,
                batch="REAL", index=0, keep_outside=False,
                drop_unknown=True, min_class_share=0.0):
    """``find_blocks`` 的行 → ``scene_truth.json`` 的形状（射线 ∩ 平面）。

    ``cam`` 给 ``{'K','R_bc','t_bc'}``（``graspnet_candidates.load_frame`` 的
    字段）。``low=True`` 的行（5 mm…min_h 的诊断簇）不是块，丢掉。
    实测 ``h`` 与块高差超过 6 mm 时**不失败**，写 ``warn_h_mm`` —— 叠放或顶面
    测量出问题由人看报告判；位置仍按「一块高」处理。
    """
    blocks = []
    n_out = 0
    n_unknown = 0
    for r in rows:
        if r.get("low"):
            continue
        # ``class == "unknown"`` 是 find_blocks 的「整簇没有饱和像素」——
        # 灰/白/木色表面，定义上不是那 7 色积木。留着会变成假障碍。
        if drop_unknown and r.get("class") == "unknown":
            n_unknown += 1
            continue
        if float(r.get("class_share") or 0.0) < float(min_class_share):
            n_unknown += 1
            continue
        size = float(block_size)
        plane_z = float(table_top) + size
        xy = ray_plane_xy(r["uv"], cam["K"], cam["R_bc"], cam["t_bc"],
                          plane_z)
        if xy is None:
            continue                      # 射线打不到平面上：不是块
        if not keep_outside and not in_zone_xy(xy[0], xy[1]):
            n_out += 1
            continue
        top = plane_z
        h_mm = float(r.get("h", 0.0))
        entry = {
            "id": int(r.get("idx", len(blocks))),
            "color": r.get("class", "unknown"),
            "layer": 0,
            "size": size,
            "top": top,
            "x": xy[0],
            "y": xy[1],
            "z": top - size / 2.0,
            "yaw": float(r.get("yaw", 0.0)),
            # 只为追溯，不参与碰撞几何
            "meas": {
                "h_mm": round(h_mm, 2),
                "uv": r.get("uv"),
                "depth_xy_mm": [round(float(r["cx"]), 1),
                                round(float(r["cy"]), 1)],
                "depth_top_mm": round(float(r.get("top", 0.0)), 2),
                "ext_mm": [round(float(v), 2) for v in r.get("ext", [])],
                "ext_rect_mm": [round(float(v), 2)
                                for v in r.get("ext_rect", [])],
                "class_share": r.get("class_share"),
            },
        }
        if abs(h_mm - size * 1000.0) > 6.0:
            entry["warn_h_mm"] = round(h_mm - size * 1000.0, 2)
        blocks.append(entry)
    blocks.sort(key=lambda b: (b["x"], b["y"]))
    for i, b in enumerate(blocks):
        b["id"] = i
    return {
        "batch": batch,
        "index": int(index),
        "source": "stage4_truth_from_frame:ray-plane",
        "block_size_m": float(block_size),
        "table_top_m": float(table_top),
        "blocks": blocks,
        "dropped_out_of_zone": n_out,
        "dropped_not_block": n_unknown,
        "zone_m": list(sp.ZONE_M),
    }


def measure_dir(frame_dir, index=None, split=True):
    """俯视帧 → (frame, find_blocks 行, 台面 z)。俯视链纯静态，不经过臂。"""
    frame = gc.load_frame(frame_dir, tip=gc.TOPCAM_TIP, chain=gc.TOPCAM_CHAIN,
                          idx=index)
    rows, table_z = gc.find_blocks(frame, split=split)
    return frame, rows, table_z


def _median(vals):
    v = sorted(vals)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def _yaw_square(vals):
    """方块朝向的圆中位；周期 90°（``4×`` 折角就是这个意思，不需要先 ``% 90``）。

    积木是正方形，yaw 与 yaw+90° 表示同一个盒子；不折周期的话 PCA 会在两组
    相差 ~90° 的值之间来回跳，直接取中位会落在两者中间（最坏）。
    """
    sx = sum(np.cos(np.radians(4.0 * v)) for v in vals)
    sy = sum(np.sin(np.radians(4.0 * v)) for v in vals)
    ang = math.degrees(math.atan2(sy, sx)) / 4.0
    return ang % 90.0


def consensus(truths, radius_m=0.020, min_frames=None):
    """多帧真值按位置聚类 → 一份稳定真值（抗单帧漏块/裂块/幻影）。

    单帧 ``find_blocks`` 在这台真机上是**不稳的**（2026-11-08 实测 10 帧：
    同一块位置摆动 2–8 mm；有的帧漏一块；一帧把青块切成 4 份；偶发幻影）。
    E8 的碰撞场景要的是静态真值，所以把同一 burst 的每一帧各量一遍再合并：

    - 每帧的块按 (x, y) 贪心归簇（阈值 ``radius_m``，对簇心判距）；
    - 出现的帧数 < ``min_frames`` 的簇丢掉（幻影/裂片只在一两帧出现）；
    - 位置取中位；颜色多数票；yaw 取方块的圆中位（见 ``_yaw_square``）；
    - 逐块记录出现帧数与位置散布，让报告里看得见「这块稳不稳」。
    """
    if not truths:
        raise ValueError("consensus 至少要一帧")
    n = len(truths)
    if min_frames is None:
        min_frames = max(3, (n + 1) // 2)
    clusters = []                      # [{pts: [...], frames: set}]
    for fi, t in enumerate(truths):
        for b in t["blocks"]:
            best, best_d = None, None
            for c in clusters:
                cx = _median([p["x"] for p in c["pts"]])
                cy = _median([p["y"] for p in c["pts"]])
                d = math.hypot(b["x"] - cx, b["y"] - cy)
                if d <= radius_m and (best_d is None or d < best_d):
                    best, best_d = c, d
            if best is None:
                clusters.append({"pts": [b], "frames": {fi}})
            else:
                best["pts"].append(b)
                best["frames"].add(fi)
    blocks = []
    for c in clusters:
        if len(c["frames"]) < min_frames:
            continue                       # 只在一两帧出现 ⇒ 幻影/裂片
        pts = c["pts"]
        xs = [p["x"] for p in pts]
        ys = [p["y"] for p in pts]
        mx, my = _median(xs), _median(ys)
        colors = {}
        for p in pts:
            colors[p["color"]] = colors.get(p["color"], 0) + 1
        color = max(colors, key=colors.get)
        size = float(pts[0]["size"])
        table_top = float(pts[0]["top"]) - size
        blocks.append({
            "id": 0,
            "color": color,
            "layer": 0,
            "size": size,
            "top": table_top + size,
            "x": mx,
            "y": my,
            "z": table_top + size / 2.0,
            "yaw": _yaw_square([float(p["yaw"]) for p in pts]),
            "meas": {"n_frames": len(c["frames"]),
                     "n_pts": len(pts),
                     "x_spread_mm": round((max(xs) - min(xs)) * 1000, 2),
                     "y_spread_mm": round((max(ys) - min(ys)) * 1000, 2),
                     "colors": colors,
                     "h_mm_median": round(_median(
                         [p["meas"]["h_mm"] for p in pts]), 2)},
        })
    blocks.sort(key=lambda b: (b["x"], b["y"]))
    for i, b in enumerate(blocks):
        b["id"] = i
    return {
        "batch": truths[0]["batch"],
        "index": truths[0]["index"],
        "source": "stage4_truth_from_frame:consensus",
        "block_size_m": float(truths[0]["block_size_m"]),
        "table_top_m": float(truths[0]["table_top_m"]),
        "n_frames": n,
        "min_frames": min_frames,
        "cluster_radius_mm": radius_m * 1000.0,
        "blocks": blocks,
        "zone_m": list(truths[0]["zone_m"]),
    }


def survey(frame_dir, split=True, table_top=0.0):
    idxs = gc.list_frame_indices(frame_dir)
    print("帧数 %d" % len(idxs))
    for idx in idxs:
        frame, rows, table_z = measure_dir(frame_dir, index=idx, split=split)
        t = build_truth(rows, frame, table_top=table_top)
        print("  idx=%-3s 深度台面=%8.2f mm  块数=%2d  区外=%2d  非积木=%2d"
              % (idx, table_z * 1000.0, len(t["blocks"]),
                 t["dropped_out_of_zone"], t["dropped_not_block"]))
        for b in t["blocks"]:
            print("      #%d %-11s x=%7.1f y=%7.1f yaw=%6.1f  "
                  "（深度口径 x=%7.1f y=%7.1f）"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["yaw"], b["meas"]["depth_xy_mm"][0],
                     b["meas"]["depth_xy_mm"][1]))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="真机俯视 burst → E8 探针的 scene_truth.json（射线 ∩ 平面）")
    ap.add_argument("--frame", required=True, help="俯视帧目录（burst）")
    ap.add_argument("--index", type=int, default=None,
                    help="用第几帧（默认：burst 里取第 0 帧）")
    ap.add_argument("--block-size", type=float, default=BLOCK_SIZE_M,
                    help="积木边长（米），默认 0.0295")
    ap.add_argument("--table-top", type=float, default=0.0,
                    help="台面顶面在 base 系的 z（米），默认 0.0（生产栈口径）")
    ap.add_argument("--out", default=None, help="写到哪；不给就打印到 stdout")
    ap.add_argument("--no-split", action="store_true",
                    help="关闭 find_blocks 的顶面切分")
    ap.add_argument("--keep-outside", action="store_true",
                    help="拾取区外的检测也写进真值（默认丢掉，防止幻影变假障碍）")
    ap.add_argument("--keep-unknown", action="store_true",
                    help="保留 class=unknown 的簇（整簇无饱和像素，默认丢）")
    ap.add_argument("--min-class-share", type=float, default=0.0,
                    help="颜色票占比下限（默认 0 = 不过滤）")
    ap.add_argument("--survey", action="store_true",
                    help="逐帧各量一遍，只打印稳定性，不写真值")
    ap.add_argument("--consensus", action="store_true",
                    help="每帧各量一遍再按位置聚类合并（抗单帧漏块/裂块/幻影）")
    ap.add_argument("--min-frames", type=int, default=None,
                    help="共识要求至少出现的帧数（默认 max(3, ⌈n/2⌉)）")
    ap.add_argument("--cluster-mm", type=float, default=20.0,
                    help="共识聚类的距离阈值（毫米），默认 20")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.frame):
        print("!! 帧目录不存在：%s" % args.frame, file=sys.stderr)
        return 2
    if not glob.glob(os.path.join(args.frame, "color*.png")):
        print("!! 帧目录里没有 color*.png：%s" % args.frame, file=sys.stderr)
        return 2

    if args.survey:
        survey(args.frame, split=not args.no_split,
               table_top=args.table_top)
        return 0

    idx = args.index
    if idx is None:
        idxs = gc.list_frame_indices(args.frame)
        idx = idxs[0] if idxs else None

    if args.consensus:
        idxs = gc.list_frame_indices(args.frame)
        truths = []
        # 逐帧**先不按区筛**：贴边界的块会因单帧噪声在区内/区外来回跳，
        # 先筛会把同一块在不同帧切掉 ⇒ 出现帧数虚低。区的判定留到合并之后，
        # 用中位位置判一次。
        for i in idxs:
            fr, rows_i, _ = measure_dir(args.frame, index=i,
                                        split=not args.no_split)
            truths.append(build_truth(
                rows_i, fr, table_top=args.table_top,
                block_size=args.block_size,
                batch=os.path.basename(args.frame.rstrip("/")), index=i,
                keep_outside=True,
                drop_unknown=not args.keep_unknown,
                min_class_share=args.min_class_share))
        truth = consensus(truths, radius_m=args.cluster_mm / 1000.0,
                          min_frames=args.min_frames)
        if not args.keep_outside:
            keep = [b for b in truth["blocks"]
                    if in_zone_xy(b["x"], b["y"])]
            truth["dropped_out_of_zone"] = len(truth["blocks"]) - len(keep)
            truth["blocks"] = keep
            for j, b in enumerate(truth["blocks"]):
                b["id"] = j
        else:
            truth["dropped_out_of_zone"] = 0
        n = len(truth["blocks"])
        print("共识 %d 帧（≥%d 帧才算一块）：块 %d 个，区外丢 %d"
              % (truth["n_frames"], truth["min_frames"], n,
                 truth["dropped_out_of_zone"]))
        for b in truth["blocks"]:
            m = b["meas"]
            print("  #%d %-9s x=%7.1f y=%7.1f top=%6.1f yaw=%5.1f  "
                  "帧 %2d/%d  散布 x=%.1f y=%.1f mm  颜色 %s"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["top"] * 1000, b["yaw"], m["n_frames"],
                     truth["n_frames"], m["x_spread_mm"], m["y_spread_mm"],
                     m["colors"]))
    else:
        frame, rows, table_z = measure_dir(args.frame, index=idx,
                                           split=not args.no_split)
        truth = build_truth(rows, frame, table_top=args.table_top,
                            block_size=args.block_size,
                            batch=os.path.basename(args.frame.rstrip("/")),
                            index=0 if idx is None else idx,
                            keep_outside=args.keep_outside,
                            drop_unknown=not args.keep_unknown,
                            min_class_share=args.min_class_share)
        n = len(truth["blocks"])
        print("帧 %s：台面 z=%.2f mm，块 %d 个（位置=射线∩平面 z=%.1f mm），"
              "区外丢 %d / 非积木丢 %d"
              % (args.frame, table_z * 1000.0, n,
                 (args.table_top + args.block_size) * 1000,
                 truth["dropped_out_of_zone"], truth["dropped_not_block"]))
        for b in truth["blocks"]:
            print("  #%d %-11s x=%7.1f y=%7.1f top=%6.1f yaw=%6.1f  h=%6.1f%s"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["top"] * 1000, b["yaw"], b["meas"]["h_mm"],
                     "" if "warn_h_mm" not in b
                     else "  ⚠ h−size=%+.1f mm" % b["warn_h_mm"]))
    if n == 0:
        print("!! 一块都没量出来 —— 俯视里没有块，或分割/颜色表不对",
              file=sys.stderr)
        return 2

    text = json.dumps(truth, ensure_ascii=False, indent=2, sort_keys=True)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print("写出 %s" % args.out)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
