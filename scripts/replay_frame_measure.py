#!/usr/bin/env python3
"""E1 · 真帧回放：把**真实 stage3 帧**喂进**当前**测量代码，比对真机记录。

判据原文（第一百四十九轮定稿）：
    「真帧回放：把一帧真实 stage3 帧喂进同一测量代码，结果与真机记录一致
      （≤1 mm/≤1°）。」

为什么是这条而不是「再跑一次真机」
==================================
「同一帧 ⇒ 同一结果」是测量链**可复现**的定义。真机每次重跑都会引入新的
物理差异（方块换了、相机抖了），失败时分不清是代码变了还是世界变了。
这里把**像素与位姿都冻在盘上**（`color.png` / `depth.png` / `tf.json`），
唯一会变的就是代码 —— 于是任何超过 1 mm 的差异一定来自代码。

比对哪几个量
============
只比**测量输出**：每块的 ``cx``、``cy``（base_link 毫米）、``top``（毫米）、
``yaw_rect``（度）。不比 ``yaw``（PCA，近正方块本来就不稳）也不比
``class`` / ``class_share``（那是颜色表，不是几何；颜色表的跨相机核对是 E18）。

``yaw_rect`` 按 **90° 周期**比：方块的矩形轴差 90° 是同一个朝向，
直接相减会把 89° 和 1° 算成 88° 的假差异。

用法
----
    # 一轮：帧目录 + 那一轮记录的 candidates.json
    python3 scripts/replay_frame_measure.py --frame-dir <round>/frame-pick \\
        --record <round>/candidates.json

    # 一次扫一整批（自动找 round-*/frame-pick + round-*/candidates.json）
    python3 scripts/replay_frame_measure.py --run-dir ~/robotics/notes/.../2026-11-04-stage3-final

    # 写 JSON 供台账引用
    python3 scripts/replay_frame_measure.py --run-dir ... --json out.json

⚠️ 必须在 graspnet venv 下跑（``analyse`` 会 import torch：
``~/robotics/venvs/graspnet/bin/python scripts/replay_frame_measure.py ...``）。
成功退出码 0，任一帧超差退出码 2。
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

import graspnet_candidates as gc                                 # noqa: E402

# 记录的字段 → 显示名；全部是「米/毫米」或「度」，没有归一化量
FIELDS_MM = (("cx", "cx"), ("cy", "cy"), ("top", "top"))
FIELD_DEG = "yaw_rect"


def _dxy(a, b):
    return math.hypot(float(a["cx"]) - float(b["cx"]),
                      float(a["cy"]) - float(b["cy"]))


def _dyaw(a, b):
    """90° 周期差（矩形轴）。"""
    d = (float(a) - float(b)) % 90.0
    return min(d, 90.0 - d)


def _match(rb, gb, tol_mm=5.0):
    """贪心最近邻配对（≤5 mm）。块间至少 30 mm，5 mm 门不会配错。

    不用下标对齐：块数一旦不同，下标就整体错位，之后全表都是假差异。
    配不上的留在 extra_in_record / extra_in_replay 里，**逐条报出来**，
    不让块数差异静默变成「全部超差」或「跳过比较」。
    """
    pairs, used = [], set()
    for i, a in enumerate(rb):
        best, best_d = None, tol_mm
        for j, b in enumerate(gb):
            if j in used:
                continue
            d = _dxy(a, b)
            if d <= best_d:
                best, best_d = j, d
        if best is None:
            continue
        used.add(best)
        pairs.append((i, best))
    extra_rec = [i for i in range(len(rb)) if i not in {p[0] for p in pairs}]
    extra_rep = [j for j in range(len(gb)) if j not in used]
    return pairs, extra_rec, extra_rep


def compare_one(frame_dir, record_path, *, tol_mm, tol_deg, run_infer=False,
                allow_extra=False):
    """回放一帧 → 逐块差值 dict。帧/记录缺文件时抛 FileNotFoundError。"""
    rec = json.load(open(record_path))
    got = gc.analyse(frame_dir, run_infer=run_infer)
    rb, gb = rec["blocks"], got["blocks"]
    out = {
        "frame": frame_dir,
        "record": record_path,
        "n_record": len(rb),
        "n_replay": len(gb),
        "blocks": [],
        "extra_in_record": [],
        "extra_in_replay": [],
        "max_dxy_mm": 0.0,
        "max_dz_mm": 0.0,
        "max_dyaw_deg": 0.0,
        "pass": False,
    }

    pairs, extra_rec, extra_rep = _match(rb, gb)
    out["extra_in_record"] = [
        {k: rb[i].get(k) for k in ("idx", "cx", "cy", "h", "ext", "cells",
                                   "class", "n")} for i in extra_rec]
    out["extra_in_replay"] = [
        {k: gb[j].get(k) for k in ("idx", "cx", "cy", "h", "ext", "cells",
                                   "class", "n")} for j in extra_rep]

    mismatched = []
    for i, j in pairs:
        a, b = rb[i], gb[j]
        dxy = _dxy(a, b)
        dz = abs(float(a["top"]) - float(b["top"]))
        dyaw = _dyaw(a[FIELD_DEG], b[FIELD_DEG])
        row = {"rec_i": i, "rep_i": j, "dxy_mm": dxy, "dz_mm": dz,
               "dyaw_deg": dyaw,
               "rec": {k: a.get(k) for k in ("cx", "cy", "top", "yaw_rect", "class")},
               "rep": {k: b.get(k) for k in ("cx", "cy", "top", "yaw_rect", "class")}}
        out["blocks"].append(row)
        out["max_dxy_mm"] = max(out["max_dxy_mm"], dxy)
        out["max_dz_mm"] = max(out["max_dz_mm"], dz)
        out["max_dyaw_deg"] = max(out["max_dyaw_deg"], dyaw)
        if dxy > tol_mm or dz > tol_mm or dyaw > tol_deg:
            mismatched.append({"rec_i": i, "rep_i": j,
                               "dxy_mm": dxy, "dz_mm": dz, "dyaw_deg": dyaw})

    # 块数差：回放比记录**多**出来的块永远是红（那是新冒出来的簇）；
    # 记录比回放多出来的，只有在 --allow-extra 时才降级成警告 —— 它必须
    # 在报告的 extra_in_record 里逐条露面，绝不能静默。
    fatal_extra = bool(extra_rep) or (bool(extra_rec) and not allow_extra)
    out["pass"] = (not mismatched and not fatal_extra
                   and out["max_dxy_mm"] <= tol_mm
                   and out["max_dz_mm"] <= tol_mm
                   and out["max_dyaw_deg"] <= tol_deg)
    if mismatched:
        out["error"] = f"超差 {len(mismatched)} 块"
    elif fatal_extra:
        out["error"] = (f"块数不一致：记录 {len(rb)} → 回放 {len(gb)}"
                        f"（回放多 {len(extra_rep)}、记录多 {len(extra_rec)}）")
    return out


def discover(run_dir, frame_name="frame-pick"):
    """→ [(frame_dir, record_path)]。只收**两者都在**的轮次。"""
    pairs = []
    for rec in sorted(glob.glob(os.path.join(run_dir, "round-*", "candidates.json"))):
        frame = os.path.join(os.path.dirname(rec), frame_name)
        if os.path.isdir(frame):
            pairs.append((frame, rec))
    return pairs


def print_one(res):
    tag = "PASS" if res["pass"] else "FAIL"
    print(f"[{tag}] {res['frame']}")
    print(f"        块 {res['n_record']}→{res['n_replay']}  "
          f"max Δxy {res['max_dxy_mm']:.4f} mm  "
          f"max Δz {res['max_dz_mm']:.4f} mm  "
          f"max Δyaw90 {res['max_dyaw_deg']:.4f}°")
    if res.get("error"):
        print(f"        {res['error']}")
    for b in res["extra_in_record"]:
        print(f"        - 只在记录里: ({b['cx']:7.1f},{b['cy']:7.1f}) "
              f"h={b['h']:.1f} ext={b['ext']} cells={b['cells']} "
              f"{b['class']}")
    for b in res["extra_in_replay"]:
        print(f"        + 只在回放里: ({b['cx']:7.1f},{b['cy']:7.1f}) "
              f"h={b['h']:.1f} ext={b['ext']} cells={b['cells']} "
              f"{b['class']}")
    for b in res["blocks"]:
        if b["dxy_mm"] > 0.001 or b["dz_mm"] > 0.001 or b["dyaw_deg"] > 0.001:
            print(f"          块 记录#{b['rec_i']}/回放#{b['rep_i']}: "
                  f"Δxy {b['dxy_mm']:.4f} mm  "
                  f"Δz {b['dz_mm']:.4f} mm  Δyaw {b['dyaw_deg']:.4f}°")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame-dir", default=None, help="单个帧目录")
    ap.add_argument("--record", default=None, help="与 --frame-dir 配对的 candidates.json")
    ap.add_argument("--run-dir", default=None,
                    help="整批：自动找 round-*/frame-pick + candidates.json")
    ap.add_argument("--frame-name", default="frame-pick")
    ap.add_argument("--tol-mm", type=float, default=1.0)
    ap.add_argument("--tol-deg", type=float, default=1.0)
    ap.add_argument("--allow-extra", action="store_true",
                    help="记录比回放**多**出来的块降级为警告（回放多出来的仍然红）。"
                         "只用于「记录写在判据加严之前」这类可解释的历史帧，"
                         "每一块都会在附录里逐条打印")
    ap.add_argument("--json", default=None, help="把逐帧结果写到这个文件")
    args = ap.parse_args()

    if args.frame_dir:
        if not args.record:
            ap.error("--frame-dir 要配 --record（真机记录的那份 candidates.json）")
        pairs = [(args.frame_dir, args.record)]
    elif args.run_dir:
        pairs = discover(args.run_dir, args.frame_name)
        if not pairs:
            print(f"!! {args.run_dir} 下没有 round-*/{args.frame_name} + candidates.json")
            return 2
    else:
        ap.error("给 --frame-dir/--record 或 --run-dir")

    results = []
    for frame, rec in pairs:
        res = compare_one(frame, rec, tol_mm=args.tol_mm, tol_deg=args.tol_deg,
                          allow_extra=args.allow_extra)
        results.append(res)
        print_one(res)

    worst = max((r["max_dxy_mm"] for r in results), default=0.0)
    worst_z = max((r["max_dz_mm"] for r in results), default=0.0)
    worst_y = max((r["max_dyaw_deg"] for r in results), default=0.0)
    npass = sum(r["pass"] for r in results)
    print(f"\n{npass}/{len(results)} 帧一致；全场最差 "
          f"Δxy {worst:.4f} mm / Δz {worst_z:.4f} mm / Δyaw {worst_y:.4f}°"
          f"（判据 {args.tol_mm} mm / {args.tol_deg}°）")
    if args.json:
        json.dump({"results": results, "n_pass": npass,
                   "worst": {"dxy_mm": worst, "dz_mm": worst_z,
                             "dyaw_deg": worst_y},
                   "tol_mm": args.tol_mm, "tol_deg": args.tol_deg},
                  open(args.json, "w"), ensure_ascii=False, indent=1)
        print(f"→ {args.json}")
    return 0 if npass == len(results) else 2


if __name__ == "__main__":
    sys.exit(main())
