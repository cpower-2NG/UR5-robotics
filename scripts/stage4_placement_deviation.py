#!/usr/bin/env python3
"""阶段 4：**跑后**用跑前 pick burst 的真值量一遍「手摆得准不准」—— 只归档，不判分。

为什么单独一个离线工具
----------------------
正式轮决定「近似手摆、跑前不实测、不阻断」。那手摆到底差多少？只有把
**跑前 pick 机位 burst 的真值**（`stage4_truth_from_frame --consensus`）与
layout manifest 的期望位置逐块比一遍才知道。这件事：

* **绝不 void、绝不改判分** —— 超差只记下来，由人在 `facts.voids` 里手写原因；
* **缺真值就不出数** —— 没有观测就说「偏差 0」是最坏的一种伪造；这里直接
  退出码 2、不写文件；
* yaw 按**方块对称**折周期：29.5 mm 立方块转 90° 是同一个盒子，所以偏差
  折进 [-45, +45)；拿 88° 与 2° 直接相减会报一个 86° 的假事故。

输入
----
``--truth``：跑前 pick burst 的共识真值 JSON（``blocks`` 里有 ``x/y/color/yaw``，
   米 / 度）。生成它的命令写在 `docs/真机起栈runbook.md`「阶段 4 正式轮」一节。
``--batch/--layout`` 或 ``--scene``：期望布局，默认取 `stage4_layouts` 的正式表。

跑法：
    python3 scripts/stage4_placement_deviation.py \\
        --truth <run>/pre/truth-pick.json --batch S0 --layout 1 \\
        --out <run>/placement-deviation.json

退出码：0 = 写出了偏差表（**不论偏差多大**）；2 = 输入缺失/读不懂（不出数）。
"""
import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_layouts as lay                                     # noqa: E402

#: 手摆目标（契约 §2.1：±3–5 mm / ±10°）。**只用于统计「多少块在目标内」**，
#: 不是判据 —— 超了也只记录。
TOL_MM = 5.0
TOL_DEG = 10.0
#: 期望块与观测块的配对半径。手摆偏差最大也就厘米级，30 mm 足够；
#: 真正漏摆/多摆的块会落到 unmatched 里，不会被硬配。
MATCH_RADIUS_MM = 30.0

VERDICT = ("record-only：本表不 void、不改判分；超差由人在 facts.voids 手写原因")


def wrap_yaw_deg(d_deg):
    """把角度偏差折进 [-45°, +45°) —— 正方形每 90° 一个周期。"""
    return (float(d_deg) + 45.0) % 90.0 - 45.0


def load_json(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def expected_blocks(batch=None, layout=None, scene=None):
    """期望块 → [(id, color, x_mm, y_mm, yaw_deg)]。

    ``scene`` 给的是 `stage4_layouts.scene_truth` 或等价文档（米、base 系）。
    两种来源都返回同样的毫米口径，后续匹配只看这一份。
    """
    if scene is not None:
        doc = load_json(scene) if isinstance(scene, str) else scene
        blocks = doc.get("blocks") or []
        src = scene if isinstance(scene, str) else "scene-doc"
    else:
        if batch is None or layout is None:
            raise ValueError("要么给 --scene，要么同时给 --batch/--layout")
        blocks = lay.layout_blocks(batch, layout)
        src = "layout:%s:%s" % (batch, layout)
    out = []
    for b in blocks:
        out.append({
            "id": int(b.get("id", len(out))),
            "color": str(b.get("color", "unknown")),
            "x_mm": float(b["x"]) * 1000.0,
            "y_mm": float(b["y"]) * 1000.0,
            "yaw_deg": float(b.get("yaw", 0.0)),
            "layer": int(b.get("layer", 0)),
        })
    return out, src


def observed_blocks(doc, source):
    """观测真值 → 同样字段。只取顶层（layer 0）：跑前 S0 的块不该叠塔，
    塔的上下层 xy 完全相同，全收进来会让一块期望块被配两次。"""
    flat = [b for b in (doc.get("blocks") or []) if int(b.get("layer", 0)) == 0]
    out = []
    for b in flat:
        out.append({
            "id": int(b.get("id", len(out))),
            "color": str(b.get("color", "unknown")),
            "x_mm": float(b["x"]) * 1000.0,
            "y_mm": float(b["y"]) * 1000.0,
            "yaw_deg": float(b.get("yaw", 0.0)),
            "n_frames": (b.get("meas") or {}).get("n_frames"),
        })
    if not out:
        raise ValueError("%s 里没有 layer 0 的块（真值为空 ⇒ 不出数）" % source)
    return out, source


def match_blocks(expected, observed, radius_mm=MATCH_RADIUS_MM):
    """期望 ↔ 观测的配对：颜色优先、再按距离贪心。

    颜色可能被分错（实测有），所以**同色不是硬条件** —— 只是更大距离的
    候选排在同色后面。返回 ``(pairs, unmatched_expected, unmatched_observed)``。
    """
    cand = []
    for ei, e in enumerate(expected):
        for oi, o in enumerate(observed):
            d = math.hypot(e["x_mm"] - o["x_mm"], e["y_mm"] - o["y_mm"])
            if d > radius_mm:
                continue
            same = (e["color"] == o["color"])
            cand.append((0 if same else 1, d, ei, oi))
    cand.sort()
    used_e, used_o, pairs = set(), set(), []
    for _bad, _d, ei, oi in cand:
        if ei in used_e or oi in used_o:
            continue
        used_e.add(ei)
        used_o.add(oi)
        pairs.append((ei, oi))
    pairs.sort()
    unmatched_e = [i for i in range(len(expected)) if i not in used_e]
    unmatched_o = [i for i in range(len(observed)) if i not in used_o]
    return pairs, unmatched_e, unmatched_o


def deviation_record(e, o):
    """一块的偏差记录（毫米 / 度；yaw 按 90° 周期折角）。"""
    dx = o["x_mm"] - e["x_mm"]
    dy = o["y_mm"] - e["y_mm"]
    dyaw = wrap_yaw_deg(o["yaw_deg"] - e["yaw_deg"])
    return {
        "id": e["id"],
        "color": e["color"],
        "observed_color": o["color"],
        "color_match": e["color"] == o["color"],
        "expected_mm": [round(e["x_mm"], 3), round(e["y_mm"], 3)],
        "observed_mm": [round(o["x_mm"], 3), round(o["y_mm"], 3)],
        "dx_mm": round(dx, 3),
        "dy_mm": round(dy, 3),
        "dist_mm": round(math.hypot(dx, dy), 3),
        "yaw_expected_deg": round(e["yaw_deg"], 3),
        "yaw_observed_deg": round(o["yaw_deg"], 3),
        "dyaw_deg": round(dyaw, 3),
        "n_frames": o.get("n_frames"),
    }


def summarize(records, tol_mm=TOL_MM, tol_deg=TOL_DEG):
    """统计（只是报表）。``within_tol`` 是**信息字段**，不是判决。"""
    if not records:
        return {
            "n": 0, "n_within_tol": 0, "within_tol": False,
            "max_dist_mm": None, "mean_dist_mm": None,
            "mean_dx_mm": None, "mean_dy_mm": None,
            "mean_dyaw_deg": None, "max_abs_dyaw_deg": None,
            "tol_mm": tol_mm, "tol_deg": tol_deg,
        }
    n_ok = sum(1 for r in records
               if r["dist_mm"] <= tol_mm and abs(r["dyaw_deg"]) <= tol_deg)
    return {
        "n": len(records),
        "n_within_tol": n_ok,
        "within_tol": n_ok == len(records),
        "max_dist_mm": round(max(r["dist_mm"] for r in records), 3),
        "mean_dist_mm": round(sum(r["dist_mm"] for r in records) / len(records), 3),
        "mean_dx_mm": round(sum(r["dx_mm"] for r in records) / len(records), 3),
        "mean_dy_mm": round(sum(r["dy_mm"] for r in records) / len(records), 3),
        "mean_dyaw_deg": round(sum(r["dyaw_deg"] for r in records)
                               / len(records), 3),
        "max_abs_dyaw_deg": round(max(abs(r["dyaw_deg"]) for r in records), 3),
        "tol_mm": tol_mm,
        "tol_deg": tol_deg,
    }


def build_report(truth_doc, truth_source, exp, exp_source,
                 radius_mm=MATCH_RADIUS_MM, tol_mm=TOL_MM, tol_deg=TOL_DEG):
    """整张偏差表（纯函数，好测）。"""
    obs, obs_source = observed_blocks(truth_doc, truth_source)
    pairs, ue, uo = match_blocks(exp, obs, radius_mm)
    records = [deviation_record(exp[ei], obs[oi]) for ei, oi in pairs]
    stats = summarize(records, tol_mm, tol_deg)
    return {
        "kind": "stage4-placement-deviation",
        "expected_source": exp_source,
        "observed_source": obs_source,
        "radius_mm": radius_mm,
        "n_expected": len(exp),
        "n_observed": len(obs),
        "n_matched": len(records),
        "blocks": records,
        "unmatched_expected": [{
            "id": exp[i]["id"], "color": exp[i]["color"],
            "expected_mm": [round(exp[i]["x_mm"], 3),
                            round(exp[i]["y_mm"], 3)],
        } for i in ue],
        "unmatched_observed": [{
            "color": obs[i]["color"],
            "observed_mm": [round(obs[i]["x_mm"], 3),
                            round(obs[i]["y_mm"], 3)],
        } for i in uo],
        "stats": stats,
        "yaw_period_deg": 90.0,
        "verdict": VERDICT,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="跑前 pick burst 真值 vs 布局：摆位偏差（只归档，不判分）")
    ap.add_argument("--truth", required=True,
                    help="跑前 pick burst 的共识真值 JSON")
    ap.add_argument("--scene", default=None,
                    help="期望布局 JSON（缺省用 --batch/--layout）")
    ap.add_argument("--batch", default="S0", choices=sorted(lay.BATCHES))
    ap.add_argument("--layout", type=int, default=None)
    ap.add_argument("--out", default=None,
                    help="偏差表输出路径；缺省打印到 stdout")
    ap.add_argument("--radius-mm", type=float, default=MATCH_RADIUS_MM)
    ap.add_argument("--tol-mm", type=float, default=TOL_MM,
                    help="手摆目标（只用于统计，不是判据）")
    ap.add_argument("--tol-deg", type=float, default=TOL_DEG)
    args = ap.parse_args(argv)

    try:
        if not os.path.exists(args.truth):
            raise ValueError("真值文件不存在：%s（不伪造偏差）" % args.truth)
        truth_doc = load_json(args.truth)
        exp, exp_source = expected_blocks(
            args.batch if args.scene is None else None,
            args.layout if args.scene is None else None,
            args.scene)
    except (ValueError, KeyError, OSError) as exc:
        print("!! %s" % exc)
        return 2
    try:
        rep = build_report(truth_doc, args.truth, exp, exp_source,
                           radius_mm=args.radius_mm,
                           tol_mm=args.tol_mm, tol_deg=args.tol_deg)
    except (ValueError, KeyError) as exc:
        print("!! %s" % exc)
        return 2

    text = json.dumps(rep, ensure_ascii=False, indent=1)
    if args.out:
        out_dir = os.path.dirname(os.path.abspath(args.out))
        if not os.path.isdir(out_dir):
            print("!! 输出目录不存在：%s" % out_dir)
            return 2
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
    else:
        print(text)

    s = rep["stats"]
    print("摆位偏差：%d/%d 块配上（观测 %d 块），"
          "距离 max %.1f / mean %.1f mm，yaw max |%.1f|°；目标内 %d/%d"
          % (rep["n_matched"], rep["n_expected"], rep["n_observed"],
             s["max_dist_mm"] if s["max_dist_mm"] is not None else -1,
             s["mean_dist_mm"] if s["mean_dist_mm"] is not None else -1,
             s["max_abs_dyaw_deg"] if s["max_abs_dyaw_deg"] is not None else -1,
             s["n_within_tol"], s["n"]))
    if rep["unmatched_expected"]:
        print("!! 没配上的期望块：%s（真值里没看见 ⇒ 只记录）"
              % [u["id"] for u in rep["unmatched_expected"]])
    if rep["unmatched_observed"]:
        print("!! 没配上的观测块：%s（多出来的 ⇒ 只记录）"
              % [u["observed_mm"] for u in rep["unmatched_observed"]])
    print("（%s）" % VERDICT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
