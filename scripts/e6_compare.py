#!/usr/bin/env python3
"""E6 / O2：GraspNet vs RNGNet vs 几何兜底 —— 同帧、同指定实例、同一套过滤器与选择器。

设计文档（§5.5 / §11.3 O2）要求：
  * 同一帧、同一指定实例上配对；
  * 冻结统一候选 schema 和**唯一确定性选择器**；
  * 换网只换候选源，不同时换输入/选择器；
  * 主终点：candidate recall@k、位姿误差、延迟、显存；记录 fallback_reason。

本脚本只做后处理：三路候选分别由
  * ``e6_graspnet_crop.py``   → 帧目录/graspnet.json（指定实例局部裁剪推理）
  * ``rngnet_offline.py``     → 轮根/rngnet-out/<帧>.json（patch 中心取真值 uv）
  * ``graspnet_candidates.py --no-infer`` → 帧目录/geom_blocks.json（深度聚类实测块）
产出，这里统一成同一 schema、过同一硬过滤、用同一排序键，再算指标。

硬过滤（§5.4，离线可判的 1/3/6/7 条；IK 与接近/撤退路径这里判不了，
记为 not_checked，不假装通过）：
  F1 开口 0–95 mm，且 ≥ 方块边长 −1 mm（夹不住更窄的开口不算候选）
  F2 倾斜 ≤ 35°（A 档俯视阶段的准入锥，与两条原型一致）
  F3 候选中心在指定块顶面内（外扩 4 mm）
  F4 候选中心 z 在块顶 ±12 mm 内（夹持深度）
  F5 与邻块不撞：候选中心不落进任何邻块顶面 + 2 mm（§7 的「邻块膨胀 2–3 mm」）
     —— 这是**近似**：只判候选中心，不模拟两指的扫掠体与接近路径；真正的碰撞
     判定按 §7 迁到 MoveIt。接触对（S1/S2）的净空写进报告，别把它当已过碰撞。
     早期版本按「指厚 11 + 2」膨胀到 13 mm，会把接触块上大半个顶面误判成禁区
     （S2 实测：两块中心距 30 mm，13 mm 膨胀把禁区推到离本块中心 2.25 mm），
     已改回 2 mm。
  F6 身份一致：xy 最近的块必须是指定块

排序键（§5.4 顺序：可抓性 → 对中 → 倾角 → 行程/时间）：
  (-min(碰撞余量, 10 mm), 对中距离, 倾斜角, 开口, -模型分数, 源)
碰撞余量**饱和到 10 mm**：余量 ≥10 mm 的候选在「可抓性」上视为同样安全，
否则一个远离所有块、压根没对准目标的候选会因为余量大而排到对中好的前面
（S0 实测：GraspNet 未饱和时 top-1 对中被推到 14.9 mm）。模型分数只当
同分最后一级，不参与主排序。

recall@k 用**同一把选择器键**给该源的全部候选排序（分数只在最后一级当同分
断点，满足 §5.1「分数不作排序主依据」），再看前 k 个里有没有通过过滤的；
真机执行的那个 grasp = 排序里第一个通过过滤的候选（fail-closed：没有就停）。
三路源因此只差候选本身，不差排序口径（§5.5）。

用法：
  python3 scripts/e6_compare.py --run-root RUN_ROOT [--rngnet-dir DIR]
"""
import argparse
import glob
import json
import math
import os
import statistics
import sys

import numpy as np

BLOCK_M = 0.0295
BLOCK_HALF_MM = BLOCK_M * 500.0          # 14.75 mm
OPEN_MAX_MM = 95.0
OPEN_MIN_MM = BLOCK_M * 1000.0 - 1.0
TILT_MAX_DEG = 35.0
ANCHOR_MM = 12.0
FOOT_MARGIN_MM = 4.0
Z_TOL_MM = 12.0
NEIGHBOR_MARGIN_MM = 2.0
SAFE_MARGIN_MM = 10.0
IK_NOTE = "IK/接近撤退路径离线未判（not_checked），不当作通过"


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def ang_mod90(a, b):
    d = (float(a) - float(b)) % 90.0
    return min(d, 90.0 - d)


def to_fixed(src_c):
    """三路候选 → 统一 schema（长度单位一律 mm，base 系）。"""
    if src_c["source"] == "graspnet":
        x = src_c["xyz_base"]
        return {"source": "graspnet",
                "center": [x[0] * 1000, x[1] * 1000, x[2] * 1000],
                "approach": [a * 1.0 for a in src_c["approach_base"]],
                "width_mm": src_c["width_mm"], "tilt_deg": src_c["tilt_deg"],
                "yaw_deg": src_c["angle_deg"], "score": src_c["score"],
                "raw": src_c}
    if src_c["source"] == "rngnet":
        x = src_c["xyz_base"]          # rngnet_offline 已是 mm
        return {"source": "rngnet",
                "center": [x[0], x[1], x[2]],
                "approach": [0.0, 0.0, -1.0],   # 只用来算倾斜；已按 tilt 过滤
                "width_mm": src_c["width_mm"], "tilt_deg": src_c["tilt_deg"],
                "yaw_deg": src_c["angle_deg"], "score": src_c["score"],
                "raw": src_c}
    raise ValueError(src_c["source"])


def geometric_candidates(geom_blocks, truth_blocks, max_match_mm=25.0):
    """实测块 → 几何候选；按最近中心一对一匹配到真值指定块。"""
    cands = []
    used = set()
    for gb in geom_blocks:
        best = None
        for tb in truth_blocks:
            d = math.hypot(gb["cx"] - tb["cx"], gb["cy"] - tb["cy"])
            if best is None or d < best[0]:
                best = (d, tb)
        if best is None or best[0] > max_match_mm or best[1]["idx"] in used:
            continue
        d, tb = best
        used.add(tb["idx"])
        cands.append({"source": "geometric", "instance": tb["idx"],
                      "center": [gb["cx"], gb["cy"], gb["top"]],
                      "approach": [0.0, 0.0, -1.0],
                      "width_mm": min(gb["ext"]) + 6.0,
                      "tilt_deg": 0.0,
                      "yaw_deg": (gb.get("yaw_rect") if gb.get("yaw_trusted")
                                  else gb.get("yaw", 0.0)),
                      "score": None,
                      "match_mm": round(d, 2),
                      "raw": {"measured_idx": gb.get("idx")}})
    return cands


def footprint_margin(center_xy, block, expand_mm):
    """候选中心在块（按块 yaw 旋转）外扩 footprint 里的「有符号余量」mm。

    正值 = 在外（越大越安全）；负值 = 落进 footprint。
    用矩形到点的近似：把点旋进块坐标系后取 max(|dx|-hx, |dy|-hy)。
    """
    v = np.array(center_xy) - np.array([block["cx"], block["cy"]])
    v = rot_z(-float(block["yaw"])) @ v
    hx = hy = BLOCK_HALF_MM + expand_mm
    return float(max(abs(v[0]) - hx, abs(v[1]) - hy))


def hard_filter(c, tb, all_blocks):
    """返回 (ok, reasons[], features)。"""
    reasons = []
    w = float(c["width_mm"])
    if not (OPEN_MIN_MM <= w <= OPEN_MAX_MM):
        reasons.append("opening %.1f mm 超出 [%.1f, %.1f]" % (w, OPEN_MIN_MM, OPEN_MAX_MM))
    if float(c["tilt_deg"]) > TILT_MAX_DEG:
        reasons.append("tilt %.1f° > %.0f°" % (c["tilt_deg"], TILT_MAX_DEG))
    # F3 顶面内（用中心 xy）
    dx = c["center"][0] - tb["cx"]
    dy = c["center"][1] - tb["cy"]
    v = rot_z(-float(tb["yaw"])) @ np.array([dx, dy])
    if (abs(v[0]) > BLOCK_HALF_MM + FOOT_MARGIN_MM
            or abs(v[1]) > BLOCK_HALF_MM + FOOT_MARGIN_MM):
        reasons.append("中心不在指定块顶面（块系 %.1f, %.1f mm）"
                       % (v[0], v[1]))
    # F4 夹持深度
    dz = c["center"][2] - tb["top"]
    if abs(dz) > Z_TOL_MM:
        reasons.append("z 偏差 %.1f mm > %g" % (dz, Z_TOL_MM))
    # F5 邻块碰撞
    margins = []
    for ob in all_blocks:
        if ob["idx"] == tb["idx"]:
            continue
        m = footprint_margin(c["center"][:2], ob, NEIGHBOR_MARGIN_MM)
        margins.append(m)
        if m < 0:
            reasons.append("落进邻块 #%d 的碰撞足迹" % ob["idx"])
    # F6 身份
    near = min(all_blocks, key=lambda b: math.hypot(
        c["center"][0] - b["cx"], c["center"][1] - b["cy"]))
    if near["idx"] != tb["idx"]:
        reasons.append("身份不一致：最近块 #%d" % near["idx"])
    centering = math.hypot(dx, dy)
    return (len(reasons) == 0, reasons,
            {"centering_mm": round(centering, 2),
             "collision_margin_mm": round(min(margins), 2) if margins else 99.0,
             "grasp_depth_mm": round(dz, 2)})


def selector_key(r):
    c, feats = r["c"], r["feats"]
    score = c.get("score")
    score = 0.0 if score is None else float(score)
    return (-min(feats["collision_margin_mm"], SAFE_MARGIN_MM),
            round(feats["centering_mm"], 1),
            round(float(c["tilt_deg"]), 1),
            round(float(c["width_mm"]), 1),
            -round(score, 4),
            c["source"])


def evaluate_instance(tb, cands, all_blocks):
    """一个 (帧, 指定块) 的评估：先把该源的候选全部排序，再取 top-k 判有效。"""
    rows = []
    for c in cands:
        ok, reasons, feats = hard_filter(c, tb, all_blocks)
        rows.append({"c": c, "ok": ok, "reasons": reasons, "feats": feats})
    n = len(rows)
    n_anchor = sum(1 for r in rows
                   if math.hypot(r["c"]["center"][0] - tb["cx"],
                                 r["c"]["center"][1] - tb["cy"]) <= ANCHOR_MM)
    ranked = sorted(rows, key=selector_key)        # 统一选择器键 → recall@k
    valid = [r for r in ranked if r["ok"]]
    top = valid[0] if valid else None              # 第一个通过过滤的 → 执行用
    res = {
        "instance": tb["idx"], "class": tb["class"],
        "n_candidates": n, "n_anchor": n_anchor,
        "n_valid": len(valid),
        "recall@1": any(r["ok"] for r in ranked[:1]),
        "recall@5": any(r["ok"] for r in ranked[:5]),
        "recall@10": any(r["ok"] for r in ranked[:10]),
        "selected": None,
    }
    if top is not None:
        res["selected"] = res["top1"] = {
            "center": [round(v, 2) for v in top["c"]["center"]],
            "width_mm": round(float(top["c"]["width_mm"]), 2),
            "tilt_deg": round(float(top["c"]["tilt_deg"]), 2),
            "score": top["c"].get("score"),
            "valid": top["ok"],
            "reasons": top["reasons"],
            "centering_mm": top["feats"]["centering_mm"],
            "grasp_depth_mm": top["feats"]["grasp_depth_mm"],
            "neighbor_margin_mm": top["feats"]["collision_margin_mm"],
            "yaw_err_deg": round(ang_mod90(top["c"]["yaw_deg"], tb["yaw"]), 1),
        }
    if res["n_valid"] > 0:
        res["fallback_reason"] = "none"
    elif n == 0:
        res["fallback_reason"] = "no_candidate"
    elif n_anchor == 0:
        res["fallback_reason"] = "no_anchor"
    else:
        ref = ranked[0] if ranked else None
        tags = sorted({r.split(" ")[0] for r in ref["reasons"]}) if ref else []
        res["fallback_reason"] = "filtered:" + ";".join(tags)
    return res


def mcnemar_pairs(a, b):
    """ids 配对：a/b 是 {instance_key: bool}。返回 (b01, b10, p)。"""
    b01 = sum(1 for k in a if not a[k] and b.get(k))
    b10 = sum(1 for k in a if a[k] and not b.get(k))
    n = b01 + b10
    if n == 0:
        return b01, b10, 1.0
    # 精确二项双尾
    from math import comb
    p = 2 * sum(comb(n, k) for k in range(0, min(b01, b10) + 1)) / 2 ** n
    return b01, b10, min(1.0, p)


def summarize(rows):
    vals = [r for r in rows]
    ok = [r["recall@1"] for r in vals]
    out = {
        "n_instances": len(vals),
        "hit_rate_any_valid": round(sum(1 for r in vals if r["n_valid"] > 0)
                                    / max(1, len(vals)), 3),
        "recall@1": round(sum(r["recall@1"] for r in vals) / max(1, len(vals)), 3),
        "recall@5": round(sum(r["recall@5"] for r in vals) / max(1, len(vals)), 3),
        "recall@10": round(sum(r["recall@10"] for r in vals) / max(1, len(vals)), 3),
        "fallback_reasons": {},
    }
    for r in vals:
        k = r["fallback_reason"].split(":")[0]
        out["fallback_reasons"][k] = out["fallback_reasons"].get(k, 0) + 1
    errs = [r["top1"]["centering_mm"] for r in vals if r.get("top1")]
    yaws = [r["top1"]["yaw_err_deg"] for r in vals if r.get("top1")]
    if errs:
        out["top1_centering_mm"] = {
            "median": round(statistics.median(errs), 2),
            "p90": round(sorted(errs)[max(0, int(0.9 * len(errs)) - 1)], 2),
            "max": round(max(errs), 2)}
    if yaws:
        out["top1_yaw_err_deg"] = {
            "median": round(statistics.median(yaws), 1),
            "max": round(max(yaws), 1)}
    margins = [r["top1"]["neighbor_margin_mm"] for r in vals if r.get("top1")]
    if margins:
        out["top1_neighbor_margin_mm"] = {
            "median": round(statistics.median(margins), 1),
            "min": round(min(margins), 1)}
        # 选中候选离邻块足迹 <15 mm（≈块边相贴）的个数：接触对，真碰撞要 MoveIt 判
        out["selected_tight_neighbors"] = sum(1 for m in margins if m < 15.0)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-root", required=True,
                    help="本轮根目录（其下 frames-*/ 与 rngnet-out/）")
    ap.add_argument("--rngnet-dir", default=None,
                    help="默认 RUN_ROOT/rngnet-out")
    ap.add_argument("--out", default=None, help="默认 RUN_ROOT/report")
    args = ap.parse_args()

    root = args.run_root
    rngnet_dir = args.rngnet_dir or os.path.join(root, "rngnet-out")
    out_base = args.out or os.path.join(root, "report")

    frame_dirs = sorted(d for d in glob.glob(os.path.join(root, "frames-*", "*"))
                        if os.path.isdir(d))
    all_results = {}
    paired = {}
    lat = {}
    for fd in frame_dirs:
        need = [os.path.join(fd, n) for n in ("truth.json", "candidates.json")]
        if not all(os.path.exists(p) for p in need):
            print("== %s  跳过（缺 truth.json / candidates.json）" % fd)
            continue
        truth = json.load(open(need[0]))
        blocks = json.load(open(need[1]))["blocks"]
        gp_path = os.path.join(fd, "graspnet.json")
        geom_path = os.path.join(fd, "geom_blocks.json")
        rn_name = "%s-%s.json" % (os.path.basename(os.path.dirname(fd)),
                                  os.path.basename(fd))
        rn_path = os.path.join(rngnet_dir, rn_name)
        have = {"graspnet": os.path.exists(gp_path),
                "rngnet": os.path.exists(rn_path),
                "geometric": os.path.exists(geom_path)}
        print("== %s  可用源 %s" % (fd, have))

        fb = {b["idx"]: b for b in blocks}
        truth_by_id = {int(b["id"]): b for b in truth["blocks"]}
        # 统一用 candidates.json 的指定块（真值）；truth.json 只提供 id 映射
        all_blocks = [{"idx": b["idx"], "class": b["class"],
                       "cx": b["cx"], "cy": b["cy"], "top": b["top"],
                       "yaw": b["yaw"]} for b in blocks]

        sources = {}
        if have["graspnet"]:
            gp = json.load(open(gp_path))
            lat.setdefault("graspnet_ms", []).extend(gp.get("infer_ms", []))
            for gb in gp["blocks"]:
                cands = []
                for c in gb["candidates"]:
                    c = dict(c, source="graspnet")
                    cands.append(to_fixed(c))
                sources.setdefault("graspnet", {})[gb["idx"]] = cands
        if have["rngnet"]:
            rn = json.load(open(rn_path))
            lat.setdefault("rngnet_frame_s", []).append(rn.get("infer_sec"))
            for pb in rn["per_block"]:
                cands = []
                for c in pb.get("near", []):
                    c = dict(c, source="rngnet")
                    cands.append(to_fixed(c))
                sources.setdefault("rngnet", {})[pb["block_idx"]] = cands
        if have["geometric"]:
            geom = json.load(open(geom_path))
            for c in geometric_candidates(geom["blocks"], blocks):
                sources.setdefault("geometric", {}).setdefault(
                    c["instance"], []).append(c)

        key = os.path.basename(os.path.dirname(fd)) + "/" + os.path.basename(fd)
        all_results[key] = {}
        for src, per in sources.items():
            all_results[key][src] = []
            for b in blocks:
                rows = per.get(b["idx"], [])
                r = evaluate_instance(b, rows, all_blocks)
                r["source"] = src
                r["frame"] = key
                all_results[key][src].append(r)

        # 配对：同一 (帧, 实例) 上 top-1 是否有效
        for src in ("graspnet", "rngnet", "geometric"):
            if src in all_results.get(key, {}):
                paired.setdefault(src, {})
                for r in all_results[key][src]:
                    paired[src]["%s#%d" % (key, r["instance"])] = r["recall@1"]

    if not all_results:
        print("没有可用帧（需要 truth.json/candidates.json/至少一个源）")
        return 2

    batches = {}
    for key, per_src in all_results.items():
        batch = key.split("/")[0]
        for src, rows in per_src.items():
            batches.setdefault(batch, {}).setdefault(src, []).extend(rows)

    summary = {"by_batch": {}, "latency": {}}
    for batch in sorted(batches):
        summary["by_batch"][batch] = {
            src: summarize(rows) for src, rows in batches[batch].items()}
    for src in ("graspnet", "rngnet", "geometric"):
        rows = [r for f in all_results.values() for r in f.get(src, [])]
        if rows:
            summary.setdefault("overall", {})[src] = summarize(rows)
    summary["latency"] = {
        "graspnet_infer_ms": {
            "n": len(lat.get("graspnet_ms", [])),
            "median": round(statistics.median(lat["graspnet_ms"]), 1)
            if lat.get("graspnet_ms") else None,
            "p90": round(sorted(lat["graspnet_ms"])[
                max(0, int(0.9 * len(lat["graspnet_ms"])) - 1)], 1)
            if lat.get("graspnet_ms") else None},
        "rngnet_infer_s_per_frame": lat.get("rngnet_frame_s"),
        "graspnet_vram_mb": json.load(open(os.path.join(
            os.path.dirname(frame_dirs[0]), "graspnet_summary.json")
        ))["frames"][0]["vram_peak_mb"] if os.path.exists(os.path.join(
            os.path.dirname(frame_dirs[0]), "graspnet_summary.json")) else None,
    }
    # E6 判据（阶段四合同）：每网在 S0–S2 各自 ≥80% 的指定块有有效候选；
    # 两网合并（任一网有有效候选）≥95%。
    acc = {}
    for batch, per_src in batches.items():
        n_inst = max(len(rows) for rows in per_src.values())
        row = {}
        merged = set()
        for src, rows in per_src.items():
            row[src] = round(sum(1 for r in rows if r["n_valid"] > 0)
                             / max(1, len(rows)), 3)
            merged |= {r["instance"] for r in rows if r["n_valid"] > 0}
        row["merged"] = round(len(merged) / max(1, n_inst), 3)
        acc[batch] = row
    summary["e6_acceptance"] = acc

    summary["mcnemar"] = {}
    if "graspnet" in paired and "rngnet" in paired:
        for batch in sorted(batches):
            pre = batch + "/"
            a = {k: v for k, v in paired["graspnet"].items() if k.startswith(pre)}
            b = {k: v for k, v in paired["rngnet"].items() if k.startswith(pre)}
            if not a or not b:
                continue
            b01, b10, p = mcnemar_pairs(a, b)
            summary["mcnemar"][batch] = {
                "n_pairs": len(a),
                "graspnet_fail_rngnet_pass": b01,
                "graspnet_pass_rngnet_fail": b10,
                "p_exact": round(p, 4)}

    json.dump({"summary": summary, "frames": all_results,
               "notes": [IK_NOTE]},
              open(out_base + ".json", "w"), ensure_ascii=False, indent=1)

    lines = ["# E6/O2 双网配对对比（同帧、同指定实例、统一过滤器/选择器）", "",
             "> 指定块取仿真真值；" + IK_NOTE + "。",
             "> 三路候选一律用同一把选择器键排序（分数只作同分断点）；"
             "「任一有效」「对中」是排序里第一个通过过滤的候选，也就是真机会执行的 grasp。",
             "> 碰撞只按候选中心近似判邻块（不模拟指形扫掠体）；"
             "「邻块余量」供看接触对的风险，真碰撞判定按 §7 交给 MoveIt。",
             ""]
    for batch in sorted(summary["by_batch"]):
        lines += ["### %s" % batch, "",
                  "| 源 | 实例数 | 任一有效 | recall@1 | recall@5 | recall@10 | 中位对中 | p90 对中 | 邻块余量中位/最小 | 贴邻块选中 | 失败原因 |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for src in ("graspnet", "rngnet", "geometric"):
            s = summary["by_batch"][batch].get(src)
            if not s:
                continue
            nm = s.get("top1_neighbor_margin_mm", {})
            lines.append("| %s | %d | %.2f | %.2f | %.2f | %.2f | %s | %s | %s/%s | %s | %s |"
                         % (src, s["n_instances"], s["hit_rate_any_valid"],
                            s["recall@1"], s["recall@5"], s["recall@10"],
                            s.get("top1_centering_mm", {}).get("median", "-"),
                            s.get("top1_centering_mm", {}).get("p90", "-"),
                            nm.get("median", "-"), nm.get("min", "-"),
                            s.get("selected_tight_neighbors", "-"),
                            s["fallback_reasons"]))
        lines.append("")
    lines += ["", "## 延迟", "",
              "- GraspNet 单块局部裁剪推理：n=%d，中位 %s ms，p90 %s ms，峰值显存 %s MB"
              % (summary["latency"]["graspnet_infer_ms"]["n"],
                 summary["latency"]["graspnet_infer_ms"]["median"],
                 summary["latency"]["graspnet_infer_ms"]["p90"],
                 summary["latency"]["graspnet_vram_mb"]),
              "- RNGNet 单帧（真值 uv 的块全跑，6 块 × 3×3 = 54 patch 一次前向）："
              "本进程各帧 %s s；**同一进程内第一帧是冷启动**（实测 4.56–5.93 s，"
              "CUDA/首次前向开销），稳态 0.02–0.08 s"
              % (summary["latency"]["rngnet_infer_s_per_frame"]),
              "- GraspNet 的首个裁剪推理同样有冷启动（473–600 ms），上表只含稳态；"
              "设计文档的「上板 ≤100 ms/帧」指单实例 patch，板端尚未测。",
              ""]
    overall = summary.get("overall", {})
    if overall:
        lines += ["## 全部批次合计", "",
                  "| 源 | 实例数 | 任一有效 | recall@1 | recall@5 | recall@10 | 中位对中 | p90 对中 |",
                  "|---|---|---|---|---|---|---|---|"]
        for src in ("graspnet", "rngnet", "geometric"):
            s = overall.get(src)
            if s:
                lines.append("| %s | %d | %.2f | %.2f | %.2f | %.2f | %s | %s |"
                             % (src, s["n_instances"], s["hit_rate_any_valid"],
                                s["recall@1"], s["recall@5"], s["recall@10"],
                                s.get("top1_centering_mm", {}).get("median", "-"),
                                s.get("top1_centering_mm", {}).get("p90", "-")))
        lines.append("")
    if summary.get("e6_acceptance"):
        lines += ["## E6 判据核算（阶段四合同）", "",
                  "口径：每网在 S0–S2 各自 ≥80% 的指定块有有效候选；两网合并 ≥95%。", ""]
        for batch in sorted(summary["e6_acceptance"]):
            a = summary["e6_acceptance"][batch]
            lines.append("- %s：GraspNet %.0f%%，RNGNet %.0f%%，合并 %.0f%%"
                         % (batch, 100 * a.get("graspnet", 0),
                            100 * a.get("rngnet", 0), 100 * a.get("merged", 0)))
        ok_net = all(a.get(s, 0) >= 0.8
                     for a in summary["e6_acceptance"].values()
                     for s in ("graspnet", "rngnet"))
        ok_merge = all(a.get("merged", 0) >= 0.95
                       for a in summary["e6_acceptance"].values())
        lines += ["", "判决：单网 ≥80%% —— %s；合并 ≥95%% —— %s。"
                  % ("通过" if ok_net else "**未通过**",
                     "通过" if ok_merge else "**未通过**"), "",
                  "**敏感度（必读）**：上表用 §7 的「邻块膨胀 2–3 mm」当 F5 门限。"
                  "把门限换成指厚近似 13 mm 时，接触对（S1/S2）大半个顶面会被判成"
                  "禁区，S2 掉成 GraspNet 67%、合并 83%，判据不过。所以这里的"
                  "「通过」只说明候选中心落在指定块顶面上、且不压邻块，**不证明"
                  "无碰撞**——指形扫掠体与接近路径要等 MoveIt 场景（§7）才算得清。"
                  "「贴邻块选中」列就是留给人看这个风险的。", ""]
    if summary["mcnemar"]:
        lines += ["## 配对检验（统一选择器下 recall@1）", ""]
        for batch, m in summary["mcnemar"].items():
            lines.append("- %s：n=%d，GraspNet 败 / RNGNet 胜 %d 对，"
                         "GraspNet 胜 / RNGNet 败 %d 对，精确双尾 p=%.4f"
                         % (batch, m["n_pairs"], m["graspnet_fail_rngnet_pass"],
                            m["graspnet_pass_rngnet_fail"], m["p_exact"]))
    lines += ["", "## 复现（在仓工作树根目录跑；RUN=轮根）", "", "```",
              "# 1 起仿真（换 scene:=S1:0 / S2:0 后重复 2–7）",
              "ros2 launch arm_grasp_sim full.launch.py skip_planner:=true headless:=true \\",
              "    scene:=S0:0 model=~/robotics/models/block_best_7cls.pt",
              "# 2 抓一帧（俯视；带 gz 真值）",
              "python3 scripts/sim_capture_frame.py RUN/frames-s0/topcam-home --camera topcam \\",
              "    --truth /tmp/grasp_scene_S0_0/scene_truth.json",
              "# 3 把真值写成指定实例（E6 不含分割误差）",
              "python3 scripts/e6_truth_blocks.py FRAME",
              "# 4 几何兜底源",
              "python3 scripts/graspnet_candidates.py FRAME --no-infer --json FRAME/geom_blocks.json",
              "# 5 GraspNet：指定实例 0.30 m 局部裁剪",
              "~/robotics/venvs/graspnet/bin/python3 scripts/e6_graspnet_crop.py FRAME",
              "# 6 RNGNet：以真值 uv 为 patch 中心",
              "~/robotics/venvs/rngnet/bin/python3 scripts/rngnet_offline.py FRAME \\",
              "    --out-root RUN/rngnet-out",
              "# 7 统一过滤/选择 + 指标 + 报告",
              "python3 scripts/e6_compare.py --run-root RUN",
              "```", ""]
    open(out_base + ".md", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
