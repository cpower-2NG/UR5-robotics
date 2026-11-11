#!/usr/bin/env python3
"""O1 离线实验：单帧几何 vs 多线索实例分割，实例 IoU + 指定 top-1（S1/S2/S3 分层）。

口径（先定义，再看数）
====================
- **真值实例的顶面像素集**：由仿真真值（``truth.json`` 的 x/y/yaw/size/top）
  和该帧深度反投影得到 —— 落在块的正方形足迹内、且高度在「顶面 −3.5 mm … 顶面
  +1.5 mm」里的像素才算。只比较**可见顶面**，不建模被遮挡的侧面：两个变体和
  真值用同一把尺子。
- **预测实例的 mask**：``find_blocks(..., return_masks=True)`` 返回的该实例顶面
  像素（生产分割结果本身，不另写一份分割）。
- **实例 IoU**：每个真值块取「与它 IoU 最大的预测实例」的 IoU。合并簇会把两块
  都盖住，IoU 掉到 ~0.5；被遮挡的块自然低。
- **能算哪一个真值实例**：预测实例与真值块 IoU ≥ ``MATCH_IOU``（0.75）且一对一
  贪心分配。低于 0.75 的簇**不能代表**任何一个真值块 —— 指定 top-1 判分只认这
  个映射，所以「把两块并成一簇」在指定 ID 上直接算错，与契约「选中的实例 =
  真值指定实例」一致。
- **指定 top-1**：命令集与 E5 完全相同（``stage4_e5_eval.gen_commands``，只留
  真值上唯一指向该块的说法），在两个变体各自的分割结果上跑同一个解析器。
  分母 = 全部命令；误配（status=ok 却选错）必须 0。
- **分层**：按 ``truth.json`` 的 batch 分组（S1/S2/S3）；S2 是同色接触，单独报。
- **遮挡记账**：真值块顶面可见像素 < 本帧最完整顶面的 1/4 ⇒ 记「遮挡」（S3
  上层块压住底层块时只露一圈边）。这类目标的命令不计入「无遮挡 top-1」分母、
  也不算误配 —— 单帧切不出可靠实例是**视角**问题，归 V4 视角规划，不是分割
  失效。**判据仍用全部命令的「指定 top-1」**，口径不因遮挡放宽。

两个变体（一次只动分割线索这一个维度）
======================================
- ``geometry``：顶面高度掩码 + 4 连通 + 顶面矩形/尺寸先验 —— 不用尺寸切分，
  也不做颜色拆簇。**颜色只用于给实例贴类别标签**（指定解析需要），不参与分割。
- ``multi``：在 geometry 之上加**尺寸先验切分**与**颜色类拆簇**
  （``graspnet_candidates.split_top_face`` / ``split_top_face_by_colour``）。

契约 §2 列的多线索还包括「凹角 / 分水岭」—— **本仓未实现，O1 没有这两项**；
本实验只量化「尺寸 + 颜色」相对纯几何的增量，报告里必须写明这句。

用法：
  python3 scripts/stage4_o1_eval.py s1=<帧目录> [...] [--json out]
帧目录 = 含 color.png / depth.png / tf.json / truth.json 的一帧。
退出码：0 = 契约 §3 的指定 ID 判据过（S1/S2）；1 = 未过；2 = 数据缺失。
"""
import argparse
import json
import os
import statistics
import sys

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402
import stage4_e5_eval as ev                                       # noqa: E402
import stage4_specify as sp                                       # noqa: E402

MATCH_IOU = 0.75          # 预测实例能代表真值块的最小 IoU
NEAR_IOU = 0.30           # 低于它：既不是合并，也不是同一块
ID_PASS = ev.ID_PASS      # 0.90，契约 §3
VARIANTS = ("geometry", "multi")


# ---------------------------------------------------------------- 数据
def load_truth(frame_dir):
    with open(os.path.join(frame_dir, "truth.json"), encoding="utf-8") as f:
        return json.load(f)


def gt_top_masks(frame, truth, dz_top=0.0035, dz_bot=0.0015):
    """真值块 → 顶面可见像素掩码（全画幅 bool 图）。"""
    depth, K = frame["depth"], frame["K"]
    P, pix = gc.backproject(depth, K)
    Pb = P @ frame["R_bc"].T + frame["t_bc"]
    out = {}
    for b in truth["blocks"]:
        half = float(b["size"]) / 2.0
        th = np.radians(float(b["yaw"]))
        ct, st = np.cos(th), np.sin(th)
        dx = Pb[:, 0] - float(b["x"])
        dy = Pb[:, 1] - float(b["y"])
        # base 点旋到块系：truth.yaw 是块边相对 base 的朝向
        u = ct * dx + st * dy
        v = -st * dx + ct * dy
        inside = (np.abs(u) <= half) & (np.abs(v) <= half)
        z = Pb[:, 2]
        sel = inside & (z > float(b["top"]) - dz_top) \
            & (z < float(b["top"]) + dz_bot)
        m = np.zeros(depth.shape, dtype=bool)
        m[pix[sel, 1], pix[sel, 0]] = True
        out[int(b["id"])] = m
    return out


def measure_variant(frame, variant):
    """生产分割函数，只改「用不用多线索」这一个开关。"""
    insts, table_z = gc.find_blocks(
        frame, split=(variant == "multi"),
        hue_table=block_classes.SIM_HUE_DEG, return_masks=True)
    pool = [r for r in insts if not r.get("low") and sp._in_zone(r)]
    masks = {}
    shape = frame["depth"].shape
    split_src = {}
    for r in pool:
        m = np.zeros(shape, dtype=bool)
        uv = np.asarray(r["mask_uv"], dtype=np.int64)
        m[uv[:, 1], uv[:, 0]] = True
        masks[int(r["idx"])] = m
        split_src[r["split_src"]] = split_src.get(r["split_src"], 0) + 1
    return pool, masks, table_z, split_src


def iou(a, b):
    inter = np.count_nonzero(a & b)
    union = np.count_nonzero(a | b)
    return float(inter) / float(union) if union else 0.0


def assign(pool, masks, gts):
    """实例 ↔ 真值的一对一贪心分配（IoU 降序，门槛 MATCH_IOU）。

    返回 (pred_rep, pairs)：pred idx → truth id；以及 (iou, pred, truth) 列表。
    """
    triples = []
    for r in pool:
        i = int(r["idx"])
        for tid, gm in gts.items():
            v = iou(masks[i], gm)
            if v >= NEAR_IOU:
                triples.append((v, i, int(tid)))
    triples.sort(key=lambda x: -x[0])
    used_p, used_t, pred_rep, pairs = set(), set(), {}, []
    for v, i, tid in triples:
        if v < MATCH_IOU:
            break
        if i in used_p or tid in used_t:
            continue
        used_p.add(i)
        used_t.add(tid)
        pred_rep[i] = tid
        pairs.append({"iou": round(v, 3), "instance": i, "truth_id": tid})
    return pred_rep, pairs


def eval_frame(label, frame_dir):
    truth = load_truth(frame_dir)
    frame = gc.load_frame(frame_dir)
    gts = gt_top_masks(frame, truth)
    gt_px = {int(tid): int(m.sum()) for tid, m in gts.items()}
    # 顶面可见像素 < 本帧最完整顶面的 1/4 = 重度遮挡（S3 上层块压住底层块，
    # 只露一圈边）。这类目标在单帧里本来就切不出可靠实例，不能算分割漏检，
    # 也不能算指定解析的失败 —— 那是 V4 视角规划要解决的问题。
    max_px = max(gt_px.values(), default=0)
    occluded = {tid for tid, n in gt_px.items() if max_px and n < 0.25 * max_px}
    cmds, skipped, _recs, disagree = ev.gen_commands(truth)

    out = {"label": label, "dir": frame_dir, "batch": truth.get("batch"),
           "n_truth": len(truth["blocks"]), "n_commands": len(cmds),
           "gt_px": gt_px, "n_gt_occluded": len(occluded),
           "n_skipped_not_unique": skipped,
           "n_indep_disagree": len(disagree),
           "indep_disagree": disagree, "variants": {}}

    for variant in VARIANTS:
        pool, masks, table_z, split_src = measure_variant(frame, variant)
        pred_rep, pairs = assign(pool, masks, gts)

        best = {}
        for tid, gm in gts.items():
            best[tid] = max((iou(masks[int(r["idx"])], gm) for r in pool),
                            default=0.0)
        vals = sorted(best.values())
        n_ge = sum(1 for v in vals if v >= MATCH_IOU)
        n_mid = sum(1 for v in vals if NEAR_IOU <= v < MATCH_IOU)
        n_miss = sum(1 for v in vals if v < NEAR_IOU)
        fp = 0
        for r in pool:
            i = int(r["idx"])
            if max((iou(masks[i], gm) for gm in gts.values()), default=0.0) \
                    < NEAR_IOU:
                fp += 1

        correct = ok_correct = misbind = misbind_occ = 0
        correct_free = n_cmd_free = 0
        counts = {"ok": 0, "ambiguous": 0, "none": 0, "bad": 0}
        rows = []
        for c in cmds:
            r = sp.select(pool, c["text"])
            status = r["status"]
            counts[status] = counts.get(status, 0) + 1
            got = (pred_rep.get(int(r["instance"]))
                   if r["instance"] is not None else None)
            hit = got == int(c["target"])
            free = int(c["target"]) not in occluded
            correct += int(hit)
            ok_correct += int(status == "ok" and hit)
            misbind += int(status == "ok" and not hit and free)
            misbind_occ += int(status == "ok" and not hit and not free)
            n_cmd_free += int(free)
            correct_free += int(hit and free)
            rows.append({"text": c["text"], "target": int(c["target"]),
                         "status": status, "instance": r["instance"],
                         "truth_id": got, "correct": hit,
                         "target_unoccluded": free})

        out["variants"][variant] = {
            "table_z_mm": round(table_z * 1000.0, 2),
            "split_src": dict(sorted(split_src.items())),
            "n_instances": len(pool), "n_matched": len(pairs),
            "n_false_pos": fp,
            "iou_best": {str(k): round(v, 3) for k, v in sorted(best.items())},
            "iou_mean": round(float(np.mean(vals)), 3) if vals else None,
            "iou_median": round(statistics.median(vals), 3) if vals else None,
            "iou_ge_075": n_ge, "merged": n_mid, "missed": n_miss,
            "pairs": pairs,
            "counts": counts, "n_commands": len(cmds),
            "n_commands_unoccluded": n_cmd_free,
            "n_commands_occluded": len(cmds) - n_cmd_free,
            "correct": correct, "ok_correct": ok_correct, "misbind": misbind,
            "misbind_occluded": misbind_occ,
            "correct_unoccluded": correct_free,
            "id_rate": (correct / len(cmds)) if cmds else None,
            "id_rate_unoccluded": (correct_free / n_cmd_free)
            if n_cmd_free else None,
            "auto_top1_rate": (ok_correct / len(cmds)) if cmds else None,
            "commands": rows,
        }
    return out


# ---------------------------------------------------------------- 合并与判定
def pool_results(results):
    groups = {}
    for r in results:
        groups.setdefault(r.get("batch") or "?", []).append(r)
    out = []
    for batch, rs in sorted(groups.items()):
        row = {"label": "%s（%d 帧）" % (batch, len(rs)), "batch": batch,
               "n_frames": len(rs), "dirs": [r["dir"] for r in rs],
               "n_truth": sum(r["n_truth"] for r in rs),
               "n_gt_occluded": sum(r.get("n_gt_occluded", 0) for r in rs),
               "n_commands": sum(r["n_commands"] for r in rs),
               "n_skipped_not_unique": sum(r["n_skipped_not_unique"]
                                           for r in rs),
               "n_indep_disagree": sum(r["n_indep_disagree"] for r in rs),
               "variants": {}}
        for variant in VARIANTS:
            vs = [r["variants"][variant] for r in rs]
            vals = [v for x in vs for v in x["iou_best"].values()]
            n_cmd = sum(x["n_commands"] for x in vs)
            correct = sum(x["correct"] for x in vs)
            ok_correct = sum(x["ok_correct"] for x in vs)
            n_cmd_free = sum(x.get("n_commands_unoccluded", 0) for x in vs)
            correct_free = sum(x.get("correct_unoccluded", 0) for x in vs)
            counts = {}
            for x in vs:
                for k, v in x["counts"].items():
                    counts[k] = counts.get(k, 0) + v
            src_counts = {}
            for x in vs:
                for k, v in x.get("split_src", {}).items():
                    src_counts[k] = src_counts.get(k, 0) + v
            row["variants"][variant] = {
                "split_src": dict(sorted(src_counts.items())),
                "n_instances": sum(x["n_instances"] for x in vs),
                "n_matched": sum(x["n_matched"] for x in vs),
                "n_false_pos": sum(x["n_false_pos"] for x in vs),
                "iou_best": {}, "iou_mean": round(float(np.mean(vals)), 3)
                if vals else None,
                "iou_median": round(statistics.median(vals), 3)
                if vals else None,
                "iou_ge_075": sum(x["iou_ge_075"] for x in vs),
                "merged": sum(x["merged"] for x in vs),
                "missed": sum(x["missed"] for x in vs),
                "counts": counts, "n_commands": n_cmd,
                "n_commands_unoccluded": n_cmd_free,
                "n_commands_occluded": sum(x.get("n_commands_occluded", 0)
                                           for x in vs),
                "correct": correct, "ok_correct": ok_correct,
                "misbind": sum(x["misbind"] for x in vs),
                "misbind_occluded": sum(x.get("misbind_occluded", 0)
                                        for x in vs),
                "correct_unoccluded": correct_free,
                "id_rate": (correct / n_cmd) if n_cmd else None,
                "id_rate_unoccluded": (correct_free / n_cmd_free)
                if n_cmd_free else None,
                "auto_top1_rate": (ok_correct / n_cmd) if n_cmd else None,
                "commands": [c for x in vs for c in x["commands"]],
            }
        out.append(row)
    return out


def judge_batch(row, variant):
    """契约 §3：S1/S2 指定 ID ≥90% 且 0 误配；S3 只报不判（基线后拍板）。"""
    v = row["variants"][variant]
    fails = []
    if row.get("n_indep_disagree"):
        fails.append("独立复核与解析器不一致 %d 条" % row["n_indep_disagree"])
    if row["batch"] in ("S1", "S2"):
        if v["id_rate"] is None or v["id_rate"] < ID_PASS:
            fails.append("指定 ID %.1f%% < %.0f%%"
                         % (100 * (v["id_rate"] or 0), 100 * ID_PASS))
        if v["misbind"]:
            fails.append("误配 %d 次" % v["misbind"])
    return not fails, fails


def render(rows):
    out = ["| 批次 | 变体 | 真值块 | 遮挡 | 实例 | 配对 | 多检 | IoU 均值 "
           "| IoU 中位 | IoU≥0.75 | 合并 | 漏检 | 命令 | 指定 top-1 "
           "| 无遮挡 top-1 | 自主 top-1 | 误配 | 判定 |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        for variant in VARIANTS:
            v = r["variants"][variant]
            passed, fails = judge_batch(r, variant)
            mark = ("—" if r["batch"] == "S3"
                    else ("✅" if passed else "❌ " + "；".join(fails)))
            out.append("| %s | %s | %d | %d | %d | %d | %d | %s | %s | %d | %d "
                       "| %d | %d | %s | %s | %s | %d | %s |" % (
                           r["label"], variant, r["n_truth"],
                           r.get("n_gt_occluded", 0),
                           v["n_instances"], v["n_matched"], v["n_false_pos"],
                           "—" if v["iou_mean"] is None
                           else "%.3f" % v["iou_mean"],
                           "—" if v["iou_median"] is None
                           else "%.3f" % v["iou_median"],
                           v["iou_ge_075"], v["merged"], v["missed"],
                           v["n_commands"],
                           "—" if v["id_rate"] is None
                           else "%.1f%%" % (100 * v["id_rate"]),
                           "—" if v.get("id_rate_unoccluded") is None
                           else "%.1f%%" % (100 * v["id_rate_unoccluded"]),
                           "—" if v["auto_top1_rate"] is None
                           else "%.1f%%" % (100 * v["auto_top1_rate"]),
                           v["misbind"], mark))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="O1：单帧几何 vs 多线索分割")
    ap.add_argument("frames", nargs="+", help="label=帧目录，如 s1=.../topcam-home")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    specs = []
    for a in args.frames:
        if "=" not in a:
            ap.error("帧参数要写成 label=目录：%r" % a)
        label, d = a.split("=", 1)
        d = os.path.expanduser(d)
        need = ("truth.json", "color.png", "depth.png", "tf.json")
        miss = [n for n in need if not os.path.exists(os.path.join(d, n))]
        if miss:
            print("缺 %s：%s" % (miss, d), file=sys.stderr)
            return 2
        specs.append((label, d))

    results = [eval_frame(lbl, d) for lbl, d in specs]
    pooled = pool_results(results)
    print("## 逐帧\n")
    print(render(results))
    print("\n## 按批合并\n")
    print(render(pooled))

    bad = [c for r in pooled for v in ("multi", "geometry")
           for c in r["variants"][v]["commands"]
           if not c["correct"] and c["status"] in ("ok", "ambiguous")
           and c.get("target_unoccluded", True)]
    if bad:
        print("\n未命中明细（最多 12 条）：")
        for c in bad[:12]:
            print("  %-26s target=%s 命中真值=%s %s" % (
                c["text"], c["target"], c["truth_id"], c["status"]))

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)) or ".",
                    exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"kind": "stage4-o1", "results": results,
                       "pooled": pooled,
                       "match_iou": MATCH_IOU,
                       "cues": {"geometry": ["height mask", "4-conn",
                                             "top-face rect"],
                                "multi": ["height mask", "4-conn",
                                          "top-face rect", "size split",
                                          "colour split"],
                                "not_implemented": ["concave", "watershed"]}},
                      f, ensure_ascii=False, indent=1)
        print("\n→ %s" % args.json)

    # 判据只看 multi 变体在 S1/S2 上是否过（契约 §3）；geometry 是对照。
    judged = [r for r in pooled if r["batch"] in ("S1", "S2")]
    return 0 if all(judge_batch(r, "multi")[0] for r in judged) else 1


if __name__ == "__main__":
    sys.exit(main())
