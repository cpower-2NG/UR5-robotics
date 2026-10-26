#!/usr/bin/env python3
"""验收：放置位置精度、类别判定、计数 —— 口径见 RESTRUCTURE_PLAN 的 A1/A2。

三件事分开量，因为它们的「真值」来自不同地方：

| 量 | 真值来自 | 误差下限 |
|---|---|---|
| 放置位置 | 槽位坐标（人手定的） vs 帧里块顶面中心 | 台面单帧噪声 σ 1.2–2.6 mm |
| 解算位置 | 候选 xy vs 块顶面中心（同帧，`candidates.json`） | 腕部外参 1.13 mm / 0.31° |
| 类别 | YOLO 7 类 vs 顶面 HSV 颜色表；再看槽位类别 | 颜色表实测色相间隔 ≥ 8° |

**不使用推理**（GraspNet）—— 验收只量几何与颜色，所以这一支只要 numpy。
YOLO 那一步才需要 ultralytics（`~/robotics/venvs/robot-vision`）。

用法：
  # 单个帧：量槽位上的块
  python3 precision_verify.py --frame F --slots "0.45,-0.20;0.45,-0.30"
  # 整轮出报告
  python3 precision_verify.py --run-dir RUN --weights W.pt --out-report RUN/report
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402


# ---------------------------------------------------------------- 位置
def measure_placements(frame_dir, slots_mm, min_h_mm=15.0, max_dist_mm=80.0):
    """槽位 → 放上去的块。每个槽取**最近的块**，超过 max_dist 视为没放上。"""
    frame = gc.load_frame(frame_dir)
    blocks, table_z = gc.find_blocks(frame)
    blocks = [b for b in blocks if b["h"] >= min_h_mm]
    for i, b in enumerate(blocks):
        b["idx"] = i
    out = {"frame": frame_dir, "table_z_mm": round(table_z * 1000, 2),
           "blocks": blocks, "slots": []}
    claimed = set()
    for k, (sx, sy) in enumerate(slots_mm):
        if blocks:
            d = [float(np.hypot(b["cx"] - sx, b["cy"] - sy)) for b in blocks]
            j = int(np.argmin(d))
            best = blocks[j] if d[j] <= max_dist_mm else None
        else:
            d, j, best = [], None, None
        rec = {"slot_idx": k, "slot_xy_mm": [sx, sy], "block": best,
               "dx_mm": None, "dy_mm": None, "dist_mm": None}
        if best is not None:
            claimed.add(best["idx"])
            rec.update(dx_mm=round(best["cx"] - sx, 2),
                       dy_mm=round(best["cy"] - sy, 2),
                       dist_mm=round(d[j], 2))
        out["slots"].append(rec)
    # 不属于任何槽的块（放歪了 / 没搬走的）也要报出来，不能静默丢掉
    out["unclaimed"] = [b for b in blocks if b["idx"] not in claimed]
    return out


# ---------------------------------------------------------------- 类别
def yolo_detect(frame_dir, weights, conf=0.25, imgsz=640):
    """YOLO 7 类。返回每个检测的类别名与中心像素。

    类别顺序以**模型自己的 names** 为准；与 ``DEFAULT_CLASSES`` 不一致时大声告警 ——
    顺序错了会让「红色块判成橙色」这种错看起来像模型精度问题。
    """
    from ultralytics import YOLO                                  # 延迟 import
    m = YOLO(weights)
    names = {int(k): v for k, v in (m.names or {}).items()}
    order = [names[k] for k in sorted(names)]
    warn = None
    if order != list(block_classes.DEFAULT_CLASSES):
        warn = ("⚠️ 模型类别顺序 %s ≠ 代码里的 %s —— 类别判定要人工复核"
                % (order, list(block_classes.DEFAULT_CLASSES)))
    r = m.predict(os.path.join(frame_dir, "color.png"), conf=conf, imgsz=imgsz,
                  verbose=False)[0]
    dets = []
    for b in r.boxes:
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
        dets.append({"class": names.get(int(b.cls[0]), "?"),
                     "conf": round(float(b.conf[0]), 3),
                     "uv": [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)],
                     "box": [round(v, 1) for v in (x1, y1, x2, y2)]})
    dets.sort(key=lambda d: -d["conf"])
    return dets, warn


def classify_blocks(frame_dir, weights=None, conf=0.25, imgsz=640,
                    max_uv_mm=60.0, min_h_mm=15.0):
    """每块两列类别：颜色表（顶面 HSV）与 YOLO（最近的检测框）。

    ``max_uv_mm`` 的用法：YOLO 的框中心与块顶面中心投影到图像后，**像素距离**
    超过阈值就不配对（否则一个检测框会被配给好几块）。
    """
    frame = gc.load_frame(frame_dir)
    blocks, _ = gc.find_blocks(frame)
    blocks = [b for b in blocks if b["h"] >= min_h_mm]
    for i, b in enumerate(blocks):
        b["idx"] = i
    dets, warn = ([], None) if not weights else yolo_detect(frame_dir, weights, conf, imgsz)
    used = set()
    for b in blocks:
        b["yolo"] = None
        b["yolo_conf"] = None
        b["uv_dist_px"] = None
        if not dets:
            continue
        d = [float(np.hypot(x["uv"][0] - b["uv"][0], x["uv"][1] - b["uv"][1]))
             for x in dets]
        j = int(np.argmin(d))
        if d[j] <= max_uv_mm and j not in used:
            used.add(j)
            b["yolo"] = dets[j]["class"]
            b["yolo_conf"] = dets[j]["conf"]
            b["uv_dist_px"] = round(d[j], 1)
    return {"frame": frame_dir, "blocks": blocks, "detections": dets,
            "unmatched_detections": [x for i, x in enumerate(dets) if i not in used],
            "warning": warn}


# ---------------------------------------------------------------- 执行日志
def summarize_exec_log(path):
    """计划 JSONL → 计数口径要的几个数（A1：闭合回读 > 0、抬升后不变）。"""
    recs = []
    if path and os.path.exists(path):
        for line in open(path):
            line = line.strip()
            if line:
                try:
                    recs.append(json.loads(line))
                except ValueError:
                    pass
    # 被拒的闸门之后**又跑过轨迹** ⇒ 那是另起了一段（人工 --resume 续跑）。
    # 前一段那次拒绝是历史，不该把这一轮判成「闸门没过」—— 块其实已经放好了。
    # 反过来，被拒的闸门之后什么都没有（正常在闸门处停住），就照常判红。
    cut = 0
    for i, r in enumerate(recs):
        if r.get("event") == "gate" and not r.get("ok"):
            if any(x.get("event") == "traj" for x in recs[i + 1:]):
                cut = i + 1
    # 只有闸门/轨迹按段切；夹爪回读与 grasp_check 是**整轮的证据**（闭合、抬升
    # 都在第一段里），切掉就拿不到 close_mm / lift_mm 了。
    traj = [r for r in recs[cut:] if r.get("event") == "traj"]
    gates = [r for r in recs[cut:] if r.get("event") == "gate"]
    grips = [r for r in recs if r.get("event") == "gripper"]
    arrive = [r for r in recs if r.get("event") == "arrive"]
    check = next((r for r in recs if r.get("event") == "grasp_check"), None)

    def grip_pos(action, where_suffix=".after"):
        for r in grips:
            if (r.get("action") == action and str(r.get("where", "")).endswith(where_suffix)
                    and r.get("position_mm") is not None):
                return r["position_mm"]
        return None

    return {
        "n_traj": len(traj),
        "n_traj_failed": sum(1 for r in traj if not r.get("ok")),
        "traj_ok": bool(traj) and all(r.get("ok") for r in traj),
        "n_gate": len(gates),
        "gate_all_ok": bool(gates) and all(r.get("ok") for r in gates),
        "arrive_err_max_deg": (max(r.get("err_deg", 0.0) for r in arrive)
                               if arrive else None),
        "close_mm": grip_pos("close"),
        "open_mm": grip_pos("open"),
        "lift_mm": (check or {}).get("lifted_mm"),
        "slipped": (check or {}).get("slipped"),
    }


# ---------------------------------------------------------------- 报告
def _class_status(colour, yolo, slot_class, yolo_ran=True):
    """类别判定的**五态**，别把「没跑」「没检出」「判错了」混成一句 ✗：

    · ``ok``            YOLO == 颜色表，且槽位类别（若有限制）== YOLO
    · ``not_run``       报告时没给权重 —— YOLO 根本没跑（**不是**结论）
    · ``yolo_missing``  YOLO 跑了但没给框（也**不是**判错 —— 通常是模型没覆盖这种视角）
    · ``mismatch``      YOLO 与颜色表不一致
    · ``wrong_slot``    两者一致，但槽位类别不对
    """
    if not yolo_ran:
        return "not_run"
    if not slot_class:
        if yolo is None:
            return "yolo_missing"
        return "ok" if colour is not None and yolo == colour else "mismatch"
    if yolo is None:
        return "yolo_missing"
    if colour is not None and yolo != colour:
        return "mismatch"
    return "ok" if yolo == slot_class else "wrong_slot"


def build_report(run_dir, weights=None, conf=0.25, imgsz=640):
    state = json.load(open(os.path.join(run_dir, "round.json")))
    slots = state["slots_mm"]
    slot_classes = state.get("slot_classes") or [""] * len(slots)
    cycles = state.get("cycles", [])
    rows = []
    for cyc in cycles:
        frame = cyc.get("place_frame")
        meas = cls = None
        if frame and os.path.isdir(os.path.join(run_dir, frame)):
            fdir = os.path.join(run_dir, frame)
            meas = measure_placements(fdir, slots)
            cls = classify_blocks(fdir, weights, conf, imgsz)
        slot_idx = cyc.get("slot_idx")
        place = None
        if meas is not None and slot_idx is not None:
            place = next((s for s in meas["slots"] if s["slot_idx"] == slot_idx), None)
        colour = yolo = None
        if cls is not None and place and place["block"]:
            bidx = place["block"]["idx"]
            b = next((x for x in cls["blocks"] if x["idx"] == bidx), None)
            if b:
                colour, yolo = b.get("class"), b.get("yolo")
        slot_class = slot_classes[slot_idx] if slot_idx is not None else ""
        logsum = summarize_exec_log(os.path.join(run_dir, cyc["exec_log"])
                                   if cyc.get("exec_log") else None)
        cstatus = _class_status(colour, yolo, slot_class, yolo_ran=weights is not None)
        rows.append({
            "n": cyc.get("n"), "ok": cyc.get("ok"),
            "block_idx": cyc.get("block_idx"), "pick_class": cyc.get("class"),
            "cand_dist_mm": cyc.get("cand_dist_mm"), "cand_score": cyc.get("cand_score"),
            "slot_idx": slot_idx,
            "slot_xy_mm": [round(v, 1) for v in slots[slot_idx]]
            if slot_idx is not None else None,
            "placed_xy_mm": [round(place["block"]["cx"], 2),
                             round(place["block"]["cy"], 2)] if place and place["block"] else None,
            "place_dx_mm": place["dx_mm"] if place else None,
            "place_dy_mm": place["dy_mm"] if place else None,
            "place_dist_mm": place["dist_mm"] if place else None,
            "class_colour": colour, "class_yolo": yolo,
            "yolo_conf": next((b.get("yolo_conf") for b in (cls or {}).get("blocks", [])
                               if place and place["block"]
                               and b["idx"] == place["block"]["idx"]), None),
            "slot_class": slot_class or "",
            "class_status": cstatus,
            "class_ok": cstatus == "ok",
            "close_mm": logsum["close_mm"], "lift_mm": logsum["lift_mm"],
            "slipped": logsum["slipped"], "traj_ok": logsum["traj_ok"],
            "gate_all_ok": logsum["gate_all_ok"],
            "arrive_err_max_deg": logsum["arrive_err_max_deg"],
        })
    placed = [r for r in rows if r["ok"] and r["placed_xy_mm"] is not None]
    dists = [r["place_dist_mm"] for r in placed if r["place_dist_mm"] is not None]
    n_slots = len(slots)
    stop = state.get("stop_reason") or ("循环未结束" if len(rows) < n_slots else "槽位用尽")
    report = {
        "run_dir": run_dir,
        "n_slots": n_slots,
        "placed": "%d/%d" % (len(placed), n_slots),
        "stop_reason": stop,
        "acceptance": {
            "count": "%d/%d" % (len(placed), n_slots),
            "position": {
                "n": len(dists),
                "median_mm": round(float(np.median(dists)), 2) if dists else None,
                "max_mm": round(max(dists), 2) if dists else None,
                "baseline_note": "外参 1.13 mm / 0.31°，台面单帧噪声 σ 1.2–2.6 mm",
            },
            "class": {
                "n": len(rows),
                "correct": sum(1 for r in rows if r["class_ok"]),
                "not_run": sum(1 for r in rows if r["class_status"] == "not_run"),
                "yolo_missing": sum(1 for r in rows if r["class_status"] == "yolo_missing"),
                "mismatch": sum(1 for r in rows if r["class_status"] == "mismatch"),
                "wrong_slot": sum(1 for r in rows if r["class_status"] == "wrong_slot"),
                "all_ok": all(r["class_ok"] for r in rows) if rows else False,
            },
            "exec": {
                "all_traj_ok": all(r["traj_ok"] for r in rows) if rows else False,
                "all_gates_ok": all(r["gate_all_ok"] for r in rows) if rows else False,
                "none_slipped": all(r["slipped"] is False for r in rows) if rows else False,
            },
        },
        "rows": rows,
        "weights": weights,
    }
    return report


def _md(report):
    L = ["# 精密抓取一轮 · 验收报告", "",
         "运行目录：`%s`" % report["run_dir"], "",
         "| 项 | 结果 |", "|---|---|",
         "| 放置计数 | **%s**（收工：%s） |" % (report["placed"], report["stop_reason"]),
         "| 位置精度 中位 / 最大 | %s / %s mm（%d 块） |"
         % (report["acceptance"]["position"]["median_mm"],
            report["acceptance"]["position"]["max_mm"],
            report["acceptance"]["position"]["n"]),
         "| 类别正确 | %d/%d（YOLO 未跑 %d，未检出 %d，不一致 %d，错槽 %d） |"
         % (report["acceptance"]["class"]["correct"],
            report["acceptance"]["class"]["n"],
            report["acceptance"]["class"]["not_run"],
            report["acceptance"]["class"]["yolo_missing"],
            report["acceptance"]["class"]["mismatch"],
            report["acceptance"]["class"]["wrong_slot"]),
         "| 轨迹 / 闸门 / 无滑移 | %s / %s / %s |"
         % (report["acceptance"]["exec"]["all_traj_ok"],
            report["acceptance"]["exec"]["all_gates_ok"],
            report["acceptance"]["exec"]["none_slipped"]),
         "",
         "位置误差的**读数下限**：" + report["acceptance"]["position"]["baseline_note"], "",
         "## 逐块", "",
         "| # | 抓的块 | 候选偏 | 槽 | 放后中心 | Δx | Δy | 距离 | 颜色表 | YOLO | 槽类别 | 类别判定 | 闭合回读 | 抬升回读 |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    badge = {"ok": "✓", "not_run": "没跑YOLO", "yolo_missing": "YOLO未检出",
             "mismatch": "**不一致**", "wrong_slot": "**错槽**"}
    for r in report["rows"]:
        L.append("| %s | #%s %s | %s | %s %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["n"], r["block_idx"], r["pick_class"], r["cand_dist_mm"],
            r["slot_idx"], r["slot_xy_mm"], r["placed_xy_mm"],
            r["place_dx_mm"], r["place_dy_mm"], r["place_dist_mm"],
            r["class_colour"], r["class_yolo"], r["slot_class"] or "*",
            badge.get(r["class_status"], r["class_status"]),
            r["close_mm"], r["lift_mm"]))
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default=None)
    ap.add_argument("--slots", default=None, help="x,y;x,y  （毫米或米，按小数位判）")
    ap.add_argument("--weights", default=None, help="7 类权重（不给就不跑 YOLO）")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--out", default=None)
    ap.add_argument("--run-dir", default=None)
    ap.add_argument("--out-report", default=None)
    args = ap.parse_args()

    if args.run_dir:
        rep = build_report(args.run_dir, args.weights, args.conf, args.imgsz)
        base = args.out_report or os.path.join(args.run_dir, "report")
        json.dump(rep, open(base + ".json", "w"), ensure_ascii=False, indent=1)
        open(base + ".md", "w").write(_md(rep))
        print(_md(rep))
        print("报告 → %s.json / %s.md" % (base, base))
        return 0

    if not args.frame:
        ap.error("要么 --frame，要么 --run-dir")
    slots = []
    for part in (args.slots or "").split(";"):
        if part.strip():
            x, y = [float(v) for v in part.split(",")]
            # 槽位习惯用米写（0.45）；>10 的数字当成毫米（450）
            slots.append((x * 1000 if abs(x) < 10 else x, y * 1000 if abs(y) < 10 else y))
    res = {"placements": measure_placements(args.frame, slots) if slots else None,
           "classes": classify_blocks(args.frame, args.weights, args.conf, args.imgsz)}
    if res["classes"].get("warning"):
        print(res["classes"]["warning"])
    for b in res["classes"]["blocks"]:
        print("#%d (%7.1f,%7.1f) 色 %-11s 票 %.0f%%  YOLO %s%s"
              % (b["idx"], b["cx"], b["cy"], b["class"], b["class_share"] * 100,
                 b["yolo"] or "—",
                 "" if b["yolo"] is None else " %.2f" % b["yolo_conf"]))
    if res["placements"]:
        for s in res["placements"]["slots"]:
            print("槽 %d (%s) → %s  Δ=(%s, %s) %s mm"
                  % (s["slot_idx"], s["slot_xy_mm"],
                     "空" if not s["block"] else "块#%d" % s["block"]["idx"],
                     s["dx_mm"], s["dy_mm"], s["dist_mm"]))
        if res["placements"]["unclaimed"]:
            print("⚠️ 不在任何槽位附近的块：%s"
                  % [(b["idx"], round(b["cx"], 1), round(b["cy"], 1))
                     for b in res["placements"]["unclaimed"]])
    if args.out:
        json.dump(res, open(args.out, "w"), ensure_ascii=False, indent=1)
        print("→ %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
