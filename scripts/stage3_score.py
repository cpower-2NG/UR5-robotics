#!/usr/bin/env python3
"""阶段 3 出口判分器：逐层判 z 增量 / 相对对中 / 相对 yaw / 稳定性。

输入 = 一轮的 ``observations.json``（``stage3_round`` 产出）::

    {"kind": "stage3-round", "round": 3, "group": "open", "n_layers": 3,
     "block_h_mm": 29.5, "grid_center_base_mm": [422.65, -114.86],
     "layers": [{"layer": 1, "block_idx": 4, "exec_rc": 0,
                 "planned": {"predicted_center_base_mm": [...],
                             "yaw_place_deg": 0.0, "predicted_top_mm": 32.5},
                 "measured": {  # stage3_measure.measure_layer 的产物
                     "merged": {"found": true, "center_base_mm": [...],
                                "top_mm": 33.2, "yaw_deg": 1.2,
                                "yaw_trusted": true},
                     "consistency": {"ok": true}}}, ...],
     "terminal": {"wait_s": 10, "measured": {...}},
     "human_review": {"approved": true, "note": "...", "layers": {"1": "ok"}}}

判据（契约 §4.1，用户 2026-10-31 定）
────────────────────────────────────
· **z 增量**：``top_k − top_{k−1}`` = 29.5 ± 2 mm，**从第 2 层起算**；
  第 1 层只作基准（``status = "baseline"``），它的绝对顶高只报告；
· **相对对中**：本层中心 − 下层中心 ≤ 5 mm；
· **相对 yaw**：``fold90`` 后 ≤ 5°；
· **稳定性**：终态复测（最上层静置 10 s）vs 该层放好瞬间：Δxy ≤ 2 mm、
  Δyaw ≤ 2°、Δz ≤ 2 mm；
· **绝对格心偏差**一并报告，**不**单独判红（塔成立与否看相对下层）；
· 多视点一致性不过 / yaw 不可信 ⇒ ``person_review``，不进机判；
· **所有 yaw 判据走彩色掩码**（契约 §4.2.5，见 `yaw_metric`）：
  深度 ``yaw_rect`` 只作报告，彩色 yaw 缺失/不可信 ⇒ ``person_review``，
  **不拿深度顶替**；
· 人眼复核没填（``human_review`` 缺失或未批准）⇒ 不得判 ``ok``。

退出码：0 = 逐层全过；1 = 有不过 / 待人评；2 = 数据缺失或坏。
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

import stage2_score as s2s                                         # noqa: E402
import stage3_measure as m3                                        # noqa: E402


def diff_mod90(a, b):
    """``a − b`` 折到 (−45°, +45°]（方块朝向按 90° 周期）。"""
    d = (float(a) - float(b)) % 90.0
    return d - 90.0 if d > 45.0 else d


DEFAULT_THRESHOLDS = {
    "z_mm": 2.0,
    "xy_rel_mm": 5.0,
    "yaw_rel_deg": 5.0,
    "stability_xy_mm": 2.0,
    "stability_yaw_deg": 2.0,
    "stability_z_mm": 2.0,
    # 与 stage3_measure 的一致性门同值；判分只复核测量产物的结论。
    "pose_xy_mm": 10.0,
    "pose_yaw_deg": 6.0,
    "pose_z_mm": 10.0,
}

#: 终态顶高比最上层放好瞬间低这么多 ⇒ 判「塔塌 / 掉落」，不是单纯缺数据。
COLLAPSE_DROP_MM = 10.0

STATUS_BASELINE = "baseline"
STATUS_OK = "ok"
STATUS_FAIL = "fail"
STATUS_REVIEW = "person_review"
STATUS_MISSING = "missing"


# ---------------------------------------------------------------- 小工具
def _merged(layer):
    """层记录 → 测量融合值（缺数据时给空 dict）。"""
    m = (layer.get("measured") or {}).get("merged")
    return m or {}


def _consistency(layer):
    return (layer.get("measured") or {}).get("consistency") or {}


def _planned(layer):
    return layer.get("planned") or {}


#: 判据用的 yaw 来源规则**只有一份**，在 ``stage3_measure.metric_yaw`` ——
#: 闭环控制（``stage3_round``）调的是同一个函数，两处各写一份会漂。
yaw_metric = m3.metric_yaw


def _xy(rec):
    xy = rec.get("center_base_mm")
    if not xy or len(xy) < 2:
        return None
    return [float(xy[0]), float(xy[1])]


def _dist(a, b):
    if a is None or b is None:
        return None
    return round(math.hypot(a[0] - b[0], a[1] - b[1]), 2)


def _num(v):
    return None if v is None else round(float(v), 3)


# ---------------------------------------------------------------- 逐层判分
def score_layer(rec, prev, thresholds, grid_center=None,
                block_h_mm=29.5, grid_yaw_base_deg=0.0):
    """一层 → 判分行。``prev`` 是下一层（物理更低一层）的记录，第 1 层为 None。"""
    k = int(rec.get("layer", 0))
    m = _merged(rec)
    cons = _consistency(rec)
    row = {"layer": k, "block_idx": rec.get("block_idx"),
           "class": rec.get("class"),
           "measured": {"center_base_mm": _xy(m) if m.get("found") else None,
                        "top_mm": m.get("top_mm") if m.get("found") else None,
                        # 深度 yaw —— 只报告，不进任何判据（契约 §4.2.5）。
                        "yaw_deg": m.get("yaw_deg") if m.get("found") else None,
                        "yaw_trusted": m.get("yaw_trusted") if m.get("found")
                        else None,
                        # 彩色 yaw —— 判据用的就是这一路。
                        "yaw_color_deg": (m.get("yaw_color_deg")
                                          if m.get("found") else None),
                        "yaw_color_trusted": (m.get("yaw_color_trusted")
                                              if m.get("found") else None),
                        "yaw_source": None},
           "consistency": cons,
           "z_increment_mm": None, "z_err_mm": None,
           "xy_rel_mm": None, "yaw_rel_deg": None,
           "xy_abs_mm": None, "yaw_abs_deg": None,
           "release": None,
           "reasons": []}

    def bad(status, why):
        row["status"] = status
        row["reasons"].append(why)
        return row

    # 执行失败：计划没跑完，这一层没有被放上去 —— 不是「没量到」。
    if rec.get("exec_rc") not in (0, None):
        return bad(STATUS_FAIL, "exec_rc=%s" % rec.get("exec_rc"))
    if not m.get("found"):
        return bad(STATUS_MISSING, "measured_missing")

    center = _xy(m)
    top = m.get("top_mm")
    yaw, yaw_src = yaw_metric(m)
    row["measured"]["yaw_source"] = yaw_src
    # 多视点一致性 / yaw 可信度：数据在、但不可信 ⇒ 人评。
    if cons.get("ok") is not True:
        why = "pose_consistency=%s" % (cons.get("reason") or cons.get("ok"))
        if cons.get("ok") is None:
            why = "single_pose_only"
        return bad(STATUS_REVIEW, why)
    # 彩色 yaw 判不了就转人评：既不用深度顶替，也不把缺数据当合格。
    if yaw is None:
        return bad(STATUS_REVIEW, yaw_src)

    # 绝对格心偏差：只报告。yaw 的绝对目标 = 格线方向（grid_yaw_base_deg）。
    if grid_center is not None:
        row["xy_abs_mm"] = _dist(center, grid_center)
    row["yaw_abs_deg"] = round(abs(diff_mod90(yaw, grid_yaw_base_deg)), 2)

    # 释放扰动（预测 vs 实测）：只报告，不参与达标。
    pred = _planned(rec).get("predicted_center_base_mm")
    pred_yaw = _planned(rec).get("yaw_place_deg")
    pred_top = _planned(rec).get("predicted_top_mm")
    row["release"] = {
        "dxy_mm": _dist(center, pred) if pred else None,
        "dyaw_deg": (round(abs(diff_mod90(yaw, pred_yaw)), 2)
                     if pred_yaw is not None else None),
        "dz_mm": (round(float(top) - float(pred_top), 2)
                  if pred_top is not None else None),
    }

    # 第 1 层：只作基准 —— 没有「下层」可比，z 增量从第 2 层起算。
    if k <= 1 or prev is None:
        row["status"] = STATUS_BASELINE
        return row

    pm = _merged(prev)
    if not pm.get("found"):
        return bad(STATUS_MISSING, "previous_layer_missing")
    pc, ptop = _xy(pm), pm.get("top_mm")
    # 相对 yaw 也走彩色；下层判不了同样不放行（两次噪声相加，更不能混源）。
    pyaw, pyaw_src = yaw_metric(pm)
    if pyaw is None:
        return bad(STATUS_REVIEW, "previous_%s" % pyaw_src)

    z_inc = round(float(top) - float(ptop), 2)
    row["z_increment_mm"] = z_inc
    row["z_err_mm"] = round(z_inc - float(block_h_mm), 2)
    row["xy_rel_mm"] = _dist(center, pc)
    row["yaw_rel_deg"] = (round(abs(diff_mod90(yaw, pyaw)), 2)
                          if pyaw is not None else None)

    checks = [
        ("z", row["z_err_mm"], thresholds["z_mm"]),
        ("xy_rel", row["xy_rel_mm"], thresholds["xy_rel_mm"]),
        ("yaw_rel", row["yaw_rel_deg"], thresholds["yaw_rel_deg"]),
    ]
    fails = ["%s=%.2f>%.2f" % (name, abs(v), tol)
             for name, v, tol in checks if v is not None and abs(v) > tol]
    missing = [name for name, v, _ in checks if v is None]
    if missing:
        return bad(STATUS_MISSING, "missing:" + ",".join(missing))
    if fails:
        row["status"] = STATUS_FAIL
        row["reasons"].extend(fails)
    else:
        row["status"] = STATUS_OK
    return row


def score_stability(obs, layers, thresholds):
    """终态 vs 最上层放好瞬间：Δxy / Δyaw / Δz + 塔是否塌。"""
    term = obs.get("terminal") or {}
    tm = (term.get("measured") or {}).get("merged") or {}
    out = {"wait_s": term.get("wait_s"),
           "status": STATUS_MISSING, "dxy_mm": None, "dyaw_deg": None,
           "dz_mm": None, "reasons": []}
    if not tm.get("found"):
        out["reasons"].append("terminal_measured_missing")
        return out
    if not layers:
        out["reasons"].append("no_layers")
        return out
    last = layers[-1]
    lm = _merged(last)
    if not lm.get("found"):
        out["reasons"].append("top_layer_missing")
        return out
    center, top = _xy(tm), tm.get("top_mm")
    lc, lt = _xy(lm), lm.get("top_mm")
    # Δyaw 也走彩色（契约 §4.2.5）：2° 的稳定性门比深度法的自带散差还小。
    yaw, yaw_src = yaw_metric(tm)
    ly, ly_src = yaw_metric(lm)
    out["yaw_source"] = {"terminal": yaw_src, "layer": ly_src}
    out["dxy_mm"] = _dist(center, lc)
    out["dz_mm"] = round(float(top) - float(lt), 2)
    out["dyaw_deg"] = (round(abs(diff_mod90(yaw, ly)), 2)
                       if yaw is not None and ly is not None else None)
    if out["dz_mm"] <= -COLLAPSE_DROP_MM:
        out["status"] = STATUS_FAIL
        out["reasons"].append("collapse_drop=%.1fmm" % out["dz_mm"])
        return out
    if out["dyaw_deg"] is None:
        # 彩色 yaw 判不了 ⇒ 人评（契约 §4.2.5），不拿深度顶替。
        out["status"] = STATUS_REVIEW
        out["reasons"].append("dyaw_unavailable:%s"
                              % (yaw_src if yaw is None else ly_src))
        return out
    checks = [("dxy", out["dxy_mm"], thresholds["stability_xy_mm"]),
              ("dyaw", out["dyaw_deg"], thresholds["stability_yaw_deg"]),
              ("dz", out["dz_mm"], thresholds["stability_z_mm"])]
    missing = [n for n, v, _ in checks if v is None]
    fails = ["%s=%.2f>%.2f" % (n, abs(v), tol)
             for n, v, tol in checks if v is not None and abs(v) > tol]
    if missing:
        out["status"] = STATUS_MISSING
        out["reasons"].extend("missing:" + n for n in missing)
    elif fails:
        out["status"] = STATUS_FAIL
        out["reasons"].extend(fails)
    else:
        out["status"] = STATUS_OK
    return out


def apply_human_review(rows, review):
    """人眼复核：未批准 / 未填的层不得判 ``ok`` / ``baseline``。"""
    if review is None:
        for r in rows:
            if r["status"] in (STATUS_OK, STATUS_BASELINE):
                r["status"] = STATUS_REVIEW
                r["reasons"].append("human_review_missing")
        return rows
    if review.get("approved") is not True:
        for r in rows:
            if r["status"] in (STATUS_OK, STATUS_BASELINE):
                r["status"] = STATUS_REVIEW
                r["reasons"].append("human_review_not_approved")
        return rows
    per = review.get("layers") or {}
    for r in rows:
        key = str(r["layer"])
        if key not in per:
            continue
        verdict = per[key]
        if verdict is True or verdict == "ok":
            continue
        if r["status"] in (STATUS_OK, STATUS_BASELINE):
            r["status"] = STATUS_REVIEW
            r["reasons"].append("human_review_layer:%s" % verdict)
    return rows


def score_round(obs, thresholds=None):
    """整轮 → 判分结果（不写文件）。"""
    th = dict(DEFAULT_THRESHOLDS)
    th.update(thresholds or {})
    block_h = float(obs.get("block_h_mm", 29.5))
    grid_center = obs.get("grid_center_base_mm")
    if not grid_center:
        # 兼容把整份 grid.json 塞进来的写法（stage2 的 observations 先例）。
        grid_center = (obs.get("grid") or {}).get("grid_center_base_mm")
    if grid_center and len(grid_center) >= 2:
        grid_center = [float(grid_center[0]), float(grid_center[1])]
    else:
        grid_center = None
    raw_layers = obs.get("layers") or []
    n_layers = int(obs.get("n_layers") or len(raw_layers))
    rows = []
    for rec in raw_layers:
        k = int(rec.get("layer", len(rows) + 1))
        prev = None
        for r in raw_layers:
            if int(r.get("layer", 0)) == k - 1:
                prev = r
                break
        rows.append(score_layer(rec, prev, th, grid_center=grid_center,
                                block_h_mm=block_h,
                                grid_yaw_base_deg=float(
                                    obs.get("grid_yaw_base_deg") or 0.0)))
    rows = apply_human_review(rows, obs.get("human_review"))
    stability = score_stability(obs, raw_layers, th)

    term_merged = ((((obs.get("terminal") or {}).get("measured")) or {}
                    ).get("merged")) or {}
    verdict, reason = "ok", ""
    if len(raw_layers) < n_layers:
        verdict, reason = "missing_data", "layers=%d<%d" % (len(raw_layers),
                                                           n_layers)
    elif any(r["status"] == STATUS_MISSING for r in rows):
        verdict, reason = "missing_data", "layer_missing"
    elif not term_merged.get("found"):
        # 塔已经完整叠起来却量不到终态：更像塌了 / 掉出塔窗，不是单纯缺数据。
        verdict, reason = "fail", "terminal_missing_after_complete_tower"
    elif any(r["status"] == STATUS_FAIL for r in rows):
        verdict, reason = "fail", "layer_threshold"
    elif stability["status"] == STATUS_FAIL:
        verdict, reason = "fail", "stability"
    elif any(r["status"] == STATUS_REVIEW for r in rows):
        verdict, reason = "person_review", "layer_review"
    elif stability["status"] == STATUS_MISSING:
        verdict, reason = "missing_data", "stability_missing"
    elif stability["status"] != STATUS_OK:
        verdict, reason = "person_review", "stability_review"
    return {"version": 1, "kind": "stage3-score",
            "round": obs.get("round"), "group": obs.get("group"),
            "n_layers": n_layers, "block_h_mm": block_h,
            "grid_center_base_mm": grid_center,
            "thresholds": th, "layers": rows, "stability": stability,
            "human_review": obs.get("human_review"),
            "verdict": verdict, "reason": reason}


def exit_code(res):
    return {"ok": 0, "fail": 1, "person_review": 1,
            "missing_data": 2}.get(res.get("verdict"), 2)


# ---------------------------------------------------------------- 报告
_COLOR = {STATUS_OK: (0, 220, 0), STATUS_BASELINE: (200, 200, 200),
          STATUS_FAIL: (0, 0, 230), STATUS_REVIEW: (0, 200, 255),
          STATUS_MISSING: (255, 0, 255)}


def _bar_panel(img, x0, y0, w, h, values, target, tol, title, unit,
               max_abs):
    """一张绝对值柱状图 + 目标线 + ±tol 带。"""
    import cv2
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (70, 70, 70), 1)
    img = s2s._draw_text(img, (x0 + 8, y0 + 6), title, (225, 225, 225), 15)
    plot_y0, plot_y1 = y0 + 34, y0 + h - 24
    mid = (plot_y0 + plot_y1) // 2
    # ±tol 带
    band = int((plot_y1 - plot_y0) / 2.0 * (tol / max_abs)) if max_abs > 0 else 0
    cv2.rectangle(img, (x0 + 6, mid - band), (x0 + w - 6, mid + band),
                  (45, 70, 45), -1)
    cv2.line(img, (x0 + 6, mid), (x0 + w - 6, mid), (120, 120, 120), 1)
    if target != 0.0:
        ty = mid - int((plot_y1 - plot_y0) / 2.0 * (target / max_abs))
        cv2.line(img, (x0 + 6, ty), (x0 + w - 6, ty), (60, 160, 255), 1)
    n = max(1, len(values))
    slot = (w - 20) / n
    for i, v in enumerate(values):
        bx = int(x0 + 12 + slot * i + slot * 0.15)
        bw = max(4, int(slot * 0.7))
        if v is None:
            img = s2s._draw_text(img, (bx, mid - 10), "?", (255, 0, 255), 13)
            continue
        bh = int((plot_y1 - plot_y0) / 2.0 * (abs(v) / max_abs)) if max_abs else 0
        top = mid - bh if v >= 0 else mid
        col = (0, 220, 0) if abs(v) <= tol else (0, 0, 230)
        cv2.rectangle(img, (bx, top), (bx + bw, top + max(bh, 1)), col, -1)
        img = s2s._draw_text(img, (bx - 2, y0 + h - 20),
                             "L%d %.1f" % (i + 1, v), (200, 200, 200), 12)
    img = s2s._draw_text(img, (x0 + w - 70, y0 + 6), unit,
                         (160, 160, 160), 12)
    return img


def draw_score_png(res, path, size=(1200, 760)):
    """判分图：逐层数值 + z 增量柱 + 相对 xy/yaw 柱。"""
    import cv2
    W, H = size
    img = np.full((H, W, 3), 26, np.uint8)
    img = s2s._draw_text(
        img, (16, 10),
        "阶段 3 · round %s · group %s · verdict=%s · %s"
        % (res.get("round"), res.get("group"), res.get("verdict"),
           res.get("reason") or "-"), (235, 235, 235), 20)
    img = s2s._draw_text(
        img, (16, 40),
        "块高 %.1f mm · z 增量 <=%.1f mm · 相对 xy <=%.1f mm · "
        "相对 yaw <=%.1f° · 稳定性 %.1f mm / %.1f° / %.1f mm"
        % (res["block_h_mm"], res["thresholds"]["z_mm"],
           res["thresholds"]["xy_rel_mm"], res["thresholds"]["yaw_rel_deg"],
           res["thresholds"]["stability_xy_mm"],
           res["thresholds"]["stability_yaw_deg"],
           res["thresholds"]["stability_z_mm"]),
        (180, 180, 180), 14)

    # 表头
    y = 78
    heads = ["层", "状态", "顶高mm", "z增量mm", "z误差mm", "xy相对mm",
             "yaw相对°", "xy绝对mm", "yaw绝对°"]
    xs = [20, 70, 220, 310, 410, 510, 610, 710, 810]
    for x, htxt in zip(xs, heads):
        img = s2s._draw_text(img, (x, y), htxt, (200, 200, 200), 14)
    for r in res["layers"]:
        y += 26
        vals = [str(r["layer"]), r["status"],
                "-" if r["measured"]["top_mm"] is None
                else "%.2f" % r["measured"]["top_mm"],
                "-" if r["z_increment_mm"] is None
                else "%.2f" % r["z_increment_mm"],
                "-" if r["z_err_mm"] is None else "%+.2f" % r["z_err_mm"],
                "-" if r["xy_rel_mm"] is None else "%.2f" % r["xy_rel_mm"],
                "-" if r["yaw_rel_deg"] is None else "%.2f" % r["yaw_rel_deg"],
                "-" if r["xy_abs_mm"] is None else "%.2f" % r["xy_abs_mm"],
                "-" if r["yaw_abs_deg"] is None
                else "%.2f" % r["yaw_abs_deg"]]
        col = _COLOR.get(r["status"], (200, 200, 200))
        for i, (x, v) in enumerate(zip(xs, vals)):
            img = s2s._draw_text(img, (x, y), v,
                                 col if i == 1 else (215, 215, 215), 13)

    st = res["stability"]
    y += 34
    col = _COLOR.get(st["status"], (200, 200, 200))
    img = s2s._draw_text(
        img, (20, y),
        "稳定性（静置 %ss）：Δxy=%s mm  Δyaw=%s°  Δz=%s mm  [%s]  %s"
        % (st.get("wait_s"), st.get("dxy_mm"), st.get("dyaw_deg"),
           st.get("dz_mm"), st["status"], ",".join(st.get("reasons") or [])),
        col, 15)

    # 柱状图
    zvals = [r["z_err_mm"] for r in res["layers"] if r["layer"] >= 2]
    xyvals = [r["xy_rel_mm"] for r in res["layers"] if r["layer"] >= 2]
    ywvals = [r["yaw_rel_deg"] for r in res["layers"] if r["layer"] >= 2]
    pw, ph = 350, 220
    top = y + 40
    img = _bar_panel(img, 20, top, pw, ph, zvals, 0.0,
                     res["thresholds"]["z_mm"], "z 增量误差",
                     "mm", max(4.0, max([abs(v) for v in zvals if v is not None]
                                        or [0.0])))
    img = _bar_panel(img, 390, top, pw, ph, xyvals, 0.0,
                     res["thresholds"]["xy_rel_mm"], "相对对中",
                     "mm", max(8.0, max([abs(v) for v in xyvals
                                         if v is not None] or [0.0])))
    img = _bar_panel(img, 760, top, pw, ph, ywvals, 0.0,
                     res["thresholds"]["yaw_rel_deg"], "相对 yaw",
                     "deg", max(8.0, max([abs(v) for v in ywvals
                                          if v is not None] or [0.0])))
    cv2.imwrite(path, img)
    return path


def _md(res):
    th = res["thresholds"]
    lines = ["# 阶段 3 判分 · round %s · group %s"
             % (res.get("round"), res.get("group")), "",
             "- verdict: **%s**（%s）" % (res["verdict"], res.get("reason") or "-"),
             "- 块高 %.1f mm；阈值 z ≤%.1f mm、相对 xy ≤%.1f mm、相对 yaw ≤%.1f°"
             % (res["block_h_mm"], th["z_mm"], th["xy_rel_mm"],
                th["yaw_rel_deg"]),
             "- 稳定性阈值：Δxy ≤%.1f mm、Δyaw ≤%.1f°、Δz ≤%.1f mm"
             % (th["stability_xy_mm"], th["stability_yaw_deg"],
                th["stability_z_mm"]), "",
             "- yaw 判据一律走**彩色掩码**（契约 §4.2.5）；深度 `yaw_deg` 只落盘报告，"
             "不进任何阈值。每层来源：%s"
             % "、".join("%s=%s" % (r["layer"],
                                   r["measured"].get("yaw_source") or "-")
                        for r in res["layers"]), "",
             "| 层 | 状态 | 顶高 mm | z 增量 mm | z 误差 mm | xy 相对 mm | "
             "yaw 相对 °（彩色） | xy 绝对 mm | yaw 绝对 °（彩色） | 释放 dxy/dyaw/dz |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in res["layers"]:
        rel = r.get("release") or {}
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |"
                     % (r["layer"], r["status"],
                        r["measured"]["top_mm"],
                        r["z_increment_mm"], r["z_err_mm"], r["xy_rel_mm"],
                        r["yaw_rel_deg"], r["xy_abs_mm"], r["yaw_abs_deg"],
                        "%s/%s/%s" % (rel.get("dxy_mm"), rel.get("dyaw_deg"),
                                      rel.get("dz_mm"))))
    st = res["stability"]
    ysrc = st.get("yaw_source") or {}
    lines += ["", "稳定性（终态 vs 最上层放好瞬间，静置 %s s）："
              "Δxy=%s mm、Δyaw=%s°（彩色：终态 %s / 层 %s）、Δz=%s mm → **%s**"
              % (st.get("wait_s"), st.get("dxy_mm"), st.get("dyaw_deg"),
                 ysrc.get("terminal"), ysrc.get("layer"),
                 st.get("dz_mm"), st["status"])]
    if st.get("reasons"):
        lines.append("- " + "；".join(st["reasons"]))
    hr = res.get("human_review")
    lines += ["", "人眼复核：%s"
              % ("已批准" if hr and hr.get("approved") else "**未填 / 未批准**")]
    if hr and hr.get("note"):
        lines.append("- 备注：%s" % hr["note"])
    return "\n".join(lines) + "\n"


def load_observations(path):
    obs = json.load(open(path))
    if not isinstance(obs.get("layers"), list) or not obs["layers"]:
        raise ValueError("%s 里没有 layers" % path)
    return obs


def write_outputs(res, prefix):
    os.makedirs(os.path.dirname(os.path.abspath(prefix)) or ".", exist_ok=True)
    json_path = prefix + ".json"
    png_path = prefix + ".png"
    md_path = prefix + ".md"
    json.dump(res, open(json_path, "w"), ensure_ascii=False, indent=1)
    draw_score_png(res, png_path)
    open(md_path, "w").write(_md(res))
    return {"json": json_path, "png": png_path, "md": md_path}


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 3 出口判分器")
    ap.add_argument("--observations", required=True)
    ap.add_argument("--out-report", default=None,
                    help="输出前缀（生成 .json/.png/.md）；缺省只打印")
    ap.add_argument("--threshold-z-mm", type=float,
                    default=DEFAULT_THRESHOLDS["z_mm"])
    ap.add_argument("--threshold-xy-mm", type=float,
                    default=DEFAULT_THRESHOLDS["xy_rel_mm"])
    ap.add_argument("--threshold-yaw-deg", type=float,
                    default=DEFAULT_THRESHOLDS["yaw_rel_deg"])
    args = ap.parse_args(argv)
    obs = load_observations(args.observations)
    res = score_round(obs, {"z_mm": args.threshold_z_mm,
                            "xy_rel_mm": args.threshold_xy_mm,
                            "yaw_rel_deg": args.threshold_yaw_deg})
    if args.out_report:
        paths = write_outputs(res, args.out_report)
        print("判分 → %s" % paths["json"])
    print(_md(res))
    return exit_code(res)


if __name__ == "__main__":
    sys.exit(main())
