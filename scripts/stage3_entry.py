#!/usr/bin/env python3
"""阶段 3 入口四实验的驱动（真机；每次 ``--go`` 都要**当次授权**）。

四项实验（契约 §2.2）
────────────────────
1. ``--mode zcal`` —— 塔位单块重复放置 ≥5 次，两个高度跟踪视点各 ≥3 帧；
   偏差 ``z_bias = 顶高 − 垫面 − 29.5``，取中位落盘 ``zcal.json``（``by_site``），
   每次残差 |≤2 mm| 才算过。顺带用拾取帧记一份拾取侧偏差。
2. ``--mode release`` —— 开口 {50,70,95} mm × 停留 {0,0.5} s 共 6 组 ×5 次，
   每组量终态中心/yaw/顶高，算「实测 − 预测」的重复性（极差、σ）；
   最优组再补 ``--confirm`` 次。**数值闸门不在这里编**，数据出表由用户拍板。
3. ``--mode occlusion`` —— 人手先搭好 1/3/5 层塔，机器人夹着块停在
   ``place_approach`` 抓帧：下层塔顶还能不能被量出来。结果决定 eye-in-hand
   对照组做不做。**抓完这一帧就把这块放上去**（塔变成 h+1 层，由人重搭）。
4. ``--mode tolerance`` —— 人手把上块相对下块偏 0/3/5/8 mm 放上去，机器人
   **只动臂不碰夹爪**：量下层基准 → 量上层 → 静置 10 s 再量。

真机纪律（每次会话重来）
────────────────────────
· ``--go`` 才动臂；执行器自己在板上核对 PLAYING / JTC active / robot_mode 7；
· 夹爪寄存器写入（初始化/开合）是**另一件事**的当次授权：急停就绪、
  两指整个行程内无手无遮挡；
· 任何一次执行判红 / 测量缺失 ⇒ 停在这一步，**不自动续跑**，交给人判读；
· 人搬块/搭塔的每一步都压成一行 ``>>> 需要人动手``，由 AI 原样转达；
· 不 push。

不带 ``--go`` 只打印协议（不抓帧、不动臂）—— 离线计划预演用
``stage3_round.py --plan-only``。
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                   # noqa: E402
import precision_plan as pp                                        # noqa: E402
import precision_round as pr                                       # noqa: E402
import stage2_round as s2                                          # noqa: E402
import stage3_measure as m3                                        # noqa: E402
import stage3_round as s3r                                         # noqa: E402
import urkin                                                      # noqa: E402

MODES = ("zcal", "release", "occlusion", "tolerance")

#: 入口线：校正后每次高度对 29.5 mm 的偏差绝对值上限。
ZCAL_TOL_MM = 2.0
#: 遮挡实验里「塔顶簇」的判据：top 落在期望顶高 ±这个值内。
OCCL_TOL_MM = 6.0
#: 塔位「空」的判据窗：比测量窗（50 mm）更紧。相邻格心最近 60 mm，
#: 阶段 2 的放置容差 10 mm ⇒ 邻块中心最远能到 ~50 mm，收紧到 40 mm 不误伤。
TOWER_CLEAR_RADIUS_MM = 40.0
#: 人手搭的塔顶 / 放的下块顶面，与理论值差超过它 ⇒ 层数不对或塔位没清空。
#: 半块高 14.75 mm，取 12 mm：漏一层（29.5）必被拦，读数偏差 +1~2 mm 不拦。
TOWER_HEIGHT_TOL_MM = 12.0

#: **机器人自己回位**（拆塔）时，回位块离拾取区已有块中心的最小距离 ——
#: 与场景里块间距同一口径（`precision_plan.MIN_BLOCK_GAP` = 60 mm）。
PICK_RETURN_GAP_MM = 60.0
#: 回位块中心离拾取区边界的最小余量（块心，不是块边；`in_zone` 也按块心算）。
PICK_RETURN_MARGIN_MM = 25.0
#: 塔上那块在计划里的伪块序号 —— 拾取区的真 idx 从 0 起，−1 不会撞。
TOWER_BLOCK_IDX = -1


class EntryStop(Exception):
    """某一步判红：停住，不续跑。``rc`` 直接当退出码。"""

    def __init__(self, why, rc=2):
        super().__init__(why)
        self.why = why
        self.rc = rc


# ---------------------------------------------------------------- 纯逻辑（可测）
def group_stats(vals, digits=2):
    """一组数的重复性摘要；空组返回 ``{"n": 0}``。"""
    vals = [float(v) for v in vals if v is not None]
    if not vals:
        return {"n": 0}
    arr = np.asarray(vals, dtype=float)
    return {"n": int(arr.size),
            "mean": round(float(arr.mean()), digits),
            "median": round(float(np.median(arr)), digits),
            "min": round(float(arr.min()), digits),
            "max": round(float(arr.max()), digits),
            "range": round(float(arr.max() - arr.min()), digits),
            "std": round(float(arr.std(ddof=1)) if arr.size > 1 else 0.0,
                         digits)}


def zcal_doc(rows, pad_mm, block_h_mm=m3.BLOCK_TRUE_H_MM,
             tol_mm=ZCAL_TOL_MM, meta=None):
    """入口实验 1 的落盘格式 —— **``load_zcal`` 直接吃的就是这份**。

    ``rows``：每次放置一条 ``{"top_mm": …}``；``bias = top − 垫面 − 真高``。
    拾取侧偏差（若有）取 ``pick_bias_mm`` 的中位，作为 ``by_site.pick``。
    """
    pad = float(pad_mm)
    h = float(block_h_mm)
    biases, picks = [], []
    out_rows = []
    for i, r in enumerate(rows, 1):
        top = float(r["top_mm"])
        b = round(top - pad - h, 3)
        biases.append(b)
        if r.get("pick_bias_mm") is not None:
            picks.append(float(r["pick_bias_mm"]))
        out_rows.append(dict(r, rep=i, bias_mm=b))
    tower = float(np.median(biases)) if biases else 0.0
    residuals = [abs(b - tower) for b in biases]
    pick = float(np.median(picks)) if picks else None
    doc = {
        "version": 1, "model": "constant",
        "block_h_mm": h, "pad_z_mm": round(pad, 3),
        "by_site": {"tower": round(tower, 3),
                    "pick": None if pick is None else round(pick, 3)},
        "n": len(biases),
        "residual_mm": {"median": round(float(np.median(residuals)), 3)
                        if residuals else None,
                        "max": round(max(residuals), 3) if residuals else None},
        "tol_mm": float(tol_mm),
        # 判据是「**每一次**校正后的高度对真高都 ≤2 mm」，不是只看中位。
        "pass": bool(residuals) and max(residuals) <= float(tol_mm),
        "rows": out_rows,
    }
    if meta:
        doc["meta"] = meta
    return doc


def release_summary(groups):
    """开口×停留 → 每组的重复性；``best`` 按 (dxy 极差, dyaw 极差) 排序。

    ``groups = {组名: [{"dxy_mm","dyaw_deg","dz_mm"}, …]}``。
    """
    per = {}
    for name, rows in groups.items():
        per[name] = {
            "n": len(rows),
            "dxy_mm": group_stats([r["dxy_mm"] for r in rows]),
            "dyaw_deg": group_stats([r["dyaw_deg"] for r in rows]),
            "dz_mm": group_stats([r["dz_mm"] for r in rows]),
        }

    def key(name):
        s = per[name]
        return (s["dxy_mm"].get("range", float("inf")),
                s["dyaw_deg"].get("range", float("inf")))

    usable = [n for n, s in per.items() if s["n"]]
    best = min(usable, key=key) if usable else None
    return {"groups": per, "best": best,
            "order": sorted(usable, key=key)}


def occlusion_report(blocks, expected_top_mm, window_xy_mm,
                     radius_mm=m3.TOWER_RADIUS_MM, held_top_mm=None,
                     tol_mm=OCCL_TOL_MM, min_h_mm=m3.MIN_H_MM):
    """夹着块那一帧 → 「塔顶还在不在视野里」。

    只看塔窗内、非 low、高度 ≥``min_h_mm`` 的簇。**手里那块会挡在塔顶上方**
    （其顶面约比塔顶高 74.5 mm），所以按期望顶高分类，不拿「最高簇」当塔顶。
    """
    wx, wy = float(window_xy_mm[0]), float(window_xy_mm[1])
    inside = []
    for b in blocks:
        if b.get("low") or b.get("h", 0.0) < min_h_mm:
            continue
        d = math.hypot(b["cx"] - wx, b["cy"] - wy)
        if d <= float(radius_mm):
            inside.append({"idx": b["idx"], "cx": round(float(b["cx"]), 2),
                           "cy": round(float(b["cy"]), 2),
                           "top": round(float(b["top"]), 2),
                           "yaw_deg": (None if b.get("yaw_rect") is None
                                       else round(float(b["yaw_rect"]), 3)),
                           "yaw_trusted": bool(b.get("yaw_trusted")),
                           "dist_mm": round(d, 2)})
    tower = [b for b in inside if abs(b["top"] - float(expected_top_mm))
             <= float(tol_mm)]
    held = []
    if held_top_mm is not None:
        held = [b for b in inside if abs(b["top"] - float(held_top_mm))
                <= float(tol_mm)]
    return {"window_xy_mm": [wx, wy], "radius_mm": float(radius_mm),
            "expected_top_mm": round(float(expected_top_mm), 2),
            "held_top_mm": (None if held_top_mm is None
                            else round(float(held_top_mm), 2)),
            "n_window": len(inside), "clusters": inside,
            "tower_clusters": tower, "held_clusters": held,
            # 「量得到」只表示视野里有一簇落在塔顶高度上；中心/yaw 能不能用
            # 还要人看 —— 这就是本实验要回答的。
            "tower_top_visible": bool(tower)}


def tol_offset_mm(lower_center_mm, upper_center_mm):
    """人手偏置实验：实际偏移向量（上块中心 − 下块中心）与模长。"""
    dx = float(upper_center_mm[0]) - float(lower_center_mm[0])
    dy = float(upper_center_mm[1]) - float(lower_center_mm[1])
    return {"dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
            "d_mm": round(math.hypot(dx, dy), 2)}


# ---------------------------------------------------------------- 板侧动作
def _dump(path, obj):
    json.dump(obj, open(path, "w"), ensure_ascii=False, indent=1)
    return path


def ask(text):
    """把「要人动手」压成一行；有终端就等回车，没有就只提示不阻塞。"""
    line = ">>> 需要人动手：" + text
    print(line, flush=True)
    try:
        input("    完成后按回车继续 …")
    except EOFError:
        print("    （非交互终端：直接继续 —— 确认已经做完再往下一步）")


def _survey_pair(top_mm, view_xy_mm, d_mm):
    return pp.stage3_survey_tool0(float(top_mm) / 1000.0,
                                  [view_xy_mm[0] / 1000.0,
                                   view_xy_mm[1] / 1000.0],
                                  distance=float(d_mm) / 1000.0)


def measure_pair(board, chain, scene, cdir, tag, pair, grid, window_xy_mm,
                 layer, z_bias_mm, move=True, require_found=True,
                 radius_mm=None):
    """在高度跟踪位姿对上测量一次（``move=False`` 表示已经停在 pair[0]）。

    ``require_found=False`` 供**塔位预检**用：量不到块才是通过的条件。
    """
    if move:
        rc = s3r.go_move(board, chain, cdir, tag + "-pose1", pair[0], _URDF)
        if rc != 0:
            raise EntryStop("%s 走到位姿 1 失败（rc=%d）" % (tag, rc), rc)
    n = int(scene.get("terminal_frames", 3))
    dirs1 = s3r.capture_frames(board, os.path.join(cdir, "frame-%s-pose1" % tag),
                               "entry-%s-p1" % tag, n)
    rc = s3r.go_move(board, chain, cdir, tag + "-pose2", pair[1], _URDF)
    if rc != 0:
        raise EntryStop("%s 走到位姿 2 失败（rc=%d）" % (tag, rc), rc)
    dirs2 = s3r.capture_frames(board, os.path.join(cdir, "frame-%s-pose2" % tag),
                               "entry-%s-p2" % tag, n)
    kw = {} if radius_mm is None else {"radius_mm": float(radius_mm)}
    meas = m3.measure_layer([("pose1", dirs1), ("pose2", dirs2)],
                            grid=grid, window_xy_mm=window_xy_mm,
                            layer=layer, z_bias_mm=float(z_bias_mm),
                            block_h_mm=m3.BLOCK_TRUE_H_MM, **kw)
    _dump(os.path.join(cdir, "measure-%s.json" % tag), meas)
    merged = meas.get("merged") or {}
    if require_found and not merged.get("found"):
        raise EntryStop("%s 测量缺失（塔窗内没有合格块）" % tag)
    return meas


def layer_height_check(measured_mm, expected_mm, tol_mm=TOWER_HEIGHT_TOL_MM):
    """人手摆好的高度对不对。返回 ``(ok, 差多少)``；差 = 实测 − 应有。"""
    d = round(float(measured_mm) - float(expected_mm), 2)
    return abs(d) <= float(tol_mm), d


def tower_occupancy_verdict(merged, cell_idx=m3.TOWER_CELL_IDX):
    """塔位测量的融合值 → 「还有块」的一句话；空着返回 ``None``。"""
    if not (merged or {}).get("found"):
        return None
    return ("塔位（格 %d）上还有块：顶 %.1f mm、中心 %s"
            % (int(cell_idx), merged["top_mm"], merged["center_base_mm"]))


def tower_precheck(args, board, chain, scene, grid, cdir, tag="precheck"):
    """塔位必须是**空的** —— 非空立刻停（fail-closed）。

    2026-10-31 现状层记着：垫上格 0/1/2/4/5/6/7 各一块，塔位（中心格 idx4）
    上正压着阶段 2 的一块。第 1 层计划按「表面 = 垫面」算 ``place_z``，
    不清格就会直接往下压 —— 所以在任何一次放置之前先量一遍。
    """
    view_xy = m3.grid_center(grid)
    pad_mm = m3.pad_z_mm(grid)
    pair = _survey_pair(pad_mm, view_xy, args.d_mm)
    meas = measure_pair(board, chain, scene, cdir, tag, pair, grid, view_xy,
                        layer=0, z_bias_mm=0.0, require_found=False,
                        radius_mm=TOWER_CLEAR_RADIUS_MM)
    msg = tower_occupancy_verdict(meas.get("merged"), args.tower_cell_idx)
    if msg:
        raise EntryStop(msg + " —— 先清空再跑")
    return meas


def tower_clear_now(board, scene, cdir, tag, grid, window_xy_mm,
                    radius_mm=TOWER_CLEAR_RADIUS_MM,
                    cell_idx=m3.TOWER_CELL_IDX):
    """人清完塔位后**不动臂**就地核一次：还有块就停。

    只在刚做完 ``measure_pair``（臂停在 survey 位姿 2）之后调用 ——
    那时相机本来就对着塔位，不必再走一趟。
    """
    n = int(scene.get("terminal_frames", 3))
    dirs = s3r.capture_frames(board, os.path.join(cdir, "frame-%s" % tag),
                              "entry-%s" % tag, n)
    meas = m3.measure_layer([("now", dirs)], grid=grid,
                            window_xy_mm=window_xy_mm, layer=0,
                            z_bias_mm=0.0, radius_mm=float(radius_mm))
    _dump(os.path.join(cdir, "measure-%s.json" % tag), meas)
    msg = tower_occupancy_verdict(meas.get("merged"), cell_idx)
    if msg:
        raise EntryStop(msg + " —— 还没清空，不往下跑")
    return meas


def pick_item(args, analysis, scene, zcal, n=1):
    """拾取帧 → 要抓的块；候选不够时转成 ``EntryStop``（不是 traceback）。"""
    try:
        return s3r.build_items(analysis, scene, n, arm=args.arm, zcal=zcal)
    except RuntimeError as exc:
        raise EntryStop("拾取区没有可用块：%s" % exc)


#: `measure_pair` 里 `go_move` 要 URDF；由 `main` 设一次。
_URDF = None


def capture_pick(args, board, scene, cdir, tag):
    """走到拾取巡检位 → 按 ``scene.pick_frames`` 连拍，返回 (首帧, 额外帧)。"""
    rc, frame_dir = s2.do_capture(board, "stage2pick",
                                  os.path.join(cdir, "frame-pick"),
                                  args.urdf, cdir, tag)
    if rc != 0:
        raise EntryStop("走到拾取巡检位失败（rc=%d）" % rc, rc)
    extras = []
    for j in range(1, max(1, int(scene.get("pick_frames", 1)))):
        fd = os.path.join(cdir, "frame-pick-%s" % "bcdefgh"[j - 1])
        board.capture(fd, "%s-pick-%d" % (tag, j + 1))
        extras.append(fd)
    return frame_dir, extras


def analyse_pick(frame_dir, extras, cdir):
    """拾取帧 → 分析结果（多帧融合 yaw）。"""
    analysis = s2.analyse_frame(frame_dir, cdir)
    if extras:
        fused = []
        for fd in extras:
            f = gc.load_frame(fd)
            eb, _tz = gc.find_blocks(f)
            fused.append(s2.add_pixel_yaw(f, eb))
        s2.fuse_block_yaw(analysis["blocks"], fused)
    return analysis


def pick_bias_of(analysis, item):
    """拾取侧偏差：被选中那块在帧里的 ``h − 29.5``（与历史复算同口径）。"""
    for b in analysis["blocks"]:
        if int(b["idx"]) == int(item["block_idx"]):
            h = b.get("h")
            if h is None:
                return None, b
            return round(float(h) - m3.BLOCK_TRUE_H_MM, 3), b
    return None, None


def load_rows(cdir):
    """``--resume``：把已跑完的 ``<tag>/row.json`` 读回来（按 rep 排序）。

    只收 ``rep`` 是整数的条目；坏文件跳过并留一句 —— 续跑宁可少一条让人补跑，
    也不要半条脏数据混进 z 标定 / 释放统计。
    """
    rows = []
    if not os.path.isdir(cdir):
        return rows
    for name in sorted(os.listdir(cdir)):
        path = os.path.join(cdir, name, "row.json")
        if not os.path.isfile(path):
            continue
        try:
            row = json.load(open(path))
        except ValueError:
            print("  续跑：跳过坏文件 %s" % path)
            continue
        if isinstance(row, dict) and isinstance(row.get("rep"), int):
            rows.append(row)
    rows.sort(key=lambda x: x["rep"])
    return rows


def free_slot_mm(blocks, zone_m, gap_mm=PICK_RETURN_GAP_MM,
                 margin_mm=PICK_RETURN_MARGIN_MM, step_mm=5.0):
    """拾取区里给「回位的块」找一个空位（格点搜索，纯逻辑）。

    ``zone_m`` = ``[x0, x1, y0, y1]``（**米**，base 系，同 ``scene["pick_zone"]``）。
    合格 = 离**每个**已有块中心 ≥ ``gap_mm``、离区边 ≥ ``margin_mm``；
    合格点里取**离区心最近**的（同分取离块更远者）—— 回位块尽量靠中间，
    不往边界角上堆，也不贴着要抓的那块。

    返回 ``{"xy_mm": [x, y], "gap_mm": 最近块距, "edge_mm": 离边最近}``；
    一个合格点都没有就返回 ``None`` —— 调用方 fail-closed，不硬塞。
    """
    x0, x1, y0, y1 = [float(v) * 1000.0 for v in zone_m]
    pts = [(float(b["cx"]), float(b["cy"])) for b in (blocks or [])]
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    lo_x, hi_x = x0 + float(margin_mm), x1 - float(margin_mm)
    lo_y, hi_y = y0 + float(margin_mm), y1 - float(margin_mm)
    if lo_x > hi_x or lo_y > hi_y:
        return None
    best, best_key = None, None
    y = lo_y
    while y <= hi_y + 1e-6:
        x = lo_x
        while x <= hi_x + 1e-6:
            gap = (min(math.hypot(x - px, y - py) for px, py in pts)
                   if pts else float("inf"))
            edge = min(x - x0, x1 - x, y - y0, y1 - y)
            if gap >= float(gap_mm) and edge >= float(margin_mm):
                key = (round(math.hypot(x - mid_x, y - mid_y), 3), -gap)
                if best_key is None or key < best_key:
                    best_key = key
                    best = {"xy_mm": [round(x, 2), round(y, 2)],
                            "gap_mm": (None if gap == float("inf")
                                       else round(gap, 2)),
                            "edge_mm": round(edge, 2)}
            x += float(step_mm)
        y += float(step_mm)
    return best


def tower_block_item(merged, scene, zcal, arm="geometric"):
    """把「塔上那块」的实测值包装成 ``build_items`` 同款的 item（拆塔用）。

    ``merged`` = 上一轮塔顶的融合测量（``measure_pair(...)["merged"]``）。
    中心 / yaw 直接用；``h`` 用 ``顶高 − z 标定(tower)`` —— ``build_pick``
    的抓取高度整个锚在**台面**（``TABLE_PLANE_Z``），塔上的块只是 ``wz``
    比台面上的块大了一个块高（垫 + 29.5）。

    yaw 只有 ``yaw_rect`` 一个来源（没有像素最小外接矩形），按几何臂的
    ``sign·θ + δ`` 走；量不到 yaw 时 ``yaw_for_arm`` 会返回 0 并标
    ``geometric_missing`` —— 半开 50 mm 仍容得下 30 mm 方块的对角，不换全开。
    """
    scene = scene or {}
    cx, cy = [float(v) for v in merged["center_base_mm"]]
    h_mm = float(merged["top_mm"]) - float((zcal or {}).get("tower", 0.0))
    yaw = merged.get("yaw_deg")
    block = {"idx": TOWER_BLOCK_IDX, "cx": cx, "cy": cy,
             "top": h_mm, "h": h_mm,
             "yaw_pix": None, "yaw_rect": yaw,
             # ``yaw_for_arm`` 只在**这条**为 False 时标 untrusted
             "yaw_pix_trusted": bool(merged.get("yaw_trusted", True)),
             "class": merged.get("class")}
    cand = {"xyz_base": [cx / 1000.0, cy / 1000.0, h_mm / 1000.0],
            "dist_mm": 0.0,               # 块心就是候选点（grasp_at=块心）
            "angle_deg": 0.0 if yaw is None else float(yaw),
            "score": 1.0,
            # 只进计划 meta（开合宽度由 scene 的 pick/place_open_mm 定），
            # 塔上那块没有 GraspNet 候选宽度可读，给标称值。
            "width_mm": 30.0}
    yg, yp, src, why = s2.yaw_for_arm(arm, block, cand, scene)
    return {"order": 1, "block_idx": TOWER_BLOCK_IDX,
            "class": merged.get("class"), "arm": arm,
            "block": block, "cand": cand,
            "grasp_at": scene.get("grasp_at", "block_center"),
            "block_h_raw_mm": round(h_mm, 3),
            "block_h_used_mm": round(h_mm, 3),
            "yaw_grasp_deg": round(float(yg), 3),
            "yaw_place_deg": round(float(yp), 3),
            "yaw_source": src, "yaw_note": why}


def place_one(args, board, chain, scene, grid, analysis, item, slot_mm,
              surface_mm, cdir, tag, survey_top_mm=None, view_xy_mm=None,
              z_bias_mm=0.0, plan_kwargs=None, exec_kwargs=None):
    """抓一块放到塔位 ``surface_mm`` 上并测量；返回记录。

    ``exec_kwargs`` 可带 ``upto`` —— 遮挡实验停在 ``place_approach``；
    测量在调用方做（停下时相机不在 survey 位）。
    """
    plan_kwargs = dict(plan_kwargs or {})
    exec_kwargs = dict(exec_kwargs or {})
    view_xy = view_xy_mm or slot_mm
    top_for_survey = (float(survey_top_mm) if survey_top_mm is not None
                      else float(surface_mm) + m3.BLOCK_TRUE_H_MM)
    pair = _survey_pair(top_for_survey, view_xy, args.d_mm)
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q_start = s2.chain_q(js)
    surface_m = float(surface_mm) / 1000.0
    carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0, surface_m)
    plan = s3r.layer_plan(chain, q_start, item, analysis["blocks"],
                          [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0],
                          surface_m, survey_tool0=pair[0], carry_z=carry_z,
                          scene=scene, **plan_kwargs)
    plan_path = s3r.write_plan(os.path.join(cdir, "plan-%s.json" % tag), plan)
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True, **exec_kwargs)
    rec = {"tag": tag, "block_idx": item["block_idx"],
           "class": item.get("class"),
           "block_h_used_mm": item["block_h_used_mm"],
           "surface_z_mm": round(float(surface_mm), 3),
           "survey_tool0": plan["meta"]["survey_tool0"],
           "predicted_place_mm": [
               round(v * 1000.0, 2)
               for v in s3r.predicted_place(
                   item, [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0])[0]],
           "predicted_top_mm": round(float(surface_mm)
                                     + pp.PLACE_CLEARANCE * 1000.0
                                     + m3.BLOCK_TRUE_H_MM, 2),
           "exec_rc": rc, "pair": pair, "plan": os.path.basename(plan_path)}
    if rc != 0:
        raise EntryStop("exec 判红（rc=%d，%s）—— 停在原地，不续跑"
                        % (rc, os.path.basename(plan_path)), rc)
    if exec_kwargs.get("upto"):
        rec["stopped_at"] = exec_kwargs["upto"]
        return rec
    meas = measure_pair(board, chain, scene, cdir, tag, pair, grid, view_xy,
                        layer=1, z_bias_mm=z_bias_mm, move=False)
    rec["measured"] = meas
    return rec


def finish_plan(board, cdir, tag, plan_path, after):
    """把停在半路的计划跑完（``--from`` 下一条航点）。

    返回 ``(rc, 实际起跑的航点名)``。
    """
    plan = json.load(open(plan_path))
    nxt = pr.next_waypoint(plan, after)
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True, from_=nxt, append=True)
    return rc, nxt


def return_to_pick(args, board, chain, scene, cdir, tag, grid, slot_mm,
                   merged, zcal, rdir=None):
    """**机器人自己**把塔上那块抓下来、放回拾取区空位，再回核塔位清空。

    这是「拆塔」的最小形态，不引入新传感器：
    · 抓哪儿 = 上一轮塔顶实测（中心 / yaw / 顶高），``tower_block_item`` 包装；
    · 放哪儿 = **新抓一帧**拾取区，``free_slot_mm`` 选空位；
    · 收尾 = 计划的巡检位就设在塔上方，跑完**原地**``tower_clear_now`` 核空。

    计划从**当前实测关节角**起（刚在拾取巡检位抓完帧），一条计划走完
    「拾取位 → 塔上方抓块 → 抬到载运高 → 拾取区空位放下 → 回塔上方」。
    """
    rdir = rdir or cdir
    os.makedirs(rdir, exist_ok=True)
    frame_dir, extras = capture_pick(args, board, scene, rdir, tag + "-rp")
    analysis = analyse_pick(frame_dir, extras, rdir)
    spot = free_slot_mm(analysis["blocks"], scene["pick_zone"])
    if spot is None:
        raise EntryStop("拾取区放不下回位的块：没有离块 ≥%.0f mm、离边 ≥%.0f mm "
                        "的空位（先把块挪一挪再续跑）"
                        % (PICK_RETURN_GAP_MM, PICK_RETURN_MARGIN_MM))
    item = tower_block_item(merged, scene, zcal, arm="geometric")
    pair = _survey_pair(merged["top_mm"], merged["center_base_mm"], args.d_mm)
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q_start = s2.chain_q(js)
    carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0,
                                pp.TABLE_PLANE_Z)
    plan = s3r.layer_plan(chain, q_start, item, analysis["blocks"],
                          [spot["xy_mm"][0] / 1000.0,
                           spot["xy_mm"][1] / 1000.0],
                          pp.TABLE_PLANE_Z, survey_tool0=pair[0],
                          carry_z=carry_z, scene=scene,
                          release_dwell_s=0.0,
                          tower_half_edge=args.tower_half_edge)
    plan_path = s3r.write_plan(os.path.join(cdir, "plan-%s.json" % tag), plan)
    rec = {"tag": tag, "kind": "return",
           "spot_mm": spot["xy_mm"], "spot_gap_mm": spot["gap_mm"],
           "spot_edge_mm": spot["edge_mm"],
           "tower_top_mm": merged["top_mm"],
           "tower_center_mm": merged["center_base_mm"],
           "yaw_grasp_deg": item["yaw_grasp_deg"],
           "yaw_source": item["yaw_source"],
           "predicted_place_mm": [
               round(v * 1000.0, 2)
               for v in s3r.predicted_place(
                   item, [spot["xy_mm"][0] / 1000.0,
                          spot["xy_mm"][1] / 1000.0])[0]],
           "plan": os.path.basename(plan_path)}
    rc = board.exec_plan(plan_path, os.path.join(cdir, "log-%s.jsonl" % tag),
                         go=True)
    rec["exec_rc"] = rc
    _dump(os.path.join(cdir, "return-%s.json" % tag), rec)
    if rc != 0:
        raise EntryStop("回位执行判红（rc=%d）—— 停在原地不续跑；"
                        "块可能还在爪里或半放，先看示教器再决定"
                        % rc, rc)
    print("  回位：塔上那块 → 拾取区 %s（离最近块 %s mm，离边 %s mm，yaw %s）"
          % (spot["xy_mm"], spot["gap_mm"], spot["edge_mm"],
             item["yaw_source"]))
    tower_clear_now(board, scene, cdir, tag + "-clear", grid, slot_mm,
                    cell_idx=args.tower_cell_idx)
    return rec


# ---------------------------------------------------------------- 模式
def clear_tower_step(args, board, chain, scene, cdir, tag, grid, slot_mm,
                     placed, merged=None, zcal=None, rdir=None):
    """每一轮放置前：把塔上那块清掉，并**就地核一次**塔位空了。

    默认**机器人自己回位**（``return_to_pick``）——纯测试性的往返不该压给人。
    ``--no-self-return`` 退回人手动拿：``ask`` 一句 + ``tower_clear_now``。
    ``merged`` 缺（老调用/数据不全）时也退回人手动，并在日志里写明原因。
    """
    if not placed:
        return None
    if getattr(args, "no_self_return", False):
        ask("把这块从塔位拿走，放回拾取区；确认塔位清空")
        return tower_clear_now(board, scene, cdir, tag + "-clear", grid,
                               slot_mm, cell_idx=args.tower_cell_idx)
    if not merged:
        raise EntryStop("要机器人自己回位，但上一轮没有塔顶实测（merged 为空）"
                        " —— 用人手清塔后加 --no-self-return 再跑")
    return return_to_pick(args, board, chain, scene, cdir, tag, grid, slot_mm,
                          merged, zcal, rdir=rdir)


def mode_zcal(args, ctx):
    """① 塔位 z 标定：单块重复放置。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    cdir = os.path.join(args.run_dir, "entry-zcal")
    os.makedirs(cdir, exist_ok=True)
    if not getattr(args, "resume", False):
        # 续跑时塔位上**本来就压着**上一次的块（正是要机器人拆回来的那块），
        # 所以预检只在全新开跑时做。
        try:
            tower_precheck(args, board, chain, scene, grid, cdir)
        except EntryStop as exc:
            print("!! " + exc.why)
            return exc.rc
    rows, stop, placed = [], None, False
    last_merged = None
    if getattr(args, "resume", False):
        rows = load_rows(cdir)
        if rows:
            placed = True
            last_merged = rows[-1].get("merged") or rows[-1]
            print("  续跑：已有 %d 次结果，从第 %d 次接着跑"
                  % (len(rows), len(rows) + 1))
            if not getattr(args, "no_self_return", False):
                # 第一次「机器人自己拆塔」是**新的动作形态**：当次握手，不继承
                # 之前任何一次授权（AGENTS §1 红线 6）。
                ask("确认急停就绪、两指整个行程内无手无遮挡："
                    "接下来由机器人自己把塔上那块抓回拾取区")
    for r in range(len(rows) + 1, int(args.reps) + 1):
        tag = "zcal-%02d" % r
        rdir = os.path.join(cdir, tag)
        os.makedirs(rdir, exist_ok=True)
        try:
            clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                             slot_mm, placed, merged=last_merged, zcal=zcal,
                             rdir=rdir)
            frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
            analysis = analyse_pick(frame_dir, extras, rdir)
            item = pick_item(args, analysis, scene, zcal)[0]
            pick_bias, _b = pick_bias_of(analysis, item)
            # 第 1 层：表面 = 垫面；期望顶高 = 垫面 + 29.5（survey 位姿按它算）
            rec = place_one(args, board, chain, scene, grid, analysis, item,
                            slot_mm, pad_mm, rdir, tag,
                            survey_top_mm=pad_mm + m3.BLOCK_TRUE_H_MM,
                            z_bias_mm=0.0, plan_kwargs={"tower_half_edge":
                                                        args.tower_half_edge})
        except EntryStop as exc:
            stop = exc.why
            break
        placed = True
        merged = rec["measured"]["merged"]
        last_merged = merged
        row = {"rep": r, "top_mm": merged["top_mm"],
               "center_base_mm": merged["center_base_mm"],
               "yaw_deg": merged.get("yaw_deg"),
               "pick_bias_mm": pick_bias,
               "block_idx": item["block_idx"], "class": item.get("class"),
               # 整份融合观测 —— `--resume` 拿它当「塔上那块」，回位/拆塔要用
               "merged": merged}
        rows.append(row)
        _dump(os.path.join(rdir, "row.json"), row)
        print("  第 %d/%d 次：顶高 %.2f mm，中心 %s，偏差 %.2f mm"
              % (r, int(args.reps), merged["top_mm"],
                 merged["center_base_mm"],
                 merged["top_mm"] - pad_mm - m3.BLOCK_TRUE_H_MM))
    if placed and stop is None:
        # 收尾：最后一块也由机器人拿回拾取区，塔位留给下一项实验是空的
        # （不然下一项实验的塔位预检会直接判红）。失败不吞：记进 stop。
        try:
            clear_tower_step(args, board, chain, scene, cdir,
                             "zcal-%02d-end" % len(rows), grid, slot_mm, True,
                             merged=last_merged, zcal=zcal)
            placed = False
        except EntryStop as exc:
            stop = "收尾回位失败：%s" % exc.why
    doc = zcal_doc(rows, pad_mm, meta={
        "kind": "stage3-entry-zcal", "round_dir": cdir,
        "d_mm": float(args.d_mm), "arm": args.arm,
        "tower_cell_idx": int(args.tower_cell_idx),
        "stop_reason": stop,
    })
    path = os.path.join(cdir, "zcal.json")
    _dump(path, doc)
    print("\nz 标定：n=%d 中位 %s mm，残差最大 %s mm，%s"
          % (doc["n"], doc["by_site"]["tower"],
             doc["residual_mm"]["max"],
             "通过" if doc["pass"] else "**未过**（残差 >%.1f mm）"
             % ZCAL_TOL_MM))
    print("  落盘 → %s（`stage3_round.py --zcal` 直接吃）" % path)
    if stop:
        print("!! 停在：%s —— 不自动续跑" % stop)
        return 2
    return 0 if doc["pass"] else 1


def _release_row(item, rec, rep):
    """一次放置的「实测 − 预测」：Δxy、Δyaw（mod 90）、Δz（落稳下沉）。"""
    merged = rec["measured"]["merged"]
    dx = merged["center_base_mm"][0] - rec["predicted_place_mm"][0]
    dy = merged["center_base_mm"][1] - rec["predicted_place_mm"][1]
    dyaw = (None if merged.get("yaw_deg") is None else
            round(float(s2.signed_err_mod90(
                merged["yaw_deg"], float(item["yaw_place_deg"]))), 3))
    return {"rep": rep, "dxy_mm": round(math.hypot(dx, dy), 3),
            "dx_mm": round(dx, 3), "dy_mm": round(dy, 3),
            "dyaw_deg": dyaw,
            "dz_mm": round(merged["top_mm"] - rec["predicted_top_mm"], 3),
            "measured": merged,
            "predicted_place_mm": rec["predicted_place_mm"],
            "predicted_top_mm": rec["predicted_top_mm"],
            "block_idx": item["block_idx"]}


def mode_release(args, ctx):
    """② 释放扫参：开口 × 停留，各若干次。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    cdir = os.path.join(args.run_dir, "entry-release")
    os.makedirs(cdir, exist_ok=True)
    opens = [float(v) for v in str(args.opens).split(",") if v]
    dwells = [float(v) for v in str(args.dwells).split(",") if v]
    groups = {}
    order = [(("open%.0f_dwell%.2f" % (op, dw)), op, dw)
             for op in opens for dw in dwells]
    try:
        tower_precheck(args, board, chain, scene, grid, cdir)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    stop, placed, last_merged = None, False, None
    for name, op, dw in order:
        if stop:
            break
        reps = int(args.reps)
        gdir = os.path.join(cdir, name)
        os.makedirs(gdir, exist_ok=True)
        rows = groups.setdefault(name, [])
        for r in range(1, reps + 1):
            tag = "%s-%02d" % (name, r)
            rdir = os.path.join(gdir, tag)
            os.makedirs(rdir, exist_ok=True)
            try:
                clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                                 slot_mm, placed, merged=last_merged,
                                 zcal=zcal, rdir=rdir)
                frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
                analysis = analyse_pick(frame_dir, extras, rdir)
                item = pick_item(args, analysis, scene, zcal)[0]
                rec = place_one(args, board, chain, scene, grid, analysis,
                                item, slot_mm, pad_mm, rdir, tag,
                                z_bias_mm=zcal["tower"],
                                plan_kwargs={
                                    "place_open_mm": op,
                                    "release_dwell_s": dw,
                                    "tower_half_edge": args.tower_half_edge})
            except EntryStop as exc:
                stop = exc.why
                break
            placed = True
            last_merged = rec["measured"]["merged"]
            row = _release_row(item, rec, r)
            rows.append(row)
            _dump(os.path.join(rdir, "row.json"), row)
            print("  %s 第 %d/%d 次：Δxy %.2f mm，Δyaw %s°，Δz %.2f mm"
                  % (name, r, reps, row["dxy_mm"], row["dyaw_deg"],
                     row["dz_mm"]))
        groups[name] = rows
        _dump(os.path.join(cdir, "summary.json"),
              {"kind": "stage3-entry-release", "groups_raw": groups,
               "summary": release_summary(groups), "stop_reason": stop})
    # 最优组补次数
    summ = release_summary(groups)
    best = summ.get("best")
    if stop is None and best and args.confirm and int(args.confirm) > 0:
        print("最优组是 %s；接下来补 %d 次确认（每次仍由机器人自己回位）"
              % (best, int(args.confirm)))
        op, dw = [(o, d) for n, o, d in order if n == best][0]
        gdir = os.path.join(cdir, best)
        rows = groups.setdefault(best, [])
        for r in range(1, int(args.confirm) + 1):
            tag = "%s-confirm-%02d" % (best, r)
            rdir = os.path.join(gdir, tag)
            os.makedirs(rdir, exist_ok=True)
            try:
                clear_tower_step(args, board, chain, scene, cdir, tag, grid,
                                 slot_mm, placed, merged=last_merged,
                                 zcal=zcal, rdir=rdir)
                frame_dir, extras = capture_pick(args, board, scene, rdir, tag)
                analysis = analyse_pick(frame_dir, extras, rdir)
                item = pick_item(args, analysis, scene, zcal)[0]
                rec = place_one(args, board, chain, scene, grid, analysis,
                                item, slot_mm, pad_mm, rdir, tag,
                                z_bias_mm=zcal["tower"],
                                plan_kwargs={
                                    "place_open_mm": op,
                                    "release_dwell_s": dw,
                                    "tower_half_edge": args.tower_half_edge})
            except EntryStop as exc:
                stop = exc.why
                break
            placed = True
            last_merged = rec["measured"]["merged"]
            row = _release_row(item, rec, int(args.reps) + r)
            rows.append(row)
            _dump(os.path.join(rdir, "row.json"), row)
            print("  确认 %d/%d：Δxy %.2f mm，Δyaw %s°"
                  % (r, int(args.confirm), row["dxy_mm"], row["dyaw_deg"]))
    if placed and stop is None:
        # 收尾：最后一块也由机器人拿回拾取区（塔位留给下一项实验）
        try:
            clear_tower_step(args, board, chain, scene, cdir, "release-end",
                             grid, slot_mm, True, merged=last_merged, zcal=zcal)
        except EntryStop as exc:
            stop = "收尾回位失败：%s" % exc.why
    summ = release_summary(groups)
    doc = {"kind": "stage3-entry-release", "grid_center_base_mm": slot_mm,
           "opens": opens, "dwells": dwells, "reps": int(args.reps),
           "confirm": int(args.confirm) if args.confirm else 0,
           "groups_raw": groups, "summary": summ, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    print("\n释放扫参（按重复性排序）：")
    for name in summ["order"]:
        s = summ["groups"][name]
        print("  %-18s n=%-3d Δxy 极差 %.2f 中位 %.2f | Δyaw 极差 %s"
              % (name, s["n"], s["dxy_mm"].get("range", float("nan")),
                 s["dxy_mm"].get("median", float("nan")),
                 s["dyaw_deg"].get("range")))
    print("  数据 → %s" % path)
    print("  数值闸门不在这里编：把这张表带给用户拍板，再写进契约 §3-2")
    return 2 if stop else 0


def mode_occlusion(args, ctx):
    """③ 持块遮挡：夹着块停在 place_approach，看下层塔顶还在不在。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    heights = [int(v) for v in str(args.heights).split(",") if v]
    cdir = os.path.join(args.run_dir, "entry-occlusion")
    os.makedirs(cdir, exist_ok=True)
    try:
        tower_precheck(args, board, chain, scene, grid, cdir)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    out, stop = [], None
    for h in heights:
        hdir = os.path.join(cdir, "layer%02d" % h)
        os.makedirs(hdir, exist_ok=True)
        tower_top_mm = pad_mm + h * m3.BLOCK_TRUE_H_MM
        try:
            # ---- 基准：人手搭好的塔，先按正常距离量一遍
            ask("请在塔位（格 %d）用 %d 块搭好直塔，然后离开工作区"
                % (int(args.tower_cell_idx), h))
            pair = _survey_pair(tower_top_mm, slot_mm, args.d_mm)
            base = measure_pair(board, chain, scene, hdir, "base", pair, grid,
                                slot_mm, layer=h, z_bias_mm=zcal["tower"])
            # 搭错层数 / 塔位没清空 ⇒ 后面按期望顶高分类会全错，就地停
            ok, dh = layer_height_check(base["merged"]["top_mm"],
                                        tower_top_mm)
            if not ok:
                raise EntryStop(
                    "%d 层塔顶量到 %.1f mm，应有 %.1f mm（差 %+.1f mm）——"
                    "塔位没清空或块数不对，拆掉重搭"
                    % (h, base["merged"]["top_mm"], tower_top_mm, dh))
            # ---- 夹着块停在 place_approach
            frame_dir, extras = capture_pick(args, board, scene, hdir, "pick")
            analysis = analyse_pick(frame_dir, extras, hdir)
            item = pick_item(args, analysis, scene, zcal)[0]
            rec = place_one(args, board, chain, scene, grid, analysis, item,
                            slot_mm, tower_top_mm, hdir, "hold",
                            survey_top_mm=tower_top_mm,
                            z_bias_mm=zcal["tower"],
                            plan_kwargs={"tower_xy_mm": slot_mm,
                                         "tower_top_mm": tower_top_mm,
                                         "tower_half_edge": args.tower_half_edge},
                            exec_kwargs={"upto": "place_approach"})
            n = int(scene.get("terminal_frames", 3))
            dirs = s3r.capture_frames(
                board, os.path.join(hdir, "frame-hold"),
                "entry-occl-l%02d-hold" % h, n)
            blocks, _tz = gc.find_blocks(gc.load_frame(dirs[0]),
                                         max_h_mm=m3.MAX_H_MM,
                                         min_h_mm=m3.MIN_H_MM)
            held_top_mm = (tower_top_mm + pp.PLACE_APPROACH_GAP * 1000.0
                           + pp.PLACE_CLEARANCE * 1000.0
                           + m3.BLOCK_TRUE_H_MM)
            rep = occlusion_report(blocks, tower_top_mm, slot_mm,
                                   held_top_mm=held_top_mm)
            rep.update({"layer": h, "frames": [os.path.basename(d)
                                               for d in dirs],
                        "baseline_merged": base["merged"],
                        "held_frame_dir": dirs[0]})
            _dump(os.path.join(hdir, "occlusion.json"), rep)
            out.append(rep)
            print("  %d 层：塔窗内 %d 簇；塔顶高度上 %d 簇 ⇒ %s"
                  % (h, rep["n_window"], len(rep["tower_clusters"]),
                     "看得见" if rep["tower_top_visible"] else "**看不见**"))
            # ---- 把这一块放上去（塔变 h+1 层，由人重搭）
            rc, nxt = finish_plan(board, hdir, "hold",
                                  os.path.join(hdir, "plan-hold.json"),
                                  "place_approach")
            if rc != 0:
                raise EntryStop("补完放置失败（rc=%d，从 %s）" % (rc, nxt), rc)
        except EntryStop as exc:
            stop = exc.why
            break
        if h != heights[-1]:
            ask("刚才这块已经放到塔顶（塔现在 %d 层）。请把塔拆掉，"
                "按下一档高度重搭" % (h + 1))
    doc = {"kind": "stage3-entry-occlusion", "heights": heights,
           "d_mm": float(args.d_mm), "results": out, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    visible = [r["layer"] for r in out if r["tower_top_visible"]]
    print("\n遮挡实验结果：%s"
          % ("、".join("%d 层可见" % v for v in visible) if visible
             else "所有测过的塔层都看不见塔顶"))
    print("  数据 → %s" % path)
    print("  eye-in-hand 对照组做不做由用户看这张表拍板")
    return 2 if stop else 0


def mode_tolerance(args, ctx):
    """④ 对中容差：人手偏置放置，机器人只动臂。"""
    board, chain, scene, grid, slot_mm, pad_mm, zcal = ctx
    offsets = [float(v) for v in str(args.offsets).split(",") if v]
    cdir = os.path.join(args.run_dir, "entry-tolerance")
    os.makedirs(cdir, exist_ok=True)
    try:
        tower_precheck(args, board, chain, scene, grid, cdir)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    out, stop = [], None
    for nominal in offsets:
        odir = os.path.join(cdir, "off%02.0f" % nominal)
        os.makedirs(odir, exist_ok=True)
        try:
            # ---- 下块（基准）
            ask("请在塔位（格 %d）居中放**一块**，四边与垫子格线对齐，然后离开工作区"
                % int(args.tower_cell_idx))
            pair = _survey_pair(pad_mm + m3.BLOCK_TRUE_H_MM, slot_mm, args.d_mm)
            lower = measure_pair(board, chain, scene, odir, "lower", pair,
                                 grid, slot_mm, layer=1,
                                 z_bias_mm=zcal["tower"])
            ok, dh = layer_height_check(lower["merged"]["top_mm"],
                                        pad_mm + m3.BLOCK_TRUE_H_MM)
            if not ok:
                raise EntryStop(
                    "下块顶高 %.1f mm，单块应为 %.1f mm（差 %+.1f mm）——"
                    "塔位没清空？清掉重放"
                    % (lower["merged"]["top_mm"],
                       pad_mm + m3.BLOCK_TRUE_H_MM, dh))
            # ---- 上块（人手偏置）
            if nominal <= 0:
                ask("请在它正上方摆第二块（尽量对齐）")
            else:
                ask("请把第二块摆在下块正上方，沿 base x 正方向偏约 %g mm"
                    "（yaw 尽量对齐）" % nominal)
            pair2 = _survey_pair(pad_mm + 2 * m3.BLOCK_TRUE_H_MM, slot_mm,
                                 args.d_mm)
            upper = measure_pair(board, chain, scene, odir, "upper", pair2,
                                 grid, slot_mm, layer=2,
                                 z_bias_mm=zcal["tower"])
            ok, dh = layer_height_check(upper["merged"]["top_mm"],
                                        pad_mm + 2 * m3.BLOCK_TRUE_H_MM)
            if not ok:
                raise EntryStop(
                    "两层顶高 %.1f mm，应有 %.1f mm（差 %+.1f mm）——"
                    "第二块没放上去？"
                    % (upper["merged"]["top_mm"],
                       pad_mm + 2 * m3.BLOCK_TRUE_H_MM, dh))
            print("  静置 %.1f s 后复测" % float(args.hold_s))
            time.sleep(float(args.hold_s))
            pair3 = _survey_pair(pad_mm + 2 * m3.BLOCK_TRUE_H_MM, slot_mm,
                                 args.d_mm)
            after = measure_pair(board, chain, scene, odir, "after", pair3,
                                 grid, slot_mm, layer=2,
                                 z_bias_mm=zcal["tower"])
            lo = lower["merged"]
            up = upper["merged"]
            af = after["merged"]
            off = tol_offset_mm(lo["center_base_mm"], up["center_base_mm"])
            rec = {"nominal_mm": nominal, "offset": off,
                   "lower_center_mm": lo["center_base_mm"],
                   "lower_top_mm": lo["top_mm"],
                   "upper_center_mm": up["center_base_mm"],
                   "upper_top_mm": up["top_mm"],
                   "upper_yaw_deg": up.get("yaw_deg"),
                   "after_center_mm": af["center_base_mm"],
                   "after_top_mm": af["top_mm"],
                   "drift_xy_mm": round(math.hypot(
                       af["center_base_mm"][0] - up["center_base_mm"][0],
                       af["center_base_mm"][1] - up["center_base_mm"][1]), 2),
                   "drift_z_mm": round(af["top_mm"] - up["top_mm"], 2),
                   "dz_layer_mm": round(up["top_mm"] - lo["top_mm"], 2),
                   # 塔倒的机判：静置后顶高掉 10 mm 以上（契约 §3 的塌陷判据）
                   "collapsed": bool(af["top_mm"] - up["top_mm"] <= -10.0)}
            out.append(rec)
            _dump(os.path.join(odir, "row.json"), rec)
            print("  标称 %g mm → 实测 %.2f mm；静置漂移 xy %.2f / z %.2f mm%s"
                  % (nominal, off["d_mm"], rec["drift_xy_mm"],
                     rec["drift_z_mm"],
                     "；**塌了**" if rec["collapsed"] else ""))
            ask("请把这两块都拿走，塔位清空")
        except EntryStop as exc:
            stop = exc.why
            break
    doc = {"kind": "stage3-entry-tolerance", "offsets": offsets,
           "hold_s": float(args.hold_s), "results": out, "stop_reason": stop}
    path = _dump(os.path.join(cdir, "summary.json"), doc)
    print("\n对中容差：")
    for r in out:
        print("  标称 %-4g mm → 实测 %.2f mm，静置漂移 %.2f/%.2f mm%s"
              % (r["nominal_mm"], r["offset"]["d_mm"], r["drift_xy_mm"],
                 r["drift_z_mm"], "（塌）" if r["collapsed"] else ""))
    print("  数据 → %s" % path)
    return 2 if stop else 0


# ---------------------------------------------------------------- 上下文/CLI
def load_ctx(args):
    """场景 / 网格 / 塔位 / z 标定 / URDF —— 四个模式共用。"""
    global _URDF
    _URDF = args.urdf
    try:
        scene = s2.load_scene(args.scene)
    except ValueError as exc:
        raise EntryStop("场景坏：%s" % exc)
    grid_path = args.grid or scene.get("grid_json")
    if not grid_path:
        raise EntryStop("缺 grid.json —— 塔基、垫面、格线方向都要它")
    try:
        grid = s2.load_grid(grid_path)
        s2.check_grid_order(scene, grid)
    except ValueError as exc:
        raise EntryStop("网格坏：%s" % exc)
    slot_mm = m3.grid_center(grid, args.tower_cell_idx)
    pad_mm = m3.pad_z_mm(grid)
    zcal = s3r.load_zcal(args.zcal)
    chain = urkin.Chain(args.urdf)
    return scene, grid, slot_mm, pad_mm, zcal, chain


def protocol(mode):
    head = ("开工前先量一次塔位：非空立刻停（阶段 2 的块还压着中心格时要先清格）；"
            "拾取区没有可用块也停。")
    return head + {
        "zcal": "① z 标定：塔位单块重复放置 --reps 次（默认 5），"
                "两个高度跟踪视点各连拍 scene.terminal_frames 帧（默认 3）；"
                "每次由**机器人自己**把塔上那块抓回拾取区空位"
                "（--no-self-return 才退回人手动），放回后就地核一次「塔位真的空了」。",
        "release": "② 释放扫参：开口 --opens × 停留 --dwells，每组 --reps 次；"
                   "最优组再 --confirm 次。每次由**机器人自己**回位（同上）。",
        "occlusion": "③ 持块遮挡：人手搭 1/3/5 层塔，机器人先量基准（塔高不对就停），"
                     "再夹块停在 place_approach 抓帧；抓完把这块放上去，由人重搭。",
        "tolerance": "④ 对中容差：人手把上块偏 --offsets 放上去，"
                     "机器人只动臂测量 + 静置复测（下块/两层高度对不上就停）。",
    }[mode]


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 3 入口四实验驱动")
    ap.add_argument("--mode", required=True, choices=MODES)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None)
    ap.add_argument("--zcal", default=None, help="已有的 z 标定（zcal 模式不用）")
    ap.add_argument("--arm", default="geometric",
                    choices=("geometric", "zero", "graspnet"))
    ap.add_argument("--tower-cell-idx", type=int, default=m3.TOWER_CELL_IDX)
    ap.add_argument("--tower-half-edge", type=float, default=0.020)
    ap.add_argument("--d-mm", type=float,
                    default=pp.STAGE3_SURVEY_DEFAULT_D * 1000.0)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--opens", default="50,70,95")
    ap.add_argument("--dwells", default="0,0.5")
    ap.add_argument("--confirm", type=int, default=10)
    ap.add_argument("--heights", default="1,3,5")
    ap.add_argument("--offsets", default="0,3,5,8")
    ap.add_argument("--hold-s", type=float, default=10.0)
    ap.add_argument("--no-self-return", action="store_true",
                    help="把「塔上那块」交回人手动拿走；默认由机器人自己抓回拾取区")
    ap.add_argument("--resume", action="store_true",
                    help="zcal：复用 run-dir 里已有的 row.json，从下一位接着跑")
    ap.add_argument("--go", action="store_true",
                    help="真跑；**每次都当次授权**，闸门在 precision_exec")
    args = ap.parse_args(argv)

    if not args.go:
        print(protocol(args.mode))
        print("（没加 --go：只打印协议，不动臂、不抓帧）")
        return 0

    try:
        scene, grid, slot_mm, pad_mm, zcal, chain = load_ctx(args)
    except EntryStop as exc:
        print("!! " + exc.why)
        return exc.rc
    os.makedirs(args.run_dir, exist_ok=True)
    board = pr.Board(s2.BOARD_ALIAS, args.run_dir)
    print(protocol(args.mode))
    print(">>> 需要人动手：确认急停就绪、两指行程内无手无遮挡；"
          "执行器在板上还会核对 PLAYING / JTC / robot_mode")
    board.push()
    ctx = (board, chain, scene, grid, slot_mm, pad_mm, zcal)
    fn = {"zcal": mode_zcal, "release": mode_release,
          "occlusion": mode_occlusion, "tolerance": mode_tolerance}[args.mode]
    return fn(args, ctx)


if __name__ == "__main__":
    sys.exit(main())
