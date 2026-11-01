#!/usr/bin/env python3
"""阶段 3 一轮堆叠编排：抓帧 → 选块 → 逐层「计划 / 执行 / 立即测量」→ 终态 → 判分。

与阶段 2 的关键差别
──────────────────
1. **每层重新规划**：起点必须取当次实测关节角（`board.joints()`）。一帧只能给
   一条计划的起点；第一块放完臂已经不在抓帧位姿，后面几条计划若共用抓帧的
   q_start，执行器闸门会按「别人动过臂」拒掉（阶段 2 实测最大差 56°）。
2. **放置面每层变**：第 1 层 = 垫面（`grid.json` 的 `T_base_pad[2,3]`），
   第 k≥2 层 = 上一层**实测**顶面（已扣 z 读数偏差）。理论值同时算，差 >3 mm
   就停轮（不静默选一个）。
3. **载运高度**用 `precision_plan.stage3_carry_z`：必须先高过塔顶的放置接近点
   50 mm，再水平移到塔位上方 —— 否则 `to_slot` 会在塔侧横移。
4. **测量只看得见最上层**：每层放好、撤走后**立即**从高度跟踪位姿对抓帧；
   该层指标的唯一来源是这次测量（`metric_source="immediate"`）。终态只复测
   最上层，回答 10 s 稳定性 / 塔是否塌。
5. **测量位姿随塔高上移**：`stage3_survey_tool0` 让相机—塔顶距离恒为 D
   （默认 331 mm，复现阶段 2 的 `stage2close`）。固定 `stage2high` 已知顶面碎、
   yaw 不可信（2026-10-27 B2），固定 `stage2close` 在 5 层时只剩 ~214 mm。

真机纪律（每次会话重来，不继承上次）
────────────────────────────────────
· `--go` 才动臂；执行器自己在板上核对 PLAYING / JTC active / robot_mode 7；
· 夹爪开合（初始化/闭/开）是**另一件事**的当次授权：急停就绪、两指整个行程内
  无手无遮挡；
· 任何一层执行判红 / 测量缺失 / 多视点不一致 ⇒ **停在原地，不续跑**
  （块可能还在爪里或半放），交给人判读；
· 不 push。
"""
import argparse
import json
import math
import os
import subprocess
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
import stage3_score as s3                                          # noqa: E402
import urkin                                                      # noqa: E402

GROUPS = ("open", "closed", "eih")
ARMS = ("geometric", "zero", "graspnet")


# ---------------------------------------------------------------- z 标定
def load_zcal(path):
    """读入口实验产出的 z 标定；缺省两侧都按 0（未标定，只作离线/首轮）。"""
    doc = {"pick": 0.0, "tower": 0.0, "source": None, "model": "constant"}
    if not path:
        return doc
    raw = json.load(open(path))
    site = raw.get("by_site") or {}
    doc["tower"] = float(site.get("tower", raw.get("bias_mm", 0.0)))
    doc["pick"] = float(site.get("pick", raw.get("pick_bias_mm", 0.0)))
    doc.update({"source": os.path.abspath(path),
                "model": raw.get("model", "constant"),
                "residual_mm": raw.get("residual_mm"),
                "pass": raw.get("pass")})
    return doc


# ---------------------------------------------------------------- 计划
def _finish_plan(wps, checks, meta, q_start, chain):
    """与 `precision_plan.main` 完全相同的计划 JSON（执行器只认这一种）。"""
    n_before = len(wps)
    wps, added = pp.split_jumps(wps, chain, q_start=q_start)
    if len(checks) != n_before:
        raise RuntimeError("预检记录不完整：%d 条航点只有 %d 条记录"
                           % (n_before, len(checks)))
    return {"version": 2,
            "meta": dict(meta, checks=checks, splits_added=added,
                         q_start_deg=[round(float(np.degrees(v)), 2)
                                      for v in q_start]),
            "q_start": [float(v) for v in q_start], "waypoints": wps}


def layer_plan(chain, q_start, item, blocks, slot_m, surface_m,
               tower_xy_mm=None, tower_top_mm=None, survey_tool0=None,
               carry_z=None, scene=None, place_open_mm=None,
               release_dwell_s=None, tower_half_edge=0.020,
               pick_open_mm=None, place_clearance=None):
    """一层 = 抓 ``item["block"]`` 放到 ``slot_m`` 的 ``surface_m`` 上。"""
    scene = scene or {}
    block = dict(item["block"])
    # build_pick 用 block["h"] 当 ``wz``；这里换成**扣过拾取侧偏差**的值。
    block["h"] = float(item["block_h_used_mm"])
    wps, checks, meta = pp.build_stack(
        chain, list(q_start), item["cand"], block, blocks, slot_m, surface_m,
        tower_xy_mm=tower_xy_mm, tower_top_mm=tower_top_mm,
        survey_tool0=survey_tool0, carry_z=carry_z,
        tower_half_edge=float(tower_half_edge),
        yaw_grasp_deg=item["yaw_grasp_deg"],
        yaw_place_deg=item["yaw_place_deg"],
        yaw_source=item["yaw_source"],
        pick_open_mm=(pick_open_mm if pick_open_mm is not None
                      else scene.get("pick_open_mm")),
        place_open_mm=(place_open_mm if place_open_mm is not None
                       else scene.get("place_open_mm")),
        grasp_at=scene.get("grasp_at", "block_center"),
        grip_speed_pct=scene.get("grasp_close_speed_pct"),
        release_speed_pct=scene.get("release_open_speed_pct"),
        release_dwell_s=(release_dwell_s if release_dwell_s is not None
                         else scene.get("release_dwell_s", 0.0)),
        place_clearance=place_clearance)
    return _finish_plan(wps, checks, meta, q_start, chain)


def move_plan(chain, q_start, xyz, name="move"):
    wps, checks, meta = pp.build_move(chain, q_start, xyz, name=name)
    return _finish_plan(wps, checks, meta, q_start, chain)


def nudge_plan(chain, q_start, block, blocks, shift_xy_mm=(0.0, 0.0),
               dyaw_deg=0.0, surface_m=pp.TABLE_PLANE_Z, yaw_block_deg=0.0,
               open_mm=None, survey_tool0=None, tower_box=None,
               held_half_edge=0.020):
    """塔顶「只夹不抬」的放置后矫正计划（契约 §3 第 5 项）。

    与 `layer_plan` 同一个收尾（`_finish_plan`），执行器只认这一种 JSON。
    ``block`` 与 `build_nudge` 同形（``cx``/``cy`` mm、``h`` mm 相对脚下那层）；
    ``yaw_block_deg`` 是**当前 tool0 yaw**（不是实测块 yaw，换算在调用方）。
    """
    wps, checks, meta = pp.build_nudge(
        chain, q_start, block, shift_xy_mm=shift_xy_mm, dyaw_deg=dyaw_deg,
        surface_z=surface_m, yaw_block_deg=yaw_block_deg, blocks=blocks,
        open_mm=open_mm, survey_tool0=survey_tool0, tower_box=tower_box,
        held_half_edge=held_half_edge)
    return _finish_plan(wps, checks, meta, q_start, chain)


def predicted_place(item, slot_m):
    """放置瞬间块心的预测（base，m）：``slot + R(Δyaw)·抓取偏心``。

    ``grasp_at="block_center"`` 时偏心为 0（阶段 3 默认），预测就等于槽心 ——
    这样「释放扰动」量的是纯释放滑移，不掺抓取偏心。
    """
    block = item["block"]
    block_h = float(item["block_h_used_mm"])
    cand = item["cand"]
    if item.get("grasp_at") == "block_center" or cand is None:
        gx, gy = block["cx"] / 1000.0, block["cy"] / 1000.0
    else:
        gx, gy = float(cand["xyz_base"][0]), float(cand["xyz_base"][1])
    ex = block["cx"] / 1000.0 - gx
    ey = block["cy"] / 1000.0 - gy
    dyaw = math.radians(float(item["yaw_place_deg"])
                        - float(pp.fold_yaw_180(item["yaw_grasp_deg"])))
    return [round(slot_m[0] + ex * math.cos(dyaw) - ey * math.sin(dyaw), 5),
            round(slot_m[1] + ex * math.sin(dyaw) + ey * math.cos(dyaw), 5)], block_h


def layer_surface_mm(layer, prev, pad_mm):
    """第 1 层 = 垫面；第 k≥2 层 = 上一层**校正顶高**（= 本层脚下的面）。

    别用 ``surface_est_mm``：那是上一层块**脚下**的面，比本层要放的面低
    整整一块（29.5 mm）—— 拿它当表面就是「第二层按垫面高度放」。
    """
    if int(layer) <= 1:
        return float(pad_mm)
    nxt = (prev or {}).get("next_surface_est_mm")
    if nxt is None:
        raise RuntimeError("第 %d 层没有上一层的校正顶高" % layer)
    return float(nxt)


# ---------------------------------------------------------------- 选块
def build_items(analysis, scene, n, arm="geometric", zcal=None):
    """候选池 → 逐层要抓的块（按候选离块心最近排序，不按颜色）。"""
    zcal = zcal or {"pick": 0.0}
    pool = s2.candidate_pool(analysis, scene["pick_zone"])
    chosen, missing, unknown = s2.choose_blocks(pool, order=None, n=n)
    if len(chosen) < n:
        raise RuntimeError("候选池只够 %d 块（要 %d）—— 缺 %s，未定色 %d 块"
                           % (len(chosen), n, missing or "无", len(unknown)))
    items = []
    for k, entry in enumerate(chosen, 1):
        block, cand = entry["block"], entry["best"]
        note = ""
        if arm == "graspnet":
            cand, note = s2.choose_graspnet_cand(entry, block, scene)
            if cand is None:
                raise RuntimeError("块 #%d 没有贴块边的 candidate：%s"
                                   % (block["idx"], note))
        yg, yp, src, why = s2.yaw_for_arm(arm, block, cand, scene)
        h_raw = block.get("h")
        if h_raw is None:
            h_raw = block.get("top")
        items.append({
            "order": k, "block_idx": int(block["idx"]),
            "class": block.get("class"), "arm": arm,
            "block": block, "cand": cand,
            "grasp_at": scene.get("grasp_at", "block_center"),
            "block_h_raw_mm": round(float(h_raw), 3),
            "block_h_used_mm": round(float(h_raw) - float(zcal["pick"]), 3),
            "yaw_grasp_deg": round(float(yg), 3),
            "yaw_place_deg": round(float(yp), 3),
            "yaw_source": src, "yaw_note": note or why,
        })
    return items


# ---------------------------------------------------------------- 板侧动作
def _dump(path, obj):
    json.dump(obj, open(path, "w"), ensure_ascii=False, indent=1)
    return path


def write_plan(path, plan):
    return _dump(path, plan)


def capture_frames(board, out_prefix, tag, n):
    """同一姿态连拍 ``n`` 帧，返回帧目录列表。"""
    dirs = []
    for j in range(max(1, int(n))):
        fd = out_prefix if j == 0 else "%s-%s" % (out_prefix, "bcdefgh"[j - 1])
        board.capture(fd, "%s-%d" % (tag, j + 1))
        dirs.append(fd)
    return dirs


def go_move(board, chain, cdir, tag, xyz, urdf):
    """从当次实测关节角走到 ``xyz``（只动臂）。"""
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q = s2.chain_q(js)
    plan = move_plan(chain, q, xyz, name="survey_" + tag)
    path = write_plan(os.path.join(cdir, "move-%s.json" % tag), plan)
    return board.exec_plan(path, os.path.join(cdir, "move-%s.jsonl" % tag),
                           go=True)


def run_round(args):
    try:
        scene = s2.load_scene(args.scene)
    except ValueError as exc:
        print("场景坏：%s" % exc)
        return 2
    grid_path = args.grid or scene.get("grid_json")
    if not grid_path:
        print("缺 grid.json —— 阶段 3 的塔基、垫面、格线方向都要它")
        return 2
    try:
        grid = s2.load_grid(grid_path)
        s2.check_grid_order(scene, grid)
    except ValueError as exc:
        print("网格坏：%s" % exc)
        return 2
    slot_mm = m3.grid_center(grid, args.tower_cell_idx)
    pad_mm = m3.pad_z_mm(grid)
    grid_yaw = float(grid.get("grid_yaw_base_deg") or 0.0)
    zcal = load_zcal(args.zcal)
    chain = urkin.Chain(args.urdf)

    os.makedirs(args.run_dir, exist_ok=True)
    rdir = args.run_dir
    round_no = args.round
    if round_no is None:
        if args.measure_only:
            have = [int(d[6:]) for d in os.listdir(rdir)
                    if d.startswith("round-") and d[6:].isdigit()
                    and os.path.exists(os.path.join(rdir, d, "round.json"))]
            round_no = max(have) if have else 1
        else:
            have = [int(d[6:]) for d in os.listdir(rdir)
                    if d.startswith("round-") and d[6:].isdigit()]
            round_no = len(have) + 1
    cdir = os.path.join(rdir, "round-%02d" % round_no)
    os.makedirs(cdir, exist_ok=True)

    if args.measure_only:
        return measure_only(args, chain, cdir, slot_mm, grid, zcal)

    n_layers = int(args.n_layers)
    if n_layers < 1:
        print("--n-layers 至少 1")
        return 2
    board = pr.Board(s2.BOARD_ALIAS, rdir)
    if args.go:
        print(">>> 需要人动手：确认急停就绪、两指行程内无手无遮挡；"
              "执行器在板上还会核对 PLAYING / JTC / robot_mode")
        board.push()

    # ---- 抓帧（拾取巡检位）
    extra_frames = list(args.extra_frames or [])
    if args.frame:
        frame_dir = os.path.abspath(args.frame)
    elif args.go:
        rc, frame_dir = s2.do_capture(board, "stage2pick",
                                      os.path.join(cdir, "frame-pick"),
                                      args.urdf, cdir,
                                      "stage3-pick-%02d" % round_no)
        if rc != 0:
            print("走到拾取巡检位失败（rc=%d）—— 不发抓取计划" % rc)
            return rc
        for j in range(1, max(1, int(scene.get("pick_frames", 1)))):
            fd = os.path.join(cdir, "frame-pick-%s" % "bcdefgh"[j - 1])
            board.capture(fd, "stage3-pick-%02d-%d" % (round_no, j + 1))
            extra_frames.append(fd)
    else:
        print("没给 --frame：离线规划需要一帧拾取区图。"
              "先 --frame DIR，或加 --go 由板子抓。")
        return 2

    try:
        analysis = s2.analyse_frame(frame_dir, cdir)
    except (RuntimeError, OSError, ValueError) as exc:
        print("分析失败：%s" % exc)
        return 2
    if extra_frames:
        extras = []
        for fd in extra_frames:
            f = gc.load_frame(fd)
            eb, _tz = gc.find_blocks(f)
            extras.append(s2.add_pixel_yaw(f, eb))
        s2.fuse_block_yaw(analysis["blocks"], extras)

    try:
        items = build_items(analysis, scene, n_layers, arm=args.arm, zcal=zcal)
    except RuntimeError as exc:
        print("选块失败：%s" % exc)
        return 2
    print("轮内块序（按候选离块心）：%s"
          % ["#%d%s" % (it["block_idx"], it["class"] or "")
             for it in items])

    doc = {
        "version": 2, "kind": "stage3-round", "round": round_no,
        "group": args.group, "arm": args.arm, "n_layers": n_layers,
        "grid_json": os.path.abspath(grid_path),
        "tower_cell_idx": int(args.tower_cell_idx),
        "grid_center_base_mm": [round(v, 2) for v in slot_mm],
        "grid_yaw_base_deg": round(grid_yaw, 3),
        "pad_z_mm": round(pad_mm, 3),
        "block_h_mm": m3.BLOCK_TRUE_H_MM,
        "z_bias_mm": zcal,
        "d_mm": round(float(args.d_mm), 2),
        "scene": {k: scene.get(k) for k in
                  ("pick_zone", "grasp_at", "pick_open_mm", "place_open_mm",
                   "release_dwell_s", "yaw_sign", "finger_axis_delta_deg")},
        "items": [{k: it[k] for k in
                   ("order", "block_idx", "class", "arm", "block_h_raw_mm",
                    "block_h_used_mm", "yaw_grasp_deg", "yaw_place_deg",
                    "yaw_source", "yaw_note")} for it in items],
        "layers": [], "terminal": None, "stop_reason": None,
        "human_review": ({"approved": True, "note": args.review_note or ""}
                         if args.review_ok else None),
    }

    prev = None            # 上一层记录（含 merged / surface_est_mm）
    last_pair = None
    prev_q = None
    stop = None
    for k, item in enumerate(items, 1):
        # 每层起点 = 当次实测关节角。阶段 2 的教训：多条计划共用抓帧的
        # q_start，第一块放完后执行器闸门会按「别人动过臂」拒掉（最大差 56°）。
        # 离线预检没有臂：第 1 层读 `--joints`，之后从上一份计划的终点接。
        if args.go:
            js = board.joints(os.path.join(cdir, "joints-layer%02d" % k))
            q_start = s2.chain_q(js)
        elif k == 1:
            if not args.joints:
                print("离线规划要给 --joints <joint_states.json>（起点不能猜）")
                return 2
            q_start = s2.chain_q(json.load(open(args.joints)))
        else:
            q_start = np.asarray(prev_q, dtype=float)
        theory_mm = pad_mm + (k - 1) * m3.BLOCK_TRUE_H_MM
        if args.go:
            surface_mm = layer_surface_mm(k, prev, pad_mm)
            # 理论值同时算：差 >3 mm 说明测量或塔已异常，停轮（不静默选一个）。
            if abs(surface_mm - theory_mm) > 3.0:
                stop = ("surface_theory_mismatch: survey %.1f vs theory %.1f"
                        % (surface_mm, theory_mm))
                print("!! " + stop)
                break
        else:
            # 离线预检没有实测，按理论面逐层规划 —— 只验证运动学/护栏，
            # 数值不进判分（stop_reason=plan_only）。
            surface_mm = theory_mm
        # 物理塔心：下一层的相机要对着它；第 1 层就是格心。
        if prev and prev["merged"].get("found"):
            view_xy = prev["merged"]["center_base_mm"]
        else:
            view_xy = slot_mm
        # 放置目标：主组固定格心；闭环组用上一层实测中心 / yaw。
        slot_target = [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0]
        yaw_place = float(item["yaw_place_deg"])
        if args.group == "closed" and prev and prev["merged"].get("found"):
            lm = prev["merged"]
            slot_target = [lm["center_base_mm"][0] / 1000.0,
                           lm["center_base_mm"][1] / 1000.0]
            yaw_place = float(lm["yaw_deg"])
        item = dict(item, yaw_place_deg=yaw_place)
        surface_m = surface_mm / 1000.0
        survey_top_m = surface_m + m3.BLOCK_TRUE_H_MM / 1000.0
        pair = pp.stage3_survey_tool0(
            survey_top_m, [view_xy[0] / 1000.0, view_xy[1] / 1000.0],
            distance=float(args.d_mm) / 1000.0)
        carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0,
                                    surface_m)
        tower_xy = (prev["merged"]["center_base_mm"]
                    if prev and prev["merged"].get("found") else None)
        tower_top = (prev["merged"]["top_mm"]
                     if prev and prev["merged"].get("found") else None)
        if not args.go and k >= 2:
            # 离线预检没有实测：用**理论塔**开护栏，否则这一段等于没验。
            tower_xy, tower_top = list(slot_mm), surface_mm
        try:
            plan = layer_plan(
                chain, q_start, item, analysis["blocks"], slot_target,
                surface_m, tower_xy_mm=tower_xy, tower_top_mm=tower_top,
                survey_tool0=pair[0], carry_z=carry_z, scene=scene,
                place_open_mm=args.place_open_mm,
                release_dwell_s=args.release_dwell_s,
                tower_half_edge=args.tower_half_edge)
        except (pp.PlanError, RuntimeError, ValueError) as exc:
            stop = "plan_failed: %s" % exc
            print("!! 第 %d 层计划失败：%s" % (k, exc))
            break
        plan_path = write_plan(os.path.join(cdir, "plan-layer%02d.json" % k),
                               plan)
        meta = plan["meta"]
        pred_xy, _bh = predicted_place(item, slot_target)
        rec = {
            "layer": k, "block_idx": item["block_idx"],
            "class": item.get("class"), "arm": item["arm"],
            "yaw_source": item["yaw_source"],
            "block_h_used_mm": item["block_h_used_mm"],
            "exec_rc": None, "plan": os.path.relpath(plan_path, cdir),
            "planned": {
                "surface_z_mm": round(surface_mm, 2),
                "surface_theory_mm": round(theory_mm, 2),
                "place_z_mm": round(float(meta["place_z"]) * 1000.0, 2),
                "carry_z_mm": round(float(meta["carry_z"]) * 1000.0, 2),
                "survey_tool0": meta["survey_tool0"],
                "tower_box": meta.get("tower_box"),
                "yaw_place_deg": round(yaw_place, 3),
                "predicted_center_base_mm": [round(v * 1000.0, 2)
                                             for v in pred_xy],
                # 松手瞬间的块顶 = 表面 + 5 mm 放置间隙 + 真高。
                "predicted_top_mm": round(
                    surface_mm + pp.PLACE_CLEARANCE * 1000.0
                    + m3.BLOCK_TRUE_H_MM, 2),
            },
            "measured": None,
        }
        doc["layers"].append(rec)
        _dump(os.path.join(cdir, "round.json"), doc)
        print("第 %d 层：块 #%d %s → (%.1f, %.1f) mm，表面 %.2f mm，"
              "carry %.1f mm"
              % (k, item["block_idx"], item.get("class") or "?",
                 slot_target[0] * 1000.0, slot_target[1] * 1000.0,
                 surface_mm, float(meta["carry_z"]) * 1000.0))

        if not args.go:
            print("只规划不执行（没加 --go）。计划 → %s" % plan_path)
            prev_q = plan["waypoints"][-1]["q"]
            continue

        rc = board.exec_plan(plan_path,
                             os.path.join(cdir, "log-layer%02d.jsonl" % k),
                             go=True)
        rec["exec_rc"] = rc
        _dump(os.path.join(cdir, "round.json"), doc)
        if rc != 0:
            stop = "exec_rc=%d" % rc
            print("!! 第 %d 层执行判红（rc=%d）—— 停在原地，不续跑" % (k, rc))
            break

        # ---- 立即测量：计划最后一条航点已经停在 survey_place（pair[0]）
        dirs1 = capture_frames(board, os.path.join(cdir, "frame-layer%02d-pose1" % k),
                               "stage3-r%02d-l%02d-p1" % (round_no, k),
                               scene.get("terminal_frames", 3))
        rc2 = go_move(board, chain, cdir, "layer%02d-pose2" % k, pair[1],
                      args.urdf)
        if rc2 != 0:
            stop = "survey_pose2_rc=%d" % rc2
            break
        dirs2 = capture_frames(board, os.path.join(cdir, "frame-layer%02d-pose2" % k),
                               "stage3-r%02d-l%02d-p2" % (round_no, k),
                               scene.get("terminal_frames", 3))
        meas = m3.measure_layer(
            [("pose1", dirs1), ("pose2", dirs2)],
            grid=grid, window_xy_mm=view_xy, layer=k,
            z_bias_mm=zcal["tower"],
            block_h_mm=m3.BLOCK_TRUE_H_MM,
            radius_mm=m3.TOWER_RADIUS_MM)
        rec["measured"] = meas
        _dump(os.path.join(cdir, "round.json"), doc)
        merged = meas.get("merged") or {}
        print("  测量：top=%s mm  center=%s mm  yaw=%s°  "
              "一致性=%s"
              % (merged.get("top_mm"), merged.get("center_base_mm"),
                 merged.get("yaw_deg"),
                 (meas.get("consistency") or {}).get("reason")))
        if not merged.get("found"):
            stop = "measured_missing"
            break
        if (meas.get("consistency") or {}).get("ok") is not True:
            stop = "pose_mismatch"
            break
        prev = {"merged": merged,
                "surface_est_mm": meas.get("surface_est_mm"),
                "next_surface_est_mm": meas.get("next_surface_est_mm")}
        last_pair = pair
        _dump(os.path.join(cdir, "round.json"), doc)

    # ---- 离线预检到此为止：只验证计划能不能生成，不判分（没有测量）。
    if not args.go:
        doc["stop_reason"] = "plan_only"
        _dump(os.path.join(cdir, "round.json"), doc)
        print("离线预检：%d 层计划全部生成（未执行、未判分）" % len(doc["layers"]))
        return 0

    # ---- 终态：静置后复测最上层（只回答顶层有没有动 / 塔是否塌）
    if stop is None and last_pair is not None:
        wait_s = float(args.terminal_wait_s)
        print("终态：静置 %.1f s 后复测最上层" % wait_s)
        time.sleep(wait_s)
        rc = go_move(board, chain, cdir, "terminal-pose1", last_pair[0],
                     args.urdf)
        dirs1 = dirs2 = None
        if rc == 0:
            dirs1 = capture_frames(
                board, os.path.join(cdir, "frame-terminal-pose1"),
                "stage3-r%02d-term-p1" % round_no,
                scene.get("terminal_frames", 3))
            rc = go_move(board, chain, cdir, "terminal-pose2", last_pair[1],
                         args.urdf)
        if rc == 0:
            dirs2 = capture_frames(
                board, os.path.join(cdir, "frame-terminal-pose2"),
                "stage3-r%02d-term-p2" % round_no,
                scene.get("terminal_frames", 3))
            term = m3.measure_layer(
                [("pose1", dirs1), ("pose2", dirs2)],
                grid=grid,
                window_xy_mm=(prev or {}).get("merged", {}).get(
                    "center_base_mm") or slot_mm,
                layer=n_layers, z_bias_mm=zcal["tower"],
                block_h_mm=m3.BLOCK_TRUE_H_MM, metric_source="terminal")
            doc["terminal"] = {"layer": n_layers, "wait_s": wait_s,
                               "measured": term}
        else:
            stop = "terminal_move_rc=%d" % rc
    else:
        if stop is None:
            stop = "no_layers_executed"

    doc["stop_reason"] = stop
    _dump(os.path.join(cdir, "round.json"), doc)

    res = s3.score_round(doc, thresholds=scene.get("stage3_thresholds"))
    paths = s3.write_outputs(res, os.path.join(cdir, "score"))
    print("\n判分：verdict=%s（%s）" % (res["verdict"], res["reason"] or "-"))
    print("  数据 → %s" % paths["json"])
    if stop:
        print("!! 本轮停在：%s —— 交给人判读后再决定；**不自动续跑**" % stop)
    return s3.exit_code(res)


def measure_only(args, chain, cdir, slot_mm, grid, zcal):
    """只做终态复测/判分（不抓不放）。某层判红停在原地后补测量用。"""
    path = os.path.join(cdir, "round.json")
    if not os.path.exists(path):
        print("--measure-only 要读 %s" % path)
        return 2
    doc = json.load(open(path))
    layers = doc.get("layers") or []
    if not layers:
        print("!! round.json 里没有层记录 —— 没有可复测的东西")
        return 2
    last = layers[-1]
    m = (last.get("measured") or {}).get("merged") or {}
    if not m.get("found"):
        print("!! 最上层没有 immediate 测量 —— 终态也复测不了（契约：不赌终态能补）")
        return 2
    if not args.go:
        print("--measure-only 也会动臂（走到两个高度跟踪位姿）。"
              "确认现场后加 --go；**每次都要当次授权**。")
        return 2
    board = pr.Board(s2.BOARD_ALIAS, os.path.dirname(cdir))
    board.push()
    view_xy = m["center_base_mm"]
    top_m = float(m["top_mm"]) / 1000.0
    pair = pp.stage3_survey_tool0(top_m,
                                  [view_xy[0] / 1000.0, view_xy[1] / 1000.0],
                                  distance=float(args.d_mm) / 1000.0)
    rc = go_move(board, chain, cdir, "terminal-pose1", pair[0], args.urdf)
    if rc != 0:
        return rc
    dirs1 = capture_frames(board, os.path.join(cdir, "frame-terminal-pose1"),
                           "stage3-mo-p1", 3)
    rc = go_move(board, chain, cdir, "terminal-pose2", pair[1], args.urdf)
    if rc != 0:
        return rc
    dirs2 = capture_frames(board, os.path.join(cdir, "frame-terminal-pose2"),
                           "stage3-mo-p2", 3)
    doc["terminal"] = {"layer": int(last["layer"]),
                       "wait_s": float(args.terminal_wait_s),
                       "measured": m3.measure_layer(
                           [("pose1", dirs1), ("pose2", dirs2)],
                           grid=grid, window_xy_mm=view_xy,
                           layer=int(last["layer"]),
                           z_bias_mm=zcal["tower"],
                           block_h_mm=m3.BLOCK_TRUE_H_MM,
                           metric_source="terminal")}
    _dump(path, doc)
    res = s3.score_round(doc)
    paths = s3.write_outputs(res, os.path.join(cdir, "score"))
    print("判分：verdict=%s（%s）→ %s"
          % (res["verdict"], res["reason"] or "-", paths["json"]))
    return s3.exit_code(res)


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 3 一轮堆叠")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None, help="覆盖 scene.grid_json")
    ap.add_argument("--frame", default=None, help="复用已抓的拾取区帧")
    ap.add_argument("--joints", default=None,
                    help="离线规划（不加 --go）时的起点 joint_states.json；"
                         "第 1 层用它，之后从上一层计划终点接")
    ap.add_argument("--extra-frame", dest="extra_frames", action="append",
                    default=None, help="同姿态另一帧（yaw 融合），可重复")
    ap.add_argument("--round", type=int, default=None)
    ap.add_argument("--n-layers", type=int, required=True)
    ap.add_argument("--group", choices=GROUPS, default="open")
    ap.add_argument("--arm", choices=ARMS, default="geometric")
    ap.add_argument("--zcal", default=None,
                    help="入口实验产出的 z 标定 JSON（缺省两侧偏差都按 0）")
    ap.add_argument("--d-mm", type=float, default=pp.STAGE3_SURVEY_DEFAULT_D
                    * 1000.0, help="高度跟踪位姿的相机—塔顶距离（mm）")
    ap.add_argument("--tower-cell-idx", type=int, default=m3.TOWER_CELL_IDX)
    ap.add_argument("--tower-half-edge", type=float, default=0.020,
                    help="塔体 box 半边长（m）：块半宽 + 余量")
    ap.add_argument("--place-open-mm", type=float, default=None)
    ap.add_argument("--release-dwell-s", type=float, default=None)
    ap.add_argument("--terminal-wait-s", type=float, default=10.0)
    ap.add_argument("--review-ok", action="store_true",
                    help="人眼复核已通过（逐层数值之外必须由人确认）")
    ap.add_argument("--review-note", default=None)
    ap.add_argument("--measure-only", action="store_true",
                    help="只做终态复测/判分，不抓不放")
    ap.add_argument("--go", action="store_true",
                    help="真跑；**每次都要当次授权**，闸门在 precision_exec")
    args = ap.parse_args(argv)
    return run_round(args)


if __name__ == "__main__":
    sys.exit(main())
