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
6. **放置偏置补偿**：每层量完记 b̂ = 实测 − **命令**；命令目标 = 期望目标 − b̂
   （b̂ = 先验种子 + 本轮已测各层的均值，`--bias-comp none` 退回纯开环）。
   **种子必需**：第 1 层若不补、第 2 层补满，两层之间就有个 ≈|b̂| 的折角，
   而判分看的正是相对对中。种子优先级 `--bias-seed` > `<run>/bias_estimate.json`。
   这个偏置住「块 ↔ 夹爪」之间，闭环组也消不掉；超 15 mm 按失败停轮。

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

#: 真正实现了的组。``eih``（eye-in-hand 第二对照组）只占了个名字：它要把一层
#: 拆成「抓 → 载运」与「载运 → 放」两段、在 carry 处量一次塔顶再改目标，现在
#: 没有任何一段。选它只会**跑成开环**、却把 round.json 的 ``group`` 记成 ``eih``
#: —— 一份贴着错标签的结果比没跑更坏（入口③ 刚踩过「绿了但测的不是那件事」）。
#: 因此**拒绝执行**，而不是静默降级。
IMPLEMENTED_GROUPS = ("open", "closed")


def unsupported_group(group):
    """``group`` 还没实现时返回拒绝原因；实现了返回 ``None``。"""
    if group in IMPLEMENTED_GROUPS:
        return None
    return ("--group %s 还没有实现：eih 组要把一层拆成「抓 → 载运」与"
            "「载运 → 放」两段、在 carry 处按期望顶高挑出塔顶再改目标。"
            "现在选它只会跑成开环组、却把结果记成 eih —— fail-closed，拒绝执行。"
            % group)


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


def closed_loop_yaw(prev_merged):
    """闭环组：上一层实测塔顶 yaw → ``(yaw_deg, 来源)``；判不了给 ``(None, 原因)``。

    **必须和判分同源**（契约 §4.2.5，走 `m3.metric_yaw`，即彩色掩码那一路）。
    拿深度 ``yaw_deg`` 来闭环 = 把 2–5° 的估计器噪声主动拧进塔里：判分那边已经
    改成彩色，控制这边再用深度，两层之间就隔着一个看不出来的朝向偏差。
    彩色判不了 ⇒ 返回 ``None``，调用方**停轮**，不偷偷顶替。
    """
    return m3.metric_yaw(prev_merged or {})


# ---------------------------------------------------------------- 放置偏置补偿
#: 偏置上限（mm）。超过它**不是偏置，是失败**（块掉了 / 没抓住 / 塔被撞了），
#: 按失败停轮，不用它去推下一层 —— 拿一个 30 mm 的「偏置」补偿下一层，
#: 只会把塔越推越歪，而且事后从数据上看不出是哪一步坏的。
BIAS_LIMIT_MM = 15.0


def bias_from_layer(measured_mm, commanded_mm, limit_mm=BIAS_LIMIT_MM):
    """一层放完后：``实测块顶中心 − 命令目标`` = 这一层的放置偏置 b̂。

    返回 ``(bias, why)``：正常时 ``bias=[bx, by]``（mm）、``why=None``；
    判不了时 ``bias=None``、``why`` 是原因（``measured_missing`` / 超上限）。

    **为什么必须有这一项**：机放的块落地后系统性偏向 base 的 (−x, −y) ——
    入口② open50/dwell0.5 五次均值 (−3.56, −2.56) mm、|dxy| σ 0.84 mm；今天三次
    机放对格心 (−3.4, +0.6)/(−2.8, +0.8)/(−0.9, +0.4)（方位角 170°/164°/156°，稳）。
    这个 offset 住**块与夹爪之间**，不在「目标 → 臂」这条链上 ⇒ 闭环组把下一层
    目标改成「上一层实测中心」也消不掉它。只有显式预补偿（命令 = 期望 − b̂）能压。
    """
    if measured_mm is None or commanded_mm is None:
        return None, "measured_missing"
    bx = float(measured_mm[0]) - float(commanded_mm[0])
    by = float(measured_mm[1]) - float(commanded_mm[1])
    mag = math.hypot(bx, by)
    if not math.isfinite(mag):
        return None, "bias_nonfinite"
    if mag > float(limit_mm):
        return None, ("bias_too_large: 实测比命令偏 %.2f mm（上限 %.1f）"
                      "—— 这不是偏置，是失败" % (mag, limit_mm))
    return [bx, by], None


def applied_bias(bias_hist, mode="adaptive", seed=None):
    """这一层要用的 b̂（mm）：``种子 + 本轮已测各层`` 的均值。

    ``none`` ⇒ ``[0, 0]``（纯开环）。**种子是必需的，不是优化**：第 1 层没有
    本轮历史，不补；第 2 层补满 b̂ ⇒ 第 1↔2 层之间凭空多出一个 ≈|b̂| 的**折角**。
    判分看的是**相对**对中，4 mm 的折角直接把「≤5 mm」吃掉 —— 而物理上
    「相对第 1 层放准第 2 层」本来就要求先知道 b̂，轮内数据给不了，只能给先验。
    """
    if mode != "adaptive":
        return [0.0, 0.0]
    vals = ([list(seed)] if seed else []) + list(bias_hist or [])
    if not vals:
        return [0.0, 0.0]
    n = float(len(vals))
    return [sum(float(b[0]) for b in vals) / n,
            sum(float(b[1]) for b in vals) / n]


def target_with_bias(desired_mm, bias_hist, mode="adaptive", seed=None):
    """``期望目标 − b̂``：命令给臂的目标。``desired`` 仍是要落到的那个点。"""
    b = applied_bias(bias_hist, mode, seed=seed)
    return [float(desired_mm[0]) - b[0], float(desired_mm[1]) - b[1]]


def commanded_target_m(desired_m, bias_hist, mode="adaptive", seed=None):
    """米制包装：臂与计划用 m，b̂ 用 mm —— 换算只在这一行，并由测试把守。"""
    t = target_with_bias([float(v) * 1000.0 for v in desired_m],
                         bias_hist, mode, seed=seed)
    return [t[0] / 1000.0, t[1] / 1000.0]


#: 轮间累积的偏置估计（写在 run 目录下）。下一轮默认拿它当种子 ——
#: 不这样做，每轮第 1 层都只能不补，每轮都会重新造一次那个折角。
BIAS_ESTIMATE_FILE = "bias_estimate.json"


def parse_bias_seed(spec):
    """``"bx,by"``（mm，允许中文逗号）→ ``[bx, by]``；空 ⇒ ``None``。"""
    if spec is None:
        return None
    text = str(spec).strip().replace("，", ",")
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise ValueError("--bias-seed 要两个数（bx,by），实得 %r" % (spec,))
    try:
        return [float(parts[0]), float(parts[1])]
    except ValueError:
        raise ValueError("--bias-seed 不是数：%r" % (spec,))


def load_bias_estimate(path):
    """读轮间估计；坏文件当没有（**不猜**，但也不让一次坏文件炸掉整轮）。"""
    try:
        doc = json.load(open(path))
    except (OSError, ValueError):
        return None
    val = doc.get("bias_mm")
    if not val or len(val) != 2:
        return None
    try:
        return [float(val[0]), float(val[1])]
    except (TypeError, ValueError):
        return None


def write_bias_estimate(rdir, doc):
    """把本轮的 b̂ 并进 ``<run>/bias_estimate.json``：下一轮第 1 层就有先验。

    没有测量（离线预检 / 第一层就停）⇒ 不写，返回 ``None``。
    """
    hist = [h["bias_mm"] for h in (doc.get("bias_history") or [])]
    if not hist:
        return None
    path = os.path.join(rdir, BIAS_ESTIMATE_FILE)
    try:
        prev = json.load(open(path))
    except (OSError, ValueError):
        prev = None
    merged = merge_bias_estimate(prev, hist)
    if merged is None:
        return None
    n = float(len(hist))
    merged["rounds"] = list((prev or {}).get("rounds") or []) + [
        {"round": doc.get("round"), "n": len(hist),
         "bias_mm": [round(sum(float(h[0]) for h in hist) / n, 3),
                     round(sum(float(h[1]) for h in hist) / n, 3)]}]
    with open(path, "w") as fh:
        json.dump(merged, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    return merged


def merge_bias_estimate(prev, new_layers):
    """把本轮的 b̂ 并进轮间估计：``n`` 加权平均，返回落盘用的 dict。"""
    prev = prev or {}
    n0 = int(prev.get("n") or 0)
    vals = [list(map(float, prev.get("bias_mm") or [0.0, 0.0]))] * n0
    vals += [list(map(float, b)) for b in (new_layers or [])]
    if not vals:
        return None
    n = len(vals)
    return {"version": 1, "model": "constant",
            "bias_mm": [round(sum(v[0] for v in vals) / n, 3),
                        round(sum(v[1] for v in vals) / n, 3)],
            "n": n,
            "rounds": list(prev.get("rounds") or [])}


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
        # 阶段 3 的块是 29.5 mm 正方体（四面对称）⇒ 抓/放角按 90° 折，
        # 每个角落在 (−45°, 45°] 内，腕的总行程随之最小。见 pp.fold_yaw_90。
        yg, yp = pp.fold_yaw_90(yg), pp.fold_yaw_90(yp)
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
    why = unsupported_group(args.group)
    if why:                      # 放在最前：任何动臂动作之前就拒
        print("!! " + why)
        return 2
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
    # 偏置先验：命令行 > run 目录里上一轮攒下的估计 > 无。
    # 没有它，每轮第 1 层都只能不补，第 1↔2 层之间就有一个 ≈|b̂| 的折角。
    try:
        b_seed = parse_bias_seed(getattr(args, "bias_seed", None))
    except ValueError as exc:
        print("!! %s" % exc)
        return 2
    b_seed_src = "arg" if b_seed else None
    if b_seed is None:
        b_seed = load_bias_estimate(os.path.join(rdir, BIAS_ESTIMATE_FILE))
        b_seed_src = "estimate" if b_seed else None
    if b_seed and args.bias_comp == "adaptive":
        print("偏置先验 b̂₀=(%+.2f, %+.2f) mm（来源：%s）—— 第 1 层也补偿，"
              "避免 1↔2 层之间那个 ≈|b̂| 的折角"
              % (b_seed[0], b_seed[1],
                 "命令行 --bias-seed" if b_seed_src == "arg"
                 else "run 目录上一轮的估计"))
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
        # 轮内自适应放置偏置补偿：mode=none 时逐字节复现旧的「纯开环」语义。
        "bias_comp": {"mode": args.bias_comp, "limit_mm": BIAS_LIMIT_MM,
                      "seed_mm": (None if b_seed is None
                                  else [round(v, 3) for v in b_seed]),
                      "seed_source": b_seed_src},
        "bias_history": [],
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
        # **期望目标**（要落到的点，判分与报告看它）与**命令目标**（给臂的点）
        # 从这一层起分开：命令 = 期望 − b̂，b̂ = 先验种子 + 本轮已测各层的均值。
        # b̂ 住「块 ↔ 夹爪」之间，闭环也消不掉，只能这样显式预补偿。
        desired_target = [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0]
        yaw_place = float(item["yaw_place_deg"])
        yaw_place_src = "plan"          # 主组：目标就是计划里的块朝向
        if args.group == "closed" and prev and prev["merged"].get("found"):
            lm = prev["merged"]
            desired_target = [lm["center_base_mm"][0] / 1000.0,
                              lm["center_base_mm"][1] / 1000.0]
            pyaw, pyaw_src = closed_loop_yaw(lm)
            if pyaw is None:
                stop = "closed_yaw_%s（第 %d 层，缺可用的彩色 yaw）" % (
                    pyaw_src, k)
                print("!! " + stop)
                break
            yaw_place = pyaw
            yaw_place_src = pyaw_src
        item = dict(item, yaw_place_deg=yaw_place)
        b_applied = applied_bias(doc["bias_history"], args.bias_comp,
                                 seed=b_seed)
        slot_target = commanded_target_m(desired_target, doc["bias_history"],
                                         args.bias_comp, seed=b_seed)
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
                # 闭环目标来自哪一路 yaw —— 事后能解释每一层的朝向是怎么来的。
                "yaw_place_source": yaw_place_src,
                # 期望（该落在哪）/ 命令（实际让臂去哪）/ 本层用的 b̂。
                # 三者分开记，事后才能分辨「补偿生效」与「塔本来就放歪了」。
                "desired_center_base_mm": [round(v * 1000.0, 2)
                                           for v in desired_target],
                "commanded_center_base_mm": [round(v * 1000.0, 2)
                                             for v in slot_target],
                "applied_bias_mm": [round(v, 3) for v in b_applied],
                "predicted_center_base_mm": [round(v * 1000.0, 2)
                                             for v in pred_xy],
                # 松手瞬间的块顶 = 表面 + 5 mm 放置间隙 + 真高。
                "predicted_top_mm": round(
                    surface_mm + pp.PLACE_CLEARANCE * 1000.0
                    + m3.BLOCK_TRUE_H_MM, 2),
            },
            "measured": None,
            # 本层「实测 − 命令」；下一层的预补偿就用它的历史均值。
            "placement_bias_mm": None,
        }
        doc["layers"].append(rec)
        _dump(os.path.join(cdir, "round.json"), doc)
        print("第 %d 层：块 #%d %s → 命令 (%.1f, %.1f) mm%s，表面 %.2f mm，"
              "carry %.1f mm"
              % (k, item["block_idx"], item.get("class") or "?",
                 slot_target[0] * 1000.0, slot_target[1] * 1000.0,
                 ("（期望 %.1f, %.1f，b̂=(%+.2f, %+.2f)）"
                  % (desired_target[0] * 1000.0, desired_target[1] * 1000.0,
                     b_applied[0], b_applied[1])
                  if any(b_applied) else ""),
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
        # 这一层落地偏了多少 —— 下一层用它预补偿。超上限按失败停轮，
        # 不拿一个「块掉了 / 没抓住」的大数去把下一层推得更歪。
        bias, bwhy = bias_from_layer(
            merged.get("center_base_mm"),
            [slot_target[0] * 1000.0, slot_target[1] * 1000.0])
        rec["placement_bias_mm"] = (None if bias is None
                                    else [round(v, 3) for v in bias])
        if bwhy:
            stop = "layer%02d_%s" % (k, bwhy)
            print("!! 第 %d 层：%s" % (k, bwhy))
            _dump(os.path.join(cdir, "round.json"), doc)
            break
        doc["bias_history"].append({"layer": k,
                                    "bias_mm": [round(v, 3) for v in bias]})
        print("  放置偏置 b̂=(%+.2f, %+.2f) mm —— %s"
              % (bias[0], bias[1],
                 "下一层按历史均值预补偿" if args.bias_comp == "adaptive"
                 else "补偿关闭（--bias-comp none），只记账"))
        prev = {"merged": merged,
                "surface_est_mm": meas.get("surface_est_mm"),
                "next_surface_est_mm": meas.get("next_surface_est_mm")}
        last_pair = pair
        _dump(os.path.join(cdir, "round.json"), doc)

    # 本轮的 b̂ 并进 run 目录的估计：下一轮（含第 1 层）直接有先验。
    merged_est = write_bias_estimate(rdir, doc)
    if merged_est:
        print("偏置估计更新：b̂=(%+.2f, %+.2f) mm，n=%d → %s"
              % (merged_est["bias_mm"][0], merged_est["bias_mm"][1],
                 merged_est["n"],
                 os.path.join(rdir, BIAS_ESTIMATE_FILE)))

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
    ap.add_argument("--bias-comp", choices=("adaptive", "none"), default="adaptive",
                    help="轮内自适应放置偏置补偿（默认 adaptive）：每层量完把"
                         "「实测 − 命令」记进 bias_history，下一层命令目标 = "
                         "期望目标 − 历史均值。none = 旧的纯开环语义，只记不补。")
    ap.add_argument("--bias-seed", default=None,
                    help="偏置先验 bx,by（mm），第 1 层也补；不给就吃 "
                         "<run-dir>/bias_estimate.json（上一轮攒下的）")
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
