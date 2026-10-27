#!/usr/bin/env python3
"""阶段 2 · 一轮宫格归位的编排（离线可测；``--go`` 才动臂）。

与 A3 的关系
────────────
**不重写 A3**：板侧抓帧/执行、SSH + 文件接缝、闸门都在 ``precision_round.Board``
和 ``precision_exec`` 里，这里只负责「阶段 2 多出来的那几件事」：

1. 场景：颜色格序（``grid.json`` 来自 ``stage2_pad.py``）与**块顺序**（可选）；
2. 选块：按块顺序，或按候选离块心最近的自动选；
3. 三臂：同一轮里块轮流分给 ``geometric / zero / graspnet``，轮次整体偏移；
   每臂只改**腕部 yaw**，抓取点/放置运动完全相同 —— 隔离变量；
4. 两段 yaw：``yaw_grasp`` 按块面（几何臂 / 候选角）对齐，``yaw_place = δ``
   （夹住之后块相对 tool0 的角固定，放置只需转回参考角）；
5. 终态在**整轮结束后**从 ≥2 个 survey 位姿各抓一帧，量每块色/格/yaw/位置，
   交给 ``stage2_score`` 判分。

场景文件（``--scene``，全部可省）
────────────────────────────────
    {
      "grid_json": "…/grid.json",          # 必给（stage2_pad.py 产出）
      "grid_order": ["red", …, "spare"],   # 可选；给了就必须与 grid.json 一致
      "blocks_order": ["red", "cyan", …],  # 可选；空 = 自动选块
      "arms_order": ["geometric", "zero", "graspnet"],
      "yaw_sign": -1.0,                    # 契约 ψ = −θ_face；标定若证明反了改 +1
      "finger_axis_delta_deg": 0.0,        # 指轴标定（B2）的 δ
      "pick_open_mm": 50.0, "place_open_mm": 50.0,
      "blocks_per_round": 6,
      "pick_zone": [0.38, 0.62, 0.25, 0.45],
      "survey_poses": ["place", "place2"],
      "thresholds": {"yaw_deg": 8.0, "pos_mm": 12.0}
    }

命令行快捷方式（优先级低于 ``--scene``）：``--grid-order`` / ``--block-order``，
逗号分隔；``--grid-order`` 与垫子 sidecar 的颜色顺序必须一致，否则直接报错
（颜色顺序与垫子不一致 = 全部色格判错，而且现场看不出来）。

「块 → 测量」怎么配对（判分是否可信的关键）
──────────────────────────────────────────
终态一帧里量到的是**一堆块**，不是「第 k 次抓的那块」。按位置就近配对等于
假设「放对了」，判分就变成自证。这里按 **颜色匹配 + 离预期格最近** 做一对一
贪心：颜色是独立观测量，格是判据本身。颜色对不上的块不配对（items 记
``not_measured``），多出来的块单独列出 —— 宁可报「没量到」，不许把放错的块
当成放对的那块。

两种单品模式
────────────
· ``--capture-pose {pick,place,place2} --capture-out DIR --go``：只走到巡检位
  抓一帧（B1 垫子定位用；不动夹爪、不需要 grid.json）；
· ``--calib``：指轴标定轮 —— 只用几何臂、3 块、δ 先按 0 跑，终态量每块 yaw 偏
  差并写 ``calib.json``（``verdict``：zero / constant_offset / inconsistent）。

退出码：0 = 计划/一轮正常；1 = 有块/有格放不下或执行判红；2 = 数据/配置坏。
"""

import argparse
import json
import math
import os
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import precision_round as pr                                      # noqa: E402
import stage2_color as sc                                         # noqa: E402
import stage2_geom as sg                                          # noqa: E402

PY_SELF = sys.executable
PY_GRASPNET = os.path.expanduser("~/robotics/venvs/graspnet/bin/python")
BOARD_ALIAS = os.environ.get("STAGE2_BOARD", "jetson-hotspot")   # 与 precision_round 一致；有线备选用 STAGE2_BOARD=jetson-wifi

ARMS = ("geometric", "zero", "graspnet")
DEFAULT_SCENE = {
    "grid_json": None,
    "grid_order": None,
    "blocks_order": [],
    "arms_order": list(ARMS),
    "yaw_sign": -1.0,
    "finger_axis_delta_deg": 0.0,
    "pick_open_mm": 50.0,
    "place_open_mm": 50.0,
    "blocks_per_round": 6,
    "pick_zone": list(pr.PICK_ZONE),
    "survey_poses": ["place", "place2"],
    "thresholds": {"yaw_deg": 8.0, "pos_mm": 12.0},
}


# ---------------------------------------------------------------- 场景
def load_scene(path=None, grid_order=None, block_order=None):
    """读场景文件 + 命令行快捷方式；返回补全默认值后的 dict。"""
    scene = json.loads(json.dumps(DEFAULT_SCENE))
    if path:
        try:
            raw = json.load(open(path))
        except (OSError, ValueError) as exc:
            raise ValueError("读不到场景 %s：%s" % (path, exc))
        scene.update({k: v for k, v in raw.items() if v is not None})
    if grid_order:
        cells = [c.strip() for c in grid_order.split(",")]
        if len(cells) != 9:
            raise ValueError("--grid-order 要给 9 个颜色（备用格写 spare），收到 %d 个"
                             % len(cells))
        scene["grid_order"] = cells
    if block_order:
        scene["blocks_order"] = [c.strip() for c in block_order.split(",") if c.strip()]
    return scene


def load_grid(path):
    try:
        grid = json.load(open(path))
    except (OSError, ValueError) as exc:
        raise ValueError("读不到 grid.json %s：%s" % (path, exc))
    if not isinstance(grid.get("cells"), list) or len(grid["cells"]) != 9:
        raise ValueError("grid.json 的 cells 不是 9 个")
    for key in ("T_base_pad", "grid_center_base_mm", "grid_yaw_base_deg"):
        if key not in grid:
            raise ValueError("grid.json 缺 %s（stage2_pad.py 的产物才全）" % key)
    return grid


def check_grid_order(scene, grid):
    """``grid_order`` 必须与垫子/网格一致；不一致 = 全部色格判错，直接拒绝。"""
    want = scene.get("grid_order")
    if not want:
        return
    got = grid.get("grid_order")
    if got and list(got) != list(want):
        raise ValueError("grid_order %s 与 grid.json 的 %s 不一致 —— "
                         "按哪个摆都会有一半格判错；先确认垫子是哪种" % (want, got))
    for c, color in zip(sorted(grid["cells"], key=lambda c: c["idx"]), want):
        if c["color"] != color:
            raise ValueError("格 %d 在 grid.json 里是 %s，配置写的是 %s"
                             % (c["idx"], c["color"], color))


def cell_slots(grid):
    """grid.json → (slots_mm, slot_classes, cells)。备用格 → 空串（通配）。"""
    cells = sorted(grid["cells"], key=lambda c: c["idx"])
    slots = [[float(c["center_base_mm"][0]), float(c["center_base_mm"][1])]
             for c in cells]
    classes = ["" if c["color"] in ("spare", "") else c["color"] for c in cells]
    return slots, classes, cells


# ---------------------------------------------------------------- 选块 / 分配
def in_zone(block, zone):
    x0, x1, y0, y1 = zone
    return (x0 * 1000.0 <= block["cx"] <= x1 * 1000.0
            and y0 * 1000.0 <= block["cy"] <= y1 * 1000.0)


def candidate_pool(analysis, zone, max_cand_mm=pr.MAX_CAND_MM):
    """有合格候选、在拾取区内的块：``[{p, block, best}]``。"""
    blocks = {b["idx"]: b for b in analysis["blocks"]}
    out = []
    for p in analysis.get("per_block", []):
        best = p.get("best")
        if not best or p.get("dist_mm") is None or p["dist_mm"] > max_cand_mm:
            continue
        b = blocks.get(p["block_idx"])
        if b is None or not in_zone(b, zone):
            continue
        out.append({"p": p, "block": b, "best": best})
    return out


def choose_blocks(pool, order=None, n=6):
    """按块顺序挑，或按候选离块心最近挑。

    ``order`` 里的类名用一次消耗一块；同色有多块时取候选最近的那块。
    返回 ``(chosen, missing, skipped)``：找不到的类名进 ``missing``，
    没有颜色判定的块进 ``skipped`` —— 都不静默跳过。
    """
    chosen, missing = [], []
    usable, unknown = [], []
    for x in pool:
        if x["p"]["class"] in (None, "", "unknown"):
            unknown.append(x)
        else:
            usable.append(x)
    if order:
        left = list(usable)
        for cls in order:
            if len(chosen) >= n:
                break
            cands = [x for x in left if x["p"]["class"] == cls]
            if not cands:
                missing.append(cls)
                continue
            pick = min(cands, key=lambda x: x["p"]["dist_mm"])
            chosen.append(pick)
            left.remove(pick)
        return chosen, missing, unknown
    ordered = sorted(usable, key=lambda x: x["p"]["dist_mm"])
    return ordered[:n], missing, unknown


def assign_arm(index, arms, offset=0):
    """第 ``index`` 块（轮内从 0 起）分给哪条臂。"""
    if not arms:
        raise ValueError("arms_order 为空")
    return arms[(int(index) + int(offset)) % len(arms)]


def wrap_deg(a):
    """角度归一到 [0, 360)。"""
    return float(a) % 360.0


def yaw_for_arm(arm, block, cand, scene):
    """一条臂的 (yaw_grasp, yaw_place, source, note)。

    · geometric：θ_face（像素最小外接矩形）→ ψ = sign·θ + δ；放置转回 δ；
    · zero：两段都是 0（基线臂）；
    · graspnet：候选角 + δ；放置 δ。候选宽**不用**（A3 已证明它对不上块边）。
    """
    delta = float(scene.get("finger_axis_delta_deg", 0.0))
    sign = float(scene.get("yaw_sign", -1.0))
    if arm == "geometric":
        theta = block.get("yaw_pix")
        if theta is None:
            theta = block.get("yaw_rect")
        if theta is None:
            return 0.0, delta, "geometric_missing", "块没有几何 yaw"
        if block.get("yaw_pix_trusted") is False:
            return (wrap_deg(sign * float(theta) + delta), delta,
                    "geometric_untrusted", "矩形填充率/像素数不足")
        return (wrap_deg(sign * float(theta) + delta), delta, "geometric", "")
    if arm == "zero":
        return 0.0, 0.0, "zero", ""
    if arm == "graspnet":
        return (wrap_deg(float(cand["angle_deg"]) + delta), delta,
                "graspnet", "")
    raise ValueError("不认识的臂 %r（只认 %s）" % (arm, list(ARMS)))


def cluster_warnings(items, min_mm=60.0):
    """被选块之间中心距 < ``min_mm`` 的对（计划器会拒；这里先提醒）。"""
    out = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i]["block_xy_mm"], items[j]["block_xy_mm"]
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d < min_mm:
                out.append((items[i]["block_idx"], items[j]["block_idx"],
                            round(d, 1)))
    return out


def build_assignments(pool, scene, round_no, grid, order=None):
    """一轮的完整分配：块 → 臂 → 格 → 两段 yaw。

    返回 ``{'items', 'missing', 'skipped', 'stop_reason'}``。
    选格复用 ``slot_routing.pick_slot``（同色优先、备用格通配），**不改它的语义**。
    """
    slots, classes, _cells = cell_slots(grid)
    used = [False] * len(slots)
    n = int(scene.get("blocks_per_round", 6))
    chosen, missing, skipped = choose_blocks(pool, order=order, n=n)
    arms = scene.get("arms_order") or list(ARMS)
    offset = (int(round_no) - 1) % len(arms)
    items, stop = [], None
    for x in chosen:
        cls = x["p"]["class"]
        slot_idx = pr.pick_slot(used, classes, cls)
        if slot_idx is None:
            stop = "类别 %s 没有空槽了" % cls
            break
        arm = assign_arm(len(items), arms, offset)
        yg, yp, src, note = yaw_for_arm(arm, x["block"], x["best"], scene)
        items.append({
            "block_idx": int(x["p"]["block_idx"]), "class": cls,
            "arm": arm, "slot_idx": int(slot_idx), "slot_color": classes[slot_idx],
            "slot_mm": slots[slot_idx],
            "yaw_grasp_deg": round(yg, 3), "yaw_place_deg": round(yp, 3),
            "yaw_source": src, "yaw_note": note,
            "cand_dist_mm": round(float(x["p"]["dist_mm"]), 2),
            "cand_angle_deg": round(float(x["best"]["angle_deg"]), 2),
            "cand_width_mm": round(float(x["best"]["width_mm"]), 2),
            "block_xy_mm": [round(float(x["block"]["cx"]), 2),
                            round(float(x["block"]["cy"]), 2)],
        })
        used[slot_idx] = True
    return {"items": items, "missing": missing, "skipped": skipped,
            "stop_reason": stop}


# ---------------------------------------------------------------- 分析
def add_pixel_yaw(frame, blocks):
    """给每个块补 ``yaw_pix`` / ``yaw_pix_trusted``（像素最小外接矩形）。"""
    for b in blocks:
        try:
            r = sg.block_rect_yaw(frame, b)
        except Exception as exc:                                  # noqa: BLE001
            b["yaw_pix"] = None
            b["yaw_pix_trusted"] = False
            b["yaw_pix_error"] = "%s: %s" % (type(exc).__name__, exc)
            continue
        b["yaw_pix"] = None if r["yaw_deg"] is None else round(r["yaw_deg"], 3)
        b["yaw_pix_trusted"] = bool(r["trusted"])
        b["yaw_pix_ext_mm"] = r["ext_mm"]
        b["yaw_pix_fill"] = round(float(r["fill"]), 3)
    return blocks


def analyse_frame(frame_dir, run_dir, weights=None, yolo_conf=0.25,
                  yolo_imgsz=640):
    """一帧 → ``analysis``（块表 + 候选 + 双源颜色 + 像素 yaw）。

    候选在 graspnet venv 里跑；颜色与几何在系统 python 里跑（分工照旧）。
    返回的 ``_candidates_json`` 是 ``precision_plan --candidates`` 要吃的文件。
    """
    os.makedirs(run_dir, exist_ok=True)
    cand_json = os.path.join(run_dir, "candidates.json")
    r = subprocess.run([PY_GRASPNET,
                        os.path.join(HERE, "graspnet_candidates.py"),
                        frame_dir, "--json", cand_json],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("候选失败（rc=%d）：%s" % (r.returncode,
                                                     r.stderr.strip()[-800:]))
    analysis = json.load(open(cand_json))
    frame = gc.load_frame(frame_dir)

    dets, warn = [], None
    if weights:
        import precision_verify as pv
        dets, warn = pv.yolo_detect(frame_dir, weights, conf=yolo_conf,
                                    imgsz=yolo_imgsz)
    classified = sc.classify_blocks(analysis["blocks"], dets)
    by_idx = {c["block_idx"]: c for c in classified}
    for p in analysis["per_block"]:
        c = by_idx.get(p["block_idx"])
        if c is None:
            continue
        p["class"] = c["class"] if c["class"] else "unknown"
        p["class_status"] = c["status"]
        p["class_source"] = c["source"]
        p["class_detail"] = c
    analysis["blocks"] = add_pixel_yaw(frame, analysis["blocks"])
    analysis["_candidates_json"] = cand_json
    analysis["_yolo_warn"] = warn
    return analysis


# ---------------------------------------------------------------- 终态测量
def _pad_xy(grid, xy_mm):
    """base mm → pad mm（用 T_base_pad 的逆）。"""
    T = np.asarray(grid["T_base_pad"], dtype=np.float64)
    p = np.linalg.inv(T) @ np.array([xy_mm[0] / 1000.0, xy_mm[1] / 1000.0,
                                     0.0, 1.0])
    return [round(float(p[0]) * 1000.0, 2), round(float(p[1]) * 1000.0, 2)]


def measure_pose(frame_dir, grid, weights=None):
    """终态一帧 → 该姿态下每块的 (class, class_status, pad 位置, yaw)。

    只量几何 + 颜色，不做任何「它应该是哪块」的假设；配对交给
    :func:`link_poses`。低于 15 mm 的东西（8 mm 平板等）不算块。
    """
    frame = gc.load_frame(frame_dir)
    blocks, _tz = gc.find_blocks(frame)
    blocks = [b for b in blocks if b.get("h", 0.0) >= 15.0 and not b.get("low")]
    blocks = add_pixel_yaw(frame, blocks)
    dets = []
    if weights:
        import precision_verify as pv
        dets, _warn = pv.yolo_detect(frame_dir, weights)
    classified = {c["block_idx"]: c for c in sc.classify_blocks(blocks, dets)}
    out = []
    for b in blocks:
        c = classified.get(b["idx"], {})
        out.append({
            "class": c.get("class") or "unknown",
            "class_status": c.get("status", "unknown"),
            "center_base_mm": [round(float(b["cx"]), 2),
                               round(float(b["cy"]), 2)],
            "center_pad_mm": _pad_xy(grid, [b["cx"], b["cy"]]),
            "yaw_deg": b.get("yaw_pix"),
            "yaw_trusted": b.get("yaw_pix_trusted"),
            "frame": os.path.basename(os.path.abspath(frame_dir)),
        })
    return out


def match_block_ids(poses, pos_tol_mm=10.0):
    """跨姿态把同一物理块配对成一个 ``block_id``（按 pad 位置最近）。

    ``poses``：``[[block, …], …]``。第一姿态的块各拿一个 id，后续姿态按
    ≤``pos_tol_mm`` 的最近邻配对；配不上的新块**也发新 id**（不硬塞给旧块 ——
    那会把「多出来一块」伪装成「同一块挪了位置」）。
    """
    if not poses:
        return []
    ids_by_pose = [[None] * len(p) for p in poses]
    anchor = []
    for j, b in enumerate(poses[0]):
        ids_by_pose[0][j] = len(anchor)
        anchor.append(b["center_pad_mm"])
    for i in range(1, len(poses)):
        for j, b in enumerate(poses[i]):
            p = b["center_pad_mm"]
            best, best_d = None, pos_tol_mm
            for bid, a in enumerate(anchor):
                d = math.hypot(p[0] - a[0], p[1] - a[1])
                if d <= best_d:
                    best, best_d = bid, d
            if best is None:
                best = len(anchor)
                anchor.append(p)
            ids_by_pose[i][j] = best
    out = []
    for i, p in enumerate(poses):
        out.append([dict(b, block_id=ids_by_pose[i][j])
                    for j, b in enumerate(p)])
    return out


def block_means(matched):
    """跨姿态合并：``{block_id: {class, class_status, center_pad_mm, …}}``。"""
    out = {}
    for pose in matched:
        for b in pose:
            rec = out.setdefault(b["block_id"], {"centers": [], "yaws": [],
                                                 "class": None,
                                                 "class_status": None})
            rec["centers"].append(b["center_pad_mm"])
            if b.get("yaw_deg") is not None:
                rec["yaws"].append(float(b["yaw_deg"]))
            rec["class"] = rec["class"] or b.get("class")
            rec["class_status"] = rec["class_status"] or b.get("class_status")
    for bid, rec in out.items():
        cs = rec["centers"]
        rec["center_pad_mm"] = [round(sum(c[0] for c in cs) / len(cs), 2),
                                round(sum(c[1] for c in cs) / len(cs), 2)]
        rec["yaw_deg"] = (round(sum(rec["yaws"]) / len(rec["yaws"]), 3)
                          if rec["yaws"] else None)
        rec["n_poses"] = len(cs)
    return out


def link_poses(matched, items, grid, class_gate=True):
    """测量块 → 计划里的 item 一对一配对（颜色匹配 + 离预期格最近）。

    返回 ``(poses, unmatched)``：``poses`` 里每块的 ``block_id`` 换成 item 下标
    （``stage2_score.expected`` 的键）；``unmatched`` 是配不上任何 item 的测量块。
    配对只看**颜色与格心距离**，不看分数、不看执行结果 —— 判分不能开卷。
    """
    _slots, _classes, cells = cell_slots(grid)
    pad_of = {c["idx"]: c["center_pad_mm"] for c in cells}
    means = block_means(matched)
    pairs = []
    for bid, rec in means.items():
        for k, it in enumerate(items):
            if class_gate and it["class"] not in (None, "", "unknown"):
                if rec["class"] != it["class"]:
                    continue
            cell = pad_of.get(it["slot_idx"])
            if cell is None:
                continue
            d = math.hypot(rec["center_pad_mm"][0] - cell[0],
                           rec["center_pad_mm"][1] - cell[1])
            pairs.append((d, bid, k))
    pairs.sort()
    take_b, take_k = {}, set()
    for d, bid, k in pairs:
        if bid in take_b or k in take_k:
            continue
        take_b[bid] = k
        take_k.add(k)
    poses = []
    for pose in matched:
        out = []
        for b in pose:
            k = take_b.get(b["block_id"])
            if k is None:
                continue
            out.append(dict(b, block_id=k))
        poses.append(out)
    unmatched = [dict(means[bid], block_id=bid)
                 for bid in means if bid not in take_b]
    return poses, unmatched


# ---------------------------------------------------------------- 真机
def _board(run_dir):
    return pr.Board(BOARD_ALIAS, run_dir)


def survey_plan(plan_path, pose, joints, urdf):
    """从当前关节角生成一条 survey 计划（只动臂，不碰夹爪）。"""
    q = ",".join("%.6f" % v for v in joints)
    r = subprocess.run([PY_SELF, os.path.join(HERE, "precision_plan.py"),
                        "--survey", pose, "--urdf", urdf, "--q-start", q,
                        "--out", plan_path], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("survey %s 计划失败：%s" % (pose, r.stderr.strip()))
    return plan_path


def pick_plan(plan_path, cand_json, block_idx, slot_mm, item, urdf, frame_dir):
    """生成一条阶段 2 的拾放计划（两段 yaw + 半开）。"""
    args = [PY_SELF, os.path.join(HERE, "precision_plan.py"), "--pick",
            "--candidates", cand_json, "--block", str(block_idx),
            "--slot", "%.6f,%.6f" % (slot_mm[0] / 1000.0,
                                     slot_mm[1] / 1000.0),
            "--urdf", urdf, "--from-frame", frame_dir, "--out", plan_path,
            "--yaw-grasp-deg", "%.4f" % item["yaw_grasp_deg"],
            "--yaw-place-deg", "%.4f" % item["yaw_place_deg"],
            "--yaw-source", item["yaw_source"]]
    for key, flag in (("pick_open_mm", "--pick-open-mm"),
                      ("place_open_mm", "--place-open-mm")):
        v = item.get(key)
        if v is not None:
            args += [flag, "%.3f" % float(v)]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("计划失败（块 #%d，%s）：%s"
                           % (block_idx, item["yaw_source"],
                              r.stderr.strip()[-600:]))
    return plan_path


def signed_err_mod90(measured, target=0.0):
    """``measured − target`` 折到 (−45°, +45°]（方块的朝向按 90° 周期）。"""
    d = (float(measured) - float(target)) % 90.0
    return d - 90.0 if d > 45.0 else d


def calib_report(linked, items, yaw_tol_deg=3.0):
    """指轴标定：三块几何臂放完的**实测 yaw** → δ 估计。

    方块朝向按 mod 90 判，放置目标是 ``yaw_place = δ``；δ 未知时先用 0 跑。
    实测 yaw 的 mod 90 偏差就是 δ + 本块误差：
      · 三块偏差都是一个常数（极差 ≤ ``yaw_tol_deg``）→ δ = 该常数；
      · 极差大 → 不是常数偏置（更可能是 ψ 符号/几何估计有问题），**不许**把
        中位数当 δ 用，那样只会把两块改对、第三块更错。
    """
    means = block_means(linked)
    order = {}
    for pose in linked:
        for b in pose:
            order[b["block_id"]] = True
    errs = []
    for bid in sorted(order):
        rec = means[bid]
        if rec.get("yaw_deg") is None:
            continue
        k = bid if isinstance(bid, int) and 0 <= bid < len(items) else None
        arm = items[k]["arm"] if k is not None else "unknown"
        errs.append({"block_id": bid, "arm": arm,
                     "yaw_deg": rec["yaw_deg"],
                     "err_deg": round(signed_err_mod90(rec["yaw_deg"]), 3),
                     "n_poses": rec["n_poses"]})
    if not errs:
        return {"ok": False, "verdict": "insufficient", "reason": "没有量到 yaw",
                "blocks": []}
    vals = [e["err_deg"] for e in errs]
    spread = max(vals) - min(vals)
    delta = sorted(vals)[len(vals) // 2]
    if len(vals) < 2:
        verdict, reason = "insufficient", "样本不足（%d 块）" % len(vals)
    elif spread > yaw_tol_deg:
        verdict, reason = "inconsistent", "偏差不是常数（极差 %.2f°）—— 别当 δ 用" % spread
    elif abs(delta) <= yaw_tol_deg:
        verdict, reason = "zero", ""
    else:
        verdict, reason = "constant_offset", "常数偏置 %+.2f°：δ 用它，改完要重验" % delta
    return {"ok": verdict in ("zero", "constant_offset"),
            "verdict": verdict,
            "delta_deg_estimate": round(delta, 3) if verdict != "insufficient" else None,
            "spread_deg": round(spread, 3), "yaw_tol_deg": yaw_tol_deg,
            "blocks": errs, "reason": reason}


def do_capture(board, pose, out_dir, urdf, plan_dir, tag, go=True):
    """走到 ``pose`` 巡检位 → 抓一帧到 ``out_dir``。**只动臂，不碰夹爪。**

    这是 B1（垫子定位）的入口：不需要 grid.json，不做推理。
    返回 ``(rc, frame_dir)``；``go=False`` 只预演（不抓帧）。
    """
    plan_dir = plan_dir or out_dir
    os.makedirs(plan_dir, exist_ok=True)
    js = board.joints(os.path.join(plan_dir, "joints-%s" % pose))
    plan = survey_plan(os.path.join(plan_dir, "survey-%s.json" % pose), pose,
                       js["position"], urdf)
    rc = board.exec_plan(plan, os.path.join(plan_dir, "survey-%s.jsonl" % pose),
                         go=go)
    if rc != 0 or not go:
        return rc, None
    board.capture(out_dir, tag)
    return 0, out_dir


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 2 一轮宫格归位")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None, help="覆盖 scene.grid_json")
    ap.add_argument("--frame", default=None, help="复用已抓的帧（不重新抓）")
    ap.add_argument("--capture-pose", choices=["pick", "place", "place2"],
                    default=None,
                    help="只走到巡检位抓一帧（B1 垫子定位；不动夹爪）")
    ap.add_argument("--capture-out", default=None,
                    help="--capture-pose 的帧目录")
    ap.add_argument("--round", type=int, default=None, help="默认 = 已有轮数 + 1")
    ap.add_argument("--grid-order", default=None)
    ap.add_argument("--block-order", default=None)
    ap.add_argument("--weights", default=None, help="七类 YOLO 权重（可选）")
    ap.add_argument("--go", action="store_true",
                    help="真跑；**每次都要当次授权**，闸门在 precision_exec")
    ap.add_argument("--calib", action="store_true",
                    help="指轴标定轮：只用几何臂、3 块，终态量 δ 并写 calib.json")
    args = ap.parse_args(argv)

    # ---- 只抓帧模式（B1）：不需要 grid.json，也绝不规划抓放
    if args.capture_pose:
        if not args.capture_out:
            print("--capture-pose 要同时给 --capture-out")
            return 2
        if not args.go:
            print("--capture-pose 会动臂（走到巡检位）。确认现场后加 --go；"
                  "**每次都要当次授权**。")
            return 2
        out = os.path.abspath(args.capture_out)
        board = _board(os.path.dirname(out) or ".")
        board.push()
        rc, fdir = do_capture(board, args.capture_pose, out, args.urdf,
                              os.path.dirname(out), "stage2-mat-%s" % args.capture_pose)
        if rc != 0:
            print("走到巡检位 %s 失败（rc=%d）" % (args.capture_pose, rc))
            return rc
        print("帧已到 %s" % fdir)
        print("下一步：python3 scripts/stage2_pad.py --frame %s --mat <stage2-mat.json>"
              % fdir)
        return 0

    try:
        scene = load_scene(args.scene, args.grid_order, args.block_order)
    except ValueError as exc:
        print("场景坏：%s" % exc)
        return 2
    if args.calib:
        # 标定轮只有一个变量：几何臂 / 3 块 / δ 先按 0 跑
        scene["arms_order"] = ["geometric"]
        scene["blocks_per_round"] = min(3, int(scene["blocks_per_round"]))
        scene["finger_axis_delta_deg"] = float(
            scene.get("finger_axis_delta_deg") or 0.0)
        print("标定轮：几何臂 × %d 块，δ 先按 %.2f° 跑"
              % (scene["blocks_per_round"], scene["finger_axis_delta_deg"]))
    grid_path = args.grid or scene.get("grid_json")
    if not grid_path:
        print("缺 grid.json —— 先跑 stage2_pad.py（--grid 或 scene.grid_json）")
        return 2
    try:
        grid = load_grid(grid_path)
        check_grid_order(scene, grid)
    except ValueError as exc:
        print("网格坏：%s" % exc)
        return 2
    if not scene.get("blocks_order"):
        print("块顺序：未指定 → 按候选离块心最近自动选块")
    else:
        print("块顺序：%s" % scene["blocks_order"])

    os.makedirs(args.run_dir, exist_ok=True)
    rdir = args.run_dir
    if args.round is None:
        rdirs = sorted(d for d in os.listdir(rdir)
                       if d.startswith("round-") and d[6:].isdigit())
        round_no = len(rdirs) + 1
    else:
        round_no = args.round
    cdir = os.path.join(rdir, "round-%02d" % round_no)
    os.makedirs(cdir, exist_ok=True)

    # 抓帧：复用 / 板侧抓（抓之前先走到拾取巡检位）
    board = None
    if args.frame:
        frame_dir = os.path.abspath(args.frame)
    elif args.go:
        board = _board(cdir)
        board.push()
        js = board.joints(os.path.join(cdir, "joints-pick"))
        plan = survey_plan(os.path.join(cdir, "survey-pick.json"), "pick",
                           js["position"], args.urdf)
        rc = board.exec_plan(plan, os.path.join(cdir, "survey-pick.jsonl"),
                             go=True)
        if rc != 0:
            print("走到拾取巡检位失败（rc=%d）—— 不发抓取计划" % rc)
            return rc
        frame_dir = os.path.join(cdir, "frame-pick")
        board.capture(frame_dir, "stage2-pick-%02d" % round_no)
    else:
        print("没给 --frame：离线分配需要一帧 survey 图。"
              "先 --frame DIR，或加 --go 由板子抓。")
        return 2

    try:
        analysis = analyse_frame(frame_dir, cdir, weights=args.weights)
    except (RuntimeError, OSError, ValueError) as exc:
        print("分析失败：%s" % exc)
        return 2
    if analysis.get("_yolo_warn"):
        print(analysis["_yolo_warn"])
    pool = candidate_pool(analysis, scene["pick_zone"])
    print("候选块 %d 个（拾取区内、候选 ≤%.0f mm）" % (len(pool),
                                                      pr.MAX_CAND_MM))
    res = build_assignments(pool, scene, round_no, grid,
                            order=scene.get("blocks_order"))
    for m in res["missing"]:
        print("⚠️ 块顺序里的 %s 没有可用块（候选/遮挡/出区）" % m)
    for x in res["skipped"]:
        print("⚠️ 块 #%d 没有颜色判定（%s），本轮不抓"
              % (x["p"]["block_idx"], x["p"].get("class_status")))
    if not res["items"]:
        print("没有可抓的块%s" % ("" if not res["stop_reason"]
                                  else "：" + res["stop_reason"]))
        return 1
    for it in res["items"]:
        it["pick_open_mm"] = scene.get("pick_open_mm")
        it["place_open_mm"] = scene.get("place_open_mm")
    for a, b, d in cluster_warnings(res["items"]):
        print("⚠️ 块 #%d 与 #%d 中心距只有 %.1f mm（下限 60）—— 计划器会拒"
              % (a, b, d))

    # 计划（每条独立文件，便于现场逐条复核）
    plans = []
    for k, it in enumerate(res["items"], 1):
        path = os.path.join(cdir, "pick-%02d.json" % k)
        try:
            pick_plan(path, analysis["_candidates_json"], it["block_idx"],
                      it["slot_mm"], it, args.urdf, frame_dir)
        except RuntimeError as exc:
            print("块 #%d 计划失败：%s" % (it["block_idx"], exc))
            res["stop_reason"] = "计划失败（块 #%d）" % it["block_idx"]
            break
        it["plan"] = os.path.relpath(path, rdir)
        plans.append((k, it, path))
        print("  块 #%-2d %-12s 臂 %-9s 格 %d(%s)  yaw 抓 %7.1f° 放 %6.1f°"
              "  [%s]" % (it["block_idx"], it["class"], it["arm"],
                         it["slot_idx"], it["slot_color"] or "spare",
                         it["yaw_grasp_deg"], it["yaw_place_deg"],
                         it["yaw_source"]))

    round_doc = {"version": 1, "kind": "stage2-round", "round": round_no,
                 "frame_pick": os.path.relpath(frame_dir, rdir),
                 "grid_json": os.path.relpath(os.path.abspath(grid_path), rdir),
                 "grid": {"grid_order": grid.get("grid_order"),
                          "grid_center_base_mm": grid["grid_center_base_mm"],
                          "grid_yaw_base_deg": grid["grid_yaw_base_deg"],
                          "T_base_pad": grid["T_base_pad"]},
                 "scene": scene, "items": res["items"],
                 "missing_order": res["missing"],
                 "stop_reason": res["stop_reason"]}
    with open(os.path.join(cdir, "round.json"), "w") as f:
        json.dump(round_doc, f, ensure_ascii=False, indent=1)

    if not args.go:
        print("\n⇒ 只出了计划（%d 条）：%s" % (len(plans), cdir))
        print("   真动加 --go；每条计划执行前仍要当次授权。")
        return 0 if plans else 1

    # ---- 真跑：逐块执行；任何一条判红就停
    if board is None:
        board = _board(cdir)
        board.push()
    for k, it, path in plans:
        print("\n== 块 #%d / %s / 臂 %s ==" % (it["block_idx"], it["class"],
                                               it["arm"]))
        log = os.path.join(cdir, "exec-%02d.jsonl" % k)
        rc = board.exec_plan(path, log, go=True)
        it["exec_rc"] = rc
        it["exec_log"] = os.path.relpath(log, rdir) if os.path.exists(log) else None
        round_doc["stop_reason"] = (None if rc == 0 else
                                    "执行判红（块 #%d）" % it["block_idx"])
        with open(os.path.join(cdir, "round.json"), "w") as f:
            json.dump(round_doc, f, ensure_ascii=False, indent=1)
        if rc != 0:
            print("!! 块 #%d 执行判红（rc=%d），停在原地" % (it["block_idx"], rc))
            return rc

    # ---- 终态：≥2 个 survey 位姿各抓一帧（不同视点，不是同点连拍）
    poses = []
    for i, pose in enumerate(scene.get("survey_poses", ["place"])[:2], 1):
        js = board.joints(os.path.join(cdir, "joints-pose%d" % i))
        plan = survey_plan(os.path.join(cdir, "survey-pose%d.json" % i),
                           pose, js["position"], args.urdf)
        rc = board.exec_plan(plan, os.path.join(cdir, "survey-pose%d.jsonl" % i),
                             go=True)
        if rc != 0:
            print("走到终态 survey %s 失败（rc=%d）" % (pose, rc))
            return rc
        fdir = os.path.join(cdir, "frame-pose%d" % i)
        board.capture(fdir, "stage2-pose-%02d-%d" % (round_no, i))
        poses.append(measure_pose(fdir, grid, weights=args.weights))
    matched = match_block_ids(poses)
    linked, unmatched = link_poses(matched, res["items"], grid)
    obs = {"version": 1, "round": round_no,
           "poses": [{"name": "pose_%d" % (i + 1), "blocks": p}
                     for i, p in enumerate(linked)],
           "unmatched_blocks": unmatched,
           "arms": [it["arm"] for it in res["items"]],
           "expected": [{"block_id": i, "arm": it["arm"],
                         "cell_idx": it["slot_idx"], "ran": True,
                         "exec_ok": it.get("exec_rc", 1) == 0}
                        for i, it in enumerate(res["items"])]}
    obs_path = os.path.join(cdir, "observations.json")
    json.dump(obs, open(obs_path, "w"), ensure_ascii=False, indent=1)
    if unmatched:
        print("⚠️ %d 个测量块配不上任何一次抓取（多出来的块/颜色不明）：%s"
              % (len(unmatched), [b["center_pad_mm"] for b in unmatched]))

    if args.calib:
        cal = calib_report(linked, res["items"])
        cal["round"] = round_no
        cal["frame_pose1"] = os.path.relpath(
            os.path.join(cdir, "frame-pose1"), rdir)
        json.dump(cal, open(os.path.join(cdir, "calib.json"), "w"),
                  ensure_ascii=False, indent=1)
        print("\n== 指轴标定 ==")
        for b in cal.get("blocks", []):
            print("  块 #%s yaw %7.2f°  偏差 %+6.2f°" % (b["block_id"],
                                                       b["yaw_deg"],
                                                       b["err_deg"]))
        if cal.get("verdict") == "zero":
            print("⇒ 三块偏差一致且 ≤%.0f°：δ = 0"
                  % cal["yaw_tol_deg"])
        elif cal.get("verdict") == "constant_offset":
            print("⇒ 常数偏置：δ = %+.2f°（改场景里的 finger_axis_delta_deg 后再跑一遍复核）"
                  % cal["delta_deg_estimate"])
        else:
            print("⇒ 标定判红：%s" % cal.get("reason", ""))
            print("   常数偏置才改 δ；极差大先查 ψ 符号与几何 yaw，别拿中位数当 δ")
        print("   %s" % os.path.join(cdir, "calib.json"))

    score_args = [PY_SELF, os.path.join(HERE, "stage2_score.py"),
                  "--observations", obs_path, "--grid", os.path.abspath(grid_path),
                  "--out-report", os.path.join(cdir, "score"),
                  "--threshold-yaw-deg",
                  "%.2f" % scene["thresholds"]["yaw_deg"],
                  "--threshold-pos-mm",
                  "%.2f" % scene["thresholds"]["pos_mm"]]
    r = subprocess.run(score_args, capture_output=True, text=True)
    print(r.stdout.strip())
    if r.returncode == 2:
        print("判分数据坏：%s" % r.stderr.strip())
    print("\n本轮归档：%s" % cdir)
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
