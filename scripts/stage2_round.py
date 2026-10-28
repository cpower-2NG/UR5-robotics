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
      "yaw_sign": 1.0,                     # ψ = +θ_face（2026-10-27 B2 实测判决，见 DEFAULT_SCENE）
      "finger_axis_delta_deg": 0.0,        # 指轴标定（B2）的 δ
      "pick_open_mm": 50.0, "place_open_mm": 50.0,
      "blocks_per_round": 6,
      "pick_zone": [0.25, 0.575, 0.17, 0.42],
      "survey_poses": ["stage2close", "stage2close2"],
      "pick_frames": 3, "terminal_frames": 3,   # 同姿态连拍 → yaw 融合
      "thresholds": {"yaw_deg": 6.0, "pos_mm": 10.0}   # 用户 2026-10-28 拍板
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
· ``--capture-pose {stage2high,stage2close,stage2pick,…} --capture-out DIR --go``：只走到巡检位
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
import precision_plan as pp                                       # noqa: E402
import precision_round as pr                                      # noqa: E402
import stage2_color as sc                                         # noqa: E402
import stage2_geom as sg                                          # noqa: E402

PY_SELF = sys.executable
PY_GRASPNET = os.path.expanduser("~/robotics/venvs/graspnet/bin/python")
BOARD_ALIAS = os.environ.get("STAGE2_BOARD", "jetson-hotspot")   # 与 precision_round 一致；有线备选用 STAGE2_BOARD=jetson-wifi

ARMS = ("geometric", "zero", "graspnet")

# 颜色投票的「说不清」带宽活在家门口 stage2_color（判分器也要用同一条规则），
# 这里只是给本模块的调用点留个短名字。
CLASS_VOTE_MARGIN = sc.CLASS_VOTE_MARGIN
DEFAULT_SCENE = {
    "grid_json": None,
    "grid_order": None,
    "blocks_order": [],
    "arms_order": list(ARMS),
    # 2026-10-27 B2 实测判决：**+1**（即 ψ = +θ_face），不是契约原先假设的 −1。
    # 证据：四块几何臂按 ψ=−θ 抓放后的终态块面角（target 0）：黄 −0.3°、青 −2.1°、
    # 紫 −0.4°、橙 −37.9°。前三块的拾取角分别 169.9/179.6/35.2°，若映射是 −1，
    # 终态应为 2θ（−20.2/−0.7/−19.5），与实际全不符；若是 +1，终态应与 θ 无关
    # （≈ −1.3°），前三块正好对上。橙色那块拾取角 24.7°，−1 的镜像误差
    # |−2θ| 折叠后 = 40.7°——它落在最坏点上，爪合拢时按对角接触，块在指间转不动，
    # 于是终态偏 37.9°。这就是「橙色方块抓取转角不准」的根因：**镜像映射**，
    # 不是标定常数偏置。δ 的估计因此回到 0（前三块 −0.3/−2.1/−0.4，极差 1.8°）。
    "yaw_sign": 1.0,
    "finger_axis_delta_deg": 0.0,
    # graspnet 臂的候选只接受**贴块边**的（沿用 DEFAULT_SCENE 上面那段判决的
    # 同一个机制：按对角接触时块在指间转不动）。放置 yaw = δ，所以
    # 「候选角 − 块边对齐 yaw」就是这一块的**预测终态 yaw 误差**；15° 是用户
    # 2026-10-28 拍板的上限（A 案），挡住的是 30–45° 的对角候选，不是精修。
    "graspnet_max_edge_offset_deg": 15.0,
    "pick_open_mm": 50.0,
    "place_open_mm": 50.0,
    # 松手弹滑（2026-10-27 用户目视：夹爪动作快、不太平衡，块落到桌面会弹/滑）。
    # 放置间隙 5 mm **不动**（用户决定：再小不安全）。改用三件事一起压：
    #   · 抓**块心**（合拢时块两侧受力对称，指间转动更小）；
    #   · 松手后原地停 0.5 s 再撤，让块自己落稳；
    #   · ~~速度寄存器~~ —— **这台固件不支持**：写 0x0104=40 后回读恒 100，
    #     而同一条链路写 0x0101（力）/0x0103（位置）都能读回写进去的值
    #     （2026-10-27 实机，`round-05-abort-speed`）。所以默认**不写速度**：
    #     写它只会让第一块在合拢前 fail-closed 停下，而且「以为设了其实没设」
    #     比没设更糟。将来换固件再开。
    "grasp_at": "block_center",
    "grasp_close_speed_pct": None,
    "release_open_speed_pct": None,
    "release_dwell_s": 0.5,
    "blocks_per_round": 6,
    # 2026-10-27 实测：用户摆的 9 块落在 base x 0.389–0.582 / y 0.176–0.399 m，
    # 旧的 pr.PICK_ZONE（y ≥ 0.25）会把下半排挡在池外。
    #
    # **2026-10-28 用户拍板扩区**（原 [0.36, 0.62, 0.17, 0.45]）。四条边都以抓帧
    # 实测的物理边界为据，图见 round-14/pickzone-proposal.png：
    #   x 下 0.36 → **0.25**：画面左边界在 base x ≈ 196 mm；再往左，块的 28 mm
    #        顶面掩码会被画幅切掉。这一边是本次扩区的主要来源（+110 mm）。
    #   x 上 0.62 → **0.575**：**白纸右边界恒定在 base x ≈ 605 mm**（v=60…700
    #        逐行实测 605/606），605 以外是木桌面；旧的 0.62 其实已过纸边 15 mm。
    #   y 下 0.17 **不动**：画面底边 = base y ≈ 134 mm，黄绿块在 y=151 mm 就已被
    #        切掉半个顶面（顶面点 2349，别的块 5435–6723）⇒ GraspNet 零候选。
    #        见 round-11/pick-zone.png 的 y=170 虚线。
    #   y 上 0.45 → **0.42**：0.45 正好是画面顶边（v=0），摆那儿的块有一半出画；
    #        本轮最高的块在 y=403（v≈91）。
    # **臂不是限制**：两方案四角 + 中心 + 45° yaw 的 IK 全收敛（~1e-13）、
    # check_wrist 全过（seed = round-14 实测关节角，grasp_z=0.2242）。想再大只能
    # 抬 pick 机位（重标外参 + 重跑闸门），不做。
    # ⚠️ y 170–220 仍是**掠射带**：块进得来，但顶面点云有 ~10 mm 斜坡（round-14
    #    橙块 y=191，四种角度估计散在 55.4/61.6/69.5/72.6）。摆块仍建议 y ≥ 250。
    "pick_zone": [0.25, 0.575, 0.17, 0.42],
    # 终态视点：**只用低视点**。2026-10-27 B2 第 4 轮的实测复核：「高位广视」
    # （相机 ~523 mm）上顶面碎、`ext_rect` 只有 12–24 mm（真值 ~36），yaw 不可信，
    # 黄块因此量出 −0.34° / −11.86° 的跨姿态差。两个视点都换到 ~362 mm 的
    # stage2close / stage2close2（同高、平移 40/30 mm），保留「换视点才看得见系统
    # 性偏差」这一层，但不再拿一个已知坏视点去污染中位数。
    "survey_poses": ["stage2close", "stage2close2"],
    # 同姿态连拍帧数：抓取侧（θ 单帧噪声 0–30°）与终态测量侧都取多帧再融合。
    "pick_frames": 3,
    "terminal_frames": 3,
    # 出口阈值由用户 2026-10-28 基线后拍板（8/12 → 6/10），与 stage2_score 同值；
    # 两份默认值必须一起改，否则判分器收到的阈值随入口而变。
    "thresholds": {"yaw_deg": 6.0, "pos_mm": 10.0},
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
    ``"#<idx>"`` 按**块序号**指定（同色多块时颜色是二义的 —— 指轴标定要的是
    「yaw 相互 ≥30° 的那三块」，只能按序号点）。
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
            if isinstance(cls, str) and cls.startswith("#"):
                try:
                    idx = int(cls[1:])
                except ValueError:
                    missing.append(cls)
                    continue
                cands = [x for x in left if x["p"]["block_idx"] == idx]
            else:
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


#: graspnet 臂：候选的手指轴与「块边对齐 yaw」最多差多少度。超过就不抓。
#: 对角抓的代价是**整块被转掉同样的角度**：round-16 青块候选 137.2°、块边
#: 5.5° ⇒ 预测 41.7°、实测 42.42°，抬升回读 34.8 mm 与抓帧一致（没滑）。
GRASPNET_MAX_EDGE_OFFSET_DEG = 15.0


def block_edge_yaw(block, scene):
    """块边对齐时腕部该在的 yaw（= 几何臂那支 ψ）；没有几何 yaw 时 None。"""
    theta = block.get("yaw_pix")
    if theta is None:
        theta = block.get("yaw_rect")
    if theta is None:
        return None
    sign = float(scene.get("yaw_sign", 1.0))
    delta = float(scene.get("finger_axis_delta_deg", 0.0))
    return wrap_deg(sign * float(theta) + delta)


def choose_graspnet_cand(entry, block, scene):
    """graspnet 臂挑候选：**只接受贴块边的**，再按 (离块心, −分数) 取第一个。

    为什么筛：这条臂把**候选角**当抓取 yaw、放置 yaw 取 δ，所以
    「候选角 − 块边对齐 yaw」（mod 90）就是这一块的**预测终态 yaw 误差**。
    网络在别处会给对角候选 —— round-16 青块 137.2° vs 块边 5.5°，预测 41.7°、
    实测 42.42°；同一机制在 B2 也出现过（橙块 ψ=−θ 的镜像误差 40.7° ⇒ 终态 37.9°，
    见 ``DEFAULT_SCENE`` 的 yaw_sign 判决）。抓取点/放置运动都不动，只换候选。

    返回 ``(cand, note)``。贴边的候选一个都没有时 ``cand=None`` —— 调用方按
    「这一块不抓」处理并记原因，**不退回对角候选**（fail-closed）。
    块没有几何 yaw 就筛不了：退回最近候选，note 里写明。
    """
    best = entry.get("best")
    aligned = block_edge_yaw(block, scene)
    if aligned is None:
        return best, ("块没有几何 yaw，筛不了（用最近候选）" if best else "")
    max_off = float(scene.get("graspnet_max_edge_offset_deg",
                              GRASPNET_MAX_EDGE_OFFSET_DEG))
    cands = entry.get("near") or ([best] if best else [])
    if not cands:
        return None, "块没有候选"
    for c in cands:                       # ``near`` 已按 (dist, −score) 排好
        if gc.ang_diff_mod90(float(c["angle_deg"]), aligned) <= max_off:
            return c, ""
    off = gc.ang_diff_mod90(float(cands[0]["angle_deg"]), aligned)
    return None, ("最近的候选离块边 %.1f°（>%.0f°）" % (off, max_off))


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


def completed_items(round_doc):
    """``--resume`` 用：上一版 round.json 里 ``exec_rc == 0`` 的块 → ``{block_idx: item}``。

    2026-10-27 B2 第 4 轮的实况：块 #0 跑完、块 #9 在发轨迹前的闸门拿到
    ``program=None``（Dashboard 只读查询超时，不是示教器停了），脚本按 fail-closed
    停在原地。那一轮的三个块里已经有两个不再在原位，整轮重跑会把空位当块抓；
    续跑只补没完成的块，终态测量仍覆盖整轮的全部块。
    """
    out = {}
    for it in (round_doc or {}).get("items", []):
        if it.get("exec_rc") == 0 and it.get("block_idx") is not None:
            out[int(it["block_idx"])] = it
    return out


def apply_resume(items, resume_done):
    """把上一版已完成的状态贴到本轮 items 上（``resumed=True`` ⇒ 执行循环跳过）。"""
    for it in items:
        done = resume_done.get(int(it["block_idx"]))
        if done is not None:
            it["exec_rc"] = 0
            it["exec_log"] = done.get("exec_log")
            it["resumed"] = True
    return items


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


def placed_cells(round_doc):
    """一份 ``round.json`` 里**真的放进格子里**的格号（``exec_rc == 0`` 的块）。"""
    out = []
    for it in round_doc.get("items") or []:
        if it.get("exec_rc") == 0 and it.get("slot_idx") is not None:
            out.append(int(it["slot_idx"]))
    return out


def occupied_from_previous(run_dir, exclude_dir=None, cells_free=()):
    """扫前几轮的 ``round.json``，把已经放进格子的格号并起来。

    为什么必须这么做：`build_assignments` 的 ``used`` 原来只在一轮内有意义，
    新一轮看不见上一轮放了什么。2026-10-27 B2 第 5 轮重跑就是这么把第二块
    黄色叠到第 4 轮已经占了**格 3** 上的（该轮块顺序 #9 → 格 3）。
    ``cells_free`` 是**人**的口子：真把某格的块拿走了，就在这里声明它空着。
    """
    used = set()
    for d in sorted(os.listdir(run_dir)):
        p = os.path.join(run_dir, d, "round.json")
        if not d.startswith("round-") or not d[6:].isdigit():
            continue
        if exclude_dir is not None and os.path.abspath(p).startswith(
                os.path.abspath(exclude_dir) + os.sep):
            continue
        if os.path.exists(p):
            try:
                used.update(placed_cells(json.load(open(p))))
            except (ValueError, OSError):
                continue                      # 坏文件不阻断：当它没放过
    return sorted(used - {int(c) for c in cells_free})


def build_assignments(pool, scene, round_no, grid, order=None, used_prev=()):
    """一轮的完整分配：块 → 臂 → 格 → 两段 yaw。

    返回 ``{'items', 'missing', 'skipped', 'stop_reason'}``。
    选格复用 ``slot_routing.pick_slot``（同色优先、备用格通配），**不改它的语义**。
    ``used_prev``：前几轮**已经放进去**的格号（见 `occupied_from_previous`），
    本轮必须绕开它们。
    """
    slots, classes, _cells = cell_slots(grid)
    used = [False] * len(slots)
    for c in used_prev:
        if 0 <= int(c) < len(used):
            used[int(c)] = True
    n = int(scene.get("blocks_per_round", 6))
    chosen, missing, skipped = choose_blocks(pool, order=order, n=n)
    arms = scene.get("arms_order") or list(ARMS)
    offset = (int(round_no) - 1) % len(arms)
    items, stop = [], None
    for x in chosen:
        cls = x["p"]["class"]
        arm = assign_arm(len(items), arms, offset)
        cand = x["best"]
        if arm == "graspnet":
            # 这条臂拿候选角当抓取 yaw ⇒ 候选必须贴块边（见 choose_graspnet_cand）。
            # 没有贴边候选就**不抓这一块**：宁可少抓，不退回对角候选。
            cand, why = choose_graspnet_cand(x["p"], x["block"], scene)
            if cand is None:
                skipped.append(dict(x, skip_reason=why))
                continue
        slot_idx = pr.pick_slot(used, classes, cls)
        if slot_idx is None:
            stop = "类别 %s 没有空槽了" % cls
            break
        yg, yp, src, note = yaw_for_arm(arm, x["block"], cand, scene)
        aligned = block_edge_yaw(x["block"], scene)
        edge_off = (gc.ang_diff_mod90(float(cand["angle_deg"]), aligned)
                    if aligned is not None else None)
        cdist = cand.get("dist_mm")
        if cdist is None:                      # 测试/旧数据里的 best 不带 dist
            cdist = x["p"]["dist_mm"]
        items.append({
            "block_idx": int(x["p"]["block_idx"]), "class": cls,
            "arm": arm, "slot_idx": int(slot_idx), "slot_color": classes[slot_idx],
            "slot_mm": slots[slot_idx],
            "yaw_grasp_deg": round(yg, 3), "yaw_place_deg": round(yp, 3),
            "yaw_source": src, "yaw_note": note,
            "cand_dist_mm": round(float(cdist), 2),
            "cand_angle_deg": round(float(cand["angle_deg"]), 2),
            "cand_width_mm": round(float(cand["width_mm"]), 2),
            "cand_index": cand.get("i"),
            "cand_edge_offset_deg": (None if edge_off is None
                                     else round(edge_off, 2)),
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
            # 票份额要带出去：跨姿态合并按它加权投票（见 block_means），
            # 丢了它就只能「先到的赢」。
            "class_share": c.get("hsv_share"),
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


def mean_yaw_mod90(vals, period=90.0, fold=True):
    """跨姿态合并 yaw：先把各姿态折到同一支再取均值。

    块面 yaw 只有 90° 周期意义，**不能直接算术平均**。2026-10-27 B2 第 4 轮的实况：
    紫块两个姿态量到 98.40° 与 1.70°（差 96.7°，mod 90 只差 6.7°），算术平均得
    50.05°，标定据此报「偏差 −39.95°」——把同一个角度算成了两个，判红也是假的。

    ``fold=True``（默认）把结果折进 ``[0, period)``，判分/标定要的就是这个；
    ``fold=False`` 让结果**贴合第一个值**（只做 ±period/2 的折叠）—— 抓取侧的 ψ
    要的是「和第一帧同一个支」，折到 0–90 会把 134.7° 变成 44.7°，物理上等价但
    腕部要多转 90°，没必要。
    """
    if not vals:
        return None
    ref = float(vals[0])
    acc = []
    for v in vals:
        d = (float(v) - ref) % period
        if d > period / 2.0:
            d -= period
        acc.append(ref + d)
    mean = sum(acc) / len(acc)
    return round(mean % period if fold else mean, 3)


def yaw_span_mod90(vals, period=90.0):
    """一组 yaw 的 mod-``period`` 极差（用来量化「同一块各帧各姿态差多少」）。"""
    if not vals or len(vals) < 2:
        return None
    ref = float(vals[0])
    acc = []
    for v in vals:
        d = (float(v) - ref) % period
        if d > period / 2.0:
            d -= period
        acc.append(d)
    return round(max(acc) - min(acc), 3)


#: 同姿态连拍时，同一物理块在相邻两帧的块心允许平移多少（臂不动，只差深度噪声）。
FUSE_MATCH_MM = 12.0


def fuse_block_yaw(primary, extra_block_lists, match_mm=FUSE_MATCH_MM):
    """同姿态连拍 → 把额外帧的 ``yaw_pix`` 并进第一帧的块表（mod 90 圆均值）。

    为什么值得多抓几帧：2026-10-27 B2 实测，同一物理块、同一姿态、相隔几分钟的
    两帧，θ_pix 极差 0.00°–29.86°（中位约 5°），而计划只用其中一帧；抓取角因此
    带着单帧噪声，终态判红也就分不清「映射错」还是「估计抖」。

    块号、候选、颜色仍取第一帧 —— 只有 yaw 被覆盖（原值留在 ``yaw_pix_single``）。
    配对按块心平面距离（``≤match_mm``），臂在连拍期间不动，同一块只差毫米级噪声。
    只有**都 trusted** 的帧才参与；第一帧就不 trusted 的块不改（不靠融合「救活」
    一个本来判不了可信的块）。

    返回 ``{block_idx: {"n": 用了几帧, "span_deg": 极差}}``，写进 ``round.json``。
    """
    out = {}
    for b in primary:
        if b.get("yaw_pix") is None or b.get("yaw_pix_trusted") is False:
            out[b["idx"]] = {"n": 0, "span_deg": None}
            continue
        vals = [float(b["yaw_pix"])]
        for blist in extra_block_lists:
            best, best_d = None, match_mm
            for c in blist:
                if c.get("yaw_pix") is None or c.get("yaw_pix_trusted") is False:
                    continue
                d = math.hypot(float(c["cx"]) - float(b["cx"]),
                               float(c["cy"]) - float(b["cy"]))
                if d <= best_d:
                    best, best_d = c, d
            if best is not None:
                vals.append(float(best["yaw_pix"]))
        if len(vals) > 1:
            b["yaw_pix_single"] = b["yaw_pix"]
            b["yaw_pix"] = mean_yaw_mod90(vals, fold=False)
        b["yaw_pix_frames"] = len(vals)
        out[b["idx"]] = {"n": len(vals), "span_deg": yaw_span_mod90(vals)}
    return out


# 颜色投票的实现在 ``stage2_color.vote_class`` —— 判分器（stage2_score）也要用
# 同一条规则，两处各写一份迟早会分叉。这里留个别名，调用点不必改。
vote_class = sc.vote_class


def fuse_measurements(frames, match_mm=8.0):
    """同一 survey 姿态连拍的多帧 → 一帧（位置取中位数，yaw 按 mod 90 圆均值）。

    终态测量噪声是实打实的：2026-10-27 B2 黄块在两个视点量到 −0.34° 与 −11.86°。
    同一视点连拍几帧能把**随机**那部分压下去；换视点才能暴露**系统性**那部分
    （所以 ``survey_poses`` 仍然是两个不同位姿，这里只做视点内的融合）。
    """
    frames = [f for f in frames if f]
    if not frames:
        return []
    if len(frames) == 1:
        return [dict(b, frames_used=1) for b in frames[0]]
    base = frames[0]
    out = []
    for b in base:
        bx, by = float(b["center_pad_mm"][0]), float(b["center_pad_mm"][1])
        hits = [b]
        for other in frames[1:]:
            best, best_d = None, match_mm
            for c in other:
                d = math.hypot(float(c["center_pad_mm"][0]) - bx,
                               float(c["center_pad_mm"][1]) - by)
                if d <= best_d:
                    best, best_d = c, d
            if best is not None:
                hits.append(best)
        yaws = [float(h["yaw_deg"]) for h in hits
                if h.get("yaw_deg") is not None and h.get("yaw_trusted") is not False]
        rec = dict(b)
        rec["center_pad_mm"] = [
            round(float(np.median([float(h["center_pad_mm"][k]) for h in hits])), 2)
            for k in (0, 1)]
        if b.get("center_base_mm"):
            rec["center_base_mm"] = [
                round(float(np.median([float(h["center_base_mm"][k]) for h in hits
                                       if h.get("center_base_mm")])), 2)
                for k in (0, 1)]
        rec["yaw_deg"] = mean_yaw_mod90(yaws) if yaws else None
        rec["yaw_span_deg"] = yaw_span_mod90(yaws)
        # 颜色也要在视点内投票，不能取第一帧的：视点1 里三帧分别给了
        # orange 0.65 / orange 0.62 / 认不出 —— 取第一帧等于把最弱的证据定成结论。
        cls, status, share, reason = vote_class(hits)
        rec["class"] = cls if cls else None
        rec["class_status"] = status
        rec["class_share"] = round(share, 3)
        if reason:
            rec["class_reason"] = reason
        rec["frames_used"] = len(hits)
        out.append(rec)
    return out


def block_means(matched):
    """跨姿态合并：``{block_id: {class, class_status, center_pad_mm, …}}``。

    颜色**按票权投票**，不是「先到的赢」：每个姿态一票，权重是该姿态自己的
    HSV 票份额。2026-10-28 round-10 就是栽在这里 —— 木棕块视点1 判 orange
    (0.65)、视点2 判 tan (0.93)，先到的 orange 定了调，于是它配不上「tan→格2」
    那条、被记成 ``not_measured``。两类的票权差 < ``CLASS_VOTE_MARGIN`` 才标
    ``conflict``（交人看），不猜。
    """
    out = {}
    for pose in matched:
        for b in pose:
            rec = out.setdefault(b["block_id"], {"centers": [], "yaws": [],
                                                 "poses": []})
            rec["centers"].append(b["center_pad_mm"])
            if b.get("yaw_deg") is not None:
                rec["yaws"].append(float(b["yaw_deg"]))
            # 「这一姿态说了什么」整条留着交给 vote_class —— 它按 class_share
            # 加权，且 class_status=unknown 的姿态自然不投票（不会像 `or` 那样
            # 被先到的 "unknown" 卡死）。
            rec["poses"].append(b)
    for bid, rec in out.items():
        cls, status, share, reason = vote_class(rec.pop("poses"))
        rec["class"] = cls
        rec["class_status"] = status
        rec["class_share"] = round(share, 3)
        if reason:
            rec["class_reason"] = reason
        cs = rec["centers"]
        rec["center_pad_mm"] = [round(sum(c[0] for c in cs) / len(cs), 2),
                                round(sum(c[1] for c in cs) / len(cs), 2)]
        rec["yaw_deg"] = mean_yaw_mod90(rec["yaws"])
        # 跨姿态极差：同样两帧「量同一块」的 yaw 差，是测量噪声的直接读数
        # （2026-10-27 B2 黄块 −0.34° vs −11.86°，差 11.52°）。
        rec["yaw_pose_span_deg"] = yaw_span_mod90(rec["yaws"])
        rec["n_poses"] = len(cs)
    return out


def link_poses(matched, items, grid, class_gate=True, max_mm=None):
    """测量块 → 计划里的 item 一对一配对（颜色匹配 + 离预期格最近）。

    返回 ``(poses, unmatched)``：``poses`` 里每块的 ``block_id`` 换成 item 下标
    （``stage2_score.expected`` 的键）；``unmatched`` 是配不上任何 item 的测量块。
    配对只看**颜色与格心距离**，不看分数、不看执行结果 —— 判分不能开卷。
    """
    _slots, _classes, cells = cell_slots(grid)
    pad_of = {c["idx"]: c["center_pad_mm"] for c in cells}
    if max_mm is None:
        # 离预期格超过 3 个格距就不是「放歪了」，是另一件东西（背景/幻影）。
        # 不设上限时，只要某颜色只剩一个幻影，它就会被硬配给同色的 item——
        # 2026-10-27 B2 就出现过一个 383 mm 外的「purple」被配成 #2 块。
        max_mm = 3.0 * float(grid.get("pitch_mm", 60.0))
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
            if d > max_mm:
                continue
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


# joint_states 的 position 顺序是**发布顺序**（pan, w2, w3, w1, elbow, lift），
# 与 URDF 链序不同。直接把它当 q_start 会与实测差 157°（2026-10-27 实测：
# 闸门 `最大差 157.76° → 拒绝`）。凡是要进 precision_plan 的关节角，先按 name 重排。
JOINT_ORDER = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def chain_q(js):
    """把 precision_capture 的 joint_states.json 重排成 URDF 链序（弧度）。"""
    by = dict(zip(js["name"], js["position"]))
    miss = [n for n in JOINT_ORDER if n not in by]
    if miss:
        raise ValueError("joint_states 缺关节 %s（有 %s）" % (miss, js["name"]))
    return [float(by[n]) for n in JOINT_ORDER]


def survey_plan(plan_path, pose, joints, urdf):
    """从当前关节角生成一条 survey 计划（只动臂，不碰夹爪）。

    ⚠️ ``--q-start`` 必须用 ``=`` 形式：肩关节角为负时值是 ``-0.79,…``，
    分开写会被 argparse 当成「又一个选项」拒掉（``expected one argument``）——
    2026-10-27 臂停在负角位姿上才暴露。
    """
    q = ",".join("%.6f" % v for v in joints)
    r = subprocess.run([PY_SELF, os.path.join(HERE, "precision_plan.py"),
                        "--survey", pose, "--urdf", urdf, "--q-start=" + q,
                        "--out", plan_path], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("survey %s 计划失败：%s" % (pose, r.stderr.strip()))
    return plan_path


def pick_plan(plan_path, cand_json, block_idx, slot_mm, item, urdf, frame_dir,
              q_start=None):
    """生成一条阶段 2 的拾放计划（两段 yaw + 半开）。

    ``q_start``：**当前实测关节角**（URDF 链序，弧度）。给了就用 ``--q-start``，
    不再从帧里取起点 —— 一帧只能给一条计划的起点，而一轮要连跑三块：第一块
    放完臂已经不在抓帧位姿，后面几条计划若还拿帧里的 q_start，`precision_exec`
    的闸门会按「别人动过臂」拒掉（2026-10-27 B2 实测最大差 56°）。
    """
    args = [PY_SELF, os.path.join(HERE, "precision_plan.py"), "--pick",
            "--candidates", cand_json, "--block", str(block_idx),
            "--slot", "%.6f,%.6f" % (slot_mm[0] / 1000.0,
                                     slot_mm[1] / 1000.0),
            "--urdf", urdf, "--out", plan_path,
            "--yaw-grasp-deg", "%.4f" % item["yaw_grasp_deg"],
            "--yaw-place-deg", "%.4f" % item["yaw_place_deg"],
            "--yaw-source", item["yaw_source"]]
    if q_start is None:
        args += ["--from-frame", frame_dir]
    else:
        # 同上：负角必须用 `=` 形式，否则 argparse 把 "-0.79,…" 当选项。
        args += ["--q-start=" + ",".join("%.6f" % float(v) for v in q_start)]
    for key, flag in (("pick_open_mm", "--pick-open-mm"),
                      ("place_open_mm", "--place-open-mm"),
                      ("grasp_close_speed_pct", "--grip-speed-pct"),
                      ("release_open_speed_pct", "--release-speed-pct"),
                      ("release_dwell_s", "--release-dwell-s")):
        v = item.get(key)
        if v is not None:
            args += [flag, "%.3f" % float(v) if isinstance(v, float) else str(v)]
    if item.get("grasp_at"):
        args += ["--grasp-at", str(item["grasp_at"])]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("计划失败（块 #%d，%s）：%s"
                           % (block_idx, item["yaw_source"],
                              r.stderr.strip()[-600:]))
    return plan_path


def plan_items(items, cdir, rdir, make_plan):
    """逐块生成计划；**单块失败只跳过它**，不打断整轮。

    2026-10-27 round-06 与 round-08 两次都是「一块过不了预检 ⇒ 后面的块连
    规划都没做」，一轮只剩 1 块、标定判「样本不足（1 块）」。预检是**逐块的**
    物理判据（邻块太近、限位、绕线…），一块不合格不代表别的块也不行。
    全部块都失败，才等于这一轮没得跑。

    ``make_plan(path, item)``：真正调用 `precision_plan.py` 的东西（测试里换成假的）。
    返回 ``(plans, failed)``：``plans`` 是 ``(序号, item, 路径)``，``failed`` 是块号。
    """
    plans, failed = [], []
    for k, it in enumerate(items, 1):
        path = os.path.join(cdir, "pick-%02d.json" % k)
        try:
            make_plan(path, it)
        except RuntimeError as exc:
            print("!! 块 #%d 计划失败，跳过它继续规划后面的：%s"
                  % (it["block_idx"], exc))
            it["plan_error"] = str(exc)[-400:]
            failed.append(it["block_idx"])
            continue
        it["plan"] = os.path.relpath(path, rdir)
        plans.append((k, it, path))
        print("  块 #%-2d %-12s 臂 %-9s 格 %d(%s)  yaw 抓 %7.1f° 放 %6.1f°"
              "  [%s]" % (it["block_idx"], it["class"], it["arm"],
                         it["slot_idx"], it["slot_color"] or "spare",
                         it["yaw_grasp_deg"], it["yaw_place_deg"],
                         it["yaw_source"]))
    return plans, failed


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
                       chain_q(js), urdf)
    rc = board.exec_plan(plan, os.path.join(plan_dir, "survey-%s.jsonl" % pose),
                         go=go)
    if rc != 0 or not go:
        return rc, None
    board.capture(out_dir, tag)
    return 0, out_dir


# ---------------------------------------------------------------- CLI
def terminal_measure(board, cdir, rdir, round_no, grid, grid_path, scene,
                 items, args):
    """终态测量 → 标定判决 → 判分。**只走到 ≥2 个 survey 位姿，不抓不放。**

    2026-10-27 round-09 的教训：脚本在「某块执行判红」时 `return`，
    于是**已经放好的 4 块永远量不到** —— 标定缺的就是这一步。测量是独立的
    收尾动作（只动臂、不动夹爪），不该绑在「这一轮一块都没失败」上。

    ``items``：块表（通常来自本轮的 ``round.json``），测量块按位置/颜色配给它。
    正常流程传 ``res["items"]``；``--measure-only`` 传上一轮存档里的块表。
    """
    # ---- 终态：≥2 个 survey 位姿各抓一帧（不同视点，不是同点连拍）
    #      每个视点再连拍几帧、视点内融合（位置中位数 + yaw 圆均值）——
    #      换视点看的是系统性偏差，连拍压的是随机噪声，两样都要。
    n_term = max(1, int(scene.get("terminal_frames", 1)))
    poses = []
    for i, pose in enumerate(scene.get("survey_poses", ["place"])[:2], 1):
        js = board.joints(os.path.join(cdir, "joints-pose%d" % i))
        plan = survey_plan(os.path.join(cdir, "survey-pose%d.json" % i),
                           pose, chain_q(js), args.urdf)
        rc = board.exec_plan(plan, os.path.join(cdir, "survey-pose%d.jsonl" % i),
                             go=True)
        if rc != 0:
            print("走到终态 survey %s 失败（rc=%d）" % (pose, rc))
            return rc
        frames = []
        for j in range(n_term):
            fdir = os.path.join(cdir, "frame-pose%d" % i if j == 0
                                else "frame-pose%d-%s" % (i, "bcdefgh"[j - 1]))
            board.capture(fdir, "stage2-pose-%02d-%d-%d" % (round_no, i, j + 1))
            frames.append(measure_pose(fdir, grid, weights=args.weights))
        merged = fuse_measurements(frames)
        spans = [b["yaw_span_deg"] for b in merged if b.get("yaw_span_deg")]
        print("  视点 %d（%s）：%d 块，连拍 %d 帧%s"
              % (i, pose, len(merged), n_term,
                 "" if not spans else "，帧内 yaw 极差中位 %.2f°"
                 % sorted(spans)[len(spans) // 2]))
        poses.append(merged)
    matched = match_block_ids(poses)
    linked, unmatched = link_poses(matched, items, grid)
    obs = {"version": 1, "round": round_no,
           "poses": [{"name": "pose_%d" % (i + 1), "blocks": p}
                     for i, p in enumerate(linked)],
           "unmatched_blocks": unmatched,
           "arms": [it["arm"] for it in items],
           "expected": [{"block_id": i, "arm": it["arm"],
                         "cell_idx": it["slot_idx"], "ran": True,
                         "exec_ok": it.get("exec_rc", 1) == 0}
                        for i, it in enumerate(items)]}
    obs_path = os.path.join(cdir, "observations.json")
    json.dump(obs, open(obs_path, "w"), ensure_ascii=False, indent=1)
    if unmatched:
        print("⚠️ %d 个测量块配不上任何一次抓取（多出来的块/颜色不明）：%s"
              % (len(unmatched), [b["center_pad_mm"] for b in unmatched]))

    if args.calib:
        cal = calib_report(linked, items)
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


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 2 一轮宫格归位")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None, help="覆盖 scene.grid_json")
    ap.add_argument("--frame", default=None, help="复用已抓的帧（不重新抓）")
    ap.add_argument("--extra-frame", dest="extra_frames", action="append",
                    default=None, metavar="DIR",
                    help="与 --frame 同姿态的另一帧；给了就做 yaw 多帧融合"
                         "（可重复，离线复算用）")
    ap.add_argument("--capture-pose", choices=sorted(pp.SURVEY_TOOL0),
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
    ap.add_argument("--resume", action="store_true",
                    help="续跑同一轮：跳过该轮 round.json 里 exec_rc==0 的块，"
                         "只补没完成的，终态测量仍覆盖整轮")
    ap.add_argument("--measure-only", action="store_true",
                    help="只做终态测量/判分（块表取本轮的 round.json）："
                         "已放好的块不再抓放。某块判红停在原地后补测量用")
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
        # 标定轮只有一个变量：几何臂 / 默认 3 块 / δ 先按 0 跑。
        # `calib_blocks` 可以调大：单块预检不过会被跳过（见下方的计划循环），
        # 3 块里废一块就只剩 2 块、判「样本不足」—— 多排一两块是这一轮的余量。
        scene["arms_order"] = ["geometric"]
        want = int(scene.get("calib_blocks") or 3)
        scene["blocks_per_round"] = min(want, int(scene["blocks_per_round"]))
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
    if args.round is not None:
        round_no = args.round
    elif args.measure_only:
        # 补测量量的是**上一轮已经放好的块**，默认要落在最后一份有块表的轮次上，
        # 不能像正常轮那样往下开新号：2026-10-27 round-10 收尾崩掉后补测量，
        # 默认号算成 round-11，于是去读不存在的 round-11/round.json 而 rc=2
        # （还在 runs/ 下留了一个空 round-11 目录）。
        have = [int(d[6:]) for d in os.listdir(rdir)
                if d.startswith("round-") and d[6:].isdigit()
                and os.path.exists(os.path.join(rdir, d, "round.json"))]
        round_no = max(have) if have else 1
    else:
        rdirs = sorted(d for d in os.listdir(rdir)
                       if d.startswith("round-") and d[6:].isdigit())
        round_no = len(rdirs) + 1
    cdir = os.path.join(rdir, "round-%02d" % round_no)
    os.makedirs(cdir, exist_ok=True)

    resume_done = {}
    if args.resume:
        prev_path = os.path.join(cdir, "round.json")
        if os.path.exists(prev_path):
            resume_done = completed_items(json.load(open(prev_path)))
        print("续跑 round-%02d：已完成 %d 块 %s，只补其余"
              % (round_no, len(resume_done), sorted(resume_done)))
        if not resume_done:
            print("⚠️ 这一轮没有已完成的块，--resume 等同于整轮重跑")

    # ---- 只测量模式：块表取本轮的 round.json，只做终态测量/判分
    #      2026-10-27 round-09：第 5 块的闸门拒发后整轮 `return`，已经放进格子的
    #      4 块**一次都没量**，标定因此又判「样本不足」。测量是收尾动作，
    #      不该被「这一轮有没有块失败」绑住。
    if args.measure_only:
        prev_path = os.path.join(cdir, "round.json")
        if not os.path.exists(prev_path):
            print("--measure-only 要读 %s —— 没有块表就配不上测量" % prev_path)
            return 2
        items = json.load(open(prev_path)).get("items") or []
        ok_items = [it for it in items if it.get("exec_rc") == 0]
        if not ok_items:
            print("!! round-%02d 没有成功执行的块 —— 没有可测量的东西" % round_no)
            return 2
        if not args.go:
            print("--measure-only 也会动臂（走到两个 survey 位姿）。"
                  "确认现场后加 --go；**每次都要当次授权**。")
            return 2
        print("只做终态测量：round-%02d 已放好 %d 块（块表 %d 条），不抓不放"
              % (round_no, len(ok_items), len(items)))
        board = _board(cdir)
        board.push()
        return terminal_measure(board, cdir, rdir, round_no, grid, grid_path,
                                scene, items, args)

    # 抓帧：复用 / 板侧抓（抓之前先走到拾取巡检位）
    # stage2pick：0.40 m 高、相机 ~ (485, 292, 400) mm —— 0.52 m 高位实测块
    # 顶高偏低且碎簇多，抓帧要近；0.34 m 又装不下九块 ~0.34 m 的 y 跨度。
    board = None
    extra_pick_frames = []
    if args.frame:
        frame_dir = os.path.abspath(args.frame)
        extra_pick_frames = list(getattr(args, "extra_frames", None) or [])
    elif args.go:
        board = _board(cdir)
        board.push()
        js = board.joints(os.path.join(cdir, "joints-pick"))
        plan = survey_plan(os.path.join(cdir, "survey-pick.json"), "stage2pick",
                           chain_q(js), args.urdf)
        rc = board.exec_plan(plan, os.path.join(cdir, "survey-pick.jsonl"),
                             go=True)
        if rc != 0:
            print("走到拾取巡检位失败（rc=%d）—— 不发抓取计划" % rc)
            return rc
        frame_dir = os.path.join(cdir, "frame-pick")
        board.capture(frame_dir, "stage2-pick-%02d" % round_no)
        # 同姿态连拍：臂不动、只多按几次相机。θ_pix 的单帧噪声实测 0–30°，
        # 抓取角就是从这里来的（见 fuse_block_yaw 的注释）。
        for j in range(1, max(1, int(scene.get("pick_frames", 1)))):
            fd = os.path.join(cdir, "frame-pick-%s" % "bcdefgh"[j - 1])
            board.capture(fd, "stage2-pick-%02d-%d" % (round_no, j + 1))
            extra_pick_frames.append(fd)
    else:
        print("没给 --frame：离线分配需要一帧 survey 图。"
              "先 --frame DIR，或加 --go 由板子抓。")
        return 2

    try:
        analysis = analyse_frame(frame_dir, cdir, weights=args.weights)
    except (RuntimeError, OSError, ValueError) as exc:
        print("分析失败：%s" % exc)
        return 2
    fused_yaw = {}
    if extra_pick_frames:
        extras = []
        for fd in extra_pick_frames:
            f = gc.load_frame(fd)
            eb, _tz = gc.find_blocks(f)
            extras.append(add_pixel_yaw(f, eb))
        fused_yaw = fuse_block_yaw(analysis["blocks"], extras)
        used = [v for v in fused_yaw.values() if v["n"] > 1]
        if used:
            print("yaw 多帧融合：%d 块用上 ≥2 帧，跨帧极差中位 %.2f°（最大 %.2f°）"
                  % (len(used),
                     sorted(v["span_deg"] for v in used)[len(used) // 2],
                     max(v["span_deg"] for v in used)))
        # 极差过大只有两种解释：那一帧的矩形估计坏了，或者**配错了块**。
        # 连拍时臂不动、场景不动，正常应 ≤10°；超了要人看一眼再往下走。
        for idx, v in sorted(fused_yaw.items()):
            if v["span_deg"] is not None and v["span_deg"] > 15.0:
                print("⚠️ 块 #%d 的 yaw 跨帧极差 %.1f°（>15°）—— 查是不是配错了块"
                      "或某一帧的矩形估计坏了" % (idx, v["span_deg"]))
    if analysis.get("_yolo_warn"):
        print(analysis["_yolo_warn"])
    pool = candidate_pool(analysis, scene["pick_zone"])
    print("候选块 %d 个（拾取区内、候选 ≤%.0f mm）" % (len(pool),
                                                      pr.MAX_CAND_MM))
    prev_used = occupied_from_previous(rdir, exclude_dir=cdir,
                                       cells_free=scene.get("cells_free") or ())
    if prev_used:
        print("已占用的格（前几轮真的放进去的）：%s —— 本轮绕开；"
              "要是人已经把某格的块拿走了，在 scene.json 的 cells_free 里声明"
              % prev_used)
    res = build_assignments(pool, scene, round_no, grid,
                            order=scene.get("blocks_order"), used_prev=prev_used)
    for m in res["missing"]:
        print("⚠️ 块顺序里的 %s 没有可用块（候选/遮挡/出区）" % m)
    for x in res["skipped"]:
        why = x.get("skip_reason")
        if why:
            print("⚠️ 块 #%d（%s）本轮不抓：%s"
                  % (x["p"]["block_idx"], x["p"].get("class"), why))
        else:
            print("⚠️ 块 #%d 没有颜色判定（%s），本轮不抓"
                  % (x["p"]["block_idx"], x["p"].get("class_status")))
    if not res["items"]:
        print("没有可抓的块%s" % ("" if not res["stop_reason"]
                                  else "：" + res["stop_reason"]))
        return 1
    for it in res["items"]:
        it["pick_open_mm"] = scene.get("pick_open_mm")
        it["place_open_mm"] = scene.get("place_open_mm")
        it["grasp_at"] = scene.get("grasp_at")
        it["grasp_close_speed_pct"] = scene.get("grasp_close_speed_pct")
        it["release_open_speed_pct"] = scene.get("release_open_speed_pct")
        it["release_dwell_s"] = scene.get("release_dwell_s")
    apply_resume(res["items"], resume_done)
    for a, b, d in cluster_warnings(res["items"]):
        print("⚠️ 块 #%d 与 #%d 中心距只有 %.1f mm（下限 60）—— 计划器会拒"
              % (a, b, d))

    # 计划（每条独立文件，便于现场逐条复核）
    plans, failed = plan_items(
        res["items"], cdir, rdir,
        lambda path, it: pick_plan(path, analysis["_candidates_json"],
                                   it["block_idx"], it["slot_mm"], it,
                                   args.urdf, frame_dir))
    if failed:
        res["stop_reason"] = "有块计划失败（%s）" % ", ".join("#%d" % b for b in failed)
    if not plans:
        print("!! 所有块都规划不出来 —— 这一轮没得跑（检查摆件间距/遮挡）")
        return 1
    print("本轮能跑 %d 块%s" % (len(plans),
                              "" if not failed else
                              "；跳过 %d 块（%s）" % (len(failed),
                                                     ", ".join("#%d" % b for b in failed))))

    round_doc = {"version": 1, "kind": "stage2-round", "round": round_no,
                 "frame_pick": os.path.relpath(frame_dir, rdir),
                 "grid_json": os.path.relpath(os.path.abspath(grid_path), rdir),
                 "grid": {"grid_order": grid.get("grid_order"),
                          "grid_center_base_mm": grid["grid_center_base_mm"],
                          "grid_yaw_base_deg": grid["grid_yaw_base_deg"],
                          "T_base_pad": grid["T_base_pad"]},
                 "scene": scene, "items": res["items"],
                 "pick_yaw_fusion": fused_yaw,
                 "missing_order": res["missing"],
                 # 本轮**没抓**的块与原因（没有颜色判定 / graspnet 臂没有贴边候选）。
                 # 落盘是为了事后能分清「少抓」与「没看见」。
                 "skipped": [{"block_idx": int(x["p"]["block_idx"]),
                              "class": x["p"].get("class"),
                              "reason": x.get("skip_reason") or "没有颜色判定"}
                             for x in res["skipped"]],
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
        if it.get("resumed"):
            print("\n== 块 #%d / %s —— 上一遍已完成（exec_rc=0），跳过 =="
                  % (it["block_idx"], it["class"]))
            continue
        print("\n== 块 #%d / %s / 臂 %s ==" % (it["block_idx"], it["class"],
                                               it["arm"]))
        # 逐块重规划：起点必须是**此刻实测的关节角**。三条计划共用抓帧时的
        # q_start 会在第一块放完后全部被闸门拒掉（2026-10-27 B2 实测）。
        js_now = board.joints(os.path.join(cdir, "joints-live-%02d" % k))
        try:
            pick_plan(path, analysis["_candidates_json"], it["block_idx"],
                      it["slot_mm"], it, args.urdf, frame_dir,
                      q_start=chain_q(js_now))
        except (RuntimeError, ValueError) as exc:
            print("!! 重规划失败（块 #%d）：%s" % (it["block_idx"], exc))
            round_doc["stop_reason"] = "重规划失败（块 #%d）" % it["block_idx"]
            return 1
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

    return terminal_measure(board, cdir, rdir, round_no, grid,
                            grid_path, scene, res["items"], args)


if __name__ == "__main__":
    sys.exit(main())
