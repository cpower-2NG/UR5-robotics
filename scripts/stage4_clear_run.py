#!/usr/bin/env python3
"""E9 真机半段：把探针的「清障方案」变成一份**按顺序可执行**的抓放计划序列。

本模块**纯离线**：只读探针报告 + 场景真值，产出计划 JSON 与执行脚本；
不 import ROS、不发轨迹、不写夹爪寄存器。真执行是板上
``precision_exec.py --go`` 的事（每条动作都要当次运动令牌）。

分工（为什么这样切）
--------------------
``stage4_clear_probe`` 回答「**存不存在**一个合法方案」：目标可抓 ∧ 清障 ∧
放置位姿合法 ∧ 终态合法，夹爪间隙用自有 OBB+SAT 逐个候审判。它不产生轨迹。
本模块回答「**怎么把那个方案做出来**」：把每一步清障与最终目标翻译成
``precision_plan.build_pick`` 的 A3 抓放几何（竖直抓、18.5 mm 夹持深度、
整链选支、高度护栏），``q_start`` 逐段衔接。

一个必须说清的例外
------------------
``build_pick`` 有一条粗判据「被抓块离最近邻块中心距 ≥60 mm」。E9 的目标
恰恰是**贴面/邻块**场景：探针已经用 OBB 判过「这个候选位姿夹爪不碰」，
它比中心距细（贴面块只要在合拢轴方向让开就能抓）。所以这里显式给
``min_block_gap=0.0``，并在 manifest 里写 ``gap_check: probe_verified``。
**没有探针判决的调用方不许这样放宽。**

跑后核验（``--verify``）另读两份输入：跑后深度真值 json，和（可选的）跑后
俯视 burst 目录。目标块「落在指定格」用**彩色顶面质心**量 —— 垫面在俯视深度
里分辨不出 29.5 mm 台阶（2026-11-08 实测，见 ``measure_target_colour``）；
其余判据仍用深度真值。仍不 import ROS。

指令顺序（每条都在 manifest 里可见）
------------------------------------
1. 先按探针给的顺序执行清障步骤（``clearing_steps``：谁 → 从哪 → 到哪）；
2. 最后执行目标块 → 指定宫格；
3. 每步内部固定：approach → close → lift → to_slot → place_approach →
   place(open) → retreat → survey（``build_pick`` 的老航点序列，不另立）。

用法（开发机，链式 q_start）
---------------------------
    python3 scripts/stage4_clear_run.py \\
        --probe-report runs/<run>/e9/<scene>-probe.json \\
        --urdf ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf \\
        --from-frame <joint_states.json 所在目录> \\
        --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<run>/e9-plan/<scene>

退出码：0 = 全部计划写出且预检通过；1 = 某一步规划失败（不写半份序列）；
2 = 输入缺失/不可用。
"""
import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import precision_plan as pp                                       # noqa: E402
import stage4_clear_model as cm                                   # noqa: E402
import stage4_truth_from_frame as tf                              # noqa: E402
import urkin                                                      # noqa: E402

#: 收尾巡检位（base 系，m）。用阶段 4 工作区上方的空位，不用阶段 2 的
#: ``SURVEY_TOOL0["place"]``（那在 y≈−0.39 m，每步白跑一趟）。
#: tool0 z=0.40 ⇒ 指尖 0.196 m，高过任何 29.5 mm 块；x/y 在拾取区上方。
E9_SURVEY_TOOL0 = (0.520, 0.300, 0.400)


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 输入解析
def load_scene(report, scene_index, truth_override=None):
    scenes = report.get("scenes") or []
    if not scenes:
        raise ValueError("探针报告里没有 scenes")
    if not (0 <= scene_index < len(scenes)):
        raise ValueError("--scene-index 越界：%d（共 %d 场）"
                         % (scene_index, len(scenes)))
    sc = scenes[scene_index]
    truth_path = truth_override or sc["truth"]
    if truth_override:
        sc = dict(sc, truth=truth_override)
    with open(truth_path, encoding="utf-8") as f:
        truth = json.load(f)
    return sc, truth


def pick_attempt(scene, target=None):
    attempts = scene.get("attempts") or []
    ok = [a for a in attempts if a.get("ok")]
    if target is not None:
        for a in ok:
            if str(a.get("id")) == str(target):
                return a
        raise ValueError("目标 %r 在报告里没有成功的尝试" % (target,))
    if not ok:
        raise ValueError("这一场没有成功的尝试（ok=True）")
    return ok[0]


# ---------------------------------------------------------------- 几何
def grasp_yaw_deg(hit, block_yaw):
    """探针命中的抓取候选 → ``build_pick`` 的 tool0 绕 z 角。

    ``grasp_candidates`` 的约定：tool0 z = 接近方向 a，tool0 x = 合拢轴 c。
    B 档竖直抓只可能是 a=(0,0,−1)（axis=2, sign=−1）。``closing`` 是块局部轴：
    0 → c = 块 x 轴 ⇒ ψ = 块 yaw；1 → c = 块 y 轴 ⇒ ψ = 块 yaw + 90°。
    ``variant`` 是绕 a 转 180°，对方块在几何上等价（``fold_yaw_180`` 会折掉）。
    """
    if not hit:
        raise ValueError("没有抓取命中记录")
    if not hit.get("vertical") or int(hit.get("axis", -1)) != 2 \
            or int(hit.get("sign", 0)) != -1:
        raise ValueError("E9 只执行 B 档竖直抓，收到 axis=%r sign=%r vertical=%r"
                         % (hit.get("axis"), hit.get("sign"), hit.get("vertical")))
    closing = int(hit.get("closing", 0))
    if closing not in (0, 1):
        raise ValueError("closing 只认 0/1，收到 %r" % (closing,))
    return float(block_yaw) + (0.0 if closing == 0 else 90.0)


def find_block(blocks, bid):
    for b in blocks:
        if str(b.get("id")) == str(bid):
            return b
    return None


def move_in_layout(blocks, bid, x, y, z_bottom):
    """就地更新布局里的一块（不改原 truth）。``top`` 是块顶离台面高度。"""
    b = find_block(blocks, bid)
    b["x"], b["y"] = float(x), float(y)
    b["z"] = float(z_bottom) + float(b["size"]) / 2.0
    b["top"] = float(z_bottom) + float(b["size"])
    return b


def surface_of(to_z, size, where):
    """落点块心 z → 支撑面高度；只认台面与垫面两种（多认就是把错当真）。"""
    z = float(to_z) - float(size) / 2.0
    for cand in (0.0, float(cm.PAD_THICK)):
        if abs(z - cand) <= 1e-4:
            return cand
    raise ValueError("%s 的落点块心 z=%.4f 推不出支撑面（得到 %.4f m，"
                     "既不是台面也不是垫面）" % (where, float(to_z), z))


def actions_from_attempt(truth, attempt, pad_thick):
    """探针的一次成功尝试 → 有序动作表（清障在前，目标在最后）。

    返回的 ``blocks`` 是**原始布局**（不预先搬运）：搬运由
    ``plan_sequence`` 在**算完那一步之后**才施加 —— 否则清障那一步会按
    「块已经躺在落点」去算，臂会跑去错的地方。
    起点用探针记录的 ``step["from"]``（那一刻的实测位姿），不用布局推算。
    """
    blocks = [dict(b) for b in truth["blocks"]]
    actions = []
    for step in attempt.get("clearing_steps") or []:
        bid = step["move"]
        blk = find_block(blocks, bid)
        if blk is None:
            raise ValueError("清障步骤引用了不存在的块 %r" % (bid,))
        src, to = step["from"], step["to"]
        surf = surface_of(to[2], blk["size"], "清障块 %r" % (bid,))
        actions.append({
            "kind": "clear",
            "block": str(bid),
            "from_xy": [float(src[0]), float(src[1])],
            "from_top": float(src[2]) + float(blk["size"]) / 2.0,
            "slot": [float(to[0]), float(to[1])],
            "surface_z": surf,
            "grasp_yaw_deg": grasp_yaw_deg(step.get("grasp"),
                                           blk.get("yaw", 0.0)),
            "grasp_hit": {k: step["grasp"].get(k) for k in
                          ("axis", "sign", "closing", "variant", "verdict")}
            if step.get("grasp") else None,
        })

    tid = str(attempt["id"])
    tgt = find_block(blocks, tid)
    if tgt is None:
        raise ValueError("真值里没有目标块 %r" % (tid,))
    final = find_block(attempt.get("final_blocks") or [], tid)
    if final is None:
        raise ValueError("尝试记录里没有目标块 %r 的终态位姿" % (tid,))
    surf = surface_of(final["z"], tgt["size"], "目标块 %r" % (tid,))
    if abs(surf - float(pad_thick)) > 1e-4:
        raise ValueError("目标块 %r 的落点支撑面 %.4f m 不是垫面 %.4f m —— "
                         "探针与执行器对放置面的理解不一致"
                         % (tid, surf, float(pad_thick)))
    actions.append({
        "kind": "target",
        "block": tid,
        "from_xy": [float(tgt["x"]), float(tgt["y"])],
        "from_top": float(tgt["top"]),
        "slot": [float(final["x"]), float(final["y"])],
        "surface_z": surf,                  # 目标放到垫上
        "grasp_yaw_deg": grasp_yaw_deg(attempt.get("grasp_hit"),
                                       tgt.get("yaw", 0.0)),
        "grasp_hit": {k: attempt["grasp_hit"].get(k) for k in
                      ("axis", "sign", "closing", "variant", "verdict")},
    })
    return actions, blocks


# ---------------------------------------------------------------- 计划
def to_pp_block(b):
    """真值块（m）→ precision_plan 的块字典（cx/cy/top 单位 mm）。"""
    size_mm = float(b["size"]) * 1000.0
    top_mm = float(b["top"]) * 1000.0
    return {"idx": int(b["id"]),
            "class": b.get("color") or b.get("class") or "block",
            "cx": float(b["x"]) * 1000.0,
            "cy": float(b["y"]) * 1000.0,
            "top": top_mm,
            "h": top_mm,
            "size": size_mm,
            "yaw": float(b.get("yaw", 0.0))}


def plan_sequence(chain, q_start, actions, blocks, out_dir,
                  survey_tool0=E9_SURVEY_TOOL0, place_clearance=None):
    """逐步生成计划；任何一步预检失败就抛 ``pp.PlanError``，不写半份序列。"""
    layout = [dict(b) for b in blocks]
    q = [float(v) for v in q_start]
    plans = []
    for i, act in enumerate(actions, start=1):
        blk = find_block(layout, act["block"])
        if blk is None:
            raise ValueError("布局里没有块 %r" % (act["block"],))
        pp_blocks = [to_pp_block(b) for b in layout]
        # ``build_pick`` 只从 cand 里取两样真东西：抓点 xy（这里不用 ——
        # ``grasp_at="block_center"``）与「候选离块心 ≤12 mm」的检查。
        # 其余字段（score/width/angle）是阶段 2 的 GraspNet 元数据，E9 没有，
        # 填的是几何真值而不是编的分数：width = 块边长，angle = 合拢轴角。
        cand = {"xyz_base": [float(blk["x"]), float(blk["y"]),
                             float(blk.get("top", blk["size"]))],
                "score": 0.0,
                "width_mm": float(blk["size"]) * 1000.0,
                "dist_mm": 0.0,
                "angle_deg": float(act["grasp_yaw_deg"]),
                "seed": None}
        wps, checks, meta = pp.build_pick(
            chain, q, cand, to_pp_block(blk), pp_blocks, act["slot"],
            yaw_grasp_deg=act["grasp_yaw_deg"], grasp_at="block_center",
            surface_z=act["surface_z"], survey_tool0=tuple(survey_tool0),
            place_clearance=place_clearance,
            # 压在目标上的块：`h`（块顶离台面）≠ 物理块高，必须显式给
            # 物理块高，否则抓取高度按两个块高算（= 2026-11-02 拆塔事故）。
            block_h_m=float(blk["size"]),
            # 见模块头：探针已用 OBB 判过间隙，这里的 60 mm 粗判据必须让位。
            min_block_gap=0.0)
        plan = {"version": 2,
                "meta": dict(meta,
                             e9={"kind": act["kind"], "block": act["block"],
                                 "slot": list(act["slot"]),
                                 "from_xy": list(act["from_xy"]),
                                 "grasp_yaw_deg": round(
                                     float(act["grasp_yaw_deg"]), 3),
                                 "gap_check": "probe_verified",
                                 "checks": checks}),
                "q_start": list(q),
                "waypoints": wps}
        name = "%02d-%s-block-%s.json" % (i, act["kind"], act["block"])
        path = os.path.join(out_dir, name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(plan, f, ensure_ascii=False, indent=1)
        plans.append({"index": i, "name": name, "path": path,
                      "kind": act["kind"], "block": act["block"],
                      "slot": act["slot"], "from_xy": act["from_xy"],
                      "grasp_yaw_deg": act["grasp_yaw_deg"],
                      "n_waypoints": len(wps)})
        q = [float(v) for v in wps[-1]["q"]]
        move_in_layout(layout, act["block"], act["slot"][0], act["slot"][1],
                       act["surface_z"])
    return plans, layout, q


def _match_blocks(exp, post):
    """两份真值按**三维最近位置**一一配对，返回 ``(match, missing, extra)``。

    **不能按 id 配对**：id 只是同一份真值内部的序号，两份真值各自按位置重排
    （2026-11-08 实测：跑后真值的 id 顺序和计划里的 ``final_layout`` 完全不同，
    按 id 对齐会把紫塔的块配到橙塔上、报出 240 mm 的假位移）。
    配对之后也不比颜色 —— 塔底块的颜色取自塔顶面，真值里写着塔顶的颜色。
    """
    pairs = []
    for ei, e in exp.items():
        for pi, p in post.items():
            d = math.sqrt((float(p["x"]) - float(e["x"])) ** 2
                          + (float(p["y"]) - float(e["y"])) ** 2
                          + (float(p["z"]) - float(e["z"])) ** 2) * 1000.0
            pairs.append((d, ei, pi))
    pairs.sort()
    used_e, used_p, match = set(), set(), {}
    for d, ei, pi in pairs:            # 全局按距离贪心，一对一
        if ei in used_e or pi in used_p:
            continue
        used_e.add(ei)
        used_p.add(pi)
        match[ei] = (pi, d)
    return (match, sorted(set(exp) - used_e, key=lambda s: (len(s), s)),
            sorted(set(post) - used_p, key=lambda s: (len(s), s)))


# ------------------------------------------------------------ 目标块彩色量具
#: 彩色门限：白垫 s≈0、黑标记 v≈0，都被这两条排除。
COLOUR_S_MIN, COLOUR_V_MIN = 90.0, 60.0
#: 顶面取「窗内最亮的一批」：顶面比侧面亮，取最亮 40% 让质心落在顶面，
#: 不被侧面的透视拉伸拖偏（2026-11-08 实测 10 帧极差 ≤7 mm）。
COLOUR_TOP_PCT = 40.0
#: 一帧至少要这么多个色彩像素才算「看见目标块」。29.5 mm 的块在垫面约 16×16 px
#: ⇒ 顶面 250 px 量级；150 只在远/斜视时触发。
COLOUR_MIN_PX = 150


def hsv_of(img):
    """RGB uint8 图像 → ``(h 度 0–360, s 0–255, v 0–255)``。"""
    a = np.asarray(img).astype(np.float32) / 255.0
    mx, mn = a.max(2), a.min(2)
    d = mx - mn
    h = np.zeros_like(mx)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    m = d > 1e-6
    i = m & (mx == r)
    h[i] = (60.0 * ((g[i] - b[i]) / d[i])) % 360.0
    i = m & (mx == g)
    h[i] = 60.0 * ((b[i] - r[i]) / d[i]) + 120.0
    i = m & (mx == b)
    h[i] = 60.0 * ((r[i] - g[i]) / d[i]) + 240.0
    s = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0) * 255.0
    return h, s, mx * 255.0


def _hue_peak(h, step=15.0):
    """主色相簇 → ``(簇中心角, 落在峰 ±1 bin 内的掩码)``。

    只取峰附近、不取「窗口里所有彩色像素」：窗口里万一还有别的颜色的东西
    （邻塔、反光），按整窗平均会把两样东西的颜色混成一个不存在的色。

    簇中心用**保留像素的圆均值**、不用 bin 中心：一个窄色簇全落在同一 bin
    时，bin 中心最多差半步（实测偏 15°），而且环形平滑会让相邻 bin 打平。
    """
    n = int(round(360.0 / step))
    idx = (np.mod(h, 360.0) / step).astype(int) % n
    cnt = np.bincount(idx, minlength=n).astype(int)
    p = int(np.argmax(cnt))
    keep = np.zeros(n, bool)
    keep[(p - 1) % n] = keep[p] = keep[(p + 1) % n] = True
    m = keep[idx]
    ang = np.radians(h[m])
    mean = math.degrees(math.atan2(float(np.sin(ang).mean()),
                                  float(np.cos(ang).mean()))) % 360.0
    return mean, m


def colour_blob_uv(img, uv0, half_px, min_px=COLOUR_MIN_PX):
    """窗口里「一坨有颜色的东西」的**顶面质心**像素坐标；看不见 ⇒ ``None``。

    ``uv0`` = 窗口中心（计划宫格在块顶高的投影），``half_px`` = 半窗宽。
    返回 ``{"uv", "n_px", "n_top", "hue_deg"}``。

    **看不见就返回 None，不猜**：调用方必须把 None 判红，不许拿计划位当量值。
    """
    h, s, v = hsv_of(img)
    H, W = h.shape
    r = int(round(float(half_px)))
    u0, v0 = int(round(float(uv0[0]))), int(round(float(uv0[1])))
    ua, ub = max(0, u0 - r), min(W, u0 + r + 1)
    va, vb = max(0, v0 - r), min(H, v0 + r + 1)
    if ub - ua < 5 or vb - va < 5:
        return None
    sh, ss, sv = h[va:vb, ua:ub], s[va:vb, ua:ub], v[va:vb, ua:ub]
    m = (ss >= COLOUR_S_MIN) & (sv >= COLOUR_V_MIN)
    if int(m.sum()) < min_px:
        return None
    peak, keep = _hue_peak(sh[m])
    mm = np.zeros_like(m)
    mm[m] = keep
    if int(mm.sum()) < min_px:
        return None
    ys, xs = np.nonzero(mm)
    vv = sv[ys, xs]
    # 最亮 40%（分位取 100−40）：平坦面（整块同色）分位就等于该值 ⇒ 全取，
    # 不会被阈值空掉；真实块上这就是顶面，侧面更暗、被排除。
    top = vv >= np.percentile(vv, 100.0 - COLOUR_TOP_PCT)
    if int(top.sum()) < 20:
        return None
    return {"uv": (ua + float(xs[top].mean()), va + float(ys[top].mean())),
            "n_px": int(mm.sum()), "n_top": int(top.sum()),
            "hue_deg": round(peak, 1)}


def _base_to_uv(p, K, R, t):
    """base 系一点 → 俯视像素（``ray_plane_xy`` 的逆）。在相机后方 ⇒ None。"""
    pc = np.asarray(R, float).T @ (np.asarray(p, float) - np.asarray(t, float))
    if pc[2] <= 1e-6:
        return None
    uv = np.asarray(K, float) @ pc
    return (float(uv[0] / uv[2]), float(uv[1] / uv[2]))


def _window_half_px(xy, z, half_mm, K, R, t):
    """以 (x, y, z) 为心、边长 2·half_mm 的正方形投影到像面的半窗宽（px）。"""
    uv0 = _base_to_uv((xy[0], xy[1], z), K, R, t)
    if uv0 is None:
        return None
    h = half_mm * 0.001
    cors = [_base_to_uv((xy[0] + dx * h, xy[1] + dy * h, z), K, R, t)
            for dx in (-1.0, 1.0) for dy in (-1.0, 1.0)]
    cors = [c for c in cors if c is not None]
    if not cors:
        return None
    return max(8.0, max(max(abs(c[0] - uv0[0]), abs(c[1] - uv0[1]))
                        for c in cors))


def measure_target_colour(burst_dir, slot_xy, top_z, search_mm=60.0,
                          max_frames=10, min_frames=5):
    """跑后俯视 burst → 目标块顶面质心（台面 mm，中位）与可复现性。

    **为什么目标块要换彩色量**（2026-11-08 用户拍板）：垫面在俯视深度里
    分辨不出 29.5 mm 的台阶 —— 实测 10 帧只有 3 帧有真台阶，跑前同一带本来
    就有 15–30 mm 的幻影穹顶，``find_blocks`` 把块并进 48×46 mm 的一坨、
    质心被拉偏 33 mm（场景 2 因此被深度判成「没落格」，实际落在格内）。
    同一块在彩色里清清楚楚。**其余判据仍用深度真值**：块数、旁观块、终态、
    清空块都在台面上，深度是准的。

    每帧：窗内取色彩像素 → 主色相簇 → 最亮 40%（顶面）质心 → 射线 ∩
    块顶水平面。跨帧取中位；``ok`` 要求至少 ``min_frames`` 帧看见。

    返回 ``{"ok", "xy_mm", "hue_deg", "n_frames", "n_missed", "miss_reasons",
    "per_frame", "search_mm", "top_z"}``。漏帧不静默：每一帧漏的原因都写下来
    （缺静态 TF / 窗内没看见 / 投不到像面），``ok`` 只看看得见的帧数够不够。
    """
    import glob
    paths = sorted(glob.glob(os.path.join(burst_dir, "color_*.png")))[:max_frames]
    rows, miss, reasons = [], 0, []
    for path in paths:
        idx = int(os.path.basename(path).rsplit("_", 1)[-1].split(".")[0])
        try:
            fr = gc.load_frame(burst_dir, tip=gc.TOPCAM_TIP,
                               chain=gc.TOPCAM_CHAIN, idx=idx)
        except KeyError as e:
            # 起栈后的头一两帧可能还没收到锁存的 ``base_link:topcam_link``
            # 静态 TF（2026-11-08 实测）。记成漏帧，别让核验整个崩掉。
            miss += 1
            reasons.append("帧%d 缺静态 TF（%s）" % (idx, e))
            continue
        K = np.asarray(fr["K"], float)
        R = np.asarray(fr["R_bc"], float)
        t = np.asarray(fr["t_bc"], float)
        uv0 = _base_to_uv((slot_xy[0], slot_xy[1], top_z), K, R, t)
        hp = (None if uv0 is None
              else _window_half_px(slot_xy, top_z, search_mm, K, R, t))
        if hp is None:
            miss += 1
            reasons.append("帧%d 计划宫格投不到像面" % idx)
            continue
        with Image.open(path) as im:
            img = np.asarray(im.convert("RGB"))
        blob = colour_blob_uv(img, uv0, hp)
        if blob is None:
            miss += 1
            reasons.append("帧%d 窗内没看见彩色块" % idx)
            continue
        xy = tf.ray_plane_xy(list(blob["uv"]), K, R, t, float(top_z))
        if xy is None:
            miss += 1
            reasons.append("帧%d 射线与块顶平面不相交" % idx)
            continue
        rows.append({"frame": idx, "uv": [round(c, 2) for c in blob["uv"]],
                     "x_mm": round(float(xy[0]) * 1000.0, 2),
                     "y_mm": round(float(xy[1]) * 1000.0, 2),
                     "n_px": blob["n_px"], "hue_deg": blob["hue_deg"]})
    out = {"ok": len(rows) >= max(int(min_frames), 1),
           "search_mm": search_mm, "top_z": top_z,
           "n_frames": len(rows), "n_missed": miss,
           "miss_reasons": reasons, "per_frame": rows}
    if rows:
        a = np.array([[r["x_mm"], r["y_mm"]] for r in rows], float)
        med = np.median(a, axis=0)
        out["xy_mm"] = [round(float(med[0]), 2), round(float(med[1]), 2)]
        out["max_dev_mm"] = round(float(np.hypot(a[:, 0] - med[0],
                                                 a[:, 1] - med[1]).max()), 2)
        out["range_mm"] = [round(float(a[:, 0].max() - a[:, 0].min()), 2),
                           round(float(a[:, 1].max() - a[:, 1].min()), 2)]
        sn = sum(math.sin(math.radians(r["hue_deg"])) for r in rows)
        cs = sum(math.cos(math.radians(r["hue_deg"])) for r in rows)
        out["hue_deg"] = round(math.degrees(math.atan2(sn, cs)) % 360.0, 1)
    return out


def compare_layout(manifest, post_blocks, tol_untouched_mm=5.0,
                   tol_untouched_deg=3.0, tol_moved_mm=40.0,
                   target_colour=None):
    """跑完再拍一遍真值 → 判「不丢指定块 / 不掉块 / 不撞」。

    三条判据各自的量（都不靠「看起来对」）：

    · **不丢指定块**：目标块的终态必须落在计划的那一格（``cell_index_of``）。
      **量具是彩色顶面质心**（``target_colour``，见 ``measure_target_colour``）：
      垫面在俯视深度里分辨不出 29.5 mm 台阶，深度真值在这里会假红。给了
      ``target_colour`` 而它 ``ok=False``（看得见的帧不够）⇒ 直接判红，
      **不退回深度、也不拿计划位当量值**。深度给出的同一位置写进 ``notes``
      供对照，只记录、不判。
    · **不掉块**：``stage4_clear_model.terminal_report`` —— 每块有真支撑
      （台面 / 垫面 / 真的压在另一块上）、不穿插、不出桌。掉到地上 = 不在
      台面支撑里，必然红。
    · **不撞**：**没被碰的块不许动** —— 按**位移**判（``tol_untouched_mm``）。
      静置的块被推动只有一种原因：臂或夹爪碰到了它。清障块本身有 40 mm 的落点
      公差（抓取/落点误差），不加严。这条是间接判据，所以报告里把每块的位移
      都列出来，别只看结论。

      **转角只记录、不判红**（2026-11-08 用户拍板）。依据是实测的重复性：臂不动、
      同一场景连拍两组 burst，深度真值的块 yaw 自己就差 4.5°（塔）到 36°（单块）
      —— 3° 的公差**低于测量底噪**，判了只会假红。同一场景的彩色剪影取向只差
      1–2°，可见这是深度口径的问题，不是块真的在转。位移的重复性是 0.2–1.7 mm，
      5 mm 的公差撑得住。要恢复 yaw 判据，先换测量口径（见台账第二百八十四轮）。

    计划里的块与跑后真值里的块**按三维位置配对**（见 ``_match_blocks``），
    不按 id、不比颜色。

    返回 ``{"ok": bool, "checks": [...], "blocks": [...], "terminal": ...}``。
    """
    exp = {str(b["id"]): b for b in manifest.get("final_layout") or []}
    post = {str(b["id"]): b for b in post_blocks}
    moved = {str(a["block"]) for a in manifest.get("actions") or []}
    match, missing, extra = _match_blocks(exp, post)
    checks, rows = [], []
    ok = True

    def add(name, good, detail):
        nonlocal ok
        ok = ok and bool(good)
        checks.append({"check": name, "ok": bool(good), "detail": detail})

    add("块数一致（没丢块、没多块）", not missing and not extra,
        "缺 %s / 多 %s" % (missing or "无", extra or "无"))

    rep = cm.terminal_report(post_blocks)
    add("全桌终态合法（不悬空、不穿插、不出桌）", rep["ok"],
        "overlaps=%d off_table=%s out_of_bounds=%s"
        % (len(rep.get("overlaps") or []), rep.get("off_table"),
           rep.get("out_of_bounds")))

    tid = str(manifest.get("target"))
    cell = manifest.get("cell")
    notes = []
    if target_colour is not None and cell is not None:
        name = "指定块落在计划的那一格（#%s，彩色顶面质心）" % cell
        if not target_colour.get("ok"):
            add(name, False,
                "彩色量不够帧（%d 帧看见 / 需 ≥5，漏 %d 帧）—— 没有量值，不判过"
                % (target_colour.get("n_frames") or 0,
                   target_colour.get("n_missed") or 0))
        else:
            tx, ty = [float(v) / 1000.0 for v in target_colour["xy_mm"]]
            got = cm.cell_index_of(tx, ty)
            cx, cy = cm.cell_center(int(cell))
            rng = target_colour.get("range_mm") or [0.0, 0.0]
            add(name, got == int(cell),
                "实际格 %r，位置 (%.1f, %.1f) mm，距格心 %.1f mm"
                "（%d 帧中位；极差 x=%.1f / y=%.1f mm；单帧最大偏离 %.1f mm；"
                "色相 %.0f°；窗 ±%.0f mm）"
                % (got, tx * 1000, ty * 1000,
                   math.hypot(tx - cx, ty - cy) * 1000.0,
                   target_colour.get("n_frames") or 0, rng[0], rng[1],
                   target_colour.get("max_dev_mm") or 0.0,
                   target_colour.get("hue_deg") or 0.0,
                   target_colour.get("search_mm") or 0.0))
        if tid in match:
            db = post[match[tid][0]]
            dgot = cm.cell_index_of(float(db["x"]), float(db["y"]))
            notes.append("同一位置深度真值给出 (%.1f, %.1f) mm → 格 %r"
                         % (float(db["x"]) * 1000, float(db["y"]) * 1000, dgot))
        else:
            notes.append("深度真值里配不上目标块 #%s" % tid)
    elif tid in match and cell is not None:
        pi = match[tid][0]
        tb = post[pi]
        got = cm.cell_index_of(float(tb["x"]), float(tb["y"]))
        add("指定块落在计划的那一格（#%s）" % cell, got == int(cell),
            "实际格 %r，位置 (%.1f, %.1f) mm"
            % (got, float(tb["x"]) * 1000, float(tb["y"]) * 1000))
    elif tid not in match:
        add("指定块落在计划的那一格（#%s）" % cell, False,
            "跑后真值里找不到目标块 #%s（已丢？）" % tid)

    for bid in sorted(match, key=lambda s: (len(s), s)):
        pi, d3 = match[bid]
        b, e = post[pi], exp[bid]
        d = math.hypot(float(b["x"]) - float(e["x"]),
                       float(b["y"]) - float(e["y"])) * 1000.0
        # 积木是正方形：yaw 与 yaw+90° 是同一个盒子，必须按 90° 折角比
        dyaw = abs((float(b.get("yaw", 0.0)) - float(e.get("yaw", 0.0))
                    + 45.0) % 90.0 - 45.0)
        row = {"id": bid, "post_id": pi, "moved": bid in moved,
               "d_mm": round(d, 2), "dyaw_deg": round(dyaw, 2),
               "dz_mm": round((float(b["z"]) - float(e["z"])) * 1000.0, 2)}
        rows.append(row)
        if bid in moved:
            add("被搬的块 #%s 落在计划位附近" % bid, d <= tol_moved_mm,
                "%.1f mm（公差 %.0f）" % (d, tol_moved_mm))
        else:
            # 只按位移判；转角写进 detail 供人看（口径见函数 docstring）
            add("没被碰的块 #%s 没被动过" % bid, d <= tol_untouched_mm,
                "位移 %.1f mm（公差 %.0f）；转角 %.1f°（只记录，不判）"
                % (d, tol_untouched_mm, dyaw))
    for pi in extra:
        rows.append({"id": None, "post_id": pi, "moved": False,
                     "d_mm": None, "dyaw_deg": None, "dz_mm": None})
    return {"ok": ok, "checks": checks, "blocks": rows, "terminal": rep,
            "notes": notes,
            "tolerances": {"untouched_mm": tol_untouched_mm,
                           "untouched_deg": tol_untouched_deg,
                           "moved_mm": tol_moved_mm}}


def verify_main(args):
    """跑后核验：读真值 + manifest，判「不丢指定块 / 不掉块 / 不撞」。"""
    mpath = args.manifest or os.path.join(args.out_dir or "", "manifest.json")
    with open(mpath, encoding="utf-8") as f:
        manifest = json.load(f)
    with open(args.verify, encoding="utf-8") as f:
        post = json.load(f)
    colour = None
    if args.colour_burst:
        exp = {str(b["id"]): b for b in manifest.get("final_layout") or []}
        slot = exp.get(str(manifest.get("target")))
        if slot is None:
            log("!! manifest 的 final_layout 里没有目标块，算不出彩色量的窗口")
            return 2
        size = float(slot.get("size") or cm.PAD_PITCH)
        top_z = float(slot["z"]) + size / 2.0
        colour = measure_target_colour(
            args.colour_burst, (float(slot["x"]), float(slot["y"])), top_z,
            search_mm=args.colour_search_mm)
        if colour.get("ok"):
            rng = colour.get("range_mm") or [0.0, 0.0]
            log("  彩色量：%d/%d 帧看见，中位 (%.1f, %.1f) mm，极差 x=%.1f "
                "y=%.1f mm，色相 %.0f°，块顶高 %.1f mm"
                % (colour["n_frames"], colour["n_frames"] + colour["n_missed"],
                   colour["xy_mm"][0], colour["xy_mm"][1], rng[0], rng[1],
                   colour["hue_deg"], top_z * 1000.0))
        else:
            log("  彩色量：只看见 %d 帧（需 ≥5），漏 %d 帧 —— 判红"
                % (colour["n_frames"], colour["n_missed"]))
    rep = compare_layout(manifest, post["blocks"],
                         tol_untouched_mm=args.untouched_mm,
                         tol_untouched_deg=args.untouched_deg,
                         tol_moved_mm=args.moved_mm,
                         target_colour=colour)
    out_dir = args.out_dir or os.path.dirname(mpath)
    vpath = os.path.join(out_dir, "verify.json")
    with open(vpath, "w", encoding="utf-8") as f:
        json.dump(dict(rep, manifest=mpath, post_truth=args.verify,
                       colour=colour), f, ensure_ascii=False, indent=1)
    for c in rep["checks"]:
        log("  %s %s   %s" % ("PASS" if c["ok"] else "FAIL",
                              c["check"], c["detail"]))
    for n in rep.get("notes") or []:
        log("    注：%s" % n)
    for b in rep["blocks"]:
        log("    块 %-3s→%-3s %s  位移 %s mm / 转角 %s° / Δz %s mm"
            % (b["id"], b.get("post_id"), "被搬" if b["moved"] else "未动",
               b.get("d_mm"), b.get("dyaw_deg"), b.get("dz_mm")))
    log("→ %s" % vpath)
    log("结论：%s" % ("PASS" if rep["ok"] else "FAIL"))
    return 0 if rep["ok"] else 1


def run_script_text(plans, board_dir, log_dir):
    lines = ["#!/bin/bash",
             "# E9 自主移遮挡：按顺序执行本场计划。任何一步 rc≠0 立即停",
             "# （fail-closed，不跳过、不重试）。令牌 = 当次 STOP→PLAY 握手产物。",
             "set -u",
             'TOKEN="$1"',
             '[ -n "${TOKEN:-}" ] || { echo "用法: $0 <token>"; exit 2; }',
             'fail() { echo "!!!!! STOP: $*"; exit 9; }']
    for p in plans:
        lines.append('echo "===== %s : exec"' % p["name"])
        lines.append(
            '~/precision/run.sh ~/precision/precision_exec.py '
            '--plan %s/%s --go --require-auth "$TOKEN" '
            '--log %s/%s.jsonl' % (board_dir, p["name"], log_dir, p["name"]))
        lines.append('rc=$?')
        lines.append('[ $rc -eq 0 ] || fail "%s rc=$rc"' % p["name"])
        lines.append('sleep 1')
    lines.append('echo "ALL DONE"')
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="E9 真机半段：探针方案 → 可执行计划序列（离线）")
    ap.add_argument("--probe-report", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--verify", default=None,
                    help="跑后真值 json；给了就只做「不丢/不掉/不撞」核验，不生成计划")
    ap.add_argument("--manifest", default=None,
                    help="配合 --verify：manifest 路径（缺省 = --out-dir/manifest.json）")
    ap.add_argument("--untouched-mm", type=float, default=5.0,
                    help="没被碰的块允许的最大位移（mm，缺省 5）")
    ap.add_argument("--untouched-deg", type=float, default=3.0,
                    help="没被碰的块允许的最大转角（°，缺省 3）")
    ap.add_argument("--moved-mm", type=float, default=40.0,
                    help="被搬的块离计划落点的最大距离（mm，缺省 40）")
    ap.add_argument("--colour-burst", default=None,
                    help="跑后俯视 burst 目录；给了就用彩色顶面质心量目标块落格"
                         "（垫面深度分辨不出 29.5 mm 台阶；缺省 None = 用深度真值）")
    ap.add_argument("--colour-search-mm", type=float, default=60.0,
                    help="彩色量的搜索窗半径（mm，围绕计划宫格；缺省 60）")
    ap.add_argument("--scene-index", type=int, default=0)
    ap.add_argument("--truth", default=None,
                    help="覆盖报告里记的 truth 路径（报告在板上生成时是板上绝对路径）")
    ap.add_argument("--target", default=None,
                    help="指定目标块 id（默认取这一场第一个成功尝试）")
    ap.add_argument("--from-frame", default=None,
                    help="从帧目录的 joint_states.json 取 q_start")
    ap.add_argument("--q-start", default=None,
                    help="逗号分隔的 6 个关节角（弧度）")
    ap.add_argument("--pad-thick", type=float, default=None,
                    help="垫厚（m），目标块放垫上；缺省 = stage4_clear_model.PAD_THICK")
    ap.add_argument("--place-clearance", type=float, default=None,
                    help="放置时块底离表面间隙（m）；缺省 = precision_plan 的 5 mm")
    ap.add_argument("--survey", default=None,
                    help="收尾巡检位 tool0 x,y,z（m）；缺省 %s"
                         % (E9_SURVEY_TOOL0,))
    ap.add_argument("--board-dir", default="$HOME/precision/plans/e9",
                    help="板上计划目录（写进 run.sh）")
    ap.add_argument("--board-log-dir", default="$HOME/precision/logs/e9",
                    help="板上执行日志目录（写进 run.sh）")
    args = ap.parse_args(argv)

    if args.verify:
        return verify_main(args)

    if not (args.probe_report and args.out_dir):
        ap.error("生成计划要给 --probe-report 与 --out-dir（核验请用 --verify）")
    if not args.urdf:
        ap.error("生成计划要给 --urdf")

    with open(args.probe_report, encoding="utf-8") as f:
        report = json.load(f)
    scene, truth = load_scene(report, args.scene_index, args.truth)
    attempt = pick_attempt(scene, args.target)
    pad_thick = cm.PAD_THICK if args.pad_thick is None else float(args.pad_thick)
    actions, blocks = actions_from_attempt(truth, attempt, pad_thick)

    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame,
                                         "joint_states.json")))
        chain = urkin.Chain(args.urdf)
        q_start = [float(js["position"][js["name"].index(n)])
                   for n in chain.move_joints]
    elif args.q_start:
        chain = urkin.Chain(args.urdf)
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        ap.error("要给 --from-frame 或 --q-start")

    survey = (E9_SURVEY_TOOL0 if args.survey is None else
              tuple(float(v) for v in args.survey.split(",")))
    os.makedirs(args.out_dir, exist_ok=True)

    log("场景 %s:%s，目标块 %s，%d 个动作（清障 %d + 目标 1）"
        % (truth.get("batch"), truth.get("index"), attempt["id"],
           len(actions), len(actions) - 1))
    try:
        plans, final_layout, q_end = plan_sequence(
            chain, q_start, actions, blocks, args.out_dir,
            survey_tool0=survey, place_clearance=args.place_clearance)
    except pp.PlanError as e:
        log("!! 预检不过：%s" % e)
        log("   计划**没有**写出完整序列 —— 不要执行。")
        return 1

    manifest = {
        "probe_report": args.probe_report,
        "truth": scene["truth"],
        "batch": truth.get("batch"), "index": truth.get("index"),
        "target": attempt["id"],
        "scene_index": args.scene_index,
        "cell": report.get("cell"),
        "pad_thick": pad_thick,
        "survey_tool0": list(survey),
        "gap_check": "probe_verified",
        "actions": actions,
        "plans": plans,
        "q_start": q_start,
        "q_end": q_end,
        "final_layout": [{k: b.get(k) for k in
                          ("id", "x", "y", "z", "yaw", "size")}
                         for b in final_layout],
        "terminal": attempt.get("terminal"),
    }
    mpath = os.path.join(args.out_dir, "manifest.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)

    sh = run_script_text(plans, args.board_dir, args.board_log_dir)
    spath = os.path.join(args.out_dir, "run.sh")
    with open(spath, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(spath, 0o755)

    for p in plans:
        log("  %-28s %-6s 块 %-3s (%.3f, %.3f) → (%.3f, %.3f)  ψ=%6.1f°  %d 航点"
            % (p["name"], p["kind"], p["block"],
               p["from_xy"][0], p["from_xy"][1], p["slot"][0], p["slot"][1],
               p["grasp_yaw_deg"], p["n_waypoints"]))
    log("→ %s" % mpath)
    log("→ %s" % spath)
    return 0


if __name__ == "__main__":
    sys.exit(main())
