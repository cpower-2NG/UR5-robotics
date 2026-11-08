#!/usr/bin/env python3
"""E9 自主移遮挡的**纯几何**部分（不 import rclpy，秒级可单测）。

这个模块只回答四个问题，全部是「规划后的块位」上的几何判断：

1. **遮挡是谁**：从 E7 口径的候选位姿逐条结果里，统计哪些块在夹爪占据的空间
   里（``blocker_scores``）。
2. **自由区落点在哪**：``free_destinations`` 给出桌上可放的候选点 —— 在拾取区
   与放置垫之外，避免「挪开的块又挡住别人」或压到宫格。
3. **指定格在哪**：垫面上的 3×3 宫格（``cell_center`` / ``cell_contains``）。
   几何**不在这里发明**：垫心 / 垫偏航 / 宫格间距 / 厚度直接取
   ``arm_grasp_sim/scripts/gen_scene.py`` 的常量（仿真世界与真机垫是同一份
   实测值），自检会反查那份源码。
4. **终态是否合法**：``terminal_report`` —— 契约 §4.3 的「所有块在合法格 /
   自由区，不重叠、不落地、不越界」。⚠️ S3/S4 允许块间**接触**，所以判「重叠」
   必须用带符号的 ``sm.separation`` 加容差，**不能**用 ``sm.gap``（接触与穿插
   都返回 0，见 ``test_stage4_scene_model`` 里那条对照）。

不在这里的东西：IK / 碰撞服务、场景增删、递归清障的搜索 —— 那些要 ROS，
在 ``stage4_clear_probe.py``。
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_scene_model as sm                                  # noqa: E402


# ── 场景几何的**唯一家**：仿真场景生成器 ──────────────────────────────
def _load_gen_scene():
    """按源码路径 import ``arm_grasp_sim/scripts/gen_scene.py``。

    不走 ``ros2_ws/install``：改了源码不用 colcon build（与 sim.launch.py 的
    取向一致）。找不到就直接报错 —— 静默用一份抄来的常量会让「宫格在哪」
    悄悄和仿真世界分家。
    """
    root = os.path.dirname(HERE)          # 仓根（HERE = <仓根>/scripts）
    path = os.path.join(root, "ros2_ws", "src", "arm_grasp_sim", "scripts")
    if path not in sys.path:
        sys.path.insert(0, path)
    import gen_scene                                                # noqa: E402
    return gen_scene


_GS = _load_gen_scene()

BLOCK_M = _GS.BLOCK_M                               # 29.5 mm 方块
PICK_ZONE = tuple(_GS.ZONE)                         # (xmin,xmax,ymin,ymax)
PAD_CENTER = (float(_GS.PAD_CENTER_MM[0]) / 1000.0,
              float(_GS.PAD_CENTER_MM[1]) / 1000.0)
PAD_YAW_RAD = math.radians(_GS.PAD_YAW_DEG)
PAD_PITCH = float(_GS.PAD_PITCH_MM) / 1000.0        # 宫格 60 mm
PAD_THICK = float(_GS.PAD_THICK_M)                  # 垫厚 2.88 mm
PAD_HALF = 0.15                                     # 垫 300×300 mm

# 仿真桌面（1.15×1.10×0.04，位姿 (0.225, 0.10, −0.02)）。这份字面量在
# gen_scene.build_world 的 SDF 字符串里，没有名字；这里自检会反查源码，
# 改了桌子这个文件会红。
TABLE_CENTER = (0.225, 0.10)
TABLE_SIZE = (1.15, 1.10)
TABLE_XMIN = TABLE_CENTER[0] - TABLE_SIZE[0] / 2.0
TABLE_XMAX = TABLE_CENTER[0] + TABLE_SIZE[0] / 2.0
TABLE_YMIN = TABLE_CENTER[1] - TABLE_SIZE[1] / 2.0
TABLE_YMAX = TABLE_CENTER[1] + TABLE_SIZE[1] / 2.0

# ── 判据参数（E9 口径，改这里 = 改判据，须记录）──────────────────────
PEN_TOL = 0.0002      # 终态判「穿插」的容差 0.2 mm。S4 的接触对是精确贴面
                      # （数值上贴 0），任何真实叠放至少差几 mm —— 0.2 mm 把
                      # 浮点噪声和真穿插分得很开。
FREE_CLEAR = 0.005    # 挪到自由区的块，离别的块至少 5 mm（不贴面，免得又被算遮挡）
# 自由区：拾取区北侧的空台面（拾取区 y ≤ 0.42，垫在 y ≤ 0.038，都不碰）
FREE_X = (0.28, 0.75)
FREE_Y = (0.46, 0.62)
FREE_STEP = 0.04
FREE_ANCHOR = (0.515, 0.54)   # 落点排序锚点（带中央）


# ---------------------------------------------------------------- 垫宫格
def cell_center(index):
    """宫格 ``index``（0–8，行主序）的世界坐标 (x, y)。与 gen_scene 的
    ``_pad_grid_sdf`` 同一个式子：局部 ((c−1)·pitch, (1−r)·pitch) 再按垫偏航旋转。"""
    if not 0 <= index < 9:
        raise ValueError("宫格下标 0–8，收到 %r" % (index,))
    r, c = divmod(index, 3)
    lx = (c - 1) * PAD_PITCH
    ly = (1 - r) * PAD_PITCH
    ca, sa = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    return (PAD_CENTER[0] + ca * lx - sa * ly,
            PAD_CENTER[1] + sa * lx + ca * ly)


def to_pad_frame(x, y):
    """世界 (x, y) → 垫系局部 (u, v)。"""
    dx, dy = x - PAD_CENTER[0], y - PAD_CENTER[1]
    ca, sa = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    return (ca * dx + sa * dy, -sa * dx + ca * dy)


def cell_index_of(x, y):
    """(x, y) 落在哪个宫格里（按中心最近、且不超过半格距）；不在垫上给 None。"""
    u, v = to_pad_frame(x, y)
    c = round(u / PAD_PITCH) + 1
    r = 1 - round(v / PAD_PITCH)       # 局部 ly = (1−r)·pitch（行 0 在 +v 侧）
    if not (0 <= c <= 2 and 0 <= r <= 2):
        return None
    uu, vv = (c - 1) * PAD_PITCH, (1 - r) * PAD_PITCH
    if abs(u - uu) > PAD_PITCH / 2.0 or abs(v - vv) > PAD_PITCH / 2.0:
        return None
    return r * 3 + c


def cell_contains(x, y, yaw_deg, size):
    """块（中心 x,y、yaw、边长 size）的四角是否都在**同一个** 60 mm 宫格里。

    这是「不压格」的判据。29.5 mm 方块任意 yaw 的半对角 20.9 mm < 30 mm，
    所以在格心处永远成立；用它是为了拦住「中心偏了 / 块太大」的将来。
    """
    idx = cell_index_of(x, y)
    if idx is None:
        return False, None
    ca, sa = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    ca_p, sa_p = math.cos(PAD_YAW_RAD), math.sin(PAD_YAW_RAD)
    half = size / 2.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            lx = sx * half * ca - sy * half * sa
            ly = sx * half * sa + sy * half * ca
            wx, wy = x + lx, y + ly
            dx, dy = wx - PAD_CENTER[0], wy - PAD_CENTER[1]
            u = ca_p * dx + sa_p * dy
            v = -sa_p * dx + ca_p * dy
            r, c = divmod(idx, 3)
            uu, vv = (c - 1) * PAD_PITCH, (1 - r) * PAD_PITCH
            if abs(u - uu) > PAD_PITCH / 2.0 or abs(v - vv) > PAD_PITCH / 2.0:
                return False, idx
    return True, idx


# ---------------------------------------------------------------- 自由区
def free_destinations(step=FREE_STEP):
    """自由区里的候选落点（按离锚点的距离排序，近的先试）。"""
    xs = np.arange(FREE_X[0], FREE_X[1] + 1e-9, step)
    ys = np.arange(FREE_Y[0], FREE_Y[1] + 1e-9, step)
    pts = [(float(x), float(y)) for x in xs for y in ys]
    pts.sort(key=lambda p: math.hypot(p[0] - FREE_ANCHOR[0],
                                      p[1] - FREE_ANCHOR[1]))
    return pts


def in_free_band(x, y):
    return FREE_X[0] <= x <= FREE_X[1] and FREE_Y[0] <= y <= FREE_Y[1]


# ---------------------------------------------------------------- 块位
def ghost(block, x, y, z_bottom=None):
    """真值块 → 搬到 (x, y) 的副本（保持 id / size / yaw）。

    ``z_bottom`` 默认落台面（0）；放垫上就传 ``PAD_THICK``。
    """
    size = float(block["size"])
    z = (0.0 if z_bottom is None else float(z_bottom)) + size / 2.0
    out = dict(block)
    out.update({"x": float(x), "y": float(y), "z": z})
    return out


def place_in_cell(block, index):
    """把块放到宫格 ``index`` 的格心、垫面上。"""
    cx, cy = cell_center(index)
    return ghost(block, cx, cy, z_bottom=PAD_THICK)


# ---------------------------------------------------------------- 终态
def terminal_report(blocks, pen_tol=PEN_TOL):
    """全桌终态合法性（契约 §4.3 的那条）。

    每块：``zone`` ∈ {slot, free}、``in_bounds``、``legal``；整桌：
    ``overlaps``（穿插对数）、``off_table``（不在台面/垫上或掉下桌）、
    ``out_of_bounds``（四角出桌）。

    「有支撑」三种都算：落台面、落垫面、**压在另一块上**（S4 的塔本来就
    叠着 —— 只看台面会把合法的塔判成悬空）。
    """
    obbs = [sm.block_obb(b, 0.0, name="block-%s" % b.get("id")) for b in blocks]
    tops = [float(b["z"]) + float(b["size"]) / 2.0 for b in blocks]

    rows = []
    for k, b in enumerate(blocks):
        size = float(b["size"])
        z_bottom = float(b["z"]) - size / 2.0
        half = size / 2.0
        ca = math.cos(math.radians(b.get("yaw", 0.0)))
        sa = math.sin(math.radians(b.get("yaw", 0.0)))
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                corners.append((b["x"] + sx * half * ca - sy * half * sa,
                                b["y"] + sx * half * sa + sy * half * ca))
        in_bounds = all(TABLE_XMIN <= x <= TABLE_XMAX and
                        TABLE_YMIN <= y <= TABLE_YMAX for x, y in corners)

        on_table = abs(z_bottom) <= 0.001
        on_pad = (abs(z_bottom - PAD_THICK) <= 0.001
                  and abs(b["x"] - PAD_CENTER[0]) <= PAD_HALF
                  and abs(b["y"] - PAD_CENTER[1]) <= PAD_HALF)
        on_block = False
        if not (on_table or on_pad):
            for j, other in enumerate(blocks):
                if j == k:
                    continue
                if abs(z_bottom - tops[j]) > 0.001:
                    continue
                # 高度对上还不够：**必须是同一块真的在它下面**。两个 OBB 在
                # 3D 里贴着（separation ≈ 0）才说明有接触；只在高度上对齐、XY
                # 离得很远的块不能当支撑 —— 上一版只查「不穿插」，于是塔顶块
                # 在塔底被挪走后仍被判「有支撑」，这是假绿灯。
                if sm.separation(obbs[k], obbs[j]) <= 0.001:
                    on_block = True
                    break
        supported = on_table or on_pad or on_block
        off_table = (not supported) and z_bottom < 0.0
        # 悬空（没落地也没落垫/落块）既不是「掉下桌」也不是合法终态，单列
        floating = (not supported) and z_bottom > 0.0

        zone = "slot" if on_pad else "free"
        reasons = []
        if not in_bounds:
            reasons.append("出桌")
        if floating:
            reasons.append("悬空")
        if off_table:
            reasons.append("不在台面")
        if on_pad:
            inside, idx = cell_contains(b["x"], b["y"], b.get("yaw", 0.0), size)
            if not inside:
                reasons.append("压格")
            else:
                zone = "slot"
        rows.append({"id": b.get("id"), "zone": zone, "in_bounds": in_bounds,
                     "legal": not reasons, "reasons": reasons,
                     "on_block": on_block,
                     "z_bottom_mm": round(z_bottom * 1000.0, 2),
                     "cell": cell_index_of(b["x"], b["y"])})

    overlaps = 0
    pairs = []
    for i in range(len(obbs)):
        for j in range(i + 1, len(obbs)):
            sep = sm.separation(obbs[i], obbs[j])
            if sep < -pen_tol:
                overlaps += 1
                pairs.append([blocks[i].get("id"), blocks[j].get("id"),
                              round(sep * 1000.0, 2)])

    n_bad = len([r for r in rows if not r["legal"]])
    n_off = len([r for r in rows if "不在台面" in r["reasons"]])
    n_out = len([r for r in rows if "出桌" in r["reasons"]])
    ok = (overlaps == 0 and n_bad == 0)
    return {"ok": ok, "overlaps": overlaps, "overlap_pairs": pairs,
            "off_table": n_off, "out_of_bounds": n_out,
            "illegal": n_bad, "blocks": rows}


def dependents(blocks, bid, tol=0.001):
    """压在块 ``bid`` 上的块 id 列表（必须**先**挪走，才能动它）。

    这是第二种「遮挡」：不是夹爪碰不到，而是**一动它上面那块就会掉**。
    判据与 ``terminal_report`` 的支撑判定同一口径：z_bottom 贴着 bid 的
    顶面（±1 mm），且两块 OBB 真的接触（separation ≤ 1 mm）。
    """
    idx = None
    tops = [float(b["z"]) + float(b["size"]) / 2.0 for b in blocks]
    # 按字符串比：``blocker_scores`` 的键是 ``str(id)``（探针递归时 bid 是 str），
    # 直接用 == 会让「被挡住的块自己上面还压着块」这种情况静默漏掉依赖。
    for k, b in enumerate(blocks):
        if str(b.get("id")) == str(bid):
            idx = k
            break
    if idx is None:
        return []
    obbs = {k: sm.block_obb(b, 0.0, name="block-%s" % b.get("id"))
            for k, b in enumerate(blocks)}
    out = []
    for j, b in enumerate(blocks):
        if j == idx:
            continue
        z_bottom = float(b["z"]) - float(b["size"]) / 2.0
        if abs(z_bottom - tops[idx]) > tol:
            continue
        if sm.separation(obbs[j], obbs[idx]) <= tol:
            out.append(b.get("id"))
    return out


# ---------------------------------------------------------------- 遮挡归属
def blocker_scores(rows):
    """E7 候选结果 → 每块「挡了多少个候选位姿」。

    ``rows`` 是 ``stage4_clear_probe`` 每块评测的逐位姿记录，带
    ``per_other``（块 id → 该位姿下夹爪与它的最小间隙，米）与 ``verdict``。
    只统计**夹爪真的碰到它**（间隙 ≤ 0）的位姿：``ok`` 的位姿不算遮挡，但一个
    位姿若同时挨着两块，两块都记 —— 挪走一块可能还不够，靠重测决定。

    返回 ``{id: {"n": 命中的位姿数, "worst_mm": 最负的间隙}}``；按 ``n`` 降序。
    """
    out = {}
    for r in rows:
        per = r.get("per_other") or {}
        hit = [(k, g) for k, g in per.items() if g is not None and g <= 0.0]
        for k, g in hit:
            e = out.setdefault(k, {"n": 0, "worst_mm": 0.0})
            e["n"] += 1
            e["worst_mm"] = min(e["worst_mm"], round(g * 1000.0, 1))
    return dict(sorted(out.items(), key=lambda kv: (-kv[1]["n"],
                                                    kv[1]["worst_mm"])))


def pick_blocker(scores, exclude=()):
    """按「挡得最多」选一个遮挡块；``exclude`` 里的（正在清/已挪走）跳过。"""
    for bid, _ in scores.items():
        if bid not in exclude:
            return bid
    return None
