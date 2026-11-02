#!/usr/bin/env python3
"""阶段 3 塔顶测量：只量**当前最上层**，并把 z 读数偏差按标定常数扣掉。

为什么只量最上层
────────────────
竖直俯视下，第 k 层放好后第 k+1 层就把它压住了 —— 下层顶面不可见。
逐层指标的唯一来源因此是「该层放好瞬间」的一次测量（``metric_source =
"immediate"``）；终态复测只回答最上层有没有动、塔有没有塌，**不能**反推下层。

测量怎么做
──────────
· ``graspnet_candidates.find_blocks(max_h_mm=220)``：阶段 2 的上限 70 mm 只容
  单块；5 层塔顶约 150 mm，整根柱子要进同一个簇；
· 塔窗：第 1 层 = 中心格心 ±``radius_mm``（默认 50 mm），第 k≥2 层 = 上一层
  实测中心 ±``radius_mm``；窗内取**最高**的合格簇（``top`` 是 base 系绝对 z，
  97 分位，与 ``find_blocks`` 同一口径）；
· yaw **两套并行记录，判据只用彩色那套**：深度 ``yaw_rect``（``yaw_deg``）与
  彩色掩码最小外接矩形（``yaw_color_deg``）。深度那套会被侧面污染（同一次放置
  两视点差到 26°），连完整正方形也自带 ~2° 散差 ⇒ 契约 §4.2.5 定为
  「彩色是判据，深度只报告」。取用一律走 `metric_yaw`；彩色判不了 ⇒ 该层转人评，
  不拿深度顶替；
· z 标定：``surface = top − block_h − z_bias``。``z_bias`` 的定义是
  「同一位置单块实测顶高 − 真高」，塔位侧由入口实验 1 标定（扣掉垫厚）。

融合
────
同一视点连拍压随机噪声：中心/顶高取中位，yaw 走 ``mean_yaw_mod90``。
两个视点各出一份融合值，再比一致性（默认 ≤10 mm / ≤6° / ≤10 mm）——
不过就转人评，不进机判。
"""
import argparse
import glob
import json
import math
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import cv2                                                        # noqa: E402
import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                   # noqa: E402
import stage2_geom as sg                                           # noqa: E402
import stage2_round as s2                                          # noqa: E402

#: 同一批方块的**真高**（用户多次实测 29.5 mm，2026-10 定）。
BLOCK_TRUE_H_MM = 29.5
#: 塔窗半径：中心格心 / 上一层实测中心 ± 这个值。
TOWER_RADIUS_MM = 50.0
#: 找块高度上限：5 层 ≈ 150 mm，留到 220 mm 把整根柱子容进来。
MAX_H_MM = 220.0
#: 低于这个高度的东西不算块（沿用阶段 2）。
MIN_H_MM = 15.0

#: 两视点一致性门（沿用阶段 2 的 6° / 10 mm；z 同按 10 mm 算位置）。
POSE_XY_TOL_MM = 10.0
POSE_YAW_TOL_DEG = 6.0
POSE_Z_TOL_MM = 10.0

#: 中心格 —— 塔基（用户拍板）。
TOWER_CELL_IDX = 4

#: 塔顶面点集离**图像边缘**的最小像素裕度闸门。低于它就判「顶面被画幅/
#: 持块切掉」，中心不可用（fail-closed）。
#:
#: 2026-11-01 三层实测（离线重建，见台账）：裕度与质心偏差是**一一对应**的——
#:   place_approach 处（裕度 −1~1 px）顶面只露出 62~67%，质心沿「相机→塔」
#:   方向偏 +5.4~+5.6 mm（实测簇 +6.4~+7.2 mm）；
#:   裕度 7~15 px（抬 20 mm）偏 +1.6~+1.9；裕度 20~27 px（抬 30 mm）偏 +0.2~+0.6；
#:   **裕度 ≥32 px（抬 40 mm 起）偏 0.00**。
#: 取 20 px：拦下偏差 ≥1.6 mm 的帧，放过 ≤0.6 mm 的。正常基准帧（近垂直）
#: 裕度 239~325 px、carry 高度抓帧 44~61 px，都远在线上。
#:
#: **2026-11-01 从 `stage3_entry` 搬到这里**：原来只有入口实验用它，生产测量
#: 链（`measure_frame`）没有 —— 同一套测量位姿一旦被改低，就会静默地量出偏
#: +5~+7 mm 的中心。搬家而不是复制（全仓一个投影口径）。
TOP_FACE_CLEAR_PX = 20.0


def metric_yaw(merged):
    """测量融合值 → ``(yaw_deg, 来源)``；判不了给 ``(None, 原因)``。

    **契约 §4.2.5：yaw 这一维一律走彩色掩码估计器**，深度 ``yaw_rect`` 只作
    报告。判不了就转 ``person_review``，**不拿深度顶替**。

    判分（``stage3_score``）与闭环控制（``stage3_round`` 的 closed 组）都走这一个
    函数——两处各写一份就会漂：控制拿深度 yaw 去摆位，等于把 2–5° 的估计器噪声
    直接拧进塔里。

    两套估计器量的不是同一件事：彩色走像素 ``cv2.minAreaRect``；深度走点云
    ``graspnet_candidates.min_area_rect_2d``。后者对**完整的、无噪声的
    29.5 mm 正方形**本身就带散差：2–98 分位裁剪 + 采样密度给一个固定偏置，
    再加 0.3 mm 的边缘抖动就是 σ≈2.1°、极差 6.5°（台账第一百九十八轮）。
    """
    y = merged.get("yaw_color_deg")
    if y is None:
        return None, "no_color_yaw"
    if not merged.get("yaw_color_trusted"):
        return None, "color_yaw_untrusted"
    return float(y), "color"


def grid_center(grid, cell_idx=TOWER_CELL_IDX):
    """``grid.json`` → 某格的 base 系中心 (mm)。"""
    for c in grid["cells"]:
        if int(c["idx"]) == int(cell_idx):
            return [float(c["center_base_mm"][0]), float(c["center_base_mm"][1])]
    raise ValueError("grid.json 里没有格 %d" % cell_idx)


def pad_z_mm(grid):
    """垫面高度（base 系 z，mm）：``T_base_pad[2,3]``。"""
    T = np.asarray(grid["T_base_pad"], dtype=np.float64)
    return round(float(T[2, 3]) * 1000.0, 3)


#: 颜色掩码：从块中心 9×9 中位色出发做 4 连通生长，判据是
#: **色相 / 饱和度 / 明度各自贴住种子**，不是「RGB 通道最大差」。
#: 为什么换（2026-11-01，台账第一百九十九轮）：块被自身阴影压暗时，通道差判据
#: 会把旁边的**灰色地面**一起收进来 —— 暗黄种子 (105,80,56) 与灰地面 (100,95,90)
#: 的通道最大差只有 34 < 45，掩码沿阴影长出一条尾巴，外接尺寸从 29.5 mm 撑到
#: 33–47 mm，`fill` 还常判合格（入口③ 66 帧里 8 帧）。灰地面**没有色相、饱和度≈0**，
#: 块顶面饱和度≈0.45 ⇒ 饱和度和明度两项分得干净，色相反而分不开（两者都在 ~30°）。
#: 实测：ext 偏差中位 3.11→1.16 mm（调参集）/ 3.93→1.44 mm（另一批 run 的 50 帧），
#: 同状态跨视点 yaw 极差中位 1.07→0.17°。
COLOR_TOL = 45.0
#: 色相容差（度，取环形最短距离）。
COLOR_HUE_TOL_DEG = 15.0
#: 饱和度容差（0–1）。**这是真正把块顶面与灰影分开的那一项。**
COLOR_SAT_TOL = 0.18
#: 明度容差（0–1）。放得宽：阴影下的同一面本来就该收进来。
COLOR_VAL_TOL = 0.45
#: 种子饱和度低于此值就认为块本身无彩（白/灰块），判据退回 RGB 通道差 ``COLOR_TOL``。
COLOR_ACHROMATIC_SAT = 0.12
#: 颜色掩码的搜索半窗（像素）。块在 1280×720 里约 76–82 px 宽，80 px 够。
COLOR_HALF_PX = 80
#: 颜色掩码可信门槛：轮廓填充率 + 像素数。
COLOR_FILL_MIN = 0.85
COLOR_MIN_PIXELS = 300
#: 贴块异物的**救援**：原掩码判假时，用一个「边长 = 块顶面真边长」的正方形去切。
#: 起因（2026-11-01 正式档）：塔位左侧贴着一片**同色异物**，颜色漫水把它一起收进来，
#: 外接尺寸从 29 mm 撑到 36.9 mm、fill 掉到 0.75，10 次里判假作废 2 次
#: （nudge-07/09，`no_colour_yaw`）。那片异物的**整片区域没有有效深度**（深度图
#: 空洞），所以按深度同高带分不开；能分开它的只有「顶面必须是这么大」这条先验 ——
#: 方块是立方体，顶面边长就是块高。**只在判假时启用**，判真的帧一个字节都不动。
COLOR_SIDE_PAD_MM = 2.0
COLOR_SIDE_SEARCH_MM = 6.0
COLOR_SIDE_STEP_MM = 1.0
#: 被切掉的像素比例上限：切太多说明这不是「块 + 一点粘连」，救援放弃。
COLOR_RESCUE_MAX_EXCL = 0.30


def _hsv1(rgb):
    """单个 RGB 三元组 → ``(h_deg, s, v)``；``s``/``v`` 归一到 0–1。"""
    c = np.asarray(rgb, dtype=np.float64) / 255.0
    mx, mn = float(c.max()), float(c.min())
    d = mx - mn
    s = d / mx if mx > 1e-6 else 0.0
    if d < 1e-6:
        return 0.0, s, mx
    r, g, b = (float(v) for v in c)
    if mx == r:
        hh = ((g - b) / d) % 6.0
    elif mx == g:
        hh = (b - r) / d + 2.0
    else:
        hh = (r - g) / d + 4.0
    return hh * 60.0, s, mx


def _same_face_colour(px, seed, tol=COLOR_TOL):
    """像素是否属于与种子同一块「同色面」。

    有彩种子走色相/饱和度/明度三项；无彩种子（白/灰块）退回 RGB 通道差 ——
    灰块的颜色信息全在明度里，HSV 那三项对它没有区分力。
    """
    hs, ss, vs = _hsv1(seed)
    if ss < COLOR_ACHROMATIC_SAT:
        return float(np.abs(np.asarray(px, dtype=np.float64)
                            - np.asarray(seed, dtype=np.float64)).max()) < tol
    hp, sp, vp = _hsv1(px)
    d = abs(hp - hs)
    d = min(d, 360.0 - d)
    return (d < COLOR_HUE_TOL_DEG and abs(sp - ss) < COLOR_SAT_TOL
            and abs(vp - vs) < COLOR_VAL_TOL)


def color_face_mask(frame, uv, half_px=COLOR_HALF_PX, tol=COLOR_TOL):
    """块中心邻域内、与中心色相近的 4 连通区域（**只用彩色图**）。

    为什么不直接用 ``stage2_geom.top_face_mask``：那是**深度**掩码，会把斜视时
    探出顶面轮廓的可见侧面一起收进来；近正方形上做最小外接矩形是病态的
    （2026-11-01 实测同一次放置两个视点差到 26.2°，点云外形 32×25 mm）。
    颜色掩码只认顶面同色区域，输入与深度无关，才能当独立估计器。
    """
    img = frame["color"]
    h, w = img.shape[:2]
    u, v = int(round(float(uv[0]))), int(round(float(uv[1])))
    mask = np.zeros((h, w), dtype=np.uint8)
    if not (0 <= u < w and 0 <= v < h):
        return mask
    u0, u1 = max(0, u - half_px), min(w, u + half_px)
    v0, v1 = max(0, v - half_px), min(h, v + half_px)
    sub = img[v0:v1, u0:u1].astype(np.int16)
    sh, sw = sub.shape[:2]
    cu, cv = u - u0, v - v0
    if not (0 <= cu < sw and 0 <= cv < sh):
        return mask
    patch = sub[max(0, cv - 4):cv + 5, max(0, cu - 4):cu + 5]
    seed = np.median(patch.reshape(-1, 3), axis=0)
    local = np.zeros((sh, sw), dtype=bool)
    local[cv, cu] = True
    q = deque([(cv, cu)])
    while q:
        y, x = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < sh and 0 <= nx < sw and not local[ny, nx]:
                if _same_face_colour(sub[ny, nx], seed, tol):
                    local[ny, nx] = True
                    q.append((ny, nx))
    mask[v0:v1, u0:u1] = local.astype(np.uint8)
    return mask


def _plane_xy_of_mask(frame, mask, z_top_m):
    """掩码内每个像素 → ``z=z_top`` 平面上的 base 系 xy（米），形状 ``(N,2)``。"""
    K, R, t = frame.get("K"), frame.get("R_bc"), frame.get("t_bc")
    if K is None or R is None or t is None:
        return None
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    vs, us = np.nonzero(mask)
    if us.size == 0:
        return None
    Kinv = np.linalg.inv(np.asarray(K, dtype=np.float64))
    rays = Kinv @ np.stack([us, vs, np.ones(us.size, dtype=np.float64)])
    dirs = (R @ rays).T                                  # 相机方向 → base 系
    dz = dirs[:, 2]
    good = np.abs(dz) > 1e-9
    if not np.any(good):
        return None
    s = np.zeros(us.size, dtype=np.float64)
    s[good] = (float(z_top_m) - t[2]) / dz[good]
    return (dirs * s[:, None] + t)[:, :2]


def _project_base_xy(frame, xy_m, z_top_m):
    """``z=z_top`` 平面上的 base xy（米）→ 像素坐标 ``(N,2)``。"""
    K, R, t = frame.get("K"), frame.get("R_bc"), frame.get("t_bc")
    if K is None or R is None or t is None:
        return None
    K = np.asarray(K, dtype=np.float64)
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    xy = np.asarray(xy_m, dtype=np.float64).reshape(-1, 2)
    P = np.column_stack([xy, np.full(len(xy), float(z_top_m))])
    Pc = (P - t) @ R                                     # base → 相机
    if np.any(np.abs(Pc[:, 2]) < 1e-9):
        return None
    uv = (K @ Pc.T).T
    return uv[:, :2] / uv[:, 2:3]


def _square_corners(centre_xy_m, yaw_deg, side_mm):
    """边长 ``side_mm``、朝向 ``yaw_deg`` 的正方形四个角（base xy，米）。"""
    a = math.radians(float(yaw_deg))
    e1 = np.array([math.cos(a), math.sin(a)])
    e2 = np.array([-math.sin(a), math.cos(a)])
    h = float(side_mm) / 2000.0
    c = np.asarray(centre_xy_m, dtype=np.float64).reshape(2)
    return np.array([c + h * (e1 + e2), c + h * (e1 - e2),
                     c - h * (e1 + e2), c - h * (e1 - e2)])


def _poly_mask(shape, corners_px):
    m = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillPoly(m, [np.round(np.asarray(corners_px)).astype(np.int32)], 1)
    return m


def _rescue_by_side(frame, mask, block, yaw_deg, side_mm, z_top_m):
    """用「顶面边长已知」把贴着的同色异物切出掩码，返回重拟合结果或 None。

    在**深度给的中心**（``block['cx'/'cy']``，与颜色无关）附近网格搜一个
    边长 ``side_mm + 2*COLOR_SIDE_PAD_MM`` 的正方形，取覆盖掩码像素最多的一处；
    框外像素视为异物切掉，在剩下的核上重做 ``mask_rect_yaw``。

    新 ``fill`` **不是**重拟合自带的比值（那会是自证的），而是「原掩码覆盖
    『重拟合出的真边长正方形』的比例」—— 块被遮掉一半、或颜色只长了一半，
    这个值都会掉下来。
    """
    if block.get("cx") is None or block.get("cy") is None:
        return None                     # 没有深度质心就没法搜框，交给原判据
    pts = _plane_xy_of_mask(frame, mask, z_top_m)
    n_all = int(cv2.countNonZero(mask))
    if pts is None or n_all < COLOR_MIN_PIXELS:
        return None
    half = (float(side_mm) + 2.0 * COLOR_SIDE_PAD_MM) / 2000.0
    a = math.radians(float(yaw_deg))
    e1 = np.array([math.cos(a), math.sin(a)])
    e2 = np.array([-math.sin(a), math.cos(a)])
    c0 = np.array([float(block["cx"]), float(block["cy"])]) / 1000.0
    rel = pts - c0
    along, across = rel @ e1, rel @ e2
    n = int(round(COLOR_SIDE_SEARCH_MM / COLOR_SIDE_STEP_MM))
    best_k, best_d = -1, [0.0, 0.0]
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            du = i * COLOR_SIDE_STEP_MM / 1000.0
            dv = j * COLOR_SIDE_STEP_MM / 1000.0
            k = int(np.count_nonzero((np.abs(along - du) <= half)
                                     & (np.abs(across - dv) <= half)))
            d2 = du * du + dv * dv
            if k > best_k or (k == best_k and d2 < best_d[0] ** 2
                              + best_d[1] ** 2):
                best_k, best_d = k, [du, dv]
    centre = c0 + best_d[0] * e1 + best_d[1] * e2
    cut_px = _project_base_xy(
        frame, _square_corners(centre, yaw_deg,
                               float(side_mm) + 2.0 * COLOR_SIDE_PAD_MM),
        z_top_m)
    if cut_px is None:
        return None
    core = mask & _poly_mask(mask.shape, cut_px)
    n_core = int(cv2.countNonZero(core))
    if n_core < COLOR_MIN_PIXELS:
        return None
    excl = 1.0 - n_core / float(n_all)
    if excl > COLOR_RESCUE_MAX_EXCL:
        return None
    r = sg.mask_rect_yaw(core, frame["K"], frame["R_bc"], frame["t_bc"],
                         z_top_m)
    if r is None:
        return None
    # 覆盖：用**重拟合出的**中心/朝向、按真边长算方框，量原掩码盖住了多少。
    box_px = _project_base_xy(
        frame, _square_corners(centre, r["yaw_deg"], float(side_mm)), z_top_m)
    if box_px is None:
        return None
    box = _poly_mask(mask.shape, box_px)
    n_box = int(cv2.countNonZero(box))
    if n_box == 0:
        return None
    r["fill"] = float(cv2.countNonZero(mask & box)) / float(n_box)
    r["n_pixels"] = n_core
    r["rescue"] = {"excl": round(float(excl), 4),
                   "shift_mm": [round(best_d[0] * 1000.0, 2),
                                round(best_d[1] * 1000.0, 2)],
                   "n_all_px": n_all,
                   "cut_ext_mm": r["ext_mm"]}
    return r


def color_rect_yaw(frame, block, half_px=COLOR_HALF_PX, tol=COLOR_TOL,
                   side_mm=None):
    """颜色掩码 → 最小外接矩形 → **base 系** yaw（复用 ``sg.mask_rect_yaw``）。

    与深度 ``yaw_rect`` 并行记录、事后对比；两者不一致时以本值为主 ——
    它的输入与深度无关，不会被侧面污染。

    给了 ``side_mm``（块顶面真边长）且原掩码判假时，走一次 ``_rescue_by_side``：
    贴块同色异物会被切掉重算。**判真的帧原样返回**，不受影响。
    """
    mask = color_face_mask(frame, block["uv"], half_px=half_px, tol=tol)
    z_top = float(block["top"]) / 1000.0
    r = sg.mask_rect_yaw(mask, frame["K"], frame["R_bc"], frame["t_bc"],
                         z_top)
    if r is None:
        return {"yaw_deg": None, "ext_mm": [0.0, 0.0], "fill": 0.0,
                "n_pixels": 0, "trusted": False, "reason": "掩码/轮廓太小"}
    r["trusted"] = bool(r["fill"] >= COLOR_FILL_MIN
                        and r["n_pixels"] >= COLOR_MIN_PIXELS)
    r["reason"] = ""
    if r["trusted"] or side_mm is None:
        return r
    fix = _rescue_by_side(frame, mask, block, r["yaw_deg"],
                          float(side_mm), z_top)
    if fix is None:
        r["reason"] = "贴块异物切不掉"
        return r
    fix["trusted"] = bool(fix["fill"] >= COLOR_FILL_MIN
                          and fix["n_pixels"] >= COLOR_MIN_PIXELS)
    fix["reason"] = "" if fix["trusted"] else "救援后仍不可信"
    return fix


#: 顶面连通块的点数下限：占选中点的这个比例以下 = 零星点，不算顶面的一部分。
#: 实测（2026-11-02，5 层遮挡帧）：真顶面 8038~8453 点，混进来的画幅底边伪影
#: 只有 12~93 点（0.15~1.1 %），却能把 bbox 往下拉 40 px，把「看得见」误判成
#: 「贴边不可用」。取 5 % 离两头都很远。
TOP_FACE_MIN_FRAC = 0.05
TOP_FACE_MIN_PTS = 50


def _largest_face_only(vv, uu):
    """丢掉像素上不与顶面连通的零星点，只留下够大的连通块。

    顶面在图像上是一整块连续面；同一高度上别处的散点（深度伪影、邻块边角）
    在像素上不连着，却会把 min/max 拉到画幅边上。按 4 连通拆开后，
    点数不到 5 % 的块丢掉。
    """
    y0, y1 = int(vv.min()), int(vv.max())
    x0, x1 = int(uu.min()), int(uu.max())
    occ = np.zeros((y1 - y0 + 1, x1 - x0 + 1), bool)
    occ[vv - y0, uu - x0] = True
    lab, n_lab = gc.label4(occ)
    if n_lab <= 1:
        return vv, uu
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0                                  # 0 是背景
    need = max(TOP_FACE_MIN_PTS, TOP_FACE_MIN_FRAC * len(vv))
    keep = np.where(sizes >= need)[0]
    if len(keep) == 0:                            # 全都很小 ⇒ 留最大的那块
        keep = [int(sizes.argmax())]
    m = np.isin(lab[vv - y0, uu - x0], keep)
    return vv[m], uu[m]


def top_face_framing(frame, cx_mm, cy_mm, ztop_mm, radius_mm=25.0,
                     band_mm=3.0, clear_px=TOP_FACE_CLEAR_PX):
    """量一簇的顶面离图像边缘还剩多少像素 —— 判断顶面有没有被切掉。

    只回一个**结构性**判据，不含任何遮挡模型：把 ``ztop_mm`` 往下
    ``band_mm`` 以内的点取出来，看它们最近离图像四边几像素。顶面被画幅
    切掉、或又被手里那块压掉一角，都会让点集贴到边上（见 ``TOP_FACE_CLEAR_PX``
    的实测表）。``radius_mm`` 取 25：块半对角线 20.9 mm，即使中心已偏 7 mm，
    真实顶面仍落在这个圈里；而邻格中心 ≥60 mm，不会误抓邻块。
    """
    depth = frame["depth"]
    R, t, K = frame["R_bc"], frame["t_bc"], frame["K"]
    P, pix = gc.backproject(depth, K)
    Q = (P @ R.T + t) * 1000.0
    d = np.hypot(Q[:, 0] - float(cx_mm), Q[:, 1] - float(cy_mm))
    sel = (d < float(radius_mm)) & (np.abs(Q[:, 2] - float(ztop_mm))
                                    <= float(band_mm))
    n = int(sel.sum())
    if n < 50:
        return {"n": n, "clear_px": 0.0, "span_px": [0.0, 0.0],
                "clipped": True, "reason": "顶面点太少（%d < 50）" % n}
    vv, uu = _largest_face_only(pix[sel, 1], pix[sel, 0])
    h, w = depth.shape
    clear = min(float(uu.min()), float((w - 1) - uu.max()),
                float(vv.min()), float((h - 1) - vv.max()))
    return {"n": n, "n_kept": int(len(vv)), "clear_px": round(clear, 1),
            "span_px": [round(float(uu.max() - uu.min()), 1),
                        round(float(vv.max() - vv.min()), 1)],
            "clipped": bool(clear < float(clear_px)),
            "reason": "" if clear >= float(clear_px) else
            "顶面贴到图像边缘（裕度 %.0f px < %.0f px）——中心会沿"
            "「相机→塔」方向偏，不可用" % (clear, clear_px)}


def pick_tower_top(blocks, window_xy_mm, radius_mm=TOWER_RADIUS_MM,
                   min_h_mm=MIN_H_MM):
    """塔窗内取**最高**的合格簇；返回 ``(block|None, diag)``。

    只认 ``low=False`` 且高度 ≥``min_h_mm`` 的簇 —— 低处的连接件/台面噪声
    不参与。窗内有多簇时取 ``top`` 最大者：塔顶就是最高的那一层。
    """
    wx, wy = float(window_xy_mm[0]), float(window_xy_mm[1])
    cands = []
    for b in blocks:
        if b.get("low") or b.get("h", 0.0) < min_h_mm:
            continue
        d = math.hypot(b["cx"] - wx, b["cy"] - wy)
        if d <= radius_mm:
            cands.append(dict(b, _dist_mm=round(d, 2)))
    diag = {"n_window": len(cands),
            "window_xy_mm": [round(wx, 2), round(wy, 2)],
            "radius_mm": float(radius_mm),
            "candidates": [{"idx": b["idx"], "cx": round(b["cx"], 2),
                            "cy": round(b["cy"], 2), "top": round(b["top"], 2),
                            "dist_mm": b["_dist_mm"]} for b in cands]}
    if not cands:
        return None, diag
    best = max(cands, key=lambda b: b["top"])
    return best, diag


def measure_frame(frame_dir, grid=None, window_xy_mm=None,
                  z_bias_mm=0.0, block_h_mm=BLOCK_TRUE_H_MM,
                  radius_mm=TOWER_RADIUS_MM, max_h_mm=MAX_H_MM,
                  min_h_mm=MIN_H_MM, frame=None):
    """一帧 → 塔顶观测。

    ``grid`` / ``window_xy_mm``：至少给一个；都没给就报错（塔窗不能靠猜）。
    返回的 ``surface_est_mm`` 已扣 ``z_bias_mm``，是**本层放置面**的估计。
    """
    if window_xy_mm is None:
        if grid is None:
            raise ValueError("measure_frame 需要 grid 或 window_xy_mm")
        window_xy_mm = grid_center(grid)
    if frame is None:
        frame = gc.load_frame(frame_dir)
    blocks, table_z = gc.find_blocks(frame, max_h_mm=max_h_mm,
                                     min_h_mm=min_h_mm)
    top, diag = pick_tower_top(blocks, window_xy_mm, radius_mm=radius_mm,
                               min_h_mm=min_h_mm)
    out = {"frame": os.path.basename(os.path.abspath(frame_dir)),
           "found": top is not None,
           "window_xy_mm": [round(float(window_xy_mm[0]), 2),
                            round(float(window_xy_mm[1]), 2)],
           "table_z_mm": round(float(table_z) * 1000.0, 3),
           "z_bias_mm": round(float(z_bias_mm), 3),
           "block_h_mm": round(float(block_h_mm), 3),
           "diag": diag}
    if top is None:
        return out
    yaw = top.get("yaw_rect")
    cyaw = color_rect_yaw(frame, top, side_mm=block_h_mm)
    # 顶面贴到画幅边 ⇒ 中心的偏差可达 +5~+7 mm（`TOP_FACE_CLEAR_PX` 实测表），
    # 这一帧的中心/yaw 一律不进融合。**fail-closed**：宁可少一帧、转人评，
    # 也不让带偏的读数混进中位数。
    framing = top_face_framing(frame, top["cx"], top["cy"], top["top"],
                               clear_px=TOP_FACE_CLEAR_PX)
    out.update({
        "framing": framing,
        "usable": not framing["clipped"],
        "center_base_mm": [round(float(top["cx"]), 2),
                           round(float(top["cy"]), 2)],
        # 绝对顶高（base 系）—— 层间增量与稳定性都用它，标定偏差在差分里抵消。
        "top_mm": round(float(top["top"]), 2),
        # 相对**帧内检出台面**的高度，单独留档（垫子会把它抬高）。
        "h_mm": round(float(top["h"]), 2),
        "yaw_deg": None if yaw is None else round(float(yaw), 3),
        "yaw_trusted": bool(top.get("yaw_trusted")),
        # 颜色掩码的并行读数（与深度无关的独立估计器，2026-11-01 起记录）。
        "yaw_color_deg": (None if cyaw["yaw_deg"] is None
                          else round(float(cyaw["yaw_deg"]), 3)),
        "yaw_color_trusted": bool(cyaw["trusted"]),
        "yaw_color_ext_mm": cyaw["ext_mm"],
        "yaw_color_fill": round(float(cyaw["fill"]), 3),
        "yaw_color_n_px": int(cyaw["n_pixels"]),
        # 判假后走没走「贴块异物救援」、切掉了多少 —— 只为事后审计，不参与判分。
        "yaw_color_reason": cyaw.get("reason", ""),
        "yaw_color_rescue": cyaw.get("rescue"),
        "ext_rect_mm": top.get("ext_rect"),
        "rect_fill": top.get("rect_fill"),
        "class": top.get("class"),
        "class_share": top.get("class_share"),
        "surface_est_mm": round(float(top["top"]) - float(block_h_mm)
                                - float(z_bias_mm), 2),
    })
    return out


def fuse_frames(records):
    """同视点连拍（或两视点已融合值）→ 一份融合值 + 离散度。

    中心 / 顶高取中位；yaw 走 mod-90 圆均值（``yaw_deg=None`` 的帧不投票）。
    """
    found = [r for r in records if r.get("found")]
    n = len(records)
    if not found:
        return {"found": False, "n_frames": n, "n_found": 0}
    cx = float(np.median([r["center_base_mm"][0] for r in found]))
    cy = float(np.median([r["center_base_mm"][1] for r in found]))
    top = float(np.median([r["top_mm"] for r in found]))
    yaws = [r["yaw_deg"] for r in found if r.get("yaw_deg") is not None]
    yaw = s2.mean_yaw_mod90(yaws) if yaws else None
    yaw_span = s2.yaw_span_mod90(yaws) if len(yaws) > 1 else None
    # 彩色 yaw 的圆均值**只收可信帧**：可信与否是另一路判据（``all()``），
    # 不能一边让不可信帧的值污染均值、一边又用标志把它挡在门外 ——
    # 那样 `pose_consistency` 会拿污染值算出一个「一致」的假报告。
    cyaws = [r["yaw_color_deg"] for r in found
             if r.get("yaw_color_deg") is not None
             and r.get("yaw_color_trusted")]
    cyaw = s2.mean_yaw_mod90(cyaws) if cyaws else None
    cyaw_span = s2.yaw_span_mod90(cyaws) if len(cyaws) > 1 else None
    tops = [r["top_mm"] for r in found]
    dists = [math.hypot(r["center_base_mm"][0] - cx,
                        r["center_base_mm"][1] - cy) for r in found]
    return {"found": True, "n_frames": n, "n_found": len(found),
            "center_base_mm": [round(cx, 2), round(cy, 2)],
            "top_mm": round(top, 2),
            "top_span_mm": round(max(tops) - min(tops), 2),
            "center_span_mm": round(2.0 * max(dists), 2),
            "yaw_deg": yaw,
            "yaw_span_deg": yaw_span,
            # 任一帧不可信就不进机判：宁可人评，别把烂 yaw 当数据。
            "yaw_trusted": bool(found) and all(r.get("yaw_trusted")
                                               for r in found),
            # 颜色掩码的并行读数（独立于深度）。
            "yaw_color_deg": cyaw,
            "yaw_color_span_deg": cyaw_span,
            "yaw_color_trusted": bool(found) and all(
                r.get("yaw_color_trusted") for r in found),
            # 有几个可信帧真的投了票 / 一共有几帧出了彩色读数（审计用）。
            "yaw_color_votes": [len(cyaws), sum(
                1 for r in found if r.get("yaw_color_deg") is not None)],
            "frames_used": [r.get("frame") for r in found]}


def pose_consistency(fused_poses, xy_tol_mm=POSE_XY_TOL_MM,
                     yaw_tol_deg=POSE_YAW_TOL_DEG, z_tol_mm=POSE_Z_TOL_MM):
    """两（或多）视点融合值 → 两两最大差 + 是否过门。

    只有一个视点能测到时 ``ok=None``（**证明不了**一致性，转人评）。

    **yaw 判据走彩色掩码**（2026-11-01 用户拍板）。同一批 15 次放置上，深度
    ``yaw_rect`` 与彩色掩码逐次相关 r=0.897、颜色 ≈ 0.70 × 深度 ⇒ 深度法把偏差
    放大约 1.43 倍，且组内散差 6.5–15.5°（彩色 0.3–2.1°）。拿深度当 ``ok`` 的
    判据会把合格放置判红。深度值继续报告（``dyaw_deg``），只是不进 ``ok``。

    彩色 yaw 缺失时 ``ok=None``（reason ``no_color_yaw``）——证明不了就转人评，
    既不拿深度顶替，也不把缺数据当合格。
    """
    found = [p for p in fused_poses if p.get("found")]
    if not found:
        return {"ok": False, "reason": "no_pose_found",
                "n_poses": len(fused_poses)}
    if len(found) < 2:
        return {"ok": None, "reason": "single_pose", "n_poses": len(found)}
    dxy = dyaw = dz = dyaw_color = 0.0
    n_color = 0            # 有多少对真的比过彩色 yaw（缺数据不能当 0 差）
    for i in range(len(found)):
        for j in range(i + 1, len(found)):
            a, b = found[i], found[j]
            dxy = max(dxy, math.hypot(a["center_base_mm"][0]
                                      - b["center_base_mm"][0],
                                      a["center_base_mm"][1]
                                      - b["center_base_mm"][1]))
            dz = max(dz, abs(a["top_mm"] - b["top_mm"]))
            if a.get("yaw_deg") is not None and b.get("yaw_deg") is not None:
                dyaw = max(dyaw, abs(s2.signed_err_mod90(a["yaw_deg"],
                                                         b["yaw_deg"])))
            if (a.get("yaw_color_deg") is not None
                    and b.get("yaw_color_deg") is not None):
                n_color += 1
                dyaw_color = max(dyaw_color, abs(s2.signed_err_mod90(
                    a["yaw_color_deg"], b["yaw_color_deg"])))
    base = {"n_poses": len(found), "dxy_mm": round(dxy, 2),
            "dyaw_deg": round(dyaw, 2), "dz_mm": round(dz, 2),
            "tol": {"xy_mm": xy_tol_mm, "yaw_deg": yaw_tol_deg,
                    "z_mm": z_tol_mm}}
    if n_color == 0:
        return dict(base, ok=None, dyaw_color_deg=None, ok_color=None,
                    reason="no_color_yaw")
    ok = dxy <= xy_tol_mm and dz <= z_tol_mm and dyaw_color <= yaw_tol_deg
    return dict(base, ok=bool(ok), dyaw_color_deg=round(dyaw_color, 2),
                ok_color=bool(ok), reason="ok" if ok else "pose_mismatch")


def measure_layer(pose_frames, grid=None, window_xy_mm=None, layer=None,
                  z_bias_mm=0.0, block_h_mm=BLOCK_TRUE_H_MM,
                  radius_mm=TOWER_RADIUS_MM, max_h_mm=MAX_H_MM,
                  min_h_mm=MIN_H_MM, metric_source="immediate"):
    """一层的完整测量：``[(视点名, [帧目录, …]), …]`` → 观测记录。

    逐帧测 → 每视点融合 → 两视点一致性 → 再融合成该层唯一取值。
    ``merged`` 里的 ``center_base_mm`` / ``top_mm`` / ``yaw_deg`` 就是该层指标
    （``metric_source="immediate"``）。

    两个容易混的量，这里分开写：
    · ``surface_est_mm`` = 校正顶高 − 块真高 = **本层块脚下的面**（对账用）；
    · ``next_surface_est_mm`` = 校正顶高 = **下一层要放的表面**。
    拿前者当下一层表面会整整差一块高（29.5 mm）。
    """
    poses = []
    dropped = []
    for name, dirs in pose_frames:
        frames = [measure_frame(d, grid=grid, window_xy_mm=window_xy_mm,
                                z_bias_mm=z_bias_mm, block_h_mm=block_h_mm,
                                radius_mm=radius_mm, max_h_mm=max_h_mm,
                                min_h_mm=min_h_mm) for d in dirs]
        # ``usable=False`` = 顶面贴到画幅边（`TOP_FACE_CLEAR_PX`）：中心会偏
        # +5~+7 mm，**不进融合**。全被剔掉就按「这一视点没测到」处理。
        kept = [f for f in frames if f.get("usable", True)]
        dropped += [{"pose": name, "frame": f.get("frame"),
                     "clear_px": (f.get("framing") or {}).get("clear_px")}
                    for f in frames if not f.get("usable", True)]
        poses.append({"name": name, "frames": frames,
                      "n_dropped": len(frames) - len(kept),
                      "fused": fuse_frames(kept)})
    fused_all = [p["fused"] for p in poses]
    cons = pose_consistency(fused_all)
    # 融合时带上视点名，``merged["frames_used"]`` 才不会是一串 None。
    merged = fuse_frames([dict(p["fused"], frame=p["name"]) for p in poses])
    out = {"version": 1, "layer": layer, "metric_source": metric_source,
           "z_bias_mm": round(float(z_bias_mm), 3),
           "block_h_mm": round(float(block_h_mm), 3),
           "window_xy_mm": ([round(float(window_xy_mm[0]), 2),
                             round(float(window_xy_mm[1]), 2)]
                            if window_xy_mm is not None else
                            (grid_center(grid) if grid else None)),
           "poses": poses, "consistency": cons, "merged": merged,
           "frames_dropped": dropped}
    if merged.get("found"):
        out["surface_est_mm"] = round(merged["top_mm"] - float(block_h_mm)
                                      - float(z_bias_mm), 2)
        out["next_surface_est_mm"] = round(merged["top_mm"]
                                           - float(z_bias_mm), 2)
    return out


def bias_from_single(top_mm, pad_z, block_h_mm=BLOCK_TRUE_H_MM):
    """入口实验 1：塔位单块 → ``z_bias = top − 垫面 − 真高``（mm）。"""
    return round(float(top_mm) - float(pad_z) - float(block_h_mm), 3)


def scan_biases(frame_dirs, block_h_mm=BLOCK_TRUE_H_MM,
                min_h_mm=MIN_H_MM, max_h_mm=70.0):
    """一批帧 → 逐块 ``bias = h − 真高`` 表 + 汇总（入口实验 1 / 历史复算）。

    ``h`` 是相对**帧内检出台面**的高度（`find_blocks` 的 ``h``）。用 ``h`` 而不是
    绝对 ``top``，是因为历史帧的台面读数本身带偏移；``bias`` 的定义就是
    「同一块在深度图里比 29.5 mm 高/低多少」，与台面锚点无关。
    """
    rows = []
    for fd in frame_dirs:
        blocks, table_z = gc.find_blocks(gc.load_frame(fd), max_h_mm=max_h_mm,
                                         min_h_mm=min_h_mm)
        for b in blocks:
            if b.get("low"):
                continue
            rows.append({
                "frame": os.path.basename(os.path.abspath(fd)),
                "dir": os.path.abspath(fd),
                "block_idx": int(b["idx"]),
                "cx_mm": round(float(b["cx"]), 2),
                "cy_mm": round(float(b["cy"]), 2),
                "h_mm": round(float(b["h"]), 2),
                "bias_mm": round(float(b["h"]) - float(block_h_mm), 2),
                "class": b.get("class"),
            })
    vals = [r["bias_mm"] for r in rows]
    summary = {"n": len(vals), "n_blocks": len(rows), "n_frames": len(frame_dirs)}
    if vals:
        arr = np.asarray(vals, dtype=float)
        summary.update({
            "median_mm": round(float(np.median(arr)), 2),
            "mean_mm": round(float(arr.mean()), 2),
            "min_mm": round(float(arr.min()), 2),
            "max_mm": round(float(arr.max()), 2),
            "n_positive": int((arr > 0).sum()),
        })
    return {"version": 1, "block_h_mm": float(block_h_mm), "summary": summary,
            "rows": rows}


# ---------------------------------------------------------------- CLI
def _parse_poses(items):
    out = []
    for spec in items:
        if "=" not in spec:
            raise SystemExit("--pose 要 name=dir1,dir2 形式，收到 %r" % spec)
        name, dirs = spec.split("=", 1)
        out.append((name, [d for d in dirs.split(",") if d]))
    return out


def main():
    ap = argparse.ArgumentParser(description="阶段 3 塔顶测量（只量最上层）")
    ap.add_argument("--pose", action="append", default=None,
                    help="视点：name=frame_dir[,frame_dir…]；可重复")
    ap.add_argument("--scan", action="append", default=None,
                    help="只出偏差表：帧目录（可含通配符）；可重复")
    ap.add_argument("--grid", default=None, help="grid.json（给塔窗与垫面）")
    ap.add_argument("--window", default=None,
                    help="塔窗中心 x,y（mm）；缺省取 grid 中心格")
    ap.add_argument("--layer", type=int, default=None)
    ap.add_argument("--bias-mm", type=float, default=0.0,
                    help="塔位 z 读数偏差（实测顶高 − 真高），从入口实验来")
    ap.add_argument("--block-h-mm", type=float, default=BLOCK_TRUE_H_MM)
    ap.add_argument("--radius-mm", type=float, default=TOWER_RADIUS_MM)
    ap.add_argument("--max-h-mm", type=float, default=MAX_H_MM)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    if args.scan:
        dirs = []
        for spec in args.scan:
            hits = sorted(glob.glob(os.path.expanduser(spec)))
            dirs.extend(hits or [os.path.expanduser(spec)])
        res = scan_biases(dirs, block_h_mm=args.block_h_mm,
                          min_h_mm=MIN_H_MM, max_h_mm=args.max_h_mm)
        text = json.dumps(res, ensure_ascii=False, indent=1)
        if args.out:
            open(args.out, "w").write(text + "\n")
        print(text)
        s = res["summary"]
        print("偏差（h − %.1f mm）：n=%d 中位 %.2f 均值 %.2f 极差 %.2f..%.2f"
              % (args.block_h_mm, s["n"], s.get("median_mm", 0.0),
                 s.get("mean_mm", 0.0), s.get("min_mm", 0.0),
                 s.get("max_mm", 0.0)), file=sys.stderr)
        return 0
    if not args.pose:
        raise SystemExit("要么给 --pose，要么给 --scan")

    grid = json.load(open(args.grid)) if args.grid else None
    window = None
    if args.window:
        window = [float(v) for v in args.window.split(",")]
        if len(window) != 2:
            raise SystemExit("--window 要给 x,y 两个数")
    elif grid is not None:
        window = grid_center(grid)
    else:
        raise SystemExit("要给 --grid 或 --window（塔窗不能靠猜）")

    res = measure_layer(_parse_poses(args.pose), grid=grid,
                        window_xy_mm=window, layer=args.layer,
                        z_bias_mm=args.bias_mm,
                        block_h_mm=args.block_h_mm,
                        radius_mm=args.radius_mm, max_h_mm=args.max_h_mm)
    if grid is not None:
        res["pad_z_mm"] = pad_z_mm(grid)
        if res["merged"].get("found"):
            res["bias_est_mm"] = bias_from_single(
                res["merged"]["top_mm"], res["pad_z_mm"], args.block_h_mm)

    text = json.dumps(res, ensure_ascii=False, indent=1)
    if args.out:
        open(args.out, "w").write(text + "\n")
    print(text)
    m = res["merged"]
    if not m.get("found"):
        print("!! 塔窗内没有合格块 —— 本层 measured_missing", file=sys.stderr)
        return 2
    if res["consistency"].get("ok") is not True:
        print("⚠ 一致性未过（%s）—— 转人评" % res["consistency"].get("reason"),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
