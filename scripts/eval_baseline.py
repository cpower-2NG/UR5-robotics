#!/usr/bin/env python3
"""精度基线的离线评估：**腕部相机自己当尺子**。

为什么不用俯视相机（2026-10-25 起）
====================================
俯视相机没有固定，会频繁漂移，已经基本移出系统。所以这条链路里**不引入
任何外部真值设备**：

    每帧画面里同时有 ChArUco 板（方格 30.93 mm，见下）和积木。
      · 板 → 相机：16 个棋盘内角点 + 12 个 ArUco 码的 ID 解掉方形板的
        8 重对称歧义，再 solvePnP；
      · 积木真值：**按深度分层**找每块积木的顶面质心 —— 与颜色无关、与
        YOLO 也无关（见 `segment_blocks_depth()` 的说明）；
      · 质心像素射线 ∩「板面 + 方块高」平面 → 真值（板系）。

⚠️ 真值**绝不能从 YOLO 检测框里取**（2026-10-25 改）：旧实现拿检测框当 ROI
再按颜色分割，于是「YOLO 没检到的块」根本没有真值 —— 「第 4 块是没看见还是
分数低」这个问题**永远答不了**；而误检会反过来变成真值。实测那一帧 22 个检测
里有 13 个落在 ChArUco 板上，其中 1 个还成了「真值」。颜色分割本身也不稳：
紫色块暗部色相漂移、亮色块顶面饱和度掉到 60 以下，逐色调参必然反复。

估计值同样进板系：`/targets_3d`（base 系）用**同一帧记录的 TF** 逆变换回
相机系，再用同一帧的板位姿进板系。手眼标定的误差在两边同时出现，
比较时**抵消**——所以这份误差量的是「感知定位」，不是「手眼」。

用法
====
    python3 scripts/eval_baseline.py --frame <帧目录> [--frame ...] \
        --out report.json [--debug-dir /tmp/eval_vis]

帧目录由 `scripts/capture_baseline.py` 生成（color.png / camera_info.json /
detections.json / targets_3d.json / tf.json）。

⚠️ 方格边长默认 **30.93 mm**，不是标称 30 mm：这张板用 100 mm 刻度尺量的
打印比例是 1.031（2026-10-24 实测）。用标称值会让板系整体缩 3.1%，
在 100 mm 外就是 3 mm 的系统误差，而且残差依旧好看（同一类静默错误）。
"""

import argparse
import glob
import json
import os
import sys

import numpy as np

# 标定板：与 docs/calib/charuco_A4_30mm_5x5.json 对应
SQUARE_MM_DEFAULT = 30.93     # 实测打印比例 1.031 × 30.0
MARKER_RATIO = 22.0 / 30.0    # 码边长 / 方格边长，打印缩放下同比
BLOCK_H_MM_DEFAULT = 28.5     # 方块高（现场口径）
BOARD_RECT_MM = 150.0         # 板面 150×150；落在板矩形内的「真值」是假阳性


# ─────────────────────────────── 基础 IO ──────────────────────────────

def load_json(path, what):
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except OSError as exc:
        raise SystemExit("读不到%s：%s（%s）" % (what, path, exc))


def load_frame(d):
    """一帧 → dict。缺文件直接退：**评估不能拿半截数据凑出数字**。"""
    need = ("color.png", "depth.png", "camera_info.json", "detections.json",
            "targets_3d.json", "tf.json")
    missing = [n for n in need if not os.path.isfile(os.path.join(d, n))]
    if missing:
        raise SystemExit("帧目录 %s 缺：%s" % (d, ", ".join(missing)))
    import cv2
    img = cv2.imread(os.path.join(d, "color.png"))
    if img is None:
        raise SystemExit("读不到彩色图：%s" % os.path.join(d, "color.png"))
    depth = cv2.imread(os.path.join(d, "depth.png"), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise SystemExit("读不到深度图：%s" % os.path.join(d, "depth.png"))
    info = load_json(os.path.join(d, "camera_info.json"), "相机内参")
    return {
        "dir": d,
        "img": img,
        "depth": depth,
        "K": np.array(info["K"], float).reshape(3, 3),
        "D": np.array(info.get("D") or [], float).reshape(-1, 1),
        "detections": load_json(os.path.join(d, "detections.json"), "2D 检测"),
        "targets": load_json(os.path.join(d, "targets_3d.json"), "3D 候选"),
        "tf": load_json(os.path.join(d, "tf.json"), "TF"),
        "meta": load_json(os.path.join(d, "meta.json"), "元数据")
        if os.path.isfile(os.path.join(d, "meta.json")) else {},
    }


# ─────────────────────────── 板位姿（板 ← 相机）───────────────────────────

def make_board(square_mm):
    import cv2
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    s = square_mm / 1000.0
    return cv2.aruco.CharucoBoard_create(5, 5, s, s * MARKER_RATIO, d)


def detect_markers(gray, dictionary):
    import cv2
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for c, i in zip(corners, np.array(ids).ravel())}


def board_pose(img, K, D, board):
    """→ dict(R, t, rms_med, rms_max, n_markers, sym) 或 None。

    方形 5×5 板有 8 重对称：`findChessboardCorners` 只给一组按行列排好的
    角点，不知道谁对谁。这里对 8 种对称各拟合一个单应，把**码中心**投回去、
    与实测码中心比距离，取中位最小的那种。码 ID 在这里是唯一的裁判。
    """
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCorners(
        gray, (4, 4), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
    c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), crit).reshape(16, 2)

    markers = detect_markers(gray, board.dictionary)
    if len(markers) < 3:
        return None
    obj3 = np.array(board.chessboardCorners, float).reshape(16, 3)
    id_index = {int(i): k for k, i in enumerate(np.array(board.ids).ravel())}
    marker_c = np.array([np.array(board.objPoints[k]).reshape(4, 3).mean(axis=0)
                         for k in range(len(id_index))])
    det_c = {i: pts.mean(axis=0) for i, pts in markers.items() if i in id_index}

    best = None
    for k in range(4):
        for transpose in (False, True):
            g = np.arange(16).reshape(4, 4)
            for _ in range(k):
                g = np.rot90(g)
            if transpose:
                g = g.T
            objp = obj3[g.reshape(-1)]
            H, _ = cv2.findHomography(objp[:, :2], c)
            if H is None:
                continue
            pred = cv2.perspectiveTransform(
                marker_c[:, :2].reshape(-1, 1, 2), H).reshape(-1, 2)
            dist = [float(np.linalg.norm(pred[id_index[i]] - p))
                    for i, p in det_c.items()]
            med = float(np.median(dist))
            if best is None or med < best[0]:
                best = (med, k, transpose, objp)
    if best is None:
        return None
    marker_med, k, transpose, objp = best

    ok, rvec, tvec = cv2.solvePnP(
        objp.reshape(-1, 1, 3), c.reshape(-1, 1, 2), K, D,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(objp.reshape(-1, 1, 3), rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - c, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    return {"R": R, "t": tvec.reshape(3),
            "rms_med_px": float(np.median(err)),
            "rms_max_px": float(err.max()),
            "marker_med_px": marker_med,
            "n_markers": len(det_c),
            "sym": {"rot90": k, "transpose": bool(transpose)}}


# ─────────────────────────── 积木顶面真值 ───────────────────────────

def segment_blocks_depth(depth, K, D, R_cb, t_cb,
                         z_lo_mm=16.0, z_hi_mm=45.0, min_area=800):
    """按**深度分层**找每块积木的顶面质心 —— 与颜色、与 YOLO 都无关。

    为什么不用颜色、也不用检测框（2026-10-25 改）
    ============================================
    旧实现拿 YOLO 检测框当 ROI 再按颜色分割。两个毛病都是**结构性的**：

      · YOLO 没检到的块**根本没有真值** —— 「第 4 块是没看见、还是分数低」
        这个问题就永远答不了；而误检反而会变成「真值」（实测那帧 22 个检测里
        13 个落在 ChArUco 板上，其中 1 个成了真值）。
      · 逐色调参必然反复：紫色块暗部色相漂移、亮色块顶面饱和度掉到 60 以下，
        同一套阈值换一批积木就失效。

    这里只用深度：每个有效像素换到板系 → 「离板面多高」。台面/板面在 0 附近，
    积木顶面在 20–35 mm（深度的**标度**偏差只影响这一层的绝对高度，不影响
    「哪些像素属于积木」）。取 16–45 mm 那一层做连通域，每块取**最高的显著层**
    （2 mm 直方图里占比 ≥15% 的最高格 ±2.5 mm）当顶面，取质心。

    ⚠️ 深度的绝对读数**不能**当真值高度用（彩色塑料比白纸读得更远，实测差
    4 mm 量级）；这里只借它定**像素**，真值高度用 `--block-h-mm`（卡尺口径）。

    返回 [{"u","v","n_top","area","truncated"}]；`truncated` = 连通域贴图像边缘
    （质心只是可见部分的质心，会偏，误差统计要剔除）。
    """
    import cv2
    h, w = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    d = depth.astype(np.float64) / 1000.0
    valid = d > 0
    p = np.stack([(us - cx) / fx * d, (vs - cy) / fy * d, d], axis=-1)
    z = ((p - t_cb) @ R_cb)[:, :, 2] * 1000.0          # 板系高度 mm

    m = (valid & (z > z_lo_mm) & (z < z_hi_mm)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)

    out = []
    for k in range(1, n):
        if stats[k, cv2.CC_STAT_AREA] < min_area:
            continue
        # ⚠️ 闭运算会把块内的小洞（深度 0、或者 1 m 之外的野值）填回来，连通域
        #    于是「看起来」是整块；但这些像素的 z 是垃圾：d=0 换算出 z=+400 mm，
        #    1 m 野值换算出 z=−600 mm。它们只许参与连通性，不许进直方图和质心，
        #    否则一层真顶面被拉散在几百毫米的范围里、一格都不显著（合成图 [4]）。
        comp = (lab == k) & valid & (z > z_lo_mm) & (z < z_hi_mm)
        zz = z[comp]
        zz = zz[zz <= np.percentile(zz, 95)]      # 甩掉零星深度野值
        if zz.size < min_area // 2:
            continue
        # 2 mm 一格。⚠️ 不能用 arange：z 恰好落在格边界上时（合成图就是精确
        # 28.000…025），浮点误差会把最大值甩出最后一格，整块判成「没有显著层」。
        # linspace 保证 hi 就是最后一格右边界（histogram 的最后一格右端闭合）。
        lo, hi = float(zz.min()), float(zz.max())
        nb = max(1, int(np.ceil((hi - lo) / 2.0)))
        if nb > 1:
            edges = np.linspace(lo, hi, nb + 1)
        else:                                 # 高度全相同（合成图/理想平面）
            edges = np.array([lo - 1.0, hi + 1.0])
        hist, edges = np.histogram(zz, bins=edges)
        need = 0.15 * zz.size
        bins = [i for i in range(len(hist)) if hist[i] >= need]
        if not bins:
            continue
        z_top = 0.5 * (edges[bins[-1]] + edges[bins[-1] + 1])
        top = comp & (np.abs(z - z_top) <= 2.5)
        if top.sum() < 100:
            continue
        ty, tx = np.nonzero(top)
        x = stats[k, cv2.CC_STAT_LEFT]
        y = stats[k, cv2.CC_STAT_TOP]
        bw = stats[k, cv2.CC_STAT_WIDTH]
        bh = stats[k, cv2.CC_STAT_HEIGHT]
        out.append({
            "u": float(tx.mean()), "v": float(ty.mean()),
            "n_top": int(top.sum()), "z_top_mm": round(float(z_top), 2),
            "area": int(stats[k, cv2.CC_STAT_AREA]),
            "truncated": bool(x <= 1 or y <= 1 or x + bw >= w - 1
                              or y + bh >= h - 1),
        })
    return out


def ray_plane_board(u, v, K, D, R_cb, t_cb, z_plane):
    """像素 → 射线 ∩ 板系平面 `z = z_plane` 的点（板系，米）。"""
    import cv2
    pts = cv2.undistortPoints(np.array([[[u, v]]], float), K, D).reshape(2)
    d_cam = np.array([pts[0], pts[1], 1.0])
    o_board = -R_cb.T @ t_cb
    d_board = R_cb.T @ d_cam
    if abs(d_board[2]) < 1e-9:
        return None
    s = (z_plane - o_board[2]) / d_board[2]
    if s <= 0:
        return None
    return o_board + s * d_board


# ─────────────────────────── 估计值 → 板系 ───────────────────────────

def _tf_matrix(entry):
    R = quat_to_matrix(entry["rotation"])
    t = np.array([entry["translation"][k] for k in ("x", "y", "z")], float)
    return R, t


def quat_to_matrix(q):
    x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def targets_to_board(frame, R_cb, t_cb):
    """`/targets_3d`（base 系）→ 板系。用的 TF 是**同一帧记录的那份**。"""
    tf_ok = (frame["tf"].get("ok") or {})
    key = "base_link:camera_color_optical_frame"
    if key not in tf_ok:
        raise SystemExit("帧 %s 缺 %s 的 TF（记录到的有：%s）"
                         % (frame["dir"], key, ", ".join(sorted(tf_ok))))
    R_bc, t_bc = _tf_matrix(tf_ok[key])          # p_base = R_bc p_cam + t_bc
    out = []
    tag = frame["targets"].get("frame_id", "")
    if tag != "base_link":
        raise SystemExit("帧 %s 的 /targets_3d 在 %r 系，不是 base_link —— "
                         "比较的口径不对" % (frame["dir"], tag))
    for t in frame["targets"]["targets"]:
        p_base = np.array([t["x"], t["y"], t["z"]], float)
        p_cam = R_bc.T @ (p_base - t_bc)
        p_board = R_cb.T @ (p_cam - t_cb)
        out.append({"p_board": p_board, "p_base": p_base,
                    "score": (t.get("best") or {}).get("score", 0.0),
                    "class_id": (t.get("best") or {}).get("class_id", "")})
    return out


# ─────────────────────────────── 一帧 ───────────────────────────────

def assoc(truths, ests, max_mm):
    """贪心最近邻。→ (pairs, 未配对的真值下标, 未配对的估计下标)。"""
    pairs, used_t, used_e = [], set(), set()
    cand = []
    for i, tr in enumerate(truths):
        for j, es in enumerate(ests):
            d = float(np.linalg.norm(tr - es) * 1000.0)
            if d <= max_mm:
                cand.append((d, i, j))
    for d, i, j in sorted(cand):
        if i in used_t or j in used_e:
            continue
        pairs.append((i, j, d))
        used_t.add(i); used_e.add(j)
    return (pairs,
            [i for i in range(len(truths)) if i not in used_t],
            [j for j in range(len(ests)) if j not in used_e])


def eval_frame(f, square_mm, block_h_mm, max_assoc_mm, debug_dir=None):
    board = make_board(square_mm)
    pose = board_pose(f["img"], f["K"], f["D"], board)
    if pose is None:
        return {"dir": f["dir"], "ok": False,
                "error": "板没解出来（内角点不足或码太少）"}
    R_cb, t_cb = pose["R"], pose["t"]
    z_top = block_h_mm / 1000.0

    # ---- 真值：深度分层找块 → 顶面质心射线 ∩「板面 + 块高」 ----
    blocks = segment_blocks_depth(f["depth"], f["K"], f["D"], R_cb, t_cb)
    truths, truth_meta = [], []
    for b in blocks:
        p = ray_plane_board(b["u"], b["v"], f["K"], f["D"], R_cb, t_cb, z_top)
        if p is None:
            continue
        truths.append(p)
        truth_meta.append(b)

    # ---- 估计：落在**板矩形**内的候选单列 ----
    # ChArUco 板摆在视野里当尺子，YOLO 会对板的黑白图案出框（实测一帧 13 个），
    # 那是**摆拍装置**造成的误检，不是算法缺陷，也不该稀释精度统计 —— 但必须
    # 单独数出来，否则「假阳性」会把它的账算到算法头上。
    ests_raw = targets_to_board(f, R_cb, t_cb)
    ests, ests_kept, on_board = [], [], []
    for e in ests_raw:
        pb = e["p_board"] * 1000.0
        if 0.0 <= pb[0] <= BOARD_RECT_MM and 0.0 <= pb[1] <= BOARD_RECT_MM:
            on_board.append({"est_mm": pb.round(1).tolist(),
                             "score": e["score"]})
            continue
        ests.append(e["p_board"])
        ests_kept.append(e)

    pairs, miss_t, miss_e = assoc(truths, ests, max_assoc_mm)
    rows = []
    for i, j, d in pairs:
        rows.append({"err_mm": round(d, 3),
                     "truth_mm": (truths[i] * 1000.0).round(1).tolist(),
                     "est_mm": (ests[j] * 1000.0).round(1).tolist(),
                     "score": ests_kept[j]["score"],
                     "truth_truncated": bool(truth_meta[i]["truncated"])})
    # 贴图像边缘的块：质心只是**可见部分**的质心，误差不算数（覆盖率照算）
    errs = [r["err_mm"] for r in rows if not r["truth_truncated"]]
    covered = []
    for t in truths:
        dmin = min((float(np.linalg.norm(t - e)) * 1000.0 for e in ests),
                   default=None)
        covered.append(dmin is not None and dmin <= max_assoc_mm)
    fp = [{"est_mm": (ests[j] * 1000.0).round(1).tolist(),
           "score": ests_kept[j]["score"],
           "class_id": ests_kept[j]["class_id"]} for j in miss_e]

    out = {
        "dir": f["dir"], "ok": True,
        "board": {k: pose[k] for k in ("rms_med_px", "rms_max_px",
                                       "marker_med_px", "n_markers", "sym")},
        "n_detections": len(f["detections"]["detections"]),
        "n_targets": len(ests_raw),
        "n_targets_offboard": len(ests),
        "n_truth": len(truths),
        "n_truth_truncated": sum(1 for b in truth_meta if b["truncated"]),
        "n_matched": len(pairs),
        "coverage": {"covered": sum(covered), "total": len(truths)},
        "matched": rows,
        "false_positives": fp,
        "board_artifacts": on_board,
        "truth": [{"u": round(b["u"], 1), "v": round(b["v"], 1),
                   "z_top_mm": b["z_top_mm"], "n_top": b["n_top"],
                   "truncated": bool(b["truncated"]),
                   "covered": c} for b, c in zip(truth_meta, covered)],
        "errors_mm": {"median": float(np.median(errs)) if errs else None,
                      "p95": float(np.percentile(errs, 95)) if errs else None,
                      "max": float(max(errs)) if errs else None},
        "scores": {
            "all_targets": sorted(round(e["score"], 3) for e in ests_kept),
            "board_artifacts": sorted(round(o["score"], 3) for o in on_board),
        },
    }

    if debug_dir:
        out["debug_png"] = _draw_debug(f, R_cb, t_cb, truths, ests,
                                       pose, pairs, debug_dir)
    return out


def _draw_debug(f, R_cb, t_cb, truths, ests, pose, pairs, debug_dir):
    """把真值/估计都投回像素画出来 —— 肉眼一眼就能看出关联对不对。"""
    import cv2
    img = f["img"].copy()
    K, D = f["K"], f["D"]
    axis = np.array([[0, 0, 0], [0.03, 0, 0], [0, 0.03, 0], [0, 0, 0.03]], float)
    p_cam = (R_cb @ axis.T + t_cb.reshape(3, 1)).T
    pix, _ = cv2.projectPoints(p_cam.reshape(-1, 1, 3), np.zeros(3),
                               np.zeros(3), K, D)
    pix = pix.reshape(-1, 2).astype(int)
    cv2.line(img, tuple(pix[0]), tuple(pix[1]), (0, 0, 255), 2)
    cv2.line(img, tuple(pix[0]), tuple(pix[2]), (0, 255, 0), 2)
    cv2.line(img, tuple(pix[0]), tuple(pix[3]), (255, 0, 0), 2)

    def to_px(p_board):
        p_cam = R_cb @ p_board + t_cb
        px, _ = cv2.projectPoints(p_cam.reshape(1, 1, 3), np.zeros(3),
                                  np.zeros(3), K, D)
        return tuple(px.reshape(2).astype(int))

    for p in truths:
        cv2.drawMarker(img, to_px(p), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
    for p in ests:
        cv2.drawMarker(img, to_px(p), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 10, 2)
    os.makedirs(debug_dir, exist_ok=True)
    name = os.path.basename(os.path.normpath(f["dir"])) + ".png"
    path = os.path.join(debug_dir, name)
    cv2.imwrite(path, img)
    return path


# ─────────────────────────────── 汇总 ───────────────────────────────

def summarize(frames):
    ok = [f for f in frames if f.get("ok")]
    if not ok:
        return {"frames": len(frames), "ok": 0}
    all_err, scores = [], []
    n_matched = 0
    for f in ok:
        use = [r for r in f["matched"] if not r["truth_truncated"]]
        all_err += [r["err_mm"] for r in use]
        n_matched += len(use)
        scores += f["scores"]["all_targets"]
    sweep = {}
    for th in (0.05, 0.1, 0.2, 0.3, 0.5, 0.6):
        hit = sum(1 for f in ok for r in f["matched"]
                  if r["score"] >= th and not r["truth_truncated"])
        tot = sum(1 for f in ok for e in f["scores"]["all_targets"] if e >= th)
        sweep["%.2f" % th] = {"matched": hit, "targets": tot,
                              "precision": round(hit / tot, 3) if tot else None}
    n_truth = sum(f["n_truth"] for f in ok)
    n_cov = sum(f["coverage"]["covered"] for f in ok)
    return {
        "frames": len(frames), "ok": len(ok),
        "detections": sum(f["n_detections"] for f in ok),
        "targets": sum(f["n_targets_offboard"] for f in ok),
        "targets_all": sum(f["n_targets"] for f in ok),
        "board_artifacts": sum(len(f["board_artifacts"]) for f in ok),
        "truths": n_truth,
        "truths_truncated": sum(f["n_truth_truncated"] for f in ok),
        "covered": n_cov,
        "coverage_rate": round(n_cov / n_truth, 3) if n_truth else None,
        "matched": n_matched,
        "false_positives": sum(len(f["false_positives"]) for f in ok),
        "errors_mm": {"median": float(np.median(all_err)) if all_err else None,
                      "p95": float(np.percentile(all_err, 95)) if all_err else None,
                      "max": float(max(all_err)) if all_err else None},
        "score_sweep": sweep,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frame", action="append", required=True,
                    help="帧目录（可重复）；也接受通配符")
    ap.add_argument("--square-mm", type=float, default=SQUARE_MM_DEFAULT)
    ap.add_argument("--block-h-mm", type=float, default=BLOCK_H_MM_DEFAULT)
    ap.add_argument("--max-assoc-mm", type=float, default=40.0,
                    help="真值与估计配对的最近邻上限（毫米）")
    ap.add_argument("--debug-dir", default=None, help="把叠加图写到这个目录")
    ap.add_argument("--out", default=None, help="机器可读 JSON 输出路径")
    args = ap.parse_args(argv)

    dirs = []
    for pat in args.frame:
        hit = sorted(glob.glob(os.path.expanduser(pat)))
        dirs += [h for h in hit if os.path.isdir(h)] or [os.path.expanduser(pat)]
    if not dirs:
        raise SystemExit("没有可评估的帧目录")

    frames = []
    for d in dirs:
        f = load_frame(d)
        r = eval_frame(f, args.square_mm, args.block_h_mm,
                       args.max_assoc_mm, args.debug_dir)
        frames.append(r)
        print(_fmt_frame(r))

    s = summarize(frames)
    print("\n" + "=" * 70)
    print("汇总：%d/%d 帧可用；检测 %d / 候选 %d（板区摆拍 %d）/ 真值 %d（贴边 %d）"
          " / 覆盖 %d/%d / 配对 %d / 假阳性 %d"
          % (s["ok"], s["frames"], s["detections"], s["targets"],
             s["board_artifacts"], s["truths"], s["truths_truncated"],
             s["covered"], s["truths"], s["matched"], s["false_positives"]))
    if s.get("coverage_rate") is not None:
        print("覆盖率：%.1f%%" % (s["coverage_rate"] * 100))
    e = s["errors_mm"]
    if e["median"] is not None:
        print("误差（板系，毫米）：中位 %.2f / p95 %.2f / 最大 %.2f"
              % (e["median"], e["p95"], e["max"]))
    print("分数扫描（阈值 → 命中/候选，精度）：")
    for th, v in s["score_sweep"].items():
        print("  ≥%s: %d/%d  精度 %s" % (th, v["matched"], v["targets"],
                                         v["precision"]))

    if args.out:
        doc = {"frames": frames, "summary": s,
               "params": {"square_mm": args.square_mm,
                          "block_h_mm": args.block_h_mm,
                          "max_assoc_mm": args.max_assoc_mm}}
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1)
        print("→ %s" % os.path.abspath(args.out))
    return 0


def _fmt_frame(r):
    head = os.path.basename(os.path.normpath(r["dir"]))
    if not r.get("ok"):
        return "%-18s ❌ %s" % (head, r.get("error"))
    e = r["errors_mm"]
    cov = r["coverage"]
    line = ("%-18s 板 %.2f/%.2f px 码 %d | 检测 %2d 候选 %2d(板区 %2d) "
            "真值 %2d(贴边 %d) 覆盖 %d/%d 配对 %d 假阳性 %2d | "
            "误差 中位 %s p95 %s 最大 %s"
            % (head, r["board"]["rms_med_px"], r["board"]["rms_max_px"],
               r["board"]["n_markers"], r["n_detections"],
               r["n_targets_offboard"], len(r["board_artifacts"]),
               r["n_truth"], r["n_truth_truncated"],
               cov["covered"], cov["total"], r["n_matched"],
               len(r["false_positives"]),
               "—" if e["median"] is None else "%.2f" % e["median"],
               "—" if e["p95"] is None else "%.2f" % e["p95"],
               "—" if e["max"] is None else "%.2f" % e["max"]))
    if r.get("debug_png"):
        line += "\n   → %s" % r["debug_png"]
    return line


if __name__ == "__main__":
    sys.exit(main())
