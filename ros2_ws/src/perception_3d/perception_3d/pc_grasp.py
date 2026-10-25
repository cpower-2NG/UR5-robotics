"""点云抓取点：检测框 → 「台面上那块积木的顶面中心」在目标系下的坐标。

为什么单独一个模块
==================
这段数学先在 ``scripts/eval_baseline.py`` 里离线长出来（2026-10-25），
在 10 帧真实腕部画面上量到 **90/90 覆盖、0 误发、3D 中位 3.49 mm**；
而原有管线把射线截在 base 系**固定平面** ``target_plane_z`` 上，那个平面
比真实顶面高 8–16 mm，3D 中位 11.48 mm。要真抓，就得让节点走这条路。

于是这里只放**纯 numpy/cv2**，不 import rclpy：

  · 节点（``projector_node.py``，``grasp_point_mode:=pc_top``）直接调用；
  · 离线评估（``scripts/eval_baseline.py``，``--est pc``）调用**同一份**。

这不是洁癖。这个仓库里颜色曾同时在四处各写一份，对不齐时**完全静默**
（标签与 names 错位，训练照样跑完）。抓取点的数学要是在节点和评估里各
长一份，症状会是「离线很好、上板偏几毫米」，而两边都不知道该信谁。

怎么算（每一步都对应一条实测教训）
==================================
1. **局部桌面**：检测框外扩 40 px 的 ROI 里，取 z 中位 ±8 mm 的点拟合
   平面。不能用「框外一圈」—— YOLO 框常比块**小**（实测一块 39×51 px，
   块实际 ~64 px），贴着框的那一圈其实还在块顶上，于是局部桌面被拟成块
   顶、高度闸门随机误杀。
2. **块体**：ROI 内高出局部桌面 >8 mm 的连通域中、包含框中心的那一块。
   外扩检测框是为了矩形积木：YOLO 框可能只盖住它的一半。
3. **顶面**：该连通域 z 最高的 10% 的中位层 ±2.5 mm 拟合平面。
4. **顶面中心 (u,v)**：取**彩色图**里该块最亮的一层（Otsu 分亮/暗）的
   质心。为什么不用深度点质心：0.7 m 处深度有位置相关的偏置，顶面被压
   成几度的斜坡，按固定层高选点只选到斜坡的一条带，质心偏 5–10 px；而
   亮层质心与立方体模型真值差 1–2 px。
5. **射线 ∩ 顶面平面**：位置不取深度值，取第 4 步那个像素的射线与第 3
   步那个平面的交点 —— 绝对深度偏置于是只进「平面高度」，不进 x/y。
6. **高度闸门**：交点高出局部桌面不到 15 mm 的不要 —— ChArUco 板、白色
   支架、空台面都会出框，它们贴桌面。

⚠️ 已知长尾（2026-10-25，0.7 m）：深度绝对偏置修不掉，逐帧 Z 中位在
±3 mm 间换号；不规则积木（2 块叠放/2×1 长条）会被当成一块立方体，
中心偏 ~5 mm。**精度验收要回近距（~0.35 m）**；高位只做覆盖与粗定位。
"""

import cv2
import numpy as np

# ---- 默认参数：与离线评估实测过的那组一致（改任何一个数都要重跑 10 帧）----
MIN_SCORE = 0.30          # 低于这个分数不进点云估计（板/支架的误检多在 0.05–0.4）
TOP_POINTS_MIN = 200      # 顶面点过少 = 不是一块积木的顶面
TOP_BAND_MM = 2.5         # 顶面层厚：取「最高层 ±2.5 mm」
MIN_HEIGHT_MM = 15.0      # 抓取点高出局部桌面不到 15 mm 的，不是积木
PAD_PX = 22               # 块体连通域：检测框外扩（YOLO 框可能只盖住半块）
LOCAL_PAD_PX = 40         # 局部桌面：检测框外扩
LOCAL_BAND_MM = 8.0       # 局部桌面：z 中位附近取带
MIN_COMPONENT_MM = 8.0    # 块体：高出局部桌面多少才算「不是桌面」
GLOBAL_BAND_LO_M = -0.08  # 整帧兜底桌面：相对 table_hint 的下界
GLOBAL_BAND_HI_M = 0.05   # 整帧兜底桌面：相对 table_hint 的上界


def base_cloud(depth_m, K, R_bc, t_bc):
    """深度图 → 目标系点云。→ ``(P, valid)``，``P`` 形状 ``(H, W, 3)``。

    ``R_bc, t_bc`` 是 target_frame ← 相机光学系（ROS TF 的语义，不是它的逆）。
    深度 0 视为无效；点云里这些位置的值无意义，必须配 ``valid`` 用。
    """
    h, w = depth_m.shape[:2]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    p_cam = np.stack([(us - cx) / fx * depth_m,
                      (vs - cy) / fy * depth_m,
                      depth_m], axis=-1)
    return p_cam @ R_bc.T + t_bc, depth_m > 0


def fit_plane(P, thresh_mm=3.0, iters=600, seed=0):
    """RANSAC 平面，法向约束在竖直附近（桌面/顶面）。→ ``(n, c, n_inliers)``。

    ``n`` 未必朝上（RANSAC 的随机三点不保证方向），调用方按需翻号。
    """
    if len(P) < 50:
        return None
    rng = np.random.default_rng(seed)
    best = None
    tol = thresh_mm / 1000.0
    for _ in range(iters):
        q = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(q[1] - q[0], q[2] - q[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if abs(n[2]) < 0.9:               # 桌面法向与 z 轴夹角 < 26°
            continue
        c = n @ q[0]
        cnt = int((np.abs(P @ n - c) < tol).sum())
        if best is None or cnt > best[0]:
            best = (cnt, n, c)
    if best is None:
        return None
    cnt, n, c = best
    for _ in range(3):                    # 用内点重拟合三轮
        I = P[np.abs(P @ n - c) < tol * 0.66]
        if len(I) < 30:
            break
        m = I.mean(0)
        _, _, vt = np.linalg.svd(I - m, full_matrices=False)
        n = vt[-1]
        c = n @ m
    return n, c, cnt


def global_table_plane(P, valid, table_hint=0.0):
    """整帧兜底桌面。局部拟合失败时用它，不让一帧整帧退掉。

    ``table_hint``：桌面在目标系里的高度（base 系下桌≈0）。带是相对它取的，
    不写死绝对高度 —— 台面高度换个装配就变，写死会**静默**退化成「没有
    平面、全部不发」。
    """
    z = P[:, :, 2]
    band = valid & (z > table_hint + GLOBAL_BAND_LO_M) & (z < table_hint + GLOBAL_BAND_HI_M)
    pts = P[band][::5]
    if len(pts) < 500:
        return None
    fit = fit_plane(pts)
    return None if fit is None else (fit[0], fit[1])


def local_table_plane(P, valid, box, pad_px=LOCAL_PAD_PX, band_mm=LOCAL_BAND_MM,
                      global_plane=None):
    """在检测框外扩 ``pad_px`` 的 ROI 里拟合局部桌面。

    用「z 中位 ±band」而不是「框外一圈」：ROI 里桌面占绝大多数，块顶高出
    桌面 28.5 mm ⇒ 带内只含桌面，块在带外（见模块文档第 1 条）。

    ``box`` 是 ROI 的像素边界 ``(x0, x1, y0, y1)``。返回 ``(n, c)`` 或
    ``global_plane``。
    """
    h, w = valid.shape
    x0, x1, y0, y1 = box
    x0 = max(0, x0 - pad_px); x1 = min(w, x1 + pad_px)
    y0 = max(0, y0 - pad_px); y1 = min(h, y1 + pad_px)
    rv = valid[y0:y1, x0:x1]
    pts = P[y0:y1, x0:x1][rv]
    if len(pts) < 500:
        return global_plane
    zmed = float(np.median(pts[:, 2]))
    pts = pts[np.abs(pts[:, 2] - zmed) < band_mm / 1000.0]
    if len(pts) < 300:
        return global_plane
    fit = fit_plane(pts)
    if fit is None or fit[2] < 200:
        return global_plane
    n, c, _ = fit
    if abs(n[2]) <= 0.985:             # 局部桌面倾角 < 10°
        return global_plane
    return n, c


def plane_height(P, n, c):
    """点到平面的**有向**高度（朝上为正）。"""
    return (P[..., 0] * n[0] + P[..., 1] * n[1] + P[..., 2] * n[2] - c) \
        * (1.0 if n[2] > 0 else -1.0)


def detection_box(det_cx, det_cy, size_x, size_y, pad_px=PAD_PX):
    """检测框（像素）→ 外扩后的 ROI 边界，并夹到图像内。

    返回 ``None`` 表示框退化（小于 12 px 见方）—— 那种框上做连通域没有意义。
    """
    x0 = int(round(det_cx - size_x / 2.0 - pad_px))
    x1 = int(round(det_cx + size_x / 2.0 + pad_px))
    y0 = int(round(det_cy - size_y / 2.0 - pad_px))
    y1 = int(round(det_cy + size_y / 2.0 + pad_px))
    return x0, x1, y0, y1


def estimate_top_point(P, valid, img, det, plane, K, R_bc, t_bc,
                       rel=None,
                       pad_px=PAD_PX, top_band_mm=TOP_BAND_MM,
                       min_top=TOP_POINTS_MIN, min_height_mm=MIN_HEIGHT_MM):
    """一块积木 → 目标系抓取点（顶面中心）或 ``None``。

    参数
    ----
    P, valid : 目标系点云与有效掩码（见 ``base_cloud``）
    img      : BGR 彩色图（与深度**像素对齐**；真机用 aligned depth）
    det      : ``{'cx','cy','size_x','size_y'}``，像素坐标、原点左上
    plane    : 局部桌面 ``(n, c)``；``None`` 直接放弃
    K        : 3×3 内参
    R_bc,t_bc: target_frame ← 相机光学系
    rel      : 可选 ``(R, t)``，把结果同时变到一个参考系（离线评估用它拿
               板系坐标）。**约定是「相机光学系 ← 参考系」**，与 ROS TF 的
               ``camera←board`` 同向 —— 参考系不是目标系，所以这里必须先用
               ``(R_bc, t_bc)`` 转回相机系再转参考系（2026-10-25 自检抓到
               过一版直接拿 base 点去减板原点，结果偏 150 mm）。

    返回 ``None`` 或 dict：``p_base`` / ``p_rel`` / ``score`` / ``class_id`` /
    ``n_top`` / ``z_above_mm``。
    """
    if plane is None:
        return None
    n, c = plane
    h, w = valid.shape
    cx, cy = det["cx"], det["cy"]
    box = detection_box(cx, cy, det["size_x"], det["size_y"], pad_px)
    x0, x1, y0, y1 = box
    if x1 - x0 < 12 or y1 - y0 < 12:
        return None
    x0 = max(0, x0); x1 = min(w, x1)
    y0 = max(0, y0); y1 = min(h, y1)
    roi = P[y0:y1, x0:x1]
    rv = valid[y0:y1, x0:x1]

    # ---- 块体：高出局部桌面、含框中心的连通域 ----
    hh = plane_height(roi, n, c)
    mask = ((hh > MIN_COMPONENT_MM / 1000.0) & rv).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ncomp, lab, _, _ = cv2.connectedComponentsWithStats(mask, 8)
    if ncomp < 2:
        return None
    ccx, ccy = int(round(cx - x0)), int(round(cy - y0))
    ccx = min(max(ccx, 0), lab.shape[1] - 1)
    ccy = min(max(ccy, 0), lab.shape[0] - 1)
    k = int(lab[ccy, ccx])
    if k == 0:                                    # 框中心落在缝里：取最近的域
        ys, xs = np.nonzero(lab)
        if len(ys) == 0:
            return None
        i = int(np.argmin((xs - ccx) ** 2 + (ys - ccy) ** 2))
        k = int(lab[ys[i], xs[i]])
    comp = (lab == k) & rv
    if int(comp.sum()) < min_top:
        return None

    # ---- 顶面层：最高 10% 的中位 ±band ----
    zc = roi[:, :, 2][comp]
    zt = float(np.percentile(zc, 90))
    sel = comp & (np.abs(roi[:, :, 2] - zt) < top_band_mm / 1000.0)
    if int(sel.sum()) < min_top:
        return None
    py, px = np.nonzero(sel)
    fit = fit_plane(roi[py, px], thresh_mm=2.5)
    if fit is None:
        return None
    n_top, c_top, _ = fit
    if n_top[2] < 0:
        n_top, c_top = -n_top, -c_top

    # ---- 顶面中心 (u,v)：彩色图里最亮那一层的质心 ----
    hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    S_ch, V_ch = hsv[:, :, 1], hsv[:, :, 2]
    # 区域判据用「相对**局部桌面**的高度」，不是绝对深度：0.7 m 处绝对
    # 高度逐帧在 ±3 mm 间换号，高位帧里更能漂 10–30 mm —— 拿绝对值当判据
    # 会把整块切掉。局部桌面是**逐块**拟合的，它跟着漂移一起走。
    in_zone = (hh > -10.0 / 1000.0) & (hh < 50.0 / 1000.0)
    cm = ((S_ch > 55) & (V_ch > 40) & rv & in_zone).astype(np.uint8)
    cm = cv2.morphologyEx(cm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cm = cv2.morphologyEx(cm, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    nc, lc, _, _ = cv2.connectedComponentsWithStats(cm, 8)
    kc = int(lc[ccy, ccx])
    if kc == 0 and nc > 1:
        ys, xs = np.nonzero(lc)
        i = int(np.argmin((xs - ccx) ** 2 + (ys - ccy) ** 2))
        kc = int(lc[ys[i], xs[i]])
    cmask = (lc == kc) if kc else (comp > 0)
    if int(cmask.sum()) >= 100:
        vv = V_ch[cmask].reshape(-1, 1)
        thr = cv2.threshold(vv, 0, 255,
                            cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
        top = cmask & (V_ch > thr)
        if int(top.sum()) < 100:
            top = cmask
    else:
        top = comp
    ys, xs = np.nonzero(top)
    u_c = float(xs.mean() + x0)
    v_c = float(ys.mean() + y0)

    # ---- 像素射线 ∩ 顶面平面 ----
    fx, fy = K[0, 0], K[1, 1]
    cx_px, cy_px = K[0, 2], K[1, 2]
    d_cam = np.array([(u_c - cx_px) / fx, (v_c - cy_px) / fy, 1.0])
    d_base = R_bc @ d_cam
    denom = float(n_top @ d_base)
    if abs(denom) < 1e-12:
        return None
    t = float((c_top - n_top @ t_bc) / denom)
    p_base = t_bc + t * d_base

    # ---- 高度闸门：贴桌面的东西（板、支架、空台面误检）一律不发 ----
    plane_z = ((c - n[0] * p_base[0] - n[1] * p_base[1]) / n[2]
               if abs(n[2]) > 1e-9 else p_base[2])
    z_above_mm = float((p_base[2] - plane_z) * 1000.0)
    if z_above_mm < min_height_mm:
        return None

    out = {"p_base": p_base, "p_rel": None,
           "score": float((det.get("best") or {}).get("score", 0.0)),
           "class_id": (det.get("best") or {}).get("class_id", ""),
           "n_top": int(top.sum()), "z_above_mm": round(z_above_mm, 2),
           "uv": (u_c, v_c)}
    if rel is not None:
        R_rel, t_rel = rel                      # 相机 ← 参考系
        p_cam = R_bc.T @ (p_base - t_bc)
        out["p_rel"] = R_rel.T @ (p_cam - t_rel)
    return out
