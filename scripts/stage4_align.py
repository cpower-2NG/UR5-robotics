#!/usr/bin/env python3
"""E4b：俯视 burst → 低悬停同帧相对量 ``(dx, dy, dyaw)`` / 抓后偏心 ``e``。

为什么是「同帧相对量」
----------------------
俯视的深度读数有绝对偏置（台面读远 ~1.9%，B1 里 −30 mm），可这不等于外参
错了。**同一帧里**量「块顶面质心」与「夹爪轴」的相对差，外参只这样进入：

· 块顶面质心 → ``graspnet_candidates.find_blocks`` 反投影再投影回像素，结果对
  ``(R, t)`` **严格不变**（两者互为逆操作），深度尺度也约掉；
· 夹爪轴横向 ``u`` → 两指深色列的**像素中点**，不进外参；
· 夹爪轴纵向 ``v`` → FK 在块顶高度的投影。这不是纯像素量，但 B1 的 20 帧实测
  里它和图像上指的指尖下沿差 **1 px**（≈1 mm），够用。

所以 ``Δuv = 块 − 轴`` 基本不含平移/尺度偏置，换成毫米时只用到 ``R`` 的旋转。
倾角误差修不了（入口实验 §7.3 已写明）—— 这正是 E4b 要修的东西。

**符号**：``(dx, dy)`` = 块相对轴的位移 ⇒ 把计划目标平移 ``+(dx, dy)``、抓取角
转 ``+dyaw`` 就是对准。

量法（逐帧）
------------
1. FK（``joint_states_*.json`` + URDF）给出 tool0 与指尖平面，只当**先验**：
   定位指尖所在的像素带、并给出轴在图像里的延伸方向。
2. 两指在图像里近似竖直、颜色深。在锚点上方取 3 个候选带，按列统计深色像素
   比例，得到两条「深色列串」= 两指；取满足「间距 20–125 px、中点贴着 FK 预测」
   的那一对的中点 ``u_mid``。台面用**深度闸门**剔掉（``z_base < 台面+50 mm``
   的像素不算深色指）—— 否则阴影/暗台面会把列串连成一片（2026-11-08 实测）。
3. 轴的纵向 = 「带中点 v」+ FK/外参给出的**轴向微分**（带 → 块顶高）。等价于
   直接投影轴在块顶高度的 ``v``，拆成两步是为了让 ``u`` 与 ``v`` 分别来自
   实测与投影。
4. 块顶面质心 = ``find_blocks`` 在该帧的输出（与规划/判分同一条路径）。
5. ``Δuv`` 用块顶深度的 ``(fx, fy)`` 与 ``R`` 的旋转解出 base 系 ``(dx, dy)``：
       du = fx/zc · (R00·dx + R10·dy)
       dv = fy/zc · (R01·dx + R11·dy)
6. ``dyaw`` = 块顶面矩形角（find_blocks 的 ``yaw_rect``，base 系）− 计划抓取角，
   折到 (−45°, 45°]（方块 90° 自对称）。

多帧共识（**不许取单帧**，见入口实验 §13.3：burst 前 2 帧是暖机坏帧）
------------------------------------------------------------------
默认丢前 2 帧、取至多 10 帧；``dx/dy`` 取中位、``dyaw`` 取 90° 周期圆均值；
另报逐帧极差。帧数 <3 或分不出两指 → ``ok=false``（fail-closed，调用方不许猜）。

用法
----
  # 抓前低悬停（tool0 z≈360 mm，见入口实验 §7.3）
  python3 scripts/stage4_align.py --burst <run>/e4b/r1/block-3 --urdf <urdf> \
      --block-prior-mm 420,300 --expect-class yellow --grasp-yaw-deg 12.3 \
      --out <run>/e4b/r1/block-3.json

  # 抓后偏心（在 pick 计划的 lift 航点拍；块的顶面夹在两指之间）
  python3 scripts/stage4_align.py --burst <dir> --urdf <urdf> --held \
      --expect-class yellow --block-top-mm 169.5 --out <dir>/e.json

退出码：0 = 出了数（看 ``checks`` 判是否收敛）；2 = 量不出来（``ok=false``）。
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import block_classes                                              # noqa: E402
import graspnet_candidates as gc                                  # noqa: E402
import measure_held as mh                                         # noqa: E402
import urkin                                                      # noqa: E402

TOOL0_TO_BARE_FINGERTIP = 0.2042     # tool0 → 裸指尖外沿（AG-95 实测）
FINGER_DARK_V = 100                  # V < 100 判「深色指」（2026-11-08 B1 实测）
FINGER_RUN_MIN = 0.45                # 列深色比例的「命中」阈值
TABLE_BAND_M = 0.05                  # z_base < 台面+50 mm 的像素不当指
DEFAULT_DROP = 2                     # burst 暖机坏帧（§13.3）
DEFAULT_MAX = 10
MIN_FRAMES = 3
TOL_MM = 2.0
TOL_DEG = 2.0


# ---------------------------------------------------------------- 纯几何
def project_pt(p_base, K, R, t):
    """base 点 → ``(u, v, z_cam)``。``R, t`` 是 base←camera。"""
    pc = R.T @ (np.asarray(p_base, float) - t)
    z = float(pc[2])
    if z <= 1e-6:
        return None
    return (float(K[0, 0] * pc[0] / z + K[0, 2]),
            float(K[1, 1] * pc[1] / z + K[1, 2]), z)


def axis_shift_uv(axis_xy, z_from, z_to, K, R, t):
    """同一条竖直轴上 ``z_from → z_to`` 的像素位移（轴向微分）。"""
    a = project_pt([axis_xy[0], axis_xy[1], z_from], K, R, t)
    b = project_pt([axis_xy[0], axis_xy[1], z_to], K, R, t)
    if a is None or b is None:
        return None
    return (b[0] - a[0], b[1] - a[1])


def z_at_v_on_axis(axis_xy, v_target, K, R, t, z_hi, z_lo):
    """在竖直轴上找投影 v = ``v_target`` 的高度（v 随 z 单调，线性插值）。"""
    zs = np.linspace(z_hi, z_lo, 240)
    vs = []
    for z in zs:
        p = project_pt([axis_xy[0], axis_xy[1], z], K, R, t)
        if p is None:
            return None
        vs.append(p[1])
    vs = np.asarray(vs)
    order = np.argsort(vs)
    v_sorted, z_sorted = vs[order], zs[order]
    if not (v_sorted[0] <= v_target <= v_sorted[-1]):
        return None
    return float(np.interp(v_target, v_sorted, z_sorted))


def uv_to_xy_base(du, dv, zc, K, R):
    """像素差 → base 水平位移（``dz=0``）。返回 ``(dx, dy, 条件数)``。

    在块处线性化投影（雅可比只含 ``R``，**不含平移** —— 平移只改变锚点在哪里、
    不改变屏幕上的像素差）。二阶项没算：3 mm 的位移在 1 m 处带来 ~5% 的欠/过
    修正（合成场景实测 3.0 mm 读成 2.86 mm）。对 **闭环** 无害 —— 修正一次剩
    5%，两次剩 0.25%，而 E4b 允许 ≤2 次修正。要求单次读数绝对准的地方别用它。
    """
    fx, fy = float(K[0, 0]), float(K[1, 1])
    # p_cam = R^T (p_base - t) ⇒ δp_cam_x = R[0,0]·dx + R[1,0]·dy
    a11 = fx * R[0, 0] / zc
    a12 = fx * R[1, 0] / zc
    a21 = fy * R[0, 1] / zc
    a22 = fy * R[1, 1] / zc
    M = np.array([[a11, a12], [a21, a22]], float)
    if abs(np.linalg.det(M)) < 1e-12:
        return None
    dx, dy = np.linalg.solve(M, np.array([du, dv], float))
    return float(dx), float(dy), float(np.linalg.cond(M))


def fold_deg(a, period=90.0):
    """折到 (−period/2, +period/2]。"""
    return float((a + period / 2.0) % period - period / 2.0)


def circular_mean_deg(vals, period=90.0):
    """period 周期角的圆均值（返回 (−period/2, period/2]）。"""
    if not vals:
        return None
    k = 2.0 * math.pi / period
    s = np.mean(np.sin(np.asarray(vals) * k))
    c = np.mean(np.cos(np.asarray(vals) * k))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return float(math.atan2(s, c) / k)


# ---------------------------------------------------------------- 图像量化
def _k_from(frame_dir, idx):
    return gc.camera_k(frame_dir, idx=idx)


def _v_channel(color):
    """HSV 的 V（0–255）= max(R,G,B)；不引 cv2。"""
    return color.max(axis=2).astype(np.int16)


def _table_z_m(depth, K, R, t):
    P, pix = gc.backproject(depth, K)
    ok = np.isfinite(P).all(axis=1) & (P[:, 2] > 0)
    Pb = P[ok] @ R.T + t
    if len(Pb) < 200:
        return None
    step = max(1, len(Pb) // 60000)
    return gc.table_plane_z(Pb[::step])


def _runs(prof, thresh, u0, min_len=2, merge_gap=3):
    """列比例序列 → 连续串 ``[(u_start, u_end), ...]``（端点都含）。"""
    runs, st = [], None
    for i, p in enumerate(prof):
        if p > thresh and st is None:
            st = i
        elif p <= thresh and st is not None:
            if i - st >= min_len:
                runs.append([u0 + st, u0 + i - 1])
            st = None
    if st is not None and len(prof) - st >= min_len:
        runs.append([u0 + st, u0 + len(prof) - 1])
    merged = []
    for r in runs:
        if merged and r[0] - merged[-1][1] <= merge_gap:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    return merged


def detect_fingers(color, depth, table_z, K, R, t, u_tip, anchor_v,
                   bands=((-42, -12), (-38, -8), (-30, -6))):
    """在 ``anchor_v`` **上方**的带里找两只深色指 → 中点 ``u``（像素）。

    ``anchor_v`` 取「指尖」与「块顶」两者在上（v 小）的那个的投影 v：
      · 抓前：指尖远高于块顶 ⇒ 带落在块上方，带内只有两根指；
      · 抓后（held）：块顶**高于**指尖 ⇒ 锚点换成块顶，带落在块顶上方，
        仍只有两根指（否则块会填进两指之间的列，把两指连成一片）。
    只用列剖面，不用连通域：B1 实测两指常与夹爪本体的深色连成一片，
    按列取带内比例反而干净（本体在带上方）。
    """
    v = _v_channel(color)
    H, W = v.shape
    ysg, xsg = np.mgrid[0:H, 0:W]
    pc = np.stack([(xsg - K[0, 2]) / K[0, 0] * depth,
                   (ysg - K[1, 2]) / K[1, 1] * depth, depth], -1)
    zb = (pc @ R.T + t)[..., 2]
    dark = (v < FINGER_DARK_V) & ~((depth > 0.1) & (zb < table_z + TABLE_BAND_M))
    u0, u1 = int(round(u_tip - 90)), int(round(u_tip + 90))
    u0, u1 = max(0, u0), min(W - 1, u1)
    cands = []
    for b0, b1 in bands:
        va = int(round(anchor_v + b0))
        vb = int(round(anchor_v + b1))
        if va < 0 or vb <= va or vb > H:
            continue
        band = dark[va:vb, u0:u1 + 1]
        if band.size == 0:
            continue
        prof = band.mean(0)
        runs = _runs(prof, FINGER_RUN_MIN, u0)
        runs = [r for r in runs if (r[1] - r[0]) <= 30]     # 太宽的不是指
        cxs = [(a + b) / 2.0 for a, b in runs]
        best = None
        for i in range(len(cxs) - 1):
            sep = cxs[i + 1] - cxs[i]
            if not (20.0 <= sep <= 125.0):
                continue
            mid = (cxs[i] + cxs[i + 1]) / 2.0
            if abs(mid - u_tip) > 40.0:
                continue
            cost = abs(mid - u_tip) + 0.2 * abs(sep - 55.0)
            if best is None or cost < best[0]:
                best = (cost, mid, sep, runs[i], runs[i + 1], va, vb)
        if best:
            cands.append(best[1:])
    if not cands:
        return None
    if len(cands) >= 2:
        mids = sorted(c[0] for c in cands)
        if mids[-1] - mids[0] <= 3.0:
            picks = [c for c in cands if abs(c[0] - np.median(mids)) <= 1.5]
            best = max(picks, key=lambda c: c[5] - c[4])   # 带最高（离本体最远）
        else:
            best = cands[0]
    else:
        best = cands[0]
    mid, sep, left, right, va, vb = best
    return {"u_mid": float(mid), "sep_px": float(sep),
            "finger_left": list(left), "finger_right": list(right),
            "band_v": [int(va), int(vb)], "n_bands": len(cands)}


# ---------------------------------------------------------------- 块
def _crop_frame(frame, u0, v0, u1, v1):
    """裁 ROI；K 的主点跟着平移（裁+主点平移 = 同一针孔模型）。"""
    K = np.array(frame["K"], float).copy()
    K[0, 2] -= u0
    K[1, 2] -= v0
    return {"dir": frame["dir"],
            "depth": frame["depth"][v0:v1, u0:u1],
            "color": frame["color"][v0:v1, u0:u1],
            "K": K, "R_bc": frame["R_bc"], "t_bc": frame["t_bc"],
            "joints": frame.get("joints")}


def detect_block(frame, table_z, u_prior, v_prior, expect_class=None,
                 radius_px=130.0, min_h_mm=10.0, max_h_mm=70.0,
                 z_expect_mm=None, z_tol_mm=60.0, u_range=None, v_range=None):
    """ROI 里找块，按「类名优先 + 离先验最近」选一块。返回 dict / None。

    注意：顶面点云里会混进**近侧竖壁**靠近顶面那 3 mm 的点（相机斜俯视时壁
    朝相机那面看得见），把 ``uv`` 朝相机方向偏 ~1.5–2 px（≈2 mm）。规划器的
    抓取点也走 ``uv``（``build_truth`` 用「射线 ∩ 已知平面」），所以这个偏置
    是**共模**的 —— 对中判据跟着规划器走，不去它。

    ``max_h_mm``：抓后（held）块被抬到台面上方 140+ mm，必须放大到能容下；
    ``z_expect_mm``：只认顶面高度落在期望 ±``z_tol_mm`` 内的簇 —— 否则夹爪本体
    （同轴、更高的深色块）也会被当成「块」。容差给到 60 mm 是因为俯视深度的
    绝对高度有固定偏置（B1 台面读 -30 mm），这里只做粗筛。
    ``u_range/v_range``：直接给定裁切区（held 用两指之间的窄条 —— 指与块在
    物理上贴在一起，不做裁切的话 find_blocks 会把两指和块连成一簇、把顶面
    定到指顶）。
    """
    H, W = frame["depth"].shape
    if u_range is None:
        u0, u1 = max(0, int(u_prior - 220)), min(W, int(u_prior + 220))
    else:
        u0, u1 = max(0, int(u_range[0])), min(W, int(u_range[1]))
    if v_range is None:
        v0, v1 = max(0, int(v_prior - 160)), min(H, int(v_prior + 340))
    else:
        v0, v1 = max(0, int(v_range[0])), min(H, int(v_range[1]))
    if u1 - u0 < 40 or v1 - v0 < 40:
        return None
    crop = _crop_frame(frame, u0, v0, u1, v1)
    rows, _tz = gc.find_blocks(crop, table_z=table_z, min_h_mm=min_h_mm,
                               max_h_mm=max_h_mm)
    if not rows:
        return None
    scored = []
    for b in rows:
        if b.get("low"):
            continue
        if z_expect_mm is not None and abs(float(b["top"])
                                           - z_expect_mm) > z_tol_mm:
            continue
        bu = float(b["uv"][0]) + u0
        bv = float(b["uv"][1]) + v0
        d = math.hypot(bu - u_prior, bv - v_prior)
        # 给了类名就**只认这一类**：附近别的颜色的块不是目标，宁可报「没找到」，
        # 也不能悄悄换成它（那是 E9 里判分永远发现不了的错抓）。
        if expect_class is not None and b.get("class") != expect_class:
            continue
        if d > radius_px:
            continue
        score = d
        scored.append((score, b, bu, bv))
    if not scored:
        return None
    scored.sort(key=lambda s: s[0])
    _score, b, bu, bv = scored[0]
    out = dict(b)
    out["uv"] = [bu, bv]
    out["class_match"] = bool(expect_class is None
                              or b.get("class") == expect_class)
    return out


# ---------------------------------------------------------------- 抓着块时的块顶面
HELD_HIGH_M = 0.080          # 与 measure_held 同值：立着的块最高 35 mm
HELD_TOP_STRATUM = 0.003     # 与 find_blocks 同一把尺子：顶面取最高 3 mm


def held_block_uv(frame, table_z, expect_class=None):
    """块被夹着时的顶面 → 像素 ``uv`` + 诊断。失败抛 ``RuntimeError``。

    为什么不用 find_blocks：指垫与块在物理上贴在一起，4 mm 网格会把它俩连成
    一簇，``ztop`` 变成指顶的高度。改用 **颜色类 + 台面以上 80 mm** 隔离
    （``measure_held.py`` 2026-10-26 已在真机上用过这套掩码）：指垫是黑的
    （S/V 低），天然不进掩码。

    ``uv`` = 顶面 3D 质心的重投影 —— 与 ``find_blocks`` 同一个定义。它对深度
    尺度不敏感（相机系整体缩放，投影里约掉），所以俯视深度的 z 偏置不影响它，
    只影响 ``top`` 那个数（这里作为诊断报出来）。
    """
    K, R, t = frame["K"], frame["R_bc"], frame["t_bc"]
    P, pix = gc.backproject(frame["depth"], K)
    Pb = P @ R.T + t
    hue, sat, val = mh.rgb_to_hsv_vec(frame["color"])
    hp = (hue[pix[:, 1], pix[:, 0]], sat[pix[:, 1], pix[:, 0]],
          val[pix[:, 1], pix[:, 0]])
    base = (hp[1] >= mh.S_MIN) & (hp[2] >= mh.V_MIN) \
        & (Pb[:, 2] > table_z + HELD_HIGH_M)
    names = list(block_classes.DEFAULT_CLASSES)
    idx, _ = mh.class_of_hue_vec(hp[0], names)
    if base.sum() < 200:
        raise RuntimeError("台面+80 mm 以上几乎没有彩色点（%d）—— 臂真的停在 lift 吗？"
                           % int(base.sum()))
    if expect_class is None:
        w = [hp[1][base & (idx == i)].sum() for i in range(len(names))]
        cls = names[int(np.argmax(w))]
    else:
        if expect_class not in names:
            raise RuntimeError("类别 %r 不在表里（有 %s）" % (expect_class, names))
        cls = expect_class
    sel = base & (idx == names.index(cls))
    if sel.sum() < 200:
        raise RuntimeError("高位点上没有 %s 的点（%d）—— 夹的是这一块吗？"
                           % (cls, int(sel.sum())))
    Q = Pb[sel]
    ztop = float(np.percentile(Q[:, 2], 97))
    F = Q[Q[:, 2] > ztop - HELD_TOP_STRATUM]
    c = F[:, :2].mean(0)
    p = project_pt([c[0], c[1], ztop], K, R, t)
    if p is None:
        raise RuntimeError("块顶质心重投影不可用")
    return {"uv": [p[0], p[1]], "class": cls, "n_points": int(sel.sum()),
            "top_depth_mm": round(ztop * 1000.0, 1),
            "spread_mm": round(float(np.ptp(F[:, :2], axis=0).max()) * 1000.0, 1)}


# ---------------------------------------------------------------- 逐帧
def measure_frame(frame_dir, idx, th, block_top_mm,
                  grasp_yaw_deg=None, expect_class=None, prior_xy_mm=None,
                  held=False, table_z=None, frame=None):
    """一帧 → base 系 ``(dx, dy[, dyaw])``。失败抛 ``RuntimeError(原因)``。

    ``block_top_mm`` **必填**：块顶在 base 系的**已知**高度，不是俯视深度量的。
    俯视深度有固定的 z 偏置（B1 实测台面读 -30 mm），而轴向锚点对高度敏感
    （拾取区 ≈1 px/mm）⇒ 拿带偏置的高度去放锚点，会把偏心算成几毫米。
    真值管线本来就是「像素射线 ∩ 已知平面 z=table_top+n·size」，这里沿用同一口径。

    两种用法共用一条流水线，只换「带锚点」：
      · 抓前：锚点 = min(指尖投影 v, 轴在块顶的投影 v) —— 指尖远高，取指尖；
      · 抓后（``held``）：块比指尖高 ⇒ 取块顶，带落在块顶上方，带里只剩两根指。
    """
    if frame is None:
        frame = gc.load_frame(frame_dir, tip=gc.TOPCAM_TIP,
                              chain=gc.TOPCAM_CHAIN, idx=idx)
    K, R, t = frame["K"], frame["R_bc"], frame["t_bc"]
    js = frame.get("joints")
    if not js or "position" not in js or "name" not in js:
        raise RuntimeError("帧 %s 没有可用的 joint_states" % idx)
    try:
        q = [float(js["position"][js["name"].index(n)]) for n in th.move_joints]
    except (KeyError, ValueError):
        raise RuntimeError("帧 %s 的 joint_states 缺关节名" % idx)
    T = th.fk(q)
    tool0 = T[:3, 3]
    tip_pt = tool0 + T[:3, :3] @ np.array([0.0, 0.0, TOOL0_TO_BARE_FINGERTIP])
    p_tip = project_pt(tip_pt, K, R, t)
    if p_tip is None:
        raise RuntimeError("指尖投影落在相机后方")
    u_tip, v_tip = p_tip[0], p_tip[1]
    H = frame["depth"].shape[0]
    if not (0 <= v_tip <= H):
        # 指尖不在画幅内（悬停太高 / 相机被碰偏）⇒ 这一帧没有夹爪可量
        raise RuntimeError("指尖投影 v=%.0f 不在画幅内" % v_tip)

    if table_z is None:
        table_z = _table_z_m(frame["depth"], K, R, t) or 0.0
    axis_xy = (tool0[0], tool0[1])
    z_block = float(block_top_mm) / 1000.0
    p_axis_top = project_pt([axis_xy[0], axis_xy[1], z_block], K, R, t)
    if p_axis_top is None:
        raise RuntimeError("轴在块顶高度的投影不可用")

    # 带锚点：指尖与块顶里更靠上（v 更小）的那个
    anchor_v = min(v_tip, p_axis_top[1])
    fingers = detect_fingers(frame["color"], frame["depth"], table_z,
                             K, R, t, u_tip, anchor_v)
    if fingers is None:
        raise RuntimeError("分不出两指（深色列串不足）")

    # 选块。抓前：外参/先验（腕部真值或工具轴）在块顶平面上的投影；
    # 抓后：块被夹在指间，改用颜色类掩码（见 held_block_uv 的说明）。
    held_info = None
    if held:
        held_info = held_block_uv(frame, table_z, expect_class=expect_class)
        blk = {"uv": held_info["uv"], "class": held_info["class"],
               "yaw_trusted": False, "rect_fill": None, "yaw_rect": None,
               "cx": held_info["uv"][0], "cy": held_info["uv"][1],
               "top": held_info["top_depth_mm"], "class_match": None}
    else:
        prior_pt = ((prior_xy_mm[0] / 1000.0, prior_xy_mm[1] / 1000.0)
                    if prior_xy_mm else (tool0[0], tool0[1]))
        p_prior = project_pt([prior_pt[0], prior_pt[1], z_block], K, R, t)
        if p_prior is None:
            raise RuntimeError("块先验投影不可用")
        max_h_mm = (z_block - table_z) * 1000.0 + 60.0
        blk = detect_block(frame, table_z, p_prior[0], p_prior[1],
                           expect_class=expect_class,
                           min_h_mm=10.0, max_h_mm=max(80.0, max_h_mm),
                           z_expect_mm=z_block * 1000.0, z_tol_mm=60.0)
        if blk is None:
            raise RuntimeError("ROI 里没检出块")

    # 轴在块顶高度处的像素点 = 实测中点 + 轴向微分（带 → 块顶）
    band_v = 0.5 * (fingers["band_v"][0] + fingers["band_v"][1])
    z_band = z_at_v_on_axis(axis_xy, band_v, K, R, t,
                            tool0[2], tool0[2] - 0.45)
    if z_band is None:
        raise RuntimeError("轴上找 v=%.0f 的高度失败" % band_v)
    shift = axis_shift_uv(axis_xy, z_band, z_block, K, R, t)
    if shift is None:
        raise RuntimeError("轴向微分不可用")
    axis_u = fingers["u_mid"] + shift[0]
    axis_v = band_v + shift[1]

    du = float(blk["uv"][0]) - axis_u
    dv = float(blk["uv"][1]) - axis_v
    # 像素差 → base 位移需要块处的**相机深度** zc。抓前用块的三维质心；抓后块
    # 就在轴上（只差 e），所以取轴在块顶高度的那个点 —— 差不到 3 mm，对 zc
    # 的影响可忽略（zc 只进分母的除法）。
    p_blk = p_axis_top if held else project_pt(
        [float(blk["cx"]) / 1000.0, float(blk["cy"]) / 1000.0,
         z_block], K, R, t)
    if p_blk is None:
        raise RuntimeError("块的相机深度不可用")
    conv = uv_to_xy_base(du, dv, p_blk[2], K, R)
    if conv is None:
        raise RuntimeError("像素差 → base 位移退化（相机轴与台面近平行？）")
    dx, dy, cond = conv
    dyaw = None
    if grasp_yaw_deg is not None and blk.get("yaw_trusted"):
        dyaw = fold_deg(float(blk["yaw_rect"]) - float(grasp_yaw_deg))
    return {"idx": idx,
            "u_mid_px": round(fingers["u_mid"], 2),
            "finger_sep_px": round(fingers["sep_px"], 2),
            "band_v": fingers["band_v"], "n_bands": fingers["n_bands"],
            "anchor_v": round(anchor_v, 1),
            "z_band_mm": round(z_band * 1000.0, 1),
            "axis_uv": [round(axis_u, 2), round(axis_v, 2)],
            "block_uv": [round(float(blk["uv"][0]), 2),
                         round(float(blk["uv"][1]), 2)],
            "block_class": blk.get("class"),
            "class_match": blk.get("class_match"),
            "block_top_depth_mm": round(float(blk["top"]), 1),
            "block_xy_depth_mm": (None if held else
                                  [round(float(blk["cx"]), 1),
                                   round(float(blk["cy"]), 1)]),
            "block_yaw_rect": (round(float(blk["yaw_rect"]), 2)
                               if blk.get("yaw_rect") is not None else None),
            "yaw_trusted": bool(blk.get("yaw_trusted")),
            "rect_fill": blk.get("rect_fill"),
            "held_n_points": (held_info["n_points"] if held_info else None),
            "held_spread_mm": (held_info["spread_mm"] if held_info else None),
            "du_px": round(du, 2), "dv_px": round(dv, 2),
            "dx_mm": round(dx * 1000.0, 2), "dy_mm": round(dy * 1000.0, 2),
            "dyaw_deg": None if dyaw is None else round(dyaw, 3),
            "depth_block_m": round(p_blk[2], 3),
            "cond": round(cond, 2)}


def consensus(records, held=False):
    """多帧共识 + 质量指标。``held`` 时输出偏心 ``e``。"""
    ok = [r for r in records if r.get("dx_mm") is not None]
    if not ok:
        return {"n_used": 0}
    dxs = np.array([r["dx_mm"] for r in ok])
    dys = np.array([r["dy_mm"] for r in ok])
    dx = float(np.median(dxs))
    dy = float(np.median(dys))
    keep = [r for r in ok
            if math.hypot(r["dx_mm"] - dx, r["dy_mm"] - dy) <= 3.0]
    if len(keep) >= MIN_FRAMES:
        dx = float(np.median([r["dx_mm"] for r in keep]))
        dy = float(np.median([r["dy_mm"] for r in keep]))
    yaws = [r["dyaw_deg"] for r in ok if r.get("dyaw_deg") is not None]
    dyaw = circular_mean_deg(yaws)
    if dyaw is not None and len(yaws) >= MIN_FRAMES:
        yaw_keep = [y for y in yaws if abs(fold_deg(y - dyaw)) <= 5.0]
        if len(yaw_keep) >= MIN_FRAMES:
            yaws = yaw_keep
            dyaw = circular_mean_deg(yaws)
    umids = [r["u_mid_px"] for r in keep]
    cons = {"n_used": len(keep), "dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
            "spread_mm": round(float(max(
                math.hypot(r["dx_mm"] - dx, r["dy_mm"] - dy) for r in keep)), 2),
            "spread_u_px": round(max(umids) - min(umids), 2)}
    if yaws:
        cons["dyaw_deg"] = round(dyaw, 3)
        cons["n_dyaw"] = len(yaws)
        cons["spread_deg"] = round(float(max(abs(fold_deg(
            y - dyaw)) for y in yaws)), 3)
    cons["e_mm"] = round(math.hypot(dx, dy), 2)
    checks = {"within_2mm": bool(math.hypot(dx, dy) <= TOL_MM)}
    if yaws:
        checks["within_2deg"] = bool(abs(dyaw) <= TOL_DEG)
    # 「稳」不能只看保留下来的内点（那等于自证）：还要求绝大多数帧都是内点。
    # 3 帧里有 1 帧野值时比例 0.67 < 0.8 ⇒ 不稳，不许报收敛。
    checks["stable"] = bool(cons["spread_mm"] <= 2.0
                            and len(keep) >= 0.8 * len(ok))
    cons["n_outlier"] = len(ok) - len(keep)
    cons["checks"] = checks
    cons["converged"] = bool(all(checks.values()))
    return cons


# ---------------------------------------------------------------- 驱动
def run_burst(burst, urdf, block_top_mm, grasp_yaw_deg=None,
              expect_class=None, prior_xy_mm=None, held=False,
              drop_first=DEFAULT_DROP, max_frames=DEFAULT_MAX,
              table_z_mm=None):
    """整组 burst → 结果 dict（不抛：失败写 ``ok=false`` + ``reason``）。"""
    out = {"burst": os.path.abspath(burst), "held": bool(held),
           "block_top_mm": block_top_mm,
           "ok": False, "reason": None, "frames": []}
    if not os.path.isdir(burst):
        out["reason"] = "burst 目录不存在"
        return out
    th = urkin.Chain(urdf)
    idxs = gc.list_frame_indices(burst)
    if len(idxs) < MIN_FRAMES:
        out["reason"] = "只有 %d 帧（下限 %d）" % (len(idxs), MIN_FRAMES)
        return out
    use = idxs[drop_first:drop_first + max_frames]
    out["dropped_first"] = int(min(drop_first, len(idxs)))
    out["frames_considered"] = use
    table_z = None if table_z_mm is None else table_z_mm / 1000.0
    if table_z is None:
        f0 = gc.load_frame(burst, tip=gc.TOPCAM_TIP, chain=gc.TOPCAM_CHAIN,
                           idx=use[0])
        table_z = _table_z_m(f0["depth"], f0["K"], f0["R_bc"], f0["t_bc"])
    out["table_z_mm"] = None if table_z is None else round(table_z * 1000, 1)
    errs = []
    for i in use:
        try:
            rec = measure_frame(burst, i, th,
                                grasp_yaw_deg=grasp_yaw_deg,
                                expect_class=expect_class,
                                prior_xy_mm=prior_xy_mm, held=held,
                                block_top_mm=block_top_mm, table_z=table_z)
            out["frames"].append(rec)
        except Exception as e:                       # noqa: BLE001
            errs.append({"idx": i, "error": str(e)})
    out["errors"] = errs
    if len(out["frames"]) < MIN_FRAMES:
        out["reason"] = ("有效帧 %d < %d；逐帧错误：%s"
                         % (len(out["frames"]), MIN_FRAMES,
                            "; ".join("f%02d %s" % (e["idx"], e["error"])
                                      for e in errs[:4])))
        return out
    out["consensus"] = consensus(out["frames"], held=held)
    out["ok"] = out["consensus"]["n_used"] >= MIN_FRAMES
    if not out["ok"]:
        out["reason"] = "共识帧数 %d < %d" % (out["consensus"]["n_used"],
                                             MIN_FRAMES)
    return out


def _fmt(res):
    if not res["ok"]:
        return "!! 量不出来：%s" % (res.get("reason") or "?")
    c = res["consensus"]
    s = ("%-6s n=%d/%d  dx=%+6.2f dy=%+6.2f mm  |e|=%.2f mm  极差 %.2f mm"
         % ("held" if res["held"] else "hover", c["n_used"],
            len(res.get("frames_considered") or []), c["dx_mm"], c["dy_mm"],
            c["e_mm"],
            c["spread_mm"]))
    if "dyaw_deg" in c:
        s += "  dyaw=%+5.2f° (n=%d, 极差 %.2f°)" % (c["dyaw_deg"],
                                                  c["n_dyaw"],
                                                  c["spread_deg"])
    chk = c["checks"]
    s += "  " + ("收敛" if c.get("converged") else "**未收敛**")
    return s


# ---------------------------------------------------------------- E4b 汇总
#: E4b 的验收判据（入口实验 E4b 行）：收敛场景比例 ≥90%、抓后偏心 e 的中位
#: ≤2 mm、p90 ≤3 mm、每场修正 ≤2 次。数值改了 = 改判据，要同步文档。
E4B_CONV_RATE = 0.90
E4B_E_MEDIAN_MM = 2.0
E4B_E_P90_MM = 3.0
E4B_MAX_ROUNDS = 2


def _load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def summarise(spec):
    """多场 ``stage4_align.py`` 结果 → E4b 判定。

    ``spec`` 是场景列表，每项 ``{"scene": 名, "pre": [r1.json, r2.json],
    "held": held.json}``：``pre`` 是**同一次抓取**的逐轮对中结果（1–2 份，
    顺序即轮次），``held`` 是抓后偏心。

    每场「收敛」= 最后一轮 ``ok`` ∧ ``consensus.converged``（残差 ≤2 mm /
    ≤2° 且帧间稳）∧ 轮数 ≤2。抓后偏心 ``e`` 取自 ``held`` 的共识；没有
    ``held`` 的场景不计入 e 统计，但也**不算**E4b 通过（判据要两半）。
    """
    scenes, bad = [], []
    for sc in spec:
        name = sc.get("scene") or "?"
        pres = [_load_json(p) for p in (sc.get("pre") or [])]
        held = _load_json(sc["held"]) if sc.get("held") else None
        rounds = len(pres)
        last = pres[-1] if pres else None
        cons = (last or {}).get("consensus") or {}
        conv = bool(last and last.get("ok") and cons.get("converged")
                    and rounds <= E4B_MAX_ROUNDS)
        if not pres:
            bad.append("%s: 没有对中结果" % name)
        elif rounds > E4B_MAX_ROUNDS:
            bad.append("%s: 修正了 %d 轮（上限 %d）—— 没收敛"
                       % (name, rounds, E4B_MAX_ROUNDS))
        hc = (held or {}).get("consensus") or {}
        e_mm = (float(hc["e_mm"]) if held and held.get("ok")
                and hc.get("e_mm") is not None else None)
        if e_mm is None:
            bad.append("%s: 没有可用的抓后偏心 e" % name)
        scenes.append({
            "scene": name,
            "n_rounds": rounds,
            "converged": conv,
            "dx_mm": cons.get("dx_mm"), "dy_mm": cons.get("dy_mm"),
            "dyaw_deg": cons.get("dyaw_deg"),
            "spread_mm": cons.get("spread_mm"),
            "n_used": cons.get("n_used"),
            "e_mm": None if e_mm is None else round(e_mm, 2),
        })

    n = len(scenes)
    conv_rate = (sum(1 for s in scenes if s["converged"]) / float(n)
                 if n else 0.0)
    es = np.array([s["e_mm"] for s in scenes if s["e_mm"] is not None],
                  dtype=float)
    e_med = float(np.median(es)) if len(es) else None
    e_p90 = float(np.percentile(es, 90)) if len(es) else None
    agg = {"n_scenes": n, "conv_rate": round(conv_rate, 3),
           "n_e": int(len(es)),
           "e_median_mm": None if e_med is None else round(e_med, 2),
           "e_p90_mm": None if e_p90 is None else round(e_p90, 2)}
    reasons = []
    if not n:
        reasons.append("没有场景")
    if conv_rate < E4B_CONV_RATE:
        reasons.append("收敛比例 %.0f%% < %.0f%%"
                       % (conv_rate * 100, E4B_CONV_RATE * 100))
    if e_med is None:
        reasons.append("没有抓后偏心数据")
    else:
        if e_med > E4B_E_MEDIAN_MM:
            reasons.append("e 中位 %.2f mm > %.1f mm"
                           % (e_med, E4B_E_MEDIAN_MM))
        if e_p90 > E4B_E_P90_MM:
            reasons.append("e p90 %.2f mm > %.1f mm"
                           % (e_p90, E4B_E_P90_MM))
    return {"ok": not reasons, "scenes": scenes, "aggregate": agg,
            "reasons": reasons, "scene_problems": bad,
            "thresholds": {"conv_rate": E4B_CONV_RATE,
                           "e_median_mm": E4B_E_MEDIAN_MM,
                           "e_p90_mm": E4B_E_P90_MM,
                           "max_rounds": E4B_MAX_ROUNDS}}


def _fmt_summary(res):
    lines = []
    for s in res["scenes"]:
        lines.append(
            "%-10s 轮 %d  %s  dx=%s dy=%s  e=%s mm"
            % (s["scene"], s["n_rounds"],
               "收敛" if s["converged"] else "**未收敛**",
               s["dx_mm"], s["dy_mm"],
               "—" if s["e_mm"] is None else "%.2f" % s["e_mm"]))
    a = res["aggregate"]
    lines.append("总：%d 场，收敛 %.0f%%；e 中位 %s mm、p90 %s mm"
                 % (a["n_scenes"], a["conv_rate"] * 100,
                    a["e_median_mm"], a["e_p90_mm"]))
    lines.append("E4b %s%s" % ("通过" if res["ok"] else "**未通过**",
                              "" if res["ok"] else
                              "：" + "；".join(res["reasons"])))
    for b in res["scene_problems"]:
        lines.append("   · %s" % b)
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--burst", default=None, help="俯视 burst 目录")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--summary", default=None, metavar="SPEC.json",
                    help="E4b 多场汇总：场景列表 [{\"scene\",\"pre\":[..],"
                         "\"held\":..}]；给了它就不量 burst")
    ap.add_argument("--out-summary", default=None,
                    help="汇总结果写 JSON（缺省只打印）")
    ap.add_argument("--out", default=None, help="结果 JSON（缺省只打印）")
    ap.add_argument("--expect-class", default=None,
                    help="目标块类名（red/orange/tan/yellow/yellowgreen/cyan/purple）")
    ap.add_argument("--block-prior-mm", default=None, metavar="X,Y",
                    help="跑前：块心先验（腕部真值，mm）。只用于选块")
    ap.add_argument("--grasp-yaw-deg", type=float, default=None,
                    help="跑前：计划抓取角（dyaw = 块角 − 它）")
    ap.add_argument("--held", action="store_true",
                    help="跑后偏心模式：块夹在两指之间（在 lift 航点拍）")
    ap.add_argument("--block-top-mm", type=float, default=None,
                    help="目标块顶离 base 的绝对高度（mm，按真值/计划给，"
                         "不要用俯视深度量的）")
    ap.add_argument("--drop-first", type=int, default=DEFAULT_DROP)
    ap.add_argument("--max-frames", type=int, default=DEFAULT_MAX)
    ap.add_argument("--table-z-mm", type=float, default=None,
                    help="台面高度（缺省从深度直方图取）")
    args = ap.parse_args(argv)

    if args.summary:
        if args.burst or args.urdf:
            ap.error("--summary 不量 burst，别同时给 --burst/--urdf")
        res = summarise(_load_json(args.summary))
        print(_fmt_summary(res))
        if args.out_summary:
            d = os.path.dirname(os.path.abspath(args.out_summary))
            os.makedirs(d or ".", exist_ok=True)
            with open(args.out_summary, "w", encoding="utf-8") as f:
                json.dump(res, f, ensure_ascii=False, indent=1)
            print("→ %s" % args.out_summary)
        return 0 if res["ok"] else 2
    if not args.burst or not args.urdf:
        ap.error("量 burst 要给 --burst 与 --urdf（或改用 --summary）")

    prior = None
    if args.block_prior_mm:
        parts = [float(v) for v in args.block_prior_mm.split(",")]
        if len(parts) != 2:
            ap.error("--block-prior-mm 要 X,Y 两个数")
        prior = parts
    if args.held and args.block_prior_mm:
        ap.error("--held 不用 --block-prior-mm（块就在轴上）")

    if args.block_top_mm is None:
        ap.error("量 burst 要给 --block-top-mm")
    res = run_burst(args.burst, args.urdf, args.block_top_mm,
                    grasp_yaw_deg=args.grasp_yaw_deg,
                    expect_class=args.expect_class,
                    prior_xy_mm=prior, held=args.held,
                    drop_first=args.drop_first, max_frames=args.max_frames,
                    table_z_mm=args.table_z_mm)
    print(_fmt(res))
    for e in res.get("errors", [])[:6]:
        print("   f%02d 跳过：%s" % (e["idx"], e["error"]))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".",
                    exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        print("→ %s" % args.out)
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
