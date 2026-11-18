#!/usr/bin/env python3
"""把一次 burst（俯视或**腕部**）量成 E8/E9 探针能吃的 ``scene_truth.json``。

两路相机只用 ``--camera`` 区分（2026-11-11 v0.6 起俯视退出阶段 4）：

    ``--camera wrist``    规划感知 + 跑后三机位判分（**默认**）：手眼外参定位，毫米级
    ``--camera topcam``   已退出阶段 4；只用于复现旧证据，CLI 必须再加 ``--allow-topcam``

两者**只差 tf 链与内参** —— 下面的几何（射线 ∩ 平面、分层、颜色、yaw）一层不动。

为什么需要这个工具
------------------
``/targets_3d`` 只有位置 —— ``bbox3d.size`` 全 0、朝向单位阵，喂不了 E8 探针
（探针要 ``id/x/y/z/size/yaw/top``，base 系、米）。块的分割/颜色/朝向来自 E5
感知链（``graspnet_candidates.find_blocks``，用户 2026-11-08 拍板的真值来源），
但**位置不能直接用它的深度反投影**：

    2026-11-08 用垫 PnP（纯彩色、不经过深度）当场量到俯视深度**系统性读远
    3.0–3.3%**（1.5 m 处 ≈ 46 mm，三条 burst 一致）。直接反投影会把台面算到
    −30 mm、块位整体偏出 20–30 mm。

所以位置走**射线 ∩ 平面**（与生产管线 ``projector_node`` 的
``use_target_plane`` 同一套几何，那里写明的精度是「标定水平」而不是深度水平）：

    uv（find_blocks 的顶面中心像素，深度尺度不改变它）
      → 射线 ∩ 平面 z = table_top + 块高
      → x, y

    top = table_top + 块高        size = --block-size
    z   = top − size / 2

**叠放（塔）**：``find_blocks`` 一根塔只给一个连通域（每层的侧面落在同一批格子
上），``h`` 是塔高。``layer_count`` 按 ``h/size`` 定层数，顶层用实测顶面算
``z = table_top + n·size``，下层按 coaxial 推定 —— 细节见 ``build_truth`` 的
docstring。E9 真机半段必须开着这一条（``--no-layers`` 是旧行为的逃生口）。
    yaw = 彩色像素掩码矩形（``stage2_geom.block_rect_yaw``）；掩码不 trusted
          时才回退到 find_blocks 的顶面 PCA 长边

    为什么不再用 PCA：PCA 的「长边」是顶面那十几个深度点的主轴，近正方形块
    两个特征值几乎相等 ⇒ 主轴没有定义。2026-11-09 E9 真机实测四场目标塔，
    PCA yaw mod 90 = 88.5 / 12.1 / 39.0 / 56.9°，彩色掩码 = 2.6 / 2.5 / 3.9 / 4.4°
    （与亮度选顶面、人工覆盖图三方一致，真实 ≈0–2°）。s3/s4 按错 yaw 合拢，
    手指夹在**两个相邻面**上把块夹倒滑落（用户当场看出来的）。像素掩码在
    1280×720 上每毫米约 2.5 像素，角度不靠稀疏点，是阶段 2 A3 验证过的口径
    （7/7 组 ≤6°）。yaw 走哪一路逐行记在 ``yaw_source``，共识里也标出来。

``table_top`` 默认 0.0 —— 生产栈在真机上用的就是 ``target_plane_z = 台面 + 块高``
（runbook 里 28.5 mm），即台面视为 base z 0。示教器碰面读数 +3.88 mm、垫 PnP
的垫顶 ≈ +1.3 mm 都落在这个值的 ±5 mm 内；要更准就传 ``--table-top``。

拾取区按**射线 ∩ 平面后的 x,y** 判（``ZONE_M``，E5 同一常量）—— 不用深度口径，
否则深度偏差会把边界附近的块判错。区外的检测当幻影丢掉并记 ``dropped_out_of_zone``
（它们若进规划场景会变成假障碍，把 E8 的 M1 冤枉成碰撞）。``class == "unknown"``
（find_blocks 的「整簇没有饱和像素」）同理丢掉并记 ``dropped_not_block`` ——
真机那批积木是 7 个彩色类，灰白表面不是块。

深度仍然要，但只用来：(a) 找块/切顶面/判 yaw，(b) 把实测高度 ``h`` 记进
``meas`` 供人核对（读远 ⇒ h 也偏大，别拿它当尺子）。

用法：

    python3 scripts/stage4_truth_from_frame.py \\
        --frame runs/<run>/e8/topcam-e8 --out runs/<run>/e8/scene_truth_real.json

    # 想先看这一 burst 稳不稳（每帧各量一遍，只打印，不写真值）
    python3 scripts/stage4_truth_from_frame.py --frame <dir> --survey

退出码：0 = 真值写出；2 = 输入不足/量不出块（缺 tf、缺 camera_info、0 块）。
"""
import argparse
import glob
import json
import math
import os
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 每进程只许一个 BLAS/OpenMP 线程（必须在 import numpy 之前）。
# 2026-11-12 板上实测：6 个 worker 进程各自默认开多线程 ⇒ 36 线程抢 6 核，
# 同一批 6 帧从 3.6 s（线程=1）恶化到 22.4 s。这里的运算是逐元素数组操作 +
# 小矩阵乘，多线程没有收益；``setdefault`` 不覆盖调用方显式设置的值。
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage4_specify as sp                                       # noqa: E402

BLOCK_SIZE_M = 0.0295
MAX_LAYERS = 5
LAYER_TOL = 0.4          # |h − n·size| / size 的容忍；超了不认成塔


def layer_count(h_m, size, tol=LAYER_TOL, max_layers=MAX_LAYERS):
    """一个顶面簇的实测高度 → ``(层数 n, 说明)``；单块给 ``(1, None)``。

    为什么层数只能从高度反推：``find_blocks`` 的顶面掩码取「每格**最高**点」，
    而塔的每一层侧面都落在同一批格子上 ⇒ 整根塔连成**一个**连通域，``ext`` 仍是
    一块的边长，``h`` 才是塔高。2026-11-08 用 11-02 三阶塔的真机腕部帧核实过：
    二层塔给出 ``h=62.9 mm / ext 29.8×31.9``，**塔底一个点都不可见**（同帧
    ``max_h_mm=33`` 的带扫只剩台面噪声）。

    所以塔的各层位置是**推出来的**（见 ``build_truth`` 的 coaxial 注释），不是量
    出来的。残差超过 ``tol*size`` 就不认（宁可当单块留 warn，也不凭空造块）。
    """
    h_mm = float(h_m) * 1000.0
    s_mm = float(size) * 1000.0
    if h_mm < 1.5 * s_mm:
        return 1, None
    n = int(round(h_mm / s_mm))
    if n < 2:
        return 1, None
    if n > max_layers:
        return 1, "h=%.1f mm 超过 %d 层上限，按单块处理" % (h_mm, max_layers)
    if abs(h_mm - n * s_mm) > tol * s_mm:
        return 1, "h=%.1f mm 不像 %d 层（%.1f mm/层），按单块处理" % (
            h_mm, n, s_mm)
    return n, None


def ray_plane_xy(uv, K, R_bc, t_bc, plane_z):
    """像素 → 射线 ∩ 平面 ``z = plane_z``（base 系）。返回 ``(x, y)`` 或 None。

    射线方向只用像素与内参（与 ``projector_node._ray_plane_pose`` 同义：
    借深度反投影点表达方向 ``(x/z, y/z, 1)``，但位置不经过深度）。
    平面在相机背后 / 射线与平面平行时返回 None。
    """
    u, v = float(uv[0]), float(uv[1])
    d_cam = np.array([(u - K[0, 2]) / K[0, 0],
                      (v - K[1, 2]) / K[1, 1], 1.0])
    d = np.asarray(R_bc, dtype=float) @ d_cam
    if abs(d[2]) < 1e-9:
        return None
    lam = (float(plane_z) - float(t_bc[2])) / d[2]
    if lam <= 0.0:
        return None
    p = np.asarray(t_bc, dtype=float) + lam * d
    return float(p[0]), float(p[1])


def in_zone_xy(x, y, zone_m=None):
    """base 系 (x, y) 是否在拾取区（米）。``zone_m`` 缺省用 ``sp.ZONE_M``。"""
    x0, x1, y0, y1 = (sp.ZONE_M if zone_m is None else zone_m)
    return x0 <= x <= x1 and y0 <= y <= y1


def colour_yaw_rows(frame, rows):
    """把每行的 ``yaw`` 换成彩色像素掩码矩形（``stage2_geom.block_rect_yaw``）。

    - trusted（轮廓填充率 ≥0.70 且像素 ≥200）⇒ 覆盖 ``yaw``，
      ``yaw_source='colour_rect'``，原 PCA 值留在 ``yaw_pca``；
    - 不 trusted 或没有 cv2 ⇒ 保留 PCA，``yaw_source='depth_pca'``，
      原因记 ``yaw_note``。**不失败**：掩码坏掉不该让整场真值变空，但
      ``yaw_source`` 会让报告和共识看得见这一行没走彩色。

    返回 ``(n_colour, n_rows)``，供调用方打印。
    """
    try:
        import stage2_geom as sg
    except Exception as e:                                    # noqa: BLE001
        for r in rows:
            r["yaw_source"] = "depth_pca"
            r["yaw_note"] = "stage2_geom 不可用：%s" % e
        return 0, sum(1 for r in rows if not r.get("low"))
    n_ok = n = 0
    for r in rows:
        if r.get("low"):
            continue
        n += 1
        r["yaw_pca"] = float(r.get("yaw", 0.0))
        try:
            rr = sg.block_rect_yaw(frame, r)
        except Exception as e:                                # noqa: BLE001
            rr = {"yaw_deg": None, "trusted": False, "reason": str(e),
                  "ext_mm": [0.0, 0.0], "fill": 0.0, "n_pixels": 0}
        r["yaw_rect"] = {"ext_mm": rr.get("ext_mm"),
                         "fill": round(float(rr.get("fill") or 0.0), 3),
                         "n_pixels": int(rr.get("n_pixels") or 0)}
        if rr.get("yaw_deg") is not None and rr.get("trusted"):
            r["yaw"] = float(rr["yaw_deg"])
            r["yaw_source"] = "colour_rect"
            n_ok += 1
        else:
            r["yaw_source"] = "depth_pca"
            r["yaw_note"] = rr.get("reason") or "掩码不 trusted"
    return n_ok, n


def build_truth(rows, cam, table_top=0.0, block_size=BLOCK_SIZE_M,
                batch="REAL", index=0, keep_outside=False,
                drop_unknown=True, min_class_share=0.0, layers=True,
                zone_m=None):
    """``find_blocks`` 的行 → ``scene_truth.json`` 的形状（射线 ∩ 平面）。

    ``cam`` 给 ``{'K','R_bc','t_bc'}``（``graspnet_candidates.load_frame`` 的
    字段）。``low=True`` 的行（5 mm…min_h 的诊断簇）不是块，丢掉；
    ``mixed_pair_unsplit`` 的行（没拆开且票面双类 ≥25% —— 并成一簇的贴合对，
    位置落在两块中间）同样丢掉：它进共识会把「整簇心 + 两块心」并成一个
    Chimera（S1:4 实测，台账第三百三十九轮），宁可这帧少一个观测也不污染
    共识。丢失数记 ``dropped_unsplit_pair``。

    **叠放（E9 的塔）**：``layers=True`` 时，一个簇的实测 ``h`` 若是层高的整数倍
    （见 ``layer_count``），就把它展开成 n 块。位置口径：

    - 顶层 = **实测**：``uv`` 射线 ∩ 平面 ``z = table_top + n·size``（顶层顶面）；
    - 下面各层 = **推定 coaxial**：xy/yaw 沿用顶层，``z`` 每层差一个 ``size``。

    为什么下层只能推定：塔的每层侧面落在同一批格子上 ⇒ ``find_blocks`` 只给一个
    连通域，塔底被顶层整块遮住、一个点都不剩（``layer_count`` 的注释里有实测）。
    物理上顶层必须压在塔底范围内才站得住，xy 差 ≤ 半块（14.75 mm）；A3 开口
    95 mm 对 29.5 mm 的块有 32 mm 余量。每块都记 ``meas.from``，人看得出哪块是
    量的、哪块是推的。

    实测 ``h`` 与 ``n·size`` 差超过 6 mm 时**不失败**，写 ``warn_h_mm``。
    """
    blocks = []
    n_out = 0
    n_unknown = 0
    n_unsplit = 0
    for r in rows:
        if r.get("low"):
            continue
        # ``class == "unknown"`` 是 find_blocks 的「整簇没有饱和像素」——
        # 灰/白/木色表面，定义上不是那 7 色积木。留着会变成假障碍。
        if drop_unknown and r.get("class") == "unknown":
            n_unknown += 1
            continue
        if float(r.get("class_share") or 0.0) < float(min_class_share):
            n_unknown += 1
            continue
        if r.get("mixed_pair_unsplit"):
            n_unsplit += 1
            continue
        size = float(block_size)
        h_mm = float(r.get("h", 0.0))
        n_layers, layer_note = (layer_count(h_mm / 1000.0, size)
                                if layers else (1, None))
        # 顶层顶面 = table_top + n·size；射线 ∩ 的就是这个平面
        plane_z = float(table_top) + n_layers * size
        xy = ray_plane_xy(r["uv"], cam["K"], cam["R_bc"], cam["t_bc"],
                          plane_z)
        if xy is None:
            continue                      # 射线打不到平面上：不是块
        if not keep_outside and not in_zone_xy(xy[0], xy[1], zone_m):
            n_out += 1
            continue
        yaw = float(r.get("yaw", 0.0))
        for k in range(1, n_layers + 1):          # k=1 塔底 … n 顶层
            top = float(table_top) + k * size
            entry = {
                "id": int(r.get("idx", len(blocks))),
                "color": r.get("class", "unknown"),
                "layer": k - 1,
                "size": size,
                "top": top,
                "x": xy[0],
                "y": xy[1],
                "z": top - size / 2.0,
                "yaw": yaw,
                # 只为追溯，不参与碰撞几何
                "meas": {},
            }
            if k == n_layers:                     # 顶层：实测的那个
                entry["meas"] = {
                    "from": "measured_top_face",
                    "h_mm": round(h_mm, 2),
                    "uv": r.get("uv"),
                    "depth_xy_mm": [round(float(r["cx"]), 1),
                                    round(float(r["cy"]), 1)],
                    "depth_top_mm": round(float(r.get("top", 0.0)), 2),
                    "ext_mm": [round(float(v), 2) for v in r.get("ext", [])],
                    "ext_rect_mm": [round(float(v), 2)
                                    for v in r.get("ext_rect", [])],
                    "class_share": r.get("class_share"),
                    "yaw_source": r.get("yaw_source", "depth_pca"),
                    "yaw_pca": round(float(r.get("yaw_pca",
                                                 yaw)), 2),
                    "yaw_rect": r.get("yaw_rect"),
                }
                if r.get("yaw_note"):
                    entry["meas"]["yaw_note"] = r["yaw_note"]
                # 图像空间块心（只记录；2026-11-18 块心精度，见
                # sam_blocks.attach_image_centers）。规划中心仍是 x/y。
                if r.get("img_ok") is not None or r.get("img_why"):
                    entry["meas"]["img_ok"] = bool(r.get("img_ok"))
                    entry["meas"]["img_why"] = r.get("img_why")
                    if r.get("img_ok"):
                        entry["meas"]["img_xy_mm"] = [
                            round(float(r["img_cx"]), 2),
                            round(float(r["img_cy"]), 2)]
                        entry["meas"]["img_shift_mm"] = r.get("img_shift_mm")
                        entry["meas"]["img_support"] = r.get("img_support")
                if abs(h_mm - n_layers * size * 1000.0) > 6.0:
                    entry["warn_h_mm"] = round(
                        h_mm - n_layers * size * 1000.0, 2)
            else:                                 # 下层：coaxial 推定
                entry["meas"] = {
                    "from": "coaxial_with_top_face",
                    "tower_layers": n_layers,
                    "top_face_h_mm": round(h_mm, 2),
                    # 下层 yaw 是推定的（沿用顶层），来源跟着顶层走
                    "yaw_source": r.get("yaw_source", "depth_pca"),
                }
            if layer_note:
                entry["meas"]["layer_note"] = layer_note
            blocks.append(entry)
    # 同一根塔的各层 xy 相同 —— 用 layer 作最后一级键，别让排序抖动
    blocks.sort(key=lambda b: (b["x"], b["y"], b["layer"]))
    for i, b in enumerate(blocks):
        b["id"] = i
    return {
        "batch": batch,
        "index": int(index),
        "source": "stage4_truth_from_frame:ray-plane",
        "block_size_m": float(block_size),
        "table_top_m": float(table_top),
        "blocks": blocks,
        "dropped_out_of_zone": n_out,
        "dropped_not_block": n_unknown,
        "dropped_unsplit_pair": n_unsplit,
        "zone_m": list(sp.ZONE_M if zone_m is None else zone_m),
    }


CAMERAS = {
    # 规划感知走**腕部**（用户 2026-11-09 定）：俯视的绝对 xy 有 20–45 mm
    # 系统偏差，只能当视场引导；腕部链由手眼外参定，是毫米级。
    "wrist": (gc.TIP, gc.CHAIN),
    # 俯视只剩一个角色：跑完之后的判分器（`--verify --colour-burst`）。
    "topcam": (gc.TOPCAM_TIP, gc.TOPCAM_CHAIN),
}


def measure_dir(frame_dir, index=None, split=True, yaw_method="colour",
                camera="wrist", seg="grid"):
    """一帧 → (frame, find_blocks 行, 台面 z)。

    ``camera``：``'wrist'``（默认，规划感知 + 跑后三机位判分）或
    ``'topcam'``（已退出阶段 4，只用于复现旧证据；CLI 要 --allow-topcam）。
    两者只差 tf 链与 K —— 腕部链经过 ``tool0``，位置随当帧关节走。

    ``yaw_method='colour'``（默认）把每行的 yaw 换成彩色像素掩码矩形
    （见 ``colour_yaw_rows``）；``'depth'`` 是旧口径（PCA），只为复现旧证据。

    ``seg='sam'``：分割/身份改走 SAM 感知层（``sam_blocks.find_blocks_sam``，
    第 345 轮立项）；``'grid'``（缺省）= 现行网格链。其余下游
    （build_truth/consensus/判分）对两种行同构消费。
    """
    if camera not in CAMERAS:
        raise ValueError("camera 只能是 %s，收到 %r" % (sorted(CAMERAS), camera))
    if seg not in ("grid", "sam"):
        raise ValueError("seg 只能是 grid / sam，收到 %r" % (seg,))
    tip, chain = CAMERAS[camera]
    frame = gc.load_frame(frame_dir, tip=tip, chain=chain, idx=index)
    if seg == "sam":
        import sam_blocks
        predictor = sam_blocks.load_predictor()
        rows, table_z = sam_blocks.find_blocks_sam(frame, predictor)
    else:
        rows, table_z = gc.find_blocks(frame, split=split)
    if yaw_method == "colour":
        colour_yaw_rows(frame, rows)
    else:
        for r in rows:
            r["yaw_source"] = "depth_pca"
            r["yaw_pca"] = float(r.get("yaw", 0.0))
    return frame, rows, table_z


def _measure_index(job):
    """进程池的一帧任务：量一帧 → 单帧真值。

    顶层函数（不是闭包/方法）才可被 ``ProcessPoolExecutor`` pickle 过去。
    参数是一个 dict，省得加参数时要同步改签名两侧。
    """
    kw = job
    fr, rows, _ = measure_dir(kw["frame_dir"], index=kw["index"],
                              split=kw["split"], yaw_method=kw["yaw_method"],
                              camera=kw["camera"], seg=kw.get("seg", "grid"))
    return build_truth(rows, fr, table_top=kw["table_top"],
                       block_size=kw["block_size"],
                       batch=kw["batch"], index=kw["index"],
                       keep_outside=True,
                       drop_unknown=kw["drop_unknown"],
                       min_class_share=kw["min_class_share"],
                       layers=kw["layers"], zone_m=kw["zone"])


def _median(vals):
    v = sorted(vals)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def _yaw_square(vals):
    """方块朝向的圆中位；周期 90°（``4×`` 折角就是这个意思，不需要先 ``% 90``）。

    积木是正方形，yaw 与 yaw+90° 表示同一个盒子；不折周期的话 PCA 会在两组
    相差 ~90° 的值之间来回跳，直接取中位会落在两者中间（最坏）。
    """
    sx = sum(np.cos(np.radians(4.0 * v)) for v in vals)
    sy = sum(np.sin(np.radians(4.0 * v)) for v in vals)
    ang = math.degrees(math.atan2(sy, sx)) / 4.0
    return ang % 90.0


def consensus(truths, radius_m=0.020, min_frames=None):
    """多帧真值按位置聚类 → 一份稳定真值（抗单帧漏块/裂块/幻影）。

    单帧 ``find_blocks`` 在这台真机上是**不稳的**（2026-11-08 实测 10 帧：
    同一块位置摆动 2–8 mm；有的帧漏一块；一帧把青块切成 4 份；偶发幻影）。
    E8 的碰撞场景要的是静态真值，所以把同一 burst 的每一帧各量一遍再合并：

    - 每帧的块按 (x, y) 贪心归簇（阈值 ``radius_m``，对簇心判距）；
    - 出现的帧数 < ``min_frames`` 的簇丢掉（幻影/裂片只在一两帧出现）；
    - 位置取中位；颜色多数票；yaw 取方块的圆中位（见 ``_yaw_square``）；
    - 逐块记录出现帧数与位置散布，让报告里看得见「这块稳不稳」。
    """
    if not truths:
        raise ValueError("consensus 至少要一帧")
    n = len(truths)
    if min_frames is None:
        min_frames = max(3, (n + 1) // 2)
    clusters = []                      # [{pts: [...], frames: set}]
    for fi, t in enumerate(truths):
        for b in t["blocks"]:
            best, best_d = None, None
            for c in clusters:
                cx = _median([p["x"] for p in c["pts"]])
                cy = _median([p["y"] for p in c["pts"]])
                cz = _median([p["z"] for p in c["pts"]])
                # 三维距离：塔的上下层 xy 完全相同（coaxial），只按 xy 聚会把
                # 两层并成一块 —— 层间差一个 size（29.5 mm）> radius 就分开。
                d = math.sqrt((b["x"] - cx) ** 2 + (b["y"] - cy) ** 2
                              + (b["z"] - cz) ** 2)
                if d <= radius_m and (best_d is None or d < best_d):
                    best, best_d = c, d
            if best is None:
                clusters.append({"pts": [b], "frames": {fi}})
            else:
                best["pts"].append(b)
                best["frames"].add(fi)
    blocks = []
    for c in clusters:
        if len(c["frames"]) < min_frames:
            continue                       # 只在一两帧出现 ⇒ 幻影/裂片
        pts = c["pts"]
        xs = [p["x"] for p in pts]
        ys = [p["y"] for p in pts]
        mx, my = _median(xs), _median(ys)
        colors = {}
        for p in pts:
            colors[p["color"]] = colors.get(p["color"], 0) + 1
        color = max(colors, key=colors.get)
        size = float(pts[0]["size"])
        # top 逐层不同（塔顶 = n·size），沿用各帧自己的 top 取中位；
        # 旧写法 ``top − size`` 反推台面，在塔上会算出 n−1 层的高度。
        top = _median([float(p["top"]) for p in pts])
        layer = int(max(p.get("layer", 0) for p in pts))
        # yaw 只认彩色掩码那一路：PCA 在近正方形块上没有定义，混进来会把
        # 共识角拖偏（E9 s4：PCA 146.9° vs 彩色 4.4°）。彩色一帧都没有时才
        # 退回全 PCA，并在 yaw_source 里写明。
        pairs = [(float(p["yaw"]),
                  p["meas"].get("yaw_source", "depth_pca")) for p in pts]
        colour = [v for v, s in pairs if s == "colour_rect"]
        src_counts = {}
        for _, s in pairs:
            src_counts[s] = src_counts.get(s, 0) + 1
        if colour:
            yaw = _yaw_square(colour)
            yaw_source = "colour_rect"
        else:
            yaw = _yaw_square([v for v, _ in pairs])
            yaw_source = "depth_pca"
        # 图像空间块心（只记录；2026-11-18）：跨帧取中位，附上与该块
        # 深度方框中心（x/y）的差 —— 规划中心暂不切换（队列条目）。
        imgs = [(float(p["meas"]["img_xy_mm"][0]),
                 float(p["meas"]["img_xy_mm"][1]))
                for p in pts if p["meas"].get("img_xy_mm")]
        img_meas = {"img_frames": len(imgs)}
        if imgs:
            img_meas["img_xy_mm"] = [
                round(_median([v[0] for v in imgs]), 2),
                round(_median([v[1] for v in imgs]), 2)]
            img_meas["img_minus_xy_mm"] = [
                round(_median([v[0] for v in imgs]) - mx * 1000, 2),
                round(_median([v[1] for v in imgs]) - my * 1000, 2)]
        blocks.append({
            "id": 0,
            "color": color,
            "layer": layer,
            "size": size,
            "top": top,
            "x": mx,
            "y": my,
            "z": top - size / 2.0,
            "yaw": yaw,
            "meas": {"n_frames": len(c["frames"]),
                     "n_pts": len(pts),
                     "from": pts[0]["meas"].get("from"),
                     "yaw_source": yaw_source,
                     "yaw_colour_frames": len(colour),
                     "yaw_sources": src_counts,
                     "x_spread_mm": round((max(xs) - min(xs)) * 1000, 2),
                     "y_spread_mm": round((max(ys) - min(ys)) * 1000, 2),
                     "colors": colors,
                     "h_mm_median": round(_median(
                         [p["meas"].get("h_mm", 0.0) for p in pts]), 2),
                     **img_meas},
        })
    # 共识后的**贴合对再钳一次**：逐块的 x/y 各自取中位会把成对几何（距离/
    # 朝向）揉掉——S2:5 实测逐帧 29.50 mm → 共识 29.22。这里只对「同层、最小
    # 距、落在贴合带 0.55–1.02×块边」的那一对沿连线对称推开到块边，中点不动；
    # 与 sam_blocks.clamp_pairs 同口径（2026-11-18 块心精度）。
    if len(blocks) >= 2:
        best = None
        for i in range(len(blocks)):
            for j in range(i + 1, len(blocks)):
                a, b = blocks[i], blocks[j]
                if int(a.get("layer", 0)) != int(b.get("layer", 0)):
                    continue
                d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                side = float(a.get("size") or BLOCK_SIZE_M)
                if 0.55 * side <= d <= 1.02 * side \
                        and (best is None or d < best[0]):
                    best = (d, a, b, side)
        if best is not None:
            d, a, b, side = best
            ux, uy = (b["x"] - a["x"]) / d, (b["y"] - a["y"]) / d
            push = (side - d) / 2.0
            for blk, sign in ((a, -1.0), (b, 1.0)):
                blk["x"] = round(blk["x"] + sign * ux * push, 6)
                blk["y"] = round(blk["y"] + sign * uy * push, 6)
                blk["meas"]["pair_clamp_from_mm"] = round(d * 1000.0, 2)
    # 钳制后再算一次图像块心对 x/y 的差（钳制会移动 ≤0.4 mm）。
    for b in blocks:
        im = b["meas"].get("img_xy_mm")
        if im:
            b["meas"]["img_minus_xy_mm"] = [
                round(im[0] - b["x"] * 1000.0, 2),
                round(im[1] - b["y"] * 1000.0, 2)]
    blocks.sort(key=lambda b: (b["x"], b["y"]))
    for i, b in enumerate(blocks):
        b["id"] = i
    return {
        "batch": truths[0]["batch"],
        "index": truths[0]["index"],
        "source": "stage4_truth_from_frame:consensus",
        "block_size_m": float(truths[0]["block_size_m"]),
        "table_top_m": float(truths[0]["table_top_m"]),
        "n_frames": n,
        "min_frames": min_frames,
        "cluster_radius_mm": radius_m * 1000.0,
        "blocks": blocks,
        "zone_m": list(truths[0]["zone_m"]),
    }


def survey(frame_dir, split=True, table_top=0.0, yaw_method="colour",
           camera="topcam", seg="grid"):
    idxs = gc.list_frame_indices(frame_dir)
    print("帧数 %d（yaw 口径：%s，相机：%s，分割：%s）"
          % (len(idxs), yaw_method, camera, seg))
    for idx in idxs:
        frame, rows, table_z = measure_dir(frame_dir, index=idx, split=split,
                                           yaw_method=yaw_method, camera=camera,
                                           seg=seg)
        t = build_truth(rows, frame, table_top=table_top)
        print("  idx=%-3s 深度台面=%8.2f mm  块数=%2d  区外=%2d  非积木=%2d"
              % (idx, table_z * 1000.0, len(t["blocks"]),
                 t["dropped_out_of_zone"], t["dropped_not_block"]))
        for b in t["blocks"]:
            # 塔的下层是 coaxial 推定的，没有实测深度点 —— 直接下标会 KeyError
            # （2026-11-10 E1 复验时在 E9 真机腕部帧上踩到，塔一出现必崩）。
            dxy = b["meas"].get("depth_xy_mm")
            extra = ("" if not dxy else
                     "（深度口径 x=%7.1f y=%7.1f）" % (dxy[0], dxy[1]))
            print("      #%d %-11s x=%7.1f y=%7.1f yaw=%6.1f%s"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["yaw"], extra))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="真机 burst（俯视/腕部）→ 探针的 scene_truth.json"
                    "（射线 ∩ 平面）")
    ap.add_argument("--frame", required=True, help="帧目录（burst）")
    ap.add_argument("--index", type=int, default=None,
                    help="用第几帧（默认：burst 里取第 0 帧）")
    ap.add_argument("--block-size", type=float, default=BLOCK_SIZE_M,
                    help="积木边长（米），默认 0.0295")
    ap.add_argument("--table-top", type=float, default=0.0,
                    help="台面顶面在 base 系的 z（米），默认 0.0（生产栈口径）")
    ap.add_argument("--out", default=None, help="写到哪；不给就打印到 stdout")
    ap.add_argument("--no-split", action="store_true",
                    help="关闭 find_blocks 的顶面切分")
    ap.add_argument("--keep-outside", action="store_true",
                    help="拾取区外的检测也写进真值（默认丢掉，防止幻影变假障碍）")
    ap.add_argument("--keep-unknown", action="store_true",
                    help="保留 class=unknown 的簇（整簇无饱和像素，默认丢）")
    ap.add_argument("--allow-empty", action="store_true",
                    help="零块也照常写出真值并返 0：**干净的机位是合法结果**。"
                         "跑后三机位用它 —— 否则「本机位没有块」会退化成"
                         "「本机位没有证据」，整轮的全桌终态就少了一只眼"
                         "（2026-11-12 S0:1 实测：free 机位空 ⇒ rc=2 ⇒ 被剔"
                         "出溯源）。默认仍返 2，防止分割/相机整体坏掉时出假绿")
    ap.add_argument("--min-class-share", type=float, default=0.0,
                    help="颜色票占比下限（默认 0 = 不过滤）")
    ap.add_argument("--survey", action="store_true",
                    help="逐帧各量一遍，只打印稳定性，不写真值")
    ap.add_argument("--consensus", action="store_true",
                    help="每帧各量一遍再按位置聚类合并（抗单帧漏块/裂块/幻影）")
    ap.add_argument("--jobs", type=int, default=None, metavar="N",
                    help="--consensus 逐帧量测的并行进程数（默认 min(CPU, 帧数)；"
                         "1 = 串行，用于对照/调试）")
    ap.add_argument("--min-frames", type=int, default=None,
                    help="共识要求至少出现的帧数（默认 max(3, ⌈n/2⌉)）")
    ap.add_argument("--cluster-mm", type=float, default=20.0,
                    help="共识聚类的距离阈值（毫米），默认 20；按三维距离算"
                         "（塔的上下层 xy 相同，靠 z 分开）")
    ap.add_argument("--no-layers", action="store_true",
                    help="关掉塔的分层展开（每簇只当一块；E9 的塔必须开着）")
    ap.add_argument("--yaw-depth", action="store_true",
                    help="yaw 退回 find_blocks 的深度 PCA 长边（旧口径，只为"
                         "复现旧证据；默认用彩色像素掩码矩形）")
    ap.add_argument("--camera", choices=sorted(CAMERAS), default="wrist",
                    help="用哪路相机：wrist=腕部（默认，规划感知 + 跑后三机位"
                         "判分；要当帧关节在帧目录里）；topcam=已退出阶段 4，"
                         "只用于复现旧证据，必须同时给 --allow-topcam")
    ap.add_argument("--seg", choices=("grid", "sam"), default="sam",
                    help="分割/身份走哪条感知链（第 345 轮立项、347 轮回放"
                         "达标后翻缺省）：sam=SAM 感知层（sam_blocks，权重 "
                         "~/robotics/third_party/weights/mobile_sam.pt），"
                         "grid=网格链（回退/对照）")
    ap.add_argument("--allow-topcam", action="store_true",
                    help="显式允许用俯视相机复现旧证据（不给它却传 "
                         "--camera topcam ⇒ 退出码 2）")
    ap.add_argument("--zone", default=None, metavar="X0,X1,Y0,Y1",
                    help="真值保留区（base 系米）。缺省 = 拾取区 ZONE_M。"
                         "E9 真机半段要覆盖垫面与北侧自由区，例如 "
                         "0.20,0.80,-0.30,0.65")
    args = ap.parse_args(argv)

    # 俯视已退出阶段 4（v0.6）：默认腕部；显式 topcam 必须再过一道开关。
    if args.camera == "topcam" and not args.allow_topcam:
        print("!! --camera topcam 已退出阶段 4 正式流程。\n"
              "   正式轮用缺省 wrist；确需复现旧证据请同时给 --allow-topcam。",
              file=sys.stderr)
        return 2

    zone = None
    if args.zone:
        try:
            vals = [float(v) for v in args.zone.split(",")]
        except ValueError:
            vals = []
        if len(vals) != 4:
            print("!! --zone 要四个数 X0,X1,Y0,Y1（米），收到 %r" % args.zone,
                  file=sys.stderr)
            return 2
        zone = tuple(vals)

    if not os.path.isdir(args.frame):
        print("!! 帧目录不存在：%s" % args.frame, file=sys.stderr)
        return 2
    if not glob.glob(os.path.join(args.frame, "color*.png")):
        print("!! 帧目录里没有 color*.png：%s" % args.frame, file=sys.stderr)
        return 2

    if args.survey:
        survey(args.frame, split=not args.no_split,
               table_top=args.table_top,
               yaw_method="depth" if args.yaw_depth else "colour",
               camera=args.camera, seg=args.seg)
        return 0

    yaw_method = "depth" if args.yaw_depth else "colour"

    idx = args.index
    if idx is None:
        idxs = gc.list_frame_indices(args.frame)
        idx = idxs[0] if idxs else None

    if args.consensus:
        idxs = gc.list_frame_indices(args.frame)
        batch = os.path.basename(args.frame.rstrip("/"))
        # 逐帧**先不按区筛**：贴边界的块会因单帧噪声在区内/区外来回跳，
        # 先筛会把同一块在不同帧切掉 ⇒ 出现帧数虚低。区的判定留到合并之后，
        # 用中位位置判一次。
        #
        # 每帧互相独立（只读各自 PNG + tf json），所以丢进程池并行。2026-11-12
        # 实测 20 帧单进程 30.4 s（串行、只吃 1 核，本机 16 核闲着）—— 这是
        # 整轮里最容易拿回来的一段。``--jobs 1`` 保留串行路径做对照。
        jobs = {k: v for k, v in (
            ("frame_dir", args.frame), ("split", not args.no_split),
            ("yaw_method", yaw_method), ("camera", args.camera),
            ("seg", args.seg),
            ("table_top", args.table_top), ("block_size", args.block_size),
            ("batch", batch), ("drop_unknown", not args.keep_unknown),
            ("min_class_share", args.min_class_share),
            ("layers", not args.no_layers), ("zone", zone))}
        tasks = [dict(jobs, index=i) for i in idxs]
        n_jobs = args.jobs
        if n_jobs is None:
            n_jobs = min(len(tasks), os.cpu_count() or 1)
        n_jobs = max(1, min(n_jobs, len(tasks) or 1))
        t0 = time.time()
        if n_jobs == 1 or len(tasks) <= 1:
            truths = [_measure_index(t) for t in tasks]
        else:
            import concurrent.futures as cf
            with cf.ProcessPoolExecutor(max_workers=n_jobs) as ex:
                # map 保序：共识里 frames 集合用枚举下标，顺序变了不影响结果，
                # 但首帧元数据（batch/index/zone）取自 truths[0]，保序更直观。
                truths = list(ex.map(_measure_index, tasks))
        print("逐帧量测 %d 帧 / %d 进程：%.1f s（串行时约 %.1f s）"
              % (len(tasks), n_jobs, time.time() - t0,
                 (time.time() - t0) * n_jobs if n_jobs > 1 else 0.0))
        if len(truths) < max(3, args.min_frames) and args.min_frames is None:
            # 2026-11-14 审计：共识门 min_frames = max(3, ⌈n/2⌉) 在 n≤4 时
            # 自锁死（min_frames > n ⇒ 全簇被剔 ⇒ 0 块），而 main 的 0 块
            # 分支会误报「分割/颜色表不对」。burst 太短在这里就明说。
            print("!! 帧数 %d < 3：共识门要求 ≥3 帧 —— burst 太短，"
                  "补抓帧而不是查分割/颜色表" % len(truths), file=sys.stderr)
            return 2
        truth = consensus(truths, radius_m=args.cluster_mm / 1000.0,
                          min_frames=args.min_frames)
        if not args.keep_outside:
            keep = [b for b in truth["blocks"]
                    if in_zone_xy(b["x"], b["y"], zone)]
            truth["dropped_out_of_zone"] = len(truth["blocks"]) - len(keep)
            truth["blocks"] = keep
            for j, b in enumerate(truth["blocks"]):
                b["id"] = j
        else:
            truth["dropped_out_of_zone"] = 0
        n = len(truth["blocks"])
        print("共识 %d 帧（≥%d 帧才算一块）：块 %d 个，区外丢 %d"
              % (truth["n_frames"], truth["min_frames"], n,
                 truth["dropped_out_of_zone"]))
        for b in truth["blocks"]:
            m = b["meas"]
            print("  #%d %-9s x=%7.1f y=%7.1f top=%6.1f 层 %d yaw=%5.1f  "
                  "帧 %2d/%d  散布 x=%.1f y=%.1f mm  颜色 %s  yaw来源 %s(%d)"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["top"] * 1000, b["layer"] + 1, b["yaw"], m["n_frames"],
                     truth["n_frames"], m["x_spread_mm"], m["y_spread_mm"],
                     m["colors"], m.get("yaw_source", "?"),
                     m.get("yaw_colour_frames", 0)))
    else:
        frame, rows, table_z = measure_dir(args.frame, index=idx,
                                           split=not args.no_split,
                                           yaw_method=yaw_method,
                                           camera=args.camera, seg=args.seg)
        truth = build_truth(rows, frame, table_top=args.table_top,
                            block_size=args.block_size,
                            batch=os.path.basename(args.frame.rstrip("/")),
                            index=0 if idx is None else idx,
                            keep_outside=args.keep_outside,
                            drop_unknown=not args.keep_unknown,
                            min_class_share=args.min_class_share,
                            layers=not args.no_layers, zone_m=zone)
        n = len(truth["blocks"])
        print("帧 %s：台面 z=%.2f mm，块 %d 个（位置=射线∩各自顶面平面），"
              "区外丢 %d / 非积木丢 %d"
              % (args.frame, table_z * 1000.0, n,
                 truth["dropped_out_of_zone"], truth["dropped_not_block"]))
        for b in truth["blocks"]:
            print("  #%d %-11s x=%7.1f y=%7.1f top=%6.1f yaw=%6.1f  "
                  "h=%6.1f  层 %d（%s）  yaw来源 %s%s"
                  % (b["id"], b["color"], b["x"] * 1000, b["y"] * 1000,
                     b["top"] * 1000, b["yaw"],
                     float(b["meas"].get("h_mm") or 0.0), b["layer"] + 1,
                     b["meas"].get("from", "?"),
                     b["meas"].get("yaw_source", "?"),
                     "" if "warn_h_mm" not in b
                     else "  ⚠ h−层高=%+.1f mm" % b["warn_h_mm"]))
    if n == 0 and not args.allow_empty:
        print("!! 一块都没量出来 —— 俯视里没有块，或分割/颜色表不对",
              file=sys.stderr)
        return 2
    if n == 0:
        print("（--allow-empty：本机位零块，按「干净机位」记）")

    text = json.dumps(truth, ensure_ascii=False, indent=2, sort_keys=True)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print("写出 %s" % args.out)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
