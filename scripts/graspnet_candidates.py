#!/usr/bin/env python3
"""一帧 → 点云 → GraspNet 候选 → 按「离块真值 ≤ 12 mm」锚定 —— 精密抓取的第 2、3 段。

为什么在开发机跑，不在板上
--------------------------
板上没有 torch / MinkowskiEngine / graspness 权重，也没有外网；开发机的跨进程
DDS 又是坏的（当不了 ROS 节点）。所以板上只抓帧，推理全在这里。见
``precision_capture.py`` 的说明。

为什么候选不能靠分数认块
------------------------
round 144 实测：全场最高分的两个候选（0.397 / 0.331）落在**平板上**，不是积木。
GraspNet 的分数是「这个点像不像可抓」，不是「这是哪一块」。所以这里的分工是：

    块真值（几何聚类）→ 定「抓哪一块、在哪」；
    GraspNet 候选       → 定「这一块上抓哪儿」。

锚定判据：候选 xy 离某块顶面中心 ≤ ``--anchor-mm``（默认 12 mm）。分数只在
**同一块的合格候选之间**排序。

预处理**故意与离线原型完全一致**（voxel 5 mm、20000 点、seed 0、score≥0.20、
倾斜≤35°）。原型在真机上已经抓成功过一次；这一轮要回答的是「换到整轮流程还成不成立」，
不是「调预处理能不能更好」。改输入分布会让两次结果不可比。

用法：
  python3 graspnet_candidates.py <帧目录> [--json out.json]
  python3 graspnet_candidates.py <帧目录> --no-infer      # 只量块，不跑网络
  python3 graspnet_candidates.py <帧目录> --block 2       # 只看第 2 块（0 起）

需要的环境：numpy/PIL/scipy（量块）+ torch/MinkowskiEngine/graspness（推理）。
推理权重默认 ``~/robotics/third_party/graspnet-baseline/logs/checkpoint-rs.tar``。
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import block_classes                                              # noqa: E402

import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

# 聚类只用 numpy 手写（binary closing + 4 连通标号），**不依赖 scipy**：
# 推理环境（graspnet venv）里没有 scipy，而这两步加起来不到 20 行。
# 语义要与 scipy.ndimage 的默认一致：closing 3×3、label 4 连通 —— 否则同一帧
# 会数出不同的块，历史结果就不可比了。

# 与 capture_live / precision_capture 同一个链
CHAIN = (
    ("base_link", "base_link_inertia"), ("base_link_inertia", "shoulder_link"),
    ("shoulder_link", "upper_arm_link"), ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"), ("wrist_1_link", "wrist_2_link"),
    ("wrist_2_link", "wrist_3_link"), ("wrist_3_link", "flange"),
    ("flange", "tool0"), ("tool0", "camera_link"),
    ("camera_link", "camera_color_frame"),
    ("camera_color_frame", "camera_color_optical_frame"),
)
TIP = "camera_color_optical_frame"

VOXEL = 0.005
NUM_POINTS = 20000
MIN_SCORE = 0.20
MAX_TILT_DEG = 35.0
TABLE_Z = 0.0285          # 规划器 place_z；块顶面实测 32–35 mm
ANCHOR_MM = 12.0
GRASPNET_ROOT = os.path.expanduser("~/robotics/third_party/graspness_unofficial")
CKPT = os.path.expanduser("~/robotics/third_party/graspnet-baseline/logs/checkpoint-rs.tar")


# ---------------------------------------------------------------- 帧 I/O
def quat_to_R(q):
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tf_entry(tf, parent, child):
    t = tf["%s:%s" % (parent, child)]
    M = np.eye(4)
    M[:3, :3] = quat_to_R([t["rotation"][k] for k in "xyzw"])
    M[:3, 3] = [t["translation"][k] for k in "xyz"]
    return M


def cam_pose(frame_dir):
    """base_link ← camera_color_optical_frame。老帧可能直接存了合成结果。"""
    tf = json.load(open(os.path.join(frame_dir, "tf.json")))["ok"]
    direct = "base_link:%s" % TIP
    if direct in tf:
        T = _tf_entry(tf, "base_link", TIP)
        return T[:3, :3], T[:3, 3]
    T = np.eye(4)
    cur = "base_link"
    while cur != TIP:
        nxt = next(b for a, b in CHAIN if a == cur)
        T = T @ _tf_entry(tf, cur, nxt)
        cur = nxt
    return T[:3, :3], T[:3, 3]


def camera_k(frame_dir):
    p = os.path.join(frame_dir, "camera_info.json")
    if os.path.exists(p):
        return np.array(json.load(open(p))["K"], float).reshape(3, 3)
    m = json.load(open(os.path.join(frame_dir, "meta.json")))
    return np.array(m["camera_info"]["K"], float).reshape(3, 3)


def load_frame(frame_dir):
    depth = np.array(Image.open(os.path.join(frame_dir, "depth.png")),
                     dtype=np.float64) / 1000.0
    color = np.array(Image.open(os.path.join(frame_dir, "color.png")).convert("RGB"))
    R, t = cam_pose(frame_dir)
    js_path = os.path.join(frame_dir, "joint_states.json")
    joints = json.load(open(js_path)) if os.path.exists(js_path) else None
    return {"dir": frame_dir, "depth": depth, "color": color,
            "K": camera_k(frame_dir), "R_bc": R, "t_bc": t, "joints": joints}


def backproject(depth, K):
    """深度图 → 相机系点（m）+ 对应像素。只保留有限深度。"""
    h, w = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vs, us = np.mgrid[0:h, 0:w]
    valid = (depth > 0.10) & (depth < 1.5)
    P = np.stack([(us - cx) / fx * depth,
                  (vs - cy) / fy * depth,
                  depth], -1)[valid]
    return P, np.stack([us[valid], vs[valid]], -1)


def _shift_or(a):
    """3×3 膨胀：自身与 8 个平移版求或。平移用 roll，回绕落在四周的空边框上。"""
    out = a.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= np.roll(np.roll(a, dy, axis=0), dx, axis=1)
    return out


def binary_closing(a):
    """膨胀再腐蚀（腐蚀 = 对补集膨胀再取反）。"""
    return ~_shift_or(~_shift_or(a))


def label4(occ):
    """4 连通标号 → (labels, n)。等价于 scipy.ndimage.label 的默认结构。"""
    labels = np.zeros(occ.shape, np.int32)
    n = 0
    h, w = occ.shape
    for y0, x0 in zip(*np.where(occ)):
        if labels[y0, x0]:
            continue
        n += 1
        stack = [(y0, x0)]
        labels[y0, x0] = n
        while stack:
            y, x = stack.pop()
            for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= yy < h and 0 <= xx < w and occ[yy, xx] and not labels[yy, xx]:
                    labels[yy, xx] = n
                    stack.append((yy, xx))
    return labels, n


def table_plane_z(points_base):
    """台面 z：±60 mm 带内直方图的众数。

    不能用全局众数 —— 台面外的地板会抢走它（rot50 那帧地板 308k 点 > 台面 231k 点，
    整帧被判成 z=-78 mm）。
    """
    hist, edges = np.histogram(points_base[:, 2], bins=np.arange(-0.060, 0.060, 0.002))
    return float(edges[np.argmax(hist)])


# ---------------------------------------------------------------- 块真值
def rgb_to_hsv255(rgb):
    """RGB(0-255) → (H 度, S 0-255, V 0-255)。与 cv2 同一定义，但不依赖 cv2
    （推理环境里没有 OpenCV）。"""
    r, g, b = [float(v) / 255.0 for v in rgb]
    mx, mn = max(r, g, b), min(r, g, b)
    d = mx - mn
    if d == 0:
        h = 0.0
    elif mx == r:
        h = 60.0 * (((g - b) / d) % 6.0)
    elif mx == g:
        h = 60.0 * ((b - r) / d + 2.0)
    else:
        h = 60.0 * ((r - g) / d + 4.0)
    s = 0.0 if mx == 0 else d / mx * 255.0
    return h % 360.0, s, mx * 255.0


def circular_median_deg(angles):
    """角度中位数：先取单位向量平均定参考方向，再在 ±180 内取中位。"""
    a = np.asarray(angles, float)
    if len(a) == 0:
        return 0.0
    ref = np.degrees(np.arctan2(np.sin(np.radians(a)).mean(),
                                np.cos(np.radians(a)).mean()))
    rel = ((a - ref + 180.0) % 360.0) - 180.0
    return float((ref + np.median(rel)) % 360.0)


def find_blocks(frame, table_z=None):
    """把「立在台面上的块」逐块量出来：顶面中心/高度/尺寸/yaw/颜色。

    与 ``find_blocks.py``（临时脚本）的区别：颜色取**顶面那 3 mm**的像素，
    不是整块。整块的平均会把侧面的阴影拉进来 —— 侧面比顶面暗，色相会往短波挪，
    而橙↔棕只差 8°。
    """
    R, t, K = frame["R_bc"], frame["t_bc"], frame["K"]
    depth, color = frame["depth"], frame["color"]
    P, pix = backproject(depth, K)
    Pb = P @ R.T + t
    if table_z is None:
        table_z = table_plane_z(Pb)

    m = (Pb[:, 2] > table_z + 0.005) & (Pb[:, 2] < table_z + 0.070)
    Q, qpix = Pb[m], pix[m]
    if len(Q) == 0:
        return [], table_z

    cell = 0.004
    ij = np.floor(Q[:, :2] / cell).astype(np.int64)
    ij -= ij.min(0) - 1
    occ = np.zeros(ij.max(0) + 3, bool)
    occ[ij[:, 0], ij[:, 1]] = True
    occ = binary_closing(occ)
    lab, n = label4(occ)
    cell_lab = lab[ij[:, 0], ij[:, 1]]

    out = []
    for l in range(1, n + 1):
        sel = cell_lab == l
        if sel.sum() < 200:
            continue
        Qs = Q[sel]
        lo, hi = Qs.min(0), Qs.max(0)
        if (hi[0] - lo[0]) < 0.010 or (hi[1] - lo[1]) < 0.010:
            continue
        if len(Qs) > 20000:
            continue                       # 台面级的大平板，不是积木
        ztop = np.percentile(Qs[:, 2], 97)
        F = Qs[Qs[:, 2] > ztop - 0.003]
        c = F[:, :2].mean(0)
        d = F[:, :2] - c
        evals, evecs = np.linalg.eigh(d.T @ d / len(d))
        yaw = float(np.degrees(np.arctan2(evecs[1, -1], evecs[0, -1])))
        ext = (d @ evecs).max(0) - (d @ evecs).min(0)

        # 顶面像素：F 是 Qs 的子集，要回到 Q/qpix 的下标才能索引彩色图
        sel_idx = np.where(sel)[0]
        f_idx = sel_idx[Qs[:, 2] > ztop - 0.003]
        f_pix = qpix[f_idx]
        px = color[f_pix[:, 1], f_pix[:, 0]].astype(float)
        med_rgb = np.median(px, axis=0)
        hue, sat, val = rgb_to_hsv255(med_rgb)

        # 逐像素判类 → **按饱和度加权**的多数票。
        #
        # 为什么必须加权、且要滤掉低饱和像素：灰/白/木色像素的色相是纯噪声。
        # 本帧那块 8 mm 的平板（S=29/255）第一版就被投票判成了 cyan —— 它的
        # 中位色相是 0°，与 cyan 的 191° 差了 169°，只有逐像素噪声能做到这种错。
        # ``palette_survey.py`` 的直方图也是**按饱和度加权**的，同一个理由。
        samples = px[::max(1, len(px) // 400)]
        votes, wsum = {}, 0.0
        for p in samples:
            h_, s_, v_ = rgb_to_hsv255(p)
            if s_ < 60 or v_ < 40:          # 与 palette_survey 的 s_min/v_min 同值
                continue
            name, _ = block_classes.nearest_class_by_hue(h_)
            votes[name] = votes.get(name, 0.0) + s_
            wsum += s_
        if wsum <= 0:
            cls, share = "unknown", 0.0
        else:
            cls = max(votes, key=votes.get)
            share = votes[cls] / wsum
        votes = {k: int(v) for k, v in votes.items()}

        # 顶面中心投到图像上：块贴边时 GraspNet 几乎给不出候选（实测 live-04 的
        # 三块都在 v≈670/720 的底边上，紫块最好候选偏 10 mm）。这个像素坐标
        # 就是「取景够不够」的判据，别等到候选数掉了才反推原因。
        pc = np.array([c[0], c[1], ztop])
        uv = K @ (R.T @ (pc - t))
        u_px, v_px = (float(uv[0] / uv[2]), float(uv[1] / uv[2])) if uv[2] > 1e-6 else (0.0, 0.0)

        out.append({
            "cx": float(c[0] * 1000), "cy": float(c[1] * 1000),
            "top": float(ztop * 1000), "h": float((ztop - table_z) * 1000),
            "uv": [round(u_px, 1), round(v_px, 1)],
            "ext": [float(v * 1000) for v in ext],
            "yaw": yaw, "n": int(sel.sum()),
            "rgb": [round(float(v)) for v in med_rgb],
            "hsv": [round(hue, 1), round(sat), round(val)],
            "class": cls, "class_share": round(float(share), 3),
            "votes": dict(sorted(votes.items(), key=lambda kv: -kv[1])),
        })
    out.sort(key=lambda r: (r["cx"], r["cy"]))
    for i, r in enumerate(out):
        r["idx"] = i
    return out, table_z


# ---------------------------------------------------------------- GraspNet
def infer(points_cam, ckpt=CKPT, seed=0):
    """相机系点云 → GraspNet 原始输出（N×17）。**延迟 import**：
    量块的部分不需要 torch/ME，没有推理环境的机器也能跑 ``--no-infer``。"""
    import torch
    import MinkowskiEngine as ME
    sys.path.insert(0, GRASPNET_ROOT)
    sys.path.insert(0, os.path.join(GRASPNET_ROOT, "pointnet2"))
    sys.path.insert(0, os.path.join(GRASPNET_ROOT, "utils"))
    from models.graspnet import GraspNet, pred_decode

    rng = np.random.default_rng(seed)
    pts = points_cam[rng.choice(len(points_cam), NUM_POINTS,
                                replace=len(points_cam) < NUM_POINTS)].astype(np.float32)

    dev = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net = GraspNet(seed_feat_dim=512, is_training=False).to(dev).eval()
    net.load_state_dict(torch.load(ckpt, map_location="cpu",
                                   weights_only=True)["model_state_dict"],
                        strict=True)

    coors = np.ascontiguousarray(pts / VOXEL, dtype=np.int32)
    feats = np.ones_like(pts, dtype=np.float32)
    coors_b, feats_b = ME.utils.sparse_collate([coors], [feats])
    coors_b = np.ascontiguousarray(coors_b, dtype=np.int32)
    coors_b, feats_b, _, q2o = ME.utils.sparse_quantize(
        coors_b, feats_b, return_index=True, return_inverse=True)

    def as_tensor(x, dtype):
        t = x if torch.is_tensor(x) else torch.from_numpy(np.asarray(x))
        return t.to(dtype=dtype, device=dev)

    ep = {"point_clouds": torch.from_numpy(pts[None]).to(dev),
          "coors": as_tensor(coors_b, torch.int32),
          "feats": as_tensor(feats_b, torch.float32),
          "quantize2original": as_tensor(q2o, torch.int64)}
    with torch.no_grad():
        return pred_decode(net(ep))[0].cpu().numpy()


def candidates_base(g, R_bc, t_bc, table_z=TABLE_Z,
                    min_score=MIN_SCORE, max_tilt_deg=MAX_TILT_DEG):
    """原始输出 → base 系候选行（已按 score / 倾斜 / 高度过滤）。

    GraspNet 的旋转矩阵 ``R[:, 0]`` 是 **approach**（从视角指向物体），
    ``R[:, 1]`` 是夹爪开合方向（width 沿它）。这里只把 approach 转进 base 系
    用来量倾斜；抓取姿态本身这一轮不用（见 precision_plan 的风险说明）。
    """
    rots = g[:, 4:13].reshape(-1, 3, 3)
    centers = g[:, 13:16] @ R_bc.T + t_bc
    rots_b = np.einsum("ij,njk->nik", R_bc, rots)
    approach = rots_b[:, :, 0]
    tilt = np.degrees(np.arccos(np.clip(-approach[:, 2], -1, 1)))
    keep = ((g[:, 0] >= min_score) & (tilt <= max_tilt_deg)
            & (centers[:, 2] > table_z - 0.03) & (centers[:, 2] < table_z + 0.08))
    rows = []
    for i in np.where(keep)[0]:
        rows.append({"i": int(i), "score": float(g[i, 0]),
                     "width_mm": float(g[i, 1] * 1000),
                     "depth_mm": float(g[i, 3] * 1000),
                     "tilt_deg": float(tilt[i]),
                     "xyz_base": [round(float(v), 4) for v in centers[i]],
                     "approach_base": [round(float(v), 3) for v in approach[i]],
                     # 绕 approach 的平面内角：从 R 的第一/二列算，仅记录，不用
                     "angle_deg": round(float(np.degrees(np.arctan2(
                         rots_b[i][1, 1], rots_b[i][0, 1]))), 1)})
    rows.sort(key=lambda r: -r["score"])
    return rows


def select(blocks, cands, anchor_mm=ANCHOR_MM):
    """每块在 ≤ ``anchor_mm`` 的候选里，选**离块心最近**的那个。

    为什么不是「分数最高」：在这个场景尺度上 graspness 的分数**没有校准** ——
    过滤后只剩 11–84 个候选，分数挤在 0.20–0.31；round 144 里全场最高分
    （0.397/0.331）更是落在**平板**上。分数在这里只当**准入门槛**
    （≥ 0.20，见 ``candidates_base``），不参与排序。
    排序键改成「离几何中心的距离」，因为它直接对应用户要的那件事：
    夹爪中心压在块上，而不是压偏。
    距离差 1 mm 以内时再看分数（同一位置的两个候选，取模型更自信的那个）。
    """
    per = []
    for b in blocks:
        near = []
        for c in cands:
            d = float(np.hypot(c["xyz_base"][0] * 1000 - b["cx"],
                               c["xyz_base"][1] * 1000 - b["cy"]))
            if d <= anchor_mm:
                near.append(dict(c, dist_mm=round(d, 2)))
        near.sort(key=lambda c: (round(c["dist_mm"]), -c["score"]))
        per.append({"block_idx": b["idx"], "class": b["class"],
                    "block_xy": [b["cx"], b["cy"]], "n_near": len(near),
                    "best": near[0] if near else None,
                    "dist_mm": near[0]["dist_mm"] if near else None})
    return per


def analyse(frame_dir, run_infer=True, anchor_mm=ANCHOR_MM, seeds=(0, 1, 2),
            min_h_mm=15.0):
    """量块 + 推理 + 锚定。

    ``min_h_mm=15`` 只用来把**不是积木的东西**（8 mm 平板、木板边缘）挡在外面。
    实测积木高 30–33 mm，平板 8.1 mm —— 间隔够宽，不会误伤；被挡掉的仍留在
    ``skipped_low`` 里，不是悄悄丢掉。
    """
    frame = load_frame(frame_dir)
    P, _ = backproject(frame["depth"], frame["K"])
    all_blocks, table_z = find_blocks(frame)
    blocks = [b for b in all_blocks if b["h"] >= min_h_mm]
    skipped = [b for b in all_blocks if b["h"] < min_h_mm]
    for i, b in enumerate(blocks):
        b["idx"] = i
    result = {"frame": frame_dir, "table_z_mm": table_z * 1000,
              "camera_base_xyz_mm": [float(v * 1000) for v in frame["t_bc"]],
              "n_points": int(len(P)), "blocks": blocks,
              "skipped_low": skipped, "min_h_mm": min_h_mm,
              "anchor_mm": anchor_mm, "candidates": [], "per_block": [],
              "chosen": None}
    if not run_infer:
        return result
    # 推理输入：只留**台面那一条 z 带**（台面 −20 mm … +80 mm），x/y 不动。
    #
    # 这一条是实测定的，不是照抄原型。同一帧 live-04 上对比（候选数 / 中位宽度 /
    # 各块最近候选）：
    #   整帧 88 万点        → 11 个候选，宽度中位 49.7 mm，紫块最近 10.1 mm
    #   台面带（56 万点）   → 84 个候选，宽度中位 45.9 mm，黄 2.4 mm / 红 1.2 mm
    #   只留拾取区（4 万点）→ **0 个候选**（网络要靠周边上下文判碰撞，不能裁太狠）
    # 地板占了大半视场，随机采样 20000 点时积木只分到几百个点 —— 不是模型不行，
    # 是输入里几乎没有它。剔除地板后候选数 8 倍、分数 0.20→0.30。
    Pb_all = P @ frame["R_bc"].T + frame["t_bc"]
    crop = (Pb_all[:, 2] > table_z - 0.02) & (Pb_all[:, 2] < table_z + 0.08)
    P_in = P[crop]
    result["n_points_cropped"] = int(crop.sum())
    result["crop"] = "z in [table-20, table+80] mm (base_link), x/y unbounded"
    # 多个采样种子**合并**候选。
    #
    # 为什么：整条链唯一有随机性的地方就是「从 N 个点里随机取 20000 个」。
    # 同一帧 live-04 上换种子，过滤后的候选数在 29~87 之间跳，同一块最近的
    # 候选距离也跳（红块 1.2→5.4 mm，紫块 9.1 mm↔没有）。这是**采样方差**，
    # 不是模型不确定性；多取几组、合并候选，就把这一个方差源压下去了。
    # 输入分布没变（同样 20000 点、同样 voxel、同样阈值），所以与原型仍可比。
    cands, raw_counts = [], []
    for s in seeds:
        g = infer(P_in, seed=s)
        raw_counts.append(int(len(g)))
        for c in candidates_base(g, frame["R_bc"], frame["t_bc"]):
            cands.append(dict(c, seed=int(s)))
    result["seeds"] = list(seeds)
    result["n_raw"] = raw_counts
    result["candidates"] = cands
    result["per_block"] = select(blocks, cands, anchor_mm)
    anchored = [p for p in result["per_block"] if p["best"]]
    if anchored:
        # 默认挑「候选离块心最近」的那一块：它最能代表这一帧的可抓质量。
        # 整轮里由编排脚本按「类别 → 空槽」再决定抓哪一块，不认这个默认值。
        best = min(anchored, key=lambda p: (p["dist_mm"], -p["best"]["score"]))
        result["chosen"] = {"block_idx": best["block_idx"],
                           "class": best["class"], **best["best"]}
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frame")
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-infer", action="store_true")
    ap.add_argument("--anchor-mm", type=float, default=ANCHOR_MM)
    ap.add_argument("--seeds", default="0,1,2",
                    help="采样种子列表（逗号分隔）；候选合并，抵消采样方差")
    ap.add_argument("--block", type=int, default=None, help="只看这一块（0 起）")
    args = ap.parse_args()

    seeds = tuple(int(x) for x in args.seeds.split(",") if x.strip())
    res = analyse(args.frame, run_infer=not args.no_infer,
                  anchor_mm=args.anchor_mm, seeds=seeds)
    print("%s  相机 base_link (%7.1f, %7.1f, %7.1f) mm  台面 z=%+.1f mm  点 %d"
          % (args.frame, *res["camera_base_xyz_mm"], res["table_z_mm"], res["n_points"]))
    print("  块 %d 个：" % len(res["blocks"]))
    for b in res["blocks"]:
        print("    #%d (%7.1f,%7.1f) 高 %4.1f  长x宽 %4.1f x %4.1f  yaw %6.1f  "
              "色 %-11s H %5.1f S %3d 票 %.0f%%"
              % (b["idx"], b["cx"], b["cy"], b["h"], *b["ext"], b["yaw"],
                 b["class"], b["hsv"][0], b["hsv"][1], b["class_share"] * 100))
    if not args.no_infer:
        print("  候选：原始 %s → 过滤后 %d（score≥%.2f、倾斜≤%.0f°，种子 %s）"
              % (res["n_raw"], len(res["candidates"]), MIN_SCORE, MAX_TILT_DEG,
                 ",".join(str(s) for s in res["seeds"])))
        for p in res["per_block"]:
            if args.block is not None and p["block_idx"] != args.block:
                continue
            if p["best"]:
                c = p["best"]
                print("    块 #%d %-11s 有 %2d 个候选，最好 score %.3f  "
                      "w %4.1f mm  距块心 %4.1f mm  xyz(mm) [%7.1f %7.1f %6.1f]  角度 %6.1f°"
                      % (p["block_idx"], p["class"], p["n_near"], c["score"],
                         c["width_mm"], c["dist_mm"],
                         *(v * 1000 for v in c["xyz_base"]), c["angle_deg"]))
            else:
                print("    块 #%d %-11s **没有** ≤%.0f mm 的候选"
                      % (p["block_idx"], p["class"], args.anchor_mm))
    if args.json:
        json.dump(res, open(args.json, "w"), ensure_ascii=False, indent=1)
        print("  → %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
