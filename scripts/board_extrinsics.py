#!/usr/bin/env python3
"""俯视相机外参：**B 路（用臂点一下）** —— 把两块互不相干的观测拼成
`base_link ← camera_link`。

为什么是这么拼的
================
外参要的是 `base_link ← 相机`。而**没有任何一次观测能直接给出它** ——
相机看得见台面但不知道臂在哪，臂摸得到台面但看不见相机。所以拆成两半，
各自用**它最擅长的那件事**去测：

    ① `相机 ← 靶`   —— 靶在台面上，相机拍一张 →  solvePnP。  **不动臂**
    ② `base ← 靶`   —— 臂用 TCP 尖端碰靶上**已知坐标**的点 → FK + 刚体配准。**要动臂**

    合成：  base ← 相机  =  (base ← 靶) ∘ (靶 → 相机)

    设 p_base = R_bb·p_board + t_bb ，  p_cam = R_cb·p_board + t_cb
    ⇒  p_base = (R_bb·R_cbᵀ)·p_cam + (t_bb − R_bb·R_cbᵀ·t_cb)

两支各有一个**可检验的残差**（solvePnP 重投影误差 / 刚体配准残差），
所以结果不是「看起来对」而已。

⚠️ 为什么必须是**两半**，而不是「把靶放在臂基座旁边、用卷尺量」
────────────────────────────────────────────────────────────
因为那种做法最大的误差源**恰恰在卷尺那一步**，而它没有任何残差可查：
量错了不会报错，只会让整片工作区的抓取点**整体平移**，且**越大越偏**
（角度误差会被工作区边缘的力臂放大）。B 路把「量」换成「碰」——
臂自己的 FK 是**毫米级**的，且碰完立刻能算残差。

🔴 前置：**TCP 必须先标定**（示教器上用 4 点法）
─────────────────────────────────────────────
碰出来的点是**法兰上那个具体的尖端**，不是 `tool0`。若 TCP 没标，
这个尖端相对法兰的偏移就是**未知常数**，它会**原样灌进外参**，
而且**看不出来** —— 残差依旧很小（因为所有点带了同一个偏移，
刚体配准会把它吸收掉一部分、剩下的变成整体平移）。

所以顺序是：**示教器标 TCP → 本脚本才允许碰**。
这一条同时解决 D44 的 `tool0_to_fingertip`（那个 0.13 是 Robotiq 2F-85
的推导值，真机是 AG-95，必须按实物重推）—— TCP 标完，指尖位置就已知了。

用法
====
    # ⓪ 不动臂：抓一帧 + 内参（机器转写，省得手抄内参抄错一位）
    python3 scripts/board_extrinsics.py capture --out-dir /tmp/topcam

    # ① 不动臂：解 相机←靶
    python3 scripts/board_extrinsics.py pnp --image /tmp/topcam/frame.png \
        --info /tmp/topcam/info.json --square-mm 20 --cols 9 --rows 12

    # 【中间的动臂环节**不在本脚本里**】碰点由人在示教器上点动完成，
    # 每个点从示教器读出 base 系坐标，手写进 points.json。
    # 为什么不做成子命令：那需要在臂上开一个会**指挥运动**的口子，
    # 而这条路的全部价值恰恰在于「碰」的过程完全由人控制。

    # ② 纯离线：从碰点记录解 base←靶（不碰 ROS、不碰臂）
    python3 scripts/board_extrinsics.py solve-touch --points points.json

    # ③ 合成，打印可直接粘进 real.launch.py 的参数
    python3 scripts/board_extrinsics.py compose \
        --base-from-board base_from_board.json \
        --cam-from-board cam_from_board.json --out extrinsics.json

⚠️ 还有一个**不属于本脚本、但少了它就白标**的步骤：碰点之前必须先看
   `scripts/annotate_target_corners.py` 生成的编号图，确认「①(0,0) 是纸上
   哪个角」。标反 180° 时两处残差**都照样漂亮** —— 理由见那个脚本的文档字符串。

坐标系约定（**容易错，写清楚**）
================================
· **靶系**：原点 = 靶面**左上**(图示方向)那个**外角点**，x 沿列方向、y 沿行方向、
  z 垂直靶面向外。于是**内角点**（OpenCV `patternSize` 数的那种）落在
  `((j+1)·s, (i+1)·s, 0)` —— 比外角点**各进去一整格**。
· **相机系**：solvePnP 走的是 OpenCV 约定（x 右 / y 下 / z 向前）。
· **camera_link**：ROS REP-103（x 前 / y 左 / z 上）。两者差一个固定旋转，
  见 `OPTICAL_TO_LINK`。

⚠️ 输出**必须**是 `base_link ← camera_link`，**不是** `← optical_frame`：
`real.launch.py` 的注释写明了理由 —— 驱动的 TF 是
`camera_link → camera_color_frame → camera_color_optical_frame`，
补父帧只能补在**根** `camera_link` 上；直接发到 optical frame 会给它**两个父帧**。
"""

import argparse
import json
import os
import sys
import time

import numpy as np

# OpenCV 光学系 → ROS REP-103 机体系（camera_link）。
#   x_link =  z_opt（向前 → 向前）
#   y_link = −x_opt（向右 → 取反成向左）
#   z_link = −y_opt（向下 → 取反成向上）
# 行列式 = +1（是纯旋转，不是镜像）—— 自检里钉住了这一条。
OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


# ② 刚体配准残差的闸门（米）。超过它 `solve-touch` **不再往下走**。
#
# 为什么是个**硬闸门**而不是一句 ⚠️：② 的输入是**手抄的六个数字**，抄错一位
# 就会解出一个「合法但错」的 base←靶 —— 残差会大（实测：把④抄成①的重复点，
# 残差 **94 mm**），可它**照样写出 base_from_board.json 并且退出码 0**，
# 后面 `compose` 照单全收，最后那台相机会被放到离基座几米外的地方，
# 而**打印出来的"相机离基座 X m"看着就像一个正常数字**。
# 现场是四条命令连着敲的，一句 ⚠️ 太容易从屏幕上滑过去。
#
# 3 mm 这个值有实测依据（合成复演，见 RESTRUCTURE_PLAN 第六十轮 §七）：
# 示教器读数是 1 mm 分辨率，四个角即使在**零噪声**下也会有 ±0.29 mm 的取整，
# 正常操作下残差落在 0.3~0.8 mm。3 mm ≈ 正常值的 4~10 倍，留足了余量。
TOUCH_RMS_MAX_M = 3e-3


# ────────────────────────────── 纯数学（可离线自检） ──────────────────────────

def rigid_from_correspondences(P_src, P_dst):
    """Kabsch：求把 `P_src` 刚体搬到 `P_dst` 的 (R, t)，即 `p_dst ≈ R·p_src + t`。

    :param P_src: (N,3)，至少 3 个**不共线**的点
    :param P_dst: (N,3)，与 P_src 逐行对应
    :returns: (R (3,3), t (3,), rms 残差)

    ⚠️ 为什么要 SVD 而不是「解个 3×4 的最小二乘」：后者会解出带缩放/剪切的
    仿射变换。而「相机对靶」「基座对靶」两件事**都是刚体**，多出来的自由度
    只会去拟合噪声，并且让结果**不再是一个合法位姿**（旋转矩阵不再正交，
    后面转四元数时会得到非单位四元数）。SVD 那一步（`det` 修正）就是在
    强制「只许旋转」。

    ⚠️ 残差要**打出来看**：它是这条路唯一的质检。残差小 = 点碰得准、
    TCP 标得对；残差大 = 先别用这个外参，回去查前面两步。
    """
    P_src = np.asarray(P_src, dtype=float)
    P_dst = np.asarray(P_dst, dtype=float)
    if P_src.shape != P_dst.shape or P_src.ndim != 2 or P_src.shape[1] != 3:
        raise ValueError('两组点必须都是 (N,3) 且 N 相同，现在是 %s / %s'
                         % (P_src.shape, P_dst.shape))
    if P_src.shape[0] < 3:
        raise ValueError('至少要 3 个点；只有 2 个点的话绕连线的旋转是不定的')

    src_c = P_src.mean(axis=0)
    dst_c = P_dst.mean(axis=0)
    H = (P_src - src_c).T @ (P_dst - dst_c)
    U, _, Vt = np.linalg.svd(H, full_matrices=False)
    R = Vt.T @ U.T
    # det = −1 说明解出了**镜像**（点共线/近似共线时会这样）。
    # 翻转最后一个奇异向量把它掰回真旋转 —— 这是标准做法，不是凑数。
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = dst_c - R @ src_c

    res = P_dst - (P_src @ R.T + t)
    rms = float(np.sqrt((res ** 2).sum(axis=1).mean()))
    return R, t, rms


def quat_to_matrix(q):
    """(x,y,z,w) → 3×3。与 `auto_label_grid.py` 的同名函数保持一致。"""
    x, y, z, w = [float(v) for v in q]
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n <= 0:
        raise ValueError('零四元数')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """3×3 → (x,y,z,w)。用「取最大对角元那个分支」的写法：
    朴素写法在 w≈0（旋转 180°）时会除以接近 0 的数，精度崩掉。"""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0:            # 统一取 w ≥ 0 的那一半，免得同一个位姿有两个写法
        q = -q
    return q


def board_inner_corners(square, cols, rows):
    """靶上**内角点**在靶系里的坐标 (N,3)，z=0。

    :param square: 单格边长 —— **单位随你传入的**，不做任何换算。
        本模块内部**一律用米**（`real.launch.py` 与 TF 都是米）。
        ⚠️ 命令行上的 `--square-mm` 是毫米（量棋盘格就是这么量的），
        所以 `cmd_pnp` 里**先除 1000 再调这里**。
    :param cols: **方格数**（不是内角点数）。内角点数 = (cols-1, rows-1)。
        ⚠️ 与 `measure_board_scale.py` 的 `--cols/--rows` **约定相反** ——
        那边传的是内角点数（OpenCV `patternSize`），A4 那张是 (8, 11)；
        这边传方格数，同一张是 (9, 12)。同一个板子、两组数，别串。

    靶系原点取靶面**外角点**（第一格的角），所以第 (i,j) 个内角点
    落在 `((j+1)·s, (i+1)·s, 0)` —— **各进去一整格**。
    这一格之差要是搞错，外参会整整平移一个格宽（20~30 mm），
    而且**残差依然很小**，看不出来。
    """
    s = float(square)
    pts = []
    for i in range(rows - 1):
        for j in range(cols - 1):
            pts.append([(j + 1) * s, (i + 1) * s, 0.0])
    return np.array(pts, dtype=float)


def board_pattern_corners(square, cols, rows):
    """靶面**外角点**（整个图案的四个角）在靶系里的坐标 —— 碰点用这四个。

    碰这四个而不是内角点：内角点是「四格交点」，尖端压上去时**没有明确的
    落点**（往哪边偏半格都还是看着对）；外角点是**图案的直角尖**，
    尖端顶住它有一个明确的机械止感。

    :param square: 单格边长，**单位随你传入的**（本模块内部一律用米）。
    """
    s = float(square)
    W, H = cols * s, rows * s
    return np.array([[0.0, 0.0, 0.0], [W, 0.0, 0.0],
                     [W, H, 0.0], [0.0, H, 0.0]], dtype=float)


def compose_base_from_camera(R_bb, t_bb, R_cb, t_cb):
    """(base←靶) ∘ (靶→相机) → base←相机。推导见模块头。"""
    R_bc = np.asarray(R_bb, float) @ np.asarray(R_cb, float).T
    t_bc = np.asarray(t_bb, float) - R_bc @ np.asarray(t_cb, float)
    return R_bc, t_bc


def optical_to_link(R_bc, t_bc):
    """把 `base ← 光学系` 换成 `base ← camera_link`。

    为什么必须换：`real.launch.py` 只接受发到 **camera_link** 的父帧
    （理由见该文件注释：驱动 TF 的根是 camera_link，发到 optical frame
    会让它有两个父帧）。solvePnP 给的是光学系，所以这一步不能省 ——
    省了的表现是**相机外参整体转错 90°**，且因为旋转矩阵仍合法，
    **不会有任何报错**。
    """
    R = np.asarray(R_bc, float) @ OPTICAL_TO_LINK.T
    return R, np.asarray(t_bc, float)


# ───────────────────────── 找棋盘格（两条路共用） ──────────────────────────

def adjacent_spacing_min_ratio(grid):
    """相邻角点间距的最小值 / 中位数。用来抓「重复行」这种**沉默**的退化。

    `findChessboardCorners` 在图案只占画面一小块（本仓库俯视相机实测：
    整块 A4 靶只有 ~200×175 px）时会返回一个**末行复制上一行**的网格 ——
    88 个点里有 8 个是假的，**函数照样返回 True**。
    而它的表现恰好躲过「看中位数」这种质检：

        1x 检测：中位 0.271 px（漂亮）  95 分位 6.394  最大 6.981
        3x 检测：中位 0.171 px          95 分位 0.347  最大 0.480

    同一个靶、同一帧图。所以判据必须看**最大值**，而这里这个比值是更早的
    一道闸：重复行的间距≈0，比值会掉到 0.0x；正常检测在 0.7 以上。

    ⚠️ **必须喂给 `cornerSubPix` 之后的结果。** 实测：`findChessboardCorners`
    的**原始**输出里那一行还是分开的（比值 0.75，看着正常），是 `cornerSubPix`
    把幻影行**收敛到真实行上**才塌成 0.00 的。拿未细化的角点来查会**假通过**。
    """
    g = np.asarray(grid, float)                       # (rows, cols, 2)
    d = []
    if g.shape[0] > 1:
        d.append(np.linalg.norm(np.diff(g, axis=0), axis=2).ravel())
    if g.shape[1] > 1:
        d.append(np.linalg.norm(np.diff(g, axis=1), axis=2).ravel())
    d = np.concatenate(d)
    med = float(np.median(d))
    return float(d.min() / med) if med > 0 else 0.0


def _rough_roi(gray, ps_list, pad=0.35, min_pad=30):
    """1x 粗定位靶在画面里的位置，给放大那一轮裁 ROI 用。定位不到就整幅。"""
    import cv2

    h, w = gray.shape[:2]
    for ps in ps_list:
        ok, c = cv2.findChessboardCorners(
            gray, ps,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            c = c.reshape(-1, 2)
            x0, y0 = c.min(0)
            x1, y1 = c.max(0)
            px = max(min_pad, pad * (x1 - x0))
            py = max(min_pad, pad * (y1 - y0))
            return (max(0, int(x0 - px)), max(0, int(y0 - py)),
                    min(w, int(x1 + px)), min(h, int(y1 + py)))
    return (0, 0, w, h)


def _roi_variants(gray, ps_list, scales, roi_pads):
    """产出放大档要试的 `(scale, sub, ox, oy, tag)`，**由紧到松，整幅兜底**。

    1x 一律用整幅（它同时充当「有没有重复行」那一判的输入，必须与旧行为一致）。
    放大档先按 `roi_pads` 里的边距逐个裁，**最后一定再补一档整幅**。

    ⚠️ **为什么必须有这一档兜底**（2026-10-02 实测，一条真踩到的坑）：
    `_rough_roi` 的 `pad=0.35` 只把外接矩形外扩三分之一。同一张靶、同一个位置，
    只因为画面里**别的东西变了**（机械臂吊进来、投在纸上的影子），紧框下
    2x/3x **双双检不出**，而放宽到 0.9 或整幅立刻检出。紧框失败**不报错**，
    症状是「靶明明在画面中央，脚本却说找不到角点」——
    正是这一类里最难查的。`_rough_roi` 本身也缓存，别每个 scale 重算。
    """
    import cv2

    h, w = gray.shape[:2]
    yield 1, gray, 0, 0, '1x 全幅'
    cache = {}
    for scale in scales:
        if scale == 1:
            continue
        boxes = []
        for pad in roi_pads:
            if pad not in cache:
                cache[pad] = _rough_roi(gray, ps_list, pad=pad, min_pad=30)
            boxes.append(cache[pad])
        boxes.append((0, 0, w, h))          # ⭐ 整幅兜底，永远排最后
        seen = set()
        for ox, oy, x1, y1 in boxes:
            if (ox, oy, x1, y1) in seen or x1 <= ox or y1 <= oy:
                continue
            seen.add((ox, oy, x1, y1))
            sub = cv2.resize(gray[oy:y1, ox:x1], None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_CUBIC)
            yield (scale, sub, ox, oy,
                   '%dx %s' % (scale, '全幅' if (ox, oy, x1, y1) == (0, 0, w, h)
                               else 'roi(%d,%d,%d,%d)' % (ox, oy, x1, y1)))


def detect_chessboard(gray, ps_list, scales=(1, 2, 3), min_ratio=0.45, verbose=True,
                      roi_pads=(0.35, 0.90)):
    """逐级放大找棋盘格 → `(corners(原图像素, N,1,2), patternSize, scale)`。

    **为什么要放大**：这不是精度洁癖，是正确性。同一个靶在 1x 下会被检出
    重复行（见 `adjacent_spacing_min_ratio`），解出来的外参在远端差 11 mm。
    放大 3x 后 88 点全部 ≤0.5 px。放大只在 ROI 上做，代价可以忽略。

    **为什么要放宽 ROI**：见 `_roi_variants` ——「紧框检不出、松框检得出」
    是实测踩过的，且**不报错**。失败后自动放宽，最后退到整幅。

    :param ps_list: 要试的 `patternSize`，**两种朝向都该传** ——
        `findChessboardCorners` 对非正方图案不是旋转不变的。
    :param min_ratio: 相邻间距比低于它就判为退化，换更大倍数重试。
    :param roi_pads: 放大档的 ROI 边距，**由紧到松**；整幅永远最后兜底。
    :return: 找不到时 `(None, None, None)`。
    """
    import cv2

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
    for scale, sub, ox, oy, tag in _roi_variants(gray, ps_list, scales, roi_pads):
        for ps in ps_list:
            ok, c = cv2.findChessboardCorners(
                sub, ps,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                continue
            c = cv2.cornerSubPix(sub, c, (11, 11), (-1, -1), crit)
            grid = c.reshape(ps[1], ps[0], 2)
            ratio = adjacent_spacing_min_ratio(grid)
            if ratio < min_ratio:
                if verbose:
                    print('  ⚠️ %s 检出**重复行**（相邻间距比 %.2f < %.2f），'
                          '放宽重试 —— 这份结果别用' % (tag, ratio, min_ratio))
                continue
            pts = grid.reshape(-1, 2) / scale + np.array([ox, oy], float)
            if verbose and tag != '1x 全幅':
                print('  ✓ %s 检出 %d 个内角点（间距比 %.2f）'
                      % (tag, len(pts), ratio))
            return pts.reshape(-1, 1, 2).astype(np.float32), ps, scale
    return None, None, None


# ────────────────────────────── 两个前端（要数据） ────────────────────────────

def cmd_capture(args):
    """⓪ 抓一帧俯视相机图 + 内参 → `pnp` 要的 `frame.png` + `info.json`。

    **只订阅，不发布；不动臂。**在板子上跑（要 ROS 环境）。

    为什么单独一步：`pnp` 要的 `--info` 是 `{"K":..., "D":...}`，
    而 ROS 那边给的是 `sensor_msgs/CameraInfo`；中间这个转换以前没人做，
    现场就得手抄内参 —— 抄错一位数，外参会整体歪掉而重投影照样好看。
    所以这里让它**机器转、机器写**。

    默认话题是 `/camera/color/...`，对应 `config/realsense_camera.yaml` 里
    **钉死的那台俯视相机**（SDK 号 `<topcam-serial>`）。腕部那台走的是同一组
    话题名，所以**换相机等于换整个 config，别只改这里的话题名**。
    """
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class Cam(Node):
        def __init__(self):
            super().__init__('board_extrinsics_capture')
            self.info = self.color = None
            self.seen = set()
            self.create_subscription(CameraInfo, args.info_topic, self._i,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, args.color_topic, self._c,
                                     qos_profile_sensor_data)

        def _note(self, name):
            # 第一帧就报一声：真出问题时要能分清「全都没来」和「只缺一路」——
            # 这两者的查法完全不同（前者多办是 DDS 发现，后者是那一路的话题）。
            if name not in self.seen:
                self.seen.add(name)
                print('  ← 收到第一帧 %s' % name, flush=True)

        def _i(self, m):
            self.info = m
            self._note('camera_info')

        def _c(self, m):
            if self.color is None:
                self.color = m
            self._note('color')

    rclpy.init()
    node = Cam()
    try:
        t0 = time.time()
        while time.time() - t0 < args.timeout and not (
                node.info is not None and node.color is not None):
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.info is None or node.color is None:
            raise SystemExit('没收到帧：camera_info=%s color=%s（等 %.0f s）'
                             % (node.info is not None, node.color is not None,
                                args.timeout))

        msg = node.color
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = np.ascontiguousarray(a[:, :, ::-1])
        elif msg.encoding == 'bgr8':
            img = np.ascontiguousarray(a[:, :, :3])
        else:
            raise SystemExit('不认识的编码 %r（只处理 rgb8 / bgr8）' % msg.encoding)

        os.makedirs(args.out_dir, exist_ok=True)
        img_path = os.path.join(args.out_dir, 'frame.png')
        info_path = os.path.join(args.out_dir, 'info.json')
        cv2.imwrite(img_path, img)

        k = np.array(node.info.k, float).reshape(3, 3)
        d = np.array(node.info.d, float).ravel()
        frame = node.info.header.frame_id
        with open(info_path, 'w') as fh:
            json.dump({'K': k.tolist(), 'D': d.tolist(),
                       'width': int(node.info.width),
                       'height': int(node.info.height),
                       'frame_id': frame,
                       'color_topic': args.color_topic,
                       'info_topic': args.info_topic}, fh, indent=2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('图   %s  (%dx%d, %s)' % (img_path, msg.width, msg.height, msg.encoding))
    print('内参 %s' % info_path)
    print('  fx=%.2f fy=%.2f cx=%.2f cy=%.2f' % (k[0, 0], k[1, 1], k[0, 2], k[1, 2]))
    print('  畸变 D = %s' % np.array2string(d, precision=5))
    print('  帧 %s   ← 记下它：这份外参是相对**这个**光学帧的' % frame)
    print()
    print('下一步（不动臂）：')
    print('  python3 scripts/board_extrinsics.py pnp --image %s --info %s \\'
          % (img_path, info_path))
    print('      --square-mm 20 --cols 9 --rows 12')


def cmd_pnp(args):
    """① `相机 ← 靶`：一张靶图 + 内参 → solvePnP。**不动臂。**"""
    import cv2

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit('读不到图：%s' % args.image)
    with open(args.info) as fh:
        info = json.load(fh)
    K = np.array(info['K'], float).reshape(3, 3)
    dist = np.array(info.get('D', [0, 0, 0, 0, 0]), float).reshape(-1, 1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ⚠️ 两种朝向都试 —— 非正方图案的 findChessboardCorners **不是旋转不变的**，
    #    腕部/俯视相机都可能侧装，画面里的靶可能是横的也可能是竖的（实测踩过）。
    #    `detect_chessboard` 还会**逐级放大**：小靶在 1x 下会检出重复行。
    corners, found, scale = detect_chessboard(
        gray, ((args.cols - 1, args.rows - 1), (args.rows - 1, args.cols - 1)))
    if found is None:
        raise SystemExit('没找到内角点（试过 (%d,%d) 与 (%d,%d)，放大到 3x）'
                         % (args.cols - 1, args.rows - 1,
                            args.rows - 1, args.cols - 1))

    # 图案在画面里转了 90° 时，内角点的行列是互换的，物体点也要跟着换，
    # 否则会把靶的长短边对调 —— 外参会错得离谱但残差依旧好看（对称图案尤其）。
    c_eff, r_eff = found[0] + 1, found[1] + 1
    # ⚠️ 物体点**一律用米**。`square_mm` 只在命令行上是毫米（量棋盘格就是这么量的），
    #    进 solvePnP 之前先除 1000 —— 否则 tvec 出来是毫米，
    #    而 `solve-touch` 那边是拿米在算，`compose` 把两者一混就得到
    #    「相机离基座 730 米」这种数（实测踩过，见 compose 里那道自检）。
    objp = board_inner_corners(args.square_mm / 1000.0, c_eff, r_eff)
    imgp = corners.reshape(-1, 2).astype(float)
    if len(objp) != len(imgp):
        raise SystemExit('角点数对不上：物体点 %d / 图像点 %d'
                         % (len(objp), len(imgp)))

    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise SystemExit('solvePnP 失败')
    R_cb, _ = cv2.Rodrigues(rvec)
    t_cb = tvec.reshape(3)

    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - imgp, axis=1)
    print('内角点 %d 个（patternSize=%s，检测放大 %dx）' % (len(objp), found, scale))
    print('重投影误差：中位 %.3f px，95 分位 %.3f px，最大 %.3f px'
          % (np.median(err), np.percentile(err, 95), err.max()))
    # ⚠️ 判据看**最大值**，不看中位数。重复行那种退化中位数照样 0.27 px
    #    （实测），只有最大值会露馅 —— 88 个点里坏 8 个，中位数根本不动。
    if err.max() > 1.0:
        print('❌ 最大 %.2f px > 1 px —— **别用这份外参**。先查靶平不平、图糊不糊、'
              '有没有反光；再怀疑检测本身。' % err.max())
    elif np.percentile(err, 95) > 1.0:
        print('❌ 95 分位 %.2f px > 1 px —— 有一片点没对上，同样别用。'
              % np.percentile(err, 95))
    else:
        print('✅ 全部内角点 ≤ %.2f px。' % err.max())
    doc = {'R': R_cb.tolist(), 't': t_cb.tolist(),
           'unit': 'm',
           'reproj_median_px': float(np.median(err)),
           'reproj_max_px': float(err.max()),
           'detect_scale': int(scale),
           'square_mm': args.square_mm, 'cols': c_eff, 'rows': r_eff}
    with open(args.out, 'w') as fh:
        json.dump(doc, fh, indent=2)
    print('→ %s' % os.path.abspath(args.out))


def cmd_solve_touch(args):
    """② `base ← 靶`：从「碰点记录」解刚体。**纯离线**，不碰 ROS、不指挥运动。

    记录格式（`points` 里每个元素是一组对应：靶系已知点 ↔ base 系实测点）：
        {"R_board_pts": [[x,y,0],...], "P_base_pts": [[x,y,z],...]}
    碰点那一步（读 /joint_states + 调 /compute_fk）由现场单独做，
    本脚本只负责**解**——这样解算这部分今晚就能用合成数据验完。
    """
    with open(args.points) as fh:
        doc = json.load(fh)
    # 单位必须**说清楚**：记录碰点时用毫米是人之常情（量靶就是这么量的），
    # 而后面全是米。这个差别一旦漏掉，解出来的靶→基座会整体差 1000 倍，
    # **残差却依旧只有零点几毫米**（刚体配准吸收得掉），看起来完全正常。
    unit = doc.get('unit', args.unit)
    if unit not in ('m', 'mm'):
        raise SystemExit('unit 只能是 m 或 mm，拿到 %r' % unit)
    if 'unit' not in doc:
        print('⚠️ %s 里没写 unit，按 --unit %s 处理（默认值，别靠这个）'
              % (args.points, unit))
    scale = 1e-3 if unit == 'mm' else 1.0
    # 两组点各自解析 + 报出**是哪一行**填坏了。这一步不是洁癖：记录是人手抄的，
    # 最常见的错就是**有一个点忘了改**（留着模板里的占位符，或复制了上一行）。
    # 不拦的话 `np.array` 要么抛一句与坐标毫无关系的 `could not convert string
    # to float: 'x'`，要么把 null 变成 NaN 一路传到 SVD —— 现场没人能从
    # 「SVD did not converge」反推出「④那一行没填」。
    def _pts(key):
        rows = doc.get(key)
        if not isinstance(rows, list):
            raise SystemExit('%s 里缺 %r（或它不是个数组）' % (args.points, key))
        out = []
        for i, row in enumerate(rows):
            try:
                v = [float(x) for x in row]
            except (TypeError, ValueError):
                raise SystemExit(
                    '%s 的 %s[%d] = %r 不是三个数字 —— **这一行没填**？'
                    '\n  （模板里的占位符要换成示教器上读到的 Base 坐标，单位 %s）'
                    % (args.points, key, i, row, unit))
            if len(v) != 3:
                raise SystemExit('%s 的 %s[%d] 有 %d 个数，应该是 3 个'
                                 % (args.points, key, i, len(v)))
            if not all(np.isfinite(v)):
                raise SystemExit('%s 的 %s[%d] = %r 里含 nan/inf'
                                 % (args.points, key, i, row))
            out.append(v)
        return np.array(out, float) * scale

    P_board = _pts('R_board_pts')
    P_base = _pts('P_base_pts')
    R_bb, t_bb, rms = rigid_from_correspondences(P_board, P_base)
    print('碰了 %d 个点（记录单位 %s），刚体配准残差 RMS = %.2f mm'
          % (len(P_board), unit, rms * 1e3))
    out = {'R': R_bb.tolist(), 't': t_bb.tolist(), 'unit': 'm', 'rms_m': rms,
           'n_points': int(len(P_board)), 'src_unit': unit}
    if rms > TOUCH_RMS_MAX_M:
        # 文件照写（要能回头看是哪几个点解歪的），但**退出码非 0** ——
        # 这样 `solve-touch && compose` 这条链会**停在原地**，
        # 不会拿着一个错的 base←靶继续往下合成。
        with open(args.out, 'w') as fh:
            json.dump(out, fh, indent=2)
        print('❌ 残差 %.1f mm > %.0f mm —— **这份解不能用**，已写出的 %s 只当草稿看。'
              % (rms * 1e3, TOUCH_RMS_MAX_M * 1e3, os.path.abspath(args.out)))
        print('   按顺序查三样：')
        print('   ① 示教器上 TCP 标了吗（没标的话尖端偏移会原样灌进来）')
        print('   ② 碰的时候是不是顶住了同一个物理点（尖端有没有侧滑）')
        print('   ③ 靶在碰的过程中有没有被推动')
        print('   ④ 六个数是不是抄串行了 —— 这是最常见的一种')
        raise SystemExit(2)
    print('✅ 残差在闸门内（%.0f mm）。' % (TOUCH_RMS_MAX_M * 1e3))
    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=2)
    print('→ %s' % os.path.abspath(args.out))


def cmd_compose(args):
    """③ 合成 → `base_link ← camera_link`，并打印可直接粘进 launch 的参数。"""
    with open(args.base_from_board) as fh:
        b = json.load(fh)
    with open(args.cam_from_board) as fh:
        c = json.load(fh)

    # ⚠️ 两个输入必须**都是米**，而且要说出来。这一关是补上的：
    # 原先 `pnp` 写的是毫米、`solve-touch` 按米算，这里直接混合，
    # 结果打印出「相机离基座 730 米」而**没有任何一步报错** ——
    # 因为 1000 倍只是同一套数学上的一个尺度，合成出来的仍是合法位姿。
    # 合成结果再自检一次（下面那段），因为「两个输入单位都写错且写成了同一个」
    # 是这一关拦不住的。
    bad = []
    for tag, d, path in (('base←靶', b, args.base_from_board),
                         ('相机←靶', c, args.cam_from_board)):
        if d.get('unit', 'm') != 'm':
            bad.append('%s (%s) 的单位是 %r，不是 m' % (tag, path, d['unit']))
    if bad:
        raise SystemExit('单位不一致，停下来：\n  ' + '\n  '.join(bad) +
                         '\n（现在这一版所有中间文件都统一用米；'
                         '`solve-touch` 加 --unit mm 可以把毫米记录转进来）')

    # ② 的残差闸门在这里**再过一次**，不能只靠 `solve-touch` 那一道。
    # 理由：那个残差是**手抄坐标**唯一的质检，而它一旦被忽略（旧版只打一句 ⚠️、
    # 退出码仍是 0），合成出来的位姿照样「合法」，最后只是相机被放到几米外，
    # 而打印行看着完全正常。闸门要放在**生成外参的这一步**才算真拦住。
    # （`solve-touch` 直接跑会被自己的闸门挡住；但 base_from_board.json 也可能
    #  是手改的、或是别的工具产出的 —— 所以这里按文件里的 rms_m 字段再判一次。）
    rms_b = b.get('rms_m')
    if rms_b is not None and float(rms_b) > TOUCH_RMS_MAX_M:
        raise SystemExit(
            '② 的刚体配准残差 %.1f mm > %.0f mm（%s），停下来：\n'
            '  这份 base←靶 是歪的，合成出来的外参会把相机放到错的地方。\n'
            '  重碰一遍那几个点，别拿它凑合。'
            % (float(rms_b) * 1e3, TOUCH_RMS_MAX_M * 1e3, args.base_from_board))

    R_bc, t_bc = compose_base_from_camera(
        b['R'], b['t'], c['R'], c['t'])
    R_link, t_link = optical_to_link(R_bc, t_bc)
    q = matrix_to_quat(R_link)

    # 几何自检：这是**唯一**能拦住「两个输入一起错 1000 倍」的一关。
    # 判据取得很宽 —— 只挡物理上不可能的，不挡「装得偏」。
    dist_xy = float(np.linalg.norm(t_link[:2]))
    if not (-1.0 < t_link[2] < 5.0) or dist_xy > 5.0:
        print('⚠️⚠️ 合成结果**物理上讲不通**，别用：')
        print('     相机离基座 %.3f（水平）%.3f m（高）' % (dist_xy, t_link[2]))
        print('     一台相机不该离基座几米以上，也不该在基座下方 1 m 以下。')
        print('     最可能是**单位差 1000 倍**（毫米当成米了）。先查两个输入的 unit。')

    print('base_link ← 相机光学系   t = [%.4f %.4f %.4f] m' % tuple(t_bc))
    print('base_link ← camera_link  t = [%.4f %.4f %.4f] m' % tuple(t_link))
    print('                          q = [%.5f %.5f %.5f %.5f]  (x y z w)'
          % tuple(q))
    # 相机在基座上方多高、离基座多远 —— 一眼能判「离谱不离谱」
    print('  ⇒ 相机离基座 %.3f m，高 %.3f m' % (dist_xy, t_link[2]))

    doc = {'R': R_link.tolist(), 't': t_link.tolist(), 'q': q.tolist(),
           'unit': 'm',
           'frame_parent': 'base_link', 'frame_child': 'camera_link',
           'rms_touch_m': b.get('rms_m'), 'reproj_px': c.get('reproj_median_px')}
    with open(args.out, 'w') as fh:
        json.dump(doc, fh, indent=2)
    print()
    print('喂给 real.launch.py（camera_link 的父帧就是它，别再换帧）：')
    print('  publish_camera_tf:=true \\')
    print('  camera_x:=%.4f camera_y:=%.4f camera_z:=%.4f \\' % tuple(t_link))
    print('  camera_qx:=%.5f camera_qy:=%.5f camera_qz:=%.5f camera_qw:=%.5f'
          % tuple(q))
    print('→ %s' % os.path.abspath(args.out))


def main():
    ap = argparse.ArgumentParser(
        description='俯视相机外参（B 路：用臂点一下）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p0 = sub.add_parser('capture', help='⓪ 抓一帧俯视图 + 内参（不动臂，需 ROS）')
    p0.add_argument('--out-dir', default='/tmp/topcam')
    p0.add_argument('--color-topic', default='/camera/color/image_raw')
    p0.add_argument('--info-topic', default='/camera/color/camera_info')
    p0.add_argument('--timeout', type=float, default=25.0)
    p0.set_defaults(func=cmd_capture)

    p1 = sub.add_parser('pnp', help='① 相机←靶（不动臂）')
    p1.add_argument('--image', required=True)
    p1.add_argument('--info', required=True, help='含 {"K": [[...]], "D": [...]}')
    p1.add_argument('--square-mm', type=float, default=20.0)
    p1.add_argument('--cols', type=int, default=9,
                    help='**方格数**（不是内角点数；A4 那张 9×12 格就传 9）')
    p1.add_argument('--rows', type=int, default=12, help='方格数（同上）')
    p1.add_argument('--out', default='cam_from_board.json')
    p1.set_defaults(func=cmd_pnp)

    p2 = sub.add_parser('solve-touch', help='② base←靶（纯离线解算）')
    p2.add_argument('--points', required=True,
                    help='{"R_board_pts": [...], "P_base_pts": [...], "unit": "m"}')
    p2.add_argument('--unit', choices=('m', 'mm'), default='m',
                    help='记录里没写 unit 时按这个解释（写进文件里最保险）')
    p2.add_argument('--out', default='base_from_board.json')
    p2.set_defaults(func=cmd_solve_touch)

    p3 = sub.add_parser('compose', help='③ 合成 → base_link←camera_link')
    p3.add_argument('--base-from-board', required=True)
    p3.add_argument('--cam-from-board', required=True)
    p3.add_argument('--out', default='extrinsics.json')
    p3.set_defaults(func=cmd_compose)

    a = ap.parse_args()
    a.func(a)


if __name__ == '__main__':
    main()
