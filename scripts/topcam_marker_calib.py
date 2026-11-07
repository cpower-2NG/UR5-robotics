#!/usr/bin/env python3
"""俯视相机的外参**重算**工具：一张图，不动机械臂。

背景
────
俯视相机装在三角架上，外参会缓慢变化（实测一天约 0.945°），而旧判据无法察觉。
碰点法重标一次要动机械臂、要摆靶纸，代价高，所以现实中没人愿意勤做 ——
结果是整条链路长期跑在一份悄悄过期了的外参上。

本工具把重算的代价降到「拍一张图」：工作台上常驻一块 ChArUco 板
（`make_charuco_board.py` 生成），它相对台面固定不动，于是：

    一次性（--init）：
        用**一份已知可信的**外参 `base←cam` 拍一张 → 检出 `cam←板`
        ⇒ `base←板` = `base←cam` ∘ `cam←板`      （存盘，之后一直用它）

    每次（--solve）：
        拍一张 → 检出 `cam←板`
        ⇒ `base←cam` = `base←板` ∘ (`cam←板`)^-1   （一张图，约 1 秒）

⇒ 相机漂多少都能重算，不再需要判断「外参过期没有」。

``--image`` 也可以给 ``precision_capture --burst`` 的帧目录：逐帧检测后做
SE(3) 融合。**基准侧必须融合**——单帧基准的角噪声会经杠杆臂放大成相机平移
（0.087° × 0.934 m ≈ 1.4 mm），门就永远差一点（2026-11-07 实测：单帧基准
1.43 mm，独立两半各 10 帧融合后 0.08 mm）。

用法
────
    # ① 一次性：用当前可信的外参，把「板在 base 系里的位姿」定下来
    python3 scripts/grab_frames.py /tmp/mk 1
    python3 scripts/topcam_marker_calib.py --init \\
        --image /tmp/mk/img_0000.png --info /tmp/mk/info.json \\
        --board docs/calib/charuco_A4_30mm_5x5.json \\
        --extrinsics docs/calib/extrinsics-2026-10-02.json \\
        --store ~/topcam_marker.json

    # ② 每次要用之前：重算，并把启动参数打出来
    python3 scripts/grab_frames.py /tmp/mk2 1
    python3 scripts/topcam_marker_calib.py --solve \\
        --image /tmp/mk2/img_0000.png --info /tmp/mk2/info.json \\
        --store ~/topcam_marker.json

⚠️ 两条前提：
   · `--init` 用的那份外参必须是**当前可信**的 —— 它是这条链的基准。
     基准错了，之后每次重算都会忠实地复现同一个错。
   · 打印出来的板必须按 100% 出，且用图上那条 100 mm 校验尺量过；
     被整体缩放时，把量出的比例乘到 `--square-mm/--marker-mm` 上
     （板 JSON 里有原始值）。
"""

import argparse
import datetime
import glob
import json
import math
import os
import re
import sys

sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

#: `camera_link` → 相机光学系的固定旋转（REP-103/105）：
#:   opt_x = −link_y, opt_y = −link_z, opt_z = link_x
R_LINK_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])

#: 理想旋转只是**兜底**。驱动实测的同一条链（从帧的 tf.json 取）与之差约 15 mm，
#: 所以真机上一律用 ``--tf-json``，别让理想值参与联合标定。
WRIST_LINK_TO_OPTICAL_KEYS = (
    'camera_link:camera_color_frame',
    'camera_color_frame:camera_color_optical_frame')
TOPCAM_LINK_TO_OPTICAL_KEYS = (
    'topcam_link:topcam_color_frame',
    'topcam_color_frame:topcam_color_optical_frame')

WRIST_LINK = 'camera_link'
TOPCAM_LINK = 'topcam_link'
OPTICAL_FRAMES = ('camera_color_optical_frame', 'camera_optical_frame',
                  'topcam_color_optical_frame', 'topcam_optical_frame')


def quat_to_matrix(q):
    """``[x, y, z, w]`` → 3×3。"""
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('四元数长度为 0')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """3×3 → ``[x, y, z, w]``。数值稳定的分支写法（不靠 trace>0 一条路）。"""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def compose(A, B):
    """``A ∘ B``：先施加 B 再施加 A。每个元素是 ``(R, t)``。"""
    Ra, ta = A
    Rb, tb = B
    return (np.asarray(Ra) @ np.asarray(Rb),
            np.asarray(Ra) @ np.asarray(tb) + np.asarray(ta))


def invert(A):
    """``(R, t)`` 的逆。``R`` 必须是正交阵（本工具里都是）。"""
    R, t = A
    Rt = np.asarray(R, dtype=float).T
    return (Rt, -Rt @ np.asarray(t, dtype=float))


def _mean_pose(Ts):
    """一组 4×4 的 SE(3) 平均：平移取均值，旋转取弦距离最小（SVD 投影）。

    与 ``stage4_joint_calib.mean_pose`` 同一公式。这里不 import 它：那个模块
    反过来 import 本模块，顶层互相 import 会成环。
    """
    Ts = [np.asarray(T, dtype=np.float64) for T in Ts]
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(np.sum([T[:3, :3] for T in Ts], axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rodrigues_to_matrix(rvec):
    """OpenCV 的旋转向量 → 3×3。"""
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return R


def load_camera(info_path):
    """内参文件 → ``(K, dist, (w, h))``。两种写法都认：

    · ``grab_frames.py`` 的 info.json：``fx/fy/cx/cy`` + ``D``；
    · ``precision_capture.py`` 的 meta.json：ROS ``camera_info``（``K``/``D``）。

    认第二种是为了 E2/E3b 每一轮直接用刚抓的帧，不必再手工转一份 info.json ——
    手转的那一步迟早会转成上一帧的内参。
    """
    info = json.load(open(info_path, encoding='utf-8'))
    if 'camera_info' in info:
        info = info['camera_info']
        K = np.array(info['K'], dtype=float).reshape(3, 3)
    else:
        K = np.array([[info['fx'], 0.0, info['cx']],
                      [0.0, info['fy'], info['cy']],
                      [0.0, 0.0, 1.0]], dtype=float)
    dist = np.array(info.get('D') or [0.0] * 5, dtype=float).reshape(-1, 1)
    return K, dist, (int(info['width']), int(info['height']))


def _consistent_board_markers(board, corners, ids, K, dist=None,
                              inlier_px=3.0):
    """一个 ID 只留一条检出：把同字典**别的板子**的检出剔掉。

    2026-11-07 真机俯视实测：垫子页 A 也是 ``DICT_4X4_50``、ID 0–9 与 ChArUco
    板撞车，同一张图里两簇并存（板簇 x≈1207–1378/y≈645–769，垫簇
    x≈727–869/y≈378–402）。``interpolateCornersCharuco`` 按 ID 配对，
    谁先谁后决定它拿哪一簇 —— 检出顺序一变，解出的外参就静默错掉
    （实测同一场景一帧 RMS 0.28 px、另一帧 53 px）。

    判据与 ``stage2_pad._consistent_markers`` 同一思路，但用**两块标记的 8 个
    角点**估一张单应（16 个方程对 8 个自由度，超定；只用一块标记的 4 点是
    恰好定解，噪声会被放大到十几像素），再数内点；选出最优簇后用全部内点
    重拟合两轮。没有重复 ID ⇒ 原样返回（旧行为）；一簇都拟合不上 ⇒ 也原样
    交出去，让上层按 RMS 判红（fail-closed，不在这里假装挑对了）。
    """
    import cv2

    ids = np.asarray(ids).reshape(-1)
    id_index = {int(v): i for i, v in enumerate(np.asarray(board.ids).ravel())}
    dets = []
    for c, i in zip(corners, ids):
        mid = int(i)
        if mid not in id_index:
            continue
        dets.append({'id': mid,
                     'img': np.asarray(c, dtype=np.float64).reshape(4, 2),
                     'obj': np.asarray(board.objPoints[id_index[mid]],
                                       dtype=np.float64).reshape(4, 3)[:, :2]})
    if len(dets) < 2 or len({d['id'] for d in dets}) == len(dets):
        return list(corners), ids

    def _fit(pairs):
        obj = np.vstack([d['obj'] for d in pairs]).astype(np.float32)
        img = np.vstack([d['img'] for d in pairs]).astype(np.float32)
        H, _ = cv2.findHomography(obj.reshape(-1, 1, 2),
                                  img.reshape(-1, 1, 2), 0)
        return H

    def _inliers(H):
        inl = []
        for d in dets:
            uv = cv2.perspectiveTransform(
                d['obj'].astype(np.float32).reshape(-1, 1, 2), H).reshape(4, 2)
            err = float(np.sqrt(np.mean(np.sum((uv - d['img']) ** 2,
                                               axis=1))))
            if err <= inlier_px:
                inl.append((d, err))
        return inl

    best = None
    for i in range(len(dets)):
        for j in range(i + 1, len(dets)):
            if dets[i]['id'] == dets[j]['id']:
                continue
            H = _fit([dets[i], dets[j]])
            if H is None:
                continue
            inl = _inliers(H)
            if len(inl) < 2:
                continue
            rms = float(np.sqrt(np.mean([e * e for _, e in inl])))
            score = (len(inl), -rms)
            if best is None or score > best[0]:
                best = (score, inl)
    if best is None:
        return list(corners), ids
    inl = best[1]
    for _ in range(2):                       # 用全部内点重拟合，去掉抽样偏差
        H = _fit([d for d, _ in inl])
        if H is None:
            break
        refit = _inliers(H)
        if len(refit) <= len(inl):
            break
        inl = refit
    picked = {}
    for d, err in inl:
        if d['id'] not in picked or err < picked[d['id']][1]:
            picked[d['id']] = (d, err)
    out_c = [d['img'].reshape(1, 4, 2).astype(np.float32)
             for d, _ in picked.values()]
    out_i = np.array([[mid] for mid in picked], dtype=np.int32)
    return out_c, out_i


def detect_board(image_path, board_meta, K, dist):
    """检出棋盘格内角点 → ``(cam←board 的 (R,t), 用到的角点数, 重投影 RMS px)``。

    返回 ``None`` 表示这张图上没检出足够的角点。
    """
    import cv2
    import cv2.aruco as aruco
    dict_id = getattr(aruco, board_meta['dictionary'])
    dictionary = aruco.Dictionary_get(dict_id)
    s_m = float(board_meta['square_mm']) / 1000.0
    m_m = float(board_meta['marker_mm']) / 1000.0
    board = aruco.CharucoBoard_create(int(board_meta['squares_x']),
                                      int(board_meta['squares_y']),
                                      s_m, m_m, dictionary)

    img = cv2.imread(image_path)
    if img is None:
        raise SystemExit(f'读不到图片：{image_path}')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) == 0:
        return None
    corners, ids = _consistent_board_markers(board, corners, ids, K, dist)
    n, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
        corners, ids, gray, board)
    if ch_corners is None or n < 6:
        return None
    ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, board, K, dist, None, None)
    if not ok:
        return None
    # 重投影 RMS：把板上的物点投回像素，与检出的角点比 —— 这是**可检验的残差**
    obj = board.chessboardCorners[ch_ids.reshape(-1)].reshape(-1, 3)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - ch_corners.reshape(-1, 2), axis=1)
    return (rodrigues_to_matrix(rvec),
            np.asarray(tvec, dtype=float).reshape(3)), int(n), float(np.sqrt((err ** 2).mean()))


def chain_from_tf(doc, keys):
    """``tf.json`` → 整条链的 ``(R, t)``。

    ``keys`` 按 ``父:子`` 的顺序给；结果 = 第一段 ∘ 第二段 ∘ …（即父←末端）。
    缺任何一段就停 —— 少存一段时宁可不标，不拿理想值顶替。
    """
    ok = (doc or {}).get('ok') or {}
    acc = (np.eye(3), np.zeros(3))
    for key in keys:
        if key not in ok:
            raise SystemExit('tf.json 缺 %s（有的键：%s）'
                             % (key, ', '.join(sorted(ok)) or '无'))
        e = ok[key]
        tr, ro = e['translation'], e['rotation']
        acc = compose(acc, (quat_to_matrix([ro['x'], ro['y'], ro['z'], ro['w']]),
                            np.array([tr['x'], tr['y'], tr['z']],
                                     dtype=float)))
    return acc


def _link_to_optical(tf_doc, keys):
    """有 tf.json 就用驱动实测链，没有才退回理想旋转（并在调用处标明来源）。"""
    if tf_doc is None:
        return (R_LINK_TO_OPTICAL.copy(), np.zeros(3))
    return chain_from_tf(tf_doc, keys)


def base_from_camera(doc, link_to_optical=None):
    """外参文件 → ``base←相机光学系`` 的 ``(R, t)``。

    接受两种 `frame_child`：已是光学系（原样用），或 ``*_link``（左乘
    link←optical）。``link_to_optical`` 给定时用它（驱动实测），否则用理想旋转。
    其它帧名一律拒绝 —— 猜帧关系是这一类错误里最难查的一种。
    """
    child = str(doc.get('frame_child') or doc.get('camera_frame') or '')
    R = (np.array(doc['R'], dtype=float) if 'R' in doc
         else quat_to_matrix(doc['q'] if 'q' in doc else doc['rotation']))
    t = np.array([float(v) for v in (doc.get('t') or doc['translation'])], dtype=float)
    if child in OPTICAL_FRAMES:
        return (R, t)
    if child in (WRIST_LINK, TOPCAM_LINK):
        return compose((R, t), link_to_optical or
                       (R_LINK_TO_OPTICAL, np.zeros(3)))
    raise SystemExit(f'不认识的 frame_child={child!r}：只认 '
                     f'{" / ".join((WRIST_LINK, TOPCAM_LINK) + OPTICAL_FRAMES)}'
                     f'（拒绝猜）')


def fmt_launch(R, t, prefix='camera'):
    """``base_link←相机 link`` → launch 静态 TF 的那一串启动参数。

    ``prefix='camera'`` 是腕部那条（``publish_camera_tf``），``'topcam'``
    是俯视那条（``topcam_publish_tf``）。注意传进来的必须是 **link** 系，
    不是光学系——驱动自己再发 link→optical。
    """
    q = matrix_to_quat(R)
    return (f'publish_{prefix}_tf:=true \\\n'
            f'  {prefix}_x:={t[0]:.6f}  {prefix}_y:={t[1]:.6f}  '
            f'{prefix}_z:={t[2]:.6f} \\\n'
            f'  {prefix}_qx:={q[0]:.6f} {prefix}_qy:={q[1]:.6f} '
            f'{prefix}_qz:={q[2]:.6f} {prefix}_qw:={q[3]:.6f}')


def main():
    ap = argparse.ArgumentParser(
        description='俯视相机外参重算：一张 ChArUco 图 / 一个 burst 帧目录，'
                    '不动机械臂')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--init', action='store_true',
                   help='用当前可信的外参把「板在 base 系里的位姿」定下来并存盘')
    g.add_argument('--solve', action='store_true',
                   help='由存下的板位姿重算 base←cam，并打印启动参数')
    ap.add_argument('--image', required=True,
                    help='单帧 png，或 precision_capture --burst 的帧目录'
                         '（目录模式逐帧检测后融合）')
    ap.add_argument('--info', default=None,
                    help='单帧模式的 info.json（内参）；帧目录模式逐帧读 meta_NNN.json')
    ap.add_argument('--tf-json', default=None,
                    help='同一次采集的 tf.json：用驱动实测的 link→optical 链'
                         '（理想旋转与实测差约 15 mm，真机必须给）')
    ap.add_argument('--prefix', choices=('camera', 'topcam'), default=None,
                    help='启动参数前缀；默认按 --tf-json 的键 / 外参 frame_child 推断')
    ap.add_argument('--out', default=None,
                    help='--solve 时把结果写成 JSON（E3b 逐轮比较用）')
    ap.add_argument('--board', default='docs/calib/charuco_A4_30mm_5x5.json',
                    help='板参数 JSON（make_charuco_board.py 产出）')
    ap.add_argument('--extrinsics', help='--init 用：当前可信的外参文件')
    ap.add_argument('--store', default=os.path.expanduser('~/topcam_marker.json'),
                    help='板位姿的存盘位置')
    ap.add_argument('--square-mm', type=float, default=None,
                    help='打印被缩放时按量出的比例覆盖方格边长')
    ap.add_argument('--marker-mm', type=float, default=None)
    a = ap.parse_args()

    board_meta = json.load(open(a.board, encoding='utf-8'))
    if a.square_mm:
        board_meta['square_mm'] = a.square_mm
    if a.marker_mm:
        board_meta['marker_mm'] = a.marker_mm
    tf_doc = None
    if a.tf_json:
        tf_doc = json.load(open(a.tf_json, encoding='utf-8'))
    ext_doc = (json.load(open(a.extrinsics, encoding='utf-8'))
               if a.extrinsics else None)

    # 前缀：显式给了就照办；没给就按镜头来源推断（topcam 前缀会打出
    # topcam_x..qw 那组参数——前缀打错不报错，只是静态 TF 永远不生效）。
    prefix = a.prefix
    if prefix is None:
        child = str((ext_doc or {}).get('frame_child') or '')
        top_keys = (tf_doc or {}).get('ok') or {}
        prefix = ('topcam' if child.startswith('topcam')
                  or any(k.startswith(TOPCAM_LINK + ':') for k in top_keys)
                  else 'camera')
    link_keys = (TOPCAM_LINK_TO_OPTICAL_KEYS if prefix == 'topcam'
                 else WRIST_LINK_TO_OPTICAL_KEYS)
    link_to_optical = _link_to_optical(tf_doc, link_keys)

    # --image 可以是单帧，也可以是 precision_capture --burst 的帧目录。
    # 多帧融合压掉单帧的角噪声：2026-11-07 真机实测，**单帧基准**的门读数
    # 1.43 mm，同一个静止 burst 取独立两半（0–9 做基准、10–19 做解算）后
    # 只有 0.08 mm。单帧行为一字未改；目录模式只是每帧各解一次再融合。
    if os.path.isdir(a.image):
        names = sorted(glob.glob(os.path.join(a.image, 'color_*.png')))
        if not names:
            print('!! 帧目录里没有 color_*.png：%s' % a.image)
            return 2
        dets = []
        for p in names:
            m = re.search(r'color_(\d+)\.png$', p)
            if not m:
                continue
            idx = m.group(1)
            info_p = os.path.join(a.image, 'meta_%s.json' % idx)
            tf_p = os.path.join(a.image, 'tf_%s.json' % idx)
            if not os.path.exists(info_p):
                print('!! 缺内参 %s（burst 帧不齐）' % info_p)
                return 2
            K_i, dist_i, _ = load_camera(info_p)
            det_i = detect_board(p, board_meta, K_i, dist_i)
            if det_i is None:
                print('  跳过 %s：没检出足够的 ChArUco 角点'
                      % os.path.basename(p))
                continue
            tf_doc_i = (json.load(open(tf_p, encoding='utf-8'))
                        if os.path.exists(tf_p) else tf_doc)
            dets.append((p, tf_doc_i, det_i))
        if not dets:
            print('!! 帧目录里一帧都没检出 ChArUco')
            return 2
        n_pts = min(d[2][1] for d in dets)
        rms = max(d[2][2] for d in dets)
        print('帧目录：%d/%d 帧检出，最小角点 %d，最大重投影 RMS %.3f px'
              % (len(dets), len(names), n_pts, rms))
    else:
        if not a.info:
            print('!! 单帧模式必须给 --info（内参）')
            return 2
        K, dist, (w, h) = load_camera(a.info)
        det = detect_board(a.image, board_meta, K, dist)
        if det is None:
            print('!! 这张图上没检出足够的 ChArUco 角点 —— 板在画面里吗？'
                  '被遮挡、反光、或纸不平都会导致这个结果')
            return 2
        (R_cam_board, t_cam_board), n_pts, rms = det
        print(f'检出 {n_pts} 个角点，重投影 RMS = {rms:.3f} px'
              f'（>1.0 px 通常说明纸不平或被整体缩放）')
        dets = [(a.image, tf_doc, det)]

    # 帧目录里每帧自带 tf.json：没显式给 --tf-json 时用第一帧的驱动实测链。
    if a.tf_json is None and dets[0][1] is not None:
        tf_doc = dets[0][1]
        link_to_optical = _link_to_optical(tf_doc, link_keys)
    tf_source = ('tf-json:%s' % a.tf_json if a.tf_json is not None
                 else ('tf-json:%s' % os.path.join(a.image, 'tf_*.json')
                       if tf_doc is not None else 'ideal'))

    if a.init:
        if not a.extrinsics:
            print('!! --init 必须给 --extrinsics：它是这条链的**基准**')
            return 2
        boards = []
        for img, tf_doc_i, det_i in dets:
            (R_cb, t_cb) = det_i[0]
            base_from_cam_i = base_from_camera(
                ext_doc, _link_to_optical(tf_doc_i, link_keys))
            Rb_i, tb_i = compose(base_from_cam_i, (R_cb, t_cb))
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = Rb_i, tb_i
            boards.append(T)
        if len(boards) > 1:
            T_f = _mean_pose(boards)
            Rb, tb = T_f[:3, :3], T_f[:3, 3]
            dev_t = max(float(np.linalg.norm(T[:3, 3] - tb)) * 1000.0
                        for T in boards)
            dev_a = max(math.degrees(math.acos(min(1.0, max(-1.0,
                (np.trace(T[:3, :3].T @ Rb) - 1.0) / 2.0)))) for T in boards)
            print('多帧融合：%d 帧，相对融合值最大偏离 %.3f mm / %.3f°'
                  % (len(boards), dev_t, dev_a))
        else:
            Rb, tb = boards[0][:3, :3], boards[0][:3, 3]
        data = {'board': a.board,
                'square_mm': board_meta['square_mm'],
                'marker_mm': board_meta['marker_mm'],
                'dictionary': board_meta['dictionary'],
                'base_from_board_R': Rb.tolist(), 'base_from_board_t': tb.tolist(),
                'from_extrinsics': a.extrinsics,
                'link_to_optical_source': tf_source,
                'reproj_rms_px': rms, 'n_corners': n_pts,
                'n_frames': len(boards)}
        with open(a.store, 'w', encoding='utf-8') as fh:
            json.dump(data, fh, ensure_ascii=False, indent=1)
        print(f'板在 base 系里的位置 = {np.round(tb, 4).tolist()} m')
        print(f'已存盘：{a.store}')
        print('⇒ 之后每次用之前跑一次 --solve 即可（一张图，不动机械臂）')
        return 0

    # ---- --solve ----
    if not os.path.exists(a.store):
        print(f'!! 没有 {a.store} —— 先跑一次 --init')
        return 2
    data = json.load(open(a.store, encoding='utf-8'))
    base_from_board = (np.array(data['base_from_board_R'], dtype=float),
                       np.array(data['base_from_board_t'], dtype=float))
    cams = []
    for img, tf_doc_i, det_i in dets:
        (R_cb, t_cb) = det_i[0]
        Rc_i, tc_i = compose(base_from_board, invert((R_cb, t_cb)))
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rc_i, tc_i
        cams.append(T)
    if len(cams) > 1:
        T_c = _mean_pose(cams)
        Rc, tc = T_c[:3, :3], T_c[:3, 3]
    else:
        Rc, tc = cams[0][:3, :3], cams[0][:3, 3]
    base_from_link = compose((Rc, tc), invert(link_to_optical))
    Rl, tl = base_from_link
    print(f'相机光心（光学系原点）在 base_link = {np.round(tc, 4).tolist()} m'
          f'（离基座水平 {math.hypot(tc[0], tc[1]):.3f} m，高 {tc[2]:.3f} m）')
    print('link→optical 链来源：%s'
          % ('tf.json 驱动实测' if tf_doc is not None
             else '理想旋转（真机上这会让联合标定偏约 15 mm）'))
    if 'from_extrinsics' in data:
        prev_doc = json.load(open(data['from_extrinsics'], encoding='utf-8'))
        prev = base_from_camera(prev_doc, link_to_optical)
        d = float(np.linalg.norm(tc - prev[1]))
        ang = math.degrees(math.acos(min(1.0, max(-1.0,
            (np.trace(Rc.T @ prev[0]) - 1.0) / 2.0))))
        print(f'与 --init 那次相比：位移 {d * 1000:.1f} mm，转角 {ang:.3f}°'
              f'（这就是这段时间相机漂掉的量）')
    print('\n启动参数（%s 前缀，base_link←%s_link）：' % (prefix, 'topcam' if prefix == 'topcam' else 'camera'))
    print(fmt_launch(Rl, tl, prefix=prefix))

    if a.out:
        out_doc = {
            'version': 1, 'kind': 'camera-extrinsics',
            'frame_parent': 'base_link',
            'frame_child': ('topcam_color_optical_frame' if prefix == 'topcam'
                            else 'camera_color_optical_frame'),
            'R': Rc.tolist(), 't': tc.tolist(),
            'q': matrix_to_quat(Rc).tolist(),
            'R_link': Rl.tolist(), 't_link': tl.tolist(),
            'link_to_optical_source': tf_source,
            'n_frames': len(cams),
            'reproj_rms_px': rms, 'n_corners': n_pts,
            'image': a.image, 'store': a.store, 'board': a.board,
            'square_mm': board_meta['square_mm'],
            'marker_mm': board_meta['marker_mm'],
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
        }
        with open(a.out, 'w', encoding='utf-8') as fh:
            json.dump(out_doc, fh, ensure_ascii=False, indent=1)
        print('写入 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
