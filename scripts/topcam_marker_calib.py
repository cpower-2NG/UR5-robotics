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
import json
import math
import os
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


def rodrigues_to_matrix(rvec):
    """OpenCV 的旋转向量 → 3×3。"""
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return R


def load_camera(info_path):
    """`grab_frames.py` 产出的 info.json → ``(K, dist, (w, h))``。"""
    info = json.load(open(info_path, encoding='utf-8'))
    K = np.array([[info['fx'], 0.0, info['cx']],
                  [0.0, info['fy'], info['cy']],
                  [0.0, 0.0, 1.0]], dtype=float)
    dist = np.array(info.get('D', [0.0] * 5), dtype=float).reshape(-1, 1)
    return K, dist, (int(info['width']), int(info['height']))


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
        description='俯视相机外参重算：一张 ChArUco 图，不动机械臂')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--init', action='store_true',
                   help='用当前可信的外参把「板在 base 系里的位姿」定下来并存盘')
    g.add_argument('--solve', action='store_true',
                   help='由存下的板位姿重算 base←cam，并打印启动参数')
    ap.add_argument('--image', required=True)
    ap.add_argument('--info', required=True, help='同一次采集的 info.json（内参）')
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

    K, dist, (w, h) = load_camera(a.info)
    det = detect_board(a.image, board_meta, K, dist)
    if det is None:
        print('!! 这张图上没检出足够的 ChArUco 角点 —— 板在画面里吗？'
              '被遮挡、反光、或纸不平都会导致这个结果')
        return 2
    (R_cam_board, t_cam_board), n_pts, rms = det
    cam_from_board = (R_cam_board, t_cam_board)
    print(f'检出 {n_pts} 个角点，重投影 RMS = {rms:.3f} px'
          f'（>1.0 px 通常说明纸不平或被整体缩放）')

    if a.init:
        if not a.extrinsics:
            print('!! --init 必须给 --extrinsics：它是这条链的**基准**')
            return 2
        base_from_cam = base_from_camera(ext_doc, link_to_optical)
        base_from_board = compose(base_from_cam, cam_from_board)
        Rb, tb = base_from_board
        data = {'board': a.board,
                'square_mm': board_meta['square_mm'],
                'marker_mm': board_meta['marker_mm'],
                'dictionary': board_meta['dictionary'],
                'base_from_board_R': Rb.tolist(), 'base_from_board_t': tb.tolist(),
                'from_extrinsics': a.extrinsics,
                'link_to_optical_source': ('tf-json:%s' % a.tf_json
                                           if tf_doc is not None else 'ideal'),
                'reproj_rms_px': rms, 'n_corners': n_pts}
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
    base_from_cam = compose(base_from_board, invert(cam_from_board))
    Rc, tc = base_from_cam
    base_from_link = compose(base_from_cam, invert(link_to_optical))
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
            'link_to_optical_source': ('tf-json:%s' % a.tf_json
                                       if tf_doc is not None else 'ideal'),
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
