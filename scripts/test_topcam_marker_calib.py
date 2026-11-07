#!/usr/bin/env python3
"""`topcam_marker_calib.py` 的纯函数自检（**不需要 ROS、相机、图片、标定板**）。

要钉住的是**这一条链的数学** —— 它整条建立在一个复合上：

    base←cam = (base←板) ∘ (cam←板)^-1

复合的顺序、求逆的写法、以及「板位姿怎么由一次已知可信的外参推出来」，
只要错一处，结果会是一个**看着完全合理**的相机位姿（数值都是三位数），
而外参错了的表现是「机械臂抓偏」—— 与坐标系错的表现一模一样，很难倒推。
所以这里逐条校验代数性质。

跑法：
    python3 scripts/test_topcam_marker_calib.py
退出码 0 = 全通过。
"""

import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402
from topcam_marker_calib import (                                 # noqa: E402
    R_LINK_TO_OPTICAL, base_from_camera, compose, invert,
    matrix_to_quat, quat_to_matrix)

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def safe(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except SystemExit as e:                   # 本文件里「拒绝」是用 SystemExit 表达的
        return f'!! 拒绝：{e}'
    except Exception as e:                    # noqa: BLE001
        return f'!! 崩了：{type(e).__name__}: {e}'


# 一个任意的刚体变换，当测试素材
R_TEST = quat_to_matrix([0.3, -0.2, 0.5, 0.78])
T_TEST = np.array([0.35, 0.87, 0.65])
A = (R_TEST, T_TEST)

print('--- A. 四元数 ↔ 矩阵 ---')
check('往返一致（矩阵 → 四元数 → 矩阵）',
      np.allclose(quat_to_matrix(matrix_to_quat(R_TEST)), R_TEST, atol=1e-12))
check('单位化：任意缩放的四元数给出同一个矩阵',
      np.allclose(quat_to_matrix([0.6, -0.4, 1.0, 1.56]),
                  quat_to_matrix([0.3, -0.2, 0.5, 0.78]), atol=1e-12))
check('det = +1（是旋转不是镜像）',
      abs(float(np.linalg.det(quat_to_matrix(matrix_to_quat(R_TEST)))) - 1.0) < 1e-12)
check('零四元数 ⇒ 抛（不返回垃圾矩阵）',
      isinstance(safe(quat_to_matrix, [0, 0, 0, 0]), str))
# 覆盖 matrix_to_quat 的四个分支：绕 x / y / z 各 180°，以及小角度
for axis, name in ((np.array([1.0, 0, 0]), 'x'), (np.array([0, 1.0, 0]), 'y'),
                   (np.array([0, 0, 1.0]), 'z')):
    for deg in (180.0, 179.0, 1.0):
        th = math.radians(deg)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
        check(f'绕 {name} 轴 {deg:g}° 的往返（trace 分支的另一条路）',
              np.allclose(quat_to_matrix(matrix_to_quat(R)), R, atol=1e-9))

print('\n--- B. 复合与求逆 ---')
check('逆：A ∘ A⁻¹ = 单位变换',
      all(np.allclose(x, y, atol=1e-12)
          for x, y in zip(compose(A, invert(A)), (np.eye(3), np.zeros(3)))))
check('逆：A⁻¹ ∘ A = 单位变换',
      all(np.allclose(x, y, atol=1e-12)
          for x, y in zip(compose(invert(A), A), (np.eye(3), np.zeros(3)))))
check('★ 求逆用的是 −Rᵀt，不是 −t（把两者写反在数值上也是「像样的三位数」）',
      np.allclose(invert(A)[1], -R_TEST.T @ T_TEST)
      and not np.allclose(invert(A)[1], -T_TEST))
B = (quat_to_matrix([-0.1, 0.4, 0.2, 0.88]), np.array([-0.2, 0.1, 0.05]))
check('复合不交换：A∘B ≠ B∘A（顺序有意义）',
      not np.allclose(compose(A, B)[1], compose(B, A)[1]))

print('\n--- C. 帧名判定（拒绝猜） ---')
ARCH = {'R': R_TEST.tolist(), 't': T_TEST.tolist(),
        'frame_parent': 'base_link', 'frame_child': 'camera_link'}
Rb, tb = safe(base_from_camera, ARCH)
check('camera_link ⇒ 左乘固定旋转',
      np.allclose(Rb, R_TEST @ R_LINK_TO_OPTICAL) and np.allclose(tb, T_TEST))
check('  t 不参与旋转（光心在 parent 系就是 t）', np.allclose(tb, T_TEST))
Rb2, tb2 = safe(base_from_camera, {**ARCH, 'frame_child': 'camera_color_optical_frame'})
check('★ 已是光学系 ⇒ 原样返回，不重复左乘', np.allclose(Rb2, R_TEST))
check('  且与 camera_link 那支不相等（说明这一步真做了事）',
      not np.allclose(Rb, Rb2))
check('`camera_frame` 这个键名也认',
      np.allclose(safe(base_from_camera,
                       {k: v for k, v in ARCH.items() if k != 'frame_child'}
                       | {'camera_frame': 'camera_color_optical_frame'})[0], R_TEST))
check('★ 不认识的帧名 ⇒ 拒绝（不猜）',
      isinstance(safe(base_from_camera, {**ARCH, 'frame_child': 'some_frame'}), str))
check('  缺帧名也拒绝', isinstance(safe(base_from_camera, {'R': R_TEST.tolist(),
                                                          't': T_TEST.tolist()}), str))
check('四元数形式（q）也认，且与 R 一致',
      np.allclose(safe(base_from_camera, {'q': matrix_to_quat(R_TEST).tolist(),
                                          't': T_TEST.tolist(),
                                          'frame_child': 'camera_link'})[0],
                  R_TEST @ R_LINK_TO_OPTICAL, atol=1e-9))

print('\n--- D. ★ 整条链的闭环：init 之后再 solve，应当拿回原外参 ---')
# 设相机真值 base←cam；由它和某次观测 cam←板 推出 base←板（--init）；
# 再用**另一次**观测（板没动、相机漂了）反推 base←cam（--solve）。
base_from_cam_true = (quat_to_matrix([0.02, 0.31, 0.64, 0.70]),
                      np.array([0.35, 0.87, 0.65]))
cam_from_board_init = (quat_to_matrix([0.1, -0.5, 0.2, 0.83]),
                       np.array([0.05, -0.02, 0.62]))
base_from_board = compose(base_from_cam_true, cam_from_board_init)
# 相机漂了一个小旋转：新的 cam←板 = 漂移 ∘ 旧的
R_drift = quat_to_matrix([0.0, 0.0, math.sin(math.radians(0.5) / 2),
                          math.cos(math.radians(0.5) / 2)])
cam_from_board_new = compose((R_drift, np.zeros(3)), cam_from_board_init)
base_from_cam_solved = compose(base_from_board, invert(cam_from_board_new))
ang = math.degrees(math.acos(min(1.0, max(-1.0,
    (np.trace(base_from_cam_solved[0].T @ base_from_cam_true[0]) - 1.0) / 2.0))))
check('★ 无漂移时 solve 精确还原 init 的外参',
      all(np.allclose(x, y, atol=1e-12) for x, y in zip(
          compose(base_from_board, invert(cam_from_board_init)),
          base_from_cam_true)))
check(f'★ 漂 0.5° 时 solve 出来的位姿也正好偏 0.5°（差 {ang:.3f}°）',
      abs(ang - 0.5) < 1e-6, f'got {ang}')
check('  位移不受纯转动漂移影响（相机光心没动）',
      np.allclose(base_from_cam_solved[1], base_from_cam_true[1], atol=1e-12))
check('★ 反过来：板若被写错（把 cam←板 当成 base←板），闭环就崩 —— 值得显式记',
      not np.allclose(compose(A, invert(B))[0], compose(A, invert(A))[0]))

print('\n--- E. ★ 同 ID 撞车：按板版式挑回自己那一簇（真机 2026-11-07 实测缺陷） ---')
# 背景：垫子页 A 与 ChArUco 板都用 DICT_4X4_50，id 0–9 重合。真机上两张靶
# 同时进俯视画面，旧写法按下标取用 ⇒ 板 PnP 混进垫子的角点，同一条命令
# 一帧 RMS 0.28 px、另一帧 53 px（见 topcam_marker_calib._consistent_board_markers
# 的 docstring）。这里造一块**同 id、不同版式**的外来靶（标记间距/行列都不同，
# 所以它不可能是自板平面的某个单应），钉住两件事：
#   · 无论外来靶排在检出列表前面还是后面，只能留下自板那一簇；
#   · 无撞车时函数**原样**返回，不偷偷改单张标定的行为。
import cv2                                                    # noqa: E402
import cv2.aruco as aruco                                     # noqa: E402
import topcam_marker_calib as tmc                             # noqa: E402


def _mk_charuco():
    d = aruco.Dictionary_get(aruco.DICT_4X4_50)
    return aruco.CharucoBoard_create(5, 5, 0.03, 0.022, d), d


def _proj(H, xy):
    p = np.hstack([np.asarray(xy, dtype=float),
                   np.ones((len(xy), 1))]) @ np.asarray(H, float).T
    return p[:, :2] / p[:, 2:3]


board_e, dict_e = _mk_charuco()
ids_true = [int(v) for v in np.asarray(board_e.ids).ravel()]
H_true = np.array([[900.0, 25.0, 720.0], [-18.0, 890.0, 430.0], [0.0, 0.0, 1.0]])
rng = np.random.default_rng(7)
true_c = []
for i in range(len(board_e.objPoints)):
    xy = np.asarray(board_e.objPoints[i], float).reshape(4, 3)[:, :2]
    true_c.append((_proj(H_true, xy)
                   + rng.normal(0.0, 0.12, (4, 2))).astype(np.float32))
# 外来靶：只用 id 0–9，版式 2×5 / 42 mm（自板是 5×5 / 30 mm），投影到另一处
n_decoy = min(10, len(ids_true))
decoy_ids = ids_true[:n_decoy]
H_dec = np.array([[1100.0, -40.0, 300.0], [30.0, 1050.0, 900.0], [0.0, 0.0, 1.0]])
decoy_c = []
for k in range(n_decoy):
    cx, cy = (k % 5) * 0.042, (k // 5) * 0.042
    o = np.array([[cx, cy], [cx + 0.03, cy], [cx + 0.03, cy + 0.03],
                  [cx, cy + 0.03]])
    decoy_c.append((_proj(H_dec, o)
                    + rng.normal(0.0, 0.12, (4, 2))).astype(np.float32))
K_e = np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]])


def _check_cluster(order):
    if order == 'decoy_last':
        cc = list(true_c) + list(decoy_c)
        ii = np.array(ids_true + decoy_ids).reshape(-1, 1)
    else:
        cc = list(decoy_c) + list(true_c)
        ii = np.array(decoy_ids + ids_true).reshape(-1, 1)
    gc, gi = tmc._consistent_board_markers(board_e, cc, ii, K_e)
    got = {int(i): np.asarray(c, float).reshape(4, 2) for c, i in zip(gc, gi)}
    want = {mid: np.asarray(true_c[ids_true.index(mid)], float).reshape(4, 2)
            for mid in ids_true}
    kept = sorted(got) == sorted(want)
    err = (max(np.abs(got[m] - want[m]).max() for m in want) if kept else 9.99)
    check('%s：只留下自板 %d 个标记（留下 %d 个）'
          % (order, len(ids_true), len(got)), kept)
    check('%s：留下的角点就是自板的（最大差 %.1e px）' % (order, err),
          kept and err < 1e-6)


_check_cluster('decoy_last')
_check_cluster('decoy_first')
gc, gi = tmc._consistent_board_markers(
    board_e, list(true_c[:5]), np.array(ids_true[:5]).reshape(-1, 1), K_e)
check('  无撞车时原样返回（5 个标记，角点不变）',
      len(gc) == 5 and np.allclose(np.asarray(gc[0], float),
                                   np.asarray(true_c[0], float), atol=1e-9))

print('\n通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(1 if N_FAIL else 0)
