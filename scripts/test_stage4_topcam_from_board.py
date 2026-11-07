#!/usr/bin/env python3
"""阶段 4 · 「腕部看板裁定俯视外参」的合成测试：**不需要相机、真机、ROS**。

这条链把三个量复合起来（腕部 base←board、存盘 base←board、旧外参 base←topcam），
符号错一处结果仍然「像个数」——所以必须用**已知答案**的合成数据钉符号：

1. 腕部量到的 base←board 与存盘**一模一样** ⇒ 解出的新外参必须**逐位等于**旧外参；
2. 腕部量到的板往 +x 挪 10 mm ⇒ 新外参也必须往 +x 挪 10 mm（不是 −10、不是不动）；
3. 腕部量到的板绕 z 转 1° ⇒ 新外参与旧外参的旋转差正好 1°；
4. link←optical 的复合是自洽的（新光学系 ∘ link←optical⁻¹ = 新 link 系）；
5. 旧外参文档缺 link 系时**必须**退回理想旋转并**标明来源**，不许假装是实测。

跑法：``python3 scripts/test_stage4_topcam_from_board.py``；退出码 0 = 全通过。
"""

import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import stage4_joint_calib as s4jc                                 # noqa: E402
import stage4_topcam_from_board as tb                             # noqa: E402

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


def ang_deg(Ra, Rb):
    return float(np.degrees(np.linalg.norm(
        Rot.from_matrix(Ra.T @ Rb).as_rotvec())))


def synthetic():
    """一份「像现场」的合成三元组：旧外参、存盘板位姿、板在腕部里的位姿。"""
    # 俯视相机装在台面上方约 0.8 m、朝下、略前倾
    R_old = Rot.from_euler('xyz', [80.0, 0.0, 12.0], degrees=True).as_matrix()
    t_old = np.array([0.3906, 1.0781, 0.8043])
    # 板平放在台面上，原点在板角
    R_board = Rot.from_euler('z', 35.0, degrees=True).as_matrix()
    t_board = np.array([0.2361, 0.6439, -0.0067])
    store = {'base_from_board_R': R_board.tolist(),
             'base_from_board_t': t_board.tolist()}
    R_link = Rot.from_euler('xyz', [100.0, 0.0, 12.0], degrees=True).as_matrix()
    t_link = t_old + np.array([0.001, -0.002, 0.003])
    doc = {'frame_child': 'topcam_color_optical_frame',
           'R': R_old.tolist(), 't': t_old.tolist(),
           'R_link': R_link.tolist(), 't_link': t_link.tolist()}
    T_board = s4jc._T(R_board, t_board)
    return T_board, store, doc


def main():
    T_board, store, doc = synthetic()

    print('[A] 腕部与存盘完全一致 ⇒ 新外参 = 旧外参')
    cam_new, link_new, (R_old, t_old), src = tb.compose_extrinsic(
        T_board, store, doc)
    check('A1 平移逐位相同（<1e-9 m）',
          np.allclose(cam_new[:3, 3], t_old, atol=1e-9),
          str(np.round(cam_new[:3, 3] - t_old, 12)))
    check('A2 旋转逐位相同', np.allclose(cam_new[:3, :3], R_old, atol=1e-9))
    check('A3 link 系也回到文档里的值',
          np.allclose(link_new[:3, 3], doc['t_link'], atol=1e-9)
          and np.allclose(link_new[:3, :3], doc['R_link'], atol=1e-9))
    check("A4 链来源标成 'doc'", src == 'doc', src)

    print('[B] 腕部量到的板挪了 10 mm ⇒ 外参也必须挪 10 mm（同向！）')
    T_shift = T_board.copy()
    T_shift[:3, 3] = T_shift[:3, 3] + np.array([0.010, 0.0, 0.0])
    cam_s, _, _, _ = tb.compose_extrinsic(T_shift, store, doc)
    d = cam_s[:3, 3] - t_old
    check('B1 Δx = +10.000 mm', abs(d[0] * 1000 - 10.0) < 1e-6,
          'Δ = %s mm' % np.round(d * 1000, 6))
    check('B2 Δy、Δz 为零', abs(d[1]) < 1e-9 and abs(d[2]) < 1e-9)
    check('B3 旋转不受平移影响', ang_deg(cam_s[:3, :3], R_old) < 1e-9)

    print('[C] 腕部量到的板转了 1° ⇒ 外参旋转差正好 1°')
    T_rot = T_board.copy()
    T_rot[:3, :3] = Rot.from_euler('z', 1.0, degrees=True).as_matrix() @ T_board[:3, :3]
    cam_r, _, _, _ = tb.compose_extrinsic(T_rot, store, doc)
    check('C1 旋转差 = 1.000°',
          abs(ang_deg(R_old, cam_r[:3, :3]) - 1.0) < 1e-6,
          '%.6f°' % ang_deg(R_old, cam_r[:3, :3]))
    check('C2 板转 1° 时平移只动约 t_board 半径 × 1°（不是原封不动）',
          np.linalg.norm(cam_r[:3, 3] - t_old) > 1e-4,
          '|Δ| = %.4f mm' % (np.linalg.norm(cam_r[:3, 3] - t_old) * 1000))

    print('[D] link←optical 自洽')
    T_lo, _ = tb.link_from_optical(doc)
    recomposed = s4jc._compose(link_new, T_lo)
    check('D1 link 系 ∘ link←optical = 光学系',
          np.allclose(recomposed[:3, 3], cam_new[:3, 3], atol=1e-9)
          and np.allclose(recomposed[:3, :3], cam_new[:3, :3], atol=1e-9))

    print('[E] 文档缺 link 系 ⇒ 退回理想旋转并标明')
    doc2 = {k: v for k, v in doc.items() if not k.startswith(('R_link', 't_link'))}
    _, _, _, src2 = tb.compose_extrinsic(T_board, store, doc2)
    T_lo2, src2b = tb.link_from_optical(doc2)
    check("E1 来源标成 'ideal'", src2b == 'ideal', src2b)
    check('E2 理想链就是 R_LINK_TO_OPTICAL',
          np.allclose(T_lo2[:3, :3], tb.tmc.R_LINK_TO_OPTICAL))

    print('\n%d passed, %d failed' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
