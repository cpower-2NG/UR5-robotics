#!/usr/bin/env python3
"""阶段 4 · E4a 改法 B 的自检：**不需要 ROS、相机、真机**。

钉住三件事：

1. **倾斜必须由「相机横偏」驱动**：把 ``t_tool0←camera`` 的横偏移置零后，所需倾斜
   角必须掉到 0° —— 防止有人把 ``solve_tilt`` 写成常数/永远返回某一角度；
2. **倾斜之后块顶面必须进画幅**：同一块、同一高度，倾斜前面距 <20 px（复现 E4a
   紫/黄的切边），倾斜后 >200 px；
3. **IK 与几何一致**：解出的关节角做 FK，与目标位姿差 <0.1 mm / 0.1°，且都在软限位内。

跑法：``python3 scripts/test_stage4_bcase_plan.py``；退出码 0 = 全通过。
"""

import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import stage4_bcase_plan as bp                                    # noqa: E402
import urkin                                                      # noqa: E402

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


def urdf_path():
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(os.path.dirname(here), '..', '..', 'notes', 'ur5-vision-grasp',
                        'runs', '2026-10-25-precision', 'robot.urdf')
    cand = os.path.abspath(cand)
    if os.path.exists(cand):
        return cand
    return os.path.join(os.path.expanduser('~'), 'robotics', 'notes',
                        'ur5-vision-grasp', 'runs', '2026-10-25-precision', 'robot.urdf')


def synthetic_mount(offset_mm=(-31.9, -85.3, 24.3)):
    """tool0←camera 的合成 4×4：轴与 tool0 z 平行，只在 tool0 系里横偏 ``offset_mm``。"""
    T = np.eye(4)
    T[:3, 3] = np.array(offset_mm) / 1000.0
    return T


def main():
    K = np.array([[908.15, 0.0, 639.44],
                  [0.0, 908.38, 363.31],
                  [0.0, 0.0, 1.0]])
    W, H = 1280, 720
    R_nom = np.diag([1.0, -1.0, -1.0])          # 现场 tool0 的标称姿态（九组采集一致）
    T_tc = synthetic_mount()
    z_top = 0.03142
    hover = 0.050
    block = np.array([0.5637, 0.2633])
    P = np.array([block[0], block[1], z_top + hover + bp.TOOL0_TO_BARE_FINGERTIP_M])
    edge = 0.030

    print('[A] 倾斜由相机横偏驱动')
    th, sign, R, miss = bp.solve_tilt(R_nom, T_tc, P, block, z_top)
    check('A1 有横偏 ⇒ 倾斜 18–23°', 18.0 <= th <= 23.0, 'tilt=%.2f' % th)
    check('A2 光轴落点误差 <3 mm', miss < 3.0, 'miss=%.3f mm' % miss)
    tip_clear = (P + bp.TOOL0_TO_BARE_FINGERTIP_M * R[:, 2])[2] - z_top
    check('A3 指尖仍在块顶之上 (>50 mm)', tip_clear > 0.050,
          'clear=%.1f mm' % (tip_clear * 1000))
    th0, _, R0, _ = bp.solve_tilt(R_nom, synthetic_mount((0.0, 0.0, 0.0)),
                                  P, block, z_top)
    check('A4 横偏移置零 ⇒ 倾斜角 0°（防常数）', th0 < 0.05, 'tilt=%.3f' % th0)
    check('A5 置零时解出的姿态就是标称姿态',
          np.allclose(R0, R_nom, atol=1e-6))

    print('[B] 倾斜前后面距（复现 E4a 切边 → B 案进画幅）')
    before = bp.project_face(R_nom, T_tc, P, block, z_top, K, W, H, edge)
    after = bp.project_face(R, T_tc, P, block, z_top, K, W, H, edge)
    check('B1 倾斜前面距 <20 px（复现切边）', before[4] < 20.0,
          'margin=%.0f px' % before[4])
    check('B2 倾斜后面距 >200 px', after[4] > 200.0, 'margin=%.0f px' % after[4])
    check('B3 倾斜后水平方向仍居中（|u_c−640|<60 px）',
          abs((after[0] + after[1]) / 2.0 - W / 2.0) < 60.0,
          'u_c=%.0f' % ((after[0] + after[1]) / 2.0))

    print('[C] 三个悬停高度')
    urdf = urdf_path()
    if not os.path.exists(urdf):
        print('  SKIP  找不到 URDF：%s' % urdf)
    else:
        chain = urkin.Chain(urdf)
        q_seed = {j: 0.0 for j in chain.move_joints}
        q_seed.update({'elbow_joint': 1.5, 'shoulder_lift_joint': -1.2,
                       'wrist_1_joint': -1.88, 'wrist_2_joint': -1.57,
                       'wrist_3_joint': -1.31, 'shoulder_pan_joint': 0.26})
        prev = None
        for hv in (0.050, 0.065, 0.080):
            r = bp.plan_one(chain, q_seed, T_tc, R_nom, K, W, H, block, z_top, hv, edge)
            check('C%.0f IK 收敛且软限位内 (%g mm)' % (hv * 1000, hv * 1000),
                  r['ik']['ok'],
                  'pe=%.3f re=%.3f margin=%.0f°' % (r['ik']['pos_err_mm'],
                                                    r['ik']['rot_err_deg'],
                                                    r['ik']['min_limit_margin_deg']))
            check('C%.0f 面距 >200 px' % (hv * 1000),
                  r['face_projection']['margin_px'] > 200.0)
            check('C%.0f 指尖净空 >50 mm' % (hv * 1000),
                  r['fingertip_above_block_top_mm'] > 50.0)
            if prev is not None:
                check('C%.0f 悬停越高倾斜越小' % (hv * 1000),
                      r['tilt_deg'] < prev)
            prev = r['tilt_deg']

    print('\n%d passed, %d failed' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
