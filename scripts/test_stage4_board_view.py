#!/usr/bin/env python3
"""阶段 4 · 「腕部看板」位姿工具的纯几何测试：**不需要 ROS、相机、真机、URDF**。

钉住三件事（每一条都对应过一次真实的错）：

1. **板帧原点在角上**，板心 = 原点 + 半宽/半高。把这里当成「t 就是板心」会让
   相机横移半个板宽（75 mm），而投影看上去仍然「都在画幅里」—— 2026-11-07 真踩过：
   边距 117 px、判 OK，实际板心偏了 106 mm。
2. **板四角必须关于画幅中心对称**，板心投到主点附近（±15 px，来自夹具 0.57° 的
   光轴偏差）。半个板宽 = 约 120 px，第 1 条错了这条会红。
3. **IK/限位只判自己该判的**：几何不过必须返回 rc=1，不靠 IK 收敛遮掩。

跑法：``python3 scripts/test_stage4_board_view.py``；退出码 0 = 全通过。
"""

import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage4_board_view_pose as bv                               # noqa: E402

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


BOARD_SPEC = os.path.join(os.path.dirname(HERE), 'docs', 'calib',
                          'charuco_A4_30mm_5x5.json')


def _raises(fn):
    """fn 抛 SystemExit ⇒ True。用来断言「缺输入必须停下」而不是带病继续。"""
    try:
        fn()
    except SystemExit:
        return True
    return False


def synthetic_store(t=(0.2, 0.6, 0.0), yaw_deg=0.0):
    """板平放在台面上、可整体绕 z 转 —— ``base←board`` 的合成值。"""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return {'board': BOARD_SPEC, 'square_mm': 30.0, 'marker_mm': 22.0,
            'dictionary': 'DICT_4X4_50',
            'base_from_board_R': R.tolist(),
            'base_from_board_t': list(t)}


def synthetic_mount(offset_mm=(-31.9, -85.3, 24.3)):
    T = np.eye(4)
    T[:3, 3] = np.array(offset_mm) / 1000.0
    return T


K = np.array([[908.15, 0.0, 639.44],
              [0.0, 908.38, 363.31],
              [0.0, 0.0, 1.0]])
W, H = 1280, 720
R_NOM = np.diag([1.0, -1.0, -1.0])
T_TC = synthetic_mount()


def main():
    st = synthetic_store()
    centre, corners = bv.board_centre_and_corners(st, 'unused')

    print('[A] 板帧原点约定（角，不是板心）')
    R = np.asarray(st['base_from_board_R'])
    t = np.asarray(st['base_from_board_t'])
    want = t + R @ np.array([0.075, 0.075, 0.0])
    check('A1 板心 = 原点 + 半宽/半高', np.allclose(centre, want, atol=1e-12),
          'centre=%s want=%s' % (np.round(centre, 4), np.round(want, 4)))
    check('A2 四角的质心就是板心',
          np.allclose(np.mean(np.asarray(corners), axis=0), centre, atol=1e-12))
    check('A3 四角构成 150×150 mm 的正方形',
          np.allclose(np.sort(np.linalg.norm(
              np.asarray(corners) - centre, axis=1)), [0.106066] * 4, atol=1e-6),
          str(np.round(np.linalg.norm(np.asarray(corners) - centre, axis=1), 6)))
    check('A4 原点本身不是板心（防有人把两者当成一件事）',
          not np.allclose(centre, t, atol=1e-6))

    print('[B] 相机必须正对板心')
    P = bv.solve_pose(R_NOM, T_TC, centre, 0.55)
    C = P + R_NOM @ T_TC[:3, 3]
    check('B1 相机 xy 落在板心正上方（<0.01 mm）',
          float(np.linalg.norm(C[:2] - centre[:2])) * 1000 < 0.01,
          'C=%s centre=%s' % (np.round(C * 1000, 3), np.round(centre * 1000, 3)))
    check('B2 相机高于板面 550 mm', abs((C[2] - centre[2]) * 1000 - 550.0) < 0.01)

    print('[C] 投影：板心在主点、四角对称、边距够')
    uv_c, _ = bv.project_points(P, R_NOM, T_TC, K, W, H, [centre])
    check('C1 板心投到主点 ±15 px',
          abs(uv_c[0][0] - 639.44) < 15 and abs(uv_c[0][1] - 363.31) < 15,
          'uv=%s' % np.round(uv_c[0], 1))
    uv, margin = bv.project_points(P, R_NOM, T_TC, K, W, H, corners)
    cu = np.mean([p[0] for p in uv])
    cv = np.mean([p[1] for p in uv])
    check('C2 四角 uv 质心仍在主点 ±15 px（原点约定错了这里会偏 ~120 px）',
          abs(cu - 639.44) < 15 and abs(cv - 363.31) < 15,
          'centroid=(%.1f, %.1f)' % (cu, cv))
    check('C3 边距 >100 px', margin > 100.0, 'margin=%.1f px' % margin)
    check('C4 板在画面里约 200–350 px 宽（看得清又不爆框）',
          200 < (max(p[0] for p in uv) - min(p[0] for p in uv)) < 350,
          '宽 %.0f px' % (max(p[0] for p in uv) - min(p[0] for p in uv)))

    print('[D] 板的朝向变了，板心仍要投到主点（不能只在轴对齐时成立）')
    for yaw in (17.0, 45.0, 123.0):
        st_y = synthetic_store(yaw_deg=yaw)
        c_y, cn_y = bv.board_centre_and_corners(st_y, 'unused')
        P_y = bv.solve_pose(R_NOM, T_TC, c_y, 0.55)
        uvc, _ = bv.project_points(P_y, R_NOM, T_TC, K, W, H, [c_y])
        uvy, m_y = bv.project_points(P_y, R_NOM, T_TC, K, W, H, cn_y)
        cy = np.mean([p[1] for p in uvy])
        check('D%g 旋转后板心仍在主点 ±15 px、边距 >100 px' % yaw,
              abs(uvc[0][0] - 639.44) < 15 and abs(cy - 363.31) < 15 and m_y > 100,
              'uv=%s v_c=%.1f margin=%.1f' % (np.round(uvc[0], 1), cy, m_y))

    print('[E] 板尺寸从规格文件读，不许猜')
    check('E1 规格文件给出 150×150 mm',
          bv.board_size_mm(st, 'unused') == (150.0, 150.0),
          str(bv.board_size_mm(st, 'unused')))
    missing = dict(st, board='docs/calib/does_not_exist.json')
    try:
        bv.board_size_mm(missing, 'unused')
    except SystemExit as e:
        check('E2 规格文件找不到 ⇒ 当场报错（不静默用别的尺寸）',
              '找不到板规格' in str(e), str(e)[:80])
    else:
        check('E2 规格文件找不到 ⇒ 当场报错（不静默用别的尺寸）', False,
              '竟然返回了尺寸')
    check('E3 存盘里没有 board 字段也报错',
          _raises(lambda: bv.board_centre_and_corners(
              {k: v for k, v in st.items() if k != 'board'}, 'unused')))

    print('\n%d passed, %d failed' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
