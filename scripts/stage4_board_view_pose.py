#!/usr/bin/env python3
"""阶段 4 · 俯视外参：算「腕部相机正对 ChArUco 板」的位姿（纯离线，不动臂）。

**为什么要这个位姿**：俯视外参现在有两条路线差 4.4 mm（垫 PnP vs ChArUco 板），
两条都是「垫/板在本路相机里的位姿」再相除/相乘，垫本身不平、俯视看板又只有
4–6 个标记 ⇒ 谁也裁不了谁。要裁，就得找一条**跟垫无关**的外部量：

    腕部看板 → base←board（近距离、标记全、RMS 小）
    俯视看板 → cam←board（已有的 16 角点 × 20 帧融合存盘）
    两者复合   → base←topcam

板静止，两路**不需要同时**拍；缺的只是腕部一次「看得全」的采集。E2 那批腕部帧
只看到 7–12 个角点、RMS 12–25 px，用不了。

**几何**：相机的光轴与 tool0 z 平行（差 0.57°），但本体在 tool0 系里横偏
``(-31.9, -85.3) mm``。所以「相机正对板心」不等于「tool0 正对板心」——
tool0 要往相反方向让开这段横偏。高度按画幅边距选，默认 550 mm。

本工具只做几何与 IK，**不产生任何运动**；输出 JSON 给 ``precision_plan.py
--move --move-quat`` 直接吃。

退出码：0 = IK 收敛且在软限位、板四角全部进画幅；1 = 不过；2 = 输入缺失。
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import stage4_bcase_plan as bp                                    # noqa: E402
import urkin                                                      # noqa: E402

#: 板四角离画幅边缘至少留这么多像素（E4a 判据同款；这里只用来判「看得全」）
MIN_FRAME_MARGIN_PX = 40


def board_size_mm(store, store_path):
    """板的全尺寸（mm, mm）。存盘里只有 ``board`` 这个**规格文件路径**，尺寸去那里读。

    规格文件里的 ``board_w_mm``/``board_h_mm`` 是**全尺寸**；拿 ``square_mm`` 自己
    乘格数也行，但格数在存盘里没有 —— 别猜，去读。
    """
    spec = store.get('board')
    if not spec:
        raise SystemExit('存盘没有 board 规格路径，不知道板多大')
    cands = [spec, os.path.join(os.path.dirname(os.path.dirname(HERE)), spec),
             os.path.join(os.path.dirname(os.path.abspath(store_path)), spec)]
    for p in cands:
        if os.path.exists(p):
            d = json.load(open(p, encoding='utf-8'))
            return float(d['board_w_mm']), float(d['board_h_mm'])
    raise SystemExit('找不到板规格文件：%s（试过 %s）' % (spec, cands))


def board_centre_and_corners(store, store_path):
    """存盘 ``base←board`` → (板心, 四角) 都在基座系，米。

    ChArUco 板的原点在一个**角**上（不是板心）：板心 = 原点 + 半宽沿 x、半高沿 y。
    """
    R = np.asarray(store['base_from_board_R'], dtype=float)
    t = np.asarray(store['base_from_board_t'], dtype=float)
    bw, bh = board_size_mm(store, store_path)
    w_m, h_m = bw / 1000.0, bh / 1000.0
    # 原点在**板角**（OpenCV 的板帧就是这样：objPoints 落在 1..4 个方格上，
    # 即 30–120 mm，离两边各半个方格）。所以板占 [0,W]×[0,H]。
    corners = [t + R @ np.array([x, y, 0.0])
               for x, y in ((0.0, 0.0), (w_m, 0.0), (w_m, h_m), (0.0, h_m))]
    centre = t + R @ np.array([w_m / 2.0, h_m / 2.0, 0.0])
    return centre, corners


def solve_pose(R_nom, t_tc, centre, height_m):
    """相机放在板心正上方 ``height_m`` 处 ⇒ 反解 tool0 位置（姿态不变）。"""
    off = R_nom @ t_tc[:3, 3]                        # 相机在基座系里相对 tool0 的横偏
    P = np.array([centre[0] - off[0], centre[1] - off[1],
                  centre[2] + height_m - off[2]])
    return P


def project_points(P, R, t_tc, K, w, h, pts):
    """基座系点集投到该姿态的腕部画面 → ``(uv 列表, margin_px)``。"""
    Tcam = np.eye(4)
    Tcam[:3, :3] = R[:3, :3] @ t_tc[:3, :3]
    Tcam[:3, 3] = P + R[:3, :3] @ t_tc[:3, 3]
    Ti = np.linalg.inv(Tcam)
    us, vs = [], []
    for p in pts:
        pc = Ti @ np.array([p[0], p[1], p[2], 1.0])
        if pc[2] <= 1e-6:
            return None, -1e9
        uv = K @ (pc[:3] / pc[2])
        us.append(float(uv[0]))
        vs.append(float(uv[1]))
    margin = min(min(us), min(vs), w - max(us), h - max(vs))
    return list(zip(us, vs)), float(margin)


def main(argv=None):
    ap = argparse.ArgumentParser(description='腕部看 ChArUco 板的位姿离线规划')
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--ref-frame', required=True,
                    help='任一张腕部采集目录（取 T_tool0←camera、标称姿态、内参、种子）')
    ap.add_argument('--store', required=True, help='存盘 base←board（融合版）JSON')
    ap.add_argument('--height-mm', type=float, default=550.0,
                    help='相机光心离板面的高度')
    ap.add_argument('--out', help='写 JSON 报告的位置')
    a = ap.parse_args(argv)

    store = json.load(open(a.store, encoding='utf-8'))
    chain = urkin.Chain(a.urdf)
    T_tc, R_nom, q_ref, K, w, h = bp.load_reference(a.ref_frame)

    centre, corners = board_centre_and_corners(store, a.store)
    P = solve_pose(R_nom, T_tc, centre, a.height_mm / 1000.0)
    R = R_nom.copy()
    uv, margin = project_points(P, R, T_tc, K, w, h, corners)

    Ttgt = np.eye(4)
    Ttgt[:3, :3] = R
    Ttgt[:3, 3] = P
    seed = [q_ref[j] for j in chain.move_joints]
    sol, pe, re, nfev = chain.ik(Ttgt, seed)
    lim = chain.limits
    in_lim = all(lim[j][0] <= sol[i] <= lim[j][1]
                 for i, j in enumerate(chain.move_joints))
    margin_deg = min(min(sol[i] - lim[j][0], lim[j][1] - sol[i])
                     for i, j in enumerate(chain.move_joints))
    quat = Rot.from_matrix(R).as_quat()

    report = {
        'version': 1,
        'kind': 'stage4-wrist-board-view-pose',
        'store': os.path.abspath(a.store),
        'ref_frame': os.path.abspath(a.ref_frame),
        'board_centre_mm': [round(float(v) * 1000, 2) for v in centre],
        'camera_height_above_board_mm': a.height_mm,
        'tool0_xyz_mm': [round(float(v) * 1000, 2) for v in P],
        'tool0_quat_xyzw': [round(float(v), 8) for v in quat],
        'board_corners_uv': [[round(u, 1), round(v, 1)] for u, v in uv]
                            if uv else None,
        'frame_margin_px': round(margin, 1),
        'frame_margin_ok': bool(margin >= MIN_FRAME_MARGIN_PX),
        'ik': {
            'q_rad': {j: round(float(sol[i]), 6)
                      for i, j in enumerate(chain.move_joints)},
            'pos_err_mm': round(float(pe), 4),
            'rot_err_deg': round(float(re), 4),
            'nfev': int(nfev),
            'within_soft_limits': bool(in_lim),
            'min_limit_margin_deg': round(float(np.degrees(margin_deg)), 1),
            'ok': bool(pe < 0.1 and re < 0.1 and in_lim),
        },
    }
    report['all_ok'] = bool(report['ik']['ok'] and report['frame_margin_ok'])

    print('板心 %s mm  相机高 %.1f mm' % (report['board_centre_mm'], a.height_mm))
    print('tool0 %s mm  quat %s' % (report['tool0_xyz_mm'],
                                    [round(float(v), 5) for v in quat]))
    print('四角 uv %s  边距 %.1f px %s'
          % (report['board_corners_uv'], margin,
             'OK' if report['frame_margin_ok'] else 'FAIL'))
    print('IK pe=%.4f mm re=%.4f° 限位余量 %.1f° %s'
          % (pe, re, report['ik']['min_limit_margin_deg'],
             'OK' if report['ik']['ok'] else 'FAIL'))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('→ %s' % a.out)
    return 0 if report['all_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
