#!/usr/bin/env python3
"""阶段 4 · E4a 改法 B：算「腕部倾斜对准块」的悬停姿态（纯离线，不动臂）。

**为什么要 B**：腕部相机光轴与 tool0 z 平行（实测夹具夹角 0.57°），但相机本体
在 tool0 坐标系里横偏 ``(-31.9, -85.3) mm``（tool0 系）／基座系 ``(-31.9, +85.3)``。
tool0 正对块心悬停时，块在光轴外 91 mm，0.25 m 高度上就是 21° 的偏角 —— 落在
画幅下缘外，这就是 E4a 紫/黄 9 组里 6 组「可用率 0%」的根因。

**B 的做法**：悬停点 xy 仍留在块正上方（抓取对准不变），把 tool0 绕一条**水平轴**
转过 θ，让光轴指到块顶心。绕 tool0 转会把 204.2 mm 的指尖甩出去 ≈204.2·sinθ。

本工具只做几何与 IK，**不产生任何运动**；输出给动作生成用：

    · tool0 目标位姿（xyz + 四元数）与倾斜角 θ
    · 光轴在块顶高度的落点误差（mm）
    · 块顶面四角在该姿态下投到腕部画面的 bbox / 边距（px）
    · 数值 IK 的关节解、残差、软限位余量、指尖位置与离块顶的净空

退出码：0 = 全部姿态 IK 收敛且在软限位内；1 = 有姿态不收敛/超限；2 = 输入缺失。
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

import urkin                                                      # noqa: E402

#: tool0 → **裸指尖**（2026-10-26 定案）
TOOL0_TO_BARE_FINGERTIP_M = 0.2042
#: 判据边距（``入口实验.md`` §4）：块顶面离画幅边缘 ≥20 px 才算「看得全」
MIN_FRAME_MARGIN_PX = 20
#: 倾斜角搜索上限；超过就说明几何不对劲，不许硬凑
MAX_TILT_DEG = 45.0

ARM_EDGES = [
    ('base_link', 'base_link_inertia'), ('base_link_inertia', 'shoulder_link'),
    ('shoulder_link', 'upper_arm_link'), ('upper_arm_link', 'forearm_link'),
    ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
    ('wrist_2_link', 'wrist_3_link'), ('wrist_3_link', 'flange'),
    ('flange', 'tool0'),
]
CAM_EDGES_TAIL = [('tool0', 'camera_link'), ('camera_link', 'camera_color_frame'),
                  ('camera_color_frame', 'camera_color_optical_frame')]


# ---------------------------------------------------------------- TF 读取
def _q2T(v):
    T = np.eye(4)
    T[:3, 3] = [v['translation'][k] for k in 'xyz']
    x, y, z, w = [v['rotation'][k] for k in 'xyzw']
    T[:3, :3] = Rot.from_quat([x, y, z, w]).as_matrix()
    return T


def chain_T(tf, edges):
    """按 ``[(父, 子), …]`` 顺序累乘 tf.json 里的实测变换。"""
    ok = tf.get('ok') or {}
    T = np.eye(4)
    for a, b in edges:
        key = '%s:%s' % (a, b)
        if key not in ok:
            raise SystemExit('tf.json 缺 %s' % key)
        T = T @ _q2T(ok[key])
    return T


def load_reference(frame_dir):
    """参考帧目录 → ``(T_tool0←optical, R_tool0, q_ref, K, 宽, 高)``。

    用同一帧的 tf 链与 joint_states：URDF FK 与 tf 实测在 2026-11-07 的九组采集里
    相差 <0.04 mm，这里两边都留着并**互相核对**，对不上就报错而不是挑一个用。
    """
    tf = json.load(open(os.path.join(frame_dir, 'tf_000.json'), encoding='utf-8'))
    js = json.load(open(os.path.join(frame_dir, 'joint_states_000.json'),
                        encoding='utf-8'))
    meta = json.load(open(os.path.join(frame_dir, 'meta_000.json'), encoding='utf-8'))
    T_tool0 = chain_T(tf, ARM_EDGES)
    T_opt = chain_T(tf, ARM_EDGES + CAM_EDGES_TAIL)
    q = dict(zip(js['name'], js['position']))
    ci = meta['camera_info']
    K = np.asarray(ci['K'], dtype=float).reshape(3, 3)
    return np.linalg.inv(T_tool0) @ T_opt, T_tool0[:3, :3], q, K, ci['width'], ci['height']


# ---------------------------------------------------------------- 几何
def optical_miss_mm(R_tool0, t_tc, P, block_xy, z_top):
    """tool0 姿态 ``R_tool0`` 在 ``P`` 时，光轴在 ``z_top`` 高度的落点与块心距离（mm）。"""
    R = R_tool0[:3, :3]
    C = P + R @ t_tc[:3, 3]
    d = R @ t_tc[:3, :3] @ np.array([0.0, 0.0, 1.0])
    s = (z_top - C[2]) / d[2]
    H = C[:2] + d[:2] * s
    return float(np.linalg.norm(H - block_xy) * 1000.0)


def solve_tilt(R_nom, t_tc, P, block_xy, z_top):
    """绕 tool0 处的水平轴搜倾斜角，让光轴指向块顶心。

    轴 = 水平面内与「相机→块」方向垂直的那条线；两个转向都试（转错边会把块甩得更远）。
    返回 ``(tilt_deg, sign, R_tool0, miss_mm)``。
    """
    cam_xy = (P[:2] + (R_nom @ t_tc[:3, 3])[:2])
    u = block_xy - cam_xy
    n = np.linalg.norm(u)
    if n < 1e-6:
        # 相机正好在块心正上方（横偏移为零）：不需要倾斜，光轴本来就落在块心。
        # 这是退化解，不是错误 —— 夹具若真的居中，B 案就退化成原悬停位。
        return 0.0, 1, R_nom.copy(), optical_miss_mm(R_nom, t_tc, P, block_xy, z_top)
    u = u / n
    axis = np.array([-u[1], u[0], 0.0])
    best = None
    for sign in (1.0, -1.0):
        for th in np.arange(0.0, MAX_TILT_DEG, 0.01):
            R = Rot.from_rotvec(np.radians(sign * th) * axis).as_matrix() @ R_nom
            m = optical_miss_mm(R, t_tc, P, block_xy, z_top)
            if best is None or m < best[3]:
                best = (th, sign, R, m)
    th, sign, R, m = best
    return float(th), int(sign), R, float(m)


def project_face(R_tool0, t_tc, P, block_xy, z_top, K, w, h, edge_m):
    """块顶面四角投到腕部画面 → ``(u_min, u_max, v_min, v_max, margin_px)``。"""
    Tcam = np.eye(4)
    Tcam[:3, :3] = R_tool0[:3, :3] @ t_tc[:3, :3]
    Tcam[:3, 3] = P + R_tool0[:3, :3] @ t_tc[:3, 3]
    Ti = np.linalg.inv(Tcam)
    us, vs = [], []
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        pw = np.array([block_xy[0] + dx * edge_m / 2.0,
                       block_xy[1] + dy * edge_m / 2.0,
                       z_top, 1.0])
        pc = Ti @ pw
        uv = K @ (pc[:3] / pc[2])
        us.append(uv[0])
        vs.append(uv[1])
    margin = float(min(min(us), min(vs), w - max(us), h - max(vs)))
    return float(min(us)), float(max(us)), float(min(vs)), float(max(vs)), margin


# ---------------------------------------------------------------- 汇总
def plan_one(chain, q_ref, T_tc, R_nom, K, w, h, block_xy_m, z_top_m, hover_m,
             edge_m):
    P = np.array([block_xy_m[0], block_xy_m[1],
                  z_top_m + hover_m + TOOL0_TO_BARE_FINGERTIP_M])
    th, sign, R, miss = solve_tilt(R_nom, T_tc, P, block_xy_m, z_top_m)
    bb = project_face(R, T_tc, P, block_xy_m, z_top_m, K, w, h, edge_m)
    Ttgt = np.eye(4)
    Ttgt[:3, :3] = R
    Ttgt[:3, 3] = P
    seed = [q_ref[j] for j in chain.move_joints]
    sol, pe, re, nfev = chain.ik(Ttgt, seed)
    in_lim = all(chain.limits[j][0] <= sol[i] <= chain.limits[j][1]
                 for i, j in enumerate(chain.move_joints))
    margin_deg = min(min(sol[i] - chain.limits[j][0],
                         chain.limits[j][1] - sol[i])
                     for i, j in enumerate(chain.move_joints))
    tip = P + TOOL0_TO_BARE_FINGERTIP_M * R[:, 2]
    quat = Rot.from_matrix(R).as_quat()          # x, y, z, w
    return {
        'block_xy_mm': [round(block_xy_m[0] * 1000, 2), round(block_xy_m[1] * 1000, 2)],
        'block_top_mm': round(z_top_m * 1000, 2),
        'hover_mm': round(hover_m * 1000, 2),
        'tool0_xyz_mm': [round(v * 1000, 2) for v in P],
        'tool0_quat_xyzw': [round(float(v), 8) for v in quat],
        'tilt_deg': round(th, 3),
        'tilt_sign': sign,
        'optical_miss_mm': round(miss, 3),
        'face_projection': {
            'u_min': round(bb[0], 1), 'u_max': round(bb[1], 1),
            'v_min': round(bb[2], 1), 'v_max': round(bb[3], 1),
            'margin_px': round(bb[4], 1),
            'ok': bool(bb[4] >= MIN_FRAME_MARGIN_PX),
        },
        'ik': {
            'q_rad': {j: round(float(sol[i]), 6) for i, j in enumerate(chain.move_joints)},
            'dq_from_ref_rad': {j: round(float(sol[i] - q_ref[j]), 6)
                                for i, j in enumerate(chain.move_joints)},
            'pos_err_mm': round(float(pe), 4),
            'rot_err_deg': round(float(re), 4),
            'nfev': int(nfev),
            'within_soft_limits': bool(in_lim),
            'min_limit_margin_deg': round(float(np.degrees(margin_deg)), 1),
            'ok': bool(pe < 0.1 and re < 0.1 and in_lim),
        },
        'fingertip_xyz_mm': [round(float(v) * 1000, 1) for v in tip],
        'fingertip_above_block_top_mm': round(float((tip[2] - z_top_m) * 1000), 1),
        'fingertip_swing_mm': round(
            TOOL0_TO_BARE_FINGERTIP_M * np.sin(np.radians(th)) * 1000.0, 1),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description='E4a 改法 B：悬停倾斜姿态离线规划')
    ap.add_argument('--urdf', required=True, help='板上 robot_description 导出的 URDF')
    ap.add_argument('--ref-frame', required=True,
                    help='任一张腕部采集目录（取 T_tool0←camera、标称姿态、内参、种子）')
    ap.add_argument('--block', action='append', required=True,
                    metavar='NAME,X_MM,Y_MM,TOP_MM',
                    help='块名与它在基座系里的 xy、块顶 z；可重复')
    ap.add_argument('--hover', default='50,65,80', help='悬停高度 mm，逗号分隔')
    ap.add_argument('--block-edge-mm', type=float, default=30.0,
                    help='块边长（只用于投影框，判边距）')
    ap.add_argument('--out', help='写 JSON 报告的位置')
    a = ap.parse_args(argv)

    chain = urkin.Chain(a.urdf)
    T_tc, R_nom, q_ref, K, w, h = load_reference(a.ref_frame)
    hovers = [float(v) / 1000.0 for v in a.hover.split(',') if v.strip()]
    edge_m = a.block_edge_mm / 1000.0

    blocks = []
    for spec in a.block:
        parts = spec.split(',')
        if len(parts) != 4:
            raise SystemExit('--block 要 NAME,X,Y,TOP：%s' % spec)
        blocks.append((parts[0], np.array([float(parts[1]) / 1000.0,
                                           float(parts[2]) / 1000.0]),
                       float(parts[3]) / 1000.0))

    report = {
        'version': 1,
        'kind': 'stage4-e4a-bcase-plan',
        'ref_frame': os.path.abspath(a.ref_frame),
        'urdf': os.path.abspath(a.urdf),
        'tool0_to_bare_fingertip_mm': TOOL0_TO_BARE_FINGERTIP_M * 1000.0,
        'min_frame_margin_px': MIN_FRAME_MARGIN_PX,
        't_tool0_from_camera_mm': [round(float(v) * 1000, 3) for v in T_tc[:3, 3]],
        'camera_axis_tilt_in_tool0_deg': round(
            float(np.degrees(np.arccos(np.clip(
                (T_tc[:3, :3] @ np.array([0.0, 0.0, 1.0]))[2], -1.0, 1.0)))), 4),
        'camera_offset_in_base_mm': [round(float(v) * 1000, 2)
                                     for v in (R_nom @ T_tc[:3, 3])],
        'blocks': {},
    }
    all_ok = True
    for name, xy, ztop in blocks:
        rows = []
        for hv in hovers:
            r = plan_one(chain, q_ref, T_tc, R_nom, K, w, h, xy, ztop, hv, edge_m)
            rows.append(r)
            ok = r['ik']['ok'] and r['face_projection']['ok']
            all_ok = all_ok and ok
            print('%-6s hover=%-3.0f tilt=%5.2f° miss=%5.2fmm  '
                  'u[%5.0f,%5.0f] v[%5.0f,%5.0f] margin=%5.0fpx  IK pe=%.3fmm '
                  'deg_margin=%4.0f°  tip_clear=%.1fmm  %s'
                  % (name, hv * 1000, r['tilt_deg'], r['optical_miss_mm'],
                     r['face_projection']['u_min'], r['face_projection']['u_max'],
                     r['face_projection']['v_min'], r['face_projection']['v_max'],
                     r['face_projection']['margin_px'], r['ik']['pos_err_mm'],
                     r['ik']['min_limit_margin_deg'],
                     r['fingertip_above_block_top_mm'],
                     'OK' if ok else 'FAIL'))
        report['blocks'][name] = rows

    report['all_ok'] = bool(all_ok)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('→ %s' % a.out)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
