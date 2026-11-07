#!/usr/bin/env python3
"""阶段 4 · 用**腕部看板**裁定俯视外参（纯离线算，不动臂、不碰相机）。

**为什么要第三条路线**：俯视外参现在有两条路线差 4.4 mm ——

    · 垫 PnP 相除        t=(0.387649, 1.078836, 0.801255)
    · ChArUco 板路线     t=(0.390584, 1.078065, 0.804260)   ← launch 在用的

两条都是「在某一台相机里量到的位姿」再相除/相乘，而且**垫本身不平**（腕部量出来
两页垫的倾角差 1.2–6.4°、Δz −3.7…+5.3 mm），所以谁也裁不了谁。

第三条路线不经过垫、也不需要两路同时看：

    腕部→板   base←board        （近距离、板 150 mm、角点全）
    俯视→板   cam←board         （存盘里已有：16 角点 × 20 帧融合，RMS 0.32 px）
    复合      base←topcam = (base←board) ∘ (cam←board)⁻¹

板静止 ⇒ 两路**不必同时**拍；缺的只是腕部一次「看得全」的采集（E2 那批腕部帧
只看到 7–12 个角点、RMS 12–25 px，用不了）。

⚠️ 这里假设「板自那次存盘之后没动过」。输出的数**不是**外参的事实源，必须再过
E3b 的端到端两路垫 PnP（≤2 mm/0.5°）才算通过。

用法：

    python3 scripts/stage4_topcam_from_board.py \\
        --wrist-burst runs/<run>/wrist-board --board docs/calib/charuco_A4_30mm_5x5.json \\
        --store runs/<run>/marker-store-board-moved-fused.json \\
        --out runs/<run>/topcam-from-board.json

退出码：0 = 解得出来（并打印与旧外参的差）；2 = 数据不足/输入缺失。
"""

import argparse
import glob
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage4_joint_calib as s4jc                                 # noqa: E402
import topcam_marker_calib as tmc                                 # noqa: E402

REPO = os.path.dirname(HERE)


def _p(d, name, ext, idx):
    return os.path.join(d, name if idx is None else '%s_%03d' % (name, idx) + ext)


def frame_indices(burst_dir):
    hits = sorted(glob.glob(os.path.join(burst_dir, 'color_*.png')))
    if hits:
        return sorted(int(h.rsplit('_', 1)[1].split('.')[0]) for h in hits)
    if os.path.exists(os.path.join(burst_dir, 'color.png')):
        return [None]
    return []


def fuse_base_board(burst_dir, board_meta):
    """腕部 burst → ``base←board`` 的融合位姿 + 逐帧统计。

    每帧：``cam←board``（ChArUco）∘ ``base←cam``（帧内 TF 链）。**逐帧保留**，
    只在一帧都没解出来时判死 —— E2 轮 05 就是「只看末帧」把 28 帧可解记成了 0 帧。
    """
    Ts, rows, fails = [], [], 0
    for idx in frame_indices(burst_dir):
        meta = json.load(open(_p(burst_dir, 'meta', '.json', idx), encoding='utf-8'))
        info = meta['camera_info']
        K = np.asarray(info['K'], dtype=float).reshape(3, 3)
        dist = np.asarray(info.get('D') or [0.0] * 5, dtype=float).reshape(-1, 1)
        det = tmc.detect_board(_p(burst_dir, 'color', '.png', idx),
                               board_meta, K, dist)
        if det is None:
            fails += 1
            rows.append({'idx': idx, 'ok': False})
            continue
        (R_cb, t_cb), n_pts, rms = det
        tf_doc = json.load(open(_p(burst_dir, 'tf', '.json', idx),
                                encoding='utf-8'))
        R_bc, t_bc = gc.cam_pose_tf(tf_doc['ok'], tip=gc.TIP, chain=gc.CHAIN)
        T = s4jc._compose(s4jc._T(R_bc, t_bc), s4jc._T(R_cb, t_cb))
        Ts.append(T)
        rows.append({'idx': idx, 'ok': True, 'n_corners': int(n_pts),
                     'rms_px': round(float(rms), 3)})
    if not Ts:
        return None, {'n_attempted': len(rows), 'n_failed': fails, 'frames': rows}
    T_f = s4jc.mean_pose(Ts)
    dev_t = max(float(np.linalg.norm(T[:3, 3] - T_f[:3, 3])) * 1000 for T in Ts)
    dev_a = max(float(np.degrees(np.linalg.norm(
        Rot.from_matrix(T[:3, :3].T @ T_f[:3, :3]).as_rotvec()))) for T in Ts)
    ok_rows = [r for r in rows if r['ok']]
    stat = {'n_attempted': len(rows), 'n_failed': fails, 'n_used': len(Ts),
            'corner_min': min(r['n_corners'] for r in ok_rows),
            'corner_median': int(np.median([r['n_corners'] for r in ok_rows])),
            'rms_median_px': round(float(np.median([r['rms_px'] for r in ok_rows])), 3),
            'max_dev_mm': round(dev_t, 3), 'max_dev_deg': round(dev_a, 4),
            'frames': rows}
    return T_f, stat


def link_from_optical(doc):
    """旧外参文档 → ``link←optical``（米/矩阵）。文档里没有 link 系就退回理想旋转并标明。"""
    if 'R_link' in doc and 't_link' in doc:
        Rb_l = s4jc._T(np.asarray(doc['R_link'], dtype=float),
                       np.asarray(doc['t_link'], dtype=float))
        Rb_o = s4jc._T(np.asarray(doc['R'], dtype=float),
                       np.asarray(doc['t'], dtype=float))
        return s4jc._compose(s4jc._inv(Rb_l), Rb_o), 'doc'
    return s4jc._T(tmc.R_LINK_TO_OPTICAL, np.zeros(3)), 'ideal'


def compose_extrinsic(base_board_wrist, store, old_doc):
    """``base←board``(腕) ∘ ``cam←board``⁻¹ → 新的 ``base←topcam``（光学系 + link 系）。"""
    T_base_board_store = s4jc._T(
        np.asarray(store['base_from_board_R'], dtype=float),
        np.asarray(store['base_from_board_t'], dtype=float))
    R_old, t_old = tmc.base_from_camera(old_doc)
    base_cam_old = s4jc._T(R_old, t_old)
    cam_board_store = s4jc._compose(s4jc._inv(base_cam_old), T_base_board_store)
    base_cam_new = s4jc._compose(base_board_wrist, s4jc._inv(cam_board_store))
    T_lo, src = link_from_optical(old_doc)
    base_link_new = s4jc._compose(base_cam_new, s4jc._inv(T_lo))
    return base_cam_new, base_link_new, (R_old, t_old), src


def main(argv=None):
    ap = argparse.ArgumentParser(description='用腕部看板裁定俯视外参')
    ap.add_argument('--wrist-burst', required=True,
                    help='腕部看板的采集目录（precision_capture --burst）')
    ap.add_argument('--board', required=True, help='板规格 JSON（含 board_w_mm）')
    ap.add_argument('--store', required=True, help='存盘 base←board（融合版）')
    ap.add_argument('--extrinsics', default=None,
                    help='旧外参文档；缺省 = 存盘里 from_extrinsics 记的那份')
    ap.add_argument('--out', help='写结果 JSON 的位置')
    a = ap.parse_args(argv)

    board_meta = json.load(open(a.board, encoding='utf-8'))
    store = json.load(open(a.store, encoding='utf-8'))
    ext_path = a.extrinsics or store.get('from_extrinsics')
    if not ext_path:
        raise SystemExit('存盘没有 from_extrinsics，必须显式给 --extrinsics')
    if not os.path.isabs(ext_path):
        ext_path = os.path.join(REPO, ext_path)
    if not os.path.exists(ext_path):
        raise SystemExit('旧外参文档不存在：%s' % ext_path)
    old_doc = json.load(open(ext_path, encoding='utf-8'))

    T_wb, stat = fuse_base_board(a.wrist_burst, board_meta)
    if T_wb is None:
        print('!! 腕部 burst 一帧都没解出来（试了 %d 帧）—— 先看板是不是全在画幅里、'
              '是不是被夹爪挡了' % stat['n_attempted'], file=sys.stderr)
        return 2

    base_cam_new, base_link_new, (R_old, t_old), link_src = compose_extrinsic(
        T_wb, store, old_doc)
    R_new, t_new = base_cam_new[:3, :3], base_cam_new[:3, 3]
    R_link, t_link = base_link_new[:3, :3], base_link_new[:3, 3]

    d_t = float(np.linalg.norm(t_new - t_old)) * 1000.0
    d_a = float(np.degrees(np.linalg.norm(
        Rot.from_matrix(R_old.T @ R_new).as_rotvec())))
    launch = tmc.fmt_launch(R_link, t_link, prefix='topcam')

    report = {
        'version': 1,
        'kind': 'stage4-topcam-extrinsic-from-board',
        'wrist_burst': os.path.abspath(a.wrist_burst),
        'store': os.path.abspath(a.store),
        'old_extrinsics': os.path.abspath(ext_path),
        'link_from_optical_source': link_src,
        'wrist_base_board': {'R': T_wb[:3, :3].tolist(), 't': T_wb[:3, 3].tolist(),
                             **stat},
        'new_base_topcam_optical': {'R': R_new.tolist(),
                                    't': [float(v) for v in t_new]},
        'new_base_topcam_link': {'R': R_link.tolist(),
                                 't': [float(v) for v in t_link],
                                 'q': [float(v) for v in tmc.matrix_to_quat(R_link)]},
        'delta_vs_old': {'translation_mm': round(d_t, 4),
                         'angle_deg': round(d_a, 4)},
        'launch_args': launch,
        'note': '必须再过 E3b 端到端两路垫 PnP 复核（≤2 mm/0.5°）才能进 launch',
    }
    print('腕部看板：%d/%d 帧可用，角点 min/中位 %d/%d，RMS 中位 %s px，'
          '帧间最大偏离 %.3f mm / %.3f°'
          % (stat['n_used'], stat['n_attempted'], stat['corner_min'],
             stat['corner_median'], stat['rms_median_px'],
             stat['max_dev_mm'], stat['max_dev_deg']))
    print('base←topcam(光学)：t = %s m' % np.round(t_new, 6).tolist())
    print('与旧外参差：%.3f mm / %.4f°（链来源 %s）' % (d_t, d_a, link_src))
    print(launch)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
            f.write('\n')
        print('→ %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
