#!/usr/bin/env python3
"""阶段 4 · E2：双相机联合标定（纯离线算，不动臂）。

一轮 = 同一时刻的**两帧**（腕部高位 + 俯视），做四件事：

1. 腕部帧 → ``stage2_pad.solve_pad`` → ``base←pad``；
2. 俯视帧 → 同一 ``solve_pad``，但外参传**单位阵** → ``topcam_optical←pad``；
3. ``base←topcam_optical``：
   - 第 1 轮没有已发布的外参 ⇒ 由上面两路相除得到（``derived``）；
   - 第 2 轮起 launch 已发 ``base_link→topcam_link`` ⇒ 用发布的那份 + 驱动实测
     的 ``link→optical`` 链（``published-tf``）。**这两者不是一回事**：前者是
     定义出来的，后者才是独立检验。
   - ``link→optical`` 一律从同一帧 ``tf.json`` 取，不用理想旋转（实测差约 15 mm）。
4. 用「实际要用的那份外参」重算 ``base←pad``，与腕部那路比（≤2 mm / ≤0.5°），
   再做覆盖检查（台面矩形、宫格四角、悬停点、两页标记是否全检出）。

第 1 轮的 two_way 是代数恒等式（相除得出的外参再乘回去），记录里会写明
``extrinsic_source=derived``，**不能**当成「两路独立一致」的证据。真正的独立
检验从第 2 轮开始（对照已发布 TF）。

退出码：0 = 全过；1 = 解出来了但判据不过；2 = 数据缺失/坏（fail-closed）。
"""

import argparse
import datetime
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                        # noqa: E402
import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage2_pad as sp                                           # noqa: E402
import topcam_marker_calib as tmc                                 # noqa: E402

DEFAULT_MAT = os.path.expanduser(
    '~/robotics/notes/ur5-vision-grasp/stage2-mat/stage2-mat.json')
#: 拾取区（base 系，米），与 ``stage2_round.py`` 的 ``pick_zone`` 同值。
PICK_ZONE = [0.25, 0.575, 0.17, 0.42]
#: 台面在 base 系的 z（mm，2026-10-02 实测 +3.88，≈0）。
TABLE_Z_MM = 3.88


# ---------------------------------------------------------------- 几何
def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    M[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return M


def _inv(T):
    return np.linalg.inv(np.asarray(T, dtype=np.float64))


def _compose(A, B):
    return np.asarray(A, dtype=np.float64) @ np.asarray(B, dtype=np.float64)


def _rel_delta(Ta, Tb):
    """``Ta`` 与 ``Tb`` 的差：平移范数（mm）与转角（度）。

    转角给两种：``angle_deg`` 是总旋转角（trace 式，任意轴都行），
    ``yaw_deg`` / ``tilt_deg`` 沿用 ``stage2_pad._pose_delta`` 的口径，
    便于与阶段 2/3 的数字直接比。
    """
    rel = _inv(Ta) @ np.asarray(Tb, dtype=np.float64)
    d = sp._pose_delta(rel)
    tr = float(np.clip((np.trace(rel[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    return {'translation_mm': float(np.linalg.norm(rel[:3, 3])) * 1000.0,
            'angle_deg': float(np.degrees(np.arccos(tr))),
            'yaw_deg': d['yaw_deg'], 'tilt_deg': d['tilt_deg'],
            't_mm': d['translation_mm']}


def _project(T_cam_obj, pts_m, K, dist):
    """物体系点（米）→ 像素。返回 ``(uv (N,2), 所有点都在相机前方)``。"""
    P = np.asarray(pts_m, dtype=np.float64).reshape(-1, 3)
    T = np.asarray(T_cam_obj, dtype=np.float64)
    z = (T[:3, :3] @ P.T).T[:, 2] + T[2, 3]
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    uv, _ = cv2.projectPoints(P, rvec, T[:3, 3], K, dist)
    return uv.reshape(-1, 2), bool(np.all(z > 0.0))


def _in_frame(uv, w, h, margin):
    uv = np.asarray(uv, dtype=np.float64)
    return bool(np.all((uv[:, 0] >= margin) & (uv[:, 0] < w - margin)
                       & (uv[:, 1] >= margin) & (uv[:, 1] < h - margin)))


# ---------------------------------------------------------------- 帧 I/O
def _p(frame_dir, name, ext, idx):
    base = name if idx is None else '%s_%03d' % (name, idx)
    return os.path.join(frame_dir, base + ext)


def load_camera_frame(frame_dir, idx=None):
    """只读 color + 内参（**不碰 TF**）。

    俯视第 1 轮还没有 ``base_link→topcam_link``，``gc.load_frame`` 会在算
    base←cam 时直接 KeyError —— 这里是刻意的逃生门：E2 要的恰恰是解出那段
    TF，所以只取图上量得到的东西（像素 + 内参 + ``tf.json`` 里的固定链）。
    ``idx`` 给定时读 burst 帧（``color_000.png``）。
    """
    with open(_p(frame_dir, 'meta', '.json', idx), encoding='utf-8') as fh:
        meta = json.load(fh)
    info = meta['camera_info']
    K = np.array(info['K'], dtype=np.float64).reshape(3, 3)
    dist = np.array(info.get('D') or [0.0] * 5, dtype=np.float64).reshape(-1, 1)
    color = np.array(Image.open(_p(frame_dir, 'color', '.png', idx)).convert('RGB'))
    return {'color': color, 'K': K, 'dist': dist,
            'width': int(info['width']), 'height': int(info['height']),
            'meta': meta}


def load_tf(frame_dir, idx=None):
    with open(_p(frame_dir, 'tf', '.json', idx), encoding='utf-8') as fh:
        return json.load(fh)


def mean_pose(Ts):
    """一组 4×4 的 SE(3) 平均：平移取均值，旋转取弦距离最小（SVD 投影）。

    多帧融合用它压噪声——单帧俯视垫位姿在 1.45 m 处的重复性只有
    ~0.4 mm / 0.18°（1σ），但逐帧会抖；融合后两路差才回到判据以内
    （2026-11-07 实测，见台账）。
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


# ---------------------------------------------------------------- 外参入口
def _extrinsic_link_from_doc(doc):
    """JSON → ``base_link←topcam_link`` 的 4×4。两种写法都认，别的拒绝。"""
    if 'R_link' in doc and 't_link' in doc:
        return _T(doc['R_link'], doc['t_link'])
    child = str(doc.get('frame_child') or doc.get('camera_frame') or '')
    if child == tmc.TOPCAM_LINK and ('R' in doc or 'q' in doc or 'rotation' in doc):
        R = (np.array(doc['R'], dtype=float) if 'R' in doc
             else tmc.quat_to_matrix(doc['q'] if 'q' in doc else doc['rotation']))
        t = np.array(doc.get('t') or doc['translation'], dtype=float)
        return _T(R, t)
    raise SystemExit('认不出这份外参文件：要给 frame_child=topcam_link 的 R/t，'
                     '或本工具写出的 R_link/t_link')


def _published_link(tf_doc):
    """从帧的 ``tf.json`` 取已发布的 ``base_link→topcam_link``；没有就 None。"""
    ok = (tf_doc or {}).get('ok') or {}
    e = ok.get('%s:%s' % ('base_link', tmc.TOPCAM_LINK))
    if e is None:
        return None
    tr, ro = e['translation'], e['rotation']
    return _T(tmc.quat_to_matrix([ro['x'], ro['y'], ro['z'], ro['w']]),
              [tr['x'], tr['y'], tr['z']])


# ---------------------------------------------------------------- 覆盖
def _grid_corners_pad_mm(meta):
    """宫格四角（垫系，mm）：格心包络外扩半个 pitch。"""
    cells = meta['grid']['cells']
    pitch = float(meta['pitch_mm'])
    xs = [float(c['center_pad_mm'][0]) for c in cells]
    ys = [float(c['center_pad_mm'][1]) for c in cells]
    x0, x1 = min(xs) - pitch / 2.0, max(xs) + pitch / 2.0
    y0, y1 = min(ys) - pitch / 2.0, max(ys) + pitch / 2.0
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _marker_corners_pad_mm(meta):
    """两页垫**所有**标记的角点（垫系，mm）。只取几何，不管检出与否。"""
    pts = []
    for page in (meta.get('pages') or {}).values():
        for mk in page.get('markers') or []:
            pts.extend(mk.get('corners_pad_mm') or [])
    return pts


def _hover_points_base_m(zone, table_z_m, block_m, hover_m):
    """5 个悬停点：拾取区四角 + 中心，z = 台面 + 块高 + 悬停高度。"""
    x0, x1, y0, y1 = zone
    z = table_z_m + block_m + hover_m
    xy = [(x0, y0), (x1, y0), (x1, y1), (x0, y1),
          ((x0 + x1) / 2.0, (y0 + y1) / 2.0)]
    return [(x, y, z) for x, y in xy]


def coverage(meta, T_base_pad, T_cam_base, K, dist, w, h,
             zone=PICK_ZONE, table_z_m=TABLE_Z_MM / 1000.0,
             block_m=0.0295, hover_m=0.080, margin=20.0,
             detected=None):
    """把台面矩形 / 宫格四角 / 悬停点（以及两页垫的全部标记角点）投到俯视画幅。

    **标记的硬门是几何**（两页每个角点都在画幅内），不是「每个码都解码成功」：
    实测（2026-11-07）俯视在 base y=+1.08 m、离垫 1.45 m，远端那页的码只有
    ~12 px，1920×1080 下也只认出 5/10（1208×720 时更少，参数放到最松也只多 1 个）。
    那是分辨率的物理上限，不是布场错。`detected` 里的检出数仍然全量记录，但只作
    诊断：垫位姿能不能解，由 `solve_pad` 自己的「每页 ≥4 个 + RMS ≤1.5 px」把守。
    """
    def _proj_base(pts_m):
        P = np.asarray(pts_m, dtype=np.float64).reshape(-1, 3)
        Ph = np.column_stack([P, np.ones(len(P))])
        in_cam = (np.asarray(T_cam_base, dtype=np.float64) @ Ph.T).T[:, :3]
        uv, front = _project(np.eye(4), in_cam, K, dist)
        return uv, front

    zone = [float(v) for v in zone]
    z = table_z_m
    pick = [(zone[0], zone[2], z), (zone[1], zone[2], z),
            (zone[1], zone[3], z), (zone[0], zone[3], z)]
    uv_pick, front_pick = _proj_base(pick)

    gc_pad = _grid_corners_pad_mm(meta)
    T_bp = np.asarray(T_base_pad, dtype=np.float64)
    gc_base = [(T_bp @ np.array([x / 1000.0, y / 1000.0, 0.0, 1.0]))[:3]
               for x, y in gc_pad]
    uv_grid, front_grid = _proj_base(gc_base)

    uv_hover, front_hover = _proj_base(
        _hover_points_base_m(zone, table_z_m, block_m, hover_m))

    def _block(uv, front):
        return {'uv_px': [[round(float(u), 2), round(float(v), 2)] for u, v in uv],
                'in_front': front,
                'in_frame': bool(front) and _in_frame(uv, w, h, margin)}

    # 两页垫的每个标记角点都必须落在画幅内（几何可见性，与能否解码无关）
    mk_pad = _marker_corners_pad_mm(meta)
    if mk_pad:
        mk_base = [(T_bp @ np.array([x / 1000.0, y / 1000.0, 0.0, 1.0]))[:3]
                   for x, y in mk_pad]
        uv_mk, front_mk = _proj_base(mk_base)
        uv_arr = np.asarray(uv_mk, dtype=np.float64)
        pad_markers = {
            'n_markers': len(mk_pad) // 4,
            'in_frame': bool(front_mk) and _in_frame(uv_mk, w, h, margin),
            'uv_bbox_px': [[round(float(uv_arr[:, 0].min()), 2),
                            round(float(uv_arr[:, 1].min()), 2)],
                           [round(float(uv_arr[:, 0].max()), 2),
                            round(float(uv_arr[:, 1].max()), 2)]]}
    else:
        # 垫定义里没有标记几何 ⇒ 这一条判不了。写成 None 而不是 True：
        # 下游能看出「没判」，不会把它当成通过。
        pad_markers = {'n_markers': 0, 'in_frame': None}

    markers = None
    if detected is not None:
        markers = {
            'page_a': {'n': len(detected.get('a', [])),
                       'missing_ids': detected.get('a_missing', [])},
            'page_b': {'n': len(detected.get('b', [])),
                       'missing_ids': detected.get('b_missing', [])},
        }
        # 只作诊断：远端那页的码在 1.45 m 外只有 ~12 px，解不出来是分辨率的
        # 物理上限（证据见函数头）。硬门看上面的 pad_markers 几何 + solve_pad
        # 自己的「每页 ≥4 个、RMS ≤1.5 px」。
        markers['all_detected'] = bool(
            not markers['page_a']['missing_ids']
            and not markers['page_b']['missing_ids'])

    res = {
        'image_wh': [int(w), int(h)], 'margin_px': float(margin),
        'pick_zone_base_mm': [zone[0] * 1000, zone[1] * 1000,
                              zone[2] * 1000, zone[3] * 1000],
        'pick_zone': _block(uv_pick, front_pick),
        'grid_corners': _block(uv_grid, front_grid),
        'hover_points': _block(uv_hover, front_hover),
        'pad_markers': pad_markers,
    }
    if markers is not None:
        res['markers'] = markers
    ok = bool(res['pick_zone']['in_frame'] and res['grid_corners']['in_frame']
              and res['hover_points']['in_frame'])
    if pad_markers['in_frame'] is not None:
        ok = ok and pad_markers['in_frame']
    res['ok'] = ok
    return res


# ---------------------------------------------------------------- 主流程
def run(args):
    with open(args.mat, encoding='utf-8') as fh:
        meta = json.load(fh)

    # 帧目录可以是单帧（color.png）也可以是 burst（color_000.png …）：
    # 逐帧解垫位姿，再融合。单帧时 n=1，行为与旧版完全一致。
    w_idx = gc.list_frame_indices(args.wrist_frame)
    t_idx = gc.list_frame_indices(args.topcam_frame)
    reasons = []
    if not w_idx:
        reasons.append('腕部帧目录里没有帧：%s' % args.wrist_frame)
    if not t_idx:
        reasons.append('俯视帧目录里没有帧：%s' % args.topcam_frame)

    # 逐帧解 → 只留解的出来的帧。**单帧解失败不得把整串 burst 判死**：
    # 20–40 帧是同一个静止场景，某一帧被机械臂挡住一角很正常；拿最后一帧
    # 的结果当整串的结论，会把 28/40 帧都成功的 round-05 直接丢掉
    # （2026-11-07 实测：round-05 最后一帧页 B 只有 3 个标记 ⇒ 整轮 0 帧；
    # 实际 28 帧可解，两路 0.73 mm / 0.31°）。每帧自己的 RMS/标记数门槛
    # 仍由 sp.solve_pad 逐帧把守，这里只决定「这一帧算不算数」。
    w_ok, t_ok = [], []
    w_all, t_all = [], []
    wres = tres = None
    for i in w_idx or []:
        wframe = gc.load_frame(args.wrist_frame, idx=i)   # 腕部链完整，照旧
        r = sp.solve_pad(wframe['color'], wframe['K'], wframe['R_bc'],
                         wframe['t_bc'], meta,
                         max_rms_px=args.max_rms_px,
                         min_markers=args.min_markers)
        w_all.append(r)
        if r['T_base_pad'] is not None:
            w_ok.append(r['T_base_pad'])
    for i in t_idx or []:
        tframe_i = load_camera_frame(args.topcam_frame, idx=i)
        r = sp.solve_pad(tframe_i['color'], tframe_i['K'], np.eye(3),
                         np.zeros(3), meta,
                         max_rms_px=args.max_rms_px,
                         min_markers=args.min_markers)
        t_all.append(r)
        if r['T_base_pad'] is not None:
            t_ok.append(r['T_base_pad'])
    # 对外报告用的代表帧：优先第一帧解出来的（标记数只用来描述布场，
    # 不再当作整串的判决）；全都解不出来时退回最后一帧，好把失败原因说清楚。
    wres = next((r for r in w_all if r['T_base_pad'] is not None),
                w_all[-1] if w_all else None)
    tres = next((r for r in t_all if r['T_base_pad'] is not None),
                t_all[-1] if t_all else None)

    tframe = load_camera_frame(args.topcam_frame, idx=t_idx[-1]) if t_idx else None
    tf_doc = load_tf(args.topcam_frame, idx=t_idx[-1]) if t_idx else None

    if w_idx and not w_ok:
        reasons.append('腕部 %d 帧没有一帧解出来：%s'
                       % (len(w_idx), (wres or {}).get('checks', {})
                          .get('reason', '未知')))
    if t_idx and not t_ok:
        reasons.append('俯视 %d 帧没有一帧解出来：%s'
                       % (len(t_idx), (tres or {}).get('checks', {})
                          .get('reason', '未知')))

    # link→optical：必须用驱动实测的那条链
    try:
        R_lo, t_lo = tmc.chain_from_tf(tf_doc, tmc.TOPCAM_LINK_TO_OPTICAL_KEYS)
        lo_source = 'tf-json'
    except SystemExit as exc:
        reasons.append('link→optical 取不到：%s' % exc)
        R_lo, t_lo = tmc.R_LINK_TO_OPTICAL.copy(), np.zeros(3)
        lo_source = 'ideal(fallback)'
    T_link_optical = _T(R_lo, t_lo)

    if reasons:
        return {'ok': False, 'fatal': True, 'reasons': reasons,
                'wrist': ({'ok': bool(wres['ok']), 'checks': wres['checks']}
                          if wres else None),
                'topcam': ({'ok': bool(tres['ok']), 'checks': tres['checks']}
                           if tres else None),
                'link_to_optical': {'source': lo_source}}

    T_base_pad_w = mean_pose(w_ok)
    T_cam_pad = mean_pose(t_ok)
    frames = {}
    for name, P, allr in (('wrist', w_ok, w_all), ('topcam', t_ok, t_all)):
        m = mean_pose(P)
        spread_mm = max(float(np.linalg.norm(np.asarray(T)[:3, 3] - m[:3, 3]))
                        * 1000.0 for T in P)
        spread_deg = max(_rel_delta(m, np.asarray(T, dtype=np.float64))
                         ['angle_deg'] for T in P)
        # 每页标记数的分布：俯视看垫子常被机械臂挡掉几个，这组数就是
        # 「这一轮的靶几何有多弱」的唯一现场证据，必须逐轮留档。
        pages = {}
        for page in ('page_a', 'page_b'):
            ns = [int(((r['checks'].get(page) or {}).get('n_markers')) or 0)
                  for r in allr]
            if ns:
                pages[page + '_min'] = min(ns)
                pages[page + '_median'] = float(np.median(ns))
        frames[name] = {'n_frames': len(P), 'n_attempted': len(allr),
                        'n_failed': len(allr) - len(P),
                        'spread_mm': round(spread_mm, 3),
                        'spread_deg': round(spread_deg, 3)}
        frames[name].update(pages)

    # base←topcam_link：优先已发布的（独立检验）；第 1 轮没有就得自己解
    T_pub_link = None
    pub_source = None
    if args.base_topcam_link:
        with open(args.base_topcam_link, encoding='utf-8') as fh:
            T_pub_link = _extrinsic_link_from_doc(json.load(fh))
        pub_source = 'file:%s' % args.base_topcam_link
    elif args.derive_extrinsic:
        # 第 1 轮：**忽略**帧里已发布的 TF，强制两路相除。
        # 不加这个开关的话，只要 launch 发过 TF，下面 _published_link 一定会命中，
        # 导出支永远走不到 —— 导出的「derived」文件其实是已发布值原样抄回
        # （2026-11-07 实测：R_link/t_link 与 launch 参数逐位相同）。
        pub_source = 'ignored(--derive-extrinsic)'
    else:
        T_pub_link = _published_link(tf_doc)
        if T_pub_link is not None:
            pub_source = 'tf-json:base_link:topcam_link'

    if T_pub_link is not None:
        T_base_cam = _compose(T_pub_link, T_link_optical)
        ext_source = 'published-tf'
    else:
        T_base_cam = _compose(T_base_pad_w, _inv(T_cam_pad))
        ext_source = 'derived-this-round'

    T_base_pad_t = _compose(T_base_cam, T_cam_pad)
    two_way = _rel_delta(T_base_pad_w, T_base_pad_t)
    two_way['thresholds'] = {'translation_mm': float(args.max_trans_mm),
                             'angle_deg': float(args.max_rot_deg)}
    two_way['ok'] = bool(two_way['translation_mm'] <= args.max_trans_mm
                         and two_way['angle_deg'] <= args.max_rot_deg)

    # 覆盖用「实际要用的那份外参」投，不用俯视自己解的那份
    T_cam_base_used = _inv(T_base_cam)
    detected = {
        'a': tres['checks']['page_a']['ids'],
        'b': tres['checks']['page_b']['ids'],
        'a_missing': tres['checks']['page_a']['missing_ids'],
        'b_missing': tres['checks']['page_b']['missing_ids'],
    }
    cov = coverage(meta, T_base_pad_w, T_cam_base_used,
                   tframe['K'], tframe['dist'],
                   tframe['width'], tframe['height'],
                   zone=args.pick_zone, table_z_m=args.table_z_mm / 1000.0,
                   block_m=args.block_mm / 1000.0,
                   hover_m=args.hover_mm / 1000.0,
                   margin=args.margin_px, detected=detected)

    T_base_link = _compose(T_base_cam, _inv(T_link_optical))

    # 单帧时 wres/tres 就是那唯一一帧；burst 时用的是「解出来的那些帧」的
    # 融合位姿，逐帧是否全过已经由 sp.solve_pad 的 RMS/标记数把关。
    ok = bool(w_ok and t_ok and two_way['ok'] and cov['ok'])
    if wres['ok'] is False and len(w_idx) == 1:
        reasons.append('腕部判据不过：%s' % wres['checks'].get('reason', ''))
    if tres['ok'] is False and len(t_idx) == 1:
        reasons.append('俯视判据不过：%s' % tres['checks'].get('reason', ''))
    if not two_way['ok']:
        reasons.append('两路不一致：%.2f mm / %.3f°（上限 %.2f mm / %.2f°）'
                       % (two_way['translation_mm'], two_way['angle_deg'],
                          args.max_trans_mm, args.max_rot_deg))
    if not cov['ok']:
        bad = [k for k in ('pick_zone', 'grid_corners', 'hover_points',
                           'pad_markers') if cov[k]['in_frame'] is False]
        reasons.append('覆盖不过：%s' % (', '.join(bad) or '未知'))

    return {
        'version': 1, 'kind': 'stage4-joint-calib',
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
        'inputs': {'wrist_frame': args.wrist_frame,
                   'topcam_frame': args.topcam_frame, 'mat': args.mat},
        'wrist': {'ok': bool(wres['ok']), 'checks': wres['checks'],
                  'T_base_pad': T_base_pad_w.tolist()},
        'topcam': {'ok': bool(tres['ok']), 'checks': tres['checks'],
                   'T_cam_pad': T_cam_pad.tolist(),
                   'T_base_pad_from_cam': T_base_pad_t.tolist()},
        'frames': frames,
        'link_to_optical': {'source': lo_source,
                            'R': R_lo.tolist(), 't': t_lo.tolist()},
        'extrinsic': {'source': ext_source,
                      'published_source': pub_source,
                      'frame_parent': 'base_link',
                      'frame_child': tmc.TOPCAM_LINK + '→' + gc.TOPCAM_TIP,
                      'R': T_base_cam[:3, :3].tolist(),
                      't': T_base_cam[:3, 3].tolist(),
                      'q': tmc.matrix_to_quat(T_base_cam[:3, :3]).tolist(),
                      'T_base_optical': T_base_cam.tolist()},
        'extrinsic_link': {'R_link': T_base_link[:3, :3].tolist(),
                           't_link': T_base_link[:3, 3].tolist()},
        'two_way': two_way,
        'coverage': cov,
        'ok': ok, 'reasons': reasons,
        'note': ('第 1 轮 extrinsics 是相除导出的：two_way 是代数恒等式，'
                 '不是独立证据；第 2 轮起为 published-tf，才是真检验。'
                 if ext_source == 'derived-this-round' else ''),
    }


def extrinsics_doc(res, joint_path, created=None):
    """joint-calib 结果 → ``camera-extrinsics`` 文档（光学系 R/t + link 系 R/t）。

    ``frame_child`` 写成**光学系**：``topcam_marker_calib.base_from_camera``
    只认光学系或 ``*_link``，写成 ``a→b`` 这种会直接被拒。
    """
    ext, lk = res['extrinsic'], res['extrinsic_link']
    return {
        'version': 1, 'kind': 'camera-extrinsics',
        'frame_parent': 'base_link',
        'frame_child': gc.TOPCAM_TIP,
        'R': ext['R'], 't': ext['t'], 'q': ext['q'],
        'R_link': lk['R_link'], 't_link': lk['t_link'],
        'source': 'stage4-joint-calib:%s' % ext['source'],
        'joint_calib': joint_path,
        'inputs': res.get('inputs'),
        'link_to_optical_source': res['link_to_optical']['source'],
        'created': (created or
                    datetime.datetime.now().isoformat(timespec='seconds')),
    }


def _frames_note(frames, side):
    fr = (frames or {}).get(side)
    if not fr or fr['n_frames'] <= 1:
        return ''
    return '，%d 帧融合（逐帧散布 %.2f mm / %.3f°）' % (
        fr['n_frames'], fr['spread_mm'], fr['spread_deg'])


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='阶段 4 E2：双相机联合标定（只读，不动臂）')
    ap.add_argument('--wrist-frame', required=True)
    ap.add_argument('--topcam-frame', required=True)
    ap.add_argument('--mat', default=DEFAULT_MAT,
                    help='垫子 sidecar stage2-mat.json（仓外）')
    ap.add_argument('--out', default=None,
                    help='默认 <wrist-frame>/../joint-calib.json')
    ap.add_argument('--extrinsics-out', default=None,
                    help='通过时再写一份 camera-extrinsics（可直接喂 '
                         'topcam_marker_calib --init --extrinsics 与 '
                         'topcam_extrinsic_gate --prev-extrinsics）')
    ap.add_argument('--base-topcam-link', default=None,
                    help='已发布的 base_link←topcam_link JSON；不给则从帧 tf.json 找，'
                         '再没有就按第 1 轮相除导出')
    ap.add_argument('--derive-extrinsic', action='store_true',
                    help='★ 强制走「两路相除导出」，忽略帧 tf.json 里的 '
                         'base_link→topcam_link（E2 第 1 轮用）。'
                         '没有它时，只要 launch 发过 TF 导出支就永远走不到，'
                         '导出的文件其实是把已发布值原样抄回。')
    ap.add_argument('--max-trans-mm', type=float, default=2.0)
    ap.add_argument('--max-rot-deg', type=float, default=0.5)
    ap.add_argument('--max-rms-px', type=float, default=sp.MAX_RMS_PX)
    ap.add_argument('--min-markers', type=int, default=sp.MIN_MARKERS)
    ap.add_argument('--margin-px', type=float, default=20.0)
    ap.add_argument('--pick-zone', type=float, nargs=4, default=PICK_ZONE,
                    metavar=('X0', 'X1', 'Y0', 'Y1'),
                    help='拾取区（base 系，米）；默认与 stage2_round 同值')
    ap.add_argument('--table-z-mm', type=float, default=TABLE_Z_MM)
    ap.add_argument('--block-mm', type=float, default=29.5)
    ap.add_argument('--hover-mm', type=float, default=80.0)
    args = ap.parse_args(argv)

    try:
        res = run(args)
    except (OSError, ValueError, KeyError) as exc:
        print('数据缺失/坏：%s' % exc)
        return 2

    if res.get('fatal'):
        for r in res.get('reasons', []):
            print('!! %s' % r)
        return 1

    out = args.out or os.path.join(os.path.dirname(
        os.path.abspath(args.wrist_frame)), 'joint-calib.json')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)

    if args.extrinsics_out:
        if res['ok']:
            edoc = extrinsics_doc(res, out)
            os.makedirs(os.path.dirname(os.path.abspath(args.extrinsics_out)),
                        exist_ok=True)
            with open(args.extrinsics_out, 'w', encoding='utf-8') as fh:
                json.dump(edoc, fh, ensure_ascii=False, indent=1)
            print('写外参 %s（可直接喂 --init --extrinsics）'
                  % args.extrinsics_out)
        else:
            print('判红：不写 --extrinsics-out（坏外参不许进链）')

    tw = res['two_way']
    cov = res['coverage']
    fr = res.get('frames') or {}
    print('腕部：RMS %.2f px（%s）%s'
          % (res['wrist']['checks'].get('rms_adopted_px', float('nan')),
             res['wrist']['checks'].get('adopted'), _frames_note(fr, 'wrist')))
    print('俯视：RMS %.2f px（%s）%s'
          % (res['topcam']['checks'].get('rms_adopted_px', float('nan')),
             res['topcam']['checks'].get('adopted'), _frames_note(fr, 'topcam')))
    print('外参来源：%s（link→optical：%s）'
          % (res['extrinsic']['source'], res['link_to_optical']['source']))
    print('两路差：%.2f mm / %.3f°（yaw %+.3f°，tilt %.3f°）%s'
          % (tw['translation_mm'], tw['angle_deg'], tw['yaw_deg'],
             tw['tilt_deg'], '' if tw['ok'] else '  ← 超限'))
    print('覆盖：拾取区 %s、宫格 %s、悬停点 %s、两页垫标记几何 %s → %s'
          % (cov['pick_zone']['in_frame'], cov['grid_corners']['in_frame'],
             cov['hover_points']['in_frame'], cov['pad_markers']['in_frame'],
             'OK' if cov['ok'] else 'FAIL'))
    if cov.get('markers'):
        mk = cov['markers']
        print('  （诊断，不判分）标记检出：页A %d 缺 %s；页B %d 缺 %s'
              % (mk['page_a']['n'], mk['page_a']['missing_ids'] or '无',
                 mk['page_b']['n'], mk['page_b']['missing_ids'] or '无'))
    if res['note']:
        print('注：%s' % res['note'])
    print('写入 %s' % out)
    if not res['ok']:
        for r in res['reasons']:
            print('判红：%s' % r)
        return 1
    print('写外参到 launch 的参数（base_link←topcam_link）：')
    print(tmc.fmt_launch(np.array(res['extrinsic_link']['R_link']),
                         np.array(res['extrinsic_link']['t_link']),
                         prefix='topcam'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
