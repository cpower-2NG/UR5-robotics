#!/usr/bin/env python3
"""阶段 2 · 从一帧 survey 图解出**垫系**并写 ``grid.json``（入口判据 ④）。

做三件事
────────
1. 同时检测两页标记（第 1 页 ``DICT_4X4_50``、第 2 页 ``DICT_5X5_50``，各 ID 0–9），
   并**按版式一致性**把同 ID 的别的板子（E3b 的 ChArUco 板也是 4×4、ID 撞车）剔掉；
2. 用 sidecar ``stage2-mat.json`` 里每个标记的**垫系**角点做 PnP；
   两页分开解一次，得到页 B 相对页 A 的**实测**位移 ``T_A_B``（不假设拼贴对齐）；
3. 用 ``T_A_B`` 把页 B 的角点搬到页 A 系，做联合 PnP（只在重投影不比单页差时才采用），
   再按 ``T_base_pad`` 把 9 个格心算到 base 系，写 ``grid.json``。

垫系（与 ``stage2_mat.build_meta`` 一致）
────────────────────────────────────────
原点 = 宫格中心；x = 页面 +x；y = 页面 −y（朝上）；z = 纸面外（朝相机）。
因此 marker 角点的规范顺序（TL,TR,BR,BL，页面 y 向下）在垫系里就是顺时针，
与 ArUco 检出的角点顺序**手性一致**——这里不做任何翻转，翻转会解出镜像位姿。

为什么页 B 不直接用名义位置
──────────────────────────
两页是**人拼贴**的：重叠 15 mm、上缘对齐全靠手。实测 ``T_A_B`` 若偏离单位阵，
说明页 B 真的歪了/偏了；直接把名义坐标喂给联合 PnP，会把这点拼贴误差摊到
整个垫系上。先测出来、再搬过去，才叫「视觉实测，不假设对齐」。

E2 用法（阶段 4，俯视那一路）
────────────────────────────
``--camera topcam`` 换成俯视帧的 TF 链与光学系 tip；再加 ``--cam-identity``
就把 ``R_bc=I, t_bc=0`` 传给 ``solve_pad``，于是结果的语义**不再是 base←pad，
而是 topcam_color_optical←pad**（相机系里解垫）。E2 要的正是后者：腕部帧给
base←pad，俯视帧给 cam←pad，两者相除才得到 base←topcam。屏幕会大字标出来，
默认输出文件名也换成 ``T_cam_pad.json``——**不要**把它当 grid.json 用。

退出码：0 = 判据全过；1 = 解出来了但有判据不过；2 = 数据缺失/坏。
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                        # noqa: E402
import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage2_aruco as sa                                         # noqa: E402

#: 重投影 RMS 上限（px）。28 mm 标记在 0.34 m 处约 75 px，1.5 px≈0.5 mm。
MAX_RMS_PX = 1.5
#: 每页至少要检出几个标记才解 PnP（4 个 = 单应最小解）。
MIN_MARKERS = 4


# ---------------------------------------------------------------- 几何工具
def _apply(T, pts):
    """4×4 变换作用在 (N,2) 平面点（z=0）上，返回 (N,2)。单位 mm。"""
    P = np.asarray(pts, dtype=np.float64)
    if P.ndim == 1:
        P = P.reshape(1, 2)
    H = np.column_stack([P, np.zeros(len(P)), np.ones(len(P))])
    Q = (T @ H.T).T
    return Q[:, :2]


def _inv(T):
    return np.linalg.inv(np.asarray(T, dtype=np.float64))


def _yaw_of(T):
    """变换的 z 轴转角（度，base 系里 x 轴方向）。"""
    R = np.asarray(T, dtype=np.float64)[:3, :3]
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def _translation_mm(T):
    return [float(v) * 1000.0 for v in np.asarray(T, dtype=np.float64)[:3, 3]]


def _pose_delta(T):
    """相对单位阵的位移（mm）与转角（度）。"""
    T = np.asarray(T, dtype=np.float64)
    return {'translation_mm': _translation_mm(T),
            'yaw_deg': _yaw_of(T),
            'tilt_deg': float(np.degrees(np.arccos(
                float(np.clip(T[2, 2], -1.0, 1.0)))))}


# ---------------------------------------------------------------- 检测
def _consistent_markers(meta, key, corners, ids, K, inlier_px=3.0):
    """一页的全部检出里，只留与垫子**版式**几何一致的那些（每个 ID 一条）。

    场景里不止有垫子：E3b 的 ChArUco 板也用 ``DICT_4X4_50``，ID 与页 A 撞车。
    旧写法 ``{id: corners}`` 让「检测顺序里后到的那条」静默覆盖先到的——碰巧
    全落在垫子上就解对，混进一块板子上的标记就会解出 2000+ px 的重投影误差
    （2026-11-07 俯视连拍第 0 帧实测：id 6/9 被 ChArUco 板抢走，RMS 2437 px）。
    这里拿 sidecar 的物点做 RANSAC：两条不同 ID 的检出起假设，重投影
    ≤``inlier_px`` 的算内点，取内点最多（并列取 RMS 小）的那一簇。
    """
    idx = _marker_index(meta, key)
    dets = []
    for c, i in zip(corners, ids):
        mid = int(i)
        if mid not in idx:
            continue
        dets.append({'id': mid,
                     'img': np.asarray(c, dtype=np.float64).reshape(4, 2),
                     'obj': np.asarray(idx[mid]['corners_pad_mm'],
                                       dtype=np.float64).reshape(4, 2) / 1000.0})
    if len(dets) < 2:
        return {d['id']: d['img'] for d in dets}
    # 没有重复 ID ⇒ 没有歧义可消。保持旧行为（每个检出都进 PnP），这样
    # 「一个角点画错 5 mm」仍然照旧判红，而不是被当成外点悄悄剔掉。
    if len({d['id'] for d in dets}) == len(dets):
        return {d['id']: d['img'] for d in dets}

    best = None
    for i in range(len(dets)):
        for j in range(i + 1, len(dets)):
            if dets[i]['id'] == dets[j]['id']:
                continue
            obj = np.vstack([dets[i]['obj'], dets[j]['obj']])
            img = np.vstack([dets[i]['img'], dets[j]['img']])
            try:
                rvec, tvec, _ = sa.solve_plane_pnp(obj, img, K)
            except (ValueError, cv2.error):
                continue
            inl = []
            for d in dets:
                uv = sa.project_points(d['obj'], rvec, tvec, K)
                err = float(np.sqrt(np.mean(np.sum((uv - d['img']) ** 2,
                                                   axis=1))))
                if err <= inlier_px:
                    inl.append((d, err))
            if len(inl) < 2:
                continue
            rms = float(np.sqrt(np.mean([e * e for _, e in inl])))
            score = (len(inl), -rms)
            if best is None or score > best[0]:
                best = (score, inl)
    if best is None:
        # 一组都拟合不上：原样交出去，让上层按重投影判红（fail-closed）。
        return {d['id']: d['img'] for d in dets}

    out = {}
    for d, err in best[1]:
        if d['id'] not in out or err < out[d['id']][1]:
            out[d['id']] = (d['img'], err)
    return {mid: v[0] for mid, v in out.items()}


def detect_pages(gray, meta, K):
    """按页分别检测；返回 ``{'a': {id: corners(4,2)}, 'b': {...}}``。

    ``K`` 只用来在多簇同 ID 检出里判版式一致性（见 ``_consistent_markers``）。
    """
    out = {}
    for key in ('a', 'b'):
        dict_name = meta['pages'][key]['dictionary']
        corners, ids = sa.detect_markers(gray, dict_name)
        out[key] = (_consistent_markers(meta, key, corners, ids, K)
                    if ids is not None else {})
    return out


def _marker_index(meta, key):
    return {int(m['id']): m for m in meta['pages'][key]['markers']}


def corner_pairs(meta, key, det, transform=None):
    """检出 → (obj_pad_mm (N,2), img_px (N,2), ids)。

    ``transform`` 给定时先把角点从名义垫系搬到另一系（联合 PnP 用）。
    没检测到或不在 sidecar 里的 ID 直接跳过并计数。
    """
    idx = _marker_index(meta, key)
    obj, img, ids, missing = [], [], [], []
    for mid in sorted(idx):
        c = det.get(mid)
        if c is None:
            missing.append(mid)
            continue
        pts = np.asarray(idx[mid]['corners_pad_mm'], dtype=np.float64)
        if transform is not None:
            pts = _apply(transform, pts)
        obj.extend(pts.tolist())
        img.extend(np.asarray(c, dtype=np.float64).tolist())
        ids.append(mid)
    return (np.asarray(obj, dtype=np.float64).reshape(-1, 2),
            np.asarray(img, dtype=np.float64).reshape(-1, 2),
            ids, missing)


# ---------------------------------------------------------------- 求解
def _pnpsolve(obj, img, K):
    """物点（**米**）+ 像素 → (cam←obj 的 4×4，米制, 重投影 RMS px)。"""
    if len(obj) < 4:
        raise ValueError('角点太少（%d 个），解不了 PnP' % len(obj))
    rvec, tvec, rmse = sa.solve_plane_pnp(obj, img, K)
    return sa.rodrigues_to_matrix(rvec, tvec), float(rmse)


def solve_pad(color, K, R_bc, t_bc, meta,
              max_rms_px=MAX_RMS_PX, min_markers=MIN_MARKERS):
    """解垫系。``color`` 是 RGB（graspnet_candidates.load_frame 的约定）。

    返回 ``{'ok', 'checks', 'T_base_pad', 'T_A_B', 'adopted'}``；
    数据不足时 ``ok=False`` 且 ``T_base_pad=None``（fail-closed）。
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    gray = cv2.cvtColor(np.asarray(color), cv2.COLOR_RGB2GRAY)
    det = detect_pages(gray, meta, K)
    pairs = {}
    checks = {'max_rms_px': float(max_rms_px),
              'min_markers_per_page': int(min_markers)}
    for key in ('a', 'b'):
        obj, img, ids, missing = corner_pairs(meta, key, det[key])
        pairs[key] = {'obj': obj, 'img': img, 'ids': ids, 'missing': missing}
        checks['page_%s' % key] = {
            'dict': meta['pages'][key]['dictionary'],
            'n_markers': len(ids), 'ids': ids, 'missing_ids': missing}

    if (len(pairs['a']['ids']) < min_markers
            or len(pairs['b']['ids']) < min_markers):
        checks['reason'] = (
            '两页都要各检出 ≥%d 个标记：页 A %d 个、页 B %d 个'
            % (min_markers, len(pairs['a']['ids']), len(pairs['b']['ids'])))
        return {'ok': False, 'checks': checks, 'T_base_pad': None,
                'T_A_B': None, 'adopted': None}

    Ta, rms_a = _pnpsolve(pairs['a']['obj'] / 1000.0,
                          pairs['a']['img'], K)
    Tb, rms_b = _pnpsolve(pairs['b']['obj'] / 1000.0,
                          pairs['b']['img'], K)
    # 页 B 实测位移：名义角点 → 实际位置（垫系内，单位 **米**）。
    # PnP 的物点必须是米，才能与 base←cam 的米制平移直接相乘（见 _pnpsolve
    # 的注释：mm 物点会解出 mm 平移，和 R_bc/t_bc 拼起来就是单位混用）。
    X = _inv(Ta) @ Tb
    obj_b = _apply(X, pairs['b']['obj'] / 1000.0)
    obj = np.vstack([pairs['a']['obj'] / 1000.0, obj_b])
    img = np.vstack([pairs['a']['img'], pairs['b']['img']])
    Tj, rms_j = _pnpsolve(obj, img, K)

    # 联合解只在**不比两页单解更差**时采用（否则说明 T_A_B 估计被噪声带跑）。
    worst_single = max(rms_a, rms_b)
    if rms_j <= worst_single + 0.25 and rms_j <= max_rms_px:
        T_cam_pad, adopted, rms = Tj, 'joint', rms_j
    elif rms_a <= max_rms_px:
        T_cam_pad, adopted, rms = Ta, 'page_a', rms_a
    else:
        checks['reason'] = ('重投影 RMS 超限：页 A %.2f px、页 B %.2f px、'
                            '联合 %.2f px（上限 %.2f px）'
                            % (rms_a, rms_b, rms_j, max_rms_px))
        checks.update(rms_a_px=round(rms_a, 3), rms_b_px=round(rms_b, 3),
                      rms_joint_px=round(rms_j, 3))
        return {'ok': False, 'checks': checks, 'T_base_pad': None,
                'T_A_B': X.tolist(), 'adopted': None}

    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = np.asarray(R_bc, dtype=np.float64).reshape(3, 3)
    T_base_cam[:3, 3] = np.asarray(t_bc, dtype=np.float64).reshape(3)
    T_base_pad = T_base_cam @ T_cam_pad

    # 格心必须在画面里（用相机系投，与 base 无关）。
    T_cam_base = _inv(T_base_cam)
    T_cam_pad = T_cam_base @ T_base_pad
    cells = np.array([[c['center_pad_mm'][0] / 1000.0,
                       c['center_pad_mm'][1] / 1000.0, 0.0]
                      for c in meta['grid']['cells']], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T_cam_pad[:3, :3])
    uv = sa.project_points(cells, rvec, T_cam_pad[:3, 3], K)
    h, w = gray.shape[:2]
    in_view = bool(np.all((uv[:, 0] >= 0) & (uv[:, 0] < w)
                          & (uv[:, 1] >= 0) & (uv[:, 1] < h)))

    checks.update({
        'rms_a_px': round(rms_a, 3), 'rms_b_px': round(rms_b, 3),
        'rms_joint_px': round(rms_j, 3), 'rms_adopted_px': round(rms, 3),
        'adopted': adopted, 'cells_in_view': in_view,
        'cell_centers_px': [[round(float(u), 2), round(float(v), 2)]
                            for u, v in uv],
        'T_A_B_delta': _pose_delta(X),
    })
    ok = bool(rms <= max_rms_px and in_view)
    if not ok and 'reason' not in checks:
        checks['reason'] = ('格心出画面' if not in_view
                            else 'RMS %.2f px 超限' % rms)
    return {'ok': ok, 'checks': checks, 'T_base_pad': T_base_pad,
            'T_A_B': X.tolist(), 'adopted': adopted}


# ---------------------------------------------------------------- grid.json
def build_grid(meta, T_base_pad, checks, T_A_B=None, pitch_mm=None):
    """把 sidecar + 解出的垫位姿 → ``grid.json``（A5 判分器 / A6 主链的输入）。"""
    T = np.asarray(T_base_pad, dtype=np.float64)
    cells = []
    for c in meta['grid']['cells']:
        p = _apply(T, [[c['center_pad_mm'][0] / 1000.0,
                        c['center_pad_mm'][1] / 1000.0]])[0]
        cells.append({'idx': int(c['idx']), 'row': int(c['row']),
                      'col': int(c['col']), 'color': c['color'],
                      'center_pad_mm': [float(v) for v in c['center_pad_mm']],
                      'center_base_mm': [round(float(p[0]) * 1000.0, 2),
                                         round(float(p[1]) * 1000.0, 2)]})
    center = _apply(T, [[0.0, 0.0]])[0] * 1000.0
    return {
        'version': 1,
        'kind': 'stage2-grid',
        'pitch_mm': float(pitch_mm if pitch_mm is not None
                          else meta['pitch_mm']),
        'frame': 'pad',
        'cells': cells,
        'grid_center_base_mm': [round(float(center[0]), 2),
                                round(float(center[1]), 2)],
        'grid_yaw_base_deg': round(_yaw_of(T), 3),
        'grid_order': list(meta['grid_order']),
        'T_base_pad': [[float(v) for v in row] for row in T],
        'mat': {
            'page_a': {'dictionary': meta['pages']['a']['dictionary'],
                       'n_markers': checks['page_a']['n_markers']},
            'page_b': {'dictionary': meta['pages']['b']['dictionary'],
                       'n_markers': checks['page_b']['n_markers']},
            'T_A_B': ([] if T_A_B is None else
                      [list(map(float, row)) for row in
                       np.asarray(T_A_B, dtype=np.float64)]),
            'T_A_B_delta': checks.get('T_A_B_delta'),
        },
        'checks': checks,
    }


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        description='阶段 2：survey 帧 → 垫系 → grid.json（只读，不动臂）')
    ap.add_argument('--frame', required=True, help='帧目录（color.png/tf.json/meta.json）')
    ap.add_argument('--mat', required=True, help='stage2-mat.json（sidecar）')
    ap.add_argument('--out', default=None,
                    help='默认 <frame>/grid.json；--cam-identity 时默认 <frame>/T_cam_pad.json')
    ap.add_argument('--camera', choices=('wrist', 'topcam'), default='wrist',
                    help='用哪一路的 TF 链解（topcam 帧须由 '
                         'precision_capture.py --camera topcam 抓）')
    ap.add_argument('--cam-identity', action='store_true',
                    help='R_bc=I, t_bc=0 ⇒ 输出语义变成 cam←pad（E2 俯视那一路用）')
    ap.add_argument('--max-rms-px', type=float, default=MAX_RMS_PX)
    ap.add_argument('--min-markers', type=int, default=MIN_MARKERS)
    args = ap.parse_args(argv)

    try:
        meta = json.load(open(args.mat))
    except (OSError, ValueError) as exc:
        print('读不到垫子 sidecar %s：%s' % (args.mat, exc))
        return 2
    chain = gc.CHAIN if args.camera == 'wrist' else gc.TOPCAM_CHAIN
    tip = gc.TIP if args.camera == 'wrist' else gc.TOPCAM_TIP
    try:
        frame = gc.load_frame(args.frame, tip=tip, chain=chain)
    except (OSError, ValueError, KeyError) as exc:
        print('读不到帧 %s：%s' % (args.frame, exc))
        return 2

    R_bc, t_bc = frame['R_bc'], frame['t_bc']
    if args.cam_identity:
        R_bc, t_bc = np.eye(3), np.zeros(3)
    res = solve_pad(frame['color'], frame['K'], R_bc, t_bc,
                    meta, max_rms_px=args.max_rms_px,
                    min_markers=args.min_markers)
    c = res['checks']
    print('页 A：%d 个标记（缺 %s）  RMS %.2f px'
          % (c['page_a']['n_markers'], c['page_a']['missing_ids'] or '无',
             c.get('rms_a_px', float('nan'))))
    print('页 B：%d 个标记（缺 %s）  RMS %.2f px'
          % (c['page_b']['n_markers'], c['page_b']['missing_ids'] or '无',
             c.get('rms_b_px', float('nan'))))
    if c.get('T_A_B_delta'):
        d = c['T_A_B_delta']
        print('页 B 相对页 A：Δ=(%+.1f, %+.1f, %+.1f) mm，yaw %+.2f°，tilt %.2f°'
              % (*d['translation_mm'], d['yaw_deg'], d['tilt_deg']))
    if res['T_base_pad'] is None:
        print('判红：%s' % c.get('reason', '未知'))
        return 1

    if args.cam_identity:
        out = args.out or os.path.join(args.frame, 'T_cam_pad.json')
        T = np.asarray(res['T_base_pad'], dtype=np.float64)
        delta = _pose_delta(T)
        doc = {
            'version': 1, 'kind': 'stage2-cam-pad',
            'camera': args.camera, 'frame': 'cam', 'chain_tip': tip,
            'T_cam_pad': [[float(v) for v in row] for row in T],
            'pad_pose_in_cam': delta,
            'rms_adopted_px': c['rms_adopted_px'], 'adopted': res['adopted'],
            'T_A_B': res['T_A_B'], 'checks': c,
            'note': 'T_cam_pad 是 %s←pad，不是 base←pad；别当 grid.json 用'
                    % tip,
        }
        with open(out, 'w') as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)
        bar = '=' * 68
        print(bar)
        print('!!  --cam-identity：以下数字是  %s←pad  （相机系！）' % tip)
        print('!!  不是 base←pad，不能直接当 grid.json 用')
        print(bar)
        print('垫心在该相机系：(%+.1f, %+.1f, %+.1f) mm，yaw %+.2f°，tilt %.2f°'
              % (*delta['translation_mm'], delta['yaw_deg'], delta['tilt_deg']))
        print('采用 %s 解，RMS %.2f px' % (res['adopted'], c['rms_adopted_px']))
        print('写入 %s' % out)
        if not res['ok']:
            print('判红：%s' % c.get('reason', '未知'))
            return 1
        return 0

    out = args.out or os.path.join(args.frame, 'grid.json')
    grid = build_grid(meta, res['T_base_pad'], c, T_A_B=res['T_A_B'])
    with open(out, 'w') as f:
        json.dump(grid, f, ensure_ascii=False, indent=1)
    print('采用 %s 解，RMS %.2f px；垫心 base (%.1f, %.1f) mm，yaw %.2f°'
          % (res['adopted'], c['rms_adopted_px'],
             grid['grid_center_base_mm'][0], grid['grid_center_base_mm'][1],
             grid['grid_yaw_base_deg']))
    print('写入 %s' % out)
    if not res['ok']:
        print('判红：%s' % c.get('reason', '未知'))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
