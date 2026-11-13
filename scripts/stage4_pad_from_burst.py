#!/usr/bin/env python3
"""跑前垫位姿：从**腕部 burst** 逐帧解 base←pad，取共识，写本场的 ``grid-pre.json``。

为什么是腕部、为什么是 burst（2026-11-09 用户拍板「每场开跑实测」，台账第 295 轮）：

* 垫会被人挪。模型常量 ``gen_scene.PAD_CENTER_MM / PAD_YAW_DEG`` 与印刷垫实测差
  **87.5 mm / 88.7°**，而它同时喂计划落点与判分格心 ⇒ 放错判错同源、互相自洽。
* 俯视在 1.45 m 处量 ChArUco 的 RMS 已到 0.9 px 量级、且外参本身会无声过期；
  腕部机位离垫 ~0.5 m，同一张垫 20 帧全绿、xy 极差 0.35 mm / yaw 极差 0.133°
  （2026-11-09 s8 实测）。计划与判分读的必须是**这一份**。
* 逐帧解、取中位：一帧的标记遮挡/反光不该污染整场；离散度本身是自检
  （超 ``--max-spread-mm`` / ``--max-spread-deg`` 就停下，不拿「差不多」的垫位姿
  去放块）。

跑法（开发机，离线；burst 是板端 ``precision_capture.py --camera wrist`` 拉回来的）::

    python3 scripts/stage4_pad_from_burst.py <burst 目录> --out <run>/grid-pre.json

输出 ``kind: stage2-grid``，字段与 ``stage2_pad.py`` 的产出一致（``grid_center_base_mm``
/ ``grid_yaw_base_deg`` / ``T_base_pad``），外加共识与离散度记账。计划端
``gen_scene --grid`` 与判分端 ``stage4_clear_run --grid`` 都读这一份。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import graspnet_candidates as gc                                 # noqa: E402
import stage2_pad as sp                                          # noqa: E402

DEFAULT_MAT = os.path.expanduser(
    '~/robotics/notes/ur5-vision-grasp/stage2-mat/stage2-mat.json')


def wrap_deg(a):
    """折到 [−180, 180)。"""
    return (float(a) + 180.0) % 360.0 - 180.0


def pose_xy_yaw(T):
    """``T_base_pad`` → (平移 mm, 绕 z 的 yaw 度)。"""
    T = np.asarray(T, dtype=np.float64)
    return T[:3, 3] * 1000.0, float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))


def consensus(rows):
    """多帧位姿 → 共识。``rows`` = ``[{'t': mm, 'yaw': deg, 'T': ...}, ...]``。

    平移取**逐轴中位**；yaw 取**圆中位**（先按单位向量平均定参考角，再把各帧
    展开到参考角附近取中位 —— 直接对 −179°/179° 取中位会得到 0°）。

    返回 ``(t_med, yaw_med, spread, k)``，``spread = {'xy': {...}, 'yaw': {...}}``，
    ``k`` 是离共识最近的那一帧下标（输出完整 ``T_base_pad`` 用它）。
    """
    if not rows:
        raise ValueError('没有可用帧')
    ts = np.array([r['t'] for r in rows], dtype=np.float64)
    yaws = np.array([float(r['yaw']) for r in rows], dtype=np.float64)
    ref = float(np.degrees(np.arctan2(
        np.sin(np.radians(yaws)).mean(), np.cos(np.radians(yaws)).mean())))
    unwrapped = ref + np.array([wrap_deg(y - ref) for y in yaws])

    t_med = np.median(ts, axis=0)
    yaw_med = float(np.median(unwrapped))
    dxy = np.hypot(ts[:, 0] - t_med[0], ts[:, 1] - t_med[1])
    dyaw = np.abs(np.array([wrap_deg(y - yaw_med) for y in yaws]))
    spread = {'xy': {'max': float(dxy.max()), 'median': float(np.median(dxy))},
              'yaw': {'max': float(dyaw.max()), 'median': float(np.median(dyaw))}}
    k = int(np.argmin(dxy + dyaw * 10.0))     # 1° 折 10 mm 一起比，量纲对齐
    return t_med, yaw_med, spread, k


def solve_burst(burst, camera='wrist', mat=DEFAULT_MAT, max_rms_px=None,
                min_markers=None):
    """逐帧解垫位姿。返回 ``(rows, failures, n_total)``。

    每帧都把守由 ``sp.solve_pad`` 自己做（两页各 ≥``min_markers``、RMS
    ≤``max_rms_px``）；这里只决定「这一帧算不算数」，不改判据。
    """
    chain, tip = ((gc.CHAIN, gc.TIP) if camera == 'wrist'
                  else (gc.TOPCAM_CHAIN, gc.TOPCAM_TIP))
    with open(mat, encoding='utf-8') as f:
        meta = json.load(f)
    names = sorted(f for f in os.listdir(burst) if f.startswith('color_'))
    rows, failures = [], []
    for idx in range(len(names)):
        try:
            fr = gc.load_frame(burst, tip=tip, chain=chain, idx=idx)
            kw = {}
            if max_rms_px is not None:
                kw['max_rms_px'] = max_rms_px
            if min_markers is not None:
                kw['min_markers'] = min_markers
            res = sp.solve_pad(fr['color'], fr['K'], fr['R_bc'], fr['t_bc'],
                               meta, **kw)
        except Exception as exc:                        # noqa: BLE001
            failures.append({'idx': idx, 'reason': str(exc)[:120]})
            continue
        if res.get('T_base_pad') is None:
            failures.append({'idx': idx,
                             'reason': str(res.get('checks', {}).get('reason',
                                                                     ''))[:120]})
            continue
        t, yaw = pose_xy_yaw(res['T_base_pad'])
        rows.append({'idx': idx, 't': t, 'yaw': yaw,
                     'rms_px': res['checks'].get('rms_adopted_px'),
                     'T': np.asarray(res['T_base_pad'], dtype=np.float64)})
    return rows, failures, len(names)


def build_doc(rows, t_med, yaw_med, spread, k, camera, burst, mat, n_total):
    """共识 → ``stage2-grid`` 文档（计划端与判分端共用）。"""
    rms = [r['rms_px'] for r in rows if r.get('rms_px') is not None]
    return {
        'version': 1,
        'kind': 'stage2-grid',
        'camera': camera,
        'grid_center_base_mm': [float(v) for v in t_med],
        'grid_yaw_base_deg': float(yaw_med),
        'T_base_pad': [[float(v) for v in row] for row in rows[k]['T']],
        'adopted_frame': 'color_%03d.png' % rows[k]['idx'],
        'n_frames_ok': len(rows),
        'n_frames_total': int(n_total),
        'spread_xy_mm': spread['xy'],
        'spread_yaw_deg': spread['yaw'],
        'source_burst': burst,
        'source_mat': mat,
        'checks': {                       # 判分日志读这两个键（stage4_clear_run）
            'n_frames': len(rows),
            'n_frames_total': int(n_total),
            'rms_adopted_px': (float(np.median(rms)) if rms else None),
        },
    }


def embed_pad_spec(doc, spec_doc):
    """把 pad-spec（足迹/厚度真值）嵌进 grid 文档，使其**自洽**。

    ``pad_thickness_m``：spec 的 thickness_m > 解垫垫面 z（``T_base_pad[2,3]``，
    台面 = base 0）。``pad_footprint_m``：只认 spec（模型默认 0.30×0.30 与实垫
    差 90 mm，状态层 §4 早有记录）。spec 缺席时厚度仍嵌入（垫面 z 是实测），
    足迹不嵌 —— 下游用到时按 C1 哨兵 fail-closed。

    纯函数，可离线自测。
    """
    doc = dict(doc)
    try:
        doc['pad_thickness_m'] = float(doc['T_base_pad'][2][3])
    except (KeyError, TypeError, IndexError, ValueError):
        pass
    if spec_doc:
        if spec_doc.get('thickness_m') is not None:
            doc['pad_thickness_m'] = float(spec_doc['thickness_m'])
        if spec_doc.get('footprint_m'):
            doc['pad_footprint_m'] = [float(v) for v in spec_doc['footprint_m']]
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='从腕部 burst 解本场垫位姿 → grid-pre.json')
    ap.add_argument('burst', help='burst 目录（flat color_NNN.png / depth_NNN.png）')
    ap.add_argument('--out', required=True, help='写出的 grid JSON')
    ap.add_argument('--camera', choices=('wrist', 'topcam'), default='wrist')
    ap.add_argument('--mat', default=DEFAULT_MAT,
                    help='stage2-mat.json（默认取仓外 notes 里的那份）')
    ap.add_argument('--min-frames', type=int, default=5,
                    help='至少多少帧解出（默认 5；一帧就下结论不算共识）')
    ap.add_argument('--max-spread-mm', type=float, default=2.0,
                    help='共识平移的最大离散（默认 2 mm）')
    ap.add_argument('--max-spread-deg', type=float, default=0.5,
                    help='共识 yaw 的最大离散（默认 0.5°）')
    ap.add_argument('--max-rms-px', type=float, default=None,
                    help='透传给 stage2_pad（缺省用它的默认门）')
    ap.add_argument('--min-markers', type=int, default=None,
                    help='透传给 stage2_pad（缺省用它的默认门）')
    ap.add_argument('--pad-spec', default=None,
                    help='仓外 pad-spec.json（足迹/厚度真值）。给了就把 '
                         'pad_footprint_m / pad_thickness_m 写进 grid 文档 '
                         '—— **grid 文档自此自洽**：探针/判分在新进程里 '
                         'load_grid 一份文件就拿到全部实测垫参数（C1 后置）')
    args = ap.parse_args(argv)

    if not os.path.isdir(args.burst):
        print('!! burst 目录不存在：%s' % args.burst)
        return 2
    rows, failures, n_total = solve_burst(
        args.burst, camera=args.camera, mat=args.mat,
        max_rms_px=args.max_rms_px, min_markers=args.min_markers)
    print('解出 %d/%d 帧' % (len(rows), n_total))
    for f in failures:
        print('  ✗ idx=%03d %s' % (f['idx'], f['reason']))
    if len(rows) < args.min_frames:
        print('!! 只有 %d 帧解出（< --min-frames %d）⇒ 不写文件'
              % (len(rows), args.min_frames))
        return 1

    t_med, yaw_med, spread, k = consensus(rows)
    print('共识 xy=(%+.1f, %+.1f) mm  z=%+.1f mm  yaw=%+.3f°'
          % (t_med[0], t_med[1], t_med[2], yaw_med))
    print('离散：xy max %.3f mm（中位 %.3f）  yaw max %.3f°（中位 %.3f）'
          % (spread['xy']['max'], spread['xy']['median'],
             spread['yaw']['max'], spread['yaw']['median']))
    bad = []
    if spread['xy']['max'] > args.max_spread_mm:
        bad.append('xy 极差 %.3f mm > %.3f' % (spread['xy']['max'],
                                               args.max_spread_mm))
    if spread['yaw']['max'] > args.max_spread_deg:
        bad.append('yaw 极差 %.3f° > %.3f' % (spread['yaw']['max'],
                                              args.max_spread_deg))
    if bad:
        print('!! 离散超限：%s ⇒ 不写文件（垫位姿没有共识）' % '；'.join(bad))
        return 1

    doc = build_doc(rows, t_med, yaw_med, spread, k, args.camera, args.burst,
                    args.mat, n_total)
    spec_doc = None
    if args.pad_spec:
        if not os.path.exists(args.pad_spec):
            print('!! --pad-spec 不存在：%s' % args.pad_spec)
            return 2
        with open(args.pad_spec, encoding='utf-8') as f:
            spec_doc = json.load(f)
    doc = embed_pad_spec(doc, spec_doc)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    print('写出 %s（采用 %s）' % (args.out, doc['adopted_frame']))
    if spec_doc is None:
        print('  ⚠ 未给 --pad-spec：grid 文档缺 pad_footprint_m —— 下游'
              '（探针/判分）用到足迹时会 fail-closed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
