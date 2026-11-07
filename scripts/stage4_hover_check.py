#!/usr/bin/env python3
"""阶段 4 · E4a：悬停位可见性门（纯离线量，不动臂）。

在每个悬停高度各拍一组连续帧（``precision_capture.py --camera wrist --burst N``），
逐帧走 ``stage3_measure.measure_frame``，判三件事：

    · 可用率 ≥60%          —— 这个高度上看得到块顶的比例
    · 中心半极差 ≤2 mm     —— 可用帧中心 ``(max−min)/2``（判据行里的 center_span/2）
    · yaw 极差 ≤2°         —— 彩色掩膜那一路的 mod-90 极差

几何铁律：悬停点是 ``tool0_z = 块顶 z + TOOL0_TO_BARE_FINGERTIP(0.2042) + hover``。
**不能用** ``0.1757`` —— 那是「块高折扣后的抓取点」，拿它算悬停会把臂压低近 30 mm。
给 ``--block-top-mm`` 时，本工具会把每个高度该发的 ``tool0_z`` 一并记进报告，
供动作生成核对（本工具自己不产生任何运动）。

帧目录两种形态都认：``--burst 1`` 的单帧（``color.png``）与 ``--burst N`` 的
批量帧（``color_000.png`` …）。后者不要求先拆目录。

**选簇按同帧相对高度**（``select_h_mm`` = 块高，默认 29.5，可用
``--block-h-mm`` 改）：悬停时夹爪手指就在块顶正上方 hover 毫米处，相对台面的
高度 ≈ 块高 + hover；若沿用「取窗口内最高簇」，量到的会是手指 —— 手指相对相机
静止，中心/yaw 极差恒为 0，会把 E4a 判成败假绿。相对高度在同一帧里测得，
腕部深度沿光轴的偏差在块与台面两边同量抵消。

退出码：0 = 三个高度全过；1 = 有高度不过；2 = 数据缺失/坏。
"""

import argparse
import datetime
import glob
import json
import os
import re
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage2_geom as s2g                                         # noqa: E402
import stage3_measure as sm                                       # noqa: E402

#: tool0 → **裸指尖**（2026-10-26 定案）。悬停高度加在它上面。
TOOL0_TO_BARE_FINGERTIP_M = 0.2042
HOVER_MM = (50.0, 65.0, 80.0)
MIN_USABLE_RATIO = 0.60
MAX_CENTER_HALF_SPAN_MM = 2.0
MAX_YAW_SPAN_DEG = 2.0


# ---------------------------------------------------------------- 帧 I/O
_INDEX_RE = re.compile(r'color_(\d+)\.png$')


def list_indices(capture_dir):
    """帧目录 → 帧序号列表；单帧形态返回 ``[None]``，空目录返回 ``[]``。"""
    hits = sorted(glob.glob(os.path.join(capture_dir, 'color_*.png')))
    if hits:
        return sorted(int(_INDEX_RE.search(h).group(1)) for h in hits)
    if os.path.exists(os.path.join(capture_dir, 'color.png')):
        return [None]
    return []


def load_indexed_frame(capture_dir, idx):
    """按序号读一帧（与 ``gc.load_frame`` 同样的内容，但文件名带 ``_NNN``）。"""
    def p(name, ext):
        base = name if idx is None else '%s_%03d' % (name, idx)
        return os.path.join(capture_dir, base + ext)

    depth = np.array(Image.open(p('depth', '.png')), dtype=np.float64) / 1000.0
    color = np.array(Image.open(p('color', '.png')).convert('RGB'))
    with open(p('tf', '.json'), encoding='utf-8') as fh:
        tf_ok = json.load(fh)['ok']
    R, t = gc.cam_pose_tf(tf_ok)
    with open(p('meta', '.json'), encoding='utf-8') as fh:
        K = np.array(json.load(fh)['camera_info']['K'],
                     dtype=np.float64).reshape(3, 3)
    js = p('joint_states', '.json')
    joints = json.load(open(js, encoding='utf-8')) if os.path.exists(js) else None
    return {'dir': capture_dir, 'depth': depth, 'color': color, 'K': K,
            'R_bc': R, 't_bc': t, 'joints': joints}


# ---------------------------------------------------------------- 统计
def _median(vals):
    return float(np.median(vals)) if vals else None


def _span(vals):
    return float(max(vals) - min(vals)) if vals else None


def summarize(samples, block_xy_mm=None, block_top_mm=None, hover_mm=None):
    """一组帧的测量 → 判据块。``samples`` 是 measure_frame 的输出。"""
    n = len(samples)
    usable = [s for s in samples if s.get('usable') is True]
    cs = [s['center_base_mm'] for s in usable if s.get('center_base_mm')]
    xs = [float(c[0]) for c in cs]
    ys = [float(c[1]) for c in cs]
    yaws = [s.get('yaw_color_deg') for s in usable
            if s.get('yaw_color_trusted') is True and s.get('yaw_color_deg') is not None]
    covers = [float(s['face_cover']) for s in usable
              if s.get('face_cover') is not None]
    tops = [float(s['top_mm']) for s in usable
            if s.get('top_mm') is not None]

    span_x, span_y = _span(xs), _span(ys)
    half = (max(span_x, span_y) / 2.0
            if span_x is not None and span_y is not None else None)
    yaw_span = s2g.ang_span_mod90(yaws) if yaws else None
    cx, cy = _median(xs), _median(ys)

    out = {
        'n_frames': n, 'n_usable': len(usable),
        'usable_ratio': round(len(usable) / n, 3) if n else 0.0,
        'center_median_mm': (None if cx is None
                             else [round(cx, 2), round(cy, 2)]),
        'center_span_mm': (None if span_x is None
                           else [round(span_x, 3), round(span_y, 3)]),
        'center_half_span_mm': None if half is None else round(half, 3),
        'yaw_color_span_deg': None if yaw_span is None else round(yaw_span, 3),
        'yaw_color_n_trusted': len(yaws),
        'face_cover': ({'median': round(_median(covers), 3),
                        'min': round(min(covers), 3),
                        'max': round(max(covers), 3)} if covers else None),
        'reasons': [],
    }
    if block_xy_mm is not None and cx is not None:
        out['offset_from_block_mm'] = [round(cx - float(block_xy_mm[0]), 2),
                                       round(cy - float(block_xy_mm[1]), 2)]
    if block_top_mm is not None and hover_mm is not None:
        out['tool0_z_mm'] = round(float(block_top_mm)
                                  + TOOL0_TO_BARE_FINGERTIP_M * 1000.0
                                  + float(hover_mm), 2)
    # 命令高度是按「俯视测的块顶」算的；俯视深度历来读远 ~1.9%，真实余量未必
    # 等于 hover。腕部这一路自己量的块顶（同帧相对高度选簇）给出**实际余量**，
    # 只记录、不改判据 —— 报告里必须能看出「说 50 mm 实际是多少」。
    if tops:
        out['measured_top_mm'] = round(_median(tops), 2)
        if out.get('tool0_z_mm') is not None:
            out['actual_hover_mm'] = round(
                out['tool0_z_mm'] - TOOL0_TO_BARE_FINGERTIP_M * 1000.0
                - out['measured_top_mm'], 2)

    if n == 0:
        out['reasons'].append('没有帧')
    elif out['usable_ratio'] < MIN_USABLE_RATIO:
        out['reasons'].append('可用率 %.0f%% < %.0f%%'
                              % (out['usable_ratio'] * 100, MIN_USABLE_RATIO * 100))
    if half is None:
        out['reasons'].append('没有可用帧的中心')
    elif half > MAX_CENTER_HALF_SPAN_MM:
        out['reasons'].append('中心半极差 %.2f mm > %.2f mm'
                              % (half, MAX_CENTER_HALF_SPAN_MM))
    if yaw_span is None:
        out['reasons'].append('没有可信的彩色 yaw（判不了，不能算通过）')
    elif yaw_span > MAX_YAW_SPAN_DEG:
        out['reasons'].append('yaw 极差 %.2f° > %.2f°'
                              % (yaw_span, MAX_YAW_SPAN_DEG))
    out['ok'] = not out['reasons']
    return out


def _frame_record(idx, res, err=None):
    rec = {'index': idx,
           'usable': bool(res.get('usable')) if err is None else False,
           'center_base_mm': res.get('center_base_mm'),
           'yaw_color_deg': res.get('yaw_color_deg'),
           'yaw_color_trusted': res.get('yaw_color_trusted'),
           'face_cover': res.get('face_cover'),
           'found': res.get('found')}
    if err:
        rec['error'] = err
    return rec


# ---------------------------------------------------------------- 主流程
def check_height(capture_dir, hover_mm, block_xy_mm, block_top_mm=None,
                 z_bias_mm=0.0, block_h_mm=None):
    if not os.path.isdir(capture_dir):
        raise ValueError('帧目录不存在：%s' % capture_dir)
    idxs = list_indices(capture_dir)
    # 悬停时夹爪手指在块顶正上方 hover 毫米处，相对台面的高度 ≈ 块高 + hover。
    # 不按这个相对高度选簇，pick_tower_top 的「取最高簇」会量到手指本身 ——
    # 手指相对相机静止，中心/yaw 极差都会是 0，E4a 会拿夹爪自己判成满分。
    select_h = (float(block_h_mm) if block_h_mm is not None
                else float(sm.BLOCK_TRUE_H_MM))
    samples, records = [], []
    for idx in idxs:
        try:
            frame = load_indexed_frame(capture_dir, idx)
            kw = {'window_xy_mm': tuple(block_xy_mm), 'z_bias_mm': z_bias_mm,
                  'frame': frame, 'select_h_mm': select_h}
            if block_h_mm is not None:
                kw['block_h_mm'] = block_h_mm
            res = sm.measure_frame(capture_dir, **kw)
        except Exception as exc:                                   # noqa: BLE001
            res, err = {}, '%s: %s' % (type(exc).__name__, exc)
            records.append(_frame_record(idx, res, err))
            samples.append({'usable': False})
            continue
        samples.append(res)
        records.append(_frame_record(idx, res))
    summary = summarize(samples, block_xy_mm=block_xy_mm,
                        block_top_mm=block_top_mm, hover_mm=hover_mm)
    summary.update({'hover_mm': float(hover_mm), 'capture': capture_dir,
                    'select_h_mm': select_h, 'frames': records})
    return summary


def parse_capture(spec):
    if ':' not in spec:
        raise ValueError('--capture 要写成 高度:目录，例如 50:runs/x/hover-50')
    h, d = spec.split(':', 1)
    return float(h), os.path.abspath(d)


def run(captures, hover_mm=HOVER_MM, block_xy_mm=None, block_top_mm=None,
        z_bias_mm=0.0, block_h_mm=None):
    wants = sorted(float(h) for h in hover_mm)
    got = sorted(float(h) for h, _ in captures)
    heights = []
    reasons = []
    for h in wants:
        dirs = [d for hh, d in captures if abs(hh - h) < 1e-9]
        if len(dirs) != 1:
            reasons.append('悬停高度 %.0f mm 的 --capture 给了 %d 个（要正好 1 个）'
                           % (h, len(dirs)))
            continue
        heights.append(check_height(dirs[0], h, block_xy_mm, block_top_mm,
                                    z_bias_mm=z_bias_mm,
                                    block_h_mm=block_h_mm))
    extra = [h for h in got if h not in wants]
    if extra:
        reasons.append('多给了不判的高度 %s（判据只要 %s）'
                       % (extra, wants))
    ok = (not reasons) and len(heights) == len(wants) and all(
        x['ok'] for x in heights)
    return {
        'version': 1, 'kind': 'stage4-hover-check',
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
        'block_xy_mm': None if block_xy_mm is None else list(block_xy_mm),
        'block_top_mm': block_top_mm,
        'tool0_to_bare_fingertip_mm': TOOL0_TO_BARE_FINGERTIP_M * 1000.0,
        'thresholds': {'usable_ratio_min': MIN_USABLE_RATIO,
                       'center_half_span_mm_max': MAX_CENTER_HALF_SPAN_MM,
                       'yaw_span_deg_max': MAX_YAW_SPAN_DEG,
                       'hover_mm': [float(h) for h in wants]},
        'heights': heights, 'reasons': reasons, 'ok': bool(ok),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='阶段 4 E4a：悬停位可见性门（只读，不动臂）')
    ap.add_argument('--capture', action='append', required=True,
                    metavar='H:DIR', help='悬停高度(mm):帧目录；三个高度各一个')
    ap.add_argument('--block-xy-mm', default=None, metavar='X,Y',
                    help='块中心（base 系 mm），同时作为测量窗口中心')
    ap.add_argument('--block-top-mm', type=float, default=None,
                    help='块顶 z（base 系 mm）；给了才记 tool0_z')
    ap.add_argument('--hover', default=','.join('%g' % h for h in HOVER_MM),
                    help='要判的悬停高度（默认 50,65,80）')
    ap.add_argument('--z-bias-mm', type=float, default=0.0)
    ap.add_argument('--block-h-mm', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)

    try:
        captures = [parse_capture(s) for s in args.capture]
        hover = [float(x) for x in args.hover.split(',') if x.strip()]
        block_xy = None
        if args.block_xy_mm:
            parts = [float(v) for v in args.block_xy_mm.split(',')]
            if len(parts) != 2:
                raise ValueError('--block-xy-mm 要 x,y 两个数')
            block_xy = parts
        res = run(captures, hover_mm=hover, block_xy_mm=block_xy,
                  block_top_mm=args.block_top_mm, z_bias_mm=args.z_bias_mm,
                  block_h_mm=args.block_h_mm)
    except (ValueError, OSError) as exc:
        print('数据缺失/坏：%s' % exc)
        return 2

    for h in res['heights']:
        print('悬停 %2.0f mm：可用 %d/%d（%.0f%%）  中心半极差 %s mm  '
              'yaw 极差 %s°  → %s'
              % (h['hover_mm'], h['n_usable'], h['n_frames'],
                 h['usable_ratio'] * 100, h['center_half_span_mm'],
                 h['yaw_color_span_deg'], 'OK' if h['ok'] else 'FAIL'))
        for r in h['reasons']:
            print('    !! %s' % r)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, ensure_ascii=False, indent=1)
        print('写入 %s' % args.out)
    if res['reasons']:
        for r in res['reasons']:
            print('数据缺失/坏：%s' % r)
        return 2
    if not res['ok']:
        print('判红：三个高度没全过（见上）')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
