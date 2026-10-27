#!/usr/bin/env python3
"""入口判据 ②：几何 yaw 重复性（同一块多帧、mod 90° 极差 ≤6°）。

数据
────
A3 存档帧 `runs/2026-10-25-precision/frames/{pick-02,pick-05,pick-06}`：
同一相机位姿、同一批块（时间 12:56 / 13:35 / 13:57，块逐个被拿走）。
这三帧不是「同一块被重新摆三次」，而是**同一次摆放的三个时刻** —— 对
「估计器抖不抖」正合适；对「块换了位置还准不准」不构成证据（那要 B 批现场）。

三种 yaw 同时报
───────────────
· ``yaw_pca``  —— `find_blocks` 的 PCA（历史字段，近正方块不稳）；
· ``yaw_rect`` —— `find_blocks` 的纯 numpy 分位矩形（graspnet venv 回退）；
· ``yaw_pix``  —— `stage2_geom` 的像素掩码 minAreaRect（**主链采用**）。

判据
────
每个「同色 + 位置 ≤10 mm」组，`yaw_pix` fold mod 90 的极差 ≤ ``--threshold-deg``
（默认 6°）。组数 < ``--min-groups``（默认 3）→ 退出码 2（样本不足，不假装通过）。
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                               # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import stage2_geom as sg                                          # noqa: E402


def _high_blocks(frame_dir):
    frame = gc.load_frame(frame_dir)
    blocks, table_z = gc.find_blocks(frame)
    out = []
    for b in blocks:
        if b['low']:
            continue
        r = sg.block_rect_yaw(frame, b)
        out.append({'block': b, 'pix': r})
    return out, frame, table_z


def _match(frame_a, frame_b, max_mm):
    """a 里每块在 b 里找同色、位置最近且 ≤max_mm 的块；一对一。"""
    pairs = []
    for i, a in enumerate(frame_a):
        for j, b in enumerate(frame_b):
            if a['block']['class'] != b['block']['class']:
                continue
            d = float(np.hypot(a['block']['cx'] - b['block']['cx'],
                               a['block']['cy'] - b['block']['cy']))
            if d <= max_mm:
                pairs.append((d, i, j))
    pairs.sort()
    used_a, used_b, out = set(), set(), {}
    for d, i, j in pairs:
        if i in used_a or j in used_b:
            continue
        used_a.add(i)
        used_b.add(j)
        out[i] = (j, d)
    return out


def group_blocks(per_frame, max_mm=10.0):
    """以第一帧为锚，把后续帧里同色同位置的块串成组。

    只按第一帧的块建组；后续帧新出现的块（第一帧没有）不参与 —— 重复性要
    的是「同一块 ≥N 帧」，锚定第一帧最直接，也不会把两块拼成一组。
    """
    if not per_frame:
        return []
    anchors = per_frame[0]
    groups = []
    for i, a in enumerate(anchors):
        g = [(0, i, a)]
        for fi in range(1, len(per_frame)):
            m = _match(anchors, per_frame[fi], max_mm)
            if i in m:
                j, d = m[i]
                g.append((fi, j, per_frame[fi][j]))
        groups.append(g)
    return groups


def _vals(group, key):
    return [x[2][key] for x in group]


def summarize(groups, threshold_deg):
    rows = []
    for g in groups:
        pix = _vals(g, 'pix')
        yaw_pix = [round(x['yaw_deg'], 2) if x['yaw_deg'] is not None else None
                   for x in pix]
        yaw_pca = [round(x[2]['block']['yaw'], 2) for x in g]
        yaw_rect = [round(x[2]['block']['yaw_rect'], 2) for x in g]
        rows.append({
            'class': g[0][2]['block']['class'],
            'xy_mm': [round(g[0][2]['block']['cx'], 1),
                      round(g[0][2]['block']['cy'], 1)],
            'frames': [x[0] for x in g],
            'n': len(g),
            'yaw_pca': yaw_pca,
            'yaw_rect': yaw_rect,
            'yaw_pix': yaw_pix,
            'span_pca': sg.ang_span_mod90(yaw_pca),
            'span_rect': sg.ang_span_mod90(yaw_rect),
            'span_pix': sg.ang_span_mod90(yaw_pix),
            'trusted': [bool(x['trusted']) for x in pix],
            'ext_mm': [x['ext_mm'] for x in pix],
            'pass': (sg.ang_span_mod90(yaw_pix) is not None
                     and sg.ang_span_mod90(yaw_pix) <= threshold_deg),
        })
    rows.sort(key=lambda r: (r['class'], r['xy_mm']))
    return rows


def _md(rows, threshold_deg, min_frames=3):
    lines = ['# A3 · 几何 yaw 重复性（mod 90°）', '',
             '判据：同一块 ≥%d 帧，`yaw_pix` mod 90° 极差 ≤ %.1f°。' % (
                 min_frames, threshold_deg), '',
             '| 类 | 位置 (mm) | 帧数 | span PCA | span rect | **span pix** | 通过 |',
             '|---|---|---|---|---|---|---|']
    for r in rows:
        if r['n'] < min_frames:
            continue
        lines.append('| %s | (%.1f, %.1f) | %d | %s | %s | **%.2f** | %s |' % (
            r['class'], r['xy_mm'][0], r['xy_mm'][1], r['n'],
            '%.2f' % r['span_pca'] if r['span_pca'] is not None else '-',
            '%.2f' % r['span_rect'] if r['span_rect'] is not None else '-',
            r['span_pix'], '✅' if r['pass'] else '❌'))
    lines += ['', '## 逐帧 yaw_pix', '',
              '| 类 | 位置 | 各帧 yaw_pix | trusted | ext_mm |',
              '|---|---|---|---|---|']
    for r in rows:
        if r['n'] < min_frames:
            continue
        lines.append('| %s | (%.1f, %.1f) | %s | %s | %s |' % (
            r['class'], r['xy_mm'][0], r['xy_mm'][1],
            [round(v, 2) if v is not None else None for v in r['yaw_pix']],
            r['trusted'], r['ext_mm']))
    lines += ['', '## 判读', '',
              '- `yaw_pix` 是本判据采用的值；`yaw_rect` 只在没有 cv2 的 graspnet venv 里作回退。',
              '- PCA 在近正方块上会翻 45°/90°，不能用于宫格对齐。',
              '- 本数据是「同一次摆放的三个时刻」；块被重新摆放后的准确性要 B 批现场再验。']
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', nargs='+', required=True,
                    help='按时间顺序列出帧目录（第一帧作锚）')
    ap.add_argument('--match-mm', type=float, default=10.0)
    ap.add_argument('--threshold-deg', type=float, default=6.0)
    ap.add_argument('--min-frames', type=int, default=3)
    ap.add_argument('--min-groups', type=int, default=3)
    ap.add_argument('--out-json', default=None)
    ap.add_argument('--out-md', default=None)
    args = ap.parse_args()

    per_frame = []
    for fd in args.frames:
        if not os.path.isdir(fd):
            print('缺帧目录：%s' % fd, file=sys.stderr)
            return 2
        blocks, _, _ = _high_blocks(fd)
        per_frame.append(blocks)
        print('%s：高块 %d' % (os.path.basename(fd.rstrip('/')), len(blocks)))

    groups = group_blocks(per_frame, max_mm=args.match_mm)
    rows = summarize(groups, args.threshold_deg)
    enough = [r for r in rows if r['n'] >= args.min_frames]

    out = {'frames': [os.path.abspath(p) for p in args.frames],
           'threshold_deg': args.threshold_deg,
           'match_mm': args.match_mm, 'min_frames': args.min_frames,
           'min_groups': args.min_groups, 'rows': rows,
           'n_groups': len(enough),
           'n_pass': sum(1 for r in enough if r['pass'])}
    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('JSON：%s' % args.out_json)
    if args.out_md:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_md)), exist_ok=True)
        with open(args.out_md, 'w', encoding='utf-8') as f:
            f.write(_md(rows, args.threshold_deg, args.min_frames))
        print('MD：%s' % args.out_md)

    print('\n组（≥%d 帧）：%d，通过 %d' % (
        args.min_frames, len(enough), out['n_pass']))
    for r in enough:
        print('  %-6s (%5.1f,%5.1f) n=%d span_pix=%.2f span_rect=%s pass=%s' % (
            r['class'], r['xy_mm'][0], r['xy_mm'][1], r['n'], r['span_pix'],
            '%.2f' % r['span_rect'] if r['span_rect'] is not None else '-',
            r['pass']))

    if len(enough) < args.min_groups:
        print('样本不足：只有 %d 组 ≥%d 帧（要求 ≥%d）' % (
            len(enough), args.min_frames, args.min_groups), file=sys.stderr)
        return 2
    return 0 if out['n_pass'] == len(enough) else 1


if __name__ == '__main__':
    sys.exit(main())
