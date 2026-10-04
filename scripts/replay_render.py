#!/usr/bin/env python3
"""把 `replay_record.py` 录下的一次抓取渲染成**可逐帧看的 mp4**。

画面上有什么（这一条是它存在的理由）
────────────────────────────────────
    左：彩色，叠 **哪些是检测到的方块**、**区划在哪**、以及
        **由当时的关节角算出的 tool0 实际位置**（`urdf_fk`，不是规划器报的目标）
    右：**对齐深度**（伪彩），同一套叠加

「规划器以为自己去了哪」和「机械臂实际在哪」画在同一帧上，
于是「差多少、差在哪一步」是**可以直接读出来的**，不靠感觉。
深度那一栏用来分开两类原因：几何算错了（深度正常、位置偏），
还是深度本身不可信（深度那一块发虚或被台面填平）。

用法
────
    python3 scripts/replay_render.py /tmp/run1 --out /tmp/run1.mp4
    python3 scripts/replay_render.py /tmp/run1 --out /tmp/run1.mp4 --zone 0.40,0.60,0.00,0.45
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

PANEL_W = 960          # 每一栏的宽度（彩色与深度各一栏）
TRAIL = 40             # 轨迹尾巴保留多少帧


def load_recording(d):
    """读回录制目录 → ``(info, meta 列表)``。"""
    info = json.load(open(os.path.join(d, 'info.json'), encoding='utf-8'))
    meta = [json.loads(ln) for ln in
            open(os.path.join(d, 'meta.jsonl'), encoding='utf-8') if ln.strip()]
    return info, meta


def depth_to_bgr(depth):
    """16 位毫米深度 → 伪彩 BGR（逐帧归一化，只为看形状，不是量测）。"""
    import cv2
    d = depth.astype(np.float32)
    valid = d > 0
    if not valid.any():
        return np.zeros((*depth.shape, 3), np.uint8), 0.0, 0.0
    lo, hi = np.percentile(d[valid], [2, 98])
    if hi - lo < 1e-6:
        hi = lo + 1.0
    norm = np.clip((d - lo) / (hi - lo), 0, 1)
    norm[~valid] = 1.0                      # 无效像素涂白，一眼能看出空洞
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    return img, float(lo), float(hi)


def main():
    ap = argparse.ArgumentParser(description='把录制的抓取渲染成 mp4')
    ap.add_argument('rec', help='replay_record.py 的输出目录')
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=float, default=4.0, help='播放帧率')
    ap.add_argument('--zone', default=None,
                    help='要把哪个区画出来：xmin,xmax,ymin,ymax[,z]')
    ap.add_argument('--scale', type=float, default=0.75,
                    help='每一栏相对原图的缩放（默认 0.75 → 960 宽）')
    ap.add_argument('--trail-px', type=int, default=TRAIL)
    ap.add_argument('--limit', type=int, default=0, help='只渲染前 N 帧（调试用）')
    a = ap.parse_args()

    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from draw_zone_on_frame import compose_base_optical, project

    info, meta = load_recording(a.rec)
    if not meta:
        print('!! meta.jsonl 是空的 —— 录的时候话题没上来？')
        return 2
    if a.limit:
        meta = meta[:a.limit]

    K = (info['fx'], info['fy'], info['cx'], info['cy'])
    W, H = int(info['width']), int(info['height'])
    pw, ph = int(W * a.scale), int(H * a.scale)

    # 外参：投影要用它把 base_link 的点画回像素
    ext = json.load(open(os.path.expanduser(info['extrinsics_file']), encoding='utf-8'))
    R, t = compose_base_optical(ext)

    zone = None
    if a.zone:
        v = [float(x) for x in a.zone.split(',')]
        z = v[4] if len(v) == 5 else 0.0
        zone = [(v[0], v[2], z), (v[1], v[2], z), (v[1], v[3], z), (v[0], v[3], z)]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    band_h, cap_h = 26, 54
    vw, vh = pw * 2, ph + band_h + cap_h
    vw -= vw % 2
    vh -= vh % 2
    wr = cv2.VideoWriter(a.out, fourcc, a.fps, (vw, vh))
    if not wr.isOpened():
        print('!! 打不开 VideoWriter')
        return 2

    trail = []
    n_det = 0
    for k, rec in enumerate(meta):
        cf = os.path.join(a.rec, 'frames', f"{rec['i']:06d}.jpg")
        df = os.path.join(a.rec, 'depth', f"{rec['i']:06d}.png")
        color = cv2.imread(cf)
        depth = cv2.imread(df, cv2.IMREAD_UNCHANGED)
        if color is None or depth is None:
            continue
        color = cv2.resize(color, (pw, ph))
        dvis, dlo, dhi = depth_to_bgr(depth)
        dvis = cv2.resize(dvis, (pw, ph))
        dets = rec.get('dets', [])
        n_det = max(n_det, len(dets))

        if 'tool0' in rec:
            trail.append(tuple(rec['tool0']))
        trail = trail[-a.trail_px:]

        for panel in (color, dvis):
            if zone:
                pts = [project(R, t, p, K) for p in zone]
                if all(p is not None for p in pts):
                    pts = [(int(p[0] * a.scale), int(p[1] * a.scale)) for p in pts]
                    cv2.polylines(panel, [np.array(pts)], True, (255, 220, 0), 2)
            # tool0 轨迹（**实际**位置，由关节角算出来）
            for i, p in enumerate(trail):
                q = project(R, t, p, K)
                if q is None:
                    continue
                f = (i + 1) / len(trail)
                cv2.circle(panel, (int(q[0] * a.scale), int(q[1] * a.scale)), 2,
                           (int(255 * f), int(255 * (1 - f)), 0), -1)
            # 检测到的方块（base_link 坐标 → 像素）
            for dd in dets:
                q = project(R, t, (dd['x'], dd['y'], dd['z'] + 0.014), K)
                if q is None:
                    continue
                c = (int(q[0] * a.scale), int(q[1] * a.scale))
                cv2.circle(panel, c, 9, (0, 255, 0), 2)
                cv2.putText(panel, f"{dd['s']:.2f}", (c[0] + 11, c[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        lab = np.zeros((band_h, pw * 2, 3), np.uint8)
        cv2.putText(lab, 'COLOR + overlays', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(lab, f'DEPTH (turbo, {dlo:.0f}..{dhi:.0f} mm)', (pw + 6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cap = np.zeros((cap_h, pw * 2, 3), np.uint8)
        t0 = rec['tool0'] if 'tool0' in rec else None
        cv2.putText(cap, f"frame {k+1}/{len(meta)}   t={rec['t']:.1f}s   "
                         f"detections={len(dets)}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        if t0:
            cv2.putText(cap, f"tool0 (from joints, urdf_fk) = "
                             f"({t0[0]:+.3f}, {t0[1]:+.3f}, {t0[2]:+.3f}) m", (6, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if dets:
            d0 = dets[0]
            cv2.putText(cap, f"nearest target = ({d0['x']:+.3f}, {d0['y']:+.3f}, "
                             f"{d0['z']:+.4f})", (pw + 6, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        top = np.hstack([color, dvis])
        frame = np.vstack([lab, top, cap])
        if frame.shape[1] != vw or frame.shape[0] != vh:
            frame = cv2.resize(frame, (vw, vh))
        wr.write(frame)

    wr.release()
    print(f'✅ 渲染 {len(meta)} 帧 → {a.out}'
          f'（{os.path.getsize(a.out) / 1e6:.1f} MB，{pw*2}×{vh}，'
          f'最多同时 {n_det} 个检测）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
