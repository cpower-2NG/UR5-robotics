#!/usr/bin/env python3
"""阶段 2 的离线渲染器：把一次 A3 抓取渲染成可逐帧看的 MP4。

画面上必须有（契约入口判据 ③）
──────────────────────────────
  · 腕部彩色帧 + 深度点云按高度着色（看清块和台面）；
  · 机械臂骨架 + tool0 轨迹（规划器以为去了哪）；
  · 候选抓取位姿（GraspNet 的点 + approach 方向）；
  · 执行位姿（plan 的 waypoints → FK）与槽位/宫格；
  · 文本面板：轮次/块/类/候选分数/当前 waypoint。

为什么不复用 `replay_render.py`
───────────────────────────────
它读的是 `replay_record.py` 的 JPG 目录 + meta.jsonl，而 A3 run 是
`frames/<name>/{color,depth}.png + tf.json + meta.json`、`cycles/NN/pick.json`、
`round.json`。目录约定不同，硬套会把「哪一帧是哪一轮」搞混。

用法
────
    python3 scripts/stage2_render.py \\
        --run-dir ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision \\
        --cycle 5 --out .../render-cycle05.mp4

    # 只渲染前 120 帧（调试）
    python3 scripts/stage2_render.py --run-dir ... --cycle 5 --out /tmp/a.mp4 --max-frames 120
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import urkin                                                      # noqa: E402
from draw_zone_on_frame import project                            # noqa: E402

PANEL_H = 116
DEFAULT_FPS = 15.0


def _load_json(path, default=None):
    if not path or not os.path.exists(path):
        return default
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _cycle_dir(run_dir, cycle):
    return os.path.join(run_dir, 'cycles', '%02d' % int(cycle))


def _pick_path(run_dir, cycle):
    """``pick.json`` 的实际位置（A3 在 ``cycles/NN/``；阶段 2 在轮目录根上）。"""
    if _is_stage2_layout(run_dir):
        return os.path.join(run_dir, 'pick-%02d.json' % int(cycle))
    return os.path.join(_cycle_dir(run_dir, cycle), 'pick.json')


def _is_stage2_layout(run_dir):
    """A3 run 用 ``cycles/NN/``；阶段 2 轮用 ``pick-0N.json`` + ``exec-0N.jsonl``。

    2026-10-28 加：原来只认 A3 布局，于是阶段 2 的 5 轮一支 MP4 都出不来
    （契约 §出口要求「每轮 MP4」）。两套布局并行认，A3 路径完全不变。
    """
    return (not os.path.isdir(os.path.join(run_dir, 'cycles'))
            and os.path.exists(os.path.join(run_dir, 'pick-01.json')))


def _resolve_dir(run_dir, rel):
    """``round.json`` 里的相对路径按**轮目录 → 场景目录（父目录）**依次试。

    早期轮的 ``frame_pick`` 写的是 ``b2/frame-pick4`` / ``pick-now`` 这类相对
    **场景目录**的路径，晚期轮写 ``round-18/frame-pick``（同样相对场景目录）。
    """
    if not rel:
        return None
    bases = [''] if os.path.isabs(rel) else [
        run_dir, os.path.dirname(os.path.abspath(run_dir))]
    for b in bases:
        p = rel if not b else os.path.join(b, rel)
        if os.path.isdir(p):
            return p
    return None


def _load_stage2_cycle(run_dir, cycle, rnd):
    """阶段 2 轮：``pick-0N.json`` + ``exec-0N.jsonl`` + 抓取帧目录。"""
    plan = _load_json(os.path.join(run_dir, 'pick-%02d.json' % int(cycle)))
    items = rnd.get('items') or []
    # items 与 pick-0N.json 同序（规划时逐块写 plan），所以第 N 个 plan 对应 items[N-1]
    entry = dict(items[cycle - 1]) if 0 < cycle <= len(items) else {}
    frame_dir = (_resolve_dir(run_dir, rnd.get('frame_pick'))
                 or _resolve_dir(run_dir, 'frame-pick'))
    # 阶段 2 的 candidates.json 在轮目录根上（A3 放在帧目录里）
    cands = (_load_json(os.path.join(run_dir, 'candidates.json'))
             or (_load_json(os.path.join(frame_dir, 'candidates.json'))
                 if frame_dir else None))
    events = []
    ep = os.path.join(run_dir, 'exec-%02d.jsonl' % int(cycle))
    if os.path.exists(ep):
        for ln in open(ep, encoding='utf-8'):
            if ln.strip():
                try:
                    events.append(json.loads(ln))
                except json.JSONDecodeError:
                    pass
    return {'plan': plan, 'entry': entry, 'frame_dir': frame_dir,
            'candidates': cands, 'exec_events': events, 'round': rnd}


def load_cycle(run_dir, cycle):
    """返回 ``{plan, entry, frame_dir, candidates, exec_events}``；缺件给 None。"""
    if _is_stage2_layout(run_dir):
        return _load_stage2_cycle(
            run_dir, cycle, _load_json(os.path.join(run_dir, 'round.json'), {}))
    cd = _cycle_dir(run_dir, cycle)
    plan = _load_json(os.path.join(cd, 'pick.json'))
    rnd = _load_json(os.path.join(run_dir, 'round.json'), {})
    entry = next((c for c in rnd.get('cycles', []) if c.get('n') == cycle), {})
    frame_rel = entry.get('frame_pick')
    frame_dir = None
    if frame_rel:
        frame_dir = frame_rel if os.path.isabs(frame_rel) else os.path.join(
            run_dir, frame_rel)
    cands = _load_json(os.path.join(frame_dir, 'candidates.json')) if frame_dir else None
    events = []
    ep = os.path.join(cd, 'exec.jsonl')
    if os.path.exists(ep):
        for ln in open(ep, encoding='utf-8'):
            if ln.strip():
                try:
                    events.append(json.loads(ln))
                except json.JSONDecodeError:
                    pass
    return {'plan': plan, 'entry': entry, 'frame_dir': frame_dir,
            'candidates': cands, 'exec_events': events, 'round': rnd}


def _q_start(plan):
    qs = plan.get('q_start') or plan.get('meta', {}).get('q_start')
    if qs:
        return [float(v) for v in qs]
    return None


def trajectory_samples(plan, fps=DEFAULT_FPS):
    """``[(t, q, waypoint_name, phase)]``：从 q_start 线性插值到每个 waypoint。

    ``phase`` = ``'before'``（到达该 waypoint 前）/ ``'after'``。
    """
    q0 = _q_start(plan)
    if q0 is None:
        raise ValueError('plan 里没有 q_start')
    out = [(0.0, list(q0), 'start', 'before')]
    t = 0.0
    prev = list(q0)
    for i, wp in enumerate(plan.get('waypoints', [])):
        dur = float(wp.get('duration') or 2.0)
        n = max(2, int(round(dur * fps)))
        q1 = [float(v) for v in wp['q']]
        for k in range(1, n + 1):
            u = k / n
            q = [(1.0 - u) * a + u * b for a, b in zip(prev, q1)]
            out.append((t + dur * u, q, wp.get('name', 'wp%d' % i), 'after'))
        t += dur
        prev = q1
    return out


def _fk_points(chain, q):
    """base_link→tool0 各 link 的 base 坐标；失败返回空。"""
    pts, names = [], []
    try:
        pts.append(np.array([0.0, 0.0, 0.0]))
        names.append('base_link')
        for link in chain.link_names():
            T = chain.fk_link(link, q)
            pts.append(T[:3, 3])
            names.append(link)
    except Exception:                                            # noqa: BLE001
        return [], []
    return pts, names


def _height_colors(z, z0, span=0.09):
    """base z → BGR；台面蓝色、高处红色。"""
    u = np.clip((z - z0) / span, 0.0, 1.0)
    r = (u * 255).astype(np.uint8)
    b = ((1.0 - u) * 255).astype(np.uint8)
    g = np.full_like(r, 40)
    return np.column_stack([b, g, r])


def draw_pointcloud(img, frame, table_z_mm=None, max_points=30000):
    """深度点云按 base 高度着色，半透明叠回原图。

    只画台面带（``table_z_mm−8 mm … +100 mm``）：否则地板、桌面外沿也被
    涂上红蓝，整幅图变成雪花，看不出块。
    """
    P, pix = gc.backproject(frame['depth'], frame['K'])
    Pb = P @ frame['R_bc'].T + frame['t_bc']
    if len(Pb) == 0:
        return img
    if table_z_mm is not None:
        lo = (float(table_z_mm) - 8.0) / 1000.0
        hi = (float(table_z_mm) + 100.0) / 1000.0
        keep = (Pb[:, 2] >= lo) & (Pb[:, 2] <= hi)
        Pb, pix = Pb[keep], pix[keep]
        if len(Pb) == 0:
            return img
    step = max(1, len(Pb) // max_points)
    Pbs, uvs = Pb[::step], pix[::step]
    z0 = (float(table_z_mm) / 1000.0 if table_z_mm is not None
          else float(np.percentile(Pbs[:, 2], 1)))
    cols = _height_colors(Pbs[:, 2], z0)
    ov = img.copy()
    for (u, v), c in zip(uvs, cols):
        cv2.circle(ov, (int(u), int(v)), 1, [int(x) for x in c], -1)
    return cv2.addWeighted(ov, 0.35, img, 0.65, 0)


def _px(R, t, K, p_base, shape):
    uv = project(R, t, p_base, K)
    if uv is None:
        return None
    u, v = uv
    if not (-50 <= u < shape[1] + 50 and -50 <= v < shape[0] + 50):
        return None
    return int(round(u)), int(round(v))


def draw_skeleton(img, chain, q, R, t, K, color=(255, 220, 0)):
    pts, names = _fk_points(chain, q)
    if not pts:
        return
    uv = [_px(R, t, K, p, img.shape) for p in pts]
    for i in range(len(uv) - 1):
        if uv[i] is not None and uv[i + 1] is not None:
            cv2.line(img, uv[i], uv[i + 1], color, 2, cv2.LINE_AA)
    for i, p in enumerate(uv):
        if p is None:
            continue
        cv2.circle(img, p, 3, (0, 0, 0), -1)
        cv2.circle(img, p, 2, color, -1)


def draw_slots(img, round_doc, R, t, K, grid=None):
    """槽位（A3）或 3×3 宫格（阶段 2）投影到画面。"""
    z = 0.0
    cells = []
    if grid:
        cells = [c for c in grid.get('cells', [])
                 if c.get('center_base_mm')]
        for c in cells:
            c['_xy_mm'] = c['center_base_mm']
    else:
        for i, xy in enumerate(round_doc.get('slots_mm', [])):
            cells.append({'idx': i, '_xy_mm': xy,
                          'color': (round_doc.get('slot_classes') or [''] * 9)[i]
                          if i < len(round_doc.get('slot_classes') or []) else ''})
    for c in cells:
        x, y = float(c['_xy_mm'][0]), float(c['_xy_mm'][1])
        p = _px(R, t, K, [x / 1000.0, y / 1000.0, z], img.shape)
        if p is None:
            continue
        cv2.circle(img, p, 22, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, str(c.get('idx', '')), (p[0] - 6, p[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1,
                    cv2.LINE_AA)


def draw_candidates(img, cands, R, t, K):
    if not cands:
        return
    best_by_block = {}
    for pb in cands.get('per_block', []):
        if pb.get('best'):
            best_by_block[pb.get('block_idx')] = pb['best']
    for i, c in enumerate(cands.get('candidates', [])[:80]):
        p = _px(R, t, K, c['xyz_base'], img.shape)
        if p is None:
            continue
        col = (0, 220, 0) if c['score'] >= 0.3 else (0, 170, 170)
        cv2.circle(img, p, 2, col, -1)
        a = np.asarray(c.get('approach_base', [0, 0, -1]), float)
        if np.linalg.norm(a) > 1e-9:
            a = a / np.linalg.norm(a) * 0.025
            q = _px(R, t, K, np.asarray(c['xyz_base']) + a, img.shape)
            if q is not None and np.hypot(q[0] - p[0], q[1] - p[1]) <= 70:
                cv2.line(img, p, q, col, 1, cv2.LINE_AA)
    for bi, best in best_by_block.items():
        p = _px(R, t, K, best['xyz_base'], img.shape)
        if p is None:
            continue
        cv2.drawMarker(img, p, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, '#%d %.2f' % (bi, best['score']),
                    (p[0] + 8, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 0), 1, cv2.LINE_AA)


def draw_plan(img, chain, plan, R, t, K):
    """执行位姿：每个 waypoint 的 tool0 点 + 折线。"""
    pts = []
    for wp in plan.get('waypoints', []):
        try:
            T = chain.fk([float(v) for v in wp['q']])
        except Exception:                                        # noqa: BLE001
            continue
        p = _px(R, t, K, T[:3, 3], img.shape)
        if p is None:
            continue
        pts.append((p, wp.get('name', '')))
    for i in range(len(pts) - 1):
        cv2.line(img, pts[i][0], pts[i + 1][0], (0, 165, 255), 1, cv2.LINE_AA)
    for p, name in pts:
        cv2.drawMarker(img, p, (0, 165, 255), cv2.MARKER_DIAMOND, 10, 1)
        cv2.putText(img, name, (p[0] + 5, p[1] + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 165, 255), 1,
                    cv2.LINE_AA)


def _slot_cells(round_doc, grid=None):
    """槽位/宫格统一成 ``[(idx, [x_mm, y_mm])]``。"""
    out = []
    if grid:
        for c in grid.get('cells', []):
            xy = c.get('center_base_mm')
            if xy:
                out.append((c.get('idx', len(out)), [float(xy[0]), float(xy[1])]))
    else:
        for i, xy in enumerate(round_doc.get('slots_mm', [])):
            out.append((i, [float(xy[0]), float(xy[1])]))
    return out


def draw_topview(img, plan, chain, q, cands, round_doc, grid=None,
                 size=(360, 300), margin=18):
    """右上角俯视插图（base XY）。

    腕部相机**看不到机械臂**：tool0 就在相机背后，``project`` 必然返回
    None。所以骨架和轨迹只能在俯视图里画 —— 这不是退而求其次，是腕部
    视角的物理事实。插图里同时放块、槽位/格、候选点、规划轨迹和当前 tool0。
    """
    path, links = [], []
    for wp in plan.get('waypoints', []):
        try:
            path.append(chain.fk([float(v) for v in wp['q']])[:3, 3])
        except Exception:                                        # noqa: BLE001
            pass
    try:
        path.append(chain.fk(q)[:3, 3])
    except Exception:                                            # noqa: BLE001
        pass
    pts, _ = _fk_points(chain, q)
    links = [p for p in pts]
    blocks = [[pb['block_xy'][0] / 1000.0, pb['block_xy'][1] / 1000.0]
              for pb in (cands or {}).get('per_block', []) if pb.get('block_xy')]
    slots = [[x / 1000.0, y / 1000.0] for _, (x, y) in _slot_cells(round_doc, grid)]

    allp = path + links + [np.array([0.0, 0.0, 0.0])] + [
        np.array([p[0], p[1], 0.0]) for p in blocks + slots]
    if not allp:
        return
    xy = np.array([[p[0], p[1]] for p in allp], float)
    lo = xy.min(axis=0) - 0.04
    hi = xy.max(axis=0) + 0.04
    span = np.maximum(hi - lo, 0.05)
    w, h = size
    if img.shape[1] < w + 20 or img.shape[0] < h + 20:
        return
    scale = min((w - 2 * margin) / span[0], (h - 2 * margin) / span[1])
    mid = (lo + hi) / 2.0
    cx, cy = w / 2.0, h / 2.0

    def pxy(p):
        return (int(round(cx + (p[0] - mid[0]) * scale)),
                int(round(cy - (p[1] - mid[1]) * scale)))

    ox, oy = img.shape[1] - w - 8, 8
    roi = cv2.addWeighted(img[oy:oy + h, ox:ox + w], 0.2,
                          np.full((h, w, 3), 20, np.uint8), 0.8, 0)
    cv2.rectangle(roi, (0, 0), (w - 1, h - 1), (130, 130, 130), 1)
    cv2.putText(roi, 'base XY (top view)', (8, 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1,
                cv2.LINE_AA)

    # 100 mm 网格
    step = 0.1
    gx = np.ceil(lo[0] / step) * step
    while gx <= hi[0]:
        p1, p2 = pxy([gx, lo[1]]), pxy([gx, hi[1]])
        cv2.line(roi, p1, p2, (55, 55, 55), 1)
        gx += step
    gy = np.ceil(lo[1] / step) * step
    while gy <= hi[1]:
        p1, p2 = pxy([lo[0], gy]), pxy([hi[0], gy])
        cv2.line(roi, p1, p2, (55, 55, 55), 1)
        gy += step

    for _, (x, y) in _slot_cells(round_doc, grid):
        cv2.circle(roi, pxy([x / 1000.0, y / 1000.0]), 8,
                   (200, 200, 200), 1, cv2.LINE_AA)
    for p in blocks:
        cv2.rectangle(roi, (pxy(p)[0] - 3, pxy(p)[1] - 3),
                      (pxy(p)[0] + 3, pxy(p)[1] + 3), (80, 200, 80), -1)
    for c in (cands or {}).get('candidates', [])[:40]:
        xyz = c['xyz_base']
        cv2.circle(roi, pxy([xyz[0], xyz[1]]), 1, (0, 160, 0), -1)

    for i in range(1, len(path)):
        cv2.line(roi, pxy(path[i - 1]), pxy(path[i]), (0, 165, 255), 2,
                 cv2.LINE_AA)
    for p in links:
        cv2.circle(roi, pxy(p), 2, (255, 220, 0), -1)
    for i in range(1, len(links)):
        cv2.line(roi, pxy(links[i - 1]), pxy(links[i]), (255, 220, 0), 1,
                 cv2.LINE_AA)
    if path:
        cv2.drawMarker(roi, pxy(path[-1]), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.drawMarker(roi, pxy([0.0, 0.0]), (255, 255, 255), cv2.MARKER_STAR, 12, 1)

    # 比例尺
    p1, p2 = pxy([lo[0] + 0.02, lo[1] + 0.02]), pxy([lo[0] + 0.12, lo[1] + 0.02])
    cv2.line(roi, p1, p2, (220, 220, 220), 2)
    cv2.putText(roi, '100 mm', (p1[0], p1[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1,
                cv2.LINE_AA)
    img[oy:oy + h, ox:ox + w] = roi


def _panel(img, lines, font_path='/mnt/c/Windows/Fonts/msyh.ttc'):
    """底部文本面板；有中文字体就用，没有就英文回退。"""
    h, w = img.shape[:2]
    panel = np.full((PANEL_H, w, 3), 28, np.uint8)
    try:
        from PIL import Image, ImageDraw, ImageFont
        if os.path.exists(font_path):
            font = ImageFont.truetype(font_path, 19)
        else:
            font = ImageFont.load_default()
        rgb = cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)
        d = ImageDraw.Draw(im)
        for i, s in enumerate(lines[:4]):
            d.text((10, 8 + i * 25), s, fill=(235, 235, 235), font=font)
        panel = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    except ImportError:
        for i, s in enumerate(lines[:4]):
            cv2.putText(panel, s, (10, 24 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1)
    return np.vstack([img, panel])


def _writer(path, fps, size):
    for codec in ('mp4v', 'avc1', 'XVID'):
        wr = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*codec), fps, size)
        if wr.isOpened():
            return wr, codec
        wr.release()
    return None, None


def available_cycles(run_dir):
    """阶段 2 轮里实际存在的 ``pick-0N.json`` 编号（升序）。"""
    out = []
    for name in os.listdir(run_dir):
        if name.startswith('pick-') and name.endswith('.json'):
            try:
                out.append(int(name[len('pick-'):-len('.json')]))
            except ValueError:
                pass
    if not out:                                  # A3 布局
        cdir = os.path.join(run_dir, 'cycles')
        if os.path.isdir(cdir):
            for name in os.listdir(cdir):
                if name.isdigit():
                    out.append(int(name))
    return sorted(out)


def _render_cycles(run_dir, cycles, out_path, fps=DEFAULT_FPS, max_frames=0,
                   urdf=None, grid=None):
    """把若干轮次（阶段 2 = 一块一次）连成一支出 MP4。"""
    if not urdf or not os.path.exists(urdf):
        raise FileNotFoundError('缺 URDF：--urdf 或 $PRECISION_URDF')
    wr = None
    codec = None
    total = 0
    try:
        for cycle in cycles:
            meta = load_cycle(run_dir, cycle)
            if not meta['plan']:
                raise FileNotFoundError('缺 %s' % _pick_path(run_dir, cycle))
            if not meta['frame_dir'] or not os.path.isdir(meta['frame_dir']):
                raise FileNotFoundError('缺抓取帧目录（round.json 的 frame_pick）')

            frame = gc.load_frame(meta['frame_dir'])
            R, t, K = frame['R_bc'], frame['t_bc'], frame['K']
            Kt = (float(K[0, 0]), float(K[1, 1]), float(K[0, 2]),
                  float(K[1, 2]))
            table_z_mm = (meta['candidates'] or {}).get('table_z_mm')
            base = cv2.cvtColor(frame['color'], cv2.COLOR_RGB2BGR)
            base = draw_pointcloud(base, frame, table_z_mm=table_z_mm)

            chain = urkin.Chain(urdf)
            samples = trajectory_samples(meta['plan'], fps=fps)
            if max_frames:
                samples = samples[:max_frames]

            if wr is None:
                wr, codec = _writer(out_path, fps,
                                    (base.shape[1], base.shape[0] + PANEL_H))
                if wr is None:
                    raise RuntimeError(
                        'VideoWriter 打不开（mp4v/avc1/XVID 都失败）')

            entry = meta['entry'] or {}
            plan_meta = meta['plan'].get('meta', {})
            cand_best = None
            for pb in (meta['candidates'] or {}).get('per_block', []):
                if pb.get('best'):
                    cand_best = pb['best']
                    break
            for i, (tt, q, wp_name, _phase) in enumerate(samples):
                img = base.copy()
                draw_slots(img, meta['round'], R, t, Kt, grid=grid)
                draw_candidates(img, meta['candidates'], R, t, Kt)
                draw_plan(img, chain, meta['plan'], R, t, Kt)
                # 腕部相机跟 tool0 一起动：tool0 的投影只在极少数帧落进画面，
                # 留下的残影会横贯全图且越积越久。轨迹统一画在右上俯视图里。
                draw_skeleton(img, chain, q, R, t, Kt)
                draw_topview(img, meta['plan'], chain, q, meta['candidates'],
                             meta['round'], grid=grid)

                try:
                    tool = chain.fk(q)[:3, 3]
                except Exception:                                # noqa: BLE001
                    tool = np.zeros(3)
                lines = [
                    '轮次 %02d · %s · 块 #%s %s' % (
                        int(cycle),
                        os.path.basename(meta['frame_dir'].rstrip('/')),
                        entry.get('block_idx',
                                  plan_meta.get('block_idx', '?')),
                        entry.get('class') or plan_meta.get('class') or '?'),
                    'waypoint %s (%d/%d)  t=%.1fs' % (
                        wp_name, i + 1, len(samples), tt),
                    '候选 %s · 槽位 %s' % (
                        ('score %.2f width %.1f mm angle %.1f°' % (
                            cand_best['score'], cand_best['width_mm'],
                            cand_best['angle_deg'])) if cand_best else '无',
                        entry.get('slot_idx', '?')),
                    'tool0 (%+.0f, %+.0f, %+.0f) mm · 骨架/轨迹见右上俯视图 · %s'
                    % (tool[0] * 1000, tool[1] * 1000, tool[2] * 1000, codec),
                ]
                wr.write(_panel(img, lines))
                total += 1
    finally:
        if wr is not None:
            wr.release()
    return {'out': out_path, 'frames': total, 'fps': fps,
            'codec': codec, 'cycles': len(cycles)}


def render(run_dir, cycle, out_path, fps=DEFAULT_FPS, max_frames=0,
           urdf=None, grid=None):
    return _render_cycles(run_dir, [cycle], out_path, fps=fps,
                          max_frames=max_frames, urdf=urdf, grid=grid)


def resolve_grid(run_dir, grid_arg=None):
    """宫格来源：``--grid`` → ``round.json`` 内嵌（**带 cells 才用**）→ ``grid_json`` 指路。

    阶段 2 轮的 ``round.json`` 只内嵌 ``grid_order`` / ``grid_center_base_mm`` /
    ``grid_yaw_base_deg`` / ``T_base_pad``，**没有 cells**；逐格中心在 ``grid.json``
    （``grid_json`` 字段给的是相对场景目录的路径，轮目录是它的子目录）。
    """
    if grid_arg:
        return _load_json(grid_arg)
    rnd = _load_json(os.path.join(run_dir, 'round.json'), {}) or {}
    g = rnd.get('grid')
    if g and g.get('cells'):
        return g
    rel = rnd.get('grid_json')
    if rel:
        paths = [rel] if os.path.isabs(rel) else [
            os.path.join(run_dir, rel),
            os.path.join(os.path.dirname(os.path.abspath(run_dir)), rel)]
        for p in paths:
            if os.path.exists(p):
                return _load_json(p)
    return g


def main():
    ap = argparse.ArgumentParser(description='阶段 2 离线渲染器')
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--cycle', required=True,
                    help='轮次号；阶段 2 轮可写 all = 已有的 pick-0N 全连成一支')
    ap.add_argument('--out', required=True)
    ap.add_argument('--fps', type=float, default=DEFAULT_FPS)
    ap.add_argument('--max-frames', type=int, default=0,
                    help='每轮最多帧数（0 = 不限）')
    ap.add_argument('--urdf', default=os.environ.get('PRECISION_URDF'))
    ap.add_argument('--grid', default=None, help='grid.json（阶段 2 宫格）')
    a = ap.parse_args()
    if str(a.cycle).lower() == 'all':
        cycles = available_cycles(a.run_dir)
        if not cycles:
            print('渲染失败：%s 里没有可渲染的轮次' % a.run_dir, file=sys.stderr)
            return 2
    else:
        try:
            cycles = [int(a.cycle)]
        except ValueError:
            print('渲染失败：--cycle 要轮次号或 all，收到 %r' % a.cycle,
                  file=sys.stderr)
            return 2
    urdf = a.urdf or os.path.join(a.run_dir, 'robot.urdf')
    grid = resolve_grid(a.run_dir, a.grid)
    try:
        r = _render_cycles(a.run_dir, cycles, a.out, fps=a.fps,
                           max_frames=a.max_frames, urdf=urdf, grid=grid)
    except Exception as e:                                       # noqa: BLE001
        print('渲染失败：%s: %s' % (type(e).__name__, e), file=sys.stderr)
        return 2
    print('MP4：%(out)s (%(frames)d 帧, %(codec)s, %(cycles)d 轮)' % r)
    return 0


if __name__ == '__main__':
    sys.exit(main())
