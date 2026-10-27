#!/usr/bin/env python3
"""阶段 2 出口判分器：逐块判「色格 / yaw / 格内位置」。

输入（一份 JSON，由 `stage2_round` / 人工整理）
──────────────────────────────────────────────
```json
{
  "round": 1,
  "grid": { ... stage2_pad.py 产出的 grid.json ... },
  "poses": [
    {"name": "survey_place",
     "blocks": [{"block_id": 0, "class": "red", "class_status": "both",
                 "center_pad_mm": [1.2, -2.0], "yaw_deg": 3.4}]},
    {"name": "survey_place2", "blocks": [...]}
  ],
  "expected": [
    {"block_id": 0, "arm": "geometric", "cell_idx": 0,
     "yaw_exec_deg": 12.0, "ran": true, "exec_ok": true}
  ],
  "arms": ["geometric", "zero", "graspnet"]
}
```

* `center_pad_mm` / `yaw_deg` 都在**垫系**；格线方向 = 0°。
* `class_status` 取 A1 双源的 `both/hsv/yolo/unknown/conflict`。
  `unknown/conflict` 不进机判，转 `person_review` —— 不能把「没测准」
  混进成功率。
* 没有 `expected` 时从 `round.json` 的 `cycles` 推：`block_idx`、`slot_idx`、
  `go`、`ok`；`arm` 缺省 `unknown`。

三项判据（阈值由用户基线后拍板；默认 8° / 12 mm）
────────────────────────────────────────────────
1. **色格**：观测类 == 该格颜色；且观测块落在预期格；
2. **yaw**：`|fold90(观测 yaw)| <= --threshold-yaw-deg`；
3. **位置**：块心到**预期格心**的距离 <= `--threshold-pos-mm`。

多姿态一致性：同一 `block_id` 在 ≥2 个 pose 的 yaw 差 ≤3°、位置差 ≤3 mm；
不过 → `person_review`，不进机判统计。

退出码：0 = 机判项全过；1 = 已判但有不过/待复核；2 = 数据缺失或坏。
"""

import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

YAW_TOL_DEG = 8.0
POS_TOL_MM = 12.0
POSE_YAW_TOL_DEG = 3.0
POSE_POS_TOL_MM = 3.0
CELL_RADIUS_FRAC = 0.45

STATUSES = ('ok', 'color_wrong', 'wrong_cell', 'yaw_fail', 'pos_fail',
            'not_measured', 'not_run', 'person_review')
GRADED = ('ok', 'color_wrong', 'wrong_cell', 'yaw_fail', 'pos_fail')

CLASS_BGR = {
    'red': (40, 40, 220), 'orange': (0, 140, 255), 'tan': (60, 130, 180),
    'yellow': (0, 220, 220), 'yellowgreen': (60, 200, 60),
    'cyan': (220, 200, 0), 'purple': (200, 60, 140),
    'spare': (90, 90, 90), '': (90, 90, 90),
}


# ---------------------------------------------------------------- 纯函数
def fold_mod90(deg):
    """角度折到 [0, 90)；89.9 保留 89.9，−1 → 89。"""
    return float(deg % 90.0)


def ang_diff_mod90(a, b):
    """两角在 mod 90 圆周上的最短距离，0..45。"""
    d = abs((float(a) - float(b)) % 90.0)
    return float(min(d, 90.0 - d))


def yaw_err_mod90(yaw_deg):
    """块边相对格线（0°）的误差，0..45。"""
    return ang_diff_mod90(yaw_deg, 0.0)


def pos_err(obs_xy, cell_xy):
    return float(math.hypot(float(obs_xy[0]) - float(cell_xy[0]),
                            float(obs_xy[1]) - float(cell_xy[1])))


def nearest_cell(xy, cells, max_mm):
    """``cells`` = ``[{idx, center_pad_mm, color}]``；超界返回 (None, d)。"""
    best, bd = None, float('inf')
    for c in cells:
        d = pos_err(xy, c['center_pad_mm'])
        if d < bd:
            best, bd = c, d
    if best is None or bd > max_mm:
        return None, bd
    return best, bd


def _median(vals):
    v = [x for x in vals if x is not None]
    return float(np.median(v)) if v else None


def _max(vals):
    v = [x for x in vals if x is not None]
    return float(max(v)) if v else None


def cell_color(grid, cell_idx):
    for c in grid.get('cells', []):
        if int(c.get('idx', -1)) == int(cell_idx):
            col = c.get('color', '')
            return '' if col == 'spare' else col
    return None


def cell_center(grid, cell_idx):
    for c in grid.get('cells', []):
        if int(c.get('idx', -1)) == int(cell_idx):
            return [float(v) for v in c['center_pad_mm']]
    return None


# ---------------------------------------------------------------- 观测合并
def merge_poses(poses, yaw_tol=POSE_YAW_TOL_DEG, pos_tol=POSE_POS_TOL_MM):
    """同一 block_id 跨 pose 合并；返回 ``{block_id: {..., 'consistent': bool}}``。

    没有 block_id 的记录按出现顺序编号（仅测试/兜底用）。
    """
    by_id = {}
    for pi, p in enumerate(poses):
        for bi, b in enumerate(p.get('blocks', [])):
            bid = b.get('block_id', bi)
            rec = by_id.setdefault(bid, {
                'block_id': bid, 'obs': [], 'class': None,
                'class_status': None, 'center_pad_mm': [],
                'yaw_deg': [], 'poses': [],
            })
            rec['obs'].append((pi, b))
            rec['poses'].append(pi)
            rec['class'] = rec['class'] or b.get('class')
            rec['class_status'] = rec['class_status'] or b.get('class_status')
            if b.get('center_pad_mm') is not None:
                rec['center_pad_mm'].append([float(v) for v in b['center_pad_mm']])
            if b.get('yaw_deg') is not None:
                rec['yaw_deg'].append(float(b['yaw_deg']))

    for rec in by_id.values():
        rec['consistent'] = True
        rec['reason'] = ''
        ys, ps = rec['yaw_deg'], rec['center_pad_mm']
        if len(ys) >= 2:
            span = _angular_span(ys)
            if span is not None and span > yaw_tol:
                rec['consistent'] = False
                rec['reason'] = 'yaw 跨姿态差 %.2f° > %.2f°' % (span, yaw_tol)
        if len(ps) >= 2:
            d = max(pos_err(a, b) for a in ps for b in ps)
            if d > pos_tol:
                rec['consistent'] = False
                rec['reason'] = '位置跨姿态差 %.2f mm > %.2f mm' % (d, pos_tol)
        rec['center_pad_mm'] = _mean_xy(ps)
        rec['yaw_deg'] = _circular_mean_deg(ys)
        rec['n_poses'] = len(set(rec['poses']))
    return by_id


def _mean_xy(ps):
    if not ps:
        return None
    return [float(np.mean([p[0] for p in ps])),
            float(np.mean([p[1] for p in ps]))]


def _circular_mean_deg(vals):
    """mod 90 圆周均值（用双角公式，避免 89/1 平均成 45）。"""
    v = [x for x in vals if x is not None]
    if not v:
        return None
    a = np.radians(np.array(v, float) * 4.0)
    m = float(np.degrees(np.arctan2(np.sin(a).mean(),
                                    np.cos(a).mean())) / 4.0 % 90.0)
    # 浮点负零会算成 90.0；在 mod 90 圆周上 90 就是 0。
    return 0.0 if m >= 90.0 - 1e-9 else m


def _angular_span(vals):
    """mod 90 圆周上最小覆盖弧长（0..90）。"""
    v = [x % 90.0 for x in vals if x is not None]
    if len(v) < 2:
        return 0.0 if v else None
    v.sort()
    gaps = [v[i + 1] - v[i] for i in range(len(v) - 1)]
    gaps.append(v[0] + 90.0 - v[-1])
    return float(90.0 - max(gaps))


# ---------------------------------------------------------------- 判分
def expected_rows(obs, grid):
    """优先用观测文件的 expected；否则从 round.json 的 cycles 推。"""
    if obs.get('expected'):
        return obs['expected']
    rows = []
    for c in (obs.get('round_doc') or {}).get('cycles', []):
        if not c.get('go'):
            continue
        rows.append({'block_id': c.get('block_idx'), 'cell_idx': c.get('slot_idx'),
                     'arm': c.get('arm', 'unknown'), 'ran': bool(c.get('go')),
                     'exec_ok': bool(c.get('ok'))})
    return rows


def score_cell(rec, exp, grid, yaw_tol=YAW_TOL_DEG, pos_tol=POS_TOL_MM,
               cell_radius_frac=CELL_RADIUS_FRAC):
    """一块一行；返回 ``{block_id, arm, status, ...}``。"""
    out = {
        'block_id': rec.get('block_id'),
        'arm': exp.get('arm', 'unknown'),
        'cell_idx': exp.get('cell_idx'),
        'cell_color': cell_color(grid, exp.get('cell_idx')),
        'class': rec.get('class'),
        'class_status': rec.get('class_status'),
        'center_pad_mm': rec.get('center_pad_mm'),
        'yaw_deg': rec.get('yaw_deg'),
        'n_poses': rec.get('n_poses', 0),
        'yaw_err_deg': None, 'pos_err_mm': None, 'obs_cell_idx': None,
    }
    if not exp.get('ran', True):
        out['status'] = 'not_run'
        return out
    if rec.get('center_pad_mm') is None:
        out['status'] = 'not_measured'
        return out
    if rec.get('n_poses', 0) < 2:
        out['status'] = 'person_review'
        out['reason'] = '只有 %d 个姿态（要求 ≥2）' % rec.get('n_poses', 0)
        return out
    if not rec.get('consistent', True):
        out['status'] = 'person_review'
        out['reason'] = rec.get('reason', '多姿态不一致')
        return out
    if rec.get('class_status') in ('unknown', 'conflict', None):
        out['status'] = 'person_review'
        out['reason'] = '颜色来源 %r 不机判' % rec.get('class_status')
        return out

    pitch = float(grid.get('pitch_mm', 60.0))
    cell, _ = nearest_cell(rec['center_pad_mm'], grid.get('cells', []),
                           pitch * cell_radius_frac)
    out['obs_cell_idx'] = cell['idx'] if cell else None
    out['yaw_err_deg'] = round(yaw_err_mod90(rec['yaw_deg']), 3) \
        if rec.get('yaw_deg') is not None else None

    cc = cell_center(grid, exp.get('cell_idx'))
    if cc is None:
        out['status'] = 'person_review'
        out['reason'] = 'grid 里没有预期格 %r' % exp.get('cell_idx')
        return out
    out['pos_err_mm'] = round(pos_err(rec['center_pad_mm'], cc), 3)

    want = out['cell_color']
    if want and rec.get('class') != want:
        out['status'] = 'color_wrong'
    elif cell is None or int(cell['idx']) != int(exp.get('cell_idx')):
        out['status'] = 'wrong_cell'
    elif out['yaw_err_deg'] is None:
        out['status'] = 'person_review'
        out['reason'] = '没有 yaw（观测缺角）'
    elif out['yaw_err_deg'] > yaw_tol:
        out['status'] = 'yaw_fail'
    elif out['pos_err_mm'] > pos_tol:
        out['status'] = 'pos_fail'
    else:
        out['status'] = 'ok'
    return out


def summarize(rows, arms=None):
    arms = arms or sorted({r['arm'] for r in rows})
    per_arm = {}
    for arm in arms:
        rs = [r for r in rows if r['arm'] == arm]
        graded = [r for r in rs if r['status'] in GRADED]
        per_arm[arm] = {
            'n_expected': len(rs),
            'n_machine': len(graded),
            'n_ok': sum(1 for r in graded if r['status'] == 'ok'),
            'color_ok': sum(1 for r in graded
                            if r['status'] not in ('color_wrong', 'wrong_cell')),
            'yaw_median_deg': _median([r['yaw_err_deg'] for r in graded]),
            'yaw_max_deg': _max([r['yaw_err_deg'] for r in graded]),
            'pos_median_mm': _median([r['pos_err_mm'] for r in graded]),
            'pos_max_mm': _max([r['pos_err_mm'] for r in graded]),
            'success_rate': (sum(1 for r in graded if r['status'] == 'ok')
                             / len(graded)) if graded else None,
            'statuses': {s: sum(1 for r in rs if r['status'] == s)
                         for s in STATUSES},
        }
    graded_all = [r for r in rows if r['status'] in GRADED]
    overall = {
        'n_expected': len(rows),
        'n_machine': len(graded_all),
        'n_ok': sum(1 for r in graded_all if r['status'] == 'ok'),
        'n_person_review': sum(1 for r in rows if r['status'] == 'person_review'),
        'yaw_median_deg': _median([r['yaw_err_deg'] for r in graded_all]),
        'yaw_max_deg': _max([r['yaw_err_deg'] for r in graded_all]),
        'pos_median_mm': _median([r['pos_err_mm'] for r in graded_all]),
        'pos_max_mm': _max([r['pos_err_mm'] for r in graded_all]),
        'success_rate': (sum(1 for r in graded_all if r['status'] == 'ok')
                         / len(graded_all)) if graded_all else None,
    }
    return {'overall': overall, 'per_arm': per_arm}


# ---------------------------------------------------------------- 标注图
def _font(size=16, path='/mnt/c/Windows/Fonts/msyh.ttc'):
    try:
        from PIL import ImageFont
        return ImageFont.truetype(path, size) if os.path.exists(path) else \
            ImageFont.load_default()
    except ImportError:
        return None


def _draw_text(img, xy, text, color=(240, 240, 240), size=15):
    try:
        from PIL import Image, ImageDraw
        im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        ImageDraw.Draw(im).text(xy, str(text), fill=color[::-1],
                                font=_font(size))
        return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    except ImportError:
        cv2.putText(img, str(text), xy, cv2.FONT_HERSHEY_SIMPLEX,
                    size / 30.0, color, 1, cv2.LINE_AA)
        return img


def draw_score_png(grid, rows, path, size=900):
    """垫系俯视：格、预期块、观测块、yaw 箭头、状态文字。"""
    cells = grid.get('cells', [])
    if not cells:
        raise ValueError('grid.cells 为空，画不了判分图')
    pts = np.array([[c['center_pad_mm'][0], c['center_pad_mm'][1]]
                    for c in cells], float)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = max(float((hi - lo).max()) + 2 * float(grid.get('pitch_mm', 60.0)),
               60.0)
    mid = (lo + hi) / 2.0
    scale = (size - 120) / span

    def px(xy):
        return (int(round(size / 2.0 + (xy[0] - mid[0]) * scale)),
                int(round(size / 2.0 - (xy[1] - mid[1]) * scale)))

    img = np.full((size, size, 3), 26, np.uint8)
    pitch_px = float(grid.get('pitch_mm', 60.0)) * scale
    for c in cells:
        x, y = px(c['center_pad_mm'])
        col = CLASS_BGR.get(c.get('color', ''), (90, 90, 90))
        cv2.rectangle(img, (int(x - pitch_px / 2), int(y - pitch_px / 2)),
                      (int(x + pitch_px / 2), int(y + pitch_px / 2)), col, 2)
        cv2.rectangle(img, (int(x - 6), int(y - 6)), (int(x + 6), int(y + 6)),
                      col, -1)
        img = _draw_text(img, (x - 18, y + 10),
                         '%s' % (c.get('color') or 'spare'), (200, 200, 200),
                         13)

    for r in rows:
        if r.get('center_pad_mm') is None:
            continue
        x, y = px(r['center_pad_mm'])
        st = r['status']
        col = (0, 220, 0) if st == 'ok' else (
            (0, 200, 255) if st in ('not_measured', 'person_review')
            else (0, 0, 230))
        cv2.rectangle(img, (x - 9, y - 9), (x + 9, y + 9), col, 2)
        if r.get('yaw_deg') is not None:
            a = math.radians(r['yaw_deg'])
            q = (int(x + 26 * math.cos(a)), int(y - 26 * math.sin(a)))
            cv2.line(img, (x, y), q, col, 2, cv2.LINE_AA)
        img = _draw_text(img, (x + 12, y - 22),
                         '#%s %s' % (r['block_id'], st), col, 13)

    img = _draw_text(img, (14, 12),
                     'round %s · yaw<=%.0f° · pos<=%.0f mm · %d 行' % (
                         grid.get('round', '?'), grid.get('_yaw_tol', 8),
                         grid.get('_pos_tol', 12), len(rows)),
                     (230, 230, 230), 16)
    cv2.imwrite(path, img)
    return path


# ---------------------------------------------------------------- 报告
def _md(res, yaw_tol, pos_tol):
    o = res['summary']['overall']
    lines = ['# 阶段 2 判分', '',
             '阈值：yaw ≤ %.1f°（mod 90）、位置 ≤ %.1f mm。' % (yaw_tol, pos_tol),
             '',
             '| 臂 | 预期 | 机判 | 通过 | 色格正确 | yaw 中位/最大 | 位置 中位/最大 | 成功率 |',
             '|---|---|---|---|---|---|---|---|']
    for arm, s in res['summary']['per_arm'].items():
        fmt = lambda v, u='': '-' if v is None else '%.2f%s' % (v, u)  # noqa: E731
        lines.append('| %s | %d | %d | %d | %d | %s / %s | %s / %s | %s |' % (
            arm, s['n_expected'], s['n_machine'], s['n_ok'], s['color_ok'],
            fmt(s['yaw_median_deg'], '°'), fmt(s['yaw_max_deg'], '°'),
            fmt(s['pos_median_mm'], ' mm'), fmt(s['pos_max_mm'], ' mm'),
            '-' if s['success_rate'] is None else '%.0f%%' % (
                s['success_rate'] * 100)))
    lines += ['', '总计：预期 %d，机判 %d，通过 %d，待人工复核 %d，成功率 %s' % (
        o['n_expected'], o['n_machine'], o['n_ok'], o['n_person_review'],
        '-' if o['success_rate'] is None else '%.0f%%' % (
            o['success_rate'] * 100)), '',
        '## 逐块', '',
        '| 块 | 臂 | 状态 | 类 | 色源 | 预期格/色 | 实测格 | yaw 误差 | 位置误差 | 姿态数 | 备注 |',
        '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in res['rows']:
        lines.append('| %s | %s | %s | %s | %s | %s/%s | %s | %s | %s | %d | %s |' % (
            r['block_id'], r['arm'], r['status'], r.get('class') or '-',
            r.get('class_status') or '-', r.get('cell_idx'),
            r.get('cell_color') or 'spare', r.get('obs_cell_idx'),
            '-' if r.get('yaw_err_deg') is None else '%.2f°' % r['yaw_err_deg'],
            '-' if r.get('pos_err_mm') is None else '%.2f mm' % r['pos_err_mm'],
            r.get('n_poses', 0), r.get('reason', '')))
    lines += ['', '## 判读', '',
              '- `person_review` 不混进机判统计（颜色来源不明 / 多姿态不一致 / 缺 yaw）。',
              '- `not_run` 是计划没跑；`not_measured` 是跑了但终态没测到。',
              '- 阈值是基线后由用户拍板的；改阈值只是重判，不需要重跑机械臂。']
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------- CLI
def load_observations(run_dir=None, obs_path=None):
    if obs_path:
        with open(obs_path, encoding='utf-8') as f:
            return json.load(f)
    if run_dir:
        p = os.path.join(run_dir, 'score-input.json')
        if os.path.exists(p):
            with open(p, encoding='utf-8') as f:
                return json.load(f)
        rp = os.path.join(run_dir, 'round.json')
        if os.path.exists(rp):
            with open(rp, encoding='utf-8') as f:
                return {'round_doc': json.load(f)}
    return None


def score(obs, grid, yaw_tol=YAW_TOL_DEG, pos_tol=POS_TOL_MM,
          cell_radius_frac=CELL_RADIUS_FRAC):
    grid = grid or obs.get('grid')
    if not grid:
        raise ValueError('缺 grid.json（--grid 或 observations.grid）')
    if not grid.get('cells'):
        raise ValueError('grid.cells 为空')
    poses = obs.get('poses') or []
    if not poses:
        raise ValueError('没有终态观测 poses')
    merged = merge_poses(poses)
    exps = expected_rows(obs, grid)
    if not exps:
        raise ValueError('没有 expected（也没有可推的 round.json cycles）')
    rows = []
    for exp in exps:
        bid = exp.get('block_id')
        rec = merged.get(bid)
        if rec is None:
            rec = {'block_id': bid, 'center_pad_mm': None, 'yaw_deg': None,
                   'n_poses': 0, 'consistent': True, 'class': None,
                   'class_status': None}
        rows.append(score_cell(rec, exp, grid, yaw_tol=yaw_tol,
                               pos_tol=pos_tol, cell_radius_frac=cell_radius_frac))
    rows.sort(key=lambda r: (str(r['arm']), str(r['block_id'])))
    res = {'round': obs.get('round'), 'thresholds': {'yaw_deg': yaw_tol,
                                                     'pos_mm': pos_tol},
           'n_poses': len(poses), 'rows': rows,
           'summary': summarize(rows, obs.get('arms'))}
    return res


def main():
    ap = argparse.ArgumentParser(description='阶段 2 出口判分器')
    ap.add_argument('--run-dir', default=None)
    ap.add_argument('--observations', default=None)
    ap.add_argument('--grid', default=None)
    ap.add_argument('--out-report', default=None, help='前缀；写 .json/.md/.png')
    ap.add_argument('--threshold-yaw-deg', type=float, default=YAW_TOL_DEG)
    ap.add_argument('--threshold-pos-mm', type=float, default=POS_TOL_MM)
    ap.add_argument('--cell-radius-frac', type=float, default=CELL_RADIUS_FRAC)
    a = ap.parse_args()

    try:
        obs = load_observations(a.run_dir, a.observations)
    except (OSError, json.JSONDecodeError) as e:
        print('读观测失败：%s: %s' % (type(e).__name__, e), file=sys.stderr)
        return 2
    if obs is None:
        print('缺观测：给 --observations 或 run-dir 下的 score-input.json/round.json',
              file=sys.stderr)
        return 2
    grid = None
    if a.grid:
        try:
            with open(a.grid, encoding='utf-8') as f:
                grid = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            print('读 grid 失败：%s: %s' % (type(e).__name__, e), file=sys.stderr)
            return 2
    try:
        res = score(obs, grid, yaw_tol=a.threshold_yaw_deg,
                    pos_tol=a.threshold_pos_mm,
                    cell_radius_frac=a.cell_radius_frac)
    except (ValueError, KeyError, TypeError) as e:
        print('判分失败：%s: %s' % (type(e).__name__, e), file=sys.stderr)
        return 2

    prefix = a.out_report
    if prefix:
        base = os.path.abspath(prefix)
        os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
        with open(base + '.json', 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        with open(base + '.md', 'w', encoding='utf-8') as f:
            f.write(_md(res, a.threshold_yaw_deg, a.threshold_pos_mm))
        g = dict((grid or obs.get('grid')))
        g['round'] = obs.get('round', '?')
        g['_yaw_tol'] = a.threshold_yaw_deg
        g['_pos_tol'] = a.threshold_pos_mm
        try:
            draw_score_png(g, res['rows'], base + '.png')
        except (ValueError, KeyError) as e:                       # noqa: BLE001
            print('判分图跳过：%s' % e, file=sys.stderr)
        print('报告：%s.{json,md,png}' % base)

    o = res['summary']['overall']
    print('预期 %d，机判 %d，通过 %d，待复核 %d' % (
        o['n_expected'], o['n_machine'], o['n_ok'], o['n_person_review']))
    for arm, s in res['summary']['per_arm'].items():
        print('  %-10s 通过 %d/%d  成功率 %s' % (
            arm, s['n_ok'], s['n_machine'],
            '-' if s['success_rate'] is None else '%.0f%%' % (
                s['success_rate'] * 100)))

    all_ok = (o['n_machine'] == o['n_expected'] and
              o['n_ok'] == o['n_expected'])
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
