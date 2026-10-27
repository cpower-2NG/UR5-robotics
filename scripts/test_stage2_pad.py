#!/usr/bin/env python3
"""stage2_pad 的自检：合成 survey 视图 → 两页检测 → 垫系 → grid.json。

钉住的事
────────
  · 拼接**正确**的垫子：两页各 10 个标记都检出，解出的垫位姿与渲染单应一致
    （位置 <1.5 mm、转角 <0.3°），格心/垫心/yaw 落进 grid.json；
  · 页 B 被**挪了 2 mm**：``T_A_B`` 必须量出来（±0.5 mm），而不是假装对齐；
  · 页 B 不可见：fail-closed（ok=False，不出垫位姿），不许拿页 A 硬解；
  · 一个标记角点错 5 mm：RMS 判红；
  · CLI 退出码 0 / 2，grid.json 真写出来。

复用 ``test_stage2_mat`` 的模拟相机（同一套单应与腕部内参，不另造一份）。

跑法：
    python3 scripts/test_stage2_pad.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                        # noqa: E402
import numpy as np                                                # noqa: E402

import stage2_aruco as sa                                         # noqa: E402
import stage2_mat as sm                                           # noqa: E402
import stage2_pad as sp                                           # noqa: E402
import test_stage2_mat as tm                                      # noqa: E402

CASES = []
TMP = tempfile.mkdtemp(prefix='stage2-pad-test-')
IDENTITY = np.eye(3)
PPM = 6.0          # 整垫自绘的像素密度（px/mm）


def case(fn):
    CASES.append(fn)
    return fn


def _meta():
    return sm.build_meta(sm.DEFAULT_ORDER, 60.0)


def _compose_mat(meta, ppm=PPM, shift_b_mm=(0.0, 0.0)):
    """按垫系自绘整垫（白底黑标记）；页 B 可整体平移 ``shift_b_mm``。"""
    w = int(round(meta['pad']['mat_w_mm'] * ppm))
    h = int(round(meta['pad']['mat_h_mm'] * ppm))
    canvas = np.full((h, w, 3), 255, dtype=np.uint8)
    cx = meta['grid']['center_page_mm'][0]
    cy = meta['grid']['center_page_mm'][1]
    for key in ('a', 'b'):
        dx, dy = shift_b_mm if key == 'b' else (0.0, 0.0)
        for m in meta['pages'][key]['markers']:
            side = int(round(m['size_mm'] * ppm))
            art = sa.draw_marker(meta['pages'][key]['dictionary'], m['id'],
                                 side)
            art = cv2.cvtColor(art, cv2.COLOR_GRAY2BGR)
            x_pad, y_pad = m['corners_pad_mm'][0]        # TL：pad y 最高
            u0 = int(round((x_pad + dx + cx) * ppm))
            v0 = int(round((cy - (y_pad + dy)) * ppm))
            canvas[v0:v0 + side, u0:u0 + side] = art
    return canvas


def _view(meta, canvas):
    """整垫图 → 1280×720 合成相机画面（与 test_stage2_mat 同一单应）。"""
    H_pad = tm._pad_to_image_homography()
    ppm_x = canvas.shape[1] / float(meta['pad']['mat_w_mm'])
    ppm_y = canvas.shape[0] / float(meta['pad']['mat_h_mm'])
    center = meta['grid']['center_page_mm']
    T_src_pad = np.array([[1.0 / ppm_x, 0.0, -center[0]],
                          [0.0, -1.0 / ppm_y, center[1]],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
    out = cv2.warpPerspective(canvas, H_pad @ T_src_pad, (1280, 720),
                              flags=cv2.INTER_LINEAR,
                              borderValue=(255, 255, 255))
    return out, H_pad


def _gt_pose(H, K):
    """由渲染单应还原 pad→camera 的 4×4（**米**；H 本身是 mm→px）。"""
    M = np.linalg.inv(K) @ H
    r1 = M[:, 0] / np.linalg.norm(M[:, 0])
    r2 = M[:, 1] / np.linalg.norm(M[:, 1])
    t = M[:, 2] / np.linalg.norm(M[:, 0]) / 1000.0
    T = np.eye(4)
    T[:3, :3] = np.column_stack([r1, r2, np.cross(r1, r2)])
    T[:3, 3] = t
    return T


def _solve(view_bgr, meta, **kw):
    rgb = cv2.cvtColor(view_bgr, cv2.COLOR_BGR2RGB)
    return sp.solve_pad(rgb, tm._camera_matrix(), IDENTITY, np.zeros(3), meta,
                        **kw)


def _rot_err_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


# ---------------------------------------------------------------- 正例
@case
def test_solve_synthetic_matches_render_pose():
    meta = _meta()
    view, H = _view(meta, _compose_mat(meta))
    cv2.imwrite(os.path.join(TMP, 'view-ok.png'), view)
    res = _solve(view, meta)
    assert res['ok'], res['checks'].get('reason')
    c = res['checks']
    assert c['page_a']['n_markers'] == 10, c['page_a']
    assert c['page_b']['n_markers'] == 10, c['page_b']
    assert c['cells_in_view'], c['cell_centers_px']

    gt = _gt_pose(H, tm._camera_matrix())
    got = res['T_base_pad']
    d = np.linalg.norm(got[:3, 3] - gt[:3, 3]) * 1000.0
    assert d < 1.5, '垫位姿平移差 %.2f mm' % d
    e = _rot_err_deg(gt[:3, :3], got[:3, :3])
    assert e < 0.3, '垫位姿转角差 %.3f°' % e
    assert c['rms_adopted_px'] < 1.0, c['rms_adopted_px']

    grid = sp.build_grid(meta, got, c, T_A_B=res['T_A_B'])
    assert len(grid['cells']) == 9
    assert [round(v, 6) for v in grid['cells'][0]['center_pad_mm']] == \
        [-60.0, 60.0]
    # 格心 base 位置必须与渲染单应一致（垫子在这个视图里转了 6°，不能只加平移）
    for cell in grid['cells']:
        want = sp._apply(gt, [[cell['center_pad_mm'][0] / 1000.0,
                               cell['center_pad_mm'][1] / 1000.0]])[0] * 1000.0
        err = np.hypot(cell['center_base_mm'][0] - want[0],
                       cell['center_base_mm'][1] - want[1])
        assert err < 1.0, '格 %d base 位置与渲染单应差 %.3f mm' % (
            cell['idx'], err)
    assert abs(grid['grid_yaw_base_deg'] - 6.0) < 0.5, grid['grid_yaw_base_deg']
    assert grid['pitch_mm'] == 60.0


@case
def test_page_b_shift_is_measured():
    """页 B 挪 2 mm → T_A_B 必须量出 ~2 mm（不是假设对齐）。"""
    meta = _meta()
    view, _ = _view(meta, _compose_mat(meta, shift_b_mm=(2.0, -1.0)))
    res = _solve(view, meta)
    assert res['ok'], res['checks'].get('reason')
    d = res['checks']['T_A_B_delta']
    assert abs(d['translation_mm'][0] - 2.0) < 0.5, d
    assert abs(d['translation_mm'][1] - (-1.0)) < 0.5, d
    assert abs(d['translation_mm'][2]) < 0.5, d


# ---------------------------------------------------------------- 判红
@case
def test_missing_page_b_is_fail_closed():
    meta = _meta()
    canvas = _compose_mat(meta)
    # 把页 B 区域涂白 = 相机只看得见页 A。整垫图像素 u = 页面 x × ppm。
    xb = int(round(meta['pad']['page_b_origin_page_mm'][0] * PPM))
    canvas[:, xb:] = 255
    view, _ = _view(meta, canvas)
    res = _solve(view, meta)
    assert not res['ok']
    assert res['T_base_pad'] is None, '页 B 都没看见，却解出了垫位姿'
    assert '两页' in res['checks']['reason'], res['checks']['reason']


@case
def test_bad_corner_is_red():
    """一个标记的四个角各错 5 mm → 重投影 RMS 必须越限。"""
    meta = _meta()
    view, _ = _view(meta, _compose_mat(meta))
    bad = json.loads(json.dumps(meta))
    m = bad['pages']['a']['markers'][0]
    m['corners_pad_mm'] = [[c[0] + 5.0, c[1] + 5.0] for c in m['corners_pad_mm']]
    res = _solve(view, bad)
    assert not res['ok'], '角点错 5 mm 竟然判过：%s' % res['checks']
    assert res['T_base_pad'] is None


@case
def test_rms_limit_is_not_vacuous():
    """把 max_rms 压到 1e-4 → 同一条合格视图必须判红（阈值真的在参与）。"""
    meta = _meta()
    view, _ = _view(meta, _compose_mat(meta))
    ok = _solve(view, meta)
    assert ok['ok']
    res = _solve(view, meta, max_rms_px=1e-4)
    assert not res['ok'], '阈值压到 1e-4 还能过 —— RMS 判据是摆设'


# ---------------------------------------------------------------- CLI
def _write_frame(root, view):
    os.makedirs(root, exist_ok=True)
    rgb = cv2.cvtColor(view, cv2.COLOR_BGR2RGB)
    cv2.imwrite(os.path.join(root, 'color.png'), rgb)
    # load_frame 无条件读 depth.png；全 0 = 无有效深度（本测试只用彩色）。
    cv2.imwrite(os.path.join(root, 'depth.png'),
                np.zeros(view.shape[:2], dtype=np.uint16))
    K = tm._camera_matrix()
    q = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    tr = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    tf = {'ok': {'base_link:camera_color_optical_frame':
                 {'rotation': q, 'translation': tr}}}
    json.dump(tf, open(os.path.join(root, 'tf.json'), 'w'))
    json.dump({'camera_info': {'K': K.reshape(-1).tolist(), 'D': [0.0] * 5,
                               'width': 1280, 'height': 720}},
              open(os.path.join(root, 'meta.json'), 'w'))


def _run(args):
    return subprocess.run([sys.executable,
                           os.path.join(HERE, 'stage2_pad.py')] + args,
                          capture_output=True, text=True)


@case
def test_cli_writes_grid_and_exit_codes():
    meta = _meta()
    mat = os.path.join(TMP, 'mat.json')
    json.dump(meta, open(mat, 'w'))
    view, _ = _view(meta, _compose_mat(meta))
    frame = os.path.join(TMP, 'frame-ok')
    _write_frame(frame, view)

    p = _run(['--frame', frame, '--mat', mat])
    assert p.returncode == 0, '应退出 0：%d\n%s\n%s' % (p.returncode,
                                                        p.stdout, p.stderr)
    assert 'Traceback' not in p.stderr, p.stderr
    out = os.path.join(frame, 'grid.json')
    assert os.path.exists(out), p.stdout
    grid = json.load(open(out))
    assert grid['kind'] == 'stage2-grid'
    for key in ('cells', 'T_base_pad', 'grid_center_base_mm',
                'grid_yaw_base_deg', 'grid_order', 'checks'):
        assert key in grid, key
    assert grid['cells'][4]['center_pad_mm'] == [0.0, 0.0]
    assert grid['cells'][4]['color'] == 'yellowgreen'

    p2 = _run(['--frame', frame, '--mat', os.path.join(TMP, 'nope.json')])
    assert p2.returncode == 2, '缺 sidecar 应退出 2：%d' % p2.returncode
    assert 'Traceback' not in p2.stderr, p2.stderr


# ---------------------------------------------------------------- runner
def main():
    npass = nfail = nskip = 0
    for fn in CASES:
        try:
            fn()
        except AssertionError as e:
            print('FAIL %s: %s' % (fn.__name__, e))
            nfail += 1
        except Exception as e:                                   # noqa: BLE001
            print('ERROR %s: %s: %s' % (fn.__name__, type(e).__name__, e))
            nfail += 1
        else:
            print('PASS %s' % fn.__name__)
            npass += 1
    print('\n%d PASS, %d FAIL, %d SKIP' % (npass, nfail, nskip))
    shutil.rmtree(TMP, ignore_errors=True)
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
