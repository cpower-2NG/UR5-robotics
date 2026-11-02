#!/usr/bin/env python3
"""find_blocks 的回归：合簇静默漏块必须修好，平板不能被当块。

为什么有「存档帧」和「合成帧」两节
────────────────────────────────
· 存档帧（A3 的 pick-02/05/06）是**真实缺陷现场**：旧算法在低处把相邻块桥接成
  31604 / 57537 / 46263 点的大簇，随后 ``len(Qs) > 20000`` 整簇丢弃。
  这些帧在仓外 ``runs/2026-10-25-precision/frames/``，没有就 SKIP —— 不假装通过。
· 合成帧不依赖外部数据，钉住「高度切顶面」的语义：30 mm 块算块，8 mm 平板
  只能进 ``low``；把阈值改回 5 mm 就必须变红（变异测试）。

跑法：
    python3 scripts/test_graspnet_blocks.py
"""

import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                               # noqa: E402

import graspnet_candidates as gc                                 # noqa: E402

CASES = []
RUN = os.path.expanduser(
    '~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/frames')
ZONE = (380.0, 620.0, 250.0, 450.0)      # A3 的拾取区 x0,x1,y0,y1（mm）


def case(fn):
    CASES.append(fn)
    return fn


def _in_zone(b):
    x0, x1, y0, y1 = ZONE
    return x0 <= b['cx'] <= x1 and y0 <= b['cy'] <= y1


def _near(b, x, y, tol=8.0):
    return float(np.hypot(b['cx'] - x, b['cy'] - y)) <= tol


def _archived(name):
    d = os.path.join(RUN, name)
    return d if os.path.isdir(d) else None


# ---------------------------------------------------------------- 存档帧
@case
def test_archived_counts_and_missed_blocks():
    need = [('pick-02', 9, 8, 7), ('pick-05', 8, 7, 5), ('pick-06', 7, 6, 5)]
    missing = [n for n, *_ in need if _archived(n) is None]
    if missing:
        print('  SKIP test_archived_counts_and_missed_blocks: 缺帧 %s' % missing)
        return

    old_zone = {}
    for name, full, zone, legacy_zone in need:
        f = gc.load_frame(_archived(name))
        blocks, _ = gc.find_blocks(f)
        high = [b for b in blocks if not b['low']]
        assert len(high) == full, '%s 全帧高块 %d（应为 %d）' % (
            name, len(high), full)
        assert len([b for b in high if _in_zone(b)]) == zone, \
            '%s 区内高块数不对' % name

        old, _ = gc.find_blocks(f, legacy=True)
        old_high = [b for b in old if b['h'] >= 15.0]
        old_zone[name] = len([b for b in old_high if _in_zone(b)])
        assert old_zone[name] == legacy_zone, \
            '%s 旧算法区内块 %d（应为 %d，对照基准变了要复核）' % (
                name, old_zone[name], legacy_zone)
        assert zone > old_zone[name], \
            '%s 新算法没有多找到块（新 %d / 旧 %d）' % (
                name, zone, old_zone[name])

    # 旧算法丢掉的两块，新算法必须报出来。
    f2 = gc.load_frame(_archived('pick-02'))
    b2 = [b for b in gc.find_blocks(f2)[0] if not b['low']]
    assert any(_near(b, 428.8, 256.5) for b in b2), \
        'pick-02 的 (428.8,256.5) 仍没报出'
    old2 = [b for b in gc.find_blocks(f2, legacy=True)[0] if b['h'] >= 15.0]
    assert not any(_near(b, 428.8, 256.5) for b in old2), \
        '旧算法居然报出了 (428.8,256.5)，对照基准要复核'

    f5 = gc.load_frame(_archived('pick-05'))
    b5 = [b for b in gc.find_blocks(f5)[0] if not b['low']]
    assert any(_near(b, 418.4, 329.1) for b in b5), \
        'pick-05 的青块 (418.4,329.1) 仍没报出'


@case
def test_archived_low_blocks_are_diagnostic_only():
    if _archived('pick-02') is None:
        print('  SKIP test_archived_low_blocks_are_diagnostic_only: 缺帧')
        return
    f = gc.load_frame(_archived('pick-02'))
    blocks, _ = gc.find_blocks(f)
    low = [b for b in blocks if b['low']]
    assert low, 'pick-02 的 8 mm 平板应留在 low（skipped_low）里'
    for b in low:
        assert b['h'] < 15.0, 'low 块高度 %.1f 不该 >=15' % b['h']
        assert b['class'] in ('unknown', 'orange', 'cyan'), b['class']
    high = [b for b in blocks if not b['low']]
    assert all(b['h'] >= 15.0 for b in high)


# ---------------------------------------------------------------- 合成帧
def _synth_frame():
    """台面 z=0、一块 30 mm、一块 8 mm 平板；相机朝下，R=diag(1,-1,-1)。"""
    h, w = 480, 640
    fx = fy = 500.0
    cx, cy = 320.0, 240.0
    depth = np.full((h, w), 0.500, dtype=np.float32)
    color = np.full((h, w, 3), 40, dtype=np.uint8)

    def put(u0, v0, half_u, half_v, d, rgb):
        depth[v0 - half_v:v0 + half_v, u0 - half_u:u0 + half_u] = d
        color[v0 - half_v:v0 + half_v, u0 - half_u:u0 + half_u] = rgb

    # 30 mm 块：物理 36×36 mm，顶面 z=0.47 m（相机系），投影像素约 38×38
    put(214, 240, 19, 19, 0.470, (220, 30, 30))
    # 8 mm 平板：物理 100×20 mm，顶面 z=0.492 m，投影约 102×20
    put(422, 240, 51, 10, 0.492, (120, 120, 120))
    R = np.diag([1.0, -1.0, -1.0])
    t = np.array([0.0, 0.0, 0.500])
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    return {'depth': depth, 'color': color, 'R_bc': R, 't_bc': t, 'K': K}


@case
def test_synthetic_height_split():
    frame = _synth_frame()
    blocks, tz = gc.find_blocks(frame)
    assert abs(tz) < 0.002, '合成台面 z 应≈0，实际 %.4f' % tz
    high = [b for b in blocks if not b['low']]
    low = [b for b in blocks if b['low']]
    assert len(high) == 1, '高块应只有 1 个，实际 %d：%r' % (
        len(high), [(round(b['cx']), round(b['cy']), round(b['h'])) for b in high])
    b = high[0]
    # 台面 z 用 2 mm 直方图众数，合成帧会有 ±2 mm 的量化；30 mm 判到 32 mm 属正常。
    assert abs(b['h'] - 30.0) < 3.5, '块高 %.1f mm（应≈30）' % b['h']
    assert abs(b['ext'][0] - 36.0) < 3.0 and abs(b['ext'][1] - 36.0) < 3.0, b['ext']
    assert b['class'] == 'red', b['class']
    assert len(low) == 1, '低块应只有 1 个平板，实际 %d' % len(low)
    assert abs(low[0]['h'] - 8.0) < 2.0, '平板高 %.1f mm（应≈8）' % low[0]['h']


@case
def test_synthetic_mutation_min_h_5mm():
    """把 min_h 改回 5 mm → 8 mm 平板会被当高块；这正是旧阈值的问题。"""
    frame = _synth_frame()
    blocks, _ = gc.find_blocks(frame, min_h_mm=5.0)
    high = [b for b in blocks if not b['low']]
    assert len(high) == 2, '5 mm 阈值下应把平板也算进来（2 个），实际 %d' % len(high)
    assert any(abs(b['h'] - 8.0) < 2.0 for b in high)


@case
def test_synthetic_flat_table_no_blocks():
    h, w = 240, 320
    frame = {
        'depth': np.full((h, w), 0.500, dtype=np.float32),
        'color': np.full((h, w, 3), 40, dtype=np.uint8),
        'R_bc': np.diag([1.0, -1.0, -1.0]),
        't_bc': np.array([0.0, 0.0, 0.500]),
        'K': np.array([[400.0, 0.0, 160.0], [0.0, 400.0, 120.0], [0.0, 0.0, 1.0]]),
    }
    blocks, tz = gc.find_blocks(frame)
    assert blocks == [], '平地不该报块：%r' % blocks


def _synth_frame_edge_artifact():
    """块 + 画幅底边的深度伪影，两者在俯视网格上连成一簇（-b 帧的复现）。

    台面 z=0，相机朝下、R=diag(1,-1,-1)、t=(0,0,0.5) ⇒ z_base = 0.5 − 深度。
    块顶 0.470 m → 30 mm；底边 8 行读 0.455 m → 45 mm（伪影方向：比真顶面**高**）。
    """
    h, w = 240, 320
    fx = fy = 400.0
    cx, cy = 160.0, 120.0
    depth = np.full((h, w), 0.500, dtype=np.float32)
    color = np.full((h, w, 3), 40, dtype=np.uint8)
    # 36×36 mm 块，投影像素约 30×30
    depth[194:224, 145:175] = 0.470
    color[194:224, 145:175] = (220, 30, 30)
    # 伪影：从块的下沿一路画到画幅最后一行。深度更小 ⇒ xy 更靠近光轴，
    # 于是尽管在图上是上下相接，俯视网格里仍与块连成同一簇（就是 -b 的形态）。
    depth[224:h, 145:175] = 0.455
    color[224:h, 145:175] = (220, 30, 30)
    return {'depth': depth, 'color': color,
            'R_bc': np.diag([1.0, -1.0, -1.0]),
            't_bc': np.array([0.0, 0.0, 0.500]),
            'K': np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])}


@case
def test_synthetic_edge_artifact_does_not_steal_the_top():
    """底边伪影点数越过簇的 ~3% 时，97 分位顶高会被拽到伪影上（frame-hold-b）。"""
    frame = _synth_frame_edge_artifact()
    blocks, _ = gc.find_blocks(frame)
    high = [b for b in blocks if not b['low']]
    assert len(high) == 1, '应只有 1 块，实际 %r' % [
        (round(b['cx']), round(b['cy']), round(b['h'], 1)) for b in high]
    b = high[0]
    assert abs(b['h'] - 30.0) < 3.5, (
        '顶高被底边伪影拽走了：%.1f mm（应≈30，伪影在 45）' % b['h'])


@case
def test_synthetic_edge_artifact_mutation_margin_off():
    """变异：把边距关掉 ⇒ 同一帧必须复现「顶高 ≈45 mm」的错误，否则这测试是空的。"""
    frame = _synth_frame_edge_artifact()
    old = gc.EDGE_MARGIN_PX
    gc.EDGE_MARGIN_PX = 0
    try:
        blocks, _ = gc.find_blocks(frame)
    finally:
        gc.EDGE_MARGIN_PX = old
    high = [b for b in blocks if not b['low']]
    assert len(high) == 1, high
    assert high[0]['h'] > 40.0, (
        '关掉边距后应复现伪影（>40 mm），实际 %.1f —— 合成帧没造出退化条件'
        % high[0]['h'])


# ---------------------------------------------------------------- 几何
def _rect_pts(w, h, deg, n=40, center=(0.0, 0.0)):
    """矩形边界上的点（米）；用于验最小外接矩形。"""
    a = np.radians(deg)
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    t = np.linspace(0.0, 1.0, n)
    pts = np.vstack([
        np.column_stack([np.linspace(-w/2, w/2, n), np.full(n, -h/2)]),
        np.column_stack([np.full(n, w/2), np.linspace(-h/2, h/2, n)]),
        np.column_stack([np.linspace(w/2, -w/2, n), np.full(n, h/2)]),
        np.column_stack([np.full(n, -w/2), np.linspace(h/2, -h/2, n)]),
    ])
    return (pts @ R.T) + np.array(center)


@case
def test_min_area_rect_rotated_square():
    pts = _rect_pts(0.040, 0.040, 30.0)
    r = gc.min_area_rect_2d(pts)
    assert abs(r['yaw_deg'] % 90.0 - 30.0) < 1.0, r
    assert abs(r['ext_mm'][0] - 40.0) < 1.0, r
    assert abs(r['ext_mm'][1] - 40.0) < 1.0, r
    assert r['fill'] > 0.95, r


@case
def test_min_area_rect_elongated_and_outlier():
    pts = _rect_pts(0.050, 0.020, 10.0)
    r = gc.min_area_rect_2d(pts)
    assert abs(r['ext_mm'][0] - 50.0) < 1.0, r
    assert abs(r['ext_mm'][1] - 20.0) < 1.0, r
    assert abs(r['yaw_deg'] % 90.0 - 10.0) < 1.0, r
    # 一个飞点不应把角度拉走（分位裁剪的作用）。
    bad = np.vstack([pts, [[0.060, 0.060]]])
    rb = gc.min_area_rect_2d(bad)
    assert abs(rb['yaw_deg'] % 90.0 - 10.0) < 1.5, rb


@case
def test_fold_and_diff_mod90():
    assert gc.fold_mod90(89.9) == 89.9
    assert abs(gc.fold_mod90(90.0)) < 1e-9
    assert abs(gc.fold_mod90(-1.0) - 89.0) < 1e-9
    assert abs(gc.ang_diff_mod90(1.0, 89.0) - 2.0) < 1e-9
    assert abs(gc.ang_diff_mod90(0.0, 45.0) - 45.0) < 1e-9
    assert abs(gc.ang_diff_mod90(10.0, 10.0)) < 1e-9


@case
def test_select_keeps_the_whole_near_list():
    """``select`` 除了 ``best`` 还要留下整串近邻 —— graspnet 臂按块边筛候选靠它。

    ``best``/``dist_mm``/``n_near`` 的语义**不变**（A3 与判分器都在用）。
    """
    blocks = [{"idx": 0, "class": "cyan", "cx": 500.0, "cy": 300.0}]
    cands = [{"i": 1, "score": 0.30, "angle_deg": 137.2,
              "xyz_base": [0.520, 0.300, 0.0]},      # 离块心 20 mm，对角
             {"i": 2, "score": 0.25, "angle_deg": 5.0,
              "xyz_base": [0.500, 0.301, 0.0]},      # 离块心 1 mm，贴边
             {"i": 3, "score": 0.28, "angle_deg": 5.5,
              "xyz_base": [0.500, 0.304, 0.0]}]      # 离块心 4 mm
    per = gc.select(blocks, cands, anchor_mm=25.0)
    p = per[0]
    assert p["n_near"] == 3 and [c["i"] for c in p["near"]] == [2, 3, 1], p
    assert p["best"]["i"] == 2 and abs(p["dist_mm"] - 1.0) < 1e-9
    assert all("dist_mm" in c for c in p["near"])
    # 锚定半径外的候选不进 near
    per = gc.select(blocks, cands, anchor_mm=5.0)
    assert [c["i"] for c in per[0]["near"]] == [2, 3], per[0]["near"]


# ---------------------------------------------------------------- runner
def main():
    npass = nfail = 0
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
    print('\n%d PASS, %d FAIL' % (npass, nfail))
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
