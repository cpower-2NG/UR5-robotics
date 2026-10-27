#!/usr/bin/env python3
"""stage2_mat 的自检：版面几何、PDF 页尺寸、标记检测、PnP、垫系、变异测试。

跑法：
    python3 scripts/test_stage2_mat.py

为什么有「模拟相机」这一节
──────────────────────────
打印前只检查 PDF 页面尺寸，**不能**证明标记能被腕部相机看见。这里按
0.34 m、腕部内参（fx 908.15 / fy 908.38 / cx 639.44 / cy 363.31，1280×720）
生成一个单应，把拼贴后的垫子图（392×210 mm）投到相机画面里，再跑真正的
ArUco 检测与 PnP。只有这一步过了，才能说「两页标记同时可见、垫系解得出来」。

变异测试
────────
一个永远返回 True 的校验函数也会说「过了」。所以这里故意：
  · 把一个标记 ID 改成重复值 → ``validate_meta`` 必须报错；
  · 把一个角点挪 1 mm → 必须报错；
  · 把第 2 页移出画面 → 「两页都可见」必须失败。
如果哪天有人把校验删了，这几项会立刻变红。
"""

import copy
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import stage2_aruco as sa                                       # noqa: E402
import stage2_mat as sm                                          # noqa: E402

CASES = []
TMP = tempfile.mkdtemp(prefix='stage2-mat-test-')


def case(fn):
    CASES.append(fn)
    return fn


def _meta():
    return sm.build_meta(sm.DEFAULT_ORDER, 60.0)


def _camera_matrix():
    return np.array([[908.15, 0.0, 639.44],
                     [0.0, 908.38, 363.31],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _pad_to_image_homography(height_mm=340.0, yaw_deg=6.0, tilt_deg=2.0):
    """垫系（mm，z=0）→ 像素的单应；相机在垫原点上方 height_mm。"""
    K = _camera_matrix()
    # 相机轴相对世界的基准：x 右、y 下、z 朝下（即世界 −z）
    R0 = np.diag([1.0, -1.0, -1.0])
    R_cw = _rot_x(np.radians(tilt_deg)) @ _rot_z(np.radians(yaw_deg)) @ R0
    R = R_cw.T
    C = np.array([0.0, 0.0, height_mm], dtype=np.float64)
    t = -R @ C
    return K @ np.column_stack([R[:, 0], R[:, 1], t])


def _render_view(meta, mat_png, height_mm=340.0, yaw_deg=6.0, tilt_deg=2.0,
                 size=(1280, 720)):
    img = cv2.imread(mat_png, cv2.IMREAD_COLOR)
    if img is None:
        raise AssertionError('读不到垫子 PNG：%s' % mat_png)
    H_pad = _pad_to_image_homography(height_mm, yaw_deg, tilt_deg)
    center = meta['grid']['center_page_mm']
    sx = img.shape[1] / float(meta['pad']['mat_w_mm'])
    sy = img.shape[0] / float(meta['pad']['mat_h_mm'])
    # 源图像素 → 垫系 mm：mat_x = u/sx, mat_y = v/sy;
    # pad_x = mat_x − center_x, pad_y = center_y − mat_y
    T_src_pad = np.array([[1.0 / sx, 0.0, -center[0]],
                          [0.0, -1.0 / sy, center[1]],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
    H_src = H_pad @ T_src_pad
    return cv2.warpPerspective(img, H_src, size, flags=cv2.INTER_LINEAR,
                               borderValue=(255, 255, 255)), H_pad


def _detect_both(view):
    gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY)
    out = {}
    for name, key in ((sm.DICT_A, 'a'), (sm.DICT_B, 'b')):
        corners, ids = sa.detect_markers(gray, name)
        out[key] = ({int(i): c for c, i in zip(corners, ids)}
                    if ids is not None else {})
    return out


def _solve(meta, det):
    obj, img = [], []
    for key in ('a', 'b'):
        for m in meta['pages'][key]['markers']:
            c = det[key].get(m['id'])
            if c is None:
                continue
            obj.extend(m['corners_pad_mm'])
            img.extend(c.tolist())
    if len(obj) < 8:
        raise AssertionError('可用的标记角点太少：%d' % (len(obj) // 4))
    return sa.solve_plane_pnp(obj, img, _camera_matrix())


# ---------------------------------------------------------------- 几何
@case
def test_meta_geometry():
    meta = _meta()
    assert sm.validate_meta(meta)
    assert meta['grid']['size_mm'] == 180.0
    assert meta['grid']['x0_mm'] == 90.0
    assert meta['grid']['y0_mm'] == 15.0
    assert meta['grid']['center_page_mm'] == [180.0, 105.0]
    assert meta['grid_order'] == sm.DEFAULT_ORDER
    cells = meta['grid']['cells']
    assert [c['color'] for c in cells] == sm.DEFAULT_ORDER
    assert cells[0]['center_pad_mm'] == [-60.0, 60.0], cells[0]
    assert cells[4]['center_pad_mm'] == [0.0, 0.0], cells[4]
    assert cells[8]['center_pad_mm'] == [60.0, -60.0], cells[8]
    assert cells[7]['color'] == 'spare' and cells[8]['color'] == 'spare'

    for key, dict_name in (('a', sm.DICT_A), ('b', sm.DICT_B)):
        page = meta['pages'][key]
        assert page['dictionary'] == dict_name
        ids = sorted(m['id'] for m in page['markers'])
        assert ids == list(range(10)), (key, ids)
        for m in page['markers']:
            assert len(m['corners_page_mm']) == 4
            assert len(m['corners_pad_mm']) == 4
            assert abs(m['size_mm'] - 28.0) < 1e-9

    # 标记场：第 1 页坐标 x 10..364、y 23..187；宽 354、高 164
    f = meta['marker_field_page_mm']
    assert abs((f[1][0] - f[0][0]) - 354.0) < 1e-9, f
    assert abs((f[1][1] - f[0][1]) - 164.0) < 1e-9, f
    assert abs(f[0][0] - 10.0) < 1e-9 and abs(f[1][0] - 364.0) < 1e-9
    assert abs(f[0][1] - 23.0) < 1e-9 and abs(f[1][1] - 187.0) < 1e-9
    assert meta['pad']['mat_w_mm'] == 392.0
    assert meta['pad']['overlap_mm'] == 15.0
    assert meta['pad']['trim_local_mm'] == 110.0
    # 第 2 页第一个标记（局部 x=20）在垫系 x=20+282−180=122
    assert abs(meta['pages']['b']['markers'][0]['centroid_pad_mm'][0] - 122.0 - 14.0) < 1e-6


@case
def test_pitch_55():
    meta = sm.build_meta(sm.DEFAULT_ORDER, 55.0)
    assert sm.validate_meta(meta)
    assert meta['grid']['size_mm'] == 165.0
    assert meta['grid']['y0_mm'] == (210.0 - 165.0) / 2.0
    assert abs(meta['grid']['center_page_mm'][0] - 172.5) < 1e-9
    assert meta['grid']['cells'][0]['center_page_mm'] == [90.0 + 27.5,
                                                          meta['grid']['y0_mm'] + 27.5]


@case
def test_order_validation():
    bad = [
        'red,orange,tan,yellow,yellowgreen,cyan,purple,spare',       # 8
        'red,red,tan,yellow,yellowgreen,cyan,purple,spare,spare',    # 重复
        'red,orange,tan,yellow,yellowgreen,cyan,blue,spare,spare',   # 未知
        'red,orange,tan,yellow,yellowgreen,cyan,purple,spare,spare,spare',
    ]
    for spec in bad:
        try:
            sm.parse_order(spec)
        except ValueError:
            pass
        else:
            raise AssertionError('非法顺序被放行：%r' % spec)
    assert sm.parse_order(None) == sm.DEFAULT_ORDER


@case
def test_meta_mutations():
    base = _meta()
    muts = []

    m = copy.deepcopy(base)
    m['pages']['a']['markers'][0]['id'] = m['pages']['a']['markers'][1]['id']
    muts.append(('重复标记 ID', m))

    m = copy.deepcopy(base)
    m['pages']['a']['markers'][0]['corners_page_mm'][0][0] += 1.0
    muts.append(('角点挪 1 mm', m))

    m = copy.deepcopy(base)
    m['grid']['cells'][0]['color'] = 'cyan'
    muts.append(('格色与顺序不符', m))

    m = copy.deepcopy(base)
    m['pad']['page_b_origin_page_mm'][0] += 1.0
    muts.append(('第 2 页偏移改了', m))

    m = copy.deepcopy(base)
    m['grid']['center_page_mm'][0] += 1.0
    muts.append(('宫格中心改了', m))

    for name, m in muts:
        try:
            sm.validate_meta(m)
        except ValueError:
            pass
        else:
            raise AssertionError('%s 被 validate_meta 放行' % name)


# ---------------------------------------------------------------- 输出
@case
def test_pdf_and_png_outputs():
    meta = sm.build_meta(sm.DEFAULT_ORDER, 60.0)
    meta = sm.add_checksum(meta)
    pdf = os.path.join(TMP, 'mat.pdf')
    png = os.path.join(TMP, 'mat.png')
    sm.write_pdf(meta, pdf, dpi=150)
    sm.write_mat_png(meta, png, dpi=200)
    assert os.path.getsize(pdf) > 1000
    assert cv2.imread(png) is not None

    # PDF 两页、每页 A4 横向（MediaBox 已由 write_pdf 自检；这里再数一遍）
    with open(pdf, 'rb') as fh:
        data = fh.read()
    assert data.count(b'/MediaBox') >= 2
    try:
        import pymupdf as fitz
    except ImportError:
        try:
            import fitz
        except ImportError:
            fitz = None
    if fitz is not None:
        doc = fitz.open(pdf)
        assert doc.page_count == 2
        for page in doc:
            assert abs(page.rect.width / 72.0 * 25.4 - 297.0) < 0.2
            assert abs(page.rect.height / 72.0 * 25.4 - 210.0) < 0.2
        doc.close()

    # 100 mm 校验尺：在 PNG 上量黑色连续段（两条，取长的）
    img = cv2.imread(png)
    sx = img.shape[1] / float(meta['pad']['mat_w_mm'])
    row = img[int(203.0 * img.shape[0] / 210.0), :, :].mean(axis=1)
    dark = row < 50
    runs = []
    i = 0
    while i < len(dark):
        if dark[i]:
            j = i
            while j < len(dark) and dark[j]:
                j += 1
            runs.append((j - i) / sx)
            i = j
        else:
            i += 1
    assert runs, '校验尺没找到'
    best = max(runs)
    assert abs(best - 100.0) < 0.6, '校验尺量出 %.2f mm（应为 100）' % best


@case
def test_checksum():
    meta = sm.add_checksum(_meta())
    js = json.loads(json.dumps(meta))
    got = js.pop('checksum_sha256')
    want = sm.hashlib.sha256(sm._canonical_json(js).encode('utf-8')).hexdigest()
    assert got == want
    js['grid_order'][0] = 'cyan'
    assert sm.hashlib.sha256(sm._canonical_json(js).encode('utf-8')).hexdigest() != got


# ---------------------------------------------------------------- 模拟相机
@case
def test_synthetic_camera_and_pnp():
    meta = _meta()
    png = os.path.join(TMP, 'mat-synth.png')
    sm.write_mat_png(meta, png, dpi=300)
    view, H_pad = _render_view(meta, png)
    cv2.imwrite(os.path.join(TMP, 'synthetic-view.png'), view)

    det = _detect_both(view)
    for key in ('a', 'b'):
        ids = sorted(det[key])
        assert ids == list(range(10)), \
            '%s 页只检出 %r —— 两页标记没有同时可见' % (key, ids)

    # 28 mm 标记在 0.34 m 处约 75 px
    sides = [float(np.linalg.norm(c[(j + 1) % 4] - c[j]))
             for cs in det.values() for c in cs.values() for j in range(4)]
    assert sides and 65.0 < float(np.mean(sides)) < 85.0, np.mean(sides)

    rvec, tvec, rmse = _solve(meta, det)
    assert rmse < 1.5, 'PnP 重投影 RMS %.2f px 太大' % rmse

    pts = np.array([[c['center_pad_mm'][0], c['center_pad_mm'][1], 0.0]
                    for c in meta['grid']['cells']], dtype=np.float64)
    got = sa.project_points(pts, rvec, tvec, _camera_matrix())
    gt = H_pad @ np.array([[p[0], p[1], 1.0] for p in pts], dtype=np.float64).T
    gt = (gt[:2] / gt[2]).T
    err = np.linalg.norm(got - gt, axis=1)
    assert float(err.max()) < 1.5, '格心最大误差 %.2f px' % float(err.max())

    # 两个页面各 10 个标记都应落在画面内
    for key in ('a', 'b'):
        for c in det[key].values():
            assert np.all(c[:, 0] > 0) and np.all(c[:, 0] < view.shape[1])
            assert np.all(c[:, 1] > 0) and np.all(c[:, 1] < view.shape[0])


@case
def test_synthetic_camera_mutation_page_b_out_of_view():
    """把第 2 页裁出画面 → 「两页都可见」必须失败（不能自证）。"""
    meta = _meta()
    png = os.path.join(TMP, 'mat-synth-mut.png')
    sm.write_mat_png(meta, png, dpi=200)
    view, _ = _render_view(meta, png, height_mm=340.0, yaw_deg=6.0, tilt_deg=2.0)
    det = _detect_both(view)
    assert sorted(det['a']) == list(range(10))
    assert sorted(det['b']) == list(range(10))
    # 第 1 页标记在画面左约 1/4、第 2 页在右约 4/5；把右 3/8 涂白，
    # 等价于「相机视野只罩住第 1 页」。
    cut = int(view.shape[1] * 0.625)
    view2 = view.copy()
    view2[:, cut:] = (255, 255, 255)
    cv2.imwrite(os.path.join(TMP, 'synthetic-view-page-b-cut.png'), view2)
    det2 = _detect_both(view2)
    assert sorted(det2['a']) == list(range(10)), '裁切不应伤到第 1 页'
    assert len(det2['b']) < 10, '第 2 页被裁出画面却仍检出 %d 个' % len(det2['b'])


@case
def test_pdf_render_detection():
    """从**PDF 本身**栅格化再拼贴，确认打印件（不是 PNG）也能被检出。

    PNG 自检只能证明绘图函数对；PDF 里标记是位图嵌入，若编码/缩放出问题，
    只有渲染 PDF 才看得见。没有 PyMuPDF 时明确 SKIP，不假装通过。
    """
    try:
        import pymupdf as fitz
    except ImportError:
        try:
            import fitz
        except ImportError:
            print('  SKIP test_pdf_render_detection: 没有 PyMuPDF')
            return

    meta = _meta()
    pdf = os.path.join(TMP, 'mat-pdfrender.pdf')
    sm.write_pdf(meta, pdf, dpi=150)
    doc = fitz.open(pdf)
    dpi = 300
    ppm = dpi / 25.4
    page_a = doc[0].get_pixmap(dpi=dpi)
    page_b = doc[1].get_pixmap(dpi=dpi)
    doc.close()
    a = np.frombuffer(page_a.samples, dtype=np.uint8).reshape(
        page_a.height, page_a.width, page_a.n)
    b = np.frombuffer(page_b.samples, dtype=np.uint8).reshape(
        page_b.height, page_b.width, page_b.n)
    a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    b = cv2.cvtColor(b, cv2.COLOR_RGB2BGR)

    W = int(round(meta['pad']['mat_w_mm'] * ppm))
    H = int(round(meta['pad']['mat_h_mm'] * ppm))
    canvas = np.full((H, W, 3), 255, dtype=np.uint8)
    xb = int(round(meta['pad']['page_b_origin_page_mm'][0] * ppm))
    for img, x0 in ((b, xb), (a, 0)):
        h = min(img.shape[0], H)
        w = min(img.shape[1], W - x0)
        canvas[:h, x0:x0 + w] = img[:h, :w]
    out = os.path.join(TMP, 'mat-from-pdf.png')
    cv2.imwrite(out, canvas)

    view, _ = _render_view(meta, out)
    det = _detect_both(view)
    for key in ('a', 'b'):
        assert sorted(det[key]) == list(range(10)), \
            'PDF 渲染件上 %s 页只检出 %r' % (key, sorted(det[key]))
    _, _, rmse = _solve(meta, det)
    assert rmse < 1.5, 'PDF 渲染件的 PnP RMS %.2f px' % rmse


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
    print('临时产物：%s' % TMP)
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
