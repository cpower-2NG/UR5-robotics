#!/usr/bin/env python3
"""摆位图（A3 1:1）的离线自检：几何 / 比例 / 与布局表一致。

为什么值得单测：1:1 摆位图最坏的失败是**悄悄缩放** —— 图看起来一模一样，
人照着摆完整盘系统性偏，而且到判分时才暴露。所以这里把「映射是纯平移」
「比例尺正好 100 mm」「放不下就报错、不缩」三件事钉死。

跑法：python3 scripts/test_stage4_layout_sheet.py
"""
import math
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_layout_sheet as ls                                 # noqa: E402
import stage4_layouts as lay                                     # noqa: E402
import stage4_clear_model as cm                                  # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def section(title):
    print()
    print('== %s ==' % title)


def main():
    size = lay.BLOCK_SIZE_M * 1000.0

    # ------------------------------------------------------------------
    section('A. 块轮廓（真实大小 + 朝向）')
    c = ls.block_corners(100.0, 200.0, 0.0, size)
    check('A1 四个角', len(c) == 4, repr(c))
    check('A2 边长 = 布局边长', all(close(dist(c[i], c[(i + 1) % 4]), size)
                                     for i in range(4)),
          str([round(dist(c[i], c[(i + 1) % 4]), 6) for i in range(4)]))
    cx = sum(p[0] for p in c) / 4.0
    cy = sum(p[1] for p in c) / 4.0
    check('A3 形心 = 块心', close(cx, 100.0) and close(cy, 200.0),
          '(%.6f, %.6f)' % (cx, cy))
    check('A4 yaw=0 轴对齐',
          close(c[0][0], 100.0 - size / 2) and close(c[0][1], 200.0 - size / 2)
          and close(c[2][0], 100.0 + size / 2) and close(c[2][1], 200.0 + size / 2),
          repr(c))
    c90 = ls.block_corners(100.0, 200.0, 90.0, size)
    check('A5 yaw=90 转对了（角 0 → 右下）',
          close(c90[0][0], 100.0 + size / 2) and close(c90[0][1], 200.0 - size / 2),
          repr(c90[0]))
    c45 = ls.block_corners(0.0, 0.0, 45.0, size)
    check('A6 yaw=45 对角线成水平/竖直（±边长/√2）',
          close(sorted(p[1] for p in c45)[0], -size / math.sqrt(2))
          and close(sorted(p[1] for p in c45)[-1], size / math.sqrt(2)),
          repr([round(p[1], 6) for p in c45]))
    a, b = ls.block_diagonal(100.0, 200.0, 33.0, size)
    cc = ls.block_corners(100.0, 200.0, 33.0, size)
    check('A7 对角标记 = 角 0 → 角 2', a == cc[0] and b == cc[2])

    # ------------------------------------------------------------------
    section('B. 页面排版（1:1 + 放不下就报错）')
    zone = [v * 1000.0 for v in cm.PICK_ZONE]
    geom = ls.sheet_geometry(zone)
    check('B1 scale 恒为 1.0', geom['scale'] == 1.0, repr(geom['scale']))
    px0, py0, px1, py1 = geom['paper_zone_mm']
    check('B2 纸面拾取区 = 实际尺寸（宽）',
          close(px1 - px0, zone[1] - zone[0]), '%.6f' % (px1 - px0))
    check('B3 纸面拾取区 = 实际尺寸（高）',
          close(py1 - py0, zone[3] - zone[2]), '%.6f' % (py1 - py0))
    check('B4 在页内且留边',
          px0 >= ls.MARGIN_MM - 1e-9 and py0 >= ls.MARGIN_MM - 1e-9
          and px1 <= ls.PAGE_A3_MM[0] - ls.MARGIN_MM + 1e-9
          and py1 <= ls.PAGE_A3_MM[1] - ls.MARGIN_MM + 1e-9)
    ix, iy, iw, ih = geom['info_rect_mm']
    check('B5 说明栏不压图', ix >= px1 + ls.GUTTER_MM - 1e-9, '%.1f vs %.1f'
          % (ix, px1))
    check('B6 说明栏在页内',
          ix >= 0 and iy >= 0 and ix + iw <= ls.PAGE_A3_MM[0] + 1e-9
          and iy + ih <= ls.PAGE_A3_MM[1] + 1e-9)

    raised = False
    try:
        ls.sheet_geometry([0.0, 0.0, 330.0, 100.0])
    except ValueError:
        raised = True
    check('B7 拾取区比页面宽 ⇒ 报错（绝不缩图）', raised)
    raised = False
    try:
        ls.sheet_geometry([0.0, 0.0, 100.0, 300.0])
    except ValueError:
        raised = True
    check('B8 拾取区比页面高 ⇒ 报错', raised)
    raised = False
    try:
        ls.sheet_geometry([10.0, 10.0, 10.0, 20.0])
    except ValueError:
        raised = True
    check('B9 非正矩形 ⇒ 报错', raised)

    # ------------------------------------------------------------------
    section('C. 坐标映射：纯平移（1:1 的真正判据）')
    p0 = ls.paper_point(0.0, 0.0, geom)
    p1 = ls.paper_point(1.0, 0.0, geom)
    p2 = ls.paper_point(0.0, 1.0, geom)
    check('C1 x 方向 1 mm → 1 mm', close(p1[0] - p0[0], 1.0)
          and close(p1[1] - p0[1], 0.0), repr((p1[0] - p0[0], p1[1] - p0[1])))
    check('C2 y 方向 1 mm → 1 mm', close(p2[1] - p0[1], 1.0)
          and close(p2[0] - p0[0], 0.0), repr((p2[0] - p0[0], p2[1] - p0[1])))
    check('C3 paper_delta(100) = 100', close(ls.paper_delta(100.0, geom), 100.0))
    for layout in sorted(lay.BATCHES['S0']):
        doc = ls.sheet_doc('S0', layout)
        pts = [ls.paper_point(b['x_mm'], b['y_mm'], geom) for b in doc['blocks']]
        ok = True
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                d_base = math.hypot(doc['blocks'][i]['x_mm'] - doc['blocks'][j]['x_mm'],
                                    doc['blocks'][i]['y_mm'] - doc['blocks'][j]['y_mm'])
                if not close(dist(pts[i], pts[j]), d_base):
                    ok = False
        check('C4 S0:%d 图上两两距离 = base 距离' % layout, ok)
    sb0, sb1 = ls.scale_bar_points(geom)
    check('C5 比例尺正好 100 mm', close(dist(sb0, sb1), 100.0),
          '%.9f' % dist(sb0, sb1))

    # ------------------------------------------------------------------
    section('D. 与布局表逐块一致 + 自检能抓错')
    for layout in sorted(lay.BATCHES['S0']):
        doc = ls.sheet_doc('S0', layout)
        truth = lay.layout_blocks('S0', layout)
        ok = (len(doc['blocks']) == len(truth) == 5)
        for got, want in zip(sorted(doc['blocks'], key=lambda d: d['id']),
                             sorted(truth, key=lambda d: d['id'])):
            ok = ok and got['id'] == want['id'] and got['color'] == want['color']
            ok = ok and close(got['x_mm'], want['x'] * 1000.0)
            ok = ok and close(got['y_mm'], want['y'] * 1000.0)
            ok = ok and close(got['yaw_deg'], want['yaw'])
        check('D1 S0:%d 文档 = 布局表' % layout, ok)
        check('D2 S0:%d 自检通过（无问题）' % layout,
              ls.check_doc(doc, geom) == [], repr(ls.check_doc(doc, geom)))
    doc1 = ls.sheet_doc('S0', 1)
    check('D3 块边长与布局同源', close(doc1['block_size_mm'], size))
    check('D4 拾取区 = cm.PICK_ZONE（毫米）',
          doc1['zone_mm'] == [v * 1000.0 for v in cm.PICK_ZONE])
    bad = ls.sheet_doc('S0', 1)
    bad['blocks'][0]['x_mm'] = 240.0          # 挪出拾取区（x0=250）
    check('D5 变异：块挪出拾取区 ⇒ 自检报错', len(ls.check_doc(bad, geom)) > 0)
    bad2 = ls.sheet_doc('S0', 1)
    bad2['blocks'][0]['yaw_deg'] = 45.0       # 只改朝向：角可能出框，允许也能抓
    check('D6 文档块数固定 5（S0 是隔离基线）',
          all(len(ls.sheet_doc('S0', l)['blocks']) == 5
              for l in lay.BATCHES['S0']))

    # ------------------------------------------------------------------
    section('E. 说明栏文字')
    zh = ls.info_lines(doc1, zh=True)
    en = ls.info_lines(doc1, zh=False)
    import re
    room = re.compile(r'^\d\s')
    zh_rows = [l for l in zh if room.match(l)]
    en_rows = [l for l in en if room.match(l)]
    check('E1 中英各有一行一块', len(zh_rows) == 5 and len(en_rows) == 5,
          '%d / %d' % (len(zh_rows), len(en_rows)))
    check('E2 行里有坐标（1 位小数）',
          all(re.search(r'\d+\.\d', l) for l in zh_rows))
    check('E3 行里有颜色名',
          all(b['color'] in l for b, l in zip(sorted(doc1['blocks'],
                                                     key=lambda d: d['id']),
                                              zh_rows)))
    check('E4 S0 用到的颜色都有中文名',
          all(b['color'] in ls.COLOR_ZH
              for l in lay.BATCHES['S0'] for b in lay.layout_blocks('S0', l)))
    check('E5 拾取区坐标写进了说明栏',
          any('%.0f' % zone[0] in l and '%.0f' % zone[1] in l for l in zh))
    check('E6 有「收走图纸再摆块」这一条',
          any('收走图纸' in l for l in zh) and any('Remove the sheet' in l
                                                  for l in en))
    check('E7 pick_font 只有一份实现（= plot_table_envelope 的）',
          ls.pick_font is ls.pte.pick_font)
    check('E8 pick_font 按优先级挑', ls.pick_font(
        {'Noto Sans CJK SC', 'Microsoft YaHei'}) == 'Noto Sans CJK SC')
    check('E9 没有 CJK 字体 ⇒ None（整图退英文）',
          ls.pick_font({'Arial', 'DejaVu Sans'}) is None)

    # ------------------------------------------------------------------
    section('F. 出图（有 matplotlib 才测）')
    try:
        import matplotlib                                     # noqa: F401
        have_mpl = True
    except ImportError:
        have_mpl = False
        print('  SKIP  没有 matplotlib')
    if have_mpl:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(ls.PAGE_A3_MM[0] / 25.4,
                                  ls.PAGE_A3_MM[1] / 25.4))
        ax = ls.draw_page(fig, geom, doc1, None)
        # 拾取区框 1 + 说明栏框 1 + 每块（填充 1 + 描边 1）
        check('F1 图上有拾取区框 + 说明栏 + 5 块出两层的多边形',
              len(ax.patches) == 2 + 2 * 5, 'patches=%d' % len(ax.patches))
        check('F2 坐标范围就是 A3 毫米',
              ax.get_xlim() == (0.0, ls.PAGE_A3_MM[0])
              and ax.get_ylim() == (0.0, ls.PAGE_A3_MM[1]),
              repr((ax.get_xlim(), ax.get_ylim())))
        plt.close(fig)

        td = tempfile.mkdtemp(prefix='sheet-')
        png = os.path.join(td, 'one.png')
        rc = ls.draw([doc1], png, dpi=150)
        check('F3 单页 PNG 写出', rc == 0 and os.path.getsize(png) > 20000,
              'rc=%r size=%s' % (rc, os.path.getsize(png)
                                 if os.path.exists(png) else 'missing'))
        pdf = os.path.join(td, 'three.pdf')
        rc = ls.draw([ls.sheet_doc('S0', l) for l in (1, 2, 3)], pdf)
        blob = open(pdf, 'rb').read() if os.path.exists(pdf) else b''
        n_pages = len(re.findall(rb'/Type\s*/Page[^s]', blob))
        check('F4 三页 PDF 写出且正好 3 页',
              rc == 0 and n_pages == 3, 'rc=%r pages=%d size=%d'
              % (rc, n_pages, len(blob)))
        # 失败路径：写不出的目录 ⇒ 非 0 退出
        rc = ls.main(['--layout', '1', '--out',
                      os.path.join(td, 'no-such-dir', 'x.pdf')])
        check('F5 输出目录不存在 ⇒ rc 2', rc == 2, 'rc=%r' % rc)

    print()
    if N_FAIL:
        print('%d/%d FAIL（%s）' % (N_FAIL, N_PASS + N_FAIL, ', '.join(FAILS)))
        return 1
    print('%d/%d PASS' % (N_PASS, N_PASS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
