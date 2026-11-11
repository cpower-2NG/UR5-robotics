#!/usr/bin/env python3
"""摆位偏差工具的自检：配对 / 90° 折角 / 缺真值不伪造 / 超差也不 void。

最坏的两个失败都**不会报错**：
1. yaw 差 88° 与 2° 直接相减 → 报 86° 的假事故（积木是正方形，90° 一个周期）；
2. 真值文件不存在/为空时「补 0」→ 报表看起来完美，实际什么都没量。
所以这两件事在这里逐条钉死。

跑法：python3 scripts/test_stage4_placement_deviation.py
"""
import json
import math
import os
import sys
import tempfile
import io
import contextlib

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_placement_deviation as pd                          # noqa: E402
import stage4_layouts as lay                                     # noqa: E402

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


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def section(title):
    print()
    print('== %s ==' % title)


def make_doc(blocks):
    return {"blocks": blocks}


def ob(color, x, y, yaw=0.0, n=None, layer=0):
    d = {"id": 0, "color": color, "x": x / 1000.0, "y": y / 1000.0,
         "yaw": yaw, "layer": layer}
    if n is not None:
        d["meas"] = {"n_frames": n}
    return d


def write_json(path, doc):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)


def run_main(argv):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = pd.main(argv)
    return rc, buf.getvalue()


def main():
    td = tempfile.mkdtemp(prefix='deviation-')

    # ------------------------------------------------------------------
    section('A. yaw 折角（90° 周期）')
    check('A1 0 → 0', close(pd.wrap_yaw_deg(0.0), 0.0))
    check('A2 +1 → +1', close(pd.wrap_yaw_deg(1.0), 1.0))
    check('A3 88° − 2° = 86° ⇒ −4°（不是 86°）',
          close(pd.wrap_yaw_deg(86.0), -4.0), repr(pd.wrap_yaw_deg(86.0)))
    check('A4 2° − 88° = −86° ⇒ +4°', close(pd.wrap_yaw_deg(-86.0), 4.0))
    check('A5 90 → 0', close(pd.wrap_yaw_deg(90.0), 0.0))
    check('A6 −90 → 0', close(pd.wrap_yaw_deg(-90.0), 0.0))
    check('A7 45 的折法（半开区间）', close(pd.wrap_yaw_deg(45.0), -45.0))
    check('A8 −45 留在 −45', close(pd.wrap_yaw_deg(-45.0), -45.0))
    ok = all(-45.0 <= pd.wrap_yaw_deg(v) < 45.0
             for v in range(-400, 401, 7))
    check('A9 任意角度都落在 [−45, 45)', ok)

    # ------------------------------------------------------------------
    section('B. 期望块：布局表 / scene 文档同源')
    exp, src = pd.expected_blocks('S0', 1)
    truth = lay.layout_blocks('S0', 1)
    ok = len(exp) == len(truth) == 5
    for e, t in zip(exp, truth):
        ok = ok and e['id'] == t['id'] and e['color'] == t['color']
        ok = ok and close(e['x_mm'], t['x'] * 1000.0)
        ok = ok and close(e['y_mm'], t['y'] * 1000.0)
        ok = ok and close(e['yaw_deg'], t['yaw'])
    check('B1 与布局表逐块一致（毫米）', ok)
    check('B2 来源可追溯', src == 'layout:S0:1', src)
    exp2, src2 = pd.expected_blocks(scene=lay.scene_truth('S0', 1))
    check('B3 scene 文档给出同样的期望',
          [(e['id'], round(e['x_mm'], 3), round(e['y_mm'], 3), e['yaw_deg'])
           for e in exp2] ==
          [(e['id'], round(e['x_mm'], 3), round(e['y_mm'], 3), e['yaw_deg'])
           for e in exp])
    raised = False
    try:
        pd.expected_blocks(None, None, None)
    except ValueError:
        raised = True
    check('B4 什么都没给 ⇒ 报错（不猜）', raised)

    # ------------------------------------------------------------------
    section('C. 配对')
    A = {'id': 10, 'color': 'red', 'x_mm': 100.0, 'y_mm': 100.0, 'yaw_deg': 0.0}
    B = {'id': 11, 'color': 'cyan', 'x_mm': 200.0, 'y_mm': 100.0, 'yaw_deg': 0.0}
    pairs, ue, uo = pd.match_blocks([A, B], [
        {'id': 0, 'color': 'red', 'x_mm': 101.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
        {'id': 1, 'color': 'cyan', 'x_mm': 201.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
    ])
    check('C1 两块对两块', pairs == [(0, 0), (1, 1)] and not ue and not uo,
          repr((pairs, ue, uo)))
    pairs, ue, uo = pd.match_blocks([A, B], [
        {'id': 0, 'color': 'cyan', 'x_mm': 101.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
        {'id': 1, 'color': 'red', 'x_mm': 201.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
    ])
    check('C2 颜色对调、位置没动 ⇒ 仍按位置配上（色错不硬拆）',
          pairs == [(0, 0), (1, 1)], repr(pairs))
    pairs, ue, uo = pd.match_blocks([A, B], [
        {'id': 0, 'color': 'red', 'x_mm': 101.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
    ])
    check('C3 少一块 ⇒ 只有 1 对 + 1 个未配期望',
          pairs == [(0, 0)] and ue == [1] and uo == [], repr((pairs, ue, uo)))
    pairs, ue, uo = pd.match_blocks([A], [
        {'id': 0, 'color': 'red', 'x_mm': 101.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
        {'id': 1, 'color': 'purple', 'x_mm': 100.0, 'y_mm': 102.0, 'yaw_deg': 0.0},
    ])
    check('C4 多一块 ⇒ 只配一对，另一块记未配观测',
          pairs == [(0, 0)] and ue == [] and uo == [1], repr((pairs, ue, uo)))
    pairs, ue, uo = pd.match_blocks([A], [
        {'id': 0, 'color': 'red', 'x_mm': 160.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
    ])
    check('C5 超出配对半径 ⇒ 不硬配',
          pairs == [] and ue == [0] and uo == [0], repr((pairs, ue, uo)))
    pairs, ue, uo = pd.match_blocks([A], [
        {'id': 0, 'color': 'red', 'x_mm': 101.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
        {'id': 1, 'color': 'red', 'x_mm': 102.0, 'y_mm': 100.0, 'yaw_deg': 0.0},
    ])
    check('C6 一块期望不会被配两次', len(pairs) == 1 and ue == [] and len(uo) == 1)

    # ------------------------------------------------------------------
    section('D. 单块偏差')
    e = {'id': 3, 'color': 'orange', 'x_mm': 500.0, 'y_mm': 300.0, 'yaw_deg': 40.0}
    o = {'id': 0, 'color': 'orange', 'x_mm': 503.0, 'y_mm': 296.0,
         'yaw_deg': 44.0, 'n_frames': 9}
    r = pd.deviation_record(e, o)
    check('D1 dx/dy', close(r['dx_mm'], 3.0) and close(r['dy_mm'], -4.0),
          repr((r['dx_mm'], r['dy_mm'])))
    check('D2 dist = 5.0', close(r['dist_mm'], 5.0), repr(r['dist_mm']))
    check('D3 dyaw = +4', close(r['dyaw_deg'], 4.0), repr(r['dyaw_deg']))
    check('D4 颜色一致 + 帧数带过来',
          r['color_match'] and r['n_frames'] == 9)
    r2 = pd.deviation_record(dict(e, yaw_deg=5.0), dict(o, yaw_deg=88.0))
    check('D5 88° vs 5° ⇒ −7°（90° 周期，不报 83°）',
          close(r2['dyaw_deg'], -7.0), repr(r2['dyaw_deg']))
    r3 = pd.deviation_record(e, dict(o, color='purple'))
    check('D6 颜色不同 ⇒ color_match=False（但仍记偏差）',
          r3['color_match'] is False and r3['observed_color'] == 'purple')

    # ------------------------------------------------------------------
    section('E. 统计（空表不伪造 0）')
    empty = pd.summarize([])
    check('E1 空表：计数 0、其它是 None 而不是 0',
          empty['n'] == 0 and empty['max_dist_mm'] is None
          and empty['mean_dist_mm'] is None and empty['within_tol'] is False,
          repr(empty))
    recs = [
        {'dist_mm': 2.0, 'dx_mm': 1.0, 'dy_mm': 2.0, 'dyaw_deg': 3.0},
        {'dist_mm': 6.0, 'dx_mm': -2.0, 'dy_mm': 4.0, 'dyaw_deg': -12.0},
    ]
    s = pd.summarize(recs, tol_mm=5.0, tol_deg=10.0)
    check('E2 一块超差 ⇒ within_tol=False（只是报表）',
          s['n_within_tol'] == 1 and s['within_tol'] is False, repr(s))
    check('E3 max/mean 距离', close(s['max_dist_mm'], 6.0)
          and close(s['mean_dist_mm'], 4.0), repr(s))
    check('E4 均值 dx/dy', close(s['mean_dx_mm'], -0.5)
          and close(s['mean_dy_mm'], 3.0), repr(s))
    check('E5 均值/最大 |dyaw|', close(s['mean_dyaw_deg'], -4.5)
          and close(s['max_abs_dyaw_deg'], 12.0), repr(s))
    s2 = pd.summarize([{'dist_mm': 5.0, 'dx_mm': 5.0, 'dy_mm': 0.0,
                        'dyaw_deg': 10.0}], 5.0, 10.0)
    check('E6 正好在目标边界 ⇒ 算在目标内', s2['within_tol'] is True)

    # ------------------------------------------------------------------
    section('F. 整表（build_report）')
    obs = [ob('yellowgreen', 552.7 + 2.0, 197.8 - 1.0, 40.7 + 3.0, n=8),
           ob('yellow', 292.7, 375.0, 18.2, n=9),
           ob('cyan', 415.5, 290.8, 16.3 + 1.0, n=9),
           ob('orange', 507.8, 313.8, 100.5, n=9),
           ob('purple', 284.4, 215.9, 126.9, n=9)]
    rep = pd.build_report(make_doc(obs), 'truth.json', exp, 'layout:S0:1')
    check('F1 5 块全配上', rep['n_matched'] == 5 and rep['n_expected'] == 5
          and rep['n_observed'] == 5, repr((rep['n_matched'], rep['n_observed'])))
    check('F2 kind / yaw 周期 / verdict',
          rep['kind'] == 'stage4-placement-deviation'
          and rep['yaw_period_deg'] == 90.0 and '不 void' in rep['verdict'])
    b0 = [b for b in rep['blocks'] if b['id'] == 0][0]
    check('F3 第一块 dx=+2 dy=−1 dyaw=+3',
          close(b0['dx_mm'], 2.0) and close(b0['dy_mm'], -1.0)
          and close(b0['dyaw_deg'], 3.0), repr(b0))
    check('F4 未配列表为空', rep['unmatched_expected'] == []
          and rep['unmatched_observed'] == [])
    rep2 = pd.build_report(make_doc(obs[:4]), 'truth.json', exp, 'layout:S0:1')
    check('F5 少一块 ⇒ 记未配，不补数',
          rep2['n_matched'] == 4 and len(rep2['unmatched_expected']) == 1
          and rep2['stats']['n'] == 4,
          repr((rep2['n_matched'], rep2['unmatched_expected'])))
    raised = False
    try:
        pd.build_report(make_doc([]), 'truth.json', exp, 'layout:S0:1')
    except ValueError:
        raised = True
    check('F6 观测为空 ⇒ 抛错（不出数）', raised)

    # ------------------------------------------------------------------
    section('G. CLI（超差也 rc 0；缺真值 rc 2 且不写文件）')
    good = os.path.join(td, 'truth-good.json')
    write_json(good, make_doc(obs))
    out = os.path.join(td, 'dev-good.json')
    rc, txt = run_main(['--truth', good, '--batch', 'S0', '--layout', '1',
                        '--out', out])
    check('G1 正常跑 ⇒ rc 0 + 写文件', rc == 0 and os.path.exists(out), txt)
    wrote = json.load(open(out, encoding='utf-8')) if os.path.exists(out) else {}
    check('G2 文件内容与 build_report 一致',
          wrote.get('n_matched') == 5 and wrote.get('kind')
          == 'stage4-placement-deviation')
    check('G3 终端摘要含「只归档」类说明', '摆位偏差' in txt and 'record-only' in txt)

    far = [ob(b['color'], b['x_mm'] + 20.0, b['y_mm'] - 15.0, b['yaw_deg'] + 30.0)
           for b in exp]
    bad = os.path.join(td, 'truth-far.json')
    write_json(bad, make_doc(far))
    out2 = os.path.join(td, 'dev-far.json')
    rc, txt = run_main(['--truth', bad, '--batch', 'S0', '--layout', '1',
                        '--out', out2])
    rep_far = json.load(open(out2, encoding='utf-8')) if os.path.exists(out2) else {}
    check('G4 摆得很歪 ⇒ 仍然 rc 0（绝不 void）', rc == 0, 'rc=%r' % rc)
    check('G5 但报表里 within_tol=False、数是真的',
          rep_far.get('stats', {}).get('within_tol') is False
          and rep_far.get('stats', {}).get('max_dist_mm', 0) > 20.0,
          repr(rep_far.get('stats')))

    rc, txt = run_main(['--truth', os.path.join(td, 'nope.json'),
                        '--batch', 'S0', '--layout', '1',
                        '--out', os.path.join(td, 'dev-missing.json')])
    check('G6 真值文件不存在 ⇒ rc 2 且不写文件（不伪造）',
          rc == 2 and not os.path.exists(os.path.join(td, 'dev-missing.json')), txt)

    empty_doc = os.path.join(td, 'truth-empty.json')
    write_json(empty_doc, make_doc([]))
    rc, txt = run_main(['--truth', empty_doc, '--batch', 'S0', '--layout', '1'])
    check('G7 真值为空 ⇒ rc 2', rc == 2, txt)

    rc, txt = run_main(['--truth', good, '--batch', 'S0'])
    check('G8 没给 --layout ⇒ rc 2（不猜布局）', rc == 2, txt)

    scene = os.path.join(td, 'scene.json')
    write_json(scene, lay.scene_truth('S0', 1))
    rc, txt = run_main(['--truth', good, '--scene', scene])
    check('G9 --scene 路径可用 + 无 --out 打到 stdout', rc == 0
          and 'n_matched' in txt, txt[:120])

    mixed = list(obs)
    mixed.append(ob('red', 330.0, 250.0, 0.0))
    mix_path = os.path.join(td, 'truth-mixed.json')
    write_json(mix_path, make_doc(mixed))
    rc, txt = run_main(['--truth', mix_path, '--batch', 'S0', '--layout', '1'])
    check('G10 多出来的块只记录、不崩', rc == 0 and '没配上的观测块' in txt, txt)

    tower = [ob('red', 330.0, 250.0, 0.0, layer=0),
             ob('red', 330.0, 250.0, 0.0, layer=1)]
    rep_t = pd.build_report(make_doc(tower), 'tower.json', exp, 'layout:S0:1')
    check('G11 塔的下层不参与配对（只取 layer 0）',
          rep_t['n_observed'] == 1, repr(rep_t['n_observed']))

    print()
    if N_FAIL:
        print('%d/%d FAIL（%s）' % (N_FAIL, N_PASS + N_FAIL, ', '.join(FAILS)))
        return 1
    print('%d/%d PASS' % (N_PASS, N_PASS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
