#!/usr/bin/env python3
"""A5 判分器的自检：三项判据、多姿态一致性、退出码、标注图。

钉住：
  · fold/误差函数在 0/45/89.9/负角上不抖；
  · 色格、错格、yaw、位置四种失败各判对，且优先级稳定；
  · `person_review` 不混进机判（颜色来源不明/多姿态不一致）；
  · CLI 退出码 0（全过）/1（有不过）/2（数据缺失）；
  · 标注 PNG 真写出来。

跑法：
    python3 scripts/test_stage2_score.py
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

import cv2                                                       # noqa: E402

import stage2_score as ss                                        # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def _grid():
    colors = ['red', 'orange', 'tan', 'yellow', 'yellowgreen', 'cyan',
              'purple', 'spare', 'spare']
    cells = []
    for i, col in enumerate(colors):
        r, c = divmod(i, 3)
        cells.append({'idx': i, 'row': r, 'col': c, 'color': col,
                      'center_pad_mm': [(-1 + c) * 60.0, (1 - r) * 60.0]})
    return {'version': 1, 'pitch_mm': 60.0, 'cells': cells,
            'grid_yaw_base_deg': 0.0}


def _pose(bid, cls, xy, yaw, status='both', name='survey_place'):
    return {'name': name, 'blocks': [{
        'block_id': bid, 'class': cls, 'class_status': status,
        'center_pad_mm': list(xy), 'yaw_deg': yaw}]}


def _obs(poses, expected, round_no=1):
    return {'round': round_no, 'poses': poses, 'expected': expected,
            'arms': ['geometric']}


def _exp(bid=0, cell=0, arm='geometric', ran=True):
    return {'block_id': bid, 'arm': arm, 'cell_idx': cell,
            'yaw_exec_deg': 0.0, 'ran': ran, 'exec_ok': True}


def _one(obs, **kw):
    res = ss.score(obs, _grid(), **kw)
    return res['rows'][0], res


# ---------------------------------------------------------------- 纯函数
@case
def test_angles():
    assert ss.fold_mod90(89.9) == 89.9
    assert abs(ss.fold_mod90(90.0)) < 1e-9
    assert abs(ss.fold_mod90(-1.0) - 89.0) < 1e-9
    assert abs(ss.ang_diff_mod90(1.0, 89.0) - 2.0) < 1e-9
    assert abs(ss.ang_diff_mod90(0.0, 45.0) - 45.0) < 1e-9
    assert abs(ss.yaw_err_mod90(88.0) - 2.0) < 1e-9
    assert abs(ss.pos_err([0, 0], [3, 4]) - 5.0) < 1e-9


@case
def test_circular_mean_and_span_wrap():
    assert abs(ss._circular_mean_deg([89.0, 1.0]) - 0.0) < 1e-6
    assert abs(ss._angular_span([89.0, 1.0]) - 2.0) < 1e-6
    assert abs(ss._angular_span([0.0, 45.0]) - 45.0) < 1e-6


# ---------------------------------------------------------------- 单块判分
@case
def test_ok():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 2.0),
                      _pose(0, 'red', [-60, 60], 1.0)], [_exp()]))
    assert r['status'] == 'ok', r
    assert abs(r['yaw_err_deg'] - 1.5) < 1e-6, r
    assert abs(r['pos_err_mm']) < 1e-9, r
    assert r['obs_cell_idx'] == 0


@case
def test_color_wrong():
    r, _ = _one(_obs([_pose(0, 'cyan', [-60, 60], 1.0),
                      _pose(0, 'cyan', [-60, 60], 2.0)], [_exp(cell=0)]))
    assert r['status'] == 'color_wrong', r


@case
def test_wrong_cell():
    # 红块放进橙色格（cell 1 中心 (60,60)）：类没错，格错。
    r, _ = _one(_obs([_pose(0, 'red', [0, 60], 1.0),
                      _pose(0, 'red', [0, 60], 2.0)], [_exp(cell=0)]))
    assert r['status'] == 'wrong_cell', r
    assert r['obs_cell_idx'] == 1


@case
def test_yaw_fail():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 12.0),
                      _pose(0, 'red', [-60, 60], 12.5)], [_exp()]))
    assert r['status'] == 'yaw_fail', r


@case
def test_pos_fail():
    r, _ = _one(_obs([_pose(0, 'red', [-40, 60], 1.0),
                      _pose(0, 'red', [-40, 60], 2.0)], [_exp()]))
    assert r['status'] == 'pos_fail', r
    assert abs(r['pos_err_mm'] - 20.0) < 1e-6


@case
def test_person_review_on_class_status():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0, status='conflict'),
                      _pose(0, 'red', [-60, 60], 2.0, status='conflict')],
                     [_exp()]))
    assert r['status'] == 'person_review', r
    assert '颜色来源' in r['reason']


@case
def test_person_review_on_pose_disagreement():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 0.0),
                      _pose(0, 'red', [-60, 60], 8.0)], [_exp()]))
    assert r['status'] == 'person_review', r
    assert 'yaw 跨姿态' in r['reason']
    # 位置门 10 mm（2026-10-28 按实测噪声放宽，原 3 mm）：8 mm 现在**不算**不一致，
    # 12 mm 才算 —— 两边都要钉住，否则「放宽」和「门被摘掉」分不出来。
    r_in, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0),
                         _pose(0, 'red', [-52, 60], 1.0)], [_exp()]))
    assert r_in['status'] != 'person_review', r_in
    r2, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0),
                       _pose(0, 'red', [-48, 60], 1.0)], [_exp()]))
    assert r2['status'] == 'person_review', r2
    assert '位置跨姿态' in r2['reason']


@case
def test_not_run_and_not_measured():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0)], [_exp(ran=False)]))
    assert r['status'] == 'not_run', r
    r2, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0)],
                      [_exp(bid=3, cell=3)]))
    assert r2['status'] == 'not_measured', r2


@case
def test_single_pose_is_person_review():
    r, _ = _one(_obs([_pose(0, 'red', [-60, 60], 1.0)], [_exp()]))
    assert r['status'] == 'person_review', r
    assert '只有 1 个姿态' in r['reason']


# ---------------------------------------------------------------- 汇总
@case
def test_summary_excludes_person_review():
    obs = _obs([_pose(0, 'red', [-60, 60], 1.0),
                _pose(0, 'red', [-60, 60], 2.0),
                _pose(1, 'orange', [0, 60], 1.0, status='unknown'),
                _pose(1, 'orange', [0, 60], 2.0, status='unknown')],
               [_exp(0, 0), _exp(1, 1)])
    res = ss.score(obs, _grid())
    s = res['summary']['overall']
    assert s['n_expected'] == 2
    assert s['n_machine'] == 1, s
    assert s['n_ok'] == 1
    assert s['n_person_review'] == 1


# ---------------------------------------------------------------- CLI
def _write_inputs(root, ok=True):
    grid = _grid()
    gpath = os.path.join(root, 'grid.json')
    json.dump(grid, open(gpath, 'w'))
    yaw = 2.0 if ok else 20.0
    obs = _obs([_pose(0, 'red', [-60, 60], yaw),
                _pose(0, 'red', [-60, 60], yaw)], [_exp()])
    opath = os.path.join(root, 'obs.json')
    json.dump(obs, open(opath, 'w'))
    return gpath, opath


def _run(args):
    return subprocess.run([sys.executable,
                           os.path.join(HERE, 'stage2_score.py')] + args,
                          capture_output=True, text=True)


@case
def test_cli_exit_codes_and_png():
    root = tempfile.mkdtemp(prefix='stage2-score-')
    try:
        gpath, opath = _write_inputs(root, ok=True)
        prefix = os.path.join(root, 'report')
        p = _run(['--observations', opath, '--grid', gpath,
                  '--out-report', prefix])
        assert p.returncode == 0, '全过应退出 0：%d\n%s\n%s' % (
            p.returncode, p.stdout, p.stderr)
        for ext in ('.json', '.md', '.png'):
            assert os.path.exists(prefix + ext), '缺 %s\n%s' % (ext, p.stdout)
        img = cv2.imread(prefix + '.png')
        assert img is not None and img.size > 0

        gpath2, opath2 = _write_inputs(root, ok=False)
        p2 = _run(['--observations', opath2, '--grid', gpath2])
        assert p2.returncode == 1, '有不过应退出 1：%d' % p2.returncode

        p3 = _run(['--observations', os.path.join(root, 'nope.json'),
                   '--grid', gpath])
        assert p3.returncode == 2, '缺数据应退出 2：%d' % p3.returncode
        assert 'Traceback' not in p3.stderr, p3.stderr
    finally:
        shutil.rmtree(root, ignore_errors=True)


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
