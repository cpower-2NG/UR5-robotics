#!/usr/bin/env python3
"""阶段 2 双源颜色判定的自检（纯逻辑，不需要模型/图片）。

跑法：
    python3 scripts/test_stage2_color.py
"""

import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage2_color as sc                                        # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def _block(cls, share, uv=(100.0, 100.0), cx=0.0, cy=0.0):
    return {'class': cls, 'class_share': share, 'uv': list(uv),
            'cx': cx, 'cy': cy}


def _det(cls, conf, uv=(100.0, 100.0)):
    return {'class': cls, 'conf': conf, 'uv': list(uv)}


@case
def test_both_agree():
    r = sc.classify_block_dual(_block('red', 0.99), _det('red', 0.90), 3.0)
    assert r['status'] == 'ok' and r['source'] == 'both' and r['class'] == 'red'
    assert r['agree'] is True


@case
def test_hsv_only():
    r = sc.classify_block_dual(_block('purple', 0.99), None, None)
    assert r['status'] == 'ok' and r['source'] == 'hsv' and r['class'] == 'purple'


@case
def test_yolo_only():
    r = sc.classify_block_dual(_block('unknown', 0.0), _det('cyan', 0.50), 10.0)
    assert r['status'] == 'ok' and r['source'] == 'yolo' and r['class'] == 'cyan'


@case
def test_conflict_default_is_unknown():
    r = sc.classify_block_dual(_block('red', 0.99), _det('cyan', 0.80), 1.0)
    assert r['status'] == 'conflict' and r['class'] is None
    assert r['agree'] is False


@case
def test_conflict_policies():
    b, d = _block('red', 0.99), _det('cyan', 0.80)
    rh = sc.classify_block_dual(b, d, 1.0, conflict_policy='hsv')
    ry = sc.classify_block_dual(b, d, 1.0, conflict_policy='yolo')
    assert rh['class'] == 'red' and rh['source'] == 'hsv'
    assert ry['class'] == 'cyan' and ry['source'] == 'yolo'
    try:
        sc.classify_block_dual(b, d, 1.0, conflict_policy='guess')
    except ValueError:
        pass
    else:
        raise AssertionError('非法 conflict_policy 被放行')


@case
def test_low_conf_yolo_ignored():
    r = sc.classify_block_dual(_block('red', 0.99), _det('cyan', 0.20), 1.0)
    assert r['status'] == 'ok' and r['source'] == 'hsv' and r['class'] == 'red'


@case
def test_low_hsv_share_uses_yolo():
    r = sc.classify_block_dual(_block('red', 0.30), _det('yellow', 0.80), 1.0)
    assert r['status'] == 'ok' and r['source'] == 'yolo' and r['class'] == 'yellow'


@case
def test_neither_source():
    r = sc.classify_block_dual(_block('unknown', 0.0), None, None)
    assert r['status'] == 'unknown' and r['class'] is None and r['source'] == 'none'


@case
def test_match_threshold():
    b = _block('red', 0.99, uv=(100, 100))
    d = _det('red', 0.9, uv=(200, 100))
    det, dist = sc.match_yolo(b, [d], max_uv_px=60)
    assert det is None and dist is None
    det, dist = sc.match_yolo(b, [d], max_uv_px=120)
    assert det is d and abs(dist - 100.0) < 1e-9


@case
def test_one_to_one_assignment():
    blocks = [_block('red', 0.99, uv=(100, 100)),
              _block('cyan', 0.99, uv=(110, 100))]
    dets = [_det('red', 0.9, uv=(105, 100))]
    assigned = sc.assign_yolo(blocks, dets, max_uv_px=60)
    assert len(assigned) == 1, '一个检测框不能同时配给两块'
    assert 0 in assigned


@case
def test_classify_blocks_returns_every_block():
    blocks = [_block('red', 0.99, uv=(100, 100), cx=1, cy=2),
              _block('purple', 0.99, uv=(200, 100), cx=3, cy=4)]
    dets = [_det('red', 0.9, uv=(101, 100))]
    out = sc.classify_blocks(blocks, dets)
    assert len(out) == 2
    assert out[0]['class'] == 'red' and out[0]['source'] == 'both'
    assert out[1]['class'] == 'purple' and out[1]['source'] == 'hsv'
    assert out[1]['block_idx'] == 1


@case
def test_classify_blocks_keeps_block_own_idx():
    """块自带 ``idx`` 时必须沿用它 —— ``find_blocks`` 的 idx 会跳号。

    2026-10-27 B2 终态测量的真实事故：``find_blocks`` 给出 idx=3,5,8,11,15,16，
    ``classify_blocks`` 却按位置报 0..5，调用方按 idx 查表查空，四块真积木全成
    unknown，标定直接判「样本不足」。
    """
    b0 = _block('red', 0.99, uv=(100, 100), cx=1, cy=2)
    b1 = _block('purple', 0.99, uv=(200, 100), cx=3, cy=4)
    b0['idx'], b1['idx'] = 3, 11
    dets = [_det('red', 0.9, uv=(101, 100))]
    out = sc.classify_blocks([b0, b1], dets)
    assert [r['block_idx'] for r in out] == [3, 11], out
    assert out[0]['class'] == 'red' and out[1]['class'] == 'purple'
    # 没有 idx 的块退回位置号（candidates.json 的块就有 idx，这里测兜底）
    del b0['idx']
    out = sc.classify_blocks([b0, b1], dets)
    assert [r['block_idx'] for r in out] == [0, 11], out


@case
def test_classify_blocks_accepts_max_uv_px():
    """``max_uv_px`` 只给分配用，不能转进 classify_block_dual（会 TypeError）。"""
    blocks = [_block('red', 0.99, uv=(100, 100))]
    dets = [_det('red', 0.9, uv=(160, 100))]
    out = sc.classify_blocks(blocks, dets, max_uv_px=50)
    assert out[0]['class'] == 'red' and out[0]['source'] == 'hsv'
    assert out[0]['yolo_class'] is None
    out2 = sc.classify_blocks(blocks, dets, max_uv_px=70)
    assert out2[0]['source'] == 'both'


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
