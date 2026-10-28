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


# ---------------------------------------------------------------- 投票
def _rec(cls, share, status='ok'):
    return {'class': cls, 'class_share': share, 'class_status': status}


@case
def test_vote_picks_by_share_not_order():
    """两强差够大 ⇒ 票多的赢，**与出现顺序无关**（先到的赢就是原缺陷）。"""
    a = [_rec('orange', 0.65), _rec('tan', 0.93)]
    b = list(reversed(a))
    for recs in (a, b):
        cls, status, w, reason = sc.vote_class(recs)
        assert (cls, status) == ('tan', 'ok'), (recs, cls, status, reason)
        # 返回的权重是**赢家自己的票权**（0.93），不是两强之和 —— 调用方拿它
        # 当「这条判定的票份额」用，跟单帧 ``class_share`` 同一量纲。
        assert abs(w - 0.93) < 1e-9, w


@case
def test_vote_close_is_conflict():
    cls, status, _w, reason = sc.vote_class([_rec('orange', 0.80),
                                             _rec('tan', 0.75)])
    assert cls == 'orange' and status == 'conflict', (cls, status, reason)
    assert '票权接近' in reason, reason


@case
def test_vote_inner_conflict_leaks_through():
    """某条自己就是两源冲突：它不投票，但这份不确定必须带上来。"""
    cls, status, _w, reason = sc.vote_class([_rec('tan', 0.95),
                                             _rec('orange', 0.4, 'conflict')])
    assert cls == 'tan' and status == 'conflict', (cls, status, reason)


@case
def test_vote_ignores_self_contradictory_unknown():
    """状态说「认不出」却带类名的记录不投票 —— 否则一条 unknown 能把整块推成 ok。

    这是 ``test_summary_excludes_person_review`` 变红时定位到的：判分器会把
    「本该人看」的块当成机器判过。
    """
    cls, status, _w, _r = sc.vote_class([_rec('orange', 1.0, 'unknown')])
    assert cls is None and status == 'unknown', (cls, status)
    # 与一条真票混在一起时，unknown 那条不参与加权
    cls, status, w, _r = sc.vote_class([_rec('orange', 1.0, 'unknown'),
                                        _rec('tan', 0.6)])
    assert (cls, status) == ('tan', 'ok') and abs(w - 0.6) < 1e-9, (cls, w)


@case
def test_vote_missing_share_counts_as_one():
    """老记录没有 class_share：一票就是一票，不能当 0 票丢掉。"""
    cls, status, w, _r = sc.vote_class([{'class': 'red'},
                                        {'class': 'cyan', 'class_share': 0.6}])
    assert cls == 'red' and status == 'ok' and abs(w - 1.0) < 1e-9, (cls, w)


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
