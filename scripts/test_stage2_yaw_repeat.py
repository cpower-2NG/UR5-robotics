#!/usr/bin/env python3
"""A3 重复性脚本的纯逻辑自检（不读帧、不需要 cv2）。

钉住：
  · 同色 + 位置 ≤10 mm 才配对，且一对一；
  · 跨帧组内 mod 90 极差正确；
  · 极差超阈值 → pass=False；
  · 组数不足由 CLI 报 2（这里测 `summarize` 的过滤口径）。

跑法：
    python3 scripts/test_stage2_yaw_repeat.py
"""

import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage2_yaw_repeat as yr                                   # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


def _item(cls, cx, cy, yaw, yaw_rect=None, trusted=True, ext=(36.0, 36.0)):
    return {'block': {'class': cls, 'cx': cx, 'cy': cy, 'yaw': yaw,
                      'yaw_rect': yaw if yaw_rect is None else yaw_rect},
            'pix': {'yaw_deg': yaw, 'trusted': trusted,
                    'ext_mm': list(ext), 'fill': 0.9, 'n_pixels': 900}}


@case
def test_match_same_class_and_distance():
    a = [_item('red', 100, 100, 10)]
    b = [_item('red', 105, 100, 12), _item('cyan', 100, 100, 40)]
    m = yr._match(a, b, max_mm=10)
    assert m[0][0] == 0, '应配到同色的红块，不能配青块'
    assert abs(m[0][1] - 5.0) < 1e-9


@case
def test_match_rejects_far_and_other_class():
    a = [_item('red', 100, 100, 10)]
    b = [_item('red', 120, 100, 12)]
    assert yr._match(a, b, max_mm=10) == {}
    b2 = [_item('cyan', 100, 100, 12)]
    assert yr._match(a, b2, max_mm=10) == {}


@case
def test_match_one_to_one():
    a = [_item('red', 100, 100, 10), _item('red', 106, 100, 20)]
    b = [_item('red', 103, 100, 12)]
    m = yr._match(a, b, max_mm=10)
    assert len(m) == 1, '一个块不能被两个锚共用'
    assert 0 in m or 1 in m


@case
def test_groups_and_span_pass():
    per = [
        [_item('red', 100, 100, 10), _item('cyan', 200, 100, 40)],
        [_item('red', 101, 100, 12), _item('cyan', 199, 100, 43)],
        [_item('red', 100, 101, 9), _item('cyan', 200, 101, 41)],
    ]
    gs = yr.group_blocks(per)
    assert len(gs) == 2
    rows = yr.summarize(gs, threshold_deg=6.0)
    assert all(r['n'] == 3 for r in rows), rows
    assert all(r['pass'] for r in rows), rows


@case
def test_span_fail_on_10deg():
    per = [
        [_item('red', 100, 100, 0)],
        [_item('red', 100, 100, 3)],
        [_item('red', 100, 100, 10)],
    ]
    rows = yr.summarize(yr.group_blocks(per), threshold_deg=6.0)
    assert rows[0]['n'] == 3
    assert abs(rows[0]['span_pix'] - 10.0) < 1e-9, rows[0]
    assert rows[0]['pass'] is False


@case
def test_span_is_mod90():
    per = [
        [_item('red', 100, 100, 1)],
        [_item('red', 100, 100, 91)],
        [_item('red', 100, 100, 46)],
    ]
    rows = yr.summarize(yr.group_blocks(per), threshold_deg=6.0)
    # 1 与 91 在 mod 90 下同角；46 在另一侧，极差 45。
    assert abs(rows[0]['span_pix'] - 45.0) < 1e-9, rows[0]


@case
def test_missing_frames_kept_in_rows_but_filtered():
    per = [
        [_item('red', 100, 100, 10), _item('cyan', 200, 100, 40)],
        [_item('red', 100, 100, 11)],
    ]
    rows = yr.summarize(yr.group_blocks(per), threshold_deg=6.0)
    assert len(rows) == 2
    assert {r['n'] for r in rows} == {2, 1}


@case
def test_no_anchor_groups():
    assert yr.summarize(yr.group_blocks([]), 6.0) == []


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
