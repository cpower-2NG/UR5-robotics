#!/usr/bin/env python3
"""E4a 悬停门（``stage4_hover_check.py``）的纯函数自检：不需要图片/相机/臂。

钉住四件事：

1. 帧目录两种命名（单帧 ``color.png`` / 批量 ``color_000.png``）都认，空目录不冒充有帧；
2. 判据三条（可用率 ≥60%、中心半极差 ≤2 mm、yaw 极差 ≤2°）**真的被查**——
   每条都做一次变异（把阈值改严），期望同一条数据从 PASS 翻成 FAIL；
3. 悬停几何用的是 **0.2042**（裸指尖），不是 0.1757（块高折扣后的抓取点）；
   两者差 28.5 mm，改错会静默把臂压低；
4. 少给/多给悬停高度、目录不存在 ⇒ 判「数据缺失」（退出码 2），不是悄悄放过。

跑法：``python3 scripts/test_stage4_hover.py``；退出码 0 = 全通过。
"""

import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import stage4_hover_check as hc                                 # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def sample(usable=True, x=0.0, y=0.0, yaw=10.0, trusted=True, cover=0.5):
    if not usable:
        return {'usable': False, 'found': False}
    return {'usable': True, 'found': True, 'center_base_mm': [x, y],
            'yaw_color_deg': yaw, 'yaw_color_trusted': trusted,
            'face_cover': cover}


def grid(n=15, dx=1.0, dy=0.0, dyaw=0.5):
    return [sample(x=i * dx, y=i * dy, yaw=10.0 + i * dyaw) for i in range(n)]


def good_set(n_unusable=5, n_usable=15, **kw):
    return grid(n_usable, **kw) + [sample(usable=False) for _ in range(n_unusable)]


# =====================================================================
print('--- A. 帧目录识别 ---')
with tempfile.TemporaryDirectory() as td:
    d1 = os.path.join(td, 'burst')
    os.makedirs(d1)
    for i in (0, 1, 2):
        open(os.path.join(d1, 'color_%03d.png' % i), 'w').close()
    check('批量帧 → 序号 [0,1,2]', hc.list_indices(d1) == [0, 1, 2],
          repr(hc.list_indices(d1)))

    d2 = os.path.join(td, 'single')
    os.makedirs(d2)
    open(os.path.join(d2, 'color.png'), 'w').close()
    check('单帧 → [None]', hc.list_indices(d2) == [None])

    d3 = os.path.join(td, 'empty')
    os.makedirs(d3)
    check('空目录 → []', hc.list_indices(d3) == [])

check('parse_capture 正常', hc.parse_capture('50:runs/x') ==
      (50.0, os.path.abspath('runs/x')))
it = None
try:
    hc.parse_capture('runs/x')
except ValueError as e:
    it = str(e)
check('parse_capture 缺高度 ⇒ 拒绝', it is not None and '高度:目录' in it, repr(it))

# =====================================================================
print('--- B. summarize：三条判据（含变异）---')
base = hc.summarize(good_set(dx=0.1, dyaw=0.1), block_xy_mm=[0.0, 0.0],
                    block_top_mm=100.0, hover_mm=50.0)
check('好数据 → ok', base['ok'], repr(base['reasons']))
check('可用率 15/20 = 0.75', abs(base['usable_ratio'] - 0.75) < 1e-9,
      repr(base['usable_ratio']))
check('中心半极差 14·0.1/2 = 0.7 mm',
      abs(base['center_half_span_mm'] - 0.7) < 1e-6,
      repr(base['center_half_span_mm']))
check('yaw 极差 14·0.1 = 1.4°', abs(base['yaw_color_span_deg'] - 1.4) < 1e-6,
      repr(base['yaw_color_span_deg']))
check('tool0_z = 100 + 204.2 + 50', abs(base['tool0_z_mm'] - 354.2) < 1e-6,
      repr(base.get('tool0_z_mm')))
check('offset_from_block 记中位数', base['offset_from_block_mm'] == [0.7, 0.0],
      repr(base.get('offset_from_block_mm')))

small = hc.summarize(good_set(n_usable=15, dx=0.1, dyaw=0.1))
check('小跨度 → ok', small['ok'] and small['yaw_color_span_deg'] <= 2.0,
      repr(small['reasons']))

bad_ratio = hc.summarize([sample()] * 10 + [sample(usable=False)] * 10)
check('可用率 50% → 判红', not bad_ratio['ok'] and
      any('可用率' in r for r in bad_ratio['reasons']), repr(bad_ratio['reasons']))

bad_center = hc.summarize(good_set(dx=1.0))
check('中心半极差 7 mm → 判红', not bad_center['ok'] and
      any('中心半极差' in r for r in bad_center['reasons']),
      repr(bad_center['reasons']))

bad_yaw = hc.summarize(good_set(dyaw=1.0))
check('yaw 极差 14° → 判红', not bad_yaw['ok'] and
      any('yaw 极差' in r for r in bad_yaw['reasons']), repr(bad_yaw['reasons']))

no_yaw = hc.summarize([sample(yaw=None, trusted=False) for _ in range(15)])
check('没有可信 yaw → 判红（不能算通过）', not no_yaw['ok'] and
      any('yaw' in r for r in no_yaw['reasons']), repr(no_yaw['reasons']))

empty = hc.summarize([])
check('零帧 → 判红', not empty['ok'] and '没有帧' in empty['reasons'],
      repr(empty['reasons']))

# 变异 1：把可用率阈值改到 0.8，同一份 75% 数据必须翻红
_old = hc.MIN_USABLE_RATIO
hc.MIN_USABLE_RATIO = 0.8
mut1 = hc.summarize(good_set(dx=0.1, dyaw=0.1))
hc.MIN_USABLE_RATIO = _old
check('变异：阈值 0.8 ⇒ 75% 翻红（阈值真的被查）', not mut1['ok'],
      repr(mut1['reasons']))

# 变异 2：把中心阈值收到 0.5 mm，同一份 1 mm 数据必须翻红
_old = hc.MAX_CENTER_HALF_SPAN_MM
hc.MAX_CENTER_HALF_SPAN_MM = 0.05
mut2 = hc.summarize(good_set(dx=0.1, dyaw=0.1))
hc.MAX_CENTER_HALF_SPAN_MM = _old
check('变异：中心阈值 0.05 ⇒ 0.7 mm 翻红', not mut2['ok'], repr(mut2['reasons']))

# 变异 3：把 yaw 阈值收到 0.1°，同一份 1.4° 数据必须翻红
_old = hc.MAX_YAW_SPAN_DEG
hc.MAX_YAW_SPAN_DEG = 0.1
mut3 = hc.summarize(good_set(dx=0.1, dyaw=0.1))
hc.MAX_YAW_SPAN_DEG = _old
check('变异：yaw 阈值 0.1 ⇒ 1.4° 翻红', not mut3['ok'], repr(mut3['reasons']))

# =====================================================================
print('--- C. 悬停几何：0.2042 而不是 0.1757 ---')
_old = hc.TOOL0_TO_BARE_FINGERTIP_M
hc.TOOL0_TO_BARE_FINGERTIP_M = 0.1757
wrong = hc.summarize(good_set(dx=0.1, dyaw=0.1), block_top_mm=100.0, hover_mm=50.0)
hc.TOOL0_TO_BARE_FINGERTIP_M = _old
check('用 0.1757 ⇒ tool0_z 会低 28.5 mm',
      abs((wrong['tool0_z_mm'] - 354.2) + 28.5) < 1e-6,
      repr(wrong.get('tool0_z_mm')))
check('源码常量为 0.2042', abs(hc.TOOL0_TO_BARE_FINGERTIP_M - 0.2042) < 1e-12)

# =====================================================================
print('--- D. run()：高度配对 ---')
_h = hc.check_height
seen = []


def fake_check(d, h, block_xy_mm=None, block_top_mm=None, **kw):
    seen.append((d, h))
    out = {'hover_mm': h, 'ok': True, 'reasons': [], 'n_frames': 1,
           'n_usable': 1, 'usable_ratio': 1.0, 'center_half_span_mm': 0.1,
           'yaw_color_span_deg': 0.1, 'capture': d, 'frames': []}
    if block_top_mm is not None:
        out['tool0_z_mm'] = (block_top_mm
                             + hc.TOOL0_TO_BARE_FINGERTIP_M * 1000.0 + h)
    return out


hc.check_height = fake_check
r_ok = hc.run([(50.0, '/a'), (65.0, '/b'), (80.0, '/c')])
check('三个高度齐全 → ok', r_ok['ok'] and len(r_ok['heights']) == 3,
      repr(r_ok['reasons']))
check('按 50/65/80 顺序判', [h['hover_mm'] for h in r_ok['heights']] ==
      [50.0, 65.0, 80.0])

r_miss = hc.run([(50.0, '/a'), (80.0, '/c')])
check('少一个高度 → 判红（数据缺失）', not r_miss['ok'] and
      any('65' in r for r in r_miss['reasons']), repr(r_miss['reasons']))

r_dup = hc.run([(50.0, '/a'), (50.0, '/a2'), (65.0, '/b'), (80.0, '/c')])
check('同一高度给两个目录 → 判红', not r_dup['ok'] and
      any('正好 1 个' in r for r in r_dup['reasons']), repr(r_dup['reasons']))

r_extra = hc.run([(50.0, '/a'), (65.0, '/b'), (80.0, '/c'), (100.0, '/d')])
check('多给不判的高度 → 判红', not r_extra['ok'] and
      any('多给了' in r for r in r_extra['reasons']), repr(r_extra['reasons']))

run_j = hc.run([(50.0, '/a'), (65.0, '/b'), (80.0, '/c')], block_top_mm=100.0)
check('run 把 block_top 传给每层，tool0_z 都记上',
      [h.get('tool0_z_mm') for h in run_j['heights']] == [354.2, 369.2, 384.2],
      repr([h.get('tool0_z_mm') for h in run_j['heights']]))
hc.check_height = _h

# =====================================================================
print('--- E. check_height：坏帧计入不可用，不中断 ---')
with tempfile.TemporaryDirectory() as td:
    for i in range(6):
        open(os.path.join(td, 'color_%03d.png' % i), 'w').close()

    _li, _mf = hc.load_indexed_frame, hc.sm.measure_frame
    calls = []

    def fake_load(d, idx):
        calls.append(idx)
        return {'fake_index': idx}

    def fake_measure(frame_dir, **kw):
        i = kw['frame']['fake_index']
        if i in (1, 2, 3):
            raise RuntimeError('坏帧')
        return sample(x=float(i), y=0.0, yaw=10.0 + i)

    hc.load_indexed_frame, hc.sm.measure_frame = fake_load, fake_measure
    try:
        res = hc.check_height(td, 50.0, [0.0, 0.0])
    finally:
        hc.load_indexed_frame, hc.sm.measure_frame = _li, _mf
    check('六帧都尝试', calls == [0, 1, 2, 3, 4, 5], repr(calls))
    check('坏帧让可用率降到 3/6', res['n_usable'] == 3 and
          abs(res['usable_ratio'] - 0.5) < 1e-9, repr(res))
    check('坏帧在 frames 里留 error', res['frames'][1].get('error') and
          not res['frames'][1]['usable'], repr(res['frames'][1]))
    check('3/6 可用 < 60% ⇒ 判红', not res['ok'], repr(res['reasons']))

# 目录不存在 ⇒ ValueError（main 会转成退出码 2）
try:
    hc.check_height('/no/such/dir', 50.0, [0.0, 0.0])
    raised = False
    msg = ''
except ValueError as e:
    raised, msg = True, str(e)
check('目录不存在 ⇒ ValueError', raised and '不存在' in msg, msg)

# =====================================================================
print('--- F. main()：退出码 ---')
rc = hc.main(['--capture', '50:/no/such/dir', '--capture', '65:/also/no',
              '--capture', '80:/no', '--block-xy-mm', '0,0'])
check('目录不存在 ⇒ 退出码 2', rc == 2, 'rc=%r' % rc)
rc = hc.main(['--capture', 'x'])
check('格式错（缺冒号）⇒ 退出码 2', rc == 2, 'rc=%r' % rc)
with tempfile.TemporaryDirectory() as td:
    for i in range(3):
        open(os.path.join(td, 'color_%03d.png' % i), 'w').close()
    _cp = hc.check_height
    hc.check_height = lambda d, h, block_xy_mm=None, block_top_mm=None, **kw: {
        'hover_mm': h, 'ok': True, 'reasons': [], 'n_frames': 1,
        'n_usable': 1, 'usable_ratio': 1.0, 'center_half_span_mm': 0.1,
        'yaw_color_span_deg': 0.1, 'capture': d, 'frames': []}
    try:
        rc = hc.main(['--capture', '50:' + td, '--capture', '65:' + td])
    finally:
        hc.check_height = _cp
    check('少一个高度 ⇒ 退出码 2（数据缺失，不是判红）', rc == 2, 'rc=%r' % rc)

# =====================================================================
print()
print('通过 %d，失败 %d' % (N_PASS, N_FAIL))
if FAILS:
    print('失败项：')
    for f in FAILS:
        print('  - %s' % f)
sys.exit(0 if N_FAIL == 0 else 1)
