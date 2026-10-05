#!/usr/bin/env python3
"""类别→槽位路由的自检。**不需要 ROS、不需要 MoveIt、不需要 colcon build**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
分拣里「哪一类进哪一格」错了**不会有任何报错**：机械臂动作正常、日志正常、
收工正常，只有录像里那一格颜色不对。所以这段判据的正确性只能靠断言，
而断言本身是不是真的在盯着那行代码，只能靠**变异**来证明。

末尾三段变异把 ``slot_routing.py`` 的三个关键判据各改坏一处，然后断言
「改坏之后，前面那些断言会失败」。不做这一步的话，一个恒真的断言
（比如把 ``pick_slot`` 的返回值接错）也能全绿通过。
"""
import importlib.util
import os
import sys
import tempfile

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC = os.path.join(REPO, "ros2_ws/src/grasp_planner/grasp_planner/slot_routing.py")

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


def load(src_text, tag):
    """把一份源码文本当模块导入（用临时文件，避免与真模块的 .pyc 混）。"""
    d = tempfile.mkdtemp(prefix='slotroute_')
    path = os.path.join(d, 'slot_routing_%s.py' % tag)
    with open(path, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('slot_routing_%s' % tag, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate(old, new):
    """对真源码做一次文本替换；替换没命中就抛（防止「变异其实没生效」）。"""
    s = open(SRC).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


# ----------------------------------------------------------------------
# 被断言的行为：一组固定输入 → 一组固定输出
# ----------------------------------------------------------------------
ROUTED = ['red', 'red', 'yellow', 'yellow', '']       # 末位是通配
SLOTS = [(0.4, 0.2), (0.6, 0.2), (0.4, -0.2), (0.6, -0.2), (0.8, 0.0)]


def behaviour(m):
    """把模块的关键行为压成一部字典，方便逐个字段比对。"""
    used_none = [False] * 5
    used_0 = [True, False, False, False, False]
    used_2 = [False, False, True, False, False]
    return {
        # 第一格给 red
        'red_first': m.pick_slot(used_none, ROUTED, 'red'),
        # 0 号被占 → red 拿 1 号
        'red_skips_used': m.pick_slot(used_0, ROUTED, 'red'),
        # yellow 绝不能拿到 0/1 号（那是 red 的）
        'yellow_avoid_red': m.pick_slot(used_none, ROUTED, 'yellow'),
        # red 的两格都占了 → 只能退到通配的 4 号
        'red_falls_back_to_wildcard': m.pick_slot([True, True, False, False, False],
                                                  ROUTED, 'red'),
        # 通配被占、red 满 → None
        'red_exhausted': m.pick_slot([True, True, False, False, True],
                                     ROUTED, 'red'),
        # 没见过的类别 → 只有通配能接
        'unknown_only_wildcard': m.pick_slot(used_none, ROUTED, 'cyan'),
        # 空类别名（检测器没给名字）→ 同样只有通配
        'empty_name_only_wildcard': m.pick_slot(used_none, ROUTED, ''),
        # 通配也占了 → None
        'unknown_none': m.pick_slot([False, False, False, False, True],
                                    ROUTED, 'cyan'),
        # 不路由时行为与 A1 一致：永远取第一个未占用的
        'unrouted_seq': [m.pick_slot([False] * 3, ['', '', ''], c)
                         for c in ('red', '', 'cyan')],
        # 只读：pick_slot 不能改动 used
        'used_untouched': (lambda u: (m.pick_slot(u, ROUTED, 'red'), u)[1])(
            [False] * 5),
        # 长度对不上 → None，而不是 IndexError
        'length_mismatch': m.pick_slot([False] * 3, ROUTED, 'red'),
        # 只判别「还有空位」的辅助
        # red 自己的两格 + 通配都占了 → 放不下
        'has_free_red_full': m.has_free_slot([True, True, False, False, True],
                                             ROUTED, 'red'),
        # red 自己的两格占了，但通配还空着 → 通配能接，所以还放得下
        'has_free_via_wildcard': m.has_free_slot([True, True, True, True, False],
                                                 ROUTED, 'red'),
        'is_routed': (m.is_routed(ROUTED), m.is_routed(['', '']),
                      m.is_routed([]), m.is_routed(None)),
        'hist': m.class_histogram(ROUTED),
    }


EXPECT = {
    'red_first': 0,
    'red_skips_used': 1,
    'yellow_avoid_red': 2,
    'red_falls_back_to_wildcard': 4,
    'red_exhausted': None,
    'unknown_only_wildcard': 4,
    'empty_name_only_wildcard': 4,
    'unknown_none': None,
    'unrouted_seq': [0, 0, 0],
    'used_untouched': [False] * 5,
    'length_mismatch': None,
    'has_free_red_full': False,
    'has_free_via_wildcard': True,
    'is_routed': (True, False, False, False),
    'hist': {'red': 2, 'yellow': 2, '': 1},
}


def main():
    src = open(SRC).read()
    m = load(src, 'real')

    # ------------------------------------------------ [1] 正常行为
    print('\n[1] 路由判据（5 个槽位：red red yellow yellow 通配）')
    got = behaviour(m)
    for k, want in EXPECT.items():
        check(k, got[k] == want, '期望 %r，实际 %r' % (want, got[k]))

    # ------------------------------------------------ [2] 校验
    print('\n[2] validate_slot_classes —— 长度错必须拒绝启动，不能截断')
    # 归一化成「全通配」而不是空列表 —— 空列表会让 pick_slot 全返回 None
    # （= 把默认的单类别路径弄坏，见 validate_slot_classes 的注释）
    check('空列表 = 不路由 → 归一化成全通配',
          m.validate_slot_classes([], 5) == [''] * 5,
          str(m.validate_slot_classes([], 5)))
    check('None = 不路由 → 归一化成全通配',
          m.validate_slot_classes(None, 5) == [''] * 5)
    check('首尾空白被去掉', m.validate_slot_classes([' red ', 'yellow'], 2)
          == ['red', 'yellow'])
    check('None 项当空串（通配）', m.validate_slot_classes(['red', None], 2)
          == ['red', ''])
    for bad, why in ((['red'], '少一项'), (['red'] * 6, '多一项'),
                     (['red', 'red', 'red'], '只有三项')):
        try:
            m.validate_slot_classes(bad, 5)
            check('长度不符（%s）→ 抛 ValueError' % why, False, '没有抛')
        except ValueError as e:
            check('长度不符（%s）→ 抛 ValueError' % why,
                  '等长' in str(e), str(e)[:60])

    # ------------------------------------------------ [3] 启动日志
    print('\n[3] 路由表描述 / 容量直方图')
    d = m.describe_routing(ROUTED, SLOTS)
    check('描述里有每个下标与类别名', '0:' in d and 'red' in d and 'yellow' in d, d)
    check('通配槽位打成 *', d.rstrip().endswith('*'), d)
    check('坐标进了描述（现场对表用）', '(0.40,0.20)' in d, d)
    check('不路由时明说不分', '不按类别分' in m.describe_routing([]),
          m.describe_routing([]))

    # ================================================ [4] 变异
    # 只有「改坏判据 → 断言会失败」这一步，才能证明上面那些断言
    # 真的盯住了那行代码，而不是碰巧恒真。
    print('\n[4] 变异：把判据改坏，前面的断言必须失效')
    mutations = [
        ('类别不比较（谁都放进第一格）',
         'if not want or (class_name and want == class_name):',
         'if not want or True:',
         'yellow_avoid_red'),
        ('不看占用（同一个槽位反复用）',
         '        if used[i]:\n            continue\n',
         '        if False:\n            continue\n',
         'red_skips_used'),
        ('通配失效（空串槽位谁都不给）',
         'if not want or (class_name and want == class_name):',
         'if class_name and want == class_name:',
         'unknown_only_wildcard'),
        ('不校验长度（截断代替报错）',
         '    if len(vals) != n_slots:',
         '    if False:',
         '__validate__'),
    ]
    for label, old, new, field in mutations:
        mm = load(mutate(old, new), 'mut%d' % len(FAILS))
        if field == '__validate__':
            try:
                mm.validate_slot_classes(['red'], 5)
                broke = True        # 没抛 = 变异生效（原本该抛）
            except ValueError:
                broke = False
            check('变异「%s」→ 长度校验失效' % label, broke,
                  '改坏之后居然还是抛错，变异没生效')
            continue
        bad = behaviour(mm)
        check('变异「%s」→ %s 与期望不符（断言确实盯着它）' % (label, field),
              bad[field] != EXPECT[field],
              '改坏后仍是 %r —— 说明这条断言根本没盯着那行' % (bad[field],))

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
