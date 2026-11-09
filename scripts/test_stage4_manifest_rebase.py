#!/usr/bin/env python3
"""判分基线重写（``stage4_manifest_rebase``）的离线自检。

钉住的是**判分口径**，不是几何：这一层唯一能犯的错是「把不该重写的重写了」
或「把两块配错了」。三条都真实可发生：

1. 被搬动的块也重写 ⇒ 它的期望值从「计划落格」变成「跑前测量」，
   ``tol_moved_mm`` 那一判就失去了意义；
2. 塔的两层 xy 相同、只靠 z 分开 ⇒ 不做一对一约束时两层会**抢同一个基线块**，
   位移被判成 0，塔的错位被掩盖；
3. 基线其实不是同一个场景（场景被重摆过）⇒ 不报错就会拿别人的位置当基线。

跑法：``python3 scripts/test_stage4_manifest_rebase.py``；退出码 0 = 全通过。
"""
import inspect
import sys

sys.dont_write_bytecode = True

import stage4_manifest_rebase as rb                             # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('   ← %s' % (detail,)) if detail else ''))


def raises(fn, *a, **kw):
    try:
        fn(*a, **kw)
    except ValueError as e:
        return str(e)
    return None


# ---------------------------------------------------------------- 夹具
def manif():
    """三块：0/1 是一座塔（同 xy、z 差 29.5 mm），2 被搬动。"""
    return {
        'actions': [{'block': '2', 'kind': 'clear'}],
        'final_layout': [
            {'id': 0, 'x': 0.4400, 'y': 0.3430, 'z': 0.01475,
             'yaw': 1.5, 'size': 0.0295},
            {'id': 1, 'x': 0.4400, 'y': 0.3430, 'z': 0.04425,
             'yaw': 1.5, 'size': 0.0295},
            {'id': 2, 'x': 0.3840, 'y': -0.1120, 'z': 0.01475,
             'yaw': 0.0, 'size': 0.0295},
        ],
    }


def baseline(shift=(0.0, -0.028), z_ok=True):
    """俯视跑前真值：id 顺序与计划不同（真实的），并带一个 28 mm 的系统偏差。

    ``z_ok=False`` 模拟「俯视把 2 层塔读成单块」：两块都落在下层高度。
    """
    dx, dy = shift
    ztop = 0.04425 if z_ok else 0.01475
    return {'blocks': [
        {'id': 7, 'color': 'orange', 'x': 0.3840 + dx + 0.020,
         'y': -0.0850 + dy, 'z': 0.01475, 'yaw': 0.1},
        {'id': 5, 'color': 'yellowgreen', 'x': 0.4400 + dx,
         'y': 0.3430 + dy + 0.018, 'z': ztop, 'yaw': 1.4},
        {'id': 6, 'color': 'yellowgreen', 'x': 0.4400 + dx,
         'y': 0.3430 + dy, 'z': 0.01475, 'yaw': 1.4},
    ]}


# ---------------------------------------------------------------- 用例
def section_moved():
    print('\n[1] 被搬动的块不重写，没被搬的换成基线实测')
    out, rep = rb.rebase_final_layout(manif(), baseline())
    lay = {int(e['id']): e for e in out['final_layout']}
    check('被搬动的 #2 期望位置原样（仍是计划落格）',
          abs(lay[2]['x'] - 0.3840) < 1e-9 and abs(lay[2]['y'] + 0.1120) < 1e-9,
          lay[2])
    check('#0/#1 的 y 换成了基线实测（腕部 0.3430 → 基线 0.3150/0.3330）',
          abs(lay[0]['y'] - (0.3430 - 0.028)) < 1e-9
          and abs(lay[1]['y'] - (0.3430 - 0.028 + 0.018)) < 1e-9,
          (lay[0]['y'], lay[1]['y']))
    check('id/yaw/z/size 保留腕部值（俯视 yaw 不可用）',
          lay[0]['id'] == 0 and lay[0]['yaw'] == 1.5
          and lay[0]['z'] == 0.01475 and lay[0]['size'] == 0.0295)
    check('报告只列重写的 2 块，并点名被搬动的',
          rep['rebased'] == 2 and rep['moved_kept'] == ['2'], rep)


def section_tower():
    print('\n[2] 塔的上下层不能互相顶替（一对一）')
    out, rep = rb.rebase_final_layout(manif(), baseline())
    lay = {int(e['id']): e for e in out['final_layout']}
    check('下层 #0 取基线里同样在 z=14.75 mm 的那块（y=0.3150）',
          abs(lay[0]['y'] - 0.3150) < 1e-9, lay[0]['y'])
    check('上层 #1 取 z=44.25 mm 的那块（y=0.3330）',
          abs(lay[1]['y'] - 0.3330) < 1e-9, lay[1]['y'])
    ds = [r['dz_mm'] for r in rep['rows']]
    check('两条的 |Δz| 都是 0（没有跨层错配）',
          all(abs(v) < 1e-6 for v in ds), ds)


def section_guard():
    print('\n[3] 基线不是同一个场景 ⇒ 报错，不静默保留')
    e = raises(rb.rebase_final_layout, manif(), baseline(shift=(0.0, -0.120)))
    check('错位 120 mm 被拒', e is not None and '相距' in e, e)
    e = raises(rb.rebase_final_layout, manif(), baseline(z_ok=False))
    check('层数结构不一致（z 差 29.5 mm）被拒',
          e is not None and '层数结构' in e, e)
    e = raises(rb.rebase_final_layout, manif(), {'blocks': []})
    check('基线空 ⇒ 报错', e is not None, e)


def section_purity():
    print('\n[4] 不改入参（可重复调用 / --check 安全）')
    m = manif()
    before = [dict(e) for e in m['final_layout']]
    rb.rebase_final_layout(m, baseline())
    check('入参 manifest 未被就地改写', m['final_layout'] == before)


def section_mutation():
    print('\n[5] 变异：拆掉 z 结构闸 ⇒ 用例 [3] 必须红')
    src = inspect.getsource(rb)
    old = "        if abs(dz) > max_z_mm:"
    check('变异锚点存在', old in src)
    ns = {'__name__': 'mut'}
    exec(compile(src.replace(old, "        if False:", 1),
                 '<mut>', 'exec'), ns)
    try:
        out, rep = ns['rebase_final_layout'](manif(), baseline(z_ok=False))
    except ValueError as e:
        check('拆掉 z 闸后仍报错（用例 [3] 没有承重）', False, e)
        return
    check('拆掉 z 闸后跨层错配被静默接受（证用例 [3] 是承重的）',
          rep['max_dz_mm'] > 20.0, rep['max_dz_mm'])


def main():
    print('=' * 68)
    for fn in (section_moved, section_tower, section_guard, section_purity,
               section_mutation):
        try:
            fn()
        except Exception as e:                            # noqa: BLE001
            import traceback
            traceback.print_exc()
            check('%s 未抛异常' % fn.__name__, False, repr(e))
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
