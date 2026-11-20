#!/usr/bin/env python3
"""探针 ``--cells`` 的接线自检（只查源码结构 + 纯函数，不连 ROS）。

为什么不直接 import 探针：它在模块顶层 ``import rclpy`` 与一堆 moveit 消息，
单测里既起不了 ROS 也不该起。这里做两件事：

1. **接线**：用 AST 查 ``--cells`` 真的接到了 argparse、每个 attempt 真的记了
   ``cell`` / ``seconds``、报告真的带 ``cells``。这些点删掉不会报错，只会让
   正式轮悄悄退回「全部进垫心」—— 探针照绿、计划全撞格。
2. **语义**：格位映射本身在 ``stage4_clear_model.assign_cells``，这里按探针
   的调用形状（truth 块 id 升序 + ``parse_cells``）复核一遍。

跑法：python3 scripts/test_stage4_clear_probe.py
"""
import ast
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

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


def _parse():
    path = os.path.join(HERE, 'stage4_clear_probe.py')
    with open(path, encoding='utf-8') as f:
        return ast.parse(f.read(), path), open(path, encoding='utf-8').read()


def _main_fn(tree):
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            return node
    raise AssertionError('main() 不见了')


def _opt_strings(call):
    """``ap.add_argument(...)`` 的位置参数字符串。"""
    out = []
    for a in call.args:
        if isinstance(a, ast.Constant) and isinstance(a.value, str):
            out.append(a.value)
    return out


def section_wiring():
    print('\n[1] 探针接线：--cells → 逐目标 cell → attempt.cell/seconds → 报告 cells')
    tree, src = _parse()
    main = _main_fn(tree)

    opts = [_opt_strings(n) for n in ast.walk(main)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'add_argument']
    flats = [o for o in opts if o]
    check('argparse 有 --cells', any('--cells' in o for o in flats),
          str([o[0] for o in flats if o]))
    check('argparse 保留 --cell（旧调用兼容，作回退值）',
          any('--cell' in o for o in flats), str([o[:1] for o in flats]))

    check('格位由 assign_cells 唯一给出（不是探针现编）',
          'cm.assign_cells' in src and 'cm.parse_cells' in src)
    check('逐目标取自己的格，缺省回退 --cell',
          '.get(str(tid), args.cell)' in src)
    check('每个 attempt 记 cell', 'a["cell"] = cell' in src)
    check('每个 attempt 记 seconds（Round 0 标定要用）',
          'a["seconds"] = round(time.time() - t0, 2)' in src)
    check('报告带 cells 映射', '"cells": cells_report or None' in src)
    check('场景级也记 cells', '"cells": cell_map' in src)


def section_semantics():
    print('\n[2] 探针形状的格位语义（truth 块 id 升序 → 8,6,4,2,0）')
    truth_ids = ['05', '03', '01', '04', '02']
    m = cm.assign_cells(truth_ids, cm.parse_cells('8,6,4,2,0'))
    check('5 块 → 五格各一、不重不漏',
          sorted(m.values()) == [0, 2, 4, 6, 8], str(m))
    check('id 最小的块去最远的 8 号格', m['01'] == 8, str(m))
    check('同一布局两次重复映射完全相同',
          cm.assign_cells(list(reversed(truth_ids)),
                          cm.parse_cells('8,6,4,2,0')) == m)
    # 探针缺省 --cells 时全部进 --cell（默认 4 = 垫心）：旧行为不变。
    _, src = _parse()
    check('不给 --cells ⇒ cell_map 为 None ⇒ 逐目标回退 args.cell',
          'if args.cells else None' in src)


def section_cumulative_wiring():
    print('\n[3] 累积清障接线（S3，2026-11-20）')
    tree, src = _parse()
    main = _main_fn(tree)
    opts = [_opt_strings(n) for n in ast.walk(main)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == 'add_argument']
    flats = [o for o in opts if o]
    for flag in ('--cumulative', '--dest-mode', '--assign-by',
                 '--target-order'):
        check('argparse 有 %s' % flag,
              any(flag in o for o in flats), str([o[:1] for o in flats]))
    for name in ('def run_scene_cumulative', 'def find_cell_destination',
                 'def scene_fingerprint'):
        check('源码有 %s' % name, name in src)
    for call in ('cm.assign_cells_by_colour', 'cm.grid_order_of',
                 'cm.validate_pick_counts', 'cm.check_attempt_coverage',
                 'cm.release_sweep_conflicts', 'cm.scene_fingerprint'):
        check('源码调用 %s（单一出处）' % call, call in src)
    check('主循环按 --cumulative 分支', 'if args.cumulative:' in src)
    for key in ('"mode": "cumulative"', '"dest_mode": dest_mode',
                '"picked": picked', '"batch_final": rep',
                '"scene_fingerprint": fp'):
        check('报告带 %s' % key, key in src)


def main():
    print('=' * 68)
    section_wiring()
    section_semantics()
    section_cumulative_wiring()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
