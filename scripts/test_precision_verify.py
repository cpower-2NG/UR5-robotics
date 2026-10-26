#!/usr/bin/env python3
"""验收报告判据的自检。**不需要 ROS、不需要真机、不需要 ultralytics**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
这一轮报告初稿把**整轮都跑绿的轮次**印成了 ``轨迹 / 闸门 / 无滑移 | False / False /
False``，把**根本没放过块**的轮次印成「YOLO未检出」，还把一个没放满的轮次写成
「收工：槽位用尽」。三处都不报错、都是「看起来像结论」的错误说法，只能靠断言盯住：

* ``_class_status`` 的 ``not_measured`` 必须**压过** ``not_run`` / ``yolo_missing``
  —— 「没放成」跟「YOLO 没检出」是两件事，后者会被读成检测器的锅；
* 执行汇总的分母是 ``n_moved``（真发过轨迹的轮次），不是 ``len(rows)``；
* 收工原因只照 ``round.json`` 写的；没写就照实说还有几个槽空着。

末尾三段变异各改坏一处，断言「改坏之后上面那些断言会红」——证明断言真的盯着
那几行，而不是刚好都能通过。
"""
import importlib.util
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "precision_verify.py")

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
    d = tempfile.mkdtemp(prefix='pverify_')
    path = os.path.join(d, 'precision_verify_%s.py' % tag)
    with open(path, 'w') as f:
        f.write(src_text)
    # 模块里 sys.path.insert 的是它**自己**所在目录；替身文件在临时目录，
    # 得把真目录也放进去，否则 import block_classes / graspnet_candidates 找不到。
    sys.path.insert(0, HERE)
    spec = importlib.util.spec_from_file_location('precision_verify_%s' % tag, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate(old, new):
    s = open(SRC).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


def write_run(tmp, used, stop_reason, n_cycles, exec_log_cycle=None):
    """造一个最小 run 目录：只有 round.json 与一份总绿的 exec.jsonl。

    不放帧 —— 帧要真深度图，属于 `precision_verify --frame` 那支的事；
    这里只钉聚合口径（计数、类别状态、收工原因）。
    """
    os.makedirs(tmp, exist_ok=True)
    cycles = []
    for i in range(1, n_cycles + 1):
        cyc = {"n": i, "ok": False, "block_idx": i - 1, "class": "red",
               "slot_idx": min(i - 1, len(used) - 1),
               "exec_log": None, "place_frame": None}
        if exec_log_cycle == i:
            rel = "cycles/%02d/exec.jsonl" % i
            os.makedirs(os.path.join(tmp, "cycles", "%02d" % i), exist_ok=True)
            recs = [{"event": "gate", "ok": True, "where": "approach"},
                    {"event": "traj", "ok": True, "where": "approach"},
                    {"event": "traj", "ok": True, "where": "grasp"},
                    {"event": "traj", "ok": True, "where": "lift"},
                    {"event": "arrive", "err_deg": 0.01},
                    {"event": "gripper", "action": "close", "where": "grasp.after",
                     "position_mm": 25.0},
                    {"event": "grasp_check", "lifted_mm": 25.0, "slipped": False}]
            with open(os.path.join(tmp, rel), "w") as f:
                for r in recs:
                    f.write(json.dumps(r) + "\n")
            cyc["exec_log"] = rel
        cycles.append(cyc)
    state = {"slots_mm": [[450.0, -250.0 + 100 * i] for i in range(len(used))],
             "slot_classes": [""] * len(used),
             "used": used, "stop_reason": stop_reason, "cycles": cycles}
    json.dump(state, open(os.path.join(tmp, "round.json"), "w"))
    return tmp


def main():
    src = open(SRC).read()
    m = load(src, 'real')

    # ------------------------------------------------ [1] 六态与优先级
    print('\n[1] _class_status 六态（没放成 ≠ YOLO 没检出）')
    cs = m._class_status
    check('没放成 → not_measured（哪怕 YOLO 也没跑）',
          cs(None, None, '', yolo_ran=False, measured=False) == 'not_measured',
          cs(None, None, '', yolo_ran=False, measured=False))
    check('放了但没给权重 → not_run',
          cs('red', None, '', yolo_ran=False, measured=True) == 'not_run')
    check('YOLO 没有这个块的框 → yolo_missing',
          cs('red', None, '', yolo_ran=True, measured=True) == 'yolo_missing')
    check('颜色表与 YOLO 不一致 → mismatch',
          cs('red', 'cyan', '', measured=True) == 'mismatch')
    check('颜色表与 YOLO 一致、槽位不路由 → ok',
          cs('red', 'red', '', measured=True) == 'ok')
    check('颜色表 = YOLO = 槽类别 → ok',
          cs('red', 'red', 'red', measured=True) == 'ok')
    check('颜色表 = YOLO 但槽不对 → wrong_slot',
          cs('red', 'red', 'cyan', measured=True) == 'wrong_slot')
    check('槽上没块时不给槽类别也不判错（不编造结论）',
          cs(None, None, 'cyan', yolo_ran=False, measured=False) == 'not_measured')

    # ------------------------------------------------ [2] 聚合口径
    print('\n[2] 整轮聚合：分母用「真发过轨迹的轮次」，收工原因照实说')
    d1 = write_run(tempfile.mkdtemp(prefix='run1_'), [True, True, False], None,
                   n_cycles=2, exec_log_cycle=2)
    r1 = m.build_report(d1, weights=None)
    ex = r1['acceptance']['exec']
    check('n_cycles = 2', ex['n_cycles'] == 2, ex['n_cycles'])
    check('n_moved = 1（第 1 轮没发轨迹，不算轨迹失败）', ex['n_moved'] == 1, ex['n_moved'])
    check('n_traj_ok = 1', ex['n_traj_ok'] == 1, ex['n_traj_ok'])
    check('n_gate_ok = 1', ex['n_gate_ok'] == 1, ex['n_gate_ok'])
    check('n_no_slip = 1', ex['n_no_slip'] == 1, ex['n_no_slip'])
    cl = r1['acceptance']['class']
    check('没放成的轮次进 not_measured（2 轮）', cl['not_measured'] == 2, cl['not_measured'])
    check('没放成的轮次不进 not_run / yolo_missing',
          cl['not_run'] == 0 and cl['yolo_missing'] == 0, cl)
    check('收工原因照实说还剩几个槽',
          r1['stop_reason'] == '未收工，1 个槽还空着：#2', r1['stop_reason'])
    md1 = m._md(r1)
    check('md 汇总行是计数而不是 False/False/False',
          '1/1 · 1/1 · 1/1 轮（共 2 轮，1 轮没发轨迹）' in md1,
          [l for l in md1.splitlines() if l.startswith('| 轨迹')])
    check('md 里没发轨迹的轮次写「没发轨迹」', '没发轨迹' in md1)
    check('md 里没放成的轮次写「没放成」而不是 YOLO未检出',
          '没放成' in md1 and 'YOLO未检出' not in md1)

    d2 = write_run(tempfile.mkdtemp(prefix='run2_'), [True, True, True], None, n_cycles=2)
    r2 = m.build_report(d2, weights=None)
    check('槽位全占 → 收工原因 = 槽位用尽', r2['stop_reason'] == '槽位用尽',
          r2['stop_reason'])

    d3 = write_run(tempfile.mkdtemp(prefix='run3_'), [True, False], '拾取区没有新的可抓块',
                   n_cycles=1)
    r3 = m.build_report(d3, weights=None)
    check('round.json 写了收工原因就用它', r3['stop_reason'] == '拾取区没有新的可抓块',
          r3['stop_reason'])

    # ------------------------------------------------ [3] 变异
    print('\n[3] 变异：把关键判据改坏，前面的断言必须变红')
    muts = [
        ('「没放成」不再单独报，退回 yolo_missing',
         '    if not measured:\n        return "not_measured"',
         '    if not measured:\n        return "yolo_missing"',
         lambda mm: mm._class_status(None, None, '', yolo_ran=False,
                                     measured=False) != 'not_measured'),
        ('分母改回 len(rows)',
         '"n_moved": sum(1 for r in rows if r["n_traj"] > 0)',
         '"n_moved": len(rows)',
         lambda mm: mm.build_report(write_run(tempfile.mkdtemp(prefix='mut_'),
                                              [True, True, False], None,
                                              n_cycles=2, exec_log_cycle=2),
                                    weights=None)['acceptance']['exec']['n_moved'] != 1),
        ('空着的槽被说成「槽位用尽」',
         '            stop = "未收工，%d 个槽还空着：%s" % (\n'
         '                len(free), ", ".join("#%d" % i for i in free))',
         '            stop = "槽位用尽"',
         lambda mm: mm.build_report(write_run(tempfile.mkdtemp(prefix='mut2_'),
                                              [True, False], None, n_cycles=1),
                                    weights=None)['stop_reason'] == '槽位用尽'),
    ]
    for i, (label, old, new, broke) in enumerate(muts):
        mm = load(mutate(old, new), 'mut%d' % i)
        check('变异「%s」→ 断言确实盯着它' % label, bool(broke(mm)),
              '改坏之后那条断言居然还是绿的')

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
