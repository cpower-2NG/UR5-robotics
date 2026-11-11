#!/usr/bin/env python3
"""V4 分析自检：步数分类、侧向下界、叠放依赖与不可区分判定。

跑法：
    python3 scripts/test_stage4_v4_occlusion.py
"""
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_v4_occlusion as v4                                  # noqa: E402

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
        print('  FAIL  %s%s' % (name, '   ← %s' % (detail,) if detail else ''))


def step(move, z):
    return {"move": move, "from": [0.4, 0.3, z], "to": [0.5, 0.5, 0.015]}


def attempt(bid, steps, b_ok, rows):
    return {"id": bid, "clearing_steps": steps, "initial_b_ok": b_ok,
            "initial_failed_rows": rows,
            "terminal": {"ok": True}, "ok": True, "reason": None}


def row(verdict, per):
    return {"axis": 2, "sign": -1, "closing": 0, "variant": 0,
            "verdict": verdict, "per_other": per}


def section_classify():
    print('\n[1] 步数分类')
    check('层 1 的块 = 叠放依赖',
          v4.classify_step(step(8, 0.044)) == "stack")
    check('层 0 的块 = 侧向清障',
          v4.classify_step(step(3, 0.015)) == "lateral")


def section_bound():
    print('\n[2] 侧向下界')
    rows = [row("gripper_collide", {"1": 0.02, "2": -0.001, "3": 0.01}),
            row("gripper_collide", {"1": -0.002, "2": -0.003})]
    check('取最小非空遮挡集 = 1', v4.lateral_lower_bound(rows) == 1)
    check('没有接触 = 0', v4.lateral_lower_bound(
        [row("blocked_arm", {"1": 0.01})]) == 0)
    check('空输入 = 0', v4.lateral_lower_bound([]) == 0)


def section_attempt():
    print('\n[3] 单次尝试')
    a = attempt(0, [step(8, 0.044)], True,
                [row("gripper_collide", {"8": -0.001})])
    r = v4.analyse_attempt(a)
    check('初始可抓 ⇒ 侧向不计入下界', r["lower_bound"] == 1, r)
    check('叠放步计数', r["n_stack_steps"] == 1 and r["n_lateral_steps"] == 0)
    a = attempt(1, [step(3, 0.015)], False,
                [row("gripper_collide", {"3": -0.001})])
    r = v4.analyse_attempt(a)
    check('初始不可抓 ⇒ 侧向下界 1', r["lower_bound"] == 1, r)
    # 回归：阻塞集合里的堆叠块不得重复计数（S4:1 目标 1 实测虚高一格）。
    a = attempt(1, [step(8, 0.044), step(0, 0.015)], False,
                [row("gripper_collide", {"0": 0.0, "8": 0.0})])
    r = v4.analyse_attempt(a)
    check('堆叠块不重复计入侧向下界', r["lower_bound"] == 2, r)
    check('下界不超过贪心步数', r["lower_bound"] <= r["greedy_steps"], r)


def section_report():
    print('\n[4] 报告汇总')
    d = {"scenes": [{"batch": "S4", "index": 0, "truth": "x", "attempts": [
        attempt(0, [], True, []),
        attempt(1, [step(3, 0.015)], False,
                [row("gripper_collide", {"3": -0.001})]),
    ]}]}
    import json
    import tempfile
    p = os.path.join(tempfile.mkdtemp(), "r.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(d, f)
    r = v4.analyse_report(p)
    check('2 个尝试', r["n"] == 2)
    check('贪心总步数 1', r["greedy_steps_total"] == 1)
    check('下界总步数 1', r["lower_bound_total"] == 1)
    check('1 个侧向决策点', r["n_lateral_decision_points"] == 1)
    check('全桌合法率 100%', r["whole_table_legal_rate"] == 1.0)


def main():
    for fn in (section_classify, section_bound, section_attempt,
               section_report):
        fn()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：%s' % ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
