#!/usr/bin/env python3
"""`ik_repeat_check.py` 的纯函数自检（**不需要 ROS、不需要 move_group**）。

只测不取数的那一半：怎么把 N 次结果**归并成几个解**、怎么量「离种子多远」、
以及判词在三种情形下各说什么。

**真正的「解会不会变」不在这里测** —— 那要连一台活着的 move_group 问 20 次，
是这个工具存在的理由本身。这里只保证「数出来的东西是对的」：数错了，
结论就会从「分支翻转」变成「不可复现」或者反过来。

跑法：
    python3 scripts/test_ik_repeat.py
退出码 0 = 全通过。
"""

import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ik_repeat_check import spread, tally, verdict                  # noqa: E402

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


A = (0.0, -1.0, 2.0, -1.5, -1.5, 0.0)
B = (0.0, -2.0, 2.5, -1.0, -1.5, 0.0)

# ----------------------------------------------------------------------
# A. tally：归并
# ----------------------------------------------------------------------
print("\n--- A. tally ---")
check("同一个解重复 N 次 ⇒ 一个分支", tally([A] * 5) == [((0.0, -1.0, 2.0, -1.5, -1.5, 0.0), 5)])
check("两个解 ⇒ 两个分支，**多的在前**",
      tally([A, B, A, A]) == [((0.0, -1.0, 2.0, -1.5, -1.5, 0.0), 3),
                              ((0.0, -2.0, 2.5, -1.0, -1.5, 0.0), 1)])
check("★ 末位噪声**不许**被数成两个分支（0.7853981633974483 vs …85）",
      len(tally([(0.7853981633974483,), (0.7853981633974485,)])) == 1)
check("  上面那条不是空转：两个数在 3 位小数下确实相等、在浮点上不等",
      round(0.7853981633974483, 3) == round(0.7853981633974485, 3)
      and 0.7853981633974483 != 0.7853981633974485)
check("真分支翻转（差 ~1 rad）**要**被数成两个",
      len(tally([(1.0,), (2.0,)])) == 2)
check("None 单独一类，且**排在最后**",
      [k for k, _ in tally([None, A, None])] == [(0.0, -1.0, 2.0, -1.5, -1.5, 0.0), None])
check("全是 None ⇒ 只有一个条目", tally([None, None]) == [(None, 2)])
check("空输入 ⇒ 空（不抛）", tally([]) == [])
check("整数解也能收（不假定是 float）", tally([(1, 2)]) == [((1.0, 2.0), 1)])


# ----------------------------------------------------------------------
# B. spread：离种子多远
# ----------------------------------------------------------------------
print("\n--- B. spread ---")
check("同一个解 ⇒ 0", spread(A, A) == 0.0)
check("取的是**最大**的逐关节差，不是第一个", spread((0.0, 5.0, 0.0), (0.0, 1.0, 9.0)) == 9.0)
check("符号无关（取绝对值）：|-3 − 1| = 4，不是 −4 也不是 2",
      spread((0.0, -3.0, 0.0), (0.0, 1.0, 0.0)) == 4.0)
check("分支是 None ⇒ None（不是 0 —— 0 会被读成「一致」）", spread(None, A) is None)
check("种子是 None ⇒ None", spread(A, None) is None)
check("★ 长度不等 ⇒ None（不许 zip 截断到短的，那会给一个看着合理的数）",
      spread((1.0, 2.0), (1.0, 2.0, 3.0)) is None
      and spread((1.0, 2.0, 3.0), (1.0, 2.0)) is None)


# ----------------------------------------------------------------------
# C. verdict：三种情形各说什么
# ----------------------------------------------------------------------
print("\n--- C. verdict ---")
V_OK = verdict(tally([A] * 20), A)
check("★ 一个解 ⇒ 说「可复现」", "可复现" in V_OK and "❌" not in V_OK, f"got {V_OK!r}")
V_BAD = verdict(tally([A] * 7 + [B] * 3), A)
check("★ 两个解 ⇒ 说「不可复现」", "不可复现" in V_BAD, f"got {V_BAD!r}")
check("  并且点名是**肘分支翻转**（不是噪声）", "肘分支翻转" in V_BAD, f"got {V_BAD!r}")
check("  并且给出**读法**（留一格 0.05 m）", "0.05" in V_BAD and "一格" in V_BAD)
check("  并且说清**矩形仍然可用**（不能把整张图作废）",
      "最大全安全矩形" in V_BAD, f"got {V_BAD!r}")
check("  并且给**根治**（attempts / consistency_limits）",
      "attempts" in V_BAD and "consistency_limits" in V_BAD, f"got {V_BAD!r}")
check("  带上相差最大的那个分支的数（A→B 的最大关节差 = 1.0 rad）",
      "1.000 rad" in V_BAD, f"got {V_BAD!r}")
V_NONE = verdict(tally([None] * 5), A)
# ⚠️ 这里**不能**用 `"可复现" not in V_NONE` 来断言：判词里那句对比是
# 「这不是「可复现」，是够不到」，**字面上含着**「可复现」。用子串判会把一句
# 正确的话判红（第一次就是这么红的）。判**判词的头一个符号**才是这里该钉的契约：
# ❌ = 别照这张图划区，✅ = 可以照。
check("★ 一次都没解出来 ⇒ 判词是 ❌ 开头，且说「够不到」",
      V_NONE.startswith("❌") and "够不到" in V_NONE, f"got {V_NONE!r}")
check("★ 而且**不许**出现 ✅（那会让人读成「可以照这张图划区」）",
      "✅" not in V_NONE, f"got {V_NONE!r}")
check("  反过来：可复现那条必须是 ✅ 开头（符号与结论不许脱钩）",
      V_OK.startswith("✅") and V_BAD.startswith("❌"))
check("有失败但只有一个解 ⇒ 仍然算可复现，并把失败次数说出来",
      "可复现" in verdict(tally([A] * 9 + [None]), A)
      and "1 次" in verdict(tally([A] * 9 + [None]), A))


print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
