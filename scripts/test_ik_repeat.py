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

import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ik_repeat_check import fold, n_poses, spread, tally, verdict   # noqa: E402

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


def safe(fn, *a, **kw):
    """跑一个**可能抛**的调用；抛了返回一句「崩了」，而不是让套件当场中断。

    ⚠️ 为什么非有它不可：一个抛异常的调用会让套件**停在那里**，后面的检查一条
    都不跑，也不印「通过/失败」汇总 —— 而变异脚本只能数 `FAIL` 行，于是
    「崩了」会被读成「变异存活」，一个**假阴性**。本项目已经栽过三次
    （`pick_zone_z_note(None,…)` 那次、`tail -1`、`tail -6`），这是第四种形态，
    所以这次把它**变成一条红的检查**而不是靠记性。
    """
    try:
        return fn(*a, **kw)
    except Exception as e:                       # noqa: BLE001 —— 就是要全接住
        return f"!! 崩了：{type(e).__name__}: {e}"


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
V_OK = safe(verdict, tally([A] * 20), A)
check("★ 一个解 ⇒ 说「可复现」", "可复现" in V_OK and "❌" not in V_OK, f"got {V_OK!r}")
V_BAD = safe(verdict, tally([A] * 7 + [B] * 3), A)
check("★ 两个解 ⇒ 说「不可复现」", "不可复现" in V_BAD, f"got {V_BAD!r}")
check("  并且点名是**肘分支翻转**（不是噪声）", "肘分支翻转" in V_BAD, f"got {V_BAD!r}")
check("  并且给出**读法**（留一格 0.05 m）", "0.05" in V_BAD and "一格" in V_BAD)
check("  并且说清**矩形仍然可用**（不能把整张图作废）",
      "最大全安全矩形" in V_BAD, f"got {V_BAD!r}")
check("  并且给**根治**（attempts / consistency_limits）",
      "attempts" in V_BAD and "consistency_limits" in V_BAD, f"got {V_BAD!r}")
check("  带上相差最大的那个分支的数（A→B 的最大关节差 = 1.0 rad）",
      "1.000 rad" in V_BAD, f"got {V_BAD!r}")
V_NONE = safe(verdict, tally([None] * 5), A)
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
      "可复现" in safe(verdict, tally([A] * 9 + [None]), A)
      and "1 次" in safe(verdict, tally([A] * 9 + [None]), A))

# ---- C2. 复验（recheck）：判词里的病根必须是**量出来的** ----
print("\n--- C2. 复验 ---")
V_NORECHECK = safe(verdict, tally([A] * 7 + [B] * 3), A)
# ⚠️ 这里**不能**断言 `"随机重启" not in V_NORECHECK`：那句告警本身就在说
# 「别直接断言『随机重启』」，**字面上含着**这四个字。用子串判会把一句正确的
# 话判红 —— 与 C 段 `V_NONE` 那次是**同一个坑**，所以这里钉的是**断言因果**
# 才有的那半句（`随机性来自`），以及结论行的 `★` 标记不许出现。
check("★ 没做复验 ⇒ 明说「病根还没量」，**不许**断言因果",
      "还没量" in V_NORECHECK and "随机性来自" not in V_NORECHECK
      and "★" not in V_NORECHECK,
      f"got {V_NORECHECK!r}")
check("  没做复验时那句是**告警**（⚠️），不是结论",
      "⚠️" in V_NORECHECK)
V_GOOD = safe(verdict, tally([A] * 7 + [B] * 3), A, recheck=tally([B] * 20))
check("★ 复验只回来一个解 ⇒ 说「随机性来自种子解不出来时的随机重启」",
      "种子解不出来时的随机重启" in V_GOOD, f"got {V_GOOD!r}")
check("  这时不再印那句「还没量」", "还没量" not in V_GOOD)
V_BADRECHECK = safe(verdict, tally([A] * 7 + [B] * 3), A,
                       recheck=tally([A] * 5 + [B] * 5 + [(1.0,) * 6]))
check("★ 复验还是多个解 ⇒ 改口说「解算器本身就不可复现」（不是种子的锅）",
      "解算器本身就不可复现" in V_BADRECHECK, f"got {V_BADRECHECK!r}")
check("  两种复验结论**不许同时出现**（说清楚到底是哪一种）",
      not ("种子解不出来时的随机重启" in V_BADRECHECK))


# ----------------------------------------------------------------------
# D. fold / n_poses：±2π 绕数**不是**不同的位形
# ----------------------------------------------------------------------
print("\n--- D. fold / n_poses ---")
check("本来就在 (−π,π] 里的原样不动", fold(A) == tuple(round(v, 3) for v in A))
check("整圈绕出去的被折回来：0.0 + 2π ⇒ 0.0",
      fold((2 * math.pi,)) == (0.0,))
check("★ 逐关节加 2π 后折回，与原来**同一个位形**",
      fold(tuple(v + 2 * math.pi for v in A)) == fold(A))
check("  而它们在 tally 里是**两个**条目（原始值确实不同）",
      len(tally([A, tuple(v + 2 * math.pi for v in A)])) == 2)
check("★ n_poses 把它们算成 **1** 个位形（这正是旧版判错的地方）",
      n_poses(tally([A, tuple(v + 2 * math.pi for v in A)])) == 1)
check("★ 反过来：真肘翻转（差 ~1 rad）**不许**被折回合并",
      n_poses(tally([A, B])) == 2)
check("★ ±π 接缝上不分叉：fold(π) 与 fold(−π) 相等",
      fold((math.pi,)) == fold((-math.pi,)))
check("  （而且是落在 −π 那一侧，不是随机两边）",
      fold((math.pi,)) == (round(-math.pi, 3),))
check("None ⇒ None（不是抛，也不是 0）", fold(None) is None)
check("ndigits=None ⇒ **不取整**（原值回来，不是 3 位）",
      fold((0.123456789,), None)[0] != round(0.123456789, 3)
      and abs(fold((0.123456789,), None)[0] - 0.123456789) < 1e-12)
check("n_poses 忽略 None 条目", n_poses(tally([None, A, None])) == 1)
check("全是 None ⇒ 0 个位形", n_poses(tally([None, None])) == 0)
check("空 ⇒ 0 个位形", n_poses(tally([])) == 0)
check("★ 判词里两个数都报：绕数被折回时要**说出来**",
      "个真正不同的位形" in safe(
          verdict, tally([A, tuple(v + 2 * math.pi for v in A)] * 3), A))
check("  而原始数就等于位形数时**不许多嘴**（免得看的人以为有绕数）",
      "个真正不同的位形" not in V_BAD)


print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
