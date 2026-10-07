#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`run_all_guards.py` 的自检 —— **纯离线**，不跑真正的守卫。

它存在的理由就只有一条，所以自检也**只钉这一条**：
**「清单是算出来的，漏不掉」** —— 其余全是附带。

分四节：
  1. 扫得全：目录里每一个 `test_*.py` 都在清单里（逐个比对，不数个数）
  2. 不扫自己、扫到 `state_check.py`（它是守卫但没有 `test_` 前缀）
  3. 三态与退出码：`classify` / `summarize` 的映射
  4. 变异：把「扫目录」换成**手写清单**并**故意漏一道** ⇒ 第 1 节必须翻红

第 4 节是本文件的重点。第 1 节若只写「清单非空」或「条数 ≥ 30」，
漏掉一条**照样绿** —— 而那正是第一百零九轮那个病的**同一个形状**
（手工清单漏掉 `test_startup_docs.py`，它崩死 13.5 小时没人发现）。

     python3 scripts/test_run_all_guards.py

退出码：0 = 全过；1 = 有断言失败。
"""

import sys

sys.dont_write_bytecode = True

import importlib.util            # noqa: E402
import os                        # noqa: E402
import tempfile                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import run_all_guards as rag     # noqa: E402

_failed = []
_n = 0
_TMP = tempfile.mkdtemp(prefix="rag_test_")


def check(name, cond, detail=""):
    global _n
    _n += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f"  {detail}" if detail else ""))
    if not cond:
        _failed.append(name)


def section(title):
    print(f"\n{title}")


def _mutate(src, old, new, tag):
    """锚点找不到就记一次失败 —— 锚点漂了而变异「照样跑过」是假绿。"""
    if old not in src:
        check("变异锚点还在（%s）" % tag, False, "源码里找不到：%r" % old[:70])
        return None
    return src.replace(old, new, 1)


def load_mutant(src_text, tag):
    d = tempfile.mkdtemp(prefix="mut_%s_" % tag, dir=_TMP)
    p = os.path.join(d, "rag_mut_%s.py" % tag)
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location("rag_mut_%s" % tag, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


with open(os.path.join(HERE, "run_all_guards.py"), encoding="utf-8") as _fh:
    SRC = _fh.read()

# =====================================================================
section("[1] 扫得全：目录里每一个 `test_*.py` 都在清单里")

# ⚠️ 这里**自己数一遍目录**，不读被测对象的任何中间量 —— 否则就是拿它给自己
#    打分。判据是**逐个比对**（差集为空），不是「条数看起来够多」。
# 期望集合 = 目录里的 `test_*.py` ∪ `EXTRA_GUARDS`，再扣掉自己。`state_check.py`
# 走的是后一条路（它是守卫，但没有 `test_` 前缀）—— 不并进来的话，「一个都没多」
# 会把**本来就该在**的它误判成多余。
_want = sorted(
    {fn for fn in os.listdir(HERE)
     if fn.startswith("test_") and fn.endswith(".py")}
    | set(rag.EXTRA_GUARDS)
    - {os.path.basename(os.path.abspath(rag.__file__))})
_got = sorted(os.path.basename(f) for f in rag.guard_files())

_missing = [n for n in _want if n not in _got]
check("★ 目录里的 `test_*.py` **一个不少**（差集为空）"
      "—— 这条就是本文件存在的全部理由",
      not _missing, "缺：%s" % (_missing or "（无）"))
check("  一个都没多（清单里不许有目录里没有的名字）",
      not [n for n in _got if n not in _want],
      "多：%s" % ([n for n in _got if n not in _want] or "（无）"))
check("  显然不是「恰好扫到 0 个」那种假绿（真目录里守卫有几十道）",
      len(_got) > 20, "%d 道" % len(_got))

# =====================================================================
section("[2] 不扫自己；但扫得到 `state_check.py`")

check("  `run_all_guards.py` **不在**清单里（它是跑守卫的，不是守卫）",
      "run_all_guards.py" not in _got)
check("  `state_check.py` 在清单里 —— 它是守卫，只是没有 `test_` 前缀，"
      "靠 `EXTRA_GUARDS` 补进来",
      "state_check.py" in _got)
check("  `EXTRA_GUARDS` 与「扫出来的」是**两回事**：加项不能变成主清单"
      "（主清单永远是 `os.listdir` 算出来的）",
      len(rag.EXTRA_GUARDS) <= 3, repr(rag.EXTRA_GUARDS))

# =====================================================================
section("[3] 三态与退出码")

check("  `classify`：0→PASS、1→FAIL、2→**SKIP（环境不满足）**、其余→ERROR",
      (rag.classify(0), rag.classify(1), rag.classify(2), rag.classify(7))
      == (rag.PASS, rag.FAIL, rag.SKIP, rag.ERROR))
check("★ `rc=2` **不许**算通过 —— 它是「这个环境跑不了我」，"
      "与 `state_check.py` 的约定同源",
      rag.summarize([("a", rag.SKIP, 1.0)])[0] == 0
      and rag.summarize([("a", rag.SKIP, 1.0)])[1].find("环境不满足") >= 0)
check("★ 总表按**拦不拦**说事：警告与失败**各有名字**，不靠红绿两色"
      "（用户 2026-10-07 裁定：「可以有只 warning 的错误，"
      "但要有 failed 拦截重要错误」）",
      "警告" in rag.summarize([("a", rag.SKIP, 1.0)])[1]
      and "失败" in rag.summarize([("a", rag.PASS, 1.0),
                                   ("b", rag.FAIL, 1.0)])[1])
check("★ 警告再多也**不拦**：42 道里 30 道「环境不满足」＋ 1 道通过 ⇒ 仍退 0"
      "（第一百一十一轮悬案 ⑧ 的裁决：**不设阈值**，缺依赖是这台机器的事）",
      rag.summarize([("g%d" % i, rag.SKIP, 1.0) for i in range(30)]
                    + [("p", rag.PASS, 1.0)])[0] == 0)
check("  有 FAIL ⇒ 退 1，且**点名**是哪一道",
      rag.summarize([("a", rag.PASS, 1.0), ("b", rag.FAIL, 1.0)])[0] == 1
      and "b" in rag.summarize([("a", rag.PASS, 1.0),
                                ("b", rag.FAIL, 1.0)])[1])
check("  超时**不假装**成「环境不满足」（够不着板子 ≠ 这个环境跑不了我）",
      rag.summarize([("a", rag.TIMEOUT, 1.0)])[0] == 1)
check("  一道都没跑成 ⇒ 退 2（不是退 0）",
      rag.summarize([])[0] == 2)

# =====================================================================
section("[4] 变异：把「扫目录」换成手写清单并**故意漏一道** ⇒ [1] 必须翻红")

_m = load_mutant(_mutate(
    SRC,
    "    for fn in os.listdir(here):\n"
    "        if fn.startswith(\"test_\") and fn.endswith(\".py\"):\n"
    "            names.add(fn)\n",
    "    names |= {\"test_board_lib.py\"}\n", "hard"), "hard")
if _m is not None:
    # ⚠️ 必须**显式把真实的 scripts 目录传进去**：变异体住在临时目录里，
    #    它自己的 `HERE` 是那个空目录 —— 不传的话 `_m_missing` 会是「全部」，
    #    断言**为了错的理由**变红（测的是「临时目录是空的」，不是「手写清单会漏」）。
    _m_got = sorted(os.path.basename(f) for f in _m.guard_files(HERE))
    _m_missing = [n for n in _want if n not in _m_got]
    check("  变异体确实只认得手写的那两道（否则下面那条是空断言）",
          set(_m_got) == {"state_check.py", "test_board_lib.py"},
          repr(_m_got))
    check("★ 手写清单漏掉一堆 ⇒ 第 [1] 节那条「差集为空」**真的翻红**"
          "（不是恒亮 —— 恒亮的断言等于没写）",
          bool(_m_missing) and not _missing,
          "变异后缺 %d 个（原版缺 0 个）" % len(_m_missing))

# =====================================================================
section("[5] 命令行：`--list` 退 0 且真的列出清单")

import subprocess                # noqa: E402

_p = subprocess.run([sys.executable, os.path.join(HERE, "run_all_guards.py"),
                     "--list"], capture_output=True, text=True, cwd=REPO)
check("  `--list` 退 0", _p.returncode == 0, "rc=%d" % _p.returncode)
check("  `--list` 列出了 `state_check.py` 与某道 `test_*`",
      "state_check.py" in _p.stdout and "test_board_lib.py" in _p.stdout)
check("  `--list` **不提**「扫出来的」以外的东西（它只是列清单，不跑）",
      "通过" not in _p.stdout.split("共")[0])

# =====================================================================
print("\n" + "=" * 68)
if _failed:
    print("❌ %d / %d 条断言失败：" % (len(_failed), _n))
    for f in _failed:
        print("   · %s" % f)
    sys.exit(1)
print("✅ 全部通过（test_run_all_guards.py，%d 条）" % _n)
