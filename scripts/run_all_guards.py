#!/usr/bin/env python3
"""全部**离线**守卫一把跑完 —— 清单是**算**出来的，不是手抄的。

为什么要它（第一百零九轮的病）
==============================
启动器的守卫清单一直是**手工维护**的。2026-10-06/07 那一轮发现
`scripts/test_startup_docs.py` **崩死了约 13.5 小时**：它在 `[0/3]` 之前就抛
`ValueError`，**一条判据都没跑**。它没被判红 —— 因为**它压根不在那份清单里**。
（当时列的是「七项」，真切的是八项。）⇒ 这不是「某道守卫坏了」，是
**「谁的清单」坏**：靠人记的清单，迟早漏掉一条，而漏掉的那条**静默**。

本仓对这类问题已经有过一个解，抄它就是了：`test_docs_map.py` 管
「**每份文档在地图里恰好出现一次**」—— 把「别忘了登记」变成**断言**。
这里同理：**守卫清单由目录扫出来**。新增一个 `scripts/test_*.py` 自动被收进来，
**想漏也漏不掉**；清单不可能与目录分叉，因为它是从目录算出来的。
（`test_run_all_guards.py` 把「扫得全」这条钉住，并用变异证明它吃劲。）

⚠️ **为什么是 `.py` 不是 `.sh`**（原提法是 `run_all_guards.sh`）：只有 Python
才能被同一个仓的守卫体系自测 —— 那个「扫得全」的断言是本文件的**全部价值**，
它必须自己也被钉住。一个 shell 包装脚本做不到这件事。

判据（退出码沿用本仓约定：0 通过 / 1 断言失败 / 2 环境不满足）
==============================================================
  · `0` —— 全部通过（**允许**有 rc=2 的「环境不满足」，那不算失败）
  · `1` —— 至少一道守卫**真判红**（rc=1），或有守卫**超时/异常**
  · `2` —— **一道都没跑成**（环境坏到跑不动守卫本身）

⚠️ 超时与「环境不满足」**不是一回事**，分开记：`rc=2` 是守卫自己说「这个环境
跑不了我」（照 `state_check.py` 的约定，离线核不了的行它明说自己核不了）；
**超时**是它没说 —— 那大概率是它去够了一个够不着的东西（板子/真机），
**不假装是「环境不满足」**，也不假装通过。

用法
====
    python3 scripts/run_all_guards.py            # 跑全部
    python3 scripts/run_all_guards.py --list     # 只列清单（不跑）
    python3 scripts/run_all_guards.py -k board   # 只跑名字里含 board 的
    python3 scripts/run_all_guards.py --timeout 300
"""

import argparse
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SELF = os.path.basename(os.path.abspath(__file__))

# 三态（与仓里别处同词）+ 两个**不是通过**的档
PASS, FAIL, SKIP, TIMEOUT, ERROR = "PASS", "FAIL", "SKIP", "TIMEOUT", "ERROR"

#: 除 `test_*.py` 之外还要跑的守卫（不带 `test_` 前缀，扫不到）。
#: ⚠️ 这里是**加**项，不是主清单 —— 主清单永远是扫出来的。
EXTRA_GUARDS = ("state_check.py",)

DEFAULT_TIMEOUT = 300.0


def guard_files(here=HERE):
    """本目录下**全部**离线守卫的绝对路径（**算出来的**，不是手抄的）。**纯函数**。

    规则只有一条：`test_*.py` ＋ `EXTRA_GUARDS`。自己（`run_all_guards.py`）
    不在其内 —— 它不是守卫，是跑守卫的。

    返回**排序后**的列表（好让 `--list` 与报告稳定、可 diff）。
    """
    names = set(EXTRA_GUARDS)
    for fn in os.listdir(here):
        if fn.startswith("test_") and fn.endswith(".py"):
            names.add(fn)
    names.discard(SELF)
    return [os.path.join(here, n) for n in sorted(names)
            if os.path.isfile(os.path.join(here, n))]


def classify(rc):
    """守卫退出码 → 报告里的档。**纯函数**。

    沿用本仓约定：`0` 通过 / `1` 断言失败 / `2` **环境不满足**。
    `2` **不算失败**（离线核不了的行由 `state_check.py` 明说自己核不了）——
    但也**不算通过**，单独一档，别把它渲染成绿。
    """
    if rc == 0:
        return PASS
    if rc == 1:
        return FAIL
    if rc == 2:
        return SKIP
    return ERROR


def summarize(rows):
    """→ `(退出码, 一句话)`。**纯函数**。

    `rows` = `[(名字, 档, 秒), ...]`。
    """
    n_pass = sum(1 for _, s, _ in rows if s == PASS)
    bad = [r for r in rows if r[1] in (FAIL, TIMEOUT, ERROR)]
    n_skip = sum(1 for _, s, _ in rows if s == SKIP)
    detail = ("%d 通过 / %d 红 / %d 环境不满足 / 共 %d"
              % (n_pass, len(bad), n_skip, len(rows)))
    if not rows or (n_pass == 0 and n_skip == 0 and not bad):
        return 2, detail + " —— **一道都没跑成**"
    if bad:
        return 1, detail + " —— 红的是：" + "、".join(r[0] for r in bad)
    return 0, detail


def run_one(path, timeout):
    """跑一道守卫 → `(档, 秒, 输出尾, 备注)`。**只有这里碰进程**。"""
    t0 = time.monotonic()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    try:
        p = subprocess.run([sys.executable, path], cwd=REPO, env=env,
                           capture_output=True, text=True, timeout=timeout)
        dt = time.monotonic() - t0
        out = ((p.stdout or "") + (p.stderr or "")).strip()
        return classify(p.returncode), dt, out, "rc=%d" % p.returncode
    except subprocess.TimeoutExpired:
        dt = time.monotonic() - t0
        return (TIMEOUT, dt,
                "**超时** %.0fs —— 它没说「环境不满足」，大概率是在够一个够不着的"
                "东西（板子/真机）。**不假装是环境不满足，也不假装通过。**"
                % timeout, "timeout")
    except OSError as exc:
        return ERROR, time.monotonic() - t0, "起不来：%s" % exc, "OSError"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="全部离线守卫一把跑完（清单是算出来的）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-k", "--filter", default=None,
                    help="只跑文件名里含这个词的守卫")
    ap.add_argument("--list", action="store_true", help="只列清单，不跑")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                    help="单道守卫的超时秒数（默认 %g）" % DEFAULT_TIMEOUT)
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="只印失败那几道的输出")
    args = ap.parse_args(argv)

    files = guard_files()
    if args.filter:
        files = [f for f in files if args.filter in os.path.basename(f)]
    if not files:
        print("⚠️ 一个守卫都没扫到（--filter 太窄？）")
        return 2
    if args.list:
        for f in files:
            print("  %s" % os.path.relpath(f, REPO))
        print("  共 %d 道（**扫出来的**，不是手抄的）" % len(files))
        return 0

    print("=" * 70)
    print("全部离线守卫 —— 共 %d 道（清单由 %s 扫出，不是手抄的）"
          % (len(files), os.path.relpath(__file__, REPO)))
    print("=" * 70)
    rows, outs = [], {}
    for f in files:
        name = os.path.basename(f)
        state, dt, out, note = run_one(f, args.timeout)
        rows.append((name, state, dt))
        outs[name] = out
        # `SKIP` 后面点明是「环境不满足」而不是「没验」，免得被读成通过。
        mark = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭ ",
                "TIMEOUT": "⏰", "ERROR": "💥"}[state]
        print("  %s %-32s %-8s %6.1fs  %s"
              % (mark, name, state, dt, "" if state == PASS else note))

    bad = [r for r in rows if r[1] in (FAIL, TIMEOUT, ERROR)]
    if bad:
        for name, state, _ in bad:
            print("\n" + "-" * 70)
            print("❌ %s（%s）的最后 25 行：" % (name, state))
            print("-" * 70)
            for line in (outs.get(name) or "").splitlines()[-25:]:
                print("  " + line)
    elif not args.quiet:
        skipped = [r[0] for r in rows if r[1] == SKIP]
        if skipped:
            print("\n  ⏭  环境不满足（**不算通过**，也不算失败）：%s"
                  % "、".join(skipped))

    code, detail = summarize(rows)
    print("\n" + "=" * 70)
    print("  %s" % detail)
    print("  退出码 0 = 全过（允许有 rc=2 的环境不满足）／1 = 有守卫真判红"
          "／2 = 一道都没跑成")
    print("=" * 70)
    return code


if __name__ == "__main__":
    sys.exit(main())
