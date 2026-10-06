#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""board_tree_manifest.py 的自检 —— **纯离线**，只用临时目录造树。

分四节：
  1. 跳过规则：colcon 产物与 .pyc 不许进哈希（否则每次编译都「有差异」）
  2. 差异分类：多 / 少 / 改 三种都要认得出，且**空清单不算一致**
  3. 变异检查：把判据改坏，结论必须真的翻面
  4. 命令行行为：没有清单 ⇒ 退 2（fail-closed），不是退 0

     python3 scripts/test_board_tree_manifest.py

退出码：0 = 全过；1 = 有断言失败。
"""

import sys

sys.dont_write_bytecode = True

import importlib.util            # noqa: E402
import json                      # noqa: E402
import os                        # noqa: E402
import shutil                    # noqa: E402
import subprocess                # noqa: E402
import tempfile                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import board_tree_manifest as btm   # noqa: E402

_failed = []
_n = 0


def check(name, cond, detail=""):
    global _n
    _n += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not cond:
        _failed.append(name)


def tree(spec, root):
    """按 `{相对路径: 内容}` 造一棵真树。"""
    for rel, body in spec.items():
        full = os.path.join(root, rel)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "w", encoding="utf-8") as fh:
            fh.write(body)
    return root


FIX = {
    "src/pkg/package.xml": "<package/>",
    "src/pkg/pkg/node.py": "print('hi')\n",
    "src/pkg/config/x.yaml": "a: 1\n",
}


# ------------------------------------------------ 1. 跳过规则
def section_skip():
    print("\n[1] 跳过规则：产物与字节码不许进哈希")
    for rel, want in (
            ("build/pkg/x.py", True),
            ("install/pkg/lib/y.py", True),
            ("log/build.log", True),
            ("src/pkg/__pycache__/node.cpython-310.pyc", True),
            ("src/pkg/node.pyc", True),
            ("src/pkg/package.xml", False),
            ("src/pkg/pkg/node.py", False),
            # 注意：路径里**含** build/ 字样但不是那个目录，不许误伤
            ("src/pkg/rebuild/x.py", False),
            ("docs/build.md", False),
    ):
        check(f"skip({rel!r}) == {want}", btm.skip(rel) is want, f"实得 {btm.skip(rel)}")

    d = tempfile.mkdtemp(prefix="btm_skip_")
    try:
        tree(dict(FIX, **{"src/pkg/__pycache__/node.cpython-310.pyc": "x",
                          "build/pkg/out.py": "x",
                          "install/setup.bash": "x"}), d)
        files = btm.collect(d)
        check("真树上只留下 3 个源文件", len(files) == 3, str(sorted(files)))
        check("  产物一个都没进来",
              not any(k.startswith(("build/", "install/")) for k in files))
        check("  相对路径用正斜杠（清单要跨机器比）",
              all("/" in k and "\\" not in k for k in files))
    finally:
        shutil.rmtree(d, ignore_errors=True)


# ------------------------------------------------ 2. 差异分类
def section_compare():
    print("\n[2] 差异分类：多 / 少 / 改 各归各的")
    old = {"a.py": "1", "b.py": "2", "c.py": "3"}
    new = {"a.py": "1", "b.py": "X", "d.py": "4"}
    diff = btm.compare(old, new)
    check("改：只有 b.py（同键不同值）", diff["modified"] == ["b.py"], str(diff))
    check("少：只有 c.py", diff["removed"] == ["c.py"], str(diff))
    check("多：只有 d.py", diff["added"] == ["d.py"], str(diff))
    check("有差异 ⇒ differing() 为真", btm.differing(diff))
    check("完全一致 ⇒ differing() 为假",
          not btm.differing(btm.compare(old, dict(old))))
    check("**两个空清单** 也算一致（这是 compare 的语义，不是漏洞）",
          not btm.differing(btm.compare({}, {})))
    # 内容改了但**长度没变**——只比长度/文件名的实现会漏掉它
    check("只比长度/文件名的实现会漏掉它：同长不同内容 ⇒ 判为「改」",
          btm.compare({"a": "aa"}, {"a": "bb"})["modified"] == ["a"])

    d = tempfile.mkdtemp(prefix="btm_cmp_")
    try:
        tree(FIX, d)
        h1 = btm.collect(d)
        check("哈希是 sha256 十六进制（64 位）",
              all(len(v) == 64 and all(c in "0123456789abcdef" for c in v)
                  for v in h1.values()))
        h2 = btm.collect(d)
        check("同一棵树两次哈希一致（确定性）", h1 == h2)
        tree({"src/pkg/pkg/node.py": "print('hi')\n# 改一行\n"}, d)
        h3 = btm.collect(d)
        check("动一个文件 ⇒ 只有它变（其余 hash 逐位相同）",
              set(h3) == set(h1) and h3["src/pkg/pkg/node.py"] != h1["src/pkg/pkg/node.py"]
              and all(h3[k] == h1[k] for k in h1 if k != "src/pkg/pkg/node.py"))
        check("  并且 compare 只报这一处",
              btm.compare(h1, h3)["modified"] == ["src/pkg/pkg/node.py"])
    finally:
        shutil.rmtree(d, ignore_errors=True)


# ------------------------------------------------ 3. 变异检查
def load_mutant(src_text, tag):
    d = tempfile.mkdtemp(prefix=f"btmmut_{tag}_")
    path = os.path.join(d, "btm_mutant.py")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location(f"btm_mutant_{tag}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mutate(src, old, new, tag):
    if old not in src:
        check(f"{tag} 找得到要改的那一行", False, old[:60])
        return None
    return load_mutant(src.replace(old, new, 1), tag)


def section_mutation():
    print("\n[3] 变异检查：把判据改坏，结论必须**真的翻面**")
    src = open(os.path.join(HERE, "board_tree_manifest.py"), encoding="utf-8").read()

    # M9：把 .pyc 的跳过拿掉 —— 于是「编译过一次」会被报成源码差异。
    m = _mutate(src, 'SKIP_SUFFIX = (".pyc", ".pyo", ".swp")',
                'SKIP_SUFFIX = (".pyo", ".swp")  # MUT', "m9")
    if m is not None:
        # ⚠️ 夹具**不能**用 `__pycache__/x.pyc` —— 那条路径被目录规则挡掉了，
        #    于是拿掉后缀规则也照样 True（变异不生效，测出来的是「恰好为真」）。
        #    要钉后缀规则，就得用一个**不在** __pycache__ 里的 .pyc。
        probe = "src/pkg/legacy.pyc"
        base = btm.skip(probe)
        got = m.skip(probe)
        check("M9 不再跳过 .pyc ⇒ 产物被当成源码（与改前不同）",
              base is True and got is False, f"改前={base} 改后={got}")
        check("  同一条判据对 __pycache__ 里的仍然生效（两道网，各管一段）",
              m.skip("src/pkg/__pycache__/n.pyc") is True)
        # 反向：源码仍要留下，不是靠「全跳过」换来的
        check("  同一变异没把真源码也跳过",
              m.skip("src/pkg/node.py") is False)

    # M10：compare 把「改了」漏成「没变」（只比键集合）。
    m = _mutate(src, '        "modified": sorted(k for k in old if k in new and old[k] != new[k]),',
                '        "modified": [],  # MUT', "m10")
    if m is not None:
        old, new = {"a": "1"}, {"a": "X"}
        base = btm.differing(btm.compare(old, new))
        got = m.differing(m.compare(old, new))
        check("M10 漏掉「改了」⇒ 内容不同却报全树一致（与改前不同）",
              base is True and got is False, f"改前={base} 改后={got}")

    # M11：清单**形状不对**时不再单独判 —— 于是一份「什么都没记住」的清单会被当成
    #      一份「什么都对得上」的空清单往下走。
    #      ⚠️ 变异点不选「文件缺失」那一支：那一支拿掉之后 `open()` 抛
    #      FileNotFoundError，仍被同一个 `except (OSError, ValueError)` 兜住、
    #      抛的还是 ManifestError —— **看起来一样**，测不出翻面。
    m = _mutate(src, '    if not isinstance(data, dict) or not isinstance(data.get("files"), dict):',
                "    if False:  # MUT", "m11")
    if m is not None:
        d = tempfile.mkdtemp(prefix="btm_badshape_")
        try:
            p = os.path.join(d, "bad.json")
            with open(p, "w", encoding="utf-8") as fh:
                json.dump({"files": []}, fh)

            def outcome(mod):
                try:
                    mod.read_manifest(p)
                    return "没抛错"
                except mod.ManifestError:
                    return "抛 ManifestError"
                except Exception as exc:                  # noqa: BLE001
                    return f"抛 {exc.__class__.__name__}"

            base, got = outcome(btm), outcome(m)
            check("M11 形状不再校验 ⇒ 空清单被当成「一致」往下走（与改前不同）",
                  base == "抛 ManifestError" and got == "没抛错",
                  f"改前={base} 改后={got}")
            try:
                btm.read_manifest(p)
                why = ""
            except btm.ManifestError as exc:
                why = str(exc)
            check("  改前的红说的是**形状**不对，不是撞上了别的判据",
                  "形状" in why, why[:80])
        finally:
            shutil.rmtree(d, ignore_errors=True)

    # 非变异断言：文件**不存在**也要单独判（这条是 CLI 退 2 的来源）
    d = tempfile.mkdtemp(prefix="btm_missing_")
    try:
        try:
            btm.read_manifest(os.path.join(d, "nope.json"))
            check("清单文件不存在 ⇒ 抛 ManifestError（不是返回空清单）", False, "没抛错")
        except btm.ManifestError:
            check("清单文件不存在 ⇒ 抛 ManifestError（不是返回空清单）", True)
    finally:
        shutil.rmtree(d, ignore_errors=True)


# ------------------------------------------------ 4. 命令行行为
def _run(*args):
    return subprocess.run([sys.executable, os.path.join(HERE, "board_tree_manifest.py"),
                           *args], capture_output=True, text=True)


def section_cli():
    print("\n[4] 命令行行为：没有清单不许退 0（fail-closed）")
    d = tempfile.mkdtemp(prefix="btm_cli_")
    try:
        root = tree(FIX, os.path.join(d, "tree"))
        man = os.path.join(d, "manifest.json")

        r = _run("--root", root, "--manifest", man)
        check("没有清单 ⇒ 退 2 并说去哪写", r.returncode == 2 and "--write" in r.stdout,
              f"rc={r.returncode} {r.stdout.strip()[:100]}")

        r = _run("--root", root, "--manifest", man, "--write")
        check("--write ⇒ 退 0", r.returncode == 0, f"rc={r.returncode} {r.stdout.strip()[:80]}")
        check("  清单真的落在指定位置（不在仓库里）",
              os.path.isfile(man) and REPO not in os.path.abspath(man))

        r = _run("--root", root, "--manifest", man)
        check("原样再 check ⇒ 退 0", r.returncode == 0, f"rc={r.returncode}")

        # 动一个文件、加一个文件、删一个文件 —— 三种差异都要报出来
        tree({"src/pkg/pkg/node.py": "print('changed')\n",
              "src/pkg/new.py": "x = 1\n"}, root)
        os.remove(os.path.join(root, "src/pkg/config/x.yaml"))
        r = _run("--root", root, "--manifest", man)
        check("多/少/改各一处 ⇒ 退 1",
              r.returncode == 1, f"rc={r.returncode}")
        check("  逐条点名，且带上两个哈希的前缀",
              all(k in r.stdout for k in
                  ("src/pkg/new.py", "src/pkg/config/x.yaml", "src/pkg/pkg/node.py"))
              and "…" in r.stdout, r.stdout.strip()[-200:])
        check("  提醒同步完要 colcon build（不变量 4 的另一半）",
              "colcon build" in r.stdout)

        # 根不对（差一层前缀）时要提醒，否则会表现为「全都对不上」
        r = _run("--root", os.path.join(d, "tree", "src"), "--manifest", man)
        check("两次的根不是同一个 ⇒ 明确提醒，而不是让人去猜 800 行差异",
              "不是同一个目录" in r.stdout, r.stdout.strip()[:160])

        r = _run("--root", root, "--manifest", man, "--write", "--check")
        check("--write 与 --check 同时给 ⇒ 退 2", r.returncode == 2)

        with open(man, "w", encoding="utf-8") as fh:
            fh.write("{ 这不是 json")
        r = _run("--root", root, "--manifest", man)
        check("清单坏了 ⇒ 退 2，不当成「一致」", r.returncode == 2, f"rc={r.returncode}")
        with open(man, "w", encoding="utf-8") as fh:
            json.dump({"files": []}, fh)
        r = _run("--root", root, "--manifest", man)
        check("清单形状不对（files 不是映射）⇒ 退 2", r.returncode == 2,
              f"rc={r.returncode}")

        r = _run("--root", os.path.join(d, "nope"), "--manifest", man, "--write")
        check("根不存在 ⇒ 退 2", r.returncode == 2)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def main():
    print("board_tree_manifest 自检（纯离线：只在临时目录里造树）")
    section_skip()
    section_compare()
    section_mutation()
    section_cli()
    print(f"\n{_n} 条断言，{len(_failed)} 条失败")
    if _failed:
        for n in _failed:
            print(f"  ✗ {n}")
        return 1
    print("结果：✅ board_tree_manifest.py 全过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
