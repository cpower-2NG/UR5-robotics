#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全树哈希清单 —— 关掉「板子上那份代码树静默过期」（不变量 4）。

**为什么需要它。** 板子上那份检出**不是**本机这份的镜像，而且会**静默**过期：
两边都在改，谁也不报错。历史教训是**抽查**（只对几个文件）—— 抽查漏掉的那个文件
恰恰就是改过的那个，于是「对过了」变成一句空话。这里改成对**全树**哈希：
只要有一个文件不同，`--check` 就红。

**清单必须放仓库外**：里面是路径与哈希，属现场信息；而且它本来就不该被提交
（本仓是公开副本）。默认 `~/.config/ur5-vision-grasp/tree-manifest.json`，
可用 `--manifest PATH` 或 `$UR5VG_MANIFEST` 覆盖。

两条使用姿势：

    # 本机（改之前/之后各一次）
    python3 scripts/board_tree_manifest.py --write

    # 到了板子上，或者同步完之后
    python3 scripts/board_tree_manifest.py --check

退出码：0 = 与清单一致；1 = 有差异（多/少/改了，逐条列出）；2 = 清单不在或读不动
（**fail-closed**：没有清单不等于「一致」）。
"""

import argparse
import datetime as dt
import hashlib
import json
import os
import sys

sys.dont_write_bytecode = True   # 本仓约定：防 .pyc 假绿

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DEFAULT_ROOT = os.path.join(REPO, "ros2_ws")
DEFAULT_MANIFEST = "~/.config/ur5-vision-grasp/tree-manifest.json"
MANIFEST_ENV = "UR5VG_MANIFEST"

# 这些目录**不进哈希**，且各有理由：
#   build/ install/ log/  —— colcon 的产物，是**结果**不是源码；跟着一起哈希，
#                            每次编译都会「有差异」，等于把清单变成噪声。
#   __pycache__            —— .pyc 与源码一一对应，且本仓约定不许留 .pyc。
#   .git/                  —— 版本库内部结构，两边本来就不一样。
SKIP_DIRS = ("build", "install", "log", "__pycache__", ".git")
SKIP_SUFFIX = (".pyc", ".pyo", ".swp")


def skip(rel):
    """这条相对路径要不要跳过。**纯函数**（好让离线测试直接喂路径）。"""
    parts = rel.replace(os.sep, "/").split("/")
    if any(p in SKIP_DIRS for p in parts[:-1]):
        return True
    if parts and parts[-1] in SKIP_DIRS:          # 顶层就叫 build/… 的情况
        return True
    return rel.endswith(SKIP_SUFFIX)


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def collect(root):
    """走一遍 `root`，返回 `{相对路径: sha256}`。**I/O 只剩这一个口子。**

    相对路径统一用正斜杠 —— 清单要能跨机器比，不能带上本机的分隔符。
    """
    out = {}
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root)
        rel_dir = "" if rel_dir == "." else rel_dir
        dirnames[:] = sorted(d for d in dirnames
                             if not skip(os.path.join(rel_dir, d) + "/"))
        for name in sorted(filenames):
            rel = os.path.normpath(os.path.join(rel_dir, name)).replace(os.sep, "/")
            if skip(rel):
                continue
            full = os.path.join(root, rel)
            try:
                with open(full, "rb") as fh:
                    out[rel] = sha256_bytes(fh.read())
            except OSError as exc:
                out[rel] = f"<读不了：{exc.__class__.__name__}>"
    return out


def compare(old, new):
    """两份清单的差异。**纯函数**，返回 `{added, removed, modified}`（都是排序好的列表）。"""
    return {
        "added": sorted(k for k in new if k not in old),
        "removed": sorted(k for k in old if k not in new),
        "modified": sorted(k for k in old if k in new and old[k] != new[k]),
    }


def differing(diff):
    return bool(diff["added"] or diff["removed"] or diff["modified"])


def manifest_path(explicit=None):
    return os.path.expanduser(explicit or os.environ.get(MANIFEST_ENV)
                              or DEFAULT_MANIFEST)


class ManifestError(Exception):
    """清单缺失/不可读。与「有差异」分开：一个是环境不满足，一个是结论。"""


def read_manifest(path):
    """读清单。读不到/不是合法 JSON ⇒ 抛 `ManifestError`（**不**返回空清单：
    空清单会让「什么都对不上」和「什么都没记」长得一样）。"""
    if not os.path.isfile(path):
        raise ManifestError(
            f"没有清单：{path}\n"
            f"   先在**已知是好的那一侧**跑一次：\n"
            f"     python3 scripts/board_tree_manifest.py --write")
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError) as exc:
        raise ManifestError(f"清单读不动：{path}（{exc}）")
    if not isinstance(data, dict) or not isinstance(data.get("files"), dict):
        raise ManifestError(f"清单形状不对：{path} —— 缺 'files' 映射")
    return data


def stale_installs(root):
    """哪些包**没 build 过**（有 `src/<pkg>` 却没有 `install/<pkg>`）。

    不变量 4 的另一半：`install/` 指向 `build/`，改完必须 `colcon build`。
    只**报告**、不影响退出码 —— 本机（开发机）压根不装 `install/`，
    拿它判失败会把正常的本机跑变成红。
    """
    src = os.path.join(root, "src")
    inst = os.path.join(root, "install")
    if not os.path.isdir(src) or not os.path.isdir(inst):
        return None                               # 不是「有没有」，是「判不了」
    pkgs = [d for d in sorted(os.listdir(src))
            if os.path.isfile(os.path.join(src, d, "package.xml"))]
    return [p for p in pkgs if not os.path.isdir(os.path.join(inst, p))]


def build_manifest(root, stamp=None):
    return {
        "root": os.path.abspath(root),
        "generated": (stamp or dt.datetime.now()).isoformat(timespec="seconds"),
        "note": "全树 sha256。清单本身**不进仓库**（公开副本）。",
        "files": collect(root),
    }


def cmd_write(a):
    if not os.path.isdir(a.root):
        print(f"❌ 目录不存在：{a.root}")
        return 2
    data = build_manifest(a.root)
    path = manifest_path(a.manifest)
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=1, sort_keys=True)
    print(f"✅ 已写清单：{path}")
    print(f"   根：{data['root']}")
    print(f"   {len(data['files'])} 个文件")
    return 0


def cmd_check(a):
    path = manifest_path(a.manifest)
    try:
        old = read_manifest(path)
    except ManifestError as exc:
        print(f"❌ {exc}")
        return 2
    if not os.path.isdir(a.root):
        print(f"❌ 目录不存在：{a.root}")
        return 2

    new = collect(a.root)
    diff = compare(old["files"], new)
    print(f"清单：{path}（记于 {old.get('generated', '未知')}，根 {old.get('root')}）")
    print(f"现场：{os.path.abspath(a.root)} —— {len(new)} 个文件")
    if old.get("root") and os.path.abspath(old["root"]) != os.path.abspath(a.root):
        print(f"⚠️ 两次的根不是同一个目录：{old['root']} ≠ {os.path.abspath(a.root)}"
              f" —— 差一个前缀就会「全都对不上」")

    stale = stale_installs(a.root)
    if stale is None:
        print("   （没有 install/ 可对 —— 本机开发树就是这样，跳过 build 检查）")
    elif stale:
        print(f"  ⚠️ 这些包有源码但没有 install/ ⇒ **改了没 colcon build**：{stale}")
    else:
        print("   ✓ install/ 里的包与 src/ 对得上")

    if not differing(diff):
        print(f"\n结果：✅ 全树一致（{len(new)} 个文件逐一 sha256）")
        return 0

    print(f"\n❌ 有差异：多 {len(diff['added'])} / 少 {len(diff['removed'])}"
          f" / 改 {len(diff['modified'])}")
    for tag, key in (("多", "added"), ("少", "removed"), ("改", "modified")):
        for rel in diff[key]:
            print(f"  {tag}  {rel}")
            if tag == "改":
                print(f"       清单 {old['files'][rel][:12]}… → 现在 {new[rel][:12]}…")
    print("\n（同步完记得 `colcon build` —— install/ 指向 build/）")
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="全树哈希清单：对全树，不抽查（不变量 4）")
    ap.add_argument("--root", default=DEFAULT_ROOT,
                    help=f"要哈希的根（默认 {os.path.relpath(DEFAULT_ROOT, REPO)}）")
    ap.add_argument("--manifest", default=None,
                    help=f"清单路径（默认 ${MANIFEST_ENV} 或 {DEFAULT_MANIFEST}）")
    ap.add_argument("--write", action="store_true", help="写清单")
    ap.add_argument("--check", action="store_true", help="对清单（不给 --write 时默认）")
    a = ap.parse_args(argv)
    if a.write and a.check:
        print("❌ --write 与 --check 只能给一个")
        return 2
    return cmd_write(a) if a.write else cmd_check(a)


if __name__ == "__main__":
    sys.exit(main())
