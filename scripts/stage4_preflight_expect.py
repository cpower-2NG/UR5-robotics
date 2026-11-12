#!/usr/bin/env python3
"""从**本地检出**生成板端预检期望值（只重算 ``tree`` 两项，身份字段继承旧基线）。

为什么需要这份工具
------------------
预检的 ``--expect`` 是 fail-closed 的锚：没有它，编排器直接判红（「不知道板上
该是哪块板 / 哪棵树」）。而只要部署树变了一个字节 —— **新增一份文档也算** ——
旧基线就必然判红。手改 JSON 的风险是把 ``serial`` / ``stack`` 一起改坏，那等于
把「认亲」判据静默放宽。

这里只做一件事：用**与板上完全相同**的 `PREFLIGHT_PY` 扫本地检出，得到新的
``tree_sha256`` / ``tree_files``；``serial`` / ``wrist_usb3`` / ``stack`` /
``note`` 从上一份基线原样继承。本地检出就是 rsync 的源，所以「期望哈希」=
「即将部署上去的那棵树」，不是「板上现在是什么」（后者是 got，不在这里读）。

跑法（开发机，离线，不碰板子、不碰臂）::

    python3 scripts/stage4_preflight_expect.py --repo <worktree> \\
        --from <old-expect.json> --out <new-expect.json>

判据：退出码 0 = 写出；本地树扫到 0 个文件 / 旧基线缺 ``serial`` / 输出里
序列号为空，一律非 0（这类期望值写出去只会让预检假绿或永远红）。
"""

import argparse
import json
import os
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_round_run as rr                                    # noqa: E402

EXIT_OK, EXIT_BAD_ARGS = 0, 2


def scan_local_tree(repo):
    """用板上同一份哈希代码扫本地目录，只取 ``tree`` 两项。

    只拼 `PREFLIGHT_TREE_CODE`（全仓唯一实现），**不跑整份预检** —— 整份要开
    夹爪串口、查 dashboard，在本机只会白等超时（实测 28 s）。
    其它字段（serial / gripper / realsense）是**板子**的属性，本地扫会得到空值，
    这里不采用；身份字段从旧基线继承。
    """
    src = ("import hashlib\nimport json\nimport os\nimport sys\n"
           + rr.PREFLIGHT_TREE_CODE
           + "\nprint(json.dumps(scan_tree(sys.argv[1])))\n")
    r = subprocess.run([sys.executable, "-", repo],
                       input=src, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("本地树哈希失败（rc=%d）：%s"
                         % (r.returncode, r.stderr.strip()[-800:]))
    try:
        tree = json.loads(r.stdout)
    except ValueError:
        raise SystemExit("本地树哈希输出不是 JSON：%r"
                         % r.stdout.strip()[-400:])
    if not tree.get("files"):
        raise SystemExit("本地树扫到 0 个文件（--repo 指错了？）：%s" % repo)
    return tree


def build_expect(tree, old):
    if not old.get("serial"):
        raise SystemExit("旧基线没有 serial —— 认亲判据会失效，拒绝生成")
    out = dict(old)
    out["tree_sha256"] = tree["sha256"]
    out["tree_files"] = tree["files"]
    out.setdefault("wrist_usb3", True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="从本地检出重算预检期望值（tree 两项），身份字段继承旧基线")
    ap.add_argument("--repo", required=True, help="本地检出（= rsync 的源）")
    ap.add_argument("--from", dest="old", required=True,
                    help="上一份 preflight-expect.json（serial/stack 从它继承）")
    ap.add_argument("--out", required=True, help="写出的新期望值 JSON")
    a = ap.parse_args(argv)

    for p, what in ((a.old, "--from"), (a.out, "--out")):
        if not p:
            print("%s 为空" % what, file=sys.stderr)
            return EXIT_BAD_ARGS
    if not os.path.isdir(a.repo):
        print("--repo 不是目录：%s" % a.repo, file=sys.stderr)
        return EXIT_BAD_ARGS
    if not os.path.exists(a.old):
        print("--from 不存在：%s" % a.old, file=sys.stderr)
        return EXIT_BAD_ARGS

    old = json.load(open(a.old, encoding="utf-8"))
    tree = scan_local_tree(os.path.abspath(a.repo))
    expect = build_expect(tree, old)

    with open(a.out, "w", encoding="utf-8") as fh:
        json.dump(expect, fh, ensure_ascii=False, indent=1)
        fh.write("\n")
    print("写出 %s\ntree %s… / %d 文件（serial 指纹沿用旧基线）"
          % (a.out, expect["tree_sha256"][:12], expect["tree_files"]))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
