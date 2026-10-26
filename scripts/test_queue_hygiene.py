#!/usr/bin/env python3
"""队列卫生守卫 —— 把「该归档的归档了没有」从**自觉**变成**红灯**。

背景
====
`docs/待执行命令.md` 自订的规矩是「只放还没做的」「做完一条删一条、正文下沉仓外留档」。
没有守卫时这条规矩必然漂移：追加零成本、删除有心理负担，于是一份「待办」里长出
几百行打 ✅ 的历史记录（2026-10-26 清账前实测 3111 行，其中真正的 pending 不到 200 行）。

判什么
====
对 `docs/待执行命令.md` 的每一行：

  · 出现 ✅ / ☑ / ⛔ / ❌ 之一 ⇒ 红（已完成记录、纪律条款 —— 前者下沉仓外留档，
    后者属于 AGENTS.md §1，队列里不该有）；
  · 出现「已完成 / 已了结 / 作废」字样 ⇒ 红（同上；判据请写成正向表述）；
  · 结构检查：文件必须存在，且必须给出去处 —— 提到「当前状态.md」与
    「待执行命令-留档」（队列的退役区），否则红。

退出码（本仓约定 0 / 1 / 2）
====
  0 = 干净（打印总行数，只作信息，不设预算）
  1 = 有残留 / 结构缺失
  2 = 找不到队列文件

用法
====
    python3 scripts/test_queue_hygiene.py
    python3 scripts/test_queue_hygiene.py --file docs/待执行命令.md   # 默认即此
"""

import argparse
import os
import re
import sys

sys.dont_write_bytecode = True  # 防 .pyc 假绿（本仓约定）

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QUEUE = os.path.join(REPO, 'docs', '待执行命令.md')

MARKER_RX = re.compile(r'[✅☑⛔❌]')
VERDICT_RX = re.compile(r'已完成|已了结|作废')
STRUCTURE_NEEDS = ['当前状态.md', '待执行命令-留档']


def _strip_code_spans(line):
    """去掉行内代码段（`...`）再判 —— 队列头部会示范节头格式（`` `☑ 日期 …` ``），
    那是「要往留档里写什么」的说明，不是本文件里的完结记录。真正的 ✅ 记录
    从不写在反引号里，所以剥掉代码段不会漏判。"""
    return re.sub(r'`[^`]*`', '', line)


def main():
    ap = argparse.ArgumentParser(description='队列卫生守卫')
    ap.add_argument('--file', default=QUEUE)
    args = ap.parse_args()

    if not os.path.isfile(args.file):
        print(f'找不到队列文件 {args.file}', file=sys.stderr)
        return 2

    with open(args.file, encoding='utf-8') as fh:
        lines = fh.read().splitlines()

    print('=' * 70)
    print(f'  队列卫生检查 —— {os.path.relpath(args.file, REPO)}（共 {len(lines)} 行）')
    print('=' * 70)

    hits = []
    for lineno, line in enumerate(lines, 1):
        bare = _strip_code_spans(line)
        for rx, why in ((MARKER_RX, '已完成/纪律标记'), (VERDICT_RX, '完结字样')):
            if rx.search(bare):
                hits.append((lineno, why, line.strip()))
                break

    text = '\n'.join(lines)
    missing = [s for s in STRUCTURE_NEEDS if s not in text]

    for lineno, why, line in hits:
        print(f'  ✗ :{lineno}  [{why}]  {line[:80]}')
    for s in missing:
        print(f'  ✗ 结构缺失：队列里没有提到「{s}」—— 退役/现状的出路必须写在文件里')

    print('=' * 70)
    if hits or missing:
        print(f'FAILED: {len(hits)} 行残留，{len(missing)} 处结构缺失。')
        print('  已完成的条目：整段删掉，正文追加到仓外 '
              '~/robotics/notes/ur5-vision-grasp/待执行命令-留档.md（带「☑ 日期 队列清账」节头）。')
        print('  纪律条款：移入 AGENTS.md §1；现状描述：移入 docs/当前状态.md。')
        return 1
    print('干净：队列里只有还没做的事。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
