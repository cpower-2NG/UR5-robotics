#!/usr/bin/env python3
"""文档体量守卫 —— 防止「常驻文档」悄悄膨胀到挤占注意力。

为什么要它
==========
本仓的文档按**维护时钟**分四层（见 `docs/文档地图.md` §1）。问题不在文档多，
而在**该检索的被当常驻读**：AI 每次开工都要读的那几份，才是真正的注意力预算。
没有上限的文档一定膨胀 —— 本仓已经量到过：`docs/待执行命令.md` 涨到 4102 行，
而它**自己第 10 行**写着「这份文件超过两屏，就是该收拾了」。**它没有能力自己执行
自己写的规矩。** 所以要有这份脚本。

它判什么
========
每份「常驻文档」的行数不得超过它的预算。超了就是失败。

⚠️ **预算不是目标，是天花板。** 它不是要逼你把信息删掉，而是要你在超线时**做一次
分层判断**：这条内容真的该常驻吗？该进恒久层还是该进流水（仓库外）？
本仓的既有结论是「没人把守的索引已经烂掉两份」，所以这里也用机器把守，不靠自觉。

⚠️ **为什么只查这几份**：清单里的都是「AI 开工时要读」或「人会照着做」的文档。
`docs/机械臂系统参考.md`、`README.md`、仓外台账那些**按需检索**的不在此列 ——
它们长一点没关系，因为不进上下文。**别把这份清单扩成"所有文档"**，
那会让按需文档被迫删掉有用的细节。

怎么改预算
==========
改下面 `BUDGETS` 里对应的数字，并在同行把「为什么是这个数」改掉。
**理由留空或与数字对不上，下次就没人知道这数哪来的** —— 照本仓「白名单必须带理由」
的老规矩（见 `scripts/test_deidentified.py`）。

用法
====
    python3 scripts/test_doc_budget.py

退出码：`0` 通过 / `1` 有文档超预算。
"""

import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (相对路径, 行数上限, 为什么是这个数)
#
# ⚠️ 数字是**量出来的**，不是拍的：先看实测量到什么，再留一点余量，
#    而不是先定一个漂亮数字再逼文档去够。
BUDGETS = [
    ('AGENTS.md', 200,
     '恒久层入口，AI 每次接手都要读完。它只该缩不该涨 —— 涨说明有内容该挪去'
     '意图层（阶段相关的）或按需层（细节），而不是往里加。'),
    ('docs/当前状态.md', 140,
     '状态层唯一的家（见 docs/事实源登记.md）。它要**常驻**才能替代"到处翻"。'
     '超了通常是有人往里追加历史 —— 状态层是原地覆写，旧的应进仓外台账。'),
    ('docs/阶段/README.md', 60,
     '阶段梯子 + 契约模板。每进一个新阶段才动一次，正常不会涨。'),
    ('docs/文档地图.md', 220,
     '索引。它会随文档增删缓慢变长，属正常；但若涨得快，通常说明有人把'
     '**内容**写进了索引 —— 索引只该有"谁在哪"，不该有"内容是什么"。'),
    ('docs/待执行命令.md', 2100,
     '在办队列。2026-10-08 收拾过两次：先把「已了结」的章节整体搬进仓外留档'
     '（4102 → 2391），再把 A1 的现场 runbook 拆成独立一份（2391 → 2015）。'
     '**这个数仍是棘轮**，作用是不许再涨回去。⚠️ 它离自己第 11 行写的「两屏」'
     '仍有距离 —— 剩下的长块是若干「悬案 / 待裁」，不是可搬的过程记录。'
     '**别再靠放宽这个数解决问题**：它上次就是被这么放宽到 2400 的。'),
    ('docs/A1现场runbook.md', 420,
     '真机启动的**唯一权威序列**，人会拿着它站在机器旁边逐条敲，守卫 '
     'scripts/test_startup_docs.py 在运行期解析它。它**不该长**：一套验证过的'
     '动作序列的长度是固定的。涨了通常说明有人把「某一轮跑出来的结果」写进去了 '
     '—— 那属于状态层或仓外留档，不属于 runbook。'),
]


def main():
    print('=' * 70)
    print('  文档体量守卫 —— 常驻文档不得超预算')
    print('=' * 70)

    over, missing = [], []
    for path, budget, why in BUDGETS:
        full = os.path.join(REPO, path)
        if not os.path.exists(full):
            missing.append(path)
            print(f'  ?? {path}  —— 文件不存在')
            continue
        with open(full, encoding='utf-8') as fh:
            n = len(fh.read().splitlines())
        margin = budget - n
        mark = '✓' if margin >= 0 else '✗'
        print(f'  {mark} {path:<28} {n:>5} 行 / 上限 {budget:<5} '
              f'{"余 " + str(margin) if margin >= 0 else "超 " + str(-margin)}')
        if margin < 0:
            over.append((path, n, budget, why))

    print('\n' + '=' * 70)
    if over:
        print(f'FAILED：{len(over)} 份常驻文档超预算\n')
        for path, n, budget, why in over:
            print(f'  ✗ {path}：{n} 行，上限 {budget} 行（超 {n - budget}）')
            print(f'    这个上限的理由：{why}')
            print('    → 收拾办法：把"某一次的过程/结果"挪去仓库外流水层，'
                  '把"现在是什么"挪进 docs/当前状态.md（原地覆写），')
            print('      只留"以后一直这么做"的部分。**别为了凑行数删掉判据。**')
            print('    → 若确认该上限本身不合理，改 scripts/test_doc_budget.py '
                  '里的数字**并同时改理由**。')
        return 1
    if missing:
        print(f'注意：{len(missing)} 份文件不存在，已跳过：{"、".join(missing)}')
    print(f'PASS：{len(BUDGETS)} 份常驻文档都在预算内。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
