#!/usr/bin/env python3
"""状态层守卫 —— 把「哪些行过期了」从**判断**变成**计算**。

为什么需要它
============
`docs/当前状态.md` 是全仓唯一允许写「现在」的地方，代价是它**必然会过期**。
光靠「记得去看一眼」是不行的 —— 本仓为此已经付过代价：一份维护得最认真的
参考文档，它的现状段三天就烂光了，而其余部分完好（见 `docs/文档地图.md`）。

所以给它定一条机械规矩：

    每一行都带「上次核对日期」和「保质期」。**过了保质期 = 这一行不再算数。**

本脚本做的事，就是**把这条规矩算出来**，而不是留给读者去心算。
超期的行不会被删，只会被标红 —— 它仍然是有用的历史值，只是**不能当依据**。

判什么
======
对一个可核对行（五列：`项 | 取值命令 | 上次核对 | 保质期 | 结果`）：

  · `上次核对` 能不能解析成日期？解析不了 ⇒ **结构性错误**（退出码 1）；
  · `保质期` 能不能解析？解析不了 ⇒ **结构性错误**（退出码 1）；
  · `今天 - 上次核对 > 保质期` ⇒ **已超期**（打印出来；默认不影响退出码）。

为什么超期**不**直接让退出码非零
--------------------------------
因为超期是**常态**，不是失误 —— 俯视外参的保质期只有 1 天，隔一天跑一次
提交前检查就红，人会学会忽略它，闸门就废了。**结构性错误才是真错误**
（有人加了一行却忘了写日期/保质期，那一行永远无法被判过期，等于没写）。
想看超期也失败的，加 `--strict`（对着「这个阶段刚验证完」的场景用）。

退出码
======
  0 = 结构完好（可能有超期行，已列出）
  1 = 有结构性错误（行解析不了），或 `--strict` 下有超期行
  2 = 找不到 `docs/当前状态.md`

用法
====
    python3 scripts/state_check.py             # 报告
    python3 scripts/state_check.py --strict    # 超期也算失败
    python3 scripts/state_check.py --today 2026-10-20   # 换「今天」（便于测试）
"""

import argparse
import datetime as dt
import os
import re
import sys

sys.dont_write_bytecode = True  # 防 .pyc 假绿（本仓约定）

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATE_DOC = os.path.join(REPO, 'docs', '当前状态.md')

COLUMNS = ['项', '取值命令', '上次核对', '保质期', '结果']

DATE_RX = re.compile(r'(\d{4})-(\d{2})-(\d{2})')
SHELF_RX = re.compile(r'(\d+)\s*(小时|天|周|月)')
UNIT_DAYS = {'小时': 1 / 24, '天': 1, '周': 7, '月': 30}

# 这两类写法表示「不按时间过期」，不是解析失败。
NEVER_RX = re.compile(r'永久|不适用|N/?A|—|-')


def _cells(line):
    """把 `| a | b |` 拆成 ['a','b']。

    ⚠️ 必须**只认没被转义的 `|`**：取值命令里天然会带管道
    （`... 2>&1 \\| grep -i uvcdynctrl`），按裸 `|` 拆会把一行拆成六列，
    于是整行报「列数不对」。这正是本脚本第一次跑就抓到的错。
    """
    parts = re.split(r'(?<!\\)\|', line.strip())
    if parts and parts[0].strip() == '':
        parts = parts[1:]
    if parts and parts[-1].strip() == '':
        parts = parts[:-1]
    return [p.strip().replace('\\|', '|') for p in parts]


def _is_sep(line):
    return bool(re.fullmatch(r'\|[\s:|-]+\|', line.strip()))


def parse_tables(text):
    """找出所有表头正好是 COLUMNS 的表格，yield (起始行号, [行...])。"""
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        if lines[i].lstrip().startswith('|') and _cells(lines[i]) == COLUMNS:
            start = i + 1
            j = i + 1
            if j < len(lines) and _is_sep(lines[j]):
                j += 1
            body = []
            while j < len(lines) and lines[j].lstrip().startswith('|'):
                if not _is_sep(lines[j]):
                    body.append((j + 1, lines[j]))
                j += 1
            yield start, body
            i = j
        else:
            i += 1


def parse_date(s):
    m = DATE_RX.search(s or '')
    if not m:
        return None
    try:
        return dt.date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
    except ValueError:
        return None


def parse_shelf(s):
    """返回 (天数 | None, 说明)。None 天数 = 不过期或解析失败，看说明。"""
    s = (s or '').strip()
    if NEVER_RX.search(s):
        return None, '不过期'
    m = SHELF_RX.search(s)
    if not m:
        return None, '解析失败'
    return int(m.group(1)) * UNIT_DAYS[m.group(2)], 'ok'


def main():
    ap = argparse.ArgumentParser(description='状态层守卫：算过期，不靠记性')
    ap.add_argument('--strict', action='store_true',
                    help='有超期行也让退出码非零')
    ap.add_argument('--today', default=None,
                    help='把「今天」定成 YYYY-MM-DD（便于测试）')
    args = ap.parse_args()

    today = (parse_date(args.today) if args.today else None) or dt.date.today()

    if not os.path.isfile(STATE_DOC):
        print(f'找不到 {STATE_DOC}', file=sys.stderr)
        return 2

    with open(STATE_DOC, encoding='utf-8') as fh:
        text = fh.read()

    print('=' * 70)
    print(f'  状态层过期检查 —— {os.path.relpath(STATE_DOC, REPO)}')
    print(f'  以 {today} 为「今天」')
    print('=' * 70)

    struct_errors = []
    expired = []
    ok_rows = []
    n_tables = 0

    for start, body in parse_tables(text):
        n_tables += 1
        if not body:
            continue
        for lineno, line in body:
            cells = _cells(line)
            if len(cells) != len(COLUMNS):
                struct_errors.append(
                    (lineno, f'列数 {len(cells)}，应为 {len(COLUMNS)}'))
                continue
            name, _cmd, date_s, shelf_s, _res = cells
            d = parse_date(date_s)
            if d is None:
                struct_errors.append(
                    (lineno, f'「上次核对」解析不出日期：{date_s!r} —— '
                             f'这一行永远无法被判过期'))
                continue
            days, why = parse_shelf(shelf_s)
            if why == '解析失败':
                struct_errors.append(
                    (lineno, f'「保质期」解析不了：{shelf_s!r} —— '
                             f'写「永久」或「N 天 / N 周 / N 月」'))
                continue
            age = (today - d).days
            if days is None:
                ok_rows.append((lineno, name, d, age, '不过期'))
            elif age > days:
                expired.append((lineno, name, d, age, shelf_s))
            else:
                ok_rows.append((lineno, name, d, age, f'剩 {days - age:.1f} 天'))

    print(f'扫到 {n_tables} 张可核对表，共 {len(ok_rows) + len(expired)} 行\n')

    if ok_rows:
        print('── 未超期 ──────────────────────────────────────────────')
        for lineno, name, d, age, note in ok_rows:
            print(f'  ✓ :{lineno}  {name}   核对于 {d}（{age} 天前，{note}）')

    if expired:
        print('\n── 🔴 已超期：这些行**不得作为依据**，先跑核对命令 ──────────')
        for lineno, name, d, age, shelf in expired:
            print(f'  ✗ :{lineno}  {name}')
            print(f'        核对于 {d}（{age} 天前），保质期 {shelf}')
            print(f'        → 先跑该行的「取值命令」，再更新结果与日期')

    if struct_errors:
        print('\n── ⛔ 结构性错误：这些行永远不会被判过期 ──────────────────')
        for lineno, msg in struct_errors:
            print(f'  ✗ :{lineno}  {msg}')

    print('\n' + '=' * 70)
    if struct_errors:
        print(f'FAILED: {len(struct_errors)} 处结构性错误')
        return 1
    if expired and args.strict:
        print(f'FAILED（--strict）: {len(expired)} 行已超期')
        return 1
    if expired:
        print(f'结构完好，但有 {len(expired)} 行已超期（见上）。'
              f'不是错误，是提醒。')
    else:
        print('结构完好，且没有超期的行。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
