#!/usr/bin/env python3
"""文档地图守卫 —— 防止「地图」自己变成第三份烂掉的索引。

为什么要它
==========
`docs/文档地图.md` 存在的理由是：本仓的文档会**搬家、归档、改名**，而搬家之后
「原来那份去哪了」只有地图知道。可是**索引本身最容易烂** —— 本仓已经有两份
烂掉的索引了（README 的目录结构段、`RESTRUCTURE_PLAN.md` §1.4 的缺陷表，
后者冻结在第七轮）。所以地图必须由机器把守，不能靠自觉。

它判三条
========
  1. **全覆盖**：每一份受跟踪的 `.md`（含根目录的 `README.md` / `AGENTS.md` /
     `CLAUDE.md`）在映射表里**恰好出现一次**。少了 = 有人加了文档没说；
     多了 = 有人删了文档没清地图。
  2. **层声明一致**：文档自己若写了 `层级：X`，必须与地图里给它标的层一致。
  3. **相对链接可达**：全仓 `.md` 里的本地相对链接，目标必须存在。
     （自动跳过 http(s) / mailto / 纯锚点；代码块里的链接不算。）
  4. **反引号里的路径提及也有效**：`` `docs/xxx.md` `` 这种**不是链接**，但同样是
     引用。目标不存在、又不在本文档的别名表里 ⇒ 报错。

⚠️ 第 3、4 条正是**搬家之后**最容易漏的：文件挪了位置，引用它的一堆文档不会自己改。
   本仓历史上引用链很密（`机械臂系统参考.md` §8 是一张索引表），漏一处就是断链。

   ⚠️ **第 4 条是本脚本第一次跑之后才补上的** —— 只查 markdown 链接时它报「无断链」，
   而实际上被搬走的两份文档在 §8 索引表里**还留着旧路径**（写在反引号里，不是链接）。
   「绿的测试可能是**因为判据没在看**而绿」，这条就是这么发现的。

用法
====
    python3 scripts/test_docs_map.py
退出码 0 = 全过，1 = 有问题（逐条打印）。
"""

import os
import re
import subprocess
import sys
from collections import Counter

sys.dont_write_bytecode = True  # 防 .pyc 假绿（本仓约定）

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_DOC = os.path.join('docs', '文档地图.md')

MAP_COLUMNS = ['文件', '层', '阶段', '现行', '动作', '旧文件名别名']
ALIAS_COLUMNS = ['旧路径', '新路径']
WHITELIST_COLUMNS = ['出处', '引用', '为什么不算错']
VALID_LAYERS = {'恒久', '意图', '状态', '流水'}

# 自动生成的、或不该进地图的东西
EXCLUDE_MD = {
    MAP_DOC,
    'CLAUDE.md',          # 只有一行 @AGENTS.md，是入口的转发，不是文档
}

# 文档自己声明层级用的写法：`层级：恒久` 或 `> **层级**：恒久层`
LAYER_DECL_RX = re.compile(r'层级\*{0,2}\s*[:：]\s*\*{0,2}\s*(恒久|意图|状态|流水)')

LINK_RX = re.compile(r'\[[^\]]*\]\(\s*<?([^)>\s]+)>?(?:\s+"[^"]*")?\s*\)')
# 反引号里的 `docs/xxx.md`（可能带 `:行号`）—— 不是链接，但一样是引用。
MENTION_RX = re.compile(r'`([A-Za-z0-9_./一-鿿-]+\.md)(?::\d+)?`')
FENCE_RX = re.compile(r'^```')

# 只有「看起来就是本仓路径」的提及才检查；裸文件名（如 `README.md`）歧义太大，跳过。
ROOT_DOCS = {'AGENTS.md', 'README.md', 'CLAUDE.md'}


def _is_repo_path(p):
    return p.startswith('docs/') or p.startswith('ros2_ws/') or p in ROOT_DOCS


def git_ls_md():
    out = subprocess.run(['git', '-C', REPO, 'ls-files', '-z', '*.md'],
                         capture_output=True, text=True, check=False)
    if out.returncode != 0:
        raise SystemExit(f'git ls-files 失败: {out.stderr.strip()}')
    return [p for p in out.stdout.split('\0') if p]


def strip_fences(text):
    """去掉 ``` 围起来的代码块 —— 里面的链接不是链接。

    ⚠️ **围栏行要换成空行，不能删掉**：早先这里是 `continue`，于是返回文本的
    行号比原文件小，报出来的 `文件:行号` **指不到真正那一行**（本脚本第二次跑
    时就这样误导过一次 —— 按报的行号去看，看到的是不相干的内容）。
    保修的行号也是判据的一部分。
    """
    out, infence = [], False
    for line in text.splitlines():
        if FENCE_RX.match(line.strip()):
            infence = not infence
            out.append('')
            continue
        out.append('' if infence else line)
    return '\n'.join(out)


def cells(line):
    return [c.strip() for c in line.strip().strip('|').split('|')]


def parse_table(text, columns):
    """从地图里取出表头为 columns 的那张表 → [(行号, [单元格...])]。"""
    lines = text.splitlines()
    rows = []
    for i, line in enumerate(lines):
        if line.lstrip().startswith('|') and cells(line) == columns:
            j = i + 2  # 跳过表头与分隔行
            while j < len(lines) and lines[j].lstrip().startswith('|'):
                c = cells(lines[j])
                if len(c) == len(columns):
                    rows.append((j + 1, c))
                j += 1
            break
    return rows


def parse_map(text):
    return parse_table(text, MAP_COLUMNS)


def parse_aliases(text):
    """旧路径 → 新路径。

    两个来源：§2 映射表的「旧文件名别名」列，§4 搬迁记录的 旧/新 两列。
    **这就是别名表的用处** —— 「搬过家的要能在旧路径下找到」（地图开头原话），
    所以指向旧路径的引用**不该报断链**，只该被记成「经别名可达」。
    """
    alias = {}
    for _, c in parse_map(text):
        old = c[5].strip('`').strip()
        if old and old not in ('—', '-', ''):
            alias[old] = c[0].strip('`').strip()
    for _, c in parse_table(text, ALIAS_COLUMNS):
        old, new = c[0].strip('`').strip(), c[1].strip('`').strip()
        if old and new:
            alias[old] = new
    return alias


def parse_whitelist(text):
    """§6 白名单 → {(出处, 引用)}。

    **每条必须写理由**（本仓约定：白名单必须带原因，见 scripts/test_deidentified.py）。
    理由留空的行**不生效**，会在下面被当作普通失效引用报出来。
    """
    wl = set()
    for lineno, c in parse_table(text, WHITELIST_COLUMNS):
        src, ref, why = (x.strip('`').strip() for x in c)
        if not why or why in ('—', '-'):
            print(f'  ⚠️ {MAP_DOC}:{lineno} 白名单条目「{ref}」没写理由 ⇒ 不生效')
            continue
        wl.add((src, ref))
    return wl


def read(rel):
    with open(os.path.join(REPO, rel), encoding='utf-8', errors='replace') as fh:
        return fh.read()


def main():
    print('=' * 68)
    print('  文档地图守卫')
    print('=' * 68)

    problems = []

    map_path = os.path.join(REPO, MAP_DOC)
    if not os.path.isfile(map_path):
        print(f'找不到 {MAP_DOC}', file=sys.stderr)
        return 1

    rows = parse_map(read(MAP_DOC))
    if not rows:
        print(f'{MAP_DOC} 里找不到表头为 {MAP_COLUMNS} 的表', file=sys.stderr)
        return 1

    listed = {}
    for lineno, c in rows:
        # 表格里路径习惯写成 `code span`，比较前把反引号去掉
        path, layer = c[0].strip('`').strip(), c[1].strip('`').strip()
        if path in listed:
            problems.append(
                f'{MAP_DOC}:{lineno}  「{path}」重复出现'
                f'（上次在 :{listed[path][0]}）—— 一份文档只能占一行')
        listed[path] = (lineno, layer)
        if layer not in VALID_LAYERS:
            problems.append(
                f'{MAP_DOC}:{lineno}  「{path}」的层写成 {layer!r}，'
                f'应为 {sorted(VALID_LAYERS)} 之一')

    # ---- 1. 全覆盖 ----
    tracked = [p for p in git_ls_md()]
    expected = [p for p in tracked if p not in EXCLUDE_MD]
    missing = [p for p in expected if p not in listed]
    extra = [p for p in listed if p not in expected]
    print(f'[1/4] 受跟踪 .md 共 {len(tracked)} 份'
          f'（排除 {len(EXCLUDE_MD)} 份自动/转发）⇒ 应入地图 {len(expected)} 份')
    for p in missing:
        problems.append(f'「{p}」没有被登记进 {MAP_DOC}')
    for p in extra:
        problems.append(
            f'{MAP_DOC}:{listed[p][0]}  登记了「{p}」，但它不是一个受跟踪的 .md')

    # ---- 2. 层声明一致 ----
    print('[2/4] 核对各文档自报的层级')
    n_decl = 0
    for p in expected:
        try:
            body = read(p)
        except OSError:
            continue
        m = LAYER_DECL_RX.search(body)
        if not m:
            continue
        n_decl += 1
        declared = m.group(1)
        listed_layer = listed.get(p, (0, None))[1]
        if listed_layer and declared != listed_layer:
            problems.append(
                f'{p} 自报「层级：{declared}」，但地图登记的是「{listed_layer}」')
    print(f'      其中 {n_decl} 份写明了层级')

    map_text = read(MAP_DOC)
    alias = parse_aliases(map_text)
    white = parse_whitelist(map_text)
    # §6 白名单表格**本身就在引用那些死路径**（它就是登记处），扫它必然自指。
    white_rows = {ln for ln, _ in parse_table(map_text, WHITELIST_COLUMNS)}
    via_alias, via_white = [], []

    def _ok(p, lineno, resolved):
        """目标不存在时，看它是否经别名或白名单合法。"""
        if resolved in alias and os.path.exists(os.path.join(REPO, alias[resolved])):
            if p != MAP_DOC:      # 别名登记处自己引用别名，不值得报
                via_alias.append((p, resolved, alias[resolved]))
            return True
        if (p, resolved) in white:
            via_white.append(f'{p}:{lineno}  {resolved}')
            return True
        return False

    # ---- 3. 相对链接可达 ----
    print('[3/4] 核对相对链接')
    n_links = 0
    for p in tracked:
        try:
            body = strip_fences(read(p))
        except OSError:
            continue
        base = os.path.dirname(p)
        for lineno, line in enumerate(body.splitlines(), 1):
            for m in LINK_RX.finditer(line):
                target = m.group(1).strip()
                if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:', target):
                    continue          # http: / https: / mailto: ...
                if target.startswith('#') or target.startswith('<'):
                    continue
                target = target.split('#', 1)[0]
                if not target:
                    continue
                n_links += 1
                resolved = os.path.normpath(os.path.join(base, target))
                if not os.path.exists(os.path.join(REPO, resolved)) \
                        and not _ok(p, lineno, resolved):
                    problems.append(
                        f'{p}:{lineno}  断链 → {target}'
                        f'（解析为 {resolved}，不存在）')
    print(f'      共检查 {n_links} 条本地相对链接')

    # ---- 4. 反引号里的路径提及 ----
    n_mention = 0
    for p in tracked:
        try:
            body = strip_fences(read(p))
        except OSError:
            continue
        for lineno, line in enumerate(body.splitlines(), 1):
            if p == MAP_DOC and lineno in white_rows:
                continue
            for m in MENTION_RX.finditer(line):
                cand = m.group(1)
                if not _is_repo_path(cand):
                    continue
                n_mention += 1
                if os.path.exists(os.path.join(REPO, cand)) \
                        or _ok(p, lineno, cand):
                    continue
                problems.append(
                    f'{p}:{lineno}  路径提及已失效 → `{cand}` 不存在，'
                    f'也不在 {MAP_DOC} 的别名表 / 白名单里')
    print(f'[4/4] 共检查 {n_mention} 条反引号路径提及')

    # ---- 别名 / 白名单：**不藏起来** ----
    # 这两类不是错误，但必须打印出来，否则「经别名可达」会变成新的黑洞：
    # 一份文档指着早就搬走的旧路径，脚本说没问题，谁也不知道。
    if via_alias:
        print(f'\n  经别名可达（{len(via_alias)} 处，引用的是搬家前的旧路径）：')
        for (src, old, new), n in sorted(Counter((a, b, c) for a, b, c in via_alias).items()):
            print(f'    ↪ {src} ×{n}  {old} → {new}')
        print('    （这些不报错，但**不等于修好了**：指向旧路径的引用迟早要改；'
              '流水层的老记录可以不改。）')
    if via_white:
        print(f'\n  白名单放行（{len(via_white)} 处，理由见 {MAP_DOC} §6）：')
        for line in via_white:
            print(f'    · {line}')

    # ---- 结论 ----
    print('\n' + '=' * 68)
    if not problems:
        print('PASS：地图全覆盖、层声明一致、无断链'
              '（失效引用只剩别名与白名单两类，上面已逐条列出）。')
        return 0
    print(f'FAILED: {len(problems)} 处问题\n')
    for msg in problems:
        print(f'  ✗ {msg}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
