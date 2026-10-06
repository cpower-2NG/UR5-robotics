#!/usr/bin/env python3
"""真机启动文档守卫 —— 防止「启动序列」再次分叉成七八份互相矛盾的说法。

为什么要它
==========
本仓有五份以上文档各讲一套「真机怎么起」：README、阶段 1 技术文档、机械臂系统
参考、TECHNICAL_SPEC、两块 board 文档，加上 `docs/待执行命令.md` 的 A1 一节。
其中**只有 A1 那一节真机跑通过**（2026-10-05，4/4）。2026-10-06 把它们收敛成
「A1 是唯一权威序列，其余改成指针」，代价是十三条互相矛盾的说法 —— 而**光靠
自觉，指针迟早在某次编辑里重新长成第二份参数集**。

它判五条
========
  1. **参数集对齐（核心）**：A1 一节里的每一条 `ros2 launch` 命令，它的
     `key:=value` **键集合**必须与启动器 `scripts/board_a1.py::build_cmd()`
     实际下发的键集合**逐项相等**（允许的差别只有一处：只走位那一段没有
     `start_delay`）。值也要比对，只有三个「随调用而变」的键例外：
     `robot_ip` / `planner_params_file` / `start_delay`。
  2. **入口不许写错**：受约束的文档里**不得**出现
     `pipeline.launch.py … camera_source:=realsense` 这种调用 —— 那是 D45，
     起来的规划器解不出任何 IK 且不报错。（引用块里的「别这么起」不受此限。）
  3. **`robot_ip` 不许漏**：受约束的文档里每一条 `camera_source:=realsense`
     的启动命令都必须带 `robot_ip:=`，否则 `real.launch.py::_check` 直接抛错。
  4. **`ROS_DOMAIN_ID=99` 不许漏**：A1 一节的起栈块必须显式 `export
     ROS_DOMAIN_ID=99`。板子上这条漏了会看到「话题列表是空的」，极容易误判。
  5. **指针不许缺**：任何一份写了真机启动命令的文档，必须同时提到
     `待执行命令.md`（也就是「权威序列在那边」）。这条防的是「第六份参数集」。

⚠️ 本脚本**自带变异测试**（同 `test_board_a1.py` 的先例）：判据函数是纯函数，
   跑完真文件之后立刻拿五份**故意改坏**的文本再跑一遍，**每一份都必须判红**。
   一个只会说 PASS 的守卫等于没有守卫 —— 而「绿的测试可能是因为判据没在看而绿」
   在本仓已经发生过（见 `test_docs_map.py` 头部第 4 条的来历）。

用法
====
    python3 scripts/test_startup_docs.py
退出码 0 = 全过，1 = 有问题（逐条打印），2 = 环境不满足（读不到文件等）。
**只要 Python 标准库**，不需要 source ROS，不需要真机、板子或串口。
"""

import importlib.util
import os
import re
import sys

sys.dont_write_bytecode = True  # 防 .pyc 假绿（本仓约定）

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.join(REPO, 'scripts')

RUNBOOK = os.path.join('docs', '待执行命令.md')
# A1 一节的起点；终点是它之后的第一个二级标题。
RUNBOOK_SECTION_RX = re.compile(r'^##\s+🟢\s*A1\s')
H2_RX = re.compile(r'^##\s')
HEADING_RX = re.compile(r'^#{1,6}\s')

# 受本守卫约束的「启动文档」集合。**故意写死**，不自动扫全仓：
# 收哪些是人的判断（改这份清单要过脑），漏了谁由 `test_docs_map.py` 的另一条
# 判据兜底（每份文档恰好登记一次）——不是本脚本的活。
STARTUP_DOCS = [
    RUNBOOK,
    'README.md',
    'docs/TECHNICAL_SPEC.md',
    'docs/机械臂系统参考.md',
    'docs/阶段/阶段1-点对点抓放/技术文档.md',
    'docs/board/UR5接入方案.md',
    'docs/board/机械臂板卡-体检与改造清单.md',
]

FENCE_RX = re.compile(r'^\s*```')
LAUNCH_RX = re.compile(r'ros2\s+launch\s+yolo_bringup\s+(\S+)')
ARG_RX = re.compile(r'([A-Za-z_][A-Za-z0-9_]*):=(\S+)')

REAL_CAMERA = 'camera_source:=realsense'
# 这三个键的「值」随调用而变，不参与值比对（键仍然必须出现）。
FREE_KEYS = {'robot_ip', 'planner_params_file', 'start_delay'}

# 唯一允许的键集合差异：只走位那一段没有 `start_delay`。
WALK_ONLY = 'start_delay'


# ---------------------------------------------------------------- 解析（纯函数）

def fenced_blocks(text, offset=0):
    """取出所有围栏代码块的**内部**行。返回 [(绝对起始行号, [行, ...]), ...]。"""
    out, cur, start = [], None, 0
    for i, raw in enumerate(text.splitlines(), 1):
        if FENCE_RX.match(raw):
            if cur is None:
                cur, start = [], i + 1
            else:
                out.append((offset + start, cur))
                cur = None
            continue
        if cur is not None:
            cur.append(raw)
    if cur is not None:          # 围栏没关：不静默吞掉，照样交出去
        out.append((offset + start, cur))
    return out


def logical_lines(start, lines):
    """把以 `\\` 结尾的续行接成一条逻辑行。返回 [(绝对行号, 整条文本), ...]。"""
    out, buf, first = [], None, start
    for off, raw in enumerate(lines):
        s = raw.rstrip()
        if buf is None:
            first, buf = start + off, s
        else:
            buf += ' ' + s.lstrip()
        if buf.rstrip().endswith('\\'):
            buf = buf.rstrip()[:-1]
            continue
        out.append((first, buf.strip()))
        buf = None
    if buf is not None:
        out.append((first, buf.strip()))
    return out


def launch_cmds(text, offset=0, skip_blockquote=False):
    """抽出文档里所有 `ros2 launch yolo_bringup …` 命令。

    `skip_blockquote=True` 时忽略引用块（`>` 开头）里的行 —— 那里是「别这么起」
    的警告，引用它正是我们希望发生的事。
    返回 [(行号, 入口文件名, {键: 值})]。
    """
    cmds = []
    for start, lines in fenced_blocks(text, offset):
        for ln, one in logical_lines(start, lines):
            if skip_blockquote and one.lstrip().startswith('>'):
                continue
            m = LAUNCH_RX.search(one)
            if m:
                cmds.append((ln, m.group(1), dict(ARG_RX.findall(one))))
    return cmds


def norm_value(key, value):
    """把「随调用而变」的值归一，好让两边可比。"""
    if key == 'robot_ip':
        return '<IP>'
    if key == 'planner_params_file':
        return os.path.basename(value.replace('$HOME', '~'))
    return value


def load_launcher():
    """导入 `scripts/board_a1.py`（只要标准库，导入无副作用）。

    读**真值**而不是把参数抄一份进本脚本 —— 抄一份就是给自己再造一个会烂的副本。
    """
    path = os.path.join(HERE, 'board_a1.py')
    spec = importlib.util.spec_from_file_location('board_a1_for_guard', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _LauncherArgs(object):
    """喂给 `build_cmd()` 的站位参数。值只影响 FREE_KEYS，故随便取。"""
    robot_ip = '<IP>'
    params = '~/site_real.yaml'
    start_delay = 120.0


def launcher_param_set(mod):
    """启动器实际会下发的 (入口文件, 键集合, {键: 归一值})。"""
    cmd = mod.build_cmd(_LauncherArgs())
    text = ' '.join(cmd)
    m = LAUNCH_RX.search(text)
    params = dict(ARG_RX.findall(text))
    return m.group(1), set(params), {k: norm_value(k, v) for k, v in params.items()}


def runbook_section(text):
    """切出 A1 一节（含起始行号）。切不出来返回 (None, 0)。"""
    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if RUNBOOK_SECTION_RX.match(line):
            start = i
            break
    if start is None:
        return None, 0
    for j in range(start + 1, len(lines)):
        if H2_RX.match(lines[j]):
            return '\n'.join(lines[start:j]), start + 1
    return '\n'.join(lines[start:]), start + 1


# ---------------------------------------------------------------- 判据（纯函数）

def check_runbook(section, offset, entry, keys, values):
    """A1 一节的判据。`section` 为 None 表示没切到这一节。"""
    bad = []
    if section is None:
        return ['切不出 A1 一节（找不到 `## 🟢 A1` 标题）—— 守卫失效，'
                '别把「没找到」当成「没问题」']

    cmds = [c for c in launch_cmds(section, offset)]
    if len(cmds) < 3:
        bad.append(f'A1 一节只解析出 {len(cmds)} 条 launch 命令，少于 3 条'
                   '（第一段 / 第二段 / 录像各一条）—— 解析结果可疑，先修守卫')

    seen_keys = set()
    for ln, launch_file, params in cmds:
        got = set(params)

        # ① 入口
        if launch_file != entry:
            bad.append(f'{RUNBOOK}:{ln} 入口是 `{launch_file}`，'
                       f'应为 `{entry}`')

        # ② 键集合
        if got != keys and got != keys - {WALK_ONLY}:
            missing = sorted(keys - got)
            extra = sorted(got - keys)
            detail = []
            if missing:
                detail.append(f'少 {missing}')
            if extra:
                detail.append(f'多 {extra}')
            bad.append(f'{RUNBOOK}:{ln} 参数键集合与启动器不一致：'
                       f'{"、".join(detail)}（启动器共 {len(keys)} 个键）')
        seen_keys |= got

        # ③ 值
        for k, v in sorted(params.items()):
            if k in FREE_KEYS or k not in values:
                continue
            want = values[k]
            # 只走位那一段（没有 start_delay）本来就不该自动抓 —— 这是**唯一**
            # 允许偏离启动器的地方，而且是往安全的一侧偏。
            if k == 'carry_objects' and WALK_ONLY not in got:
                if v != 'false':
                    bad.append(f'{RUNBOOK}:{ln} 只走位那一段（无 `{WALK_ONLY}`）'
                               f'应写 `carry_objects:=false`，实为 `{v}`')
                continue
            if norm_value(k, v) != want:
                bad.append(f'{RUNBOOK}:{ln} `{k}` 是 `{v}`，'
                           f'启动器下发的是 `{want}`')

        # ④ 真机必须给 robot_ip
        if REAL_CAMERA in ' '.join(f'{k}:={v}' for k, v in params.items()) \
                and 'robot_ip' not in got:
            bad.append(f'{RUNBOOK}:{ln} 是真机命令却没有 `robot_ip:=`')

    if seen_keys and seen_keys != keys:
        bad.append(f'A1 一节所有 launch 块合起来仍缺参数：'
                   f'{sorted(keys - seen_keys)}')

    # ⑤ ROS_DOMAIN_ID 必须显式导出
    if 'ROS_DOMAIN_ID=99' not in section:
        bad.append(f'{RUNBOOK}: A1 一节里没有 `ROS_DOMAIN_ID=99` —— 现场跑在 99，'
                   '漏了会看到「话题列表是空的」并被误判成「栈没起来」')
    return bad


def check_startup_doc(path, text, skip_blockquote=True):
    """非 runbook 的启动文档：入口、robot_ip、指针三条。"""
    bad = []
    cmds = launch_cmds(text, skip_blockquote=skip_blockquote)
    has_real = False

    for ln, launch_file, params in cmds:
        joined = ' '.join(f'{k}:={v}' for k, v in params.items())
        if REAL_CAMERA not in joined:
            continue
        has_real = True
        if launch_file.endswith('pipeline.launch.py'):
            bad.append(f'{path}:{ln} 把 `pipeline.launch.py` 当真机入口 '
                       f'（D45：没有驱动 / MoveIt / TF，规划器解不出 IK 且不报错）')
        if 'robot_ip' not in params:
            bad.append(f'{path}:{ln} 是真机命令却没有 `robot_ip:=`'
                       f'（`real.launch.py::_check` 会直接抛错，什么都不起）')

    if has_real and '待执行命令.md' not in text:
        bad.append(f'{path} 写了真机启动命令，却没有指向 `docs/待执行命令.md` '
                   f'的 A1 一节 —— 那等于又开了一份参数集的头')
    return bad, has_real


# ---------------------------------------------------------------- 变异测试

MUTATIONS = [
    ('少一个参数（删掉 tf_required:=true）',
     lambda s: re.sub(r'tf_required:=true\s*\\?\n?', '', s, count=1)),
    ('多一个参数（加 foo:=1）',
     lambda s: s.replace('publish_camera_tf:=true',
                         'publish_camera_tf:=true foo:=1', 1)),
    # ⚠️ 这一条**故意不带 `count=1`**：那个字面量在 A1 一节的**文案里也被引用**
    # （2026-10-06 新增的那条「下面三处 `camera_x:=…` 已作废」的提醒就引了它）。
    # 按「第一处」替换会打在**文案**上、三个命令块一字不动 ⇒ 变异体存活，
    # 而报告会把它说成「判据没在看这件事」——**反过来诬告判据**。
    # 2026-10-06 实际发生了一次。替换全部出现处，保证变异落在命令块上。
    ('值改了（camera_x 少一位）',
     lambda s: s.replace('camera_x:=0.34694477757210807',
                         'camera_x:=0.3469447775721081')),
    ('入口换成 pipeline.launch.py',
     lambda s: s.replace('ros2 launch yolo_bringup bringup.launch.py',
                         'ros2 launch yolo_bringup pipeline.launch.py', 1)),
    ('删掉 robot_ip:=<控制箱IP>',
     lambda s: re.sub(r'robot_ip:=\S+', 'robot_ip:=', s, count=1)),
]


DOC_MUTATIONS = [
    ('真机命令漏 robot_ip',
     lambda s: s.replace('camera_source:=realsense robot_ip:=<控制箱IP>',
                         'camera_source:=realsense', 1),
     lambda probs: any('robot_ip' in p for p in probs)),
    ('把 pipeline.launch.py 当真机入口',
     lambda s: s.replace('bringup.launch.py \\\n    camera_source:=realsense',
                         'pipeline.launch.py \\\n    camera_source:=realsense', 1),
     lambda probs: any('D45' in p for p in probs)),
    ('删掉指向权威序列的指针',
     lambda s: s.replace('待执行命令.md', '别的文件.md'),
     lambda probs: any('A1 一节' in p for p in probs)),
]


def doc_mutation_selftest(sample_path, sample_text):
    """非 runbook 文档的三条判据，也要各自有一条真反例。"""
    print('\n[变异测试·文档级] 三条判据各给一个真反例：')
    ok = True

    probs, has_real = check_startup_doc(sample_path, sample_text)
    if probs or not has_real:
        ok = False
        print(f'  ✗ 对照组 {sample_path} 就判红了（{probs}）—— 先修真问题')
        return ok
    print(f'  ✓ 对照组：{sample_path} 判绿（且认得出它含真机命令）')

    for name, mutate, hit in DOC_MUTATIONS:
        mutated = mutate(sample_text)
        if mutated == sample_text:
            ok = False
            print(f'  ✗ {name}：改坏没生效（夹具与文本对不上）—— 变异无效')
            continue
        probs, _ = check_startup_doc(sample_path, mutated)
        if probs and hit(probs):
            print(f'  ✓ {name} → 判红（{probs[0][:70]}…）')
        else:
            ok = False
            print(f'  ✗ {name} → 判绿或没打中该条判据（得到 {probs}）')
    return ok


def mutation_selftest(section, offset, entry, keys, values, n_cmds):
    """把 A1 一节故意改坏五遍，每一遍都必须判红。"""
    print('\n[变异测试] 判据函数必须对下列五种改坏都判红：')
    ok = True

    if check_runbook(section, offset, entry, keys, values):
        ok = False
        print('  ✗ 未改坏的原文就判红了 —— 先修真问题，变异测试没有意义')
        return ok
    print(f'  ✓ 对照组：未改坏的原文判绿（解析出 {n_cmds} 条 launch 命令）')

    base_cmds = launch_cmds(section, offset)
    for name, mutate in MUTATIONS:
        mutated = mutate(section)
        if mutated == section:
            ok = False
            print(f'  ✗ {name}：**这次改坏根本没生效**（文本没变）—— 变异无效')
            continue
        # 「文本变了」还不够，还要**打在 launch 命令上**。命令块与命令块之间夹着
        # 大段文案，而文案里引用了同一个字面量时，按第一处替换的变异会去改文案、
        # 命令块一字不动 —— 变异体因此存活，报告却写成「判据没在看这件事」，
        # **把守卫自己的失手说成判据的缺陷**。2026-10-06 的 `camera_x` 那条就这么
        # 假死过一次。这条断言专防它：变异没落到命令上，就报「变异无效」，
        # 既不放过它、也不诬告判据。
        if launch_cmds(mutated, offset) == base_cmds:
            ok = False
            print(f'  ✗ {name}：**变异没打在 launch 命令上**（解析出的命令一字未变，'
                  f'多半打在文案里了）—— 这次变异无效，别读成判据通过')
            continue
        problems = check_runbook(mutated, offset, entry, keys, values)
        if problems:
            print(f'  ✓ {name} → 判红（{problems[0][:70]}…）')
        else:
            ok = False
            print(f'  ✗ {name} → **竟然判绿了**，判据没在看这件事')
    return ok


# ---------------------------------------------------------------- 主流程

def read(path):
    with open(os.path.join(REPO, path), encoding='utf-8') as fh:
        return fh.read()


def main():
    try:
        runbook_text = read(RUNBOOK)
    except OSError as exc:
        print(f'环境不满足：读不到 {RUNBOOK}（{exc}）')
        return 2

    mod = load_launcher()
    entry, keys, values = launcher_param_set(mod)
    print(f'[0/3] 启动器真值（{os.path.join("scripts", "board_a1.py")}::build_cmd）：')
    print(f'      入口 `{entry}`，{len(keys)} 个参数键')
    print(f'      {", ".join(sorted(keys))}')

    problems = []

    # ---- A1 一节 ----
    section, offset = runbook_section(runbook_text)
    n_cmds = len(launch_cmds(section, offset)) if section else 0
    print(f'\n[1/3] runbook（{RUNBOOK} 的 A1 一节）：{n_cmds} 条 launch 命令'
          f'（起始行 {offset}）')
    problems += check_runbook(section, offset, entry, keys, values)

    # ---- 其余启动文档 ----
    print(f'[2/3] 其余启动文档：入口 / robot_ip / 指针')
    n_real = 0
    for path in STARTUP_DOCS:
        if path == RUNBOOK:
            continue
        try:
            text = read(path)
        except OSError as exc:
            problems.append(f'{path}: 读不到（{exc}）')
            continue
        bad, has_real = check_startup_doc(path, text)
        n_real += bool(has_real)
        problems += bad
        print(f'      {"·" if has_real else "○"} {path}'
              f'{"（含真机启动命令）" if has_real else "（无真机启动命令）"}')

    # ---- 变异测试 ----
    print('[3/3] 变异测试')
    if section is None:
        problems.append('变异测试跳过：切不出 A1 一节')
        selftest_ok = False
    else:
        selftest_ok = mutation_selftest(section, offset, entry, keys, values, n_cmds)
        # 文档级判据的夹具用 README —— 它同时含真机命令与指向权威序列的指针。
        try:
            selftest_ok &= doc_mutation_selftest('README.md', read('README.md'))
        except OSError as exc:
            selftest_ok = False
            print(f'  ✗ 文档级变异测试读不到 README.md（{exc}）')

    # ---- 结论 ----
    print('\n' + '=' * 68)
    if not problems and selftest_ok:
        print(f'PASS：A1 一节的参数集与启动器一致（{len(keys)} 个键）；'
              f'{len(STARTUP_DOCS) - 1} 份启动文档里 {n_real} 份含真机命令、'
              f'全部带 robot_ip 且指向权威序列；变异测试 8/8 判红。')
        return 0
    print(f'FAILED: {len(problems)} 处问题'
          + ('' if selftest_ok else '（变异测试未全过）') + '\n')
    for msg in problems:
        print(f'  ✗ {msg}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
