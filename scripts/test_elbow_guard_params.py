#!/usr/bin/env python3
"""`elbow_guard_probe.py` 读运行节点参数那一步的自检（不需要 ROS、不需要真机）。

（沿用本仓库 ``scripts/test_*.py`` 的约定：``check()`` + PASS/FAIL 计数，
退出码 0 = 通过，2 = 环境没准备好。``sys.dont_write_bytecode = True`` 防 .pyc 假绿。）

为什么值得单独钉
================
`elbow_guard_probe.py` 是**现场上电前的那道闸门**（退出码 0 = 可以动）。它的
护栏下限是

    floor = table_plane_z + guard_margin

而 `table_plane_z` 是**从运行中的规划器读来的**。于是「读参数」这一步一旦读错，
闸门松紧就跟着错，而**它自己是不会报错的** —— 它会照样打印一行「下限 -0.370」，
看上去像个正常结论。

真实踩到的（2026-10-05，板端）：原先的写法把**合法的 0** 当成「未声明」——

    if v in ([], 0, 0.0, "") and FALLBACK[name] not in ([], 0, 0.0, ""):
        v = FALLBACK[name]

而真机现场 yaml 里 `table_plane_z` 的**正确值恰好是 0.0**（台面就在 base 上），
于是被静默换成仿真兜底 **−0.39**，下限从 **+0.02** 变成 **−0.37**：
连杆压到台面以下 0.37 m 仍判「安全」。同一式子还吃掉
`elbow_guard_enabled: false`（`False == 0`）。

所以本文件钉两件事：
  1. **判据是 type**（`PARAMETER_NOT_SET`），不是「值看起来像空的」；
  2. 那几个「看起来像空」的值**必须**原样保留 —— 并且用**变异**证明
     第 1 条换了写法就会在这些输入上答错（否则本文件测不出东西）。

怎么在没有 rclpy 的机器上测
---------------------------
`elbow_guard_probe` 一 import 就拖进 rclpy / moveit_msgs，开发机上没有。
所以这里按 AST **只抽出 `pick` 那个纯函数**（它不依赖任何 ROS 类型），
单独 compile + exec 来测。变异同理：改它的源码文本再 exec 一遍。
"""

import ast
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'elbow_guard_probe.py')

N_PASS = N_FAIL = 0
FAILS = []

# 与 rcl_interfaces/ParameterType 对齐（只列本文件用得到的）
DOUBLE, BOOL, STRING_ARRAY = 3, 1, 9


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def load_pick(src_text):
    """从源码里抽出 `pick` 与 `NOT_SET`，exec 成一个可调用的函数。

    ⚠️ 只 exec **这一个函数定义**，不 import 整个模块 —— 那会把 rclpy 拖进来，
    本文件就再也不能在开发机上跑，而「上电前先跑得起来」正是它存在的理由。
    """
    tree = ast.parse(src_text)
    consts = {}
    fn = None
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 \
                and isinstance(n.targets[0], ast.Name) \
                and isinstance(n.value, ast.Constant):
            consts[n.targets[0].id] = n.value.value
        if isinstance(n, ast.FunctionDef) and n.name == 'pick':
            fn = n
    if fn is None:
        raise LookupError('源码里没有 pick()')
    if 'NOT_SET' not in consts:
        raise LookupError('源码里没有 NOT_SET')
    text = ast.get_source_segment(src_text, fn)
    ns = {'NOT_SET': consts['NOT_SET']}
    exec(compile(text, SRC, 'exec'), ns)               # noqa: S102
    return ns['pick'], text


def section_discriminating_inputs():
    """先把「哪些输入能分辨新旧写法」固定下来 —— 变异那节拿它们当尺子。"""
    print('\n[1] 能分辨新旧写法的输入（合法的零/假值）')
    # 真机现场 yaml 的真实取值：table_plane_z=0.0（台面就在 base 上）
    check('DOUBLE 0.0 是真值不是「未声明」（真机 table_plane_z 就是它）',
          (DOUBLE, 0.0, -0.39) is not None)
    # BOOL False：elbow_guard_enabled 关掉时就是这个值
    check('BOOL False 是真值不是「未声明」（护栏关掉时）',
          (BOOL, False, True) is not None)
    # 空字符串数组：真有可能被配上
    check('空数组是真值不是「未声明」', (STRING_ARRAY, [], [1.0]) is not None)


def section_behavior():
    print('\n[2] pick()：合法值原样返回，只有 PARAMETER_NOT_SET 才用兜底')
    src = open(SRC, encoding='utf-8').read()
    pick, _ = load_pick(src)

    check('table_plane_z：节点说 0.0 就是 0.0（**不能**变成兜底 -0.39）',
          pick(DOUBLE, 0.0, -0.39) == 0.0,
          '得到 %r —— 这正是 2026-10-05 那个 bug 的样子' % (pick(DOUBLE, 0.0, -0.39),))
    check('elbow_guard_enabled：节点说 False 就是 False（**不能**变成兜底 True）',
          pick(BOOL, False, True) is False,
          '得到 %r' % (pick(BOOL, False, True),))
    check('空数组原样返回（**不能**变成兜底）',
          pick(STRING_ARRAY, [], [1.0]) == [])
    check('type=PARAMETER_NOT_SET(0) 才用兜底',
          pick(0, 0.0, -0.39) == -0.39,
          '得到 %r' % (pick(0, 0.0, -0.39),))
    check('非零真值照常返回（防止把 pick 写成恒返回兜底）',
          pick(DOUBLE, 0.1757, 0.13) == 0.1757)

    # 判据必须是 type：拿同一个 value 换 type，结论必须跟着换
    check('同一个 value、换 type ⇒ 结论跟着换（证明它看的是 type）',
          pick(DOUBLE, 0.0, -0.39) != pick(0, 0.0, -0.39))


def _is_empty_literal(e):
    """`[]` / `()` / `{}` / `set()` 这种**空字面量**。

    ⚠️ 必须单独认它：`([], 0, 0.0, "")` 的第一个元素是 `ast.List` 而不是
    `ast.Constant`，只认 Constant 的话这条判据**一个都抓不到**，而源码里
    「恰好没有」时它照样是绿的 —— 正是本仓库反复强调的假绿。
    （第一版只认 Constant，被下面那条合成样本断言当场逮住。）
    """
    if isinstance(e, (ast.List, ast.Tuple, ast.Set)):
        return not e.elts
    if isinstance(e, ast.Dict):
        return not e.keys
    return False


def truthiness_compares(tree):
    """找出「`x in (...常量元组...)`」这种**真值判据**（返回节点列表）。

    ⚠️ 必须按 AST 判，不能 `'...' not in src` 扫原文：`pick` 的注释里**正当地**
    引用了那条历史写法（留着它才讲得清为什么改），扫原文会把注释也算命中 ——
    于是这条断言要么恒红、要么逼着人把解释删掉。第一版就是这么写的，
    自己把自己绊倒：断言红在「源码里有这句话」上，而代码里早已没有这个判据。
    """
    out = []
    for n in ast.walk(tree):
        if not isinstance(n, ast.Compare):
            continue
        if not any(isinstance(o, (ast.In, ast.NotIn)) for o in n.ops):
            continue
        for c in n.comparators:
            if isinstance(c, (ast.Tuple, ast.List)) and c.elts \
                    and all(isinstance(e, ast.Constant) or _is_empty_literal(e)
                            for e in c.elts):
                out.append(n)
    return out


def section_read_params_uses_pick():
    """结构断言：`read_params` 不许再自带一份「值看起来像空的」判据。"""
    print('\n[3] read_params 走 pick()，且源码里不再有真值判据')
    src = open(SRC, encoding='utf-8').read()
    tree = ast.parse(src)

    rp = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == 'read_params'), None)
    check('找得到 read_params', rp is not None)
    if rp is not None:
        calls = [n for n in ast.walk(rp) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'pick']
        check('read_params 里调用了 pick()', len(calls) >= 1,
              '找到 %d 处' % len(calls))

    check('源码里没有 `x in (常量元组)` 这种真值判据',
          not truthiness_compares(tree),
          '还有 %d 处' % len(truthiness_compares(tree)))
    # 上一条会「扫了个空」吗？拿合成的一段源码反向确认它认得出
    synth = ast.parse('v = 1\nif v in ([], 0, 0.0, ""):\n    pass\n')
    check('上一条不是空转：合成的同款判据它认得出',
          len(truthiness_compares(synth)) == 1,
          '合成样本没被认出 —— 第 3 节那条是假绿')
    # 而注释里引用同一句话**不该**被算中（这正是第一版栽的地方）
    commented = ast.parse('x = 1\n# 以前写的是 if v in ([], 0, 0.0, ""):\n')
    check('注释里引用这句话不算命中（否则解释历史就得删）',
          truthiness_compares(commented) == [])


def section_mutation():
    print('\n[4] 变异：把旧判据塞回 pick()，第 2 节必须答错')
    src = open(SRC, encoding='utf-8').read()
    pick, fn_text = load_pick(src)

    anchor = 'if field_type == NOT_SET:'
    assert anchor in fn_text, \
        '变异锚点没找到 —— 断言锚点漂了，本节会空转'
    mutated = fn_text.replace(anchor, 'if value in ([], 0, 0.0, ""):')
    ns = {'NOT_SET': 0}
    exec(compile(mutated, SRC, 'exec'), ns)            # noqa: S102
    old = ns['pick']

    check('变异「换回真值判据」→ table_plane_z 的 0.0 被答成兜底 -0.39',
          old(DOUBLE, 0.0, -0.39) == -0.39,
          '变异后得到 %r —— 说明第 2 节那条断言测不出东西'
          % (old(DOUBLE, 0.0, -0.39),))
    check('变异「换回真值判据」→ guard_enabled 的 False 被答成 True',
          old(BOOL, False, True) is True,
          '变异后得到 %r' % (old(BOOL, False, True),))
    # 反过来确认修复版在这些输入上与变异版确实不同（不是两边都错）
    check('同一个输入，修复版与变异版结论不同（本节的尺子真的在量东西）',
          pick(DOUBLE, 0.0, -0.39) != old(DOUBLE, 0.0, -0.39))


def main():
    print('=' * 68)
    if not os.path.exists(SRC):
        print('SKIP  找不到 %s（退出码 2）' % SRC)
        return 2
    section_discriminating_inputs()
    section_behavior()
    section_read_params_uses_pick()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：\n  - ' + '\n  - '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
