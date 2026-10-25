#!/usr/bin/env python3
"""`pipeline.launch.py` 的参数面自检：**声明 ↔ 引用 ↔ 节点声明**三边必须对齐。

为什么需要这条
==============
`pipeline.launch.py` 里写死了一条规则（文件自己的注释）：

    传给节点的每个参数，都要在本文件里声明。
    不声明的话现场传了 `depth_grid:=7` 会**静默无效** —— 未声明的参数会被
    launch 接受并塞进共享的 launch_configurations（实测），只是没有任何人读它。

这条规则靠人记是记不住的：节点长出一个新参数（2026-10-25 pc_top 一下加了
四个：`pc_min_score` / `pc_table_hint` / `color_topic` / `color_buffer_len`），
launch 忘了加，症状是**上板怎么传都没反应**，而日志里一切正常。

所以这里把三种静默失效都钉成红的：

  1. **节点 declare_parameter 了，launch 没传** → 现场调不了这个旋钮；
  2. **launch 传了，节点没声明** → 那个值谁都不读（还会让人以为调过了）；
  3. **`'a': LaunchConfiguration('b')` 键名与变量名不一致** → 传 `a:=` 无效，
     得传 `b:=`，而文档/台账里写的都是 `a`。
  4. **DeclareLaunchArgument 了却没人引用** → 与 1 相反的方向，同样是「传了没反应」。
  5. **LaunchConfiguration 引用了未声明的名字** → launch 直接抛错（这条是响的，
     但响在启动那一刻，不如在这里先红）。
  6. **两边的默认值不一致**（launch 的 default_value 与节点的 declare_parameter
     默认值）→ 不传参时用 launch 的值，于是「节点默认值」成了谎话。

范围
====
· 第 1/3/6 条对 **projector_node** 逐参数比（它的 parameters 是内联 dict，能
  用 AST 精确取到）。detector_node 大部分参数是字面量、planner_node 用的是
  `planner_params` 变量，AST 里取不到内联 dict —— 它们只受第 4/5 条（全文件）
  保护。将来谁把它们改成内联 dict，就在这里加进 NODES。
· 第 4/5 条对**整个文件**：凡是 `LaunchConfiguration('x')` 与
  `DeclareLaunchArgument('x')` 的名字集合必须相等。

用法
====
    python3 scripts/test_launch_param_surface.py
不需要 ROS 环境（纯文本 + AST）。退出码 0 = 全通过，1 = 有失败项。
"""

import ast
import os
import sys

# ⚠️ 与本目录其他自检同理：.pyc 的失效判据是 (mtime, size)，等长改动落在同一
# 秒里会被当「没变」。这里不 import 被测模块，但保持口径一致不算多余。
sys.dont_write_bytecode = True

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup",
                      "launch", "pipeline.launch.py")
NODES = {
    "projector_node": os.path.join(PROJECT_ROOT, "ros2_ws", "src",
                                   "perception_3d", "perception_3d",
                                   "projector_node.py"),
}

_failed = []


def check(name, problems, detail=""):
    ok = not problems
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        for p in problems:
            print(f"        · {p}")
        _failed.append(name)
    return ok


# ---------------------------------------------------------------- 解析（AST）
def _lit(node):
    """ast 常量 → Python 值；不是常量就返回哨兵。"""
    if isinstance(node, ast.Constant):
        return node.value
    return _NON_LITERAL


class _Sentinel:
    def __repr__(self):
        return "<非字面量>"


_NON_LITERAL = _Sentinel()


def _launch_config_name(call):
    """`LaunchConfiguration('x')` → 'x'；别的调用 → 哨兵。"""
    if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
            and call.func.id == "LaunchConfiguration" and call.args):
        return _lit(call.args[0])
    return _NON_LITERAL


def node_param_pairs(launch_src, exe):
    """launch 里 `Node(executable=exe, parameters=[{...}])` → {键: 变量名} 列表。

    返回 ``(pairs, problems)``；``pairs`` 是 ``[(键, 变量名或哨兵, 是否有字面量值)]``。
    找不到该节点、或 parameters 不是内联 dict 时，problems 里说清楚 ——
    静默返回空集合会让第 1 条**假绿**。
    """
    tree = ast.parse(launch_src)
    problems, pairs = [], []
    found = False
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        kw = {k.arg: k.value for k in call.keywords if k.arg}
        if _lit(kw.get("executable")) != exe:
            continue
        found = True
        params = kw.get("parameters")
        if not (isinstance(params, ast.List) and len(params.elts) == 1
                and isinstance(params.elts[0], ast.Dict)):
            problems.append(f"{exe}: parameters 不是内联 dict，本自检取不到参数面")
            continue
        d = params.elts[0]
        for k, v in zip(d.keys, d.values):
            key = _lit(k)
            if key is _NON_LITERAL:
                problems.append(f"{exe}: 有参数键不是字面量")
                continue
            pairs.append((key, _launch_config_name(v), v))
    if not found:
        problems.append(f"launch 里找不到 executable='{exe}' 的节点")
    return pairs, problems


def declared_args(launch_src):
    """全文件 `DeclareLaunchArgument('x', default_value=...)` → {x: 默认值或哨兵}。"""
    tree = ast.parse(launch_src)
    out = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "DeclareLaunchArgument" and call.args):
            name = _lit(call.args[0])
            if name is _NON_LITERAL:
                continue
            kw = {k.arg: k.value for k in call.keywords if k.arg}
            out[name] = _lit(kw.get("default_value"))
    return out


def referenced_args(launch_src):
    """全文件 `LaunchConfiguration('x')` → 名字集合。"""
    tree = ast.parse(launch_src)
    return {_launch_config_name(c) for c in ast.walk(tree)
            if _launch_config_name(c) is not _NON_LITERAL}


def node_declared_params(node_src):
    """`self.declare_parameter('x', 默认值)` → {x: 默认值或哨兵}。"""
    tree = ast.parse(node_src)
    out = {}
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == "declare_parameter" and call.args):
            name = _lit(call.args[0])
            if name is _NON_LITERAL:
                continue
            out[name] = _lit(call.args[1]) if len(call.args) > 1 else _NON_LITERAL
    return out


# ---------------------------------------------------------------- 判据
def _same_value(a, b):
    """默认值是否等价。

    难在 launch 的默认值一律是**字符串**（`default_value='0.3'`、
    `'false'`），节点那边是真 bool / float。所以先把字符串按对方的类型念
    一遍，念不出来才退回原样比 —— 直接 `a == b` 会把 36 个参数里的
    `tf_required`/`use_target_plane` 全判成不一致（实测第一次跑就撞上）。
    """
    if a is _NON_LITERAL or b is _NON_LITERAL:
        return True                      # 取不到就不判，别制造假红

    def conv(v, like):
        if isinstance(v, str) and isinstance(like, bool):
            low = v.strip().lower()
            if low in ("true", "false"):
                return low == "true"
        if isinstance(like, (int, float)) and not isinstance(like, bool):
            try:
                return float(v)
            except (TypeError, ValueError):
                return v
        return v

    a2, b2 = conv(a, b), conv(b, a)
    if isinstance(a2, bool) or isinstance(b2, bool):
        return a2 is b2
    try:
        return abs(float(a2) - float(b2)) < 1e-12
    except (TypeError, ValueError):
        return a2 == b2


def problems(launch_src, node_src_by_exe):
    """→ 问题列表（空 = 全对齐）。所有判据都在这里，变异测试打的就是它。"""
    out = []
    decl = declared_args(launch_src)
    ref = referenced_args(launch_src)

    # 4/5：全文件的名字集合必须相等
    out += [f"LaunchConfiguration('{n}') 没有对应的 DeclareLaunchArgument"
            for n in sorted(ref - set(decl))]
    out += [f"DeclareLaunchArgument('{n}') 声明了却没人引用（现场传了也没人读）"
            for n in sorted(set(decl) - ref)]

    for exe, node_src in node_src_by_exe.items():
        pairs, p = node_param_pairs(launch_src, exe)
        out += p
        keys = {k for k, _, _ in pairs}
        nparams = node_declared_params(node_src)

        # 1：节点声明了、launch 没传
        out += [f"{exe}: 节点声明了 {n} 但 launch 没传（现场调不动）"
                for n in sorted(set(nparams) - keys)]
        # 2：launch 传了、节点没声明
        out += [f"{exe}: launch 传了 {k} 但节点没 declare_parameter"
                for k in sorted(keys - set(nparams))]
        # 3：键名与变量名不一致
        out += [f"{exe}: '{k}' 键名与 LaunchConfiguration 变量名不一致"
                f"（变量名 {var!r}）"
                for k, var, _ in pairs
                if var is not _NON_LITERAL and var != k]
        # 6：默认值不一致
        for k, var, _ in pairs:
            if var is _NON_LITERAL or var not in decl:
                continue
            if not _same_value(decl[var], nparams.get(k)):
                out.append(f"{exe}: {k} 默认值不一致 —— launch 声明 "
                           f"{decl[var]!r} / 节点声明 {nparams.get(k)!r}"
                           f"（不传参时用 launch 的，节点那份成了谎话）")
    return out


# ---------------------------------------------------------------- 变异测试
def mutations(launch_src, node_src):
    """(名字, 改后的 launch, 改后的 node, 期望在问题里出现的片段)。

    每一条都必须让 problems() 变红，且红在**对应的那条判据**上 ——
    否则判据是空转的（本仓库反复撞过：绿灯比红灯更危险）。
    """
    our = [l for l in launch_src.splitlines() if "pc_min_score" in l]
    assert len(our) >= 2, f"变异测试的前提变了：pc_min_score 只出现 {len(our)} 行"
    same_node = node_src
    return [
        # 5：声明被改名 → 引用悬空
        ("删掉 DeclareLaunchArgument('pc_min_score')",
         launch_src.replace(
             "DeclareLaunchArgument(\n            'pc_min_score'",
             "DeclareLaunchArgument(\n            'pc_score_unused'", 1),
         same_node,
         "LaunchConfiguration('pc_min_score') 没有对应的 DeclareLaunchArgument"),
        # 3：键与变量名错位（现场得传错名字才生效）
        ("键名与变量名错位",
         launch_src.replace(
             "'pc_min_score': LaunchConfiguration('pc_min_score')",
             "'pc_min_score': LaunchConfiguration('pc_score')", 1),
         same_node,
         "键名与 LaunchConfiguration 变量名不一致"),
        # 1：节点长出参数而 launch 没传
        ("节点多声明一个参数",
         launch_src,
         node_src.replace(
             "        self.declare_parameter('color_buffer_len', 5)",
             "        self.declare_parameter('color_buffer_len', 5)\n"
             "        self.declare_parameter('pc_new_knob', 1.0)", 1),
         "但 launch 没传"),
        # 6：默认值漂了
        ("launch 默认值与节点不一致",
         launch_src.replace("'pc_min_score', default_value='0.30'",
                            "'pc_min_score', default_value='0.35'", 1),
         same_node,
         "默认值不一致"),
        # 4：声明了没人用
        ("声明了却没人引用",
         launch_src.replace(
             "        OpaqueFunction(function=_guard_realsense_serial),",
             "        DeclareLaunchArgument('orphan_arg', default_value='1'),\n"
             "        OpaqueFunction(function=_guard_realsense_serial),", 1),
         same_node,
         "声明了却没人引用"),
    ]


def main():
    launch_src = open(LAUNCH, encoding="utf-8").read()
    node_src = {exe: open(p, encoding="utf-8").read() for exe, p in NODES.items()}

    print(f"launch: {os.path.relpath(LAUNCH, PROJECT_ROOT)}")
    print(f"声明 {len(declared_args(launch_src))} 个 / 引用 "
          f"{len(referenced_args(launch_src))} 个")

    print("\n[1/2] 声明 ↔ 引用 ↔ 节点声明")
    check("参数面三边对齐", problems(launch_src, node_src))
    for exe in NODES:
        pairs, _ = node_param_pairs(launch_src, exe)
        print(f"      {exe}: {len(pairs)} 个参数全部对上")

    print("\n[2/2] 变异测试（每条判据都要能红）")
    exe = next(iter(NODES))
    for name, mut_launch, mut_node, expect in mutations(
            launch_src, node_src[exe]):
        # ⚠️ 变异没打到文本 = 这条「判据能红」的证明是空的。第一次跑就撞上：
        # launch 的默认值改成 '0.30' 后，变异里还在 replace '0.3'，静默不生效，
        # 于是「默认值不一致」那条判据看起来白绿了一次。宁可这里炸。
        assert (mut_launch, mut_node) != (launch_src, node_src[exe]), \
            f"变异「{name}」一字未改"
        got = problems(mut_launch, {exe: mut_node})
        hit = any(expect in g for g in got)
        check(f"变异「{name}」", [] if hit else
              [f"没被发现（期望包含「{expect}」，实际 {got or '全绿'}）"])

    print("\n" + "=" * 60)
    if _failed:
        print(f"❌ {len(_failed)} 项没过：")
        for n in _failed:
            print(f"   - {n}")
        return 1
    print("✅ 全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
