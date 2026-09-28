#!/usr/bin/env python3
"""
真机 launch 的结构与守卫自检 —— 不需要真机、不需要仿真器、不需要 colcon build（W2）
=================================================================================
`real.launch.py` 是**第一次真机联调要跑的东西**，而它没法在真机上反复试错：真机上的
每次试错都要占用机器人、都要授权、都可能动臂。所以它的**结构性约定**必须在本机
钉死，让「参数写错 / 守卫失效 / 帧名拍脑袋」在**不接机器人的时候**就暴露。

本脚本测的是四类东西，每一类都对应一个真实会犯的错：

1. **守卫真的会拦**（`_check`）。`robot_ip` 没给就启动 = 驱动去连一个空 IP 超时，
   日志看起来像「驱动坏了」；四元数全零就发 TF = 发出的不是旋转（模长 0），
   而链路上**没有任何一步会报错**，只是抓取点偏掉。这两条都必须**抛异常**，
   不是打日志。
2. **守卫不在不该拦的时候拦**。`start_ur_driver:=false`（只起 MoveIt / 只起感知）
   时不需要 `robot_ip`；`publish_camera_tf:=false` 时不需要外参。误拦会让
   分阶段验收（W5 第 4 步）走不下去。
3. **帧名与「两份 description 一致」这类不能靠猜的约定**：相机静态 TF 的子帧必须是
   `camera_link` —— 发到 `camera_color_optical_frame` 会给它第二个父节点，TF 树直接
   坏掉；`safety_limits` 必须**同时**传给驱动与 `move_group`（两侧各有一份
   robot_description，不一致时 FK/IK 与真实 TF 是两套几何，且两边都不报错）。
4. **参数真的到得了节点**（`max_sync_dt`，D50 家族）。参数在本仓库有两种
   「定义了但没用」的坏法：定义在一个**没人 include 的 launch 文件**上
   （`perception_3d/launch/perception.launch.py` 就是），或者定义了两处、
   其中一处静默失效。所以既断言 pipeline.launch.py **声明并传下去**，
   也断言 real/bringup **没有**第二处定义。
5. **D64 回归**：唯一入口 `bringup.launch.py` **不得声明**真机专属参数。
   `launch_configurations` 是整棵树共享的一个 dict，而下游的
   `DeclareLaunchArgument` 见到「已设置」就不再应用自己的默认值 —— 在入口声明
   真机参数的默认值，会把**仿真路径**的默认值一起改掉，且不报错。
   （这正是本轮真实发生过的一次回退，被 skip_detector 冒烟抓到。）

用法:
    source /opt/ros/humble/setup.bash      # 只需要 ROS 基础包，不需要本工作区
    python3 scripts/test_real_launch.py
退出码 0 = 全通过。
"""

import ast
import importlib.util
import os
import sys
import types
from pathlib import Path

try:  # 只为 W2-16 里「把别人包的真路径解回来」用；缺了也不该让整份自检跑不起来
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory as _gpsd,
    )
except ImportError:  # pragma: no cover —— 没 source ROS 时下面会先报错退出
    PackageNotFoundError = Exception  # type: ignore[assignment,misc]
    _gpsd = None  # type: ignore[assignment]

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LAUNCH_DIR = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "launch"

_failed = []
_skipped = []

# 真机专属参数：**只能在 real.launch.py 里声明**（D64）。这份清单要和
# bringup.launch.py 文件头那段说明一起看。
REAL_ONLY_ARGS = [
    "robot_ip", "ur_type", "safety_limits",
    "start_ur_driver", "start_moveit", "start_pipeline",
    "publish_camera_tf",
    "camera_x", "camera_y", "camera_z",
    "camera_qx", "camera_qy", "camera_qz", "camera_qw",
]


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not cond:
        _failed.append(name)


def skip(name, why):
    print(f"  [SKIP] {name}  {why}")
    _skipped.append(name)


def _raises(fn, exc=Exception):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


def load_launch(name):
    """按路径加载 launch 文件，并把包路径解析换成假的。

    只需要 `/opt/ros/humble` 被 source 过（launch / launch_ros / ament_index_python
    都是 ROS 基础包）。**不需要** `colcon build` —— 包路径解析被我们替换掉了，
    所以它检查的是**源码**，而不是 install 里那份拷贝。
    """
    path = LAUNCH_DIR / name
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location(f"probe_{name[:-3]}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda pkg: f"/nonexistent/{pkg}"
    return mod


def load_launch_at(path):
    """同 `load_launch`，但按完整路径加载（用于**别包**的 launch 文件）。"""
    if not Path(path).exists():
        return None
    spec = importlib.util.spec_from_file_location(f"probe_{Path(path).stem}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda pkg: f"/nonexistent/{pkg}"
    return mod


def _node_defaults(node_path):
    """从节点源码里解析 `declare_parameter('name', <default>)` 的默认值。

    这一项没有运行时接口可查（`ros2 param get` 在本机不可用，见 README 与
    第十三轮记录），但**源码就是权威** —— 而且这里要比的正是「launch 的默认值
    与节点默认值是否一致」，两边都从源码读最直接。
    """
    import re

    src = Path(node_path).read_text(encoding="utf-8")
    out = {}
    for m in re.finditer(r"declare_parameter\(\s*'([^']+)'\s*,\s*([^)]+?)\s*\)", src):
        name, raw = m.group(1), m.group(2).strip()
        if raw.startswith("'") and raw.endswith("'"):
            raw = raw[1:-1]
        out[name] = raw
    return out


def ctx(**overrides):
    """假的 launch context：这些函数只读 launch_configurations。"""
    base = {"start_ur_driver": "true", "publish_camera_tf": "false"}
    base.update(overrides)
    return types.SimpleNamespace(launch_configurations=base)


def _resolve_default(subs):
    """把 `default_value` 求值成字符串；求不出来就回退成 repr。

    ⚠️ `default_value` **不是字符串**，是被 `normalize_to_list_of_substitutions`
    包过的**替换对象元组**（实测：
    `[<launch.substitutions.text_substitution.TextSubstitution object …>]`）。
    写 `== ""` 会永远不等 —— 本脚本第一版就是这么写错的，而它的症状是
    「参数明明写着空串，测试却报 FAIL」，很容易被误读成 launch 文件写错了。

    `tf_required` 的默认值是 `PythonExpression`（引用 `camera_source`），在空
    context 里求不出来，所以这里吞掉异常 —— 需要断言它的地方会自己造 context。
    """
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    try:
        return perform_substitutions(LaunchContext(), subs)
    except Exception:
        return repr(subs)


def declared_names(mod):
    """从 `generate_launch_description()` 的返回值里把参数名与默认值取出来。

    顺带覆盖 `Node`：它有 `.name`（节点名）但没有 `default_value`，
    所以「无默认值」要和「默认值是空串」区分开（`robot_ip` 就是空串默认值，
    它是**必填项**的标志）。
    """
    desc = mod.generate_launch_description()
    out = []
    for entity in desc.entities:
        name = getattr(entity, "name", None)
        if name is None:
            continue
        subs = getattr(entity, "default_value", None)
        out.append((name, None if subs is None else _resolve_default(subs)))
    return out


# ---------------------------------------------------------------------------
# 只走位模式的可达性：解析 planner_node.py，而不是 grep 行号（W2-8b）
# ---------------------------------------------------------------------------

PLANNER_NODE_PY = (PROJECT_ROOT / "ros2_ws" / "src" / "grasp_planner"
                   / "grasp_planner" / "planner_node.py")

GRIPPER_METHODS = ("open", "close", "current_position", "is_empty")


def _is_carry_guard(test):
    """认出 ``if not self.carry_objects:`` 这个测试表达式（不看注释、不看缩进）。"""
    return (isinstance(test, ast.UnaryOp) and isinstance(test.op, ast.Not)
            and isinstance(test.operand, ast.Attribute)
            and test.operand.attr == "carry_objects"
            and isinstance(test.operand.value, ast.Name)
            and test.operand.value.id == "self")


def _own_nodes(stmts):
    """遍历一串语句，但**不进入**内层 ``def`` 的函数体。

    不这样做，``_open_and_reset`` 里那个嵌套的 ``_then_open`` 会被数两遍
    （一次算在外层头上、一次算在自己头上）—— 本函数的第一版就是这么错的，
    被自检当场抓出来（``open()`` 的调用点从 3 个变成 4 个）。
    """
    stack = list(stmts)
    while stack:
        node = stack.pop()
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue                      # 它自己由调用方收录，函数体归它自己
        yield node
        stack.extend(ast.iter_child_nodes(node))


def _collect_functions(tree):
    """把每个函数（含嵌套 ``def``）的「自有节点 + 守卫行号」收出来。

    键是限定名：``_on_descend_done`` 或 ``_open_and_reset._then_open``。
    """
    out = {}

    def visit(func, prefix):
        qname = prefix + func.name
        own, nested = [], []
        for stmt in func.body:
            if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef)):
                nested.append((stmt, qname + "."))
                own.append(stmt)          # ``def`` 这条语句本身留着（用于认名字）
            else:
                own.extend(_own_nodes([stmt]))
        # 守卫必须是**函数体顶层**的 ``if not self.carry_objects:`` 且**真的返回**
        guards = [s.lineno for s in func.body
                  if isinstance(s, ast.If) and _is_carry_guard(s.test)
                  and any(isinstance(b, ast.Return) for b in s.body)]
        out[qname] = (own, guards)
        for stmt, pfx in nested:
            visit(stmt, pfx)

    def scan(body):
        # ⚠️ 必须**进到 class 里面**去：这些方法都挂在 ``class PlannerNode`` 上，
        # 只看模块顶层的 ``def`` 会一个方法都找不到（第一版就是这么错的，
        # 于是「解析到 0 处引用」把每一条断言都变成了 FAIL —— 而这恰好是
        # 正向对照存在的意义：空解析**不会**伪装成通过）。
        for stmt in body:
            if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef)):
                visit(stmt, "")
            elif isinstance(stmt, ast.ClassDef):
                scan(stmt.body)

    scan(tree.body)
    return out


def _ref_line(node):
    """一个 ``self.X`` / ``self._gripper.X`` 形式的引用；不是就返回 None。

    ``self._gripper.is_empty`` 是「属性套属性」，所以这里要分两层看。
    """
    if not isinstance(node, ast.Attribute):
        return None
    base = node.value
    if isinstance(base, ast.Name) and base.id == "self":
        return ("self", node.attr)
    if (isinstance(base, ast.Attribute) and base.attr == "_gripper"
            and isinstance(base.value, ast.Name) and base.value.id == "self"):
        return ("gripper", node.attr)
    return None


def relay_mode_reachability():
    """只走位模式（``carry_objects:=false``）下，哪些函数 / 夹爪方法**到不了**。

    返回 ``(到不了的集合, 夹爪方法的引用明细)``。判定规则 ——
    一个函数**到不了**，当且仅当它有入边、且**每一条**入边都满足
    「被 ``not self.carry_objects`` 的早返回挡在前面」**或**「来自另一个到不了的
    函数」。**没有入边的函数一律当作到得了** —— 拿不出证据就假设它是活的，
    反过来会把危险的方法判成安全，那是这个自检最坏的一种错。

    ⚠️ 为什么必须做这层**传递**：``is_empty()`` / ``current_position()`` 只被
    ``_evaluate_grasp`` 调用，而 ``_evaluate_grasp`` 只被 ``_on_close_done`` 里的
    ``Timer`` 挂上、``_on_close_done`` 又只作为 ``done_cb`` 挂在**守卫之后**那次
    ``close()`` 上。只看**直接**外层函数会得出「没有守卫 ⇒ 到得了」的错误结论。
    """
    tree = ast.parse(PLANNER_NODE_PY.read_text(encoding="utf-8"))
    funcs = _collect_functions(tree)
    plain = {}
    for qname in funcs:
        plain.setdefault(qname.split(".")[-1], []).append(qname)
    local = {p: qs[0] for p, qs in plain.items() if len(qs) == 1}

    # 入边：target -> [(来源函数, 行号, 在来源函数里是否已被守卫挡住)]
    in_edges = {}

    def add_edge(target, src, lineno, guarded):
        in_edges.setdefault(target, []).append((src, lineno, guarded))

    gripper_refs = []
    for qname, (nodes, guards) in funcs.items():
        for node in nodes:
            hit = _ref_line(node)
            if hit is None:
                continue
            kind, name = hit
            guarded = any(g < node.lineno for g in guards)
            if kind == "gripper":
                if name in GRIPPER_METHODS:
                    target = f"_gripper.{name}"
                    add_edge(target, qname, node.lineno, guarded)
                    gripper_refs.append((name, qname, node.lineno, guarded))
            elif name in funcs:                     # self.<某方法>
                add_edge(name, qname, node.lineno, guarded)
            elif name in local:                     # 嵌套局部函数（self._then_open 少见）
                add_edge(local[name], qname, node.lineno, guarded)
            # 局部函数按**裸名字**被引用（``done_cb=_then_open``）—— 只有它确实是
            # 本文件里定义过的函数名才算，免得把 ``result`` 这类普通变量认成调用。
        for node in nodes:
            name = getattr(node, "id", None)
            if name in local and local[name] != qname:
                guarded = any(g < node.lineno for g in guards)
                add_edge(local[name], qname, node.lineno, guarded)

    # 不动点：反复摘掉「所有入边都被挡住」的节点，剩下的就是到得了的。
    alive = set(funcs) | {f"_gripper.{m}" for m in GRIPPER_METHODS}

    def blocked(src, guarded):
        return guarded or src not in alive

    changed = True
    while changed:
        changed = False
        for node in sorted(alive):
            edges = in_edges.get(node)
            if edges and all(blocked(src, g) for src, _, g in edges):
                alive.discard(node)
                changed = True

    return {n for n in (set(funcs) | {f"_gripper.{m}" for m in GRIPPER_METHODS})
            if n not in alive}, gripper_refs


def _src(name):
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


# 真机 / 仿真两条链路会起的可执行文件（含一个手工使用的）。D70 说的是它们
# **收工时的样子** —— 只有在板上真的跑一遍才看得出来，所以拿自检钉住。
NODE_MAINS = [
    "ros2_ws/src/yolo_detector/yolo_detector/detector_node.py",
    "ros2_ws/src/perception_3d/perception_3d/projector_node.py",
    "ros2_ws/src/grasp_planner/grasp_planner/planner_node.py",
    "ros2_ws/src/yolo_bringup/yolo_bringup/test_image_publisher.py",
]


def _is_attr_call(node, obj, attr):
    return (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == attr
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == obj)


def _is_spin_call(node):
    """``rclpy.spin(...)`` 或 ``executor.spin()`` —— 两者抛的是同一族异常。

    ⚠️ 只认 ``rclpy.spin`` 会漏掉 planner_node（它用
    ``MultiThreadedExecutor().spin()``），而漏掉的后果是那条断言变成 SKIP、
    看上去「不适用」，实际它**同样**会被 Ctrl-C 那段竞态打中。
    """
    return (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "spin")


def shutdown_hygiene(path):
    """检查一个节点文件的 ``main()`` 能不能**干净收工**（D70）。

    返回 ``(找到几处 rclpy.shutdown(), 没被 rclpy.ok() 挡住的那些的行号,
    spin 所在的 try 有没有接 ExternalShutdownException,
    收工竞态有没有被 rclpy.ok() 闸门吞掉, main() 之外有几处 publish,
    main() 里有几处 destroy_node, 其中没挡住 KeyboardInterrupt 的行号)``。

    四条要求，都是 2026-09-28 板端实测出来的：
      1. ``rclpy.shutdown()`` 必须包在 ``if rclpy.ok():`` 里。``rclpy.init()``
         装的信号处理器在 SIGINT 时已经关过一次 context，**在 finally 里**再
         无条件关一次会抛 ``RCLError``，而 finally 里抛出的异常**顶掉**原本那个
         ⇒ launch 报 ``process has died ... exit code 1``。
      2. ``rclpy.spin()`` 的那个 try 必须接 ``ExternalShutdownException``。
         Ctrl-C 下 rclpy 抛的是它、**不是** ``KeyboardInterrupt``（只接后者的
         话它一路逃出 main()，同样是一段 traceback）。
      3. 还要有一条**只在 context 还活着时才 re-raise** 的兜底：SIGINT 落在
         回调执行到 ``publish`` 那一行时，rclpy 把回调里那个异常从 ``spin_once``
         重新抛出，而它既不是第 1 种也不是第 2 种（见 `main()` 里那段注释）。
      4. ``destroy_node()`` 必须挡住 ``KeyboardInterrupt``。``ros2 launch`` 收到
         SIGINT 后会把 SIGINT **转发**给每个子进程，于是子进程一共收到两次；实测
         第二次落在 ``destroy_node()`` 里面（rclpy 的
         ``client.destroy_when_not_in_use``），抛 ``KeyboardInterrupt`` 从
         finally 逃出去 ⇒ traceback ＋ ``process has died ... exit code -2``。
    """
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    mains = [n for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef) and n.name == "main"]
    calls, unguarded = [], []
    spin_ok = race_ok = None
    teardown, teardown_unguarded = [], []
    for fn in mains:
        for node in ast.walk(fn):
            if _is_attr_call(node, "rclpy", "shutdown"):
                calls.append(node.lineno)
                if not _inside_rclpy_ok(fn, node):
                    unguarded.append(node.lineno)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                    and node.func.attr == "destroy_node":
                teardown.append(node.lineno)
                if not _protected_from(fn, node):
                    teardown_unguarded.append(node.lineno)
        for t in [n for n in ast.walk(fn) if isinstance(n, ast.Try)]:
            if not any(_is_spin_call(c) for c in ast.walk(t)):
                continue
            caught = set()
            race_ok = False
            for h in t.handlers:
                if h.type is None:                       # 裸 except：什么都接得住
                    caught.add("ExternalShutdownException")
                for sub in ast.walk(h.type) if h.type else []:
                    if isinstance(sub, ast.Name):
                        caught.add(sub.id)
                    elif isinstance(sub, ast.Attribute):
                        caught.add(sub.attr)
            # 第 3 条：某个处理器里有「`if rclpy.ok(): raise`」这道闸
            for h in t.handlers:
                for node in ast.walk(h):
                    if (isinstance(node, ast.If)
                            and _is_attr_call(node.test, "rclpy", "ok")
                            and any(isinstance(d, ast.Raise)
                                    for b in node.body for d in ast.walk(b))):
                        race_ok = True
            spin_ok = "ExternalShutdownException" in caught
    # 回调里（= main() 之外）到底有没有 publish —— 第 3 条只对**有发布**的文件
    # 成立。planner_node 一处 publish 都没有（它只发 action / service 请求），
    # 拿这条去要求它就是个假阳性。
    pub_lines = [n.lineno for n in ast.walk(tree)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute)
                 and n.func.attr == "publish"
                 and not any(n is d for fn in mains for d in ast.walk(fn))]
    return calls, unguarded, spin_ok, race_ok, pub_lines, teardown, teardown_unguarded


def _protected_from(fn, target, names=("KeyboardInterrupt", "BaseException")):
    """``target`` 是否落在 ``fn`` 里某个 ``try`` 的**体**内，且那个 try 的处理器
    接得住 ``names`` 里的异常（裸 ``except`` 也算）。"""
    for node in ast.walk(fn):
        if not isinstance(node, ast.Try):
            continue
        if not any(target is d for b in node.body for d in ast.walk(b)):
            continue
        for h in node.handlers:
            if h.type is None:
                return True
            for sub in ast.walk(h.type):
                name = sub.id if isinstance(sub, ast.Name) else getattr(sub, "attr", None)
                if name in names:
                    return True
    return False


def _inside_rclpy_ok(fn, target):
    """``target`` 是否落在 ``fn`` 里某个 ``if rclpy.ok():`` 的分支内。"""
    for node in ast.walk(fn):
        if isinstance(node, ast.If) and _is_attr_call(node.test, "rclpy", "ok"):
            if any(target is d for b in node.body for d in ast.walk(b)):
                return True
    return False


def _lc_name(v):
    """取参数值指向的 launch 参数名。

    ⚠️ `LaunchConfiguration.variable_name` 在 Humble 里**不是字符串**，是替换
    对象列表（实测 `[<TextSubstitution ...>]`）—— 与 `default_value` 那个坑
    同一类。所以还要再求值一次，否则比出来永远是个 repr 串。
    """
    if isinstance(v, str):
        return v
    name = getattr(v, "variable_name", None)
    if name is None:
        return None
    return name if isinstance(name, str) else _resolve_default_in(name)


def _raw_default(mod, name):
    """取某个 DeclareLaunchArgument 的 `default_value` **替换对象本身**。

    `declared_names()` 已经把默认值求值成字符串了，但 `PythonExpression`
    在空 context 里求不出来，只能拿到 repr —— 所以「默认值随 camera_source
    变」这种要另拿原对象、在填好 camera_source 的 context 里再求一次。
    """
    for e in _walk(mod.generate_launch_description().entities):
        if getattr(e, "name", None) == name:
            return getattr(e, "default_value", None)
    return None


def _resolve_default_in(subs, **confs):
    """在**给定** launch_configurations 的环境里求值默认值。

    `_resolve_default` 用的是空 context，所以像 `PythonExpression`（引用
    `camera_source`）这类默认值只能拿到 repr。这里把待用的键填进去，
    让「默认值随 camera_source 变」这件事**真的被求值出来**再断言。
    """
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    c = LaunchContext()
    c.launch_configurations.update(confs)
    try:
        return perform_substitutions(c, subs)
    except Exception:
        return repr(subs)


def _walk(entities):
    """展平 LaunchDescription 的实体树（Node 在 pipeline 里被 TimerAction 包着）。"""
    for e in entities:
        yield e
        inner = getattr(e, "actions", None)
        if inner:
            yield from _walk(inner)


def node_params(mod, node_name):
    """取某个 Node 的 parameters 列表；找不到返回 None。

    ⚠️ 两个坑，都会让断言变成「永远 PASS」或直接崩：
      · `Node` 在 pipeline.launch.py 里**不是顶层实体** —— 四个节点都裹在一个
        `TimerAction(period=LaunchConfiguration('start_delay'), actions=[...])` 里
        （统一延时启动）。只遍历 `desc.entities` 一个节点都找不到 ⇒ 必须展平。
      · `node.node_name` 是**公开属性但会抛**（`cannot access 'node_name'
        before executing action`）—— 我们在 `generate_launch_description()`
        之后、执行之前读它，正好落在这个禁区里。所以读它存进去的私有字段
        （`Node.__init__` 里的 `self.__node_name`）。这是测试代码读 launch 的
        内部结构，用私有名是**有意的**：公开接口在这个时机不可用。
    """
    for e in _walk(mod.generate_launch_description().entities):
        raw = getattr(e, "_Node__node_name", None)
        if isinstance(raw, str) and raw == node_name:
            # 名字同样是私有字段（`self.__parameters`）：公开的 `.parameters`
            # 在 launch_ros 里**不存在**（Humble 实测 AttributeError）。
            return getattr(e, "_Node__parameters", None)
    return None


def main():
    try:
        import launch  # noqa: F401
    except ImportError:
        print("需要 ROS 环境：先 `source /opt/ros/humble/setup.bash`")
        return 2

    real = load_launch("real.launch.py")
    if real is None:
        print("找不到 real.launch.py —— 本脚本测的就是它")
        return 2
    bringup = load_launch("bringup.launch.py")

    # ---------------------------------------------------------------- 守卫
    print("\n--- W2-1 启动守卫：该拦的必须拦 ---")
    check("start_ur_driver:=true 且 robot_ip 为空 → 抛错",
          _raises(lambda: real._check(ctx(robot_ip="")), RuntimeError))
    check("robot_ip 只有空白字符也算空 → 抛错",
          _raises(lambda: real._check(ctx(robot_ip="   ")), RuntimeError))
    err = None
    try:
        real._check(ctx(robot_ip=""))
    except RuntimeError as e:
        err = str(e)
    check("robot_ip 的报错给出两个入口的用法（现场可操作性）",
          err is not None and "real.launch.py" in err and "bringup.launch.py" in err)

    print("\n--- W2-2 守卫：不该拦的不能拦 ---")
    check("start_ur_driver:=false 时不需要 robot_ip",
          real._check(ctx(start_ur_driver="false")) == [])
    check("publish_camera_tf:=false 时不需要外参（默认即此）",
          real._check(ctx(robot_ip="1.2.3.4")) == [])
    check("给了合格外参 → 通过",
          real._check(ctx(robot_ip="1.2.3.4", publish_camera_tf="true",
                          camera_qw="1.0")) == [])

    print("\n--- W2-3 守卫：全零四元数必须拦（模长为 0，不是旋转）---")
    check("publish_camera_tf:=true 且四元数全零 → 抛错",
          _raises(lambda: real._check(ctx(robot_ip="1.2.3.4",
                                          publish_camera_tf="true")), RuntimeError))
    check("只给 qw=0.0 的显式写法也算全零 → 抛错",
          _raises(lambda: real._check(ctx(robot_ip="1.2.3.4", publish_camera_tf="true",
                                          camera_qw="0.0")), RuntimeError))
    check("_is_zero 对非字面量保守返回 False（不误报）",
          real._is_zero("${some_substitution}") is False)

    # ------------------------------------------------- 不能靠猜的帧名 / 一致性
    print("\n--- W2-4 相机静态 TF：子帧必须是 camera_link ---")
    src = _src("real.launch.py")
    # 只看 `camera_tf = Node(...)` 那一段，免得被注释里提到的 optical frame 误判
    tf_block = ""
    if "camera_tf = Node(" in src:
        tf_block = src.split("camera_tf = Node(", 1)[1].split("\n    )", 1)[0]
    check("child-frame-id 是 camera_link",
          '"--child-frame-id", "camera_link"' in tf_block)
    check("**没有**把 optical frame 当子帧（会让它有两个父节点）",
          "camera_color_optical_frame" not in tf_block)
    check("父帧是 base_link", '"--frame-id", "base_link"' in tf_block)

    print("\n--- W2-5 两份 robot_description 必须用同一个 safety_limits ---")
    check("驱动那一处传了 safety_limits",
          '"safety_limits": LaunchConfiguration("safety_limits")' in src)
    check("move_group 那一处也传了（同一个 LaunchConfiguration）",
          src.count('"safety_limits": LaunchConfiguration("safety_limits")') == 2)

    print("\n--- W2-6 上游两个默认 true 的开关必须显式关掉（D59）---")
    check("launch_rviz 关掉（两处：驱动与 move_group）",
          src.count('"launch_rviz": "false"') == 2)
    check("launch_servo 关掉（moveit_servo 能直接指挥机器人）",
          '"launch_servo": "false"' in src)
    check("初始控制器显式写 scaled_（尊重示教器速度滑条）",
          real.INITIAL_JOINT_CONTROLLER == "scaled_joint_trajectory_controller"
          and '"initial_joint_controller": INITIAL_JOINT_CONTROLLER' in src)

    # ------------------------------------------------------------------ D64
    print("\n--- W2-7 D64 回归：唯一入口不得声明真机专属参数 ---")
    if bringup is None:
        skip("bringup.launch.py 的 D64 回归", "文件不存在")
    else:
        names = [n for n, _ in declared_names(bringup)]
        leaked = [n for n in REAL_ONLY_ARGS if n in names]
        check("bringup.launch.py 没有声明任何真机专属参数",
              not leaked, f"泄漏了: {leaked}" if leaked else f"已声明: {names}")
        check("但它**必须**声明 camera_source（路由的依据）",
              "camera_source" in names)

    print("\n--- W2-8 D64 的另一半：真机细分项在 real.launch.py 里齐不齐 ---")
    real_names = [n for n, _ in declared_names(real)]
    missing = [n for n in REAL_ONLY_ARGS if n not in real_names]
    check("真机专属参数都在 real.launch.py 里声明了",
          not missing, f"缺: {missing}" if missing else "")
    check("robot_ip 的默认值是空串（**必填项**的标志）",
          dict(declared_names(real)).get("robot_ip") == "")
    check("第一个 demo 的默认组合 = 不夹取（gripper_backend=none + carry_objects=false）",
          dict(declared_names(real)).get("gripper_backend") == "none"
          and dict(declared_names(real)).get("carry_objects") == "false")

    # ------------------------------------ 上面那个组合**为什么**安全（W2-8b）
    print("\n--- W2-8b 只走位模式的可达性：NullGripper 那张表按结构核一遍 ---")
    # `NullGripper` 只实现 `open()`，其余三个方法**抛 RuntimeError**。这个设计的
    # 依据是「只走位模式下那三个调用点不可达」—— 一条**关于 planner_node.py 结构**
    # 的断言，而它原先只写在 gripper.py 的文档字符串里，没人核。文档里的行号已经
    # 漂过一次（八个数字齐齐偏 20 行，且不报错），所以这里解析语法树、按**函数名**
    # 核结构：调用点挪到守卫之前，判定会**当场**翻转。
    #
    # 判据的强弱要说清：「到不了」这一侧是**可靠**的 —— 函数体顶层、调用点之前的
    # 早返回确定地挡住了它（传递闭包只在**每一条**入边都被挡住时才判到不了）；
    # 「到得了」那一侧只是**必要条件**（没守卫 ≠ 运行时会走到），所以后两条配了
    # 精确计数，靠「数量对不上就 FAIL」来兜。
    if not PLANNER_NODE_PY.exists():
        skip("NullGripper 的可达性表", f"{PLANNER_NODE_PY} 不存在")
    else:
        unreachable, refs = relay_mode_reachability()
        check("解析到了 planner_node 里的夹爪引用（否则下面全是空断言）",
              len(refs) >= 4,
              f"实际 {len(refs)} 处: {[(n, ln) for n, _, ln, _ in refs]}")
        for meth in ("close", "is_empty", "current_position"):
            mine = [r for r in refs if r[0] == meth]
            check(f"``{meth}()`` 在只走位模式下到不了"
                  f"（NullGripper 才敢对它抛错而不是静默）",
                  bool(mine) and f"_gripper.{meth}" in unreachable,
                  f"引用点 {[(r[1], r[2]) for r in mine]}"
                  + ("" if f"_gripper.{meth}" in unreachable
                     else "，**但可达性分析说它到得了**"))
        opens = [r for r in refs if r[0] == "open"]
        live = [r for r in opens if not r[3]]
        check("``open()`` 恰有两处**可达**（初始化 + 失败恢复）"
              " —— 这就是 NullGripper 只实现 open() 的根据",
              len(opens) == 3 and len(live) == 2,
              f"共 {len(opens)} 处，其中可达 {len(live)} 处 "
              f"{[(r[1], r[2]) for r in live]}")
        check("…剩下那一处 open()（放置松开）是被守卫挡住的",
              len(opens) - len(live) == 1 and "_gripper.open" not in unreachable,
              f"被挡 {[(r[1], r[2]) for r in opens if r[3]]}")

    print("\n--- W2-9 参数名唯一（重复声明会让后一个静默失效）---")
    for mod, label in ((real, "real"), (bringup, "bringup")):
        if mod is None:
            continue
        names = [n for n, _ in declared_names(mod)]
        dupes = sorted({n for n in names if names.count(n) > 1})
        check(f"{label}.launch.py 无重复声明", not dupes, f"重复: {dupes}" if dupes else "")

    # ------------------------------------------- 参数真的到得了投影节点（D50 家族）
    print("\n--- W2-10 max_sync_dt 在唯一入口上真的有定义、真的传下去了 ---")
    pipeline = load_launch("pipeline.launch.py")
    if pipeline is None:
        skip("max_sync_dt 的可达性", "pipeline.launch.py 不存在")
    else:
        p_names = [n for n, _ in declared_names(pipeline)]
        p_defaults = dict(declared_names(pipeline))
        check("pipeline.launch.py 声明了 max_sync_dt",
              "max_sync_dt" in p_names)
        subs = _raw_default(pipeline, "max_sync_dt")
        check("仿真默认 0.1（= 拿到 9/9 那套结果时的值，不为了让日志干净去改它）",
              _resolve_default_in(subs, camera_source="sim") == "0.1",
              f"got {_resolve_default_in(subs, camera_source='sim')!r}")
        check("真机默认放宽到 0.3（窗口太紧的症状是**看不见目标**）",
              _resolve_default_in(subs, camera_source="realsense") == "0.3",
              f"got {_resolve_default_in(subs, camera_source='realsense')!r}")
        check("external 也走放宽值（话题来自 rosbag/自建 launch，同样会抖）",
              _resolve_default_in(subs, camera_source="external") == "0.3")

        params = node_params(pipeline, "perception_3d")
        check("找得到 perception_3d 这个 Node（否则下面的断言全是空的）",
              params is not None, "" if params is not None else "Node 不在实体树里")
        if params:
            # ⚠️ `Node.__init__` 会 `normalize_parameters`：**键与值都变成替换对象
            # 元组**（`(<TextSubstitution>,)` / `(<LaunchConfiguration>,)`），
            # 既不是 str 也不是可下标比较的对象 —— 直接 `d.get("max_sync_dt")`
            # 恒 None，`sorted(d)` 还会抛 TypeError。所以两边都各求值一次。
            d = params[0] if isinstance(params[0], dict) else {}
            pairs = {_resolve_default_in(k): _lc_name(v[0]) for k, v in d.items()}
            check("投影节点的参数里 max_sync_dt 指向同名 launch 参数",
                  pairs.get("max_sync_dt") == "max_sync_dt",
                  f"got {pairs.get('max_sync_dt')!r}; keys={sorted(pairs)}")
            check("原有三项没被顶掉（target_frame / depth_topic / tf_required）",
                  {"target_frame", "depth_topic", "tf_required"} <= set(pairs),
                  f"keys={sorted(pairs)}")
        else:
            _failed.append("perception_3d 的参数不可读")

        # 节点自身还留着一个兜底默认值（launch 什么都不传时用它）——也钉住，
        # 免得它和 launch 侧的默认值悄悄分叉。
        node_src = (PROJECT_ROOT / "ros2_ws" / "src" / "perception_3d"
                    / "perception_3d" / "projector_node.py").read_text(encoding="utf-8")
        check("节点侧兜底默认值仍是 0.1（与仿真路径一致）",
              "declare_parameter('max_sync_dt', 0.1)" in node_src)
        check("丢弃检测时给出**累计**批数（限流日志里要能分辨「偶尔」与「一直」）",
              "_sync_drops += 1" in node_src and "累计 {self._sync_drops} 批" in node_src)

        # ------------------------------------------------ 参数面完整性（整条规则）
        print("\n--- W2-12 唯一入口的参数面完整性：传给节点的每一个参数都有声明 ---")
        # 这条规则是 D50 家族的一般化：**未声明的参数会被 launch 接受**并塞进
        # 共享的 launch_configurations（实测），然后**没有任何人读它** ——
        # 于是「我明明传了」和「一点反应都没有」同时成立。所以对**每个**节点
        # 都断言「它收到的参数 ⊂ 本文件声明的参数」。
        for node in ("yolo_detector", "perception_3d", "grasp_planner"):
            prm = node_params(pipeline, node)
            keys = set()
            if prm:
                for item in prm:
                    if isinstance(item, dict):
                        keys |= {_resolve_default_in(k) for k in item}
            undeclared = sorted(k for k in keys if k not in p_names)
            check(f"{node} 收到的参数都在入口声明了（{len(keys)} 个）",
                  bool(keys) and not undeclared,
                  f"未声明: {undeclared}" if undeclared else
                  ("" if keys else "一个参数都没读到 —— 断言是空的"))

        # 反向：声明了却从没被引用 = 死旋钮（传了永远没反应，同样静默）
        p_src = _src("pipeline.launch.py")
        unused = [n for n in p_names
                  if f"LaunchConfiguration('{n}')" not in p_src
                  and f'LaunchConfiguration("{n}")' not in p_src]
        check("没有「声明了但文件里从不引用」的死旋钮", not unused,
              f"死旋钮: {unused}" if unused else f"{len(p_names)} 个声明都有引用")

        print("\n--- W2-13 三个「单独启动」文件里不得藏着主入口没有的旋钮 ---")
        # 它们是**不在主链路上**的文件（全仓库没有一处 include）。若某个参数
        # 只在那边定义，现场照文档去那边调就是白忙 —— 主链路读的是 pipeline。
        # `params_file` 是别名：同一个东西，主入口叫 `planner_params_file`。
        ALIASES = {"params_file": "planner_params_file"}
        for rel in ("perception_3d/launch/perception.launch.py",
                    "yolo_detector/launch/detector.launch.py",
                    "grasp_planner/launch/planner.launch.py"):
            path = PROJECT_ROOT / "ros2_ws" / "src" / rel
            mod = load_launch_at(path)
            if mod is None:
                skip(f"{path.name} 的参数面", "文件不存在")
                continue
            own = [n for n, _ in declared_names(mod)]
            only_there = [n for n in own
                          if n not in p_names and ALIASES.get(n) not in p_names]
            check(f"{path.name} 的参数都在主入口有对应（{len(own)} 个）",
                  not only_there,
                  f"只在它这里有: {only_there}" if only_there else "")

        print("\n--- W2-14 投影节点的默认值 = 节点自己的 declare_parameter（除两处刻意的）---")
        # launch 侧写了一个与节点默认值不同的数，就是**静默改行为**（当年
        # grasp_planner 的 yaml 与节点默认值「碰巧逐条相同」才没暴露）。这里
        # 用节点源码里的 declare_parameter 值当基准，逐条比。
        # 两处**刻意**分叉：depth_topic（真机用对齐深度，D2）、max_sync_dt（真机放宽）。
        INTENTIONAL = {"depth_topic", "max_sync_dt"}
        node_defaults = _node_defaults(PROJECT_ROOT / "ros2_ws" / "src"
                                       / "perception_3d" / "perception_3d"
                                       / "projector_node.py")
        check("从节点源码里解析出参数默认值（解析失败下面全是空的）",
              len(node_defaults) >= 8, f"解析到 {len(node_defaults)} 条")
        mismatched = []
        for name, node_val in node_defaults.items():
            if name in INTENTIONAL or name not in p_names:
                continue
            launch_val = _resolve_default_in(_raw_default(pipeline, name),
                                             camera_source="sim")
            if str(launch_val).lower() != str(node_val).lower():
                mismatched.append(f"{name}: launch={launch_val!r} node={node_val!r}")
        check("除 depth_topic / max_sync_dt 外，两边默认值逐条相同", not mismatched,
              "; ".join(mismatched) if mismatched else
              f"比了 {len(node_defaults) - len(INTENTIONAL & set(node_defaults))} 条")

    print("\n--- W2-11 max_sync_dt 只能有一处定义（D50/D64 的同一类错）---")
    for mod, label in ((real, "real"), (bringup, "bringup")):
        if mod is None:
            continue
        names = [n for n, _ in declared_names(mod)]
        check(f"{label}.launch.py **不**声明 max_sync_dt（第二处定义会静默抢走）",
              "max_sync_dt" not in names)

    # ---------------------------------------- 板端只编子集时，非仿真分支仍能加载
    print("\n--- W2-15 板端只编子集：环境里没有 arm_grasp_sim 时，非仿真分支照样加载 ---")
    # 这条不是「防御性编程」，是**板端部署的既定做法**：Orin 上不需要任何仿真
    # 资产，按子集编译（`colcon build --packages-select yolo_bringup perception_3d
    # ...`）就是正常路径。而 launch 在**解析期**求值
    # `get_package_share_directory('arm_grasp_sim')`，`IfCondition` 只影响
    # **执行**、不影响解析 —— 所以只要有人把它写成 `IncludeLaunchDescription
    # (..., condition=...)`，缺包就会让**真机分支**（乃至 `--show-args`）一起
    # 加载失败，而报错会是「找不到 arm_grasp_sim」，把方向指错。
    # 用「查包就抛」的探测器来证明：真机/外部分支**连查都不查**。
    if bringup is None:
        skip("arm_grasp_sim 缺包容错", "bringup.launch.py 不存在")
    else:
        real_lookup = bringup.get_package_share_directory

        def _boom(pkg):
            # ⚠️ 只对 arm_grasp_sim 炸。本文件**确实**会在解析期查 `yolo_bringup`
            # 自己（D50 的修复把 grasp_planner.yaml 的路径解在
            # `get_package_share_directory('yolo_bringup')` 上）—— 那个包就是
            # 正在加载的这个包，永远存在，查它是**对的**。探测范围放大到「任何
            # 查包都炸」会把这条正确行为误判成缺陷（第一版就是这么写的，被抓到了）。
            if pkg == "arm_grasp_sim":
                raise AssertionError(f"不该在这里查包：{pkg}")
            return real_lookup(pkg)

        def _probe(src, lookup):
            """换掉查包函数后跑一次 _sim_backend，返回 (结果, 异常描述)。"""
            bringup.get_package_share_directory = lookup
            try:
                return bringup._sim_backend(ctx(camera_source=src)), ""
            except Exception as exc:  # noqa: BLE001 —— 这里就是要看它抛什么
                return None, f"{type(exc).__name__}: {exc}"

        for src in ("realsense", "external"):
            out, err = _probe(src, _boom)
            check(f"camera_source:={src} → 一个包都不查、返回空列表",
                  out == [] and not err,
                  err or (f"返回了 {out!r}" if out != [] else ""))

        # 最强的一条：整个入口在「没有 arm_grasp_sim」的环境里必须能构造出来
        bringup.get_package_share_directory = _boom
        try:
            desc = bringup.generate_launch_description()
            ops = [type(e).__name__ for e in desc.entities
                   if type(e).__name__ == "OpaqueFunction"]
            # 真正的结构证据：两个后端都是 OpaqueFunction（执行期构造），
            # 而不是 `IncludeLaunchDescription(..., condition=...)`。
            check("缺 arm_grasp_sim 时 generate_launch_description() 仍能构造（解析期不查包）",
                  len(desc.entities) > 0 and len(ops) == 2,
                  f"实体 {len(desc.entities)} 个，其中 OpaqueFunction {len(ops)} 个（应为 2）")
        except Exception as exc:  # noqa: BLE001
            check("缺 arm_grasp_sim 时 generate_launch_description() 仍能构造（解析期不查包）",
                  False, f"{type(exc).__name__}: {exc}")

        # 反过来：真选 sim 且缺包时，报错必须**可操作** —— 给出补编命令与绕开办法。
        def _missing(pkg):
            raise bringup.PackageNotFoundError(pkg)

        out, err = _probe("sim", _missing)
        check("camera_source:=sim 且缺包 → 抛 RuntimeError（不是静默空启动）",
              out is None and err.startswith("RuntimeError"), err or "没有抛")
        msg = err
        for want, why in (
            ("colcon build --packages-select arm_grasp_sim", "给出补编命令"),
            ("camera_source:=realsense", "给出绕开办法（真机部署就是这条路）"),
            ("Orin", "点明板上不需要仿真资产，免得用户以为部署缺东西"),
        ):
            check(f"报错里{why}", want in msg,
                  "" if want in msg else f"缺 {want!r}")

        # 不能只证明「坏路径坏得对」——还得证明**好路径仍然好**（否则上面几条
        # 在一个彻底坏掉的函数上也会全 PASS）。
        bringup.get_package_share_directory = lambda pkg: f"/nonexistent/{pkg}"
        out, err = _probe("sim", bringup.get_package_share_directory)
        check("包在时 camera_source:=sim 仍然 include 仿真后端（上面几条不是空断言）",
              isinstance(out, list) and len(out) == 1 and not err,
              err or f"返回了 {out!r}")

        bringup.get_package_share_directory = real_lookup

    # ------------------------------------ 传给「别人的」launch 文件的参数名必须真实存在
    print("\n--- W2-16 传进被 include 的文件里的参数名，必须真被那个文件声明 ---")
    # 为什么值得单独钉：**参数名拼错在 launch 里是静默的**。`launch_arguments`
    # 写一个上游没声明的键，launch 只是把它塞进 launch_configurations，上游照旧
    # 用自己的默认值 —— 不报错、不警告，行为与「没传」完全一样。于是
    # `"launch_servo": "false"` 一旦写成 `"launch_servoo"`，就悄悄退回 `true`，
    # 起一个**能直接指挥机器人**的节点（D59）。真机上这种错要靠「怎么 servo 起来了」
    # 才被发现，而那时候臂已经接上了。
    #
    # 做法是**问 launch 自己**而不是猜：载入 real.launch.py，把三个 include 拿
    # 出来，读它真正持有的 launch_arguments，再拿被 include 的文件的声明去比。
    # ⚠️ 读私有字段是有意的（同 `node_params`）：`IncludeLaunchDescription` 的
    # 这两个东西没有公开访问器，而我们在执行前读。
    if real is None:
        skip("include 参数名核对", "real.launch.py 不存在")
    else:
        from launch import LaunchContext
        from launch.actions import IncludeLaunchDescription
        from launch.utilities import perform_substitutions

        confs = {"start_ur_driver": "true", "start_moveit": "true",
                 "start_pipeline": "true", "robot_ip": "1.2.3.4"}

        def _includes(mod):
            """[(来源标签, IncludeLaunchDescription 实体, 出错说明)]。

            三个 include 分两处产出：驱动与 move_group 是 `_driver` / `_moveit`
            两个 OpaqueFunction **执行期**返回的，pipeline 那个在
            `generate_launch_description()` 的实体树里（裹在 IfCondition 里）。
            两边都取，别只取一处 —— 只取实体树会漏掉两个 ur_*，
            那样「比过几条」的对照还会假装通过。
            """
            out = []
            for fn in (mod._driver, mod._moveit):
                try:
                    ents = list(fn(ctx(**confs)))
                except Exception as exc:  # noqa: BLE001 —— 缺包时这里会抛（正常）
                    out.append((fn.__name__, None, f"{type(exc).__name__}: {exc}"))
                    continue
                out += [(fn.__name__, e, None) for e in _walk(ents)
                        if isinstance(e, IncludeLaunchDescription)]
            out += [("generate_launch_description", e, None)
                    for e in _walk(mod.generate_launch_description().entities)
                    if isinstance(e, IncludeLaunchDescription)]
            return out

        seen = 0
        for label, e, err in _includes(real):
            if err is not None or not isinstance(e, IncludeLaunchDescription):
                # 缺包（本机没有 ur_robot_driver）—— 按仓库约定 SKIP 并给原因，
                # 不当成通过。板子上这几个包都在，同一条断言在板上会真跑。
                skip(f"{label} 的 include 参数名核对",
                     f"环境里拿不到：{err.splitlines()[0] if err else '?'}")
                continue
            src_obj = e._IncludeLaunchDescription__launch_description_source
            # 路径藏在 `location` 里，是一串 substitution（**没有** `launch_file_path`
            # 这种属性 —— 找它只会得到 None，于是整条断言静默变 SKIP，第一版就是这样）。
            path = perform_substitutions(
                LaunchContext(), src_obj._LaunchDescriptionSource__location)
            keys = sorted(k for k, _ in e._IncludeLaunchDescription__launch_arguments)
            if not path or not keys:
                skip(f"{label} 的 include 参数名核对", "没读到路径或键")
                continue
            # 先认**本仓库的源码**：`LAUNCH_DIR` 下的同名文件就是它。
            #   · `install/` 是拷贝安装、可能滞后（launch 改了要重新 colcon build）；
            #   · `load_launch` 还**故意**把查包函数换成 `/nonexistent/<pkg>`，
            #     所以本仓库文件的路径到这里本来就是假的。
            # 这一条要在「解包路径」**之前**判：否则板子上没 colcon build 时，
            # yolo_bringup 解不出来，本仓库自己的 pipeline.launch.py 反而被 SKIP
            # （第一版就是这个顺序，板子上看到的结果就是这么来的）。
            local = LAUNCH_DIR / Path(path).name
            if local.is_file():
                target = local
            else:
                # 别人包的路径是假的时候，用**真的**查包函数重新解一次 ——
                # 否则本机明明装了 ur_moveit_config，也会因为读到
                # `/nonexistent/...` 而 SKIP，白白少测一条。
                if path.startswith("/nonexistent/"):
                    rest = path[len("/nonexistent/"):]
                    pkg, tail = rest.split("/", 1)
                    try:
                        path = os.path.join(_gpsd(pkg), tail)
                    except PackageNotFoundError:
                        skip(f"{label} 的 include 参数名核对",
                             f"没装 {pkg}（不在 /opt/ros/humble 里）")
                        continue
                target = Path(path)
            mod = load_launch_at(target)
            if mod is None:
                skip(f"{label} 的 include 参数名核对", f"载入不了 {target}")
                continue
            declared = {n for n, _ in declared_names(mod)}
            undeclared = [k for k in keys if k not in declared]
            seen += 1
            check(f"{Path(path).name}：real.launch.py 传了 {len(keys)} 个参数，"
                  f"都在它自己的声明里（{label}）",
                  bool(declared) and not undeclared,
                  f"它没声明: {undeclared}" if undeclared else
                  ("" if declared else "读不到任何声明 —— 断言是空的"))

        # 正向对照：**必须**至少比过一条（pipeline 那个是本仓库自己的文件，
        # 永远拿得到）。否则「一条都没比到」会伪装成全部通过。
        check("至少真的比过一条 include（否则上面几条是空断言）", seen >= 1,
              f"实际比了 {seen} 条")

    # ------------------------------------------- W2-9 收工时的样子（D70）
    # 2026-09-28 板端实测：同一份 launch 里，有 `rclpy.ok()` 闸门的
    # planner_node 报 `process has finished cleanly`，没有它的 detector_node /
    # projector_node 每次都报 `process has died ... exit code 1` ＋ 一段
    # traceback。功能上无害，但**每次正常收工都像崩了**。
    print("\n" + "-" * 56)
    print("W2-9 收工时必须是「cleanly」，不是「died」（D70）")
    seen_mains = 0
    for rel in NODE_MAINS:
        path = PROJECT_ROOT / rel
        if not path.exists():
            check(f"{Path(rel).name}：文件在", False, f"{path} 不存在")
            continue
        calls, unguarded, spin_ok, race_ok, pub_lines, teardown, teardown_bad = \
            shutdown_hygiene(path)
        name = Path(rel).name
        if not calls:
            check(f"{name}：main() 里找得到 rclpy.shutdown()", False,
                  "一处都没找到 —— 断言会是空的（别让它静默通过）")
            continue
        seen_mains += 1
        check(f"{name}：{len(calls)} 处 rclpy.shutdown() 都在 `if rclpy.ok():` 里",
              not unguarded,
              f"没挡住的行: {unguarded}" if unguarded else "")
        if spin_ok is None:
            skip(f"{name}：spin 的 try 接了 ExternalShutdownException",
                 "main() 里没有 rclpy.spin —— 不适用")
        else:
            check(f"{name}：spin 的 try 接了 ExternalShutdownException", spin_ok,
                  "" if spin_ok else
                  "只接 KeyboardInterrupt 的话，Ctrl-C 会带着 traceback 逃出 main()")
            if not pub_lines:
                skip(f"{name}：收工竞态有 `if rclpy.ok(): raise` 兜底",
                     "本文件在回调里一处 publish 都没有 —— 那条竞态不适用")
            else:
                check(f"{name}：收工竞态有 `if rclpy.ok(): raise` 兜底", bool(race_ok),
                      "" if race_ok else
                      f"回调里有 {len(pub_lines)} 处 publish（行 "
                      f"{pub_lines}）：SIGINT 落在其中一行时，异常会从 spin_once "
                      f"重抛出来、绕过上面那个 except"
                      f"（板端实测：publisher's context is invalid）")
            # 第 4 条：收工清理本身要扛得住第二次 SIGINT（`ros2 launch` 会把
            # SIGINT 转发给子进程，于是子进程收到两次）。
            if not teardown:
                skip(f"{name}：收工清理扛得住第二次 SIGINT",
                     "main() 里没有 destroy_node —— 不适用")
            else:
                check(f"{name}：{len(teardown)} 处 destroy_node() 挡住了 KeyboardInterrupt",
                      not teardown_bad,
                      f"没挡住的行: {teardown_bad}（第二次 SIGINT 落进去就是一段 "
                      f"traceback ＋ `process has died ... exit code -2`）"
                      if teardown_bad else "")
    # 正向对照：**必须**真的查到文件。查不到就是「0 个文件全部通过」那种假绿。
    check("至少真的查了 3 个节点文件（否则上面几条是空断言）", seen_mains >= 3,
          f"实际查了 {seen_mains} 个")

    # ------------------------------------------------------------------ 收尾
    print("\n" + "=" * 56)
    if _skipped:
        print(f"跳过 {len(_skipped)} 项: {_skipped}")
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
