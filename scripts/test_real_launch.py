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

import importlib.util
import sys
import types
from pathlib import Path

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


def _src(name):
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


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
