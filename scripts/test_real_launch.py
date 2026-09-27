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
4. **D64 回归**：唯一入口 `bringup.launch.py` **不得声明**真机专属参数。
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
