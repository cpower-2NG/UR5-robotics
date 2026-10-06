#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`board_run.py` 的自检 —— **纯离线**：不连板子、不连控制柜、不起真栈。

为什么这里测得了
----------------
`board_run.py` 顶层**不 import rclpy / launch**（AST 钉子见 [1]），判定逻辑全在纯函数
里，I/O 只在几个薄适配器里，且那些适配器都能注入。本机（WSL）的 `/opt/ros/humble`
下**根本没装 rclpy**，顶层一 import 这个文件连启动都过不去 —— 而离线层正是这套东西
**唯一**能成测的地方。

所以本文件问的是「**这套判定会不会放行坏的东西**」，不是「板子现在通不通」。
板端那一层我跑不了（没有板子访问权，且每次运动都要当次授权）—— [11]/[12] 把
「离线的整跑必须退 2」「拼错的参数拦在起栈之前」用假 `ros2` 补上；其余板端项
（真 Dashboard、真串口、真控制器）**一律没验**，本文件不假装验过。

这一份特别钉的三件事
--------------------
1. **M6（拼错不报错）**：往拼出的 launch 参数里注入一个拼写错
   （`target_plane_z` → `target_plane_zz`），断言声明集检查**真的**变红，而且红到
   **栈根本不起**。写法是「断言**改前 ≠ 改后**」，不是「改坏了会红」——后者自说自话：
   一条恒红的判据也能满足它。
2. **判分器解不出来时不许静默跳过**（跳过 = 没判分却报绿）。`config/tasks/a2.yaml`
   就是靠这一点现成地立着：它的 `board_a2:grade` 还不存在。
3. **没验到（SKIP）不许渲染成绿，也不许算通过** —— 离线整跑**必须**退 2。

⚠️ 本文件在**板子上**跑会真的起栈（[12] 那两条）。所以 [12] 用的是**假 `ros2`**，
   并且 P3 一失败就立刻收栈；[11] 的整跑走的是「PATH 里没有 ros2」那一档。
   真机动作要当次授权（红线 6），自检不许替人做主。

用法:
    python3 scripts/test_board_run.py
退出码 0 = 全通过，1 = 有失败项，2 = 环境不满足。
"""

import sys

sys.dont_write_bytecode = True   # 防 .pyc 假绿（仓库约定）

import ast                        # noqa: E402
import contextlib                 # noqa: E402
import datetime                   # noqa: E402
import importlib.util             # noqa: E402
import io                         # noqa: E402
import json                       # noqa: E402
import os                         # noqa: E402
import re                         # noqa: E402
import shutil                     # noqa: E402
import subprocess                 # noqa: E402
import tempfile                   # noqa: E402
import time                       # noqa: E402
import types                      # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import board_a1                   # noqa: E402
import board_gate                 # noqa: E402
import board_lib                  # noqa: E402
import board_run                  # noqa: E402

RUN_PATH = os.path.join(HERE, "board_run.py")
SRC = open(RUN_PATH, encoding="utf-8").read()
TREE = ast.parse(SRC)

_failed = []


def check(name, cond, detail=''):
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        _failed.append(name)


def section(t):
    print("\n" + t)


def mentions(text, *keys):
    return all(k in text for k in keys)


def toplevel_imports(path):
    """**顶层** import 的名字（不递归进函数体）。

    为什么用 AST 而不是 `"import rclpy" not in src`：本文件的说明里就得写
    「顶层不 import rclpy」这句话，字符串判据会把**说明**当成**代码**
    （`test_board_lib.py` 里有同样的先例与理由）。
    """
    out = []
    for node in ast.parse(open(path, encoding="utf-8").read()).body:
        if isinstance(node, ast.Import):
            out += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            out.append(node.module or "")
    return out


def string_literals(tree=TREE):
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)]


def func_names(tree=TREE):
    return [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]


def func_code(name, tree=TREE, src=SRC):
    """某个函数的**代码**（去掉 docstring）。判「它到底会不会调某个东西」时用 ——
    把说明文字算进去，会让「文档里写了别这么干」变成「它这么干了」。"""
    node = [n for n in tree.body
            if isinstance(n, ast.FunctionDef) and n.name == name][0]
    seg = ast.get_source_segment(src, node) or ""
    head = node.body[0] if node.body else None
    if (isinstance(head, ast.Expr) and isinstance(head.value, ast.Constant)
            and isinstance(head.value.value, str)):
        doc = ast.get_source_segment(src, head)
        if doc:
            seg = seg.replace(doc, "", 1)
    return seg


def _split_raises(entry):
    try:
        board_run.split_entry(entry)
        return False
    except board_run.TaskError:
        return True


def ref_raises(ref):
    """这个点号引用形状不对吗（只判形状，不去 import）。"""
    try:
        board_run.parse_callable(ref)
        return False
    except board_run.CallableError:
        return True


def _stage_raises(record):
    try:
        board_run.stage_from_gate(record, "P4", board_run.GATE_STAGE_ITEMS["P4"])
        return False
    except board_run.GateError:
        return True


def _mutate(src, old, new, tag):
    """把 `src` 里 `old` 换成 `new`。**锚点找不到就记一次失败** —— 锚点漂了而变异
    「照样跑过」，那是假绿（`test_board_gate.py` 同款）。"""
    if old not in src:
        check("变异锚点还在（%s）" % tag, False, "源码里找不到：%r" % old[:70])
        return None
    return src.replace(old, new, 1)


TMP_ROOT = None      # 由下面建起来（变异体也写进这里，跑完一起删）


def load_mutant(src_text, tag):
    """把改过的源码写成临时文件再 exec（与 `test_board_a1.py` 同一条路子）。

    它能跑起来，是因为 `board_gate` / `board_lib` 已经在本进程的 `sys.modules` 里，
    变异体的 `import board_gate` 直接命中（本机没有 ROS，重跑一遍顶层 import 是安全的）。
    """
    d = tempfile.mkdtemp(prefix="mut_%s_" % tag, dir=TMP_ROOT)
    p = os.path.join(d, "board_run_mut_%s.py" % tag)
    with open(p, "w", encoding="utf-8") as fh:
        fh.write(src_text)
    spec = importlib.util.spec_from_file_location("board_run_mut_%s" % tag, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def I(name, status, detail="", evidence=None):
    return board_gate.Item(name, status, detail, evidence)


def unquoted_scalar(text, key):
    """`key: 值` 里值**没加引号**的那一行（注释行不算）。没有就返回 None。

    launch 参数只有字符串一种形态：`carry_objects: true`（YAML 布尔）会被拼成
    `carry_objects:=True`，与 `:=true` 不是同一个字符串 —— 而那类差异**不报错**。
    """
    for ln in text.splitlines():
        s = ln.strip()
        if s.startswith("#") or not s.startswith(key + ":"):
            continue
        v = s.split(":", 1)[1].strip()
        if not (v.startswith('"') or v.startswith("'")):
            return ln.strip()
    return None


# ===================================================================== 夹具
# 真机 `ros2 launch … --show-args` 的输出形状：每条声明一行 `    '名字':`，
# 下面缩进的是说明与默认值。本机没有 ROS 跑不了真的，所以按这个形状**手抄**一份，
# 内容 = 三份入口文件里 `DeclareLaunchArgument` 声明过的全部名字（46 个）。
# [3] 会拿三份源码 AST 出来的名字**回头逐字对一遍** —— 源码改了名这里就该红，
# 红了就照着改夹具（夹具是「真机输出」的替身，改它不是作弊，是同步）。
FIX_SHOW_ARGS = """
Arguments (pass arguments as '<name>:=<value>'):

    'robot_ip':
        控制柜地址。真机必给（不给的话 UR 驱动起不来）。

        (default: None)

    'camera_source':
        相机后端。真机走 realsense。

        (default: 'sim')

    'camera_x':
        俯视相机外参平移 x（m）。⚠️ 现场值放 ~/.config/ur5-vision-grasp/site.yaml。
        'use_target_plane' 为 true 时还会用到 target_plane_z —— 见下面那条。

        (default: '0.0')

    'camera_y':
        俯视相机外参平移 y（m）。
        这四个值（camera_x/y/z + camera_q*）必须**同时**更新。

        (default: '0.0')

    'camera_z':
        俯视相机外参平移 z（m）。

        (default: '0.0')

    'camera_qx':
        俯视相机外参四元数 x。

        (default: '0.0')

    'camera_qy':
        俯视相机外参四元数 y。

        (default: '0.0')

    'camera_qz':
        俯视相机外参四元数 z。

        (default: '0.0')

    'camera_qw':
        俯视相机外参四元数 w。

        (default: '0.0')

    'model':
        YOLO 权重文件名。解析不到时 detector_node 直接退出，而**整栈其余照跑**。

        (default: 'block_best.pt')

    'device':
        推理设备。

        (default: 'cuda:0')

    'target_frame':
        点云/目标的参考帧。

        (default: 'base_link')

    'tf_required':
        planning 前是否必须等到 TF。

        (default: 'false')

    'headless':
        仿真里的开关，真机路径用不到。

        (default: 'false')

    'sim_start_delay':
        仿真启动延迟（秒）。

        (default: '0')

    'start_delay':
        规划器推迟启动的秒数。planner_params_file 也要一起给，否则静默用仿真几何。

        (default: '0.0')

    'publish_camera_tf':
        是否由 launch 发布相机 TF。

        (default: 'true')

    'planner_params_file':
        规划器参数文件（现场真值，仓库外）。

        (default: '')

    'use_target_plane':
        位置改用「像素射线 ∩ 平面」求得。

        (default: 'false')

    'target_plane_z':
        那块平面的高度（m）—— 真机是台面 0.0285，仿真是 -0.39。

        (default: '0.0285')

    'plane_depth_tol':
        深度与平面的最大容许差（m）。

        (default: '0.02')

    'gripper_backend':
        sim / ag95 / none。

        (default: 'sim')

    'carry_objects':
        规划器自己搬运（true）还是只动臂（false）。

        (default: 'false')

    'ik_group':
        MoveIt 的 IK 组名。

        (default: 'ur5_manipulator')

    'trajectory_action':
        规划器要用的轨迹 action。

        (default: '/joint_trajectory_controller/follow_joint_trajectory')

    'table_scene_enabled':
        是否往 planning scene 里塞桌面。

        (default: 'false')

    'skip_detector':
        调试用：不起 detector。

        (default: 'false')

    'skip_planner':
        调试用：不起规划器。

        (default: 'false')

    'ur_type':
        UR 型号。

        (default: 'ur5')

    'safety_limits':
        是否启用 UR 驱动的安全限速。

        (default: 'false')

    'start_ur_driver':
        是否起 UR 驱动。

        (default: 'true')

    'start_moveit':
        是否起 MoveIt。

        (default: 'true')

    'start_pipeline':
        是否起感知+规划流水线。

        (default: 'true')

    'realsense_params_file':
        realsense 的参数文件。

        (default: '')

    'depth_topic':
        深度图话题。

        (default: '/camera/camera/aligned_depth_to_color/image_raw')

    'camera_info_topic':
        相机内参话题。

        (default: '/camera/camera/color/camera_info')

    'detections_topic':
        检测结果话题。

        (default: '/detections')

    'max_sync_dt':
        彩色/深度同步的最大时间差（s）。

        (default: '0.05')

    'depth_scale':
        深度单位换算。

        (default: '0.001')

    'depth_grid':
        下采样格边长（m）。

        (default: '0.01')

    'depth_cluster_tol':
        聚类容差（m）。

        (default: '0.02')

    'depth_median_window':
        中值滤波窗口。

        (default: '5')

    'depth_buffer_len':
        深度缓存长度。

        (default: '30')

    'confidence_threshold':
        检测置信度下限。

        (default: '0.3')

    'tool0_to_fingertip':
        指尖相对 tool0 的偏移（m）—— 官方尺寸定不了「垫高」。

        (default: '0.0')

    'place_release_lift':
        放置后抬起高度（m）。

        (default: '0.01')
"""

LAUNCH_DIR = os.path.join(REPO, "ros2_ws", "src", "yolo_bringup", "launch")
FIX_SOURCES = ("bringup.launch.py", "real.launch.py", "pipeline.launch.py")


def launch_declared_names():
    """三份入口源码里 `DeclareLaunchArgument('名字', …)` 声明的名字（并集）。

    用 AST 而不是正则：这些声明**跨行**写（`DeclareLaunchArgument(` 换行后才给名字），
    单行正则会漏掉大半，而漏掉的表现是夹具「看起来对」。
    """
    out = set()
    for f in FIX_SOURCES:
        tree = ast.parse(open(os.path.join(LAUNCH_DIR, f), encoding="utf-8").read())
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, "id", "") == "DeclareLaunchArgument"
                    and node.args and isinstance(node.args[0], ast.Constant)):
                out.add(node.args[0].value)
    return out


# 站点夹具：**全是占位符**（本仓是公开副本，红线 3）。IP 用 RFC 5737 文档地址。
SITE_YAML = """\
robot_ip: 203.0.113.7
ros_domain_id: 99
planner_params: {planner}
gripper:
  port: /dev/serial/by-id/usb-<厂商>_<型号>_<序列号>-if00-port0
topcam:
  verified: "{today}"
  ttl_days: 1
  extrinsics:
    camera_x: "0.3412"
    camera_y: "-0.0521"
    camera_z: "1.0834"
    camera_qx: "0.0021"
    camera_qy: "0.7069"
    camera_qz: "-0.0026"
    camera_qw: "0.7073"
"""

PLANNER_YAML = """\
grasp_planner:
  ros__parameters:
    pick_zone: [0.30, 0.90, -0.18, 0.16, -0.45, -0.30]
    place_slots: [
      [0.44, 0.24], [0.60, 0.24], [0.76, 0.24], [0.44, -0.24],
    ]
    place_z: -0.35
    table_plane_z: -0.39
    trajectory_action: /joint_trajectory_controller/follow_joint_trajectory
    ik_group: ur5_manipulator
    gripper_backend: sim
    carry_objects: true
    table_scene_enabled: false
"""

# 假 ros2：**只在离线自检里用**。`--show-args` 打印夹具（= 真机那条命令的形状），
# 其余（也就是 launch）立刻退 0。每次被调用都会 `touch` 一个哨兵文件 —— 于是
# 「它到底有没有被调用过」是可观测的（[11] 靠它证明 --dry-run 不跑子进程）。
FAKE_ROS2 = """#!/bin/sh
: > "$(dirname "$0")/invoked"
case "$*" in
  *--show-args*) cat "$(dirname "$0")/showargs.txt"; exit 0;;
esac
exit 0
"""


def make_fake_ros2(tmp):
    """造一个假 ros2 → `(bindir, 哨兵路径)`。"""
    bindir = os.path.join(tmp, "fakebin")
    if not os.path.isdir(bindir):
        os.makedirs(bindir)
    with open(os.path.join(bindir, "showargs.txt"), "w", encoding="utf-8") as fh:
        fh.write(FIX_SHOW_ARGS)
    fake = os.path.join(bindir, "ros2")
    with open(fake, "w", encoding="utf-8") as fh:
        fh.write(FAKE_ROS2)
    os.chmod(fake, 0o755)
    return bindir, os.path.join(bindir, "invoked")


def tmp_env_files(tmp):
    """写站点 + 规划器参数文件 → `(site_path, planner_path)`。

    `verified` 取**今天**：写死日期的话夹具过几天就变成「超期」，
    而超期会让 [11]（--dry-run 应当退 0）莫名其妙地退 2。
    """
    planner = os.path.join(tmp, "planner.yaml")
    site = os.path.join(tmp, "site.yaml")
    with open(planner, "w", encoding="utf-8") as fh:
        fh.write(PLANNER_YAML)
    with open(site, "w", encoding="utf-8") as fh:
        fh.write(SITE_YAML.format(planner=planner,
                                  today=datetime.date.today().isoformat()))
    return site, planner


# 会影响判定的环境变量一律摘掉（`test_board_lib.py` 同款）。
DROP_ENV = ("ROS_DOMAIN_ID", "UR5VG_SITE", "UR_ROBOT_IP", "UR5VG_MANIFEST",
            "ROS_DISTRO", "AMENT_PREFIX_PATH")


def clean_env(**extra):
    e = {k: v for k, v in os.environ.items() if k not in DROP_ENV}
    e.update(extra)
    return e


def run_cli(*args, env=None):
    """跑一次**真的** `board_run.py`（子进程）。

    `stdin=DEVNULL`：不这样写的话，在**真终端**上跑本文件时子进程的 stdin 是 tty，
    「非交互 ⇒ 拒绝」那几条断言就会随「谁在跑测试」变结果（拿运气当判据）。
    给了 DEVNULL，交互档一律拿到 EOF —— 那正是「没有人在旁边授权」的样子。
    """
    return subprocess.run([sys.executable, RUN_PATH] + list(args),
                          capture_output=True, text=True, stdin=subprocess.DEVNULL,
                          env=clean_env(**(env or {})))


tmpdir = TMP_ROOT = tempfile.mkdtemp(prefix="board_run_test_")
site_path, planner_path = tmp_env_files(tmpdir)
try:
    # =================================================================
    # 1. 源码钉子
    # =================================================================
    section("[1] 源码钉子：离线可测的前提 + 红线不许被代码违反")

    tops = toplevel_imports(RUN_PATH)
    check("顶层没有 rclpy / launch（连 `import rclpy.x` 也不算数）",
          [m for m in tops if m.split(".")[0] in ("rclpy", "launch")] == [],
          "顶层 import：%s" % tops)
    check("  也没有顶层 import board_a1（它只能经任务描述的点号引用被找到 —— 所以"
          "「回落 board_a1.CAM」在结构上就做不到）",
          "board_a1" not in tops, "顶层 import：%s" % tops)
    check("  board_gate / board_lib 是顶层 import（复用的两块砖）",
          "board_gate" in tops and "board_lib" in tops)
    check("  没有 import socket —— 唯一的网络路径是 board_lib.dashboard（只读 29999）",
          "socket" not in tops and "board_lib.dashboard" in SRC)

    lits = {s.strip().lower() for s in string_literals()}
    bad = sorted(lits & {"play", "power", "stop", "pause", "brake release",
                         "play\n", "power\n", "stop\n"})
    check("源码里没有任何一条字符串就是 play/power/stop（Dashboard 收的就是这些词）",
          bad == [], "命中：%s" % bad)
    check("  不重实现控制器的写路径：没有 switch_controller 服务名、没有它的调用",
          "/controller_manager/switch_controller" not in SRC
          and "switch_controllers(" not in SRC)
    check("  写路径的唯一开关是 `--repair-controller`：CLI 上是 store_true，"
          "透传给 board_gate 时只在 repair=True 时拼进去（两处，没有第三处）",
          SRC.count('"--repair-controller"') == 2
          and 'ap.add_argument("--repair-controller", action="store_true"' in SRC
          and "repair=a.repair_controller" in SRC)
    check("收栈用 board_lib.teardown（不另抄一套 kill 逻辑）",
          "board_lib.teardown" in SRC)
    check("--stop 面对的是**别人的**进程 ⇒ 逐个 pid 发信号，绝不 killpg",
          "killpg" not in func_code("stop_stack"))

    funcs = func_names()
    for want in ("decide", "check_declared", "parse_declared_args", "resolve_callable",
                 "validate_task", "judge_preflight", "judge_place_slots",
                 "effective_params", "counted", "render_item", "render_summary",
                 "stage_from_gate", "gate_argv", "argv_json_report", "read_gate_json",
                 "gate_items", "judge_handoff_window", "read_verdict", "dig",
                 "candidate_margin_note", "judge_ros_env", "judge_domain",
                 "judge_score_refs", "judge_problems", "judge_cabinet", "is_playing",
                 "split_entry", "parse_callable", "build_launch_cmd", "params_to_argv",
                 "resolve_launch_params", "planner_yaml_params", "path_in_repo",
                 "find_task_file", "find_marker", "render_counts", "score_round",
                 "find_authorization_keys", "task_dotted_paths", "judge_write_path",
                 "judge_repair_outcome", "switch_payload_lines", "repair_hint",
                 "print_repair_hint", "task_params", "judge_entry_parity",
                 "resolve_launch_params"):
        check("  纯函数 %s() 在（判定是算出来的，不是散在 if 里的）" % want,
              want in funcs)

    # =================================================================
    # 2. 任务描述
    # =================================================================
    section("[2] 任务描述：a1 读得进、缺键点得出名、点号引用解得开")

    a1_path = os.path.join(REPO, "config", "tasks", "a1.yaml")
    a2_path = os.path.join(REPO, "config", "tasks", "a2.yaml")
    task_a1 = board_run.load_task_file(a1_path)
    task_a2 = board_run.load_task_file(a2_path)
    a1_text = open(a1_path, encoding="utf-8").read()
    a2_text = open(a2_path, encoding="utf-8").read()

    check("a1.yaml 读得进且零毛病", board_run.validate_task(task_a1.raw) == [],
          str(board_run.validate_task(task_a1.raw)))
    check("a1 的入口是 bringup.launch.py（不是 pipeline —— 那个解不出 IK 且不报错）",
          task_a1.entry == "yolo_bringup/bringup.launch.py", task_a1.entry)
    check("a1 的槽位数 = planner_params 里 place_slots 的个数（能推导的不复述）",
          task_a1.slots_total == 4, str(task_a1.slots_total))
    check("a1 的声明集不止 entry 一份（真机细分项是在 real/pipeline 里声明的）",
          len(task_a1.declared_entries) == 3, str(task_a1.declared_entries))
    # ⚠️ 只管 `params:` 里那几项：`launch.start_delay: 120` 是**任务字段**
    #    （启动器自己用 %g 拼成字符串），它不带引号是对的。
    quoted_keys = ("carry_objects", "gripper_backend", "use_target_plane",
                   "target_plane_z")
    check("a1.yaml 的 params 里这几个值**带引号**（`:=True` 与 `:=true` 不是同一个"
          "字符串，而且那种差异不报错）",
          all(unquoted_scalar(a1_text, k) is None for k in quoted_keys),
          "没带引号的行：%s" % [unquoted_scalar(a1_text, k)
                                for k in quoted_keys])

    check("a2.yaml（下一阶段的模板）形状合法",
          board_run.validate_task(task_a2.raw) == [])
    check("  a2 明说了自己只是模板/起点，别拿去上真机",
          mentions(a2_text, "模板", "别在阶段 2 契约落地之前拿它上真机"))

    bad_task = {"task": "x", "launch": {"entry": "yolo_bringup/bringup.launch.py"},
                "expect": {"slots_total": 4}}
    probs = board_run.validate_task(bad_task)
    check("缺键时**点名**缺的是哪一个（点号路径，一次列全）",
          mentions("\n".join(probs), "缺 launch.camera_source",
                   "缺 expect.min_candidates", "缺 score.callable"), str(probs))
    check("  槽位数写成 0 / 写成布尔都要点出来",
          len(board_run.validate_task(dict(
              bad_task, expect={"slots_total": 0, "min_candidates": True}))) >= 2)
    check("  入口写成 `pipeline.launch.py`（缺包名）要点出来",
          any("launch.entry" in p for p in board_run.validate_task(dict(
              bad_task, launch={"entry": "pipeline.launch.py"}))))
    check("  params 里给嵌套结构要点出来（launch 参数只有标量）",
          any("params" in p for p in board_run.validate_task(dict(
              bad_task, params={"a": {"b": 1}}))))
    check("  allow_skip 里出现 controllers 要点出来（那是闸门存在的理由）",
          any("controllers" in p for p in board_run.validate_task(dict(
              bad_task, launch={"entry": "yolo_bringup/bringup.launch.py",
                                "allow_skip": ["controllers"]}))))

    check("score.callable 解析得到 **board_a1:grade 本体**（不是同名替身）",
          board_run.resolve_callable("board_a1:grade") is board_a1.grade)
    check("  a1 的判分引用就是 \"board_a1:grade\"",
          task_a1.score_ref == "board_a1:grade", task_a1.score_ref)

    try:
        board_run.resolve_callable("board_a2:grade")
        a2_raised = False
    except board_run.CallableError as exc:
        a2_raised = "board_a2" in str(exc)
    check("a2 的 board_a2:grade **还不存在** ⇒ 抛 CallableError（大声，不静默）",
          a2_raised)
    check("  形状不对的引用也抛（没有冒号 / 两个冒号 / 空属性 / 空模块）",
          all(ref_raises(r) for r in ("board_a1", "a:b:c", "board_a1:", ":grade")))
    check("  合法的点号引用不误伤", not ref_raises("board_a1:grade"))

    v_state, v_ok = board_run.score_round(task_a2, [])
    check("  判分器解不出来时**不许**算通过：给 (说明, None)，说明里点出这件事",
          v_ok is None and "解不出来" in v_state, v_state.splitlines()[0])
    check("  判分结果形状不对（没有 ok 键）⇒ None，不是 True",
          board_run.read_verdict({"verdict": "ok"}) is None
          and board_run.read_verdict({"ok": False}) is False
          and board_run.read_verdict({"ok": True}) is True
          and board_run.read_verdict(None) is None)

    try:
        board_run.find_task_file("不存在的任务名")
        find_err = ""
    except board_run.TaskError as exc:
        find_err = str(exc)
    check("任务描述找不到时列出**找过的路径**（现场才知道往哪儿放）",
          mentions(find_err, "找过", "ur5-vision-grasp/tasks"))

    # =================================================================
    # 3. 声明集
    # =================================================================
    section("[3] 声明集：`--show-args` 的解析（含诱饵行）+ 夹具与 launch 源码对齐")

    declared = board_run.parse_declared_args(FIX_SHOW_ARGS)
    check("解析出 46 个声明名", len(declared) == 46, "得到 %d 个" % len(declared))
    check("真机要用的那几个都在里面",
          {"robot_ip", "planner_params_file", "camera_x", "camera_qw",
           "target_plane_z", "use_target_plane", "gripper_backend",
           "carry_objects", "start_delay", "model", "camera_source"} <= declared)

    decoys = [
        ("    'use_target_plane' 为 true 时还会用到 target_plane_z —— 见下面那条。",
         "说明行里引号包着名字，但没有行尾的 `':`"),
        ("        这四个值（camera_x/y/z + camera_q*）必须**同时**更新。",
         "说明行里裸提到名字"),
        ("        (default: 'block_best.pt')", "默认值那一行"),
        ("Arguments (pass arguments as '<name>:=<value>')", "抬头行的占位符"),
    ]
    for line, why in decoys:
        got = board_run.parse_declared_args(line)
        check("诱饵行不吃进来（%s）" % why, got == set(), "解析成 %s" % got)

    names = launch_declared_names()
    check("夹具与三份 launch 源码里的声明名**完全一致**（一个不多一个不少）",
          declared == names,
          "夹具多：%s；夹具少：%s" % (sorted(declared - names),
                                      sorted(names - declared)))
    check("  源码的声明确实是**跨行**写的（所以提取必须用 AST，单行正则会漏）",
          "DeclareLaunchArgument(\n" in open(
              os.path.join(LAUNCH_DIR, "real.launch.py"), encoding="utf-8").read())

    cmds = board_run.declared_cmds(task_a1.declared_entries)
    check("要给三份入口各跑一条 `--show-args`", len(cmds) == 3, str(len(cmds)))
    check("  每条都是 `ros2 launch 包 文件 --show-args`（**不启动任何节点**）",
          all(c[:2] == ["ros2", "launch"] and c[-1] == "--show-args"
              and c[3].endswith(".launch.py") for c in cmds), str(cmds))
    check("  入口形状不对时抛 TaskError，不猜",
          _split_raises("pipeline.launch.py")
          and not _split_raises("yolo_bringup/bringup.launch.py"))

    # =================================================================
    # 4. M6
    # =================================================================
    section("[4] M6 —— 拼错的 `k:=v` 不报错，只静默不生效（改前 ≠ 改后）")

    site = board_lib.load_site(path=site_path)
    params_before, sources, conflicts = board_run.resolve_launch_params(task_a1, site)
    # 旧入口（`board_a1.build_cmd`）会发的键集 —— [10]/[14] 的参数集守卫拿它当对照。
    # 组装 launch 参数的地方有两个（旧脚本 + 本启动器），守卫必须同时钉住两边。
    _ns = types.SimpleNamespace(robot_ip=site.robot_ip, params=site.planner_params,
                                start_delay=task_a1.start_delay, cam=None)
    entry_keys = [x.split(":=")[0] for x in board_a1.build_cmd(_ns) if ":=" in x]
    check("拼装零冲突（站点权威项没有被任务描述重复声明）", conflicts == [],
          str(conflicts))
    check("外参 7 个键全部来自站点，一个不少",
          set(board_lib.CAM_KEYS) <= set(params_before),
          "缺：%s" % sorted(set(board_lib.CAM_KEYS) - set(params_before)))
    check("  每个键都记下了来源（有效值 = 站点 ⊕ 任务，报告要逐项注明）",
          set(params_before) <= set(sources)
          and "站点" in sources["robot_ip"]
          and "a1.yaml" in sources["carry_objects"],
          sources.get("robot_ip", "?"))

    st_before, det_before = board_run.check_declared(list(params_before), declared)
    check("改前：全部键都在声明集里 ⇒ PASS",
          st_before == board_run.PASS, "%s：%s" % (st_before, det_before))

    mutated = dict(params_before)
    mutated["target_plane_zz"] = mutated.pop("target_plane_z")
    check("变异只改了**键名**，没改个数（否则「红」可能是撞上了别的原因）",
          len(mutated) == len(params_before) and set(mutated) != set(params_before))

    st_after, det_after = board_run.check_declared(list(mutated), declared)
    check("改后：**真的**变红（不是恒亮/恒红）",
          st_after == board_run.FAIL and st_before != st_after,
          "%s → %s" % (st_before, st_after))
    check("  而且红的是**拼错的那个键**，不是别的（漏报 = 没验）",
          "target_plane_zz" in det_after
          and "target_plane_z" not in det_after.replace("target_plane_zz", ""),
          det_after)
    check("  说明里点出「静默不生效」这层机理（现场才知道为什么值得花这一关）",
          "静默" in det_after)

    check("拿不到声明集 ⇒ SKIP（**没验到**），不是 PASS 也不是 FAIL",
          board_run.check_declared(list(params_before), set())[0] == board_run.SKIP)
    check("  空键集也不算通过（一个参数都没有 = 命令拼错了）",
          board_run.check_declared([], set())[0] == board_run.SKIP)

    only_entry = board_run.parse_declared_args(
        "\n    'model':\n        (default: 'block_best.pt')\n"
        "\n    'camera_source':\n        (default: 'sim')\n")
    st_only, det_only = board_run.check_declared(list(params_before), only_entry)
    check("反例（这就是 declared_entries 要写三份的原因）：只问 bringup 那一份 ⇒ "
          "**正确的**参数也会被报红",
          st_only == board_run.FAIL and "robot_ip" in det_only, det_only[:100])

    # =================================================================
    # 5. 阶段判定
    # =================================================================
    section("[5] 阶段判定：三态 + 退出码 0/1/2 —— 只写一遍，P0–P5 都走它")

    check("有 FAIL ⇒ 退 2，且 P0/P1/P2 阶段**不许**起栈（launched=False）",
          board_run.decide([I("x", board_run.FAIL)])[:2] == (2, False))
    check("有 FAIL 且栈已经起了 ⇒ 退 2 **并收栈**",
          board_run.decide([I("x", board_run.FAIL)], launched=True)[:2] == (2, True))
    check("有 SKIP（没验到）⇒ 退 2，即使一个 FAIL 都没有（fail-closed）",
          board_run.decide([I("x", board_run.SKIP)])[0] == 2)
    check("闸门全过但判分没过 ⇒ 退 1",
          board_run.decide([I("x", board_run.PASS)], verdict=False)[0] == 1)
    check("闸门全过 + 判分通过 ⇒ 退 0（**只有真跑完一轮才可能拿到**）",
          board_run.decide([I("x", board_run.PASS)], verdict=True)[0] == 0)
    check("  「判不了」（没有 ok 键）**不是**通过：read_verdict 给 None",
          board_run.read_verdict({}) is None)

    rows = [("P0", I("P0.a", board_run.PASS), None),
            ("P0", I("P0.b", board_run.FAIL), "dry-run"),
            ("P1", I("P1.c", board_run.SKIP), None)]
    check("缺席（声明缺席）的项**不计入**三态",
          [i.name for i in board_run.counted({"rows": rows})] == ["P0.a", "P1.c"])
    check("  按阶段取也只看计入的那些",
          [i.name for i in board_run.counted({"rows": rows}, "P0")] == ["P0.a"])

    check("站点不合法 ⇒ P0 就退 2，且**不收栈**（栈还没起，没东西可收）",
          board_run.decide([I("P0.站点配置", board_run.FAIL)])[:2] == (2, False))
    check("控制器闸门不过 ⇒ 退 2 + 收栈",
          board_run.decide([I("P3.controllers", board_run.FAIL)],
                           launched=True)[:2] == (2, True))
    check("闸门绿、判分没过 ⇒ 退 1 + 收栈（栈已经跑完了，收栈是收尾）",
          board_run.decide([I("P4.camera", board_run.PASS)], verdict=False,
                           launched=True)[:2] == (1, True))

    moved, note = board_run.judge_handoff_window(0.0, 200.0, 120.0)
    check("起栈 200s 后才停、start_delay=120 ⇒ 承认「规划器可能已经动过臂」"
          "（退出码 2 不许自称「什么都没发出去」）",
          moved and "可能已经启动" in note, note)
    moved, note = board_run.judge_handoff_window(0.0, 40.0, 120.0)
    check("起栈 40s 就停 ⇒ 定时器还没到点，话可以说得实",
          not moved and "还没到点" in note, note)
    check("没起栈 / --attach ⇒ 「有没有动过臂」明写**未判**，不含糊",
          board_run.judge_handoff_window(None, None, 120.0)[1].count("未判") == 1)

    note = board_run.candidate_margin_note(4, 4)
    check("候选 4 = 槽位 4 ⇒ 点名「拿不到收工判据」并给出该摆几块",
          note is not None and "≥5 块" in note and "拿不到" in note, note)
    check("候选 5 > 槽位 4 ⇒ 不出声（只在该出声的时候出声）",
          board_run.candidate_margin_note(5, 4) is None)
    check("候选数读不到 ⇒ 不出声（不拿「不知道」当「不够」）",
          board_run.candidate_margin_note(None, 4) is None)

    # =================================================================
    # 6. 外参
    # =================================================================
    section("[6] 外参：站点说了算；缺/过期一律退 2，绝不回落仓库里的 CAM")

    no_cam = dict(site.raw)
    no_cam["topcam"] = dict(no_cam["topcam"], extrinsics={})
    site_nocam = board_lib.parse_site(no_cam, path="<夹具：外参全缺>")
    probs = board_lib.validate_site(site_nocam)
    check("外参缺键时 validate_site 逐条点出来",
          any("外参缺键" in p for p in probs), str(probs))

    p_nocam, _, _ = board_run.resolve_launch_params(task_a1, site_nocam)
    check("  ⇒ 拼出来的命令里**一个外参都没有**（不给就是不给，不猜）",
          [k for k in p_nocam if k in board_lib.CAM_KEYS] == []
          and "camera_source" in p_nocam)
    cam_vals = {str(v) for v in board_a1.CAM.values()}
    check("  ⇒ 而且**绝不**回落 board_a1.CAM（7 个值一个都不许出现在命令里）",
          not (cam_vals & set(p_nocam.values())),
          "跑进命令里的 CAM 值：%s" % sorted(cam_vals & set(p_nocam.values())))
    check("  结构上也不可能回落：拼参数的那段代码里没有 CAM 字样",
          "CAM" not in func_code("resolve_launch_params"))
    check("  P0 会因此退 2（站点配置 FAIL ⇒ 不起栈）",
          board_run.decide([I("P0.站点配置", board_run.FAIL)])[0] == 2)

    # ---- `publish_camera_tf`：**显式**写下时以它为准 -------------------------
    # 推导那条本身不在这里钉 —— `board_run.judge_entry_parity` 专门管「本启动器发的
    # 键 vs 旧入口发的键」，那一节把正反向、四元数全零/缺项、以及「删掉推导 ⇒ 守卫
    # 当场红」的变异都钉住了，别在两处各写一份（两份断言漂开比没有更糟）。
    # 这里只留**它没覆盖**的那一支：推导要能被**显式值**否掉。
    # （`publish_camera_tf` 不在 `SITE_OWNED` 里，所以这不算「站点权威项被覆盖」。）
    explicit = board_run.Task(dict(task_a1.raw, params=dict(
        task_a1.params, publish_camera_tf="false")))
    p_exp, _, _ = board_run.resolve_launch_params(explicit, site)
    check("任务描述**显式**写了 publish_camera_tf ⇒ 以它为准"
          "（「我有外部 TF 源」这种刻意选择得让人说得出口）",
          p_exp.get("publish_camera_tf") == "false",
          "实际：%r" % p_exp.get("publish_camera_tf"))

    # ---- 退出码 2：两处必须**同待遇**（同一个码不许两种读法） ----------------
    # 2 在本仓的定义是「**环境不满足**，不是失败」（AGENTS.md §6）。`deploy_rc` 与
    # `tree_rc` 都只有**一处**退 2 —— `check_board_deploy.py:174`「需要 ROS 环境」、
    # `board_tree_manifest.py` 的「没有清单」—— 含义都是「这一项**没验到**」。
    # 曾经一个读成 FAIL、一个读成 SKIP：同一个码两种待遇，报告就没法信了。
    def _pre(**kw):
        f = {"leftovers": [], "port_busy": False, "deploy_rc": 0, "tree_rc": 0}
        f.update(kw)
        return dict((i.name, i.status) for i in board_run.judge_preflight(f))

    check("部署前置退 2（= 环境不满足）⇒ SKIP「没验到」，**不是** FAIL"
          "（它唯一那处退 2 是「需要 ROS 环境」，不可能意味着缺依赖）",
          _pre(deploy_rc=2)["P1.部署前置"] == board_run.SKIP,
          _pre(deploy_rc=2)["P1.部署前置"])
    check("  板子树退 2（= 没有清单）⇒ 同样 SKIP —— 同一个退出码不许两种待遇",
          _pre(tree_rc=2)["P1.板子树"] == board_run.SKIP)
    check("  但 1 仍然是 FAIL（「看到了，而且它不对」）—— 别把 SKIP 那支扩过头",
          _pre(deploy_rc=1)["P1.部署前置"] == board_run.FAIL
          and _pre(tree_rc=1)["P1.板子树"] == board_run.FAIL)
    check("  0 仍然是 PASS（两个方向都别改坏）",
          _pre()["P1.部署前置"] == board_run.PASS
          and _pre()["P1.板子树"] == board_run.PASS)

    old = dict(site.raw)
    old["topcam"] = dict(old["topcam"], verified="2020-01-01")
    site_old = board_lib.parse_site(old, path="<夹具：外参过期>")
    st, det = board_lib.freshness_gate(site_old)
    check("外参过期 ⇒ FAIL（俯视外参实测一天漂 0.945°）",
          st == board_run.FAIL and "超期" in det, det)
    check("  过期时 P1 退 2，**不起栈**",
          board_run.decide([I("P1.外参时效", st, det)])[:2] == (2, False))
    off_state, off_why = board_lib.freshness_gate(
        site_old, override="顶视相机没动过，急着验一次")
    check("  过期但人写了理由 ⇒ PASS 且**理由原样进报告**（破例要留痕）",
          off_state == board_run.PASS and "顶视相机没动过" in off_why, off_why)
    check("  空理由的破例被拒（没有理由的破例 = 没有破例）",
          board_lib.freshness_gate(site_old, override="   ")[0] == board_run.FAIL)

    # =================================================================
    # 7. board_gate 的记录
    # =================================================================
    section("[7] board_gate 的记录：读不出**不许**当成「没有 FAIL」")

    import argparse                                          # noqa: E402

    gate_args = argparse.Namespace(
        trajectory_action="/scaled_joint_trajectory_controller/follow_joint_trajectory",
        stable_for=8.0, allow_skip=["telemetry", "params"], require=None)
    rec_path = os.path.join(tmpdir, "gate.json")
    board_gate._write_json(
        rec_path, gate_args,
        {"nodes": ["grasp_planner"], "topics": ["/targets_3d"], "services": []},
        [I("controllers", "PASS", "stable 8s"),
         I("camera", "PASS", "22.3 Hz"),
         I("targets", "PASS", "4 个候选全部落在 pick_zone 内",
           {"points": [[0.44, -0.12, 0.37]] * 4}),
         I("detector", "PASS", "model=block_best.pt"),
         I("telemetry", "SKIP", "没收到遥测（调用方 --allow-skip 声明本档没有它）"),
         I("params", "SKIP", "读不到（调用方 --allow-skip 声明本档没有它）")],
        0, False, "99")
    rec = json.load(open(rec_path, encoding="utf-8"))

    pairs = board_run.stage_from_gate(rec, "P4", board_run.GATE_STAGE_ITEMS["P4"])
    by_name = dict((i.name, (i, ab)) for i, ab in pairs)
    check("六项都被换成 P4.* 并保留三态",
          len(pairs) == 6 and by_name["P4.controllers"][0].status == "PASS")
    check("--allow-skip 的项算**缺席**（调用方声明这一档本来就没有它），不是「没验到」"
          "—— 否则 skip_planner 的任务会恒定退 2",
          by_name["P4.telemetry"][1] is not None
          and by_name["P4.params"][1] is not None)
    check("  没被 allow-skip 的项不许蹭成缺席（controllers 永远不许）",
          by_name["P4.controllers"][1] is None)
    check("  改前：把六项全算进来 ⇒ 这一档退 0",
          board_run.decide([i for i, ab in pairs if not ab])[0] == 0)

    st, det = board_run.check_declared(["a"], board_run.parse_declared_args(""))
    check("拿不到声明集 ⇒ SKIP ⇒ decide 退 2（不是「无声明 = 全放行」）",
          st == board_run.SKIP and board_run.decide([I("P2.声明集", st, det)])[0] == 2)

    try:
        board_run.gate_items({"items": [{"item": "controllers"}]})   # 缺 status
        shape_raised = False
    except board_run.GateError as exc:
        shape_raised = "形状不对" in str(exc)
    check("记录形状不对 ⇒ 抛 GateError（读不出**不能**当成「没有 FAIL」）",
          shape_raised)
    check("  空 items 也不许当成「全过」—— 取不到要的那一项就抛",
          board_run.gate_items({"items": []}) == {} and _stage_raises({"items": []}))
    check("  记录整个读不出（文件不在）时，调用方只能拿到「没验到」",
          board_run.read_gate_json(os.path.join(tmpdir, "不存在.json"))[0] is None)

    argv3 = board_run.gate_argv("/tmp/x.json", only="controllers",
                                trajectory_action="A/B/c", repair=True, fast=True)
    skip3 = argv3[argv3.index("--allow-skip") + 1].split(",")
    check("P3 的闸门只要 controllers，其余五项全部声明缺席",
          "controllers" not in skip3
          and set(skip3) == set(board_gate.ITEM_NAMES) - {"controllers"}, str(skip3))
    check("  P3 用短窗口（探测 8s / 控制器 60s），写路径只经 --repair-controller",
          "--discover-timeout" in argv3 and "--repair-controller" in argv3)
    argv4 = board_run.gate_argv("/tmp/y.json", slots_total=4, min_candidates=4,
                                expect_params={"carry_objects": "true"},
                                allow_skip=["telemetry"],
                                trajectory_action="A/B/c")
    check("P4 的闸门带上槽位数/候选下限/要核对的参数",
          "--slots-total" in argv4 and "4" in argv4
          and "--min-candidates" in argv4 and "carry_objects=true" in argv4)
    check("  没给 repair 时绝不出现 --repair-controller（写路径默认关着）",
          "--repair-controller" not in argv4)
    check("  `--json-report` 的路径能找回来（否则读记录会读成「它没跑」）",
          board_run.argv_json_report(argv4) == "/tmp/y.json")

    # =================================================================
    # 8. 渲染
    # =================================================================
    section("[8] 渲染：没验到不许印成通过")

    line = "\n".join(board_run.render_item(I("P1.串口", board_run.SKIP, "没查")))
    check("SKIP 打印成 [SKIP]，并额外写一句「**没验到**，不计入通过」",
          "[SKIP]" in line and "没验到" in line and "不计入通过" in line)
    line = "\n".join(board_run.render_item(
        I("P1.串口", board_run.SKIP, "连不上控制柜"), absent="--dry-run"))
    check("声明缺席的行**不显示**底下的结论（免得被读成「测出来是坏的」）",
          "连不上控制柜" not in line and "这一档没测" in line)
    check("  缺席行也不写成 PASS / FAIL",
          "[PASS]" not in line and "[FAIL]" not in line)

    summary = "\n".join(board_run.render_summary([I("a", board_run.SKIP)], 2))
    check("退 2 的总结写「在**交棒之前**停下了」，并列出三档退出码的语义",
          "交棒之前" in summary and "1 = " in summary and "2 = " in summary)
    summary_dry = "\n".join(board_run.render_summary(
        [I("a", board_run.PASS)], 0, {"b": "dry"}, dry_run=True))
    check("--dry-run 退 0 **不许**印成「验收通过」（它的绿说的是文件，不是设备）",
          "计划立得住" in summary_dry and "不是**验收" in summary_dry)
    check("  真跑完的 0 才印「全绿（闸门全过 + 本任务判分通过）」",
          "全绿" in "\n".join(board_run.render_summary(
              [I("a", board_run.PASS)], 0, verdict_state="ok")))
    check("三态计数分开报（PASS/FAIL/SKIP 不是二值的）",
          board_run.render_counts([I("a", board_run.PASS), I("b", board_run.FAIL),
                                   I("c", board_run.SKIP)]) == (1, 1, 1))

    # =================================================================
    # 9. 参数来源与死键
    # =================================================================
    section("[9] 有效参数 = yaml ⊕ launch：逐项注明来源，5 个死键点名")

    yaml_vals, why = board_run.read_planner_yaml(planner_path)
    check("读得进 planner_params_file（读不进就该红一次、别猜）",
          yaml_vals is not None, why)
    eff = board_run.effective_params(yaml_vals, params_before,
                                     board_run.PLANNER_KEYS_OF_INTEREST)
    val, src = eff["carry_objects"]
    check("launch 命令行给过的键 ⇒ 来源写「launch 命令行（**覆盖** yaml）」",
          val == "true" and "覆盖" in src, "%s ← %s" % (val, src))
    val, src = eff["pick_zone"]
    check("只有 yaml 给过的键 ⇒ 来源写 planner yaml",
          val is not None and "yaml" in src and "死键" not in src, src)
    val, src = eff["ik_group"]
    check("5 个死键之一 ⇒ 来源里点出「死键…yaml 里写的不是有效值」",
          "死键" in src and "不是有效值" in src, src)
    val, src = eff["target_frame"]
    check("两处都没有 ⇒ 明写「未设置（节点自己的默认值）」，不编一个值",
          val is None and "未设置" in src, "%s ← %s" % (val, src))
    check("  死键名单来自 board_gate（不在这里再抄一份）",
          board_run.effective_params.__defaults__[0] is board_gate.DEAD_KEYS)
    check("  5 个死键一个不少地会被标出来",
          all("死键" in board_run.effective_params(
              {k: "x" for k in board_gate.DEAD_KEYS}, {}, board_gate.DEAD_KEYS)[k][1]
              for k in board_gate.DEAD_KEYS))

    check("planner yaml 的 place_slots 与任务描述对得上 ⇒ PASS",
          board_run.judge_place_slots(
              task_a1, board_lib.parse_site(
                  {"planner_params": planner_path})).status == board_run.PASS)
    check("  yaml 读不到 ⇒ FAIL（真机上给错路径会**静默用仿真几何**，先修这个）",
          board_run.judge_place_slots(task_a1, board_lib.parse_site(
              {"planner_params": os.path.join(tmpdir, "没有这个文件.yaml")})
          ).status == board_run.FAIL)
    slots5 = open(planner_path, encoding="utf-8").read().replace(
        "[0.44, -0.24],", "[0.44, -0.24], [0.60, -0.24],")
    p5 = os.path.join(tmpdir, "planner5.yaml")
    with open(p5, "w", encoding="utf-8") as fh:
        fh.write(slots5)
    check("  yaml 里槽位数与任务描述对不上 ⇒ FAIL（两边不一致时判分必然判错）",
          board_run.judge_place_slots(task_a1, board_lib.parse_site(
              {"planner_params": p5})).status == board_run.FAIL)
    noplace = os.path.join(tmpdir, "noplace.yaml")
    with open(noplace, "w", encoding="utf-8") as fh:
        fh.write("grasp_planner:\n  ros__parameters:\n    place_z: -0.35\n")
    check("  yaml 里没有 place_slots ⇒ SKIP（没验到），不是 PASS",
          board_run.judge_place_slots(task_a1, board_lib.parse_site(
              {"planner_params": noplace})).status == board_run.SKIP)

    conflict_task = board_run.Task(dict(task_a1.raw, params=dict(
        task_a1.params, robot_ip="198.51.100.9")))
    _, _, conflicts = board_run.resolve_launch_params(conflict_task, site)
    check("任务描述里重复声明 robot_ip ⇒ 当场报冲突（不静默让后者赢）",
          len(conflicts) == 1 and "robot_ip" in conflicts[0], str(conflicts)[:120])

    check("--json-report 指向仓库内 ⇒ 拒写（公开副本，运行记录必须在仓库外）",
          board_run.path_in_repo(os.path.join(REPO, "scripts", "x.json"))
          and not board_run.path_in_repo(os.path.expanduser("~/x.json")))

    # =================================================================
    # 10. 变异
    # =================================================================
    section("[10] 变异：把判定**真的**改坏，断言结论翻面（「改前 ≠ 改后」）")

    m = load_mutant(_mutate(SRC, "    if unknown:", "    if False:", "decl"), "decl")
    st_mut, _ = m.check_declared(list(mutated), declared)
    check("削掉「键不在声明集里」这一支 ⇒ 拼错的键**不再**被报出来"
          "（[4] 那条断言是吃劲的）",
          st_mut != board_run.FAIL, "变异后 status=%s" % st_mut)

    m = load_mutant(_mutate(SRC, "    if any(i.status == SKIP for i in items):",
                            "    if False:", "skip"), "skip")
    check("削掉「有 SKIP ⇒ 退 2」⇒ 没验到就变成退 0（fail-closed 那条是吃劲的）",
          m.decide([I("x", board_run.SKIP)])[0] == 0,
          "变异后 rc=%s" % m.decide([I("x", board_run.SKIP)])[0])

    m = load_mutant(_mutate(SRC,
                            "if (n in allow and it.status == SKIP) else None",
                            "if (False and it.status == SKIP) else None",
                            "allowskip"), "allowskip")
    pairs_mut = m.stage_from_gate(rec, "P4", m.GATE_STAGE_ITEMS["P4"])
    check("把 allow-skip 的项从「缺席」改回「没验到」⇒ 它们进三态计数、把这一档"
          "拖成退 2（[7] 那条断言是吃劲的）",
          all(ab is None for _, ab in pairs_mut)
          and m.decide([i for i, ab in pairs_mut if not ab])[0] == 2
          and board_run.decide([i for i, ab in pairs if not ab])[0] == 0)

    m = load_mutant(_mutate(SRC, "    if verdict is False:", "    if False:",
                            "verdict"), "verdict")
    check("削掉「判分没过 ⇒ 退 1」⇒ 判分没过也退 0（[5] 那条是吃劲的）",
          m.decide([I("x", board_run.PASS)], verdict=False)[0] == 0)

    # ---- 红线 6：授权与「未生效」这两支吃不吃劲 -------------------------
    spy = {"launch": {"allow_auto_switch": True}}
    m = load_mutant(_mutate(SRC, "        if AUTH_ISH_RX.search(p):", "        if False:",
                            "authkeys"), "authkeys")
    mut_hit, real_hit = m.find_authorization_keys(spy), board_run.find_authorization_keys(spy)
    check("削掉「看着像授权的键 ⇒ 报红」⇒ 数据文件又能冒充授权了（[13] 吃劲）",
          mut_hit == [] and real_hit != [],
          "改后报 %d 条 / 改前报 %d 条（%s）"
          % (len(mut_hit), len(real_hit),
             real_hit[0][0] if real_hit else "—"))

    m = load_mutant(_mutate(SRC, "        if k not in TASK_KNOWN_TOP:",
                            "        if False:", "topkeys"), "topkeys")
    probe = {"task": "x", "auto_repair": True}
    mut_p, real_p = m.validate_task(probe), board_run.validate_task(probe)
    check("削掉「顶层不认识的键 ⇒ 报红」⇒ 谁都能往 profile 里加键（[13] 吃劲）",
          not any("不认识的键" in x for x in mut_p)
          and any("不认识的键" in x for x in real_p),
          "改后 %d 条毛病 / 改前 %d 条" % (len(mut_p), len(real_p)))

    m = load_mutant(_mutate(SRC, "    if isatty():", "    if True:", "wpath"), "wpath")
    mut_w = m.judge_write_path(lambda: False)[0]
    real_w = board_run.judge_write_path(lambda: False)[0]
    check("削掉「非交互 ⇒ 拒绝写路径」⇒ 没有当次授权也能发切换（[13] 吃劲）",
          mut_w == board_run.PASS and real_w == board_run.FAIL,
          "改后 %s / 改前 %s" % (mut_w, real_w))

    m = load_mutant(_mutate(SRC, '    return FAIL, ("已执行但**未生效**（复核仍 %s）',
                            '    return PASS, ("已执行但**未生效**（复核仍 %s）',
                            "notify"), "notify")
    mut_r = m.judge_repair_outcome(True, board_run.FAIL)[0]
    real_r = board_run.judge_repair_outcome(True, board_run.FAIL)[0]
    check("削掉「已执行但未生效 ⇒ FAIL」⇒ 复核没过也报绿（[13] 吃劲）",
          mut_r == board_run.PASS and real_r == board_run.FAIL,
          "改后 %s / 改前 %s" % (mut_r, real_r))

    # 这条变异**就是当初那个缺陷**：站点给了外参，却不打开消费它们的发布器。
    m = load_mutant(_mutate(SRC, '    if "publish_camera_tf" not in params and ex:',
                            "    if False:", "camtf"), "camtf")
    mut_p, _, _ = m.resolve_launch_params(task_a1, site)
    mut_st, mut_det = m.judge_entry_parity(mut_p, entry_keys)
    real_st, _ = board_run.judge_entry_parity(params_before, entry_keys)
    check("削掉「站点给了外参 ⇒ 打开相机 TF 发布器」⇒ 参数集少一个键、**守卫当场红**"
          "（publish_camera_tf 当初就是这么漏掉的）",
          mut_st == board_run.FAIL and real_st == board_run.PASS
          and "publish_camera_tf" in mut_det,
          "改后 %s（%s） / 改前 %s"
          % (mut_st, [x for x in mut_det.split("\n") if "publish_camera_tf" in x][:1],
             real_st))

    m = load_mutant(_mutate(SRC, "    if t_launch is None or t_stop is None:",
                            "    if False:", "window"), "window")
    try:
        moved_mut, note_mut = m.judge_handoff_window(None, None, 120.0)
        window_wrong = bool(moved_mut) or "未判" not in note_mut
    except TypeError:      # 削掉 guard 后直接拿 None 相减 —— 同样证明它吃劲
        window_wrong = True
    check("削掉交接窗口的「不知道就明说」⇒ 它要么崩、要么编一个数（拿不知道当知道）",
          window_wrong)


    # =================================================================
    # 11. 整跑
    # =================================================================
    section("[11] 整跑：离线**必须**退 2（绝不能退 0）；--dry-run 不跑任何子进程")

    # PATH 洗成没有 ros2 的样子：这样「离线整跑退 2」是**确定**的，而不是取决于
    # 跑测试的人有没有 source 过 ROS（那个变量一进来，这条断言就变成碰运气）。
    no_ros = os.pathsep.join(
        p for p in clean_env().get("PATH", "").split(os.pathsep)
        if p and not os.path.exists(os.path.join(p, "ros2")))

    r = run_cli("--task", "a1", "--site", site_path,
                "--log", os.path.join(tmpdir, "a1.log"),
                "--json-report", os.path.join(REPO, "scripts", "_never_write.json"),
                env={"PATH": no_ros})
    check("离线整跑退 2（**绝不能退 0**：fail-closed）", r.returncode == 2,
          "rc=%s" % r.returncode)
    check("  红的理由是**说得出名字**的那一个（PATH 里没有 ros2）",
          "PATH 里没有 `ros2`" in r.stdout)
    check("  栈**没有起**（闸门没过就不起栈）",
          "栈**没有起**" in r.stdout and "起栈中" not in r.stdout)
    check("  P0 里该过的过了、该红的红了（红在上面说的那一项）",
          "[PASS] P0.站点配置" in r.stdout and "[FAIL] P0.ROS 环境" in r.stdout)
    check("  P1 一项都没跑（P0 就没过去）", "[PASS] P1." not in r.stdout)
    check("  --json-report 指向仓库内 ⇒ 拒写，且**真的没写**",
          "指向仓库内" in r.stdout
          and not os.path.exists(os.path.join(REPO, "scripts", "_never_write.json")))
    check("  报告末尾印出三档退出码的语义（0/1/2 各是什么）",
          mentions(r.stdout, "1 = ", "2 = ", "交棒"))

    bindir, sentinel = make_fake_ros2(tmpdir)
    r = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                env={"PATH": bindir + os.pathsep + no_ros})
    check("--dry-run 退 0，但总结里写明**这不是验收**",
          r.returncode == 0 and "计划立得住" in r.stdout
          and "不是**验收" in r.stdout, "rc=%s" % r.returncode)
    check("  --dry-run 不起栈，并明说绿只说明「计划拼得出来」",
          "起栈中" not in r.stdout and "栈没有起，也不该起" in r.stdout)
    check("  --dry-run **一个子进程都没跑**（PATH 里的假 ros2 一次都没被调用）",
          not os.path.exists(sentinel), "哨兵在 ⇒ 有东西被调用了")
    check("  打印了「会发什么命令」，每一项参数都标了来源",
          "ros2 launch yolo_bringup bringup.launch.py"
          in r.stdout.replace(" \\\n      ", " ")
          and "a1.yaml:params.carry_objects" in r.stdout
          and "站点" in r.stdout)
    check("  声明缺席的行明写「这一档没测」", "这一档没测" in r.stdout)

    r = run_cli("--task", "a2", "--site", site_path, "--dry-run",
                env={"PATH": no_ros})
    check("a2（下一阶段模板）在**静态**这层就被拦下：判分函数还不存在",
          r.returncode == 2 and "board_a2" in r.stdout
          and "[FAIL] P0.判分函数" in r.stdout, "rc=%s" % r.returncode)

    r = run_cli("--dry-run", env={"PATH": no_ros})
    check("不给 --task（又不是 --stop）⇒ 退 2 并说清怎么给",
          r.returncode == 2 and "--task a1" in r.stdout)

    # --stop：拿一个**真的**进程当残留（[12] 的哨兵已经证明这个路子可观测）。
    # 它的命令行里含 `planner_node` ⇒ 命中 board_lib 的残留判据；它由本测试起，
    # **不在** board_run 的祖先链上 ⇒ 不会被自我排除掉，于是这一条是**真收**。
    pre = board_lib.leftovers()
    if pre:
        print("  （提醒：动手前就有 %d 条别的残留，--stop 会连它们一起收 —— "
              "那正是它的语义）" % len(pre))
    decoy = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(120)", "planner_node"])
    try:
        time.sleep(0.5)
        check("先确认这个『残留』真的被判据看得见（否则下面的退 0 是空绿）",
              any(str(decoy.pid) == s.split()[0] for s in board_lib.leftovers()))
        r = run_cli("--stop")
        check("--stop 退 0，并明说「收栈不代替在示教器上按 STOP」",
              r.returncode == 0 and "按 STOP" in r.stdout, "rc=%s" % r.returncode)
        dead = False
        for _ in range(40):
            if decoy.poll() is not None:
                dead = True
                break
            time.sleep(0.1)
        check("  那个『残留』真的被收掉了（不是只在报告里说收了）", dead)
    finally:
        with contextlib.suppress(Exception):
            decoy.kill()
        with contextlib.suppress(Exception):
            decoy.wait(timeout=5)

    # =================================================================
    # 12. 端到端
    # =================================================================
    section("[12] 端到端（假 ros2）：拼错的参数**拦在起栈之前**（改前 ≠ 改后）")

    # 这一档把 P1 的事实**喂**进去（不连串口、不问控制柜、不跑那两个守卫脚本），
    # 因为要测的是 P2 的声明集闸门**能不能拦住起栈**。P3 也不问真控制柜
    # （把 dashboard 换成「连不上」）—— 真 Dashboard 那条路属于板端，本文件不假装验过。
    FACTS_GREEN = {
        "leftovers": [], "port": "/dev/serial/by-id/usb-<占位>-if00-port0",
        "port_busy": False, "gripper": (1, "ok"),
        "cabinet": {"robotmode": "RUNNING", "programState": "STOPPED",
                    "safetystatus": "NORMAL"},
        "deploy_rc": 0, "tree_rc": 0,
        "freshness": (board_gate.PASS, "外参核对于今天，保质期 1 天"),
    }

    def end_to_end(task_path, tag):
        """跑一次真的 `run()`：P1 全绿 → P2 起（假）栈 → P3 等不到播放 ⇒ 退 2 + 收栈。"""
        old_env = {k: os.environ.get(k) for k in
                   ("PATH", "ROS_DISTRO", "AMENT_PREFIX_PATH", "ROS_DOMAIN_ID")}
        old_install = board_run.INSTALL_SETUP
        old_collect = board_run.collect_preflight
        old_dash = board_lib.dashboard
        os.environ["PATH"] = bindir + os.pathsep + no_ros
        os.environ.update(ROS_DISTRO="humble", AMENT_PREFIX_PATH="/opt/ros/humble",
                          ROS_DOMAIN_ID="99")
        board_run.INSTALL_SETUP = os.path.join(bindir, "ros2")   # 假装 install 在
        board_run.collect_preflight = (
            lambda site, do_net=True, override=None: dict(FACTS_GREEN))
        board_lib.dashboard = lambda ip, timeout=5.0: None       # 问不到控制柜
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                rc = board_run.run(["--task", task_path, "--site", site_path,
                                    "--play-timeout", "0.1", "--timeout", "5",
                                    "--log", os.path.join(tmpdir, "e2e_%s.log" % tag),
                                    "--json-report",
                                    os.path.join(tmpdir, "e2e_%s.json" % tag)])
        finally:
            board_run.INSTALL_SETUP = old_install
            board_run.collect_preflight = old_collect
            board_lib.dashboard = old_dash
            for k, v in old_env.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
        return rc, buf.getvalue()

    if os.path.exists(sentinel):
        os.remove(sentinel)
    rc, out = end_to_end(a1_path, "ok")
    check("参数都对时：声明集检查过 ⇒ **栈真的起了**（下面那条对比才有意义）",
          "起栈中" in out and "[PASS] P2.声明集" in out
          and "[PASS] P2.槽位数" in out)
    check("  等不到播放 ⇒ 退 2，并**立刻收栈**（不把栈留给下一次）",
          rc == 2 and "收栈完成" in out and "[FAIL] P3.示教器 PLAYING" in out,
          "rc=%s" % rc)
    check("  板子不在线 ⇒ 控制器闸门是 SKIP（**没验到**），不是 PASS",
          "[SKIP] P3.控制器闸门" in out)
    rec_ok = json.load(open(os.path.join(tmpdir, "e2e_ok.json"), encoding="utf-8"))
    check("  JSON 记录写出来了，exit=2、launched_stack=true、缺席项不进 items",
          rec_ok["exit"] == 2 and rec_ok["launched_stack"] is True
          and rec_ok["absent"] == {}
          and all(it["item"].count(".") == 1 for it in rec_ok["items"]),
          "exit=%r launched=%r items=%d absent=%r"
          % (rec_ok["exit"], rec_ok["launched_stack"], len(rec_ok["items"]),
             rec_ok["absent"]))
    check("  记录里**逐项**留着这一轮的判定（当场可查的一手证据）",
          {it["item"] for it in rec_ok["items"]} >=
          {"P0.任务描述", "P1.外参时效", "P2.声明集", "P3.示教器 PLAYING",
           "P3.收栈"},
          str(sorted(it["item"] for it in rec_ok["items"])))
    check("  哨兵在 ⇒ 假 ros2 真的被调用过（这是 --dry-run 那条断言的对照组）",
          os.path.exists(sentinel))

    # ---- 同一个任务，只把 target_plane_z 拼成 target_plane_zz --------------
    typo_src = a1_text.replace('  target_plane_z: "0.0285"',
                               '  target_plane_zz: "0.0285"')
    check("变异确实改到了那一行", typo_src != a1_text)
    typo_path = os.path.join(tmpdir, "a1_typo.yaml")
    with open(typo_path, "w", encoding="utf-8") as fh:
        fh.write(typo_src)

    rc2, out2 = end_to_end(typo_path, "typo")
    check("M6 端到端：拼错一个字母 ⇒ 声明集红 ⇒ **栈根本不起**（退 2）",
          rc2 == 2 and "[FAIL] P2.声明集" in out2 and "起栈中" not in out2,
          "rc=%s；起栈=%s" % (rc2, "起栈中" in out2))
    check("  而且报告里指得出那个拼错的键", "target_plane_zz" in out2)
    check("  改前 ≠ 改后：改前「栈起了、红在 P3」，改后「栈压根没起、红在 P2」",
          ("起栈中" in out) != ("起栈中" in out2)
          and "P3.示教器 PLAYING" in out and "P3.示教器 PLAYING" not in out2)

    # =================================================================
    # 13. 红线 6：控制器切换的授权从哪来
    # =================================================================
    section("[13] 红线 6：控制器切换要**当次**授权 —— 数据文件不许冒充授权")

    # ---- ① 授权不许来自任务描述 --------------------------------------
    check("a1.yaml 的每个点号路径都干净（没有看着像授权的键）",
          board_run.find_authorization_keys(task_a1.raw) == [],
          str(board_run.find_authorization_keys(task_a1.raw))[:200])
    check("a2.yaml 同理（模板最会被复制、被沿用，最容易被塞这种键）",
          board_run.find_authorization_keys(task_a2.raw) == []
          and board_run.validate_task(task_a2.raw) == [])
    check("  a1 仍然过 validate_task（别把闸门做成谁都过不去）",
          board_run.validate_task(task_a1.raw) == [],
          str(board_run.validate_task(task_a1.raw))[:200])

    bad_l = {"launch": {"allow_auto_switch": True}}
    hit = board_run.find_authorization_keys(bad_l)
    check("`launch.allow_auto_switch` ⇒ 报红，并点名红线 6 与「当次」",
          len(hit) == 1 and hit[0][0] == "launch.allow_auto_switch"
          and "红线 6" in hit[0][1] and "当次" in hit[0][1], str(hit)[:200])
    check("  `params.repair_controller: \"true\"` 也拦（launch 参数是真能改行为的开关）",
          [p for p, _ in board_run.find_authorization_keys(
              {"params": {"repair_controller": "true"}})] == ["params.repair_controller"])
    check("  `expect.unattended: true` 也拦（换个体位塞进来照样红）",
          [p for p, _ in board_run.find_authorization_keys(
              {"expect": {"unattended": True}})] == ["expect.unattended"])
    check("  `launch.allow_skip` **不**拦：它说的是「这一档没有它」（缺席），"
          "不是「允许发写命令」",
          board_run.find_authorization_keys(
              {"launch": {"allow_skip": ["telemetry"]}}) == [])

    bad_top = {"task": "x", "auto_repair": True}
    probs = board_run.validate_task(bad_top)
    check("顶层出现一个不认识的键 ⇒ 报红（加键必须改代码，改了才有人复核）",
          any("auto_repair" in x and "不认识的键" in x for x in probs),
          str(probs)[:200])
    check("  **任意**新顶层键都拦得住，不只是名字像授权的那些",
          any("whatever" in x for x in board_run.validate_task(
              {"task": "x", "whatever": 1})))
    check("  允许的顶层键就是那 7 个（多一个都要改代码）",
          board_run.TASK_KNOWN_TOP == ("task", "launch", "params", "model",
                                       "start_delay", "expect", "score"))

    # ---- ② 当次：起栈之前就要有交互终端 ------------------------------
    st, det = board_run.judge_write_path(lambda: False)
    check("--repair-controller + 非交互终端 ⇒ **FAIL**（不是 SKIP：请求本身不成立）",
          st == board_run.FAIL)
    check("  理由里点名「当次」，并明说授权不许来自任务描述/环境变量/上次的记忆",
          "当次" in det and "任务描述" in det and "环境变量" in det
          and "红线 6" in det)
    check("  有交互终端才 PASS", board_run.judge_write_path(lambda: True)[0]
          == board_run.PASS)

    # ---- ② 被告知：报文要先完整摆出来 --------------------------------
    lines = "\n".join(board_run.switch_payload_lines(task_a1, timeout=8.0))
    check("报文里有**服务名**（人确认的是那一条，不是「大概会切一下」）",
          board_gate.SWITCH_SERVICE in lines)
    req1 = board_run._required_controllers_for(task_a1)
    check("  activate 列表 = 闸门实际会激活的那一份（同一份推导，不另抄）",
          req1 == ["scaled_joint_trajectory_controller", "joint_state_broadcaster"]
          and all(n in lines for n in req1), str(req1))
    check("  deactivate_controllers 明写 `[]`（内容安全：永不 deactivate）",
          "deactivate_controllers []" in lines)
    check("  而且报文里**没有**任何非空的 deactivate 列表",
          re.search(r"deactivate_controllers\s+\[(?!\])", lines) is None)
    check("  strictness 明写 BEST_EFFORT(=1)（STRICT 会让整批 abort）",
          "BEST_EFFORT(=1)" in lines)
    check("  timeout 明写出来", "8s" in lines)
    check("  并且明说授权只认这一次", "只认**这一次**" in lines)
    func_payload = func_code("switch_payload_lines")
    check("  服务名来自 board_gate.SWITCH_SERVICE，**不是**本文件里的字面量"
          "（写路径不在这里重实现）",
          "board_gate.SWITCH_SERVICE" in func_payload
          and "/controller_manager/switch_controller" not in func_payload)
    gate_src = open(board_gate.__file__, encoding="utf-8").read()
    check("  deactivate 恒空的那一份判定留在 board_gate 里（本文件只透传 flag）",
          "req.deactivate_controllers = []" in gate_src)
    check("  印出来的 strictness 与 board_gate 实际发的是同一个（改一边要改两边）",
          "SwitchController.Request.BEST_EFFORT" in gate_src)
    check("  本文件从不给 board_gate 传 --require ⇒ 两边算的是同一份 activate",
          '"--require"' not in SRC and "'--require'" not in SRC)
    check("controller_stopper 那条**待查**线索留在注释里（是线索，不是事实；"
          "别据此删写路径）",
          "controller_stopper" in SRC and "待查" in SRC)

    # ---- ③ 可复验：切完只读复核，未生效不许绿 -------------------------
    check("没执行过切换 ⇒ 没有「修复」这一项（None ≠ PASS）",
          board_run.judge_repair_outcome(False, board_run.FAIL) == (None, ""))
    check("执行过且只读复核过了 ⇒ PASS（复核的是结果，不是「我们发成功了吗」）",
          board_run.judge_repair_outcome(True, board_run.PASS)[0] == board_run.PASS)
    st, det = board_run.judge_repair_outcome(True, board_run.FAIL)
    check("执行过但复核不过 ⇒ **FAIL**，文案直说「已执行但未生效」（不许绿）",
          st == board_run.FAIL and "已执行但**未生效**" in det)
    check("  「未生效」不是「没验到」：复核拿到 SKIP 也算 FAIL",
          board_run.judge_repair_outcome(True, board_run.SKIP)[0] == board_run.FAIL)

    buf = io.StringIO()
    sink = lambda *a: buf.write(" ".join(str(x) for x in a) + "\n")   # noqa: E731
    printed = board_run.print_repair_hint(board_run.FAIL, False, "a1", out=sink)
    check("控制器不过、又没走写路径 ⇒ 印**一行**修复命令（先报文 + 当次确认）",
          printed and "--repair-controller" in buf.getvalue()
          and "当次" in buf.getvalue() and "默认只读" in buf.getvalue(),
          buf.getvalue().strip()[:120])
    check("  那一行就是能直接粘的（含脚本路径与 --task）",
          board_run.repair_hint("a1").endswith(
              "scripts/board_run.py --task a1 --repair-controller"))
    buf2 = io.StringIO()
    sink2 = lambda *a: buf2.write(" ".join(str(x) for x in a) + "\n")  # noqa: E731
    check("  已经执行过切换就不印修复命令（那是「未生效」，不是「没修」）",
          not board_run.print_repair_hint(board_run.FAIL, True, "a1", out=sink2)
          and buf2.getvalue() == "")
    check("  闸门过的时候也不印（没什么可修的）",
          not board_run.print_repair_hint(board_run.PASS, False, "a1", out=sink2))

    # ---- 端到端：非交互 + --repair-controller ⇒ 起栈之前就退 2 --------
    r = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                "--repair-controller", env={"PATH": no_ros})
    check("真跑一次：--repair-controller 在非交互终端 ⇒ P0 红、退 2",
          r.returncode == 2 and "[FAIL] %s" % board_run.WRITE_PATH_ITEM in r.stdout,
          "rc=%s" % r.returncode)
    check("  而且**停在 P0**：P1/P2 那两个阶段头一次都没印（拦在起栈之前）",
          "P1 前置" not in r.stdout and "P2 起栈" not in r.stdout
          and "起栈中" not in r.stdout)
    check("  报文在起栈之前就摆出来了（服务名 + 恒空的 deactivate）",
          board_gate.SWITCH_SERVICE in r.stdout
          and "deactivate_controllers []" in r.stdout)
    r_ro = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                   env={"PATH": no_ros})
    check("  同一份 profile 不带这个开关时 P0 里没有这一项（只读是默认）",
          board_run.WRITE_PATH_ITEM not in r_ro.stdout
          and "P2 起栈" in r_ro.stdout, "rc=%s" % r_ro.returncode)
    check("  P0 就把参数集摆出来了（P0 不过的话走不到 P2，披露不能只在顺利路径上）",
          "参数集（" in r_ro.stdout)
    r_ns = run_cli("--task", "a1", "--site", os.path.join(tmpdir, "没有这个站点.yaml"),
                   "--dry-run", env={"PATH": no_ros})
    check("  **站点读不到**时也把「已经知道的那部分」摆出来（最需要披露的失败）",
          r_ns.returncode == 2 and "参数集（" in r_ns.stdout
          and "只有任务描述能定的那部分" in r_ns.stdout
          and "P2 起栈" not in r_ns.stdout, "rc=%s" % r_ns.returncode)
    check("  并且明说站点那几项还不知道（robot_ip / 外参 / publish_camera_tf）",
          mentions(r_ns.stdout, "robot_ip", "camera_* 外参", "publish_camera_tf"))

    # =================================================================
    # 14. 参数集守卫：本启动器发出的键 vs 旧入口 board_a1.build_cmd
    # =================================================================
    section("[14] 两个入口的参数集：差集逐项有理由，少一个/多一个都红")

    check("旧入口 board_a1.build_cmd 解得出它的键集（对照物本身要立得住）",
          len(entry_keys) == 17 and "tf_required" in entry_keys
          and "publish_camera_tf" in entry_keys, str(sorted(entry_keys)))

    st, det = board_run.judge_entry_parity(params_before, entry_keys)
    check("a1 这条路径：两份参数集**差集逐项有理由** ⇒ PASS", st == board_run.PASS,
          det.replace("\n", " / ")[:300])
    check("  而且理由**印在结论里**（不是一张看不见的白名单）",
          "tf_required" in det and "model" in det and "少发" in det and "多发" in det)

    check("**publish_camera_tf 这个键发出来了**（当初漏的就是它：七个外参没人消费）",
          params_before.get("publish_camera_tf") == "true",
          "publish_camera_tf=%r" % params_before.get("publish_camera_tf"))
    check("  来源写明是**推导**（站点给了外参 ⇒ 打开消费它们的发布器），"
          "不是从 profile 里复述的",
          "推导" in sources["publish_camera_tf"]
          and "publish_camera_tf" not in task_a1.params)
    check("  命令行里**真的有**它（纯函数说得再好，发出去的才算）",
          "publish_camera_tf:=true" in r_ro.stdout)

    check("少一个键 ⇒ **红**，并点名是哪个（这就是 publish_camera_tf 当初那种漏）",
          board_run.judge_entry_parity(
              set(params_before) - {"publish_camera_tf"}, entry_keys)[0]
          == board_run.FAIL
          and "publish_camera_tf" in board_run.judge_entry_parity(
              set(params_before) - {"publish_camera_tf"}, entry_keys)[1])
    check("多一个键 ⇒ **也红**（手滑加的 / 拼错的都拦得住）",
          board_run.judge_entry_parity(
              set(params_before) | {"hand_tuned_z"}, entry_keys)[0] == board_run.FAIL
          and "hand_tuned_z" in board_run.judge_entry_parity(
              set(params_before) | {"hand_tuned_z"}, entry_keys)[1])
    st, det = board_run.judge_entry_parity(
        params_before, entry_keys, launched_only={"model": "x", "gone": "y"})
    check("差集表**过时** ⇒ 也红（某条理由不再成立时，不许留一张永远对的名单）",
          st == board_run.FAIL and "gone" in det and "不再差" in det)
    check("  理由表为空、两份又真的一致 ⇒ PASS 且写明「完全一致」",
          board_run.judge_entry_parity(["a", "b"], ["a", "b"],
                                       launched_only={}, entry_only={})[1]
          .endswith("**完全一致**"))
    check("  `tf_required` 那条理由**有出处**：bringup 按 camera_source 推导",
          "tf_required_default" in open(LAUNCH_DIR + "/bringup.launch.py",
                                        encoding="utf-8").read()
          or "'false' if '" in open(LAUNCH_DIR + "/bringup.launch.py",
                                    encoding="utf-8").read())

    # 全零四元数：**不**打开发布器（打开的话 real.launch.py 当场抛）
    real_src = open(LAUNCH_DIR + "/real.launch.py", encoding="utf-8").read()
    check("先把平台那一侧的判据钉住：默认值是 false，判据是 == \"true\"",
          'DeclareLaunchArgument(\n            "publish_camera_tf", default_value="false"'
          in real_src
          and 'launch_configurations.get("publish_camera_tf", "false") == "true"'
          in real_src)
    check("  且四元数全零时它**抛**（所以我们不能把全零那份也一起放过去）",
          "这不是一个合法的旋转" in real_src and "_is_zero" in real_src)
    zero_site = board_lib.parse_site({
        "robot_ip": "203.0.113.9", "planner_params": planner_path,
        "topcam": {"verified": "2026-10-06", "ttl_days": 1,
                   "extrinsics": {k: "0.0" for k in board_lib.CAM_KEYS}}})
    zp, zs, zc = board_run.resolve_launch_params(task_a1, zero_site)
    check("外参四元数全零 ⇒ **不**打开相机 TF 发布器，并且当场报出这条毛病",
          "publish_camera_tf" not in zp
          and any("全零" in x for x in zc), str(zc)[:200])
    part = board_lib.parse_site({
        "robot_ip": "203.0.113.9", "planner_params": planner_path,
        "topcam": {"verified": "2026-10-06", "ttl_days": 1,
                   "extrinsics": {"camera_x": "0.3", "camera_y": "0.0",
                                  "camera_z": "1.0"}}})
    pp, _, pc = board_run.resolve_launch_params(task_a1, part)
    check("  外参只给了一半（四元数缺项）⇒ 同样不开，不拿半份去撞 launch 的自检",
          "publish_camera_tf" not in pp and any("四元数" in x for x in pc),
          str(pc)[:160])
    nos = board_lib.parse_site({"robot_ip": "203.0.113.9",
                                "planner_params": planner_path, "topcam": {}})
    np_, _, _ = board_run.resolve_launch_params(task_a1, nos)
    check("  站点一个外参都没给 ⇒ 不擅自开发布器（缺项由 P0.站点配置 报，不重复喊）",
          "publish_camera_tf" not in np_)

finally:
    shutil.rmtree(tmpdir, ignore_errors=True)

print("\n" + "=" * 66)
if _failed:
    print("❌ %d 项没过：" % len(_failed))
    for n in _failed:
        print("   - " + n)
    sys.exit(1)
print("✅ 全部通过（%s）" % os.path.basename(__file__))
sys.exit(0)
