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


def I(name, status, detail="", evidence=None, **kw):
    # `**kw` 是给 `vacuous=` 走的（[5e] 要用）。夹具**转发**而不是自己再实现一遍
    # 三态 —— 抄一份的话，`Item` 加字段时这里会静默漏掉，测试就成了假绿。
    return board_gate.Item(name, status, detail, evidence, **kw)


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
# 内容 = 三份入口文件里 `DeclareLaunchArgument` 声明过的全部名字（48 个）。
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

    'grasp_point_mode':
        抓取点取框中心还是顶面中心。

        (default: 'bbox_center')

    'table_plane_z':
        台面高度（m），顶面中心修正要用。

        (default: '0.0')

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
    # ⚠️ ⚠️ **扁平**的 [x0,y0, x1,y1, …] —— **不是** [[x,y], …]。
    #    这里原来写的是嵌套的 `[[0.44,0.24], …]`，而真机节点**明确不接受**那个形状
    #    （planner_node.py:203 逐字写了是扁平的，:621 奇数长度直接抛）。
    #    后果：`judge_place_slots` 数 `len(place_slots)` 时，在**夹具上恒对**
    #    （嵌套 4 项 = 4 个槽）、在**真机上恒错**（扁平 8 项 = 4 个槽却读成 8）。
    #    这条缺陷因此藏到 2026-10-06 第一次真读现场文件才露头，还差点让我把一个
    #    「站点指错文件了」的错结论发给用户。**夹具必须照现场抄，不能按「显然的形状」写。**
    place_slots: [0.44, 0.24, 0.60, 0.24, 0.76, 0.24, 0.44, -0.24]
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
    # 写路径的开关是**闭集**：控制器切换 + 夹爪寄存器这**两条**。断言钉的是
    # 「每一条都：① CLI 上是 store_true（默认只读）② 过同一道 P0 授权闸门
    # ③ 出现次数就是下面这个数 ⇒ 多出一条没有闸门的写路径会当场红」。
    # 计数是**故意**的（照旧版对 `--repair-controller` 的做法）：新增一条写路径
    # 必须回来改这个数，改的时候就得说清它的授权闸门在哪。
    check("  写路径①控制器切换 `--repair-controller`：store_true、只在 repair=True "
          "时透传给 board_gate（三处，没有第四处）",
          SRC.count('"--repair-controller"') == 3
          and 'ap.add_argument("--repair-controller", action="store_true"' in SRC
          and "repair=a.repair_controller" in SRC)
    check("  写路径②夹爪寄存器 `--init-gripper`：store_true、过**同一道** "
          "judge_write_path（两处，没有第三处）",
          SRC.count('"--init-gripper"') == 2
          and 'ap.add_argument("--init-gripper", action="store_true"' in SRC
          and 'judge_write_path(\n            isatty, "--init-gripper"' in SRC)
    check("  两条写路径**共用**同一个授权判定（不是各写一份 —— 分叉的那一份会给"
          "出假的授权）",
          SRC.count("def judge_write_path(") == 1
          and SRC.count("judge_write_path(") == 3)
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
                 "playing_is_vacuous", "count_vacuous", "needs_repress",
                 "handoff_mode",
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
    noplan_path = os.path.join(REPO, "config", "tasks", "a1-noplan.yaml")
    task_a1 = board_run.load_task_file(a1_path)
    task_a2 = board_run.load_task_file(a2_path)
    task_noplan = board_run.load_task_file(noplan_path)
    a1_text = open(a1_path, encoding="utf-8").read()
    a2_text = open(a2_path, encoding="utf-8").read()

    check("a1.yaml 读得进且零毛病", board_run.validate_task(task_a1.raw) == [],
          str(board_run.validate_task(task_a1.raw)))
    check("a1 的入口是 bringup.launch.py（不是 pipeline —— 那个解不出 IK 且不报错）",
          task_a1.entry == "yolo_bringup/bringup.launch.py", task_a1.entry)
    check("a1 的槽位数 = planner_params 里 place_slots 的个数（能推导的不复述）",
          task_a1.slots_total == 4, str(task_a1.slots_total))

    # ---- 「这一档有没有规划器」写了两遍，两遍必须同向（2026-10-07 新增的 P0 判据）----
    # 真夹具仍然是仓库里那三份 profile。
    def _prof(raw, **params):
        """同一份真 profile，只改 `params` 里的几项（造矛盾用）。"""
        return dict(raw, params=dict(raw.get("params") or {}, **params))

    check("★ 三份真 profile 的「因/果」都同向 ⇒ 一份都不许被这条判据冤枉",
          all(board_run.validate_task(t.raw) == []
              for t in (task_a1, task_a2, task_noplan)),
          str({t.name: board_run.validate_task(t.raw)
               for t in (task_a1, task_a2, task_noplan)}))
    _contra = [x for x in board_run.validate_task(
        _prof(task_a1.raw, skip_planner="true")) if "互相矛盾" in x]
    check("★ `skip_planner: true` 而 **没**声明 allow_skip ⇒ P0 报「互相矛盾」"
          "（规划器其实不起，却没人声明 ⇒ P4 恒定退 2）", len(_contra) == 1,
          str(_contra))
    check("★ 声明了 allow_skip 而 **没**开 skip_planner ⇒ 也报（这个方向更危险："
          "规划器活着，`/grasp_planner/telemetry` 却**永远不被验**）",
          any("互相矛盾" in x for x in board_run.validate_task(
              dict(task_noplan.raw,
                   params={k: v for k, v in task_noplan.params.items()
                           if k != "skip_planner"}))))
    check("  矛盾时把**两边**的现状都印出来（不是只说「不一致」）",
          _contra and "不起" in _contra[0] and "会起" in _contra[0])
    check("a1 的声明集不止 entry 一份（真机细分项是在 real/pipeline 里声明的）",
          len(task_a1.declared_entries) == 3, str(task_a1.declared_entries))
    # ⚠️ 只管 `params:` 里那几项：`launch.start_delay: 120` 是**任务字段**
    #    （启动器自己用 %g 拼成字符串），它不带引号是对的。
    quoted_keys = ("carry_objects", "gripper_backend", "use_target_plane",
                   "target_plane_z", "grasp_point_mode", "table_plane_z")
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
    check("解析出 48 个声明名", len(declared) == 48, "得到 %d 个" % len(declared))
    check("真机要用的那几个都在里面",
          {"robot_ip", "planner_params_file", "camera_x", "camera_qw",
           "target_plane_z", "use_target_plane", "gripper_backend",
           "grasp_point_mode", "table_plane_z",
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
    # ⚠️ 这里要的是**键集**，不是值 ⇒ 外参给一组占位即可。
    #    2026-10-06 起 `board_a1.build_cmd` **没有外参就抛**（缺 `--site` 退 2，
    #    绝不回落那份作废的 `CAM`）—— 所以这里必须显式给一份，不能再传 `cam=None`。
    #    给占位而不是真值，也是刻意的：**仓库里不出现第二份外参真值。**
    _ns = types.SimpleNamespace(
        robot_ip=site.robot_ip, params=site.planner_params,
        start_delay=task_a1.start_delay,
        cam={k: "0.0" for k in board_lib.CAM_KEYS})
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

    # ---- 无运动档：时间推断**整条不适用**（2026-10-07 R0 run 6 实测的假警报）----
    # 真夹具 `task_noplan` 见 [2]（仓库里那份 a1-noplan.yaml，不是现造的 dict）。
    check("★ R0 档（a1-noplan.yaml）的 `skip_planner` 判成「规划器不起」"
          "（真夹具，不是现造的 dict）",
          board_run.planner_alive(task_noplan) is False,
          "skip_planner=%r" % task_noplan.params.get("skip_planner"))
    check("  A1 档（有运动）判成「规划器会起」—— 这是这条判据的另一半",
          board_run.planner_alive(task_a1) is True,
          "a1 的 params=%r" % sorted(task_a1.params))
    check("  规划器不起 ⇒ 起栈活了 273s 也**不许**说「可能已经启动并动过臂」"
          "（run 6 原话，🔴 + 红字，把这一档最该立的结论说反了）",
          board_run.judge_handoff_window(0.0, 273.0, 120.0,
                                         planner_alive=False)[0] is False
          and "根本不会起" in board_run.judge_handoff_window(
              0.0, 273.0, 120.0, planner_alive=False)[1]
          and "可能已经启动" not in board_run.judge_handoff_window(
              0.0, 273.0, 120.0, planner_alive=False)[1])
    check("  而且它说得出**为什么**不适用（引到 UnlessCondition 那一行，不是空口）",
          "UnlessCondition" in board_run.judge_handoff_window(
              0.0, 273.0, 120.0, planner_alive=False)[1])
    check("  规划器不起时，`--attach` 那一路也不许退回「未判」（不适用 ≠ 没验到）",
          board_run.judge_handoff_window(None, None, 120.0,
                                         planner_alive=False)[1].count("未判") == 0)
    check("  默认仍按「规划器会起」判（新增的这一维**不许**悄悄改掉老行为）",
          board_run.judge_handoff_window(0.0, 200.0, 120.0)[0] is True)
    check("  `skip_planner` 是**大写/带空格**也认（launch 参数是字符串，来自 yaml）",
          board_run.planner_alive(
              board_run.Task({"params": {"skip_planner": " True "}})) is False
          and board_run.planner_alive(
              board_run.Task({"params": {"skip_planner": "false"}})) is True
          and board_run.planner_alive(
              board_run.Task({"params": {}})) is True,
          "「没写」= 没声明不起 ⇒ 规划器会起")

    # ---- P4 不许抢在 `start_delay` 之前跑（2026-10-06 真机第 2 次实测的缺陷丙）----
    _G = board_run.P4_GRACE
    check("★ 起栈 10s 就体检、start_delay=120 ⇒ **必须拦住**（不然相机/检测器/感知"
          "本来就还没起，人按播放**按得快**反被判红）",
          board_run.p4_wait_seconds(0.0, 10.0, 120.0) == 120.0 - 10.0 + _G,
          str(board_run.p4_wait_seconds(0.0, 10.0, 120.0)))
    check("  已经等够（起栈 200s，start_delay=120）⇒ 不再等，别白拖",
          board_run.p4_wait_seconds(0.0, 200.0, 120.0) == 0.0)
    check("  刚好卡在边界（起栈 130s，start_delay=120，宽限 10）⇒ 归零，不许等成负数",
          board_run.p4_wait_seconds(0.0, 130.0, 120.0) == 0.0)
    check("  start_delay=0 ⇒ 只等宽限那一小段（不是 0，也不是 120）",
          board_run.p4_wait_seconds(0.0, 0.0, 0.0) == _G)
    check("★ --attach / 没起栈（t_launch=None）⇒ **0**，不许假装等过 —— "
          "没有「起栈时刻」就没有「这一段」可言",
          board_run.p4_wait_seconds(None, 0.0, 120.0) == 0.0)
    check("  start_delay 缺失（None）也当成 0，不是崩",
          board_run.p4_wait_seconds(0.0, 0.0, None) == _G)

    # 源码钉子：**P4 那一段真的要等**。纯函数测过 ≠ 接上了（本仓栽过这个）。
    _p4w = [n for n in ast.walk(TREE)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == "p4_wait_seconds"]
    check("源码钉子：`p4_wait_seconds` 恰好调一次，且第 3 个实参是"
          "`task.start_delay`（**写死常量**就等于永远不等，正是第 2 次那个坑）",
          len(_p4w) == 1 and len(_p4w[0].args) == 3
          and isinstance(_p4w[0].args[2], ast.Attribute)
          and _p4w[0].args[2].attr == "start_delay"
          and getattr(_p4w[0].args[2].value, "id", "") == "task",
          "%d 处调用" % len(_p4w))
    _p4i = [n for n in ast.walk(TREE)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == "Item" and n.args
            and isinstance(n.args[0], ast.Constant)
            and n.args[0].value == "P4.等启动延迟"]
    check("源码钉子：等出来的 `_note4` 真的进了 `P4.等启动延迟` 这一项"
          "（等了却不写进报告 ⇒ 下次没人知道它为什么慢）",
          len(_p4i) == 1 and len(_p4i[0].args) >= 3
          and isinstance(_p4i[0].args[2], ast.Name)
          and _p4i[0].args[2].id == "_note4",
          "%d 处" % len(_p4i))

    _m_p4 = load_mutant(_mutate(
        SRC, "    if t_launch is None:\n        return 0.0\n",
        "    if t_launch is None:\n        return 0.0\n    return 0.0  # MUTANT\n",
        "p4w"), "p4w")
    if _m_p4 is not None:
        _o = board_run.p4_wait_seconds(0.0, 10.0, 120.0)
        _n = _m_p4.p4_wait_seconds(0.0, 10.0, 120.0)
        check("变异：让 `p4_wait_seconds` **恒返回 0**（＝第 2 次那个「P4 抢跑」）"
              "⇒ 那条 `★` 断言**翻面**（改前 ≠ 改后）",
              _o > 0.0 and _n == 0.0, "改前 %r / 改后 %r" % (_o, _n))

    note = board_run.attach_log_missing_note("/tmp/nope.log")
    check("--attach 找不到日志：文案要**指出路径**，并把两种情形都写出来"
          "（「--log 指错了」与「**根本没有栈可附着**」）",
          "/tmp/nope.log" in note and "--log" in note and "没有栈可附着" in note, note)

    note = board_run.candidate_margin_note(4, 4)
    check("候选 4 = 槽位 4 ⇒ 点名「拿不到收工判据」并给出该摆几块",
          note is not None and "≥5 块" in note and "拿不到" in note, note)
    check("候选 5 > 槽位 4 ⇒ 不出声（只在该出声的时候出声）",
          board_run.candidate_margin_note(5, 4) is None)
    check("候选数读不到 ⇒ 不出声（不拿「不知道」当「不够」）",
          board_run.candidate_margin_note(None, 4) is None)

    # -----------------------------------------------------------------
    # 5b. 控制柜回复的**线上格式**（2026-10-06 实测）
    # -----------------------------------------------------------------
    # ⚠️ 下面三个值是**逐字原文**，不是纯值：UR 自加标签、programState 还缀
    #    程序名。旧判据拿原文与 'RUNNING' / 'PLAYING' 精确比 ⇒ P1 恒 FAIL、
    #    P3 恒等不到播放（R0 卡住的三条之一）。夹具曾写成猜的格式，
    #    所以这一段是本次唯一能抓到它的地方 —— 改它之前先读 board_lib.py 那节。
    section("[5b] 控制柜回复带标签/后缀：P1 与 P3 都得照样读懂")
    CAB_RUN = {"robotmode": "Robotmode: RUNNING",
               "programState": "STOPPED ur5ros.urp",
               "safetystatus": "Safetystatus: NORMAL"}
    st, det = board_run.judge_cabinet(CAB_RUN)
    check("P1.控制柜：原文 'Robotmode: RUNNING' ⇒ PASS（旧的精确比在这里恒 FAIL）",
          st == board_run.PASS, det)
    check("  PASS 文案印的是**读出来的**值，不含线上标签（'RUNNING | STOPPED | NORMAL'）",
          "RUNNING | STOPPED | NORMAL" in det and "Robotmode:" not in det, det)
    st2, det2 = board_run.judge_cabinet({**CAB_RUN,
                                         "robotmode": "Robotmode: POWER_OFF"})
    check("P1.控制柜：真没使能仍然拦（'Robotmode: POWER_OFF' ⇒ FAIL）"
          "—— 别为了消假红把闸门一起消掉",
          st2 == board_run.FAIL, det2)
    check("  FAIL 文案**同时**给出原文与读出的值（下一个人别再拿原文去比）",
          "Robotmode: POWER_OFF" in det2 and "POWER_OFF" in det2, det2)
    check("  连不上照旧 FAIL", board_run.judge_cabinet(None)[0] == board_run.FAIL)
    check("P3：播放时原文 'PLAYING ur5ros.urp' ⇒ is_playing 为真"
          "（旧的精确比在这里恒 False ⇒ 300s 超时退 2，人却按了播放）",
          board_run.is_playing({**CAB_RUN, "programState": "PLAYING ur5ros.urp"}))
    check("  固件若只回 'PLAYING'（无后缀）也认 —— 别把格式差异变成假红",
          board_run.is_playing({**CAB_RUN, "programState": "PLAYING"}))
    check("  STOPPED / PAUSED / 连不上 一律不算在播放",
          not board_run.is_playing(CAB_RUN)
          and not board_run.is_playing({**CAB_RUN,
                                        "programState": "PAUSED ur5ros.urp"})
          and not board_run.is_playing(None))
    check("  但**不许**放宽成子串：'NOT_PLAYING x.urp' 与 'STOPPED myPLAYING.urp' "
          "都不算在播放（这是它取代 'in' 判定的理由）",
          not board_run.is_playing({**CAB_RUN,
                                    "programState": "NOT_PLAYING x.urp"})
          and not board_run.is_playing({**CAB_RUN,
                                        "programState": "STOPPED myPLAYING.urp"}))

    # -----------------------------------------------------------------
    # 5c. P0 的 ROS 环境：启动器**自己 source**，判据是**行为**
    # -----------------------------------------------------------------
    section("[5c] P0.ROS 环境：自己 source；判据是「工作区 install 真在路径上」")
    ROSD = tempfile.mkdtemp(prefix="rosenv_", dir=tmpdir)
    sysdir = os.path.join(ROSD, "sys")
    wsins = os.path.join(ROSD, "ws", "install")
    os.makedirs(sysdir)
    os.makedirs(wsins)
    sys_setup = os.path.join(sysdir, "setup.bash")
    ws_setup = os.path.join(wsins, "setup.bash")
    with open(sys_setup, "w", encoding="utf-8") as fh:
        fh.write("export ROS_DISTRO=sysdistro\n"
                 "export AMENT_PREFIX_PATH=/opt/ros/sysdistro\n"
                 "export WHO=系统\n")
    with open(ws_setup, "w", encoding="utf-8") as fh:
        # ⚠️ 夹具必须照**现场实测**的形状写（2026-10-06，板上）：
        #    colcon 的**隔离式**安装放进 AMENT_PREFIX_PATH 的是
        #    `<ws>/install/<包名>` 一条条，**没有** `<ws>/install` 本身。
        #    第一版夹具写的是裸前缀，于是判据「install 目录在列表里」全绿、
        #    真机恒红 —— **夹具跟现场长得不一样，绿就是假的。**
        fh.write("export ROS_DISTRO=humble\n"
                 'export AMENT_PREFIX_PATH="%s:%s:$AMENT_PREFIX_PATH"\n'
                 "export WHO=工作区\n"
                 % (os.path.join(wsins, "yolo_bringup"),
                    os.path.join(wsins, "perception_3d")))

    check("两个 setup.bash **先系统后工作区**（反了会把工作区压在下面，且不报错）",
          board_run.ros_setup_files(distro="humble", install_setup="/w/install/setup.bash",
                                    system_setup="/s/setup.bash")
          == ["/s/setup.bash", "/w/install/setup.bash"])

    env, note = board_run.build_ros_env(base={"PATH": "/usr/bin:/bin"},
                                        system_setup=sys_setup,
                                        install_setup=ws_setup)
    check("build_ros_env 真的 source 了两个、拿回整份环境", env is not None, note)
    check("  **后 source 的说了算**（WHO=工作区）—— 这就是顺序的意义",
          bool(env) and env.get("WHO") == "工作区", str((env or {}).get("WHO")))
    pkgs = str((env or {}).get("AMENT_PREFIX_PATH", "")).split(os.pathsep)
    check("  工作区**逐包**前缀真的进了 AMENT_PREFIX_PATH（现场就是这个形状）",
          os.path.join(wsins, "yolo_bringup") in pkgs, str(pkgs))
    check("  而 `<ws>/install` 本身**不在**列表里 —— 判据不许要求它在（我第一版错在这）",
          wsins not in pkgs, str(pkgs))
    check("  说明里写清 source 了谁（报告里要看得见，不许藏）",
          "source 了" in note and ws_setup in note, note)
    check("★ 拿这份环境判 P0 ⇒ PASS（判据是行为：**这棵工作区**在贡献）",
          board_run.judge_ros_env(env or {}, True, ws_setup, sourced=note)[0]
          == board_run.PASS)
    check("  说明里带上了 env_note（「来源未记」不该出现）", "来源未记" not in note)

    # 合并式安装（裸 `<ws>/install`）也是合法布局 —— 两种都得认
    wsm = os.path.join(ROSD, "wsm", "install")
    os.makedirs(wsm)
    merged = os.path.join(wsm, "setup.bash")
    with open(merged, "w", encoding="utf-8") as fh:
        fh.write('export AMENT_PREFIX_PATH="%s:$AMENT_PREFIX_PATH"\n' % wsm)
    check("★ 合并式安装（裸 `<ws>/install`）也要 PASS —— 两种布局都合法",
          board_run.judge_ros_env({"ROS_DISTRO": "humble", "AMENT_PREFIX_PATH": wsm},
                                  True, merged)[0] == board_run.PASS)
    check("  但前缀撞名的邻居不许混进来（`/ws/installX` ≠ `/ws/install`）",
          board_run.judge_ros_env({"ROS_DISTRO": "humble",
                                   "AMENT_PREFIX_PATH": wsm + "X"}, True,
                                  merged)[0] == board_run.FAIL)

    # 板上实测的那个形状（2026-10-06）：只 source 了**别的**工作区
    ONLY_OTHER = {"ROS_DISTRO": "humble",
                  "AMENT_PREFIX_PATH": ("/opt/ros/humble"
                                        + os.pathsep + "/home/<用户>/yahboom_ws/install")}
    st, det = board_run.judge_ros_env(ONLY_OTHER, True, ws_setup)
    check("★ 只 source 了**别人的**工作区 ⇒ FAIL（旧的「非空」判据在这里是绿的）",
          st == board_run.FAIL, det)
    check("  且把「现在有什么」摆出来，点名别的那个工作区（症状离原因不远）",
          "yahboom_ws" in det, det)
    check("  而 P0 仍然拦得住「根本没 source」和「没有 ROS_DISTRO」",
          board_run.judge_ros_env({}, True, ws_setup)[0] == board_run.FAIL
          and board_run.judge_ros_env({"AMENT_PREFIX_PATH": "/x"}, True,
                                      ws_setup)[0] == board_run.FAIL)

    env2, note2 = board_run.build_ros_env(base={"PATH": "/usr/bin:/bin"},
                                          system_setup=sys_setup,
                                          install_setup=os.path.join(ROSD, "没有"))
    check("工作区 setup.bash 不在 ⇒ 说明点名它、并给出 colcon 的线索",
          "不在" in note2 and "colcon" in note2, note2)
    check("  此时 P0 判 FAIL，文案让人去 colcon build（不伪装成「缺包」）",
          board_run.judge_ros_env(env2 or {}, True,
                                  os.path.join(ROSD, "没有"))[0] == board_run.FAIL)

    # ---- P0.域号一致：判据 2026-10-06 改过（用户拍板「按启动器方便的来」）----
    # 旧判据「没设 ⇒ FAIL」拦的是一个**启动器自己就会修**的条件（P2 会给它起的每
    # 个子进程注入站点值），代价是每开一个终端都要记得 export。现在只拦**真冲突**。
    class _Site:                 # 这里只用得上这一个字段
        ros_domain_id = 99

    st, det = board_run.judge_domain({}, _Site)
    check("★ 环境里**没设** ROS_DOMAIN_ID ⇒ PASS（栈仍会被起在站点那个域上）",
          st == board_run.PASS, det)
    check("  但**必须**说清值从哪来、以及你自己另开的终端要做什么（不是只吐个绿）",
          "启动器" in det and "export ROS_DOMAIN_ID=99" in det, det)
    check("  空串与没设同义（`ROS_DOMAIN_ID=` 这种写过头的也放过）",
          board_run.judge_domain({"ROS_DOMAIN_ID": ""}, _Site)[0] == board_run.PASS)
    st, det = board_run.judge_domain({"ROS_DOMAIN_ID": "0"}, _Site)
    check("★ 你自己的 shell 在**别的**域 ⇒ 仍然 FAIL（这才是那个「栈起着、话题空」的坑）",
          st == board_run.FAIL, det)
    check("  文案给出两条出路（export 成站点值 / 改站点）并点名症状",
          "export ROS_DOMAIN_ID=99" in det and "0" in det and "话题" in det, det)
    check("  一致就 PASS，且容忍手抄来的空格",
          board_run.judge_domain({"ROS_DOMAIN_ID": " 99 "}, _Site)[0] == board_run.PASS)

    # -----------------------------------------------------------------
    # 5d. 残状态：起栈**之前**就在 PLAYING —— 排在 [5c] 之后是为了不打乱台账
    #     对 `[5c]`＝P0 ROS 环境的引用（RESTRUCTURE_PLAN 第 14378 行）。
    # -----------------------------------------------------------------
    section("[5d] 残状态：起栈**之前**就在 PLAYING ⇒ 这条 PASS 不算「人按了」"
            "（用户裁决 b：说清 + 放行，不是拒）")
    _resid = board_run.residual_playing_note({**CAB_RUN,
                                              "programState": "PLAYING ur5ros.urp"})
    check("★ 起栈前已是 PLAYING ⇒ 明说**不算「人在环走通了」**，并点名是残状态"
          "（第 2 次 R0 实测 '0s 后 PLAYING'，读起来却像人按了）",
          "不算" in _resid and "残状态" in _resid and "STOP" in _resid, _resid)
    check("  且指出**收栈收的是这套栈、不是控制柜里的程序**（下一步才知道该干什么）",
          "这套栈" in _resid and "控制柜" in _resid, _resid)
    check("★ 起栈前 STOPPED ⇒ **不追加任何话**（正常的一轮别被噪声淹没）",
          board_run.residual_playing_note(CAB_RUN) == "",
          repr(board_run.residual_playing_note(CAB_RUN)))
    check("  Dashboard 没答话（None）⇒ 明写**未判**，不许默认成「人按了」也不许"
          "默认成「残状态」",
          "未判" in board_run.residual_playing_note(None),
          board_run.residual_playing_note(None))
    check("  PAUSED 不算 PLAYING（沿 is_playing 的那一条，不另立一套判据）",
          board_run.residual_playing_note({**CAB_RUN,
                                           "programState": "PAUSED ur5ros.urp"}) == "")

    # 源码钉子：判据只能是**起栈之前**那个值。若哪天有人把 `waited` 塞进来
    # （「等得久才算人按的」），就把「手快」误判成「残状态」—— 与缺陷丙同一个形状。
    _rp = [n for n in ast.walk(TREE)
           if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
           and n.func.id == "residual_playing_note"]
    check("源码钉子：`residual_playing_note` 取的是 `facts.get(\"cabinet\")`"
          "（＝**P1、起栈之前**读的那一份），且**只传这一个**实参 —— "
          "塞进 `waited` 就等于拿「按得快慢」当判据",
          len(_rp) == 1 and len(_rp[0].args) == 1
          and isinstance(_rp[0].args[0], ast.Call)
          and getattr(_rp[0].args[0].func, "attr", "") == "get"
          and isinstance(_rp[0].args[0].args[0], ast.Constant)
          and _rp[0].args[0].args[0].value == "cabinet",
          "%d 处调用" % len(_rp))

    _m_rp = load_mutant(_mutate(
        SRC, "    if board_lib.is_playing(cabinet):\n",
        "    if False:\n", "rp"), "rp")
    if _m_rp is not None:
        check("变异：把残状态这一条**掐掉**（恒不追加）⇒ 那条 `★` 断言**翻面**，"
              "报告又变回「0s 后 PLAYING」那种读起来像人按了的 PASS",
              board_run.residual_playing_note(
                  {**CAB_RUN, "programState": "PLAYING ur5ros.urp"}) != ""
              and _m_rp.residual_playing_note(
                  {**CAB_RUN, "programState": "PLAYING ur5ros.urp"}) == "")

    # -----------------------------------------------------------------
    # 5e. 上面那根判据的**机器可读**那一面（2026-10-07 R0 阶梯第 2 次逼出来的）
    # -----------------------------------------------------------------
    section("[5e] 空洞绿：`PASS` 有两种含义 ⇒ 机器也要分得清（不只写在散文里）")

    _pl = {"programState": "PLAYING ur5ros.urp"}
    _pa = {"programState": "PAUSED ur5ros.urp"}
    check("★ 起栈前 PLAYING ⇒ `True`（确认**空洞**）",
          board_run.playing_is_vacuous({**CAB_RUN, **_pl}) is True,
          repr(board_run.playing_is_vacuous({**CAB_RUN, **_pl})))
    check("  起栈前 STOPPED ⇒ `False`（确认是**人**按的）",
          board_run.playing_is_vacuous(CAB_RUN) is False)
    check("  ★ Dashboard 没答话 ⇒ `None`（**判不了**）—— 不许当 `False`："
          "把「没读到」算成「确认是人按的」正是本仓最贵的那类误读",
          board_run.playing_is_vacuous(None) is None,
          repr(board_run.playing_is_vacuous(None)))
    check("  PAUSED 不算（与 `is_playing` 同一条，不另立判据）",
          board_run.playing_is_vacuous({**CAB_RUN, **_pa}) is False)

    # ★ 两面必须同源：散文说「空洞」而 JSON 说「不是」的话，就没有任何东西拦得住。
    check("★ 与 `residual_playing_note` **逐档一致**（同一根判据的两个出口，"
          "不许一个说空洞、另一个说没事）",
          all((board_run.residual_playing_note(c) != "")
              == (board_run.playing_is_vacuous(c) is not False)
              for c in ({**CAB_RUN, **_pl}, CAB_RUN, None, {**CAB_RUN, **_pa})))

    check("  `count_vacuous` 把「确认空洞」与「未判」**分开数**"
          "（合成一个数会重演「同一个标签盖两种状态」）",
          board_run.count_vacuous([
              I("a", board_run.PASS, "d", vacuous=True),
              I("b", board_run.PASS, "d", vacuous=None),
              I("c", board_run.PASS, "d"),
              I("e", board_run.FAIL, "d", vacuous=True),
          ]) == (1, 1),
          repr(board_run.count_vacuous([
              I("a", board_run.PASS, "d", vacuous=True),
              I("b", board_run.PASS, "d", vacuous=None)])))

    _vline = "\n".join(board_run.render_item(
        I("P3.示教器 PLAYING", board_run.PASS, "0s 后 PLAYING", vacuous=True)))
    _uline = "\n".join(board_run.render_item(
        I("P3.示教器 PLAYING", board_run.PASS, "PLAYING", vacuous=None)))
    _nline = "\n".join(board_run.render_item(
        I("P4.camera", board_run.PASS, "30.0 Hz")))
    check("★ 空洞绿在**状态行底下**补一行「空洞绿…没有证明人在环走通了」",
          "空洞绿" in _vline and "人在环" in _vline)
    check("  未判那一档也补一行，且**明写不许读成验过了**",
          "未判" in _uline and "不许" in _uline)
    check("  与人在环无关的 PASS（P4.camera）**一个字都不多**"
          "（正常的一轮别被噪声淹没）",
          "空洞" not in _nline and "未判" not in _nline)

    _sum_v = "\n".join(board_run.render_summary(
        [I("a", board_run.PASS, "d", vacuous=True),
         I("b", board_run.PASS, "d"),
         I("c", board_run.FAIL, "d")], 2))
    check("★ 总表**必须**单独说一句空洞绿 —— 否则三连跑会被读成"
          "「人在环走通 3 次」，而真相是 1 次",
          "2 项 PASS" in _sum_v and "1 项空洞" in _sum_v, _sum_v)
    check("  没有空洞项时**不多说**（`PASS 1 / FAIL 0 / SKIP 0` 就完了）",
          "空洞" not in "\n".join(board_run.render_summary(
              [I("a", board_run.PASS, "d")], 2)))

    check("★ `Item.as_dict()` 只在**不是 False** 时才写 `vacuous`"
          "（False 是默认值 ⇒ 写满屏是噪声；但 `None`（未判）**必须**落进 JSON）",
          "vacuous" not in I("a", board_run.PASS, "d").as_dict()
          and I("a", board_run.PASS, "d", vacuous=True).as_dict()["vacuous"] is True
          and I("a", board_run.PASS, "d", vacuous=None).as_dict()["vacuous"] is None)

    check("源码钉子：P3 那条 Item 传的是 `vacuous=playing_is_vacuous(...)`，"
          "且取的是 `facts.get(\"cabinet\")`（与散文那支**同源**）",
          "vacuous=playing_is_vacuous(" in SRC
          and "playing_is_vacuous(facts.get(\"cabinet\"))" in SRC)

    _m_pv = load_mutant(_mutate(
        SRC, "    return bool(board_lib.is_playing(cabinet))\n",
        "    return False\n", "pv"), "pv")
    if _m_pv is not None:
        check("变异：把 `playing_is_vacuous` 的残状态那一支**掐掉**（恒 False）"
              "⇒ 第 2 次 R0 那种空洞绿又变回「看起来是人按的」",
              board_run.playing_is_vacuous({**CAB_RUN, **_pl}) is True
              and _m_pv.playing_is_vacuous({**CAB_RUN, **_pl}) is False)

    # 键改名（2026-10-07）：旧名读起来像「P3/P4 阶段的判决」，其实是子进程退出码。
    _rec_ast = [n for n in ast.walk(TREE)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and n.value in ("gate_p3_exit", "gate_p4_exit",
                                "controllers_gate_exit", "p4_gate_exit")]
    _seen = {n.value for n in _rec_ast}
    check("★ 报告键已改名：`gate_p3_exit`/`gate_p4_exit` 在源码里**不再出现**"
          "（它们读起来像阶段判决，实为 `board_gate.py` 子进程的退出码）",
          _seen == {"controllers_gate_exit", "p4_gate_exit"}, repr(sorted(_seen)))

    # -----------------------------------------------------------------
    # 5f. 空洞绿在**要跑运动**的那一轮是**动作问题**，不只是记录问题
    #     （2026-10-07 用户裁定：拦住并引导重按）
    # -----------------------------------------------------------------
    section("[5f] 交棒前复核「人在环」：空洞绿在 R0 **不拦**、在运动轮 **必拦**")

    _P3_V = I("P3.示教器 PLAYING", board_run.PASS, "", vacuous=True)
    _P3_F = I("P3.示教器 PLAYING", board_run.PASS, "", vacuous=False)
    _P3_N = I("P3.示教器 PLAYING", board_run.PASS, "", vacuous=None)

    check("★ 实绿（起栈前不是 PLAYING ⇒ 人真按了）⇒ **不用**重按",
          board_run.needs_repress([_P3_F]) is False)
    check("★ 空洞绿 ⇒ **要**重按（人可能压根没按，而臂会动）",
          board_run.needs_repress([_P3_V]) is True)
    check("  **判不了**（`None`）也 ⇒ **要**（fail-closed）",
          board_run.needs_repress([_P3_N]) is True)
    check("  那一项**压根找不到** ⇒ 也**要**（判不了就按「要」办，fail-closed）",
          board_run.needs_repress([]) is True)
    check("  与 `playing_is_vacuous` **逐档同源**（一个说空洞、另一个却放行"
          "＝没有任何东西拦得住）",
          all(board_run.needs_repress([I("P3.示教器 PLAYING", board_run.PASS, "",
                                         vacuous=board_run.playing_is_vacuous(c))])
              == (board_run.playing_is_vacuous(c) is not False)
              for c in ({**CAB_RUN, **_pl}, CAB_RUN, None)))

    # ★ 关键边界：**不许下沉进 `decide()`**。`decide()` 是全流程唯一的退出码
    #   出口（P0–P5 都走它）——把空洞绿放进去，**R0（无运动）**也会被拦在 P3，
    #   白丢掉 P4 的证据。空洞绿该不该拦，取决于**下一步会不会动**。
    check("★ `decide()` **不**看 vacuous：空洞绿在 R0（无运动）照旧不拦",
          board_run.decide([_P3_V])[0] == 0,
          "decide=%r" % (board_run.decide([_P3_V]),))
    check("  源码级钉子：`decide()` 的函数体里**不出现** vacuous"
          "（想搬进去之前，先想清楚 R0 会不会被一起拦掉）",
          "vacuous" not in func_code("decide"))
    check("★ P5 确实按 `needs_repress` 拦，且复核项挡在交棒之前（源码级）",
          '"P5.人在环复核"' in SRC and "needs_repress(counted(report, S3))" in SRC)

    # ---- 两步握手：**真跑**（注入假 dashboard ＋ 假时钟 ＋ 空 sleep）----
    _CLK = [0.0]

    def _tick():
        _CLK[0] += 1.0
        return _CLK[0]

    def _scripted(seq):
        it = iter(seq)
        cur = [seq[0]]

        def f(_ip):
            try:
                cur[0] = next(it)
            except StopIteration:
                pass
            return cur[0]
        return f

    _STOPPED = dict(CAB_RUN)
    _PLAYING = {**CAB_RUN, **_pl}

    def _loop(seq, timeout=6.0):
        _CLK[0] = 0.0
        return board_run.ensure_in_the_loop(
            "x", timeout, poll=1.0, dash=_scripted(seq), clock=_tick,
            sleep=lambda _s: None, prompt=lambda *_a: None)

    _ok, _why = _loop([_PLAYING, _STOPPED, _PLAYING])
    check("★★ 人真按了（PLAYING → STOPPED → PLAYING **都看到**）⇒ 放行",
          _ok is True, _why)

    _ok, _why = _loop([_PLAYING])
    check("★★ 它**一直** PLAYING（没看见离开）⇒ **不放行**"
          "（这正是「残 PLAYING」那一档；只看一句「按一下」是判不出来的）",
          _ok is False and "没看到它离开" in _why, _why)

    _ok, _why = _loop([_PLAYING, _STOPPED])
    check("  按了停止却没等到播放 ⇒ **不放行**",
          _ok is False and "没等到 PLAYING" in _why, _why)

    _ok, _why = _loop([_STOPPED, _PLAYING])
    check("  本来就没在跑 ⇒ 离开那一步立刻成立，只等播放 ⇒ 放行",
          _ok is True, _why)

    _m_nr = load_mutant(_mutate(
        SRC, "            return getattr(i, \"vacuous\", False) is not False\n",
        "            return getattr(i, \"vacuous\", False) is True\n", "nr"), "nr")
    if _m_nr is not None:
        check("★ 变异：把「判不了」当成实绿 ⇒ 那一档**翻面**"
              "（不翻面就说明本条断言恒真、等于没写）",
              _m_nr.needs_repress([_P3_N]) is False
              and board_run.needs_repress([_P3_N]) is True)

    # -----------------------------------------------------------------
    # 5g. 两种跑法：AI 驱动 / 人手动。**把 AI 那条路堵死是缺陷，不是谨慎**
    #     （2026-10-07 用户要求：「要能够支持 ai 自己能运行，也能支持我手动启动」）
    # -----------------------------------------------------------------
    section("[5g] 交棒确认的两条路：有终端敲 `yes`，没终端用示教器上的按压")

    check("★ 有交互终端 ⇒ `keyboard`（人自己起的那一轮，原样敲 `yes`）",
          board_run.handoff_mode(lambda: True) == "keyboard")
    check("★★ **没有**交互终端 ⇒ `pendant`（AI 驱动的那一轮）—— "
          "旧版在这里**直接拒绝**，等于把 AI 驱动的跑法整个堵死",
          board_run.handoff_mode(lambda: False) == "pendant")

    def _boom(*_a, **_k):
        raise AssertionError("pendant 模式**不该**向人要键盘输入")

    _ok_p, _why_p = board_run.confirm_handoff("pendant", prompt_fn=_boom)
    check("★★ pendant 模式**不碰 stdin**（给它一个「一调用就炸」的 prompt_fn）"
          "⇒ 照样放行，且说明里讲清「证明改由示教器承担」",
          _ok_p is True and "示教器" in _why_p, _why_p)

    _ok_k1 = board_run.confirm_handoff("keyboard", prompt_fn=lambda _p: "yes")[0]
    _ok_k2 = board_run.confirm_handoff("keyboard", prompt_fn=lambda _p: "no")[0]
    check("  keyboard 模式仍然**必须**敲 `yes`：敲 `yes` 放行、敲别的**不**放行",
          _ok_k1 is True and _ok_k2 is False)

    check("★ 源码级钉子：P5 按 `handoff_mode(isatty)` 选路，且报告里**记下"
          "这一次是哪种**（事后要分得清 AI 跑的与人手动跑的）",
          "handoff_mode(isatty)" in SRC and '"P5.交棒确认"' in SRC)

    # ★ 变异：退回「永远要键盘」—— 那正是用户指出的缺陷本身。
    #   必须看到 **AI 驱动那一档由「交棒」翻成「不交棒」**。
    _m_hm = load_mutant(_mutate(
        SRC, '    return "keyboard" if isatty() else "pendant"\n',
        '    return "keyboard"\n', "hm"), "hm")
    if _m_hm is not None:
        _mut_ok = _m_hm.confirm_handoff(_m_hm.handoff_mode(lambda: False),
                                        prompt_fn=lambda _p: "yes",
                                        isatty=lambda: False)[0]
        _real_ok = board_run.confirm_handoff(
            board_run.handoff_mode(lambda: False), prompt_fn=_boom)[0]
        check("★ 变异：退回「永远要键盘」⇒ **AI 驱动那一档翻面**"
              "（由「已确认」变成「不交棒」；不翻面说明 5g 断言恒真、等于没写）",
              _m_hm.handoff_mode(lambda: False) == "keyboard"
              and _mut_ok is False and _real_ok is True,
              "改后 %r / 改前 %r" % (_mut_ok, _real_ok))

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

    # ---- `--pick-zone`：**只在规划器不起的档**才递 ----------------------------
    # 出处：2026-10-06 真机 R0 第 1 次，`P4.targets` **恒 SKIP**（「拿不到 pick_zone
    # （规划器不在图上，也没给 --pick-zone）」），而启动器手里**有**这个值
    # （P0「参数集」就把它印出来了）。而 `a1-noplan.yaml` 开头写明 R0 存在的意义
    # 正是「/targets_3d 候选数仍然验得到」⇒ 那一档的 P4 有一项**结构性验不了**。
    check("`planner_absent`：allow_skip 同时含 telemetry 与 params ⇔ 本档没有规划器",
          board_run.planner_absent(["telemetry", "params"]) is True
          and board_run.planner_absent(["telemetry"]) is False
          and board_run.planner_absent([]) is False
          and board_run.planner_absent(None) is False)
    _pz = [0.4, 0.6, 0.0, 0.45, 0.0, 0.0485]
    check("  规划器不起 ⇒ 递出 6 个数（不递就恒 SKIP）",
          board_run.pick_zone_arg(["telemetry", "params"],
                                  {"pick_zone": _pz}) == _pz)
    check("★ 规划器活着 ⇒ **不递** —— 递了会把「活读运行中的规划器」降级成"
          "静态 yaml（`--pick-zone` 在 board_gate 里优先于活读）",
          board_run.pick_zone_arg(["telemetry"], {"pick_zone": _pz}) is None)
    check("  pick_zone 读不到（文件里没这一项）⇒ 不递，让闸门自己报 SKIP",
          board_run.pick_zone_arg(["telemetry", "params"], {}) is None)
    check("  形状不是 6 个数 ⇒ 不递（不拿半份去撞闸门的自检）",
          board_run.pick_zone_arg(["telemetry", "params"],
                                  {"pick_zone": [1, 2, 3]}) is None)
    check("  launch 命令行给的值**覆盖** yaml（与 P0「参数集」同一套算法）",
          board_run.pick_zone_arg(["telemetry", "params"],
                                  {"pick_zone": [9.0] * 6},
                                  {"pick_zone": _pz}) == _pz)
    _argv_pz = board_run.gate_argv("/tmp/z.json", pick_zone=_pz)
    check("  `gate_argv` 拼成 `--pick-zone a,b,c,d,e,f`（闸门要的就是这个形状）",
          _argv_pz[_argv_pz.index("--pick-zone") + 1]
          == "0.4,0.6,0.0,0.45,0.0,0.0485", str(_argv_pz))
    check("  不递时命令行里**没有** --pick-zone（不是给个空串）",
          "--pick-zone" not in board_run.gate_argv("/tmp/z.json"))

    # 源码钉子：**接线那一行**也要钉。纯函数测过 ≠ 接上了 —— P4 的调用点漏掉
    # `pick_zone=` 的话，上面全绿而真机照旧恒 SKIP（正是这一轮修的那个病）。
    _p4c = [n for n in ast.walk(TREE)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == "gate_argv"
            and any(k.arg == "slots_total" for k in n.keywords)]
    check("源码钉子：P4 那次 `gate_argv(...)` 真的传了 `pick_zone=pick_zone_arg(...)`",
          len(_p4c) == 1
          and any(k.arg == "pick_zone" and isinstance(k.value, ast.Call)
                  and getattr(k.value.func, "id", "") == "pick_zone_arg"
                  for k in _p4c[0].keywords),
          "%d 处带 slots_total 的 gate_argv 调用" % len(_p4c))

    _m_pz = load_mutant(_mutate(
        SRC, '    if not planner_absent(allow_skip):\n        return None',
        "    if False:\n        return None", "pz"), "pz")
    if _m_pz is not None:
        check("削掉「只在规划器不起的档才递」⇒ R1 的活读判据被降级成静态值",
              _m_pz.pick_zone_arg(["telemetry"], {"pick_zone": _pz}) == _pz
              and board_run.pick_zone_arg(["telemetry"], {"pick_zone": _pz}) is None,
              "mut=%r real=%r" % (_m_pz.pick_zone_arg(["telemetry"], {"pick_zone": _pz}),
                                  board_run.pick_zone_arg(["telemetry"],
                                                          {"pick_zone": _pz})))

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

    # 两侧都钉。只钉一侧的话，「把缺席改回 [SKIP]」或「把真 SKIP 改成 [缺席]」
    # 有一半改法照样绿 —— 而这两种改法都会让结算里的两个数自相矛盾。
    skip_line = "\n".join(board_run.render_item(
        I("P1.串口", board_run.SKIP, "没查")))
    check("  缺席与真 SKIP **标签必须不同**：缺席 = [缺席]，真 SKIP = [SKIP]"
          "（同一个标签盖两种状态，结算就会写「SKIP 0」而满屏 [SKIP]）",
          "[缺席]" in line and "[SKIP]" not in line
          and "[SKIP]" in skip_line and "[缺席]" not in skip_line)

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
    # ⚠️ 加一对槽 = 在**扁平**表尾再接 **两个**数（不是再接一个 [x,y]）。
    #    这里原来接的是 `[0.44, -0.24],`，是**嵌套**夹具时代的写法；夹具改成扁平之后
    #    那个串匹配不上 ⇒ `.replace` 原样返回 ⇒ p5 与 4 槽的夹具逐字节相同，
    #    「5 个槽」的用例其实在测「4 个槽」⇒ **恒假**（且看起来还是绿的？不：它会红，
    #    但红的是「对不上」这条断言 —— 换句话说它红了，只是红的理由不是它声称的那个）。
    slots5 = open(planner_path, encoding="utf-8").read().replace(
        "[0.44, 0.24, 0.60, 0.24, 0.76, 0.24, 0.44, -0.24]",
        "[0.44, 0.24, 0.60, 0.24, 0.76, 0.24, 0.44, -0.24, 0.60, -0.24]")
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

    # ★ 2026-10-06：`place_slots` 是**扁平**的 [x0,y0, x1,y1, …]
    #   （`planner_node.py:203` 逐字写着；:213-215 就是 `[0::2]`/`[1::2]` 解回来的）。
    #   所以**槽位数是 len//2**。这条曾经在夹具上恒对、在真机上恒错 —— 见 [11]。
    n, why = board_run.slot_count([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
    check("★ 扁平 8 个数 ⇒ **4** 个槽（不是 8 —— 这是本轮真机那次误报的根）",
          n == 4 and why == "", "n=%s why=%s" % (n, why))
    n, why = board_run.slot_count([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    check("  扁平**奇数**个数 ⇒ None，且说明里点名「奇数」（节点自己也会因此抛）",
          n is None and "奇数" in why, "n=%s why=%s" % (n, why))
    n, why = board_run.slot_count([[0.1, 0.2], [0.3, 0.4]])
    check("  **嵌套** [[x,y], …] ⇒ None，且说明里点名「扁平」（真机节点明确不收这个形状）",
          n is None and "扁平" in why and "嵌套" in why, "n=%s why=%s" % (n, why))
    n, why = board_run.slot_count([])
    check("  空表 / 不是列表 ⇒ None（「没验到」与「验过是 0 个」是两回事）",
          board_run.slot_count([])[0] is None
          and board_run.slot_count(None)[0] is None
          and board_run.slot_count("0.44")[0] is None)
    check("  纯函数：不读文件、不碰环境（签名只有 vals）",
          board_run.slot_count.__code__.co_argcount == 1)
    # 同目录候选：现场拿不到鼠标时，报告得直接说「我该指哪一份」
    sib = dict(board_run.sibling_slots(os.path.join(tmpdir, "planner5.yaml")))
    check("  sibling_slots 列出**同目录**的其它 yaml 及各自的槽数，且不含自己",
          sib.get("planner.yaml") == 4 and "planner5.yaml" not in sib, str(sib)[:160])
    check("  sibling_slots 用的是同一个 slot_count（扁平 8 个值 ⇒ 报 4，不是 8）",
          sib.get("planner.yaml") != 8, str(sib)[:160])

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

    # 这条变异**就是当初那个洞**：`--log` 那一半没人判 ⇒ 栈日志直接往公开仓里写。
    m = load_mutant(_mutate(SRC, '    if log_path and path_in_repo(log_path, repo):',
                            "    if False:", "outpath"), "outpath")
    mut_o = m.judge_output_paths(os.path.join(REPO, "scripts", "x.log"), None)[0]
    real_o = board_run.judge_output_paths(
        os.path.join(REPO, "scripts", "x.log"), None)[0]
    check("削掉「`--log` 不许进仓库」⇒ 栈日志（含 IP/序列号）往公开仓里写",
          mut_o == board_run.PASS and real_o == board_run.FAIL,
          "改后 %s / 改前 %s" % (mut_o, real_o))

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

    # 变异：把「无运动档 ⇒ 时间推断不适用」整个削掉 —— run 6 那条假警报必须原样回来。
    m = load_mutant(_mutate(SRC, "    if not planner_alive:",
                            "    if False:", "palive"), "palive")
    mut_moved, mut_note = m.judge_handoff_window(0.0, 273.0, 120.0,
                                                 planner_alive=False)
    real_moved, _ = board_run.judge_handoff_window(0.0, 273.0, 120.0,
                                                   planner_alive=False)
    check("变异：削掉「规划器不起 ⇒ 时间推断不适用」⇒ R0 档那条假警报**原样回来**"
          "（★ 断言吃劲：改前 ≠ 改后）",
          bool(mut_moved) and real_moved is False
          and "可能已经启动" in mut_note,
          "变异后 moved=%s；改前 moved=%s" % (mut_moved, real_moved))


    # =================================================================
    # 11. 整跑
    # =================================================================
    section("[11] 整跑：离线**必须**退 2（绝不能退 0）；--dry-run 不跑任何子进程")

    # PATH 洗成没有 ros2 的样子：这样「离线整跑退 2」是**确定**的，而不是取决于
    # 跑测试的人有没有 source 过 ROS（那个变量一进来，这条断言就变成碰运气）。
    #
    # ⚠️ 2026-10-06 起启动器**自己 source** `/opt/ros/humble/setup.bash`（见
    #    `build_ros_env`），所以洗 PATH **再也造不出**「PATH 里没有 ros2」——
    #    它会把 ros2 找回来。
    #
    # ⚠️⚠️ 而**红的理由必须与机器无关**（2026-10-06 板上抓到的第二次）：
    #    这里原来断言红的是「缺本机工作区的 install ⇒ 去 colcon build」——
    #    可**板上那份检出是编过的**，同一份代码在板上就不红了，自检当场失败。
    #    **拿环境当行为**，与 [5b] 那个「夹具猜线上格式」是同一个病。
    #    第一次改成钉「域号没设」，但域号判据随后也被放开了（没设 ⇒ PASS）……
    #    两次都是**拿一个会变的现状当钉子**。
    #    ⇒ 现在钉的这条**由夹具自己造**：把站点文件写坏（少 `topcam.ttl_days`）。
    #      它在**任何**机器上都红、说的是同一件事，且钉住的是那条因果
    #      「P0 不过 ⇒ 栈不起」，这正是本档要证的。
    bad_site = os.path.join(tmpdir, "site_broken.yaml")
    with open(bad_site, "w", encoding="utf-8") as fh:
        fh.write(SITE_YAML.format(planner=planner_path,
                                  today=datetime.date.today().isoformat())
                 .replace("  ttl_days: 1\n", ""))

    no_ros = os.pathsep.join(
        p for p in clean_env().get("PATH", "").split(os.pathsep)
        if p and not os.path.exists(os.path.join(p, "ros2")))

    r = run_cli("--task", "a1", "--site", bad_site,
                "--log", os.path.join(tmpdir, "a1.log"),
                "--json-report", os.path.join(REPO, "scripts", "_never_write.json"),
                env={"PATH": no_ros})
    check("离线整跑退 2（**绝不能退 0**：fail-closed）", r.returncode == 2,
          "rc=%s" % r.returncode)
    check("  红的是**夹具自己造**的那条（站点少写一个键）—— 与跑测试的机器无关",
          "[FAIL] P0.站点配置" in r.stdout and "ttl_days" in r.stdout)
    check("  每条红都说得出名字（`[FAIL] P0.<项>`），没有无名红",
          len(re.findall(r"\[FAIL\] P0\.\S", r.stdout)) >= 1)
    check("  栈**没有起**（闸门没过就不起栈）",
          "栈**没有起**" in r.stdout and "起栈中" not in r.stdout)
    check("  P1 一项都没跑（P0 就没过去）", "[PASS] P1." not in r.stdout)
    check("  --json-report 指向仓库内 ⇒ 拒写，且**真的没写**",
          "指向仓库内" in r.stdout
          and not os.path.exists(os.path.join(REPO, "scripts", "_never_write.json")))
    check("  报告末尾印出三档退出码的语义（0/1/2 各是什么）",
          mentions(r.stdout, "1 = ", "2 = ", "交棒"))

    # ---- 红线 3：`--log` 指进仓库 ⇒ **P0 就退 2，一个字节都不写** ------------
    # 这是 2026-10-06 抓到的洞的钉子：此前 `--log` **没有任何守卫**，起栈那一刻
    # `open(path,"w")` 就把栈日志写进公开仓 —— 里面有控制箱 IP、相机序列号、
    # 串口 by-id。而 `--json-report` 那句拒绝是在**跑完之后**才打印、且不翻退出码。
    inrepo_log = os.path.join(REPO, "scripts", "_never_write_log.txt")
    r = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                "--log", inrepo_log, env={"PATH": no_ros})
    check("★ `--log` 指进仓库 ⇒ 退 2（P0 就拦，栈压根不起）",
          r.returncode == 2, "rc=%s" % r.returncode)
    check("  红的那一条叫 `P0.输出路径`，并且**真的一个字节都没写**",
          "[FAIL] P0.输出路径" in r.stdout and not os.path.exists(inrepo_log))
    if os.path.exists(inrepo_log):
        os.remove(inrepo_log)     # 判据坏了的时候，别把垃圾留在仓库里

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
    # ⚠️ 2026-10-06 修：`P2.槽位数` 曾经在 --dry-run 里**声明缺席**，理由是
    #    「要读 planner_params_file，这一档留给真跑」—— **理由本身是错的**，
    #    它只是读一个文件。代价很贵：任务描述与 planner_params_file 的槽位数不一致，
    #    本来只有**上板起栈**那一刻才暴露（而槽位数正是判分用的量）。本轮就是这么
    #    在板上被抓到的（站点 8 槽 / a1-noplan 期望 4 槽）。这两条钉子钉住它之后
    #    不再退回去。
    check("★ --dry-run **也**跑槽位数检查（纯文件读，不跑子进程 ⇒ 在合同之内）",
          "[PASS] P2.槽位数" in r.stdout and "留给真跑" not in r.stdout)
    # ⚠️ 判据要读**那一行**渲染成什么，不是数「声明缺席」这个词出现了几次
    #    （2026-10-06 我自己先写错一版：那个词在 20 多条别的断言里也出现 ⇒ 恒假）。
    check("  且它**不**被标成缺席行，而声明集与起栈**仍然**是缺席行",
          "[缺席] P2.槽位数" not in r.stdout
          and "[缺席] P2.声明集" in r.stdout
          and "[缺席] P2.起栈" in r.stdout)

    # 站点那份 planner 参数文件槽位数对不上 ⇒ **桌面上的 --dry-run 就该红**
    bad_planner = os.path.join(tmpdir, "planner_slots8.yaml")
    with open(bad_planner, "w", encoding="utf-8") as fh:
        fh.write(PLANNER_YAML.replace(
            "    place_slots: [0.44, 0.24, 0.60, 0.24, 0.76, 0.24, 0.44, -0.24]\n",
            "    place_slots: [0.44, 0.24, 0.60, 0.24, 0.76, 0.24, 0.44, -0.24,\n"
            "                  0.60, -0.24, 0.76, -0.24, 0.44, 0.0, 0.60, 0.0]\n"))
    slots_site = os.path.join(tmpdir, "site_slots8.yaml")
    with open(slots_site, "w", encoding="utf-8") as fh:
        fh.write(SITE_YAML.format(planner=bad_planner,
                                  today=datetime.date.today().isoformat()))
    r2 = run_cli("--task", "a1", "--site", slots_site, "--dry-run",
                 env={"PATH": no_ros})
    check("★ 槽位数对不上 ⇒ **--dry-run 就退 2**（不必上板才暴露）",
          r2.returncode == 2 and "[FAIL] P2.槽位数" in r2.stdout,
          "rc=%s" % r2.returncode)
    check("  文案让人**先看台面**、并给出两个方向（改任务 / 站点指错文件了）",
          "先看台面再改" in r2.stdout and "expect.slots_total" in r2.stdout
          and "指错文件" in r2.stdout)
    check("  且把**同目录下别的** planner 参数文件连同槽位数摆出来"
          "（「我该指哪一份」才是现场真正要回答的）",
          "        · planner.yaml → 4 个槽" in r2.stdout
          and "        · planner_slots8.yaml" not in r2.stdout)
    check("  栈**没有起**（dry-run 本来就不起，但失败路径也不许漏）",
          "起栈中" not in r2.stdout)

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
    # ⚠️ 控制柜那三个值**逐字照抄线上原文**（2026-10-06 实测，29999 只读）。
    #    别「顺手整理」成纯值 —— 那正是本文件里曾经发生过的事故：
    #    夹具写的是**猜的**格式 {'robotmode': 'RUNNING'}，猜错了，
    #    于是这里 1294 行全绿，而真机上 P1.控制柜恒 FAIL、P3 恒等不到播放。
    FACTS_GREEN = {
        "leftovers": [], "port": "/dev/serial/by-id/usb-<占位>-if00-port0",
        "port_busy": False, "gripper": (1, "ok"),
        "cabinet": {"robotmode": "Robotmode: RUNNING",
                    "programState": "STOPPED ur5ros.urp",
                    "safetystatus": "Safetystatus: NORMAL"},
        "deploy_rc": 0, "tree_rc": 0,
        "freshness": (board_gate.PASS, "外参核对于今天，保质期 1 天"),
    }

    # ⚠️ 「板子不在线」这一档要的是闸门**连启动都走不到**（⇒ 没有记录 ⇒
    #    `[SKIP] P3.控制器闸门`）。原来靠「本机恰好没 source ROS」来实现 ——
    #    **靠不住**：`board_gate.py` 是 `run_gate()` 另起的**子进程**，上面那个
    #    `board_lib.dashboard` 桩只在**本进程**生效，子进程看不见。只要调用方的
    #    shell source 了 ROS（而本文件一直说那**就是**该有的环境），子进程就能
    #    import rclpy、真的跑一遍闸门，于是留下记录，`[SKIP] P3.控制器闸门`
    #    永不出现 ⇒ **断言在它自己文档指定的环境里恒红**。
    #    2026-10-06 真机第 1 次当天实测：不 source 退 0、source 了退 1，红的正是
    #    这一条；而板上 `sync-board.sh` 跑它时没 source，所以一直没露头。
    #    ⇒ 把「没有 rclpy」变成**夹具自己造的事实**：`board_gate.py` 的 `argv[0]`
    #    是 `sys.executable`（绝对路径），它唯一的 rclpy 来源就是 `PYTHONPATH`。
    #    `ROS_DISTRO` / `AMENT_PREFIX_PATH` **必须留着** —— P0.ROS 环境 的判据查的
    #    就是它们（`judge_ros_env`），一起摘掉会把 P0 弄红，那是**另一个**假红。
    def _rosless_env(*a, **kw):
        e = dict(os.environ)
        e.pop("PYTHONPATH", None)
        e.pop("PYTHONHOME", None)
        return e, ("（测试桩：**故意**摘掉 PYTHONPATH —— 『板子不在线』这一档要的就是"
                   "闸门子进程 import 不到 rclpy）")

    # 先把**前提**钉住（照本文件「先确认这个『残留』真的被判据看得见」的先例）。
    # 不钉的话，哪天本机 pip 装了 rclpy，这一档会**悄悄**变成另一回事，而下面那条
    # 断言还绿着 —— 那就是空绿。
    _probe = subprocess.run([sys.executable, "-c", "import rclpy"],
                            env=_rosless_env()[0], capture_output=True, text=True)
    check("★ 夹具前提：摘掉 PYTHONPATH 后闸门子进程**真的** import 不到 rclpy",
          _probe.returncode != 0,
          "rc=%s %s" % (_probe.returncode, _probe.stderr.strip()[-160:]))

    def end_to_end(task_path, tag):
        """跑一次真的 `run()`：P1 全绿 → P2 起（假）栈 → P3 等不到播放 ⇒ 退 2 + 收栈。"""
        old_env = {k: os.environ.get(k) for k in
                   ("PATH", "ROS_DISTRO", "AMENT_PREFIX_PATH", "ROS_DOMAIN_ID")}
        old_install = board_run.INSTALL_SETUP
        old_collect = board_run.collect_preflight
        old_dash = board_lib.dashboard
        old_build = board_run.build_ros_env
        os.environ["PATH"] = bindir + os.pathsep + no_ros
        # ⚠️ AMENT_PREFIX_PATH 里**必须**有本项目工作区（= 下面 INSTALL_SETUP 所在
        #    那棵树）贡献的一条 —— 新判据查的就是这个**行为**，不是「非空」这个代理。
        #    而且照现场形状：**逐包**前缀 + 裸 install 目录，两种都在（colcon 隔离式
        #    只放逐包那条，合并式只放裸那条；两种布局都得认）。
        ws_ins = os.path.join(bindir, "ws", "install")
        os.makedirs(ws_ins, exist_ok=True)
        with open(os.path.join(ws_ins, "setup.bash"), "w", encoding="utf-8") as fh:
            fh.write("# 夹具：假装这份检出 colcon build 过了\n")
        os.environ.update(ROS_DISTRO="humble",
                          AMENT_PREFIX_PATH=os.pathsep.join(
                              [os.path.join(ws_ins, "yolo_bringup"), ws_ins,
                               "/opt/ros/humble"]),
                          ROS_DOMAIN_ID="99")
        board_run.INSTALL_SETUP = os.path.join(ws_ins, "setup.bash")  # 假装 install 在
        # 换掉「自己 source」：本机没有那份工作区，而且这里要测的是闸门不是 shell。
        # 用**摘掉 PYTHONPATH**的那一份（理由见上面 `_rosless_env` 的长注释）。
        board_run.build_ros_env = _rosless_env
        board_run.collect_preflight = (
            lambda site, do_net=True, override=None, env=None: dict(FACTS_GREEN))
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
            board_run.build_ros_env = old_build
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

    # ---- ②′ 红线 3：运行记录不许落进公开仓（**P0 就拦，起栈之前**）-----------
    # 2026-10-06 抓到的洞：`--json-report` 只在**最后**判了一下、且**不翻退出码**
    # （拒绝了却退 0）；`--log` 则**根本没判** —— 它是 `open(path,"w")` 直接写的，
    # 起栈那一刻就往里灌栈日志（里面有控制箱 IP、相机序列号、串口 by-id）。
    inrepo = os.path.join(REPO, "scripts", "_never_write_log.txt")
    out_repo = os.path.join(tmpdir, "fine.log")
    st, det = board_run.judge_output_paths(inrepo, None)
    check("★ `--log` 指向仓库内 ⇒ FAIL（这一半在 2026-10-06 之前**根本没人判**）",
          st == board_run.FAIL and "--log" in det)
    check("  理由里点名红线 3，并说清「不会写、也不会起栈」",
          "红线 3" in det and "不会写" in det and "起栈" in det)
    st, det = board_run.judge_output_paths(None, os.path.join(REPO, "x.json"))
    check("`--json-report` 指向仓库内 ⇒ FAIL", st == board_run.FAIL
          and "--json-report" in det)
    check("  两个都在仓库外 ⇒ PASS（不是「一律拒」）",
          board_run.judge_output_paths(out_repo, out_repo)[0] == board_run.PASS)
    check("  两个都没给 ⇒ PASS（默认日志落在 `~/`，本来就在仓库外）",
          board_run.judge_output_paths(None, None)[0] == board_run.PASS)
    check("  纯函数：只按路径算，不读环境、不碰文件",
          board_run.judge_output_paths.__code__.co_argcount <= 3)

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
    # 13b. 红线 6 第二条写路径：`--init-gripper`（夹爪寄存器）
    # =================================================================
    section("[13b] 红线 6：夹爪初始化并进启动器 —— 选最小一档、报文先摆、当次确认")

    # ---- ① 选档：读到的 0x0200 → 该跑哪一档（**最小**那一档）---------
    argv1, note1 = board_run.gripper_init_plan(1)
    check("0x0200=1（已初始化）⇒ **只合拢**：argv 里**没有** --go "
          "（不重走全行程自检 —— 那是白动一次）",
          argv1 == ["--close"], "argv=%s" % (argv1,))
    check("  而且说明里写得出「不重走全行程自检」",
          "只合拢" in note1 and "全行程自检" in note1, note1)
    check("0x0200=0（未初始化）⇒ 初始化 + 合拢（一步到位）",
          board_run.gripper_init_plan(0)[0] == ["--go", "--close"])
    a2, n2 = board_run.gripper_init_plan(2)
    check("0x0200=2（**初始化进行中**）⇒ 拒绝，不给命令（再发一次是往半途插一脚）",
          a2 is None and "进行" in n2, "%s / %s" % (a2, n2))
    aN, nN = board_run.gripper_init_plan(None)
    check("读不到 0x0200 ⇒ 拒绝（串口不通时写指令到没到都说不准）—— fail-closed",
          aN is None and "读不到" in nN, "%s / %s" % (aN, nN))
    aX, nX = board_run.gripper_init_plan(7)
    check("不认识的值 ⇒ 拒绝，且**点名**那个值（不是默默当成 0）",
          aX is None and "7" in nX, "%s / %s" % (aX, nX))

    # ---- ② 报文：人确认的是**那一条** ---------------------------------
    close_lines = "\n".join(board_run.gripper_init_payload_lines(["--close"]))
    both_lines = "\n".join(board_run.gripper_init_payload_lines(["--go", "--close"]))
    check("报文里写着要跑的**确切命令行**",
          "ag95_init_close.py --close" in close_lines)
    check("`--close` 那一档明说**不写 0x0100**（不重走自检）",
          "0x0100" in close_lines and "**否**" in close_lines)
    check("`--go --close` 那一档明说 0x0100 要写、且是一次**全行程自检**",
          "全行程自检" in both_lines and "**是**" in both_lines)
    check("两档印出来**不是同一份**（否则「确认」确认的不是将要发生的事）",
          close_lines != both_lines)
    check("报文里写了 15.6 mm 那个物理后果（合拢时指尖再探出）",
          "15.6 mm" in close_lines and "15.6 mm" in both_lines)
    check("力的值与 ag95_init_close 的常量同一个数（50 N —— 与 grasp_planner.yaml 的"
          "ag95_force 一致）",
          str(board_run.ag95_init_close.FORCE_N) in close_lines
          and board_run.ag95_init_close.FORCE_N == 50)
    check("  而且是**引用**来的，不是这里又抄一个数（抄了就会静默分叉，"
          "而人确认的是**那一条**）",
          "ag95_init_close.FORCE_N" in SRC
          and "ag95_init_close.TARGET_PERMILLE" in SRC
          and "ag95_init_close.REG_INIT" in SRC)
    check("报文里写了当次授权那句（数据文件不许冒充授权）",
          "只认**这一次**" in close_lines)

    # ---- ③ 整条链（依赖注入）：没确认就**一个寄存器都不跑** -----------
    calls = []
    saw = []

    def _fake_run(argv, **kw):
        calls.append(list(argv))
        return 0, "✅ 假装写完了"

    def _step(status, answer=True):
        calls[:] = []
        return board_run.gripper_init_step(
            "/dev/假装",
            read_status=lambda port: (status, "ok"),
            ask=lambda: answer,
            run=_fake_run, out=lambda *a: saw.append(" ".join(map(str, a))))

    rec = _step(1)
    check("0x0200=1 + 确认 ⇒ 真的跑了，且跑的是 `--close`（不写 0x0100）",
          calls == [["--close"]] and rec["rc"] == 0, "calls=%s" % (calls,))
    rec = _step(0)
    check("0x0200=0 ⇒ 跑 `--go --close`", calls == [["--go", "--close"]])
    rec = _step(2)
    check("0x0200=2 ⇒ **一次都没跑**（竞态里的半途不许插一脚）",
          calls == [] and rec["refused"], "calls=%s" % (calls,))
    rec = _step(None)
    check("读不到 ⇒ **一次都没跑**", calls == [] and rec["refused"])
    rec = _step(1, answer=False)
    check("★ 已初始化、但**没确认** ⇒ `--close` 也**一次都不跑**"
          "（确认的是「将要发生的事」，不是「要不要继续」）",
          calls == [] and rec["refused"], "calls=%s" % (calls,))
    check("  这一条不是空转：同样的 status 换成 answer=True 就跑得起来",
          _step(1, answer=True) and calls == [["--close"]])

    # ---- ④ 跑完之后这一项怎么算（三态，不许把「没做」渲染成绿） ------
    check("没跑 ⇒ 没有这一项（None ≠ PASS）",
          board_run.judge_gripper_init(None) == (None, ""))
    st, det = board_run.judge_gripper_init(
        {"rc": 0, "out": "✅ 成功", "note": "已初始化（0x0200=1）⇒ **只合拢**"})
    check("退出码 0 ⇒ PASS，并写明 P1 那次读数就是**做完之后**的状态",
          st == board_run.PASS and "做完之后" in det, det)
    st, det = board_run.judge_gripper_init(
        {"rc": 2, "out": "❌ AG-95 未初始化（0x0200=0）—— 拒绝只合拢",
         "note": "未初始化（0x0200=0）"})
    check("退出码 2 ⇒ **FAIL**（不是「没做」，也不是绿），并把脚本自己那行带出来",
          st == board_run.FAIL and "退出码 2" in det and "拒绝只合拢" in det, det)
    st_rc2 = st          # M4 的对照物：下面的 `st` 会被别的夹具覆盖
    # 超时这一路：**判不出退出码 ≠ 进程没在动** —— 这句提醒长在**适配器**里
    # （它才知道自己刚才是超时还是压根没起得来），判定函数把它整段带出去。
    def _boom(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, 90.0)

    rc_to, text_to = board_run.run_gripper_init(["--close"], runner=_boom)
    check("超时 ⇒ 拿不到退出码，且说明里点出**它可能还在动**（先确认两指停了）",
          rc_to is None and "还在动" in text_to, text_to)
    st, det = board_run.judge_gripper_init({"rc": rc_to, "out": text_to, "note": "n"})
    check("  判定函数把这段原样带进报告（不许换一句「失败」了事）",
          st == board_run.FAIL and "还在动" in det, det)

    def _enoent(cmd, **kw):
        raise OSError(2, "No such file or directory")

    rc_nf, text_nf = board_run.run_gripper_init(["--close"], runner=_enoent)
    check("脚本根本起不来 ⇒ 也是 FAIL，说明里带上**具体异常类型**"
          "（`OSError(2, …)` 会细化成 `FileNotFoundError` —— 现场一眼看出是"
          "「找不到脚本」还是「串口没权限」）",
          rc_nf is None and "FileNotFoundError" in text_nf
          and "No such file" in text_nf, text_nf)
    check("  而且给出的命令是**绝对路径**（CWD 不该决定跑的是哪一份脚本）",
          board_run.run_gripper_init(
              ["--close"], runner=lambda cmd, **kw: calls.append(cmd) or
              types.SimpleNamespace(returncode=0, stdout="", stderr=""))
          and calls[-1][1] == os.path.join(REPO, "scripts", "ag95_init_close.py"),
          str(calls[-1]) if calls else "没有调用")
    st, det = board_run.judge_gripper_init({"dry_run": True})
    check("--dry-run ⇒ **SKIP**（是「没做」，不是绿）", st == board_run.SKIP, det)

    # ---- ⑤ 端到端：非交互 + --init-gripper ⇒ 起栈之前就退 2 ----------
    r = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                "--init-gripper", env={"PATH": no_ros})
    check("真跑一次：--init-gripper 在非交互终端 ⇒ P0 红、退 2",
          r.returncode == 2 and "[FAIL] %s" % board_run.GRIPPER_INIT_ITEM in r.stdout,
          "rc=%s" % r.returncode)
    check("  而且**停在 P0**：P1/P2 那两个阶段头一次都没印（拦在起栈之前）",
          "P1 前置" not in r.stdout and "P2 起栈" not in r.stdout)
    check("  文案指出它要的是**当次**授权（与控制器那条同一个判据函数）",
          "--init-gripper" in r.stdout and "当次" in r.stdout)
    r_gd = run_cli("--task", "a1", "--site", site_path, "--dry-run",
                   env={"PATH": no_ros})
    check("  不带这个开关时 P0 里没有这一项（写路径是逐一显式开的）",
          board_run.GRIPPER_INIT_ITEM not in r_gd.stdout
          and "P2 起栈" in r_gd.stdout, "rc=%s" % r_gd.returncode)
    check("  并且报告里那一栏说得清「谁把它弄成这样的」（没给开关时不冒充验证过）",
          "gripper init exercised: no（没给 --init-gripper）" in r_gd.stdout,
          r_gd.stdout[-400:])
    check("  报告 JSON 里有 `gripper_init_exercised` 这一栏（照 repair_exercised）",
          "gripper_init_exercised" in SRC and '"gripper_init_exercised":'
          in SRC)

    # ---- ⑥ 变异：上面每一条「吃劲」吗（改前 ≠ 改后，不是「改坏了会红」）----
    section("[13b-mut] 变异：把闸门拆掉，断言必须翻面")

    # M1 退回 2026-10-07 之前的形状：已初始化也重走全行程自检（`--go --close`）。
    m = load_mutant(_mutate(SRC, '        return (["--close"],',
                            '        return (["--go", "--close"],', "gplan"),
                       "gplan")
    mut_argv, _ = m.gripper_init_plan(1)
    check("M1 已初始化也重走全行程自检（= 改之前的行为）⇒ 「只合拢」那条翻面",
          mut_argv == ["--go", "--close"] and argv1 == ["--close"],
          "变异后 %s；改前 %s" % (mut_argv, argv1))

    # M2 削掉「初始化进行中就拒绝」那道闸门。
    m = load_mutant(_mutate(SRC, "    if status == 2:", "    if False:", "g2"),
                    "g2")
    _ma, mut_n2 = m.gripper_init_plan(2)
    check("M2 削掉「0x0200=2 ⇒ 拒绝」⇒ 文案不再说得出「进行中」（拿半途当没做）",
          "进行" not in (mut_n2 or "") and "进行" in n2,
          "变异后 %r；改前 %r" % (mut_n2, n2))

    # M3 把「当次确认」那道闸门拆掉（先写后问 = 确认变成了走个形式）。
    m = load_mutant(_mutate(SRC, "    if not ask():", "    if False:", "gask"),
                    "gask")
    calls[:] = []
    m.gripper_init_step("/dev/假装",
                        read_status=lambda port: (1, "ok"),
                        ask=lambda: False, run=_fake_run, out=lambda *a: None)
    check("M3 拆掉「没确认就不写」⇒ 一次**都没确认**的调用照样动了手指"
          "（★ 这一条防的是「确认」退化成走过场）",
          calls == [["--close"]], "变异后 calls=%s" % (calls,))

    # M4 非 0 退出码也当成功（把「报成功」与「真做成」混为一谈）。
    m = load_mutant(_mutate(SRC, "    if rc != 0:", "    if False:", "grc"),
                    "grc")
    mut_st, _ = m.judge_gripper_init(
        {"rc": 2, "out": "❌ AG-95 未初始化（0x0200=0）", "note": "未初始化"})
    check("M4 非 0 退出码也当成功 ⇒ FAIL 那条翻成 PASS（脚本说没做成，报告说绿）",
          mut_st == board_run.PASS and st_rc2 == board_run.FAIL,
          "变异后 %s；改前 %s" % (mut_st, st_rc2))

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
