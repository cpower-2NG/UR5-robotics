#!/usr/bin/env python3
"""附着在**已经起着**的栈上做只读体检 —— 「这一轮起栈健康吗」。

它是怎么来的
============
本仓反复踩的那批启动失败里，有一类的形状是一样的：**栈起来了，但某一环是坏的，
而没有任何一处会报错**。第九十轮最典型 —— 同一段代码，通过轮与失败轮只差
「连接 → 控制器激活」是 93 s 还是 4.3 s（驱动在 URCap 连上时会发一次 STRICT
切换，把刚激活的 `scaled_joint_trajectory_controller` 又打回 not-running）。
臂一下都不动，日志里只有一行 `goal_status=<未知 None>`。

`board_a1.py` 的前置检查覆盖**起栈之前**（残留/串口/夹爪/示教器），
`real_chain_check.py` 覆盖**服务与 TF 通不通**。中间这一段 —— 「控制器到底激活
没有、相机在不在出帧、检测器是不是已经死了、规划器是不是早就收工了」—— 原先只有
**人手敲 `ros2 topic echo`** 一条路，而且敲错了会得到空结果，被读成「栈没起来」。

本脚本把这段变成一条命令。

它**不做**什么（这几条比它做什么更重要）
========================================
· **不建 action client、不发 FollowJointTrajectory、不发任何轨迹** —— 与
  `real_chain_check.py` 同一条纪律。
· **不碰 30001/30002/30003/30004**（AGENTS.md 红线 1）。
· **默认一个字节都不写**：只调 `list_controllers` / `get_parameters`（都是只读
  服务），加上只读订阅。唯一的写路径在 `--repair-controller` 后面，见下。
· **不起栈、不收栈**。栈是别人起的，它只附着。

⚠️ 与 Dashboard（29999）的关系（2026-10-08 改）：默认**不碰**。只有
   `--repair-controller` **且没有交互终端**（AI 驱动的那一轮）时，才走
   `board_lib.dashboard()` 做**只读**核验（`robotmode` / `programState` /
   `safetystatus` 三条查询）—— 那是「示教器上那一下是不是人按的」的**唯一**证据
   来源。**仍然不发** play / power / stop（红线 1 + 红线 6：那是人的动作）。

退出码（沿用全仓的 0/1/2，不加第四个）
=====================================
  0 = 每一项都 PASS。
  1 = **有 FAIL** —— 看到了，而且它不对。
  2 = **有 SKIP 但没有 FAIL** —— 我们**没看成**（ROS 图整体不可达，或调用方
      用 `--allow-skip` 显式声明这一档没有某个提供方）。**fail-closed**：
      没验到就不算通过。离线跑本脚本必然落到这一档，那是**正确**的结果。

⚠️ **SKIP 永远不渲染成绿色**；有任何一项 SKIP 时，总结行不写「全绿」。

`--repair-controller`（唯一的写路径）
====================================
默认**只读**：控制器不合格就报出来、退 1，**不碰它**。

理由：`switch_controller` 本身是 `/controller_manager` 上的标准 ROS 2 服务，
不发轨迹点、不碰 30001–30004 ⇒ **不违红线 1**；但它激活
`scaled_joint_trajectory_controller` 等于**把指挥权交给这套栈**，而臂此刻已上电
⇒ 属 AGENTS.md 红线 6 的**真机动作**，吃**当次**授权。所以：
  · 必须显式传 `--repair-controller`；
  · **当次授权**两条路（与 `board_run.handoff_mode()` 同源，两档都要人动手）：
      · 有**交互终端**（人手动跑的那一轮）⇒ 再敲一次 `yes`；
        敲不出来（EOF / 不是 yes）就**不发** switch_controller。
      · **没有**终端（**AI 驱动**那一轮）⇒ **不敲键盘**：改由**示教器上的
        STOP → PLAY 两步握手**承担，用只读 Dashboard 查询核验，**两步都看到**
        才算；核验不了（没给 `--dashboard-host`、读不到控制柜、没看到那两步）
        就**不发**。此时需要调用方给 `--dashboard-host <控制柜 IP>`。
        ⚠️ 这不是「无人值守模式」：它要求的恰恰是**人物理上在示教器前动手**；
           环境变量 / 任务描述 / 上一次的记忆**一律不算授权**（红线 6）。
  · `deactivate_controllers` **恒为 `[]`** —— **永不 deactivate** 任何控制器；
  · 修复走**收敛式**重试而不是盲 `while`：每发一次就重读一次
    `list_controllers`，达标即停。
报告里**必定**有一行 `repair exercised: yes/no` —— 没被绊倒时的绿是**空的**，
不写这行就会把「没触发」读成「验证过了」。

一个必须写在代码里的坑：**别让观测改变被观测的东西**
====================================================
`rclpy` 的 `get_topic_names_and_types()` 会把**我们自己刚建的那条订阅**也算进去。
所以「先订阅 `/camera/color/image_raw`，再问图里有没有这个话题」是**自证式**的
—— 无论相机起没起都答「有」。本脚本因此**先冻结图快照，再建订阅/客户端**
（见 `_freeze_graph`）。这跟仓库里 `pkill -f` 匹配到执行它的那条 shell 是同一族
问题：**判据的载体本身进了判据的定义域**。

用法:
    python3 scripts/board_gate.py                      # 只看，最常用
    python3 scripts/board_gate.py --slots-total 4      # 顺带核对槽位数
    python3 scripts/board_gate.py --allow-skip telemetry,params   # skip_planner 档
    python3 scripts/board_gate.py --json-report ~/gate.json
    python3 scripts/board_gate.py --list-items         # 看项名
退出码 0 / 1 / 2，见上。
"""

import argparse
import datetime
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
# ⚠️ `board_lib` **顶层不 import rclpy/launch**（它的模块 docstring 里写着为什么），
#    所以这里 import 它不会把本脚本「离线可测」的性质弄坏。
#    用它只为两件事：① 「人在环」的两步握手（`ensure_in_the_loop`）；
#    ② 那份**唯一**的 Dashboard 只读查询（`dashboard`）—— 写路径的判据不许在本
#    文件里再抄一份（抄了就会静默分叉）。
#    ⛔ 反过来不许：`board_lib` 不许 import `board_gate`（会成环），
#    `board_gate` 也不许 import `board_run`（它是被 board_run 当子进程调的）。
import board_lib                                          # noqa: E402

# ⚠️ 本模块顶层**不 import rclpy / 任何消息包**。这不是风格洁癖：本机（WSL）的
#    /opt/ros/humble 下**根本没装 rclpy**（只有纯 Python 的 ament_* 工具），
#    顶层一 import，`test_board_gate.py` 连 import 都过不去，判定逻辑也就没法
#    离线测 —— 而离线测正是这套东西唯一能被我验证的部分。rclpy 只在 `_ros()`
#    与几个 `_item_*` 里懒加载，判定函数一律是纯函数。

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"

# ---- 名字 ----
CTRL_SERVICE = "/controller_manager/list_controllers"
SWITCH_SERVICE = "/controller_manager/switch_controller"
COLOR_TOPIC = "/camera/color/image_raw"
DETECTIONS_TOPIC = "/detections"
TARGETS_TOPIC = "/targets_3d"
TELEMETRY_TOPIC = "/grasp_planner/telemetry"
PLANNER_NODE = "grasp_planner"
DETECTOR_NODE = "yolo_detector"
MY_NODE = "board_gate"

# 真机入口的轨迹 action（`pipeline.launch.py` 的 REAL_TRAJECTORY_ACTION）。
# ⚠️ 这是**真机**值；仿真入口用的是 `joint_trajectory_controller`。本闸门针对真机，
#    所以拿它当默认，再由 `params` 项去核对规划器**实际**用的是不是这一个。
DEFAULT_TRAJECTORY_ACTION = (
    "/scaled_joint_trajectory_controller/follow_joint_trajectory"
)
# `/joint_states` 的来源。没有它，TF 与 MoveIt 都无从谈起。
JOINT_STATE_BROADCASTER = "joint_state_broadcaster"

# `grasp_planner.yaml` 里的 **5 个死键**（见 `docs/机械臂系统参考.md` §5.5）：
# 在整车入口（`bringup.launch.py`）下改它们**不生效且不报错** —— 因为
# `planner_node._declare_params` 全是硬编码默认值，只有 launch 的
# `parameters=[...]` 覆盖得进去。所以**有效值 = yaml ⊕ launch 参数**，唯一的
# 查法是回读运行中的节点。报告里逐项打出来，就是让人看见生效的到底是哪个。
DEAD_KEYS = ("gripper_backend", "table_scene_enabled", "ik_group",
             "trajectory_action", "carry_objects")

ITEM_NAMES = ("controllers", "camera", "targets", "detector", "telemetry",
              "params")


# =====================================================================
# 判定：全部是纯函数，不 import ROS，可离线测（见 test_board_gate.py）
# =====================================================================

def controller_from_action(action):
    """从轨迹 action 名里解出**控制器名**。

    `/scaled_joint_trajectory_controller/follow_joint_trajectory`
        → `scaled_joint_trajectory_controller`

    这样「该盯哪个控制器」不需要另抄一份常量：它由规划器实际要用的那个 action
    推导出来，两份真值合成一份（正是 `docs/机械臂系统参考.md` §5.5 那个老毛病）。
    解不出来（段数不够）返回 `None`，由调用方报错，**不猜**。
    """
    parts = [p for p in str(action).split("/") if p]
    if len(parts) < 2:
        return None
    return parts[-2]


def in_zone(p, zone):
    """点是否落在 `pick_zone` 里，闭区间（两端都 `<=`）。

    zone 必须是 6 个数（3D）：2D 矩形挡不住「滚到地上、但 XY 恰在投影里」的方块，
    那会让臂收到一条扎向地面的轨迹。
    """
    xmin, xmax, ymin, ymax, zmin, zmax = zone
    x, y, z = p
    return (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax)


def zone_str(zone):
    xmin, xmax, ymin, ymax, zmin, zmax = zone
    return (f"x[{xmin:+.3f},{xmax:+.3f}] y[{ymin:+.3f},{ymax:+.3f}] "
            f"z[{zmin:+.3f},{zmax:+.3f}]")


def is_active(state):
    """`state` 是不是**恰好**等于 `"active"`。

    ⚠️ 这里必须是**精确相等**。写成 `"act" in state` 之类的子串判定，`inactive`
    会被判成 active（`i-n-a-c-t-i-v-e` 里确实连着含 `act`）—— 一次静默的自欺，
    而且方向是**把坏的报成好的**。抽成独立函数是为了让它只有一处、能被
    `test_board_gate.py` 的变异检查单独打掉。
    """
    return state == "active"


def judge_controllers(samples, required, stable_for):
    """`samples: [(t, {控制器名: state}), ...]`（t 单调不减）→ `(status, detail)`。

    判据：`required` 里每个控制器**逐个具名**取到 `is_active()`，
    并且这个「全 active」的窗口**连续保持** `stable_for` 秒。

    两个都不能省：

    1. **具名 + 精确相等**（见 `is_active`），
    2. **连续保持**，不许只取一个瞬间快照 —— 第九十轮的失败轮就是「短暂 active
       了一下又被 STRICT 切换打回去」，而快照正好落在缝里。93 s / 4.3 s 那个
       教训必须写成代码，写成注释就又会烂。
    """
    if not samples:
        return SKIP, "一次采样都没有（list_controllers 没答话）"
    if not required:
        return FAIL, "调用方没给出要盯的控制器名（--require 与推导都为空）"

    t_end = samples[-1][0]
    since = None
    last_states = {}
    for t, states in samples:
        last_states = states
        if all(is_active(states.get(n)) for n in required):
            if since is None:
                since = t
        else:
            since = None

    bad = [f"{n}={last_states.get(n) or '不在控制器列表里'}"
           for n in required if not is_active(last_states.get(n))]
    if bad:
        return FAIL, "未激活/缺失（末条采样如此）：" + "，".join(bad)

    held = t_end - since
    if held + 1e-9 < stable_for:
        return FAIL, (f"全 active 只保持了 {held:.1f}s（要求 {stable_for:g}s）"
                      f" —— 很可能正撞上驱动的 STRICT 切换，再守一会儿看它掉不掉")
    return PASS, f"{len(required)} 个控制器 active 且连续稳定 {held:.1f}s"


def judge_camera(hz, min_hz):
    """频率判据。`hz is None` ⇒ SKIP（表示话题没测到，由调用方负责区分原因）。"""
    if hz is None:
        return SKIP, "没测到频率（相机话题不在图上）"
    if hz < min_hz:
        return FAIL, (f"{hz:.1f} Hz < 要求 {min_hz:g} Hz —— 相机在图上却不出帧，"
                      f"或者只是个还没退出的空壳节点")
    return PASS, f"{hz:.1f} Hz"


def judge_targets(points, labels, zone, min_candidates):
    """`/targets_3d` 的候选。

    判据两条：**个数 ≥ min_candidates**，且**每一点都落在 pick_zone 内**。

    第二条才是这关的价值所在：外参 `q`/`t` 填错时，投影**照样出数、YOLO 照样给
    0.96 分**，只是整片点被平移/镜像到臂够不到的地方（实测 y≈1.38~1.57）。
    这是**唯一**能发现它的地方（见 `docs/待执行命令.md` 的三组 A/B 表）。

    ⚠️ 刻意取「**全部**在区内」而不是「区内有 N 个」：一片点里只要有一个飞到区外，
    我们就**不知道**哪些是真的 —— 与其替人挑，不如把整条证据摆出来让人看一眼。
    代价是远处一个误检会挡下一轮，这是**故意**的偏向。
    """
    n = len(points)
    if n == 0:
        return FAIL, ("一个候选都没有 —— 检测器没看到方块，或投影全被丢了"
                      "（`projector_node` **不发空数组**，「没消息」与"
                      "「零候选」在它那儿本来就是一回事）")
    if n < min_candidates:
        return FAIL, f"只有 {n} 个候选 < 要求 {min_candidates} 个"
    outside = [(p, l) for p, l in zip(points, labels) if not in_zone(p, zone)]
    if outside:
        head = "；".join(f"({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) {l}"
                         for p, l in outside[:3])
        tail = f"……共 {len(outside)} 个" if len(outside) > 3 else ""
        return FAIL, (f"{n} 个候选里有 {len(outside)} 个落在 pick_zone 外"
                      f"（{zone_str(zone)}）：{head}{tail}"
                      f" —— 「整片点被挪走」正是外参填错的签名")
    return PASS, f"{n} 个候选全部落在 pick_zone 内"


def judge_detector(model, detections_hz):
    """检测器活着没有。

    `model:` 路径解析不到时 `detector_node` **直接退出**，而整栈其余照跑 ——
    现场表现是「臂完全不动」，日志里翻半天才看得到。所以这里既看**参数读不读得到**
    （节点在不在图上），也看它**到底在不在发东西**。
    """
    if model is None:
        return SKIP, "读不到检测器的 model 参数（节点不在图上）"
    if not str(model).strip():
        return FAIL, "检测器的 model 参数是空串 —— 它起不来或起错了"
    if detections_hz is None:
        return FAIL, "参数读得到，但 /detections 不在图上 —— 检测器没在发帧"
    if detections_hz <= 0.0:
        return FAIL, (f"model={model} 但 /detections 一条都没发 —— "
                      f"节点可能已经退出，或在等相机")
    return PASS, f"model={model}，/detections {detections_hz:.1f} Hz"


def judge_telemetry(values, slots_total=None):
    """规划器遥测（`diagnostic_msgs/DiagnosticArray`）解出来的 `values` 字典。

    ⚠️ 订阅方**必须**声明 `TRANSIENT_LOCAL`（见 `latched_qos()`）：发布方 latched
    只表示「它留了一份给晚到者」，而 VOLATILE 的订阅者永远拿不到那份锁存 ——
    **不报错，只是空着**，最容易被读成「话题没在发」。本项只要拿到 `values`，
    就说明 QoS 配对了。
    """
    if values is None:
        return SKIP, "没收到遥测"
    state = values.get("state")
    if state is None:
        return FAIL, "遥测里没有 state —— 话题在发，但报文不是这个形状"
    if state == "FINISHED":
        return FAIL, (f"规划器**已经收工**（state=FINISHED，"
                      f"reason={values.get('finish_reason', '?')}）"
                      f" —— 它不再接新目标，这一轮跑下去只会看到臂不动")
    try:
        free = int(values.get("slots_free"))
        total = int(values.get("slots_total"))
    except (TypeError, ValueError):
        return FAIL, ("读不出槽位计数：slots_free="
                      f"{values.get('slots_free')} slots_total="
                      f"{values.get('slots_total')}")
    if free != total:
        return FAIL, (f"槽位不是全空：已占 {total - free}/{total}"
                      f" —— 上一轮的方块还压在槽位上？")
    if slots_total is not None and total != slots_total:
        return FAIL, (f"槽位总数 {total} ≠ 本任务期望的 {slots_total}"
                      f" —— 多半是 planner_params_file 没传进去，"
                      f"静默用了默认几何")
    return PASS, f"state={state}，槽位 {free}/{total} 全空"


def judge_params(values, expect, trajectory_action):
    """有效参数回读（`/grasp_planner/get_parameters`，**只读**）。

    一次同时关掉两类静默失效：
      · 「参数改了不生效」—— 回读的是**运行中节点**的有效值（yaml ⊕ launch）；
      · 「板子树过期」—— 板上那份代码树不是本机这份的镜像，回读能看出它认的参数
        名/值与本机代码对不上。

    `expect` 是 `{参数名: 期望值}`（来自 `--expect-param`，由任务描述给）。
    空表示只做三条：「读得到、形状对、与闸门盯的轨迹 action 是同一个」。
    """
    if values is None:
        return SKIP, "读不到 /grasp_planner 的参数（规划器不在图上）"

    live = values.get("trajectory_action")
    if live is None:
        return FAIL, ("回读里没有 trajectory_action —— 要么这个节点不是规划器，"
                      "要么板上那份树比本机旧（参数声明对不上）")
    if str(live) != str(trajectory_action):
        return FAIL, (f"规划器实际用的轨迹 action 是 {live}，而本次闸门盯的是 "
                      f"{trajectory_action} —— 两者不是同一个控制器，"
                      f"闸门看的是别人（一份真值变两份的老毛病）")

    for k in sorted(expect):
        if k not in values:
            return FAIL, (f"要核对的参数 {k} 不在回读里 —— "
                          f"要么没声明，要么静默失效了")
        if _norm(values[k]) != _norm(expect[k]):
            return FAIL, f"{k} 实际是 {values[k]!r}，期望 {expect[k]!r}"

    shown = "，".join(f"{k}={_short(values[k])}" for k in DEAD_KEYS
                      if k in values)
    return PASS, ("有效参数（yaml ⊕ launch，回读所得）："
                  + (shown or "（5 个死键一个都没回读到 —— 可疑）"))


def _norm(v):
    """把参数值与命令行给的值归一到同一种写法上比较。

    命令行只能给字符串，而回读回来的是 bool / float / list。`carry_objects` 是
    bool：`True` 与 `"true"` 必须判等，否则任务描述里写 `carry_objects=true`
    会被报成不符 —— 那是**判据自己错**，比不判还坏。
    """
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(_norm(x) for x in v) + "]"
    if isinstance(v, float):
        return f"{v:g}"
    s = str(v).strip()
    return s.lower() if s.lower() in ("true", "false") else s


def _short(v, limit=70):
    s = _norm(v)
    return s if len(s) <= limit else s[:limit - 1] + "…"


# =====================================================================
# ROS 侧薄适配器 —— 只有这里碰 rclpy
# =====================================================================

def _ros():
    import rclpy
    return rclpy


def latched_qos():
    """遥测那几条话题的 QoS：**必须显式 TRANSIENT_LOCAL**。

    发布方是 latched（`telemetry.py` 的 `_latched()`），订阅方漏声明不会报错，
    只会永远收不到锁存的那一条 —— 本仓实测踩过（2026-09-29）。抽成独立函数是
    为了让 `test_board_gate.py` 能在**源码层面**把它钉住（本机没有 rclpy，
    跑不出真的 QoS 对象），并钉住「遥测那一项确实用了它」。
    """
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Probe:
    """把一个 rclpy 节点包成几个「读一次」的方法。

    ⚠️ 除 `switch_controllers()` 外**全是只读**：`call()` 只该用来调
    `list_controllers` / `get_parameters`。
    """

    def __init__(self, rclpy, node):
        self._rclpy = rclpy
        self.node = node
        self._clients = {}

    def graph(self):
        return {
            "topics": dict(self.node.get_topic_names_and_types()),
            "services": dict(self.node.get_service_names_and_types()),
            "nodes": [n for n in self.node.get_node_names() if n != MY_NODE],
        }

    def call(self, srv_cls, name, req, timeout, wait=5.0):
        cli = self._clients.get(name)
        if cli is None:
            cli = self.node.create_client(srv_cls, name)
            self._clients[name] = cli
        if not cli.wait_for_service(timeout_sec=wait):
            return None
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return fut.result() if fut.done() else None

    def list_controllers(self, timeout=5.0):
        """→ `{控制器名: state}`；读不到返回 `None`。"""
        from controller_manager_msgs.srv import ListControllers
        res = self.call(ListControllers, CTRL_SERVICE,
                        ListControllers.Request(), timeout)
        if res is None:
            return None
        return {c.name: c.state for c in res.controller}

    def switch_controllers(self, names, timeout=8.0):
        """⚠️ **写路径**。`deactivate_controllers` **恒为 `[]`** —— 永不 deactivate。

        `strictness: 1`（BEST_EFFORT）是**必须**的：已经在 active 的控制器
        （`friction_model_controller` 通常就是）只会**告警**，不会中止整批；
        用 STRICT 的话整批 `Aborting, no controller is switched!`。
        """
        from controller_manager_msgs.srv import SwitchController
        req = SwitchController.Request()
        req.activate_controllers = list(names)
        req.deactivate_controllers = []          # ← 恒空，别改
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout.sec = int(timeout)
        return self.call(SwitchController, SWITCH_SERVICE, req, timeout + 2.0)

    def get_params(self, node_name, names, timeout=5.0):
        """只读回读参数 → `{名: 值}`。

        未声明的键（`PARAMETER_NOT_SET`）**直接丢掉**，好让调用方用
        `k not in values` 判定「这个节点上没有这个参数」—— 否则未声明的键会带着
        type=NOT_SET 出现在响应里，看起来「有」，值却是空的。
        """
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        req = GetParameters.Request()
        req.names = list(names)
        res = self.call(GetParameters, f"/{node_name}/get_parameters", req,
                        timeout)
        if res is None:
            return None
        out = {}
        for name, pv in zip(names, res.values):
            if pv.type == ParameterType.PARAMETER_NOT_SET:
                continue
            out[name] = _param_value(pv)
        return out

    def hz(self, msg_cls, topic, window, qos=10):
        """在 `window` 秒里数消息条数换算成 Hz。**零条也返回 0.0**
        （与「话题不在图上」是两件事，由调用方分开判）。"""
        n = [0]
        sub = self.node.create_subscription(
            msg_cls, topic, lambda _m: n.__setitem__(0, n[0] + 1), qos)
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < window:
                self._rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = time.monotonic() - t0
        finally:
            self.node.destroy_subscription(sub)
        return n[0] / max(elapsed, 1e-9)

    def count_and_sample(self, msg_cls, topic, timeout, qos=10):
        """同一个窗口里既数条数、又留最后一条。

        `/targets_3d` 专用：它**不发空数组**，所以必须区分「窗口里没消息」与
        「有消息但零候选」—— 前者是投影全丢，后者才是真的没目标。
        """
        box = []
        sub = self.node.create_subscription(msg_cls, topic, box.append, qos)
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                self._rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        return len(box), (box[-1] if box else None)

    def sample(self, msg_cls, topic, timeout, qos=10):
        """取**一条**消息（取不到返回 `None`）。"""
        _n, msg = self.count_and_sample(msg_cls, topic, timeout, qos)
        return msg


def _param_value(pv):
    from rcl_interfaces.msg import ParameterType as PT
    return {
        PT.PARAMETER_BOOL: lambda: bool(pv.bool_value),
        PT.PARAMETER_INTEGER: lambda: int(pv.integer_value),
        PT.PARAMETER_DOUBLE: lambda: float(pv.double_value),
        PT.PARAMETER_STRING: lambda: str(pv.string_value),
        PT.PARAMETER_BYTE_ARRAY: lambda: list(pv.byte_array_value),
        PT.PARAMETER_BOOL_ARRAY: lambda: list(pv.bool_array_value),
        PT.PARAMETER_INTEGER_ARRAY: lambda: list(pv.integer_array_value),
        PT.PARAMETER_DOUBLE_ARRAY: lambda: list(pv.double_array_value),
        PT.PARAMETER_STRING_ARRAY: lambda: list(pv.string_array_value),
    }[pv.type]()


def _det_point(d3):
    """从 `Detection3D` 里取 `((x,y,z), 标签)`。

    位置**优先取 `bbox.center`**：`projector_node` 把它点名成「3D 位置在此」，
    且把同一个 `base_pose` 也写进了 `results[0].pose`，两处应当一致。
    """
    p = d3.bbox.center.position
    label = "?"
    if d3.results:
        label = d3.results[0].hypothesis.class_id or "?"
    return (p.x, p.y, p.z), label


# =====================================================================
# 逐项
# =====================================================================

class Item:
    """一个三态项 ＋ 一个**正交**的 `vacuous`：这一条绿**算不算数**。

    `vacuous` 三态，且**只在 `P3.示教器 PLAYING` 上用**：

    | 值 | 含义 |
    |---|---|
    | `False`（默认） | 与「人在环」无关，或已确认是**人**干的 |
    | `True` | **确认空洞**：绿是真的，但「人在环走通了」是**假的**（残 PLAYING） |
    | `None` | **判不了**（起栈前没读到控制柜）—— 最容易被读成「验过了」的那一档 |

    ⚠️ 为什么要有它：状态列只有 `PASS`，而 `PASS` 有**两种含义**（「人按的」与
    「上一轮残状态，人根本没动手」），真相原先**只在 detail 的散文里**。⇒ 连跑
    三次若只看状态列，会把第 2 次数成一次「人在环走通了」（2026-10-07 R0 阶梯
    实测，差一点就这么数了）。这与本仓「**SKIP 不得渲染成绿色**」是同一个病：
    一个绿有两种含义，而记录里只剩一种字面。
    """
    __slots__ = ("name", "status", "detail", "evidence", "vacuous")

    def __init__(self, name, status, detail, evidence=None, vacuous=False):
        self.name = name
        self.status = status
        self.detail = detail
        self.evidence = evidence or {}
        self.vacuous = vacuous

    def as_dict(self):
        d = {"item": self.name, "status": self.status,
             "detail": self.detail, "evidence": self.evidence}
        # 只在**不是** `False` 时才写这个键：`False` 是默认值（＝这一项与人在环
        # 无关），给每一项都盖一个 `"vacuous": false` 是噪声。`True` 与 `None`
        # 都必须落进 JSON —— 尤其 `None`，它是最容易被读成「验过了」的那一档。
        if self.vacuous is not False:
            d["vacuous"] = self.vacuous
        return d


def _required_controllers(args):
    if args.require:
        return [n.strip() for n in args.require.split(",") if n.strip()]
    out = []
    derived = controller_from_action(args.trajectory_action)
    if derived:
        out.append(derived)
    out.append(JOINT_STATE_BROADCASTER)
    return out


def _item_controllers(args, probe):
    required = _required_controllers(args)
    if args.require is None and controller_from_action(
            args.trajectory_action) is None:
        return Item("controllers", FAIL,
                    f"从 {args.trajectory_action!r} 解不出控制器名 —— "
                    f"形状不对，请用 --require 显式给")

    samples = []
    t0 = time.monotonic()
    deadline = t0 + args.controller_timeout
    hb = 0.0
    while True:
        states = probe.list_controllers()
        if states is not None:
            samples.append((time.monotonic() - t0, states))
        status, detail = judge_controllers(samples, required, args.stable_for)
        if status == PASS:
            break
        if time.monotonic() >= deadline:
            break
        # 心跳：默认要守 8 s 稳定窗口，期间一言不发会让人以为卡死
        if time.monotonic() - hb >= 5.0:
            hb = time.monotonic()
            cur = "，".join(f"{n}={(states or {}).get(n, '不在列表里')}"
                            for n in required)
            print(f"    … 等控制器（已 {time.monotonic() - t0:.0f}s：{cur}）",
                  flush=True)
        time.sleep(max(0.05, args.controller_poll))

    return Item("controllers", status,
                detail + f"（盯 {', '.join(required)}）",
                {"required": required,
                 "samples": [[round(t, 2), s] for t, s in samples]})


def _item_camera(args, probe, graph):
    from sensor_msgs.msg import Image
    if args.camera_topic not in graph["topics"]:
        hint = ""
        if args.camera_topic == COLOR_TOPIC:
            hint = ("；⚠️ 别去查 `/camera/camera/color/image_raw` —— 那是相机节点"
                    "**重映射前**的名字，查错名会误判成「相机没起来」")
        return Item("camera", FAIL, f"{args.camera_topic} 不在图上{hint}")
    hz = probe.hz(Image, args.camera_topic, args.camera_window)
    status, detail = judge_camera(hz, args.camera_min_hz)
    return Item("camera", status, detail,
                {"topic": args.camera_topic, "hz": round(hz, 2)})


def _item_targets(args, probe, graph):
    from vision_msgs.msg import Detection2DArray, Detection3DArray

    if args.pick_zone_list is not None:
        zone, zone_src = args.pick_zone_list, "**来自 --pick-zone，不是活读**"
    else:
        # 能推导的不复述：pick_zone 从运行中的规划器**活读**。
        vals = probe.get_params(PLANNER_NODE, ["pick_zone"])
        if vals and "pick_zone" in vals:
            zone, zone_src = [float(v) for v in vals["pick_zone"]], "活读自规划器"
        else:
            return Item("targets", SKIP,
                        "拿不到 pick_zone（规划器不在图上，也没给 --pick-zone）"
                        " —— 没有它就无法判断点落在哪儿")

    if args.targets_topic not in graph["topics"]:
        return Item("targets", FAIL, f"{args.targets_topic} 不在图上")

    n_tgt, msg = probe.count_and_sample(Detection3DArray, args.targets_topic,
                                        args.targets_timeout)
    if msg is None:
        # 顺带数一下 /detections，把「检测器没看到」与「投影全丢了」分开
        n_det = 0
        if DETECTIONS_TOPIC in graph["topics"]:
            n_det, _ = probe.count_and_sample(Detection2DArray,
                                              DETECTIONS_TOPIC, 2.0)
        return Item("targets", FAIL,
                    f"{args.targets_timeout:g}s 内一条 {args.targets_topic} "
                    f"都没有（同期 /detections 有 {n_det} 条）—— "
                    + ("检测器在发而投影全丢，先看 TF 与深度"
                       if n_det else "两个话题都没消息，先看检测器与相机"),
                    {"detections_msgs": n_det, "targets_msgs": 0,
                     "zone": zone, "zone_src": zone_src})

    pts, labels = [], []
    for d3 in msg.detections:
        p, lab = _det_point(d3)
        pts.append(p)
        labels.append(lab)
    status, detail = judge_targets(pts, labels, zone, args.min_candidates)
    return Item("targets", status, detail,
                {"points": [[round(c, 4) for c in p] for p in pts],
                 "labels": labels, "zone": zone, "zone_src": zone_src,
                 "targets_msgs": n_tgt})


def _item_detector(args, probe, graph):
    from vision_msgs.msg import Detection2DArray
    if DETECTOR_NODE not in graph["nodes"]:
        return Item("detector", FAIL,
                    f"图上没有 {DETECTOR_NODE} —— 要么它已经退出（`model:` 路径"
                    f"解析不到会直接死），要么这一档用 skip_detector:=true 没起它"
                    f"（没起就请 --allow-skip detector）")
    vals = probe.get_params(DETECTOR_NODE, ["model"])
    model = vals.get("model") if vals else None
    hz = (probe.hz(Detection2DArray, DETECTIONS_TOPIC, args.camera_window)
          if DETECTIONS_TOPIC in graph["topics"] else None)
    status, detail = judge_detector(model, hz)
    return Item("detector", status, detail,
                {"model": model,
                 "detections_hz": None if hz is None else round(hz, 2)})


def _item_telemetry(args, probe, graph):
    from diagnostic_msgs.msg import DiagnosticArray
    if args.telemetry_topic not in graph["topics"]:
        return Item("telemetry", FAIL,
                    f"{args.telemetry_topic} 不在图上 —— 规划器没起"
                    f"（skip_planner:=true？没起就请 --allow-skip telemetry）")
    msg = probe.sample(DiagnosticArray, args.telemetry_topic,
                       args.telemetry_timeout, qos=latched_qos())
    if msg is None or not msg.status:
        return Item("telemetry", FAIL,
                    f"{args.telemetry_timeout:g}s 内没收到锁存的遥测 —— 话题在图"
                    f"上而这里空着的话，先怀疑订阅侧漏了 TRANSIENT_LOCAL")
    values = {kv.key: kv.value for kv in msg.status[0].values}
    status, detail = judge_telemetry(values, args.slots_total)
    return Item("telemetry", status, detail, {"values": values})


def _item_params(args, probe, graph):
    if PLANNER_NODE not in graph["nodes"]:
        return Item("params", FAIL,
                    f"图上没有 {PLANNER_NODE} —— 规划器没起"
                    f"（没起就请 --allow-skip params）")
    names = ["trajectory_action", "pick_zone", "place_slots", "place_z"]
    names += [k for k in DEAD_KEYS if k not in names]
    vals = probe.get_params(PLANNER_NODE, names)
    status, detail = judge_params(vals, args.expect_params,
                                  args.trajectory_action)
    return Item("params", status, detail,
                {"readback": {k: _norm(v) for k, v in (vals or {}).items()}})


# =====================================================================
# 修复路径（唯一的写路径）
# =====================================================================

def _run_repair(args, probe, required, isatty=None, loop=None, out=print):
    """收敛式把 `required` 拉起来。

    第一步不是「发命令」，是**要当次授权**（AGENTS.md 红线 6）。两条路，**都要人
    动手**（与 `board_run.handoff_mode()` 同源，判据只有一份）：
      · 有交互终端 ⇒ 敲 `yes`；
      · **没有**终端（AI 驱动）⇒ 不敲键盘，改由**示教器上的 STOP → PLAY 两步握手**
        承担，只读核验（`board_lib.ensure_in_the_loop`），两步都看到才算。
    拿不到证明就**不发** switch_controller —— fail-closed，不是「那就当确认了」。
    """
    isatty = sys.stdin.isatty if isatty is None else isatty
    loop = board_lib.ensure_in_the_loop if loop is None else loop
    out()
    out("    ⚠️  --repair-controller 会对 /controller_manager 发 "
        "switch_controller，")
    out("        激活 %s —— 这等于**把指挥权交给这套栈**，" % ", ".join(required))
    out("        而臂此刻已上电。属 AGENTS.md 红线 6 的**真机动作**。")
    out("        deactivate_controllers 恒为 []，本脚本**永不 deactivate**。")
    if not isatty():
        out("    没有交互终端（**AI 驱动**）⇒ **不敲键盘**：这次授权由**示教器上"
            "的物理动作**承担（STOP → PLAY 两步握手，只读 Dashboard 核验）。")
        out("    请确认：**急停就在手边**、**工作区已隔离**。")
        ok, why = loop(getattr(args, "dashboard_host", None),
                       getattr(args, "play_timeout", 300.0), prompt=out,
                       why="这次 switch_controller 的授权",
                       consequence="不发 switch_controller", doing="修复控制器",
                       step=(1, 2))
        out("    %s %s" % ("✅" if ok else "❌", why))
        if not ok:
            return False
    else:
        try:
            ans = input("        确认这是**当次**授权、急停就绪、工作区已隔离？"
                        "(yes) > ")
        except EOFError:
            return False
        if ans.strip().lower() != "yes":
            out("        未确认 —— 不动控制器。")
            return False

    for i in range(1, args.repair_attempts + 1):
        probe.switch_controllers(required)
        time.sleep(1.5)
        states = probe.list_controllers()
        if states and all(is_active(states.get(n)) for n in required):
            out(f"        第 {i} 次尝试后全部 active。")
            return True
        out(f"        第 {i} 次尝试后仍未达标（最多 {args.repair_attempts} 次）。")
    return False


def _recheck_controllers(args, probe):
    """修复后再守一遍稳定窗口 —— 修完立刻判 PASS 会正好落进 STRICT 的缝里。"""
    required = _required_controllers(args)
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < max(args.stable_for + 2.0,
                                      args.controller_poll * 2):
        states = probe.list_controllers()
        if states is not None:
            samples.append((time.monotonic() - t0, states))
        time.sleep(max(0.05, args.controller_poll))
    status, detail = judge_controllers(samples, required, args.stable_for)
    return Item("controllers", status, detail + "（修复后复核）",
                {"required": required, "after_repair": True,
                 "samples": [[round(t, 2), s] for t, s in samples]})


# =====================================================================
# 主流程
# =====================================================================

def _freeze_graph(probe, timeout):
    """**先冻结图快照，再建订阅/客户端。**

    顺序反了就是自证式判据：`rclpy` 的 `get_topic_names_and_types()` 会把我们
    自己刚建的那条订阅算进去，「图里有没有这个话题」于是永远答「有」。
    判据的载体进了判据的定义域 —— 与 `pkill -f` 匹配到自己那条 shell 同族。

    冻结条件：看见 `/controller_manager/list_controllers` **之后**，图的规模
    （节点/话题/服务数）连续两次采样不变 —— 发现是异步的，一看见控制器服务就
    定稿会漏掉还没被发现的相机话题。
    """
    t0 = time.monotonic()
    g = probe.graph()
    last_shape = None
    stable = 0
    while time.monotonic() - t0 < timeout:
        g = probe.graph()
        shape = (len(g["nodes"]), len(g["topics"]), len(g["services"]))
        if CTRL_SERVICE in g["services"] and shape == last_shape:
            stable += 1
            if stable >= 2:
                break
        else:
            stable = 0
        last_shape = shape
        time.sleep(0.5)
    return g, time.monotonic() - t0


def _is_skipped_ok(it, allow_skip):
    """`--allow-skip` 的落点：**调用方显式声明这一档本来就没有它**。

    只有被声明的项才转成 SKIP。`controllers` 永远不许被声明 —— 它是这个脚本
    存在的理由，允许跳过等于允许它不干活。
    """
    return it.name in allow_skip and it.name != "controllers"


def _emit(it, allow_skip):
    if _is_skipped_ok(it, allow_skip) and it.status != PASS:
        it.status = SKIP
        it.detail += "（调用方 --allow-skip 声明本档没有它）"
    print(f"  [{it.status}] {it.name:<12} {it.detail}")
    if it.status == SKIP:
        print("             ↳ **没验到**，不计入通过。")


def _parse_zone(text):
    if not text:
        return None
    parts = [p for p in text.replace(" ", "").split(",") if p]
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    return vals if len(vals) == 6 else None


def _parse_expect(pairs):
    out = {}
    for p in pairs:
        if "=" not in p:
            continue
        k, v = p.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _write_json(path, args, graph, items, code, repair_exercised, domain):
    rec = {
        "script": "board_gate.py",
        "when": datetime.datetime.now().astimezone().isoformat(
            timespec="seconds"),
        "ros_domain_id": domain,
        "graph": {"nodes": len(graph["nodes"]), "topics": len(graph["topics"]),
                  "services": len(graph["services"])},
        "trajectory_action": args.trajectory_action,
        "required_controllers": _required_controllers(args),
        "stable_for": args.stable_for,
        "allow_skip": list(args.allow_skip),
        "repair_exercised": repair_exercised,
        "items": [it.as_dict() for it in items],
        "exit": code,
    }
    path = os.path.expanduser(path)
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(rec, fh, ensure_ascii=False, indent=2)
        print(f"  （JSON 记录 -> {path}）")
    except OSError as exc:
        print(f"  ⚠️ JSON 记录写不进去：{exc}")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="附着在已起的栈上做只读体检（控制器闸门 + 起栈后预检）")
    ap.add_argument("--trajectory-action", default=DEFAULT_TRAJECTORY_ACTION,
                    help="规划器要用的轨迹 action；控制器名由它推导。默认真机值")
    ap.add_argument("--require", default=None,
                    help="逗号分隔的控制器名；默认 = 推导出的轨迹控制器 + "
                         "joint_state_broadcaster")
    ap.add_argument("--stable-for", type=float, default=8.0,
                    help="控制器必须连续稳定多少秒（默认 8；⚠️ 这是 n=2 的假设，"
                         "不是实测常量）")
    ap.add_argument("--controller-timeout", type=float, default=60.0)
    ap.add_argument("--controller-poll", type=float, default=1.0)
    ap.add_argument("--camera-topic", default=COLOR_TOPIC)
    ap.add_argument("--camera-window", type=float, default=3.0)
    ap.add_argument("--camera-min-hz", type=float, default=5.0,
                    help="「在出帧」的松判据（实测 22.3 Hz，这里只要 ≥5）")
    ap.add_argument("--targets-topic", default=TARGETS_TOPIC)
    ap.add_argument("--targets-timeout", type=float, default=20.0)
    ap.add_argument("--min-candidates", type=int, default=4)
    ap.add_argument("--pick-zone", default=None,
                    help="xmin,xmax,ymin,ymax,zmin,zmax；不给就从规划器活读")
    ap.add_argument("--slots-total", type=int, default=None,
                    help="本任务期望的槽位总数；给了就核对")
    ap.add_argument("--expect-param", action="append", default=[],
                    metavar="K=V", help="额外核对的参数，可重复")
    ap.add_argument("--telemetry-topic", default=TELEMETRY_TOPIC)
    ap.add_argument("--telemetry-timeout", type=float, default=15.0)
    ap.add_argument("--discover-timeout", type=float, default=25.0)
    ap.add_argument("--allow-skip", default="",
                    help="调用方**显式**声明这一档本来就没有的项（逗号分隔），"
                         "如 skip_planner:=true 时给 telemetry,params")
    ap.add_argument("--repair-controller", action="store_true",
                    help="⚠️ 写路径：控制器不合格时收敛式拉起（当次授权：终端里敲 yes，"
                         "或示教器 STOP→PLAY 两步握手 + --dashboard-host）")
    ap.add_argument("--repair-attempts", type=int, default=3)
    ap.add_argument("--dashboard-host", default=None,
                    help="控制柜 IP —— **只在** `--repair-controller` 且没有交互终端"
                         "时用到：走 29999 做**只读**核验（robotmode/programState/"
                         "safetystatus），看示教器上那一下是不是人按的。不发 play/"
                         "power/stop。不给 ⇒ 核验不了 ⇒ **不发** switch_controller")
    ap.add_argument("--play-timeout", type=float, default=300.0,
                    help="两步握手每一步最多等多久（秒）")
    ap.add_argument("--json-report", default=None)
    ap.add_argument("--list-items", action="store_true")
    a = ap.parse_args(argv)

    if a.list_items:
        print("\n".join(ITEM_NAMES))
        return 0

    a.allow_skip = [s.strip() for s in a.allow_skip.split(",") if s.strip()]
    a.expect_params = _parse_expect(a.expect_param)
    a.pick_zone_list = _parse_zone(a.pick_zone)
    if a.pick_zone and a.pick_zone_list is None:
        print(f"❌ --pick-zone 要 6 个数（xmin,xmax,ymin,ymax,zmin,zmax），"
              f"得到 {a.pick_zone!r}")
        return 2
    if "controllers" in a.allow_skip:
        print("❌ controllers 不许 --allow-skip —— 它是这个脚本存在的理由")
        return 2
    unknown = [s for s in a.allow_skip if s not in ITEM_NAMES]
    if unknown:
        print(f"❌ --allow-skip 里有不认识的项：{unknown}；"
              f"可用项：{', '.join(ITEM_NAMES)}")
        return 2

    print("=" * 74)
    print("  board_gate —— 附着在**已经起着**的栈上，只读体检")
    print("  不发轨迹、默认不改任何状态；唯一的写路径在 --repair-controller 后面")
    print("=" * 74)

    domain = os.environ.get("ROS_DOMAIN_ID")
    print(f"  ROS_DOMAIN_ID = {domain!r}"
          + ("   ← ⚠️ 现场是 99；不是它的话图里会什么都看不到"
             if domain != "99" else ""))

    try:
        rclpy = _ros()
        from rclpy.node import Node
    except ImportError as exc:                             # noqa: BLE001
        print(f"❌ 这台机器上 import 不了 rclpy（{exc}）")
        print("   board_gate 需要 ROS 2 环境；板子上先 "
              "`source /opt/ros/humble/setup.bash`")
        return 2

    rclpy.init()
    node = Node(MY_NODE)
    repair_exercised = False
    try:
        probe = Probe(rclpy, node)
        graph, waited = _freeze_graph(probe, a.discover_timeout)
        print(f"  图快照（冻结于 {waited:.1f}s 处）：{len(graph['nodes'])} 个节点 / "
              f"{len(graph['topics'])} 个话题 / {len(graph['services'])} 个服务")
        if CTRL_SERVICE not in graph["services"]:
            print(f"  ⚠️ 没看到 {CTRL_SERVICE} —— 栈没起？还是 ROS_DOMAIN_ID "
                  f"不对（当前 {domain!r}）？")
        print()

        print("—— 逐项 ——")
        items = []

        ctrl = _item_controllers(a, probe)
        if ctrl.status == FAIL and a.repair_controller:
            repair_exercised = True
            if _run_repair(a, probe, _required_controllers(a)):
                ctrl = _recheck_controllers(a, probe)
            else:
                ctrl = Item("controllers", ctrl.status,
                            ctrl.detail + "（修复未走通或未获授权）",
                            ctrl.evidence)
        items.append(ctrl)
        _emit(ctrl, a.allow_skip)

        for fn in (_item_camera, _item_targets, _item_detector,
                   _item_telemetry, _item_params):
            it = fn(a, probe, graph)
            items.append(it)
            _emit(it, a.allow_skip)
        print()

        real = [it for it in items if not _is_skipped_ok(it, a.allow_skip)]
        n_pass = sum(1 for it in real if it.status == PASS)
        n_fail = sum(1 for it in real if it.status == FAIL)
        n_skip = sum(1 for it in real if it.status == SKIP)

        print("—— 结论 ——")
        print(f"  PASS {n_pass} / FAIL {n_fail} / SKIP {n_skip}"
              + (f"（另有 {len(a.allow_skip)} 项由 --allow-skip 声明缺席）"
                 if a.allow_skip else ""))
        print(f"  repair exercised: {'yes' if repair_exercised else 'no'}"
              + ("" if repair_exercised else
                 "   ← 没被绊倒时的绿是**空的**，别读成「验证过了」"))

        if n_fail:
            code = 1
            print("  结果：❌ 有 FAIL（看到了，而且它不对）")
        elif n_skip:
            code = 2
            print("  结果：⚠️ 没验全（有 SKIP 无 FAIL）—— **不算通过**，fail-closed")
        else:
            code = 0
            print("  结果：✅ 全绿")

        print("\n  ⚠️ 本闸门只回答「这一轮起栈健康吗」。它**不替代**「单步低速 + "
              "急停就绪的第一次真机动作」。")
        print("=" * 74)

        if a.json_report:
            _write_json(a.json_report, a, graph, items, code, repair_exercised,
                        domain)
        return code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
