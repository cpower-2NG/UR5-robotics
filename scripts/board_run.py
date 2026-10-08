#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务无关的真机启动器 —— 把「启动这条路」从**判断**变成**计算**（P0–P5）。

为什么要有它
============
本仓反复踩的启动失败里，有一类的形状是一样的：**已成文、却被重复违反**，
而且失效是**静默**的 ——
  · `pkill -f` 匹配到执行它的那条 shell 自己（台账自记「记过两次，这是第三次」）；
  · `planner_params_file` 不给 ⇒ 静默用仿真几何（`table_plane_z=-0.39`），臂不动；
  · `model:=` 解析不到 ⇒ detector 死掉而**整栈其余照跑**，现象同样是「臂不动」；
  · `k:=v` 拼错一个字母 ⇒ `ros2 launch` **不报错**，只是那个参数没生效；
  · 起栈与控制器激活差 93 s 还是 4.3 s，同一段代码一轮过、一轮不过。
⇒ 答案是把启动这条路写成计算：**每一道闸门都有判据、有实测值、有三态**。

它与既有脚本的关系（**只编排、只 import，不重写**）
==================================================
  · `board_lib.py`  —— 站点配置、残留进程、串口/夹爪/Dashboard 只读查询（前置层）；
  · `board_gate.py` —— 附着在**已起着**的栈上做只读体检。它**就是**本文件的 P3/P4：
                       原样以子进程调用、读它的 JSON 记录，**一行判定都不重抄**；
  · `board_a1.py`   —— 阶段 1 的**判分本体**（`grade()`）。本文件**不碰它**，
                       只按任务描述里的点号引用（`score.callable: "board_a1:grade"`）去调。
  · 它自己**不认识任何任务**：换场景 = 加一份 `config/tasks/<名>.yaml`，本文件一行不改。

启动器**永不**发 play / power / stop
===================================
按示教器 PLAY 是**人的动作**（AGENTS.md 红线 6）。P3 只**轮询只读** `dashboard()`
等 PLAYING。

两条写路径的授权从哪来（红线 6 点名了「控制器切换」与「初始化 / 开合」）
====================================================================
**两条**：`--repair-controller`（控制器切换）与 `--init-gripper`（夹爪寄存器）。
切换控制器是 `/controller_manager` 上的标准 ROS 2 服务 —— 不手搓 RTDE、不碰
30001–30004，所以它**不违红线 1**；夹爪走 Modbus RTU，是**独立设备**（不变量 3）。
两条的共同点：问题**不在「能不能发」，在「授权从哪来」**。三条落在这份文件里：

  ① **数据文件不许冒充授权**。任务描述里出现看着像授权的键 ⇒ `validate_task` 报红
     （`find_authorization_keys`）。授权也不来自环境变量 / 上一次的记忆 / shell
     别名 —— 红线 6 后半句「上一次的授权不覆盖这一次」禁的就是这个形状：一条会被
     复制、会被沿用的**数据**，不能冒充**一次**授权。
  ② **当次 + 被告知**。两个开关在**起栈之前**都要一个交互终端（P0.写路径授权 /
     P0.夹爪初始化授权；非交互 ⇒ FAIL、退 2，**栈都不起**，两条共用一个
     `judge_write_path`）；真写之前先把**完整报文**摆出来。控制器那份是服务名 /
     activate 列表 / `deactivate_controllers` 恒空 / strictness / timeout，确认
     动作在 `board_gate._run_repair` 里（**臂已上电**那一刻）；夹爪那份是**哪几个
     寄存器、走多少行程**，并**按读到的 `0x0200` 选最小的一档**（已初始化就只合拢，
     不重走全行程自检）—— 人确认的是**那一条**，所以两档印出来必须不是同一份。
  ③ **可复验**。切换执行过之后，按**只读复核**的结果判「已生效 / 已执行但未生效」
     （P3.控制器修复）。发出去的请求 ≠ 结果：未生效是 **FAIL**，不是绿。
     夹爪那一条对应的是 `gripper init exercised: yes/no` —— 而且它排在 P1 **之前**，
     于是 P1 那次只读读数（`0x0200==1`）说的就是**做完之后**的状态，一个读数同时
     回答「写成了吗」与「合规吗」，不需要第二处判定。

写路径的**本体**不在这份文件里，本文件原样透传 / 起子进程、**不重实现**：
  · 控制器切换 → `board_gate.py`（本文件里连那个服务的全名都不出现）；
  · 夹爪寄存器 → `scripts/ag95_init_close.py`（**全仓唯一**会驱动夹爪的脚本）。

待查（**别据此删掉写路径**）：本部署已经在跑 `ur_robot_driver/controller_stopper`。
  如果那次 STRICT 绊倒能在**配置源头**避免，控制器闸门就永远只需只读，
  `--repair-controller` 本可以退役。**这条线索没人核实过**，写在这里只是别让它丢了。

退出码（沿用全仓的 0/1/2，不加第四个）
=====================================
  0 = 闸门全过 **且**本任务判分通过（**只有真跑完一轮才可能拿到 0**）；
  1 = 闸门过了、任务也跑了，但**这一轮自己的判分没过**（「它跑了但没过」）；
  2 = 在**交棒之前**停下了（「我拒绝了」）：有 FAIL、有 SKIP（没验到）、
      或没给 `--authorize-motion`。**这一档不许读成通过。**

★ **两种跑法，都要支持（AI 驱动 / 人手动）**：这个入口**不是只给人用的**。
  `--authorize-motion` 那一轮要「当次授权」（红线 6），而**授权的证明可以有两种**，
  由有没有交互终端决定（纯函数 `handoff_mode()`）：

  · `keyboard`（有终端：人自己起的那一轮）—— 起栈后**敲一次 `yes`**。
  · `pendant`（没终端：**AI 驱动**的那一轮）—— 不敲键盘。证明改由
    **示教器上那次物理按压**承担：起栈**前**不是 PLAYING、起栈**后**是 PLAYING
    ⇒ 那一下**只可能是人按的**（`playing_is_vacuous()`）。同一个终端跑不了两种
    模式的旧版**直接把 AI 驱动堵死**，而 AI 恰恰不是终端。

  ⚠️ **pendant 不是「无人值守」**：它每一步都要求**人在示教器上动手**，
  只是不要键盘。上面那条物理证明**验不了**时（残 PLAYING / 起栈前读不到控制柜）
  就**退化成要求 STOP → PLAY 两步握手** —— 那同样只有真人做得到。

★ **交棒前复核「人在环」（只在给了 `--authorize-motion` 的那一轮）**：P3 的
  `P3.示教器 PLAYING` 可能是一条**空洞绿** —— 上一轮收栈之后控制柜**仍停在
  PLAYING**（第一百一十轮实测），于是启动器看到「PLAYING」就以为人按了，
  而其实**这一轮没人按**。不跑运动时那只是个记录问题；**要跑运动时它是动作问题：
  臂会动**。所以真交棒之前会让人**重按一次（先 STOP，再 PLAY）**，并且
  **两步都看到**才算数（只看一句「按一下」的话，两个动作挨得近时轮询可能一次都
  没看见 STOPPED ⇒ 判不了）。判据是纯函数 `needs_repress()`。
  ⚠️ 这一条**不下沉进 `decide()`**：放进去会把 **R0（无运动）**也拦在 P3，
  白丢 P4 的证据 —— 空洞绿该不该拦，取决于**下一步会不会动**。

⚠️ 离线（本机 WSL 没有 rclpy、没有板子）整跑**必然退 2** —— 那是**正确**的结果
   （fail-closed）。`--dry-run` 只看静态（只读文件、只拼命令，**不跑任何子进程**）；
   它退 0 **不代表**任何闸门验过，报告里会逐项标「声明缺席」。

⚠️ 退出码 2 的官方说法是「我拒绝了、什么都没发出去」。但 `start_delay` 是可配的：
   按播放按晚了的话，规划器可能**已经启动并动过臂**。本文件不把话说过头 ——
   停下时会把「起栈到停下」的秒数摆出来，并注明规划器定时器是不是已经到点
   （`judge_handoff_window`）。

用法:
    python3 scripts/board_run.py --task a1 --dry-run      # 只看会发什么命令、值从哪来
    python3 scripts/board_run.py --task a1                # 全流程（P5 还要 --authorize-motion）
    python3 scripts/board_run.py --task a1 --attach --log ~/a1_board.log
    python3 scripts/board_run.py --task a1 --init-gripper  # 先把夹爪弄到「已初始化」
    python3 scripts/board_run.py --stop                   # 收掉别人起的栈（不发运动命令）
    python3 scripts/board_run.py --help                   # 本文件是真相源，--help 即权威
"""

import sys

sys.dont_write_bytecode = True   # 本仓约定：防 .pyc 假绿。**必须在 import 本仓模块之前**

import argparse                  # noqa: E402
import contextlib                # noqa: E402
import datetime                  # noqa: E402
import importlib                 # noqa: E402
import json                      # noqa: E402
import os                        # noqa: E402
import re                        # noqa: E402
import shutil                    # noqa: E402
import signal                    # noqa: E402
import subprocess                # noqa: E402
import tempfile                  # noqa: E402
import time                      # noqa: E402
import types                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

# ⚠️ 本模块顶层**不 import rclpy / launch**。本机（WSL）的 /opt/ros/humble 下根本没装
#    rclpy，顶层一 import，`test_board_run.py` 连 import 都过不去 —— 而离线层是这套
#    东西**唯一**能成测的地方（`board_gate.py` 里有同样的先例与理由）。
import board_gate                                               # noqa: E402
import board_lib                                                # noqa: E402
# ⚠️ 导入它是为了**拿常量**（`FORCE_N` / 寄存器号），不是要在这里驱动夹爪 —— 它是
#    **全仓唯一**会写夹爪寄存器的脚本，报文里写的值必须与它实际发的**同源**
#    （同 `_required_controllers_for` 调 board_gate 私有函数的理由：抄一份会静默分叉，
#    而人确认的是**那一条**）。它顶层只 import argparse/inspect/sys/time，
#    pymodbus 是**函数内**才导的 ⇒ 本机（无 pymodbus、无 ROS）也 import 得动。
import ag95_init_close                                          # noqa: E402

PASS, FAIL, SKIP = board_gate.PASS, board_gate.FAIL, board_gate.SKIP
Item = board_gate.Item          # 复用三态项的载体，JSON 形状也一致

TASK_DIRS = ("~/.config/ur5-vision-grasp/tasks", "config/tasks")
TASK_SUFFIX = ".yaml"
GATE_SCRIPT = os.path.join(HERE, "board_gate.py")
DEPLOY_SCRIPT = os.path.join(HERE, "check_board_deploy.py")
TREE_SCRIPT = os.path.join(HERE, "board_tree_manifest.py")
INSTALL_SETUP = os.path.join(REPO, "ros2_ws", "install", "setup.bash")

S0 = "P0 自检（静态）"
S1 = "P1 前置（只读查询）"
S2 = "P2 起栈"
S3 = "P3 人在环 + 控制器闸门"
S4 = "P4 起栈后预检（全走 rclpy）"
S5 = "P5 交棒"

P1_NAMES = ("P1.残留进程", "P1.串口", "P1.AG-95 初始化", "P1.控制柜",
            "P1.部署前置", "P1.板子树", "P1.外参时效")
# `--attach` 假设了「栈是别人起的」⇒ 这两项量的是「起栈前该干净」，附着时不适用。
ATTACH_ABSENT = ("P1.残留进程", "P1.串口")
DRY_RUN_ABSENT_P1 = tuple(n for n in P1_NAMES if n != "P1.外参时效")

GATE_STAGE_ITEMS = {"P3": ("controllers",),
                    "P4": ("controllers", "camera", "targets", "detector",
                           "telemetry", "params")}

# 报告末尾必定印出来的退出码说明。这一行是这套东西的**语义本身**，不许改写。
EXIT_LEGEND = ("退出码：0 = 闸门全过**且**本任务判分通过；"
               "1 = 闸门过了、任务跑了，但**这一轮自己的判分没过**（「它跑了但没过」）；"
               "2 = 在**交棒之前**停下了（「我拒绝了」：有 FAIL / 有 SKIP / "
               "没给 --authorize-motion）")


# =====================================================================
# 任务描述（profile）—— 数据文件，不是代码里的 if-else
# =====================================================================

class TaskError(Exception):
    """任务描述读不到 / 形状不对。内容问题由 `validate_task` 逐条列出。"""


class CallableError(Exception):
    """点号引用解不出来。**绝不**允许被静默跳过 —— 跳过 = 不判分却报绿。"""


class GateError(Exception):
    """board_gate 的记录读不出来。**不许**把「读不出」当成「没有 FAIL」。"""


def dig(raw, dotted, default=None):
    """按 `a.b.c` 取值；任一段缺了就返回 `default`。**纯函数**。"""
    cur = raw
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


TASK_REQUIRED = ("task", "launch.entry", "launch.camera_source",
                 "expect.slots_total", "expect.min_candidates", "score.callable")

ENTRY_RX = re.compile(r"^[A-Za-z_]\w*/[A-Za-z_]\w*\.launch\.py$")

# 任务描述**只认**这几个顶层键。别的键一律报红 —— 加一个键要改代码，改了才有人
# 复核。这不是洁癖：见下面「授权不许来自数据文件」（AGENTS.md 红线 6）。
TASK_KNOWN_TOP = ("task", "launch", "params", "model", "start_delay",
                  "expect", "score")

# 看着像「允许我自动做某个写操作」的键名。命中就红 —— **哪怕它其实不会被读到**：
# 一份会被复制、会被沿用的数据里出现这种键，本身就是红线 6 要禁的形状。
AUTH_ISH_RX = re.compile(r"allow|authoriz|approv|consent|repair|unattend|"
                         r"auto|switch|bypass|force", re.I)

# 唯一的例外：`launch.allow_skip`。它说的是「这一档本来就没有这一项」（**缺席**），
# 不是「允许发写命令」。`params.*` **不**豁免 —— launch 参数是真能改变节点行为的
# 开关，`params.repair_controller: "true"` 正是要拦的那种东西。
AUTH_ISH_OK = ("launch.allow_skip",)


def task_dotted_paths(raw, _prefix=""):
    """把任务描述摊成**点号路径**列表（含中间节点）。**纯函数**。"""
    out = []
    if isinstance(raw, dict):
        for k, v in raw.items():
            p = "%s%s" % (_prefix, k)
            out.append(p)
            out.extend(task_dotted_paths(v, p + "."))
    return out


def find_authorization_keys(raw):
    """任务描述里有没有「看着在承载授权」的键。**纯函数** → `[(路径, 为什么)]`。

    空列表 = 干净。这不是风格检查：AGENTS.md 红线 6 要的是**当次**授权，它后半句
    「上一次的授权不覆盖这一次」针对的正是「把授权写进某份文件/某个变量，之后一直
    算数」。任务描述恰恰是最会被复制、被沿用的那种文件。
    """
    bad = []
    for p in task_dotted_paths(raw):
        if p in AUTH_ISH_OK:
            continue
        if AUTH_ISH_RX.search(p):
            bad.append((p, "任务描述是**数据文件**，会被复制、会被沿用 —— 授权"
                           "**不许**来自它（AGENTS.md 红线 6：「上一次的授权不覆盖"
                           "这一次」）。也不许来自环境变量或上一次的记忆。要动控制器"
                           "就用 `--repair-controller`：它先打印完整报文，再要你"
                           "**当次**在交互终端上确认。"))
    return bad


def split_entry(entry):
    """`yolo_bringup/bringup.launch.py` → `("yolo_bringup", "bringup.launch.py")`。

    形状不对 ⇒ 抛 `TaskError`（**不猜**）。这一条挡的是「入口写成
    `pipeline.launch.py`」那类老坑：真机只起 pipeline 时规划器解不出任何 IK，
    而且**不报错**。
    """
    s = str(entry or "").strip()
    if not ENTRY_RX.match(s):
        raise TaskError("launch.entry 要写成 `包名/文件.launch.py`，得到 %r" % (entry,))
    pkg, _, fname = s.partition("/")
    return pkg, fname


def parse_callable(ref):
    """`"board_a1:grade"` → `("board_a1", "grade")`。形状不对 ⇒ `CallableError`。"""
    s = str(ref or "").strip()
    if s.count(":") != 1:
        raise CallableError("点号引用要写成 `模块:属性`，得到 %r" % (ref,))
    mod, _, attr = s.partition(":")
    mod, attr = mod.strip(), attr.strip()
    if not re.match(r"^[A-Za-z_][\w.]*$", mod) or not re.match(r"^[A-Za-z_]\w*$", attr):
        raise CallableError("点号引用里模块/属性名不合法：%r" % (ref,))
    return mod, attr


def resolve_callable(ref, importer=None):
    """把点号引用变成**真的可调用对象**。解不出来 ⇒ 抛 `CallableError`。

    ⚠️ 这里**必须**报错而不是跳过：跳过就等于「没判分，但报告里都是绿的」——
    那正是本项目最常见的病（`SKIP` 不许渲染成绿色，同理）。
    `importer` 是给离线测试的注入口（默认 `importlib.import_module`）。
    """
    mod_name, attr = parse_callable(ref)
    importer = importlib.import_module if importer is None else importer
    try:
        mod = importer(mod_name)
    except Exception as exc:                       # noqa: BLE001
        raise CallableError("import 不了 %s（%s: %s）—— 判分**不会**被静默跳过，"
                            "也**不会**自动算通过"
                            % (mod_name, type(exc).__name__, exc))
    fn = getattr(mod, attr, None)
    if not callable(fn):
        have = ", ".join(sorted(a for a in dir(mod) if not a.startswith("_")))[:160]
        raise CallableError("%s.%s 不是一个可调用对象（%s 里有：%s）"
                            % (mod_name, attr, mod_name, have))
    return fn


def validate_task(raw):
    """逐条列出任务描述的毛病（**纯函数**；空 list = 没有毛病）。

    一次列全、并且**点名缺的是哪一个键**（点号路径）—— 现场来回改五行比来回改
    一行省事（与 `board_lib.validate_site` 同一条规矩）。
    """
    if not isinstance(raw, dict):
        return ["任务描述的顶层不是映射（是 %s）" % type(raw).__name__]
    p = []

    # ---- 红线 6：授权不许来自数据文件（先查这个，免得「能跑」盖过「不该这么写」）
    for k in sorted(raw):
        if k not in TASK_KNOWN_TOP:
            p.append("顶层有本启动器不认识的键 %r。任务描述只认这几项：%s —— 加键要"
                     "**改代码**（改代码才有人复核）；用一份会被复制、会被沿用的数据"
                     "承载行为/授权，是 AGENTS.md 红线 6 明令禁的形状"
                     % (k, "、".join(TASK_KNOWN_TOP)))
    for path, why in find_authorization_keys(raw):
        p.append("%s 看着在承载授权：%s" % (path, why))

    for k in TASK_REQUIRED:
        v = dig(raw, k)
        if v is None or (isinstance(v, str) and not v.strip()):
            p.append("缺 %s" % k)

    name = dig(raw, "task")
    if name is not None and not isinstance(name, str):
        p.append("task 要是名字（字符串），得到 %r" % (name,))

    try:
        split_entry(dig(raw, "launch.entry"))
    except TaskError as exc:
        p.append(str(exc))
    entries = dig(raw, "launch.declared_entries")
    if entries is not None:
        if not isinstance(entries, list) or not entries:
            p.append("launch.declared_entries 要给**非空列表**（不给就默认只看 entry）")
        else:
            for e in entries:
                try:
                    split_entry(e)
                except TaskError as exc:
                    p.append("launch.declared_entries 里有一项不合法：%s" % exc)

    for key in ("score.callable", "score.explain"):
        c = dig(raw, key)
        if c is None:
            continue
        try:
            parse_callable(c)
        except CallableError as exc:
            p.append("%s %s" % (key, exc))

    for k in ("expect.slots_total", "expect.min_candidates"):
        v = dig(raw, k)
        if v is None:
            continue
        if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
            p.append("%s 要是**正整数**，得到 %r" % (k, v))

    params = dig(raw, "params")
    if params is not None and not isinstance(params, dict):
        p.append("params 要是映射（键: 值），得到 %s" % type(params).__name__)
    elif isinstance(params, dict):
        for k, v in params.items():
            if isinstance(v, (dict, list)) or v is None:
                p.append("params.%s 只接受标量（launch 参数就是字符串），得到 %r"
                         % (k, v))

    a = dig(raw, "launch.allow_skip")
    if a is not None:
        if not isinstance(a, list):
            p.append("launch.allow_skip 要给列表")
        else:
            bad = [x for x in a if x not in board_gate.ITEM_NAMES]
            if bad:
                p.append("launch.allow_skip 里有不认识的项 %r；可用项：%s"
                         % (bad, ", ".join(board_gate.ITEM_NAMES)))
            if "controllers" in a:
                p.append("launch.allow_skip 不许含 controllers —— 它是闸门存在的理由")

    # 「这一档有没有规划器」在任务描述里被写了**两遍**，两遍必须一致：
    #   · `params.skip_planner`   —— **因**（`pipeline.launch.py:702` 的
    #     `UnlessCondition` 就是拿它挡 `planner_node` 的）；
    #   · `launch.allow_skip` 含 `telemetry`+`params` —— **果**（这两项只有规划器
    #     提供，`planner_absent()` 的判据）。
    # 两个判据在不同的地方吃劲（`pick_zone_arg()` 吃「果」、`planner_alive()` 吃「因」），
    # 所以不能只留一个；但**它俩一旦打架就是静默的**：声明了缺席而规划器其实活着
    # ⇒ `/grasp_planner/telemetry` 永远不会被验，规划器真坏了也看不出来。
    # 与其在两处各猜，不如在 P0 把这对不一致**当场拦下**（退 2，人来看）。
    if isinstance(params, dict):
        _sp = params.get("skip_planner")
        _cause = (_sp is not None
                  and str(_sp).strip().lower() in ("true", "1", "yes", "on"))
        _decl = planner_absent(a) if isinstance(a, list) else False
        if _cause != _decl:
            p.append(
                "`params.skip_planner`=%r（⇒ 规划器**%s**）与 `launch.allow_skip`=%r"
                "（⇒ 规划器**%s**）**互相矛盾** —— 同一个事实在任务描述里写了两遍，"
                "两遍必须同向。声明了缺席而规划器其实活着，会让 "
                "`/grasp_planner/telemetry` **永远不被验**（规划器坏了也看不出来）；"
                "反过来则会让 P4 恒定退 2。改到一致（见 `config/tasks/a1-noplan.yaml`）"
                % (_sp, "不起" if _cause else "会起", a,
                   "不起" if _decl else "会起"))
    return p


class Task(object):
    """一份任务描述（`config/tasks/<名>.yaml`）。取值一律经属性，缺项返回 None。"""

    __slots__ = ("raw", "path", "rel")

    def __init__(self, raw, path="<memory>"):
        self.raw = raw or {}
        self.path = path
        try:
            rel = os.path.relpath(path, REPO)
        except ValueError:                          # 不同盘符之类
            rel = path
        self.rel = path if rel.startswith("..") else rel

    def get(self, dotted, default=None):
        return dig(self.raw, dotted, default)

    @property
    def name(self):
        return str(self.get("task") or "?")

    @property
    def entry(self):
        return str(self.get("launch.entry") or "")

    @property
    def camera_source(self):
        return str(self.get("launch.camera_source") or "")

    @property
    def declared_entries(self):
        e = self.get("launch.declared_entries")
        return [str(x) for x in e] if isinstance(e, list) and e else [self.entry]

    @property
    def params(self):
        p = self.get("params")
        return {str(k): str(v) for k, v in p.items()} if isinstance(p, dict) else {}

    @property
    def model(self):
        m = self.get("model")
        return str(m).strip() if m is not None else None

    @property
    def start_delay(self):
        """规划器推迟启动的秒数（`start_delay:=`）。缺省 0 —— 与 launch 一致。

        ⚠️ 它同时是「起栈后留给人按播放的窗口」：`start_delay` 包的是
        [realsense, detector, perception, planner]，**不含 UR 驱动**。所以 A1 给 120
        的语义正是「驱动立刻起 → 给你两分钟按 PLAY → 规划器才启动」。
        """
        v = self.get("launch.start_delay")
        return 0.0 if v is None else float(v)

    @property
    def slots_total(self):
        return int(self.get("expect.slots_total"))

    @property
    def min_candidates(self):
        return int(self.get("expect.min_candidates"))

    @property
    def finish_marker(self):
        return str(self.get("expect.finish_marker") or "收工：")

    @property
    def timeout(self):
        v = self.get("expect.timeout")
        return None if v is None else float(v)

    @property
    def expect_params(self):
        p = self.get("expect.params")
        return {str(k): str(v) for k, v in p.items()} if isinstance(p, dict) else {}

    @property
    def trajectory_action(self):
        v = self.get("launch.trajectory_action")
        return str(v) if v else board_gate.DEFAULT_TRAJECTORY_ACTION

    @property
    def allow_skip(self):
        a = self.get("launch.allow_skip")
        return [str(x) for x in a] if isinstance(a, list) else []

    @property
    def score_ref(self):
        return str(self.get("score.callable") or "")

    @property
    def explain_ref(self):
        v = self.get("score.explain")
        return str(v) if v else None


def find_task_file(name, exists=os.path.exists):
    """`--task a1` → 真正要读的那份文件路径。**纯函数**（`exists` 可注入）。

    搜索顺序**现场真值优先**：`~/.config/ur5-vision-grasp/tasks/` 在前，仓内
    `config/tasks/` 在后。理由与站点配置同一条：**本仓是公开副本**，仓内那份是
    **模板/示例**，现场那份才是真值。
    名字里带 `/` 或 `.yaml` 的，当成显式路径。
    """
    s = str(name or "").strip()
    if not s:
        raise TaskError("没有给 --task")
    if "/" in s or s.endswith(TASK_SUFFIX):
        p = os.path.expanduser(s)
        if not exists(p):
            raise TaskError("任务描述读不到：%s" % p)
        return p
    cands = []
    for d in TASK_DIRS:
        base = os.path.expanduser(d)
        base = base if os.path.isabs(base) else os.path.join(REPO, base)
        cands.append(os.path.join(base, s + TASK_SUFFIX))
    for p in cands:
        if exists(p):
            return p
    raise TaskError("找不到任务描述 %r，找过：\n      %s"
                    % (s, "\n      ".join(cands)))


def load_task_file(path):
    """读任务描述。**yaml 在这里才 import** —— 本机有没有 PyYAML 都不影响
    `validate_task` 的离线测试（与 `board_lib.load_site` 同一条理由）。"""
    try:
        import yaml
    except ImportError as exc:                      # noqa: BLE001
        raise TaskError("缺 PyYAML（%s）—— pip install pyyaml" % exc)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except Exception as exc:                        # noqa: BLE001
        raise TaskError("%s 不是合法 YAML：%s" % (path, exc))
    return Task(raw, path=path)


# =====================================================================
# 参数拼装：能推导的不复述（站点给外参，任务给任务特有的）
# =====================================================================

# 站点才是权威、任务描述**不许**覆盖的键（覆盖了要当场报冲突，不静默后者赢）。
SITE_OWNED = ("robot_ip", "planner_params_file") + tuple(board_lib.CAM_KEYS)


# ---- 参数集的守卫：本文件发出的键 vs 旧入口 `board_a1.build_cmd` -------------
# 组装 launch 参数的地方**从一个变成了两个**（`board_a1.build_cmd` 与这里）。
# 旧守卫只钉住旧的那一个 ⇒ 新入口的参数集**完全没有守卫**，而少一个键
# （`publish_camera_tf` 就是这么漏掉的）在真机上是**静默**的。
# 所以把「两份的差集」变成**数据 + 判据**：
#   · 差集要逐项写出**为什么**（不许只列白名单）；
#   · 两个方向都查（少一个 / 多一个都要红）；
#   · 差集表**自己过时了也红**：某个键不再差 ⇒ 代码变了，理由得重写。
#     （理由会过期的名单比没有名单更坏 —— 它会让人以为已经查过了。）
# 判据只在 `test_board_run.py` 里跑：`board_a1` 是任务特有的，本文件**不许**
# 顶层 import 它（那是「本文件不认识任何任务」这条不变量）。
ENTRY_ONLY_OK = {
    "tf_required":
        "bringup.launch.py:227-232 按 camera_source 推导：`'false' if "
        "camera_source == 'sim' else 'true'` ⇒ 真机入口（realsense）解析成 true。"
        "复述一份反而多一处会跟它打架的真值。",
}
LAUNCHED_ONLY_OK = {
    "model":
        "旧入口靠 launch 的默认值给 model（解析不到时 detector 死掉而整栈其余"
        "照跑）；本启动器把文件名写进任务描述，显式优于默认（台账 D67）。",
    "plane_depth_tol":
        "平面闸门比的是**绝对** base z，而俯视 D435i 的深度**系统性读远 ~1.9%**"
        "（2026-10-07 量定：靶纸 PnP +1.96%、臂定标相机高度 +3.45%）。相机在 "
        "830 mm 高处时台面被重建到 −30.0 mm，8 个真方块的残差是 30–52 mm ⇒ 默认的 "
        "30 mm 把真方块**全判死**（`/targets_3d` 塌成 1），而那个一直在的假阳性框是 "
        "258–264 mm。放宽到 0.06 实测 2006/2006 批「发布 8 个候选」、假阳性仍被挡。"
        "旧入口没有这个键，是因为它写死在 **2026-10-06 相机抬高之前**的几何"
        "（H≈630 mm ⇒ 绝对误差 ~12 mm，默认容差刚好够）—— 不是它不需要，是它当时"
        "碰巧没越线。⚠️ 它随相机高度线性变，属**任务+现场**参数：相机一动必须重测"
        "（见 RESTRUCTURE_PLAN 第一百零八轮）。只有显式传了它的 profile 才算「多发」，"
        "别的 profile 不带它时本表不适用（本判据当前只在 a1 这条路径上跑）。",
    "grasp_point_mode":
        "检测框框的是方块在图像上的**整个侧影**，不是顶面。斜视 α 角下侧影中心比"
        "顶面中心低 (S·sinα·f)/(2D) 像素 —— 2026-10-08 在真帧上量定：这帧 6.44 px "
        "≈ base 方向 13.7 mm，而方块才 28.5 mm 宽，夹爪正好咬在顶面靠相机那一侧。"
        "加这个键把它修回来（两棱中点法，不需要知道方块多大或相机多斜）。"
        "旧入口没有它，是因为它写死在**修这条之前** —— 不是不需要，是当时没人量过。"
        "⚠️ 只在 `use_target_plane=true` 时有意义：深度模式高度未知，两条棱所在的"
        "平面无从谈起（那时本键会被忽略，不报错）。",
    "table_plane_z":
        "`grasp_point_mode=top_face_center` 要用到的台面高度（近端下棱一定落在"
        "台面上）。它必须**小于** target_plane_z，写反了 projector_node 会在构造时"
        "记 ERROR（不静默）。默认 0.0 与 target_plane_z=0.0285「台面 + 方块高」的"
        "口径一致。旧入口没有它，同上：它那时没有会用到它的那条路。",
}


def judge_entry_parity(launched_keys, entry_keys,
                       launched_only=None, entry_only=None):
    """「本启动器发出去的键」vs「旧入口 `board_a1.build_cmd` 发的键」。**纯函数**。

    → `(status, detail)`。等价 ⇒ PASS；差集**逐项有理由**且**没有过时项** ⇒ PASS
    （detail 里把每一条理由摆出来）；否则 FAIL 并点名是哪个键、哪个方向。
    """
    launched_only = LAUNCHED_ONLY_OK if launched_only is None else launched_only
    entry_only = ENTRY_ONLY_OK if entry_only is None else entry_only
    launched, entry = set(launched_keys), set(entry_keys)

    missing = sorted(entry - launched)          # 本文件**少**发的
    extra = sorted(launched - entry)            # 本文件**多**发的
    stale = sorted([k for k in entry_only if k not in missing]
                   + [k for k in launched_only if k not in extra])

    bad, why = [], []
    for k in missing:
        if k in entry_only:
            why.append("少发 %s：%s" % (k, entry_only[k]))
        else:
            bad.append("**少发** %s —— 旧入口发了，本文件没发，而且没有理由"
                       "（真机上大概是静默失效）" % k)
    for k in extra:
        if k in launched_only:
            why.append("多发 %s：%s" % (k, launched_only[k]))
        else:
            bad.append("**多发** %s —— 本文件发了，旧入口没有，而且没有理由"
                       "（先确认不是手滑加的）" % k)
    for k in stale:
        bad.append("差集表里的 %s 已经**不再差**了 —— 代码变了，这条理由得重写"
                   "（留着一张永远对的白名单比没有更坏）" % k)

    if bad:
        return FAIL, ("%d 处对不上：\n" % len(bad)
                      + "\n".join("      · %s" % x for x in bad))
    head = "两份参数集：本文件 %d 个键 / 旧入口 %d 个键" % (len(launched), len(entry))
    if not why:
        return PASS, head + " —— **完全一致**"
    return PASS, head + "，差集逐项有理由：\n" + "\n".join(
        "      · %s" % x for x in why)


def _put(params, src, k, v, s):
    """空值不落进参数集（`None` / 空串 = 「这一项没人给」）。**纯函数**。"""
    if v is None or str(v).strip() == "":
        return
    params[k] = str(v)
    src[k] = s


def task_params(task):
    """**任务描述单独**能定的参数 → `({"键": "值"}, {"键": 来源})`。**纯函数**。

    拆出来是为了「站点配置读不到时也能把**已经知道的那部分**摆给人看」——
    否则披露完备性会在**最需要它**的那种失败上瞎掉（见 `_print_plan`）。
    """
    params, src = {}, {}
    tsrc = "任务描述 %s" % task.rel
    _put(params, src, "camera_source", task.camera_source,
         "%s:launch.camera_source" % tsrc)
    _put(params, src, "model", task.model, "%s:model" % tsrc)
    for k, v in task.params.items():
        _put(params, src, k, v, "%s:params.%s" % (tsrc, k))
    if task.get("launch.start_delay") is not None:
        _put(params, src, "start_delay", "%g" % task.start_delay,
             "%s:launch.start_delay" % tsrc)
    return params, src


def _quat_is_zero(params):
    """四元数四项是不是都是 0（= 不是一个合法的旋转）。**纯函数**。

    与 `real.launch.py` 里 `publish_camera_tf:=true` 时那道自检**同一个判据**
    （它用的是 `_is_zero`：|v| < 1e-12）；这里只判「站点那一份够不够格开发布器」。
    """
    q = [params.get(k) for k in ("camera_qx", "camera_qy", "camera_qz", "camera_qw")]
    if any(v is None for v in q):
        return True                      # 缺项也算「不够格」——别开
    try:
        return all(abs(float(v)) < 1e-12 for v in q)
    except ValueError:
        return True                      # 不是数（validate_site 会另外报）


def resolve_launch_params(task, site):
    """→ `({"键": "值"}, {"键": 来源}, [冲突...])`。**纯函数**。

    来源逐项记下来，因为**有效值 = 站点 ⊕ 任务**，而现场最需要知道的就是「这一个值
    是谁给的」——「传了标定」不等于「两边都传了」（AGENTS.md §1 的两套几何）。
    """
    params, src = task_params(task)
    conflicts = []

    ssrc = "站点 %s" % site.path
    _put(params, src, "robot_ip", site.robot_ip, "%s:robot_ip（或 $UR_ROBOT_IP）" % ssrc)
    _put(params, src, "planner_params_file", site.planner_params,
         "%s:planner_params" % ssrc)
    ex = site.extrinsics()
    for k, v in ex.items():
        _put(params, src, k, v, "%s:topcam.extrinsics" % ssrc)

    # ---- 相机 TF 发布器：**由外参推导**，不写进任务描述 ----
    # `real.launch.py` 的 `publish_camera_tf` 默认是 `"false"`，而它的判据是
    # `launch_configurations.get("publish_camera_tf", "false") == "true"`
    # —— **不传就等于 false**，launch 于是假定你有**外部** TF 源。后果是本站
    # 发出去的那七个 `camera_*` 外参**没有任何东西消费**：外参是死的，TF 链
    # 不存在，`tf_required` 解析为 true 之后 perception_3d 直接退出，而整栈其余
    # 照跑 —— 正是本启动器要消灭的那类**静默失败**。
    # 「站点给了外参 ⇒ 打开消费它们的发布器」是能推导的，所以不复述进 profile；
    # 任务描述若**显式**写了 `params.publish_camera_tf`，以它为准（那是「我有
    # 外部 TF 源」这类刻意选择，得让人说得出口）。
    # 站点一个外参都没给 ⇒ 没有「消费外参」这回事：缺项由 `validate_site` 在
    # P0.站点配置 里报，这里不重复喊（也**不**擅自打开发布器去撞 real.launch.py
    # 的自检）。
    if "publish_camera_tf" not in params and ex:
        if _quat_is_zero(params):
            conflicts.append(
                "站点给了外参，但四元数（camera_qx..qw）不能用（**全零或缺项**）"
                "—— 全零不是合法旋转（模长 0），缺项同理。因此**不打开**相机 TF "
                "发布器（打开的话 real.launch.py 会当场抛错退出），本站也就没有"
                "相机 TF 链。把标定值填进站点配置的 topcam.extrinsics；确知相机与"
                "基座重合时才显式传 camera_qw:=1.0")
        else:
            _put(params, src, "publish_camera_tf", "true",
                 "推导：站点给了相机外参（%s:topcam.extrinsics）⇒ 打开消费它们的"
                 "发布器；不给就是 launch 默认的 false（= 假定你有外部 TF 源，"
                 "本站那七个外参没人用）" % ssrc)

    # 站点权威项：任务描述里也写了同名键 ⇒ 记冲突（**不静默让后者赢**）
    for k in SITE_OWNED:
        v = task.params.get(k)
        if v is not None and str(v) != params.get(k):
            conflicts.append(
                "%s：任务描述写 %r，站点给 %r —— 站点是权威，任务描述里这一项"
                "**不会生效**，请把它从 params 里删掉"
                % (k, v, params.get(k)))
    return params, src, conflicts


def params_to_argv(params):
    """`{k: v}` → `["k:=v", ...]`（**按键排序**：命令要可复现、可 diff）。"""
    return ["%s:=%s" % (k, params[k]) for k in sorted(params)]


def build_launch_cmd(entry, params):
    """→ 完整的 `ros2 launch` 命令（**纯函数**）。`entry` 形状不对 ⇒ `TaskError`。"""
    pkg, fname = split_entry(entry)
    return ["ros2", "launch", pkg, fname] + params_to_argv(params)


# ---- 声明集：拼错的 `k:=v` 不报错，只静默跑成另一回事 ---------------------

DECLARED_RX = re.compile(r"^\s*'([A-Za-z_]\w*)':\s*$")


def parse_declared_args(text):
    """解析 `ros2 launch <入口> --show-args` 的输出 → 声明过的参数名集合。
    **纯函数**（本机没有 ROS，跑不了真的 launch；离线测的就是这个解析器）。

    `--show-args` 的形状是每条声明一行 `    '名字':`，下面缩进的是说明与默认值。
    只认「整行就是一个带引号的名字加冒号」——说明文字里提到别的参数名**不会**被吃
    进来（`test_board_run.py` 里有诱饵行钉这一条）。
    """
    out = set()
    for ln in str(text or "").splitlines():
        m = DECLARED_RX.match(ln)
        if m:
            out.add(m.group(1))
    return out


def declared_cmds(entries):
    """要跑哪几条 `--show-args`（**纯函数**）。

    ⚠️ **为什么不是只问入口**：本仓的真机细分项（`robot_ip` / `camera_*` /
    `planner_params_file` / `use_target_plane` / `target_plane_z` …）**故意不在
    `bringup.launch.py` 里声明** —— 它们靠 `launch_configurations` 直通给
    `real.launch.py` / `pipeline.launch.py`（见 `bringup.launch.py` 文件头：在那儿
    声明过会把**仿真路径**也一起改掉，且改得静默）。所以只问入口，会把**正确的**参数
    全报成拼错。任务描述里的 `launch.declared_entries` 就是「这几份文件的声明集
    合起来算数」。
    """
    out = []
    for e in entries:
        pkg, fname = split_entry(e)
        out.append(["ros2", "launch", pkg, fname, "--show-args"])
    return out


def check_declared(param_keys, declared):
    """逐个断言 `k:=v` 的键都在声明集里。**纯函数** → `(三态, 说明)`。

    为什么值得单列一关：`ros2 launch` 对**未声明**的参数**不报错**，只是把它塞进
    `launch_configurations` 就完事 —— 于是 `target_plane_zz:=0.0285`（拼错一个字母）
    的现象是「命令成功、日志干净、那个高度没生效」。拼错不报错、只静默跑成另一回事，
    正是本仓最贵的一类故障。
    """
    if not declared:
        return SKIP, ("拿不到声明集（一条 `--show-args` 都没跑成）—— **没验到**，"
                      "不算通过")
    unknown = sorted(k for k in param_keys if k not in declared)
    if unknown:
        return FAIL, ("%d 个键**不在声明集里**：%s（声明集共 %d 个）—— 拼错一个字母"
                      "不会报错，只会**静默不生效**；先核对名字，再核对是不是该在 "
                      "launch.declared_entries 里补一份入口"
                      % (len(unknown), ", ".join(unknown), len(declared)))
    return PASS, "%d 个键都在声明集里（声明集共 %d 个）" % (len(param_keys),
                                                            len(declared))


# ---- 有效参数：yaml ⊕ launch，逐项注明来源 -------------------------------

DEAD_KEYS_NOTE = "死键：全车入口下被 pipeline 的覆盖表盖掉，yaml 里写的不是有效值"


def effective_params(yaml_vals, launch_vals, keys,
                     dead_keys=board_gate.DEAD_KEYS):
    """→ `{键: (值, 来源)}`。**纯函数**。

    规矩（`docs/机械臂系统参考.md` §5.5）：`grasp_planner.yaml` 有 **5 个死键**，
    在整车入口下改了**不生效也不报错**。所以「这一个参数现在到底是多少」不能只看
    yaml，得看 yaml ⊕ launch —— 而这两处都只是**候选**，真值只有起栈后回读
    （`board_gate` 的 `params` 项，P4 会做）能给。

    来源四种：`launch 命令行` / `planner yaml` / `planner yaml（⚠️ 死键…）` / `未设置`
    """
    out = {}
    for k in keys:
        if k in launch_vals:
            out[k] = (launch_vals[k], "launch 命令行（**覆盖** yaml）")
        elif k in yaml_vals:
            note = ("（⚠️ %s）" % DEAD_KEYS_NOTE) if k in dead_keys else ""
            out[k] = (yaml_vals[k], "planner yaml" + note)
        else:
            out[k] = (None, "未设置（节点自己的默认值）")
    return out


def planner_absent(allow_skip):
    """这一档是不是**没有规划器**。**纯函数** → `bool`。

    判据用任务描述里**已经声明过的事实**，不另猜：`launch.allow_skip` 里同时含
    `telemetry` 与 `params` —— 这两项**只有规划器提供**（`/grasp_planner/telemetry`、
    `/grasp_planner/get_parameters`；`board_gate.ITEM_NAMES` 里的取法同此）。
    两项都声明缺席 ⇔ 本档没有规划器。
    """
    return {"telemetry", "params"} <= {str(x) for x in (allow_skip or ())}


def pick_zone_arg(allow_skip, yaml_vals, launch_vals=()):
    """该不该把 `pick_zone` **递给闸门** → 6 个数 | `None`。**纯函数**。

    ⚠️ 为什么**只在规划器不起的档**才递（2026-10-06 用户拍板）：
    `board_gate` 里 `--pick-zone` **优先于活读** —— 给了它，`zone_src` 会被写成
    「**来自 --pick-zone，不是活读**」。规划器活着时，「活读运行中的规划器」是
    **更强**的判据：它验的是**实际生效**的值，而不是我们手里那份 yaml。
    总递等于把 R1 的判据**悄悄降级**成静态值。

    可规划器不起时（R0：`skip_planner:=true`）活读必然拿不到 ⇒ `P4.targets`
    **结构性恒 SKIP**。而 `config/tasks/a1-noplan.yaml` 开头写明这一档存在的意义
    正是「`/targets_3d` 候选数仍然验得到（否则 P4 一多半都是空的）」——
    2026-10-06 真机第 1 次实测到的就是这个恒 SKIP。

    递下去的是**有效值**（yaml ⊕ launch），与 P0「参数集」印的是**同一套算法**。
    另有两道自我约束：`pick_zone` 若是**死键**就不递（那时 yaml 那个值**不生效**，
    拿它当判据是**反向**的错，比不判更坏）；值形状不是 6 个数也不递（交给闸门
    自己去活读/报 SKIP，不拿半份去撞它的自检）。
    """
    if not planner_absent(allow_skip):
        return None
    if "pick_zone" in board_gate.DEAD_KEYS:
        return None
    v = effective_params(dict(yaml_vals or {}), dict(launch_vals or {}),
                         ("pick_zone",))["pick_zone"][0]
    if v is None:
        return None
    try:
        vals = [float(x) for x in v]
    except (TypeError, ValueError):
        return None
    return vals if len(vals) == 6 else None


PLANNER_KEYS_OF_INTEREST = (
    "pick_zone", "place_slots", "place_z", "target_frame", "trajectory_action",
    "ik_group", "gripper_backend", "carry_objects", "table_scene_enabled",
    "table_plane_z",
)


def planner_yaml_params(raw):
    """从 `grasp_planner.yaml` 的已解析内容里取出 `ros__parameters` 那一层。
    **纯函数**。取不到返回 `None`（调用方按「读不出」处置，不假装读到）。"""
    if not isinstance(raw, dict):
        return None
    node = raw.get("grasp_planner")
    if not isinstance(node, dict):
        return None
    p = node.get("ros__parameters")
    return p if isinstance(p, dict) else None


def read_planner_yaml(path):
    """读 `planner_params_file` 的 `ros__parameters`。返回 `(值字典|None, 原因)`。"""
    if not path:
        return None, "站点没给 planner_params"
    p = os.path.expanduser(str(path))
    if not os.path.exists(p):
        return None, "文件不存在"
    try:
        import yaml
        with open(p, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except Exception as exc:                        # noqa: BLE001
        return None, "%s: %s" % (type(exc).__name__, exc)
    vals = planner_yaml_params(raw)
    if vals is None:
        return None, "里面没有 grasp_planner: ros__parameters: 这一层"
    return vals, ""


# =====================================================================
# 判定：全部纯函数（不 import ROS、不碰设备，见 test_board_run.py）
# =====================================================================

#: 系统 ROS 的 setup.bash（按 ROS_DISTRO 填）。
ROS_SETUP_TMPL = "/opt/ros/%s/setup.bash"


def ros_setup_files(distro=None, install_setup=None, repo=REPO, system_setup=None):
    """本项目要 source 的两个 setup.bash —— **顺序有意义**。**纯函数**。

    先系统、后工作区：后 source 的那个在 `AMENT_PREFIX_PATH` 前面，工作区的包
    才盖得住同名的系统包。反过来会把工作区的包压在下面（**不报错**，只是找不到）。
    """
    return [system_setup or (ROS_SETUP_TMPL % (distro or "humble")),
            install_setup or os.path.join(repo, "ros2_ws", "install", "setup.bash")]


def build_ros_env(base=None, distro=None, install_setup=None, repo=REPO,
                  system_setup=None, timeout=120.0):
    """把 ROS 环境**算出来**：启动器自己 source 系统与工作区的 setup.bash。

    返回 `(env | None, 说明)`。`None` = 没算成（bash 起不动 / source 报错），
    交给 P0 判红 —— **不猜**，也不退回调用者的环境假装没事。

    🔴 为什么必须自己 source（2026-10-06 实测）：板上 `~/.bashrc` 只 source 了
    `/opt/ros/humble` 与**另一个**工作区 `~/yahboom_ws`，于是**交互式** shell 里
    本项目的 `ros2_ws/install` **也不在** `AMENT_PREFIX_PATH` 上。而旧 P0 判据只是
    「`AMENT_PREFIX_PATH` 非空 + `install/setup.bash` 文件存在」——两条都满足，
    于是 P0 绿、失败推迟到 P1，报成「缺 yolo_ur_description」，**听起来像缺包、
    其实是没 source**。人每次开终端都重演一次，正是「启动老是卡住」的一类根因。
    """
    e0 = dict(os.environ)
    if base:
        e0.update(base)
    files = ros_setup_files(distro or e0.get("ROS_DISTRO"),
                            install_setup, repo, system_setup)
    have = [f for f in files if os.path.exists(f)]
    missing = [f for f in files if f not in have]
    script = "".join('source "%s" >/dev/null 2>&1\n' % f for f in have) + "env -0"
    try:
        r = subprocess.run(["bash", "-c", script], cwd=repo, env=e0,
                           capture_output=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:     # noqa: BLE001
        return None, "起不动 bash：%s: %s" % (type(exc).__name__, exc)
    if r.returncode != 0:
        tail = (r.stderr or b"").decode("utf-8", "replace").strip()[-200:]
        return None, "source 报错（rc=%d）：%s" % (r.returncode, tail or "无 stderr")
    out = {}
    for chunk in r.stdout.split(b"\x00"):
        if not chunk:
            continue
        k, _, v = chunk.partition(b"=")
        out[k.decode("utf-8", "replace")] = v.decode("utf-8", "replace")
    if not out:
        return None, "`env -0` 没输出 —— 起的可能不是 bash"
    note = "source 了 " + ("、".join(have) if have else "（一个都没有）")
    if missing:
        note += "；**不在**：" + "、".join(missing) + "（多半没 colcon build）"
    return out, note


def judge_ros_env(env, ros2_on_path, install_setup, sourced="", repo=REPO):
    """P0：**算出来的**环境能不能跑本项目。**纯函数**。

    ⚠️ 判据是**行为**，不是代理指标（2026-10-06 修）：必须**真的**在
    `AMENT_PREFIX_PATH` 里看到本项目工作区的 install 前缀。旧判据只是
    「非空 + setup.bash 文件存在」—— 板上 `.bashrc` 恰好只 source 了**另一个**
    工作区，于是旧判据绿、P1 红，而红的话说的是「缺包」。同一种病：
    **判据报的是它自己的主题，不是你关心的那个行为。**
    """
    distro = str(env.get("ROS_DISTRO") or "").strip()
    ament = str(env.get("AMENT_PREFIX_PATH") or "").strip()
    if not ros2_on_path:
        return FAIL, ("PATH 里没有 `ros2` —— 启动器会自己 source %s；看到这条说明"
                      "它没 source 成，看下面「来源」那半句"
                      % (ROS_SETUP_TMPL % "humble"))
    if not distro:
        return FAIL, "没有 ROS_DISTRO —— 环境没 source 起来"
    if not ament:
        return FAIL, "AMENT_PREFIX_PATH 是空的 —— 环境没 source 全"
    if not install_setup or not os.path.exists(install_setup):
        return FAIL, ("%s 不在 —— 板子上那份检出**必须先 colcon build**"
                      "（install/ 指向 build/，不编就等于跑旧代码）"
                      % (install_setup or "ros2_ws/install/setup.bash"))
    want = os.path.normpath(os.path.dirname(os.path.abspath(install_setup)))
    have = [e for e in ament.split(os.pathsep) if e.strip()]
    norm = [os.path.normpath(os.path.expanduser(e)) for e in have]
    # ⚠️ 判据是「**这棵工作区**在贡献」，**不是**「install 目录本身在列表里」：
    #    colcon 的**隔离式**安装（板上就是）放进 AMENT_PREFIX_PATH 的是
    #    `<ws>/install/<包名>` 一条条，**没有** `<ws>/install` 本身。
    #    2026-10-06 板上实测把这条抓了出来 —— 我第一版写的正是「目录本身在
    #    列表里」，而离线夹具里的假 setup.bash 恰好导出裸前缀，于是测试全绿、
    #    真机恒红。**夹具跟现场长得不一样，绿就是假的。**
    if not any(e == want or e.startswith(want + os.sep) for e in norm):
        return FAIL, ("AMENT_PREFIX_PATH 里**没有**本项目工作区（%s）贡献的任何一条"
                      "—— 现在是：%s\n      ⇒ 多半 source 到了**别的**工作区"
                      "（板上 `.bashrc` 就是 source 了 ~/yahboom_ws），"
                      "或者这份检出没 colcon build" % (want, have or "（空）"))
    return PASS, ("ROS_DISTRO=%s，ros2 在 PATH 里，工作区 install 在搜索路径上"
                  "（%s）" % (distro, sourced or "来源未记"))


def judge_domain(env, site):
    """P0：`ROS_DOMAIN_ID` 与站点**冲不冲突**。**纯函数**。

    ⚠️ 判据 2026-10-06 改过（用户拍板「按启动器方便的来」）：**没设不再算失败**。
    理由：启动器在 P2 **自己**会把站点的域号注入它起的每一个子进程，所以栈
    无论如何都跑在站点那个域上 —— 旧判据拦的是一个**它自己就会修**的条件，
    代价是「每开一个新终端都得记得 export」，正是要消灭的那类摩擦。

    只在**两边冲突**时才红：那时你**自己另开**的终端在另一个 DDS 域上，会看见
    「栈明明起着、话题列表却是空的」—— 本仓实测踩过，且极易误判成「栈没起来」。
    无论哪种情况都把**值从哪来**印清楚（要报告来源，不要只报结论）。
    """
    want = str(site.ros_domain_id)
    got = env.get("ROS_DOMAIN_ID")
    if got is None or str(got).strip() == "":
        return PASS, ("环境里**没设** —— 启动器按站点用 %s（只为它起的子进程注入）。"
                      "⚠️ 你**自己另开**的终端要 `export ROS_DOMAIN_ID=%s` 才看得见话题"
                      % (want, want))
    if str(got).strip() != want:
        return FAIL, ("你的 shell 是 %s，站点是 %s —— 启动器会按**站点**用 %s，"
                      "于是你这个终端在**另一个** DDS 域上，会看见「栈起着、话题却是空的」。"
                      "先 `export ROS_DOMAIN_ID=%s`（或把站点改成 %s）"
                      % (got, want, want, want, got))
    return PASS, "ROS_DOMAIN_ID=%s（与站点一致）" % want


def judge_score_refs(task, resolver=None):
    """P0：任务描述里的判分函数**现在就能解析**吗。**纯函数**（`resolver` 可注入）。

    为什么要放到起栈**之前**判：判分器失效的方式是「看起来一切正常」—— 点号引用写错、
    模块搬了家、函数改了名，都不会报错，只会**没判分**。放到 P5 才发现，代价是
    一整轮真机（十几分钟 + 一场空）。所以这里在**静态**阶段先解析一次。
    """
    resolver = resolve_callable if resolver is None else resolver
    bad = []
    for label, ref in (("score.callable", task.score_ref),
                       ("score.explain", task.explain_ref)):
        if not ref:
            continue
        try:
            resolver(ref)
        except CallableError as exc:
            bad.append("%s：%s" % (label, exc))
    if bad:
        return FAIL, ("判分函数解析不出来（**不会**静默跳过判分，也**不会**自动算通过）："
                      "\n" + "\n".join("      " + x for x in bad))
    return PASS, "点号引用解析得到：%s%s" % (
        task.score_ref, (" / %s" % task.explain_ref) if task.explain_ref else "")


def judge_problems(what, problems, path=""):
    """把一串毛病包成一个三态项（**纯函数**）。空 list ⇒ PASS。"""
    if not problems:
        return PASS, "逐项合法" + (("（%s）" % path) if path else "")
    head = "%d 条毛病" % len(problems) + (("（%s）" % path) if path else "")
    return FAIL, head + "：\n" + "\n".join(
        "      %d. %s" % (i, x) for i, x in enumerate(problems, 1))


def is_playing(d):
    """示教器在 PLAYING 吗。**薄转发** —— 判定实现在 `board_lib.is_playing`。

    ⚠️ 这里从「本地精确相等」改成了转发（2026-10-06）。原来的
    `str(d["programState"]).strip() == "PLAYING"` 是**恒 False**：
    实测原文是 `'STOPPED ur5ros.urp'`，播放时是 `'PLAYING ur5ros.urp'`
    —— 状态后面缀着**当前载入的程序名**，与 `'PLAYING'` 永不相等。
    后果不是「误报」，是 P3 **永远等不到播放**、300 s 超时退 2，
    而现场表现是「人明明按了播放」。全文见 `board_lib` 里那节实测记录。
    """
    return board_lib.is_playing(d)


def judge_cabinet(d):
    """P1：控制柜连得上、且 `robotmode == RUNNING`。**纯函数**。

    ⚠️ 这里**不要求** PLAYING。次序是「**起栈 → 按播放 → 拉回控制器**」：URCap 的
    External Control 要先有对端，先按播放再起栈，示教器上会报「连接被拒绝」、程序
    停在 STOPPED。等播放的逻辑在 P3。

    ⚠️ 判据走 `board_lib.dash_mode`（剥标签），**不是**与原文精确比：UR 的回复
    自带标签（实测 `'Robotmode: RUNNING'`），精确比在**任何正常控制柜上恒红**
    （2026-10-06 修，当时它正是 R0 卡在 P1 的三条之一）。
    """
    if d is None:
        return FAIL, ("连不上控制柜 29999（只读查询 robotmode / programState / "
                      "safetystatus；**不发** play/power/stop）")
    raw = str(d.get("robotmode", "")).strip()
    mode = board_lib.dash_mode(d)
    if mode != "RUNNING":
        return FAIL, ("robotmode 原文 %r 读成 %r，不是 'RUNNING' —— 控制柜没上电 / "
                      "没使能 / 还没连上驱动？（原文带 'Robotmode: ' 标签是**正常**的，"
                      "由 board_lib.dash_mode 剥掉，别改回去精确比）" % (raw, mode))
    return PASS, ("%s | %s | %s（此刻**不要求** PLAYING：次序是 "
                  "起栈→按播放→拉回控制器）"
                  % (mode, board_lib.dash_program_state(d),
                     board_lib.dash_field(d.get("safetystatus"))))


def residual_playing_note(cabinet):
    """`P3.示教器 PLAYING` 通过时**追加**的那句话：这算不算「人按的」。**纯函数** → `str`。

    ⚠️ **为什么要有这一条**（2026-10-06 用户裁决 **(b)**）：启动器收栈收的是**这套栈**，
    **不是**控制柜里的程序 —— 收栈之后示教器常常仍停在 `PLAYING`。那样的话下一轮起栈时
    P3 的轮询**第一次就命中**，报告里照样是一条 PASS，读起来像「人在环走通了」，
    **而其实什么都没走**（人根本没动手）。第 2 次 R0 实测正是如此：
    `[PASS] P3.示教器 PLAYING  0s 后 PLAYING`，而起栈**之前** P1 读到的已经是
    `PLAYING ur5ros.urp`。

    **用户选的是「说清 ＋ 放行」，不是「拒」**：这样一轮仍然把 P4 的体检跑完了，
    拒掉反而什么都没验到。所以这一条**不改三态**，只把话说实。

    ⚠️ 与「按得快」**无关**：判据只看**起栈之前**那个值，不看等了多久。所以
    「起栈后 20 s 人按的」照样算人按的（第 2b 轮实测 `24s 后 PLAYING`）——
    拿 `waited` 当判据会把「手快」误判成「残状态」，那正是缺陷丙的同一个形状。
    """
    if cabinet is None:
        return ("；⚠️ 起栈前**没读到**示教器状态（Dashboard 没答话）—— "
                "这一条「是不是人按的」**未判**")
    if board_lib.is_playing(cabinet):
        return ("；⚠️ **但这一条不算「人在环走通了」**：起栈**之前**它就已经是 %s 了"
                "（上一轮收栈留下的残状态 —— 收栈收的是**这套栈**，不是控制柜里的"
                "程序）。要让这一项真的考到人，下一轮起栈**之前**先在示教器上按 STOP。"
                % (board_lib.dash_program_state(cabinet) or "PLAYING"))
    return ""


def playing_is_vacuous(cabinet):
    """`P3.示教器 PLAYING` 这一条绿**算不算数**。**纯函数** → `True` / `False` / `None`。

    与 `residual_playing_note()` 是**同一根判据的两面**：那一支给**人**读（散文），
    这一支给**机器**读（进 JSON、进结算）。两者必须同步改 —— 合在一处算就是
    为了让它们不可能漂开。

    · `False` —— 起栈**之前**控制柜不是 PLAYING ⇒ 这个 PLAYING 是**人按出来的**。
    · `True`  —— 起栈之前**就已经** PLAYING（残状态）⇒ 绿是真的（控制器确实连得上，
      闸门该放行 —— 用户的裁决是「说清 ＋ 放行」），但「**人在环走通了**」是假的。
    · `None`  —— 起栈前**没读到**（Dashboard 没答话）⇒ **判不了**。⚠️ 这一档
      **不许**当成 `False`：把「没读到」算成「确认是人按的」，正是本仓最贵的那类
      误读（「一次状态查询报告的往往是它自己那个对象的活性」）。
    """
    if cabinet is None:
        return None
    return bool(board_lib.is_playing(cabinet))


IN_LOOP_ITEM = "P3.示教器 PLAYING"


def needs_repress(items):
    """交棒**之前**要不要让人重按一次（STOP → PLAY）。**纯函数** → `bool`。

    `items` = 已计入三态的 `P3` 那几项。

    · `False` —— `P3.示教器 PLAYING` 是**实绿**（起栈前不是 PLAYING ⇒ 人真按了）
    · `True`  —— 空洞绿（`True`）或**判不了**（`None`）；**那一项压根找不到**也算
      `True`（fail-closed：判不了就按「要」办）

    ⚠️ **这一条只在「真要跑运动」的那一轮生效，绝不能下沉进 `decide()`。**
    `decide()` 是全流程唯一的退出码出口（P0–P5 都走它），把空洞绿放进去会连
    **R0（`skip_planner:=true`、无运动）**一起拦在 P3 ⇒ 白丢掉 P4 的证据。
    空洞绿在**不跑运动**时只是一个**记录**问题（用户 2026-10-07 的裁决：
    「标记 ＋ 总表分开计数」），在**要跑运动**时才是**动作**问题（人可能压根没按，
    而臂会动）。同一根判据，**要不要拦取决于下一步会不会动**。

    ★ 先例就在同一个文件里：`counted()` 把「声明缺席」的项排除在三态计数之外，
    理由也是「同一件事在不同场合该不该算数不一样」。同一条道理。
    """
    for i in items:
        if getattr(i, "name", None) == IN_LOOP_ITEM:
            return getattr(i, "vacuous", False) is not False
    return True


def count_vacuous(items):
    """→ `(确认空洞的 PASS 数, 未判的 PASS 数)`。**纯函数**。

    分开数是因为它们**不一样**：`True` 是**查实了**的（人没动手），`None` 是
    **没查到**。合成一个数会重演「同一个标签盖两种状态」。
    """
    def _v(i):
        return getattr(i, "vacuous", False)
    return (sum(1 for i in items if i.status == PASS and _v(i) is True),
            sum(1 for i in items if i.status == PASS and _v(i) is None))


def judge_preflight(facts):
    """P1 全部前置项 → `[Item, ...]`（顺序 = `P1_NAMES`）。**纯函数**。

    I/O 全在 `collect_preflight`。`facts` 里的值给 `None` 表示「这一项**没测成**」
    —— 一律按 **SKIP**（fail-closed：没验到不算通过），**绝不**当成通过。
    """
    out = []

    lo = facts.get("leftovers")
    if lo is None:
        out.append(Item("P1.残留进程", SKIP, "没查（ps 没跑成）"))
    elif lo:
        out.append(Item("P1.残留进程", FAIL,
                        "%d 条残留（先收干净再来，否则端口/串口会打架）：\n%s"
                        % (len(lo), "\n".join("      " + s[:120] for s in lo[:6]))))
    else:
        out.append(Item("P1.残留进程", PASS, "无残留（判据含自我排除，见 board_lib）"))

    port = facts.get("port") or "<未配置>"
    if facts.get("port_busy") is None:
        out.append(Item("P1.串口", SKIP, "没查（%s 的占用没测成）" % port))
    elif facts["port_busy"]:
        out.append(Item("P1.串口", FAIL,
                        "%s 被占用 —— 多半就是残留的规划器；先收干净" % port))
    else:
        out.append(Item("P1.串口", PASS, "%s 空闲" % port))

    g = facts.get("gripper")
    if g is None:
        out.append(Item("P1.AG-95 初始化", SKIP, "没读（0x0200 没读到）"))
    else:
        st, why = g
        if st is None:
            out.append(Item("P1.AG-95 初始化", FAIL,
                            "读不到夹爪状态（%s）—— 串口没通就别急着起栈" % why))
        elif st != 1:
            name = {0: "未初始化", 2: "初始化进行中"}.get(st, "未知值 %s" % st)
            out.append(Item("P1.AG-95 初始化", FAIL,
                            "%s（0x0200=%s）—— 先跑 "
                            "`python3 scripts/ag95_init_close.py --go`（写完停在"
                            "**全开**；规划器起栈时自己会 `open()`，所以这里不需要"
                            "闭合）。未初始化时"
                            "起栈，规划器会在启动序列上**静默卡住十几分钟**"
                            % (name, st)))
        else:
            out.append(Item("P1.AG-95 初始化", PASS, "0x0200=1（已初始化）"))

    out.append(Item("P1.控制柜", *judge_cabinet(facts.get("cabinet"))))

    rc = facts.get("deploy_rc")
    if rc is None:
        out.append(Item("P1.部署前置", SKIP,
                        "没跑 check_board_deploy.py（脚本不在 / 跑不起来）"))
    elif rc == 0:
        out.append(Item("P1.部署前置", PASS, "check_board_deploy.py 退 0（依赖齐）"))
    elif rc == 2:
        # ⚠️ 与下面 `tree_rc` 的 2 **同样处置** —— 同一个退出码不许两种待遇。
        # 2 在本仓的定义是「**环境不满足**，不是失败」（AGENTS.md §6）。而这个脚本
        # 只有**一处**退 2：`check_board_deploy.py:174-175`「需要 ROS 环境」。
        # 它**不可能**意味着「缺依赖」—— `run_helper` 继承本进程的环境，而 P0
        # 已经查过 ROS 环境与域号并且**已经放行**才走到 P1。所以这里说「没验到」
        # 才是实话：本项没测成，不是「看到了，而且它不对」。
        out.append(Item("P1.部署前置", SKIP,
                        "check_board_deploy.py 退 2 —— 它只有「需要 ROS 环境」这一处"
                        "退 2，而 P0 已经确认环境在位，所以这一项**没验到**"
                        "（不是缺依赖）。多半是子进程没拿到环境，先看 P0 那一行"))
    else:
        out.append(Item("P1.部署前置", FAIL,
                        "check_board_deploy.py 退 %s —— 缺依赖时症状离原因很远"
                        "（缺一个 apt 包的表现是「三个节点全起不来」），先照它列的修"
                        % rc))

    rc = facts.get("tree_rc")
    if rc is None:
        out.append(Item("P1.板子树", SKIP, "没跑 board_tree_manifest.py（脚本不在）"))
    elif rc == 0:
        out.append(Item("P1.板子树", PASS, "全树哈希与清单一致（对全树，不抽查）"))
    elif rc == 2:
        out.append(Item("P1.板子树", SKIP,
                        "**没有清单**（fail-closed：没有清单 ≠ 一致）—— 先在已知是好的"
                        "那一侧跑 `python3 scripts/board_tree_manifest.py --write`，"
                        "再把清单同步过去"))
    else:
        out.append(Item("P1.板子树", FAIL,
                        "板子上那份检出与本机**不是同一棵树**（不变量 4：它会静默过期）"
                        " —— 同步后**必须 colcon build**（install/ 指向 build/）"))

    fr = facts.get("freshness")
    if fr is None:
        out.append(Item("P1.外参时效", SKIP, "没判（站点没读进来）"))
    else:
        out.append(Item("P1.外参时效", fr[0], fr[1]))
    return out


def candidate_margin_note(n_candidates, slots_total):
    """P4 之后的**提示**（不是判据）：候选数 ≤ 槽位数时，A1 的判据 2 拿不到。

    真机实测：4 块 / 4 槽时第 4 块放下的**同一刻**抓取区也空了，
    `_select_and_approach` 先命中 `if best is None:`，**根本走不到**下面那道槽位闸门
    ⇒ 收工走 `idle_finish_timeout`，「放置槽位已用尽」那行永远不会打出来。
    这一条**只能靠人摆件**解决，所以这里只出声、不拦（拦下来就变成「判据自己错」，
    比不判还坏）。
    """
    if n_candidates is None or n_candidates > slots_total:
        return None
    return ("候选 %d 个 ≤ 槽位 %d 个 ⇒ 本轮**拿不到**「放置槽位已用尽」那条收工判据"
            "（实测）。抓取区里要摆 **≥%d 块**"
            % (n_candidates, slots_total, slots_total + 1))


def planner_alive(task):
    """本档的规划器**会不会起来**。**纯函数** → `bool`。

    `skip_planner:=true` 时 `pipeline.launch.py:702` 用 `UnlessCondition` 把
    `planner_node` **整个**挡住 ⇒ 本档**没有任何东西会发轨迹**（detector 与
    perception 照常起，但它们只发话题，不发目标）。R0 档存在的意义就是这个。

    ⚠️ 判据只认任务描述里**显式写的那一项**，不去猜 launch 的默认值：这个键的
    默认是 false（规划器起），而「没写」就是「没声明不起」—— 拿默认值当依据，
    哪天 launch 改了默认值，这里会**静默**跟着变。
    """
    if task is None:
        return True
    v = task.params.get("skip_planner")
    return not (v is not None and str(v).strip().lower() in ("true", "1", "yes", "on"))


def judge_handoff_window(t_launch, t_stop, start_delay, planner_alive=True):
    """停下时，规划器**是不是已经起来了**。**纯函数** → `(bool, 说明)`。

    这一条是为了不把话说过头：退出码 2 的官方说法是「**我拒绝了、什么都没发出去**」。
    但 `start_delay` 是**可配**的（A1 用 120 s），如果这一轮按播放按晚了、或者
    `start_delay` 很小，规划器可能**已经启动并动过臂** —— 那时再收栈就不是
    「什么都没发出去」了。与其含混，不如把时间摆出来让人自己看。

    ⚠️ `planner_alive=False`（`skip_planner:=true` 的档，见 `planner_alive()`）时
    上面那套时间推断**整条不适用**：规划器根本没起，臂不可能被这套栈动过，
    与起栈活了 273 s 还是 3 s 无关。**2026-10-07 R0 run 6 实测**：那一档
    `skip_planner:=true`，起栈活了 273 s，于是这条按 `start_delay` 判的判据
    印出「🔴 规划器**可能已经启动并动过臂**……**不能**自称什么都没发出去」——
    **两个断言都是假的**，而它长着一副「发现了问题」的样子（红字 + 🔴）。
    这一档本来就是要证明「无运动」的，红字恰好把它最该立的结论说反了。
    """
    if not planner_alive:
        return False, ("本档的规划器**根本不会起**（任务描述里 `skip_planner` 为真 ⇒ "
                       "`pipeline.launch.py:702` 用 `UnlessCondition` 挡住 "
                       "`planner_node`）⇒ 这套栈**没有任何东西会发轨迹**，与起栈活了"
                       "多久无关。所以这一轮**可以**自称「什么都没发出去」——"
                       "这比「臂应当没动过」更强")
    if t_launch is None or t_stop is None:
        return False, ("没有起栈时间戳（--attach / --dry-run / 没起栈）—— "
                       "「有没有动过臂」**未判**")
    elapsed = t_stop - t_launch
    if elapsed >= start_delay:
        return True, ("起栈到停下已经 %.0fs ≥ start_delay=%gs ⇒ **规划器可能已经启动"
                      "并动过臂** —— 这一轮**不能**自称「什么都没发出去」"
                      % (elapsed, start_delay))
    return False, ("起栈到停下只有 %.0fs < start_delay=%gs ⇒ 规划器的定时器还没到点，"
                   "臂**应当**没动过" % (elapsed, start_delay))


# P4 体检相对 `start_delay` 的宽限。`TimerAction` 是从 **launch 进程自己起来之后**
# 才开始计时的，而我们量的 `t_launch` 是 `Popen` 那一瞬 —— 中间那段要被补回来。
P4_GRACE = 10.0


def p4_wait_seconds(t_launch, now, start_delay):
    """P4 之前还得等多少秒。**纯函数** → `float`（`0.0` = 不用等）。

    ⚠️ **为什么非等不可 —— 2026-10-06 真机第 2 次实测**：`pipeline.launch.py:738` 是
    `TimerAction(period=…start_delay…, actions=[realsense, detector, perception, planner])`
    —— `start_delay` 包住的是**整组**（相机与检测器也在里头），不只是规划器。
    而 P4 是在 P3 一通过就**立刻**跑的（`board_run.py` 里 `if code == 0:` 紧跟其后）。
    人按播放**按得快**（本来就该快）⇒ P4 落在 `start_delay` **之内** ⇒ 相机 / 检测器 /
    感知**本来就还没起** ⇒ 三项记 FAIL。**越是照做的操作者越被判红。**
    `board_gate` 那一侧的任何超时都救不了：第 2 次栈总共只活了 **32 s**，而
    `start_delay=120`，闸门默认那点等待（discover 25 s + targets 20 s）**不在一个量级上**。

    第 1 次之所以没露头，是因为人**按晚了**（1175 s 才按），无意中把这一段盖过去了 ——
    与 `test_board_run.py` 那个「红只在没被跑到的那一侧」是同一个形状。

    `t_launch is None`（`--attach` / `--dry-run` / 没起栈）⇒ `0.0`：没有「起栈时刻」，
    这一档无从谈起，也**不许**假装等过。
    """
    if t_launch is None:
        return 0.0
    left = (t_launch + float(start_delay or 0.0) + P4_GRACE) - now
    return left if left > 0.0 else 0.0


def read_verdict(v):
    """判分结果字典 → `True` / `False` / `None`。**纯函数**。

    `None` = **形状不对**（没有 `ok` 键）—— 调用方必须当成「判不了」，**不是**通过。
    这一条挡的是「判分器换了个形状，报告里却全是绿的」。
    """
    if not isinstance(v, dict) or "ok" not in v:
        return None
    return bool(v["ok"])


def decide(items, verdict=None, launched=False):
    """三态项 + 判分结果 → `(退出码, 要不要收栈, 为什么)`。**纯函数**。

    全流程唯一决定退出码的地方（P0–P5 都走它），所以那三条语义只写一遍：
      · 有 FAIL  → 2（看到了，而且它不对）
      · 有 SKIP  → 2（**没验到不算通过**，fail-closed）
      · 判分没过 → 1（闸门都过了、它跑了，但这一轮结果不行）
      · 其余     → 0（**只有真跑完一轮且判分通过才可能拿到**）
    `launched` 决定「要不要收栈」：P0/P1/P2 判定时栈还没起，就**不起栈**；
    P3 之后栈已经起了（而且是**我们起的**），就**立刻收掉**（不发任何运动命令）。
    """
    if any(i.status == FAIL for i in items):
        return 2, launched, "有 FAIL（看到了，而且它不对）"
    if any(i.status == SKIP for i in items):
        return 2, launched, "有 SKIP（**没验到**，fail-closed 不算通过）"
    if verdict is False:
        return 1, launched, "闸门全过、任务跑了，但**这一轮自己的判分没过**"
    return 0, launched, "闸门全过" + ("，判分通过" if verdict else "")


# =====================================================================
# 渲染：SKIP 永远不是绿的
# =====================================================================

def counted(report, title=None):
    """报告里**计入三态**的项（声明缺席的不计入）。**纯函数**。"""
    return [i for (t, i, absent) in report["rows"]
            if not absent and (title is None or t == title)]


def render_item(item, absent=None):
    """一个三态项 → 要打印的行。**纯函数**（好离线钉「SKIP 不许渲染成绿色」）。

    声明缺席的行**不显示**底下那一层的结论：缺席项是按「没测」造出来的，它的
    detail 可能是「连不上控制柜」这种**没测过就下的结论**，印出来会被读成
    「测出来是坏的」—— 那正是这个仓最贵的一类误读。

    ⚠️ 缺席**不印成 `[SKIP]`**。缺席（调用方声明这一档本来没有它）与真 SKIP
    （有这一项、但没验成）是**两回事**，在结算里也是两个数 —— `--dry-run` 那档
    结算写「SKIP 0（另有 11 项声明缺席）」，若缺席也印 `[SKIP]`，满屏的 `[SKIP]`
    会和「SKIP 0」当场自相矛盾。同一个标签盖两种状态，正是本文件到处在防的那类
    静默误读，只不过这次是防在读报告的人身上。
    """
    if absent:
        return ["  [缺席] %-18s —（这一档没测）" % item.name,
                "         ↳ 声明缺席（%s）—— 既不计入通过，也不计入失败" % absent]
    lines = ["  [%s] %-18s %s" % (item.status, item.name, item.detail)]
    # `SKIP` 与「空洞绿」是同一条规矩的两个实例：**一条状态行不许自带两种含义**。
    # 所以两者都在状态行底下补一行，把「这一条到底证明了什么」说死。
    v = getattr(item, "vacuous", False)
    if item.status == SKIP:
        lines.append("         ↳ **没验到**，不计入通过。")
    elif item.status == PASS and v is True:
        lines.append("         ↳ ⚠️ **空洞绿**：状态是 `PASS`，但这一条**没有**证明"
                     "「人在环走通了」（理由见上）。")
    elif item.status == PASS and v is None:
        lines.append("         ↳ ⚠️ **未判**：「人在环」这一半**没读到**控制柜 ⇒ "
                     "**不许**读成验过了。")
    return lines


def render_counts(items):
    """→ `(PASS 数, FAIL 数, SKIP 数)`。**纯函数**。"""
    return (sum(1 for i in items if i.status == PASS),
            sum(1 for i in items if i.status == FAIL),
            sum(1 for i in items if i.status == SKIP))


def render_summary(items, code, absent=None, verdict_state=None, dry_run=False):
    """总结区（**纯函数**）。三态计数 + 退出码 + 「2 是我拒绝了 / 1 是它跑了但没过」。

    ⚠️ `--dry-run` 退 0 **不许**印成「全绿/验收通过」：那一档里一个闸门都没跑，
    只是「计划拼得出来」。换句话说，它的绿说的是**文件**，不是**设备**。
    """
    p, f, s = render_counts(items)
    out = ["  PASS %d / FAIL %d / SKIP %d" % (p, f, s)
           + ("（另有 %d 项声明缺席，不计入）" % len(absent) if absent else "")]
    # ⚠️ 总表**必须**把空洞绿单独说一句。理由：连跑 N 次时，只有第 1 次真的考到人
    # （之后每轮起栈前控制柜还是 PLAYING，除非人按 STOP）。若总表只印
    # `PASS 24`，三连跑会被读成「人在环走通 3 次」，而真相是 1 次。
    n_vac, n_unjudged = count_vacuous(items)
    if n_vac or n_unjudged:
        bits = []
        if n_vac:
            bits.append("**%d 项空洞**（人没按，是上一轮留下的残状态）" % n_vac)
        if n_unjudged:
            bits.append("**%d 项未判**（起栈前没读到控制柜）" % n_unjudged)
        # 分母是**全部** PASS（`p`），不是空洞数 —— 写成空洞数会印出
        # 「上面这 1 项 PASS 里，1 项空洞」，而总数其实是 2。
        out.append("  ⚠️ 上面这 %d 项 PASS 里，%s —— **都不算「人在环走通了」**。"
                   % (p, "、".join(bits)))
    if verdict_state:
        out.append("  本任务判分：%s" % verdict_state.splitlines()[0])
    if code == 0 and dry_run:
        out.append("  结果：✅ 计划立得住（**静态**检查全过）—— 这**不是**验收："
                   "%d 项声明缺席，一道闸门都没验" % len(absent or []))
    elif code == 0:
        out.append("  结果：✅ 全绿（闸门全过 + 本任务判分通过）")
    elif code == 1:
        out.append("  结果：❌ 闸门过了、任务跑了，但**这一轮自己的判分没过**")
    else:
        out.append("  结果：⛔ 在**交棒之前**停下了")
    out.append("")
    out.append("  " + EXIT_LEGEND)
    return out


# =====================================================================
# I/O 薄适配器 —— 只有这里碰进程 / 文件 / 设备
# =====================================================================

def path_in_repo(path, repo=REPO):
    """这个路径在仓库里吗。**纯函数**。给 `--json-report` 当门卫用。

    为什么要挡：**本仓是公开副本，一切提交永久公开**（红线 3）。运行记录里带着现场
    路径与时间，写进仓库就有被提交的风险 —— 它本来就该落在仓库外。
    """
    p = os.path.abspath(os.path.expanduser(str(path)))
    r = os.path.abspath(repo)
    return p == r or p.startswith(r + os.sep)


def collect_preflight(site, do_net=True, override=None, env=None):
    """把 P1 要的事实**读一遍**（全是只读查询），交给 `judge_preflight` 判。

    每个 `None` 都表示「这一项没测成」，由纯函数按 SKIP 处置 —— 这里**不**替它下结论。
    `do_net=False`（`--dry-run`）只读站点字段 + 判时效，不跑任何子进程。
    `env` 是 `build_ros_env()` 算出来的那份，两个守卫脚本都用它跑。
    """
    if not do_net:
        return {"leftovers": None, "port": site.gripper_port, "port_busy": None,
                "gripper": None, "cabinet": None, "deploy_rc": None,
                "tree_rc": None,
                "freshness": board_lib.freshness_gate(site, override=override)}
    f = {"port": site.gripper_port}

    try:
        f["leftovers"] = board_lib.leftovers()
    except Exception:                               # noqa: BLE001
        f["leftovers"] = None

    if site.gripper_port:
        try:
            f["port_busy"] = board_lib.port_busy(site.gripper_port)
        except Exception:                           # noqa: BLE001
            f["port_busy"] = None
    else:
        f["port_busy"] = None

    try:
        f["gripper"] = board_lib.gripper_state(site.gripper_port)
    except Exception as exc:                        # noqa: BLE001
        f["gripper"] = (None, "%s: %s" % (type(exc).__name__, exc))
    try:
        f["cabinet"] = board_lib.dashboard(site.robot_ip)
    except Exception:                               # noqa: BLE001
        f["cabinet"] = None

    f["deploy_rc"] = run_helper(DEPLOY_SCRIPT, [], timeout=180.0, env=env)
    f["tree_rc"] = run_helper(TREE_SCRIPT, ["--check"], timeout=300.0, env=env)
    f["freshness"] = board_lib.freshness_gate(site, override=override)
    return f


def run_helper(script, args, timeout, env=None):
    """跑一个**只读**的守卫脚本，返回退出码；跑不起来（不在/起不动）返回 `None`。

    `env` 必须是 `build_ros_env()` 算出来的那份 —— 守卫脚本（`check_board_deploy.py`
    等）要按 `package.xml` 解析依赖，**它看的就是 `AMENT_PREFIX_PATH`**。
    继承调用者的环境曾让这一项在板上恒红（见 `build_ros_env` 的实测记录）。
    """
    if not os.path.exists(script):
        return None
    try:
        r = subprocess.run([sys.executable, script] + list(args), cwd=REPO,
                           capture_output=True, text=True, timeout=timeout,
                           env=env)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.returncode


def run_show_args(entry, env, timeout=120.0):
    """跑一条 `--show-args`（**不启动任何节点**），返回 `(rc|None, 输出)`。"""
    pkg, fname = split_entry(entry)
    try:
        r = subprocess.run(["ros2", "launch", pkg, fname, "--show-args"],
                           cwd=REPO, capture_output=True, text=True,
                           timeout=timeout, env=env)
    except FileNotFoundError:
        return None, "PATH 里没有 `ros2`（先 source /opt/ros/humble/setup.bash）"
    except subprocess.SubprocessError as exc:
        return None, "%s: %s" % (type(exc).__name__, exc)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def gate_argv(json_path, *, only=None, slots_total=None, min_candidates=None,
              expect_params=None, allow_skip=None, trajectory_action=None,
              pick_zone=None, repair=False, fast=False):
    """拼 `board_gate.py` 的命令行。**纯函数**（离线钉得住「P3 只要控制器」）。

    `only="controllers"` 时，其余五项交给它的 `--allow-skip` 声明缺席 —— 那正是
    `--allow-skip` 的语义（「调用方显式声明这一档本来就没有它」）：P3 那会儿规划器还
    没起（`start_delay` 那一段），那五项**本来**就不该有。
    """
    argv = [sys.executable, GATE_SCRIPT, "--json-report", json_path]
    skip = list(allow_skip or [])
    if only is not None:
        skip += [n for n in board_gate.ITEM_NAMES if n != only]
    if trajectory_action:
        argv += ["--trajectory-action", str(trajectory_action)]
    if skip:
        argv += ["--allow-skip", ",".join(sorted(set(skip)))]
    if slots_total is not None:
        argv += ["--slots-total", str(slots_total)]
    if min_candidates is not None:
        argv += ["--min-candidates", str(min_candidates)]
    # 只在**规划器不起**的档递（`pick_zone_arg` 里那节说明为什么不能总递）。
    if pick_zone:
        argv += ["--pick-zone", ",".join(str(float(x)) for x in pick_zone)]
    for k, v in sorted((expect_params or {}).items()):
        argv += ["--expect-param", "%s=%s" % (k, v)]
    if repair:
        argv.append("--repair-controller")
    if fast:
        argv += ["--discover-timeout", "8", "--controller-timeout", "60",
                 "--camera-window", "0.5", "--targets-timeout", "0.5",
                 "--telemetry-timeout", "0.5"]
    return argv


def argv_json_report(argv):
    """从拼好的命令行里找回 `--json-report` 那个路径。**纯函数**。"""
    for i, t in enumerate(argv):
        if t == "--json-report" and i + 1 < len(argv):
            return argv[i + 1]
    return None


def read_gate_json(path):
    """读 board_gate 留下的记录 → `(record|None, 原因)`。"""
    if not path or not os.path.exists(path):
        return None, ("board_gate 没有留下 JSON 记录（`--json-report` 没给，或它连"
                      "启动都没走到 —— 例如本机没有 rclpy）")
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh), ""
    except (OSError, ValueError) as exc:
        return None, "board_gate 的 JSON 记录读不出来：%s" % exc


def run_gate(argv, env, timeout):
    """跑 board_gate 并读回它的记录 → `(rc|None, record|None, 原因)`。"""
    path = argv_json_report(argv)
    try:
        r = subprocess.run(argv, cwd=REPO, capture_output=True, text=True,
                           timeout=timeout, env=env)
        rc = r.returncode
    except subprocess.SubprocessError as exc:
        return None, None, "%s: %s" % (type(exc).__name__, exc)
    rec, why = read_gate_json(path)
    return rc, rec, why


def gate_items(record):
    """board_gate 的 JSON 记录 → `{项名: Item}`。**纯函数**。

    形状不对 ⇒ 抛 `GateError`。**不许**把「读不出」当成「没有 FAIL」——
    那正是「绿得没有理由」的另一种写法。
    """
    if not isinstance(record, dict) or not isinstance(record.get("items"), list):
        raise GateError("board_gate 的记录里没有 items 列表 —— 读不出就不能算过")
    out = {}
    for it in record["items"]:
        if not isinstance(it, dict) or "item" not in it or "status" not in it:
            raise GateError("board_gate 的记录里有一项形状不对：%r" % (it,))
        out[it["item"]] = Item(it["item"], it["status"], str(it.get("detail", "")),
                               it.get("evidence") or {})
    return out


def stage_from_gate(record, prefix, names):
    """把 board_gate 的六项挑出 `names` 那几项，改成 P3/P4 的项名。
    **纯函数** → `[(Item, 缺席理由|None), ...]`。

    ⚠️ `--allow-skip` 的项在 board_gate 里被**改写成 SKIP**（并在 detail 里注明），
    在本文件里它们该算**缺席**（调用方显式声明这一档没有它），**不是**没验到 ——
    否则「`skip_planner:=true` 的任务」会在 P4 恒定退 2。判据取自记录里的
    `allow_skip` 字段，不另猜。
    """
    got = gate_items(record)
    allow = [str(x) for x in (record.get("allow_skip") or [])]
    out = []
    for n in names:
        it = got.get(n)
        if it is None:
            raise GateError("board_gate 的记录里没有 %r 这一项（有：%s）"
                            % (n, ", ".join(sorted(got))))
        absent = ("board_gate --allow-skip：本档没有它"
                  if (n in allow and it.status == SKIP) else None)
        out.append((Item("%s.%s" % (prefix, n), it.status, it.detail,
                         it.evidence), absent))
    return out


def open_log(path):
    """开日志文件（栈的 stdout+stderr 都往里灌；P5 判分也读它）。"""
    return open(os.path.expanduser(path), "w")


def wait_for_state(ip, want, timeout, poll=2.0, dash=None, clock=time.time,
                   sleep=None):
    """轮询只读直到 `want(回复)` 为真 → `(到了没, 最后一次回复, 用时)`。

    只读查询（`robotmode` / `programState` / `safetystatus`）；**不发** play / power /
    stop —— 按播放是**人的动作**（红线 6）。

    `dash` / `clock` / `sleep` 都可注入 ⇒ 这个循环**离线测得动**
    （照 `wait_for_finish` 的先例）。
    """
    dash = dash or board_lib.dashboard
    sleep = time.sleep if sleep is None else sleep
    t0 = clock()
    last = None
    while True:
        last = dash(ip)
        if want(last):
            return True, last, clock() - t0
        if clock() - t0 >= timeout:
            return False, last, clock() - t0
        sleep(poll)


def wait_for_playing(ip, timeout, poll=2.0, dash=None, clock=time.time):
    """起栈**之后**轮询只读直到 PLAYING → `(到了没, 最后一次回复, 用时)`。

    只读查询（`robotmode` / `programState` / `safetystatus`）；**不发** play / power /
    stop —— 按播放是**人的动作**（红线 6）。
    """
    return wait_for_state(ip, is_playing, timeout, poll=poll, dash=dash, clock=clock)


def ensure_in_the_loop(ip, timeout, poll=2.0, dash=None, clock=time.time,
                       sleep=None, prompt=print):
    """交棒**之前**复核「人在环」—— 让人重按一次（**先 STOP，再 PLAY**），
    并**看到那两步都真的发生**。→ `(过了没, 说明)`。

    为什么是**两步握手**，不是只喊一句「按 STOP 再按 PLAY」：两个动作挨得近时，
    2 s 一次的轮询可能**一次都没看见 STOPPED** —— 那时「PLAYING 一直为真」
    既可能是「人按了」也可能是「压根没动」，**判不了**。所以卡在两步之间：
    先**确认**它离开 PLAYING，再等它回来 ⇒ 「按过」这件事是**看到**的，不是猜的。
    第一步没看到就直接**不交棒**（fail-closed）。

    这**不是新规矩**：第一百一十轮实测过「收栈之后控制柜可能仍停在 PLAYING」，
    当时的结论就是「每轮之间必须在示教器上按一次 STOP」—— 这里只是把它写成
    启动器自己会走的步骤。

    全程只读（`dashboard`）；**不发** play / power / stop。
    """
    prompt("")
    prompt("=" * 66)
    prompt("👉 交棒前复核「人在环」：请在示教器上按一下 **停止（STOP）**。")
    prompt("   （上一轮收栈后控制柜可能还停在 PLAYING —— 那种绿**不算**你按过。）")
    prompt("=" * 66)
    left, d, _ = wait_for_state(ip, lambda x: not is_playing(x), timeout,
                                poll=poll, dash=dash, clock=clock, sleep=sleep)
    if not left:
        return False, ("%.0fs 内没看到它离开 PLAYING（原文 %r）—— 「人在环」"
                       "是不是**这一轮**走通的，判不了 ⇒ **不交棒**"
                       % (timeout, str((d or {}).get("programState", "")).strip()))

    prompt("")
    prompt("=" * 66)
    prompt("👉 现在请按示教器上的 **播放（PLAY）**。")
    prompt("=" * 66)
    back, d, waited = wait_for_state(ip, is_playing, timeout,
                                     poll=poll, dash=dash, clock=clock,
                                     sleep=sleep)
    if not back:
        return False, ("按了停止之后，%.0fs 内没等到 PLAYING（原文 %r）⇒ **不交棒**"
                       % (timeout, str((d or {}).get("programState", "")).strip()))
    return True, ("STOP → PLAY 两步**都看到了**（%.0fs 后回到 PLAYING）—— "
                  "「人在环」是这一轮走通的" % waited)


def find_marker(lines, marker):
    """日志里出现「收工」那一行了吗。**纯函数**。

    ⚠️ 逐行判，不是「整份日志找子串」—— 正文里**引用**收工语（注释、报错回显）
    不该算数（`board_a1.py` 的诱饵行就是钉这一条）。
    """
    return any(marker in ln for ln in lines)


def read_lines(path):
    try:
        with open(os.path.expanduser(path), errors="replace") as fh:
            return fh.read().splitlines()
    except OSError:
        return []


def wait_for_finish(log_path, deadline, marker, clock=time.time, sleep=None):
    """盯日志到出现收工行或超时 → `(全部行, 收工了没)`。"""
    sleep = time.sleep if sleep is None else sleep
    lines = []
    while clock() < deadline:
        lines = read_lines(log_path)
        if find_marker(lines, marker):
            return lines, True
        sleep(2.0)
    lines = read_lines(log_path)
    return lines, find_marker(lines, marker)


# =====================================================================
# 红线 6：两条写路径（控制器切换 / 夹爪寄存器）—— 报文、授权、复核
# =====================================================================
# 发文那一行在 board_gate.py（`Probe.switch_controllers`），本文件**不重实现**。
# 本文件只做三件它该做的事：起栈**之前**拒绝没有当次授权的请求、把要发的报文
# **先完整摆出来**、切完按**只读复核**判「生效 / 未生效」。三条见文件头。
#
# 夹爪那一条（`--init-gripper`）**照抄同一套纪律**，理由见 `gripper_init_plan`。
# 发文那一行在 `scripts/ag95_init_close.py`（**全仓唯一**会驱动夹爪的脚本），
# 同样是**子进程跑它**、本文件不重实现。

OUTPUT_PATH_ITEM = "P0.输出路径"
WRITE_PATH_ITEM = "P0.写路径授权"
GRIPPER_INIT_ITEM = "P0.夹爪初始化授权"
REPAIR_ITEM = "P3.控制器修复"
SWITCH_TIMEOUT = 8.0            # 与 board_gate.Probe.switch_controllers 的默认值同
GRIPPER_SCRIPT = os.path.join("scripts", "ag95_init_close.py")
GRIPPER_INIT_TIMEOUT = 90.0     # 初始化要跑一次全行程（张开再合上），给足


def judge_output_paths(log_path, json_report, repo=REPO):
    """`--log` / `--json-report` 必须落在**仓库外**。**纯函数** → `(status, detail)`。

    ⚠️ 为什么要在 **P0** 判，而不是等要写的时候再判（这里原来只有 `--json-report`
    在**最后**判了一下，`--log` **根本没判**）：

    1. `--log` 是拿 `open(path, "w")` **直接开写**的（`open_log`）—— 起栈那一刻
       就开始往那个文件里灌。栈的日志里有**控制箱 IP、相机序列号、串口 by-id**，
       而**本仓是公开副本**（红线 3）⇒ 把 `--log` 指进仓库，等于一边跑一边在准备
       一份**带真值的待提交文件**。
    2. 原来那句 `--json-report 指向仓库内…**没有写**` 是在**跑完之后**才打印的，
       而且**不翻退出码**（拒绝了却退 0）。后果有两层：脚本化调用者把「拒绝」
       读成「成功」；而且**记录是在跑完一整轮之后才丢的** —— R0/R1 要的正是那份
       JSON，等看到这一行时这一轮已经白跑了。

    两件都在「**P0 不合法就不起栈**」的合同之内，所以挪到 P0：早退 2，什么都不写。
    `--attach` 与 `--dry-run` 同样走这一关（都是静态判断，不依赖任何设备）。
    """
    bad = []
    if log_path and path_in_repo(log_path, repo):
        bad.append("--log（%s）" % log_path)
    if json_report and path_in_repo(json_report, repo):
        bad.append("--json-report（%s）" % json_report)
    if not bad:
        return (PASS, "两个输出路径都在仓库外 —— 运行记录不进公开仓")
    return (FAIL, "、".join(bad) + " 指向**仓库内**。本仓是**公开副本，"
                  "一切提交永久公开**（AGENTS.md 红线 3），运行记录里带着现场路径、"
                  "IP、序列号 ⇒ 必须写在仓库外。**这一次不会写、也不会起栈**。"
                  "改成仓库外的路径（例如 `~/r0_runs/…`）再来；"
                  "不要为了绕开它把记录写进仓库，也不要事后 `git add` 那个文件。")


def judge_write_path(isatty, flag="--repair-controller", fire_note=None):
    """写路径那一档能不能往下走。**纯函数** → `(status, detail)`。

    两条写路径（`--repair-controller` 控制器切换、`--init-gripper` 夹爪寄存器）
    **共用本函数** —— 纪律是同一条，两处各写一份迟早会分叉，而分叉的那一份会给出
    假的授权。`flag` 与 `fire_note` 只影响文案。

    非交互 ⇒ **FAIL**，不是 SKIP：这不是「没验到」，是「这个请求本身不成立」。
    放在 P0，是为了**在起栈之前**就拒 —— 臂上电、栈起着的时候才发现没法授权，
    那已经晚了。
    """
    if isatty():
        return (PASS, "有交互终端 —— 到时候会**先打印完整报文**，再要你敲一次 yes；"
                      + (fire_note or "发文那一刻（臂已上电）还会再确认一次"))
    return (FAIL, "`%s` 要的是**当次**授权，而这里不是终端。"
                  "授权**不许**来自任务描述 / 环境变量 / 上一次的记忆（AGENTS.md "
                  "红线 6：「上一次的授权不覆盖这一次」）。换成交互终端再来；"
                  "不要写路径的话去掉这个开关即可（默认只读）。" % flag)


def _required_controllers_for(task):
    """要激活哪几个控制器 —— 复用 `board_gate` 的**那一份**推导。**纯函数**。

    ⚠️ 调的是 board_gate 的私有函数，这是**故意**的：「盯哪个控制器」只能有一处
    真值，本文件另抄一套，就会在改一边时静默对不上 —— 而报文一旦和实际发出去的不
    一样，比不打印更糟（人确认的是**那一条**）。前提是**本文件从不传 `--require`**
    ⇒ 两边走同一条分支。这两件事都在 `test_board_run.py` 里钉着。
    """
    ns = types.SimpleNamespace(require=None,
                               trajectory_action=task.trajectory_action)
    return list(board_gate._required_controllers(ns))


def switch_payload_lines(task, timeout=SWITCH_TIMEOUT, required=None):
    """切换前要**先给人看**的完整报文。**纯函数** → 行列表。

    服务名取自 `board_gate.SWITCH_SERVICE`（**不是**本文件里的字面量）；activate
    列表取自 `board_gate._required_controllers`（同一份推导）。`strictness` 印的是
    board_gate 实际发的那个：BEST_EFFORT(=1)。用 STRICT 的话，已经在 active 的
    控制器会让整批 `Aborting, no controller is switched!`。
    """
    req = _required_controllers_for(task) if required is None else list(required)
    return ["    ⚠️  马上要对控制器服务发一条**写**请求（红线 6 点名的「控制器切换」）：",
            "        服务                   %s" % board_gate.SWITCH_SERVICE,
            "        activate_controllers   %s" % (", ".join(req) or "(空)"),
            "        deactivate_controllers []        ← 恒空，**永不** deactivate"
            "（内容安全，与授权是两件独立的事）",
            "        strictness             BEST_EFFORT(=1)   ← 不是 STRICT",
            "        activate_asap          True",
            "        timeout                %gs" % timeout,
            "    授权只认**这一次**：任务描述 / 环境变量 / 上一次给过的，都不算。"]


def judge_repair_outcome(repair_exercised, ctrl_status):
    """切换**执行过**之后，只读复核的结果怎么算。**纯函数** → `(status, detail)`。

    没执行过 ⇒ `(None, "")`（没有这一项）。执行过 ⇒ 必须给个说法：
      · 复核过了 ⇒ PASS（复核走的是只读 `list_controllers`，不是「我们发成功了吗」）；
      · 复核没过 ⇒ **FAIL**，文案直说「已执行但未生效」。
    发出去的请求不等于结果 —— 这一项**不许**绿，也不许被当成「没验到」混过去。
    """
    if not repair_exercised:
        return None, ""
    if ctrl_status == PASS:
        return PASS, ("切换已执行，复核**已生效**（复核走的是只读 list_controllers，"
                      "不是「请求发成功了吗」）")
    return FAIL, ("已执行但**未生效**（复核仍 %s）—— 发出去的请求不等于结果。"
                  "别把这行读成绿：控制器没拉起来，规划器就驱动不了臂。"
                  % ctrl_status)


def repair_hint(task_name, rel=None):
    """控制器闸门不过时该敲的**那一行**命令（纯函数，便于离线钉住）。"""
    rel = rel or os.path.relpath(os.path.abspath(__file__), REPO)
    return "python3 %s --task %s --repair-controller" % (rel, task_name)


def print_repair_hint(ctrl_status, repair_exercised, task_name, out=print):
    """不过就印**一行**修复命令；印了返回 True。**纯函数**（`out` 可注入）。

    已经执行过切换的不印：那时候的问题是「未生效」，不是「没修」——
    再喊一遍同一条命令只会让人以为没试过。
    """
    if ctrl_status != FAIL or repair_exercised:
        return False
    out("  ↳ 要修的话就这一条（先打印完整报文，再要**当次**交互确认；默认只读）：")
    out("      %s" % repair_hint(task_name))
    return True


# =====================================================================
# 红线 6 第二条写路径：夹爪初始化 / 合拢（`--init-gripper`）
# =====================================================================
# 为什么值得并进启动器（而不是继续让人自己记得跑那条命令）：
#
#   未初始化时 `0x0202` **恒 0**、`close()` 会「报成功但手指一步没动」，**没有任何
#   日志说得出来**。而 P1 的闸门（`0x0200==1`）**只能拦**，拦下之后人得自己去跑那条
#   命令 —— 于是「记住跑哪一档」就成了启动流程里最后一个靠人记的环节，而它恰好是
#   **会动手指**的那一步。并进来 = 把「读状态 → 选最小的一档 → 摆出报文 → 当次确认」
#   变成计算（本启动器的整条思路）。
#
# 纪律与 `--repair-controller` **逐条相同**（同一个 `judge_write_path`）：显式开关、
# 报文**先完整摆出来**、交互终端没了就退 2（**没有无人值守模式**）、报告里写
# `gripper init exercised: yes/no` —— 没被绊倒时的绿是**空的**。
#
# 两条硬约束：
#   · 只在 **P1 之前**跑：这样 P1 自己那次只读读数反映的就是**做完之后**的状态，
#     「刚写完」与「合规」是同一个读数说的，不需要第二处判定。
#   · 走**子进程**跑 `ag95_init_close.py`，本文件不重实现写寄存器 —— 写路径只有一处。

def gripper_init_plan(status):
    """读到的 `0x0200` → 该跑哪一档。**纯函数** → `(argv|None, 说明)`。

    **选最小那一档**：已经初始化就**不重走全行程自检**（两指先张开再合上）——
    那正是 `--close` 单独这一档存在的理由（2026-10-07 加）。
    读不到 / 不认识的值 ⇒ **不写**：串口不通时写指令能不能到得了都说不准，
    而它会动到手指。fail-closed。
    """
    if status is None:
        return None, ("读不到 `0x0200`（串口没通 / 夹爪没上电）—— **一个寄存器都不写**。"
                      "串口不通时写指令能不能到得了都说不准，而它会动手指")
    if status == ag95_init_close.INIT_READY:          # 1
        return (["--close"],
                "已初始化（0x0200=1）⇒ **只合拢**，不重走全行程自检")
    if status == 0:
        return (["--go", "--close"],
                "未初始化（0x0200=0）⇒ 初始化 + 合拢，一步到位")
    if status == 2:
        return None, ("`0x0200=2`（**初始化正在进行**）—— **不写**。"
                      "这时候再发一次初始化是往半途里插一脚；等它做完再跑一次本命令")
    return None, ("`0x0200=%s` —— 只认得 0 / 1 / 2，这个值不敢猜，**一个寄存器都不写**"
                  % status)


def gripper_init_payload_lines(argv):
    """写之前要**先给人看**的完整报文。**纯函数** → 行列表。

    人确认的是**那一条**：`--close` 与 `--go --close` 动的是**不同的寄存器、不同的
    行程**（一个只合拢、一个先张开再合上），报文必须照实分开写 —— 否则「确认」确认
    的不是将要发生的事。力/目标值取自 `ag95_init_close` 自己的常量（**同源**）。
    """
    init = "--go" in argv
    close = "--close" in argv
    return [
        "    ⚠️  马上要**写夹爪寄存器**（红线 2 点名的「初始化 / 开合」，**会动手指**）：",
        "        脚本                 python3 %s %s"
        % (GRIPPER_SCRIPT, " ".join(argv)),
        "        写 0x%04X（初始化）   %s" % (
            ag95_init_close.REG_INIT,
            "**是** —— 一次**全行程自检**：两指先走到全开（1000）再合上"
            if init else "**否**（不重走自检，两指不做全行程）"),
        "        写 0x%04X / 0x%04X  %s" % (
            ag95_init_close.REG_FORCE, ag95_init_close.REG_TARGET,
            "**是** —— 力 %d N、目标 %d 千分（全闭）⇒ **合拢**；"
            "指尖沿法兰轴**再探出 15.6 mm**"
            % (ag95_init_close.FORCE_N, ag95_init_close.TARGET_PERMILLE)
            if close else "**否**"),
        "    授权只认**这一次**：任务描述 / 环境变量 / 上一次给过的，都不算。",
        "    跑之前请确认（红线 2 的物理事实）：**急停就在手边**，"
        "**两指整个行程里没有手、也没有遮挡**。",
    ]


def run_gripper_init(argv, timeout=GRIPPER_INIT_TIMEOUT, runner=None, repo=REPO):
    """跑 `ag95_init_close.py`（**子进程**，本文件不重实现写寄存器）。

    返回 `(rc, 输出)`；起不来 / 超时 ⇒ `(None, 说明)`。`runner` 可注入 ⇒ 离线可测。
    """
    cmd = [sys.executable, os.path.join(repo, GRIPPER_SCRIPT)] + list(argv)
    run = subprocess.run if runner is None else runner
    try:
        p = run(cmd, cwd=repo, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, ("超时（%gs）—— 进程已放弃等待，**但它可能还在动**。"
                      "先确认两指停了再往下" % timeout)
    except OSError as exc:
        return None, "%s: %s" % (type(exc).__name__, exc)
    return p.returncode, ((p.stdout or "") + (p.stderr or "")).strip()


def judge_gripper_init(record):
    """跑完之后这一项怎么算。**纯函数** → `(status, detail)`。

    没跑 ⇒ `(None, "")`（没有这一项，不是绿）。跑了 ⇒ 按脚本退出码给说法：
    非 0 一律 **FAIL**，文案带上脚本自己那几行 —— 它会说清是哪一步卡住的
    （串口打不开 / 未初始化 / 写失败），转述会丢信息。
    """
    if not record:
        return None, ""
    if record.get("dry_run"):
        return SKIP, ("`--dry-run` ⇒ **一个寄存器都没写**（这是「没做」，不是绿）。"
                      "实跑时会按读到的 0x0200 选最小一档，并在写之前把完整报文"
                      "摆出来要一次**当次**确认")
    rc, out = record.get("rc"), record.get("out") or ""
    note = record.get("note") or ""
    # 取脚本自己那几行结论（它用 ❌/✅ 起头）—— 转述会丢信息。
    tail = "；".join([ln.strip() for ln in out.splitlines()
                      if ln.strip() and ("❌" in ln or "✅" in ln)][-2:])
    if rc is None:
        return FAIL, ("**没跑成**：%s%s" % (out, ("（%s）" % note) if note else ""))
    if rc != 0:
        return FAIL, ("脚本退出码 %d（%s，不是「没做」）—— %s"
                      % (rc, note or "环境不满足", tail or out[-300:]))
    return PASS, ("已执行，脚本退出码 0（%s）；P1 那次只读读数就是**做完之后**的状态"
                  % (note or ""))


def ask_gripper_authorization(prompt_fn=input, isatty=None):
    """真写之前**当场**再确认一次。**没有无人值守模式**（非终端直接拒）。

    与 `ask_authorization` / `board_gate._run_repair` 同一条纪律。区别只在于
    这里问的是**手指**：所以确认的是那三件物理事实，不是「要不要继续」。
    """
    isatty = sys.stdin.isatty if isatty is None else isatty
    if not isatty():
        print("    ❌ 写夹爪寄存器需要**当次**交互确认，而这里不是终端。"
              "**没有**无人值守模式。")
        return False
    print("        请确认：这是**当次**授权、**急停就在手边**、"
          "**两指整个行程里没有手也没有遮挡**。")
    try:
        ans = prompt_fn("        确认写夹爪寄存器？(yes) > ")
    except EOFError:
        return False
    if str(ans).strip().lower() != "yes":
        print("        未确认 —— **一个寄存器都不写**。")
        return False
    return True


def gripper_init_step(port, read_status=None, ask=None, run=None, out=print):
    """读 `0x0200` → 选最小一档 → 摆报文 → **当次确认** → 跑。

    返回 `{"status","argv","rc","out","note"}` 或 `None`（没写）。**依赖全可注入**
    ⇒ 整条链离线可测（含「没确认就一个寄存器都不写」）。
    """
    read_status = board_lib.gripper_state if read_status is None else read_status
    ask = ask_gripper_authorization if ask is None else ask
    run = run_gripper_init if run is None else run

    st, why = read_status(port)
    argv, note = gripper_init_plan(st)
    out("  夹爪初始化：%s" % note)
    if argv is None:
        out("      ⛔ **不写**。%s" % (("（%s）" % why) if why else ""))
        return {"status": st, "argv": None, "rc": None, "out": note, "note": note,
                "refused": True}

    out("  从 0x0200=%s 选出的那一档：`%s`" % (st, " ".join(argv)))
    for ln in gripper_init_payload_lines(argv):
        out(ln)
    if not ask():
        return {"status": st, "argv": argv, "rc": None, "out": "未确认",
                "note": note, "refused": True}
    rc, text = run(argv)
    return {"status": st, "argv": argv, "rc": rc, "out": text, "note": note}


def attach_log_missing_note(log_path):
    """`--attach` 找不到日志时那条 FAIL 的文案。**纯函数**（好让离线断言钉得住它）。

    这条 FAIL 在**只读预检**用法下会稳定出现，而原文案
    「--attach 要判分就得有日志，而 X 不存在」是**实话但不够**：它没区分
    「`--log` 指错了地方」与「**根本没有栈可附着**」—— 后者恰恰是
    「拿 `--attach` 当零副作用体检」时最常见的形态（栈没起，P3 的 PLAYING 本来也等不到）。
    两种都写出来，别让现场自己猜。判据本身（找不到日志 ⇒ FAIL）**没变**。
    """
    return ("找不到日志 %s —— `--attach` 是**附着别人起的栈**，判分要读那份日志。"
            "两种情形：\n"
            "      · 栈起着、日志在别处 ⇒ 用 `--log <那份日志>` 指过去；\n"
            "      · **你根本没起过栈**（只想拿 --attach 当零副作用体检）⇒ "
            "这条 FAIL 就是这个意思：**没有栈可附着**，这次体检到此为止。"
            % log_path)


def ask_authorization(prompt_fn=input, isatty=None):
    """交棒前**再问一次**（AGENTS.md 红线 6：真机动作逐次授权）。

    与 `board_gate._run_repair` 同一条纪律：非交互终端**直接拒绝**，**本脚本没有
    无人值守模式**。`--authorize-motion` 是你**表达意图**；这一问是让你在**臂已经
    上电、栈已经起着**的那一刻再确认一次。
    """
    isatty = sys.stdin.isatty if isatty is None else isatty
    if not isatty():
        print("    ❌ 交棒需要交互确认，而这里不是终端。**没有**无人值守模式。")
        return False
    print("    ⚠️  接下来把指挥权交给这套栈：规划器会**真的驱动机械臂**。")
    print("        请确认：这是**当次**授权、**急停就在手边**、**工作区已隔离**。")
    try:
        ans = prompt_fn("        确认交棒？(yes) > ")
    except EOFError:
        return False
    if str(ans).strip().lower() != "yes":
        print("        未确认 —— 不交棒。")
        return False
    return True


def handoff_mode(isatty):
    """交棒的「当次授权」走哪一条路。**纯函数** → `'keyboard'` / `'pendant'`。

    · `'keyboard'` —— 有交互终端（**人自己起的那一轮**）：敲 `yes`。
    · `'pendant'`  —— 没有交互终端（**AI 驱动的那一轮**）：不敲键盘，证明改由
      **示教器上那次物理按压**承担，由 `needs_repress()` / `ensure_in_the_loop()`
      **去验**（验不了就要求 STOP → PLAY 两步握手）。

    ⚠️ **为什么必须有 `'pendant'` 这一路**：用户 2026-10-07 的要求是这个入口
    「**要能支持 AI 自己跑，也要能支持人手动跑**」。旧版在非终端上**直接拒绝**
    （`ask_authorization` 里那句「没有无人值守模式」），把 AI 驱动整个堵死 ——
    而 AI 起栈时 stdin 恰恰不是终端。堵死它并不能换来安全：**能敲键盘的人
    未必在臂旁边**，而按示教器的人一定在。

    ⚠️ `'pendant'` **不等于无人值守**：它要求人**物理上在示教器前动手**。
    纯靠环境的判据（环境变量 / 任务描述 / 上一次的记忆）仍然一律不算数
    —— 见 `find_authorization_keys()`。
    """
    return "keyboard" if isatty() else "pendant"


def confirm_handoff(mode, prompt_fn=input, isatty=None):
    """交棒确认（红线 6 的「当次授权」那一关）。→ `(过了没, 说明)`。

    `keyboard` 走 `ask_authorization()`（原样，一个字都没改）；`pendant` **不提问**，
    只把「接下来由什么证明」印清楚，然后**交给下面的复核**去验 ——
    那一关不通就照样不交棒。
    """
    if mode == "keyboard":
        ok = ask_authorization(prompt_fn=prompt_fn,
                               isatty=(lambda: True) if isatty is None else isatty)
        return ok, ("有交互终端 —— 已敲 `yes`" if ok
                    else "给了 `--authorize-motion`，但终端上没有确认 ⇒ 不交棒")

    print("    ℹ️ 没有交互终端（AI 驱动）⇒ **不敲键盘**，确认改由**示教器**承担：")
    print("       你起栈时按的那次 PLAY 会被**核对**（起栈前不是 PLAYING、")
    print("       起栈后是 PLAYING ⇒ 那一下只可能是人按的）。")
    print("       核对不了（上一轮的残 PLAYING / 起栈前读不到控制柜）就会")
    print("       要你**当场补一次 STOP → PLAY** —— 两步都看见才算。")
    return True, "无交互终端：确认由**示教器上的物理按压**承担（下一步复核它）"


def stop_stack(clock=time.time):
    """收掉**别人起的**栈（`--stop`）→ 收完还剩几条。

    ⚠️ 与 `board_lib.teardown()` 的区别：`teardown` 收的是**我们自己起的**进程组，
    所以它敢 `killpg(proc.pid)`。这里面对的是**别人的**进程，pgid 可能落在别人的
    会话里 —— `killpg` 会误伤，所以只**逐个 pid** 发信号。「谁是栈进程」的判据仍然
    用 `board_lib.leftovers()` 那一份（含它修好的自我排除），不另抄一套。
    """
    stale = board_lib.leftovers()
    if not stale:
        return []
    for line in stale:
        with contextlib.suppress(ValueError, ProcessLookupError, PermissionError):
            os.kill(int(line.split()[0]), signal.SIGTERM)
    deadline = clock() + 15
    while clock() < deadline and board_lib.leftovers():
        time.sleep(1)
    for line in board_lib.leftovers():
        with contextlib.suppress(ValueError, ProcessLookupError, PermissionError):
            os.kill(int(line.split()[0]), signal.SIGKILL)
    time.sleep(2)
    return board_lib.leftovers()


# =====================================================================
# 主流程
# =====================================================================

def run(argv=None, *, prompt_fn=input, isatty=None, clock=time.time):
    ap = _parser()
    a = ap.parse_args(argv)
    # 交互与否**只解析一次**：P0 的写路径闸门与 P5 的交棒确认必须看到同一件事
    # （两处各查一次 isatty，中间被换掉的话就是自证式判据）。
    isatty = sys.stdin.isatty if isatty is None else isatty

    print("=" * 74)
    print("  board_run —— 任务无关的真机启动器")
    print("  永不发 play/power/stop；控制器闸门默认只读；退出码 0/1/2（见文件头）")
    print("=" * 74)

    if a.stop:
        print("\n—— 收栈（--stop）——")
        left = stop_stack(clock=clock)
        if left:
            print("  [FAIL] 收完还有 %d 条：" % len(left))
            for s in left:
                print("      " + s[:120])
            print("\n  " + EXIT_LEGEND)
            return 2
        print("  [PASS] 已收干净（判据用 board_lib.leftovers()，不另抄一套）")
        print("\n  ⚠️ 收栈**不**代表臂停下：示教器上的程序还开着。要停臂，在示教器上"
              "按 STOP —— 那是人的动作。")
        return 0

    if not a.task:
        print("\n❌ 缺 --task（`--stop` 之外都要给）：例如 `--task a1`")
        return 2

    report = {"rows": [], "absent": {}}
    task = None
    site = None
    # 夹爪初始化那一次的记录（`--init-gripper` 才会被赋值）。放在 `def finish` 之前：
    # `finish` 是闭包，读的是**调用时**的值 —— 这样每一处 `return finish(...)` 都自动
    # 带上它，不必在十几个出口各传一遍（漏一个，报告里那一栏就会静默变成「没做」，
    # 而那正是这一栏要防的误读）。
    gripper_init = None
    params, sources = {}, {}

    def add(title, item, absent=None):
        report["rows"].append((title, item, absent))
        if absent:
            report["absent"][item.name] = absent
        print()
        for ln in render_item(item, absent):
            print(ln)
        return item

    def finish(code, **kw):
        kw.setdefault("gripper_init", gripper_init)
        return _finish(report, code, a, task=task, **kw)

    # ================================================== P0 自检（静态）
    print("\n—— %s ——" % S0)

    try:
        task = load_task_file(find_task_file(a.task))
    except TaskError as exc:
        add(S0, Item("P0.任务描述", FAIL, str(exc)))
    if task is not None:
        st, det = judge_problems("任务描述", validate_task(task.raw), task.rel)
        add(S0, Item("P0.任务描述", st, det))
        print("\n  任务：%s（%s）" % (task.name, task.rel))
        print("  入口：%s   相机来源：%s   start_delay：%gs"
              % (task.entry, task.camera_source, task.start_delay))
        print("  期望：槽位 %d 个 / 至少 %d 个候选   收工标记：%r"
              % (task.slots_total, task.min_candidates, task.finish_marker))
        print("  判分：%s（判据本体不在本启动器里）" % task.score_ref)
        add(S0, Item("P0.判分函数", *judge_score_refs(task)))

    try:
        site = board_lib.load_site(path=board_lib.site_path(a.site))
    except board_lib.SiteError as exc:
        add(S0, Item("P0.站点配置", FAIL,
                     "%s\n      ⚠️ **不会**回落 board_a1.CAM —— 那份是仓库里的孤本"
                     "记录，不是现场核对过的现行值" % exc))
    if site is not None:
        st, det = judge_problems("站点配置", board_lib.validate_site(site), site.path)
        add(S0, Item("P0.站点配置", st, det))

    ros_env = None
    if a.dry_run:
        add(S0, Item("P0.ROS 环境", SKIP, "没查（--dry-run 不跑任何子进程）"),
            absent="--dry-run 只看静态（文件与命令）")
        add(S0, Item("P0.域号一致", SKIP, "没查（--dry-run 不跑任何子进程）"),
            absent="--dry-run 只看静态（文件与命令）")
    else:
        # 环境**算出来**，不继承调用者的。这不是洁癖：板上 `.bashrc` 只 source 了
        # 系统 ROS 与**另一个**工作区（~/yahboom_ws），人开个正常终端跑，本项目的
        # install 也不在搜索路径上 —— 而旧判据照样绿，失败推迟到 P1 并伪装成
        # 「缺包」。见 build_ros_env 的实测记录。这一份 env 贯穿 P0→P1→P2。
        ros_env, env_note = build_ros_env(install_setup=INSTALL_SETUP)
        if ros_env is None:
            add(S0, Item("P0.ROS 环境", FAIL, env_note))
        else:
            add(S0, Item("P0.ROS 环境", *judge_ros_env(
                ros_env,
                shutil.which("ros2", path=ros_env.get("PATH")) is not None,
                INSTALL_SETUP, sourced=env_note)))
        if site is not None:
            add(S0, Item("P0.域号一致",
                         *judge_domain(ros_env or os.environ, site)))

    if task is not None and site is not None:
        params, sources, conflicts = resolve_launch_params(task, site)
        add(S0, Item("P0.参数拼装", *judge_problems("参数拼装", conflicts)))
        _print_plan(params, sources)
    elif task is not None:
        # 站点读不到 ⇒ 拼不出完整参数集，但**已经知道的那些**还是要摆出来：
        # 否则「我到底会发什么」在最需要它的时候（配置没配好）反而看不见。
        tparams, tsources = task_params(task)
        _print_plan(tparams, tsources,
                    note="站点配置没读到 ⇒ 下面**只有任务描述能定的那部分**；"
                         "robot_ip / planner_params_file / camera_* 外参 / "
                         "publish_camera_tf 都还不知道")

    # 红线 3：运行记录（`--log` / `--json-report`）**不许落进公开仓**。
    # 在这里判 = **起栈之前**，`--log` 还没被 `open(path,"w")` 打开过。
    add(S0, Item(OUTPUT_PATH_ITEM, *judge_output_paths(
        os.path.expanduser(a.log) if a.log else None, a.json_report)))

    # 红线 6：写路径要**当次**授权 ⇒ 起栈**之前**就问清楚有没有交互终端。
    # （要发的那条报文先完整摆出来 —— 人确认的是**那一条**；确认动作在 board_gate
    #   里、臂已上电的那一刻再问一次。）
    if a.repair_controller:
        if task is not None:
            print()
            for ln in switch_payload_lines(task):
                print(ln)
        add(S0, Item(WRITE_PATH_ITEM, *judge_write_path(isatty)))

    # 红线 6 第二条写路径（夹爪寄存器）：同一条纪律、同一个判定函数。
    # 报文**取决于读到的 0x0200**（已初始化就只合拢）⇒ 这一项列在这里，报文在真写
    # 之前才摆得出来（就在下面那道 S0 闸门之后）。
    if a.init_gripper:
        add(S0, Item(GRIPPER_INIT_ITEM, *judge_write_path(
            isatty, "--init-gripper",
            fire_note="真写之前会**按读到的 0x0200 选最小的一档**、把完整报文"
                      "（哪些寄存器、走多少行程）摆出来，再要你敲一次 yes")))

    code, _, _ = decide(counted(report, S0), None, launched=False)
    if code != 0:
        return finish(code, launched=False)

    # ---- 夹爪初始化：**在读 P1 之前**做完，这样 P1 那次只读读数反映的就是
    #      「做完之后」的状态（一个读数同时回答「写成了吗」与「合规吗」）。
    gripper_init = None
    if a.init_gripper:
        print("\n—— 夹爪初始化（`--init-gripper`：红线 6 写路径，会动手指）——")
        if a.dry_run:
            print("  （`--dry-run`：**一个寄存器都不写**。实跑时按读到的 0x0200 选档：）")
            for st in (1, 0):
                _argv, _note = gripper_init_plan(st)
                print("      0x0200=%s ⇒ %s" % (st, _note))
            print("      0x0200=2 ⇒ **不写**（初始化进行中，再发一次是往半途插一脚）")
            print("      读不到   ⇒ **不写**（串口不通时指令到没到都说不准）")
            gripper_init = {"dry_run": True}
        else:
            gripper_init = gripper_init_step(site.gripper_port if site else None)
            add(S0, Item(GRIPPER_INIT_ITEM, *judge_gripper_init(gripper_init)))
            code, _, _ = decide(counted(report, S0), None, launched=False)
            if code != 0:
                return finish(code, launched=False)

    # ================================================== P1 前置（只读）
    print("\n—— %s ——" % S1)
    facts = collect_preflight(site, do_net=not a.dry_run,
                              override=a.override_freshness, env=ros_env)
    for it in judge_preflight(facts):
        if a.attach and it.name in ATTACH_ABSENT:
            add(S1, it, absent="--attach：栈就是别人起的")
        elif a.dry_run and it.name in DRY_RUN_ABSENT_P1:
            add(S1, it, absent="--dry-run 只看静态（文件与命令）")
        else:
            add(S1, it)

    code, _, _ = decide(counted(report, S1), None, launched=False)
    if code != 0:
        return finish(code, launched=False)

    # ================================================== P2 起栈
    print("\n—— %s ——" % S2)
    cmd = build_launch_cmd(task.entry, params)
    print("  命令（**这一条就是权威**；每一项是谁给的见上面 P0 的「参数集」）：")
    print("    " + " \\\n      ".join(cmd))
    _print_effective(site, params)

    if a.dry_run:
        # ⚠️ `P2.槽位数` **不在这里缺席**（2026-10-06 修）。它只**读一个文件**
        #    （`read_planner_yaml`），不跑子进程、不连设备 —— 完全在 `--dry-run`
        #    的合同之内。原来写的是「要读 planner_params_file，这一档留给真跑」：
        #    **理由本身是错的**，而代价很贵 —— 任务描述与 `planner_params_file` 的
        #    **槽位数不一致**这件事，本来只有**上板起栈**那一刻才暴露，而槽位数正是
        #    判分用的量（「已放置 N/N」「槽位已用尽（N 个）」）⇒ 形态是
        #    「跑完一整轮才发现判错了」。本轮它就是这么被抓到的（站点指向 8 槽的文件、
        #    `a1-noplan` 期望 4 槽），而那时我**人已经在板上**了。
        #    ⇒ 现在 `--dry-run` 就会红，也就是**在桌面上、起栈之前**就能发现。
        for nm, det in (("P2.声明集",
                         "要跑 `ros2 launch … --show-args` 才拿得到声明集，"
                         "而 --dry-run 不跑任何子进程"),
                        ("P2.起栈", "--dry-run 不起栈")):
            add(S2, Item(nm, SKIP, det), absent="--dry-run 只看静态（文件与命令）")
        add(S2, judge_place_slots(task, site))
        code, _, _ = decide(counted(report, S2), None, launched=False)
        print()
        print("  ❌ 注意：--dry-run 退 0 只表示「计划拼得出来」，**不代表**任何闸门"
              "验过 —— 上面每一项都标着「声明缺席」。")
        return finish(code, launched=False, dry_run=True)

    env = dict(ros_env or os.environ, ROS_DOMAIN_ID=str(site.ros_domain_id),
               PYTHONUNBUFFERED="1")

    declared, failed = set(), []
    for e in task.declared_entries:
        rc, out = run_show_args(e, env)
        if rc != 0:
            tail = out.strip().splitlines()[-1][:160] if out.strip() else "无输出"
            failed.append("%s：rc=%s；%s" % (e, rc, tail))
            continue
        declared |= parse_declared_args(out)
    if failed:
        add(S2, Item("P2.声明集", SKIP,
                     "这几条 `--show-args` 没跑成 ⇒ 声明集不完整，**没验到**：\n"
                     + "\n".join("      " + x for x in failed)))
    else:
        add(S2, Item("P2.声明集", *check_declared(list(params), declared)))

    add(S2, judge_place_slots(task, site))

    code, _, _ = decide(counted(report, S2), None, launched=False)
    if code != 0:
        return finish(code, launched=False)

    # ---- 真的起栈 --------------------------------------------------------
    log_path = os.path.expanduser(a.log or ("~/board_run_%s.log" % task.name))
    proc = None
    logf = None
    t_launch = None
    if a.attach:
        if not os.path.exists(log_path):
            add(S2, Item("P2.起栈", FAIL, attach_log_missing_note(log_path)))
            return finish(2, launched=False)
        print("\n  （--attach：不起栈，只附着已起的栈；日志 %s）" % log_path)
    else:
        print("\n  起栈中…日志 -> %s" % log_path)
        logf = open_log(log_path)
        try:
            proc = subprocess.Popen(cmd, cwd=REPO, stdout=logf,
                                    stderr=subprocess.STDOUT, env=env,
                                    start_new_session=True)
        except OSError as exc:
            logf.close()
            add(S2, Item("P2.起栈", FAIL, "起栈失败：%s" % exc))
            return finish(2, launched=False)
        t_launch = clock()
        add(S2, Item("P2.起栈", PASS,
                     "已起（pid=%d，独立进程组；日志 %s）" % (proc.pid, log_path)))

    launched = proc is not None
    gate_dir = tempfile.mkdtemp(prefix="board_run_gate_")
    rec3 = rec4 = None
    verdict = None
    verdict_state = None
    repair_exercised = None
    code = 2
    try:
        # ============================================== P3 人在环 + 控制器闸门
        print("\n—— %s ——" % S3)
        print()
        print("=" * 66)
        print("👉 现在去示教器按**播放**（URCap 的 External Control 程序）。")
        print("   本启动器**不发** play / power / stop —— 那是人的动作（红线 6）。")
        print("=" * 66)
        ok, d, waited = wait_for_playing(site.robot_ip, a.play_timeout, clock=clock)
        if ok:
            # `facts["cabinet"]` 是 **P1**（＝起栈**之前**）读到的那一份 —— 只有它
            # 能回答「这一条是不是人按的」（理由见 `residual_playing_note`）。
            # `vacuous` 与上面那句 `residual_playing_note(...)` 是**同一根判据**的
            # 两个出口（给人读 / 给机器读），必须同源 —— 所以这里也传同一个
            # `facts["cabinet"]`（＝**P1、起栈之前**读到的那一份）。
            add(S3, Item("P3.示教器 PLAYING", PASS,
                         "%.0fs 后 %s（原文 %r）%s"
                         % (waited, board_lib.dash_program_state(d),
                            str(d.get("programState", "")).strip(),
                            residual_playing_note(facts.get("cabinet"))),
                         vacuous=playing_is_vacuous(facts.get("cabinet"))))
        else:
            got = str((d or {}).get("programState", "")).strip()
            add(S3, Item("P3.示教器 PLAYING", FAIL,
                         "%.0fs 内没等到 PLAYING（原文 %r 读成 %r）—— 次序是"
                         "「起栈 → 按播放 → 拉回控制器」"
                         % (a.play_timeout, got,
                            board_lib.dash_program_state(d) or "连不上控制柜")))

        j3 = os.path.join(gate_dir, "p3.json")
        argv3 = gate_argv(j3, only="controllers",
                          trajectory_action=task.trajectory_action,
                          repair=a.repair_controller, fast=True)
        rc3, rec3, why3 = run_gate(argv3, env, timeout=240.0)
        if rec3 is None:
            add(S3, Item("P3.控制器闸门", SKIP,
                         "board_gate 没给出记录：%s" % why3))
        else:
            repair_exercised = bool(rec3.get("repair_exercised"))
            try:
                ctrl3 = None
                for it, ab in stage_from_gate(rec3, "P3", GATE_STAGE_ITEMS["P3"]):
                    add(S3, it, absent=ab)
                    if it.name == "P3.controllers":
                        ctrl3 = it.status
                # 红线 6 第三条：切换**执行过** ⇒ 必须给「生效 / 未生效」一个说法
                # （board_gate 的复核本身就是只读的；这里只负责不许把它读成绿）。
                st, det = judge_repair_outcome(repair_exercised, ctrl3)
                if st is not None:
                    add(S3, Item(REPAIR_ITEM, st, det))
                print_repair_hint(ctrl3, repair_exercised, task.name)
            except GateError as exc:
                add(S3, Item("P3.控制器闸门", SKIP, str(exc)))

        code, teardown, why = decide(counted(report, S3), None, launched=launched)
        if code != 0:
            print("\n  ⛔ %s" % why)
            if teardown:
                add(S3, Item("P3.收栈", PASS, "抢在规划器定时器之前收掉这套栈"))

        # ============================================== P4 起栈后预检
        if code == 0:
            print("\n—— %s ——" % S4)
            # ⚠️ **先等 `start_delay` 那一段过去**，再体检。不等的话，人按播放**按得快**
            #    就会让 P4 落进那一段，把「本来就还没起」的相机/检测器/感知记成 FAIL
            #    （2026-10-06 第 2 次实测，理由与数据见 `p4_wait_seconds`）。
            _left = p4_wait_seconds(t_launch, clock(), task.start_delay)
            _t_wait0 = clock()
            if _left > 0.0:
                print("  先等 %.0fs：`start_delay=%gs` 那一段里 realsense/detector/"
                      "perception **本来就还没起**，抢在它前面体检会把三项记成 FAIL"
                      % (_left, task.start_delay))
                _end = clock() + _left
                while True:
                    _r = _end - clock()
                    if _r <= 0.0:
                        break
                    time.sleep(min(15.0, _r))
                    _r = _end - clock()
                    if _r > 0.0:
                        print("    …还差 %.0fs" % _r)
            _waited4 = clock() - _t_wait0
            if t_launch is None:
                _note4 = "没有起栈时刻（--attach/--dry-run）⇒ 无从等起"
            elif _waited4 > 0.0:
                _note4 = ("等了 %.0fs（start_delay=%gs，起栈后共 %.0fs）"
                          % (_waited4, task.start_delay, clock() - t_launch))
            else:
                _note4 = ("没等（start_delay=%gs，起栈后已经过去 %.0fs）"
                          % (task.start_delay, clock() - t_launch))
            add(S4, Item("P4.等启动延迟", PASS, _note4))
            j4 = os.path.join(gate_dir, "p4.json")
            # `--pick-zone` **只在规划器不起的档**才递：那时活读必然拿不到，
            # 不递就恒 SKIP（理由与出处见 `pick_zone_arg` 的 docstring）。
            _pz_yaml, _ = read_planner_yaml(site.planner_params)
            argv4 = gate_argv(j4, slots_total=task.slots_total,
                              min_candidates=task.min_candidates,
                              expect_params=task.expect_params,
                              allow_skip=task.allow_skip,
                              trajectory_action=task.trajectory_action,
                              pick_zone=pick_zone_arg(task.allow_skip, _pz_yaml,
                                                      params))
            rc4, rec4, why4 = run_gate(argv4, env, timeout=600.0)
            if rec4 is None:
                add(S4, Item("P4.体检", SKIP, "board_gate 没给出记录：%s" % why4))
            else:
                if "repair_exercised" in rec4:
                    repair_exercised = (bool(repair_exercised)
                                        or bool(rec4["repair_exercised"]))
                ctrl4 = None
                try:
                    for it, ab in stage_from_gate(rec4, "P4",
                                                  GATE_STAGE_ITEMS["P4"]):
                        add(S4, it, absent=ab)
                        if it.name == "P4.controllers":
                            ctrl4 = it.status
                except GateError as exc:
                    add(S4, Item("P4.体检", SKIP, str(exc)))
                print_repair_hint(ctrl4, repair_exercised, task.name)
                _print_margin(rec4, task)
            code, _, why4b = decide(counted(report, S4), None, launched=launched)
            if code != 0:
                print("\n  ⛔ %s" % why4b)

        # ============================================== P5 交棒
        if code == 0:
            print("\n—— %s ——" % S5)
            print("  ℹ️ 闸门只回答「这一轮起栈健康吗」，它**不替代**「单步低速 + "
                  "急停就绪的第一次真机动作」。")
            if not a.authorize_motion:
                add(S5, Item("P5.交棒", SKIP,
                             "没有 --authorize-motion ⇒ **任务未交棒**（红线 6：真机"
                             "动作逐次授权）。栈会收掉；退 2 **不许**读成通过"))
                code = 2
            else:
                # ★ 「当次授权」的证明有两条路，见 `handoff_mode()`：
                #   有终端（人手动起的轮次）⇒ 敲 yes；没终端（**AI 驱动**）⇒ 由
                #   示教器上那次物理按压承担。**两条都要人动手**，都不接受
                #   环境变量/任务描述/上次记忆冒充（红线 6）。
                mode = handoff_mode(isatty)
                ok_conf, why_conf = confirm_handoff(mode, prompt_fn=prompt_fn,
                                                    isatty=isatty)
                if not ok_conf:
                    add(S5, Item("P5.交棒", SKIP, why_conf))
                    code = 2
                else:
                    # 记下**这一次的授权是怎么证明的** —— 与 `repair exercised`
                    # 同一个道理：事后要能分清「AI 跑的」还是「人手动跑的」。
                    add(S5, Item("P5.交棒确认", PASS, why_conf))
                # ★ 交棒前复核「人在环」—— **只在这一轮真会动的时候做**。
                # P3 那条绿可能是**残 PLAYING**（上一轮收栈留下的），那时启动器
                # 嘴上说「人在环走通了」而人根本没按 ⇒ 臂会动。这是启动器最该
                # 消灭的一类失误，所以在这里拦住（判据见 `needs_repress`）。
                # ⚠️ 不给 `--authorize-motion` 的那一轮走不到这里 ⇒ R0 的
                # P0–P4 证据**一点不受影响**。
                if code == 0 and needs_repress(counted(report, S3)):
                    ok_loop, why_loop = ensure_in_the_loop(site.robot_ip,
                                                           a.play_timeout, clock=clock)
                    add(S5, Item("P5.人在环复核",
                                 PASS if ok_loop else FAIL, why_loop))
                    if not ok_loop:
                        code = 2
                if code == 0:
                    add(S5, Item("P5.交棒", PASS,
                                 "已授权（当次）；盯日志到收工并按任务描述的判分器判分"))
                    timeout = (a.timeout if a.timeout is not None else
                               (task.timeout if task.timeout is not None else 900.0))
                    print("  盯日志（最多 %.0fs）：%s" % (timeout, log_path))
                    lines, finished = wait_for_finish(log_path, clock() + timeout,
                                                      task.finish_marker, clock=clock)
                    if not finished:
                        print("\n  ❌ %.0fs 内没等到 %r —— 被超时中断"
                              % (timeout, task.finish_marker))
                    verdict_state, verdict = score_round(task, lines)
                    add(S5, Item("P5.判分",
                                 PASS if verdict is True else
                                 (FAIL if verdict is False else SKIP),
                                 verdict_state))
                    if verdict is None:
                        code = 2
                    elif verdict is False:
                        code = 1
                    else:
                        code = 0 if finished else 2
    finally:
        if proc is not None:
            print("\n—— 收栈 ——")
            board_lib.teardown(proc)
            print("  ✓ 收栈完成（⚠️ 收栈不代替在示教器上按 STOP）")
        if logf is not None:
            with contextlib.suppress(OSError):
                logf.close()
        with contextlib.suppress(OSError):
            shutil.rmtree(gate_dir, ignore_errors=True)

    return finish(code, launched=launched, gate_p3=rec3, gate_p4=rec4,
                  repair_exercised=repair_exercised, t_launch=t_launch,
                  t_stop=clock(), verdict_state=verdict_state, log_path=log_path)


# ---------------------------------------------------------------- 小组件

def _parser():
    ap = argparse.ArgumentParser(
        prog="board_run.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "任务无关的真机启动器（P0–P5）。**永不**发 play/power/stop；控制器闸门"
            "默认只读。\n"
            "退出码：0 = 闸门全过且本任务判分通过；1 = 闸门过了、任务跑了但判分没过；\n"
            "        2 = 在交棒之前停下了（有 FAIL / 有 SKIP / 没给 "
            "--authorize-motion）。"),
        epilog=(
            "任务描述：config/tasks/<名>.yaml。仓内那份是**模板/示例**，现场真值放\n"
            "~/.config/ur5-vision-grasp/tasks/（同名时以现场那份为准）。\n"
            "一份任务描述只声明**任务特有**的东西：launch 入口、params、槽位数、判分\n"
            "函数的点号引用；控制柜 IP / 外参 / 参数文件一律从站点配置推导，不在这里复述。"))
    ap.add_argument("--task", default=None,
                    help="任务名（找 config/tasks/<名>.yaml）或直接的 .yaml 路径；"
                         "`--stop` 时不需要")
    ap.add_argument("--site", default=None,
                    help="站点配置路径（默认 $%s 或 %s）—— **必须能读到**；"
                         "读不到/不合法的外参**绝不**回落到 board_a1.CAM"
                         % (board_lib.SITE_ENV, board_lib.SITE_DEFAULT))
    ap.add_argument("--attach", action="store_true",
                    help="不起栈，只对**已经起着**的栈做 P3/P4/P5（配合 --log）")
    ap.add_argument("--authorize-motion", action="store_true",
                    help="声明这是**当次**的真机动作授权（红线 6）。仍是逐次授权："
                         "交棒前会在终端上再问一次，**没有**无人值守模式")
    ap.add_argument("--repair-controller", action="store_true",
                    help="⚠️ 写路径：控制器闸门不过时交给 board_gate 的收敛式修复。"
                         "红线 6 点名了「控制器切换」⇒ 这一档要**当次**授权：P0 会先"
                         "确认有交互终端（没有就退 2，**栈都不起**），发文前把完整报文"
                         "摆出来，臂已上电时还要再确认一次，切完按只读复核判"
                         "「生效 / 未生效」。deactivate 恒空 —— 那一份判定在 "
                         "board_gate 里，本文件原样透传")
    ap.add_argument("--init-gripper", action="store_true",
                    help="⚠️ 写路径：把夹爪弄到「已初始化」（红线 2 点名的"
                         "「初始化 / 开合」，**会动手指**）。与 --repair-controller "
                         "同一条纪律：P0 先确认有交互终端（没有就退 2，**栈都不起**），"
                         "**按读到的 0x0200 选最小的一档**（已初始化 ⇒ 只合拢，"
                         "不重走全行程自检），真写之前把完整报文（哪些寄存器、"
                         "走多少行程）摆出来再要一次**当次**确认，**没有**无人值守"
                         "模式。必须在臂使能、急停在手边、两指行程内无人无遮挡时给。")
    ap.add_argument("--override-freshness", default=None, metavar="理由",
                    help="外参超期时由人显式破例，**必须写下为什么**（原样进报告）")
    ap.add_argument("--timeout", type=float, default=None,
                    help="P5 等收工的秒数上限（默认取任务描述的 expect.timeout，"
                         "再默认 900）")
    ap.add_argument("--log", default=None,
                    help="栈的日志文件（默认 ~/board_run_<任务>.log；"
                         "--attach 时必须已存在）")
    ap.add_argument("--json-report", default=None,
                    help="把这一轮的记录写成 JSON（**必须在仓库外** —— "
                         "本仓是公开副本）")
    ap.add_argument("--play-timeout", type=float, default=300.0,
                    help="起栈后等人在示教器按「播放」的秒数上限（默认 300）")
    ap.add_argument("--stop", action="store_true",
                    help="收掉别人起的栈（逐个 pid 发信号，判据用 board_lib），"
                         "不发任何运动命令")
    ap.add_argument("--dry-run", action="store_true",
                    help="只看会发什么命令、每个值从哪来：**不起栈、不连设备、"
                         "不跑任何子进程**。它退 0 只表示「计划立得住」，"
                         "**不代表**任何闸门验过")
    return ap


def slot_count(vals):
    """`ros__parameters` 里**有几个放置槽**。**纯函数** → `(槽数|None, 说明)`。

    ⚠️⚠️ `place_slots` 是**扁平**的 `[x0, y0, x1, y1, …]`，**不是** `[[x, y], …]`
    —— `planner_node.py:203` 逐字写着这件事，`:213-215` 就是这么解回来的。
    而**为什么**必须扁平，码里也写了（`:204-210`）：**ROS 2 的参数类型系统没有
    嵌套数组**（`rcl_interfaces` 只有扁平的 `double_array_value`），
    `[[x,y], …]` 会让**节点直接起不来**。所以这不是风格问题，是平台的硬约束。
    于是槽位数是 **`len // 2`，不是 `len`**。

    2026-10-06 之前这里写的是 `len(place_slots)`，而**离线夹具**用的是
    `[[x,y], …]` —— 一个真机节点**明确不接受**的形状（`planner_node.py:621` 会
    因为奇数长度直接抛）。夹具与现场差一个形状 ⇒ **判据在夹具上恒对、在真机上恒错**，
    于是这条缺陷藏到今天第一次真读现场文件才露头。**「夹具替代码说谎」的第三次**，
    而且这次连累了一个我差点发出去的错结论（见下面 `judge_place_slots` 的注释）。
    """
    if not isinstance(vals, list) or not vals:
        return None, "没有 place_slots（或不是列表）"
    if any(isinstance(v, (list, dict, tuple)) for v in vals):
        return None, ("place_slots 里有**嵌套**的元素 —— 节点只接受**扁平**的 "
                      "[x0,y0, x1,y1, …]（`planner_node.py:203`；写成 [[x,y], …] "
                      "它会在 float() 上直接抛）")
    if len(vals) % 2:
        return None, ("place_slots 有 %d 个值（**奇数**）—— 它必须成对 "
                      "[x0,y0, x1,y1, …]，节点自己也会因此抛（planner_node.py:621）"
                      % len(vals))
    return len(vals) // 2, ""


def sibling_slots(path, limit=6):
    """`path` 同目录下**别的** planner 参数文件各有多少个槽 —— 「站点指错文件了」的证据。

    **纯函数**（只读文件；任何一步失败都跳过，不抛）。返回 `[(文件名, 槽位数), …]`。

    为什么要有它：这台设备上**同时躺着好几份** planner 参数文件（A1 的、A2 分拣的、
    包内那份仿真默认值），而站点只指得动一份。槽位数对不上时报「有 N 个 / 期望 M 个」
    是**对的但不够** —— 现场要的是「我该指哪一份」。把同目录的候选连同它们的槽位数
    摆出来，这一步就从「再查一遍」变成「看一眼」。
    """
    d = os.path.dirname(os.path.abspath(os.path.expanduser(path or "")))
    me = os.path.basename(path or "")
    out = []
    try:
        names = sorted(os.listdir(d))
    except OSError:
        return out
    for n in names:
        if n == me or not n.endswith((".yaml", ".yml")):
            continue
        vals, _ = read_planner_yaml(os.path.join(d, n))
        n_slots, _ = slot_count((vals or {}).get("place_slots"))
        if n_slots:
            out.append((n, n_slots))
        if len(out) >= limit:
            break
    return out


def judge_place_slots(task, site):
    """P2：`planner_params_file` 里的**槽位数**要等于任务期望的槽位数。

    能推导的不复述：槽位数**不作为任务描述里的唯一真值**，而是从活的那份 yaml 读出来
    对一遍 —— 两边不一致时，A1 的判据（「已放置 N/N」与「槽位已用尽（N 个）」）必然
    判错，而错误会以「判分没过」的形式出现在**跑完之后**。

    ⚠️ 它是**纯文件读**，所以 `--dry-run` 也跑它（2026-10-06 起）—— 见 P2 那段的注释。

    ⚠️ **这条判据 2026-10-06 修过一次实质错误**：它原来数的是 `len(place_slots)`
    （= 值的个数 = 槽数的**两倍**）。第一次真读现场文件时报了
    「有 8 个，期望 4 个」，我差点把它当成「站点指错了文件」发给用户 ——
    而现场那份 `site_real.yaml` 里恰恰是 **4 个槽**（8 个数），与阶段 1 契约
    「放置槽位留 4 个」**完全一致**。**判据错了，不是现场错了。**
    """
    path = site.planner_params
    vals, why = read_planner_yaml(path)
    if vals is None:
        return Item("P2.槽位数", FAIL,
                    "读不到 planner_params_file（%s）：%s —— 真机上不给它 / 给错路径会"
                    "**静默用仿真几何**，先修这个" % (path, why))
    n_slots, bad = slot_count(vals.get("place_slots"))
    if n_slots is None:
        return Item("P2.槽位数", FAIL if "奇数" in bad else SKIP,
                    "%s 的 %s —— 没验到" % (path, bad))
    if n_slots != task.slots_total:
        det = ("%s 的 place_slots 有 %d 个槽（%s 里 %d 个值，扁平两两成对），"
               "任务描述期望 %d 个 —— 两边不一致时判分必然判错"
               "（「已放置 N/N」「槽位已用尽（N 个）」都按**后者**算，而错会以"
               "「跑完一整轮、判分没过」的形式出现）。\n"
               "      ⚠️ **先看台面再改**：这个数不是配置偏好，是"
               "**台面上真的有几个放料位**。\n"
               "      · 台面确实是 %d 个 ⇒ 改任务描述的 `expect.slots_total`；\n"
               "      · 台面是 %d 个 ⇒ 站点 `planner_params` **指错文件了**。"
               % (path, n_slots, path, n_slots * 2, task.slots_total,
                  n_slots, task.slots_total))
        near = sibling_slots(path)
        if near:
            det += ("\n      同目录下别的 planner 参数文件（供你认哪一份是现场的）：\n"
                    + "\n".join("        · %s → %d 个槽" % (n, c) for n, c in near))
        return Item("P2.槽位数", FAIL, det)
    return Item("P2.槽位数", PASS,
                "place_slots %d 个槽（%d 个值，扁平两两成对），与任务描述一致"
                "（读自 %s）" % (n_slots, n_slots * 2, path))


def _print_plan(params, sources, note=None):
    """把「会发哪些参数、每一个是谁给的」摆出来 —— **在 P0 就摆**。

    为什么不等 P2：P0 不过的话根本走不到 P2，于是**最需要看清参数集**的那种失败
    （站点读不到、外参过期）恰恰什么也不披露。披露完备性不许只在顺利的路径上成立。
    """
    print("\n  参数集（%d 项；有效值 = 站点 ⊕ 任务）：" % len(params))
    if note:
        print("    ⚠️ %s" % note)
    for k in sorted(params):
        print("    %-22s %s" % (k, sources.get(k, "?")))


def _print_effective(site, params):
    """把「这一个参数现在到底是多少」摆出来，**逐项注明来源**。

    规矩（`docs/机械臂系统参考.md` §5.5）：`grasp_planner.yaml` 有 **5 个死键**，
    走全车入口时改了**不生效也不报错** ⇒ 有效值 = yaml ⊕ launch，而**真值只有起栈后
    回读**（那是 `board_gate` 的 `params` 项，P4 会做）。
    """
    vals, why = read_planner_yaml(site.planner_params)
    print("\n  有效参数（yaml ⊕ launch；**真值以起栈后回读为准**，见 P4 的 params 项）：")
    if vals is None:
        print("    ⚠️ 读不到 %s：%s" % (site.planner_params, why))
        return
    for k, (v, src) in sorted(effective_params(
            vals, params, PLANNER_KEYS_OF_INTEREST).items()):
        print("    %-22s %-34s ← %s"
              % (k, board_gate._short(v) if v is not None else "—", src))
    print("    （planner_params_file = %s）" % site.planner_params)


def _print_margin(record, task):
    """P4 之后打一句「候选够不够触发判据 2」的提示（**不是判据**）。"""
    try:
        ev = gate_items(record)["targets"].evidence or {}
    except (GateError, KeyError):
        return None
    pts = ev.get("points")
    n = len(pts) if pts else None
    note = candidate_margin_note(n, task.slots_total)
    if note:
        print("  ℹ️ " + note)
    return note


def score_round(task, lines):
    """按任务描述里的点号引用判分 → `(人话, True/False/None)`。"""
    try:
        fn = resolve_callable(task.score_ref)
    except CallableError as exc:
        return ("❌ 判分器解不出来：%s\n   （**不会**静默跳过判分 —— 跳过就等于"
                "「没判分却报绿」）" % exc, None)
    try:
        v = fn(lines, task.slots_total)
    except Exception as exc:                        # noqa: BLE001
        return "❌ 判分器抛异常：%s: %s" % (type(exc).__name__, exc), None

    text = ["判分器 %s（判据本体不在本启动器里）" % task.score_ref]
    if task.explain_ref:
        try:
            text.append(str(resolve_callable(task.explain_ref)(v, task.slots_total)))
        except Exception as exc:                    # noqa: BLE001
            text.append("（explain 出不来：%s: %s）" % (type(exc).__name__, exc))
    elif isinstance(v, dict):
        text.append("  %r" % (v,))
    ok = read_verdict(v)
    if ok is None:
        text.append("  ⚠️ 判分结果里没有 `ok` 键 —— **形状不对，判不了**，按不通过处置")
    return "\n".join(text), ok


def _finish(report, code, a, *, task=None, gate_p3=None, gate_p4=None,
            repair_exercised=None, gripper_init=None, t_launch=None, t_stop=None,
            verdict_state=None, log_path=None, launched=False, dry_run=False):
    print()
    print("—— 结论 ——")
    for ln in render_summary(counted(report), code, report["absent"],
                             verdict_state, dry_run=dry_run):
        print(ln)
    print("  repair exercised: %s%s"
          % ("yes" if repair_exercised else "no",
             "" if repair_exercised else
             "   ← 没被绊倒时的绿是**空的**，别读成「验证过了」"))
    # 同一条规定：没被绊倒 / 没给开关时的绿是**空的**。这一栏尤其要说清 ——
    # 「夹爪是好的」有两种来路（**刚写的** / **本来就是**），报告里必须分得开。
    if a.init_gripper:
        if gripper_init is None:
            print("  gripper init exercised: 未走到（在上面被挡下了）")
        elif gripper_init.get("dry_run"):
            print("  gripper init exercised: dry-run   ← **一个寄存器都没写**")
        elif gripper_init.get("refused"):
            print("  gripper init exercised: no（拒绝：%s）"
                  % (gripper_init.get("note") or "未确认"))
        else:
            print("  gripper init exercised: yes（读了 0x0200=%s ⇒ `%s`，脚本退出码 %s）"
                  % (gripper_init.get("status"),
                     " ".join(gripper_init.get("argv") or []),
                     gripper_init.get("rc")))
    else:
        print("  gripper init exercised: no（没给 --init-gripper）"
              "   ← P1 那次读数只说明**当时**是什么状态，不说明是谁弄成那样的")
    if launched:
        moved, note = judge_handoff_window(
            t_launch, t_stop, task.start_delay if task else 0.0,
            planner_alive(task))
        print("  " + ("🔴 " if moved else "• ") + note)
        print("  收栈不代替在示教器上按 STOP —— 收的是这套栈，不是控制柜里的程序。")
    elif dry_run and code == 0:
        print("  （--dry-run：栈没有起，也不该起；上面的绿只说明**计划拼得出来**）")
    else:
        print("  ⛔ 栈**没有起**（闸门没过就不起栈，fail-closed）。")

    if a.json_report:
        _write_report(a, report, code, task, gate_p3, gate_p4, repair_exercised,
                      gripper_init, t_launch, t_stop, verdict_state, log_path,
                      launched)
    print("=" * 74)
    return code


def _write_report(a, report, code, task, gate_p3, gate_p4, repair_exercised,
                  gripper_init, t_launch, t_stop, verdict_state, log_path,
                  launched):
    if path_in_repo(a.json_report):
        # 第二道锁：`P0.输出路径` 已经会退 2、根本走不到这里。留着是因为这条路径
        # 通向**红线 3**（公开仓），多一道便宜的锁值得；但它**不是**主判据 ——
        # 主判据在 P0，那里才能做到「早退、不写、不丢记录」。
        print("  ❌ --json-report 指向仓库内（%s）—— 本仓是公开副本，运行记录必须写在"
              "仓库外。**没有写**。" % a.json_report)
        return
    moved, note = judge_handoff_window(t_launch, t_stop,
                                       task.start_delay if task else 0.0,
                                       planner_alive(task))
    rec = {
        "script": "board_run.py",
        "when": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "task": getattr(task, "name", None),
        "task_path": getattr(task, "path", None),
        "site": board_lib.site_path(a.site),
        "log": log_path,
        "launched_stack": bool(launched),
        "authorized_motion": bool(a.authorize_motion),
        "repair_exercised": (None if repair_exercised is None
                             else bool(repair_exercised)),
        # 同一条纪律（没被绊倒时的绿是空的）：没给 `--init-gripper` ⇒ None。
        # 给了没写成 ⇒ 记下**为什么**（拒绝 / 未确认 / 脚本非 0），
        # 免得事后把「那一栏是 no」读成「夹爪本来就是好的」。
        "gripper_init_exercised": (None if not a.init_gripper else
                                   bool(gripper_init and not
                                        (gripper_init.get("dry_run")
                                         or gripper_init.get("refused")))),
        "gripper_init": (None if not a.init_gripper else gripper_init),
        "handoff_window": {"moved_maybe": moved, "note": note},
        "items": [i.as_dict() for (_, i, ab) in report["rows"] if not ab],
        "absent": report["absent"],
        # ⚠️ 这两个键**改过名**（2026-10-07）。旧名 `gate_p3_exit` / `gate_p4_exit`
        # 读起来像「P3 / P4 阶段的判决」，**其实不是**：它们是 `board_gate.py`
        # 那一次**子进程**的退出码，而两次的子进程覆盖范围**不一样** ——
        #   · P3 那次是 `--only controllers` ⇒ 只管**控制器**子闸门；
        #     P3 的 `示教器 PLAYING` 那一半是**本文件自己**判的（`wait_for_playing`），
        #     **不进**这个字段。
        #   · P4 那次覆盖 P4 的全套预检。
        # 实测踩点：2026-10-07 R0 阶梯第 3′ 次 JSON 是 `exit=2` 配
        # `gate_p3_exit=0`，而那一轮 P3 **明确 FAIL**（示教器 300 s 没等到 PLAYING）
        # —— 差一点被读成「P3 过了」。阶段的判决看 `items` 与末行的 `exit`。
        "controllers_gate_exit": (gate_p3 or {}).get("exit"),
        "p4_gate_exit": (gate_p4 or {}).get("exit"),
        "verdict": verdict_state,
        "exit": code,
    }
    try:
        with open(os.path.expanduser(a.json_report), "w", encoding="utf-8") as fh:
            json.dump(rec, fh, ensure_ascii=False, indent=2)
        print("  （JSON 记录 -> %s）" % a.json_report)
    except OSError as exc:
        print("  ⚠️ JSON 记录写不进去：%s" % exc)


def main(argv=None):
    # ⚠️ **人在环的提示语必须当场可见。** `print` 写进**重定向的文件**时 Python 是
    #    块缓冲（攒满 8 KB 才冲），而 P3 那句「现在去示教器按**播放**」正是
    #    **只有被人看见才有意义**的一行 —— 看不见 ⇒ 空等 `--play-timeout` ⇒ 退 2，
    #    而报告把它记成「你没按」，**反过来怪操作的人**。
    #    2026-10-06 真机第 1 次实际发生：启动器 `nohup` 到文件跑，提示语 18:13:16
    #    就发出去了，可盯着那个文件的那一头**同一时刻读到的是空文件**，
    #    300 s 到点，P3 记 FAIL。**这不是操作者按晚了，是提示语没送到。**
    #    （栈子进程另有 `PYTHONUNBUFFERED=1`，与这里无关：这里管的是启动器**自己**。）
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(line_buffering=True)
        except (AttributeError, ValueError):
            pass          # 流被别的东西接管（如测试里的 StringIO）—— 不该致命
    try:
        return run(argv)
    except KeyboardInterrupt:
        print("\n⚠️ 被人中断（Ctrl-C）—— 正在收栈…")
        return 2


if __name__ == "__main__":
    sys.exit(main())
