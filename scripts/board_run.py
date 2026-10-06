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

「控制器切换」的授权从哪来（红线 6 **逐字**点名了它）
==================================================
切换控制器是 `/controller_manager` 上的标准 ROS 2 服务 —— 不手搓 RTDE、不碰
30001–30004，所以它**不违红线 1**。问题**不在「能不能发」，在「授权从哪来」**。
三条落在这份文件里：

  ① **数据文件不许冒充授权**。任务描述里出现看着像授权的键 ⇒ `validate_task` 报红
     （`find_authorization_keys`）。授权也不来自环境变量 / 上一次的记忆 / shell
     别名 —— 红线 6 后半句「上一次的授权不覆盖这一次」禁的就是这个形状：一条会被
     复制、会被沿用的**数据**，不能冒充**一次**授权。
  ② **当次 + 被告知**。`--repair-controller` 在**起栈之前**就要一个交互终端
     （P0.写路径授权；非交互 ⇒ FAIL、退 2，**栈都不起**）；真发文之前先把**完整
     报文**摆出来（服务名 / activate 列表 / `deactivate_controllers` 恒空 /
     strictness / timeout），人确认的是**那一条**。确认动作在
     `board_gate._run_repair` 里，它在**臂已上电**的那一刻再问一次。
  ③ **可复验**。切换执行过之后，按**只读复核**的结果判「已生效 / 已执行但未生效」
     （P3.控制器修复）。发出去的请求 ≠ 结果：未生效是 **FAIL**，不是绿。

写路径的**本体**（那个服务调用、`deactivate_controllers` 恒空）在 `board_gate.py`
里，本文件原样透传、**不重实现**；本文件里连那个服务的全名都不出现。

待查（**别据此删掉写路径**）：本部署已经在跑 `ur_robot_driver/controller_stopper`。
  如果那次 STRICT 绊倒能在**配置源头**避免，控制器闸门就永远只需只读，
  `--repair-controller` 本可以退役。**这条线索没人核实过**，写在这里只是别让它丢了。

退出码（沿用全仓的 0/1/2，不加第四个）
=====================================
  0 = 闸门全过 **且**本任务判分通过（**只有真跑完一轮才可能拿到 0**）；
  1 = 闸门过了、任务也跑了，但**这一轮自己的判分没过**（「它跑了但没过」）；
  2 = 在**交棒之前**停下了（「我拒绝了」）：有 FAIL、有 SKIP（没验到）、
      或没给 `--authorize-motion`。**这一档不许读成通过。**

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

def judge_ros_env(env, ros2_on_path, install_setup_exists, repo=REPO):
    """P0：这台机器**能不能跑 ROS 2**。**纯函数**。"""
    distro = str(env.get("ROS_DISTRO") or "").strip()
    ament = str(env.get("AMENT_PREFIX_PATH") or "").strip()
    if not ros2_on_path:
        return FAIL, ("PATH 里没有 `ros2` —— 先 source /opt/ros/humble/setup.bash"
                      "（本机 WSL 上 /opt/ros/humble **没装 rclpy**，只能看不能跑）")
    if not distro:
        return FAIL, "没有 ROS_DISTRO —— 环境没 source 起来"
    if not ament:
        return FAIL, "AMENT_PREFIX_PATH 是空的 —— 环境没 source 全"
    if not install_setup_exists:
        return FAIL, ("%s 不在 —— 板子上那份检出**必须先 colcon build**"
                      "（install/ 指向 build/，不编就等于跑旧代码）"
                      % os.path.join(repo, "ros2_ws", "install", "setup.bash"))
    return PASS, "ROS_DISTRO=%s，ros2 在 PATH 里，install/setup.bash 在" % distro


def judge_domain(env, site):
    """P0：`ROS_DOMAIN_ID` 与站点一致。**纯函数**。

    漏了它 / 两边不同 = 两个互不可见的 DDS 网络，现象是「栈明明起着、话题列表空的」
    —— 实测踩过，且极易误判成「栈没起来」。
    """
    want = str(site.ros_domain_id)
    got = env.get("ROS_DOMAIN_ID")
    if got is None or str(got).strip() == "":
        return FAIL, ("环境里没有 ROS_DOMAIN_ID（站点是 %s）—— 不设就是域 0，"
                      "与站点是两个互不可见的 DDS 网络；先 `export ROS_DOMAIN_ID=%s`"
                      % (want, want))
    if str(got).strip() != want:
        return FAIL, ("环境是 %s，站点是 %s —— 两边不同的域号 = 互相看不见，"
                      "症状是「栈起着但什么话题都没有」" % (got, want))
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
    """示教器在 PLAYING 吗。**精确相等**，不用子串。

    ⚠️ 子串判定的教训写在 `board_gate.is_active` 里：`inactive` 里连着含 `act`，
    子串判定会把坏的报成好的。这里同样只认整串相等（值域就
    `STOPPED|PLAYING|PAUSED`，没有别的）。
    """
    return bool(d) and str(d.get("programState", "")).strip() == "PLAYING"


def judge_cabinet(d):
    """P1：控制柜连得上、且 `robotmode == RUNNING`。**纯函数**。

    ⚠️ 这里**不要求** PLAYING。次序是「**起栈 → 按播放 → 拉回控制器**」：URCap 的
    External Control 要先有对端，先按播放再起栈，示教器上会报「连接被拒绝」、程序
    停在 STOPPED。等播放的逻辑在 P3。
    """
    if d is None:
        return FAIL, ("连不上控制柜 29999（只读查询 robotmode / programState / "
                      "safetystatus；**不发** play/power/stop）")
    mode = str(d.get("robotmode", "")).strip()
    if mode != "RUNNING":
        return FAIL, ("robotmode=%r，不是 'RUNNING' —— 控制柜没上电 / 没使能 / "
                      "还没连上驱动？" % mode)
    return PASS, ("%s | %s | %s（此刻**不要求** PLAYING：次序是 "
                  "起栈→按播放→拉回控制器）"
                  % (mode, str(d.get("programState", "")).strip(),
                     str(d.get("safetystatus", "")).strip()))


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
                            "`python3 scripts/ag95_init_close.py --go`。未初始化时"
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


def judge_handoff_window(t_launch, t_stop, start_delay):
    """停下时，规划器**是不是已经起来了**。**纯函数** → `(bool, 说明)`。

    这一条是为了不把话说过头：退出码 2 的官方说法是「**我拒绝了、什么都没发出去**」。
    但 `start_delay` 是**可配**的（A1 用 120 s），如果这一轮按播放按晚了、或者
    `start_delay` 很小，规划器可能**已经启动并动过臂** —— 那时再收栈就不是
    「什么都没发出去」了。与其含混，不如把时间摆出来让人自己看。
    """
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
    """
    if absent:
        return ["  [SKIP] %-18s —（这一档没测）" % item.name,
                "         ↳ 声明缺席（%s）—— 既不计入通过，也不计入失败" % absent]
    lines = ["  [%s] %-18s %s" % (item.status, item.name, item.detail)]
    if item.status == SKIP:
        lines.append("         ↳ **没验到**，不计入通过。")
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


def collect_preflight(site, do_net=True, override=None):
    """把 P1 要的事实**读一遍**（全是只读查询），交给 `judge_preflight` 判。

    每个 `None` 都表示「这一项没测成」，由纯函数按 SKIP 处置 —— 这里**不**替它下结论。
    `do_net=False`（`--dry-run`）只读站点字段 + 判时效，不跑任何子进程。
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

    f["deploy_rc"] = run_helper(DEPLOY_SCRIPT, [], timeout=180.0)
    f["tree_rc"] = run_helper(TREE_SCRIPT, ["--check"], timeout=300.0)
    f["freshness"] = board_lib.freshness_gate(site, override=override)
    return f


def run_helper(script, args, timeout):
    """跑一个**只读**的守卫脚本，返回退出码；跑不起来（不在/起不动）返回 `None`。"""
    if not os.path.exists(script):
        return None
    try:
        r = subprocess.run([sys.executable, script] + list(args), cwd=REPO,
                           capture_output=True, text=True, timeout=timeout)
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
              repair=False, fast=False):
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


def wait_for_playing(ip, timeout, poll=2.0, dash=None, clock=time.time):
    """起栈**之后**轮询只读直到 PLAYING → `(到了没, 最后一次回复, 用时)`。

    只读查询（`robotmode` / `programState` / `safetystatus`）；**不发** play / power /
    stop —— 按播放是**人的动作**（红线 6）。
    """
    dash = dash or board_lib.dashboard
    t0 = clock()
    last = None
    while True:
        last = dash(ip)
        if is_playing(last):
            return True, last, clock() - t0
        if clock() - t0 >= timeout:
            return False, last, clock() - t0
        time.sleep(poll)


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
# 红线 6：控制器切换（唯一的写路径）—— 报文、授权、复核
# =====================================================================
# 发文那一行在 board_gate.py（`Probe.switch_controllers`），本文件**不重实现**。
# 本文件只做三件它该做的事：起栈**之前**拒绝没有当次授权的请求、把要发的报文
# **先完整摆出来**、切完按**只读复核**判「生效 / 未生效」。三条见文件头。

WRITE_PATH_ITEM = "P0.写路径授权"
REPAIR_ITEM = "P3.控制器修复"
SWITCH_TIMEOUT = 8.0            # 与 board_gate.Probe.switch_controllers 的默认值同


def judge_write_path(isatty):
    """`--repair-controller` 这一档能不能往下走。**纯函数** → `(status, detail)`。

    非交互 ⇒ **FAIL**，不是 SKIP：这不是「没验到」，是「这个请求本身不成立」。
    放在 P0，是为了**在起栈之前**就拒 —— 臂上电、栈起着的时候才发现没法授权，
    那已经晚了。
    """
    if isatty():
        return (PASS, "有交互终端 —— 到时候会**先打印完整报文**，再要你敲一次 yes；"
                      "发文那一刻（臂已上电）还会再确认一次")
    return (FAIL, "`--repair-controller` 要的是**当次**授权，而这里不是终端。"
                  "授权**不许**来自任务描述 / 环境变量 / 上一次的记忆（AGENTS.md "
                  "红线 6：「上一次的授权不覆盖这一次」）。换成交互终端再来；"
                  "不要写路径的话去掉这个开关即可（默认只读）。")


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

    if a.dry_run:
        add(S0, Item("P0.ROS 环境", SKIP, "没查（--dry-run 不跑任何子进程）"),
            absent="--dry-run 只看静态（文件与命令）")
        add(S0, Item("P0.域号一致", SKIP, "没查（--dry-run 不跑任何子进程）"),
            absent="--dry-run 只看静态（文件与命令）")
    else:
        add(S0, Item("P0.ROS 环境", *judge_ros_env(
            os.environ, shutil.which("ros2") is not None,
            os.path.exists(INSTALL_SETUP))))
        if site is not None:
            add(S0, Item("P0.域号一致", *judge_domain(os.environ, site)))

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

    # 红线 6：写路径要**当次**授权 ⇒ 起栈**之前**就问清楚有没有交互终端。
    # （要发的那条报文先完整摆出来 —— 人确认的是**那一条**；确认动作在 board_gate
    #   里、臂已上电的那一刻再问一次。）
    if a.repair_controller:
        if task is not None:
            print()
            for ln in switch_payload_lines(task):
                print(ln)
        add(S0, Item(WRITE_PATH_ITEM, *judge_write_path(isatty)))

    code, _, _ = decide(counted(report, S0), None, launched=False)
    if code != 0:
        return finish(code, launched=False)

    # ================================================== P1 前置（只读）
    print("\n—— %s ——" % S1)
    facts = collect_preflight(site, do_net=not a.dry_run,
                              override=a.override_freshness)
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
        for nm, det in (("P2.声明集",
                         "要跑 `ros2 launch … --show-args` 才拿得到声明集，"
                         "而 --dry-run 不跑任何子进程"),
                        ("P2.槽位数", "要读 planner_params_file，这一档留给真跑"),
                        ("P2.起栈", "--dry-run 不起栈")):
            add(S2, Item(nm, SKIP, det), absent="--dry-run 只看静态（文件与命令）")
        print()
        print("  ❌ 注意：--dry-run 退 0 只表示「计划拼得出来」，**不代表**任何闸门"
              "验过 —— 上面每一项都标着「声明缺席」。")
        return finish(0, launched=False, dry_run=True)

    env = dict(os.environ, ROS_DOMAIN_ID=str(site.ros_domain_id),
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
            add(S2, Item("P2.起栈", FAIL,
                         "--attach 要判分就得有日志，而 %s 不存在" % log_path))
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
            add(S3, Item("P3.示教器 PLAYING", PASS,
                         "%.0fs 后 PLAYING（%s）"
                         % (waited, str(d.get("programState", "")).strip())))
        else:
            got = str((d or {}).get("programState", "")).strip() or "连不上控制柜"
            add(S3, Item("P3.示教器 PLAYING", FAIL,
                         "%.0fs 内没等到 PLAYING（现在 %r）—— 次序是「起栈 → 按播放"
                         " → 拉回控制器」" % (a.play_timeout, got)))

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
            j4 = os.path.join(gate_dir, "p4.json")
            argv4 = gate_argv(j4, slots_total=task.slots_total,
                              min_candidates=task.min_candidates,
                              expect_params=task.expect_params,
                              allow_skip=task.allow_skip,
                              trajectory_action=task.trajectory_action)
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
            elif not ask_authorization(prompt_fn=prompt_fn, isatty=isatty):
                add(S5, Item("P5.交棒", SKIP,
                             "--authorize-motion 给了，但终端上没有确认 ⇒ 不交棒"))
                code = 2
            else:
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


def judge_place_slots(task, site):
    """P2：`planner_params_file` 里的 `place_slots` 长度要等于任务期望的槽位数。

    能推导的不复述：槽位数**不作为任务描述里的唯一真值**，而是从活的那份 yaml 读出来
    对一遍 —— 两边不一致时，A1 的判据（「已放置 N/N」与「槽位已用尽（N 个）」）必然
    判错，而错误会以「判分没过」的形式出现在**跑完之后**。
    """
    path = site.planner_params
    vals, why = read_planner_yaml(path)
    if vals is None:
        return Item("P2.槽位数", FAIL,
                    "读不到 planner_params_file（%s）：%s —— 真机上不给它 / 给错路径会"
                    "**静默用仿真几何**，先修这个" % (path, why))
    slots = vals.get("place_slots")
    if not isinstance(slots, list) or not slots:
        return Item("P2.槽位数", SKIP,
                    "%s 里没有 place_slots（或不是列表）—— 没验到" % path)
    if len(slots) != task.slots_total:
        return Item("P2.槽位数", FAIL,
                    "%s 的 place_slots 有 %d 个，任务描述期望 %d 个 —— 两边不一致时"
                    "判分必然判错（改完记得同步）"
                    % (path, len(slots), task.slots_total))
    return Item("P2.槽位数", PASS,
                "place_slots %d 个，与任务描述一致（读自 %s）"
                % (len(slots), path))


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
            repair_exercised=None, t_launch=None, t_stop=None,
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
    if launched:
        moved, note = judge_handoff_window(
            t_launch, t_stop, task.start_delay if task else 0.0)
        print("  " + ("🔴 " if moved else "• ") + note)
        print("  收栈不代替在示教器上按 STOP —— 收的是这套栈，不是控制柜里的程序。")
    elif dry_run and code == 0:
        print("  （--dry-run：栈没有起，也不该起；上面的绿只说明**计划拼得出来**）")
    else:
        print("  ⛔ 栈**没有起**（闸门没过就不起栈，fail-closed）。")

    if a.json_report:
        _write_report(a, report, code, task, gate_p3, gate_p4, repair_exercised,
                      t_launch, t_stop, verdict_state, log_path, launched)
    print("=" * 74)
    return code


def _write_report(a, report, code, task, gate_p3, gate_p4, repair_exercised,
                  t_launch, t_stop, verdict_state, log_path, launched):
    if path_in_repo(a.json_report):
        print("  ❌ --json-report 指向仓库内（%s）—— 本仓是公开副本，运行记录必须写在"
              "仓库外。**没有写**。" % a.json_report)
        return
    moved, note = judge_handoff_window(t_launch, t_stop,
                                       task.start_delay if task else 0.0)
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
        "handoff_window": {"moved_maybe": moved, "note": note},
        "items": [i.as_dict() for (_, i, ab) in report["rows"] if not ab],
        "absent": report["absent"],
        "gate_p3_exit": (gate_p3 or {}).get("exit"),
        "gate_p4_exit": (gate_p4 or {}).get("exit"),
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
    try:
        return run(argv)
    except KeyboardInterrupt:
        print("\n⚠️ 被人中断（Ctrl-C）—— 正在收栈…")
        return 2


if __name__ == "__main__":
    sys.exit(main())
