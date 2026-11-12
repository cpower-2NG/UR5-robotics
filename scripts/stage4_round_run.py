#!/usr/bin/env python3
"""阶段 4 正式轮**整轮编排器**：一次授权跑完「观察 → 计算 → 执行 → 判分」。

为什么要有它（用户 2026-11-11 定）
────────────────────────────────
以前的入口一次只跑**一个目标**：观察 → 解垫 → 探针 → 规划 → 执行之间 Agent
要停下来算，而运动令牌只有 600 s。用户要求：**所有真机运动都走「整轮编排」
结构**，不是「一动一定、再让 Agent 调查」。所以这里把一段真机运动拆成八个
阶段，每段都有 fail-closed 判据，段与段之间不等人：

    0 预检  1 观察计划  2 授权（人 STOP→PLAY）  3 观察执行
    4 轮内计算（R1 → 解垫 → 探针 → 规划 → R2）  5 执行
    6 判分  7 归档

**一轮授权的范围**（2026-11-11 用户拍板）= 该轮的观察（含臂移动到观察机位）
→ 执行 → 终态评分三段。轮内任何偏离计划的重规划 / 重试 / 换目标仍要停、
重新授权 —— 本编排器**不自动重试**，执行阶段失败就停。

它只做编排与闸门，**不复制任何运动几何**：
``plan_views`` / ``plan_sequence`` / ``actions_from_attempt`` / ``run_script_text``
全部来自 ``stage4_clear_run``；解垫来自 ``stage4_pad_from_burst``；探针、判分
各自复用现成脚本。

时间闸门（600 s 硬窗口，见契约 §5）
──────────────────────────────────
· **R1**（观察后、探针前）：`剩余 ≥ C_est + T_exec_est + 60 s`；
· **R2**（执行起手）：`剩余 ≥ T_exec_est + max(45 s, 15%·T_exec_est)`；
· 板端 ``run.sh`` 首行 ``--min-remaining`` 守卫，执行中过期记 ``aborted_token``
  —— 本轮 void、不计连续 streak，**不记抓取失败**。

b̂ 偏置补偿（契约 §4.7，继承阶段三）
──────────────────────────────────────
`命令落点 = 格心 − b̂`，整轮 ``fixed`` 一个值；种子优先级
``--bias-seed`` > ``<run>/bias_estimate.json``；两者都没有且未显式
``--bias-comp none`` ⇒ 停轮（阶段三教训：缺种子会「以为在补、其实没补」）。
逐块 ``b̂_round = 彩色质心 − 命令`` 在判分阶段算，``|b̂| > 15 mm`` 按**失败**
记，不当偏置。判分口径不变：仍比实际块心 vs 格心。

退出码
──────
0 全绿；2 输入/证据缺失；3 授权或时间闸门；4 场景/计划生成失败；
5 执行失败；6 判分不完整。每阶段把 ``{rc, at, seconds}`` 写进
``phases.json``，支持 ``--from/--to`` 单步重跑（只读阶段可重跑，执行阶段
不自动重跑）。

跑法（开发机，经 SSH 驱动板子）::

    python3 scripts/stage4_round_run.py \
        --batch S0 --layout 1 --run 1 \
        --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<date>-stage4-s0/s0-1-r1 \
        --urdf <robot.urdf> --board <ssh-alias> --board-repo ~/stage4-entry \
        --expect <preflight-expect.json> --pad-spec ~/.config/ur5-vision-grasp/pad-spec.json \
        --bias-seed -2.791,-0.248

``--dry-run``（Round 0 计时干跑，仍要真授权）：只到阶段 4，出计划不执行。
"""

import argparse
import hashlib
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import block_classes as bc                                      # noqa: E402
import grasp_bias as gb                                         # noqa: E402
import precision_plan as pp                                     # noqa: E402
import precision_round as pr                                    # noqa: E402
import stage4_clear_model as cm                                 # noqa: E402
import stage4_clear_run as cr                                   # noqa: E402
import stage4_layouts as lay                                    # noqa: E402
import stage4_pad_from_burst as pfb                             # noqa: E402
import stage4_placement_deviation as pdv                        # noqa: E402
import stage4_round_report as rr                                # noqa: E402
import stage4_score as sc                                       # noqa: E402
import stage4_truth_from_frame as tf                            # noqa: E402
import urkin                                                    # noqa: E402

#: 退出码（给测试与自动化用；写进 phases.json 的 ``rc``）。
EXIT_OK, EXIT_MISSING, EXIT_GATE, EXIT_PLAN, EXIT_EXEC, EXIT_SCORE = 0, 2, 3, 4, 5, 6

#: 阶段顺序（``--from/--to`` 用这些名字）。
PHASES = ("preflight", "views", "authorize", "observe",
          "compute", "execute", "score", "archive")

#: 各阶段「没预料到的异常」映射到哪个退出码（协议见模块头）。
PHASE_ERROR = {"preflight": EXIT_MISSING, "views": EXIT_PLAN,
               "authorize": EXIT_GATE, "observe": EXIT_GATE,
               "compute": EXIT_PLAN, "execute": EXIT_EXEC,
               "score": EXIT_SCORE, "archive": EXIT_MISSING}

#: ``T_exec_est`` 的初值（Round 0 实测后由 ``--timing-prior`` 覆盖）。
#:
#: 2026-11-12 实测（S0:1 满五块一轮，8 份计划）：航点标称总时长 140 s，run.sh
#: 实际 ~390 s ⇒ **每份计划 ~30 s 固定开销**（每航点 1.5 s 停稳 ×8 = 12 s、
#: 夹爪开合 ~8 s、独立进程起 rclpy/等服务发现 5–20 s）。旧值 5 s 把执行估成
#: 206 s、实际 440 s —— R1/R2 用这种估时等于没有闸门（2026-11-12 实测）。
PLAN_OVERHEAD_S = 30.0       # 每份计划：停稳 / 夹爪开合 / 进程启动
PER_PLAN_S = 1.0             # 执行器逐条建链 / 日志开销
BURST_OVERHEAD_S = 6.0       # 每组腕部 burst 20 帧
R1_MARGIN_S = 60.0           # R1 的固定余量
R2_MIN_MARGIN_S = 45.0       # R2 的最小余量
R2_MARGIN_FRAC = 0.15        # R2 的按比例余量

#: 跑前只拍 pick / pad 两个机位（S0 无清障、free 只在跑后拍）。
PRE_VIEWS = tuple(v for v in cr.E9_VIEWS if v["name"] in ("pick", "pad"))

#: 真值保留区的兜底值（只在 burst 目录名认不出机位时用）。正式轮每个机位
#: 各用**自己 zone 外扩 25 mm** 的区，见 ``truth_zone_for``。
TRUTH_ZONE = "0.20,0.80,-0.30,0.65"

#: 机位 zone 的外扩量（米）。区外检测由真值工具按「幻影」丢掉（它自己的语义），
#: 所以这份区必须只圈「本机位允许出现块的地方」。
TRUTH_ZONE_MARGIN_M = 0.025


def truth_zone_for(burst):
    """按 burst 目录名（``wrist-pick`` / ``wrist-pad`` / ``wrist-free``）取保留区。

    为什么不再用一张大区：2026-11-12 S0:1 r1 实测，自由区机位把**示教器**
    当成一块 42 mm 高的青色块（785, 509 mm）——它在旧的大区（x ≤ 0.80）里，
    块角只越出模型桌沿 0.3 mm，于是「全桌终态」判 0/5、整轮假红。真值工具
    自己的语义是「区外 = 幻影，默认丢掉」，喂它整张台就等于把台外的东西也
    当成块。丢块不会因此瞒过去：块数一致 / 被搬块逐块配对仍会报缺。
    """
    name = os.path.basename(os.path.normpath(burst))
    if name.startswith("wrist-"):
        name = name[len("wrist-"):]
    for v in cr.E9_VIEWS:
        if v["name"] == name:
            x0, x1, y0, y1 = v["zone"]
            m = TRUTH_ZONE_MARGIN_M
            return "%.3f,%.3f,%.3f,%.3f" % (x0 - m, x1 + m, y0 - m, y1 + m)
    return TRUTH_ZONE


def log(*a):
    print("  " + " ".join(str(x) for x in a), flush=True)


def read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_json(path, doc, indent=1):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=indent)
    return path


class RoundError(Exception):
    """一个阶段判红：带退出码与「为什么」，由 ``Round.run`` 落进 ``phases.json``。"""

    def __init__(self, code, phase, why, void=False):
        super().__init__(why)
        self.code = int(code)
        self.phase = phase
        self.why = str(why)
        self.void = bool(void)


# =====================================================================
# 纯函数（可单测，不碰板子 / 不碰 ROS）
# =====================================================================
def exec_estimate(plans, plan_overhead=PLAN_OVERHEAD_S,
                  per_plan=PER_PLAN_S, burst_overhead=BURST_OVERHEAD_S,
                  read=read_json):
    """``T_exec_est`` = Σ 航点 duration + 每份计划开销 + 每组 burst 开销。

    口径见契约 §5。``duration`` 是执行器真正发给 JTC 的秒数（``precision_plan``
    逐航点算好），所以这是**下界加固定开销**，不是拍脑袋；``--timing-prior``
    用 Round 0 的实测值替换。

    **遇到读不了的计划直接抛**：估时是安全闸门，不是报告 —— 拿 0 去比令牌，
    等于把闸门拆了（fail-closed）。
    """
    total = 0.0
    n_burst = 0
    for p in plans or []:
        doc = read(p["path"])
        wps = doc.get("waypoints") or []
        total += sum(float(w.get("duration") or 0.0) for w in wps)
        total += float(plan_overhead) + float(per_plan)
        if p.get("kind") == "view" and p.get("view"):
            n_burst += 1
    return round(total + n_burst * float(burst_overhead), 1)


def r1_need(compute_est_s, exec_est_s, margin=R1_MARGIN_S):
    """R1 要求的剩余秒数。"""
    return float(compute_est_s) + float(exec_est_s) + float(margin)


def r1_gate(remaining, compute_est_s, exec_est_s, margin=R1_MARGIN_S):
    """观察后、探针前：令牌够不够「算完 + 跑完 + 余量」。返回 ``(ok, 说明)``。"""
    need = r1_need(compute_est_s, exec_est_s, margin)
    if remaining is None:
        return False, "R1 不过：没有有效令牌（先做 STOP→PLAY 两步握手）"
    ok = float(remaining) >= need
    return ok, ("R1 %s：令牌还剩 %.0f s，需 ≥ %.0f s"
                "（计算 %.0f + 执行 %.0f + 余量 %.0f）"
                % ("通过" if ok else "不过", float(remaining), need,
                   float(compute_est_s), float(exec_est_s), float(margin)))


def r2_margin(exec_est_s):
    """R2 的余量：``max(45 s, 15%·T_exec_est)``。"""
    return max(R2_MIN_MARGIN_S, R2_MARGIN_FRAC * float(exec_est_s))


def r2_gate(remaining, exec_est_s, margin=None):
    """执行起手：令牌够不够跑完执行段（含机位 burst）。返回 ``(ok, 说明)``。"""
    m = r2_margin(exec_est_s) if margin is None else float(margin)
    need = float(exec_est_s) + m
    if remaining is None:
        return False, "R2 不过：没有有效令牌"
    ok = float(remaining) >= need
    return ok, ("R2 %s：令牌还剩 %.0f s，需 ≥ %.0f s（执行 %.0f + 余量 %.0f）"
                % ("通过" if ok else "不过", float(remaining), need,
                   float(exec_est_s), m))


def bias_setup(mode, seed_spec, estimate_path):
    """``--bias-comp`` / ``--bias-seed`` / ``<run>/bias_estimate.json`` → 本轮 b̂。

    缺种子且未显式 ``none`` ⇒ 抛 ``RoundError(2)`` 停轮（契约 §4.7）。
    返回 ``{mode, seed_mm, applied_mm, source, note}``。
    """
    mode = (mode or "fixed").strip().lower()
    if mode not in ("fixed", "none"):
        raise ValueError("--bias-comp 只收 fixed / none（阶段四不用 adaptive："
                         "它要求放一块测一块，与整轮一次成脚本冲突），收到 %r"
                         % (mode,))
    if mode == "none":
        return {"mode": "none", "seed_mm": None, "applied_mm": [0.0, 0.0],
                "source": "none", "note": gb.bias_note("none")}
    seed = gb.parse_bias_seed(seed_spec)
    source = "--bias-seed" if seed is not None else None
    if seed is None and estimate_path:
        seed = gb.load_bias_estimate(estimate_path)
        if seed is not None:
            source = estimate_path
    if seed is None:
        raise RoundError(
            EXIT_MISSING, "compute",
            "没有 b̂ 种子：--bias-seed 没给，%s 也没有可用的 v%d 估计 —— "
            "停轮。要么显式给种子（阶段三先验 -2.791,-0.248），要么显式 "
            "--bias-comp none（并在报告里标「未补偿」）"
            % (estimate_path or "<run>/bias_estimate.json",
               gb.BIAS_ESTIMATE_VERSION))
    applied = gb.applied_bias([], mode, seed=seed)
    return {"mode": mode, "seed_mm": [float(seed[0]), float(seed[1])],
            "applied_mm": [float(applied[0]), float(applied[1])],
            "source": source, "note": gb.bias_note(mode)}


def validate_pad_doc(doc, min_frames=20, max_spread_mm=2.0, max_spread_deg=0.5):
    """解垫文档是否够格当本轮计划与判分的**同一份**垫位姿。返回问题列表。"""
    probs = []
    if not isinstance(doc, dict):
        return ["解垫输出不是 JSON 对象"]
    if doc.get("kind") != "stage2-grid":
        probs.append("kind=%r ≠ stage2-grid" % (doc.get("kind"),))
    try:
        n_ok = int(doc.get("n_frames_ok") or 0)
        n_tot = int(doc.get("n_frames_total") or 0)
    except (TypeError, ValueError):
        return probs + ["n_frames_ok/n_frames_total 不是整数"]
    if n_ok < int(min_frames):
        probs.append("只有 %d 帧解出（要求 ≥%d）" % (n_ok, int(min_frames)))
    if n_tot < int(min_frames):
        probs.append("burst 只有 %d 帧（要求 ≥%d）" % (n_tot, int(min_frames)))
    xy = ((doc.get("spread_xy_mm") or {}).get("max")
          if isinstance(doc.get("spread_xy_mm"), dict) else None)
    yaw = ((doc.get("spread_yaw_deg") or {}).get("max")
           if isinstance(doc.get("spread_yaw_deg"), dict) else None)
    try:
        if xy is None or float(xy) > float(max_spread_mm):
            probs.append("xy 极差 %r 超过 %.2f mm" % (xy, float(max_spread_mm)))
        if yaw is None or float(yaw) > float(max_spread_deg):
            probs.append("yaw 极差 %r 超过 %.2f°" % (yaw, float(max_spread_deg)))
    except (TypeError, ValueError):
        probs.append("spread 字段不是数：xy=%r yaw=%r" % (xy, yaw))
    for k in ("grid_center_base_mm", "grid_yaw_base_deg"):
        if k not in doc:
            probs.append("缺字段 %s" % k)
    return probs


def serial_fp(serial):
    """模组序列号的短指纹 —— 日志/问题文本只许打这个，不许打真值（红线 3）。"""
    return hashlib.sha256(str(serial).encode("utf-8")).hexdigest()[:12]


def evaluate_preflight(got, expect):
    """板端预检事实 vs 期望值 → 问题列表（空 = 全绿）。

    **期望值必需**：不知道「这块板是哪块、这棵树是哪棵」的预检只能证明
    「板子开着了」，不能证明是**我们这台**。缺 ``--expect`` 直接判红
    （fail-closed，与 §3.2 阶段 0 一致）。
    """
    probs = []
    got = got or {}
    if not expect:
        return ["没有预检期望值（--expect）：不知道板上该是哪块板 / 哪棵树"]

    serial = got.get("serial")
    if not serial:
        probs.append("读不到模组序列号（/proc/device-tree/serial-number）")
    elif expect.get("serial") and serial != expect["serial"]:
        # 只打指纹：序列号是硬件真值，不许贴进对话/日志（AGENTS.md 红线 3）
        probs.append("模组序列号不匹配（板上指纹 %s，期望指纹 %s）"
                     % (serial_fp(serial), serial_fp(expect["serial"])))

    tree = got.get("tree") or {}
    if not tree.get("files"):
        # 空树 = 路径不存在/为空，哈希会退化成空串的 sha256（e3b0c442…）。
        # 这种情况绝不能靠「期望值恰好没填 tree_files」蒙过去。
        probs.append("板上代码树扫到 0 个文件（board_repo 路径对不上？）")
    if expect.get("tree_sha256"):
        if tree.get("sha256") != expect["tree_sha256"]:
            probs.append("板上代码树哈希不一致（板上 %s，期望 %s）—— "
                         "先同步/重编，再跑"
                         % (str(tree.get("sha256"))[:12],
                            str(expect["tree_sha256"])[:12]))
    if expect.get("tree_files") and tree.get("files") != expect["tree_files"]:
        probs.append("板上代码树文件数 %r ≠ 期望 %r"
                     % (tree.get("files"), expect["tree_files"]))

    g = got.get("gripper") or {}
    if not g.get("ok"):
        probs.append("读不到夹爪状态：%s" % (g.get("why"),))
    elif int(g.get("init_state") or 0) != 1:
        probs.append("夹爪 0x0200=%r ≠ 1（未初始化 / 进行中）—— 停，"
                     "不得自动初始化（红线 2 要当次授权）"
                     % (g.get("init_state"),))

    if expect.get("wrist_usb3", True):
        speeds = got.get("realsense") or []
        if not speeds:
            probs.append("没有找到 RealSense USB 设备（腕部相机没插？）")
        elif not any(str(s.get("speed")) == "5000" for s in speeds):
            probs.append("腕部相机不在 USB3（RealSense 设备速度 %s）—— "
                         "USB2 上会「能枚举 / 能起节点但 0 帧」"
                         % [s.get("speed") for s in speeds])

    for name, want in (expect.get("stack") or {}).items():
        if want and not (got.get("stack") or {}).get(name):
            probs.append("栈里没有 %s 进程" % name)

    # dashboard 连接必须是活的：死了不会报「错」，只会让第一次闸门在**令牌窗口
    # 内**等 20 s × 重试（2026-11-12 实测 147 s）。预检阶段（授权前）先修好。
    d = got.get("dashboard")
    if not isinstance(d, dict):
        probs.append("预检没查 dashboard_client 连接（板上预检脚本版本旧）—— "
                     "不查就可能在令牌窗口里白等 ~150 s")
    elif not d.get("ok"):
        probs.append("dashboard_client 没响应（state=%r）：先重连再跑"
                     % (d.get("state"),))
    return probs


def build_facts(round_id, batch, layout, spec_blocks, bias, human=None):
    """本轮 ``facts.json``：机器能证明的 + 人给的安全/干预事实。

    ``safety`` / ``interventions`` / ``human_review`` **不从执行日志猜**：
    契约要求它们是人的计数，猜出来的 0 会把「没测」伪装成「没发生」。
    缺了就缺 —— ``stage4_round_report`` 会判 ``complete=false``。
    """
    facts = {
        "round_id": round_id,
        "scene": {"batch": batch, "layout": int(layout), "seed": None},
        "truth_source": "layout-manifest+postrun-wrist",
        "specs": {str(b["id"]): {"color": b["color"],
                                 "size_m": float(b["size"])}
                  for b in spec_blocks},
        "auto_top1": None,      # S0 是结构化指定，不是自动 top-1 ⇒ N/A
        "bias": dict(bias),
    }
    for k in ("safety", "interventions", "human_review", "voids",
              "failure_class"):
        if isinstance(human, dict) and k in human:
            facts[k] = human[k]
    return facts


def _sorted_ids(ids):
    """与 ``cm.assign_cells`` 同一排序：数字 id 按数值，其余按字符串。"""
    def key(v):
        s = str(v)
        return (0, int(s), s) if s.isdigit() else (1, 0, s)
    return sorted([str(v) for v in ids], key=key)


def _q_from_joints(doc, chain):
    """``joint_states.json`` → 按 ``chain.move_joints`` 排序的关节角。"""
    names = doc.get("name") or []
    pos = doc.get("position") or []
    try:
        return [float(pos[names.index(n)]) for n in chain.move_joints]
    except ValueError as e:
        raise RoundError(EXIT_GATE, "observe",
                         "关节状态缺关节：%s（拿到的 %s）" % (e, names))


# =====================================================================
# 板端预检脚本（只读：序列号 / 全树哈希 / 夹爪 0x0200 / USB 速度 / 栈进程）
# =====================================================================
PREFLIGHT_PY = r'''
import glob
import hashlib
import json
import os
import subprocess
import sys

repo = sys.argv[1] if len(sys.argv) > 1 else "~/stage4-entry"
# 参数是 shell 引号过的（shlex.quote），``~`` 不会被远端 shell 展开 ⇒ 自己展开。
# 2026-11-11 实测：传 ``~/<repo>`` 而这里不展开 ⇒ os.walk 走空
# ⇒ 树哈希 = e3b0c442…（空串的 sha256）、文件数 0。
repo = os.path.expanduser(repo)
out = {}

try:
    with open("/proc/device-tree/serial-number", "rb") as f:
        out["serial"] = f.read().replace(b"\x00", b"").decode().strip()
except OSError as exc:
    out["serial"] = None
    out["serial_error"] = str(exc)

# 树哈希**只覆盖源码**：`__pycache__` / `*.pyc` / 构建产物 / 备份目录每次跑都会
# 变（2026-11-11 实测：跑一次 `--help` 就让文件数 1129→1130、哈希全变 ⇒ 预检
# 假红，把「部署变了」和「有人 import 过一次」混为一谈）。`.git` 也不进。
_SKIP_DIRNAMES = {".git", "__pycache__"}
_SKIP_PREFIX = ("ros2_ws/build", "ros2_ws/install", "ros2_ws/log")
h = hashlib.sha256()
n = 0
for root, dirs, files in os.walk(repo):
    rel_root = os.path.relpath(root, repo).replace(os.sep, "/")
    keep = []
    for d in sorted(dirs):
        rel = d if rel_root == "." else rel_root + "/" + d
        if d in _SKIP_DIRNAMES or d.startswith("scripts.bak"):
            continue
        if any(rel == p or rel.startswith(p + "/") for p in _SKIP_PREFIX):
            continue
        keep.append(d)
    dirs[:] = keep
    for name in sorted(files):
        if name.endswith((".pyc", ".pyo")) or ".bak-" in name:
            continue
        path = os.path.join(root, name)
        rel = os.path.relpath(path, repo)
        h.update(rel.encode("utf-8") + b"\x00")
        try:
            with open(path, "rb") as fh:
                while True:
                    chunk = fh.read(1 << 20)
                    if not chunk:
                        break
                    h.update(chunk)
        except OSError:
            h.update(b"<unreadable>")
        n += 1
out["tree"] = {"sha256": h.hexdigest(), "files": n}

try:
    sys.path.insert(0, os.path.join(repo, "scripts"))
    import ag95_init_close as ag
    g = ag.Gripper.open_port()
    if g is None:
        out["gripper"] = {"ok": False, "why": "串口打不开"}
    else:
        try:
            out["gripper"] = {"ok": True,
                              "init_state": int(g.read(ag.REG_INIT_STATUS))}
        finally:
            g.close_port()
except Exception as exc:                                  # noqa: BLE001
    out["gripper"] = {"ok": False, "why": str(exc)[:160]}

speeds = []
for d in glob.glob("/sys/bus/usb/devices/*/idVendor"):
    try:
        with open(d) as fh:
            if fh.read().strip() != "8086":
                continue
        sp = os.path.join(os.path.dirname(d), "speed")
        with open(sp) as fh:
            speeds.append({"dev": os.path.basename(os.path.dirname(d)),
                           "speed": fh.read().strip()})
    except OSError:
        continue
out["realsense"] = speeds

ps = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                    capture_output=True, text=True).stdout
lines = [l for l in ps.splitlines() if "bash -c" not in l]
out["stack"] = {p: any(p in l for l in lines)
                for p in ("ur_robot_driver", "move_group", "controller_manager")}


# dashboard_client 的 TCP 会静默死掉（控制柜断过电 / 久置）：症状是第一次
# 闸门查询回 "Did not receive answer from dashboard server in time (20 s)"，
# ``precision_exec`` 原地重试到 ~150 s。这段等待如果发生在令牌窗口里，就是
# 白烧整轮（2026-11-12 实测 147.7 s，观察段 193.6 s 里的四分之三）。
# 所以把它挪到**授权前**的预检里：只读查一次 programState，读不到就
# ``/dashboard_client/connect`` 重连再读 —— 只读服务，不动臂、不碰 RTDE。
def _ros_shell(cmd, timeout):
    return subprocess.run(
        ["bash", "-c",
         "export ROS_DOMAIN_ID=99; . /opt/ros/humble/setup.bash; "
         ". $HOME/yolo_ros_original/ros2_ws/install/setup.bash; " + cmd],
        capture_output=True, text=True, timeout=timeout)


def _program_state():
    r = _ros_shell(
        'timeout 25 ros2 service call /dashboard_client/raw_request '
        'ur_dashboard_msgs/srv/RawRequest "{query: programState}"', 40)
    return ((r.stdout or "") + (r.stderr or "")).strip()


def _state_of(text):
    if "PLAYING" in text:
        return "PLAYING"
    if "STOPPED" in text:
        return "STOPPED"
    return "unknown"


try:
    raw = _program_state()
    reconnected = False
    if "Did not receive answer" in raw or _state_of(raw) == "unknown":
        _ros_shell('timeout 25 ros2 service call /dashboard_client/connect '
                   'std_srvs/srv/Trigger "{}"', 40)
        reconnected = True
        raw = _program_state()
    out["dashboard"] = {"ok": _state_of(raw) != "unknown",
                        "state": _state_of(raw),
                        "reconnected": reconnected}
except Exception as exc:                                  # noqa: BLE001
    out["dashboard"] = {"ok": False, "state": "unknown",
                        "why": str(exc)[:160]}

print(json.dumps(out))
'''


def default_board_dir(out_dir):
    """板上本轮镜像目录缺省值：``~/precision/e9/<run>/<round>``。

    **只有一个来源**：``Round`` 与 ``main()`` 都调它。2026-11-11 实测过不一致的
    后果 —— ``main()`` 曾把未解析的 ``None`` 直接交给 ``RoundBoard``，不给
    ``--board-dir`` 就崩在 ``None.rstrip``。
    """
    out = os.path.abspath(out_dir)
    return "~/precision/e9/%s/%s" % (os.path.basename(os.path.dirname(out)),
                                     os.path.basename(out))


class RoundBoard:
    """板子适配器：SSH / 推文件（md5 核对）/ 取关节 / 跑脚本 / 令牌剩余。

    开发机不能当 ROS 节点（跨进程 DDS 不可用），所以一切板上动作都在这里。
    **不打印令牌值**：命令走 ``quiet=True``，只报剩余秒数。
    """

    def __init__(self, alias, board_dir, exp_dir):
        self.alias = alias
        if not board_dir:
            raise RoundError(EXIT_MISSING, "board", "板上镜像目录为空")
        self.board_dir = board_dir.rstrip("/")
        self.exp_dir = exp_dir
        self._b = pr.Board(alias, exp_dir)

    # ---- 基础管道
    def ssh(self, cmd, quiet=True, check=False):
        return self._b.ssh(cmd, quiet=quiet, check=check)

    def _scp(self, src, dst):
        """本地 → 板上（**只管这一个方向**）。"""
        r = subprocess.run(["scp", "-o", "LogLevel=ERROR", src,
                            "%s:%s" % (self.alias, dst)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RoundError(EXIT_MISSING, "board",
                             "scp %s → %s 失败（rc=%d）：%s"
                             % (src, dst, r.returncode,
                                (r.stderr or "").strip()[:200]))
        return r

    def _scp_from(self, src_remote, dst_local):
        """板上 → 本地。

        **不能复用 ``_scp``**：它无条件给**目标**加 ``<alias>:`` 前缀，用反方向
        就是把本地路径当板上路径 ⇒ ``scp -q host:remote host:/local/x``。
        2026-11-11 Round 0 实测：探针报告明明在板上（1493 B），编排器却报
        「拉不回来」，而且 ``-q`` 把原因也吞了（错误文本是空的）。
        """
        r = subprocess.run(["scp", "-o", "LogLevel=ERROR",
                            "%s:%s" % (self.alias, src_remote), dst_local],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RoundError(EXIT_MISSING, "board",
                             "scp %s:%s → %s 失败（rc=%d）：%s"
                             % (self.alias, src_remote, dst_local,
                                r.returncode, (r.stderr or "").strip()[:200]))
        return r

    def push(self, local, remote, label=None):
        """推一个文件，并逐个 md5 核对（推没推上去要能证明）。"""
        self.ssh("mkdir -p %s" % os.path.dirname(remote))
        self._scp(local, remote)
        want = hashlib.md5(open(local, "rb").read()).hexdigest()
        r = self.ssh("md5sum %s" % remote)
        got = (r.stdout or "").split()[0] if (r.stdout or "").strip() else "?"
        if got != want:
            raise RoundError(EXIT_MISSING, "board",
                             "推送后 md5 不一致：%s（本地 %s / 板上 %s）"
                             % (label or os.path.basename(remote),
                                want[:12], got[:12]))
        log("  推送 %-40s md5 %s 一致" % (label or os.path.basename(remote),
                                         want[:12]))

    def push_many(self, pairs):
        """``[(local, remote), ...]``；全部推完逐个核对。"""
        for local, remote in pairs:
            self.push(local, remote)

    def mkdir(self, remote):
        """板上建目录（幂等）。

        ``push`` 只建**被推文件所在**的目录；像 ``pre/exec/`` 这种只写 jsonl、
        从不推文件的目录不会自己出现。2026-11-11 Round 0 实测：``precision_exec``
        发轨迹前写执行日志 ⇒ ``FileNotFoundError`` ⇒ 观察段 rc=9 停轮。
        """
        if not remote:
            raise RoundError(EXIT_MISSING, "board", "板上目录为空")
        self.ssh("mkdir -p %s" % remote)

    def clear(self, remote):
        """删掉板上一条产物路径（幂等）。

        ``precision_exec`` / ``precision_capture`` 只追加/覆写、从不清理：同一轮
        重跑时旧执行日志会留在文件里（2026-11-12 实测 ``pre/exec/01-view-pick
        .json.jsonl`` 同时有 20:40 与 21:28 两次运行的记录）⇒ 判分会把上一次
        尝试的记录当成本轮证据。每次推/跑之前先清。
        """
        self.ssh("rm -rf %s" % remote)

    def pull_file(self, remote, local):
        os.makedirs(os.path.dirname(os.path.abspath(local)) or ".", exist_ok=True)
        return self._scp_from(remote, local)

    def pull_dir(self, remote, local):
        """把板上目录的**内容**拉到 local。

        用 ``rsync`` 而不是 ``scp -r host:path/. local``：新版 OpenSSH 的 scp 走
        SFTP，末尾的 ``/.`` 直接报 ``error: unexpected filename: .`` 且**退出码
        仍是 0**、一个文件都不拷。2026-11-11 Round 0 实测：观察段跑完 20 帧
        好端端躺在板上，拉回来却是 0 帧，白白烧掉一次真机授权。
        拉失败一律抛错，不许「静默 0 帧」。
        """
        os.makedirs(local, exist_ok=True)
        r = subprocess.run(["rsync", "-a", "%s:%s/" % (self.alias, remote),
                            local.rstrip("/") + "/"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RoundError(EXIT_MISSING, "board",
                             "rsync 拉目录失败（%s → %s，rc=%d）：%s"
                             % (remote, local, r.returncode,
                                (r.stderr or "").strip()[:200]))
        return r

    # ---- 板上动作
    def joints(self, dest_local):
        try:
            return self._b.joints(dest_local)
        except SystemExit as e:
            raise RoundError(EXIT_GATE, "observe", "取板上关节失败：%s" % e)

    def preflight(self, repo):
        cmd = "python3 - %s <<'PY'\n%s\nPY" % (shlex.quote(repo), PREFLIGHT_PY)
        r = self.ssh(cmd)
        if r.returncode != 0:
            raise RoundError(EXIT_MISSING, "preflight",
                             "板上预检脚本失败（rc=%d）：%s"
                             % (r.returncode, (r.stderr or "").strip()[:300]))
        try:
            return json.loads((r.stdout or "").strip().splitlines()[-1])
        except (ValueError, IndexError) as e:
            raise RoundError(EXIT_MISSING, "preflight",
                             "板上预检输出不是 JSON：%s（%s）"
                             % (str(e), (r.stdout or "").strip()[:200]))

    def token_state(self):
        """读板上握手令牌 → ``(token, 剩余秒数, 原因)``；用**板上时钟**算。"""
        r = self.ssh('cat ' + pr.AUTH_TOKEN_FILE + ' 2>/dev/null; '
                     'echo ---; date +%s')
        doc_s, _, now_s = (r.stdout or "").partition("---")
        try:
            doc = json.loads(doc_s.strip())
            now = float(now_s.strip())
        except (ValueError, TypeError):
            return None, None, "读不到板上的令牌文件 / 板上时钟"
        tok = (doc or {}).get("token")
        if not tok:
            return None, None, "板上没有运动授权令牌（先做 STOP→PLAY 两步握手）"
        if not doc.get("stop_at") or not doc.get("play_at"):
            return None, None, "令牌缺 STOP/PLAY 两步记录 —— 重做握手"
        rem = float(doc.get("expires") or 0) - now
        if rem <= 0:
            return None, None, ("令牌已过期 %.0f 秒（按板上时钟）"
                                % (-rem))
        return tok, rem, ""

    def wait_token(self, timeout_s, poll_s, now=time.time, sleep=time.sleep,
                   out=log, fresh_min=None):
        """在板上轮询到「**这一次**两步握手写完令牌」为止；超时 ⇒ ``RoundError(3)``。

        ``fresh_min``：令牌剩余秒数下限。**不给下限就会捡到上一轮/上一次尝试
        留下的令牌**——它还没过期，``wait_token`` 立刻「授权成立」，等于沿用
        旧授权。2026-11-11 Round 0 重跑时就发生过一次（令牌还剩 479 s，而这一
        次人还没按 STOP/PLAY）。红线 6 要求逐次授权 ⇒ 缺省要求令牌几乎是新
        签发的（600 s TTL 留 60 s 余量）。
        """
        deadline = float(now()) + float(timeout_s)
        last = None
        while float(now()) < deadline:
            tok, rem, why = self.token_state()
            if tok and (fresh_min is None or float(rem) >= float(fresh_min)):
                log("授权成立：按板上时钟令牌还剩 %.0f 秒" % rem)
                return tok, rem
            if tok:
                why = ("令牌只剩 %.0f 秒（< %.0f）—— 不是这一次签发的，继续等"
                       "新的 STOP→PLAY" % (float(rem), float(fresh_min)))
            if why != last:
                log("等待授权：%s" % why)
                last = why
            sleep(float(poll_s))
        raise RoundError(EXIT_GATE, "authorize",
                         "等 %.0f 秒仍没有有效令牌（最后一次：%s）—— "
                         "按了 PLAY 也不算，要两步都看到（fail-closed）"
                         % (float(timeout_s), last or "没有读到令牌文件"))

    def run_script(self, remote_script, token):
        """在板上跑 ``bash <script> <token>``；返回 ``(rc, 输出)``。

        **不能**把脚本路径交给 ``shlex.quote``：它会把 ``~/...`` 包进单引号，
        而 bash 不展开单引号里的波浪号（板端路径统一以 ``~`` 开头）。这里只对
        令牌用 ``shlex.quote``；脚本路径由我们自己拼、不含空格。
        """
        if " " in remote_script or "'" in remote_script:
            raise RoundError(EXIT_MISSING, "board",
                             "板上脚本路径含空格/引号：%r" % remote_script)
        cmd = "bash %s %s" % (remote_script, shlex.quote(token))
        r = self.ssh(cmd)
        return r.returncode, ((r.stdout or "") + (r.stderr or ""))

    def probe(self, repo, cells, group, truth_remote, grid_remote,
              report_remote):
        """板上跑一轮只读探针（MoveIt 服务，不发轨迹）。

        ``~/precision/run.sh`` **本身就是解释器包装器**（``exec python3 "$@"``），
        所以 ``run.sh`` 后面直接接脚本路径；再写一个 ``python3`` 会被当成
        「要打开的脚本名」——2026-11-11 Round 0 实测报
        ``python3: can't open file '<repo>/python3'``（rc=2），干跑停在探针这一步、
        白烧一次授权。``cd <repo>`` 保留：探针按仓库内的相对路径找脚本。

        **路径不能过 ``shlex.quote``**：板端路径一律以 ``~`` 开头，单引号会把
        波浪号焊死、不展开。改为直接要求不含空白/引号（本仓生成的都是这种）。
        """
        parts = [repo, truth_remote, str(cells), group, grid_remote,
                 report_remote]
        for p in parts:
            if not p or " " in p or "'" in p or '"' in p:
                raise RoundError(EXIT_MISSING, "board",
                                 "探针参数含空白/引号：%r" % p)
        cmd = ("cd %s && %s/run.sh scripts/stage4_clear_probe.py "
               "--truth %s --grid %s --cells %s --group %s --report %s"
               % (repo, pr.BOARD_DIR, truth_remote, grid_remote, cells,
                  group, report_remote))
        return self.ssh(cmd)


# =====================================================================
# 默认依赖（测试里整包替换成假的，不碰 SSH / ROS / 相机）
# =====================================================================
def default_solve_pad(burst, out, min_frames=20, max_spread_mm=2.0,
                      max_spread_deg=0.5):
    return pfb.main([burst, "--out", out,
                     "--min-frames", str(int(min_frames)),
                     "--max-spread-mm", str(float(max_spread_mm)),
                     "--max-spread-deg", str(float(max_spread_deg))])


def default_truth_from_view(burst, out, allow_empty=False):
    """量一个机位的真值。

    ``allow_empty`` 只给**跑后三机位**用：干净的机位（该区域确实没有块）是
    合法结果，不能退化成「没有证据」被剔出溯源。跑前 pick 机位不传它 ——
    那里本来就必须有块，零块就是分割/相机坏了，该红。
    """
    argv = ["--frame", burst, "--consensus", "--min-frames", "5",
            "--camera", "wrist", "--zone", truth_zone_for(burst),
            "--out", out]
    if allow_empty:
        argv.append("--allow-empty")
    return tf.main(argv)


def default_measure_colour(burst, xy_mm, top_z, expect_class, hue_deg,
                           search_mm=25.0):
    return cr.measure_target_colour(
        burst, (float(xy_mm[0]) / 1000.0, float(xy_mm[1]) / 1000.0),
        float(top_z), search_mm=float(search_mm), camera="wrist",
        expect_hue_deg=hue_deg, expect_class=expect_class)


def default_round_report(argv):
    return rr.main(argv)


def default_placement_deviation(argv):
    return pdv.main(argv)


def default_score(argv):
    return sc.main(argv)


def default_deps():
    return {
        "solve_pad": default_solve_pad,
        "truth_from_view": default_truth_from_view,
        "measure_colour": default_measure_colour,
        "round_report": default_round_report,
        "placement_deviation": default_placement_deviation,
        "score": default_score,
        "now": time.time,
        "sleep": time.sleep,
    }


# =====================================================================
# 整轮
# =====================================================================
class Round:
    """一轮 = 一个布局的全部指定块，一次授权，八个阶段。"""

    def __init__(self, args, board, deps=None):
        self.args = args
        self.board = board
        self.deps = default_deps()
        if deps:
            self.deps.update(deps)
        self.out = os.path.abspath(args.out_dir)
        self.board_dir = (args.board_dir or default_board_dir(self.out))
        self.round_id = (args.round_id or
                         "%s-%d-r%d" % (args.batch, int(args.layout),
                                        int(args.run)))
        self.phases = []
        self.timing = {}
        self._t_phase = {}

    # ---- 目录
    def p(self, *parts):
        return os.path.join(self.out, *parts)

    def b(self, *parts):
        return "/".join([self.board_dir] + list(parts))

    def _t0(self):
        self._t_phase = {"start": float(self.deps["now"]())}

    def _phase_seconds(self):
        return round(float(self.deps["now"]()) - self._t_phase.get("start", 0.0), 1)

    # ---- 记录与主循环
    def rec(self, name, rc, why="", extra=None):
        row = {"phase": name, "index": PHASES.index(name), "rc": int(rc),
               "seconds": self._phase_seconds(), "why": why}
        if extra:
            row.update(extra)
        self.phases.append(row)
        write_json(self.p("phases.json"), self.phases)
        return row

    def _merge_prior(self, start, end):
        """``--from/--to`` 单步重跑时**不许丢前几段的证据**。

        2026-11-11 Round 0 实测：只重跑 compute 会把 ``phases.json`` /
        ``timing.json`` 整个覆盖 —— 观察段、授权的记录就此消失，而它们正是
        归档与「令牌还剩多少」要看的证据。规则：本次要跑的段落一律重测重写，
        范围外的旧行原样保留。
        """
        run_names = set(PHASES[start:end + 1])
        ppath = self.p("phases.json")
        if os.path.exists(ppath):
            try:
                prior = read_json(ppath)
            except Exception:                                   # noqa: BLE001
                prior = []
            self.phases = [r for r in prior
                           if isinstance(r, dict)
                           and r.get("phase") not in run_names]
        tpath = self.p("timing.json")
        if os.path.exists(tpath):
            try:
                tprior = read_json(tpath)
            except Exception:                                   # noqa: BLE001
                tprior = {}
            if isinstance(tprior, dict):
                merged = dict(tprior)
                merged.update(self.timing)
                self.timing = merged

    def run(self):
        """跑 ``--from`` → ``--to``；第一个非 0 就停并返回它的退出码。"""
        start = PHASES.index(self.args.from_) if self.args.from_ else 0
        end = PHASES.index(self.args.to) if self.args.to else len(PHASES) - 1
        if start > end:
            raise SystemExit("--from %s 在 --to %s 之后" % (self.args.from_,
                                                           self.args.to))
        if self.args.dry_run and end > PHASES.index("compute"):
            end = PHASES.index("compute")
            log("干跑：只到阶段 4（compute），不执行、不判分")
        os.makedirs(self.out, exist_ok=True)
        if start > 0 or end < len(PHASES) - 1:
            self._merge_prior(start, end)
        for i in range(start, end + 1):
            name = PHASES[i]
            self._t0()
            log("=" * 62)
            log("阶段 %d/7：%s" % (i, name))
            try:
                rc = getattr(self, "phase_" + name)()
                if rc is None:
                    rc = EXIT_OK
                rc = int(rc)
                why = ""
            except RoundError as e:
                rc, why = e.code, e.why
                self.rec(name, rc, why, {"void": e.void} if e.void else None)
                log("!! 阶段 %s 停：%s" % (name, why))
                return rc
            except Exception as e:                          # noqa: BLE001
                if os.environ.get("STAGE4_ROUND_TRACEBACK"):
                    import traceback
                    traceback.print_exc()
                rc = PHASE_ERROR[name]
                why = "%s: %s" % (type(e).__name__, e)
                self.rec(name, rc, why)
                log("!! 阶段 %s 异常停：%s" % (name, why))
                return rc
            self.rec(name, rc, why)
            if rc != EXIT_OK:
                log("!! 阶段 %s 退出码 %d：%s" % (name, rc, why))
                return rc
        return EXIT_OK

    # ---------------------------------------------------------------- 0 预检
    def phase_preflight(self):
        expect = None
        if self.args.expect:
            expect = read_json(self.args.expect)
        got = self.board.preflight(self.args.board_repo)
        probs = evaluate_preflight(got, expect)
        write_json(self.p("preflight.json"),
                   {"at": self.deps["now"](), "board_repo": self.args.board_repo,
                    "got": got, "expect": expect, "problems": probs})
        for k in ("serial", "tree", "gripper", "realsense", "stack",
                  "dashboard"):
            v = got.get(k)
            if k == "serial" and v:
                # 指纹 + 是否匹配期望；序列号真值绝不进对话/日志（红线 3）
                v = {"fp": serial_fp(v),
                     "match": None if not (expect or {}).get("serial")
                     else v == expect["serial"]}
            log("  %-9s %s" % (k, json.dumps(v, ensure_ascii=False)))
        if probs:
            raise RoundError(EXIT_MISSING, "preflight",
                             "；".join(probs) + " —— 任一红即停，不授权、不运动")
        log("预检全绿")
        return EXIT_OK

    # ---------------------------------------------------------------- 1 观察计划
    def phase_views(self):
        if not self.args.urdf:
            raise RoundError(EXIT_MISSING, "views", "缺少 --urdf")
        chain = urkin.Chain(self.args.urdf)
        js = self.board.joints(self.p("pre"))
        q0 = _q_from_joints(js, chain)
        write_json(self.p("pre", "q_start.json"), {"q": q0})
        pre_plan = self.p("pre", "plan")
        os.makedirs(pre_plan, exist_ok=True)
        try:
            plans, _q = cr.plan_views(chain, q0, PRE_VIEWS, pre_plan)
        except pp.PlanError as e:
            raise RoundError(EXIT_PLAN, "views",
                             "跑前机位预检不过：%s —— 计划没有写出" % e)
        bursts = {v["name"]: self.b("wrist-pre", v["name"]) for v in PRE_VIEWS}
        script = cr.run_script_text(plans, self.b("pre", "plan"),
                                    self.b("pre", "exec"), burst_dirs=bursts,
                                    token_min_remaining=30.0)
        spath = self.p("pre.sh")
        with open(spath, "w", encoding="utf-8") as f:
            f.write(script)
        os.chmod(spath, 0o755)
        for pl in plans:
            log("  %-24s 机位 %-5s 相机 (%.3f, %.3f)  %d 航点"
                % (pl["name"], pl["view"], pl["slot"][0], pl["slot"][1],
                   pl["n_waypoints"]))
        write_json(self.p("pre", "plans.json"), plans)
        log("→ %s（跑前只拍 pick / pad）" % spath)
        return EXIT_OK

    # ---------------------------------------------------------------- 2 授权
    def phase_authorize(self):
        layout_blocks = lay.layout_blocks(self.args.batch, int(self.args.layout))
        print("\n" + "=" * 62)
        print("本轮摘要（给现场的人看）：")
        print("  布局 %s:%d  第 %d 次  轮次 %s"
              % (self.args.batch, int(self.args.layout), int(self.args.run),
                 self.round_id))
        cells = cm.parse_cells(self.args.cells, len(layout_blocks))
        cell_map = cm.assign_cells([b["id"] for b in layout_blocks], cells)
        for bid in _sorted_ids([b["id"] for b in layout_blocks]):
            blk = [b for b in layout_blocks if str(b["id"]) == bid][0]
            print("  块 %-2s %-11s 摆位 (%6.1f, %6.1f) mm yaw %5.1f° → 格 %d"
                  % (bid, blk["color"], blk["x"] * 1000, blk["y"] * 1000,
                     blk["yaw"], cell_map[bid]))
        print("  跑前机位：pick / pad    跑后机位：pick / free / pad")
        print("  失败即停：预检红 / 解垫不过 / 探针无解 / 任一计划预检不过 / "
              "令牌不足")
        print("=" * 62)
        print("")
        print(">>> 需要人动手：确认两指整个行程内无手无遮挡、急停在手，")
        print(">>> 然后示教器先按 STOP，看到 STOPPED 后再按 PLAY。")
        print("")
        tok, rem = self.board.wait_token(self.args.auth_timeout,
                                         self.args.auth_poll,
                                         now=self.deps["now"],
                                         sleep=self.deps["sleep"],
                                         fresh_min=self.args.auth_fresh_min)
        write_json(self.p("motion-auth.json"),
                   {"at": self.deps["now"](), "token_remaining_s": round(rem, 1),
                    "source": "板上 %s（STOP→PLAY 两步握手）"
                              % pr.AUTH_TOKEN_FILE,
                    "note": "不记令牌值；600 s 从 play_at 起算，用板上时钟判"})
        self._token = tok
        self._token_remaining = rem
        return EXIT_OK

    # ---------------------------------------------------------------- 3 观察
    def phase_observe(self):
        tok = self._current_token()
        plans = read_json(self.p("pre", "plans.json"))
        # 同一轮重跑时板上旧日志/旧帧会留在原处（工具只追加不清理）⇒ 先清。
        self.board.clear(self.b("pre", "exec"))
        for v in PRE_VIEWS:
            self.board.clear(self.b("wrist-pre", v["name"]))
        files = [(self.p("pre", "plan", p["name"]),
                  self.b("pre", "plan", p["name"])) for p in plans]
        self.board.push_many(files)
        self.board.push(self.p("pre.sh"), self.b("pre.sh"), "pre.sh")
        # 执行日志目录：只写不推，必须显式建（见 Board.mkdir 的说明）
        self.board.mkdir(self.b("pre", "exec"))
        rc, out = self.board.run_script(self.b("pre.sh"), tok)
        os.makedirs(self.p("pre"), exist_ok=True)
        with open(self.p("pre", "stdout.txt"), "w", encoding="utf-8") as f:
            f.write(out)
        log("pre.sh rc=%d" % rc)
        for line in out.strip().splitlines()[-6:]:
            log("  | " + line)
        if rc != 0:
            # 只认守卫**失败**那句（``!!!!! STOP: 令牌剩余不足 …``）：脚本第一行
            # 就有「令牌剩余时间守卫」的说明，按裸「令牌」判会把任何失败都误诊成
            # 授问题（2026-11-11 Round 0：真因是板上缺 exec 目录）。
            if "aborted_token" in out or "令牌剩余不足" in out:
                raise RoundError(EXIT_GATE, "observe",
                                 "观察段被令牌守卫拦下（rc=%d）—— 重做 "
                                 "STOP→PLAY 后再跑本轮" % rc)
            raise RoundError(EXIT_EXEC, "observe",
                             "pre.sh rc=%d（观察段失败，计划未执行）" % rc)
        # 只拉 pad：解垫下一步就要用。pick 机位的 burst 只喂**跑后的**摆位偏差表
        # （record-only），留到判分段再拉 —— 观察段在令牌窗口里，少传 100 帧
        # （2026-11-12：观察段 193.6 s 里有相当一部分是这种可以挪走的搬运）。
        for v in PRE_VIEWS:
            if v["name"] != "pad":
                continue
            local = self.p("pre", "wrist-" + v["name"])
            self.board.pull_dir(self.b("wrist-pre", v["name"]), local)
            n = len([f for f in os.listdir(local) if f.startswith("color_")])
            log("  观察 burst %-5s %d 帧" % (v["name"], n))
            if n == 0:
                raise RoundError(EXIT_EXEC, "observe",
                                 "观察机位 %s 一帧都没拉回来" % v["name"])
        log("  跑前 pick burst 留在板上，判分段再拉（只喂摆位偏差表）")
        chain = urkin.Chain(self.args.urdf)
        js = self.board.joints(self.p("pre", "after"))
        q = _q_from_joints(js, chain)
        write_json(self.p("pre", "q_after_observe.json"), {"q": q})
        log("观察后关节已重读（目标 1 的 q_start 用它，不用计划末航点）")
        return EXIT_OK

    def _current_token(self):
        tok = getattr(self, "_token", None)
        if tok:
            return tok
        tok, rem, why = self.board.token_state()
        if not tok:
            raise RoundError(EXIT_GATE, "token", "没有有效令牌：%s" % why)
        self._token = tok
        self._token_remaining = rem
        return tok

    # ---------------------------------------------------------------- 4 计算
    def phase_compute(self):
        t_start = float(self.deps["now"]())
        prior = {}
        if self.args.timing_prior and os.path.exists(self.args.timing_prior):
            prior = read_json(self.args.timing_prior)
        compute_est = float(prior.get("compute_est_s")
                            or self.args.compute_est)
        exec_est_prior = float(prior.get("exec_est_s") or self.args.exec_est)
        # 干跑（Round 0）不执行 ⇒ R1 里不该再预留执行时间。否则就是死结：
        # R1 用**还没测出来**的 T_exec_est 判闸门，把量 T_probe/T_plan 的那一步
        # 本身拦住（2026-11-11 实测：缺省 210+450+60=720 s > 600 s TTL，永远过不去）。
        gate_exec_est = 0.0 if self.args.dry_run else exec_est_prior

        # ---- R1：观察后、探针前
        #
        # **干跑（Round 0 / 离线排练）不要求令牌**：干跑只到 compute 就返回，
        # 从不发轨迹，令牌闸门在这里没有保护对象；反过来，要求令牌就会逼人为
        # 一次纯计算做 STOP→PLAY，而且板子关机时连代码都验不了。
        # 2026-11-11 用户定调：先把计算链离线跑通，再拿真机授权去跑执行。
        # 带 ``--probe-report`` 时连板子都不碰（纯离线）；否则只**读**一次剩余秒数。
        if self.args.dry_run:
            rem = None
            if not self.args.probe_report:
                _tok, rem, _why = self.board.token_state()
            ok, msg = True, ("干跑：跳过 R1（不执行、不动臂%s）"
                             % ("，也不碰板子" if self.args.probe_report
                                else ""))
        else:
            tok, rem, why = self.board.token_state()
            if not tok:
                raise RoundError(EXIT_GATE, "compute",
                                 "令牌已失效：%s —— 重做 STOP→PLAY" % why)
            ok, msg = r1_gate(rem, compute_est, gate_exec_est)
        log(msg)
        self.timing["r1"] = {"remaining_s": (None if rem is None
                                             else round(rem, 1)),
                             "compute_est_s": compute_est,
                             "exec_est_s": exec_est_prior,
                             "gate_exec_est_s": gate_exec_est,
                             "dry_run": bool(self.args.dry_run),
                             "skipped": "dry_run" if self.args.dry_run else None,
                             "ok": bool(ok)}
        if not ok:
            write_json(self.p("timing.json"), self.timing)
            raise RoundError(EXIT_GATE, "compute", msg + " —— 停，重做 STOP→PLAY")

        # ---- b̂ 种子（缺则停）
        bias = bias_setup(self.args.bias_comp, self.args.bias_seed,
                          self.args.bias_estimate)
        log("b̂：%s  种子 %s  mm  来源 %s"
            % (bias["mode"], bias["seed_mm"], bias["source"]))
        write_json(self.p("bias.json"), bias)

        # ---- 解垫
        t_pad = float(self.deps["now"]())
        grid_path = self.p("grid-pre.json")
        rc = self.deps["solve_pad"](self.p("pre", "wrist-pad"), grid_path,
                                    self.args.pad_min_frames,
                                    self.args.pad_max_spread_mm,
                                    self.args.pad_max_spread_deg)
        self.timing["pad_s"] = round(float(self.deps["now"]()) - t_pad, 1)
        if rc != 0 or not os.path.exists(grid_path):
            raise RoundError(EXIT_PLAN, "compute",
                             "解垫失败（rc=%r）：垫位姿没有共识，停" % (rc,))
        pad_doc = read_json(grid_path)
        probs = validate_pad_doc(pad_doc, self.args.pad_min_frames,
                                 self.args.pad_max_spread_mm,
                                 self.args.pad_max_spread_deg)
        if probs:
            raise RoundError(EXIT_PLAN, "compute",
                             "解垫不过：%s" % "；".join(probs))
        self._apply_pad(pad_doc, grid_path)
        log("垫心 (%.1f, %.1f) mm / yaw %.3f°（%d/%d 帧）"
            % (pad_doc["grid_center_base_mm"][0],
               pad_doc["grid_center_base_mm"][1], pad_doc["grid_yaw_base_deg"],
               pad_doc.get("n_frames_ok") or 0,
               pad_doc.get("n_frames_total") or 0))

        # ---- 本轮真值（布局表 = 真值；带上实测垫位姿）
        layout_blocks = lay.layout_blocks(self.args.batch, int(self.args.layout))
        truth = lay.scene_truth(self.args.batch, int(self.args.layout),
                                pad_pose=self._pad_pose(pad_doc, grid_path))
        truth_path = self.p("truth", "layout.json")
        write_json(truth_path, truth)
        errs = lay.validate_layout(self.args.batch, int(self.args.layout))
        if errs:
            raise RoundError(EXIT_PLAN, "compute",
                             "布局自检不过：%s" % "；".join(errs))
        mism = lay.compare_with_gen_scene(self.args.batch)
        if mism:
            raise RoundError(EXIT_PLAN, "compute",
                             "布局表与 gen_scene 不一致：%s" % "；".join(mism))

        ids = _sorted_ids([b["id"] for b in layout_blocks])
        cells = cm.parse_cells(self.args.cells, len(ids))
        cell_map = cm.assign_cells(ids, cells)

        # ---- 探针（板上一趟，只读）。``--probe-report``：复用一份**已经跑过**
        # 的探针报告（只读规划服务，不动臂、不占令牌；§5 超预算时的杠杆 1）。
        # 复用时必须核对报告里的格位/规划组与本轮一致（``_check_probe`` 干这事）。
        probe_path = self.p("probe.json")
        if self.args.probe_report:
            shutil.copyfile(self.args.probe_report, probe_path)
            self.timing["probe_s"] = None
            self.timing["probe_source"] = "复用 %s" % self.args.probe_report
            log("探针：复用 %s（本轮不重跑）" % self.args.probe_report)
        else:
            t_probe = float(self.deps["now"]())
            self.board.push(truth_path, self.b("truth", "layout.json"), "truth")
            self.board.push(grid_path, self.b("grid-pre.json"), "grid-pre.json")
            r = self.board.probe(self.args.board_repo, self.args.cells,
                                 self.args.probe_group,
                                 self.b("truth", "layout.json"),
                                 self.b("grid-pre.json"),
                                 self.b("probe.json"))
            self.timing["probe_s"] = round(float(self.deps["now"]()) - t_probe, 1)
            if r.returncode not in (0, 1):
                raise RoundError(EXIT_PLAN, "compute",
                                 "探针跑不起来（rc=%d）：%s"
                                 % (r.returncode,
                                    (r.stderr or "").strip()[:300]))
            try:
                self.board.pull_file(self.b("probe.json"), probe_path)
            except RoundError as e:
                raise RoundError(EXIT_PLAN, "compute", "探针报告没拉回来：%s" % e)
        report = read_json(probe_path)
        self._check_probe(report, ids, cell_map)

        # ---- 逐目标规划（共用 plan/，q 与布局逐段衔接；三机位只追加一次）
        t_plan = float(self.deps["now"]())
        chain = urkin.Chain(self.args.urdf)
        q = read_json(self.p("pre", "q_after_observe.json"))["q"]
        plan_dir = self.p("plan")
        os.makedirs(plan_dir, exist_ok=True)
        all_plans, layout, idx = [], [dict(b) for b in layout_blocks], 0
        scenes = report.get("scenes") or []
        attempts = {str(a["id"]): a for a in
                    (scenes[0].get("attempts") if scenes else []) or []}
        for tid in ids:
            attempt = attempts[tid]
            actions, _blocks = cr.actions_from_attempt(
                truth, attempt, cm.PAD_THICK, bias_mm=bias["applied_mm"])
            q_before = list(q)
            try:
                plans_k, layout, q = cr.plan_sequence(
                    chain, q, actions, layout, plan_dir,
                    place_clearance=self.args.place_clearance,
                    wrist_survey=False,
                    place_yaw_equiv=bool(self.args.place_yaw_equiv180),
                    start=idx, min_block_gap=0.06, gap_check="layout_gap")
            except pp.PlanError as e:
                raise RoundError(EXIT_PLAN, "compute",
                                 "块 %s 的计划预检不过：%s —— run.sh 不写、"
                                 "整轮不执行" % (tid, e))
            tgt_plans = [p for p in plans_k]
            idx += len(tgt_plans)
            all_plans.extend(tgt_plans)
            manifest = self._manifest(tid, attempt, actions, tgt_plans,
                                      q_before, q, layout, truth_path,
                                      probe_path, grid_path, bias)
            write_json(self.p("plan", "manifest-%s.json" % tid), manifest)
            log("  目标 %s：%d 个动作 → 计划 %s"
                % (tid, len(actions), [p["name"] for p in tgt_plans]))
        # 跑后三机位：只追加一次
        vplans, q = cr.plan_views(chain, q, cr.E9_VIEWS, plan_dir, start=idx)
        all_plans.extend(vplans)
        self.timing["plan_s"] = round(float(self.deps["now"]()) - t_plan, 1)

        # ---- 执行脚本 + 估时 + R2
        bursts = {v["name"]: self.b("wrist-post", v["name"])
                  for v in cr.E9_VIEWS}
        t_exec = exec_estimate(all_plans)
        self.timing["exec_est_s"] = t_exec
        if self.args.dry_run:
            rem = None
            if not self.args.probe_report:
                _tok, rem, _why = self.board.token_state()
            ok, msg = True, "干跑：跳过 R2（不写执行、不动臂）"
        else:
            tok, rem, why = self.board.token_state()
            ok, msg = r2_gate(rem, t_exec)
        log(msg)
        self.timing["r2"] = {"remaining_s": (None if rem is None
                                             else round(rem, 1)),
                             "exec_est_s": t_exec,
                             "skipped": "dry_run" if self.args.dry_run else None,
                             "ok": bool(ok)}
        if not ok:
            write_json(self.p("timing.json"), self.timing)
            raise RoundError(EXIT_GATE, "compute", msg + " —— 停，重做 STOP→PLAY")
        script = cr.run_script_text(all_plans, self.b("plan"), self.b("exec"),
                                    burst_dirs=bursts,
                                    token_min_remaining=r2_margin(t_exec))
        rpath = self.p("run.sh")
        with open(rpath, "w", encoding="utf-8") as f:
            f.write(script)
        os.chmod(rpath, 0o755)
        write_json(self.p("plan", "plans.json"), all_plans)
        write_json(self.p("plan", "final_layout.json"),
                   [{k: b.get(k) for k in ("id", "x", "y", "z", "yaw", "size")}
                    for b in layout])
        # 纯离线排练（干跑 + 复用探针报告）不碰板子：机械臂关机时也能验整条
        # 计算链，别为了推文件再要一次上板。
        if self.args.dry_run and self.args.probe_report:
            log("纯离线排练：计划只写本地，不推板子")
        else:
            self._push_run(all_plans, rpath)
        self.timing["observe_s"] = self._phase_seconds_of("observe")
        self.timing["compute_s"] = round(float(self.deps["now"]()) - t_start, 1)
        self.timing["compute_est_s"] = self.args.compute_est
        write_json(self.p("timing.json"), self.timing)
        log("执行计划 %d 份（含跑后三机位），估时 %.0f s；令牌%s"
            % (len(all_plans), t_exec,
               ("剩 %.0f s" % rem) if rem is not None else "：干跑未查"))
        return EXIT_OK

    def _phase_seconds_of(self, name):
        for row in self.phases:
            if row["phase"] == name:
                return row["seconds"]
        return None

    def _apply_pad(self, doc, grid_path):
        """实测垫位姿同时装进 ``cm`` 与 ``gen_scene``（两份源必须同步）。"""
        cm.load_grid(grid_path)
        cm._GS.configure_pad(center_mm=doc["grid_center_base_mm"],
                             yaw_deg=doc["grid_yaw_base_deg"],
                             source=os.path.basename(grid_path))
        if self.args.pad_thick is not None:
            cm.configure_pad(thickness_m=float(self.args.pad_thick))
            cm._GS.configure_pad(thickness_m=float(self.args.pad_thick))

    @staticmethod
    def _pad_pose(doc, grid_path):
        return {"center_mm": [float(v) for v in doc["grid_center_base_mm"][:2]],
                "yaw_deg": float(doc["grid_yaw_base_deg"]),
                "source": os.path.basename(grid_path), "measured": True}

    def _check_probe(self, report, ids, cell_map):
        # 规划组名先查：用错组**不会报错**，只会让每个目标 0.4 s 内
        # `no_clearing_plan`（2026-11-11 Round 0：缺省写成了仿真组
        # `ur5_manipulator`）。别把它当成「场景无解」。
        grp = report.get("group")
        if grp == "ur5_manipulator":
            raise RoundError(EXIT_PLAN, "compute",
                             "探针用的是**仿真**规划组 ur5_manipulator（真机是 "
                             "ur_manipulator）—— 全部目标会秒失败，停")
        if grp and self.args.probe_group and grp != self.args.probe_group:
            raise RoundError(EXIT_PLAN, "compute",
                             "探针报告里的规划组 %r ≠ 本次 --probe-group %r"
                             % (grp, self.args.probe_group))
        scenes = report.get("scenes") or []
        if not scenes:
            raise RoundError(EXIT_PLAN, "compute", "探针报告里没有 scenes")
        cleanup = report.get("scene_cleanup") or {}
        if not cleanup.get("ok") or cleanup.get("world") or \
                cleanup.get("attached"):
            raise RoundError(EXIT_PLAN, "compute",
                             "探针在 MoveIt 里留下幽灵块（cleanup=%r）—— "
                             "先清场景或重起 MoveIt，不执行" % (cleanup,))
        attempts = {str(a["id"]): a for a in scenes[0].get("attempts") or []}
        for tid in ids:
            a = attempts.get(tid)
            if a is None:
                raise RoundError(EXIT_PLAN, "compute",
                                 "探针没测目标 %s" % tid)
            if not a.get("ok"):
                raise RoundError(EXIT_PLAN, "compute",
                                 "目标 %s 探针无解：%s —— 停，不执行"
                                 % (tid, a.get("reason")))
            if int(a.get("cell", -1)) != int(cell_map[tid]):
                raise RoundError(EXIT_PLAN, "compute",
                                 "目标 %s 的探针格 %r ≠ 本轮格 %d"
                                 % (tid, a.get("cell"), cell_map[tid]))
            # 整轮执行是「逐目标施加到同一个世界」：第 k 个目标的清障动作，是按
            # **第 k 次探针开始时**的布局算的（每次探针都从原始布局起），而真机
            # 上前面几块已经被搬走了。S0 的块两两 ≥60 mm，探针不该报出任何清障
            # 步；一旦报出，说明「逐目标独立探针」这个前提破了（S1/S2 累积探针
            # 才会处理这种情况）。这里 fail-closed，不假装能执行。
            steps = a.get("clearing_steps") or []
            if steps:
                raise RoundError(
                    EXIT_PLAN, "compute",
                    "目标 %s 探针要求 %d 步清障，但本轮按「布局间距 ≥60 mm、"
                    "无接触」逐目标独立规划 —— 累积清障尚未实现，停"
                    % (tid, len(steps)))

    def _manifest(self, tid, attempt, actions, plans, q_before, q_after,
                  layout, truth_path, probe_path, grid_path, bias):
        blk = [b for b in layout if str(b["id"]) == tid][0]
        return {
            "probe_report": probe_path,
            "truth": truth_path,
            "batch": self.args.batch, "index": int(self.args.layout),
            "target": tid, "scene_index": 0,
            "cell": int(attempt["cell"]),
            "bias_mm": (bias["applied_mm"] if bias["mode"] == "fixed"
                        else None),
            "pad_pose": cr.pad_pose_record(grid_path),
            "pad_thick": cm.PAD_THICK,
            "survey_tool0": list(cr.E9_SURVEY_TOOL0),
            "gap_check": "layout_gap",
            "place_yaw_equiv180": bool(self.args.place_yaw_equiv180),
            "actions": actions,
            "plans": plans,
            "q_start": list(q_before),
            "q_end": list(q_after),
            "wrist_survey": False,
            "views": [dict(v) for v in cr.E9_VIEWS],
            "view_burst_dirs": {v["name"]: self.b("wrist-post", v["name"])
                                for v in cr.E9_VIEWS},
            "final_layout": [{k: b.get(k) for k in
                              ("id", "x", "y", "z", "yaw", "size")}
                             for b in layout],
            "terminal": attempt.get("terminal"),
            "align": [], "align_burst_dir": None, "held_burst_dir": None,
            "target_color": blk["color"],
        }

    def _push_run(self, plans, run_sh_path):
        pairs = [(p["path"], self.b("plan", p["name"])) for p in plans]
        pairs.append((run_sh_path, self.b("run.sh")))
        self.board.push_many(pairs)
        self.board.ssh("chmod +x %s" % self.b("run.sh"))

    # ---------------------------------------------------------------- 5 执行
    def phase_execute(self):
        tok, rem, why = self.board.token_state()
        if not tok:
            raise RoundError(EXIT_GATE, "execute",
                             "执行前没有有效令牌：%s —— 停，重做 STOP→PLAY"
                             % why)
        plans = read_json(self.p("plan", "plans.json"))
        log("执行 %d 份计划（一条 run.sh，Agent 不介入）" % len(plans))
        # 旧执行日志与旧机位帧**先清**：工具只追加不清理，重跑时会把上一次尝试
        # 的记录留进本轮证据（判分读到两轮混在一起的 jsonl）。
        self.board.clear(self.b("exec"))
        for v in cr.E9_VIEWS:
            self.board.clear(self.b("wrist-post", v["name"]))
        self.board.mkdir(self.b("exec"))     # 只写不推，必须显式建
        rc, out = self.board.run_script(self.b("run.sh"), tok)
        os.makedirs(self.p("exec"), exist_ok=True)
        with open(self.p("exec", "stdout.txt"), "w", encoding="utf-8") as f:
            f.write(out)
        self._pull_evidence(plans)
        log("run.sh rc=%d" % rc)
        for line in out.strip().splitlines()[-8:]:
            log("  | " + line)
        if rc == 0:
            if "ALL DONE" not in out:
                raise RoundError(EXIT_EXEC, "execute",
                                 "run.sh 返回 0 但没有 ALL DONE —— 不认")
            return EXIT_OK
        if "aborted_token" in out or "令牌剩余不足" in out:
            raise RoundError(EXIT_GATE, "execute",
                             "执行中令牌过期 / 不足（rc=%d）—— 本轮 void"
                             "（aborted_token），不计连续 streak，不记抓取失败"
                             % rc, void=True)
        raise RoundError(EXIT_EXEC, "execute",
                         "执行失败（rc=%d）—— 不自动重试，按失败分类停轮" % rc)

    def _pull_evidence(self, plans):
        """把板上证据拉回来。**拉不到就记缺，不掩盖真正的退出码**：

        执行失败时往往正是中间某份 jsonl / burst 不存在。若这里直接抛
        ``EXIT_MISSING``，真因（执行失败）就被「缺证据」盖掉了。
        """
        os.makedirs(self.p("exec"), exist_ok=True)
        for p in plans:
            remote = self.b("exec", p["name"] + ".jsonl")
            local = self.p("exec", p["name"] + ".jsonl")
            try:
                self.board.pull_file(remote, local)
            except RoundError as e:
                log("  !! 执行日志拉不回：%s（%s）" % (p["name"], e))
        for v in cr.E9_VIEWS:
            try:
                self.board.pull_dir(self.b("wrist-post", v["name"]),
                                    self.p("post", "wrist-" + v["name"]))
            except RoundError as e:
                log("  !! 跑后 burst %s 拉不回：%s" % (v["name"], e))

    # ---------------------------------------------------------------- 6 判分
    def phase_score(self):
        layouts = lay.layout_blocks(self.args.batch, int(self.args.layout))
        ids = _sorted_ids([b["id"] for b in layouts])
        colour_dir = self.p("colour")
        os.makedirs(colour_dir, exist_ok=True)

        # 判分与计划必须读**同一份**实测垫位姿：`--from score` 是设计里允许的
        # 单步重跑，那个进程没跑过 compute，``cm`` 还停在模型常量（与印刷垫差
        # 87.5 mm / 88.7°，台账第 295 轮）。2026-11-12 S0:1 r1 实测：彩色量
        # 用模型格心开窗 ⇒ 5 块里 3 块「窗内没看见彩色块」、2 块量到邻块。
        # 所以这里自己装一遍（幂等，全流程跑下来也不吃亏）。
        grid_path = self.p("grid-pre.json")
        if not os.path.exists(grid_path):
            raise RoundError(EXIT_MISSING, "score",
                             "缺 %s —— 判分用的垫位姿与计划必须是同一份"
                             % grid_path)
        self._apply_pad(read_json(grid_path), grid_path)

        # 跑后三机位真值（缺一个就缺，不拿默认值补）
        view_truth = []
        for v in cr.E9_VIEWS:
            out = self.p("truth", v["name"] + ".json")
            burst = self.p("post", "wrist-" + v["name"])
            rc = self.deps["truth_from_view"](burst, out, allow_empty=True)
            if rc == 0 and os.path.exists(out):
                view_truth.append((v["name"], out))
                log("  跑后真值 %-5s：%s" % (v["name"], out))
            else:
                log("  !! 跑后真值 %-5s 失败（rc=%r）—— 记缺证据" % (v["name"], rc))

        # 逐目标彩色顶面质心（窗 25 mm；期望色相只交叉核对，不改 ok）
        colour_paths = []
        bias = read_json(self.p("bias.json"))
        samples, bfail = [], []
        manifests = []
        for tid in ids:
            man_path = self.p("plan", "manifest-%s.json" % tid)
            if not os.path.exists(man_path):
                raise RoundError(EXIT_SCORE, "score",
                                 "缺 manifest %s" % man_path)
            man = read_json(man_path)
            manifests.append(man)
            tgt = [a for a in man["actions"] if a["kind"] == "target"]
            if len(tgt) != 1:
                raise RoundError(EXIT_SCORE, "score",
                                 "块 %s 的 actions 里目标动作不是 1 个" % tid)
            cmd_mm = tgt[0].get("commanded_center_mm")
            cell = int(man["cell"])
            xy_des = [v * 1000.0 for v in cm.cell_center(cell)]
            blk = [b for b in layouts if str(b["id"]) == tid][0]
            top_z = cm.PAD_THICK + float(blk["size"])
            colour = self.deps["measure_colour"](
                self.p("post", "wrist-pad"), xy_des, top_z, blk["color"],
                bc.MEASURED_HUE_DEG.get(blk["color"]),
                self.args.colour_search_mm)
            colour["target"] = tid
            colour["cell"] = cell
            colour["desired_center_mm"] = [round(v, 3) for v in xy_des]
            colour["commanded_center_mm"] = cmd_mm
            cpath = os.path.join(colour_dir, "%s.json" % tid)
            write_json(cpath, colour)
            colour_paths.append(cpath)
            if colour.get("ok") and cmd_mm is not None and \
                    bias["mode"] == "fixed":
                b, why = gb.bias_from_layer(colour["xy_mm"], cmd_mm)
                if b is None:
                    bfail.append({"target": tid, "why": why,
                                  "measured_mm": colour["xy_mm"],
                                  "commanded_mm": cmd_mm})
                else:
                    samples.append({"target": tid, "bias_mm": b})
            log("  彩色量 块 %-2s 格 %d：%s"
                % (tid, cell, ("(%+.1f, %+.1f) mm  色相 %.0f°  匹配 %s"
                               % (colour["xy_mm"][0], colour["xy_mm"][1],
                                  colour.get("hue_deg") or -1.0,
                                  colour.get("class_match")))
                   if colour.get("ok") else
                   "!! 量不出（%d/%d 帧）"
                   % (colour.get("n_frames") or 0,
                      (colour.get("n_frames") or 0)
                      + (colour.get("n_missed") or 0))))

        # 本轮 b̂ 估计（只记账 + 定下一轮种子，不改本轮已执行命令）
        run_dir = os.path.dirname(self.out)
        est_after = None
        if samples:
            # 阶段三格式：bias_history 的每一项都要是 {"bias_mm": [...]}（写回函数
            # 按 h["bias_mm"] 取值，裸列表会在这里炸）。
            doc = {"round": self.round_id,
                   "bias_history": [{"layer": s["target"],
                                     "bias_mm": s["bias_mm"]}
                                    for s in samples]}
            merged = gb.write_bias_estimate(run_dir, doc)
            if merged:
                est_after = merged.get("bias_mm")
                log("  b̂ 本轮 %d 块并入 %s/bias_estimate.json ⇒ %s（n=%d）"
                    % (len(samples), run_dir, est_after, merged.get("n")))
        bias_report = dict(bias)
        bias_report.update({
            "samples_mm": samples, "failures_mm": bfail,
            "estimate_after_mm": est_after, "n": len(samples)})
        write_json(self.p("bias.json"), bias_report)
        if bias["mode"] == "fixed" and samples:
            mags = [math.hypot(*s["bias_mm"]) for s in samples]
            log("  |b̂| 本轮：均值 %.2f mm，最大 %.2f mm（>%.0f 记失败）"
                % (sum(mags) / len(mags), max(mags), gb.BIAS_LIMIT_MM))

        # facts（机器能证明的 + 人给的安全/干预）
        # 给了 --facts 却不在 ⇒ 明说缺什么。安全/干预是人给的计数，缺了就是
        # 「没测」，不许悄悄按空字典当「都没发生」判 complete=true。
        if self.args.facts and not os.path.exists(self.args.facts):
            raise RoundError(EXIT_SCORE, "score",
                             "facts 输入不存在：%s —— 安全事件/人工干预是人给的"
                             "计数，拿不到就不判（补好证据后 --from score）"
                             % self.args.facts)
        human = read_json(self.args.facts) if self.args.facts else {}
        facts = build_facts(self.round_id, self.args.batch,
                            int(self.args.layout), layouts, bias_report, human)
        facts_path = self.p("facts.json")
        write_json(facts_path, facts)

        # round.json
        # --manifest 是 action="append"：每个文件都要带一次自己的开关。
        # 写成 ["--manifest", p1, p2, ...] 时 argparse 只认第一个参数，其余全
        # 落进 unrecognized（2026-11-12 S0:1 r1 判分就是这么炸的）。
        argv = []
        for t in ids:
            argv += ["--manifest", self.p("plan", "manifest-%s.json" % t)]
        argv += ["--facts", facts_path,
                 "--exec-log", self.p("exec"),
                 "--pad", self.p("grid-pre.json"),
                 "--out", self.p("round.json"),
                 "--round-id", self.round_id]
        if self.args.pad_spec:
            argv += ["--pad-spec", self.args.pad_spec]
        for name, path in view_truth:
            argv += ["--view-truth", "%s=%s" % (name, path)]
        for path in colour_paths:
            argv += ["--colour", path]
        rc_round = int(self.deps["round_report"](argv) or 0)
        log("round.json：%s" % ("字段齐全" if rc_round == 0 else "不完整 / 失败"))

        # 轮级布局核验（块数 / 终态 / 未动块 / 被搬块落点）
        self._write_verify(manifests, view_truth, colour_paths, layouts)

        # 手摆偏差（用跑前 pick burst 事后量；只归档，绝不改退出码）
        self._pull_pre_pick_burst()
        self._write_placement_deviation()

        # 判分器
        score_prefix = self.p("score")
        rc_score = int(self.deps["score"](
            [self.p("round.json"), "--out-prefix", score_prefix,
             "--json", self.p("score.json")]) or 0)
        if rc_round != 0 or rc_score != 0:
            raise RoundError(EXIT_SCORE, "score",
                             "判分不完整（round rc=%d / score rc=%d）—— "
                             "看 round.json 的 missing 与 score.md"
                             % (rc_round, rc_score))
        return EXIT_OK

    def _pull_pre_pick_burst(self):
        """把跑前 pick burst 从板上拉回来（观察段故意没拉 —— 它在令牌窗口里）。

        只给 ``placement-deviation.json`` 用；拉不回来就跳过偏差表，不改判分。
        """
        local = self.p("pre", "wrist-pick")
        if os.path.isdir(local) and any(
                f.startswith("color_") for f in os.listdir(local)):
            return
        try:
            self.board.pull_dir(self.b("wrist-pre", "pick"), local)
        except RoundError as e:
            log("  !! 跑前 pick burst 拉不回：%s —— 不出摆位偏差表" % e)

    def _write_placement_deviation(self):
        """跑后量一遍**手摆偏差**（只归档，缺证据就跳过，绝不改退出码）。

        正式轮是「近似手摆、跑前不实测」：块摆歪多少只有拿**跑前 pick 机位
        burst** 离线量一遍才知道。这里用与判分同一套真值工具，输出只进
        ``placement-deviation.json`` —— 不 void、不改判分（契约 §4.3）。
        """
        dev = self.deps.get("placement_deviation")
        if dev is None:
            return
        burst = self.p("pre", "wrist-pick")
        if not os.path.isdir(burst):
            log("  !! 没有跑前 pick burst —— 不出摆位偏差表（不补数）")
            return
        truth = self.p("pre", "truth-pick.json")
        try:
            rc = int(self.deps["truth_from_view"](burst, truth) or 0)
        except Exception as exc:                          # noqa: BLE001
            log("  !! 跑前真值失败：%s —— 不出摆位偏差表" % exc)
            return
        if rc != 0 or not os.path.exists(truth):
            log("  !! 跑前真值 rc=%d —— 不出摆位偏差表（不补数）" % rc)
            return
        out = self.p("placement-deviation.json")
        try:
            rc = int(dev(["--truth", truth, "--batch", self.args.batch,
                          "--layout", str(int(self.args.layout)),
                          "--out", out]) or 0)
        except Exception as exc:                          # noqa: BLE001
            log("  !! 摆位偏差失败：%s —— 只归档失败，不影响判分" % exc)
            return
        if rc != 0:
            log("  !! 摆位偏差 rc=%d —— 只归档失败，不影响本轮判分" % rc)
        else:
            log("  摆位偏差：%s（record-only，不 void）" % out)

    def _write_verify(self, manifests, view_truth, colour_paths, layouts):
        """轮级核验：块数 / 全桌终态 / 未动块 / 被搬块。缺证据就记缺。"""
        doc = {"round_id": self.round_id, "checks": [], "targets": []}
        if not view_truth:
            doc["missing"] = ["没有任何跑后机位真值"]
            write_json(self.p("round-verify.json"), doc)
            return
        post_blocks, prov = rr.merge_views(view_truth)
        doc["truth_provenance"] = prov
        final = read_json(self.p("plan", "final_layout.json"))
        actions = [a for m in manifests for a in m.get("actions") or []]
        last = manifests[-1]
        man = {"target": last["target"], "cell": last.get("cell"),
               "truth": last.get("truth"), "final_layout": final,
               "actions": actions}
        colour = None
        for path in colour_paths:
            c = read_json(path)
            if str(c.get("target")) == str(last["target"]):
                colour = c
                break
        try:
            doc["layout"] = cr.compare_layout(man, post_blocks,
                                              target_colour=colour)
        except (ValueError, KeyError) as e:
            doc["layout"] = {"ok": False, "error": str(e)[:200]}
        for tid in _sorted_ids([b["id"] for b in layouts]):
            c = None
            for path in colour_paths:
                x = read_json(path)
                if str(x.get("target")) == tid:
                    c = x
                    break
            doc["targets"].append({"target": tid,
                                   "colour_ok": bool((c or {}).get("ok")),
                                   "xy_mm": (c or {}).get("xy_mm")})
        write_json(self.p("round-verify.json"), doc)

    # ---------------------------------------------------------------- 7 归档
    def phase_archive(self):
        write_json(self.p("phases.json"), self.phases)
        write_json(self.p("round-done.json"),
                   {"round_id": self.round_id,
                    "batch": self.args.batch, "layout": int(self.args.layout),
                    "run": int(self.args.run), "at": self.deps["now"](),
                    "out_dir": self.out, "board_dir": self.board_dir,
                    "phases": [r["phase"] for r in self.phases]})
        log("轮目录：%s" % self.out)
        log("板上镜像：%s" % self.board_dir)
        return EXIT_OK


# =====================================================================
# CLI
# =====================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description="阶段 4 正式轮整轮编排（一次授权：观察 → 计算 → 执行 → 判分）")
    ap.add_argument("--batch", default="S0", help="正式批次（目前只有 S0）")
    ap.add_argument("--layout", type=int, required=True, help="布局号（S0: 1/2/3）")
    ap.add_argument("--run", type=int, default=1, help="同一布局第几次（1/2）")
    ap.add_argument("--out-dir", required=True, help="轮目录（仓外）")
    ap.add_argument("--round-id", default=None)
    ap.add_argument("--urdf", default=None, help="真机 robot.urdf（规划用）")
    ap.add_argument("--board", default=None, help="SSH 别名 / 主机")
    ap.add_argument("--board-repo", default=None,
                    help="板上本仓检出路径（探针与预检在里面跑）。阶段 4 是 "
                         "~/stage4-entry —— 探针脚本在 <repo>/scripts/ 下，"
                         "~/yolo_ros_original 里没有 stage4 脚本（2026-11-11 实测）")
    ap.add_argument("--board-dir", default=None,
                    help="板上本轮镜像目录（缺省 ~/precision/e9/<run>/<round>）")
    ap.add_argument("--cells", default=None, help="块 id 升序 → 格：8,6,4,2,0")
    ap.add_argument("--expect", default=None,
                    help="预检期望值 JSON（serial / tree_sha256 / tree_files / "
                         "wrist_usb3 / stack）；不给 ⇒ 预检判红")
    ap.add_argument("--facts", default=None,
                    help="人给的安全 / 干预 / 人眼复核 JSON（缺 ⇒ 判分不完整）")
    ap.add_argument("--pad-spec", default=None,
                    help="垫参数 JSON（footprint_m + 可选 thickness_m）")
    ap.add_argument("--pad-thick", type=float, default=None)
    ap.add_argument("--place-clearance", type=float, default=None)
    ap.add_argument("--place-yaw-equiv180", action="store_true")
    ap.add_argument("--probe-report", default=None,
                    help="复用一份已经跑过的探针报告（本地 JSON），本轮不重跑"
                         "板端探针；只读、不动臂，格位与规划组仍逐项核对")
    ap.add_argument("--probe-group", default="ur_manipulator",
                    help="MoveIt 规划组：真机 ur_manipulator（缺省）/ 仿真 "
                         "ur5_manipulator。**仿真组名在真机上不报错**：每个 "
                         "attempt ~0.4 s 就失败成 no_clearing_plan —— 2026-11-11 "
                         "Round 0 实测，白烧一次授权")

    ap.add_argument("--bias-comp", default="fixed", choices=("fixed", "none"))
    ap.add_argument("--bias-seed", default=None, help="\"bx,by\" 毫米")
    ap.add_argument("--bias-estimate", default=None,
                    help="<run>/bias_estimate.json（缺省 = out-dir 的上一级）")

    ap.add_argument("--timing-prior", default=None,
                    help="Round 0 实测的 timing.json（标定 R1/R2）")
    # 2026-11-11 Round 0 实测：解垫 0.9 s + 规划 8.6 s（5 目标 + 3 机位）
    # + 探针 6.1 s（板上一趟）≈ 16 s。缺省从 210 s 收到 90 s（约 5× 余量）；
    # 估时只影响 R1 保守程度，不是判分阈值 —— 但改了就记在台账里。
    ap.add_argument("--compute-est", type=float, default=30.0,
                    help="R1 用的轮内计算估时（秒；缺省 30 = 实测 16–25 s 的 ~1.5×）")
    ap.add_argument("--exec-est", type=float, default=400.0,
                    help="无 timing-prior 时的执行估时（秒；缺省 400 = 实测 run.sh "
                         "~390 s）")

    ap.add_argument("--pad-min-frames", type=int, default=20)
    ap.add_argument("--pad-max-spread-mm", type=float, default=2.0)
    ap.add_argument("--pad-max-spread-deg", type=float, default=0.5)
    ap.add_argument("--colour-search-mm", type=float, default=25.0)
    ap.add_argument("--auth-timeout", type=float, default=1800.0)
    ap.add_argument("--auth-poll", type=float, default=2.0)
    ap.add_argument("--auth-fresh-min", type=float, default=540.0,
                    help="令牌剩余秒数下限：低于它就算「不是这一次签发的」，"
                         "继续等新的 STOP→PLAY（600 s TTL 留 60 s 余量；"
                         "缺省挡住沿用上一次授权，红线 6）")

    ap.add_argument("--from", dest="from_", default=None, choices=PHASES,
                    help="从哪个阶段开始（单步重跑）")
    ap.add_argument("--to", default=None, choices=PHASES,
                    help="只跑到哪个阶段（含）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只到阶段 4（出计划不执行；Round 0 计时干跑用）")
    return ap


def prepare(ap, args):
    """``parse_args`` 之后的归一化（命令行缺省值 → 实值）。测试也走这里。

    把这段从 ``main`` 里拆出来，是为了让测试用**和真机完全同一条**参数路径：
    如果只在 main 里做缺省填充，测试就会自造一套参数，漂了也看不见。
    """
    if not args.board:
        ap.error("要给 --board（SSH 别名）")
    if not args.board_repo:
        ap.error("要给 --board-repo（板上本仓检出路径）")
    if not args.urdf:
        ap.error("要给 --urdf")
    if args.bias_estimate is None:
        args.bias_estimate = os.path.join(
            os.path.dirname(os.path.abspath(args.out_dir)),
            gb.BIAS_ESTIMATE_FILE)
    if args.cells is None:
        args.cells = ",".join(str(c) for c in cm.DEFAULT_CELLS)
    if args.batch not in lay.BATCHES:
        ap.error("未知批次 %r（可选 %s）" % (args.batch, sorted(lay.BATCHES)))
    if int(args.layout) not in lay.BATCHES[args.batch]:
        ap.error("%s 没有选中的布局 %r（选中 %s）"
                 % (args.batch, args.layout,
                    sorted(lay.BATCHES[args.batch])))
    if args.pad_spec and not os.path.exists(args.pad_spec):
        ap.error("--pad-spec 不存在：%s" % args.pad_spec)
    if args.expect and not os.path.exists(args.expect):
        ap.error("--expect 不存在：%s" % args.expect)
    return args


def parse_cli(ap, argv=None):
    """``argparse`` 前把 ``--bias-seed -2.791,-0.248`` 归一成 ``=`` 写法。

    b̂ 种子几乎总是负的，而 argparse 只认「看起来像负数」的独立令牌
    （``-2.79`` 行、``-2.79,-0.25`` 不行）—— 于是用户照文档写「--bias-seed
    -2.791,-0.248」会被当成「--bias-seed 缺参数」。这里替用户吸收掉：
    两种写法都能用，文档里不必逼人记 argparse 的怪癖。
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    for i in range(len(argv) - 1):
        if argv[i] == "--bias-seed" and argv[i + 1].startswith("-"):
            argv[i] = "--bias-seed=" + argv[i + 1]
            del argv[i + 1]
            break
    return ap.parse_args(argv)


def main(argv=None):
    ap = build_parser()
    args = prepare(ap, parse_cli(ap, argv))
    board = RoundBoard(args.board,
                       args.board_dir or default_board_dir(args.out_dir),
                       os.path.dirname(os.path.abspath(args.out_dir)))
    return Round(args, board).run()


if __name__ == "__main__":
    sys.exit(main())
