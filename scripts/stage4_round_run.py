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

分析段在板上算（``--analysis board``，缺省）
────────────────────────────────────────────
2026-11-12 用户定调：**本机不承担运行时任务**。解垫 / 跑前实测真值 / 跑后三
机位真值 / 彩色顶面质心全部在板上跑（``stage4_board_analysis.py``），轮内只
回传 KB 级 JSON；五组 burst 在**归档段**（整轮结束、令牌窗口之外）才拉回。
``--analysis local`` 只给离线排练（必须配 ``--dry-run``）。板上分析失败 =
停轮（fail-closed），**不许偷偷回退到本机算**。

时间闸门（900 s 硬窗口，2026-11-15 契约 v0.8 由 600 扩到 900；见契约 §5）
──────────────────────────────────
· **R1**（观察后、探针前）：`剩余 ≥ C_est + T_exec_est + 60 s`；
· **R2**（执行起手）：`剩余 ≥ T_exec_est + max(45 s, 15%·T_exec_est)`；
· 板端 ``run.sh`` 首行 ``--min-remaining`` 守卫，执行中过期记 ``aborted_token``
  —— 本轮 void、不计连续 streak，**不记抓取失败**。

b̂ 偏置补偿（契约 §4.7，继承阶段三；2026-11-15 v0.8 降级为**记录项**）
──────────────────────────────────────
`命令落点 = 格心 − b̂`，整轮 ``fixed`` 一个值；种子优先级
``--bias-seed`` > ``<run>/bias_estimate.json``；两者都没有 ⇒ **不再停轮**，
自动按未补偿跑（mode ``none``、source ``none-unseeded``），报告与 facts 标
「未补偿」—— b̂ 对 30 mm 判格容差是 ~3 mm 量级的锦上添花，为它停一轮真机
的代价远大于收益。显式 ``--bias-comp none`` 行为不变。逐块
``b̂_round = 彩色质心 − 命令`` 在判分阶段算，``|b̂| > 15 mm`` 按**失败**
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
import types

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import block_classes as bc                                      # noqa: E402
import grasp_bias as gb                                         # noqa: E402
import precision_plan as pp                                     # noqa: E402
import precision_round as pr                                    # noqa: E402
import real_motion_authorize as rma                             # noqa: E402
import home_pose as hp                                          # noqa: E402
import stage4_clear_model as cm                                 # noqa: E402
import stage4_clear_run as cr                                   # noqa: E402
import stage4_specify as spec                                   # noqa: E402
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
BURST_OVERHEAD_S = 6.0       # 每组腕部 burst（v0.8 起 10 帧，原 20）
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


#: 实测块心离布局表多远还算「同一块」。
#:
#: 手摆要求 ±3–5 mm，但**实测过 15–38 mm**（2026-11-12 S0:1 r1：用户按摆位图重摆
#: 后逐块偏 −33/−20、+35/+4、+15/+1、−9/+32、+25/−2 mm）。所以这条不是判分容差，
#: 只是「布局表与画面是不是同一个场景」的兜底：超过它多半是换错布局 / 少摆多摆 /
#: 台上有别的块 —— 盲抓只会抓空，停轮给人看。
PICK_OFFSET_LIMIT_MM = 120.0


def _mm(v):
    """米 → 毫米（真值文档统一用米，日志与偏差表用毫米）。"""
    return float(v) * 1000.0


def wrap180(deg):
    """角度折到 (−180, 180]。"""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def resolve_tower_colours(measured_blocks, manifest_blocks,
                          allow_fallback=False):
    """S3：塔下层颜色的裁决（同色塔先验 / 异色塔 fail-closed）。

    在**指定与配对之前**跑。下层的 ``meas.colour_source`` 以 ``side`` 开头时：
    - 侧带**置信**且读到颜色 ⇒ 必须与布局一致，否则停（摆错 / 清单错）；
    - 侧带**不置信**（``unknown``）：
      · 布局是**同色塔**（本层与其支撑/被支撑层的布局色相同）⇒ 用布局色填充，
        标 ``same_colour_prior``（同色塔的颜色本来就由塔顶直接量到）；
      · 布局是**异色塔** ⇒ 默认停，**不许拿清单颜色冒充实测**；
        ``allow_fallback=True`` 才按布局填充并标 ``fallback=true``。

    其余块（顶面颜色）原样返回。
    """
    man = [dict(b) for b in (manifest_blocks or [])]
    out = []
    for mb in (measured_blocks or []):
        mb = dict(mb)
        mmeas = dict(mb.get("meas") or {})
        src = str(mmeas.get("colour_source") or "")
        if not src.startswith("side"):
            out.append(mb)
            continue
        layer = int(mb.get("layer", 0))

        def _dd(m):
            return math.hypot(float(m["x"]) - float(mb["x"]),
                              float(m["y"]) - float(mb["y"]))
        cand = [m for m in man if int(m.get("layer", 0)) == layer]
        if not cand:
            raise ValueError("实测层 %d 在布局表里没有对应块" % layer)
        partner = min(cand, key=_dd)
        # 塔的另一层（同 xy 的不同层块）—— 同色塔的判据
        mate = [p for p in man if int(p.get("layer", 0)) != layer
                and math.hypot(float(p["x"]) - float(partner["x"]),
                               float(p["y"]) - float(partner["y"])) <= 0.006]
        same_colour = bool(mate) and any(
            str(p["color"]) == str(partner["color"]) for p in mate)
        got = str(mb.get("color"))
        want = str(partner["color"])
        if mmeas.get("colour_confident") and got not in ("unknown", ""):
            if got != want:
                raise ValueError(
                    "塔下层侧色读到 %s，布局是 %s（层 %d）—— 摆位或清单不一致，"
                    "停（不许用清单颜色覆盖实测）" % (got, want, layer))
            out.append(mb)
            continue
        if same_colour:
            mb["color"] = want
            mmeas["colour_source"] = "same_colour_prior"
            mmeas["colour_confident"] = False
            mmeas["colour_warn"] = ("同色塔：下层侧色不置信，按布局同色填充"
                                    "（塔顶已直接量到 %s）" % want)
        elif allow_fallback:
            mb["color"] = want
            mmeas["colour_source"] = "manifest_fallback"
            mmeas["colour_confident"] = False
            mmeas["fallback"] = True
        else:
            raise ValueError(
                "异色塔下层颜色未测出（层 %d，侧色=%s）—— 先查光照/机位；"
                "确要用布局颜色请显式 --allow-side-colour-fallback（报告会标"
                " fallback=true）"
                % (layer, mmeas.get("colour_source")))
        mb["meas"] = mmeas
        out.append(mb)
    return out


def match_measured_blocks(manifest_blocks, measured_blocks,
                          limit_mm=PICK_OFFSET_LIMIT_MM, strict_layer=False):
    """布局表 ↔ 实测块 一一配对，返回 ``[(manifest, measured, dx_mm, dy_mm)]``。

    先按**颜色**配；颜色重复（S2 同色对）时取**同色且仍自由**的最近块——
    2026-11-18 S2:5 实证：旧逻辑对重复颜色直接跨色「最近位置」贪心，第二个
    yellow 会抢到更近的 red（48 mm < 剩下的 yellow 88 mm），连锁错配成
    yellow→red、red→yellow，最后两个计划都去抓 yellow 那一对（偏心/抓空）。
    只有在**没有同色自由块**时才跨色兜底，每个实测块只用一次。配不出 1:1、
    或某一块离标称超过 ``limit_mm``，都抛 ``ValueError`` —— 抓点没有可信来源
    就不发计划。

    ``strict_layer``（S3）：**同色且同层**优先；同色块在别的层出现 ⇒ 直接停
    （塔顶/塔底互换会让臂先去抓被压住的那块），也不做跨色兜底。
    """
    man = [dict(b) for b in (manifest_blocks or [])]
    obs = [dict(b) for b in (measured_blocks or [])]
    if len(man) != len(obs):
        raise ValueError("布局表 %d 块、实测 %d 块 —— 不是同一个场景"
                         % (len(man), len(obs)))
    # 2026-11-19（S2:2）：**颜色集合不符 ⇒ fail-closed**，不换色配对。
    # 实测橙块被判成 tan 时，下面的跨色「最近位置」兜底会把
    # orange↔黄绿↔对块连锁错配（真实一轮：计划去抓错块、对块被拆成两个目标）。
    # 缺任何一个布局颜色就停，交给人核对（第 349 轮挂起的决策，本次落定）。
    man_counts, obs_counts = {}, {}
    for b in man:
        man_counts[str(b.get("color"))] = man_counts.get(str(b.get("color")), 0) + 1
    for b in obs:
        obs_counts[str(b.get("color"))] = obs_counts.get(str(b.get("color")), 0) + 1
    missing = sorted(c for c, n in man_counts.items()
                     if obs_counts.get(c, 0) < n)
    if missing:
        raise ValueError(
            "实测颜色集合不含布局所需：%s（布局 %s / 实测 %s）—— "
            "识别不可靠，fail-closed 不换色配对"
            % (", ".join(missing), man_counts, obs_counts))
    by_col = {}
    for b in obs:
        by_col.setdefault(str(b.get("color")), []).append(b)
    pairs = []
    free = list(obs)
    for m in man:
        c = str(m.get("color"))
        layer = int(m.get("layer", 0))

        def _d(o):
            return math.hypot(float(o["x"]) - float(m["x"]),
                              float(o["y"]) - float(m["y"]))
        # ① 同色且仍自由：取最近（重复颜色也走这里，不再跨色抢）
        cand = [o for o in (by_col.get(c) or []) if o in free]
        same = [o for o in cand if int(o.get("layer", 0)) == layer]
        pick = min(same, key=_d) if same else None
        if pick is None and cand and strict_layer:
            raise ValueError(
                "块 %s（%s，层 %d）的实测同色块都在别的层（%s）—— 塔的上下层"
                "互换会让臂先抓被压住的那块，停"
                % (m.get("id"), c, layer,
                   sorted({int(o.get("layer", 0)) for o in cand})))
        if pick is None:
            pick = min(cand, key=_d) if cand else None
        # ② 没有同色自由块 ⇒ 才按位置跨色兜底（颜色不可靠的场合）
        if pick is None and free and not strict_layer:
            pick = min(free, key=_d)
        if pick is None:
            raise ValueError("块 %s（%s）没有对应的实测块" % (m.get("id"), c))
        free.remove(pick)
        dx = (_mm(pick["x"]) - _mm(m["x"]))
        dy = (_mm(pick["y"]) - _mm(m["y"]))
        d = math.hypot(dx, dy)
        if d > float(limit_mm):
            raise ValueError(
                "块 %s（%s）实测 (%.1f, %.1f) 离标称 (%.1f, %.1f) 有 %.1f mm"
                "（上限 %.0f）—— 布局表与台面不是同一个场景，先核对摆位"
                % (m.get("id"), c, _mm(pick["x"]), _mm(pick["y"]),
                   _mm(m["x"]), _mm(m["y"]), d, limit_mm))
        pairs.append((m, pick, dx, dy))
    return pairs


def resolve_designation(commands, measured_blocks, manifest_blocks,
                        limit_mm=PICK_OFFSET_LIMIT_MM):
    """S3：结构化命令 → 每条解析成**唯一实测实例** → 映射到布局 id。

    判据（任一不满足即 ``ValueError``，执行前停）：
    - 每条命令在**实测真值**上 ``status == "ok"``（0 匹配 / 歧义都停）；
    - 各条落到不同的实例；
    - 覆盖全部实测块（条数 = 块数、无遗漏）；
    - 实测解析到的布局块，必须与**在布局表上**解析到的那一块是同一个
      （测量与清单不一致就停，不许拿清单坐标顶替实测）。

    返回 ``{"commands", "order", "resolved", "accuracy", "n_ok"}``；``order``
    是**布局块 id** 按命令顺序排列的执行序。
    """
    rows_m = spec.rows_from_truth(measured_blocks)
    rows_c = spec.rows_from_truth(manifest_blocks)
    pairs = match_measured_blocks(manifest_blocks, measured_blocks, limit_mm,
                                  strict_layer=True)
    m2m = {str(o["id"]): m for m, o, _dx, _dy in pairs}
    seen = set()
    resolved = []
    for raw in commands:
        r = spec.select(rows_m, raw)
        if r["status"] != "ok":
            raise ValueError("指定 %r 解析不了：%s（%s）"
                             % (raw, r["status"], r.get("reason") or ""))
        sid = str(r["instance"])
        if sid in seen:
            raise ValueError("指定 %r 与前面的命令落到同一实测实例 %s"
                             % (raw, sid))
        seen.add(sid)
        m = m2m.get(sid)
        if m is None:
            raise ValueError("指定 %r 的实测实例 %s 没有配到布局块" % (raw, sid))
        e = spec.select(rows_c, raw)
        if e["status"] != "ok":
            raise ValueError("指定 %r 在布局表上解析不了：%s（%s）"
                             % (raw, e["status"], e.get("reason") or ""))
        if str(e["instance"]) != str(m["id"]):
            raise ValueError(
                "指定 %r：实测指向布局块 %s，布局表指向 %s —— 测量与清单不一致"
                % (raw, m["id"], e["instance"]))
        resolved.append({"raw": raw, "measured_id": sid,
                         "manifest_id": str(m["id"]),
                         "confidence": r.get("confidence"),
                         "echo": r.get("block")})
    all_ids = {str(b["id"]) for b in measured_blocks}
    if seen != all_ids:
        raise ValueError("指定命令覆盖不全：解析到 %s，实测块 %s"
                         % (sorted(seen), sorted(all_ids)))
    if len(commands) != len(manifest_blocks):
        raise ValueError("命令条数 %d ≠ 布局块数 %d"
                         % (len(commands), len(manifest_blocks)))
    return {"version": 1, "kind": "stage4-designation",
            "commands": resolved,
            "order": [x["manifest_id"] for x in resolved],
            "n_commands": len(resolved), "n_ok": len(resolved),
            "accuracy": 1.0}


def merge_measured_truth(truth_doc, measured_doc,
                         limit_mm=PICK_OFFSET_LIMIT_MM, strict_layer=False):
    """抓点用**实测块位姿**（2026-11-12 用户定调，覆盖旧的「布局表即真值」）。

    布局表仍定义**块身份**（id / 颜色 / 尺寸 / 层）与判分口径；``x / y / yaw``
    一律换成跑前 pick burst 的实测值。为什么必须换：手摆偏差实测 15–38 mm，
    按标称坐标抓就是夹爪全闭的空夹（2026-11-12 S0:1 r1 第一块，回读 0.0 mm）。

    ``strict_layer``（S3）：配对必须同层；实测颜色与布局色不一致直接停
    （同色塔先验 / 显式 fallback 已在 ``resolve_tower_colours`` 里对齐）。

    返回 ``(doc, records)``：``doc`` 形状与 ``lay.scene_truth`` 相同（探针与
    ``actions_from_attempt`` 直接吃），每个块多带 ``nominal`` 与 ``meas``；
    ``records`` 是逐块偏差（只归档，不判分）。
    """
    pairs = match_measured_blocks(truth_doc.get("blocks") or [],
                                  measured_doc.get("blocks") or [], limit_mm,
                                  strict_layer=strict_layer)
    blocks, recs = [], []
    for m, o, dx, dy in pairs:
        if strict_layer and int(o.get("layer", 0)) != int(m.get("layer", 0)):
            raise ValueError("块 %s 实测层 %d ≠ 布局层 %d —— 塔上下层互换，停"
                             % (m.get("id"), int(o.get("layer", 0)),
                                int(m.get("layer", 0))))
        if strict_layer and str(o.get("color")) != str(m.get("color")):
            raise ValueError("块 %s 实测色 %s ≠ 布局色 %s（层 %d）—— 停止"
                             % (m.get("id"), o.get("color"), m.get("color"),
                                int(m.get("layer", 0))))
        nb = dict(m)
        nb["color"] = str(o.get("color", m.get("color")))
        nb["nominal_color"] = m.get("color")
        nb["x"] = float(o["x"])
        nb["y"] = float(o["y"])
        nb["yaw"] = float(o.get("yaw", m.get("yaw", 0.0)))
        nb["nominal"] = {"x_mm": round(_mm(m["x"]), 2),
                         "y_mm": round(_mm(m["y"]), 2),
                         "yaw_deg": round(float(m.get("yaw", 0.0)), 2)}
        nb["meas"] = dict(o.get("meas") or {})
        blocks.append(nb)
        recs.append({"id": m.get("id"), "color": m.get("color"),
                     "layer": int(m.get("layer", 0)),
                     "colour_source": nb["meas"].get("colour_source"),
                     "dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
                     "dist_mm": round(math.hypot(dx, dy), 2),
                     "dyaw_deg": round(wrap180(float(o.get("yaw", 0.0)) -
                                               float(m.get("yaw", 0.0))), 2),
                     "x_mm": round(_mm(o["x"]), 2),
                     "y_mm": round(_mm(o["y"]), 2)})
    doc = dict(truth_doc)
    doc["blocks"] = blocks
    doc["truth_source"] = "pick-burst-measured"
    doc["nominal_source"] = "layout-manifest"
    doc["pick_measurement"] = {
        "burst": measured_doc.get("batch"),
        "blocks": len(blocks),
        "max_offset_mm": (max(r["dist_mm"] for r in recs) if recs else None),
    }
    return doc, recs


#: 落格判据：SAM 跑后 pad 真值里「格内块」与期望格心的最大距离。
#: 格距 60 mm、邻块最近角距 45.25 mm ⇒ 45 mm 不会跨格，又能容忍摆位/测量误差。
PAD_TRUTH_PLACE_MAX_MM = 45.0


def pad_truth_place_block(pad_blocks, xy_des_mm,
                          max_mm=PAD_TRUTH_PLACE_MAX_MM):
    """SAM 跑后 pad 真值 → 离期望格心最近的块（``None`` = 该格附近没块）。

    2026-11-17 用户拍板：落格判据改吃这条**独立跑后测量链**（SAM 2D 分割给
    块颜色/位置），不再依赖彩色窗量「窗内主色相簇」——后者在垫面印刷/暖光下
    会被小暖色块骗走（s1-1-attempt5：紫块物理落点离格心 4.0 mm，彩色窗量却
    报 26.6 mm / 色相 18° / 191 px，把落格判红）。返回
    ``{"block": 真值块, "dist_mm": 距离}``；距离是水平欧氏（mm）。
    """
    best = None
    for b in pad_blocks or []:
        try:
            x = float(b["x"]) * 1000.0
            y = float(b["y"]) * 1000.0
        except (KeyError, TypeError, ValueError):
            continue
        d = math.hypot(x - float(xy_des_mm[0]), y - float(xy_des_mm[1]))
        if best is None or d < best[1]:
            best = (b, d)
    if best is None or float(best[1]) > float(max_mm):
        return None
    return {"block": best[0], "dist_mm": float(best[1])}


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


#: 按批的邻块粗判据（``build_pick`` 的 ``min_block_gap`` / 判决来源）。
#: S0 无接触布局仍走 60 mm 表级粗判，判决来源 = 布局表自检（layout_gap）；
#: S1/S2 的贴合对（中心距 ≈29.5 mm）过不了 60 mm ⇒ 放宽到 0，判决来源 =
#: **探针 OBB 已逐邻块核隙**（probe_verified）。判得必须有来源落盘
#: （``cr.plan_sequence`` 文档同句；manifest 的 ``gap_check`` 字段照记）。
GAP_POLICY_BY_BATCH = {"S0": (0.06, "layout_gap"),
                       # S3：两层塔 + 贴合对 ⇒ 表级 60 mm 不成立；邻块间隙由
                       # **累积探针**逐邻块 OBB 核（含「塔顶被搬走后」的世界）。
                       "S3": (0.0, "cumulative_probe_verified")}


def gap_policy(batch):
    """批次 → ``(min_block_gap_m, gap_check)``。未知批次（S1/S2/…）= 贴面口径。"""
    return GAP_POLICY_BY_BATCH.get(batch, (0.0, "probe_verified"))


def _placement_hint_s3(layout_blocks):
    """S3 摆块提示：塔（上下颜色/顺序）+ 贴合对 + 其余摆散 + 备用格说明。"""
    n = len(layout_blocks)
    by_id = {str(b["id"]): b for b in layout_blocks}
    tower = None
    for b in layout_blocks:
        if int(b.get("layer", 0)) == 0:
            continue
        sup = [a for a in layout_blocks if int(a.get("layer", 0)) == 0
               and math.hypot(a["x"] - b["x"], a["y"] - b["y"]) <= 0.006]
        if sup:
            tower = (sup[0], b)
            break
    parts = []
    if tower:
        lo, hi = tower
        same = lo["color"] == hi["color"]
        parts.append("两块叠成两层塔（%s：下层 %s / 上层 %s，同心 ≤3 mm、"
                     "上层压在正上方）"
                     % ("同色" if same else "异色", lo["color"], hi["color"]))
    pair_ids = lay.contact_pairs(layout_blocks)
    if len(pair_ids) == 1:
        ca, cb = (by_id[str(pid)]["color"] for pid in pair_ids[0])
        parts.append(("同色 %s 两块贴住成一对" % ca if ca == cb
                      else "%s 与 %s 两块贴住成一对（异色）" % (ca, cb)))
    group = set()
    if tower:
        group |= {str(tower[0]["id"]), str(tower[1]["id"])}
    for pid in pair_ids:
        group |= {str(p) for p in pid}
    rest = [str(b["color"]) for b in layout_blocks if str(b["id"]) not in group]
    if rest:
        parts.append("%s 三块互相摆散（离塔和贴合对都留空隙）"
                     % "/".join(rest))
    return ("%d 块自由摆：%s。全部落在拾取区内；**不要把块预放到垫上**。"
            "垫面 7 个色格 + 2 个备用格：每块按自己的颜色进对应色格，"
            "重复色的第二块进备用格（7 → 8）。" % (n, "；".join(parts)))


def placement_hint(batch, layout_blocks):
    """现场摆块提示（契约 v0.8：真机自由摆位，位置/yaw 不要求照标称）。

    贴合对颜色从布局表读（S1「X 与 Y 两块贴住（异色）」/ S2「同色 X 两块
    贴住成一对」）；其余块互相摆散。
    """
    n = len(layout_blocks)
    if batch == "S0":
        return ("%d 块自由摆：拾取区内、互相摆散（无接触）" % n)
    if batch == "S3":
        return _placement_hint_s3(layout_blocks)
    pair_ids = lay.contact_pairs(layout_blocks)
    if len(pair_ids) == 1:
        colors = []
        for pid in pair_ids[0]:
            blk = next(b for b in layout_blocks if str(b["id"]) == str(pid))
            colors.append(str(blk["color"]))
        word = ("同色 %s 两块贴住成一对" % colors[0] if colors[0] == colors[1]
                else "%s 与 %s 两块贴住成一对（异色）" % (colors[0], colors[1]))
        return ("%d 块自由摆：%s，其余 %d 块互相摆散（离贴合对和彼此都留空隙）"
                % (n, word, n - 2))
    return ("%d 块自由摆：颜色集合照本轮清单，位置 / yaw 不要求（贴合对未在表"
            "中标出，按 %s 场景描述摆）" % (n, batch))


def bias_setup(mode, seed_spec, estimate_path):
    """``--bias-comp`` / ``--bias-seed`` / ``<run>/bias_estimate.json`` → 本轮 b̂。

    缺种子且未显式 ``none`` ⇒ **不再停轮**（2026-11-15 契约 v0.8 降级为记录项）：
    自动按未补偿跑，``source="none-unseeded"``、报告与 facts 标「未补偿」。
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
        return {"mode": "none", "seed_mm": None, "applied_mm": [0.0, 0.0],
                "source": "none-unseeded",
                "note": gb.bias_note("none") + "；缺 b̂ 种子（--bias-seed 没给，"
                        "%s 也没有可用的 v%d 估计）⇒ 本轮未补偿，只记录不停轮"
                        "（2026-11-15 契约 v0.8）"
                        % (estimate_path or "<run>/bias_estimate.json",
                           gb.BIAS_ESTIMATE_VERSION)}
    applied = gb.applied_bias([], mode, seed=seed)
    return {"mode": mode, "seed_mm": [float(seed[0]), float(seed[1])],
            "applied_mm": [float(applied[0]), float(applied[1])],
            "source": source, "note": gb.bias_note(mode)}


def grid_order_from_mat(path):
    """读 ``stage2-mat.json`` 的 ``grid_order``（S3 启动期摘要用，不需要动臂）。

    运行时真正的色序来自本场解垫后写进 ``grid-pre.json`` 的那一份；这里只用于
    **授权前**打印「块 → 色格 / 备用格」。两处不一致时，解垫后的那道校验会红。
    """
    with open(os.path.expanduser(path), encoding="utf-8") as f:
        doc = json.load(f)
    order = doc.get("grid_order")
    if not order:
        raise ValueError("垫纸材质 %s 缺 grid_order —— S3 的格位没有来源"
                         % path)
    cm.validate_grid_order(order)
    return list(order)


def validate_pad_doc(doc, min_frames=10, max_spread_mm=2.0, max_spread_deg=0.5,
                     require_grid_order=False):
    """解垫文档是否够格当本轮计划与判分的**同一份**垫位姿。返回问题列表。

    ``min_frames`` 缺省 10（2026-11-15 v0.8：burst 20→10，用户拍板；共识下限
    5 帧之上仍有 2× 余量）。
    """
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
    if require_grid_order:
        try:
            order = cm.grid_order_of(doc)
        except ValueError as e:
            probs.append(str(e))
        else:
            cells = doc.get("cells") or []
            if cells:
                probs.extend(_check_cell_geometry(cells, order))
    return probs


def _check_cell_geometry(cells, order):
    """``grid-pre.json`` 的 cells 与实测垫位姿 + ``cell_center`` 同一几何。

    垫心/偏航已经 ``cm.configure_pad`` 过；这里按垫系坐标反算 base 系，与
    ``cm.cell_center(idx)`` 比（容差 0.1 mm）—— 防止 mat 的格心约定与代码分家。
    """
    probs = []
    try:
        # 垫心/偏航已经 cm.configure_pad 过：按垫系坐标反算 base 系
        ca = math.cos(cm.PAD_YAW_RAD)
        sa = math.sin(cm.PAD_YAW_RAD)
        by_idx = {int(c["idx"]): c for c in cells}
        if sorted(by_idx) != list(range(9)):
            probs.append("cells 必须恰好是 0–8（收到 %s）" % sorted(by_idx))
            return probs
        for idx, c in by_idx.items():
            want_colour = order[idx] if idx < len(order) else None
            if want_colour is not None and str(c.get("color")) != want_colour:
                probs.append("格 %d 颜色 %r ≠ grid_order[%d]=%r"
                             % (idx, c.get("color"), idx, want_colour))
            u = float(c["center_pad_mm"][0]) / 1000.0
            v = float(c["center_pad_mm"][1]) / 1000.0
            x = cm.PAD_CENTER[0] + ca * u - sa * v
            y = cm.PAD_CENTER[1] + sa * u + ca * v
            cx, cy = cm.cell_center(idx)
            d = math.hypot(x - cx, y - cy) * 1000.0
            if d > 0.1:
                probs.append("格 %d 的 mat 格心与 cell_center 差 %.2f mm"
                             % (idx, d))
    except (KeyError, TypeError, ValueError, IndexError) as e:
        probs.append("cells 几何校验失败：%s" % e)
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

    # ~/precision 可执行副本（precision_exec / capture / 授权 / traj_util…）：
    # 树哈希不覆盖它，attempt3/4 两次事故都来自副本过期 ⇒ 2026-11-15 起它的
    # md5 也是预检的一部分。fail-closed 双向：期望值没有这一段（旧 expect）或
    # 板上预检脚本太旧（got 缺字段）都算红，不许静默放行。
    want_md5 = expect.get("precision_md5")
    got_md5 = got.get("precision_md5")
    if not want_md5:
        probs.append("期望值缺 precision_md5（旧 expect 文件）—— 重跑 "
                     "stage4_preflight_expect.py 重算，别手改 JSON")
    elif not isinstance(got_md5, dict):
        probs.append("板上预检没回 precision_md5（~/precision 副本的 md5；"
                     "板上预检脚本版本旧）—— 先同步 scripts 再跑")
    else:
        bad = sorted(k for k, v in want_md5.items() if got_md5.get(k) != v)
        if bad:
            probs.append("~/precision 可执行副本与本地不一致：%s —— 先推脚本"
                         "（push 流程 md5 自带核对）再跑" % ", ".join(bad))

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
                                 "size_m": float(b["size"]),
                                 "layer": int(b.get("layer", 0))}
                  for b in spec_blocks},
        "auto_top1": None,      # S0 是结构化指定，不是自动 top-1 ⇒ N/A
        "bias": dict(bias),
    }
    for k in ("safety", "interventions", "human_review", "voids",
              "failure_class"):
        if isinstance(human, dict) and k in human:
            facts[k] = human[k]
    if isinstance(human, dict) and isinstance(human.get("designation"),
                                              dict):
        facts["designation"] = human["designation"]
        cmds = {str(c.get("manifest_id")): c.get("raw")
                for c in human["designation"].get("commands") or []}
        for bid, sp in facts["specs"].items():
            if cmds.get(bid):
                sp["command"] = cmds[bid]
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
#: 全树哈希算法 —— **唯一实现**。板上预检把它拼进 `PREFLIGHT_PY` 执行；本机的
#: ``stage4_preflight_expect.py`` 在同一份字符串上拼个 main 跑（重算期望值），
#: 两地不许各写一份（否则「期望哈希」与「板上哈希」会静默用不同口径）。
#:
#: 只覆盖源码：`__pycache__` / `*.pyc` / 构建产物 / 备份目录每次跑都会变
#: （2026-11-11 实测：跑一次 `--help` 就让文件数 1129→1130、哈希全变 ⇒ 预检
#: 假红，把「部署变了」和「有人 import 过一次」混为一谈）。`.git` 也不进 ——
#: 注意它在 **worktree 里是文件**（gitdir 指针），不只跳过同名目录，
#: 否则「本地 worktree vs 板上普通目录」会永远差一个文件（2026-11-12 实测）。
PREFLIGHT_TREE_CODE = r'''
_SKIP_DIRNAMES = {".git", "__pycache__"}
_SKIP_PREFIX = ("ros2_ws/build", "ros2_ws/install", "ros2_ws/log")


def scan_tree(repo):
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
            if name == ".git":          # worktree 里 .git 是文件，不是目录
                continue
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
    return {"sha256": h.hexdigest(), "files": n}
'''

_PRECISION_FILES = tuple(pr.BOARD_PY_FILES) + ("run.sh",)
PREFLIGHT_PY = r'''
import glob
import hashlib
import json
import os
import subprocess
import sys
''' + "PRECISION_FILES = %r\n" % (_PRECISION_FILES,) \
    + PREFLIGHT_TREE_CODE + r'''

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

# 树哈希（算法见 `PREFLIGHT_TREE_CODE`：全仓唯一实现，本机重算期望值也用它）。
out["tree"] = scan_tree(repo)

# ~/precision 可执行副本的 md5。预检树哈希只覆盖 board-repo（~/stage4-entry），
# 不覆盖这条部署通道；attempt3/4 两次真机事故都来自「板上副本停在旧版」
# （argparse 旧签名 / 旧安装树没有 set_stall_detect）。2026-11-15 起它纳入预检：
# 与期望值逐文件比对，缺文件记 "missing"（fail-closed，由 evaluate_preflight 判）。
_md5s = {}
for _name in PRECISION_FILES:
    _h = hashlib.md5()
    try:
        with open(os.path.join(os.path.expanduser("~/precision"), _name), "rb") as _fh:
            _h.update(_fh.read())
        _md5s[_name] = _h.hexdigest()
    except OSError:
        _md5s[_name] = "missing"
out["precision_md5"] = _md5s

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


def pad_thickness_and_footprint(doc, pad_thick_arg, pad_spec_path):
    """垫厚/足迹的实测来源链。返回 ``(thickness_m, footprint_m|None)``。

    ``--pad-thick`` > ``--pad-spec`` 的 ``thickness_m`` > 解垫的垫面 z
    （``T_base_pad[2,3]``）。三处皆无 ⇒ ``RoundError``（EXIT_PLAN）—— C1 哨兵
    不许「静默退回模型常量」（2026-11-13 attempt 干跑实测咬合过一次）。
    """
    thick = None
    foot = None
    if pad_thick_arg is not None:
        thick = float(pad_thick_arg)
    if pad_spec_path and os.path.exists(os.path.expanduser(pad_spec_path)):
        with open(os.path.expanduser(pad_spec_path), encoding="utf-8") as f:
            spec = json.load(f)
        if thick is None and spec.get("thickness_m") is not None:
            thick = float(spec["thickness_m"])
        if spec.get("footprint_m"):
            foot = (float(spec["footprint_m"][0]),
                    float(spec["footprint_m"][1]))
    if thick is None:
        try:
            thick = float(doc["T_base_pad"][2][3])
        except (KeyError, TypeError, IndexError, ValueError):
            raise RoundError(
                EXIT_PLAN, "compute",
                "垫厚没有实测来源：--pad-thick / --pad-spec 的 thickness_m / "
                "解垫 T_base_pad 的垫面 z 三处皆无 —— 不许静默用模型常量，停")
    return thick, foot


class BoardPort(object):
    """Board 端口（B1，2026-11-13）：编排器对「板子」的全部依赖面。

    八个阶段只经这 13 个方法碰板子；``SshBoard``（= RoundBoard）经 SSH 实现，
    ``LocalBoard`` 在**板子上本机跑编排器**时用同机实现 —— ``Round`` 的阶段
    逻辑与闸门对两者零感知。这就是「编排器上板」的结构改动点：搬的是适配器，
    不是状态机。

    端口方法：ssh / push / push_many / mkdir / clear / pull_file / pull_dir /
    joints / preflight / token_state / wait_token / run_script / analyze / probe /
    warm。
    """

    def warm(self, seg="sam"):
        """预热板端分析（模型加载）。缺省 no-op；LocalBoard 同进程预热。"""
        return None

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


def _require_clean_args(parts, what):
    for p in parts:
        if not p or " " in p or "'" in p or '"' in p:
            raise RoundError(EXIT_MISSING, "board",
                             "%s参数含空白/引号：%r" % (what, p))


def _analysis_cmd(repo, kind, opts, out_remote):
    """板上分析命令串（ssh 与 LocalBoard 共用一条构造）。"""
    argv = [str(kind)] + [str(v) for v in opts] + ["--out", out_remote]
    _require_clean_args([repo] + argv, "板上分析")
    return ("cd %s && %s/run.sh scripts/stage4_board_analysis.py %s"
            % (repo, pr.BOARD_DIR, " ".join(argv)))


def _probe_cmd(repo, cells, group, truth_remote, grid_remote, report_remote,
               extra=()):
    """板上探针命令串（ssh 与 LocalBoard 共用一条构造）。

    ``extra`` 是累积模式（S3）的附加旗标，**必须写成 ``--flag=value``**（值里
    不许有空白）—— 这里的参数全部直接拼进 shell，不走 shlex（板端 ``~`` 不展开
    的教训，见调用方 docstring），所以用 ``_require_clean_args`` 先把关。
    """
    extra = [str(x) for x in (extra or ())]
    parts = [repo, truth_remote, group, grid_remote, report_remote] + extra
    if cells:
        parts.append(str(cells))
    _require_clean_args(parts, "探针")
    cmd = ("cd %s && %s/run.sh scripts/stage4_clear_probe.py "
           "--truth %s --grid %s --group %s --report %s"
           % (repo, pr.BOARD_DIR, truth_remote, grid_remote, group,
              report_remote))
    if cells:
        cmd += " --cells %s" % cells
    for x in extra:
        cmd += " " + x
    return cmd


def resolve_validator(where, dry_run, allow_no_validator, joint_names,
                      log=log):
    """A2 段级碰撞裁决的**构造与准入策略**。

    2026-11-14 审计：`make_segment_validator` 造好了、测试齐了，但编排器
    从不构造、调用点从不传 —— 正式轮一直在**没有碰撞裁决**的情况下规划。
    本函数把「谁在什么条件下必须带着裁决规划」钉成一处：

    · ``where == "board"``（编排器在板上，与 move_group 同图）：**必构**，
      ``require=True`` —— move_group 不可达当场 RuntimeError ⇒ 停轮
      （契约 §4.3 前置「段级碰撞裁决可用且模型含夹爪」的落地）。
    · 开发机 + 干跑：不构造（离线排练没有保护对象，也不该为纯计算要求
      move_group）。
    · 开发机 + 非干跑：规划物理上够不着板上的 move_group（跨机 DDS 坏，
      AGENTS §4）⇒ 只能无裁决规划；必须显式 ``--allow-no-validator``，
      否则停轮。这不是宽松门，是「不许静默裸奔」的闸。
    """
    if where == "board":
        import stage4_moveit_validator as mv
        log("碰撞裁决：板上构造（require=True，move_group 不可达即停轮）")
        return mv.make_segment_validator(joint_names, require=True)
    if dry_run:
        log("碰撞裁决：干跑不构造（离线排练）")
        return None
    if not allow_no_validator:
        raise RoundError(
            EXIT_PLAN, "compute",
            "开发机上构造不出碰撞裁决（跨机 DDS 坏，规划够不着板上的 "
            "move_group）—— 正式轮要么 `--where board` 在板上跑编排器，"
            "要么显式 `--allow-no-validator`（响亮告警 + 记 manifest）；"
            "不许静默地在无裁决下规划真机轨迹")
    log("!! 碰撞裁决缺席（--allow-no-validator）：计划只受步长/限位/护栏"
        "约束，拦不住合法关节角的自碰撞（attempt6 C153A3 的形态）—— "
        "已记入 manifest")
    return None


class RoundBoard(BoardPort):
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
        r = subprocess.run(["scp"] + pr.ssh_opts() + ["-o", "LogLevel=ERROR", src,
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
        r = subprocess.run(["scp"] + pr.ssh_opts() + ["-o", "LogLevel=ERROR",
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
        r = subprocess.run(["rsync", "-a",
                            "-e", " ".join(["ssh"] + pr.ssh_opts()),
                            "%s:%s/" % (self.alias, remote),
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

    def analyze(self, repo, kind, opts, out_remote, out_local):
        """板上跑一次**纯分析**（解垫 / 真值 / 彩色；不动臂、不碰 ROS 服务），
        只把产出的 KB 级 JSON 拉回来。

        为什么不在本机算（2026-11-12 用户定调）：本机不能承担运行时任务；
        板子 6 核、依赖齐全，轮内拉 44 MB burst（0.33–1.4 MB/s）是架构遗留。
        板端失败一律抛错（fail-closed），**不许偷偷回退到本机算** —— 回退会把
        令牌窗口重新烧在搬运上，而且掩盖板上依赖缺失。

        ``~/precision/run.sh`` 就是 ``exec python3 "$@"``（与探针同一条通路）；
        参数不许含空白/引号（板端路径一律以 ``~`` 开头，不过 ``shlex.quote``）。
        """
        cmd = _analysis_cmd(repo, kind, opts, out_remote)
        self.ssh("mkdir -p %s" % os.path.dirname(out_remote))
        r = self.ssh(cmd)
        if r.returncode != 0:
            log("  !! 板上分析 %s rc=%d：%s"
                % (kind, r.returncode,
                   ((r.stderr or "") + (r.stdout or "")).strip()[-300:]))
            return r
        self.pull_file(out_remote, out_local)
        return r

    def probe(self, repo, cells, group, truth_remote, grid_remote,
              report_remote, extra=()):
        """板上跑一轮只读探针（MoveIt 服务，不发轨迹）。

        ``~/precision/run.sh`` **本身就是解释器包装器**（``exec python3 "$@"``），
        所以 ``run.sh`` 后面直接接脚本路径；再写一个 ``python3`` 会被当成
        「要打开的脚本名」——2026-11-11 Round 0 实测报
        ``python3: can't open file '<repo>/python3'``（rc=2），干跑停在探针这一步、
        白烧一次授权。``cd <repo>`` 保留：探针按仓库内的相对路径找脚本。

        **路径不能过 ``shlex.quote``**：板端路径一律以 ``~`` 开头，单引号会把
        波浪号焊死、不展开。改为直接要求不含空白/引号（本仓生成的都是这种）。
        """
        cmd = _probe_cmd(repo, cells, group, truth_remote, grid_remote,
                         report_remote, extra=extra)
        return self.ssh(cmd)


class LocalBoard(BoardPort):
    """``--where board`` 的适配器：编排器**就在板上跑**，同机直呼。

    与 ``RoundBoard``（SSH）逐方法同义，但执行体从「ssh 到板子」换成「本地
    bash / 文件系统」—— 八个阶段的逻辑与闸门两边零感知。板上路径仍以 ``~``
    开头（与 ssh 时代同一套命令串），文件系统操作前 ``expanduser``。

    这就是「编排器上板」的落点：开发机断网/睡眠不再触及令牌窗口内的任何
    一步（验收判据见方案文件 B3 —— 发起后 kill 本地 ssh，整轮照跑）。
    """

    def __init__(self, board_dir, exp_dir=None, alias=None):
        if not board_dir:
            raise RoundError(EXIT_MISSING, "board", "板上镜像目录为空")
        self.alias = alias or "local"
        self.board_dir = board_dir.rstrip("/")
        self.exp_dir = exp_dir
        self._root = os.path.expanduser("/")

    def _p(self, path):
        return os.path.expanduser(str(path))

    def ssh(self, cmd, quiet=True, check=False):
        """同机执行同一条命令串（bash -c；``~`` 由 shell 展开，与 ssh 一致）。"""
        r = subprocess.run(["bash", "-c", cmd], capture_output=True,
                           text=True)
        if check and r.returncode != 0:
            raise RoundError(EXIT_MISSING, "board",
                             "本地命令 rc=%d：%s（%s）"
                             % (r.returncode, cmd[:80],
                                (r.stderr or "").strip()[:200]))
        return r

    def push(self, local, remote, label=None):
        dst = self._p(remote)
        os.makedirs(os.path.dirname(dst) or "/", exist_ok=True)
        shutil.copyfile(self._p(local), dst)
        want = hashlib.md5(open(self._p(local), "rb").read()).hexdigest()
        got = hashlib.md5(open(dst, "rb").read()).hexdigest()
        if got != want:
            raise RoundError(EXIT_MISSING, "board",
                             "推送后 md5 不一致：%s" % (label or remote))
        log("  推送 %-40s md5 %s 一致" % (label or os.path.basename(remote),
                                         want[:12]))

    def push_many(self, pairs):
        for local, remote in pairs:
            self.push(local, remote)

    def mkdir(self, remote):
        if not remote:
            raise RoundError(EXIT_MISSING, "board", "板上目录为空")
        os.makedirs(self._p(remote), exist_ok=True)

    def clear(self, remote):
        shutil.rmtree(self._p(remote), ignore_errors=True)

    def pull_file(self, remote, local):
        os.makedirs(os.path.dirname(os.path.abspath(local)) or ".", exist_ok=True)
        shutil.copyfile(self._p(remote), local)

    def pull_dir(self, remote, local):
        os.makedirs(local, exist_ok=True)
        shutil.copytree(self._p(remote), local, dirs_exist_ok=True)

    def joints(self, dest_local):
        remote = pr.BOARD_SCRATCH + "/joints"
        self.mkdir(remote)
        self.ssh("%s/run.sh %s/precision_capture.py %s --joints-only --timeout 20"
                 % (pr.BOARD_DIR, pr.BOARD_DIR, remote))
        os.makedirs(dest_local, exist_ok=True)
        self.pull_file(remote + "/joint_states.json",
                       os.path.join(dest_local, "joint_states.json"))
        return json.load(open(os.path.join(dest_local,
                                           "joint_states.json")))

    def preflight(self, repo):
        cmd = "python3 - %s <<'PY'\n%s\nPY" % (shlex.quote(repo),
                                               PREFLIGHT_PY)
        r = self.ssh(cmd)
        if r.returncode != 0:
            raise RoundError(EXIT_MISSING, "preflight",
                             "预检脚本失败（rc=%d）：%s"
                             % (r.returncode, (r.stderr or "").strip()[:300]))
        try:
            return json.loads((r.stdout or "").strip().splitlines()[-1])
        except (ValueError, IndexError) as e:
            raise RoundError(EXIT_MISSING, "preflight",
                             "预检输出不是 JSON：%s（%s）"
                             % (str(e), (r.stdout or "").strip()[:200]))

    def token_state(self):
        """令牌与时钟都在本机 —— 读文件 + 本机 ``time.time()``（= 板上时钟）。"""
        path = self._p(pr.AUTH_TOKEN_FILE)
        try:
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
        except (OSError, ValueError):
            return None, None, "读不到令牌文件（%s）" % pr.AUTH_TOKEN_FILE
        tok = (doc or {}).get("token")
        if not tok:
            return None, None, "没有运动授权令牌（先做 STOP→PLAY 两步握手）"
        if not doc.get("stop_at") or not doc.get("play_at"):
            return None, None, "令牌缺 STOP/PLAY 两步记录 —— 重做握手"
        rem = float(doc.get("expires") or 0) - time.time()
        if rem <= 0:
            return None, None, "令牌已过期 %.0f 秒（本机时钟 = 板上时钟）" % (-rem)
        return tok, rem, ""

    def run_script(self, remote_script, token):
        script = self._p(remote_script)
        if " " in remote_script or "'" in remote_script:
            raise RoundError(EXIT_MISSING, "board",
                             "板上脚本路径含空格/引号：%r" % remote_script)
        if not os.path.exists(script):
            raise RoundError(EXIT_MISSING, "board",
                             "脚本不存在：%s" % remote_script)
        r = self.ssh("bash %s %s" % (remote_script, shlex.quote(token)))
        return r.returncode, ((r.stdout or "") + (r.stderr or ""))

    def warm(self, seg="sam"):
        """把 MobileSAM 加载进本进程（授权等待期间做，不进令牌窗口）。"""
        try:
            import stage4_board_analysis as ba
            ba.warm(seg)
        except Exception as e:                              # noqa: BLE001
            log("!! 模型预热失败（不致命，真值时会再加载）：%s" % e)

    def analyze(self, repo, kind, opts, out_remote, out_local):
        """板上分析：**同进程**调用 ``stage4_board_analysis``（模型只加载一次）。

        2026-11-20 实测：每起一个分析子进程都要重载 MobileSAM（≈25 s），跑前
        真值 + 跑后三机位 = 4 次。``--where board`` 的编排器与分析同在本机 ⇒
        直接 import 调用，预测器缓存跨阶段复用（授权前预热即可零加载）。
        """
        out_exp = os.path.expanduser(str(out_remote))
        argv = [str(kind)] + [
            os.path.expanduser(str(v)) if str(v).startswith("~")
            else str(v) for v in (opts or [])]
        argv += ["--out", out_exp]
        self.mkdir(os.path.dirname(out_exp))
        err = ""
        try:
            import stage4_board_analysis as ba
            rc = int(ba.main(argv) or 0)
        except SystemExit as e:
            rc = int(e.code or 0)
        except Exception as e:                              # noqa: BLE001
            rc = 1
            err = "%s: %s" % (type(e).__name__, e)
        r = types.SimpleNamespace(returncode=rc, stdout="", stderr=err)
        if r.returncode != 0:
            log("  !! 板上分析 %s rc=%d：%s"
                % (kind, r.returncode,
                   ((r.stderr or "") + (r.stdout or "")).strip()[-300:]))
            return r
        self.pull_file(out_remote, out_local)
        return r

    def probe(self, repo, cells, group, truth_remote, grid_remote,
              report_remote, extra=()):
        cmd = _probe_cmd(repo, cells, group, truth_remote, grid_remote,
                         report_remote, extra=extra)
        return self.ssh(cmd)


# =====================================================================
# 默认依赖（测试里整包替换成假的，不碰 SSH / ROS / 相机）
# =====================================================================
def default_solve_pad(burst, out, min_frames=10, max_spread_mm=2.0,
                      max_spread_deg=0.5):
    return pfb.main([burst, "--out", out,
                     "--min-frames", str(int(min_frames)),
                     "--max-spread-mm", str(float(max_spread_mm)),
                     "--max-spread-deg", str(float(max_spread_deg))])


def default_truth_from_view(burst, out, allow_empty=False, seg="sam"):
    """量一个机位的真值。

    ``allow_empty`` 只给**跑后三机位**用：干净的机位（该区域确实没有块）是
    合法结果，不能退化成「没有证据」被剔出溯源。跑前 pick 机位不传它 ——
    那里本来就必须有块，零块就是分割/相机坏了，该红。

    ``seg``：感知链选择（第 345 轮 SAM 立项、第 347 轮回放达标后缺省
    ``sam``；``grid`` = 网格链回退/对照）。
    """
    argv = ["--frame", burst, "--consensus", "--min-frames", "5",
            "--camera", "wrist", "--zone", truth_zone_for(burst),
            "--seg", seg,
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

    # ---- 分析段：板上算、只回传 JSON（``--analysis board``，缺省）
    #
    # 本机不能承担运行时任务（2026-11-12 用户定调）。``--analysis local`` 只给
    # **离线排练**（还没上真机、手里只有本地 burst）用，由 ``prepare`` 挡在
    # ``--dry-run`` 之内；正式轮永远走板上分析。
    def _analysis(self, kind, opts, out_remote, out_local):
        """板上跑一次分析 → 返回 rc；只有「rc=0 却没有产物」才抛错。

        **rc ≠ 0 不在这里抛**：同一次失败在不同阶段含义不同 —— 计算段的解垫
        失败 = 停轮（EXIT_PLAN），跑后真值失败 = 记缺证据（判分不完整），
        彩色失败 = EXIT_SCORE。由调用方决定。
        """
        fn = self.deps.get("analyze")
        opts = [str(v) for v in opts]
        if fn is not None:
            rc = int(fn(kind, opts, out_remote, out_local) or 0)
        else:
            r = self.board.analyze(self.args.board_repo, kind, opts,
                                   out_remote, out_local)
            rc = int(getattr(r, "returncode", 0) or 0)
        if rc == 0 and not os.path.exists(out_local):
            raise RoundError(EXIT_MISSING, "board",
                             "板上分析 %s 回了 0 却没有产物（%s）—— 回传失败？"
                             % (kind, out_local))
        return rc

    def _pad_mat_remote(self):
        """2D 平面标定推上板（KB 级，md5 核对；同一轮只推一次）。

        板子没有外网也没有本仓外的 ``~/robotics/notes``，所以 mat 必须随轮推。
        版本跟着 round 目录走：这份 mat 是哪天的标定，看本轮命令即可。
        """
        remote = getattr(self, "_mat_remote", None)
        if remote:
            return remote
        remote = self.b("analysis", "stage2-mat.json")
        self.board.push(self.args.pad_mat, remote, "pad-mat")
        self._mat_remote = remote
        return remote

    def _pad_spec_remote(self):
        """pad-spec（足迹/厚度真值）推上板：解垫把它嵌进 grid 文档（自洽），
        探针等**新进程**才不会因哨兵 fail-closed（C1 后置）。"""
        remote = getattr(self, "_padspec_remote", None)
        if remote:
            return remote
        remote = self.b("analysis", "pad-spec.json")
        self.board.push(self.args.pad_spec, remote, "pad-spec")
        self._padspec_remote = remote
        return remote

    def _solve_pad(self, grid_path):
        """解垫：板上算（缺省），只回传 ``grid-pre.json``。"""
        if self.args.analysis == "local":
            return int(self.deps["solve_pad"](
                self.p("pre", "wrist-pad"), grid_path,
                self.args.pad_min_frames, self.args.pad_max_spread_mm,
                self.args.pad_max_spread_deg) or 0)
        opts = ["--burst", self.b("wrist-pre", "pad"),
                "--mat", self._pad_mat_remote(),
                "--min-frames", int(self.args.pad_min_frames),
                "--max-spread-mm", float(self.args.pad_max_spread_mm),
                "--max-spread-deg", float(self.args.pad_max_spread_deg)]
        if self.args.pad_spec:
            opts += ["--pad-spec", self._pad_spec_remote()]
        return self._analysis("pad", opts, self.b("analysis", "grid-pre.json"),
                              grid_path)

    def _truth(self, burst_remote, burst_local, out_local, allow_empty=False):
        """burst → 块真值：板上算（缺省），只回传 JSON。

        跑前 pick 机位不给 ``--allow-empty``（那里本来就必须有块）；跑后三机位
        给（干净机位是合法结果）。
        """
        if self.args.analysis == "local":
            return int(self.deps["truth_from_view"](
                burst_local, out_local, allow_empty=allow_empty,
                seg=self.args.seg) or 0)
        # ``--jobs 1``：SAM 时代进程池的每个 worker 都要重载模型（≈25 s/个），
        # 串行 + 预热缓存（LocalBoard 同进程）才是最快的形态（2026-11-20 实测）。
        opts = ["--burst", burst_remote,
                "--camera", "wrist",
                "--seg", self.args.seg,
                "--zone", truth_zone_for(burst_remote),
                "--min-frames", "5", "--jobs", "1", "--img-jobs", "4"]
        if allow_empty:
            opts.append("--allow-empty")
        name = os.path.basename(os.path.normpath(out_local))
        return self._analysis("truth", opts,
                              self.b("analysis", "truth-" + name), out_local)

    def _colour(self, burst_remote, xy_mm, top_z, expect_class, hue_deg,
                out_local):
        """目标块彩色顶面质心：板上算（缺省），只回传 JSON。"""
        if self.args.analysis == "local":
            return self.deps["measure_colour"](
                burst_remote, xy_mm, top_z, expect_class, hue_deg,
                self.args.colour_search_mm)
        opts = ["--burst", burst_remote,
                "--xy", "%.6f,%.6f" % (float(xy_mm[0]), float(xy_mm[1])),
                "--top-z", "%.6f" % float(top_z),
                "--search-mm", float(self.args.colour_search_mm)]
        if hue_deg is not None:
            opts += ["--hue", "%.3f" % float(hue_deg)]
        if expect_class:
            opts += ["--expect-class", str(expect_class)]
        name = os.path.basename(os.path.normpath(out_local))
        rc = self._analysis("colour", opts,
                            self.b("analysis", "colour-" + name), out_local)
        if rc != 0:
            # 工具跑失败 ≠ 「窗里没看见块」（后者是 ok=False 的**证据**）。
            # 缺证据 ⇒ 判分不完整，不许当默认值继续。
            raise RoundError(EXIT_SCORE, "score",
                             "板上彩色量失败（rc=%d）—— 判分缺证据，停" % rc)
        return read_json(out_local)

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
        self._validator = resolve_validator(
            getattr(self.args, "where", "dev"), bool(self.args.dry_run),
            bool(getattr(self.args, "allow_no_validator", False)),
            chain.move_joints)
        self._validator_on = self._validator is not None
        try:
            plans, _q = cr.plan_views(chain, q0, PRE_VIEWS, pre_plan,
                                      validator=self._validator)
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
        if self.args.batch == "S3":
            order = grid_order_from_mat(self.args.pad_mat)
            cell_map = cm.assign_cells_by_colour(layout_blocks, order)
            print("  格位（按垫纸颜色；重复色进备用格）：%s"
                  % "  ".join("%s→%d" % kv
                              for kv in sorted(cell_map.items(),
                                               key=lambda kv: int(kv[0]))))
        else:
            cells = cm.parse_cells(self.args.cells, len(layout_blocks),
                                   batch=self.args.batch)
            cell_map = cm.assign_cells([b["id"] for b in layout_blocks], cells)
        print("  摆块（契约 v0.8 自由摆位，不要求照标称坐标）：%s"
              % placement_hint(self.args.batch, layout_blocks))
        for bid in _sorted_ids([b["id"] for b in layout_blocks]):
            blk = [b for b in layout_blocks if str(b["id"]) == bid][0]
            print("  块 %-2s %-11s L%d → 格 %d   (标称 %6.1f, %6.1f mm / %5.1f°，"
                  "仅参考)"
                  % (bid, blk["color"], int(blk.get("layer", 0)),
                     cell_map[bid], blk["x"] * 1000,
                     blk["y"] * 1000, blk["yaw"]))
        if self.args.batch == "S3":
            cmds = self._designate_commands()
            if not cmds:
                raise RoundError(EXIT_PLAN, "authorize",
                                 "S3 需要 --designate-file / --designate：每块"
                                 "一条结构化命令（%d 条）—— 不退回 id 顺序"
                                 % len(layout_blocks))
            if len(cmds) != len(layout_blocks):
                raise RoundError(EXIT_PLAN, "authorize",
                                 "指定命令 %d 条 ≠ 布局块 %d 块"
                                 % (len(cmds), len(layout_blocks)))
            for raw in cmds:
                try:
                    spec.parse_command(raw)
                except spec.CommandError as e:
                    raise RoundError(EXIT_PLAN, "authorize",
                                     "指定语法错：%s" % e)
            print("  指定（语法已核；实例解析在观察后的实测真值上做）：")
            for i, raw in enumerate(cmds):
                print("    %d. %s" % (i + 1, raw))
        print("  跑前机位：pick / pad    跑后机位：pick / free / pad"
              "    跑后回 HOME（%s）" % (hp.HOME_JOINTS_DEG,))
        print("  失败即停：预检红 / 解垫不过 / 探针无解 / 任一计划预检不过 / "
              "令牌不足")
        print("=" * 62)
        print("")
        print(">>> 需要人动手：确认两指整个行程内无手无遮挡、急停在手，")
        print(">>> 然后示教器先按 STOP，看到 STOPPED 后再按 PLAY。")
        print("")
        # 模型预热藏在人做两步握手的这段时间里（BoardPort.warm；LocalBoard 同进程）
        self.board.warm(getattr(self.args, "seg", "sam"))
        tok, rem = self.board.wait_token(self.args.auth_timeout,
                                         self.args.auth_poll,
                                         now=self.deps["now"],
                                         sleep=self.deps["sleep"],
                                         fresh_min=self.args.auth_fresh_min)
        write_json(self.p("motion-auth.json"),
                   {"at": self.deps["now"](), "token_remaining_s": round(rem, 1),
                    "source": "板上 %s（STOP→PLAY 两步握手）"
                              % pr.AUTH_TOKEN_FILE,
                    "note": "不记令牌值；900 s 从 play_at 起算，用板上时钟判"})
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
        # 图像**不拉**（缺省）：解垫 / 实测抓点 / 判分全在板上算，轮内只回传
        # KB 级 JSON；burst 本体在整轮结束的归档段再拉（2026-11-12 用户定调：
        # 本机不承担运行时任务，跨机搬运不许占令牌窗口）。
        # ``--analysis local``（只给离线排练）才在这里把 burst 拉回来。
        if self.args.analysis == "local":
            for v in PRE_VIEWS:
                local = self.p("pre", "wrist-" + v["name"])
                self.board.pull_dir(self.b("wrist-pre", v["name"]), local)
                n = len([f for f in os.listdir(local)
                         if f.startswith("color_")])
                log("  观察 burst %-5s %d 帧（--analysis local）"
                    % (v["name"], n))
                if n == 0:
                    raise RoundError(EXIT_EXEC, "observe",
                                     "观察机位 %s 一帧都没拉回来" % v["name"])
        else:
            log("  观察 burst 留在板上做分析（只回传 JSON；图像整轮后归档）")
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
        # **续跑（--from compute）没有 authorize 段**：令牌可能还没签发。
        # 这里补一次**同样的物理两步握手**（只读核验 STOP→PLAY），否则续跑
        # 会拿旧令牌/无令牌直接停。正常轮次 authorize 已把 _token 设好，不走这里。
        if (not self.args.dry_run and not getattr(self, "_token", None)):
            print("")
            print(">>> 需要人动手：确认两指整个行程内无手无遮挡、急停在手，")
            print(">>> 然后示教器先按 STOP，看到 STOPPED 后再按 PLAY。")
            print("")
            tok, rem = self.board.wait_token(
                self.args.auth_timeout, self.args.auth_poll,
                now=self.deps["now"], sleep=self.deps["sleep"],
                fresh_min=self.args.auth_fresh_min)
            self._token = tok
            self._token_remaining = rem
            write_json(self.p("motion-auth.json"),
                       {"at": self.deps["now"](),
                        "token_remaining_s": round(float(rem), 1),
                        "source": "续跑 compute 段的 STOP→PLAY 两步握手"
                                  "（板上 %s）" % pr.AUTH_TOKEN_FILE,
                        "note": "不记令牌值；900 s 从 play_at 起算，用板上时钟判"})
            log("续跑授权成立：令牌还剩 %.0f s" % float(rem))

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
        if self.args.analysis == "local":
            log("  解垫在本机算（--analysis local：离线排练）")
        else:
            log("  解垫在板上算（只回传 grid-pre.json）")
        rc = self._solve_pad(grid_path)
        self.timing["pad_s"] = round(float(self.deps["now"]()) - t_pad, 1)
        if rc != 0 or not os.path.exists(grid_path):
            raise RoundError(EXIT_PLAN, "compute",
                             "解垫失败（rc=%r）：垫位姿没有共识，停" % (rc,))
        pad_doc = read_json(grid_path)
        # 先装垫位姿再校验：S3 的色序/格心几何校验要调 ``cm.cell_center``，
        # 而新进程里垫参数是「未实测」哨兵 —— 校验前不装就会抛 PadNotMeasured
        # （2026-11-20 轮 A 实测：tests 里装了模型默认值，所以没暴露）。
        self._apply_pad(pad_doc, grid_path)
        probs = validate_pad_doc(pad_doc, self.args.pad_min_frames,
                                 self.args.pad_max_spread_mm,
                                 self.args.pad_max_spread_deg,
                                 require_grid_order=(
                                     self.args.batch == "S3"))
        if probs:
            raise RoundError(EXIT_PLAN, "compute",
                             "解垫不过：%s" % "；".join(probs))
        log("垫心 (%.1f, %.1f) mm / yaw %.3f°（%d/%d 帧）"
            % (pad_doc["grid_center_base_mm"][0],
               pad_doc["grid_center_base_mm"][1], pad_doc["grid_yaw_base_deg"],
               pad_doc.get("n_frames_ok") or 0,
               pad_doc.get("n_frames_total") or 0))

        # ---- 本轮真值：布局表定**块身份**，抓点位姿用跑前 pick burst 实测
        layout_blocks = lay.layout_blocks(self.args.batch, int(self.args.layout))
        truth = lay.scene_truth(self.args.batch, int(self.args.layout),
                                pad_pose=self._pad_pose(pad_doc, grid_path))
        truth_path = self.p("truth", "layout.json")
        errs = lay.validate_layout(self.args.batch, int(self.args.layout))
        if errs:
            raise RoundError(EXIT_PLAN, "compute",
                             "布局自检不过：%s" % "；".join(errs))
        mism = lay.compare_with_gen_scene(self.args.batch)
        if mism:
            raise RoundError(EXIT_PLAN, "compute",
                             "布局表与 gen_scene 不一致：%s" % "；".join(mism))
        desig = None
        if self.args.truth_source == "measured":
            truth, desig = self._measured_truth(truth)
        else:
            log("抓点：用布局表标称坐标（--truth-source manifest）—— 手摆偏差"
                "不进计划，偏差只进摆位表")
        write_json(truth_path, truth)

        ids = _sorted_ids([b["id"] for b in layout_blocks])
        if self.args.batch == "S3":
            if self.args.truth_source != "measured":
                raise RoundError(EXIT_PLAN, "compute",
                                 "S3 必须用跑前实测真值（--truth-source "
                                 "measured）；布局标称坐标只做身份先验")
            grid_order = cm.grid_order_of(pad_doc)
            cell_map = cm.assign_cells_by_colour(truth["blocks"], grid_order)
            # 执行顺序 = 结构化指定给出的顺序（命令即意图；不退回 id 升序）
            order = ([str(x) for x in desig["order"]] if desig
                     else [str(t) for t in ids])
            write_json(self.p("truth", "cell-map.json"),
                       {"grid_order": grid_order, "cell_map": cell_map,
                        "order": order,
                        "dest_mode": self.args.clear_dest})
            log("  格位（按颜色）：%s"
                % "  ".join("%s→%d" % kv
                            for kv in sorted(cell_map.items(),
                                             key=lambda kv: int(kv[0]))))
        else:
            cells = cm.parse_cells(self.args.cells, len(ids),
                                   batch=self.args.batch)
            cell_map = cm.assign_cells(ids, cells)
            grid_order = None
            order = [str(t) for t in ids]

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
            extra = []
            if self.args.batch == "S3":
                extra = ["--cumulative", "--assign-by=colour",
                         "--dest-mode=%s" % self.args.clear_dest,
                         "--target-order=%s" % ",".join(order)]
            r = self.board.probe(self.args.board_repo, self.args.cells,
                                 self.args.probe_group,
                                 self.b("truth", "layout.json"),
                                 self.b("grid-pre.json"),
                                 self.b("probe.json"),
                                 extra=extra)
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
        fp = (cm.scene_fingerprint(truth, order, cell_map,
                                   self.args.clear_dest, grid_order)
              if self.args.batch == "S3" else None)
        self._check_probe(report, ids, cell_map, order=order,
                          dest_mode=self.args.clear_dest,
                          batch=self.args.batch, fingerprint=fp)

        # ---- 逐目标规划（共用 plan/，q 与布局逐段衔接；三机位只追加一次）
        t_plan = float(self.deps["now"]())
        chain = urkin.Chain(self.args.urdf)
        q = read_json(self.p("pre", "q_after_observe.json"))["q"]
        plan_dir = self.p("plan")
        os.makedirs(plan_dir, exist_ok=True)
        # --from compute 单步重跑时 views 阶段没跑过 ⇒ 这里自建（幂等）。
        if getattr(self, "_validator", None) is None and \
                not getattr(self, "_validator_done", False):
            self._validator = resolve_validator(
                getattr(self.args, "where", "dev"), bool(self.args.dry_run),
                bool(getattr(self.args, "allow_no_validator", False)),
                chain.move_joints)
        self._validator_done = True
        self._validator_on = getattr(self, "_validator", None) is not None
        # 抓点/邻块几何必须吃**本轮真正要抓的那份块位姿**：``--truth-source
        # measured`` 时 ``truth`` 已被换成跑前 pick burst 实测（``_measured_truth``），
        # 若这里还拿布局表的标称坐标喂 ``plan_sequence``，就是「量了不用」——
        # 2026-11-12 S0:1 r1 attempt5 空夹（回读 0.0 mm）的根因：计划 meta 里
        # ``e9.from_xy`` 是实测，而 IK 目标是标称，两者差 25 mm。
        layout_src = truth["blocks"] if self.args.truth_source == "measured" \
            else layout_blocks
        all_plans, layout, idx = [], [dict(b) for b in layout_src], 0
        scenes = report.get("scenes") or []
        attempts = {str(a["id"]): a for a in
                    (scenes[0].get("attempts") if scenes else []) or []}
        if self.args.batch == "S3":
            all_plans, q, idx = self._plan_s3(
                chain, q, plan_dir, truth, report, bias, truth_path,
                probe_path, grid_path, cell_map, order)
        for tid in ([] if self.args.batch == "S3" else ids):
            attempt = attempts[tid]
            actions, _blocks = cr.actions_from_attempt(
                truth, attempt, cm.PAD_THICK, bias_mm=bias["applied_mm"])
            q_before = list(q)
            try:
                gap_m, gap_check = gap_policy(self.args.batch)
                plans_k, layout, q = cr.plan_sequence(
                    chain, q, actions, layout, plan_dir,
                    place_clearance=self.args.place_clearance,
                    wrist_survey=False,
                    place_yaw_equiv=bool(self.args.place_yaw_equiv180),
                    start=idx, min_block_gap=gap_m, gap_check=gap_check,
                    # 最后一个块要为**随后追加的三机位**选收尾分支：只参与选支、
                    # 不写进它的计划（见 cr.view_targets / build_pick 的
                    # post_targets）。不给的话两条链可能差整整 360°，compute 时
                    # 才发现机位拍不了（2026-11-12 attempt6）。
                    tail_targets=(cr.view_targets() if tid == ids[-1] else None),
                    # A2：段级碰撞裁决随计划进 solve_path（构造与准入见
                    # resolve_validator；2026-11-14 审计前这里从不传）
                    validator=getattr(self, "_validator", None))
            except pp.PlanError as e:
                raise RoundError(EXIT_PLAN, "compute",
                                 "块 %s 的计划预检不过：%s —— run.sh 不写、"
                                 "整轮不执行" % (tid, e))
            tgt_plans = [p for p in plans_k]
            # 释放扫掠 vs 已就位块（台账第三百四十一轮）：探针从原始布局重来、
            # 看不见「先放的目标已经在格上」；这里按每份计划的**命令落点**
            # （commanded_center_mm = 格心 − b̂，就是释放轴）补一道静态判。
            # 50 mm 开爪对 60 mm 格距恒不冲突；开爪回 95 当场红。
            placed = []
            for p in all_plans:
                c = p.get("commanded_center_mm")
                if c:
                    placed.append({"x": float(c[0]) / 1000.0,
                                   "y": float(c[1]) / 1000.0,
                                   "size": float(cm.BLOCK_M),
                                   "id": p.get("index")})
            cur = tgt_plans[-1].get("commanded_center_mm") \
                if tgt_plans else None
            sweep = (cm.release_sweep_conflicts(
                (float(cur[0]) / 1000.0, float(cur[1]) / 1000.0),
                cr.PLACE_OPEN_MM, placed) if cur else [])
            if sweep:
                raise RoundError(EXIT_PLAN, "compute",
                                 "块 %s 的释放扫掠会碰到已就位块 %s"
                                 "（开爪 %.0f mm）—— 计划期拒发"
                                 % (tid, sweep, cr.PLACE_OPEN_MM))
            idx += len(tgt_plans)
            all_plans.extend(tgt_plans)
            manifest = self._manifest(tid, attempt, actions, tgt_plans,
                                      q_before, q, layout, truth_path,
                                      probe_path, grid_path, bias)
            # 只增字段：扫掠检查口径与结果（冲突已在上面 fail-closed）
            manifest["sweep_check"] = {"opening_mm": cr.PLACE_OPEN_MM,
                                       "n_placed_before": len(placed),
                                       "conflicts": sweep}
            write_json(self.p("plan", "manifest-%s.json" % tid), manifest)
            log("  目标 %s：%d 个动作 → 计划 %s"
                % (tid, len(actions), [p["name"] for p in tgt_plans]))
        # 跑后三机位：只追加一次
        vplans, q = cr.plan_views(chain, q, cr.E9_VIEWS, plan_dir, start=idx,
                                  validator=getattr(self, "_validator", None))
        all_plans.extend(vplans)
        # 跑后回 HOME（2026-11-15 用户定：所有任务跑完都回标准停靠位姿）——
        # **计划内的收尾动作**，随整轮授权执行（不是轮后偏离）。段级裁决同样
        # 盖章；闸门（限位/奇异/缠绕/高度护栏/大跳插点）在 hp.plan_park 内。
        park, _pk = hp.plan_park(chain, q)
        if getattr(self, "_validator", None) is not None:
            vok, vwhy = self._validator(q, park["waypoints"][-1]["q"])
            if not vok:
                raise RoundError(EXIT_PLAN, "compute",
                                 "回 HOME 的段级碰撞裁决拒绝：%s —— 不写计划"
                                 % vwhy)
            park["meta"]["collision_validator"] = True
        pname = "%02d-home.json" % (idx + len(vplans) + 1)
        ppath = os.path.join(plan_dir, pname)
        write_json(ppath, park)
        all_plans.append({"index": idx + len(vplans) + 1, "name": pname,
                          "path": ppath, "kind": "park", "block": None,
                          "slot": None, "from_xy": None,
                          "grasp_yaw_deg": 0.0,
                          "grasp_yaw_equiv180": False,
                          "place_yaw_equiv180": False,
                          "gap_check": "n/a",
                          "n_waypoints": len(park["waypoints"])})
        log("  跑后回 HOME（%s）：%d 个航点%s"
            % (hp.HOME_JOINTS_DEG, len(park["waypoints"]),
               "，段级裁决已盖章" if park["meta"]["collision_validator"] else ""))
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
        """实测垫位姿同时装进 ``cm`` 与 ``gen_scene``（两份源必须同步）。

        厚度/足迹的实测来源链（C1 后置，2026-11-13）：``--pad-thick`` >
        ``--pad-spec`` 的 ``thickness_m`` > **解垫的垫面 z**（``T_base_pad[2,3]``，
        台面 = base 0）。全都缺 ⇒ ``RoundError``（fail-closed）—— C1 哨兵已把
        「静默用模型常量」堵死（attempt 干跑实测：没有这段就会 PadNotMeasured）。
        足迹只认 ``--pad-spec`` 的 ``footprint_m``（11-11 卷尺实测 0.393×0.21；
        模型默认 0.30×0.30 与实垫差 90 mm，会判错「垫外」，状态层 §4 早有记录）。
        """
        cm.load_grid(grid_path)
        cm._GS.configure_pad(center_mm=doc["grid_center_base_mm"],
                             yaw_deg=doc["grid_yaw_base_deg"],
                             source=os.path.basename(grid_path))
        thick, foot = pad_thickness_and_footprint(
            doc, self.args.pad_thick, self.args.pad_spec)
        cm.configure_pad(thickness_m=thick, footprint_m=foot)
        cm._GS.configure_pad(thickness_m=thick, footprint_m=foot)

    @staticmethod
    def _pad_pose(doc, grid_path):
        return {"center_mm": [float(v) for v in doc["grid_center_base_mm"][:2]],
                "yaw_deg": float(doc["grid_yaw_base_deg"]),
                "source": os.path.basename(grid_path), "measured": True}

    def _check_probe(self, report, ids, cell_map, order=None,
                     dest_mode=None, batch=None, fingerprint=None):
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
        if (batch or self.args.batch) == "S3":
            self._check_probe_s3(report, scenes[0], attempts, ids, cell_map,
                                 order or [], dest_mode, fingerprint)
            return
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

    def _check_probe_s3(self, report, scene, attempts, ids, cell_map,
                        order, dest_mode, fingerprint):
        """S3 累积探针的准入：模式/顺序/格位/指纹/次数/覆盖/终态，任一不符即停。"""
        probs = []
        if str(scene.get("mode") or report.get("mode")) != "cumulative":
            probs.append("探针不是累积模式（mode=%r）"
                         % (scene.get("mode") or report.get("mode")))
        want_mode = dest_mode or "cell"
        if str(scene.get("dest_mode") or want_mode) != str(want_mode):
            probs.append("探针落点模式 %r ≠ 本轮 %r"
                         % (scene.get("dest_mode"), want_mode))
        got_order = [str(x) for x in (scene.get("order") or [])]
        want_order = [str(x) for x in (order or [])]
        if got_order != want_order:
            probs.append("探针目标顺序 %s ≠ 本轮 %s" % (got_order, want_order))
        got_cells = {str(k): int(v) for k, v in
                     (scene.get("cell_map") or {}).items()}
        want_cells = {str(k): int(v) for k, v in (cell_map or {}).items()}
        if got_cells != want_cells:
            probs.append("探针格位 %s ≠ 本轮 %s" % (got_cells, want_cells))
        if fingerprint and scene.get("scene_fingerprint") != fingerprint:
            probs.append("场景指纹不符（报告 %r ≠ 本轮 %r）—— 旧探针报告配新摆位"
                         % (scene.get("scene_fingerprint"), fingerprint))
        probs.extend(scene.get("pick_problems") or [])
        probs.extend(scene.get("coverage_problems") or [])
        term = scene.get("batch_final") or scene.get("terminal") or {}
        if not term.get("ok"):
            probs.append("批末终态不合法（%s）"
                         % "；".join(term.get("reasons") or []))
        for tid in [str(t) for t in ids]:
            a = attempts.get(tid)
            if a is None:
                probs.append("探针没测目标 %s" % tid)
                continue
            if not a.get("ok"):
                probs.append("目标 %s 探针失败：%s" % (tid, a.get("reason")))
                continue
            if a.get("cell") is not None and tid in cell_map:
                if int(a["cell"]) != int(cell_map[tid]):
                    probs.append("目标 %s 的探针格 %r ≠ 本轮 %r"
                                 % (tid, a.get("cell"), cell_map[tid]))
            for st in a.get("clearing_steps") or []:
                bid = str(st.get("move"))
                if not st.get("absorbed"):
                    continue
                if bid not in cell_map:
                    probs.append("被吸收的块 %s 没有格位" % bid)
                elif st.get("cell") is None or \
                        int(st["cell"]) != int(cell_map[bid]):
                    probs.append("被吸收的块 %s 落在格 %r ≠ 自己的格 %r"
                                 % (bid, st.get("cell"), cell_map[bid]))
        if probs:
            raise RoundError(EXIT_PLAN, "compute",
                             "S3 累积探针不过：%s —— 停，不执行"
                             % "；".join(probs))

    def _plan_s3(self, chain, q_in, plan_dir, truth, report, bias,
                 truth_path, probe_path, grid_path, cell_map, order):
        """S3：累积报告 → 一份平铺动作表 → **一次** ``plan_sequence`` → 逐目标 manifest。

        动作表里每个块恰好一次（被吸收的塔顶 = 它自己的目标动作）。释放扫掠
        按全局顺序累计已就位块（清障落格也算），任一冲突 fail-closed。
        """
        pack = cr.actions_from_cumulative(
            truth, report, cm.PAD_THICK, bias_mm=bias["applied_mm"])
        actions = pack["actions"]
        gap_m, gap_check = gap_policy(self.args.batch)
        q_start = list(q_in)
        try:
            plans, layout, q = cr.plan_sequence(
                chain, q_start, actions, [dict(b) for b in truth["blocks"]],
                plan_dir, place_clearance=self.args.place_clearance,
                wrist_survey=False,
                place_yaw_equiv=bool(self.args.place_yaw_equiv180),
                start=0, min_block_gap=gap_m, gap_check=gap_check,
                tail_targets=cr.view_targets(),
                validator=getattr(self, "_validator", None),
                from_check_mm=2.0)
        except pp.PlanError as e:
            raise RoundError(EXIT_PLAN, "compute",
                             "S3 计划预检不过：%s —— run.sh 不写、整轮不执行"
                             % e)
        if len(plans) != len(actions):
            raise RoundError(EXIT_PLAN, "compute",
                             "计划数 %d ≠ 动作数 %d" % (len(plans), len(actions)))
        placed = []
        sweeps = []
        for a, p in zip(actions, plans):
            c = p.get("commanded_center_mm")
            if not c:
                sweeps.append(None)
                continue
            sweep = cm.release_sweep_conflicts(
                (float(c[0]) / 1000.0, float(c[1]) / 1000.0),
                cr.PLACE_OPEN_MM, placed)
            sweeps.append({"opening_mm": cr.PLACE_OPEN_MM,
                           "n_placed_before": len(placed),
                           "conflicts": sweep})
            if sweep:
                raise RoundError(EXIT_PLAN, "compute",
                                 "块 %s 的释放扫掠会碰到已就位块 %s（开爪 "
                                 "%.0f mm）—— 计划期拒发"
                                 % (a.get("block"), sweep, cr.PLACE_OPEN_MM))
            placed.append({"x": float(c[0]) / 1000.0,
                           "y": float(c[1]) / 1000.0,
                           "size": float(cm.BLOCK_M),
                           "id": a.get("block")})
        scene = (report.get("scenes") or [{}])[0]
        attempts = {str(x["id"]): x for x in
                    (scene.get("attempts") or [])}
        picked = dict(scene.get("picked") or {})
        q_end = list(q)
        for tid in [str(t) for t in order]:
            own = (pack["owner"] or {}).get(tid) or {}
            idxs = list(own.get("actions") or [])
            acts = [actions[i] for i in idxs]
            tplans = [plans[i] for i in idxs]
            att = attempts.get(tid) or {"id": tid, "status": "absorbed",
                                        "cell": cell_map.get(tid)}
            manifest = self._manifest(tid, att, acts, tplans, q_start, q_end,
                                      layout, truth_path, probe_path,
                                      grid_path, bias)
            manifest["dest_mode"] = self.args.clear_dest
            manifest["cell_color"] = (cm.DEFAULT_GRID_ORDER[int(cell_map[tid])]
                                      if tid in cell_map else None)
            manifest["action_index"] = min(idxs) if idxs else None
            manifest["pick_count"] = int(picked.get(tid, 0) or 0)
            manifest["absorbed_by"] = own.get("absorbed_by")
            own_target = own.get("target_action")
            manifest["clearing"] = [
                {"index": i, "kind": actions[i].get("kind"),
                 "block": actions[i].get("block"),
                 "cell": actions[i].get("cell"),
                 "absorbed": actions[i].get("absorbed"),
                 "absorbed_by": actions[i].get("absorbed_by"),
                 "plan": plans[i].get("name"),
                 "from_xy": actions[i].get("from_xy"),
                 "desired_center_mm": actions[i].get("desired_center_mm"),
                 "commanded_center_mm": actions[i].get("commanded_center_mm")}
                for i in idxs if i != own_target]
            manifest["sweep_check"] = (sweeps[idxs[-1]] if idxs else None)
            write_json(self.p("plan", "manifest-%s.json" % tid), manifest)
            log("  目标 %s：%d 个动作（清障 %d）"
                % (tid, len(acts), len(manifest["clearing"])))
        log("  S3 动作表 %d 个（每块恰好 1）→ %d 份计划"
            % (len(actions), len(plans)))
        return plans, q, len(plans)

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
            "gap_check": gap_policy(self.args.batch)[1],
            "place_yaw_equiv180": bool(self.args.place_yaw_equiv180),
            # 可追溯（改造方案 §3 不变量 3）：这轮计划是谁规划的、有没有带着
            # 段级碰撞裁决出闸（2026-11-14 审计：此前 manifest 层答不了这两问）
            "planner_backend": "urkin",
            "collision_validator": bool(getattr(self, "_validator_on", False)),
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
        # 上一轮残留的软停请求**先清**：它是「让这一轮早停」的一次性减权指令，
        # 留着会让下一轮在第一个计划边界就停（watch_round --stop 每次要重新写）。
        self.board.ssh("rm -f %s" % rma.STOP_REQUEST_FILE)
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
        if "stop_requested" in out:
            # 人为软停（watch_round --stop，计划边界生效）：同为 void、不计
            # streak、不记抓取失败，但原因与令牌过期分开记。
            write_json(self.p("void.json"),
                       {"reason": "stop_requested",
                        "at": self.deps["now"](),
                        "exec_rc": rc,
                        "note": "人为软停 ⇒ 本轮 void：不计连续 streak、"
                                "不记抓取失败；正在走的轨迹已按计划边界停"})
            raise RoundError(EXIT_GATE, "execute",
                             "人为软停（rc=%d）—— 本轮 void"
                             "（stop_requested），不计连续 streak，不记抓取失败"
                             % rc, void=True)
        if "aborted_token" in out or "令牌剩余不足" in out:
            # 令牌过期的**机器证据落盘**：`--from score` 单步重跑判分时，
            # 不再依赖人记得在 facts 里填 voids（2026-11-14 审计：此前这条
            # 只活在 stdout 字符串里，直接走 round_report 会被判 missing）。
            write_json(self.p("void.json"),
                       {"reason": "aborted_token",
                        "at": self.deps["now"](),
                        "exec_rc": rc,
                        "note": "执行中令牌过期/不足 ⇒ 本轮 void：不计连续 "
                                "streak、不记抓取失败（契约 §4.6）"})
            raise RoundError(EXIT_GATE, "execute",
                             "执行中令牌过期 / 不足（rc=%d）—— 本轮 void"
                             "（aborted_token），不计连续 streak，不记抓取失败"
                             % rc, void=True)
        raise RoundError(EXIT_EXEC, "execute",
                         "执行失败（rc=%d）—— 不自动重试，按失败分类停轮" % rc)

    def _pull_evidence(self, plans):
        """把板上**小**证据拉回来（执行 jsonl，KB 级）。

        图像（burst）**不在这里拉**：判分在板上算（``--analysis board``），
        66 MB 的跑后三机位搬到归档段（整轮结束、令牌窗口之外）再拉
        （2026-11-12 用户定调：本机不承担运行时任务）。``--analysis local``
        是离线排练，没有板端分析，所以照旧在这里拉。

        **拉不到就记缺，不掩盖真正的退出码**：执行失败时往往正是中间某份
        jsonl / burst 不存在。若这里直接抛 ``EXIT_MISSING``，真因（执行失败）
        就被「缺证据」盖掉了。
        """
        os.makedirs(self.p("exec"), exist_ok=True)
        for p in plans:
            remote = self.b("exec", p["name"] + ".jsonl")
            local = self.p("exec", p["name"] + ".jsonl")
            try:
                self.board.pull_file(remote, local)
            except RoundError as e:
                log("  !! 执行日志拉不回：%s（%s）" % (p["name"], e))
        if self.args.analysis != "local":
            return
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

        # 跑后三机位真值（缺一个就缺，不拿默认值补；缺省在板上算）
        view_truth = []
        for v in cr.E9_VIEWS:
            out = self.p("truth", v["name"] + ".json")
            rc = self._truth(self.b("wrist-post", v["name"]),
                             self.p("post", "wrist-" + v["name"]), out,
                             allow_empty=True)
            if rc == 0 and os.path.exists(out):
                view_truth.append((v["name"], out))
                log("  跑后真值 %-5s：%s" % (v["name"], out))
            else:
                log("  !! 跑后真值 %-5s 失败（rc=%r）—— 记缺证据" % (v["name"], rc))

        # SAM 跑后 pad 真值：落格判据的主来源（2026-11-17 用户拍板）。
        # 彩色窗量降为回退（pad 真值没看见该格时）与交叉核对。
        pad_truth_blocks = []
        for _name, _path in view_truth:
            if _name == "pad":
                pad_truth_blocks = (read_json(_path).get("blocks") or [])
                break
        if not pad_truth_blocks:
            log("  !! 跑后 pad 真值里没有块 —— 落格回退到彩色窗量")

        # 逐目标落格位置（SAM pad 真值优先；回退彩色窗量）
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
            # 本目标的目标动作：S3 的 manifest 可能还带「被吸收的清障」动作
            # （它是别的块的目标动作）⇒ 按 block 选，不能数个数。
            tgt = [a for a in man["actions"] if a["kind"] == "target"
                   and str(a.get("block")) == str(tid)]
            if not tgt:
                tgt = [a for a in man["actions"] if a["kind"] == "target"]
            if len(tgt) != 1:
                raise RoundError(EXIT_SCORE, "score",
                                 "块 %s 的 actions 里目标动作不是 1 个" % tid)
            cmd_mm = tgt[0].get("commanded_center_mm")
            cell = int(man["cell"])
            xy_des = [v * 1000.0 for v in cm.cell_center(cell)]
            blk = [b for b in layouts if str(b["id"]) == tid][0]
            top_z = cm.PAD_THICK + float(blk["size"])
            cpath = os.path.join(colour_dir, "%s.json" % tid)
            sam = pad_truth_place_block(pad_truth_blocks, xy_des)
            if sam is not None:
                sb = sam["block"]
                got_color = str(sb.get("color") or "")
                colour = {
                    "ok": True,
                    "source": "sam-pad-truth",
                    "xy_mm": [round(float(sb["x"]) * 1000.0, 2),
                              round(float(sb["y"]) * 1000.0, 2)],
                    "hue_deg": bc.MEASURED_HUE_DEG.get(got_color),
                    "truth_color": got_color,
                    "expected_class": blk["color"],
                    "class_match": got_color == str(blk["color"]),
                    "n_frames": (sb.get("meas") or {}).get("n_frames"),
                    "n_missed": 0,
                    "nearest_mm": round(sam["dist_mm"], 2),
                    "search_mm": None,
                    "top_z": top_z,
                }
            else:
                # 回退：SAM pad 真值没看见该格附近的块 ⇒ 用旧的彩色窗量，
                # 并在文档里标来源（判分记录仍可追溯是哪条链出的数）。
                colour = self._colour(
                    self.b("wrist-post", "pad"), xy_des, top_z, blk["color"],
                    bc.MEASURED_HUE_DEG.get(blk["color"]), cpath)
                colour["source"] = colour.get("source") or "colour-window"
                colour["fallback_no_pad_truth"] = True
            colour["target"] = tid
            colour["cell"] = cell
            colour["desired_center_mm"] = [round(v, 3) for v in xy_des]
            colour["commanded_center_mm"] = cmd_mm
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
            if colour.get("ok"):
                log("  落格量 块 %-2s 格 %d：(%+.1f, %+.1f) mm  颜色 %s  匹配 %s"
                    "（来源 %s）"
                    % (tid, cell, colour["xy_mm"][0], colour["xy_mm"][1],
                       colour.get("truth_color")
                       or colour.get("measured_class") or "?",
                       colour.get("class_match"),
                       colour.get("source") or "?"))
            else:
                log("  落格量 块 %-2s 格 %d：!! 量不出（%d/%d 帧，来源 %s）"
                    % (tid, cell, colour.get("n_frames") or 0,
                       (colour.get("n_frames") or 0)
                       + (colour.get("n_missed") or 0),
                       colour.get("source") or "?"))

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
        if self.args.batch == "S3":
            dpath = self.p("truth", "designation.json")
            if os.path.exists(dpath):
                human = dict(human or {})
                human["designation"] = read_json(dpath)
        # 执行中令牌过期的机器证据（phase_execute 落盘 void.json）自动并入：
        # 契约 §4.6「本轮 void、不计 streak、不记抓取失败」。人填的同键文案
        # 优先，但键本身不会被人的漏填抹掉 —— 机器证据是事实，不是默认值。
        void_evidence = self.p("void.json")
        if os.path.exists(void_evidence):
            vd = read_json(void_evidence)
            why_text = {"aborted_token": "执行中令牌过期/不足（机器证据）",
                        "stop_requested": "人为软停（watch_round --stop，"
                                          "计划边界生效）"}.get(
                vd.get("reason"), str(vd.get("reason")))
            merged_voids = {tid: why_text for tid in ids}
            merged_voids.update(human.get("voids") or {})
            human = dict(human) if isinstance(human, dict) else {}
            human["voids"] = merged_voids
            log("  void.json 在（%s）：本轮 %d 个目标全记 void"
                % (vd.get("reason"), len(merged_voids)))
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

        # 手摆偏差（用跑前 pick 真值事后量；只归档，绝不改退出码）
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

    def _designate_commands(self):
        """``--designate``（可重复）与 ``--designate-file``（一行一条）合并。"""
        cmds = [str(x) for x in (getattr(self.args, "designate", None) or [])]
        path = getattr(self.args, "designate_file", None)
        if path:
            with open(path, encoding="utf-8") as f:
                for line in f:
                    s = line.strip()
                    if not s or s.startswith("#"):
                        continue
                    cmds.append(s)
        return cmds

    def _measured_truth(self, truth):
        """抓点用跑前 pick burst 的**实测块位姿**（用户 2026-11-12 定调）。

        为什么要它：手摆偏差实测 15–38 mm，按标称坐标抓 = 夹爪全闭的空夹
        （2026-11-12 S0:1 r1 第一块回读 0.0 mm）。拉不回 / 量不出就**停轮** ——
        抓点没有可信来源时不许发计划，更不许偷偷退回标称坐标。
        """
        if self.args.analysis == "local":
            self._ensure_pre_pick_burst()
        out = self.p("truth", "pick-measured.json")
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        t0 = float(self.deps["now"]())
        rc = self._truth(self.b("wrist-pre", "pick"),
                         self.p("pre", "wrist-pick"), out,
                         allow_empty=False)
        self.timing["pick_truth_s"] = round(float(self.deps["now"]()) - t0, 1)
        if rc != 0 or not os.path.exists(out):
            raise RoundError(EXIT_PLAN, "compute",
                             "跑前 pick burst 量不出块（rc=%r）—— 抓点没有实测"
                             "来源，停（先查分割/腕部相机，别退回标称坐标）"
                             % (rc,))
        meas = read_json(out)
        strict_layer = (self.args.batch == "S3")
        desig = None
        if strict_layer:
            try:
                meas["blocks"] = resolve_tower_colours(
                    meas.get("blocks") or [], truth.get("blocks") or [],
                    allow_fallback=bool(
                        getattr(self.args, "allow_side_colour_fallback",
                                False)))
            except ValueError as e:
                raise RoundError(EXIT_PLAN, "compute",
                                 "塔下层颜色裁决不过：%s" % e)
            try:
                desig = resolve_designation(
                    self._designate_commands(), meas.get("blocks") or [],
                    truth.get("blocks") or [],
                    self.args.pick_offset_limit_mm)
            except ValueError as e:
                raise RoundError(EXIT_PLAN, "compute",
                                 "结构化指定不过：%s" % e)
            write_json(self.p("truth", "tower-colours.json"),
                       {"blocks": meas.get("blocks") or []})
            write_json(self.p("truth", "designation.json"), desig)
            for x in desig["commands"]:
                log("  指定 %-18r → 块 %s（实测 %s，conf %s）"
                    % (x["raw"], x["manifest_id"], x["measured_id"],
                       x.get("confidence")))
        try:
            doc, recs = merge_measured_truth(truth, meas,
                                             self.args.pick_offset_limit_mm,
                                             strict_layer=strict_layer)
        except ValueError as e:
            raise RoundError(EXIT_PLAN, "compute",
                             "实测块位姿对不上布局表：%s" % e)
        write_json(self.p("truth", "measured-merge.json"), recs)
        for r in recs:
            log("  实测块 %s %-11s (%7.1f, %7.1f) mm  离标称 %5.1f mm  "
                "yaw 差 %+6.1f°"
                % (r["id"], r["color"], r["x_mm"], r["y_mm"], r["dist_mm"],
                   r["dyaw_deg"]))
        log("抓点：跑前 pick burst 实测（%d 块，最远离标称 %.1f mm）"
            % (len(recs), max(r["dist_mm"] for r in recs) if recs else 0.0))
        return doc, desig

    def _ensure_pre_pick_burst(self):
        """把跑前 pick burst 拉到本地（**只有 ``--analysis local`` 用**）。

        缺省的板上分析不需要图像在本机：真值在板上算、只回传 JSON。离线排练
        手里只有本地 burst / 要本地算，才在这里拉。
        """
        local = self.p("pre", "wrist-pick")
        if os.path.isdir(local) and any(
                f.startswith("color_") for f in os.listdir(local)):
            return
        try:
            self.board.pull_dir(self.b("wrist-pre", "pick"), local)
        except RoundError as e:
            raise RoundError(EXIT_PLAN, "compute",
                             "跑前 pick burst 拉不回：%s —— 抓点没有实测来源" % e)
        n = len([f for f in os.listdir(local) if f.startswith("color_")])
        if n == 0:
            raise RoundError(EXIT_PLAN, "compute",
                             "跑前 pick burst 一帧都没有（%s）" % local)

    def _write_placement_deviation(self):
        """跑后量一遍**手摆偏差**（只归档，缺证据就跳过，绝不改退出码）。

        正式轮是「近似手摆、跑前不实测」：块摆歪多少只有拿**跑前 pick 机位
        burst** 离线量一遍才知道。这里用与判分同一套真值工具，输出只进
        ``placement-deviation.json`` —— 不 void、不改判分（契约 §4.3）。
        """
        dev = self.deps.get("placement_deviation")
        if dev is None:
            return
        truth = self.p("pre", "truth-pick.json")
        # 计算段已经用**同一组 burst** 量过一遍（``_measured_truth`` → truth/
        # pick-measured.json）。这里再跑一次 20 帧共识纯属白花时间，直接复用。
        measured = self.p("truth", "pick-measured.json")
        if os.path.exists(measured):
            shutil.copyfile(measured, truth)
            rc = 0
        else:
            try:
                if self.args.analysis == "local":
                    self._ensure_pre_pick_burst()
                rc = int(self._truth(self.b("wrist-pre", "pick"),
                                     self.p("pre", "wrist-pick"), truth) or 0)
            except Exception as exc:                      # noqa: BLE001
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
    def _archive_images(self):
        """把板上 burst 拉回本地（**整轮结束之后**，不占令牌窗口）。

        判分已经在板上算完（只回传 JSON），图像只是留档/复盘用：拉失败只告警
        —— 本轮判分已经落盘，不能因为归档失败把它判成不完整。板上原物仍在
        ``<board_dir>/wrist-*``，随时可补拉。
        """
        if self.args.analysis == "local":
            return
        pairs = [(self.b("wrist-pre", v["name"]),
                  self.p("pre", "wrist-" + v["name"])) for v in PRE_VIEWS]
        pairs += [(self.b("wrist-post", v["name"]),
                   self.p("post", "wrist-" + v["name"])) for v in cr.E9_VIEWS]
        rows = []
        for remote, local in pairs:
            try:
                self.board.pull_dir(remote, local)
                n = len([f for f in os.listdir(local)
                         if f.startswith("color_")])
                rows.append({"remote": remote, "local": local,
                             "frames": n, "ok": n > 0})
                log("  归档 burst %-28s %d 帧" % (os.path.basename(local), n))
            except (RoundError, OSError) as e:
                rows.append({"remote": remote, "local": local, "ok": False,
                             "why": str(e)[:200]})
                log("  !! 归档 burst %s 失败：%s（板上仍在，可补拉）"
                    % (os.path.basename(local), e))
        write_json(self.p("archive.json"), {"bursts": rows})
        if not all(r.get("ok") for r in rows):
            log("  !! 部分 burst 未归档（只影响复盘，不影响本轮判分）")

    def phase_archive(self):
        write_json(self.p("phases.json"), self.phases)
        write_json(self.p("round-done.json"),
                   {"round_id": self.round_id,
                    "batch": self.args.batch, "layout": int(self.args.layout),
                    "run": int(self.args.run), "at": self.deps["now"](),
                    "out_dir": self.out, "board_dir": self.board_dir,
                    "analysis": self.args.analysis,
                    "phases": [r["phase"] for r in self.phases]})
        self._archive_images()
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
    ap.add_argument("--where", choices=("dev", "board"), default="dev",
                    help="编排器在哪跑：dev = 本机经 SSH 驱动板子（过渡态）；"
                         "board = 就在板上跑（B3 目标态：本机断网不影响令牌"
                         "窗口内的任何一步）。board 时 --board 别名不需要")
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
                         "precision_md5 / wrist_usb3 / stack）；不给 ⇒ 预检判红，"
                         "缺 precision_md5（旧基线）也判红")
    ap.add_argument("--facts", default=None,
                    help="人给的安全 / 干预 / 人眼复核 JSON（缺 ⇒ 判分不完整）")
    ap.add_argument("--pad-spec", default=None,
                    help="垫参数 JSON（footprint_m + 可选 thickness_m）")
    ap.add_argument("--pad-thick", type=float, default=None)
    ap.add_argument("--analysis", default="board", choices=("board", "local"),
                    help="分析段（解垫 / 真值 / 彩色）在哪算：board = 板上算、"
                         "只回传 KB 级 JSON（缺省；本机不承担运行时任务，图像"
                         "整轮后才归档）；local = 本机算，**只给离线排练**，"
                         "必须配 --dry-run")
    ap.add_argument("--pad-mat", default=pfb.DEFAULT_MAT,
                    help="2D 平面标定 JSON（解垫用；板上分析时随轮推上去，"
                         "KB 级、md5 核对）")
    ap.add_argument("--place-clearance", type=float, default=None)
    ap.add_argument("--place-yaw-equiv180", action="store_true")
    ap.add_argument("--allow-no-validator", action="store_true",
                    help="开发机上无碰撞裁决规划真机轨迹的显式豁免（干跑不需要；"
                         "正式轮要么 --where board，要么带这面旗 + 响亮告警"
                         "+ 记 manifest）")
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
    # 估时依据：**2026-11-15 attempt11 首个干净全链轮**（B3-lite 板上规划）：
    # compute_s = 94.9 s = 解垫 2.4 + 跑前 pick 真值 5.8 + 探针 6.6 + 规划 80.0。
    # 旧缺省 100 来自开发机规划时代（解垫 2.0 + 探针 28.1 + 规划 9.1 ≈ 47.7 s 的
    # ~90 ×1.1）；attempt8 的 131.8 s 含块 4 规划失败的分支扫描，不是干净值。
    # 取 95（向上取整 0.1 s）——估大了闸门更严，不危险。
    ap.add_argument("--compute-est", type=float, default=95.0,
                    help="R1 用的轮内计算估时（秒；缺省 95 = attempt11 干净实测 "
                         "94.9，板上解垫/真值/探针/规划四段合计）")
    ap.add_argument("--exec-est", type=float, default=400.0,
                    help="无 timing-prior 时的执行估时（秒；缺省 400 = 实测 run.sh "
                         "~390 s）")

    ap.add_argument("--truth-source", default="measured",
                    choices=("measured", "manifest"),
                    help="抓点位姿来源：measured = 跑前 pick burst 实测（缺省，"
                         "2026-11-12 用户定调）；manifest = 布局表标称坐标（只给"
                         "离线排练/没有 burst 的场合，真机正式轮禁用）")
    ap.add_argument("--pick-offset-limit-mm", type=float,
                    default=PICK_OFFSET_LIMIT_MM,
                    help="实测块心离布局表超过它就不抓（不是判分容差，是"
                         "「布局表与台面是不是同一场景」的兜底；缺省 %.0f）"
                         % PICK_OFFSET_LIMIT_MM)
    ap.add_argument("--allow-side-colour-fallback", action="store_true",
                    help="S3 异色塔下层侧色不置信时，允许按布局清单颜色填充"
                         "（默认 fail-closed；打开后报告/真值会标 fallback=true，"
                         "身份不是实测）")
    ap.add_argument("--clear-dest", choices=("cell", "free"), default="cell",
                    help="S3 清障落点：cell（默认）= 被清块进自己的色格，若它"
                         "本身是后面的目标则同时算它的目标抓取（每块只抓一次）；"
                         "free = 落自由区，之后还要再抓一次")
    ap.add_argument("--designate", action="append", default=[],
                    metavar="CMD",
                    help="S3 结构化指定命令（可重复；与 --designate-file 合并）。"
                         "例：\"red bottom\"。顺序即执行顺序")
    ap.add_argument("--designate-file", default=None, metavar="FILE",
                    help="S3 结构化指定：一行一条命令（空行 / # 注释忽略）；"
                         "板端路径由 launch_round 推送")
    ap.add_argument("--seg", choices=("sam", "grid"), default="sam",
                    help="感知链（第 345 轮 SAM 立项、347 轮回放达标后缺省 "
                         "sam；grid=网格链回退/对照）。SAM 权重在板上 "
                         "~/robotics/third_party/weights/mobile_sam.pt")
    ap.add_argument("--pad-min-frames", type=int, default=10,
                    help="解垫 burst 最少帧数（2026-11-15 v0.8 起 burst 20→10，"
                         "缺省随之 20→10；共识下限 5）")
    ap.add_argument("--pad-max-spread-mm", type=float, default=2.0)
    ap.add_argument("--pad-max-spread-deg", type=float, default=0.5)
    ap.add_argument("--colour-search-mm", type=float, default=25.0)
    ap.add_argument("--auth-timeout", type=float, default=1800.0)
    ap.add_argument("--auth-poll", type=float, default=2.0)
    ap.add_argument("--auth-fresh-min", type=float, default=840.0,
                    help="令牌剩余秒数下限：低于它就算「不是这一次签发的」，"
                         "继续等新的 STOP→PLAY（900 s TTL 留 60 s 余量，"
                         "2026-11-15 v0.8；缺省挡住沿用上一次授权，红线 6）")

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
    if getattr(args, "where", "dev") == "dev" and not args.board:
        ap.error("要给 --board（SSH 别名）")
    if not args.board_repo:
        ap.error("要给 --board-repo（板上本仓检出路径）")
    if not args.urdf:
        ap.error("要给 --urdf")
    if args.bias_estimate is None:
        args.bias_estimate = os.path.join(
            os.path.dirname(os.path.abspath(args.out_dir)),
            gb.BIAS_ESTIMATE_FILE)
    if args.cells is None and args.batch != "S3":
        args.cells = ",".join(str(c) for c in
                              cm.parse_cells(None, None, batch=args.batch))
    if args.batch == "S3" and args.cells:
        ap.error("S3 的格位由颜色决定（垫纸 grid_order + 重复色进备用格），"
                 "不接受 --cells；去掉它即可")
    if args.batch not in lay.BATCHES:
        ap.error("未知批次 %r（可选 %s）" % (args.batch, sorted(lay.BATCHES)))
    if int(args.layout) not in lay.BATCHES[args.batch]:
        ap.error("%s 没有选中的布局 %r（选中 %s）"
                 % (args.batch, args.layout,
                    sorted(lay.BATCHES[args.batch])))
    if args.pad_spec and not os.path.exists(args.pad_spec):
        ap.error("--pad-spec 不存在：%s" % args.pad_spec)
    if args.analysis == "local" and not args.dry_run:
        ap.error("--analysis local 只给离线排练：正式轮的分析必须在板上算"
                 "（本机不承担运行时任务）。要真跑就删掉这个旗标")
    if args.pad_mat and not os.path.exists(os.path.expanduser(args.pad_mat)):
        ap.error("--pad-mat 不存在：%s" % args.pad_mat)
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
    board_dir = args.board_dir or default_board_dir(args.out_dir)
    if getattr(args, "where", "dev") == "board":
        # B3 目标态：编排器就在板上跑 —— 同机适配器，SSH 只剩「发起/观察」。
        board = LocalBoard(board_dir,
                           os.path.dirname(os.path.abspath(args.out_dir)))
    else:
        board = RoundBoard(args.board, board_dir,
                           os.path.dirname(os.path.abspath(args.out_dir)))
    return Round(args, board).run()


if __name__ == "__main__":
    sys.exit(main())
