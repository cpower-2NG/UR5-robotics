#!/usr/bin/env python3
"""E9 真机半段：把探针的「清障方案」变成一份**按顺序可执行**的抓放计划序列。

本模块**纯离线**：只读探针报告 + 场景真值，产出计划 JSON 与执行脚本；
不 import ROS、不发轨迹、不写夹爪寄存器。真执行是板上
``precision_exec.py --go`` 的事（每条动作都要当次运动令牌）。

分工（为什么这样切）
--------------------
``stage4_clear_probe`` 回答「**存不存在**一个合法方案」：目标可抓 ∧ 清障 ∧
放置位姿合法 ∧ 终态合法，夹爪间隙用自有 OBB+SAT 逐个候审判。它不产生轨迹。
本模块回答「**怎么把那个方案做出来**」：把每一步清障与最终目标翻译成
``precision_plan.build_pick`` 的 A3 抓放几何（竖直抓、18.5 mm 夹持深度、
整链选支、高度护栏），``q_start`` 逐段衔接。

一个必须说清的例外
------------------
``build_pick`` 有一条粗判据「被抓块离最近邻块中心距 ≥60 mm」。E9 的目标
恰恰是**贴面/邻块**场景：探针已经用 OBB 判过「这个候选位姿夹爪不碰」，
它比中心距细（贴面块只要在合拢轴方向让开就能抓）。所以探针判过的调用方显式给
``min_block_gap=0.0`` 并在 manifest 里写 ``gap_check: probe_verified``；
**没探针判决的（S0 这种无接触布局）走默认 60 mm 粗判据，manifest 记
``gap_check: layout_gap``** —— 不许无判决地放宽。

阶段 4 正式轮（2026-11-11 起）的机位分两段：**跑前**只拍 pick / pad 两个机位
（解垫位姿、量目标块、看拾取区；S0 无清障，不需要 free），**跑后**三个机位
（``E9_VIEWS``：拾取区 / 北侧自由区 / 垫面）齐全。全部走腕部几何链，俯视不参与
任何绝对坐标（2026-11-09 用户拍板：规划与判分都用腕部相机）。跑后核验
（``--view-truth``）读这三份机位真值合并成一份终态。
目标块「落在指定格」用**彩色顶面质心**量 —— 垫面在深度里分辨不出 29.5 mm 台阶
（2026-11-08 实测，见 ``measure_target_colour``）；其余判据仍用深度真值。
仍不 import ROS。

指令顺序（每条都在 manifest 里可见）
------------------------------------
1. 先按探针给的顺序执行清障步骤（``clearing_steps``：谁 → 从哪 → 到哪）；
2. 最后执行目标块 → 指定宫格；
3. 每步内部固定：approach → close → lift → to_slot → place_approach →
   place(open) → retreat → survey（``build_pick`` 的老航点序列，不另立）。

用法（开发机，链式 q_start）
---------------------------
    python3 scripts/stage4_clear_run.py \\
        --probe-report runs/<run>/e9/<scene>-probe.json \\
        --urdf ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf \\
        --from-frame <joint_states.json 所在目录> \\
        --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<run>/e9-plan/<scene>

退出码：0 = 全部计划写出且预检通过；1 = 某一步规划失败（不写半份序列）；
2 = 输入缺失/不可用。
"""
import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

import graspnet_candidates as gc                                  # noqa: E402
import precision_plan as pp                                       # noqa: E402
import stage4_clear_model as cm                                   # noqa: E402
import stage4_specify as sp                                       # noqa: E402
import stage4_truth_from_frame as tf                              # noqa: E402
import urkin                                                      # noqa: E402

#: 收尾巡检位（base 系，m）。用阶段 4 工作区上方的空位，不用阶段 2 的
#: ``SURVEY_TOOL0["place"]``（那在 y≈−0.39 m，每步白跑一趟）。
#: tool0 z=0.40 ⇒ 指尖 0.196 m，高过任何 29.5 mm 块；x/y 在拾取区上方。
E9_SURVEY_TOOL0 = (0.520, 0.300, 0.400)

#: E4b 低悬停高度（tool0，m）。取 360：俯视在同一帧里能同时看清**块顶面**与
#: **两根指**，且指在块上方有 30–40 px 的干净段（2026-11-08 B1 实测）。低于
#: 300 指尖进画面下缘，高于 420 指与块的像素都缩得太小。
ALIGN_HOVER_Z = 0.360

#: 单次修正量的上限（mm）。E4b 的判据是 ≤2 mm 残差、≤2 次修正；一次要求挪
#: 50 mm 以上的只能是量错了（选错块/外参被碰），不是没对准 ⇒ 拒绝应用。
ALIGN_MAX_CORR_MM = 50.0

#: E4b 允许的修正轮数（判据「≤2 次修正」）。第 3 轮还想改 = 没收敛，停下来查
#: 量具是在漂还是块被碰了，**不许**接着掰。
MAX_ALIGN_ROUNDS = 2

#: 收敛判据。与 ``stage4_align`` 的 TOL_MM/TOL_DEG 同源：残差已经在这个量级
#: 以内就不再发一次运动（量具自己也分辨不到更好）。
ALIGN_TOL_MM = 2.0
ALIGN_TOL_DEG = 2.0

#: 跑前/跑后**判分机位**（base 系，m）：一个量具、三个机位，全部由**腕部相机**拍。
#:
#: 为什么是三个（2026-11-09 用户拍板「规划与判分都走腕部，俯视不给绝对坐标」）：
#: 一个 500 mm 高的俯视视场只有约 0.70×0.40 m，装不下全场；三个区**互不重叠**
#: 且合起来覆盖所有会放块的地方 ——
#:
#: · ``pick`` 拾取区（塔、块数/塔层、旁观块）—— 既有 500 mm 巡检位，已验证；
#: · ``free`` 北侧自由区（清障块的落点，``FREE_X``/``FREE_Y``）—— 在 pick 视场外
#:   （pick 只到 y≈0.49），不给它机位就**看不见被搬走的块**，判分直接缺块；
#: · ``pad`` 垫面（目标块落格）—— 在 pick 视场外（垫在 y≤0），而且垫面在深度里
#:   分不出 29.5 mm 台阶，落格要用彩色顶面质心。
#:
#: 位姿由 ``stage4_wrist_survey.survey_plan`` 从 zone+height **现算**，跑前跑后
#: 各调一次这个常量 ⇒ 逐位一致，不靠人手抄坐标。改这里 = 改判分机位，须记录。
#:
#: ``yaw_deg``（tool0 绕工具轴的角，缺省 0）：**只有 pad 用 180°**。这不是画幅
#: 需要（那个区 0/90/180/270 都在画幅里），而是**可达性**：free 机位的 IK 解在
#: wrist_2 上有 ±90° 两支（互为 180° 翻腕），pad 机位在 yaw 0 时的合格解全部落在
#: 翻腕的另一侧，接不上（2026-11-12 attempt6：块 4 收尾与三机位差整整 170–180°）。
#: yaw 180° 与 0° 是同一个"竖直朝下"姿态绕工具轴转半圈，pad 区照样满画幅
#: （实测四边余量 330/331/54/47 px，与 yaw 0 逐像素一致），而 IK 支换到能接上的
#: 那一侧。跑前/跑后都走同一份常量 ⇒ 两帧仍同一位姿。
#:
#: 次序即执行次序（跑前只用 pick / pad 两项，见编排器 PRE_VIEWS）。
#: ``pad`` 排在 ``free`` 前面不是随手：自由区机位在臂展边上（tool0 y≈0.455、
#: z≈0.474），它的 IK 只有 j0≈−131° 与 +31° 两族、腕 2 只能用 −269.8° 那支；
#: 从拾取区机位（j0≈−142°、腕 2 −90.2°）直接过去，最小单步 179.6°，正好卡在
#: 170° 上限外。垫面机位有 j0≈−23°、腕 2 +89.8° 的解，先绕它一脚，两端分别
#: 差 119° 与 0.4°（2026-11-12 attempt6 实测）。
E9_VIEWS = (
    {"name": "pick", "zone": tuple(cm.PICK_ZONE), "height": 0.500,
     "purpose": "块数/塔层/旁观块（拾取区）"},
    {"name": "pad", "zone": (0.310, 0.610, -0.300, 0.000), "height": 0.500,
     "yaw_deg": 180.0,
     "purpose": "目标块落格（垫面，彩色顶面质心）"},
    {"name": "free", "zone": (cm.FREE_X[0], cm.FREE_X[1],
                              cm.FREE_Y[0], cm.FREE_Y[1]), "height": 0.450,
     "purpose": "清障块落点（北侧自由区）"},
)

#: 机位之间的悬停抬高（m）。**不能用 0.10**：free 机位在 y≈0.455、z≈0.474，
#: 再抬 0.10 到 z≈0.574 就出了 UR5 的臂展（2026-11-09 实测：IK 位置残差
#: 11.4 mm，从 s7 与 home 两个起点都复现）。0.06 两个起点都过，0.04 会在
#: home 起点上逼出 236° 的整圈翻转（``unwrap`` 的 180° 上限）。改它要重测。
VIEW_TRANSIT_DZ = 0.06

#: 每组腕部 burst 的帧数（2026-11-15 契约 v0.8 由 20 降到 10，用户拍板）：
#: 共识真值的下限是 5 帧（``--min-frames``），10 帧仍有 2× 余量；直接省一半
#: 跨机搬运与抓帧时间（第 321 轮实测 20 帧 = 44 MB / 2×20 帧，60–95 s/轮）。
#: 改它要同步 ``run_script_text`` 的回显文案与 ``exec_estimate`` 的 burst 开销。
BURST_FRAMES = 10


def log(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------- 输入解析
def load_scene(report, scene_index, truth_override=None):
    scenes = report.get("scenes") or []
    if not scenes:
        raise ValueError("探针报告里没有 scenes")
    if not (0 <= scene_index < len(scenes)):
        raise ValueError("--scene-index 越界：%d（共 %d 场）"
                         % (scene_index, len(scenes)))
    sc = scenes[scene_index]
    truth_path = truth_override or sc["truth"]
    if truth_override:
        sc = dict(sc, truth=truth_override)
    with open(truth_path, encoding="utf-8") as f:
        truth = json.load(f)
    return sc, truth


def pick_attempt(scene, target=None):
    attempts = scene.get("attempts") or []
    ok = [a for a in attempts if a.get("ok")]
    if target is not None:
        for a in ok:
            if str(a.get("id")) == str(target):
                return a
        raise ValueError("目标 %r 在报告里没有成功的尝试" % (target,))
    if not ok:
        raise ValueError("这一场没有成功的尝试（ok=True）")
    return ok[0]


# ---------------------------------------------------------------- 几何
def grasp_yaw_deg(hit, block_yaw):
    """探针命中的抓取候选 → ``build_pick`` 的 tool0 绕 z 角。

    ``grasp_candidates`` 的约定：tool0 z = 接近方向 a，tool0 x = 合拢轴 c。
    B 档竖直抓只可能是 a=(0,0,−1)（axis=2, sign=−1）。``closing`` 是块局部轴：
    0 → c = 块 x 轴 ⇒ ψ = 块 yaw；1 → c = 块 y 轴 ⇒ ψ = 块 yaw + 90°。
    ``variant`` 是绕 a 转 180°，对方块在几何上等价（``fold_yaw_180`` 会折掉）。
    """
    if not hit:
        raise ValueError("没有抓取命中记录")
    if not hit.get("vertical") or int(hit.get("axis", -1)) != 2 \
            or int(hit.get("sign", 0)) != -1:
        raise ValueError("E9 只执行 B 档竖直抓，收到 axis=%r sign=%r vertical=%r"
                         % (hit.get("axis"), hit.get("sign"), hit.get("vertical")))
    closing = int(hit.get("closing", 0))
    if closing not in (0, 1):
        raise ValueError("closing 只认 0/1，收到 %r" % (closing,))
    return float(block_yaw) + (0.0 if closing == 0 else 90.0)


def find_block(blocks, bid):
    for b in blocks:
        if str(b.get("id")) == str(bid):
            return b
    return None


def move_in_layout(blocks, bid, x, y, z_bottom):
    """就地更新布局里的一块（不改原 truth）。``top`` 是块顶离台面高度。"""
    b = find_block(blocks, bid)
    b["x"], b["y"] = float(x), float(y)
    b["z"] = float(z_bottom) + float(b["size"]) / 2.0
    b["top"] = float(z_bottom) + float(b["size"])
    return b


def surface_of(to_z, size, where, pad_thick=None):
    """落点块心 z → 支撑面高度；只认台面与**本轮实测**垫面（多认就是把错当真）。

    ``pad_thick`` 缺省 = ``cm.PAD_THICK``（真机实测常量）；带 ``--pad-thick``
    的轮必须把它传进来，否则计划与执行对垫面的理解会各说各话。
    """
    z = float(to_z) - float(size) / 2.0
    pad = float(cm.PAD_THICK if pad_thick is None else pad_thick)
    for cand in (0.0, pad):
        if abs(z - cand) <= 1e-4:
            return cand
    raise ValueError("%s 的落点块心 z=%.4f 推不出支撑面（得到 %.4f m，"
                     "既不是台面也不是垫面）" % (where, float(to_z), z))


def _bias_xy_mm(bias_mm):
    """``bias_mm`` → ``(bx, by)`` 毫米；``None`` ⇒ ``(0, 0)``（不补）。"""
    if bias_mm is None:
        return 0.0, 0.0
    return float(bias_mm[0]), float(bias_mm[1])


def _target_xy_action(kind, tid, from_xy, from_top, desired_xy_m,
                      surface_z, grasp_yaw, grasp_hit, bias_mm):
    """目标动作的落点三坐标：**期望**（格心）/ **命令**（期望 − b̂）/ 所用 b̂。"""
    bx, by = _bias_xy_mm(bias_mm)
    desired_mm = [float(desired_xy_m[0]) * 1000.0,
                  float(desired_xy_m[1]) * 1000.0]
    commanded_mm = [desired_mm[0] - bx, desired_mm[1] - by]
    return {
        "kind": kind,
        "block": str(tid),
        "from_xy": [float(from_xy[0]), float(from_xy[1])],
        "from_top": float(from_top),
        "slot": [commanded_mm[0] / 1000.0, commanded_mm[1] / 1000.0],
        "surface_z": float(surface_z),
        "grasp_yaw_deg": grasp_yaw,
        "grasp_hit": grasp_hit,
        # 三份坐标分开记（阶段三 planned.desired/commanded/applied_bias 同构）：
        # 判分永远比 desired，臂收到的是 commanded，差出来的就是 b̂。
        "desired_center_mm": [round(v, 3) for v in desired_mm],
        "commanded_center_mm": [round(v, 3) for v in commanded_mm],
        "applied_bias_mm": ([round(bx, 3), round(by, 3)]
                            if bias_mm is not None else None),
    }


def build_target_action(block, grasp_hit, cell, pad_thick, bias_mm=None):
    """按**实测宫格**现算目标动作（不依赖探针的 ``final_blocks``）。

    这是「探针前置」后备路径的入口：探针在授权前按旧布局跑（只读、不动臂），
    授权后解出新垫，就用本函数以 ``cm.cell_center(cell)`` 重算落点 —— 探针的
    place 合法性只说明「那个位姿在旧垫上合法」，落点必须按新垫给。
    默认路径仍走 ``actions_from_attempt``（探针结果 + 实测格心断言）。

    ``bias_mm`` 是 b̂（毫米）；命令落点 = 格心 − b̂，期望仍是格心。
    """
    cx, cy = cm.cell_center(int(cell))
    return _target_xy_action(
        "target", block["id"], [block["x"], block["y"]], block["top"],
        [cx, cy], float(pad_thick),
        grasp_yaw_deg(grasp_hit, block.get("yaw", 0.0)),
        {k: (grasp_hit or {}).get(k) for k in
         ("axis", "sign", "closing", "variant", "verdict")},
        bias_mm)


def actions_from_attempt(truth, attempt, pad_thick, bias_mm=None):
    """探针的一次成功尝试 → 有序动作表（清障在前，目标在最后）。

    返回的 ``blocks`` 是**原始布局**（不预先搬运）：搬运由
    ``plan_sequence`` 在**算完那一步之后**才施加 —— 否则清障那一步会按
    「块已经躺在落点」去算，臂会跑去错的地方。
    起点用探针记录的 ``step["from"]``（那一刻的实测位姿），不用布局推算。

    ``attempt["cell"]``（``--cells`` 逐目标格位）给了就**钉住**：探针终态块心
    必须落在 ``cell_center(cell)`` 1 mm 内、yaw 与真值一致，否则抛错 ——
    否则探针可能读的是模型垫、计划按实测垫放，两边各自自洽却差 87 mm。
    ``bias_mm``（b̂，毫米）：目标格心的**命令**落点 = 格心 − b̂（阶段三 §4.1）。
    """
    blocks = [dict(b) for b in truth["blocks"]]
    actions = []
    for step in attempt.get("clearing_steps") or []:
        bid = step["move"]
        blk = find_block(blocks, bid)
        if blk is None:
            raise ValueError("清障步骤引用了不存在的块 %r" % (bid,))
        src, to = step["from"], step["to"]
        surf = surface_of(to[2], blk["size"], "清障块 %r" % (bid,),
                          pad_thick=pad_thick)
        actions.append({
            "kind": "clear",
            "block": str(bid),
            "from_xy": [float(src[0]), float(src[1])],
            "from_top": float(src[2]) + float(blk["size"]) / 2.0,
            "slot": [float(to[0]), float(to[1])],
            "surface_z": surf,
            "grasp_yaw_deg": grasp_yaw_deg(step.get("grasp"),
                                           blk.get("yaw", 0.0)),
            "grasp_hit": {k: step["grasp"].get(k) for k in
                          ("axis", "sign", "closing", "variant", "verdict")}
            if step.get("grasp") else None,
        })

    tid = str(attempt["id"])
    tgt = find_block(blocks, tid)
    if tgt is None:
        raise ValueError("真值里没有目标块 %r" % (tid,))
    final = find_block(attempt.get("final_blocks") or [], tid)
    if final is None:
        raise ValueError("尝试记录里没有目标块 %r 的终态位姿" % (tid,))
    surf = surface_of(final["z"], tgt["size"], "目标块 %r" % (tid,),
                      pad_thick=pad_thick)
    if abs(surf - float(pad_thick)) > 1e-4:
        raise ValueError("目标块 %r 的落点支撑面 %.4f m 不是垫面 %.4f m —— "
                         "探针与执行器对放置面的理解不一致"
                         % (tid, surf, float(pad_thick)))
    cell = attempt.get("cell")
    if cell is not None:
        cx, cy = cm.cell_center(int(cell))
        d = math.hypot(float(final["x"]) - cx, float(final["y"]) - cy)
        if d > 1e-3:
            raise ValueError(
                "目标块 %r 的探针终态 (%.1f, %.1f) mm 离实测格 %d 的格心 "
                "(%.1f, %.1f) mm 有 %.2f mm（上限 1）—— 探针与计划不是同一份"
                "垫位姿，停下来查，不要套格"
                % (tid, final["x"] * 1000, final["y"] * 1000, int(cell),
                   cx * 1000, cy * 1000, d * 1000.0))
        dyaw = abs(float(final.get("yaw", 0.0)) - float(tgt.get("yaw", 0.0)))
        if dyaw > 1e-6:
            raise ValueError("目标块 %r 的探针终态 yaw %.3f° ≠ 真值 %.3f°"
                             % (tid, final.get("yaw", 0.0), tgt.get("yaw", 0.0)))
        desired = [cx, cy]
    else:
        desired = [float(final["x"]), float(final["y"])]
    actions.append(_target_xy_action(
        "target", tid, [tgt["x"], tgt["y"]], tgt["top"], desired, surf,
        grasp_yaw_deg(attempt.get("grasp_hit"), tgt.get("yaw", 0.0)),
        {k: (attempt.get("grasp_hit") or {}).get(k) for k in
         ("axis", "sign", "closing", "variant", "verdict")},
        bias_mm))
    return actions, blocks


# ---------------------------------------------------------------- E4b 对中
def apply_align(truth, actions, align, target, max_corr_mm=ALIGN_MAX_CORR_MM):
    """把 E4b 的 ``(dx, dy, dyaw)`` 折进目标块。返回写进 manifest 的记录。

    ``(dx, dy)`` 的定义见 ``stage4_align.py``：**块相对夹爪轴**的位移，所以
    修正就是把目标点加上它。只改目标块，清障块不动（它们的位姿由探针定，
    与俯视的相对量无关）。

    fail-closed 三条：``ok=false`` / 共识帧 <3 / 单次修正 >``max_corr_mm``
    ⇒ 抛 ``ValueError``，不应用。量错了比没对上更危险 —— 那会把臂送去抓
    旁边的块。
    """
    if not align.get("ok"):
        raise ValueError("对中结果 ok=false（%s）" % (align.get("reason") or "?"))
    cons = align.get("consensus") or {}
    if int(cons.get("n_used") or 0) < 3:
        raise ValueError("对中只用上 %s 帧（下限 3）—— 这不是共识"
                         % cons.get("n_used"))
    dx, dy = float(cons["dx_mm"]), float(cons["dy_mm"])
    dyaw = float(cons.get("dyaw_deg") or 0.0)
    mag = math.hypot(dx, dy)
    if mag > max_corr_mm:
        raise ValueError("修正量 %.1f mm，单次上限 %.0f mm —— 先查量具"
                         "（选块/外参），不要硬掰" % (mag, max_corr_mm))
    if mag <= ALIGN_TOL_MM and abs(dyaw) <= ALIGN_TOL_DEG:
        # 已经对准了：残差在量具自身分辨率以内，再发一次运动只是白跑。
        return {"applied": False, "reason": "already_in_tol",
                "target": str(target),
                "dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
                "dyaw_deg": round(dyaw, 3), "e_before_mm": round(mag, 2),
                "n_frames": int(cons["n_used"]),
                "spread_mm": cons.get("spread_mm"),
                "source": align.get("burst")}
    tid = str(target)
    blk = find_block(truth["blocks"], tid)
    if blk is None:
        raise ValueError("真值里没有目标块 %r" % (tid,))
    act = next((a for a in actions
                if a["kind"] == "target" and str(a["block"]) == tid), None)
    if act is None:
        raise ValueError("动作表里没有目标块 %r" % (tid,))
    before = [round(float(blk["x"]) * 1000.0, 1),
              round(float(blk["y"]) * 1000.0, 1)]
    blk["x"] = float(blk["x"]) + dx / 1000.0
    blk["y"] = float(blk["y"]) + dy / 1000.0
    blk["yaw"] = float(blk.get("yaw", 0.0)) + dyaw
    act["from_xy"] = [act["from_xy"][0] + dx / 1000.0,
                      act["from_xy"][1] + dy / 1000.0]
    act["grasp_yaw_deg"] = float(act["grasp_yaw_deg"]) + dyaw
    return {"applied": True, "target": tid,
            "dx_mm": round(dx, 2), "dy_mm": round(dy, 2),
            "dyaw_deg": round(dyaw, 3),
            "e_before_mm": round(mag, 2),
            "n_frames": int(cons["n_used"]),
            "spread_mm": cons.get("spread_mm"),
            "source": align.get("burst"),
            "block_xy_before_mm": before,
            "block_xy_after_mm": [round(float(blk["x"]) * 1000.0, 1),
                                  round(float(blk["y"]) * 1000.0, 1)]}


def apply_align_jsons(truth, actions, paths, target):
    """按顺序折进多轮 ``(dx, dy, dyaw)``。返回每轮记录（manifest 用）。

    轮数是**累计**的：第 2 轮量的是「悬停在已修正位置上」的残差，所以两次
    增量相加才对得上真值。超过 ``MAX_ALIGN_ROUNDS`` 直接拒绝 —— 那不是还没
    对准，是量具/场景出了问题。
    """
    paths = list(paths or [])
    if len(paths) > MAX_ALIGN_ROUNDS:
        raise ValueError("对中修正给了 %d 轮，上限 %d —— 先查为什么没收敛"
                         % (len(paths), MAX_ALIGN_ROUNDS))
    recs = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            align = json.load(f)
        recs.append(apply_align(truth, actions, align, target))
    return recs


def topcam_view_gate(burst, xyz, yaw_deg, min_deg=None):
    """出计划时先判「这个抓取朝向挡不挡俯视视线」。返回检查记录。

    E4b 的低悬停要求俯视在**同一帧**里同时看清块顶与两指；两指前后一条线时
    近侧手指的影子直接压在块顶上（2026-11-10 s11 实拍：ψ=88.75°，两指尖只差
    ~20 px，而 95 mm 开口该有 ~124 px）。所以这也是**计划阶段**的闸门，不是等
    量不出数才发现：偏好的合拢轴没被探针选中时（``hit.closing`` 回落）在这里就红。

    ``burst`` 只用来取俯视外参（``tf_*.json``，静态链，不需要深度/彩色）。
    相机近乎垂直向下时返回 ``lateral=None``（没有水平视线可言），不拦。
    """
    import stage4_align as sa
    if len(list(xyz)) != 3:
        raise ValueError("topcam_view_gate 要 (x, y, z) 三个数，收到 %r"
                         % (xyz,))
    try:
        R, _t = gc.cam_pose(burst, tip=gc.TOPCAM_TIP, chain=gc.TOPCAM_CHAIN)
    except FileNotFoundError:
        R, _t = gc.cam_pose(burst, tip=gc.TOPCAM_TIP, chain=gc.TOPCAM_CHAIN,
                            idx=0)
    T = pp.target_matrix([float(v) for v in xyz], float(yaw_deg))
    rec = sa.view_clearance_deg(T[:3, :3], R, **({} if min_deg is None
                                                 else {"min_deg": min_deg}))
    rec["min_deg"] = (sa.VIEW_LATERAL_MIN_DEG if min_deg is None
                      else float(min_deg))
    rec["burst"] = os.path.abspath(burst)
    rec["grasp_yaw_deg"] = round(float(yaw_deg), 3)
    return rec


def plan_align_hover(chain, q_start, action, out_dir, hover_z=ALIGN_HOVER_Z,
                     transit_dz=0.10, name="00-align-hover.json"):
    """E4b 低悬停计划：tool0 停在目标块正上方 ``hover_z``，姿态 = 计划抓取角。

    两个航点：先竖直抬到 ``hover_z+transit_dz``（跨过任何塔顶），再落到
    ``hover_z``。不带走位：这一段只负责「臂停住，让俯视拍同一帧」。
    张爪（``before.gripper=open``）与真正抓取前的状态一致 —— 量到的两指像素
    位置才对应抓取那一刻。**张爪要写寄存器，属当次授权范围。**
    """
    x, y = float(action["from_xy"][0]), float(action["from_xy"][1])
    yaw = float(action["grasp_yaw_deg"])
    q = [float(v) for v in q_start]
    wps = []
    for i, (nm, z) in enumerate((("align_transit", hover_z + transit_dz),
                                 ("align", hover_z))):
        T = pp.target_matrix([x, y, z], yaw)
        qn, err_mm, err_deg, _it, conv = chain.ik(T, q)
        if qn is None or not conv or err_mm > 2.0 or err_deg > 1.0:
            raise pp.PlanError(
                "对中悬停航点 %s IK 不过：%serr=%.2f mm / %.2f°"
                % (nm, "" if conv else "撞迭代上限（半解），",
                   0.0 if qn is None else err_mm,
                   0.0 if qn is None else err_deg))
        wp = {"name": nm, "q": [float(v) for v in qn],
              "duration": pp.duration_for(q, qn)}
        if i == 0:
            wp["before"] = {"gripper": "open"}
        wps.append(wp)
        q = [float(v) for v in qn]
    plan = {"version": 2,
            "meta": {"kind": "align_hover",
                     "block_idx": str(action["block"]),
                     "block_xy_mm": [round(x * 1000.0, 1),
                                     round(y * 1000.0, 1)],
                     "hover_z_mm": round(hover_z * 1000.0, 1),
                     "yaw_grasp_deg": round(yaw, 3),
                     "e9": {"kind": "align_hover", "block": action["block"],
                            "purpose": "E4b 低悬停：俯视同一帧量 (dx,dy,dyaw)"}},
            "q_start": [float(v) for v in q_start],
            "waypoints": wps}
    path = os.path.join(out_dir, name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(plan, f, ensure_ascii=False, indent=1)
    return {"index": 0, "name": name, "path": path, "kind": "align_hover",
            "block": action["block"], "slot": None,
            "from_xy": [x, y], "grasp_yaw_deg": yaw,
            "n_waypoints": len(wps)}, plan


def align_burst_needed(plans):
    """哪些计划后面要拍对中/偏心 burst —— 只回报，不改计划。"""
    return [p["name"] for p in plans if p.get("kind") == "align_hover"]


# ---------------------------------------------------------------- 计划
def to_pp_block(b):
    """真值块（m）→ precision_plan 的块字典（cx/cy/top 单位 mm）。"""
    size_mm = float(b["size"]) * 1000.0
    top_mm = float(b["top"]) * 1000.0
    return {"idx": int(b["id"]),
            "class": b.get("color") or b.get("class") or "block",
            "cx": float(b["x"]) * 1000.0,
            "cy": float(b["y"]) * 1000.0,
            "top": top_mm,
            "h": top_mm,
            "size": size_mm,
            "yaw": float(b.get("yaw", 0.0))}


def view_targets(views=None):
    """判分机位的**后续目标**：每个机位两点，直接就是 ``build_pick`` 的
    ``post_targets`` 格式 —— ``[(名字, xyz, yaw_deg)]``。

    与 ``plan_views`` / ``survey_plan`` 用的是同一个 ``ws.coverage`` —— 坐标只有
    一处出处。用途只有一个：交给末块的 ``pp.build_pick(post_targets=...)``，让
    收尾那支 IK 在「之后还得走得到这些点」的前提下选 —— 否则两条链可能分属
    不同的 ±360 折圈族，事后怎么折圈都差 360°（2026-11-12 attempt6）。
    """
    import stage4_wrist_survey as ws   # 惰性 import，同 plan_views
    out = []
    for v in (E9_VIEWS if views is None else views):
        yaw = float(v.get("yaw_deg", 0.0))
        _, tool0, _ = ws.coverage(tuple(v["zone"]), float(v["height"]), yaw)
        dz = float(v.get("transit_dz", VIEW_TRANSIT_DZ))
        out.append(("%s_transit" % v["name"],
                    (float(tool0[0]), float(tool0[1]),
                     float(tool0[2]) + dz), yaw))
        out.append(("%s_pose" % v["name"],
                    tuple(float(x) for x in tool0), yaw))
    return out


def plan_views(chain, q_start, views, out_dir, start=0, validator=None):
    """从 ``q_start`` 依次平动到每个判分机位，各写一份计划。

    **跑前（``--views-only``）与跑后（``plan_sequence`` 末尾）都走这一个函数**，
    所以两边的机位逐位一致 —— 「跑前跑后是同一台量具」就是这么落地的，不靠人
    手抄坐标。任何机位出画幅 / IK 不过 ⇒ ``pp.PlanError``，调用方整份不写
    （fail-closed：不许跑完才发现某个机位拍不了）。

    **整串机位由一个 DP 一次解出**（``pp.solve_path``），再按机位切片交给
    ``survey_plan(qs=...)``。以前每个机位各自局部 IK，跨机位时会各挑各的
    ±360 折圈支：末块收尾在 j0≈−139°、自由区机位只有 j0≈−131° 与 +31° 两族，
    逐点就近正好卡在 179.6°（单步上限 170°）和 212°（整圈翻转）之间 ——
    2026-11-12 attempt6 的 compute 就是这么失败两次的。

    返回 ``(plans, q_end)``。
    """
    import stage4_wrist_survey as ws   # 惰性 import：只在这条路径上拉 scipy
    targets = view_targets(views)
    try:
        qpath = pp.solve_path(
            chain, [(n, xyz) for n, xyz, _ in targets], q_start,
            yaws=[y for _, _, y in targets],
            # 机位链 = 高空平移段：单步帽用 TRANSIT（170°），不用作业段的 90°
            # —— attempt6 的机位链实测要 144.4° 单步（见 TRANSIT_STEP_CAP_DEG）
            max_step_deg=pp.TRANSIT_STEP_CAP_DEG,
            validator=validator)
    except pp.BranchChainError as e:
        raise pp.PlanError("判分机位链解不出来：%s" % e)
    plans, q = [], [float(v) for v in q_start]
    for i, v in enumerate(views, start=int(start) + 1):
        yaw = float(v.get("yaw_deg", 0.0))
        # 每个机位两点：transit 在前、pose 在后（见 ``view_targets``）。
        base = 2 * (i - int(start) - 1)
        vp, checks, cam, marg = ws.survey_plan(
            chain, q, zone=tuple(v["zone"]), height=float(v["height"]),
            yaw_deg=yaw,
            transit_dz=float(v.get("transit_dz", VIEW_TRANSIT_DZ)),
            qs=[qpath[base], qpath[base + 1]])
        name = "%02d-view-%s.json" % (i, v["name"])
        path = os.path.join(out_dir, name)
        vp["meta"]["e9"] = {
            "kind": "view", "view": v["name"], "block": None, "slot": None,
            "from_xy": [float(cam[0]), float(cam[1])],
            "grasp_yaw_deg": yaw, "gap_check": "n/a",
            "checks": checks,
            "coverage_px": [round(float(m), 1) for m in marg],
            "zone_m": [float(x) for x in v["zone"]],
            "height_m": float(v["height"]),
            "purpose": "判分机位「%s」：%s"
                       % (v["name"], v.get("purpose", "")),
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(vp, f, ensure_ascii=False, indent=1)
        plans.append({"index": i, "name": name, "path": path,
                      "kind": "view", "view": v["name"], "block": None,
                      "slot": [float(cam[0]), float(cam[1])],
                      "from_xy": [float(cam[0]), float(cam[1])],
                      "grasp_yaw_deg": yaw,
                      "zone_m": [float(x) for x in v["zone"]],
                      "n_waypoints": len(vp["waypoints"])})
        q = [float(x) for x in vp["waypoints"][-1]["q"]]
    return plans, q


def plan_sequence(chain, q_start, actions, blocks, out_dir,
                  survey_tool0=E9_SURVEY_TOOL0, place_clearance=None,
                  wrist_survey=True, views=None, place_yaw_equiv=False,
                  start=0, min_block_gap=0.0, gap_check="probe_verified",
                  tail_targets=None, validator=None):
    """逐步生成计划；任何一步预检失败就抛 ``pp.PlanError``，不写半份序列。

    ``wrist_survey=True``（缺省）在序列末尾依次平动到 ``views`` 里的每个
    **判分机位**（缺省 ``E9_VIEWS``：pick / free / pad），每份计划后面都拍一组
    腕部 burst（见 ``run_script_text``）。跑后判分帧必须与跑前在**同一个位姿**
    上拍（2026-11-09 用户定：规划与判分都走腕部），所以这些机位写进本场
    ``run.sh``，由执行链末端自动完成。任何一个机位过不了预检就抛错 ⇒
    ``run.sh`` 不写、整场不执行（fail-closed），不许跑完才发现拍不了判分帧。

    ``place_yaw_equiv=True``：某个动作按「放置朝向 = 抓取朝向」过不了预检时，
    再试 180° 等价写法（``fold_yaw_180`` 的物理等价：只换手指、不换面，
    方块/矩形块的落点与朝向逐字节相同）。**只对 `pp.BranchChainError` 重试**，
    其它拒绝换朝向救不回来。用的是哪个写法逐条记进计划 meta 与 manifest。

    ``start``：计划名的 ``%02d`` 前缀与 ``index`` 的起点（整轮编排器把 5 个
    目标接在同一个 ``plan/`` 目录里，名字必须错开，否则后一份覆盖前一份、
    执行日志也跟着串名）。旧调用不传 ⇒ 从 1 开始，行为不变。

    ``min_block_gap`` / ``gap_check``：``build_pick`` 的邻块粗判据与它的**判决
    来源**。缺省 ``0.0 / probe_verified`` 是 E9 贴面场景；S0 这类无接触布局
    **必须**用 ``60 mm / layout_gap`` —— 放宽要有判决，判决来源要落盘。

    ``tail_targets``：**最后一个动作之后还要去的目标**（见 ``view_targets``），
    只交给该动作的 ``build_pick(post_targets=...)`` 选支，不写进它的计划。
    ``wrist_survey=True`` 时缺省自动取 ``views`` 的机位目标（本函数随后自己
    就走这些机位）；``wrist_survey=False``（整轮编排器逐目标调用）时由调用方
    在**最后一个目标**上显式传，否则末块的收尾分支可能和后续机位分属不同的
    ±360 折圈族（2026-11-12 attempt6 的 compute 失败）。
    """
    if tail_targets is None and wrist_survey:
        tail_targets = view_targets(E9_VIEWS if views is None else views)
    layout = [dict(b) for b in blocks]
    q = [float(v) for v in q_start]
    plans = []
    for i, act in enumerate(actions, start=1):
        blk = find_block(layout, act["block"])
        if blk is None:
            raise ValueError("布局里没有块 %r" % (act["block"],))
        # 抓点只认动作表里的 ``from_xy``（探针记录 / 跑前实测 / 对中修正后的
        # 抓取点），就地覆盖布局里该块的 x/y，``pp_blocks`` 也跟着更新。
        # ``apply_align`` 本来就同时改这两处、保持一致；只信布局会把实测抓点
        # 丢掉 —— 2026-11-12 S0:1 r1 attempt5：``e9.from_xy`` 是实测，IK 目标
        # 却是标称坐标，差 25 mm，空夹回读 0.0 mm。
        from_xy = act.get("from_xy")
        if from_xy is not None:
            blk["x"] = float(from_xy[0])
            blk["y"] = float(from_xy[1])
        pp_blocks = [to_pp_block(b) for b in layout]
        # ``build_pick`` 只从 cand 里取两样真东西：抓点 xy（这里不用 ——
        # ``grasp_at="block_center"``）与「候选离块心 ≤12 mm」的检查。
        # 其余字段（score/width/angle）是阶段 2 的 GraspNet 元数据，E9 没有，
        # 填的是几何真值而不是编的分数：width = 块边长，angle = 合拢轴角。
        cand = {"xyz_base": [float(blk["x"]), float(blk["y"]),
                             float(blk.get("top", blk["size"]))],
                "score": 0.0,
                "width_mm": float(blk["size"]) * 1000.0,
                "dist_mm": 0.0,
                "angle_deg": float(act["grasp_yaw_deg"]),
                "seed": None}
        yaw_grasp_raw = float(act["grasp_yaw_deg"])
        yaw_grasp = pp.fold_yaw_180(yaw_grasp_raw)
        # 抓取朝向的候选写法：**两条物理等价**的写法（平行两指绕工具轴转 180°
        # 只是两根手指互换位置，抓的还是同一对面 —— 见 `pp.fold_yaw_180`）：
        # ① 折叠写法（默认，腕 3 行程最短）；② 同一位姿整体 +180°（给
        # `build_pick` 传 ``fold_grasp_yaw=False``，**不再折叠**）。
        # 2026-11-14 审计：旧写法两支都过 `fold_yaw_180` ⇒ 折叠后同一个
        # 目标 yaw、同种子同解集 —— 第二支从未真正试过，重试是空转。
        # 现在 +180° 的目标矩阵不同 ⇒ IK 支族不同，重试才有「救回链」的能力；
        # 两支都救不回来才 fail-closed。
        grasp_variants = [(yaw_grasp, True), (yaw_grasp + 180.0, False)]
        wps = checks = meta = None
        grasp_used = None
        yaw_used = None
        first_err = None
        for g, g_fold in grasp_variants:
            # 放置朝向的候选写法：默认（= 本变体的抓取朝向）先试，旗子开了才加
            # 180° 等价写法（`fold_yaw_180` 的物理等价：只换手指、不换面，
            # 方块/矩形块的落点与朝向逐字节相同）。
            yaw_places = [None]
            if place_yaw_equiv:
                yaw_places.append(pp.fold_yaw_180(g) - 180.0)
            for yaw_place in yaw_places:
                try:
                    wps, checks, meta = pp.build_pick(
                        chain, q, cand, to_pp_block(blk), pp_blocks, act["slot"],
                        yaw_grasp_deg=g, grasp_at="block_center",
                        surface_z=act["surface_z"], survey_tool0=tuple(survey_tool0),
                        place_clearance=place_clearance,
                        yaw_place_deg=yaw_place,
                        yaw_source=("place-equiv180" if yaw_place is not None
                                    else "zero"),
                        # 压在目标上的块：`h`（块顶离台面）≠ 物理块高，必须显式给
                        # 物理块高，否则抓取高度按两个块高算（= 2026-11-02 拆塔事故）。
                        block_h_m=float(blk["size"]),
                        # 见模块头：探针已用 OBB 判过间隙才允许放宽；S0 无接触布局
                        # 仍走 60 mm 粗判据（gap_check=layout_gap）。
                        min_block_gap=float(min_block_gap),
                        # 只有最后一个动作需要为「之后的机位」选支（见 tail_targets）
                        post_targets=(tail_targets if i == len(actions) else None),
                        # A2：段级碰撞裁决（板上 MoveIt /check_state_validity）；
                        # None = 本机离线排练（无裁决，只靠步长/护栏）
                        validator=validator,
                        # ② 整体 +180° 的变体不折叠（见 grasp_variants 注释）
                        fold_grasp_yaw=g_fold)
                except pp.BranchChainError as e:
                    first_err = first_err or e
                    continue
                grasp_used = g
                yaw_used = yaw_place
                break
            if wps is not None:
                break
        if wps is None:
            # 所有等价写法都被分支链/单步上限拒了 ⇒ 抛第一次（默认朝向）的原始
            # 报错：它才是「按实测朝向根本走不通」这件事本身。
            raise first_err
        grasp_equiv = grasp_used != yaw_grasp
        if yaw_used is not None:
            log("  !! 块 %s：默认放置朝向过不了分支链/单步上限，改用 180° 等价"
                "写法（只换手指、不换面）：ψ_place %.3f° → %.3f°"
                % (act["block"], yaw_grasp, yaw_grasp - 180.0))
        if grasp_equiv:
            log("  !! 块 %s：折叠写法（ψ=%.3f°）过不了分支链/单步上限，改用整体"
                "+180° 等价写法（只换手指、不换面）：ψ_grasp %.3f° → %.3f°"
                % (act["block"], yaw_grasp, yaw_grasp, grasp_used))
        # 相邻航点的腕奇异跨零检查 + 大跳插点（与 stage3 / 机位计划同一道门；
        # 2026-11-14 审计：stage4 抓放计划此前不经 split_jumps，中段穿 q5=0
        # 无人查 —— 正是「动作扭曲」的来源之一）。
        n_split = 0
        try:
            wps, n_split = pp.split_jumps(wps, chain, q_start=q)
        except pp.JointSwingError as e:
            raise pp.PlanError("块 %s 的计划分跳检查不过：%s" % (act["block"], e))
        if n_split:
            log("  块 %s：大跳插了 %d 个中间航点（指令变小，轨迹形状不变）"
                % (act["block"], n_split))
        plan = {"version": 2,
                "meta": dict(meta,
                             e9={"kind": act["kind"], "block": act["block"],
                                 "slot": list(act["slot"]),
                                 "from_xy": list(act["from_xy"]),
                                 "grasp_yaw_deg": round(
                                     float(act["grasp_yaw_deg"]), 3),
                                 "grasp_yaw_used_deg": round(
                                     float(grasp_used), 3),
                                 "grasp_yaw_equiv180": grasp_equiv,
                                 "place_yaw_equiv180": yaw_used is not None,
                                 "gap_check": gap_check,
                                 "desired_center_mm": act.get(
                                     "desired_center_mm"),
                                 "commanded_center_mm": act.get(
                                     "commanded_center_mm"),
                                 "applied_bias_mm": act.get("applied_bias_mm"),
                                 "checks": checks}),
                "q_start": list(q),
                "waypoints": wps}
        name = "%02d-%s-block-%s.json" % (int(start) + i, act["kind"],
                                          act["block"])
        path = os.path.join(out_dir, name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(plan, f, ensure_ascii=False, indent=1)
        plans.append({"index": int(start) + i, "name": name, "path": path,
                      "kind": act["kind"], "block": act["block"],
                      "slot": act["slot"], "from_xy": act["from_xy"],
                      "grasp_yaw_deg": act["grasp_yaw_deg"],
                      "grasp_yaw_equiv180": grasp_equiv,
                      "place_yaw_equiv180": yaw_used is not None,
                      "desired_center_mm": act.get("desired_center_mm"),
                      "commanded_center_mm": act.get("commanded_center_mm"),
                      "applied_bias_mm": act.get("applied_bias_mm"),
                      "gap_check": gap_check,
                      "n_waypoints": len(wps)})
        q = [float(v) for v in wps[-1]["q"]]
        move_in_layout(layout, act["block"], act["slot"][0], act["slot"][1],
                       act["surface_z"])
    if wrist_survey:
        vplans, q = plan_views(
            chain, q, E9_VIEWS if views is None else views, out_dir,
            start=int(start) + len(plans), validator=validator)
        plans.extend(vplans)
    return plans, layout, q


def fuse_post_blocks(topcam_blocks, wrist_blocks, zone=None, dedup_m=0.040):
    """跑后两份真值合成**一份**终态：腕部管内是权威，俯视只贡献区外的块。

    为什么按区切而不是合并点云（2026-11-09 用户定，见台账第二百九十二轮）：

    · **区内**（拾取区）判块数/塔层只有腕部可信 —— 同一根 59 mm 的塔，俯视深度
      读成 37–45 mm，而 ``layer_count`` 的 1↔2 层界在 44.25 mm：一根塔在俯视里
      可能被数成 1 块也可能被数成 2 块（s6 实测）。腕部 500 mm 巡检位量同一批塔
      塔顶 60.5–61.7 mm（真值 59）。
    · **区外**（垫面、北侧自由区）腕部巡检位根本看不见（视场 y∈[0.10, 0.49]），
      而那里的块都是**单块**、俯视量单块是准的；俯视在区内的幻影（实测
      (305.4, 281.2) mm）又恰好被这条规则挡在门外。

    跨界的同一块可能两边都报到（腕部真值按 zone 裁、俯视按坐标判，差几毫米就会
    一边在里一边在外），所以区外的俯视块若离某个腕部块 ≤ ``dedup_m`` 就当重复丢弃。

    返回 ``(blocks, provenance)``；``blocks`` 的每块带 ``_source``
    （``wrist`` / ``topcam``），供报告追溯。
    """
    zone = sp.ZONE_M if zone is None else tuple(zone)
    wrist = [dict(b, _source="wrist") for b in wrist_blocks]
    out = list(wrist)
    inside, dup, taken = [], [], []
    for b in topcam_blocks:
        x, y = float(b["x"]), float(b["y"])
        if tf.in_zone_xy(x, y, zone):
            inside.append({"x_mm": round(x * 1000.0, 1),
                           "y_mm": round(y * 1000.0, 1)})
            continue
        near = [w for w in wrist
                if math.hypot(float(w["x"]) - x, float(w["y"]) - y) <= dedup_m]
        if near:
            dup.append({"x_mm": round(x * 1000.0, 1),
                        "y_mm": round(y * 1000.0, 1),
                        "near_wrist_mm": round(
                            1000.0 * math.hypot(float(near[0]["x"]) - x,
                                                float(near[0]["y"]) - y), 1)})
            continue
        taken.append(dict(b, _source="topcam"))
    out += taken
    # id 空间可能撞车（两份真值各自从 0 编号）⇒ 按来源加前缀，配对只按位置
    for i, b in enumerate(out):
        b["id"] = "%s%d" % ("w" if b["_source"] == "wrist" else "t", i)
    prov = {"zone_m": list(zone), "dedup_mm": round(dedup_m * 1000.0, 1),
            "n_wrist": len(wrist), "n_topcam_used": len(taken),
            "topcam_inside_zone_dropped": inside,
            "topcam_duplicate_dropped": dup}
    return out, prov


def merge_truth_blocks(truths, dedup_m=0.040, layer_m=0.015):
    """多个**机位**的跑后真值 → 一份终态（2026-11-09 全腕部口径）。

    ``truths`` = ``[(机位名, blocks), ...]``，按给定顺序定权威（先来的留下）。
    三个机位的覆盖区互不重叠，正常情况下没有重复；只在区界上（拾取区与自由区
    都到 y≈0.46–0.49）同一块可能被两个机位各报一次 —— 那时按 ``dedup_m`` 距离
    判重复，只留先报的，并列进 ``duplicates`` 供追溯。

    ⚠ **同一机位内部不做去重**：相机一次共识已经把同一块的多次观测聚过了；
    机位内两块再近也是两块真块（2026-11-11 s11 重放：自由区两块中心距 37.0 mm
    < 40 mm，被同机位去重吃掉一块 ⇒ 判「缺块」）。去重只在**跨机位**做。

    ⚠ 去重必须**带层**：塔的两层 XY 完全重合、只差 29.5 mm 高。只按 XY 去重会把
    上层当重复吃掉（2026-11-09 实测：3 根两层塔 6 块被合掉 2 块，判成「缺块」）。
    ⇒ 只有 XY ≤ ``dedup_m`` **且** 顶面高度差 ≤ ``layer_m`` 才算同一块。
    ``layer_m`` 取半层高：同层重复的读数差在毫米级，不同层差一整层 29.5 mm。

    返回 ``(blocks, provenance)``；每块带 ``_source``（机位名）与唯一 ``id``。
    """
    out, dup = [], []
    prov = {"views": [], "duplicates": dup,
            "dedup_mm": round(float(dedup_m) * 1000.0, 1),
            "layer_mm": round(float(layer_m) * 1000.0, 1)}

    def _same(o, b, x, y):
        if math.hypot(float(o["x"]) - x, float(o["y"]) - y) > float(dedup_m):
            return False
        ot, bt = o.get("top"), b.get("top")
        if ot is None or bt is None:
            return False          # 没高度就判不了层 ⇒ 宁可不合
        return abs(float(ot) - float(bt)) <= float(layer_m)   # top 单位是米

    for name, blocks in truths:
        kept = 0
        prev = list(out)          # 只跟**先前机位**比；同机位内永远不是重复
        for b in blocks:
            x, y = float(b["x"]), float(b["y"])
            near = [o for o in prev if _same(o, b, x, y)]
            if near:
                dup.append({"view": str(name),
                            "x_mm": round(x * 1000.0, 1),
                            "y_mm": round(y * 1000.0, 1),
                            "top_mm": (None if b.get("top") is None
                                       else round(float(b["top"]) * 1000.0, 1)),
                            "kept_view": near[0]["_source"]})
                continue
            out.append(dict(b, _source=str(name)))
            kept += 1
        prov["views"].append({"view": str(name), "n_blocks": len(blocks),
                              "n_kept": kept})
    for i, b in enumerate(out):
        b["id"] = "%s%d" % (b["_source"], i)
    return out, prov


def _match_blocks(exp, post):
    """两份真值按**三维最近位置**一一配对，返回 ``(match, missing, extra)``。

    **不能按 id 配对**：id 只是同一份真值内部的序号，两份真值各自按位置重排
    （2026-11-08 实测：跑后真值的 id 顺序和计划里的 ``final_layout`` 完全不同，
    按 id 对齐会把紫塔的块配到橙塔上、报出 240 mm 的假位移）。
    配对之后也不比颜色 —— 塔底块的颜色取自塔顶面，真值里写着塔顶的颜色。
    """
    pairs = []
    for ei, e in exp.items():
        for pi, p in post.items():
            d = math.sqrt((float(p["x"]) - float(e["x"])) ** 2
                          + (float(p["y"]) - float(e["y"])) ** 2
                          + (float(p["z"]) - float(e["z"])) ** 2) * 1000.0
            pairs.append((d, ei, pi))
    pairs.sort()
    used_e, used_p, match = set(), set(), {}
    for d, ei, pi in pairs:            # 全局按距离贪心，一对一
        if ei in used_e or pi in used_p:
            continue
        used_e.add(ei)
        used_p.add(pi)
        match[ei] = (pi, d)
    return (match, sorted(set(exp) - used_e, key=lambda s: (len(s), s)),
            sorted(set(post) - used_p, key=lambda s: (len(s), s)))


# ------------------------------------------------------------ 目标块彩色量具
#: 彩色门限：白垫 s≈0、黑标记 v≈0，都被这两条排除。
COLOUR_S_MIN, COLOUR_V_MIN = 90.0, 60.0
#: 顶面取「窗内最亮的一批」：顶面比侧面亮，取最亮 40% 让质心落在顶面，
#: 不被侧面的透视拉伸拖偏（2026-11-08 实测 10 帧极差 ≤7 mm）。
COLOUR_TOP_PCT = 40.0
#: 一帧至少要这么多个色彩像素才算「看见目标块」。29.5 mm 的块在垫面约 16×16 px
#: ⇒ 顶面 250 px 量级；150 只在远/斜视时触发。
COLOUR_MIN_PX = 150


def hsv_of(img):
    """RGB uint8 图像 → ``(h 度 0–360, s 0–255, v 0–255)``。"""
    a = np.asarray(img).astype(np.float32) / 255.0
    mx, mn = a.max(2), a.min(2)
    d = mx - mn
    h = np.zeros_like(mx)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    m = d > 1e-6
    i = m & (mx == r)
    h[i] = (60.0 * ((g[i] - b[i]) / d[i])) % 360.0
    i = m & (mx == g)
    h[i] = 60.0 * ((b[i] - r[i]) / d[i]) + 120.0
    i = m & (mx == b)
    h[i] = 60.0 * ((r[i] - g[i]) / d[i]) + 240.0
    s = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0) * 255.0
    return h, s, mx * 255.0


def _hue_peak(h, step=15.0):
    """主色相簇 → ``(簇中心角, 落在峰 ±1 bin 内的掩码)``。

    只取峰附近、不取「窗口里所有彩色像素」：窗口里万一还有别的颜色的东西
    （邻塔、反光），按整窗平均会把两样东西的颜色混成一个不存在的色。

    簇中心用**保留像素的圆均值**、不用 bin 中心：一个窄色簇全落在同一 bin
    时，bin 中心最多差半步（实测偏 15°），而且环形平滑会让相邻 bin 打平。
    """
    n = int(round(360.0 / step))
    idx = (np.mod(h, 360.0) / step).astype(int) % n
    cnt = np.bincount(idx, minlength=n).astype(int)
    p = int(np.argmax(cnt))
    keep = np.zeros(n, bool)
    keep[(p - 1) % n] = keep[p] = keep[(p + 1) % n] = True
    m = keep[idx]
    ang = np.radians(h[m])
    mean = math.degrees(math.atan2(float(np.sin(ang).mean()),
                                  float(np.cos(ang).mean()))) % 360.0
    return mean, m


def colour_blob_uv(img, uv0, half_px, min_px=COLOUR_MIN_PX):
    """窗口里「一坨有颜色的东西」的**顶面质心**像素坐标；看不见 ⇒ ``None``。

    ``uv0`` = 窗口中心（计划宫格在块顶高的投影），``half_px`` = 半窗宽。
    返回 ``{"uv", "n_px", "n_top", "hue_deg"}``。

    **看不见就返回 None，不猜**：调用方必须把 None 判红，不许拿计划位当量值。
    """
    h, s, v = hsv_of(img)
    H, W = h.shape
    r = int(round(float(half_px)))
    u0, v0 = int(round(float(uv0[0]))), int(round(float(uv0[1])))
    ua, ub = max(0, u0 - r), min(W, u0 + r + 1)
    va, vb = max(0, v0 - r), min(H, v0 + r + 1)
    if ub - ua < 5 or vb - va < 5:
        return None
    sh, ss, sv = h[va:vb, ua:ub], s[va:vb, ua:ub], v[va:vb, ua:ub]
    m = (ss >= COLOUR_S_MIN) & (sv >= COLOUR_V_MIN)
    if int(m.sum()) < min_px:
        return None
    peak, keep = _hue_peak(sh[m])
    mm = np.zeros_like(m)
    mm[m] = keep
    if int(mm.sum()) < min_px:
        return None
    ys, xs = np.nonzero(mm)
    vv = sv[ys, xs]
    # 最亮 40%（分位取 100−40）：平坦面（整块同色）分位就等于该值 ⇒ 全取，
    # 不会被阈值空掉；真实块上这就是顶面，侧面更暗、被排除。
    top = vv >= np.percentile(vv, 100.0 - COLOUR_TOP_PCT)
    if int(top.sum()) < 20:
        return None
    return {"uv": (ua + float(xs[top].mean()), va + float(ys[top].mean())),
            "n_px": int(mm.sum()), "n_top": int(top.sum()),
            "hue_deg": round(peak, 1)}


def _base_to_uv(p, K, R, t):
    """base 系一点 → 俯视像素（``ray_plane_xy`` 的逆）。在相机后方 ⇒ None。"""
    pc = np.asarray(R, float).T @ (np.asarray(p, float) - np.asarray(t, float))
    if pc[2] <= 1e-6:
        return None
    uv = np.asarray(K, float) @ pc
    return (float(uv[0] / uv[2]), float(uv[1] / uv[2]))


def _window_half_px(xy, z, half_mm, K, R, t):
    """以 (x, y, z) 为心、边长 2·half_mm 的正方形投影到像面的半窗宽（px）。"""
    uv0 = _base_to_uv((xy[0], xy[1], z), K, R, t)
    if uv0 is None:
        return None
    h = half_mm * 0.001
    cors = [_base_to_uv((xy[0] + dx * h, xy[1] + dy * h, z), K, R, t)
            for dx in (-1.0, 1.0) for dy in (-1.0, 1.0)]
    cors = [c for c in cors if c is not None]
    if not cors:
        return None
    return max(8.0, max(max(abs(c[0] - uv0[0]), abs(c[1] - uv0[1]))
                        for c in cors))


#: 彩色量具能用在哪些相机上 → ``(tip, chain)``。2026-11-09 起 E9 用 ``wrist``
#: （垫视角），``topcam`` 只为复现旧证据保留。
COLOUR_CAMERAS = {"topcam": (gc.TOPCAM_TIP, gc.TOPCAM_CHAIN),
                  "wrist": (gc.TIP, gc.CHAIN)}


def measure_target_colour(burst_dir, slot_xy, top_z, search_mm=25.0,
                          max_frames=10, min_frames=5, camera="wrist",
                          expect_hue_deg=None, expect_class=None):
    """跑后某一路相机的 burst → 目标块顶面质心（台面 mm，中位）与可复现性。

    **为什么目标块要换彩色量**（2026-11-08 用户拍板）：垫面在俯视深度里
    分辨不出 29.5 mm 的台阶 —— 实测 10 帧只有 3 帧有真台阶，跑前同一带本来
    就有 15–30 mm 的幻影穹顶，``find_blocks`` 把块并进 48×46 mm 的一坨、
    质心被拉偏 33 mm（场景 2 因此被深度判成「没落格」，实际落在格内）。
    同一块在彩色里清清楚楚。**其余判据仍用深度真值**：块数、旁观块、终态、
    清空块都在台面上，深度是准的。

    每帧：窗内取色彩像素 → 主色相簇 → 最亮 40%（顶面）质心 → 射线 ∩
    块顶水平面。跨帧取中位；``ok`` 要求至少 ``min_frames`` 帧看见。

    返回 ``{"ok", "xy_mm", "hue_deg", "n_frames", "n_missed", "miss_reasons",
    "per_frame", "search_mm", "top_z"}``。漏帧不静默：每一帧漏的原因都写下来
    （缺静态 TF / 窗内没看见 / 投不到像面），``ok`` 只看看得见的帧数够不够。

    **默认窗 25 mm**（正式轮）：格心彼此 ≥84.85 mm、邻块最近角 45.25 mm，
    60 mm 窗会把对角邻块的角罩进来 —— 彩色量一旦抓到邻块，质心偏一个格距的
    分量，落格判分跟着错，而且看起来「有数」。

    ``expect_hue_deg`` / ``expect_class``：交叉核对（**只告警，不改 ``ok``**）。
    ``class_match`` 走 ``block_classes.nearest_class_by_hue``（真机实测色相表）
    —— 量出来的色相若会被判成**另一种颜色**，说明窗里抓错了块或光变了。
    """
    if camera not in COLOUR_CAMERAS:
        raise ValueError("camera 只能是 %s，收到 %r"
                         % (sorted(COLOUR_CAMERAS), camera))
    tip, chain = COLOUR_CAMERAS[camera]
    import glob
    paths = sorted(glob.glob(os.path.join(burst_dir, "color_*.png")))[:max_frames]
    rows, miss, reasons = [], 0, []
    for path in paths:
        idx = int(os.path.basename(path).rsplit("_", 1)[-1].split(".")[0])
        try:
            fr = gc.load_frame(burst_dir, tip=tip, chain=chain, idx=idx)
        except KeyError as e:
            # 起栈后的头一两帧可能还没收到锁存的 ``base_link:topcam_link``
            # 静态 TF（2026-11-08 实测）。记成漏帧，别让核验整个崩掉。
            miss += 1
            reasons.append("帧%d 缺静态 TF（%s）" % (idx, e))
            continue
        K = np.asarray(fr["K"], float)
        R = np.asarray(fr["R_bc"], float)
        t = np.asarray(fr["t_bc"], float)
        uv0 = _base_to_uv((slot_xy[0], slot_xy[1], top_z), K, R, t)
        hp = (None if uv0 is None
              else _window_half_px(slot_xy, top_z, search_mm, K, R, t))
        if hp is None:
            miss += 1
            reasons.append("帧%d 计划宫格投不到像面" % idx)
            continue
        with Image.open(path) as im:
            img = np.asarray(im.convert("RGB"))
        blob = colour_blob_uv(img, uv0, hp)
        if blob is None:
            miss += 1
            reasons.append("帧%d 窗内没看见彩色块" % idx)
            continue
        xy = tf.ray_plane_xy(list(blob["uv"]), K, R, t, float(top_z))
        if xy is None:
            miss += 1
            reasons.append("帧%d 射线与块顶平面不相交" % idx)
            continue
        rows.append({"frame": idx, "uv": [round(c, 2) for c in blob["uv"]],
                     "x_mm": round(float(xy[0]) * 1000.0, 2),
                     "y_mm": round(float(xy[1]) * 1000.0, 2),
                     "n_px": blob["n_px"], "hue_deg": blob["hue_deg"]})
    out = {"ok": len(rows) >= max(int(min_frames), 1),
           "search_mm": search_mm, "top_z": top_z,
           "n_frames": len(rows), "n_missed": miss,
           "miss_reasons": reasons, "per_frame": rows}
    if rows:
        a = np.array([[r["x_mm"], r["y_mm"]] for r in rows], float)
        med = np.median(a, axis=0)
        out["xy_mm"] = [round(float(med[0]), 2), round(float(med[1]), 2)]
        out["max_dev_mm"] = round(float(np.hypot(a[:, 0] - med[0],
                                                 a[:, 1] - med[1]).max()), 2)
        out["range_mm"] = [round(float(a[:, 0].max() - a[:, 0].min()), 2),
                           round(float(a[:, 1].max() - a[:, 1].min()), 2)]
        sn = sum(math.sin(math.radians(r["hue_deg"])) for r in rows)
        cs = sum(math.cos(math.radians(r["hue_deg"])) for r in rows)
        out["hue_deg"] = round(math.degrees(math.atan2(sn, cs)) % 360.0, 1)
        if expect_hue_deg is not None or expect_class is not None:
            import block_classes as bc     # 惰性 import：多数调用不需要色表
            hue = float(out["hue_deg"])
            out["expected_hue_deg"] = (None if expect_hue_deg is None
                                       else round(float(expect_hue_deg), 1))
            if expect_hue_deg is not None:
                out["hue_delta_deg"] = round(
                    bc.hue_distance(hue, float(expect_hue_deg)), 1)
            if expect_class is not None:
                got, dist = bc.nearest_class_by_hue(hue)
                out["expected_class"] = str(expect_class)
                out["measured_class"] = got
                out["class_match"] = bool(got == str(expect_class))
                out.setdefault("hue_delta_deg", round(float(dist), 1))
    return out


def compare_layout(manifest, post_blocks, tol_untouched_mm=5.0,
                   tol_untouched_deg=3.0, tol_moved_mm=40.0,
                   target_colour=None):
    """跑完再拍一遍真值 → 判「不丢指定块 / 不掉块 / 不撞」。

    三条判据各自的量（都不靠「看起来对」）：

    · **不丢指定块**：目标块的终态必须落在计划的那一格（``cell_index_of``）。
      **量具是彩色顶面质心**（``target_colour``，见 ``measure_target_colour``）：
      垫面在俯视深度里分辨不出 29.5 mm 台阶，深度真值在这里会假红。给了
      ``target_colour`` 而它 ``ok=False``（看得见的帧不够）⇒ 直接判红，
      **不退回深度、也不拿计划位当量值**。深度给出的同一位置写进 ``notes``
      供对照，只记录、不判。
    · **不掉块**：``stage4_clear_model.terminal_report`` —— 每块有真支撑
      （台面 / 垫面 / 真的压在另一块上）、不穿插、不出桌。掉到地上 = 不在
      台面支撑里，必然红。
    · **不撞**：**没被碰的块不许动** —— 按**位移**判（``tol_untouched_mm``）。
      静置的块被推动只有一种原因：臂或夹爪碰到了它。清障块本身有 40 mm 的落点
      公差（抓取/落点误差），不加严。这条是间接判据，所以报告里把每块的位移
      都列出来，别只看结论。

      **转角只记录、不判红**（2026-11-08 用户拍板）。依据是实测的重复性：臂不动、
      同一场景连拍两组 burst，深度真值的块 yaw 自己就差 4.5°（塔）到 36°（单块）
      —— 3° 的公差**低于测量底噪**，判了只会假红。同一场景的彩色剪影取向只差
      1–2°，可见这是深度口径的问题，不是块真的在转。位移的重复性是 0.2–1.7 mm，
      5 mm 的公差撑得住。要恢复 yaw 判据，先换测量口径（见台账第二百八十四轮）。

    计划里的块与跑后真值里的块**按三维位置配对**（见 ``_match_blocks``），
    不按 id、不比颜色。

    返回 ``{"ok": bool, "checks": [...], "blocks": [...], "terminal": ...}``。
    """
    exp = {str(b["id"]): b for b in manifest.get("final_layout") or []}
    post = {str(b["id"]): b for b in post_blocks}
    moved = {str(a["block"]) for a in manifest.get("actions") or []}
    match, missing, extra = _match_blocks(exp, post)
    checks, rows = [], []
    ok = True

    def add(name, good, detail):
        nonlocal ok
        ok = ok and bool(good)
        checks.append({"check": name, "ok": bool(good), "detail": detail})

    add("块数一致（没丢块、没多块）", not missing and not extra,
        "缺 %s / 多 %s" % (missing or "无", extra or "无"))

    rep = cm.terminal_report(post_blocks)
    add("全桌终态合法（不悬空、不穿插、不出桌）", rep["ok"],
        "overlaps=%d off_table=%s out_of_bounds=%s"
        % (len(rep.get("overlaps") or []), rep.get("off_table"),
           rep.get("out_of_bounds")))

    tid = str(manifest.get("target"))
    cell = manifest.get("cell")
    notes = []
    if target_colour is not None and cell is not None:
        name = "指定块落在计划的那一格（#%s，彩色顶面质心）" % cell
        if not target_colour.get("ok"):
            add(name, False,
                "彩色量不够帧（%d 帧看见 / 需 ≥5，漏 %d 帧）—— 没有量值，不判过"
                % (target_colour.get("n_frames") or 0,
                   target_colour.get("n_missed") or 0))
        else:
            tx, ty = [float(v) / 1000.0 for v in target_colour["xy_mm"]]
            got = cm.cell_index_of(tx, ty)
            cx, cy = cm.cell_center(int(cell))
            rng = target_colour.get("range_mm") or [0.0, 0.0]
            add(name, got == int(cell),
                "实际格 %r，位置 (%.1f, %.1f) mm，距格心 %.1f mm"
                "（%d 帧中位；极差 x=%.1f / y=%.1f mm；单帧最大偏离 %.1f mm；"
                "色相 %.0f°；窗 ±%.0f mm）"
                % (got, tx * 1000, ty * 1000,
                   math.hypot(tx - cx, ty - cy) * 1000.0,
                   target_colour.get("n_frames") or 0, rng[0], rng[1],
                   target_colour.get("max_dev_mm") or 0.0,
                   target_colour.get("hue_deg") or 0.0,
                   target_colour.get("search_mm") or 0.0))
        if tid in match:
            db = post[match[tid][0]]
            dgot = cm.cell_index_of(float(db["x"]), float(db["y"]))
            notes.append("同一位置深度真值给出 (%.1f, %.1f) mm → 格 %r"
                         % (float(db["x"]) * 1000, float(db["y"]) * 1000, dgot))
        else:
            notes.append("深度真值里配不上目标块 #%s" % tid)
    elif tid in match and cell is not None:
        pi = match[tid][0]
        tb = post[pi]
        got = cm.cell_index_of(float(tb["x"]), float(tb["y"]))
        add("指定块落在计划的那一格（#%s）" % cell, got == int(cell),
            "实际格 %r，位置 (%.1f, %.1f) mm"
            % (got, float(tb["x"]) * 1000, float(tb["y"]) * 1000))
    elif tid not in match:
        add("指定块落在计划的那一格（#%s）" % cell, False,
            "跑后真值里找不到目标块 #%s（已丢？）" % tid)

    for bid in sorted(match, key=lambda s: (len(s), s)):
        pi, d3 = match[bid]
        b, e = post[pi], exp[bid]
        d = math.hypot(float(b["x"]) - float(e["x"]),
                       float(b["y"]) - float(e["y"])) * 1000.0
        # 积木是正方形：yaw 与 yaw+90° 是同一个盒子，必须按 90° 折角比
        dyaw = abs((float(b.get("yaw", 0.0)) - float(e.get("yaw", 0.0))
                    + 45.0) % 90.0 - 45.0)
        row = {"id": bid, "post_id": pi, "moved": bid in moved,
               "d_mm": round(d, 2), "dyaw_deg": round(dyaw, 2),
               "dz_mm": round((float(b["z"]) - float(e["z"])) * 1000.0, 2)}
        rows.append(row)
        if bid in moved:
            add("被搬的块 #%s 落在计划位附近" % bid, d <= tol_moved_mm,
                "%.1f mm（公差 %.0f）" % (d, tol_moved_mm))
        else:
            # 只按位移判；转角写进 detail 供人看（口径见函数 docstring）
            add("没被碰的块 #%s 没被动过" % bid, d <= tol_untouched_mm,
                "位移 %.1f mm（公差 %.0f）；转角 %.1f°（只记录，不判）"
                % (d, tol_untouched_mm, dyaw))
    for pi in extra:
        rows.append({"id": None, "post_id": pi, "moved": False,
                     "d_mm": None, "dyaw_deg": None, "dz_mm": None})
    return {"ok": ok, "checks": checks, "blocks": rows, "terminal": rep,
            "notes": notes,
            "tolerances": {"untouched_mm": tol_untouched_mm,
                           "untouched_deg": tol_untouched_deg,
                           "moved_mm": tol_moved_mm}}


def apply_pad_pose(args, hint_dir, recorded=None):
    """把本场**实测**垫位姿装进宫格参考（计划与判分共用这一份）。

    优先级：``--grid`` > manifest 里记的路径（它记的就是这份计划当初用的那份）
    > 同目录约定文件（``grid-pre.json`` / ``grid.json``）。一个都找不到 ⇒
    报错停下（fail-closed），除非显式给了 ``--allow-model-pad``。

    为什么 fail-closed：垫会被人挪动，模型常量是**仿真世界**的位姿。2026-11-09
    实测两者差 87.5 mm / 88.7°（台账第二百九十五轮）—— 用常量会把块放到错格上，
    再用同一个错格心判它「对」，两边一起错。
    """
    path = getattr(args, "grid", None)
    if not path and isinstance(recorded, dict):
        # manifest 直接记下了当初用的那份**实测值**（source + center_mm + yaw_deg）：
        # 就地装上，别再去猜文件路径 —— 计划目录里通常没有那份 grid.json。
        center = recorded.get("center_mm")
        yaw = recorded.get("yaw_deg")
        if recorded.get("measured") and center is not None and yaw is not None:
            cm.configure_pad(center, yaw,
                             source="manifest.pad_pose（%s）"
                             % recorded.get("source", "?"))
            log("  垫位姿：%.1f, %.1f mm / %.3f°  ← manifest 记录（%s）"
                % (float(center[0]), float(center[1]), float(yaw),
                   recorded.get("source", "?")))
            return None
        recorded = None           # 模型常量或字段不全 ⇒ 走常规查找（fail-closed）
    if not path and recorded:
        path = recorded if os.path.isabs(recorded) else os.path.join(
            hint_dir, recorded)
        if not os.path.exists(path):
            path = None
    if not path:
        path = cm.find_grid(hint_dir)
    if path:
        doc = cm.load_grid(path)
        log("  垫位姿：%.1f, %.1f mm / %.3f°  ← %s（%d 帧 RMS %.2f px）"
            % (doc["grid_center_base_mm"][0], doc["grid_center_base_mm"][1],
               doc["grid_yaw_base_deg"], path,
               doc.get("checks", {}).get("n_frames", 0),
               doc.get("checks", {}).get("rms_adopted_px", float("nan"))))
        return path
    if getattr(args, "allow_model_pad", False):
        # C1：显式安装模型常量（仿真复现专用）。真复位（清实测值）由
        # reset_pad 的哨兵语义管；这里要的是「装上模型世界」。
        cm.install_model_pad_defaults()
        log("  ⚠ 没找到实测垫位姿，按 --allow-model-pad 用模型常量 %.1f, %.1f mm / %.3f°"
            % (cm.PAD_CENTER[0] * 1000.0, cm.PAD_CENTER[1] * 1000.0,
               math.degrees(cm.PAD_YAW_RAD)))
        return None
    raise SystemExit(
        "没有实测垫位姿（grid.json）：%s\n"
        "  每场开跑前必须先解：python3 scripts/stage4_pad_from_burst.py "
        "<跑前腕部 pad 机位 burst> --out %s/grid-pre.json\n"
        "  （或用 --grid 指定；只做仿真复现时才加 --allow-model-pad）"
        % (cm.PAD_POSE_SOURCE, hint_dir or "<out-dir>"))


def pad_pose_record(path):
    """manifest 里记一行「这份计划用的是哪份垫位姿」。"""
    if not path:
        return {"source": cm.PAD_POSE_SOURCE, "measured": False}
    return {"source": os.path.basename(path), "measured": True,
            "center_mm": [round(cm.PAD_CENTER[0] * 1000.0, 2),
                          round(cm.PAD_CENTER[1] * 1000.0, 2)],
            "yaw_deg": round(math.degrees(cm.PAD_YAW_RAD), 3)}


def warn_pad_pose_mismatch(manifest, tol_mm=5.0, tol_deg=1.0):
    """计划与判分必须用**同一份**垫位姿。

    计划落点来自探针报告（``gen_scene --grid`` 那一刻写进 ``scene_truth.json``），
    判分走本场实测的 grid.json —— 两个入口，可能拿到两份。差得明显就喊出来，
    免得「计划按 A 放、判分按 B 判」又变成自洽的错误。"""
    tpath = manifest.get("truth")
    if not tpath or not os.path.exists(tpath):
        return
    try:
        with open(tpath, encoding="utf-8") as f:
            posed = (json.load(f) or {}).get("pad_pose")
    except (ValueError, OSError):
        return
    if not posed:
        log("  ⚠ 计划侧 scene_truth 没记垫位姿（旧场景）—— 无法核对计划/判分同源")
        return
    d_mm = math.hypot(posed["center_mm"][0] - cm.PAD_CENTER[0] * 1000.0,
                      posed["center_mm"][1] - cm.PAD_CENTER[1] * 1000.0)
    d_deg = abs(posed["yaw_deg"] - math.degrees(cm.PAD_YAW_RAD))
    if d_mm > tol_mm or d_deg > tol_deg:
        log("  ⚠ 计划与判分的垫位姿不是同一份：计划 %.1f, %.1f mm / %.3f°，"
            "判分 %.1f, %.1f mm / %.3f°（差 %.1f mm / %.3f°）—— 这条判决不可信"
            % (posed["center_mm"][0], posed["center_mm"][1], posed["yaw_deg"],
               cm.PAD_CENTER[0] * 1000.0, cm.PAD_CENTER[1] * 1000.0,
               math.degrees(cm.PAD_YAW_RAD), d_mm, d_deg))


def round_gate(out_dir):
    """收尾闸门（§20.3）：没有脚本写出的完整 ``round.json``，这一轮不算通过。

    返回 ``(rc, notes)``：rc=0 = 可以出判决；2 = 缺文件或不完整。

    为什么单独一条：``verify.json`` 只回答「全桌终态对不对」，而抓取三元组
    （回读 / 滑移 / 跑后三机位）、安全计数、干预、人眼复核都只在
    ``round.json`` 里。缺它就给 0，等于把「没归档的一轮」算成绿灯 —— 这正是
    s11 暴露的问题：结论有，证据链没有。
    """
    path = os.path.join(out_dir or "", "round.json")
    if not os.path.exists(path):
        return 2, ["缺 round.json（先用 scripts/stage4_round_report.py 生成）"]
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError) as e:
        return 2, ["round.json 读不了：%s" % e]
    miss = []
    if doc.get("kind") != "stage4-round":
        miss.append("kind=%r（应为 stage4-round）" % doc.get("kind"))
    if doc.get("complete") is not True:
        miss.append("complete!=true：%s"
                    % "；".join(doc.get("missing") or ["未标原因"]))
    return (2 if miss else 0), miss


def verify_main(args):
    """跑后核验：读真值 + manifest，判「不丢指定块 / 不掉块 / 不撞」。

    三种口径，按 ``args`` 选（**新证据一律用第一条**）：

    1. ``--view-truth 机位=json``（可重复，2026-11-09 起 E9 的唯一口径）：
       跑后**每个腕部机位**各出一份真值，合并成一份终态（``merge_truth_blocks``）。
       全场不碰俯视的绝对坐标 —— 俯视 base 外参今天两路自测互差 75 mm，不能用。
    2. ``--verify + --wrist-truth``：区内用腕部、区外补俯视（``fuse_post_blocks``）。
    3. ``--verify`` 单份俯视真值：旧口径，对塔的块数不可靠，只为复现旧证据。
    """
    mpath = args.manifest or os.path.join(args.out_dir or "", "manifest.json")
    with open(mpath, encoding="utf-8") as f:
        manifest = json.load(f)
    apply_pad_pose(args, os.path.dirname(os.path.abspath(mpath)),
                   manifest.get("pad_pose"))
    warn_pad_pose_mismatch(manifest)
    fusion = None
    if args.view_truth:
        truths = []
        for spec in args.view_truth:
            if "=" in spec:
                name, path = spec.split("=", 1)
            else:
                name = os.path.splitext(os.path.basename(spec))[0]
                path = spec
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
            blocks = doc.get("blocks") or []
            truths.append((name, blocks))
            log("  机位 %-5s %2d 块  ← %s" % (name, len(blocks), path))
        merged, fusion = merge_truth_blocks(
            truths, dedup_m=args.merge_dedup_mm / 1000.0)
        log("  合并：%d 块（区界重复丢 %d）"
            % (len(merged), len(fusion["duplicates"])))
        post = {"blocks": merged}
    else:
        with open(args.verify, encoding="utf-8") as f:
            post = json.load(f)
    if not args.view_truth and args.wrist_truth:
        with open(args.wrist_truth, encoding="utf-8") as f:
            wtruth = json.load(f)
        zone = None
        if wtruth.get("zone_m"):
            zone = tuple(float(v) for v in wtruth["zone_m"])
            if args.wrist_zone:
                ask = tuple(float(v) for v in args.wrist_zone.split(","))
                if ask != zone:
                    log("  ⚠ 腕部真值自带的区 %s 与 --wrist-zone %s 不一致，"
                        "按真值里记的区融合" % (list(zone), list(ask)))
        elif args.wrist_zone:
            zone = tuple(float(v) for v in args.wrist_zone.split(","))
        merged, fusion = fuse_post_blocks(
            post.get("blocks") or [], wtruth.get("blocks") or [], zone=zone,
            dedup_m=args.merge_dedup_mm / 1000.0)
        fusion["wrist_truth"] = args.wrist_truth
        log("  融合：腕部 %d 块 + 俯视区外 %d 块（俯视区内丢 %d、重复丢 %d）"
            % (fusion["n_wrist"], fusion["n_topcam_used"],
               len(fusion["topcam_inside_zone_dropped"]),
               len(fusion["topcam_duplicate_dropped"])))
        post = dict(post, blocks=merged)
    colour = None
    if args.colour_burst:
        exp = {str(b["id"]): b for b in manifest.get("final_layout") or []}
        slot = exp.get(str(manifest.get("target")))
        if slot is None:
            log("!! manifest 的 final_layout 里没有目标块，算不出彩色量的窗口")
            return 2
        size = float(slot.get("size") or cm.PAD_PITCH)
        top_z = float(slot["z"]) + size / 2.0
        colour = measure_target_colour(
            args.colour_burst, (float(slot["x"]), float(slot["y"])), top_z,
            search_mm=args.colour_search_mm, camera=args.colour_camera)
        colour["camera"] = args.colour_camera
        if colour.get("ok"):
            rng = colour.get("range_mm") or [0.0, 0.0]
            log("  彩色量（%s）：%d/%d 帧看见，中位 (%.1f, %.1f) mm，极差 x=%.1f "
                "y=%.1f mm，色相 %.0f°，块顶高 %.1f mm"
                % (args.colour_camera,
                   colour["n_frames"], colour["n_frames"] + colour["n_missed"],
                   colour["xy_mm"][0], colour["xy_mm"][1], rng[0], rng[1],
                   colour["hue_deg"], top_z * 1000.0))
        else:
            log("  彩色量：只看见 %d 帧（需 ≥5），漏 %d 帧 —— 判红"
                % (colour["n_frames"], colour["n_missed"]))
    rep = compare_layout(manifest, post["blocks"],
                         tol_untouched_mm=args.untouched_mm,
                         tol_untouched_deg=args.untouched_deg,
                         tol_moved_mm=args.moved_mm,
                         target_colour=colour)
    out_dir = args.out_dir or os.path.dirname(mpath)
    vpath = os.path.join(out_dir, "verify.json")
    with open(vpath, "w", encoding="utf-8") as f:
        json.dump(dict(rep, manifest=mpath, post_truth=args.verify,
                       colour=colour, fusion=fusion), f,
                  ensure_ascii=False, indent=1)
    for c in rep["checks"]:
        log("  %s %s   %s" % ("PASS" if c["ok"] else "FAIL",
                              c["check"], c["detail"]))
    for n in rep.get("notes") or []:
        log("    注：%s" % n)
    for b in rep["blocks"]:
        log("    块 %-3s→%-3s %s  位移 %s mm / 转角 %s° / Δz %s mm"
            % (b["id"], b.get("post_id"), "被搬" if b["moved"] else "未动",
               b.get("d_mm"), b.get("dyaw_deg"), b.get("dz_mm")))
    log("→ %s" % vpath)
    log("结论：%s" % ("PASS" if rep["ok"] else "FAIL"))
    rrc, rmiss = round_gate(out_dir)
    if rrc != 0:
        log("!! 收尾不通过（§20.3）：%s" % "；".join(rmiss))
        return 2
    return 0 if rep["ok"] else 1


def _exec_cmd(board_dir, log_dir, name, extra=""):
    return ('~/precision/run.sh ~/precision/precision_exec.py '
            '--plan %s/%s --go --require-auth "$TOKEN" %s'
            '--log %s/%s.jsonl' % (board_dir, name, extra, log_dir, name))


def _exec_cmd_multi(board_dir, log_dir, names):
    """多份计划**同一个进程**跑完（一个节点、一根串口、一次夹爪自检）。

    2026-11-12：之前每份计划起一个 ``precision_exec``，每次约 2.7 s 的
    import/rclpy/串口/自检开销，S0 一轮 8 份就是 20 多秒纯开销。
    """
    return ('~/precision/run.sh ~/precision/precision_exec.py '
            '--plan %s --go --require-auth "$TOKEN" --log-dir %s'
            % (" ".join("%s/%s" % (board_dir, n) for n in names), log_dir))


def _burst_cmd(camera, out_dir, n=BURST_FRAMES):
    return ('~/precision/run.sh ~/precision/precision_capture.py '
            '--camera %s --burst %d %s' % (camera, n, out_dir))


def run_script_text(plans, board_dir, log_dir, burst_dirs=None,
                    align_burst_dir=None, held_burst_dir=None,
                    token_min_remaining=None):
    """计划序列 → 板端 ``.sh`` 文本。

    ``burst_dirs``：``{机位名: 板上目录}``。某个机位计划执行完就**当场**拍一组
    腕部 burst —— 臂必须停在该机位上拍，不能等全部跑完再补（那时臂在最后一个
    机位）。拍不到就 fail：没有跑后帧就没有判分，不许把「没拍」混成「没跑红」。

    ``align_burst_dir``（E4b 抓前对中）：``align_hover`` 计划跑完停住，拍一组
    **俯视** burst。

    ``held_burst_dir``（E4b 抓后偏心）：目标计划在 ``lift`` 处**切开** ——
    ``--upto lift`` → 拍俯视 burst → ``--from to_slot --holding-block`` 接着跑。
    切开是为了让块**还夹在爪里、臂静止**时拍；跑完整条就没块了。

    ``token_min_remaining``（秒）：非 ``None`` 时在脚本**开头**插一行板端守卫
    —— 令牌剩余不够就把整份脚本判失败（rc 9），一条轨迹都不发。900 s 令牌
    （2026-11-15 v0.8，原 600 s）从 STOP→PLAY 起算，观察/解垫/探针/规划都在
    里面耗；执行起手时剩多少，只有板子自己知道。最后一道仍是
    ``precision_exec --require-auth``（逐条计划前查），
    但它过期发生在**跑到一半**，那时块可能已经夹在爪里 —— 所以先拦一道。

    **合并**（2026-11-12）：相邻、中间不插拍帧的计划写进**同一条**
    ``precision_exec --plan a b c``（一个进程跑完）—— 每次起进程约 2.7 s 的
    import/rclpy/串口/自检开销，S0 一轮光是这个就 20 多秒。合并只在「同一双
    手、同一次授权、同一个不变量」的前提下做：失败仍然 fail-closed 整条停。
    """
    burst_dirs = burst_dirs or {}
    lines = ["#!/bin/bash",
             "# E9 自主移遮挡：按顺序执行本场计划。任何一步 rc≠0 立即停",
             "# （fail-closed，不跳过、不重试）。令牌 = 当次 STOP→PLAY 握手产物。",
             "set -u",
             'TOKEN="$1"',
             '[ -n "${TOKEN:-}" ] || { echo "用法: $0 <token>"; exit 2; }',
             'fail() { echo "!!!!! STOP: $*"; exit 9; }']
    if token_min_remaining is not None:
        lines.append(
            'echo "===== 令牌剩余时间守卫（要求 ≥ %.0f s）"'
            % float(token_min_remaining))
        lines.append(
            '~/precision/run.sh ~/precision/real_motion_authorize.py '
            '--check-token "$TOKEN" --min-remaining %.0f'
            % float(token_min_remaining))
        lines.append(
            'rc=$?')
        lines.append(
            '[ $rc -eq 0 ] || fail "令牌剩余不足 %.0f 秒（rc=$rc）—— 先重做 '
            'STOP→PLAY；本轮 void（aborted_token），不算抓取失败"'
            % float(token_min_remaining))
    # 相邻计划合并成一个 ``precision_exec`` 进程（见 ``_exec_cmd_multi``）。
    # 能吃进同一组的条件很窄：中间**不插拍帧**。机位计划跑完必须就地在那个机位
    # 拍 burst（换机位就拍不到了），被 180° 拆开的计划也不能和人合并 ——
    # 那些切口就是刷子，必须留在两段之间。
    group = []

    def flush():
        if not group:
            return
        if len(group) == 1:
            lines.append('echo "===== %s : exec"' % group[0])
            lines.append(_exec_cmd(board_dir, log_dir, group[0]))
        else:
            lines.append('echo "===== %s : exec（一个进程跑完）"'
                         % " ".join(group))
            lines.append(_exec_cmd_multi(board_dir, log_dir, group))
        lines.append('rc=$?')
        lines.append('[ $rc -eq 0 ] || fail "%s rc=$rc"'
                     % " ".join(group))
        del group[:]

    for p in plans:
        split = (held_burst_dir is not None and p.get("kind") == "target")
        if split:
            flush()
            lines.append('echo "===== %s : exec（lift 处切开拍帧）"' % p["name"])
            lines.append(_exec_cmd(board_dir, log_dir, p["name"],
                                   "--upto lift "))
            lines.append('rc=$?')
            lines.append('[ $rc -eq 0 ] || fail "%s --upto lift rc=$rc"'
                         % p["name"])
            lines.append('sleep 1')
            lines.append('echo "===== 抓后偏心 %s : 俯视 burst %d"'
                         % (p["name"], BURST_FRAMES))
            lines.append(_burst_cmd("topcam", held_burst_dir))
            lines.append('rc=$?')
            lines.append('[ $rc -eq 0 ] || fail "抓后偏心 burst rc=$rc"')
            lines.append('sleep 1')
            lines.append('echo "===== %s : 续跑 to_slot 之后"' % p["name"])
            lines.append(_exec_cmd(board_dir, log_dir, p["name"],
                                   "--from to_slot --holding-block "))
            lines.append('rc=$?')
            lines.append('[ $rc -eq 0 ] || fail "%s --from to_slot rc=$rc"'
                         % p["name"])
            continue
        group.append(p["name"])
        view = p.get("view")
        burst_after = ((p.get("kind") == "view" and view in burst_dirs)
                       or (p.get("kind") == "align_hover" and align_burst_dir))
        if not burst_after:
            continue
        # 这份跑完就地拍帧 ⇒ 它自己收尾：先把之前的合并组跑完，再把这一份
        # 单独跑完，然后才拍。**不能**留在合并组尾巴上 —— 那会让拍帧挪到
        # 别的计划之后（臂已经不在这个机位）。
        _tail = group.pop()
        flush()
        group.append(_tail)
        flush()
        if p.get("kind") == "view" and view in burst_dirs:
            # 判分帧：臂刚停在这个机位上，就地拍（换机位就拍不到了）。
            lines.append('sleep 1')     # 等臂停稳再拍（原来是每条计划后都睡）
            lines.append('echo "===== 判分机位 %s : burst %d"' % (view, BURST_FRAMES))
            lines.append(_burst_cmd("wrist", burst_dirs[view]))
            lines.append('rc=$?')
            lines.append('[ $rc -eq 0 ] || fail "机位 %s burst rc=$rc"' % view)
        else:
            lines.append('sleep 1')
            lines.append('echo "===== 对中悬停 : 俯视 burst %d"' % BURST_FRAMES)
            lines.append(_burst_cmd("topcam", align_burst_dir))
            lines.append('rc=$?')
            lines.append('[ $rc -eq 0 ] || fail "对中 burst rc=$rc"')
    flush()
    lines.append('echo "ALL DONE"')
    return "\n".join(lines) + "\n"


def align_plan_main(ap, args):
    """E4b 一轮：出低悬停计划 ``00-align-hover.json`` + ``align.sh``。

    ``align.sh`` = 执行悬停计划 → **停住**拍一组俯视 burst（``--camera topcam
    --burst 20``）。它同样要当次运动令牌（动臂）。拍完把 burst 拉回开发机，
    用 ``stage4_align.py --burst <dir> --held 否 --block-top-mm ...`` 量
    ``(dx, dy, dyaw)``，再带着 ``--align-json`` 回来出下一轮（或主计划）。
    """
    with open(args.probe_report, encoding="utf-8") as f:
        report = json.load(f)
    scene, truth = load_scene(report, args.scene_index, args.truth)
    attempt = pick_attempt(scene, args.target)
    pad_thick = (cm.PAD_THICK if args.pad_thick is None
                 else float(args.pad_thick))
    actions, _blocks = actions_from_attempt(truth, attempt, pad_thick)
    os.makedirs(args.out_dir, exist_ok=True)
    try:
        recs = apply_align_jsons(truth, actions, args.align_json, attempt["id"])
    except ValueError as e:
        log("!! 对中结果不可用：%s" % e)
        log("   计划**没有**写出 —— 不要执行。")
        return 1
    act = next((a for a in actions
                if a["kind"] == "target" and str(a["block"]) == str(attempt["id"])),
               None)
    if act is None:
        log("!! 动作表里没有目标块 %s" % attempt["id"])
        return 1
    view = None
    if args.topcam_burst:
        try:
            view = topcam_view_gate(args.topcam_burst,
                                    [float(act["from_xy"][0]),
                                     float(act["from_xy"][1]),
                                     float(args.hover_z)],
                                    act["grasp_yaw_deg"])
        except (OSError, ValueError) as e:
            log("!! 视线闸门算不出来（%s）—— 计划**没有**写出。" % e)
            return 1
        log("  视线闸门：分离轴 vs 俯视水平视线 %.1f°（%s）"
            % (view["angle_deg"], "侧向" if view["lateral"] else "**挡视线**"))
        if view["lateral"] is False and not args.allow_blocking_view:
            log("!! 这个抓取朝向会挡住俯视看块顶（%.1f° < %.0f°）⇒ 量不出可信 dy。\n"
                "   先把探针改成 --prefer-closing 1 重跑（让两指转 90° 侧对视线），\n"
                "   或显式 --allow-blocking-view 承担读数无效。计划**没有**写出。"
                % (view["angle_deg"], view["min_deg"]))
            return 1
    else:
        log("  视线闸门：没给 --topcam-burst，**跳过**（量的时候还会再判一次）")
    chain, q_start = q_start_of_args(ap, args)
    try:
        rec, _plan = plan_align_hover(chain, q_start, act, args.out_dir,
                                      hover_z=args.hover_z,
                                      name="00-align-hover-r%d.json"
                                           % (len(recs) + 1))
    except pp.PlanError as e:
        log("!! 对中悬停预检不过：%s" % e)
        log("   计划**没有**写出 —— 不要执行。")
        return 1
    burst = args.align_burst_dir
    if burst is None:
        burst = "$HOME/precision/e9/%s/topcam-align/r%d" % (
            os.path.basename(os.path.abspath(args.out_dir)), len(recs) + 1)
    sh = run_script_text([rec], args.board_dir, args.board_log_dir,
                         align_burst_dir=burst)
    spath = os.path.join(args.out_dir, "align-r%d.sh" % (len(recs) + 1))
    with open(spath, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(spath, 0o755)
    man = {"round": len(recs) + 1, "target": attempt["id"],
           "align_applied": recs, "align_burst_dir": burst,
           "view_gate": view,
           "plan": rec, "q_start": [float(v) for v in q_start]}
    mpath = os.path.join(args.out_dir, "align-r%d.json" % (len(recs) + 1))
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False, indent=1)
    for r in recs:
        log("  已折进修正 r: d=(%.2f, %.2f) mm  dψ=%.2f°  (%s)"
            % (r["dx_mm"], r["dy_mm"], r["dyaw_deg"],
               r.get("reason") or "applied"))
    log("  悬停 tool0 = (%.3f, %.3f, %.3f)  ψ=%6.1f°"
        % (rec["from_xy"][0], rec["from_xy"][1], args.hover_z,
           rec["grasp_yaw_deg"]))
    log("  对中 burst → %s" % burst)
    log("→ %s" % spath)
    log("→ %s" % mpath)
    return 0


def q_start_of_args(ap, args):
    """``--from-frame`` / ``--q-start`` → ``(chain, q_start)``；都没有就报错。"""
    chain = urkin.Chain(args.urdf)
    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame,
                                         "joint_states.json")))
        q_start = [float(js["position"][js["name"].index(n)])
                   for n in chain.move_joints]
    elif args.q_start:
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        ap.error("要给 --from-frame 或 --q-start")
    return chain, q_start


def views_only_main(ap, args):
    """只出**跑前**判分机位计划：``E9_VIEWS`` 每个机位一份 + 各自一组 burst。

    跑前这一趟的目的是两件事：解垫位姿（pad 机位）与量场景真值（pick 机位）、
    确认自由区是空的（free 机位）。位姿与跑后逐位一致，因为两边都走
    ``plan_views``。
    """
    chain, q_start = q_start_of_args(ap, args)
    os.makedirs(args.out_dir, exist_ok=True)
    base = args.board_wrist_burst_dir
    if base is None:
        base = "$HOME/precision/e9/%s/wrist-pre" % os.path.basename(
            os.path.abspath(args.out_dir))
    burst_dirs = {v["name"]: os.path.join(base, v["name"]) for v in E9_VIEWS}
    try:
        plans, _q_end = plan_views(chain, q_start, E9_VIEWS, args.out_dir)
    except pp.PlanError as e:
        log("!! 机位预检不过：%s" % e)
        log("   计划**没有**写出 —— 不要执行。")
        return 1
    sh = run_script_text(plans, args.board_dir, args.board_log_dir, burst_dirs)
    spath = os.path.join(args.out_dir, "pre.sh")
    with open(spath, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(spath, 0o755)
    for p, v in zip(plans, E9_VIEWS):
        log("  %-24s 机位 %-5s 相机 (%0.3f, %0.3f)  区 %s  %d 航点"
            % (p["name"], p["view"], p["slot"][0], p["slot"][1],
               tuple(v["zone"]), p["n_waypoints"]))
    for k in ("pick", "free", "pad"):
        if k in burst_dirs:
            log("  跑前 burst %-5s → %s" % (k, burst_dirs[k]))
    log("→ %s" % spath)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="E9 真机半段：探针方案 → 可执行计划序列（离线）")
    ap.add_argument("--probe-report", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--verify", default=None,
                    help="跑后真值 json；给了就只做「不丢/不掉/不撞」核验，不生成计划")
    ap.add_argument("--manifest", default=None,
                    help="配合 --verify：manifest 路径（缺省 = --out-dir/manifest.json）")
    ap.add_argument("--untouched-mm", type=float, default=5.0,
                    help="没被碰的块允许的最大位移（mm，缺省 5）")
    ap.add_argument("--untouched-deg", type=float, default=3.0,
                    help="没被碰的块允许的最大转角（°，缺省 3）")
    ap.add_argument("--moved-mm", type=float, default=40.0,
                    help="被搬的块离计划落点的最大距离（mm，缺省 40）")
    ap.add_argument("--colour-burst", default=None,
                    help="跑后**垫视角** burst 目录；给了就用彩色顶面质心量目标块"
                         "落格（垫面深度分辨不出 29.5 mm 台阶；缺省 None = 用深度"
                         "真值）。E9 从 2026-11-09 起给腕部的 pad 机位")
    ap.add_argument("--colour-camera", choices=sorted(COLOUR_CAMERAS),
                    default="wrist",
                    help="彩色量用哪一路相机（缺省 wrist；topcam 已退出正式"
                         "流程，只用于复现旧证据，必须同时给 --allow-topcam）")
    ap.add_argument("--allow-topcam", action="store_true",
                    help="显式允许误用俯视相机（只用于诊断/复现旧证据；"
                         "不给它却传 --colour-camera topcam ⇒ 非 0 退出）")
    ap.add_argument("--view-truth", action="append", default=None,
                    metavar="机位=JSON",
                    help="跑后某个**机位**的共识真值（可重复；E9 用 pick/free/pad "
                         "三份）。给了它就不需要 --verify：合并成一份终态")
    ap.add_argument("--views-only", action="store_true",
                    help="只出**跑前机位计划**（E9_VIEWS 三个机位 + 各自 burst），"
                         "不生成抓放序列；写 <out-dir>/pre.sh")
    ap.add_argument("--colour-search-mm", type=float, default=25.0,
                    help="彩色量的搜索窗半径（mm，围绕计划宫格；缺省 25）。"
                         "60 mm 会罩进对角邻块的角（邻块最近角 45.25 mm）")
    ap.add_argument("--wrist-truth", default=None,
                    help="跑后**腕部** burst 的共识真值 json；给了就按区融合"
                         "（区内腕部权威、区外补俯视）。E9 真机判分必须给")
    ap.add_argument("--wrist-zone", default=None, metavar="X0,X1,Y0,Y1",
                    help="腕部真值的覆盖区（米）；缺省取真值里记的 zone_m，"
                         "再缺省用拾取区")
    ap.add_argument("--merge-dedup-mm", type=float, default=40.0,
                    help="融合时判「同一块两边都报」的距离（mm，缺省 40）")
    ap.add_argument("--scene-index", type=int, default=0)
    ap.add_argument("--truth", default=None,
                    help="覆盖报告里记的 truth 路径（报告在板上生成时是板上绝对路径）")
    ap.add_argument("--target", default=None,
                    help="指定目标块 id（默认取这一场第一个成功尝试）")
    ap.add_argument("--from-frame", default=None,
                    help="从帧目录的 joint_states.json 取 q_start")
    ap.add_argument("--q-start", default=None,
                    help="逗号分隔的 6 个关节角（弧度）")
    ap.add_argument("--grid", default=None, metavar="GRID.json",
                    help="本场**实测**垫位姿（stage4_pad_from_burst.py 从跑前"
                         "腕部 pad 机位 burst 解出）。"
                         "缺省 = <out-dir>/grid-pre.json 或 grid.json；"
                         "一个都找不到就停（fail-closed）")
    ap.add_argument("--allow-model-pad", action="store_true",
                    help="只在仿真复现/自检时用：允许退回 gen_scene 模型常量")
    ap.add_argument("--pad-thick", type=float, default=None,
                    help="垫厚（m），目标块放垫上；缺省 = stage4_clear_model.PAD_THICK")
    ap.add_argument("--place-clearance", type=float, default=None,
                    help="放置时块底离表面间隙（m）；缺省 = precision_plan 的 5 mm")
    ap.add_argument("--survey", default=None,
                    help="收尾巡检位 tool0 x,y,z（m）；缺省 %s"
                         % (E9_SURVEY_TOOL0,))
    ap.add_argument("--board-dir", default="$HOME/precision/plans/e9",
                    help="板上计划目录（写进 run.sh）")
    ap.add_argument("--board-log-dir", default="$HOME/precision/logs/e9",
                    help="板上执行日志目录（写进 run.sh）")
    ap.add_argument("--board-wrist-burst-dir", default=None,
                    help="板上腕部 burst 的**根**目录（写进 run.sh）；缺省 "
                         "$HOME/precision/e9/<本场目录名>/wrist-post，每个机位一个子目录")
    ap.add_argument("--align-plan", action="store_true",
                    help="只出 E4b **一轮**低悬停计划（00-align-hover-r<N>.json）"
                         "与 align.sh（执行后停住拍俯视 burst）；不动主序列")
    ap.add_argument("--align-json", action="append", default=None,
                    metavar="ALIGN.json",
                    help="stage4_align.py 的对中结果（可重复，最多 %d 轮）；"
                         "按顺序折进目标块再出计划。与 --align-plan 合用即下一轮"
                         % MAX_ALIGN_ROUNDS)
    ap.add_argument("--align-burst-dir", default=None,
                    help="板上对中俯视 burst 目录（写进 align.sh / run.sh）；"
                         "缺省 $HOME/precision/e9/<本场>/topcam-align/r<N>")
    ap.add_argument("--held-burst-dir", default=None,
                    help="板上**抓后偏心**俯视 burst 目录；给了就把目标计划在 "
                         "lift 处切开（--upto lift → 拍 → --from to_slot "
                         "--holding-block）。缺省不切")
    ap.add_argument("--hover-z", type=float, default=ALIGN_HOVER_Z,
                    help="E4b 低悬停高度（tool0，m；缺省 %.3f）" % ALIGN_HOVER_Z)
    ap.add_argument("--topcam-burst", default=None, metavar="DIR",
                    help="任意一份**俯视**帧目录（只取 tf 里的外参）：出计划时就判"
                         "「两指分离轴会不会挡俯视视线」。不给 ⇒ 跳过（量的时候还会"
                         "再判一次，但那时已经动了臂）")
    ap.add_argument("--allow-blocking-view", action="store_true",
                    help="视线闸门红了也照样出计划（读数已知不可信，只用于诊断）")
    ap.add_argument("--no-wrist-survey", action="store_true",
                    help="不在序列末尾加判分机位（只为复现旧计划；"
                         "没有它就没有跑后判分帧，E9 判分不许用）")
    ap.add_argument("--place-yaw-equiv180", action="store_true",
                    help="放置朝向按实测过不了分支链/单步上限时，改用 180° "
                         "等价写法（只换手指、不换面；方块/矩形块落点相同）。"
                         "缺省不开 ⇒ 照旧 fail-closed")
    args = ap.parse_args(argv)

    # 俯视已退出阶段 4（v0.6）：彩色量默认腕部。显式要用俯视必须再加
    # --allow-topcam；只传 topcam 不给开关 ⇒ fail-closed，防止旧命令静默沿用。
    if args.colour_camera == "topcam" and not args.allow_topcam:
        log("!! --colour-camera topcam 已退出阶段 4 正式流程。")
        log("   正式轮用缺省 wrist；确需复现旧证据请同时给 --allow-topcam。")
        return 2

    if args.verify or args.view_truth:
        return verify_main(args)

    if not args.out_dir:
        ap.error("要给 --out-dir")
    if not args.urdf:
        ap.error("要给 --urdf")

    if args.views_only:
        return views_only_main(ap, args)

    if not args.probe_report:
        ap.error("生成计划要给 --probe-report（核验请用 --verify）")

    if args.align_plan:
        return align_plan_main(ap, args)

    with open(args.probe_report, encoding="utf-8") as f:
        report = json.load(f)
    scene, truth = load_scene(report, args.scene_index, args.truth)
    attempt = pick_attempt(scene, args.target)
    os.makedirs(args.out_dir, exist_ok=True)
    grid_path = apply_pad_pose(args, args.out_dir)
    pad_thick = cm.PAD_THICK if args.pad_thick is None else float(args.pad_thick)
    actions, blocks = actions_from_attempt(truth, attempt, pad_thick)
    try:
        align_recs = apply_align_jsons(truth, actions, args.align_json,
                                       attempt["id"])
    except ValueError as e:
        log("!! 对中结果不可用：%s" % e)
        log("   计划**没有**写出 —— 不要执行。")
        return 1

    chain, q_start = q_start_of_args(ap, args)

    survey = (E9_SURVEY_TOOL0 if args.survey is None else
              tuple(float(v) for v in args.survey.split(",")))
    os.makedirs(args.out_dir, exist_ok=True)

    log("场景 %s:%s，目标块 %s，%d 个动作（清障 %d + 目标 1）"
        % (truth.get("batch"), truth.get("index"), attempt["id"],
           len(actions), len(actions) - 1))
    try:
        plans, final_layout, q_end = plan_sequence(
            chain, q_start, actions, blocks, args.out_dir,
            survey_tool0=survey, place_clearance=args.place_clearance,
            wrist_survey=not args.no_wrist_survey,
            place_yaw_equiv=args.place_yaw_equiv180)
    except pp.PlanError as e:
        log("!! 预检不过：%s" % e)
        log("   计划**没有**写出完整序列 —— 不要执行。")
        return 1

    burst_base = args.board_wrist_burst_dir
    if burst_base is None:
        burst_base = "$HOME/precision/e9/%s/wrist-post" % os.path.basename(
            os.path.abspath(args.out_dir))
    burst_dirs = ({} if args.no_wrist_survey else
                  {v["name"]: os.path.join(burst_base, v["name"])
                   for v in E9_VIEWS})
    manifest = {
        "probe_report": args.probe_report,
        "truth": scene["truth"],
        "batch": truth.get("batch"), "index": truth.get("index"),
        "target": attempt["id"],
        "scene_index": args.scene_index,
        # 逐目标格位（--cells）优先；旧单格探针报告只有 report["cell"]。
        "cell": (attempt.get("cell") if attempt.get("cell") is not None
                 else report.get("cell")),
        "bias_mm": None,          # 单目标入口不补 b̂；整轮编排器传参
        "pad_pose": pad_pose_record(grid_path),
        "pad_thick": pad_thick,
        "survey_tool0": list(survey),
        "gap_check": "probe_verified",
        "place_yaw_equiv180": bool(args.place_yaw_equiv180),
        "actions": actions,
        "plans": plans,
        "q_start": q_start,
        "q_end": q_end,
        "wrist_survey": not args.no_wrist_survey,
        "views": [] if args.no_wrist_survey else [dict(v) for v in E9_VIEWS],
        "view_burst_dirs": burst_dirs,
        "final_layout": [{k: b.get(k) for k in
                          ("id", "x", "y", "z", "yaw", "size")}
                         for b in final_layout],
        "terminal": attempt.get("terminal"),
        "align": align_recs,
        "align_burst_dir": args.align_burst_dir,
        "held_burst_dir": args.held_burst_dir,
    }
    mpath = os.path.join(args.out_dir, "manifest.json")
    with open(mpath, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)

    sh = run_script_text(plans, args.board_dir, args.board_log_dir,
                         burst_dirs=burst_dirs,
                         align_burst_dir=args.align_burst_dir,
                         held_burst_dir=args.held_burst_dir)
    spath = os.path.join(args.out_dir, "run.sh")
    with open(spath, "w", encoding="utf-8") as f:
        f.write(sh)
    os.chmod(spath, 0o755)

    for p in plans:
        if p["kind"] == "view":
            log("  %-28s %-6s 机位 %-5s 相机 (%.3f, %.3f)  %d 航点"
                % (p["name"], p["kind"], p["view"], p["slot"][0],
                   p["slot"][1], p["n_waypoints"]))
            continue
        log("  %-28s %-6s 块 %-3s (%.3f, %.3f) → (%.3f, %.3f)  ψ=%6.1f°  %d 航点"
            % (p["name"], p["kind"], p["block"],
               p["from_xy"][0], p["from_xy"][1], p["slot"][0], p["slot"][1],
               p["grasp_yaw_deg"], p["n_waypoints"]))
        if p.get("place_yaw_equiv180"):
            log("    放置朝向用了 180° 等价写法（ψ_place 与 ψ_grasp 差 180°，"
                "只换手指、不换面；见计划 meta.e9.place_yaw_equiv180）")
    for r in align_recs:
        log("  对中修正 d=(%+.2f, %+.2f) mm  dψ=%+.2f°  %s"
            % (r["dx_mm"], r["dy_mm"], r["dyaw_deg"],
               r.get("reason") or "已应用"))
    for k in ("pick", "free", "pad"):
        if k in burst_dirs:
            log("  跑后 burst %-5s → %s（判分时 --view-truth %s=… 用它）"
                % (k, burst_dirs[k], k))
    if args.held_burst_dir:
        log("  抓后偏心 burst → %s（目标计划在 lift 处切开拍）"
            % args.held_burst_dir)
    log("→ %s" % mpath)
    log("→ %s" % spath)
    return 0


if __name__ == "__main__":
    sys.exit(main())
