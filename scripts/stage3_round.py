#!/usr/bin/env python3
"""阶段 3 一轮堆叠编排：抓帧 → 选块 → 逐层「计划 / 执行 / 立即测量」→ 终态 → 判分。

与阶段 2 的关键差别
──────────────────
1. **每层重新规划**：起点必须取当次实测关节角（`board.joints()`）。一帧只能给
   一条计划的起点；第一块放完臂已经不在抓帧位姿，后面几条计划若共用抓帧的
   q_start，执行器闸门会按「别人动过臂」拒掉（阶段 2 实测最大差 56°）。
2. **放置面每层变**：第 1 层 = 垫面（`grid.json` 的 `T_base_pad[2,3]`），
   第 k≥2 层 = 上一层**实测**顶面（已扣 z 读数偏差）。理论值同时算，差 >3 mm
   就停轮（不静默选一个）。
3. **载运高度**用 `precision_plan.stage3_carry_z`：必须先高过塔顶的放置接近点
   50 mm，再水平移到塔位上方 —— 否则 `to_slot` 会在塔侧横移。
4. **测量只看得见最上层**：每层放好、撤走后**立即**从高度跟踪位姿对抓帧；
   该层指标的唯一来源是这次测量（`metric_source="immediate"`）。终态只复测
   最上层，回答 10 s 稳定性 / 塔是否塌。
5. **测量位姿随塔高上移**：`stage3_survey_tool0` 让相机—塔顶距离恒为 D
   （默认 331 mm，复现阶段 2 的 `stage2close`）。固定 `stage2high` 已知顶面碎、
   yaw 不可信（2026-10-27 B2），固定 `stage2close` 在 5 层时只剩 ~214 mm。
6. **放置偏置补偿**：每层量完记 b̂ = 实测 − **命令**；命令目标 = 期望目标 − b̂
   （b̂ = 先验种子 + 本轮已测各层的均值，`--bias-comp none` 退回纯开环）。
   **种子必需**：第 1 层若不补、第 2 层补满，两层之间就有个 ≈|b̂| 的折角，
   而判分看的正是相对对中。种子优先级 `--bias-seed` > `<run>/bias_estimate.json`。
   这个偏置住「块 ↔ 夹爪」之间，闭环组也消不掉；超 15 mm 按失败停轮。

真机纪律（每次会话重来，不继承上次）
────────────────────────────────────
· `--go` 才动臂；执行器自己在板上核对 PLAYING / JTC active / robot_mode 7；
· 夹爪开合（初始化/闭/开）是**另一件事**的当次授权：急停就绪、两指整个行程内
  无手无遮挡；
· 任何一层执行判红 / 测量缺失 / 多视点不一致 ⇒ **停在原地，不续跑**
  （块可能还在爪里或半放），交给人判读；
· 每轮 `--go` 开工先量塔位：非空 ⇒ **机器人自己逐层拆回拾取区**
  （`stage3_entry.teardown_tower`），塔位空了才开始本轮 —— 轮间不压人拆塔；
· 不 push。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import numpy as np                                                # noqa: E402

import graspnet_candidates as gc                                   # noqa: E402
import precision_plan as pp                                        # noqa: E402
import precision_round as pr                                       # noqa: E402
import stage2_round as s2                                          # noqa: E402
import stage3_measure as m3                                        # noqa: E402
import stage3_score as s3                                          # noqa: E402
import urkin                                                      # noqa: E402

GROUPS = ("open", "closed", "eih", "nudge")
ARMS = ("geometric", "zero", "graspnet")

#: 真正实现了的组。
#: ``eih``（eye-in-hand 第二对照组）＝把一层拆成「抓 → 载运」与「载运 → 放」
#: 两段，在 carry 处按**期望顶高**量一次塔顶再改目标（2026-11-02 落地；
#: 入口③ 三层都量得到，见台账第 207 轮）。
#: ``nudge``（放置后矫正第三对照组）＝放置流程与 ``open`` 完全一样（期望目标
#: 固定格心、开环放），量完之后**只夹不抬**把块推到目标，再量一次。判分与
#: 下一层的表面用矫正后的实测；b̂ 记账仍用矫正前那次（见层循环里的注释）。
IMPLEMENTED_GROUPS = ("open", "closed", "eih", "nudge")


def unsupported_group(group):
    """``group`` 还没实现时返回拒绝原因；实现了返回 ``None``。"""
    if group in IMPLEMENTED_GROUPS:
        return None
    return ("--group %s 不在 %s 里（已知组 %s）。闸门排在抓帧/动臂**之前**，"
            "免得跑成开环组却把结果记成别的组 —— fail-closed，拒绝执行。"
            % (group, IMPLEMENTED_GROUPS, GROUPS))


# ---------------------------------------------------------------- z 标定
def load_zcal(path):
    """读入口实验产出的 z 标定；缺省两侧都按 0（未标定，只作离线/首轮）。"""
    doc = {"pick": 0.0, "tower": 0.0, "source": None, "model": "constant"}
    if not path:
        return doc
    raw = json.load(open(path))
    site = raw.get("by_site") or {}
    doc["tower"] = float(site.get("tower", raw.get("bias_mm", 0.0)))
    doc["pick"] = float(site.get("pick", raw.get("pick_bias_mm", 0.0)))
    doc.update({"source": os.path.abspath(path),
                "model": raw.get("model", "constant"),
                "residual_mm": raw.get("residual_mm"),
                "pass": raw.get("pass")})
    return doc


# ---------------------------------------------------------------- 计划
def _finish_plan(wps, checks, meta, q_start, chain):
    """与 `precision_plan.main` 完全相同的计划 JSON（执行器只认这一种）。"""
    n_before = len(wps)
    wps, added = pp.split_jumps(wps, chain, q_start=q_start)
    if len(checks) != n_before:
        raise RuntimeError("预检记录不完整：%d 条航点只有 %d 条记录"
                           % (n_before, len(checks)))
    return {"version": 2,
            "meta": dict(meta, checks=checks, splits_added=added,
                         q_start_deg=[round(float(np.degrees(v)), 2)
                                      for v in q_start]),
            "q_start": [float(v) for v in q_start], "waypoints": wps}


def layer_plan(chain, q_start, item, blocks, slot_m, surface_m,
               tower_xy_mm=None, tower_top_mm=None, survey_tool0=None,
               carry_z=None, scene=None, place_open_mm=None,
               release_dwell_s=None, tower_half_edge=0.020,
               pick_open_mm=None, place_clearance=None):
    """一层 = 抓 ``item["block"]`` 放到 ``slot_m`` 的 ``surface_m`` 上。"""
    scene = scene or {}
    block = dict(item["block"])
    # build_pick 用 block["h"] 当 ``wz``；这里换成**扣过拾取侧偏差**的值。
    block["h"] = float(item["block_h_used_mm"])
    # ``wz`` 是**块顶离台面**。物理块高另给：台面上的块留空（build_pick 沿用
    # 「块顶 = 块高」，逐字节不变）；塔上的块（拆塔）块顶 91 mm 而块高只有
    # 29.5 mm，漏给会被 build_pick 的护栏当场拒，不会悄悄抓到最底层。
    _bh = item.get("block_true_h_mm")
    block_h_m = None if _bh is None else float(_bh) / 1000.0
    wps, checks, meta = pp.build_stack(
        chain, list(q_start), item["cand"], block, blocks, slot_m, surface_m,
        tower_xy_mm=tower_xy_mm, tower_top_mm=tower_top_mm,
        survey_tool0=survey_tool0, carry_z=carry_z,
        block_h_m=block_h_m,
        tower_half_edge=float(tower_half_edge),
        yaw_grasp_deg=item["yaw_grasp_deg"],
        yaw_place_deg=item["yaw_place_deg"],
        yaw_source=item["yaw_source"],
        pick_open_mm=(pick_open_mm if pick_open_mm is not None
                      else scene.get("pick_open_mm")),
        place_open_mm=(place_open_mm if place_open_mm is not None
                       else scene.get("place_open_mm")),
        grasp_at=scene.get("grasp_at", "block_center"),
        grip_speed_pct=scene.get("grasp_close_speed_pct"),
        release_speed_pct=scene.get("release_open_speed_pct"),
        release_dwell_s=(release_dwell_s if release_dwell_s is not None
                         else scene.get("release_dwell_s", 0.0)),
        place_clearance=place_clearance)
    return _finish_plan(wps, checks, meta, q_start, chain)


def place_plan(chain, q_start, item, slot_m, surface_m, carry_z,
               tower_xy_mm=None, tower_top_mm=None, survey_tool0=None,
               scene=None, place_open_mm=None, release_dwell_s=None,
               tower_half_edge=0.020, place_clearance=None,
               grasp_ecc_mm=None):
    """eye-in-hand 组的**后半段**：已夹着块停在 carry，按新目标放下。

    与 `layer_plan` 唯一的区别是起点（carry 处当次实测关节角）与不含抓取段。
    护栏用**同一套**塔体 box（`pp.tower_box_from`），不许这里少开一项 ——
    目标改过之后最容易出的就是「横移扫过塔顶」。
    """
    scene = scene or {}
    block = dict(item["block"])
    block["h"] = float(item["block_h_used_mm"])
    # 持块包络要用「放置那一刻」块心相对 tool0 的偏置：抓取时的偏心随夹着块
    # 转腕一起转，转角 = yaw_place − yaw_grasp。抓点就是块心时偏心为 0。
    ex, ey = (float(v) / 1000.0 for v in (grasp_ecc_mm or (0.0, 0.0)))
    dyaw = math.radians(float(item["yaw_place_deg"])
                        - float(pp.fold_yaw_180(item["yaw_grasp_deg"])))
    held_off_xy = (ex * math.cos(dyaw) - ey * math.sin(dyaw),
                   ex * math.sin(dyaw) + ey * math.cos(dyaw))
    wps, checks, meta = pp.build_place(
        chain, list(q_start), block, slot_m, surface_m, float(carry_z),
        yaw_place_deg=item["yaw_place_deg"],
        yaw_grasp_deg=item["yaw_grasp_deg"],
        open_mm=(place_open_mm if place_open_mm is not None
                 else scene.get("place_open_mm")),
        survey_tool0=survey_tool0,
        tower_box=pp.tower_box_from(tower_xy_mm, tower_top_mm,
                                    float(tower_half_edge)),
        held_half_edge=float(tower_half_edge),
        held_off_xy=held_off_xy,
        release_dwell_s=(release_dwell_s if release_dwell_s is not None
                         else scene.get("release_dwell_s", 0.0)),
        release_speed_pct=scene.get("release_open_speed_pct"),
        place_clearance=place_clearance)
    meta["kind"] = "place"
    meta["layer_surface_mm"] = round(float(surface_m) * 1000.0, 2)
    return _finish_plan(wps, checks, meta, q_start, chain)


def move_plan(chain, q_start, xyz, name="move"):
    wps, checks, meta = pp.build_move(chain, q_start, xyz, name=name)
    return _finish_plan(wps, checks, meta, q_start, chain)


#: 方块对 90° 自对称 ⇒ 接近朝向的等价写法就这四种，按「离当前腕角最近」试。
_NUDGE_YAW_SHIFTS = (0.0, -90.0, 90.0, 180.0)


def nudge_plan(chain, q_start, block, blocks, shift_xy_mm=(0.0, 0.0),
               dyaw_deg=0.0, surface_m=pp.TABLE_PLANE_Z, yaw_block_deg=0.0,
               open_mm=None, survey_tool0=None, tower_box=None,
               held_half_edge=0.020, lift_mm=0.0):
    """塔顶的放置后矫正计划（契约 §3 第 5 项；`lift_mm` >0 时夹起来再动）。

    与 `layer_plan` 同一个收尾（`_finish_plan`），执行器只认这一种 JSON。
    ``block`` 与 `build_nudge` 同形（``cx``/``cy`` mm、``h`` mm 相对脚下那层）；
    ``yaw_block_deg`` 是**当前 tool0 yaw**（不是实测块 yaw，换算在调用方）。

    朝向的等价写法：方块对 90° 自对称 ⇒ ``yaw_block_deg + k·90°`` 是**同一个
    抓取**。`nudge_delta` 折的是 mod 180（`fold_yaw_180`），块实测落在 84° 附近
    时折完还是 84°，腕 3 要为这份「本该只有 6°」的差异转 199.1°——2026-11-02
    round-04 第 3 层就是这么被 `MAX_JOINT_SWING` 拦下的。这里按「离当前 tool0
    朝向最近」的顺序试等价写法，**只对 `JointSwingError` 重试**：超限平移、
    邻块太近、护栏撞箱换朝向也救不回来，重试只会把真问题掩盖成别的报错。
    """
    cur_yaw = float(chain.pose(q_start)[1][2])

    def _near(shift):
        return abs(pp.fold_yaw_180(float(yaw_block_deg) + shift - cur_yaw))

    last = None
    for shift in sorted(_NUDGE_YAW_SHIFTS, key=_near):
        try:
            wps, checks, meta = pp.build_nudge(
                chain, q_start, block, shift_xy_mm=shift_xy_mm,
                dyaw_deg=dyaw_deg, surface_z=surface_m,
                yaw_block_deg=yaw_block_deg, blocks=blocks,
                open_mm=open_mm, survey_tool0=survey_tool0,
                tower_box=tower_box, held_half_edge=held_half_edge,
                yaw_shift_deg=shift, lift_mm=lift_mm)
        except pp.JointSwingError as exc:
            last = exc
            continue
        return _finish_plan(wps, checks, meta, q_start, chain)
    if last is not None:
        raise last
    raise pp.PlanError("nudge 没有可试的等价朝向（_NUDGE_YAW_SHIFTS 为空）")


# ---------------------------------------------------------------- 后矫正（⑤）
#: 矫正后判据 = 契约 §4.1 的门（≤5 mm / ≤5°），**不新编数**。第 1 层没有下层，
#: 就按「目标 = 格心 + 0°」算；主轮里同样是这两个数，换成「相对下层实测」。
NUDGE_XY_TOL_MM = 5.0
NUDGE_YAW_TOL_DEG = 5.0
#: 死区：实测偏差**两轴都在内**就不碰块（跳过整整一次夹/扭/松）。
#: 依据是 round-05 真机（台账第 222 轮）：夹持循环本身沿 −x 拖 1.07–3.15 mm，
#: 与这一层要扭多少基本无关（L3 只扭 0.005° 却拖了本轮最大的 3.15 mm）。
#: 门宽取 1°，因为彩色 yaw 的逐帧散差 0.2–0.9°、而判分门是 5°；
#: 平移取 1 mm，落在多帧中心散差（0.6–1.2 mm）量级。<0 或 0 = 关闭死区。
NUDGE_DEADBAND_YAW_DEG = 1.0
NUDGE_DEADBAND_XY_MM = 1.0
#: 夹起来再动的最小一步（mm）。2026-11-03 用户拍板（台账第 224 轮）：
#: 贴着下层推时表面摩擦是 push gain 不可预测的大头（0.23–1.55、⟂ 最大 5.06 mm）；
#: 抬起让块脱开摩擦，gain 才可能稳定。取 2 mm —— 足够断开接触，又小到
#: 落下时的落点扰动可控。**0 = 旧行为「只夹不抬」**。
NUDGE_LIFT_MM = 2.0
#: 一轮最多矫正几次（含第一次）。每次都要重新规划 + 退开复测（两视点）。
#: 2 = 一次矫正后看复测，不过门再来一次；再多就开始和不叠层的代价失衡。
NUDGE_MAX_CYCLES = 2
#: 排查「有没有顶动邻块」的半径（mm，离目标格心）。手指半开 50 mm ⇒
#: 半径 90 mm 已把两指行程全包住。
NUDGE_NEAR_MM = 90.0
#: 该半径内、离目标格心比这更近的簇，当成被矫正的那块自己
#: （块半对角线 17.7 mm；矫正量上限 15 mm，两者加起来仍 <25）。
NUDGE_SELF_MM = 25.0


def nudge_delta(measured_xy_mm, measured_yaw_deg, target_xy_mm,
                yaw_sign=1.0, delta_deg=0.0, yaw_quadrant=0,
                target_yaw_deg=0.0):
    """「目标 − 实测」→ 要下的平移 δ(mm) 与 tool0 转角 δ(°)。纯函数。

    ``yaw_sign`` / ``delta_deg`` 与 `stage2_round.yaw_for_arm` **同一套映射**
    （``tool0_yaw = sign·θ + δ``）；目标块朝向 ``target_yaw_deg``（base 系，
    单位 °）对应的 tool0 角就是 ``sign·target_yaw_deg + δ``。

    ``target_yaw_deg`` 默认 0.0 = 旧行为（对齐 ``grid.json`` 的格线方向）。
    主轮从 2026-11-03 起传**下一层实测朝向** —— 判分 `yaw_rel_deg` 本来就是
    「层 N vs 层 N−1」的相对量，把目标也改成相对量，这一招才不依赖垫子/格线，
    换场景能复用（台账第 221 轮）。

    方块按 **90° 周期**折最短角（`stage2_round.signed_err_mod90` 同口径）：
    方块的正面/侧面互换，转 87° 与转 −3° 是同一件事。`fold_yaw_180` 那一次
    折是给夹爪挑一支更短的等价解，不是近似。

    返回里两个角分开写：
    · ``dyaw_deg`` —— **tool0** 要转多少（喂给 `build_nudge`）；
    · ``dyaw_block_deg`` —— 块**应跟着转**多少（= sign·dyaw_deg，用于对账）。

    ``yaw_quadrant``（0–3，`--nudge-yaw-quadrant`）把**接近朝向**再加 k·90°。
    方块对 90° 自对称 ⇒ 夹这一对侧面与夹那一对侧面是同一个合法抓取，所以
    这个数不改任何判据；它只改 tool0 绕 z 的朝向。用途是**方向帧判别**：
    零位移档实测的 ≈5 mm 侧推全落在世界 −y，而那些样本的指轴也都在 75–90°
    （≈世界 +y）——两者重合，分不出「推在夹爪系里」还是「推在世界系里」。
    把指轴转 90°，夹爪系的推就该跟着转到 ±x，世界系的推则仍留在 −y。
    ``dyaw`` 本身按 mod 90 折，绕 90° 转一次不变（块的等价解）。
    """
    sign = float(yaw_sign)
    quad = int(yaw_quadrant) % 4
    dx = float(target_xy_mm[0]) - float(measured_xy_mm[0])
    dy = float(target_xy_mm[1]) - float(measured_xy_mm[1])
    yaw_from = pp.fold_yaw_180(sign * float(measured_yaw_deg) + float(delta_deg)
                               + 90.0 * quad)
    yaw_to = pp.fold_yaw_180(sign * float(target_yaw_deg)
                             + float(delta_deg))
    dyaw = float(s2.signed_err_mod90(yaw_to, yaw_from))
    return {"dx_mm": dx, "dy_mm": dy, "d_mm": math.hypot(dx, dy),
            "dyaw_deg": dyaw, "dyaw_block_deg": sign * dyaw,
            "yaw_from_deg": float(yaw_from), "yaw_to_deg": float(yaw_to)}


def nudge_in_deadband(cmd, axes=("xy", "yaw"), yaw_deg=NUDGE_DEADBAND_YAW_DEG,
                      xy_mm=NUDGE_DEADBAND_XY_MM):
    """**这次要动的那些轴**，实测偏差是否都已在死区内（决定要不要碰块）。纯函数。

    ``cmd`` 是 ``nudge_delta`` 的输出（实测 δ，不是被 ``axes`` 置零后的 applied）。
    ``axes`` 只筛「判哪几个门」，不筛数值 —— 没开的轴本来就不在这次矫正里，
    拿它的偏差挡门等于让死区永不触发：主组只扭转档实测 xy 偏差 1.39–4.94 mm
    （round-05 五层全量），**两轴同判的话一个门都过不去**，加死区等于没加。

    两个门任一 ≤0 = 关闭死区，永不判「已在门内」—— 关闭要能真的退回旧行为，
    不能靠恰好没有偏差。

    依据（台账第 222 轮，round-05 真机）：夹持循环本身沿 −x 拖 1.07–3.15 mm，
    与这层要扭多少基本无关；L3 只扭 0.005° 仍拖了本轮最大的 3.15 mm。
    而不碰块时这笔拖动不存在 ⇒ 偏差已在噪声量级就该跳过。
    """
    y = float(yaw_deg)
    x = float(xy_mm)
    if y <= 0.0 or x <= 0.0:
        return False
    axes = tuple(axes)
    if "yaw" in axes and abs(float(cmd["dyaw_deg"])) > y:
        return False
    if "xy" in axes and float(cmd["d_mm"]) > x:
        return False
    return True


def nearby_blocks(blocks, target_xy_mm, radius_mm=NUDGE_NEAR_MM,
                  exclude_mm=NUDGE_SELF_MM):
    """离目标格心 ``[exclude_mm, radius_mm]`` 内的**其它**块（诊断顶没顶邻块）。

    ``low`` 的簇（连接件 / 台面噪声）不算。
    """
    out = []
    for b in blocks or []:
        if b.get("low"):
            continue
        d = math.hypot(float(b["cx"]) - float(target_xy_mm[0]),
                       float(b["cy"]) - float(target_xy_mm[1]))
        if float(exclude_mm) <= d <= float(radius_mm):
            out.append({"idx": b.get("idx"),
                        "center_mm": [round(float(b["cx"]), 2),
                                      round(float(b["cy"]), 2)],
                        "top_mm": round(float(b.get("top", 0.0)), 2),
                        "dist_mm": round(d, 2)})
    return sorted(out, key=lambda r: r["dist_mm"])


def nudge_push_pred(bias_xy_mm, yaw_deg):
    """预测「闭合那一下」把块推走多少：``E`` 沿指轴的分量再沿指轴返回。

    只保留沿指轴的那一维：垂着指轴的方向上两指没有推力面，块不动
    （12 行实测的 ⟂ 分量 ≤0.34 mm，都在噪声地板里）。

    反向变异：把 ``yaw_deg`` 换成「指轴 = tool0 的 y」、或把分量换成 E 本身，
    都会让 `test_nudge_bias_comp_cancels_predicted_push` 变红。
    """
    ex, ey = float(bias_xy_mm[0]), float(bias_xy_mm[1])
    t = math.radians(float(yaw_deg))
    ax, ay = math.cos(t), math.sin(t)      # 指轴 = tool0 x（数据定的，见上）
    s = ex * ax + ey * ay
    return s * ax, s * ay


def nudge_bias_comp(spec):
    """`--nudge-bias-comp "x,y"` → ``(x, y)`` mm（base 系）；空 ⇒ 不补偿。

    不给默认值：E 只在 4 号格附近 8 mm 内估过（台账第一百八十九轮「未决 1」），
    静默套一个常数比不补偿更危险 —— 换位置若不成立，人会以为是别的地方坏了。
    """
    if spec is None or str(spec).strip() == "":
        return None
    vals = [float(v) for v in str(spec).replace("，", ",").split(",") if v.strip()]
    if len(vals) != 2:
        raise SystemExit("--nudge-bias-comp 要两个数（x,y，mm），拿到 %r" % (spec,))
    return (vals[0], vals[1])


def nudge_axes(args):
    """`--nudge-axes` → 这次要执行的轴元组；``None`` = 不单独指定，按组默认。

    空值（没传）在 `--group nudge` 下等价 ``("xy","yaw")``；传了值就同时
    表示「这轮要做放后矫正」——主组（``--group open``）靠它按需打开这条路径。
    """
    spec = getattr(args, "nudge_axes", None)
    spec = "both" if spec is None else str(spec)
    return {"both": ("xy", "yaw"), "xy": ("xy",), "yaw": ("yaw",)}[spec]


def nudge_enabled(args):
    """这轮要不要做放后矫正：nudge 组一律要；其它组**只在显式给了
    `--nudge-axes` 时**才要（主流程采用「只扭转」那一档，用户 2026-11-02 拍板）。

    显式给值而不是给主组设个默认值：默认打开等于把主组悄悄改版，而主组
    round-01..03 是纯开环跑的，不能让人以为前后同一条件。
    """
    return args.group == "nudge" or getattr(args, "nudge_axes", None) is not None


def applied_nudge_command(cmd, zero=False, bias_xy_mm=None,
                          axes=("xy", "yaw")):
    """这次**真正发给计划**的指令；``cmd`` 是实测 δ，另存一份留作对账。

    ``zero=True``（`--nudge-zero`，诊断用）：平移与转角全置零，只走
    「接近 → 闭合 → 松开 → 退开」。2026-11-01 真机 5 次显示 nudge 自带一个
    ≈5.3 mm、与指令无关的恒定侧推（详见仓外台账第一百八十七轮）；这一档用来
    分辨它是出在这四个动作上，还是出在横移/扭转上。

    ``bias_xy_mm``（`--nudge-bias-comp`）：预补偿。闭合时块会被推走 Δ，
    就在**横移目标**里减掉 Δ（不动接近点），落点回到「实测 + 想要的 δ」。
    这一手只依赖「Δ 可预测」，不假定 E 是臂的定位残差还是测量的固定偏置
    —— 两者对补偿等价（台账第一百八十九轮 §未决）。

    ``axes``（`--nudge-axes`）：只执行列表里的轴，缺的轴指令置零。
    ``("xy","yaw")`` = 原行为；``("yaw",)`` = **只扭转**（主流程采用的那一档，
    2026-11-02 用户拍板）；``("xy",)`` = 只横移（诊断用）。
    """
    keys = ("dx_mm", "dy_mm", "d_mm", "dyaw_deg", "dyaw_block_deg",
            "yaw_from_deg", "yaw_to_deg")
    out = {k: float(cmd[k]) for k in keys if k in cmd}
    if zero:
        for k in ("dx_mm", "dy_mm", "d_mm", "dyaw_deg", "dyaw_block_deg"):
            if k in out:
                out[k] = 0.0
    # ``axes``（`--nudge-axes`）只执行其中几轴：2026-11-02 round-10 实测
    # 扭转增益 0.84–1.16 稳定、横移增益 0.23–1.55 且 ⟂ 分量 0.11–5.06 mm，
    # 用户据此定下「只收扭转那一段」⇒ ``axes=("yaw",)`` 时把平移整段置零。
    # **置零而不是删键**：`nudge_plan` 与 `nudge_row` 都要读 dx/dy/d_mm，
    # 缺键会让 gain/对账那一套拿不到 0；``_requested`` 里另存原始值备查。
    axes = tuple(axes)
    if "xy" not in axes:
        for k in ("dx_mm", "dy_mm", "d_mm"):
            if k in out:
                out[k] = 0.0
    if "yaw" not in axes:
        for k in ("dyaw_deg", "dyaw_block_deg"):
            if k in out:
                out[k] = 0.0
    if bias_xy_mm is not None and "xy" in axes and "dx_mm" in out and "dy_mm" in out:
        px, py = nudge_push_pred(bias_xy_mm, out.get("yaw_from_deg", 0.0))
        out["dx_mm"] -= px
        out["dy_mm"] -= py
        out["d_mm"] = math.hypot(out["dx_mm"], out["dy_mm"])
        out["push_pred_x_mm"] = px
        out["push_pred_y_mm"] = py
    return out


def nudge_row(rep, cmd, before, after, target_xy_mm,
              xy_tol_mm=NUDGE_XY_TOL_MM, yaw_tol_deg=NUDGE_YAW_TOL_DEG,
              axes=("xy", "yaw"), yaw_target_deg=0.0):
    """一次矫正的前后对账：实走 δ、push gain、带转角、终态过不过门。

    ``before`` / ``after`` 是矫正前后的 ``merged``（`stage3_measure` 融合值）。
    **yaw 判据一律走彩色掩码**（契约 §4.2.5），深度 ``yaw_deg`` 只报告：
    两个估计器逐次差 1.43 倍，拿深度当 ``ok`` 会把合格的矫正判红。

    ``yaw_target_deg`` 是这一层的朝向目标（base 系）。默认 0.0 = 对齐格线，
    与判分旧口径一致；主轮现在传**下一层实测朝向**，与 `stage3_score.py`
    的 ``yaw_rel_deg``（层 N vs 层 N−1 的彩色 yaw）同口径。

    ``ok=None``（``reason="no_colour_yaw"``）= 证明不了，转人评 —— 与
    `stage3_measure.pose_consistency` 同一口径。
    """
    def centre(m):
        return [float(m["center_base_mm"][0]), float(m["center_base_mm"][1])]

    def yaw_color(m):
        y = m.get("yaw_color_deg")
        return None if y is None else float(y)

    def err_vs_target(c):
        return math.hypot(float(target_xy_mm[0]) - c[0],
                          float(target_xy_mm[1]) - c[1])

    bc, ac = centre(before), centre(after)
    by, ay = yaw_color(before), yaw_color(after)
    yaw_err0 = None if by is None else float(
        s2.signed_err_mod90(by, float(yaw_target_deg)))
    yaw_err1 = None if ay is None else float(
        s2.signed_err_mod90(ay, float(yaw_target_deg)))
    dyaw_act = (None if (by is None or ay is None)
                else float(s2.signed_err_mod90(ay, by)))
    dyaw_cmd = float(cmd["dyaw_block_deg"])
    yaw_gain = (None if (dyaw_act is None or abs(dyaw_cmd) < 1e-9)
                else dyaw_act / dyaw_cmd)

    # 实走 δ：沿指令方向的有符号投影比（push gain）+ 侧向走偏（粘滑/推力偏）。
    dx, dy = ac[0] - bc[0], ac[1] - bc[1]
    d_act = math.hypot(dx, dy)
    cx, cy = float(cmd["dx_mm"]), float(cmd["dy_mm"])
    d_cmd = math.hypot(cx, cy)
    if d_cmd > 1e-9:
        gain = (dx * cx + dy * cy) / (d_cmd * d_cmd)
        perp = abs(-dx * cy + dy * cx) / d_cmd
    else:
        gain, perp = None, d_act

    e0, e1 = err_vs_target(bc), err_vs_target(ac)
    if yaw_err1 is None:
        ok, reason = None, "no_colour_yaw"
    else:
        # ``axes``：没矫正的那一轴不参与 ``final_ok`` —— 只扭转的档位里
        # 横移本来就没人管，拿它判红等于每层都「still_off」（实测仍会照常
        # 落盘、不停止，但信号会被噪声淹掉）。
        ok = True
        if "xy" in axes:
            ok = ok and e1 <= float(xy_tol_mm)
        if "yaw" in axes:
            ok = ok and abs(yaw_err1) <= float(yaw_tol_deg)
        ok = bool(ok)
        reason = "ok" if ok else "still_off"

    def state(c, e, y, ye):
        return {"center_mm": [round(c[0], 2), round(c[1], 2)],
                "xy_err_mm": round(e, 2), "yaw_color_deg": y,
                "yaw_err_deg": None if ye is None else round(ye, 2)}

    return {"rep": int(rep),
            "axes": list(axes),
            "yaw_target_deg": round(float(yaw_target_deg), 3),
            "commanded": {k: round(float(v), 3) for k, v in cmd.items()},
            "before": dict(state(bc, e0, by, yaw_err0),
                           yaw_depth_deg=before.get("yaw_deg"),
                           top_mm=before.get("top_mm")),
            "after": dict(state(ac, e1, ay, yaw_err1),
                          yaw_depth_deg=after.get("yaw_deg"),
                          top_mm=after.get("top_mm")),
            "push": {"d_actual_mm": round(d_act, 2),
                     "gain_xy": None if gain is None else round(gain, 2),
                     "perp_mm": round(perp, 2),
                     "d_cmd_mm": round(d_cmd, 2)},
            "yaw": {"dyaw_actual_deg": (None if dyaw_act is None
                                        else round(dyaw_act, 2)),
                    "dyaw_expected_deg": round(dyaw_cmd, 2),
                    "gain_yaw": (None if yaw_gain is None
                                 else round(yaw_gain, 2))},
            "delta_xy_mm": round(e0 - e1, 2),
            "final_ok": ok, "reason": reason,
            "tol": {"xy_mm": float(xy_tol_mm),
                    "yaw_deg": float(yaw_tol_deg)}}


def nudge_pose_gate(before, after=None):
    """契约 §4.2.5 的多姿态一致性门 —— 过门返回 ``None``，否则返回人评理由。

    ``after`` 省略（``None``）= 只查矫正前那一次（还没动臂，用来决定夹不夹）；
    ``before`` 传 ``None`` = 只查矫正后那一次（矫正已经做完，只看结果）。
    **两侧都要按「传没传」判断**：把缺失的一侧当成空字典查，会凭空报
    「只有一个视点能测到」，主轮里表现成每层都停轮（2026-11-02 真机 round-10 第一次撞上）。

    ``before`` / ``after`` 是 ``stage3_measure`` 的融合值；这里只读它自己记下的
    两视点差（``center_span_mm`` / ``yaw_color_span_deg``），不重算。

    为什么要单独补这一道（2026-11-01 实测）：nudge 这条路径原来只取融合后的
    ``center_base_mm``，把离散度记进 row.json 就没人看了。零位移诊断档第 1 次
    的两个视点相差 **11.17 mm**（超过契约的 10 mm 门），却仍被记成一条普通的
    ``status: ok``，混进了 ``n_final_ok``。几十次跑下来这种东西会把结论带偏，
    而且从汇总里根本看不出来。

    ``span`` 缺失 = 只有一个视点能测到 ⇒ **证明不了一致**，同样转人评
    （与 ``stage3_measure.pose_consistency`` 的 ``ok=None`` 同一口径）。
    """
    pairs = []
    if before is not None:
        pairs.append(("矫正前", before))
    if after is not None:
        pairs.append(("矫正后", after))
    bad = []
    for name, m in pairs:
        sp = m.get("center_span_mm")
        ys = m.get("yaw_color_span_deg")
        if sp is None or ys is None:
            bad.append("%s只有一个视点能测到（证明不了一致）" % name)
            continue
        if sp > m3.POSE_XY_TOL_MM:
            bad.append("%s两视点差 %.1f mm > %.0f mm"
                       % (name, sp, m3.POSE_XY_TOL_MM))
        if ys > m3.POSE_YAW_TOL_DEG:
            bad.append("%s两视点 yaw 差 %.1f° > %.0f°"
                       % (name, ys, m3.POSE_YAW_TOL_DEG))
    return "；".join(bad) or None


def tower_candidates(meas):
    """塔位测量里 pose1 第一帧的候选簇（**含目标块自己**）。

    nudge 要用它把「被矫正的那块」在 `check_block_gap` 里排除掉；取不到就返回
    空表 —— 调用方退回「只放这一块」的单簇世界，不猜。
    """
    try:
        return list(meas["poses"][0]["frames"][0]["diag"]["candidates"])
    except (KeyError, IndexError, TypeError):
        return []


def nudge_block_ref(merged, meas, tol_mm=10.0):
    """实测塔顶 → ``build_nudge`` 的 ``block`` 与 ``blocks``（含邻块）。

    用**真高**（29.5 mm）而不是实测顶高：nudge 的夹持深度 = 块高 − 18.5 mm，
    高度读出偏差不该混进来。中心取两视点融合值。

    测量中心与帧内最近候选差 > ``tol_mm`` ⇒ 对不上（块可能被撞/量错），
    返回 ``None``，调用方不夹。
    """
    bcx, bcy = (float(v) for v in merged["center_base_mm"])
    cands = tower_candidates(meas)
    idx = -1
    if cands:
        near = min(cands, key=lambda c: math.hypot(c["cx"] - bcx,
                                                   c["cy"] - bcy))
        if math.hypot(near["cx"] - bcx, near["cy"] - bcy) > float(tol_mm):
            return None
        idx = int(near["idx"])
    blocks = [{"idx": int(c["idx"]), "cx": float(c["cx"]),
               "cy": float(c["cy"])} for c in cands]
    if not blocks:
        blocks = [{"idx": idx, "cx": bcx, "cy": bcy}]
    block = {"idx": idx, "cx": bcx, "cy": bcy,
             "top": float(merged.get("top_mm") or m3.BLOCK_TRUE_H_MM),
             "h": m3.BLOCK_TRUE_H_MM}
    return block, blocks


def nudge_after_place(args, board, chain, scene, grid, zcal, cdir, k,
                      round_no, merged, meas, desired_target, surface_mm,
                      surface_m, tower_xy, pair,
                      yaw_target_deg=0.0, yaw_target_src="plan"):
    """第 k 层放好、量完之后：夹住块**先绕块心扭转、再平移**（必要时抬最小一步），
    退开复测；仍超门就按新实测再来一次（默认最多 2 次）。

    ``yaw_target_deg`` / ``yaw_target_src``：这一层的**朝向目标**与它的来源
    （``plan`` = 计划里的绝对朝向；``color`` = **下一层实测的彩色 yaw**）。
    主轮从 2026-11-03 起第 2 层往上都传下一层实测朝向 —— 与判分 `yaw_rel_deg`
    同口径，塔自己跟自己走，不锚在 `grid.json` 那一刻的垫子上。

    返回 ``(row, stop, after_meas)``。``row`` 落进 ``rec["nudge"]``；
    ``stop`` 非空 ＝ 这一层不能再往上叠（块可能还在指间 / 量不住 / 偏得根本
    不是「放歪」）；``after_meas`` 是**最后一次**矫正后的完整测量，一次都没
    矫正（死区）就是 ``None``。逐次明细在 ``row["cycles"]``，顶层
    ``before`` / ``after`` / ``final_ok`` 是「矫正前 → 最后一次复测」的合并口径。

    为什么超限要**停轮**而不是像入口⑤ 那样跳过继续：入口⑤ 每次是一个独立样本，
    跳过只少一条；主轮里这一层已经在塔上了，偏到 15 mm 说明塔本身出了问题，
    往上叠只会放大。
    """
    target_mm = [desired_target[0] * 1000.0, desired_target[1] * 1000.0]
    axes = nudge_axes(args)
    row = {"target_mm": [round(v, 2) for v in target_mm], "axes": list(axes),
           "yaw_target_deg": round(float(yaw_target_deg), 3),
           "yaw_target_source": str(yaw_target_src)}
    yaw_c = merged.get("yaw_color_deg")
    if yaw_c is None or not merged.get("yaw_color_trusted"):
        row.update({"mode": "skipped", "reason": "no_colour_yaw"})
        return row, "nudge_no_colour_yaw", None
    why = nudge_pose_gate(merged, None)
    if why:
        row.update({"mode": "skipped", "reason": "person_review", "detail": why})
        return row, "nudge_pose_review", None

    # 死区（2026-11-03 用户拍板，台账第 222 轮）：这次要动的轴都已在门内就别碰块。
    # 判的是**实测偏差** ``cmd``，但只判 ``axes`` 里那几个门：没开的轴不在这次
    # 矫正的范围内，拿它的偏差挡门会让死区在主组（只扭转）永不触发 —— 实测
    # xy 偏差 1.39–4.94 mm 全过 1 mm 门。
    db_yaw = float(getattr(args, "nudge_deadband_yaw_deg",
                           NUDGE_DEADBAND_YAW_DEG) or 0.0)
    db_xy = float(getattr(args, "nudge_deadband_xy_mm",
                          NUDGE_DEADBAND_XY_MM) or 0.0)
    bias = None            # 第一次真要发指令时才算（死区路径要能完全不碰 args）
    # 迭代（2026-11-03 用户拍板，台账第 224 轮）：每次复测仍超门就按**新实测**
    # 再来一次（默认最多 2 次）。每次一个完整动作段，代价是一次规划 + 两视点复测。
    max_cycles = max(1, int(getattr(args, "nudge_max_cycles",
                                    NUDGE_MAX_CYCLES) or 1))
    lift_mm = float(getattr(args, "nudge_lift_mm", NUDGE_LIFT_MM) or 0.0)
    tower_box = None       # 同上，第一次真要发指令时再算
    cur_merged, cur_meas = merged, meas
    first_merged, first_applied = None, None
    cycles, after = [], None
    stop_why = None
    for cyc in range(1, max_cycles + 1):
        sfx = "" if cyc == 1 else str(cyc)
        # 第 2 次起，用的「矫正前」是上一次的复测：那一份彩色 yaw 可能不可信 ——
        # 没有可信 yaw 就没法算 δ，更不能拿半套数据硬扭（fail-closed，收工）。
        if cyc > 1 and (cur_merged.get("yaw_color_deg") is None
                        or not cur_merged.get("yaw_color_trusted")):
            stop_why = "no_colour_yaw"
            break
        cmd = nudge_delta(cur_merged["center_base_mm"],
                          cur_merged["yaw_color_deg"], target_mm,
                          yaw_sign=float(scene.get("yaw_sign", 1.0)),
                          delta_deg=float(scene.get("finger_axis_delta_deg",
                                                    0.0)),
                          target_yaw_deg=float(yaw_target_deg))
        if nudge_in_deadband(cmd, axes, db_yaw, db_xy):
            if cyc == 1:
                row.update(nudge_row(k, cmd, cur_merged, cur_merged, target_mm,
                                     axes=axes, yaw_target_deg=yaw_target_deg))
                row.update({"mode": "deadband", "reason": "within_deadband",
                            "deadband_yaw_deg": round(db_yaw, 3),
                            "deadband_xy_mm": round(db_xy, 3)})
                return row, None, None
            # 矫正过、复测已在门内 ⇒ 收工（不再白碰一次块）
            stop_why = "deadband"
            break
        if first_merged is None:
            bias = nudge_bias_comp(args.nudge_bias_comp)
            # 塔体 box 的顶取**被矫正这一块的底面**（= 它脚下那层的表面）：
            # 块本来就站在箱顶上，箱再往上一分就是「持块包络进箱」误报。
            # 第 1 层没有塔（``tower_xy`` 为 ``None``）→ 不开护栏。
            tower_box = pp.tower_box_from(tower_xy, surface_mm,
                                          args.tower_half_edge)
        applied = applied_nudge_command(cmd, bias_xy_mm=bias, axes=axes)
        if first_merged is None:
            first_merged, first_applied = cur_merged, applied
            row["commanded"] = {kk: round(float(v), 3)
                                for kk, v in cmd.items()}
            row["applied"] = {kk: round(float(v), 3)
                              for kk, v in applied.items()}
            if bias is not None:
                row["bias_comp_mm"] = [round(float(v), 3) for v in bias]
        if (abs(applied["d_mm"]) > pp.NUDGE_MAX_SHIFT_MM
                or abs(applied["dyaw_deg"]) > pp.NUDGE_MAX_YAW_DEG):
            if cyc == 1:
                row.update({"mode": "skipped", "reason": "out_of_range"})
                return row, "nudge_out_of_range", None
            stop_why = "out_of_range"
            break
        ref = nudge_block_ref(cur_merged, cur_meas)
        if ref is None:
            if cyc == 1:
                row.update({"mode": "skipped", "reason": "centre_mismatch"})
                return row, "nudge_centre_mismatch", None
            stop_why = "centre_mismatch"
            break
        block, blocks = ref
        q_start = s2.chain_q(board.joints(
            os.path.join(cdir, "joints-layer%02d-nudge%s" % (k, sfx))))
        try:
            plan = nudge_plan(chain, q_start, block, blocks,
                              shift_xy_mm=(applied["dx_mm"], applied["dy_mm"]),
                              dyaw_deg=applied["dyaw_deg"],
                              surface_m=surface_m,
                              yaw_block_deg=applied["yaw_from_deg"],
                              open_mm=float(args.nudge_open_mm),
                              survey_tool0=pair[0], tower_box=tower_box,
                              held_half_edge=args.tower_half_edge,
                              lift_mm=lift_mm)
        except (pp.PlanError, RuntimeError, ValueError) as exc:
            if cyc == 1:
                row.update({"mode": "skipped", "reason": "plan_rejected",
                            "detail": str(exc)})
                return row, "nudge_plan_failed", None
            row["stop_detail"] = str(exc)
            stop_why = "plan_failed"
            break
        plan_path = write_plan(
            os.path.join(cdir, "plan-layer%02d-nudge%s.json" % (k, sfx)), plan)
        row["plan"] = os.path.relpath(plan_path, cdir)
        rc = board.exec_plan(
            plan_path,
            os.path.join(cdir, "log-layer%02d-nudge%s.jsonl" % (k, sfx)),
            go=True)
        row["exec_rc"] = rc
        if rc != 0:
            if cyc == 1:
                return row, "nudge_exec_rc=%d" % rc, None
            stop_why = "exec_rc=%d" % rc
            break

        # 计划最后一条航点就停在 pair[0]，不用再走一趟。
        dirs1 = capture_frames(
            board, os.path.join(cdir, "frame-layer%02d-nudge%s-pose1" % (k, sfx)),
            "stage3-r%02d-l%02d-np%d" % (round_no, k, cyc),
            scene.get("terminal_frames", 3))
        rc2 = go_move(board, chain, cdir,
                      "layer%02d-nudge%s-pose2" % (k, sfx), pair[1], args.urdf)
        if rc2 != 0:
            return row, "nudge_survey_pose2_rc=%d" % rc2, None
        dirs2 = capture_frames(
            board, os.path.join(cdir, "frame-layer%02d-nudge%s-pose2" % (k, sfx)),
            "stage3-r%02d-l%02d-np%dp2" % (round_no, k, cyc),
            scene.get("terminal_frames", 3))
        after = m3.measure_layer(
            [("pose1", dirs1), ("pose2", dirs2)],
            grid=grid, window_xy_mm=target_mm, layer=k,
            z_bias_mm=zcal["tower"], block_h_mm=m3.BLOCK_TRUE_H_MM,
            radius_mm=m3.TOWER_RADIUS_MM, metric_source="after_nudge")
        after_merged = after.get("merged") or {}
        if not after_merged.get("found"):
            row["cycles"] = cycles
            if not cycles:
                row.update(nudge_row(k, applied, cur_merged, cur_merged,
                                     target_mm, axes=axes,
                                     yaw_target_deg=yaw_target_deg))
            row.update({"mode": "failed", "reason": "after_missing"})
            return row, "nudge_after_missing", after
        nrow = nudge_row(k, applied, cur_merged, after_merged, target_mm,
                         axes=axes, yaw_target_deg=yaw_target_deg)
        nrow["cycle"] = cyc
        nrow["lift_mm"] = round(lift_mm, 3)
        cycles.append(nrow)
        row["cycles"] = cycles
        row["plan"] = os.path.relpath(plan_path, cdir)
        if (after.get("consistency") or {}).get("ok") is not True:
            row.update(nudge_row(k, applied, first_merged, after_merged,
                                 target_mm, axes=axes,
                                 yaw_target_deg=yaw_target_deg))
            row["mode"] = "twisted" if axes == ("yaw",) else "nudged"
            row["n_cycles"] = cyc
            row["after_pose_reason"] = (after.get("consistency") or {}).get(
                "reason")
            return row, "nudge_after_pose_mismatch", after
        why = nudge_pose_gate(None, after_merged)
        if why:
            row.update(nudge_row(k, applied, first_merged, after_merged,
                                 target_mm, axes=axes,
                                 yaw_target_deg=yaw_target_deg))
            row["mode"] = "twisted" if axes == ("yaw",) else "nudged"
            row["n_cycles"] = cyc
            row["after_pose_reason"] = why
            return row, "nudge_after_review", after
        if nrow["final_ok"] is True:
            stop_why = "ok"
            break
        if nrow["final_ok"] is None:
            stop_why = "no_colour_yaw"
            break
        cur_merged, cur_meas = after_merged, after

    # 顶层对账 = 「矫正前」到「最后一次复测」；逐次明细在 row["cycles"]。
    row.update(nudge_row(k, first_applied, first_merged,
                         (after or {}).get("merged") or merged, target_mm,
                         axes=axes, yaw_target_deg=yaw_target_deg))
    row["mode"] = "twisted" if axes == ("yaw",) else "nudged"
    row["n_cycles"] = len(cycles)
    row["stop_reason"] = stop_why or "budget"
    return row, None, after


def predicted_place(item, slot_m):
    """放置瞬间块心的预测（base，m）：``slot + R(Δyaw)·抓取偏心``。

    ``grasp_at="block_center"`` 时偏心为 0（阶段 3 默认），预测就等于槽心 ——
    这样「释放扰动」量的是纯释放滑移，不掺抓取偏心。
    """
    block = item["block"]
    block_h = float(item["block_h_used_mm"])
    cand = item["cand"]
    if item.get("grasp_at") == "block_center" or cand is None:
        gx, gy = block["cx"] / 1000.0, block["cy"] / 1000.0
    else:
        gx, gy = float(cand["xyz_base"][0]), float(cand["xyz_base"][1])
    ex = block["cx"] / 1000.0 - gx
    ey = block["cy"] / 1000.0 - gy
    dyaw = math.radians(float(item["yaw_place_deg"])
                        - float(pp.fold_yaw_180(item["yaw_grasp_deg"])))
    return [round(slot_m[0] + ex * math.cos(dyaw) - ey * math.sin(dyaw), 5),
            round(slot_m[1] + ex * math.sin(dyaw) + ey * math.cos(dyaw), 5)], block_h


def layer_surface_mm(layer, prev, pad_mm):
    """第 1 层 = 垫面；第 k≥2 层 = 上一层**校正顶高**（= 本层脚下的面）。

    别用 ``surface_est_mm``：那是上一层块**脚下**的面，比本层要放的面低
    整整一块（29.5 mm）—— 拿它当表面就是「第二层按垫面高度放」。
    """
    if int(layer) <= 1:
        return float(pad_mm)
    nxt = (prev or {}).get("next_surface_est_mm")
    if nxt is None:
        raise RuntimeError("第 %d 层没有上一层的校正顶高" % layer)
    return float(nxt)


def closed_loop_yaw(prev_merged):
    """闭环组：上一层实测塔顶 yaw → ``(yaw_deg, 来源)``；判不了给 ``(None, 原因)``。

    **必须和判分同源**（契约 §4.2.5，走 `m3.metric_yaw`，即彩色掩码那一路）。
    拿深度 ``yaw_deg`` 来闭环 = 把 2–5° 的估计器噪声主动拧进塔里：判分那边已经
    改成彩色，控制这边再用深度，两层之间就隔着一个看不出来的朝向偏差。
    彩色判不了 ⇒ 返回 ``None``，调用方**停轮**，不偷偷顶替。
    """
    return m3.metric_yaw(prev_merged or {})


def nudge_yaw_target(k, prev, plan_yaw_deg):
    """nudge 这一层的朝向目标 → ``(yaw_deg, 来源)``；判不了给 ``(None, 原因)``。

    **第 1 层**没有下层可跟 ⇒ 用计划里的绝对朝向（``plan``，即格线方向）。
    **第 2 层往上**一律跟**下一层实测**（`closed_loop_yaw`，彩色掩码那一路，
    与判分同源）—— 塔自己跟自己走。

    为什么不锚 base 系绝对值（2026-11-03 用户拍板，台账第 221 轮）：
    格线纸是松的，垫系 10-27 → 11-03 已转 +3.72°。锚绝对朝向的话，整座塔
    比**印刷格线**每层偏 ≈3.4°，而判分的 `yaw_rel_deg` 是相对量、看不见这一项；
    更要紧的是这招换个场景（没垫子、垫子换了）就废。跟下一层走 = 只用相对量，
    与判分同口径，技巧本身可复用。

    ``prev`` 是上一层最终状态（**矫正后**的 ``merged``）。第 2 层往上缺它 /
    彩色 yaw 不可信 ⇒ ``None``，调用方**停轮**（不拿深度 yaw、也不拿计划值
    偷偷顶替，契约 §4.2.5）。离线预检在到达这里之前就 ``continue`` 了，
    所以「没有 prev」在真机上只可能是上一层没量到 —— 那是失败，不是缺省。
    """
    if int(k) <= 1:
        return float(plan_yaw_deg), "plan"
    pm = (prev or {}).get("merged") or {}
    if not pm.get("found"):
        return None, "no_prev_layer"
    return closed_loop_yaw(pm)


# ---------------------------------------------------------------- 放置偏置补偿
#: 偏置上限（mm）。超过它**不是偏置，是失败**（块掉了 / 没抓住 / 塔被撞了），
#: 按失败停轮，不用它去推下一层 —— 拿一个 30 mm 的「偏置」补偿下一层，
#: 只会把塔越推越歪，而且事后从数据上看不出是哪一步坏的。
BIAS_LIMIT_MM = 15.0

#: eye-in-hand 在 carry 处允许的「小修正」上限（mm）。超过它就不是测量细化，
#: 是塔被撞了 / 上一层其实没量准 —— 停轮查，不拿大数改目标硬放。
EIH_MAX_CORRECTION_MM = 10.0


def bias_from_layer(measured_mm, commanded_mm, limit_mm=BIAS_LIMIT_MM):
    """一层放完后：``实测块顶中心 − 命令目标`` = 这一层的放置偏置 b̂。

    返回 ``(bias, why)``：正常时 ``bias=[bx, by]``（mm）、``why=None``；
    判不了时 ``bias=None``、``why`` 是原因（``measured_missing`` / 超上限）。

    **为什么必须有这一项**：机放的块落地后系统性偏向 base 的 (−x, −y) ——
    入口② open50/dwell0.5 五次均值 (−3.56, −2.56) mm、|dxy| σ 0.84 mm；今天三次
    机放对格心 (−3.4, +0.6)/(−2.8, +0.8)/(−0.9, +0.4)（方位角 170°/164°/156°，稳）。
    这个 offset 住**块与夹爪之间**，不在「目标 → 臂」这条链上 ⇒ 闭环组把下一层
    目标改成「上一层实测中心」也消不掉它。只有显式预补偿（命令 = 期望 − b̂）能压。
    """
    if measured_mm is None or commanded_mm is None:
        return None, "measured_missing"
    bx = float(measured_mm[0]) - float(commanded_mm[0])
    by = float(measured_mm[1]) - float(commanded_mm[1])
    mag = math.hypot(bx, by)
    if not math.isfinite(mag):
        return None, "bias_nonfinite"
    if mag > float(limit_mm):
        return None, ("bias_too_large: 实测比命令偏 %.2f mm（上限 %.1f）"
                      "—— 这不是偏置，是失败" % (mag, limit_mm))
    return [bx, by], None


def applied_bias(bias_hist, mode="adaptive", seed=None):
    """这一层要用的 b̂（mm）：``种子 + 本轮已测各层`` 的均值。

    ``none`` ⇒ ``[0, 0]``（纯开环）。**种子是必需的，不是优化**：第 1 层没有
    本轮历史，不补；第 2 层补满 b̂ ⇒ 第 1↔2 层之间凭空多出一个 ≈|b̂| 的**折角**。
    判分看的是**相对**对中，4 mm 的折角直接把「≤5 mm」吃掉 —— 而物理上
    「相对第 1 层放准第 2 层」本来就要求先知道 b̂，轮内数据给不了，只能给先验。
    """
    if mode != "adaptive":
        return [0.0, 0.0]
    vals = ([list(seed)] if seed else []) + list(bias_hist or [])
    if not vals:
        return [0.0, 0.0]
    n = float(len(vals))
    return [sum(float(b[0]) for b in vals) / n,
            sum(float(b[1]) for b in vals) / n]


def target_with_bias(desired_mm, bias_hist, mode="adaptive", seed=None):
    """``期望目标 − b̂``：命令给臂的目标。``desired`` 仍是要落到的那个点。"""
    b = applied_bias(bias_hist, mode, seed=seed)
    return [float(desired_mm[0]) - b[0], float(desired_mm[1]) - b[1]]


def commanded_target_m(desired_m, bias_hist, mode="adaptive", seed=None):
    """米制包装：臂与计划用 m，b̂ 用 mm —— 换算只在这一行，并由测试把守。"""
    t = target_with_bias([float(v) * 1000.0 for v in desired_m],
                         bias_hist, mode, seed=seed)
    return [t[0] / 1000.0, t[1] / 1000.0]


#: 轮间累积的偏置估计（写在 run 目录下）。下一轮默认拿它当种子 ——
#: 不这样做，每轮第 1 层都只能不补，每轮都会重新造一次那个折角。
BIAS_ESTIMATE_FILE = "bias_estimate.json"


def parse_bias_seed(spec):
    """``"bx,by"``（mm，允许中文逗号）→ ``[bx, by]``；空 ⇒ ``None``。"""
    if spec is None:
        return None
    text = str(spec).strip().replace("，", ",")
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise ValueError("--bias-seed 要两个数（bx,by），实得 %r" % (spec,))
    try:
        return [float(parts[0]), float(parts[1])]
    except ValueError:
        raise ValueError("--bias-seed 不是数：%r" % (spec,))


def load_bias_estimate(path):
    """读轮间估计；坏文件当没有（**不猜**，但也不让一次坏文件炸掉整轮）。"""
    try:
        doc = json.load(open(path))
    except (OSError, ValueError):
        return None
    val = doc.get("bias_mm")
    if not val or len(val) != 2:
        return None
    try:
        return [float(val[0]), float(val[1])]
    except (TypeError, ValueError):
        return None


def write_bias_estimate(rdir, doc):
    """把本轮的 b̂ 并进 ``<run>/bias_estimate.json``：下一轮第 1 层就有先验。

    没有测量（离线预检 / 第一层就停）⇒ 不写，返回 ``None``。
    """
    hist = [h["bias_mm"] for h in (doc.get("bias_history") or [])]
    if not hist:
        return None
    path = os.path.join(rdir, BIAS_ESTIMATE_FILE)
    try:
        prev = json.load(open(path))
    except (OSError, ValueError):
        prev = None
    merged = merge_bias_estimate(prev, hist)
    if merged is None:
        return None
    n = float(len(hist))
    merged["rounds"] = list((prev or {}).get("rounds") or []) + [
        {"round": doc.get("round"), "n": len(hist),
         "bias_mm": [round(sum(float(h[0]) for h in hist) / n, 3),
                     round(sum(float(h[1]) for h in hist) / n, 3)]}]
    with open(path, "w") as fh:
        json.dump(merged, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    return merged


def merge_bias_estimate(prev, new_layers):
    """把本轮的 b̂ 并进轮间估计：``n`` 加权平均，返回落盘用的 dict。"""
    prev = prev or {}
    n0 = int(prev.get("n") or 0)
    vals = [list(map(float, prev.get("bias_mm") or [0.0, 0.0]))] * n0
    vals += [list(map(float, b)) for b in (new_layers or [])]
    if not vals:
        return None
    n = len(vals)
    return {"version": 1, "model": "constant",
            "bias_mm": [round(sum(v[0] for v in vals) / n, 3),
                        round(sum(v[1] for v in vals) / n, 3)],
            "n": n,
            "rounds": list(prev.get("rounds") or [])}


# ---------------------------------------------------------------- 选块
def build_items(analysis, scene, n, arm="geometric", zcal=None):
    """候选池 → 逐层要抓的块（按候选离块心最近排序，不按颜色）。"""
    zcal = zcal or {"pick": 0.0}
    pool = s2.candidate_pool(analysis, scene["pick_zone"])
    chosen, missing, unknown = s2.choose_blocks(pool, order=None, n=n)
    if len(chosen) < n:
        raise RuntimeError("候选池只够 %d 块（要 %d）—— 缺 %s，未定色 %d 块"
                           % (len(chosen), n, missing or "无", len(unknown)))
    items = []
    for k, entry in enumerate(chosen, 1):
        block, cand = entry["block"], entry["best"]
        note = ""
        if arm == "graspnet":
            cand, note = s2.choose_graspnet_cand(entry, block, scene)
            if cand is None:
                raise RuntimeError("块 #%d 没有贴块边的 candidate：%s"
                                   % (block["idx"], note))
        yg, yp, src, why = s2.yaw_for_arm(arm, block, cand, scene)
        # 阶段 3 的块是 29.5 mm 正方体（四面对称）⇒ 抓/放角按 90° 折，
        # 每个角落在 (−45°, 45°] 内，腕的总行程随之最小。见 pp.fold_yaw_90。
        yg, yp = pp.fold_yaw_90(yg), pp.fold_yaw_90(yp)
        h_raw = block.get("h")
        if h_raw is None:
            h_raw = block.get("top")
        items.append({
            "order": k, "block_idx": int(block["idx"]),
            "class": block.get("class"), "arm": arm,
            "block": block, "cand": cand,
            "grasp_at": scene.get("grasp_at", "block_center"),
            "block_h_raw_mm": round(float(h_raw), 3),
            "block_h_used_mm": round(float(h_raw) - float(zcal["pick"]), 3),
            "yaw_grasp_deg": round(float(yg), 3),
            "yaw_place_deg": round(float(yp), 3),
            "yaw_source": src, "yaw_note": note or why,
        })
    return items


# ---------------------------------------------------------------- 板侧动作
def _dump(path, obj):
    json.dump(obj, open(path, "w"), ensure_ascii=False, indent=1)
    return path


def write_plan(path, plan):
    return _dump(path, plan)


def capture_frames(board, out_prefix, tag, n):
    """同一姿态连拍 ``n`` 帧，返回帧目录列表。"""
    dirs = []
    for j in range(max(1, int(n))):
        fd = out_prefix if j == 0 else "%s-%s" % (out_prefix, "bcdefgh"[j - 1])
        board.capture(fd, "%s-%d" % (tag, j + 1))
        dirs.append(fd)
    return dirs


def go_move(board, chain, cdir, tag, xyz, urdf):
    """从当次实测关节角走到 ``xyz``（只动臂）。"""
    js = board.joints(os.path.join(cdir, "joints-%s" % tag))
    q = s2.chain_q(js)
    plan = move_plan(chain, q, xyz, name="survey_" + tag)
    path = write_plan(os.path.join(cdir, "move-%s.json" % tag), plan)
    return board.exec_plan(path, os.path.join(cdir, "move-%s.jsonl" % tag),
                           go=True)


def run_round(args):
    why = unsupported_group(args.group)
    if why:                      # 放在最前：任何动臂动作之前就拒
        print("!! " + why)
        return 2
    try:
        scene = s2.load_scene(args.scene)
    except ValueError as exc:
        print("场景坏：%s" % exc)
        return 2
    grid_path = args.grid or scene.get("grid_json")
    if not grid_path:
        print("缺 grid.json —— 阶段 3 的塔基、垫面、格线方向都要它")
        return 2
    try:
        grid = s2.load_grid(grid_path)
        s2.check_grid_order(scene, grid)
    except ValueError as exc:
        print("网格坏：%s" % exc)
        return 2
    slot_mm = m3.grid_center(grid, args.tower_cell_idx)
    pad_mm = m3.pad_z_mm(grid)
    grid_yaw = float(grid.get("grid_yaw_base_deg") or 0.0)
    zcal = load_zcal(args.zcal)
    chain = urkin.Chain(args.urdf)

    os.makedirs(args.run_dir, exist_ok=True)
    rdir = args.run_dir
    # 偏置先验：命令行 > run 目录里上一轮攒下的估计 > 无。
    # 没有它，每轮第 1 层都只能不补，第 1↔2 层之间就有一个 ≈|b̂| 的折角。
    try:
        b_seed = parse_bias_seed(getattr(args, "bias_seed", None))
    except ValueError as exc:
        print("!! %s" % exc)
        return 2
    b_seed_src = "arg" if b_seed else None
    if b_seed is None:
        b_seed = load_bias_estimate(os.path.join(rdir, BIAS_ESTIMATE_FILE))
        b_seed_src = "estimate" if b_seed else None
    if b_seed and args.bias_comp == "adaptive":
        print("偏置先验 b̂₀=(%+.2f, %+.2f) mm（来源：%s）—— 第 1 层也补偿，"
              "避免 1↔2 层之间那个 ≈|b̂| 的折角"
              % (b_seed[0], b_seed[1],
                 "命令行 --bias-seed" if b_seed_src == "arg"
                 else "run 目录上一轮的估计"))
    round_no = args.round
    if round_no is None:
        if args.measure_only:
            have = [int(d[6:]) for d in os.listdir(rdir)
                    if d.startswith("round-") and d[6:].isdigit()
                    and os.path.exists(os.path.join(rdir, d, "round.json"))]
            round_no = max(have) if have else 1
        else:
            have = [int(d[6:]) for d in os.listdir(rdir)
                    if d.startswith("round-") and d[6:].isdigit()]
            round_no = len(have) + 1
    cdir = os.path.join(rdir, "round-%02d" % round_no)
    os.makedirs(cdir, exist_ok=True)

    if args.measure_only:
        return measure_only(args, chain, cdir, slot_mm, grid, zcal)

    n_layers = int(args.n_layers)
    if n_layers < 1:
        print("--n-layers 至少 1")
        return 2
    board = pr.Board(s2.BOARD_ALIAS, rdir)
    teardown = None
    if args.go:
        print(">>> 需要人动手：确认急停就绪、两指行程内无手无遮挡；"
              "执行器在板上还会核对 PLAYING / JTC / robot_mode。"
              "塔位若还有块，机器人会先自动拆塔回位（不需要人拆）")
        board.push()
        # 开工先把上一轮的塔拆回拾取区（这个动作形态本身也要当次授权 ——
        # 上面那句 + `--go` 的 STOP→PLAY 握手就是；不继承上一轮）。
        import stage3_entry as se
        try:
            teardown = se.teardown_tower(args, board, chain, scene, grid, cdir,
                                         slot_mm, zcal)
        except se.EntryStop as exc:
            print("!! 拆塔停：%s" % exc.why)
            return exc.rc
    else:
        print("离线预演不拆塔：塔位若有块，真跑（--go）时会先自动拆塔回位")

    # ---- 抓帧（拾取巡检位）
    extra_frames = list(args.extra_frames or [])
    if args.frame:
        frame_dir = os.path.abspath(args.frame)
    elif args.go:
        rc, frame_dir = s2.do_capture(board, "stage2pick",
                                      os.path.join(cdir, "frame-pick"),
                                      args.urdf, cdir,
                                      "stage3-pick-%02d" % round_no)
        if rc != 0:
            print("走到拾取巡检位失败（rc=%d）—— 不发抓取计划" % rc)
            return rc
        for j in range(1, max(1, int(scene.get("pick_frames", 1)))):
            fd = os.path.join(cdir, "frame-pick-%s" % "bcdefgh"[j - 1])
            board.capture(fd, "stage3-pick-%02d-%d" % (round_no, j + 1))
            extra_frames.append(fd)
    else:
        print("没给 --frame：离线规划需要一帧拾取区图。"
              "先 --frame DIR，或加 --go 由板子抓。")
        return 2

    try:
        analysis = s2.analyse_frame(frame_dir, cdir)
    except (RuntimeError, OSError, ValueError) as exc:
        print("分析失败：%s" % exc)
        return 2
    if extra_frames:
        extras = []
        for fd in extra_frames:
            f = gc.load_frame(fd)
            eb, _tz = gc.find_blocks(f)
            extras.append(s2.add_pixel_yaw(f, eb))
        s2.fuse_block_yaw(analysis["blocks"], extras)

    try:
        items = build_items(analysis, scene, n_layers, arm=args.arm, zcal=zcal)
    except RuntimeError as exc:
        print("选块失败：%s" % exc)
        return 2
    print("轮内块序（按候选离块心）：%s"
          % ["#%d%s" % (it["block_idx"], it["class"] or "")
             for it in items])

    doc = {
        "version": 2, "kind": "stage3-round", "round": round_no,
        "group": args.group, "arm": args.arm, "n_layers": n_layers,
        "grid_json": os.path.abspath(grid_path),
        "tower_cell_idx": int(args.tower_cell_idx),
        "grid_center_base_mm": [round(v, 2) for v in slot_mm],
        "grid_yaw_base_deg": round(grid_yaw, 3),
        "pad_z_mm": round(pad_mm, 3),
        "block_h_mm": m3.BLOCK_TRUE_H_MM,
        "z_bias_mm": zcal,
        "d_mm": round(float(args.d_mm), 2),
        "scene": {k: scene.get(k) for k in
                  ("pick_zone", "grasp_at", "pick_open_mm", "place_open_mm",
                   "release_dwell_s", "yaw_sign", "finger_axis_delta_deg")},
        "items": [{k: it[k] for k in
                   ("order", "block_idx", "class", "arm", "block_h_raw_mm",
                    "block_h_used_mm", "yaw_grasp_deg", "yaw_place_deg",
                    "yaw_source", "yaw_note")} for it in items],
        # 开工时的自动拆塔（塔位本来就空则 n_removed=0）；明细在
        # `<round>/teardown/teardown.json`。
        "teardown": teardown,
        "layers": [], "terminal": None, "stop_reason": None,
        # 轮内自适应放置偏置补偿：mode=none 时逐字节复现旧的「纯开环」语义。
        "bias_comp": {"mode": args.bias_comp, "limit_mm": BIAS_LIMIT_MM,
                      "seed_mm": (None if b_seed is None
                                  else [round(v, 3) for v in b_seed]),
                      "seed_source": b_seed_src},
        "bias_history": [],
        "human_review": ({"approved": True, "note": args.review_note or ""}
                         if args.review_ok else None),
    }

    prev = None            # 上一层记录（含 merged / surface_est_mm）
    last_pair = None
    prev_q = None
    stop = None
    for k, item in enumerate(items, 1):
        # 每层起点 = 当次实测关节角。阶段 2 的教训：多条计划共用抓帧的
        # q_start，第一块放完后执行器闸门会按「别人动过臂」拒掉（最大差 56°）。
        # 离线预检没有臂：第 1 层读 `--joints`，之后从上一份计划的终点接。
        if args.go:
            js = board.joints(os.path.join(cdir, "joints-layer%02d" % k))
            q_start = s2.chain_q(js)
        elif k == 1:
            if not args.joints:
                print("离线规划要给 --joints <joint_states.json>（起点不能猜）")
                return 2
            q_start = s2.chain_q(json.load(open(args.joints)))
        else:
            q_start = np.asarray(prev_q, dtype=float)
        theory_mm = pad_mm + (k - 1) * m3.BLOCK_TRUE_H_MM
        if args.go:
            surface_mm = layer_surface_mm(k, prev, pad_mm)
            # 理论值同时算：差 >3 mm 说明测量或塔已异常，停轮（不静默选一个）。
            if abs(surface_mm - theory_mm) > 3.0:
                stop = ("surface_theory_mismatch: survey %.1f vs theory %.1f"
                        % (surface_mm, theory_mm))
                print("!! " + stop)
                break
        else:
            # 离线预检没有实测，按理论面逐层规划 —— 只验证运动学/护栏，
            # 数值不进判分（stop_reason=plan_only）。
            surface_mm = theory_mm
        # 物理塔心：下一层的相机要对着它；第 1 层就是格心。
        if prev and prev["merged"].get("found"):
            view_xy = prev["merged"]["center_base_mm"]
        else:
            view_xy = slot_mm
        # 放置目标：主组固定格心；闭环组用上一层实测中心 / yaw。
        # **期望目标**（要落到的点，判分与报告看它）与**命令目标**（给臂的点）
        # 从这一层起分开：命令 = 期望 − b̂，b̂ = 先验种子 + 本轮已测各层的均值。
        # b̂ 住「块 ↔ 夹爪」之间，闭环也消不掉，只能这样显式预补偿。
        desired_target = [slot_mm[0] / 1000.0, slot_mm[1] / 1000.0]
        yaw_place = float(item["yaw_place_deg"])
        yaw_place_src = "plan"          # 主组：目标就是计划里的块朝向
        # eih 组的**初始**目标与闭环组相同（上一层实测中心）——它的增量是
        # 走到 carry 之后再量一次、把目标改掉，见下面的分段执行。
        if (args.group in ("closed", "eih")
                and prev and prev["merged"].get("found")):
            lm = prev["merged"]
            desired_target = [lm["center_base_mm"][0] / 1000.0,
                              lm["center_base_mm"][1] / 1000.0]
            pyaw, pyaw_src = closed_loop_yaw(lm)
            if pyaw is None:
                stop = "closed_yaw_%s（第 %d 层，缺可用的彩色 yaw）" % (
                    pyaw_src, k)
                print("!! " + stop)
                break
            yaw_place = pyaw
            yaw_place_src = pyaw_src
        # nudge 的**朝向目标**：第 1 层跟计划，第 2 层往上跟下一层实测
        # （`nudge_yaw_target`）。在抓帧/发轨迹**之前**就定 —— 下层朝向判不了
        # 就是这一层不该往上叠，停轮，而不是先叠上去再发现矫正不了。
        nudge_yaw = nudge_yaw_src = None
        if nudge_enabled(args):
            nudge_yaw, nudge_yaw_src = nudge_yaw_target(k, prev, yaw_place)
            if nudge_yaw is None:
                stop = "nudge_yaw_%s（第 %d 层，缺可用的下层朝向）" % (
                    nudge_yaw_src, k)
                print("!! " + stop)
                break
        item = dict(item, yaw_place_deg=yaw_place)
        b_applied = applied_bias(doc["bias_history"], args.bias_comp,
                                 seed=b_seed)
        slot_target = commanded_target_m(desired_target, doc["bias_history"],
                                         args.bias_comp, seed=b_seed)
        surface_m = surface_mm / 1000.0
        survey_top_m = surface_m + m3.BLOCK_TRUE_H_MM / 1000.0
        pair = pp.stage3_survey_tool0(
            survey_top_m, [view_xy[0] / 1000.0, view_xy[1] / 1000.0],
            distance=float(args.d_mm) / 1000.0)
        carry_z = pp.stage3_carry_z(item["block_h_used_mm"] / 1000.0,
                                    surface_m)
        tower_xy = (prev["merged"]["center_base_mm"]
                    if prev and prev["merged"].get("found") else None)
        tower_top = (prev["merged"]["top_mm"]
                     if prev and prev["merged"].get("found") else None)
        if not args.go and k >= 2:
            # 离线预检没有实测：用**理论塔**开护栏，否则这一段等于没验。
            tower_xy, tower_top = list(slot_mm), surface_mm
        try:
            plan = layer_plan(
                chain, q_start, item, analysis["blocks"], slot_target,
                surface_m, tower_xy_mm=tower_xy, tower_top_mm=tower_top,
                survey_tool0=pair[0], carry_z=carry_z, scene=scene,
                place_open_mm=args.place_open_mm,
                release_dwell_s=args.release_dwell_s,
                tower_half_edge=args.tower_half_edge)
        except (pp.PlanError, RuntimeError, ValueError) as exc:
            stop = "plan_failed: %s" % exc
            print("!! 第 %d 层计划失败：%s" % (k, exc))
            break
        plan_path = write_plan(os.path.join(cdir, "plan-layer%02d.json" % k),
                               plan)
        meta = plan["meta"]
        pred_xy, _bh = predicted_place(item, slot_target)
        rec = {
            "layer": k, "block_idx": item["block_idx"],
            "class": item.get("class"), "arm": item["arm"],
            "yaw_source": item["yaw_source"],
            "block_h_used_mm": item["block_h_used_mm"],
            "exec_rc": None, "plan": os.path.relpath(plan_path, cdir),
            "planned": {
                "surface_z_mm": round(surface_mm, 2),
                "surface_theory_mm": round(theory_mm, 2),
                "place_z_mm": round(float(meta["place_z"]) * 1000.0, 2),
                "carry_z_mm": round(float(meta["carry_z"]) * 1000.0, 2),
                "survey_tool0": meta["survey_tool0"],
                "tower_box": meta.get("tower_box"),
                "yaw_place_deg": round(yaw_place, 3),
                # 闭环目标来自哪一路 yaw —— 事后能解释每一层的朝向是怎么来的。
                "yaw_place_source": yaw_place_src,
                # 期望（该落在哪）/ 命令（实际让臂去哪）/ 本层用的 b̂。
                # 三者分开记，事后才能分辨「补偿生效」与「塔本来就放歪了」。
                "desired_center_base_mm": [round(v * 1000.0, 2)
                                           for v in desired_target],
                "commanded_center_base_mm": [round(v * 1000.0, 2)
                                             for v in slot_target],
                "applied_bias_mm": [round(v, 3) for v in b_applied],
                "predicted_center_base_mm": [round(v * 1000.0, 2)
                                             for v in pred_xy],
                # 松手瞬间的块顶 = 表面 + 5 mm 放置间隙 + 真高。
                "predicted_top_mm": round(
                    surface_mm + pp.PLACE_CLEARANCE * 1000.0
                    + m3.BLOCK_TRUE_H_MM, 2),
            },
            "measured": None,
            # 本层「实测 − 命令」；下一层的预补偿就用它的历史均值。
            "placement_bias_mm": None,
        }
        doc["layers"].append(rec)
        _dump(os.path.join(cdir, "round.json"), doc)
        print("第 %d 层：块 #%d %s → 命令 (%.1f, %.1f) mm%s，表面 %.2f mm，"
              "carry %.1f mm"
              % (k, item["block_idx"], item.get("class") or "?",
                 slot_target[0] * 1000.0, slot_target[1] * 1000.0,
                 ("（期望 %.1f, %.1f，b̂=(%+.2f, %+.2f)）"
                  % (desired_target[0] * 1000.0, desired_target[1] * 1000.0,
                     b_applied[0], b_applied[1])
                  if any(b_applied) else ""),
                 surface_mm, float(meta["carry_z"]) * 1000.0))

        # ---- eye-in-hand：先走到 carry（`to_slot`），量一次塔顶，改目标再放。
        # 第 1 层没有下层塔顶可量（垫面 2 mm < MIN_H_MM），整层走普通路径。
        if args.group == "eih" and k >= 2:
            cm = None
            if args.go:
                rc_c = board.exec_plan(
                    plan_path,
                    os.path.join(cdir, "log-layer%02d-carry.jsonl" % k),
                    go=True, upto="to_slot")
                rec["exec_rc_carry"] = rc_c
                _dump(os.path.join(cdir, "round.json"), doc)
                if rc_c != 0:
                    stop = "exec_rc_carry=%d" % rc_c
                    print("!! 第 %d 层走到 carry 判红（rc=%d）—— 块还在爪里，"
                          "停在原地" % (k, rc_c))
                    break
                q_carry = s2.chain_q(board.joints(
                    os.path.join(cdir, "joints-layer%02d-carry" % k)))
                dirs_c = capture_frames(
                    board, os.path.join(cdir, "frame-layer%02d-carry" % k),
                    "stage3-r%02d-l%02d-carry" % (round_no, k),
                    scene.get("terminal_frames", 3))
                # 按**期望顶高**认塔顶：手里那块高 74–123 mm，按「最高簇」取
                # 会量到它（入口③ 的旧记录正是这么错的）。
                cm_meas = m3.measure_layer(
                    [("carry", dirs_c)], grid=grid, window_xy_mm=view_xy,
                    layer=k, z_bias_mm=zcal["tower"],
                    block_h_mm=m3.BLOCK_TRUE_H_MM,
                    radius_mm=m3.TOWER_RADIUS_MM, metric_source="carry",
                    expected_top_mm=(prev or {}).get("merged", {}).get("top_mm"))
                cm = cm_meas.get("merged") or {}
                rec["eih"] = {"mode": "measured", "measured": cm_meas}
                _dump(os.path.join(cdir, "round.json"), doc)
                if not cm.get("found"):
                    stop = "eih_measured_missing"
                    print("!! 第 %d 层：carry 处没量到塔顶（按期望顶高）—— "
                          "停轮，不拿旧目标硬放" % k)
                    break
            else:
                # 离线：没有实测。按**初值**走一遍分段，只验运动学与护栏。
                to_slot_wp = next((w for w in plan["waypoints"]
                                   if w["name"] == "to_slot"), None)
                if to_slot_wp is None:
                    stop = "eih_plan_missing_to_slot"
                    print("!! 第 %d 层计划里没有 to_slot —— 无法分段" % k)
                    break
                q_carry = np.asarray(to_slot_wp["q"], dtype=float)
                rec["eih"] = {"mode": "plan_only"}

            if cm is not None:
                nyaw, nyaw_src = m3.metric_yaw(cm)
                if nyaw is None:
                    stop = "eih_yaw_%s" % nyaw_src
                    print("!! 第 %d 层：carry 处彩色 yaw 判不了（%s）—— 停轮，"
                          "不拿深度顶替" % (k, nyaw_src))
                    break
                new_desired = [float(cm["center_base_mm"][0]) / 1000.0,
                               float(cm["center_base_mm"][1]) / 1000.0]
                corr_mm = [(new_desired[0] - desired_target[0]) * 1000.0,
                           (new_desired[1] - desired_target[1]) * 1000.0]
            else:
                nyaw, nyaw_src = yaw_place, "plan"
                new_desired, corr_mm = list(desired_target), [0.0, 0.0]

            if math.hypot(corr_mm[0], corr_mm[1]) > EIH_MAX_CORRECTION_MM:
                stop = "eih_correction_too_large"
                print("!! 第 %d 层：carry 实测比上一层记录偏 %.1f mm（上限 "
                      "%.0f）—— 这不是「小修正」，停轮查塔"
                      % (k, math.hypot(corr_mm[0], corr_mm[1]),
                         EIH_MAX_CORRECTION_MM))
                break

            desired_before_mm = [desired_target[0] * 1000.0,
                                 desired_target[1] * 1000.0]
            desired_target = new_desired
            b_applied = applied_bias(doc["bias_history"], args.bias_comp,
                                     seed=b_seed)
            slot_target = commanded_target_m(desired_target,
                                             doc["bias_history"],
                                             args.bias_comp, seed=b_seed)
            item = dict(item, yaw_place_deg=nyaw)
            try:
                plan = place_plan(
                    chain, q_carry, item, slot_target, surface_m, carry_z,
                    tower_xy_mm=tower_xy, tower_top_mm=tower_top,
                    survey_tool0=pair[0], scene=scene,
                    place_open_mm=args.place_open_mm,
                    release_dwell_s=args.release_dwell_s,
                    tower_half_edge=args.tower_half_edge,
                    # 抓取偏心必须从**抓取那一段**的 meta 来：这里是块心相对
                    # tool0 的偏置，换目标重规划时不能当成 0。
                    grasp_ecc_mm=meta.get("grasp_ecc_mm"))
            except (pp.PlanError, RuntimeError, ValueError) as exc:
                stop = "eih_place_plan_failed: %s" % exc
                print("!! 第 %d 层改目标后的放置计划失败：%s" % (k, exc))
                break
            plan_path = write_plan(
                os.path.join(cdir, "plan-layer%02d-place.json" % k), plan)
            meta = plan["meta"]
            pred_xy, _bh = predicted_place(item, slot_target)
            rec.update({"plan": os.path.relpath(plan_path, cdir),
                        "yaw_source": nyaw_src})
            rec["planned"].update({
                "yaw_place_deg": round(nyaw, 3),
                "yaw_place_source": nyaw_src,
                "desired_center_base_mm": [round(v * 1000.0, 2)
                                           for v in desired_target],
                "commanded_center_base_mm": [round(v * 1000.0, 2)
                                             for v in slot_target],
                "applied_bias_mm": [round(v, 3) for v in b_applied],
                "predicted_center_base_mm": [round(v * 1000.0, 2)
                                             for v in pred_xy],
                "place_z_mm": round(float(meta["place_z"]) * 1000.0, 2),
                "carry_z_mm": round(float(meta["carry_z"]) * 1000.0, 2),
                "surface_z_mm": round(surface_mm, 2),
                "tower_box": meta.get("tower_box")})
            rec["eih"].update({
                # 段 A（抓 → carry，旧目标）也留个指针：改目标后 rec["plan"]
                # 指向的是段 B，事后要能回看这两段到底各让臂走到哪。
                "plan_carry": os.path.relpath(
                    os.path.join(cdir, "plan-layer%02d.json" % k), cdir),
                "correction_mm": [round(corr_mm[0], 3),
                                  round(corr_mm[1], 3)],
                "correction_norm_mm": round(math.hypot(*corr_mm), 3),
                "desired_before_mm": [round(v, 2) for v in desired_before_mm],
                "yaw_place_deg": round(nyaw, 3),
                "yaw_place_source": nyaw_src,
                "single_view": True})
            _dump(os.path.join(cdir, "round.json"), doc)
            print("  eih：目标改为 (%.2f, %.2f) mm（修正 %+.2f, %+.2f mm，"
                  "|%.2f| mm）"
                  % (slot_target[0] * 1000.0, slot_target[1] * 1000.0,
                     corr_mm[0], corr_mm[1], math.hypot(*corr_mm)))

        if not args.go:
            print("只规划不执行（没加 --go）。计划 → %s" % plan_path)
            prev_q = plan["waypoints"][-1]["q"]
            continue

        rc = board.exec_plan(plan_path,
                             os.path.join(cdir, "log-layer%02d.jsonl" % k),
                             go=True)
        rec["exec_rc"] = rc
        _dump(os.path.join(cdir, "round.json"), doc)
        if rc != 0:
            stop = "exec_rc=%d" % rc
            print("!! 第 %d 层执行判红（rc=%d）—— 停在原地，不续跑" % (k, rc))
            break

        # ---- 立即测量：计划最后一条航点已经停在 survey_place（pair[0]）
        dirs1 = capture_frames(board, os.path.join(cdir, "frame-layer%02d-pose1" % k),
                               "stage3-r%02d-l%02d-p1" % (round_no, k),
                               scene.get("terminal_frames", 3))
        rc2 = go_move(board, chain, cdir, "layer%02d-pose2" % k, pair[1],
                      args.urdf)
        if rc2 != 0:
            stop = "survey_pose2_rc=%d" % rc2
            break
        dirs2 = capture_frames(board, os.path.join(cdir, "frame-layer%02d-pose2" % k),
                               "stage3-r%02d-l%02d-p2" % (round_no, k),
                               scene.get("terminal_frames", 3))
        meas = m3.measure_layer(
            [("pose1", dirs1), ("pose2", dirs2)],
            grid=grid, window_xy_mm=view_xy, layer=k,
            z_bias_mm=zcal["tower"],
            block_h_mm=m3.BLOCK_TRUE_H_MM,
            radius_mm=m3.TOWER_RADIUS_MM)
        rec["measured"] = meas
        _dump(os.path.join(cdir, "round.json"), doc)
        merged = meas.get("merged") or {}
        print("  测量：top=%s mm  center=%s mm  yaw=%s°  "
              "一致性=%s"
              % (merged.get("top_mm"), merged.get("center_base_mm"),
                 merged.get("yaw_deg"),
                 (meas.get("consistency") or {}).get("reason")))
        if not merged.get("found"):
            stop = "measured_missing"
            break
        if (meas.get("consistency") or {}).get("ok") is not True:
            stop = "pose_mismatch"
            break
        # ---- nudge 组：量完之后**只夹不抬**把块推到目标，退开再量一次。
        # 判分与下一层的表面用**矫正后**的实测；b̂ 记账仍用矫正前那次 ——
        # b̂ 描述的是「开环放置落哪」，拿矫正后的值记会把补偿器教成「我放得
        # 很准」，下一层不再补，那 4–5 mm 系统偏置会原样回来。这一句以前
        # 只是注释（代码实际吃了矫正后的 merged），2026-11-02 round-10 的
        # `bias_history` 与 `nudge.after` 逐层对得上，证实吃错了，这次改到
        # 真的用矫正前那次。
        bias_merged = merged
        if nudge_enabled(args):
            nrow, nstop, after = nudge_after_place(
                args, board, chain, scene, grid, zcal, cdir, k, round_no,
                merged, meas, desired_target, surface_mm, surface_m,
                tower_xy, pair,
                yaw_target_deg=nudge_yaw, yaw_target_src=nudge_yaw_src)
            rec["nudge"] = nrow
            rec["measured_before_nudge"] = meas
            _dump(os.path.join(cdir, "round.json"), doc)
            if nstop:
                stop = nstop
                print("!! 第 %d 层 nudge 停在：%s —— 不续跑，交给人判读"
                      % (k, nstop))
                break
            if after is None:
                # 死区：整段夹/扭/松都没发 ⇒ 没有「矫正后」这一趟测量，
                # `meas` / `merged` / `rec["measured"]` 保持矫正前那一份
                # （块没被碰过，两者本来就该相同）。
                print("  nudge：死区（yaw %.2f° ≤ %.2f°、xy %.2f mm ≤ %.2f mm）"
                      "—— 未碰块"
                      % (float(nrow["commanded"]["dyaw_deg"]),
                         float(nrow["deadband_yaw_deg"]),
                         float(nrow["commanded"]["d_mm"]),
                         float(nrow["deadband_xy_mm"])))
            else:
                after_merged = after.get("merged") or {}
                print("  nudge：误差 %.2f → %.2f mm，yaw %.2f → %.2f°，残余 %s"
                      % (nrow["before"]["xy_err_mm"],
                         nrow["after"]["xy_err_mm"],
                         float("nan") if nrow["before"]["yaw_err_deg"] is None
                         else nrow["before"]["yaw_err_deg"],
                         float("nan") if nrow["after"]["yaw_err_deg"] is None
                         else nrow["after"]["yaw_err_deg"],
                         "过门" if nrow["final_ok"] else nrow["reason"]))
                rec["measured"] = after
                meas, merged = after, after_merged
        # 这一层落地偏了多少 —— 下一层用它预补偿。超上限按失败停轮，
        # 不拿一个「块掉了 / 没抓住」的大数去把下一层推得更歪。
        bias, bwhy = bias_from_layer(
            bias_merged.get("center_base_mm"),
            [slot_target[0] * 1000.0, slot_target[1] * 1000.0])
        rec["placement_bias_mm"] = (None if bias is None
                                    else [round(v, 3) for v in bias])
        if bwhy:
            stop = "layer%02d_%s" % (k, bwhy)
            print("!! 第 %d 层：%s" % (k, bwhy))
            _dump(os.path.join(cdir, "round.json"), doc)
            break
        doc["bias_history"].append({"layer": k,
                                    "bias_mm": [round(v, 3) for v in bias]})
        print("  放置偏置 b̂=(%+.2f, %+.2f) mm —— %s"
              % (bias[0], bias[1],
                 "下一层按历史均值预补偿" if args.bias_comp == "adaptive"
                 else "补偿关闭（--bias-comp none），只记账"))
        prev = {"merged": merged,
                "surface_est_mm": meas.get("surface_est_mm"),
                "next_surface_est_mm": meas.get("next_surface_est_mm")}
        last_pair = pair
        _dump(os.path.join(cdir, "round.json"), doc)

    # 本轮的 b̂ 并进 run 目录的估计：下一轮（含第 1 层）直接有先验。
    merged_est = write_bias_estimate(rdir, doc)
    if merged_est:
        print("偏置估计更新：b̂=(%+.2f, %+.2f) mm，n=%d → %s"
              % (merged_est["bias_mm"][0], merged_est["bias_mm"][1],
                 merged_est["n"],
                 os.path.join(rdir, BIAS_ESTIMATE_FILE)))

    # ---- 离线预检到此为止：只验证计划能不能生成，不判分（没有测量）。
    if not args.go:
        doc["stop_reason"] = "plan_only"
        _dump(os.path.join(cdir, "round.json"), doc)
        print("离线预检：%d 层计划全部生成（未执行、未判分）" % len(doc["layers"]))
        return 0

    # ---- 终态：静置后复测最上层（只回答顶层有没有动 / 塔是否塌）
    if stop is None and last_pair is not None:
        wait_s = float(args.terminal_wait_s)
        print("终态：静置 %.1f s 后复测最上层" % wait_s)
        time.sleep(wait_s)
        rc = go_move(board, chain, cdir, "terminal-pose1", last_pair[0],
                     args.urdf)
        dirs1 = dirs2 = None
        if rc == 0:
            dirs1 = capture_frames(
                board, os.path.join(cdir, "frame-terminal-pose1"),
                "stage3-r%02d-term-p1" % round_no,
                scene.get("terminal_frames", 3))
            rc = go_move(board, chain, cdir, "terminal-pose2", last_pair[1],
                         args.urdf)
        if rc == 0:
            dirs2 = capture_frames(
                board, os.path.join(cdir, "frame-terminal-pose2"),
                "stage3-r%02d-term-p2" % round_no,
                scene.get("terminal_frames", 3))
            term = m3.measure_layer(
                [("pose1", dirs1), ("pose2", dirs2)],
                grid=grid,
                window_xy_mm=(prev or {}).get("merged", {}).get(
                    "center_base_mm") or slot_mm,
                layer=n_layers, z_bias_mm=zcal["tower"],
                block_h_mm=m3.BLOCK_TRUE_H_MM, metric_source="terminal")
            doc["terminal"] = {"layer": n_layers, "wait_s": wait_s,
                               "measured": term}
        else:
            stop = "terminal_move_rc=%d" % rc
    else:
        if stop is None:
            stop = "no_layers_executed"

    doc["stop_reason"] = stop
    _dump(os.path.join(cdir, "round.json"), doc)

    res = s3.score_round(doc, thresholds=scene.get("stage3_thresholds"))
    paths = s3.write_outputs(res, os.path.join(cdir, "score"))
    print("\n判分：verdict=%s（%s）" % (res["verdict"], res["reason"] or "-"))
    print("  数据 → %s" % paths["json"])
    if stop:
        print("!! 本轮停在：%s —— 交给人判读后再决定；**不自动续跑**" % stop)
    return s3.exit_code(res)


def measure_only(args, chain, cdir, slot_mm, grid, zcal):
    """只做终态复测/判分（不抓不放）。某层判红停在原地后补测量用。"""
    path = os.path.join(cdir, "round.json")
    if not os.path.exists(path):
        print("--measure-only 要读 %s" % path)
        return 2
    doc = json.load(open(path))
    layers = doc.get("layers") or []
    if not layers:
        print("!! round.json 里没有层记录 —— 没有可复测的东西")
        return 2
    last = layers[-1]
    m = (last.get("measured") or {}).get("merged") or {}
    if not m.get("found"):
        print("!! 最上层没有 immediate 测量 —— 终态也复测不了（契约：不赌终态能补）")
        return 2
    if not args.go:
        print("--measure-only 也会动臂（走到两个高度跟踪位姿）。"
              "确认现场后加 --go；**每次都要当次授权**。")
        return 2
    board = pr.Board(s2.BOARD_ALIAS, os.path.dirname(cdir))
    board.push()
    view_xy = m["center_base_mm"]
    top_m = float(m["top_mm"]) / 1000.0
    pair = pp.stage3_survey_tool0(top_m,
                                  [view_xy[0] / 1000.0, view_xy[1] / 1000.0],
                                  distance=float(args.d_mm) / 1000.0)
    rc = go_move(board, chain, cdir, "terminal-pose1", pair[0], args.urdf)
    if rc != 0:
        return rc
    dirs1 = capture_frames(board, os.path.join(cdir, "frame-terminal-pose1"),
                           "stage3-mo-p1", 3)
    rc = go_move(board, chain, cdir, "terminal-pose2", pair[1], args.urdf)
    if rc != 0:
        return rc
    dirs2 = capture_frames(board, os.path.join(cdir, "frame-terminal-pose2"),
                           "stage3-mo-p2", 3)
    doc["terminal"] = {"layer": int(last["layer"]),
                       "wait_s": float(args.terminal_wait_s),
                       "measured": m3.measure_layer(
                           [("pose1", dirs1), ("pose2", dirs2)],
                           grid=grid, window_xy_mm=view_xy,
                           layer=int(last["layer"]),
                           z_bias_mm=zcal["tower"],
                           block_h_mm=m3.BLOCK_TRUE_H_MM,
                           metric_source="terminal")}
    _dump(path, doc)
    res = s3.score_round(doc)
    paths = s3.write_outputs(res, os.path.join(cdir, "score"))
    print("判分：verdict=%s（%s）→ %s"
          % (res["verdict"], res["reason"] or "-", paths["json"]))
    return s3.exit_code(res)


def main(argv=None):
    ap = argparse.ArgumentParser(description="阶段 3 一轮堆叠")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--grid", default=None, help="覆盖 scene.grid_json")
    ap.add_argument("--frame", default=None, help="复用已抓的拾取区帧")
    ap.add_argument("--joints", default=None,
                    help="离线规划（不加 --go）时的起点 joint_states.json；"
                         "第 1 层用它，之后从上一层计划终点接")
    ap.add_argument("--extra-frame", dest="extra_frames", action="append",
                    default=None, help="同姿态另一帧（yaw 融合），可重复")
    ap.add_argument("--round", type=int, default=None)
    ap.add_argument("--n-layers", type=int, required=True)
    ap.add_argument("--group", choices=GROUPS, default="open")
    ap.add_argument("--arm", choices=ARMS, default="geometric")
    ap.add_argument("--zcal", default=None,
                    help="入口实验产出的 z 标定 JSON（缺省两侧偏差都按 0）")
    ap.add_argument("--d-mm", type=float, default=pp.STAGE3_SURVEY_DEFAULT_D
                    * 1000.0, help="高度跟踪位姿的相机—塔顶距离（mm）")
    ap.add_argument("--tower-cell-idx", type=int, default=m3.TOWER_CELL_IDX)
    ap.add_argument("--tower-half-edge", type=float, default=0.020,
                    help="塔体 box 半边长（m）：块半宽 + 余量")
    ap.add_argument("--place-open-mm", type=float, default=None)
    ap.add_argument("--release-dwell-s", type=float, default=None)
    ap.add_argument("--bias-comp", choices=("adaptive", "none"), default="adaptive",
                    help="轮内自适应放置偏置补偿（默认 adaptive）：每层量完把"
                         "「实测 − 命令」记进 bias_history，下一层命令目标 = "
                         "期望目标 − 历史均值。none = 旧的纯开环语义，只记不补。")
    ap.add_argument("--bias-seed", default=None,
                    help="偏置先验 bx,by（mm），第 1 层也补；不给就吃 "
                         "<run-dir>/bias_estimate.json（上一轮攒下的）")
    ap.add_argument("--nudge-open-mm", type=float, default=40.0,
                    help="nudge 组矫正时的张开宽度（mm）：够跨进两侧、又不把"
                         "行程撑到邻块上")
    ap.add_argument("--nudge-bias-comp", default=None,
                    help="nudge 组：预扣「闭合那一下」的推力，格式 x,y（mm，"
                         "base 系，例 -2.05,-4.78）。不给就不补偿 —— E 只在"
                         "4 号格附近估过，换位置若不成立，人容易以为别处坏了")
    ap.add_argument("--nudge-axes", choices=("both", "xy", "yaw"), default=None,
                    help="放后矫正执行哪几轴：both=横移+扭转（nudge 组默认）、"
                         "yaw=只扭转（主流程采用）、xy=只横移（诊断）。"
                         "**主组给这个值 = 打开放后矫正**；不给 = 纯开环，"
                         "与 round-01..03 同条件。")
    ap.add_argument("--nudge-deadband-yaw-deg", type=float,
                    default=NUDGE_DEADBAND_YAW_DEG,
                    help="死区：实测 yaw 偏差 ≤ 此值**且** xy 偏差 ≤ xy 门时，"
                         "跳过整个夹/扭/松（默认 1.0°；0 = 关闭）")
    ap.add_argument("--nudge-deadband-xy-mm", type=float,
                    default=NUDGE_DEADBAND_XY_MM,
                    help="死区：实测 xy 偏差 ≤ 此值时算已对齐（默认 1.0 mm；"
                         "0 = 关闭）")
    ap.add_argument("--nudge-lift-mm", type=float, default=NUDGE_LIFT_MM,
                    help="夹住后先抬这么多再扭/移（默认 2.0 mm；"
                         "0 = 旧行为「只夹不抬」）")
    ap.add_argument("--nudge-max-cycles", type=int, default=NUDGE_MAX_CYCLES,
                    help="一轮最多矫正几次（含第一次，默认 2）：每次复测仍超门"
                         "就按新实测再来一次")
    ap.add_argument("--terminal-wait-s", type=float, default=10.0)
    ap.add_argument("--review-ok", action="store_true",
                    help="人眼复核已通过（逐层数值之外必须由人确认）")
    ap.add_argument("--review-note", default=None)
    ap.add_argument("--measure-only", action="store_true",
                    help="只做终态复测/判分，不抓不放")
    ap.add_argument("--go", action="store_true",
                    help="真跑；**每次都要当次授权**，闸门在 precision_exec")
    args = ap.parse_args(argv)
    return run_round(args)


if __name__ == "__main__":
    sys.exit(main())
