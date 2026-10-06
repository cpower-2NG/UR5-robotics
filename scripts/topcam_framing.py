#!/usr/bin/env python3
"""**俯视相机的取景收敛器** —— 「还差多少像素 / 等效于升几厘米」。

为什么需要它
============
A2 跑完才发现放置区**根本不在画面里**（8 个槽投影到 v∈[-130,-5]，全在上边界
之外），而现场唯一的调整手段是「搬三脚架 → 起一次栈 → 看一次日志」——
一次迭代十几分钟，而且只能靠试。这个工具把那个环缩成**一条只读命令**：
把抓取区与 8 个槽（连同方块的像素外延）投回画面，直接说还差多少像素、
等效于升高几厘米、或把 place_slots 沿 y 挪几厘米。

它复用 `draw_zone_on_frame.py` 的投影核（`compose_base_optical` / `project`），
**不另写一份投影** —— 那份代码里含 `camera_link → 光学系` 那 15 mm 平移，
是本项目修过的一个真 bug（见该文件的注释）：两份投影就是两次机会把它漏掉。

灵敏度是**数值微分算出来的**，不是抄的常数
==========================================
「升高 1 cm = 下移 7.2 px」是**旧机位**实测的，机位一挪就过期 —— 而本工具
存在的全部意义就是「机位要挪的时候用」。所以这里把当前外参按 1 cm / 1° 动一下、
重新投影、量平均像素位移。**换机位后它自己跟着变**。

⚠️ 前提：这份外参**必须与相机当前的实际位姿一致**。相机若已被物理挪动而外参
没重标，下面所有数字都只是「按旧外参推的」—— 输出里会显式提示。要预估一次
**计划中**的挪动，用 `--sim-*`。

退出码
======
    0  取景满足要求（全部项目在画面内，且离边界 ≥ --margin）
    1  取景不满足（有项目出框或余量不足）—— 这是**正常结论**，不是故障
    2  环境/参数不满足：读不到外参、帧语义对不上、参数非法

**只读**：读外参 / 内参 / 站点 yaml，打印。不碰相机、不碰机械臂、不发轨迹。

跑法
====
    # 看当前取景
    python3 scripts/topcam_framing.py \\
        --extrinsics docs/calib/extrinsics-2026-10-02.json

    # 「我打算再升高 10 cm」—— 先看结果再动手
    python3 scripts/topcam_framing.py --extrinsics ... --sim-raise-cm 10
"""

import argparse
import json
import math
import os
import sys

sys.dont_write_bytecode = True

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import draw_zone_on_frame as dzf          # noqa: E402  （要先把 scripts/ 放进 sys.path）

# ---- 真机实测常数（都有出处，别改着好玩；要改先改出处）----
# 内参：1280×720，docs/待执行命令.md ⓪ 步（2026-10-02 抓帧实测，与记录逐位相同）
REAL_K = (917.62, 916.43, 649.55, 344.91)
REAL_W, REAL_H = 1280, 720
# 方块：**真机 28.5 mm 高**（不是仿真的 40）—— docs/待执行命令.md 丙-2 与
# 现场 yaml 的 place_z=0.0285 两处互印。脚印取 40 mm（夹爪行程与实测框宽）
REAL_BLOCK_H = 0.0285
REAL_BLOCK_W = 0.040
# 站点：docs/现场yaml草稿-真机.yaml（同一份配置）
REAL_SLOTS = [
    (0.50, -0.10), (0.60, -0.10),
    (0.50, -0.20), (0.60, -0.20),
    (0.50, -0.30), (0.60, -0.30),
    (0.50, -0.40), (0.60, -0.40),
]
REAL_PICK = [0.40, 0.60, 0.0, 0.45, 0.0, 0.0485]
DEFAULT_SITE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "docs", "现场yaml草稿-真机.yaml")


# ----------------------------------------------------------------------
# 站点配置
# ----------------------------------------------------------------------
def load_site(path):
    """读站点 yaml 的 place_slots / pick_zone。读不到就返回 ``(None, None)``。

    ⚠️ 用的是**纯文本解析的 yaml**（PyYAML），与规划器读的是同一份文件 ——
    但规划器的真机值有一部分被 `pipeline.launch.py` 无条件覆盖（见那份 yaml
    开头的警告）。这里只读 place_slots / pick_zone 两项，它们**不被覆盖**
    （实测逐字一致），所以离线读这份文件是安全的。
    """
    try:
        import yaml
    except ImportError:
        return None, None
    try:
        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        p = doc["grasp_planner"]["ros__parameters"]
        flat = [float(v) for v in p["place_slots"]]
        slots = list(zip(flat[0::2], flat[1::2]))
        pick = [float(v) for v in p["pick_zone"]]
        return slots, pick
    except (OSError, KeyError, TypeError, ValueError):
        return None, None


# ----------------------------------------------------------------------
# 位姿：模拟一次「计划中的挪动」
# ----------------------------------------------------------------------
def apply_move(R, t, raise_cm=0.0, move_x_cm=0.0, move_y_cm=0.0, tilt_deg=0.0):
    """把外参按**平移 + 俯仰**动一下，返回新的 ``(R, t)``。

    俯仰的转轴：**水平、且垂直于光轴的水平投影** —— 也就是三脚架云台俯仰的
    那根轴。绕着它转 = 「抬头/低头」，不改变相机的水平朝向。
    """
    R = np.asarray(R, dtype=float)
    t = np.asarray(t, dtype=float).copy()
    t[0] += move_x_cm / 100.0
    t[1] += move_y_cm / 100.0
    t[2] += raise_cm / 100.0
    if tilt_deg:
        axis = _pitch_axis(R)
        if axis is not None:
            R = _rot_axis(axis, math.radians(tilt_deg)) @ R
    return R, t


def _pitch_axis(R):
    """水平、垂直于光轴水平投影的单位向量；光轴竖直时返回 ``None``。"""
    a = np.asarray(R, dtype=float)[:, 2]
    h = np.array([a[0], a[1], 0.0])
    n = np.linalg.norm(h)
    if n < 1e-9:
        return None
    h /= n
    return np.array([-h[1], h[0], 0.0])


def _rot_axis(axis, ang):
    """罗德里格斯：绕单位轴 ``axis`` 转 ``ang`` 弧度的 3×3 旋转。"""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


# ----------------------------------------------------------------------
# 取景：把「必须看得见的东西」投回画面
# ----------------------------------------------------------------------
def framing_items(slots, pick, block_w, block_h):
    """→ ``[(名字, [必须落在画面内的 3D 点...]), ...]``。

    槽位给的是**方块 8 个角**而不是中心：要判的是「整块看得见」，而方块在
    斜视下有高度——只投中心的话，「中心在框里、方块顶边出框」查不出来，
    而检测器报的正是**可见顶面**。
    """
    items = []
    hw = block_w / 2.0
    for i, (sx, sy) in enumerate(slots):
        pts = []
        for dx in (-hw, hw):
            for dy in (-hw, hw):
                for z in (0.0, block_h):
                    pts.append((sx + dx, sy + dy, z))
        items.append((f"槽位[{i}] ({sx:.2f},{sy:.2f})", pts))
    if pick:
        xmin, xmax, ymin, ymax = pick[:4]
        items.append(("抓取区四角", [(xmin, ymin, 0.0), (xmax, ymin, 0.0),
                                    (xmax, ymax, 0.0), (xmin, ymax, 0.0)]))
    return items


def frame_one(R, t, K, items):
    """→ ``[(名字, u_min, u_max, v_min, v_max, 背后有点吗), ...]``。"""
    out = []
    for name, pts in items:
        px = [dzf.project(R, t, p, K) for p in pts]
        behind = any(p is None for p in px)
        uv = [p for p in px if p is not None]
        if not uv:
            out.append((name, None, None, None, None, True))
            continue
        us = [p[0] for p in uv]
        vs = [p[1] for p in uv]
        out.append((name, min(us), max(us), min(vs), max(vs), behind))
    return out


def check_one(row, W, H, margin):
    """一行 → ``(是否合格, 需要沿 v 挪多少 px, 说明)``。

    「需要挪多少」的**符号约定：正数 = 这一项要往画面下方挪（v 变大）** ——
    与灵敏度同号，所以「升高几厘米」直接就是 ``need_v / s_raise``，
    现场不用再想一次方向。上下都出框时取绝对值大的那一边（更急的那一边）。

    边界用 ``min(v)`` / ``max(v)``：方块有高度，可见的是**整个投影框**，
    只看中心的话「中心在框里、方块顶边出框」查不出来。
    """
    name, umin, umax, vmin, vmax, behind = row
    if umin is None:
        return False, None, "**整个在相机背后**（投影拿不到像素 —— 外参或站点配置错了）"
    if behind:
        return False, None, "有角点在相机背后 —— 同一个区不可能一半在前一半在后，检查外参"
    notes = []
    cands = []
    # ⚠️「出框」与「余量不足」是**两种症状**，文案要分开：前者是方块被画面
    #    切掉了（检测器根本报不出它），后者是看得见、但离边界太近（检测框
    #    一抖就出框）。都写成「出框」会让人以为方块已经不见了。
    if vmin < margin:                       # 上边界（画面顶部 = 远处）
        if vmin < 0:
            notes.append(f"上边出框 {-vmin:.0f} px（另需 {margin:.0f} px 余量）")
        else:
            notes.append(f"上边余量只剩 {vmin:.0f} px（要求 ≥{margin:.0f}）")
        cands.append(margin - vmin)
    if vmax > H - 1 - margin:               # 下边界（画面底部 = 近处）
        over = vmax - (H - 1)
        if over > 0:
            notes.append(f"下边出框 {over:.0f} px（另需 {margin:.0f} px 余量）")
        else:
            notes.append(f"下边余量只剩 {H - 1 - vmax:.0f} px（要求 ≥{margin:.0f}）")
        cands.append(-(vmax - (H - 1 - margin)))
    if umin < margin:
        notes.append(f"左边出框 {margin - umin:.0f} px")
    if umax > W - 1 - margin:
        notes.append(f"右边出框 {umax - (W - 1 - margin):.0f} px")
    need_v = max(cands, key=abs) if cands else None
    return (not notes), need_v, ("；".join(notes) if notes else "在框内")


# ----------------------------------------------------------------------
# 灵敏度：数值微分
# ----------------------------------------------------------------------
def sensitivity(R, t, K, probes, kind, step=1.0):
    """把外参按 ``kind`` 动 ``step``（cm 或 deg），返回**平均 v 位移 px/单位**。

    取平均而不是某个点：斜视下不同位置的像素灵敏度本来就不同（近处大、
    远处小），报一个点会让人以为哪个位置都一样。均值的含义是「整体挪多少」。

    ⚠️ 返回值要**除以 step** 才是「每 cm / 每度」的速率。不除的话，调用方
    传 step=2 就会拿到两倍的斜率，而它看上去仍像一个速率 —— 静默差一倍。
    默认步长 1.0 时两者数值恰好相同，所以这个错可以长期不暴露
    （本文件的测试里有一条「步长减半、速率不变」专门钉它）。
    """
    if kind == "raise":
        R2, t2 = apply_move(R, t, raise_cm=step)
    elif kind == "move_y":
        R2, t2 = apply_move(R, t, move_y_cm=step)
    elif kind == "move_x":
        R2, t2 = apply_move(R, t, move_x_cm=step)
    elif kind == "tilt":
        R2, t2 = apply_move(R, t, tilt_deg=step)
    else:
        raise ValueError(f"不认识的 kind={kind!r}")
    d = []
    for p in probes:
        p1 = dzf.project(R, t, p, K)
        p2 = dzf.project(R2, t2, p, K)
        if p1 is None or p2 is None:
            continue
        d.append(p2[1] - p1[1])
    if not d or not step:
        return None
    return sum(d) / len(d) / step


def item_sensitivity(R, t, K, pts, kind, step=1.0):
    """**这一项自己**的灵敏度（不是全场平均）。

    ⚠️ 换算「还差几厘米」必须用项自己的：斜视下远处与近处的像素灵敏度差得
    不少（远处小、近处大），拿全场平均去除，最急的那一项会**系统性偏小**
    —— 于是现场照做一次、发现还差一点、再挪一次。那正是本工具要消掉的循环。
    """
    if kind == "slots_y":
        return sensitivity_slots_y(R, t, K, pts, step)
    return sensitivity(R, t, K, pts, kind, step)


def slot_clashes(slots, pick, dy_cm):
    """把 ``place_slots`` 整体沿 y 挪 ``dy_cm`` 之后，有没有槽位落进抓取区。

    返回**落在抓取区里的槽位下标列表**（空 = 不冲突）；没有 ``pick`` 时返回
    ``[]``（无约束）。

    为什么这条要单独成函数并被测试钉住：「挪槽位」是本工具给出的**零成本**建议
    （纯配置改动），而它有个硬边界 —— 槽位挪进抓取区，规划器启动时判
    「槽位与抓取区重叠」**硬错误**（那会让机械臂去抓自己刚放下的方块）。
    不挡住的话，这条建议会把人直接送进一个启动失败，而那条错误信息看起来
    与取景毫无关系。

    ⚠️ 单位：``dy_cm`` 是 **cm**（灵敏度是 px/cm），槽位坐标是 **m** ——
    这里差一个 100 倍，护栏就永远不触发（这一条被测试的 M5 变异钉着）。
    """
    if not pick:
        return []
    dy_m = dy_cm / 100.0
    return [i for i, s in enumerate(slots)
            if pick[0] <= s[0] <= pick[1]
            and pick[2] <= s[1] + dy_m <= pick[3]]


def sensitivity_slots_y(R, t, K, slot_probes, step_cm=1.0):
    """``place_slots`` 整体沿 y 移 1 cm → 平均 v 位移 px。

    **另算一份而不是复用相机那一路**：动相机是动所有点（含抓取区），动槽位
    只动槽位。两者量级接近但不等，而现场要据此填配置 —— 差几毫米就是
    「改完还得再试一次」。相机 y 的灵敏度**不能**直接拿来当槽位用。

    与 :func:`sensitivity` 同一约定：返回值是**每 cm** 的速率（要除以 step）。
    """
    d = []
    for p in slot_probes:
        p1 = dzf.project(R, t, p, K)
        p2 = dzf.project(R, t, (p[0], p[1] + step_cm / 100.0, p[2]), K)
        if p1 is None or p2 is None:
            continue
        d.append(p2[1] - p1[1])
    if not d or not step_cm:
        return None
    return sum(d) / len(d) / step_cm


def main():
    ap = argparse.ArgumentParser(
        description="俯视相机取景收敛器（只读：读外参/内参/站点 yaml，打印）")
    ap.add_argument("--extrinsics", required=True, help="外参 json（如 docs/calib/extrinsics-2026-10-02.json）")
    ap.add_argument("--info", default=None, help="含 fx,fy,cx,cy[,width,height] 的 json；不给就用文档实测的 1280×720 内参")
    ap.add_argument("--site", default=DEFAULT_SITE, help="站点 yaml（读 place_slots/pick_zone）")
    ap.add_argument("--block-size", type=float, default=REAL_BLOCK_W, help="方块脚印边长（m）")
    ap.add_argument("--block-height", type=float, default=REAL_BLOCK_H, help="方块高（m）")
    ap.add_argument("--margin", type=float, default=8.0,
                    help="要求的边界余量（px）。默认 8 —— 够吸收检测框的抖动")
    ap.add_argument("--expect-normal-deg", type=float, default=None,
                    help="光轴与台面法线的期望夹角；不给就只做宽判据（20~70°）")
    # ---- 模拟一次计划中的挪动 ----
    ap.add_argument("--sim-raise-cm", type=float, default=0.0)
    ap.add_argument("--sim-move-x-cm", type=float, default=0.0)
    ap.add_argument("--sim-move-y-cm", type=float, default=0.0)
    ap.add_argument("--sim-tilt-deg", type=float, default=0.0)
    ap.add_argument("--json", default=None, help="把结果另存一份 json（可选）")
    args = ap.parse_args()

    # ---- 外参 ----
    try:
        with open(args.extrinsics, encoding="utf-8") as fh:
            doc = json.load(fh)
        R, t = dzf.compose_base_optical(doc)
    except (OSError, ValueError, KeyError) as e:
        print(f"!! 读不了外参 {args.extrinsics}：{e}")
        return 2

    # ---- 内参 ----
    if args.info:
        try:
            with open(args.info, encoding="utf-8") as fh:
                info = json.load(fh)
            K = (float(info["fx"]), float(info["fy"]),
                 float(info["cx"]), float(info["cy"]))
            W = int(info.get("width", REAL_W))
            H = int(info.get("height", REAL_H))
        except (OSError, KeyError, TypeError, ValueError) as e:
            print(f"!! 读不了内参 {args.info}：{e}")
            return 2
    else:
        K, W, H = REAL_K, REAL_W, REAL_H
        print(f"内参：用文档实测的 {W}×{H}（fx={K[0]} fy={K[1]} "
              f"cx={K[2]} cy={K[3]}，出处 docs/待执行命令.md ⓪ 步）")

    # ---- 站点 ----
    slots, pick = load_site(args.site)
    src = args.site
    if slots is None:
        slots, pick = REAL_SLOTS, REAL_PICK
        src = "（站点 yaml 读不到，退回内置的真机值）"
    else:
        src = args.site
    print(f"站点：{len(slots)} 个槽位，抓取区 "
          f"{'有' if pick else '无'}   ← {src}")

    # ---- 帧语义自检：**这条不过，后面画什么都不算数** ----
    # 用宽判据而不是拿 35.9° 硬卡：机位一挪这个角就会变（本轮抬高就变了
    # 2.25°），硬卡会把「取景变了」误报成「帧关系搞错了」。宽判据抓的是
    # 真正的危险——差一个 90° 级固定旋转那种错。
    to_normal, to_plane = dzf.axis_angles(R)
    if to_normal is None:
        print("!! 光轴方向算不出来（外参的旋转是退化的）")
        return 2
    lo, hi = 20.0, 70.0
    if args.expect_normal_deg is not None:
        lo = hi = None
        if abs(to_normal - args.expect_normal_deg) > 5.0:
            print(f"!! 光轴与台面法线 {to_normal:.2f}°，与期望 "
                  f"{args.expect_normal_deg:.1f}° 差得多于 5° —— 帧关系或机位不对")
            return 2
    elif not (lo <= to_normal <= hi):
        print(f"!! 光轴与台面法线 {to_normal:.2f}°，落在宽判据 "
              f"[{lo:.0f},{hi:.0f}]° 之外 —— **帧关系搞错了**（差一个 90° 级旋转？）")
        return 2
    print(f"★ 光轴 vs 台面法线 {to_normal:.2f}°（vs 台面 {to_plane:.2f}°）"
          f" —— 帧语义自检通过")
    if abs(args.sim_raise_cm) + abs(args.sim_move_x_cm) + abs(args.sim_move_y_cm) \
            + abs(args.sim_tilt_deg) > 0:
        print("⚠️ 这份外参是**旧机位**的：若相机已被物理挪动过，下面的「当前」"
              "一列只是按旧外参推的。挪动**后**必须重标外参再跑一次。")

    # ---- 当前 / 模拟 ----
    items = framing_items(slots, pick, args.block_size, args.block_height)
    probes = [p for _, pts in items for p in pts]
    # 槽位单独一份：`place_slots` 沿 y 挪的灵敏度只能对**槽位那些点**算
    # （抓取区不动），见 sensitivity_slots_y。
    slot_probes = [p for _, pts in items[:len(slots)] for p in pts]

    # ---- 先验收一个**物理前提**：抓取区必须看得见 ----
    # 这不是额外假设，是已经发生过的事实：A1/A2 每一次成功抓取的目标都来自
    # 抓取区，而抓取靠的就是这台相机的 /targets_3d。所以「抓取区投出画面」只能
    # 意味着**这份外参与当前机位不一致**（拿错了留档文件、或相机被挪过没重标）。
    # 此时下面关于放置区的任何结论都没有意义 ⇒ 退出码 **2**（环境不满足），
    # 不是 1（取景不满足）。
    #
    # 实测价值：这条能把 docs/calib 里两份外参**立刻**分开。
    #   extrinsics-2026-10-02.json 的 x/y 是镜像的（t=(-0.347,-0.869,…)），
    #   它把 8 个槽投到 u∈[1492,1922]、抓取区投到 u∈[1234,1568] —— 整块区域
    #   跑到画面右侧 300~650 px 外面，而且**它的「光轴 vs 台面法线」照样是
    #   35.95°**（那个角对「绕 z 转 180°」是不变的），所以只靠那条不变式
    #   分不出来，必须靠这条物理前提。
    if pick:
        pick_rows = frame_one(R, t, K, [items[-1]])
        ok_pick, _, why_pick = check_one(pick_rows[0], W, H, args.margin)
        if not ok_pick:
            print()
            print("!! 抓取区**自己**就不在画面里 —— " + why_pick)
            print("   而抓取区一直是可见的（A1/A2 的每一次成功抓取都发生在那里）。")
            print("   ⇒ **这份外参与当前机位不一致**（拿错留档文件？相机挪过没重标？）。")
            print("   先解决这个再谈取景 —— 否则下面算出来的只是按错外参推的。")
            return 2

    def report(R_, t_, title):
        rows = frame_one(R_, t_, K, items)
        print()
        print("=" * 78)
        print(f"{title}     （v 越小 = 越靠画面上边；画面 {W}×{H}，"
              f"要求离边界 ≥ {args.margin:.0f} px）")
        print("=" * 78)
        print(f"{'项目':<24} {'u 范围':>17} {'v 范围':>17}  结论")
        bad = []
        for row in rows:
            name, umin, umax, vmin, vmax, behind = row
            ok, need_v, why = check_one(row, W, H, args.margin)
            if umin is None:
                ur = vr = "——"
            else:
                ur = f"[{umin:7.1f},{umax:7.1f}]"
                vr = f"[{vmin:7.1f},{vmax:7.1f}]"
            mark = "✅" if ok else "❌"
            print(f"{name:<24} {ur:>17} {vr:>17}  {mark} {why}")
            if not ok:
                bad.append((name, need_v, why))
        return bad

    bad = report(R, t, "当前取景（按这份外参）")

    # 灵敏度一律**现算**（见 sensitivity 的文档）—— 不写死 7.2 px/cm 那类
    # 旧机位的常数：机位一挪就过期，而本工具存在的意义正是机位要挪的时候用。
    sens = {k: sensitivity(R, t, K, probes, k)
            for k in ("raise", "move_y", "move_x", "tilt")}
    s_slot = sensitivity_slots_y(R, t, K, slot_probes)

    if bad:
        print()
        print("-" * 78)
        print("灵敏度（对**当前外参**数值微分得到，换机位后自动跟着变）")
        print("-" * 78)
        print(f"  升高 1 cm       → v 平均 {sens['raise']:+6.2f} px")
        print(f"  相机 y +1 cm    → v 平均 {sens['move_y']:+6.2f} px")
        print(f"  相机 x +1 cm    → v 平均 {sens['move_x']:+6.2f} px")
        print(f"  抬高俯仰 1°     → v 平均 {sens['tilt']:+6.2f} px")
        if s_slot is not None:
            print(f"  place_slots y +1 cm → v 平均 {s_slot:+6.2f} px"
                  "（只影响槽位，不动抓取区）")

        needed = [(name, nv) for name, nv, _ in bad if nv is not None]
        if not needed:
            print("\n  出框的原因**不在 v 方向**（见上面每一行的说明）："
                  "抬高与俯仰都救不了，那是横向机位问题。")
        else:
            name, nv = max(needed, key=lambda kv: abs(kv[1]))
            signs = {1 if v > 0 else -1 for _, v in needed}
            pts_of = {n: p for n, p in items}[name]
            s_item_raise = item_sensitivity(R, t, K, pts_of, "raise")
            s_item_slot = (item_sensitivity(R, t, K, pts_of, "slots_y")
                           if name.startswith("槽位") else None)
            print()
            print(f"  最急的是 {name}：v 还需 **{nv:+.0f} px**"
                  f"（它自己的灵敏度 {s_item_raise:+.2f} px/cm，"
                  f"不是全场平均 {sens['raise']:+.2f}）")
            if len(signs) > 1:
                print("  ⚠️ 有几项**方向相反**（有的要往下挪、有的要往上）—— "
                      "单纯升高/平移救不了：那说明视场张角不够，"
                      "得把机位挪远一点（两区都变小）或把槽位铺得更紧凑。")
            if s_item_raise:
                print(f"      ⇒ 升高 **{nv/s_item_raise:+.1f} cm**")
            if sens["move_y"]:
                print(f"      或 相机沿 y **{nv/sens['move_y']:+.1f} cm**")
            if s_item_slot:
                d_slot = nv / s_item_slot
                print(f"      或 place_slots 沿 y **{d_slot:+.1f} cm**"
                      "（纯配置改动，不动任何硬件）")
                # 「纯配置、零成本」有个硬边界：槽位挪进抓取区，规划器**启动就拒**
                # （`_validate_zones` 里「槽位与抓取区重叠」是硬错误 —— 那会让
                # 机械臂去抓自己刚放下的方块）。不挡住的话，这条建议会把人直接
                # 送进一个启动失败，而且失败信息看起来与取景毫无关系。
                if pick:
                    clash = slot_clashes(slots, pick, d_slot)
                    if clash:
                        print(f"         ❌ 但这一挪会把槽位 {clash} 推进抓取区 "
                              f"(y∈[{pick[2]:.2f},{pick[3]:.2f}]) —— "
                              "规划器启动时判「槽位与抓取区重叠」硬错误走不通，"
                              "请改用升高。")

            # ---- 自动复核这个建议：按算出来的量模拟一遍，重新判一次 ----
            # 这一步是**自证**：线性外推在几十厘米上有几个百分点的误差，
            # 与其让现场跑一次才发现差一点，不如这里先算第二遍。
            if s_item_raise:
                cm = nv / s_item_raise
                # ⚠️ `apply_move` 把「升高」写进**返回的 t**（不是 R）——
                #    复核时必须用返回的那一份，否则升高量被丢掉、复核结果
                #    与挪动前一模一样（这个 bug 真发生过：它让「自证」这一步
                #    永远报「仍然不合格」，看上去像外推不准，其实是没挪）。
                R_chk, t_chk = apply_move(R, t, raise_cm=cm)
                left = [r for r in frame_one(R_chk, t_chk, K, items)
                        if not check_one(r, W, H, args.margin)[0]]
                if left:
                    worst2 = max((check_one(r, W, H, args.margin)[1] or 0.0)
                                 for r in left)
                    print(f"         （按升高 {cm:+.1f} cm 复核：仍有 {len(left)} 项"
                          f"不合格，最差还差 {worst2:.0f} px —— 实际挪到位后再跑一次，"
                          "这类外推本来就要迭代一两次）")
                else:
                    print(f"         （按升高 {cm:+.1f} cm 复核：**全部合格** ✅）")
            if sens["tilt"]:
                n_tilt = nv / sens["tilt"]
                n_after, _ = dzf.axis_angles(apply_move(R, t, tilt_deg=n_tilt)[0])
                fit = "✅ 仍在 ≤45° 界内" if n_after <= 45.0 else "❌ **破了 ≤45° 判据**"
                print(f"      或 抬高俯仰 {n_tilt:+.1f}° → 光轴到法线 "
                      f"{n_after:.1f}°，{fit}")
                # ⚠️ 这句**不能**无条件说「会破判据」：上一行刚算过，有时确实还
                #    在界内（44.2° 那种）。两行自相矛盾会让人不再信这个工具。
                #    判据的结论只在上一行给，这里只说「转动本身」的性质。
                print("         ⚠️ 转动**不改变任何一条视线**，对遮挡零帮助 —— "
                      "它只是把画面挪一下（判据是否被破见上一行）。"
                      "有升高/平移的余量就别用它。")

    sim_done = any([args.sim_raise_cm, args.sim_move_x_cm, args.sim_move_y_cm,
                    args.sim_tilt_deg])
    bad2 = None
    if sim_done:
        R2, t2 = apply_move(R, t, args.sim_raise_cm, args.sim_move_x_cm,
                            args.sim_move_y_cm, args.sim_tilt_deg)
        n2, _ = dzf.axis_angles(R2)
        print()
        bad2 = report(R2, t2, "模拟挪动之后"
                      f"（升高 {args.sim_raise_cm:+.1f} cm、x {args.sim_move_x_cm:+.1f}、"
                      f"y {args.sim_move_y_cm:+.1f}、俯仰 {args.sim_tilt_deg:+.1f}°）")
        print(f"  模拟后的光轴 vs 台面法线：{n2:.2f}°"
              f"（≤45° 判据：{'✅ 满足' if n2 <= 45.0 else '❌ 破了'}）")
        if args.sim_tilt_deg:
            print("  ⚠️ 提醒：**转动不改变任何一条视线**，对遮挡毫无帮助 —— "
                  "它只是把画面挪一挪。要越过遮挡只能升高/平移。")
        if bad2:
            for name, need_v, why in bad2:
                print(f"    仍然不合格：{name} —— {why}")
        else:
            print("  ✅ 模拟之后全部在框内")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({
                "extrinsics": os.path.abspath(args.extrinsics),
                "camera_center_base": [float(v) for v in t],
                "axis_to_normal_deg": to_normal,
                "image": [W, H], "margin_px": args.margin,
                "sensitivity_px_per_unit": {
                    "raise_cm": sens["raise"],
                    "camera_move_y_cm": sens["move_y"],
                    "camera_move_x_cm": sens["move_x"],
                    "tilt_deg": sens["tilt"],
                    "slots_y_cm": s_slot,
                },
            }, fh, ensure_ascii=False, indent=2)
        print(f"\n结果已写出：{args.json}")

    # ---- 结论 ----
    # ⚠️ 退出码描述的是**当前机位**（这份外参下的实际取景），不是模拟结果 ——
    #    模拟只是建议。所以「现在还不满足」时退出码就该是 1，哪怕模拟出来
    #    全部合格：那正是「还需要动手挪一次」的意思。
    print()
    if bad:
        print(f"取景结论（**当前**）：不满足 —— {len(bad)} 项出框或余量不足")
        if sim_done:
            print(f"取景结论（**模拟之后**）：{'满足 —— 照上面那个量挪即可' if not bad2 else f'仍不满足（{len(bad2)} 项）'}")
        return 1
    print("取景结论（**当前**）：满足 —— 全部项目在框内且余量 ≥ "
          f"{args.margin:.0f} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
