#!/usr/bin/env python3
"""放置落点复核的判据。**纯函数，不 import rclpy**（与 ``slot_routing.py`` 同套路）。

为什么单独成一个模块
====================
「方块放下之后到底落在哪」这件事，此前**全链路上没有任何回读**：规划器拿配置里
的槽位坐标走 IK、下降、张开、抬起回 HOME，日志里只有「已放置 N/M 个」。
落点误差有多大、有没有放偏到隔壁格、方块有没有滚出槽位 —— 一个都答不上来。

这段判据的价值全在**它判错的方式**上，而判错的方式只有把逻辑单独拿出来喂合成
数据才试得出来（``planner_node.py`` 一 import 就拖进 rclpy / MoveIt / 话题 / 服务）。
所以判据在这里，接线在节点里。

四态为什么必须分开
==================
「复核失败」这三个字底下藏着四件完全不同的事，混在一起的话现场没法据此判断：

  ``OK``           期望类别就在槽位附近，Δ 记下来 —— 这是要拿的真值。
  ``WRONG_CLASS``  槽位附近**有东西，但是别的类别** ⇒ 放错格了（真错）。
  ``NOT_FOUND``    放置区**看得见**，但这一格附近什么都不是 ⇒ 方块飞了。
  ``UNAVAILABLE``  放置区**看不见 / 一条消息都没有** ⇒ 判不了，**不是**放失败了。

把后两者混起来是有害的：相机取景不对时每一格都会报「失败」，于是「8 个槽全错」
这个结论会把真正的取景问题掩盖成放置问题。这条区分是 ``zone_seen`` 字段存在的
全部理由。

⚠️ 上游有个反直觉的行为必须知道：``projector_node.py`` **从不发空数组**
（``if not targets_msg.detections: return``）。所以「相机看不见放置区」在节点侧
的表现是 **``/targets_3d`` 一条新消息都没有**，而不是「收到一批空检测」。
节点层的超时文案据此分成「一帧都没来」与「来了 N 帧但都不新」两条 —— 那件事
在 ``planner_node`` 里，这里只提供判据。
"""

import math
from collections import namedtuple

#: 一条观测：检测器报的类别名、分数、以及它在 ``base_frame`` 下的位置。
Observation = namedtuple("Observation", ["cls", "score", "x", "y", "z"])

# ---- 判决 ----
OK = "OK"
WRONG_CLASS = "WRONG_CLASS"
NOT_FOUND = "NOT_FOUND"
UNAVAILABLE = "UNAVAILABLE"
#: 等帧超时且**一帧新的都没拿到** —— 与 ``UNAVAILABLE`` 分开（见模块文档）。
TIMEOUT = "TIMEOUT"
#: 这一格压根没走到复核（放置后回 HOME 失败走了恢复流程 / 收工发生在复核途中）。
SKIPPED = "SKIPPED"

#: 由 :func:`judge_place` 给出的判决（其余两种由节点层的超时/取消路径给出）。
JUDGE_VERDICTS = (OK, WRONG_CLASS, NOT_FOUND, UNAVAILABLE)

#: 「判出来了」的三种 —— 收到这些就可以**立刻结束等待**，不必熬到超时。
DECIDED = (OK, WRONG_CLASS, NOT_FOUND)

VERDICT_NAMES = {
    OK: "通过",
    WRONG_CLASS: "失败：放进了别的类别",
    NOT_FOUND: "失败：槽位附近没有东西",
    UNAVAILABLE: "不可用：放置区看不见",
    TIMEOUT: "超时未复核",
    SKIPPED: "未复核",
}


#: :func:`judge_place` 的返回值。``obs`` 是**据以判决的那一条观测**（可能为 None），
#: 留着它是为了日志能打出「我到底看到的是什么」—— 只报结论不报依据的话，
#: 现场无法判断是放置偏了还是检测认错了。
Result = namedtuple("Result", ["verdict", "obs", "detail", "z_warn"])


def place_roi(slots, margin):
    """全部槽位的包围盒外扩 ``margin`` → ``(xmin, xmax, ymin, ymax)``。

    无槽位时返回 ``None``（= 不筛）。这个 ROI **只用来回答「放置区这一块相机看
    不看得见」**，不参与「放对没放对」的判定 —— 后者由 ``tol_xy`` 说了算。
    两者分开的理由见 ``UNAVAILABLE`` 的说明。
    """
    if not slots:
        return None
    xs = [float(s[0]) for s in slots]
    ys = [float(s[1]) for s in slots]
    return (min(xs) - margin, max(xs) + margin,
            min(ys) - margin, max(ys) + margin)


def in_roi(roi, x, y):
    """点是否在放置区 ROI 内（闭区间）。``roi is None`` = 不筛。"""
    if roi is None:
        return True
    xmin, xmax, ymin, ymax = roi
    return xmin <= x <= xmax and ymin <= y <= ymax


def _dist(o, xy):
    return math.hypot(o.x - xy[0], o.y - xy[1])


def _ok_result(o, expected_xy, expected_z, z_tol):
    """构造 ``OK``，并把 z 偏离折成一条**只告警**的后缀。

    ⚠️ z 刻意**不参与通过/失败**：方块斜着落、两台叠在一起、深度在物体边缘取到
    台面 —— 这些都会让 z 失准，而它们恰恰是本项目要测的现象。拿 z 当硬门，
    等于把要测的量在测量环节丢掉。所以 z 只留告警，留在台账里给人看。
    """
    dx = o.x - expected_xy[0]
    dy = o.y - expected_xy[1]
    z_warn = None
    if expected_z is not None:
        dz = o.z - float(expected_z)
        if abs(dz) > z_tol:
            z_warn = (f"z 偏离 {dz*1000:+.0f} mm（> {z_tol*1000:.0f} mm，"
                      f"只告警不判失败）")
    return Result(OK, o,
                  f"实测({o.x:.3f},{o.y:.3f})  "
                  f"Δ=({dx*1000:+.1f},{dy*1000:+.1f}) mm  "
                  f"|Δ|={math.hypot(dx, dy)*1000:.1f}",
                  z_warn)


def judge_place(observations, expected_xy, expected_class,
                tol_xy, min_score, z_tol,
                roi=None, expected_z=None):
    """一批检测 → 这一个槽位的判决（:class:`Result`）。

    参数
    ----
    ``observations``   :class:`Observation` 列表（``/targets_3d`` 里那一批）。
    ``expected_xy``    期望落点 ``(x, y)``，``base_frame``，就是配置里的槽位坐标。
    ``expected_class`` 期望类别名。**``None`` / 空串 = 不按类别分**（A1 单类别
                       那条路），此时只判「槽位附近有没有东西、分数够不够」。
    ``tol_xy``         落点容差（m）。
    ``min_score``      最低分数 —— 与抓取段用**同一个** ``min_score``，不另编数。
    ``z_tol``          z 告警阈值（m），只告警。
    ``roi``            放置区 ROI（见 :func:`place_roi`）。
    ``expected_z``     期望高度（``place_z``，即方块静置时**可见顶面**的高度）。
    """
    obs = list(observations or [])
    if not obs:
        # 投影器从不发空数组（见模块文档），所以这一条**正常永远不该走到**。
        # 留着是为了让「上游行为变了」这件事响亮，而不是静静地判成「没找到」。
        return Result(UNAVAILABLE, None,
                      "这一批检测是空的（投影器不该发空数组，上游行为变了？）",
                      None)

    inside = [o for o in obs if in_roi(roi, o.x, o.y)]
    if not inside:
        # 相机可能根本没取景到放置区 —— 这是**判不了**，不是放失败。
        return Result(UNAVAILABLE, None,
                      f"放置区里一个检测都没有（本批 {len(obs)} 条全在区外）",
                      None)

    near = [o for o in inside if _dist(o, expected_xy) <= tol_xy]
    if not near:
        close = min(inside, key=lambda o: _dist(o, expected_xy))
        return Result(NOT_FOUND, close,
                      f"{tol_xy*1000:.0f} mm 内没有任何检测；最近的是 "
                      f"{close.cls} {close.score:.2f} 距 "
                      f"{_dist(close, expected_xy)*1000:.0f} mm",
                      None)

    good = [o for o in near if o.score >= min_score]
    if expected_class:
        mine = [o for o in good if o.cls == expected_class]
        if mine:
            return _ok_result(max(mine, key=lambda o: o.score),
                              expected_xy, expected_z, z_tol)
        others = [o for o in good if o.cls != expected_class]
        if others:
            pick = min(others, key=lambda o: _dist(o, expected_xy))
            return Result(WRONG_CLASS, pick,
                          f"{tol_xy*1000:.0f} mm 内是 {pick.cls} "
                          f"{pick.score:.2f}，不是 {expected_class}",
                          None)
        # ``near`` 里全是低分的
    elif good:
        # 不按类别分：期望类别未知，只判位置与分数。
        return _ok_result(max(good, key=lambda o: o.score),
                          expected_xy, expected_z, z_tol)

    best = max(near, key=lambda o: o.score)
    return Result(NOT_FOUND, best,
                  f"{tol_xy*1000:.0f} mm 内有 {best.cls}，但分数 "
                  f"{best.score:.2f} < {min_score:.2f}",
                  None)


def format_slot_line(index, expected_xy, expected_class, verdict,
                     obs=None, detail="", z_warn=None):
    """收工台账里的一行（格式见本项目的台账惯例）。"""
    cls = expected_class or "通配"
    head = (f"    槽位[{index}] {cls:<8}"
            f" 期望({expected_xy[0]:.3f},{expected_xy[1]:.3f})")
    if verdict == OK and obs is not None:
        dx = obs.x - expected_xy[0]
        dy = obs.y - expected_xy[1]
        body = (f"  实测({obs.x:.3f},{obs.y:.3f})  "
                f"Δ=({dx*1000:+.1f},{dy*1000:+.1f}) mm  "
                f"|Δ|={math.hypot(dx, dy)*1000:.1f}  通过")
        if z_warn:
            body += f"  ⚠️ {z_warn}"
        return head + body
    note = detail or VERDICT_NAMES.get(verdict, verdict)
    return f"{head}  —— {VERDICT_NAMES.get(verdict, verdict)}：{note}"


def summarize(verdicts):
    """全部槽位的汇总行。**五种计数一个都不能省** —— 少了哪一种，现场就会把
    那一种当成另一种（最常见的是把「不可用」读成「失败」）。"""
    counts = {}
    for v in verdicts:
        counts[v] = counts.get(v, 0) + 1
    failed = counts.get(WRONG_CLASS, 0) + counts.get(NOT_FOUND, 0)
    return ("  落点复核汇总：通过 {}  失败 {}  不可用 {}  超时未复核 {}  未复核 {}"
            .format(counts.get(OK, 0), failed,
                    counts.get(UNAVAILABLE, 0), counts.get(TIMEOUT, 0),
                    counts.get(SKIPPED, 0)))
