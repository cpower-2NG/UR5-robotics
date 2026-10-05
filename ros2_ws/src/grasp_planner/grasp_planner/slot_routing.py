#!/usr/bin/env python3
"""类别 → 放置槽位的路由。**纯函数，不 import rclpy**（所以能秒级单测）。

为什么单独成一个模块
====================
多类别分拣里，「哪一类放进哪个槽位」是**唯一一处错了不会报错**的地方：
把 red 放进 yellow 的区域，机械臂动作完全正常、日志完全正常、收工也正常 ——
只有录像里那一格颜色不对。这种错只能靠断言钉住，而断言要先能把这段逻辑
**单独拿出来跑**。`planner_node.py` 一 import 就拖进 rclpy / MoveIt / 话题 /
服务，没法做纯单测；把判据抽到这里就没有这个问题。

约定
====
``slot_classes``：与 ``place_slots`` **等长**的字符串列表，第 i 项是第 i 个槽位
专属的类别名。**空串 = 通配**（这一格谁都能放）。整个列表为空 = 不做路由，
所有槽位通配 —— 这时行为与 A1（单类别）完全一致。

为什么用**类别名**而不是类别号
------------------------------
类别号是「类别表里的下标」，而类别表在生成器/标注器/数据集/权重四处各有
一份；顺序一变，同一个数字的含义就全反了，而且**每一处都不会报错**
（见 ``scripts/block_classes.py`` 的文档）。名字没有这个自由度：名字对不上
就是匹配不到槽位，是可见的。

⚠️ 名字的拼写必须与**权重文件里的类别名**逐字相同（检测器把
``model.names[cls]`` 直接塞进 ``ObjectHypothesis.class_id``）。差一个字母
（``yellow`` vs ``Yellow``）的后果是这一类**一个槽位都匹配不到**，于是它永远
放不下去 —— 启动日志里的路由表就是给这件事对照用的。
"""


def validate_slot_classes(slot_classes, n_slots):
    """校验并归一化路由表。返回**长度恒等于 ``n_slots``** 的列表。

    ``None`` / 空列表 = 不路由 → 归一化成 ``[''] * n_slots``（全是通配）。

    ⚠️ 为什么必须归一化，而不是原样返回空列表：``pick_slot`` 拿到空列表会
    直接返回 ``None``（它分不清「没配路由」和「配了个空表」），于是**不路由时
    一个槽位都用不了** —— 也就是把默认路径（A1 的单类别行为）彻底弄坏。
    这个 bug 真出现过，是 ``test_planner_class_sort.py`` 的 B2 段抓到的。
    让「不路由」与「全通配」在内部是**同一个表示**，这类分叉就不存在了。

    值错就**抛 ValueError**，由调用方在启动期拒绝启动 —— 与 ``place_slots``
    的奇偶检查同一个理由：槽位表决定了「还能放几个」，悄悄少一个的后果是
    少放一个物体且没有任何提示。
    """
    if slot_classes is None:
        return [""] * n_slots
    # ROS 2 的空数组默认值会被推断成 BYTE_ARRAY（见 planner_node 里的参数注释），
    # 真读进来可能是一串 int。长度 0 就是「不路由」，与元素类型无关。
    vals = [("" if v is None else str(v)).strip() for v in slot_classes]
    if not vals:
        return [""] * n_slots
    if len(vals) != n_slots:
        raise ValueError(
            f"place_slot_classes 有 {len(vals)} 项，但 place_slots 有 "
            f"{n_slots} 个槽位 —— 两者必须等长（第 i 项描述第 i 个槽位）。\n"
            f"⚠️ 不能靠截断或补齐来「修好」：那会让某几个槽位静默地换一类，"
            f"或凭空多出/少掉槽位，而槽位数正是收工条件。\n"
            f"  要么写满 {n_slots} 项（不需要限制的槽位写空串 \"\"），"
            f"要么整个留空 = 不按类别分（所有槽位谁都能放）。"
        )
    return vals


def is_routed(slot_classes):
    """是否真的在按类别分（任一项非空串）。"""
    return any(c for c in (slot_classes or []))


def pick_slot(used, slot_classes, class_name):
    """第一个**未占用**且能接收 ``class_name`` 的槽位下标；没有则 ``None``。

    ``used``：与槽位等长的 bool 列表。**不修改任何入参** —— 占用要靠调用方
    在真的放下之后自己标，否则「查一下有没有空位」这个只读动作会消耗掉槽位。

    ⚠️ ``class_name`` 为空/None（检测器没给类别名）时**只有通配槽位能接**。
    不能反过来当成通配符：那会让「不知道是什么」的物体占用任意一格，
    而分拣的全部意义就是每类进自己的格。宁可放不下（响亮地收工告警），
    也不要放错格（安静地出错）。
    """
    if not used or not slot_classes or len(used) != len(slot_classes):
        return None
    for i, want in enumerate(slot_classes):
        if used[i]:
            continue
        if not want or (class_name and want == class_name):
            return i
    return None


def has_free_slot(used, slot_classes, class_name):
    """这一类还放得下吗。``pick_slot(...) is not None`` 的同义写法。"""
    return pick_slot(used, slot_classes, class_name) is not None


def describe_routing(slot_classes, slots=None):
    """启动日志用的一行路由表，例如 ``0:(0.44,0.24)→red  1:(0.60,0.24)→red``。

    ``slots`` 给了就连坐标一起打 —— 现场对表时坐标比下标好认得多。
    """
    if not is_routed(slot_classes):
        return "不按类别分（所有槽位谁都能放）"
    parts = []
    for i, name in enumerate(slot_classes):
        where = ""
        if slots is not None and i < len(slots):
            where = "(%s) " % ",".join("%.2f" % v for v in slots[i])
        parts.append("%d:%s%s" % (i, where, name or "*"))
    return "  ".join(parts)


def class_histogram(slot_classes):
    """类别名 → 槽位数。给「每类至少两个槽位」这类容量检查用。"""
    hist = {}
    for name in slot_classes or []:
        hist[name] = hist.get(name, 0) + 1
    return hist
