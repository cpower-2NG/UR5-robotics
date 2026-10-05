#!/usr/bin/env python3
"""方块「类别」的唯一定义处 —— 类别 = **颜色**。

为什么要有这个文件
------------------
多类别分拣里，「类别」这个词在四个地方各出现一次，而它们**必须说的是同一件事**：

    generate_grid_world.py  --classes      → 世界里的方块属于哪一类（写进 meta）
    auto_label_grid.py                     → 每帧标签的类别号（读 meta）
    build_dataset.py        --class-names  → data.yaml 的 names / nc，以及类别号上界校验
    planner_node.py         place_slot_classes → 哪一类放进哪几个槽位
    detector_node.py                       → 从**权重文件**读 names（本文件管不到，见下）

四份手抄的类别表一定会漂移，而漂移的症状是**静默的**：标签里的类别号与
data.yaml 的 names 错位，训练照样跑完、loss 照样下降，只是模型学到的名字
全是错的。所以类别表只在这里定义一次，其余全部 import。

⚠️ **顺序即索引。**
`CLASS_NAMES` 的顺序决定类别号 0、1、2……**改顺序等于重编所有已标注数据**。
`build_dataset.py --class-names` 必须传**完全相同顺序**的列表，
否则 `parse_label_line` 只在「类别号超出 [0,nc)」时才报错 —— 顺序错了它
一声不吭（因为每个号都还在范围内），错的是语义。

⚠️ **检测端不读这个文件。**
`detector_node.py` 的类别名来自权重文件自带的 `model.names`，这是**故意**的：
线上以权重为准，改了这里不会影响已经训好的模型。
代价是两边可能不一致 —— `scripts/test_block_classes.py` 会拿 data.yaml 的
names 与这里对照，作为交付前的关卡。

类别到底该有几类
----------------
不是拍脑袋定的，两条独立约束各自量出来，都落在 5 附近：

**约束一 · 颜色可分性**（`block_dataset` 4695 个框，同一张照片 = 一个光照条件）
    同一张照片内部的暖色展开中位 **49.3°**，而跨照片漂移只有 **3.4°**
    ⇒ 暖色那一大团不是「一种颜色在光照下漂移」，是真有好几种；
      111/210 张照片内部就能切成 3 段。
    青色图内展开 **2.0°**（90 分位 5.6°）⇒ 严格一种。
    紫/品红图内展开 13.4°，饱和 0.33、明度 0.31 ⇒ 一种，但**弱**。
    65°–180° **一个框都没有** ⇒ 没有绿色，别往类别表里放绿。

**约束二 · 台面容量**（`table_safety_envelope.py` 扫过的 357 格包线）
    最大整块安全矩形 x∈[0.40,0.75] y∈[-0.45,+0.45]；
    `pick_zone` 吃掉 y≥0，放置带只剩 y∈[-0.45,-0.05]；
    现场 4 槽 @0.10 m 间距（AG-95 行程 95 mm，**压线通过**，且间距检查只是
    告警不是错误）⇒ 0.10 间距能排 4×4=16 槽，0.16 间距 6~9 槽。
    每类至少留 2 槽 ⇒ 类数上限 **3（保守）～6（激进）**。

⚠️ **这两个数来自「前负责人那批积木」的数据集，不是用户手上那堆积木。**
现场照片里只看得到青、黄两种。要定死类数，得把全部积木摊在俯视相机下
拍一张，用同一把尺子（色相图内展开 vs 跨图漂移）量一遍。
"""

# 类别名的合法字符集：只允许小写字母、数字、下划线。
# 为什么限制：类别名会进 `data.yaml`、进 ROS 话题里的 `class_id` 字符串、
# 进日志。带空格或逗号的类别名会把 `--class-names a,b,c` 这个最常用的
# 传参形式直接切错，而症状是「类别数莫名其妙少了一个」。
import re

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


# 调色板：类别名 → 仿真世界里的 RGB（0~1，线性 sRGB，与 generate_grid_world.py
# 原来的 COLOR_POOL 同一量纲）。
#
# 这些值是**照着真机实测色相挑的**，不是随手取的漂亮颜色：
#   红 ~5°  橙 ~25°  黄 ~50°  青 ~190°  紫 ~315°
# 相邻暖色的色相间隔约 20°，而跨轮次漂移约 3.4° —— 间隔是漂移的 6 倍，
# 所以可分，但**暖色三类是整条链上最脆弱的一环**，别再加第四种暖色。
#
# ⚠️ 仿真里渲染出来的色相不完全等于这里的 RGB 色相（光照、材质、色调映射
#    都会挪一点）。这里给的是**偏纯**的颜色，好让各类之间留出余量。
PALETTE = {
    "red":    (0.85, 0.15, 0.15),
    "orange": (0.90, 0.45, 0.05),
    "yellow": (0.92, 0.82, 0.10),
    "cyan":   (0.10, 0.70, 0.72),
    "purple": (0.48, 0.20, 0.58),
    # 以下是备选：默认不用，但传 --classes 时可以取。
    # ⚠️ blue 与 cyan 色相只差约 50°，在真机的暖白光下容易互相吃；
    #    green 在实测数据里**一个框都没有**，启用前先确认现场真有绿积木。
    "blue":   (0.15, 0.25, 0.80),
    "green":  (0.15, 0.70, 0.20),
    "pink":   (0.85, 0.35, 0.55),
}


# 默认类别表 —— 顺序即索引，改动前先读模块 docstring 的 ⚠️。
#
# 为什么默认是这 5 个而不是文档里写的「红绿蓝」：那是规划书里的旧默认值，
# 从没量过；实测下来绿在数据里根本不存在，而暖色确实有三种。
DEFAULT_CLASSES = ["red", "orange", "yellow", "cyan", "purple"]


def parse_classes(spec, allow_unknown=False):
    """把 ``"red,yellow,cyan"`` 解析成类别名列表，顺带把非法输入挡在门外。

    这是**唯一的入口**：命令行、launch 参数、测试都从这儿走，
    免得每个调用点各写一遍 split/strip 而漏掉校验。

    ``allow_unknown=True`` 时允许调色板里没有的名字（真机想用别的颜色时），
    此时该类在仿真里没有颜色可用 —— 调用方必须自己处理 ``color_of`` 的
    缺失，别在这里悄悄补一个默认灰。
    """
    if isinstance(spec, (list, tuple)):
        names = [str(x).strip() for x in spec]
    else:
        names = [x.strip() for x in str(spec).split(",")]
    if not names or not any(names):
        raise ValueError("类别表是空的 —— 至少要有 1 类。")
    # 空项**不当空格跳过**：`red,,yellow` 与 `red,yellow,` 都是笔误，而
    # 「静默少一类」的症状恰好是类别号整体错位 —— 与本模块开头警告的是同一类
    # 故障（不报错、只是语义全错），所以宁可在这里炸。写成 `red,,yellow` 的人
    # 多半是想写四个类别里的某一个。
    if any(not n for n in names):
        raise ValueError(
            "类别表里有空项（多余的逗号？）：%r\n"
            "空项会让类别数比你以为的少一个，且不会报错。" % spec)
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise ValueError(
            "类别名重复：%s。\n"
            "重复的名字会让「类别号 → 名字」这个映射不再是单射，"
            "两个号指向同一个名字，而混淆矩阵会把它们当成两行打印出来。"
            % ", ".join(dup))
    bad = [n for n in names if not _NAME_RE.match(n)]
    if bad:
        raise ValueError(
            "类别名只允许小写字母开头的 [a-z0-9_]*：%s\n"
            "（带空格/逗号的名字会把 --class-names a,b,c 切错）" % ", ".join(bad))
    if not allow_unknown:
        unknown = [n for n in names if n not in PALETTE]
        if unknown:
            raise ValueError(
                "调色板里没有这些类别名：%s\n可用的是：%s\n"
                "（真机想用别的颜色就传 allow_unknown=True，"
                "但仿真渲染那里要自己给颜色）"
                % (", ".join(unknown), ", ".join(sorted(PALETTE))))
    return names


def class_index(name, names=None):
    """类别名 → 类别号。名字不在表里**抛异常**，不返回 -1 或 None。

    为什么这么严：调用点拿到 -1 之后最可能的写法是 `max(0, idx)`，
    于是所有认不出来的颜色悄悄塌成第 0 类 —— 混淆矩阵上看不出任何异常，
    只是第 0 类的召回率虚高。宁可在写标签时当场炸。
    """
    names = list(names or DEFAULT_CLASSES)
    try:
        return names.index(name)
    except ValueError:
        raise ValueError(
            "类别名 %r 不在类别表 %s 里。\n"
            "多半是生成世界用的 --classes 与标注/装配时用的 --class-names 不一致 ——"
            "这两处必须传同一个列表。" % (name, names))


def color_of(name):
    """类别名 → 仿真 RGB。缺失即抛异常（见 parse_classes 的 allow_unknown 说明）。"""
    if name not in PALETTE:
        raise ValueError("类别 %r 没有颜色定义，无法渲染。" % name)
    return PALETTE[name]


def describe(names):
    """一行说明，给各脚本启动时打印 —— 让日志里留着「这次用的类别表长什么样」。"""
    return " / ".join("%d:%s" % (i, n) for i, n in enumerate(names))
