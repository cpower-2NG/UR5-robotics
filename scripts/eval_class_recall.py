#!/usr/bin/env python3
"""A2 验收面：**每类**召回 / 精确、混淆矩阵、最严重的类别间混淆对。

为什么单独一个脚本
==================
A2 的验收判据是「每类召回/混淆矩阵 + 分拣录像」，而 `model.val()` 自己打出来的
是一行**汇总**（mAP50 / mAP50-95）—— 7 类里整整一类认不出来，汇总数字几乎不动。
所以这里把矩阵拆开按类看，并且**判定**：任何一类的召回低于阈值就退出码 1。

三个静默坑
==========
都实测过。写错了不会报错，只是报表说的是另一件事。

1. **行 = 预测，列 = 真值。** ultralytics 与 sklearn 相反
   （`ConfusionMatrix.process_batch` 里写的是 `matrix[dc, gc]`）。行/列搞反的
   后果是**召回与精确整体对调** —— 一张好看的报表，讲的是另一件事。
   详见 `per_class_scores()`。

2. **不显式给 `conf`，矩阵建在一个你不知道的阈值上。** `ConfusionMatrix.process_batch`
   里有一行会把「默认值」**悄悄改写**：

       conf = 0.25 if conf in {None, 0.01 if is_obb else 0.001} else conf

   也就是说 val 的默认 `conf=0.001`（或 `None`）到这里变成 **0.25**，不是 0.001
   （实测：0.15 分的框在 conf=0.001 / None / 0.25 下都被丢掉，在 0.10 下才算数）。
   于是「不传 conf」拿到的是一张 **0.25 阈值**的矩阵 —— 仍然不是线上工作点。
   本脚本**强制**显式传 `--conf`，默认 `DEFAULT_CONF`
   （= `pipeline.launch.py` 的 `confidence_threshold`，两者由
   `test_eval_class_recall.py` 钉住不许漂）。

3. **分母是整列，不是类别块。** 漏检（判成背景）必须计入该类的真值总数，
   否则「一整类全漏」会因为分母塌成 0 而从报表里**消失**。

退出码：0 = 判定全过；1 = 有类别没过；2 = 环境没准备好（缺 ultralytics 等）。
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                # 与其它脚本同一套 import 约定
from block_classes import DEFAULT_CLASSES      # noqa: E402

# 线上工作点：`pipeline.launch.py` 与 `detector_node.py` 的
# `confidence_threshold` 默认都是 0.5。验收就该量在这个点上 —— 检测器在
# 0.3 分的框上不会去抓，把它算进召回是自欺。
# `test_eval_class_recall.py` 会去读 pipeline.launch.py 断言两者相等。
DEFAULT_CONF = 0.5


# ----------------------------------------------------------------------
# 纯函数 —— 测试要能直接喂假矩阵进来（板上没装 torch 时也要能跑）
# ----------------------------------------------------------------------
def per_class_scores(matrix, nc):
    """混淆矩阵 → ``(recalls, precisions, supports)``，各长度 ``nc``。

    ⚠️ **行 = 预测，列 = 真值。** 与 sklearn 相反。2026-10-05 在板上用构造批次
    实测确认：GT 两个 class 0、预测两个 class 1/2（都错），得到
    ``matrix[1,0] = matrix[2,0] = 1`` —— 正是「行是预测」。源码里也对得上：
    FP 落 ``matrix[dc, nc]``（预测类那一行、背景列），FN 落 ``matrix[nc, gc]``。

    于是：

        support_i   = **列和**，含被判成背景的那一行（漏检也是真值）
        recall_i    = matrix[i, i] / support_i
        precision_i = matrix[i, i] / **行和**，含被判成背景的那一列（虚警）

    分母用**整列**、不是类别块内的和：漏检必须计入分母，否则「一类全漏」
    会因为分母塌成 0 而被当成「没有样本」，从报表里消失 —— 那正好是最该报的
    故障。没有真值样本时召回返回 ``None``（不是 0，也不是 NaN）：
    ``None`` 会被判定环节明确跳过，``0`` 会被当成「一类都没认出来」而误判。

    ``matrix`` 是 ``(nc+1) x (nc+1)`` 的计数（最后一行/列是背景）。
    只用下标取数，不依赖 numpy —— 纯逻辑部分保持标准库，离线自检才好跑
    （numpy 数组照样能这么索引）。
    """
    if len(matrix) != nc + 1 or any(len(r) != nc + 1 for r in matrix):
        raise ValueError("混淆矩阵形状不是 %d x %d：%s"
                         % (nc + 1, nc + 1, [len(r) for r in matrix]))

    tp = [int(matrix[i][i]) for i in range(nc)]
    supports = [sum(int(matrix[i][j]) for i in range(nc + 1)) for j in range(nc)]
    preds = [sum(int(matrix[i][j]) for j in range(nc + 1)) for i in range(nc)]

    recalls = [float(tp[i]) / supports[i] if supports[i] > 0 else None
               for i in range(nc)]
    precisions = [float(tp[i]) / preds[i] if preds[i] > 0 else None
                  for i in range(nc)]
    return recalls, precisions, supports


def top_confusions(matrix, nc, names, k=8, min_share=0.0):
    """最容易混的类别对，最严重的在前。

    返回 ``[(真值类名, 被判成, 个数, 占该类真值的比例), ...]``。

    只取**类别之间**的（``i != j``，都 ``< nc``）：背景在报表里另有归属 ——
    漏检是「没找到」，虚警是「无中生有」，和「认错成另一类」是三件不同的事，
    混在一起报会让人以为是颜色分不开。

    ⚠️ 读这个表要先知道匹配规则：`process_batch` 用的是 **`box_iou` 类别无关**、
    再**按 IoU 降序**贪心配对（不是按置信度）。所以一条「真值 j → 预测 i」
    的含义是：**在 j 的框上，没有任何一个判成 j 的检测框，IoU 高过那个判成 i 的**。
    这带来一个反直觉的后果 —— 阈值调低时，某对混淆可能**消失**：多出来的
    低分框若更贴合方块（IoU 更高），它会赢下匹配。实测过：purple→cyan 在
    conf=0.5 时占 14.2%，降到 0.25 只剩 1.9%，就是因为紫框重新赢回了匹配。
    """
    out = []
    # 只用下标、不切 numpy —— 测试要能直接喂手搓的嵌套列表进来。
    #
    # ⚠️ 分母要取到 **nc+1** 行：漏检那行（背景行）也是这个类的真值。
    # 只加到 nc 就会把「漏检」从分母里剔掉，于是「一半漏检、一半认错」的一类
    # 会被算成「认错占 100%」—— 这正是本脚本文件头警告的坑 #3，这里同样适用。
    supports = [sum(int(matrix[i][j]) for i in range(nc + 1)) for j in range(nc)]
    for j in range(nc):                     # j = 真值（列）
        if supports[j] <= 0:
            continue
        for i in range(nc):                 # i = 预测（行）
            if i == j:
                continue
            n = int(matrix[i][j])
            if n <= 0:
                continue
            share = n / float(supports[j])
            if share < min_share:
                continue
            out.append((names[j], names[i], n, share))
    out.sort(key=lambda t: (-t[3], -t[2]))
    return out[:k]


def subset_matrix(matrix, nc, names, keep):
    """把混淆矩阵**裁剪**成「只分拣 ``keep`` 这几类」，返回 ``(新矩阵, 新类别表)``。

    用途：某几类今天不打算用（难度高 / 训练集不够），先把易于区分的做成一个
    可交付的分拣任务。裁剪后重新算召回与精确，才是那个任务的真实指标。

    ⚠️ **被裁掉的行列是并进背景，不是删掉。** 差别是实质性的：

      删掉「真值 = purple」那一列，等于宣告「桌上不会有紫方块」—— 于是一个紫
      方块被认成青，会**凭空消失**，青的精确率一点不掉。可现实里那是一个
      青槽位被塞进了紫方块，是**分拣错误**。

      并进背景列才对：`matrix[cyan][purple]` 变成 `matrix'[cyan][背景]` ——
      青的**虚警**。行同理：被裁类的**预测**并进背景行，即漏检。

    所以「只分拣 5 类」的指标天然比「7 类指标里挑出那 5 行」更差、也更诚实。
    """
    for n in keep:
        if n not in names:
            raise ValueError("类别 %r 不在 %s 里" % (n, names))
    if not keep:
        raise ValueError("keep 是空的 —— 至少要留一类")
    if len(set(keep)) != len(keep):
        raise ValueError("keep 里有重复：%r" % (keep,))

    idx = [names.index(n) for n in keep]
    nc2 = len(keep)
    out = [[0] * (nc2 + 1) for _ in range(nc2 + 1)]

    def slot(k, n_classes):
        """原类别号 → 新位置；被裁掉的都落到背景（下标 n_classes）。"""
        return idx.index(k) if k in idx else n_classes

    # 类别块
    for i in range(nc):                      # i = 预测
        for j in range(nc):                  # j = 真值
            out[slot(i, nc2)][slot(j, nc2)] += int(matrix[i][j])
    # 背景列（虚警）：预测是 i、真值其实不存在
    for i in range(nc):
        out[slot(i, nc2)][nc2] += int(matrix[i][nc])
    # 背景行（漏检）：真值是 j、没预测出来
    for j in range(nc):
        out[nc2][slot(j, nc2)] += int(matrix[nc][j])
    out[nc2][nc2] += int(matrix[nc][nc])
    return out, list(keep)


def missed_per_class(matrix, nc):
    """每类**漏检**（判成背景）的个数 —— 行是背景、列是该类。"""
    return [int(matrix[nc][j]) for j in range(nc)]


def false_alarms_per_class(matrix, nc):
    """每类**虚警**的个数 —— 行是该类、列是背景。"""
    return [int(matrix[i][nc]) for i in range(nc)]


def gate(recalls, supports, names, confusions,
         min_recall=0.90, min_support=20, max_pair_share=0.10, misses=None):
    """判定。返回 ``(ok, 条目列表)``，条目是 ``(标记, 一句话)``。

    两条独立的判据，都要过：

    - **每类召回 >= min_recall**，只判真值数够（``>= min_support``）的类。
      支撑数太小的类一次漏检就是 0 或 1 的跳变，判了只会得到噪声。
    - **没有哪一对「真值 j → 预测 i」占到该类的 ``max_pair_share`` 以上**。
      这一条与召回不重复：一类可以有 0.92 的召回、剩下 8% 却**全部**涌向
      同一类 —— 那是系统性的相邻色问题，值得单独报出来。

    被跳过的类**明确列在条目里**，不当成通过 —— 报表上不能有沉默的类。
    """
    items = []
    ok = True

    for i, n in enumerate(names):
        if supports[i] < min_support:
            items.append(("跳过", "%-12s 真值仅 %d 个（<%d），不参与判定"
                          % (n, supports[i], min_support)))
            continue
        r = recalls[i]
        if r is None:
            ok = False
            items.append(("失败", "%-12s 有 %d 个真值却算不出召回（分母塌了？）"
                          % (n, supports[i])))
        elif r < min_recall:
            ok = False
            # 措辞是「没认对」不是「漏了」：这两个数里既有漏检、也有认成别的类。
            # 两种故障要分开看（上表里有各自的列），一句话混着说会指错方向。
            n_bad = supports[i] - int(round(r * supports[i]))
            if misses is None:
                detail = "%d/%d 没认对" % (n_bad, supports[i])
            else:
                detail = ("%d/%d 没认对：漏检 %d + 认成别类 %d"
                          % (n_bad, supports[i], int(misses[i]), n_bad - int(misses[i])))
            items.append(("失败", "%-12s 召回 %.3f < %.3f   （%s）"
                          % (n, r, min_recall, detail)))
        else:
            items.append(("通过", "%-12s 召回 %.3f（%d 个真值）"
                          % (n, r, supports[i])))

    idx = {n: i for i, n in enumerate(names)}
    for true_n, pred_n, cnt, share in confusions:
        if share <= max_pair_share:
            continue
        j = idx.get(true_n)
        if j is None or supports[j] < min_support:
            continue                        # 支撑数太小的对不判（同上）
        ok = False
        items.append(("失败", "「%s」有 %.1f%% 被判成「%s」（%d 个）—— 超过 %.1f%%"
                      % (true_n, share * 100, pred_n, cnt, max_pair_share * 100)))
    return ok, items


def format_matrix(matrix, nc, names):
    """把混淆矩阵画成文本表。行 = 预测，列 = 真值（见 `per_class_scores`）。

    最后一行/列是背景，单独用 ``|`` 隔开 —— 它们与类别块不是一回事：
    行上的背景 = 漏检（真值有、没预测出来），列上的背景 = 虚警。
    """
    w = max(6, max(len(n) for n in names) + 1)
    # 末列 = `matrix[i][nc]` = 预测了、真值没有 → **虚警**（不是漏检）
    lines = [" " * (w + 2) + "".join("%*s" % (w, n) for n in names)
             + " | %*s" % (w, "虚警")]
    for i in range(nc):
        row = "".join("%*d" % (w, int(matrix[i][j])) for j in range(nc))
        lines.append("%-*s  %s | %*d" % (w, names[i], row, w, int(matrix[i][nc])))
    lines.append("-" * (w + 2 + w * nc + 3 + w))
    # 末行 = `matrix[nc][j]` = 真值有、没预测出来 → **漏检**（不是虚警）
    miss = "".join("%*d" % (w, int(matrix[nc][j])) for j in range(nc))
    lines.append("%-*s  %s | %*d" % (w, "漏检", miss, w, int(matrix[nc][nc])))
    return "\n".join(lines)


def format_report(matrix, nc, names, recalls, precisions, supports,
                  confusions, misses, fps, a):
    """整份报表。返回字符串 —— 同时进 stdout 和报告文件。"""
    out = []
    out.append("=" * 72)
    out.append("A2 每类召回 / 混淆矩阵")
    out.append("=" * 72)
    out.append("模型      : %s" % a.model)
    out.append("数据集    : %s" % a.data)
    out.append("工作点    : conf=%.3f  iou=%.3f  imgsz=%d" % (a.conf, a.iou, a.imgsz))
    out.append("判定阈值  : 召回 >= %.2f，真值 < %d 个的类不判，单对混淆 <= %.0f%%"
               % (a.min_recall, a.min_support, a.max_pair_share * 100))
    out.append("")
    out.append(format_matrix(matrix, nc, names))
    out.append("")
    out.append("-" * 72)
    out.append("%-14s %6s %8s %8s %8s %8s" % ("类别", "真值", "召回", "精确", "漏检", "虚警"))
    out.append("-" * 72)
    for i, n in enumerate(names):
        out.append("%-14s %6d %8s %8s %8d %8d"
                   % (n, supports[i],
                      "  --  " if recalls[i] is None else "%6.3f" % recalls[i],
                      "  --  " if precisions[i] is None else "%6.3f" % precisions[i],
                      misses[i], fps[i]))
    tot_t = sum(supports)
    tot_hit = sum(int(matrix[i][i]) for i in range(nc))
    out.append("-" * 72)
    out.append("%-14s %6d %8s" % ("合计", tot_t,
                                  "  --  " if not tot_t else "%6.3f" % (tot_hit / tot_t)))
    out.append("")

    out.append("-" * 72)
    if confusions:
        out.append("最严重的类别间混淆（真值 → 被判成）：")
        for true_n, pred_n, cnt, share in confusions:
            out.append("  %-12s → %-12s %5d 个  %5.1f%%" % (true_n, pred_n, cnt, share * 100))
    else:
        out.append("类别间混淆：一个都没有（没有哪个真值类被判成别的类）。")
    out.append("")

    ok, items = gate(recalls, supports, names, confusions,
                     a.min_recall, a.min_support, a.max_pair_share, misses)
    out.append("-" * 72)
    out.append("判定：")
    for mark, text in items:
        out.append("  %s  %s" % (mark, text))
    out.append("")
    out.append("=" * 72)
    out.append("结果：%s" % ("通过" if ok else "**未通过**"))
    out.append("=" * 72)
    return "\n".join(out), ok


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="A2 验收面：每类召回 / 混淆矩阵 + 判定")
    ap.add_argument("--model", required=True, help="训好的权重（best.pt）")
    ap.add_argument("--data", default="block_dataset_7cls",
                    help="数据集目录或 data.yaml")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--device", default=None, help="如 0 / cpu；默认交给 ultralytics")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF,
                    help="建混淆矩阵的置信度阈值。默认 %.2f = pipeline.launch.py 的 "
                         "confidence_threshold（线上工作点）。**别用 ultralytics 的 "
                         "default** —— 不传的话 ultralytics 会把它悄悄改写成 "
                         "0.25，矩阵建在一个你不知道的阈值上。" % DEFAULT_CONF)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--min-recall", type=float, default=0.90)
    ap.add_argument("--min-support", type=int, default=20,
                    help="真值少于这个数的类不参与判定（但仍在报表里）")
    ap.add_argument("--max-pair-share", type=float, default=0.10,
                    help="单对「真值→被判成」占该类的比例上限")
    ap.add_argument("--classes", default=None,
                    help="类别表，逗号分隔（默认取 block_classes.DEFAULT_CLASSES，"
                         "运行时以权重自带的 names 为准）")
    ap.add_argument("--subset", default=None,
                    help="只判这几类（逗号分隔），其余**并进背景**。用于「今天先做"
                         "易区分的子集」那种分拣任务 —— 裁掉类的方块若被认成留下的类，"
                         "会算成那个类的虚警，不会凭空消失。不给 = 全判。")
    ap.add_argument("--report", default="class_recall_report.txt",
                    help="报表落盘路径（''=不落盘）")
    ap.add_argument("--json", dest="json_out", default=None,
                    help="另外落一份机器可读的 JSON")
    ap.add_argument("--name", default="class_recall",
                    help="val 的可视化输出目录名（plots=True 时的混淆矩阵图）")
    ap.add_argument("--project", default="runs",
                    help="val 可视化输出根目录")
    a = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError as e:
        print("需要 ultralytics 才能跑本脚本（%s）。\n"
              "纯逻辑部分（per_class_scores / gate / ...）可以直接 import 用 —— "
              "离线自检 scripts/test_eval_class_recall.py 走的就是那条路。" % e)
        return 2

    # data 可以直接给目录，ultralytics 认 <dir>/data.yaml
    data = a.data
    if os.path.isdir(data):
        data = os.path.join(data, "data.yaml")
    if not os.path.isfile(data):
        print("找不到数据集配置：%s" % data)
        return 2

    model = YOLO(a.model)
    print("模型 %s 的类别表：%s"
          % (a.model, " / ".join("%d:%s" % (k, v) for k, v in model.names.items())))

    metrics = model.val(data=data, imgsz=a.imgsz, batch=a.batch, device=a.device,
                        conf=a.conf, iou=a.iou, plots=True,
                        project=a.project, name=a.name, exist_ok=True)

    cm = getattr(metrics, "confusion_matrix", None)
    if cm is None:
        print("metrics 里没有 confusion_matrix —— ultralytics 版本变了？")
        return 2

    nc = int(cm.nc)
    # 类别名以**权重自带的 names**为准（线上检测端也是这么取的，
    # 见 block_classes.py 的说明）。与调色板不一致只告警不判死。
    names = [str(cm.names[i]) for i in range(nc)]
    expect = list(DEFAULT_CLASSES) if a.classes is None else \
        [s.strip() for s in a.classes.split(",") if s.strip()]
    if names != expect:
        print("\n⚠️ 权重里的类别表与预期不一致：")
        print("   权重：%s" % names)
        print("   预期：%s" % expect)
        print("   （顺序不同 = 类别号整体错位，报表的名字会是错的）\n")

    matrix = cm.matrix

    if a.subset:
        keep = [s.strip() for s in a.subset.split(",") if s.strip()]
        dropped = [n for n in names if n not in keep]
        matrix, names = subset_matrix(matrix, nc, names, keep)
        nc = len(names)
        print("\n只按这几类判：%s" % " / ".join(names))
        if dropped:
            print("  裁掉的是 %s —— 它们的行列**并进了背景**（不是删掉）："
                  % " / ".join(dropped))
            print("  这一类的方块若被认成留下的类，会算成那个类的**虚警**，"
                  "而不是凭空消失。")

    recalls, precisions, supports = per_class_scores(matrix, nc)
    confusions = top_confusions(matrix, nc, names, k=10)
    misses = missed_per_class(matrix, nc)
    fps = false_alarms_per_class(matrix, nc)

    text, ok = format_report(matrix, nc, names, recalls, precisions, supports,
                             confusions, misses, fps, a)
    print("\n" + text)

    if a.report:
        with open(a.report, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print("\n报表已落盘：%s" % os.path.abspath(a.report))

    if a.json_out:
        blob = {
            "model": a.model, "data": a.data,
            "conf": a.conf, "iou": a.iou, "imgsz": a.imgsz,
            "classes": names,
            "supports": supports,
            "recall": recalls, "precision": precisions,
            "missed": misses, "false_alarms": fps,
            "confusions": [{"true": t, "pred": p, "count": c, "share": s}
                           for t, p, c, s in confusions],
            "matrix": [[int(v) for v in row] for row in matrix],
            "passed": ok,
            "thresholds": {"min_recall": a.min_recall,
                           "min_support": a.min_support,
                           "max_pair_share": a.max_pair_share},
        }
        with open(a.json_out, "w", encoding="utf-8") as f:
            json.dump(blob, f, ensure_ascii=False, indent=2)
        print("JSON 已落盘：%s" % os.path.abspath(a.json_out))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
