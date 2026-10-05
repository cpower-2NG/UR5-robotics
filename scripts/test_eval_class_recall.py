#!/usr/bin/env python3
"""A2 验收面的**尺子**准不准：`eval_class_recall.py` 的召回/混淆判据。

（沿用本仓库 ``scripts/test_*.py`` 的约定：退出码 0 = 通过，2 = 环境没准备好。）

为什么这个自检非有不可
======================
A2 的验收判据里写着一句「**类别错分必须能被测试发现**」。这句话的对象不是模型，
是**这把尺子**：如果评估脚本自己把行/列读反、或把分母算错，那么一个把红认成橙的
模型照样能拿到一张漂亮的召回表 —— 错分就"发现不了"了。所以这里既验正常路径，
也**逐个变异**：把实现故意改坏，断言判据必须变红。

三个变异对应三个实测过的坑（都在 `eval_class_recall.py` 文件头里）：

  M1 行/列对调     —— ultralytics 是「行=预测、列=真值」，sklearn 是反的。
                      对调后召回与精确整体互换，报表仍然自洽、只是说的是另一件事。
  M2 分母只取类别块 —— 漏检不计入分母，「一整类全漏」会因为分母塌成 0
                      被当成「没有样本」，正好从报表里消失。
  M3 类别对混淆不判 —— 召回过了就放行，于是某一类的错分全涌向同一类也发现不了。

另有三条**结构**检查，钉住两个不许漂的数：建矩阵的 `conf` 必须显式传、
且等于 `pipeline.launch.py` 的 `confidence_threshold`；`live_infer_record.py`
的录像阈值必须与线上工作点一致。
"""

import ast
import os
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)                # 与其它脚本同一套 import 约定
import eval_class_recall as E           # noqa: E402
from block_classes import DEFAULT_CLASSES   # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def make(names, rows):
    """从 ``{预测类: {真值类: 个数}}`` 造一个 (nc+1)x(nc+1) 计数矩阵。

    行 = 预测、列 = 真值，最后一行/列是背景 —— 与 ultralytics 同构。
    测试里手写矩阵最容易把行写错，所以这里用**名字**写，落成矩阵由这一个函数管。
    """
    nc = len(names)
    bg = 'bg'
    grid = [[0] * (nc + 1) for _ in range(nc + 1)]
    for pred, row in rows.items():
        for true, n in row.items():
            i = names.index(pred) if pred in names else nc
            j = names.index(true) if true in names else nc
            grid[i][j] = n
    return grid


def approx(a, b, tol=1e-9):
    return a is not None and b is not None and abs(a - b) <= tol


# ----------------------------------------------------------------------
# 1. 正常路径：手算得出的召回/精确
# ----------------------------------------------------------------------
def part_1():
    print("\n===== 1. 已知答案的矩阵：召回/精确/漏检/虚警 =====")
    names = ['a', 'b', 'c']
    #       真值 a  b  c   bg
    # 预测 a [10  0  0   2]
    #      b [ 1  7  0   0]
    #      c [ 0  3  5   0]
    #      bg[ 9  0  5   0]
    m = make(names, {
        'a': {'a': 10, 'bg': 2},
        'b': {'a': 1, 'b': 7},
        'c': {'b': 3, 'c': 5},
        'bg': {'a': 9, 'c': 5},
    })
    r, p, s = E.per_class_scores(m, 3)
    # 分母是**整列**：a 的真值 = 10+1+0+9 = 20（漏检的 9 也算）
    check("a 的真值数 = 20（含漏检）", s[0] == 20, "实际 %d" % s[0])
    check("b 的真值数 = 10", s[1] == 10, "实际 %d" % s[1])
    check("c 的真值数 = 10", s[2] == 10, "实际 %d" % s[2])
    check("召回 a = 10/20", approx(r[0], 0.5), "实际 %r" % r[0])
    check("召回 b = 7/10", approx(r[1], 0.7), "实际 %r" % r[1])
    check("召回 c = 5/10", approx(r[2], 0.5), "实际 %r" % r[2])
    # 精确的分母是**行和**：a 被预测了 10+0+0+2 = 12 次
    check("精确 a = 10/12", approx(p[0], 10 / 12.0), "实际 %r" % p[0])
    check("精确 b = 7/8", approx(p[1], 7 / 8.0), "实际 %r" % p[1])
    check("精确 c = 5/8", approx(p[2], 5 / 8.0), "实际 %r" % p[2])
    check("漏检 a = 9（背景行、a 列）", E.missed_per_class(m, 3)[0] == 9,
          "实际 %d" % E.missed_per_class(m, 3)[0])
    check("虚警 a = 2（a 行、背景列）", E.false_alarms_per_class(m, 3)[0] == 2,
          "实际 %d" % E.false_alarms_per_class(m, 3)[0])
    return m, names


# ----------------------------------------------------------------------
# 2. 混淆对：排序、且**不含背景**
# ----------------------------------------------------------------------
def part_2(m, names):
    print("\n===== 2. 类别间混淆对 =====")
    c = E.top_confusions(m, 3, names)
    check("列出 2 对（背景不算类别对）", len(c) == 2, "实际 %d 对：%r" % (len(c), c))
    check("最严重的是 b→c 占 30%",
          len(c) == 2 and c[0][0] == 'b' and c[0][1] == 'c' and approx(c[0][3], 0.3),
          "实际 %r" % (c[0] if c else None,))
    check("第二是 a→b 占 5%",
          len(c) == 2 and c[1][0] == 'a' and approx(c[1][3], 0.05),
          "实际 %r" % (c[1] if len(c) > 1 else None,))
    # a 漏了 9 个，那是「没找到」不是「认错成另一类」—— 不能出现在类别对里
    check("漏检（背景）不出现在类别对里",
          all(x[0] != 'bg' and x[1] != 'bg' for x in c), "%r" % (c,))


# ----------------------------------------------------------------------
# 3. M1 变异：行/列对调 —— 召回与精确互换
# ----------------------------------------------------------------------
def part_3(m, names):
    print("\n===== 3. M1 变异：行/列对调必须能被发现 =====")
    r, p, _ = E.per_class_scores(m, 3)
    # 变异体：把「行=预测」当成「行=真值」读 —— 于是召回拿到的是行和分母，即精确
    wrong = [approx(r[i], p[i]) for i in range(3)]
    # 这个矩阵是不对称的，所以对调后**必须**与正确答案不同
    check("对调后的召回 ≠ 正确的召回（矩阵不对称，能区分）",
          not all(wrong), "两者都是 %r / %r —— 这个夹具区分不出行/列，得换个矩阵"
          % (r, p))
    check("正确的召回是手算的 [0.5, 0.7, 0.5]",
          approx(r[0], 0.5) and approx(r[1], 0.7) and approx(r[2], 0.5),
          "实际 %r" % (r,))
    check("精确与召回确实不同（否则本变异是空转）",
          not all(approx(p[i], r[i]) for i in range(3)),
          "精确 %r 与召回 %r 一样" % (p, r))


# ----------------------------------------------------------------------
# 4. M2 变异：分母只取类别块 —— 「整类全漏」会静默消失
# ----------------------------------------------------------------------
def part_4():
    print("\n===== 4. M2 变异：分母口径（漏检必须计入真值数）=====")
    names = ['a', 'b']
    #       真值 a  b   bg
    # 预测 a [ 5  0   0]
    #      b [ 0  0   0]      ← b 一个都没预测出来
    #      bg[ 0  8   0]      ← b 的 8 个全漏
    m = make(names, {
        'a': {'a': 5},
        'bg': {'b': 8},
    })
    r, _, s = E.per_class_scores(m, 2)
    check("b 的真值数 = 8（整列，含漏检）", s[1] == 8, "实际 %d" % s[1])
    check("b 的召回 = 0.0（不是 None）", r[1] == 0.0, "实际 %r" % r[1])
    check("b 的召回不是 None —— None 会被当成「没有样本」而跳过",
          r[1] is not None, "实际 %r" % r[1])

    # 变异体：分母只数类别块（行 0..nc-1），漏检那行不算 —— b 的分母塌成 0
    block_sum_b = sum(m[i][1] for i in range(2))
    check("变异体的分母确实塌成 0（所以本变异不是空转）", block_sum_b == 0,
          "类别块内 b 列有 %d 个" % block_sum_b)

    ok, items = E.gate(r, s, names, [], min_recall=0.9, min_support=1)
    check("b 全漏 → 判定失败", not ok, "gate 却通过了")
    check("失败条目点名了 b",
          any(mark == '失败' and 'b' in text for mark, text in items),
          "%r" % (items,))


# ----------------------------------------------------------------------
# 5. M3 变异：类别对混淆必须单独判，不能只看召回
# ----------------------------------------------------------------------
def part_5():
    print("\n===== 5. M3 变异：单对混淆超限必须判失败 =====")
    names = ['a', 'b']
    # a 有 100 个真值，92 个对、8 个被判成 b → 召回 0.92
    m = make(names, {
        'a': {'a': 92},
        'b': {'a': 8, 'b': 100},
    })
    r, _, s = E.per_class_scores(m, 2)
    check("召回 a = 0.92（过了 0.90 的召回闸）", approx(r[0], 0.92), "实际 %r" % r[0])
    c = E.top_confusions(m, 2, names)
    check("混淆对里 a→b 占 8%",
          len(c) == 1 and c[0][0] == 'a' and approx(c[0][3], 0.08), "%r" % (c,))

    ok, items = E.gate(r, s, names, c, min_recall=0.5, max_pair_share=0.05)
    check("召回闸放松后，仍被**单对混淆**判失败", not ok, "gate 却通过了：%r" % (items,))
    check("失败条目点名了「被判成」",
          any(mark == '失败' and '被判成' in text for mark, text in items),
          "%r" % (items,))
    # 反向：把上限抬到 10% 就该过 —— 证明判的是那一对、不是随手失败
    ok2, _ = E.gate(r, s, names, c, min_recall=0.5, max_pair_share=0.10)
    check("上限抬到 10% 后通过（判的是那一对，不是恒假）", ok2)


# ----------------------------------------------------------------------
# 6. 支撑数不足的类：明确「跳过」，不许沉默
# ----------------------------------------------------------------------
def part_6():
    print("\n===== 6. 真值太少的类：报「跳过」，不当通过 =====")
    names = ['a', 'b']
    m = make(names, {'a': {'a': 30}, 'bg': {'b': 3}})
    r, _, s = E.per_class_scores(m, 2)
    ok, items = E.gate(r, s, names, [], min_recall=0.9, min_support=20)
    check("b（真值 3 个）被标为「跳过」",
          any(mark == '跳过' and 'b' in text for mark, text in items), "%r" % (items,))
    check("b 没有被标成「通过」",
          not any(mark == '通过' and text.strip().startswith('b ') for mark, text in items),
          "%r" % (items,))
    check("只有 b 被跳过时判定仍然通过（小样本不背锅）", ok, "%r" % (items,))
    check("a 照常判定并通过",
          any(mark == '通过' and text.strip().startswith('a ') for mark, text in items),
          "%r" % (items,))


# ----------------------------------------------------------------------
# 7. 报表：每一类都得出现，一个不许少
# ----------------------------------------------------------------------
def part_7():
    print("\n===== 7. 报表必须逐类列出（不许汇总掩盖）=====")
    nc = len(DEFAULT_CLASSES)
    names = list(DEFAULT_CLASSES)
    m = make(names, {n: {n: 40} for n in names})        # 完美对角
    r, p, s = E.per_class_scores(m, nc)
    c = E.top_confusions(m, nc, names)
    text, ok = E.format_report(
        m, nc, names, r, p, s, c,
        E.missed_per_class(m, nc), E.false_alarms_per_class(m, nc),
        type('A', (), dict(model='m.pt', data='d', conf=0.5, iou=0.45, imgsz=640,
                           min_recall=0.9, min_support=20, max_pair_share=0.1))())
    missing = [n for n in names if n not in text]
    check("7 个类别名都出现在报表里", not missing, "缺：%r" % (missing,))
    check("完美对角 → 判定通过", ok, "gate 却失败了")
    check("报表里有「工作点」一行（conf 看得见）", 'conf=0.500' in text,
          "报表里找不到工作点")
    check("报表的表头标了「漏检」这一列", '漏检' in text, "表头没标背景列")


# ----------------------------------------------------------------------
# 8. 结构：两个不许漂的数
# ----------------------------------------------------------------------
def _pipeline_conf():
    """从 pipeline.launch.py 抠出 confidence_threshold 的默认值。"""
    path = os.path.join(REPO, 'ros2_ws', 'src', 'yolo_bringup', 'launch',
                        'pipeline.launch.py')
    if not os.path.isfile(path):
        return None
    import re
    text = open(path, encoding='utf-8').read()
    m = re.search(r"'confidence_threshold'\s*,\s*default_value\s*=\s*'([0-9.]+)'", text)
    return float(m.group(1)) if m else None


def _val_call_has_conf():
    """`eval_class_recall.py` 里调 `model.val(...)` 时有没有显式传 conf。"""
    path = os.path.join(HERE, 'eval_class_recall.py')
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == 'val':
            if any(k.arg == 'conf' for k in node.keywords):
                return True
    return False


def _module_imports_ultralytics_at_top():
    path = os.path.join(HERE, 'eval_class_recall.py')
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in tree.body:                  # 只看模块顶层
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            mod = getattr(node, 'module', None) or ''
            names = [a.name for a in getattr(node, 'names', [])]
            if 'ultralytics' in mod or 'ultralytics' in names:
                return True
    return False


def _live_infer_conf():
    """`live_infer_record.py` 里 `--conf` 的默认值。"""
    path = os.path.join(HERE, 'live_infer_record.py')
    if not os.path.isfile(path):
        return None
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == 'add_argument':
            if node.args and isinstance(node.args[0], ast.Constant) \
                    and node.args[0].value == '--conf':
                for k in node.keywords:
                    if k.arg == 'default' and isinstance(k.value, ast.Constant):
                        return float(k.value.value)
    return None


def part_8():
    print("\n===== 8. 结构：工作点不许漂 =====")
    check("model.val() 显式传了 conf（否则 ultralytics 把它悄悄改写成 0.25，"
          "矩阵建在一个你不知道的阈值上）",
          _val_call_has_conf(),
          "eval_class_recall.py 里找不到带 conf= 的 model.val(...) 调用")
    pipe = _pipeline_conf()
    check("能读到 pipeline.launch.py 的 confidence_threshold", pipe is not None)
    check("DEFAULT_CONF == pipeline.launch.py 的 confidence_threshold",
          pipe is not None and approx(E.DEFAULT_CONF, pipe),
          "评估用 %r，线上是 %r" % (E.DEFAULT_CONF, pipe))
    live = _live_infer_conf()
    check("能读到 live_infer_record.py 的 --conf 默认值", live is not None)
    check("录像阈值 == 线上工作点（录像要显示真正会去抓的框）",
          live is not None and approx(live, E.DEFAULT_CONF),
          "录像用 %r，线上是 %r —— 差值会让审核看到实际不会动的框"
          % (live, E.DEFAULT_CONF))
    check("纯逻辑部分不在模块顶层 import ultralytics（离线可 import）",
          not _module_imports_ultralytics_at_top())


def part_9(m, names):
    """矩阵表里两个背景轴的**标注**必须与它所装的数一致。

    这条是补的漏：第一版把末列标成「漏检」、末行标成「虚警」—— 正好反了。
    而当时的检查只断言「'漏检' 这三个字出现在报表里」，**不问它在哪个轴**，
    所以照样全绿。真正该问的是：底行装的是不是 FN、末列装的是不是 FP。
    """
    print("\n===== 9. 矩阵表的两个背景轴：标注必须与所装的数一致 =====")
    table = E.format_matrix(m, 3, names)
    lines = table.splitlines()

    check("表头末列标「虚警」（末列装的是 matrix[i][nc] = FP）",
          lines[0].rstrip().endswith("虚警"), "表头：%r" % lines[0].rstrip()[-20:])

    bottom = [ln for ln in lines if ln.startswith("漏检")]
    check("底行标「漏检」（底行装的是 matrix[nc][j] = FN）",
          len(bottom) == 1, "找到 %d 行以「漏检」开头" % len(bottom))
    if bottom:
        nums = [int(x) for x in bottom[0].split()[1:4]]
        # 本夹具 a 的 FN = 9（手算，见 part_1）；若标反就会读到 FP = 2
        want = E.missed_per_class(m, 3)
        check("底行的数 = 每类漏检 %r" % (want,), nums == want,
              "底行读到 %r —— 若等于 %r 就是轴标反了"
              % (nums, E.false_alarms_per_class(m, 3)))

    parts = lines[1].split()          # "a  10  0  0 |  2" → 中间那个 '|' 要跳过
    fa_row = [int(x) for x in parts[1:4]] + [int(parts[-1])]
    check("首行末列的数 = a 的虚警 %d" % E.false_alarms_per_class(m, 3)[0],
          fa_row == [int(m[0][j]) for j in range(3)] + [int(m[0][3])],
          "首行读到 %r" % (fa_row,))


def part_10():
    """子集裁剪：被裁掉的类要**并进背景**，不是删掉。

    「今天先做易区分的子集」这种做法有个陷阱：把难类从任务里删掉之后，
    它的方块被认成留下的类，报表上看不见了 —— 可现实里那是那个槽位被塞了
    错块。所以裁剪必须把被裁类的行/列并进背景，让留下的类**虚警涨上去**。
    """
    print("\n===== 10. 子集裁剪：被裁的类是并进背景，不是删掉 =====")
    names = ['a', 'b', 'c']
    #       真值 a  b  c  bg
    # 预测 a[10  0  0   2]
    #      b[ 0  8  0   0]
    #      c[ 0  0  6   1]
    #      bg[1  0  0   0]
    m = [[10, 0, 0, 2],
         [0, 8, 0, 0],
         [0, 0, 6, 1],
         [1, 0, 0, 0]]

    sm, sn = E.subset_matrix(m, 3, names, ['a', 'c'])
    check("裁剪后类别表 = ['a','c']", sn == ['a', 'c'], "%r" % (sn,))
    check("裁剪后是 3x3（两类 + 背景）",
          len(sm) == 3 and all(len(r) == 3 for r in sm), "%r" % (sm,))
    # 手算：b 那一行/列并进背景。
    #   对 a：真值 b 的 0 个 + 原来背景列的 2 → 末列 2
    #   对 c：原来背景列 1 → 末列 1
    #   背景行对 a：原来背景行的 1 + b 行预测成 a 的 0 → 1
    #   角上：b→b 的 8 个（既被裁的预测、也被裁的真值）也得收着
    want = [[10, 0, 2],
            [0, 6, 1],
            [1, 0, 8]]
    check("裁剪结果与手算一致", sm == want,
          "实际 %r" % (sm,))
    check("总数不丢（28 = 28）",
          sum(sum(r) for r in sm) == sum(sum(r) for r in m),
          "裁剪后 %d，原矩阵 %d" % (sum(sum(r) for r in sm), sum(sum(r) for r in m)))

    # 语义要害：4 个**真值是 b、却被预测成 a** 的方块，裁掉 b 之后必须变成
    # **a 的虚警**，不能消失。
    #   行 = 预测 [a, b, bg]，列 = 真值 [a, b, bg]
    #   m2[0][1] = 4 就是「预测 a、真值 b」—— 行/列别写反（ultralytics 是行=预测）
    m2 = [[5, 4, 0],
          [0, 0, 0],
          [0, 0, 0]]
    sm2, sn2 = E.subset_matrix(m2, 2, ['a', 'b'], ['a'])
    r2, p2, s2 = E.per_class_scores(sm2, 1)
    check("裁剪后是 2x2（a + 背景）",
          len(sm2) == 2 and all(len(r) == 2 for r in sm2), "%r" % (sm2,))
    check("裁掉 b 后，4 个被认成 a 的 b 仍在 a 的虚警里",
          sm2[0][1] == 4, "a 的虚警（背景列）= %d，期望 4。实际矩阵 %r"
          % (sm2[0][1], sm2))
    check("因此 a 的精确 = 5/9（不是 1.0）",
          approx(p2[0], 5 / 9.0), "实际 %r —— 等于 1.0 就说明用的是「删掉」而不是「并入」" % p2[0])
    check("a 的召回不受影响（真值还是 5 个）", approx(r2[0], 1.0) and s2[0] == 5,
          "召回 %r 真值 %r" % (r2[0], s2))

    # 变异：把「并入」写成「删掉」—— 总数会丢，且精确虚高
    deleted = [[m[i][j] for j in range(3) if names[j] in ('a', 'c')]
               for i in range(3) if names[i] in ('a', 'c')]
    check("「删掉」变体确实会丢计数（所以本变异不是空转）",
          sum(sum(r) for r in deleted) != sum(sum(r) for r in m),
          "删掉后还剩 %d，原 %d" % (sum(sum(r) for r in deleted), sum(sum(r) for r in m)))

    # 入参校验
    for bad, why in ((['a', 'zzz'], "名字不在表里"),
                     ([], "空表"),
                     (['a', 'a'], "重复")):
        try:
            E.subset_matrix(m, 3, names, bad)
            check("非法 keep %r（%s）应抛 ValueError" % (bad, why), False, "没抛")
        except ValueError:
            check("非法 keep %r（%s）抛 ValueError" % (bad, why), True)


def main():
    print(__doc__.strip().splitlines()[1])
    m, names = part_1()
    part_2(m, names)
    part_3(m, names)
    part_4()
    part_5()
    part_6()
    part_7()
    part_8()
    part_9(m, names)
    part_10()
    print("\n%d PASS  %d FAIL" % (N_PASS, N_FAIL))
    if FAILS:
        print("失败项：")
        for f in FAILS:
            print("  - %s" % f)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
