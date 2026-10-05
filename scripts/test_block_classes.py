#!/usr/bin/env python3
"""类别表的**交付前关卡**：调色板 / 默认表 / 数据集三份名字必须说的是同一件事。

（沿用本仓库 ``scripts/test_*.py`` 的约定：退出码 0 = 通过，2 = 环境没准备好。）

为什么要有它
============
``block_classes.py`` 的模块 docstring 把类别表定成**单点定义**，但有一处它管不到、
而且**故意**管不到：``detector_node.py`` 的类别名来自**权重文件**自带的
``model.names``（线上以权重为准，改了调色板不会影响已训好的模型）。
所以「调色板」与「权重」之间那条缝是设计上就存在的，只能在**交付前**拿关卡堵：

    调色板（本文件） ←→ data.yaml 的 names ←→ dataset_manifest.json 的 class_names
                              ↑
                        权重文件的 model.names（训完才知道，这里只告警不判死）

为什么是「顺序」而不是「集合」
------------------------------
类别号 = 列表下标。三份表只要有**一处顺序不同**，标签与名字就整体错位，
而每个类别号仍然落在 ``[0, nc)`` 里 —— ``parse_label_line`` 一声不吭，
训练照样跑完，只是模型学到的名字全是错的。所以这里比的是**逐位相等**，
不是「集合相同」。

数据集缺席时
============
本仓的 ``block_dataset/`` 不在版本控制里（体积大），所以常见的运行结果是
「没找到数据集」—— 那不是失败，第 2 节会明说跳过，只跑第 1 节的表内自洽。
"""

import json
import os
import sys

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)                # 与其它脚本同一套 import 约定
import block_classes                    # noqa: E402

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


# ----------------------------------------------------------------------
# 纯比较函数 —— 测试要能直接喂假数据进来（真数据集不在仓里）
# ----------------------------------------------------------------------
def names_mismatch(yaml_names, manifest_names):
    """两份名字列表是否**逐位**不同。返回 None 表示一致，否则返回一句人话。

    不返回 bool 的原因：不一致的**症状**（哪一位错了）比结论本身有用得多。
    这是**唯一**同时看两份表的地方，所以顺序、长度、大小写都在这里判。
    """
    y = [str(x).strip() for x in (yaml_names or [])]
    m = [str(x).strip() for x in (manifest_names or [])]
    if not y and not m:
        return None                     # 两份都空 —— 不算错，但调用方会另说一句
    if len(y) != len(m):
        return ('data.yaml 有 %d 类，manifest 有 %d 类 —— 两者必须逐位对应'
                % (len(y), len(m)))
    for i, (a, b) in enumerate(zip(y, m)):
        if a != b:
            return ('第 %d 位对不上：data.yaml 是 %r，manifest 是 %r。\n'
                    '    类别号 = 下标，第 %d 位错 = 该类的所有标签语义全反，'
                    '且不会有任何报错。' % (i, a, b, i))
    return None


def raises_with(mod, spec_str, needle):
    """``parse_classes`` 是否抛了 ValueError，**且消息里有 needle**。

    为什么要 needle：一条输入常能被好几道检查挡住（空项也会撞字符集检查），
    只问「抛没抛」的话，把某一道检查删掉仍然抛 —— 变异就永远测不出来。
    """
    try:
        mod.parse_classes(spec_str)
        return False
    except ValueError as e:
        return needle in str(e)


def palette_problems(names=None):
    """表内自洽性：默认表的每个名字都得在调色板里，且能被 parse_classes 原样吃回去。"""
    names = list(names or block_classes.DEFAULT_CLASSES)
    bad = []
    for n in names:
        if n not in block_classes.PALETTE:
            bad.append('%r 在 DEFAULT_CLASSES 里但调色板里没有' % n)
    try:
        if block_classes.parse_classes(names) != names:
            bad.append('parse_classes 往返之后变了：%r' % names)
    except ValueError as e:
        bad.append('默认类别表自己过不了 parse_classes：%s' % e)
    return bad


# ----------------------------------------------------------------------
def section_palette():
    print('\n[1] 调色板与默认类别表自洽')
    p = palette_problems()
    check('默认表的每一项都在调色板里，且能原样解析回来', not p, '; '.join(p))

    wrong = [(n,) for n in block_classes.PALETTE
             if not block_classes._NAME_RE.match(n)]
    check('调色板里每个名字都是合法类别名', not wrong, str(wrong))

    # parse_classes 的每一条拒绝理由都要**由它自己那一条**挡住。
    # ⚠️ 只断言「抛了」是不够的（实测）：`red,,yellow` 去掉空项检查后仍会被
    #    **字符集检查**挡住，`Red` 去掉字符集检查后仍会被**调色板检查**挡住 ——
    #    于是「抛了」永远为真，变异改坏了也照样绿。所以这里连**消息**一起断言。
    for spec, why, needle in [
            ('red,,yellow', '空项（多余逗号）', '空项'),
            ('red,', '末尾逗号', '空项'),
            ('red,red', '重复', '重复'),
            ('Red', '大写', '只允许'),
            ('', '整个空', '至少要有'),
            ('red,notacolor', '调色板里没有', '调色板里没有')]:
        check('parse_classes 拒绝「%s」（且是那一条在挡）' % why,
              raises_with(block_classes, spec, needle),
              '要么放行了 %r，要么是被别的检查挡下的（消息里没有 %r）'
              % (spec, needle))

    # 名字 → 号 → 名字，往返
    names = block_classes.DEFAULT_CLASSES
    rt = all(block_classes.class_index(n, names) == i
             for i, n in enumerate(names))
    check('class_index 往返一致（顺序即索引）', rt)
    try:
        block_classes.class_index('nope', names)
        check('class_index 对未知名字**抛异常**（不是 -1/None）', False, '没抛')
    except ValueError:
        check('class_index 对未知名字**抛异常**（不是 -1/None）', True)
    try:
        block_classes.color_of('nope')
        check('color_of 对未知名字抛异常', False, '没抛')
    except ValueError:
        check('color_of 对未知名字抛异常', True)

    # 顺序不同的表必须被当成不同的表 —— 这是本文件存在的理由
    swapped = [names[1], names[0]] + names[2:]
    check('顺序换过的表 ≠ 原表（逐位比较，不是集合比较）',
          block_classes.parse_classes(swapped) != names,
          '换顺序居然相等 —— 那说明这份表根本没在比顺序')
    print('  默认类别表：%s' % block_classes.describe(names))


def find_datasets():
    """找出仓里所有 dataset 目录（有 data.yaml 或 dataset_manifest.json 的）。"""
    out = []
    for entry in sorted(os.listdir(REPO)):
        d = os.path.join(REPO, entry)
        if not os.path.isdir(d):
            continue
        y = os.path.join(d, 'data.yaml')
        m = os.path.join(d, 'dataset_manifest.json')
        if os.path.exists(y) or os.path.exists(m):
            out.append((d, y, m))
    return out


def read_yaml_names(path):
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    nm = doc.get('names')
    if isinstance(nm, dict):            # ultralytics 写回来是 {0: 'block', ...}
        nm = [nm[k] for k in sorted(nm)]
    return list(nm or []), int(doc.get('nc', len(nm or [])))


def section_datasets():
    print('\n[2] 交付关卡：data.yaml ↔ dataset_manifest.json')
    found = find_datasets()
    if not found:
        print('  （仓里没有数据集目录 —— 跳过。'
              '这一节要在 build_dataset.py 跑完之后、交付之前再跑。）')
        return
    for d, y, m in found:
        tag = os.path.basename(d)
        if not (os.path.exists(y) and os.path.exists(m)):
            print('  · %s：只有一半（data.yaml=%s manifest=%s）—— 跳过'
                  % (tag, os.path.exists(y), os.path.exists(m)))
            continue
        y_names, nc = read_yaml_names(y)
        with open(m) as f:
            man = json.load(f)
        m_names = man.get('class_names') or []
        prob = names_mismatch(y_names, m_names)
        check('%s：data.yaml 的 names 与 manifest 的 class_names 逐位一致'
              % tag, prob is None, prob or '')
        check('%s：nc 与 names 长度一致' % tag, nc == len(y_names),
              'nc=%d，names=%d 项' % (nc, len(y_names)))
        # 多类别时才要求与默认表对上（单类别 'block' 是旧数据集，允许）
        if len(y_names) > 1:
            check('%s：多类别的顺序与 DEFAULT_CLASSES 前缀一致（%s）'
                  % (tag, ",".join(y_names)),
                  y_names == block_classes.DEFAULT_CLASSES[:len(y_names)],
                  '实际 %s，默认表前 %d 项是 %s'
                  % (y_names, len(y_names),
                     block_classes.DEFAULT_CLASSES[:len(y_names)]))
        else:
            print('  · %s：单类别 %s —— 是旧数据集，不参与多类顺序校验'
                  % (tag, y_names))
        counts = man.get('class_counts') or {}
        zero = [k for k, v in counts.items() if not v]
        if zero:
            print('  ⚠️ %s：这些类在这批数据里一个框都没有：%s'
                  '（不是召回率 0，是根本没有样本）' % (tag, ", ".join(zero)))


def section_mutation():
    """变异：把 block_classes 的某一条校验改坏，第 1 节的断言必须失效。

    没有这一步的话，「parse_classes 拒绝空项」这类断言可能只是因为**别的原因**
    在抛异常（比如 unknown 检查先命中），而我们以为守住了。
    """
    import importlib.util
    import tempfile

    print('\n[3] 变异：把校验改坏，对应的拒绝必须失效')
    src_path = os.path.join(HERE, 'block_classes.py')
    src = open(src_path).read()

    def load(text, tag):
        d = tempfile.mkdtemp(prefix='blockcls_')
        p = os.path.join(d, 'block_classes_%s.py' % tag)
        with open(p, 'w') as f:
            f.write(text)
        spec = importlib.util.spec_from_file_location('block_classes_%s' % tag, p)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    muts = [
        ('不查空项', '    if any(not n for n in names):', '    if False:',
         'red,,yellow', '空项'),
        ('不查重复', '    if dup:', '    if False:', 'red,red', '重复'),
        ('不查非法字符', '    if bad:', '    if False:', 'Red', '只允许'),
        ('调色板检查失效', '    if not allow_unknown:', '    if False:',
         'red,notacolor', '调色板里没有'),
    ]
    for label, old, new, spec_str, needle in muts:
        assert old in src, '变异锚点没找到：%r' % old
        mm = load(src.replace(old, new), 'mut%d' % len(FAILS))
        check('变异「%s」→ parse_classes(%r) 不再报「%s」'
              % (label, spec_str, needle),
              not raises_with(mm, spec_str, needle),
              '改坏了还在报同一条 —— 说明第 1 节那条断言没盯着这一行')

    # class_index 改成「返回 -1」必须能被第 1 节那种断言发现
    old = ('    try:\n        return names.index(name)\n    except ValueError:')
    assert old in src
    mm = load(src.replace(old, '    try:\n        return names.index(name)\n'
                              '    except ValueError:\n        return -1\n'
                              '    if False:'), 'mutidx')
    check('变异「class_index 返回 -1 而不抛」→ 不再抛异常',
          mm.class_index('nope', ['red']) == -1,
          '还是抛了 —— 说明变异没生效')


def main():
    print('=' * 68)
    section_palette()
    section_datasets()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
