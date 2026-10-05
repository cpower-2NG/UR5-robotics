#!/usr/bin/env python3
"""``label_blocks.py`` 的自检 —— 不需要显示器、不需要相机、不需要真图。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
`label_blocks.py` 是**手工标注**那条路的唯一入口，而它的失效方式特别安静：
旧版把类别号读进来后**丢掉**，写回去时统一写 ``0``。于是「打开一份多类别
标签、按一下 D 跳到下一张、退出」这串动作会把整份标签的类别全抹成第 0 类 ——
没有报错、没有提示，文件时间戳变了而已。等发现时，训练数据已经被污染了。

所以本文件的核心断言不是「格式对不对」，而是 **往返幂等**：
`load() → save()` 必须逐字节还原原文件。把那行 ``0`` 改回去，这一条立刻红。

怎么在没有显示器的环境下跑起来：模块级就执行 ``plt.subplots()`` 与
``plt.show()``，其中 ``show()`` 会**阻塞**（模块自己把后端设成 TkAgg，
盖过 MPLBACKEND 环境变量）。所以在 exec 之前先把 ``plt.show`` 换成空函数。
"""
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
LABELER = os.path.join(HERE, "label_blocks.py")

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
        print('  FAIL  %s%s' % (name, ('   ← %s' % detail) if detail else ''))


# 在子进程里 headless 跑 label_blocks 的引导脚本。用子进程是因为这个模块
# **导入即执行**（还带 argparse 与 plt.show），每个用例需要各自的参数。
BOOTSTRAP = r'''
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.show = lambda *a, **k: None      # 模块级 show() 会阻塞，先掐掉
import importlib.util, sys, json
spec = importlib.util.spec_from_file_location("lb", sys.argv[1])
mod = importlib.util.module_from_spec(spec)
sys.argv = ["label_blocks.py"] + sys.argv[2:]
spec.loader.exec_module(mod)
print("__READY__" + json.dumps({"classes": mod.CLASS_NAMES}))
json.dump(mod.__dict__.get("_probe", None), sys.stdout)
'''


def run_headless(img_dir, lbl_dir, *extra, probe_many=None):
    """起一个 headless 的 label_blocks 进程；返回 CompletedProcess。

    ``probe_many=N``：绕过 ``block_classes.parse_classes`` 的调色板校验，
    假装有 N 个类别名 —— 用来验「>9 类拒绝启动」那条守卫（正常传参够不着，
    因为调色板只有 8 色）。
    """
    boot = BOOTSTRAP
    if probe_many:
        boot = BOOTSTRAP.replace(
            'spec.loader.exec_module(mod)',
            'import os\n'
            'sys.path.insert(0, os.path.dirname(os.path.abspath(spec.origin)))\n'
            'import block_classes\n'
            'block_classes.parse_classes = (\n'
            '    lambda *a, **k: ["c" + str(i) for i in range('
            + str(probe_many) + ')])\n'
            'spec.loader.exec_module(mod)')
    return subprocess.run(
        [sys.executable, '-c', boot, LABELER,
         '--images', img_dir, '--labels', lbl_dir] + list(extra),
        capture_output=True, text=True, timeout=120)


def import_labeler(img_dir, lbl_dir, classes=None):
    """在**本进程**里导入 label_blocks（只需一次，用于读/写往返的断言）。

    同样要先掐掉 plt.show：模块自己 `matplotlib.use("TkAgg")`，会盖过
    MPLBACKEND 环境变量，不掐就会卡在 show() 上（实测超时 60 s 被 kill）。
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.show = lambda *a, **k: None

    spec = importlib.util.spec_from_file_location("lb", LABELER)
    mod = importlib.util.module_from_spec(spec)
    argv = ["label_blocks.py", "--images", img_dir, "--labels", lbl_dir]
    if classes:
        argv += ["--classes", classes]
    old = sys.argv
    sys.argv = argv
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.argv = old
    return mod


def write_img(img_dir, name="img_0000.jpg"):
    import cv2
    os.makedirs(img_dir, exist_ok=True)
    cv2.imwrite(os.path.join(img_dir, name), np.zeros((480, 640, 3), np.uint8))


def read_lines(path):
    return [l for l in open(path).read().splitlines() if l.strip()]


def main():
    with tempfile.TemporaryDirectory() as td:
        img_dir = os.path.join(td, 'img')
        lbl_dir = os.path.join(td, 'lbl')
        os.makedirs(lbl_dir, exist_ok=True)
        write_img(img_dir)
        lbl = os.path.join(lbl_dir, 'img_0000.txt')

        # ------------------------------------------------ [1] 类别表
        print('\n[1] 类别表来自 --classes，顺序即类别号')
        m = import_labeler(img_dir, lbl_dir, classes='cyan,red,yellow')
        check('类别表就是传进去的顺序',
              m.CLASS_NAMES == ['cyan', 'red', 'yellow'], str(m.CLASS_NAMES))
        check('默认类别表是 block_classes.DEFAULT_CLASSES',
              m.DEFAULT_CLASSES == ['red', 'orange', 'yellow', 'cyan', 'purple'],
              str(m.DEFAULT_CLASSES))
        check('导入后已装好第一张图（img_list 非空）', len(m.img_list) == 1)

        # ------------------------------------------------ [2] 写出的格式
        print('\n[2] save() —— 逐框写类别号，5 字段')
        m.boxes[:] = [(2, 0.5, 0.5, 0.1, 0.1), (0, 0.3, 0.3, 0.2, 0.2)]
        m.save()
        rows = read_lines(lbl)
        check('写了两行', len(rows) == 2, str(rows))
        fields = [r.split() for r in rows]
        check('每行 5 个字段', all(len(f) == 5 for f in fields), str(fields))
        check('**第一列是类别号**，且按框写出（2 然后 0）',
              [int(f[0]) for f in fields] == [2, 0], str(fields))
        check('坐标是 6 位小数',
              all(len(f[1].split('.')[1]) == 6 for f in fields), str(fields))

        # ------------------------------------------------ [3] 往返幂等（核心）
        print('\n[3] **load() → save() 必须逐字节还原**（旧版会把类别抹成 0）')
        original = open(lbl).read()
        m.load(0)
        check('load() 读回来的框数对', len(m.boxes) == 2, str(m.boxes))
        check('**读回来的类别号没丢**（不是全 0）',
              [b[0] for b in m.boxes] == [2, 0], str(m.boxes))
        m.save()
        check('**往返后文件逐字节不变**',
              open(lbl).read() == original,
              '%r → %r' % (original, open(lbl).read()))
        check('  类别号在框元组里是第一位',
              all(len(b) == 5 for b in m.boxes), str(m.boxes))

        # ------------------------------------------------ [4] 三类的往返
        print('\n[4] 三类混排的往返（各类都活下来）')
        m.boxes[:] = [(0, 0.1, 0.1, 0.05, 0.05), (1, 0.4, 0.4, 0.05, 0.05),
                      (2, 0.7, 0.7, 0.05, 0.05), (0, 0.9, 0.9, 0.05, 0.05)]
        m.save()
        orig3 = open(lbl).read()
        m.load(0)
        m.save()
        check('四框三类的往返逐字节不变', open(lbl).read() == orig3,
              '%r → %r' % (orig3, open(lbl).read()))

        # ------------------------------------------------ [5] 越界类别号
        print('\n[5] 标签里的类别号超出当前类别表 → 不崩，明确降级并告警')
        with open(lbl, 'w') as f:
            f.write('7 0.5 0.5 0.1 0.1\n')      # 只有 3 类，7 是越界的
        m.load(0)
        check('越界号降级为 0（而不是崩）',
              [b[0] for b in m.boxes] == [0], str(m.boxes))

    # ------------------------------------------------ [6] 独立的进程级用例
    print('\n[6] 参数校验（子进程，因为这些在导入期就 sys.exit）')
    with tempfile.TemporaryDirectory() as td:
        img_dir = os.path.join(td, 'img')
        os.makedirs(os.path.join(td, 'lbl'), exist_ok=True)
        write_img(img_dir)

        r = run_headless(img_dir, os.path.join(td, 'lbl'), '--classes', 'red,red')
        check('类别名重复 → 非 0 退出', r.returncode != 0, '实际 %d' % r.returncode)
        check('  说清是重复', '重复' in (r.stdout + r.stderr), (r.stdout + r.stderr)[-200:])

        r = run_headless(img_dir, os.path.join(td, 'lbl'), '--classes', 'red,,yellow')
        check('空项 → 非 0 退出', r.returncode != 0, '实际 %d' % r.returncode)

        # 调色板只有 8 个颜色名，所以**走 CLI 到不了 9 类** —— 先钉住这个事实，
        # 免得以后有人以为「9 类上限」是在 CLI 上生效的。
        r = run_headless(img_dir, os.path.join(td, 'lbl'), '--classes',
                         'red,orange,yellow,green,cyan,blue,purple,pink')
        check('8 个调色板颜色 = CLI 能给出的最大类别数，能正常启动',
              r.returncode == 0, (r.stdout + r.stderr)[-300:])
        check('  启动横幅里确实是 8 类',
              '8 类' in (r.stdout + r.stderr), (r.stdout + r.stderr)[-300:])

        r = run_headless(img_dir, os.path.join(td, 'lbl'), '--classes',
                         'red,orange,yellow,green,cyan,blue,purple,pink,magenta')
        check('第 9 个名字只能是调色板外的（magenta）→ 被调色板校验拒绝，'
              '所以 CLI 到不了 9 类上限',
              r.returncode != 0 and '调色板' in (r.stdout + r.stderr),
              (r.stdout + r.stderr)[-200:])

        # 「>9 类就拒绝」那条守卫走 CLI 够不着（调色板只有 8 色），只能绕过
        # parse_classes 直接验。这不是形式主义：守卫是给「以后调色板变长」
        # 或「有人开 allow_unknown」准备的，没有人验过它就等于没有。
        r = run_headless(img_dir, os.path.join(td, 'lbl'), probe_many=10)
        check('**10 类（绕过调色板校验）→ 拒绝启动**，并说清切换键只有 1..9',
              r.returncode != 0 and '9' in (r.stdout + r.stderr),
              (r.stdout + r.stderr)[-300:])
        check('  拒绝的理由是按键方案，不是别的',
              '1..9' in (r.stdout + r.stderr), (r.stdout + r.stderr)[-300:])

        r = run_headless(img_dir, os.path.join(td, 'lbl'), '--classes', 'red,yellow,cyan')
        check('合法三类的 headless 启动退出码为 0（不卡在 show()）',
              r.returncode == 0, (r.stdout + r.stderr)[-300:])

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
