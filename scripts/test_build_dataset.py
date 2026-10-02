#!/usr/bin/env python3
"""``build_dataset.py`` 的自检 —— 不需要相机、不需要板子、不需要真图。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
这份脚本唯一不可替代的职责是**按排布切分**。它错了不会报错、不会崩，
只会让 val 指标虚高 —— 而虚高的 val 恰恰是「模型上线才发现不行」的标准前奏。
所以第 [3] 节不只测「切得对」，还测**那条断言真的在拦**：
把 val 的排布混进 train，端到端必须看到 train 里出现该排布的文件名。

第 [1] 节把标签校验的边界钉死（`0` 与 `1` 是合法值，越界要看得出）。
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_dataset as B                                          # noqa: E402

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


def raises(fn, *a, **k):
    """返回异常信息，没抛就返回 None。"""
    try:
        fn(*a, **k)
    except Exception as e:                                          # noqa: BLE001
        return '%s: %s' % (type(e).__name__, e)
    return None


BOILER = {n: ('.png', [(0, 1)]) for n, _i, _l in B.SOURCES}


def listing(out, sub):
    """某个 split 下 images/ 里的文件名（显式传 out —— 不要做成闭包，
    第 [5] 节用的是另一个临时目录，闭包会悄悄指回第 [4] 节那个）。"""
    return sorted(os.listdir(os.path.join(out, sub, 'images')))


def write_tree(root, spec):
    """``spec = {排布名: (扩展名, [(帧号, 框数), ...])}`` —— 按 ``B.SOURCES`` 的**真实
    相对路径**造一棵假的 rounds 树。

    用真实路径而不是自造一张表，是因为子进程会重新 import 模块，
    测试进程里改 ``B.SOURCES`` 对它无效 —— 那正是第一版测试踩的坑。
    走真实路径还顺带把 SOURCES 表本身测了。
    """
    rel = {n: (i, l) for n, i, l in B.SOURCES}
    for name, (ext, frames) in spec.items():
        idir = os.path.join(root, rel[name][0])
        ldir = os.path.join(root, rel[name][1])
        os.makedirs(idir, exist_ok=True)
        os.makedirs(ldir, exist_ok=True)
        for i, nbox in frames:
            with open(os.path.join(idir, 'img_%04d%s' % (i, ext)), 'wb') as f:
                f.write(b'\xff\xd8\xff' if ext == '.jpg' else b'\x89PNG\r\n\x1a\n')
            with open(os.path.join(ldir, 'img_%04d.txt' % i), 'w') as f:
                for _b in range(nbox):
                    f.write('0 0.5 0.5 0.1 0.1\n')
    return root


def main():
    # ------------------------------------------- [1] parse_label_line（纯函数）
    print('\n[1] parse_label_line —— 标签校验，边界要钉死')
    cls, cx, cy, w, h = B.parse_label_line('0 0.5 0.4 0.1 0.2', 1)
    check('正常一行解析正确', (cls, cx, cy, w, h) == (0, 0.5, 0.4, 0.1, 0.2))
    check('  首尾空白/多空格容忍', B.parse_label_line('  0\t0.5   0.4 0.1 0.2 ', 1)[1] == 0.5)
    check('整幅框（0 0.5 0.5 1 1）合法', B.parse_label_line('0 0.5 0.5 1 1', 1)[3] == 1.0)
    check('贴边零宽不算越界（eps 容差）',
          B.parse_label_line('0 0.05 0.5 0.1 0.5', 1)[1] == 0.05)

    check('字段数不对 → 报错', raises(B.parse_label_line, '0 0.5 0.5 0.1', 1))
    check('非数字 → 报错', raises(B.parse_label_line, '0 x 0.5 0.1 0.2', 1))
    check('类别号越界 → 报错', raises(B.parse_label_line, '1 0.5 0.5 0.1 0.2', 1))
    check('类别号为负 → 报错', raises(B.parse_label_line, '-1 0.5 0.5 0.1 0.2', 1))
    check('坐标 >1 → 报错', raises(B.parse_label_line, '0 1.2 0.5 0.1 0.2', 1))
    check('坐标为负 → 报错', raises(B.parse_label_line, '0 -0.1 0.5 0.1 0.2', 1))
    check('宽为 0 → 报错', raises(B.parse_label_line, '0 0.5 0.5 0 0.2', 1))
    check('高为负 → 报错', raises(B.parse_label_line, '0 0.5 0.5 0.1 -0.2', 1))
    # 这条是「中心在画面内、框却伸到画面外」—— 只看 cx 范围是抓不住的
    check('**框伸出画面**（cx 合法但 cx+w/2>1）→ 报错',
          raises(B.parse_label_line, '0 0.98 0.5 0.1 0.2', 1))
    check('  上面那条不是被坐标范围挡下的（cx=0.98 本身合法）',
          B.parse_label_line('0 0.98 0.5 0.02 0.2', 1)[1] == 0.98)
    check('报错信息里带出处', 'where' in (raises(B.parse_label_line, 'x', 1) or '')
          or ':1' in (raises(B.parse_label_line, 'x', 1, 'f.txt:1') or ''))

    # ------------------------------------------------------- [2] pair_frames
    print('\n[2] pair_frames —— 配对要报出「缺谁」，不能静默丢帧')
    with tempfile.TemporaryDirectory() as td:
        for s in ('a', 'b', 'c'):
            open(os.path.join(td, s + '.png'), 'w').close()
        for s in ('a', 'b', 'd'):
            open(os.path.join(td, s + '.txt'), 'w').close()
        pairs, only_img, only_lbl = B.pair_frames(
            [os.path.join(td, s + '.png') for s in 'abc'],
            [os.path.join(td, s + '.txt') for s in 'abd'])
        check('配对出 a、b', sorted(B.stem_of(p) for p, _ in pairs) == ['a', 'b'])
        check('只有图的是 c', only_img == ['c'])
        check('只有标签的是 d', only_lbl == ['d'])

        # 两份标签同 stem（img_0000.txt 出现在两个目录里）必须报错，
        # 不能悄悄挑一份 —— 悄悄挑就是「一半标签来自别处」这类事故
        check('同一 stem 有两份标签 → 报错',
              raises(B.pair_frames, [os.path.join(td, 'a.png')],
                     [os.path.join(td, 'x', 'a.txt'), os.path.join(td, 'y', 'a.txt')]))
        check('同一 stem 有两张图 → 报错',
              raises(B.pair_frames,
                     [os.path.join(td, 'a.png'), os.path.join(td, 'x', 'a.png')],
                     [os.path.join(td, 'a.txt')]))

    # -------------------------------------------------------- [3] plan_split
    print('\n[3] plan_split —— 排布绝不允许两边都出现')
    names = ['r00', 'r01', 'r02', 'r03', 'r05', 'r06a1', 'r06a2']
    tr, va = B.plan_split(names, ['r02', 'r06a2'])
    check('val 是点名的两个', va == ['r02', 'r06a2'])
    check('train 是其余五个', tr == ['r00', 'r01', 'r03', 'r05', 'r06a1'])
    check('**两边无交集**', not (set(tr) & set(va)))
    check('  且并集恰好是全体（没漏、没重）',
          sorted(tr + va) == sorted(names) and len(tr + va) == len(names))
    check('val 为空 → 报错', raises(B.plan_split, names, []))
    check('val 点了不存在的排布 → 报错', raises(B.plan_split, names, ['r99']))
    check('val 点了全体 → 报错（train 会空）', raises(B.plan_split, names, names))
    check('报错信息列出可选排布',
          'r00' in (raises(B.plan_split, names, ['r99']) or ''))

    # ------------------------------------ [4] 端到端 CLI：切分 + 产出布局
    print('\n[4] 端到端 —— 切分、命名前缀、data.yaml、非帧文件被列出')
    CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build_dataset.py')

    def run_cli(out, root, val, *extra):
        return subprocess.run([sys.executable, CLI, '--out', out, '--rounds-root', root,
                               '--val', val] + list(extra), capture_output=True, text=True)

    with tempfile.TemporaryDirectory() as td:
        root = os.path.join(td, 'rounds')
        spec = dict(BOILER)
        spec['r00'] = ('.png', [(0, 2), (1, 2)])
        spec['r01'] = ('.png', [(0, 3)])
        spec['r02'] = ('.png', [(0, 1), (1, 1), (2, 1)])
        write_tree(root, spec)
        # 非帧文件：混在数据目录里，必须**被列出来**而不是静默收进去
        for nm in ('info.json', 'click_frame.jpg', 'notes.txt'):
            open(os.path.join(root, 'r02', 'images', nm), 'w').close()

        out = os.path.join(td, 'ds')
        r = run_cli(out, root, 'r02')
        check('退出码 0', r.returncode == 0, r.stdout[-500:] + r.stderr[-500:])
        for nm in ('info.json', 'click_frame.jpg', 'notes.txt'):
            check('  %s 被列出来' % nm, nm in r.stdout, r.stdout[-500:])

        tr_i, va_i = listing(out, 'train'), listing(out, 'val')
        check('val 只有 r02 的 3 帧', va_i == ['r02__img_0000.png', 'r02__img_0001.png',
                                               'r02__img_0002.png'], str(va_i))
        check('train 是另外 6 个排布、共 7 帧',
              tr_i == ['r00__img_0000.png', 'r00__img_0001.png', 'r01__img_0000.png',
                       'r03__img_0000.png', 'r05__img_0000.png', 'r06a1__img_0000.png',
                       'r06a2__img_0000.png'], str(tr_i))

        # ⬇⬇ 这就是「按排布切分」的判据本身：拿文件名前缀反推排布，两边不许撞
        def arrs(fs):
            return set(f.split('__')[0] for f in fs)

        check('**train 与 val 的排布集合不相交**', not (arrs(tr_i) & arrs(va_i)),
              '%s vs %s' % (arrs(tr_i), arrs(va_i)))
        check('  val 里没有 r06a1/r06a2（点名的才进 val）', arrs(va_i) == {'r02'}, str(arrs(va_i)))
        check('  排布并集覆盖全部 7 个',
              arrs(tr_i) | arrs(va_i) == {n for n, _i, _l in B.SOURCES})
        check('标签与图同名同数',
              sorted(os.listdir(os.path.join(out, 'val', 'labels'))) ==
              [os.path.splitext(f)[0] + '.txt' for f in va_i])
        check('train 标签也是 7 份',
              len(os.listdir(os.path.join(out, 'train', 'labels'))) == 7)

        yml = open(os.path.join(out, 'data.yaml')).read()
        check('data.yaml **没有 path:** 行（D57）',
              not any(l.split(':')[0].strip() == 'path' for l in yml.splitlines()), yml)
        check('data.yaml 有 train/val/names/nc',
              all(k in yml for k in ('train: train/images', 'val: val/images',
                                     '0: block', 'nc: 1')), yml)

        man = json.load(open(os.path.join(out, 'dataset_manifest.json')))
        check('清单记了 train 框数 = 11', man['split']['train']['boxes'] == 11, str(man['split']))
        check('清单记了 val 框数 = 3', man['split']['val']['boxes'] == 3, str(man['split']))
        check('清单的排布名单与文件名一致',
              set(man['split']['val']['arrangements']) == arrs(va_i) and
              set(man['split']['train']['arrangements']) == arrs(tr_i))
        check('清单记了每帧的出处',
              man['files']['r00__img_0000.png']['image'].endswith('img_0000.png'))

        print('\n[4b] 图缺标签 → 退出码 1，且点名是谁')
        os.remove(os.path.join(root, 'r01', 'labels', 'img_0000.txt'))
        r = run_cli(os.path.join(td, 'ds2'), root, 'r02')
        check('退出码 1', r.returncode == 1, '实际 %d' % r.returncode)
        check('  输出点名 img_0000', 'img_0000' in r.stdout, r.stdout[-300:])
        check('  没有 Traceback', 'Traceback' not in r.stderr, r.stderr[-300:])

        print('\n[4c] 标签越界 → 退出码 1（错标签必须在入库前被挡下）')
        with open(os.path.join(root, 'r01', 'labels', 'img_0000.txt'), 'w') as f:
            f.write('0 0.5 0.5 1.4 0.1\n')
        r = run_cli(os.path.join(td, 'ds3'), root, 'r02')
        check('退出码 1', r.returncode == 1, '实际 %d' % r.returncode)
        check('  输出说明越界', '越' in r.stdout or '不在' in r.stdout, r.stdout[-300:])

        print('\n[4d] --val 点名不存在的排布 → 退出码 2（这是用法错，不是数据错）')
        r = run_cli(os.path.join(td, 'ds4'), root, 'nope')
        check('退出码 2', r.returncode == 2, '实际 %d' % r.returncode)

        print('\n[4e] --clean 真的在删残留（不是摆设）')
        os.remove(os.path.join(root, 'r01', 'labels', 'img_0000.txt'))    # 还原来
        with open(os.path.join(root, 'r01', 'labels', 'img_0000.txt'), 'w') as f:
            f.write('0 0.5 0.5 0.1 0.1\n' * 3)
        r = run_cli(out, root, 'r02', '--clean')
        check('退出码 0', r.returncode == 0, r.stdout[-400:])
        check('  先建出 train 7 + val 3',
              (len(listing(out, 'train')), len(listing(out, 'val'))) == (7, 3),
              str((listing(out, 'train'), listing(out, 'val'))))

        # 从源里撤掉一帧 —— 输出的旧文件不会自己消失，这正是 --clean 要对付的
        os.remove(os.path.join(root, B.SOURCES[0][1], 'img_0001.png'))
        os.remove(os.path.join(root, B.SOURCES[0][2], 'img_0001.txt'))
        r = run_cli(out, root, 'r02')
        check('  不 --clean 重跑：撤掉的帧留在输出里（残留是真会发生的）',
              'r00__img_0001.png' in listing(out, 'train'), str(listing(out, 'train')))
        r = run_cli(out, root, 'r02', '--clean')
        check('  加 --clean 重跑：撤掉的帧被清掉',
              'r00__img_0001.png' not in listing(out, 'train')
              and len(listing(out, 'train')) == 6, str(listing(out, 'train')))
        check('  排布仍不相交',
              not (arrs(listing(out, 'train')) & arrs(listing(out, 'val'))),
              str(listing(out, 'train')) + str(listing(out, 'val')))

    # ------------------------------------------ [5] --jpeg-quality 转码行为
    print('\n[5] --jpeg-quality —— 非 JPEG 转码、已是 JPEG 的原样复制')
    try:
        import cv2
    except ImportError:
        cv2 = None
    if cv2 is None:
        print('  SKIP  没装 opencv-python')
    else:
        with tempfile.TemporaryDirectory() as td:
            root = os.path.join(td, 'rounds')
            rng = np.random.default_rng(0)
            rel = {n: (i, l) for n, i, l in B.SOURCES}
            # 转码要真读得出图，所以这一节必须写真 PNG/JPEG（噪声图是最坏情况）
            for name, _i, _l in B.SOURCES:
                ext = '.jpg' if name.startswith('r06') else '.png'
                idir = os.path.join(root, rel[name][0])
                ldir = os.path.join(root, rel[name][1])
                os.makedirs(idir, exist_ok=True)
                os.makedirs(ldir, exist_ok=True)
                img = rng.integers(0, 255, (64, 64, 3), dtype=np.uint8)
                cv2.imwrite(os.path.join(idir, 'img_0000' + ext), img)
                with open(os.path.join(ldir, 'img_0000.txt'), 'w') as f:
                    f.write('0 0.5 0.5 0.1 0.1\n')

            out = os.path.join(td, 'ds')
            r = run_cli(out, root, 'r02', '--jpeg-quality', '90')
            check('退出码 0', r.returncode == 0, r.stdout[-400:] + r.stderr[-400:])
            va = listing(out, 'val')
            check('非 JPEG 源全部变成 .jpg', va == ['r02__img_0000.jpg'], str(va))
            check('  所有帧都成了 .jpg',
                  not any(f.endswith('.png') for f in listing(out, 'train') + va),
                  str(listing(out, 'train') + va))

            src_png = os.path.join(root, rel['r02'][0], 'img_0000.png')
            got_jpg = os.path.join(out, 'val', 'images', 'r02__img_0000.jpg')
            check('  PNG 转码后确实变小（噪声图是最坏情况）',
                  os.path.getsize(got_jpg) < os.path.getsize(src_png),
                  '%d vs %d' % (os.path.getsize(got_jpg), os.path.getsize(src_png)))
            # 已经是 JPEG 的**不做二次有损** —— 不然每装配一次就掉一次代
            src_jpg = os.path.join(root, rel['r06a1'][0], 'img_0000.jpg')
            got_jpg2 = os.path.join(out, 'train', 'images', 'r06a1__img_0000.jpg')
            check('已是 JPEG 的原样复制（字节相同，没有二次有损）',
                  open(got_jpg2, 'rb').read() == open(src_jpg, 'rb').read())

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
