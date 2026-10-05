#!/usr/bin/env python3
"""``relabel_by_color.py`` 的交付前关卡。

（沿用本仓库 ``scripts/test_*.py`` 的约定：``check()`` + PASS/FAIL 计数，
退出码 0 = 通过，2 = 环境没准备好。）

为什么要有它
============
这个工具**在给别人改标签**，而改错的症状全是静默的：

  · 把紫块判成橙 → 标签照样写出来，训练照样跑完，只是那一类的召回率虚低；
  · 类别号与名字错位 → 每个号都还在 ``[0,nc)`` 里，一声不吭；
  · 人工改判没生效 → 我训的就是一份**自己已经知道是错**的标签。

前两条正是 ``block_classes.py`` 开头警告的那类故障。所以这里测的不是"代码能跑"，
而是**这台尺子在几个已知会翻车的地方还有没有刻度**。

这些用例不是编出来的，每一条都对应一次真实翻车
------------------------------------------------
[2] 环形距离：色相跨 0° 时线性距离会算成 347°。真实的紫在 340~355°，
    与红(8°) 只差十几度 —— 用线性距离，紫会被判成红。
[3] 框心权重：**这条是本文件存在的主要理由**。工具第一版按"框内哪类像素多"
    投票，于是「大紫块 + 边缘一小块高饱和黄邻块」被判成黄。而 YOLO 的框是
    **以目标为中心**的，邻居只会从框边渗进来 —— 所以权重必须是框心高斯，
    且**不能**再乘饱和度（紫的饱和天生就低，一乘就等于给紫全体降权）。
[4] 组内饱和度归一化：同一批积木换一次曝光，绝对饱和度就变。橙/棕这对只差
    8° 的邻居，靠饱和度分胜负；不归一化的话，换一张图就可能整类翻面。
[5] 掩码：白纸(S≈24)与木桌面(S≈30)必须被 S_MIN 挡住，否则浅色像素会进来投票。
[6] 人工改判：MANUAL_OVERRIDES 是**人眼看过的结论**，必须真落到写出的标签里。
"""

import collections
import importlib.util
import os
import sys
import tempfile

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import block_classes                    # noqa: E402

try:
    import numpy as np
except ImportError:
    np = None
try:
    import cv2
except ImportError:
    cv2 = None

N_PASS = N_FAIL = N_SKIP = 0
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


def skip(name, why):
    global N_SKIP
    N_SKIP += 1
    print('  SKIP  %s   ← %s' % (name, why))


def load_module():
    """把 relabel_by_color.py 当模块导入（它 import 时会自己把 scripts/ 加进 path）。"""
    spec = importlib.util.spec_from_file_location(
        'relabel_by_color', os.path.join(HERE, 'relabel_by_color.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ----------------------------------------------------------------------
# 造假像素：不依赖真数据集与 cv2，直接喂 classify()
# ----------------------------------------------------------------------
def fabricate(regions):
    """把 ``[(色相度, 饱和度, 框心权重, 个数), ...]`` 拼成 box_pixels 那种 dict。"""
    hue, sat, cw = [], [], []
    for h, s, w, n in regions:
        hue.append(np.full(n, float(h)))
        sat.append(np.full(n, float(s)))
        cw.append(np.full(n, float(w)))
    hue = np.concatenate(hue)
    sat = np.concatenate(sat)
    return dict(hue=hue, sat=sat, val=np.full(hue.shape, 200.0),
                cw=np.concatenate(cw), n_px=len(hue), n_box=len(hue))


# ----------------------------------------------------------------------
def section_ring(rc):
    print('\n[1] 环形统计：跨 0° 不能按线性算')
    check('circ_mean([356, 4]) 落在 0 附近，不是 180',
          rc.circ_dist(rc.circ_mean([356.0, 4.0]), 0.0) < 5.0,
          '%.1f' % rc.circ_mean([356.0, 4.0]))
    check('circ_dist(350, 10) == 20（不是 340）',
          abs(rc.circ_dist(350, 10) - 20.0) < 1e-9, '%.1f' % rc.circ_dist(350, 10))
    # 紫在 340~355°，离红(8°) 只有十几度 —— 线性距离会把它甩到 347°
    check('circ_dist_h(355, 8) == 13 —— 紫块与红中心其实很近',
          abs(float(rc.circ_dist_h(np.array([355.0]), 8.0)[0]) - 13.0) < 1e-9)


def section_pixelwise(rc):
    print('\n[2] 像素级最近类：每个像素判给已知中心，不做聚类')
    med = 150.0
    # 逐类各取一个"标准样"：色相取中心值、sn 取中心值
    for name, h, sn in rc.CENTERS:
        px = fabricate([(h, sn * med, 1.0, 50)])
        got = rc.classify(px, med)['cls']
        check('色相 %5.1f° / sn %.2f → %s' % (h, sn, name), got == name, got)

    # 关键的跨 0° 用例：355° 的高饱和像素是**红**，不是紫
    px = fabricate([(355.0, 1.07 * med, 1.0, 50)])
    check('色相 355° / sn 1.07 → red（跨 0° 的邻居，不是 purple）',
          rc.classify(px, med)['cls'] == 'red',
          rc.classify(px, med)['cls'])


def section_center_weight(rc):
    """本文件的主要理由：框心权重，且不乘饱和度。"""
    print('\n[3] 框心权重：大邻居在框边，目标在框心 → 必须判成框心那个')
    med = 150.0
    # 目标：紫块，压在框心（cw 高），饱和天生低（70）
    # 邻居：黄块，只在框边（cw 低），面积更大（900 > 300）、饱和更高（180）
    px = fabricate([(308.0, 70.0, 0.90, 300),      # 紫，框心
                    (40.0, 180.0, 0.15, 900)])     # 黄，框边
    got = rc.classify(px, med)['cls']
    check('大而高饱和的框边邻居，压不过小而低饱和的框心目标 → purple',
          got == 'purple', got)

    # 反过来：目标换到框边、邻居在框心，结论必须跟着翻 —— 否则上面那条
    # 可能只是"面积"或"饱和度"碰巧对了，而不是框心在起作用。
    px2 = fabricate([(308.0, 70.0, 0.15, 300), (40.0, 180.0, 0.90, 900)])
    got2 = rc.classify(px2, med)['cls']
    check('把两者对调（黄在框心）→ 结论跟着翻成 yellow', got2 == 'yellow', got2)


def section_normalisation(rc):
    """橙/棕只差 8°，靠饱和度分胜负 —— 绝对值换一张图就变，必须按组归一化。"""
    print('\n[4] 组内饱和度归一化：同一 sn、不同曝光 → 同一类')
    # 色相取橙/棕正中间，让饱和度成为唯一的分胜负者
    h = 24.5
    # 亮的那一组：中位 S=150
    a = rc.classify(fabricate([(h, 1.15 * 150.0, 1.0, 50)]), 150.0)['cls']
    # 暗的那一组：中位 S=120，同一个 sn 对应的**绝对**饱和度完全不同
    b = rc.classify(fabricate([(h, 1.15 * 120.0, 1.0, 50)]), 120.0)['cls']
    check('同一 sn 在不同曝光组里判成同一类（%s / %s）' % (a, b), a == b)

    # 这条断言必须真的有载荷：不归一化（钉死 med_s）时，暗组要翻面
    c = rc.classify(fabricate([(h, 1.15 * 120.0, 1.0, 50)]), 150.0)['cls']
    check('若不按组归一化，暗组会翻面（证明上一条不是恒真）', c != b,
          '归一化与不归一化给出同一结果 %s —— 第 4 节测不出东西' % c)


def section_mask(rc):
    print('\n[5] box_pixels：白纸/木桌面被 S_MIN 挡住，权重是框心高斯')
    if cv2 is None or np is None:
        skip('box_pixels 全部用例', '没装 opencv')
        return
    # 造一张 100×100 的图：框是 (20,80)²，但饱和方块只占 (30,70)² ——
    # 刻意**小于框**，这样框内四周留着低饱和的"白纸"，掩码才有东西可挡。
    img = np.full((100, 100, 3), 230, dtype=np.uint8)       # BGR 近白
    img[30:70, 30:70] = (0, 0, 200)                          # 纯红(饱和 255)
    px = rc.box_pixels(img, 0.5, 0.5, 0.6, 0.6)
    check('读出像素了', px is not None)
    if px is None:
        return
    check('低饱和的白纸没有混进来（读到的都是高饱和像素）',
          float(np.min(px['sat'])) >= rc.S_MIN,
          '最小 S=%.0f' % float(np.min(px['sat'])))
    check('框内四周的白纸被挡住（有效像素远少于框内总像素）',
          px['n_px'] < px['n_box'] * 0.6,
          '%d / %d' % (px['n_px'], px['n_box']))
    # 权重形状。⚠️ 别断言 max == 1.0：框尺寸是偶数时中心落在 4 个像素之间，
    # 最近的像素离中心也有半个像素，权重必然略小于 1（实测 0.998）。
    # 真正要守的性质是「中心高、边角低」，不是某个具体数值。
    cw = px['cw']
    check('框心权重的最大值接近 1（中心像素几乎不受罚）',
          float(cw.max()) > 0.99, '%.4f' % float(cw.max()))
    check('框心权重都 > 0（边缘只是降权，不是删掉）',
          float(cw.min()) > 0.0, '%.4f' % float(cw.min()))
    check('中心与边角的权重差 3 倍以上（确实是高斯，不是平权）',
          float(cw.max()) / float(cw.min()) > 3.0,
          'max/min = %.2f' % (float(cw.max()) / float(cw.min())))

    # 太小 / 太空的框要返回 None，而不是硬判一类出来
    check('过小的框返回 None', rc.box_pixels(img, 0.5, 0.5, 0.01, 0.01) is None)
    white = np.full((100, 100, 3), 235, dtype=np.uint8)      # 全白：没有有效像素
    check('全白（没有有效像素）的框返回 None',
          rc.box_pixels(white, 0.5, 0.5, 0.6, 0.6) is None)


def section_manual(rc):
    print('\n[6] 人工改判：结论必须落到写出的标签里')
    valid = [c[0] for c in rc.CENTERS]
    bad = {k: v for k, v in rc.MANUAL_OVERRIDES.items() if v not in valid}
    check('每条人工改判的类别名都在类别表里', not bad, str(bad))
    check('改判表非空（本次确实有一处人眼复核的结论）',
          len(rc.MANUAL_OVERRIDES) >= 1)

    yaml_names = [c[0] for c in rc.CENTERS]
    check('CENTERS 的顺序与 DEFAULT_CLASSES 逐位相同（类别号 = 下标）',
          yaml_names == list(block_classes.DEFAULT_CLASSES),
          '%s vs %s' % (yaml_names, list(block_classes.DEFAULT_CLASSES)))

    # 端到端：数据集若在，就翻开文件看那一行到底写了什么类别号
    ds = os.path.join(REPO, 'block_dataset_7cls')
    if not os.path.isdir(ds):
        skip('改判已写进数据集文件', 'block_dataset_7cls/ 不在（数据集不入库）')
        return
    for (grp, idx), want in sorted(rc.MANUAL_OVERRIDES.items()):
        hit = None
        for split in ('train', 'val'):
            ldir = os.path.join(ds, split, 'labels')
            if not os.path.isdir(ldir):
                continue
            cand = sorted(f for f in os.listdir(ldir) if f.startswith(grp + '__'))
            if cand:
                hit = os.path.join(ldir, cand[0])
                break
        if hit is None:
            skip('改判 %s #%d 已落盘' % (grp, idx), '数据集里没有这个采集组')
            continue
        rows = [l for l in open(hit).read().splitlines() if l.strip()]
        cid = int(rows[idx].split()[0])
        check('%s #%d 在 %s 里写的是 %s' % (grp, idx, os.path.basename(hit), want),
              yaml_names[cid] == want, '文件里是 %s' % yaml_names[cid])


def section_mutation(rc):
    """把三处改坏，对应的断言必须失效 —— 否则第 2/3/4 节测的是空气。"""
    print('\n[7] 变异：改坏判据，第 2/3/4 节的断言必须跟着变红')
    src = open(os.path.join(HERE, 'relabel_by_color.py')).read()

    def load(text, tag):
        d = tempfile.mkdtemp(prefix='relabel_')
        p = os.path.join(d, 'relabel_%s.py' % tag)
        with open(p, 'w') as f:
            f.write(text)
        spec = importlib.util.spec_from_file_location('relabel_%s' % tag, p)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    def mutate(tag, old, new):
        assert old in src, '变异锚点没找到：%r' % old
        return load(src.replace(old, new, 1), tag)

    med = 150.0

    # (a) 框心权重改成"再乘饱和度" —— 就是工具第一版的那个错
    mm = mutate('wtsat', '    w = px["cw"]', '    w = px["cw"] * px["sat"]')
    check('变异「权重乘上饱和度」→ 框边的大黄邻居翻盘',
          mm.classify(fabricate([(308.0, 70.0, 0.90, 300),
                                 (40.0, 180.0, 0.15, 900)]), med)['cls'] != 'purple',
          '改坏了结论却没变 —— 第 3 节那条断言没盯着这一行')

    # (b) 环形距离改成线性 —— 跨 0° 的紫会被甩到红那边
    mm = mutate('lin', '    d = np.abs(arr - c) % 360.0\n'
                       '    return np.minimum(d, 360.0 - d)',
                '    return np.abs(arr - c)')
    check('变异「环形距离改线性」→ 355° 的红被判成别的类',
          mm.classify(fabricate([(355.0, 1.07 * med, 1.0, 50)]), med)['cls'] != 'red',
          '改坏了结论却没变 —— 第 2 节那条跨 0° 断言没盯着这一行')

    # (c) 不按组归一化，钉死一个中位饱和度
    mm = mutate('norm', '    sn = px["sat"] / med_s', '    sn = px["sat"] / 150.0')
    a = mm.classify(fabricate([(24.5, 1.15 * 120.0, 1.0, 50)]), 120.0)['cls']
    b = rc.classify(fabricate([(24.5, 1.15 * 120.0, 1.0, 50)]), 120.0)['cls']
    check('变异「不按组归一化」→ 暗曝光组的结论翻面', a != b,
          '改坏了结论却没变（都是 %s）—— 第 4 节测不出东西' % a)

    # (d) 框心高斯改成平权 —— 第 5 节那条 max/min 断言必须能发现
    if cv2 is not None:
        mm = mutate('flat', 'cw = np.exp(-(dx * dx + dy * dy) / (2.0 * SIGMA * SIGMA))',
                    'cw = np.ones_like(dx)')
        img = np.full((100, 100, 3), 230, dtype=np.uint8)
        img[30:70, 30:70] = (0, 0, 200)
        mpx = mm.box_pixels(img, 0.5, 0.5, 0.6, 0.6)
        check('变异「框心高斯改平权」→ max/min 降到 1，第 5 节会红',
              mpx is not None and float(mpx['cw'].max() / mpx['cw'].min()) <= 3.0,
              '改坏了形状却没变 —— 第 5 节那条 max/min 断言没盯着这一行')


def main():
    if np is None:
        print('需要 numpy（退 2）')
        return 2
    print('=' * 68)
    rc = load_module()
    section_ring(rc)
    section_pixelwise(rc)
    section_center_weight(rc)
    section_normalisation(rc)
    section_mask(rc)
    section_manual(rc)
    section_mutation(rc)
    print('\n%d PASS  %d FAIL  %d SKIP' % (N_PASS, N_FAIL, N_SKIP))
    if FAILS:
        print('未通过：\n  - ' + '\n  - '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
