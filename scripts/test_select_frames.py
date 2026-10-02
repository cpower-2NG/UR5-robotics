#!/usr/bin/env python3
"""``select_frames.py`` 的自检 —— 不需要相机、不需要板子、不需要真的连拍。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
这一步是**唯一会把数据默默扔掉**的地方：判据写松了，同一张图进几十遍、
标签数字好看而模型没多学；写紧了，把真正的光照变化当噪声合并掉、
光照多样性白丢。两种错都**不报错、不崩**，只是数据集悄悄变差。
所以阈值那几条要用**恰好卡在边界上**的输入钉死。

第 [4] 节跑真 CLI，用的是合成图 —— 它同时证明「顺序固定、跑两次一样」。
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import select_frames as S                                          # noqa: E402

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


def flat(v, h=100, w=100):
    return np.full((h, w), v, dtype=np.uint8)


def with_n_pixels(base, n, delta=30, h=100, w=100):
    """在 base 上把开头 n 个像素改掉 delta —— n 用来卡 ±1 个像素的边界。"""
    g = base.copy()
    total = h * w
    assert n <= total
    g.reshape(-1)[:n] = (int(base[0, 0]) + delta) % 256
    return g


def main():
    # --------------------------------------- [1] differs() 的判据（含边界 ±1 像素）
    print('\n[1] differs() —— 阈值是「严格多于 2%」，±1 个像素就要翻转')
    SZ = 100 * 100                       # 10000 像素 → 2% = 200 像素
    a = flat(100)
    check('完全一样的两张 → 相同', not S.differs(a, flat(100)))
    check('差 1 个像素 → 相同', not S.differs(a, with_n_pixels(a, 1)))
    check('差 199 个像素 → 相同（阈值下）', not S.differs(a, with_n_pixels(a, 199)))
    check('差 200 个像素 → 相同（恰好 2%，不是「多于」）',
          not S.differs(a, with_n_pixels(a, 200)))
    check('差 201 个像素 → **不同**（跨过 2%）', S.differs(a, with_n_pixels(a, 201)))
    check('整帧都差 → 不同', S.differs(a, flat(200)))

    print('\n[1b] 两个阈值各自都要到量 —— 单靠一个都能骗过去')
    # 差一大片但每处都很轻（自动曝光的整体漂移）：灰阶差 10 < 25 → 不该算不同，
    # 否则每一次曝光微调都会被当成一个新的「光照状态」收进来。
    drift = flat(110)
    check('整帧偏 10 个灰阶（<25）→ 相同，不当成新光照',
          not S.differs(a, drift, frac=0.02, levels=25))
    check('整帧偏 25 个灰阶（不 >25）→ 仍算相同',
          not S.differs(a, flat(125), frac=0.02, levels=25))
    check('整帧偏 26 个灰阶（>25）→ 不同', S.differs(a, flat(126), frac=0.02, levels=25))
    # 反过来：差得够狠但面积不够（一块 50×50 = 2500 像素的阴影）——
    # 这块该收！2500 > 2% → 不同。用它确认面积阈值不是「全帧」级别的严苛。
    shade = flat(100)
    shade[10:60, 10:60] = 200
    check('50×50 区域变亮（2500 px > 2%）→ 不同', S.differs(a, shade))

    print('\n[1c] 尺寸不同要报错，不能悄悄按 broadcast 算')
    try:
        S.differs(flat(100, 100, 100), flat(100, 50, 50))
        check('尺寸不同抛 ValueError', False, '没抛')
    except ValueError:
        check('尺寸不同抛 ValueError', True)
    except Exception as e:                                          # noqa: BLE001
        check('尺寸不同抛 ValueError', False, '抛的是 %s' % type(e).__name__)

    # --------------------------------------------------- [2] 亮度窗口
    print('\n[2] 亮度窗口 —— 全黑和过曝的帧不该占名额')
    imgs = {'dark': flat(10), 'ok': flat(100), 'bright': flat(240), 'edge_up': flat(224)}
    kept = S.select(list(imgs), lambda p: imgs[p], bmin=40.0, bmax=225.0)
    names = [p for p, _, _ in kept]
    check('亮度 10（全黑）被丢掉', 'dark' not in names)
    check('亮度 240（过曝）被丢掉', 'bright' not in names)
    check('亮度 100 留下', 'ok' in names)
    check('亮度 224（窗口内）留下', 'edge_up' in names)
    check('窗口是开区间：40 与 225 本身不该被收',
          not S.select(['x'], lambda p: flat(40)) and
          not S.select(['y'], lambda p: flat(225)))

    # --------------------------------- [3] 贪心：和**已保留的每一帧**比，不是和上一帧比
    print('\n[3] 贪心 semantics —— A≈B≈C 里的 B、C 都该被 A 挡掉')
    A = flat(100)
    B = with_n_pixels(A, 5)              # 与 A 几乎一样
    C = flat(200)                        # 与 A 差很多
    D = with_n_pixels(A, 5)              # 与 A 几乎一样（排在 C 后面）
    pool = {'A': A, 'B': B, 'C': C, 'D': D}
    kept = S.select(list(pool), lambda p: pool[p])
    names = [p for p, _, _ in kept]
    check('A、C 留下', names == ['A', 'C'], '实际 %r' % (names,))
    check('B 被 A 挡掉（不是「和上一帧比」那种写法）', 'B' not in names)
    check('D 被 A 挡掉，**不是**因为和 C 比',
          'D' not in names)

    print('\n[3b] 同一份输入、同样顺序 → 跑两次逐位一致（可复现）')
    import random
    rng = random.Random(7)
    frames = [flat(int(rng.uniform(60, 200))) for _ in range(40)]
    pool2 = {'f%02d' % i: g for i, g in enumerate(frames)}
    order = list(pool2)
    k1 = [p for p, _, _ in S.select(order, lambda p: pool2[p])]
    k2 = [p for p, _, _ in S.select(order, lambda p: pool2[p])]
    check('同样顺序下两次结果逐位一致', k1 == k2, '%r vs %r' % (k1, k2))

    print('\n[3c] 贪心**确实**依赖顺序 —— 把它当成已知性质，别当成 bug')
    # 这不是缺陷：早进来的帧挡住后来相似的帧，是贪心挑选的定义。
    # 用一条「亮度链」把两种顺序的结果钉死（不用随机图，免得时灵时不灵）：
    #   A=100 — B=120 — C=140 — D=160，相邻差 20（≤25，算同一状态），
    #   隔一个差 40（>25，算不同）。于是 A≈B≈C≈D 首尾不相邻。
    chain = {'A': flat(100), 'B': flat(120), 'C': flat(140), 'D': flat(160)}
    fwd = [p for p, _, _ in S.select(['A', 'B', 'C', 'D'], lambda p: chain[p])]
    rev = [p for p, _, _ in S.select(['B', 'D', 'A', 'C'], lambda p: chain[p])]
    check('正序 [A,B,C,D] → 选 {A,C}', fwd == ['A', 'C'], '%r' % (fwd,))
    check('另序 [B,D,A,C] → 选 {B,D}',
          rev == ['B', 'D'], '%r' % (rev,))
    check('两种顺序的**集合**确实不同（固有性质，已在文档写明）',
          set(fwd) != set(rev))

    print('\n[3d] 链上每一对的关系都如预期（[3c] 的结论建立在这上面）')
    check('A~B 相同', not S.differs(chain['A'], chain['B']))
    check('B~C 相同', not S.differs(chain['B'], chain['C']))
    check('A~C 不同', S.differs(chain['A'], chain['C']))
    check('B~D 不同', S.differs(chain['B'], chain['D']))
    check('A~D 不同', S.differs(chain['A'], chain['D']))

    # ------------------------------------------- [4] 真 CLI（合成图，落盘）
    print('\n[4] CLI 端到端 —— 合成图落盘，跑真脚本')
    import cv2
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'src')
        out = os.path.join(td, 'out')
        os.makedirs(src)
        # 5 张重复的 + 1 张不同光照的 + 1 张全黑（该被亮度窗口丢）
        for i in range(5):
            cv2.imwrite(os.path.join(src, 'img_%02d.png' % i), flat(100))
        cv2.imwrite(os.path.join(src, 'img_05.png'), flat(190))
        cv2.imwrite(os.path.join(src, 'img_06.png'), flat(5))
        r = subprocess.run(
            [sys.executable,
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'select_frames.py'),
             src, '--out', out, '--pattern', 'img_*'],
            capture_output=True, text=True)
        check('退出码 0', r.returncode == 0, r.stdout.strip()[-300:] + r.stderr.strip()[-300:])
        got = sorted(os.listdir(out))
        n_img = len([f for f in got if f.startswith('img_')])
        check('7 张里选出 2 张（5 重复合并成 1，全黑丢掉）',
              n_img == 2, '实际留下 %r' % (got,))
        check('写了 selected_sources.txt', 'selected_sources.txt' in got)
        check('输出是顺序编号 img_0000/img_0001',
              'img_0000.png' in got and 'img_0001.png' in got, '%r' % (got,))
        man = open(os.path.join(out, 'selected_sources.txt')).read()
        check('清单里记了来源路径与亮度', '<-' in man and 'mean=' in man)
        check('清单行数 = 选出帧数', len(man.strip().splitlines()) == n_img)

        print('\n[4b] 源目录里有 info.json 时要顺手带过去（标注器要内参）')
        import json
        with open(os.path.join(src, 'info.json'), 'w') as f:
            json.dump({'fx': 1.0, 'fy': 2.0}, f)
        out2 = os.path.join(td, 'out2')
        subprocess.run([sys.executable,
                        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'select_frames.py'),
                        src, '--out', out2], capture_output=True, text=True)
        check('info.json 被复制到输出目录',
              os.path.exists(os.path.join(out2, 'info.json')))

        print('\n[4c] 目录的先后不该影响结果 —— 结果是「文件集合」的函数')
        # 建两个目录，各自放几张。用 **chain** 那组图，因为 [3c] 已经证明
        # 它们的挑选结果**确实**依赖顺序 —— 所以这条测试是真有牙的：
        # 若 CLI 退回「每个目录各自排序」，两个目录的先后就会改变结果。
        for d, vals in (('d1', (100, 140)), ('d2', (120, 160))):
            os.makedirs(os.path.join(src, d), exist_ok=True)
            for i, v in enumerate(vals):
                cv2.imwrite(os.path.join(src, d, 'img_%02d.png' % i), flat(v))
        o_fwd = os.path.join(td, 'o_fwd')
        o_rev = os.path.join(td, 'o_rev')
        for outdir, order in ((o_fwd, ('d1', 'd2')), (o_rev, ('d2', 'd1'))):
            r = subprocess.run(
                [sys.executable,
                 os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'select_frames.py')]
                + [os.path.join(src, d) for d in order] + ['--out', outdir],
                capture_output=True, text=True)
            if r.returncode != 0:
                check('目录顺序 %r 跑通' % (order,), False, r.stdout + r.stderr)
        m_fwd = open(os.path.join(o_fwd, 'selected_sources.txt')).read()
        m_rev = open(os.path.join(o_rev, 'selected_sources.txt')).read()
        check('--dirs A B 与 --dirs B A 选出**同一批**帧',
              m_fwd == m_rev,
              '\n' + m_fwd + '\n---vs---\n' + m_rev)

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
