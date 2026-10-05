#!/usr/bin/env python3
"""``palette_survey.py`` 的自检 —— 用**合成图**验它数得对不对、变异能不能变红。

（沿用本仓库 ``scripts/test_*.py`` 的约定：退出码 0 = 通过，2 = 环境没准备好。）

为什么值得钉
============
这把尺子是**用来定类别数的**，而类别数错了的后果是静默的：模型照样训完、
标签照样在范围内，只是分拣时两类混成一类。所以「尺子本身准不准」必须能被
单独验证 —— 不能拿它量出来的数去证明它自己准。

做法：合成几张色相已知的图，断言它数出的色团数/中心/名字与真值一致；
再用**变异**证明那些断言真的盯着代码（改坏一处 → 断言必须变红）。

四个变异各对应一种真实失效方式：

| 变异 | 现实里长什么样 |
|---|---|
| 去掉饱和度下限 | 白纸/木桌面（S≈24~30）被当成一个「类别」——而且面积最大，占比最高 |
| 去掉合并 | 一块积木表面的光照渐变被数成两三种颜色 → 类别虚高 |
| 去掉最小占比 | 一个反光斑、一小撮杂色自己成一类 |
| 取名字改成「永远取第一个」 | 所有颜色都叫 red，而数字全对，报告看着正常 |

⚠️ **第一版的两个变异「改了没反应」，那不是变异没生效，是断言瞄错了机制**
（实测，两条都值得记下来）：

  · **去掉 `s_min` 却没变化** —— 因为直方图的权重**就是**饱和度，S=0 的灰底
    权重恒为 0，本来也进不来。真正的威胁是**低饱和但非零**的表面（白纸 S≈24、
    木桌面），权重小而面积大。所以夹具改用 S=30、色相 110° 的底 ——
    这才打得到 `s_min` 那一行。
  · **去掉 `min_share` 却没变化** —— 那撮噪点先被**噪声底**挡住了
    （`is_peak` 里的 `sm > total/NBINS`），根本走不到 `min_share`。
    所以夹具把杂色从 30 px 加到 150 px：过得了噪声底，仍低于 3% 的占比线。
"""
import importlib.util
import os
import sys
import tempfile

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC = os.path.join(HERE, "palette_survey.py")

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


def load(src_text, tag):
    """把一份源码文本当模块导入（临时文件，避免与真模块的 .pyc 混）。"""
    d = tempfile.mkdtemp(prefix='palette_')
    path = os.path.join(d, 'palette_survey_%s.py' % tag)
    with open(path, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('palette_survey_%s' % tag, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate(old, new):
    """文本替换一次；锚点没命中就抛 —— 防止「变异其实没生效」还当成通过。"""
    s = open(SRC).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


# ----------------------------------------------------------------------
# 合成图：色相已知、饱和度拉满、背景是无饱和的灰
# ----------------------------------------------------------------------
W, H = 240, 120
BLK = 40            # 每块 40×40
HUES = [0.0, 28.0, 53.0, 182.0, 284.0]        # 调色板里五个色的实测色相
TRUTH_NAMES = ['red', 'orange', 'yellow', 'cyan', 'purple']


def synth(hues, bg_sat=0, bg_val=170, bg_hue=0.0,
          blk_sat=220, blk_val=200, blk=BLK):
    """造一张 HSV 图：底 + 若干纯色方块。

    ⚠️ OpenCV 的 H 是 0~179，所以度要**除以 2** 才写进去 —— 而且存进 uint8 时
    会取整，**奇数度会被折到偶数上**（359° 写进去读回来是 360°→折成 0°）。
    造夹具时必须挑**偶数度**，否则你以为在测 359° 而实际测的是 0°。
    ``bg_sat`` 默认 0（纯灰，权重为 0）；要打 ``s_min`` 那一行就得给个
    **低但非零**的饱和度（真实的白纸 ≈24、木桌面 ≈30）。
    """
    import numpy as np
    hsv = np.zeros((H, W, 3), dtype=np.uint8)
    hsv[..., 0] = int(round(bg_hue / 2.0))
    hsv[..., 1] = bg_sat
    hsv[..., 2] = bg_val
    for i, hue in enumerate(hues):
        u0 = 4 + i * (blk + 2)
        if u0 + blk > W:
            break
        hsv[2:2 + blk, u0:u0 + blk, 0] = int(round(hue / 2.0))
        hsv[2:2 + blk, u0:u0 + blk, 1] = blk_sat
        hsv[2:2 + blk, u0:u0 + blk, 2] = blk_val
    return hsv


def with_noise(hues, n=150, hue=100.0):
    """加一小撮杂色像素 —— 用来验「最小占比」那道门。

    ⚠️ 像素数**不能太小**：太小会先被 `is_peak` 里的噪声底挡掉，
    于是 `min_share` 那一行根本没被执行，变异自然「改了没反应」。
    150 px 是实测选出来的：过得了噪声底，仍低于 3% 的占比线。
    """
    import numpy as np
    hsv = synth(hues)
    rows = H - 4
    hsv[rows:rows + 2, 0:n // 2, 0] = int(round(hue / 2.0))
    hsv[rows:rows + 2, 0:n // 2, 1] = 230
    hsv[rows:rows + 2, 0:n // 2, 2] = 210
    return hsv


# 低饱和但非零的底（白纸/木桌面那个量级）—— 用来打 `s_min` 那一行
LOW_SAT_BG = dict(bg_sat=30, bg_val=180, bg_hue=110.0)


def behaviour(m):
    """把模块的关键行为压成一部字典 —— 变异后逐字段比对。"""
    five = synth(HUES)
    merged = synth([48.0, 56.0])            # 相差 8°，必须并成一团
    noisy = with_noise(HUES)
    low_sat = synth(HUES, **LOW_SAT_BG)
    res = m.survey([five, five])            # 两张 → 跨图漂移量得出来
    modes = res['modes']
    red = min(modes, key=lambda x: m._circ_dist(x['hue'], 0.0)) if modes else {}
    return {
        'n_modes': len(modes),
        'names': [x['name'] for x in modes],
        'centers_ok': all(
            min(m._circ_dist(x['hue'], t) for t in HUES) <= 8.0
            for x in modes),
        'red_share': round(red.get('share', 0.0), 3),
        'merged_b': len(m.survey([merged])['modes']),
        'noise_extra_mode': len(m.survey([noisy])['modes']) > len(modes),
        'low_sat_modes': len(m.survey([low_sat])['modes']),
        'drift_available': all(x['drift'] is not None for x in modes),
        'one_image_drift_none': m.survey([five])['modes'][0]['drift'] is None,
        # ⚠️ 让红**真的跨在 0°/360° 两侧**：两张图红分别是 356° 与 4°。
        # 环形漂移 4.0°，线性标准差 176.0°（实测）。
        #
        # 这个夹具试错了两轮，两次都是「断言因为错误的原因通过」：
        #   ① 两张**完全相同**的图 —— 逐图中位数都是 0.0，压根不跨零，
        #      线性标准差也给出 0，变异改坏了照样绿；
        #   ② 359° 与 1° —— `per` 那一步已经把角度折回 [0,360)，
        #      359.0 与 360.0 都变成 0.0，又没跨成。
        # 教训：**要测「跨零」，夹具就得真的落在零点两侧，且两侧都要非零。**
        'wrap_drift_small': (
            min(m.survey([synth([356.0] + HUES[1:]), synth([4.0] + HUES[1:])])
                ['modes'], key=lambda x: m._circ_dist(x['hue'], 0.0))['drift']
            < 10.0),
    }


EXPECT = {
    'n_modes': 5,
    'names': TRUTH_NAMES,
    'centers_ok': True,
    'red_share': 0.2,        # 五块等面积 → 各占 ~0.2
    'merged_b': 1,
    'noise_extra_mode': False,
    'low_sat_modes': 5,      # 低饱和的底不能自己成一类
    'drift_available': True,
    'one_image_drift_none': True,
    'wrap_drift_small': True,   # 红跨 0°/360°（356° vs 4°）⇒ 漂移要算成 ~4°
}


def main():
    try:
        import numpy  # noqa: F401
    except ImportError as e:
        print('需要 numpy：%s（退 2）' % e)
        return 2

    src = open(SRC).read()
    m = load(src, 'real')

    # ------------------------------------------------ [1] 五个已知色团
    print('\n[1] 五个已知色相（红0 橙28 黄53 青182 紫284）')
    got = behaviour(m)
    check('色团数 = 5', got['n_modes'] == EXPECT['n_modes'],
          '实际 %d' % got['n_modes'])
    check('名字按色相顺序对上调色板', got['names'] == TRUTH_NAMES,
          str(got['names']))
    check('每个中心都在真值 ±8° 内', got['centers_ok'])
    check('各占约 1/5（等面积 ⇒ 等权重）',
          abs(got['red_share'] - 0.2) < 0.03, str(got['red_share']))

    # ------------------------------------------------ [2] 合并与占比
    print('\n[2] 合并窗口 / 最小占比 —— 两个「类别虚高」的来源')
    check('相差 8° 的两块并成一团（< merge 25°）', got['merged_b'] == 1,
          '实际 %d 团' % got['merged_b'])
    check('150 个杂色像素（~2%）不自己成一类', not got['noise_extra_mode'])
    check('低饱和的底（S=30，白纸那个量级）不自己成一类',
          got['low_sat_modes'] == 5, '实际 %d 团' % got['low_sat_modes'])

    # ------------------------------------------------ [3] 跨图漂移
    print('\n[3] 展开 vs 漂移 —— 判据本身')
    check('两张图 ⇒ 每个色团都量得出漂移', got['drift_available'])
    check('红跨 0°/360°（356° vs 4°）⇒ 漂移算成 ≈4°，不是 ≈176°',
          got['wrap_drift_small'],
          '红色那个多半是线性标准差算出来的假数（环形才是对的）')
    check('一张图 ⇒ 漂移是 None（量不出来，不是 0）',
          got['one_image_drift_none'])
    res = m.survey([synth(HUES), synth(HUES)])
    txt = "\n".join(m.verdict_lines(res))
    check('同色重复 ⇒ 判成「是一种颜色」', '是一种颜色' in txt,
          [x for x in txt.splitlines() if '色团' in x][:3])
    # 反例：一张图里同一色相被拉成 40° 宽 ⇒ 必须判成「该拆」
    wide = synth([0.0, 12.0, 24.0, 36.0])
    res2 = m.survey([wide, wide])
    t2 = "\n".join(m.verdict_lines(res2))
    check('展开远大于漂移 ⇒ 判成「该拆」', '该拆' in t2,
          [x for x in t2.splitlines() if '色团' in x][:3])

    # ------------------------------------------------ [4] 结论行
    print('\n[4] 结论行：建议类别数 + 可直接粘的 --classes')
    check('给出建议类别数', '建议类别数 = **5**' in txt,
          [x for x in txt.splitlines() if '建议类别数' in x])
    check('给出 --classes 一行',
          '--classes red,orange,yellow,cyan,purple' in txt,
          [x for x in txt.splitlines() if '--classes' in x])
    # 调色板里没有的颜色 → 必须提示加名字，而不是硬塞一个近似的
    # 100° 离最近的 green(125.5°) 差 25.5°，155° 离最近的 cyan(181.9°) 差 26.9°
    # —— 两个都超过 --name-tol 25°，必须提示加名字。
    # ⚠️ 第一版用的是 70°：它离 yellow(52.7°) 只差 17.3°，在容差内，
    #    于是这条断言测的其实是「容差够宽」而不是「提示加名字」。
    near = m.survey([synth([100.0, 155.0])])
    t3 = "\n".join(m.verdict_lines(near))
    check('调色板里没有的颜色 → 提示往 PALETTE 里加',
          '调色板里没有这个颜色' in t3,
          [x for x in t3.splitlines() if '⚠️' in x][:2])

    # ================================================ [5] 变异
    print('\n[5] 变异：改坏一处，上面的断言必须失效')
    mutations = [
        # ⚠️ 断言的是 low_sat_modes 而不是 red_share：权重就是饱和度，
        #    S=0 的纯灰底权重恒 0，去掉下限也进不来（见文件头）。
        ('去掉饱和度下限（低饱和的底会变成一类）',
         'mask = (s >= s_min) & (v >= v_min) & (v <= v_max)',
         'mask = (v >= v_min) & (v <= v_max)',
         'low_sat_modes'),
        ('不合并（光照渐变被数成多类）',
         'if all(_circ_dist(c, kc) >= merge_deg for kc, _ in kept):',
         'if True:',
         'merged_b'),
        ('不砍小占比（噪点自成一类）',
         '        if share < min_share:\n            continue\n',
         '        if False:\n            continue\n',
         'noise_extra_mode'),
        # 这一条钉的是**本轮真逮到的那个 bug**：漂移用了线性标准差，
        # 红色（跨 0°/360°）会算出 168.8°，而「不是 None」的断言照样通过。
        ('漂移用线性标准差（红在 0°/360° 上会炸出假数）',
         '            dev = [((x - m[\'hue\'] + 180.0) % 360.0) - 180.0 for x in per]\n'
         '            m[\'drift\'] = float(np.std(dev))',
         '            m[\'drift\'] = float(np.std(per))',
         'wrap_drift_small'),
        ('取名字永远取第一个（数字全对，名字全错）',
         '    best, best_d = None, 1e9\n'
         '    for n in names:\n'
         '        d = _circ_dist(hue, palette_hue(n))\n'
         '        if d < best_d:\n'
         '            best, best_d = n, d\n'
         '    return best, best_d',
         '    return names[0], 0.0',
         'names'),
    ]
    for label, old, new, field in mutations:
        mm = load(mutate(old, new), 'mut%d' % len(FAILS))
        bad = behaviour(mm)
        check('变异「%s」→ %s 与期望不符' % (label, field),
              bad[field] != EXPECT[field],
              '改坏后仍是 %r —— 说明这条断言根本没盯着那行' % (bad[field],))

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
