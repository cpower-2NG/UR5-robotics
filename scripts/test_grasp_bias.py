#!/usr/bin/env python3
"""b̂（抓放偏置）纯函数的搬家回归：符号 / 上限 / 版本 / 合并 / 轮间落盘。

为什么值得单测：这套东西**补错方向不会报错**——命令 = 期望 + b̂ 时块只是往反方向
多偏 2×|b̂|（≈5.6 mm），计划预检照绿、执行照跑、日志照写「已补偿」。阶段四正式轮
要靠它把 60 mm 格里的系统偏置扣掉，所以符号、fixed 只认种子、版本闸门这三件事
必须在离线就钉死。

另：2026-11-11 把函数从 ``stage3_round.py`` **搬**进本模块（AGENTS §3：搬家 =
移动 + 原位留桩）。这里同时断言 ``stage3_round`` 的同名符号就是本模块的对象
（不是第二份拷贝）——否则两边会各自演化。

跑法：python3 scripts/test_grasp_bias.py
"""
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import grasp_bias as gb                                          # noqa: E402

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


def close(a, b, tol=1e-9):
    return abs(float(a) - float(b)) <= tol


def main():
    # ---------------------------------------------------------------- 符号
    # b̂ = 实测 − 命令。实测比命令偏 (−3, −2) mm ⇒ b̂ = (−3, −2)。
    b, why = gb.bias_from_layer([100.0, 100.0], [103.0, 102.0])
    check('bias_from_layer: 实测−命令', b == [-3.0, -2.0] and why is None,
          repr((b, why)))
    # 命令 = 期望 − b̂ ⇒ 期望 (0,0)、b̂ (−3,−2) ⇒ 命令 (+3, +2)。
    # 这是**正负号唯一写下来的地方**；反了就是「往偏的方向再推一倍」。
    t = gb.target_with_bias([0.0, 0.0], [], 'fixed', seed=[-3.0, -2.0])
    check('命令 = 期望 − b̂（fixed 用种子）', t == [3.0, 2.0], repr(t))
    # 米制包装：期望 400/300 mm、b̂ (−2.791, −0.248) mm。
    m = gb.commanded_target_m([0.400, 0.300], [], 'fixed',
                              seed=[-2.791, -0.248])
    check('commanded_target_m: mm→m 换算只在一处',
          close(m[0], 0.402791) and close(m[1], 0.300248), repr(m))
    # 期望点本身不被改动（列表不许原地改）。
    desired = [0.4, 0.3]
    gb.commanded_target_m(desired, [], 'fixed', seed=[-3.0, -2.0])
    check('期望点不被原地修改', desired == [0.4, 0.3], repr(desired))

    # ---------------------------------------------------------------- 上限 / 缺测
    b, why = gb.bias_from_layer([20.0, 0.0], [0.0, 0.0])
    check('|b̂| > 15 mm ⇒ 不是偏置', b is None and 'bias_too_large' in why,
          repr((b, why)))
    b, why = gb.bias_from_layer([15.0, 0.0], [0.0, 0.0])
    check('|b̂| = 15 mm 仍算偏置（边界是 >）', b == [15.0, 0.0] and why is None,
          repr((b, why)))
    b, why = gb.bias_from_layer(None, [0.0, 0.0])
    check('缺实测 ⇒ measured_missing', b is None and why == 'measured_missing',
          repr((b, why)))
    b, why = gb.bias_from_layer([float('nan'), 0.0], [0.0, 0.0])
    check('非有限 ⇒ bias_nonfinite', b is None and why == 'bias_nonfinite',
          repr((b, why)))

    # ---------------------------------------------------------------- 三种模式
    hist = [[-4.0, -2.0], [-2.0, -2.0]]           # 本轮已测两层
    b = gb.applied_bias(hist, 'fixed', seed=[-2.791, -0.248])
    check('fixed: 只用种子，历史不参与',
          b == [-2.791, -0.248], repr(b))
    b = gb.applied_bias(hist, 'fixed', seed=None)
    check('fixed 无种子 ⇒ [0,0]（「以为在补、其实没补」）', b == [0.0, 0.0],
          repr(b))
    b = gb.applied_bias(hist, 'none', seed=[-2.791, -0.248])
    check('none: 即使给种子也不补', b == [0.0, 0.0], repr(b))
    b = gb.applied_bias(hist, 'adaptive', seed=[-3.0, -1.0])
    check('adaptive: (种子 + 历史) 均值',
          close(b[0], (-3.0 - 4.0 - 2.0) / 3.0) and
          close(b[1], (-1.0 - 2.0 - 2.0) / 3.0), repr(b))
    b = gb.applied_bias([{'layer': 1, 'bias_mm': [-4.0, -2.0]}],
                        'adaptive', seed=None)
    check('adaptive: round.json 的 dict 形状也收',
          b == [-4.0, -2.0], repr(b))
    b = gb.applied_bias([], 'adaptive', seed=None)
    check('adaptive 无种子无历史 ⇒ [0,0]', b == [0.0, 0.0], repr(b))
    # 日志口径：fixed 不许说成「补偿关闭」。
    notes = {m: gb.bias_note(m) for m in ('adaptive', 'fixed', 'none')}
    check('bias_note: 三模式各自说得清',
          notes['fixed'] != notes['none'] and '固定' in notes['fixed'],
          repr(notes))

    # ---------------------------------------------------------------- 种子解析
    check('parse_bias_seed: 半角逗号',
          gb.parse_bias_seed('-2.791,-0.248') == [-2.791, -0.248])
    check('parse_bias_seed: 中文逗号',
          gb.parse_bias_seed('-2.791，-0.248') == [-2.791, -0.248])
    check('parse_bias_seed: 空 ⇒ None',
          gb.parse_bias_seed('') is None and gb.parse_bias_seed(None) is None)
    try:
        gb.parse_bias_seed('1,2,3')
        ok = False
    except ValueError:
        ok = True
    check('parse_bias_seed: 三个数要报错（不许猜）', ok)

    # ---------------------------------------------------------------- 合并 / 落盘
    prev = {'version': gb.BIAS_ESTIMATE_VERSION, 'n': 2,
            'bias_mm': [-4.0, -2.0], 'rounds': [{'round': 1}]}
    merged = gb.merge_bias_estimate(prev, [[-2.0, -2.0], [-3.0, -2.0]])
    check('merge: n 加权（2×旧 + 2×新）',
          merged['n'] == 4 and close(merged['bias_mm'][0], -3.25) and
          close(merged['bias_mm'][1], -2.0), repr(merged))
    check('merge: 版本与模型随行',
          merged['version'] == gb.BIAS_ESTIMATE_VERSION and
          merged['model'] == gb.BIAS_ESTIMATE_MODEL, repr(merged))
    check('merge: 空输入 ⇒ None', gb.merge_bias_estimate(None, []) is None)

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, gb.BIAS_ESTIMATE_FILE)
        # 没有历史 ⇒ 不写（离线预检不许伪造一个估计）。
        check('write: 无测量不落盘',
              gb.write_bias_estimate(td, {'bias_history': []}) is None and
              not os.path.exists(path))
        doc = {'round': 7, 'bias_history': [{'bias_mm': [-3.0, -1.0]},
                                           {'bias_mm': [-1.0, -1.0]}]}
        out = gb.write_bias_estimate(td, doc)
        check('write: 落盘 n=2 均值 (−2,−1)',
              out and out['n'] == 2 and out['bias_mm'] == [-2.0, -1.0],
              repr(out))
        doc2 = {'round': 8, 'bias_history': [{'bias_mm': [-4.0, -3.0]}]}
        out2 = gb.write_bias_estimate(td, doc2)
        check('write: 第二轮并入（n=3）',
              out2 and out2['n'] == 3 and len(out2['rounds']) == 2,
              repr(out2))
        check('load: 读回同一份',
              gb.load_bias_estimate(path) == out2['bias_mm'],
              repr(gb.load_bias_estimate(path)))
        # 判分是只读阶段、可以重跑：同一个 round 再写一次必须**覆盖**而不是追加，
        # 否则 n 会翻倍、同一批样本被重复计权（2026-11-12 实测 n=15 凭空多两轮）。
        out3 = gb.write_bias_estimate(td, doc2)
        check('write: 同一 round 重跑不重复计权（n 仍 3、轮记录仍 2）',
              out3 and out3['n'] == 3 and len(out3['rounds']) == 2,
              repr(out3))
        doc2b = {'round': 8, 'bias_history': [{'bias_mm': [0.0, 2.0]}]}
        out4 = gb.write_bias_estimate(td, doc2b)
        check('write: 同一 round 改样本 ⇒ 以最后一次为准（均值跟着变）',
              out4 and out4['n'] == 3 and out4['bias_mm'] == [-1.333, 0.0],
              repr(out4))
        # 版本闸门：v2（矫正后含拖拽）样本口径不同，必须当没有。
        bad = dict(out2)
        bad['version'] = 2
        json.dump(bad, open(path, 'w'))
        check('load: 版本不符 ⇒ None（样本不能相加）',
              gb.load_bias_estimate(path) is None)
        open(path, 'w').write('{not json')
        check('load: 坏文件 ⇒ None（不炸整轮）',
              gb.load_bias_estimate(path) is None)
        check('load: 文件不存在 ⇒ None',
              gb.load_bias_estimate(os.path.join(td, 'nope.json')) is None)

    # ---------------------------------------------------------------- 搬家留桩
    sys.path.insert(0, HERE)
    import stage3_round as s3r                                       # noqa: E402
    same = all(getattr(s3r, n) is getattr(gb, n) for n in (
        'bias_from_layer', 'bias_note', 'applied_bias', 'target_with_bias',
        'commanded_target_m', 'parse_bias_seed', 'load_bias_estimate',
        'write_bias_estimate', 'merge_bias_estimate'))
    check('stage3_round 的同名符号 = 本模块对象（搬家不是复制）', same)
    check('常量同源',
          s3r.BIAS_LIMIT_MM is gb.BIAS_LIMIT_MM and
          s3r.BIAS_ESTIMATE_FILE == gb.BIAS_ESTIMATE_FILE and
          s3r.BIAS_ESTIMATE_VERSION == gb.BIAS_ESTIMATE_VERSION)

    # ---------------------------------------------------------------- 变异
    # 把符号反过来（− b̂ → + b̂），上面那条「命令 = 期望 − b̂」必须变红。
    src = open(os.path.join(HERE, 'grasp_bias.py'), encoding='utf-8').read()
    flipped = src.replace("return [float(desired_mm[0]) - b[0], "
                          "float(desired_mm[1]) - b[1]]",
                          "return [float(desired_mm[0]) + b[0], "
                          "float(desired_mm[1]) + b[1]]")
    check('变异前置：真的改到了那一行', flipped != src)
    ns = {'__name__': 'grasp_bias_mutant'}
    exec(compile(flipped, '<mutant>', 'exec'), ns)
    t_mut = ns['target_with_bias']([0.0, 0.0], [], 'fixed', seed=[-3.0, -2.0])
    check('变异：符号翻反 ⇒ 与判据不符（测试会红）', t_mut != [3.0, 2.0],
          repr(t_mut))

    print()
    if N_FAIL:
        print('%d/%d FAIL（%s）' % (N_FAIL, N_PASS + N_FAIL, ', '.join(FAILS)))
        return 1
    print('%d/%d PASS' % (N_PASS, N_PASS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
