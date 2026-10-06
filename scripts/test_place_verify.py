#!/usr/bin/env python3
"""放置落点复核判据的自检。**不需要 ROS、不需要 MoveIt、不需要 colcon build**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么这段判据值得单独钉
========================
复核判决错了**几乎不会响**：它只往日志里写一行。现场真正会看的是收工块那行
汇总（「通过 6 失败 2 …」），而汇总里最容易出事的错法有两种，两种都不报错：

  · 把 ``UNAVAILABLE``（相机没取景到，判不了）与 ``NOT_FOUND``（放了，但飞了）
    **合并计数** —— 取景一旦不对，8 格全报「失败」，于是「8 格全放错」这个
    结论会把取景问题伪装成放置问题。这是本模块存在的全部理由。
  · 把 ``z`` 当硬门 —— 斜放、叠放、深度在物体边缘取到台面都会让 z 失准，
    而它们恰恰是本阶段要测的现象；拿 z 判失败等于把要测的量在测量环节丢掉。

断言是不是真的在盯着那几行代码，只能靠**末尾四段变异**证明：把判据各改坏
一处，前面相应的断言必须失效。

⚠️ 与 ``test_planner_place_verify.py`` 的分工：那个测**接线**（判据有没有真的
接进 planner_node、帧戳门有没有生效），这个测**判据本身**。两者缺一不可。
"""
import importlib.util
import os
import sys
import tempfile

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC = os.path.join(REPO,
                   "ros2_ws/src/grasp_planner/grasp_planner/place_verify.py")

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
    """把一份源码文本当模块导入（用临时文件，避免与真模块的 .pyc 混）。"""
    d = tempfile.mkdtemp(prefix='placeverify_')
    path = os.path.join(d, 'place_verify_%s.py' % tag)
    with open(path, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('place_verify_%s' % tag, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate(old, new):
    """对真源码做一次文本替换；替换没命中就抛（防止「变异其实没生效」）。"""
    s = open(SRC).read()
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


# ----------------------------------------------------------------------
# 固定输入：真机 A2 的 8 个槽位与它们的类别（与 docs/现场yaml草稿-真机.yaml 同）
# ----------------------------------------------------------------------
SLOTS = [(0.50, -0.10), (0.60, -0.10), (0.50, -0.20), (0.60, -0.20),
         (0.50, -0.30), (0.60, -0.30), (0.50, -0.40), (0.60, -0.40)]
CLASSES = ['red', 'red', 'orange', 'orange', 'yellow', 'yellow', 'cyan', 'cyan']
PLACE_Z = 0.0285
#: 判据参数与 planner_node 的默认值**同值**（改了默认值这里要跟着改 ——
#: 否则测的是一套参数、跑的是另一套）
TOL = 0.025
MIN_SCORE = 0.30
Z_TOL = 0.03
MARGIN = 0.05


def obs(cls, x, y, score=0.9, z=PLACE_Z):
    return (cls, score, x, y, z)


def judge(m, observations, index=0, expected_class='__slot__'):
    """按第 ``index`` 个槽位判一次。``expected_class=None`` = 不按类别分。"""
    cls = CLASSES[index] if expected_class == '__slot__' else expected_class
    return m.judge_place(
        [m.Observation(*o) for o in observations],
        SLOTS[index], cls, TOL, MIN_SCORE, Z_TOL,
        roi=m.place_roi(SLOTS, MARGIN), expected_z=PLACE_Z)


# ----------------------------------------------------------------------
# [1] 四种判决各自要在该出现的时候出现
# ----------------------------------------------------------------------
def part1_verdicts(m):
    print('\n[1] 四态判决：该出的出、不该出的不出')

    r = judge(m, [obs('red', 0.500, -0.100)])
    check('A1 正中 → OK', r.verdict == m.OK, r.verdict)
    check('  Δ 记的是实测 − 期望', r.detail.find('Δ=(+0.0,+0.0)') >= 0,
          r.detail)

    # 25 mm 是半径：24 mm 进、26 mm 出。边界钉住，免得日后有人把
    # `<=` 改成 `<` 而没人发现（那会让「刚好在容差上」的一批静默变成失败）。
    r = judge(m, [obs('red', 0.500 + 0.024, -0.100)])
    check('A2 距 24 mm（容差内）→ OK', r.verdict == m.OK, r.verdict)
    r = judge(m, [obs('red', 0.500 + 0.026, -0.100)])
    check('A2b 距 26 mm（容差外）→ NOT_FOUND',
          r.verdict == m.NOT_FOUND, r.verdict)

    r = judge(m, [obs('yellow', 0.498, -0.102, 0.88)])
    check('A3 槽位附近是别的类别 → WRONG_CLASS',
          r.verdict == m.WRONG_CLASS, r.verdict)
    check('  报出看到的类别与分数，不只报结论',
          'yellow' in r.detail and '0.88' in r.detail, r.detail)

    # 这一条是**唯一**能把「判不了」与「放失败了」分开的断言
    r = judge(m, [obs('red', 0.500, 0.350)])       # 在抓取区，远在 ROI 之外
    check('A4 整批都在放置区 ROI 之外 → UNAVAILABLE（不是失败）',
          r.verdict == m.UNAVAILABLE, r.verdict)
    check('  UNAVAILABLE **不**算进 summarize 的「失败」里',
          '失败 0' in m.summarize([r.verdict]), m.summarize([r.verdict]))

    r = judge(m, [])
    check('A5 空数组 → UNAVAILABLE（上游投影器不该发空数组）',
          r.verdict == m.UNAVAILABLE, r.verdict)
    check('  文案指向「上游行为变了」而不是「没找到」',
          '上游' in r.detail, r.detail)

    r = judge(m, [obs('red', 0.500, -0.100, 0.12)])
    check('A6 位置对但分数 < min → NOT_FOUND（不是 OK）',
          r.verdict == m.NOT_FOUND, r.verdict)
    check('  分数门与抓取段同值', MIN_SCORE == 0.30, MIN_SCORE)

    # 距离门先于分数门：低分且在容差外时，报的是距离，不是分数
    r = judge(m, [obs('red', 0.500 + 0.030, -0.100, 0.12)])
    check('A6b 低分 + 容差外 → 仍报 NOT_FOUND，且文案讲距离',
          r.verdict == m.NOT_FOUND and 'mm 内没有任何检测' in r.detail,
          r.detail)


# ----------------------------------------------------------------------
# [2] 不按类别分的那条路（A1 单类别行为，必须逐字节不变）
# ----------------------------------------------------------------------
def part2_unrouted(m):
    print('\n[2] 不按类别分（通配槽位 / A1 行为）')

    r = judge(m, [obs('whatever', 0.498, -0.103, 0.55)], expected_class=None)
    check('B1 期望类别为空 → 只判位置与分数，照过', r.verdict == m.OK,
          r.verdict)
    r = judge(m, [obs('whatever', 0.498, -0.103, 0.20)], expected_class=None)
    check('B2 但分数不够还是不过', r.verdict == m.NOT_FOUND, r.verdict)

    # 无槽位 = 不筛 ROI（place_roi 返回 None），此时「区外」这个概念不存在
    r = m.judge_place([m.Observation('red', 0.9, 0.50, -0.10, PLACE_Z)],
                      (0.50, -0.10), None, TOL, MIN_SCORE, Z_TOL,
                      roi=m.place_roi([], MARGIN), expected_z=PLACE_Z)
    check('B3 无槽位 → roi=None = 不筛，不因「区外」误报不可用',
          r.verdict == m.OK, r.verdict)
    check('  place_roi([]) 确实是 None', m.place_roi([], MARGIN) is None)


# ----------------------------------------------------------------------
# [3] z 只告警 —— 这是本阶段最容易做错的一条
# ----------------------------------------------------------------------
def part3_z_warn_only(m):
    print('\n[3] z 只告警、不判失败（方块斜着落 / 叠着落都要能测出来）')

    r = judge(m, [obs('red', 0.500, -0.100, 0.9, z=PLACE_Z + 0.060)])
    check('C1 z 偏离 60 mm（> 30 mm 阈值）**仍然判通过**',
          r.verdict == m.OK, r.verdict)
    check('  但带上了告警后缀', r.z_warn is not None, r.z_warn)
    check('  告警里写了偏离量', '+60 mm' in (r.z_warn or ''), r.z_warn)

    r = judge(m, [obs('red', 0.500, -0.100, 0.9, z=PLACE_Z + 0.060)])
    line = m.format_slot_line(0, SLOTS[0], CLASSES[0], r.verdict, r.obs,
                              r.detail, r.z_warn)
    check('C2 台账行里既有「通过」也有 z 告警',
          '通过' in line and '⚠️' in line, line)

    r = judge(m, [obs('red', 0.500, -0.100, 0.9, z=PLACE_Z + 0.010)])
    check('C3 z 只偏 10 mm → 不告警（阈值不虚报）', r.z_warn is None, r.z_warn)

    # expected_z 没给 = 不做 z 判断，也**不该**报出 None 相关的噪声
    r = m.judge_place([m.Observation('red', 0.9, 0.50, -0.10, 9.9)],
                      SLOTS[0], 'red', TOL, MIN_SCORE, Z_TOL,
                      roi=m.place_roi(SLOTS, MARGIN), expected_z=None)
    check('C4 不给 expected_z 就不判 z（也不崩）',
          r.verdict == m.OK and r.z_warn is None, r.verdict)


# ----------------------------------------------------------------------
# [4] 台账与汇总的格式（现场靠 grep 日志验收，格式就是接口）
# ----------------------------------------------------------------------
def part4_format(m):
    print('\n[4] 台账格式：现场靠 grep 日志验收，格式就是接口')

    r = judge(m, [obs('red', 0.498, -0.103)])
    line = m.format_slot_line(0, SLOTS[0], CLASSES[0], r.verdict, r.obs,
                              r.detail, r.z_warn)
    want = ('    槽位[0] red      期望(0.500,-0.100)  实测(0.498,-0.103)  '
            'Δ=(-2.0,-3.0) mm  |Δ|=3.6  通过')
    check('D1 通过行的格式与计划书逐字一致', line == want,
          '\n    实际：%s\n    期望：%s' % (line, want))

    line = m.format_slot_line(3, SLOTS[3], CLASSES[3], m.NOT_FOUND,
                              None, '25 mm 内没有 orange')
    check('D2 失败行带槽位/类别/期望坐标与原因',
          line.strip().startswith('槽位[3] orange')
          and '期望(0.600,-0.200)' in line
          and '失败' in line and '25 mm 内没有 orange' in line, line)

    line = m.format_slot_line(7, SLOTS[7], CLASSES[7], m.TIMEOUT)
    check('D3 超时行用的是「超时未复核」而不是「失败」',
          '超时未复核' in line and '失败' not in line, line)
    line = m.format_slot_line(7, SLOTS[7], CLASSES[7], m.SKIPPED)
    check('D4 未复核行不冒充通过', '未复核' in line and '通过' not in line, line)

    line = m.format_slot_line(0, SLOTS[0], None, m.OK,
                              m.Observation('x', 0.9, 0.50, -0.10, PLACE_Z))
    check('D5 不按类别分时类别列写「通配」', '通配' in line, line)

    # 五种计数一个都不能省 —— 少了哪一种，现场就会把那种读成另一种
    s = m.summarize([m.OK, m.OK, m.WRONG_CLASS, m.NOT_FOUND, m.UNAVAILABLE,
                     m.TIMEOUT, m.SKIPPED])
    want = ('  落点复核汇总：通过 2  失败 2  不可用 1  超时未复核 1  未复核 1')
    check('D6 汇总行五类各归各位', s == want, '\n    实际：%s' % s)
    check('D7 全通过时五个数都在（不会退化成只打「通过 8」）',
          m.summarize([m.OK] * 8).count('0') == 4, m.summarize([m.OK] * 8))


# ----------------------------------------------------------------------
# [5] 变异：把判据各改坏一处，上面相应的断言必须失效
# ----------------------------------------------------------------------
def part5_mutation():
    print('\n[5] 变异：改坏判据后，对应断言必须变红')

    # M1：ROI 过滤失效 —— 「相机没取景到」会被当成「放失败了」。
    #     这正是本模块要防的头号错误，所以它必须真的在拦东西。
    m = load(mutate('def in_roi(roi, x, y):\n    """点是否在放置区 ROI 内（闭区间）。'
                    '``roi is None`` = 不筛。"""\n    if roi is None:\n'
                    '        return True',
                    'def in_roi(roi, x, y):\n    """变异：永远在区内"""\n'
                    '    if True:\n        return True'),
             'm1')
    r = judge(m, [obs('red', 0.500, 0.350)])
    check('M1 去掉 ROI 过滤 → 「整批在区外」不再判 UNAVAILABLE',
          r.verdict != m.UNAVAILABLE, r.verdict)

    # M2：不看类别 —— 放错格（红块进黄格）会静默判通过。这是分拣里
    #     唯一一处错了不报错的地方，必须被钉住。
    #     ⚠️ 锚点是 4 空格缩进（judge_place 函数体第一层），不是 8 ——
    #        缩进写错时 mutate() 会抛「锚点没找到」，不会静默空转。
    m = load(mutate('    if expected_class:', '    if False:'), 'm2')
    r = judge(m, [obs('yellow', 0.498, -0.102, 0.88)])
    check('M2 不看类别 → 别的类别也判 OK（所以 A3 不是空转）',
          r.verdict == m.OK, r.verdict)

    # M3：把 z 做成硬门 —— 斜放/叠放会被判失败，要测的量在测量环节被丢掉。
    #     ⚠️ 锚点必须在**判决**那一行（`return Result(OK, ...)`），不能在
    #        「何时算 z_warn」那一行 —— 后者只影响告警文案，判决压根走不到，
    #        变异就变成空转（第一版正是这么写错的：变异生效了却照样 OK，
    #        看上去像「C1 是空转」，其实是变异挑错了杠杆）。
    m = load(mutate(
        '    return Result(OK, o,',
        '    if (expected_z is not None\n'
        '            and abs(o.z - float(expected_z)) > z_tol):\n'
        '        return Result(NOT_FOUND, o, "变异：z 超阈值就当没找到", None)\n'
        '    return Result(OK, o,'), 'm3')
    r = judge(m, [obs('red', 0.500, -0.100, 0.9, z=PLACE_Z + 0.060)])
    check('M3 z 变硬门 → z 偏离时不再判通过（所以 C1 不是空转）',
          r.verdict != m.OK, r.verdict)

    # M4：空数组判成「没找到」—— 上游若改了「不发空数组」的行为，
    #     现场会看到「8 格全失败」，而不是「判不了」。
    m = load(mutate('    if not obs:\n        # 投影器从不发空数组',
                    '    if not obs:\n        return Result(NOT_FOUND, None,'
                    ' "变异：空批当没找到", None)\n        # 投影器从不发空数组'),
             'm4')
    r = judge(m, [])
    check('M4 空批当「没找到」→ A5 的断言确实盯着那一行',
          r.verdict == m.NOT_FOUND, r.verdict)

    # M5：分数门失效 —— 低分检测会被当成真方块（复核会给出假「通过」）
    m = load(mutate('o.score >= min_score', 'True'), 'm5')
    r = judge(m, [obs('red', 0.500, -0.100, 0.12)])
    check('M5 去掉分数门 → 低分也判通过（所以 A6 不是空转）',
          r.verdict == m.OK, r.verdict)


def main():
    src = open(SRC).read()
    real = load(src, 'real')

    print('=' * 72)
    print('放置落点复核判据自检   %s' % os.path.relpath(SRC, REPO))
    print('=' * 72)

    part1_verdicts(real)
    part2_unrouted(real)
    part3_z_warn_only(real)
    part4_format(real)
    part5_mutation()

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
