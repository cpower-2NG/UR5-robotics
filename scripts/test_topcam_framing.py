#!/usr/bin/env python3
"""俯视相机「取景收敛器」的自检。**不需要 ROS、不需要相机、不需要 colcon build**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
这个工具是**给现场拿主意用的**：它说「升高 22 cm」，人就真去升高 22 cm，
一次迭代十几分钟。所以它错的方向比错本身更要紧 ——

  · 少报一项出框（比如只看中心不看方块顶边）⇒ 人挪完一次，回来发现还差一点，
    于是**再挪一次**。这正是它要消掉的循环，等于白做。
  · 把「横向出框」也算进「还差几 px 的 v」⇒ 人照着升高，横向问题一点没动。
  · 灵敏度用**全场平均**去换算最急那一项 ⇒ 系统性偏小，同样要多挪一次。
  · 「自证」那一步把升高量丢掉（`apply_move` 的高度写在**返回的 t** 里）⇒
    复核输出与挪动前一模一样，看起来像「外推不准」。这个 bug 真发生过。

末尾五段变异把上述每一处各改坏一次，对应断言必须失效。

⚠️ 涉及的外参数字（8 槽在上边界外 159 px、升高 +34 cm 后进框、7.37 px/cm）
都是**这台机器这台相机**在当前机位下的实测值。换机位后外参一变，本测试里
带绝对数字的那几条要跟着改 —— 它们钉的是「几何算对了」，不是「永远是这些数」。
"""
import importlib.util
import os
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True          # 防 .pyc 假绿（见仓库约定）

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SCRIPT = os.path.join(HERE, "topcam_framing.py")
sys.path.insert(0, HERE)                # 让变异副本也能 import draw_zone_on_frame

EXTR = {
    "10-03": os.path.join(REPO, "docs/calib/extrinsics-2026-10-03-base_link.json"),
    "10-02": os.path.join(REPO, "docs/calib/extrinsics-2026-10-02.json"),
}

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
    d = tempfile.mkdtemp(prefix='topcamframe_')
    path = os.path.join(d, 'topcam_framing_%s.py' % tag)
    with open(path, 'w') as f:
        f.write(src_text)
    spec = importlib.util.spec_from_file_location('topcam_framing_%s' % tag, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mutate(old, new, src=None):
    """对真源码做一次文本替换；替换没命中就抛（防止「变异其实没生效」）。"""
    s = open(SCRIPT).read() if src is None else src
    assert old in s, '变异锚点没找到：%r' % old
    return s.replace(old, new)


def run(*argv):
    """跑一次 CLI，返回 ``(rc, stdout+stderr)``。"""
    p = subprocess.run([sys.executable, SCRIPT] + list(argv),
                       capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


REAL = load(open(SCRIPT).read(), 'real')
import numpy as np                      # noqa: E402


def mat_to_quat(R):
    """3×3 → ``[x, y, z, w]``（迹法）。测试里自己带一份，是为了不依赖
    ``topcam_marker_calib``（它 import cv2/aruco，本机不一定装得上）。
    ⚠️ 它只用来**构造** `--camera` 的输入：算错了这条断言会直接变红
    （两条路的投影对不上），不会静默放过。"""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s,
             (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s,
             (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
             0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def same_rows(a, b):
    """两份 CLI 输出里的**投影表**逐行相同（表行以「槽位」「抓取区」开头）。"""
    def rows(s):
        return [l for l in s.splitlines()
                if l.startswith('槽位') or l.startswith('抓取区')]
    ra, rb = rows(a), rows(b)
    return bool(ra) and ra == rb


# ----------------------------------------------------------------------
# [1] 几何：apply_move / 俯仰轴
# ----------------------------------------------------------------------
def part1_geometry():
    print('\n[1] 挪动的几何含义（升高 / 平移 / 俯仰）')
    R0, t0 = REAL.dzf.compose_base_optical(
        __import__('json').load(open(EXTR['10-03'], encoding='utf-8')))

    R1, t1 = REAL.apply_move(R0, t0, raise_cm=10.0)
    check('1.1 升高 10 cm 只动 t[2]（+0.10 m）',
          abs(t1[2] - t0[2] - 0.10) < 1e-9 and abs(t1[0] - t0[0]) < 1e-12
          and abs(t1[1] - t0[1]) < 1e-12, str(t1 - t0))
    check('1.2 升高不改朝向（R 逐位相同）', np.allclose(R1, R0))
    check('1.3 入参 t 不被就地修改（返回的是拷贝）',
          abs(t0[2] - t1[2] + 0.10) < 1e-9)

    _, t2 = REAL.apply_move(R0, t0, move_x_cm=3.0, move_y_cm=-2.0)
    check('1.4 平移 x/y 各就各位且带符号',
          abs(t2[0] - t0[0] - 0.03) < 1e-9 and abs(t2[1] - t0[1] + 0.02) < 1e-9,
          str(t2 - t0))

    # 俯仰轴：水平、垂直于光轴的水平投影 —— 就是三脚架云台那根轴
    ax = REAL._pitch_axis(R0)
    check('1.5 俯仰轴是水平的（z=0）', abs(ax[2]) < 1e-12, str(ax))
    check('1.6 俯仰轴是单位向量', abs(np.linalg.norm(ax) - 1.0) < 1e-9)
    horiz = R0[:, 2].copy()
    horiz[2] = 0.0
    horiz /= np.linalg.norm(horiz)
    check('1.7 俯仰轴垂直于光轴的水平投影',
          abs(float(np.dot(ax, horiz))) < 1e-9,
          '点积 %.2e' % float(np.dot(ax, horiz)))

    R3, _ = REAL.apply_move(R0, t0, tilt_deg=5.0)
    check('1.8 俯仰后 R 仍是正交阵（det=+1）',
          abs(np.linalg.det(R3) - 1.0) < 1e-9
          and np.allclose(R3 @ R3.T, np.eye(3), atol=1e-9))
    h3 = R3[:, 2].copy()
    h3[2] = 0.0
    h3 /= np.linalg.norm(h3)
    check('1.9 俯仰不改变水平朝向（只抬头/低头）',
          float(np.dot(h3, horiz)) > 0.9999,
          '方向余弦 %.6f' % float(np.dot(h3, horiz)))

    # 绕 z 转 90°：x → y
    Rz = REAL._rot_axis(np.array([0.0, 0.0, 1.0]), np.pi / 2)
    check('1.10 罗德里格斯绕 z 转 90° 把 x 映到 y',
          np.allclose(Rz @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-9),
          str(Rz @ np.array([1.0, 0, 0])))

    # 光轴竖直时没有俯仰轴（返回 None，不是崩）
    check('1.11 光轴竖直（第三列 = z）时俯仰轴为 None（调用方要能容忍）',
          REAL._pitch_axis(np.eye(3)) is None)


# ----------------------------------------------------------------------
# [2] 判据 check_one：余量是硬判据，且「还差多少」有符号
# ----------------------------------------------------------------------
def part2_check_one():
    print('\n[2] 判据 check_one：余量是硬判据，且「还差多少」带符号')
    W, H, m = 1280, 720, 8.0

    ok = (True, None, '在框内')
    r = REAL.check_one(('x', 100.0, 200.0, 100.0, 200.0, False), W, H, m)
    check('2.1 完全在框内 → 合格', r == ok, str(r))

    # 余量是**硬**判据：贴着上边界（v=0）不算合格，虽然它确实「在画面里」
    r = REAL.check_one(('x', 100.0, 200.0, 0.0, 200.0, False), W, H, m)
    check('2.2 贴着上边界（v=0）→ 不合格（余量是硬判据，不是「看得见就行」）',
          r[0] is False and '上边余量只剩 0 px' in r[2], str(r))
    check('2.3 且「还差多少」= +8 px（正数 = 往画面下方挪）', r[1] == 8.0,
          str(r[1]))
    r = REAL.check_one(('x', 100.0, 200.0, 5.0, 200.0, False), W, H, m)
    check('2.4 只差 3 px 余量 → 仍判不合格（不给「差不多」留口子）',
          r[0] is False and r[1] == 3.0, str(r))

    r = REAL.check_one(('x', 100.0, 200.0, -35.0, 200.0, False), W, H, m)
    check('2.5 上边出框 35 px → +43 px（出框量 + 余量）', r[1] == 43.0, str(r))
    check('2.6 说明里写的是「上边出框 35 px」（告诉人症状，不只给数字）',
          '上边出框 35 px' in r[2], r[2])
    check('2.6b 「真出框」与「余量不足」是两种文案（前者方块已被切掉）',
          '上边余量只剩' in REAL.check_one(
              ('x', 100.0, 200.0, 5.0, 200.0, False), W, H, m)[2], '')
    check('2.6c 出框时也不把余量算进「出框」那个数字里',
          '另需 8 px 余量' in r[2], r[2])

    r = REAL.check_one(('x', 100.0, 200.0, 100.0, 740.0, False), W, H, m)
    check('2.7 下边出框 → **负数**（要往画面上方挪）', r[1] < 0, str(r[1]))

    r = REAL.check_one(('x', 100.0, 200.0, -50.0, 760.0, False), W, H, m)
    check('2.8 上下都出框 → 取绝对值大的那一边（更急的那边）', r[1] > 0,
          'need_v=%r（上边差 58，下边差 49，该给 +58）' % r[1])

    # 横向出框**不能**给出 v 方向的建议 —— 否则人会照着升高，横向一点没动
    r = REAL.check_one(('x', -30.0, 200.0, 100.0, 200.0, False), W, H, m)
    check('2.9 左边出框 → 不合格', r[0] is False and '左边出框' in r[2], str(r))
    check('2.10 但**不给 v 方向的建议**（升高救不了横向）', r[1] is None,
          '给了 need_v=%r —— 现场会照着升高，横向问题一点没动' % (r[1],))
    r = REAL.check_one(('x', 100.0, 1300.0, 100.0, 200.0, False), W, H, m)
    check('2.11 右边出框同理，need_v 也为 None',
          r[0] is False and r[1] is None and '右边出框' in r[2], str(r))

    r = REAL.check_one(('x', 100.0, 200.0, 100.0, 200.0, True), W, H, m)
    check('2.12 有角点在相机背后 → 不合格，且**不猜数字**',
          r[0] is False and r[1] is None and '背后' in r[2], str(r))
    r = REAL.check_one(('x', None, None, None, None, True), W, H, m)
    check('2.13 整个在后面 → 指向「外参或站点配置错了」',
          r[0] is False and '外参' in r[2], r[2])


# ----------------------------------------------------------------------
# [3] 灵敏度：数值微分 + 「用项自己的灵敏度」
# ----------------------------------------------------------------------
def part3_sensitivity():
    print('\n[3] 灵敏度（数值微分，换机位后自动跟着变）')
    import json
    doc = json.load(open(EXTR['10-03'], encoding='utf-8'))
    R, t = REAL.dzf.compose_base_optical(doc)
    K, W, H = REAL.REAL_K, REAL.REAL_W, REAL.REAL_H
    items = REAL.framing_items(REAL.REAL_SLOTS, REAL.REAL_PICK,
                               REAL.REAL_BLOCK_W, REAL.REAL_BLOCK_H)
    slot_probes = [p for name, pts in items if name.startswith('槽位')
                   for p in pts]
    pick_probes = [p for name, pts in items if not name.startswith('槽位')
                   for p in pts]

    s_raise = REAL.sensitivity(R, t, K, slot_probes, 'raise')
    check('3.1 升高 1 cm ≈ +7.4 px（实测 7.37，与旧机位实测的 7.2 同量级）',
          7.0 < s_raise < 7.8, '%.2f' % s_raise)
    check('3.2 升高把画面内容往**下**推（正号）—— 与「还差 +N px」同号',
          s_raise > 0)
    s_mx = REAL.sensitivity(R, t, K, slot_probes, 'move_x')
    check('3.3 这台机位下相机沿 x 平移几乎不动画面（|s| < 0.5 px/cm）',
          abs(s_mx) < 0.5, '%.3f' % s_mx)

    # 相机沿 +y 与槽位沿 +y 是**相反**的图像效果、量级接近 ——
    # 这一条把两路换算互相钉住（任一路写错都会破）
    s_my = REAL.sensitivity(R, t, K, slot_probes, 'move_y')
    s_sloty = REAL.sensitivity_slots_y(R, t, K, slot_probes)
    check('3.4 相机 +y 与槽位 +y 的图像效果**反向**（这是几何必然而非巧合）',
          s_my * s_sloty < 0, '相机 %.2f / 槽位 %.2f' % (s_my, s_sloty))
    check('3.5 两者量级接近（互差 < 20%）',
          abs(abs(s_my) - abs(s_sloty)) / abs(s_my) < 0.20,
          '相机 %.2f / 槽位 %.2f' % (s_my, s_sloty))

    # 抓取区与槽位在画面不同位置 ⇒ 灵敏度不同。这正是「不能用全场平均」的理由
    s_pick = REAL.sensitivity(R, t, K, pick_probes, 'raise')
    check('3.6 抓取区与槽位的升高灵敏度确实不同（所以必须用项自己的）',
          abs(s_pick - s_raise) > 0.05,
          '抓取区 %.2f vs 槽位 %.2f' % (s_pick, s_raise))

    # item_sensitivity 必须给出**那一项自己**的值，而不是全局平均
    worst = None
    for name, pts in items:
        if not name.startswith('槽位'):
            continue
        s = REAL.item_sensitivity(R, t, K, pts, 'raise')
        if worst is None or s < worst[1]:
            worst = (name, s)
    check('3.7 最远那一项的灵敏度**小于**全场平均（斜视下远处小）',
          worst[1] < s_raise, '%s %.2f vs 平均 %.2f' % (worst[0], worst[1],
                                                        s_raise))

    # 数值微分的一致性：把步长减半，结果应当基本不变（差分没写错）
    s_half = REAL.sensitivity(R, t, K, slot_probes, 'raise', step=0.5)
    check('3.8 步长减半后**速率**不变（返回值是 px/单位，不是 px/该步长）',
          abs(s_half - s_raise) / s_raise < 0.02,
          '%.3f vs %.3f —— 不除 step 的话这里会差一倍' % (s_half, s_raise))
    s_double = REAL.sensitivity(R, t, K, slot_probes, 'raise', step=2.0)
    check('3.9 步长加倍也不变（同一条契约）',
          abs(s_double - s_raise) / s_raise < 0.02,
          '%.3f vs %.3f' % (s_double, s_raise))
    s_sloty_half = REAL.sensitivity_slots_y(R, t, K, slot_probes, step_cm=0.5)
    check('3.10 挪槽位那一路同样按 px/cm 报',
          abs(s_sloty_half - s_sloty) / abs(s_sloty) < 0.02,
          '%.3f vs %.3f' % (s_sloty_half, s_sloty))


# ----------------------------------------------------------------------
# [4] CLI 端到端：退出码与那条「放置区必须可见」的不变量
# ----------------------------------------------------------------------
def part4_cli():
    print('\n[4] CLI 端到端（退出码 0/1/2 与「抓取区必须可见」这条不变量）')

    rc, out = run('--extrinsics', EXTR['10-03'])
    check('4.1 10-03（与物理一致的那份）→ rc=1「当前不满足」', rc == 1,
          'rc=%d' % rc)
    check('4.2 报出 8 个槽全部在上边界外，且最急的是槽位[6]',
          out.count('上边出框') == 8 and '槽位[6]' in out,
          '\n'.join(l for l in out.splitlines() if '出框' in l)[:400])
    check('4.3 抓取区**在框内**（它一直看得见 —— 这个是物理不变量）',
          '抓取区四角' in out and '✅ 在框内' in out)
    check('4.4 自检打印的「光轴 vs 台面法线」是 35.95°，帧语义自检通过',
          '35.95' in out and '帧语义自检通过' in out)
    check('4.5 给出了等效调整量（升高 / 平移 / 挪槽位三条路）',
          '升高' in out and 'place_slots 沿 y' in out)

    # 10-02 是**镜像**的那一份：它把抓取区自己投到画面外 —— 这就是判它的依据
    rc2, out2 = run('--extrinsics', EXTR['10-02'])
    check('4.6 10-02（镜像版）→ rc=2，而不是 rc=1', rc2 == 2, 'rc=%d' % rc2)
    check('4.7 且给出的理由是「抓取区自己就不在画面里」'
          '（不是「取景不满足」—— 那是两件事）',
          '抓取区' in out2 and '不在画面' in out2,
          '\n'.join(l for l in out2.splitlines() if '抓取区' in l)[:300])
    check('4.8 这条不变量确实能分开两份外参（不是靠 35.95° 那种两边都成立的判据）',
          '35.95' in out2, '两份外参的光轴夹角确实都是 35.95°')

    # 计划中的挪动：模拟 +34 cm 后应当全部进框
    rc3, out3 = run('--extrinsics', EXTR['10-03'], '--sim-raise-cm', '34')
    check('4.9 模拟升高 34 cm → 打印「模拟之后全部在框内」',
          '模拟之后全部在框内' in out3,
          '\n'.join(l for l in out3.splitlines() if '模拟之后' in l))
    check('4.10 但退出码仍描述**当前**机位（=1，还需要真的动手挪一次）',
          rc3 == 1, 'rc=%d' % rc3)
    check('4.11 结论里「当前」与「模拟之后」两句都在，不会混淆',
          '取景结论（**当前**）' in out3 and '取景结论（**模拟之后**）' in out3)

    rc4, out4 = run('--extrinsics', '/nonexistent.json')
    check('4.12 外参读不到 → rc=2（环境问题，不是「取景不满足」）', rc4 == 2,
          'rc=%d' % rc4)

    # ---- `--camera`：重标定的产出（`topcam_marker_calib.py --solve`）只**打印**
    # 数、不落盘，所以取景收敛器必须能直接吃它，否则「solve → 看还差多少」这一环
    # 是断的（只能手抄进 json，而抄错一个数是**静默**的）。
    # 帧语义按 launch：`base_link ← camera_link`（三方证据见实现里的注释）。
    R_lnk, t_lnk = (lambda d: (np.array(d['R'], dtype=float),
                               np.array(d['t'], dtype=float)))(
        __import__('json').load(open(EXTR['10-03'], encoding='utf-8')))
    R_opt, t_opt = REAL.dzf.compose_base_optical(
        __import__('json').load(open(EXTR['10-03'], encoding='utf-8')))

    rc5, out5 = run('--extrinsics', EXTR['10-03'])
    cam_link = [str(v) for v in list(t_lnk) + list(mat_to_quat(R_lnk))]
    rc6, out6 = run('--camera', *cam_link)
    check('4.13 `--camera` 喂文件里的 (t,R→q) 与 `--extrinsics` 读同一个文件 '
          '**逐字同结论**（证明命令行这条也做了 link→光学系的复合，不是照单全收）',
          rc6 == rc5 and same_rows(out5, out6), 'rc=%d vs %d' % (rc6, rc5))
    check('4.14 且打印出它用的 7 个数与假定的帧（抄错/给错帧时看得出来）',
          '外参：--camera' in out6 and '帧=camera_link' in out6)

    # 帧声明给错的**两个方向**，各自被不同的闸门拦下 —— 两条都要钉住，
    # 因为少钉一条就会以为「反正错了会报」，而另一条其实是静默的。
    cam_opt = [str(v) for v in list(t_opt) + list(mat_to_quat(R_opt))]

    # (a) 光学系的数当 camera_link 喂 = **多复合一次** link→光学系
    rc7, out7 = run('--camera', *cam_opt)
    check('4.15 光学系数当 camera_link 喂（多复合一次）→ rc=2，'
          '拦它的是**光轴夹角**（88.05°，掉出 [20,70]）',
          rc7 == 2 and '88.05' in out7 and '帧关系搞错' in out7, 'rc=%d' % rc7)

    # (b) 反过来：link 的数声明成 optical = **少复合一次**。这条夹角**拦不住**
    #     （54.33° 仍在 [20,70] 内），靠的是「抓取区必须可见」这条物理不变量。
    rc7b, out7b = run('--camera', *cam_link, '--camera-frame', 'optical')
    check('4.15b link 数声明成 optical（少复合一次）→ rc=2，而拦它的是'
          '「抓取区自己就不在画面里」',
          rc7b == 2 and '不在画面' in out7b, 'rc=%d' % rc7b)
    check('    且这条夹角确实**拦不住**它（54.33° 仍在 [20,70] 内）—— '
          '两个方向的拦法不同，不能只靠夹角',
          '54.33' in out7b and '帧语义自检通过' in out7b,
          '\n'.join(l for l in out7b.splitlines() if '法线' in l))

    rc8, out8 = run('--camera', *cam_opt, '--camera-frame', 'optical')
    check('4.16 同一组数显式声明 `--camera-frame optical` → 与文件路同结论 '
          '（所以两条路的差别**只**在帧声明，没有别的隐藏换算）',
          rc8 == rc5 and same_rows(out5, out8), 'rc=%d vs %d' % (rc8, rc5))

    # 文档里那组「实机 A/B 定下的」7 个数（docs/待执行命令.md:142-158）—— 现场
    # 真正会填进 launch 的就是它。它与留档文件 10-03 的 (R,t) 是不是同一组，
    # 此前只有文档里的一句断言；这里用投影钉住（口径统一是段二的验收项之一）。
    doc7 = ["0.34694477757210807", "0.8693043936314018", "0.6454397556505314",
            "-0.32913450910630504", "-0.316374655430605",
            "0.6185281368827056", "-0.6395314661538762"]
    rc12, out12 = run('--camera', *doc7)
    check('4.19 文档里 A/B 定下的那组数与留档 10-03 的 (R,t) 投出**同一份画面**'
          '（文档与文件口径一致 —— 此前只有文档里一句断言）',
          rc12 == rc5 and same_rows(out5, out12), 'rc=%d vs %d' % (rc12, rc5))

    # 互斥且必给其一；`--camera-frame` 只对 --camera 有意义（静默忽略会让人
    # 以为它能把任意 JSON「解释成」别的帧）
    rc9, _ = run()
    rc10, _ = run('--extrinsics', EXTR['10-03'], '--camera', *cam_link)
    rc11, out11 = run('--extrinsics', EXTR['10-03'], '--camera-frame', 'optical')
    check('4.17 两种给法互斥且必给其一（都不给 / 同时给 → 都用不了）',
          rc9 != 0 and rc10 != 0, 'rc=%d / %d' % (rc9, rc10))
    check('4.18 `--extrinsics` + `--camera-frame` → rc=2 明说只对 --camera 有意义'
          '（不静默忽略）',
          rc11 == 2 and '只对 --camera 有意义' in out11, 'rc=%d' % rc11)


# ----------------------------------------------------------------------
# [5] 变异
# ----------------------------------------------------------------------
def part5_mutation():
    print('\n[5] 变异：改坏一处，对应断言必须失效')

    # M1：把余量判据降级成「只要在画面里」—— 于是「差 3 px 余量」会放行，
    #     现场挪完还得再挪一次（这个工具存在的意义就没了）。
    m = load(mutate('    if vmin < margin:', '    if vmin < 0.0:'), 'm1')
    r = m.check_one(('x', 100.0, 200.0, 5.0, 200.0, False), 1280, 720, 8.0)
    check('M1 余量判据降级 → 只差 3 px 余量也判合格（所以 2.4 不是空转）',
          r[0] is True, str(r))

    # M2：干脆不看 v（只留横向）—— 8 个槽全在上边界外会被判「合格」，
    #     而这正是 A2 跑完才发现的那件事。
    m = load(mutate('    if vmin < margin:                       '
                    '# 上边界（画面顶部 = 远处）',
                    '    if False:                               '
                    '# 上边界（画面顶部 = 远处）'), 'm2')
    r = m.check_one(('x', 500.0, 600.0, -150.0, -100.0, False), 1280, 720, 8.0)
    check('M2 去掉上边界判据 → 整项在画面上方 150 px 也判「在框内」',
          r[0] is True, str(r))

    # M3：把 y 灵敏度的作用轴写错（动 y 却改了 x）—— 3.4/3.5 的反向关系必须破。
    m = load(mutate("    t[1] += move_y_cm / 100.0",
                    "    t[0] += move_y_cm / 100.0"), 'm3')
    import json
    doc = json.load(open(EXTR['10-03'], encoding='utf-8'))
    R, t = REAL.dzf.compose_base_optical(doc)
    items = REAL.framing_items(REAL.REAL_SLOTS, REAL.REAL_PICK,
                               REAL.REAL_BLOCK_W, REAL.REAL_BLOCK_H)
    probes = [p for name, pts in items if name.startswith('槽位') for p in pts]
    s_my = m.sensitivity(R, t, REAL.REAL_K, probes, 'move_y')
    s_sloty = m.sensitivity_slots_y(R, t, REAL.REAL_K, probes)
    s_my_ok = REAL.sensitivity(R, t, REAL.REAL_K, probes, 'move_y')
    # ⚠️ 判据要用**量级**关系（3.5），不能用符号（3.4）：这台机位下动 x 的
    #    效应也是负的（-0.14 px/cm），所以「反向」在变异后照样成立 ——
    #    拿符号当判据会放过这个变异（第一版就是这么写的，变异被放过了）。
    check('M3 动 y 却改了 x → 与槽位那一路的量级关系破掉'
          '（所以 3.5 承担了这条交叉校验）',
          abs(abs(s_my) - abs(s_sloty)) / abs(s_my) > 0.20,
          '变异后 相机 %.3f / 槽位 %.3f（真代码是 %.2f / %.2f）'
          % (s_my, s_sloty, s_my_ok, s_sloty))

    # M4：「自证」把升高量丢掉（用原来的 t 复核）—— 复核输出会与挪动前
    #     一模一样，看上去像「外推不准」。这个 bug 真发生过。
    print('  --- M4：自证丢掉升高量（apply_move 的高度写在返回的 t 里）---')
    src = open(SCRIPT).read()
    mut = mutate('                R_chk, t_chk = apply_move(R, t, raise_cm=cm)\n'
                 '                left = [r for r in frame_one(R_chk, t_chk, K, items)',
                 '                R_chk, t_chk = apply_move(R, t, raise_cm=cm)\n'
                 '                left = [r for r in frame_one(R_chk, t, K, items)',
                 src)
    d = tempfile.mkdtemp(prefix='topcamframe_')
    mpath = os.path.join(d, 'topcam_framing_m4.py')
    with open(mpath, 'w') as f:
        f.write(mut)
    # ⚠️ 变异副本在临时目录里，而它自己要 `import draw_zone_on_frame` ——
    #    不给 PYTHONPATH 的话它会 ImportError 退出，而捕获的输出里什么都没有，
    #    看上去像「变异没生效」。（第一版就踩了这个：断言失败但原因是脚本没跑起来。）
    env = dict(os.environ)
    env['PYTHONPATH'] = HERE + os.pathsep + env.get('PYTHONPATH', '')
    p = subprocess.run([sys.executable, mpath, '--extrinsics', EXTR['10-03']],
                       capture_output=True, text=True, env=env)
    out_mut = p.stdout + p.stderr
    line_mut = '\n'.join(l for l in out_mut.splitlines() if '按升高' in l)
    _, out_real = run('--extrinsics', EXTR['10-03'])
    line_real = '\n'.join(l for l in out_real.splitlines() if '按升高' in l)
    check('M4 丢掉升高量后，复核结果与挪动前**一模一样**（仍有 8 项）',
          '仍有 8 项' in line_mut, line_mut)
    check('  而真代码的复核结果确实变好了（不合格项数 < 8）—— '
          '所以自证那一步是有内容的',
          '仍有 8 项' not in line_real and '按升高' in line_real, line_real)

    # M6：`--camera` 的四元数顺序写错（喂 [w,x,y,z] 而不是 [x,y,z,w]）——
    #     4.13 的「两条路逐字同结论」必须失效。不钉这一条的话，4.13 可能只是
    #     「两条路都恰好算出同一份错的投影」而看不出来。
    m6 = load(mutate("            R0 = dzf.quat_to_matrix([qx, qy, qz, qw])",
                     "            R0 = dzf.quat_to_matrix([qw, qx, qy, qz])"), 'm6')
    # 这里的 R_lnk/t_lnk 是 part4 里的局部量，不跨函数 —— 就地重取一次。
    _d = json.load(open(EXTR['10-03'], encoding='utf-8'))
    R_lnk = np.array(_d['R'], dtype=float)
    t_lnk = list(_d['t'])
    q_ok = mat_to_quat(R_lnk)

    def rows_of(R):
        return m6.frame_one(*m6.dzf.compose_base_optical(
            {"R": R.tolist(), "t": list(t_lnk),
             "frame_child": "camera_link"}), m6.REAL_K, items)

    R_right = m6.dzf.quat_to_matrix([q_ok[0], q_ok[1], q_ok[2], q_ok[3]])
    R_wrong = m6.dzf.quat_to_matrix([q_ok[3], q_ok[0], q_ok[1], q_ok[2]])
    check('M6 四元数顺序写错 → 投影与正确顺序**不同**（所以 4.13 不是空转）',
          rows_of(R_right) != rows_of(R_wrong),
          '两种顺序投出了同一份画面 —— 那 4.13 就证明不了什么')
    check('    而正确顺序下 R→q→R 是恒等的（4.13 的两条路本该同结论）',
          float(np.abs(R_right - R_lnk).max()) < 1e-9,
          str(float(np.abs(R_right - R_lnk).max())))

    # M5：挪槽位的建议少了单位换算（cm → m）——「会把槽位推进抓取区」这条
    #     护栏就永远不会触发，而照着做会让规划器**启动就拒**。
    m = load(mutate('    dy_m = dy_cm / 100.0', '    dy_m = dy_cm'), 'm5')
    check('M5 单位换算丢掉后，护栏判不出「槽位推进抓取区」（所以 4.x 的 '
          '❌ 那条不是空转）',
          not m.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, 43.3),
          '变异体仍判出冲突（说明 6.1 是单位换算在起作用）')


def part6_slot_clash_guard():
    """4.x 里那条 ❌ 的判据本身（真代码路径）。"""
    print('\n[6] 「挪槽位会推进抓取区」的护栏')
    check('6.1 把 8 个槽整体沿 y 挪 +43 cm → 判出冲突（规划器启动会拒）',
          bool(REAL.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, 43.3)))
    check('6.1b 并且点名是哪些槽（现场要据此判断能不能只挪一部分）',
          len(REAL.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, 43.3)) == 8,
          str(REAL.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, 43.3)))
    check('6.2 挪 +3 cm → 不冲突（小改动照做）',
          not REAL.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, 3.0))
    check('6.3 沿 y 挪 -5 cm（更远离抓取区）→ 不冲突',
          not REAL.slot_clashes(REAL.REAL_SLOTS, REAL.REAL_PICK, -5.0))
    check('6.4 没有抓取区配置时不报冲突（无约束不等于有约束）',
          not REAL.slot_clashes(REAL.REAL_SLOTS, None, 43.3))


def main():
    print('=' * 72)
    print('俯视取景收敛器自检   %s' % os.path.relpath(SCRIPT, REPO))
    print('=' * 72)
    part1_geometry()
    part2_check_one()
    part3_sensitivity()
    part4_cli()
    part6_slot_clash_guard()
    part5_mutation()

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
