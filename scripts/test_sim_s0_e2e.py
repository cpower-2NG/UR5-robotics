#!/usr/bin/env python3
"""E1 评分器自检（离线）：``sim_s0_e2e.score_run`` / ``compare_runs``。

为什么评分器要单独自检
======================
E1 要防的是**假绿灯**：一次抓取失败、方块根本没动，评分器却写 PASS。
仿真里那种错误**很难靠眼神发现**（终端上一大串 PASS），所以评分器本身必须
被注入验证 —— 而且每条注入都要指出**哪一条判据**该红，只看总 PASS/FAIL
是不够的：判据被写死成恒真时，总判还是能因为别的判据而红，掩盖掉死判据。

本文件钉住：
  A. 正常 run（初始位姿与落点抄 S0:0 的**实测值**）→ PASS
  B. 四类失败注入 → 红，且红在预期的那条判据上：
       collider_missing   邻块被撞开 30 mm（碰撞体没进场景的后果）
       attach_missing     指定块原地没动、也没被抬起（attach 丢失）
       wrong_frame        位姿按绕 z 180° 换错坐标系
       place_z_high       规划器把方块在槽位上 30 mm 处就松手（阈值错）
  C. 阈值承重：同一个 12 mm 落点偏差，20 mm 容差放过、5 mm 容差抓住
  D. 复现比对：三次一致 → PASS；换目标块 / 落点散 15 mm → 红
  E. 缺块必须报错，不许静默当成「没动」而给 PASS

跑法：``python3 scripts/test_sim_s0_e2e.py``；退出码 0 = 全通过，2 = 有失败。
本自检**不需要 ROS、不需要仿真**（``sim_s0_e2e`` 的 ROS 依赖全在函数体内）。
"""

import copy
import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim_s0_e2e as E                                          # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


# ---------------------------------------------------------------------
# 夹具：S0:0 的初始真值（gen_scene.py 固定种子，逐位相同）
# ---------------------------------------------------------------------
PRE = {
    0: (0.3212728, 0.2916508, 0.01475),   # purple
    1: (0.4317579, 0.2658501, 0.01475),   # cyan
    2: (0.2991401, 0.2007207, 0.01475),   # orange
    3: (0.5395167, 0.2152334, 0.01475),   # tan
    4: (0.4620780, 0.1906095, 0.01475),   # yellow
}
SLOTS = [[0.2942, -0.17724], [0.4242, -0.17724], [0.5542, -0.17724],
         [0.2942, -0.04724], [0.4242, -0.04724], [0.5542, -0.04724]]
# 2026-11-10 实测：紫块落在 0 号槽 2.2 mm 处、中心 z=0.01763（垫面 + 半块高）
POST_OK = dict(PRE)
POST_OK[0] = (0.2942 + 0.0015, -0.17724 - 0.0016, 0.01763)
LIFT_OK = 0.19


def run(pre=PRE, post=POST_OK, lift=LIFT_OK, **kw):
    return E.score_run(pre, post, 0, SLOTS, lift, **kw)


def failing(res):
    return {k for k, v in res['checks'].items() if not v['ok']}


def rotate_z180(p, c=(0.4, 0.1)):
    """绕 base z 转 180°（真机上 tcp_pose 帧搞错时的典型表现）。"""
    return (2 * c[0] - p[0], 2 * c[1] - p[1], p[2])


def main():
    # =================================================================
    print('--- A. 正常 run ---')
    r = run()
    check('A1 正常 run 判 PASS', r['pass'], str(failing(r)))
    check('A2 槽位认成 0 号', r['slot_index'] == 0, str(r['slot_index']))
    check('A3 落点距槽位 ≤ 5 mm', r['slot_dist_mm'] < 5.0,
          '%.2f mm' % r['slot_dist_mm'])
    check('A4 五条判据齐全',
          set(r['checks']) == {'target_moved', 'target_lifted',
                               'target_in_slot', 'others_unmoved',
                               'all_above_table'}, str(sorted(r['checks'])))
    check('A5 post_xy 回报（compare_runs 要用）',
          r['post_xy'] == list(POST_OK[0][:2]), str(r.get('post_xy')))

    # =================================================================
    print('\n--- B. 失败注入（每一条都要红在指定判据上）---')
    # B1 删掉碰撞体：邻块被撞开
    post = dict(POST_OK)
    post[1] = (PRE[1][0] + 0.030, PRE[1][1], PRE[1][2])
    r = run(post=post)
    check('B1 碰撞体缺失（邻块被撞 30 mm）→ 红',
          not r['pass'] and 'others_unmoved' in failing(r),
          str(failing(r)))

    # B2 删掉 attach：方块没跟着走
    r = run(post=dict(PRE), lift=PRE[0][2])
    check('B2 attach 缺失（块没动、没抬）→ 红',
          not r['pass'] and {'target_moved', 'target_lifted'} <= failing(r),
          str(failing(r)))

    # B3 错坐标系：绕 z 180°
    post = {i: rotate_z180(p) for i, p in POST_OK.items()}
    r = run(post=post)
    check('B3 坐标系错（绕 z 180°）→ 红',
          not r['pass'] and 'target_in_slot' in failing(r), str(failing(r)))

    # B4 阈值错：规划器在槽位上方 30 mm 松手，方块滚下垫子
    post = dict(POST_OK)
    post[0] = (0.2942 + 0.030, -0.17724 + 0.018, 0.01475)
    r = run(post=post)
    check('B4 place_z 偏高 30 mm → 红',
          not r['pass'] and 'target_in_slot' in failing(r), str(failing(r)))

    # =================================================================
    print('\n--- C. 阈值承重：同一偏差在两种容差下必须翻面 ---')
    post = dict(POST_OK)
    post[0] = (0.2942 + 0.012, -0.17724, 0.01763)     # 偏 12 mm
    loose = run(post=post, slot_tol_m=0.020)
    tight = run(post=post, slot_tol_m=0.005)
    check('C1 12 mm 偏差在 20 mm 容差下放过（说明默认容差是会漏的）',
          loose['pass'], str(failing(loose)))
    check('C2 同一偏差在 5 mm 容差下抓住（说明容差真的接线了）',
          not tight['pass'] and 'target_in_slot' in failing(tight),
          str(failing(tight)))

    # =================================================================
    print('\n--- D. 复现比对 ---')
    a = run()
    b = run(post={**POST_OK, 0: (0.2942 - 0.001, -0.17724 + 0.001, 0.01763)})
    c = run(post={**POST_OK, 0: (0.2942, -0.17724 - 0.002, 0.01763)})
    cmp_ok = E.compare_runs([a, b, c])
    check('D1 三次一致 → 可复现', cmp_ok['pass'], str(cmp_ok['checks']))
    check('D2 落点两两最大差 ≤ 5 mm（实测口径）', cmp_ok['spread_mm'] <= 5.0,
          '%.2f mm' % cmp_ok['spread_mm'])

    bad = copy.deepcopy(c)
    bad['target_block'] = 1
    cmp_bad = E.compare_runs([a, b, bad])
    check('D3 换目标块 → 红',
          not cmp_bad['pass'] and not cmp_bad['checks']['same_target']['ok'],
          str(cmp_bad['checks']))

    far = run(post={**POST_OK, 0: (0.2942 - 0.015, -0.17724, 0.01763)})
    cmp_far = E.compare_runs([a, far, c])
    check('D4 落点散 15 mm（各自仍可能 pass）→ 复现判红',
          not cmp_far['pass']
          and not cmp_far['checks']['landing_spread']['ok'],
          str(cmp_far['checks']))

    # =================================================================
    print('\n--- E. 缺块不许静默 ---')
    post = dict(POST_OK)
    del post[3]
    try:
        run(post=post)
        raised = False
    except ValueError:
        raised = True
    check('E1 运行后读不到某块 → 抛错（不当成"没动"）', raised)

    # =================================================================
    print('\n--- F. 指定通道的输出解析 ---')
    sample = ('解析: {"kind": "position", "color": "purple", "position": "center"}\n'
              '结果: {\n "status": "ok", "instance": 4,\n'
              ' "block": {"id": 4, "x": 321.8, "y": 292.9, "top": 30.5}\n}\n')
    got = E.parse_specify_output(sample)
    check('F1 从「结果:」之后取 JSON（不整段 loads）',
          got['status'] == 'ok' and got['block']['x'] == 321.8, str(got))
    try:
        E.parse_specify_output('没有任何标记\n')
        raised = False
    except ValueError:
        raised = True
    check('F2 输出格式变了 → 抛错（不返回空 dict）', raised)

    print('\n%s' % ('=' * 60))
    print('PASS %d   FAIL %d' % (N_PASS, N_FAIL))
    if FAILS:
        for f in FAILS:
            print('  ✗ %s' % f)
    sys.exit(0 if N_FAIL == 0 else 2)


if __name__ == '__main__':
    main()
