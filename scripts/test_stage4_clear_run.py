#!/usr/bin/env python3
"""E9 执行器（探针方案 → 抓放计划序列）的离线自检。

为什么这些值得单测：这一层不产生任何「新几何」，它的全部风险都在**翻译**
——把探针的 ``clearing_steps`` 翻成 ``build_pick`` 的入参。翻错的三种形态
在这轮开发里都真实出现过：

1. 起点/终点错位：把「块已搬到落点」的布局拿去算搬运那一步，臂会跑去
   错的地方（且计划本身看起来完全合法）；
2. 方块高度当成块顶高度：压在目标上的块（S4 有塔）会被按 59 mm 高的块去
   放置 ⇒ 放到 249.7 mm 而不是 220.2 mm，等于把块从 30 mm 高处扔下；
3. 60 mm 邻块判据：E9 的目标恰恰是贴面块，粗判据会把本来合法的方案拒掉。

跑法：python3 scripts/test_stage4_clear_run.py
"""
import inspect
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_plan as pp                                     # noqa: E402
import stage4_clear_model as cm                                 # noqa: E402
import stage4_clear_run as cr                                   # noqa: E402
import urkin                                                    # noqa: E402

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


def raises(fn, *a, **kw):
    try:
        fn(*a, **kw)
    except Exception as e:                                # noqa: BLE001
        return True, repr(e)
    return False, ''


def _mut(old, new):
    src = inspect.getsource(cr)
    assert old in src, '变异锚点不在源码里：%r' % old
    ns = {'__file__': cr.__file__, '__name__': 'mut'}
    exec(compile(src.replace(old, new, 1), '<mut>', 'exec'), ns)
    return ns


# ---------------------------------------------------------------- 夹具
URDF = os.path.expanduser('~/robotics/notes/ur5-vision-grasp/runs/'
                          '2026-10-25-precision/robot.urdf')
Q_HOME = {'shoulder_pan_joint': -0.68785, 'shoulder_lift_joint': -1.51061,
          'elbow_joint': 1.21013, 'wrist_1_joint': -1.26955,
          'wrist_2_joint': -1.56841, 'wrist_3_joint': -2.26060}


def chain():
    return urkin.Chain(URDF)


def q_start_of(ch):
    return [Q_HOME[n] for n in ch.move_joints]


def blk(i, x, y, top=0.0295, size=0.0295, yaw=0.0):
    return {'id': i, 'color': 'c%d' % i, 'size': size, 'top': top,
            'z': top - size / 2.0, 'x': x, 'y': y, 'yaw': yaw}


def tower_truth():
    """一个 2 块塔 + 一个独立块：目标 0 在塔底，块 8 压在上面。"""
    return {'batch': 'T', 'index': 0,
            'blocks': [blk(0, 0.40, 0.30, yaw=146.63),
                       blk(8, 0.40, 0.30, top=0.059, yaw=153.35),
                       blk(3, 0.60, 0.30, yaw=20.0)]}


def tower_attempt():
    return {'id': 0, 'ok': True,
            'grasp_hit': {'axis': 2, 'sign': -1, 'closing': 0,
                          'variant': 0, 'vertical': True, 'verdict': 'ok'},
            'clearing_steps': [{
                'move': 8,
                'from': [0.40, 0.30, 0.04425],
                'to': [0.52, 0.54, 0.01475],
                'grasp': {'axis': 2, 'sign': -1, 'closing': 1,
                          'variant': 1, 'vertical': True, 'verdict': 'ok'}}],
            'final_blocks': [{'id': 0, 'x': 0.4242, 'y': -0.11224,
                              'z': 0.01763, 'yaw': 0.0, 'size': 0.0295},
                             {'id': 8, 'x': 0.52, 'y': 0.54, 'z': 0.01475,
                              'yaw': 0.0, 'size': 0.0295},
                             {'id': 3, 'x': 0.60, 'y': 0.30, 'z': 0.01475,
                              'yaw': 0.0, 'size': 0.0295}]}


# ------------------------------------------------------------------ [1]
def section_yaw():
    print('\n[1] 抓取候选 → tool0 绕 z 角')
    hit0 = {'axis': 2, 'sign': -1, 'closing': 0, 'variant': 0,
            'vertical': True}
    hit1 = dict(hit0, closing=1)
    check('closing=0 ⇒ ψ = 块 yaw', cr.grasp_yaw_deg(hit0, 30.0) == 30.0)
    check('closing=1 ⇒ ψ = 块 yaw + 90°',
          cr.grasp_yaw_deg(hit1, 30.0) == 120.0)
    check('variant 不改变 ψ（绕 a 转 180° 是物理等价）',
          cr.grasp_yaw_deg(dict(hit0, variant=1), 30.0)
          == cr.grasp_yaw_deg(hit0, 30.0))
    ok, why = raises(cr.grasp_yaw_deg, dict(hit0, axis=0, sign=1,
                                            vertical=False), 0.0)
    check('非竖直候选 ⇒ 拒绝（E9 只执行 B 档）', ok, why)
    ok, why = raises(cr.grasp_yaw_deg, dict(hit0, vertical=False), 0.0)
    check('vertical=False ⇒ 拒绝', ok, why)
    ok, why = raises(cr.grasp_yaw_deg, None, 0.0)
    check('没有命中记录 ⇒ 拒绝（fail-closed）', ok, why)
    # 180° 折叠：ψ 与 ψ+180 在 build_pick 里是同一份几何
    check('ψ+180° 折叠后与原角等价',
          pp.fold_yaw_180(30.0) == pp.fold_yaw_180(210.0))


# ------------------------------------------------------------------ [2]
def section_surface():
    print('\n[2] 落点支撑面：只认台面 / 垫面')
    check('块心 14.75 mm、边长 29.5 ⇒ 台面 0.0',
          abs(cr.surface_of(0.01475, 0.0295, 'x') - 0.0) < 1e-12)
    check('块心 垫厚+14.75 mm ⇒ 垫面',
          abs(cr.surface_of(cm.PAD_THICK + 0.01475, 0.0295, 'x')
              - cm.PAD_THICK) < 1e-12)
    ok, why = raises(cr.surface_of, 0.030, 0.0295, 'x')
    check('既不是台面也不是垫面 ⇒ 拒绝（不猜）', ok, why)


# ------------------------------------------------------------------ [3]
def section_actions():
    print('\n[3] 探针方案 → 动作表')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    check('清障在前、目标在最后',
          [a['kind'] for a in acts] == ['clear', 'target'])
    check('清障块 id 与探针一致', acts[0]['block'] == '8')
    check('起点用探针记录的 step["from"]（0.40, 0.30），不是落点',
          acts[0]['from_xy'] == [0.40, 0.30])
    check('清障落点 = step["to"] 的 xy',
          acts[0]['slot'] == [0.52, 0.54])
    check('清障落点支撑面 = 台面', acts[0]['surface_z'] == 0.0)
    check('目标落点支撑面 = 垫面',
          abs(acts[1]['surface_z'] - cm.PAD_THICK) < 1e-12)
    check('返回的是**原始**布局（块 8 仍在塔上 top=59 mm）',
          cr.find_block(blocks, '8')['top'] == 0.059)
    check('清障 ψ = 块 yaw + 90°（closing=1）',
          abs(acts[0]['grasp_yaw_deg'] - 243.35) < 1e-6)
    ok, why = raises(cr.actions_from_attempt, truth,
                     dict(att, final_blocks=[]), cm.PAD_THICK)
    check('终态里没有目标块 ⇒ 拒绝', ok, why)
    bad = json.loads(json.dumps(att))
    bad['final_blocks'][0]['z'] = 0.01475          # 目标被当成放在台面
    ok, why = raises(cr.actions_from_attempt, truth, bad, cm.PAD_THICK)
    check('目标落点不在垫上 ⇒ 拒绝（与探针理解不一致）', ok, why)


# ------------------------------------------------------------------ [4]
def section_plans():
    print('\n[4] 计划序列：整链衔接 + 塔块高度')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    out = tempfile.mkdtemp(prefix='e9-plan-unit-')
    os.makedirs(out, exist_ok=True)
    plans, layout, q_end = cr.plan_sequence(ch, q0, acts, blocks, out)

    check('两个动作各出一份计划', len(plans) == 2)
    p1 = json.load(open(plans[0]['path'], encoding='utf-8'))
    p2 = json.load(open(plans[1]['path'], encoding='utf-8'))
    check('第一份 q_start = 起始关节', p1['q_start'] == q0)
    check('第二份 q_start = 第一份终点（整链衔接）',
          p2['q_start'] == p1['waypoints'][-1]['q'])
    check('每份 8 个航点（A3 老序列）',
          len(p1['waypoints']) == 8 and len(p2['waypoints']) == 8)
    check('航点顺序与 A3 一致',
          [w['name'] for w in p1['waypoints']]
          == ['approach', 'grasp', 'lift', 'to_slot', 'place_approach',
              'place', 'retreat', 'survey_place'])
    m1 = p1['meta']
    check('塔上的块：块顶 59 mm、物理块高 29.5 mm、块底 29.5 mm',
          m1['block_top_mm'] == 59.0 and m1['block_true_h_mm'] == 29.5
          and m1['block_bottom_mm'] == 29.5)
    check('夹持深度 18.5 mm', abs(m1['grasp_depth_mm'] - 18.5) < 0.05)
    check('指尖高于块底 5 mm 下限',
          m1['fingertip_z_mm'] - m1['block_bottom_mm']
          >= pp.FINGERTIP_MIN_ABOVE_BLOCK_BOTTOM * 1000 - 1e-9)
    check('塔块放下高度 ≈ 220.2 mm（按 29.5 mm 挂长算）',
          abs(m1['place_z'] - 0.2202) < 1e-3)
    check('gap 判据被显式放宽（记录在案）',
          p1['meta']['e9']['gap_check'] == 'probe_verified')
    check('清障那一步按块 8 的 yaw+90° 抓（ψ=−26.653° 折叠后）',
          abs(p1['meta']['yaw_grasp_deg'] - pp.fold_yaw_180(243.35)) < 1e-3)
    check('终态布局：块 8 落到 (0.52, 0.54)、台面',
          cr.find_block(layout, '8')['x'] == 0.52
          and cr.find_block(layout, '8')['top'] == 0.0295)
    check('终态布局：目标块在垫上 top = 垫厚 + 边长',
          abs(cr.find_block(layout, '0')['top']
              - (cm.PAD_THICK + 0.0295)) < 1e-9)
    check('q_end = 最后一份计划的终点',
          q_end == p2['waypoints'][-1]['q'])

    # 搬运那一步的抓点必须在**塔上的位置**
    ap = p1['waypoints'][0]
    check('approach 的 q 与 q_start 不同（确实动了）', ap['q'] != q0)


# ------------------------------------------------------------------ [5]
def section_gap():
    print('\n[5] 60 mm 邻块判据：E9 必须能放宽')
    ch = chain()
    q0 = q_start_of(ch)
    a = blk(0, 0.40, 0.30)
    b = blk(1, 0.44, 0.30)          # 中心距 40 mm —— 贴面/近邻
    ppb = [cr.to_pp_block(a), cr.to_pp_block(b)]
    cand = {'xyz_base': [0.40, 0.30, 0.0295], 'score': 0.0,
            'width_mm': 29.5, 'dist_mm': 0.0, 'angle_deg': 0.0, 'seed': None}
    ok, why = raises(pp.build_pick, ch, q0, cand, ppb[0], ppb, [0.52, 0.54],
                     grasp_at='block_center')
    check('默认（60 mm）下贴面块被拒 —— 老行为不变', ok, why)
    try:
        pp.build_pick(ch, q0, cand, ppb[0], ppb, [0.52, 0.54],
                      grasp_at='block_center', min_block_gap=0.0)
        passed = True
    except pp.PlanError:
        passed = False
    check('min_block_gap=0.0 ⇒ 放行（E9 的探针已判过间隙）', passed)


# ------------------------------------------------------------------ [6]
def section_mutation():
    print('\n[6] 变异：翻错就该红')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    out = tempfile.mkdtemp(prefix='e9-plan-mut-')
    os.makedirs(out, exist_ok=True)
    good, _, _ = cr.plan_sequence(ch, q0, acts, blocks, out)
    good_place = json.load(open(good[0]['path']))['meta']['place_z']

    mut = _mut('block_h_m=float(blk["size"]),', 'block_h_m=None,')
    ok, why = raises(mut['plan_sequence'], ch, q0, acts,
                     [dict(b) for b in blocks], out)
    check('变异「不给物理块高」：塔块被 50 mm 护栏当场拦下', ok, why)

    # 护栏拦下之后，还有一种更隐蔽的翻法：把「块顶高」当成「块高」显式传进去。
    # 这时护栏不响，夹持深度也对（18.5 mm 只跟 wz 有关），**只有放置高度错**。
    a0 = cr.to_pp_block(cr.find_block(blocks, '8'))      # 塔上那块（top=59）
    ppb = [cr.to_pp_block(b) for b in blocks]
    cand = {'xyz_base': [0.40, 0.30, 0.059], 'score': 0.0, 'width_mm': 29.5,
            'dist_mm': 0.0, 'angle_deg': 0.0, 'seed': None}
    _w, _c, meta_ok = pp.build_pick(ch, q0, cand, a0, ppb, [0.52, 0.54],
                                    grasp_at='block_center',
                                    block_h_m=0.0295, min_block_gap=0.0)
    _w, _c, meta_bad = pp.build_pick(ch, q0, cand, a0, ppb, [0.52, 0.54],
                                     grasp_at='block_center',
                                     block_h_m=0.059, min_block_gap=0.0)
    check('把块顶高当块高 ⇒ 放置高度抬高 9 mm（护栏看不见的那种翻法）',
          abs(meta_bad['place_z'] - meta_ok['place_z']) > 0.008,
          'ok=%.4f bad=%.4f' % (meta_ok['place_z'], meta_bad['place_z']))
    check('且夹持深度从 18.5 变成 39 mm —— 指尖落到塔底那块里 9.5 mm',
          abs(meta_bad['grasp_depth_mm'] - meta_ok['grasp_depth_mm']) > 0.02
          and meta_bad['fingertip_z_mm'] < 29.5,
          'bad depth=%.1f 指尖=%.1f（真块底=29.5）'
          % (meta_bad['grasp_depth_mm'], meta_bad['fingertip_z_mm']))
    check('这个错不会自己暴露：三条老护栏（50 mm 块顶、深度≤块高、指尖≥块底+5）'
          '在 bad 版里全部放行',
          meta_bad['fingertip_z_mm'] >= pp.FINGERTIP_MIN_ABOVE_BLOCK_BOTTOM
          * 1000 - 1e-9
          and meta_bad['grasp_depth_mm'] <= meta_bad['block_true_h_mm'] + 1e-9)

    # 贴面场景：目标与邻块中心距 40 mm。E9 探针判过 OBB 间隙 ⇒ 应能出计划；
    # 把放宽去掉就必须红 —— 否则说明这个参数根本没接上。
    adj = {'batch': 'T', 'index': 1,
           'blocks': [blk(0, 0.40, 0.30), blk(1, 0.44, 0.30)]}
    adj_att = {'id': 0, 'ok': True,
               'grasp_hit': {'axis': 2, 'sign': -1, 'closing': 0,
                             'variant': 0, 'vertical': True, 'verdict': 'ok'},
               'clearing_steps': [],
               'final_blocks': [{'id': 0, 'x': 0.4242, 'y': -0.11224,
                                 'z': 0.01763, 'yaw': 0.0, 'size': 0.0295},
                                {'id': 1, 'x': 0.44, 'y': 0.30,
                                 'z': 0.01475, 'yaw': 0.0, 'size': 0.0295}]}
    a_acts, a_blocks = cr.actions_from_attempt(adj, adj_att, cm.PAD_THICK)
    try:
        cr.plan_sequence(ch, q0, a_acts, a_blocks, out)
        adj_ok = True
    except pp.PlanError:
        adj_ok = False
    check('贴面块（中心距 40 mm）能出计划', adj_ok)
    mut2 = _mut('min_block_gap=0.0)', 'min_block_gap=None)')
    ok, why = raises(mut2['plan_sequence'], ch, q0, a_acts,
                     [dict(b) for b in a_blocks], out)
    check('变异「不放宽 60 mm」：贴面场景必须红', ok, why)

    mut3 = _mut('"from_xy": [float(src[0]), float(src[1])],',
                '"from_xy": [float(to[0]), float(to[1])],')
    mut_acts, _ = mut3['actions_from_attempt'](truth, att, cm.PAD_THICK)
    check('变异「起点误用落点」：from_xy 变成 (0.52, 0.54)',
          mut_acts[0]['from_xy'] == [0.52, 0.54])

    mut4 = _mut('    b["top"] = float(z_bottom) + float(b["size"])',
                '    pass')
    check('变异「搬完不更新块顶高度」：后续规划拿到旧高度',
          mut4['move_in_layout']([dict(blk(1, 0.5, 0.5))], 1, 0.6, 0.6,
                                 0.0295)['top'] == 0.0295,
          '未变异时应为 0.059')


def main():
    print('=' * 68)
    for fn in (section_yaw, section_surface, section_actions, section_plans,
               section_gap, section_mutation):
        try:
            fn()
        except Exception as e:                            # noqa: BLE001
            import traceback
            traceback.print_exc()
            check('%s 未抛异常' % fn.__name__, False, repr(e))
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
