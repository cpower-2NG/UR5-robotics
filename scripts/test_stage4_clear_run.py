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
import argparse
import inspect
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import precision_plan as pp                                     # noqa: E402
import stage4_clear_model as cm                                 # noqa: E402
import stage4_clear_run as cr                                   # noqa: E402
import urkin                                                    # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

# 垫面上静止块心的 z。**不许写死**：以前这里写死「0.01763」（= 旧 2.88 mm 垫厚
# + 半块高），§20.5 把垫厚改成真机实测 0.17 mm 后，写死的值会让
# `surface_of` 推出 0.00288 m 而判「既不是台面也不是垫面」。
# C1：模块缺省 = 未实测哨兵 ⇒ 本套件（模型世界）在 import 后显式装模型常量，
# 这个模块级常量才能保持「跟 cm 走」的原义。
cm.install_model_pad_defaults()
PAD_TOP_Z = cm.PAD_THICK + cm.BLOCK_M / 2.0


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


def system_exits(fn, *a, **kw):
    """``SystemExit`` 不是 ``Exception``（是 ``BaseException``），raises() 抓不到。"""
    try:
        fn(*a, **kw)
    except SystemExit as e:
        return True, str(e)
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


def solo_truth():
    """S0 形状：两块相隔 ~283 mm（无接触 ⇒ 60 mm 邻块粗判据应放行）。"""
    return {'batch': 'T', 'index': 0,
            'blocks': [blk(0, 0.40, 0.30, yaw=146.63),
                       blk(3, 0.60, 0.50, yaw=20.0)]}


def solo_attempt():
    return {'id': 0, 'ok': True,
            'grasp_hit': {'axis': 2, 'sign': -1, 'closing': 0,
                          'variant': 0, 'vertical': True, 'verdict': 'ok'},
            'clearing_steps': [],
            'final_blocks': [{'id': 0, 'x': 0.4242, 'y': -0.11224,
                              'z': PAD_TOP_Z, 'yaw': 146.63, 'size': 0.0295},
                             {'id': 3, 'x': 0.60, 'y': 0.50, 'z': 0.01475,
                              'yaw': 20.0, 'size': 0.0295}]}


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
                              'z': PAD_TOP_Z, 'yaw': 0.0, 'size': 0.0295},
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


# ------------------------------------------------------------------ [3b]
def section_bias():
    print('\n[3b] b̂ 接线：命令 = 格心 − b̂、三份坐标分开记、格位断言')
    truth, att = tower_truth(), tower_attempt()
    tgt_yaw = cr.find_block(truth['blocks'], 0)['yaw']
    att4 = json.loads(json.dumps(att))
    att4['cell'] = 4
    att4['final_blocks'][0]['yaw'] = tgt_yaw      # 探针按 place_in_cell 保块 yaw
    b = [-2.791, -0.248]
    acts, _ = cr.actions_from_attempt(truth, att4, cm.PAD_THICK, bias_mm=b)
    a = acts[-1]
    cx, cy = cm.cell_center(4)
    # 三份坐标落盘时 round(., 3)（卡尺精度就够），这里按同一精度比。
    check('期望仍是格心（判分口径不动）',
          abs(a['desired_center_mm'][0] - round(cx * 1000.0, 3)) < 1e-9 and
          abs(a['desired_center_mm'][1] - round(cy * 1000.0, 3)) < 1e-9,
          repr(a['desired_center_mm']))
    check('命令 = 格心 − b̂（符号：b̂ 负 ⇒ 命令往正方向推）',
          abs(a['commanded_center_mm'][0]
              - round(cx * 1000.0 + 2.791, 3)) < 1e-9 and
          abs(a['commanded_center_mm'][1]
              - round(cy * 1000.0 + 0.248, 3)) < 1e-9,
          repr(a['commanded_center_mm']))
    check('臂收到的 slot = 命令落点（不是期望）',
          abs(a['slot'][0] * 1000.0 - a['commanded_center_mm'][0]) < 1e-3)
    check('applied_bias_mm 原样记下', a['applied_bias_mm'] == [-2.791, -0.248],
          repr(a.get('applied_bias_mm')))
    check('清障块不补 b̂（自由区落点不是垫面）',
          'applied_bias_mm' not in acts[0])
    nb, _ = cr.actions_from_attempt(truth, att4, cm.PAD_THICK)
    check('不给 b̂ ⇒ 命令 = 期望、applied_bias_mm 为空',
          abs(nb[-1]['slot'][0] - cx) < 1e-12
          and abs(nb[-1]['slot'][1] - cy) < 1e-12
          and nb[-1]['applied_bias_mm'] is None,
          repr((nb[-1]['slot'], nb[-1]['applied_bias_mm'])))

    # 同一轮 5 个目标必须用**同一个** b̂（fixed 语义）
    blk0 = cr.find_block(truth['blocks'], 0)
    a8 = cr.build_target_action(blk0, att['grasp_hit'], 8, cm.PAD_THICK,
                                bias_mm=b)
    a0 = cr.build_target_action(blk0, att['grasp_hit'], 0, cm.PAD_THICK,
                                bias_mm=b)
    check('build_target_action：5 格同一 b̂、期望 = 各自格心',
          a8['applied_bias_mm'] == a0['applied_bias_mm'] == b and
          abs(a8['desired_center_mm'][0]
              - round(cm.cell_center(8)[0] * 1000.0, 3)) < 1e-9 and
          abs(a0['desired_center_mm'][0]
              - round(cm.cell_center(0)[0] * 1000.0, 3)) < 1e-9,
          repr((a8['applied_bias_mm'], a8['desired_center_mm'])))

    # 格位断言：探针终态必须落在实测格心 1 mm 内（否则计划按 A 垫、探针按 B 垫）
    off = json.loads(json.dumps(att4))
    off['final_blocks'][0]['x'] += 0.003
    ok, why = raises(cr.actions_from_attempt, truth, off, cm.PAD_THICK)
    check('探针终态离格心 3 mm ⇒ 拒绝（两份垫位姿不一致）', ok, why)
    dy = json.loads(json.dumps(att4))
    dy['final_blocks'][0]['yaw'] = tgt_yaw + 5.0
    ok, why = raises(cr.actions_from_attempt, truth, dy, cm.PAD_THICK)
    check('探针终态 yaw 与真值不一致 ⇒ 拒绝', ok, why)


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

    check('两个动作各出一份计划 + 收尾三个判分机位',
          len(plans) == 5 and [p['kind'] for p in plans]
          == ['clear', 'target', 'view', 'view', 'view']
          and [p['view'] for p in plans[2:]] == ['pick', 'pad', 'free'])
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
    check('塔块放下高度 ≈ 218.2 mm（29.5 mm 挂长 + 3 mm 落差，第 342 轮'
          '对齐阶段 3 口径；5 mm 时代是 220.2）',
          abs(m1['place_z'] - 0.2182) < 1e-3)
    check('gap 判据被显式放宽（记录在案）',
          p1['meta']['e9']['gap_check'] == 'probe_verified')
    # 第 342 轮（开爪推挤复盘）：接近/释放半开 50 mm + 释放静置 0.5 s——
    # 阶段 2/3 拍板值，阶段 4 此前没传 ⇒ 全开 95 mm 释放推飞邻格的黄绿块。
    w_approach, w_place = p1['waypoints'][0], p1['waypoints'][5]
    check('接近/释放半开 50 mm（build_pick 的 pick/place_open_mm）',
          w_approach.get('before') == {'gripper': 'open_to', 'mm': 50.0}
          and w_place.get('after') == {'gripper': 'open_to', 'mm': 50.0},
          repr((w_approach.get('before'), w_place.get('after'))))
    check('释放静置 0.5 s 挂在 place 航点（阶段 3 第 244 轮口径）',
          w_place.get('dwell_s') == 0.5, repr(w_place.get('dwell_s')))
    check('清障那一步按块 8 的 yaw+90° 抓（ψ=−26.653° 折叠后）',
          abs(p1['meta']['yaw_grasp_deg'] - pp.fold_yaw_180(243.35)) < 1e-3)
    check('终态布局：块 8 落到 (0.52, 0.54)、台面',
          cr.find_block(layout, '8')['x'] == 0.52
          and cr.find_block(layout, '8')['top'] == 0.0295)
    check('终态布局：目标块在垫上 top = 垫厚 + 边长',
          abs(cr.find_block(layout, '0')['top']
              - (cm.PAD_THICK + 0.0295)) < 1e-9)
    # 收尾那条：跑后判分帧与跑前巡检帧必须**逐位一致**（相机落在拾取区中心
    # 上方 500 mm），否则两次真值的块位置没有可比性。
    p3 = json.load(open(plans[2]['path'], encoding='utf-8'))
    check('第三份 q_start = 第二份终点（整链衔接）',
          p3['q_start'] == p2['waypoints'][-1]['q'])
    check('收尾第一份是 pick 机位（e9 kind/view 标明）',
          p3['meta']['e9'].get('kind') == 'view'
          and p3['meta']['e9'].get('view') == 'pick')
    cam = p3['meta']['survey_cam_base_m']
    check('巡检位相机 = 拾取区中心、台面上方 500 mm',
          abs(cam[0] - 0.4125) < 1e-6 and abs(cam[1] - 0.295) < 1e-6
          and abs(cam[2] - 0.5) < 1e-6)
    check('巡检位四边余量全 >0（拾取区整块在画幅内）',
          min(p3['meta']['coverage_px']) > 0)
    p5 = json.load(open(plans[4]['path'], encoding='utf-8'))
    check('四、五份依次衔接（pick→free→pad）',
          json.load(open(plans[3]['path'], encoding='utf-8'))['q_start']
          == p3['waypoints'][-1]['q']
          and p5['q_start']
          == json.load(open(plans[3]['path'],
                            encoding='utf-8'))['waypoints'][-1]['q'])
    check('q_end = 最后一份计划的终点（停在 pad 机位）',
          q_end == p5['waypoints'][-1]['q'])

    # 搬运那一步的抓点必须在**塔上的位置**
    ap = p1['waypoints'][0]
    check('approach 的 q 与 q_start 不同（确实动了）', ap['q'] != q0)

    # 旧计划（--no-wrist-survey）只为复现：序列回到 2 份、q_end 是第二次搬运
    out2 = tempfile.mkdtemp(prefix='e9-plan-unit-nowrist-')
    plans2, _, q2 = cr.plan_sequence(ch, q0, acts, blocks, out2,
                                     wrist_survey=False)
    last2 = json.load(open(plans2[-1]['path'], encoding='utf-8'))
    check('--no-wrist-survey ⇒ 只有 2 份、q_end 回到第二次搬运终点',
          len(plans2) == 2 and q2 == last2['waypoints'][-1]['q'])


# ------------------------------------------------------------------ [4b]
def section_multitarget():
    print('\n[4b] 整轮形状：计划名错开、机位只追加一次、令牌守卫')
    import inspect
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    out = tempfile.mkdtemp(prefix='e9-plan-multi-')
    # 编排器的形状：每目标一次 plan_sequence（start 递增，不拍机位），
    # 最后统一 plan_views(start=5)。5 个目标共用同一个 plan/ 目录。
    names, idxs = [], []
    q = q0
    start = 0
    for _k in range(2):                       # 用 2 个目标代表 5 个
        plans, _lay, q = cr.plan_sequence(ch, q, acts, blocks, out,
                                          wrist_survey=False, start=start)
        names += [p['name'] for p in plans]
        idxs += [p['index'] for p in plans]
        start += len(plans)
    vplans, _q = cr.plan_views(ch, q, cr.E9_VIEWS, out, start=start)
    names += [p['name'] for p in vplans]
    idxs += [p['index'] for p in vplans]
    check('两轮调用 + 机位：计划名全不重复（执行日志跟着不串）',
          len(names) == len(set(names)) == 7, str(names))
    check('index 连续 1..7（%=%02d 前缀与 index 同步偏移）',
          idxs == list(range(1, 8)), str(idxs))
    check('start 偏移真的写进了文件名',
          names[0].startswith('01-') and names[2].startswith('03-')
          and names[-1].startswith('07-'), str(names))
    check('wrist_survey=False ⇒ 本次调用不带机位',
          all(p['kind'] != 'view' for p in plans))
    check('机位只追加一次（三份，不互相覆盖）',
          [p['view'] for p in vplans] == ['pick', 'pad', 'free'])
    check('跑前默认机位只剩 pick / pad（正式轮：观察不解 free）',
          [v['name'] for v in cr.E9_VIEWS] == ['pick', 'pad', 'free'])

    # gap_check 判决来源随参数落盘；S0 无接触布局走 60 mm 粗判据。
    # 夹具必须**无接触**（上面的塔夹具里块 0/8 中心距 0.0 mm，60 mm 判据本就该拒）。
    s_truth, s_att = solo_truth(), solo_attempt()
    s_acts, s_blocks = cr.actions_from_attempt(s_truth, s_att, cm.PAD_THICK)
    out2 = tempfile.mkdtemp(prefix='e9-plan-gapchk-')
    plans3, _, _ = cr.plan_sequence(ch, q0, s_acts, s_blocks, out2,
                                    wrist_survey=False, min_block_gap=0.06,
                                    gap_check='layout_gap')
    m3 = json.load(open(plans3[0]['path'], encoding='utf-8'))
    check('manifest/meta 记 gap_check=layout_gap（不是一律 probe_verified）',
          m3['meta']['e9']['gap_check'] == 'layout_gap')
    check('S0 形状（无清障、60 mm 粗判据）下计划只有 1 份', len(plans3) == 1)
    ok, why = raises(cr.plan_sequence, ch, q0, acts, blocks,
                     tempfile.mkdtemp(prefix='e9-plan-gaprej-'),
                     wrist_survey=False, min_block_gap=0.06)
    check('塔夹具（块心重合）在 60 mm 判据下 ⇒ 拒绝（判据真的生效）',
          ok, why)

    # 令牌守卫：执行脚本第一件事就是查剩余时间。
    mk = [{'name': '01-target-block-0.json', 'kind': 'target'}]
    sh = cr.run_script_text(mk, '$HOME/precision/e9/x',
                            '$HOME/precision/e9/x/exec',
                            token_min_remaining=120)
    check('run.sh 含板端 --min-remaining 守卫', '--min-remaining 120' in sh)
    check('守卫在第一条执行之前',
          sh.index('--min-remaining 120') < sh.index('01-target-block-0.json'))
    check('过期时写 aborted_token（本轮 void，不算抓取失败）',
          'aborted_token' in sh)
    sh0 = cr.run_script_text(mk, '$HOME/precision/e9/x',
                             '$HOME/precision/e9/x/exec')
    check('不传 token_min_remaining ⇒ 脚本与旧版一致（没有守卫行）',
          'real_motion_authorize.py --check-token' not in sh0)

    check('彩色量默认搜索窗 25 mm（60 mm 会罩进对角邻块角）',
          inspect.signature(cr.measure_target_colour)
          .parameters['search_mm'].default == 25.0)


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
                                 'z': PAD_TOP_Z, 'yaw': 0.0, 'size': 0.0295},
                                {'id': 1, 'x': 0.44, 'y': 0.30,
                                 'z': 0.01475, 'yaw': 0.0, 'size': 0.0295}]}
    a_acts, a_blocks = cr.actions_from_attempt(adj, adj_att, cm.PAD_THICK)
    try:
        cr.plan_sequence(ch, q0, a_acts, a_blocks, out)
        adj_ok = True
    except pp.PlanError:
        adj_ok = False
    check('贴面块（中心距 40 mm）能出计划', adj_ok)
    mut2 = _mut('min_block_gap=float(min_block_gap),',
                'min_block_gap=0.06,')
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


# ------------------------------------------------------------------ [7]
def section_verify():
    print('\n[7] 跑后核验：不丢指定块 / 不掉块 / 不撞')
    final = [
        # 目标：垫心（宫格 4）
        {'id': 0, 'x': 0.4242, 'y': -0.11224, 'z': PAD_TOP_Z, 'yaw': 0.0,
         'size': 0.0295},
        # 清障块：自由区落点
        {'id': 8, 'x': 0.52, 'y': 0.54, 'z': 0.01475, 'yaw': 10.0,
         'size': 0.0295},
        # 旁观块：原地不动
        {'id': 3, 'x': 0.60, 'y': 0.30, 'z': 0.01475, 'yaw': 20.0,
         'size': 0.0295},
    ]
    mani = {'target': 0, 'cell': 4,
            'actions': [{'block': '8'}, {'block': '0'}],
            'final_layout': [dict(b) for b in final]}
    post = [dict(b) for b in final]

    r = cr.compare_layout(mani, post)
    check('完美终态 ⇒ PASS', r['ok'])
    check('三条判据都出现了（块数 / 终态 / 格 / 每块）',
          len(r['checks']) >= 5)

    b = [dict(x) for x in post]
    b[1]['x'] += 0.020                       # 清障块落偏 20 mm
    check('清障块落点偏 20 mm ⇒ PASS（40 mm 公差内）',
          cr.compare_layout(mani, b)['ok'])

    b = [dict(x) for x in post]
    b[2]['x'] += 0.020                       # 旁观块被推 20 mm
    r = cr.compare_layout(mani, b)
    check('旁观块被推 20 mm ⇒ FAIL（不撞）',
          not r['ok'] and any('没被动过' in c['check'] and not c['ok']
                              for c in r['checks']))

    # 2026-11-08 用户拍板：转角只记录、不判（深度口径对块的 yaw 重复性 4.5–36°，
    # 3° 的公差低于测量底噪）。原地转 8° ⇒ 仍 PASS，但报告里看得见这 8°。
    b = [dict(x) for x in post]
    b[2]['yaw'] += 8.0
    r = cr.compare_layout(mani, b)
    check('旁观块只被转 8°（位移 0）⇒ PASS（yaw 只记录）', r['ok'])
    check('转过的角度仍出现在报告里',
          any(abs(float(row.get('dyaw_deg') or 0) - 8.0) < 0.01
              for row in r['blocks']))

    b = [dict(x) for x in post]
    b[1]['x'], b[1]['y'] = 0.40, 0.30        # 没抓住，块还在原位
    r = cr.compare_layout(mani, b)
    check('清障块没搬走（抓空了）⇒ FAIL（不丢）',
          not r['ok'] and any('落在计划位' in c['check'] and not c['ok']
                              for c in r['checks']))

    b = [dict(x) for x in post]
    b[1]['z'] = -0.5                         # 掉到地上
    r = cr.compare_layout(mani, b)
    check('块掉到地上 ⇒ FAIL（不掉块 / 终态非法）', not r['ok']
          and not r['terminal']['ok'])

    b = [dict(x) for x in post]
    b[0]['x'] += 0.065                       # 目标没放到垫心（偏出宫格）
    r = cr.compare_layout(mani, b)
    check('目标没落在指定格 ⇒ FAIL', not r['ok'])

    b = [dict(x) for x in post][:2]          # 少一块
    r = cr.compare_layout(mani, b)
    check('少一块 ⇒ FAIL（块数一致）', not r['ok'])

    b = [dict(x) for x in post] + [{'id': 9, 'x': 0.30, 'y': 0.20,
                                    'z': 0.01475, 'yaw': 0.0,
                                    'size': 0.0295}]
    r = cr.compare_layout(mani, b)
    check('多一块 ⇒ FAIL（块数一致）', not r['ok'])

    # --- 2026-11-08 真机第一场暴露的两个 bug（都修了，这里钉住） --------
    # 真值的 id 是「同一份文件内按位置重排」的序号：跑后再拍一遍，块的 id 会和
    # 计划里的 final_layout 完全不同。按 id 配对会报出 240 mm 的假位移。
    ren = sorted([dict(x) for x in post], key=lambda x: (x['x'], x['y']))
    for i, x in enumerate(ren):
        x['id'] = i
    check('场景1 实况：跑后 id 按位置重排 ⇒ 仍 PASS（按三维位置配对）',
          cr.compare_layout(mani, ren)['ok'])
    r = cr.compare_layout(mani, ren)
    check('报告里同时给出计划 id 与跑后 id',
          all('post_id' in row for row in r['blocks']))

    # 方块 yaw 只有 mod 90 有定义：转 90° 是同一个盒子，不是「被转了 90°」
    b = [dict(x) for x in post]
    for x in b:
        x['yaw'] += 90.0
    check('旁观块 yaw 差 90° ⇒ PASS（方块按 90° 折角比）',
          cr.compare_layout(mani, b)['ok'])

    # 真机第一场的实测数：两塔 y 差 4.2/5.1 mm、yaw 差 7.6/17.2°。彩色剪影证明
    # 块没动（取向差 ≤2°、重心差 <1 px）。位移 5 mm 内 ⇒ PASS；yaw 进报告不判。
    b = [dict(x) for x in post]
    b[2]['yaw'] = 37.2
    b[2]['y'] += 0.0045
    r = cr.compare_layout(mani, b)
    check('场景1 实况 4.5 mm / 17.2° ⇒ PASS（位移在 5 mm 内）', r['ok'])

    # 但位移超 5 mm 仍然红 —— 判据放宽的是 yaw，不是位移
    b = [dict(x) for x in post]
    b[2]['y'] += 0.0055
    r = cr.compare_layout(mani, b)
    check('旁观块位移 5.5 mm ⇒ FAIL（5 mm 仍是硬判据）',
          not r['ok'] and any('没被动过' in c['check'] and not c['ok']
                              for c in r['checks']))

    # 掉块 + 重排同时发生：缺块要按「配不上」报，不能因为 id 对不上就误报位移
    few = sorted([dict(x) for x in post][:2], key=lambda x: (x['x'], x['y']))
    for i, x in enumerate(few):
        x['id'] = i
    r = cr.compare_layout(mani, few)
    check('少一块且 id 重排 ⇒ FAIL 且报「缺」',
          not r['ok'] and any('块数一致' in c['check'] and not c['ok']
                              for c in r['checks']))


def section_fuse():
    print('\n[7b] 跑后融合：腕部管内权威、俯视补区外')
    zone = (0.25, 0.575, 0.17, 0.42)
    # 拾取区：两座 2 层塔（腕部看见的）
    wrist = [
        {'id': 0, 'x': 0.4406, 'y': 0.3443, 'z': 0.01475, 'yaw': 86.0,
         'size': 0.0295},
        {'id': 1, 'x': 0.4406, 'y': 0.3443, 'z': 0.04425, 'yaw': 86.0,
         'size': 0.0295},
        {'id': 2, 'x': 0.5350, 'y': 0.1737, 'z': 0.01475, 'yaw': 90.0,
         'size': 0.0295},
        {'id': 3, 'x': 0.5350, 'y': 0.1737, 'z': 0.04425, 'yaw': 90.0,
         'size': 0.0295},
    ]
    # 俯视看见的：区外两块（垫面 / 北侧自由区）+ 区内同一批塔（含一根被数错层的）
    topcam = [
        {'id': 0, 'x': 0.415, 'y': -0.1622, 'z': 0.01475, 'yaw': 10.0,
         'size': 0.0295},                       # 垫面：区外，保留
        {'id': 1, 'x': 0.5207, 'y': 0.5021, 'z': 0.01475, 'yaw': 20.0,
         'size': 0.0295},                       # 自由区：区外，保留
        {'id': 2, 'x': 0.4406, 'y': 0.3443, 'z': 0.01475, 'yaw': 86.0,
         'size': 0.0295},                       # 区内：俯视不算数
        {'id': 3, 'x': 0.5350, 'y': 0.1737, 'z': 0.01475, 'yaw': 90.0,
         'size': 0.0295},                       # 区内：俯视把 2 层塔数成 1 块
        {'id': 4, 'x': 0.3054, 'y': 0.2812, 'z': 0.01475, 'yaw': 0.0,
         'size': 0.0295},                       # 区内的幻影：必须被挡掉
    ]
    blocks, prov = cr.fuse_post_blocks(topcam, wrist, zone=zone)
    check('融合后 = 腕部 4 + 俯视区外 2', len(blocks) == 6
          and prov['n_wrist'] == 4 and prov['n_topcam_used'] == 2)
    check('俯视在区内报的 3 块全被丢掉（含幻影）',
          len(prov['topcam_inside_zone_dropped']) == 3)
    check('两块 2 层塔在融合结果里仍是 4 块（俯视数错层不影响）',
          sum(1 for b in blocks
              if abs(b['x'] - 0.4406) < 1e-4 or abs(b['x'] - 0.5350) < 1e-4)
          == 4)
    check('垫面 / 自由区的块来自俯视',
          sorted(b['_source'] for b in blocks).count('topcam') == 2)
    check('融合后的 id 不撞车（配对靠位置，不靠 id）',
          len({str(b['id']) for b in blocks}) == 6)

    # 跨界的同一块：腕部按 zone 裁到了区边内、俯视按坐标判成区外，差 3 mm。
    # 不设去重就会多出一块 ⇒ 假「多块」。40 mm 内视为同一块。
    w2 = [{'id': 0, 'x': 0.3000, 'y': 0.4170, 'z': 0.01475, 'yaw': 0.0,
           'size': 0.0295}]
    t2 = [{'id': 0, 'x': 0.3000, 'y': 0.4230, 'z': 0.01475, 'yaw': 0.0,
           'size': 0.0295}]
    blocks, prov = cr.fuse_post_blocks(t2, w2, zone=zone)
    check('跨界同一块（差 6 mm）⇒ 只算一块', len(blocks) == 1
          and len(prov['topcam_duplicate_dropped']) == 1)

    t3 = [{'id': 0, 'x': 0.3000, 'y': 0.5000, 'z': 0.01475, 'yaw': 0.0,
           'size': 0.0295}]
    blocks, _ = cr.fuse_post_blocks(t3, w2, zone=zone)
    check('区外 83 mm 处的不同块 ⇒ 保留（去重不吞真块）', len(blocks) == 2)

    # 融合结果直接喂现有判据：块数守恒、目标落格仍按彩色量
    final = [
        {'id': 0, 'x': 0.4242, 'y': -0.11224, 'z': PAD_TOP_Z, 'yaw': 0.0,
         'size': 0.0295},
        {'id': 1, 'x': 0.52, 'y': 0.54, 'z': 0.01475, 'yaw': 86.0,
         'size': 0.0295},
        {'id': 2, 'x': 0.5350, 'y': 0.1737, 'z': 0.01475, 'yaw': 90.0,
         'size': 0.0295},
        {'id': 3, 'x': 0.5350, 'y': 0.1737, 'z': 0.04425, 'yaw': 90.0,
         'size': 0.0295},
        {'id': 4, 'x': 0.4406, 'y': 0.3443, 'z': 0.01475, 'yaw': 86.0,
         'size': 0.0295},
        {'id': 5, 'x': 0.4406, 'y': 0.3443, 'z': 0.04425, 'yaw': 86.0,
         'size': 0.0295},
    ]
    mani = {'target': 0, 'cell': 4, 'actions': [{'block': 1}, {'block': 0}],
            'final_layout': [dict(b) for b in final]}
    # 跑后实况：区内 4 块来自腕部；垫面 / 自由区 2 块来自俯视
    fused, _ = cr.fuse_post_blocks(
        [{'id': 0, 'x': 0.4242, 'y': -0.11224, 'z': PAD_TOP_Z, 'yaw': 0.0,
          'size': 0.0295},
         {'id': 1, 'x': 0.52, 'y': 0.54, 'z': 0.01475, 'yaw': 86.0,
          'size': 0.0295}],
        [dict(b) for b in wrist], zone=zone)
    r = cr.compare_layout(mani, fused)
    check('融合结果喂 compare_layout ⇒ PASS', r['ok'])

    # 清障块不见了（自由区那格空了）
    fused2 = [dict(b) for b in fused
              if not (abs(b['x'] - 0.52) < 1e-6 and abs(b['y'] - 0.54) < 1e-6)]
    r = cr.compare_layout(mani, fused2)
    check('融合后少一块 ⇒ FAIL（块数一致）', not r['ok']
          and any('块数一致' in c['check'] and not c['ok']
                  for c in r['checks']))


def section_step_cap():
    print('\n[4b] 单关节跳变上限：同一末端位姿换分支不许「先原地转半圈」')
    rad = math.radians

    def q_at(deg0, deg_other=0.0):
        q = [rad(deg_other)] * 6
        q[0] = rad(deg0)
        return q

    ok_guard = lambda qa, qb: (True, 0.0)                 # noqa: E731
    real_ik = pp.ik_candidates
    real_cap = pp.MAX_JOINT_STEP_DEG
    try:
        # 两航点、各只有一支解，第二支要 joint0 转 200°（s9 的 309°/201° 同形）
        sets = {'w1': [q_at(5.0)], 'w2': [q_at(205.0)]}

        def fake_ik(chain, xyz, yaw_deg, seeds=None, n_random=0):
            return [np.asarray(q, float) for q in sets[xyz[2]]]

        pp.ik_candidates = fake_ik
        tgt = [('w1', (0.0, 0.0, 'w1')), ('w2', (0.0, 0.0, 'w2'))]

        ok, err = raises(pp.solve_path, None, tgt, [0.0] * 6, guard=ok_guard)
        check('单步 200° 超 170° 上限 ⇒ PlanError、不返回计划',
              ok and '单步' in err, err)
        check('报错点明是哪一串航点', ok and 'w2' in err, err)

        path = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard,
                             max_step_deg=float('inf'))
        check('放开上限后同一组解能出链（拒绝确实来自上限）', len(path) == 2)

        # 上限是运行时读模块常量，不是 def 时绑定的默认参数（这条是回归点）
        pp.MAX_JOINT_STEP_DEG = 210.0
        path = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard)
        check('改模块常量当场生效（不是 def 时绑死的默认值）', len(path) == 2)
        pp.MAX_JOINT_STEP_DEG = 170.0

        # 不误杀：单步全在限内的链照常放行
        sets = {'w1': [q_at(80.0)], 'w2': [q_at(180.0)]}
        path = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard)
        check('单步 80°/100° 全在限内 ⇒ 放行（不误杀）', len(path) == 2)

        # 真语义：宁可多走一点，也不许有一跳 200°。
        # 候选 B 总行程 200°（但单关节 200°，超限）；候选 C 总行程 √5·90° ≈ 201.2°
        # （每个关节 90°，都合规）⇒ 有上限时必须选 C。
        sets_w2 = [q_at(200.0), q_at(0.0, deg_other=90.0)]
        sets = {'w1': [q_at(0.0)], 'w2': sets_w2}
        free = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard,
                             max_step_deg=float('inf'))
        capped = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard)
        check('不设上限 ⇒ 挑总行程最小的坏分支（200° 单步）',
              abs(math.degrees(free[-1][0]) - 200.0) < 1e-6,
              str(np.degrees(free[-1])))
        check('有上限 ⇒ 改挑合规分支（多走 1° 也认）',
              abs(math.degrees(capped[-1][0])) < 1e-6
              and abs(math.degrees(capped[-1][1]) - 90.0) < 1e-6,
              str(np.degrees(capped[-1])))
    finally:
        pp.ik_candidates = real_ik
        pp.MAX_JOINT_STEP_DEG = real_cap


def section_seed_propagation():
    print('\n[4e] IK 沿链传种子：独立播种漏解 ⇒ 用上一航点候选当种子补解')
    rad = math.radians

    def q_at(deg0):
        q = [0.0] * 6
        q[0] = rad(deg0)
        return np.asarray(q, float)

    ok_guard = lambda qa, qb: (True, 0.0)                 # noqa: E731
    tgt = [('w1', (0.0, 0.0, 'w1')), ('w2', (0.0, 0.0, 'w2'))]
    # 链起点 = 全 0；w1 只有 60° 一支；w2 的近支是 80°（距 w1 20°），
    # 独立播种只采到 240° 的那支（距 w1 180°，超 170° 上限）。
    a, near, far = q_at(60.0), q_at(80.0), q_at(240.0)
    real_ik = pp.ik_candidates

    def factory(mode):
        calls = []

        def fake(chain, xyz, yaw_deg=0.0, seeds=None, n_random=0):
            sl = [np.asarray(s, float) for s in (seeds or [])]
            calls.append((xyz[2], sl))
            if xyz[2] == 'w1':
                return [a]
            has_a = any(np.allclose(s, a) for s in sl)
            if mode == 'ok' or (mode == 'defect' and has_a):
                return [near]
            return [far]

        return fake, calls

    try:
        fake, calls = factory('defect')
        pp.ik_candidates = fake
        path = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard)
        check('独立播种只采到 180° 时，沿链传种子找到 80°（距 w1 仅 20°）近支',
              np.allclose(path[-1], near), str(np.degrees(path[-1])))
        w2 = [s for n, s in calls if n == 'w2']
        check('第二个航点解了两遍，第二遍的种子里带着上一航点的候选',
              len(w2) == 2 and any(np.allclose(s, a) for s in w2[1]),
              [len(s) for s in w2])

        fake, calls = factory('ok')
        pp.ik_candidates = fake
        path = pp.solve_path(None, tgt, [0.0] * 6, guard=ok_guard)
        check('第一遍能过 ⇒ 每个航点只解一遍（不白跑传播）',
              len(calls) == 2, len(calls))

        fake, calls = factory('hopeless')
        pp.ik_candidates = fake
        ok, err = raises(pp.solve_path, None, tgt, [0.0] * 6, guard=ok_guard)
        check('两遍都过不了 ⇒ 仍按分支链拒绝（不静默放行）',
              ok and '单步' in err, err)
    finally:
        pp.ik_candidates = real_ik


def section_view_chain():
    print('\n[4f] 末块收尾支 → 三机位：整串一个 DP，不许逐点重新 IK')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    out = tempfile.mkdtemp(prefix='e9-plan-viewchain-')
    tgt = cr.view_targets()
    check('机位目标 = 每机位两点（transit + pose），次序同 E9_VIEWS',
          [t[0] for t in tgt] == ['pick_transit', 'pick_pose',
                                  'pad_transit', 'pad_pose',
                                  'free_transit', 'free_pose'], str(
                                      [t[0] for t in tgt]))

    # ① 末块的收尾支必须由 build_pick 一并解出来（post_q），且真的到得了那些点。
    q = q0
    last = None
    for i in range(2):
        plans, _lay, q = cr.plan_sequence(
            ch, q, acts, blocks, out, wrist_survey=False, start=i,
            tail_targets=(tgt if i == 1 else None))
        last = json.load(open(plans[-1]['path'], encoding='utf-8'))
    pq = last['meta'].get('post_q')
    check('末块 meta 带 post_q（选支用的后续目标关节角）',
          isinstance(pq, list) and len(pq) == len(tgt), 'len=%r' % (
              None if pq is None else len(pq)))
    if isinstance(pq, list) and len(pq) == len(tgt):
        worst_p, worst_r = 0.0, 0.0
        for (nm, xyz, yaw), qq in zip(tgt, pq):
            pe, re = pp.pose_residual(ch, qq, xyz, yaw)
            worst_p, worst_r = max(worst_p, pe), max(worst_r, re)
        check('post_q 逐点到位（残差 ≤0.1 mm / 0.05°）',
              worst_p <= 0.1 and worst_r <= 0.05,
              '%.3f mm / %.4f°' % (worst_p, worst_r))
        seq = [last['waypoints'][-1]['q']] + pq
        step = max(pp.max_joint_step_deg(a, b)
                   for a, b in zip(seq, seq[1:]))
        check('末航点 → … → 最后一机位：相邻单步都 ≤170°', step <= 170.0,
              '%.1f°' % step)

    # ② plan_views 必须**一次** DP 解完整串，并把解好的关节角交给每个机位
    #    （旧实现是逐机位局部 IK —— 就是它把末块收尾支重新挑到了另一支）。
    import stage4_wrist_survey as ws
    q_end = last['waypoints'][-1]['q']
    real_solve = pp.solve_path
    calls = []

    def spy(*a, **kw):
        calls.append(len(a[1]))
        return real_solve(*a, **kw)

    real_sv = ws.survey_plan
    given = []

    def spy_sv(chain_, q_, **kw):
        given.append(kw.get('qs'))
        return real_sv(chain_, q_, **kw)

    pp.solve_path = spy
    ws.survey_plan = spy_sv
    try:
        vplans, qv = cr.plan_views(ch, q_end, cr.E9_VIEWS, out, start=2)
    finally:
        pp.solve_path = real_solve
        ws.survey_plan = real_sv
    check('三份机位计划只调一次 solve_path（整串 6 点一起解）',
          calls == [2 * len(cr.E9_VIEWS)], str(calls))
    check('每个机位都收到 DP 解好的两点关节角（qs 不为空）',
          all(isinstance(s, list) and len(s) == 2 for s in given), str(
              [(None if s is None else len(s)) for s in given]))

    files = [json.load(open(p['path'], encoding='utf-8')) for p in vplans]
    chain_q = [q_end] + [w['q'] for f in files for w in f['waypoints']]
    step = max(pp.max_joint_step_deg(a, b) for a, b in zip(chain_q, chain_q[1:]))
    check('末航点 → 三机位 12 个航点：相邻单步都 ≤170°', step <= 170.0,
          '%.1f°' % step)
    check('每份机位计划都是兩点（transit + pose）',
          [len(f['waypoints']) for f in files] == [2, 2, 2])
    used = all([w['q'] for w in f['waypoints']]
               == [[float(x) for x in s[0]], [float(x) for x in s[1]]]
               for f, s in zip(files, given))
    check('写进计划的 q 就是 DP 给的那两点（没有悄悄重新 IK）', used)

    # ③ 给了 qs 就必须照着用：换一组别的关节角进来，预检要当场拒。
    import stage4_wrist_survey as ws
    v = cr.E9_VIEWS[0]
    ok, why = raises(ws.survey_plan, ch, q_end, tuple(v['zone']),
                     height=float(v['height']), yaw_deg=0.0,
                     transit_dz=cr.VIEW_TRANSIT_DZ,
                     qs=[[0.0] * 6, [0.0] * 6])
    check('survey_plan(qs=...) 真按给的关节角走（错的当场拒）',
          ok and '残差' in why, why)


def section_place_yaw_equiv():
    print('\n[4c] 放置朝向 180° 等价兜底：只对 BranchChainError 生效')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    real = pp.build_pick
    calls = []

    def fake(*a, **kw):
        calls.append((round(float(kw.get('yaw_grasp_deg')), 4),
                      kw.get('yaw_place_deg'),
                      bool(kw.get('fold_grasp_yaw', True))))
        if kw.get('yaw_place_deg') is None:
            raise pp.BranchChainError(
                '这些航点的所有 IK 分支组合都过不了高度护栏或单步 170° 上限'
                '（测试注入）')
        # 接线验证：等价写法换回默认朝向喂给真规划器（这个夹具换 180° 物理上
        # 过不了 IK，测的是「试哪个写法、怎么记录」，不是几何）
        kw = dict(kw, yaw_place_deg=None)
        return real(*a, **kw)

    # 抓取朝向 ±180° 等价重试的两支（2026-11-14 修后语义）：
    # ① 折叠写法（fold_yaw_180(raw)，fold_grasp_yaw=True）；
    # ② 整体 +180°（fold_yaw_180(raw) + 180，fold_grasp_yaw=False —— 不再
    #    折叠，目标矩阵才真的不同）。修前两支都过 fold ⇒ 塌缩成同一目标，
    #    重试是空转（审计缺陷 3）。
    g_raw = [float(a['grasp_yaw_deg']) for a in acts]
    g_folded = [pp.fold_yaw_180(g) for g in g_raw]
    g_alt = [g + 180.0 for g in g_folded]
    try:
        pp.build_pick = fake
        out = tempfile.mkdtemp(prefix='e9-equiv-off-')
        ok, err = raises(cr.plan_sequence, ch, q0, acts, blocks, out)
        check('缺省不开旗子 ⇒ BranchChainError 原样上抛', ok and '测试注入' in err)
        check('缺省不开旗子 ⇒ 抓取朝向两支各试一次（折叠 / +180 不折叠）',
              calls == [(round(g_folded[0], 4), None, True),
                        (round(g_alt[0], 4), None, False)],
              calls)
        check('缺省不开旗子 ⇒ 两支的 yaw 真的不同（修前塌缩成同一个）',
              abs(calls[0][0] - calls[1][0]) > 90.0, calls)
        check('缺省不开旗子 ⇒ 一份计划都不写',
              not any(f.endswith('.json') for f in os.listdir(out)), os.listdir(out))

        calls.clear()
        pp.build_pick = fake
        out2 = tempfile.mkdtemp(prefix='e9-equiv-on-')
        plans, layout, q_end = cr.plan_sequence(ch, q0, acts, blocks, out2,
                                                wrist_survey=False,
                                                place_yaw_equiv=True)
        want = [pp.fold_yaw_180(g) - 180.0 for g in g_raw]
        expected = []
        for i in range(len(acts)):
            expected += [(round(g_folded[i], 4), None, True),
                         (round(g_folded[i], 4), want[i], True)]
        check('开了旗子 ⇒ 每个动作先默认抓取朝向、place 失败再试 180° 等价写法',
              calls == expected, calls)
        check('180° 等价写法 = 折叠后的抓取朝向 −180°',
              all(abs(c[1] - w) < 1e-9 for c, w in zip(calls[1::2], want)), calls)
        check('等价写法落在计划 meta 里（可追溯）',
              all(p['place_yaw_equiv180'] for p in plans)
              and all(json.load(open(p['path']))['meta']['yaw_source']
                      == 'place-equiv180' for p in plans))
        check('换写法不影响航点序列与终态布局',
              all(p['n_waypoints'] == 8 for p in plans)
              and cr.find_block(layout, '0') is not None)

        # 换写法也救不回来 ⇒ fail-closed，两份都试完才报错
        calls.clear()

        def both_fail(*a, **kw):
            calls.append((kw.get('yaw_grasp_deg'), kw.get('yaw_place_deg')))
            raise pp.BranchChainError('这些航点的所有 IK 分支组合都被拒（测试注入）')

        pp.build_pick = both_fail
        out3 = tempfile.mkdtemp(prefix='e9-equiv-dead-')
        ok, err = raises(cr.plan_sequence, ch, q0, acts, blocks, out3,
                         place_yaw_equiv=True)
        check('两种写法都过不了 ⇒ 仍抛 BranchChainError（fail-closed）',
              ok and '测试注入' in err, err)
        check('两种写法都过不了 ⇒ 不写计划',
              not any(f.endswith('.json') for f in os.listdir(out3)),
              os.listdir(out3))

        # 别的 PlanError（邻块太近/夹持深度…）换朝向救不回来，不许重试
        calls.clear()

        def plain_fail(*a, **kw):
            calls.append((kw.get('yaw_grasp_deg'), kw.get('yaw_place_deg')))
            raise pp.PlanError('块 #0 离最近邻块只有 30.0 mm（测试注入）')

        pp.build_pick = plain_fail
        out4 = tempfile.mkdtemp(prefix='e9-equiv-plain-')
        ok, err = raises(cr.plan_sequence, ch, q0, acts, blocks, out4,
                         place_yaw_equiv=True)
        ok2 = (len(calls) == 1 and calls[0][1] is None
               and abs(calls[0][0] - g_folded[0]) < 1e-6)
        check('非 BranchChainError ⇒ 不重试（只试了默认写法）',
              ok and ok2 and '测试注入' in err, '%r / %r' % (calls, err))
    finally:
        pp.build_pick = real


def section_from_xy():
    print('\n[4d] 抓点只认动作表的 from_xy（实测 / 对中修正后的抓取点）')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    tgt = next(a for a in acts if a['kind'] == 'target')
    blk = cr.find_block(blocks, tgt['block'])
    nominal = [float(blk['x']), float(blk['y'])]
    # 布局故意停在标称，from_xy 改成「实测」：抓点必须跟着 from_xy 走。
    # （回归 2026-11-12 S0:1 r1 attempt5：计划 meta 的 from_xy 是实测，IK 却按
    # 布局标称坐标算，差 25 mm ⇒ 空夹回读 0.0 mm。）
    tgt['from_xy'] = [nominal[0] + 0.015, nominal[1] - 0.010]
    out = tempfile.mkdtemp(prefix='e9-fromxy-')
    plans, layout, q_end = cr.plan_sequence(ch, q0, acts, blocks, out,
                                            wrist_survey=False)
    m = json.load(open(plans[-1]['path'], encoding='utf-8'))['meta']
    check('IK 抓点 = act.from_xy（不是布局标称坐标）',
          abs(m['grasp_point'][0] - tgt['from_xy'][0]) < 1e-12
          and abs(m['grasp_point'][1] - tgt['from_xy'][1]) < 1e-12,
          repr((m.get('grasp_point'), tgt['from_xy'], nominal)))
    check('计划照样记下 from_xy（可追溯）',
          m['e9']['from_xy'] == tgt['from_xy'], repr(m['e9'].get('from_xy')))


def section_merge_truth():
    print('\n[7c] 多机位跑后真值合并：去重必须带层')
    two = [{'x': 0.4406, 'y': 0.3443, 'top': 0.04425, 'size': 0.0295},
           {'x': 0.4406, 'y': 0.3443, 'top': 0.01475, 'size': 0.0295}]
    blocks, prov = cr.merge_truth_blocks([('pick', two)])
    check('同一机位两层塔：XY 完全重合也保留 2 块（只按 XY 去重的回归）',
          len(blocks) == 2 and not prov['duplicates'], str(blocks))

    # s11 自由区现场（2026-11-11 重放）：两块**同层不同色**，中心距 37.0 mm
    # < 去重半径 40 mm。同机位内部不是重复 —— 相机一次共识已经聚过类，
    # 一块真缝 6.5 mm 的两块被去重吃掉 ⇒ 判「缺块」（块数一致红）。
    free2 = [{'x': 0.4785, 'y': 0.5360, 'top': 0.0295, 'size': 0.0295},
             {'x': 0.5154, 'y': 0.5328, 'top': 0.0295, 'size': 0.0295}]
    blocks, prov = cr.merge_truth_blocks([('free', free2)])
    check('同一机位两块相距 37 mm ⇒ 保留 2 块、不算重复',
          len(blocks) == 2 and not prov['duplicates'],
          [len(blocks), prov['duplicates']])

    towers = [{'x': x, 'y': y, 'top': t, 'size': 0.0295}
              for (x, y) in ((0.4406, 0.3443), (0.5350, 0.1737),
                             (0.4100, 0.2500))
              for t in (0.01475, 0.04425)]
    blocks, _ = cr.merge_truth_blocks([('pick', towers)])
    check('三根两层塔 = 6 块（2026-11-09 假「缺块」的回归）',
          len(blocks) == 6, str(len(blocks)))

    # 区界上同一块被两个机位各报一次：同层、差 5 mm ⇒ 合一块
    a = [{'x': 0.3000, 'y': 0.4600, 'top': 0.01475, 'size': 0.0295}]
    b = [{'x': 0.3000, 'y': 0.4650, 'top': 0.01480, 'size': 0.0295}]
    blocks, prov = cr.merge_truth_blocks([('pick', a), ('free', b)])
    check('两机位各报一次（同层、差 5 mm）⇒ 合一块',
          len(blocks) == 1 and len(prov['duplicates']) == 1, str(prov))
    check('duplicates 记了层高（top_mm），可追溯',
          prov['duplicates'][0]['top_mm'] == 14.8
          and prov['duplicates'][0]['kept_view'] == 'pick',
          str(prov['duplicates']))
    check('每块带 _source、id 唯一', blocks[0]['_source'] == 'pick'
          and len({str(x['id']) for x in blocks}) == 1)

    # 同 XY 不同层：塔的上下两块，不许当重复吃掉
    blocks, prov = cr.merge_truth_blocks(
        [('pick', [{'x': 0.30, 'y': 0.46, 'top': 0.01475}]),
         ('free', [{'x': 0.30, 'y': 0.46, 'top': 0.04425}])])
    check('同 XY 不同层 ⇒ 2 块、不算重复',
          len(blocks) == 2 and not prov['duplicates'], str(prov))

    # 高度缺失 ⇒ 判不了层，宁可不合（不合是「多一块」，会是显式红，不是静默丢块）
    blocks, _ = cr.merge_truth_blocks(
        [('pick', [{'x': 0.30, 'y': 0.46}]),
         ('free', [{'x': 0.30, 'y': 0.46}])])
    check('缺 top ⇒ 判不了层，宁可不合', len(blocks) == 2)


def _img(w, h, rgb=(255, 255, 255)):
    a = np.zeros((h, w, 3), np.uint8)
    a[:, :] = rgb
    return a


def _box(a, r0, r1, c0, c1, rgb):
    a[r0:r1, c0:c1] = rgb


def _colour(x_mm, y_mm, n=10, ok=True):
    return {'ok': ok, 'xy_mm': [x_mm, y_mm], 'n_frames': n,
            'n_missed': 10 - n, 'range_mm': [3.0, 4.0], 'max_dev_mm': 4.0,
            'hue_deg': 35.0, 'search_mm': 60.0}


def section_colour():
    print('\n[8] 目标块彩色量具（垫面深度分辨不出 29.5 mm 台阶）')
    h, s, v = cr.hsv_of(np.array([[[255, 0, 0]]], np.uint8))
    check('红：h≈0、s=255、v=255', abs(h[0, 0] % 360.0) < 1e-3
          and abs(s[0, 0] - 255) < 0.5 and abs(v[0, 0] - 255) < 0.5)
    h, s, v = cr.hsv_of(np.array([[[255, 255, 255]]], np.uint8))
    check('白垫：s=0（被饱和度门限排除）', s[0, 0] == 0.0)
    h, s, v = cr.hsv_of(np.array([[[0, 0, 0]]], np.uint8))
    check('黑标记：v=0（被亮度门限排除）', v[0, 0] == 0.0)

    tan = (210, 160, 90)
    a = _img(80, 80)
    _box(a, 30, 50, 30, 50, tan)
    b = cr.colour_blob_uv(a, (40, 40), 35)
    check('白底 + 20×20 色块 ⇒ 质心在块心',
          b is not None and abs(b['uv'][0] - 39.5) < 0.6
          and abs(b['uv'][1] - 39.5) < 0.6)
    check('质地相 ≈ 35°', b is not None and abs(b['hue_deg'] - 35.0) < 8.0)

    a = _img(80, 80)
    _box(a, 30, 50, 30, 50, (0, 0, 0))
    check('只有黑块 ⇒ None（不靠亮度门限硬凑）',
          cr.colour_blob_uv(a, (40, 40), 35) is None)
    check('全白窗口 ⇒ None（不靠饱和度门限硬凑）',
          cr.colour_blob_uv(_img(80, 80), (40, 40), 35) is None)

    # 侧面比顶面暗：取最亮 40% ⇒ 质心落在顶面，不被侧面拖向画面下方
    a = _img(80, 80)
    _box(a, 30, 40, 30, 50, (230, 180, 100))
    _box(a, 40, 50, 30, 50, (170, 120, 60))
    b = cr.colour_blob_uv(a, (40, 40), 35)
    check('明暗两半 ⇒ 质心落在亮的顶面（v < 块心 39.5）',
          b is not None and b['uv'][1] < 36.0 and abs(b['uv'][0] - 39.5) < 0.6)

    # 窗口里两种颜色：按像素数取主色簇，不把两色混成一个假色
    a = _img(80, 80)
    _box(a, 25, 55, 25, 55, (150, 50, 200))
    _box(a, 5, 15, 5, 15, tan)
    b = cr.colour_blob_uv(a, (40, 40), 40)
    check('两种颜色 ⇒ 取占多数的那一簇（紫 277°）',
          b is not None and abs(b['hue_deg'] - 277.5) < 8.0
          and abs(b['uv'][0] - 39.5) < 1.0)

    # 几何：_base_to_uv 与 ray_plane_xy 必须互为逆（同一投影点，不另写一份）
    K = np.array([[1376.4, 0.0, 974.3], [0.0, 1374.6, 517.4], [0.0, 0.0, 1.0]])
    R = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    t = np.array([0.0, 0.0, 1.5])
    p = (0.4242, -0.11224, 0.03238)
    uv = cr._base_to_uv(p, K, R, t)
    back = cr.tf.ray_plane_xy(uv, K, R, t, p[2])
    check('base→uv→base 往返 < 0.01 mm',
          back is not None
          and math.hypot(back[0] - p[0], back[1] - p[1]) < 1e-5)
    h60 = cr._window_half_px((0.4242, -0.11224), 0.03238, 60.0, K, R, t)
    h30 = cr._window_half_px((0.4242, -0.11224), 0.03238, 30.0, K, R, t)
    # 相机距垫面 ≈1.47 m、fx≈1376 ⇒ 30 mm 半格投影 ≈ 28 px（不是 16 px）
    check('30 mm 半窗 ≈28 px、60 mm 窗约是其两倍',
          26.0 < h30 < 30.0 and h60 > 1.9 * h30)

    with tempfile.TemporaryDirectory() as d:
        r = cr.measure_target_colour(d, (0.4242, -0.11224), 0.03238)
    check('空 burst ⇒ ok=False（fail-closed，不拿计划位当量值）',
          r['ok'] is False and not r['per_frame'])

    # ---- 接进 compare_layout -------------------------------------------
    final = [{'id': 0, 'x': 0.4242, 'y': -0.11224, 'z': PAD_TOP_Z, 'yaw': 0.0,
              'size': 0.0295},
             {'id': 8, 'x': 0.52, 'y': 0.54, 'z': 0.01475, 'yaw': 10.0,
              'size': 0.0295},
             {'id': 3, 'x': 0.60, 'y': 0.30, 'z': 0.01475, 'yaw': 20.0,
              'size': 0.0295}]
    mani = {'target': 0, 'cell': 4,
            'actions': [{'block': '8'}, {'block': '0'}],
            'final_layout': [dict(b) for b in final]}
    post = [dict(b) for b in final]

    r = cr.compare_layout(mani, post, target_colour=_colour(424.2, -112.24))
    check('彩色量在格心 ⇒ PASS', r['ok'])
    check('判据名写明用彩色量',
          any('彩色' in c['check'] for c in r['checks']))

    # 场景 2 实况：深度把垫上那块并进幻影、质心拉偏到 (433.7, −145.3)，且
    # z 读成台面高（0.01475）⇒ 深度判「没落格」；彩色 (419.7, −121.1) 距格心
    # 9.9 mm ⇒ PASS。数值取自 runs/2026-11-08-stage4-entry/e9/s2/ 实测。
    post2 = [dict(b) for b in final]
    post2[0]['x'], post2[0]['y'], post2[0]['z'] = 0.4337, -0.1453, 0.01475
    check('场景2 实况：只用深度 ⇒ 目标块判「没落格」（这是要修掉的假红）',
          not cr.compare_layout(mani, post2)['ok'])
    r = cr.compare_layout(mani, post2, target_colour=_colour(419.7, -121.1))
    check('场景2 实况：改彩色量 ⇒ PASS', r['ok'])
    check('深度给出的位置仍写进 notes（看得见分歧，不藏）',
          any('深度真值' in n for n in r.get('notes') or []))
    d4 = [c['detail'] for c in r['checks'] if '彩色' in c['check']][0]
    check('报告写清距格心 9.9 mm、帧数、极差、色相',
          '9.9 mm' in d4 and '10 帧' in d4 and '色相' in d4)

    r = cr.compare_layout(mani, post2, target_colour=_colour(460.0, -112.24))
    check('彩色量偏出宫格 36 mm ⇒ FAIL（判据仍是 30 mm 半格）', not r['ok'])
    r = cr.compare_layout(mani, post2, target_colour=_colour(424.2, -112.24,
                                                             n=3, ok=False))
    check('彩色量只看见 3 帧 ⇒ FAIL（不够帧不判过）',
          not r['ok'] and any('不够帧' in c['detail'] for c in r['checks']))

    # 变异检查：把饱和度门限拆掉，全白窗口那条用例必须红 —— 否则那条用例
    # 通过的原因不是它自己以为的原因。
    ns = _mut('COLOUR_S_MIN, COLOUR_V_MIN = 90.0, 60.0',
              'COLOUR_S_MIN, COLOUR_V_MIN = 0.0, 60.0')
    check('变异：拆掉饱和度门限 ⇒ 全白窗口不再返回 None（用例是承重的）',
          ns['colour_blob_uv'](_img(80, 80), (40, 40), 35) is not None)


# ----------------------------------------------------------------- [10]
class _Args(object):
    def __init__(self, **kw):
        self.grid = kw.get('grid')
        self.allow_model_pad = kw.get('allow_model_pad', False)


def section_pad_pose():
    print('\n[10] 实测垫位姿接线：--grid > manifest 记录 > 目录约定文件；都没有就停')
    tmp = tempfile.mkdtemp(prefix='cr-padpose-')
    real = {'kind': 'stage2-grid', 'grid_center_base_mm': [442.3, -197.9],
            'grid_yaw_base_deg': -91.07,
            'checks': {'n_frames': 3, 'rms_adopted_px': 0.84}}
    gpath = os.path.join(tmp, 'grid-pre.json')
    with open(gpath, 'w', encoding='utf-8') as f:
        json.dump(real, f)
    try:
        p = cr.apply_pad_pose(_Args(), tmp)
        check('只有目录约定文件 ⇒ 用它', p == gpath, str(p))
        check('装进去的是实测值（不是模型常量）',
              abs(cm.PAD_CENTER[0] * 1000 - 442.3) < 1e-9 and
              abs(math.degrees(cm.PAD_YAW_RAD) + 91.07) < 1e-9,
              str(cm.PAD_CENTER))

        # --grid 优先于目录里的文件
        alt = os.path.join(tmp, 'alt.json')
        with open(alt, 'w', encoding='utf-8') as f:
            json.dump({'kind': 'stage2-grid', 'grid_center_base_mm': [400.0, -150.0],
                       'grid_yaw_base_deg': -180.0}, f)
        p = cr.apply_pad_pose(_Args(grid=alt), tmp)
        check('--grid 优先于目录文件',
              p == alt and abs(cm.PAD_CENTER[0] * 1000 - 400.0) < 1e-9, str(p))

        # manifest 记录（相对路径按 hint_dir 解析）
        cr.apply_pad_pose(_Args(), tmp, recorded='grid-pre.json')
        check('manifest 记录的相对路径解析到 hint_dir',
              abs(cm.PAD_CENTER[0] * 1000 - 442.3) < 1e-9, str(cm.PAD_CENTER))

        # manifest 记录的路径不存在 ⇒ 退到目录约定文件，而不是崩
        cr.apply_pad_pose(_Args(), tmp, recorded='gone.json')
        check('记录的路径不在了 ⇒ 退到目录约定文件',
              abs(cm.PAD_CENTER[0] * 1000 - 442.3) < 1e-9)

        # 一个都没有 ⇒ SystemExit（fail-closed）
        empty = os.path.join(tmp, 'empty')
        ok, err = system_exits(cr.apply_pad_pose, _Args(), empty)
        check('一处都找不到 ⇒ 报错停下（fail-closed）', ok and 'stage4_pad_from_burst' in err)
        ok, _ = system_exits(cr.apply_pad_pose, _Args(), empty, 'missing.json')
        check('manifest 记录的路径也不在、目录里也没有 ⇒ 停', ok)

        # 显式 --allow-model-pad 才退回常量
        p = cr.apply_pad_pose(_Args(allow_model_pad=True), empty)
        check('--allow-model-pad ⇒ 退回模型常量且返回 None', p is None)

        # manifest 直接记下**实测值本身**（dict，不是路径）。计划目录里通常
        # 没有那份 grid.json（2026-11-09 实测：TypeError: expected str, bytes
        # or os.PathLike object, not dict），就地装上即可。
        cm.reset_pad()
        p = cr.apply_pad_pose(
            _Args(), empty,
            recorded={'source': 'grid-pre.json', 'measured': True,
                      'center_mm': [461.3, -158.2], 'yaw_deg': -89.41})
        check('manifest 记的是实测值 ⇒ 就地装上（不必去找文件）',
              p is None and abs(cm.PAD_CENTER[0] * 1000 - 461.3) < 1e-9
              and abs(math.degrees(cm.PAD_YAW_RAD) + 89.41) < 1e-9,
              str(cm.PAD_CENTER))
        ok, err = system_exits(
            cr.apply_pad_pose, _Args(), empty,
            {'source': 'model', 'measured': False})
        check('manifest 记的是模型常量 ⇒ 不装，仍 fail-closed 停下',
              ok and 'stage4_pad_from_burst' in err, err)
        ok, _ = system_exits(
            cr.apply_pad_pose, _Args(), empty,
            {'measured': True, 'center_mm': [461.3, -158.2]})   # 缺 yaw
        check('manifest 字段不全 ⇒ 不猜，仍 fail-closed 停下', ok)

        # manifest 记账（此时垫是上面 manifest 实测装上的；C1 后未实测会抛，
        # 这本身就是「记账只认实测」的守卫）
        rec = cr.pad_pose_record(gpath)
        check('pad_pose_record 标 measured、不记绝对路径',
              rec['measured'] is True and rec['source'] == 'grid-pre.json'
              and 'center_mm' in rec and 'yaw_deg' in rec, str(rec))
        rec0 = cr.pad_pose_record(None)
        check('没用实测值 ⇒ measured=False（不装成实测）',
              rec0['measured'] is False and rec0['source'] == cm.PAD_POSE_SOURCE)

        # 计划端与判分端同源核对：两份垫位姿差得明显要喊出来
        cr.apply_pad_pose(_Args(grid=gpath), tmp)     # 判分端 = 442.3/−91.07
        lines = []
        real_log = cr.log
        cr.log = lambda *a: lines.append(' '.join(str(x) for x in a))
        try:
            same = os.path.join(tmp, 'truth-same.json')
            with open(same, 'w', encoding='utf-8') as f:
                json.dump({'pad_pose': {'center_mm': [442.3, -197.9],
                                        'yaw_deg': -91.07}}, f)
            cr.warn_pad_pose_mismatch({'truth': same})
            check('计划/判分同一份 ⇒ 不喊', not lines, str(lines))
            diff = os.path.join(tmp, 'truth-diff.json')
            with open(diff, 'w', encoding='utf-8') as f:
                json.dump({'pad_pose': {'center_mm': [424.2, -112.24],
                                        'yaw_deg': -179.778}}, f)
            cr.warn_pad_pose_mismatch({'truth': diff})
            check('计划/判分不是同一份 ⇒ 喊出来',
                  any('不是同一份' in ln for ln in lines), str(lines))
            lines.clear()
            legacy = os.path.join(tmp, 'truth-old.json')
            with open(legacy, 'w', encoding='utf-8') as f:
                json.dump({'blocks': []}, f)
            cr.warn_pad_pose_mismatch({'truth': legacy})
            check('旧场景没记垫位姿 ⇒ 明说无法核对',
                  any('无法核对' in ln for ln in lines), str(lines))
        finally:
            cr.log = real_log
    finally:
        # C1：后续段落（align/view_gate）是模型世界 ⇒ 装回模型常量；
        # 「未实测 ⇒ fail-closed」的用例在本节内部已经覆盖。
        cm.install_model_pad_defaults()
        cm._GS.reset_pad()

    check('模型世界装回（后续段落可用）',
          cm.PAD_CENTER == cm._PAD_MODEL_CENTER
          and '显式安装' in cm.PAD_POSE_SOURCE, cm.PAD_POSE_SOURCE)


# ----------------------------------------------------------------- [11]
def _align_rec(dx=3.0, dy=-2.0, dyaw=1.5, n=8, ok=True, spread=0.8,
               reason=None):
    return {'ok': ok, 'reason': reason, 'burst': '/tmp/burst',
            'consensus': {'dx_mm': dx, 'dy_mm': dy, 'dyaw_deg': dyaw,
                          'n_used': n, 'spread_mm': spread}}


def _write_json(d, name, obj):
    p = os.path.join(d, name)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False)
    return p


class _A(object):
    """align_plan_main 用到的那几个属性。"""
    def __init__(self, **kw):
        self.probe_report = kw['probe_report']
        self.scene_index = kw.get('scene_index', 0)
        self.truth = kw.get('truth')
        self.target = kw.get('target')
        self.pad_thick = kw.get('pad_thick')
        self.out_dir = kw['out_dir']
        self.align_json = kw.get('align_json')
        self.hover_z = kw.get('hover_z', cr.ALIGN_HOVER_Z)
        self.topcam_burst = kw.get('topcam_burst')
        self.allow_blocking_view = kw.get('allow_blocking_view', False)
        self.align_burst_dir = kw.get('align_burst_dir')
        self.board_dir = kw.get('board_dir', '$HOME/precision/plans/e9')
        self.board_log_dir = kw.get('board_log_dir', '$HOME/precision/logs/e9')
        self.from_frame = kw.get('from_frame')
        self.q_start = kw.get('q_start')
        self.urdf = kw.get('urdf', URDF)


def section_align():
    print('\n[11] E4b 对中接线：折进目标块 / 低悬停计划 / 脚本切开')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)

    # ---- 1) 应用修正
    rec = cr.apply_align(truth, acts, _align_rec(3.0, -2.0, 1.5), 0)
    b0 = cr.find_block(truth['blocks'], '0')
    check('修正折进块心：x +3 mm、y −2 mm、yaw +1.5°',
          abs(b0['x'] - 0.403) < 1e-9 and abs(b0['y'] - 0.298) < 1e-9
          and abs(b0['yaw'] - 148.13) < 1e-6, str(b0))
    tgt = [a for a in acts if a['kind'] == 'target'][0]
    check('修正同时折进目标动作（from_xy / 抓取角）',
          abs(tgt['from_xy'][0] - 0.403) < 1e-9
          and abs(tgt['grasp_yaw_deg'] - (146.63 + 1.5)) < 1e-6, str(tgt))
    check('清障动作**不动**（它的位姿由探针定，与俯视相对量无关）',
          acts[0]['from_xy'] == [0.40, 0.30])
    check('记录里写明 e_before / 帧数 / 前后位姿',
          abs(rec['e_before_mm'] - 3.6056) < 0.01 and rec['n_frames'] == 8
          and rec['block_xy_before_mm'] == [400.0, 300.0]
          and rec['block_xy_after_mm'] == [403.0, 298.0], str(rec))

    # ---- 2) fail-closed
    for nm, bad in (('ok=false', _align_rec(ok=False, reason='no_fingers')),
                    ('只用上 2 帧', _align_rec(n=2)),
                    ('单次要挪 80 mm', _align_rec(80.0, 0.0))):
        t2, a2 = tower_truth(), cr.actions_from_attempt(
            tower_truth(), tower_attempt(), cm.PAD_THICK)[0]
        ok, why = raises(cr.apply_align, t2, a2, bad, 0)
        check('拒绝 %s（fail-closed）' % nm, ok, why)

    # ---- 3) 已在容差内 ⇒ 不发运动
    t3 = tower_truth()
    a3 = cr.actions_from_attempt(tower_truth(), tower_attempt(),
                                 cm.PAD_THICK)[0]
    r3 = cr.apply_align(t3, a3, _align_rec(1.2, -0.5, 0.4), 0)
    check('残差已在 ≤2 mm / ≤2° 内 ⇒ applied=False，且**不改**任何位姿',
          r3['applied'] is False and r3['reason'] == 'already_in_tol'
          and cr.find_block(t3['blocks'], '0')['x'] == 0.40
          and a3[0]['from_xy'] == [0.40, 0.30], str(r3))

    # ---- 4) 多轮累加 + 轮数上限
    tmp = tempfile.mkdtemp(prefix='cr-align-')
    j1 = _write_json(tmp, 'r1.json', _align_rec(3.0, 0.0, 0.0))
    j2 = _write_json(tmp, 'r2.json', _align_rec(0.0, 2.5, 0.0))
    t4 = tower_truth()
    a4 = cr.actions_from_attempt(tower_truth(), tower_attempt(),
                                 cm.PAD_THICK)[0]
    rs = cr.apply_align_jsons(t4, a4, [j1, j2], 0)
    check('两轮按顺序累加（+3 mm 与 +2.5 mm 都进)',
          len(rs) == 2
          and abs(cr.find_block(t4['blocks'], '0')['x'] - 0.403) < 1e-9
          and abs(cr.find_block(t4['blocks'], '0')['y'] - 0.3025) < 1e-9)
    t5 = tower_truth()
    a5 = cr.actions_from_attempt(tower_truth(), tower_attempt(),
                                 cm.PAD_THICK)[0]
    ok, why = raises(cr.apply_align_jsons, t5, a5, [j1, j2, j1], 0)
    check('超过 %d 轮 ⇒ 拒绝（没收敛要查，不接着掰）'
          % cr.MAX_ALIGN_ROUNDS, ok, why)
    check('没给 align json ⇒ 空记录（不等于「已对准」）',
          cr.apply_align_jsons(tower_truth(),
                               cr.actions_from_attempt(
                                   tower_truth(), tower_attempt(),
                                   cm.PAD_THICK)[0], None, 0) == [])

    # ---- 5) 低悬停计划
    ch = chain()
    q0 = q_start_of(ch)
    out = tempfile.mkdtemp(prefix='cr-align-plan-')
    prec, plan = cr.plan_align_hover(ch, q0, acts[1], out, hover_z=0.360,
                                     name='00-align-hover-r1.json')
    path = os.path.join(out, '00-align-hover-r1.json')
    check('写出 00-align-hover-r1.json', os.path.exists(path))
    on_disk = json.load(open(path, encoding='utf-8'))
    wps = on_disk['waypoints']
    check('两个航点：先抬到 460 mm，再落到悬停 360 mm',
          [w['name'] for w in wps] == ['align_transit', 'align']
          and len(wps) == 2, str([w['name'] for w in wps]))
    zs = [float(ch.fk([float(v) for v in w['q']])[2, 3]) for w in wps]
    check('FK 落点 z = 460 / 360 mm（±1 mm）',
          abs(zs[0] - 0.460) < 1e-3 and abs(zs[1] - 0.360) < 1e-3, str(zs))
    xs = [float(ch.fk([float(v) for v in w['q']])[0, 3]) for w in wps]
    check('落点在目标块正上方（x ±2 mm）',
          all(abs(x - acts[1]['from_xy'][0]) < 2e-3 for x in xs), str(xs))
    check('**张爪**只在第一个航点前（量到的两指才对得上抓取那一刻）',
          wps[0].get('before', {}).get('gripper') == 'open'
          and 'before' not in wps[1], str(wps))
    check('plan 记录 kind/q_start/n_waypoints 齐全',
          prec['kind'] == 'align_hover' and prec['n_waypoints'] == 2
          and on_disk['q_start'] == q0, str(prec))
    ok, why = raises(cr.plan_align_hover, ch, q0, acts[1], out,
                     hover_z=3.0)
    check('悬停高度离谱 ⇒ IK 不过、不写计划', ok, why)

    # ---- 6) 执行脚本：对中 burst / 抓后切开
    plans = [{'name': '00-align-hover-r1.json', 'kind': 'align_hover',
              'n_waypoints': 2},
             {'name': '01-clear-block-8.json', 'kind': 'clear'},
             {'name': '02-target-block-0.json', 'kind': 'target'},
             {'name': '03-view-pick.json', 'kind': 'view', 'view': 'pick'}]
    txt = cr.run_script_text(plans, '$BD', '$LD',
                             burst_dirs={'pick': '$W/pick'},
                             align_burst_dir='$T/r1',
                             held_burst_dir='$T/held')
    i_align = txt.index('--camera topcam --burst 10 $T/r1')
    i_held = txt.index('--camera topcam --burst 10 $T/held')
    i_upto = txt.index('--upto lift')
    i_from = txt.index('--from to_slot --holding-block')
    check('对中 burst 在悬停计划之后、清障之前',
          txt.index('--plan $BD/00-align-hover-r1.json') < i_align < i_held)
    check('目标计划被切开：--upto lift → 拍 → --from to_slot --holding-block',
          i_upto < i_held < i_from, '%d/%d/%d' % (i_upto, i_held, i_from))
    exec_lines = [ln for ln in txt.splitlines()
                  if 'precision_exec.py' in ln]
    check('切开只发生在目标计划上（清障计划照常整条跑）',
          sum('--upto lift' in ln for ln in exec_lines) == 1
          and sum('--holding-block' in ln for ln in exec_lines) == 1
          and sum('--upto lift' not in ln and '--from to_slot' not in ln
                  for ln in exec_lines) == 3, str(exec_lines))
    check('切开后的续跑仍带同一枚令牌（被 --require-auth 管住）',
          all('--require-auth "$TOKEN"' in ln
              for ln in txt.splitlines() if 'precision_exec.py' in ln))
    check('任一 burst 失败 ⇒ 整条停（fail-closed）',
          txt.count('[ $rc -eq 0 ] || fail') >= 6)

    txt2 = cr.run_script_text(plans, '$BD', '$LD')
    check('不给对中/偏心目录 ⇒ 老行为（不切开、不拍俯视）',
          '--upto lift' not in txt2 and '--camera topcam' not in txt2)

    # ---- 6b) 相邻计划合并成**一个** precision_exec 进程（2026-11-12）
    seq = [{'name': '10-target-0.json', 'kind': 'target'},
           {'name': '11-target-1.json', 'kind': 'target'},
           {'name': '12-target-2.json', 'kind': 'target'},
           {'name': '13-view-pick.json', 'kind': 'view', 'view': 'pick'},
           {'name': '14-view-pad.json', 'kind': 'view', 'view': 'pad'},
           {'name': '15-view-free.json', 'kind': 'view', 'view': 'free'}]
    txt3 = cr.run_script_text(seq, '$BD', '$LD',
                              burst_dirs={'pick': '$W/pick', 'pad': '$W/pad',
                                          'free': '$W/free'})
    ex = [ln for ln in txt3.splitlines() if 'precision_exec.py' in ln]
    check('三份相邻目标计划合成一次 exec',
          len(ex) == 4 and ex[0] == (
              '~/precision/run.sh ~/precision/precision_exec.py '
              '--plan $BD/10-target-0.json $BD/11-target-1.json '
              '$BD/12-target-2.json --go --require-auth "$TOKEN" '
              '--log-dir $LD'), str(ex))
    check('机位计划不许合并（每份跑完就地拍 burst，所以各占一个进程）',
          all(ln.count('--plan ') == 1 and '--log $LD/' in ln
              for ln in ex[1:]), str(ex[1:]))
    check('合并组的失败提示点名整组（run.sh 只认 rc）',
          '|| fail "10-target-0.json 11-target-1.json 12-target-2.json '
          'rc=$rc"' in txt3, txt3[-400:])
    i_exec_pick = txt3.index('--plan $BD/13-view-pick.json')
    check('机位 burst 紧跟它自己的 exec（中间不插别的计划）',
          i_exec_pick < txt3.index('--camera wrist --burst 10 $W/pick')
          < txt3.index('--plan $BD/14-view-pad.json'), txt3[-600:])
    # 切开的目标计划**不与邻居合并**（切口必须留在两段之间）
    txt4 = cr.run_script_text(
        [{'name': '20-target-0.json', 'kind': 'target'},
         {'name': '21-target-1.json', 'kind': 'target'}],
        '$BD', '$LD', held_burst_dir='$T/held')
    ex4 = [ln for ln in txt4.splitlines() if 'precision_exec.py' in ln]
    check('被切开的目标计划各占独立进程（--upto / --from 两条）',
          len(ex4) == 4 and sum('--upto lift' in ln for ln in ex4) == 2
          and sum('--from to_slot' in ln for ln in ex4) == 2, str(ex4))

    # ---- 7) align_plan_main 端到端（写 json + sh）
    rep = tempfile.mkdtemp(prefix='cr-align-main-')
    tp = _write_json(rep, 'truth.json', tower_truth())
    rp = _write_json(rep, 'probe.json',
                     {'cell': 'T', 'scenes': [{'truth': tp,
                                               'attempts': [tower_attempt()]}]})
    outp = os.path.join(rep, 'plan')
    a = _A(probe_report=rp, out_dir=outp, q_start=','.join(str(v) for v in q0))
    rc = cr.align_plan_main(None, a)
    check('align_plan_main 成功返回 0', rc == 0, str(rc))
    check('写出 00-align-hover-r1.json / align-r1.sh / align-r1.json',
          all(os.path.exists(os.path.join(outp, f)) for f in
              ('00-align-hover-r1.json', 'align-r1.sh', 'align-r1.json')))
    man = json.load(open(os.path.join(outp, 'align-r1.json'), encoding='utf-8'))
    check('align-r1.json 记 round=1、target、burst 目录',
          man['round'] == 1 and str(man['target']) == '0'
          and 'topcam-align/r1' in man['align_burst_dir'], str(man))
    a2 = _A(probe_report=rp, out_dir=outp,
            q_start=','.join(str(v) for v in q0), align_json=[j1])
    rc = cr.align_plan_main(None, a2)
    man2 = json.load(open(os.path.join(outp, 'align-r2.json'),
                          encoding='utf-8'))
    check('带 --align-json ⇒ 出**第 2 轮**（r2），并把修正记进去',
          rc == 0 and man2['round'] == 2
          and man2['align_applied'][0]['applied'] is True
          and 'topcam-align/r2' in man2['align_burst_dir'], str(man2))
    a3 = _A(probe_report=rp, out_dir=outp,
            q_start=','.join(str(v) for v in q0),
            align_json=[_write_json(rep, 'bad.json', _align_rec(ok=False))])
    lines = []
    real_log = cr.log
    cr.log = lambda *x: lines.append(' '.join(str(v) for v in x))
    try:
        rc = cr.align_plan_main(None, a3)
    finally:
        cr.log = real_log
    check('对中结果不可用 ⇒ 返回 1、不写计划、明说不要执行',
          rc == 1 and not os.path.exists(os.path.join(
              outp, '00-align-hover-r3.json'))
          and any('不要执行' in ln for ln in lines), str(lines))


#: 俯视外参（B1 实测的存值）：相机光轴朝 −y 略斜，仰角 ~41.6°。
RCAM_TOP = np.array([[-0.9961, 0.0306, -0.0830],
                     [0.0824, 0.6637, -0.7434],
                     [0.0324, -0.7474, -0.6636]])


def _topcam_burst(tmp, R=RCAM_TOP, t=(0.4041, 1.0755, 0.8012)):
    """造一份**只有外参**的俯视帧目录（闸门只读 ``tf.json``）。"""
    import stage4_scene_model as sm
    q = sm.mat_to_quat(R)
    tf = {'base_link:topcam_color_optical_frame': {
        'rotation': {k: float(v) for k, v in zip('xyzw', q)},
        'translation': {k: float(v) for k, v in zip('xyz', t)}}}
    d = tempfile.mkdtemp(prefix='topcam-tf-', dir=tmp)
    _write_json(d, 'tf.json', {'ok': tf})
    return d


def section_view_gate():
    print('\n[12] 视线闸门：出计划时就判「两指挡不挡俯视」')
    tmp = tempfile.mkdtemp(prefix='cr-view-')
    burst = _topcam_burst(tmp)
    xyz = (0.5612, 0.3548, 0.360)

    # ψ = 88.75°（closing=0）：分离轴 = tool x ≈ base +y ≈ 视线 ⇒ 挡。
    v_bad = cr.topcam_view_gate(burst, xyz, 88.75)
    check('closing=0（ψ=88.75°）⇒ 视线闸门红（实测那条正是这个姿态）',
          v_bad['lateral'] is False and v_bad['angle_deg'] < 15.0, str(v_bad))
    # ψ 转 90°：两指侧对视线。
    v_ok = cr.topcam_view_gate(burst, xyz, 178.75)
    check('closing=1（ψ=178.75°）⇒ 侧向、闸门绿',
          v_ok['lateral'] is True and v_ok['angle_deg'] >= 60.0, str(v_ok))
    check('闸门记录带阈值与 burst 路径（manifest 要能复盘）',
          abs(v_ok['min_deg'] - 60.0) < 1e-9
          and 'topcam-tf' in v_ok['burst'], str(v_ok))

    # 端到端：挡视线时 align_plan_main 必须返回 1 且**不写**计划。
    # 用 s11 的真实几何：目标块 yaw 88.75° + closing=0 ⇒ ψ=88.75° ⇒ 两指顺着视线。
    q0 = q_start_of(chain())
    rep = tempfile.mkdtemp(prefix='cr-view-main-')
    truth_s11 = tower_truth()
    truth_s11['blocks'][0]['yaw'] = 88.75
    tp = _write_json(rep, 'truth.json', truth_s11)
    rp = _write_json(rep, 'probe.json',
                     {'cell': 'T', 'scenes': [{'truth': tp,
                                               'attempts': [tower_attempt()]}]})
    outp = os.path.join(rep, 'plan')
    a = _A(probe_report=rp, out_dir=outp,
           q_start=','.join(str(v) for v in q0), topcam_burst=burst)
    lines = []
    real_log = cr.log
    cr.log = lambda *x: lines.append(' '.join(str(v) for v in x))
    try:
        rc = cr.align_plan_main(None, a)
    finally:
        cr.log = real_log
    check('视线闸门红 ⇒ rc=1、不写计划、点名 --prefer-closing',
          rc == 1 and not os.path.exists(os.path.join(
              outp, '00-align-hover-r1.json'))
          and any('prefer-closing' in ln for ln in lines), str(lines))
    # 显式承担 ⇒ 照样出计划（诊断用）。
    a_allow = _A(probe_report=rp, out_dir=outp,
                 q_start=','.join(str(v) for v in q0),
                 topcam_burst=burst, allow_blocking_view=True)
    rc = cr.align_plan_main(None, a_allow)
    man = json.load(open(os.path.join(outp, 'align-r1.json'), encoding='utf-8'))
    check('--allow-blocking-view ⇒ 出计划，但 manifest 里留下闸门红记录',
          rc == 0 and man['view_gate']['lateral'] is False, str(man.get(
              'view_gate')))
    # 侧向朝向（ψ=178.75°）⇒ 闸门绿、正常出计划。
    att = tower_attempt()
    att['grasp_hit'] = dict(att['grasp_hit'] or {}, closing=1)
    rp2 = _write_json(rep, 'probe2.json',
                      {'cell': 'T', 'scenes': [{'truth': tp,
                                                'attempts': [att]}]})
    ou2 = os.path.join(rep, 'plan2')
    a2 = _A(probe_report=rp2, out_dir=ou2,
            q_start=','.join(str(v) for v in q0), topcam_burst=burst)
    rc = cr.align_plan_main(None, a2)
    man2 = json.load(open(os.path.join(ou2, 'align-r1.json'), encoding='utf-8'))
    check('探针给了 closing=1 ⇒ 抓取角 +90°、闸门绿、计划写出',
          rc == 0 and man2['view_gate']['lateral'] is True,
          str(man2.get('view_gate')))
    p2 = json.load(open(os.path.join(ou2, '00-align-hover-r1.json'),
                        encoding='utf-8'))
    check('悬停姿态用的就是 ψ = 块 yaw + 90',
          abs(p2['meta']['yaw_grasp_deg'] - (88.75 + 90.0)) < 1e-6,
          str(p2['meta']['yaw_grasp_deg']))


def section_finish():
    print('\n[13] 收尾闸门：没有 round.json 不得给 0（§20.3）')
    final = [{'id': 0, 'x': 0.4242, 'y': -0.11224, 'z': PAD_TOP_Z, 'yaw': 0.0,
              'size': 0.0295},
             {'id': 3, 'x': 0.60, 'y': 0.30, 'z': 0.01475, 'yaw': 20.0,
              'size': 0.0295}]
    mani = {'target': 0, 'cell': 4, 'actions': [{'block': '0'}],
            'final_layout': [dict(b) for b in final]}

    def _args(d):
        return argparse.Namespace(
            manifest=os.path.join(d, 'manifest.json'),
            out_dir=d, verify=os.path.join(d, 'post.json'),
            view_truth=[], wrist_truth=None, wrist_zone=None,
            merge_dedup_mm=40.0, colour_burst=None, colour_search_mm=60.0,
            colour_camera='wrist', untouched_mm=5.0, untouched_deg=3.0,
            moved_mm=40.0, grid=None, allow_model_pad=True)

    def _stage(d):
        with open(os.path.join(d, 'manifest.json'), 'w') as f:
            json.dump(mani, f)
        with open(os.path.join(d, 'post.json'), 'w') as f:
            json.dump({'blocks': final}, f)

    with tempfile.TemporaryDirectory() as d:
        _stage(d)
        rc = cr.verify_main(_args(d))
        check('终态 PASS 但没 round.json ⇒ 收尾返回 2（不是 0）', rc == 2,
              'rc=%r' % rc)
        check('verify.json 仍写出（判决能复盘）',
              os.path.exists(os.path.join(d, 'verify.json')))

        with open(os.path.join(d, 'round.json'), 'w') as f:
            json.dump({'kind': 'stage4-round', 'complete': False,
                       'missing': ['执行日志缺 target.jsonl']}, f)
        rc = cr.verify_main(_args(d))
        check('round.json 标 complete=false ⇒ 收尾返回 2', rc == 2,
              'rc=%r' % rc)

        with open(os.path.join(d, 'round.json'), 'w') as f:
            json.dump({'kind': 'stage4-round', 'complete': True,
                       'missing': []}, f)
        rc = cr.verify_main(_args(d))
        check('round.json 完整 ⇒ 收尾按终态给 0', rc == 0, 'rc=%r' % rc)


# ------------------------------------------ A2 接线（2026-11-14 补洞）
def section_validator_passthrough():
    print('\n[4d] A2 段级裁决：validator 从 plan_sequence 一路传到每个 DP')
    truth, att = tower_truth(), tower_attempt()
    acts, blocks = cr.actions_from_attempt(truth, att, cm.PAD_THICK)
    ch = chain()
    q0 = q_start_of(ch)
    seen = []
    vcalls = []

    def sentinel(qa, qb):
        # 必须**可调用**：solve_path 对每个转移真调它（这里放行，只记账）
        vcalls.append(1)
        return True, ""

    real_solve = pp.solve_path

    def spy(ch_, targets, q_start, **kw):
        seen.append(kw.get('validator'))
        return real_solve(ch_, targets, q_start, **kw)

    real = pp.solve_path
    try:
        pp.solve_path = spy
        out = tempfile.mkdtemp(prefix='e9-validator-')
        plans, layout, q_end = cr.plan_sequence(
            ch, q0, acts, blocks, out, wrist_survey=False,
            place_yaw_equiv=True, validator=sentinel)
        check('每个动作的 DP 都拿到同一个 validator（%d 次 solve_path）'
              % len(seen),
              len(seen) >= 2 and all(s is sentinel for s in seen), seen)
        check('validator 真被 DP 逐转移调用（%d 次，放行）' % len(vcalls),
              len(vcalls) >= 10, len(vcalls))
        check('计划 meta 记录裁决在场（collision_validator=true）',
              all(json.load(open(p['path']))['meta']['collision_validator']
                  is True for p in plans))
        seen.clear()
        plans2, _l, _q = cr.plan_sequence(
            ch, q0, acts, blocks, out, wrist_survey=True,
            place_yaw_equiv=True, validator=sentinel)
        check('末尾机位链（plan_views）也透传 validator（追加 3 份机位计划）',
              len(plans2) == len(plans) + 3
              and all(s is sentinel for s in seen), (len(plans2), seen))
    finally:
        pp.solve_path = real
    # 不传（缺省 None）⇒ DP 拿 None、meta 如实记 false（审计：此前 None 静默）
    out2 = tempfile.mkdtemp(prefix='e9-novalidator-')
    seen.clear()
    try:
        pp.solve_path = spy
        cr.plan_sequence(ch, q0, acts, blocks, out2, wrist_survey=False,
                         place_yaw_equiv=True)
    finally:
        pp.solve_path = real
    check('缺省（None）⇒ DP 全部收到 None',
          seen and all(s is None for s in seen), seen)


def main():
    print('=' * 68)
    # C1：模块缺省 = 未实测哨兵；本套件是模型世界（仿真几何），显式装模型常量。
    # section_pad_pose 自己负责「未实测态」的用例（它内部 reset）。
    cm.install_model_pad_defaults()
    for fn in (section_yaw, section_surface, section_actions, section_bias,
               section_plans, section_multitarget,
               section_step_cap, section_seed_propagation,
               section_view_chain,
               section_place_yaw_equiv, section_validator_passthrough,
               section_from_xy,
               section_gap, section_mutation, section_verify, section_fuse,
               section_merge_truth, section_colour, section_pad_pose,
               section_align, section_view_gate, section_finish):
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
