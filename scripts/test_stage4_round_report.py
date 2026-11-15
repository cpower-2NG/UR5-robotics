#!/usr/bin/env python3
"""``stage4_round_report.py`` 的自检（§20.3）。

钉住三件事：

1. **装配正确**：执行日志 → 抓取四键 + 回读/滑移/跑后；彩色量 → 放置三键；
   三机位真值 → 全桌终态。生成的 ``round.json`` 必须能被 ``stage4_score.py``
   直接判，且一轮全绿时 verdict=ok。
2. **缺证据如实报缺**：少执行日志 / 少 facts / 少垫参数 / 少彩色量 ⇒
   ``complete=false``、退出码 2、判分器判 ``missing_data``，**不许填默认值
   把「没测」算成「失败」或「通过」**。
3. **s11 重放**：用仓库外的历史记录跑一遍，确认装配器不崩、能写出文件；
   因为板上没有归档执行日志、也没有人的观察事实，这一轮**应该**是
   ``missing_data``（这正是 §20.3 要的行为，不是缺陷）。

跑法：``python3 scripts/test_stage4_round_report.py``
"""
import json
import math
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                               # noqa: E402
import stage4_round_report as rr                              # noqa: E402
import stage4_score as sc                                     # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []

#: s11 的历史记录在仓外台账目录（只读使用；不许往那里写）。
S11 = os.path.expanduser(
    "~/robotics/notes/ur5-vision-grasp/runs/2026-11-10-stage4-entry/e9-s11")

# C1：模块缺省 = 未实测哨兵 ⇒ 模型世界套件显式装模型常量（main() 里再装一次
# 以覆盖中途 reset 的段落）。
cm.install_model_pad_defaults()
PAD_TOP_Z = cm.PAD_THICK + 0.0295 / 2.0


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def _write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    return path


def _stage(tmp):
    """造一轮完整的假数据；返回所有路径。"""
    plan_dir = os.path.join(tmp, 'plan')
    log_dir = os.path.join(tmp, 'board-log')
    view_dir = os.path.join(tmp, 'views')
    for d in (plan_dir, log_dir, view_dir):
        os.makedirs(d, exist_ok=True)

    # 目标块（id=2）原在自由区，跑后在垫面宫格 4
    origin = {'id': 2, 'x': 0.40, 'y': 0.30, 'z': 0.01475, 'yaw': 0.0,
              'size': 0.0295}
    other = {'id': 5, 'x': 0.60, 'y': 0.30, 'z': 0.01475, 'yaw': 0.0,
             'size': 0.0295}
    layout = _write(os.path.join(tmp, 'layout-truth.json'),
                    {'blocks': [origin, other]})

    # cell 4 在默认垫位姿下就是垫心；实测 grid 与它重合，便于断言
    cx, cy = cm.cell_center(4)
    on_pad = {'id': 0, 'x': cx, 'y': cy, 'z': PAD_TOP_Z, 'yaw': 0.0,
              'size': 0.0295}
    view_pick = _write(os.path.join(view_dir, 'pick.json'),
                       {'blocks': [on_pad]})
    view_free = _write(os.path.join(view_dir, 'free.json'),
                       {'blocks': [dict(other)]})

    plan = _write(os.path.join(plan_dir, 'target.json'),
                  {'meta': {'kind': 'pick', 'block_idx': 2}})
    manifest = _write(os.path.join(tmp, 'manifest.json'), {
        'target': 2, 'cell': 4, 'truth': layout,
        'actions': [{'kind': 'target', 'block': '2', 'slot': [cx, cy]}],
        'plans': [{'kind': 'target', 'name': 'target.json', 'path': plan}],
    })

    # 板端日志：双后缀（计划名本身带 .json）+ 非事件行
    with open(os.path.join(log_dir, 'target.json.jsonl'), 'w') as f:
        f.write('[1, 2, 3]\n')                       # 轨迹采样行，必须跳过
        f.write('这是一行人看的文字\n')
        f.write(json.dumps({'event': 'gate', 'ok': True}) + '\n')
        f.write(json.dumps({'event': 'traj', 'waypoint': 'grasp',
                            'ok': True}) + '\n')
        f.write(json.dumps({'event': 'gripper', 'where': 'grasp.after',
                            'action': 'close', 'position_mm': 26.2}) + '\n')
        f.write(json.dumps({'event': 'traj', 'waypoint': 'lift',
                            'ok': True}) + '\n')
        f.write(json.dumps({'event': 'grasp_check', 'closed_mm': 26.2,
                            'lifted_mm': 26.1, 'slipped': False}) + '\n')

    colour = _write(os.path.join(tmp, 'colour.json'), {
        'ok': True, 'xy_mm': [cx * 1000.0, cy * 1000.0], 'n_frames': 10,
        'n_missed': 0, 'range_mm': [1.0, 1.0], 'max_dev_mm': 1.0,
        'hue_deg': 35.0, 'search_mm': 60.0})

    grid = _write(os.path.join(tmp, 'grid-pre.json'), {
        'grid_center_base_mm': [cx * 1000.0, cy * 1000.0, 0.16842],
        'grid_yaw_base_deg': 0.0})
    spec = _write(os.path.join(tmp, 'pad-spec.json'), {
        'thickness_m': 0.00016842, 'footprint_m': [0.30, 0.30]})

    facts = _write(os.path.join(tmp, 'facts.json'), {
        'round_id': 'S1-L2-r1',
        'scene': {'batch': 'S1', 'layout': 'L2', 'seed': 11},
        'truth_source': 'postrun-wrist-3view',
        'specs': {'2': '橙色、靠北那块'},
        'auto_top1': {'2': True},
        'safety': {'collisions': 0, 'drops': 0, 'unexpected_motion': 0,
                   'estop': 0},
        'interventions': {'count': 0},
        'human_review': {'required': False, 'approved': True}})

    return {'tmp': tmp, 'manifest': manifest, 'log_dir': log_dir,
            'view_pick': view_pick, 'view_free': view_free,
            'colour': colour, 'grid': grid, 'spec': spec, 'facts': facts,
            'origin': origin, 'on_pad': on_pad}


def _argv(fx, out):
    return ['--manifest', fx['manifest'], '--facts', fx['facts'],
            '--exec-log', fx['log_dir'],
            '--view-truth', 'pick=' + fx['view_pick'],
            '--view-truth', 'free=' + fx['view_free'],
            '--colour', fx['colour'], '--pad', fx['grid'],
            '--pad-spec', fx['spec'], '--round-id', 'S1-L2-r1',
            '--out', out]


# ------------------------------------------------------------------ [1]
def section_exec_evidence():
    print('\n[1] 执行日志 → 抓取证据（只认 .after 的 close 回读）')
    ev = rr.exec_evidence([
        {'event': 'gate', 'ok': True},
        {'event': 'traj', 'ok': True},
        {'event': 'gripper', 'where': 'grasp.after', 'action': 'close',
         'position_mm': 26.2},
        {'event': 'grasp_check', 'closed_mm': 26.2, 'lifted_mm': 26.1,
         'slipped': False},
    ])
    check('全绿日志 ⇒ exec_rc=0 / lifted / slipped=false / 回读 26.2',
          ev['exec_rc'] == 0 and ev.get('lifted') is True
          and ev.get('slipped') is False
          and abs(ev.get('closed_mm') - 26.2) < 1e-9, str(ev))

    ev = rr.exec_evidence([{'event': 'gate', 'ok': False}])
    check('闸门不过 ⇒ exec_rc=2', ev['exec_rc'] == 2, str(ev))
    ev = rr.exec_evidence([{'event': 'traj', 'ok': False}])
    check('轨迹失败 ⇒ exec_rc=1', ev['exec_rc'] == 1, str(ev))
    ev = rr.exec_evidence([{'event': 'gripper_fail', 'where': 'grasp'}])
    check('夹爪动作判红 ⇒ exec_rc=1', ev['exec_rc'] == 1, str(ev))

    ev = rr.exec_evidence([
        {'event': 'gate', 'ok': True},
        {'event': 'aborted_token', 'reason': 'expired 33s ago'},
    ])
    check('令牌过期事件 ⇒ aborted_token=True（不算 traj/gripper 失败）',
          ev.get('aborted_token') is True and ev['exec_rc'] == 0, str(ev))
    ev = rr.exec_evidence([
        {'event': 'gate', 'ok': True},
        {'event': 'stop_requested', 'reason': '现场按停'},
    ])
    check('软停事件 ⇒ stop_requested=True（与 aborted_token 分开记）',
          ev.get('stop_requested') is True and 'aborted_token' not in ev
          and ev['exec_rc'] == 0, str(ev))
    ev = rr.exec_evidence([{'event': 'traj', 'ok': True}])
    check('没有令牌事件 ⇒ 不写 aborted_token 键', 'aborted_token' not in ev,
          str(ev))

    ev = rr.exec_evidence([
        {'event': 'gripper', 'where': 'selftest.close', 'action': 'close',
         'position_mm': 0.1}])
    check('selftest 的 close 回读不算抓取回读', 'closed_mm' not in ev,
          str(ev))

    ev = rr.exec_evidence([
        {'event': 'gripper', 'where': 'grasp.after', 'action': 'close',
         'position_mm': None}])
    check('回读是 None ⇒ 不写 closed_mm（缺证据，不写 0）',
          'closed_mm' not in ev, str(ev))

    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'x.jsonl')
        with open(p, 'w') as f:
            f.write('[1, 2]\n')
            f.write('文字行\n')
            f.write('{"event": "gate", "ok": true}\n')
            f.write('\n')
        evs = rr.read_events(p)
    check('read_events 跳过轨迹数组 / 文字 / 空行', len(evs) == 1
          and evs[0]['event'] == 'gate', str(evs))


# ------------------------------------------------------------------ [2]
def section_happy():
    print('\n[2] 完整一轮：日志 + 三机位 + 彩色 + 实测垫 ⇒ 字段齐全')
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        out = os.path.join(d, 'round.json')
        rc = rr.main(_argv(fx, out))
        check('字段齐全 ⇒ 退出码 0', rc == 0, 'rc=%r' % rc)
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)

    check('kind / round_id / scene 写对',
          doc['kind'] == 'stage4-round' and doc['round_id'] == 'S1-L2-r1'
          and doc['scene'] == {'batch': 'S1', 'layout': 'L2', 'seed': 11},
          json.dumps(doc['scene'], ensure_ascii=False))
    check('complete=true 且 missing 为空',
          doc['complete'] is True and doc['missing'] == [], str(doc['missing']))

    a = doc['attempts'][0]
    check('target.block_id = "2"，spec 来自 facts',
          a['target']['block_id'] == '2' and a['spec'] == '橙色、靠北那块',
          json.dumps(a.get('target'), ensure_ascii=False))
    check('selected.instance_id = 2（plan.meta.block_idx）+ auto_top1',
          a['selected']['instance_id'] == 2
          and a['selected']['auto_top1'] is True, str(a.get('selected')))
    check('truth 带独立来源白名单里的值',
          a['truth']['instance_id'] == 2
          and a['truth']['truth_source'] == 'postrun-wrist-3view',
          str(a.get('truth')))
    g = a['grasp']
    check('grasp 四键：lifted / dropped / wrong_block / exec_rc',
          g['lifted'] is True and g['dropped'] is False
          and g['wrong_block'] is False and g['exec_rc'] == 0, str(g))
    check('回读判定夹住（26.2 > 5 mm）、没滑',
          g['readback']['gripped'] is True and g['readback']['closed_mm'] == 26.2
          and g['slip'] is False, str(g.get('readback')))
    check('postrun：离开原点且到达放置格',
          g['postrun']['target_left_origin'] is True
          and g['postrun']['arrived'] is True, str(g['postrun']))
    check('place 三键都是真布尔',
          [a['place'][k] for k in ('in_slot', 'over_line', 'out_of_bounds')]
          == [True, False, False], str(a.get('place')))
    check('place 附带实测坐标与格心（扰动检测要吃，第 342 轮）',
          a['place'].get('xy_mm') is not None
          and a['place'].get('cell') is not None, str(a.get('place')))
    check('成功尝试不硬塞失败类', a['failure'] == {'class': None},
          str(a.get('failure')))

    t = doc['terminal']
    check('终态：3 块全合法、0 穿插 / 0 掉桌 / 0 出桌',
          t['overlaps'] == 0 and t['off_table'] == 0
          and t['out_of_bounds'] == 0
          and all(b['legal'] is True for b in t['blocks']), str(t))
    check('pad 记实测厚度 / 足迹 / 来源',
          abs(doc['pad']['thickness_m'] - 0.00016842) < 1e-12
          and doc['pad']['footprint_m'] == [0.30, 0.30]
          and 'grid-pre.json' in doc['pad']['source'], str(doc['pad']))

    res = sc.score_round(doc)
    check('判分器接受：verdict=ok、四条率 100%',
          res['verdict'] == 'ok'
          and all(abs((res['rates'][k] or 0) - 1.0) < 1e-9 for k in
                  ('id_rate', 'grasp_rate', 'place_rate', 'terminal_rate')),
          '%s：%s' % (res['verdict'], res.get('reason')))
    check('判分器数到 1 次尝试、0 void / 0 missing',
          res['n_scored'] == 1 and res['n_void'] == 0
          and res['attempts'][0]['status'] == 'ok',
          str(res['attempts'][0]))


# ------------------------------------------------------------------ [3]
def section_missing():
    print('\n[3] 缺证据：complete=false、退出码 2、判分器判 missing_data')
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        out = os.path.join(d, 'round.json')
        argv = _argv(fx, out)
        # 去掉 --exec-log（板端日志没归档）
        i = argv.index('--exec-log')
        argv = argv[:i] + argv[i + 2:]
        rc = rr.main(argv)
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
        res = sc.score_round(doc)
    check('没板端执行日志 ⇒ 退出码 2', rc == 2, 'rc=%r' % rc)
    check('round.json 标 complete=false 并点名缺日志',
          doc['complete'] is False
          and any('exec-log' in m for m in doc['missing']), str(doc['missing']))
    check('判分器判 missing_data（不把无证据算成抓取失败）',
          res['verdict'] == 'missing_data', res['verdict'])
    check('尝试级也标 missing（不是 FAIL）',
          res['attempts'][0]['status'] == 'missing_data',
          str(res['attempts'][0]))

    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        out = os.path.join(d, 'round.json')
        argv = _argv(fx, out)
        i = argv.index('--facts')
        argv = argv[:i] + argv[i + 2:]
        rc = rr.main(argv)
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    check('没 --facts ⇒ 退出码 2、点名安全/干预/人眼/来源',
          rc == 2 and any('facts' in m for m in doc['missing'])
          and doc['safety'] is None, str(doc['missing']))

    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        out = os.path.join(d, 'round.json')
        argv = _argv(fx, out)
        i = argv.index('--pad-spec')
        argv = argv[:i] + argv[i + 2:]
        rc = rr.main(argv)
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    check('没实测垫厚/足迹 ⇒ 退出码 2、pad=None',
          rc == 2 and doc['pad'] is None
          and any('垫厚' in m or '垫足迹' in m for m in doc['missing']),
          str(doc['missing']))
    check('垫缺 ⇒ 放置三键也不写（不许猜）',
          'place' not in doc['attempts'][0], str(doc['attempts'][0]))

    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        out = os.path.join(d, 'round.json')
        argv = _argv(fx, out)
        i = argv.index('--colour')
        argv = argv[:i] + argv[i + 2:]
        rc = rr.main(argv)
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    check('没彩色量 ⇒ 退出码 2（垫面深度判不了落格）',
          rc == 2 and 'place' not in doc['attempts'][0]
          and any('彩色' in m for m in doc['missing']), str(doc['missing']))


# ------------------------------------------------------------------ [4]
def section_pad_constant():
    print('\n[4] 模型常量垫位姿：产出器照实记，判分器拒收')
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        # 把 grid 文件改名成 gen_scene 开头：来源里带模型常量标记
        bad = os.path.join(d, 'gen_scene-grid.json')
        os.replace(fx['grid'], bad)
        fx['grid'] = bad
        out = os.path.join(d, 'round.json')
        rr.main(_argv(fx, out))
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    res = sc.score_round(doc)
    check('来源含 gen_scene ⇒ 判分器 missing_data（不认模型常量）',
          res['verdict'] == 'missing_data' and '模型常量' in res['reason'],
          '%s：%s' % (res['verdict'], res.get('reason')))


# ------------------------------------------------------------------ [4b]
def section_pad_thickness_unmeasured():
    print('\n[4b] 垫厚不卡尺实测：用 pad solve 的垫面 z，不算缺失')
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        spec = _write(os.path.join(d, 'pad-spec.json'),
                      {'footprint_m': [0.30, 0.30]})
        pad, missing = rr.load_pad(fx['grid'], spec)
    check('spec 缺 thickness_m 不报缺', missing == [], missing)
    check('厚度取垫面 z（0.16842 mm）',
          pad is not None and abs(pad['thickness_m'] - 0.00016842) < 1e-12, pad)
    check('来源写明 surface_z',
          'surface_z' in (pad or {}).get('thickness_source', ''), pad)
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        spec = _write(os.path.join(d, 'pad-spec.json'),
                      {'thickness_m': 0.002, 'footprint_m': [0.30, 0.30]})
        pad2, missing2 = rr.load_pad(fx['grid'], spec)
    check('显式 thickness_m 优先', missing2 == []
          and abs(pad2['thickness_m'] - 0.002) < 1e-12, pad2)
    check('显式厚度的来源 = pad-spec',
          pad2.get('thickness_source') == 'pad-spec', pad2)


# ------------------------------------------------------------------ [5]
def section_void_and_class():
    print('\n[5] void 与失败七类')
    with tempfile.TemporaryDirectory() as d:
        fx = _stage(d)
        facts = json.load(open(fx['facts'], encoding='utf-8'))
        facts['voids'] = {'2': '相机掉线'}
        _write(fx['facts'], facts)
        out = os.path.join(d, 'round.json')
        rr.main(_argv(fx, out))
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
    a = doc['attempts'][0]
    check('void 尝试只留 target/spec/void（不写抓取/放置）',
          a.get('void', {}).get('reason') == '相机掉线'
          and 'grasp' not in a and 'place' not in a, str(a))
    row = sc.score_attempt(a)
    check('判分器把 void 排除出分母', row['status'] == 'void', str(row))

    def att(**kw):
        base = {'target': {'block_id': '2'}, 'spec': 'x',
                'selected': {'instance_id': 2},
                'truth': {'instance_id': 2,
                          'truth_source': 'postrun-wrist-3view'},
                'grasp': {'lifted': True, 'dropped': False,
                          'wrong_block': False, 'exec_rc': 0,
                          'readback': {'gripped': True}, 'slip': False,
                          'postrun': {'target_left_origin': True,
                                      'arrived': True}},
                'place': {'in_slot': True, 'over_line': False,
                          'out_of_bounds': False},
                'failure': {'class': None}}
        base.update(kw)
        return base

    check('选错块 ⇒ perception',
          rr._failure_class(att(selected={'instance_id': 3}), None)
          == 'perception')
    check('没抬起 ⇒ candidate',
          rr._failure_class(att(grasp=dict(att()['grasp'], lifted=False)), None)
          == 'candidate')
    check('执行 rc≠0 ⇒ execution',
          rr._failure_class(att(grasp=dict(att()['grasp'], exec_rc=1)), None)
          == 'execution')
    check('放错格 ⇒ placement',
          rr._failure_class(att(place={'in_slot': False, 'over_line': False,
                                       'out_of_bounds': False}), None)
          == 'placement')
    check('人给的七类值优先（human）',
          rr._failure_class(att(), 'human') == 'human')
    check('不在七类的值不采信（回落到推导）',
          rr._failure_class(att(), 'bogus') in (None, 'placement'))
    check('七类就是判分器那一份（同源）',
          set(rr._failure_class(att(), c) for c in sc.FAIL_CLASSES)
          == set(sc.FAIL_CLASSES))


# ------------------------------------------------------------------ [6]
def section_s11_replay():
    print('\n[6] s11 历史重放：能装配、能判、如实报缺')
    if not os.path.isdir(S11):
        check('s11 记录存在（仓外台账目录）', False, S11)
        return
    with tempfile.TemporaryDirectory() as d:
        spec = _write(os.path.join(d, 'pad-spec.json'), {
            'thickness_m': 0.00016842, 'footprint_m': [0.30, 0.30]})
        out = os.path.join(d, 'round.json')
        rc = rr.main([
            '--manifest', os.path.join(S11, 'plan', 'manifest.json'),
            '--view-truth', 'pick=' + os.path.join(S11, 'post-pick.json'),
            '--view-truth', 'free=' + os.path.join(S11, 'post-free.json'),
            '--view-truth', 'pad=' + os.path.join(S11, 'post-pad.json'),
            '--pad', os.path.join(S11, 'grid-pre.json'),
            '--pad-spec', spec, '--out', out])
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)

    check('s11 重放：能写出 round.json（哪怕证据不全）',
          doc.get('kind') == 'stage4-round', '%r' % rc)
    check('s11 没有板上执行日志 ⇒ 退出码 2（预期的缺证据）', rc == 2,
          'rc=%r' % rc)
    check('s11 三机位真值合并出终态（非 None）',
          isinstance(doc.get('terminal'), dict)
          and isinstance(doc['terminal'].get('blocks'), list),
          str(type(doc.get('terminal'))))
    a = doc['attempts'][0]
    check('s11 目标块 = 2，selected 来自计划 meta',
          a['target']['block_id'] == '2'
          and a['selected']['instance_id'] == 2, str(a.get('selected')))
    check('s11 真值来源没声明 ⇒ truth_source=None（不猜）',
          a['truth']['truth_source'] is None, str(a.get('truth')))
    res = sc.score_round(doc)
    check('s11 重放判分器接受且判 missing_data',
          res['verdict'] == 'missing_data', res['verdict'])


def section_disturbance():
    """邻块扰动嫌疑（台账第三百四十一轮）：沿「i 格心→j 格心」方向推 >5 mm
    记一条；垂直偏移与次序不符不记（自身放置误差 ≠ 推挤）。"""
    print('\n[7] 邻块扰动嫌疑：方向 + 次序都对才算')
    c0, c1 = cm.cell_center(0), cm.cell_center(1)      # 相邻格（60 mm）
    ux, uy = c1[0] - c0[0], c1[1] - c0[1]
    n = math.hypot(ux, uy)
    ux, uy = ux / n, uy / n

    def att(tid, cell, xy_mm, i_text=''):
        return {'target': {'block_id': tid},
                'place': {'in_slot': True, 'over_line': False,
                          'out_of_bounds': False, 'cell': cell,
                          'xy_mm': [round(v * 1000.0, 1) for v in xy_mm]}}

    pushed = att('A', 0, (c0[0] + ux * 0.008, c0[1] + uy * 0.008))
    pusher = att('B', 1, (c1[0], c1[1]))
    s = rr.disturbance_suspects([pushed, pusher])
    check('沿推挤方向偏 8 mm ⇒ 记嫌疑（A 被 B 推，8 mm）',
          len(s) == 1 and s[0]['pushed'] == 'A' and s[0]['by'] == 'B'
          and abs(s[0]['offset_mm'] - 8.0) < 0.2, str(s))
    s2 = rr.disturbance_suspects([pusher, pushed])
    check('放置次序反了（后放的偏）⇒ 不记', s2 == [], str(s2))
    side = att('A', 0, (c0[0] - uy * 0.008, c0[1] + ux * 0.008))
    s3 = rr.disturbance_suspects([side, pusher])
    check('垂直于格心连线的偏移 ⇒ 不记（自身放置误差）', s3 == [], str(s3))
    s4 = rr.disturbance_suspects([pushed])
    check('只有一个有证据的块 ⇒ 无从谈推挤', s4 == [], str(s4))


def main():
    print('=' * 68)
    # C1：模块缺省 = 未实测哨兵；模型世界用例显式装模型常量。
    cm.install_model_pad_defaults()
    for fn in (section_exec_evidence, section_happy, section_missing,
               section_pad_constant, section_pad_thickness_unmeasured,
               section_void_and_class, section_disturbance,
               section_s11_replay):
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
