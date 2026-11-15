#!/usr/bin/env python3
"""E9 清障模型的离线自检：宫格 / 自由区 / 终态合法性 / 遮挡归属 / 变异。

为什么这些值得单测：E9 的结论全部压在「终态合法」这一条上，而 S3/S4 的布局
**允许块间接触**（S4 有 1 个塔 + 2 对贴面块）。用 ``sm.gap`` 判重叠会把接触
判成非法、用无容差的 ``separation`` 判会把浮点噪声判成穿插 —— 两个方向都会
让「全桌合法 ≥80%」这个门失真。这里把三种状态（分离 / 接触 / 穿插）都钉住。

跑法：python3 scripts/test_stage4_clear_model.py
"""
import inspect
import json
import math
import os
import re
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                                 # noqa: E402
import stage4_scene_model as sm                                 # noqa: E402

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


def _raises_pad_not_measured(fn, *a):
    try:
        fn(*a)
    except cm.PadNotMeasured:
        return True
    return False


def _mut(old, new):
    src = inspect.getsource(cm)
    assert old in src, '变异锚点不在源码里：%r' % old
    ns = {'__file__': cm.__file__, '__name__': 'mut'}
    exec(compile(src.replace(old, new, 1), '<mut>', 'exec'), ns)
    # C1：变异模块重新 exec ⇒ 回到「未实测」哨兵；变异用例全是模型世界，
    # 统一显式装模型常量。
    ns['install_model_pad_defaults']()
    return ns


def s4_blocks(index=0):
    """按固定种子重算 S4 布局（不落盘，与 gen_scene 同一份生成器）。"""
    return [dict(b) for b in cm._GS.make_layout("S4", index)]


# ------------------------------------------------------------------ [1]
def section_gen_scene_sync():
    print('\n[1] 反查 gen_scene：垫 / 宫格 / 拾取区 / 桌面不是一个家')
    src = open(cm._GS.__file__, encoding='utf-8').read()
    m = re.search(r'<geometry><box><size>([\d.]+) ([\d.]+) 0\.04</size>', src)
    check('桌面尺寸与 gen_scene 的 SDF 一致（1.15×1.10）',
          m is not None and [float(m.group(1)), float(m.group(2))]
          == [cm.TABLE_SIZE[0], cm.TABLE_SIZE[1]],
          m.groups() if m else '没找到')
    m = re.search(r'<model name="table">.*?<pose>([-\d.]+) ([-\d.]+) [-\d.]+',
                  src, re.S)
    check('桌面位姿与 gen_scene 一致（0.225, 0.10）',
          m is not None and [float(m.group(1)), float(m.group(2))]
          == list(cm.TABLE_CENTER), m.groups() if m else '没找到')
    check('拾取区就是 gen_scene 的 ZONE',
          cm.PICK_ZONE == tuple(cm._GS.ZONE), str(cm.PICK_ZONE))
    check('宫格间距 = PAD_PITCH_MM / 1000',
          cm.PAD_PITCH == cm._GS.PAD_PITCH_MM / 1000.0)
    check('垫偏航 = PAD_YAW_DEG', math.degrees(cm.PAD_YAW_RAD) == cm._GS.PAD_YAW_DEG)


# ------------------------------------------------------------------ [2]
def section_cells():
    print('\n[2] 宫格：9 格、间距 60 mm、格心反查一致、29.5 mm 方块任意 yaw 不压格')
    centers = [cm.cell_center(i) for i in range(9)]
    check('9 个格心互不相同', len({(round(x, 9), round(y, 9))
                               for x, y in centers}) == 9)
    check('第 4 格 = 垫心', all(abs(a - b) < 1e-12 for a, b in
                             zip(centers[4], cm.PAD_CENTER)))
    # 同排相邻格心间距 = 60 mm；同列同理
    d_row = math.hypot(centers[1][0] - centers[0][0], centers[1][1] - centers[0][1])
    d_col = math.hypot(centers[3][0] - centers[0][0], centers[3][1] - centers[0][1])
    check('同排间距 60 mm', abs(d_row - cm.PAD_PITCH) < 1e-9, str(d_row))
    check('同列间距 60 mm', abs(d_col - cm.PAD_PITCH) < 1e-9, str(d_col))
    check('格心反查 = 自己',
          all(cm.cell_index_of(*cm.cell_center(i)) == i for i in range(9)))
    check('垫外一点不给格号',
          cm.cell_index_of(cm.PAD_CENTER[0] + 0.3, cm.PAD_CENTER[1]) is None)
    ok_all = all(cm.cell_contains(*cm.cell_center(i), yaw, cm.BLOCK_M)[0]
                 for i in range(9) for yaw in (0, 15, 45, 90, 135, 179))
    check('29.5 mm 方块在格心、任意 yaw 都在格内（半对角 20.9 < 30 mm）', ok_all)
    x, y = cm.cell_center(0)
    inside, idx = cm.cell_contains(x + 0.02, y, 0.0, cm.BLOCK_M)
    check('偏离格心 20 mm → 压格', idx == 0 and not inside)
    inside, _ = cm.cell_contains(*cm.cell_center(0), 0.0, 0.070)
    check('70 mm 的大块放格心 → 压格（半边长 35 > 30）', not inside)


# ------------------------------------------------------------------ [2b]
def section_cell_plan():
    print('\n[2b] 轮级格位映射：默认 8,6,4,2,0、id 升序、奇偶一致、两两 ≥84.85 mm')
    check('DEFAULT_CELLS = 8,6,4,2,0', tuple(cm.DEFAULT_CELLS) == (8, 6, 4, 2, 0))
    check('parse_cells: 半角 / 中文逗号都收',
          cm.parse_cells('8,6,4,2,0') == [8, 6, 4, 2, 0] and
          cm.parse_cells('8，6，4，2，0') == [8, 6, 4, 2, 0])
    check('parse_cells: 空 ⇒ 默认', cm.parse_cells(None) == [8, 6, 4, 2, 0] and
          cm.parse_cells('  ') == [8, 6, 4, 2, 0])
    ok = False
    try:
        cm.parse_cells('8,6,4', n=5)
    except ValueError:
        ok = True
    check('parse_cells: 个数与目标数不符 ⇒ 报错（不许少一个格）', ok)
    ok = False
    try:
        cm.parse_cells('8,6,6,2,0')
    except ValueError:
        ok = True
    check('parse_cells: 重复格号 ⇒ 报错', ok)
    ok = False
    try:
        cm.parse_cells('8,6,9,2,0')
    except ValueError:
        ok = True
    check('parse_cells: 越界格号 ⇒ 报错', ok)
    # 按批缺省格（2026-11-15 S1/S2 接入编排器：六块六格，S0 不变）
    check('parse_cells: 按批缺省 S1/S2 = 8,6,4,2,1,0；S0/未知/无批 = 旧五格',
          cm.parse_cells(None, 6, batch='S1') == [8, 6, 4, 2, 1, 0] and
          cm.parse_cells(None, 6, batch='S2') == [8, 6, 4, 2, 1, 0] and
          cm.parse_cells(None, 5, batch='S0') == [8, 6, 4, 2, 0] and
          cm.parse_cells(None, 5, batch='S9') == [8, 6, 4, 2, 0] and
          cm.parse_cells(None) == [8, 6, 4, 2, 0])
    check('parse_cells: 显式 spec 优先于按批缺省（行为不变）',
          cm.parse_cells('8,5,2', 3, batch='S1') == [8, 5, 2])
    check('DEFAULT_CELLS_BY_BATCH: S1 六格互不重复、都在 0–8、S0 同旧值',
          len(set(cm.DEFAULT_CELLS_BY_BATCH['S1'])) == 6 and
          all(0 <= c < 9 for c in cm.DEFAULT_CELLS_BY_BATCH['S1']) and
          cm.DEFAULT_CELLS_BY_BATCH['S0'] == tuple(cm.DEFAULT_CELLS))

    ids = ['03', '01', '05', '02', '04']          # 故意乱序
    mapping = cm.assign_cells(ids)
    check('assign_cells: id 升序 → 8,6,4,2,0',
          mapping == {'01': 8, '02': 6, '03': 4, '04': 2, '05': 0},
          str(mapping))
    check('assign_cells: 数字 id 也按数值排（不是字典序）',
          cm.assign_cells([2, 10, 1], [8, 6, 4]) == {'1': 8, '2': 6, '10': 4})
    check('assign_cells: 显式格位照用',
          cm.assign_cells(['a', 'b'], [0, 8]) == {'a': 0, 'b': 8})
    ok = False
    try:
        cm.assign_cells(['01', '02'], [8])
    except ValueError:
        ok = True
    check('assign_cells: 格比目标少 ⇒ 报错', ok)

    # 几何约束：同一组格奇偶一致（同一组斜线）、两两格心 ≥ 60·√2 mm。
    # 每块半对角 20.9 mm ⇒ 相邻斜格也放得下，但轮级映射只选这四个角 + 心。
    parity = {((c // 3) + (c % 3)) % 2 for c in cm.DEFAULT_CELLS}
    check('默认格奇偶一致（同一组斜线）', parity == {0}, str(parity))
    dmin = min(math.hypot(cm.cell_center(a)[0] - cm.cell_center(b)[0],
                          cm.cell_center(a)[1] - cm.cell_center(b)[1])
               for i, a in enumerate(cm.DEFAULT_CELLS)
               for b in cm.DEFAULT_CELLS[i + 1:])
    check('默认格两两 ≥ 84.85 mm', dmin >= 60e-3 * math.sqrt(2) - 1e-12,
          '%.2f mm' % (dmin * 1000.0))


# ------------------------------------------------------------------ [3]
def section_free():
    print('\n[3] 自由区：不碰拾取区、不压垫、在桌上')
    pts = cm.free_destinations()
    xz = cm.PICK_ZONE
    check('候选点至少 12 个', len(pts) >= 12, str(len(pts)))
    check('全部在拾取区外',
          all(not (xz[0] <= x <= xz[1] and xz[2] <= y <= xz[3]) for x, y in pts))
    check('全部离垫 ≥ 80 mm（垫半宽 150 mm）',
          all(abs(x - cm.PAD_CENTER[0]) > 0.15 + 0.08 or
              abs(y - cm.PAD_CENTER[1]) > 0.15 + 0.08 for x, y in pts))
    check('全部在桌面边界内',
          all(cm.TABLE_XMIN + 0.03 <= x <= cm.TABLE_XMAX - 0.03 and
              cm.TABLE_YMIN + 0.03 <= y <= cm.TABLE_YMAX - 0.03 for x, y in pts))
    d = [math.hypot(x - cm.FREE_ANCHOR[0], y - cm.FREE_ANCHOR[1]) for x, y in pts]
    check('按离锚点距离升序', d == sorted(d))


# ------------------------------------------------------------------ [4]
def section_blocks():
    print('\n[4] 块位：ghost 保持身份、放垫上抬实测垫厚')
    b = s4_blocks(0)[0]
    g = cm.ghost(b, 0.5, 0.55)
    check('ghost：id / size / yaw 不变、只动 xyz',
          g['id'] == b['id'] and g['size'] == b['size'] and g['yaw'] == b['yaw'])
    check('ghost：落台面时块底 z = 0',
          abs(g['z'] - cm.BLOCK_M / 2.0) < 1e-12, str(g['z']))
    p = cm.place_in_cell(b, 7)
    cx, cy = cm.cell_center(7)
    check('放宫格：块心 = 格心', abs(p['x'] - cx) < 1e-12
          and abs(p['y'] - cy) < 1e-12)
    check('放宫格：块底 = 实测垫厚 %.3f mm' % (cm.PAD_THICK * 1000.0),
          abs(p['z'] - (cm.PAD_THICK + cm.BLOCK_M / 2.0)) < 1e-12, str(p['z']))


# ------------------------------------------------------------------ [5]
def section_terminal():
    print('\n[5] 终态合法性：接触合法、穿插非法、掉桌/出界/悬空各有各的原因')
    blocks = s4_blocks(0)
    rep = cm.terminal_report(blocks)
    check('S4:0 原始布局（塔 + 两对贴面）→ 合法', rep['ok'],
          json.dumps(rep['overlap_pairs'], ensure_ascii=False))
    check('原始布局 0 穿插、0 掉桌、0 出桌',
          rep['overlaps'] == 0 and rep['off_table'] == 0
          and rep['out_of_bounds'] == 0)
    check('9 块全部 legal', len(rep['blocks']) == 9
          and all(r['legal'] for r in rep['blocks']))

    # 贴面块：separation ≈ 0 但不判穿插
    a, bb = blocks[1], blocks[2]
    sep = sm.separation(sm.block_obb(a, 0.0), sm.block_obb(bb, 0.0))
    check('S4:0 的贴面对 separation ≈ 0（−0.2 mm 容差内）',
          abs(sep) <= cm.PEN_TOL, str(sep))

    # 真穿插：把 2 号块往 1 号块里推 5 mm
    bad = [dict(x) for x in blocks]
    bad[2]['x'] = a['x'] + (bb['x'] - a['x']) * 0.8
    bad[2]['y'] = a['y'] + (bb['y'] - a['y']) * 0.8
    rep2 = cm.terminal_report(bad)
    check('推进 5 mm → 判穿插（overlaps ≥ 1、整桌不合法）',
          not rep2['ok'] and rep2['overlaps'] >= 1, str(rep2['overlaps']))

    # 掉下桌
    bad = [dict(x) for x in blocks]
    bad[0]['z'] = -0.1
    rep3 = cm.terminal_report(bad)
    check('块底 −100 mm → 掉桌（off_table ≥ 1、整桌不合法）',
          not rep3['ok'] and rep3['off_table'] >= 1)

    # 悬空
    bad = [dict(x) for x in blocks]
    bad[5]['z'] = cm.BLOCK_M / 2.0 + 0.05
    rep4 = cm.terminal_report(bad)
    check('离台面 50 mm → 悬空（legal=false、off_table 不涨）',
          not rep4['ok'] and rep4['off_table'] == 0
          and any('悬空' in r['reasons'] for r in rep4['blocks']))

    # 出界
    bad = [dict(x) for x in blocks]
    bad[6]['x'] = cm.TABLE_XMAX + 0.02
    rep5 = cm.terminal_report(bad)
    check('出桌边 20 mm → 出界（out_of_bounds ≥ 1、整桌不合法）',
          not rep5['ok'] and rep5['out_of_bounds'] >= 1)

    # 「有支撑」必须是**那块真的在它下面**，不是「高度对上了」。
    # S4:0 的塔 = 0（底）+ 8（顶）；把 0 挪到垫上，8 留在原处必须判悬空。
    moved = [cm.place_in_cell(b, 2) if b['id'] == 0 else dict(b)
             for b in blocks]
    rep_t = cm.terminal_report(moved)
    top = [r for r in rep_t['blocks'] if r['id'] == 8][0]
    check('把塔底挪走、塔顶留在原处 → 塔顶悬空、整桌不合法',
          not rep_t['ok'] and '悬空' in top['reasons'], json.dumps(top))

    # 高度对齐不算支撑：把孤立块 7 抬到塔顶层高度，它离所有块 ≥50 mm。
    bad = [dict(x) for x in blocks]
    bad[7]['z'] = cm.BLOCK_M / 2.0 + 0.0295   # z_bottom 恰好 = 第 0 层块顶
    rep_f = cm.terminal_report(bad)
    row7 = [r for r in rep_f['blocks'] if r['id'] == 7][0]
    check('孤立块抬到塔顶层高度 → 悬空（不能被远处的塔底「支撑」）',
          not rep_f['ok'] and '悬空' in row7['reasons'], json.dumps(row7))

    # 放垫上：合法格 —— 用**孤立块**（7 号）。拿塔底块会把塔顶留在半空，
    # 那是「悬空」，真实流程里会先把塔顶挪走（这正是 E9 要规划的）。
    tgt = cm.place_in_cell(blocks[7], 2)
    rest = [b for b in blocks if b['id'] != blocks[7]['id']]
    rep6 = cm.terminal_report(rest + [tgt])
    cells = [r['cell'] for r in rep6['blocks'] if r['id'] == tgt['id']]
    check('目标放进宫格 2 → zone=slot、整桌合法',
          rep6['ok'] and cells == [2]
          and [r for r in rep6['blocks'] if r['id'] == tgt['id']][0]['zone'] == 'slot')

    # 垫上但压在格线上
    tgt2 = dict(tgt)
    u = cm.PAD_PITCH / 2.0
    ca, sa = math.cos(cm.PAD_YAW_RAD), math.sin(cm.PAD_YAW_RAD)
    tgt2['x'] = cm.PAD_CENTER[0] + ca * u
    tgt2['y'] = cm.PAD_CENTER[1] + sa * u
    rep7 = cm.terminal_report([tgt2])
    check('垫上但压在两格之间 → 压格、不合法',
          not rep7['ok'] and any('压格' in r['reasons'] for r in rep7['blocks']))


# ------------------------------------------------------------------ [6]
def section_blockers():
    print('\n[6] 遮挡归属：只数「夹爪真的碰到」的位姿')
    rows = [
        {"verdict": "gripper_collide", "per_other": {"0": 0.0, "1": 0.02}},
        {"verdict": "gripper_collide", "per_other": {"0": -0.003, "1": 0.01}},
        {"verdict": "gripper_collide", "per_other": {"2": 0.0}},
        {"verdict": "ok", "per_other": {"0": 0.005}},
    ]
    s = cm.blocker_scores(rows)
    check('挡了 2 个位姿的排第一，且最负间隙被记下',
          list(s)[0] == "0" and s["0"]["n"] == 2
          and abs(s["0"]["worst_mm"] + 3.0) < 1e-9, json.dumps(s))
    check('间隙 > 0 的不算遮挡', "1" not in s)
    check('ok 位姿不算遮挡', s.get("0", {}).get("n") == 2)
    check('pick_blocker 跳过排除项',
          cm.pick_blocker(s, exclude=("0",)) == "2"
          and cm.pick_blocker(s, exclude=("0", "2")) is None)

    # 第二种遮挡：压在目标上的块 —— 不先挪它，一动目标它就掉。
    blocks = s4_blocks(0)
    check('dependents：塔底 0 认出塔顶 8', cm.dependents(blocks, 0) == [8])
    check('dependents：塔顶 8 顶上没有块', cm.dependents(blocks, 8) == [])
    check('dependents：孤立块 7 没有依赖块', cm.dependents(blocks, 7) == [])
    check('dependents：未知 id 返回空', cm.dependents(blocks, 'nope') == [])
    # 探针递归里 bid 是 blocker_scores 的 str 键 —— 不能因为类型不同就漏依赖
    check('dependents：str 形式的塔底 id 也认塔顶',
          cm.dependents(blocks, '0') == [8])
    moved = [cm.place_in_cell(b, 2) if b['id'] == 8 else dict(b)
             for b in blocks]
    check('dependents：先把塔顶 8 挪到垫上后，塔底 0 顶面空了',
          cm.dependents(moved, 0) == [])


# ------------------------------------------------------------------ [7]
def section_mutation():
    print('\n[7] 变异：把「接触 vs 穿插」的口径改坏，用例必须红')
    blocks = s4_blocks(0)
    real = cm.terminal_report(blocks)
    mut = _mut('if sep < -pen_tol:', 'if sep <= 0.0:')
    # 变异模块是重新 exec 的一份源码 ⇒ import 时回到「未实测」哨兵；
    # 它和正身一样跑模型世界的用例，显式装模型常量（C1）。
    mut['install_model_pad_defaults']()
    bad = mut['terminal_report'](blocks)
    check('变异「接触也算重叠」：真实合法、改坏后贴面被判穿插',
          real['ok'] and not bad['ok'] and bad['overlaps'] >= 1,
          str(bad['overlaps']))

    # 容差判反了：把「离得近」当成「穿插」（接触对就被判红）
    mut = _mut('if sep < -pen_tol:', 'if sep < 0.001:')
    bad = mut['terminal_report'](blocks)
    check('变异「间隙 < 1 mm 也算穿插」：真实合法、改坏后贴面被判穿插',
          real['ok'] and not bad['ok'])

    # 整桌判绿的条件漏掉「每块合法」：掉桌的块也会被判合法
    dropped = [dict(x) for x in blocks]
    dropped[0]['z'] = -0.1
    mut = _mut('ok = (overlaps == 0 and n_bad == 0)', 'ok = (overlaps == 0)')
    check('变异「整桌只看穿插」：真实掉桌判红、改坏后判绿',
          not cm.terminal_report(dropped)['ok']
          and mut['terminal_report'](dropped)['ok'])

    # 支撑判定退回旧版（只查「不穿插」，不查真的接触）：
    # 塔顶被单独留下时会被远处的块「支撑」，整桌假绿。
    orphan = [cm.place_in_cell(b, 2) if b['id'] == 0 else dict(b)
              for b in blocks]
    mut = _mut('if sm.separation(obbs[k], obbs[j]) <= 0.001:',
               'if sm.separation(obbs[k], obbs[j]) >= -pen_tol:')
    check('变异「支撑只看高度对齐」：真实塔顶悬空判红、改坏后判绿',
          not cm.terminal_report(orphan)['ok']
          and mut['terminal_report'](orphan)['ok'])


# ------------------------------------------------------------------ [8]
def section_pad_pose():
    print('\n[8] 垫位姿：每场实测 —— 计划与判分读同一份，缺文件就停')
    import tempfile
    gs = cm._GS
    tmp = tempfile.mkdtemp(prefix='padpose-')
    real_center, real_yaw = (442.3, -197.9), -91.07
    grid = {'kind': 'stage2-grid', 'pitch_mm': cm.PAD_PITCH * 1000.0,
            'grid_center_base_mm': list(real_center),
            'grid_yaw_base_deg': real_yaw,
            'checks': {'n_frames': 3, 'rms_adopted_px': 0.84}}
    gpath = os.path.join(tmp, 'grid-pre.json')
    with open(gpath, 'w', encoding='utf-8') as f:
        json.dump(grid, f)

    try:
        # C1：缺省 = 未实测哨兵（不是模型常量）—— 回到未实测态再断言
        cm.reset_pad()
        check('默认垫参数 = 未实测哨兵（没有隐式模型回退）',
              'PAD_UNMEASURED' in repr(cm.PAD_THICK)
              and '未实测' in cm.PAD_POSE_SOURCE, cm.PAD_POSE_SOURCE)
        check('未实测哨兵当坐标用 ⇒ 抛（不是悄悄给个格号）',
              _raises_pad_not_measured(cm.cell_index_of, 0.0, 0.0))

        doc = cm.load_grid(gpath)
        check('load_grid 读入实测垫心',
              abs(cm.PAD_CENTER[0] * 1000 - real_center[0]) < 1e-9 and
              abs(cm.PAD_CENTER[1] * 1000 - real_center[1]) < 1e-9,
              str(cm.PAD_CENTER))
        check('load_grid 读入实测 yaw',
              abs(math.degrees(cm.PAD_YAW_RAD) - real_yaw) < 1e-9,
              str(math.degrees(cm.PAD_YAW_RAD)))
        check('load_grid 记下来源（不再自称模型常量）',
              cm.PAD_POSE_SOURCE == os.path.abspath(gpath), cm.PAD_POSE_SOURCE)
        check('load_grid 原样返回文档', doc is not None and doc['kind'] == 'stage2-grid')

        # 换了垫位姿之后，格心/反查必须整体跟着换
        check('第 4 格 = 实测垫心',
              all(abs(a - b) < 1e-12 for a, b in zip(cm.cell_center(4), cm.PAD_CENTER)))
        check('重配后格心反查 = 自己',
              all(cm.cell_index_of(*cm.cell_center(i)) == i for i in range(9)))

        # 判分端与计划端（gen_scene 的 SDF）必须落到同一批坐标
        gs.load_grid(gpath)
        check('gen_scene 与判分端读到同一垫心', gs.PAD_CENTER_MM ==
              (real_center[0], real_center[1]), str(gs.PAD_CENTER_MM))
        check('gen_scene 与判分端读到同一 yaw',
              abs(gs.PAD_YAW_DEG - real_yaw) < 1e-9, str(gs.PAD_YAW_DEG))
        sdf = gs._pad_grid_sdf()
        worst = 0.0
        n_found = 0
        for i in range(9):
            r, c = divmod(i, 3)
            m = re.search(r'<visual name="cell_%d%d">.*?<pose>([-\d.]+) ([-\d.]+) '
                          r'[-\d.]+ 0 0 ([-\d.]+)</pose>' % (r, c), sdf, re.S)
            if not m:
                continue
            n_found += 1
            px, py = float(m.group(1)), float(m.group(2))
            cx, cy = cm.cell_center(i)
            worst = max(worst, math.hypot(px - cx, py - cy) * 1000.0)
        check('SDF 里 9 个宫格', n_found == 9, str(n_found))
        check('SDF 宫格心 = 判分格心（≤0.01 mm）', worst < 0.01, '%.4f mm' % worst)

        # 计划侧落盘的 scene_truth.json 必须带上实测标记
        out = os.path.join(tmp, 'scene')
        gs.write_scene('S0', 0, out)
        truth = json.load(open(os.path.join(out, 'scene_truth.json'), encoding='utf-8'))
        check('scene_truth 记录实测垫位姿',
              truth['pad_pose']['measured'] is True and
              abs(truth['pad_pose']['yaw_deg'] - real_yaw) < 1e-9,
              str(truth['pad_pose']))

        # fail-closed：缺文件 / 不是 stage2-grid / 缺字段 一律抛
        for name, payload in (
                ('非 stage2-grid', {'kind': 'something-else',
                                    'grid_center_base_mm': [0, 0],
                                    'grid_yaw_base_deg': 0}),
                ('缺 grid_center_base_mm', {'kind': 'stage2-grid',
                                            'grid_yaw_base_deg': 0}),
                ('缺 grid_yaw_base_deg', {'kind': 'stage2-grid',
                                          'grid_center_base_mm': [0, 0]})):
            p = os.path.join(tmp, 'bad.json')
            with open(p, 'w', encoding='utf-8') as f:
                json.dump(payload, f)
            raised = False
            try:
                cm.load_grid(p)
            except ValueError:
                raised = True
            check('%s → 报错停下' % name, raised and cm.PAD_CENTER ==
                  (real_center[0] / 1000.0, real_center[1] / 1000.0))
        raised = False
        try:
            cm.load_grid(os.path.join(tmp, 'nope.json'))
        except (IOError, OSError):
            raised = True
        check('缺文件 → 报错停下', raised)

        # find_grid：grid-pre.json 优先于 grid.json
        open(os.path.join(tmp, 'grid.json'), 'w').close()
        check('find_grid 优先 grid-pre.json',
              cm.find_grid(tmp) == os.path.join(tmp, 'grid-pre.json'),
              str(cm.find_grid(tmp)))
        os.remove(os.path.join(tmp, 'grid-pre.json'))
        check('没有 grid-pre.json 时退到 grid.json',
              cm.find_grid(tmp) == os.path.join(tmp, 'grid.json'))
        check('空目录 → None', cm.find_grid(os.path.join(tmp, 'empty')) is None)
    finally:
        cm.reset_pad()
        gs.reset_pad()

    # C1（2026-11-13）：reset_pad 的语义改成「回到未实测哨兵」—— 任何数值
    # 使用当场抛 PadNotMeasured；模型常量必须显式 install_model_pad_defaults()
    # 才回来（真机不许走它）。计划端 gen_scene 自己的 reset_pad 仍是模型常量
    # （那是仿真世界的家，两边语义从此有意不同）。
    check('reset_pad ⇒ 未实测哨兵（repr 可见）',
          'PAD_UNMEASURED' in repr(cm.PAD_THICK), repr(cm.PAD_THICK))
    try:
        float(cm.PAD_THICK)
    except cm.PadNotMeasured:
        check('哨兵被当数字用 ⇒ 抛 PadNotMeasured（带指引）', True)
    else:
        check('哨兵被当数字用 ⇒ 抛 PadNotMeasured（带指引）', False, '竟然能算')
    cm.install_model_pad_defaults()
    check('install_model_pad_defaults 显式装回模型常量',
          cm.PAD_CENTER == cm._PAD_MODEL_CENTER
          and cm.PAD_YAW_RAD == cm._PAD_MODEL_YAW_RAD
          and '显式安装' in cm.PAD_POSE_SOURCE,
          (cm.PAD_CENTER, cm.PAD_POSE_SOURCE))
    check('装回后 gen_scene 默认值仍与判分端同步',
          math.degrees(cm.PAD_YAW_RAD) == gs.PAD_YAW_DEG and
          tuple(v * 1000.0 for v in cm.PAD_CENTER) == gs.PAD_CENTER_MM)


def main():
    print('=' * 68)
    # C1：模块缺省 = 未实测哨兵；本套件全是**模型世界**的几何测试，
    # 夹具显式装模型常量（真机流程走 load_grid，不经过这里）。
    cm.install_model_pad_defaults()
    section_gen_scene_sync()
    section_cells()
    section_cell_plan()
    section_free()
    section_blocks()
    section_terminal()
    section_blockers()
    section_mutation()
    section_pad_pose()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
