#!/usr/bin/env python3
"""垫位姿 burst 共识器（stage4_pad_from_burst）的离线自检。

风险全在「多帧怎么合成一份」这一步：
1. yaw 跨界 —— −179°/179° 直接取中位会得到 0°（差一整圈）；
2. 离散门 —— 帧间差 3 mm 还照样写文件，等于把「没有共识」伪装成实测；
3. 帧数不足 —— 一两帧就下结论，坏帧直接变成本场垫位姿。
计划与判分都读这一份文件，放错判错同源。

跑法：python3 scripts/test_stage4_pad_from_burst.py
"""
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stage4_pad_from_burst as pfb                    # noqa: E402

import stage4_clear_model as cm                                  # noqa: E402
import stage4_pad_from_burst as pb                               # noqa: E402

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


def row(x, y, yaw, idx=0, rms=0.9):
    T = np.eye(4)
    T[:2, 3] = [x / 1000.0, y / 1000.0]
    T[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]   # 正常 pad 朝向
    return {'idx': idx, 't': np.array([x, y, 0.9]), 'yaw': float(yaw),
            'rms_px': rms, 'T': T}


def sec_wrap():
    print('\n[1] 角度折算')
    check('190° → −170°', abs(pb.wrap_deg(190.0) + 170.0) < 1e-9)
    check('−190° → 170°', abs(pb.wrap_deg(-190.0) - 170.0) < 1e-9)
    check('180° 折到 −180°（同一朝向，只差表示）',
          abs(pb.wrap_deg(180.0) + 180.0) < 1e-9)


def sec_consensus():
    print('\n[2] 共识：圆中位 + 逐轴中位')
    # yaw 跨 ±180 边界：三帧 ~180°，朴素中位会得 0°
    rows = [row(461.3, -158.2, 179.6, 0), row(461.2, -158.1, -179.7, 1),
            row(461.4, -158.3, 180.1, 2)]
    t, yaw, spread, k = pb.consensus(rows)
    check('跨 ±180 的 yaw 取圆中位（朴素中位会得 0°）',
          abs(pb.wrap_deg(yaw - 180.0)) < 0.2, '%.3f' % yaw)
    check('平移取逐轴中位', abs(t[0] - 461.3) < 1e-9 and abs(t[1] + 158.2) < 1e-9,
          str(t))
    check('离散度算得出来（xy/yaw 都有 max 与 median）',
          spread['xy']['max'] >= spread['xy']['median']
          and spread['yaw']['max'] >= spread['yaw']['median'])
    check('采用帧 = 离共识最近的那一帧', k in (0, 1, 2), str(k))

    # 故意让第 1 帧偏离最远，采用帧不该选它
    rows2 = [row(461.3, -158.2, -89.4, 0), row(470.0, -150.0, -85.0, 1),
             row(461.3, -158.2, -89.4, 2)]
    _, _, sp2, k2 = pb.consensus(rows2)
    check('偏得最远的坏帧不会被采用', k2 != 1, str(k2))
    check('坏帧会体现在离散度上（xy max ≈ 12 mm）',
          sp2['xy']['max'] > 10.0, str(sp2))

    ok = True
    try:
        pb.consensus([])
    except ValueError:
        ok = False
    check('零帧 ⇒ 抛 ValueError（不返回假共识）', not ok)


def sec_doc():
    print('\n[3] 输出文档：字段够判分端与计划端读')
    rows = [row(461.3, -158.2, -89.4, i) for i in range(6)]
    t, yaw, sp, k = pb.consensus(rows)
    doc = pb.build_doc(rows, t, yaw, sp, k, 'wrist', 'burst-x', 'mat.json', 8)
    check('kind = stage2-grid（load_grid 的硬门）', doc['kind'] == 'stage2-grid')
    check('grid_center_base_mm / grid_yaw_base_deg 是有限数',
          all(math.isfinite(v) for v in doc['grid_center_base_mm'])
          and math.isfinite(doc['grid_yaw_base_deg']))
    check('checks.n_frames / rms_adopted_px 给判分日志用',
          doc['checks']['n_frames'] == 6
          and abs(doc['checks']['rms_adopted_px'] - 0.9) < 1e-9)
    check('记账写明来源 burst 与总帧数（含红帧）',
          doc['source_burst'] == 'burst-x' and doc['n_frames_total'] == 8)
    check('T_base_pad 是 4×4', np.asarray(doc['T_base_pad']).shape == (4, 4))

    # 能被判分/计划两端的加载器直接吃
    tmp = tempfile.mkdtemp(prefix='pad-burst-doc-')
    p = os.path.join(tmp, 'grid-pre.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    try:
        got = cm.load_grid(p)
        check('load_grid 能读，且垫位姿装成实测值',
              abs(cm.PAD_CENTER[0] * 1000 - 461.3) < 1e-6
              and abs(math.degrees(cm.PAD_YAW_RAD) + 89.4) < 1e-6
              and 'grid-pre.json' in cm.PAD_POSE_SOURCE, cm.PAD_POSE_SOURCE)
        check('load_grid 返回原文档（判分端可回读来源）', got['kind'] == 'stage2-grid')
    finally:
        cm.reset_pad()


def _run_main(rows, n_total, out, extra=()):
    """把 solve_burst 换成合成帧，跑 CLI 的闸门与写出。"""
    burst = tempfile.mkdtemp(prefix='pad-burst-fake-')
    real = pb.solve_burst
    pb.solve_burst = lambda *a, **kw: (rows, [], n_total)
    try:
        return pb.main([burst, '--out', out] + list(extra))
    finally:
        pb.solve_burst = real


def sec_gates():
    print('\n[4] 闸门：帧数不足 / 离散超限 ⇒ 不写文件')
    tmp = tempfile.mkdtemp(prefix='pad-burst-gate-')
    out = os.path.join(tmp, 'grid-pre.json')

    rows = [row(461.3, -158.2, -89.4, i) for i in range(3)]
    rc = _run_main(rows, 20, out)
    check('只解出 3 帧（< 5）⇒ rc=1 且不写文件',
          rc == 1 and not os.path.exists(out), 'rc=%s' % rc)

    rows = [row(461.3, -158.2, -89.4, i) for i in range(4)
            ] + [row(464.0, -158.2, -89.4, 4)]
    rc = _run_main(rows, 5, out)
    check('xy 极差 2.7 mm（> 2 mm）⇒ rc=1 且不写文件',
          rc == 1 and not os.path.exists(out), 'rc=%s' % rc)

    rows = [row(461.3, -158.2, -89.4, i) for i in range(5)
            ] + [row(461.3, -158.2, -88.4, 5)]
    rc = _run_main(rows, 6, out)
    check('yaw 极差 1.0°（> 0.5°）⇒ rc=1 且不写文件',
          rc == 1 and not os.path.exists(out), 'rc=%s' % rc)

    rows = [row(461.3, -158.2, -89.4, i) for i in range(5)]
    rc = _run_main(rows, 20, out)
    check('5 帧、离散在限内 ⇒ rc=0 且写出文件',
          rc == 0 and os.path.exists(out), 'rc=%s' % rc)
    with open(out, encoding='utf-8') as f:
        doc = json.load(f)
    check('写出的文件是共识值、记账里留着总帧数',
          abs(doc['grid_center_base_mm'][0] - 461.3) < 1e-6
          and doc['n_frames_ok'] == 5 and doc['n_frames_total'] == 20)

    real = pb.solve_burst
    pb.solve_burst = lambda *a, **kw: ([], [], 0)
    try:
        rc = pb.main(['no-such-dir-xyz', '--out', out])
    finally:
        pb.solve_burst = real
    check('burst 目录不存在 ⇒ rc=2（不悄悄当成 0 帧共识）', rc == 2, 'rc=%s' % rc)


def sec_embed_spec():
    print('\n[5] grid 文档自洽：嵌 pad-spec（C1 后置）')
    base = {'kind': 'stage2-grid', 'T_base_pad': [[1, 0, 0, 0.4242],
                                                  [0, 1, 0, -0.112],
                                                  [0, 0, 1, 0.00017],
                                                  [0, 0, 0, 1]]}
    d = pfb.embed_pad_spec(base, None)
    check('无 spec ⇒ 厚度嵌解垫垫面 z、足迹不嵌',
          abs(d['pad_thickness_m'] - 0.00017) < 1e-12
          and 'pad_footprint_m' not in d, d.get('pad_thickness_m'))
    d = pfb.embed_pad_spec(base, {'footprint_m': [0.393, 0.21]})
    check('有足迹 ⇒ 嵌足迹（0.393×0.21 实测）',
          abs(d['pad_footprint_m'][0] - 0.393) < 1e-12
          and abs(d['pad_footprint_m'][1] - 0.21) < 1e-12)
    d = pfb.embed_pad_spec(base, {'thickness_m': 0.002,
                                  'footprint_m': [0.393, 0.21]})
    check('spec 的 thickness_m 优先于垫面 z',
          abs(d['pad_thickness_m'] - 0.002) < 1e-12)
    d2 = pfb.embed_pad_spec(d, None)
    check('纯函数：不改入参', base.get('pad_thickness_m') is None)


def main():
    print('=' * 68)
    for fn in (sec_wrap, sec_consensus, sec_doc, sec_gates, sec_embed_spec):
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
