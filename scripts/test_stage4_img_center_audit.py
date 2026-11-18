#!/usr/bin/env python3
"""``stage4_img_center_audit`` 自检：差场模型（平移 / 绕竖直轴旋转）解法。

判据：
- 纯平移合成差场 ⇒ 平移模型残差 ~0、ε~0；
- 纯旋转合成差场 ⇒ 旋转模型残差 ~0、ε 复原（1e-6 rad 量级）；
- 平移+旋转合成 ⇒ 两者都复原；
- ``audit_truth`` 只吃带 ``img_xy_mm`` 的块，<3 块不给拟合。

跑法：python3 scripts/test_stage4_img_center_audit.py
"""
import math
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_img_center_audit as au                               # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('   ← %s' % (detail,)) if detail else ''))


def rot_field(P, eps):
    c0 = P.mean(axis=0)
    return np.stack([-eps * (P[:, 1] - c0[1]), eps * (P[:, 0] - c0[0])],
                    axis=1)


def main():
    P = np.array([[420, 230], [480, 300], [520, 240], [430, 400],
                  [540, 410], [490, 340]], float)

    print('[1] 纯平移')
    D = np.tile([2.0, -3.0], (len(P), 1))
    r = au.fit_models(P, D)
    check('平移复原', np.allclose(r['translation']['t_mm'], [2.0, -3.0],
                                  atol=1e-3), r['translation'])
    check('平移残差 ~0', r['translation']['rms_mm'] < 1e-6,
          r['translation']['rms_mm'])
    check('ε 可忽略', abs(r['rotation']['eps_rad']) < 1e-6,
          r['rotation']['eps_rad'])
    check('判为平移', r['better'] == 'translation', r['better'])

    print('[2] 纯旋转（0.01 rad ≈ 0.573°）')
    D2 = rot_field(P, 0.01)
    r2 = au.fit_models(P, D2)
    check('ε 复原', abs(r2['rotation']['eps_rad'] - 0.01) < 1e-6,
          r2['rotation']['eps_rad'])
    check('ε_deg ≈ 0.573', abs(r2['rotation']['eps_deg'] - 0.5729578) < 1e-3,
          r2['rotation']['eps_deg'])
    check('旋转残差 ~0', r2['rotation']['rms_mm'] < 1e-6,
          r2['rotation']['rms_mm'])
    check('判为旋转', r2['better'] == 'rotation', r2['better'])

    print('[3] 平移 + 旋转')
    D3 = rot_field(P, -0.004) + np.array([1.0, 0.5])
    r3 = au.fit_models(P, D3)
    check('t 复原', np.allclose(r3['rotation']['t_mm'], [1.0, 0.5],
                                atol=1e-6), r3['rotation']['t_mm'])
    check('ε 复原', abs(r3['rotation']['eps_rad'] + 0.004) < 1e-6,
          r3['rotation']['eps_rad'])
    check('残差 ~0', r3['rotation']['rms_mm'] < 1e-6,
          r3['rotation']['rms_mm'])

    print('[4] audit_truth：只吃 img_xy_mm 块；<3 块不拟合')
    doc = {"blocks": [
        {"color": "red", "x": 0.417, "y": 0.397, "meas": {
            "img_ok": True, "img_xy_mm": [419.7, 399.6], "img_frames": 9}},
        {"color": "cyan", "x": 0.405, "y": 0.231, "meas": {
            "img_ok": True, "img_xy_mm": [404.4, 234.5], "img_frames": 8}},
        {"color": "yellow", "x": 0.483, "y": 0.306, "meas": {
            "img_ok": False, "img_why": "edge support"}},
        {"color": "orange", "x": 0.534, "y": 0.233},
    ]}
    rep = au.audit_truth(doc)
    check('只留 2 个带图像读数的块', len(rep['blocks']) == 2,
          [r['color'] for r in rep['blocks']])
    check('<3 块 ⇒ fit=None', rep['fit'] is None, rep['fit'])
    r0 = rep['blocks'][0]
    check('delta 计算', abs(r0['delta_mm'][0] - 2.7) < 1e-6
          and abs(r0['delta_mm'][1] - 2.6) < 1e-6, r0['delta_mm'])
    check('img_frames 透传', r0['img_frames'] == 9, r0['img_frames'])

    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + '; '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
