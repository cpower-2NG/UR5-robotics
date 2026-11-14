#!/usr/bin/env python3
"""C2/C4 单源守卫：区域常量与腕部手眼各只许有一个家。

区域（C2）：拾取区的唯一出处 = ``gen_scene.ZONE``（经 stage4_clear_model）；
自由区 = ``stage4_clear_model.FREE_X/FREE_Y``。历史事故形态是「四处字面拷贝、
改区漏改一处」（2026-10-28 扩区就是一次多处手改）—— 这份测试钉住相等性，
再配两条源码级断言（字面量不许回苗）。

手眼（C4）：腕部相机的安装只认 ``docs/calib/wrist_eih_camlink-*.json`` 一份
（与 ``wrist_tf_publish`` 发 TF 同源）；规划侧的 tool0→光心平移由它复合驱动
内部 link→optical 偏置得出，不再自抄 —— 回归钉住合成值 = 2026-11-07 拟合值。
"""
import inspect
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np

import stage4_clear_model as cm
import stage4_clear_run as cr
import stage4_specify as sp
import stage4_wrist_survey as ws
import wrist_tf_publish as wtp

PASS = FAIL = 0


def check(name, ok, detail=''):
    global PASS, FAIL
    if ok:
        PASS += 1
        print('  PASS  %s' % name)
    else:
        FAIL += 1
        print('  FAIL  %s  ← %s' % (name, detail))


# ---------------------------------------------------------------- C2 区域
def test_zones_single_source():
    check('gen_scene.ZONE == cm.PICK_ZONE（cm 反查源码的那一份）',
          tuple(cm.PICK_ZONE) == (0.25, 0.575, 0.17, 0.42), cm.PICK_ZONE)
    check('wrist_survey.PICK_ZONE_M 同源',
          ws.PICK_ZONE_M == tuple(cm.PICK_ZONE),
          (ws.PICK_ZONE_M, cm.PICK_ZONE))
    check('specify.ZONE_M 同源',
          sp.ZONE_M == tuple(cm.PICK_ZONE), (sp.ZONE_M, cm.PICK_ZONE))
    views = {v['name']: v for v in cr.E9_VIEWS}
    check('E9_VIEWS[pick].zone 同源',
          tuple(views['pick']['zone']) == tuple(cm.PICK_ZONE),
          views['pick']['zone'])
    check('E9_VIEWS[free].zone 同源（cm.FREE_X/FREE_Y）',
          tuple(views['free']['zone']) == (cm.FREE_X[0], cm.FREE_X[1],
                                           cm.FREE_Y[0], cm.FREE_Y[1]),
          views['free']['zone'])

    # 源码级：字面拷贝不许回苗（变异 = 把某一处改回字面量 ⇒ 这里红）。
    # 2026-11-14 审计：scan 名单原来只有 specify + wrist_survey 两个模块，
    # joint_calib / v1_sim / precision_round 三处字面拷贝不在守卫范围。
    import stage4_joint_calib as jc
    import stage4_v1_sim as v1
    check('joint_calib.PICK_ZONE 同源（cm.PICK_ZONE）',
          list(jc.PICK_ZONE) == list(cm.PICK_ZONE),
          (jc.PICK_ZONE, cm.PICK_ZONE))
    check('v1_sim.PICK_ZONE_M 同源（cm.PICK_ZONE）',
          tuple(v1.PICK_ZONE_M) == tuple(cm.PICK_ZONE),
          (v1.PICK_ZONE_M, cm.PICK_ZONE))
    import precision_round as prnd
    check('precision_round 已无同名 PICK_ZONE（改名 PATROL_ZONE，语义不同）',
          not hasattr(prnd, 'PICK_ZONE') and prnd.PATROL_ZONE ==
          (0.38, 0.62, 0.25, 0.45), getattr(prnd, 'PICK_ZONE', None))
    for mod, path in (('stage4_specify', sp), ('stage4_wrist_survey', ws),
                      ('stage4_joint_calib', jc), ('stage4_v1_sim', v1)):
        src = inspect.getsource(path)
        check('%s 源码里没有拾取区字面拷贝' % mod,
              '(0.25, 0.575' not in src and '0.575, 0.17' not in src)


# ---------------------------------------------------------------- C4 手眼
def test_handeye_single_source():
    t_old = np.array([-0.031924, -0.085321, 0.024256])
    check('tool0→光心 = 2026-11-07 拟合值（回归钉，atol 1e-12）',
          np.allclose(ws.T_TOOL0_CAM_M, t_old, atol=1e-12),
          ws.T_TOOL0_CAM_M)
    doc = wtp.parse_calib(wtp.load_calib(wtp.DEFAULT_CALIB))
    check('与 wrist_tf_publish 同一份标定文件（tool0→camera_link）',
          doc['parent'] == 'tool0' and doc['child'] == 'camera_link')
    t_tl = np.array(doc['t'])
    check('标定文件平移 ≠ 合成值（差 = 驱动 link→optical ≈14.7 mm，两帧不同）',
          abs(float(np.linalg.norm(ws.T_TOOL0_CAM_M - t_tl)) - 0.014737)
          < 2e-5,
          float(np.linalg.norm(ws.T_TOOL0_CAM_M - t_tl)))
    check('规划模块引用的就是那份文件',
          os.path.samefile(ws.WristCALIB_PATH, wtp.DEFAULT_CALIB),
          (ws.WristCALIB_PATH, wtp.DEFAULT_CALIB))


if __name__ == '__main__':
    print('[C2/C4] 坐标系常量单一出处守卫')
    test_zones_single_source()
    test_handeye_single_source()
    print('\n%d PASS, %d FAIL' % (PASS, FAIL))
    sys.exit(1 if FAIL else 0)
