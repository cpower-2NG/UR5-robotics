#!/usr/bin/env python3
"""摆位引导（腕部照片叠目标）的离线自检：投影 / 足迹 / 出画幅 / 回原位计划。

为什么值得单测：这张图**不是测量装置，是人照着摆块的引导**。最坏的失败是
「框画在了别处而人照着摆了整盘」—— 图看起来完全正常，到判分才暴露。所以这里用
一台**解析可算**的假相机（正下视、已知焦距）把投影钉死，再钉住三个行为：

- 出画幅必须被标出来（否则那张照片引导不了那几块）；
- 回原位计划的终点必须**正好**是原起始关节（不是近似）；
- 没有任何输入时 rc=2 且**不写文件**（fail-closed）。

跑法：python3 scripts/test_stage4_placement_view.py
"""
import json
import math
import os
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from PIL import Image                                             # noqa: E402

import stage4_placement_view as pv                                # noqa: E402
import stage4_layouts as lay                                      # noqa: E402

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


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def section(title):
    print()
    print('== %s ==' % title)


# 假相机：正下视，相机轴 = base（x→u、y→−v、z→−z_base），尺寸贴近真机
F_PX = 908.152
CX, CY = 639.435, 363.314
CAM_XY = (0.4125, 0.295)       # 拾取区中心（米）
CAM_H = 0.500                  # 米
K = np.array([[F_PX, 0.0, CX], [0.0, F_PX, CY], [0.0, 0.0, 1.0]])
R = np.diag([1.0, -1.0, -1.0])
T = np.array([CAM_XY[0], CAM_XY[1], CAM_H])
W, H = 1280, 720
PPM = F_PX / (CAM_H * 1000.0)  # 每毫米多少像素


def blk(i, color, x_m, y_m, yaw=0.0):
    return {"id": i, "color": color, "x": x_m, "y": y_m, "yaw": yaw}


def main():
    size = lay.BLOCK_SIZE_M * 1000.0

    # ------------------------------------------------------------------
    section('A. 投影（解析可算的假相机）')
    uv = pv.project_mm((CAM_XY[0] * 1000.0, CAM_XY[1] * 1000.0), 0.0, K, R, T)
    check('A1 光心正下方 → 主点', uv is not None and
          close(uv[0], CX, 1e-6) and close(uv[1], CY, 1e-6), repr(uv))
    uv = pv.project_mm((CAM_XY[0] * 1000.0 + 100.0, CAM_XY[1] * 1000.0), 0.0,
                       K, R, T)
    check('A2 base x +100 mm → u +100·px/mm',
          close(uv[0] - CX, 100.0 * PPM, 1e-6) and close(uv[1], CY, 1e-6),
          repr(uv))
    uv = pv.project_mm((CAM_XY[0] * 1000.0, CAM_XY[1] * 1000.0 + 100.0), 0.0,
                       K, R, T)
    check('A3 base y +100 mm → v −100·px/mm（图上朝上）',
          close(uv[1] - CY, -100.0 * PPM, 1e-6), repr(uv))
    check('A4 相机后方 → None',
          pv.project_mm((0.0, 0.0), CAM_H * 1000.0 + 10.0, K, R, T) is None)
    a = pv.project_mm((CAM_XY[0] * 1000.0 + 100.0, CAM_XY[1] * 1000.0), 0.0,
                      K, R, T)
    b = pv.project_mm((CAM_XY[0] * 1000.0 + 100.0, CAM_XY[1] * 1000.0),
                      CAM_H * 500.0, K, R, T)          # 抬到相机与台面一半高
    check('A5 点抬高 ⇒ 偏移翻倍',
          close(b[0] - CX, 2.0 * (a[0] - CX), 1e-6), '%r vs %r' % (a, b))

    # ------------------------------------------------------------------
    section('B. 块足迹（米→毫米、角点、比例、出画幅）')
    recs = pv.block_targets([blk(7, 'red', 0.5, 0.3, 0.0)], K, R, T,
                            width=W, height=H)
    r = recs[0]
    check('B1 x/y 按米读进、转成毫米', close(r['x_mm'], 500.0) and
          close(r['y_mm'], 300.0), repr((r['x_mm'], r['y_mm'])))
    check('B2 四个角都投出来了', len(r['corners_uv']) == 4 and
          all(u is not None for u in r['corners_uv']))
    cx = sum(u[0] for u in r['corners_uv']) / 4.0
    cy = sum(u[1] for u in r['corners_uv']) / 4.0
    check('B3 中心 = 四角平均', close(r['center_uv'][0], cx, 1e-2) and
          close(r['center_uv'][1], cy, 1e-2), repr(r['center_uv']))
    check('B4 px/m 与相机一致', close(r['px_per_mm'], PPM, 0.01),
          '%r vs %r' % (r['px_per_mm'], PPM))
    check('B5 在画幅内不标出画', not r['clipped'])
    far = pv.block_targets([blk(8, 'red', 1.5, 0.3, 0.0)], K, R, T,
                           width=W, height=H)[0]
    check('B6 远处块标出画幅', far['clipped'])
    yaw90 = pv.block_targets([blk(9, 'red', 0.5, 0.3, 90.0)], K, R, T,
                             width=W, height=H)[0]
    d0 = math.hypot(r['corners_uv'][0][0] - r['corners_uv'][1][0],
                    r['corners_uv'][0][1] - r['corners_uv'][1][1])
    d90 = math.hypot(yaw90['corners_uv'][0][0] - yaw90['corners_uv'][1][0],
                     yaw90['corners_uv'][0][1] - yaw90['corners_uv'][1][1])
    check('B7 转 90° 边还是边长', close(d0, d90, 1e-6) and
          close(d0, size * PPM, 0.01), '%r vs %r' % (d0, d90))

    # ------------------------------------------------------------------
    section('C. 拾取区网格')
    zone = (250.0, 575.0, 170.0, 420.0)
    lines = pv.zone_lines(zone, step_mm=100.0)
    n_border = sum(1 for _, _, k in lines if k == 'border')
    n_grid = sum(1 for _, _, k in lines if k == 'grid')
    check('C1 四条边', n_border == 4, repr(n_border))
    check('C2 网格条数（x:300/400/500，y:200/300/400）', n_grid == 6,
          repr(n_grid))
    ok = all(zone[0] - 1e-6 <= l[0][0] <= zone[1] + 1e-6 and
             zone[2] - 1e-6 <= l[0][1] <= zone[3] + 1e-6 and
             zone[0] - 1e-6 <= l[1][0] <= zone[1] + 1e-6 and
             zone[2] - 1e-6 <= l[1][1] <= zone[3] + 1e-6
             for l in lines)
    check('C3 所有线都落在拾取区内', ok)
    check('C4 step=0 只剩边框',
          sum(1 for l in pv.zone_lines(zone, 0) if l[2] == 'grid') == 0)

    # ------------------------------------------------------------------
    section('D. 标注文档')
    blocks = [blk(0, 'yellow', 0.30, 0.40, 10.0),
              blk(1, 'cyan', 0.55, 0.20, 80.0),
              blk(2, 'red', 1.60, 0.30, 0.0)]         # 最后一块出画幅
    doc = pv.build_doc(blocks, zone, K, R, T, burst='x', frame=3,
                       batch='S0', layout_id=1, width=W, height=H)
    check('D1 kind / 批次', doc['kind'] == 'stage4-placement-view' and
          doc['batch'] == 'S0' and doc['layout'] == 1)
    check('D2 网格都投出了像素', len(doc['grid']) == 10 and
          all(g['p0_uv'] is not None and g['p1_uv'] is not None
              for g in doc['grid']))
    check('D3 出画幅清单 = [2]', doc['clipped'] == [2], repr(doc['clipped']))
    check('D4 相机回读一致',
          close(doc['camera']['K'][0][0], F_PX) and
          close(doc['camera']['t_bc_m'][2], CAM_H))
    check('D5 块数不变', len(doc['blocks']) == 3)
    doc2 = pv.build_doc(blocks, zone, K, R, T)     # 不给宽高
    check('D6 不给宽高就不标出画幅', doc2['clipped'] == [], repr(doc2['clipped']))

    # 足迹框必须画在**块顶面**：块高 29.5 mm ⇒ 像面上放大 500/470.5，
    # 同一 (x, y) 的像素位置朝像心收 ~5.9% 半径。画在台面会让人摆偏（外圈 ~10 mm）。
    doc3 = pv.build_doc(blocks, zone, K, R, T, width=W, height=H,
                        block_z_mm=pv.BLOCK_TOP_MM)
    check('D7 缺省 block_z 沿用台面（旧行为不变）',
          close(doc['block_z_mm'], doc['table_z_mm']) and
          close(doc2['block_z_mm'], doc2['table_z_mm']))
    c0 = doc['blocks'][1]['center_uv']
    c3 = doc3['blocks'][1]['center_uv']
    r0 = math.hypot(c0[0] - CX, c0[1] - CY)
    r3 = math.hypot(c3[0] - CX, c3[1] - CY)
    check('D8 顶面框比台面框更远离像心 %.2f→%.2f px（比例 500/470.5）'
          % (r0, r3),
          close(r3 / r0, CAM_H / (CAM_H - pv.BLOCK_TOP_MM / 1000.0), tol=1e-3))
    check('D9 顶面框不与网格混层',
          close(doc3['grid'][0]['p0_uv'][0], doc['grid'][0]['p0_uv'][0]) and
          not close(c3[0], c0[0], tol=1.0))

    # ------------------------------------------------------------------
    section('E. 回原位计划（survey → 原起点）')
    qs = [0.1, -1.0, 1.2, -1.5, -1.5, -3.9]
    view = {"version": 2, "q_start": [0.0, -1.1, 1.1, -1.2, -1.5, -1.3],
            "waypoints": [
                {"name": "survey_transit", "q": [0.2, -1.5, 1.1, -1.2, -1.6, -1.4],
                 "duration": 10.0},
                {"name": "survey", "q": [0.2, -1.55, 1.3, -1.35, -1.6, -1.4],
                 "duration": 2.0}]}
    ret = pv.return_plan_doc(view, qs)
    check('E1 航点数 = 前向航点数（倒序 + 收尾）',
          len(ret['waypoints']) == 2, repr(len(ret['waypoints'])))
    check('E2 终点 q = 原起始关节',
          ret['waypoints'][-1]['q'] == [float(v) for v in qs],
          repr(ret['waypoints'][-1]['q']))
    check('E3 该计划的 q_start = 前向末航点',
          ret['q_start'] == view['waypoints'][-1]['q'])
    check('E4 先退 transit 再回起点',
          ret['waypoints'][0]['name'] == 'return-survey_transit' and
          ret['waypoints'][1]['name'] == 'return-start')
    check('E5 时长都为正',
          all(w['duration'] > 0 for w in ret['waypoints']))
    one = {"waypoints": [{"name": "survey", "q": [0.0] * 6, "duration": 2.0}]}
    ret1 = pv.return_plan_doc(one, [0.1] * 6)
    check('E6 单航点前向计划 → 只有 return-start',
          len(ret1['waypoints']) == 1 and
          ret1['waypoints'][0]['name'] == 'return-start' and
          ret1['q_start'] == [0.0] * 6)
    try:
        pv.return_plan_doc({"waypoints": []}, qs)
        check('E7 空航点前向计划要报错', False)
    except ValueError:
        check('E7 空航点前向计划要报错', True)

    # ------------------------------------------------------------------
    section('F. 画图冒烟（合成底图，不依赖仓外数据）')
    img = Image.new('RGB', (W, H), (128, 128, 128))
    out = pv.draw_doc(img, doc, cjk=False, title='t')
    check('F1 尺寸不变且是 RGB', out.size == (W, H) and out.mode == 'RGB',
          '%r %s' % (out.size, out.mode))
    out2 = pv.draw_doc(img, doc2, cjk=True, title='中文标题')
    check('F2 带中文标题也能画', out2.size == (W, H))

    # ------------------------------------------------------------------
    section('G. CLI（fail-closed / 写文件 / 出画幅 rc）')
    tmp = tempfile.mkdtemp(prefix='pv-test-')
    cam_json = os.path.join(tmp, 'cam.json')
    with open(cam_json, 'w', encoding='utf-8') as f:
        json.dump({"K": K.tolist(), "R_bc": R.tolist(),
                   "t_bc_m": T.tolist()}, f)
    png = os.path.join(tmp, 'view.png')
    img.save(png)

    rc = pv.main(['--image', png, '--batch', 'S0', '--layout', '1',
                  '--out', os.path.join(tmp, 'no-cam.png')])
    check('G1 只给 --image 不给 --camera-json ⇒ rc 2', rc == 2, repr(rc))

    rc = pv.main(['--burst', os.path.join(tmp, 'nope'),
                  '--batch', 'S0', '--layout', '1',
                  '--out', os.path.join(tmp, 'nope.png')])
    check('G2 burst 不存在 ⇒ rc 2', rc == 2, repr(rc))
    check('G3 rc 2 时没有写图',
          not os.path.exists(os.path.join(tmp, 'nope.json')))

    scene = os.path.join(tmp, 'scene.json')
    with open(scene, 'w', encoding='utf-8') as f:
        json.dump({"blocks": [{"id": 0, "color": 'tan', "x": 0.40,
                               "y": 0.30, "yaw": 0.0}]}, f)
    rc = pv.main(['--image', png, '--camera-json', cam_json,
                  '--scene', scene, '--out', os.path.join(tmp, 'in.png')])
    jp = os.path.join(tmp, 'in.json')
    check('G4 场景全在画幅内 ⇒ rc 0 且写出图+文档',
          rc == 0 and os.path.exists(os.path.join(tmp, 'in.png')) and
          os.path.exists(jp), repr(rc))
    with open(jp, encoding='utf-8') as f:
        d = json.load(f)
    check('G5 文档里 x 按米读对（0.40 m ⇒ 400 mm）',
          d['blocks'][0]['x_mm'] == 400.0, repr(d['blocks'][0]['x_mm']))
    check('G6 相机内参读回一致',
          close(d['camera']['K'][1][1], F_PX, 1e-6))

    with open(scene, 'w', encoding='utf-8') as f:
        json.dump({"blocks": [{"id": 0, "color": 'tan', "x": 2.0,
                               "y": 0.30, "yaw": 0.0}]}, f)
    rc = pv.main(['--image', png, '--camera-json', cam_json,
                  '--scene', scene, '--out', os.path.join(tmp, 'clip.png')])
    check('G7 有目标出画幅 ⇒ rc 1（仍写出，人能看见缺哪块）',
          rc == 1 and os.path.exists(os.path.join(tmp, 'clip.png')),
          repr(rc))

    try:
        pv.main(['--plan', '--out-dir', tmp])
        check('G8 --plan 缺 --urdf ⇒ 参数错误退出', False)
    except SystemExit:
        check('G8 --plan 缺 --urdf ⇒ 参数错误退出', True)
    try:
        pv.main(['--batch', 'S0', '--layout', '1'])
        check('G9 标注模式缺 --out ⇒ 参数错误退出', False)
    except SystemExit:
        check('G9 标注模式缺 --out ⇒ 参数错误退出', True)

    # ------------------------------------------------------------------
    print()
    print('%d passed, %d failed' % (N_PASS, N_FAIL))
    if FAILS:
        print('FAILED: %s' % ', '.join(FAILS))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
