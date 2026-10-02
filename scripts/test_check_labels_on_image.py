#!/usr/bin/env python3
"""``check_labels_on_image.py`` 的自检 —— 不需要相机、不需要板子。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得钉
============
这个脚本是采集流水线的**守门员**：它的职责是在错标签进数据集之前喊停。
它错成两边的代价不对称 ——
- **漏报**（该喊没喊）：错标签静默入库，训出来的模型「看着还行、就是抓不准」；
- **误报**（不该喊喊了）：好数据被拦下，人还得回头查。

所以第 [3] 节造了两张图：一张方块**正好**画在预测位置上（必须放行），
一张整体挪 35 px（必须拦）。只测一边等于没测。

第 [2] 节钉的是判据本身：中位数要扛得住少量幽灵块，但不能被整体错位蒙过去。
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_labels_on_image as C                                  # noqa: E402

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
        print('  FAIL  %s%s' % (name, ('   ← %s' % detail) if detail else ''))


def main():
    # ------------------------------------------- [1] nearest_distances（纯函数）
    print('\n[1] nearest_distances —— 每点到最近中心的距离')
    d = C.nearest_distances([[0, 0], [10, 0], [0, 100]], [[0, 0], [10, 0]])
    check('[0,0] 到最近的 [0,0] = 0', abs(d[0] - 0.0) < 1e-9, '%r' % d[0])
    check('[10,0] 到最近的 [10,0] = 0', abs(d[1] - 0.0) < 1e-9, '%r' % d[1])
    check('[0,100] 到最近的 [0,0] = 100', abs(d[2] - 100.0) < 1e-9, '%r' % d[2])
    check('空点集 → 空数组', len(C.nearest_distances([], [[0, 0]])) == 0)
    check('空中心集 → 空数组', len(C.nearest_distances([[0, 0]], [])) == 0)
    check('(N,2) 与 (M,2) 混着传也对',
          np.allclose(C.nearest_distances(np.array([[3.0, 4.0]]), [(0, 0)]), [5.0]))

    # ------------------------------------------------------------ [2] 判据
    print('\n[2] verdict —— 中位数要扛幽灵块，但不能被整体错位蒙过去')
    ok, why = C.verdict([1, 2, 3, 4, 5])
    check('全部贴合（中位 3）→ 通过', ok, why)
    ok, why = C.verdict([3.0] * 20)
    check('整齐的 3 px → 通过', ok, why)
    ok, why = C.verdict([30.0] * 20)
    check('整齐的 30 px（整体错位）→ **不通过**', not ok, why)

    # 中位数的意义：20 块里混进 4 块幽灵（距离很大），整体没动 —— 仍该通过。
    d = [2.0] * 16 + [500.0] * 4
    ok, why = C.verdict(d)
    check('20 块里 4 块是幽灵（500 px）→ 中位数仍是 2，通过', ok, why)
    check('  同一批数据若改用**均值**就会被带跑（说明为什么用中位数）',
          float(np.mean(d)) > 15.0, '均值 %.1f' % np.mean(d))

    ok, why = C.verdict([1, 2])
    check('块数太少（2 < 3）→ 不下结论', not ok, why)
    ok, why = C.verdict([1, 2, 3], tol_px=2.0)
    check('阈值调紧到 2 → 中位 2 仍通过（<= 是闭区间）', ok, why)

    # --------------------------------- [3] 端到端：合成图，方块画在预测位置上
    print('\n[3] 端到端 —— 方块画在预测位置上必须放行；整体挪 35 px 必须拦')
    import cv2
    with tempfile.TemporaryDirectory() as td:
        info = {'width': 1280, 'height': 720, 'fx': 600.0, 'fy': 600.0,
                'cx': 640.0, 'cy': 360.0}
        # 相机在靶系 (0,0,-1)，朝 +z 看（与 TfProjector 的 p_target = R·p_cam + t 一致）
        extr = {'target_frame': 'board', 'camera_frame': 'cam',
                'translation': [0.0, 0.0, -1.0], 'rotation': [0.0, 0.0, 0.0, 1.0]}
        blocks = [{'x': x, 'y': y, 'z': 0.0, 'size': 0.03}
                  for x in (-0.15, 0.0, 0.15) for y in (-0.15, 0.0, 0.15)]
        for nm, obj in (('info.json', info), ('extr.json', extr),
                        ('blocks.json', blocks)):
            json.dump(obj, open(os.path.join(td, nm), 'w'))
        boxes = C.predicted_boxes(blocks, info, extr)
        centres = np.column_stack([(boxes[:, 0] + boxes[:, 2]) / 2,
                                   (boxes[:, 1] + boxes[:, 3]) / 2])
        side = int(np.sqrt(np.median((boxes[:, 2] - boxes[:, 0]) *
                                     (boxes[:, 3] - boxes[:, 1]))))

        def draw(shift_x, outdir):
            os.makedirs(outdir, exist_ok=True)
            for i in range(6):                    # 6 帧不同亮度，覆盖伸缩那一步
                img = np.full((720, 1280, 3), 90 + i * 20, np.uint8)
                for (cu, cv_) in centres:
                    x = int(cu + shift_x) - side // 2
                    y = int(cv_) - side // 2
                    cv2.rectangle(img, (x, y), (x + side, y + side),
                                  (0, 0, 255), -1)   # 纯红：饱和度高
                cv2.imwrite(os.path.join(outdir, 'img_%04d.jpg' % i), img)

        good, bad = os.path.join(td, 'good'), os.path.join(td, 'bad')
        draw(0, good)
        draw(35, bad)

        def run(images):
            return subprocess.run(
                [sys.executable,
                 os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'check_labels_on_image.py'),
                 '--images', images, '--blocks', os.path.join(td, 'blocks.json'),
                 '--camera-info', os.path.join(td, 'info.json'),
                 '--extrinsics', os.path.join(td, 'extr.json'),
                 '--sample', '3'],
                capture_output=True, text=True)

        r = run(good)
        check('方块与预测重合 → 退出码 0', r.returncode == 0,
              r.stdout.strip()[-300:] + r.stderr.strip()[-300:])
        check('  输出里说了「对得上」', '对得上' in r.stdout, r.stdout.strip()[-200:])
        r = run(bad)
        check('方块整体挪 35 px → 退出码 1', r.returncode == 1,
              '实际 %d；%s' % (r.returncode, r.stdout.strip()[-200:]))
        check('  输出里点名「不是同一个排布」',
              '不是同一个' in r.stdout or '对不上' in r.stdout,
              r.stdout.strip()[-200:])

        print('\n[3b] 反向：拿**别人的** blocks 去查同一批图，必须拦')
        # 这正是 2026-10-02 那次翻车的形状：图是对的，喂错了 blocks.json。
        shifted = [dict(b, x=b['x'] + 0.05) for b in blocks]
        json.dump(shifted, open(os.path.join(td, 'shifted.json'), 'w'))
        r = subprocess.run(
            [sys.executable,
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'check_labels_on_image.py'),
             '--images', good, '--blocks', os.path.join(td, 'shifted.json'),
             '--camera-info', os.path.join(td, 'info.json'),
             '--extrinsics', os.path.join(td, 'extr.json'), '--sample', '3'],
            capture_output=True, text=True)
        check('图没动、blocks 整体偏 0.05 m（=30 px）→ 退出码 1',
              r.returncode == 1, '实际 %d' % r.returncode)

        print('\n[3c] 坏输入要干净地报错，不能崩')
        r = subprocess.run(
            [sys.executable,
             os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'check_labels_on_image.py'),
             '--images', os.path.join(td, 'nope'), '--blocks',
             os.path.join(td, 'blocks.json'), '--camera-info',
             os.path.join(td, 'info.json'), '--extrinsics',
             os.path.join(td, 'extr.json')],
            capture_output=True, text=True)
        check('目录不存在 → 退出码 2', r.returncode == 2, '实际 %d' % r.returncode)
        check('  没有 Traceback', 'Traceback' not in r.stderr, r.stderr.strip()[:200])

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
