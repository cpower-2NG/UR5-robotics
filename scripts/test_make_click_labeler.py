#!/usr/bin/env python3
"""「顶视方块点选」页面自检 —— 不需要 ROS、不需要真机、不需要画面。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
页面把**人点的那一个像素**换算成靶系坐标，用的是和
``propose_block_positions._rough_seed`` 同源的数学。它错起来不会崩、
不会报错，只会让**每一块方块的标签都偏同一个量** —— 训出来的模型
「看着还行、就是抓不准」，事后极难归因。上一轮已经在同一个坑里栽过一次
（`R.T` vs `R`，静默偏 80 mm），所以这一处必须钉死。

三节（编号即 ``main()`` 里的 ``[n]``）：

  1. **端到端**：造一份合成相机与外参，跑一遍生成器，必须退出 0 且
     自检误差 < 1 mm。

  2. **变异：点选平面必须是「顶面」而不是「中心平面」**。这两者只差
     ``size/2``（14.25 mm），选错了页面照样能用、数据照样出得来，
     但每个坐标都偏十几毫米。本节把它换算成具体数字：用中心平面反投
     同一个像素，误差必须**明显变大**。若这条不过，说明第 1 节是碰巧过的。

  3. **页面里的公式 == Python 里的公式**。从生成出来的 HTML 里**抽出
     JS 源码**（不是重打一遍）交给 node 跑，逐像素比对。重打一遍只能
     证明「我打的字是对的」，抽出来才能证明**文件里写的**是对的。
     node 不在就跳过这一节，不算失败。
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import matrix_to_quat                            # noqa: E402
import auto_label_grid as L                                            # noqa: E402
import make_click_labeler as M                                         # noqa: E402

N_PASS = N_FAIL = N_SKIP = 0
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


def skip(name, why):
    global N_SKIP
    N_SKIP += 1
    print('  SKIP  %s   ← %s' % (name, why))


# ------------------------------------------------------------ 合成一个相机
W, H = 1280, 720
FX, FY, CX, CY = 917.62, 916.43, 649.55, 344.91
T_CAM = np.array([0.1591, -0.5776, -0.6442])


def make_extrinsics():
    z_opt = -T_CAM / np.linalg.norm(T_CAM)
    x_opt = np.cross(np.array([0.0, 0.0, -1.0]), z_opt)
    x_opt /= np.linalg.norm(x_opt)
    y_opt = np.cross(z_opt, x_opt)
    R = np.column_stack([x_opt, y_opt, z_opt])
    return (R, {'target_frame': 'board', 'camera_frame': 'camera_color_optical_frame',
                'rotation': [float(x) for x in matrix_to_quat(R)],
                'translation': T_CAM.tolist()})


def main():
    global N_PASS
    import cv2

    d = tempfile.mkdtemp(prefix='clicklabel_')
    try:
        R, extr = make_extrinsics()
        info = {'width': W, 'height': H, 'fx': FX, 'fy': FY, 'cx': CX, 'cy': CY}
        p_info = os.path.join(d, 'info.json')
        p_extr = os.path.join(d, 'extr.json')
        p_img = os.path.join(d, 'frame.png')
        p_out = os.path.join(d, 'click.html')
        json.dump(info, open(p_info, 'w'))
        json.dump(extr, open(p_extr, 'w'))
        cv2.imwrite(p_img, np.full((H, W, 3), 200, np.uint8))

        size = 0.0285
        zsign = 1.0 if T_CAM[2] > 0 else -1.0
        z_top = (zsign - 1) * size / 2.0
        probe = (0.200, -0.050)

        # ------------------------------------------------------- [1] 端到端
        print('\n[1] 端到端：跑一遍生成器，必须退出 0')
        rc = M.main(['--info', p_info, '--extrinsics', p_extr, '--image', p_img,
                     '--size-mm', '28.5', '--out', p_out])
        check('生成器退出码 0', rc == 0, '实际 %r' % (rc,))
        check('HTML 写出来了', os.path.exists(p_out) and os.path.getsize(p_out) > 2000)
        html = open(p_out).read() if os.path.exists(p_out) else ''
        check('__CFG__ 占位符已全部替换', '__CFG__' not in html)
        check('画面以 data URI 内嵌', 'data:image/png;base64,' in html)

        # 交付方式是**本地文件**，所以必须自带 charset —— 否则中文界面的编码
        # 交给系统区域去猜，中文 Windows 上会被当成 GBK 解成乱码，而且页面
        # "看起来还能用"，只是字全糊了。这个坑本轮真踩到过一次。
        check('默认输出是独立文档（有 doctype）',
              html.lstrip().lower().startswith('<!doctype html>'))
        check('声明了 charset=utf-8', 'charset="utf-8"' in html)
        check('head 里有标题', '<title>' in html[:8192])
        check('body 标签闭合齐', html.rstrip().endswith('</html>'))

        p_bare = os.path.join(d, 'bare.html')
        M.main(['--info', p_info, '--extrinsics', p_extr, '--image', p_img,
                '--size-mm', '28.5', '--out', p_bare, '--bare'])
        bare = open(p_bare).read()
        check('--bare 不带骨架（供 artifact 用）',
              '<!doctype' not in bare.lower() and '<html' not in bare.lower()
              and '<body' not in bare.lower())
        check('--bare 仍然内嵌画面与 CFG',
              'data:image/png;base64,' in bare and '__CFG__' not in bare)

        # ------------------------------------------- [2] 变异：平面选错
        print('\n[2] 变异：点选平面必须是顶面（z_top），不是中心平面（zc）')
        proj = L.TfProjector(info, extr)
        h = size / 2.0
        top4 = [(probe[0] + dx, probe[1] + dy, z_top)
                for dx in (-h, h) for dy in (-h, h)]
        pix, dropped = L.project_pinhole([proj.to_camera(c) for c in top4],
                                         FX, FY, CX, CY)
        check('探针顶面四角都在画面内', not dropped and len(pix) == 4)
        u = sum(p[0] for p in pix) / 4.0
        v = sum(p[1] for p in pix) / 4.0

        def backproject(u_, v_, plane_z):
            dvec = np.array([(u_ - CX) / FX, (v_ - CY) / FY, 1.0])
            dt = R @ dvec
            lam = (plane_z - T_CAM[2]) / dt[2]
            return T_CAM + lam * dt

        err_top = float(np.linalg.norm(backproject(u, v, z_top)[:2] - np.array(probe)))
        err_cen = float(np.linalg.norm(backproject(u, v, zsign * size / 2.0)[:2]
                                       - np.array(probe)))
        check('顶面：误差 < 1 mm', err_top < 0.001, '实际 %.2f mm' % (err_top * 1000))
        check('中心平面：误差 > 10 mm（错得足够明显）', err_cen > 0.010,
              '实际 %.2f mm —— 若这条不过，说明 [1] 是碰巧过的' % (err_cen * 1000))

        # ------------------------------------------ [2b] plane_z 的独立判据
        # 生成器自己的那段自检**曾经是恒真的**：它用 plane_z() 去造探针顶面，
        # 又用 plane_z() 去反投，于是任何平面都完美往返（变异实测 8 PASS / exit 0）。
        # 现在探针改成独立取「离相机更近的那个面」，这条则把 plane_z() 直接
        # 钉在那个独立判据上 —— 变异 plane_z 到中心平面，这里立刻红。
        print('\n[2b] plane_z() == 立体几何里离相机更近的那个面')
        zc = zsign * size / 2.0
        faces = [zc - h, zc + h]
        z_nearest = min(faces, key=lambda z: abs(z - T_CAM[2]))
        z_other = max(faces, key=lambda z: abs(z - T_CAM[2]))
        z_fn = M.plane_z(size, zsign)
        check('plane_z() 与独立判据一致', abs(z_fn - z_nearest) < 1e-12,
              'plane_z=%.5f 独立判据=%.5f，差 %.2f mm'
              % (z_fn, z_nearest, abs(z_fn - z_nearest) * 1000))
        check('两个面确实分得开（差 %.1f mm）'
              % (abs(z_nearest - z_other) * 1000),
              abs(z_nearest - z_other) > size * 0.5)

        # ------------------------------------ [3] 页面 JS == Python
        print('\n[3] 页面里的 JS 公式 == Python 公式（抽源码交给 node 跑）')
        node = shutil.which('node') or shutil.which('nodejs')
        if not node:
            skip('JS/Python 逐像素一致', '本机没有 node')
        else:
            px = [(648.5, 344.9), (200.0, 120.0), (1100.0, 650.0), (640.0, 700.0)]
            js = (
                "const fs=require('fs');"
                "const src=fs.readFileSync(process.argv[2],'utf8');"
                "const raw=src.match(/var CFG = (\\{[\\s\\S]*?\\});\\s*\\n/)[1]"
                ".replace(/\"img\":\"[^\"]*\"/,'\"img\":\"\"');"
                "const CFG=JSON.parse(raw);"
                "const a=src.indexOf('  function zsign()');"
                "const b=src.indexOf('  function sizeMM()');"
                "if(a<0||b<=a){process.exit(3);}"
                "eval(src.slice(a,b));"
                "const px=JSON.parse(process.argv[3]);"
                "for(const [u,v] of px){const p=toBoard(u,v,28.5);"
                "console.log(p[0].toFixed(6)+' '+p[1].toFixed(6));}"
            )
            p_js = os.path.join(d, 'x.js')
            open(p_js, 'w').write(js)
            r = subprocess.run([node, p_js, p_out, json.dumps(px)],
                               capture_output=True, text=True)
            if r.returncode != 0:
                check('node 跑通并取到 toBoard', False,
                      'rc=%d %s' % (r.returncode, r.stderr.strip()[:200]))
            else:
                got = [tuple(float(t) for t in line.split())
                       for line in r.stdout.strip().splitlines()]
                exp = [backproject(u2, v2, z_top)[:2] for (u2, v2) in px]
                dmax = max(float(np.abs(np.array(g) - np.array(e)).max())
                           for g, e in zip(got, exp))
                check('四个像素的靶系坐标逐点一致（<1e-6 m）', dmax < 1e-6,
                      '最大差 %.3g m' % dmax)

        print('\n%d PASS  %d FAIL  %d SKIP' % (N_PASS, N_FAIL, N_SKIP))
        if FAILS:
            print('未通过：' + '; '.join(FAILS))
        return 1 if N_FAIL else 0
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
