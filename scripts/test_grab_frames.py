#!/usr/bin/env python3
"""``grab_frames.py`` 的自检 —— 不需要 ROS、不需要相机、不需要板子。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
这份脚本会写出一份 ``info.json`` 给 ④ 标注器当内参用。它唯一会**静默出错**
的地方是 ``K`` 的下标映射 —— ROS 的 ``K`` 是行主序 3×3 摊平的九个数::

    K = [fx, 0, cx,  0, fy, cy,  0, 0, 1]

把 ``cx`` 写成 ``K[1]``（=0）不会报错、不会崩，只是**每一张图的每一个标签
都偏同一个量**。这类偏差事后极难归因 —— 训出来的模型「看着还行、就是抓不准」。
所以这里用**九个互不相同**的数当输入：任何下标写错都会立刻对不上。

第 [3] 节用的是**真机实测的 K**（板上那台 D435i），不是我造的向量 ——
它同时钉住了「本文件的下标」和「相机实际报的数」两件事。
"""
import os
import subprocess
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grab_frames as G                                            # noqa: E402

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


def stub(k, d=None, w=1280, h=720, frame_id='camera_color_optical_frame'):
    return types.SimpleNamespace(
        k=k, d=d if d is not None else [0.0] * 5, width=w, height=h,
        header=types.SimpleNamespace(
            frame_id=frame_id,
            stamp=types.SimpleNamespace(sec=1790939390, nanosec=951379639)))


def main():
    # ------------------------------------------------ [1] 下标映射（九个数互异）
    print('\n[1] K 的下标映射 —— 九个数互不相同，写错下标必然对不上')
    k = [101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0]
    got = G.camera_info_to_dict(stub(k))
    check('fx == K[0]', got['fx'] == 101.0, '实际 %r' % (got['fx'],))
    check('cx == K[2]', got['cx'] == 103.0, '实际 %r' % (got['cx'],))
    check('fy == K[4]', got['fy'] == 105.0, '实际 %r' % (got['fy'],))
    check('cy == K[5]', got['cy'] == 106.0, '实际 %r' % (got['cy'],))
    check('四个平铺值互不相同（下标换位一定暴露）',
          len({got['fx'], got['fy'], got['cx'], got['cy']}) == 4)
    check('K 原样保留', got['K'] == k)
    check('宽高原样保留', (got['width'], got['height']) == (1280, 720))
    check('frame_id 同时写进两处',
          got['frame_id'] == got['info_frame_id'] == 'camera_color_optical_frame')
    check('stamp 原样保留',
          got['stamp'] == {'sec': 1790939390, 'nanosec': 951379639})

    # ------------------------------------------------------- [2] 坏输入要拒绝
    print('\n[2] K 不是 9 个元素时必须报错，不能悄悄凑合')
    for bad in ([1.0] * 8, [1.0] * 12, []):
        try:
            G.camera_info_to_dict(stub(bad))
            check('len(K)=%d 应当抛 ValueError' % len(bad), False, '没抛')
        except ValueError:
            check('len(K)=%d 抛了 ValueError' % len(bad), True)
        except Exception as e:                                     # noqa: BLE001
            check('len(K)=%d 抛的是 ValueError' % len(bad), False,
                  '抛的是 %s' % type(e).__name__)

    # ------------------------------------- [3] 真机实测的 K（回归向量，不是我造的）
    # 板上那台 D435i（俯视，SDK 号 <topcam-serial>）2026-10-02 实测报的数。
    # 与 .scratch 里那份 info.json 逐位一致。
    print('\n[3] 真机实测 K —— 与 2026-10-02 那份 info.json 逐位一致')
    real_k = [917.6205444335938, 0.0, 649.554931640625,
              0.0, 916.4271240234375, 344.9062805175781,
              0.0, 0.0, 1.0]
    r = G.camera_info_to_dict(stub(real_k))
    check('fx == 917.6205444335938', r['fx'] == 917.6205444335938)
    check('fy == 916.4271240234375', r['fy'] == 916.4271240234375)
    check('cx == 649.554931640625', r['cx'] == 649.554931640625)
    check('cy == 344.9062805175781', r['cy'] == 344.9062805175781)
    check('cx 落在画面宽度内、cy 落在高度内（不是拿反了）',
          0 < r['cx'] < 1280 and 0 < r['cy'] < 720,
          'cx=%.1f cy=%.1f' % (r['cx'], r['cy']))

    # -------------------------------------------- [4] 话题名是双层的（实测）
    print('\n[4] 话题名 —— 本板实测**只有**双层命名空间那套')
    check('COLOR 是 /camera/camera/color/image_raw',
          G.COLOR == '/camera/camera/color/image_raw', G.COLOR)
    check('INFO 是 /camera/camera/color/camera_info',
          G.INFO == '/camera/camera/color/camera_info', G.INFO)
    check('两个话题同前缀（不是一处写成单层）',
          G.COLOR.rsplit('/', 1)[0] == G.INFO.rsplit('/', 1)[0])

    # --------------------------------- [5] 没有 ROS 时要干净地拒绝，不能崩栈
    print('\n[5] 开发机上（没有 rclpy）跑 CLI：退出码 2，不许抛栈')
    if G.USE_ROS:
        print('  SKIP  本机有 rclpy，这条只在开发机上成立')
    else:
        r2 = subprocess.run(
            [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                          'grab_frames.py'), '/tmp/_nope', '1'],
            capture_output=True, text=True)
        check('退出码 2', r2.returncode == 2, '实际 %d' % r2.returncode)
        check('没有 Traceback', 'Traceback' not in r2.stderr, r2.stderr.strip()[:160])
        check('说清了要去板子上跑', '板子' in r2.stdout, r2.stdout.strip()[:160])

    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
