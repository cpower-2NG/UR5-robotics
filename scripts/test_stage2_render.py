#!/usr/bin/env python3
"""A4 渲染器的自检：合成一支 mini-run，必须能写出可打开的 MP4。

钉住：
  · 轨迹插值的帧数/端点（纯逻辑，不碰视频）；
  · 合成 mini-run（帧 + plan + candidates + 两关节 URDF）→
    MP4 存在、VideoCapture 能打开、帧数 > 0；
  · 变异：删掉 pick.json → 明确报错退出 2，不抛 traceback。

跑法：
    python3 scripts/test_stage2_render.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

import stage2_render as sr                                       # noqa: E402

CASES = []


def case(fn):
    CASES.append(fn)
    return fn


# ---------------------------------------------------------------- 纯逻辑
def _plan():
    return {
        'version': 1,
        'q_start': [0.0, 0.0],
        'meta': {'block_idx': 2, 'class': 'red'},
        'waypoints': [
            {'name': 'approach', 'q': [0.1, 0.0], 'duration': 1.0},
            {'name': 'grasp', 'q': [0.1, 0.2], 'duration': 0.5},
        ],
    }


@case
def test_trajectory_samples_endpoints_and_count():
    s = sr.trajectory_samples(_plan(), fps=10.0)
    # start + 10 + 5 = 16
    assert len(s) == 16, len(s)
    assert s[0][1] == [0.0, 0.0]
    assert s[-1][1] == [0.1, 0.2]
    assert s[-1][2] == 'grasp'
    assert abs(s[-1][0] - 1.5) < 1e-9, s[-1]
    # 中段是线性插值，不是跳变
    assert 0.0 < s[5][1][0] < 0.1
    assert abs(s[5][0] - 0.5) < 1e-9, s[5]


@case
def test_trajectory_requires_q_start():
    bad = _plan()
    bad.pop('q_start')
    bad['meta'].pop('q_start', None)
    try:
        sr.trajectory_samples(bad)
    except ValueError as e:
        assert 'q_start' in str(e)
    else:
        raise AssertionError('缺 q_start 应报 ValueError')


@case
def test_height_colors_span():
    z = np.array([0.0, 0.045, 0.09, 0.2])
    cols = sr._height_colors(z, 0.0)
    assert cols.shape == (4, 3)
    # 低处偏蓝（B>R），高处偏红（R>B）
    assert cols[0][0] > cols[0][2], cols[0]
    assert cols[2][2] > cols[2][0], cols[2]
    # 超出 span 被夹住，不溢出 uint8
    assert cols[3][2] == 255 and cols[3][0] == 0, cols[3]


@case
def test_topview_draws_and_small_image_skips():
    """640×480 上俯视图要真的画；320×240 放不下就安静跳过（不是崩）。"""
    root = tempfile.mkdtemp(prefix='stage2-topview-')
    try:
        urdf = os.path.join(root, 'robot.urdf')
        with open(urdf, 'w') as f:
            f.write(_URDF)
        import urkin
        chain = urkin.Chain(urdf)
        cands = {'per_block': [{'block_idx': 0, 'block_xy': [100.0, 100.0]}],
                 'candidates': [{'xyz_base': [0.1, 0.1, 0.0]}]}
        round_doc = {'slots_mm': [[200.0, 200.0]]}
        big = np.zeros((480, 640, 3), np.uint8)
        sr.draw_topview(big, _plan(), chain, [0.1, 0.2], cands, round_doc)
        assert big.sum() > 0, '640×480 上俯视图没画出任何像素'
        small = np.zeros((240, 320, 3), np.uint8)
        sr.draw_topview(small, _plan(), chain, [0.1, 0.2], cands, round_doc)
        assert small.sum() == 0, '320×240 放不下应跳过，不该硬画'
    finally:
        shutil.rmtree(root, ignore_errors=True)


# ---------------------------------------------------------------- mini-run
_URDF = """<?xml version="1.0"?>
<robot name="mini">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <link name="tool0"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="shoulder_link"/><child link="tool0"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
</robot>
"""


def _write_mini_run(root):
    os.makedirs(os.path.join(root, 'frames', 'f1'))
    os.makedirs(os.path.join(root, 'cycles', '01'))
    fdir = os.path.join(root, 'frames', 'f1')
    h, w = 240, 320
    cv2.imwrite(os.path.join(fdir, 'color.png'),
                np.full((h, w, 3), 40, np.uint8))
    cv2.imwrite(os.path.join(fdir, 'depth.png'),
                np.full((h, w), 500, np.uint16))
    K = [[300.0, 0.0, 160.0], [0.0, 300.0, 120.0], [0.0, 0.0, 1.0]]
    json.dump({'K': K}, open(os.path.join(fdir, 'camera_info.json'), 'w'))
    q = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    q0 = {'x': 0, 'y': 0, 'z': 0, 'w': 1}

    def _tf(q_, tr):
        return {'rotation': dict(q_),
                'translation': {'x': tr[0], 'y': tr[1], 'z': tr[2]}}

    tf = {'ok': {
        # cam_pose 见到 base_link→光学系直连就用它；相机在 z=0.5 朝下
        # （R = diag(1,-1,-1) = 绕 x 转 180°，四元数 xyzw = [1,0,0,0]）。
        'base_link:camera_color_optical_frame': _tf(
            {'x': 1, 'y': 0, 'z': 0, 'w': 0}, [0, 0, 0.5]),
        'base_link:shoulder_link': _tf(q0, [0, 0, 0.1]),
        'shoulder_link:upper_arm_link': {'rotation': [0, 0, 0, 1],
                                         'translation': [0, 0, 0.2]},
        'upper_arm_link:forearm_link': {'rotation': [0, 0, 0, 1],
                                        'translation': [0, 0, 0.3]},
        'forearm_link:wrist_1_link': {'rotation': [0, 0, 0, 1],
                                      'translation': [0, 0, 0.1]},
        'wrist_1_link:wrist_2_link': {'rotation': [0, 0, 0, 1],
                                      'translation': [0, 0, 0.1]},
        'wrist_2_link:wrist_3_link': {'rotation': [0, 0, 0, 1],
                                      'translation': [0, 0, 0.05]},
        'wrist_3_link:tool0': {'rotation': [0, 0, 0, 1],
                               'translation': [0, 0, 0.05]},
    }}
    json.dump(tf, open(os.path.join(fdir, 'tf.json'), 'w'))
    json.dump({'joints': q}, open(os.path.join(fdir, 'joint_states.json'), 'w'))
    json.dump({'candidates': [{
        'i': 0, 'score': 0.5, 'width_mm': 40.0, 'depth_mm': 20.0,
        'tilt_deg': 0.0, 'xyz_base': [0.0, 0.0, 0.0],
        'approach_base': [0.0, 0.0, 1.0], 'angle_deg': 10.0, 'seed': 0,
    }], 'per_block': [{'block_idx': 0, 'class': 'red',
                       'block_xy': [0.0, 0.0], 'n_near': 1,
                       'best': {'xyz_base': [0.0, 0.0, 0.0], 'score': 0.5,
                                'width_mm': 40.0, 'angle_deg': 10.0}}]},
              open(os.path.join(fdir, 'candidates.json'), 'w'))
    plan = {
        'version': 1, 'q_start': [1.0, 0.0],
        'meta': {'block_idx': 0, 'class': 'red'},
        'waypoints': [
            {'name': 'approach', 'q': [1.1, 0.0], 'duration': 1.0},
            {'name': 'grasp', 'q': [1.1, 0.2], 'duration': 0.5},
        ],
    }
    json.dump(plan, open(os.path.join(root, 'cycles', '01', 'pick.json'), 'w'))
    json.dump({'cycles': [{'n': 1, 'frame_pick': 'frames/f1',
                           'block_idx': 0, 'class': 'red',
                           'slot_idx': 0}],
               'slots_mm': [[100.0, 100.0]],
               'slot_classes': ['red']},
              open(os.path.join(root, 'round.json'), 'w'))
    with open(os.path.join(root, 'robot.urdf'), 'w') as f:
        f.write(_URDF)


def _run(args):
    return subprocess.run([sys.executable,
                           os.path.join(HERE, 'stage2_render.py')] + args,
                          capture_output=True, text=True)


@case
def test_render_mini_run_to_mp4():
    root = tempfile.mkdtemp(prefix='stage2-render-')
    try:
        _write_mini_run(root)
        out = os.path.join(root, 'render.mp4')
        p = _run(['--run-dir', root, '--cycle', '1', '--out', out,
                  '--fps', '10', '--max-frames', '12'])
        assert p.returncode == 0, '退出 %d\nSTDOUT %s\nSTDERR %s' % (
            p.returncode, p.stdout, p.stderr)
        assert os.path.exists(out), p.stdout
        assert os.path.getsize(out) > 1000, os.path.getsize(out)
        cap = cv2.VideoCapture(out)
        assert cap.isOpened(), 'VideoCapture 打不开 %s' % out
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        assert n == 12, '帧数 %d（应为 12）' % n
    finally:
        shutil.rmtree(root, ignore_errors=True)


@case
def test_missing_plan_exits_2_without_traceback():
    root = tempfile.mkdtemp(prefix='stage2-render-')
    try:
        _write_mini_run(root)
        os.remove(os.path.join(root, 'cycles', '01', 'pick.json'))
        out = os.path.join(root, 'render.mp4')
        p = _run(['--run-dir', root, '--cycle', '1', '--out', out])
        assert p.returncode == 2, '应退出 2，实际 %d\n%s' % (
            p.returncode, p.stderr)
        assert '渲染失败' in p.stderr, p.stderr
        assert 'Traceback' not in p.stderr, '不许抛 traceback：\n%s' % p.stderr
        assert not os.path.exists(out), '失败时不该留下半支 MP4'
    finally:
        shutil.rmtree(root, ignore_errors=True)


# ---------------------------------------------------------------- runner
def main():
    npass = nfail = 0
    for fn in CASES:
        try:
            fn()
        except AssertionError as e:
            print('FAIL %s: %s' % (fn.__name__, e))
            nfail += 1
        except Exception as e:                                   # noqa: BLE001
            print('ERROR %s: %s: %s' % (fn.__name__, type(e).__name__, e))
            nfail += 1
        else:
            print('PASS %s' % fn.__name__)
            npass += 1
    print('\n%d PASS, %d FAIL' % (npass, nfail))
    return 1 if nfail else 0


if __name__ == '__main__':
    sys.exit(main())
