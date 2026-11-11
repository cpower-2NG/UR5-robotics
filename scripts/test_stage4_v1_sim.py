#!/usr/bin/env python3
"""V1 工具自检：跨机位去重、覆盖统计、指定判分。

不需要仿真：实例池与真值全用合成数据，检查的是**记账**是否对
（去重半径、覆盖判定、误选计数），不是感知精度。
跑法：
    python3 scripts/test_stage4_v1_sim.py
"""
import os
import sys

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_v1_sim as v1                                      # noqa: E402

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
        print('  FAIL  %s%s' % (name, '   ← %s' % (detail,) if detail else ''))


def inst(idx, cx, cy, n=500, cls='red', low=False, top=30.0):
    return {"idx": idx, "cx": cx, "cy": cy, "n": n, "class": cls,
            "low": low, "top": top, "yaw_rect": 0.0,
            "ext_rect": [29.5, 29.5], "px_area_mm2": 0.93,
            "class_share": 1.0}


def truth(blocks):
    """blocks: [(id, color, x_mm, y_mm)]；top/size 给固定值。"""
    return {"batch": "S3", "blocks": [
        {"id": i, "color": c, "x": x / 1000.0, "y": y / 1000.0,
         "top": 0.030, "z": 0.015, "yaw": 0.0, "size": 0.0295}
        for i, c, x, y in blocks]}


def section_geom():
    print('\n[1] 位姿与角度')
    q = v1.unwrap_to([0.1, -5.36, 3.2], [0.0, 0.0, 0.0])
    check('unwrap 把 -5.36 换到 +0.923',
          abs(q[1] - (-5.36 + 2 * np.pi)) < 1e-9, q)
    pick = v1.view_pose('pick')
    free = v1.view_pose('free')
    check('两个机位的 tool0 不同', np.linalg.norm(pick[0] - free[0]) > 0.05)


def section_vantages():
    print('\n[1b] 同区三视点（A1：三个 vantage，不是三个区域）')
    import stage4_wrist_survey as ws
    center = np.array([0.5 * (0.25 + 0.575), 0.5 * (0.17 + 0.42), 0.500])
    check('默认三视点就是 pick/pick-out/pick-in',
          v1.VANTAGE_VIEWS == ("pick", "pick-out", "pick-in"),
          v1.VANTAGE_VIEWS)
    seen = {}
    for name, shift in (("pick", (0.0, 0.0)),
                        ("pick-out", (-0.100, +0.030)),
                        ("pick-in", (+0.100, -0.030))):
        tool0, cam, R = v1.view_pose(name)
        seen[name] = (tool0, cam, R)
        want = center + np.array([shift[0], shift[1], 0.0])
        check('%s 相机 = 区心 + %s' % (name, shift),
              np.allclose(cam, want, atol=1e-9), cam)
        check('%s 仍竖直朝下' % name,
              np.allclose(R, ws.pp.down_rotation(0.0)), R)
        marg = ws.coverage_pose(v1.PICK_ZONE_M, cam, R)
        check('%s 拾取区八角余量 > 30 px' % name, min(marg) > 30.0, marg)
    check('三个机位互不相同',
          len({tuple(np.round(s[0], 4)) for s in seen.values()}) == 3)


def section_nearest():
    print('\n[2] 最近实例')
    pool = [inst(0, 100.0, 100.0), inst(1, 140.0, 100.0)]
    b, d = v1.nearest(pool, 110.0, 100.0)
    check('取到 10 mm 那个', b is not None and b["idx"] == 0 and d == 10.0,
          (b and b["idx"], d))
    b, d = v1.nearest([], 0.0, 0.0)
    check('空池 → (None, inf)', b is None and d == float('inf'))


def section_usable():
    print('\n[3] 可用实例（不是 low、在拾取区）')
    pool = [inst(0, 300.0, 300.0), inst(1, 300.0, 300.0, low=True),
            inst(2, 900.0, 300.0, n=900)]
    got = v1.usable(pool)
    check('只留区内非 low', [b["idx"] for b in got] == [0], got)


def section_coverage():
    print('\n[4] 覆盖')
    t = truth([(0, 'red', 300.0, 300.0), (1, 'blue', 400.0, 300.0),
               (2, 'green', 900.0, 300.0)])
    cov = v1.coverage(t, [inst(0, 314.9, 300.0)])
    check('区外真值不计入', len(cov) == 2, cov)
    check('14.9 mm 算检出', cov[0]["detected"] and not cov[1]["detected"], cov)
    cov = v1.coverage(t, [inst(0, 315.1, 300.0)])
    check('15.1 mm 算漏检', not cov[0]["detected"], cov)


def section_merge():
    print('\n[5] 跨机位合并')
    pools = [("pick", [inst(0, 300.0, 300.0, n=800),
                       inst(1, 400.0, 300.0, n=600)]),
             ("free", [inst(0, 290.0, 300.0, n=500),      # 与 pick 的块 0 同块
                       inst(1, 300.0, 420.0, n=400)]),
             ("pad", [inst(0, 302.0, 301.0, n=300)])]     # 仍是块 0
    got = v1.merge_pools(pools)
    check('去重后 3 个', len(got) == 3, [(b["cx"], b["cy"]) for b in got])
    a = next(b for b in got if abs(b["cx"] - 300.0) < 5
             and abs(b["cy"] - 300.0) < 5)
    check('同块保留像素最多的那份', a["n"] == 800, a["n"])
    check('记下三个机位', a["views"] == "free,pad,pick", a["views"])
    check('n_view=3', a["n_view"] == 3, a["n_view"])
    check('idx 重排连续', [b["idx"] for b in got] == [0, 1, 2],
          [b["idx"] for b in got])
    far = next(b for b in got if abs(b["cy"] - 420.0) < 5)
    check('单机位块 n_view=1', far["n_view"] == 1, far["views"])

    # 叠放块：同一 xy、顶面差 29.5 mm ⇒ 必须留成两个（否则遮挡块被吞掉）
    stacked = v1.merge_pools([
        ("pick", [inst(0, 300.0, 300.0, n=800, top=30.0)]),
        ("pick-out", [inst(0, 303.0, 301.0, n=500, top=59.5)])])
    check('叠放上下两块不合并', len(stacked) == 2,
          [(b["cx"], b["cy"], b["top"]) for b in stacked])
    check('叠放两块各记 1 个机位',
          sorted(b["views"] for b in stacked) == ["pick", "pick-out"],
          [b["views"] for b in stacked])
    # 同一块跨机位：顶面高差几毫米（深度噪声）仍要合并
    noisy = v1.merge_pools([
        ("pick", [inst(0, 300.0, 300.0, n=800, top=30.0)]),
        ("pick-out", [inst(0, 303.0, 301.0, n=500, top=34.0)])])
    check('同一块顶面差 4 mm 仍合并', len(noisy) == 1, len(noisy))


def section_specify():
    print('\n[6] 指定 top-1')
    t = truth([(0, 'red', 300.0, 300.0)])
    good = [inst(0, 305.0, 300.0)]
    r = v1.specify_score(t, good)
    # 只有一个红块 ⇒ 九种说法都唯一指向它（E5 命令集的正常情形）。
    check('单块真值出 9 条命令', r["n_commands"] == 9, r["n_commands"])
    check('全选对、无误选', r["correct"] == 9 and r["misbind"] == 0, r)
    bad = [inst(0, 350.0, 300.0)]
    r = v1.specify_score(t, bad)
    check('选错不记 correct', r["correct"] == 0, r)
    check('非 at 的 8 条记误选', r["misbind"] == 8, r["counts"])


def section_scene():
    print('\n[7] 单场景汇总')
    t = truth([(0, 'red', 300.0, 300.0), (1, 'blue', 400.0, 300.0)])
    pools = [("pick", [inst(0, 300.0, 300.0, n=800)]),
             ("free", []), ("pad", [inst(0, 402.0, 300.0, n=300)])]
    out = v1.eval_scene("S3:0", t, pools)
    check('真值块 2', out["n_truth_in_zone"] == 2, out["n_truth_in_zone"])
    check('单机位检出 1', out["views"]["pick"]["n_detected"] == 1, out)
    check('合并检出 2', out["merged"]["n_detected"] == 2, out["merged"])
    check('补回块 1', out["occluded_miss"]["recovered"] == [1],
          out["occluded_miss"])
    check('合并后 1 个来自多机位', out["merged"]["n_from_3views"] == 0, out)


def section_stale_sim():
    print('\n[8] 残留 Gazebo 预检')
    line = ("4242 gz sim -r -s --render-engine ogre2 "
            "--physics-engine gz-physics-bullet-featherstone-plugin "
            "/tmp/grasp_scene_S3_0/grasp_world.sdf")
    check('认出另一台 Gazebo', v1.other_sims(line) == [line], v1.other_sims(line))
    check('空/None → 干净', v1.other_sims("") == [] and v1.other_sims(None) == [])
    check('pgrep 自身不算',
          v1.other_sims("999 pgrep -af gz sim") == [])
    check('launch 命令行不算',
          v1.other_sims("ros2 launch arm_grasp_sim full.launch.py scene:=S1:0") == [])


def main():
    for fn in (section_geom, section_vantages, section_nearest,
               section_usable, section_coverage, section_merge,
               section_specify, section_scene, section_stale_sim):
        fn()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：%s' % ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
