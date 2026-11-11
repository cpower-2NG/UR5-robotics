#!/usr/bin/env python3
"""V2 候选族自检：A/B/C 三档的包含关系、数量、接近角与几何。

不需要 MoveIt：只检查**候选生成**这一层。有效性判定沿用 E7，不在这里重测。
跑法：
    python3 scripts/test_stage4_v2_tiers.py
"""
import os
import sys

import numpy as np

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_scene_model as sm                                   # noqa: E402
import stage4_v2_tiers as v2                                      # noqa: E402

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


BLOCK = {"id": 0, "x": 0.35, "y": 0.30, "z": 0.015, "yaw": 17.0,
         "size": 0.0295, "top": 0.0295}
TABLE = 0.0


def key(c):
    return (c["axis"], c["sign"], c["closing"], c["variant"],
            round(c.get("tilt_deg", 0.0), 1),
            round(c.get("azimuth_deg", -1.0), 1))


def section_counts():
    print('\n[1] 三档数量与包含关系')
    t = v2.tier_candidates(BLOCK, TABLE)
    check('A = 4 个竖直位姿', len(t["A"]) == 4, len(t["A"]))
    # 斜向族每个 (角, 方位) 只有 2 个唯一腕部朝向（30/45/60 × 4 × 2 = 24）。
    check('B = A + 24 斜向', len(t["B"]) == 28, len(t["B"]))
    check('C = 生产 24 + 24 斜向', len(t["C"]) == 48, len(t["C"]))
    ka = {key(c) for c in t["A"]}
    kb = {key(c) for c in t["B"]}
    kc = {key(c) for c in t["C"]}
    check('A ⊆ B ⊆ C', ka <= kb and kb <= kc)
    check('A 全是 0°', all(c["tilt_deg"] == 0.0 and c["vertical"]
                           for c in t["A"]))


def section_angles():
    print('\n[2] 接近角与方位')
    t = v2.tier_candidates(BLOCK, TABLE)
    ang = [c for c in t["B"] if c["axis"] == 3]
    check('斜向 24 个且无重复位姿', len(ang) == 24 and
          len({key(c) for c in ang}) == 24, len(ang))
    check('角度只有 30/45/60', {c["tilt_deg"] for c in ang}
          == {30.0, 45.0, 60.0})
    check('方位 4 个', {c["azimuth_deg"] for c in ang} == {0.0, 90.0,
                                                          180.0, 270.0})
    check('接近方向都是单位向量',
          all(abs(np.linalg.norm(c["approach"]) - 1.0) < 1e-9 for c in ang))
    check('接近方向都朝下',
          all(np.dot(c["approach"], [0.0, 0.0, -1.0]) > 0.49 for c in ang))


def section_geometry():
    print('\n[3] tool0 摆位在表面外侧')
    t = v2.tier_candidates(BLOCK, TABLE)
    c = next(c for c in t["B"] if c["axis"] == 3 and c["tilt_deg"] == 45.0)
    a = np.array(c["approach"])
    pos = np.array(c["pos"])
    center = np.array([BLOCK["x"], BLOCK["y"], BLOCK["z"]])
    axes = sm.block_axes(BLOCK)
    r = BLOCK["size"] / 2.0 * sum(abs(float(a @ ax)) for ax in axes)
    expect = center - a * r - a * (v2.TIP_Z - v2.GRASP_DEPTH)
    check('tool0 位置 = 支撑点 − a·(tip−depth)',
          np.linalg.norm(pos - expect) < 1e-9, (pos, expect))
    check('tool0 在块心外侧（沿 −a 方向）',
          float(np.dot(pos - center, a)) < 0.0)
    q = np.array(c["quat"])
    check('四元数已归一', abs(np.linalg.norm(q) - 1.0) < 1e-9)
    rot = sm.quat_to_mat(tuple(q))
    check('tool0 z 轴 = 接近方向',
          np.linalg.norm(rot[:, 2] - a) < 1e-9, (rot[:, 2], a))


def section_tier_semantics():
    print('\n[4] 档位语义')
    prod = [dict(c, tilt_deg=(0.0 if c["vertical"] else
                              (180.0 if c["approach"][2] > 0.5 else 90.0)))
            for c in sm.grasp_candidates(BLOCK, TABLE)]
    n_v = sum(1 for c in prod if c["tilt_deg"] == 0.0)
    n_h = sum(1 for c in prod if c["tilt_deg"] == 90.0)
    n_u = sum(1 for c in prod if c["tilt_deg"] == 180.0)
    check('生产候选 = 4 竖直 + 16 水平 + 4 向上', (n_v, n_h, n_u) == (4, 16, 4),
          (n_v, n_h, n_u))
    t = v2.tier_candidates(BLOCK, TABLE)
    check('B 不含 90°/180°',
          all(c["tilt_deg"] <= 60.0 for c in t["B"]))


def section_robust_eval():
    print('\n[5] IK 重采样')
    # 第 k 次才 ok：包装器必须一直重试到 ok，并报出用了几次。
    calls = {"n": 0}

    def fake_eval(node, ik, fk, val, cand, target, others, table, pool):
        calls["n"] += 1
        if calls["n"] == 3:
            return "ok", {"seed": 0, "sol": "S"}
        return "blocked_arm", {"seed": 0}

    verdict, detail = v2.evaluate_robust(
        None, {"ik": None, "fk": None, "validity": None}, BLOCK,
        None, [], None, None, evaluate=fake_eval)
    check('5 次内任一 ok 即算可抓', verdict == "ok" and calls["n"] == 3, calls)
    check('报出采样次数', detail.get("samples") == 3, detail)
    calls["n"] = 0

    def always_fail(node, ik, fk, val, cand, target, others, table, pool):
        calls["n"] += 1
        return "blocked_arm", {"seed": 0}

    verdict, _ = v2.evaluate_robust(
        None, {"ik": None, "fk": None, "validity": None}, BLOCK,
        None, [], None, None, evaluate=always_fail)
    check('全失败时最多采样 5 次', calls["n"] == v2.IK_SAMPLES == 5, calls)
    check('全失败保留最后一次判词', verdict == "blocked_arm", verdict)


def main():
    for fn in (section_counts, section_angles, section_geometry,
               section_tier_semantics, section_robust_eval):
        fn()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败：%s' % ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
