#!/usr/bin/env python3
"""``stage4_wrist_survey`` 自检：巡检位的几何**必须含手眼横偏**。

钉住三件事：

1. 相机光轴与 tool0 +z 平行但本体横偏 91 mm ⇒ tool0 **不能**停在区域中心。
   2026-11-09 我第一次算就漏了这一项（把 tool0 放到相机的位置），这块锁死。
2. 契约拾取区四角在画幅内且有余量 —— 缺块比多块更贵：一块漏出画幅，
   E9 探针会以为塔不存在，规划直接错。
3. 既有 ``E9_SURVEY_TOOL0=(0.520,0.300,0.400)`` **不能**当感知位（s5 黄塔
   投到 v=968 > 720）—— 这条是「这个工具为什么存在」的反证，留在这里当活文档。

跑法：
    python3 scripts/test_stage4_wrist_survey.py
"""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage4_wrist_survey as ws                                  # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, ok, detail=None):
    global N_PASS, N_FAIL
    if ok:
        N_PASS += 1
        print("  PASS  %s" % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s   %r" % (name, detail))


def section_offset():
    print("\n[1] tool0 ≠ 区域中心：手眼横偏 91 mm 必须算进去")
    tool0, cam, _ = ws.survey_tool0()
    cx = 0.5 * (ws.PICK_ZONE_M[0] + ws.PICK_ZONE_M[1])
    cy = 0.5 * (ws.PICK_ZONE_M[2] + ws.PICK_ZONE_M[3])
    off = (tool0[:2] - np.array([cx, cy])) * 1000.0
    check("tool0 与中心横偏 > 80 mm（漏了手眼就会是 ~0）",
          float(np.hypot(*off)) > 80.0, off)
    check("tool0 横偏 = (+31.9, −85.3) mm（ψ=0）",
          np.allclose(off, [31.924, -85.321], atol=0.05), off)
    check("相机落在区域中心正上方",
          np.allclose(cam[:2], [cx, cy], atol=1e-9), cam)
    check("相机 z = 给定高度", abs(cam[2] - ws.SURVEY_H_M) < 1e-9, cam)
    check("tool0 z = 相机 z + 24.3 mm（相机比 tool0 低）",
          abs((tool0[2] - cam[2]) * 1000.0 - 24.256) < 0.05, tool0)


def section_coverage():
    print("\n[2] 契约拾取区整块进画幅，且有余量")
    marg, _, _ = ws.coverage()
    check("四边余量都 > 50 px", min(marg) > 50.0, marg)
    check("余量左右对称（相机居中）", abs(marg[0] - marg[1]) < 20.0, marg)

    # 抬高更保险、压低更容易出画幅
    hi, _, _ = ws.coverage(height=0.70)
    lo, _, _ = ws.coverage(height=0.30)
    check("抬高余量更大", min(hi) > min(marg), (hi, marg))
    check("压到 0.30 m 就有角点出画幅", min(lo) < 0.0, lo)


def section_check_blocks():
    print("\n[3] s5 三根塔（2026-11-08 真机）都在画幅内；旧巡检位看不全")
    _, tool0, cam = ws.coverage()
    R = ws.pp.down_rotation(0.0)
    for name, x, y in (("yellowgreen", 0.4586, 0.2634),
                       ("yellow", 0.5376, 0.1732),
                       ("orange", 0.5532, 0.3258)):
        u, v = ws.project_block(cam, R, (x, y), 0.060)
        check("s5 %s 在画幅内" % name,
              0.0 <= u < ws.SIZE[0] and 0.0 <= v < ws.SIZE[1], (u, v))

    # 旧位（stage4_clear_run.E9_SURVEY_TOOL0）当感知位会漏块 —— 反证
    old_tool0 = np.array([0.520, 0.300, 0.400])
    old_cam = old_tool0 + R @ ws.T_TOOL0_CAM_M
    u, v = ws.project_block(old_cam, R, (0.5376, 0.1732), 0.060)
    check("旧 E9_SURVEY_TOOL0 会把 s5 黄塔投出画幅（这就是要新位的原因）",
          not (0.0 <= u < ws.SIZE[0] and 0.0 <= v < ws.SIZE[1]), (u, v))


def section_coverage_pose():
    print("\n[2b] 任意位姿的覆盖核验（偏移视点用）")
    tool0, cam, R = ws.survey_tool0()
    base = ws.coverage()
    generic = ws.coverage_pose(ws.PICK_ZONE_M, cam, R)
    check("coverage_pose 与 coverage 逐位一致", base[0] == generic,
          (base[0], generic))
    off_cam = cam + np.array([-0.100, 0.030, 0.0])
    off = ws.coverage_pose(ws.PICK_ZONE_M, off_cam, R)
    check("偏移 100/30 mm 后仍有 >30 px 余量", min(off) > 30.0, off)
    check("偏移后左右余量不再对称", abs(off[0] - off[1]) > 200.0, off)
    # 相机平移 ⇒ tool0 同量平移（手眼刚体），位姿来源仍唯一
    off_tool0 = off_cam - R @ ws.T_TOOL0_CAM_M
    check("tool0 与相机同量平移",
          np.allclose(off_tool0 - tool0, [-0.100, 0.030, 0.0], atol=1e-9),
          off_tool0 - tool0)


def main():
    section_offset()
    section_coverage()
    section_coverage_pose()
    section_check_blocks()
    print('\n%d PASS / %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('失败项：' + ', '.join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
