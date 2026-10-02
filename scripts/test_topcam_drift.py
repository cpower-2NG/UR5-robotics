#!/usr/bin/env python3
"""俯视相机漂移判定的自检 —— 不需要 ROS、不需要真机、**不需要相片**。

（沿用本仓库 ``scripts/test_*.py`` 的约定：直接 ``python3`` 跑，退出码 0 = 通过。）

为什么值得单独钉一遍
====================
`topcam_drift_check.py` 要回答的那个问题 ——「**相机动了还是靶纸动了**」——
**两边都会让它报「动了」**，所以光看「有没有报动」永远测不出它对不对。
必须**造出已知答案的两帧**，看它分不分得开。

而这里最像「假绿」的一条是**空间铺开条件**：
一个**小块整体平移**，它自己的点 100% 落在同一个单应上（平移也是单应）。
只看「内点占比」⇒ 会把它报成「整幅一起动 ⇒ 相机动了」，
而正确答案恰恰相反。所以第 D 节把那条条件单拎出来钉。

五节：
  A. **静止**（只加噪声）         ⇒ 退出码 0、判「都没动」
  B. **相机转 1°**（真单应）       ⇒ 退出码 1、判「整幅一起动（相机）」
  C. **一小块平移 22 px**（模拟靶纸被碰） ⇒ 退出码 1、判「局部动」且**块找得对**
  D. **「铺开」判据本身**：挤在一角 ⇒ False；铺满画面 ⇒ True。
     这一节是**变异测试的目标** —— 把 `spans_image` 改成恒返回 True，
     C 必须跟着 FAIL。
  E. **最大连通块定位**：造一簇点，bbox 要罩住它、且不把远处的点圈进来。
"""

import os
import sys

import numpy as np

try:
    import cv2
except ImportError:                                          # pragma: no cover
    print("✗ 需要 cv2（python3-opencv）")
    sys.exit(2)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import topcam_drift_check as T                               # noqa: E402

_failed = []


def check(name, cond, detail=""):
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        _failed.append(name)


def scene(seed=7, w=1280, h=720):
    """造一张**有角点**的合成台面：随机暗块 + 轻噪声。

    纯白台面训不出特征点（真实场景里也正因为如此，本仓库才要光流而不是模板匹配）。
    """
    rng = np.random.RandomState(seed)
    img = np.full((h, w), 205, np.uint8)
    for _ in range(150):
        x, y = rng.randint(0, w - 70), rng.randint(0, h - 70)
        bw, bh = rng.randint(18, 70), rng.randint(18, 70)
        img[y:y + bh, x:x + bw] = rng.randint(20, 150)
    img = np.clip(img.astype(np.int16)
                  + rng.normal(0, 3, img.shape).astype(np.int16), 0, 255)
    return img.astype(np.uint8)


K = np.array([[917.62, 0, 649.55], [0, 916.43, 344.91], [0, 0, 1.0]])


def write_pair(a, b, d):
    pa, pb = os.path.join(d, "a.png"), os.path.join(d, "b.png")
    cv2.imwrite(pa, a)
    cv2.imwrite(pb, b)
    return pa, pb


def run_pair(pa, pb):
    """跑一次主脚本（子进程，取它真实的退出码）。"""
    import subprocess
    r = subprocess.run([sys.executable,
                        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "topcam_drift_check.py"),
                        "--old", pa, "--new", pb],
                       capture_output=True, text=True)
    return r.returncode, r.stdout


def part_abc(tmp):
    print("\n--- A/B/C 造两帧、看判定 ---")
    base = scene()

    # A. 静止：同图 + 噪声
    rng = np.random.RandomState(11)
    a2 = np.clip(base.astype(np.int16)
                 + rng.normal(0, 3, base.shape).astype(np.int16), 0, 255).astype(np.uint8)
    pa, pb = write_pair(base, a2, tmp)
    rc, out = run_pair(pa, pb)
    check("A 静止 ⇒ 退出码 0", rc == 0, "got %d" % rc)
    check("A 静止 ⇒ 判「都没动」", "都没动" in out)

    # B. 相机转 1°（绕相机 y 轴）—— 真·纯转动单应
    th = np.radians(1.0)
    R = np.array([[np.cos(th), 0, np.sin(th)],
                  [0, 1, 0],
                  [-np.sin(th), 0, np.cos(th)]])
    H = K @ R @ np.linalg.inv(K)
    b = cv2.warpPerspective(base, H, (base.shape[1], base.shape[0]),
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    pa, pb = write_pair(base, b, tmp)
    rc, out = run_pair(pa, pb)
    check("B 相机转 1° ⇒ 退出码 1", rc == 1, "got %d" % rc)
    check("B 相机转 1° ⇒ 判「整幅一起动（相机）」", "整幅一起动" in out)

    # C. 只把一小块平移 —— 模拟靶纸被碰
    c = base.copy()
    y0, y1, x0, x1 = 260, 460, 500, 820
    c[y0:y1, x0:x1] = base[y0 - 9:y1 - 9, x0 - 22:x1 - 22]
    pa, pb = write_pair(base, c, tmp)
    rc, out = run_pair(pa, pb)
    check("C 小块平移 ⇒ 退出码 1", rc == 1, "got %d" % rc)
    check("C 小块平移 ⇒ 判「局部动」", "局部动" in out)
    # 报出来的块必须落在真位置附近（允许粗网格 16 px 的量化）
    line = [l for l in out.splitlines() if "动的那块" in l]
    if line:
        nums = [int(v) for v in
                "".join(ch if ch.isdigit() or ch == " " else " "
                        for ch in line[0].split("x[")[1]).split()]
        bx0, bx1, by0, by1 = nums[0], nums[1], nums[2], nums[3]
        near = (bx0 <= x0 + 16 and bx1 >= x1 - 16
                and by0 <= y0 + 16 and by1 >= y1 - 16
                and bx0 > x0 - 120 and by0 > y0 - 120)
        check("C 报出的块罩住真位置 x[%d,%d] y[%d,%d]" % (x0, x1, y0, y1),
              near, "报的是 x[%d,%d] y[%d,%d]" % (bx0, bx1, by0, by1))
    else:
        check("C 打印了块的位置", False, "输出里没有「动的那块」")


def part_d():
    print("\n--- D 「铺开」判据（C 能不能被抓住全靠它）---")
    rng = np.random.RandomState(3)
    h, w = 720, 1280
    corner = rng.uniform([20, 20], [180, 180], size=(60, 2))
    spread = rng.uniform([20, 20], [w - 20, h - 20], size=(60, 2))
    ok_c, _ = T.spans_image(corner, (h, w))
    ok_s, why = T.spans_image(spread, (h, w))
    check("挤在一角 ⇒ 判「没铺开」", not ok_c)
    check("铺满画面 ⇒ 判「铺开」", ok_s, why)


def part_e():
    print("\n--- E 最大连通块定位 ---")
    rng = np.random.RandomState(5)
    blob = rng.uniform([400, 300], [520, 400], size=(80, 2))
    far = rng.uniform([900, 100], [1000, 180], size=(20, 2))
    pts = np.vstack([blob, far])
    got = T.largest_blob(pts, (720, 1280), cell=16)
    check("块找得到", got is not None)
    if got:
        covers = (got["x0"] <= 400 and got["x1"] >= 520
                  and got["y0"] <= 300 and got["y1"] >= 400)
        excludes = not (got["x0"] <= 950 <= got["x1"] and got["y0"] <= 140 <= got["y1"])
        check("块罩住目标簇", covers,
              "报 x[%d,%d] y[%d,%d]" % (got["x0"], got["x1"], got["y0"], got["y1"]))
        check("块没把远处那簇圈进来", excludes)


def main():
    import tempfile
    print("俯视相机漂移判定的自检")
    with tempfile.TemporaryDirectory() as tmp:
        part_abc(tmp)
        part_d()
        part_e()
    print("\n%s" % ("全部通过" if not _failed
                    else "FAIL %d 条：%s" % (len(_failed), "、".join(_failed))))
    return 1 if _failed else 0


if __name__ == "__main__":
    sys.exit(main())
