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

2026-11-07 起语义变了（E3a）：**「画面没动」不再是「通过」**。
相机静止与亚 2 px 漂移在像里不可分，所以第 1 条判据返回 **2（证据不足）**，
本工具**不再返回 0**。外参是否仍有效只能由 E3b 的 PnP 门回答。

九节：
  A. **静止**（只加噪声）         ⇒ 退出码 2、判「证据不足」（**不是 0**）
  B. **相机转 1°**（真单应）       ⇒ 退出码 1、判「整幅一起动（相机）」
  C. **一小块平移 22 px**（模拟靶纸被碰） ⇒ 退出码 1、判「局部动」且**块找得对**
  D. **「铺开」判据本身**：挤在一角 ⇒ False；铺满画面 ⇒ True。
  E. **最大连通块定位**：造一簇点，bbox 要罩住它、且不把远处的点圈进来。
  F. **无纹理**（平滑渐变、没有角点）⇒ 退出码 2，**绝不返回 0**。
  G. **合成漂移 1.0 / 2.0 / 5.0 px**：都**不许返回 0**；5 px 必须判「整幅一起动」。
  H. **同一静止对连跑 5 次** ⇒ 每次都 2，**没有一次静默放行**。
  I. **变异测试**：把代码改回旧语义 / 改掉关键常量，结论必须跟着变 ——
     证明 A/G/C 的期望不是空转。
  J. **`--info` 的两种形状**：顶层 `K` 与 `camera_info.K`（`precision_capture` 的
     `meta_NNN.json`）。2026-11-08 前只认第一种 ⇒ KeyError 让进程以 **1** 退出，
     把「证据不足 = 2」伪装成「检测到变化 = 1」。含一条变异测试钉住新分支。
"""

import json
import os
import re
import subprocess
import sys

import numpy as np

try:
    import cv2
except ImportError:                                          # pragma: no cover
    print("✗ 需要 cv2（python3-opencv）")
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "topcam_drift_check.py")
sys.path.insert(0, HERE)
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


def write_pair(a, b, d, tag=""):
    pa = os.path.join(d, "a%s.png" % tag)
    pb = os.path.join(d, "b%s.png" % tag)
    cv2.imwrite(pa, a)
    cv2.imwrite(pb, b)
    return pa, pb


def run_pair(pa, pb):
    """跑一次主脚本（子进程，取它真实的退出码）。"""
    r = subprocess.run([sys.executable, SCRIPT, "--old", pa, "--new", pb],
                       capture_output=True, text=True)
    return r.returncode, r.stdout


def run_pair_info(pa, pb, info):
    """同上，额外带 `--info`（内参）。"""
    r = subprocess.run([sys.executable, SCRIPT, "--old", pa, "--new", pb,
                        "--info", info], capture_output=True, text=True)
    return r.returncode, r.stdout


def _run_mutated(pa, pb, tmp, tag, subs, extra=()):
    """把主脚本按 ``subs``（[(正则, 替换), ...]）改掉再跑 —— 变异测试用。

    每条替换必须**恰好命中一次**；否则测试装置本身坏了，直接抛。
    """
    src = open(SCRIPT, encoding="utf-8").read()
    for pat, repl in subs:
        src, n = re.subn(pat, repl, src, count=1, flags=re.M)
        if n != 1:
            raise RuntimeError("变异失败：%r 命中 %d 次" % (pat, n))
    path = os.path.join(tmp, "mut-%s.py" % tag)
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)
    r = subprocess.run([sys.executable, path, "--old", pa, "--new", pb] + list(extra),
                       capture_output=True, text=True)
    return r.returncode, r.stdout


def _mut_const(name, value):
    """造一条「把常量 NAME 改成 value」的替换对。"""
    return (r"^%s\s*=\s*.*$" % re.escape(name), "%s = %r" % (name, value))


def shifted(img, dx, dy):
    """整幅平移 ``(dx, dy)`` 像素（双线性，边界反射，避免造出黑边假角点）。"""
    M = np.float32([[1.0, 0.0, dx], [0.0, 1.0, dy]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]),
                          flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT)


def part_abc(tmp):
    print("\n--- A/B/C 造两帧、看判定 ---")
    base = scene()

    # A. 静止：同图 + 噪声 ⇒ 证据不足（2），不是通过（0）
    rng = np.random.RandomState(11)
    a2 = np.clip(base.astype(np.int16)
                 + rng.normal(0, 3, base.shape).astype(np.int16), 0, 255).astype(np.uint8)
    pa, pb = write_pair(base, a2, tmp, "-a")
    rc, out = run_pair(pa, pb)
    check("A 静止 ⇒ 退出码 2", rc == 2, "got %d" % rc)
    check("A 静止 ⇒ 判「证据不足」", "证据不足" in out)
    # 注意：正文里那句「不再给『外参还有效』的结论」是**反话**，别被它误判；
    # 这里钉的是「不许出现旧的放行结论」。
    check("A 静止 ⇒ 不再说「都没动/外参有效」",
          ("都没动" not in out) and ("外参有效" not in out))

    # B. 相机转 1°（绕相机 y 轴）—— 真·纯转动单应
    th = np.radians(1.0)
    R = np.array([[np.cos(th), 0, np.sin(th)],
                  [0, 1, 0],
                  [-np.sin(th), 0, np.cos(th)]])
    H = K @ R @ np.linalg.inv(K)
    b = cv2.warpPerspective(base, H, (base.shape[1], base.shape[0]),
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    pa, pb = write_pair(base, b, tmp, "-b")
    rc, out = run_pair(pa, pb)
    check("B 相机转 1° ⇒ 退出码 1", rc == 1, "got %d" % rc)
    check("B 相机转 1° ⇒ 判「整幅一起动（相机）」", "整幅一起动" in out)

    # C. 只把一小块平移 —— 模拟靶纸被碰
    c = base.copy()
    y0, y1, x0, x1 = 260, 460, 500, 820
    c[y0:y1, x0:x1] = base[y0 - 9:y1 - 9, x0 - 22:x1 - 22]
    pa, pb = write_pair(base, c, tmp, "-c")
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

    return base


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


def part_f(tmp):
    print("\n--- F 无纹理画面（必须放不下行）---")
    # 平滑横向渐变：没有角点。真实场景里「画面太白/太糊」就是这样。
    ramp = np.tile(np.linspace(90, 210, 1280, dtype=np.uint8), (720, 1))
    ramp = cv2.GaussianBlur(ramp, (0, 0), 3)
    pa, pb = write_pair(ramp, ramp, tmp, "-f")
    rc, out = run_pair(pa, pb)
    check("F 无纹理 ⇒ 退出码 2", rc == 2, "got %d" % rc)
    check("F 无纹理 ⇒ 明确说不放行",
          ("特征点太少" in out) or ("证据不足" in out))


def part_g(tmp):
    print("\n--- G 合成漂移 1.0 / 2.0 / 5.0 px ---")
    base = scene(seed=21)
    results = {}
    for px in (1.0, 2.0, 5.0):
        pa, pb = write_pair(base, shifted(base, px, 0.0), tmp, "-g%g" % px)
        rc, out = run_pair(pa, pb)
        results[px] = (rc, out, pa, pb)
        check("G %.1f px ⇒ **不是 0**" % px, rc != 0, "got %d" % rc)
    check("G 1.0 px ⇒ 判「证据不足」", "证据不足" in results[1.0][1])
    check("G 5.0 px ⇒ 判「整幅一起动」", "整幅一起动" in results[5.0][1])
    # 2 px 正好在阈值上：判 1 或 2 都算「拦住」，但不许放行
    check("G 2.0 px ⇒ 判「整幅一起动」或「证据不足」",
          ("整幅一起动" in results[2.0][1]) or ("证据不足" in results[2.0][1]))
    return results


def part_h(tmp):
    print("\n--- H 同一静止对连跑 5 次（无静默放行）---")
    base = scene(seed=31)
    rng = np.random.RandomState(32)
    a2 = np.clip(base.astype(np.int16)
                 + rng.normal(0, 3, base.shape).astype(np.int16), 0, 255).astype(np.uint8)
    pa, pb = write_pair(base, a2, tmp, "-h")
    outcomes = [run_pair(pa, pb) for _ in range(5)]
    codes = [c for c, _ in outcomes]
    check("5 次退出码全是 2", all(c == 2 for c in codes), "got %s" % codes)
    check("5 次没有一次 0", 0 not in codes, "got %s" % codes)
    check("5 次都判「证据不足」",
          all("证据不足" in o for _, o in outcomes))


def part_i(tmp, base_g, results_g):
    print("\n--- I 变异测试（期望不是空转）---")

    # I1. 把静止分支改回**旧的 fail-open 语义**（「都没动」、退出码 0）：
    #     A 的两条期望必须跟着翻 —— 证明 A 真的挡住了旧语义。
    pa, pb = write_pair(scene(seed=11), scene(seed=11), tmp, "-i1")
    old_line = ('verdict, code = "证据不足（画面没动，无法区分相机漂移与静止）", 2')
    rc0, out0 = run_pair(pa, pb)
    rc1, out1 = _run_mutated(pa, pb, tmp, "i1-failopen", [
        (re.escape(old_line), 'verdict, code = "都没动", 0'),
        # 旧语义的退出码映射里本来就有 0；一起改掉，否则 KeyError 变成退出码 1
        (re.escape('        1: "检测到变化，外参作废；重跑 ⓪①",'),
         '        0: "都没动（旧语义）",\n        1: "检测到变化，外参作废；重跑 ⓪①",')])
    check("I1 原脚本静止 ⇒ 2", rc0 == 2, "got %d" % rc0)
    check("I1 改回旧语义 ⇒ 0（A 的期望会红）", rc1 == 0, "got %d" % rc1)
    check("I1 改回旧语义 ⇒ 不再说「证据不足」", "证据不足" not in out1)

    # I2. 把静态点残差阈值抬到无穷（让动的那块的单应「解释」一切）：
    #     C 的「局部动」必须消失 —— 证明 C 靠的正是那条判据。
    base = scene(seed=7)
    c = base.copy()
    y0, y1, x0, x1 = 260, 460, 500, 820
    c[y0:y1, x0:x1] = base[y0 - 9:y1 - 9, x0 - 22:x1 - 22]
    pa, pb = write_pair(base, c, tmp, "-i2")
    _, out_ok = run_pair(pa, pb)
    _, out_mut = _run_mutated(pa, pb, tmp, "i2-stayer",
                              [_mut_const("STAYER_OK_PX", 1e9)])
    check("I2 原脚本 C ⇒ 局部动", "局部动" in out_ok)
    check("I2 变异后 C ⇒ 不再判「局部动」（静态点判据真的在用）",
          "局部动" not in out_mut)

    # I3. 把「算动了」的阈值抬到无穷：5 px 的「整幅一起动」必须翻成证据不足 ——
    #     证明 G 走的确实是「先数动了的点」这条路，不是别的分支。
    rc0, out0 = results_g[5.0][0], results_g[5.0][1]
    pa, pb = results_g[5.0][2], results_g[5.0][3]
    rc1, out1 = _run_mutated(pa, pb, tmp, "i3-moving",
                             [_mut_const("MOVING_PX", 1e9)])
    check("I3 原脚本 5 px ⇒ 1", rc0 == 1, "got %d" % rc0)
    check("I3 变异后 5 px ⇒ 2（MOVING_PX 真的在用）", rc1 == 2, "got %d" % rc1)


def part_j(tmp):
    print("\n--- J --info 的两种形状（顶层 K / camera_info.K）---")
    base = scene(seed=41)
    th = np.radians(1.0)
    R = np.array([[np.cos(th), 0, np.sin(th)],
                  [0, 1, 0],
                  [-np.sin(th), 0, np.cos(th)]])
    Hm = K @ R @ np.linalg.inv(K)
    b = cv2.warpPerspective(base, Hm, (base.shape[1], base.shape[0]),
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    pa, pb = write_pair(base, b, tmp, "-j")

    info_top = os.path.join(tmp, "info-top.json")
    info_meta = os.path.join(tmp, "meta-j.json")
    with open(info_top, "w", encoding="utf-8") as f:
        json.dump({"K": K.reshape(-1).tolist()}, f)
    with open(info_meta, "w", encoding="utf-8") as f:
        # precision_capture.py 的 meta_NNN.json 形状
        json.dump({"camera": "topcam",
                   "camera_info": {"width": 1280, "height": 720,
                                   "K": K.reshape(-1).tolist()}}, f)

    ang_re = re.compile(r"单应分解（用内参）：旋转 \*\*([\d.]+)°\*\*")

    rc_t, out_t = run_pair_info(pa, pb, info_top)
    check("J 顶层 K ⇒ 退出码 1（旧形状照样能用）", rc_t == 1, "got %d" % rc_t)
    m_t = ang_re.search(out_t)
    check("J 顶层 K ⇒ 打出单应分解", m_t is not None)

    rc_m, out_m = run_pair_info(pa, pb, info_meta)
    check("J camera_info.K ⇒ 退出码 1（不再 KeyError）", rc_m == 1, "got %d" % rc_m)
    m_m = ang_re.search(out_m)
    check("J camera_info.K ⇒ 打出单应分解", m_m is not None)
    check("J 两种形状解出同一个角度",
          m_t is not None and m_m is not None
          and abs(float(m_t.group(1)) - float(m_m.group(1))) < 0.01,
          "%s vs %s" % (m_t.group(1) if m_t else "-", m_m.group(1) if m_m else "-"))

    # 证据不足分支**带着 info** 也必须返回 2，不能崩成 1
    rng = np.random.RandomState(43)
    a2 = np.clip(base.astype(np.int16)
                 + rng.normal(0, 3, base.shape).astype(np.int16), 0, 255).astype(np.uint8)
    ps, pt = write_pair(base, a2, tmp, "-j2")
    rc_s, out_s = run_pair_info(ps, pt, info_meta)
    check("J 静止 + camera_info.K ⇒ 退出码 2（不是崩成 1）", rc_s == 2, "got %d" % rc_s)
    check("J 静止 + camera_info.K ⇒ 判「证据不足」", "证据不足" in out_s)

    # 变异：把嵌套查找删掉 ⇒ J 的两条「打出单应分解」必须红
    nested_line = '        k = (obj.get("camera_info") or {}).get("K")'
    rc_v, out_v = _run_mutated(pa, pb, tmp, "j-nested",
                               [(re.escape(nested_line), "        k = None")],
                               extra=("--info", info_meta))
    check("J 变异（删嵌套查找）⇒ 不再打单应分解（J 的期望不空转）",
          "单应分解（用内参）" not in out_v, "got %d" % rc_v)


def main():
    import tempfile
    print("俯视相机漂移判定的自检（E3a：证据不足 ⇒ 2，不再返回 0）")
    with tempfile.TemporaryDirectory() as tmp:
        part_abc(tmp)
        part_d()
        part_e()
        part_f(tmp)
        results_g = part_g(tmp)
        part_h(tmp)
        part_i(tmp, scene(seed=21), results_g)
        part_j(tmp)
    print("\n%s" % ("全部通过" if not _failed
                    else "FAIL %d 条：%s" % (len(_failed), "、".join(_failed))))
    return 1 if _failed else 0


if __name__ == "__main__":
    sys.exit(main())
