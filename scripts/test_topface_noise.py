#!/usr/bin/env python3
"""顶面误差统计的守卫 —— 让 `topface_noise.py` 报出来的 σ 不能是假的。

为什么要它
==========
`topface_noise.py` 的结论（单帧 σ≈2.5 mm、融合能到 ~1.0 mm）会直接决定
下一步要不要开时间融合、还要不要继续打磨估计器。这类**统计估计器**有一个
阴险的失败模式：它永远会给你一个数。算错了、聚合错了、样本是零的时候，
它照样打印「σ = 0.00 mm」—— 看起来完美，其实是**自证式绿灯**。

本仓已经踩过一次：`topface_noise.py` 的第一版用滚动中位做融合，窗口与自身
重叠，N 越大 std 越接近 0，N=9 时显示 0.00 mm，而真实单帧 σ 是 2.5 mm。
如果当时有一条「喂已知 σ 的合成数据，看它能不能还原」的守卫，那一版当场
就会红。本脚本就是这条守卫。

它判什么
========
1. **能还原已知真值**：喂合成数据（每块常数 + 已知 σ 的高斯噪声），
   `summarize` 报出的单帧 σ、方差拆解、融合 σ 必须落在真值附近。
   σ=0 的数据必须报 0 —— 两条一起，才排得掉「总是 0」和「总是很大」。
2. **融合不能用重叠窗口、聚合必须是 RMS**：合成数据帧间独立时，N 帧的
   融合 σ 应接近 σ₁·√(MEDIAN_VAR[n])（半半对照两边取**中位**，所以对的
   不是 σ₁/√N）。滚动中位那种错法会给出远小于理论的值；「均值 |差|」代替
   RMS 只小 20%，落在统计容差里，另有**手算**单测钉住（4 块差 [0,0,0,2]
   → 0.71 mm，均值法给 0.35 mm）。
3. **分块与半半切分本身正确**：`group_blocks` 的容差、`split_half_diff`
   的「前 n 个中位 − 后 n 个中位」及帧不够时返回 None。
4. **真实帧冒烟**（有基准帧才跑）：10 帧重复画面里能找到块、σ **不为 0**。
   不断言具体数值 —— 那是测量结果，不是契约；约定了就会有人为了让测试变绿
   去改数据。

判据里的容差怎么来的
====================
合成数据用固定种子，所以数值确定；容差按**估计量自身的抽样散布**量出来，
不是拍脑袋：40 块 × 10 帧、换 20 个种子跑，融合 σ 与中位理论之比落在
0.83–1.20，σ 还原更稳。于是在「40 块」这个配置下，融合档取 ±30%、其余
取 ±25%（比实测带宽再留一点，免得换 numpy 版本换个流就红）。容差再松就
没有诊断力（σ 报成两倍也绿）；想收紧，先把块数加到 120 以上。

⚠️ 统计容差本身抓不住 20% 量级的**固定偏置**（比如均值代替 RMS），
所以那条用在手算样例上做精确判定，不靠容差。

用法
====
    python3 scripts/test_topface_noise.py [帧目录 ...]

不给目录时用 ``~/benchmark/2026-10-baseline/frames/wrist/rep-*``；找不到
就只跑合成部分并 [SKIP] 冒烟（基准帧在仓库外，换台机器必然缺）。
退出码 0 = 全通过（或跳过冒烟），1 = 有失败项。
"""

import glob
import os
import sys
from pathlib import Path

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import topface_noise as T  # noqa: E402

DEFAULT_GLOB = str(Path.home() / "benchmark" / "2026-10-baseline"
                   / "frames" / "wrist" / "rep-*")
TOL_REL = 0.25          # 合成数据还原真值的相对容差（见 docstring）
FUSE_TOL_REL = 0.30     # 融合 σ 相对**中位**理论的容差（中位比均值差 25%，
                        # 不单独放宽就对不上；0.30 仍能逮住滚动中位的 0）
SMOKE_SIGMA_RANGE = (0.5, 6.0)   # 真实帧上「不为 0、也没炸」的量级带

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def synth_entries(sigma_above_mm, sigma_plane_mm, n_frames, n_blocks=40, seed=0):
    """每块一个常数高度 + 已知 σ 的高斯噪声。

    ``bz = plane_z + above_m``（与真实记录的含义一致：局部桌面高度 + 高出量）。
    块与块之间距离 40 mm（远大于 ``GROUP_TOL_MM``），所以分块必须是
    一帧一块、一块一组。
    """
    rng = np.random.default_rng(seed)
    rows = []
    for b in range(n_blocks):
        x_mm = (b % 8) * 40.0
        y_mm = (b // 8) * 40.0
        z0_mm = 100.0 + 3.0 * b
        for i in range(n_frames):
            plane = z0_mm + rng.normal(0.0, sigma_plane_mm)
            above = 30.0 + rng.normal(0.0, sigma_above_mm)
            rows.append({"frame": f"f{i:02d}",
                         "bx": x_mm / 1e3, "by": y_mm / 1e3,
                         "bz": (plane + above) / 1e3,
                         "plane_z": plane / 1e3,
                         "above_m": above / 1e3})
    return rows


def near(got, want, rel):
    return abs(got - want) <= rel * abs(want)


def test_split_half():
    print("\n--- split_half_diff ---")
    check('帧不够时返回 None', T.split_half_diff([1.0, 2.0], 3) is None)
    check('正好 2n 帧时可用', T.split_half_diff([1.0, 2.0, 3.0, 4.0], 2) is not None)
    d3 = T.split_half_diff([1, 2, 3, 4, 5, 6], 3)
    check('n=3：(1,2,3) 中位 − (4,5,6) 中位 = −3', d3 == -3.0, f'得到 {d3}')
    d1 = T.split_half_diff([1, 2, 3, 4, 5, 6], 1)
    check('n=1：首 − 末 = −5', d1 == -5.0, f'得到 {d1}')
    # 必须是「前 n vs 后 n」而不只是「两组」：反过来喂要变号。
    d_rev = T.split_half_diff([6, 5, 4, 3, 2, 1], 3)
    check('前后位置有意义（倒序变号）', d_rev == 3.0, f'得到 {d_rev}')


def test_group_blocks():
    print("\n--- group_blocks ---")
    tol = T.GROUP_TOL_MM
    mk = lambda x, y: {"bx": x / 1e3, "by": y / 1e3, "bz": 0.1,
                       "plane_z": 0.07, "above_m": 0.03, "frame": "f"}
    groups = T.group_blocks([mk(0, 0), mk(tol * 0.5, 0)])
    check(f'{tol * 0.5:.0f} mm 内的估计并入同一块', len(groups) == 1,
          f'{len(groups)} 组')
    check('并入后记录数正确', len(groups[0][1]) == 2)
    groups = T.group_blocks([mk(0, 0), mk(tol * 2.0, 0)])
    check(f'{tol * 2:.0f} mm 外的估计分成两块', len(groups) == 2,
          f'{len(groups)} 组')
    # 分块是**链式**的：1 mm 步长连成一串仍算同一块（位置随均值移动）。
    chain = [mk(i * 1.0, 0) for i in range(6)]
    check('链式近邻仍归一块', len(T.group_blocks(chain)) == 1)


def test_summarize_recovery():
    print("\n--- summarize 还原已知 σ（σ=0 与 σ>0 都要对）---")
    s_above, s_plane, n_frames = 2.5, 1.0, 10
    rows = synth_entries(s_above, s_plane, n_frames)
    s = T.summarize(rows)
    want_bz = float(np.hypot(s_above, s_plane))
    check('分块数 = 合成的块数', s['blocks'] == 40, f"{s['blocks']} 块")
    check('块内帧数 = 合成的帧数',
          all(b['n'] == n_frames for b in s['per_block']))
    check('单帧 σ 还原', near(s['single_frame_sigma_mm'], want_bz, TOL_REL),
          f"得到 {s['single_frame_sigma_mm']:.2f}，应为 {want_bz:.2f} mm")
    parts = s['sigma_parts_mm']
    check('局部桌面 σ 还原', near(parts['local_plane'], s_plane, TOL_REL),
          f"得到 {parts['local_plane']:.2f}，应为 {s_plane:.2f} mm")
    check('高出桌面 σ 还原', near(parts['above_plane'], s_above, TOL_REL),
          f"得到 {parts['above_plane']:.2f}，应为 {s_above:.2f} mm")

    # ⚠️ 这一条是反「自证式绿灯」的核心：滚动中位那种错法会把融合 σ 压到
    # 远低于理论（甚至 0）。合成数据帧间是独立的，所以理论的成立条件在这里
    # 真成立 —— 达不到就是估计器错了，不是数据的问题。
    # 对照用**中位理论** σ₁√(π/2N)：半半对照两边取的是中位，不是均值。
    fus = s.get('temporal_fusion') or {}
    for n in T.FUSE_N:
        key = str(n)
        if key not in fus:
            check(f'N={n} 融合有结果', False)
            continue
        got = fus[key]['sigma_mm']
        theory = want_bz * float(np.sqrt(T.MEDIAN_VAR[n]))
        check(f'N={n} 融合 σ 接近中位理论（不是 0）',
              abs(got - theory) <= FUSE_TOL_REL * theory,
              f'得到 {got:.2f}，理论 {theory:.2f} mm')
        # 两个参考值的标签不能互换：中位融合比均值融合**差**（√MEDIAN_VAR·n>1）。
        check(f'N={n} 中位理论 > 均值理论',
              fus[key]['median_theory_mm'] > fus[key]['sqrt_n_theory_mm'],
              f"中位 {fus[key]['median_theory_mm']} / 均值 "
              f"{fus[key]['sqrt_n_theory_mm']}")

    print("\n--- σ=0 的数据必须报 0（否则「总是报个数」也会蒙混过关）---")
    flat = T.summarize(synth_entries(0.0, 0.0, 10, n_blocks=10))
    check('σ=0 → 单帧 σ 报 0', flat['single_frame_sigma_mm'] == 0.0,
          f"得到 {flat['single_frame_sigma_mm']}")
    fus0 = flat.get('temporal_fusion') or {}
    # 帧数要够最大的 FUSE_N，否则那段会**缺键**而不是报 0 —— 缺键也算红。
    check('σ=0 → 每个融合档都有结果', set(fus0) == {str(n) for n in T.FUSE_N},
          f'得到 {sorted(fus0)}')
    check('σ=0 → 融合 σ 也报 0',
          bool(fus0) and all(d['sigma_mm'] == 0.0 for d in fus0.values()),
          str({k: d['sigma_mm'] for k, d in fus0.items()}))

    print("\n--- 帧数不足的块被剔除 ---")
    rows = synth_entries(1.0, 1.0, 4, n_blocks=3)
    rows += [dict(r, frame=f"g{i}") for i, r in
             enumerate(synth_entries(1.0, 1.0, 2, n_blocks=1, seed=1))]
    s = T.summarize(rows)
    check('只有 2 帧的块不进统计', s['blocks'] == 3, f"{s['blocks']} 块")
    check('帧名去重且有序', s['frames'] == sorted(set(s['frames'])),
          str(s['frames']))

    print("\n--- 内部字段不外泄 ---")
    check('per_block 里没有 _bz_ordered',
          all('_bz_ordered' not in b for b in s['per_block']))


def test_aggregation_exact():
    """聚合公式的**精确**单测：跨块的 RMS，不是均值。

    统计容差抓不住它 —— 用「均值 |差|」代替 RMS 只会系统性小 20%，落在我
    给合成数据留的容差里。那就构造一组**手算得出**的差：四块，各自
    「前 3 帧 100、后 3 帧 100−d」，d = 0,0,0,2 mm ⇒ 差 = 0,0,0,2。
      · RMS：√(mean([0,0,0,4]) / 2) = √0.5 = 0.7071 mm；
      · 均值：mean([0,0,0,2]) / √2 = 0.3536 mm —— 差一倍，判据必须落在前一个。
    """
    print("\n--- 聚合公式（RMS vs 均值）---")
    rows = []
    for b, d in enumerate([0.0, 0.0, 0.0, 2.0]):
        vals = [100.0] * 3 + [100.0 - d] * 3
        rows += [{"frame": f"f{i:02d}", "bx": b * 40.0 / 1e3, "by": 0.0,
                  "bz": v / 1e3, "plane_z": 0.0, "above_m": v / 1e3}
                 for i, v in enumerate(vals)]
    fus = (T.summarize(rows).get('temporal_fusion') or {}).get('3') or {}
    got = fus.get('sigma_mm')
    check('4 块 [0,0,0,2] → σ = RMS/√2 = 0.71 mm（均值法会给 0.35）',
          got is not None and abs(got - 0.7071) < 0.006, f'得到 {got}')


def test_real_frames(dirs):
    print("\n--- 真实帧冒烟（有基准帧才跑）---")
    rows = T.collect(dirs)
    s = T.summarize(rows)
    check('真实帧上找到了块', s['blocks'] >= 5, f"{s['blocks']} 块")
    if s['blocks']:
        sigma = s['single_frame_sigma_mm']
        lo, hi = SMOKE_SIGMA_RANGE
        check('单帧 σ 不为 0 且量级合理', lo <= sigma <= hi,
              f'σ={sigma:.2f} mm，带 [{lo}, {hi}]')


def main(argv):
    print("=" * 66)
    print("顶面误差统计守卫")
    print("=" * 66)

    test_split_half()
    test_group_blocks()
    test_aggregation_exact()
    test_summarize_recovery()

    dirs = argv[1:] or sorted(glob.glob(DEFAULT_GLOB))
    if not dirs:
        print(f"\n[SKIP] 冒烟：没找到基准帧 {DEFAULT_GLOB}")
        print("       基准帧在仓库外（公开仓不存真实画面），换台机器必然缺。")
    elif not all(os.path.isdir(d) for d in dirs):
        print(f'\n[SKIP] 冒烟：不是目录 {[d for d in dirs if not os.path.isdir(d)][:3]}')
    else:
        test_real_frames(dirs)

    print("\n" + "=" * 66)
    if _failed:
        print(f'❌ {len(_failed)} 项没过：')
        for n in _failed:
            print(f'   - {n}')
        return 1
    print('✅ 全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
