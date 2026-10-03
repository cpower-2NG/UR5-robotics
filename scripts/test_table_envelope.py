#!/usr/bin/env python3
"""`table_safety_envelope.py` 的纯函数自检（不需要 MoveIt、不需要真机）。

只测**不碰 ROS 图**的那一半：四个 tool0 高度怎么算、单个位姿怎么分档、几层怎么
取最坏、图怎么渲染、每行边界怎么切、以及从 `pipeline.launch.py` 读真机常量。

真扫描那部分要靠活的 /compute_ik，由 `moveit_only_check.py --envelope-z` 驱动
（板端跑）。**别在这里模拟它** —— 一个假的服务回话会把「包线算对了」证成废话。

⚠️ 这些断言**钉住的是真机几何的口径**（台面 + 方块高 + 指尖偏置 怎么组合出四个
   高度、取最坏时谁赢）。改口径必须同时改这里的期望值 —— 那时你会看到它红，
   而不是让包线悄悄换了含义还把图照画出来。
⚠️ 末尾那条交叉检查钉的是**两个文件之间的关系**（`pipeline.launch.py` 的
   REAL_TOOL0_TO_FINGERTIP 与 envelope 的 REAL_BLOCK_HEIGHT 合起来必须是 AG-95
   闭合时的 204.2 mm）—— 改任一个而不改另一个，它会红。

跑法：
    source /opt/ros/humble/setup.bash
    python3 scripts/test_table_envelope.py
退出码 0 = 全通过。
"""

import os
import sys

# ⚠️ 必须在 import 被测模块**之前**设：`.pyc` 的失效判据是 (mtime, size)，
# 一次**等长**的改动落在**同一秒**里就会被当成「没变」而继续用旧字节码 ——
# 第八十四轮做变异测试时真的撞上（详见 `test_plot_table_envelope.py` 里同一段）。
sys.dont_write_bytecode = True

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from table_safety_envelope import (                          # noqa: E402
        BLOCKED, LEVEL_KEYS, NEAR, NOIK, PROJECT_ROOT, REAL_BLOCK_HEIGHT, SAFE,
        SOLVABLE_OTHER, axis, cell_key, classify_one, combine, json_axis,
        launch_const, max_rect, node_guard_note, num, pick_zone_z,
        pick_zone_z_note, render_map, row_runs, worst, z_levels, PICK_Z_PAD)
except ImportError as e:              # 没 source ROS 时 rclpy 拉不到
    print(f"环境不满足（{e}）—— 先 source /opt/ros/humble/setup.bash")
    sys.exit(2)

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
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def tup_near(a, b, tol=1e-12):
    """逐项比，**不许用 `==` 比带加法的浮点元组**。

    这个坑本文件 D2/D4 两节早就写过一次（`0.10000000000000009`），我写 F2c 时
    又踩了一遍：`0.10 + 0.05 = 0.15000000000000002`，于是
    `pick_zone_z(0.0, 0.10, 0.05) == (0.05, 0.15, 0.10)` 红，而代码是对的。
    **一条因为浮点相等而红的断言，会把注意力引到没有错的地方去。**
    """
    return len(a) == len(b) and all(near(x, y, tol) for x, y in zip(a, b))


def calls(fn):
    """`("ok", 返回值)` 或 `("raise", 异常类名)` —— 给「会不会抛」这类断言用。

    直接写 `fn(...) is None` 的问题是：守卫一去掉，异常就从 check() 的**实参**里
    抛出来，脚本当场中断，后面所有检查一条都不跑。这时的红是「崩了」而不是
    「哪条错了」，跟它要防的 bug 一样难查。
    """
    try:
        return ("ok", fn())
    except Exception as e:
        return ("raise", type(e).__name__)


# ----------------------------------------------------------------------
# A. 四个 tool0 高度：真机口径
# ----------------------------------------------------------------------
print("\n--- A. z_levels：四个高度与它们的**大小顺序** ---")

Z = z_levels(0.0, REAL_BLOCK_HEIGHT, 0.1757, 0.0156, 0.05)
check("抓取下降 = 台面 + 方块高 + 指尖偏置 = 0.2042（指尖碰台面那个高度）",
      near(Z[0], 0.2042), f"got {Z[0]!r}")
check("抓取接近 = 抓取下降 + clearance", near(Z[1], 0.2542), f"got {Z[1]!r}")
check("放置下降 = 抓取下降 + place_release_lift", near(Z[2], 0.2198), f"got {Z[2]!r}")
check("放置接近 = 放置下降 + clearance", near(Z[3], 0.2698), f"got {Z[3]!r}")
check("顺序必须是 抓取下降 < 放置下降 < 抓取接近 < 放置接近",
      Z[0] < Z[2] < Z[1] < Z[3], f"got {Z!r}")
check("层级 key 与 z_levels 的顺序一一对应（错位会让图上的层名全错）",
      LEVEL_KEYS == ("grasp_descend", "grasp_approach",
                     "place_descend", "place_approach"), f"got {LEVEL_KEYS!r}")
# 台面整体平移 ⇒ 四个高度整体平移（这是「base_frame 下臂装在桌面上」的定义）
Z2 = z_levels(-0.39, REAL_BLOCK_HEIGHT, 0.1757, 0.0156, 0.05)
check("台面变化只平移四个高度（仿真的 -0.39 → 每个都低 0.39）",
      all(near(a - b, 0.39) for a, b in zip(Z, Z2)), f"got {Z2!r}")


# ----------------------------------------------------------------------
# B. axis：等距采样
# ----------------------------------------------------------------------
print("\n--- B. axis：网格轴 ---")
xs = axis(0.15, 0.95, 0.05)
check("整除时含两端（0.15→0.95 步 0.05 ⇒ 17 点）",
      len(xs) == 17 and near(xs[0], 0.15) and near(xs[-1], 0.95),
      f"len={len(xs)} first={xs[0]} last={xs[-1]}")
xs2 = axis(0.0, 0.10, 0.03)
check("不整除时末点**略小于**上界且步长保持均匀（不是硬塞一个端点进去）",
      near(xs2[-1], 0.09) and len(xs2) == 4, f"got {xs2!r}")
check("step 大于跨度时不炸（至少给一个点）", len(axis(0.0, 0.01, 0.05)) >= 1)


# ----------------------------------------------------------------------
# C. classify_one：分档
# ----------------------------------------------------------------------
print("\n--- C. classify_one：单个位姿 ---")
FLOOR, BAND = 0.02, 0.05
cases = [
    ("没解出来（None）", None, NOIK),
    ("解出来但 FK 给空（空 dict）", {}, NOIK),
    ("连杆恰好在**下限上** ⇒ 不算撞桌（判据是严格小于）",
     {"forearm_link": FLOOR}, NEAR),
    ("比下限低一点点 ⇒ 撞桌", {"forearm_link": FLOOR - 1e-9}, BLOCKED),
    ("恰好在近限带边界 ⇒ 算安全", {"forearm_link": FLOOR + BAND}, SAFE),
    ("刚进近限带 ⇒ 近限", {"forearm_link": FLOOR + BAND - 1e-9}, NEAR),
    ("取的是**最低**那个连杆，不是第一个", {"forearm_link": 5.0, "wrist_1_link": 0.0},
     BLOCKED),
]
for label, zs, exp in cases:
    got = classify_one(zs, FLOOR, BAND)
    check(label, got == exp, f"期望 {exp}，得到 {got}")


# ----------------------------------------------------------------------
# D. worst：几层取最坏
# ----------------------------------------------------------------------
print("\n--- D. worst：一个格子跑四个高度，取最坏 ---")
for label, syms, exp in [
        ("撞桌赢过一切", [SAFE, NEAR, NOIK, BLOCKED], BLOCKED),
        ("撞桌赢过解不出", [NOIK, BLOCKED], BLOCKED),
        ("解不出赢过种子问题", [SAFE, SOLVABLE_OTHER, NOIK], NOIK),
        ("种子问题赢过近限/安全", [SAFE, NEAR, SOLVABLE_OTHER], SOLVABLE_OTHER),
        ("近限赢过安全", [SAFE, NEAR], NEAR),
        ("全安全还是安全", [SAFE, SAFE], SAFE)]:
    got = worst(syms)
    check(label, got == exp, f"期望 {exp}，得到 {got}")


# ----------------------------------------------------------------------
# D2. combine：配的种子 + 换种子重试
# ----------------------------------------------------------------------
print("\n--- D2. combine：只有配的种子全解不出时才动重试 ---")
for label, primary, retry, exp in [
        # 配的种子能解 ⇒ 重试不参与（否则会把规划器真要走的解盖掉）
        ("配的种子解得出来 ⇒ 忽略重试结果，哪怕它撞桌",
         [SAFE], [BLOCKED], SAFE),
        ("配的种子解不出、换种子撞桌 ⇒ **判撞桌**（对的保守侧：那个位置有分支会撞）",
         [NOIK, NOIK], [NOIK, BLOCKED], BLOCKED),
        ("配的种子解不出、换种子也解不出 ⇒ 够不到",
         [NOIK], [NOIK], NOIK),
        ("配的种子解不出、换种子解得出来且安全 ⇒ 种子问题",
         [NOIK], [SAFE], SOLVABLE_OTHER),
        ("配的种子解不出、换种子只是近限 ⇒ 仍然算种子问题（能解出来）",
         [NOIK], [NEAR], SOLVABLE_OTHER),
        ("没重试（None）⇒ 原样", [NOIK], None, NOIK)]:
    got = combine(primary, retry)
    check(label, got == exp, f"期望 {exp}，得到 {got}")


# ----------------------------------------------------------------------
# E. render_map：图的朝向与列对齐
# ----------------------------------------------------------------------
print("\n--- E. render_map：y 从大到小、列宽 4 ---")
XS, YS = [0.00, 0.10], [-0.10, 0.00]
CELLS = {(0.00, -0.10): SAFE, (0.10, -0.10): BLOCKED,
         (0.00, 0.00): SAFE, (0.10, 0.00): SAFE}
lines = render_map(CELLS, XS, YS).split("\n")
check("表头是 '  y\\x  ' + 每列 4 字符",
      lines[0] == "  y\\x  0.000.10", f"got {lines[0]!r}")
check("一共 len(ys)+1 行", len(lines) == len(YS) + 1, f"got {len(lines)}")
check("**第一行数据是 y 最大那行**（图是「远在上、近在下」）",
      lines[1].startswith(f"{YS[-1]:+5.2f}"), f"got {lines[1]!r}")
check("y 最小那行带着那个撞桌格", lines[-1] == "-0.10   .   X  ",
      f"got {lines[-1]!r}")
check("撞桌格落在**第 2 列**（前缀 7 + 4×1 列 + 居中偏移 1）",
      lines[-1].index("X") == 7 + 4 * 1 + 1, f"got {lines[-1].index('X')}")
check("缺失的格子画成 '?' 而不是当成安全（宁可出声）",
      "?" in render_map({}, XS, YS))


# ----------------------------------------------------------------------
# F. row_runs：每一行的**连续可用段**
# ----------------------------------------------------------------------
# ⚠️ **这一节是第三版**，前两版测的是「从近边 / 从远边起算、遇到第一个障碍
# 就停」的前缀表 —— 两版的自检**都是绿的**。可第二版在真机几何全扫里
# **21 行全空，一个值都没产出过**（远边那一段全是「够不到」，第一个格子就停），
# 而我还照着它写了「从远边起算就知道 x 至少要多大」并配了自检。
# 绿的自检没能证明那个函数有用，只证明了它在**被挑中的那几个样本上**照规则跑
# ——那些样本恰好都是「近边脏、远边干净」，正好躲开了它的死因。
# 所以这一节**故意**放几行「两端都脏、中间可用」的，最后一条还直接抄了一行
# **实测数据**（第八十六轮真机几何全扫的 y=+0.25）。
print("\n--- F. row_runs：连续可用段 ---")
X4, Y1 = [0.0, 1.0, 2.0, 3.0], [0.0]


def runs_of(syms):
    return row_runs({(x, 0.0): s for x, s in zip(X4, syms)}, X4, Y1)[0.0]


check("全安全 ⇒ 一整段",
      runs_of([SAFE] * 4) == [(0.0, 3.0, False)], f"got {runs_of([SAFE] * 4)!r}")
check("★ 两端都不可用、中间可用 ⇒ **中间那一段照旧报出来**"
      "（这正是前两版都说不出来的那种行）",
      runs_of([NOIK, SAFE, SAFE, NOIK]) == [(1.0, 2.0, False)],
      f"got {runs_of([NOIK, SAFE, SAFE, NOIK])!r}")
check("★ 一行里两个孤岛 ⇒ 两段都要在，且**自左向右**排",
      runs_of([BLOCKED, SAFE, BLOCKED, SAFE]) == [(1.0, 1.0, False),
                                                  (3.0, 3.0, False)],
      f"got {runs_of([BLOCKED, SAFE, BLOCKED, SAFE])!r}")
check("近限格**不切断**段，只给这一段打上「含近限」",
      runs_of([SAFE, NEAR, SAFE, NOIK]) == [(0.0, 2.0, True)],
      f"got {runs_of([SAFE, NEAR, SAFE, NOIK])!r}")
check("整段都是近限格 ⇒ 段还在，含近限=True",
      runs_of([BLOCKED, NEAR, NEAR, BLOCKED]) == [(1.0, 2.0, True)],
      f"got {runs_of([BLOCKED, NEAR, NEAR, BLOCKED])!r}")
check("全不可用 ⇒ **没有段**（不是 [(0,3)]、也不是 None）",
      runs_of([NOIK, BLOCKED, NOIK, BLOCKED]) == [],
      f"got {runs_of([NOIK, BLOCKED, NOIK, BLOCKED])!r}")
check("一个格子都没有 ⇒ 空表（缺失的格子按不可用处理，与 render_map 同一取向）",
      row_runs({}, X4, Y1)[0.0] == [], f"got {row_runs({}, X4, Y1)[0.0]!r}")
check("★ 每一行都要有键（哪怕值是空表）—— 少一行会被读成「这一行没扫」",
      sorted(row_runs({}, X4, [0.0, 1.0]).keys()) == [0.0, 1.0])
check("多行互不串（相邻两行符号不同时，各自只报自己那行）",
      row_runs({(0.0, 0.0): SAFE, (1.0, 0.0): SAFE,
                (0.0, 1.0): BLOCKED, (1.0, 1.0): SAFE}, [0.0, 1.0], [0.0, 1.0])
      == {0.0: [(0.0, 1.0, False)], 1.0: [(1.0, 1.0, False)]})

# ★ 实测那一行：第八十六轮真机几何全扫（table_z=0.0）里 y=+0.25 的原文 ——
#   `~  .  X X X  . . . . . . . . .  x x x`
# 近处一个近限格、一段撞桌带、中间一大段可用、远处够不到。
# 期望：**两段**，且第二段是最宽的那一段。
_RX = [round(0.15 + 0.05 * i, 2) for i in range(17)]
_RROW = ([NEAR, SAFE] + [BLOCKED] * 3 + [SAFE] * 9 + [NOIK] * 3)
check("★ 实测那一行（y=+0.25）⇒ 两段：[0.15,0.20] 与 [0.40,0.80]",
      row_runs({(x, 0.0): s for x, s in zip(_RX, _RROW)}, _RX, Y1)[0.0]
      == [(0.15, 0.20, True), (0.40, 0.80, False)],
      f"got {row_runs({(x, 0.0): s for x, s in zip(_RX, _RROW)}, _RX, Y1)[0.0]!r}")
check("★ 同一行：最宽的那一段是 [0.40,0.80]（决定 pick_zone 该往哪儿框）",
      max((b - a, (a, b)) for a, b, _ in
          row_runs({(x, 0.0): s for x, s in zip(_RX, _RROW)}, _RX, Y1)[0.0])[1]
      == (0.40, 0.80))
# ⚠️ 这里原来还有一条「两张表互补」的自检 —— 它用的样本是
# `[BLOCKED, BLOCKED, SAFE, SAFE]`（近边脏、远边干净），于是**绿着**，
# 而现实中远边从来不干净。样本挑得对，断言就只能证明那句断言。
# 删掉它，不换成「更好的样本」—— 换成样本还是同一个病：拿自造的干净行
# 去证明一个函数的用途。**用途由实测那一行来证**（上面两条 ★）。


# ----------------------------------------------------------------------
# F2b. max_rect：最大的整块**矩形**可用区（划区那一步真正要的东西）
# ----------------------------------------------------------------------
print("\n--- F2b. max_rect：最大整块矩形 ---")
XS4 = [0.0, 1.0, 2.0, 3.0]
YS3 = [0.0, 1.0, 2.0]


def rect(rows, safe_only=False, xs=None, ys=None):
    """rows 是**逐行**的符号表，第 0 行对应 ys[0]。"""
    xs = XS4 if xs is None else xs
    ys = YS3[:len(rows)] if ys is None else ys
    cells = {(x, y): s for y, r in zip(ys, rows) for x, s in zip(xs, r)}
    return max_rect(cells, xs, ys, safe_only=safe_only)


check("全可用 ⇒ 整张图（4×3）",
      rect([[SAFE] * 4] * 3) == (0.0, 3.0, 0.0, 2.0, 12),
      f"got {rect([[SAFE] * 4] * 3)!r}")
check("★ **逐行取交集会算错**的那种：三行各自都可用，但可用区间错开",
      # 行 y=0: [safe,safe,blk,blk]  行 y=1: [blk,blk,safe,safe]
      # 逐行交集 = 空；正确答案是**任一行里最大的那一块**（2 格宽 × 1 行 = 2）
      rect([[SAFE, SAFE, BLOCKED, BLOCKED],
            [BLOCKED, BLOCKED, SAFE, SAFE],
            [BLOCKED, BLOCKED, BLOCKED, BLOCKED]])[4] == 2,
      f"got {rect([[SAFE, SAFE, BLOCKED, BLOCKED], [BLOCKED, BLOCKED, SAFE, SAFE], [BLOCKED] * 4])!r}")
check("★ 高比宽值钱：2×2（4 格）胜过 1×3（3 格）",
      rect([[SAFE, SAFE, BLOCKED],
            [SAFE, SAFE, BLOCKED],
            [BLOCKED, BLOCKED, SAFE]])[4] == 4)
check("safe_only=True 时「近限」格算障碍（可用但不安全）",
      rect([[NEAR, NEAR], [SAFE, SAFE]], safe_only=True)[4] == 2
      and rect([[NEAR, NEAR], [SAFE, SAFE]], safe_only=False)[4] == 4)
check("全不可用 ⇒ None（不是 (0,0,0,0,0)）",
      rect([[BLOCKED] * 4] * 3) is None)
check("缺失的格子按不可用处理（与 render_map / row_runs 同一取向）",
      max_rect({}, XS4, YS3) is None)
check("矩形的上下边取自 ys（升序）—— 不是行号",
      rect([[BLOCKED] * 4, [SAFE] * 4, [SAFE] * 4])[:4]
      == (0.0, 3.0, 1.0, 2.0),
      f"got {rect([[BLOCKED] * 4, [SAFE] * 4, [SAFE] * 4])!r}")
check("单格结论也对（退化情形不该崩）",
      rect([[BLOCKED, SAFE], [BLOCKED, BLOCKED]]) == (1.0, 1.0, 0.0, 0.0, 1),
      f"got {rect([[BLOCKED, SAFE], [BLOCKED, BLOCKED]])!r}")


# ----------------------------------------------------------------------
# F2c. pick_zone_z：x/y 之外的最后一半
# ----------------------------------------------------------------------
# 这一节钉的是一件**会静默出错**的事：`pick_zone` 是 3D 盒子，x/y 由包线给，
# z 得按方块高填 —— 而现行 yaml 的 z 是**仿真**的（仿真台面在 base 系 −0.39）。
# 填错了不报错，只是规划器对每条检测走 `continue`（planner_node.py:1284），
# 日志只剩「候选 0 个」，看着像检测坏了。
#
# ⚠️ 这一节里最要紧的是**头两条**：中心是「方块**顶面**」不是「方块中心」。
#    我第一版按中心算（+0.01425），低了**半个方块**，而且**文档里也照着写了**。
#    它靠什么抓出来 —— 拿**仿真**那三个已知数对一遍：台面 −0.39、方块 0.04、
#    yaml 的 `place_z = -0.35` 恰好是两者之和。**真实数据自洽，才是口径对了的证据。**
print("\n--- F2c. pick_zone_z：z 上下界（中心是**顶面**）与现行值当面比一次 ---")
ZMIN, ZMAX, ZTOP = pick_zone_z(0.0, REAL_BLOCK_HEIGHT)
check("真机台面 0.0 + 方块高 28.5 mm ⇒ **顶面** +0.0285（不是中心 +0.01425）",
      near(ZTOP, 0.0 + REAL_BLOCK_HEIGHT, 1e-12), f"got {ZTOP!r}")
# 这一条是**真数据自洽**检查：按「中心」算的那一版过不了它（−0.37 vs 实际 −0.35）。
# `place_z` 是别人早就标定好写在那儿的数，拿它当标尺最难自欺。
_SIM_TABLE, _SIM_BLOCK, _SIM_PLACE_Z = -0.39, 0.04, -0.35
check("★ 用**仿真**那三个已知数对一遍：台面 −0.39 + 方块 0.04 = place_z −0.35"
      "（yaml 里就是这么标的 ⇒ 顶面口径成立）",
      near(pick_zone_z(_SIM_TABLE, _SIM_BLOCK)[2], _SIM_PLACE_Z, 1e-12),
      f"got {pick_zone_z(_SIM_TABLE, _SIM_BLOCK)[2]!r}，place_z 是 {_SIM_PLACE_Z}")
check("★ 而仿真那一带**落在** yaml 现行 z∈[-0.45,-0.30] 里 —— 仿真那份是对的，"
      "真机那份只是没跟着改（所以现行值不能当旁证用）",
      pick_zone_z(_SIM_TABLE, _SIM_BLOCK)[0] >= _SIM_TABLE - 0.45
      and pick_zone_z(_SIM_TABLE, _SIM_BLOCK)[1] <= _SIM_TABLE + 0.09,
      f"got {pick_zone_z(_SIM_TABLE, _SIM_BLOCK)!r}")
check("★ 余量在**两侧**（上下各一份），且上界高过**方块顶面**",
      near(ZMIN, ZTOP - PICK_Z_PAD) and near(ZMAX, ZTOP + PICK_Z_PAD),
      f"got {(ZMIN, ZMAX)!r}")
check("顶面**严格**在区间里（闭区间边界不算「在里面」的话就白算了）",
      ZMIN < ZTOP < ZMAX, f"got {(ZMIN, ZTOP, ZMAX)!r}")
check("下界**不低于台面**（方块躺在台面上，就不可能出现在台面之下；"
      "台面之下的归 table_min_z 管，两个界不互相串）",
      ZMIN >= 0.0, f"got {ZMIN!r}")
check("方块薄于 pad 时，下界**夹在台面**上而不是掉到台面以下",
      pick_zone_z(0.0, 0.005, 0.02)[0] == 0.0,
      f"got {pick_zone_z(0.0, 0.005, 0.02)!r}")
check("★ 真机那一带与**仿真**台面（−0.39）算出来的那一带**不相交**"
      "—— 这就是照抄 x/y 却漏了 z 会踩的那个坑",
      pick_zone_z(-0.39, REAL_BLOCK_HEIGHT)[1] < ZMIN,
      f"got {pick_zone_z(-0.39, REAL_BLOCK_HEIGHT)!r} vs {(ZMIN, ZMAX)!r}")
check("pad 放大时区间跟着变宽（上下各一份）—— 用个厚方块，免得撞上下面那条夹子",
      tup_near(pick_zone_z(0.0, 0.10, 0.05), (0.05, 0.15, 0.10)),
      f"got {pick_zone_z(0.0, 0.10, 0.05)!r}")
# ⚠️ 这条是**写上面那条时自己踩到的**：pad=0.05、方块 28.5 mm 时，
# `ztop - pad` = −0.0215 已经在台面以下了，夹子把它抬到 0.0。
# 也就是说「pad 放大 ⇒ 区间两侧对称变宽」**在薄方块上不成立** —— 那是对的
# （区不能伸到台面以下），但如果测试照着「对称」写，它会红，而红的不是代码。
check("pad 大到会伸到台面以下 ⇒ **下界夹在台面**，上界照留（不是对称变宽）",
      tup_near(pick_zone_z(0.0, REAL_BLOCK_HEIGHT, 0.05), (0.0, ZTOP + 0.05, ZTOP)),
      f"got {pick_zone_z(0.0, REAL_BLOCK_HEIGHT, 0.05)!r}")
check("pad=0 时区间塌成一个点，正好落在顶面上",
      tup_near(pick_zone_z(0.0, REAL_BLOCK_HEIGHT, 0.0), (ZTOP, ZTOP, ZTOP)))

# 判词：只在「方块顶面不在现行 z 区间里」时才说话
SIM_ZONE = [0.30, 0.90, -0.18, 0.16, -0.45, -0.30]      # ✓ 现行 yaml 的值
# ⚠️ 只算一次、存进 NOTE：这三条都拿它做 `in` / `all(... in ...)`。
# 写成「每次都现调」的话，一旦判词方向反了（该说话时回 None），
# `"x" in None` 会抛 TypeError 把整个脚本打断 —— 与上面 `calls()` 那条同一个理由。
NOTE = pick_zone_z_note(SIM_ZONE, ZTOP, ZMIN, ZMAX)
check("★ 现行 yaml 那份（z∈[-0.45,-0.30]，仿真的）⇒ **报警**", NOTE is not None)
check("  警报里带着**改成的数**（不是只说「错了」）",
      bool(NOTE) and f"[{ZMIN:+.4f},{ZMAX:+.4f}]" in NOTE)
check("  警报点名 `in_zone` 与那两行（现场要知道去哪儿看）",
      bool(NOTE) and all(s in NOTE for s in ("in_zone", ":1284")))
check("z 已经对了 ⇒ 不说话（不制造常亮的假警告）",
      pick_zone_z_note([0.40, 0.75, -0.45, 0.45, 0.005, 0.05], ZTOP, ZMIN, ZMAX) is None)
check("顶面正好落在边界上 ⇒ 也算收得到（与 `in_zone` 的闭区间同口径）",
      pick_zone_z_note([0.40, 0.75, -0.45, 0.45, ZTOP, 0.05], ZTOP, ZMIN, ZMAX) is None)
check("节点没读到 pick_zone ⇒ None（不拿 None 参与比较）",
      pick_zone_z_note(None, ZTOP, ZMIN, ZMAX) is None)
# ⚠️ 这一条**必须**用 calls() 包着：直接写 `... is None`，一旦守卫被去掉就是
# `IndexError` 从 check() 的实参里抛出来 —— 变异照样能杀掉（套件不会报「通过」），
# 但整个脚本会**当场中断**，后面所有检查一条都不跑，红的是「崩了」不是「哪条错」。
# 一条把后续检查全吞掉的断言，比它想防的那个 bug 更难查。
check("pick_zone 长度不对 ⇒ None（不是 IndexError）",
      calls(lambda: pick_zone_z_note([0.30, 0.90, -0.18, 0.16],
                                     ZTOP, ZMIN, ZMAX)) == ("ok", None),
      f"got {calls(lambda: pick_zone_z_note([0.30, 0.90, -0.18, 0.16], ZTOP, ZMIN, ZMAX))!r}")



# ----------------------------------------------------------------------
# F3. JSON 的键/轴契约：**老实读法也必须对得上**
# ----------------------------------------------------------------------
# `axis(0.15, 0.95, 0.05)` 里的 x **不是**整齐十进制浮点（0.30000000000000004
# 这种），而键按 %.4f 写成 "0.3000"。第八十四轮实测：一个老实的消费者
# （拿 xs 的值去配解析出来的键）会在 17 列里**静默漏掉 6 列**，把那里当成
# 「没有数据」—— 不报错，只是少一块。所以出口处圆一次。
print("\n--- F3. cell_key / json_axis：键与轴必须能往返 ---")
RAW_XS = axis(0.15, 0.95, 0.05)
check("这个网格里**确实有**不能往返的 x（否则整节是空转）",
      any(v != round(v, 4) for v in RAW_XS),
      f"got {[v for v in RAW_XS if v != round(v, 4)]!r}")
ROUNDED = json_axis(RAW_XS)
check("json_axis 之后每个值都能 %.4f 往返（float(f'{v:.4f}') == v）",
      all(float(f"{v:.4f}") == v for v in ROUNDED), f"got {ROUNDED!r}")
check("圆过之后，键字符串与轴值**一一对上**（这正是老实读法依赖的性质）",
      [cell_key(v, 0.0).split(",")[0] for v in ROUNDED]
      == [f"{v:.4f}" for v in ROUNDED])
check("圆只动小数点后 4 位以外的噪声（0.05 的偏差 < 1e-15）",
      all(abs(a - b) < 1e-15 for a, b in zip(RAW_XS, ROUNDED)), f"got {ROUNDED!r}")
check("cell_key 两边都按 %.4f（不是只圆一个）",
      cell_key(0.30000000000000004, 0.1000000000000000)
      == cell_key(0.3, 0.1) == "0.3000,0.1000")


# ----------------------------------------------------------------------
# G. launch_const：真机常量**从 pipeline.launch.py 读**，不抄副本
# ----------------------------------------------------------------------
print("\n--- G. launch_const / num ---")
ttf_txt = launch_const("REAL_TOOL0_TO_FINGERTIP")
prl_txt = launch_const("REAL_PLACE_RELEASE_LIFT")
check("读得到 REAL_TOOL0_TO_FINGERTIP", ttf_txt is not None, f"got {ttf_txt!r}")
check("读得到 REAL_PLACE_RELEASE_LIFT", prl_txt is not None, f"got {prl_txt!r}")
check("不存在的常量返回 None（不抛异常）",
      launch_const("NO_SUCH_CONST_XYZ") is None)
check("常量必须**同时**有 SIM_ 与 REAL_ 两份（只有一个号说明后端没分开）",
      launch_const("SIM_TOOL0_TO_FINGERTIP") is not None
      and launch_const("SIM_PLACE_RELEASE_LIFT") is not None)
check("num() 把坏值当读不到（返回 None）而不是 0.0",
      num("abc", "x") is None and num(None, "x") is None)
check("num() 正常转换", num("0.1757", "x") == 0.1757)

if ttf_txt is not None:
    # 交叉检查：AG-95 闭合时 tool0→指尖 204.2 mm（含 4 mm 转接件）= ttf + 方块高。
    # 这两个数在两个不同的文件里，改一个不改另一个就会让抓取高度整体偏移。
    total = float(ttf_txt) + REAL_BLOCK_HEIGHT
    check("REAL_TOOL0_TO_FINGERTIP + REAL_BLOCK_HEIGHT = 0.2042（AG-95 闭合尺寸）",
          near(total, 0.2042, tol=1e-9), f"got {total!r}")


# ----------------------------------------------------------------------
# H. 「照着包线划区」要有地方写：几何键必须留在 planner_params_file 那一层
# ----------------------------------------------------------------------
# 包线给出的是**边界**，现场要把它落成一个区。落在哪儿？`pipeline.launch.py` 的
# `planner_params = [<yaml 路径>, {覆盖项}]` —— 第二份 dict 里的键**赢过**现场
# 传的 yaml，`tool0_to_fingertip` 这类后端属性就该在那儿（它随 camera_source 自动
# 切）。而**几何区**（pick_zone / place_slots / place_z / table_plane_z / table_min_z）
# 是现场实测值，必须留在地下那层，现场才能用
#     planner_params_file:=<现场 yaml>
# 覆盖它们而**不必去改仓库里那份共用的 yaml**（改了仿真一起变）。
# ⇒ 这几条是**契约**：谁哪天把它们加进第二份 dict，现场唯一的覆盖口子就没了，
#    而且是静默失效。这条测试就是钉住它。
print("\n--- H. 现场覆盖口子：几何键不能进 planner_params 的第二份 dict ---")
import re                                                       # noqa: E402

with open(os.path.join(PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup", "launch",
                       "pipeline.launch.py"), encoding="utf-8") as f:
    _src = f.read()
_m = re.search(r"^(\s*)planner_params = \[(.*?)^\1\]", _src, re.S | re.M)
check("读到了 planner_params 那个列表", _m is not None)
_block = _m.group(2) if _m else ""
check("planner_params_file 仍然是声明的 launch 参数（现场覆盖的入口）",
      "'planner_params_file'" in _src and "DeclareLaunchArgument" in _src)
check("sanity：解析到的确实是那个块（它里面**应该**有 tool0_to_fingertip）",
      "'tool0_to_fingertip'" in _block)
for key in ("pick_zone", "place_slots", "place_z", "table_plane_z", "table_min_z"):
    check(f"{key} **不在**第二份 dict 里（现场 yaml 才盖得住）",
          f"'{key}'" not in _block,
          "它被 launch 覆盖了 ⇒ planner_params_file 里改它静默无效")


# ----------------------------------------------------------------------
# I. node_guard_note：**「护栏放行」与「在台面以下」是两件事**
# ----------------------------------------------------------------------
# ⚠️ 这一段是**实测踩出来的**，不是想出来的。第八十五轮加的那句判词，第一版写成
# 「节点护栏会放行它 ⇒ 所以它已在台面以下」。前半句有 `worst_z >= old_floor`
# 支持，**后半句当时没有任何东西检查过** —— 在板上扫一片本来就全安全的区域
# （x=0.5、最低连杆 +0.2868，台面之上）时，它照样印出
# 「它已在台面以下 -0.2868 m」：负数，且与事实相反。
# 一条字符串报告的是**它自己那句话**，不是它以为在描述的物理事实。
print("\n--- I. node_guard_note：放行 ≠ 在台面以下 ---")

K = dict(node_table_z=-0.39, guard_margin=0.02)      # 节点那份 ⇒ 下限 -0.3700

# ① 真正的危险格：连杆在台面**之下**，而节点护栏放行（-0.0944 >= -0.37）
n = node_guard_note(-0.0944, 0.0, **K)
check("真危险格：说了「会放行它」", n and "会放行" in n, f"got {n!r}")
check("★ 真危险格：**才**说「台面以下」，且是正数 0.0944（不是负的）",
      n and "台面以下 0.0944 m" in n, f"got {n!r}")

# ② 本次踩到的那一格：连杆在台面**之上**，节点护栏也放行
n2 = node_guard_note(0.2868, 0.0, **K)
check("★ 台面之上的格子：**不许**说「台面以下」（这就是当初印错的那句）",
      n2 and "台面以下" not in n2, f"got {n2!r}")
check("★ 台面之上的格子：要明说它**不危险**，别让人读成撞桌",
      n2 and "不危险" in n2 and "台面**之上** 0.2868 m" in n2, f"got {n2!r}")
check("台面之上的格子：仍然如实报告「节点护栏会放行」",
      n2 and "会放行" in n2, f"got {n2!r}")

# ③ 节点护栏自己就拦得下 ⇒ 另一种说法
n3 = node_guard_note(-0.50, 0.0, **K)
check("连杆低于节点下限 ⇒ 说「也会拦下」，不说放行",
      n3 and "也会拦下" in n3 and "放行" not in n3, f"got {n3!r}")

# ④ 两份台面一样 ⇒ 无可说（不印一段自我比较的废话）
check("节点台面 = 本次台面 ⇒ None（两份一样就没什么可比的）",
      node_guard_note(-0.5, 0.0, 0.0, 0.02) is None)
check("节点没读到 table_plane_z ⇒ None（不拿 None 参与比较）",
      node_guard_note(-0.5, 0.0, None, 0.02) is None)
check("边界：正好等于下限 ⇒ **放行**（判据是 >=，与护栏同口径）",
      "会放行" in (node_guard_note(-0.37, 0.0, **K) or ""))

# ⑤ 三条判词互斥：任何输入只落一种说法
says = [node_guard_note(z, 0.0, **K) for z in (-0.9, -0.5, -0.37, -0.0944, 0.0, 0.5)]
check("每一条都非空（节点台面不同 ⇒ 总有话说）", all(says))
# z = -0.9 / -0.5 低于节点下限 ⇒ 「也会拦下」，与台面高低无关，不说「台面以下」
# z = -0.37 / -0.0944 放行且低于台面 ⇒ 说「台面以下」（这两格才是真危险）
# z = 0.0 / +0.5 放行但在台面之上 ⇒ 说「台面之上、不危险」
check("说「台面以下」的正好是那两格真的低于台面的（-0.37 与 -0.0944）",
      [i for i, s in enumerate(says) if "台面以下" in s] == [2, 3],
      f"got {[i for i, s in enumerate(says) if '台面以下' in s]}")
check("说「也会拦下」的正好是那两格低于节点下限的（-0.9 与 -0.5）",
      [i for i, s in enumerate(says) if "也会拦下" in s] == [0, 1],
      f"got {[i for i, s in enumerate(says) if '也会拦下' in s]}")

print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
