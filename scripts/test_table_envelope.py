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
        SOLVABLE_OTHER, axis, classify_one, combine, launch_const, num,
        render_map, row_limits, worst, z_levels)
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
# F. row_limits：从近边起算的连续边界
# ----------------------------------------------------------------------
print("\n--- F. row_limits：连续可用边界 ---")
X4, Y1 = [0.0, 1.0, 2.0, 3.0], [0.0]


def limits(syms):
    return row_limits({(x, 0.0): s for x, s in zip(X4, syms)}, X4, Y1)[0.0]


check("撞桌挡住后面的孤岛（不跳空洞继续说安全）",
      limits([SAFE, SAFE, BLOCKED, SAFE]) == (1.0, 1.0),
      f"got {limits([SAFE, SAFE, BLOCKED, SAFE])!r}")
check("近限格不计入「安全」但计入「可用」",
      limits([NEAR, SAFE]) == (1.0, 1.0), f"got {limits([NEAR, SAFE])!r}")
check("只有近限格 ⇒ 安全列是 None（不是 0）",
      limits([NEAR, BLOCKED]) == (None, 0.0), f"got {limits([NEAR, BLOCKED])!r}")
check("全不可用 ⇒ (None, None)", limits([NOIK, BLOCKED]) == (None, None),
      f"got {limits([NOIK, BLOCKED])!r}")
check("全安全 ⇒ 一直到最远那一列", limits([SAFE] * 4) == (3.0, 3.0),
      f"got {limits([SAFE] * 4)!r}")


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


print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
