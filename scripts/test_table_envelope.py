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
        launch_const, node_guard_note, num, render_map, row_limits,
        row_limits_outer, worst, z_levels)
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
# F2. row_limits_outer：从**远边**起算，给的是 x 的下界
# ----------------------------------------------------------------------
# 为什么需要它（第八十四轮实测）：全扫之后撞桌格变成贴着基座的一条，
# 21 行里有 9 行**第一个格子就不可用**，`row_limits` 全给 «—»；
# 而那一片恰恰是「x 不能小于多少」，只有从远边起算才说得出来。
print("\n--- F2. row_limits_outer：x 的下界 ---")


def limits_out(syms):
    return row_limits_outer({(x, 0.0): s for x, s in zip(X4, syms)}, X4, Y1)[0.0]


check("撞桌格在近边 ⇒ 下界就是它后面那一格（这正是 row_limits 说不出话的那种）",
      limits_out([BLOCKED, BLOCKED, SAFE, SAFE]) == (2.0, 2.0),
      f"got {limits_out([BLOCKED, BLOCKED, SAFE, SAFE])!r}")
check("远边就是撞桌格 ⇒ 挡住里面的孤岛，下界推到远边外（None）",
      limits_out([SAFE, BLOCKED, SAFE, BLOCKED]) == (None, None),
      f"got {limits_out([SAFE, BLOCKED, SAFE, BLOCKED])!r}")
check("全安全 ⇒ 下界一直落到近边（x 可以小到 0）",
      limits_out([SAFE] * 4) == (0.0, 0.0), f"got {limits_out([SAFE] * 4)!r}")
check("贴远边的一串全是近限格 ⇒ 可用有值、安全是 None（不是 0）——"
      "近限只让它「能用」，不许它冒充「安全」",
      limits_out([SAFE, BLOCKED, NEAR, NEAR]) == (None, 2.0),
      f"got {limits_out([SAFE, BLOCKED, NEAR, NEAR])!r}")
check("远边第一个就是近限格时**不中断**扫描（否则可用会莫名其妙变成 None）",
      limits_out([SAFE, SAFE, SAFE, NEAR]) == (0.0, 0.0),
      f"got {limits_out([SAFE, SAFE, SAFE, NEAR])!r}")
check("全不可用 ⇒ (None, None)", limits_out([NOIK, BLOCKED]) == (None, None))
check("两张表在「全安全」这一行上必须同时到两端（互为镜像的自洽检查）",
      limits_out([SAFE] * 4)[0] == 0.0 and limits([SAFE] * 4)[0] == 3.0)
check("同一行上两张表是**互补**的、不是重复：近边表说不出话时远边表给出答案",
      limits([BLOCKED, BLOCKED, SAFE, SAFE]) == (None, None)
      and limits_out([BLOCKED, BLOCKED, SAFE, SAFE]) == (2.0, 2.0),
      f"近边 {limits([BLOCKED, BLOCKED, SAFE, SAFE])!r} "
      f"远边 {limits_out([BLOCKED, BLOCKED, SAFE, SAFE])!r}")


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
