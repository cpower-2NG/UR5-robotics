#!/usr/bin/env python3
"""桌面安全包线：把「真机几何」下**可以选作抓取区/放置区**的那片桌面画出来。
==========================================================================
回答一个问题：在**工作台台面这个高度**上，臂伸到哪些 (x, y) 会把大臂/腕插到
台面以下、哪些干脆够不到 —— 于是 pick_zone / place_zone 可以**照着划**，而不是
「先框一个再跑一遍看看」。

与 `elbow_guard_probe.py` 的分工（**用同一份 Probe，所以判据必然一致**）
--------------------------------------------------------------------
  · `elbow_guard_probe.py`：给**区**，逐点判护栏。回答「我打算去的这些点安全吗」，
    退出码是**闸门**（0 = 可以动，1 = 不要动）。
  · 本脚本：给**一片范围**，把整片扫成图。回答「哪片区域可以选作区」。
    退出码**不是闸门** —— 图上出现不安全格是**结论**，不是失败（扫的范围本来就
    比任何合理的区都大，跟 `--stress` 一个道理）。

它守的是 D49 那个只在真机上成立的风险：仿真里 UR5 底座坐在 0.80 m 立柱上、
桌面 0.41 m，所以**肘朝下的 IK 分支**落在桌面上方，一辈子碰不到；真机把臂直接
装在桌面上（台面在 base_frame 系 ≈ 0），同一个分支就**能落到台面以下**。
而 `avoid_collisions=True` 防的是一个**空场景**（桌子不在 MoveIt 规划场景里），
拦不住它。护栏（`elbow_guard_enabled`）拦得住，但它是**运行期**拦的 —— 那时臂
已经在动了。所以现场上电前先在这里把整片桌面扫一遍。

⚠️ **别把 D49 那句手算当成量出来的数**（第八十四轮实测更正）
计划里写的是「桌面正前方 0.45 m、**桌面高度** ⇒ 肘落到台面下约 21 cm」。
两个问题：① 那是**手算的一个候选分支**，不是规划器实际解出来的那个；
② 它取的目标高度**比规划器实际会走的还低** —— 规划器最低只到
「台面 + 方块高 + 指尖偏置」= 台面之上 204.2 mm（`tool0` 在那儿，指尖才落到台面），
而「tool0 在桌面高度」等于把指尖**压进台面 175.7 mm**，那是这个夹爪装上去之后
**根本到不了**的位姿。实测扫出来的不安全格**既不在 x=0.45、量级也不是 21 cm**。
⇒ **手算回答了「这个分支存在吗」，没有回答「规划器会不会走到它」。**
后者只能像这样把整片桌面解一遍。

⚠️ 真机几何参数**一律走命令行，不从节点参数读**
------------------------------------------------
节点（`/grasp_planner`）手上那份是**仿真** yaml：`table_plane_z: -0.39`、
`tool0_to_fingertip: 0.13`。本脚本要算的是**真机**几何，所以台面高度、方块高、
指尖偏置、放置余量四项是命令行给的，且**每一项都打印来源**。若节点参数与本次
用的值不同，会打一条 ⚠️ —— 这正是第八十三轮修掉的那类病（同一张图两套几何，
分叉了还不出声）。
`tool0_to_fingertip` / `place_release_lift` 的真机默认值**从
`pipeline.launch.py` 里读**（`REAL_TOOL0_TO_FINGERTIP` / `REAL_PLACE_RELEASE_LIFT`），
不在这里抄一份 —— 抄下来的那份迟早和真身分叉。

四个 tool0 高度（这就是臂实际会走到的那些 z）
--------------------------------------------
    抓取下降 = 台面 + 方块高 + 指尖偏置      ← **最低的一个**，=「指尖碰台面」
    抓取接近 = 上式 + approach_clearance
    放置下降 = 抓取下降 + place_release_lift
    放置接近 = 上式 + approach_clearance
判读取**四个里最坏的那个**（一个格子只有在四个高度上都安全才算安全格）——
因为规划器会把它们**全走一遍**。

这张图**不**建模规划器的哪一步（读图前必看）
-------------------------------------------
包线量的是「**规划器配的那个种子**解出来的那个分支，连杆在台面之上吗」。
它**没有**复刻 `planner_node._call_ik` 的后半段 —— 那一段是：解出来之后先过
**肘高护栏**，不过才用「另一分支」的种子重解一次（重解的解**照样要过护栏**，
D54）。所以：

  · 图上 `X`（低于下限）**不等于**「规划器会把臂开到台面下」。它的意思是
    **「这个位置不能依赖」**：护栏按对的台面高度判时，规划器会**拒掉**它
    （或换分支解出另一个解），结果是**放弃这个目标**。
  · 真正的风险恰恰是**护栏按错的台面高度判**：`table_plane_z` 现在还是仿真的
    −0.39（`grasp_planner.yaml`，而且**没有 launch 覆盖口子**），于是护栏下限是
    −0.37 —— 一个连杆高度 −0.055 的解**会一路放行**，而真机台面在 ≈ 0，
    那是**台面以下 55 mm**。这张图就是拿真机台面重算的那个判据。
  · 一个格子只有配的种子解不出、换种子才解得出时（`--retry-seed`），标 `s`：
    那是**种子**问题不是够不到 —— `_call_ik` 在这种格子上**直接返回 None**
    （跳过目标），所以它和 `x` 一样是「区别框到这儿」，只是原因不同、处置不同。

用法（板端，先起无臂预检或任何一套带 /compute_ik 的 MoveIt）
    # 最省事：跟着无臂预检一起跑（那时几何与上电用的严格同源）
    python3 scripts/moveit_only_check.py --envelope-z 0.0
    # 或者对着已经在跑的图单独跑
    python3 scripts/table_safety_envelope.py --table-z 0.0
    python3 scripts/table_safety_envelope.py --table-z 0.0 --step 0.1 --json /tmp/env.json
    # 分清「够不到」与「种子太偏」（推荐：后者的处置完全相反）
    python3 scripts/table_safety_envelope.py --table-z 0.0 --retry-seed home

退出码：0 = 图画出来了（有没有不安全格都算成功）；2 = 环境不满足
（/compute_ik 或 /compute_fk 不在，或**一个位姿都没解出来** —— 那种情况下
「没有不安全格」是假绿，判读无从谈起）。只调 /compute_ik 与 /compute_fk。
"""

import argparse
import json
import math
import os
import re
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rclpy                                                    # noqa: E402

# 复用探针：**同一个 Probe**，所以 IK 请求的构造、夹爪关节的注入规则、护栏连杆
# 的取值与肘高预检逐字一致。包线若与护栏用了两套判据，那它就没有预测力了。
from elbow_guard_probe import Probe, ERROR_NAMES                # noqa: E402

# ---------------------------------------------------------------------------
# 真机几何的默认值
# ---------------------------------------------------------------------------
# 台面在 base_frame 的 z。**实测**（2026-10-02）：`TCP_1` 指尖碰台面读到
# base 系 +3.88 mm，其中 4 mm 是 TCP 少填的那层转接件 ⇒ 台面 ≈ −0.1 mm。
# ⚠️ 读数时示教器位置下拉必须选「基座」：默认停在的「视角」比基座高 399.35 mm，
#    会读出 −395.47 这种假数（RESTRUCTURE_PLAN 第六十六轮）。
# 即便如此这个数**必须由调用方再确认一次**（`--table-z` 是必填项）——
# 包线的整个结论都架在它上面，替它兜底一个默认值就等于把一条假设变成静默前提。
REAL_BLOCK_HEIGHT = 0.0285          # 现场视觉标定的方块高（pipeline.launch.py 头注释）

PIPELINE_LAUNCH = os.path.join(
    PROJECT_ROOT, "ros2_ws", "src", "yolo_bringup", "launch", "pipeline.launch.py")

# 四个高度的名字。顺序固定，`--levels` 与 `--map` 都按这些 key 认。
LEVEL_KEYS = ("grasp_descend", "grasp_approach",
              "place_descend", "place_approach")
LEVEL_LABEL = {
    "grasp_descend": "抓取下降", "grasp_approach": "抓取接近",
    "place_descend": "放置下降", "place_approach": "放置接近",
}

# 图例。`worst()` 用这个序取最坏 —— **BLOCKED 赢过 NOIK**：一个格子在四个高度里
# 只要有一个会把连杆压到台面下，那就是个危险的格子，与它另一个高度解不解得出来无关。
SAFE, NEAR, NOIK, BLOCKED = ".", "~", "x", "X"
# `s` = 配的种子解不出、**换一个种子才解得出**。它单列一格，因为「够不到」与
# 「种子太偏」要的处置完全不同：前者该缩区，后者该换种子（或至少知道规划器会在
# 这一整片静默跳过、而 D54 那条「去种子重解」的补救恰恰会在这里翻出别的分支）。
SOLVABLE_OTHER = "s"
_ORDER = {SAFE: 0, NEAR: 1, SOLVABLE_OTHER: 2, NOIK: 3, BLOCKED: 4}


def legend(near_band, with_retry=False):
    s = (f"  {SAFE} 全安全   {NEAR} 安全但离下限 <{near_band:.2f} m（近限）   "
         f"{NOIK} 配的种子解不出（够不到）   {BLOCKED} **有连杆低于下限**（撞桌）")
    if with_retry:
        s += f"   {SOLVABLE_OTHER} 换个种子才解得出（**种子问题**，不是够不到）"
    return s


def launch_const(name):
    """从 `pipeline.launch.py` 读一个 `NAME = '0.1757'` 形式的常量。

    读不到返回 None —— 调用方**必须**出声，不能悄悄用自己的副本顶上。
    """
    try:
        with open(PIPELINE_LAUNCH, encoding="utf-8") as f:
            src = f.read()
    except OSError:
        return None
    m = re.search(rf"""^{re.escape(name)}\s*=\s*['"]([^'"]+)['"]""", src, re.M)
    return m.group(1) if m else None


def num(txt, what):
    """把 launch 常量转成 float；坏值当读不到处理（返回 None）。"""
    try:
        return float(txt)
    except (TypeError, ValueError):
        print(f"⚠️ {what} 读到的是 {txt!r}，不是数 —— 按「读不到」处理。")
        return None


def z_levels(table_z, block_h, ttf, prl, clearance):
    """四个 tool0 高度（base_frame），顺序 = ``LEVEL_KEYS``。

    ⚠️ 别把它们的**大小顺序**想当然：抓取下降最低（= 指尖碰台面那个高度），
    放置下降比它高一个 `place_release_lift`，两个接近点再各高一个 clearance。
    """
    gd = table_z + block_h + ttf
    pd = gd + prl
    return [gd, gd + clearance, pd, pd + clearance]


def axis(lo, hi, step):
    """等距采样，含 lo。步长不是整除时末点略小于 hi（打印出的实际范围为准）。"""
    n = max(1, int(round((hi - lo) / step)))
    return [lo + i * step for i in range(n + 1)]


def classify_one(zs, floor, near_band):
    """单个位姿的判读。`zs` = {连杆: z}；None 或空 = 没解出来。"""
    if not zs:
        return NOIK
    lo = min(zs.values())
    if lo < floor:
        return BLOCKED
    if lo < floor + near_band:
        return NEAR
    return SAFE


def worst(syms):
    """把同一个格子在几个高度上的判读合成一个：取最坏（`_ORDER` 里最大的）。"""
    return max(syms, key=lambda s: _ORDER[s])


def combine(primary_syms, retry_syms):
    """配的种子 + 换种子重试 ⇒ 一个符号。

    ⚠️ 顺序是**安全优先**，不是「谁先谁赢」：
      · 配的种子能解 ⇒ 就用它的判读（换种子只在解不出时才试）；
      · 配的种子全都解不出、换种子解出一个**撞桌**的解 ⇒ 判 **BLOCKED**。
        看着保守，但这是对的那一侧：规划器确实有一条 `_call_ik` 失败后**换种子
        重解**的补救路径（D54 那条「去种子重解」），而那条路径恰好会翻出别的
        分支。一个「换个种子就撞桌」的位置，得当成危险位置记，不能记成「够不到」。
      · 换种子也解不出 ⇒ NOIK（够不到）；
      · 换种子解得出来且不撞桌 ⇒ SOLVABLE_OTHER（**种子问题**）。
    """
    w = worst(primary_syms)
    if w != NOIK or retry_syms is None:
        return w
    rw = worst(retry_syms)
    if rw == BLOCKED:
        return BLOCKED
    if rw == NOIK:
        return NOIK
    return SOLVABLE_OTHER


def render_map(cells, xs, ys):
    """把 `cells[(x, y)] = 符号` 画成文本图。y 从大到小（远→近对着人看）。"""
    head = "  y\\x  " + "".join(f"{x:4.2f}" for x in xs)
    out = [head]
    for y in reversed(ys):
        row = "".join(f"{cells.get((x, y), '?'):^4}" for x in xs)
        out.append(f"{y:+5.2f}  {row}")
    return "\n".join(out)


def row_limits(cells, xs, ys):
    """每一行（固定 y）从**近边**（x 最小处）起算的连续可用边界。

    返回 `{y: (x_安全, x_可用)}`：
      · `x_安全` = 连续全是 `SAFE` 的最后一个 x（不含近限格）
      · `x_可用` = 连续是 `SAFE` 或 `NEAR` 的最后一个 x（能抓，但贴着下限）
    遇到第一个**不可用**（`NOIK` / `BLOCKED`）就停 —— 不「跳过空洞继续说安全」，
    因为远处的孤岛在校验里等于没有（中间那一段臂过不去）。x 全不可用时给 None。
    """
    out = {}
    for y in ys:
        xs_safe = xs_usable = None
        for x in xs:
            s = cells.get((x, y), BLOCKED)
            if s in (SAFE, NEAR):
                xs_usable = x
                if s == SAFE:
                    xs_safe = x
                elif xs_safe is None:
                    pass          # 近限格：可用但不计入「安全」
            else:
                break
        out[y] = (xs_safe, xs_usable)
    return out


def cell_key(x, y):
    """格子的 JSON 键。**写键和读键必须走同一个函数** —— 见 `json_axis()`。"""
    return f"{x:.4f},{y:.4f}"


def json_axis(vals):
    """网格轴进 JSON 之前先圆到 4 位小数。

    ⚠️ 为什么（第八十四轮实测踩到的）：`axis(0.15, 0.95, 0.05)` 算出来的**不是**
    整齐的十进制浮点 —— 它是 `0.30000000000000004`、`0.45000000000000007` 这种。
    键却按 `%.4f` 写成 `"0.3000"`。于是一个**老实巴交的消费者**（读 JSON、
    把 `xs` 里的值和解析出来的键配对）会在 **17 列里静默漏掉 6 列**，
    把它们当成「没有数据」—— 不报错，只是图/表上少一块。
    （我自己写的 `plot_table_envelope.py` 第一版就是这样，而且**自检还是绿的**：
    因为那份测试的样本用的是 `round(x, 2)` 的整齐数，正好躲开了。）
    ⇒ 出口处圆一次，`float(f"{v:.4f}") == v` 就恒成立，老实读法也能对上。
    """
    return [round(v, 4) for v in vals]


def row_limits_outer(cells, xs, ys):
    """每一行从**远边**（x 最大处）起算的连续可用边界 —— 即 **x 的下界**。

    ⚠️ 为什么需要第二张表（第八十四轮实测补的）：`row_limits` 是「从近边起算、
    遇到第一个不可用就停」。第一次粗扫（步长 0.1、只看两个高度）时撞桌格是散的，
    这张表够用。**全扫（步长 0.05、四个高度）之后撞桌格变成了贴着基座的一整条**
    —— 于是 21 行里有 9 行**第一个格子就不可用**，`row_limits` 全给 `—`，
    等于什么都没说。可这一片恰恰是「**x 不能小于多少**」这种约束，
    正是从远边起算才读得出来的那一种。

    实现就一句：把 x 反过来喂给 `row_limits`。近边换成远边之后，
    「连续可用的最后一个 x」正好就是原来的**下界** —— 不需要另写一套扫描，
    也就不会和 `row_limits` 悄悄分叉。返回 `{y: (x_安全 ≥, x_可用 ≥)}`。
    """
    return row_limits(cells, list(reversed(xs)), ys)


def _fmt_x(v):
    return "—" if v is None else f"{v:.2f}"


def main():
    ap = argparse.ArgumentParser(
        description="桌面安全包线（真机几何）：只读，不发轨迹")
    ap.add_argument("--node", default="/grasp_planner", help="规划器节点名")
    ap.add_argument("--table-z", type=float, required=True,
                    help="**必填**：工作台台面在 base_frame 系的高度（m）。"
                         "本机实测 ≈ 0.0（指尖碰台面读 +3.88 mm，减去 TCP 少填的"
                         "4 mm 转接件）。**故意不给默认值** —— 包线的结论整个架在"
                         "这个数上，替它兜底就等于把一条假设变成静默前提。")
    ap.add_argument("--block-height", type=float, default=REAL_BLOCK_HEIGHT,
                    help=f"方块高（m，默认真机 {REAL_BLOCK_HEIGHT} = 28.5 mm，"
                         "现场视觉标定值；换方块要改）")
    ap.add_argument("--tool0-to-fingertip", type=float, default=None,
                    help="tool0 → 抓取点距离（m）。默认从 pipeline.launch.py 的 "
                         "REAL_TOOL0_TO_FINGERTIP 读（0.1757）。"
                         "⚠️ 它**含方块高**，语义见 grasp_planner.yaml。")
    ap.add_argument("--place-release-lift", type=float, default=None,
                    help="放置下降的额外余量（m）。默认从 pipeline.launch.py 的 "
                         "REAL_PLACE_RELEASE_LIFT 读（0.0156）。")
    ap.add_argument("--clearance", type=float, default=None,
                    help="接近点在抓取高度之上多少（m）。默认取节点参数 "
                         "approach_clearance。")
    ap.add_argument("--levels", default="grasp,place",
                    help="扫哪些高度：grasp / place / grasp,place（默认全部四个）")
    ap.add_argument("--map", default="worst",
                    help="画哪一层：worst（默认，四个高度取最坏）或某个 key "
                         f"（{'/'.join(LEVEL_KEYS)}）")
    ap.add_argument("--x-min", type=float, default=0.15)
    ap.add_argument("--x-max", type=float, default=0.95)
    ap.add_argument("--y-min", type=float, default=-0.50)
    ap.add_argument("--y-max", type=float, default=0.50)
    ap.add_argument("--step", type=float, default=0.05, help="网格步长（m）")
    ap.add_argument("--near-band", type=float, default=0.05,
                    help="离下限多近算「近限」（m，默认 0.05）")
    ap.add_argument("--retry-seed", default=None,
                    help="**把「够不到」和「种子太偏」分开**：配的种子（节点参数 "
                         "ik_seed）解不出来的格子，再用这个种子试一次。"
                         "`home` = 用节点参数的 home_joints，或直接给 6 个数"
                         "（逗号分隔）。不给就不重试。"
                         "⚠️ 现象上这两类都是「解不出」，但够不到要**缩区**、"
                         "种子偏要**换种子** —— 而 D54 那条「去种子重解」的补救"
                         "恰恰会在后者里翻出别的分支（可能撞桌）。")
    ap.add_argument("--ik-group", default=None,
                    help="覆盖规划组名。**没有 /grasp_planner 可读参数时必须给** "
                         "（兜底默认是仿真的 ur5_manipulator，真机是 ur_manipulator，"
                         "用错会得到整片 INVALID_GROUP_NAME）。")
    ap.add_argument("--json", default=None, help="把网格与参数转储到这个路径")
    args = ap.parse_args()

    want = [s.strip() for s in args.levels.split(",") if s.strip()]
    bad = [s for s in want if s not in ("grasp", "place")]
    if bad:
        print(f"!! --levels 只认 grasp / place，收到 {bad}")
        return 2
    keys = [k for k in LEVEL_KEYS
            if (k.startswith("grasp") and "grasp" in want)
            or (k.startswith("place") and "place" in want)]
    if args.map != "worst" and args.map not in keys:
        print(f"!! --map 只认 worst 或 {keys}")
        return 2

    retry_seed = None
    if args.retry_seed:
        if args.retry_seed.strip() == "home":
            retry_seed = "home"        # 真正的值要等读到节点参数才知道，见下
        else:
            try:
                retry_seed = [float(v) for v in args.retry_seed.split(",")]
            except ValueError:
                print(f"!! --retry-seed 要么是 home，要么是 6 个逗号分隔的数，"
                      f"收到 {args.retry_seed!r}")
                return 2
            if len(retry_seed) != 6:
                print(f"!! --retry-seed 要 6 个数，收到 {len(retry_seed)} 个")
                return 2

    rclpy.init()
    pr = Probe(args.node, ik_group=args.ik_group)
    try:
        if not pr.ik.wait_for_service(timeout_sec=10.0):
            print("!! /compute_ik 不可用 —— move_group 起了吗？")
            return 2
        if not pr.fk.wait_for_service(timeout_sec=10.0):
            print("!! /compute_fk 不可用")
            return 2

        p = pr.read_params()

        # ---- 真机几何：命令行优先，并把**每个值的来源**打出来 ----
        src = {}
        ttf = args.tool0_to_fingertip
        if ttf is None:
            ttf = num(launch_const("REAL_TOOL0_TO_FINGERTIP"),
                      "pipeline.launch.py 的 REAL_TOOL0_TO_FINGERTIP")
            src["tool0_to_fingertip"] = (
                "pipeline.launch.py REAL_TOOL0_TO_FINGERTIP" if ttf is not None
                else "**读不到** —— 见下面的 ⚠️")
        else:
            src["tool0_to_fingertip"] = "命令行"
        prl = args.place_release_lift
        if prl is None:
            prl = num(launch_const("REAL_PLACE_RELEASE_LIFT"),
                      "pipeline.launch.py 的 REAL_PLACE_RELEASE_LIFT")
            src["place_release_lift"] = (
                "pipeline.launch.py REAL_PLACE_RELEASE_LIFT" if prl is not None
                else "**读不到** —— 见下面的 ⚠️")
        else:
            src["place_release_lift"] = "命令行"
        clear = args.clearance if args.clearance is not None else p["approach_clearance"]
        src["clearance"] = ("命令行" if args.clearance is not None
                            else f"节点参数 {args.node}")
        src["table_z"] = "命令行（必填）"
        src["block_height"] = ("命令行" if args.block_height != REAL_BLOCK_HEIGHT
                               else f"命令行默认（真机 {REAL_BLOCK_HEIGHT}）")

        missing = [k for k, v in (("tool0_to_fingertip", ttf),
                                  ("place_release_lift", prl)) if v is None]
        if missing:
            print(f"!! 读不到 {missing}（{PIPELINE_LAUNCH} 不在？）——"
                  " 拒绝用自己的一份副本顶上（那正是「两套几何」的来源）："
                  "请用命令行显式给出。")
            return 2

        links = list(p["guard_links"])
        floor = args.table_z + p["guard_margin"]
        zs = z_levels(args.table_z, args.block_height, ttf, prl, clear)
        levels = {k: z for k, z in zip(LEVEL_KEYS, zs)}
        grips = {k: (p["sim_gripper_open"] if k.startswith("grasp")
                     else p["sim_gripper_close"]) for k in LEVEL_KEYS}

        print("=" * 78)
        print("桌面安全包线（真机几何，只读）")
        print("=" * 78)
        print("参数来源（真机几何一律走命令行，**不**从节点参数取 ——"
              " 节点跑的是仿真 yaml）：")
        print(f"  台面 z            {args.table_z:+.4f}  ← {src['table_z']}")
        print(f"  方块高            {args.block_height:.4f}  ← {src['block_height']}")
        print(f"  tool0→抓取点      {ttf:.4f}  ← {src['tool0_to_fingertip']}")
        print(f"  放置余量          {prl:.4f}  ← {src['place_release_lift']}")
        print(f"  接近余量          {clear:.4f}  ← {src['clearance']}")
        print(f"  护栏下限          {floor:+.4f}  = 台面 + 余量 {p['guard_margin']:.3f}"
              f"（guard_margin ← 节点参数）")
        print(f"  护栏连杆          {links}（← 节点参数）")
        print(f"  规划组 {p['ik_group']} / 末端 {p['ik_link']} / 系 {p['base_frame']}"
              f"（← 节点参数{'，被 --ik-group 覆盖' if args.ik_group else ''}）")
        print(f"  IK 种子           {[round(v, 3) for v in p['ik_seed']]}（← 节点参数）")
        if retry_seed == "home":
            retry_seed = list(p["home_joints"])
        if retry_seed is not None:
            print(f"  重试种子          {[round(v, 3) for v in retry_seed]}"
                  f"（← 命令行 --retry-seed"
                  f"{'，取节点的 home_joints' if args.retry_seed.strip() == 'home' else ''}）")

        # 节点参数与本次用的真机值不同 ⇒ 大声说出来。分叉不出声正是第八十三轮
        # 修掉的那类病（同一张图两套几何）。
        for name, mine, theirs in (("tool0_to_fingertip", ttf, p["tool0_to_fingertip"]),
                                   ("place_release_lift", prl, p["place_release_lift"]),
                                   ("table_plane_z", args.table_z, p["table_plane_z"])):
            if isinstance(theirs, (int, float)) and abs(theirs - mine) > 1e-9:
                print(f"  ⚠️ 节点参数 {name}={theirs} **与本次用的 {mine} 不同** ——"
                      f"本次按 {mine} 算（真机几何）。要按节点那份算就显式传参。")

        print(f"\n四个 tool0 高度（base_frame，臂会全走一遍）：")
        for k in keys:
            print(f"  {LEVEL_LABEL[k]:<8}{levels[k]:+.4f}"
                  f"（比台面高 {levels[k] - args.table_z:+.4f}）")
        print(f"  ⇒ 判读取**四个里最坏的那个**；接近点不是更安全的一层，"
              f"它只是高一点、IK 分支可能不同。")

        xs = axis(args.x_min, args.x_max, args.step)
        ys = axis(args.y_min, args.y_max, args.step)
        print(f"\n扫描 x∈[{xs[0]:.3f},{xs[-1]:.3f}] × y∈[{ys[0]:+.3f},{ys[-1]:+.3f}]"
              f"，实际 {len(xs)}×{len(ys)}={len(xs)*len(ys)} 格 × {len(keys)} 个高度")
        print(legend(args.near_band, with_retry=retry_seed is not None))
        print("=" * 78)

        cells, per_level = {}, {k: {} for k in keys}
        counts = {k: {} for k in keys}
        reasons, fk_fail = {}, 0
        retry_hits = {}                 # 重试后各判读的格数（只有 --retry-seed 时有）
        worst_z, worst_where = None, None
        worst_links = {}                # 最坏那格两条护栏连杆各自的高度
        t0 = time.monotonic()
        for j, y in enumerate(ys):
            for x in xs:
                syms = []
                for k in keys:
                    arm, why = pr.solve_ik(p, x, y, levels[k], p["ik_seed"], grips[k])
                    if arm is None:
                        per_level[k][(x, y)] = NOIK
                        reasons[why] = reasons.get(why, 0) + 1
                    else:
                        h = pr.fk_heights(p, arm, links)
                        if not h:
                            fk_fail += 1
                        else:
                            lo = min(h.values())
                            if worst_z is None or lo < worst_z:
                                worst_z, worst_where = lo, (x, y, k)
                                worst_links = dict(h)
                        per_level[k][(x, y)] = classify_one(h, floor, args.near_band)
                    syms.append(per_level[k][(x, y)])
                s = worst(syms)
                # 配的种子全解不出时才动重试种子 —— 只在**失败**上加信息，
                # 不去覆盖「配的种子能解」的那些格子（那才是规划器会走的解）。
                if retry_seed is not None and s == NOIK:
                    rsyms = []
                    for k in keys:
                        arm, why = pr.solve_ik(p, x, y, levels[k], retry_seed,
                                               grips[k])
                        if arm is None:
                            rsyms.append(NOIK)
                        else:
                            rsyms.append(classify_one(pr.fk_heights(p, arm, links),
                                                      floor, args.near_band))
                    s = combine(syms, rsyms)
                    retry_hits[s] = retry_hits.get(s, 0) + 1
                cells[(x, y)] = s
                for k in keys:
                    c = counts[k]
                    c.setdefault(per_level[k][(x, y)], 0)
                    c[per_level[k][(x, y)]] += 1
            print(f"  … y={y:+.2f} 扫完（{j + 1}/{len(ys)}，"
                  f"{time.monotonic() - t0:.0f}s）", flush=True)

        print("\n" + "=" * 78)
        if args.map == "worst":
            print(f"图：**所有高度取最坏**（{len(keys)} 个高度合成）")
            shown = cells
        else:
            note = ("——⚠️ 这一层**不**含 --retry-seed 的结果（重试只进 worst 那张图）"
                    if retry_seed is not None else "")
            print(f"图：只有「{LEVEL_LABEL[args.map]}」这一层"
                  f"（{levels[args.map]:+.4f}）{note}")
            shown = per_level[args.map]
        print(render_map(shown, xs, ys))

        # 两张表：从近边起算给的是「x 上界」，从远边起算给的是「x 下界」。
        # 撞桌区贴着基座时**只有第二张表说得出来话**（第八十四轮实测，
        # 那次 21 行里 9 行在近边就是撞桌格、第一张表全给 —）。两张都打。
        lim = row_limits(shown, xs, ys)
        lim_out = row_limits_outer(shown, xs, ys)
        print("\n每行的连续可用边界（从近边 x 最小处起算；遇到第一个不可用就停）：")
        print(f"{'y':>7}   {'x 安全 ≤':>10}   {'x 可用 ≤':>10}")
        for y in reversed(ys):
            a, b = lim[y]
            mark = "" if a == b else "   ← 有近限格"
            print(f"{y:+7.2f}   {_fmt_x(a):>10}   {_fmt_x(b):>10}{mark}")

        print("\n每行的连续可用边界（从**远边** x 最大处起算；给出的是 **x 的下界**）：")
        print(f"{'y':>7}   {'x 安全 ≥':>10}   {'x 可用 ≥':>10}")
        for y in reversed(ys):
            a, b = lim_out[y]
            mark = "" if a == b else "   ← 有近限格"
            print(f"{y:+7.2f}   {_fmt_x(a):>10}   {_fmt_x(b):>10}{mark}")

        # ---- 汇总：把两类「越界」分开说，它们要的处置完全不同 ----
        print("\n" + "=" * 78)
        n = len(cells)
        blocks = [(x, y) for (x, y), s in cells.items() if s == BLOCKED]
        noiks = [(x, y) for (x, y), s in cells.items() if s == NOIK]
        seeds_only = [(x, y) for (x, y), s in cells.items() if s == SOLVABLE_OTHER]
        print(f"汇总（{n} 格）：安全 {sum(1 for s in cells.values() if s == SAFE)}"
              f" / 近限 {sum(1 for s in cells.values() if s == NEAR)}"
              f" / 解不出 {len(noiks)} / **低于下限 {len(blocks)}**"
              + (f" / 换种子才可解 {len(seeds_only)}" if retry_seed is not None else ""))
        if blocks:
            bx = [x for x, _ in blocks]
            by = [y for _, y in blocks]
            print(f"  ⛔ 撞桌格：x∈[{min(bx):.2f},{max(bx):.2f}]"
                  f" y∈[{min(by):+.2f},{max(by):+.2f}]，共 {len(blocks)} 格")
            print(f"     ⇒ 这两条边界就是 pick_zone / place_zone 该躲开的地方。"
                  f"**划区时宁愿比它保守**（护栏下限之上还有 link 本体的半径）。")
        else:
            print("  ✅ 这片范围里没有「解出来的解会把连杆压到台面下」的格子。")
        if noiks:
            nx = [x for x, _ in noiks]
            ny = [y for _, y in noiks]
            print(f"  · 够不到（两种种子都解不出）：{len(noiks)} 格，"
                  f"x∈[{min(nx):.2f},{max(nx):.2f}] y∈[{min(ny):+.2f},{max(ny):+.2f}]")
            print(f"    ⇒ 这一类**不是**新风险（仿真里同样够不到），只是说明"
                  f"区框到这儿就白框了。")
        if retry_seed is not None:
            print(f"  · 重试种子（{[round(v, 3) for v in retry_seed]}）的命中："
                  + "、".join(f"{k}={v}" for k, v in sorted(retry_hits.items()))
                  + "（都是配的种子解不出、才动重试的那些格子）")
        if seeds_only:
            sx = [x for x, _ in seeds_only]
            sy = [y for _, y in seeds_only]
            print(f"  ⚠️ **种子问题** {len(seeds_only)} 格：x∈[{min(sx):.2f},{max(sx):.2f}]"
                  f" y∈[{min(sy):+.2f},{max(sy):+.2f}]")
            print(f"     这些格子**够得到**（换 `home_joints` 当种子就解出来了），"
                  f"只是配的 `ik_seed` 解不出。两种处置，**别搞混**：")
            print(f"       · 想抓它们 ⇒ 改 `ik_seed`（它现在是仿真窄带 y∈"
                  f"{{-0.12,0,+0.12}} 调出来的，偏在一侧）；")
            print(f"       · 不改种子 ⇒ 规划器会在这一整片**静默跳过**"
                  f"（只打一条 IK 失败日志），区框到这儿等于白框。")
            print(f"     ⚠️ 说准一件事：`_call_ik` 现在**没有**「解不出来就换种子重解」"
                  f"这一步（那条 `shoulder_pan > 1.40` 的判据已删，D54）—— "
                  f"换种子重解只在**解出来之后、肘高护栏不过**时才发生，"
                  f"所以这些格子规划器就是解不出 ⇒ **跳过该目标**（只打一条日志）。")
        if fk_fail:
            print(f"  ⚠️ /compute_fk 有 {fk_fail} 次没回话 —— 那些格子按「解不出」"
                  f"计数，被算进了上面那一类，别当成几何结论。")
        for why, cnt in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"     · IK {why}: {cnt}")
            if "未应答" in why:
                print("       ↳ 服务在但不回话：move_group 没有 /joint_states 时"
                      " CurrentStateMonitor 拿不到状态，/compute_ik 会**挂住**。")
            elif why == "FRAME_TRANSFORM_FAILURE":
                print("       ↳ 请求帧的 TF 查不到：多半少了 robot_state_publisher"
                      "（真机上由驱动顺带起）—— 不是几何问题。")
        if worst_where:
            x, y, k = worst_where
            # 说清是**哪条**连杆最低：现场要拿它和「护栏在按 −0.39 判」对比 ——
            # 这一格在今天的护栏下是**放行**的（−0.094 > −0.37），这正是本脚本
            # 存在的理由，所以把两个数摆在一起看。
            which = "、".join(f"{ln}={zv:+.4f}" for ln, zv in sorted(
                worst_links.items(), key=lambda kv: kv[1]))
            print(f"  · 全场最低连杆 {worst_z:+.4f}，出现在 ({x:.2f},{y:+.2f}) 的"
                  f"「{LEVEL_LABEL[k]}」，离下限 {floor:+.4f} 还有"
                  f" {worst_z - floor:+.4f} m")
            print(f"    （该格各连杆：{which}）")
            if p["table_plane_z"] is not None and p["table_plane_z"] != args.table_z:
                old_floor = p["table_plane_z"] + p["guard_margin"]
                if worst_z >= old_floor:
                    print(f"    ⇒ **按节点参数那份护栏判（table_plane_z="
                          f"{p['table_plane_z']:+.2f} ⇒ 下限 {old_floor:+.4f}）"
                          f"会放行它**：这条连杆 {worst_z:+.4f}，而真机台面在 "
                          f"{args.table_z:+.4f} ⇒ 它已在**台面以下"
                          f" {args.table_z - worst_z:.4f} m**。")
                else:
                    print(f"    ⇒ 这一格连节点参数那份护栏（下限 {old_floor:+.4f}）"
                          f"也会拦下。")

        usable = [(x, y) for (x, y), s in cells.items() if s in (SAFE, NEAR)]
        if usable:
            print(f"  · 可用区（安全+近限）的 x 上界 {max(x for x, _ in usable):.2f}、"
                  f"y∈[{min(y for _, y in usable):+.2f},"
                  f"{max(y for _, y in usable):+.2f}]")

        print("\n提醒：本脚本给的是**几何边界**，它不含：")
        print("  · 相机视野（看得见 ≠ 够得到，反之亦然）—— 那是 pick_zone 的另一半；")
        print("  · 夹爪/腕部本体的宽度（本判据与护栏同口径：只看连杆原点高度）；")
        print("  · 桌面进 MoveIt 规划场景（W1b 第 2 项）与单步低速的首动。")

        if args.json:
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump({
                    "tool": "table_safety_envelope",
                    "table_z": args.table_z, "block_height": args.block_height,
                    "tool0_to_fingertip": ttf, "place_release_lift": prl,
                    "clearance": clear, "guard_margin": p["guard_margin"],
                    "floor": floor, "guard_links": links,
                    "ik_group": p["ik_group"], "ik_link": p["ik_link"],
                    "base_frame": p["base_frame"],
                    "levels": {k: levels[k] for k in keys},
                    "grips": {k: grips[k] for k in keys},
                    "ik_seed": list(p["ik_seed"]),
                    "retry_seed": retry_seed,
                    "retry_hits": retry_hits,
                    "xs": json_axis(xs), "ys": json_axis(ys),
                    "cells": {cell_key(x, y): cells[(x, y)] for x, y in cells},
                    "per_level": {k: {cell_key(x, y): per_level[k][(x, y)]
                                      for x, y in per_level[k]} for k in keys},
                    "row_limits": {f"{y:.4f}": list(lim[y]) for y in ys},
                    "row_limits_outer": {f"{y:.4f}": list(lim_out[y])
                                         for y in ys},
                    "counts": {k: counts[k] for k in keys},
                    "worst_link_z": worst_z,
                    "worst_links": worst_links,
                    "worst_at": list(worst_where) if worst_where else None,
                }, f, ensure_ascii=False, indent=1)
            print(f"\n网格已转储：{args.json}")

        print(f"\n用时 {time.monotonic() - t0:.0f}s（{n * len(keys)} 次 IK + FK）")
        # 退出码：**这里不是闸门**。图上出现撞桌格是结论（扫的范围本来就比任何
        # 合理的区大），不是失败；只有「一个位姿都没解出来」才是环境问题 ——
        # 那时「没有撞桌格」是假绿，跟肘高探针的 return 2 同一个理由。
        if not any(s != NOIK for s in cells.values()):
            print("!! 一个位姿都没解出来 —— 先查 ik_group 与 /compute_ik。"
                  "（这种情况下「没有撞桌格」是假绿）")
            return 2
        return 0
    finally:
        pr.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
