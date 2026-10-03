#!/usr/bin/env python3
"""**同一个位姿、同一个种子，连问 N 次 `/compute_ik`** —— 解会不会变？

为什么要有这个工具（第八十七轮实测掉出来的）：

    同一份包线脚本、**同一个 move_group 进程**、同样的命令行，两次全扫有
    **19/357 格（5.3%）**判读不同 —— 有格子从「撞桌」翻成「安全」，也有反过来
    的。病根不在包线脚本，在**解算器**那头。

真机那个规划组（`ur_manipulator`）的 KDL 配置是系统自带的
`/opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml`：

    kinematics_solver_attempts: 3          # ← 解与种子不一致时会**随机重启**
    （没有 kinematics_solver_consistency_limits）

而仿真那份 `arm_grasp_sim/config/kinematics.yaml` 早就把这两项钉死了
（`attempts: 1` + `consistency_limits: [100]*6`，注释写着「KDL 的随机重启是
**不可复现**的来源」）—— 那是 D54 的修法，**只打在了仿真那一份上**。

后果不止「包线量不准」：`planner_node._call_ik` 拿到的第一个解随机落在哪个肘
分支，决定了 `_posture_ok`（肘高护栏）过不过、要不要换分支重解。**真机上同一个
目标位姿，这次可能被接受、下次可能被判「两个分支都不满足护栏」而整个跳过。**

这个工具把这件事变成一条**可执行、可复现**的判据：同一个位姿问 N 次，数一数
回来了几个**不同的解**。

    $ python3 scripts/ik_repeat_check.py --table-z 0.0 --x 0.20 --y 0.10
    $ python3 scripts/ik_repeat_check.py --table-z 0.0 --x 0.20 --y 0.10 --n 30

**只读**：只调 `/compute_ik` 与 `/compute_fk`，不发轨迹、不写寄存器、不碰真机。

退出码：0 = 每个位姿都只回来**一个**解（可复现）；1 = 有解分支翻转（不可复现，
应当去修 `attempts` / `consistency_limits`）；2 = 环境问题（服务不在等）。
"""

import argparse
import os
import sys

# ⚠️ 必须在 import 被测模块**之前**设（理由见 test_plot_table_envelope.py 的
# 大段说明：`.pyc` 的失效判据是 (mtime, size)，等长改写 + 同秒写入会被静默吞掉）。
sys.dont_write_bytecode = True

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ----------------------------------------------------------------------
# 纯函数：摊平与判词。**不碰 ROS**，所以自检够得着（test_ik_repeat.py）。
# ----------------------------------------------------------------------
def tally(solutions, ndigits=3):
    """``[(解, 出现次数), ...]`` —— 按**出现次数从多到少**排；解为 ``None`` 的排最后。

    ``None`` 是「这一次解不出来」，它与「解出来但不同」是两件事，必须分开数
    （混在一起会把「服务偶尔超时」误报成「分支翻转」）。

    ⚠️ 比对前先按 ``ndigits`` 取整：KDL 的数值解同一个分支也会有末位噪声
    （``0.7853981633974483`` vs ``0.7853981633974485``），不取整会把一个分支
    数成好几个 —— 这个工具就废了。取整位数是**判据的一部分**，不是格式化。
    """
    counts = {}
    for s in solutions:
        key = None if s is None else tuple(round(float(v), ndigits) for v in s)
        counts[key] = counts.get(key, 0) + 1
    return sorted(counts.items(), key=lambda kv: (kv[0] is None, -kv[1]))


def spread(branch, seed):
    """``branch`` 与 ``seed`` 的最大逐关节差（rad）；算不出来时返回 ``None``。

    这是「分支翻转」的量化形式：同一分支的差是末位噪声（~1e-4），换一个肘分支
    的差是**几个弧度**。两者差着四个数量级，所以不需要精心挑阈值。

    ⚠️ 关节数不同时返回 ``None``，**不许** `zip` 悄悄截断到短的那个 ——
    两臂关节数不同本来就是「这两个东西不该比」，截断后会给一个看着合理的数，
    而那正是「话说了没查过的事」。
    """
    if branch is None or seed is None:
        return None
    if len(branch) != len(seed):
        return None
    return max(abs(float(a) - float(b)) for a, b in zip(branch, seed))


def verdict(branches, seed, ndigits=3):
    """一句话判词。``branches`` 是 ``tally()`` 的输出。"""
    real = [k for k, _ in branches if k is not None]
    n_fail = sum(v for k, v in branches if k is None)
    if not real:
        return (f"❌ 问了这么多次**一次都没解出来**"
                f"（失败 {n_fail} 次）—— 这不是「可复现」，是够不到。")
    if len(real) == 1:
        tail = f"（另有 {n_fail} 次解不出）" if n_fail else ""
        return (f"✅ 只有一个解 ⇒ **可复现**{tail}。"
                f"包线那张图的逐格判读在这台 move_group 上是可信的。")
    far = max(real, key=lambda k: spread(k, seed) or 0.0)
    d = spread(far, seed)
    return (f"❌ 回来了 **{len(real)} 个不同的解** ⇒ **不可复现**。\n"
            f"     相差最大的一个与种子差 {d:.3f} rad"
            f"（同一分支的噪声是 ~1e-4）—— 这是**肘分支翻转**，不是噪声。\n"
            f"     ⇒ 包线的**逐格**分类在这台上是随机量，只能**按带余量读**\n"
            f"       （离边界至少留一格 0.05 m）；上面那块「最大全安全矩形」\n"
            f"       两次全扫都成立，仍然可用。\n"
            f"     ⇒ 根治：给这个规划组补上 `kinematics_solver_attempts: 1` 与\n"
            f"       `kinematics_solver_consistency_limits`（D54 的修法，\n"
            f"       仿真那份已经有了，真机这份还没打）。")


# ----------------------------------------------------------------------
# 取数：只读
# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="同一位姿连问 N 次 /compute_ik，看解会不会变（只读）")
    ap.add_argument("--node", default="/grasp_planner", help="规划器节点名")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--n", type=int, default=20, help="每个位姿问几次（默认 20）")
    ap.add_argument("--table-z", type=float, default=None,
                    help="给了它就跑包线那**四个 tool0 高度**（与包线同口径）；"
                         "不给就用 --z 单独问一个高度")
    ap.add_argument("--z", type=float, default=None, help="单独问这一个 tool0 高度")
    ap.add_argument("--block-height", type=float, default=None)
    ap.add_argument("--ik-group", default=None, help="覆盖规划组名（默认读节点参数）")
    ap.add_argument("--seed", default=None,
                    help="覆盖种子；默认用节点参数里的 ik_seed（包线用的就是它）")
    args = ap.parse_args()

    if args.z is None and args.table_z is None:
        print("!! 要么给 --table-z（跑四个高度），要么给 --z（单问一个高度）")
        return 2

    import rclpy                                              # noqa: E402
    from elbow_guard_probe import Probe                       # noqa: E402
    from table_safety_envelope import (                       # noqa: E402
        LEVEL_KEYS, REAL_BLOCK_HEIGHT, launch_const, num, z_levels)

    # ⚠️ init 必须在 Probe **之前**：Probe.__init__ 起 rclpy 节点，没 init 直接
    # NotInitializedException —— 第一次板上真跑就崩在这。纯函数那一半有自检，
    # 取数这一半没有（本文件 docstring 里「真正的『解会不会变』不在这里测」
    # 说的正是这一半）。样式照抄 table_safety_envelope.main()。
    rclpy.init()
    pr = None
    try:
        # Probe 放进 try 里：它自己抛异常时 finally 里的 `pr.destroy_node()`
        # 会 NameError，把真正的错盖成一句看不懂的 —— 崩溃信息要指向病根。
        pr = Probe(args.node, ik_group=args.ik_group)
        if not pr.ik.wait_for_service(timeout_sec=10.0):
            print("!! /compute_ik 不可用 —— move_group 起了吗？")
            return 2
        p = pr.read_params()

        if args.seed:
            seed = [float(v) for v in args.seed.split(",")]
        else:
            seed = list(p["ik_seed"])

        if args.z is not None:
            targets = [("（命令行 --z）", float(args.z))]
            grips = {0: None}
        else:
            bh = args.block_height if args.block_height is not None else REAL_BLOCK_HEIGHT
            ttf = num(launch_const("REAL_TOOL0_TO_FINGERTIP"),
                      "pipeline.launch.py 的 REAL_TOOL0_TO_FINGERTIP")
            prl = num(launch_const("REAL_PLACE_RELEASE_LIFT"),
                      "pipeline.launch.py 的 REAL_PLACE_RELEASE_LIFT")
            if ttf is None or prl is None:
                print("!! 读不到 pipeline.launch.py 里的真机几何常数 —— "
                      "拒绝用自己的一份副本顶上（那正是「两套几何」的来源）")
                return 2
            zs = z_levels(args.table_z, bh, ttf, prl, p["approach_clearance"])
            targets = list(zip(LEVEL_KEYS, zs))
            # 与包线同口径：抓取那两层是**张开**的爪子，放置那两层是**闭合**的。
            # 「闭合」那一支正是 `place_release_lift` 那个只在手里有东西时才发作的坑。
            grips = {i: (p["sim_gripper_open"] if k.startswith("grasp")
                         else p["sim_gripper_close"])
                     for i, (k, _) in enumerate(targets)}

        print("=" * 74)
        print("IK 可复现性检查（只读：只调 /compute_ik，不发轨迹、不写寄存器）")
        print("=" * 74)
        print(f"  规划组 {p['ik_group']} / 末端 {p['ik_link']} / 系 {p['base_frame']}")
        print(f"  种子   {[round(v, 3) for v in seed]}"
              f"{'（命令行 --seed）' if args.seed else '（← 节点参数 ik_seed，包线用的就是它）'}")
        print(f"  位姿   x={args.x:.3f}  y={args.y:.3f}")
        print(f"  每个高度问 {args.n} 次")

        bad = 0
        for i, (label, z) in enumerate(targets):
            print(f"\n--- {label}  tool0 z = {z:+.4f} ---")
            sols = []
            reasons = {}
            for _ in range(args.n):
                arm, why = pr.solve_ik(p, args.x, args.y, z, seed, grips.get(i))
                sols.append(None if arm is None else tuple(float(v) for v in arm))
                if why:
                    reasons[why] = reasons.get(why, 0) + 1
            br = tally(sols)
            real = [k for k, _ in br if k is not None]
            print(f"  不同的解：{len(real)} 个" + (f"；解不出 {sum(v for k, v in br if k is None)} 次"
                                                  if any(k is None for k, _ in br) else ""))
            for k, n in br:
                if k is None:
                    print(f"    × {n:>3} 次  解不出")
                    continue
                d = spread(k, seed)
                print(f"    × {n:>3} 次  与种子最大关节差 {d:.4f} rad"
                      f"   {[round(a, 3) for a in k]}")
            if reasons:
                print("  （失败原因：" + "、".join(f"{k}×{v}" for k, v in reasons.items()) + "）")
            v = verdict(br, seed)
            print("  " + v.replace("\n", "\n  "))
            if len(real) > 1:
                bad += 1

        print("\n" + "=" * 74)
        if bad:
            print(f"结论：{bad} / {len(targets)} 个位姿**不可复现**。")
        else:
            print(f"结论：{len(targets)} 个位姿全部**可复现**（每个只回来一个解）。")
        return 1 if bad else 0
    finally:
        if pr is not None:
            pr.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
