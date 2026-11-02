#!/usr/bin/env python3
"""把「抓哪一块、放哪个槽」变成一份**可执行、且预检过**的关节计划（计划 JSON v2）。

这一层不碰硬件、不发轨迹：输入是候选 JSON + URDF + 当前关节角，输出是文件。
执行是 ``precision_exec.py`` 的事；两者分开，是因为**计划可以离线重放**
（今天 live-04 那一帧就能走完整条链），而执行不能重来。

为什么不走 MoveIt
-----------------
现在板上的栈是 ``start_moveit:=false``（见 ``docs/待执行命令.md`` 的启动参数）。
今天 A/B 两次真机抓取用的就是这套数值 IK + 两点式 JTC，一次到位、误差 <0.01°。
再起一套 move_group 只会多一个会漂的组件。

几何约定（**全部来自规划器，别在这里另立一套**）
------------------------------------------------
    tool0 指尖偏置        0.1757 m   (= 裸指尖 204.2 − A1 方块高 28.5)
                                    ⚠️ 这是**参数**不是尺寸：式子 wz + 0.1757 + c 里的
                                    「指尖」在顶面以下，夹持深度与块高无关。
                                    裸指尖（闭合）是 0.2042 m，报深度时用它。
    wz 用**相对台面的高度** `h`，不用绝对顶面 z —— 腕部深度实测把台面读高
    ~6 mm（本轮 table_z=+6.0），而几何锚在 table_plane_z=0。
    抓取抬升              0.01 m
    接近余量              0.05 m
    搬运高度              0.15 m
    放置：**块底**离台面  0.005 m

放置下降的高度**不能写死**。夹住之后，块底挂在指尖以下「块高 − 抓取深度」那么长，
所以要让**块底**落在台面上方 5 mm：

    place_z = 台面 + (块高 − 抓取深度) + 0.005 + 裸指尖 0.2042

旧做法写死成 0.0285 + 0.1757 + 0.01 = 0.2142，那三个数里 0.0285 正好是 A1 的块高，
等价于「28.5 mm 的块恰好落到台面」。换成 29.6 mm 的块，块底就压进台面 1.1 mm ——
2026-10-26 现场看到的就是「下放太深才松爪，把块死死压在桌上」。

姿态固定为「竖直朝下」rpy=(180°,0,ψ)，ψ 默认 0：AG-95 的手指轴相对 tool0
的方向没有文档也没实测过，本轮不跟 GraspNet 的平面内角（它只记录在 meta 里）。
A/B 把腕转 180° 仍然抓成功，说明当前方块尺度下朝向不敏感。

预检（任一不过就**不写计划**，退出码 2）
----------------------------------------
1. IK 残差 < 0.1 mm / 0.05°
2. 软限位（URDF 的 safety_controller，不是硬限位）
3. 高度护栏：肘 / 腕 1 的 z ≥ 台面 + 20 mm；**裸指尖**（tool0 + R·[0,0,0.2042]）
   的 z ≥ 台面 + 8 mm。关节空间逐段采样 8 点。指尖单独查，是因为夹着块横移时
   先碰台面的是它，而肘、腕 1 都还在高处 —— 只查连杆等于没查。
4. 段内最大关节跳变 ≤ 1.0 rad，超过就插中间航点
5. 腕 2 离 0 至少 0.15 rad（0 是腕奇异；插点也要查 —— 端点远离不等于插值点远离）
6. 高度钳制：抓取 z ∈ [0.2142, 0.2242]（裸指尖离台面 10~20 mm）
7. 邻块间距 ≥ 60 mm（夹爪全开 95 mm；A/B 实测 77 mm 不碰）
8. **整链选支**：同一末端位姿常有多支 IK 解，数值 IK 从上一个航点出发会落进
   其中一支。逐点就近选不够（2026-10-26 第 6 轮：to_slot 就近那一支下一步
   前臂掉到 49.7 mm，再一步就钻到台面下）。改成整条航点链做一次最小总关节
   行程的 DP，段与段之间仍逐段过高度护栏 —— 存不下任何一支就报错。

用法：
  # 走到抓取巡检位 / 放置巡检位
  python3 precision_plan.py --survey pick  --urdf robot.urdf --from-frame f0 --out p.json
  # 抓一块、放到某个槽
  python3 precision_plan.py --pick --candidates cand.json --block 2 \\
      --slot 0.45,-0.25 --urdf robot.urdf --from-frame f0 --out pick.json
  # 到任意 tool0 位置
  python3 precision_plan.py --move 0.53,0.27,0.36 --urdf robot.urdf --from-frame f0 --out m.json
"""
import argparse
import json
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402
from scipy.spatial.transform import Rotation as Rot               # noqa: E402

import urkin                                                      # noqa: E402

# ---- 几何常量（出处见模块 docstring）
# `TOOL0_TO_FINGERTIP` 这个**参数** = 裸指尖（闭合，tool0→指尖 204.2 mm）减去
# A1 那批方块的高度 28.5 mm。所以 `wz + TOOL0_TO_FINGERTIP + c` 这个式子里的
# 「指尖」落在**顶面以下 (204.2 − 28.5 − ...) **，夹持深度与块高无关。
# 要报「指尖在顶面下多深」时必须用**裸指尖**这个数，不能用参数。
TOOL0_TO_BARE_FINGERTIP = 0.2042
TOOL0_TO_FINGERTIP = 0.1757
APPROACH_CLEARANCE = 0.05
GRASP_CLEARANCE = 0.01
LIFT_HEIGHT = 0.15
PLACE_CLEARANCE = 0.005        # 放置时**块底**离台面多高（松开后落这 5 mm）
PLACE_APPROACH_GAP = 0.04      # 放置接近点比下降点高多少（沿旧值的 0.2542−0.2142）
# 抓取时**裸指尖**离台面的下限。0.2142 这个老数值碰巧 = 0.2042 + 0.010，
# 所以下限本身没变，只是不再借用「放置下降高度」当理由 —— 那两件事是独立的：
# 抓取深度由块高/参数 0.1757 决定，放置高度由**块底挂多长**决定。
GRASP_TIP_MIN_Z = 0.010
GRASP_Z_MIN = GRASP_TIP_MIN_Z + TOOL0_TO_BARE_FINGERTIP   # 0.2142
GRASP_Z_MAX = GRASP_Z_MIN + 0.010      # 上限给 10 mm：块高 30~35 mm，实测够用
# 腕 2（第 5 关节）离 0 至少这么远。0 就是腕奇异（1、3 轴重合），
# A/B 两次实测腕 2 = −89.8°，余量极大；这个数只是防止 IK 挑到另一支解。
WRIST_MIN_RAD = 0.15

# 巡检位（tool0，m）。相机相对 tool0 的偏置是 (−31.9, +85.3, −24.2) mm，
# 由 live-04 的 URDF FK 与帧内 TF 逐位对上（差 0.0 mm）。
# 这两个位置各解一次 IK，残差 0.000 mm / 0.000°。
SURVEY_TOOL0 = {
    "pick": (0.5319, 0.2747, 0.3643),     # 相机 ~(500, 360, 340) mm，三块都在画面中部
    "place": (0.5319, -0.3853, 0.3643),   # 相机 ~(500, -300, 340) mm，四个槽都在画面内
    # 阶段 2 的第二个终态位姿：同一放置区，tool0 平移 +45/+35 mm、高度不变。
    # 用途是「同一批已放块从**两个不同视点**各量一次」—— 同一个视点连拍两帧
    # 证明不了多姿态一致性（误差会一起共模）。位移量按 0.34 m 处的视场留足重叠。
    "place2": (0.5769, -0.3503, 0.3643),
    # ---- 阶段 2（2026-10-27 实测：垫心 base (422.7, −114.9) mm，块在拾取侧
    # x 0.39–0.58 / y 0.18–0.40）。相机（color optical）≈ tool0 + (−31.9, +85.1,
    # −24.7) mm（yaw 0）。三个位姿都保持俯视 yaw 0：
    #   stage2high  —— 高位广视：两页标记 + 九格同框（B1 垫子解算；RMS 1.0 px）
    #   stage2close —— 凑近：九格 + 块全在画面，用于终态判分（第二视点）
    #   stage2pick  —— 拾取侧：0.40 m 高，9 块全在视场（0.34 m 装不下 y 跨度）
    "stage2high": (0.452, -0.2000, 0.547),    # 相机 ~ (420, −115, 523) mm
    "stage2close": (0.452, -0.2000, 0.387),   # 相机 ~ (420, −115, 362) mm
    # 终态第二视点：与 stage2close **同高**、平移 +40/+30 mm（相机 ~ (460, −85,
    # 362)）。2026-10-27 B2 判决：stage2high 顶面碎、yaw 不可信，终态判分不再用它；
    # 但「同一视点连拍」证明不了多姿态一致性，所以第二视点要换个位置而不是换个高度。
    # 0.362 m 处视场约 510×287 mm，九格 180×180 mm 平移 50 mm 仍全在画面。
    "stage2close2": (0.492, -0.1700, 0.387),
    "stage2pick": (0.517, 0.207, 0.425),      # 相机 ~ (485, 292, 400) mm
}

GUARD_LINKS = ("forearm_link", "wrist_1_link")
GUARD_MARGIN = 0.02         # 大件（肘 / 腕 1）离台面的下限
GUARD_TIP_MARGIN = 0.008    # **裸指尖**离台面的下限（10 mm 抓取深度留 2 mm 余量）
MAX_JOINT_JUMP = 1.0
# 单个关节字面行程上限。超过它说明选支选到了「同一个位姿、绕了一圈」的那一支：
# 2026-10-27 B2 第 5 轮块 #2 就是这么让腕整圈翻转、夹爪朝天的。这是**红线**，
# 不是可插点消化的跳变 —— 所以不按 MAX_JOINT_JUMP 处理。
MAX_JOINT_SWING = float(np.pi)
# 腕 1 / 腕 3 是沿工具轴的两个可整圈关节：字面角离 0 越远，夹爪线缆拧得越死。
# 2026-10-27 回位那次把 wrist_1 转到 −262.6°、wrist_3 到 −253.4°，人只能手掰回来。
# 210° 这个口子卡在两处之间：出厂正常姿势都在 ±180° 内；此前跑通的计划最狠到
# −204.1°，放行；拧死的那次是 −262.6°，拦下。
WOUND_JOINTS = ("wrist_1_joint", "wrist_3_joint")
MAX_WRIST_TURN = float(np.radians(210.0))
GUARD_SAMPLES = 8
TABLE_PLANE_Z = 0.0
MIN_BLOCK_GAP = 0.060

# ---- 阶段 3（堆叠）用的几何常量
# 相机（color optical）相对 tool0 的偏置（m，yaw 0）；出处见 SURVEY_TOOL0 的注释。
STAGE3_CAMERA_OFFSET = (-0.0319, 0.0851, -0.0247)
# 高度跟踪巡检位：**相机到塔顶**的距离恒定。默认值复现 stage2close 在垫上的距离
# （tool0 387 − 相机下移 24.7 − 第 1 层顶 ~31.5 ≈ 331 mm）；入口在
# {300, 331, 365} mm 三档实测后定档。固定 stage2high 不可用（顶面碎、yaw 不可信）。
STAGE3_SURVEY_DEFAULT_D = 0.331
STAGE3_SURVEY_SECOND_OFFSET = (0.040, 0.030)   # 第二位姿相对第一位姿的平移（m）
TOWER_BOX_MARGIN = 0.002       # 塔体 box 半边长 = 块半边长 + 2 mm
HELD_BLOCK_MARGIN = 0.002      # 持块包络再放的余量

# 放置要不要按「抓取偏心」补：**默认不补**。
# 2026-10-26 第 6 轮实测判决：抓取偏心预测 6.44 mm，但抓着块实际只偏 2.76 mm
# （补偿后放置误差 7.07 mm，不补偿的话同一帧量出来会是 2.76 mm）—— 补了反而
# 多偏 4.3 mm。跨三轮看，夹持偏移 9.7 / 14.5 / 2.8 mm 与抓取偏心 2.4 / 5.5 /
# 6.4 mm 不对应，说明「块心停在抓取时那个偏心上」这个假设不成立。
# 开关与 meta 里的 grasp_ecc_mm 都留着：等有相机能看见夹着的块、量到真的夹持
# 偏移之后，再用它来判这块要不要开。
#
# 原先这里还拿「块在爪里会转、会移」当旁证（yaw 91.0°→25.4°、爪合到 25.3 mm
# 而块面宽 32~36 mm）。2026-10-26 复核**两条都作废**：25.4° 来自 36.01×36.05
# 的近方形顶面（长短差 0.04 mm），yaw 没有定义；25.3 mm 也不是指尖开口（同轮
# 空载自检全闭读 0.0、全开读 95.0，两端都对，偏差只长在接触段）。块在爪里到底
# 转没转、握在哪，目前既无证据也无反证。
PLACE_COMPENSATE_ECC = False


class PlanError(RuntimeError):
    """预检不过。抛这个而不是 sys.exit，测试里能直接断言。"""


def down_rotation(yaw_deg=0.0):
    """竖直朝下：R = Rz(ψ) · Rx(π)。"""
    return Rot.from_euler("z", yaw_deg, degrees=True).as_matrix() @ \
        Rot.from_euler("x", 180.0, degrees=True).as_matrix()


def target_matrix(xyz, yaw_deg=0.0):
    T = np.eye(4)
    T[:3, :3] = down_rotation(yaw_deg)
    T[:3, 3] = xyz
    return T


def duration_for(q0, q1, seconds_per_rad=3.0, base=1.5, lo=2.0, hi=12.0):
    """运动时长：按最大关节角走 3 s/rad，再加 1.5 s 底 —— 宁可慢。"""
    d = float(np.max(np.abs(np.asarray(q1) - np.asarray(q0))))
    return float(min(hi, max(lo, base + seconds_per_rad * d)))


def check_wrist(chain, q, tol=WRIST_MIN_RAD):
    """腕 2（第 5 关节）过零 ⇒ 腕奇异：第 1 与第 3 关节轴重合，关节速度发散。
    JTC 为了保持末端走直线会把腕甩起来，夹着块横移时尤其危险。
    返回 None 或 (实际角度, 下限角度)，单位都是度。
    """
    try:
        i = chain.move_joints.index("wrist_2_joint")
    except (ValueError, AttributeError):
        i = 4
    v = float(q[i])
    if abs(v) >= tol:
        return None
    return round(float(np.degrees(v)), 2), round(float(np.degrees(tol)), 2)


def check_limits(chain, q):
    """关节角按**原值**判软限位。

    ⚠️ 不要先折算到 ±π 再判：UR 报的关节角不绕圈（可以到 6.2 rad），
    折算会把一个越界的 6.5 rad 变成 −0.78 rad 放过去。软限位本身就在 ±6.133，
    所以原值直接比才对。
    """
    bad = []
    for name, val in zip(chain.move_joints, q):
        lo, hi, _ = chain.limits.get(name, (-np.inf, np.inf, 0.0))
        if not (lo <= float(val) <= hi):
            bad.append((name, round(float(np.degrees(val)), 2)))
    return bad


def check_guard(chain, q0, q1, table_z=TABLE_PLANE_Z, margin=GUARD_MARGIN,
                links=GUARD_LINKS, samples=GUARD_SAMPLES,
                tip_margin=GUARD_TIP_MARGIN):
    """关节空间直线上逐点 FK，检查两件事：

    · 肘 / 腕 1 这些**大件**离台面的高度（擦台的典型是它们，不是指尖），
      下限 `table_z + margin`；
    · **裸指尖**的高度 —— 指尖 = tool0 + R·[0,0,0.2042]（不是简单往下减，
      姿态不是竖直朝下时要按 R 算），下限 `table_z + tip_margin`。
      夹着块横移时真正会碰到台面/别的块的是它，只查连杆查不出来。

    ⚠️ 指尖必须用**裸指尖 0.2042**，不能用参数 0.1757：用 0.1757 的话这条检查实际
    看的是「块顶面那一层」（比真指尖高 28.5 mm），真指尖掉到台面以下 8.5 mm 还判绿。
    2026-10-26 跟放置深度一起修。

    返回 (ok, worst)，worst = (t, 名字, z, 下限)：不通过时是**超出最多**的那个点，
    通过时是最低的那个点。
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    lowest = None            # 全段最低的点（通过时报它）
    worst = None             # 超出下限最多的点（不通过时报它）
    for i in range(samples + 1):
        q = q0 + (q1 - q0) * (i / samples)
        qd = dict(zip(chain.move_joints, q))
        T = chain.fk(qd)
        pts = [(link, float(chain.fk_link(link, qd)[2, 3]), table_z + margin)
               for link in links]
        pts.append(("fingertip",
                    float(T[2, 3] + T[:3, 2] @ np.array([0.0, 0.0,
                                                         TOOL0_TO_BARE_FINGERTIP])),
                    table_z + tip_margin))
        for name, z, low in pts:
            rec = (i / samples, name, z, low)
            if lowest is None or z < lowest[2]:
                lowest = rec
            if worst is None or (low - z) > (worst[3] - worst[2]):
                worst = rec
    if worst is not None and worst[2] < worst[3]:
        return False, worst
    return True, lowest


def check_held_block(chain, q0, q1, half_edge, block_h, below_tip, box,
                     off_xy=(0.0, 0.0), samples=GUARD_SAMPLES,
                     margin=HELD_BLOCK_MARGIN):
    """夹着的块沿关节段移动时，是否撞进**塔体 box**。

    ``box = (x0, x1, y0, y1, z0, z1)``（base 系，米）。每个采样点用 FK 取 tool0：
    块心 xy = tool0_xy + ``off_xy``（抓取偏心，已按放置 yaw 旋转），
    块底 z = tool0_z − ``below_tip``（= 块高 − 抓取深度 + 裸指尖）。
    水平用**外接圆** ``half_edge·√2 + margin`` 判，宁严勿松。

    返回 ``(ok, worst)``；``worst = (t, "held_block@tower", clearance_m, 0.0)``，
    clearance < 0 即侵入量。
    """
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    r = float(half_edge) * np.sqrt(2.0) + margin
    x0, x1b, y0, y1b, z0, z1b = [float(v) for v in box]
    ox, oy = float(off_xy[0]), float(off_xy[1])
    worst = None
    for i in range(samples + 1):
        q = q0 + (q1 - q0) * (i / samples)
        T = chain.fk(dict(zip(chain.move_joints, q)))
        cx, cy = float(T[0, 3]) + ox, float(T[1, 3]) + oy
        zb = float(T[2, 3]) - float(below_tip)
        zt = zb + float(block_h)
        if zt <= z0 or zb >= z1b:
            continue                      # 高度不重叠，水平再近也不算撞
        dx = max(x0 - cx, 0.0, cx - x1b)
        dy = max(y0 - cy, 0.0, cy - y1b)
        clearance = float(np.hypot(dx, dy)) - r
        rec = (i / samples, "held_block@tower", clearance, 0.0)
        if worst is None or clearance < worst[2]:
            worst = rec
    if worst is not None and worst[2] < 0.0:
        return False, worst
    return True, worst


def stage3_survey_tool0(top_z_m, tower_xy_m, distance=STAGE3_SURVEY_DEFAULT_D,
                        second_offset=STAGE3_SURVEY_SECOND_OFFSET):
    """阶段 3 的**高度跟踪**巡检位对（tool0，m）。

    相机到**塔顶**的距离恒为 ``distance``：``tool0_z = top_z + 24.7 mm + distance``。
    xy 让相机正对塔心；第二位姿再平移 ``second_offset``（换视点，防共模误差）。
    """
    dx, dy, dz = STAGE3_CAMERA_OFFSET
    z = float(top_z_m) - dz + float(distance)
    x = float(tower_xy_m[0]) - dx
    y = float(tower_xy_m[1]) - dy
    return [(x, y, z),
            (x + second_offset[0], y + second_offset[1], z)]


def solve_waypoint(chain, xyz, yaw_deg, seed, max_iter=200):
    T = target_matrix(xyz, yaw_deg)
    q, pe, re, nfev = chain.ik(T, seed, max_iter=max_iter)
    # 数值 IK 可能收敛到「同一个位姿、差 2π」的那一支；折回离种子最近的
    # 字面值，否则执行器会照字面整圈转过去（见 unwrap_near 的注释）。
    q = unwrap_near(chain, q, seed)
    return q, pe, re, nfev


def pose_residual(chain, q, xyz, yaw_deg=0.0):
    """已有 q 相对目标位姿的残差：位置 mm、姿态 deg。"""
    T = chain.fk(q)
    Tt = target_matrix(xyz, yaw_deg)
    pe = float(np.linalg.norm(T[:3, 3] - Tt[:3, 3])) * 1000
    re = float(np.degrees(np.linalg.norm(
        Rot.from_matrix(Tt[:3, :3].T @ T[:3, :3]).as_rotvec())))
    return pe, re


def joint_dist(a, b):
    """关节空间距离，按 2π 取最短 —— 同一个位姿的 q 与 q+2π 是同一点。

    ⚠️ 这个函数只配用来**判重**（「两支解其实是一支吗」）。**不能拿它算行程**：
    执行器走的是字面值，见 `travel_cost`。
    """
    d = (np.asarray(a, float) - np.asarray(b, float) + np.pi) % (2 * np.pi) - np.pi
    return float(np.linalg.norm(d))


def travel_cost(a, b):
    """关节行程按**字面值**算（欧氏、不折 2π）—— JTC 插的就是字面值。

    与 `joint_dist` 的分工是硬分工：折算后的距离对「这两个 q 是不是同一个
    位姿」是对的，对「这条命令要转多少度」是错的。
    """
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def unwrap_near(chain, q, ref):
    """每个关节加减 2π 到离 `ref` **最近的等价取值**（末端位姿一个字节都不变）。

    2026-10-27 B2 第 5 轮块 #2（抓取角 148.6°）的事故就出在缺这一步：
    同一末端位姿有一支解给的是 wrist_2 = **+270.1°**，与 −89.9° 按 2π 折算
    距离为 0 —— 整链选支以为「腕不用动」，而 JTC 收到的是字面 +270.1°，
    于是腕整圈翻转：臂走到第 3 个插点（wrist_2=+64.4°）时夹爪朝天，
    人按了示教器 STOP。

    这里顺手守住软限位：折算后的值出界就不折（宁可让 `check_limits` 判红，
    也不把一个关节推到限位外）。返回新的数组，不改原值。

    **还要避开绕线**（2026-10-27 当天第二次事故）：腕 1 / 腕 3 是沿工具轴的
    两个「可整圈」关节，字面值离开 0 越远，夹爪线缆拧得越死。离 `ref` 最近
    的那一圈不一定是线松的那一圈 —— 回位那次就是挑了 wrist_1=−262.6°、
    wrist_3=−253.4°（离当时起点最近），把线又拧了约 285°，人只能手掰回来。
    所以腕 1 / 腕 3 额外受 `MAX_WRIST_TURN` 约束：最近的那圈超了就换一圈。
    """
    q = np.asarray(q, float)
    ref = np.asarray(ref, float)
    out = q.copy()
    for i, name in enumerate(chain.move_joints):
        lo, hi, _ = chain.limits.get(name, (-np.inf, np.inf, 0.0))
        k0 = int(round((ref[i] - q[i]) / (2 * np.pi)))
        opts = [q[i] + 2 * np.pi * k for k in (k0 - 1, k0, k0 + 1)]
        opts = [v for v in opts if lo <= v <= hi] or [q[i]]
        best = min(opts, key=lambda v: abs(v - ref[i]))
        if name in WOUND_JOINTS and abs(best) > MAX_WRIST_TURN:
            calmer = [v for v in opts if abs(v) <= MAX_WRIST_TURN]
            if calmer:
                best = min(calmer, key=abs)
        out[i] = best
    return out


def check_winding(chain, q, limit=MAX_WRIST_TURN):
    """腕 1 / 腕 3 的字面角离 0 太远 ⇒ 线缆在拧。返回 None 或 (关节名, 度)。"""
    for name in WOUND_JOINTS:
        try:
            i = chain.move_joints.index(name)
        except (ValueError, AttributeError):
            continue
        if abs(float(q[i])) > limit:
            return name, round(float(np.degrees(q[i])), 1)
    return None


def fold_yaw_180(deg):
    """抓取角折到 (−90°, +90°] —— **物理等价**，不是近似。

    平行两指夹爪（AG-95）绕工具轴转 180°，只是两根手指互换位置：夹的还是
    同一对侧面、合拢时夹爪照样闭合，块相对 tool0 的姿态也只差一个 180°
    （矩形块对 180° 自对称，所以放置结果逐字节相同）。但对腕 3 来说，
    这两个写法是**整整 180° 的不同行程**。

    2026-10-27 round-06 块 #4 就是被这一点拦下的：几何臂给出的抓取角是
    182.864°，腕 3 得先转到 +97.6° 抓（177.2°），再转回 −116.0° 放
    （213.6°）—— 后者越过 `MAX_JOINT_SWING`，整轮只剩 1 块。折成 +2.864°
    后，同一块的单关节最大跳变 **213.6° → 55.0°**，且不产生插点。
    """
    d = float(deg) % 180.0
    return d - 180.0 if d > 90.0 else d


def fold_yaw_90(deg):
    """抓取角折到 (−45°, +45°] —— **正方体**（四面对称）的物理等价写法。

    平行两指夹爪夹正方体，绕工具轴转 90° 只是换到**相邻的一对面**：边长一样、
    夹心一样、块相对夹爪的姿态只差 90°，而正方体对 90° 自对称 ⇒ 抓放结果相同。
    对腕 3 来说，这两个写法是整整 90° 的不同行程。

    与 `fold_yaw_180` 的分界：180° 只换手指、不换面，**矩形块**也成立；
    90° 换的是面，**只对正方体成立**。阶段 3 的块是 29.5 mm 方，阶段 2 不动。

    2026-11-01 入口⑤ 的回位就是被这一条挡死的：塔上那块 yaw 89.75°，只按 180°
    折就留在 89.75°，腕 3 从 −79.6° 起要么转 229.6°（超 180° 门槛）、要么转到
    −210.045°（超 ±210° 缠绕帽 0.045°）。按 90° 折成 −0.25° 后，同一趟回位
    10 个航点、最大单跳 42.4°、腕 3 峰值 135.6° —— 两个门槛都不用动。
    """
    d = float(deg) % 90.0
    return d - 90.0 if d > 45.0 else d


def ik_candidates(chain, xyz, yaw_deg=0.0, seeds=(), n_random=32,
                  max_iter=400, tol=1e-3):
    """同一个末端位姿的**多支**合格 IK 解（去重、去掉超限位与腕奇异的）。

    数值 IK 是局部收敛的：种子落在哪一支就收敛到哪一支。只要一支的话，
    2026-10-26 第 6 轮槽 2（y=−350 mm）那种目标就会落进肘朝下的分支，
    到 place 时前臂 z=−9.8 mm 被护栏拒绝 —— 而同一目标另有一支前臂 z=498 mm。

    随机种子用目标坐标算，**同输入必得同一组解**（测试要能钉住）。
    """
    T = target_matrix(xyz, yaw_deg)
    key = int(round((float(xyz[0]) + float(xyz[1]) + float(xyz[2])) * 1e5)) \
        % (2 ** 31)
    rng = np.random.default_rng(20261026 + key)
    all_seeds = [np.asarray(s, float) for s in seeds]
    all_seeds += [rng.uniform(-np.pi, np.pi, 6) for _ in range(n_random)]
    out = []
    for s in all_seeds:
        q, pe, re, _ = chain.ik(T, s, max_iter=max_iter)
        if pe > 0.1 or re > 0.05:
            continue
        if check_limits(chain, q) or check_wrist(chain, q):
            continue
        if any(joint_dist(q, u) < tol for u in out):
            continue
        out.append(q)
    # 判重用折算距离（上面），**选支/存盘用字面值**（这里）：
    # 全部候选先折到离起点最近的那一圈，DP 才能按真实行程比大小。
    ref = all_seeds[0] if all_seeds else np.zeros(len(chain.move_joints))
    folded = [unwrap_near(chain, q, ref) for q in out]
    # 折完要**重判一次**腕奇异：+350° 折成 −10° 之后才真正靠近奇异，
    # 折之前那一步是看不见的。
    return [q for q in folded if not check_wrist(chain, q)]


def solve_path(chain, targets, q_start, yaw_deg=0.0, table_z=TABLE_PLANE_Z,
               n_random=32, yaws=None, guard=None):
    """给一串末端目标选**一条前后一致的 IK 分支链**（最小总关节行程）。

    `targets` 是 [(名字, xyz), ...]。``yaws`` 可选，逐点姿态角；缺省全用
    ``yaw_deg``。阶段 2 需要「抓取角」和「放置角」不同（夹着块转腕），
    所以这里必须能逐点给 yaw —— 一个全局 yaw 表达不了两段姿态。
    """
    sets = [ik_candidates(chain, xyz,
                          yaw_deg if yaws is None else yaws[i],
                          seeds=[q_start], n_random=n_random)
            for i, (name, xyz) in enumerate(targets)]
    empty = [name for (name, _), s in zip(targets, sets) if not s]
    if empty:
        raise PlanError("这些航点连一支合格 IK 解都没有：%s" % "、".join(empty))

    def _guard(qa, qb):
        if guard is not None:
            return guard(qa, qb)
        return check_guard(chain, qa, qb, table_z)

    INF = float("inf")
    cost = [[INF] * len(s) for s in sets]
    prev = [[None] * len(s) for s in sets]
    for j, q in enumerate(sets[0]):
        ok, _ = _guard(q_start, q)
        if ok:
            cost[0][j] = travel_cost(q, q_start)
    for i in range(1, len(sets)):
        for j, q in enumerate(sets[i]):
            for k, qp in enumerate(sets[i - 1]):
                if not np.isfinite(cost[i - 1][k]):
                    continue
                ok, _ = _guard(qp, q)
                if not ok:
                    continue
                c = cost[i - 1][k] + travel_cost(q, qp)
                if c < cost[i][j]:
                    cost[i][j] = c
                    prev[i][j] = k
    ends = [(c, j) for j, c in enumerate(cost[-1]) if np.isfinite(c)]
    if not ends:
        raise PlanError("这些航点的所有 IK 分支组合都过不了高度护栏：%s"
                        % "、".join(name for name, _ in targets))
    _, j = min(ends)
    path = []
    for i in range(len(sets) - 1, -1, -1):
        path.append(sets[i][j])
        j = prev[i][j]
    path.reverse()
    return path


def add_move(waypoints, chain, name, xyz, q_seed, yaw_deg, table_z,
             duration=None, before=None, after=None, q=None, guard=None):
    """解一个航点并跑单点预检，追加到 waypoints。返回该点的 q。

    ``q`` 已经解好（来自 `solve_path`）时直接用，只复核残差与各项预检。
    """
    if q is None:
        q, pe, re, nfev = solve_waypoint(chain, xyz, yaw_deg, q_seed)
    else:
        q = np.asarray(q, float)
        pe, re = pose_residual(chain, q, xyz, yaw_deg)
        nfev = 0
    if pe > 0.1 or re > 0.05:
        raise PlanError("航点 %s IK 没解出来：位置残差 %.3f mm、姿态残差 %.4f° "
                        "(xyz=%s)" % (name, pe, re, xyz))
    bad = check_limits(chain, q)
    if bad:
        raise PlanError("航点 %s 超出软限位：%s" % (name, bad))
    ws = check_wrist(chain, q)
    if ws:
        raise PlanError("航点 %s 的腕 2 只有 %.2f°，离奇异位形不足 %.2f°（下限）"
                        % (name, ws[0], ws[1]))
    wd = check_winding(chain, q)
    if wd:
        raise PlanError("航点 %s 的 %s 在 %.1f°（上限 ±%.0f°）—— 这一支会把夹爪"
                        "线缆拧死；换一圈等价解（见 unwrap_near）"
                        % (name, wd[0], wd[1], float(np.degrees(MAX_WRIST_TURN))))
    if guard is None:
        ok, worst = check_guard(chain, q_seed, q, table_z)
    else:
        ok, worst = guard(q_seed, q)
    if not ok:
        raise PlanError("航点 %s 的段内高度护栏不过：t=%.2f 处 %s z=%.1f mm"
                        "（下限 %.1f mm）"
                        % (name, worst[0], worst[1], worst[2] * 1000,
                           worst[3] * 1000))
    wp = {"name": name, "q": [float(v) for v in q],
          "duration": float(duration or duration_for(q_seed, q))}
    if before:
        wp["before"] = before
    if after:
        wp["after"] = after
    waypoints.append(wp)
    return q, {"name": name, "ik_pos_mm": round(float(pe), 4),
               "ik_rot_deg": round(float(re), 5), "nfev": int(nfev),
               "elbow_min_z_mm": round(worst[2] * 1000, 1) if worst else None}


def split_jumps(waypoints, chain, max_jump=MAX_JOINT_JUMP, q_start=None):
    """相邻航点间关节跳变超过 max_jump 时，在关节空间插中间点。

    注意：JTC 本来就是在关节空间做直线插值，所以插点**不改变轨迹形状** ——
    它改变的是「这一条指令有多大」，便于日志与失败定位，也让 max_joint_jump
    这条约定在执行器的逐航点闸门里仍然有意义。

    ``q_start``：计划的起点关节角。给上它才对**第一跳**也判 ``MAX_JOINT_SWING``。
    此前这条链只比相邻航点，起点→第 1 个航点整个漏检 —— 2026-11-01 入口⑤ 第 6 次
    的回位计划里，wrist_3 就是从这里 +229.6° 起步却没被拦下（台账第一百八十七轮 §7）。
    **第一跳只判不插点**：插点只影响指令大小、不改轨迹，而这一跳的几何要跟既有
    计划的 JSON 保持一致（阶段 2 的产物不动）。
    """
    out = []
    added = 0
    prev_q = None if q_start is None else np.asarray(q_start, float)
    first = True
    for w in waypoints:
        q = np.asarray(w["q"], float)
        if prev_q is not None:
            jump = float(np.max(np.abs(q - prev_q)))
            if jump > MAX_JOINT_SWING:
                where = "起点 → 航点 %s" % w["name"] if first else "航点 %s" % w["name"]
                raise PlanError(
                    "%s 有单个关节要转 %.1f°（上限 %.0f°）—— 整圈翻转不是解法，"
                    "应换一支 IK 的等价角（见 unwrap_near）"
                    % (where, np.degrees(jump), np.degrees(MAX_JOINT_SWING)))
            # 端点都远离奇异，**插值点**不一定：q5 一过零就落在奇异上。
            # 第一跳虽然不插点（见上），中间点一样要查 —— 否则「不插点」会
            # 顺手把这条检查一起跳过（2026-11-01 首跳 wrist_2 ±89.8° 翻越就是这么漏的）。
            n = 8 if first else (int(np.ceil(jump / max_jump))
                                 if jump > max_jump else 0)
            for i in range(1, n):
                qi = prev_q + (q - prev_q) * (i / n)
                ws = check_wrist(chain, qi)
                if ws:
                    raise PlanError("插点 %s_split%d 的腕 2 只有 %.2f°，"
                                    "离奇异位形不足 %.2f°（下限）"
                                    % (w["name"], i, ws[0], ws[1]))
                if first:
                    continue
                out.append({"name": "%s_split%d" % (w["name"], i),
                            "q": [float(v) for v in qi],
                            "duration": float(duration_for(prev_q, qi))})
                added += 1
        out.append(w)
        prev_q = q
        first = False
    return out, added


def check_block_gap(block, blocks, min_gap=MIN_BLOCK_GAP):
    """抓一块时，最近的邻块中心距 —— 夹爪全开时手指要落得下。"""
    others = [b for b in blocks if b["idx"] != block["idx"]]
    if not others:
        return None
    return min(float(np.hypot(b["cx"] - block["cx"], b["cy"] - block["cy"]))
               for b in others) / 1000.0


# ---------------------------------------------------------------- 计划的三种形态
def build_survey(chain, q_start, which):
    xyz = SURVEY_TOOL0[which]
    wps, checks = [], []
    q, c = add_move(wps, chain, "survey_" + which, xyz, q_start, 0.0,
                    TABLE_PLANE_Z)
    checks.append(c)
    return wps, checks, {"kind": "survey", "which": which, "tool0": list(xyz)}


def build_move(chain, q_start, xyz, name="move"):
    wps, checks = [], []
    q, c = add_move(wps, chain, name, xyz, q_start, 0.0, TABLE_PLANE_Z)
    checks.append(c)
    return wps, checks, {"kind": "move", "tool0": list(xyz)}


def place_heights(block_h_m, grasp_depth_m, table_z=TABLE_PLANE_Z,
                  clearance=PLACE_CLEARANCE, gap=PLACE_APPROACH_GAP):
    """放置下降 / 放置接近两处的 tool0 z —— **按这一块挂多长算**，不按 A1 块高算。

    夹住之后块底在指尖以下 `block_h − grasp_depth`。要让**块底**离台面 `clearance`：

        指尖 z = 台面 + (block_h − grasp_depth) + clearance
        tool0 z = 指尖 z + 裸指尖 0.2042

    返回 (place_z, place_approach_z)。指尖若因此低于护栏下限，由 `add_move` 拦下，
    这里不自己判 —— 一处判一次。
    """
    below_tip = block_h_m - grasp_depth_m
    tip_z = table_z + below_tip + clearance
    place_z = tip_z + TOOL0_TO_BARE_FINGERTIP
    return place_z, place_z + gap


def stage3_carry_z(block_h_m, surface_z_m, grasp_gap=0.05):
    """阶段 3 的载运高度：**先高过塔顶放置接近点 50 mm，再横移**。

    原 `lift_z = 块高 + 0.1757 + 0.15` 按台面算；第 5 层时它低于
    `place_approach_z`，`to_slot` 会先在塔侧横移再上升 —— 正是持块包络
    最可能撞塔的路径。这里取 `max(lift_z, place_approach_z + grasp_gap)`。
    """
    wz = float(block_h_m)
    grasp_z = float(np.clip(wz + TOOL0_TO_FINGERTIP + GRASP_CLEARANCE,
                            GRASP_Z_MIN, GRASP_Z_MAX))
    grasp_depth = wz - (grasp_z - TOOL0_TO_BARE_FINGERTIP)
    _, place_approach = place_heights(wz, grasp_depth,
                                      table_z=float(surface_z_m))
    lift_z = wz + TOOL0_TO_FINGERTIP + LIFT_HEIGHT
    return max(lift_z, place_approach + float(grasp_gap))


def _open_action(mm):
    if mm is None:
        return {"gripper": "open"}
    return {"gripper": "open_to", "mm": float(mm)}


def _with_speed(action, speed):
    if action is None or speed is None:
        return action
    a = dict(action)
    a["speed"] = int(speed)
    return a


def build_pick(chain, q_start, cand, block, blocks, slot,
               yaw_grasp_deg=0.0, yaw_place_deg=None, yaw_source="zero",
               pick_open_mm=None, place_open_mm=None, grasp_at="candidate",
               grip_speed_pct=None, release_speed_pct=None,
               release_dwell_s=0.0, surface_z=None, carry_z=None,
               survey_tool0=None, tower_box=None, held_half_edge=0.020,
               held_off_xy=None, place_clearance=None):
    """完整拾放循环：approach → grasp(闭合) → lift → to_slot → place_approach
    → place(张开) → retreat → survey_place。

    两段 yaw（阶段 2 需要，A3 默认都是 0）
    ─────────────────────────────────────
    · ``yaw_grasp_deg``：approach/grasp/lift 的 tool0 绕 z 角。几何臂取
      「块边方向 + δ」让两指贴住块面；
    · ``yaw_place_deg``：to_slot/place_approach/place/retreat 的角。夹住块后
      块相对 tool0 的偏角固定，所以放平时 ``yaw_place = δ``，**不随块而变**。
    一个全局 yaw 表达不了「先按块面抓、再把手里这块转到格线」；这里逐航点给。

    ``pick_open_mm`` / ``place_open_mm``：接近/放置时用半开（如 50 mm），
    避免全开 95 mm 的指尖伸进 60 mm 格距的邻格。``None`` = 全开（A3 行为）。

    松手弹滑的四项处置（2026-10-27 用户定，5 mm 放置间隙**不动**）
    ─────────────────────────────────────────────────────────────
    · ``grasp_at``：``candidate``（A3 老行为，抓 GraspNet/几何候选给的锚点）
      或 ``block_center``（抓块心）。块心更对称，合拢时块两侧受力均匀，松手
      后转动/滑移更小 —— 这是 B2「抓取/松手时块被转」最便宜的解法。
    · ``grip_speed_pct`` / ``release_speed_pct``：AG-95 速度寄存器 0x0104
      （1–100 %），分别挂在「合拢」与「松手张开」两个动作上。默认 ``None``
      = 不写（A3 计划逐字节不变）。
    · ``release_dwell_s``：松手后**先停这么久再撤**，让块自己落稳。默认 0
      （A3 不变）。

    抓取角先折 180°（`fold_yaw_180`）——见该函数的说明，这是**物理等价**，
    不是近似。

    阶段 3（堆叠）的可选参数（默认全部不改变阶段 2 的计划）
    ────────────────────────────────────────────────────
    · ``surface_z``：放置表面高度（默认 `TABLE_PLANE_Z`）。第 1 层传垫面
      （`grid.json` 的 `T_base_pad[2,3]`），第 k≥2 层传上一层实测顶面。
    · ``carry_z``：载运高度。实际用 ``max(lift_z, carry_z)``——原 `lift_z`
      按台面算，第 5 层时低于 `place_approach_z`，会先横移再上升。
    · ``survey_tool0``：收尾巡检位（默认 `SURVEY_TOOL0["place"]`）。阶段 3 传
      高度跟踪位姿对里的第一个。
    · ``tower_box``：``(x0,x1,y0,y1,z0,z1)``（m，base 系）。给了就开**持块包络
      撞塔**护栏：`check_guard` 之外再查夹着的块有没有进箱。
    · ``held_half_edge`` / ``held_off_xy``：持块包络的半边长（块半宽 + 余量）
      与块心相对 tool0 的 xy 偏置（已按放置 yaw 旋转）。
    · ``place_clearance``：放置那一刻**块底离表面**的间隙（m）。默认
      `PLACE_CLEARANCE` = 5 mm，阶段 2 逐字节不变。入口实测（2026-11-01）：
      5 mm 时块是**自由落体砸在垫子上**（五次 Δz 极差 0.15 mm，落 3.8 mm），
      落定后的 xy/yaw 散乱；3/4 mm 是「放下去」而不是「掉下去」的对照。
    """
    yaw_grasp_raw = float(yaw_grasp_deg)
    yaw_grasp_deg = fold_yaw_180(yaw_grasp_deg)
    yaw_place = yaw_grasp_deg if yaw_place_deg is None else yaw_place_deg

    if grasp_at == "block_center":
        # 块心 = 顶面点聚类质心（与放置目标/判分同一个量），单位 mm → m。
        gx, gy = block["cx"] / 1000.0, block["cy"] / 1000.0
    elif grasp_at == "candidate":
        gx, gy = cand["xyz_base"][0], cand["xyz_base"][1]
    else:
        raise PlanError("grasp_at 只认 candidate / block_center，收到 %r" % (grasp_at,))

    # 用**相对台面的高度** `h`，不用绝对顶面 z：
    # 腕部深度实测把台面读高 ~6 mm（本轮帧 table_z=+6.0 mm，真块高 = top − table_z
    # = 24.3~26.0 mm），而本规划器的整个几何都锚在 table_plane_z=0（抓取高度、
    # 放置高度都按台面建）。拿绝对 top 当 wz 会把那 6 mm 原样带进抓取高度，
    # 指尖就少抓 6 mm —— 正好是用户说的「抓得更浅」那一类误差。
    wz = (block["h"] if block.get("h") is not None else block["top"]) / 1000.0
    approach_z = wz + TOOL0_TO_FINGERTIP + APPROACH_CLEARANCE
    grasp_z = float(np.clip(wz + TOOL0_TO_FINGERTIP + GRASP_CLEARANCE,
                            GRASP_Z_MIN, GRASP_Z_MAX))
    lift_z = wz + TOOL0_TO_FINGERTIP + LIFT_HEIGHT
    # 阶段 3：载运高度必须先高过塔顶的放置接近点，否则 to_slot 会在塔侧横移。
    lift_used = lift_z if carry_z is None else max(lift_z, float(carry_z))
    # 放置高度按**这一块**挂多长算（旧代码写死 0.2142，等于把 A1 的 28.5 mm 块高
    # 编进常量里；29.6 mm 的块会压进台面 1.1 mm）。
    grasp_depth_m = wz - (grasp_z - TOOL0_TO_BARE_FINGERTIP)
    table_z = TABLE_PLANE_Z if surface_z is None else float(surface_z)
    clearance_m = (PLACE_CLEARANCE if place_clearance is None
                   else float(place_clearance))
    place_z, place_approach_z = place_heights(wz, grasp_depth_m,
                                              table_z=table_z,
                                              clearance=clearance_m)
    # 抓取偏心（块心估计 − 抓点）。默认**不**拿它补放置目标：第 6 轮实测
    # 这块实际只偏 2.76 mm，补了反而多偏 4.3 mm（详见 PLACE_COMPENSATE_ECC）。
    # 量出来写进 meta，下一轮可以直接对账「夹持偏移 vs 抓取偏心」。
    ex, ey = block["cx"] / 1000.0 - gx, block["cy"] / 1000.0 - gy
    if held_off_xy is None:
        # 持块包络要用「放置那一刻」块心相对 tool0 的偏置：抓取时的 (ex, ey)
        # 随夹着块转腕一起转，转角 = yaw_place − yaw_grasp。
        dyaw = np.radians(float(yaw_place) - float(yaw_grasp_deg))
        held_off_xy = (ex * np.cos(dyaw) - ey * np.sin(dyaw),
                       ex * np.sin(dyaw) + ey * np.cos(dyaw))
    if PLACE_COMPENSATE_ECC:
        sx, sy = slot[0] - ex, slot[1] - ey
    else:
        sx, sy = slot

    gap = check_block_gap(block, blocks)
    if gap is not None and gap < MIN_BLOCK_GAP:
        raise PlanError("块 #%d 离最近邻块只有 %.1f mm（下限 %.0f mm）—— "
                        "夹爪全开 95 mm 会碰邻块" % (block["idx"], gap * 1000,
                                                   MIN_BLOCK_GAP * 1000))
    if cand["dist_mm"] > 12.0:
        raise PlanError("候选离块心 %.1f mm，超过 12 mm 锚定半径" % cand["dist_mm"])

    # 阶段 3：夹着的块也要算碰撞（原 check_guard 只查肘/腕/裸指尖）。
    guard = None
    if tower_box is not None:
        # 块底相对 tool0 的距离：裸指尖 204.2 mm + 夹持深度。
        # 少了裸指尖这一项，块会被当成贴在法兰下，塔体护栏等于没开。
        below_tip = wz - grasp_depth_m + TOOL0_TO_BARE_FINGERTIP

        def guard(qa, qb, _below=below_tip):
            ok, worst = check_guard(chain, qa, qb, TABLE_PLANE_Z)
            if not ok:
                return False, worst
            ok2, w2 = check_held_block(chain, qa, qb, held_half_edge, wz,
                                       _below, tower_box, held_off_xy)
            if not ok2:
                return False, w2
            return True, worst

    survey_xyz = (SURVEY_TOOL0["place"] if survey_tool0 is None
                  else tuple(survey_tool0))
    wps, checks = [], []
    specs = [
        ("approach", (gx, gy, approach_z), yaw_grasp_deg, None,
         _open_action(pick_open_mm)),
        ("grasp", (gx, gy, grasp_z), yaw_grasp_deg,
         _with_speed({"gripper": "close", "readback": True},
                     grip_speed_pct), None),
        ("lift", (gx, gy, lift_used), yaw_grasp_deg, {"gripper": "readback"},
         None),
        ("to_slot", (sx, sy, lift_used), yaw_place, None, None),
        ("place_approach", (sx, sy, place_approach_z), yaw_place, None, None),
        ("place", (sx, sy, place_z), yaw_place,
         _with_speed(_open_action(place_open_mm), release_speed_pct), None),
        ("retreat", (sx, sy, place_approach_z), yaw_place, None, None),
        ("survey_place", survey_xyz, 0.0, None, None),
    ]
    # 整链选一条分支（见 solve_path 的注释：逐点就近会挑到下一段过不了护栏的解）
    qpath = solve_path(chain, [(n, xyz) for n, xyz, _, _, _ in specs], q_start,
                       yaws=[y for _, _, y, _, _ in specs], guard=guard)
    q = q_start
    for (name, xyz, yaw, after, before), q_solved in zip(specs, qpath):
        q, c = add_move(wps, chain, name, xyz, q, yaw, TABLE_PLANE_Z,
                        after=after, before=before, q=q_solved, guard=guard)
        checks.append(c)
    # 松手后停在原地等块落稳，再撤（`retreat` 是下一条）。挂在 `place` 上而不是
    # 全局 `--settle`：`--settle` 每条航点都睡，那是时间，不是物理动作。
    if release_dwell_s:
        for w in wps:
            if w["name"] == "place":
                w["dwell_s"] = round(float(release_dwell_s), 3)

    meta = {"kind": "pick", "block_idx": block["idx"], "class": block["class"],
            "block_top_mm": round(block["top"], 2),
            "block_h_mm": round(wz * 1000, 2),
            "wz_measured_mm": round(block["top"], 2),
            "wz_used_mm": round(wz * 1000, 2),
            "table_z_mm": round(block["top"] - wz * 1000, 2),
            "grasp_depth_mm": round(wz * 1000 - (grasp_z * 1000
                                                 - TOOL0_TO_BARE_FINGERTIP * 1000), 2),
            "block_below_tip_mm": round((wz - grasp_depth_m) * 1000, 2),
            "place_z": round(place_z, 4),
            "place_approach_z": round(place_approach_z, 4),
            "place_clearance_mm": round(clearance_m * 1000, 2),
            "block_xy_mm": [round(block["cx"], 2), round(block["cy"], 2)],
            "grasp_point": [gx, gy], "grasp_ecc_mm": [round(ex * 1000, 2),
                                                     round(ey * 1000, 2)],
            "grasp_at": grasp_at,
            "grip_speed_pct": grip_speed_pct,
            "release_speed_pct": release_speed_pct,
            "release_dwell_s": (round(float(release_dwell_s), 3)
                                if release_dwell_s else 0.0),
            "place_target_mm": [round(sx * 1000, 2), round(sy * 1000, 2)],
            "grasp_z": round(grasp_z, 4),
            "approach_z": round(approach_z, 4), "lift_z": round(lift_z, 4),
            "carry_z": round(lift_used, 4),
            "surface_z_mm": round(table_z * 1000.0, 2),
            "tower_box": ([round(float(v), 4) for v in tower_box]
                          if tower_box is not None else None),
            "survey_tool0": [round(float(v), 4) for v in survey_xyz],
            "cand_score": round(cand["score"], 4),
            "cand_width_mm": round(cand["width_mm"], 2),
            "cand_dist_mm": round(cand["dist_mm"], 2),
            "cand_angle_deg": round(cand["angle_deg"], 2),
            "cand_seed": cand.get("seed"),
            "yaw_grasp_deg": round(float(yaw_grasp_deg), 3),
            "yaw_grasp_raw_deg": round(yaw_grasp_raw, 3),
            "yaw_place_deg": round(float(yaw_place), 3),
            "yaw_source": yaw_source,
            "pick_open_mm": pick_open_mm, "place_open_mm": place_open_mm,
            "slot": list(slot), "block_gap_mm": round(gap * 1000, 1) if gap else None}
    return wps, checks, meta


def tower_box_from(tower_xy_mm, tower_top_mm, tower_half_edge=0.020):
    """上一层实测塔顶（mm）→ 塔体 box ``(x0, x1, y0, y1, z0, z1)``（m）。

    两个规划入口（`build_stack` / `build_place`）必须用**同一个** box ——
    差一个余量就是一个多判、一个漏判。缺任一项就返回 ``None``（护栏不开，
    第 1 层本来就没有塔）。
    """
    if tower_xy_mm is None or tower_top_mm is None:
        return None
    cx = float(tower_xy_mm[0]) / 1000.0
    cy = float(tower_xy_mm[1]) / 1000.0
    half = float(tower_half_edge) + TOWER_BOX_MARGIN
    return (cx - half, cx + half, cy - half, cy + half,
            0.0, float(tower_top_mm) / 1000.0)


def build_stack(chain, q_start, cand, block, blocks, slot, surface_z,
                tower_xy_mm=None, tower_top_mm=None, survey_tool0=None,
                carry_z=None, tower_half_edge=0.020, **kwargs):
    """阶段 3 的一层：抓一块 → 放到塔顶。`build_pick` 的阶段 3 封装。

    ``surface_z``：本层放置表面（m，base 系）。第 1 层 = 垫面，第 k≥2 层 =
    上一层实测顶面（已过 z 偏差校正）。

    ``tower_xy_mm`` / ``tower_top_mm``：**上一层（本层放置前）实测**的塔顶中心
    与顶高（mm）。给了就构造塔体 box、开「持块包络撞塔」护栏；第 1 层传 None。

    ``survey_tool0``：收尾巡检位（通常取 `stage3_survey_tool0(...)[0]`）；
    其余参数透传 `build_pick`。
    """
    tower_box = tower_box_from(tower_xy_mm, tower_top_mm, tower_half_edge)
    wps, checks, meta = build_pick(
        chain, q_start, cand, block, blocks, slot,
        surface_z=surface_z, carry_z=carry_z, survey_tool0=survey_tool0,
        tower_box=tower_box, held_half_edge=tower_half_edge, **kwargs)
    meta["kind"] = "stack"
    meta["layer_surface_mm"] = round(float(surface_z) * 1000.0, 2)
    return wps, checks, meta


def build_place(chain, q_start, block, slot, surface_z, carry_z,
                yaw_place_deg=0.0, yaw_grasp_deg=0.0, open_mm=None,
                survey_tool0=None, tower_box=None, held_half_edge=0.020,
                held_off_xy=None, release_dwell_s=0.0, release_speed_pct=None,
                place_clearance=None):
    """**放置后半段**：已夹着块停在 carry 高度 → to_slot → place_approach
    → place → retreat → survey_place。

    用途只有一个：eye-in-hand 组在 carry 处抓一帧、量一次塔顶，把目标改掉之后
    重规划这一半。整条 `build_pick` 从抓取开始重解是错的 —— 块已经在手里了，
    起点的关节角也不是原来那个（阶段 2 的教训：多段计划共用 q_start 会被执行器
    闸门拒掉）。

    与 `build_pick` 的后半段**同源同式**：同一个 `place_heights`、同样的
    `check_guard` + `check_held_block`、同样的松手 dwell 与张开速度。区别只有
    起点（当次实测关节角）与 `slot`（改过的目标）。

    ``carry_z`` **必给**，而且就是当前 tool0 的高度：第一条航点 `to_slot` 让臂
    **先在这个高度横移到新目标上方，再垂直下降**。少了这一步，改过的目标会让臂
    斜着从塔侧切下去 —— 持块包络正好扫过塔顶（`stage3_carry_z` 的注释讲的
    是同一件事）。

    ``block`` 与 `build_pick` 同形：``cx``/``cy``（mm）、``h``（mm，块高，
    由调用方换成扣过拾取侧偏差的 ``block_h_used_mm``）。
    """
    wz = (block["h"] if block.get("h") is not None else block["top"]) / 1000.0
    table_z = TABLE_PLANE_Z if surface_z is None else float(surface_z)
    clearance_m = (PLACE_CLEARANCE if place_clearance is None
                   else float(place_clearance))
    # 夹持深度必须与抓取时同源：同一 wz ⇒ 同一 grasp_z ⇒ 同一深度。
    # 混用会让块高偏差以 (block_h − wz) 叠进放置高度（契约 §2 的技术事实）。
    grasp_z = float(np.clip(wz + TOOL0_TO_FINGERTIP + GRASP_CLEARANCE,
                            GRASP_Z_MIN, GRASP_Z_MAX))
    grasp_depth_m = wz - (grasp_z - TOOL0_TO_BARE_FINGERTIP)
    place_z, place_approach_z = place_heights(wz, grasp_depth_m,
                                              table_z=table_z,
                                              clearance=clearance_m)

    yaw_grasp_deg = fold_yaw_180(yaw_grasp_deg)
    yaw_place = yaw_grasp_deg if yaw_place_deg is None else yaw_place_deg
    sx, sy = float(slot[0]), float(slot[1])
    lift_used = float(carry_z)
    if lift_used < place_approach_z - 1e-9:
        raise PlanError(
            "carry_z %.1f mm 低于本层 place_approach %.1f mm —— 从塔侧直接切"
            "下降会撞塔" % (lift_used * 1000, place_approach_z * 1000))

    if held_off_xy is None:
        # 块心相对 tool0 的偏置（m）。抓点就是块心时是 (0,0)；抓点不是块心时
        # **必须由调用方算好传进来**（`place_plan` 从上一段 meta 的
        # `grasp_ecc_mm` 按 Δyaw 旋转得到）—— 这里不自己猜。
        held_off_xy = (0.0, 0.0)

    guard = None
    if tower_box is not None:
        below_tip = wz - grasp_depth_m + TOOL0_TO_BARE_FINGERTIP

        def guard(qa, qb, _below=below_tip):
            ok, worst = check_guard(chain, qa, qb, TABLE_PLANE_Z)
            if not ok:
                return False, worst
            ok2, w2 = check_held_block(chain, qa, qb, held_half_edge, wz,
                                       _below, tower_box, held_off_xy)
            if not ok2:
                return False, w2
            return True, worst

    survey_xyz = (SURVEY_TOOL0["place"] if survey_tool0 is None
                  else tuple(survey_tool0))
    specs = [
        ("to_slot", (sx, sy, lift_used), yaw_place, None, None),
        ("place_approach", (sx, sy, place_approach_z), yaw_place, None, None),
        ("place", (sx, sy, place_z), yaw_place,
         _with_speed(_open_action(open_mm), release_speed_pct), None),
        ("retreat", (sx, sy, place_approach_z), yaw_place, None, None),
        ("survey_place", survey_xyz, 0.0, None, None),
    ]
    qpath = solve_path(chain, [(n, xyz) for n, xyz, _, _, _ in specs], q_start,
                       yaws=[y for _, _, y, _, _ in specs], guard=guard)
    wps, checks = [], []
    q = q_start
    for (name, xyz, yaw, after, before), q_solved in zip(specs, qpath):
        q, c = add_move(wps, chain, name, xyz, q, yaw, TABLE_PLANE_Z,
                        after=after, before=before, q=q_solved, guard=guard)
        checks.append(c)
    if release_dwell_s:
        for w in wps:
            if w["name"] == "place":
                w["dwell_s"] = round(float(release_dwell_s), 3)
    meta = {"kind": "place", "block_idx": block.get("idx"),
            "block_h_mm": round(wz * 1000, 2),
            "grasp_depth_mm": round(grasp_depth_m * 1000, 2),
            "block_below_tip_mm": round((wz - grasp_depth_m) * 1000, 2),
            "place_z": round(place_z, 4),
            "place_approach_z": round(place_approach_z, 4),
            "place_clearance_mm": round(clearance_m * 1000, 2),
            "carry_z": round(lift_used, 4),
            "place_target_mm": [round(sx * 1000, 2), round(sy * 1000, 2)],
            "surface_z_mm": round(table_z * 1000.0, 2),
            "tower_box": ([round(float(v), 4) for v in tower_box]
                          if tower_box is not None else None),
            "survey_tool0": [round(float(v), 4) for v in survey_xyz],
            "yaw_grasp_deg": round(float(yaw_grasp_deg), 3),
            "yaw_place_deg": round(float(yaw_place), 3),
            "place_open_mm": open_mm,
            "release_speed_pct": release_speed_pct,
            "release_dwell_s": round(float(release_dwell_s or 0.0), 3)}
    return wps, checks, meta


# ---------------------------------------------------------------- 放置后矫正
#: 单次矫正允许的最大平移（mm）与最大转角（°）。超出的错误**不是**「放歪了」，
#: 是「根本没放对」，该走完整抓放；拿 nudge 硬拽只会把块拖到邻块上去。
NUDGE_MAX_SHIFT_MM = 15.0
NUDGE_MAX_YAW_DEG = 15.0


def build_nudge(chain, q_start, block, shift_xy_mm=(0.0, 0.0), dyaw_deg=0.0,
                surface_z=TABLE_PLANE_Z, yaw_block_deg=0.0, blocks=None,
                open_mm=None, survey_tool0=None, tower_box=None,
                held_half_edge=0.020):
    """放置后矫正：**只夹不抬**，把已经放下的块平移 δxy / 原地转 δyaw。

    为什么有这个原语（2026-11-01 用户提出，第一百八十五轮记账）
    ────────────────────────────────────────────────────────────
    入口② 15 次实测：Δxy σ 1.14 mm，Δyaw（彩色）σ 5.14°、6/15 超契约的 5° 门。
    yaw 那部分经两套**数据来源不同**的估计器对撞确认（r=0.897、11/15 同号）
    是**真实的释放转动**，不是测量噪声；调参调不掉，只能加动作去改。

    这招的天然优势：δ = 目标 − 本次实测，测量与校正在**同一趟、同一套腕部
    相机**里完成，静态外参偏差在相减时抵消。它不要求外参绝对准，只要求同趟
    一致 —— 比开环放置依赖 `grid.json` 绝对坐标强一档。

    动作序列
    ────────
    ``nudge_approach → nudge_grip → nudge_shift → nudge_twist → nudge_release
    → nudge_retreat → nudge_survey``

    · tool0 的 xy 先对齐**实测块心**：绕 tool0 的 z 转就是绕块心那条竖线转，
      块心不跟着跑（真滑移由松手后的复测发现，这里不假装能保证）。
    · `grip`/`shift`/`twist`/`release` **同高**：块始终贴着自己那一层，不抬。
    · 夹持深度沿用抓取那套算式（`GRASP_CLEARANCE`），够传力矩又不压块。

    会拒的情况（fail-closed，不写计划）
    ──────────────────────────────────
    · |δxy| > `NUDGE_MAX_SHIFT_MM` 或 |δyaw| > `NUDGE_MAX_YAW_DEG`；
    · 给了 ``blocks`` 时，离最近邻块 < `MIN_BLOCK_GAP`（张开的两指会碰邻块）；
    · 任一航点过不了 `check_guard`（裸指尖离台面）；
    · 给了 ``tower_box`` 还要过 `check_held_block`。**注意 box 必须把被矫正的
      这一块排除在外**（box 顶取被矫正块的底面），否则必然误报 —— 块本来就在箱里。

    参数里的 ``block`` 与 `build_pick` 同形：``cx``/``cy``（mm）、``h``（mm，
    相对它脚下那层表面）；``surface_z`` 是块脚下那层的 z（m，base 系）。
    """
    dx_mm, dy_mm = float(shift_xy_mm[0]), float(shift_xy_mm[1])
    shift_mm = float(np.hypot(dx_mm, dy_mm))
    dyaw_deg = float(dyaw_deg)
    if shift_mm > NUDGE_MAX_SHIFT_MM + 1e-9:
        raise PlanError("矫正平移 %.1f mm 超过 %.0f mm 上限 —— 这不是「放歪」"
                        "而是「没放对」，该走完整抓放"
                        % (shift_mm, NUDGE_MAX_SHIFT_MM))
    if abs(dyaw_deg) > NUDGE_MAX_YAW_DEG + 1e-9:
        raise PlanError("矫正转角 %.1f° 超过 %.0f° 上限（同上，别硬拽）"
                        % (dyaw_deg, NUDGE_MAX_YAW_DEG))

    wz = (block["h"] if block.get("h") is not None else block["top"]) / 1000.0
    surf = float(surface_z)
    top_abs = surf + wz
    bx, by = block["cx"] / 1000.0, block["cy"] / 1000.0
    yaw0 = fold_yaw_180(yaw_block_deg)
    yaw1 = fold_yaw_180(yaw_block_deg + dyaw_deg)

    # 高度算式与 `build_pick` 完全同源：先在「相对台面」的坐标里解出本地
    # tool0 z（软限位那对常量也是按台面平面标定的），再整体抬到 surf。
    approach_z = surf + wz + TOOL0_TO_FINGERTIP + APPROACH_CLEARANCE
    grip_z_local = float(np.clip(wz + TOOL0_TO_FINGERTIP + GRASP_CLEARANCE,
                                 GRASP_Z_MIN, GRASP_Z_MAX))
    grip_z = surf + grip_z_local
    # 张开宽度：默认块宽 + 10 mm（够跨进两侧，又不至于把行程撑到邻块）。
    open_used = float(open_mm) if open_mm is not None else 40.0

    if blocks:
        gap = check_block_gap(block, blocks)
        if gap is not None and gap < MIN_BLOCK_GAP:
            raise PlanError("块 #%d 离最近邻块只有 %.1f mm（下限 %.0f mm）—— "
                            "矫正时张开的两指会碰邻块"
                            % (block["idx"], gap * 1000, MIN_BLOCK_GAP * 1000))

    guard = None
    if tower_box is not None:
        grasp_depth_m = wz - (grip_z_local - TOOL0_TO_BARE_FINGERTIP)
        below_tip = wz - grasp_depth_m + TOOL0_TO_BARE_FINGERTIP

        def guard(qa, qb, _below=below_tip):
            ok, worst = check_guard(chain, qa, qb, TABLE_PLANE_Z)
            if not ok:
                return False, worst
            ok2, w2 = check_held_block(chain, qa, qb, held_half_edge, wz,
                                       _below, tower_box, None)
            if not ok2:
                return False, w2
            return True, worst

    survey_xyz = (SURVEY_TOOL0["place"] if survey_tool0 is None
                  else tuple(survey_tool0))

    def _open(mm):
        return {"gripper": "open"} if mm is None else {"gripper": "open_to",
                                                       "mm": float(mm)}

    tx, ty = bx + dx_mm / 1000.0, by + dy_mm / 1000.0
    specs = [
        ("nudge_approach", (bx, by, approach_z), yaw0, None, _open(open_used)),
        ("nudge_grip", (bx, by, grip_z), yaw0,
         {"gripper": "close", "readback": True}, None),
        ("nudge_shift", (tx, ty, grip_z), yaw0,
         {"gripper": "readback"}, None),
        ("nudge_twist", (tx, ty, grip_z), yaw1,
         {"gripper": "readback"}, None),
        ("nudge_release", (tx, ty, grip_z), yaw1, _open(open_used), None),
        ("nudge_retreat", (tx, ty, approach_z), yaw1, None, None),
        ("nudge_survey", survey_xyz, 0.0, None, None),
    ]
    qpath = solve_path(chain, [(n, xyz) for n, xyz, _, _, _ in specs], q_start,
                       yaws=[y for _, _, y, _, _ in specs], guard=guard)
    wps, checks = [], []
    q = q_start
    for (name, xyz, yaw, after, before), q_solved in zip(specs, qpath):
        q, c = add_move(wps, chain, name, xyz, q, yaw, TABLE_PLANE_Z,
                        after=after, before=before, q=q_solved, guard=guard)
        checks.append(c)

    meta = {"kind": "nudge", "block_idx": block.get("idx"),
            "surface_z_mm": round(surf * 1000.0, 2),
            "block_top_abs_mm": round(top_abs * 1000.0, 2),
            "block_h_mm": round(wz * 1000.0, 2),
            "block_xy_mm": [round(block["cx"], 2), round(block["cy"], 2)],
            "shift_xy_mm": [round(dx_mm, 3), round(dy_mm, 3)],
            "shift_norm_mm": round(shift_mm, 3),
            "dyaw_deg": round(dyaw_deg, 3),
            "yaw_from_deg": round(yaw0, 3), "yaw_to_deg": round(yaw1, 3),
            "target_xy_mm": [round(tx * 1000.0, 2), round(ty * 1000.0, 2)],
            "grip_z": round(grip_z, 4), "approach_z": round(approach_z, 4),
            "open_mm": round(open_used, 1),
            "tower_box": ([round(float(v), 4) for v in tower_box]
                          if tower_box is not None else None),
            "survey_tool0": [round(float(v), 4) for v in survey_xyz]}
    return wps, checks, meta


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--from-frame", default=None,
                    help="从帧目录的 joint_states.json 取 q_start")
    ap.add_argument("--q-start", default=None, help="逗号分隔的 6 个关节角（弧度）")
    ap.add_argument("--out", required=True)
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--survey", choices=sorted(SURVEY_TOOL0))
    grp.add_argument("--move", help="tool0 目标 x,y,z（米）")
    grp.add_argument("--pick", action="store_true")
    ap.add_argument("--candidates", default=None)
    ap.add_argument("--block", type=int, default=None)
    ap.add_argument("--slot", default=None, help="x,y（米）")
    ap.add_argument("--yaw-grasp-deg", type=float, default=0.0,
                    help="抓取段 tool0 绕 z（阶段 2；A3 保持 0）")
    ap.add_argument("--yaw-place-deg", type=float, default=None,
                    help="放置段 tool0 绕 z（缺省 = 抓取角）")
    ap.add_argument("--yaw-source", default="zero")
    ap.add_argument("--pick-open-mm", type=float, default=None,
                    help="接近时半开开口；缺省全开 95")
    ap.add_argument("--place-open-mm", type=float, default=None,
                    help="放置时半开开口；缺省全开 95")
    ap.add_argument("--grasp-at", choices=("candidate", "block_center"),
                    default="candidate",
                    help="抓取点用候选锚点（A3）还是块心（阶段 2 默认块心）")
    ap.add_argument("--grip-speed-pct", type=int, default=None,
                    help="合拢时的 AG-95 速度（0x0104，1–100 %）；缺省不写")
    ap.add_argument("--release-speed-pct", type=int, default=None,
                    help="松手张开时的 AG-95 速度（0x0104，1–100 %）；缺省不写")
    ap.add_argument("--release-dwell-s", type=float, default=0.0,
                    help="松手后停多少秒再撤（让块落稳）；缺省 0")
    args = ap.parse_args()

    if args.from_frame:
        js = json.load(open(os.path.join(args.from_frame, "joint_states.json")))
        chain = urkin.Chain(args.urdf)
        q_start = [float(js["position"][js["name"].index(n)]) for n in chain.move_joints]
    elif args.q_start:
        chain = urkin.Chain(args.urdf)
        q_start = [float(v) for v in args.q_start.split(",")]
    else:
        ap.error("要给 --from-frame 或 --q-start")

    try:
        if args.survey:
            wps, checks, meta = build_survey(chain, q_start, args.survey)
        elif args.move:
            xyz = tuple(float(v) for v in args.move.split(","))
            wps, checks, meta = build_move(chain, q_start, xyz)
        else:
            if not (args.candidates and args.slot) or args.block is None:
                ap.error("--pick 要同时给 --candidates / --block / --slot")
            cand = json.load(open(args.candidates))
            per = [p for p in cand["per_block"] if p["block_idx"] == args.block]
            if not per or not per[0]["best"]:
                raise PlanError("块 #%d 没有 ≤%.0f mm 的候选"
                                % (args.block, cand["anchor_mm"]))
            block = next(b for b in cand["blocks"] if b["idx"] == args.block)
            slot = tuple(float(v) for v in args.slot.split(","))
            wps, checks, meta = build_pick(
                chain, q_start, per[0]["best"], block, cand["blocks"], slot,
                yaw_grasp_deg=args.yaw_grasp_deg,
                yaw_place_deg=args.yaw_place_deg,
                yaw_source=args.yaw_source,
                pick_open_mm=args.pick_open_mm,
                place_open_mm=args.place_open_mm,
                grasp_at=args.grasp_at,
                grip_speed_pct=args.grip_speed_pct,
                release_speed_pct=args.release_speed_pct,
                release_dwell_s=args.release_dwell_s)
        n_before_split = len(wps)
        wps, added = split_jumps(wps, chain, q_start=q_start)
        # 每个**原始**航点都必须有一条预检记录（插出来的点继承相邻两点的检查）。
        if len(checks) != n_before_split:
            raise PlanError("预检记录不完整：%d 条航点只有 %d 条记录"
                            % (n_before_split, len(checks)))
    except PlanError as e:
        print("!! 预检不过：%s" % e, file=sys.stderr)
        print("   计划**没有**写出 —— 不要执行。", file=sys.stderr)
        return 2

    plan = {"version": 2, "meta": dict(meta, checks=checks, splits_added=added,
                                       q_start_deg=[round(float(np.degrees(v)), 2)
                                                    for v in q_start]),
            "q_start": [float(v) for v in q_start], "waypoints": wps}
    json.dump(plan, open(args.out, "w"), ensure_ascii=False, indent=1)

    print("计划 → %s（%d 条航点，插点 %d）" % (args.out, len(wps), added))
    print("  %-14s %-38s %6s %9s %9s" % ("航点", "q (deg)", "时长s", "IK mm", "肘高mm"))
    by_name = {c["name"]: c for c in checks}
    for w in wps:
        c = by_name.get(w["name"], {})
        print("  %-14s %-38s %6.1f %9.3f %9.1f"
              % (w["name"],
                 "[" + " ".join("%6.1f" % np.degrees(v) for v in w["q"]) + "]",
                 w["duration"], c.get("ik_pos_mm", float("nan")),
                 c.get("elbow_min_z_mm") or float("nan")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
