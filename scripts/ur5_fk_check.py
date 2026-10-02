#!/usr/bin/env python3
"""**零运动**判决：示教器上那个 `Base` 到底是不是机器人的 base？

为什么要有它
============
2026-10-02 拿 `TCP_1`（`z = 200.2`）用指尖碰台面，示教器报 **z = −395.47 mm**。
而记录里**两处**都说台面在 base 系 ≈ **0**：

* 前负责人 ROS1 的 `set_scene()`：桌板顶面在 `base_link` 系 **z ≈ −0.020**；
* 用户 2026-09 口述：**臂直接装在工作台桌面上**。

两者**不可能同时成立** —— 指尖触台面时 `读数_z = 台面高 − δ·r33`
（`δ` = 真值 − 200.2，`r33` = 工具轴与 base z 的夹角余弦），
要让 −395.47 与 `台面高 ≈ −0.02` 相容，就得 `δ ≥ 395 mm`，即「指尖离法兰 595 mm」。
大寰官方尺寸是 184.6/200.2 —— **不可能**。

于是只剩两种可能：

| | 是什么 | 若成立 |
|---|---|---|
| **A** | 台面**真的**在 base 下方 ≈ 0.40 m（臂坐在立柱 / 底座 / 垫板上升高了） | `table_plane_z ≈ −0.395`，真机几何要重写 |
| **B** | 示教器那个 **`Base` 不是机器人 base**（Installation 里设过 base 偏移） | 台面确实 ≈ 0，读数要减偏移 |

**判别法不需要动臂**：关节角可以唯一确定法兰位置（正运动学）。
拿示教器报的位姿反推法兰，与 FK 的结果相减 —— 差 ≈ 0 就是 A，差 ≈ 395 mm 就是 B。

    示教器报的是「法兰 + R·TCP」，所以反推：
    示教器反推的法兰 = 报的 (x,y,z) − R(rx,ry,rz) · TCP   （见 `pendant_flange`）

**这个差与填的 TCP 无关**（推导见 `Sample.d`）：示教器算 TCP 位姿用的就是
「关节角 → 法兰 → 加 R·TCP」，这里又把 R·TCP 原样减掉了，于是**工具的物理真值
根本进不到这条链路里**。所以：

* ✅ 它能判**参考系**（A / B）；
* ❌ 它**判不了**那个 **4 mm** —— 那一项只能靠**碰台面**（`docs/PROBE_LENGTH.md`）。
  （我一度以为这里能顺手把 4 mm 也解出来，推导一遍发现是错的，已改。）

FK 的可信度
===========
正运动学用**两条互相独立的链**各算一遍，必须对得上（`--self-test`）：

1. **URDF 链** —— 直接抄板子上 `ur_description` 的关节原点
   （`/opt/ros/humble/share/ur_description/urdf/ur_macro.xacro` +
   `config/ur5/default_kinematics.yaml`，标定哈希 `calib_209549117540498681`）。
   这是**驱动真正在用的**那一套。
2. **标准 DH 链** —— 通用 UR 的 DH 表（`a/d/alpha` 六个）。公式与 (1) 完全不同
   （`Rot_z·Trans_z·Trans_x·Rot_x` vs `固定原点 + 绕 z 转`）。

两条链给出同一组相对位姿 ⇒ 才敢拿它判决。**姿态按示教器惯例：旋转矢量（度）。**

用法
====
    python3 scripts/ur5_fk_check.py --self-test        # 先跑这个：验 FK 本身
    python3 scripts/ur5_fk_check.py --joints "-12.3, -78.1, 95.4, -107.3, -90.0, 5.2" \\
                                    --pose "-281.30, -134.84, -395.47, 0, 0, 0"
    python3 scripts/ur5_fk_check.py --joints ... --pose ... --tcp "0,0,0"

`--joints` 与 `--pose` 必须是**同一个姿态**下抄的（臂别动）。单位：关节**度**、
位姿 **mm**。`--tcp` 是示教器上**当时激活的那个 TCP**（mm），默认 `0,0,0`。

退出码：0 = 判决出来了；2 = 输入不合法（缺参数 / 维度不对）；3 = 判决不出来
（FK 两链对不上，或差值落在两个假设之间 —— **不猜**）。
"""
import argparse
import sys

import numpy as np

PI = np.pi

# ── 板子上 ur_description 的实际关节原点（base_link → ... → flange）─────────────
# 每项：(xyz, rpy, 是不是可动关节)。可动关节绕**本体系 z 轴**转 theta_i。
# 摘自 ur_macro.xacro:354-463 + default_kinematics.yaml（ur5）。
URDF_CHAIN = [
    ((0.0, 0.0, 0.0),           (0.0, 0.0, PI),                False),  # base_link→base_link_inertia
    ((0.0, 0.0, 0.089159),      (0.0, 0.0, 0.0),               True),   # shoulder_pan
    ((0.0, 0.0, 0.0),           (PI / 2, 0.0, 0.0),            True),   # shoulder_lift
    ((-0.425, 0.0, 0.0),        (0.0, 0.0, 0.0),               True),   # elbow
    ((-0.39225, 0.0, 0.10915),  (0.0, 0.0, 0.0),               True),   # wrist_1
    ((0.0, -0.09465, 0.0),      (PI / 2, 0.0, 0.0),            True),   # wrist_2
    ((0.0, 0.0823, 0.0),        (PI / 2, PI, PI),              True),   # wrist_3
    ((0.0, 0.0, 0.0),           (0.0, -PI / 2, -PI / 2),       False),  # wrist_3→flange
]

def rpy_to_matrix(rpy):
    """URDF 的 rpy → 3×3：R = Rz(yaw)·Ry(pitch)·Rx(roll)。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    return Rz @ Ry @ Rx


def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def urdf_fk(q_rad):
    """URDF 链正运动学 → 4×4（base_link 系）。

    链里既有固定关节也有可动关节，**可动关节按顺序**吃掉 q_rad 的 6 个数
    （不是按链上的下标 —— 链首有个固定关节，按位对齐会整体错一格）。
    """
    M = np.eye(4)
    k = 0
    for xyz, rpy, movable in URDF_CHAIN:
        M = M @ T(rpy_to_matrix(rpy), np.asarray(xyz, float))
        if movable:
            M = M @ T(rotz(q_rad[k]), np.zeros(3))
            k += 1
    if k != len(q_rad):
        raise ValueError(f"链上有 {k} 个可动关节，却给了 {len(q_rad)} 个关节角")
    return M


# ── 独立的第二条链：通用 UR 的标准 DH 表 ─────────────────────────────────────
# (a, d, alpha)。T_i = Rot_z(theta)·Trans_z(d)·Trans_x(a)·Rot_x(alpha)。
UR5_DH = [
    (0.0,       0.089159, PI / 2),
    (-0.425,    0.0,      0.0),
    (-0.39225,  0.0,      0.0),
    (0.0,       0.10915,  PI / 2),
    (0.0,       0.09465, -PI / 2),
    (0.0,       0.0823,   0.0),
]


def dh_fk(q_rad):
    """标准 DH 正运动学 → 4×4。"""
    M = np.eye(4)
    for (a, d, alpha), th in zip(UR5_DH, q_rad):
        ca, sa = np.cos(alpha), np.sin(alpha)
        Tx = np.array([[1, 0, 0, a],
                       [0, ca, -sa, 0],
                       [0, sa, ca, 0],
                       [0, 0, 0, 1]], float)
        M = M @ T(rotz(th), np.array([0.0, 0.0, d])) @ Tx
    return M


def pendant_flange(pose_mm, pose_rv_deg, tcp_mm):
    """示教器读数 → 它**认为**的法兰位姿（4×4）。

    示教器报的是 TCP 位姿。纯 z 偏移的 TCP 不改变姿态 ⇒ `R` 就是法兰的姿态；
    位置往回退 `R·tcp` 就是法兰中心。
    """
    from tcp_check import rotvec_to_matrix
    R = rotvec_to_matrix(pose_rv_deg)
    p = np.asarray(pose_mm, float) / 1000.0 - R @ (np.asarray(tcp_mm, float) / 1000.0)
    return T(R, p)


def _rand_q(rng):
    return rng.uniform(-PI, PI, 6)


def self_test(verbose=True):
    """两条独立的链必须给出**同一个法兰位置**（差一个已知的固定偏航）。

    只比**位置**，不比姿态 —— 两条链的**末端姿态约定本来就不同**：
    DH 那条的末帧落在 tool0 那一类约定上，与 `flange` 差一个随姿态变化的旋转
    （实测差 120°，且随配置变），**但末端原点恒等**。这里要用的是位置
    （台面高度那个判决只看 z 与距离），所以按位置验，并把这条差异说清楚，
    免得后来人以为姿态对不上就是 FK 错了。

    参考系关系：`p_urdf_base_link(q) = Rot_z(π) · p_dh(q)`，
    即 DH 链落在**UR 控制器的 `Base` 系**里（ur_macro.xacro 的注释明说）。
    """
    rng = np.random.default_rng(20261002)
    worst_abs = worst_rel = 0.0
    for _ in range(200):
        qa, qb = _rand_q(rng), _rand_q(rng)
        pa, pb = urdf_fk(qa)[:3, 3], urdf_fk(qb)[:3, 3]
        da, db = dh_fk(qa)[:3, 3], dh_fk(qb)[:3, 3]
        worst_abs = max(worst_abs, float(np.linalg.norm(pa - rotz(PI) @ da)))
        # 相对位置：连参考系都消掉了，只剩纯实现差异
        worst_rel = max(worst_rel, float(np.linalg.norm((pa - pb) - rotz(PI) @ (da - db))))
    ok = worst_abs < 1e-9 and worst_rel < 1e-9
    if verbose:
        print("—— FK 自检：URDF 链 vs 标准 DH 链（200 组随机姿态，比**位置**）——")
        print(f"  绝对位置差（含 Rot_z(π) 参考系换算）= {worst_abs * 1e9:.3e} nm")
        print(f"  相对位置差 = {worst_rel * 1e9:.3e} nm")
        print(f"  ⇒ {'两条独立链一致，FK 可用' if ok else '❌ 两条链对不上，FK 不可信'}")
    return ok


def _nums(s, what):
    try:
        return [float(v) for v in s.replace(" ", "").split(",")]
    except ValueError as exc:
        raise ValueError(f"{what} 解析失败（{exc}）")


class Sample:
    """一次抄表：关节角 + 示教器位姿 + 当时激活的 TCP。

    **位姿可以只给 `x,y,z` 三个数，但前提是 `TCP = (0,0,0)`** ——
    那时示教器报的就是法兰本身，`R·TCP` 那一项恒为零，姿态根本不参与计算。
    给了非零 TCP 却只抄三个数，就是**默认 rx=ry=rz=0**（几乎肯定不对），
    所以这里**直接拒绝**，不猜。
    """

    def __init__(self, q_deg, pose, tcp):
        q = _nums(q_deg, "关节角")
        if len(q) != 6:
            raise ValueError(f"关节角维度不对：{len(q)}，要 6")
        self.q = np.radians(q)

        self.tcp = _nums(tcp, "TCP")
        if len(self.tcp) != 3:
            raise ValueError(f"TCP 维度不对：{len(self.tcp)}，要 3")

        vals = _nums(pose, "位姿")
        if len(vals) == 6:
            self.pose = vals
        elif len(vals) == 3 and not any(self.tcp):
            self.pose = vals + [0.0, 0.0, 0.0]      # TCP=0 时姿态不参与，补零即可
        else:
            raise ValueError(
                f"位姿给了 {len(vals)} 个数 —— 只给 x,y,z 的前提是 TCP = (0,0,0)；"
                f"TCP 非零时必须给全 6 个（x,y,z,rx,ry,rz），"
                f"否则 rx/ry/rz 会被当成 0，算出来的法兰位置是错的")
        self.M_link = urdf_fk(self.q)
        # base_link → base：绕 z 转 π（见 ur_macro.xacro 的注释）
        self.M_base = T(rotz(PI), np.zeros(3)) @ self.M_link
        self.pend = pendant_flange(self.pose[:3], self.pose[3:], self.tcp)

    @property
    def d(self):
        """示教器反推的法兰 − FK 法兰（**同在 base 系**）= 参考系偏移。

        ⚠️ 必须和同一个系比：示教器报的是 UR 控制器的 `Base`，而 `urdf_fk`
        给的是 `base_link`，两者差一个绕 z 的 π。拿 base_link 那条去减，
        等于把「参考系差 180°」当成机器人误差（实测会把 0 报成 1271 mm）。

        ⚠️ 这个量**与填的 TCP 无关**：示教器算 TCP 位姿 = (关节角算出的法兰
        位姿) + R·TCP，`pendant_flange()` 又把 `R·TCP` 原样减掉了。所以
        **工具的物理真值根本进不到这条链路里** —— 它能验的只有「控制器的
        运动学模型 vs `ur_description`」以及「`Base` 有没有被偏移」。
        **那个 4 mm 不在这里，只能靠碰台面**（`docs/PROBE_LENGTH.md`）。
        """
        return self.pend[:3, 3] - self.M_base[:3, 3]

    @property
    def r33(self):
        return float(self.M_link[2, 2])

    def report(self, tag):
        print(f"—— {tag} ——")
        print(f"  TCP 填的是 ({self.tcp[0]:g}, {self.tcp[1]:g}, {self.tcp[2]:g}) mm")
        print(f"  FK 法兰（base_link 系） = "
              f"({self.M_link[0,3]*1000:9.3f}, {self.M_link[1,3]*1000:9.3f}, "
              f"{self.M_link[2,3]*1000:9.3f}) mm")
        print(f"  FK 法兰（base 系）      = "
              f"({self.M_base[0,3]*1000:9.3f}, {self.M_base[1,3]*1000:9.3f}, "
              f"{self.M_base[2,3]*1000:9.3f}) mm")
        print(f"  示教器反推的法兰        = "
              f"({self.pend[0,3]*1000:9.3f}, {self.pend[1,3]*1000:9.3f}, "
              f"{self.pend[2,3]*1000:9.3f}) mm")
        d, n = self.d, float(np.linalg.norm(self.d))
        print(f"  d（base 系）            = ({d[0]*1000:+9.3f}, {d[1]*1000:+9.3f}, "
              f"{d[2]*1000:+9.3f}) mm   |d| = {n*1000:.3f} mm")
        # r33 由**关节角**算出（不是从填的位姿里读的），所以只抄 3 个数时它照样有效；
        # base 与 base_link 只差绕 z 的 π，z 分量不变，两个系里同值。
        print(f"  工具轴 r33（由关节角算）  = {self.r33:+.4f}\n")
        return n


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="零运动判决示教器 Base 是不是机器人 base（可顺带定出那个 4 mm）",
        epilog="⚠️ 负数开头一定要用等号：--pose=\"-281.30,-134.84,-395.47,0,0,0\"")
    ap.add_argument("--joints", help="6 个关节角，度，逗号分隔（UR 顺序）")
    ap.add_argument("--pose", help="示教器读到的 x,y,z,rx,ry,rz（mm / 度，旋转矢量）")
    ap.add_argument("--tcp", default="0,0,0", help="当时激活的 TCP（mm），默认 0,0,0")
    ap.add_argument("--joints2", help="第二组（可选）：**换个姿态**再抄一次，判偏移是不是纯平移")
    ap.add_argument("--pose2", help="第二组的位姿")
    ap.add_argument("--tcp2", default="0,0,0", help="第二组的 TCP")
    ap.add_argument("--self-test", action="store_true", help="只验 FK 本身")
    args = ap.parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 3
    if not args.joints or not args.pose:
        ap.error("要么给 --self-test，要么同时给 --joints 与 --pose")

    try:
        s1 = Sample(args.joints, args.pose, args.tcp)
        s2 = Sample(args.joints2, args.pose2, args.tcp2) if args.joints2 and args.pose2 else None
    except ValueError as exc:
        print(f"❌ {exc}")
        return 2

    if not self_test():
        print("⇒ 判决中止（FK 本身没过自检）")
        return 3

    print("\n—— 结果 ——")
    n1 = s1.report("第 1 组")
    n2 = s2.report("第 2 组") if s2 else None

    print("—— 判决 ——")
    if n2 is None:
        if n1 < 0.010:
            print("  ✅ **A 成立**：参考系一致（|d| < 10 mm）—— 示教器的 `Base` 就是机器人 base。")
            print("     ⇒ 台面**真的**在 base 下方 ≈ 0.40 m —— 臂坐在立柱/垫板上，")
            print("        前负责人那句「臂直接装在台面上」和 `set_scene()` 的 −0.020 都得改。")
            return 0
        if 0.30 < n1 < 0.50:
            print(f"  ✅ **B 成立**：|d| = {n1*1000:.0f} mm ⇒ 示教器的 `Base` **不是**机器人 base。")
            print("     ⇒ 台面确实 ≈ 0，但**所有 `Base` 读数都要先减掉这个 d**。")
            print("        （回 `Installation → Base` 看有几个 base、当前激活的是哪个。）")
            return 0
        print(f"  ⚠️ |d| = {n1*1000:.1f} mm —— 既不是 0 也不是 ~400，**我不猜**。")
        print("     加第二组（**臂动一下**、换个姿态再抄一次）就能分辨：")
        print("       · 两组的 d 一样 ⇒ 偏移是**纯平移**，按上面两个判据重看；")
        print("       · 两组的 d 不一样 ⇒ 参考系还带**旋转**，得回去看 `Installation`。")
        return 3

    # —— 两组都在（第二组本应是**另一个姿态**）——
    dd = s2.d - s1.d
    ndd = float(np.linalg.norm(dd))
    print(f"  两组 d 之差 = {ndd*1000:.3f} mm")
    if np.linalg.norm(s2.q - s1.q) < 1e-9:
        # 同一个姿态抄两遍 ⇒ 差恒为 0，会**假装**通过「纯平移」那一关。
        # 这是典型的「绿得不是理由」—— 必须拦下来，不能顺着往下判。
        print(f"  ❌ 两组的**关节角完全相同** ⇒ 第二组没有提供任何新信息，")
        print(f"     「差 = 0」是抄了同一个姿态的必然结果，**不是**「纯平移」的证据。")
        print("     请换个姿态（关节角至少差 20°）再抄第二组。")
        return 3
    if ndd < 0.005:
        print(f"  ⇒ 两个姿态下 d **一致**（都是 {n1*1000:.1f} mm）⇒ 偏移是**纯平移**：")
        print(f"     它不是 0，也不是 ~400 mm ⇒ 再确认一遍关节角/位姿是同一个姿态抄的；")
        print("     若确实是，那就是**参考系有个几毫米到几十毫米的平移** —— 也要减掉。")
        return 0
    print("  ⇒ 两个姿态下 d **不一致** ⇒ 参考系不只平移，**还带旋转**。")
    print("     回 `Installation` 看 `Base`（可能设过 rpy），或确认两次是同一个 base。")
    return 3


if __name__ == "__main__":
    sys.exit(main())
