#!/usr/bin/env python3
"""手眼标定**自动采点/求解**：人只管挪臂，这个脚本自己判断并采样本。

它只做三件事（都是调 `easy_handeye2` 已有的服务，不自己算 TF）：

1. 轮询 `/easy_handeye2/calibration/get_current_transforms` —— 拿到**当前**这一拍的
   `base_link←tool0` 和 `相机←靶板`。**任一条查不到就返回空**，这是它的设计，
   也正是我们要的：靶板那一拍不在画面里 ⇒ 这一拍不采。
2. 跟已经采下的样本比：**离得够远**（平移 > `--min-mm` 或 转 > `--min-deg`）才采。
   否则臂没动/动了一点点也会被记成一堆几乎相同的样本 —— 那批样本对求解没有信息，
   却会把方程组的条件数拉坏（[[verify-before-asserting]]：矩阵秩不够时解出的东西看着像真的）。
3. 收满 `--target` 个样本就**停**，然后五种算法各解一遍，报**互相之间的散布**。
   （散布大 ⇒ 样本的信息量不够，该回去再转几个方向，而不是挑一个好看的数交差。）

它**不碰机器人**：不发任何运动命令，只读 TF、只调标定服务。
存盘写在 `~/.ros2/easy_handeye2/`，是板子本地的事。

用法（板上跑，三个终端或后台）：
    # ① 桥（发 相机←靶板）
    python3 scripts/charuco_tf_bridge.py --intrinsics calib/.../wrist_intrinsics.yaml ...
    # ② 服务端（读两条 TF，提供采样/求解服务）
    ros2 run easy_handeye2 handeye_server --ros-args -p name:=wrist_eye_in_hand ...
    # ③ 本脚本
    python3 scripts/handeye_auto.py --target 20
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
try:
    from sensor_msgs.msg import JointState
except ImportError:  # 笔记本上没有 ROS 消息包，但本文件仍要能 import 得动
    JointState = None
# easy_handeye2_msgs 的服务类型在板上才 import 得到；本文件在笔记本上也要能过 pyflakes。
try:
    from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, SetAlgorithm, TakeSample
except ImportError:  # pragma: no cover - 笔记本上没有这个包
    ComputeCalibration = SaveCalibration = SetAlgorithm = TakeSample = None

NS = "/easy_handeye2/calibration"
S_TAKE = f"{NS}/take_sample"
S_CURRENT = f"{NS}/get_current_transforms"
S_LIST = f"{NS}/get_sample_list"
S_COMPUTE = f"{NS}/compute_calibration"
S_SAVE = f"{NS}/save_calibration"
S_ALGO = f"{NS}/set_algorithm"


def quat_to_R(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_vec(t) -> np.ndarray:
    return np.array([t.translation.x, t.translation.y, t.translation.z])


def tf_quat(t):
    return (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)


def tf_rot(t) -> np.ndarray:
    return quat_to_R(*tf_quat(t))


def rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def T4(t, R) -> np.ndarray:
    """拼 4x4 齐次矩阵。"""
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


class Sampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("handeye_auto")
        self.args = args
        self.cli = {
            "take": self.create_client(TakeSample, S_TAKE),
            "current": self.create_client(TakeSample, S_CURRENT),
            "list": self.create_client(TakeSample, S_LIST),
            "compute": self.create_client(ComputeCalibration, S_COMPUTE),
            "save": self.create_client(SaveCalibration, S_SAVE),
            "algo": self.create_client(SetAlgorithm, S_ALGO),
        }
        self.accepted: list[dict] = []
        # 最近一次服务调用的失败原因（空串=成功）。见 `_call` 的注释：
        # 「服务调不通」和「服务端说没数据」以前被塞进同一句话里，
        # 结果把人往错误的方向引。
        self.call_error = ""
        # /joint_states 还活着吗 —— 这是「读数冻住」和「人没挪臂」的**独立**判据。
        # 光看「位姿没变」是分不开这两者的：驱动的 read() 在 RTDE 掉线时
        # 仍然 `return OK`（见 hardware_interface.cpp），屏幕上一模一样。
        self.last_joint_msg = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joint, 10)

    def _on_joint(self, _msg) -> None:
        self.last_joint_msg = time.time()

    def joint_state_age(self):
        """离最近一条 /joint_states 过了几秒；一条都没收到过则返回 None。"""
        if self.last_joint_msg == 0.0:
            return None
        return time.time() - self.last_joint_msg

    def _call(self, key, req, timeout=5.0):
        """调一个服务。超时/服务不在时返回 None，并把原因记进 `self.call_error`。

        ⚠️ **必须用 `spin_until_future_complete`，不能自己 sleep 等 future。**
        原来这里是 `while not fut.done(): time.sleep(0.02)` —— future **永远不会
        done**：响应要靠节点的 executor 去处理，光 sleep 不转节点，回调永不执行。
        症状极具误导性：服务端**收到了**请求也回了，客户端却每次都判超时，
        于是屏幕上是「没拿到完整位姿（多半是靶板不在画面里）」——
        **把人往相机那边引，而真正坏的是这一行**（实测：桥那边 16/16 角点、
        RMS 0.15px 好好的，却每一拍都报靶板不在画面里）。
        """
        cli = self.cli[key]
        if not cli.wait_for_service(timeout_sec=timeout):
            self.call_error = f"服务 {key} 不在"
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            self.call_error = f"服务 {key} 超时（{timeout:.0f}s 没回）"
            return None
        self.call_error = ""
        return fut.result()

    def wait_for_services(self, timeout=180.0) -> bool:
        """等全部服务出现。

        ⚠️ 服务端是**两条 TF 都能查到之后才创建服务**的（`setup_services_and_topics`
        每 2 s 重试）。所以「服务一直没出现」几乎总是意味着**某一条 TF 还没活**，
        而不是这个脚本坏了。
        """
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < timeout:
            missing = [k for k, c in self.cli.items() if not c.service_is_ready()]
            if not missing:
                return True
            if time.time() - last > 10:
                last = time.time()
                print(f"    等 easy_handeye2 服务… 还缺 {missing}"
                      f"（多半是 base_link→tool0 或 相机→靶板 还没活）", flush=True)
            time.sleep(0.3)
        return False

    def current(self) -> dict | None:
        """拿当前位姿。服务调不通、或服务端说「这一拍没数据」都返回 None，
        但**原因分开记在 `self.call_error` 里** —— 两者该看的地方完全不同。"""
        res = self._call("current", TakeSample.Request())
        if res is None:
            return None
        if len(res.samples.samples) == 0:
            self.call_error = "服务端说这一拍没有数据（靶板不在画面里 / TF 查不到）"
            return None
        s = res.samples.samples[0]
        return {"robot": s.robot, "tracking": s.tracking}

    def take(self):
        """收一个样本。成功返回**服务端刚收下的那一个**，失败返回 None。

        ⚠️ 必须用服务端回的这一个，不能用刚才 `current()` 拿的那一拍 ——
        那是**两次独立调用**，臂在动的时候它们根本不是同一瞬间的位姿，
        于是本地存的样本跟服务端真正拿去解算的那个对不上，
        后面的「靶板钉死」检查就查了个寂寞。

        `take_sample` 回的是**全部样本**（服务端 `take_sample_srv_callback`：
        先 `self.sampler.take_sample()`，再 `response.samples = self._retrieve_sample_list()`），
        刚收的在**最后**。
        """
        res = self._call("take", TakeSample.Request())
        if res is None or len(res.samples.samples) == 0:
            return None
        return res.samples.samples[-1]


def novelty(sample: dict, accepted: list[dict], min_mm: float, min_deg: float):
    """返回 (够不够新, (离最近样本的 mm, 离最近样本的 °), 不够新的原因)。

    判据有**两条**，都得过：

    1. **靠近邻**：平移 > min_mm **或** 转动 > min_deg —— 基本去重。
    2. **靠全局**：朝向跟**每一个**已有样本都差 > min_deg —— 逼出「真的换了
       腕部朝向」。

    ⚠️ 为什么必须有第 2 条（2026-10-10 实测栽过）：
    人拖臂是**连续**的，采样 2 Hz，于是相邻两拍天然差 ~25 mm —— 第 1 条靠
    「平移够远」次次放行，收进来的 20 个样本其实落在**一条螺旋线**上：
    相对第 1 个样本的距离 25.6 / 50.8 / 75.8 / … 每步恒等 24.9 mm，
    转角 2.3° / 4.7° / 7.1° / … 也恒等。这是绕**同一根轴**的纯螺旋运动，
    正是手眼标定教科书里的**奇异位形**：旋转轴张不满三维，方程组只有秩 1。
    后果（实测）：解出来的 X 让一块**从头到尾没动过**的靶板在基座系里跑了
    69 mm —— 而五种算法里有三种互相只差 0.1 mm，看着「高度一致」，一起错。

    加上第 2 条之后，螺旋线上每一拍都只离**前一个** 2.3° ⇒ 当场拒收，
    人只能去扭腕部 —— 而那正是方程缺的那一半信息。

    （第 1 条本身也是修的：原先拿**任意一个**旧样本比，只要跟某一个够远就放行，
    于是臂停住不动之后 12 个一模一样的位姿被当成 12 个样本收进来。）
    """
    if not accepted:
        return True, (float("inf"), float("inf")), ""
    tr, Rr = tf_vec(sample["robot"]), tf_rot(sample["robot"])
    best = (float("inf"), float("inf"))
    for a in accepted:
        d = float(np.linalg.norm(tr - a["tr"]) * 1000.0)
        ang = rot_angle_deg(Rr, a["Rr"])
        if d < best[0]:
            best = (d, ang)
    if not ((best[0] > min_mm) or (best[1] > min_deg)):
        return False, best, "位姿跟已有样本几乎重合"
    # 第 2 条：跟**每一个**已有样本都要差够远，不能只跟最近的那个比
    nearest_deg = min(rot_angle_deg(Rr, a["Rr"]) for a in accepted)
    if nearest_deg <= min_deg:
        return False, best, (f"朝向没换（离已有样本最近才 {nearest_deg:.1f}°）"
                             f" —— 请**扭腕部**换个朝向，平移不算")
    return True, best, ""


def report_diversity(accepted: list[dict]) -> None:
    """样本的姿态够不够散 —— 手眼标定最怕的就是"看着采了 20 个点、其实只有 2 个方向"。

    判据：把每个样本相对第一个的**旋转轴**拿出来看，轴若都挤在一两条线上，
    方程组的秩就不够，解出来的东西会**看起来收敛**但其实是错的。
    """
    if len(accepted) < 3:
        print("    （样本还太少，谈不了散布）")
        return
    R0 = accepted[0]["Rr"]
    axes = []
    for a in accepted[1:]:
        dR = R0.T @ a["Rr"]
        c = max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))
        ang = math.degrees(math.acos(c))
        if ang < 1e-3:
            continue
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        n = np.linalg.norm(w)
        if n > 1e-9:
            axes.append(w / n)
    if len(axes) < 2:
        print("    ⚠️ 相对旋转只有一个方向 —— 手眼方程组秩不够，再换几个方向转")
        return
    A = np.array(axes)
    sv = np.linalg.svd(A, compute_uv=False)
    print(f"    转轴散布：{len(axes)} 个方向，奇异值 {np.round(sv, 3)}")
    if sv[-1] < 0.15:
        print("    ⚠️ 最小的那个奇异值很小 ⇒ 转轴几乎共面/共线，解会不稳，建议再绕别的轴转几个姿态")
    else:
        print("    ✅ 转轴铺开得够（最小奇异值 %.3f）" % sv[-1])


def board_fixed_spread(accepted: list[dict], X: np.ndarray) -> tuple[float, float]:
    """独立判据：靶板钉在桌上没动过 ⇒ 每个样本反推的「基座→靶板」必须处处相同。

        P_i = T_base_tool^(i) · X · T_cam_board^(i)

    X 若对，所有 P_i 就是**同一个**位姿；X 若错，一块压根没动的靶板会被算得到处跑。
    返回 (平移最大差 mm, 转角最大差 °)。

    ⚠️ 这一条跟「五种算法散不散」是**互相独立**的信息源，而且更强：
    2026-10-10 实测，Tsai-Lenz / Park / Horaud 互相只差 0.1 mm（看着高度一致），
    可同一个没动过的靶板被算得到处跑 69 mm —— 三种算法**一起错得一样**。
    「算法一致」只说这几种算法的解法彼此接近，说不出解本身对不对；
    这一条是拿物理约束（板没动）去卡，卡得住才算数。
    """
    Ps = []
    for a in accepted:
        Ps.append(T4(a["tr"], a["Rr"]) @ X @ T4(a["t_board"], a["Rb"]))
    ts = np.array([P[:3, 3] for P in Ps])
    dmax = max(float(np.linalg.norm(p - q)) for p in ts for q in ts) * 1000.0
    amax = max(rot_angle_deg(Ps[i][:3, :3], Ps[j][:3, :3])
               for i in range(len(Ps)) for j in range(len(Ps)))
    return dmax, amax


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", type=int, default=20, help="收多少个样本就停")
    ap.add_argument("--min-mm", type=float, default=25.0, help="跟已有样本平移差超过这么多毫米才算新姿态")
    ap.add_argument("--min-deg", type=float, default=15.0,
                    help="朝向要跟**每一个**已有样本都差过这么多度才算新姿态"
                         "（逼出真的换腕部朝向；不是只跟最近的那个比）")
    ap.add_argument("--max-minutes", type=float, default=25.0, help="超时就收摊（0=不限）")
    ap.add_argument("--poll-hz", type=float, default=2.0)
    ap.add_argument("--min-still", type=float, default=0.7,
                    help="臂要**停稳**这么多秒才许采这一拍。不是讲究：桥给靶板 TF 打的"
                         "是发布时刻、内容是几十毫秒前的画面，臂一动就差着"
                         "「角速度×滞后」（实测让靶板落点散成 2~34 mm）")
    ap.add_argument("--algorithms", default="Tsai-Lenz,Park,Horaud,Andreff,Daniilidis",
                    help="逗号分隔；五种都解一遍互相印证")
    ap.add_argument("--save", action="store_true", help="解完把标定存盘")
    ap.add_argument("--save-algorithm", default="Tsai-Lenz",
                    help="存盘用哪一种（算法名不会写进文件，所以要显式指定并记下来）")
    args = ap.parse_args()

    rclpy.init()
    node = Sampler(args)
    t_start = time.time()
    try:
        print("等 easy_handeye2 服务…", flush=True)
        if not node.wait_for_services():
            print("❌ 服务一直没出现。先确认 ${S_TAKE} 在不在：\n"
                  "   ros2 service list | grep easy_handeye2\n"
                  "   再确认两条 TF 都查得到：\n"
                  "   ros2 run tf2_ros tf2_echo base_link tool0\n"
                  "   ros2 run tf2_ros tf2_echo camera_color_optical_frame charuco_board", flush=True)
            return 2
        print("✅ 服务都在。开始盯姿态 —— **请开始用手挪臂**（每个姿态停 1~2 秒）。\n", flush=True)

        last_status = 0.0
        # 「停稳」判据用的量。**为什么必须停稳**（2026-10-10 实测）：
        #   桥给靶板 TF 打的时间戳是**发布时刻**（`charuco_tf_bridge.py:285`
        #   `t.header.stamp = self.get_clock().now()`），可它看到的是**几十~上百毫秒
        #   前**那一帧画面。臂一停这个偏差就消失；臂在动，机器人位姿和靶板位姿之间
        #   就差着「角速度 × 滞后」。后果实测：一份几何上**明明够散**的样本
        #   （转轴最小奇异值 0.62，五种算法有三种只差 0.08 mm），靶板落点却散成
        #   2~34 mm 一片，解出来的 X 让钉死的靶板跑了 73.6 mm。
        #   原来这里只判「够不够新」，不判「停没停」—— 于是人一拖臂，
        #   采样器就**在运动的半路上**连抢 20 个，人根本没机会停。
        last_pose_vec = None
        pose_unchanged_since = None
        while rclpy.ok() and len(node.accepted) < args.target:
            cur = node.current()
            if cur is None:
                if time.time() - last_status > 5:
                    last_status = time.time()
                    print(f"    …这一拍没拿到完整位姿：{node.call_error or '原因不明'}"
                          "（靶板不在画面里就转回去；若是服务超时，那是脚本/服务端的事）",
                          flush=True)
                time.sleep(1.0 / args.poll_hz)
                continue

            cur_vec = tf_vec(cur["robot"])
            if last_pose_vec is None or float(np.linalg.norm(cur_vec - last_pose_vec)) > 0.0005:
                last_pose_vec = cur_vec
                pose_unchanged_since = time.time()
            still_s = time.time() - (pose_unchanged_since or time.time())

            # 「读数冻住」和「人没挪臂」在屏幕上是**同一个样子**（驱动的 read() 在
            # RTDE 掉线时仍然 return OK）。要分家只能用**独立**信号：/joint_states
            # 还在不在发。这个判据跟臂动不动无关。
            jage = node.joint_state_age()
            if jage is not None and jage > 2.0:
                if time.time() - last_status > 4:
                    last_status = time.time()
                    print(f"    ⚠️ /joint_states 已经 {jage:.1f}s 没更新 —— **读数可能冻住了**，"
                          f"不是你没动。查：ros2 topic hz /joint_states", flush=True)
                time.sleep(1.0 / args.poll_hz)
                continue

            ok, (d_mm, ang), why = novelty(cur, node.accepted, args.min_mm, args.min_deg)
            if ok and still_s < args.min_still:
                if time.time() - last_status > 2:
                    last_status = time.time()
                    print(f"    …位姿够新了，**等你停稳**（已停 {still_s:.1f}s / 要 "
                          f"{args.min_still:.1f}s）—— 已经 {len(node.accepted)}/{args.target} 个",
                          flush=True)
            elif ok:
                s = node.take()
                if s is not None:
                    # ⚠️ 存**服务端收下的那一个**，不是刚才 `cur` 那一拍：
                    #   两次独立调用，臂在动时不是同一瞬间的位姿。
                    node.accepted.append({
                        "tr": tf_vec(s.robot),
                        "Rr": tf_rot(s.robot),
                        "t_board": tf_vec(s.tracking),
                        "Rb": tf_rot(s.tracking),
                    })
                    z = float(np.linalg.norm(tf_vec(s.tracking)) * 1000)
                    print(f"  ✅ 第 {len(node.accepted):2d} 个样本："
                          f"法兰 {np.round(tf_vec(s.robot) * 1000, 0)} mm，"
                          f"板心距相机 {z:.0f} mm，停稳 {still_s:.1f}s，"
                          f"距最近的样本 {d_mm:.0f} mm / {ang:.1f}°", flush=True)
                    if len(node.accepted) % 5 == 0:
                        report_diversity(node.accepted)
                else:
                    print("    （服务端这一拍没取到 —— TF 在 0.2 s 前那个时刻缺了一角，跳过）", flush=True)
            else:
                if time.time() - last_status > 4:
                    last_status = time.time()
                    print(f"    …没收：{why}（离最近样本 {d_mm:.0f} mm / {ang:.1f}°）"
                          f"—— 已经 {len(node.accepted)}/{args.target} 个"
                          f"｜当前法兰 {np.round(cur_vec * 1000, 0)} mm"
                          f"，已停 {still_s:.0f}s", flush=True)
            if args.max_minutes and (time.time() - t_start) > args.max_minutes * 60:
                print(f"\n⏰ 到 {args.max_minutes} 分钟了，收摊。", flush=True)
                break

        print(f"\n收尾：共 {len(node.accepted)} 个样本", flush=True)
        report_diversity(node.accepted)
        if len(node.accepted) < 3:
            print("❌ 样本太少，解不了。", flush=True)
            return 3

        print("\n=== 五种算法各解一遍，看它们互相之间散不散 ===", flush=True)
        results = {}
        for algo in args.algorithms.split(","):
            algo = algo.strip()
            if not algo:
                continue
            # ⚠️ **必须先 set_algorithm**。不切算法的话五次 `compute_calibration`
            #    解的都是服务端当前那一个算法，会印出五个一模一样的数 —— 然后看起来
            #    "五种算法完全一致"，其实只是一种算法说了五遍。
            sa = node._call("algo", SetAlgorithm.Request(new_algorithm=f"OpenCV/{algo}"))
            if sa is None or not sa.success:
                print(f"  {algo:<12} ❌ 切不过去（服务端认不认这个名字？）", flush=True)
                continue
            r = node._call("compute", ComputeCalibration.Request())
            if r is None or not r.valid:
                print(f"  {algo:<12} ❌ 求解失败", flush=True)
                continue
            t = r.calibration.transform
            v = tf_vec(t)
            results[algo] = v
            print(f"  {algo:<12} t = [{v[0] * 1000:+8.2f}, {v[1] * 1000:+8.2f}, {v[2] * 1000:+8.2f}] mm"
                  f"   |t| = {np.linalg.norm(v) * 1000:7.2f} mm", flush=True)

        # ---- 判「这一份到底能不能用」。两条判据都收进 verdict，最后一起说 ----
        verdict: list[str] = []
        V = None
        if len(results) >= 2:
            V = np.array(list(results.values()))
            spread = np.linalg.norm(V - V.mean(axis=0), axis=1) * 1000
            print(f"\n  五种算法之间的散布：最大 {spread.max():.2f} mm"
                  f"（占 |t| 的 {100 * spread.max() / (np.linalg.norm(V.mean(axis=0)) * 1000):.2f}%）")
            if spread.max() > 3.0:
                verdict.append(f"五种算法散布 {spread.max():.1f} mm 偏大 ⇒ 样本信息量不够")
                print("  ⚠️ 散布偏大 ⇒ 样本信息量不够。**别挑好看的挑**，"
                      "回去再绕几个不同方向转几次，重采。")
            else:
                print("  ✅ 算法之间一致。")

        # ⚠️ 这一条比上面那条**强**，见 board_fixed_spread 的注释：
        #    「算法一致」说不了「解对」，几种算法完全可以一起错得一样。
        print("\n=== 独立判据：靶板钉死，反推的「基座→靶板」必须处处相同 ===", flush=True)
        want = args.save_algorithm
        # 顺便把算法切回 save-algorithm：`save_calibration` 存的是**最后一次 compute**
        # 的结果，而算法名**不进存盘文件**（HandeyeCalibration 只有 parameters + transform）
        # —— 跑完五种直接存，落盘的是最后一种（Daniilidis），文件里还看不出来。
        sa = node._call("algo", SetAlgorithm.Request(new_algorithm=f"OpenCV/{want}"))
        r_cal = None
        if sa is None or not sa.success:
            print(f"  ❌ 切不到 {want}", flush=True)
            verdict.append(f"切不到 {want}")
        else:
            r_cal = node._call("compute", ComputeCalibration.Request())
            if r_cal is None or not r_cal.valid:
                print(f"  ❌ {want} 求解失败", flush=True)
                verdict.append(f"{want} 求解失败")
            else:
                tf_cal = r_cal.calibration.transform
                X = T4(tf_vec(tf_cal), tf_rot(tf_cal))
                dmm, adeg = board_fixed_spread(node.accepted, X)
                print(f"  靶板位置的最大不一致：平移 {dmm:.2f} mm，转角 {adeg:.2f}°", flush=True)
                if dmm > 5.0:
                    verdict.append(f"靶板钉死判据不过（同一个没动过的靶板被算动了 {dmm:.1f} mm）")
                    print(f"  ❌ 同一个没动过的靶板，被算出了 {dmm:.1f} mm 的位移 ⇒ 这份 X 是错的。",
                          flush=True)
                    print("     ⇒ 常见病因：样本落在**一条螺旋线**上（人连续拖臂），"
                          "姿态没变化、方程只有秩 1。重采时请**扭着腕部换朝向**。", flush=True)
                else:
                    print("  ✅ 靶板位置处处一致 —— 这份标定经得起物理约束。", flush=True)

        if args.save:
            if verdict:
                # ⚠️ **判不过就不许存。** 原来这里是「警告一句、照存不误」——
                #    于是 `wrist_eih.calib` 里躺着一份脚本自己刚说过「重采」的结果，
                #    而算法名不进文件、下游谁也看不出来，谁读谁中招。
                #    （跟 AG-95「未初始化却 close() 报成功」同一类：静默假成功。）
                print("\n❌ **不存盘** —— 这一份自己就判不过：", flush=True)
                for v in verdict:
                    print(f"     · {v}", flush=True)
                print("   存下去只会让下游静默地用错的东西。回去重采。", flush=True)
                return 6
            r = node._call("save", SaveCalibration.Request())
            ok = r is not None and r.success
            if ok:
                t = tf_vec(r_cal.calibration.transform)
                print(f"\n存盘：✅ {r.filepath.data}")
                print(f"      用的算法 = **{want}**（文件名里看不出来，记在这里）")
                print(f"      t = {np.round(t * 1000, 2)} mm  |t| = {np.linalg.norm(t) * 1000:.2f} mm")
                if V is not None:
                    print(f"      对照：五种算法里最远的两个差 "
                          f"{np.linalg.norm(V.max(axis=0) - V.min(axis=0)) * 1000:.2f} mm")
            else:
                print("\n存盘：❌ 失败")
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n（被打断）", flush=True)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
