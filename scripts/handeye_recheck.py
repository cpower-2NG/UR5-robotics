#!/usr/bin/env python3
"""复核：**存盘的这份手眼标定，现在还算不算数**。

半个月/半年之后回头看「还准不准」，不能靠「文件还在」，也不能靠「时间久了不会变」
—— 要拿**新的**位姿去考它。判据还是那条最强的：

    靶板是钉住/摆住不动的 ⇒ 用存盘的 X 反推「基座→靶板」，**每个位姿都必须落在同一处**。

这条判据不要求知道靶板摆在哪，只要求**复核期间靶板没被挪**。所以靶板随便放，
只要在相机视野里、复核全程不碰它。

⚠️ **只查不存**：不调 `take_sample`，不写服务端样本表，也不用事后清理。

⚠️ **不依赖 easy_handeye2 服务端**（默认 `--source tf`）。原来走服务端的
   `get_current_transforms`，可「开机接回来」的流程里**根本没起服务端**
   （只有 `publish_on.sh` 的静态 TF 发布者），照着流程走会直接「服务不在」。
   现在只用两条 TF：
       `robot_base_frame → robot_effector_frame`（驱动发的 base_link→tool0）
       `tracking_base_frame → tracking_marker_frame`（桥发的 相机→靶板）
   这两个查询**各自在各自那棵树里**，所以**不需要** `handeye_publisher` 把两棵树连起来
   —— 连起来是给下游用的，这里用不上（X 由存盘文件提供，自己乘）。

⚠️ **位姿必须够散**：一个位姿必然「通过」（它自己跟自己比）。所以这里要求每个新位姿
   跟已有位姿差 >min-mm **或** >min-deg，否则不收 —— 否则这个复核是**空的**。

用法（板上）：
    ~/handeye_ws/scripts/run_ros.sh scripts/handeye_recheck.py --poses 5

跑之前需要：驱动在跑（`base_link→tool0`）、腕部相机在发图、桥在发靶板 TF。
`ros2 run tf2_ros tf2_echo base_link tool0` 和
`ros2 run tf2_ros tf2_echo camera_color_optical_frame charuco_board` 都得有输出。
"""
import argparse
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from easy_handeye2_msgs.srv import TakeSample
from easy_handeye2.handeye_calibration import load_calibration

import handeye_auto as ha

NS = "/easy_handeye2/calibration/"


def is_new(sample: dict, accepted: list[dict], min_mm: float, min_deg: float):
    """跟**最近**那个已有位姿差够远就算新 —— 只去重，不逼朝向。

    ⚠️ 这里**故意不用** `handeye_auto.novelty()`：那条还带一个「跟**每一个**已有
    样本都差 >min_deg」的全局闸门。采点时要它是为了逼出真的换朝向；复核时它只会
    让人卡住（半个月前实测卡在 6/20）。复核要的只是「这些位姿不是同一个」，
    否则判据是空的。
    """
    if not accepted:
        return True, (float("inf"), float("inf"))
    tr, Rr = ha.tf_vec(sample["robot"]), ha.tf_rot(sample["robot"])
    best = (float("inf"), float("inf"))
    for a in accepted:
        d = float(np.linalg.norm(tr - a["tr"]) * 1000.0)
        ang = ha.rot_angle_deg(Rr, a["Rr"])
        if d < best[0]:
            best = (d, ang)
    return (best[0] > min_mm) or (best[1] > min_deg), best


class TfProbe(Node):
    """直查 TF 拿「法兰位姿」和「靶板在相机里」。不需要服务端、不需要发布者。"""

    # 桥发 20 Hz、驱动发 125 Hz ⇒ 超过这么久时间戳还没往前走，就是**旧的**。
    # 取桥自己的 `--stale-s` 默认值 0.5：桥一旦判定上一帧好结果过期就不再发，
    # 所以「0.5 s 没新帧」和「桥认为靶板已经不在了」是同一件事。
    MAX_STALE = 0.5

    def __init__(self, base: str, tool: str, cam: str, marker: str) -> None:
        super().__init__("handeye_recheck")
        self.frames = (base, tool, cam, marker)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.call_error = ""
        self._stamp: dict[str, float] = {}
        self._since: dict[str, float] = {}
        self._age: dict[str, float] = {}
        # ⚠️ 必须**持续** spin，不能每拍 `spin_once` 一下。
        #    后者一次只处理**一个**回调，而 /tf 上同时灌着驱动 125 Hz + 桥 ~29 Hz，
        #    队列（深度 100）永远是满的 ⇒ 每次 lookup 拿到的是积压在最前面的那条，
        #    差不多是 **0.65 s 之前**的位姿。2026-10-24 实测：`spin_once` 版本读到的
        #    `base_link→tool0` 时间戳**8 秒纹丝不动**，换成后台 spin 之后每 0.5 s
        #    正常前进 0.5 s。采到的位姿是旧数据，连「停稳 0.7 s」也是拿旧数据判的。
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self) -> None:
        try:
            rclpy.spin(self)
        except Exception:        # noqa: BLE001 —— 收尾时 shutdown 会让 spin 抛异常
            pass

    def _fresh(self, key: str, tf) -> bool:
        """这一帧是不是**新**的。

        ⚠️ TF 缓冲会把最后一帧留着（默认 10 s）。靶板一旦**移出视野**，桥就不再
        发 TF，但 `lookup_transform(..., Time())`（= 最新可用）照样能拿到那一帧
        **旧**值 —— 静默地拿「A 位姿时的靶板」去配「B 位姿时的法兰」，反推的靶板
        当然到处跑。2026-10-24 第一次跑就吃了这个怀疑：5 个位姿散开 65 mm，而
        那天的臂是**纯平移**（转角差全 0.0°），X 即便错也只会整体偏、散不开。

        判据用「时间戳有没有往前走」，不用「stamp 离现在多远」：相机报的是
        HARDWARE_CLOCK 时间域，跟 ROS 时钟不一定同源，直接相减会算出一个假年龄。
        """
        st = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        if self._stamp.get(key) != st:
            self._stamp[key] = st
            self._since[key] = now
            self._age[key] = 0.0
            return True
        self._age[key] = now - self._since.get(key, now)
        return self._age[key] <= self.MAX_STALE

    def current(self) -> dict | None:
        rclpy.spin_once(self, timeout_sec=0.1)
        base, tool, cam, marker = self.frames
        try:
            a = self.buf.lookup_transform(base, tool, Time())
        except Exception as e:
            self.call_error = (f"TF 查不到 {base} → {tool}"
                               f"（驱动没起 / 还没 PLAY？）：{type(e).__name__}")
            return None
        try:
            b = self.buf.lookup_transform(cam, marker, Time())
        except Exception as e:
            self.call_error = (f"TF 查不到 {cam} → {marker}"
                               f"（相机没出图 / 桥没起 / 靶板不在视野？）：{type(e).__name__}")
            return None
        if not self._fresh("robot", a):
            self.call_error = (f"法兰 TF 已经 {self._age['robot']:.1f}s 没更新了"
                               f" —— 驱动断了？这一拍不算")
            return None
        if not self._fresh("marker", b):
            self.call_error = (f"靶板 TF 已经 {self._age['marker']:.1f}s 没更新了"
                               f" —— **靶板多半移出视野了**，这一拍不算")
            return None
        self.call_error = ""
        return {"robot": a.transform, "tracking": b.transform}


class ServiceProbe(Node):
    """老路子：走服务端 `get_current_transforms`（只查不写样本表）。

    ⚠️ 需要**服务端**在跑（`calibrate.launch.py` 起的那个），而开机接回的流程里
    没有这一步。留它只是为了让「标定刚做完、服务端还活着」时能顺手复核。
    """

    def __init__(self) -> None:
        super().__init__("handeye_recheck_srv")
        self.cli = self.create_client(TakeSample, NS + "get_current_transforms")
        self.call_error = ""

    def current(self) -> dict | None:
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.call_error = "get_current_transforms 服务不在（服务端没起）"
            return None
        fut = self.cli.call_async(TakeSample.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.call_error = "服务 5 秒没回"
            return None
        res = fut.result()
        if res is None or len(res.samples.samples) == 0:
            self.call_error = "服务端说这一拍没有数据（靶板不在画面里 / TF 查不到）"
            return None
        self.call_error = ""
        s = res.samples.samples[0]
        return {"robot": s.robot, "tracking": s.tracking}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="wrist_eih", help="存盘的标定名")
    ap.add_argument("--poses", type=int, default=5, help="复核用几个位姿（≥3 才有意义）")
    ap.add_argument("--min-mm", type=float, default=25.0)
    ap.add_argument("--min-deg", type=float, default=15.0)
    ap.add_argument("--still", type=float, default=0.7, help="每个位姿停稳这么多秒再取")
    ap.add_argument("--gate-mm", type=float, default=5.0, help="靶板落点散开的闸门")
    ap.add_argument("--max-minutes", type=float, default=15.0)
    ap.add_argument("--source", choices=("tf", "service"), default="tf",
                    help="位姿从哪拿：tf=直查 TF（默认，只要驱动+桥）；"
                         "service=easy_handeye2 服务端（要服务端在跑）")
    ap.add_argument("--robot-base-frame", default="")
    ap.add_argument("--robot-tool-frame", default="")
    ap.add_argument("--tracking-base-frame", default="")
    ap.add_argument("--tracking-marker-frame", default="")
    args = ap.parse_args()

    cal = load_calibration(args.name)
    p = cal.parameters
    t0 = cal.transform.translation
    q0 = cal.transform.rotation
    # tf_vec 要的是**整条 Transform**（它自己去取 .translation），不是那半个 Vector3。
    # 2026-10-24 第一次上真机就栽在这：传了 t0，`Vector3 has no attribute 'translation'`。
    X = ha.T4(ha.tf_vec(cal.transform), ha.tf_rot(cal.transform))
    print(f"存盘的标定：{args.name}（{p.calibration_type}）")
    print(f"  t = [{t0.x*1000:+.2f}, {t0.y*1000:+.2f}, {t0.z*1000:+.2f}] mm，"
          f"|t| = {np.linalg.norm([t0.x, t0.y, t0.z])*1000:.2f} mm")
    print(f"  q = [{q0.x:+.6f}, {q0.y:+.6f}, {q0.z:+.6f}, {q0.w:+.6f}]\n")

    if args.poses < 3:
        print("❌ 少于 3 个位姿，这个复核证明不了什么")
        return 2

    rclpy.init()
    if args.source == "tf":
        # 帧名从标定文件里取 —— 文件里存的就是标定时用的那四个，别在这儿另抄一份。
        # ⚠️ 用 getattr 而不是直接 `p.robot_base_frame`：这几个字段名是新版本
        #    easy_handeye2 的叫法，老版本可能叫 `robot_base_link` / `tracking_marker`
        #    之类。字段名对不上时**要报出来**，不能拿空串去查 TF（那会报一句
        #    莫名其妙的「框名无效」，看着像硬件坏了）。
        base = args.robot_base_frame or getattr(p, "robot_base_frame", "")
        tool = args.robot_tool_frame or getattr(p, "robot_effector_frame", "")
        cam = args.tracking_base_frame or getattr(p, "tracking_base_frame", "")
        marker = args.tracking_marker_frame or getattr(p, "tracking_marker_frame", "")
        miss = [n for n, v in (("robot_base_frame", base), ("robot_effector_frame", tool),
                               ("tracking_base_frame", cam),
                               ("tracking_marker_frame", marker)) if not v]
        if miss:
            print(f"❌ 标定文件里取不到帧名：{miss}")
            print("   ⇒ 手动补：--robot-base-frame / --robot-tool-frame /"
                  " --tracking-base-frame / --tracking-marker-frame")
            rclpy.shutdown()
            return 5
        node = TfProbe(base, tool, cam, marker)
        print(f"位姿来源：直查 TF  [{base} → {tool}] 与 [{cam} → {marker}]")
    else:
        node = ServiceProbe()
        print("位姿来源：easy_handeye2 服务端 get_current_transforms")
    accepted: list[dict] = []
    last_pose = None
    still_since = time.time()
    t_start = time.time()
    last_status = 0.0

    print(f"请把靶板放进相机视野，然后**挪臂到 {args.poses} 个明显不同的位姿**，")
    print("每个位姿停 1~2 秒。全程**不要碰靶板**。\n")

    while rclpy.ok() and len(accepted) < args.poses:
        if args.max_minutes and time.time() - t_start > args.max_minutes * 60:
            print(f"\n⏰ 到 {args.max_minutes} 分钟了，收摊。")
            break
        cur = node.current()
        if cur is None:
            if time.time() - last_status > 5:
                last_status = time.time()
                print(f"    …拿不到位姿：{node.call_error}", flush=True)
            time.sleep(0.5)
            continue

        rv = ha.tf_vec(cur["robot"])
        if last_pose is None or float(np.linalg.norm(rv - last_pose)) > 0.0005:
            last_pose = rv
            still_since = time.time()
        still = time.time() - still_since

        ok, (d, a) = is_new(cur, accepted, args.min_mm, args.min_deg)
        if ok and still < args.still:
            if time.time() - last_status > 2:
                last_status = time.time()
                print(f"    …位姿够新了，等停稳（{still:.1f}s / {args.still:.1f}s）"
                      f"｜已经 {len(accepted)}/{args.poses}", flush=True)
        elif ok:
            tr = ha.tf_vec(cur["robot"])
            Rr = ha.tf_rot(cur["robot"])
            t_board = ha.tf_vec(cur["tracking"])
            Rb = ha.tf_rot(cur["tracking"])
            accepted.append({"tr": tr, "Rr": Rr, "t_board": t_board, "Rb": Rb,
                             # 顺便算一份「基座→靶板」，收尾时逐条打出来 ——
                             # 只看最大散度看不出是**哪一拍**跑掉的（板被碰了一下
                             # 通常会跳一次，随距离漂则是系统误差）。
                             "P": ha.T4(tr, Rr) @ X @ ha.T4(t_board, Rb)})
            print(f"  ✅ 第 {len(accepted)}/{args.poses} 个位姿："
                  f"法兰 {np.round(ha.tf_vec(cur['robot'])*1000, 0)} mm，"
                  f"距最近的已有位姿 {d:.0f} mm / {a:.1f}°，停稳 {still:.1f}s", flush=True)
        else:
            if time.time() - last_status > 4:
                last_status = time.time()
                print(f"    …没收：跟最近的已有位姿几乎重合（{d:.0f} mm / {a:.1f}°）"
                      f" —— 请换个**明显不同**的位姿"
                      f"｜已经 {len(accepted)}/{args.poses}", flush=True)
        time.sleep(0.5)

    if len(accepted) < 3:
        print(f"\n❌ 只拿到 {len(accepted)} 个位姿，没法复核")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    print(f"\n用**存盘的 X** 反推「基座→靶板」，看 {len(accepted)} 个位姿落点散不散：")
    dmm, adeg = ha.board_fixed_spread(accepted, X)

    # 逐拍打出来。最大散度只说明「散开了」，这一列才说明**是哪一拍**、往哪偏：
    # 板被碰一下 ⇒ 某一拍之后整体跳一次；感知随距离漂 ⇒ 单调趋势。
    ts = np.array([a["P"][:3, 3] for a in accepted])
    mean = ts.mean(axis=0)
    print(f"  {'#':>2}  {'法兰 (mm)':<22}{'反推靶板 (mm)':<26}{'距均值 mm':>9}")
    for i, (a, t) in enumerate(zip(accepted, ts), 1):
        f = np.round(a["tr"] * 1000, 0)
        b = np.round(t * 1000, 1)
        print(f"  {i:>2}  [{f[0]:>6.0f},{f[1]:>6.0f},{f[2]:>6.0f}]  "
              f"[{b[0]:>8.1f},{b[1]:>8.1f},{b[2]:>8.1f}]  "
              f"{np.linalg.norm(t - mean) * 1000:>9.2f}")
    print(f"  散开：{dmm:.2f} mm / {adeg:.2f}°（闸门 {args.gate_mm} mm）")
    ok = dmm <= args.gate_mm
    if ok:
        print("\n✅ 标定仍然成立 —— 相机与法兰的刚性关系没变。")
    else:
        print("\n❌ 不成立 —— 这半个月里相机/支架/法兰很可能被**物理动过**。")
        print("   ⇒ 别用这份外参。重新标定（见 docs/待执行命令.md §1-8）。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())
