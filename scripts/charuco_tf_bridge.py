#!/usr/bin/env python3
"""ChArUco 板 → TF 桥：把「相机 ← 靶板」发布成 TF，喂给 easy_handeye2。

为什么要自己写这一段
--------------------
easy_handeye2 **自己不做检测**，它只是按固定时刻去查两条 TF
（`handeye_sampler.py`，见台账第一百三十四轮 §二）：

    robot    = lookup(robot_base_frame   → robot_effector_frame)   机器人的 base ← 法兰
    tracking = lookup(tracking_base_frame → tracking_marker_frame) 相机   ← 靶板

第二条就是这里要发的。而板上现成的检测器都不合用：

- `aruco_opencv` **认不了 ChArUco**（把 deb 拆开 grep `charuco` 全包零命中，
  它的板模型只有 `markers_x/markers_y/marker_size/separation`，是**矩形码阵**不是棋盘）；
- 官方 `cameracalibrator` 支持 `--pattern charuco`，但它要 GUI，板上 `DISPLAY` 是空的。

所以复用**唯一已经实测过的那套检测**（第一百三十三轮：12/12 码块、16/16 角点、
重投影 RMS 0.20 px），发布 `相机←靶板` 的 TF。

三条要紧的设计
--------------
1. **检不到就不发 TF**（而不是发上一帧的旧值）。
   这样 easy_handeye2 那条查询会**失败**，于是那一拍不会被记成样本 ——
   宁可少采一个点，也不能把脏样本混进去。
2. **TF 盖图像自己的时间戳**（`--stamp-source image`，默认）。
   这条一开始写反过，代价是两轮实测全废 —— 记在这里免得再犯：
   最初盖的是「发布那一刻」的 ROS 时间，理由写的是"采样的那一刻臂是静止的，这点偏移无害"。
   可是**这条假设从来没有被工具强制过**（`--min-still` 是那两轮之后才加的），
   而 easy_handeye2 那边（`handeye_sampler.py`）是拿**同一个时刻**去查两条 TF 的：
   机械臂那条由驱动按真实时刻发出（查出来是准的），靶板这条却盖着发布时间 ——
   于是在 t 时刻查回来的，其实是 t−延迟 那一刻的图像。
   臂一动手，两条就差「腕部角速度 × 延迟」那么多，这正是实测那 12~34 mm 靶板散布的来源。
   盖上图像时间戳，两条才说的是**同一个瞬间**。
   （相机若是另一个时钟域，`_tf_stamp()` 会当场报错并退回发布时间 —— 那是**降级**不是修好。）
3. **内参用我们自己标的那组（`--intrinsics`），不用 camera_info 自报的出厂值。**
   板上现役节点发的 `camera_info` 是出厂值（fx≈908、**D 全是零**），
   而我们标出来的是 **fx=900.30、RMS 0.17 px** 且带完整畸变模型。
   这里不能用"差不多"糊过去：焦距错 1% ⇒ 板心距离算错 1% ⇒
   手眼求解里 `t_x` 会被**系统性地**带偏（相对位姿的平移整体缩了 s 倍，
   `R_a t_x + t_a = R_x t_b + t_x` 里 `t_b` 变了，解出来的 `t_x` 自然跟着变）。

方向约定
--------
`estimatePoseCharucoBoard` 给出的 `(R, t)` 满足 `X_cam = R·X_board + t`，
而 TF 的 `parent→child` 正是「把 child 系的点变到 parent 系」——
所以 **parent = 相机光学系，child = 靶板系**，`(R,t)` 原样用，不取逆。

用法（在板上跑）
----------------
    python3 scripts/charuco_tf_bridge.py \\
        --image-topic /wrist/camera/color/image_raw \\
        --info-topic  /wrist/camera/color/camera_info \\
        --board docs/calib/charuco_A4_30mm_5x5.json \\
        --intrinsics calib/wrist_intrinsics_1010a/wrist_intrinsics.yaml \\
        --board-frame charuco_board

只订阅 + 发 TF，**不发布任何能让机器人动的东西**。
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

_ENCODING_TO_CV = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY, "mono8": None, "8UC1": None}


def _to_gray(msg: Image) -> np.ndarray | None:
    enc = (msg.encoding or "").lower()
    if enc not in _ENCODING_TO_CV:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("mono8", "8uc1"):
        return buf.reshape(msg.height, msg.width).copy()
    if buf.size != msg.height * msg.width * 3:
        return None
    return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), _ENCODING_TO_CV[enc])


def _load_intrinsics(path: str) -> tuple[np.ndarray, np.ndarray, int, int]:
    """读 ROS `camera_calibration` 格式的 YAML（`wrist_intrinsics_solve.py` 的输出）。

    只认这种平面格式，**不靠 yaml 库** —— 板上 PyYAML 有没有无所谓。
    """
    K = None
    D = None
    w = h = 0
    section = None
    buf: list[float] = []
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if not line.startswith(" ") and ":" in line:
                if section == "k" and len(buf) == 9:
                    K = np.array(buf, float).reshape(3, 3)
                elif section == "d" and len(buf) == 5:
                    D = np.array(buf, float).reshape(-1, 1)
                buf = []
                key, _, val = line.partition(":")
                key, val = key.strip(), val.strip()
                if key == "image_width":
                    w = int(val)
                elif key == "image_height":
                    h = int(val)
                elif key == "camera_matrix":
                    section = "k"
                elif key == "distortion_coefficients":
                    section = "d"
                else:
                    section = None
                continue
            if section and line.strip().startswith("-"):
                buf.append(float(line.strip().lstrip("-").strip()))
    if section == "k" and len(buf) == 9:
        K = np.array(buf, float).reshape(3, 3)
    elif section == "d" and len(buf) == 5:
        D = np.array(buf, float).reshape(-1, 1)
    if K is None or D is None:
        raise ValueError(f"{path} 里没读出完整的 camera_matrix / distortion_coefficients")
    return K, D, w, h


def _matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3×3 → (x, y, z, w)。数值稳定的分支写法（不靠 trace>0 一条路）。"""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = np.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


class CharucoBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("charuco_tf_bridge")
        self.args = args
        with open(args.board, encoding="utf-8") as f:
            meta = json.load(f)
        self.dictionary = aruco.Dictionary_get(getattr(aruco, meta["dictionary"]))
        self.board = aruco.CharucoBoard_create(
            int(meta["squares_x"]), int(meta["squares_y"]),
            float(meta["square_mm"]) / 1000.0, float(meta["marker_mm"]) / 1000.0,
            self.dictionary,
        )
        self.K: np.ndarray | None = None
        self.D: np.ndarray | None = None
        self.camera_frame: str | None = None
        if args.intrinsics:
            self.K, self.D, cal_w, cal_h = _load_intrinsics(args.intrinsics)
            self.get_logger().info(
                f"内参用**我们标的**那份（{args.intrinsics}）："
                f"fx={self.K[0, 0]:.2f} fy={self.K[1, 1]:.2f} "
                f"cx={self.K[0, 2]:.2f} cy={self.K[1, 2]:.2f} "
                f"k1={self.D[0, 0]:+.4f} k2={self.D[1, 0]:+.4f}（标定尺寸 {cal_w}×{cal_h}）"
                f"｜**不用** camera_info 里的出厂值"
            )
        self._latest: Image | None = None
        self._processed: Image | None = None
        self.last: dict | None = None
        # 上一次**为什么**没发 TF —— 现场排查全靠它。
        # 只报"没检出"是不够的：板没进画面 / 进了但角点不够 / 角点够但定不准，
        # 对应三种完全不同的处置（挪板 / 挪臂 / 擦镜头或调光）。
        self.why: dict = {"stage": "还没收到帧", "n_marks": 0, "n_corners": 0, "rms": None}
        self.last_ok_wall = 0.0
        self.stamp_warned = False
        self.frames_seen = 0
        self.detected = 0

        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Image, args.image_topic, self._on_image, qos)
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, qos)
        self.create_timer(1.0 / args.publish_hz, self._publish)
        self.get_logger().info(
            f"桥已起：{args.image_topic} → TF {args.parent_frame or '(取 camera_info 自报)'}"
            f" → {args.board_frame}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        if self.camera_frame is not None:
            return
        self.camera_frame = self.args.parent_frame or msg.header.frame_id
        if self.K is None:
            # 没给 --intrinsics 才退回出厂值（**不推荐**，见文件头第 3 条）
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().warn(
                f"⚠️ 没给 --intrinsics，用 camera_info 的**出厂值**："
                f"fx={self.K[0, 0]:.2f}，畸变全 {self.D.ravel()}"
            )
        self.get_logger().info(f"父帧 = {self.camera_frame}（取自 camera_info）")

    def _on_image(self, msg: Image) -> None:
        # 只存不处理：检测要几十毫秒，回调里同步做会把订阅饿死
        # （`wrist_intrinsics_capture.py` 里踩过这个坑，实测 25 s 只收到 8 帧）。
        self.frames_seen += 1
        self._latest = msg

    def process_latest(self) -> None:
        msg = self._latest
        if msg is None or msg is self._processed:
            return
        self._processed = msg
        if self.K is None:
            return
        gray = _to_gray(msg)
        if gray is None:
            return
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)
        n_marks = 0 if ids is None else int(len(ids))
        if ids is None or len(ids) == 0:
            self.last = None
            self._miss("一个码块都没认出来（板不在画面里？）", 0, 0, None)
            return
        n, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
        n_corners = 0 if ch_corners is None else int(n)
        if ch_corners is None or n < self.args.min_corners:
            self.last = None
            self._miss(f"认出 {n_marks} 个码块，但内角点只有 {n_corners}（要 ≥{self.args.min_corners}）"
                       f"——板多半出框/被切了", n_marks, n_corners, None)
            return
        ok, rvec, tvec = aruco.estimatePoseCharucoBoard(ch_corners, ch_ids, self.board, self.K, self.D, None, None)
        if not ok:
            self.last = None
            self._miss(f"角点够（{n_corners}）但 PnP 失败", n_marks, n_corners, None)
            return
        # 重投影 RMS —— **可检验的残差**，用它把"认出来了但定不准"的帧挡掉
        obj = self.board.chessboardCorners[ch_ids.reshape(-1)].reshape(-1, 3)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        err = np.linalg.norm(proj.reshape(-1, 2) - ch_corners.reshape(-1, 2), axis=1)
        rms = float(np.sqrt((err ** 2).mean()))
        if rms > self.args.max_rms:
            self.last = None
            self._miss(f"重投影 RMS {rms:.2f}px 超上限 {self.args.max_rms}（模糊/反光/动了）",
                       n_marks, n_corners, rms)
            return
        R, _ = cv2.Rodrigues(rvec)
        st = msg.header.stamp
        self.last = {
            "n": int(n), "rms": rms, "R": R, "t": np.asarray(tvec, float).reshape(3),
            # 这一帧图像自己的时刻 —— 发 TF 就盖它，别盖「现在」（见 _tf_stamp）
            "stamp": (int(st.sec), int(st.nanosec)),
            # 从「图像拍下」到「检测完」过了多久。就等于 _tf_stamp 里那个延迟，
            # 也等于「盖发布时间」时臂一动手会引入多大的时刻错配。
            "lag_s": time.time() - (st.sec + st.nanosec * 1e-9),
        }
        self.why = {"stage": "OK", "n_marks": n_marks, "n_corners": n_corners, "rms": rms}
        self.last_ok_wall = time.time()
        self.detected += 1

    def _miss(self, reason: str, n_marks: int, n_corners: int, rms: float | None) -> None:
        self.why = {"stage": reason, "n_marks": n_marks, "n_corners": n_corners, "rms": rms}

    def _publish(self) -> None:
        # 检不到，或者上一帧已经过期 ⇒ **什么都不发**。
        # 这是有意的：宁可不采那个样本，也不能让 easy_handeye2 把旧位姿当新的记下来。
        if self.last is None or self.camera_frame is None:
            return
        if time.time() - self.last_ok_wall > self.args.stale_s:
            return
        t = TransformStamped()
        t.header.stamp = self._tf_stamp()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.args.board_frame
        x, y, z = (float(v) for v in self.last["t"])
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = _matrix_to_quat(self.last["R"])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)

    def _tf_stamp(self):
        """这一帧 TF 该盖什么时间戳 —— 默认盖**图像**的。

        为什么不是「现在」：easy_handeye2 采样时拿**同一个时刻**去查两条 TF
        （`handeye_sampler.py`：`now − 200 ms` 查机械臂，同一个时刻查靶板）。
        机械臂那条是驱动按真实时刻发的，查出来是准的；靶板这条要是盖成发布时刻，
        那么在 t 时刻查回来的其实是 t−延迟 那一刻的图像 ——
        臂一动手，两条就差了「腕部角速度 × 延迟」。
        """
        now = self.get_clock().now()
        st = self.last.get("stamp") if self.last else None
        if self.args.stamp_source == "image" and st is not None:
            sec, nsec = st
            lag = now.nanoseconds * 1e-9 - (sec + nsec * 1e-9)
            if -0.05 <= lag <= 2.0:
                m = now.to_msg()
                m.sec, m.nanosec = sec, nsec
                return m
            if not self.stamp_warned:
                self.stamp_warned = True
                self.get_logger().error(
                    f"⚠️ 图像时间戳跟 ROS 时钟差 {lag:+.2f}s，**像是两个时钟域** ⇒ "
                    f"退回用发布时间。这是**降级**不是修好：臂停着没事，一动就又有偏差。"
                )
        return now.to_msg()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image-topic", default="/wrist/camera/color/image_raw")
    ap.add_argument("--info-topic", default="/wrist/camera/color/camera_info")
    ap.add_argument("--board", default="docs/calib/charuco_A4_30mm_5x5.json")
    ap.add_argument("--intrinsics", default="",
                    help="我们标的那份 YAML；**强烈建议给**，见文件头第 3 条")
    ap.add_argument("--board-frame", default="charuco_board", help="靶板系的名字（TF 的 child）")
    ap.add_argument("--parent-frame", default="", help="留空就用 camera_info 自报的 frame_id")
    ap.add_argument("--min-corners", type=int, default=10, help="至少几个 ChArUco 内角点才认（共 16）")
    ap.add_argument("--max-rms", type=float, default=1.0, help="重投影 RMS 上限（px）")
    ap.add_argument("--publish-hz", type=float, default=20.0)
    ap.add_argument("--stale-s", type=float, default=0.5, help="上一帧好结果超过这么久就不发")
    ap.add_argument("--stamp-source", choices=("image", "publish"), default="image",
                    help="TF 盖**图像**的时间戳（默认）还是**发布那一刻**的。"
                         "默认 image —— 见文件头第 2 条：盖发布时间会让臂在动时"
                         "把两个不同瞬间的位姿配成一对")
    ap.add_argument("--status-every", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = CharucoBridge(args)
    last_status = time.time()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.02)
            except (KeyboardInterrupt, ExternalShutdownException):
                break
            except Exception:
                # 收到 SIGTERM 后 rclpy 会先把 context 关掉，再 spin 就抛 RCLError。
                # 那是**收尾**，不是故障 —— 只有 context 还活着时才继续往上抛。
                if not rclpy.ok():
                    break
                raise
            node.process_latest()
            if args.status_every > 0 and time.time() - last_status >= args.status_every:
                last_status = time.time()
                if node.last is None:
                    print(f"    …心跳：收到 {node.frames_seen} 帧，**TF 未发** —— {node.why['stage']}", flush=True)
                else:
                    d = float(np.linalg.norm(node.last["t"]))
                    print(
                        f"    …心跳：{node.last['n']}/16 角点  RMS {node.last['rms']:.2f}px  "
                        f"板心距相机 {d * 1000:.0f}mm｜已发 {node.detected} 次 TF"
                        f"｜图像→检测延迟 {node.last['lag_s'] * 1000:.0f}ms",
                        flush=True,
                    )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f"\n桥停了：共收到 {node.frames_seen} 帧，成功发布 {node.detected} 次", flush=True)
        for step in (node.destroy_node, lambda: rclpy.shutdown() if rclpy.ok() else None):
            try:
                step()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
