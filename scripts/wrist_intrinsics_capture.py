#!/usr/bin/env python3
"""腕部相机内参采集：**人挪臂、脚本自动抓合格帧**。

为什么要有它
------------
板上没有显示（`DISPLAY` 空、没有 X socket），ROS 官方的 `cameracalibrator`
那个 GUI 跑不起来。而相机又装在法兰上、不能拆下来对着靶板转，
所以只能"板不动、用示教器把相机挪过去"。

于是这里做成**全自动收帧**：操作者只管把相机挪到各种角度/距离/画面位置上晃，
脚本自己判断"这一帧够不够好、和已经收的够不够不一样"，够好就存下来。
操作者**不需要看任何反馈**，也就不需要一块屏幕。

收帧的五个门槛（缺一不可）
--------------------------
1. **88 个内角点全中**（`--cols/--rows` 给的是内角点数，不是方格数）。
2. **靶板离画面边缘有 `--margin` 像素以上** —— 畸变在边缘最强，
   但压着边界的角点定位反而不可靠。
3. **板那一块够锐**（`--min-sharpness`）。⚠️ 只能量**板 ROI**，不能量整幅 ——
   实测 780 mm 处整幅 141、板 ROI 700（空台面把整幅值拉下来了）。
   这条防的是**站定后被拍糊**（离焦/太近），那种帧重投影 RMS 照样很好看。
4. **重投影 RMS < `--max-rms` px**（用相机自报的内参解 PnP；这一步只为筛质量，
   不参与标定本身）。
5. **这一帧给数据集添了新东西**：姿态差 > `--min-deg` 度，**或**板心在画面里挪开了
   `--min-center-px` 像素。⚠️ **"只是又近了一点"不算** —— 见下面那段翻车记录。
   平面靶上焦距和距离是同一条简并方向（f 和 d 一起放大，投影不变），
   只有**倾斜**能拆开它们；畸变只能靠**角点铺到画面四角**约束。

> **⚠️ 这条判据第一版写错了，实测翻车（2026-10-10 会话 1010a）**：
> 第一版是"转角差 > 4° **或** 平移差 > 15 mm 就收"，
> 于是"笔直靠近"每帧都差 15 mm ⇒ 26 帧全收下，全是**几乎同姿态的正对图**
> （倾斜角中位 3.8°、>20° 的 0 帧）。后果：焦距只约束到 ±3%；
> 5 参自由拟合解出 k2=+1.25、k3=−5.22（物理上荒谬，是在拟合噪声），
> 而重投影 RMS 照样漂亮到 0.13 px —— **自己评自己**。

另外每个姿态**连续两帧都对上**才收（躲开运动模糊）
—— 所以操作者**每挪到一个姿态要停住一两秒**，一路不停是收不到的。

⚠️ 棋盘格是 8×11 这种**非正方**图案，而 `findChessboardCorners`
**对非正方图案不是旋转不变的**；腕部相机侧装，画面里靶板常常转了约 90°。
所以这里**两种行列都试**（见 [[docs/calib/README.md]] 第五十七轮）。

用法（在板上跑）
----------------
    python3 scripts/wrist_intrinsics_capture.py \
        --cols 8 --rows 11 --square-mm 20 --out <板上工作区>/calib/<会话名>

只订阅，不发布 —— 结构上动不了机器人。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

# 图像编码 → OpenCV 转换码。只处理真机上真会出现的这几种。
_ENCODING_TO_CV = {
    "rgb8": cv2.COLOR_RGB2GRAY,
    "bgr8": cv2.COLOR_BGR2GRAY,
    "mono8": None,
    "8UC1": None,
}


def _to_gray(msg: Image) -> np.ndarray | None:
    enc = (msg.encoding or "").lower()
    if enc not in _ENCODING_TO_CV:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    rows, cols = msg.height, msg.width
    if enc in ("mono8", "8uc1"):
        return buf.reshape(rows, cols).copy()
    if buf.size != rows * cols * 3:
        return None
    img = buf.reshape(rows, cols, 3)
    return cv2.cvtColor(img, _ENCODING_TO_CV[enc])


def _tilt_deg(rvec: np.ndarray) -> float:
    """板法线（板坐标系的 z 轴）和光轴之间的夹角，单位度。

    板正对相机时 ≈ 0°，斜看时变大。**内参标定最需要的就是大角度的那几张**：
    平面靶上焦距和距离简并，只有倾斜能拆开。
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0))))


def _rotation_angle_deg(rvec_a: np.ndarray, rvec_b: np.ndarray) -> float:
    """两个旋转向量之间的夹角（度）。"""
    ra, _ = cv2.Rodrigues(rvec_a)
    rb, _ = cv2.Rodrigues(rvec_b)
    cos = (np.trace(ra.T @ rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class IntrinsicsCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wrist_intrinsics_capture")
        self.args = args
        self.pattern_candidates = [
            (args.cols, args.rows),
            (args.rows, args.cols),
        ]
        self.object_points = self._make_object_points(args.cols, args.rows, args.square_mm)

        self.K: np.ndarray | None = None
        self.D: np.ndarray | None = None
        self.image_size: tuple[int, int] | None = None

        self.accepted: list[dict] = []
        self.prev_board: tuple[np.ndarray, tuple[int, int]] | None = None
        self.last_accept_time = 0.0
        self.frames_seen = 0
        self.frames_detected = 0
        self._latest: Image | None = None
        self._processed: Image | None = None
        self.last_detect_info: dict | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, args.image_topic, self._on_image, qos)
        self.create_subscription(CameraInfo, args.info_topic, self._on_info, qos)

        os.makedirs(args.out, exist_ok=True)
        self.get_logger().info(
            f"采集会话目录：{args.out}\n"
            f"  图案：{args.cols}×{args.rows} 内角点，格 {args.square_mm} mm\n"
            f"  图像：{args.image_topic}"
        )

    @staticmethod
    def _make_object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
        """棋盘格内角点在**板坐标系**里的 3D 位置（米）。"""
        sq = square_mm / 1000.0
        pts = np.zeros((cols * rows, 3), np.float64)
        grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        pts[:, :2] = grid * sq
        return pts

    def _on_info(self, msg: CameraInfo) -> None:
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f"收到相机内参（出厂自报）：fx={self.K[0,0]:.2f} fy={self.K[1,1]:.2f} "
                f"cx={self.K[0,2]:.2f} cy={self.K[1,2]:.2f}"
            )

    def _on_image(self, msg: Image) -> None:
        """**只存不处理。**

        检测在 1280×720 上要 1.3 s（实测），而图像是 30 Hz。
        如果在这里同步做检测，订阅者会被饿死 —— 实测 25 s 只收到 8 帧。
        所以回调只把最新一帧放到槽位里，处理交给主循环，
        处理不过来就**丢帧**（我们要的是"最新的一帧"，不是"每一帧"）。
        """
        self.frames_seen += 1
        self._latest = msg

    def process_latest(self) -> None:
        msg = self._latest
        if msg is None or msg is self._processed:
            return
        self._processed = msg
        self._process(msg)

    def _process(self, msg: Image) -> None:
        gray = _to_gray(msg)
        if gray is None:
            return
        size = (msg.width, msg.height)
        if self.image_size is None:
            self.image_size = size
            self.get_logger().info(f"图像尺寸 {msg.width}×{msg.height}")

        found = self._detect(gray)
        if found is None:
            self.prev_board = None
            self.last_detect_info = None  # 心跳要反映"此刻"，不是"曾经"
            return
        corners, shape, rot90 = found
        self.frames_detected += 1

        xy0 = corners.reshape(-1, 2)
        # **板那一块**的清晰度。⚠️ 不能拿整幅图代替 —— 实测 780 mm 处
        # 整幅 141、板 ROI 700（大厅子里大部分是没纹理的白台面）。
        pad = 25
        by0 = max(0, int(xy0[:, 1].min()) - pad)
        by1 = min(gray.shape[0], int(xy0[:, 1].max()) + pad)
        bx0 = max(0, int(xy0[:, 0].min()) - pad)
        bx1 = min(gray.shape[1], int(xy0[:, 0].max()) + pad)
        board_sharp = float(cv2.Laplacian(gray[by0:by1, bx0:bx1], cv2.CV_64F).var())
        self.last_detect_info = {
            "shape": shape,
            "rot90": rot90,
            "n": len(corners),
            "width": msg.width,
            "sharp": board_sharp,
            "bbox": (float(xy0[:, 0].min()), float(xy0[:, 1].min()), float(xy0[:, 0].max()), float(xy0[:, 1].max())),
            "margin": float(
                min(
                    xy0[:, 0].min(),
                    xy0[:, 1].min(),
                    msg.width - xy0[:, 0].max(),
                    msg.height - xy0[:, 1].max(),
                )
            ),
        }

        # 门槛 1：全部内角点都在
        if len(corners) != shape[0] * shape[1]:
            self.prev_board = None
            return

        # 连续两帧同一图案形状才对上 —— 躲开运动模糊
        if self.prev_board is None or self.prev_board[1] != shape:
            self.prev_board = (corners, shape)
            return
        drift = float(np.linalg.norm(corners.reshape(-1, 2) - self.prev_board[0].reshape(-1, 2), axis=1).mean())
        self.prev_board = (corners, shape)
        if drift > self.args.max_drift_px:
            return

        # 门槛 2：不贴边
        xy = corners.reshape(-1, 2)
        if (
            xy[:, 0].min() < self.args.margin
            or xy[:, 1].min() < self.args.margin
            or xy[:, 0].max() > msg.width - self.args.margin
            or xy[:, 1].max() > msg.height - self.args.margin
        ):
            return

        # 门槛 3：**板那一块**够不够锐（board_sharp 在上面量过了）。
        # 这条防的是"站定后被拍糊"（离焦、太近超出对焦范围）：
        # 运动模糊已经由上面的连续两帧漂移拦掉了，但**静着拍糊**会一路过关 ——
        # 因为重投影 RMS 是拿出厂内参算的，糊帧的角点照样落在板上。
        if board_sharp < self.args.min_sharpness:
            return

        # 门槛 4：重投影 RMS
        objp = self._make_object_points(*shape, self.args.square_mm)
        ok, rvec, tvec = cv2.solvePnP(objp, xy, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return
        proj, _ = cv2.projectPoints(objp, rvec, tvec, self.K, self.D)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - xy) ** 2, axis=1))))
        if rms > self.args.max_rms:
            return

        # 门槛 5：这一帧得**给数据集添了新东西**。
        #
        # ⚠️ 这条判据的第一版是错的，实测翻了车（2026-10-10 第一趟 1010a）：
        # 当时写的是"转角差 > min_deg **或** 平移差 > min_mm 就收"，
        # 于是"笔直靠近"每一帧都差 15 mm ⇒ 26 帧全被收下，
        # 结果是 26 张**几乎同姿态**的正对图（倾斜角中位 3.8°、超过 20° 的 0 帧），
        # 焦距只约束到 ±3%、5 参畸变拟合出 k2=+1.25 k3=−5.22 这种荒谬值。
        #
        # 改对的样子：**要么姿态真的不同，要么板落到了画面里没落过的地方。**
        # "只是又近了一点"不算 —— 平面靶上"焦距"和"距离"本来就是同一条
        # 简并方向（f 和 d 一起放大，投影不变），只有**倾斜**能把它们分开；
        # 而畸变只能靠**角点铺到画面各个角落**来约束。
        cx_now = float((xy[:, 0].max() + xy[:, 0].min()) / 2.0)
        cy_now = float((xy[:, 1].max() + xy[:, 1].min()) / 2.0)
        for s in self.accepted:
            deg = _rotation_angle_deg(rvec, np.array(s["rvec"]))
            sc = s.get("board_center_px")
            center_px = (
                float(np.hypot(cx_now - sc[0], cy_now - sc[1])) if sc else float("inf")
            )
            if deg < self.args.min_deg and center_px < self.args.min_center_px:
                return
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.last_accept_time < self.args.cooldown:
            return

        self._save(msg, gray, xy, shape, rot90, rvec, tvec, rms, board_sharp)
        self.last_accept_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _detect(self, gray: np.ndarray):
        """两种行列都试一遍。返回 (corners, (cols, rows), 是否转了 90°)。

        **先降采样找、再全分辨率精修角点。** 实测（Orin NX，1280×720）：
        `findChessboardCorners` 全分辨率 **1343 ms**、半分辨率 **204 ms**，
        而 `cornerSubPix` 全分辨率只要 **14 ms**。
        20 mm 的格在 0.4 m 处全分辨率约 45 px、半分辨率 22 px —— 找得到，够用。
        角点精度由**全分辨率**的 `cornerSubPix` 保证，所以不损失标定精度。
        """
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        scale = self.args.detect_scale
        small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1.0 else gray
        for idx, shape in enumerate(self.pattern_candidates):
            ok, corners = cv2.findChessboardCorners(small, shape, flags=flags)
            if not ok:
                continue
            corners = corners / scale
            corners = cv2.cornerSubPix(
                gray,
                corners.astype(np.float32),
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            return corners, shape, idx == 1
        return None

    def _save(self, msg, gray, xy, shape, rot90, rvec, tvec, rms, board_sharp) -> None:
        idx = len(self.accepted)
        stem = f"view_{idx:03d}"
        img_path = os.path.join(self.args.out, stem + ".png")
        cv2.imwrite(img_path, gray)

        rec = {
            "index": idx,
            "file": stem + ".png",
            "pattern_cols": int(shape[0]),
            "pattern_rows": int(shape[1]),
            "rot90": bool(rot90),
            "image_width": int(msg.width),
            "image_height": int(msg.height),
            "corners": xy.reshape(-1, 2).tolist(),
            "rvec": np.asarray(rvec).ravel().tolist(),
            "tvec": np.asarray(tvec).ravel().tolist(),
            "distance_mm": float(np.linalg.norm(tvec) * 1000.0),
            "reprojection_rms_px": rms,
            # 板法线相对光轴歪了几度 —— **这是内参标定里最要紧的一个数**。
            # 平面靶上焦距和距离是同一条简并方向，只有倾斜能把它们分开。
            "tilt_deg": _tilt_deg(rvec),
            "board_laplacian_var": board_sharp,
            "fullframe_laplacian_var": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            "board_width_px": float(xy[:, 0].max() - xy[:, 0].min()),
            "board_height_px": float(xy[:, 1].max() - xy[:, 1].min()),
            "board_center_px": [
                float((xy[:, 0].max() + xy[:, 0].min()) / 2.0),
                float((xy[:, 1].max() + xy[:, 1].min()) / 2.0),
            ],
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
        }
        with open(os.path.join(self.args.out, stem + ".json"), "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=False, indent=1)
        self.accepted.append(rec)
        print(
            f"[{idx:03d}] 收了  RMS {rms:.2f}px  距离 {rec['distance_mm']:.0f}mm  "
            f"锐 {board_sharp:.0f}  板 {rec['board_width_px']:.0f}×{rec['board_height_px']:.0f}px"
            f"（占宽 {100 * rec['board_width_px'] / msg.width:.0f}%）"
            f"  歪 {rec['tilt_deg']:.0f}°"
            f"  板心 ({rec['board_center_px'][0]:.0f},{rec['board_center_px'][1]:.0f})",
            flush=True,
        )

    def report(self) -> None:
        """收工时自己判一次"这批数据够不够格定内参"。
        操作者看不到屏幕，所以结论得留在日志里等我看。"""
        if not self.accepted:
            print("\n❌ 一帧都没收到。", flush=True)
            return
        tilts = np.array([v["tilt_deg"] for v in self.accepted])
        cx = [v["board_center_px"][0] for v in self.accepted]
        cy = [v["board_center_px"][1] for v in self.accepted]
        pts = np.vstack([np.array(v["corners"]).reshape(-1, 2) for v in self.accepted])
        w = self.image_size[0] if self.image_size else 1280
        h = self.image_size[1] if self.image_size else 720
        cov_x = (pts[:, 0].max() - pts[:, 0].min()) / w
        cov_y = (pts[:, 1].max() - pts[:, 1].min()) / h
        n_tilt = int((tilts > 20).sum())
        print("\n=== 本趟数据够不够定内参 ===", flush=True)
        print(f"  帧数 {len(self.accepted)}", flush=True)
        print(f"  倾斜角：最小 {tilts.min():.1f}°  中位 {np.median(tilts):.1f}°  最大 {tilts.max():.1f}°", flush=True)
        print(f"  倾斜 >20° 的帧：{n_tilt} 帧  {'✅' if n_tilt >= 6 else '❌ 太少 —— 焦距定不住'}", flush=True)
        print(f"  角点覆盖：横 {100 * cov_x:.0f}%  纵 {100 * cov_y:.0f}%"
              f"  {'✅' if cov_x > 0.7 and cov_y > 0.7 else '❌ 铺得不够开 —— 畸变定不住'}", flush=True)
        print(f"  板心横跨 x {min(cx):.0f}–{max(cx):.0f}（画面宽 {w}），y {min(cy):.0f}–{max(cy):.0f}（高 {h}）", flush=True)

    def write_summary(self) -> None:
        params = {
            "image_topic": self.args.image_topic,
            "cols": self.args.cols,
            "rows": self.args.rows,
            "square_mm": self.args.square_mm,
            "image_width": self.image_size[0] if self.image_size else None,
            "image_height": self.image_size[1] if self.image_size else None,
            "factory_K": self.K.tolist() if self.K is not None else None,
            "factory_D": self.D.tolist() if self.D is not None else None,
            "frames_seen": self.frames_seen,
            "frames_detected": self.frames_detected,
            "num_views": len(self.accepted),
            "finished_at": datetime.now().isoformat(timespec="seconds"),
        }
        with open(os.path.join(self.args.out, "session.json"), "w", encoding="utf-8") as f:
            json.dump(params, f, ensure_ascii=False, indent=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="会话目录（不存在会创建）")
    ap.add_argument("--cols", type=int, default=8, help="内角点列数（A4 20mm 9x12 靶 = 8）")
    ap.add_argument("--rows", type=int, default=11, help="内角点行数（A4 20mm 9x12 靶 = 11）")
    ap.add_argument("--square-mm", type=float, default=20.0, help="方格边长 mm")
    ap.add_argument("--image-topic", default="/wrist/camera/color/image_raw")
    ap.add_argument("--info-topic", default="/wrist/camera/color/camera_info")
    ap.add_argument("--margin", type=int, default=12, help="角点离画面边缘至少多少像素")
    ap.add_argument("--max-rms", type=float, default=1.0, help="重投影 RMS 上限（px）")
    ap.add_argument("--max-drift-px", type=float, default=1.5, help="相邻两帧角点平均位移上限")
    ap.add_argument(
        "--min-sharpness",
        type=float,
        default=120.0,
        help="**板那一块 ROI** 的 Laplacian 方差下限（防离焦；0 = 关掉）",
    )
    ap.add_argument("--max-views", type=int, default=0, help="收满这么多帧就自动收工（0 = 不限）")
    ap.add_argument(
        "--min-deg",
        type=float,
        default=8.0,
        help="姿态差多少度算「不一样」（**倾斜是唯一能把焦距和距离分开的东西**，别调小）",
    )
    ap.add_argument(
        "--min-center-px",
        type=float,
        default=120.0,
        help="板心要在画面里挪开多少像素才算「落到新地方」（畸变靠这个约束）",
    )
    ap.add_argument("--cooldown", type=float, default=0.4, help="两次收帧之间的最小间隔（秒）")
    ap.add_argument(
        "--detect-scale",
        type=float,
        default=0.5,
        help="在多少倍降采样图上找棋盘格（1.0 = 全分辨率，慢 6 倍）",
    )
    ap.add_argument("--status-every", type=float, default=5.0, help="每隔几秒打一行心跳（0 = 不打）")
    args = ap.parse_args()

    rclpy.init()
    node = IntrinsicsCapture(args)
    last_status = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.process_latest()
            # 收满了自己停 —— 操作者这时人站在示教器前面，看不到这块屏幕，
            # 所以不能指望他"看到够了再停"。到数就退出，帧已经落盘。
            if args.max_views and len(node.accepted) >= args.max_views:
                print(f"\n✅ 已经收满 {args.max_views} 帧，自动收工。", flush=True)
                break
            if args.status_every > 0 and time.time() - last_status >= args.status_every:
                last_status = time.time()
                info = node.last_detect_info
                if info is None:
                    if node.frames_seen == 0:
                        # "一帧都没来"和"来了但没认出板"是两回事，别混成一句话。
                        # 一帧都没有，八成是订阅跑错域了 —— 见下面的提示。
                        print(
                            f"    …心跳：**一帧图像都没收到**（{args.image_topic}），"
                            f"已收 0 帧｜ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(未设)')}"
                            f"，foxy 默认为 0 —— 板上是 99，别忘了 export",
                            flush=True,
                        )
                    else:
                        print(
                            f"    …心跳：收到 {node.frames_seen} 帧，但**没检出棋盘格**"
                            f"（板不在画面里 / 太远 / 太糊），已收 {len(node.accepted)} 帧",
                            flush=True,
                        )
                else:
                    c, r = info["shape"]
                    bw, bh = info["bbox"][2] - info["bbox"][0], info["bbox"][3] - info["bbox"][1]
                    print(
                        f"    …心跳：检出 {info['n']} 点（{c}×{r}"
                        f"{' 转90°' if info['rot90'] else ''}）"
                        f" 板 {bw:.0f}×{bh:.0f}px（占宽 {100 * bw / info['width']:.0f}%）"
                        f" 边距 {info['margin']:.0f}px 锐 {info['sharp']:.0f}"
                        f"｜已收 {len(node.accepted)} 帧",
                        flush=True,
                    )
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException：被 SIGTERM 杀掉时（比如外面的 timeout）rclpy 抛的。
        # 它不是错误，收尾照做就行，别把一串 traceback 甩给操作者看。
        pass
    finally:
        node.report()
        node.write_summary()
        print(f"\n本次共收 {len(node.accepted)} 帧 → {args.out}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
