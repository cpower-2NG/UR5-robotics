"""
2D→3D 投影与坐标变换节点 (perception_3d)

订阅:
  /detections             (vision_msgs/Detection2DArray)
  /camera/depth/image_raw (sensor_msgs/Image)
  /camera/color/camera_info (sensor_msgs/CameraInfo)

发布:
  /targets_3d (vision_msgs/Detection3DArray) — target_frame 下**全部**目标的 3D 位姿
  /target_3d  (geometry_msgs/PoseStamped)    — 其中**最高分**那一个（向后兼容）

工作流程:
  1. 深度图按 encoding 换算成**米**后存入带时间戳的环形缓冲
  2. 收到检测 → 按时间戳就近配对深度帧（超过 max_sync_dt 则丢弃并告警）
  3. 在检测框内多区域采样，取**最近簇**的中位数作为 Z（抗背景平面干扰）
  4. 针孔投影: P_camera = Z * K⁻¹ * [u, v, 1]ᵀ
  5. TF: camera_frame → target_frame
  6. 发布 target_frame 下的 3D 坐标

为什么有**两个**发布话题（P3-6 的决定）:
  抓取规划器要做三件事——按 Y 排序（从左到右抓，避免机械臂横扫其他物体）、
  跳过已抓过的（按世界坐标去重）、取最高分。**这三件事都需要"全部候选"**，
  而一个 ``PoseStamped`` 只能装一个目标，且不含 id/类别/分数，
  订阅者无法分辨哪条是哪个物体。所以新增 ``/targets_3d`` 携带完整候选集
  （每条含 ``id`` / ``hypothesis.class_id`` / ``hypothesis.score`` / 3D 位姿），
  规划器只消费它、**不再自己做任何投影**。
  ``/target_3d`` 保留为「最高分那个」，以免破坏既有测试与文档契约
  （``scripts/test_pipeline_e2e.py``、``docs/TECHNICAL_SPEC.md``）。

输入约定（**依赖 yolo_detector 冻结的像素约定**，见 P2-5 / D7）:
  ``/detections`` 的 ``bbox`` 必须是**像素**坐标、原点在图像左上角；
  本节点把 ``center.position.x/y`` 直接当作像素 (u, v) 使用，并与
  ``camera_info`` 的 fx/fy/cx/cy 配套。**若上游改成归一化坐标，这里会静默算错**
  ——这正是 D7 记录过的陷阱。``size_x``/``size_y``（像素）用于框内采样；
  若上游未填（≤1），本节点退回单点邻域并在需要时告警。

本文件针对真机（RealSense D435i）修了四处会让链路直接失败的问题，
每条都在下面各自的代码处以「D1/D2/D8/D33」标注了原因，详见
docs/RESTRUCTURE_PLAN.md。
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2DArray,
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
    BoundingBox3D,
)
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs
import numpy as np


# 深度编码 → 「像素值 × 系数 = 米」。
# D1: 原实现把深度值**直接当成米**用，且不判断编码。仿真里深度是 32FC1（米），
#     所以蒙对了；而真机 D435i 的 16UC1 是**毫米**，直接用会放大 1000 倍。
#     这里按编码换算，两种后端都正确。
DEPTH_SCALE_BY_ENCODING = {
    '16UC1': 0.001,   # RealSense D435i 默认深度：毫米
    'mono16': 0.001,
    '32FC1': 1.0,     # Gazebo / ros_gz 仿真深度：米
    '64FC1': 1.0,
}


class Perception3D(Node):
    """2D 检测 → 3D 投影节点"""

    def __init__(self):
        super().__init__('perception_3d')

        # ---- 参数 ----
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('depth_median_window', 5)   # 每个采样点的邻域窗口
        self.declare_parameter('depth_grid', 5)            # 框内采样网格边长（5 → 5×5）
        self.declare_parameter('depth_cluster_tol', 0.03)  # 最近簇容差（米）
        # -1 = 按 msg.encoding 自动判定；显式给值则强制覆盖（单位：米/像素）
        self.declare_parameter('depth_scale', -1.0)
        self.declare_parameter('max_sync_dt', 0.1)         # 检测↔深度 最大时间差（秒）
        self.declare_parameter('depth_buffer_len', 10)     # 深度帧环形缓冲长度
        self.declare_parameter('tf_required', False)       # True = 查不到 TF 就退出

        # ---- 「已知平面」定位模式（默认关闭）----
        # 为什么需要它：俯视相机以约 54° 斜视，深度反投影出的 z 若有偏差，
        # 会**同时**在高度和横向上出错（横向误差 ≈ Δz / tan(54°) ≈ 1.4·Δz）。
        # 实测 D435i 在该距离上：台面反投影出的 z 中位 −13.2 mm（应为 0），
        # 标准差 7.0 mm —— 对 28.5 mm 的方块，这个量级足以让夹爪咬在边上。
        #
        # 方块躺在**已知高度**的台面上，因此它的位置可以由「像素射线 ∩ 平面
        # z = target_plane_z」直接得到，**完全不经过深度**；精度于是回到
        # 标定水平（碰点残差 1.1 mm），而不是深度水平（十几毫米）。
        #
        # ⚠️ 不开这项时行为与原来完全一致（走深度反投影）。
        # 开了之后深度**仍然要用**，但只用来做「那里确实有东西」的校验：
        # 深度反投影出的 z 若与平面高度相差超过 plane_depth_tol，判为无效
        # （例如空台面被检测成方块时，深度会指向台面而不是方块顶面）。
        # 开关与数值分成两个参数：单一参数用 NaN 表示「关闭」在 launch 的
        # 参数求值里不可靠（yaml 规则下 'nan' 是字符串、'.nan' 才是浮点），
        # 而 bool + float 的组合在任何一侧都明确。
        self.declare_parameter('use_target_plane', False)
        self.declare_parameter('target_plane_z', 0.0285)   # = 台面 + 方块高（顶面）
        self.declare_parameter('plane_depth_tol', 0.03)

        # 话题名做成参数（P2-7）。原因很具体：真机上 realsense2_camera 打开
        # align_depth.enable:=true 后，**对齐后的深度发布在
        # /camera/aligned_depth_to_color/image_raw**，而 /camera/depth/image_raw
        # 保持**未对齐**。若把话题写死成后者，真机会静默用未对齐深度——
        # 正是 D2 警告的视差偏差。仿真侧由 ros_gz_bridge 直接发到
        # /camera/depth/image_raw，故默认值保持向后兼容。
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detections_topic', '/detections')

        self.target_frame = self.get_parameter('target_frame').value
        self.median_win = self.get_parameter('depth_median_window').value
        self.depth_grid = max(1, self.get_parameter('depth_grid').value)
        self.cluster_tol = self.get_parameter('depth_cluster_tol').value
        self.depth_scale_param = self.get_parameter('depth_scale').value
        self.max_sync_dt = self.get_parameter('max_sync_dt').value
        self.tf_required = self.get_parameter('tf_required').value
        _use_plane = bool(self.get_parameter('use_target_plane').value)
        self.target_plane_z = (float(self.get_parameter('target_plane_z').value)
                               if _use_plane else None)
        self.plane_depth_tol = float(self.get_parameter('plane_depth_tol').value)

        # ---- 相机模型 ----
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # ---- 深度缓冲：[(stamp_sec, depth_in_metres), ...] ----
        # D2: 原实现只存「最新一帧」深度，与检测之间没有任何时间关联；
        #     改为带时间戳的环形缓冲，按 stamp 就近配对。
        self.depth_buffer = deque(maxlen=self.get_parameter('depth_buffer_len').value)
        self.depth_scale = None          # 首次收到深度图时确定
        self.bridge = CvBridge()
        self._warned_normalized = False  # D7 护栏：归一化坐标只提示一次
        # 因 max_sync_dt 被丢弃的批数。日志本身按 5 s 限流（否则每帧一条刷屏），
        # 但限流会让「偶尔丢一批」和「一直都在丢」看起来一样 —— 现场需要后者这
        # 个数才能判断该不该放宽 max_sync_dt，所以累计值进日志正文。
        self._sync_drops = 0

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_fail_streak = 0          # D8: 连续失败计数

        # ---- 发布者 ----
        # /targets_3d：全部候选（规划器消费这个，P3-6）
        self.targets_pub = self.create_publisher(Detection3DArray, '/targets_3d', 10)
        # /target_3d：最高分那个，向后兼容既有测试与文档契约
        self.target_pub = self.create_publisher(PoseStamped, '/target_3d', 10)

        # ---- 订阅者 ----
        qos_best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, qos_best_effort
        )

        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_best_effort
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.detection_callback, 10
        )

        if self.depth_scale_param > 0.0:
            self.depth_scale = float(self.depth_scale_param)
            self.get_logger().info(
                f'深度尺度被参数强制指定为 {self.depth_scale} m/px'
                f'（自动判定已禁用）'
            )

        self.get_logger().info(
            f'Perception3D node started. target_frame={self.target_frame}, '
            f'max_sync_dt={self.max_sync_dt}s\n'
            f'  订阅: {self.get_parameter("detections_topic").value}, '
            f'{self.get_parameter("depth_topic").value}, '
            f'{self.get_parameter("camera_info_topic").value}'
        )

        # D2: 深度必须与彩色对齐，否则用彩色内参配深度像素会有视差偏差。
        # 真机上「对齐」不是一句提醒就能保证的——realsense2_camera 开启
        # align_depth.enable:=true 后，对齐深度发在**另一个话题**上，所以这里
        # 既提示参数、也把当前实际订阅的话题打出来，便于当场核对。
        depth_topic = self.get_parameter('depth_topic').value
        if 'aligned' not in depth_topic:
            self.get_logger().info(
                f'提示：当前深度话题为 {depth_topic}。真机上请确认它是**与彩色对齐**的'
                f'（realsense2_camera 开 align_depth.enable:=true 时，对齐深度在 '
                f'/camera/aligned_depth_to_color/image_raw；/camera/depth/image_raw '
                f'是**未对齐**的）。用未对齐深度配彩色内参会有视差偏差。'
                f'仿真侧 ros_gz_bridge 直接发到 /camera/depth/image_raw，属正常。'
            )

    # ------------------------------------------------------------------
    # 相机内参
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参，初始化 PinholeCameraModel。"""
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera info received. '
                f'fx={self.camera_model.fx():.1f}, '
                f'fy={self.camera_model.fy():.1f}, '
                f'cx={self.camera_model.cx():.1f}, '
                f'cy={self.camera_model.cy():.1f}, '
                f'frame={msg.header.frame_id}'
            )

    # ------------------------------------------------------------------
    # 深度
    # ------------------------------------------------------------------
    def _resolve_depth_scale(self, encoding: str):
        """按编码确定「像素值 → 米」的系数（D1）。"""
        if self.depth_scale is not None:
            return self.depth_scale

        if encoding in DEPTH_SCALE_BY_ENCODING:
            self.depth_scale = DEPTH_SCALE_BY_ENCODING[encoding]
            self.get_logger().info(
                f'深度编码 {encoding} → 尺度 {self.depth_scale} m/px'
                f'（{"毫米" if self.depth_scale == 0.001 else "米"}）'
            )
        else:
            self.get_logger().error(
                f'未知深度编码 {encoding!r}，无法确定尺度。'
                f'请用 depth_scale 参数显式指定（单位：米/像素）。'
                f'已支持：{sorted(DEPTH_SCALE_BY_ENCODING)}。'
                f'本帧按「米」处理，结果可能相差 1000 倍。'
            )
            self.depth_scale = 1.0
        return self.depth_scale

    def depth_callback(self, msg: Image):
        """把深度图换算成米后连同时间戳一起入缓冲。"""
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            return

        scale = self._resolve_depth_scale(msg.encoding)
        # 统一在入口换算成 float32 的米，下游不必再关心编码。
        depth_m = raw.astype(np.float32) * scale

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_buffer.append((stamp, depth_m))

    def _nearest_depth(self, stamp_sec: float):
        """取与给定时间戳最接近的深度帧（D2）。

        返回 (depth_m, dt)；缓冲为空时返回 (None, None)。
        超过 max_sync_dt 由调用方决定是否丢弃。
        """
        if not self.depth_buffer:
            return None, None
        best = min(self.depth_buffer, key=lambda item: abs(item[0] - stamp_sec))
        return best[1], abs(best[0] - stamp_sec)

    def _depth_at(self, depth, u: float, v: float):
        """取 (u,v) 邻域内的有效深度中位数（米），无有效值返回 None。"""
        h, w = depth.shape[:2]
        half = self.median_win // 2

        r_min, r_max = max(0, int(v) - half), min(h, int(v) + half + 1)
        c_min, c_max = max(0, int(u) - half), min(w, int(u) + half + 1)
        if r_max <= r_min or c_max <= c_min:
            return None

        patch = depth[r_min:r_max, c_min:c_max]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _robust_depth(self, depth, u: float, v: float, size_x: float, size_y: float):
        """在检测框内多区域采样，取**最近簇**的中位数（P2-6）。

        俯视场景里物体顶面一定比它身后的桌面更近，所以「最近的一簇」
        就是物体本身；这样能剔除背景平面点，比单点邻域稳得多。
        """
        samples = []

        if size_x > 1.0 and size_y > 1.0:
            # 在框内 ±60% 半宽内均匀取样，避开框边缘（边缘常混入背景）
            half_x, half_y = size_x / 2.0 * 0.6, size_y / 2.0 * 0.6
            n = self.depth_grid
            for du in np.linspace(-half_x, half_x, n):
                for dv in np.linspace(-half_y, half_y, n):
                    d = self._depth_at(depth, u + du, v + dv)
                    if d is not None:
                        samples.append(d)
        else:
            # 框尺寸缺失（上游没填 size_x/size_y）→ 退回单点邻域
            d = self._depth_at(depth, u, v)
            if d is not None:
                samples.append(d)

        if not samples:
            return None

        vals = np.sort(np.asarray(samples))
        nearest = vals[vals <= vals[0] + self.cluster_tol]
        return float(np.median(nearest))

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_matrix(q):
        """四元数（x,y,z,w）→ 3×3 旋转矩阵。与 scripts/ 下同名函数同一约定。"""
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-12:
            raise ValueError('四元数长度为 0')
        x, y, z, w = x / n, y / n, z / n, w / n
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _ray_plane_pose(self, x, y, z, transform, u, v, stamp):
        """像素射线 ∩ 平面 ``z = target_plane_z``，**位置不经过深度**。

        ``x, y, z`` 是同一像素的深度反投影点（相机光学系），这里只借它表达
        射线方向 ``(x/z, y/z, 1)``；位置由射线与平面的交点给出。

        深度仍参与一处判断：把同一个反投影点变到 target_frame 后取其 z，
        与平面高度比较。相差超过 ``plane_depth_tol`` 时判为无效并返回 None ——
        这一条是用来挡「空台面被检测成方块」的：那时深度指向的是台面，
        高度会明显低于方块顶面所处的平面。

        返回 ``PoseStamped``；平面在相机背后或校验不通过时返回 ``None``。
        """
        tr = transform.transform
        R = self._quat_to_matrix(tr.rotation)
        t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])

        d_cam = np.array([x / z, y / z, 1.0])
        d_base = R @ d_cam
        if abs(float(d_base[2])) < 1e-9:
            return None                      # 射线与平面平行
        s = (self.target_plane_z - float(t[2])) / float(d_base[2])
        if s <= 0.0:
            return None                      # 平面在相机背后
        p = t + s * d_base

        z_from_depth = float((R @ np.array([x, y, z]) + t)[2])
        if abs(z_from_depth - self.target_plane_z) > self.plane_depth_tol:
            self.get_logger().debug(
                f'平面模式：({u:.1f},{v:.1f}) 深度反投影 z={z_from_depth:+.4f} '
                f'与平面 {self.target_plane_z:+.4f} 相差 '
                f'{abs(z_from_depth - self.target_plane_z) * 1000:.1f} mm '
                f'（上限 {self.plane_depth_tol * 1000:.0f} mm）—— 判为无效'
            )
            return None

        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame
        out.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        out.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return out

    def _transform_pose(self, pose_stamped: PoseStamped, transform):
        """PoseStamped 的 TF 变换（D33）。

        D33: 原实现调用 `tf2_geometry_msgs.do_transform_pose(cam_pose, tf)`，
        但该函数在 Humble 里的签名是 `do_transform_pose(pose: Pose, ...)`——
        要的是 **Pose**，传 PoseStamped 会抛
        `AttributeError: 'PoseStamped' object has no attribute 'position'`。
        而外层的 except 只捕获 TF 三类异常，**AttributeError 会逃出回调**，
        于是**每次检测都报错、/target_3d 从未被发布过**。
        这里改用 `do_transform_pose_stamped`（Humble 已提供，专为 PoseStamped）。
        """
        if hasattr(tf2_geometry_msgs, 'do_transform_pose_stamped'):
            return tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
        # 兜底：老版本没有 *_stamped 时，自己拆出 Pose、再装回 header
        out = PoseStamped()
        out.pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
        out.header = pose_stamped.header
        return out

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def detection_callback(self, msg: Detection2DArray):
        """收到检测结果 → 投影到 3D → TF 变换 → 发布。"""
        # 收工途中到达的检测结果直接丢：此时 context 已经无效，往下走必然在
        # `publish` 上抛 `RCLError`（D70 的第二个面，见 `main()` 里那段说明）。
        # 顺带省掉一次无用的投影 + TF 查询。
        if not rclpy.ok():
            return

        if not self.camera_info_received:
            self.get_logger().warn(
                'Camera info not yet received, skipping.', throttle_duration_sec=5.0
            )
            return

        if len(msg.detections) == 0:
            return

        camera_frame = msg.header.frame_id  # e.g. camera_color_optical_frame
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # ---- 配对深度帧（D2）----
        depth_m, dt = self._nearest_depth(stamp_sec)
        if depth_m is None:
            self.get_logger().warn(
                'No depth image received yet, skipping.', throttle_duration_sec=5.0
            )
            return

        if stamp_sec > 0.0 and dt > self.max_sync_dt:
            # 深度与检测差得太远：宁可不发，也不要发出一个基于错误时刻深度的坐标
            self._sync_drops += 1
            self.get_logger().warn(
                f'深度与检测时间差 {dt*1000:.0f} ms 超过 max_sync_dt'
                f'（{self.max_sync_dt*1000:.0f} ms），丢弃本批检测'
                f'（累计 {self._sync_drops} 批）。'
                f'本节点在没有目标时会**静默**，若累计数持续增长请调大 max_sync_dt。',
                throttle_duration_sec=5.0
            )
            return

        # P3-6：本批结果先收集成 Detection3DArray，循环结束后**一次性**发布。
        # 逐条发到同一话题会让订阅者无法分辨物体（没有 id、分数被覆盖）。
        targets_msg = Detection3DArray()
        targets_msg.header.stamp = stamp
        targets_msg.header.frame_id = self.target_frame
        best_score = -1.0
        best_pose = None
        best_class = ''

        for detection in msg.detections:
            # ---- Step 1: 提取检测框 ----
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y

            # D7 运行时护栏：上游若误发**归一化**坐标，投影会静默算错
            # （x/y 全落在 0~1，深度取到图像左上角而非目标）。历史上真有过
            # 一份这样的实现（已删的 yolo_vision），接口一样、语义不同。
            # 这里只提示一次，避免刷屏。
            if (not self._warned_normalized
                    and 0.0 < u < 1.0 and 0.0 < v < 1.0
                    and 0.0 < size_x <= 1.0 and 0.0 < size_y <= 1.0):
                self._warned_normalized = True
                self.get_logger().error(
                    f'检测框看起来是**归一化**坐标（center=({u:.3f},{v:.3f}), '
                    f'size=({size_x:.3f},{size_y:.3f})），而本节点按**像素**处理。'
                    f'上游若确实发了归一化坐标，3D 投影会静默算错（D7）。'
                    f'请确认 /detections 的发布者遵守像素约定。'
                )

            # ---- Step 2: 取稳健深度（最近簇中位数）----
            depth_val = self._robust_depth(depth_m, u, v, size_x, size_y)

            # 注意 `not math.isfinite` 这一条：NaN 比较恒为 False，所以单写
            # `depth_val <= 0.0` 是拦不住 NaN 的，NaN 会一路投影成 NaN 位姿发布
            # 出去。目前 `_robust_depth()` 的采样已用 np.isfinite 过滤，**不会**
            # 返回 NaN（已实测），这条属于发布边界的兜底：真机上下游拿到 NaN
            # 目标位姿，轻则 MoveIt 报错，重则规划出一条 NaN 轨迹。
            if depth_val is None or not math.isfinite(depth_val) or depth_val <= 0.0:
                self.get_logger().debug(
                    f'Invalid depth at ({u:.1f}, {v:.1f}), skipping.'
                )
                continue

            # ---- Step 3: 针孔投影 (u,v,Z) → (X_c, Y_c, Z_c) ----
            # 这一组是**深度反投影点**。两种定位模式都要先算它：
            #   · 深度模式（默认）：它就是最终位置；
            #   · 平面模式：只借它表达射线方向 (x/z, y/z, 1)，位置另由
            #     射线与已知平面求交得到（见 _ray_plane_pose 的说明）。
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            x = (u - self.camera_model.cx()) * depth_val / self.camera_model.fx()
            y = (v - self.camera_model.cy()) * depth_val / self.camera_model.fy()
            z = depth_val

            # ---- Step 4: TF 变换 camera_frame → target_frame ----
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    stamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                if self.target_plane_z is not None:
                    base_pose = self._ray_plane_pose(x, y, z, transform, u, v, stamp)
                    if base_pose is None:
                        # 平面在相机背后，或深度校验没过（例如空台面被判成方块）
                        continue
                else:
                    cam_pose = PoseStamped()
                    cam_pose.header.stamp = stamp
                    cam_pose.header.frame_id = camera_frame
                    cam_pose.pose.position = Point(x=x, y=y, z=z)
                    cam_pose.pose.orientation = Quaternion(
                        x=0.0, y=0.0, z=0.0, w=1.0)
                    base_pose = self._transform_pose(cam_pose, transform)
                base_pose.header.stamp = self.get_clock().now().to_msg()
                base_pose.header.frame_id = self.target_frame

                self.tf_fail_streak = 0

                class_name = ''
                score = 0.0
                if len(detection.results) > 0:
                    class_name = detection.results[0].hypothesis.class_id
                    score = detection.results[0].hypothesis.score

                # ---- Step 5: 收进本批候选集（P3-6）----
                # id 透传上游；上游没填时用序号兜底，保证每条都可区分。
                d3 = Detection3D()
                d3.header = base_pose.header
                d3.id = detection.id if detection.id else f'det{len(targets_msg.detections)}'
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = class_name
                hyp.hypothesis.score = score
                hyp.pose.pose = base_pose.pose
                d3.results.append(hyp)
                bbox3d = BoundingBox3D()
                bbox3d.center = base_pose.pose   # 3D 位置在此；尺寸无意义故留零
                d3.bbox = bbox3d
                targets_msg.detections.append(d3)

                if score > best_score:
                    best_score, best_pose, best_class = score, base_pose, class_name

                self.get_logger().info(
                    f'Target [{class_name}] projected: '
                    f'cam=({x:.3f},{y:.3f},{z:.3f}) → '
                    f'base=({base_pose.pose.position.x:.3f},'
                    f'{base_pose.pose.position.y:.3f},'
                    f'{base_pose.pose.position.z:.3f})'
                )

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                # D8: 原实现只 warn 一句就 continue——真机上 TF 缺失时**静默失效**，
                #     表现为「什么都检测不到」，极难定位。改为 ERROR + 可操作提示，
                #     并用连续失败计数把问题顶到明面上。
                self.tf_fail_streak += 1
                self.get_logger().error(
                    f'TF 查询失败（连续 {self.tf_fail_streak} 次）：{e} | '
                    f'需要 {self.target_frame} → {camera_frame} 的变换。'
                    f'真机请先完成手眼标定并发布静态 TF；仿真下由 URDF/SDF 提供。',
                    throttle_duration_sec=5.0
                )
                if self.tf_required:
                    raise RuntimeError(
                        f'TF {self.target_frame} → {camera_frame} 不可用，'
                        f'tf_required=True 故直接失败'
                    ) from e
                continue

        # ---- Step 6: 发布本批全部候选（P3-6）----
        # 一个都没投影成功时不发空数组：发了会让下游误以为「拍到了但没目标」，
        # 与「深度/TF 出问题导致全部丢弃」无法区分。
        if not targets_msg.detections:
            return

        self.targets_pub.publish(targets_msg)
        if best_pose is not None:
            self.target_pub.publish(best_pose)   # 最高分那个，向后兼容

        self.get_logger().info(
            f'发布 {len(targets_msg.detections)} 个候选到 /targets_3d；'
            f'/target_3d = [{best_class}] score={best_score:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Perception3D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 两个都要接：`ros2 launch` 下的 Ctrl-C 走的是 rclpy 自己的信号处理器，
        # 它关掉 context 之后 `spin()` 抛的是 **`ExternalShutdownException`**，
        # 不是 `KeyboardInterrupt`。只接后者的话它一路逃出 main()。
        pass
    except Exception:
        # D70 的**第二个面**（板端实测抓到的，第一个面修完才露出来）：
        # SIGINT 落在**回调执行到 `publish` 那一行时**，rclpy 会把回调里的异常
        # 从 `spin_once` 重新抛出（`executors.py:736 raise handler.exception()`）：
        #
        #     RCLError: Failed to publish: publisher's context is invalid
        #
        # 它既不是 `KeyboardInterrupt` 也不是 `ExternalShutdownException`，
        # 于是绕过上面那个 except 逃出 main() —— 又是一个「正常收工看起来像崩了」。
        # 判据用 `rclpy.ok()` 而**不是异常类型**：rclpy 没有公开的 `RCLError`
        # 别名（实测 `rclpy.exceptions` 里没有它，只有私有的
        # `rclpy._rclpy_pybind11.RCLError`），而「context 还活着吗」正是我们要
        # 区分的那件事 —— context 活着就照抛，免得把真正的发布失败一起藏起来。
        if rclpy.ok():
            raise
    finally:
        # ⚠️ 收工清理要扛得住**第二次 SIGINT**。`ros2 launch` 收到 SIGINT 后会把
        # SIGINT **转发**给每个子进程（日志里的 `sending signal 'SIGINT' to
        # process[...]`），加上终端/进程组那一次，子进程一共收到两次。实测第二次
        # 落在 `destroy_node()` 里面（rclpy 的 `client.destroy_when_not_in_use`），
        # 抛 `KeyboardInterrupt` 从 finally 逃出去 ⇒ 一段 traceback ＋
        # `process has died ... exit code -2`。2026-09-28 板端：`set -m` ＋ 整组
        # SIGINT 可稳定复现（pty 真 Ctrl-C 那次反倒是干净的 —— 这个面只在
        # 「launch 转发得比子进程退得早」的时候露头）。
        # 收工阶段没有「失败要处理」这回事，信号更不是错误 ⇒ 只吞
        # KeyboardInterrupt，别的异常照旧让它现形。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        # ⚠️ 必须先 `rclpy.ok()`：信号处理器已经调过一次 `rcl_shutdown`，
        # 再无条件调一次会抛
        #     RCLError: failed to shutdown: rcl_shutdown already called
        # 而**在 finally 里抛出的异常会顶掉原本那个**，于是
        # `ros2 launch` 报「process has died ... exit code 1」——功能上无害，
        # 但**每次正常收工都像崩了**（2026-09-28 板端实测：同一份 launch 里
        # 有这道闸的 planner_node 报 `process has finished cleanly`，
        # 没有它的本节点报 `process has died ... exit code 1`）。
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
